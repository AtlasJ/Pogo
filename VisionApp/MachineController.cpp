#include "MachineController.h"
#include <QMetaObject>
#include <QDebug>
#include <QFile>
#include "Logger.h"
#include "MotionController.h"
#include "SystemData.h"
#include "CommonDir.h"
#include "IMotion.h"
#include "Motion_APS.h"
#include "QOSTool.h"

using namespace nvs::motion;

//Defined next to limitRecoveryOnly(); declared here because notifyError() needs it too.
static bool isLimitError(MachineError e);

//How long the Z drive must be continuously ready and holding before the brake comes off.
//Short enough not to be felt in a normal recovery, long enough that a drive still settling
//after an e-stop reset fails the second check instead of passing the first one.
static constexpr int BRAKE_SETTLE_MS = 250;

MachineController& MachineController::instance()
{
    static MachineController inst;  
    return inst;
}

MachineController::MachineController(QObject* parent)
    : QThread(parent)
{
    
    for (int i = 0; i < (int)MachineError::COUNT; i++) {
        m_bypassErrors.insert(i, false);
    }
    //m_bypassErrors[(int)MachineError::ESTOP_PRESSED] = true;

    //E-stop safety relay wiring confirmed on the machine 2026-08-28 (X106), so it is no
    //longer bypassed - a relay fault now raises ESTOP_RELAY_FAULT and applies the Z brake.
    //Curtain relay (X107) is live too: a beam break raises CURTAIN_RELAY_FAULT (error state
    //blinks the reset LED) and the reset button then re-enables all three servo axes.

}


void MachineController::run()
{
    if (!m_enable) return;

    ct::logger::info("[QThread] Machine Controller started");

    //If config\interlock.json is present, ignore the door interlock signal (same effect as debug mode).
    m_bypassInterlock = QFile::exists(Common::Directory::ConfigPath() + "interlock.json");
    if (m_bypassInterlock) {
        ct::logger::info("[MachineController] interlock.json found in config folder, door interlock alarm is ignored.");
    }

    //Start silent. A normal close silences the buzzer itself now, but this still covers a
    //manual toggle left on from the Motion page, and any exit that never reached ~VisionApp -
    //a crash, a kill from Task Manager, or the IPC losing power mid-alarm.
    silenceBuzzer();

    if (m_redTowerTimer == nullptr) {
        m_redTowerTimer = new QTimer();

        QObject::connect(m_redTowerTimer, &QTimer::timeout, [&]() {
            /*
            * Checked here and again after the first sleep. A blink already in flight used to
            * keep writing the tower AFTER setTowerLight() had selected a new colour, because
            * stopRedTowerLight() is queued behind this very lambda and this lambda owns its
            * thread for a full second. The writes it got in were destructive, not merely late:
            * set_DO is a read-modify-write of the whole output group, so its "off" half carried
            * a snapshot taken before the green write and put green back to zero - a tower with
            * NO light on - while its "on" half did the same with amber lit instead of green.
            *
            * Returning early can leave the blink bit high; that is cleared by the colour write
            * that follows, and by the explicit RED reset in stopRedTowerLight().
            */
            if (!m_blinkActive) return;

            int bit = (int)DOA::RED_TOWER_LIGHT;
            if (SystemData::instance()._machineDebugMode) bit = (int)DOA::AMBER_TOWER_LIGHT;
            //error state: blink the reset button LED together with the red tower light
            MotionController::instance().set_DO(m_motionID, 0, bit, true);
            MotionController::instance().set_DO(m_motionID, 0, (int)DOA::RESET_BTN_LED, true);
            os_tool::doNothing(500);

            if (!m_blinkActive) return; //the sleep above is the widest part of the window
            MotionController::instance().set_DO(m_motionID, 0, bit, false);
            MotionController::instance().set_DO(m_motionID, 0, (int)DOA::RESET_BTN_LED, false);
            os_tool::doNothing(500);
        });
    }

    setMachineState(MachineState::NOT_READY);

    m_stateThread = std::thread([this]() {
        poolStates();
    });

    //Startup: single live check, release brake only if servo Z is already on
    safelyReleaseBrake(0);

    exec();
}

void MachineController::release()
{
    if (!m_enable) return;

    m_running = false;

    if (m_stateThread.joinable()) m_stateThread.join();

    /*
    * Final say on the buzzer, and the reason it is silenced twice: SOFTWARE_OFF already did it
    * at the top of ~VisionApp, but the poll loop kept running for the whole teardown after
    * that, so an error raised in that window would have sounded it again with nothing left to
    * turn it off. This write is the one that sticks - the poll thread is joined, and the blink
    * lambda, the only thing still alive on this object, never touches Y106.
    *
    * Same belt-and-braces as the brake, which ~VisionApp also applies twice, and it must stay
    * ahead of MotionController::release() on the next line - once the card is gone the write
    * is silently dropped by valid().
    */
    silenceBuzzer();

    quit();  // Exits the event loop
}

void MachineController::enable(bool enable)
{
    m_enable = enable;
}

MachineState MachineController::getMachineState() {
    //std::lock_guard<std::mutex> lock(m_mutex);
    return m_currentState;
}

bool MachineController::resetAlarm()
{
    if (!m_enable) return true;

    //std::lock_guard<std::mutex> lock(m_mutex);

    if (m_errorStatuses.isEmpty()) {
        ct::logger::info("[MachineController] Reset alarm");
        setMachineState(m_readyState);
        return true;
    }
    else {
        emit signalPromptMsg("Failed to reset alarm, machine is still in error state.");
        ct::logger::error("[MachineController] Failed to reset alarm as machine is still in error state.");
    }

    return false;
}

void MachineController::notifyWarning(MachineWarning w)
{
    if (!m_enable) return;

    //std::lock_guard<std::mutex> lock(m_mutex);

    ct::logger::info("[MachineController] Received warning: %d", (int)w);

    setMachineState(MachineState::WARNING);
    emit signalMachineWarning(w);
}

void MachineController::notifyError(MachineError e)
{
    if (!m_enable) return;

    //std::lock_guard<std::mutex> lock(m_mutex);

    ct::logger::info("[MachineController] Received error: %d", (int) e);

    if (e == MachineError::X_SERVO_OFF) assessError(false, e);
    else if (e == MachineError::Y_SERVO_OFF) assessError(false, e);
    else if (e == MachineError::Z_SERVO_OFF) assessError(false, e);
   
    setMachineState(MachineState::S_ERROR);

    /*
    * Close the motion gate for anything that is not itself a limit hit. Necessary rather than
    * redundant, because BOTH the other places that would do it are bypassed on this path:
    * notifyError() only reaches assessError() for the three servo-off codes, and
    * setMachineState() returns early when the state is already S_ERROR. Without this, a homing
    * timeout or an initialisation timeout arriving on top of a limit hit would leave motion
    * enabled on the strength of limitRecoveryOnly().
    */
    if (!isLimitError(e) && !SystemData::instance()._machineDebugMode) {
        MotionController::instance().enable_motion(false);
    }

    emit signalMachineError(e);
}

void MachineController::notifyEvent(MachineEvent e)
{
    if (!m_enable) return;

    //std::lock_guard<std::mutex> lock(m_mutex);

    ct::logger::info("[MachineController] Received event: %d", (int)e);

    m_currentEvent = e;

    switch (e)
    {
        //State
    case MachineEvent::HOMING:
        setMachineState(MachineState::MACHINE_HOMING);
        break;
    case MachineEvent::HOME_SUCCESS:
        setMachineState(MachineState::READY);
        break;
    case MachineEvent::SOFTWARE_OFF:
        turnOnBrake();
        //The app is closing (first statement of ~VisionApp). Stop the noise now rather than
        //at the end of teardown - that runs for seconds through MIL, the database and the
        //profiler, and there is no reason to keep sounding an alarm through all of it.
        silenceBuzzer();
        break;
    case MachineEvent::X_SERVO_ON:
        assessError(true, MachineError::X_SERVO_OFF);
        break;
    case MachineEvent::Y_SERVO_ON:
        assessError(true, MachineError::Y_SERVO_OFF);
        break;
    case MachineEvent::Z_SERVO_ON:
        assessError(true, MachineError::Z_SERVO_OFF);
        break;
    }
}

void MachineController::poolStates()
{
    while (m_running) {
        if (m_pausePolling) {
            m_pollingParked = true;
            os_tool::goSleep(50);
            continue;
        }
        m_pollingParked = false;

        //machine offline (controller missing or not initialized): polling IO would
        //only spam errors - idle here and pick up again once it comes back
        if (!MotionController::instance().available(m_motionID)) {
            os_tool::goSleep(1000);
            continue;
        }

        handleDIA();
        handleDIB();
        handleDOA();
        handleDOB();
        handleAxisState();
        os_tool::goSleep(10);
    }
}

bool MachineController::pauseStatePolling(bool pause)
{
    m_pausePolling = pause;
    if (!pause) return true;

    //No polling running (e.g. offline start) - nothing to park
    if (!m_stateThread.joinable()) return true;

    //Wait (bounded) for the poll loop to finish its current iteration and
    //park, so no APS call is in flight when the caller proceeds. A stuck
    //iteration must NOT be ignored - the caller has to abort rather than
    //run concurrently with it.
    const auto start = std::chrono::steady_clock::now();
    while (m_running && !m_pollingParked) {
        if (std::chrono::steady_clock::now() - start >= std::chrono::seconds(15)) {
            ct::logger::error("[MachineController] Timed out waiting for state polling to park");
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return true;
}

//The six hard limit hits, treated as a group everywhere recovery is decided. Kept next to
//limitRecoveryOnly() so a new limit code cannot be added to one without the other.
static bool isLimitError(MachineError e)
{
    switch (e) {
    case MachineError::X_POSITIVE_LIMIT_HIT:
    case MachineError::X_NEGATIVE_LIMIT_HIT:
    case MachineError::Y_POSITIVE_LIMIT_HIT:
    case MachineError::Y_NEGATIVE_LIMIT_HIT:
    case MachineError::Z_POSITIVE_LIMIT_HIT:
    case MachineError::Z_NEGATIVE_LIMIT_HIT:
        return true;
    default:
        return false;
    }
}

bool MachineController::limitRecoveryOnly() const
{
    return m_limitErrorCount > 0 && m_nonLimitErrorCount == 0;
}

void MachineController::assessError(bool good, MachineError e)
{
    int s = (int)e;

    //bypass
    if (m_bypassErrors.contains(s)) {
        if (m_bypassErrors[s]) return;
    }

    if (m_errorStatuses.contains(s)) {
        if (good) {
            m_errorStatuses.remove(s);
            if (isLimitError(e)) --m_limitErrorCount; else --m_nonLimitErrorCount;

            /*
            * A limit hit on its own is recoverable, so re-open the gate the moment the error
            * that was holding it shut clears - an e-stop released while an axis is still parked
            * on a switch, say. Without this the operator clears the e-stop and is still unable
            * to jog off the limit.
            */
            if (!SystemData::instance()._machineDebugMode && limitRecoveryOnly()) {
                MotionController::instance().enable_motion(true);
            }
        }
    }
    else {
        if (!good) {
            m_errorStatuses.insert(s);
            if (isLimitError(e)) ++m_limitErrorCount; else ++m_nonLimitErrorCount;

            /*
            * Brake BEFORE setMachineState, not after. S_ERROR calls enable_motion(false), so
            * the old order dropped the servo first and applied the brake afterwards - leaving
            * the Z axis, which carries the head, with neither holding torque nor brake for the
            * gap between them. Milliseconds, but gravity does not wait, and the correct order
            * for anything vertical is always: engage the holding device, then remove power.
            *
            * On a real e-stop the safety relay normally kills the drives in hardware before any
            * of this runs, which is why it has never bitten. That makes this cheap insurance
            * for the paths where software raises the error, not a substitute for the wiring.
            *
            * Retried, because turnOnBrake()'s return used to be discarded: assessError only
            * acts on the TRANSITION into an error, so once ESTOP_PRESSED is in m_errorStatuses
            * every later poll skips this block and a failed DO write would never be sent again.
            * An e-stop is exactly when the motion card might not answer first time.
            */
            if (e == MachineError::ESTOP_PRESSED ||
                e == MachineError::ESTOP_RELAY_FAULT)
            {
                bool braked = false;
                for (int attempt = 1; attempt <= 3 && !braked; ++attempt) {
                    braked = turnOnBrake();
                    if (!braked) {
                        ct::logger::error("[MachineController] Z brake command failed (attempt %d of 3)", attempt);
                        os_tool::goSleep(20);
                    }
                }
                if (!braked) {
                    ct::logger::error("[MachineController] Could NOT apply the Z brake after an e-stop "
                        "- the axis may be unheld. Check DO %d (Y108) and the motion card.",
                        (int)DOA::BRAKE_RELEASE);
                }

                /*
                * An e-stop cuts drive power, so the axes can move while unheld - Z especially,
                * which is why the brake above matters. Whatever position the recipe is working
                * from is no longer trustworthy, so the machine must come back UNINITIALIZED and
                * be re-homed, not restored to the READY it happened to be in beforehand.
                *
                * Forced here rather than left to handleAxisState's "if (!servo_on)" check,
                * because that check reads SVON - and SVON stays SET right through an e-stop, so
                * it never fires. Same command-echo trap as the brake release and the reset
                * trigger. resetAlarm() restores m_readyState, so setting it here is what makes
                * the recovery land on Uninitialized.
                */
                m_readyState = MachineState::NOT_READY;
            }

            setMachineState(MachineState::S_ERROR);

            /*
            * setMachineState() returns early when the state is ALREADY S_ERROR, so the
            * enable_motion(false) inside it never runs for a SECOND error landing on top of a
            * limit hit. Close the gate here for anything that is not itself a limit hit -
            * otherwise a limit followed by a driver alarm would leave motion enabled on the
            * strength of the limit alone.
            */
            if (!isLimitError(e) && !SystemData::instance()._machineDebugMode) {
                MotionController::instance().enable_motion(false);
            }

            emit signalMachineError(e);
        }
    }
}

void MachineController::handleDIA()
{
    auto optional_io = MotionController::instance().get_all_DI(m_motionID, 0);
    if (!optional_io.has_value()) {
        ct::logger::error("[Machine] EMXA DI not responding...");
        return;
    }

    auto io = optional_io.value();
    if (io.size() <= (int)DIA::LAST_INDEX) {
        ct::logger::error("[Machine] EMXA DI invalid index");
        return;
    }

    //Check button triggers - each button lights its LED while held
    auto start_btn = io[(int)DIA::START_BTN];
    if (start_btn && !m_startBtnPressed) {
        m_startBtnPressed = true;
        MotionController::instance().set_DO(m_motionID, 0, (int)DOA::START_BTN_LED, true);
        emit signalMachineEvent(MachineEvent::START_BTN);
    }
    else if (!start_btn && m_startBtnPressed) {
        m_startBtnPressed = false;
        MotionController::instance().set_DO(m_motionID, 0, (int)DOA::START_BTN_LED, false);
    }

    auto stop_btn = !io[(int)DIA::STOP_BTN]; //stop button is NC: pressed = input low
    if (stop_btn && !m_stopBtnPressed) {
        m_stopBtnPressed = true;
        MotionController::instance().set_DO(m_motionID, 0, (int)DOA::STOP_BTN_LED, true);
        emit signalMachineEvent(MachineEvent::STOP_BTN);
    }
    else if (!stop_btn && m_stopBtnPressed) {
        m_stopBtnPressed = false;
        MotionController::instance().set_DO(m_motionID, 0, (int)DOA::STOP_BTN_LED, false);
    }

    auto reset_btn = io[(int)DIA::RESET_BTN];
    if (reset_btn && !m_resetBtnPressed) {
        m_resetBtnPressed = true;
        MotionController::instance().set_DO(m_motionID, 0, (int)DOA::RESET_BTN_LED, true);

        //after a curtain trip the drives are off: reset turns all three servo axes
        //back on first (only once the curtain is clear), then clears the alarm
        /*
        * Drive recovery on a reset press. Widened past the curtain on purpose: an e-stop on its
        * own drops the drives just the same, and the only way back from that used to be the
        * Motion page's Reset Alarm button - which an operator cannot reach, because production
        * mode locks the UI to the production page.
        *
        * Still TWO presses after an e-stop, by design. The reset button is what closes the
        * safety relay in HARDWARE, so on the press that closes it X106 is still low when read
        * here and the guard below refuses; the operator presses again a moment later and it
        * goes through. Waiting for the relay in line would block the IO poll loop, and that
        * loop is the one thing that has to keep running.
        */
        /*
        * NOT servo_on alone. That member is just SVON, and SVON stays SET through an e-stop -
        * the drive keeps reporting the servo-on command while the relay has cut its power. It
        * is the same lie that dropped the head, and using it here meant an e-stop with no
        * curtain trip skipped this whole block: reset did nothing, no matter how many times it
        * was pressed, and the Motion page was still the only way back.
        *
        * A latched drive ALARM is the honest signal. reset_alarm() no-ops on an axis whose ALM
        * is clear, so the fact that the Motion page button fixes this case is itself proof the
        * drives really are alarmed after an e-stop.
        */
        const bool servosDropped = !(m_x.servo_on && m_y.servo_on && m_z.servo_on);
        const bool anyDriveAlarm = m_x.alarm || m_y.alarm || m_z.alarm;

        if (m_curtainTripped || servosDropped || anyDriveAlarm) {
            const bool curtainClear = !io[(int)DIA::CURTAIN_SAFETY_RELAY]; //inverted wiring
            const bool estopClear = io[(int)DIA::ESTOP_1] && io[(int)DIA::ESTOP_2]
                && io[(int)DIA::ESTOP_SAFETY_RELAY];                       //NC: high = healthy

            if (!curtainClear) {
                ct::logger::warn("[MachineController] Reset pressed but the curtain sensor is still triggered.");
            }
            else if (!estopClear) {
                /*
                * Do NOT power the drives back up while an e-stop is still open. servoOnAllAxes()
                * finishes by releasing the Z brake, and with the safety relay open the drive has
                * no torque to take the load, so the head drops. safelyReleaseBrake() refuses on
                * its own now as well - this branch exists so the operator gets a reason rather
                * than a silent no-op, and so the drives are never even commanded on.
                *
                * Tested against the live io read, not m_estopButtonsOk: that member is only
                * refreshed further down this same function, so it would be a cycle stale here.
                */
                ct::logger::warn("[MachineController] Reset pressed but an e-stop is still active - "
                    "servo power and the Z brake stay off. Press reset again once the safety "
                    "relay has closed.");
            }
            else if (recoverDrives()) {
                if (m_curtainTripped) {
                    m_curtainTripped = false;
                    //clear the latched error NOW - resetAlarm below refuses while any
                    //error is still active, and the poll only re-assesses next cycle
                    assessError(true, MachineError::CURTAIN_RELAY_FAULT);
                }
            }
        }

        /*
        * Deferred by one poll cycle, not called here.
        *
        * resetAlarm() refuses while ANY error is still in the set, and at this point in the
        * cycle the set is stale: ESTOP_RELAY_FAULT is re-assessed further down this very
        * function, and ESTOP_PRESSED and the drive alarms are not re-assessed until
        * handleAxisState() runs after us. So a press that genuinely fixed everything still
        * saw the previous cycle's errors and failed - which is why recovery needed one extra
        * press purely to let the poll catch up.
        *
        * Running it at the end of handleAxisState() instead means it is judged against a
        * fully refreshed error set, 10 ms later. Invisible to the operator.
        */
        m_resetRequested = true;
        emit signalMachineEvent(MachineEvent::RESET_BTN);
    }
    else if (!reset_btn && m_resetBtnPressed) {
        m_resetBtnPressed = false;
        MotionController::instance().set_DO(m_motionID, 0, (int)DOA::RESET_BTN_LED, false);
    }

    //Check estop triggers, NC: high = good. Both buttons are combined into one flag here and
    //assessed in handleAxisState alongside the drive EMG input - see m_estopButtonsOk. Calling
    //assessError() once per button would let a released button clear the error raised by a
    //pressed one, because both map to the single ESTOP_PRESSED code and the last call wins.
    m_estopButtonsOk = io[(int)DIA::ESTOP_1] && io[(int)DIA::ESTOP_2];

    //Check safety relays, high = OK
    assessError(io[(int)DIA::ESTOP_SAFETY_RELAY], MachineError::ESTOP_RELAY_FAULT);

    //Curtain sensor: X107 reads HIGH while the beam is broken (inverted wiring). The trip
    //LATCHES: the error stays active after the beam clears and never self-resets - the
    //operator must press reset (which also re-servos X/Y/Z) to acknowledge the break.
    if (io[(int)DIA::CURTAIN_SAFETY_RELAY]) m_curtainTripped = true;
    assessError(!m_curtainTripped, MachineError::CURTAIN_RELAY_FAULT);

    //Check trolley lock guard, bypassed in debug mode or when config\interlock.json exists
    bool trolleyLocked = SystemData::instance()._machineDebugMode || m_bypassInterlock || io[(int)DIA::TROLLEY_LOCK_GUARD];
    assessError(trolleyLocked, MachineError::TROLLEY_GUARD_OPEN);

    //Trolley auto-lock: the guard switch going OFF -> ON means the trolley was just
    //attached - engage the lock so it cannot be pulled out mid-run. The production run's
    //end releases it (VisionApp_Production). Both halves obey the Auto Lock Trolley config.
    const bool trolleyOn = io[(int)DIA::TROLLEY_LOCK_GUARD];
    if (trolleyOn && !m_trolleyGuardOn && SystemData::instance()._autoLockTrolley) {
        MotionController::instance().set_DO(m_motionID, 0, (int)DOA::TROLLEY_LOCK_RELEASE, true);
        ct::logger::info("[MachineController] Trolley attached - lock engaged (auto)");
    }
    m_trolleyGuardOn = trolleyOn;
}

void MachineController::handleDIB()
{
    //not in use
}

void MachineController::handleDOA()
{
    return; //need to think how to handle when brake is release, should software take responsibility or user.

    auto optional_io = MotionController::instance().get_all_DO(m_motionID, 0);
    if (!optional_io.has_value()) {
        ct::logger::error("[Machine] EMXA DO not responding...");
        return;
    }

    auto io = optional_io.value();
    if (io.size() <= (int)DOA::LAST_INDEX) {
        ct::logger::error("[Machine] EMXA DO invalid index");
        return;
    }

    //Check brake release
    auto brake_release = io[(int)DOA::BRAKE_RELEASE];
    
}

void MachineController::handleDOB()
{
    //not in use
}

void MachineController::handleAxisState()
{
    bool servo_on = true;
    bool limit = false;
    bool jogging = false;

    auto optional_axisX = MotionController::instance().get_motion_io_status(m_motionID, (int)Axis::X);
    if (optional_axisX.has_value()) {
        auto motion_io = optional_axisX.value();

        if (motion_io.size() != 9) ct::logger::error("[Motion_APS] Invalid size for motion io");

        //One assessment for all three e-stop sources - the two buttons (X103/X104, read in
        //handleDIA) and this axis's drive EMG input. They share the ESTOP_PRESSED code, so
        //assessing them separately would make them cancel each other out and, if the drive
        //latches EMG after the buttons are released, oscillate the error at the poll rate.
        assessError(m_estopButtonsOk && !motion_io[Motion_APS::EMG], MachineError::ESTOP_PRESSED);
        m_x.alarm = motion_io[Motion_APS::ALM];
        limit |= m_x.positive_limit = motion_io[Motion_APS::PEL];
        limit |= m_x.negative_limit = motion_io[Motion_APS::NEL];
        jogging |= !motion_io[Motion_APS::INP];
        servo_on &= m_x.servo_on = motion_io[Motion_APS::SVON];
    }

    auto optional_axisY = MotionController::instance().get_motion_io_status(m_motionID, (int)Axis::Y);
    if (optional_axisY.has_value()) {
        auto motion_io = optional_axisY.value();

        if (motion_io.size() != 9) ct::logger::error("[Motion_APS] Invalid size for motion io");

        m_y.alarm = motion_io[Motion_APS::ALM];
        limit |= m_y.positive_limit = motion_io[Motion_APS::PEL];
        limit |= m_y.negative_limit = motion_io[Motion_APS::NEL];
        jogging |= !motion_io[Motion_APS::INP];
        servo_on &= m_y.servo_on = motion_io[Motion_APS::SVON];
    }

    auto optional_axisZ = MotionController::instance().get_motion_io_status(m_motionID, (int)Axis::Z);
    if (optional_axisZ.has_value()) {
        auto motion_io = optional_axisZ.value();

        if (motion_io.size() != 9) ct::logger::error("[Motion_APS] Invalid size for motion io");

        m_z.alarm = motion_io[Motion_APS::ALM];
        limit |= m_z.positive_limit = motion_io[Motion_APS::PEL];
        limit |= m_z.negative_limit = motion_io[Motion_APS::NEL];
        //jogging |= !MotionController::instance().move_done(m_motionID, 0, (int)Axis::Z);
        servo_on &= m_z.servo_on = motion_io[Motion_APS::SVON];
       
        //NOTE: Comment off this, use XYZ jogging to track instead. Unless prefer to brake when Z is not in use, but might be frequent braking
        ////check for motion, if not moved for 10min, set to idle state
        //if (!MotionController::instance().move_done(m_motionID, 0, (int)Axis::Z)) {
        //    m_lastZMotionTime = std::chrono::steady_clock::now();  // reset timer
        //}

        //auto now = std::chrono::steady_clock::now();
        //auto elapsed = std::chrono::duration_cast<std::chrono::minutes>(now - m_lastZMotionTime.load());

        //if (elapsed >= std::chrono::minutes(10)) {
        //    setMachineState(MachineState::IDLE);
        //}
    }

    //check for motion, if not moved for 10min, set to idle state
    //if (jogging) m_lastZMotionTime = std::chrono::steady_clock::now();  // reset timer
   
    //auto now = std::chrono::steady_clock::now();
    //auto elapsed = std::chrono::duration_cast<std::chrono::minutes>(now - m_lastZMotionTime.load());

    //if (elapsed >= std::chrono::minutes(10)) setMachineState(MachineState::IDLE);


    assessError(!m_x.alarm, MachineError::X_DRIVER_ALARM);
    assessError(!m_y.alarm, MachineError::Y_DRIVER_ALARM);
    assessError(!m_z.alarm, MachineError::Z_DRIVER_ALARM);


    //Note: No need to check for limit if its homing, as homing uses limit to execute
    if (m_currentEvent != MachineEvent::HOMING) {
        assessError(!m_x.positive_limit, MachineError::X_POSITIVE_LIMIT_HIT);
        assessError(!m_x.negative_limit, MachineError::X_NEGATIVE_LIMIT_HIT);
        assessError(!m_y.positive_limit, MachineError::Y_POSITIVE_LIMIT_HIT);
        assessError(!m_y.negative_limit, MachineError::Y_NEGATIVE_LIMIT_HIT);
        assessError(!m_z.positive_limit, MachineError::Z_POSITIVE_LIMIT_HIT);
        assessError(!m_z.negative_limit, MachineError::Z_NEGATIVE_LIMIT_HIT);

        //Limit errors self-clear: the operator recovers by jogging OFF the switch, and a
        //reset press on top of that adds nothing. Auto-reset only when the limit was the
        //only problem - any other active error still needs the reset button.
        const bool anyLimit = m_x.positive_limit || m_x.negative_limit
            || m_y.positive_limit || m_y.negative_limit
            || m_z.positive_limit || m_z.negative_limit;
        if (anyLimit) {
            m_limitWasHit = true;
        }
        else if (m_limitWasHit) {
            m_limitWasHit = false;
            if (m_currentState == MachineState::S_ERROR && m_errorStatuses.isEmpty()) {
                ct::logger::info("[MachineController] Axis moved off the limit - error cleared automatically");
                setMachineState(m_readyState);
            }
        }
    }

    //force user home when servo is off
    if (!servo_on) {
        setMachineState(MachineState::NOT_READY);
    }

    //Every error has now been re-assessed this cycle, so a reset press from handleDIA can
    //finally be judged against the truth rather than against the state that caused it.
    if (m_resetRequested) {
        m_resetRequested = false;
        resetAlarm();
    }
}

void MachineController::setMachineState(MachineState state)
{
   //std::lock_guard<std::mutex> lock(m_mutex);
   if (m_currentState == state) return;

   if (state == MachineState::READY || state == MachineState::NOT_READY) {
       m_readyState = state;
   }

   if (m_currentState == MachineState::S_ERROR) {
       if (!m_errorStatuses.isEmpty()) {
           return;
       }
   }

   m_currentState = state;
   emit signalMachineState(state);

   ct::logger::info("[MachineController] State changed to: %d", (int)state);

   switch (m_currentState)
   {
       case MachineState::NOT_READY:
           m_readyState = m_currentState;
           setTowerLight(DOA::AMBER_TOWER_LIGHT);
           if (!SystemData::instance()._machineDebugMode) MotionController::instance().enable_motion(false);
           break;
       case MachineState::READY:
           m_readyState = m_currentState;
           setTowerLight(DOA::GREEN_TOWER_LIGHT);
           if (!SystemData::instance()._machineDebugMode) MotionController::instance().enable_motion(true);
           break;
       case MachineState::IDLE:
           turnOnBrake();
           break;
       case MachineState::JOGGING:

           break;
       case MachineState::WARNING:
           break;
       case MachineState::S_ERROR:
           setTowerLight(DOA::RED_TOWER_LIGHT);
           //Leave motion enabled when every active error is a limit hit: the axis is parked on
           //a switch and moving is the only cure. See limitRecoveryOnly(). assessError() closes
           //the gate again the moment a non-limit error joins it.
           if (!SystemData::instance()._machineDebugMode && !limitRecoveryOnly()) {
               MotionController::instance().enable_motion(false);
           }
           break;
       default:
           break;
   }

}

bool MachineController::turnOnBrake()
{
    if (!m_enable) return false;
    auto ret = MotionController::instance().set_DO(m_motionID, 0, (int)DOA::BRAKE_RELEASE, false);
    ct::logger::info("[MachineController] Brake is turned on.");
    return ret;
}

/*
* Single owner of the "buzzer off" write, so every path that has to guarantee silence spells it
* the same way. Deliberately NOT guarded by debug mode: guard what turns the buzzer on, never
* what turns it off - see startRedTowerLight() and stopRedTowerLight().
*/
void MachineController::silenceBuzzer()
{
    if (!m_enable) return;
    MotionController::instance().set_DO(m_motionID, 0, (int)DOA::BUZZER, false);
}

bool MachineController::safelyReleaseBrake(int servoWaitMs)
{
    if (!m_enable) return false;

    /*
    * Never release the Z brake while a safety circuit is open.
    *
    * SVON is NOT proof that the axis can hold. With an e-stop pressed the safety relay cuts
    * drive power in hardware, yet the drive can still report the servo-on COMMAND as set -
    * so the SVON check below passes, the brake comes off, and a vertical axis is left with
    * neither torque nor brake. That is exactly what dropped the head when reset was pressed
    * with the e-stop and the curtain both triggered.
    *
    * Read the inputs live rather than consulting m_errorStatuses: this runs from the reset
    * path, from homing and from the UI, and it must not depend on when the 10 ms poll last
    * ran. E-stops and the relay are NC (high = healthy); the curtain is inverted (high =
    * broken). An unreadable card refuses too - the brake staying on is the safe answer.
    *
    * This is the only place in the codebase that sets BRAKE_RELEASE true, so this guard
    * covers every caller.
    */
    auto optional_di = MotionController::instance().get_all_DI(m_motionID, 0);
    if (!optional_di.has_value() || (int)optional_di.value().size() <= (int)DIA::LAST_INDEX) {
        ct::logger::error("[MachineController] Cannot read the safety inputs - the Z brake stays applied.");
        return false;
    }

    const auto& di = optional_di.value();
    const bool estopOk = di[(int)DIA::ESTOP_1] && di[(int)DIA::ESTOP_2] && di[(int)DIA::ESTOP_SAFETY_RELAY];
    const bool curtainClear = !di[(int)DIA::CURTAIN_SAFETY_RELAY];

    if (!estopOk || !curtainClear) {
        ct::logger::error("[MachineController] Unsafe to release the Z brake - a safety circuit is open "
            "(estop1=%d estop2=%d relay=%d curtain_broken=%d). Brake stays applied.",
            (int)di[(int)DIA::ESTOP_1], (int)di[(int)DIA::ESTOP_2],
            (int)di[(int)DIA::ESTOP_SAFETY_RELAY], (int)di[(int)DIA::CURTAIN_SAFETY_RELAY]);
        return false;
    }

    /*
    * SVON on its own is NOT enough, and that is the second half of the dropped-head bug. It
    * only says the servo-on command latched, never that the drive can hold anything:
    *
    *   RDY  the power stage is live. On an e-stop reset the safety relay closes FIRST and the
    *        drive comes back a moment later, so there is a real window where SVON reads set
    *        while RDY is still low. Pressing reset a second time walked straight into it and
    *        the head fell again even with the safety inputs all healthy.
    *   ALM  a drive alarm means it is not holding, whatever else it reports.
    *   EMG  the drive's own view of the emergency input - independent evidence from the DI
    *        read above, which only sees the buttons and the relay.
    *
    * Wait for all four, then SETTLE and check again. Torque does not appear the instant SVON
    * latches, and one lucky sample from a drive that is still dropping in and out must not be
    * what a vertical axis is trusted to.
    */
    std::vector<bool> z;
    auto zHolding = [&]() {
        auto io = MotionController::instance().get_motion_io_status(m_motionID, (int)Axis::Z);
        if (!io.has_value() || (int)io.value().size() <= (int)Motion_APS::RDY) return false;
        z = io.value();
        return z[(int)Motion_APS::SVON] && z[(int)Motion_APS::RDY]
            && !z[(int)Motion_APS::ALM] && !z[(int)Motion_APS::EMG];
    };

    bool holding = false;
    auto start = std::chrono::steady_clock::now();
    while (true) {
        holding = zHolding();
        if (holding) break;

        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count();
        if (elapsed >= servoWaitMs) break;
        os_tool::goSleep(50);
    }

    if (holding) {
        os_tool::goSleep(BRAKE_SETTLE_MS); //let torque build, then confirm it is STILL holding
        holding = zHolding();
    }

    if (!holding) {
        if ((int)z.size() > (int)Motion_APS::RDY) {
            ct::logger::warn("[MachineController] Unsafe to release the Z brake - the drive is not holding "
                "(svon=%d rdy=%d alm=%d emg=%d). Brake stays applied.",
                (int)z[(int)Motion_APS::SVON], (int)z[(int)Motion_APS::RDY],
                (int)z[(int)Motion_APS::ALM], (int)z[(int)Motion_APS::EMG]);
        }
        else {
            ct::logger::warn("[MachineController] Unsafe to release the Z brake - the Z axis did not "
                "answer. Brake stays applied.");
        }
        return false;
    }

    auto ret = MotionController::instance().set_DO(m_motionID, 0, (int)DOA::BRAKE_RELEASE, true);
    ct::logger::info("[MachineController] Brake is released.");
    return ret;
}

bool MachineController::recoverDrives()
{
    if (!m_enable) return false;

    //Brake FIRST: an alarm reset drops the servo command, so the Z axis is unheld across it.
    //Same ordering the Motion page's Reset Alarm button uses.
    turnOnBrake();

    //Logged individually even though they are cleared automatically - a drive that alarms for a
    //real reason (overload, encoder) must still leave a trace for whoever looks later, and a
    //fault that has not gone away simply re-alarms on the next poll.
    for (int axis : { (int)Axis::X, (int)Axis::Y, (int)Axis::Z }) {
        if (!MotionController::instance().reset_alarm(m_motionID, axis)) {
            ct::logger::error("[MachineController] Axis %d alarm reset failed: %s", axis,
                qPrintable(MotionController::instance().error_msg(m_motionID)));
        }
        else {
            ct::logger::info("[MachineController] Axis %d drive alarm reset", axis);
        }
    }

    return servoOnAllAxes();
}

//Turn all three servo axes on, wait (bounded) for each drive to report SVON, clear
//the servo-off errors, and release the Z brake - the curtain-trip recovery path.
bool MachineController::servoOnAllAxes()
{
    if (!m_enable) return false;

    /*
    * Wait for each drive's power stage to come back BEFORE commanding servo on. On an e-stop
    * reset the safety relay closes first and the drives follow, so a servo-on issued into a
    * drive that is not ready yet is simply ignored - and the old code then sat waiting for an
    * SVON that was never going to arrive, or worse, saw a stale one.
    */
    /*
    * ONE loop covering all three axes, not three sequential waits. This whole function runs on
    * the IO poll thread, so every millisecond here is a millisecond the e-stop, the curtain and
    * the limits are NOT being read. Three back-to-back timeouts tripled the worst case for no
    * benefit, because the drives come back together, not one after another.
    */
    const int axes[3] = { (int)Axis::X, (int)Axis::Y, (int)Axis::Z };

    auto axisState = [&](int axis, std::vector<bool>& out) {
        auto io = MotionController::instance().get_motion_io_status(m_motionID, axis);
        if (!io.has_value() || (int)io.value().size() <= (int)Motion_APS::RDY) return false;
        out = io.value();
        return true;
    };

    {
        auto start = std::chrono::steady_clock::now();
        while (true) {
            bool allReady = true;
            std::vector<bool> s;
            for (int axis : axes) allReady &= axisState(axis, s) && s[(int)Motion_APS::RDY];
            if (allReady) break;

            if (std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - start).count() >= 3000) break;
            os_tool::goSleep(50);
        }
    }

    ct::logger::info("[MachineController] Turning ON all servo axes (X, Y, Z)");
    for (int axis : axes)
        MotionController::instance().set_servo(m_motionID, 0, axis, true);

    //Same single timeout, and the same standard safelyReleaseBrake() applies - so "all on"
    //here really means all three are holding, not merely commanded.
    bool on[3] = { false, false, false };
    {
        auto start = std::chrono::steady_clock::now();
        while (true) {
            /*
            * Re-read all three EVERY pass rather than latching each one as it comes good. An
            * axis that reports ready and then drops out again must not stay counted - the exit
            * condition has to mean "all three are holding at the same time", not "each was
            * holding at some point". on[] therefore always reflects the newest read, which is
            * also what the failure log below should be reporting.
            */
            for (int i = 0; i < 3; ++i) {
                std::vector<bool> s;
                on[i] = axisState(axes[i], s)
                    && s[(int)Motion_APS::SVON] && s[(int)Motion_APS::RDY]
                    && !s[(int)Motion_APS::ALM] && !s[(int)Motion_APS::EMG];
            }
            if (on[0] && on[1] && on[2]) break;

            if (std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - start).count() >= 2000) break;
            os_tool::goSleep(50);
        }
    }

    bool allOn = true;
    for (int i = 0; i < 3; ++i) {
        if (!on[i]) ct::logger::error("[MachineController] Axis %d did not come back ready "
            "(needs SVON and RDY, with no alarm or drive emergency)", axes[i]);
        allOn &= on[i];
    }

    if (allOn) {
        //servo-off errors are raised externally (notifyError), not re-assessed each
        //poll - clear them here so the reset that follows can succeed
        assessError(true, MachineError::X_SERVO_OFF);
        assessError(true, MachineError::Y_SERVO_OFF);
        assessError(true, MachineError::Z_SERVO_OFF);
        safelyReleaseBrake(2000);
    }

    return allOn;
}

void MachineController::setTowerLight(DOA towerLight)
{
    if (!m_enable) return;

    /*
    * Stop the blink BEFORE writing the new colour, and do it with the flag rather than the
    * timer. The invokeMethod below is queued to the controller thread, so it cannot run while
    * the blink lambda is mid-cycle - and that lambda sleeps for a second inside one cycle.
    * Waiting for it meant the blink kept overwriting the colour selected here.
    */
    m_blinkActive = false;

    if (towerLight == DOA::GREEN_TOWER_LIGHT) {
        ct::logger::info("[MachineController] Turn on GREEN tower light.");
        QMetaObject::invokeMethod(&MachineController::instance(), "stopRedTowerLight", Qt::QueuedConnection);
        MotionController::instance().set_DO(m_motionID, 0, (int)DOA::AMBER_TOWER_LIGHT, false);
        MotionController::instance().set_DO(m_motionID, 0, (int)DOA::GREEN_TOWER_LIGHT, true);
    }
    else if (towerLight == DOA::AMBER_TOWER_LIGHT) {
        ct::logger::info("[MachineController] Turn on AMBER tower light.");
        QMetaObject::invokeMethod(&MachineController::instance(), "stopRedTowerLight", Qt::QueuedConnection);
        MotionController::instance().set_DO(m_motionID, 0, (int)DOA::GREEN_TOWER_LIGHT, false);
        MotionController::instance().set_DO(m_motionID, 0, (int)DOA::AMBER_TOWER_LIGHT, true);
    }
    else if (towerLight == DOA::RED_TOWER_LIGHT) {
        ct::logger::info("[MachineController] Turn on RED tower light.");
        MotionController::instance().set_DO(m_motionID, 0, (int)DOA::GREEN_TOWER_LIGHT, false);
        MotionController::instance().set_DO(m_motionID, 0, (int)DOA::AMBER_TOWER_LIGHT, false);
        QMetaObject::invokeMethod(&MachineController::instance(), "startRedTowerLight", Qt::QueuedConnection);
    }
}

void MachineController::startRedTowerLight()
{
    //The timer is only created in run(), but these are queued slots: they stay
    //reachable even when the controller was never started.
    if (m_redTowerTimer == nullptr) return;
    m_blinkActive = true;
    m_redTowerTimer->start(500);

    /*
    * The buzzer follows the red light: one write on here, one write off in
    * stopRedTowerLight(). Deliberately NOT blinked with the lamp - the 500 ms cycle
    * exists to make the lamp flash, and the buzzer has no visual duty to satisfy, so
    * pulsing Y106 would only add two card writes a second for the length of the alarm.
    *
    * Because nothing rewrites Y106 afterwards, the Motion page's DO7 button doubles as a
    * manual silence during an alarm (admin only). The lamp rows cannot do that - the
    * timer lambda overwrites them twice a second.
    *
    * Debug mode blinks AMBER instead of red (see the lambda in run()), so there is no red
    * light to follow and the machine stays quiet while teaching. The guard is only on this
    * ON write - see stopRedTowerLight() for why the OFF is unconditional.
    */
    if (!SystemData::instance()._machineDebugMode) {
        MotionController::instance().set_DO(m_motionID, 0, (int)DOA::BUZZER, true);
    }
}

void MachineController::stopRedTowerLight()
{
    m_blinkActive = false; //before the early return: the flag must clear even with no timer

    if (m_redTowerTimer == nullptr) return;
    m_redTowerTimer->stop();

    //the blink may stop mid-phase - make sure nothing it drives is left lit. RED needs this as
    //much as the reset LED does, and it is the only bit here that no colour write clears:
    //setTowerLight() drives GREEN and AMBER explicitly but never touches RED, so a blink
    //stranded between its on and off halves left red burning underneath the new colour.
    //AMBER is deliberately NOT cleared here - this call is queued and lands AFTER
    //setTowerLight(AMBER) has raised it, so clearing it would put the amber light straight out.
    MotionController::instance().set_DO(m_motionID, 0, (int)DOA::RED_TOWER_LIGHT, false);
    MotionController::instance().set_DO(m_motionID, 0, (int)DOA::RESET_BTN_LED, false);

    //Unconditional, unlike the debug-mode guarded ON in startRedTowerLight(): switching
    //debug mode on during an alarm must not strand a buzzer that is already sounding.
    //Guard what turns it on, never what turns it off.
    silenceBuzzer();
}

bool MachineController::isServoOn(Axis axis)
{
    if (axis == Axis::X) return m_x.servo_on;
    else if (axis == Axis::Y) return m_y.servo_on;
    else if (axis == Axis::Z) return m_z.servo_on;
    return false;
}

bool MachineController::setBypassAxis(Axis axis, bool bypass)
{
    m_bypassAxes[(int)axis] = bypass;
    return true;
}

bool MachineController::isBypassAxis(Axis axis)
{
    auto key = (int)axis;
    if (m_bypassAxes.contains(key)) {
        return m_bypassAxes[key];
    }
    return false;
}

bool MachineController::setBypassError(MachineError error, bool bypass)
{
    m_bypassErrors[(int)error] = bypass;
    return true;
}

bool MachineController::isBypassError(MachineError error)
{
    auto key = (int)error;
    if (m_bypassErrors.contains(key)) {
        return m_bypassErrors[key];
    }
    return false;
}

void MachineController::trackTime(QString key)
{
    m_timer[key] = std::chrono::system_clock::now();
}

long long MachineController::logTime(QString key)
{
    if (!m_timer.contains(key)) return -1;
    auto t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - m_timer[key]).count();
    emit signalLogTime(key, t);
    return t;
}

QSet<int> MachineController::getErrorStatus()
{
    return m_errorStatuses;
}
