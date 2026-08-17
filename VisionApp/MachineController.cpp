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
    setBypassAxis(Axis::CY1, false);
    //m_bypassErrors[(int)MachineError::AIR_PRESSURE_OFF] = true;
    //m_bypassErrors[(int)MachineError::ESTOP_PRESSED] = true;
    //m_bypassErrors[(int)MachineError::RAIL_NEGATIVE_LIMIT_HIT] = true;
    //m_bypassErrors[(int)MachineError::RAIL_POSITIVE_LIMIT_HIT] = true;

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

    if (m_redTowerTimer == nullptr) {
        m_redTowerTimer = new QTimer();

        QObject::connect(m_redTowerTimer, &QTimer::timeout, [&]() {
            int bit = (int)DOA::RED_TOWER_LIGHT;
            if (SystemData::instance()._machineDebugMode) bit = (int)DOA::AMBER_TOWER_LIGHT;
            MotionController::instance().set_DO(m_motionID, 0, bit, true);
            os_tool::doNothing(500);
            MotionController::instance().set_DO(m_motionID, 0, bit, false);
            os_tool::doNothing(500);
        });
    }

    setMachineState(MachineState::NOT_READY);

    m_stateThread = std::thread([this]() {
        poolStates();
    });

    safelyReleaseBrake();

    exec();
}

void MachineController::release()
{
    if (!m_enable) return;

    m_running = false;

    if (m_stateThread.joinable()) m_stateThread.join();

    quit();  // Exits the event loop
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
    else if (e == MachineError::CONVEYOR_SERVO_OFF) assessError(false, e);
   
    setMachineState(MachineState::S_ERROR);
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
    case MachineEvent::RAIL_SERVO_ON:
        assessError(true, MachineError::RAIL_SERVO_OFF);
        break;
    }
}

void MachineController::poolStates()
{
    while (m_running) {
        handleDIA();
        handleDIB();
        handleDOA();
        handleDOB();
        handleAxisState();
        os_tool::goSleep(10);
    }
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
        }
    }
    else {
        if (!good) {
            m_errorStatuses.insert(s);
            setMachineState(MachineState::S_ERROR);
            emit signalMachineError(e);

            //Specific status handling
            if (e == MachineError::ESTOP_PRESSED ||
                e == MachineError::MAIN_POWER_OFF ||
                e == MachineError::DRIVER_OFF)
            {
                turnOnBrake();
            }
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

    //Check button triggers
    auto start_btn = io[(int)DIA::START_BTN];
    if (start_btn && !m_startBtnPressed) {
        m_startBtnPressed = true;
        emit signalMachineEvent(MachineEvent::START_BTN);
    }
    else if (!start_btn) {
        m_startBtnPressed = false;
    }

    auto stop_btn = io[(int)DIA::STOP_BTN];
    if (stop_btn && !m_stopBtnPressed) {
        m_stopBtnPressed = true;
        emit signalMachineEvent(MachineEvent::STOP_BTN);
    }
    else if (!stop_btn) {
        m_stopBtnPressed = false;
    }

    auto reset_btn = io[(int)DIA::RESET_BTN];
    if (reset_btn && !m_resetBtnPressed) {
        m_resetBtnPressed = true;
        resetAlarm();
        emit signalMachineEvent(MachineEvent::RESET_BTN);
    }
    else if (!reset_btn) {
        m_resetBtnPressed = false;
    }

    //Check estop
    //assessError(io[(int)DIA::ESTOP], MachineError::ESTOP_PRESSED); //Estop is NC, NC is invert logic

    //Check power
    assessError(!io[(int)DIA::CONTACTOR_1], MachineError::MAIN_POWER_OFF); //Contactor is NC, NC is invert logic
    assessError(!io[(int)DIA::CONTACTOR_2], MachineError::DRIVER_OFF); //Contactor is NC, NC is invert logic
    
    //Check air pressure
    assessError(io[(int)DIA::AIR_PRESSURE], MachineError::AIR_PRESSURE_OFF);

    //Check door open
    bool doorLockClosed = !io[(int)DIA::DOOR_LOCK]; //NC
    bool doorInterlockClosed = SystemData::instance()._machineDebugMode || m_bypassInterlock || io[(int)DIA::DOOR_INTERLOCK]; //NO, bypassed in debug mode or when config\interlock.json exists
    assessError(doorLockClosed && doorInterlockClosed, MachineError::DOOR_OPEN);
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

        assessError(!motion_io[Motion_APS::EMG], MachineError::ESTOP_PRESSED);
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


    auto optional_axisR1 = MotionController::instance().get_motion_io_status(m_motionID, (int)Axis::FR1);
    if (optional_axisR1.has_value()) {
        auto motion_io = optional_axisR1.value();

        if (motion_io.size() != 9) ct::logger::error("[Motion_APS] Invalid size for motion io");

        m_rail.alarm = motion_io[Motion_APS::ALM];
        limit |= m_rail.positive_limit = motion_io[Motion_APS::PEL];
        limit |= m_rail.negative_limit = motion_io[Motion_APS::NEL];
        servo_on &= m_rail.servo_on = motion_io[Motion_APS::SVON];
    }

    auto optional_axisCY1 = MotionController::instance().get_motion_io_status(m_motionID, (int)Axis::CY1);
    if (optional_axisCY1.has_value()) {
        auto motion_io = optional_axisCY1.value();

        if (motion_io.size() != 9) ct::logger::error("[Motion_APS] Invalid size for motion io");

        m_cy.alarm = motion_io[Motion_APS::ALM];
    }

    assessError(!m_x.alarm, MachineError::X_DRIVER_ALARM);
    assessError(!m_y.alarm, MachineError::Y_DRIVER_ALARM);
    assessError(!m_z.alarm, MachineError::Z_DRIVER_ALARM);
    assessError(!m_rail.alarm, MachineError::RAIL_DRIVER_ALARM);
    assessError(!m_cy.alarm, MachineError::CONVEYOR_DRIVER_ALARM);


    //Note: No need to check for limit if its homing, as homing uses limit to execute
    if (m_currentEvent != MachineEvent::HOMING) {
        assessError(!m_x.positive_limit, MachineError::X_POSITIVE_LIMIT_HIT);
        assessError(!m_x.negative_limit, MachineError::X_NEGATIVE_LIMIT_HIT);
        assessError(!m_y.positive_limit, MachineError::Y_POSITIVE_LIMIT_HIT);
        assessError(!m_y.negative_limit, MachineError::Y_NEGATIVE_LIMIT_HIT);
        assessError(!m_z.positive_limit, MachineError::Z_POSITIVE_LIMIT_HIT);
        assessError(!m_z.negative_limit, MachineError::Z_NEGATIVE_LIMIT_HIT);
        assessError(!m_rail.positive_limit, MachineError::RAIL_POSITIVE_LIMIT_HIT);
        assessError(!m_rail.negative_limit, MachineError::RAIL_NEGATIVE_LIMIT_HIT);
    }

    //force user home when servo is off
    if (!servo_on) {
        setMachineState(MachineState::NOT_READY); 
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
           if (!SystemData::instance()._machineDebugMode) MotionController::instance().enable_motion(false);
           break;
       default:
           break;
   }

}

MachineController::SensorStatus MachineController::getSensorStatus()
{
    SensorStatus status;

    //DI 8: Start, 9: Front Sensor, 10: Exit 
    auto optional_EMXA_DIs = MotionController::instance().get_all_DI(m_motionID, 0);

    if (!optional_EMXA_DIs.has_value()) {
        ct::logger::error("Unable to load front pallet! Failed to get reading from sensors");
        status.valid = false;
        return status;
    }

    auto EMXA_DIs = optional_EMXA_DIs.value();

    if (!(EMXA_DIs.size() > 10)) {
        ct::logger::error("Unable to load front pallet! Not all reading from sensors is received");
        status.valid = false;
        return status;
    }

    status.valid = true;
    status.sensors[(int)SensorIndex::START] = EMXA_DIs[8];
    status.sensors[(int)SensorIndex::RIGHT] = EMXA_DIs[20];
    status.sensors[(int)SensorIndex::LEFT] = EMXA_DIs[21];
    status.sensors[(int)SensorIndex::SLOW] = EMXA_DIs[9];
    status.sensors[(int)SensorIndex::EXIT] = EMXA_DIs[10];

    return status;
}

bool MachineController::isAllSensorOff(const SensorStatus& status) {
    bool off = true;

    for (auto& s : status.sensors) {
        off &= !s;
    }

    return off;
}

bool MachineController::turnOnBrake()
{
    if (!m_enable) return false;
    auto ret = MotionController::instance().set_DO(m_motionID, 0, (int)DOA::BRAKE_RELEASE, false);
    ct::logger::info("[MachineController] Brake is turned on.");
    return ret;
}

bool MachineController::safelyReleaseBrake()
{
    if (!m_enable) return false;

    bool ret = false;

    if (m_z.servo_on) {
        ret = MotionController::instance().set_DO(m_motionID, 0, (int)DOA::BRAKE_RELEASE, true);
        ct::logger::info("[MachineController] Brake is released.");
    }
    else {
        ct::logger::warn("Unsafe to release brake, z servo is not on.");
    }

    return ret;
}

void MachineController::setTowerLight(DOA towerLight)
{
    if (!m_enable) return;

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
    m_redTowerTimer->start(500);
}

void MachineController::stopRedTowerLight()
{
    m_redTowerTimer->stop();
}

bool MachineController::isServoOn(Axis axis)
{
    if (axis == Axis::X) return m_x.servo_on;
    else if (axis == Axis::Y) return m_y.servo_on;
    else if (axis == Axis::Z) return m_z.servo_on;
    else if (axis == Axis::FR1) return m_rail.servo_on;
    else if (axis == Axis::CY1) return m_cy.servo_on;
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
