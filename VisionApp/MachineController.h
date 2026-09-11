#pragma once
#include <QObject>
#include <QThread>
#include <QMutex>
#include <QMap>
#include <QString>
#include <QSet>
#include <QTimer>
#include <thread>
#include <atomic>
#include <mutex>
#include "SystemData.h"

enum class MachineState {
    STARTUP,
    NOT_READY, 
    READY,
    IDLE, 
    JOGGING,
    WARNING,
    S_ERROR,
    MACHINE_HOMING
};

enum class MachineEvent {
    //State
    IDLE,
    HOMING,
    HOME_SUCCESS,
    SOFTWARE_OFF,

    //Servo
    X_SERVO_ON,
    Y_SERVO_ON,
    Z_SERVO_ON,

    //Button trigger
    START_BTN,
    STOP_BTN,
    RESET_BTN,
};

enum class MachineWarning {
    X_SOFT_LIMIT_HIT,
    Y_SOFT_LIMIT_HIT,
    Z_SOFT_LIMIT_HIT
};

enum class MachineError {
    //=> Machine System: DIA
    ESTOP_PRESSED,//
    ESTOP_RELAY_FAULT,//e-stop safety relay not OK
    CURTAIN_RELAY_FAULT,//curtain sensor safety relay not OK
    TROLLEY_GUARD_OPEN,//trolley lock guard switch not on

    //=> Gantry
    //External from flow notify
    INITIALIZATION_TIMEOUT,//x

    X_SERVO_OFF,//
    X_HOMING_TIMEOUT,//x
    X_MOVE_TIMEOUT,//x

    Y_SERVO_OFF,//
    Y_HOMING_TIMEOUT,//x
    Y_MOVE_TIMEOUT,//x

    Z_SERVO_OFF,//
    Z_HOMING_TIMEOUT,//x
    Z_MOVE_TIMEOUT,//x

    //Axis state
    X_POSITIVE_LIMIT_HIT,//b
    X_NEGATIVE_LIMIT_HIT,//b
    X_DRIVER_ALARM,

    Y_POSITIVE_LIMIT_HIT,//b
    Y_NEGATIVE_LIMIT_HIT,//b
    Y_DRIVER_ALARM,

    Z_POSITIVE_LIMIT_HIT,//b
    Z_NEGATIVE_LIMIT_HIT,//b
    Z_DRIVER_ALARM,//

    //Vision
    FIDUCIAL_FAIL,
    BARCODE_FAIL,

    COUNT
};

Q_DECLARE_METATYPE(MachineState)
Q_DECLARE_METATYPE(MachineWarning)
Q_DECLARE_METATYPE(MachineError)
Q_DECLARE_METATYPE(MachineEvent)
Q_DECLARE_METATYPE(DOA)

class MachineController : public QThread
{
    Q_OBJECT

public:

    static MachineController& instance();

    void run() override;
    void release();

    //When false every entry point below is inert. Must be set false whenever the
    //controller is not activated, or the state machine half-runs without run().
    void enable(bool enable);

    bool turnOnBrake();
    void silenceBuzzer(); //drive Y106 low - inert when offline, callable from any thread
    bool safelyReleaseBrake(int servoWaitMs = 3000);
    bool servoOnAllAxes(); //servo X/Y/Z on, wait for SVON, clear servo errors, release Z brake

    /*
    * Full drive recovery behind the physical reset button: brake, clear the drives' latched
    * alarms, then servoOnAllAxes(). The alarm reset is the step that used to force the operator
    * onto the Motion page - set_servo() will not take on a drive that is still alarmed, so
    * without it no number of reset presses brings the machine back.
    */
    bool recoverDrives();

    /*
    * True when at least one error is active and EVERY active one is a hard limit hit.
    *
    * That combination is special because it is self-inflicted and self-curable: the axis is
    * parked on a switch, and the only way off is to move. Blocking motion there is a deadlock -
    * motion disabled, so the axis cannot leave the switch, so the error never clears. The drive
    * itself refuses to travel further into a tripped limit (ADLINK built-in, confirmed with CS
    * Tan), so letting software move at all is bounded by hardware in the dangerous direction.
    */
    bool limitRecoveryOnly() const;

    bool resetAlarm();

    /*
    * A Reset pressed on screen. Recovers nothing itself - it raises a flag that handleDIA()
    * consumes as a synthetic X102 edge, so the on-screen button takes the SAME path as the
    * panel button: same safety guards, same recoverDrives(), same deferred resetAlarm(). Two
    * reasons not to call recoverDrives() from the GUI thread instead: it blocks for seconds
    * (the RDY wait alone is bounded at 3 s), and it would race the poll thread that owns the
    * drives and the digital output group.
    *
    * Returns false when the controller is not enabled (machine offline), so a caller can say
    * so rather than leave a button waiting on a reply that is never coming.
    */
    bool requestReset();

    bool curtainTripped() const { return m_curtainTripped; } //latched curtain break, cleared by reset
    bool pauseStatePolling(bool pause); //park the state poll loop (for motion reconnect)
    void notifyEvent(MachineEvent e);
    void notifyWarning(MachineWarning w);
    void notifyError(MachineError e);

    MachineState getMachineState();
    bool isServoOn(Axis axis);

    bool setBypassAxis(Axis axis, bool bypass);
    bool isBypassAxis(Axis axis);

    bool setBypassError(MachineError error, bool bypass);
    bool isBypassError(MachineError error);

    void trackTime(QString key);
    long long logTime(QString key);

    QSet <int> getErrorStatus();

public slots:
    // Thread-safe event injection
    //Q_INVOKABLE void postEvent(MachineEvent e);

    void startRedTowerLight();
    void stopRedTowerLight();

signals:
    void signalMachineWarning(MachineWarning warning);
    void signalMachineError(MachineError error);
    void signalMachineState(MachineState state);
    void signalMachineEvent(MachineEvent e);
    void signalTowerLightOn(DOA doa);
    void signalPromptMsg(QString msg);
    void signalLogTime(QString key, long long ms);

private:
    explicit MachineController(QObject* parent = nullptr);
    Q_DISABLE_COPY(MachineController)

    struct AxisState {
        bool alarm = false;
        bool positive_limit = false;
        bool negative_limit = false;
        std::atomic<bool> servo_on = true;
    };

    AxisState m_x, m_y, m_z;

    bool m_enable = true;
    std::atomic<bool> m_running = true;
    std::atomic<bool> m_pausePolling = false;
    std::atomic<bool> m_pollingParked = false;
    std::atomic<std::chrono::steady_clock::time_point> m_lastZMotionTime;
    std::mutex m_mutex;
    std::thread m_stateThread;

    QString m_motionID = "motion1";
    MachineEvent m_currentEvent = MachineEvent::IDLE;
    MachineState m_currentState = MachineState::STARTUP;
    MachineState m_readyState = MachineState::NOT_READY; //only tracks ready state

    //Bypasses
    QHash<int, bool> m_bypassAxes;
    QHash<int, bool> m_bypassErrors;
    bool m_bypassInterlock = false; //set from config\interlock.json at startup

    QSet<int> m_errorStatuses;

    //Kept in step with m_errorStatuses so limitRecoveryOnly() can be answered from another
    //thread (JobThread asks before homing) without walking a set the poll loop is mutating.
    //assessError() is the only writer of all three.
    std::atomic<int> m_limitErrorCount{0};
    std::atomic<int> m_nonLimitErrorCount{0};

    bool m_startBtnPressed = false;
    bool m_stopBtnPressed = false;
    bool m_resetBtnPressed = false;
    //Set by the reset button, acted on at the END of handleAxisState so resetAlarm() is judged
    //against an error set that has been refreshed this cycle rather than the previous one.
    bool m_resetRequested = false;

    /*
    * An on-screen reset waiting for handleDIA() to pick it up. Timestamped because
    * poolStates() skips handleDIA() entirely while polling is parked for a motion reconnect,
    * and while the card is unavailable - an untimed flag would fire the moment polling
    * resumed, so a click the operator had already given up on could release the brake
    * seconds later. The panel button cannot do this: its edge is only ever read live.
    */
    std::atomic<bool> m_virtualResetRequested{ false };
    std::atomic<std::chrono::steady_clock::time_point> m_virtualResetAt;

    /*
    * Set when handleDIA() has already told the operator, in words, WHY a reset was refused - so
    * the deferred resetAlarm() a few microseconds later does not put a second dialog on top of
    * it saying the same thing less usefully.
    *
    * It has to be suppression rather than two prompts, because showMsg() reuses one shared
    * QMessageBox and calls setWindowFlags() on it, which HIDES an already-visible box: the
    * second prompt therefore kills the first mid-exec() instead of queueing behind it.
    *
    * Read-and-cleared at the top of resetAlarm() so a suppression can never linger onto the
    * Motion page's own call.
    */
    std::atomic<bool> m_resetReasonPrompted{ false };

    //Both e-stop buttons (X103/X104), NC so high = not pressed. Read in handleDIA but
    //assessed in handleAxisState together with the drive EMG input, because all three
    //feed the one ESTOP_PRESSED code and separate assessError() calls would cancel
    //each other out. Defaults true so a DI read failure cannot invent an e-stop.
    bool m_estopButtonsOk = true;
    bool m_curtainTripped = false; //latched when the curtain relay drops; reset re-servos X/Y/Z
    bool m_trolleyGuardOn = false; //last trolley guard DI state - the OFF->ON edge auto-locks
    bool m_limitWasHit = false;    //a soft/hard limit raised the current error - self-clears off the switch
    QTimer* m_redTowerTimer = nullptr;

    //Cleared SYNCHRONOUSLY by setTowerLight() before it writes a new colour. stopRedTowerLight()
    //is a QUEUED call and cannot run until the blink lambda returns - and that lambda holds its
    //own thread for a full second - so the timer alone cannot stop the blink in time.
    std::atomic<bool> m_blinkActive{false};

    //Time
    QHash<QString, std::chrono::time_point<std::chrono::system_clock>> m_timer;

    void poolStates();
    void handleDIA();
    void handleDIB();
    void handleDOA();
    void handleDOB();
    void handleAxisState();

    void assessError(bool good, MachineError e); //true = good, false = error

    void setMachineState(MachineState state);

    void setTowerLight(DOA towerLight);
};
