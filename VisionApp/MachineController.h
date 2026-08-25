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
    bool safelyReleaseBrake(int servoWaitMs = 3000);

    bool resetAlarm();
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

    bool m_startBtnPressed = false;
    bool m_stopBtnPressed = false;
    bool m_resetBtnPressed = false;
    QTimer* m_redTowerTimer = nullptr;

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
