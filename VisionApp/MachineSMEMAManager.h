#pragma once
#include <QObject>
#include <chrono>

enum class SmemaState
{
    IDLE,
    WAIT_UPSTREAM,
    BOARD_ENTERING,
    INSPECTING,
    BYPASS_INSPECTION,
    WAIT_DOWNSTREAM,
    BOARD_LEAVING
};

class MachineSMEMAManager : public QObject
{
    Q_OBJECT

public:
    static MachineSMEMAManager& instance();

    void update();    // Call this in timer loop (ex: every 10ms)
    void changeState(SmemaState newState);
    
    void setLoadingDirection(int index);
    bool entrySensor() const;
    bool exitSensor() const;
    bool moveDirection() const;

private:
    MachineSMEMAManager() = default;

    SmemaState m_state = SmemaState::IDLE;

    int m_loadingDirection = 0;
    QString m_motionID = "motion1";

    std::chrono::steady_clock::time_point m_stateStart;
    
    void handleIdle();
    void handleWaitUpstream();
    void handleBoardEntering();
    void handleInspecting();
    void handleBypass();
    void handleWaitDownstream();
    void handleBoardLeaving();

signals:
    void triggerStartProduction();

};
