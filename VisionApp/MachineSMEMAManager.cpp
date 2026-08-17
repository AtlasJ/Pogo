#include "MachineSMEMAManager.h"
#include "MotionController.h"
#include "MachineController.h"
#include "SystemData.h"
#include "Logger.h"

MachineSMEMAManager& MachineSMEMAManager::instance()
{
    static MachineSMEMAManager instance;
    return instance;
}

void MachineSMEMAManager::update()
{
    if (!SystemData::instance()._enableSMEMA) return;

    switch (m_state)
    {
        case SmemaState::IDLE:              handleIdle(); break;
        case SmemaState::WAIT_UPSTREAM:     handleWaitUpstream(); break;
        case SmemaState::BOARD_ENTERING:    handleBoardEntering(); break;
        case SmemaState::INSPECTING:        handleInspecting(); break;
        case SmemaState::BYPASS_INSPECTION: handleBypass(); break;
        case SmemaState::WAIT_DOWNSTREAM:   handleWaitDownstream(); break;
        case SmemaState::BOARD_LEAVING:     handleBoardLeaving(); break;
    }
}

void MachineSMEMAManager::changeState(SmemaState newState)
{
    if (m_state == newState) return;

    m_state = newState;
    m_stateStart = std::chrono::steady_clock::now();

    ct::logger::info("[SMEMA] State -> %d", (int)newState);
}

void MachineSMEMAManager::setLoadingDirection(int index)
{
    // 0 = normal (Entry -> Exit)
    // 1 = reverse (Exit -> Entry)
    m_loadingDirection = index;
}

bool MachineSMEMAManager::entrySensor() const
{
    return (m_loadingDirection == 0) 
        ? SystemData::instance()._Entry_Sensor 
        : SystemData::instance()._Exit_Sensor;
}

bool MachineSMEMAManager::exitSensor() const
{
    return (m_loadingDirection == 0)
        ? SystemData::instance()._Exit_Sensor
        : SystemData::instance()._Entry_Sensor;
}

bool MachineSMEMAManager::moveDirection() const
{
    return (m_loadingDirection == 0); 
}

void MachineSMEMAManager::handleIdle() // --> 0
{
    auto& sys = SystemData::instance();
    
    if (SystemData::instance()._Machine_Ready) 
    {
        MotionController::instance().set_DO(m_motionID, 1, (int)DOB::UPSTREAM, false);
        MotionController::instance().set_DO(m_motionID, 1, (int)DOB::DOWNSTREAM, false);
        MotionController::instance().stop_move(m_motionID, (int)Axis::CY1);

        sys._Inspection_Done = false;
        sys._Production_Running = false;

        changeState(SmemaState::WAIT_UPSTREAM);
    }
}

void MachineSMEMAManager::handleWaitUpstream() // --> 1
{
    // Machine ready && Upstream ready && Entry sensor OFF
    auto& sys = SystemData::instance();

    if (sys._Upstream_Ready && sys._Machine_Ready && !entrySensor())
    {
        MotionController::instance().set_DO(m_motionID, 1, (int)DOB::UPSTREAM, true);
        MotionController::instance().continuous_move(m_motionID, (int)Axis::CY1, moveDirection());

        changeState(SmemaState::BOARD_ENTERING);
    }
}

void MachineSMEMAManager::handleBoardEntering() // --> 2
{
    auto& sys = SystemData::instance();

    if (entrySensor())
    {
        if (sys._bypassInspection) {
            changeState(SmemaState::BYPASS_INSPECTION);
        }
        else {
            changeState(SmemaState::INSPECTING);
            if (!SystemData::instance()._Production_Running) {
                SystemData::instance()._Production_Running = true;
                emit triggerStartProduction();
            }
        }
        return;
    }

    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - m_stateStart).count();

    if (elapsed > 15000)
    {
        //ct::logger::error("Board Entering Timeout...");
        MachineController::instance().notifyError(MachineError::ENTRY_SENSOR_TIMEOUT);
        changeState(SmemaState::IDLE);
    }

}

void MachineSMEMAManager::handleInspecting() // --> 3
{
    auto& sys = SystemData::instance();

    if (!entrySensor() && !sys._Machine_Ready)
    {
        MotionController::instance().set_DO(m_motionID, 1, (int)DOB::UPSTREAM, false);
    }

    if (sys._Inspection_Done)
    {
        changeState(SmemaState::WAIT_DOWNSTREAM);
    }
}

void MachineSMEMAManager::handleBypass() // --> 4
{
    auto& sys = SystemData::instance();

    sys._Inspection_Done = true;
    changeState(SmemaState::WAIT_DOWNSTREAM);
}

void MachineSMEMAManager::handleWaitDownstream() // --> 5
{
    //PCB available && Exit sensor ON && Downstream ready
    auto& sys = SystemData::instance();

    if (exitSensor())
    {
        MotionController::instance().set_DO(m_motionID, 1, (int)DOB::DOWNSTREAM, true);
    }

    if (sys._Downstream_Ready && exitSensor())
    {
        MotionController::instance().continuous_move(m_motionID, (int)Axis::CY1, moveDirection());
        changeState(SmemaState::BOARD_LEAVING);
    }

}

void MachineSMEMAManager::handleBoardLeaving()  // --> 6
{
    auto& sys = SystemData::instance();

    if (!exitSensor())
    {
        MotionController::instance().stop_move(m_motionID, (int)Axis::CY1);
        MotionController::instance().set_DO(m_motionID, 1, (int)DOB::DOWNSTREAM, false);

        sys._Inspection_Done = false;
        sys._Machine_Ready = true;
        sys._IsBoardEntry = false;

        changeState(SmemaState::IDLE);
    }
}

