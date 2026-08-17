#include "Profiler_SmartRay.h"
#include "Logger.h"
#include "mtrx.h"
#include "MessageQue.h"
#include <string>

extern TMessageQue<FrameInfo> g_imageQueue;

std::string getErrorMessage(int errorCode) {
    switch (errorCode) {
    case SUCCESS: return "Successful.";
    case ERR_SR_API_SRSENSOR_NULLPOINTER: return "Sensor object is nullpointer.";
    case ERR_SR_API_ARG_NULLPOINTER: return "Input parameter is nullpointer.";
    case ERR_SR_API_MEMORY_ALLOCATION_FAILED: return "Memory allocation failed.";
    case ERR_SR_API_FILE_READ_FAILED: return "Cannot read file.";
    case ERR_SR_API_FUNCTION_NOT_SUCCESSFUL: return "Function not successful.";
    case ERR_SR_API_FILE_WRITE_FAILED: return "Cannot write file.";
    case ERR_SR_API_INVALID_FILENAME_OR_PATH: return "Invalid filename or path.";
    case ERR_SR_API_INVALID_FILETYPE: return "Invalid file type.";
    case ERR_SR_API_FIRMWARE_NOT_COMPATIBLE_WITH_SENSOR_SERIES: return "Loaded firmware file not compatible with sensor.";
    case ERR_SR_API_FUNCTION_NOT_AVAILABLE: return "Function not available.";
    case ERR_SR_API_MISSING_LICENSE: return "Missing License.";
    case ERR_SR_API_ALREADY_INITALIZED: return "API already initialized.";
    case ERR_SR_API_SENSOR_NUMBER_OUT_OF_RANGE: return "Maximum number of sensors reached.";
    case ERR_SR_API_SENSOR_ACTIVE: return "Sensor busy.";
    case ERR_SR_API_SENSOR_NOT_ACTIVE: return "Sensor not active.";
    case ERR_SR_API_SENSOR_NOT_DISCONNECTED: return "Sensor could not be disconnected.";
    case ERR_SR_API_SENSOR_NOT_CONNECTED: return "Sensor not connected.";
    case ERR_SR_API_NR_USERCB_MAX: return "Maximum callback function reached for this callback type.";
    case ERR_SR_API_NOT_VALID_CB_TYPE: return "Invalid callback type.";
    case ERR_SR_API_NO_MPAR_AVAILABLE: return "Machine parameter not available.";
    case ERR_SR_API_NO_SYSPAR_AVAILABLE: return "System parameter not available.";
    case ERR_SR_API_IP_OR_PORT_IS_ZERO: return "IP address or port number is zero.";
    case ERR_SR_API_IP_IS_INVALID: return "New IP address invalid (e.g., 255.255.255.255).";
    case ERR_SR_API_WIDTH_IS_ZERO: return "ROI-width is zero.";
    case ERR_SR_API_HEIGHT_IS_ZERO: return "ROI-height is zero.";
    case ERR_SR_API_ORIGIN_X_RANGE_ERROR: return "ROI-OriginX is not between 0 and 1920.";
    case ERR_SR_API_ORIGIN_Y_RANGE_ERROR: return "ROI-OriginY is not between 0 and 1200.";
    case ERR_SR_API_NEGATIVE_SENSOR_INDEX_NOT_ALLOWED: return "Sensor ID parameter is negative.";
    case ERR_SR_API_SENSOR_INDEX_GREATER_THAN_MAX_SENSOR_SUPPORTED: return "Sensor ID parameter is greater than the maximum sensors supported by API.";
    case ERR_SR_API_VALUES_OUT_OF_RANGE: return "Values not in range.";
    case ERR_SR_API_CALIBRATION_FILE_NOT_COMPATIBLE_WITH_SENSOR_SERIES: return "Calibration file not compatible with sensor.";
    case ERR_SR_API_PARAMAETER_SET_NOT_INITIALIZED: return "Parameter set not found.";
    case ERR_SR_API_CALIBRATION_FILE_NOT_PRESENT_ON_SENSOR: return "Calibration file not stored on sensor.";

    case ERR_SR_API_ALG_GENERAL: return "ALG: General error.";
    case ERR_SR_API_ALG_FILE_INPUT: return "ALG: Reading file failed.";
    case ERR_SR_API_ALG_FILE_OUTPUT: return "ALG: Writing file failed.";
    case ERR_SR_API_ALG_ASSERTION: return "ALG: An assertion failed.";
    case ERR_SR_API_ALG_DATA_TYPE_NOT_SUPPORTED: return "ALG: Data type not supported.";
    case ERR_SR_API_ALG_INVALID_PARAMETER_VALUE: return "ALG: Parameter value is invalid.";
    case ERR_SR_API_ALG_DIVISION_BY_ZERO: return "ALG: Division by zero.";
    case ERR_SR_API_ALG_NO_SOLUTION_FOUND: return "ALG: No solution found.";
    case ERR_SR_API_ALG_INVALID_DATA_SIZE: return "ALG: Invalid size of data.";
    case ERR_SR_API_ALG_INFORMATION_MISSING: return "ALG: Information is missing.";
    case ERR_SR_API_ALG_INDEX_OUT_OF_BOUNDS: return "ALG: Index is out of bounds.";
    case ERR_SR_API_ALG_INVALID_ID: return "ALG: ID is invalid.";
    case ERR_SR_API_ALG_INVALID_KEY: return "ALG: Key is invalid.";
    case ERR_SR_API_ALG_DATA_SIZE_TOO_LARGE: return "ALG: Data size is too large.";

    case ERR_SR_API_IPADDRESS_INVALID: return "IP address invalid.";
    case ERR_SR_API_PORTNUMBER_INVALID: return "Port number invalid. Default: 40.";

    case ERR_SR_API_CONNECTION_TIMEOUT: return "Cannot connect to sensor, connection timeout reached.";
    case ERR_SR_API_CONNECTION_TIMEOUT_INVALID: return "Connection timeout invalid.";
    case ERR_SR_API_PART_NUMBER_INVALID: return "Sensor part number invalid.";
    case ERR_SR_API_SENSOR_ALREADY_CONNECTED: return "Sensor already connected.";

    case ERR_LOG_FAILED_TO_CREATE_CONSOLESINK: return "Failed to create console sink.";
    case ERR_LOG_FAILED_TO_CREATE_FILESINK: return "Failed to create file sink.";
    case ERR_LOG_FAILED_TO_ADD_SINK: return "Failed to add sink.";

    default: return "Unknown error code.";
    }
}

Profiler_SmartRay::Profiler_SmartRay() 
    : sensorManager(false)
{
    //sensorManager.attach(&m_frameInfo);
}

//Query
const double Profiler_SmartRay::getExposure() const 
{
    if (!safeGuard()) return false;
    //int32_t status = master->getExposureTime(*m_exposure) ;
    //const bool isExposureget = (SUCCESS <= status);

    //if (!isExposureget)
    //{
    //    ct::logger::error("[Profiler_SmartRay] Failed to get exposure: %s", getErrorMessage(status).c_str());
    //}
    return *m_exposure;
}


const double Profiler_SmartRay::getYResolution() const 
{
    return YResolution;
}

const QString& Profiler_SmartRay::getFirmwareVersion() const
{
    if (!safeGuard()) return "INVALID";
    //const int32_t  getSensorFirmware = master->getFirmware(m_firmware);
    //const bool isFirmwareget = (SUCCESS <= getSensorFirmware);

    //if (!isFirmwareget)
    //{
    //    ct::logger::error("[Profiler_SmartRay] Failed to get exposure: %s", getErrorMessage(getSensorFirmware).c_str());
    //}
    return m_firmware;
}

const QString& Profiler_SmartRay::getSerialNumber() const
{
    if (!safeGuard()) return "INVALID";
    //const int32_t  getSerialNumber = master->getSensorSerialNumber(m_serial);
    //const bool isSerialNumberget = (SUCCESS <= getSerialNumber);
    //if (!isSerialNumberget)
    //{
    //    ct::logger::error("[Profiler_SmartRay] Failed to get sensor serial number: %s", getErrorMessage(getSerialNumber).c_str());
    //}
    //else
    //{
    //    ct::logger::info("[Profiler_SmartRay] Sensor Serial Number: %s", m_serial);
    //}
    //return m_serial;
}

bool Profiler_SmartRay::isConnected() const
{
    if (!safeGuard()) return false;
    if (!m_connectionStatus)
    {
        ct::logger::info("[Profiler_SmartRay] Sensor not connected");
    }
    else
    {
        ct::logger::info("[Profiler_SmartRay] Sensor is connected");
    }
    return (m_connectionStatus);
}

const bool Profiler_SmartRay::isGrabbing() const
{
    if (!safeGuard()) return false;
    master->isCapturing();
    slave->isCapturing();
}

//Connection
bool Profiler_SmartRay::enable(bool enable)
{
    m_enable = enable;
    return true;
}

bool Profiler_SmartRay::connect(QString ip)
{
    //if (!safeGuard()) return false;
    QString ip2 = incrementIPAddress(ip);
    ct::logger::info("Sensor 1 IP: %s", ip.toStdString());
    ct::logger::info("Sensor 2 IP: %s", ip2.toStdString());

    master = sensorManager.CreateSensor("master", ip.toStdString().c_str());
    slave = sensorManager.CreateSensor("slave", ip2.toStdString().c_str());

    const int32_t  connectStatus = master->Connect();
    const int32_t  connectStatus2 = slave->Connect();
    const bool     isSensorConnected = (SUCCESS <= connectStatus);
    const bool     isSensor2Connected = (SUCCESS <= connectStatus2);

    if (!isSensorConnected)
    {
        ct::logger::error("[Profiler_SmartRay] Failed to connect to master sensor: %s", getErrorMessage(connectStatus).c_str());
    }

    if (!isSensor2Connected)
    {
        ct::logger::error("[Profiler_SmartRay] Failed to connect to slave sensor: %s", getErrorMessage(connectStatus2).c_str());
    }

    m_connectionStatus = isSensorConnected && isSensor2Connected;
   loadCalibration();
    return m_connectionStatus;
};

bool Profiler_SmartRay::disconnect() {
    if (!safeGuard()) return false;
    if (master.get() != nullptr && slave.get() != nullptr)
    {
        ct::logger::info("[Profiler_SmartRay] Disconnecting Sensor");
    }

    const int32_t disconnectStatus = master->Disconnect();
    const int32_t disconnectStatus2 = slave->Disconnect();
    const bool    isSensorDisconnected = (SUCCESS <= disconnectStatus);
    const bool    isSensor2Disconnected = (SUCCESS <= disconnectStatus2);
    if (!isSensorDisconnected)
    {
        ct::logger::error("[Profiler_SmartRay] Failed to get disconnect master sensor: %s", getErrorMessage(disconnectStatus).c_str());
    }

    if (!isSensor2Disconnected)
    {
        ct::logger::error("[Profiler_SmartRay] Failed to get disconnect slave sensor: %s", getErrorMessage(disconnectStatus2).c_str());
    }

    return true;
}

//Acquisition
bool Profiler_SmartRay::start() {

    if (snapshotFlag) {
        ct::logger::info("[Profiler_SmartRay] Reverting to normal mode from snapshot");
        setDataTriggerMode(DataTriggerMode_External);
        setDataTriggerSource(DataTriggerSource_QuadEncoder);
        setDataTriggerParam(m_divider,0,TriggerEdgeMode_Both);
        snapshotFlag = false;
    }
    if (!safeGuard()) return false;
    getDataTrigger();
    getTransportResolution();
    ct::logger::info("[Profiler_SmartRay] Starting acquisition");


    const int32_t startStatus = master->StartAcquisition();
    const int32_t startStatus2 = slave->StartAcquisition();
    const bool    isStarted = (SUCCESS <= startStatus);
    const bool    isStarted2 =(SUCCESS <= startStatus2);

    if (!isStarted)
    {
        ct::logger::error("[Profiler_SmartRay] Failed to start acquisiation for master sensor: %s", getErrorMessage(startStatus).c_str());
    }

    if (!isStarted2)
    {
        ct::logger::error("[Profiler_SmartRay] Failed to start acquisiation for slave sensor: %s", getErrorMessage(startStatus2).c_str());
    }


    //master->GetLastImageData()->SaveZilImage("zmap");
    m_softTriggered = true;
    //m_conditionVariable.notify_one();
    return true;

}

bool Profiler_SmartRay::stop() {
    if (!safeGuard()) return false;
    ct::logger::info("[Profiler_SmartRay] Stopping acquisation");
    const int32_t stopStatus = master->StopAcquisition();
    const int32_t stopStatus2 = slave->StopAcquisition();
    const bool    isStopped = (SUCCESS <= stopStatus);
    const bool    isStopped2 = (SUCCESS <= stopStatus2);

    if (!isStopped)
    {
        ct::logger::error("[Profiler_SmartRay] Failed to stop acquisiation for master sensor: %s", getErrorMessage(stopStatus).c_str());
        return false;
    }
    if (!isStopped2)
    {
        ct::logger::error("[Profiler_SmartRay] Failed to start acquisiation for slave sensor: %s", getErrorMessage(stopStatus2).c_str());
        return false;
    }
    else
    {
        ct::logger::info("[Profiler_SmartRay] Stopped Acquisition");
    }
    return true;

}

bool Profiler_SmartRay::snapShot()
{
    ct::logger::info("[Profiler_SmartRay] Starting Snapshot");

    snapshotFlag = true;
    setDataTriggerMode(DataTriggerMode_FreeRunning);
    setScanLength(0.004 * m_divider);
    if (!safeGuard()) return false;
    startSnapshot();
    m_softTriggered = true;
    waitAcquisitionSnapshot(2000);
    stop();
    return 1;
}

bool Profiler_SmartRay::enableIntensityMap(bool enable)
{
    if (!safeGuard()) return false;
    m_enableImap = enable;
    if (enable) {
        const int32_t setTypeStatus = master->setImageAcquisitionType(ImageAquisitionType_ZMapIntensityLaserLineThickness);
        const int32_t setTypeStatus2 = slave->setImageAcquisitionType(ImageAquisitionType_ZMapIntensityLaserLineThickness);
        const bool    isTypeSet = (SUCCESS <= setTypeStatus);
        const bool    isTypeSet2 = (SUCCESS <= setTypeStatus2);
        if (!isTypeSet)
        {
            ct::logger::error("[Profiler_SmartRay] Failed to set image acquisisation for master sensor: %s", getErrorMessage(setTypeStatus).c_str());
            return false;
        }
        if (!isTypeSet2)
        {
            ct::logger::error("[Profiler_SmartRay] Failed to set image acquisisation for slave sensor: %s", getErrorMessage(setTypeStatus2).c_str());
            return false;
        }
    }
    else
    {
        const int32_t setTypeStatus = master->setImageAcquisitionType(ImageAquisitionType_ZMap);
        const int32_t setTypeStatus2 = slave->setImageAcquisitionType(ImageAquisitionType_ZMap);
        const bool    isTypeSet = (SUCCESS <= setTypeStatus);
        const bool    isTypeSet2 = (SUCCESS <= setTypeStatus2);
        if (!isTypeSet)
        {
            ct::logger::error("[Profiler_SmartRay] Failed to set image acquisisation for master sensor: %s", getErrorMessage(setTypeStatus).c_str());
            return false;
        }
        if (!isTypeSet2)
        {
            ct::logger::error("[Profiler_SmartRay] Failed to set image acquisisation for slave sensor: %s", getErrorMessage(setTypeStatus2).c_str());
            return false;
        }
    }
    return true;
}


bool Profiler_SmartRay::setScanLength(double mm)
{
    ct::logger::info("[Profiler_SmartRay] Set scan length (mm): %.2f", mm);
    double reso = 0.004 * m_divider;
    if (!safeGuard()) return false;
    int realmm = mm /reso;
    //master->SetTransportResolution(reso);
    //slave->SetTransportResolution(reso);
    master->SetZmapResolution(YResolution, ZResolution);
    slave->SetZmapResolution(YResolution, ZResolution);
    const int32_t  setScanLength = master->setScanlength(realmm);
    const int32_t  setScanLength2 = slave->setScanlength(realmm);
    const bool     isScanLengthset = (SUCCESS <= setScanLength);
    const bool     isScanLengthset2 = (SUCCESS <= setScanLength2);
    if (!isScanLengthset)
    {
        ct::logger::error("[Profiler_SmartRay] Failed to set scan length for master sensor: %s", getErrorMessage(setScanLength).c_str());
        return false;
    }
    if (!isScanLengthset2)
    {
        ct::logger::error("[Profiler_SmartRay] Failed to set scan length for slave sensor: %s", getErrorMessage(setScanLength2).c_str());
        return false;
    }
    else
    {
        ct::logger::info("[Profiler_SmartRay] Sensor has set profile length (px): %d", realmm);
    }

    return true;
}

bool Profiler_SmartRay::setExposure(double us)
{
    ct::logger::info("[Profiler_SmartRay] Running set exposure");
    if (!safeGuard()) return false;
    const int32_t  setExposure = master->ConfigureExposureTimeMicroS(us);
    const int32_t  setExposure2 = slave->ConfigureExposureTimeMicroS(us);
    const bool     isExposureSet = (SUCCESS <= setExposure);
    const bool     isExposureSet2 = (SUCCESS <= setExposure2);
    if (!isExposureSet)
    {
        ct::logger::error("[Profiler_SmartRay] Failed to set exposure for master sensor: %s", getErrorMessage(setExposure).c_str());
        return false;
    }
    if (!isExposureSet2)
    {
        ct::logger::error("[Profiler_SmartRay] Failed to set exposure for slave sensor: %s", getErrorMessage(setExposure2).c_str());
        return false;
    }
    else
    {
        ct::logger::info("[Profiler_SmartRay] Sensor has set exposure: %f", us);
    }
    ct::logger::info("[Profiler_SmartRay] Finish running set exposure");
    return true;
}

bool Profiler_SmartRay::setMultiExposure(double us, double us2)// need extra set gain
{
    ct::logger::info("[Profiler_SmartRay] Running set multi exposure");
    if (!safeGuard()) return false;
    const int32_t  setExposure = master->ConfigureDoubleExposureTimesMicroS(us,us2);
    const int32_t  setExposure2 = slave->ConfigureDoubleExposureTimesMicroS(us,us2);
    const bool     isExposureSet = (SUCCESS <= setExposure);
    const bool     isExposureSet2 = (SUCCESS <= setExposure2);
    if (!isExposureSet)
    {
        ct::logger::error("[Profiler_SmartRay] Failed to set exposure for master sensor: %s", getErrorMessage(setExposure).c_str());
        return false;
    }
    if (!isExposureSet2)
    {
        ct::logger::error("[Profiler_SmartRay] Failed to set exposure for slave sensor: %s", getErrorMessage(setExposure2).c_str());
        return false;
    }
    else
    {
        ct::logger::info("[Profiler_SmartRay] Sensor has set exposure: %f", us);
    }
    ct::logger::info("[Profiler_SmartRay] Finish running set exposure");
    return true;
}

bool Profiler_SmartRay::setDynamicExposure(double min_us, double max_us)
{
    //does not support
    return true;
}

bool Profiler_SmartRay::setParallelExposure(double us, double us2)
{
    ct::logger::info("[Profiler_SmartRay] Running set exposure");
    if (!safeGuard()) return false;
    const int32_t  setExposure = master->ConfigureExposureTimeMicroS(us);
    const int32_t  setExposure2 = slave->ConfigureExposureTimeMicroS(us2);
    const bool     isExposureSet = (SUCCESS <= setExposure);
    const bool     isExposureSet2 = (SUCCESS <= setExposure2);
    if (!isExposureSet)
    {
        ct::logger::error("[Profiler_SmartRay] Failed to set exposure for master sensor: %s", getErrorMessage(setExposure).c_str());
        return false;
    }
    if (!isExposureSet2)
    {
        ct::logger::error("[Profiler_SmartRay] Failed to set exposure for slave sensor: %s", getErrorMessage(setExposure2).c_str());
        return false;
    }
    else
    {
        ct::logger::info("[Profiler_SmartRay] Sensor has set exposure: %f, %f", us, us2);
    }

    ct::logger::info("[Profiler_SmartRay] Finish running set exposure");
    return true;
}

bool Profiler_SmartRay::setGain(double gain)
{
    ct::logger::info("[Profiler_SmartRay] Running set gain");
    if (!safeGuard()) return false;
 
    const int32_t  setGain = master->ConfigureGain(gain);
    const int32_t  setGain2 = slave->ConfigureGain(gain);
    const bool     isGainSet = (SUCCESS <= setGain);
    const bool     isGainSet2 = (SUCCESS <= setGain2);
    if (!isGainSet)
    {
        ct::logger::error("[Profiler_SmartRay] Failed to set gain for master sensor: %s", getErrorMessage(setGain).c_str());
        return false;
    }
    if (!isGainSet2)
    {
        ct::logger::error("[Profiler_SmartRay] Failed to set gain for slave sensor: %s", getErrorMessage(setGain2).c_str());
        return false;
    }
    else
    {
        ct::logger::info("[Profiler_SmartRay] Sensor has set gain: %f", gain);
    }
    ct::logger::info("[Profiler_SmartRay] Finish running set gain");
    return true;
}

bool Profiler_SmartRay::setDuoHeadGain(double gain, double gain2)
{
    ct::logger::info("[Profiler_SmartRay] Running set duo head gain");
    if (!safeGuard()) return false;

    const int32_t  setGain = master->ConfigureGain(gain);
    const int32_t  setGain2 = slave->ConfigureGain(gain2);
    const bool     isGainSet = (SUCCESS <= setGain);
    const bool     isGainSet2 = (SUCCESS <= setGain2);
    if (!isGainSet)
    {
        ct::logger::error("[Profiler_SmartRay] Failed to set gain for master sensor: %s", getErrorMessage(setGain).c_str());
        return false;
    }
    if (!isGainSet2)
    {
        ct::logger::error("[Profiler_SmartRay] Failed to set gain for slave sensor: %s", getErrorMessage(setGain2).c_str());
        return false;
    }
    else
    {
        ct::logger::info("[Profiler_SmartRay] Sensor has set gain: %f", gain);
    }
    ct::logger::info("[Profiler_SmartRay] Finish running set gain");
    return true;
}

bool Profiler_SmartRay::setDivider(int divider)
{
    ct::logger::info("[Profiler_SmartRay] Running set divider %d", divider);
    if (!safeGuard()) return false;

    m_divider = divider;
    setDataTriggerMode(DataTriggerMode_External);
    setDataTriggerSource(DataTriggerSource_QuadEncoder);
    setDataTriggerParam(m_divider, 0, TriggerEdgeMode_Both);
   

}

bool Profiler_SmartRay::setExposureMode(ExposureMode mode)
{
    //does not support
    m_mode = mode;
    return true;
}

bool Profiler_SmartRay::setMSR(bool enable)
{
    if (!safeGuard()) return false;
    m_setMSRmode = enable;

    if (!m_setMSRmode)
    {
        ct::logger::warn("[Profiler_SmartRay] MSR mode disabled, using Single-Head mode");
        const int32_t disableStatus = master->DisableMSRMode();
        const bool    isDisabled = (SUCCESS <= disableStatus);
        if (!isDisabled)
        {
            ct::logger::error("[Profiler_SmartRay] Failed Enabling Single-Head mode: %s", getErrorMessage(disableStatus).c_str());
            return false;
        }
        else
        {
            ct::logger::info("[Profiler_SmartRay] Done Enabling Single-Head mode");
            return true;
        }
    }
    ct::logger::info("[Profiler_SmartRay] Enabling MSR mode");
    const int32_t enableStatus = master->ConfigureMSRMode("C:/Advanced/Data/config/transformation.xml", 0.0063, YResolution, ZResolution);
    // havent added resolution
    const bool    isEnabled = (SUCCESS <= enableStatus);
    if (!isEnabled)
    {
        ct::logger::error("[Profiler_SmartRay] Failed Enabling MSR mode: %s", getErrorMessage(enableStatus).c_str());
        m_setMSRmode = false;
        return false;
    }
    else
    {
        ct::logger::info("[Profiler_SmartRay] Done Enabling MSR mode");
        return true;
    }
}

bool Profiler_SmartRay::setLaserLineThreshold(double threshold)
{
  
    if (!safeGuard()) return false;

    bool suc = true;
    ct::logger::info("[Profiler_SmartRay] Configuring Laser Line Threshold");

    const int32_t loadStatus = master->set3DLaserLineThreshold(0, threshold);
    ct::logger::info("[Profiler_SmartRay] Configured Laser Line Threshold for sensor 1");
    const int32_t loadStatus2 = slave->set3DLaserLineThreshold(0, threshold);
    ct::logger::info("[Profiler_SmartRay] Configured Laser Line Threshold for sensor 2");
    const bool isLoaded = (SUCCESS <= loadStatus);
    const bool isLoaded2 = (SUCCESS <= loadStatus2);
    if (!isLoaded)
    {
        suc = false;
        ct::logger::error("[Profiler_SmartRay] Failed to configure Laser Line Threshold for sensor 1: %s", getErrorMessage(loadStatus).c_str());
    }
    if (!isLoaded2)
    {
        suc = false;
        ct::logger::error("[Profiler_SmartRay] Failed to configure Laser Line Threshold for sensor 2: %s", getErrorMessage(loadStatus2).c_str());
    }
    else
    {
        suc = true;
        ct::logger::info("[Profiler_SmartRay] Succesfully configured sensor 1 & 2 Laser Line Threshold");
    }

    return suc;
}


bool Profiler_SmartRay::waitAcquisition(int ms)
{
    if (!m_softTriggered) {

        ct::logger::info("[Profiler_SmartRay]m_softTriggered = false");
        return true;
    }
    //std::unique_lock<std::mutex> lock(m_mutex);

    //if (m_conditionVariable.wait_for(lock, std::chrono::milliseconds(ms)) == std::cv_status::no_timeout) return true;
    ct::logger::info("[Profiler_SmartRay] Wait for image acquisition");
    if (!master->WaitForAcquisitionCycle(1, ms))
    {
        ct::logger::error("[Profiler_SmartRay] Image acquisition timeout or error has occured");
        return false;
    }
    else
    {
            ct::logger::info("[Profiler_SmartRay] Done Image acquistion now saving data");
            //master->GetLastImageData()->SaveZilImage("zmap");
            master->GetLastImageData()->SaveData(m_frameInfo, m_enableImap,m_setMSRmode, m_divider);
            ct::logger::info("[Profiler_SmartRay]  Saved data");
            //master->GetLastImageData()->SaveZilImage("zmap");

            QString e1, e2;
            e1 = m_frameInfo.opticID;

            if (m_mode == ExposureMode::PARALLEL) {
                m_frameInfo.baseOpticID = m_frameInfo.opticID;
                e1 = QString("%1E1").arg(m_frameInfo.opticID);
                e2 = QString("%1E2").arg(m_frameInfo.opticID);
            }
            else {
                m_frameInfo.baseOpticID.clear();
            }

            m_frameInfo.opticID = e1;
            g_imageQueue.push_back(m_frameInfo);

            if (m_mode == ExposureMode::PARALLEL) {
                slave->GetLastImageData()->SaveData(m_frameInfo, false, false, m_divider);
                m_frameInfo.opticID = e2;
                g_imageQueue.push_back(m_frameInfo);
                ct::logger::info("[Profiler_SmartRay]  Saved slave data");
            }


            m_softTriggered = false;
            return true;

    }


}

bool Profiler_SmartRay::waitAcquisitionSnapshot(int ms)
{
    if (!m_softTriggered) {

        ct::logger::info("[Profiler_SmartRay]m_softTriggered = false");
        return true;
    }
    //std::unique_lock<std::mutex> lock(m_mutex);

    //if (m_conditionVariable.wait_for(lock, std::chrono::milliseconds(ms)) == std::cv_status::no_timeout) return true;
    ct::logger::info("[Profiler_SmartRay] Wait for image acquisition");
    if (!master->WaitForAcquisitionCycle(1, ms))
    {
        ct::logger::error("[Profiler_SmartRay] Image acquisition timeout or error has occured");
        return false;
    }
    else
    {
        ct::logger::info("[Profiler_SmartRay] Done Image acquistion now saving data");
        //master->GetLastImageData()->SaveZilImage("zmap");
        //master->GetLastImageData()->SaveZilImage("snapshot");
        master->GetLastImageData()->SaveSnapshot(m_frameInfo);
        ct::logger::info("[Profiler_SmartRay]  Saved data");
        //master->GetLastImageData()->SaveZilImage("zmap");
        //g_imageQueue.push_back(m_frameInfo);
        m_softTriggered = false;
        return true;
    }


}


//Data
const FrameInfo& Profiler_SmartRay::getFrame() const
{
    return m_frameInfo;
}

FrameInfo& Profiler_SmartRay::getFrame()
{
    return m_frameInfo;
}

void Profiler_SmartRay::resetFrame()
{
    m_frameInfo = FrameInfo();
}

bool Profiler_SmartRay::loadConfig(QString path)
{
    if (!safeGuard()) return false;
    ct::logger::info("[Profiler_SmartRay] Loading config file");

    const int32_t loadStatus = master->LoadParameterSet(path.toUtf8().constData());//tostdstring.c_string
    ct::logger::info("[Profiler_SmartRay] Loaded sensor 1 config file");
    const int32_t loadStatus2 = slave->LoadParameterSet(path.toUtf8().constData());
    ct::logger::info("[Profiler_SmartRay] Loaded sensor 2 config file");
    const bool isLoaded = (SUCCESS <= loadStatus);
    const bool isLoaded2 = (SUCCESS <= loadStatus2);
    if (!isLoaded)
    {
        ct::logger::error("[Profiler_SmartRay] Failed to load config for master sensor: %s", getErrorMessage(loadStatus).c_str());
    }
    if (!isLoaded2)
    {
        ct::logger::error("[Profiler_SmartRay] Failed to load config for slave sensor: %s", getErrorMessage(loadStatus2).c_str());
    }
    else
    {
        ct::logger::info("[Profiler_SmartRay] Succesfully loaded config file");
    }
    enableIntensityMap(1);
    setLaserLineThreshold();
    setGain(3);

    sendConfig(); // laod studio set config file 

    setDataTriggerMode(DataTriggerMode_External);
    setDataTriggerSource(DataTriggerSource_QuadEncoder);
    setDataTriggerParam(m_divider, 0, TriggerEdgeMode_Both);
    return true;
}

bool Profiler_SmartRay::sendConfig() {

    if (!safeGuard()) return false;
    ct::logger::info("[Profiler_SmartRay] Sending config file");

    const int32_t sendStatus = master->SendParameterSet();
    ct::logger::info("[Profiler_SmartRay] Sent sensor 1 config file");
    const int32_t sendStatus2 = slave->SendParameterSet();
    ct::logger::info("[Profiler_SmartRay] Sent sensor 2 config file");
    const bool isSended = (SUCCESS <= sendStatus);
    const bool isSended2 = (SUCCESS <= sendStatus2);

    if (!isSended)
    {
        ct::logger::error("[Profiler_SmartRay] Failed to send config for master sensor: %s", getErrorMessage(sendStatus).c_str());
    }
    if (!isSended2)
    {
        ct::logger::error("[Profiler_SmartRay] Failed to send config for slave sensor: %s", getErrorMessage(sendStatus2).c_str());
    }
    else
    {
        ct::logger::info("[Profiler_SmartRay] Succesfully sent config file");
    }
}

QString Profiler_SmartRay::errorMsg()
{
    return m_errorMsg;
}

bool Profiler_SmartRay::safeGuard() const
{
    if (!m_enable) {
        ct::logger::warn("Profiler_SmartRay not enabled");
        return false;
    }
    if (master == NULL || slave == NULL) {

        ct::logger::error("[Profiler_SmartRay] Pointer = NULL: %s");
        return false;
    }
    return true;
}
  
QString Profiler_SmartRay::incrementIPAddress(const QString& ip) {
    std::string ipStr = ip.toStdString();
    std::vector<int> parts;
    std::stringstream ss(ipStr);
    std::string item;


    while (std::getline(ss, item, '.')) {
        parts.push_back(std::stoi(item));
    }


    if (parts.size() != 4) {
        return ip;
        ct::logger::error("[Profiler_SmartRay] Failed to process 2nd ip: %s");
    }


    if (++parts[3] > 255) {
        parts[3] = 0;
        if (++parts[2] > 255) {
            parts[2] = 0;
            if (++parts[1] > 255) {
                parts[1] = 0;
                if (++parts[0] > 255) {
                    return ip;
                }
            }
        }
    }

    return QString::number(parts[0]) + "." +
        QString::number(parts[1]) + "." +
        QString::number(parts[2]) + "." +
        QString::number(parts[3]);
}



bool Profiler_SmartRay::loadCalibration() 
{
    if (!safeGuard()) return false;
    ct::logger::info("[Profiler_SmartRay] Loading calibration file");

    const int32_t loadStatus = master->LoadCalibrationDataFromSensor();
    ct::logger::info("[Profiler_SmartRay] Loaded sensor 1 calibration file");
    const int32_t loadStatus2 = slave->LoadCalibrationDataFromSensor();
    ct::logger::info("[Profiler_SmartRay] Loaded sensor 2 calibration file");
    const bool isLoaded = (SUCCESS <= loadStatus);
    const bool isLoaded2 = (SUCCESS <= loadStatus2);
    if (!isLoaded)
    {
        ct::logger::error("[Profiler_SmartRay] Failed to load calibration for master sensor: %s", getErrorMessage(loadStatus).c_str());
    }
    if (!isLoaded2)
    {
        ct::logger::error("[Profiler_SmartRay] Failed to load calibration for slave sensor: %s", getErrorMessage(loadStatus2).c_str());
    }
    return 0;
}

bool Profiler_SmartRay::setDefaultAcquisitionMode()
{
    if (!safeGuard()) return false;
    ct::logger::info("[Profiler_SmartRay] Loading Image Aquisition type");

    const int32_t loadStatus = master->Configure3DImageAquisition(ImageAquisitionType_ZMapIntensityLaserLineThickness);
    ct::logger::info("[Profiler_SmartRay] Configured sensor 1 Image Aquisition type");
    const int32_t loadStatus2 = slave->Configure3DImageAquisition(ImageAquisitionType_ZMapIntensityLaserLineThickness);
    ct::logger::info("[Profiler_SmartRay] Configured sensor 2 Image Aquisition type");
    const bool isLoaded = (SUCCESS <= loadStatus);
    const bool isLoaded2 = (SUCCESS <= loadStatus2);
    if (!isLoaded)
    {
        ct::logger::error("[Profiler_SmartRay] Failed to configure sensor 1 Image Aquisition type: %s", getErrorMessage(loadStatus).c_str());
    }
    if (!isLoaded2)
    {
        ct::logger::error("[Profiler_SmartRay] Failed to configure sensor 2 Image Aquisition type: %s", getErrorMessage(loadStatus2).c_str());
    }
    else
    {
        ct::logger::info("[Profiler_SmartRay] Succesfully configured sensor 1 & 2 Image Aquisition type");
    }
}

bool Profiler_SmartRay::setLaserLineThreshold()
{
    if (!safeGuard()) return false;
    ct::logger::info("[Profiler_SmartRay] Configuring Laser Line Threshold");

    const int32_t loadStatus = master->set3DLaserLineThreshold(0,20);
    ct::logger::info("[Profiler_SmartRay] Configured Laser Line Threshold for sensor 1");
    const int32_t loadStatus2 = slave->set3DLaserLineThreshold(0,20);
    ct::logger::info("[Profiler_SmartRay] Configured Laser Line Threshold for sensor 2");
    const bool isLoaded = (SUCCESS <= loadStatus);
    const bool isLoaded2 = (SUCCESS <= loadStatus2);
    if (!isLoaded)
    {
        ct::logger::error("[Profiler_SmartRay] Failed to configure Laser Line Threshold for sensor 1: %s", getErrorMessage(loadStatus).c_str());
    }
    if (!isLoaded2)
    {
        ct::logger::error("[Profiler_SmartRay] Failed to configure Laser Line Threshold for sensor 2: %s", getErrorMessage(loadStatus2).c_str());
    }
    else
    {
        ct::logger::info("[Profiler_SmartRay] Succesfully configured sensor 1 & 2 Laser Line Threshold");
    }

}

bool Profiler_SmartRay::sendParam()
{
    if (!safeGuard()) return false;
    ct::logger::info("[Profiler_SmartRay] Sending Parameter to Sensor");

    const int32_t loadStatus = master->SendParameterSet();
    ct::logger::info("[Profiler_SmartRay] Parameter sent to sensor 1");
    const int32_t loadStatus2 = slave->SendParameterSet();
    ct::logger::info("[Profiler_SmartRay] Parameter sent to sensor 2");
    const bool isLoaded = (SUCCESS <= loadStatus);
    const bool isLoaded2 = (SUCCESS <= loadStatus2);
    if (!isLoaded)
    {
        ct::logger::error("[Profiler_SmartRay] Parameter failed to send to sensor 1: %s", getErrorMessage(loadStatus).c_str());
    }
    if (!isLoaded2)
    {
        ct::logger::error("[Profiler_SmartRay]Parameter failed to send to sensor 2: %s", getErrorMessage(loadStatus2).c_str());
    }
    else
    {
        ct::logger::info("[Profiler_SmartRay] Succesfully sent parameter to sensor 1 & 2");
    }
}

bool Profiler_SmartRay::setDataTriggerMode(DataTriggerMode aMode) 
{
    const int32_t setTriggerModeStatus = master->ConfigureDataTriggerMode(aMode);
    const int32_t setTriggerModeStatus2 = slave->ConfigureDataTriggerMode(aMode);
    const bool    isTriggerModeSet = (SUCCESS <= setTriggerModeStatus);
    const bool    isTriggerModeSet2 = (SUCCESS <= setTriggerModeStatus2);
    if (!isTriggerModeSet)
    {
        ct::logger::error("[Profiler_SmartRay] Failed to set data trigger mode for master sensor: %s", getErrorMessage(setTriggerModeStatus).c_str());
    }
    if (!isTriggerModeSet2)
    {
        ct::logger::error("[Profiler_SmartRay] Failed to set data trigger mode for slave sensor: %s", getErrorMessage(setTriggerModeStatus2).c_str());
    }
    else
    {
        ct::logger::info("[Profiler_SmartRay] Succesfully set data trigger mode to sensor 1 & 2");
    }
    return true;
}

bool Profiler_SmartRay::setDataTriggerParam(int div, int delay, TriggerEdgeMode aParam)
{
    const int32_t setTriggerParamStatus = master->ConfigureExternalDataTriggerParameters(div,delay,aParam);
    const int32_t setTriggerParamStatus2 = slave->ConfigureExternalDataTriggerParameters(div,delay, aParam);
    const bool    isTriggerParamSet = (SUCCESS <= setTriggerParamStatus);
    const bool    isTriggerParamSet2 = (SUCCESS <= setTriggerParamStatus2);
    if (!isTriggerParamSet)
    {
        ct::logger::error("[Profiler_SmartRay] Failed to set data trigger param for master sensor: %s", getErrorMessage(setTriggerParamStatus).c_str());
    }
    if (!isTriggerParamSet2)
    {
        ct::logger::error("[Profiler_SmartRay] Failed to set data trigger param for slave sensor: %s", getErrorMessage(setTriggerParamStatus2).c_str());
    }
    else
    {
        ct::logger::info("[Profiler_SmartRay] Succesfully set data trigger param to sensor 1 & 2, div: %d", div);
    }
    return true;
}

bool Profiler_SmartRay::setDataTriggerSource(DataTriggerSource aSource)
{
    const int32_t setTriggerParamStatus = master->ConfigureExternalDataTriggerSource(aSource);
    const int32_t setTriggerParamStatus2 = slave->ConfigureExternalDataTriggerSource(aSource);
    const bool    isTriggerParamSet = (SUCCESS <= setTriggerParamStatus);
    const bool    isTriggerParamSet2 = (SUCCESS <= setTriggerParamStatus2);
    if (!isTriggerParamSet)
    {
        ct::logger::error("[Profiler_SmartRay] Failed to set data trigger param for master sensor: %s", getErrorMessage(setTriggerParamStatus).c_str());
    }
    if (!isTriggerParamSet2)
    {
        ct::logger::error("[Profiler_SmartRay] Failed to set data trigger param for slave sensor: %s", getErrorMessage(setTriggerParamStatus2).c_str());
    }
    else
    {
        ct::logger::info("[Profiler_SmartRay] Succesfully set data trigger param to sensor 1 & 2");
    }
    return true;
}

bool Profiler_SmartRay::startSnapshot()
{
    ct::logger::info("[Profiler_SmartRay] Starting acquisation");

    const int32_t startStatus = master->StartAcquisition();
    const int32_t startStatus2 = slave->StartAcquisition();
    const bool    isStarted = (SUCCESS <= startStatus);
    const bool    isStarted2 = (SUCCESS <= startStatus2);

    if (!isStarted)
    {
        ct::logger::error("[Profiler_SmartRay] Failed to start acquisiation for master sensor: %s", getErrorMessage(startStatus).c_str());
        return false;
    }

    if (!isStarted2)
    {
        ct::logger::error("[Profiler_SmartRay] Failed to start acquisiation for slave sensor: %s", getErrorMessage(startStatus2).c_str());
        return false;
    }
    return true;
}

bool Profiler_SmartRay::getDataTrigger()
{
    const int32_t getStatus = master->GetDataTriggerParameters();
    const int32_t getStatus2 = slave->GetDataTriggerParameters();
    const bool    getSuccess = (SUCCESS <= getStatus);
    const bool    getSuccess2 = (SUCCESS <= getStatus2);

    if (!getSuccess)
    {
        ct::logger::error("[Profiler_SmartRay] Failed to get DataTrigger from master: %s", getErrorMessage(getStatus).c_str());
        return false;
    }

    if (!getSuccess2)
    {
        ct::logger::error("[Profiler_SmartRay] Failed to get DataTrigger from slave: %s", getErrorMessage(getStatus2).c_str());
        return false;
    }
    else
    {
        ct::logger::info("[Profiler_SmartRay] Succesfully get DataTrigger from sensor 1 & 2");
    }
    return true;
}

bool Profiler_SmartRay::getTransportResolution()
{
    const float transReso = master->GetTransportResolution();
    const float transReso2 = slave->GetTransportResolution();
    ct::logger::info("[Profiler_SmartRay] TransportResolution sensor 1:%f", transReso);
    ct::logger::info("[Profiler_SmartRay] TransportResolution sensor 2:%f", transReso2);


    return true;
}

bool Profiler_SmartRay::getZmapResolution()
{
    float yReso, zReso, yReso2, zReso2;
    master->GetZmapResolution(&yReso,&zReso);
    slave->GetZmapResolution(&yReso2, &zReso2);
    ct::logger::info("[Profiler_SmartRay] ZmapResolution sensor 1:%f");
    ct::logger::info("[Profiler_SmartRay] ZmapResolution sensor 2:%f");
    return true;
}


