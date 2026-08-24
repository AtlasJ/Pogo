#include "CAM_HIK.h"
#include <QDebug>
#include "Logger.h"
#include "TimeLogger.h"
#include <QDateTime>
#include "Utilities.h"
#include "SystemData.h"
#include "MbufPoolManager.h"

extern TMessageQue<FrameInfo> g_imageQueue;

void __stdcall CAM_HIK::FrameCallback(unsigned char* pData,
    MV_FRAME_OUT_INFO_EX* pFrameInfo,
    void* pUser)
{
    TimeLogger timer;
    QDateTime callbackTS = QDateTime::currentDateTime();

    CAM_HIK* instance = reinterpret_cast<CAM_HIK*>(pUser);

    if (!instance)
    {
        ct::logger::error("[CAM_HIK] Camera instance invalid");
        return;
    }

    if (!pData || !pFrameInfo)
    {
        ct::logger::error("[CAM_HIK] Callback frame data invalid");
        instance->m_conditionVariable.notify_one();
        return;
    }

    auto& frame = instance->m_frameInfo;
    frame.width = static_cast<int>(pFrameInfo->nWidth);
    frame.height = static_cast<int>(pFrameInfo->nHeight);
    frame.bufferSize = static_cast<int>(pFrameInfo->nFrameLen);
    frame.timeStamp = pFrameInfo->nDevTimeStampHigh; // or combine high+low
    frame.pixelFormat = ICAM_pixelFormat::Unknown;

    SystemData::instance()._index++;

    int bit = 8;
    int channel = 1;

    switch (pFrameInfo->enPixelType)
    {
    case PixelType_Gvsp_Mono8:
        frame.pixelFormat = ICAM_pixelFormat::Mono8;
        break;

    case PixelType_Gvsp_RGB8_Packed:
        frame.pixelFormat = ICAM_pixelFormat::RGB8;
        channel = 3;
        break;

    case PixelType_Gvsp_BayerRG8:
        frame.pixelFormat = ICAM_pixelFormat::BayerRG8;
        // Bayer mosaic is single-plane 8-bit. A 3-band buffer here made MbufPut
        // read 3x past the end of the SDK's frame data (crash), and ImageManager's
        // bayer_to_rgb expects a 1-band input anyway.
        channel = 1;
        break;

    case PixelType_Gvsp_BayerGB8:
        frame.pixelFormat = ICAM_pixelFormat::BayerGB8;
        channel = 1; //see BayerRG8 note
        break;

    case PixelType_Gvsp_Mono12:
        frame.pixelFormat = ICAM_pixelFormat::Mono12;
        bit = 16;
        break;

    default:
        frame.pixelFormat = ICAM_pixelFormat::Unknown;
        ct::logger::warn("[CAM_HIK] Unsupported pixel format: %lu", pFrameInfo->enPixelType);
        break;
    }

    // Copy raw data to internal MIL buffer or image storage
    frame.pImage = mtrx::MPM::instance().acquire(frame.width, frame.height, channel , bit + M_UNSIGNED);
	MbufPut(frame.pImage->id(), pData);
    ct::logger::trace("[CAM_HIK] Frame data copied to buffer");

    // Push to queue (thread-safe global or member)
    g_imageQueue.push_back(frame);
    ct::logger::info("[CAM_HIK] Pushed frame to image queue");

    frame = FrameInfo();

    // Reset soft trigger flag and notify waiting thread
    instance->m_softTriggered = false;
    instance->m_conditionVariable.notify_one();

    timer.log_duration("[CAM_HIK] Data processed");
}

CAM_HIK::CAM_HIK()
{
    int ret = MV_CC_Initialize();
    logErrorCode("Initialize SDK fail", ret);
}

CAM_HIK::~CAM_HIK()
{
    disconnect();
    
    int ret = MV_CC_Finalize();
    logErrorCode("Finalize SDK fail", ret);
}

bool CAM_HIK::accessible() const {
    if (!m_enable) {
        ct::logger::warn("[CAM_HIK] Trying to access a disabled camera: %s", m_serialNumber.toStdString().c_str());
        return false;
    }

    if (!m_handle) {
        ct::logger::warn("[CAM_HIK] Camera handle not initialized: %s", m_serialNumber.toStdString().c_str());
        return false;
    }

    if (!m_connected) {
        ct::logger::warn("[CAM_HIK] Camera is not connected: %s", m_serialNumber.toStdString().c_str());
        return false;
    }

    return true;
}

const char* CAM_HIK::getStringRetCode(int ret) const
{
    uint32_t ret32 = (uint32_t)ret;

    if (ret32 == MV_OK) return "No error";

    // ===== General Errors =====
    if (ret32 == MV_E_HANDLE)                return "Incorrect or invalid handle";
    if (ret32 == MV_E_SUPPORT)               return "The feature is not supported";
    if (ret32 == MV_E_BUFOVER)               return "The buffer is full";
    if (ret32 == MV_E_CALLORDER)             return "Incorrect calling sequence";
    if (ret32 == MV_E_PARAMETER)             return "Incorrect parameter";
    if (ret32 == MV_E_RESOURCE)              return "Applying for resource failed";
    if (ret32 == MV_E_NODATA)                return "No data";
    if (ret32 == MV_E_PRECONDITION)          return "Precondition incorrect or environment changed";
    if (ret32 == MV_E_VERSION)               return "Version mismatch";
    if (ret32 == MV_E_NOENOUGH_BUF)          return "Input memory is not enough";
    if (ret32 == MV_E_ABNORMAL_IMAGE)        return "Abnormal image (incomplete frame)";
    if (ret32 == MV_E_LOAD_LIBRARY)          return "Dynamically loading DLL failed";
    if (ret32 == MV_E_NOOUTBUF)              return "No buffer for output";
    if (ret32 == MV_E_ENCRYPT)               return "Encryption error";
    if (ret32 == MV_E_OPENFILE)              return "Error opening file";
    if (ret32 == MV_E_BUF_IN_USE)            return "Buffer already in use";
    if (ret32 == MV_E_BUF_INVALID)           return "Invalid buffer address";
    if (ret32 == MV_E_NOALIGN_BUF)           return "Buffer alignment exception";
    if (ret32 == MV_E_NOENOUGH_BUF_NUM)      return "The buffer is full";
    if (ret32 == MV_E_PORT_IN_USE)           return "Serial port occupied";
    if (ret32 == MV_E_IMAGE_DECODEC)         return "Image decoding/verification error";
    if (ret32 == MV_E_UINT32_LIMIT)          return "Image size exceeds unsigned int limit";
    if (ret32 == MV_E_IMAGE_HEIGHT)          return "Image height exception (incomplete frame dropped)";
    if (ret32 == MV_E_NOENOUGH_DDR)          return "Insufficient DDR buffer";
    if (ret32 == MV_E_NOENOUGH_STREAM)       return "Insufficient stream channel";
    if (ret32 == MV_E_UNKNOW)                return "Unknown error";

    // ===== GenICam Layer =====
    if (ret32 == MV_E_GC_GENERIC)            return "Generic GenICam error";
    if (ret32 == MV_E_GC_ARGUMENT)           return "Invalid GenICam parameter";
    if (ret32 == MV_E_GC_RANGE)              return "GenICam value out of range";
    if (ret32 == MV_E_GC_PROPERTY)           return "GenICam property error";
    if (ret32 == MV_E_GC_RUNTIME)            return "Incorrect GenICam runtime environment";
    if (ret32 == MV_E_GC_LOGICAL)            return "GenICam logic error";
    if (ret32 == MV_E_GC_ACCESS)             return "Incorrect node access condition";
    if (ret32 == MV_E_GC_TIMEOUT)            return "GenICam operation timed out";
    if (ret32 == MV_E_GC_DYNAMICCAST)        return "GenICam dynamic cast error";
    if (ret32 == MV_E_GC_UNKNOW)             return "Unknown GenICam error";

    // ===== Transport Layer / Network =====
    if (ret32 == MV_E_NOT_IMPLEMENTED)       return "Unsupported command";
    if (ret32 == MV_E_INVALID_ADDRESS)       return "Target address does not exist";
    if (ret32 == MV_E_WRITE_PROTECT)         return "Target address is write-protected";
    if (ret32 == MV_E_ACCESS_DENIED)         return "No access permission for the device";
    if (ret32 == MV_E_BUSY)                  return "Device busy or network disconnected";
    if (ret32 == MV_E_PACKET)                return "Network packet data error";
    if (ret32 == MV_E_NETER)                 return "Network error";
    if (ret32 == MV_E_SUPPORT_MODIFY_DEVICE_IP) return "Changing IP mode not supported under fixed IP";
    if (ret32 == MV_E_KEY_VERIFICATION)      return "Key verification error";
    if (ret32 == MV_E_IP_CONFLICT)           return "Device IP conflict";

    // ===== USB Layer =====
    if (ret32 == MV_E_USB_READ)              return "Error reading from USB device";
    if (ret32 == MV_E_USB_WRITE)             return "Error writing to USB device";
    if (ret32 == MV_E_USB_DEVICE)            return "USB device exception";
    if (ret32 == MV_E_USB_GENICAM)           return "USB GenICam error";
    if (ret32 == MV_E_USB_BANDWIDTH)         return "USB bandwidth insufficient";
    if (ret32 == MV_E_USB_DRIVER)            return "USB driver mismatch or not installed";
    if (ret32 == MV_E_USB_UNKNOW)            return "Unknown USB error";

    // ===== Firmware Upgrade =====
    if (ret32 == MV_E_UPG_FILE_MISMATCH)     return "Upgrade firmware mismatch";
    if (ret32 == MV_E_UPG_LANGUSGE_MISMATCH) return "Upgrade language mismatch";
    if (ret32 == MV_E_UPG_CONFLICT)          return "Upgrade conflict (duplicate request)";
    if (ret32 == MV_E_UPG_INNER_ERR)         return "Internal device error during upgrade";
    if (ret32 == MV_E_UPG_UNKNOW)            return "Unknown upgrade error";

    // ===== Exceptions =====
    if (ret32 == MV_EXCEPTION_DEV_DISCONNECT) return "Device disconnected";
    if (ret32 == MV_EXCEPTION_VERSION_CHECK)  return "SDK and driver version mismatch";

    // ===== Default =====
    return "Unknown error code";
}

bool CAM_HIK::logErrorCode(const char* msg, int ret)
{
    if (ret == MV_OK) return false;
    ct::logger::error("[CAM_HIK] %s: %s", msg, getStringRetCode(ret));
    return true;
}

void CAM_HIK::setupStandardParams()
{
    if (!accessible()) return;

    //0: Single frame, 2: Continuous
    int ret = MV_CC_SetEnumValue(m_handle, "AcquisitionMode", 2);
    if (logErrorCode("Failed to set acquisition mode", ret)) return;

    //0: Off, 1: On
    ret = MV_CC_SetEnumValue(m_handle, "TriggerMode", 1);
    if (logErrorCode("Failed to set trigger mode", ret)) return;

    //0: Line0, 2: Line2, 4: Counter, 7: Software, 21: LinkTrigger0, 25: Anyway
    ret = MV_CC_SetEnumValue(m_handle, "TriggerSource", 7);
    if (logErrorCode("Failed to set trigger source", ret)) return;

    ret = MV_CC_SetEnumValue(m_handle, "LineSelector", 2);
    if (logErrorCode("Failed to set line selector", ret)) return;

    ret = MV_CC_SetIntValueEx(m_handle, "LineDebouncerTime", 50);
    if (logErrorCode("Failed to set line debouncer time", ret)) return;

    ret = MV_CC_SetFloatValue(m_handle, "ExposureTime", 10000.0);
    if (logErrorCode("Failed to set exposure", ret)) return;
}

bool CAM_HIK::connect(QString sn)
{
    disconnect();

    int ret = MV_OK;
    m_serialNumber = sn;

    // 1️ Enumerate all interfaces (frame grabbers)
    MV_INTERFACE_INFO_LIST interfaceList = { 0 };
    ret = MV_CC_EnumInterfaces(MV_CXP_INTERFACE, &interfaceList);
    if (logErrorCode("Failed to enumerate interfaces", ret)) return false;
    
    // 2️ For each interface, open and enumerate devices
    for (unsigned int i = 0; i < interfaceList.nInterfaceNum; ++i) {

        MV_INTERFACE_INFO* iface = interfaceList.pInterfaceInfos[i];
        if (!iface) continue;

        ret = MV_CC_CreateInterface(&m_interface, iface);
        if (logErrorCode("Failed to create interface", ret)) continue;

        ret = MV_CC_OpenInterface(m_interface, nullptr);
        if (logErrorCode("Failed to open interface", ret)) {
            MV_CC_DestroyInterface(m_interface);
            continue;
        }

        MV_CC_DEVICE_INFO_LIST devList = { 0 };
        ret = MV_CC_EnumDevicesByInterface(m_interface, &devList);
        if (ret != MV_OK || devList.nDeviceNum == 0) {
            logErrorCode("Failed to enumerate devices by interface", ret);
            MV_CC_CloseInterface(m_interface);
            MV_CC_DestroyInterface(m_interface);
            continue;
        }

        // 3️ Search for matching serial number
        for (unsigned int j = 0; j < devList.nDeviceNum; ++j)
        {
            MV_CC_DEVICE_INFO* info = devList.pDeviceInfo[j];
            QString devSN;

            switch (info->nTLayerType)
            {
            case MV_GIGE_DEVICE:
            case MV_GENTL_GIGE_DEVICE:
                devSN = QString::fromLatin1((char*)info->SpecialInfo.stGigEInfo.chSerialNumber);
                break;
            case MV_GENTL_CXP_DEVICE:
                devSN = QString::fromLatin1((char*)info->SpecialInfo.stCXPInfo.chSerialNumber);
                break;
            case MV_GENTL_CAMERALINK_DEVICE:
                devSN = QString::fromLatin1((char*)info->SpecialInfo.stCMLInfo.chSerialNumber);
                break;
            default:
                break;
            }

            if (devSN == sn) {
                // Found matching camera
                ret = MV_CC_CreateHandle(&m_handle, info);
                if (logErrorCode("Failed to create handle", ret)) break;

                ret = MV_CC_OpenDevice(m_handle);
                if (logErrorCode("Failed to open device", ret)) {
                    MV_CC_DestroyHandle(m_handle);
                    m_handle = nullptr;
                    break;
                }

                m_connected = true;
                m_name = QString::fromLatin1((char*)info->SpecialInfo.stGigEInfo.chModelName);

                setupStandardParams();
                //MV_CC_CloseInterface(hInterface);
                //MV_CC_DestroyInterface(hInterface);

                ct::logger::info("[CAM_HIK] Camera connected: %s", sn.toStdString().c_str());
                return true;
            }
        }

        // 4️ Clean up this interface before next
        //MV_CC_CloseInterface(hInterface);
        //MV_CC_DestroyInterface(hInterface);
    }

    ct::logger::error("[CAM_HIK] Camera with serial %s not found", sn.toStdString().c_str());
    return false;
}

bool CAM_HIK::disconnect()
{
    if (!m_connected) return true;

    if (!accessible()) return false;

    if (m_grabbing) stopGrab();

    if (m_handle) {
        MV_CC_CloseDevice(m_handle);
        MV_CC_DestroyHandle(m_handle);
    }

    if (m_interface) {
        MV_CC_CloseInterface(m_interface);
        MV_CC_DestroyInterface(m_interface);
    }

    m_handle = nullptr;
    m_interface = nullptr;
    m_connected = false;
    return true;
}

bool CAM_HIK::startGrab()
{
    if (!accessible()) return false;
    
    int ret = MV_CC_RegisterImageCallBackEx(m_handle, CAM_HIK::FrameCallback, this);
    if (logErrorCode("Failed to register callback", ret)) return false;

    ret = MV_CC_StartGrabbing(m_handle);
    if (logErrorCode("Failed to start grabbing", ret)) return false;

    ret = MV_CC_SetCommandValue(m_handle, "AcquisitionStart");
    if (logErrorCode("Failed to start acquisition", ret)) return false;

    m_grabbing = true;
    m_softTriggered = false;

    return true;
}

bool CAM_HIK::stopGrab()
{
    if (!accessible()) return false;

    int ret = MV_CC_StopGrabbing(m_handle);
    if (logErrorCode("Failed to stop grabbing", ret)) return false;

    m_grabbing = false;
    return true;
}

bool CAM_HIK::setExposure(double exposure)
{
    if (!accessible()) return false;
    //stopGrab();
    int ret = MV_CC_SetFloatValue(m_handle, "ExposureTime", exposure);
   // startGrab();
    if (logErrorCode("Failed to set exposure", ret)) return false;
    return true;
}

bool CAM_HIK::setGain(double gain)
{
    if (!accessible()) return false;
    int ret = MV_CC_SetFloatValue(m_handle, "Gain", gain);
    if (logErrorCode("Failed to set exposure", ret)) return false;
    return true;
}

const double CAM_HIK::getExposure() const
{
    if (!accessible()) return 0.0;

    MVCC_FLOATVALUE val;
    if (MV_CC_GetFloatValue(m_handle, "ExposureTime", &val) == MV_OK)
        return val.fCurValue;
    return 0.0;
}

const double CAM_HIK::getGain() const
{
    if (!accessible()) return 0.0;

    MVCC_FLOATVALUE val;
    if (MV_CC_GetFloatValue(m_handle, "Gain", &val) == MV_OK)
        return val.fCurValue;
    return 0.0;
}

const int CAM_HIK::getWidth() const
{
    if (!accessible()) return 0;

    MVCC_INTVALUE val;
    if (MV_CC_GetIntValue(m_handle, "Width", &val) == MV_OK)
        return val.nCurValue;
    return 0;
}

const int CAM_HIK::getHeight() const
{
    if (!accessible()) return 0;

    MVCC_INTVALUE val;
    if (MV_CC_GetIntValue(m_handle, "Height", &val) == MV_OK)
        return val.nCurValue;
    return 0;
}

const int CAM_HIK::getChannel() const
{
    if (!accessible()) return 0;

    MVCC_ENUMVALUE val;
    if (MV_CC_GetEnumValue(m_handle, "PixelFormat", &val) != MV_OK)
        return 0;

    switch (val.nCurValue)
    {
    case PixelType_Gvsp_Mono8:
    case PixelType_Gvsp_Mono10:
    case PixelType_Gvsp_Mono12:
        return 1;  // grayscale

    case PixelType_Gvsp_BayerRG8:
    case PixelType_Gvsp_BayerGB8:
    case PixelType_Gvsp_BayerBG8:
    case PixelType_Gvsp_BayerGR8:
        return 1;  // raw Bayer still one channel

    case PixelType_Gvsp_RGB8_Packed:
    case PixelType_Gvsp_BGR8_Packed:
    case PixelType_Gvsp_YUV422_Packed:
        return 3;  // color

    default:
        return 1;  // fallback to 1 if unknown
    }
}

bool CAM_HIK::softTrigger()
{
    if (!accessible()) return false;

    if (m_softTriggered) {
        ct::logger::warn("[CAM_HIK] Camera busy, failed to trigger snap!");
        return false;
    }

    int ret = MV_CC_SetCommandValue(m_handle, "TriggerSoftware");
    if (logErrorCode("Failed to soft trigger", ret)) return false;

    m_softTriggered = true;

    return true;
}

bool CAM_HIK::waitAcquisition(int ms)
{
    if (!accessible()) return false;

    //To handle cases where soft trigger has been called, and return. But wait was call too late.
    if (!m_softTriggered) return true;

    std::unique_lock<std::mutex> lock(m_mutex);

    if (m_conditionVariable.wait_for(lock, std::chrono::milliseconds(ms)) == std::cv_status::no_timeout) return true;

    m_softTriggered = false;

    return false;
}

bool CAM_HIK::setTriggerOutput(QString line, QString source)
{
    ScopedTimeLogger stl("Set trigger output");
    auto ret = MV_CC_SetEnumValue(m_handle, "LineSelector", 1); //24 - 34: INOUT (0 - 10)
    logErrorCode("Failed to line select", ret);
    ret = MV_CC_SetEnumValue(m_handle, "LineMode", 1); //0: Input, 1: Output
    logErrorCode("Failed to line mode", ret);

    //255: Off, 48-51: TimerActive 0-3, 64: High, 65: Low
    //80-90 : D485Input 0-10, 96-99: Advanced IO 0-3, 112: Fval
    ret = MV_CC_SetEnumValue(m_handle, "LineSource", source.toInt()); 
    logErrorCode("Failed to line source", ret);

    MVCC_ENUMVALUE w;
    ret = MV_CC_GetEnumValue(m_handle, "LineSelector", &w);
    logErrorCode("Failed to line source", ret);
    ct::logger::info("Line Select: %d", w);
    return true;

    if (!accessible()) return false;
    // Example: line="Line0", source="ExposureActive"
    QString cmd = QString("LineSelector=%1;LineSource=%2").arg(line).arg(source);
    return MV_CC_SetEnumValueByString(m_handle, "Line Selector", line.toStdString().c_str()) == MV_OK &&
        MV_CC_SetEnumValueByString(m_handle, "Line Mode", source.toStdString().c_str()) == MV_OK;
}

bool CAM_HIK::setDO(int DO, bool on)
{
    return false;
}

bool CAM_HIK::loadConfig(QString path)
{
    return true;
    if (!accessible()) return false;

    MVCC_NODE_ERROR_LIST errorList;

   // auto ret = MV_CC_FeatureLoadEx(m_handle, path.toLocal8Bit(), &errorList);
    auto ret = MV_CC_FeatureLoad(m_handle, "C:\\Advanced\\Data\\config\\cam1.mcfg");

    QString errormsg = QString("Failed to load config (%1)").arg(path);
    if (logErrorCode(errormsg.toStdString().c_str(), ret)) return false;

    return true;
}

QString CAM_HIK::errorMsg()
{
    return m_errorMsg;
}

const FrameInfo& CAM_HIK::frame() const
{
    return m_frameInfo;
}

FrameInfo& CAM_HIK::frame()
{
    return m_frameInfo;
}

void CAM_HIK::resetFrame()
{
    m_frameInfo = FrameInfo();
}

bool CAM_HIK::isConnected() const
{
    return m_connected;
}

const bool CAM_HIK::isGrabbing() const
{
    return m_grabbing;
}

bool CAM_HIK::enable(bool enable)
{
    m_enable = enable;
    return true;
}

const QString& CAM_HIK::getName() const
{
    return m_name;
}

const QString& CAM_HIK::getSerialNumber() const
{
    return m_serialNumber;
}