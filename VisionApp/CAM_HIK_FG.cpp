#include "CAM_HIK_FG.h"
#include <QDebug>
#include "Logger.h"
#include "TimeLogger.h"
#include <QDateTime>
#include "Utilities.h"
#include "SystemData.h"
#include "MbufPoolManager.h"
#include "MvCameraControl.h"

extern TMessageQue<FrameInfo> g_imageQueue;

void __stdcall CAM_HIK_FG::FrameCallback(
    mvfg::MV_FG_BUFFER_INFO* pFrameInfo,
    void* pUser)
{
    TimeLogger timer;
    QDateTime callbackTS = QDateTime::currentDateTime();

    CAM_HIK_FG* instance = reinterpret_cast<CAM_HIK_FG*>(pUser);

    if (!instance)
    {
        ct::logger::error("[CAM_HIK_FG] Camera instance invalid");
        return;
    }

    if (!pFrameInfo)
    {
        ct::logger::error("[CAM_HIK_FG] Callback frame data invalid");
        instance->m_conditionVariable.notify_one();
        return;
    }

    auto& frame = instance->m_frameInfo;
    frame.width = static_cast<int>(pFrameInfo->nWidth);
    frame.height = static_cast<int>(pFrameInfo->nHeight);
    frame.bufferSize = static_cast<int>(pFrameInfo->nFilledSize);
    frame.timeStamp = pFrameInfo->nDevTimeStamp; // or combine high+low
    frame.pixelFormat = ICAM_pixelFormat::Unknown;

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
        channel = 3;
        break;

    case PixelType_Gvsp_BayerGB8:
        frame.pixelFormat = ICAM_pixelFormat::BayerGB8;
        channel = 3;
        break;

    case PixelType_Gvsp_Mono12:
        frame.pixelFormat = ICAM_pixelFormat::Mono12;
        bit = 16;
        break;

    default:
        frame.pixelFormat = ICAM_pixelFormat::Unknown;
        ct::logger::warn("[CAM_HIK_FG] Unsupported pixel format: %lu", pFrameInfo->enPixelType);
        break;
    }

    // Copy raw data to internal MIL buffer or image storage
    frame.pImage = mtrx::MPM::instance().acquire(frame.width, frame.height, channel, bit + M_UNSIGNED);
    MbufPut(frame.pImage->id(), pFrameInfo->pBuffer);
    ct::logger::trace("[CAM_HIK_FG] Frame data copied to buffer");

    // Push to queue (thread-safe global or member)
    g_imageQueue.push_back(frame);
    ct::logger::info("[CAM_HIK_FG] Pushed frame to image queue");

    frame = FrameInfo();

    // Reset soft trigger flag and notify waiting thread
    instance->m_softTriggered = false;
    instance->m_conditionVariable.notify_one();

    timer.log_duration("[CAM_HIK_FG] Data processed");
}

CAM_HIK_FG::CAM_HIK_FG()
{
    int ret = mvfg::MV_FG_Initialize();
    logErrorCode("Initialize SDK fail", ret);
}

CAM_HIK_FG::~CAM_HIK_FG()
{
    disconnect();

    int ret = mvfg::MV_FG_Finalize();
    logErrorCode("Finalize SDK fail", ret);
}

bool CAM_HIK_FG::accessible() const {
    if (!m_enable) {
        ct::logger::warn("[CAM_HIK_FG] Trying to access a disabled camera: %s", m_serialNumber.toStdString().c_str());
        return false;
    }

    if (!m_handle) {
        ct::logger::warn("[CAM_HIK_FG] Camera handle not initialized: %s", m_serialNumber.toStdString().c_str());
        return false;
    }

    if (!m_connected) {
        ct::logger::warn("[CAM_HIK_FG] Camera is not connected: %s", m_serialNumber.toStdString().c_str());
        return false;
    }

    return true;
}

const char* CAM_HIK_FG::getStringRetCode(int ret) const
{
    uint32_t ret32 = (uint32_t)ret;

    if (ret32 == MV_FG_SUCCESS) return "No error";

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

bool CAM_HIK_FG::logErrorCode(const char* msg, int ret)
{
    if (ret == MV_FG_SUCCESS) return false;
    ct::logger::error("[CAM_HIK_FG] %s: %s", msg, getStringRetCode(ret));
    return true;
}

void CAM_HIK_FG::setupStandardParams()
{
}

bool CAM_HIK_FG::connect(QString sn)
{
    disconnect();

    int ret = MV_FG_SUCCESS;
    m_serialNumber = sn;

    bool changed = false;
    ret = mvfg::MV_FG_UpdateInterfaceList(MV_FG_CXP_INTERFACE | MV_FG_GEV_INTERFACE | MV_FG_CAMERALINK_INTERFACE | MV_FG_XoF_INTERFACE, &changed);
    if (logErrorCode("Failed to enumerate interfaces", ret)) return false;


    unsigned int nInterfaceNum = 0;
    ret = mvfg::MV_FG_GetNumInterfaces(&nInterfaceNum);
    if (logErrorCode("Failed to get number of interfaces", ret)) return false;

    for (unsigned int i = 0; i < nInterfaceNum; i++) {
        mvfg::MV_FG_INTERFACE_INFO stInterfaceInfo = { 0 };

        ret = mvfg::MV_FG_GetInterfaceInfo(i, &stInterfaceInfo);
        if (logErrorCode("Failed to get interface info", ret)) return false;

        auto type = stInterfaceInfo.nTLayerType;
        auto displayName = stInterfaceInfo.IfaceInfo.stCXPIfaceInfo.chDisplayName;
        auto interfaceID = stInterfaceInfo.IfaceInfo.stCXPIfaceInfo.chInterfaceID;
        QString serialNum = QString::fromLatin1((char*)stInterfaceInfo.IfaceInfo.stCXPIfaceInfo.chSerialNumber);

        unsigned int nInterfaceIndex = i;

        if (i == 0) { //HARDCODE: Assume only one framegrabber, multi framegrabber will need to add sn for framegrabber
            // Open frame grabber with specified permission, and get handle
            ret = mvfg::MV_FG_OpenInterface(nInterfaceIndex, &m_interface);
            if (logErrorCode("Failed to open interface info", ret)) return false;

            // Enumerate frame grabbers on camera
            ret = mvfg::MV_FG_UpdateDeviceList(m_interface, &changed);
            if (logErrorCode("Failed to update device list", ret)) return false;

            // Get the number of devices
            unsigned int nDeviceNum = 0;
            ret = mvfg::MV_FG_GetNumDevices(m_interface, &nDeviceNum);
            if (logErrorCode("Failed to get number of device", ret)) return false;

            for (unsigned int j = 0; j < nDeviceNum; j++) {
                mvfg::MV_FG_DEVICE_INFO stDeviceInfo = { 0 };

                ret = mvfg::MV_FG_GetDeviceInfo(m_interface, j, &stDeviceInfo);
                if (logErrorCode("Failed to get device info", ret)) return false;

                auto d_type = stDeviceInfo.nDevType;
                auto d_userDefinedName = stDeviceInfo.DevInfo.stCXPDevInfo.chUserDefinedName;
                auto d_modelName = stDeviceInfo.DevInfo.stCXPDevInfo.chModelName;

                QString d_serialNum = QString::fromLatin1((char*)stDeviceInfo.DevInfo.stCXPDevInfo.chSerialNumber);
                ct::logger::info("CAM SN: %s", d_serialNum.toStdString().c_str());

                if (d_serialNum == m_serialNumber) {
                    // Open device, and get handle
                    ret = mvfg::MV_FG_OpenDevice(m_interface, j, &m_handle);
                    if (logErrorCode("Failed to open device", ret)) return false;

                    // Trigger mode on
                    ret = mvfg::MV_FG_SetEnumValueByString(m_handle, "TriggerMode", "On");
                    if (logErrorCode("Failed to set trigger mode on", ret)) return false;

                        ret = mvfg::MV_FG_SetEnumValueByString(m_handle, "TriggerSource", "Software"); 
                    

                    if (logErrorCode("Failed to set trigger source to software", ret)) return false;

                    setExposure(6666);

                    m_connected = true;

                    return true;
                }
            }
        }
    }

    ct::logger::error("[CAM_HIK_FG] Camera with serial %s not found", sn.toStdString().c_str());
    return false;
}

bool CAM_HIK_FG::disconnect()
{
    if (!m_connected) return true;

    if (!accessible()) return false;

    if (m_grabbing) stopGrab();

    if (m_handle) {
        mvfg::MV_FG_CloseDevice(m_handle);
    }

    if (m_interface) {
        mvfg::MV_FG_CloseInterface(m_interface);
    }

    m_handle = nullptr;
    m_interface = nullptr;
    m_connected = false;
    return true;
}

bool CAM_HIK_FG::startGrab()
{
    if (!accessible()) return false;

    // Get the number of stream channels
    unsigned int nStreamNum = 0;
    auto ret = mvfg::MV_FG_GetNumStreams(m_handle, &nStreamNum);
    if (logErrorCode("Failed to get number of streams", ret)) return false;

    // Open stream channel (only supports a single stream channel currently)
    ret = mvfg::MV_FG_OpenStream(m_handle, 0, &m_stream);
    if (logErrorCode("Failed to open stream", ret)) return false;

    // Set number of internal buffers
    ret = mvfg::MV_FG_SetBufferNum(m_stream, 1);
    if (logErrorCode("Failed to set buffer", ret)) return false;

    // Register frame buffer info callback function
    ret = mvfg::MV_FG_RegisterFrameCallBackEx(m_stream, CAM_HIK_FG::FrameCallback, this, true);
    if (logErrorCode("Failed to register callback", ret)) return false;

    // Start acquisition
    ret = mvfg::MV_FG_StartAcquisition(m_stream);
    if (logErrorCode("Failed to start acquisition", ret)) return false;

    m_grabbing = true;
    m_softTriggered = false;

    return true;
}

bool CAM_HIK_FG::stopGrab()
{
    if (!accessible()) return false;

    int ret = mvfg::MV_FG_StopAcquisition(m_stream);
    if (logErrorCode("Failed to stop acquisition", ret)) return false;

    ret = mvfg::MV_FG_CloseStream(m_stream);
    if (logErrorCode("Failed to close stream", ret)) return false;

    m_grabbing = false;
    return true;
}

bool CAM_HIK_FG::setExposure(double exposure)
{
    if (!accessible()) return false;
    int ret = MV_CC_SetFloatValue(m_interface, "ExposureTime", exposure);
    if (logErrorCode("Failed to set exposure", ret)) return false;
    return true;
}

bool CAM_HIK_FG::setGain(double gain)
{
    if (!accessible()) return false;
    ct::logger::warn("[CAM_HIK_FG] Set gain not supported");
    //int ret = MV_CC_SetFloatValue(m_handle, "Gain", gain);
    //if (logErrorCode("Failed to set exposure", ret)) return false;
    return true;
}

const double CAM_HIK_FG::getExposure() const
{
    if (!accessible()) return 0.0;

    mvfg::MV_FG_FLOATVALUE val;
    if (mvfg::MV_FG_GetFloatValue(m_handle, "ExposureTime", &val) == MV_FG_SUCCESS)
        return val.fCurValue;
    return 0.0;
}

const double CAM_HIK_FG::getGain() const
{
    if (!accessible()) return 0.0;

    mvfg::MV_FG_FLOATVALUE val;
    if (mvfg::MV_FG_GetFloatValue(m_handle, "Gain", &val) == MV_FG_SUCCESS)
        return val.fCurValue;
    return 0.0;
}

const int CAM_HIK_FG::getWidth() const
{
    if (!accessible()) return 0;

   /* MVCC_INTVALUE val;
    if (MV_CC_GetIntValue(m_handle, "Width", &val) == MV_OK)
        return val.nCurValue;
    return 0;*/

    mvfg::MV_FG_INTVALUE val;
    if (mvfg::MV_FG_GetIntValue(m_handle, "Width", &val) == MV_FG_SUCCESS)
        return val.nCurValue;
    return 0;
}

const int CAM_HIK_FG::getHeight() const
{
    if (!accessible()) return 0;

   /* MVCC_INTVALUE val;
    if (MV_CC_GetIntValue(m_handle, "Height", &val) == MV_OK)
        return val.nCurValue;
    return 0;*/

    mvfg::MV_FG_INTVALUE val;
    if (mvfg::MV_FG_GetIntValue(m_handle, "Height", &val) == MV_FG_SUCCESS)
        return val.nCurValue;
    return 0;
}

const int CAM_HIK_FG::getChannel() const
{
    if (!accessible()) return 0;

    mvfg::MV_FG_ENUMVALUE val;
    if (mvfg::MV_FG_GetEnumValue(m_handle, "PixelFormat", &val) != MV_FG_SUCCESS)
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

bool CAM_HIK_FG::softTrigger()
{
    if (!accessible())
    {
        qDebug() << "NOT ACCESIBLE in softtrigger!!!";
        return false;
    }

    if (m_softTriggered) {
        ct::logger::warn("[CAM_HIK_FG] Camera busy, failed to trigger snap!");
        return false;
    }

    int ret = mvfg::MV_FG_SetCommandValue(m_handle, "TriggerSoftware");
    if (logErrorCode("Failed to soft trigger", ret)) return false;

    m_softTriggered = true;

    return true;
}

bool CAM_HIK_FG::waitAcquisition(int ms)
{
    if (!accessible())
    {
        qDebug() << "NOT ACCESIBLE!!!";
        return false;
    }

    //To handle cases where soft trigger has been called, and return. But wait was call too late.
    if (!m_softTriggered)
    {
        qDebug() << "Soft Triggered";
        return true;
    }

    std::unique_lock<std::mutex> lock(m_mutex);

    if (m_conditionVariable.wait_for(lock, std::chrono::milliseconds(ms)) == std::cv_status::no_timeout) return true;

    m_softTriggered = false;

    return false;
}

bool CAM_HIK_FG::setTriggerOutput(QString line, QString source)
{
    ScopedTimeLogger stl("Set trigger output");
    mvfg::MV_FG_SetEnumValueByString(m_handle, "TriggerSource", source.toStdString().c_str());

    //auto ret = mvfg::MV_FG_SetEnumValue(m_interface, "LineSelector", 25); //24 - 34: INOUT (0 - 10)
    //logErrorCode("Failed to line select", ret);

    //ret = mvfg::MV_FG_SetEnumValue(m_interface, "LineMode", 1); //0: Input, 1: Output
    //logErrorCode("Failed to line mode", ret);

    ////255: Off, 48-51: TimerActive 0-3, 64: High, 65: Low
    ////80-90 : D485Input 0-10, 96-99: Advanced IO 0-3, 112: Fval
    //ret = mvfg::MV_FG_SetEnumValue(m_interface, "LineSource", source.toInt());
    //logErrorCode("Failed to line source", ret);
    return true;
}

bool CAM_HIK_FG::setDO(int DO, bool on)
{
    if (!accessible()) return false;

    auto ret = mvfg::MV_FG_SetEnumValue(m_interface, "LineSelector", DO + 24); //24 - 34: INOUT (0 - 10)
    if (logErrorCode("Failed to set line selector", ret)) return false;

    ret = mvfg::MV_FG_SetEnumValue(m_interface, "LineMode", 1); //0: Input, 1: Output
    if (logErrorCode("Failed to set line mode", ret)) return false;

    //255: Off, 48-51: TimerActive 0-3, 64: High, 65: Low
    //80-90 : D485Input 0-10, 96-99: Advanced IO 0-3, 112: Fval
    ret = mvfg::MV_FG_SetEnumValue(m_interface, "LineSource", on ? 65 : 64);
    if (logErrorCode("Failed to set DO", ret)) return false;

    return true;
}

bool CAM_HIK_FG::loadConfig(QString path)
{
    return true;
    if (!accessible()) return false;

    auto ret = mvfg::MV_FG_FeatureLoad(m_interface, path.toStdString().c_str());

    QString errormsg = QString("Failed to load config (%1)").arg(path);
    if (logErrorCode(errormsg.toStdString().c_str(), ret)) return false;

    return true;
}

QString CAM_HIK_FG::errorMsg()
{
    return m_errorMsg;
}

const FrameInfo& CAM_HIK_FG::frame() const
{
    return m_frameInfo;
}

FrameInfo& CAM_HIK_FG::frame()
{
    return m_frameInfo;
}

void CAM_HIK_FG::resetFrame()
{
    m_frameInfo = FrameInfo();
}

bool CAM_HIK_FG::isConnected() const
{
    return m_connected;
}

const bool CAM_HIK_FG::isGrabbing() const
{
    return m_grabbing;
}

bool CAM_HIK_FG::enable(bool enable)
{
    m_enable = enable;
    return true;
}

const QString& CAM_HIK_FG::getName() const
{
    return m_name;
}

const QString& CAM_HIK_FG::getSerialNumber() const
{
    return m_serialNumber;
}