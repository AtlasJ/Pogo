#include "CAM_OPT.h"
#include "Logger.h"
#include "Utilities.h"
#include "MbufPoolManager.h"

extern TMessageQue<FrameInfo> g_imageQueue;

void CAM_OPT::PayloadCallback(void* payload, void* pVoid)
{
	TimeLogger timer;

	QDateTime callbackTS = QDateTime::currentDateTime();
	CAM_OPT* instance = reinterpret_cast<CAM_OPT*>(pVoid);

	if (instance == nullptr)
	{
		ct::logger::error("[FCB] Camera info invalid");
		//instance->m_conditionVariable.notify_one();
		return;
	}

	if (payload == nullptr)
	{
		ct::logger::error("[FCB] Callback Frame data invalid");
		instance->m_conditionVariable.notify_one();
		return;
	}
	
	unsigned int nReVal = SCI_CAMERA_OK;
	unsigned char* pBuffer = NULL;
	QString callbackErrorMsg = "";

	OPTSciCam::SCI_CAM_PAYLOAD_ATTRIBUTE payloadAttribute;
	memset(&payloadAttribute, 0, sizeof(OPTSciCam::SCI_CAM_PAYLOAD_ATTRIBUTE));

	nReVal = SciCam_Payload_GetAttribute(payload, &payloadAttribute);
	if (nReVal != SCI_CAMERA_OK)
	{
		callbackErrorMsg = QStringLiteral("[FCB] Get attribute failed! ErrorCode: %1").arg(nReVal);
		ct::logger::error("%s", callbackErrorMsg.toStdString().c_str());
		return;
	}
	
	nReVal = OPTSciCam::SciCam_Payload_GetImage(payload, (void**)&pBuffer);
	if (nReVal != SCI_CAMERA_OK)
	{
		callbackErrorMsg = QStringLiteral("[FCB] Get image failed! ErrorCode: %1").arg(nReVal);
		ct::logger::error("%s", callbackErrorMsg.toStdString().c_str());
		return;
	}

	auto& frame = instance->m_frameInfo;
	frame.width = (int)payloadAttribute.imgAttr.width;
	frame.height = (int)payloadAttribute.imgAttr.height;
	frame.timeStamp = payloadAttribute.timeStamp;

	if (payloadAttribute.imgAttr.pixelType == OPTSciCam::RGB8) frame.pixelFormat = ICAM_pixelFormat::RGB8;
	else if (payloadAttribute.imgAttr.pixelType == OPTSciCam:: Mono8) frame.pixelFormat = ICAM_pixelFormat::Mono8;
	else if (payloadAttribute.imgAttr.pixelType == OPTSciCam::BayerGB8) frame.pixelFormat = ICAM_pixelFormat::BayerGB8;
	else if (payloadAttribute.imgAttr.pixelType == OPTSciCam::BayerRG8) frame.pixelFormat = ICAM_pixelFormat::BayerRG8;

	if (payloadAttribute.imgAttr.pixelType == OPTSciCam::Mono8) {
		frame.type = ct::s_mono;
		frame.bufferSize = frame.width * frame.height;

	}
	else {
		frame.type = ct::s_color;
		frame.bufferSize = frame.width * frame.height * 3;

	}

	auto sbuf = mtrx::MPM::instance().acquire(frame.width, frame.height, 3, 8 + M_UNSIGNED);
	MbufPut(sbuf->id(), pBuffer);
	ct::logger::trace("[FCB] Data copied");

	g_imageQueue.push_back(frame);
	ct::logger::trace("[FCB] Pushed to image queue");

	instance->m_softTriggered = false;
	instance->m_conditionVariable.notify_one();

	frame = FrameInfo();

	timer.log_duration("{FrameCallback} Done data transfer");

}

const QMap<QString, CAM_OPT::LineSelectorType> CAM_OPT::lineSelectorMap = {
	{"Line1", Line1},
	{"Line2", Line2},
	{"Line3", Line3}
};

const QMap<QString, CAM_OPT::LineSourceType> CAM_OPT::lineSourceMap = {
	{"UserOutput", UserOutput},
	{"FrameTriggerWait", FrameTriggerWait},
	{"ExposureActive", ExposureActive},
	{"Timer1Active", Timer1Active}
};

bool CAM_OPT::accessible() const
{
	if (!m_enable) {
		ct::logger::warn("Trying to access a disabled camera: %s", m_serialNumber.toStdString().c_str());
		return false;
	}

	if (!m_handle) {
		ct::logger::warn("Camera handle not initialized: %s", m_serialNumber.toStdString().c_str());
		return false;
	}

	if (!m_isConnected) {
		ct::logger::warn("Camera is not connected: %s", m_serialNumber.toStdString().c_str());
		return false;
	}

	return true;
}

CAM_OPT::CAM_OPT()
{
	//m_id = id;

}

CAM_OPT::~CAM_OPT()
{
}

const int CAM_OPT::getWidth() const
{
	return m_width;
}

const int CAM_OPT::getHeight() const
{
	return m_height;
}

const int CAM_OPT::getChannel() const
{
	return m_channel;
}

const double CAM_OPT::getExposure() const
{
	return m_exposure;
}

const double CAM_OPT::getGain() const
{
	return m_gain;
}

const QString& CAM_OPT::getName() const
{
	return m_name;
}

const QString& CAM_OPT::getSerialNumber() const
{
	return m_serialNumber;
}

const bool CAM_OPT::isGrabbing() const
{
	if (!accessible()) return false;
	return m_isStartGrabbing;

}

bool CAM_OPT::enable(bool enable)
{
	m_enable = enable;
	return true;
}

bool CAM_OPT::connect(QString sn)
{
	m_serialNumber = sn;
	m_isConnected = false;

	unsigned int nReVal = SCI_CAMERA_OK;

	// Allocate device list dynamically to avoid large stack usage
	auto deviceList = std::make_unique<OPTSciCam::SCI_DEVICE_INFO_LIST>();

	nReVal = SciCam_DiscoveryDevices(deviceList.get(), OPTSciCam::SciCamTLType::SciCam_TLType_Gige | OPTSciCam::SciCamTLType::SciCam_TLType_Usb3);
	if (nReVal != SCI_CAMERA_OK)
	{
		m_errorMsg = QStringLiteral("Discovery devices failed! ErrorCode: %1").arg(nReVal);
		ct::logger::error("%s", m_errorMsg.toStdString().c_str());
		return false;
	}

	int num = deviceList->count;
	if (num == 0)
	{
		m_errorMsg = QStringLiteral("No devices discovered");
		ct::logger::error("%s", m_errorMsg.toStdString().c_str());
		return false;
	}

	for (int i = 0; i < num; i++)
	{
		OPTSciCam::SciCamTLType devTlType = deviceList->pDevInfo[i].tlType;

		QString serialNumber;

		switch (devTlType) {
		case OPTSciCam::SciCamTLType::SciCam_TLType_Gige:
			serialNumber = QString::fromUtf8(reinterpret_cast<char*>(deviceList->pDevInfo[i].info.gigeInfo.serialNumber));
			break;
		case OPTSciCam::SciCamTLType::SciCam_TLType_Usb3:
			serialNumber = QString::fromUtf8(reinterpret_cast<char*>(deviceList->pDevInfo[i].info.usb3Info.serialNumber));
			break;
		default:
			m_errorMsg = QStringLiteral("Unknown device! Camera: %1, DeviceTypeCode: %2").arg(m_serialNumber).arg(devTlType);
			ct::logger::error("%s", m_errorMsg.toStdString().c_str());
			break;
		}

		if (serialNumber == m_serialNumber)
		{
			// Create camera handle
			nReVal = SciCam_CreateDevice(&m_handle, &deviceList->pDevInfo[i]);
			if (nReVal != SCI_CAMERA_OK)
			{
				m_errorMsg = QStringLiteral("Create device handle failed! Camera: %1, ErrorCode: %2").arg(m_serialNumber).arg(nReVal);
				ct::logger::error("%s", m_errorMsg.toStdString().c_str());
				m_handle = nullptr;
				return false;
			}

			// Open camera 
			nReVal = OPTSciCam::SciCam_OpenDevice(m_handle);
			if (nReVal != SCI_CAMERA_OK)
			{
				m_errorMsg = QStringLiteral("Open camera failed! Camera: %1, ErrorCode: %2").arg(m_serialNumber).arg(nReVal);
				ct::logger::error("%s", m_errorMsg.toStdString().c_str());
				m_handle = nullptr;
				return false;
			}
			m_isConnected = true;

			// Allocate camera properties on the heap
			auto w = std::make_unique<OPTSciCam::SCI_NODE_VAL_INT>();
			auto h = std::make_unique<OPTSciCam::SCI_NODE_VAL_INT>();
			auto pixelFormatSymbol = std::make_unique<OPTSciCam::SCI_NODE_VAL_ENUM>();

			nReVal = SciCam_GetIntValue(m_handle, "Width", w.get());
			if (nReVal != SCI_CAMERA_OK)
			{
				m_errorMsg = QStringLiteral("Load frame width failed! Camera: %1, ErrorCode: %2").arg(m_serialNumber).arg(nReVal);
				ct::logger::error("%s", m_errorMsg.toStdString().c_str());
				return false;
			}

			nReVal = SciCam_GetIntValue(m_handle, "Height", h.get());
			if (nReVal != SCI_CAMERA_OK)
			{
				m_errorMsg = QStringLiteral("Load frame height failed! Camera: %1, ErrorCode: %2").arg(m_serialNumber).arg(nReVal);
				ct::logger::error("%s", m_errorMsg.toStdString().c_str());
				return false;
			}

			nReVal = SciCam_GetEnumValue(m_handle, "PixelFormat", pixelFormatSymbol.get());
			if (nReVal != SCI_CAMERA_OK)
			{
				m_errorMsg = QStringLiteral("Get pixel format failed! Camera: %1, ErrorCode: %2").arg(m_serialNumber).arg(nReVal);
				ct::logger::error("%s", m_errorMsg.toStdString().c_str());
				return false;
			}

			QString pixelFormat = pixelFormatSymbol->items->desc;
			m_channel = pixelFormat.contains(QStringLiteral("Mono8")) ? 1 : 3;
			m_width = w->nVal;
			m_height = h->nVal;

			break;
		}
	}

	return true;
}

bool CAM_OPT::disconnect()
{
	if (!accessible()) return false;

	if (m_isStartGrabbing) stopGrab();

	if (OPTSciCam::SciCam_IsDeviceOpen(m_handle) == false) return true;

	unsigned int nReVal = SCI_CAMERA_OK;

	nReVal = OPTSciCam::SciCam_CloseDevice(m_handle);
	if (nReVal != SCI_CAMERA_OK)
	{
		m_errorMsg = QStringLiteral("Close device handle failed! Camera: %1, ErrorCode: %2").arg(m_serialNumber).arg(nReVal);
		ct::logger::error("%s", m_errorMsg.toStdString().c_str());
		return false;

	}

	nReVal = OPTSciCam::SciCam_DeleteDevice(m_handle);
	if (nReVal != SCI_CAMERA_OK)
	{
		m_errorMsg = QStringLiteral("Delete device handle failed! Camera: %1, ErrorCode: %2").arg(m_serialNumber).arg(nReVal);
		ct::logger::error("%s", m_errorMsg.toStdString().c_str());
		return false;
		
	}

	m_isConnected = false;
	m_isStartGrabbing = false;
	m_handle = nullptr;

	return true;
}

bool CAM_OPT::isConnected() const
{
	//return IMV_IsOpen(m_handle);;
	return m_isConnected;
}

bool CAM_OPT::startGrab()
{
	if (!accessible()) return false;

	unsigned int nReVal = SCI_CAMERA_OK;

	if (isGrabbing()) return true;
	
	// register camera frame callback
	nReVal = OPTSciCam::SciCam_RegisterPayloadCallBack(m_handle, &CAM_OPT::PayloadCallback, this, true);
	if (nReVal != SCI_CAMERA_OK)
	{
		m_errorMsg = QStringLiteral("Attach grabbing failed! Camera: %1, ErrorCode: %2").arg(m_serialNumber).arg(nReVal);
		ct::logger::error("%s", m_errorMsg.toStdString().c_str());
		return false;
	}

	nReVal = OPTSciCam::SciCam_StartGrabbing(m_handle);
	if (nReVal != SCI_CAMERA_OK)
	{
		m_errorMsg = QStringLiteral("Start grabbing failed! Camera: %1, ErrorCode: %2").arg(m_serialNumber).arg(nReVal);
		ct::logger::error("%s", m_errorMsg.toStdString().c_str());
		return false;
	}

	m_isStartGrabbing = true;
	m_softTriggered = false;

	return true;

}

bool CAM_OPT::stopGrab()
{
	if (!accessible()) return false;

	unsigned int nReVal = SCI_CAMERA_OK;

	nReVal = OPTSciCam::SciCam_StopGrabbing(m_handle);
	if (nReVal != SCI_CAMERA_OK)
	{
		m_errorMsg = QStringLiteral("Stop grabbing failed! Camera: %1, ErrorCode: %2").arg(m_serialNumber).arg(nReVal);
		ct::logger::error("%s", m_errorMsg.toStdString().c_str());
		return false;
	}

	m_isStartGrabbing = false;

	return true;
}

bool CAM_OPT::setExposure(double exposure)
{
	if (!accessible()) return false;

	////Note: Getting exposure took around 18ms, so not using this func due to speed concern
	//if (util::is_equal(exposure, m_exposure)) {
	//	return true;
	//}

	unsigned int nReVal = SCI_CAMERA_OK;
	
	nReVal = OPTSciCam::SciCam_SetFloatValue(m_handle, "ExposureTime", exposure);
	if (nReVal == SCI_CAMERA_OK)
	{
		ct::logger::trace("Set camera exposure: %f", exposure);
		m_exposure = exposure;
	}
	else ct::logger::error("Failed to set camera exposure: %f", exposure);

	return (nReVal == SCI_CAMERA_OK);
}

bool CAM_OPT::setGain(double gain)
{
	if (!accessible()) return false;

	////Note: Getting gain took around 3ms, so not using this func due to speed concern
	//if (util::is_equal(gain, m_gain)) {
	//	return true;
	//}

	unsigned int nReVal = SCI_CAMERA_OK;
	
	nReVal = OPTSciCam::SciCam_SetFloatValue(m_handle, "Gain", gain);
	if (nReVal == SCI_CAMERA_OK) {
		ct::logger::trace("Set camera gain: %f", gain);
		m_gain = gain;
	}
	else ct::logger::error("Failed to set camera gain: %f", gain);

	return (nReVal == SCI_CAMERA_OK);
}

bool CAM_OPT::softTrigger()
{
	if (!accessible()) return false;

	if (m_softTriggered) {
		ct::logger::warn("Camera busy, failed to trigger snap!");
		return false;
	}

	unsigned int nReVal = SCI_CAMERA_OK;

	nReVal = OPTSciCam::SciCam_SetCommandValue(m_handle, "TriggerSoftware");
	if (nReVal != SCI_CAMERA_OK)
	{
		m_errorMsg = QStringLiteral("Execute soft tigger failed! Camera: %1, ErrorCode: %2").arg(m_serialNumber).arg(nReVal);
		ct::logger::error("Execute soft tigger failed!");
		return false;
	}

	m_softTriggered = true;

	return true;
}

bool CAM_OPT::waitAcquisition(int ms)
{
	//To handle cases where soft trigger has been called, and return. But wait was call too late.
	if (!m_softTriggered) return true;

	std::unique_lock<std::mutex> lock(m_mutex);

	if (m_conditionVariable.wait_for(lock, std::chrono::milliseconds(ms)) == std::cv_status::no_timeout) return true;

	m_softTriggered = false;

	return false;
}

bool CAM_OPT::setTriggerOutput(QString line, QString source)
{
	if (!accessible()) return false;

	unsigned int nReVal = SCI_CAMERA_OK;

	if (!lineSelectorMap.contains(line)) {
		m_errorMsg = QStringLiteral("Invalid line selector: %1").arg(line);
		return false;
	}

	if (!lineSourceMap.contains(source)) {
		m_errorMsg = QStringLiteral("Invalid line source: %1").arg(source);
		return false;
	}

	nReVal = OPTSciCam::SciCam_SetEnumValue(m_handle, "LineSelector", CAM_OPT::lineSelectorMap.value(line));
	if (nReVal != SCI_CAMERA_OK)
	{
		m_errorMsg = QStringLiteral("Failed to set trigger output! Camera: %1, ErrorCode: %2").arg(m_serialNumber).arg(nReVal);
		return false;
	}

	nReVal = OPTSciCam::SciCam_SetEnumValue(m_handle, "LineSource", CAM_OPT::lineSourceMap.value(source));
	if (nReVal != SCI_CAMERA_OK)
	{
		m_errorMsg = QStringLiteral("Failed to set trigger output! Camera: %1, ErrorCode: %2").arg(m_serialNumber).arg(nReVal);
		return false;
	}

	return true;
}

bool CAM_OPT::setDO(int DO, bool on)
{
	return false;
}

bool CAM_OPT::loadConfig(QString path)
{
	if (!accessible()) return false;

	unsigned int nReVal = SCI_CAMERA_OK;

	nReVal = OPTSciCam::SciCam_FeatureLoad(m_handle, path.toLocal8Bit());
	if (nReVal != SCI_CAMERA_OK)
	{
		m_errorMsg = QStringLiteral("Load camera configuration failed! Camera: %1, ErrorCode: %2").arg(m_serialNumber).arg(nReVal);
		ct::logger::error("[Camera] %s", m_errorMsg.toStdString().c_str());
		return false;
	}

	return true;
}

QString CAM_OPT::errorMsg()
{
	return m_errorMsg;
}

const FrameInfo& CAM_OPT::frame() const
{
	return m_frameInfo;
}

FrameInfo& CAM_OPT::frame()
{
	return m_frameInfo;
}

void CAM_OPT::resetFrame()
{
	m_frameInfo = FrameInfo();
}

