#include "CAM_IRayple.h"
#include "Logger.h"
#include "Utilities.h"
#include "MbufPoolManager.h"

extern TMessageQue<FrameInfo> g_imageQueue;

void CAM_IRayple::FrameCallback(IMV_Frame* pFrame, void* pVoid)
{
	TimeLogger timer;

	QDateTime callbackTS = QDateTime::currentDateTime();
	CAM_IRayple* instance = reinterpret_cast<CAM_IRayple*>(pVoid);

	if (instance == nullptr)
	{
		ct::logger::error("Camera info invalid");
		//instance->m_conditionVariable.notify_one();
		return;
	}

	if (pFrame == nullptr)
	{
		ct::logger::error("Callback Frame data invalid");
		instance->m_conditionVariable.notify_one();
		return;
	}

	auto& frame = instance->m_frameInfo;
	frame.width = (int)pFrame->frameInfo.width;
	frame.height = (int)pFrame->frameInfo.height;
	frame.bufferSize = (int)pFrame->frameInfo.size;
	frame.timeStamp = pFrame->frameInfo.timeStamp;

	int channel = 3;
	if (pFrame->frameInfo.pixelFormat == gvspPixelRGB8) frame.pixelFormat = ICAM_pixelFormat::RGB8;
	else if (pFrame->frameInfo.pixelFormat == gvspPixelMono8) {
		frame.pixelFormat = ICAM_pixelFormat::Mono8;
		channel = 1;
	}
	else if (pFrame->frameInfo.pixelFormat == gvspPixelBayGB8) {
		frame.pixelFormat = ICAM_pixelFormat::BayerGB8;
		// Bayer mosaic is single-plane 8-bit: a 3-band buffer made MbufPut read 3x
		// past the end of the SDK frame data (crash), and ImageManager's
		// bayer_to_rgb expects a 1-band input anyway.
		channel = 1;
	}
	else if (pFrame->frameInfo.pixelFormat == gvspPixelBayRG8) {
		frame.pixelFormat = ICAM_pixelFormat::BayerRG8;
		channel = 1; //see BayerGB8 note
	}
	else {
		ct::logger::error("[FCB] Unsupported pixel format: %d", (int)pFrame->frameInfo.pixelFormat);
		instance->m_softTriggered = false;
		instance->m_conditionVariable.notify_one();
		return;
	}

	auto sbuf = mtrx::MPM::instance().acquire(frame.width, frame.height, channel, 8 + M_UNSIGNED);
	MbufPut(sbuf->id(), pFrame->pData);
	frame.pImage = sbuf; //was never assigned: frames reached the queue with a null image
	ct::logger::trace("[FCB] Data copied");

	g_imageQueue.push_back(frame);
	ct::logger::trace("[FCB] Pushed to image queue");

	instance->m_softTriggered = false;
	instance->m_conditionVariable.notify_one();

	frame = FrameInfo();

	timer.log_duration("{FrameCallback} Done data transfer");
}

bool CAM_IRayple::accessible() const
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

CAM_IRayple::CAM_IRayple()
{
	//m_id = id;
}

CAM_IRayple::~CAM_IRayple()
{
}

const int CAM_IRayple::getWidth() const
{
	return m_width;
}

const int CAM_IRayple::getHeight() const
{
	return m_height;
}

const int CAM_IRayple::getChannel() const
{
	return m_channel;
}

const double CAM_IRayple::getExposure() const
{
	return m_exposure;
}

const double CAM_IRayple::getGain() const
{
	return m_gain;
}

const QString& CAM_IRayple::getName() const
{
	return m_name;
}

const QString& CAM_IRayple::getSerialNumber() const
{
	return m_serialNumber;
}

const bool CAM_IRayple::isGrabbing() const
{
	if (!accessible()) return false;
	return IMV_IsGrabbing(m_handle);
}

bool CAM_IRayple::enable(bool enable)
{
	m_enable = enable;
	return true;
}

bool CAM_IRayple::connect(QString sn)
{
	m_serialNumber = sn;

	int ret = IMV_OK;

	IMV_DeviceList deviceList;

	ret = IMV_EnumDevices(&deviceList, interfaceTypeAll);
	if (ret != IMV_OK)
	{
		m_errorMsg = QStringLiteral("Enumeration devices failed! ErrorCode: %1").arg(ret);
		ct::logger::error("%s", m_errorMsg.toStdString().c_str());
		return false;
	}

	int num = deviceList.nDevNum;
	if (num == 0)
	{
		m_errorMsg = QStringLiteral("No camera discovered");
		ct::logger::error("%s", m_errorMsg.toStdString().c_str());
		return false;
	}

	for (int i = 0; i < num; i++)
	{
		if (deviceList.pDevInfo[i].serialNumber == m_serialNumber)
		{
			ret = IMV_CreateHandle(&m_handle, modeByIndex, (void*)&i);
			if (ret != IMV_OK)
			{
				m_errorMsg = QStringLiteral("Create camera handle failed! Camera: %1, ErrorCode: %2").arg(m_serialNumber).arg(ret);
				ct::logger::error("%s", m_errorMsg.toStdString().c_str());
				return false;
			}

			// Open camera 
			ret = IMV_Open(m_handle);
			if (ret != IMV_OK)
			{
				m_errorMsg = QStringLiteral("Open camera failed! Camera: %1, ErrorCode: %2").arg(m_serialNumber).arg(ret);
				ct::logger::error("%s", m_errorMsg.toStdString().c_str());
				return false;
			}
			m_isConnected = true;

			int64_t w, h;
			ret = IMV_GetIntFeatureValue(m_handle, "Width", &w);
			if (ret != IMV_OK)
			{
				m_errorMsg = QStringLiteral("Load frame width failed! Camera: %1, ErrorCode: %2").arg(m_serialNumber).arg(ret);
				ct::logger::error("%s", m_errorMsg.toStdString().c_str());
				return false;
			}

			ret = IMV_GetIntFeatureValue(m_handle, "Height", &h);
			if (ret != IMV_OK)
			{
				m_errorMsg = QStringLiteral("Load frame height failed! Camera: %1, ErrorCode: %2").arg(m_serialNumber).arg(ret);
				ct::logger::error("%s", m_errorMsg.toStdString().c_str());
				return false;
			}

			IMV_String pixelFormatSymbol;
			ret = IMV_GetEnumFeatureSymbol(m_handle, "PixelFormat", &pixelFormatSymbol);
			if (ret != IMV_OK)
			{
				m_errorMsg = QStringLiteral("Get pixel format failed! Camera: %1, ErrorCode: %2").arg(m_serialNumber).arg(ret);
				ct::logger::error("%s", m_errorMsg.toStdString().c_str());
				return false;
			}

			QString pixelFormat = QString(pixelFormatSymbol.str);
			m_channel = pixelFormat.contains(QStringLiteral("Mono8")) ? 1 : 3;
			m_width = w;
			m_height = h;

			break;
		}
	}

	return true;
}

bool CAM_IRayple::disconnect()
{
	if (!accessible()) return false;

	if (IMV_IsOpen(m_handle) == false) return true;

	auto ret = IMV_Close(m_handle);
	if (ret != IMV_OK)
	{
		m_errorMsg = QStringLiteral("Close camera handle failed! Camera: %1, ErrorCode: %2").arg(m_serialNumber).arg(ret);
		ct::logger::error("%s", m_errorMsg.toStdString().c_str());
		return false;
	}

	m_handle = nullptr;

	return true;
}

bool CAM_IRayple::isConnected() const
{
	//return IMV_IsOpen(m_handle);;
	return m_isConnected;
}

bool CAM_IRayple::startGrab()
{
	if (!accessible()) return false;

	int ret = IMV_OK;

	if (isGrabbing()) return true;

	ret = IMV_AttachGrabbing(m_handle, &CAM_IRayple::FrameCallback, this);
	if (ret != IMV_OK)
	{
		m_errorMsg = QStringLiteral("Attach grabbing failed! Camera: %1, ErrorCode: %2").arg(m_serialNumber).arg(ret);
		ct::logger::error("%s", m_errorMsg.toStdString().c_str());
		return false;
	}

	ret = IMV_StartGrabbing(m_handle);
	if (ret != IMV_OK)
	{
		m_errorMsg = QStringLiteral("Start grabbing failed! Camera: %1, ErrorCode: %2").arg(m_serialNumber).arg(ret);
		ct::logger::error("%s", m_errorMsg.toStdString().c_str());
		return false;
	}

	m_softTriggered = false;

	return true;
}

bool CAM_IRayple::stopGrab()
{
	if (!accessible()) return false;

	auto ret = IMV_StopGrabbing(m_handle);
	if (ret != IMV_OK)
	{
		m_errorMsg = QStringLiteral("Stop grabbing failed! Camera: %1, ErrorCode: %2").arg(m_serialNumber).arg(ret);
		ct::logger::error("%s", m_errorMsg.toStdString().c_str());
		return false;
	}

	return true;
}

bool CAM_IRayple::setExposure(double exposure)
{
	if (!accessible()) return false;

	////Note: Getting exposure took around 18ms, so not using this func due to speed concern
	//if (util::is_equal(exposure, m_exposure)) {
	//	return true;
	//}

	auto ret = IMV_SetDoubleFeatureValue(m_handle, "ExposureTime", exposure);

	if (ret == IMV_OK) {
		ct::logger::trace("Set camera exposure: %f", exposure);
		m_exposure = exposure;
	}
	else ct::logger::error("Failed to set camera exposure: %f", exposure);

	return (ret == IMV_OK);
}

bool CAM_IRayple::setGain(double gain)
{
	if (!accessible()) return false;

	////Note: Getting gain took around 3ms, so not using this func due to speed concern
	//if (util::is_equal(gain, m_gain)) {
	//	return true;
	//}

	auto ret = IMV_SetDoubleFeatureValue(m_handle, "GainRaw", gain);

	if (ret == IMV_OK) {
		ct::logger::trace("Set camera gain: %f", gain);
		m_gain = gain;
	}
	else ct::logger::error("Failed to set camera gain: %f", gain);

	return (ret == IMV_OK);
}

bool CAM_IRayple::softTrigger()
{
	if (!accessible()) return false;

	if (m_softTriggered) {
		ct::logger::warn("Camera busy, failed to trigger snap!");
		return false;
	}

	int ret = IMV_OK;

	ret = IMV_ExecuteCommandFeature(m_handle, "TriggerSoftware");
	if (ret != IMV_OK)
	{
		m_errorMsg = QStringLiteral("Execute soft tigger failed! Camera: %1, ErrorCode: %2").arg(m_serialNumber).arg(ret);
		ct::logger::error("Execute soft tigger failed!");
		return false;
	}

	m_softTriggered = true;

	return true;
}

bool CAM_IRayple::waitAcquisition(int ms)
{
	//To handle cases where soft trigger has been called, and return. But wait was call too late.
	if (!m_softTriggered) return true;

	std::unique_lock<std::mutex> lock(m_mutex);

	if (m_conditionVariable.wait_for(lock, std::chrono::milliseconds(ms)) == std::cv_status::no_timeout) return true;

	m_softTriggered = false;

	return false;
}

bool CAM_IRayple::setTriggerOutput(QString line, QString source)
{
	if (!accessible()) return false;

	int ret = IMV_OK;

	ret = IMV_SetEnumFeatureSymbol(m_handle, "LineSelector", line.toLocal8Bit());
	if (ret != IMV_OK)
	{
		m_errorMsg = QStringLiteral("Failed to set trigger output! Camera: %1, ErrorCode: %2").arg(m_serialNumber).arg(ret);
		return false;
	}

	ret = IMV_SetEnumFeatureSymbol(m_handle, "LineSource", source.toLocal8Bit());
	if (ret != IMV_OK)
	{
		m_errorMsg = QStringLiteral("Failed to set trigger output! Camera: %1, ErrorCode: %2").arg(m_serialNumber).arg(ret);
		return false;
	}

	return true;
}

bool CAM_IRayple::setDO(int DO, bool on)
{
	return false;
}

bool CAM_IRayple::loadConfig(QString path)
{
	if (!accessible()) return false;

	IMV_ErrorList errorList;
	memset(&errorList, 0, sizeof(IMV_ErrorList));

	auto ret = IMV_LoadDeviceCfg(m_handle, path.toLocal8Bit(), &errorList);
	if (ret != IMV_OK)
	{
		m_errorMsg = QStringLiteral("Load camera configuration failed! Camera: %1, ErrorCode: %2").arg(m_serialNumber).arg(ret);
		ct::logger::error("[Camera] %s", m_errorMsg.toStdString().c_str());
		return false;
	}

	return true;
}

QString CAM_IRayple::errorMsg()
{
	return m_errorMsg;
}

const FrameInfo& CAM_IRayple::frame() const
{
	return m_frameInfo;
}

FrameInfo& CAM_IRayple::frame()
{
	return m_frameInfo;
}

void CAM_IRayple::resetFrame()
{
	m_frameInfo = FrameInfo();
}
