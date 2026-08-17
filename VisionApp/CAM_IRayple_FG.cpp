#include "CAM_IRayple_FG.h"
#include "Logger.h"
#include "Utilities.h"
#include "MbufPoolManager.h"

extern TMessageQue<FrameInfo> g_imageQueue;

void CAM_IRayple_FG::FrameCallback(IMV_FG_Frame* pFrame, void* pVoid)
{
	TimeLogger timer;

	QDateTime callbackTS = QDateTime::currentDateTime();
	CAM_IRayple_FG* instance = reinterpret_cast<CAM_IRayple_FG*>(pVoid);

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

	if (pFrame->frameInfo.pixelFormat == IMV_FG_PIXEL_TYPE_RGB8) frame.pixelFormat = ICAM_pixelFormat::RGB8;
	else if (pFrame->frameInfo.pixelFormat == IMV_FG_PIXEL_TYPE_Mono8) frame.pixelFormat = ICAM_pixelFormat::Mono8;
	else if (pFrame->frameInfo.pixelFormat == IMV_FG_PIXEL_TYPE_BayGB8) frame.pixelFormat = ICAM_pixelFormat::BayerGB8;
	else if (pFrame->frameInfo.pixelFormat == IMV_FG_PIXEL_TYPE_BayRG8) frame.pixelFormat = ICAM_pixelFormat::BayerRG8;

	auto sbuf = mtrx::MPM::instance().acquire(frame.width, frame.height, 3, 8 + M_UNSIGNED);
	MbufPut(sbuf->id(), pFrame->pData);
	ct::logger::trace("[FCB] Data copied");
	
	g_imageQueue.push_back(frame);
	ct::logger::trace("[FCB] Pushed to image queue");
	
	instance->m_softTriggered = false;
	instance->m_conditionVariable.notify_one();

	frame = FrameInfo();

	timer.log_duration("{FrameCallback} Done data transfer");

}

bool CAM_IRayple_FG::accessible() const
{
	if (!m_enable) {
		ct::logger::warn("Trying to access a disabled camera: %s", m_serialNumber.toStdString().c_str());
		return false;
	}

	if (!m_ifHandle) {
		ct::logger::warn("Camera interface handle not initialized: %s", m_serialNumber.toStdString().c_str());
		return false;
	}

	if (!m_devHandle) {
		ct::logger::warn("Camera device handle not initialized: %s", m_serialNumber.toStdString().c_str());
		return false;
	}

	if (!m_isConnected) {
		ct::logger::warn("Camera is not connected: %s", m_serialNumber.toStdString().c_str());
		return false;
	}

	return true;
}

CAM_IRayple_FG::CAM_IRayple_FG()
{
	//m_id = id;
}

CAM_IRayple_FG::~CAM_IRayple_FG()
{
}

const int CAM_IRayple_FG::getWidth() const
{
	return m_width;
}

const int CAM_IRayple_FG::getHeight() const
{
	return m_height;
}

const int CAM_IRayple_FG::getChannel() const
{
	return m_channel;
}

const double CAM_IRayple_FG::getExposure() const
{
	return m_exposure;
}

const double CAM_IRayple_FG::getGain() const
{
	return m_gain;
}

const QString& CAM_IRayple_FG::getName() const
{
	return m_name;
}

const QString& CAM_IRayple_FG::getSerialNumber() const
{
	return m_serialNumber;
}

const bool CAM_IRayple_FG::isGrabbing() const
{
	if (!accessible()) return false;
	return IMV_FG_IsGrabbing(m_ifHandle);
}

bool CAM_IRayple_FG::enable(bool enable)
{
	m_enable = enable;
	return true;
}

bool CAM_IRayple_FG::connect(QString sn)
{
	m_serialNumber = sn;

	int ret = IMV_FG_OK;

	IMV_FG_INTERFACE_INFO_LIST iList;

	ret = IMV_FG_EnumInterface(typeCXPInterface, &iList);
	if (ret != IMV_FG_OK)
	{
		m_errorMsg = QStringLiteral("Enumeration devices failed! ErrorCode: %1").arg(ret);
		ct::logger::error("%s", m_errorMsg.toStdString().c_str());
		return false;
	}

	int num = iList.nInterfaceNum;
	if (num == 0)
	{
		m_errorMsg = QStringLiteral("No framegrabber discovered");
		ct::logger::error("%s", m_errorMsg.toStdString().c_str());
		return false;
	}

	IMV_FG_DEVICE_INFO_LIST dList;
	ret = IMV_FG_EnumDevices(typeCXPInterface, &dList);
	if (IMV_FG_OK != ret)
	{
		m_errorMsg = QStringLiteral("Enumeration camera devices failed!errorCode:[%d]").arg(ret);
		ct::logger::error("%s", m_errorMsg.toStdString().c_str());
		return false;
	}

	for (int i = 0; i < num; i++)
	{
		if (dList.pDeviceInfoList[i].serialNumber == m_serialNumber)
		{
			// Open capture device 
			auto fg_ret = IMV_FG_OpenInterface(i, &m_ifHandle);
			if (IMV_FG_OK != fg_ret)
			{
				m_errorMsg = QStringLiteral("Open cameralink capture board device failed! errorCode:[%d]").arg(ret);
				ct::logger::error("%s", m_errorMsg.toStdString().c_str());
				return false;
			}

			// Connect to camera 
			fg_ret = IMV_FG_OpenDevice(IMV_FG_MODE_BY_INDEX, (void*)&i, &m_devHandle);
			if (IMV_FG_OK != fg_ret)
			{
				m_errorMsg = QStringLiteral("Open camera failed! ErrorCode[%d]").arg(ret);
				ct::logger::error("%s", m_errorMsg.toStdString().c_str());
				return false;
			}

			m_isConnected = true;

			int64_t w, h;
			ret = IMV_FG_GetIntFeatureValue(m_devHandle, "Width", &w);
			if (ret != IMV_FG_OK)
			{
				m_errorMsg = QStringLiteral("Load frame width failed! Camera: %1, ErrorCode: %2").arg(m_serialNumber).arg(ret);
				ct::logger::error("%s", m_errorMsg.toStdString().c_str());
				return false;
			}

			ret = IMV_FG_GetIntFeatureValue(m_devHandle, "Height", &h);
			if (ret != IMV_FG_OK)
			{
				m_errorMsg = QStringLiteral("Load frame height failed! Camera: %1, ErrorCode: %2").arg(m_serialNumber).arg(ret);
				ct::logger::error("%s", m_errorMsg.toStdString().c_str());
				return false;
			}

			IMV_FG_String pixelFormatSymbol;
			ret = IMV_FG_GetEnumFeatureSymbol(m_devHandle, "PixelFormat", &pixelFormatSymbol);
			if (ret != IMV_FG_OK)
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

bool CAM_IRayple_FG::disconnect()
{
	if (!accessible()) return false;

	if (IMV_FG_IsDeviceOpen(m_devHandle) == false) return true;
	auto ret = IMV_FG_CloseDevice(m_devHandle);

	if (ret != IMV_FG_OK)
	{
		m_errorMsg = QStringLiteral("Close camera device handle failed! Camera: %1, ErrorCode: %2").arg(m_serialNumber).arg(ret);
		ct::logger::error("%s", m_errorMsg.toStdString().c_str());
		return false;
	}

	m_devHandle = nullptr;


	if (IMV_FG_IsOpenInterface(m_ifHandle) == false) return true;

	ret = IMV_FG_CloseInterface(m_ifHandle);

	if (ret != IMV_FG_OK)
	{
		m_errorMsg = QStringLiteral("Close camera interface handle failed! Camera: %1, ErrorCode: %2").arg(m_serialNumber).arg(ret);
		ct::logger::error("%s", m_errorMsg.toStdString().c_str());
		return false;
	}

	m_ifHandle = nullptr;

	return true;
}

bool CAM_IRayple_FG::isConnected() const
{
	//return IMV_IsOpen(m_handle);;
	return m_isConnected;
}

bool CAM_IRayple_FG::startGrab()
{
	if (!accessible()) return false;

	int ret = IMV_FG_OK;

	if (isGrabbing()) return true;

	ret = IMV_FG_AttachGrabbing(m_ifHandle, &CAM_IRayple_FG::FrameCallback, this);
	if (ret != IMV_FG_OK)
	{
		m_errorMsg = QStringLiteral("Attach grabbing failed! Camera: %1, ErrorCode: %2").arg(m_serialNumber).arg(ret);
		ct::logger::error("%s", m_errorMsg.toStdString().c_str());
		return false;
	}

	ret = IMV_FG_StartGrabbing(m_ifHandle);
	if (ret != IMV_FG_OK)
	{
		m_errorMsg = QStringLiteral("Start grabbing failed! Camera: %1, ErrorCode: %2").arg(m_serialNumber).arg(ret);
		ct::logger::error("%s", m_errorMsg.toStdString().c_str());
		return false;
	}

	m_softTriggered = false;

	return true;
}

bool CAM_IRayple_FG::stopGrab()
{
	if (!accessible()) return false;

	auto ret = IMV_FG_StopGrabbing(m_ifHandle);
	if (ret != IMV_FG_OK)
	{
		m_errorMsg = QStringLiteral("Stop grabbing failed! Camera: %1, ErrorCode: %2").arg(m_serialNumber).arg(ret);
		ct::logger::error("%s", m_errorMsg.toStdString().c_str());
		return false;
	}

	return true;
}

bool CAM_IRayple_FG::setExposure(double exposure)
{
	if (!accessible()) return false;

	////Note: Getting exposure took around 18ms, so not using this func due to speed concern
	//if (util::is_equal(exposure, m_exposure)) {
	//	return true;
	//}

	auto ret = IMV_FG_SetDoubleFeatureValue(m_devHandle, "ExposureTime", exposure);

	if (ret == IMV_FG_OK) {
		ct::logger::trace("Set camera exposure: %f", exposure);
		m_exposure = exposure;
	}
	else ct::logger::error("Failed to set camera exposure: %f", exposure);

	return (ret == IMV_FG_OK);
}

bool CAM_IRayple_FG::setGain(double gain)
{
	if (!accessible()) return false;

	////Note: Getting gain took around 3ms, so not using this func due to speed concern
	//if (util::is_equal(gain, m_gain)) {
	//	return true;
	//}

	auto ret = IMV_FG_SetDoubleFeatureValue(m_devHandle, "GainRaw", gain);

	if (ret == IMV_FG_OK) {
		ct::logger::trace("Set camera gain: %f", gain);
		m_gain = gain;
	}
	else ct::logger::error("Failed to set camera gain: %f", gain);

	return (ret == IMV_FG_OK);
}

bool CAM_IRayple_FG::softTrigger()
{
	if (!accessible()) return false;

	if (m_softTriggered) {
		ct::logger::warn("Camera busy, failed to trigger snap!");
		return false;
	}

	int ret = IMV_FG_OK;

	ret = IMV_FG_ExecuteCommandFeature(m_devHandle, "TriggerSoftware");
	if (ret != IMV_FG_OK)
	{
		m_errorMsg = QStringLiteral("Execute soft tigger failed! Camera: %1, ErrorCode: %2").arg(m_serialNumber).arg(ret);
		ct::logger::error("Execute soft tigger failed!");
		return false;
	}

	m_softTriggered = true;

	return true;
}

bool CAM_IRayple_FG::waitAcquisition(int ms)
{
	if (!m_softTriggered) return true;

	std::unique_lock<std::mutex> lock(m_mutex);

	if (m_conditionVariable.wait_for(lock, std::chrono::milliseconds(ms)) == std::cv_status::no_timeout) return true;

	m_softTriggered = false;

	return false;
}

bool CAM_IRayple_FG::setTriggerOutput(QString line, QString source)
{
	if (!accessible()) return false;

	int ret = IMV_FG_OK;

	ret = IMV_FG_SetEnumFeatureSymbol(m_devHandle, "LineSelector", line.toLocal8Bit());
	if (ret != IMV_FG_OK)
	{
		m_errorMsg = QStringLiteral("Failed to set trigger output! Camera: %1, ErrorCode: %2").arg(m_serialNumber).arg(ret);
		return false;
	}

	ret = IMV_FG_SetEnumFeatureSymbol(m_devHandle, "LineSource", source.toLocal8Bit());
	if (ret != IMV_FG_OK)
	{
		m_errorMsg = QStringLiteral("Failed to set trigger output! Camera: %1, ErrorCode: %2").arg(m_serialNumber).arg(ret);
		return false;
	}

	return true;
}

bool CAM_IRayple_FG::setDO(int DO, bool on)
{
	return false;
}

bool CAM_IRayple_FG::loadConfig(QString path)
{
	if (!accessible()) return false;

	IMV_FG_ErrorList errorList;
	memset(&errorList, 0, sizeof(IMV_FG_ErrorList));

	auto ret = IMV_FG_LoadDeviceCfg(m_devHandle, path.toLocal8Bit(), &errorList);
	if (ret != IMV_FG_OK)
	{
		m_errorMsg = QStringLiteral("Load camera configuration failed! Camera: %1, ErrorCode: %2").arg(m_serialNumber).arg(ret);
		ct::logger::error("[Camera] %s", m_errorMsg.toStdString().c_str());
		return false;
	}

	return true;
}

QString CAM_IRayple_FG::errorMsg()
{
	return m_errorMsg;
}

const FrameInfo& CAM_IRayple_FG::frame() const
{
	return m_frameInfo;
}

FrameInfo& CAM_IRayple_FG::frame()
{
	return m_frameInfo;
}

void CAM_IRayple_FG::resetFrame()
{
	m_frameInfo = FrameInfo();
}
