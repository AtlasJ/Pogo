#include "Profiler_Gocator.h"
#include "Logger.h"
#include "mtrx.h"
#include "MessageQue.h"
#include "MbufPoolManager.h"

#define INVALID_RANGE_16BIT     ((signed short)0x8000)          // gocator transmits range data as 16-bit signed integers. 0x8000 signifies invalid range data. 
#define DOUBLE_MAX              ((k64f)1.7976931348623157e+308) // 64-bit double - largest positive value.  
#define INVALID_RANGE_DOUBLE    ((k64f)-DOUBLE_MAX)             // floating point value to represent invalid range data.

#define NM_TO_MM(VALUE) (((k64f)(VALUE))/1000000.0)
#define UM_TO_MM(VALUE) (((k64f)(VALUE))/1000.0)

#define RECEIVE_TIMEOUT         5000000 

extern TMessageQue<FrameInfo> g_imageQueue;

typedef struct ProfilePoint
{
	double x;   // x-coordinate in engineering units (mm) - position along laser line
	double y;   // y-coordinate in engineering units (mm) - position along the direction of travel
	double z;   // z-coordinate in engineering units (mm) - height (at the given x position)
	unsigned char intensity;
} ProfilePoint;

kStatus kCall Profiler_Gocator::Callback(kPointer context, GoSensor m_sensor, GoDataSet dataset)
{
	ct::logger::info("Enter Laser Callback");

	Profiler_Gocator* instance = reinterpret_cast<Profiler_Gocator*>(context);

	if (instance == nullptr)
	{
		ct::logger::error("Profiler Gocator instance invalid");
		return false;
	}

	GoDataMsg dataObj;
	GoStamp* stamp = kNULL;
	GoMeasurementData* measurementData = kNULL;
	GoMeasurement measurement = kNULL;
	int frameIndex = 0;

	ProfilePoint* profileBuffer = NULL;
	k32u profilePointCount;

	GoSetup setup;
	// retrieve setup handle
	if ((setup = GoSensor_Setup(m_sensor)) == kNULL)
	{
		ct::logger::error("3D Callback failed to retrieve setup data.");
		return false;
	}

	// retrieve total number of profile points prior to starting the sensor
	if (GoSetup_UniformSpacingEnabled(setup))
	{
		// Uniform spacing is enabled. The number is based on the X Spacing setting
		profilePointCount = GoSetup_XSpacingCount(setup, GO_ROLE_MAIN);
	}
	else
	{
		// non-uniform spacing is enabled. The max number is based on the number of columns used in the camera. 
		profilePointCount = GoSetup_FrontCameraWidth(setup, GO_ROLE_MAIN);
	}

	if ((profileBuffer = (ProfilePoint*)malloc(profilePointCount * sizeof(ProfilePoint))) == kNULL)
	{
		ct::logger::error("Cannot allocate profileData, %d points\n", profilePointCount);
		return false;
	}

	//each result can have multiple data items
	//loop through all items in result message

	auto& frame = instance->m_frameInfo;
	auto& info = instance->m_goInfo;
	/*frame.type = g_Info.type;
	frame.viewID = g_Info.viewID;
	frame.opticID = g_Info.opticID;
	frame.stitchID = g_Info.stitchID;
	frame.postTask.rotationalAngle = g_rotationAngle;*/

	frame.pImage = M_NULL;

	for (int i = 0; i < GoDataSet_Count(dataset); ++i)
	{
		GoDataMsg dataObj = GoDataSet_At(dataset, i);

		switch (GoDataMsg_Type(dataObj))
		{
		case GO_DATA_MESSAGE_TYPE_STAMP:
		{
			GoStampMsg stampMsg = dataObj;

			for (int j = 0; j < GoStampMsg_Count(stampMsg); j++)
			{
				GoStamp* stamp = GoStampMsg_At(stampMsg, j);
				ct::logger::info("[Profiler]  Timestamp: %llu\n", stamp->timestamp);
				ct::logger::info("[Profiler]  Encoder position at leading edge: %lld\n", stamp->encoder);
				ct::logger::info("[Profiler]  Frame index: %llu\n", stamp->frameIndex);
				frameIndex = stamp->frameIndex;
			}
		}
		break;
		case GO_DATA_MESSAGE_TYPE_UNIFORM_SURFACE:
		{
			GoSurfaceMsg surfaceMsg = dataObj;

			double XResolution = NM_TO_MM(GoSurfaceMsg_XResolution(surfaceMsg));
			double YResolution = NM_TO_MM(GoSurfaceMsg_YResolution(surfaceMsg));
			double ZResolution = NM_TO_MM(GoSurfaceMsg_ZResolution(surfaceMsg));
			double XOffset = UM_TO_MM(GoSurfaceMsg_XOffset(surfaceMsg));
			double YOffset = UM_TO_MM(GoSurfaceMsg_YOffset(surfaceMsg));
			double ZOffset = UM_TO_MM(GoSurfaceMsg_ZOffset(surfaceMsg));

			//Note: display according to 2D
			ct::logger::info("[Profiler] Resolution X: %fmm", YResolution);
			ct::logger::info("[Profiler] Resolution Y: %fmm", XResolution);
			ct::logger::info("[Profiler] Resolution Z: %fmm", ZResolution);

			auto w = (k32u)GoSurfaceMsg_Width(surfaceMsg);
			auto l = (k32u)GoSurfaceMsg_Length(surfaceMsg);

			frame.width = (int)w;
			frame.height = (int)l;
			frame.bufferSize = w * l;
			frame.type = ct::s_height_map;
			ct::logger::info("[Profiler] Profile size: %d, %d", l, w);


			//allocate memory if needed
			if (info.mHeightMap == M_NULL || mtrx::get_width(info.mHeightMap) != w || mtrx::get_height(info.mHeightMap) != l) {
				mtrx::free_buffer(info.mHeightMap);
				info.mHeightMap = MbufAlloc2d(M_DEFAULT, w, l, 16 + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);
				ct::logger::info("allocate Mbuf");
			}

			MIL_UINT16* hostPtr = M_NULL;
			MIL_ID pitch = M_NULL;
			MbufInquire(info.mHeightMap, M_HOST_ADDRESS, &hostPtr);
			MbufInquire(info.mHeightMap, M_PITCH, &pitch);


			frame.pHeightMap = mtrx::MPM::instance().acquire(w, l, 1, 16 + M_UNSIGNED);
			MIL_UINT16* hostPtr2 = M_NULL;
			MIL_ID pitch2 = M_NULL;
			MbufInquire(frame.pHeightMap->id(), M_HOST_ADDRESS, &hostPtr2);
			MbufInquire(frame.pHeightMap->id(), M_PITCH, &pitch2);



			info.mode = GO_MODE_SURFACE;
			info.resolution_x_mm = XResolution;
			info.resolution_y_mm = YResolution;
			info.resolution_z_mm = ZResolution;
			ct::logger::info("Starting pitch loop");
			for (auto rowIdx = 0; rowIdx < l; rowIdx++) {

				k16s* data = GoSurfaceMsg_RowAt(surfaceMsg, rowIdx);

				for (auto colIdx = 0; colIdx < w; colIdx++) {
					// gocator transmits range data as 16-bit signed integers
					// to translate 16-bit range data to engineering units, the calculation for each point is: 
					//          X: XOffset + columnIndex * XResolution 
					//          Y: YOffset + rowIndex * YResolution
					//          Z: ZOffset + height_map[rowIndex][columnIndex] * ZResolution
					auto x = XOffset + XResolution * colIdx;
					auto y = YOffset + YResolution * rowIdx;
					double z = 0.0;

					if (data[colIdx] != INVALID_RANGE_16BIT) {
						z = ZOffset + ZResolution * data[colIdx];
					}
					else {
						z = INVALID_RANGE_DOUBLE;
					}
					double z16 = z;


					//8bit
					int max_range = 255;

					z = (z + 3) / 6 * max_range; //measuring range is 6mm
					if (z < 0) z = 0;
					if (z > max_range) z = max_range;

					//16bit
					/*
					* Conversion logic
					* z16 = (Zmm + 3) / 6 * 65536
					* z16 / 65536 * 6 - 3 = Zmm
					*/
					int max_range16 = 65536;

					z16 = (z16 + 3) / 6 * max_range16; //measuring range is 6mm
					if (z16 < 0) z16 = 0;
					if (z16 > max_range16) z16 = max_range16;

					hostPtr[colIdx + (rowIdx * pitch)] = z16;

					hostPtr2[colIdx + (rowIdx * pitch2)] = z16;
				}
			}
		}
		break;
		case GO_DATA_MESSAGE_TYPE_SURFACE_INTENSITY:
		{
			GoSurfaceIntensityMsg surfaceIntMsg = dataObj;
			unsigned int rowIdx, colIdx;

			double XResolution = NM_TO_MM(GoSurfaceIntensityMsg_XResolution(surfaceIntMsg));
			double YResolution = NM_TO_MM(GoSurfaceIntensityMsg_YResolution(surfaceIntMsg));
			double XOffset = UM_TO_MM(GoSurfaceIntensityMsg_XOffset(surfaceIntMsg));
			double YOffset = UM_TO_MM(GoSurfaceIntensityMsg_YOffset(surfaceIntMsg));

			ct::logger::info("[Profiler] IResolution X: %fmm", XResolution);
			ct::logger::info("[Profiler] IResolution Y: %fmm", YResolution);

			auto w = (k32u)GoSurfaceIntensityMsg_Width(surfaceIntMsg);
			auto l = (k32u)GoSurfaceIntensityMsg_Length(surfaceIntMsg);
			ct::logger::info("[Profiler] IProfile size: %d, %d", w, l);

			//allocate memory if needed
			if (info.mIntensity == M_NULL || mtrx::get_width(info.mIntensity) != w || mtrx::get_height(info.mIntensity) != l) {
				mtrx::free_buffer(info.mIntensity);
				info.mIntensity = MbufAlloc2d(M_DEFAULT, w, l, 8 + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);
			}

			MIL_UINT8* hostPtr = M_NULL;
			MIL_ID pitch = M_NULL;
			MbufInquire(info.mIntensity, M_HOST_ADDRESS, &hostPtr);
			MbufInquire(info.mIntensity, M_PITCH, &pitch);

			frame.type = ct::s_height_map;
			frame.pImage = mtrx::MPM::instance().acquire(w, l, 1, 8 + M_UNSIGNED);
			MIL_UINT8* hostPtr2 = M_NULL;
			MIL_ID pitch2 = M_NULL;
			MbufInquire(frame.pImage->id(), M_HOST_ADDRESS, &hostPtr2);
			MbufInquire(frame.pImage->id(), M_PITCH, &pitch2);


			info.resolution_x_mm = XResolution;
			info.resolution_y_mm = YResolution;

			for (rowIdx = 0; rowIdx < l; rowIdx++)
			{
				k8u* data = GoSurfaceIntensityMsg_RowAt(surfaceIntMsg, rowIdx);

				// gocator transmits intensity data as an 8-bit grayscale image of identical width and height as the corresponding height map
				for (colIdx = 0; colIdx < w; colIdx++)
				{
					auto x = XOffset + XResolution * colIdx;
					auto y = YOffset + YResolution * rowIdx;
					auto z = data[colIdx];

					hostPtr[colIdx + (rowIdx * pitch)] = z;
					hostPtr2[colIdx + (rowIdx * pitch2)] = z;
				}

			}
		}
		break;
		case GO_DATA_MESSAGE_TYPE_UNIFORM_PROFILE:
		{
			GoResampledProfileMsg profileMsg = dataObj;

			for (unsigned int k = 0; k < GoResampledProfileMsg_Count(profileMsg); ++k)
			{
				unsigned int validPointCount = 0;
				short* data = GoResampledProfileMsg_At(profileMsg, k);
				double XResolution = NM_TO_MM(GoResampledProfileMsg_XResolution(profileMsg));
				double ZResolution = NM_TO_MM(GoResampledProfileMsg_ZResolution(profileMsg));
				double XOffset = UM_TO_MM(GoResampledProfileMsg_XOffset(profileMsg));
				double ZOffset = UM_TO_MM(GoResampledProfileMsg_ZOffset(profileMsg));

				info.mode = GO_MODE_PROFILE;
				frame.profiles.clear();
				frame.type = ct::s_height_snapshot;

				double avg_z = 0.0;

				//translate 16-bit range data to engineering units and copy profiles to memory array
				for (unsigned int arrayIndex = 0; arrayIndex < GoResampledProfileMsg_Width(profileMsg); ++arrayIndex)
				{
					if (data[arrayIndex] != INVALID_RANGE_16BIT)
					{
						profileBuffer[arrayIndex].x = XOffset + XResolution * arrayIndex;
						profileBuffer[arrayIndex].z = ZOffset + ZResolution * data[arrayIndex];
						frame.profiles.push_back(profileBuffer[arrayIndex].z);
						avg_z += profileBuffer[arrayIndex].z;
						validPointCount++;
					}
					else
					{
						profileBuffer[arrayIndex].x = XOffset + XResolution * arrayIndex;
						profileBuffer[arrayIndex].z = INVALID_RANGE_DOUBLE;
					}
				}

				avg_z /= validPointCount;
				ct::logger::info("[Profiler] Snapshot's Average Z: %f", avg_z);
			}
		}
		break;
		}
	}
	ct::logger::info("done pitch loop");
	GoDestroy(dataset);
	ct::logger::info("goDestroy");
	if (frame.type != ct::s_height_snapshot) g_imageQueue.push_back(frame);
	instance->m_softTriggered = false;
	instance->m_conditionVariable.notify_one();
	return kOK;
}

Profiler_Gocator::Profiler_Gocator()
{
	// construct Gocator m_api Library
	if ((m_status = GoSdk_Construct(&m_api)) != kOK)
	{
		m_errorMsg = "Failed to construct m_api";
		ct::logger::error("[Profiler] Failed to construct m_api");
		return;
	}

	// construct GoSystem object
	if ((m_status = GoSystem_Construct(&m_system, kNULL)) != kOK)
	{
		m_errorMsg = "Failed to construct System";
		ct::logger::error("[Profiler] Failed to construct System");
		return;
	}

	//// construct GoAccelerator object
	//if ((m_status = GoAccelerator_Construct(&m_accelerator, kNULL)) != kOK)
	//{
	//	ct::logger::error("[Profiler]: GoAccelerator_Construct:%d\n", m_status);
	//	return false;
	//}

	//// use this call to specify different Web port if 8080 is not available
	//// otherwise you can skip next line....
	//m_status = GoAccelerator_SetWebPort(m_accelerator, 8080);

	//// start Accelerator service
	//if ((m_status = GoAccelerator_Start(m_accelerator)) != kOK)
	//{
	//	ct::logger::error("Error: GoAccelerator_Start:%d\n", m_status);
	//	return false;
	//}
}

Profiler_Gocator::~Profiler_Gocator()
{
	GoDestroy(m_system);
	GoDestroy(m_api);
}

const double Profiler_Gocator::getExposure() const
{
	return m_exposure;
}


const double Profiler_Gocator::getYResolution() const
{
	ct::logger::error("[Profiler_Gocator] Y Resolution not available on Gocator");
	return -1;
}

const QString& Profiler_Gocator::getFirmwareVersion() const
{
	if (!safeGuard()) return "SENSOR_NOT_CONNECTED";

	auto version = (uint32_t)GoSensor_FirmwareVersion(m_sensor);
	return QString("%1").arg(version);
}

const QString& Profiler_Gocator::getSerialNumber() const
{
	return m_serialNumber;
}

bool Profiler_Gocator::isConnected() const
{
	if (!safeGuard()) return false;

	return (bool)GoSensor_IsConnected(m_sensor);
}

const bool Profiler_Gocator::isGrabbing() const
{
	return false;
}

bool Profiler_Gocator::enable(bool enable)
{
	m_enable = true;
	return true;
}

bool Profiler_Gocator::connect(QString info)
{
	strcpy(this->m_ip_string, info.toStdString().c_str());
	return reconnect();
}

bool Profiler_Gocator::disconnect()
{
	return false;
}

bool Profiler_Gocator::start()
{
	if (!safeGuard()) return false;
	if (!verifyConnection()) return false;

	enableDataChannel(true);

	auto state = GoSensor_State(m_sensor);

	if (state == GO_STATE_RUNNING) return true;

	if ((m_status = GoSensor_Start(m_sensor)) != kOK) {
		ct::logger::error("[Profiler] Failed to start m_sensor: %s", getStatus(m_status).toStdString().c_str());
		return false;
	}

	return true;
}

bool Profiler_Gocator::stop()
{
	if (!safeGuard()) return false;
	if (!verifyConnection()) return false;

	if ((m_status = GoSensor_Stop(m_sensor)) != kOK) {
		ct::logger::error("[Profiler] Failed to stop m_sensor: %s", getStatus(m_status).toStdString().c_str());
		return false;
	}

	enableDataChannel(false);

	/*k64s encoder;
	GoSensor_Encoder(m_sensor, &encoder);
	ct::logger::debug("Encoder: %d", encoder);*/

	return true;
}

bool Profiler_Gocator::snapShot()
{
	if (!safeGuard()) return false;
	if (!verifyConnection()) return false;

	enableDataChannel(true);
	setMode(GO_MODE_PROFILE);

	if ((m_status = GoSensor_Snapshot(m_sensor)) != kOK) {
		ct::logger::error("[Profiler] Failed to load job: %s", getStatus(m_status).toStdString().c_str());
		return false;
	}

	enableDataChannel(false);

	return true;
}

bool Profiler_Gocator::enableIntensityMap(bool enable)
{
	if (!safeGuard()) return false;
	if (!verifyConnection()) return false;

	if ((m_status = GoSetup_EnableIntensity(m_setup, enable)) != kOK) {
		ct::logger::error("[Profiler] Failed to enable intensity acquisition: %s", getStatus(m_status).toStdString().c_str());
		return false;
	}

	return true;
}

bool Profiler_Gocator::setScanLength(double mm)
{
	if (!safeGuard()) return false;
	if (!verifyConnection()) return false;

	if ((m_status = GoSetup_SetScanMode(m_setup, GO_MODE_SURFACE)) != kOK) {
		ct::logger::error("[Profiler] Failed to set scan mode: %s", getStatus(m_status).toStdString().c_str());
		return false;
	}

	auto sg = GoSetup_SurfaceGeneration(m_setup);

	if ((m_status = GoSurfaceGenerationFixedLength_SetLength(sg, mm)) != kOK)
	{
		ct::logger::error("[Profiler] Failed to set fixed length: %s", getStatus(m_status).toStdString().c_str());
		return false;
	}
	else {
		ct::logger::debug("[Profiler] Set fixed length: %.3f", mm);
	}

	return true;
}

bool Profiler_Gocator::setGain(double gain)
{
	//does not support
	return true;
}

bool Profiler_Gocator::setDuoHeadGain(double gain, double gain2)
{
	//does not support
	return true;
}


bool Profiler_Gocator::setDivider(int divider)
{
	//does not support
	return true;
}

bool Profiler_Gocator::setExposureMode(ExposureMode mode)
{
	if (!safeGuard()) return false;
	if (!verifyConnection()) return false;

	auto role = GoSensor_Role(m_sensor);

	GoExposureMode gmode = GO_EXPOSURE_MODE_SINGLE;
	if (mode == MULTI) gmode = GO_EXPOSURE_MODE_MULTIPLE;
	else if (mode == DYNAMIC) gmode = GO_EXPOSURE_MODE_DYNAMIC;

	if ((m_status = GoSetup_SetExposureMode(m_setup, role, gmode)) != kOK) {
		ct::logger::error("[Profiler] Failed to set exposure mode: %s", getStatus(m_status).toStdString().c_str());
		return false;
	}

	return true;
}

bool Profiler_Gocator::setExposure(double us)
{
	if (!safeGuard()) return false;
	if (!verifyConnection()) return false;

	auto role = GoSensor_Role(m_sensor);

	if ((m_status = GoSetup_SetExposure(m_setup, role, us)) != kOK) {
		ct::logger::error("[Profiler] Failed to set exposure: %s", getStatus(m_status).toStdString().c_str());
		return false;
	}
	
	ct::logger::info("[Profiler] Set exposure: %.2fus", us);
	return true;
}

bool Profiler_Gocator::setMultiExposure(double us, double us2) 
{
	if (!safeGuard()) return false;
	if (!verifyConnection()) return false;
	
	auto role = GoSensor_Role(m_sensor);

	if ((m_status = GoSetup_ClearExposureSteps(m_setup, role)) != kOK) {
		ct::logger::error("[Profiler] Failed to clear exposure steps: %s", getStatus(m_status).toStdString().c_str());
		return false;
	}

	if ((m_status = GoSetup_AddExposureStep(m_setup, role, us)) != kOK) {
		ct::logger::error("[Profiler] Failed to set exposure1: %s", getStatus(m_status).toStdString().c_str());
		return false;
	}

	if ((m_status = GoSetup_AddExposureStep(m_setup, role, us2)) != kOK) {
		ct::logger::error("[Profiler] Failed to set exposure2: %s", getStatus(m_status).toStdString().c_str());
		return false;
	}

	ct::logger::info("[Profiler] Set multi exposure: %.2fus, %.2fus", us, us2);
	return true;
}

bool Profiler_Gocator::setDynamicExposure(double min_us, double max_us)
{
	if (!safeGuard()) return false;
	if (!verifyConnection()) return false;

	auto role = GoSensor_Role(m_sensor);

	if ((m_status = GoSetup_SetDynamicExposureMin(m_setup, role, min_us)) != kOK) {
		ct::logger::error("[Profiler] Failed to set min exposure: %s", getStatus(m_status).toStdString().c_str());
		return false;
	}

	if ((m_status = GoSetup_SetDynamicExposureMax(m_setup, role, max_us)) != kOK) {
		ct::logger::error("[Profiler] Failed to set max exposure2: %s", getStatus(m_status).toStdString().c_str());
		return false;
	}

	ct::logger::info("[Profiler] Set dynamic exposure: %.2fus, %.2fus", min_us, max_us);
	return true;
}

bool Profiler_Gocator::setParallelExposure(double us, double us2)
{
	//does not support
	return true;
}

bool Profiler_Gocator::setMSR(bool enable)
{
	if (!safeGuard()) return false;
	if (!verifyConnection()) return false;
	if (enable) {
		ct::logger::error("[Profiler] No MSR for Gocator");
		return false;
	}
	return false;

}

bool Profiler_Gocator::setLaserLineThreshold(double threshold)
{
	//does not support
	return true;
}

bool Profiler_Gocator::waitAcquisition(int ms)
{
	if (!m_softTriggered) return true;

	std::unique_lock<std::mutex> lock(m_mutex);

	if (m_conditionVariable.wait_for(lock, std::chrono::milliseconds(ms)) == std::cv_status::no_timeout) return true;

	m_softTriggered = false;

	return false;
}

const FrameInfo& Profiler_Gocator::getFrame() const
{
	return m_frameInfo;
}

FrameInfo& Profiler_Gocator::getFrame()
{
	return m_frameInfo;
}

void Profiler_Gocator::resetFrame()
{
	m_frameInfo = FrameInfo();
}

bool Profiler_Gocator::loadConfig(QString path)
{
	if (!safeGuard()) return false;
	if (!verifyConnection()) return false;

	strcpy(this->m_filename, path.toStdString().c_str());

	if ((m_status = GoSensor_CopyFile(m_sensor, "test.job", GO_SENSOR_LIVE_JOB_NAME)) != kOK) {
		ct::logger::error("[Profiler] Failed to load job: %s", getStatus(m_status).toStdString().c_str());
		return false;
	}

	return true;
}

QString Profiler_Gocator::errorMsg()
{
	return m_errorMsg;
}

QString Profiler_Gocator::getStatus(kStatus status)
{
	QString msg;
	switch (m_status) {
	case 1:
		msg = "Command succeeded";
		break;
	case 0:
		msg = "Command failed";
		break;
	case -1000:
		msg = "Command is not valid in the current state";
		break;
	case -999:
		msg = "A required item (e.g., file) was not found";
		break;
	case -998:
		msg = "Command is not recognized";
		break;
	case -997:
		msg = "One or more command parameters are incorrect";
		break;
	case -996:
		msg = "The operation is not supported";
		break;
	case -992:
		msg = "The simulation buffer is empty";
		break;
	default:
		msg = "Unknown m_status";
	}
	return msg;
}

bool Profiler_Gocator::setMode(GoMode mode)
{
	if (!safeGuard()) return false;
	if (!verifyConnection()) return false;

	auto current_mode = GoSetup_ScanMode(m_setup);

	if (current_mode == mode) return true;

	if ((m_status = GoSetup_SetScanMode(m_setup, mode)) != kOK) {
		ct::logger::error("[Profiler] Failed to set scan mode: %s", getStatus(m_status).toStdString().c_str());
		return false;
	}

	return true;
}

bool Profiler_Gocator::safeGuard() const
{
	if (!m_enable) {
		ct::logger::warn("Profiler not enabled");
		return false;
	}
	if (m_sensor == kNULL) return false;
	return true;
}

bool Profiler_Gocator::verifyConnection()
{
	if (!isConnected()) {
		return reconnect();
	}

	return true;
}

bool Profiler_Gocator::reconnect()
{
	// Parse IP address into address data structure
	kIpAddress_Parse(&m_ipAddress, m_ip_string);

	// obtain GoSensor object by IP address
	if ((m_status = GoSystem_FindSensorByIpAddress(m_system, &m_ipAddress, &m_sensor)) != kOK)
	{
		m_errorMsg = "Failed to find m_sensor with the IP Address: ";
		m_errorMsg.append(m_ip_string);
		ct::logger::error("[Profiler] %s", m_errorMsg.toStdString().c_str());
		return false;
	}

	if (!safeGuard()) return false;

	// create connection to GoSensor object
	if ((m_status = GoSensor_Connect(m_sensor)) != kOK)
	{
		m_errorMsg = "Failed to connect m_sensor";
		return false;
	}

	// retrieve setup handle
	if ((m_setup = GoSensor_Setup(m_sensor)) == kNULL)
	{
		m_errorMsg = "Failed to setup handle";
		return false;
	}

	// retrieve setup handle
	if ((m_layout = GoSetup_Layout(m_setup)) == kNULL)
	{
		m_errorMsg = "Failed to get layout";
		return false;
	}

	GoSensor_SetDataHandler(m_sensor, Callback, this);
	enableDataChannel(false);

	return true;
}

bool Profiler_Gocator::enableDataChannel(bool enable)
{
	if (!safeGuard()) return false;
	if (!verifyConnection()) return false;

	if ((m_status = GoSystem_EnableData(m_system, enable)) != kOK)
	{
		ct::logger::error("[Profiler] Failed to enable data channel: %s", getStatus(m_status).toStdString().c_str());
		return false;
	}

	return true;
}
