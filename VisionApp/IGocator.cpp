#include "IGocator.h"
#include "Logger.h"
#include <iostream>
#include <type_traits>
#include "QOStool.h"
#include <QDebug>
#include "MbufPoolManager.h"

#define INVALID_RANGE_16BIT     ((signed short)0x8000)          // gocator transmits range data as 16-bit signed integers. 0x8000 signifies invalid range data. 
#define DOUBLE_MAX              ((k64f)1.7976931348623157e+308) // 64-bit double - largest positive value.  
#define INVALID_RANGE_DOUBLE    ((k64f)-DOUBLE_MAX)             // floating point value to represent invalid range data.

#define NM_TO_MM(VALUE) (((k64f)(VALUE))/1000000.0)
#define UM_TO_MM(VALUE) (((k64f)(VALUE))/1000.0)

#define RECEIVE_TIMEOUT         5000000 

FrameInfo g_Info;
double g_rotationAngle;
IGocator IGocator::m_instance;

typedef struct ProfilePoint
{
	double x;   // x-coordinate in engineering units (mm) - position along laser line
	double y;   // y-coordinate in engineering units (mm) - position along the direction of travel
	double z;   // z-coordinate in engineering units (mm) - height (at the given x position)
	unsigned char intensity;
} ProfilePoint;

kStatus kCall data_callback(kPointer context, GoSensor m_sensor, GoDataSet dataset)
{
	ct::GoInfo* info = (ct::GoInfo*)context;

	ct::logger::info("Enter Laser Callback");

	GoDataMsg dataObj;
	GoStamp *stamp = kNULL;
	GoMeasurementData *measurementData = kNULL;
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

	FrameInfo frame;
	frame.type = g_Info.type;
	frame.viewID = g_Info.viewID;
	frame.opticID = g_Info.opticID;
	frame.stitchID = g_Info.stitchID;
	frame.postTask.rotationalAngle = g_rotationAngle;
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
				GoStamp *stamp = GoStampMsg_At(stampMsg, j);
				ct::logger::info("[Laser]  Timestamp: %llu\n", stamp->timestamp);
				ct::logger::info("[Laser]  Encoder position at leading edge: %lld\n", stamp->encoder);
				ct::logger::info("[Laser]  Frame index: %llu\n", stamp->frameIndex);
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
			ct::logger::info("[Laser] Resolution X: %fmm", YResolution);
			ct::logger::info("[Laser] Resolution Y: %fmm", XResolution);
			ct::logger::info("[Laser] Resolution Z: %fmm", ZResolution);

			auto w = (k32u)GoSurfaceMsg_Width(surfaceMsg);
			auto l = (k32u)GoSurfaceMsg_Length(surfaceMsg);

			frame.width = (int)w;
			frame.height = (int)l;
			frame.bufferSize = w * l;
			ct::logger::info("[Laser] Profile size: %d, %d", l, w);


			//allocate memory if needed

			if (info->mHeightMap == M_NULL || mtrx::get_width(info->mHeightMap) != w || mtrx::get_height(info->mHeightMap) != l) {
				mtrx::free_buffer(info->mHeightMap);
				info->mHeightMap = MbufAlloc2d(M_DEFAULT, w, l, 16 + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);
			}

			MIL_UINT16 *hostPtr = M_NULL;
			MIL_ID pitch = M_NULL;
			MbufInquire(info->mHeightMap, M_HOST_ADDRESS, &hostPtr);
			MbufInquire(info->mHeightMap, M_PITCH, &pitch);

			frame.pHeightMap = mtrx::MPM::instance().acquire(w, l, 1, 16 + M_UNSIGNED);
			MIL_UINT16* hostPtr2 = M_NULL;
			MIL_ID pitch2 = M_NULL;
			MbufInquire(frame.pHeightMap->id(), M_HOST_ADDRESS, &hostPtr2);
			MbufInquire(frame.pHeightMap->id(), M_PITCH, &pitch2);



			info->mode = GO_MODE_SURFACE;
			info->resolution_x_mm = XResolution;
			info->resolution_y_mm = YResolution;
			info->resolution_z_mm = ZResolution;

			for (auto rowIdx = 0; rowIdx < l; rowIdx++) {
				
				k16s *data = GoSurfaceMsg_RowAt(surfaceMsg, rowIdx);

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
					} else {
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

			ct::logger::info("[Laser] IResolution X: %fmm", XResolution);
			ct::logger::info("[Laser] IResolution Y: %fmm", YResolution);

			auto w = (k32u)GoSurfaceIntensityMsg_Width(surfaceIntMsg);
			auto l = (k32u)GoSurfaceIntensityMsg_Length(surfaceIntMsg);
			ct::logger::info("[Laser] IProfile size: %d, %d", w, l);

			//allocate memory if needed

			if (info->mIntensity == M_NULL || mtrx::get_width(info->mIntensity) != w || mtrx::get_height(info->mIntensity) != l) {
				mtrx::free_buffer(info->mIntensity);
				info->mIntensity = MbufAlloc2d(M_DEFAULT, w, l, 8 + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);
			}

			MIL_UINT8 *hostPtr = M_NULL;
			MIL_ID pitch = M_NULL;
			MbufInquire(info->mIntensity, M_HOST_ADDRESS, &hostPtr);
			MbufInquire(info->mIntensity, M_PITCH, &pitch);


			frame.pImage = mtrx::MPM::instance().acquire(w, l, 1, 8 + M_UNSIGNED);
			MIL_UINT8* hostPtr2 = M_NULL;
			MIL_ID pitch2 = M_NULL;
			MbufInquire(frame.pImage->id(), M_HOST_ADDRESS, &hostPtr2);
			MbufInquire(frame.pImage->id(), M_PITCH, &pitch2);


			info->resolution_x_mm = XResolution;
			info->resolution_y_mm = YResolution;

			for (rowIdx = 0; rowIdx < l; rowIdx++)
			{
				k8u *data = GoSurfaceIntensityMsg_RowAt(surfaceIntMsg, rowIdx);

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

				info->mode = GO_MODE_PROFILE;
				info->profiles.clear();

				double avg_z = 0.0;

				//translate 16-bit range data to engineering units and copy profiles to memory array
				for (unsigned int arrayIndex = 0; arrayIndex < GoResampledProfileMsg_Width(profileMsg); ++arrayIndex)
				{
					if (data[arrayIndex] != INVALID_RANGE_16BIT)
					{
						profileBuffer[arrayIndex].x = XOffset + XResolution * arrayIndex;
						profileBuffer[arrayIndex].z = ZOffset + ZResolution * data[arrayIndex];
						info->profiles.push_back(profileBuffer[arrayIndex].z);
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
				ct::logger::info("[Laser] Snapshot's Average Z: %f", avg_z);
			}
		}
		break;
		}
	}

	GoDestroy(dataset);

	info->fnc();

	g_imageQueue.push_back(frame);

	return kOK;
}

IGocator::IGocator()
{
}

IGocator::~IGocator()
{
}

IGocator& IGocator::instance()
{
	return m_instance;
}

bool IGocator::init(std::string ip)
{
	// construct Gocator m_api Library
	if ((m_status = GoSdk_Construct(&m_api)) != kOK)
	{
		m_error_msg = "Failed to construct m_api";
		ct::logger::error("[Laser] Failed to construct m_api");
		return false;
	}
	
	// construct GoSystem object
	if ((m_status = GoSystem_Construct(&m_system, kNULL)) != kOK)
	{
		m_error_msg = "Failed to construct System";
		ct::logger::error("[Laser] Failed to construct System");
		return false;
	}

	//strcpy(this->m_ip_string, ip.c_str());
	ct::logger::debug("Laser IP: %s", ip.c_str());
	strcpy(this->m_ip_string, ip.c_str());

	//// construct GoAccelerator object
	//if ((m_status = GoAccelerator_Construct(&m_accelerator, kNULL)) != kOK)
	//{
	//	ct::logger::error("[Laser]: GoAccelerator_Construct:%d\n", m_status);
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

	return reconnect();
}

bool IGocator::release()
{
	m_goInfo.reset();
	GoDestroy(m_system);
	GoDestroy(m_api);
	return true;
}

bool IGocator::flush()
{
	if (!safe_guard()) return false;
	if (!verify_connection()) return false;

	if ((m_status = GoSensor_Flush(m_sensor)) != kOK) {
		ct::logger::error("[Laser] Failed to flush m_sensor: %s", get_status_msg(m_status).c_str());
		return false;
	}

	return true;
}

bool IGocator::reset_sensor()
{
	if (!safe_guard()) return false;
	if (!verify_connection()) return false;

	if ((m_status = GoSensor_Reset(m_sensor, true)) != kOK) {
		ct::logger::error("[Laser] Failed to reset m_sensor: %s", get_status_msg(m_status).c_str());
		return false;
	}

	return true;
}

bool IGocator::refresh()
{
	if (!safe_guard()) return false;
	if (!verify_connection()) return false;

	if ((m_status = GoSensor_Refresh(m_sensor)) != kOK) {
		ct::logger::error("[Laser] Failed to refresh m_sensor: %s", get_status_msg(m_status).c_str());
		return false;
	}

	return true;
}

bool IGocator::reconnect()
{
	// Parse IP address into address data structure
	kIpAddress_Parse(&m_ipAddress, m_ip_string);
	
	// obtain GoSensor object by IP address
	if ((m_status = GoSystem_FindSensorByIpAddress(m_system, &m_ipAddress, &m_sensor)) != kOK)
	{
		m_error_msg = "Failed to find m_sensor with the IP Address: ";
		m_error_msg.append(m_ip_string);
		ct::logger::error("[Laser] %s", m_error_msg.c_str());
		return false;
	}

	if (!safe_guard()) return false;
	
	// create connection to GoSensor object
	if ((m_status = GoSensor_Connect(m_sensor)) != kOK)
	{
		m_error_msg = "Failed to connect m_sensor";
		return false;
	}
	
	// retrieve setup handle
	if ((m_setup = GoSensor_Setup(m_sensor)) == kNULL)
	{
		m_error_msg = "Failed to setup handle";
		return false;
	}

	// retrieve setup handle
	if ((m_layout = GoSetup_Layout(m_setup)) == kNULL)
	{
		m_error_msg = "Failed to get layout";
		return false;
	}

	GoSensor_SetDataHandler(m_sensor, data_callback, &m_goInfo);
	enable_data_channel(false);
	
	return true;
}

bool IGocator::start()
{
	if (!safe_guard()) return false;
	if (!verify_connection()) return false;

	enable_data_channel(true);

	auto state = GoSensor_State(m_sensor);
	//ct::logger::debug("State1: %d", state);
	if (state == GO_STATE_RUNNING) return true;

	if ((m_status = GoSensor_Start(m_sensor)) != kOK) {
		ct::logger::error("[Laser] Failed to start m_sensor: %s", get_status_msg(m_status).c_str());
		return false;
	}

	return true;
}

bool IGocator::stop()
{
	if (!safe_guard()) return false;
	if (!verify_connection()) return false;

	if ((m_status = GoSensor_Stop(m_sensor)) != kOK) {
		ct::logger::error("[Laser] Failed to stop m_sensor: %s", get_status_msg(m_status).c_str());
		return false;
	}

	enable_data_channel(false);

	/*k64s encoder;
	GoSensor_Encoder(m_sensor, &encoder);
	ct::logger::debug("Encoder: %d", encoder);*/

	return true;
}

bool IGocator::enable_data_channel(bool enable)
{
	if (!safe_guard()) return false;
	if (!verify_connection()) return false;

	if ((m_status = GoSystem_EnableData(m_system, enable)) != kOK)
	{
		ct::logger::error("[Laser] Failed to enable data channel: %s", get_status_msg(m_status).c_str());
		return false;
	}

	return true;
}

bool IGocator::set_fixed_length(double mm)
{
	if (!safe_guard()) return false;
	if (!verify_connection()) return false;

	if ((m_status = GoSetup_SetScanMode(m_setup, GO_MODE_SURFACE)) != kOK) {
		ct::logger::error("[Laser] Failed to set scan mode: %s", get_status_msg(m_status).c_str());
		return false;
	}

	auto sg = GoSetup_SurfaceGeneration(m_setup);

	if ((m_status = GoSurfaceGenerationFixedLength_SetLength(sg, mm)) != kOK)
	{
		ct::logger::error("[Laser] Failed to set fixed length: %s", get_status_msg(m_status).c_str());
		return false;
	}
	else {
		ct::logger::debug("[Laser] Set fixed length: %.3f", mm);
	}

	return true;
}

bool IGocator::set_exposure(int us)
{
	if (!safe_guard()) return false;
	if (!verify_connection()) return false;

	auto role = GoSensor_Role(m_sensor);

	ct::logger::info("[Laser] Set exposure: %dus", us);
	if ((m_status = GoSetup_SetExposure(m_setup, role, us)) != kOK) {
		ct::logger::error("[Laser] Failed to set exposure: %s", get_status_msg(m_status).c_str());
		return false;
	}

	return true;
}

bool IGocator::enable_intensity(bool enable)
{
	if (!safe_guard()) return false;
	if (!verify_connection()) return false;

	if ((m_status = GoSetup_EnableIntensity(m_setup, enable)) != kOK) {
		ct::logger::error("[Laser] Failed to enable intensity acquisition: %s", get_status_msg(m_status).c_str());
		return false;
	}

	return true;
}

bool IGocator::snapshot()
{
	if (!safe_guard()) return false;
	if (!verify_connection()) return false;

	enable_data_channel(true);
	set_scan_mode(GO_MODE_PROFILE);

	if ((m_status = GoSensor_Snapshot(m_sensor)) != kOK) {
		ct::logger::error("[Laser] Failed to load job: %s", get_status_msg(m_status).c_str());
		return false;
	}

	enable_data_channel(false);

	return true;
}

bool IGocator::load_job(std::string filename)
{
	if (!safe_guard()) return false;
	if (!verify_connection()) return false;

	strcpy(this->m_filename, filename.c_str());

	if ((m_status = GoSensor_CopyFile(m_sensor, this->m_filename, GO_SENSOR_LIVE_JOB_NAME)) != kOK) {
		ct::logger::error("[Laser] Failed to load job: %s", get_status_msg(m_status).c_str());
		return false;
	}

	return true;
}

bool IGocator::save_job()
{
	if (!safe_guard()) return false;
	if (!verify_connection()) return false;

	strcpy(this->m_filename, loaded_job().c_str());

	if ((m_status = GoSensor_CopyFile(m_sensor, GO_SENSOR_LIVE_JOB_NAME, this->m_filename)) != kOK) {
		ct::logger::error("[Laser] Failed to save job: %s", get_status_msg(m_status).c_str());
		return false;
	}

	return true;
}

bool IGocator::download(std::string filename, std::string dst_path)
{
	if (!safe_guard()) return false;
	if (!verify_connection()) return false;

	strcpy(this->m_filename, filename.c_str());
	strcpy(this->m_path, dst_path.c_str());

	if ((m_status = GoSensor_DownloadFile(m_sensor, this->m_filename, m_path)) != kOK) {
		ct::logger::error("[Laser] Failed to download: %s", get_status_msg(m_status).c_str());
		return false;
	}

	return true;
}

bool IGocator::upload(std::string filename, std::string src_path)
{
	if (!safe_guard()) return false;
	if (!verify_connection()) return false;

	strcpy(this->m_filename, filename.c_str());
	strcpy(this->m_path, src_path.c_str());

	if ((m_status = GoSensor_UploadFile(m_sensor, this->m_filename, m_path)) != kOK) {
		ct::logger::error("[Laser] Failed to upload: %s", get_status_msg(m_status).c_str());
		return false;
	}

	return true;
}

bool IGocator::copy(std::string src_file, std::string dst_file)
{
	if (!safe_guard()) return false;
	if (!verify_connection()) return false;

	strcpy(this->m_filename, src_file.c_str());
	strcpy(this->m_path, dst_file.c_str());
	if ((m_status = GoSensor_CopyFile(m_sensor, this->m_filename, m_path)) != kOK)
	{
		ct::logger::error("[Laser] Failed to copy: %s", get_status_msg(m_status).c_str());
		return false;
	}

	return true;
}

bool IGocator::set_default_job(std::string filename)
{
	if (!safe_guard()) return false;
	if (!verify_connection()) return false;

	strcpy(this->m_filename, filename.c_str());
	if ((m_status = GoSensor_SetDefaultJob(m_sensor, this->m_filename)) != kOK)
	{
		ct::logger::error("[Laser] Failed to set default job: %s", get_status_msg(m_status).c_str());
		return false;
	}

	return true;
}

bool IGocator::clear_recordings()
{
	if (!safe_guard()) return false;
	if (!verify_connection()) return false;

	if ((m_status = GoSensor_ClearReplayData(m_sensor)) != kOK)
	{
		ct::logger::error("[Laser] Failed to clear recording : %s", get_status_msg(m_status).c_str());
		return false;
	}

	return true;
}

bool IGocator::record(bool on)
{
	if (!safe_guard()) return false;
	if (!verify_connection()) return false;

	if (on) {
		auto input = GoSensor_InputSource(m_sensor);
		if (input == GO_INPUT_SOURCE_RECORDING) {
			if ((m_status = GoSensor_SetInputSource(m_sensor, GO_INPUT_SOURCE_LIVE)) != kOK) {
				ct::logger::error("[Laser] Failed to turn off replay mode before recording: %s", get_status_msg(m_status).c_str());
				return false;
			}
		}
	}

	if ((m_status = GoSensor_EnableRecording(m_sensor, on)) != kOK)
	{
		if (on) ct::logger::error("[Laser] Failed to start recording : %s", get_status_msg(m_status).c_str());
		else ct::logger::error("[Laser] Failed to stop recording: %s", get_status_msg(m_status).c_str());
		return false;
	}

	return true;
}

bool IGocator::is_connected()
{
	if (!safe_guard()) return false;

	return (bool)GoSensor_IsConnected(m_sensor);
}

bool IGocator::file_exists(std::string filename)
{
	if (!safe_guard()) return false;
	if (!verify_connection()) return false;

	return (bool)GoSensor_FileExists(m_sensor, filename.c_str());
}

std::vector<std::string> IGocator::get_filenames()
{
	std::vector<std::string> filenames;
	if (!safe_guard()) return filenames;
	if (!verify_connection()) return filenames;

	for (int i = 0; i < GoSensor_FileCount(m_sensor); i++) {
		if (GoSensor_FileNameAt(m_sensor, i, m_filename, FILENAME_MAX) == kOK) {
			filenames.emplace_back(m_filename);
		}
	}

	return filenames;
}

bool IGocator::delete_file(std::string filename)
{
	if (!safe_guard()) return false;
	if (!verify_connection()) return false;

	strcpy(this->m_filename, filename.c_str());
	if ((m_status = GoSensor_DefaultJob(m_sensor, this->m_filename, FILENAME_MAX)) != kOK) {
		ct::logger::error("[Laser] Failed to delete file: %s", get_status_msg(m_status).c_str());
		return false;
	}

	return true;
}

GoOrientation IGocator::orientation()
{
	return GoLayout_Orientation(m_layout);
}

bool IGocator::set_orientation(GoOrientation o)
{
	if ((m_status = GoLayout_SetOrientation(m_layout, o)) != kOK) {
		ct::logger::error("[Laser] Failed to set orientation: %s", get_status_msg(m_status).c_str());
		return false;
	}
	return true;
}

void IGocator::set_rotationAngle(double rotationAngle)
{
	g_rotationAngle = rotationAngle;
}

std::string IGocator::default_job()
{
	if (!safe_guard()) return "SENSOR_NOT_CONNECTED";
	if (!verify_connection()) return "SENSOR_NOT_CONNECTED";

	if ((m_status = GoSensor_DefaultJob(m_sensor, m_filename, FILENAME_MAX)) != kOK) {
		ct::logger::error("[Laser] Failed to get default job: %s", get_status_msg(m_status).c_str());
		return "INVALID_JOB";
	}

	return m_filename;
}

std::string IGocator::loaded_job()
{
	if (!safe_guard()) return "SENSOR_NOT_CONNECTED";
	if (!verify_connection()) return "SENSOR_NOT_CONNECTED";

	kBool changed;
	if ((m_status = GoSensor_LoadedJob(m_sensor, m_filename, FILENAME_MAX, &changed)) != kOK) {
		ct::logger::error("[Laser] Failed to get loaded job: %s", get_status_msg(m_status).c_str());
		return "INVALID_JOB";
	}

	return m_filename;
}

std::string IGocator::version()
{
	if (!safe_guard()) return "SENSOR_NOT_CONNECTED";

	auto version = (uint32_t)GoSensor_FirmwareVersion(m_sensor);
	return std::to_string(version);
}

std::string IGocator::part_number()
{
	if (!safe_guard()) return "SENSOR_NOT_CONNECTED";

	GoSensor_PartNumber(m_sensor, m_filename, FILENAME_MAX);
	return m_filename;
}

std::string IGocator::error()
{
	return m_error_msg;
}

ct::GoInfo & IGocator::go_info()
{
	return m_goInfo;
}

void IGocator::test()
{
	if (!safe_guard()) return;

	enable_data_channel(true);
	set_scan_mode(GO_MODE_PROFILE);
	GoSensor_Snapshot(m_sensor);
	enable_data_channel(false);
	//GoSensor_Align(m_sensor);

	//stop();

	//kStatus status;
	//ct::logger::debug("1");
	//// configure Gocator to be in stationary alignment mode
	//if ((status = GoSetup_SetAlignmentType(m_setup, GO_ALIGNMENT_TYPE_STATIONARY)) != kOK)
	//{
	//	ct::logger::error("Error: GoSetup_SetAlignmentType:%d\n", status);
	//	return;
	//}
	//ct::logger::debug("2");
	//// configure stationary alignment target to be flat surface
	//if ((status = GoSetup_SetAlignmentStationaryTarget(m_setup, GO_ALIGNMENT_TARGET_NONE)) != kOK)
	//{
	//	ct::logger::error("Error: GoSetup_SetAlignmentStationaryTarget:%d\n", status);
	//	return;
	//}
	//ct::logger::debug("3");

	//start();
	//// start Gocator alignment  
	//if ((status = GoSensor_Align(m_sensor)) != kOK)
	//{
	//	ct::logger::error("Error: GoSystem_StartAlignment:%d\n", status);
	//	return;
	//}

	//ct::logger::debug("4");

	//GoDataSet dataset;

	//if ((status = GoSystem_ReceiveData(m_system, &dataset, RECEIVE_TIMEOUT)) == kOK)
	//{
	//	for (int i = 0; i < GoDataSet_Count(dataset); ++i)
	//	{
	//		GoDataMsg dataObj = GoDataSet_At(dataset, i);
	//		if (GoDataMsg_Type(dataObj) == GO_DATA_MESSAGE_TYPE_ALIGNMENT)
	//		{
	//			GoAlignMsg alignMsg = dataObj;
	//			if (GoAlignMsg_Status(alignMsg) == kOK)
	//			{
	//				ct::logger::error("Alignment calibration complete.\n\n");
	//			}
	//			else
	//			{
	//				ct::logger::error("Alignment calibration failed.\n\n");
	//			}
	//		}
	//	}
	//	GoDestroy(dataset);
	//}
	//else if (status == kERROR_TIMEOUT)
	//{
	//	ct::logger::error("Failed to collect date for calibration within timeout limit...\n");
	//}
	//else
	//{
	//	ct::logger::error("Error: GoSystem_ReceiveData:%d\n", status);
	//}

	//if ((status = GoSystem_Stop(m_system)) != kOK)
	//{
	//	ct::logger::error("Error: GoSystem_Stop:%d\n", status);
	//	return;
	//}

	//stop();
	//ct::logger::debug("5");
}

FrameInfo* IGocator::getFrame()
{
	return &g_Info;
}

std::string IGocator::get_status_msg(kStatus m_status)
{
	std::string msg;
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

bool IGocator::set_scan_mode(GoMode mode)
{
	auto current_mode = GoSetup_ScanMode(m_setup);

	if (current_mode == mode) return true;

	if ((m_status = GoSetup_SetScanMode(m_setup, mode)) != kOK) {
		ct::logger::error("[Laser] Failed to set scan mode: %s", get_status_msg(m_status).c_str());
		return false;
	}

	return true;
}

bool IGocator::safe_guard()
{
	if (m_sensor == kNULL) return false;
	return true;
}

bool IGocator::verify_connection()
{
	if (!is_connected()) {
		return reconnect();
	}

	return true;
}

/*
=> Using unsigned char
if (frame->_width != w && frame->_height != l)
{
frame->_width = w;
frame->_height = l;

if (frame->_pImageBuf != nullptr) {
delete[] frame->_pImageBuf;
frame->_pImageBuf = nullptr;
}

frame->_pImageBuf = new unsigned char[w*l];
}

QImage image(w, l, QImage::Format_RGB32);

for (auto rowIdx = 0; rowIdx < l; rowIdx++) {

k16s *data = GoSurfaceMsg_RowAt(surfaceMsg, rowIdx);

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
} else {
z = INVALID_RANGE_DOUBLE;
}

if (z < 0) z = 0;

z = z / 6 * 255;

frame->_pImageBuf[colIdx + (rowIdx * w)] = z;
image.setPixel(colIdx, rowIdx, qRgb(z, z, z));
}
}

auto qimg = QImage(frame->_pImageBuf, frame->_width, frame->_height, QImage::Format_RGB32);
qimg.save("test.jpg");
image.save("qimg.jpg");
*/