#include "Motion_APS.h"
#include "APS Library/Include/APS168.h"
#include "APS Library/Include/APS_Define.h"
#include "APS Library/Include/ErrorCodeDef.h"
#include "Logger.h"
#include "MachineController.h"

using namespace nvs::motion;

int Motion_APS::get_group(int bit) const
{
	if (bit < 0 || bit > 31) return -1; // invalid channel
	return bit / 8;
}

const char* Motion_APS::get_code(int ret) const
{
	switch (ret)
	{
	case 0: return "No error";
	case ERR_OSVersion: return "Operating system type mismatched";
	case ERR_OpenDriverFailed: return "Failed to open device driver";
	case ERR_InsufficientMemory: return "Insufficient system memory";
	case ERR_DeviceNotInitial: return "Device not initialized";
	case ERR_NoDeviceFound: return "No device found in the system";
	case ERR_CardIdDuplicate: return "Card ID duplicated";
	case ERR_DeviceAlreadyInitialed: return "Device already initialized";
	case ERR_InterruptNotEnable: return "Interrupt not enabled or initialized";
	case ERR_TimeOut: return "Operation timed out";
	case ERR_HandshakeAckTimeout: return "Handshake acknowledgement timed out";
	case ERR_ParametersInvalid: return "Invalid input parameters";
	//case ERR_InitialModeParamInvalid: return "Invalid initial mode parameter";
	case ERR_SetEEPROM: return "Failed to set data to EEPROM";
	case ERR_GetEEPROM: return "Failed to read data from EEPROM";
	case ERR_FunctionNotAvailable: return "Function not available or unsupported";
	case ERR_HandshakeRespNotEqualCmd: return "Handshake response not equal to command";
	case ERR_NotSupportReuseMode: return "Setting type does not support reuse mode";
	//case ERR_NoCompatibleResource: return "No compatible resource found";
	case ERR_FirmwareError: return "Firmware error, please reboot system";
	case ERR_CommandInProcess: return "Previous command still in process";
	case ERR_PushCompareDataNotFinish: return "Previous comparison data not fully pushed to FIFO";
	case ERR_AxisIdDuplicate: return "Axis ID duplicated";
	case ERR_ModuleNotFound: return "Slave module not found";
	case ERR_InsufficientModuleNo: return "Insufficient module number";
	case ERR_HandShakeFailed: return "Handshake with DSP timed out";
	case ERR_FILE_FORMAT: return "Configuration file format error";
	case ERR_ParametersReadOnly: return "Parameter is read-only";
	case ERR_DistantNotEnough: return "Distance not sufficient for motion";
	case ERR_FunctionNotEnable: return "Function not enabled";
	case ERR_ServerAlreadyClose: return "Server already closed";
	case ERR_DllNotFound: return "Required DLL not found or in wrong path";
	case ERR_Emx_Offset: return "EMX offset error";
	case ERR_Smp_App_Existed: return "Sample application already exists";
	case ERR_Init_Error: return "Initialization error";
	case ERR_No_Init: return "Not initialized";
	case ERR_No_Setup: return "Setup not completed";
	case ERR_Input_Error: return "Invalid input";
	case ERR_Status_Not_Ready: return "Status not ready";
	case ERR_Axis_Busy: return "Axis is busy";
	case ERR_Network_Error: return "Network error";
	case ERR_Network_Time_Out: return "Network timeout";
	case ERR_Crc_Fail: return "CRC check failed";
	case ERR_Param_Invalid: return "Invalid parameter";
	case ERR_No_Servo_On: return "Servo not on";
	case ERR_Api_Timeout: return "API call timeout";
	case ERR_Load_Xml_Mismatch: return "Loaded XML does not match device configuration";
	case ERR_Fifo_Access_Success: return "FIFO access success";
	default: return "Unknown error code";
	}
}

bool Motion_APS::log_error_code(const char* msg, int ret) const
{
	if (ret == ERR_NoError) return false;
	m_errorMsg = std::string(msg) + ":" + std::string(get_code(ret));
	ct::logger::error("[Motion_APS] %s: %s", msg, get_code(ret));
	return true;
}

bool Motion_APS::invalid_axis(const char* msg, int axis) const
{
	if (axis < m_totalAxis) return false;
	m_errorMsg = std::string(msg) + ": Invalid axis ID " + std::to_string(axis);
	ct::logger::error("[Motion_APS] %s", m_errorMsg.c_str());
	return true;
}

double Motion_APS::to_mm(int axis, double pulse) const
{
	auto it = m_axisInfos.find(axis);
	if (it == m_axisInfos.end()) {
		ct::logger::error("[Motion_APS] No axis info for axis %d, skip unit conversion", axis);
		return pulse;
	}
	return pulse / it->second.pulse_per_mm;
}

double Motion_APS::to_pulse(int axis, double mm) const
{
	auto it = m_axisInfos.find(axis);
	if (it == m_axisInfos.end()) {
		ct::logger::error("[Motion_APS] No axis info for axis %d, skip unit conversion", axis);
		return mm;
	}
	return mm * it->second.pulse_per_mm;
}

Motion_APS::Motion_APS()
{
}

Motion_APS::~Motion_APS()
{
}

bool Motion_APS::init()
{
	I32 mode = INIT_AUTO_CARD_ID;
	I32 ret = ERR_NoError;
	I32 cardBit, cardName, firstAxisID, totalAxis;

	ret = APS_register_emx(1, 0);
	if (log_error_code("Failed to register EMX", ret)) return false;


	ret = APS_initial(&cardBit, mode);
	if (log_error_code("Failed to initialize EMX", ret)) return false;
	

	for (I32 i = 0; i < 16; i++) {
		if ((cardBit >> i) & 1) {
			APS_get_card_name(i, &cardName);
			
			if (cardName == 23) { //23:EMX-100
				m_cardIDs.push_back(i);
				ct::logger::info("Card ID: %lu", i);
			}
		}
	}

	if (m_cardIDs.size() == 0) {
		if (log_error_code("Failed to obtain card ID", ret)) return false;
	}

	m_is_init = true; 

	m_axisInfos.clear();

	for (auto cardID : m_cardIDs) {
		APS_get_first_axisId(cardID, &firstAxisID, &totalAxis);

		m_totalAxis += totalAxis;

		m_axisInfos[cardID] = AxisInfo();
	}

	ct::logger::info("[Motion_APS] Init sucess");

	return true;
}

bool Motion_APS::release()
{
	auto ret = APS_close();
	m_is_init = false;

	if (log_error_code("Failed to close EMX", ret)) return false;

	return true;
}

std::optional<std::string> Motion_APS::version(int cardID) const
{
	I32 DLL_version, software_version, middleware_version, AP_version, library_version;

	// Get DLL Version
	DLL_version = APS_version();

	// Get EMX100 software version
	auto ret = APS_get_device_info(cardID, 0x00, &software_version);
	if (log_error_code("Failed to get EMX software version", ret)) return std::nullopt;


	// Get EMX100 middleware version
	ret = APS_get_device_info(cardID, 0x01, &middleware_version);
	if (log_error_code("Failed to get EMX middleware version", ret)) return std::nullopt;


	// Get EMX100 communication AP version
	ret = APS_get_device_info(cardID, 0x11, &AP_version);
	if (log_error_code("Failed to get EMX AP version", ret)) return std::nullopt;


	// Get EMX100 library version
	ret = APS_get_device_info(cardID, 0x100, &library_version);
	if (log_error_code("Failed to get EMX library version", ret)) return std::nullopt;
	
	ct::logger::info("APS version: %lu", DLL_version);
	ct::logger::info("Software version: %lu.%lu.%lu.%lu", software_version, middleware_version, AP_version, library_version);
	/*std::string version = QString("%1.%2.%3.%4.%5")
		.arg(DLL_version)
		.arg(software_version)
		.arg(middleware_version)
		.arg(AP_version)
		.arg(library_version);*/
	std::string version;

	return version;
}

std::optional<double> Motion_APS::get_position_mm(int cardID, int axis) const
{
	I32 pos;
	auto ret = APS_get_position(axis, &pos);
	if (log_error_code("Failed to get position", ret)) return std::nullopt;

	auto pos_mm = to_mm(axis, pos);

	return pos_mm;
}

std::optional<double> Motion_APS::get_current_speed(int cardID, int axis) const
{
	I32 vel;
	auto ret = APS_get_feedback_velocity(axis, &vel);
	if (log_error_code("Failed to get speed", ret)) return std::nullopt;

	auto vel_mm = to_mm(axis, vel);

	return vel_mm;
}

std::optional<bool> Motion_APS::get_DI(int cardID, int bit) const
{
	if (bit >= MAX_DI) {
		ct::logger::error("[Motion_APS] Failed to read DI: Index out of range");
		return false;
	}

	I32 DI;
	auto ret = APS_read_d_input(cardID, get_group(bit), &DI);
	if (log_error_code("Failed to read DI", ret)) return std::nullopt;

	return (DI >> bit) & 1;
}

std::optional<bool> Motion_APS::get_DO(int cardID, int bit) const
{
	if (bit >= MAX_DO) {
		ct::logger::error("[Motion_APS] Failed to read DO: Index out of range");
		return false;
	}

	int normalized_bit = ((bit % 8) + 8) % 8;

	I32 DO;
	auto ret = APS_read_d_output(cardID, get_group(bit), &DO);
	if (log_error_code("Failed to read DO", ret)) return std::nullopt;

	return (DO >> normalized_bit) & 1;
}

std::optional<std::vector<bool>> Motion_APS::get_motion_io_status(int axis) const
{
	auto status = APS_motion_io_status(axis);

	std::vector<bool> states;
	states.push_back((status >> 0) & 1); //ALM
	states.push_back((status >> 1) & 1); //PEL
	states.push_back((status >> 2) & 1); //MEL
	states.push_back((status >> 3) & 1); //ORG
	states.push_back((status >> 4) & 1); //EMG
	states.push_back((status >> 5) & 1); //EZ
	states.push_back((status >> 6) & 1); //INP
	states.push_back((status >> 7) & 1); //SVON
	states.push_back((status >> 8) & 1); //RDY

	return states;
}

std::optional<std::vector<bool>> Motion_APS::get_all_DI(int cardID) const
{
	std::vector<bool> result(MAX_DI);

	for (int g = 0; g < 4; ++g) {
		I32 DI;
		auto ret = APS_read_d_input(cardID, g, &DI);
		if (log_error_code("Failed to read all DI", ret)) return std::nullopt;

		for (I32 i = 0; i < 8; i++) {
			result[i + (g*8)] = (DI >> i) & 1;
		}
	}

	return result;
}

std::optional<std::vector<bool>> Motion_APS::get_all_DO(int cardID) const
{
	std::vector<bool> result(MAX_DO);

	for (int g = 0; g < 2; ++g) {
		I32 DO;
		auto ret = APS_read_d_output(cardID, g, &DO);
		if (log_error_code("Failed to read all DO", ret)) return std::nullopt;

		for (I32 i = 0; i < 8; i++) {
			result[i + (g*8)] = (DO >> i) & 1;
		}
	}

	return result;
}

bool Motion_APS::set_pulse_per_mm(int axis, double scale)
{
	m_axisInfos[axis].pulse_per_mm = scale;
	return true;
}

bool nvs::motion::Motion_APS::set_positive_limit_mm(int axis, double limit)
{
	m_axisInfos[axis].positive_limit_mm = limit;
	return true;
}

bool nvs::motion::Motion_APS::set_negative_limit_mm(int axis, double limit)
{
	m_axisInfos[axis].negative_limit_mm = limit;
	return true;
}

bool nvs::motion::Motion_APS::set_home_mode(int axis, int mode)
{
	auto ret = APS_set_axis_param(axis, Param::HOME_MODE, mode); //home mode
	return !log_error_code("Failed to set home mode", ret);
}

bool nvs::motion::Motion_APS::set_home_acceleration(int axis, double value)
{
	value = to_pulse(axis, value);
	m_axisInfos[axis].home_speed.accel = value;
	auto ret = APS_set_axis_param(axis, Param::HOME_ACC, value);
	return !log_error_code("Failed to set home acceleration", ret);
}

bool nvs::motion::Motion_APS::set_home_deceleration(int axis, double value)
{
	ct::logger::warn("[Motion_APS] Deceleration not supported");
	return true;
}

bool nvs::motion::Motion_APS::set_home_origin_velocity(int axis, double value)
{
	value = to_pulse(axis, value);
	m_axisInfos[axis].home_speed.origin_velocity = value;
	auto ret = APS_set_axis_param(axis, Param::HOME_VO, value);
	return !log_error_code("Failed to set home origin velocity", ret);
}

bool nvs::motion::Motion_APS::set_home_start_velocity(int axis, double value)
{
	value = to_pulse(axis, value);
	m_axisInfos[axis].home_speed.start_velocity = value;
	auto ret = APS_set_axis_param(axis, Param::HOME_VS, value);
	return !log_error_code("Failed to set home start velocity", ret);
}

bool nvs::motion::Motion_APS::set_home_max_velocity(int axis, double value)
{
	value = to_pulse(axis, value);
	m_axisInfos[axis].home_speed.max_velocity = value;
	auto ret = APS_set_axis_param(axis, Param::HOME_VM, value); //home max velocity
	return !log_error_code("Failed to set home max velocity", ret);
}

bool nvs::motion::Motion_APS::set_move_acceleration(int axis, double value)
{
	value = to_pulse(axis, value);
	m_axisInfos[axis].move_speed.accel = value;
	auto ret = APS_set_axis_param(axis, Param::ACC, value);
	ret &= APS_set_axis_param(axis, Param::JG_ACC, value);
	return !log_error_code("Failed to set move acceleration", ret);
}

bool nvs::motion::Motion_APS::set_move_deceleration(int axis, double value)
{
	value = to_pulse(axis, value);
	m_axisInfos[axis].move_speed.decel = value;
	auto ret = APS_set_axis_param(axis, Param::DEC, value);
	return !log_error_code("Failed to set move deceleration", ret);
}

bool nvs::motion::Motion_APS::set_move_start_velocity(int axis, double value)
{
	value = to_pulse(axis, value);
	m_axisInfos[axis].move_speed.start_velocity = value;
	return true;
}

bool nvs::motion::Motion_APS::set_move_max_velocity(int axis, double value)
{
	value = to_pulse(axis, value);
	m_axisInfos[axis].move_speed.max_velocity = value;
	return true;
}

bool Motion_APS::set_position_mm(int cardID, int axis, double pos_mm)
{
	I32 pos = to_pulse(axis, pos_mm);

	auto ret = APS_set_position(axis, pos);
	if (log_error_code("Failed to set position", ret)) return false;
	return true;
}

bool Motion_APS::set_servo(int cardID, int axis, bool on)
{
	auto ret = APS_set_servo_on(axis, (on) ? 1 : 0);
	if (log_error_code("Failed to set servo", ret)) return false;

	if (axis != 0 || axis != 1) return true; //HARDCODE:
	ret = APS_set_axis_param(axis, Param::SF, 0); //0: T-curve, 10: S-curve
	if (log_error_code("Failed to set S curve", ret)) return false;
	return true;
}

bool Motion_APS::set_DO(int cardID, int bit, bool state)
{
	if (bit < 0 || bit >= MAX_DO) {
		ct::logger::error("[Motion_APS] Failed to set DO: Index out of range");
		return false;
	}

	const int group = bit / 8;
	const int bit_in_group = bit % 8;

	auto opt_DOs = get_all_DO(cardID);
	if (!opt_DOs)
		return false;

	auto DOs = opt_DOs.value();
	DOs[bit] = state;

	// build 8-bit mask for THIS group only
	I32 mask = 0;
	const int base = group * 8;

	for (int i = 0; i < 8; ++i) {
		if (DOs[base + i])
			mask |= (1 << i);
	}

	auto ret = APS_write_d_output(cardID, group, mask);
	if (log_error_code("Failed to write DO", ret))
		return false;

	return true;
}

bool Motion_APS::set_all_DO(int cardID, const std::vector<bool>& states)
{
	if (states.size() > MAX_DO) {
		ct::logger::error("[Motion_APS] Failed to write all DO: States received more than max capacity of %d", MAX_DO);
		return false;
	}

	const int CHANNELS_PER_GROUP = 8;
	const int totalGroups = (states.size() + CHANNELS_PER_GROUP - 1) / CHANNELS_PER_GROUP;

	for (int group = 0; group < totalGroups; ++group)
	{
		I32 mask = 0;

		// Compute mask for this group
		for (int bit = 0; bit < CHANNELS_PER_GROUP; ++bit)
		{
			int index = group * CHANNELS_PER_GROUP + bit;
			if (index >= static_cast<int>(states.size()))
				break;

			if (states[index])
				mask |= (1 << bit);
		}

		// Write to this DO group
		I32 ret = APS_write_d_output(cardID, group, mask);
		if (log_error_code("Failed to write DO group", ret)) return false;
	}

	return true;
}

bool Motion_APS::home(int axis)
{
	//DIR: 0 - positive, 1 - negative 
	auto ret = APS_home_move(axis);
	if (log_error_code("Failed to home", ret)) return false;
	return true;
}

bool Motion_APS::move_done(int cardID, int axis)
{
	auto status = APS_motion_status(axis);
	bool mdn = (status >> 5) & 1;   // MDN
	//bool emgs = (status >> 18) & 1;   // EMG stopped
	//bool alms = (status >> 19) & 1;   // Alarm stop
	//bool pels = (status >> 21) & 1;   // PEL stopped
	//bool mels = (status >> 22) & 1;   // MEL stopped
	//bool spels = (status >> 24) & 1;  // SPEL stopped
	//bool smels = (status >> 25) & 1;  // SMEL stopped
	//bool ezs = (status >> 28) & 1;    // EZ stopped
	//bool hmes = (status >> 29) & 1;   // Home error stopped
	//bool orgs = (status >> 30) & 1;   // ORG stopped

	auto io_status = APS_motion_io_status(axis);

	//ct::logger::info("Motion %d: %d", axis, mdn);

	bool inp = (io_status >> 6) & 1;
	if (axis == 0 || axis == 1) return inp && mdn; //HARDCODE:

	return mdn;
}

bool Motion_APS::continuous_move(int axis, bool positive_direction)
{
	if (invalid_axis("Failed to move", axis)) return false;

	auto speed = m_axisInfos[axis].move_speed.max_velocity;
	if (!positive_direction) speed = -speed;

	auto ret = APS_velocity_move(axis, speed);
	if (log_error_code("Failed to move", ret)) return false;
	return true;
}

bool Motion_APS::absolute_move(int axis, double position_mm)
{
	if (invalid_axis("Failed to move", axis)) return false;

	if (!is_safe(axis, position_mm)) { //BAD HARDCODE: Desperate times calls for desperate measures
		if (axis == 0) {
			auto positive_limit = m_axisInfos[axis].positive_limit_mm;
			auto negative_limit = m_axisInfos[axis].negative_limit_mm;

			bool safe = true;

			if (positive_limit < position_mm) position_mm = positive_limit;
			if (negative_limit > position_mm) position_mm = negative_limit;
		}
		else {
			return false;
		}
	}

	//HARDCODE:
	if (axis == 1) position_mm = -position_mm;

	const auto pulse = to_pulse(axis, position_mm);
	const auto maxVelocity = m_axisInfos[axis].move_speed.max_velocity;
	ct::logger::info("[Motion_APS] absolute_move: axis=%d, position=%.4fmm -> %d pulse, max_velocity=%d (pulse/mm=%f)",
		axis, position_mm, (int)pulse, (int)maxVelocity, m_axisInfos[axis].pulse_per_mm);

	auto ret = APS_absolute_move(axis, pulse, maxVelocity);
	if (log_error_code("Failed to move", ret)) return false;
	return true;
}

bool nvs::motion::Motion_APS::absolute_multi_move(const std::vector<MoveParam>& moveParams)
{
	I32 max_speed = 0;
	I32 dimension = moveParams.size();
	I32* axes = new I32[dimension];
	I32* positions = new I32[dimension];

	int index = 0;

	for (const auto& p : moveParams) {
		if (invalid_axis("Failed to move", p.axis)) return false;
		if (!is_safe(p.axis, p.position_mm)) { //BAD HARDCODE: Desperate times calls for desperate measures
			if (p.axis == 0) {
				auto positive_limit = m_axisInfos[p.axis].positive_limit_mm;
				auto negative_limit = m_axisInfos[p.axis].negative_limit_mm;

				bool safe = true;

				if (positive_limit < p.position_mm) positions[index] = positive_limit;
				if (negative_limit > p.position_mm) positions[index] = negative_limit;
			}
			else {
				return false;
			}
		}
		
		axes[index] = p.axis;
		positions[index] = to_pulse(p.axis, p.position_mm);

		if (p.axis == 1) positions[index] = -positions[index];

		if (m_axisInfos[p.axis].move_speed.max_velocity > max_speed) {
			max_speed = m_axisInfos[p.axis].move_speed.max_velocity;
		}

		index++;
	}

	
	auto ret = APS_absolute_linear_move(dimension, axes, positions, max_speed);
	if (log_error_code("Failed to move", ret)) return false;
	return true;
}

bool Motion_APS::relative_move(int axis, double distance)
{
	if (invalid_axis("Failed to move", axis)) return false;

	int cardID = 0;
	if (axis > 4) cardID = 1;
	auto opt_position_mm = get_position_mm(cardID, axis);

	if (!opt_position_mm.has_value()) return false;

	ct::logger::info(QStringLiteral("\n[Relative move] opt_position_mm: %1, distance: %2\n").arg(opt_position_mm.value()).arg(distance).toStdString().c_str());

	//HARDCODE FOR Y-AXIS:
	auto safety_distance = distance;
	if (axis == 1) safety_distance = -safety_distance;
	if (!is_safe(axis, opt_position_mm.value() + safety_distance)) return false;

	ct::logger::info("Relative mvoe: %.2f, %.2f", m_axisInfos[axis].move_speed.max_velocity, to_pulse(axis, distance));
	auto ret = APS_relative_move(axis, to_pulse(axis, distance), m_axisInfos[axis].move_speed.max_velocity);
	if (log_error_code("Failed to move", ret)) return false;
	return true;
}

bool Motion_APS::stop_move(int axis)
{
	if (invalid_axis("Failed to stop", axis)) return false;
	auto ret = APS_stop_move(axis);
	//auto ret = APS_emg_stop(axis);
	if (log_error_code("Failed to stop", ret)) return false;
	return true;
}

bool nvs::motion::Motion_APS::is_safe(int axis, double position_mm)
{
	ct::logger::info(QStringLiteral("\n[Relative move] axis: %1, position_mm: %2\n").arg(axis).arg(position_mm).toStdString().c_str());

	
	auto positive_limit = m_axisInfos[axis].positive_limit_mm;
	auto negative_limit = m_axisInfos[axis].negative_limit_mm;

	ct::logger::info(QStringLiteral("\n[Relative move] positive_limit: %1, negative_limit: %2\n").arg(positive_limit).arg(negative_limit).toStdString().c_str());

	bool safe = true;

	if (positive_limit < position_mm) safe = false;
	if (negative_limit > position_mm) safe = false;

	if (!safe) {
		if (axis == (int)Axis::X) MachineController::instance().notifyWarning(MachineWarning::X_SOFT_LIMIT_HIT);
		else if (axis == (int)Axis::Y) MachineController::instance().notifyWarning(MachineWarning::Y_SOFT_LIMIT_HIT);
		else if (axis == (int)Axis::Z) MachineController::instance().notifyWarning(MachineWarning::Z_SOFT_LIMIT_HIT);
		ct::logger::error("[Motion_APS] Axis %d out of bound: %.2f", axis, position_mm);
	}

	return safe;
}

bool nvs::motion::Motion_APS::get_soft_limit(int axis, double position_mm, double& softLimit_mm)
{
	auto positive_limit = m_axisInfos[axis].positive_limit_mm;
	auto negative_limit = m_axisInfos[axis].negative_limit_mm;

	bool safe = true;
	if (positive_limit < position_mm) safe = false;
	if (negative_limit > position_mm) safe = false;

	softLimit_mm = 0;
	if (!safe) {
		if (position_mm < negative_limit) softLimit_mm = negative_limit;
		else softLimit_mm = positive_limit;
	}

	QString error = QStringLiteral("[Motion_APS] Axis %1 out of bound: %2, soft limit: %3").arg(axis).arg(position_mm).arg(softLimit_mm);
	ct::logger::warn(error.toStdString().c_str());

	return safe;
}

bool Motion_APS::load_config(const std::string& path)
{
	auto ret = APS_load_param_from_file(path.c_str());
	if (log_error_code("Failed to load config file", ret)) return false;

	for (I32 i = 0; i < m_totalAxis; i++) {

		int cardID = 0;
		if (i >= 4) cardID = 1;

		ret = APS_reset_emx_alarm(i);
		log_error_code("Failed to reset alarm", ret);

		set_servo(cardID, i, true);

		//ret = APS_set_axis_param(i, Param::HOME_MODE, 2); //home mode
		//log_error_code("Failed to set home mode", ret);

		//SpeedInfo homeSpeed;
		//homeSpeed.accel = 50000;
		//homeSpeed.startVel = 200;
		//homeSpeed.maxVel = 15000;
		//set_home_speed(i, homeSpeed);

		//SpeedInfo mSpeed;
		//mSpeed.accel = 100000;
		//mSpeed.startVel = 10000;
		//mSpeed.startVel = 100000;
		//set_move_speed(i, mSpeed);
	}

	return true;
}

std::string Motion_APS::error_msg()
{
	return m_errorMsg;
}
