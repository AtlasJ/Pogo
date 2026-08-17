//#include "Motion_8134A.h"
//#include "PCI8134a.H"
//
//bool Motion_8134A::is_success(short ret)
//{
//	return (ret) ? false : true;
//}
//
//Motion_8134A::Motion_8134A()
//{
//}
//
//Motion_8134A::~Motion_8134A()
//{
//}
//
//bool Motion_8134A::init()
//{
//	auto ret = true; //_8134_initial(&totalCards);
//	return is_success(ret);
//}
//
//bool Motion_8134A::release()
//{
//	for (int i = 0; i < totalAxis; i++) {
//		//_8134_set_servo(i, 1);
//	}
//
//	auto ret = true; //_8134_int_disable(0);
//	ret += true; //_8134_close();
//	return is_success(ret);
//}
//
//std::string Motion_8134A::version()
//{
//	//U16 HardwareInfo = 0;
//	//I32 SoftwareInfo = 0, DriverInfo = 0;
//	//_8134_version_info(0, &HardwareInfo, &SoftwareInfo, &DriverInfo);
//	//return QString("%1.%2.%3").arg(HardwareInfo).arg(SoftwareInfo).arg(DriverInfo);
//	return "";
//}
//
//double Motion_8134A::get_position(Axis axis)
//{
//	double pos = 0.0;
//	//_8134_get_position((I16)axis, &pos);
//	return pos;
//}
//
//double Motion_8134A::get_current_speed(Axis axis)
//{
//	double speed = 0.0;
//	//_8134_get_current_speed(axis, &speed);
//	return speed;
//}
//
//bool Motion_8134A::set_position(Axis axis, double pos)
//{
//	auto ret = true; //_8134_set_position(1, 200000);
//	return is_success(ret);
//}
//
//bool Motion_8134A::set_servo(Axis axis, bool on)
//{
//	auto ret = true; //_8134_set_servo((I16)axis, (on) ? 0 : 1);
//	return is_success(ret);
//}
//
//bool Motion_8134A::home(Axis axis, double start_velocity, double max_velocity, double accel, double decel)
//{
//	auto ret = true; //_8134_home_move((I16)axis, start_velocity, max_velocity, accel);
//	return is_success(ret);
//}
//
//bool Motion_8134A::move(Axis axis, double distance, double start_velocity, double max_velocity, double accel, double decel)
//{
//	auto ret = true; //_8134_start_tr_move((I16)axis, distance, start_velocity, max_velocity, accel, decel);
//	return is_success(ret);
//}
//
//bool Motion_8134A::move_done(Axis axis)
//{
//	auto ret = true; //_8134_motion_done((I16)axis);
//	return is_success(ret);
//}
//
//bool Motion_8134A::load_config(std::string path)
//{
//	//char SysDir[MAX_PATH];
//	//char FStr[MAX_PATH];
//	//GetSystemDirectoryA(SysDir, MAX_PATH);
//	//sprintf(FStr, "%s\\8134.cfg", SysDir);
//	//I16 Err = true; //_8134_config_from_file((U8*)FStr);
//	//if (Err) printf("Can't File in %s\n", FStr);
//	return true;
//}
//
//std::string Motion_8134A::error_msg()
//{
//	return std::string();
//}
