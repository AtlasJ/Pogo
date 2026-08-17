#pragma once

#include "IMotion.h"
#include <string>

using namespace nvs::motion;

//class Motion_8134A : public IMotion {
//private:
//	short totalCards = 0;
//	short totalAxis = 0;
//
//	bool is_success(short ret);
//public:
//	Motion_8134A();
//	~Motion_8134A();
//
//	bool init();
//	bool release();
//
//	//Query
//	std::string version();
//	double get_position(Axis axis);
//	double get_current_speed(Axis axis);
//
//	//Config
//	bool set_position(Axis axis, double pos);
//	bool set_servo(Axis axis, bool on);
//
//	//Control
//	bool home(Axis axis, double start_velocity, double max_velocity, double accel, double decel);
//	bool move(Axis axis, double distance, double start_velocity, double max_velocity, double accel, double decel);
//	bool move_done(Axis axis);
//
//	//Misc
//	bool load_config(std::string path);
//	std::string error_msg();
//};