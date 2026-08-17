#pragma once
#include "IMotion.h"
#include "APS Library/Include/type_def.h"
#include <unordered_map>

namespace nvs {
	namespace motion {
		class Motion_APS : public IMotion {
		private:
			bool m_is_init = false;

			const int MAX_DI = 32;
			const int MAX_DO = 14;

			int get_group(int bit) const;
			const char* get_code(int ret) const;
			bool log_error_code(const char* msg, int ret) const;
			bool invalid_axis(const char* msg, int axis) const;

			double to_mm(int axis, double pulse) const;
			double to_pulse(int axis, double mm) const;

			mutable std::string m_errorMsg = "No error";

			int m_totalAxis = 0;

			std::vector<int> m_cardIDs;
			std::unordered_map<int, AxisInfo> m_axisInfos;

			enum Param {
				EL_LOGIC, ORG_LOGIC, EL_MODE = 2, ALM_LOGIC = 4, EZ_LOGIC = 6, SPEL_POS = 10, SMEL_POS = 11,
				HOME_MODE = 16, HOME_DIR = 17, HOME_ACC = 19, HOME_VS = 20, HOME_VM = 21, HOME_EZA = 24,
				HOME_VO = 25, HOME_EZ_DIR = 29, HOME_SEARCH_TARGET = 30, SF = 32, ACC = 33, DEC = 34, VS = 35,
				JG_DIR = 65, JG_ACC = 67, JG_VM = 69, JG_STOP = 76, SERVO_LOGIC = 83, PLS_IPT_MODE = 128,
				PLS_OPT_MODE = 129, PLS_IPT_PIN_DIR = 135, PLS_OPT_PIN_DIR = 136, RST_OPT_CHG_DI = 137,
				SOFT_EL_EN = 176, SOFT_EL_SRC = 177, PLS_IPT_NEG_DRIVE = 178, PLS_OPT_NEG_DRIVE = 179,
				PLS_OPT_DIR = 180, TRIG_VEL_PREVENTION_EN = 181, FEEDBACK_SRC = 513, INP_LOGIC = 529, RDY_LOGIC = 563
			};

		public:
			Motion_APS();
			~Motion_APS();

			enum IO { ALM, PEL, NEL, ORG, EMG, EZ, INP, SVON, RDY };

			bool init();
			bool release();

			//Query
			std::optional<std::string> version(int cardID) const;
			std::optional<double> get_position_mm(int cardID, int axis) const;
			std::optional<double> get_current_speed(int cardID, int axis) const;
			std::optional<bool> get_DI(int cardID, int bit) const;
			std::optional<bool> get_DO(int cardID, int bit) const;
			std::optional<std::vector<bool>> get_motion_io_status(int axis) const;
			std::optional<std::vector<bool>> get_all_DI(int cardID) const;
			std::optional<std::vector<bool>> get_all_DO(int cardID) const;

			//Config
			bool set_pulse_per_mm(int axis, double scale);
			bool set_positive_limit_mm(int axis, double limit);
			bool set_negative_limit_mm(int axis, double limit);
			bool set_home_mode(int axis, int mode);
			bool set_home_acceleration(int axis, double value);
			bool set_home_deceleration(int axis, double value);
			bool set_home_origin_velocity(int axis, double value);
			bool set_home_start_velocity(int axis, double value);
			bool set_home_max_velocity(int axis, double value);
			bool set_move_acceleration(int axis, double value);
			bool set_move_deceleration(int axis, double value);
			bool set_move_start_velocity(int axis, double value);
			bool set_move_max_velocity(int axis, double value);
			bool set_position_mm(int cardID, int axis, double pos_mm);
			bool set_servo(int cardID, int axis, bool on);
			bool set_DO(int cardID, int bit, bool state);
			bool set_all_DO(int cardID, const std::vector<bool>& states);

			//Control
			bool home(int axis);
			bool move_done(int cardID, int axis);
			bool continuous_move(int axis, bool positive_direction);
			bool absolute_move(int axis, double position_mm);
			bool absolute_multi_move(const std::vector<MoveParam>& moveParams);
			bool relative_move(int axis, double distance);
			bool stop_move(int axis);

			//Misc
			bool is_safe(int axis, double position_mm);
			bool get_soft_limit(int axis, double position_mm, double& softLimit_mm);
			bool load_config(const std::string& path);
			std::string error_msg();
		};
	}
}