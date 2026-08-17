#pragma once
#include <string>
#include <vector>
#include <optional>

namespace nvs {
    namespace motion {
        enum class MotionStatus {
            ALARM, POSITIVE_LIMIT, NEGATIVE_LIMIT,
            ORG, EMG, EZ, IN_POSITION,
            SERVO_ON, READY
        };

        struct SpeedInfo {
            double origin_velocity = 0.0; //only homing use
            double start_velocity = 0.0;
            double max_velocity = 0.0;
            double accel = 0.0;
            double decel = 0.0;
        };

        struct AxisInfo {
            double pulse_per_mm = 1000.0;
            double positive_limit_mm = 500.0;
            double negative_limit_mm = -1.0;
            SpeedInfo home_speed;
            SpeedInfo move_speed;
        };

        struct MoveParam {
            int axis = -1;
            double position_mm = 0.0;
        };

        struct AxisConfig
        {
            int id = -1;
            std::string name;

            double pulse_per_mm = 0;

            double move_acceleration = 0;
            double move_deceleration = 0;
            double move_start_velocity = 0;
            double move_max_velocity = 0;

            double home_acceleration = 0;
            double home_deceleration = 0;
            double home_to_origin_velocity = 0;
            double home_start_velocity = 0;
            double home_max_velocity = 0;

            int home_mode = 0;

            double positive_limit_mm = 0.0;
            double negative_limit_mm = 0.0;
            double max_allowable_velocity = 0.0;
        };

        struct MotionConfig
        {
            std::string id;
            std::string api;
            bool enable = false;
            std::string  config_file;

            std::vector<AxisConfig> axes;
        };
    }
}

namespace nvs {
	namespace motion {
        class IMotion {
        public:
            virtual ~IMotion() = default;

            // lifecycle
            virtual bool init() = 0;
            virtual bool release() = 0;

            // Query
            virtual std::optional<std::string> version(int cardID) const = 0;
            virtual std::optional<double> get_position_mm(int cardID, int axis) const = 0;
            virtual std::optional<double> get_current_speed(int cardID, int axis) const = 0;

            virtual std::optional<bool> get_DI(int cardID, int bit) const = 0;
            virtual std::optional<bool> get_DO(int cardID, int bit) const = 0;
            virtual std::optional<std::vector<bool>> get_all_DI(int cardID) const = 0;
            virtual std::optional<std::vector<bool>> get_all_DO(int cardID) const = 0;
            virtual std::optional<std::vector<bool>> get_motion_io_status(int axis) const = 0;

            // Config
            virtual bool set_pulse_per_mm(int axis, double scale) = 0;
            virtual bool set_positive_limit_mm(int axis, double limit) = 0;
            virtual bool set_negative_limit_mm(int axis, double limit) = 0;

            virtual bool set_home_mode(int axis, int mode) = 0;
            virtual bool set_home_acceleration(int axis, double value) = 0;
            virtual bool set_home_deceleration(int axis, double value) = 0;
            virtual bool set_home_origin_velocity(int axis, double value) = 0;
            virtual bool set_home_start_velocity(int axis, double value) = 0;
            virtual bool set_home_max_velocity(int axis, double value) = 0;

            virtual bool set_move_acceleration(int axis, double value) = 0;
            virtual bool set_move_deceleration(int axis, double value) = 0;
            virtual bool set_move_start_velocity(int axis, double value) = 0;
            virtual bool set_move_max_velocity(int axis, double value) = 0;

            virtual bool set_position_mm(int cardID, int axis, double pos_mm) = 0;
            virtual bool set_servo(int cardID, int axis, bool on) = 0;

            virtual bool set_DO(int cardID, int bit, bool state) = 0;
            virtual bool set_all_DO(int cardID, const std::vector<bool>& states) = 0;

            // Control
            virtual bool home(int axis) = 0;
            virtual bool continuous_move(int axis, bool positive_direction) = 0;
            virtual bool absolute_move(int axis, double position_mm) = 0;
            virtual bool absolute_multi_move(const std::vector<MoveParam>& moveParams) = 0;
            virtual bool relative_move(int axis, double distance_mm) = 0;
            virtual bool stop_move(int axis) = 0;
            virtual bool move_done(int cardID, int axis) = 0;

            // Misc
            virtual bool is_safe(int axis, double position_mm) = 0;
            virtual bool get_soft_limit(int axis, double position_mm, double& softLimit_mm) = 0;
            virtual bool load_config(const std::string& path) = 0;
            virtual std::string error_msg() = 0;
        };
	}
}