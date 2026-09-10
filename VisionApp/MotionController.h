#pragma once
#include "IMotion.h"
#include <QHash>
#include <optional>

class MotionController {

public:
	static MotionController& instance();

	void load_config(QString path);

    // lifecycle
    bool init(QString id);
    bool release(QString id);
    bool reconnect(QString id, bool releaseFirst = true); //release + full startup flow (init, configs)

    void enable(bool enable);
    void enable_motion(bool enable);

    //Whether commanded moves are currently allowed through. Only the move functions consult
    //this - home(), set_servo() and the IO calls deliberately ignore it, or you could never
    //home your way out of the very state that disables motion.
    bool motionEnabled() const { return m_enableMotion; }

    //quiet validity check: same answer as valid() but never logs - for pollers
    //that need to know whether the controller is worth talking to at all
    bool available(QString id) const;

    // Query
    std::optional<std::string> version(QString id, int cardID) const;
    std::optional<double> get_position_mm(QString id, int cardID, int axis) const;
    std::optional<double> get_current_speed(QString id, int cardID, int axis) const;

    std::optional<bool> get_DI(QString id, int cardID, int bit) const;
    std::optional<bool> get_DO(QString id, int cardID, int bit) const;
    std::optional<std::vector<bool>> get_motion_io_status(QString id, int axis) const;
    std::optional<std::vector<bool>> get_all_DI(QString id, int cardID) const;
    std::optional<std::vector<bool>> get_all_DO(QString id, int cardID) const;

    // Config
    bool set_pulse_per_mm(QString id, int axis, double scale);

    //Signed - negative means a positive mm command travels towards the NEGATIVE end limit.
    //Quiet (uses available(), not valid()) because a UI poll asks it several times a second.
    double pulse_per_mm(QString id, int axis) const;
    bool set_positive_limit_mm(QString id, int axis, double limit);
    bool set_negative_limit_mm(QString id, int axis, double limit);

    bool set_home_mode(QString id, int axis, int mode);
    bool set_home_acceleration(QString id, int axis, double value);
    bool set_home_deceleration(QString id, int axis, double value);
    bool set_home_origin_velocity(QString id, int axis, double value);
    bool set_home_start_velocity(QString id, int axis, double value);
    bool set_home_max_velocity(QString id, int axis, double value);

    bool set_move_acceleration(QString id, int axis, double value);
    bool set_move_deceleration(QString id, int axis, double value);
    bool set_move_start_velocity(QString id, int axis, double value);
    bool set_move_max_velocity(QString id, int axis, double value);

    bool set_position_mm(QString id, int cardID, int axis, double pos_mm);
    bool set_servo(QString id, int cardID, int axis, bool on);

    bool set_DO(QString id, int cardID, int bit, bool state);
    bool set_all_DO(QString id, int cardID, const std::vector<bool>& states);

    // Control
    bool home(QString id, int axis);
    bool continuous_move(QString id, int axis, bool positive_direction);
    bool absolute_move(QString id, int axis, double position_mm);
    bool absolute_multi_move(QString id, const std::vector<nvs::motion::MoveParam>& moveParams);
    bool relative_move(QString id, int axis, double distance_mm);
    bool stop_move(QString id, int axis);
    bool move_done(QString id, int cardID, int axis);

    // Misc
    bool reset_alarm(QString id, int axis);
    bool is_init(QString id);
    bool is_safe(QString id, int axis, double position_mm);
    bool get_soft_limit(QString id, int axis, double position_mm, double& softLimit_mm);
    bool load_config(QString id, QString path);
    QString error_msg(QString id);

private:
	MotionController();
	~MotionController();
	MotionController(const MotionController&) = delete;
	MotionController& operator=(const MotionController&) = delete;

	static MotionController m_instance;

	bool m_enable = true;
    bool m_enableMotion = true;
    mutable qint64 m_lastInvalidWarnMs = 0; //throttles the invalid-controller warning
    QString m_configPath; //motion.json path, kept for reconnect

    QHash<QString, bool> m_initStatus;
	QHash<QString, nvs::motion::IMotion*> m_motion;

	bool create(QString id, QString api);
	bool valid(QString id) const;
};