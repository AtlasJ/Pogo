#include "MotionController.h"
#include <QDateTime>
#include "Motion_8134A.h"
#include "Motion_APS.h"
#include "GantryLog.h"
#include "Logger.h"
#include "QJsonHelper.h"
#include "SystemData.h"

#include <QJsonObject>
#include <cmath>

MotionController MotionController::m_instance;

MotionController& MotionController::instance()
{
    return m_instance;
}

MotionController::MotionController()
{
}

MotionController::~MotionController()
{
}

bool MotionController::create(QString id, QString api)
{
	//Reuse the existing instance when reconnecting
	if (m_motion.contains(id)) return true;

	if (api == "8134A") {
		//auto* p = new Motion_8134A();
		//m_motion.insert(id, p);
	}
	else if (api == "APS") {
		auto* p = new Motion_APS();
		m_motion.insert(id, p);
	}
	else {
		ct::logger::error("[Motion] Failed to create motion controller: %s", api.toStdString().c_str());
		return false;
	}

	ct::logger::info("[Motion] Created motion controller: %s", api.toStdString().c_str());
	return true;
}

bool MotionController::available(QString id) const
{
    return m_enable && m_motion.contains(id) && m_initStatus.value(id, false);
}

bool MotionController::valid(QString id) const
{
	if (!m_enable) return false;
    if (!m_motion.contains(id)) {
        //offline machine: every poller lands here - warn once every 5 s, not per call
        const qint64 now = QDateTime::currentMSecsSinceEpoch();
        if (now - m_lastInvalidWarnMs >= 5000) {
            m_lastInvalidWarnMs = now;
            ct::logger::warn("[Motion] Trying to access invalid motion controller: %s", id.toStdString().c_str());
        }
        return false;
    }
    //Block APS access while the card is released or re-initializing (reconnect):
    //calls into the APS DLL during APS_close/APS_initial can crash the process
    //(e.g. the tower light timer firing set_DO mid-reconnect).
    if (!m_initStatus.value(id, false)) return false;
    return true;
}

void MotionController::load_config(QString path)
{
	QJsonObject obj;

	m_configPath = path;

	if (!jsonHelper::loadJson(path, obj)) {
		ct::logger::error("[Motion] Failed to load motion.json");
		return;
	}

	if (obj.contains("Motion")) {
		auto motions = obj["Motion"].toArray();

		for (auto doc : motions) {
			auto obj = doc.toObject();

			auto motionID = jsonHelper::getString(obj, "id");
			auto api = jsonHelper::getString(obj, "api");
			auto configPath = jsonHelper::getString(obj, "config_file");
			auto axes = jsonHelper::getArray(obj, "axes");
			auto enable = jsonHelper::getBool(obj, "enable");

			m_enable = enable;

			if (!create(motionID, api)) continue;

			if (!init(motionID)) continue;

			if (!load_config(motionID, configPath)) continue;

			//axes
			for (auto axis : axes) {
				auto axisObj = axis.toObject();

				int id = jsonHelper::getInteger(axisObj, "id");
				QString name = jsonHelper::getString(axisObj, "name");
				double pulse_per_mm = jsonHelper::getDouble(axisObj, "pulse_per(mm)");
				double move_accel = jsonHelper::getDouble(axisObj, "move_acceleration");
				double move_decel = jsonHelper::getDouble(axisObj, "move_deceleration");
				double move_svel = jsonHelper::getDouble(axisObj, "move_start_velocity");
				double move_mvel = jsonHelper::getDouble(axisObj, "move_max_velocity");
				double home_accel = jsonHelper::getDouble(axisObj, "home_acceleration");
				double home_decel = jsonHelper::getDouble(axisObj, "home_deceleration");
				double home_ovel = jsonHelper::getDouble(axisObj, "home_to_origin_velocity");
				double home_svel = jsonHelper::getDouble(axisObj, "home_start_velocity");
				double home_mvel = jsonHelper::getDouble(axisObj, "home_max_velocity");
				int home_mode = jsonHelper::getInteger(axisObj, "home_mode");
				double positive_limit = jsonHelper::getDouble(axisObj, "positive_limit(mm)");
				double negative_limit = jsonHelper::getDouble(axisObj, "negative_limit(mm)");
				double max_allow_vel = jsonHelper::getDouble(axisObj, "max_allowable_velocity");

				set_pulse_per_mm(motionID, id, pulse_per_mm);
				set_home_acceleration(motionID, id, home_accel);
				set_home_deceleration(motionID, id, home_decel);
				set_home_origin_velocity(motionID, id, home_ovel);
				set_home_start_velocity(motionID, id, home_svel);
				set_home_max_velocity(motionID, id, home_mvel);
				set_move_acceleration(motionID, id, move_accel);
				set_move_deceleration(motionID, id, move_decel);
				set_move_start_velocity(motionID, id, move_svel);
				set_move_max_velocity(motionID, id, move_mvel);
				set_home_mode(motionID, id, home_mode);
				set_positive_limit_mm(motionID, id, positive_limit);
				set_negative_limit_mm(motionID, id, negative_limit);
			}
		}
	}
}

bool MotionController::init(QString id)
{
	//direct check: valid() requires init status, which is not set yet here
	if (!m_enable || !m_motion.contains(id)) return false;
	auto ret = m_motion[id]->init();
	m_initStatus.insert(id, ret);

	if (ret) {
		ct::logger::info("[Motion] Successfully initialize motion card: %s", id.toStdString().c_str());
	}
	else {
		ct::logger::error("[Motion] Failed to initialize motion card: %s", id.toStdString().c_str());
	}

	return ret;
}

bool MotionController::release(QString id)
{
	if (!m_enable || !m_motion.contains(id)) return false;

	//block all other APS access from here on (see valid())
	m_initStatus.insert(id, false);
	return m_motion[id]->release();
}

bool MotionController::reconnect(QString id, bool releaseFirst)
{
	if (!m_enable || !m_motion.contains(id)) return false;

	m_initStatus.insert(id, false);

	if (releaseFirst) {
		ct::logger::info("[Reconnect] Releasing motion card: %s", id.toStdString().c_str());
		m_motion[id]->release(); //ignore result - the link may already be dead
		ct::logger::info("[Reconnect] Motion card released");
	}

	//Re-run the full startup flow: init, controller param file and axis configs
	ct::logger::info("[Reconnect] Running startup flow (init + configs)...");
	load_config(m_configPath);
	ct::logger::info("[Reconnect] Startup flow finished");

	if (!is_init(id)) {
		ct::logger::error("[Reconnect] Failed to reconnect motion card: %s", id.toStdString().c_str());
		return false;
	}

	ct::logger::info("[Reconnect] Motion card reconnected: %s", id.toStdString().c_str());
	return true;
}

bool MotionController::reset_alarm(QString id, int axis)
{
	if (!valid(id)) return false;
	return m_motion[id]->reset_alarm(axis);
}

void MotionController::enable(bool enable)
{
	m_enable = enable;
}

void MotionController::enable_motion(bool enable)
{
	m_enableMotion = enable;
}

std::optional<std::string> MotionController::version(QString id, int cardID) const
{
	if (!valid(id)) return std::nullopt;
	return m_motion[id]->version(cardID);
}

std::optional<double> MotionController::get_position_mm(QString id, int cardID, int axis) const
{
	if (!valid(id)) return std::nullopt;
	return m_motion[id]->get_position_mm(cardID, axis);
}

std::optional<double> MotionController::get_current_speed(QString id, int cardID, int axis) const
{
	if (!valid(id)) return std::nullopt;
	return m_motion[id]->get_current_speed(cardID, axis);
}

std::optional<bool> MotionController::get_DI(QString id, int cardID, int no) const
{
	if (!valid(id)) return std::nullopt;
	return m_motion[id]->get_DI(cardID, no);
}

std::optional<bool> MotionController::get_DO(QString id, int cardID, int no) const
{
	if (!valid(id)) return std::nullopt;
	return m_motion[id]->get_DO(cardID, no);
}

std::optional<std::vector<bool>> MotionController::get_all_DI(QString id, int cardID) const
{
	if (!valid(id)) return std::nullopt;
	return m_motion[id]->get_all_DI(cardID);
}

std::optional<std::vector<bool>> MotionController::get_all_DO(QString id, int cardID) const
{
	if (!valid(id)) return std::nullopt;
	return m_motion[id]->get_all_DO(cardID);
}

bool MotionController::set_pulse_per_mm(QString id, int axis, double scale)
{
	if (!valid(id)) return false;
	return m_motion[id]->set_pulse_per_mm(axis, scale);
}

double MotionController::pulse_per_mm(QString id, int axis) const
{
	if (!available(id)) return 1.0;
	return m_motion[id]->get_pulse_per_mm(axis);
}

std::optional<double> MotionController::get_command_mm(QString id, int cardID, int axis) const
{
	if (!available(id)) return std::nullopt;
	return m_motion[id]->get_command_mm(cardID, axis);
}

std::optional<double> MotionController::get_move_max_velocity_mm(QString id, int axis) const
{
	if (!available(id)) return std::nullopt;
	return m_motion[id]->get_move_max_velocity_mm(axis);
}

bool MotionController::limits_mm(QString id, int axis, double& negative_mm, double& positive_mm) const
{
	if (!available(id)) return false;
	return m_motion[id]->get_limits_mm(axis, negative_mm, positive_mm);
}

bool MotionController::set_positive_limit_mm(QString id, int axis, double limit)
{
	if (!valid(id)) return false;
	return m_motion[id]->set_positive_limit_mm(axis, limit);
}

bool MotionController::set_negative_limit_mm(QString id, int axis, double limit)
{
	if (!valid(id)) return false;
	return m_motion[id]->set_negative_limit_mm(axis, limit);
}

bool MotionController::set_home_mode(QString id, int axis, int mode)
{
	if (!valid(id)) return false;
	return m_motion[id]->set_home_mode(axis, mode);
}

bool MotionController::set_home_acceleration(QString id, int axis, double value)
{
	if (!valid(id)) return false;
	if (!is_init(id)) return false;
	return m_motion[id]->set_home_acceleration(axis, value);
}

bool MotionController::set_home_deceleration(QString id, int axis, double value)
{
	if (!valid(id)) return false;
	if (!is_init(id)) return false;
	return m_motion[id]->set_home_deceleration(axis, value);
}

bool MotionController::set_home_origin_velocity(QString id, int axis, double value)
{
	if (!valid(id)) return false;
	if (!is_init(id)) return false;
	return m_motion[id]->set_home_origin_velocity(axis, value);
}

bool MotionController::set_home_start_velocity(QString id, int axis, double value)
{
	if (!valid(id)) return false;
	if (!is_init(id)) return false;
	return m_motion[id]->set_home_start_velocity(axis, value);
}

bool MotionController::set_home_max_velocity(QString id, int axis, double value)
{
	if (!valid(id)) return false;
	if (!is_init(id)) return false;
	return m_motion[id]->set_home_max_velocity(axis, value);
}

bool MotionController::set_move_acceleration(QString id, int axis, double value)
{
	if (!valid(id)) return false;
	if (!is_init(id)) return false;
	return m_motion[id]->set_move_acceleration(axis, value);
}

bool MotionController::set_move_deceleration(QString id, int axis, double value)
{
	if (!valid(id)) return false;
	if (!is_init(id)) return false;
	return m_motion[id]->set_move_deceleration(axis, value);
}

bool MotionController::set_move_start_velocity(QString id, int axis, double value)
{
	if (!valid(id)) return false;
	if (!is_init(id)) return false;
	return m_motion[id]->set_move_start_velocity(axis, value);
}

bool MotionController::set_move_max_velocity(QString id, int axis, double value)
{
	if (!valid(id)) return false;
	if (!is_init(id)) return false;
	return m_motion[id]->set_move_max_velocity(axis, value);
}

std::optional<std::vector<bool>> MotionController::get_motion_io_status(QString id, int axis) const
{
	if (!valid(id)) return std::nullopt;
	return m_motion[id]->get_motion_io_status(axis);
}

bool MotionController::set_position_mm(QString id, int cardID, int axis, double pos_mm)
{
	if (!valid(id)) return false;
	return m_motion[id]->set_position_mm(cardID, axis, pos_mm);
}

bool MotionController::set_servo(QString id, int cardID, int axis, bool on)
{
	if (!valid(id)) return false;
	return m_motion[id]->set_servo(cardID, axis, on);
}

bool MotionController::sync_command_to_feedback(QString id, int axis)
{
	if (!valid(id)) return false;
	return m_motion[id]->sync_command_to_feedback(axis);
}

bool MotionController::set_DO(QString id, int cardID, int no, bool state)
{
	if (!valid(id)) return false;
	return m_motion[id]->set_DO(cardID, no, state);
}

bool MotionController::set_all_DO(QString id, int cardID, const std::vector<bool>& states)
{
	if (!valid(id)) return false;
	return m_motion[id]->set_all_DO(cardID, states);
}

bool MotionController::home(QString id, int axis)
{
	if (!valid(id)) return false;
	return m_motion[id]->home(axis);
}

bool MotionController::continuous_move(QString id, int axis, bool positive_direction)
{
	if (!m_enableMotion) { gantrylog::write("VJOG   %s %s - REFUSED: motion is disabled (home or clear the error)", gantrylog::axisName(axis), positive_direction ? "+" : "-"); return false; }
	if (!valid(id)) return false;
	return m_motion[id]->continuous_move(axis, positive_direction);
}

bool MotionController::absolute_move(QString id, int axis, double position_mm)
{
	if (!m_enableMotion) { gantrylog::write("ABS    %s -> %.4f - REFUSED: motion is disabled (home or clear the error)", gantrylog::axisName(axis), position_mm); return false; }
	if (!valid(id)) return false;
	return m_motion[id]->absolute_move(axis, position_mm);
}

/*
* The gantry log of 4 Nov 2026 showed X's encoder moving 0.21 mm while ONLY Y moved, the card still
* counting X where it had put it (INP on) - so every later X move stopped 0.28 mm short by the encoder,
* and re-sending the same target never moved it: the card believes it is there. The camera agrees with
* the encoder (fid0 measured through it stayed put within 0.02 mm), so the encoder is the truth and a
* move is aimed past by the gap. Its landing is still judged on the encoder by every caller.
* - X / Y only: aiming past in Z could lower the head, and Z has shown no gap.
* - Up to system.json "Encoder_Gap_Correction_Max_mm" (default 0.5, kept to 0.05-5.0 on load): a
*   bigger gap is an axis pushed out of place - sent as is, so its landing fails loudly instead of
*   the machine quietly steering round it. Home clears the gap.
* - Only a gap past kMinGapMm, the landing tolerance: under it the move lands within tolerance anyway,
*   and right after a move the gap is partly settling that is still closing (Y read -0.046 at its
*   settle and -0.006 a move later) - aiming past that would overshoot. Each correction is a warning
*   in the main log, so a slipping axis never goes quiet.
* - An aimed target outside the soft limits is sent as the plain target: the card would refuse (Y) or
*   clamp (X) a move whose real end is inside them.
* - system.json "Encoder_Gap_Correction" false turns it off (the motion team, or once X is fixed).
* The Motion Accuracy Test sends its moves without this - it measures the raw axis.
*/
double MotionController::aim_for_encoder(QString id, int axis, double target_mm)
{
	constexpr double kMinGapMm = 0.05;   //the landing tolerance (JobThread::settleAxis, zsMoveGroup)
	const double maxGapMm = SystemData::instance()._encoderGapMaxMm.load();

	if (axis != 0 && axis != 1) return target_mm; //HARDCODE: X = 0, Y = 1, as Motion_APS
	if (!SystemData::instance()._encoderGapCorrection.load()) return target_mm;
	//moving, the gap is the drive's following lag, not a drift - only a standing axis is corrected
	if (!move_done(id, 0, axis)) {
		gantrylog::write("CORR   %s target %.4f: axis still moving - not corrected", gantrylog::axisName(axis), target_mm);
		return target_mm;
	}
	const auto cmd = get_command_mm(id, 0, axis);
	const auto fbk = get_position_mm(id, 0, axis);
	if (!cmd || !fbk) return target_mm;

	const double gap = *fbk - *cmd;
	const char* name = gantrylog::axisName(axis);
	if (std::abs(gap) <= kMinGapMm) return target_mm;
	if (std::abs(gap) > maxGapMm) {
		ct::logger::warn("[Motion] %s: the encoder is %+.3f mm from the card's count - more than the %.3f mm corrected (Encoder_Gap_Correction_Max_mm), "
			"sent uncorrected (the axis was pushed or is slipping - home to clear it)", name, gap, maxGapMm);
		gantrylog::write("CORR   %s target %.4f: encoder %+.4f from the card's count - OVER %.3f, NOT corrected", name, target_mm, gap, maxGapMm);
		return target_mm;
	}
	const double send = target_mm - gap;
	double lo = 0.0, hi = 0.0;
	if (limits_mm(id, axis, lo, hi) && lo < hi && (send < lo || send > hi)) {
		gantrylog::write("CORR   %s target %.4f: encoder %+.4f from the card's count, but %.4f is outside the soft limits %.3f..%.3f - NOT corrected",
			name, target_mm, gap, send, lo, hi);
		return target_mm;
	}
	ct::logger::warn("[Motion] %s: the encoder is %+.3f mm from the card's count (the axis drifted) - aiming at %.3f so %s lands on %.3f",
		name, gap, send, name, target_mm);
	gantrylog::write("CORR   %s target %.4f: encoder %+.4f from the card's count - sent %.4f", name, target_mm, gap, send);
	return send;
}

bool MotionController::absolute_multi_move(QString id, const std::vector<nvs::motion::MoveParam>& moveParams)
{
	if (!m_enableMotion) { gantrylog::write("LINEAR %d axes - REFUSED: motion is disabled (home or clear the error)", (int)moveParams.size()); return false; }
	if (!valid(id)) return false;
	return m_motion[id]->absolute_multi_move(moveParams);
}

bool MotionController::relative_move(QString id, int axis, double distance_mm)
{
	if (!m_enableMotion) { gantrylog::write("REL    %s by %+.4f - REFUSED: motion is disabled (home or clear the error)", gantrylog::axisName(axis), distance_mm); return false; }
	if (!valid(id)) return false;
	return m_motion[id]->relative_move(axis, distance_mm);
}

bool MotionController::stop_move(QString id, int axis)
{
	if (!valid(id)) return false;
	return m_motion[id]->stop_move(axis);
}

bool MotionController::move_done(QString id, int cardID, int axis)
{
	if (!valid(id)) return false;
	return m_motion[id]->move_done(cardID, axis);
}

QString MotionController::error_msg(QString id)
{
	if (!valid(id)) return "";
	return m_motion[id]->error_msg().c_str();
}

bool MotionController::is_init(QString id)
{
	return m_initStatus[id];
}

bool MotionController::is_safe(QString id, int axis, double position_mm)
{
	if (!valid(id)) return false;
	return m_motion[id]->is_safe(axis, position_mm);
}

bool MotionController::get_soft_limit(QString id, int axis, double position_mm, double& softLimit_mm)
{
	if (!valid(id)) return false;
	return m_motion[id]->get_soft_limit(axis, position_mm, softLimit_mm);
}

bool MotionController::load_config(QString id, QString path)
{
	if (!valid(id)) return false;
	return m_motion[id]->load_config(path.toStdString());
}
