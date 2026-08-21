#include "VisionApp.h"
#include "MotionController.h"
#include "MachineController.h"
#include "nvsutil/nvs_qt.h"
#include "AuditLog.h"

using namespace nvs::motion;

void VisionApp::initMotion() {

	//load motion
	auto motionPath = QStringLiteral("%1/motion.json").arg(Common::Directory::ConfigPath());
	MotionController::instance().load_config(motionPath);

	bool motionEnabled = true;
	if (loadMotion()) {
		for (const auto& m : _motions) {

			motionEnabled &= m.enable;
			if (!m.enable) continue;

			//axes
			for (auto axis : m.axes) {

				QString name = axis.name.c_str();
				auto move_mvel = axis.move_max_velocity;
				auto move_accel = axis.move_acceleration;

				if (name.contains("X-Axis")) {
					_jobThread.setXSpeed(move_mvel, move_mvel / 10);
					ui.lineEdit_x_velocity->setText(QString::number(move_mvel));
					ui.lineEdit_x_acceleration->setText(QString::number(move_accel));
				}
				else if (name.contains("Y-Axis")) {
					ui.lineEdit_y_velocity->setText(QString::number(move_mvel));
					ui.lineEdit_y_acceleration->setText(QString::number(move_accel));
				}
				else if (name.contains("Z-Axis")) {
					ui.lineEdit_z_velocity->setText(QString::number(move_mvel));
					ui.lineEdit_z_acceleration->setText(QString::number(move_accel));
				}
			}
		}
	}


	if (!motionEnabled) return;

	//Digital Input
	_motionTimer = new QTimer();
	connect(_motionTimer, &QTimer::timeout, this, [this]() {

		vs_updateUptimer();

		double x = 0.0, y = 0.0, z = 0.0; //potential bug

		auto optional_x =  MotionController::instance().get_position_mm(_motionID, 0, (int)Axis::X);
		if (optional_x.has_value()) {
			x = optional_x.value();
			ui.lineEdit_x->setText(QString::number(x));
		}

		auto optional_y =  MotionController::instance().get_position_mm(_motionID, 0, (int)Axis::Y);
		if (optional_y.has_value()) {
			y = optional_y.value();
			ui.lineEdit_y->setText(QString::number(y));
		}

		auto optional_z =  MotionController::instance().get_position_mm(_motionID, 0, (int)Axis::Z);
		if (optional_z.has_value()) {
			z = optional_z.value();
			ui.lineEdit_z->setText(QString::number(z));
		}

		auto o_x_speed = MotionController::instance().get_current_speed(_motionID, 0, (int)Axis::X);
		if (o_x_speed.has_value()) ui.lineEdit_x_currentSpeed->setText(QString::number(o_x_speed.value()));
			
		auto o_y_speed = MotionController::instance().get_current_speed(_motionID, 0, (int)Axis::Y);
		if (o_y_speed.has_value()) ui.lineEdit_y_currentSpeed->setText(QString::number(o_y_speed.value()));

		auto o_z_speed = MotionController::instance().get_current_speed(_motionID, 0, (int)Axis::Z);
		if (o_z_speed.has_value()) ui.lineEdit_z_currentSpeed->setText(QString::number(o_z_speed.value()));

		SystemData::instance().setCurrentCoordinate(x, y, z);

		bool gantryState = true;

		auto optional_axisX =  MotionController::instance().get_motion_io_status(_motionID, (int)Axis::X);
		if (optional_axisX.has_value()) {
			auto motion_io = optional_axisX.value();

			if (motion_io.size() != 9) ct::logger::error("[Motion_APS] Invalid size for motion io");

			nvs::set_background_color(ui.toolButton_EMXA_X_ALM, motion_io[Motion_APS::ALM] ? Qt::green : Qt::red);
			nvs::set_background_color(ui.toolButton_EMXA_X_RDY, motion_io[Motion_APS::RDY] ? Qt::green : Qt::red);
			nvs::set_background_color(ui.toolButton_EMXA_X_PEL, motion_io[Motion_APS::PEL] ? Qt::green : Qt::red);
			nvs::set_background_color(ui.toolButton_EMXA_X_NEL, motion_io[Motion_APS::NEL] ? Qt::green : Qt::red);
			nvs::set_background_color(ui.toolButton_EMXA_X_ORG, motion_io[Motion_APS::ORG] ? Qt::green : Qt::red);
			nvs::set_background_color(ui.toolButton_EMXA_X_EMG, motion_io[Motion_APS::EMG] ? Qt::green : Qt::red);
			nvs::set_background_color(ui.toolButton_EMXA_X_INP, motion_io[Motion_APS::INP] ? Qt::green : Qt::red);
			nvs::set_background_color(ui.toolButton_EMXA_X_SVON, motion_io[Motion_APS::SVON] ? Qt::green : Qt::red);
			gantryState &= motion_io[Motion_APS::SVON];
		}

		auto optional_axisY =  MotionController::instance().get_motion_io_status(_motionID, (int)Axis::Y);
		if (optional_axisY.has_value()) {
			auto motion_io = optional_axisY.value();

			if (motion_io.size() != 9) ct::logger::error("[Motion_APS] Invalid size for motion io");

			nvs::set_background_color(ui.toolButton_EMXA_Y_ALM, motion_io[Motion_APS::ALM] ? Qt::green : Qt::red);
			nvs::set_background_color(ui.toolButton_EMXA_Y_RDY, motion_io[Motion_APS::RDY] ? Qt::green : Qt::red);
			nvs::set_background_color(ui.toolButton_EMXA_Y_PEL, motion_io[Motion_APS::PEL] ? Qt::green : Qt::red);
			nvs::set_background_color(ui.toolButton_EMXA_Y_NEL, motion_io[Motion_APS::NEL] ? Qt::green : Qt::red);
			nvs::set_background_color(ui.toolButton_EMXA_Y_ORG, motion_io[Motion_APS::ORG] ? Qt::green : Qt::red);
			nvs::set_background_color(ui.toolButton_EMXA_Y_EMG, motion_io[Motion_APS::EMG] ? Qt::green : Qt::red);
			nvs::set_background_color(ui.toolButton_EMXA_Y_INP, motion_io[Motion_APS::INP] ? Qt::green : Qt::red);
			nvs::set_background_color(ui.toolButton_EMXA_Y_SVON, motion_io[Motion_APS::SVON] ? Qt::green : Qt::red);
			gantryState &= motion_io[Motion_APS::SVON];
		}

		auto optional_axisZ =  MotionController::instance().get_motion_io_status(_motionID, (int)Axis::Z);
		if (optional_axisZ.has_value()) {
			auto motion_io = optional_axisZ.value();

			if (motion_io.size() != 9) ct::logger::error("[Motion_APS] Invalid size for motion io");

			nvs::set_background_color(ui.toolButton_EMXA_Z_ALM, motion_io[Motion_APS::ALM] ? Qt::green : Qt::red);
			nvs::set_background_color(ui.toolButton_EMXA_Z_RDY, motion_io[Motion_APS::RDY] ? Qt::green : Qt::red);
			nvs::set_background_color(ui.toolButton_EMXA_Z_PEL, motion_io[Motion_APS::PEL] ? Qt::green : Qt::red);
			nvs::set_background_color(ui.toolButton_EMXA_Z_NEL, motion_io[Motion_APS::NEL] ? Qt::green : Qt::red);
			nvs::set_background_color(ui.toolButton_EMXA_Z_ORG, motion_io[Motion_APS::ORG] ? Qt::green : Qt::red);
			nvs::set_background_color(ui.toolButton_EMXA_Z_EMG, motion_io[Motion_APS::EMG] ? Qt::green : Qt::red);
			nvs::set_background_color(ui.toolButton_EMXA_Z_INP, motion_io[Motion_APS::INP] ? Qt::green : Qt::red);
			nvs::set_background_color(ui.toolButton_EMXA_Z_SVON, motion_io[Motion_APS::SVON] ? Qt::green : Qt::red);
			gantryState &= motion_io[Motion_APS::SVON];
		}

		nvs::set_background_color(ui.toolButton_gantryStatus, gantryState ? Qt::green : Qt::red);

		auto optional_EMXA_DIs =  MotionController::instance().get_all_DI(_motionID, 0);
		if (optional_EMXA_DIs.has_value()) {
			auto EMXA_DIs = optional_EMXA_DIs.value();

			for (int i = 0; i < EMXA_DIs.size(); i++) {
				//ct::logger::info("EMXA DI%d: %d", i, (EMXA_DIs[i])? 1:0);

				auto button = findChild<QToolButton*>(QString("toolButton_EMXA_DI%1").arg(i));
				if (!button) continue;

				nvs::set_background_color(button, EMXA_DIs[i] ? Qt::green : Qt::red);
			}
		}

		auto optional_EMXA_DOs = MotionController::instance().get_all_DO(_motionID, 0);
		if (optional_EMXA_DOs.has_value()) {
			auto EMXA_DOs = optional_EMXA_DOs.value();

			for (int i = 0; i < EMXA_DOs.size(); i++) {

				auto button = findChild<QToolButton*>(QString("toolButton_EMXA_DO%1").arg(i));
				if (!button) continue;
				nvs::set_background_color(button, EMXA_DOs[i] ? Qt::green : Qt::red);
			}
		}

		nvs::set_background_color(ui.toolButton_machineReady, SystemData::instance()._Machine_Ready ? Qt::green : Qt::red);

		ui.lineEdit_totalBoardInspection->setText(QString::number(SystemData::instance()._BoardEntryQty));

	});
	_motionTimer->start(300);

	//Digital Output
	for (int i = 0; i < 14; ++i) {
		auto button = findChild<QToolButton*>(QString("toolButton_EMXA_DO%1").arg(i));
		if (!button) continue;

		nvs::set_background_color(button, Qt::red);
		auto o =  MotionController::instance().get_DO(_motionID, 0, i);
		if (o.has_value()) nvs::set_background_color(button, o.value() ? Qt::green : Qt::red);

		connect(button, &QToolButton::clicked, this, [=]() {
			ct::logger::info("[Motion] Pressing EMX(A) DO%d", i);
			auto o =  MotionController::instance().get_DO(_motionID, 0, i);
			if (!o.has_value()) { showMsg("Invalid Digital Parameter"); return; }
			auto new_output = !o.value();
			 MotionController::instance().set_DO(_motionID, 0, i, new_output);
			nvs::set_background_color(button, new_output ? Qt::green : Qt::red);
		});
	}

	//Controls
	connect(ui.lineEdit_stepMM, &QLineEdit::textEdited, this, [=](QString text) {
		auto step = ui.lineEdit_stepMM->text().toDouble();
		if (step < 0) {
			step = 0.0;
			ui.lineEdit_stepMM->setText(QString::number(step));
		}
		if (step > 100) {
			step = 100;
			ui.lineEdit_stepMM->setText(QString::number(step));
		}

		_jogDistance = step;
	});

	connect(ui.toolButton_homeX, &QToolButton::clicked, this, [=]() {
		ct::logger::info("[Motion] Homing X");
		AuditLog::instance().log(QStringLiteral("HOME"), QStringLiteral("X"));
		emit homeX();
	});

	connect(ui.toolButton_homeY, &QToolButton::clicked, this, [=]() {
		ct::logger::info("[Motion] Homing Y");
		AuditLog::instance().log(QStringLiteral("HOME"), QStringLiteral("Y"));
		emit homeY();
	});

	connect(ui.toolButton_homeZ, &QToolButton::clicked, this, [=]() {
		ct::logger::info("[Motion] Homing Z");
		AuditLog::instance().log(QStringLiteral("HOME"), QStringLiteral("Z"));
		emit homeZ();
	});

	connect(ui.toolButton_homeXYZ, &QToolButton::clicked, this, [=]() {
		ct::logger::info("[Motion] Homing XYZ");
		AuditLog::instance().log(QStringLiteral("HOME"), QStringLiteral("XYZ"));
		emit homeXYZ();
	});

	connect(ui.toolButton_homeAll, &QToolButton::clicked, this, [=]() {
		ct::logger::info("[Motion] Homing All");
		AuditLog::instance().log(QStringLiteral("HOME"), QStringLiteral("All"));
		emit homeAll();
	});

	connect(ui.toolButton_jogLeft, &QToolButton::clicked, this, [=]() {
		if (!blockJogSignal()) return;
		ct::logger::info("[Motion] Jog left");
		emit jogLeft(_jogDistance, _mainOptics[_camID]);
	});

	connect(ui.toolButton_jogRight, &QToolButton::clicked, this, [=]() {
		if (!blockJogSignal()) return;
		ct::logger::info("[Motion] Jog right");
		emit jogRight(_jogDistance, _mainOptics[_camID]);
	});

	connect(ui.toolButton_jogFront, &QToolButton::clicked, this, [=]() {
		if (!blockJogSignal()) return;
		ct::logger::info("[Motion] Jog front"); // Jog Back
		emit jogFront(_jogDistance, _mainOptics[_camID]);
	});

	connect(ui.toolButton_jogBack, &QToolButton::clicked, this, [=]() {
		if (!blockJogSignal()) return;
		ct::logger::info("[Motion] Jog back"); // Jog Front toward us
		emit jogBack(_jogDistance, _mainOptics[_camID]);
	});

	connect(ui.toolButton_jogTop, &QToolButton::clicked, this, [=]() {
		if (!blockJogSignal()) return;
		ct::logger::info("[Motion] Jog top");
		emit jogUp(_jogDistance, _mainOptics[_camID]);
	});

	connect(ui.toolButton_jogBottom, &QToolButton::clicked, this, [=]() {
		if (!blockJogSignal()) return;
		ct::logger::info("[Motion] Jog bottom");
		emit jogDown(_jogDistance, _mainOptics[_camID]);
	});

	//Servo
	connect(ui.toolButton_servo_x, &QToolButton::clicked, this, [=]() {
		ct::logger::info("[Motion] Toggle servo");
		auto optional_io =  MotionController::instance().get_motion_io_status(_motionID, (int)Axis::X);
		
		if (!optional_io.has_value()) {
			showMsg("Failed to toggle servo");
			return;
		}

		auto io = optional_io.value();
		auto state = !io[(int)Motion_APS::IO::SVON];

		auto o =  MotionController::instance().set_servo(_motionID, 0, (int)Axis::X, state);
		AuditLog::instance().log(QStringLiteral("SERVO_TOGGLE"), QStringLiteral("X"), o ? (state ? QStringLiteral("ON") : QStringLiteral("OFF")) : QStringLiteral("FAILED"));

		if (o && state) MachineController::instance().notifyEvent(MachineEvent::X_SERVO_ON);
	});

	connect(ui.toolButton_servo_y, &QToolButton::clicked, this, [=]() {
		ct::logger::info("[Motion] Toggle servo");
		auto optional_io =  MotionController::instance().get_motion_io_status(_motionID, (int)Axis::Y);

		if (!optional_io.has_value()) {
			showMsg("Failed to toggle servo");
			return;
		}

		auto io = optional_io.value();
		auto state = !io[(int)Motion_APS::IO::SVON];

		auto o =  MotionController::instance().set_servo(_motionID, 0, (int)Axis::Y, state);
		AuditLog::instance().log(QStringLiteral("SERVO_TOGGLE"), QStringLiteral("Y"), o ? (state ? QStringLiteral("ON") : QStringLiteral("OFF")) : QStringLiteral("FAILED"));

		if (o && state) MachineController::instance().notifyEvent(MachineEvent::Y_SERVO_ON);
	});

	connect(ui.toolButton_servo_z, &QToolButton::clicked, this, [=]() {
		ct::logger::info("[Motion] Toggle servo");
		auto optional_io =  MotionController::instance().get_motion_io_status(_motionID, (int)Axis::Z);

		if (!optional_io.has_value()) {
			showMsg("Failed to toggle servo");
			return;
		}

		auto io = optional_io.value();
		auto state = !io[(int)Motion_APS::IO::SVON];

		if (!state) {
			MachineController::instance().turnOnBrake();
		}

		auto o =  MotionController::instance().set_servo(_motionID, 0, (int)Axis::Z, state);
		AuditLog::instance().log(QStringLiteral("SERVO_TOGGLE"), QStringLiteral("Z"), o ? (state ? QStringLiteral("ON") : QStringLiteral("OFF")) : QStringLiteral("FAILED"));

		if (o && state) MachineController::instance().notifyEvent(MachineEvent::Z_SERVO_ON);

		if (state) {
			MachineController::instance().safelyReleaseBrake();
		}
	});

	//Velocity
	connect(ui.toolButton_updateVelocity_x, &QToolButton::clicked, this, [=]() {
		ct::logger::info("[Motion] Update velocity x");
		auto velocity = ui.lineEdit_x_velocity->text().toInt();
		auto velocity3d = ui.lineEdit_x_velocity3d->text().toInt();
		auto acel = ui.lineEdit_x_acceleration->text().toInt();
		auto axis = (int)Axis::X;
		AuditLog::instance().log(QStringLiteral("VELOCITY_UPDATE"), QStringLiteral("X vel=%1 acc=%2").arg(velocity).arg(acel));

		auto ret =  MotionController::instance().set_move_max_velocity(_motionID, axis, velocity);
		ret &=  MotionController::instance().set_move_acceleration(_motionID, axis, acel);
		ret &= MotionController::instance().set_move_deceleration(_motionID, axis, acel);

		if (ret) {
			_jobThread.setXSpeed(velocity, velocity3d);
			_jobThread.setXDecel(acel);
			saveRecipeConfig();
			showMsg("Updated velocity");
		}
		else {
			showMsg("Failed to update velocity");
		}
	});

	connect(ui.toolButton_updateVelocity_y, &QToolButton::clicked, this, [=]() {
		ct::logger::info("[Motion] Update velocity y");
		auto velocity = ui.lineEdit_y_velocity->text().toInt();
		auto acel = ui.lineEdit_y_acceleration->text().toInt();
		auto axis = (int)Axis::Y;
		AuditLog::instance().log(QStringLiteral("VELOCITY_UPDATE"), QStringLiteral("Y vel=%1 acc=%2").arg(velocity).arg(acel));

		auto ret =  MotionController::instance().set_move_max_velocity(_motionID, axis, velocity);
		ret &=  MotionController::instance().set_move_acceleration(_motionID, axis, acel);
		ret &= MotionController::instance().set_move_deceleration(_motionID, axis, acel);

		if (ret) {
			auto& m = _motions[_motionID].axes[axis];
			m.move_max_velocity = velocity;
			m.move_acceleration = acel;
			m.move_deceleration = acel;
			saveMotion();
			showMsg("Updated velocity");
		}
		else {
			showMsg("Failed to update velocity");
		}
	});

	connect(ui.toolButton_updateVelocity_z, &QToolButton::clicked, this, [=]() {
		ct::logger::info("[Motion] Update velocity z");
		auto velocity = ui.lineEdit_z_velocity->text().toInt();
		auto acel = ui.lineEdit_z_acceleration->text().toInt();
		auto axis = (int)Axis::Z;
		AuditLog::instance().log(QStringLiteral("VELOCITY_UPDATE"), QStringLiteral("Z vel=%1 acc=%2").arg(velocity).arg(acel));

		auto ret =  MotionController::instance().set_move_max_velocity(_motionID, axis, velocity);
		ret &=  MotionController::instance().set_move_acceleration(_motionID, axis, acel);
		ret &= MotionController::instance().set_move_deceleration(_motionID, axis, acel);

		if (ret) {
			auto& m = _motions[_motionID].axes[axis];
			m.move_max_velocity = velocity;
			m.move_acceleration = acel;
			m.move_deceleration = acel;
			saveMotion();
			showMsg("Updated velocity");
		}
		else {
			showMsg("Failed to update velocity");
		}
	});

}
