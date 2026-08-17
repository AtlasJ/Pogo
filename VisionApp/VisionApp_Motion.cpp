#include "VisionApp.h"
#include "MotionController.h"
#include "MachineController.h"
#include "MachineSMEMAManager.h"
#include "nvsutil/nvs_qt.h"
#include "AuditLog.h"

using namespace nvs::motion;

void VisionApp::initMotion() {

	//hide unused 
	ui.label_conveyor2->hide();
	ui.label_conveyor2_2->hide();
	ui.label_lane2_servo->hide();
	ui.toolButton_servo_lane2->hide();
	ui.label_rail2_servo->hide();
	ui.label_flexiRail2->hide();
	ui.toolButton_servo_rail2->hide();

	ui.label_rail3_servo->hide();
	ui.label_flexiRail3->hide();
	ui.toolButton_servo_rail3->hide();

	ui.toolButton_EMXB_R2_ALM->hide();
	ui.toolButton_EMXB_R2_EMG->hide();
	ui.toolButton_EMXB_R2_INP->hide();
	ui.toolButton_EMXB_R2_NEL->hide();
	ui.toolButton_EMXB_R2_PEL->hide();
	ui.toolButton_EMXB_R2_ORG->hide();
	ui.toolButton_EMXB_R2_SVON->hide();
	ui.toolButton_EMXB_R3_ALM->hide();
	ui.toolButton_EMXB_R3_EMG->hide();
	ui.toolButton_EMXB_R3_INP->hide();
	ui.toolButton_EMXB_R3_NEL->hide();
	ui.toolButton_EMXB_R3_PEL->hide();
	ui.toolButton_EMXB_R3_ORG->hide();
	ui.toolButton_EMXB_R3_SVON->hide();

	ui.label_Y203->hide();
	ui.label_Y203_->hide();
	ui.toolButton_EMXB_DO3->hide();
	
	ui.toolButton_EMXB_DO9->hide();
	ui.toolButton_EMXB_DO10->hide();
	ui.toolButton_EMXB_DO11->hide();
	ui.toolButton_EMXB_DO12->hide();

	ui.label_231->setText("EMG");

	std::vector<int> hideIndex = { 16, 17 };
	for (auto i : hideIndex) {
		auto label = findChild<QLabel*>(QString("label_X1%1").arg(i));
		if (label) label->hide();

		auto label1 = findChild<QLabel*>(QString("label_X1%1_").arg(i));
		if (label1) label1->hide();

		auto label2 = findChild<QLabel*>(QString("label_Y1%1").arg(i));
		if (label2) label2->hide();

		auto label3 = findChild<QLabel*>(QString("label_Y1%1_").arg(i));
		if (label3) label3->hide();

		auto button = findChild<QToolButton*>(QString("toolButton_EMXA_DI%1").arg(i));
		if (button) button->hide();
	}


	for (int i = 0; i <= 9; i++) {
		auto label = findChild<QLabel*>(QString("label_X20%1").arg(i));
		if (label) label->hide();
		
		auto label1 = findChild<QLabel*>(QString("label_X20%1_").arg(i));
		if (label1) label1->hide();

		auto label2 = findChild<QLabel*>(QString("label_Y20%1").arg(i));
		if (label2) label2->hide();

		auto label3 = findChild<QLabel*>(QString("label_Y20%1_").arg(i));
		if (label3) label3->hide();

		auto button = findChild<QToolButton*>(QString("toolButton_EMXB_DI%1").arg(i));
		if (button) button->hide();
	}
	for (int i = 10; i <= 13; i++) {
		auto label = findChild<QLabel*>(QString("label_X2%1").arg(i));
		if (label) label->hide();

		auto label1 = findChild<QLabel*>(QString("label_X2%1_").arg(i));
		if (label1) label1->hide();

		auto label2 = findChild<QLabel*>(QString("label_Y2%1").arg(i));
		if (label2) label2->hide();

		auto label3 = findChild<QLabel*>(QString("label_Y2%1_").arg(i));
		if (label3) label3->hide();

		auto button = findChild<QToolButton*>(QString("toolButton_EMXB_DI%1").arg(i));
		if (button) button->hide();
	}

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
				else if (name.contains("Conveyor")) {
					ui.lineEdit_cy1_velocity->setText(QString::number(move_mvel));
					ui.lineEdit_cy1_acceleration->setText(QString::number(move_accel));
				}
				else if (name.contains("Rail")) {
					ui.lineEdit_fr1_velocity->setText(QString::number(move_mvel));
					ui.lineEdit_fr1_acceleration->setText(QString::number(move_accel));
				}
			}
		}
	}


	if (!motionEnabled) return;

	//Update UI
	auto optional_fr1 = MotionController::instance().get_position_mm(_motionID, 0, (int)Axis::FR1);
	if (optional_fr1.has_value()) {
		auto fr1 = optional_fr1.value();
		ui.lineEdit_railWidth1->setText(QString::number(fr1));
	}
	
	ct::logger::debug("[Motion] Updated Rail UI");

	struct AxisEntry {
		Axis axis;
		QLineEdit* edit;
	};

	//Motion IO Status

	// RESET SMEMA OUTPUT OFF
	MotionController::instance().set_DO(_motionID, 1, (int)DOB::DOWNSTREAM, false);
	QThread::msleep(100); // must have delay, dont remove
	MotionController::instance().set_DO(_motionID, 1, (int)DOB::UPSTREAM, false);
	
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

		auto optional_rail = MotionController::instance().get_position_mm(_motionID, 0, (int)Axis::FR1);
		if (optional_rail.has_value()) {
			auto r = optional_rail.value();
			ui.lineEdit_railWidth->setText(QString::number(r));
		}

		auto o_x_speed = MotionController::instance().get_current_speed(_motionID, 0, (int)Axis::X);
		if (o_x_speed.has_value()) ui.lineEdit_x_currentSpeed->setText(QString::number(o_x_speed.value()));
			
		auto o_y_speed = MotionController::instance().get_current_speed(_motionID, 0, (int)Axis::Y);
		if (o_y_speed.has_value()) ui.lineEdit_y_currentSpeed->setText(QString::number(o_y_speed.value()));

		auto o_z_speed = MotionController::instance().get_current_speed(_motionID, 0, (int)Axis::Z);
		if (o_z_speed.has_value()) ui.lineEdit_z_currentSpeed->setText(QString::number(o_z_speed.value()));

		SystemData::instance().setCurrentCoordinate(x, y, z);

		bool gantryState = true;
		bool EMG_X_Axis = false;
		bool EMG_Y_Axis = false;
		bool EMG_Z_Axis = false;

		auto optional_axisX =  MotionController::instance().get_motion_io_status(_motionID, (int)Axis::X);
		if (optional_axisX.has_value()) {
			auto motion_io = optional_axisX.value();

			if (motion_io.size() != 9) ct::logger::error("[Motion_APS] Invalid size for motion io");

			nvs::set_background_color(ui.toolButton_EMXA_X_ALM, motion_io[Motion_APS::ALM] ? Qt::green : Qt::red);
			nvs::set_background_color(ui.toolButton_EMXA_X_PEL, motion_io[Motion_APS::PEL] ? Qt::green : Qt::red);
			nvs::set_background_color(ui.toolButton_EMXA_X_NEL, motion_io[Motion_APS::NEL] ? Qt::green : Qt::red);
			nvs::set_background_color(ui.toolButton_EMXA_X_ORG, motion_io[Motion_APS::ORG] ? Qt::green : Qt::red);
			nvs::set_background_color(ui.toolButton_EMXA_X_EMG, motion_io[Motion_APS::EMG] ? Qt::green : Qt::red);
			nvs::set_background_color(ui.toolButton_EMXA_X_INP, motion_io[Motion_APS::INP] ? Qt::green : Qt::red);
			nvs::set_background_color(ui.toolButton_EMXA_X_SVON, motion_io[Motion_APS::SVON] ? Qt::green : Qt::red);
			gantryState &= motion_io[Motion_APS::SVON];
			EMG_X_Axis = motion_io[Motion_APS::EMG];
		}

		auto optional_axisY =  MotionController::instance().get_motion_io_status(_motionID, (int)Axis::Y);
		if (optional_axisY.has_value()) {
			auto motion_io = optional_axisY.value();

			if (motion_io.size() != 9) ct::logger::error("[Motion_APS] Invalid size for motion io");

			nvs::set_background_color(ui.toolButton_EMXA_Y_ALM, motion_io[Motion_APS::ALM] ? Qt::green : Qt::red);
			nvs::set_background_color(ui.toolButton_EMXA_Y_PEL, motion_io[Motion_APS::PEL] ? Qt::green : Qt::red);
			nvs::set_background_color(ui.toolButton_EMXA_Y_NEL, motion_io[Motion_APS::NEL] ? Qt::green : Qt::red);
			nvs::set_background_color(ui.toolButton_EMXA_Y_ORG, motion_io[Motion_APS::ORG] ? Qt::green : Qt::red);
			nvs::set_background_color(ui.toolButton_EMXA_Y_EMG, motion_io[Motion_APS::EMG] ? Qt::green : Qt::red);
			nvs::set_background_color(ui.toolButton_EMXA_Y_INP, motion_io[Motion_APS::INP] ? Qt::green : Qt::red);
			nvs::set_background_color(ui.toolButton_EMXA_Y_SVON, motion_io[Motion_APS::SVON] ? Qt::green : Qt::red);
			gantryState &= motion_io[Motion_APS::SVON];
			EMG_Y_Axis = motion_io[Motion_APS::EMG];
		}

		auto optional_axisZ =  MotionController::instance().get_motion_io_status(_motionID, (int)Axis::Z);
		if (optional_axisZ.has_value()) {
			auto motion_io = optional_axisZ.value();

			if (motion_io.size() != 9) ct::logger::error("[Motion_APS] Invalid size for motion io");

			nvs::set_background_color(ui.toolButton_EMXA_Z_ALM, motion_io[Motion_APS::ALM] ? Qt::green : Qt::red);
			nvs::set_background_color(ui.toolButton_EMXA_Z_PEL, motion_io[Motion_APS::PEL] ? Qt::green : Qt::red);
			nvs::set_background_color(ui.toolButton_EMXA_Z_NEL, motion_io[Motion_APS::NEL] ? Qt::green : Qt::red);
			nvs::set_background_color(ui.toolButton_EMXA_Z_ORG, motion_io[Motion_APS::ORG] ? Qt::green : Qt::red);
			nvs::set_background_color(ui.toolButton_EMXA_Z_EMG, motion_io[Motion_APS::EMG] ? Qt::green : Qt::red);
			nvs::set_background_color(ui.toolButton_EMXA_Z_INP, motion_io[Motion_APS::INP] ? Qt::green : Qt::red);
			nvs::set_background_color(ui.toolButton_EMXA_Z_SVON, motion_io[Motion_APS::SVON] ? Qt::green : Qt::red);
			gantryState &= motion_io[Motion_APS::SVON];
			EMG_Z_Axis = motion_io[Motion_APS::EMG];
		}

		nvs::set_background_color(ui.toolButton_gantryStatus, gantryState ? Qt::green : Qt::red);

		auto optional_axisR1 =  MotionController::instance().get_motion_io_status(_motionID, (int)Axis::FR1);
		if (optional_axisR1.has_value()) {
			auto motion_io = optional_axisR1.value();

			if (motion_io.size() != 9) ct::logger::error("[Motion_APS] Invalid size for motion io");

			nvs::set_background_color(ui.toolButton_EMXA_R1_ALM, motion_io[Motion_APS::ALM] ? Qt::green : Qt::red);
			nvs::set_background_color(ui.toolButton_EMXA_R1_PEL, motion_io[Motion_APS::PEL] ? Qt::green : Qt::red);
			nvs::set_background_color(ui.toolButton_EMXA_R1_NEL, motion_io[Motion_APS::NEL] ? Qt::green : Qt::red);
			nvs::set_background_color(ui.toolButton_EMXA_R1_ORG, motion_io[Motion_APS::ORG] ? Qt::green : Qt::red);
			nvs::set_background_color(ui.toolButton_EMXA_R1_EMG, motion_io[Motion_APS::EMG] ? Qt::green : Qt::red);
			nvs::set_background_color(ui.toolButton_EMXA_R1_INP, motion_io[Motion_APS::INP] ? Qt::green : Qt::red);
			nvs::set_background_color(ui.toolButton_EMXA_R1_SVON, motion_io[Motion_APS::SVON] ? Qt::green : Qt::red);
			nvs::set_background_color(ui.toolButton_railStatus, motion_io[Motion_APS::SVON] ? Qt::green : Qt::red);
		}

		auto optional_axisCY1 = MotionController::instance().get_motion_io_status(_motionID, (int)Axis::CY1);
		if (optional_axisCY1.has_value()) {
			auto motion_io = optional_axisCY1.value();

			if (motion_io.size() != 9) ct::logger::error("[Motion_APS] Invalid size for motion io");
			nvs::set_background_color(ui.toolButton_EMXA_CY1_SVON, motion_io[Motion_APS::SVON] ? Qt::green : Qt::red);
			nvs::set_background_color(ui.toolButton_conveyorStatus, motion_io[Motion_APS::SVON] ? Qt::green : Qt::red);
		}

		auto optional_EMXA_DIs =  MotionController::instance().get_all_DI(_motionID, 0);
		if (optional_EMXA_DIs.has_value()) {
			auto EMXA_DIs = optional_EMXA_DIs.value();

			for (int i = 0; i < EMXA_DIs.size(); i++) {
				//ct::logger::info("EMXA DI%d: %d", i, (EMXA_DIs[i])? 1:0);
				
				if (i == 5) continue; // X105 Emergency Button

				auto button = findChild<QToolButton*>(QString("toolButton_EMXA_DI%1").arg(i));
				if (!button) continue;

				bool state = EMXA_DIs[i];
				if (i == 5 || i == 6 || i == 7) state = !state;
				if (i == 6) nvs::set_background_color(ui.toolButton_drivePower, state ? Qt::green : Qt::red);
				
				nvs::set_background_color(button, state ? Qt::green : Qt::red);

				if (i == 8) SystemData::instance()._Entry_Sensor = EMXA_DIs[i];
				if (i == 18) SystemData::instance()._Downstream_Ready = EMXA_DIs[i];
				if (i == 19) SystemData::instance()._Upstream_Ready = EMXA_DIs[i];

				if (i == 10) {
					//SystemData::instance()._PCB_Available = EMXA_DIs[i];
					SystemData::instance()._Exit_Sensor = EMXA_DIs[i];
				}

			}
		}

		auto optional_EMXB_DIs =  MotionController::instance().get_all_DI(_motionID, 1);
		if (optional_EMXB_DIs.has_value()) {
			auto EMXB_DIs = optional_EMXB_DIs.value();

			for (int i = 0; i < EMXB_DIs.size(); i++) {
				
				auto button = findChild<QToolButton*>(QString("toolButton_EMXB_DI%1").arg(i));
				if (!button) continue;
				nvs::set_background_color(button, EMXB_DIs[i] ? Qt::green : Qt::red);

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

		auto optional_EMXB_DOs = MotionController::instance().get_all_DO(_motionID, 1);
		if (optional_EMXB_DOs.has_value()) {
			auto EMXB_DOs = optional_EMXB_DOs.value();

			for (int i = 0; i < EMXB_DOs.size(); i++) {

				auto button = findChild<QToolButton*>(QString("toolButton_EMXB_DO%1").arg(i));
				if (!button) continue;
				nvs::set_background_color(button, EMXB_DOs[i] ? Qt::green : Qt::red);

				if (i == 2) nvs::set_background_color(ui.toolButton_clamper, EMXB_DOs[i] ? Qt::green : Qt::red);
				if (i == 4) nvs::set_background_color(ui.toolButton_downstreamOutput, EMXB_DOs[i] ? Qt::green : Qt::red);
				if (i == 5) nvs::set_background_color(ui.toolButton_upstreamOutput, EMXB_DOs[i] ? Qt::green : Qt::red);

			}
		}

		bool anyEmergencyActive = EMG_X_Axis || EMG_Y_Axis || EMG_Z_Axis;
		nvs::set_background_color(ui.toolButton_EMXA_DI5, anyEmergencyActive ? Qt::green : Qt::red);

		nvs::set_background_color(ui.toolButton_upstreamReady, SystemData::instance()._Upstream_Ready ? Qt::green : Qt::red);
		nvs::set_background_color(ui.toolButton_downstreamReady, SystemData::instance()._Downstream_Ready ? Qt::green : Qt::red);
		nvs::set_background_color(ui.toolButton_machineReady, SystemData::instance()._Machine_Ready ? Qt::green : Qt::red);

		int loadingDirection = ui.comboBox_loadingDirection->currentIndex();
		MachineSMEMAManager::instance().setLoadingDirection(loadingDirection);

		ui.lineEdit_totalBoardInspection->setText(QString::number(SystemData::instance()._BoardEntryQty));

	});
	_motionTimer->start(300);

	_smemaTimer = new QTimer(this);
	connect(_smemaTimer, &QTimer::timeout, this, []() {
		MachineSMEMAManager::instance().update();
	});
	_smemaTimer->start(20);

	connect(&MachineSMEMAManager::instance(), &MachineSMEMAManager::triggerStartProduction, this, &VisionApp::startProduction);


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

	for (int i = 0; i < 14; ++i) {
		auto button = findChild<QToolButton*>(QString("toolButton_EMXB_DO%1").arg(i));
		if (!button) continue;

		nvs::set_background_color(button, Qt::red);
		auto o =  MotionController::instance().get_DO(_motionID, 1, i);
		if (o.has_value()) nvs::set_background_color(button, o.value() ? Qt::green : Qt::red);

		connect(button, &QToolButton::clicked, this, [=]() {
			ct::logger::info("[Motion] Pressing EMX(B) DO%d", i);
			auto o =  MotionController::instance().get_DO(_motionID, 1, i);
			if (!o.has_value()) { showMsg("Invalid Digital Parameter"); return; }
			auto new_output = !o.value();
			 MotionController::instance().set_DO(_motionID, 1, i, new_output);
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

	connect(ui.toolButton_toggleLane1Left, &QToolButton::clicked, this, [=]() {
		ct::logger::info("[Motion] Toggle lane left");
		if (ui.toolButton_toggleLane1Right->isChecked()) {
			QSignalBlocker sb(ui.toolButton_toggleLane1Right);
			ui.toolButton_toggleLane1Right->setChecked(false);
			MotionController::instance().stop_move(_motionID, (int)Axis::CY1);
			os_tool::doNothing(100);
		}
		if (ui.toolButton_toggleLane1Left->isChecked()) emit signalContinuousMoveConveyor(false);
		else  MotionController::instance().stop_move(_motionID, (int)Axis::CY1);
	});

	connect(ui.toolButton_toggleLane1Right, &QToolButton::clicked, this, [=]() {
		ct::logger::info("[Motion] Toggle lane right");
		if (ui.toolButton_toggleLane1Left->isChecked()) {
			QSignalBlocker sb(ui.toolButton_toggleLane1Left);
			ui.toolButton_toggleLane1Left->setChecked(false);
			MotionController::instance().stop_move(_motionID, (int)Axis::CY1);
			os_tool::doNothing(100);
		}
		if (ui.toolButton_toggleLane1Right->isChecked()) emit signalContinuousMoveConveyor(true);
		else  MotionController::instance().stop_move(_motionID, (int)Axis::CY1);
	});

	connect(ui.toolButton_loadToSensor, &QToolButton::clicked, this, [=]() {
			
		auto pos = ui.comboBox_loadingPosition->currentText();
		ct::logger::info("[Motion] Load to sensor: %s", pos.toStdString().c_str());

		int timeout_ms = SystemData::instance()._timeoutLoad2Sensor;
		if (pos == "Entry") emit signalLoadToSensor(SensorIndex::START, true, timeout_ms);
		else if (pos == "Exit") emit signalLoadToSensor(SensorIndex::EXIT, true, timeout_ms);
		else if (pos == "Pos1") emit signalLoadToSensor(SensorIndex::RIGHT, true, timeout_ms);
		else if (pos == "Pos2") emit signalLoadToSensor(SensorIndex::LEFT, true, timeout_ms);
	});
	
	connect(ui.toolButton_setRailWidth1, &QToolButton::clicked, this, [=]() {
		ct::logger::info("[Motion] Set rail width 1");
		double step = ui.lineEdit_railWidth1->text().toDouble();
		emit signalSetRailWidth(step);
	});

	connect(ui.toolButton_homeRail1, &QToolButton::clicked, this, [=]() {
		ct::logger::info("[Motion] Home rail 1");
		//check if there's board in the conveyor
		auto status = MachineController::instance().getSensorStatus();
		if (MachineController::instance().isAllSensorOff(status)) {
			emit homeRail();
		}
		else {
			MachineController::instance().notifyError(MachineError::RAIL_UNSAFE_TO_MOVE_BOARD_PRESENT);
		}
		// OLD_BRANCH_REVERT_DISABLED_BEGIN
		// Unconditional rail home disabled; restore old UI-side sensor check.
		//emit homeRail();
		// OLD_BRANCH_REVERT_DISABLED_END
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

	connect(ui.toolButton_servo_lane1, &QToolButton::clicked, this, [=]() {
		ct::logger::info("[Motion] Toggle servo");
		auto optional_io =  MotionController::instance().get_motion_io_status(_motionID, (int)Axis::CY1);

		if (!optional_io.has_value()) {
			showMsg("Failed to toggle servo");
			return;
		}

		auto io = optional_io.value();
		auto state = !io[(int)Motion_APS::IO::SVON];

		auto o =  MotionController::instance().set_servo(_motionID, 0, (int)Axis::CY1, state);
		AuditLog::instance().log(QStringLiteral("SERVO_TOGGLE"), QStringLiteral("CY1"), o ? (state ? QStringLiteral("ON") : QStringLiteral("OFF")) : QStringLiteral("FAILED"));
	});

	connect(ui.toolButton_servo_lane2, &QToolButton::clicked, this, [=]() {
		ct::logger::info("[Motion] Toggle servo");
		auto optional_io =  MotionController::instance().get_motion_io_status(_motionID, (int)Axis::CY2);

		if (!optional_io.has_value()) {
			showMsg("Failed to toggle servo");
			return;
		}

		auto io = optional_io.value();
		auto state = !io[(int)Motion_APS::IO::SVON];

		auto o =  MotionController::instance().set_servo(_motionID, 0, (int)Axis::CY2, state);
	});

	connect(ui.toolButton_servo_rail1, &QToolButton::clicked, this, [=]() {
		ct::logger::info("[Motion] Toggle servo");
		auto optional_io =  MotionController::instance().get_motion_io_status(_motionID, (int)Axis::FR1);

		if (!optional_io.has_value()) {
			showMsg("Failed to toggle servo");
			return;
		}

		auto io = optional_io.value();
		auto state = !io[(int)Motion_APS::IO::SVON];

		auto o =  MotionController::instance().set_servo(_motionID, 0, (int)Axis::FR1, state);
		AuditLog::instance().log(QStringLiteral("SERVO_TOGGLE"), QStringLiteral("FR1"), o ? (state ? QStringLiteral("ON") : QStringLiteral("OFF")) : QStringLiteral("FAILED"));

		if (o && state) MachineController::instance().notifyEvent(MachineEvent::RAIL_SERVO_ON);
	});

	connect(ui.toolButton_servo_rail2, &QToolButton::clicked, this, [=]() {
		ct::logger::info("[Motion] Toggle servo");
		auto optional_io =  MotionController::instance().get_motion_io_status(_motionID, (int)Axis::FR2);

		if (!optional_io.has_value()) {
			showMsg("Failed to toggle servo");
			return;
		}

		auto io = optional_io.value();
		auto state = !io[(int)Motion_APS::IO::SVON];

		auto o =  MotionController::instance().set_servo(_motionID, 0, (int)Axis::FR2, state);
	});

	connect(ui.toolButton_servo_rail3, &QToolButton::clicked, this, [=]() {
		ct::logger::info("[Motion] Toggle servo");
		auto optional_io =  MotionController::instance().get_motion_io_status(_motionID, (int)Axis::FR3);

		if (!optional_io.has_value()) {
			showMsg("Failed to toggle servo");
			return;
		}

		auto io = optional_io.value();
		auto state = !io[(int)Motion_APS::IO::SVON];

		auto o =  MotionController::instance().set_servo(_motionID, 0, (int)Axis::FR3, state);
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

	connect(ui.toolButton_updateVelocity_cy1, &QToolButton::clicked, this, [=]() {
		ct::logger::info("[Motion] Update velocity lane1");
		auto velocity = ui.lineEdit_cy1_velocity->text().toInt();
		auto acel = ui.lineEdit_cy1_acceleration->text().toInt();
		auto axis = (int)Axis::CY1;
		AuditLog::instance().log(QStringLiteral("VELOCITY_UPDATE"), QStringLiteral("CY1 vel=%1 acc=%2").arg(velocity).arg(acel));

		auto ret =  MotionController::instance().set_move_max_velocity(_motionID, axis, velocity);
		ret &=  MotionController::instance().set_move_acceleration(_motionID, axis, acel);
		ret &= MotionController::instance().set_move_deceleration(_motionID, axis, acel);

		if (ret) {
			auto& m = _motions[_motionID].axes[axis];
			m.move_max_velocity = velocity;
			m.move_acceleration = acel;
			m.move_deceleration = acel;
			_jobThread.setConveyorSpeed(velocity);
			saveMotion();
			showMsg("Updated velocity");
		}
		else {
			showMsg("Failed to update velocity");
		}
	});

	connect(ui.toolButton_updateVelocity_fr1, &QToolButton::clicked, this, [=]() {
		ct::logger::info("[Motion] Update velocity rail 1");
		auto velocity = ui.lineEdit_fr1_velocity->text().toInt();
		auto acel = ui.lineEdit_fr1_acceleration->text().toInt();
		auto axis = (int)Axis::FR1;
		AuditLog::instance().log(QStringLiteral("VELOCITY_UPDATE"), QStringLiteral("FR1 vel=%1 acc=%2").arg(velocity).arg(acel));

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
