#include "VisionApp.h"
#include "ScaleManager.h"
#include "ImagePathManager.h"
#include "uidGenerator.h"
#include "MachineController.h"
#include "MotionController.h"

// for 1 time trigger set X Velocity
bool g_isStartMachine = true;

void VisionApp::connectMachineController()
{
	//return;
	if (!MotionController::instance().is_init(_motionID)) {
		ct::logger::info("Motion not init, machine controller will not be activated.");
		//Not activated means run() never executes, so the red tower timer is never
		//created and the state machine must not accept events from JobThread.
		MachineController::instance().enable(false);
		nvs::set_background_color(ui.toolButton_machineState, "#1565C0");
		ui.toolButton_machineState->setText("Offline");
		return;
	}

	qRegisterMetaType<MachineEvent>("MachineEvent");
	qRegisterMetaType<MachineState>("MachineState");
	qRegisterMetaType<MachineWarning>("MachineWarning");
	qRegisterMetaType<MachineError>("MachineError");
	qRegisterMetaType<DOA>("DOA");

	MachineController::instance();

	_stateTimer = new QTimer();
	QObject::connect(_stateTimer, &QTimer::timeout, [&]() {
		nvs::set_background_color(ui.toolButton_machineState, Qt::gray);
		os_tool::doNothing(1000);
		nvs::set_background_color(ui.toolButton_machineState, _stateColor);
		ct::logger::info("Inside state timer");
		});

	QObject::connect(&MachineController::instance(), &MachineController::signalTowerLightOn, this, [=](DOA e) {

		nvs::set_background_color(ui.toolButton_EMXA_DO3, Qt::red);
		nvs::set_background_color(ui.toolButton_EMXA_DO4, Qt::red);
		nvs::set_background_color(ui.toolButton_EMXA_DO5, Qt::red);

		switch (e)
		{
		case DOA::GREEN_TOWER_LIGHT:
			nvs::set_background_color(ui.toolButton_EMXA_DO5, Qt::green);
			break;
		case DOA::AMBER_TOWER_LIGHT:
			nvs::set_background_color(ui.toolButton_EMXA_DO4, Qt::green);
			break;
		case DOA::RED_TOWER_LIGHT:
			nvs::set_background_color(ui.toolButton_EMXA_DO3, Qt::green);
			break;
		default:
			break;
		}
		});

	QObject::connect(&MachineController::instance(), &MachineController::signalMachineEvent, this, [=](MachineEvent e) {
		switch (e)
		{
		case MachineEvent::START_BTN:
			//the physical start always lands the operator on the production page first -
			//a run started while another page is open would otherwise run out of sight
			if (ui.stackedWidgetViewSelection->currentIndex() != 6 || !ui.page_23->isVisible())
				showProductionPage();
			startProduction();
			break;
		case MachineEvent::STOP_BTN:
			stopRun();
			break;
		case MachineEvent::RESET_BTN:
			/*
			* The recovery itself is handled inside the machine controller. This event is emitted
			* once the attempt has RETURNED - for a panel press and an on-screen one alike - so it
			* is the honest edge on which to stop showing the button as busy. Whether the machine
			* actually came back is reported separately by signalMachineState.
			*/
			_resetBusy = false;
			updateResetButtonState();
			break;
		default:
			break;
		}
		});

	QObject::connect(&MachineController::instance(), &MachineController::signalMachineState, this, [=](MachineState state) {
		ct::logger::info("Machine State: %d", (int)state);

		/*
		* Mirror the panel reset LED, which blinks exactly while the state is S_ERROR - the blink
		* lambda in MachineController::run() drives Y102 alongside the tower bit, in debug mode
		* too (only the TOWER colour swaps to amber there). Keyed off the CONDITION rather than
		* those writes on purpose: the lambda runs on the controller thread and widgets are
		* main-thread only, so following it would mean a queued signal per blink edge.
		*
		* Edge-driven for free: setMachineState() returns early on an unchanged state, and while
		* S_ERROR still has errors outstanding, so this arrives once going in and once coming out.
		*/
		_machineInError = (state == MachineState::S_ERROR);
		updateResetButtonState();

		switch (state)
		{
		case MachineState::NOT_READY:
			nvs::set_background_color(ui.toolButton_machineState, QColor("#F2B705"));
			ui.toolButton_machineState->setText("Uninitialized");
			clearErrorLogs();
			//_stateTimer->start(1000);
			break;
		case MachineState::READY:
			_stateTimer->stop();
			nvs::set_background_color(ui.toolButton_machineState, QColor("#2E7D32"));
			ui.toolButton_machineState->setText("Initialized");
			clearErrorLogs();

			if (g_isStartMachine) {
				setXAxisVelocity();
				g_isStartMachine = false;
			}
			
			break;
		case MachineState::IDLE:
			_stateTimer->stop();
			nvs::set_background_color(ui.toolButton_machineState, QColor("#1565C0"));
			ui.toolButton_machineState->setText("Machine Idle");
			break;
		case MachineState::JOGGING:
			ui.toolButton_machineState->setText("Jogging");
			_stateTimer->stop();
			break;
		case MachineState::S_ERROR:
			//ui.toolButton_machineState->setText("Machine Error");
			nvs::set_background_color(ui.toolButton_machineState, QColor("#C62828"));
			stopRun();
			//_stateTimer->start(1000);
			break;
		case MachineState::MACHINE_HOMING:
			nvs::set_background_color(ui.toolButton_machineState, QColor("#F2B705"));
			ui.toolButton_machineState->setText("Homing");
			clearErrorLogs();
			break;
		default:
			break;
		}
		});

	QObject::connect(&MachineController::instance(), &MachineController::signalMachineWarning, this, [=](MachineWarning w) {
		nvs::set_background_color(ui.toolButton_machineState, "#F2B705");

		switch (w)
		{
		case MachineWarning::X_SOFT_LIMIT_HIT:
			addLogLine("Warning: X Soft Limit Hit");
			showMsg("X Soft Limit Hit");
			break;
		case MachineWarning::Y_SOFT_LIMIT_HIT:
			addLogLine("Warning: Y Soft Limit Hit");
			showMsg("Y Soft Limit Hit");
			break;
		case MachineWarning::Z_SOFT_LIMIT_HIT:
			addLogLine("Warning: Z Soft Limit Hit");
			showMsg("Z Soft Limit Hit");
			break;
		default:
			break;
		}
		});

	QObject::connect(&MachineController::instance(), &MachineController::signalMachineError, this, [=](MachineError e) {
		switch (e)
		{
		case MachineError::ESTOP_PRESSED:
			addLogLine("Error: E-Stop Triggered");
			break;
		case MachineError::ESTOP_RELAY_FAULT:
			addLogLine("Error: E-Stop Safety Relay Fault");
			break;
		case MachineError::CURTAIN_RELAY_FAULT:
			addLogLine("Error: Curtain Sensor Triggered");
			ui.toolButton_machineState->setText("Curtain Sensor Triggered");
			break;
		case MachineError::TROLLEY_GUARD_OPEN:
			addLogLine("Error: Trolley Lock Guard Open");
			break;
		case MachineError::X_SERVO_OFF:
			addLogLine("Error: X Servo OFF");
			break;
		case MachineError::X_POSITIVE_LIMIT_HIT:
			addLogLine("Error: X Positive Limit Hit");
			break;
		case MachineError::X_NEGATIVE_LIMIT_HIT:
			addLogLine("Error: X Negative Limit Hit");
			break;
		case MachineError::X_DRIVER_ALARM:
			addLogLine("Error: X Driver Alarm");
			break;
		case MachineError::Y_SERVO_OFF:
			addLogLine("Error: Y Servo OFF");
			break;
		case MachineError::Y_POSITIVE_LIMIT_HIT:
			addLogLine("Error: Y Positive Limit Hit");
			break;
		case MachineError::Y_NEGATIVE_LIMIT_HIT:
			addLogLine("Error: Y Negative Limit Hit");
			break;
		case MachineError::Y_DRIVER_ALARM:
			addLogLine("Error: Y Driver Alarm");
			break;
		case MachineError::Z_SERVO_OFF:
			addLogLine("Error: Z Servo OFF");
			break;
		case MachineError::Z_POSITIVE_LIMIT_HIT:
			addLogLine("Error: Z Positive Limit Hit");
			break;
		case MachineError::Z_NEGATIVE_LIMIT_HIT:
			addLogLine("Error: Z Negative Limit Hit");
			break;
		case MachineError::Z_DRIVER_ALARM:
			addLogLine("Error: Z Driver Alarm");
			break;
		case MachineError::INITIALIZATION_TIMEOUT:
			addLogLine("Error: Initialization Timeout");
			break;
		case MachineError::X_HOMING_TIMEOUT:
			addLogLine("Error: Homing X Timeout");
			break;
		case MachineError::X_MOVE_TIMEOUT:
			addLogLine("Error: Move X Timeout");
			break;
		case MachineError::Y_HOMING_TIMEOUT:
			addLogLine("Error: Homing Y Timeout");
			break;
		case MachineError::Y_MOVE_TIMEOUT:
			addLogLine("Error: Move Y Timeout");
			break;
		case MachineError::Z_HOMING_TIMEOUT:
			addLogLine("Error: Homing Z Timeout");
			break;
		case MachineError::Z_MOVE_TIMEOUT:
			addLogLine("Error: Move Z Timeout");
			break;
		default:
			break;
		}
		});

	QObject::connect(&MachineController::instance(), &MachineController::signalPromptMsg, this, [=](QString msg) {
		showMsg(msg);
		});

	QObject::connect(&MachineController::instance(), &MachineController::signalLogTime, this, [=](QString key, long long ms) {
		if (key == "Inspection") ui.lineEdit_inspection_time->setText(QString::number(ms));
		else if (key == "Locator") ui.lineEdit_locator_time->setText(QString::number(ms));
		else if (key == "AI") ui.lineEdit_AI_time->setText(QString::number(ms));
		else if (key == "Algo") ui.lineEdit_algo_time->setText(QString::number(ms));

		else if (key == "Jog 2D") ui.lineEdit_jog2d_time->setText(QString::number(ms));
		else if (key == "Jog 3D") ui.lineEdit_jog3d_time->setText(QString::number(ms));
		else if (key == "Snap") ui.lineEdit_snap_time->setText(QString::number(ms));
		else if (key == "Snap + Light") ui.lineEdit_snaplight_time->setText(QString::number(ms));

		else if (key == "View") ui.lineEdit_view_time->setText(QString::number(ms));
		else if (key == "Scan") ui.lineEdit_scan_time->setText(QString::number(ms));
		else if (key == "Fiducial") ui.lineEdit_fiducial_time->setText(QString::number(ms));

		else if (key == "2D Acquisition") ui.lineEdit_2DAcquisition_time->setText(QString::number(ms));
		else if (key == "3D Acquisition") ui.lineEdit_3DAcquisition_time->setText(QString::number(ms));
		});

	connect(ui.toolButton_machineState, &QToolButton::clicked, this, [=]() {
		auto state = MachineController::instance().getMachineState();
		if (state == MachineState::READY || state == MachineState::IDLE) {
			//do nothing
		}
		else if (state == MachineState::S_ERROR || state == MachineState::WARNING) {
			/*
			* Full recovery, same as the panel button - not the bare acknowledge this used to do.
			* resetAlarm() refuses while any error is still latched, and after an e-stop or a
			* curtain trip the drives stay alarmed until recoverDrives() clears them, so on its
			* own this button could never bring the machine back. It matters that it works:
			* setUiLockedToProduction() deliberately keeps this button enabled in production mode.
			*/
			MachineController::instance().requestReset();
		}
		else if (state == MachineState::NOT_READY) {
			emit homeXYZ();
		}
	});

	MachineController::instance().moveToThread(&MachineController::instance());
	MachineController::instance().start(QThread::HighPriority);
}