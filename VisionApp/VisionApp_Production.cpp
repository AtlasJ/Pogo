#include "VisionApp.h"
#include "MotionController.h"
#include "CAMManager.h"
#include "ProfilerManager.h"
#include "LSCManager.h"
#include "MachineController.h"
#include "MachineSMEMAManager.h"
#include "uidGenerator.h"
#include "AuditLog.h"

void VisionApp::initProductionUI() {
	//NOTE: for hardware side, user can only turn on in this production page
	// Only config can be toggle on off


	//This section must always be on
	connect(ui.toolButton_cameraStatus, &QToolButton::clicked, this, [=]() {
		//CAMManager::instance().connect(_camID, );
	});

	connect(ui.toolButton_3dProfilerStatus, &QToolButton::clicked, this, [=]() {
		//ProfilerManager::instance().connect()
	});

	connect(ui.toolButton_lscStatus, &QToolButton::clicked, this, [=]() {
		if (!LSCManager::instance().isConnected()) LSCManager::instance().connect();
		auto connected = LSCManager::instance().isConnected();
		nvs::set_background_color(ui.toolButton_lscStatus, connected ? Qt::green : Qt::red);
	});

	connect(ui.toolButton_drivePower, &QToolButton::clicked, this, [=]() {
		auto state = MotionController::instance().set_DO(_motionID, 0, 6, true);
		AuditLog::instance().log(QStringLiteral("DO_TOGGLE"), QStringLiteral("drive power"), state ? QStringLiteral("ON") : QStringLiteral("FAILED"));
	});

	connect(ui.toolButton_gantryStatus, &QToolButton::clicked, this, [=]() {
		MotionController::instance().set_servo(_motionID, 0, (int)Axis::X, true);
		MotionController::instance().set_servo(_motionID, 0, (int)Axis::Y, true);
		MotionController::instance().set_servo(_motionID, 0, (int)Axis::Z, true);
	});

	connect(ui.toolButton_railStatus, &QToolButton::clicked, this, [=]() {
		MotionController::instance().set_servo(_motionID, 0, (int)Axis::FR1, true);
	});

	connect(ui.toolButton_conveyorStatus, &QToolButton::clicked, this, [=]() {
		MotionController::instance().set_servo(_motionID, 0, (int)Axis::CY1, true);
	});


	//This section can be on or off
	connect(ui.toolButton_enableFiducial, &QToolButton::clicked, this, [=]() {
		enableFiducial(!ui.checkBox_enableFiducial->isChecked());
	});

	connect(ui.toolButton_enableSaveInspImages, &QToolButton::clicked, this, [=]() {
		enableSaveInspectionImage(!ui.checkBox_EnableSaveInspectionImage->isChecked());
	});

	connect(ui.toolButton_startProduction, &QToolButton::clicked, this, [=]() {
		
		if (SystemData::instance()._enableSMEMA) return;
		startProduction();
	});

	connect(ui.toolButton_autoStartProduction, &QToolButton::clicked, this, [=](bool checked) {

		QString machineState = ui.toolButton_machineState->text();
		if (machineState != "Initialized" && checked == true)
		{
			ui.toolButton_autoStartProduction->setChecked(false);
			showMsg("Please wait for the machine to initialize before starting auto production.");
			return;
		}

		//if (SystemData::instance()._IsBoardEntry)
		//{
		//	ui.toolButton_autoStartProduction->setChecked(true);
		//	showMsg("Please wait until the board has fully left the machine before stopping auto production.");
		//	return;
		//}

		if (checked)
		{
			qDebug() << "Auto Start ON";
			SystemData::instance()._enableSMEMA = true;
			ui.toolButton_startProduction->setEnabled(false); 
			
			enableUIControl(false);
		}
		else
		{
			qDebug() << "Auto Start OFF";
			SystemData::instance()._enableSMEMA = false;
			ui.toolButton_startProduction->setEnabled(true);

			enableUIControl(true);
		}

	});

	connect(ui.toolButton_stopProduction, &QToolButton::clicked, this, [=]() {
		AuditLog::instance().log(QStringLiteral("PRODUCTION_STOP"));
		stopRun();
		vs_stopElapseTimer();

	});

	connect(ui.toolButton_resetProduction, &QToolButton::clicked, this, [=]() {
		//handle error and alarm here
		AuditLog::instance().log(QStringLiteral("RESET_ALARM"));
		MachineController::instance().resetAlarm();

		SystemData::instance()._Inspection_Done = false;
		SystemData::instance()._Machine_Ready = true;
		SystemData::instance()._IsBoardEntry = false;
		MachineSMEMAManager::instance().changeState(SmemaState::IDLE);

	});

	connect(ui.toolButton_unloadBoard, &QToolButton::clicked, this, [=]() {

		if (SystemData::instance()._Unloading_Board) return;

		SystemData::instance()._Unloading_Board = true;
		
		auto& sys = SystemData::instance();
		if (sys._enableSMEMA) {
			MachineSMEMAManager::instance().changeState(SmemaState::BYPASS_INSPECTION);
		}

		if (!SystemData::instance()._Exit_Sensor || !SystemData::instance()._Entry_Sensor) {

			unloadBoard();
		}
		else if (sys._enableSMEMA && (SystemData::instance()._Exit_Sensor || SystemData::instance()._Entry_Sensor)) {
			
			vs_stopElapseTimer();

			switchToMainRecipe();
			sys._Production_Running = false;
			sys._Inspection_Done = true;
			sys._Unloading_Board = false;

		}

		
	});
}

void VisionApp::startAcquisition()
{
	_processType = ProcessType::IMAGE_COLLECTION;

	auto sensor = MachineController::instance().getSensorStatus();
	if (MachineController::instance().isAllSensorOff(sensor)) {
		MachineController::instance().notifyError(MachineError::BOARD_NOT_FOUND);
		return;
	}

	int loadingDirection = ui.comboBox_loadingDirection->currentIndex();
	SystemData::instance()._subRecipeIndex = 0;
	SystemData::instance()._offlineRun = false;
	switchToMainRecipe();
	emit signalLoadingDirection(loadingDirection);
	emit signalLoadToPosition(SystemData::instance()._subRecipeIndex);

	if (hasSubrecipe()) {
		_subrecipesToRun.clear();
		_subrecipesToRun.insert(1);
	}

	MotionController::instance().set_DO(_motionID, 0, (int)DOA::SAFETY_DOOR_LOCK, true);
}

void VisionApp::startProduction()
{
	auto state = MachineController::instance().getMachineState();
	if (state == MachineState::NOT_READY) {
		showMsg("Please initialize machine to proceed!");
		return;
	}
	else if (state == MachineState::S_ERROR) {
		showMsg("Unable to start production when machine is in ERROR state!");
		return;
	}

	// Don't touch for SMEMA USED
	SystemData::instance()._Production_Running = true;
	SystemData::instance()._Inspection_Done = false;
	// Don't touch for SMEMA USED

	AuditLog::instance().log(QStringLiteral("PRODUCTION_START"), Common::Directory::CurrentRecipe);

	ui.lineEdit_inspectionTimeMain->clear();
	ui.lineEdit_inspectionTimeSub->clear();

	_processType = ProcessType::PRODUCTION;

	uidGenerator uidGen;
	_currentProductionID = uidGen.id().c_str();

	if (!SystemData::instance()._enableSMEMA) {
		auto sensor = MachineController::instance().getSensorStatus();
		if (MachineController::instance().isAllSensorOff(sensor)) {
			MachineController::instance().notifyError(MachineError::BOARD_NOT_FOUND);
			return;
		}
	}

	if (_enableBarcode)
	{
		bool needExternal = false;

		if (_barcodeInfos.size() > 0 && _barcodeInfos[0].registration_method == 2) // 2=External
		{
			ui.lineEdit_barcodeID->clear();
			needExternal = true;
		}

		if (_barcodeInfos.size() > 1 && _barcodeInfos[1].registration_method == 2) // 2=External
		{
			ui.lineEdit_barcodeID2->clear();
			needExternal = true;
		}

		if (needExternal)
		{
			// Endpoints are already set at startup (connectJobThread) and never change;
			// re-setting them here would race the job thread reading them.
			ct::logger::info("[Barcode] Production start: triggering external readers");
			emit signalTriggerSRX();
		}
		else
		{
			ct::logger::info("[Barcode] Production start: no slot set to External, readers not triggered");
		}
	}

	switchToMainRecipe();

	if (hasSubrecipe()) {
		_subrecipesToRun.clear();
		_subrecipesToRun.insert(1);
	}

	MotionController::instance().set_DO(_motionID, 0, (int)DOA::SAFETY_DOOR_LOCK, true);

	int loadingDirection = ui.comboBox_loadingDirection->currentIndex();
	SystemData::instance()._subRecipeIndex = 0;
	SystemData::instance()._offlineRun = false;
	emit signalLoadingDirection(loadingDirection);
	emit signalLoadToPosition(SystemData::instance()._subRecipeIndex);

	vs_startElapseTimer();
}

void VisionApp::startProductionS()
{
	auto state = MachineController::instance().getMachineState();
	if (state == MachineState::NOT_READY) {
		showMsg("Please initialize machine to proceed!");
		return;
	}
	else if (state == MachineState::S_ERROR) {
		showMsg("Unable to start production when machine is in ERROR state!");
		return;
	}

	ui.lineEdit_inspectionTimeMain->clear();


	_processType = ProcessType::PRODUCTION;

	uidGenerator uidGen;
	_currentProductionID = uidGen.id().c_str();

	//if (!SystemData::instance()._enableSMEMA) {
	//	auto sensor = MachineController::instance().getSensorStatus();
	//	if (MachineController::instance().isAllSensorOff(sensor)) {
	//		MachineController::instance().notifyError(MachineError::BOARD_NOT_FOUND);
	//		return;
	//	}
	//}

	//if (_enableBarcode)
	//{
	//	bool needExternal = false;

	//	if (_barcodeInfos.size() > 0 && _barcodeInfos[0].registration_method == 2) // 2=External
	//	{
	//		ui.lineEdit_barcodeID->clear();
	//		needExternal = true;
	//	}

	//	if (_barcodeInfos.size() > 1 && _barcodeInfos[1].registration_method == 2) // 2=External
	//	{
	//		ui.lineEdit_barcodeID2->clear();
	//		needExternal = true;
	//	}

	//	if (needExternal)
	//	{
	//		_jobThread.setServerHostAddress(m_barcodeIp, m_barcodePort);
	//		_jobThread.triggerSRX();

	//	}
	//}


	//MotionController::instance().set_DO(_motionID, 0, (int)DOA::SAFETY_DOOR_LOCK, true);


	SystemData::instance()._offlineRun = false;
	runProdS();

	vs_startElapseTimer();
}


void VisionApp::boardInPosition(int pos) {
	ct::logger::info("Board in position");

	if (SystemData::instance()._enableSMEMA) {
		SystemData::instance()._Machine_Ready = false;
		SystemData::instance()._IsBoardEntry = true;
	}


	if (_processType == ProcessType::IMAGE_COLLECTION) {
		collectImages();
		ct::logger::info("pew pew");
	}
	else if (_processType == ProcessType::PRODUCTION) {
		ct::logger::info("[boardInPosition]Start InspectionThread in Production Mode");
		startInspectionThread();
		inspect2D3D();
	}
}
void VisionApp::runProdS()
{
	if (_processType == ProcessType::IMAGE_COLLECTION) {
		collectImages();
		ct::logger::info("pew pew");
	}
	else if (_processType == ProcessType::PRODUCTION) {
		ct::logger::info("[boardInPosition]Start InspectionThread in Production Mode");
		startInspectionThread();
		inspect2D3D();
	}
}


void VisionApp::unloadBoard()
{
	int loadingDirection = ui.comboBox_loadingDirection->currentIndex();
	emit signalLoadingDirection(loadingDirection);
	emit signalUnloadBoard();
}

void VisionApp::clearSubRecipe()
{
	_subrecipesToRun.clear();
}

void VisionApp::addLogLine(const QString& line)
{
	ui.toolButton_machineState->setText(line);

	// Add new line
	m_logStatus.append(line);

	// Trim to last 10 lines
	if (m_logStatus.size() > 10)
		m_logStatus.removeFirst();

	// Update the QTextEdit
	ui.textEdit_statusLog->setPlainText(m_logStatus.join("\n"));

	// Scroll to bottom
	ui.textEdit_statusLog->moveCursor(QTextCursor::End);
}

void VisionApp::clearErrorLogs()
{
	for (int i = m_logStatus.size() - 1; i >= 0; --i) {
		if (m_logStatus[i].contains("Error: "))
			m_logStatus.removeAt(i);
	}

	// Update the QTextEdit
	ui.textEdit_statusLog->setPlainText(m_logStatus.join("\n"));

	// Scroll to bottom
	ui.textEdit_statusLog->moveCursor(QTextCursor::End);
}

void VisionApp::setXAxisVelocity()
{
	// set x-axis velocity
	auto velocity = ui.lineEdit_x_velocity->text().toInt();
	auto velocity3d = ui.lineEdit_x_velocity3d->text().toInt();
	auto acel = ui.lineEdit_x_acceleration->text().toInt();
	auto axis = (int)Axis::X;

	auto ret = MotionController::instance().set_move_max_velocity(_motionID, axis, velocity);
	ret &= MotionController::instance().set_move_acceleration(_motionID, axis, acel);
	ret &= MotionController::instance().set_move_deceleration(_motionID, axis, acel);

	if (ret) {
		_jobThread.setXSpeed(velocity, velocity3d);
		_jobThread.setXDecel(acel);
		ct::logger::info("[Motion] Updated x velocity");
	}
	else {
		ct::logger::warn("[Motion] Failed to update x velocity");
	}
}

void VisionApp::clearLiveDefectTableWidget()
{
}

void VisionApp::addDefectToTableWidget()
{
}


