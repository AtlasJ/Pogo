#include "VisionApp.h"
#include "MotionController.h"
#include "CAMManager.h"
#include "ProfilerManager.h"
#include "LSCManager.h"
#include "MachineController.h"
#include "uidGenerator.h"
#include "AuditLog.h"
#include "AlgoManager.h"

void VisionApp::initProductionUI() {
	//NOTE: for hardware side, user can only turn on in this production page
	// Only config can be toggle on off

	//table sizing comes from the .ui form; only the column behaviour is set here
	ui.tableWidget_prodStatus->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);

	//pitch runs: algo results can finish after acquisition; close the progress
	//bar when the inspection queue has fully drained (increments can drift when
	//a unit produces no result, so this is the reliable closer)
	auto* inspDrainTimer = new QTimer(this);
	connect(inspDrainTimer, &QTimer::timeout, this, [=]() {
		if (InspectionThread::instance().isIdle() && !AlgoManager::instance().isBusy()) {
			inspDrainTimer->stop();
			progressBarRelease();
		}
	});
	connect(&_jobThread, &JobThread::acquisitionDone, this, [=]() {
		if (_processType == ProcessType::PRODUCTION && SystemData::instance()._setupRegionPitchMode) {
			inspDrainTimer->start(500);
		}
	}, Qt::QueuedConnection);

	//columns: 0 Time | 1 XY Unit | 2 Barcode | 3 OCR | 4 Pass/Fail
	//result cell Qt::UserRole: pass bitmask (bit0 OCR, bit1 3D height)
	//result cell Qt::UserRole+1: 1 = still inspecting (animated)
	constexpr int kOcrPassBit = 1, kHeightPassBit = 2;

	//find a unit's row by the unitID stored on its time item (newest first)
	auto findUnitRow = [=](const QString& unitID) -> int {
		auto* t = ui.tableWidget_prodStatus;
		for (int r = t->rowCount() - 1; r >= 0; r--) {
			auto* it = t->item(r, 0);
			if (it && it->data(Qt::UserRole).toString() == unitID) return r;
		}
		return t->rowCount() - 1; //fallback: newest row
	};

	connect(&_jobThread, &JobThread::unitBarcode, this, [=](QString unitID, QString code) {
		if (_processType != ProcessType::PRODUCTION) return;

		auto* t = ui.tableWidget_prodStatus;
		const int row = t->rowCount();
		t->insertRow(row);

		auto* tItem = new QTableWidgetItem(QTime::currentTime().toString("hh:mm:ss"));
		tItem->setData(Qt::UserRole, unitID);
		t->setItem(row, 0, tItem);

		//"unit_3_2" -> "3, 2"
		QString xy = unitID;
		if (unitID.startsWith("unit_")) {
			auto parts = unitID.mid(5).split('_');
			if (parts.size() == 2) xy = parts[0] + ", " + parts[1];
		}
		t->setItem(row, 1, new QTableWidgetItem(xy));

		const bool codeOk = (code != "No_Barcode" && code != "Fail_to_read_barcode" && code != "ERROR");
		auto* bItem = new QTableWidgetItem(code);
		bItem->setForeground(codeOk ? QBrush(Qt::green) : QBrush(Qt::red));
		t->setItem(row, 2, bItem);

		t->setItem(row, 3, new QTableWidgetItem("-"));

		auto* rItem = new QTableWidgetItem(codeOk ? "Inspecting" : "FAIL");
		rItem->setForeground(codeOk ? QBrush(QColor(255, 191, 0)) : QBrush(Qt::red));
		rItem->setData(Qt::UserRole, 0);
		rItem->setData(Qt::UserRole + 1, codeOk ? 1 : 0);

		//PASS requires only the algos enabled in the recipe
		int required = 0;
		if (SystemData::instance()._pitchEnableBarcode) required |= kOcrPassBit;
		if (SystemData::instance()._pitchEnable3D) required |= kHeightPassBit;
		if (required == 0) required = kOcrPassBit;
		rItem->setData(Qt::UserRole + 2, required);

		t->setItem(row, 4, rItem);

		t->scrollToBottom();
	}, Qt::QueuedConnection);

	//animate the "Inspecting" cells so in-progress units are visible at a glance
	auto* inspectAnimTimer = new QTimer(this);
	connect(inspectAnimTimer, &QTimer::timeout, this, [=]() {
		static int phase = 0;
		phase = (phase + 1) % 4;
		auto* t = ui.tableWidget_prodStatus;
		for (int r = 0; r < t->rowCount(); r++) {
			auto* rItem = t->item(r, 4);
			if (rItem && rItem->data(Qt::UserRole + 1).toInt() == 1) {
				rItem->setText(QStringLiteral("Inspecting") + QString(".").repeated(phase));
			}
		}
	});
	inspectAnimTimer->start(400);

	connect(&InspectionThread::instance(), &InspectionThread::inspectionResult, this,
		[=](QString unitID, QString algo, bool pass, QString detail) {
			//pitch mode: each finished algo is a progress step
			if (SystemData::instance()._setupRegionPitchMode) incrementProgressBar();

			auto* t = ui.tableWidget_prodStatus;
			if (t->rowCount() == 0) return;
			const int row = findUnitRow(unitID);
			if (row < 0) return;

			if (algo == "OCR") {
				auto* item = new QTableWidgetItem(detail.isEmpty() ? (pass ? "PASS" : "FAIL") : detail);
				item->setForeground(pass ? QBrush(Qt::green) : QBrush(Qt::red));
				t->setItem(row, 3, item);
			}

			auto* rItem = t->item(row, 4);
			if (!rItem) return;
			if (rItem->text() == "FAIL") return; //already failed, stays failed

			if (!pass) {
				rItem->setText("FAIL");
				rItem->setForeground(QBrush(Qt::red));
				rItem->setData(Qt::UserRole + 1, 0);
				return;
			}

			int mask = rItem->data(Qt::UserRole).toInt();
			if (algo == "OCR") mask |= kOcrPassBit;
			else if (algo == "3D Height") mask |= kHeightPassBit;
			rItem->setData(Qt::UserRole, mask);

			const int required = rItem->data(Qt::UserRole + 2).toInt();
			if (required != 0 && (mask & required) == required) {
				rItem->setText("PASS");
				rItem->setForeground(QBrush(Qt::green));
				rItem->setData(Qt::UserRole + 1, 0);
			}
		}, Qt::QueuedConnection);


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

	connect(ui.toolButton_gantryStatus, &QToolButton::clicked, this, [=]() {
		MotionController::instance().set_servo(_motionID, 0, (int)Axis::X, true);
		MotionController::instance().set_servo(_motionID, 0, (int)Axis::Y, true);
		MotionController::instance().set_servo(_motionID, 0, (int)Axis::Z, true);
	});


	//This section can be on or off
	connect(ui.toolButton_enableFiducial, &QToolButton::clicked, this, [=]() {
		enableFiducial(!ui.checkBox_enableFiducial->isChecked());
	});

	connect(ui.toolButton_enableSaveInspImages, &QToolButton::clicked, this, [=]() {
		enableSaveInspectionImage(!ui.checkBox_EnableSaveInspectionImage->isChecked());
	});

	connect(ui.toolButton_startProduction, &QToolButton::clicked, this, [=]() {
		startProduction();
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

		SystemData::instance()._Machine_Ready = true;

	});
}

//Production-mode selector (DI X108), same behaviour as 6DF:
// ON  -> jump to the production page and enable only its controls
// OFF -> restore the full UI so the user can navigate anywhere
void VisionApp::applyProductionModeDI()
{
	if (_diProductionMode) {
		ct::logger::info("[Mode] Production mode ON - locking UI to the production page");
		showProductionPage();
		setUiLockedToProduction(true);
	}
	else {
		ct::logger::info("[Mode] Production mode OFF - full UI restored");
		setUiLockedToProduction(false);
	}
}

//Production-mode lock: disable every button that is NOT on the production page, so only
//the production controls remain clickable. Remembers what it disabled so exiting the mode
//restores exactly those (won't re-enable things disabled for other reasons).
void VisionApp::setUiLockedToProduction(bool lock)
{
	if (lock) {
		if (!_prodLockedWidgets.isEmpty()) return; //already locked

		QWidget* prodPage = ui.page_23; //the production page (just shown)
		for (QAbstractButton* b : findChildren<QAbstractButton*>()) {
			if (prodPage->isAncestorOf(b)) continue;       //keep production-page controls
			if (b == ui.toolButton_machineState) continue; //keep the state button (home / reset alarm)
			if (b->isEnabled()) { _prodLockedWidgets.append(b); b->setEnabled(false); }
		}
	}
	else {
		for (const QPointer<QWidget>& w : _prodLockedWidgets)
			if (w) w->setEnabled(true);
		_prodLockedWidgets.clear();
	}
}

void VisionApp::startAcquisition()
{
	_processType = ProcessType::IMAGE_COLLECTION;

	SystemData::instance()._offlineRun = false;
	emit signalLoadToPosition(0);
}

void VisionApp::startProduction()
{
	auto state = MachineController::instance().getMachineState();
	if (state == MachineState::NOT_READY && !SystemData::instance()._machineDebugMode) {
		showMsg("Please initialize machine to proceed!"); //debug mode: allowed without init/home
		return;
	}
	else if (state == MachineState::S_ERROR) {
		showMsg("Unable to start production when machine is in ERROR state!");
		return;
	}

	//outside debug mode, production only starts with the production mode selector ON
	if (!SystemData::instance()._machineDebugMode && !_diProductionMode) {
		showMsg("Turn the production mode selector ON to start production!");
		return;
	}

	AuditLog::instance().log(QStringLiteral("PRODUCTION_START"), Common::Directory::CurrentRecipe);

	ui.lineEdit_inspectionTimeMain->clear();

	_processType = ProcessType::PRODUCTION;

	//fresh run: clear the unit status table and the machine status log
	ui.tableWidget_prodStatus->setRowCount(0);
	clearErrorLogs();

	//production runs inspect: OCR + 3D height on the acquired images
	InspectionThread::instance().setActive(true);

	uidGenerator uidGen;
	_currentProductionID = uidGen.id().c_str();

	//create the production dir now so JobThread's root path is valid before any
	//reader image is saved (the barcodeDecoded handler re-runs this to refresh info.json)
	setupProductionDir();

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

	SystemData::instance()._offlineRun = false;
	emit signalLoadToPosition(0);

	vs_startElapseTimer();
}

void VisionApp::startProductionS()
{
	auto state = MachineController::instance().getMachineState();
	if (state == MachineState::NOT_READY && !SystemData::instance()._machineDebugMode) {
		showMsg("Please initialize machine to proceed!"); //debug mode: allowed without init/home
		return;
	}
	else if (state == MachineState::S_ERROR) {
		showMsg("Unable to start production when machine is in ERROR state!");
		return;
	}

	//outside debug mode, production only starts with the production mode selector ON
	if (!SystemData::instance()._machineDebugMode && !_diProductionMode) {
		showMsg("Turn the production mode selector ON to start production!");
		return;
	}

	ui.lineEdit_inspectionTimeMain->clear();


	_processType = ProcessType::PRODUCTION;

	//fresh run: clear the unit status table and the machine status log
	ui.tableWidget_prodStatus->setRowCount(0);
	clearErrorLogs();

	//production runs inspect: OCR + 3D height on the acquired images
	InspectionThread::instance().setActive(true);

	uidGenerator uidGen;
	_currentProductionID = uidGen.id().c_str();

	//create the production dir now so JobThread's root path is valid before any
	//reader image is saved (the barcodeDecoded handler re-runs this to refresh info.json)
	setupProductionDir();

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




	SystemData::instance()._offlineRun = false;
	runProdS();

	vs_startElapseTimer();
}


void VisionApp::boardInPosition(int pos) {
	ct::logger::info("Board in position");

	if (_processType == ProcessType::IMAGE_COLLECTION) {
		collectImages();
		ct::logger::info("pew pew");
	}
	else if (_processType == ProcessType::PRODUCTION) {
		ct::logger::info("[boardInPosition]Start InspectionThread in Production Mode");
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
		inspect2D3D();
	}
}


void VisionApp::unloadBoard()
{
	emit signalUnloadBoard();
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


