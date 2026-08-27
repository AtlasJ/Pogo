#include "VisionApp.h"
#include "ScaleManager.h"
#include "ImagePathManager.h"
#include "uidGenerator.h"
#include "MachineController.h"
#include "MotionController.h"
#include "AuditLog.h"
#include "SRXManager.h"

void VisionApp::connectJobThread() {
	qRegisterMetaType<MIL_ID>("MIL_ID");
	qRegisterMetaType<InspStatus::FiducialDetail>();
	qRegisterMetaType<LaserAlignmentImage>("LaserAlignmentImage");
	qRegisterMetaType<OpticsInfo>("OpticsInfo");
	qRegisterMetaType<dat::WorldCoordinate>("dat::WorldCoordinate");
	qRegisterMetaType<mtrx::ForegoundType>("mtrx::ForegoundType");
	qRegisterMetaType<PositionPortabilityType>("PositionPortabilityType");

	QObject::connect(&_jobThread, &JobThread::promptMsg, this, [=](QString msg) {
		showMsg(msg);
	});

	QObject::connect(&_jobThread, &JobThread::signalBoardInPosition, this, [=](int pos) {
		boardInPosition(pos);
	});

	QObject::connect(&_jobThread, &JobThread::clearSubRecipe, this, [=]() {
		clearSubRecipe();
	});

	QObject::connect(&_jobThread, &JobThread::signalBoardUnloaded, this, [=]() {

		vs_stopElapseTimer();

		emit signalStopSRX();

		switchToMainRecipe();
		//runLooping(); //only run if there's loop
	});

	QObject::connect(&_jobThread, &JobThread::signalLoadSequenceFail, this, [=](QString msg) {
		stopRun();
		showMsg(msg);
	});

	QObject::connect(&_jobThread, &JobThread::displayFOV, this, [=](MIL_ID mBuf) {
		_imageFOV = mtrx::to_qimg(mBuf);
		mtrx::free_buffer(mBuf);
		displayFOV(_imageFOV);
	});

	QObject::connect(&_jobThread, &JobThread::encoderReceived, this, [=](dat::WorldCoordinate cc) {
		if (!ui.toolButton_toggleFovView->isChecked()) {
			auto wpx = ScaleManager::instance().to_world_px(QPointF(cc.wx, cc.wy));
			drawFOVInWorld(wpx.x(), wpx.y());
			ct::logger::debug("Draw to: %f, %f", wpx.x(), wpx.y());
		}

		//emit signalEncoderChanged(cc.wx, cc.wy, cc.wz);
	});

	//Camera
	QObject::connect(&_jobThread, &JobThread::cameraScalingDone, this, [=](double horizontal_scale, double vertical_scale) {
		ScaleManager::instance().set_horizontal_um_per_px(horizontal_scale);
		ScaleManager::instance().set_vertical_um_per_px(vertical_scale);

		ui.lineEdit_horizontalScale->setText(QString::number(horizontal_scale));
		ui.lineEdit_verticalScale->setText(QString::number(vertical_scale));

		saveWorldEnv();
		AuditLog::instance().log(QStringLiteral("CALIB_CAMERA_SCALE"), QStringLiteral("h=%1 v=%2").arg(horizontal_scale).arg(vertical_scale));
		showMsg(QStringLiteral("Scaling: %1, %2\n").arg(horizontal_scale).arg(vertical_scale));
	});

	QObject::connect(&_jobThread, &JobThread::cameraAlignmentDone, this, [=](double cameraAngle) {
		setCameraAngle(cameraAngle);

		jsonHelper::setJsonValue(_systemObj, "Camera_Angle", SystemData::instance()._camAngles[_camID]);
		updateSystemInfo(_systemObj);

		AuditLog::instance().log(QStringLiteral("CALIB_CAMERA_ANGLE"), QString::number(cameraAngle));
		showMsg(QStringLiteral("Camera angle: %1").arg(cameraAngle));
	});

	QObject::connect(&_jobThread, &JobThread::cameraAlignmentFailed, this, [=](QString msg) {
		setCameraAngle(_prevCamAlignedAngle);
		showMsg(msg);
	});


	//Fiducial
	QObject::connect(&_jobThread, &JobThread::locatedFiducial, this, [=](QRectF roi) { 
		_fidLocatedRegion.setGeometry(roi);
		_fidLocatedRegion.show();
		processEvents();
	});

	QObject::connect(&_jobThread, &JobThread::updateFiducialStatus, this, [=](InspStatus::FiducialDetail detail) {
		_inspStatus.fiducialHash.insert(detail.index, detail);
	});

	QObject::connect(&_jobThread, &JobThread::updateFiducialRegion, this, [=]() {
		_fidInspectionRegion.setGeometry(_fidLocatedRegion.getGeometry());
		_fidInspectionRegion.show();
		updateFiducialRegions();
	}, Qt::BlockingQueuedConnection);

	QObject::connect(&_jobThread, &JobThread::teachFiducialPoint, this, [=]() {
		auto& cc = SystemData::instance().currentCoordinate();
		teachPoint(_currentTeachPointType, _currentFidIndex, cc, false);
		toggleFidROISetupMode(true);
	}, Qt::QueuedConnection);

	QObject::connect(&_jobThread, &JobThread::fiducialDone, this, [=]() {
		setCameraAngle(-_fiducial.getAngle() + SystemData::instance()._camAngles[_camID]);
	});

	QObject::connect(&_jobThread, &JobThread::fiducialFailed, this, [=]() {
		setCameraAngle(_prevCamAlignedAngle);
		stopRun(); 
		showMsg("Failed to locate fiducial!");
	});

	QObject::connect(&_jobThread, &JobThread::barcodeDecoded, this, [=](QString code) {

		SystemData::instance()._currentBarcode = code.toStdString();

		if (_processType == ProcessType::PRODUCTION) {
			bool barcodePass = true;

			if (code == "Fail_to_read_barcode") {
				ct::logger::error("[Barcode] Production read FAILED - no barcode decoded for this board (subRecipe %d)",
					SystemData::instance()._subRecipeIndex);
				setupProductionDir();
				if (_progressDialog) _progressDialog->close();
				barcodePass = false;
			}
			else if (code == "External_Barcode") {
				QString externalCode;
				if (SystemData::instance()._subRecipeIndex == 0) {
					externalCode = ui.lineEdit_barcodeID->text();
				}
				else if (SystemData::instance()._subRecipeIndex == 1) {
					externalCode = ui.lineEdit_barcodeID2->text();
				}

				// Empty (reader too slow/never answered) or ERROR must never become the
				// barcode - it is used as the emap filename key downstream.
				if (externalCode.isEmpty() || externalCode == "ERROR") {
					ct::logger::warn("[Barcode] External barcode %d empty/ERROR at consume time, fallback to No_Barcode",
						SystemData::instance()._subRecipeIndex + 1);
					externalCode = "No_Barcode";
				}

				SystemData::instance()._currentBarcode = externalCode.toStdString();
				ct::logger::info("[Barcode] Production barcode = %s (subRecipe %d)",
					externalCode.toStdString().c_str(), SystemData::instance()._subRecipeIndex);
				setupProductionDir();
			}
			else {
				setupProductionDir();
			}

			/*if (barcodePass) {
			}*/
		}
		
		saveBarcodeResult();
	});

	QObject::connect(&_jobThread, &JobThread::locatedBarcode, this, [=](QRectF roi, int index, bool pass, QString code) {
		showBarcodeDebugImage(index);

		_barcodeLocatedRegion.setGeometry(roi);
		_barcodeLocatedRegion.show();

		ui.lineEdit_decodedBarcode->setText(code);

		QPalette palette = ui.lineEdit_decodedBarcode->palette();
		if (pass) palette.setColor(QPalette::Text, Qt::green);
		else palette.setColor(QPalette::Text, Qt::red);
		ui.lineEdit_decodedBarcode->setPalette(palette);

		updateSetupCheckList();
		processEvents();
	});

	QObject::connect(&_jobThread, &JobThread::updateLaserOffset, this, [=]() {
		saveLaserConfig();
		AuditLog::instance().log(QStringLiteral("CALIB_LASER_OFFSET"));
		updateLaserOffsetUI(_laserConfig.offset);
	});

	QObject::connect(&_jobThread, &JobThread::drawRectFOV, this, [=](QString name, QRectF rect, QColor color) {
		drawRect(_pGraphicsSceneFOV, name, rect, color);
	});

	QObject::connect(&_jobThread, &JobThread::planeCollectionDone, this, [=]() {
		stitchPlaneImage(_plane);
	});

	QObject::connect(&_jobThread, &JobThread::appendLaserAlignmentImage, this, [=](LaserAlignmentImage lai) {
		_laserAlignmentImages.append(lai);
	});

	QObject::connect(&_jobThread, &JobThread::verifyLaserAlignmentDone, this, [=]() {
		displayCurrentAlignmentImage();
		showMsg("Done scanning, you may check the Verification Images.");
	});

	QObject::connect(&_jobThread, &JobThread::laserAlignmentDone, this, [=]() {
		updateLaserOffsetUI(_laserConfig.offset);
		saveLaserConfig();
		displayCurrentAlignmentImage();
		showMsg("Done alignment, you may check the Verification Images.");
	});

	QObject::connect(&_jobThread, &JobThread::captureAlignmentDone, this, [=]() {
		displayCurrentAlignmentImage();
		showMsg("Done alignment, you may check the Verification Images.");
		});

	QObject::connect(&_jobThread, &JobThread::obtainedIdealIntensity, this, [=](int R, int G, int B) {
		auto info = QString("%1, %2, %3").arg(R).arg(G).arg(B);
		ui.lineEdit_obtainedIntensity->setText(info);
	});

	QObject::connect(&_jobThread, &JobThread::savePortabilityInfo, this, [=]() {
		savePortabilityInfo();
		showMsg("Light calibration done!");
	});

	QObject::connect(&_jobThread, &JobThread::loadPortabilityInfo, this, [=]() {
		loadPortabilityInfo();
		loadRefPositionPortabilityInfo();
		loadCurPositionPortabilityInfo();
	});

	QObject::connect(&_jobThread, &JobThread::startProgressBar, this, [=](QString title, int count, bool enableCancel) {
		progressBarSetup(title, count, enableCancel);
	});

	QObject::connect(&_jobThread, &JobThread::incrementProgress, this, [=]() {
		incrementProgressBar();
	});

	QObject::connect(&_jobThread, &JobThread::stopProgressBar, this, [=]() {
		progressBarRelease();
	});

	connect(ui.comboBox_warpageMethod, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [=](int index) {
		_jobThread.setWarpageMethod(ui.comboBox_warpageMethod->currentText());
		_warpageMethod = ui.comboBox_warpageMethod->currentText();
		saveRecipeConfig();
	});

	QObject::connect(&_jobThread, &JobThread::startLot, this, [=]() {
		ct::logger::info("Start Lot");
		// delete lot info file
		QString filePath = jsonHelper::getString(_systemObj, QStringLiteral("Machine_Share_Folder_Path")) + "LotInfoOutput.txt";
		QFile file(filePath);
		if (file.exists()) file.remove();

		ui.label_status->hide();
	
		//EMAP WARNING 
		bool stopRun = false;
		bool runGoldenRecipeFlag = false;
		QString response = QStringLiteral("R\r");

		if (!_enableEmap)
		{
			QMessageBox messageBox;
			messageBox.setWindowTitle("EMAP DISABLED!");
			messageBox.setText("<font color=\"red\"><b>!!INCOMING EMAP DISABLED WARNING!!</b></font><br>Yes to continue process<br>No to stop the lot inspection");
			messageBox.setIcon(QMessageBox::Warning);

			// Set a bigger font size for the message text
			QFont font = messageBox.font();
			font.setPointSize(24); // Adjust the font size as needed
			messageBox.setFont(font);

			// Adjust the size of the message box
			messageBox.setMinimumWidth(1600); // Adjust the width as needed
			messageBox.setMinimumHeight(1600); // Adjust the height as needed
			messageBox.setWindowFlag(Qt::WindowStaysOnTopHint);

			// Add Yes and No buttons
			messageBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);

			// Execute and get the user's response
			int result = messageBox.exec();

			if (result == QMessageBox::Yes) {
				stopRun = false;
			}
			else if (result == QMessageBox::No) {

				stopRun = true;
				response = QStringLiteral("F\r");
			}
		}

		if (_enableGoldenRecipeChecking && !stopRun)
		{
			// 1. check whether current recipe has run Golden
			bool hasRunGolden = checkGoldenRecipeRunStatus(Common::Directory::CurrentRecipe);
			if (!hasRunGolden)
			{
				runGoldenRecipeFlag = true;
				// 2. If no, Run golden recipe
				runGoldenRecipe();
			}
		}

		if (!runGoldenRecipeFlag) sendToClient(response);

	}, Qt::BlockingQueuedConnection);

	QObject::connect(&_jobThread, &JobThread::setLotSize, this, [=](int lotSize) {
		ct::logger::info("Set Lot Size to: %d", lotSize);
	});

	QObject::connect(&_jobThread, &JobThread::endLot, this, [=]() {
		ct::logger::info("End Lot");

		QString facing;
		bool isTop = true;
		checkRecipeFacing(Common::Directory::CurrentRecipe, isTop);
		facing = isTop ? "Top" : "Bottom";
		if (_lotInfo.lotNumber.isEmpty()) _lotInfo.lotNumber = "-";
		QString reportPath = jsonHelper::getString(_systemObj, QStringLiteral("Verification_Station_Ip_Address")) + "/Advanced/ResultViewerEnv/Verified/Report/htmlReport/" + _lotInfo.lotNumber + "/";;
		QString reportName = _lotInfo.lotNumber + "[@]" + facing + ".html";

		// Format the final string
		QString formattedString = "EndLotReport," + reportPath + "," + reportName + ",R\r";
		sendToClient(formattedString);
	});

	QObject::connect(&_jobThread, &JobThread::unloadStrip, this, [=]() {
		if (_progressDialog) _progressDialog->close();
	});

	QObject::connect(&_jobThread, &JobThread::uploadRecipe, this, [=](QString recipeName) {
		bool suc = pushToRmsRecipe(recipeName);
		if (!suc) sendToClient(QStringLiteral("F\r"));
		else
		{
			QString response = QStringLiteral("R\r");
			sendToClient(response);
		}
	});

	QObject::connect(&_jobThread, &JobThread::downloadRecipe, this, [=](QString recipeName) {
		bool suc = pullFromRmsRecipe(recipeName);
		if (!suc) sendToClient(QStringLiteral("F\r"));
		else
		{
			QString response = QStringLiteral("R\r");
			sendToClient(response);
			openRecipe(recipeName);
		}
	});

	QObject::connect(&_jobThread, &JobThread::frameReady, this, [=]() {
		_processType = ProcessType::PRODUCTION;

		if (_inspectionThreadBusy) sendToClient(QStringLiteral("F\r"));
		else
		{
			if (_dryRun && !_saveInspImg)
			{
				sendToClient(QStringLiteral("R\r"));

				sendToClient("01END\r");

				return;
			}

			resetLoopFlags();

			InspStatus inspStatus;
			inspStatus.inspectionStartTime = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss");
			inspStatus.productionMode = true;
			_inspStatus = inspStatus;

			ct::logger::debug("{incomingJob} Frame loaded.");

			//waitForJogDone(dat::WorldCoordinate(), true);
			sendToClient(QStringLiteral("R\r"));

			_inspMode = true;

			inspect2D3D();

		}
	});

	QObject::connect(&_jobThread, &JobThread::onLive, this, [=](QString camID) {
		ui.comboBox_cameraSelection->setCurrentText(camID);
		_camID = camID;
		startLiveView();
	});

	QObject::connect(&_jobThread, &JobThread::offLive, this, [=]() {
		stopLiveView();
	});

	QObject::connect(&_jobThread, &JobThread::openRecipe, this, [=](QString recipeName) {
		openRecipe(recipeName);
	});

	QObject::connect(&_jobThread, &JobThread::createRecipe, this, [=](const QString& recipeName) {
		createRecipe(recipeName);
	});

	QObject::connect(&_jobThread, &JobThread::stackImages, &_imageManager, &ImageManager::queueStackImage, Qt::QueuedConnection);

	QObject::connect(&_jobThread, &JobThread::acquisitionDone, this, [=]() {
		setCameraAngle(_prevCamAlignedAngle);

		if (_processType == ProcessType::IMAGE_COLLECTION) {
			if (!_subrecipesToRun.isEmpty()) {
				_subrecipesToRun.clear();
				SystemData::instance()._subRecipeIndex = 1;
				switchToSubRecipe();
				emit signalLoadToPosition(SystemData::instance()._subRecipeIndex);
			}
			else if (_loop > 0) {
				recordMemory(QString("Start #%1").arg(_loop));
				_loop--;
				startAcquisition();
			}
			else {
				showMsg("Acquisition Done!");
			}
		}
	});

	QObject::connect(this, &VisionApp::snapImage, &_jobThread, &JobThread::snapOptic, Qt::BlockingQueuedConnection);
	QObject::connect(this, &VisionApp::snapImageFastMode, &_jobThread, &JobThread::snapOpticFastMode, Qt::BlockingQueuedConnection);
	QObject::connect(this, &VisionApp::testJob, &_jobThread, &JobThread::test, Qt::QueuedConnection);

	QObject::connect(this, &VisionApp::jogTo, &_jobThread, &JobThread::jogUser, Qt::QueuedConnection);
	QObject::connect(this, &VisionApp::signalReconnectMotion, &_jobThread, &JobThread::reconnectMotion, Qt::QueuedConnection);
	QObject::connect(this, &VisionApp::jogSnap, &_jobThread, &JobThread::jogSnap, Qt::QueuedConnection);
	QObject::connect(this, &VisionApp::jogLeft, &_jobThread, &JobThread::jogLeft, Qt::QueuedConnection);
	QObject::connect(this, &VisionApp::jogRight, &_jobThread, &JobThread::jogRight, Qt::QueuedConnection);
	QObject::connect(this, &VisionApp::jogFront, &_jobThread, &JobThread::jogFront, Qt::QueuedConnection);
	QObject::connect(this, &VisionApp::jogBack, &_jobThread, &JobThread::jogBack, Qt::QueuedConnection);
	QObject::connect(this, &VisionApp::jogUp, &_jobThread, &JobThread::jogUp, Qt::QueuedConnection);
	QObject::connect(this, &VisionApp::jogDown, &_jobThread, &JobThread::jogDown, Qt::QueuedConnection);

	QObject::connect(this, &VisionApp::homeX, &_jobThread, &JobThread::homeX, Qt::QueuedConnection);
	QObject::connect(this, &VisionApp::homeY, &_jobThread, &JobThread::homeY, Qt::QueuedConnection);
	QObject::connect(this, &VisionApp::homeZ, &_jobThread, &JobThread::homeZ, Qt::QueuedConnection);
	QObject::connect(this, &VisionApp::homeXYZ, &_jobThread, &JobThread::homeXYZ, Qt::QueuedConnection);
	QObject::connect(this, &VisionApp::homeAll, &_jobThread, &JobThread::homeAll, Qt::QueuedConnection);

	// Fiducial-related methods
	//QObject::connect(this, &VisionApp::autoSetFiducialPoint, &_jobThread, &JobThread::autoSetFiducialPoint, Qt::QueuedConnection);
	QObject::connect(this, &VisionApp::autoSetFiducialPoint, this, [this]() {
		if (!passwordPromptCorrect()) return;
		backupFiducial();
		QMetaObject::invokeMethod(&_jobThread, "autoSetFiducialPoint", Qt::QueuedConnection, Q_ARG(int, _currentFidIndex));
	});

	QObject::connect(this, &VisionApp::testFiducial, &_jobThread, &JobThread::testFiducial, Qt::QueuedConnection);

	// Barcode reading
	QObject::connect(this, &VisionApp::readBarcode, &_jobThread, &JobThread::readBarcode, Qt::QueuedConnection);

	// Camera alignment
	qRegisterMetaType<AlignFeatureParams>("AlignFeatureParams");
	QObject::connect(this, &VisionApp::performCameraAlignment, &_jobThread, &JobThread::performCameraAlignment, Qt::QueuedConnection);

	QObject::connect(this, &VisionApp::performCameraScaling, &_jobThread, &JobThread::performCameraScaling, Qt::QueuedConnection);

	// Laser alignment
	QObject::connect(this, &VisionApp::performLaserAlignment, &_jobThread, &JobThread::performLaserAlignment, Qt::QueuedConnection);

	QObject::connect(this, &VisionApp::captureAlignmentImages, &_jobThread, &JobThread::captureAlignmentImages, Qt::QueuedConnection);
	

	QObject::connect(this, &VisionApp::performGuidedLaserAlignment, &_jobThread, &JobThread::performGuidedLaserAlignment, Qt::QueuedConnection);

	QObject::connect(this, &VisionApp::signalVerifyLaserAlignment, &_jobThread, &JobThread::verifyLaserAlignment, Qt::QueuedConnection);

	// Intensity and lighting calibration
	QObject::connect(this, &VisionApp::signalGetAllIntensityFromExpectedGV, &_jobThread, &JobThread::getAllIntensityFromExpectedGV, Qt::QueuedConnection);

	QObject::connect(this, &VisionApp::calibrateGoldenLightingProfile, &_jobThread, &JobThread::calibrateGoldenLightingProfile, Qt::QueuedConnection);

	QObject::connect(this, &VisionApp::calibrateCurrentLightingProfile, &_jobThread, &JobThread::calibrateCurrentLightingProfile, Qt::QueuedConnection);

	QObject::connect(this, &VisionApp::triggerMaxCurrentCalibration, &_jobThread, &JobThread::calibrateMaxCurrent, Qt::QueuedConnection);

	QObject::connect(this, &VisionApp::signalSetPortabilityPoint, &_jobThread, &JobThread::setPositionPortabilityPoint, Qt::QueuedConnection);

	QObject::connect(this, &VisionApp::sendToClient, &_jobThread, &JobThread::sendToClient, Qt::QueuedConnection);

	QObject::connect(this, &VisionApp::collectZImages, &_jobThread, &JobThread::collectZImages, Qt::QueuedConnection);

	QObject::connect(this, &VisionApp::signalOnlineStitchingSimulation, &_jobThread, &JobThread::simulateOnlineStitching, Qt::QueuedConnection);

	QObject::connect(this, &VisionApp::signalFindPortabilityPattern, &_jobThread, &JobThread::findPortabilityPattern, Qt::QueuedConnection);
	QObject::connect(this, &VisionApp::signalFindPortabilityCircle, &_jobThread, &JobThread::findPortabilityCircle, Qt::QueuedConnection);

	QObject::connect(this, &VisionApp::signalLoadToPosition, &_jobThread, &JobThread::loadToPositionSensor, Qt::QueuedConnection);
	QObject::connect(this, &VisionApp::signalUnloadBoard, &_jobThread, &JobThread::unloadBoard, Qt::QueuedConnection);

	QObject::connect(&_jobThread, &JobThread::calibrationFinished, this, &VisionApp::handleCalibrationFinished);


	QObject::connect(&SRXManager::instance(), &SRXManager::barcodeReceived, this, [=](const QString& readerID, const QString& code) {

		// One reader per barcode slot: SRX1 -> barcode 1, SRX2 -> barcode 2
		const int readerIndex = SRXManager::indexOf(readerID);
		if (readerIndex < 0 || readerIndex > 1) return;

		const bool allowed = (_enableBarcode &&
			(int)_barcodeInfos.size() > readerIndex &&
			_barcodeInfos[readerIndex].registration_method == 2);   // 2=External

		if (!allowed) {
			ct::logger::info("[Barcode] Reader %d payload ignored (slot not External): %s",
				readerIndex + 1, code.toStdString().c_str());
			return;
		}

		QLineEdit* slot = (readerIndex == 0) ? ui.lineEdit_barcodeID : ui.lineEdit_barcodeID2;

		if (code == "ERROR") {
			// Mark only this reader's slot; a later good code may still overwrite it
			if (slot->text().isEmpty()) {
				ct::logger::warn("[Barcode] Reader %d FAILED to read (reported ERROR), Barcode %d marked ERROR",
					readerIndex + 1, readerIndex + 1);
				slot->setText("ERROR");
			}
			else {
				ct::logger::warn("[Barcode] Reader %d reported ERROR but Barcode %d already holds '%s', keeping it",
					readerIndex + 1, readerIndex + 1, slot->text().toStdString().c_str());
			}
			return;
		}

		// Fill-if-empty: a captured code is never overwritten (duplicate/late sends ignored)
		if (!slot->text().isEmpty() && slot->text() != "ERROR") {
			ct::logger::info("[Barcode] Reader %d duplicate/late code ignored: %s",
				readerIndex + 1, code.toStdString().c_str());
			return;
		}

		slot->setText(code);
		ct::logger::info("[Barcode] Reader %d -> Barcode %d: %s",
			readerIndex + 1, readerIndex + 1, code.toStdString().c_str());
		});

	// Must be queued: the SR-X sockets live in the job thread, and writing to them
	// from the GUI thread trips "QSocketNotifier ... from another thread" and drops the write.
	QObject::connect(this, &VisionApp::signalTriggerSRX, &_jobThread, &JobThread::triggerSRX, Qt::QueuedConnection);
	QObject::connect(this, &VisionApp::signalStopSRX, &_jobThread, &JobThread::stopSRX, Qt::QueuedConnection);

	_jobThread.attach(ui.listWidget_paths);
	_jobThread.attach(&_fiducial);
	_jobThread.attach2ndFiducial(&_fiducial2);
	_jobThread.attach(&_fiducialInfos);
	_jobThread.attach(&_barcodeInfos);
	_jobThread.attach(&_inspStatus);
	_jobThread.attach(&_plane);
	_jobThread.attach(&_CSA);
	_jobThread.attach(&_portabilityInfo);

	_jobThread.attach(&_laserConfig.offset);

	_jobThread.moveToThread(&_jobThread);
	_jobThread.start(QThread::HighPriority);

	//production inspection worker (OCR + 3D height); idle until production starts
	QObject::connect(&InspectionThread::instance(), &InspectionThread::inspectionResult, this,
		[=](QString unitID, QString algo, bool pass, QString detail) {
			addLogLine(QString("[Inspection] %1 %2: %3 (%4)").arg(unitID, algo, pass ? "PASS" : "FAIL", detail));
		}, Qt::QueuedConnection);
	InspectionThread::instance().start();
}

void VisionApp::simulateImageSaving()
{
	ct::logger::info("Start image saving simulation");

	_processType = ProcessType::IMAGE_COLLECTION;

	_stopRun = false;

	QHash<QString, QView>::const_iterator view = _views.constBegin();
	while (view != _views.constEnd())
	{
		if (_stopRun) {
			return;
		}

		if (util::isRAMOver(SystemData::instance()._maxAllowRamUsage)) {
			ct::logger::warn("[Offline] Memory overload, waiting for more memory to proceed...");

			while (util::isRAMOver(SystemData::instance()._maxAllowRamUsage)) {
				if (_stopRun) break;
				os_tool::goSleep(1000);
			}
			ct::logger::warn("[Offline] Sufficient memory, proceed acquisition.");
		}

		if (view.value().type == ct::s_child_view) {
			view++;
			continue;
		}

		ct::logger::info("[Offline] Trying view: %s", view.value().id.toStdString().c_str());

		QVector<FrameInfo> IInfos;
		auto ipf = path::getViewPath(Common::Directory::CurrentImageSetPath.toStdString(), view.value(), _mainOptics[_camID], _recipeOptics);

		QHash<QString, QString> imgPaths;
		imgPaths = ipf.getAllOpticPaths();

		QHash<QString, QString>::const_iterator imgPath = imgPaths.constBegin();

		while (imgPath != imgPaths.constEnd())
		{
			QString iPath = util::convert_to_BMP_ext(imgPath.value());
			ct::logger::info("[Offline] Load 2D: %s", iPath.toStdString().c_str());
			MIL_INT bandSize = 3;

			FrameInfo IInfo;
			mtrx::SharedMilID img = nullptr;
			if (QFileInfo::exists(iPath))
			{
				MIL_INT sizeX = 0, sizeY = 0;
				MIL_INT type = 0;
				MbufDiskInquireA(iPath.toStdString().c_str(), M_SIZE_BAND, &bandSize);
				MbufDiskInquireA(iPath.toStdString().c_str(), M_SIZE_X, &sizeX);
				MbufDiskInquireA(iPath.toStdString().c_str(), M_SIZE_Y, &sizeY);
				MbufDiskInquireA(iPath.toStdString().c_str(), M_TYPE, &type);

				img = mtrx::MbufPoolManager::instance().acquire(sizeX, sizeY, bandSize, type);

				MIL_INT imgType = M_JPEG_LOSSY;
				if (util::isPNG(iPath)) imgType = M_PNG;
				if (util::isBMP(iPath)) imgType = M_BMP;

				MbufLoadA(iPath.toStdString().c_str(), img->id());
				copyFileToFolder(iPath, Common::Directory::getProductionImageSetPath());
			}
			else
			{
				img = mtrx::MbufPoolManager::instance().acquire(5120, 5120, 3, 8 + M_UNSIGNED);
				MbufClear(img->id(), M_BLACK);
				ct::logger::error("Path not found: %s", iPath.toStdString().c_str());
			}

			MIL_INT width, height;
			MbufInquire(img->id(), M_SIZE_X, &width);
			MbufInquire(img->id(), M_SIZE_Y, &height);
			size_t  imgSize = width * height;

			QString optType = ct::s_color;
			if (bandSize == 1) optType = ct::s_mono;

			uidGenerator uidGen;
			IInfo.width = (int)width;
			IInfo.height = (int)height;
			IInfo.bufferSize = (int)imgSize;
			IInfo.timeStamp = QString(uidGen.id().c_str()).toInt();
			IInfo.cameraID = _camID;
			IInfo.viewID = view.value().id;
			IInfo.opticID = imgPath.key();
			IInfo.type = optType;
			IInfo.pImage = img;
			g_imageQueue.push_back(IInfo);

			//IInfos.push_back(IInfo);

			imgPath++;
		}

		_progressValue++;
		if (_progressDialog)_progressDialog->setValue(_progressValue);

		if (ui.checkBox_runOneFOVonly->isChecked()) break;

		if (_stopRun) {
			return;
		}

		++view;
	}
}

void VisionApp::simulateOnlineInspection()
{
	QHash<QString, QView>::const_iterator view = _views.constBegin();

	if (false) {
		while (view != _views.constEnd())
		{
			if (_stopRun) {
				return;
			}

			if (view.value().type == ct::s_child_view) {
				view++;
				continue;
			}

			QVector<FrameInfo> IInfos;
			auto ipf = path::getViewPath(Common::Directory::CurrentImageSetPath.toStdString(), view.value(), _mainOptics[_camID], _recipeOptics);

			QHash<QString, QString> imgPaths;
			imgPaths = ipf.getAllOpticPaths();

			QHash<QString, QString>::const_iterator imgPath = imgPaths.constBegin();

			while (imgPath != imgPaths.constEnd())
			{
				QString iPath = util::convert_to_BMP_ext(imgPath.value());
				ct::logger::info("[Offline] Load 2D: %s", iPath.toStdString().c_str());
				MIL_INT bandSize = 3;

				FrameInfo IInfo;
				MIL_ID img = M_NULL;
				if (QFileInfo::exists(iPath))
				{
					MIL_INT sizeX = 0, sizeY = 0;
					MbufDiskInquireA(iPath.toStdString().c_str(), M_SIZE_BAND, &bandSize);
					MbufDiskInquireA(iPath.toStdString().c_str(), M_SIZE_X, &sizeX);
					MbufDiskInquireA(iPath.toStdString().c_str(), M_SIZE_Y, &sizeY);

					if (bandSize == 1) MbufAlloc2d(M_DEFAULT_HOST, sizeX, sizeY, 8 + M_UNSIGNED, M_IMAGE + M_PROC, &img);
					else MbufAllocColor(M_DEFAULT_HOST, 3, sizeX, sizeY, 8 + M_UNSIGNED, M_IMAGE + M_PROC, &img);

					MIL_INT imgType = M_JPEG_LOSSY;
					if (util::isPNG(iPath)) imgType = M_PNG;
					if (util::isBMP(iPath)) imgType = M_BMP;
					MbufImportA(iPath.toStdString().c_str(), imgType, M_LOAD, M_DEFAULT_HOST, &img);
				}
				else
				{
					MbufAllocColor(M_DEFAULT_HOST, 3, 5120, 5120, 8 + M_UNSIGNED, M_IMAGE + M_PROC, &img);
					MbufClear(img, M_BLACK);
					ct::logger::error("[Offline] Path not found: %s", iPath.toStdString().c_str());
				}

				MIL_INT width, height;
				MbufInquire(img, M_SIZE_X, &width);
				MbufInquire(img, M_SIZE_Y, &height);
				size_t  imgSize = width * height;

				QString optType = ct::s_color;
				if (bandSize == 1) optType = ct::s_mono;

				uidGenerator uidGen;
				IInfo.width = (int)width;
				IInfo.height = (int)height;
				IInfo.bufferSize = (int)imgSize;
				IInfo.timeStamp = QString(uidGen.id().c_str()).toInt();
				IInfo.cameraID = _camID;
				IInfo.viewID = view.value().id;
				IInfo.opticID = imgPath.key();
				IInfo.type = optType;
				IInfo.pImage = mtrx::MPM::instance().attach(img);
				IInfos.push_back(IInfo);

				imgPath++;
			}


			_progressValue++;
			if (_progressDialog)_progressDialog->setValue(_progressValue);

			if (ui.checkBox_runOneFOVonly->isChecked()) break;

			if (_stopRun) {
				ct::logger::info("load offline view returned!!!");
				return;
			}
			++view;
		}
	}


	if (_enable3D) {
		std::map<QString, QString> scanSequence;
		for (const auto& l : _lineScans) {
			if (l.type == ct::s_child_linescan) continue;
			if (l.id == "") continue;

			scanSequence.insert({ QString("%1_%2").arg(l.px.cx).arg(l.id), l.id });
		}

		for (const auto& seq : scanSequence) {

			auto l = _lineScans[seq.second];

			QVector<FrameInfo> IInfos;

			for (auto& o : _recipeOptics3D) {

				if (_stopRun) {
					ct::logger::info("load offline linescan returned!!!");
					return;
				}

				QString hid = l.id + "_HeightMap_" + o.id;
				QString iid = l.id + "_IMap";

				QString path_hm = Common::Directory::CurrentImageSetPath + "/" + hid + ".tiff";
				QString ipath = Common::Directory::CurrentImageSetPath + "/" + iid + ".jpg";

				ct::logger::trace("[Offline] Load IMap: %s", ipath.toStdString().c_str());
				ct::logger::trace("[Offline] Load 3D: %s", path_hm.toStdString().c_str());
				ct::logger::info("[Offline] IMap ID: %s", iid.toStdString().c_str());

				if (!QFile::exists(ipath)) {
					ct::logger::error("[Offline] Failed to load intensity map: %s", ipath.toStdString().c_str());
					g_viewIndex--;
					continue;
				}

				auto mBuf = MbufRestoreA(path_hm.toStdString().c_str(), M_DEFAULT_HOST, M_NULL);
				auto mImap = MbufRestoreA(ipath.toStdString().c_str(), M_DEFAULT_HOST, M_NULL);

				if (mBuf) {

					FrameInfo IInfo;

					auto w = mtrx::get_width(mBuf);
					auto h = mtrx::get_height(mBuf);

					uidGenerator uidGen;
					IInfo.width = w;
					IInfo.height = h;
					IInfo.bufferSize = w * h;
					IInfo.timeStamp = QString(uidGen.id().c_str()).toInt();
					IInfo.cameraID = _camID;
					IInfo.viewID = l.id;


					auto type = mtrx::get_type(mBuf);
					if (type == 16) {
						IInfo.type = ct::s_height_map;
						IInfo.opticID = o.id;
						IInfo.pHeightMap = mtrx::MPM::instance().attach(mBuf);
						IInfo.pImage = mtrx::MPM::instance().attach(mImap);
					}

					if (mImap) {
						IInfo.pImage = mtrx::MPM::instance().attach(mImap);
					}

					ct::logger::info("[Offline] Append heightmap: %s", l.name.toStdString().c_str());
					g_imageQueue.push_back(IInfo);
				}
			}

			_progressValue++;
			if (_progressDialog)_progressDialog->setValue(_progressValue);

			if (ui.checkBox_runOneFOVonly->isChecked()) break;
		}
	}
}

void VisionApp::simulateOnlineStitching()
{
	_processType = ProcessType::IMAGE_COLLECTION;

	_imageManager.attach(&_views, &_recipeOptics);
	_imageManager.attach(&_lineScans, &_recipeOptics3D);
	_imageManager.reset();

	emit signalOnlineStitchingSimulation();
}

void VisionApp::handleCalibrationFinished(QString msg, QHash<QString, double> limits)
{
	// 1. Safely show the popup on the main UI thread
	int choice = QMessageBox::question(this, "Calibration Results", msg, QMessageBox::Yes | QMessageBox::No);

	// 2. Apply the limits if the user clicks Yes
	if (choice == QMessageBox::Yes) {
		auto groupedKeys = OpticsControl::instance().getGroupedOptics();

		for (const auto& key : limits.keys()) {
			OpticsControl::instance().setGroupedOpticMaxCurrent(key, groupedKeys, limits[key]);
		}
		ct::logger::info("User applied the new Max Current limits from calibration.");
	}
	else {
		ct::logger::info("User rejected the proposed Max Current limits.");
	}
}
