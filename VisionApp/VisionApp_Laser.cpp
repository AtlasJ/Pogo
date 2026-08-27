#include "VisionApp.h"
#include "CAMManager.h"
#include "ScaleManager.h"
#include "Guided_2D3D_AlignmentTab.h"
#include "AuditLog.h"
#include "ProfilerManager.h"
#include <QDialog>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QVBoxLayout>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QCheckBox>
#include <QLabel>
#include <QPushButton>
#include <QJsonArray>
#include <QJsonDocument>

void VisionApp::initLaserUI()
{
	initAlignmentMethodUI();

	connect(ui.toolButton_verifyLaserAlignment, &QToolButton::clicked, this, [=]() { verifyLaserAlignment(); });
	connect(ui.toolButton_alignLaser, &QToolButton::clicked, this, [=]() { alignCameraAndLaser(); });
	connect(ui.toolButton_manualAlignLaser, &QToolButton::clicked, this, [=]() { guidedAlignCameraAndLaser(); });
	connect(ui.toolButton_jogToLaserView, &QToolButton::clicked, this, [=]() { 
		auto& cc = SystemData::instance().currentCoordinate();
		jogToLaserView(cc.wx, cc.wy, cc.wz, "2D"); 
		snapImage(_mainOptics[_camID], "", "");
	});
	connect(ui.toolButton_jogToCamView, &QToolButton::clicked, this, [=]() {
		auto& cc = SystemData::instance().currentCoordinate();
		jogToCamView(cc.wx, cc.wy, cc.wz, "2D"); 
		snapImage(_mainOptics[_camID], "", "");
	});
	connect(ui.toolButton_setCircleDiameter, &QToolButton::clicked, this, [=]() { toggleCommonDragBox(ui.toolButton_setCircleDiameter); });
	connect(ui.toolButton_prevLaserAlignmentImage, &QToolButton::clicked, this, [=]() { 
		_currentLaserAlignmentIndex--;
		if (_laserAlignmentImages.isEmpty()) return;
		if (_currentLaserAlignmentIndex < 0) _currentLaserAlignmentIndex = _laserAlignmentImages.size() - 1;
		if (_currentLaserAlignmentIndex >= _laserAlignmentImages.size()) _currentLaserAlignmentIndex = 0;

		auto& l = _laserAlignmentImages[_currentLaserAlignmentIndex];
		QImage scaledImage = l.qimg.scaled(380, 380, Qt::KeepAspectRatio);
		ui.label_laserAlignmentImage->setPixmap(QPixmap::fromImage(scaledImage));
		ui.label_laserAlignmentStatus->setText(l.info);
	});
	connect(ui.toolButton_nextLaserAlignmentImage, &QToolButton::clicked, this, [=]() { 
		_currentLaserAlignmentIndex++;
		if (_laserAlignmentImages.isEmpty()) return;
		if (_currentLaserAlignmentIndex < 0) _currentLaserAlignmentIndex = _laserAlignmentImages.size() - 1;
		if (_currentLaserAlignmentIndex >= _laserAlignmentImages.size()) _currentLaserAlignmentIndex = 0;

		auto& l = _laserAlignmentImages[_currentLaserAlignmentIndex];
		QImage scaledImage = l.qimg.scaled(380, 380, Qt::KeepAspectRatio);
		ui.label_laserAlignmentImage->setPixmap(QPixmap::fromImage(scaledImage));
		ui.label_laserAlignmentStatus->setText(l.info);
	});


	//warpage
	connect(ui.toolButton_generateWarpageMap, &QToolButton::clicked, this, [=]() { 
		//TODO: JObthread
	});
}

int VisionApp::getScanOrientation(const dat::WorldCoordinate & start, const dat::WorldCoordinate & end)
{
	//if (start.wx < end.wx) return GO_ORIENTATION_REVERSE;
	//return GO_ORIENTATION_WIDE;
	return 0;
}

QImage VisionApp::get3DImage(QImage & qimg)
{
	auto rotated = util::rotateQImage(qimg, -90);
	auto cam_w = CAMManager::instance().getWidth(_camID);
	auto cam_h = CAMManager::instance().getHeight(_camID);
	auto scale = rotated.scaled(QSize(cam_w, cam_h), Qt::KeepAspectRatio, Qt::SmoothTransformation);
	return scale;
}

MIL_ID VisionApp::get3DImage(MIL_ID mbuf)
{
	if (mbuf == M_NULL) return M_NULL;

	auto w = mtrx::get_width(mbuf);
	auto h = mtrx::get_height(mbuf);

	ct::logger::debug("[SpeedUp] Initial size: %d, %d", h, w);

	auto scale = ScaleManager::instance().um_per_px();

	MIL_INT sw = w * 7 / scale;
	MIL_INT sh = h * 7 / scale;

	ct::logger::debug("[SpeedUp] Scaled size: %d, %d", sh, sw);

	auto type = mtrx::get_type(mbuf);

	auto mRotate = MbufAlloc2d(M_DEFAULT, h, w, type + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);
	auto mScaled = MbufAlloc2d(M_DEFAULT, sh, sw, type + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);
	MimRotate(mbuf, mRotate, 90, w / 2, h / 2, h / 2, w / 2, M_BICUBIC + M_OVERSCAN_ENABLE);
	MimRotate(mRotate, mRotate, (-_fiducial.getAngle()), M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_BICUBIC + M_OVERSCAN_CLEAR);
	MimResize(mRotate, mScaled, M_FILL_DESTINATION, M_FILL_DESTINATION, M_BICUBIC + M_OVERSCAN_ENABLE);
	//warpPerspective(mScaled);
	mtrx::BufferCollector bc(mRotate);


	return mScaled;
}

void VisionApp::warpPerspective(MIL_ID & milImg)
{
	if (milImg == M_NULL) return;
	// Define four points on the output image
	cv::Point2f output_points[] = { cv::Point2f(357, 235), cv::Point2f(875, 235), cv::Point2f(357, 629), cv::Point2f(875, 629) };

	// Define four points on the input image
	cv::Point2f input_points[] = { cv::Point2f(341, 220), cv::Point2f(863, 220), cv::Point2f(341, 619), cv::Point2f(863, 619) };

	// Find the transformation matrix
	cv::Mat M = cv::getPerspectiveTransform(input_points, output_points);

	cv::Mat cvImg;
	util::Mil_to_cv(milImg, cvImg);

	//MbufSaveA("TransformedBefore.tiff", milImg);

	cv::warpPerspective(cvImg, cvImg, M, cvImg.size());

	util::cv_to_Mil(cvImg, milImg);
	//MbufSaveA("TransformedBefore.tiff", milImg);
}

void VisionApp::alignCameraAndLaser()
{
	auto camThreshold = ui.lineEdit_cameraThreshold->text().toInt();
	auto laserThreshold = ui.lineEdit_laserThreshold->text().toInt();
	emit performLaserAlignment(SystemData::instance().currentCoordinate(), _commonDragBox.getGeometry(), camThreshold, laserThreshold);
}

void VisionApp::enhancedAlignCameraAndLaser()
{
	auto camThreshold = ui.lineEdit_cameraThreshold->text().toInt();
	auto laserThreshold = ui.lineEdit_laserThreshold->text().toInt();
	emit captureAlignmentImages(SystemData::instance().currentCoordinate(), camThreshold, laserThreshold);
}

void VisionApp::guidedAlignCameraAndLaser()
{
	emit performGuidedLaserAlignment(SystemData::instance().currentCoordinate());
}

void VisionApp::guidedAlignCameraAndLaserSetup()
{
	//load 2d and 3d image into graphics View, create two graphic items where 3d is overlayed on top of 2d image
	_guided_2D3D_AlignmentTab->reloadAlignmentImages();
	//set current laser offset
	_guided_2D3D_AlignmentTab->setCurrentLaserOffset(_laserConfig.offset);
	//able to manual move 3d image to map it nicely on the 2d image
	//or can set points for 2d and points for 3d, then click confirm to map them
	_guided_2D3D_AlignmentTab->setMode(Guided_2D3D_AlignmentTab::AlignmentMode::OFFSET_2D3D);
	_guided_2D3D_AlignmentTab->show();
}

void VisionApp::verifyLaserAlignment()
{
	_laserAlignmentImages.clear();
	emit signalVerifyLaserAlignment(SystemData::instance().currentCoordinate());
}

void VisionApp::drawCrossOnQImage(int cx, int cy, int size, QImage & qimg)
{
	QPainter painter(&qimg);
	QPen pen(Qt::red);
	pen.setWidth(30);
	painter.setPen(Qt::red);

	// Draw a horizontal line (cross)
	painter.drawLine(cx - size, cy, cx + size, cy);

	// Draw a vertical line (cross)
	painter.drawLine(cx, cy - size, cx, cy + size);

	// End the painting
	painter.end();
}

void VisionApp::saveQImageWithCrossSection(QImage qimg, QString path)
{
	auto cxh = qimg.width() / 2;
	auto cyh = qimg.height() / 2;
	drawCrossOnQImage(cxh, cyh, 60, qimg);

	qimg.save(path);
}

void VisionApp::displayCurrentAlignmentImage()
{
	if (_laserAlignmentImages.isEmpty()) return;
	if (_laserAlignmentImages.size() <= _currentLaserAlignmentIndex) _currentLaserAlignmentIndex = 0;

	auto& l = _laserAlignmentImages[_currentLaserAlignmentIndex];
	QImage scaledImage = l.qimg.scaled(380, 380, Qt::KeepAspectRatio);
	ui.label_laserAlignmentImage->setPixmap(QPixmap::fromImage(scaledImage));
	ui.label_laserAlignmentStatus->setText(l.info);
}

void VisionApp::scanDone() //WARNING: Dont put UI related stuff here. Emit it thru signal slot
{
	ct::logger::debug("Scan done");
	ct::logger::debug("DONE SCAN ACTIONS");
}


/* ------------------------------------------------------- Profiler Scan Test */

/*
* Test Run page -> Online -> "Profiler Scan Test" -> Run.
*
* Only the two things that change per run are asked for here: how far, and which way. Scan
* speed is deliberately NOT on this dialog - it already has one home (Config page, X 3D
* velocity) and a second control for the same value is how the two drift apart.
*/
void VisionApp::runProfilerScanTest()
{
	if (_recipeOptics3D.isEmpty()) {
		showMsg(QStringLiteral(
			"This recipe has no 3D optics entries.\n\n"
			"Create at least one on the 3D Optics page first. The scan path picks the first "
			"entry with intensity enabled, and has no defined behaviour when there is none."));
		return;
	}

	if (!ProfilerManager::instance().keys().contains(_profilerID)) {
		showMsg(QStringLiteral(
			"No profiler is configured under ID '%1'.\n\n"
			"Check profiler.json, or the Profiler Hardware section on the 3D Optics page.")
			.arg(_profilerID));
		return;
	}

	QDialog dlg(this);
	dlg.setWindowTitle(QStringLiteral("Profiler Scan Test"));

	auto* spinDistance = new QDoubleSpinBox(&dlg);
	spinDistance->setRange(0.1, 1000.0);
	spinDistance->setDecimals(2);
	spinDistance->setSingleStep(1.0);
	spinDistance->setValue(20.0);
	spinDistance->setSuffix(QStringLiteral(" mm"));

	auto* comboDir = new QComboBox(&dlg);
	comboDir->addItem(QStringLiteral("+X   (increasing)"));
	comboDir->addItem(QStringLiteral("-X   (decreasing)"));

	auto* comboOptic = new QComboBox(&dlg);
	int mainIndex = 0;
	bool foundMain = false;
	for (const auto& o : _recipeOptics3D) {
		if (o.intensity && !foundMain) {
			mainIndex = comboOptic->count();
			foundMain = true;
		}
		comboOptic->addItem(o.id);
	}
	comboOptic->setCurrentIndex(mainIndex);
	comboOptic->setToolTip(QStringLiteral(
		"A normal scan uses the first entry with intensity enabled. Picking a different one "
		"here is allowed - the report says which was used and warns if they disagree."));

	auto* checkSave = new QCheckBox(QStringLiteral("Save height map and intensity images"), &dlg);
	checkSave->setChecked(true);

	auto* checkReturn = new QCheckBox(QStringLiteral("Return to the start position afterwards"), &dlg);
	checkReturn->setChecked(true);
	checkReturn->setToolTip(QStringLiteral(
		"Turn this off to leave the gantry parked at the end of the move, so the travel can "
		"be measured by hand."));

	const auto here = SystemData::instance().currentCoordinate();
	int speed2d = 0, speed3d = 0;
	_jobThread.getXSpeed(speed2d, speed3d);

	auto* info = new QLabel(QStringLiteral(
		"Scans along X from where the gantry is now (X = %1 mm).\n"
		"Scan speed is %2 mm/s - change it on the Config page (X 3D velocity),\n"
		"not here, so there is only ever one copy of that number.\n\n"
		"No 2D camera, no laser offset, no recipe views. A report is written to\n"
		"the recipe's Images\\ProfilerTest folder whether or not the scan runs.")
		.arg(here.wx, 0, 'f', 3).arg(speed3d), &dlg);
	info->setWordWrap(true);
	info->setStyleSheet(QStringLiteral("color: #929aaa;"));

	auto* form = new QFormLayout();
	form->addRow(QStringLiteral("Scan distance"), spinDistance);
	form->addRow(QStringLiteral("Direction"), comboDir);
	form->addRow(QStringLiteral("Optics 3D"), comboOptic);

	auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dlg);
	buttons->button(QDialogButtonBox::Ok)->setText(QStringLiteral("Run"));
	connect(buttons, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
	connect(buttons, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);

	auto* layout = new QVBoxLayout(&dlg);
	layout->addWidget(info);
	layout->addLayout(form);
	layout->addWidget(checkSave);
	layout->addWidget(checkReturn);
	layout->addWidget(buttons);

	if (dlg.exec() != QDialog::Accepted) return;

	const double distance = spinDistance->value();
	const bool positive = (comboDir->currentIndex() == 0);
	const QString opticID = comboOptic->currentText();

	ui.textEdit_loopStatus->append(QStringLiteral("Profiler Scan Test: %1 mm %2X, optics %3")
		.arg(distance).arg(positive ? "+" : "-").arg(opticID));

	emit signalProfilerScanTest(distance, positive, opticID,
		checkSave->isChecked(), checkReturn->isChecked());
}

void VisionApp::terminated(int signum)
{
	MachineController::instance().turnOnBrake();

	std::string messageCode = "Unknown";
	if (signum == 2) messageCode = "Process Interrupted";
	else if (signum == 4) messageCode = "Illegal Instruction";
	else if (signum == 6) messageCode = "Abort Signal Received";
	else if (signum == 8) messageCode = "Floating point exception (Arithmetic logic error)";
	else if (signum == 11) messageCode = "Segment Violation (Invalid memory access)";
	else if (signum == 15) messageCode = "Kill Signal Received";
	else if (signum == 21) messageCode = "Ctrl Break Sequence";
	else if (signum == 22) messageCode = "Software Abort";

	if (signum == 22) ct::logger::info("Terminate Signal %d: %s", signum, messageCode.c_str());
	else ct::logger::error("Terminate Signal %d: %s", signum, messageCode.c_str());

	if (signum != 999) exit(signum);
}

// ── camera alignment/scaling feature method (circle vs pattern) ──────────────

static const QString kAlignConfigPath = "C:/Advanced/Data/config/alignment.json";
static const QString kAlignModelPath = "C:/Advanced/Data/config/alignPattern.mpat";

void VisionApp::initAlignmentMethodUI()
{
	auto makeBox = [=](const QRectF& rect, const QColor& color, const QString& name) -> QDragBox* {
		auto box = new QDragBox();
		_pGraphicsSceneFOV->addItem(box);
		box->setOutterBarrier(_pGraphicsSceneFOV->sceneRect());
		box->setup(rect, color, name);
		box->setDragable(true);
		box->setZValue((int)UIHierarchy::DRAGGABLES);
		box->hide();
		return box;
	};

	_alignCircleRoi = makeBox(QRectF(1000, 850, 300, 300), Qt::yellow, "Circle ROI");
	_alignLearnBox = makeBox(QRectF(1000, 850, 300, 300), Qt::cyan, "Align Learn");
	_alignSearchBox = makeBox(QRectF(600, 450, 1200, 1100), Qt::magenta, "Align Search");

	loadAlignmentConfig();
	updateAlignMethodWidgets();

	connect(ui.comboBox_alignMethod, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [=](int) {
		updateAlignMethodWidgets();
		updateAlignRoiVisibility();
		saveAlignmentConfig();
	});

	connect(ui.lineEdit_alignTolerance, &QLineEdit::editingFinished, this, [=]() { saveAlignmentConfig(); });
	connect(ui.lineEdit_alignPatternScore, &QLineEdit::editingFinished, this, [=]() { saveAlignmentConfig(); });

	connect(ui.toolButton_alignLearnPattern, &QToolButton::clicked, this, [=]() {
		if (_imageFOV.isNull()) {
			showMsg("No image displayed. Snap an image first.");
			return;
		}

		auto roi = _alignLearnBox->getGeometry().intersected(QRectF(0, 0, _imageFOV.width(), _imageFOV.height()));
		if (roi.width() < 20 || roi.height() < 20) {
			showMsg("Learn ROI is too small or outside the image.");
			return;
		}

		//crop the displayed FOV, learn the model from it
		QImage crop = _imageFOV.copy(roi.toRect()).convertToFormat(QImage::Format_Grayscale8);
		const QString jpgPath = "C:/Advanced/Data/config/alignPattern.jpg";
		crop.save(jpgPath);

		MIL_ID mImport = M_NULL;
		MbufImportA(jpgPath.toStdString().c_str(), M_DEFAULT, M_RESTORE, M_DEFAULT_HOST, &mImport);
		if (mImport == M_NULL) {
			showMsg("Failed to load the learn image.");
			return;
		}

		MIL_ID mMono = mtrx::to_mono(mImport);
		MbufFree(mImport);
		mtrx::BufferCollector bc(mMono);

		mtrx::PatternInput patInput;
		patInput.filename = kAlignModelPath.toStdString();
		patInput.min_score = 0.0;
		patInput.learn_x = 0;
		patInput.learn_y = 0;
		patInput.learn_w = crop.width();
		patInput.learn_h = crop.height();
		patInput.smoothness = 85;
		patInput.enable_angle = false; //alignment feature does not rotate

		mtrx::PatternOutput patOutput;
		if (!mtrx::learn_pattern(mMono, patInput, patOutput)) {
			showMsg("Failed to learn the alignment pattern. Try a more distinctive region.");
			return;
		}

		saveAlignmentConfig();
		AuditLog::instance().log(QStringLiteral("ALIGN_PATTERN_LEARN"));
		showMsg("Alignment pattern learned.");
	});
}

void VisionApp::updateAlignMethodWidgets()
{
	const bool pattern = ui.comboBox_alignMethod->currentIndex() == 1;

	ui.comboBox_circleColor->setVisible(!pattern);
	ui.label_alignTolerance->setVisible(!pattern);
	ui.lineEdit_alignTolerance->setVisible(!pattern);

	ui.label_alignScore->setVisible(pattern);
	ui.lineEdit_alignPatternScore->setVisible(pattern);
	ui.toolButton_alignLearnPattern->setVisible(pattern);
}

void VisionApp::updateAlignRoiVisibility()
{
	const bool onPage = ui.stackedWidget->currentIndex() == (int)UIPage::LASER;
	const bool pattern = ui.comboBox_alignMethod->currentIndex() == 1;

	if (_alignCircleRoi) _alignCircleRoi->setVisible(onPage && !pattern);
	if (_alignLearnBox) _alignLearnBox->setVisible(onPage && pattern);
	if (_alignSearchBox) _alignSearchBox->setVisible(onPage && pattern);
}

void VisionApp::hideAlignRois()
{
	if (_alignCircleRoi) _alignCircleRoi->hide();
	if (_alignLearnBox) _alignLearnBox->hide();
	if (_alignSearchBox) _alignSearchBox->hide();
	saveAlignmentConfig(); //persist ROI positions when leaving the page
}

void VisionApp::saveAlignmentConfig()
{
	QJsonObject obj;
	obj.insert("method", ui.comboBox_alignMethod->currentIndex());
	obj.insert("tolerance_px", ui.lineEdit_alignTolerance->text().toInt());
	obj.insert("pattern_min_score", ui.lineEdit_alignPatternScore->text().toDouble());

	auto rectToArray = [](const QRectF& r) {
		return QJsonArray{ r.x(), r.y(), r.width(), r.height() };
	};

	if (_alignCircleRoi) obj.insert("circle_roi", rectToArray(_alignCircleRoi->getGeometry()));
	if (_alignLearnBox) obj.insert("learn_roi", rectToArray(_alignLearnBox->getGeometry()));
	if (_alignSearchBox) obj.insert("search_roi", rectToArray(_alignSearchBox->getGeometry()));

	saveJson(kAlignConfigPath, QJsonDocument(obj));
}

void VisionApp::loadAlignmentConfig()
{
	QJsonObject root;
	if (!loadJson(kAlignConfigPath, root)) return;

	auto arrayToRect = [](const QJsonValue& v) {
		auto a = v.toArray();
		if (a.size() != 4) return QRectF();
		return QRectF(a[0].toDouble(), a[1].toDouble(), a[2].toDouble(), a[3].toDouble());
	};

	{
		QSignalBlocker b1(ui.comboBox_alignMethod);
		ui.comboBox_alignMethod->setCurrentIndex(root.value("method").toInt(0));
	}
	ui.lineEdit_alignTolerance->setText(QString::number(root.value("tolerance_px").toInt(30)));
	ui.lineEdit_alignPatternScore->setText(QString::number(root.value("pattern_min_score").toDouble(70.0)));

	auto circleRoi = arrayToRect(root.value("circle_roi"));
	auto learnRoi = arrayToRect(root.value("learn_roi"));
	auto searchRoi = arrayToRect(root.value("search_roi"));

	if (!circleRoi.isEmpty() && _alignCircleRoi) _alignCircleRoi->setGeometry(circleRoi);
	if (!learnRoi.isEmpty() && _alignLearnBox) _alignLearnBox->setGeometry(learnRoi);
	if (!searchRoi.isEmpty() && _alignSearchBox) _alignSearchBox->setGeometry(searchRoi);
}

AlignFeatureParams VisionApp::buildAlignFeatureParams(bool& ok)
{
	ok = true;
	AlignFeatureParams p;
	p.usePattern = ui.comboBox_alignMethod->currentIndex() == 1;

	if (p.usePattern) {
		if (!QFile::exists(kAlignModelPath)) {
			showMsg("No alignment pattern learned yet. Draw the learn ROI over the feature and press Learn Pattern.");
			ok = false;
			return p;
		}
		p.modelPath = kAlignModelPath;
		p.searchRoi = _alignSearchBox ? _alignSearchBox->getGeometry() : QRectF();
		p.minScore = ui.lineEdit_alignPatternScore->text().toDouble();
		if (p.minScore <= 0) p.minScore = 70.0;
	}
	else {
		if (!_alignCircleRoi) { ok = false; return p; }

		//expected diameter comes from the ROI drawn over the circle
		auto r = _alignCircleRoi->getGeometry();
		const int d = (int)((r.width() + r.height()) / 2.0);
		int tol = ui.lineEdit_alignTolerance->text().toInt();
		if (tol <= 0) tol = 30;

		if (d < 10) {
			showMsg("Draw the circle ROI over the alignment circle first.");
			ok = false;
			return p;
		}

		p.minDiameter = std::max(1, d - tol);
		p.maxDiameter = d + tol;
		p.foreground = (ui.comboBox_circleColor->currentText() == "Black Circle")
			? mtrx::ForegoundType::FOREGROUND_BLACK
			: mtrx::ForegoundType::FOREGROUND_WHITE;
	}

	saveAlignmentConfig();
	return p;
}
