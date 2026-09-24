// =============================================================================
//  VisionApp_SafetyCheck.cpp
//  Safety Check page: a go/no-go look for an expected feature before a
//  production run starts. Either colour blobs with size filters, or pattern
//  matching. On this machine it answers "did the trolley dock properly?".
//
//  Modelled on IM420's side alignment, but that one measures an offset and
//  feeds it back into the motion; this only answers "is the feature there?",
//  and refuses to start production when it is not.
// =============================================================================

#include "VisionApp.h"
#include "AuditLog.h"
#include "mtrx.h"
#include "Utilities.h"

#include <QFile>

static const QColor kSafetyRoiColor(0, 200, 0); //green, matching the 3D height ROIs

void VisionApp::initSafetyCheckPage()
{
	_safetyRoiBox = new QDragBox();
	_pGraphicsSceneFOV->addItem(_safetyRoiBox);
	_safetyRoiBox->setOutterBarrier(_pGraphicsSceneFOV->sceneRect());
	_safetyRoiBox->setup(QRectF(200, 200, 400, 300), kSafetyRoiColor, "Safety Check");
	_safetyRoiBox->setDragable(true);
	_safetyRoiBox->setZValue((int)UIHierarchy::DRAGGABLES);
	_safetyRoiBox->hide();

	//the two methods are mutually exclusive - showing both invites a config where it is not
	//clear afterwards which settings actually decided the verdict
	auto applyMethod = [=]() {
		const bool colour = (ui.comboBox_scMethod->currentIndex() == 0);
		ui.frame_scColour->setVisible(colour);
		ui.frame_scPattern->setVisible(!colour);
	};

	auto touched = [=]() { captureSafetyCheckFromUI(); saveRecipeConfig(); };

	connect(ui.comboBox_scMethod, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [=](int) {
		applyMethod();
		touched();
	});
	applyMethod();

	connect(ui.checkBox_scEnabled, &QCheckBox::toggled, this, [=](bool on) {
		touched();
		AuditLog::instance().log(QStringLiteral("SAFETY_CHECK"), on ? QStringLiteral("ON") : QStringLiteral("OFF"));
	});

	for (int i = 0; i < (int)mtrx::Color::SIZE; i++) {
		auto* cb = findChild<QCheckBox*>(QStringLiteral("checkBox_scColor%1").arg(i));
		if (cb) connect(cb, &QCheckBox::toggled, this, [=](bool) { touched(); });
	}
	connect(ui.spinBox_scChroma, QOverload<int>::of(&QSpinBox::valueChanged), this, [=](int) { touched(); });
	connect(ui.spinBox_scMinBlobs, QOverload<int>::of(&QSpinBox::valueChanged), this, [=](int) { touched(); });
	for (auto* cb : { ui.checkBox_scArea, ui.checkBox_scWidth, ui.checkBox_scHeight })
		connect(cb, &QCheckBox::toggled, this, [=](bool) { touched(); });
	for (auto* sp : { ui.dspin_scAreaMin, ui.dspin_scAreaMax, ui.dspin_scWidthMin,
					  ui.dspin_scWidthMax, ui.dspin_scHeightMin, ui.dspin_scHeightMax,
					  ui.dspin_scPatternScore })
		connect(sp, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [=](double) { touched(); });

	//── taught position: the check means nothing unless it runs from a known place
	connect(ui.toolButton_scSetPoint, &QToolButton::clicked, this, [=]() {
		double wx, wy, wz;
		getCurrentPoint(wx, wy, wz);
		_safetyCheck.pointX = wx;
		_safetyCheck.pointY = wy;
		_safetyCheck.pointZ = wz;
		_safetyCheck.pointSet = true;
		refreshSafetyCheckPage();
		saveRecipeConfig();
		AuditLog::instance().log(QStringLiteral("SAFETY_CHECK_SET_POINT"),
			QStringLiteral("%1, %2, %3").arg(wx, 0, 'f', 3).arg(wy, 0, 'f', 3).arg(wz, 0, 'f', 3));
	});

	connect(ui.toolButton_scJogTo, &QToolButton::clicked, this, [=]() {
		if (!_safetyCheck.pointSet) { showMsg("Teach the check position first."); return; }
		emit jogSnap(_safetyCheck.pointX, _safetyCheck.pointY, _safetyCheck.pointZ, _mainOptics[_camID]);
	});

	connect(ui.toolButton_scSnap, &QToolButton::clicked, this, [=]() {
		snapImage(_mainOptics[_camID], "", "");
	});

	connect(ui.toolButton_scShowRoi, &QToolButton::clicked, this, [=]() {
		if (!_safetyRoiBox) return;
		const bool show = !_safetyRoiBox->isVisible();
		if (show && !_safetyCheck.roi.isEmpty()) _safetyRoiBox->setGeometry(_safetyCheck.roi);
		_safetyRoiBox->setVisible(show);
	});
	connect(_safetyRoiBox, SIGNAL(dragBoxMouseReleased(QDragBox*, QString, QPointF)), this, SLOT(safetyRoiTouched()));
	connect(_safetyRoiBox, SIGNAL(grabberReleased(QDragBox*)), this, SLOT(safetyRoiTouched()));

	connect(ui.toolButton_scLearnPattern, &QToolButton::clicked, this, [=]() { learnSafetyPattern(); });

	connect(ui.toolButton_scTest, &QToolButton::clicked, this, [=]() {
		captureSafetyCheckFromUI();
		SafetyCheckResult res;
		runSafetyCheck(_imageFOV, res);
		showSafetyCheckResult(res);
	});

	refreshSafetyCheckPage();
}

void VisionApp::safetyRoiTouched()
{
	captureSafetyCheckFromUI();
	saveRecipeConfig();
}

void VisionApp::showSafetyCheckResult(const SafetyCheckResult& res)
{
	ui.label_scResult->setText(res.message);
	ui.label_scResult->setStyleSheet(res.ok ? QStringLiteral("color:#4CAF50;") : QStringLiteral("color:#E53935;"));
}

void VisionApp::captureSafetyCheckFromUI()
{
	_safetyCheck.enabled = ui.checkBox_scEnabled->isChecked();
	_safetyCheck.method = ui.comboBox_scMethod->currentIndex();

	_safetyCheck.colors.resize((int)mtrx::Color::SIZE);
	for (int i = 0; i < (int)mtrx::Color::SIZE; i++) {
		auto* cb = findChild<QCheckBox*>(QStringLiteral("checkBox_scColor%1").arg(i));
		_safetyCheck.colors[i] = cb && cb->isChecked();
	}

	_safetyCheck.chromaThreshold = ui.spinBox_scChroma->value();
	_safetyCheck.minBlobs = ui.spinBox_scMinBlobs->value();
	_safetyCheck.enableArea = ui.checkBox_scArea->isChecked();
	_safetyCheck.areaMin = ui.dspin_scAreaMin->value();
	_safetyCheck.areaMax = ui.dspin_scAreaMax->value();
	_safetyCheck.enableWidth = ui.checkBox_scWidth->isChecked();
	_safetyCheck.widthMin = ui.dspin_scWidthMin->value();
	_safetyCheck.widthMax = ui.dspin_scWidthMax->value();
	_safetyCheck.enableHeight = ui.checkBox_scHeight->isChecked();
	_safetyCheck.heightMin = ui.dspin_scHeightMin->value();
	_safetyCheck.heightMax = ui.dspin_scHeightMax->value();
	_safetyCheck.patternScore = ui.dspin_scPatternScore->value();

	if (_safetyRoiBox) _safetyCheck.roi = _safetyRoiBox->getGeometry();
}

void VisionApp::refreshSafetyCheckPage()
{
	QSignalBlocker b0(ui.checkBox_scEnabled);
	QSignalBlocker b1(ui.comboBox_scMethod);
	QSignalBlocker b2(ui.spinBox_scChroma);
	QSignalBlocker b3(ui.spinBox_scMinBlobs);
	QSignalBlocker b4(ui.checkBox_scArea);
	QSignalBlocker b5(ui.checkBox_scWidth);
	QSignalBlocker b6(ui.checkBox_scHeight);
	QSignalBlocker b7(ui.dspin_scAreaMin);
	QSignalBlocker b8(ui.dspin_scAreaMax);
	QSignalBlocker b9(ui.dspin_scWidthMin);
	QSignalBlocker b10(ui.dspin_scWidthMax);
	QSignalBlocker b11(ui.dspin_scHeightMin);
	QSignalBlocker b12(ui.dspin_scHeightMax);
	QSignalBlocker b13(ui.dspin_scPatternScore);

	ui.checkBox_scEnabled->setChecked(_safetyCheck.enabled);
	ui.comboBox_scMethod->setCurrentIndex(_safetyCheck.method == 1 ? 1 : 0);
	ui.frame_scColour->setVisible(_safetyCheck.method == 0);
	ui.frame_scPattern->setVisible(_safetyCheck.method == 1);

	_safetyCheck.colors.resize((int)mtrx::Color::SIZE);
	for (int i = 0; i < (int)mtrx::Color::SIZE; i++) {
		auto* cb = findChild<QCheckBox*>(QStringLiteral("checkBox_scColor%1").arg(i));
		if (!cb) continue;
		QSignalBlocker b(cb);
		cb->setChecked(_safetyCheck.colors[i]);
	}

	ui.spinBox_scChroma->setValue(_safetyCheck.chromaThreshold);
	ui.spinBox_scMinBlobs->setValue(std::max(1, _safetyCheck.minBlobs));
	ui.checkBox_scArea->setChecked(_safetyCheck.enableArea);
	ui.dspin_scAreaMin->setValue(_safetyCheck.areaMin);
	ui.dspin_scAreaMax->setValue(_safetyCheck.areaMax);
	ui.checkBox_scWidth->setChecked(_safetyCheck.enableWidth);
	ui.dspin_scWidthMin->setValue(_safetyCheck.widthMin);
	ui.dspin_scWidthMax->setValue(_safetyCheck.widthMax);
	ui.checkBox_scHeight->setChecked(_safetyCheck.enableHeight);
	ui.dspin_scHeightMin->setValue(_safetyCheck.heightMin);
	ui.dspin_scHeightMax->setValue(_safetyCheck.heightMax);
	ui.dspin_scPatternScore->setValue(_safetyCheck.patternScore);

	ui.label_scPoint->setText(_safetyCheck.pointSet
		? QStringLiteral("Point: %1, %2, %3").arg(_safetyCheck.pointX, 0, 'f', 3)
			.arg(_safetyCheck.pointY, 0, 'f', 3).arg(_safetyCheck.pointZ, 0, 'f', 3)
		: QStringLiteral("Point: not set"));

	ui.label_scPatternState->setText(QFile::exists(safetyPatternPath())
		? QStringLiteral("Pattern learnt") : QStringLiteral("No pattern learnt"));

	if (_safetyRoiBox && !_safetyCheck.roi.isEmpty()) _safetyRoiBox->setGeometry(_safetyCheck.roi);
}

QString VisionApp::safetyPatternPath() const
{
	//recipe scoped: the feature belongs to the part being run, not to the machine
	return Common::Directory::getRecipeCurrentPath() + QStringLiteral("safetyCheck.mpat");
}

void VisionApp::learnSafetyPattern()
{
	if (_imageFOV.isNull()) { showMsg("No image to learn from - snap first."); return; }
	captureSafetyCheckFromUI();

	const QRect bounds(0, 0, _imageFOV.width(), _imageFOV.height());
	const QRect roi = _safetyCheck.roi.isEmpty() ? bounds : (_safetyCheck.roi.toRect() & bounds);
	if (roi.width() < 8 || roi.height() < 8) { showMsg("ROI is too small to learn."); return; }

	const QImage crop = _imageFOV.copy(roi).convertToFormat(QImage::Format_Grayscale8);
	cv::Mat gray(crop.height(), crop.width(), CV_8UC1, (void*)crop.bits(), (size_t)crop.bytesPerLine());

	MIL_ID mCrop = M_NULL;
	util::cv_to_Mil(gray.clone(), mCrop);
	if (mCrop == M_NULL) { showMsg("Failed to prepare the learn image."); return; }

	mtrx::PatternInput in;
	in.filename = safetyPatternPath().toStdString();
	in.min_score = 0.0;
	in.learn_x = 0;
	in.learn_y = 0;
	in.learn_w = gray.cols;
	in.learn_h = gray.rows;
	in.smoothness = 85;
	in.enable_angle = false; //a presence check needs no rotation search, and it only costs time

	mtrx::PatternOutput out;
	mtrx::learn_pattern(mCrop, in, out);
	MbufFree(mCrop);

	refreshSafetyCheckPage();
	saveRecipeConfig();
	showStatus("Safety check pattern learnt");
	AuditLog::instance().log(QStringLiteral("SAFETY_CHECK_LEARN"));
}

/*
* The check itself. Returns the verdict rather than acting on it, so the same code answers
* both the operator's Test button and the production gate - a test exercising a different path
* than the run would be worse than no test at all.
*/
bool VisionApp::runSafetyCheck(const QImage& fov, SafetyCheckResult& res)
{
	res = SafetyCheckResult();

	if (fov.isNull()) {
		res.message = QStringLiteral("No image");
		return false;
	}

	const QRect bounds(0, 0, fov.width(), fov.height());
	const QRect roi = _safetyCheck.roi.isEmpty() ? bounds : (_safetyCheck.roi.toRect() & bounds);
	if (roi.width() < 4 || roi.height() < 4) {
		res.message = QStringLiteral("ROI out of image");
		return false;
	}

	const QImage crop = fov.copy(roi);

	if (_safetyCheck.method == 1) {
		//── pattern matching
		if (!QFile::exists(safetyPatternPath())) {
			res.message = QStringLiteral("No pattern learnt");
			return false;
		}

		const QImage gray = crop.convertToFormat(QImage::Format_Grayscale8);
		cv::Mat g(gray.height(), gray.width(), CV_8UC1, (void*)gray.bits(), (size_t)gray.bytesPerLine());
		MIL_ID mMono = M_NULL;
		util::cv_to_Mil(g.clone(), mMono);
		if (mMono == M_NULL) { res.message = QStringLiteral("Image conversion failed"); return false; }

		mtrx::PatternOutput out;
		out.acceptance_min_score = _safetyCheck.patternScore;
		out.certainty_min_score = _safetyCheck.patternScore;
		const bool found = (mtrx::find_pattern(mMono, safetyPatternPath().toStdString(), out) != 0);
		MbufFree(mMono);

		res.ok = found && out.score >= _safetyCheck.patternScore;
		res.found = res.ok ? 1 : 0;
		res.score = found ? out.score : 0.0;
		if (res.ok) res.marks.append(QRectF(roi.x() + out.x, roi.y() + out.y, out.w, out.h));
		res.message = res.ok
			? QStringLiteral("PASS - pattern found, score %1").arg(res.score, 0, 'f', 1)
			: QStringLiteral("FAIL - pattern not found (best %1, need %2)")
				.arg(res.score, 0, 'f', 1).arg(_safetyCheck.patternScore, 0, 'f', 1);
		return res.ok;
	}

	//── colour blobs
	std::set<mtrx::Color> selected;
	for (int i = 0; i < _safetyCheck.colors.size() && i < (int)mtrx::Color::SIZE; i++)
		if (_safetyCheck.colors[i]) selected.insert(static_cast<mtrx::Color>(i));

	if (selected.empty()) {
		res.message = QStringLiteral("No colour selected");
		return false;
	}

	const QImage rgb = crop.convertToFormat(QImage::Format_RGB888);
	cv::Mat rgbMat(rgb.height(), rgb.width(), CV_8UC3, (void*)rgb.bits(), (size_t)rgb.bytesPerLine());
	cv::Mat bgr;
	cv::cvtColor(rgbMat, bgr, cv::COLOR_RGB2BGR);

	MIL_ID mColor = M_NULL;
	util::cv_to_Mil(bgr, mColor);
	if (mColor == M_NULL) { res.message = QStringLiteral("Image conversion failed"); return false; }

	mtrx::ColorBlobParams cb;
	cb.chromaThreshold = _safetyCheck.chromaThreshold;
	cb.color_to_merge.push_back(selected);

	std::vector<std::vector<mtrx::BlobInfo>> blobs;
	mtrx::find_color_blobs(mColor, cb, blobs);
	MbufFree(mColor);

	int passed = 0;
	if (!blobs.empty()) {
		for (const auto& b : blobs[0]) {
			if (_safetyCheck.enableArea && (b.area < _safetyCheck.areaMin || b.area > _safetyCheck.areaMax)) continue;
			if (_safetyCheck.enableWidth && (b.w < _safetyCheck.widthMin || b.w > _safetyCheck.widthMax)) continue;
			if (_safetyCheck.enableHeight && (b.h < _safetyCheck.heightMin || b.h > _safetyCheck.heightMax)) continue;
			passed++;
			if (res.marks.size() < 200) //overlay only - no value in drawing thousands
				res.marks.append(QRectF(roi.x() + b.x, roi.y() + b.y, b.w, b.h));
		}
	}

	res.found = passed;
	res.ok = passed >= std::max(1, _safetyCheck.minBlobs);
	res.message = res.ok
		? QStringLiteral("PASS - %1 blob(s) matched").arg(passed)
		: QStringLiteral("FAIL - %1 blob(s) matched, need %2").arg(passed).arg(std::max(1, _safetyCheck.minBlobs));
	return res.ok;
}

/*
* Production gate: jog to the taught position, grab a fresh image, run the check.
*
* Returns true when production may start - including when the check is switched off, so the
* caller never has to know whether the feature is configured. The failure message names the
* real-world cause rather than the algorithm, because that is what the operator has to fix:
* on this machine the feature being absent means the trolley is not docked.
*/
bool VisionApp::safetyCheckPassedForProduction()
{
	if (!_safetyCheck.enabled) return true;

	if (!_safetyCheck.pointSet) {
		showMsg("Safety check is enabled but its check position has not been taught.\n\n"
			"Teach it on the Safety Check page, or switch the check off.");
		return false;
	}

	ct::logger::info("[SafetyCheck] Jogging to the taught check position");
	_jobThread.jogUser(_safetyCheck.pointX, _safetyCheck.pointY, _safetyCheck.pointZ, "2D", true);
	snapImage(_mainOptics[_camID], "", "");

	//snapImage hands the frame over through the queued imageReady path, so the image this
	//check needs only lands once the event loop has run - wait for it rather than testing
	//whatever happened to be in _imageFOV from before the jog
	QElapsedTimer wait;
	wait.start();
	while (wait.elapsed() < 3000)
		QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents, 50);

	SafetyCheckResult res;
	const bool ok = runSafetyCheck(_imageFOV, res);
	showSafetyCheckResult(res);

	ct::logger::info("[SafetyCheck] %s", res.message.toStdString().c_str());
	AuditLog::instance().log(QStringLiteral("SAFETY_CHECK_RESULT"), res.message);

	if (!ok) {
		addLogLine(QStringLiteral("Safety check FAILED: %1").arg(res.message));
		showMsg(QStringLiteral("Trolley did not dock properly - production will not start.\n\n"
			"The safety check could not find the expected feature at the taught position.\n(%1)")
			.arg(res.message));
	}

	return ok;
}
