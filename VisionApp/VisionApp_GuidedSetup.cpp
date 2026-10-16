// =============================================================================
//  VisionApp_GuidedSetup.cpp
//  Guided Setup page: four fiducials and a unit grid taught by circle finding,
//  each teach auto-centring the camera on the circle nearest the image centre.
//
//  The result is NOT a new kind of recipe. It is written into the fiducials
//  (Circle Finder method), pitch region 1 and the setup-region mode that the
//  pitch-mode production run already reads - this page is a faster way to
//  fill them in, with the machine doing the aiming.
// =============================================================================

#include "VisionApp.h"
#include "AuditLog.h"
#include "mtrx.h"
#include "Utilities.h"
#include "ScaleManager.h"

#include <QTimer>
#include <QDateTime>
#include <QCoreApplication>
#include <cmath>

static const QColor kGuidedRoiColor(255, 150, 40); //reference box: orange, unlike any fiducial box

namespace {
	constexpr int kGuidedFiducials = 4;
	constexpr int kCentreIterations = 10; //each pass is a snap, a find and a jog; the loop leaves as soon as it is centred
	constexpr double kCentredPx = 1.5; //close enough: well under the finder's own scatter
}

/*
* The region the guided teaches act on. It is the SAME selection the Setup Region page uses
* (_pitchRegionSel), so switching regions on either page switches it on both - there is one
* list of regions, and two pages that edit it. A recipe with no region yet gets one here.
*/
int VisionApp::guidedRegion()
{
	auto& sd = SystemData::instance();
	if (sd.pitchRegionCount() == 0) sd.addPitchRegion(SystemData::PitchRegion());
	return std::max(0, std::min(sd._pitchRegionSel.load(), sd.pitchRegionCount() - 1));
}

void VisionApp::initGuidedSetupPage()
{
	_guidedRoiBox = new QDragBox();
	_pGraphicsSceneFOV->addItem(_guidedRoiBox);
	_guidedRoiBox->setOutterBarrier(_pGraphicsSceneFOV->sceneRect());
	_guidedRoiBox->setup(QRectF(1000, 800, 300, 300), kGuidedRoiColor, "Circle Ref");
	_guidedRoiBox->setDragable(true);
	_guidedRoiBox->setZValue((int)UIHierarchy::DRAGGABLES);
	_guidedRoiBox->hide();

	/*
	* The box's size is the point of the box: the operator drags it over a circle and reads the
	* diameter off the label. QDragBox only signals when a grabber is RELEASED, so the label is
	* polled while the box is on screen - 10 Hz is live to the eye and costs nothing.
	*/
	auto* roiTimer = new QTimer(this);
	roiTimer->setInterval(100);
	connect(roiTimer, &QTimer::timeout, this, [=]() {
		if (!_guidedRoiBox || !_guidedRoiBox->isVisible()) return;
		const QRectF g = _guidedRoiBox->getGeometry();
		ui.label_gsRoiSize->setText(QStringLiteral("Box: %1 x %2 px").arg(qRound(g.width())).arg(qRound(g.height())));
	});

	connect(ui.toolButton_gsShowRoi, &QToolButton::clicked, this, [=]() {
		if (!_guidedRoiBox) return;
		const bool show = !_guidedRoiBox->isVisible();
		if (show && !_guidedSetup.roi.isEmpty()) _guidedRoiBox->setGeometry(_guidedSetup.roi);
		_guidedRoiBox->setVisible(show);
		if (show) roiTimer->start(); else roiTimer->stop();
	});
	connect(_guidedRoiBox, SIGNAL(grabberReleased(QDragBox*)), this, SLOT(guidedRoiTouched()));
	connect(_guidedRoiBox, SIGNAL(dragBoxMouseReleased(QDragBox*, QString, QPointF)), this, SLOT(guidedRoiTouched()));

	connect(ui.comboBox_gsRecipeType, &QComboBox::currentTextChanged, this, [=](const QString& t) {
		_guidedSetup.recipeType = t;
		saveRecipeConfig();
	});

	auto spin = [=](QSpinBox* box, int GuidedSetupConfig::* field) {
		connect(box, QOverload<int>::of(&QSpinBox::valueChanged), this, [=](int v) {
			_guidedSetup.*field = v;
			saveRecipeConfig();
		});
	};
	spin(ui.spinBox_gsFidMinDia, &GuidedSetupConfig::fidMinDiaPx);
	spin(ui.spinBox_gsFidMaxDia, &GuidedSetupConfig::fidMaxDiaPx);
	spin(ui.spinBox_gsUnitMinDia, &GuidedSetupConfig::unitMinDiaPx);
	spin(ui.spinBox_gsUnitMaxDia, &GuidedSetupConfig::unitMaxDiaPx);

	for (int i = 0; i < kGuidedFiducials; i++) {
		auto* teach = findChild<QToolButton*>(QStringLiteral("toolButton_gsTeachFid%1").arg(i + 1));
		auto* jogTo = findChild<QToolButton*>(QStringLiteral("toolButton_gsJogFid%1").arg(i + 1));
		if (teach) connect(teach, &QToolButton::clicked, this, [=]() { guidedTeachFiducial(i); });
		if (jogTo) connect(jogTo, &QToolButton::clicked, this, [=]() {
			if (i >= _fiducialInfos.size() || !_fiducialInfos[i].hasTeachPoint) { showMsg("This fiducial has not been taught yet."); return; }
			const auto& p = _fiducialInfos[i].teach_point;
			emit jogSnap(p.wx, p.wy, p.wz, _mainOptics[_camID]);
		});
	}

	//regions: the same list the Setup Region page edits, selected through the same index
	connect(ui.comboBox_gsRegion, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [=](int idx) {
		if (idx < 0) return;
		SystemData::instance()._pitchRegionSel = idx;
		if (_refreshPitchLabels) _refreshPitchLabels();
		refreshGuidedSetupPage();
	});
	connect(ui.toolButton_gsAddRegion, &QToolButton::clicked, this, [=]() {
		auto& sd = SystemData::instance();
		//same seed as the Setup Region page: another array of the same parts, numbered after this one
		const auto cur = sd.pitchRegion(guidedRegion());
		SystemData::PitchRegion fresh;
		fresh.pitchX = cur.pitchX;
		fresh.pitchY = cur.pitchY;
		fresh.unitsX = cur.unitsX;
		fresh.unitsY = cur.unitsY;
		fresh.startX = std::max(1, cur.startX);
		fresh.startY = std::max(1, cur.startY) + std::max(1, cur.unitsY);
		sd._pitchRegionSel = sd.addPitchRegion(fresh);
		if (_refreshPitchLabels) _refreshPitchLabels();
		refreshGuidedSetupPage();
		saveRecipeConfig();
		AuditLog::instance().log(QStringLiteral("PITCH_REGION_ADD"), QStringLiteral("region %1 (guided)").arg(sd._pitchRegionSel.load() + 1));
	});
	connect(ui.toolButton_gsRemoveRegion, &QToolButton::clicked, this, [=]() {
		auto& sd = SystemData::instance();
		const int idx = guidedRegion();
		if (sd.pitchRegionCount() <= 1) { showMsg("At least one region is required."); return; }
		sd.removePitchRegion(idx);
		sd._pitchRegionSel = std::max(0, std::min(idx, sd.pitchRegionCount() - 1));
		{ QSignalBlocker b(ui.comboBox_gsRegion); ui.comboBox_gsRegion->clear(); } //force the rebuild below
		if (_refreshPitchLabels) _refreshPitchLabels();
		refreshGuidedSetupPage();
		saveRecipeConfig();
		AuditLog::instance().log(QStringLiteral("PITCH_REGION_REMOVE"), QStringLiteral("region %1 (guided)").arg(idx + 1));
	});

	//unit counts and numbering edit the selected region directly, like the Setup Region page;
	//the pitch follows the counts when both units are taught
	auto regionEdited = [=](std::function<void(SystemData::PitchRegion&)> apply) {
		auto& sd = SystemData::instance();
		const int idx = guidedRegion();
		auto r = sd.pitchRegion(idx);
		apply(r);
		guidedRecomputePitch(r);
		sd.setPitchRegion(idx, r);
		if (_refreshPitchLabels) _refreshPitchLabels();
		refreshGuidedSetupPage();
		saveRecipeConfig();
	};
	connect(ui.spinBox_gsUnitsX, QOverload<int>::of(&QSpinBox::valueChanged), this, [=](int v) { regionEdited([v](SystemData::PitchRegion& r) { r.unitsX = std::max(1, v); }); });
	connect(ui.spinBox_gsUnitsY, QOverload<int>::of(&QSpinBox::valueChanged), this, [=](int v) { regionEdited([v](SystemData::PitchRegion& r) { r.unitsY = std::max(1, v); }); });
	connect(ui.spinBox_gsStartX, QOverload<int>::of(&QSpinBox::valueChanged), this, [=](int v) { regionEdited([v](SystemData::PitchRegion& r) { r.startX = std::max(1, v); }); });
	connect(ui.spinBox_gsStartY, QOverload<int>::of(&QSpinBox::valueChanged), this, [=](int v) { regionEdited([v](SystemData::PitchRegion& r) { r.startY = std::max(1, v); }); });

	connect(ui.toolButton_gsTeachUnitFirst, &QToolButton::clicked, this, [=]() { guidedTeachUnit(false); });
	connect(ui.toolButton_gsTeachUnitLast, &QToolButton::clicked, this, [=]() { guidedTeachUnit(true); });

	auto jogUnit = [=](bool last) {
		const auto r = SystemData::instance().pitchRegion(guidedRegion());
		if (last ? !r.p2Set : !r.p1Set) { showMsg("This unit has not been taught yet."); return; }
		if (last) emit jogSnap(r.p2x, r.p2y, r.p2z, _mainOptics[_camID]);
		else emit jogSnap(r.p1x, r.p1y, r.p1z, _mainOptics[_camID]);
	};
	connect(ui.toolButton_gsJogUnitFirst, &QToolButton::clicked, this, [=]() { jogUnit(false); });
	connect(ui.toolButton_gsJogUnitLast, &QToolButton::clicked, this, [=]() { jogUnit(true); });

	connect(ui.toolButton_gsTeachFocusZ, &QToolButton::clicked, this, [=]() { guidedTeachFocusZ(); });
	connect(ui.toolButton_gsJogFocusZ, &QToolButton::clicked, this, [=]() {
		if (!_guidedSetup.focusZSet) { showMsg("The focus height has not been taught yet."); return; }
		double x, y, z;
		getCurrentPoint(x, y, z);
		emit jogTo(x, y, _guidedSetup.focusZ, "2D", true);
	});

	connect(ui.toolButton_gsTeachUnitFirstRight, &QToolButton::clicked, this, [=]() { guidedTeachUnitRight(); });
	connect(ui.toolButton_gsJogUnitFirstRight, &QToolButton::clicked, this, [=]() {
		const auto r = SystemData::instance().pitchRegion(guidedRegion());
		if (!r.p1RightSet) { showMsg("The right circle of unit (1,1) has not been taught yet."); return; }
		emit jogSnap(r.p1RightX, r.p1RightY, r.p1z, _mainOptics[_camID]);
	});

	refreshGuidedSetupPage();
}

void VisionApp::guidedRoiTouched()
{
	if (!_guidedRoiBox) return;
	_guidedSetup.roi = _guidedRoiBox->getGeometry();
	saveRecipeConfig();
}

void VisionApp::refreshGuidedSetupPage()
{
	QSignalBlocker b0(ui.comboBox_gsRecipeType);
	QSignalBlocker b1(ui.spinBox_gsFidMinDia);
	QSignalBlocker b2(ui.spinBox_gsFidMaxDia);
	QSignalBlocker b3(ui.spinBox_gsUnitMinDia);
	QSignalBlocker b4(ui.spinBox_gsUnitMaxDia);
	QSignalBlocker b5(ui.spinBox_gsUnitsX);
	QSignalBlocker b6(ui.spinBox_gsUnitsY);
	QSignalBlocker b7(ui.spinBox_gsStartX);
	QSignalBlocker b8(ui.spinBox_gsStartY);

	if (ui.comboBox_gsRecipeType->findText(_guidedSetup.recipeType) >= 0)
		ui.comboBox_gsRecipeType->setCurrentText(_guidedSetup.recipeType);
	ui.spinBox_gsFidMinDia->setValue(_guidedSetup.fidMinDiaPx);
	ui.spinBox_gsFidMaxDia->setValue(_guidedSetup.fidMaxDiaPx);
	ui.spinBox_gsUnitMinDia->setValue(_guidedSetup.unitMinDiaPx);
	ui.spinBox_gsUnitMaxDia->setValue(_guidedSetup.unitMaxDiaPx);

	int taught = 0;
	for (int i = 0; i < kGuidedFiducials; i++) {
		auto* label = findChild<QLabel*>(QStringLiteral("label_gsFid%1").arg(i + 1));
		if (!label) continue;
		const bool has = i < _fiducialInfos.size() && _fiducialInfos[i].hasTeachPoint;
		if (has) {
			taught++;
			const auto& p = _fiducialInfos[i].teach_point;
			label->setText(QStringLiteral("Fiducial %1: %2, %3, %4").arg(i + 1)
				.arg(p.wx, 0, 'f', 3).arg(p.wy, 0, 'f', 3).arg(p.wz, 0, 'f', 3));
		}
		else label->setText(QStringLiteral("Fiducial %1: not taught").arg(i + 1));
	}

	auto& sd = SystemData::instance();
	const int sel = guidedRegion();
	const int regionCount = sd.pitchRegionCount();
	if (ui.comboBox_gsRegion->count() != regionCount) {
		QSignalBlocker b(ui.comboBox_gsRegion);
		ui.comboBox_gsRegion->clear();
		for (int i = 0; i < regionCount; i++) ui.comboBox_gsRegion->addItem(QStringLiteral("Region %1").arg(i + 1));
	}
	if (ui.comboBox_gsRegion->currentIndex() != sel) {
		QSignalBlocker b(ui.comboBox_gsRegion);
		ui.comboBox_gsRegion->setCurrentIndex(sel);
	}
	ui.toolButton_gsRemoveRegion->setEnabled(regionCount > 1);

	const auto r = sd.pitchRegion(sel);
	ui.spinBox_gsUnitsX->setValue(std::max(1, r.unitsX));
	ui.spinBox_gsUnitsY->setValue(std::max(1, r.unitsY));
	ui.spinBox_gsStartX->setValue(std::max(1, r.startX));
	ui.spinBox_gsStartY->setValue(std::max(1, r.startY));
	ui.label_gsUnitFirst->setText(r.p1Set
		? QStringLiteral("Region %1 unit (1,1): %2, %3, %4").arg(sel + 1).arg(r.p1x, 0, 'f', 3).arg(r.p1y, 0, 'f', 3).arg(r.p1z, 0, 'f', 3)
		: QStringLiteral("Region %1 unit (1,1): not taught").arg(sel + 1));
	ui.label_gsUnitLast->setText(r.p2Set
		? QStringLiteral("Unit (%1,%2): %3, %4, %5").arg(std::max(1, r.unitsX)).arg(std::max(1, r.unitsY))
			.arg(r.p2x, 0, 'f', 3).arg(r.p2y, 0, 'f', 3).arg(r.p2z, 0, 'f', 3)
		: QStringLiteral("Unit (%1,%2): not taught").arg(std::max(1, r.unitsX)).arg(std::max(1, r.unitsY)));
	ui.toolButton_gsTeachUnitLast->setText(QStringLiteral("Teach Unit (%1,%2)").arg(std::max(1, r.unitsX)).arg(std::max(1, r.unitsY)));
	ui.label_gsPitch->setText((r.p1Set && r.p2Set)
		? QStringLiteral("Pitch: X %1 mm, Y %2 mm along the board; board angle %3 deg")
			.arg(r.pitchX, 0, 'f', 4).arg(r.pitchY, 0, 'f', 4).arg(r.gridAngleDeg, 0, 'f', 3)
		: (r.p1Set ? QStringLiteral("Pitch: teach the last unit; board angle %1 deg").arg(r.gridAngleDeg, 0, 'f', 3)
			: QStringLiteral("Pitch: teach both units")));

	//the last unit only means something once the first is down and the counts are in
	ui.toolButton_gsTeachUnitLast->setEnabled(r.p1Set);
	ui.toolButton_gsJogUnitLast->setEnabled(r.p2Set);
	ui.label_gsFocusZ->setText(_guidedSetup.focusZSet
		? QStringLiteral("Focus height: Z %1 mm (applied to every view and teach point)").arg(_guidedSetup.focusZ, 0, 'f', 3)
		: QStringLiteral("Focus height: not taught"));
	ui.toolButton_gsJogFocusZ->setEnabled(_guidedSetup.focusZSet);

	ui.toolButton_gsJogUnitFirst->setEnabled(r.p1Set);
	ui.toolButton_gsTeachUnitFirstRight->setEnabled(r.p1Set);
	ui.toolButton_gsJogUnitFirstRight->setEnabled(r.p1RightSet);
	ui.label_gsUnitFirstRight->setText(r.p1RightSet
		? QStringLiteral("Unit (1,1) right circle: %1, %2").arg(r.p1RightX, 0, 'f', 3).arg(r.p1RightY, 0, 'f', 3)
		: QStringLiteral("Unit (1,1) right circle: not taught"));
	if (r.p1RightSet) {
		const double span = std::hypot(r.p1RightX - r.p1x, r.p1RightY - r.p1y);
		ui.label_gsUnitSpan->setText(QStringLiteral("Unit centre: %1, %2 mm from the left circle; circle span %3 mm; 3D scan length %4 mm")
			.arg(r.unitCentreDx, 0, 'f', 3).arg(r.unitCentreDy, 0, 'f', 3).arg(span, 0, 'f', 3)
			.arg(sd._pitchScanLen_mm.load(), 0, 'f', 2));
	}
	else ui.label_gsUnitSpan->setText(QStringLiteral("Unit centre: the left circle (right circle not taught)"));

	//every region counts towards "ready", not only the one on screen
	int regionsReady = 0;
	for (const auto& rr : sd.pitchRegions()) if (rr.p1Set && rr.p2Set) regionsReady++;

	QString summary = QStringLiteral("Fiducials taught: %1 of %2.  ").arg(taught).arg(kGuidedFiducials);
	if (regionsReady == regionCount) summary += QStringLiteral("Regions: %1 of %1 taught - ready for a pitch-mode run.").arg(regionCount);
	else summary += QStringLiteral("Regions: %1 of %2 taught - teach unit (1,1) and the last unit of each.").arg(regionsReady).arg(regionCount);
	if (taught > 0 && taught < 2) summary += QStringLiteral("  At least two fiducials are needed for compensation.");
	ui.label_gsSummary->setText(summary);

	if (_guidedRoiBox && !_guidedSetup.roi.isEmpty()) _guidedRoiBox->setGeometry(_guidedSetup.roi);
}

/*
* A fresh FOV frame, synchronously. snapImage() is a queued hand-off to the job thread and the
* frame comes back through imagePreprocessed(), so this pumps the event loop until the FOV
* serial moves - testing _imageFOV itself would happily return the frame from before the jog.
*/
bool VisionApp::guidedSnapFov(QImage& img, int timeoutMs)
{
	const quint64 before = _fovSerial;
	emit snapImage(_mainOptics[_camID], "", "");

	QElapsedTimer t;
	t.start();
	while (_fovSerial == before && t.elapsed() < timeoutMs)
		QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents, 20);

	if (_fovSerial == before || _imageFOV.isNull()) return false;
	img = _imageFOV;
	return true;
}

/*
* Every circle in the image within the diameter range, and the one nearest the image centre
* wins - "if there are multiple circles, use the one closest to centre, always". The finder
* is the Matrox circle model (M_SHAPE_CIRCLE) the production Circle Finder fiducial uses, so
* what this page centres on is what the run will later find.
*/
bool VisionApp::guidedFindCircleNearCentre(const QImage& fov, int minDiaPx, int maxDiaPx, mtrx::Circle& out, int& count)
{
	count = 0;
	if (fov.isNull() || minDiaPx <= 0 || maxDiaPx < minDiaPx) return false;

	MIL_ID mBuf = mtrx::to_milID(fov);
	MIL_ID mMono = mtrx::to_mono(mBuf);

	std::vector<mtrx::Circle> circles;
	mtrx::find_circles(circles, mMono, minDiaPx / 2.0, maxDiaPx / 2.0, mtrx::FOREGROUND_ANY);

	mtrx::free_buffer(mMono);
	mtrx::free_buffer(mBuf);

	count = (int)circles.size();
	if (circles.empty()) return false;

	const double cx = fov.width() / 2.0, cy = fov.height() / 2.0;
	double best = std::numeric_limits<double>::max();
	for (const auto& c : circles) {
		const double d = std::hypot(c.cx - cx, c.cy - cy);
		if (d < best) { best = d; out = c; }
	}
	return true;
}

/*
* Centre the camera on the nearest circle: snap, find, convert the pixel offset to machine mm
* with the camera scale, jog, and repeat until the circle sits within kCentredPx of the image
* centre. The pixel-to-mm mapping is the one the fiducial compensation uses (located - learnt,
* added to the target), so it should point the right way - but it is checked rather than
* trusted: if a jog makes the offset GROW, the sign is flipped for that axis and the step
* retried, and the flip is logged for whoever wonders why the first move went the wrong way.
*/
bool VisionApp::guidedCentreOnCircle(int minDiaPx, int maxDiaPx, mtrx::Circle& found, QString& why)
{
	double lastDx = 0.0, lastDy = 0.0;
	bool havePrev = false;

	for (int iter = 0; iter < kCentreIterations; iter++) {
		QImage fov;
		if (!guidedSnapFov(fov, 4000)) { why = QStringLiteral("no image from the camera"); return false; }

		int count = 0;
		if (!guidedFindCircleNearCentre(fov, minDiaPx, maxDiaPx, found, count)) {
			why = QStringLiteral("no circle between %1 and %2 px in the image").arg(minDiaPx).arg(maxDiaPx);
			return false;
		}

		const double dx = found.cx - fov.width() / 2.0;
		const double dy = found.cy - fov.height() / 2.0;
		ct::logger::info("[GuidedSetup] Pass %d: %d circle(s), nearest at (%.1f, %.1f) r=%.1f px, off centre by (%.1f, %.1f) px",
			iter + 1, count, found.cx, found.cy, found.radius, dx, dy);

		if (std::abs(dx) <= kCentredPx && std::abs(dy) <= kCentredPx) return true;

		if (havePrev) {
			if (std::abs(dx) > std::abs(lastDx) + kCentredPx) {
				_guidedSignX = -_guidedSignX;
				ct::logger::warn("[GuidedSetup] X offset grew after the jog - X sense flipped to %+d", _guidedSignX);
			}
			if (std::abs(dy) > std::abs(lastDy) + kCentredPx) {
				_guidedSignY = -_guidedSignY;
				ct::logger::warn("[GuidedSetup] Y offset grew after the jog - Y sense flipped to %+d", _guidedSignY);
			}
		}
		lastDx = dx; lastDy = dy; havePrev = true;

		double x, y, z;
		getCurrentPoint(x, y, z);
		const double mx = _guidedSignX * ScaleManager::instance().to_horizontal_mm(dx);
		const double my = _guidedSignY * ScaleManager::instance().to_vertical_mm(dy);
		_jobThread.jogUser(x + mx, y + my, z, "2D", true);
	}

	why = QStringLiteral("the circle would not settle on the image centre after %1 passes").arg(kCentreIterations);
	return false;
}

/*
* Teach fiducial i (0-based) where the camera stands after centring. The fiducial becomes a
* Circle Finder fiducial with this page's diameter range; its inspect region is a box round
* the circle at the image centre and its search region three diameters wide, which is what
* production's locate reads.
*/
void VisionApp::guidedTeachFiducial(int i)
{
	if (i < 0 || i >= kGuidedFiducials) return;
	if (!MotionController::instance().is_init(_motionID)) { showMsg("Motion is not initialised."); return; }

	mtrx::Circle c;
	QString why;
	if (!guidedCentreOnCircle(_guidedSetup.fidMinDiaPx, _guidedSetup.fidMaxDiaPx, c, why)) {
		showMsg(QStringLiteral("Fiducial %1 was not taught.\n\n%2").arg(i + 1).arg(why));
		return;
	}

	//four slots, ids fid1..fid4; a shorter list grows by copying the first entry's settings
	while (_fiducialInfos.size() < kGuidedFiducials) {
		FiducialInfo f = _fiducialInfos.empty() ? FiducialInfo() : _fiducialInfos[0];
		f.id = QStringLiteral("fid%1").arg(_fiducialInfos.size() + 1);
		f.hasTeachPoint = false;
		_fiducialInfos.push_back(f);
	}

	double x, y, z;
	getCurrentPoint(x, y, z);
	const int W = _imageFOV.width(), H = _imageFOV.height();
	const double dia = 2.0 * c.radius;

	auto& f = _fiducialInfos[i];
	f.id = QStringLiteral("fid%1").arg(i + 1);
	f.fiducial_method = 2; //Circle Finder
	f.min_diameter = _guidedSetup.fidMinDiaPx;
	f.max_diameter = _guidedSetup.fidMaxDiaPx;
	f.teach_point.wx = x; f.teach_point.wy = y; f.teach_point.wz = z;
	f.hasTeachPoint = true;

	f.inspect_region.cx = W / 2; f.inspect_region.cy = H / 2;
	f.inspect_region.w = (int)std::min<double>(dia * 1.4, std::min(W, H));
	f.inspect_region.h = f.inspect_region.w;
	f.inspect_region.compute_extremum();

	f.search_region.cx = W / 2; f.search_region.cy = H / 2;
	f.search_region.w = (int)std::min<double>(dia * 3.0, std::min(W, H));
	f.search_region.h = f.search_region.w;
	f.search_region.compute_extremum();

	ui.comboBox_fids->clear();
	for (const auto& fi : _fiducialInfos) ui.comboBox_fids->addItem(fi.id);

	//the layout is compensated from all four corners at once - not two islands of two
	{
		QSignalBlocker b(ui.checkBox_doubleFiducialChecking);
		ui.checkBox_doubleFiducialChecking->setChecked(false);
	}
	SystemData::instance()._doubleFiducialChecking = false;
	SystemData::instance()._fourPointFiducial = true;
	enableFiducial(true);

	saveFiducial(); //bumps the teach revision the regions compare against
	saveRecipeConfig();
	refreshGuidedSetupPage();
	showStatus(QStringLiteral("Fiducial %1 taught at %2, %3").arg(i + 1).arg(x, 0, 'f', 3).arg(y, 0, 'f', 3));
	AuditLog::instance().log(QStringLiteral("GUIDED_TEACH_FID"),
		QStringLiteral("fid%1 %2, %3, %4 dia %5 px").arg(i + 1).arg(x, 0, 'f', 3).arg(y, 0, 'f', 3).arg(z, 0, 'f', 3).arg(dia, 0, 'f', 1));
}

/*
* The board's rotation, read off the fiducials. The four fiducials sit on the corners of a
* rectangle whose sides run along the unit rows and columns, so each side's direction, folded
* into (-45, 45] degrees, is the board angle - and folding also makes the result independent
* of which fiducial got which number. The taught points are ordered round their centroid so
* consecutive pairs are SIDES, not diagonals; the four side angles are averaged as 4x phasors,
* which is the mean that does not break at the +/-45 wrap. Three fiducials still give two
* sides. Two give one segment that may be a diagonal, so that case is refused.
*/
bool VisionApp::guidedBoardAngleDeg(double& deg, QString& why)
{
	std::vector<QPointF> pts;
	for (int i = 0; i < 4 && i < _fiducialInfos.size(); i++)
		if (_fiducialInfos[i].hasTeachPoint) pts.emplace_back(_fiducialInfos[i].teach_point.wx, _fiducialInfos[i].teach_point.wy);

	if (pts.size() < 3) {
		why = QStringLiteral("at least three fiducials must be taught to read the board angle (%1 taught)").arg((int)pts.size());
		return false;
	}

	QPointF c(0, 0);
	for (const auto& p : pts) c += p;
	c /= (double)pts.size();
	std::sort(pts.begin(), pts.end(), [&](const QPointF& a, const QPointF& b) {
		return std::atan2(a.y() - c.y(), a.x() - c.x()) < std::atan2(b.y() - c.y(), b.x() - c.x());
	});

	//consecutive points are rectangle sides; with three points the closing pair is a diagonal, so skip it
	const int sides = (pts.size() == 4) ? 4 : 2;
	double sx = 0, sy = 0;
	for (int i = 0; i < sides; i++) {
		const QPointF a = pts[i], b = pts[(i + 1) % pts.size()];
		const double ang = std::atan2(b.y() - a.y(), b.x() - a.x());
		sx += std::cos(4 * ang); sy += std::sin(4 * ang); //fold to the 90-degree symmetry
	}
	deg = std::atan2(sy, sx) / 4.0 * 180.0 / 3.14159265358979; //in (-45, 45]
	return true;
}

/*
* The right circle of unit (1,1). Centred on like everything else; from it and the taught left
* circle come two things the run needs: the unit CENTRE, as the midpoint (stored as an offset
* from the left circle and added to every unit of the region), and the unit's WIDTH, which sets
* the 3D scan length to span + one circle diameter + 2 mm so both circles are inside the scan.
*/
void VisionApp::guidedTeachUnitRight()
{
	if (!MotionController::instance().is_init(_motionID)) { showMsg("Motion is not initialised."); return; }

	auto& sd = SystemData::instance();
	const int idx = guidedRegion();
	auto r = sd.pitchRegion(idx);
	if (!r.p1Set) { showMsg("Teach the left circle of unit (1,1) first."); return; }

	mtrx::Circle c;
	QString why;
	if (!guidedCentreOnCircle(_guidedSetup.unitMinDiaPx, _guidedSetup.unitMaxDiaPx, c, why)) {
		showMsg(QStringLiteral("The right circle was not taught.\n\n%1").arg(why));
		return;
	}

	double x, y, z;
	getCurrentPoint(x, y, z);
	r.p1RightX = x; r.p1RightY = y;
	r.p1RightSet = true;
	r.unitCentreDx = (x - r.p1x) / 2.0;
	r.unitCentreDy = (y - r.p1y) / 2.0;
	sd.setPitchRegion(idx, r);

	const double span = std::hypot(x - r.p1x, y - r.p1y);
	const double diaMm = ScaleManager::instance().to_horizontal_mm(2.0 * c.radius);
	const double scanLen = span + diaMm + 2.0;
	sd._pitchScanLen_mm = scanLen;
	{
		QSignalBlocker b(ui.lineEdit_pitchScanLen);
		ui.lineEdit_pitchScanLen->setText(QString::number(scanLen, 'f', 2));
	}
	ct::logger::info("[GuidedSetup] Unit (1,1) right circle at %.3f, %.3f: centre offset %.3f/%.3f mm, span %.3f mm, circle %.3f mm -> 3D scan length %.2f mm",
		x, y, r.unitCentreDx, r.unitCentreDy, span, diaMm, scanLen);

	if (_refreshPitchLabels) _refreshPitchLabels();
	saveRecipeConfig();
	refreshGuidedSetupPage();
	showStatus(QStringLiteral("Right circle taught - 3D scan length %1 mm").arg(scanLen, 0, 'f', 2));
	AuditLog::instance().log(QStringLiteral("GUIDED_TEACH_UNIT_RIGHT"),
		QStringLiteral("region %1: %2, %3 span %4 scan %5").arg(idx + 1).arg(x, 0, 'f', 3).arg(y, 0, 'f', 3).arg(span, 0, 'f', 3).arg(scanLen, 0, 'f', 2));
}

/*
* The 2D camera's in-focus height. The operator focuses by eye (or with the live view), presses,
* and the current Z becomes THE Z of the recipe: applyViewZ() writes it into the plane, every
* view, every line scan, every region's taught points and every fiducial and barcode teach
* point, so nothing in the run is left at whatever height it happened to be taught at.
*/
void VisionApp::guidedTeachFocusZ()
{
	double x, y, z;
	getCurrentPoint(x, y, z);

	_guidedSetup.focusZSet = true;
	_guidedSetup.focusZ = z;
	applyViewZ(z); //saves the recipe files and recipeConfig, which carries _guidedSetup too

	refreshGuidedSetupPage();
	showStatus(QStringLiteral("Focus height %1 mm applied to every view and teach point").arg(z, 0, 'f', 3));
	AuditLog::instance().log(QStringLiteral("GUIDED_TEACH_FOCUS_Z"), QStringLiteral("z=%1").arg(z, 0, 'f', 3));
}

//pitch from the FAR corner, measured along the BOARD's axes: the span is rotated back by the
//board angle before it is divided over the unit count, so a tilted board does not leak Y pitch
//into X. The circle-centre error is divided over the whole board instead of one spacing.
void VisionApp::guidedRecomputePitch(SystemData::PitchRegion& r)
{
	if (!r.p1Set || !r.p2Set) return;
	const double ga = r.gridAngleDeg * 3.14159265358979 / 180.0;
	const double dx = r.p2x - r.p1x, dy = r.p2y - r.p1y;
	const double sxb = std::cos(ga) * dx + std::sin(ga) * dy;
	const double syb = -std::sin(ga) * dx + std::cos(ga) * dy;
	r.pitchX = (r.unitsX > 1) ? sxb / (r.unitsX - 1) : 0.0;
	r.pitchY = (r.unitsY > 1) ? syb / (r.unitsY - 1) : 0.0;
}

/*
* Teach unit (1,1) or the last unit (unitsX, unitsY) of region 1 where the camera stands after
* centring on the unit's circle. Teaching (1,1) also captures the fiducial reference: the four
* taught fiducial positions, which is where the fiducials ARE on this board right now - the
* run-time compensation is measured against exactly these.
*/
void VisionApp::guidedTeachUnit(bool last)
{
	if (!MotionController::instance().is_init(_motionID)) { showMsg("Motion is not initialised."); return; }

	auto& sd = SystemData::instance();
	const int idx = guidedRegion();
	auto r = sd.pitchRegion(idx);
	r.unitsX = std::max(1, ui.spinBox_gsUnitsX->value());
	r.unitsY = std::max(1, ui.spinBox_gsUnitsY->value());

	if (last && !r.p1Set) { showMsg("Teach unit (1,1) first."); return; }
	if (last && r.unitsX == 1 && r.unitsY == 1) { showMsg("A 1 x 1 layout has no pitch to teach - unit (1,1) is the whole board."); return; }

	mtrx::Circle c;
	QString why;
	if (!guidedCentreOnCircle(_guidedSetup.unitMinDiaPx, _guidedSetup.unitMaxDiaPx, c, why)) {
		showMsg(QStringLiteral("The unit was not taught.\n\n%1").arg(why));
		return;
	}

	double x, y, z;
	getCurrentPoint(x, y, z);
	const qint64 now = QDateTime::currentMSecsSinceEpoch();

	if (!last) {
		r.p1x = x; r.p1y = y; r.p1z = z;
		r.p1Set = true;
		r.p1TeachMs = now;

		//the right circle was measured against the OLD left circle - it has to be taught again
		if (r.p1RightSet) ct::logger::info("[GuidedSetup] Left circle re-taught - the right circle teach is cleared");
		r.p1RightSet = false;
		r.unitCentreDx = r.unitCentreDy = 0.0;

		int mask = 0;
		const double* refx[4] = { &r.fidRef1x, &r.fidRef2x, &r.fidRef3x, &r.fidRef4x };
		const double* refy[4] = { &r.fidRef1y, &r.fidRef2y, &r.fidRef3y, &r.fidRef4y };
		for (int i = 0; i < kGuidedFiducials && i < _fiducialInfos.size(); i++) {
			if (!_fiducialInfos[i].hasTeachPoint) continue;
			*const_cast<double*>(refx[i]) = _fiducialInfos[i].teach_point.wx;
			*const_cast<double*>(refy[i]) = _fiducialInfos[i].teach_point.wy;
			mask |= (1 << i);
		}
		r.fidRefMask = mask;
		r.fidRefTeachRev = sd._fiducialTeachRev;
		if (mask == 0) ct::logger::warn("[GuidedSetup] Unit (1,1) taught with NO fiducials taught - teach the fiducials, then teach unit (1,1) again");

		//the grid runs along the board, and the board's angle comes from the fiducial rectangle
		double deg = 0.0;
		QString angleWhy;
		if (guidedBoardAngleDeg(deg, angleWhy)) {
			r.gridAngleDeg = deg;
			ct::logger::info("[GuidedSetup] Board angle from the fiducials: %.4f deg - the unit grid is laid out along it", deg);
		}
		else {
			r.gridAngleDeg = 0.0;
			ct::logger::warn("[GuidedSetup] Grid laid out along the machine axes: %s", angleWhy.toStdString().c_str());
			showMsg(QStringLiteral("Unit (1,1) stored, but the board angle could not be read: %1.\n\n"
				"The grid is laid out along the machine axes for now. Teach the fiducials, then teach unit (1,1) again.").arg(angleWhy));
		}
	}
	else {
		r.p2x = x; r.p2y = y; r.p2z = z;
		r.p2Set = true;
		r.p2TeachMs = now;
	}
	guidedRecomputePitch(r);
	sd.setPitchRegion(idx, r);

	//this is the layout production runs: pitch mode, from here on
	sd._setupRegionPitchMode = true;
	{
		QSignalBlocker b(ui.comboBox_setupRegionMode);
		ui.comboBox_setupRegionMode->setCurrentIndex(1);
	}
	if (_applySetupRegionMode) _applySetupRegionMode();
	if (_refreshPitchLabels) _refreshPitchLabels();

	saveRecipeConfig();
	refreshGuidedSetupPage();
	showStatus(last ? QStringLiteral("Last unit taught") : QStringLiteral("Unit (1,1) taught"));
	AuditLog::instance().log(last ? QStringLiteral("GUIDED_TEACH_UNIT_LAST") : QStringLiteral("GUIDED_TEACH_UNIT_FIRST"),
		QStringLiteral("region %1: %2, %3, %4 units %5x%6").arg(idx + 1).arg(x, 0, 'f', 3).arg(y, 0, 'f', 3).arg(z, 0, 'f', 3).arg(r.unitsX).arg(r.unitsY));
}
