//
//  Guided Setup (new): the glue between VisionApp and the new page / layout view.
//
//  Built alongside the old Guided Setup, which it never touches: its own right-panel page (added
//  to stackedWidget in code, so VisionApp.ui is not edited), its own layout view in the workspace
//  stack, and its own recipe file guidedSetup2.json. Build step 1 = the page, Save / Discard and
//  the layout view. Build step 2 = the circle finder and the camera direction check, which move
//  the machine the way a Z search does: zsMove, one modal busy window with a Stop, START refused.
//  Build step 3 = teaching sections A-D (Safety check 1, fid0, 3D Z, the four corners) and the
//  Locate test, which runs START steps 1-5. Build step 4a = sections E (slot size), F (slots) and
//  H's Set, all taught in the trolley frame a Check or Locate found in this session.
//
#include "VisionApp.h"
#include "GuidedSetup2Page.h"
#include "GuidedSetup2View.h"
#include "MotionController.h"
#include "MachineController.h"
#include "ScaleManager.h"
#include "CAMManager.h"
#include "AuditLog.h"
#include "Logger.h"
#include "mtrx.h"

#include <QCoreApplication>
#include <QDateTime>
#include <QDialog>
#include <QElapsedTimer>
#include <QFile>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QTimer>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace {
	constexpr int kSnapTimeoutMs = 4000;
	constexpr double kCentreTolMm = 0.1;     //Centre on circle: done once the circle is this close to the image centre
	constexpr int kCentreMoves = 3;          //a move lands within 0.05 mm, so one is normally enough
	//direction check: +/- this on each axis, about 180 px at 27 um/px. Not more: the trolley's circles
	//sit ~12.7 mm apart, and at the diagonal corners of a bigger grid a neighbour is nearer the image
	//centre than the circle being measured
	constexpr double kDirStepMm = 5.0;
	constexpr double kDirMaxScalePct = 0.3;  //0.3 % is 0.05 mm at a quarter of the image from the centre
	//0.3 deg is 0.09 mm there. The Y axis's rotation comes out -0.34 .. -0.53 deg run to run (IPC
	//2026-10-30 and 2026-11-03; X's agrees within 0.01): X does not hold still while Y moves. At 0.15
	//a re-check failed the measurement it had just saved
	constexpr double kDirMaxAngleDeg = 0.3;
	//a point further than this from the fitted matrix: something moved, or another circle was picked
	//(those give 3-4 px and tens of px). Settled IPC checks (2026-10-30 16:31/16:33) sit at 0.4-0.5 px rms,
	//0.73-0.97 max, in the same pattern twice: the head and the encoders differ by ~20 um from move to
	//move (Y read +0.026 at every grid point while the image saw ~0.007), which no matrix can take out
	constexpr double kDirMaxFitPx = 1.5;
	constexpr double kStillPx = 0.25;        //three snaps in a row this close = the head has stopped moving
	constexpr double kStillStageMm = 0.003;  //and the encoders agree too (Y creeps for ~0.7 s after "done")
	//Once the encoders have been still this long, what the circle still does is the finder's own jitter, not
	//the head: the IPC (2026-10-30 20:05-20:12) failed three Finds with the encoders at 0.000 and the circle
	//jumping 0.27-0.74 px between snaps, after the finder settled on a weaker edge (160 px across, score ~81).
	//Then the last kQuietSnaps are taken, averaged, if every one is within kJitterPx (27 um) of their mean.
	//A stray snap among steady ones (1.5-2.4 px, IPC 2026-10-30 20:53-20:58) must not fail the lot: the
	//median of the five is the reference and at least kQuietAgree within kJitterPx of it are averaged.
	constexpr int kQuietMs = 1500;
	constexpr int kQuietSnaps = 5;
	constexpr int kQuietAgree = 3;
	constexpr double kJitterPx = 1.0;
	//A circle fitted worse than this is no measurement: good fits are 0.15-0.27 px, the stray ones on the
	//IPC (2026-11-03) 10-17 px. A snap like that, or one with no circle at all, is snapped again
	constexpr double kMaxFitErrPx = 2.0;
	constexpr int kStabilitySnaps = 10;      //Stability test
	constexpr int kStillMaxMs = 6000;        //the IPC took 0.8-4.7 s
	constexpr int kStillGapMs = 50;          //between snaps, so a Stop gets through
	constexpr double kExpectTolMm = 1.0;     //fid0 must be this close to where the pattern puts it; its neighbours are ~12.7 mm away
	constexpr double kTeachZTolMm = 0.3;     //corners are taught at fid0's 2D Z or up to this above it: in focus, as Check and START see them
	constexpr double kTeachZBelowMm = 0.02;  //and below it only by what a Z move lands within (the Z search's on-target)
	constexpr double kRowYTolMm = 1.0;       //F's Interpolate: the 2 end slots of a row line up this well in Y (trolley frame)
	constexpr double kScanOvershootMm = 15.0; //production's run past a 3D scan's end (JobThread::kScanOvershootMm, private there)
	constexpr int kVisitDwellMs = 300;       //Visit 2D: how long each snap stays before the next move
	//C: the profiler reading moves this many mm per mm of Z down (+ Z is down; the IPC measured +0.976 on
	//27 Oct 2026 - head down = reading up), so one read at the 2D Z says where the 3D Centre Z is
	constexpr double kReadingPerZ = 1.0;
	const char* const kCornerName[4] = { "fidTL", "fidTR", "fidBR", "fidBL" };   //gs2geo::Corner order
	const double kPi = 3.14159265358979323846;

	QString mm(double v) { return QString::number(v, 'f', 3); }
	QString signedMm(double v) { return (v >= 0.0 ? QStringLiteral("+") : QString()) + QString::number(v, 'f', 3); }
	QString signedNum(double v, int decimals) { return (v >= 0.0 ? QStringLiteral("+") : QString()) + QString::number(v, 'f', decimals); }
	QString badFit(double diaPx, double fitErr)
	{
		return QStringLiteral("a bad fit: %1 px across, edges %2 px off the circle").arg(diaPx, 0, 'f', 1).arg(fitErr, 0, 'f', 1);
	}

	//image px per stage mm (how a fixed circle moves when the stage moves) from world mm per image px
	bool pxPerMm(const double a[2][2], double e[2][2])
	{
		const double det = a[0][0] * a[1][1] - a[0][1] * a[1][0];
		if (std::abs(det) < 1e-12) return false;
		//e = -a^-1: the camera moving onto a point the circle shows at offset d brings the circle to the centre
		e[0][0] = -a[1][1] / det; e[0][1] = a[0][1] / det;
		e[1][0] = a[1][0] / det;  e[1][1] = -a[0][0] / det;
		return true;
	}

	//to_milID takes 8-bit grey as it is and colour as packed 32-bit; a 24- or 16-bit file from Load FOV
	//would be read as the wrong layout, so it is converted first
	QImage milReady(const QImage& image)
	{
		if (image.isGrayscale()) return image.depth() == 8 ? image : image.convertToFormat(QImage::Format_Grayscale8);
		return image.depth() == 32 ? image : image.convertToFormat(QImage::Format_RGB32);
	}

	//Safety check 1 compares small grey images: an eighth of the camera image, which is out of focus at home anyway
	constexpr int kSafetyShrink = 8;
	constexpr int kHomeSettleMs = 800;       //the head sways after the long move home; the image is not judged until it stops
	constexpr double kPatternAcceptance = 20.0; //matches are reported from this score (0..100); the recipe minimum decides
	constexpr int kMaxPatternHits = 16;         //copies of the pattern looked at in one image

	//shrink THEN grey: a smooth scale hands back 32-bit colour whatever it was given (Qt 5.9), so grey
	//first and scale second gave a 4-byte image the comparison read as grey - 2026-10-30 IPC: an
	//empty table scored ~50 against the empty images
	QImage safetySmall(const QImage& img)
	{
		return img.scaled(std::max(1, img.width() / kSafetyShrink), std::max(1, img.height() / kSafetyShrink), Qt::IgnoreAspectRatio, Qt::SmoothTransformation)
			.convertToFormat(QImage::Format_Grayscale8);
	}

	//mean grey-level difference per pixel (0..255), 0 = identical; -1 = the sizes differ
	double meanDifference(const QImage& imageA, const QImage& imageB)
	{
		//one byte per pixel whatever came in, or the loop below reads colour bytes as grey
		const QImage a = imageA.convertToFormat(QImage::Format_Grayscale8);
		const QImage b = imageB.convertToFormat(QImage::Format_Grayscale8);
		if (a.size() != b.size() || a.isNull()) return -1.0;
		double sum = 0.0;
		for (int y = 0; y < a.height(); y++) {
			const uchar* pa = a.constScanLine(y);
			const uchar* pb = b.constScanLine(y);
			for (int x = 0; x < a.width(); x++) sum += std::abs(int(pa[x]) - int(pb[x]));
		}
		return sum / (double(a.width()) * a.height());
	}

	//the busy window's progress line (see VisionApp::showBusyWindow)
	void setProgress(QDialog* window, const QString& text)
	{
		if (!window) return;
		if (auto* label = window->findChild<QLabel*>(QStringLiteral("busyProgress"))) label->setText(text);
	}

	//each section's actions report on that section's result line
	void showTeachResult(GuidedSetup2Page* page, int action, const QString& text, bool ok)
	{
		using P = GuidedSetup2Page;
		switch (action) {
		case P::TeachSnapEmpty: case P::TeachScore:
			page->showSafetyScore(text, ok); break;
		case P::TeachZ3DSpot: case P::TeachZ3DFind: case P::TeachZ3DDiff:
			page->showZ3DResult(text, ok); break;
		case P::TeachCornerTL: case P::TeachCornerTR: case P::TeachCornerBR: case P::TeachCornerBL: case P::TeachRectCheck:
		case P::TeachJogCornerTL: case P::TeachJogCornerTR: case P::TeachJogCornerBR: case P::TeachJogCornerBL:
			page->showRectResult(text, ok); break;
		case P::TeachSlotTL: case P::TeachSlotBR:
			page->showSlotSizeResult(text, ok); break;
		case P::TeachSlotAdd: case P::TeachSlotInterpolate:
			page->showSlotsResult(text, ok); break;
		case P::TeachInsp2D: case P::TeachInspBarcodeOcr: case P::TeachInsp3D:
			page->showInspResult(text, ok); break;
		case P::TeachLocate: case P::TeachMoveOnly: case P::TeachGoTo: case P::TeachSnapScan:
			page->showLocateResult(text, ok); break;
		default:
			page->showFid0Result(text, ok); break;
		}
	}

	//everything a section needs before its Find or Check can run: done, but for that run's result
	bool readyToRun(gs2::Setup s, gs2::Section sec, QString& why)
	{
		s.fid0.found.ok = s.z3d.found.ok = s.fidRect.found.ok = true;
		return gs2::sectionDone(s, sec, &why);
	}

	//where the laser reads C's spot, as the camera point zsRead3DAt / zsCheck3DRead take (they put the stage at it + the
	//camera-to-laser XY): the spot itself, or with "Add the camera-to-laser XY offset" off, wherever the laser falls
	//with the stage at fid0 + the spot offset
	QPointF z3dReadPoint(const gs2::Setup& s, double fid0X, double fid0Y, const dat::WorldCoordinate& laser)
	{
		const double x = fid0X + s.z3d.offset.x, y = fid0Y + s.z3d.offset.y;
		return s.z3d.addLaserOffset ? QPointF(x, y) : QPointF(x - laser.wx, y - laser.wy);
	}

	/*
	* Where the next corner should be, from fid0 (found now) and the corners found before it: the rigid
	* move (shift + turn) that best takes their taught spots onto where they were found, applied to the
	* next corner's taught spot. fid0 is a pair of its own (taught = found: it was just located), so the
	* trolley's turn is known from the first corner on and the later corners land near the image centre.
	*/
	gs2geo::Pt predictCorner(gs2geo::Pt fid0, const std::vector<gs2geo::Pt>& taughtDone, const std::vector<gs2geo::Pt>& foundDone, gs2geo::Pt next)
	{
		std::vector<gs2geo::Pt> a{ fid0 }, b{ fid0 };
		a.insert(a.end(), taughtDone.begin(), taughtDone.end());
		b.insert(b.end(), foundDone.begin(), foundDone.end());
		const double n = (double)a.size();
		gs2geo::Pt ca, cb;
		for (size_t k = 0; k < a.size(); k++) { ca.x += a[k].x / n; ca.y += a[k].y / n; cb.x += b[k].x / n; cb.y += b[k].y / n; }
		double sDot = 0.0, sCross = 0.0;
		for (size_t k = 0; k < a.size(); k++) {
			const double ax = a[k].x - ca.x, ay = a[k].y - ca.y, bx = b[k].x - cb.x, by = b[k].y - cb.y;
			sDot += ax * bx + ay * by;
			sCross += ax * by - ay * bx;
		}
		const double t = (sDot == 0.0 && sCross == 0.0) ? 0.0 : std::atan2(sCross, sDot);
		const double dx = next.x - ca.x, dy = next.y - ca.y;
		return { cb.x + std::cos(t) * dx - std::sin(t) * dy, cb.y + std::sin(t) * dx + std::cos(t) * dy };
	}
}

void VisionApp::initGuidedSetup2()
{
	_gs2Page = new GuidedSetup2Page();

	//the right tab is chosen by index == UIPage value, so the page has to land exactly there
	const int index = (int)UIPage::GUIDED_SETUP2;
	while (ui.stackedWidget->count() < index) ui.stackedWidget->addWidget(new QWidget()); //PMF has no right page
	if (ui.stackedWidget->count() == index) {
		ui.stackedWidget->addWidget(_gs2Page);
	}
	else {
		ct::logger::error("[GuidedSetup2] stackedWidget already has %d pages; the page is inserted at %d and later pages shift",
			ui.stackedWidget->count(), index);
		ui.stackedWidget->insertWidget(index, _gs2Page);
	}

	_gs2View = new GuidedSetup2View();
	_gs2ViewIndex = ui.stackedWidgetViewSelection->addWidget(_gs2View);
	//a trolley taken off or put on is no longer where a Check or Locate found it
	connect(&MachineController::instance(), &MachineController::signalTrolleyGuard, this, [=](bool) {
		if (_gs2View) _gs2View->clearLocated();
	}, Qt::QueuedConnection);

	_gs2CameraTimer = new QTimer(this);
	_gs2CameraTimer->setInterval(200);
	connect(_gs2CameraTimer, &QTimer::timeout, this, [=]() { gs2RefreshCamera(); });

	connect(_gs2Page, &GuidedSetup2Page::setupChanged, this, [=]() {
		if (!_gs2View || !_gs2Page) return;
		_gs2View->setLaserOffset(_laserConfig.offset.wx, _laserConfig.offset.wy);   //3D scans are drawn where the laser measures
		_gs2View->setSetup(_gs2Page->setup());
	});
	connect(_gs2Page, &GuidedSetup2Page::showLayoutToggled, this, [=](bool on) { gs2ShowLayout(on); });
	connect(_gs2Page, &GuidedSetup2Page::toolRequested, this, [=](int action) { gs2RunTool(action); });
	connect(_gs2Page, &GuidedSetup2Page::conversionChosen, this, [=](bool measured) { gs2ChooseConversion(measured); });
	connect(_gs2Page, &GuidedSetup2Page::teachRequested, this, [=](int action) { gs2Teach(action); });
	connect(_gs2Page, &GuidedSetup2Page::patternBoxToggled, this, [=](bool on) { gs2ShowPatternBox(on); });
	//one slot selection: F's list and the layout view follow each other
	connect(_gs2Page, &GuidedSetup2Page::slotSelectionChanged, this, [=](const QStringList& ids) { if (_gs2View) _gs2View->setSelectedSlots(ids); });
	connect(_gs2View, &GuidedSetup2View::slotsPicked, this, [=](const QStringList& ids) { if (_gs2Page) _gs2Page->selectSlots(ids); });
	gs2LoadCameraCal();
	gs2RefreshCalUi();

	//the fid0 pattern's learn box, on the camera image like every other learn box in the app
	_gs2PatternBox = new QDragBox();
	_pGraphicsSceneFOV->addItem(_gs2PatternBox);
	_gs2PatternBox->setOutterBarrier(_pGraphicsSceneFOV->sceneRect());
	_gs2PatternBox->setup(QRectF(924, 724, 600, 600), QColor(80, 170, 255), "fid0 pattern");
	_gs2PatternBox->setDragable(true);
	_gs2PatternBox->setZValue((int)UIHierarchy::DRAGGABLES);
	_gs2PatternBox->hide();

	//the layout can be reached from the World button or the page's Show layout button, and left by
	//the FOV / Dual buttons or another page: whichever way, the page button and the camera refresh
	//follow what the big image area actually shows
	_gs2ViewConn = connect(ui.stackedWidgetViewSelection, &QStackedWidget::currentChanged, this, [=](int i) {
		if (!_gs2Page) return;
		const bool shown = i == _gs2ViewIndex;
		_gs2Page->setLayoutShown(shown);
		if (shown) {
			gs2RefreshWorld();
			gs2RefreshFov();
			_gs2CameraTimer->start();
		}
		else {
			_gs2CameraTimer->stop();
		}
	});
}

void VisionApp::gs2PageOpened()
{
	if (!_gs2Page) return;
	//a recipe opened by a route that skipped gs2RecipeOpened() still shows up here
	if (_gs2Page->recipeDir() != Common::Directory::getRecipeCurrentPath()) {
		_gs2Page->confirmLeave();
		gs2RecipeOpened();
	}
	_gs2Open = true;
	_gs2Page->setLayoutShown(ui.stackedWidgetViewSelection->currentIndex() == _gs2ViewIndex);
	_gs2Page->refreshScale();
	gs2RefreshCalUi(); //Camera Scaling or Alignment may have been re-run since
	gs2RefreshWorld();
}

void VisionApp::gs2PageLeft()
{
	if (!_gs2Page || !_gs2Open) return;
	_gs2Open = false;
	gs2ShowPatternBox(false); //the learn box belongs to this page
	//the layout stays up if it is showing: it belongs to the World button now, not only to this page
	_gs2Page->confirmLeave();
}

void VisionApp::gs2ShowLayout(bool shown)
{
	if (!_gs2View) return;
	if (shown) gs2ShowLayoutView();
	else toggleFOVView();
}

void VisionApp::gs2ShowLayoutView()
{
	if (!_gs2View) return;
	QSignalBlocker sb1(ui.toolButton_toggleDualView);
	QSignalBlocker sb2(ui.toolButton_toggleWorldView);
	QSignalBlocker sb3(ui.toolButton_toggleFovView);
	ui.toolButton_toggleDualView->setChecked(false);
	ui.toolButton_toggleWorldView->setChecked(true);
	ui.toolButton_toggleFovView->setChecked(false);
	gs2RefreshWorld();
	_gs2View->setLaserOffset(_laserConfig.offset.wx, _laserConfig.offset.wy);   //an alignment may have changed it meanwhile
	ui.stackedWidgetViewSelection->setCurrentIndex(_gs2ViewIndex);
}

void VisionApp::gs2RefreshWorld()
{
	if (!_gs2View) return;
	double xLo = 0.0, xHi = 0.0, yLo = 0.0, yHi = 0.0;
	const bool known = MotionController::instance().limits_mm(_motionID, (int)Axis::X, xLo, xHi)
		&& MotionController::instance().limits_mm(_motionID, (int)Axis::Y, yLo, yHi);
	_gs2View->setWorld(known, xLo, xHi, yLo, yHi);
}

void VisionApp::gs2RefreshCamera()
{
	if (!_gs2View || !_gs2View->isVisible()) return;
	double x = 0.0, y = 0.0, z = 0.0;
	getCurrentPoint(x, y, z);
	_gs2View->setCamera(true, x, y, _gs2FovWmm, _gs2FovHmm);
}

void VisionApp::gs2RefreshFov()
{
	const int wPx = CAMManager::instance().getWidth(QStringLiteral("cam1"));
	const int hPx = CAMManager::instance().getHeight(QStringLiteral("cam1"));
	_gs2FovWmm = wPx > 0 ? ScaleManager::instance().to_horizontal_mm(wPx) : 0.0;
	_gs2FovHmm = hPx > 0 ? ScaleManager::instance().to_vertical_mm(hPx) : 0.0;
}

void VisionApp::gs2Shutdown()
{
	//the stacked widgets delete their pages after ~VisionApp's body and may emit currentChanged on
	//the way; with the connection cut and the pointers cleared every gs2 function is a no-op
	QObject::disconnect(_gs2ViewConn);
	if (_gs2CameraTimer) _gs2CameraTimer->stop();
	_gs2Page = nullptr;
	_gs2View = nullptr;
}

void VisionApp::gs2BeforeRecipeChange()
{
	if (_gs2Page) _gs2Page->confirmLeave();
}

/*
* Every press of the app's Close button is asked, so a mis-press costs nothing. Unsaved Guided
* Setup (new) edits turn the question into Save and close / Discard and close / Cancel. Cancel is
* the default and Esc in both.
*/
bool VisionApp::gs2ConfirmClose()
{
	if (_gs2Page && _gs2Page->isDirty()) return _gs2Page->confirmClose();

	QMessageBox box(this);
	box.setIcon(QMessageBox::Question);
	box.setWindowTitle(QStringLiteral("Close"));
	box.setText(QStringLiteral("Close the application?"));
	QPushButton* bClose = box.addButton(QStringLiteral("Close"), QMessageBox::AcceptRole);
	QPushButton* bCancel = box.addButton(QStringLiteral("Cancel"), QMessageBox::RejectRole);
	box.setDefaultButton(bCancel);
	box.setEscapeButton(bCancel);
	box.exec();
	return box.clickedButton() == bClose;
}

void VisionApp::gs2RecipeOpened()
{
	if (!_gs2Page) return;
	gs2ShowPatternBox(false);
	_gs2Session = Gs2Session(); //what was found belongs to the recipe before
	if (_gs2View) _gs2View->clearLocated();
	_gs2Page->loadRecipe(Common::Directory::getRecipeCurrentPath(), Common::Directory::CurrentRecipe);
}

// ── build step 2: circle finder + camera direction check ─────────────────────────────────────

void VisionApp::gs2LoadCameraCal()
{
	QString why;
	if (!gs2::loadCameraCal(gs2::cameraCalPath(Common::Directory::ConfigPath()), _gs2Cal, why))
		ct::logger::error("[GuidedSetup2] %s - the camera conversion is Set until it is saved again", qPrintable(why));
	else if (_gs2Cal.measured)
		ct::logger::info("[GuidedSetup2] Camera conversion in use: %s (measured %s: X %.4f, Y %.4f um/px, %+.4f deg)",
			_gs2Cal.useMeasured ? "Measured" : "Set", qPrintable(_gs2Cal.when), _gs2Cal.umPerPxX, _gs2Cal.umPerPxY, _gs2Cal.angleDeg);
}

void VisionApp::gs2RefreshCalUi()
{
	if (!_gs2Page) return;
	QString text;
	bool warn = false;
	if (!_gs2Cal.useMeasured) {
		const double h = ScaleManager::instance().horizontal_um_per_px(), v = ScaleManager::instance().vertical_um_per_px();
		text = (h > 0.0 && v > 0.0)
			? QStringLiteral("X %1, Y %2 um/px from Camera Scaling; image right = +X, image down = +Y").arg(h, 0, 'f', 3).arg(v, 0, 'f', 3)
			: QStringLiteral("The camera scale is not set - run Camera Scaling");
		warn = !(h > 0.0 && v > 0.0);
		if (_gs2Cal.measured)
			text += QStringLiteral("\nMeasured one saved %1: X %2, Y %3 um/px, rotation %4 deg")
				.arg(_gs2Cal.when).arg(_gs2Cal.umPerPxX, 0, 'f', 3).arg(_gs2Cal.umPerPxY, 0, 'f', 3).arg(signedNum(_gs2Cal.angleDeg, 3));
	}
	else {
		text = QStringLiteral("X %1, Y %2 um/px, rotation %3 deg, fit %4 px; measured %5 at X %6, Y %7, Z %8")
			.arg(_gs2Cal.umPerPxX, 0, 'f', 3).arg(_gs2Cal.umPerPxY, 0, 'f', 3).arg(signedNum(_gs2Cal.angleDeg, 3))
			.arg(_gs2Cal.fitPx, 0, 'f', 2).arg(_gs2Cal.when, mm(_gs2Cal.atX), mm(_gs2Cal.atY), mm(_gs2Cal.atZ));
		double a[2][2];
		QString why;
		if (!gs2Conversion(_gs2Cal.imgW, _gs2Cal.imgH, a, why)) { text += QStringLiteral("\nCannot be used: %1").arg(why); warn = true; }
	}
	_gs2Page->setCameraConversion(_gs2Cal.useMeasured, text, warn, _gs2LastMeasure.measured);
}

/*
* World mm per image px, by the conversion chosen on the page. A measured matrix is refused once
* the image it applies to has changed: a different image rotation (Camera Alignment re-run) or a
* different image size makes it describe a different picture.
*/
bool VisionApp::gs2Conversion(int imgW, int imgH, double a[2][2], QString& why)
{
	if (!_gs2Cal.useMeasured) {
		const double h = ScaleManager::instance().horizontal_um_per_px(), v = ScaleManager::instance().vertical_um_per_px();
		if (!(h > 0.0) || !(v > 0.0)) { why = QStringLiteral("the camera scale (um per pixel) is not set - run Camera Scaling"); return false; }
		a[0][0] = h / 1000.0; a[0][1] = 0.0;
		a[1][0] = 0.0;        a[1][1] = v / 1000.0;
		return true;
	}
	if (!_gs2Cal.measured) { why = QStringLiteral("no measured conversion is saved - choose Set"); return false; }
	const double camAngle = SystemData::instance()._camAngles.value(_camID, 0.0);
	if (std::abs(camAngle - _gs2Cal.camAngleDeg) > 1e-6) {
		why = QStringLiteral("the camera angle (Camera Alignment) is %1 deg now but was %2 deg when the conversion was measured - "
			"run Check camera direction and Save measured again, or choose Set").arg(camAngle, 0, 'f', 4).arg(_gs2Cal.camAngleDeg, 0, 'f', 4);
		return false;
	}
	if (imgW != _gs2Cal.imgW || imgH != _gs2Cal.imgH) {
		why = QStringLiteral("the image is %1 x %2 px but the conversion was measured on %3 x %4 px images - choose Set for this image")
			.arg(imgW).arg(imgH).arg(_gs2Cal.imgW).arg(_gs2Cal.imgH);
		return false;
	}
	for (int r = 0; r < 2; r++)
		for (int k = 0; k < 2; k++) a[r][k] = _gs2Cal.a[r][k];
	return true;
}

void VisionApp::gs2ChooseConversion(bool measured)
{
	if (!_gs2Page) return;
	if (_zsBusy || measured == _gs2Cal.useMeasured) { gs2RefreshCalUi(); return; }
	if (measured && !_gs2Cal.measured) {
		_gs2Page->showToolResult(QStringLiteral("No measured conversion is saved yet: run Check camera direction, then Save measured."), false);
		gs2RefreshCalUi();
		return;
	}
	if (QMessageBox::question(this, QStringLiteral("Guided Setup (new)"),
		QStringLiteral("Use the %1 pixel-to-mm conversion for Guided Setup (new) on this machine?\n\n"
			"It applies to every recipe's Guided Setup (new). Nothing else in the app uses it.")
			.arg(measured ? QStringLiteral("Measured") : QStringLiteral("Set (Camera Scaling)"))) != QMessageBox::Yes) {
		gs2RefreshCalUi();
		return;
	}
	gs2::CameraCal next = _gs2Cal;
	next.useMeasured = measured;
	QString why;
	if (!gs2::saveCameraCal(gs2::cameraCalPath(Common::Directory::ConfigPath()), next, why)) {
		_gs2Page->showToolResult(QStringLiteral("The conversion was not changed: %1.").arg(why), false);
		gs2RefreshCalUi();
		return;
	}
	_gs2Cal = next;
	ct::logger::info("[GuidedSetup2] Camera conversion in use: %s", measured ? "Measured" : "Set");
	AuditLog::instance().log(QStringLiteral("GS2_CAMERA_CONVERSION"), measured ? QStringLiteral("Measured") : QStringLiteral("Set"));
	_gs2Page->showToolResult(QStringLiteral("Pixel to mm now uses %1.").arg(measured ? QStringLiteral("the Measured conversion") : QStringLiteral("Set (Camera Scaling)")), true);
	gs2RefreshCalUi();
}

void VisionApp::gs2SaveMeasured()
{
	if (!_gs2Page || _zsBusy || !_gs2LastMeasure.measured) return;
	const auto& m = _gs2LastMeasure;
	QString replaces;
	if (_gs2Cal.measured) replaces = QStringLiteral("\n\nIt replaces the one measured %1%2.").arg(_gs2Cal.when,
		_gs2Cal.useMeasured ? QStringLiteral(", which is in use - this one is used from now on") : QString());
	if (QMessageBox::question(this, QStringLiteral("Save measured"),
		QStringLiteral("Save this measurement as the Measured conversion for Guided Setup (new) on this machine?\n\n"
			"X %1, Y %2 um/px, rotation %3 deg, fit %4 px, measured %5.%6")
			.arg(m.umPerPxX, 0, 'f', 3).arg(m.umPerPxY, 0, 'f', 3).arg(signedNum(m.angleDeg, 3)).arg(m.fitPx, 0, 'f', 2)
			.arg(m.when, replaces)) != QMessageBox::Yes) return;

	gs2::CameraCal next = m;
	next.useMeasured = _gs2Cal.useMeasured;
	QString why;
	if (!gs2::saveCameraCal(gs2::cameraCalPath(Common::Directory::ConfigPath()), next, why)) {
		_gs2Page->showToolResult(QStringLiteral("Not saved: %1.").arg(why), false);
		return;
	}
	_gs2Cal = next;
	ct::logger::info("[GuidedSetup2] Measured camera conversion saved: X %.4f, Y %.4f um/px, %+.4f deg, fit %.2f px, mm per px [%.7f %.7f; %.7f %.7f]",
		m.umPerPxX, m.umPerPxY, m.angleDeg, m.fitPx, m.a[0][0], m.a[0][1], m.a[1][0], m.a[1][1]);
	AuditLog::instance().log(QStringLiteral("GS2_CAMERA_SAVE"), QStringLiteral("X %1, Y %2 um/px, %3 deg")
		.arg(m.umPerPxX, 0, 'f', 3).arg(m.umPerPxY, 0, 'f', 3).arg(m.angleDeg, 0, 'f', 3));
	_gs2Page->showToolResult(_gs2Cal.useMeasured
		? QStringLiteral("Saved, and in use.")
		: QStringLiteral("Saved. Choose Measured under Pixel to mm to use it."), true);
	gs2RefreshCalUi();
}

/*
* The same lock a Z search takes: refused while anything else drives the machine, and while it
* runs START, the motion test and every Z search action refuse (_zsBusy), a modal window with its
* own Stop covers the screen and the tools are greyed out.
*/
bool VisionApp::gs2MachineBusy(const QString& title, const QString& note)
{
	if (zsRefuseIfBusy()) return false;
	_zsBusy = true;
	_zsLooseXYLanding = true; //every measurement reads where the camera stopped, so a little past the target costs nothing
	_stopRun = false;
	if (_gs2Page) _gs2Page->setMachineBusy(true);
	_zsBusyWindow = showBusyWindow(title, note + QStringLiteral("\nStop ends it at once; the panel Stop and the e-stop also stop it."));
	return true;
}

void VisionApp::gs2MachineIdle()
{
	closeBusyWindow(_zsBusyWindow);
	//anything still queued from the action lands on the still-disabled tools, not as a new command
	QCoreApplication::processEvents();
	if (_gs2Page) _gs2Page->setMachineBusy(false);
	_zsLooseXYLanding = false;
	_zsBusy = false;
}

namespace {

/*
* The Matrox circle model finder with every setting of a gs2::CircleFinder. mtrx::find_circles fixes
* all but the size and quietly turns "either polarity" into "bright", so the Guided Setup (new) runs
* its own. Matrox errors are not shown as message boxes on this thread while it runs - one bad setting
* would otherwise pop a box per call - and come back as why. Smoothness and detail are tuning: one
* Matrox does not take is logged and left at its default rather than failing the find. (M_ACCURACY
* was here until the IPC showed every circle model refusing it, error 21001 - 2026-11-03.)
*/
bool milCircles(const QImage& img, const gs2::CircleFinder& cf, double minR, double maxR, std::vector<mtrx::Circle>& out,
	std::vector<double>& fitErr, QString& why)
{
	out.clear();
	fitErr.clear();
	MappControl(M_DEFAULT, M_ERROR + M_THREAD_CURRENT, M_PRINT_DISABLE);
	//an error this thread already had before the finder ran is not the finder's, whether Matrox clears
	//the current error on every call or keeps the last one
	const MIL_INT before = MappGetError(M_DEFAULT, M_CURRENT, M_NULL);
	auto errorNow = [before]() {
		const MIL_INT e = MappGetError(M_DEFAULT, M_CURRENT, M_NULL);
		return (e == M_NULL_ERROR || e == before) ? (MIL_INT)M_NULL_ERROR : e;
	};
	QString failedAt;
	auto failed = [&](const char* step) {
		const MIL_INT e = errorNow();
		if (e != M_NULL_ERROR && failedAt.isEmpty()) failedAt = QStringLiteral("%1 (Matrox error %2)").arg(QLatin1String(step)).arg((qlonglong)e);
		return !failedAt.isEmpty();
	};
	auto tune = [&](const char* setting) {
		const MIL_INT e = errorNow();
		if (e != M_NULL_ERROR) ct::logger::warn("[GuidedSetup2] Circle finder: Matrox did not take %s (error %lld) - left at its default", setting, (long long)e);
	};

	MIL_ID mBuf = mtrx::to_milID(img);
	MIL_ID mMono = mBuf != M_NULL ? mtrx::to_mono(mBuf) : M_NULL;
	MIL_ID ctx = M_NULL, res = M_NULL;
	if (mMono != M_NULL) {
		//judged by the handles, not the error code
		MmodAlloc(M_DEFAULT_HOST, M_SHAPE_CIRCLE, M_DEFAULT, &ctx);
		if (ctx != M_NULL) MmodAllocResult(M_DEFAULT_HOST, M_SHAPE_CIRCLE, &res);
		if (ctx == M_NULL || res == M_NULL) failedAt = QStringLiteral("allocating the finder");
	}
	if (ctx != M_NULL && res != M_NULL && failedAt.isEmpty()) {
		const double rExp = (minR + maxR) / 2.0;
		const MIL_INT fg = cf.polarity == gs2::PolarityDark ? M_FOREGROUND_BLACK
			: cf.polarity == gs2::PolarityEither ? M_FOREGROUND_ANY : M_FOREGROUND_WHITE;
		MmodDefine(ctx, M_CIRCLE, fg, rExp, M_DEFAULT, M_DEFAULT, M_DEFAULT);
		if (!failed("the circle model (polarity)")) {
			MmodControl(ctx, 0, M_SCALE_MIN_FACTOR, minR / rExp);
			MmodControl(ctx, 0, M_SCALE_MAX_FACTOR, maxR / rExp);
			failed("the diameter range");
		}
		if (failedAt.isEmpty()) { MmodControl(ctx, M_DEFAULT, M_ACCEPTANCE, (MIL_DOUBLE)cf.minScore); failed("min score"); }
		if (failedAt.isEmpty()) { MmodControl(ctx, 0, M_NUMBER, M_ALL); failed("the number of circles"); }
		if (failedAt.isEmpty()) {
			MmodControl(ctx, M_CONTEXT, M_SMOOTHNESS, (MIL_DOUBLE)cf.smoothness);
			tune("edge smoothness");
			MmodControl(ctx, M_CONTEXT, M_DETAIL_LEVEL, cf.detail == gs2::DetailVeryHigh ? M_VERY_HIGH : cf.detail == gs2::DetailHigh ? M_HIGH : M_MEDIUM);
			tune("edge detail");
			MmodPreprocess(ctx, M_DEFAULT);
			failed("preparing the finder");
		}
		if (failedAt.isEmpty()) { MmodFind(ctx, mMono, res); failed("finding"); }
		MIL_INT n = 0;
		if (failedAt.isEmpty()) { MmodGetResult(res, M_DEFAULT, M_NUMBER + M_TYPE_MIL_INT, &n); failed("reading the count"); }
		if (failedAt.isEmpty() && n > 0) {
			const size_t count = static_cast<size_t>(n);
			std::vector<MIL_DOUBLE> cx(count), cy(count), r(count), sc(count), fe(count, -1.0);
			MmodGetResult(res, M_DEFAULT, M_CENTER_X + M_TYPE_MIL_DOUBLE, cx.data());
			MmodGetResult(res, M_DEFAULT, M_CENTER_Y + M_TYPE_MIL_DOUBLE, cy.data());
			MmodGetResult(res, M_DEFAULT, M_RADIUS + M_TYPE_MIL_DOUBLE, r.data());
			MmodGetResult(res, M_DEFAULT, M_SCORE + M_TYPE_MIL_DOUBLE, sc.data());
			if (!failed("reading the circles")) {
				MmodGetResult(res, M_DEFAULT, M_FIT_ERROR + M_TYPE_MIL_DOUBLE, fe.data());
				if (errorNow() != M_NULL_ERROR) std::fill(fe.begin(), fe.end(), -1.0); //not reported: fine
				for (size_t i = 0; i < count; i++) {
					out.push_back(mtrx::Circle{ r[i], cx[i] - r[i], cy[i] - r[i], cx[i], cy[i], sc[i] });
					fitErr.push_back(fe[i]);
				}
			}
		}
	}
	//read before the clean-up: mtrx::free_buffer sets the id it frees back to M_NULL
	const bool imageHandedOver = mBuf != M_NULL && mMono != M_NULL;
	if (res != M_NULL) MmodFree(res);
	if (ctx != M_NULL) MmodFree(ctx);
	if (mMono != M_NULL) mtrx::free_buffer(mMono);
	if (mBuf != M_NULL) mtrx::free_buffer(mBuf);
	MappControl(M_DEFAULT, M_ERROR + M_THREAD_CURRENT, M_PRINT_ENABLE);

	if (!imageHandedOver) { why = QStringLiteral("the image could not be handed to the circle finder"); return false; }
	if (!failedAt.isEmpty()) {
		ct::logger::error("[GuidedSetup2] Circle finder failed at %s", qPrintable(failedAt));
		why = QStringLiteral("the circle finder failed at %1 - check the circle settings").arg(failedAt);
		return false;
	}
	return true;
}

} // namespace

/*
* Every circle the finder accepts, drawn on the FOV (green = the one picked); the one nearest the
* image centre (or nearPx) is returned with its offset from the centre in world mm. Where the camera
* stood is the caller's: only it knows where this image was taken.
*/
bool VisionApp::gs2FindCircle(const QImage& image, const gs2::CircleFinder& cf, Gs2Circle& out, QString& why, const QPointF* nearPx,
	QVector<QPointF>* allPx, bool* none)
{
	out = Gs2Circle();
	if (allPx) allPx->clear();
	if (none) *none = false;
	if (image.isNull()) { why = QStringLiteral("there is no image"); return false; }
	double a[2][2];
	if (!gs2Conversion(image.width(), image.height(), a, why)) return false;
	const double mmPerPx = std::sqrt(std::abs(a[0][0] * a[1][1] - a[0][1] * a[1][0])); //the conversion's mean scale
	if (!(mmPerPx > 0.0)) { why = QStringLiteral("the pixel-to-mm conversion is empty"); return false; }
	if (!cf.dia.valid()) { why = QStringLiteral("the diameter range is not valid"); return false; }

	const QImage img = milReady(image);

	const double minPx = cf.dia.minMm / mmPerPx;
	const double maxPx = cf.dia.maxMm / mmPerPx;
	std::vector<mtrx::Circle> circles;
	std::vector<double> fitErr;
	if (!milCircles(img, cf, minPx / 2.0, maxPx / 2.0, circles, fitErr, why)) { clearGuidedMarks(); return false; }

	if (circles.empty()) {
		clearGuidedMarks();
		if (none) *none = true;
		why = QStringLiteral("no circle between %1 and %2 mm (%3 .. %4 px) in the image")
			.arg(mm(cf.dia.minMm), mm(cf.dia.maxMm)).arg(std::lround(minPx)).arg(std::lround(maxPx));
		return false;
	}

	//the image centre is w/2, h/2 - the convention every other pixel-to-mm conversion in the app uses
	const double cx = img.width() / 2.0, cy = img.height() / 2.0;
	const double nx = nearPx ? nearPx->x() : cx, ny = nearPx ? nearPx->y() : cy;
	int nearest = 0;
	double best = std::numeric_limits<double>::max();
	for (int i = 0; i < (int)circles.size(); i++) {
		const double d = std::hypot(circles[i].cx - nx, circles[i].cy - ny);
		if (d < best) { best = d; nearest = i; }
	}
	drawGuidedCircles(circles, nearest);
	if (allPx) for (const auto& ci : circles) allPx->push_back(QPointF(ci.cx, ci.cy));

	const auto& c = circles[nearest];
	out.px = c.cx;
	out.py = c.cy;
	out.diaPx = 2.0 * c.radius;
	out.diaMm = out.diaPx * mmPerPx;
	out.score = c.score;
	out.fitErr = nearest < (int)fitErr.size() ? fitErr[nearest] : -1.0;
	out.count = (int)circles.size();
	out.imgW = img.width();
	out.imgH = img.height();
	out.offX = a[0][0] * (c.cx - cx) + a[0][1] * (c.cy - cy);
	out.offY = a[1][0] * (c.cx - cx) + a[1][1] * (c.cy - cy);
	return true;
}

/*
* Snap where the camera stands, find, convert; no move. The drive says "done" before the head has
* stopped: on the IPC (2026-10-30) a snap ~0.3 s after a 5 mm move put the circle up to 5 px from
* where it sat at rest, and one came out blurred (173 px across instead of 159). So it keeps
* snapping until three snaps in a row agree - circle and encoders both - which measures the head
* itself rather than guessing a delay. With the encoders still for kQuietMs, a circle that keeps
* jittering under kJitterPx is the finder, and the last kQuietSnaps are averaged instead.
*/
bool VisionApp::gs2SnapCircle(const gs2::CircleFinder& cf, Gs2Circle& out, QString& why, const QPointF* nearPx)
{
	if (SystemData::instance()._snapDelay_ms > 0) zsPause(SystemData::instance()._snapDelay_ms);

	auto& mc = MotionController::instance();
	auto stageStill = [](const Gs2Circle& a, const Gs2Circle& b) {
		return std::abs(a.stageX - b.stageX) <= kStillStageMm && std::abs(a.stageY - b.stageY) <= kStillStageMm;
	};
	auto pxApart = [](const Gs2Circle& a, const Gs2Circle& b) { return std::hypot(a.px - b.px, a.py - b.py); };
	//the last n snaps averaged into out, and how far the furthest of them sits from that average
	auto average = [&out, &pxApart](const QVector<Gs2Circle>& s, int n) {
		const int first = s.size() - n;
		out = s.back();
		out.px = out.py = out.diaPx = out.diaMm = out.offX = out.offY = out.stageX = out.stageY = 0.0;
		for (int i = first; i < s.size(); i++) {
			out.px += s[i].px / n; out.py += s[i].py / n;
			out.diaPx += s[i].diaPx / n; out.diaMm += s[i].diaMm / n;
			out.offX += s[i].offX / n; out.offY += s[i].offY / n;
			out.stageX += s[i].stageX / n; out.stageY += s[i].stageY / n;
		}
		out.wx = out.stageX + out.offX;
		out.wy = out.stageY + out.offY;
		double spread = 0.0;
		for (int i = first; i < s.size(); i++) spread = std::max(spread, pxApart(s[i], out));
		return spread;
	};
	QElapsedTimer timer;
	timer.start();
	QVector<Gs2Circle> quiet; //the snaps since the encoders last moved
	qint64 quietSince = 0;
	QString how;
	int snaps = 0, missed = 0;
	while (true) {
		if (_stopRun) { why = QStringLiteral("stopped"); return false; }
		const auto x = mc.get_position_mm(_motionID, 0, (int)Axis::X);
		const auto y = mc.get_position_mm(_motionID, 0, (int)Axis::Y);
		if (!x || !y) { why = QStringLiteral("could not read the X / Y position"); return false; }

		QImage img;
		if (!guidedSnapFov(img, kSnapTimeoutMs)) { why = QStringLiteral("no image from the camera"); return false; }
		if (_stopRun) { why = QStringLiteral("stopped"); return false; }
		Gs2Circle cur;
		bool none = false;
		QString miss;
		if (!gs2FindCircle(img, cf, cur, miss, nearPx, nullptr, &none)) {
			if (!none) { why = miss; return false; } //the finder itself failed: another snap will not help
		}
		else if (cur.fitErr > kMaxFitErrPx) {
			none = true;
			miss = badFit(cur.diaPx, cur.fitErr);
		}
		snaps++;
		//a frame now and then comes back without the circle or with a bad fit (IPC 2026-11-03: the same
		//grid point of the direction check passed twice and failed twice) - snap again, within the time
		if (none) {
			missed++;
			ct::logger::info("[GuidedSetup2] Circle snap %d missed: %s", snaps, qPrintable(miss));
			if (timer.elapsed() > kStillMaxMs) {
				why = QStringLiteral("the camera did not settle within %1 s: %2 of %3 snaps missed the circle (last: %4)")
					.arg(kStillMaxMs / 1000).arg(missed).arg(snaps).arg(miss);
				return false;
			}
			zsPause(kStillGapMs);
			continue;
		}
		cur.haveStage = true;
		cur.stageX = *x;
		cur.stageY = *y;
		cur.wx = cur.stageX + cur.offX;
		cur.wy = cur.stageY + cur.offY;

		const Gs2Circle prev = quiet.isEmpty() ? cur : quiet.back();
		if (!quiet.isEmpty() && !stageStill(cur, prev)) quiet.clear();
		if (quiet.isEmpty()) quietSince = timer.elapsed();
		quiet.push_back(cur);
		const int n = quiet.size();

		//three in a row this close: the head has stopped (the three averaged: a third less finder noise)
		if (n >= 3 && pxApart(quiet[n - 1], quiet[n - 2]) <= kStillPx && pxApart(quiet[n - 2], quiet[n - 3]) <= kStillPx) {
			average(quiet, 3);
			how = QStringLiteral("3 in a row within %1 px").arg(kStillPx);
			break;
		}
		//the encoders have been still a while: what still jumps is the finder. The snaps that agree with
		//the median of the last five are averaged; a stray one is left out
		if (n >= kQuietSnaps && timer.elapsed() - quietSince >= kQuietMs) {
			QVector<double> xs, ys;
			for (int i = n - kQuietSnaps; i < n; i++) { xs << quiet[i].px; ys << quiet[i].py; }
			std::sort(xs.begin(), xs.end());
			std::sort(ys.begin(), ys.end());
			const double mx = xs[kQuietSnaps / 2], my = ys[kQuietSnaps / 2];
			QVector<Gs2Circle> agree;
			for (int i = n - kQuietSnaps; i < n; i++)
				if (std::hypot(quiet[i].px - mx, quiet[i].py - my) <= kJitterPx) agree << quiet[i];
			if (agree.size() >= kQuietAgree) {
				const double spread = average(agree, agree.size());
				how = QStringLiteral("encoders still %1 ms, %2 of the last %3 within %4 px of their median (spread %5 px)")
					.arg(timer.elapsed() - quietSince).arg(agree.size()).arg(kQuietSnaps).arg(kJitterPx).arg(spread, 0, 'f', 2);
				break;
			}
		}
		if (timer.elapsed() > kStillMaxMs) {
			if (n >= kQuietSnaps && timer.elapsed() - quietSince >= kQuietMs) {
				QStringList at;
				for (int i = n - kQuietSnaps; i < n; i++) at << QStringLiteral("(%1, %2)").arg(quiet[i].px, 0, 'f', 2).arg(quiet[i].py, 0, 'f', 2);
				ct::logger::warn("[GuidedSetup2] Circle would not hold still with the encoders still: last %d at %s px",
					kQuietSnaps, qPrintable(at.join(QStringLiteral(" "))));
				why = QStringLiteral("the circle measurement would not hold still: with the encoders still, fewer than %1 of the last %2 "
					"snaps agreed within %3 px - run the Stability test and tune the circle settings, or check it is clean, in focus and evenly lit")
					.arg(kQuietAgree).arg(kQuietSnaps).arg(kJitterPx);
			}
			else {
				why = QStringLiteral("the camera did not settle within %1 s: between the last two snaps the circle moved %2 px and the encoders %3 / %4 mm")
					.arg(kStillMaxMs / 1000).arg(pxApart(cur, prev), 0, 'f', 2)
					.arg(mm(cur.stageX - prev.stageX), mm(cur.stageY - prev.stageY));
			}
			return false;
		}
		zsPause(kStillGapMs);
	}
	if (missed > 0) how += QStringLiteral("; %1 snap(s) missed the circle").arg(missed);

	ct::logger::info("[GuidedSetup2] Circle: %d in range, nearest dia %.1f px score %.2f at (%.2f, %.2f) px = %+.3f, %+.3f mm off centre; camera %.3f, %.3f -> circle %.3f, %.3f (still after %d snaps, %lld ms: %s)",
		out.count, out.diaPx, out.score, out.px, out.py, out.offX, out.offY, out.stageX, out.stageY, out.wx, out.wy, snaps, (long long)timer.elapsed(), qPrintable(how));
	return true;
}

/*
* centre = false is the run-time locate: snap, convert, and move onto the circle (then snap again)
* only when it lies outside the middle half of the image, where lens distortion is worst.
* centre = true is the teach: move onto the circle until it is within kCentreTolMm of the image
* centre. Either way the result is the circle's own sub-pixel position in world mm, not where the
* camera stopped. The pixel-to-mm direction is never guessed: a circle that ends up further from
* the centre after the camera moved onto it stops the action with that reason.
* expectPx (fid0 at Find): the pattern says where in this image the circle is; the circle nearest
* that point is taken, and it must be within expectTolMm (0 = kExpectTolMm) of it - with rows of
* identical circles on the trolley, "nearest the image centre" took a neighbour 13 mm away (IPC
* 2026-10-30 17:50). After a move the camera stands on that circle, so the centre is right again.
*/
bool VisionApp::gs2LocateCircle(const gs2::CircleFinder& cf, bool centre, Gs2Circle& out, int& moves, QString& why, const QPointF* expectPx,
	double expectTolMm)
{
	moves = 0;
	setProgress(_zsBusyWindow, QStringLiteral("Snap and find the circle..."));
	if (!gs2SnapCircle(cf, out, why, expectPx)) return false;
	if (expectPx) {
		const double tol = expectTolMm > 0.0 ? expectTolMm : kExpectTolMm;
		const double mmPerPx = out.diaPx > 0.0 ? out.diaMm / out.diaPx : 0.0;
		const double offMm = std::hypot(out.px - expectPx->x(), out.py - expectPx->y()) * mmPerPx;
		ct::logger::info("[GuidedSetup2] Circle where expected: expected (%.1f, %.1f) px, nearest at (%.1f, %.1f) px, %.3f mm away (limit %.1f)",
			expectPx->x(), expectPx->y(), out.px, out.py, offMm, tol);
		if (offMm > tol) {
			why = QStringLiteral("no circle where it was expected - the nearest is %1 mm from that spot (limit %2 mm)").arg(mm(offMm)).arg(tol, 0, 'f', 1);
			return false;
		}
	}
	double lastOff = std::hypot(out.offX, out.offY);

	while (true) {
		const bool inMiddle = std::abs(out.px - out.imgW / 2.0) <= out.imgW / 4.0 && std::abs(out.py - out.imgH / 2.0) <= out.imgH / 4.0;
		if (centre ? lastOff <= kCentreTolMm : inMiddle) return true;
		if (moves >= (centre ? kCentreMoves : 1)) {
			why = centre
				? QStringLiteral("the circle is still %1 mm from the image centre after %2 moves").arg(mm(lastOff)).arg(moves)
				: QStringLiteral("the circle is still outside the middle half of the image after moving onto it");
			return false;
		}
		setProgress(_zsBusyWindow, QStringLiteral("Moving onto the circle at X %1, Y %2 (%3 mm away)...").arg(mm(out.wx), mm(out.wy), mm(lastOff)));
		if (!zsMove({ { (int)Axis::X, out.wx }, { (int)Axis::Y, out.wy } }, why)) return false;
		moves++;
		if (!gs2SnapCircle(cf, out, why)) return false;

		const double off = std::hypot(out.offX, out.offY);
		if (off > kCentreTolMm && off > lastOff + 0.05) {
			why = QStringLiteral("the circle went further from the image centre (%1 -> %2 mm) when the camera moved onto it. "
				"The pixel-to-mm conversion in use points the wrong way - run Check camera direction").arg(mm(lastOff), mm(off));
			return false;
		}
		lastOff = off;
	}
}

/*
* Centre on a circle, then snap it from a 3 x 3 grid of camera positions +/- a step round it; every
* snap waits for the head to stop and averages three (gs2SnapCircle). With the stage positions read
* back from the card (not the commanded ones - a move lands anywhere within 0.05 mm) the 2x2
* image-px-per-stage-mm matrix M is a least-squares fit through all ten points, and how far the
* worst point sits from it is the fit: a snap that moved or picked another circle shows there
* instead of quietly bending M. M is compared with the Set conversion (and so with production's
* convention: stage +X moves a fixed circle LEFT in the image, stage +Y moves it UP) and with the
* saved Measured one. pass = the conversion IN USE agrees. Nothing is saved here; a measurement
* with a good fit is kept for Save measured.
*/
bool VisionApp::gs2CheckDirection(const gs2::CircleFinder& cf, bool& pass, QString& report, QString& why)
{
	pass = false;
	_gs2LastMeasure = gs2::CameraCal();
	Gs2Circle c0;
	int moves = 0;
	if (!gs2LocateCircle(cf, true, c0, moves, why)) { why = QStringLiteral("centring on the circle: %1").arg(why); return false; }
	const double x0 = c0.stageX, y0 = c0.stageY;
	const auto z0 = MotionController::instance().get_position_mm(_motionID, 0, (int)Axis::Z);
	const double mmPerPx = c0.diaPx > 0.0 ? c0.diaMm / c0.diaPx : 0.0;
	const double step = std::min(kDirStepMm, 0.15 * std::min(c0.imgW, c0.imgH) * mmPerPx); //the circle stays well inside the middle half
	if (!(step > 0.5)) { why = QStringLiteral("the image is too small for the check"); return false; }

	//3 x 3 positions in a snake, so no move is longer than one diagonal step
	struct Shot { int i, j; double dx, dy; Gs2Circle c; };
	std::vector<Shot> shots;
	for (int j = -1; j <= 1; j++)
		for (int k = 0; k < 3; k++) {
			const int i = (j == 0) ? 1 - k : k - 1;
			shots.push_back({ i, j, i * step, j * step, {} });
		}
	for (const auto& s : shots)
		if (!zsCheckTarget((int)Axis::X, x0 + s.dx, why) || !zsCheckTarget((int)Axis::Y, y0 + s.dy, why)) return false;

	bool ok = true;
	for (int n = 0; n < (int)shots.size(); n++) {
		auto& s = shots[n];
		setProgress(_zsBusyWindow, QStringLiteral("Position %1 of %2: X %3, Y %4").arg(n + 1).arg(shots.size()).arg(mm(x0 + s.dx), mm(y0 + s.dy)));
		if (!zsMove({ { (int)Axis::X, x0 + s.dx }, { (int)Axis::Y, y0 + s.dy } }, why) || !gs2SnapCircle(cf, s.c, why)) { ok = false; break; }
		if (std::abs(s.c.diaPx - c0.diaPx) > 0.15 * c0.diaPx) {
			why = QStringLiteral("lost the circle: %1 px across here, %2 px at the start - another circle is nearer the image centre")
				.arg(s.c.diaPx, 0, 'f', 1).arg(c0.diaPx, 0, 'f', 1);
			ok = false;
			break;
		}
	}
	//back over the circle, unless Stop was pressed
	if (!_stopRun) {
		setProgress(_zsBusyWindow, QStringLiteral("Back over the circle..."));
		QString w;
		if (!zsMove({ { (int)Axis::X, x0 }, { (int)Axis::Y, y0 } }, w) && ok) { why = QStringLiteral("could not move back over the circle: %1").arg(w); ok = false; }
	}
	if (!ok) return false;

	//least squares through the ten points (the centring snap too), each image row on its own:
	//px = px0 + M00 dx + M01 dy and py = py0 + M10 dx + M11 dy, dx dy = stage from the start
	std::vector<const Gs2Circle*> pts{ &c0 };
	for (const auto& s : shots) pts.push_back(&s.c);
	double N[3][3] = { { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 } }, bx[3] = { 0.0, 0.0, 0.0 }, by[3] = { 0.0, 0.0, 0.0 };
	for (const auto* c : pts) {
		const double v[3] = { 1.0, c->stageX - x0, c->stageY - y0 };
		for (int r = 0; r < 3; r++) {
			for (int k = 0; k < 3; k++) N[r][k] += v[r] * v[k];
			bx[r] += v[r] * (c->px - c0.px);
			by[r] += v[r] * (c->py - c0.py);
		}
	}
	auto det3 = [](const double m[3][3]) {
		return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
			+ m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
	};
	const double dN = det3(N);
	if (std::abs(dN) < 1e-6) { why = QStringLiteral("the stage moves were too small to measure"); return false; }
	auto solve = [&](const double b[3], double x[3]) { //Cramer's rule
		for (int col = 0; col < 3; col++) {
			double t[3][3];
			for (int r = 0; r < 3; r++)
				for (int k = 0; k < 3; k++) t[r][k] = (k == col) ? b[r] : N[r][k];
			x[col] = det3(t) / dN;
		}
	};
	double fx[3], fy[3];
	solve(bx, fx);
	solve(by, fy);
	double M[2][2];
	M[0][0] = fx[1]; M[0][1] = fx[2];
	M[1][0] = fy[1]; M[1][1] = fy[2];
	const double detM = M[0][0] * M[1][1] - M[0][1] * M[1][0];
	if (std::abs(detM) < 1e-12) { why = QStringLiteral("the circle did not move with the stage"); return false; }

	//the fit: how far each point sits from the fitted matrix
	double fitPx = 0.0, sumSq = 0.0;
	QString worst = QStringLiteral("the start");
	for (int n = 0; n < (int)pts.size(); n++) {
		const auto* c = pts[n];
		const double dx = c->stageX - x0, dy = c->stageY - y0;
		const double r = std::hypot(c->px - c0.px - (fx[0] + M[0][0] * dx + M[0][1] * dy), c->py - c0.py - (fy[0] + M[1][0] * dx + M[1][1] * dy));
		sumSq += r * r;
		if (r > fitPx) {
			fitPx = r;
			if (n > 0) {
				const auto& s = shots[n - 1];
				worst = QStringLiteral("X%1 Y%2").arg(s.i > 0 ? QStringLiteral("+") : s.i < 0 ? QStringLiteral("-") : QStringLiteral("0"),
					s.j > 0 ? QStringLiteral("+") : s.j < 0 ? QStringLiteral("-") : QStringLiteral("0"));
			}
			else worst = QStringLiteral("the start");
		}
	}
	const double fitRms = std::sqrt(sumSq / pts.size());

	//one column of M against the same column of an expected matrix e
	struct Cmp { double pct = 0.0, angle = 0.0; bool sameWay = false; };
	auto compare = [&](const double e[2][2], int col) {
		Cmp r;
		const double mx = M[0][col], my = M[1][col], ex = e[0][col], ey = e[1][col];
		const double lm = std::hypot(mx, my), le = std::hypot(ex, ey);
		r.pct = lm > 0.0 ? (le / lm - 1.0) * 100.0 : 0.0;   //measured um/px against expected, in %
		r.angle = std::atan2(ex * my - ey * mx, ex * mx + ey * my) * 180.0 / kPi;
		r.sameWay = (ex * mx + ey * my) > 0.0 && std::abs(r.angle) < 45.0;
		return r;
	};
	auto agrees = [](const Cmp& x, const Cmp& y) {
		return x.sameWay && y.sameWay && std::abs(x.pct) <= kDirMaxScalePct && std::abs(y.pct) <= kDirMaxScalePct
			&& std::abs(x.angle) <= kDirMaxAngleDeg && std::abs(y.angle) <= kDirMaxAngleDeg;
	};
	auto cmpLine = [](const QString& name, const Cmp& x, const Cmp& y, bool ok) {
		return QStringLiteral("%1: X %2%, Y %3%, angle %4 / %5 deg - %6").arg(name)
			.arg(signedNum(x.pct, 2), signedNum(y.pct, 2), signedNum(x.angle, 3), signedNum(y.angle, 3))
			.arg(ok ? QStringLiteral("agrees") : QStringLiteral("OFF (limits %1 %, %2 deg)").arg(kDirMaxScalePct, 0, 'f', 2).arg(kDirMaxAngleDeg, 0, 'f', 2));
	};

	//the production convention, direction only: stage +X moves the circle left, +Y moves it up
	const double conv[2][2] = { { -1.0, 0.0 }, { 0.0, -1.0 } };
	const Cmp convX = compare(conv, 0), convY = compare(conv, 1);
	const bool conventionOk = convX.sameWay && convY.sameWay;

	const double h = ScaleManager::instance().horizontal_um_per_px(), v = ScaleManager::instance().vertical_um_per_px();
	const bool haveSet = h > 0.0 && v > 0.0;
	const double setE[2][2] = { { haveSet ? -1000.0 / h : 0.0, 0.0 }, { 0.0, haveSet ? -1000.0 / v : 0.0 } };
	const Cmp setX = compare(setE, 0), setY = compare(setE, 1);
	const bool setOk = haveSet && agrees(setX, setY);

	double savedE[2][2] = { { 0.0, 0.0 }, { 0.0, 0.0 } };
	const bool haveSaved = _gs2Cal.measured && pxPerMm(_gs2Cal.a, savedE);
	const Cmp savX = haveSaved ? compare(savedE, 0) : Cmp(), savY = haveSaved ? compare(savedE, 1) : Cmp();
	const bool savedOk = haveSaved && agrees(savX, savY);

	const bool fitOk = fitPx <= kDirMaxFitPx;
	pass = fitOk && (_gs2Cal.useMeasured ? savedOk : setOk);

	const double umX = 1000.0 / std::hypot(M[0][0], M[1][0]), umY = 1000.0 / std::hypot(M[0][1], M[1][1]);
	QStringList lines;
	lines << QStringLiteral("%1 - the conversion in use (%2) %3. Circle at X %4, Y %5, 3 x 3 grid +/- %6 mm, fit %7 px (rms %8, worst at %9).")
		.arg(pass ? QStringLiteral("PASS") : QStringLiteral("FAIL"), _gs2Cal.useMeasured ? QStringLiteral("Measured") : QStringLiteral("Set"),
			pass ? QStringLiteral("agrees with this measurement") : QStringLiteral("does not agree with this measurement"),
			mm(c0.wx), mm(c0.wy), mm(step)).arg(fitPx, 0, 'f', 2).arg(fitRms, 0, 'f', 2).arg(worst);
	lines << QStringLiteral("Measured: X %1, Y %2 um/px, rotation %3 / %4 deg. %5")
		.arg(umX, 0, 'f', 3).arg(umY, 0, 'f', 3).arg(signedNum(convX.angle, 3), signedNum(convY.angle, 3))
		.arg(conventionOk ? QStringLiteral("Image right = +X and image down = +Y, as production assumes.")
			: QStringLiteral("The image does NOT read the way production assumes (image right = +X, image down = +Y)."));
	lines << (haveSet ? cmpLine(QStringLiteral("Set (Camera Scaling, X %1, Y %2 um/px)").arg(h, 0, 'f', 3).arg(v, 0, 'f', 3), setX, setY, setOk)
		: QStringLiteral("Set: the camera scale is not set."));
	lines << (haveSaved ? cmpLine(QStringLiteral("Saved measured (%1)").arg(_gs2Cal.when), savX, savY, savedOk)
		: QStringLiteral("Saved measured: none yet."));
	if (!fitOk)
		lines << QStringLiteral("The %1 points disagree by up to %2 px, worst at %3 (limit %4): something moved, or another circle was picked. "
			"This measurement cannot be saved - run it again.").arg(pts.size()).arg(fitPx, 0, 'f', 2).arg(worst).arg(kDirMaxFitPx, 0, 'f', 1);
	else if (!pass)
		lines << QStringLiteral("To use this measurement instead: Save measured, then choose Measured under Pixel to mm. Nothing has been saved.");
	else
		lines << QStringLiteral("Save measured keeps this one as the Measured conversion. Nothing has been saved.");
	report = lines.join(QLatin1Char('\n'));

	double A[2][2]; //world mm per px = -M^-1
	A[0][0] = -M[1][1] / detM; A[0][1] = M[0][1] / detM;
	A[1][0] = M[1][0] / detM;  A[1][1] = -M[0][0] / detM;
	ct::logger::info("[GuidedSetup2] Camera direction %s (in use %s): %d points, fit max %.3f px (%s) rms %.3f px, px per mm [%.4f %.4f; %.4f %.4f], "
		"mm per px [%.7f %.7f; %.7f %.7f], X %.4f um/px, Y %.4f um/px, rotation %+.4f / %+.4f deg, vs Set %+.3f%% %+.3f%%, step %.3f mm",
		pass ? "PASS" : "FAIL", _gs2Cal.useMeasured ? "Measured" : "Set", (int)pts.size(), fitPx, qPrintable(worst), fitRms,
		M[0][0], M[0][1], M[1][0], M[1][1], A[0][0], A[0][1], A[1][0], A[1][1], umX, umY, convX.angle, convY.angle, setX.pct, setY.pct, step);
	AuditLog::instance().log(QStringLiteral("GS2_CAMERA_DIRECTION"), QStringLiteral("%1: X %2 um/px, Y %3 um/px, %4 deg, fit %5 px")
		.arg(pass ? QStringLiteral("PASS") : QStringLiteral("FAIL"))
		.arg(umX, 0, 'f', 3).arg(umY, 0, 'f', 3).arg(convX.angle, 0, 'f', 3).arg(fitPx, 0, 'f', 2));

	if (fitOk) {
		auto& m = _gs2LastMeasure;
		m.measured = true;
		for (int r = 0; r < 2; r++)
			for (int k = 0; k < 2; k++) m.a[r][k] = A[r][k];
		m.umPerPxX = umX;
		m.umPerPxY = umY;
		m.angleDeg = (convX.angle + convY.angle) / 2.0;
		m.fitPx = fitPx;
		m.camAngleDeg = SystemData::instance()._camAngles.value(_camID, 0.0);
		m.imgW = c0.imgW;
		m.imgH = c0.imgH;
		m.atX = c0.wx;
		m.atY = c0.wy;
		m.atZ = z0 ? *z0 : 0.0;
		m.when = QDateTime::currentDateTime().toString(QStringLiteral("yyyy-MM-dd hh:mm"));
	}
	return true;
}

/*
* Stability test: the circle snapped kStabilitySnaps times where the camera stands, nothing moved.
* How far its centre and diameter wander from snap to snap is what the settle has to live with, so
* this is the number to tune the circle settings against. A snap with no circle or a bad fit is
* counted as missed and left out, as the settle leaves it out and snaps again.
*/
bool VisionApp::gs2StabilityTest(const gs2::CircleFinder& cf, QString& report, QString& why)
{
	QVector<Gs2Circle> shots;
	QStringList misses; //snaps with no circle or a bad fit: counted, not measured - the settle snaps those again
	QPointF follow;
	for (int i = 0; i < kStabilitySnaps; i++) {
		if (_stopRun) { why = QStringLiteral("stopped"); return false; }
		setProgress(_zsBusyWindow, QStringLiteral("Snap %1 of %2...").arg(i + 1).arg(kStabilitySnaps));
		QImage img;
		if (!guidedSnapFov(img, kSnapTimeoutMs)) { why = QStringLiteral("no image from the camera"); return false; }
		if (_stopRun) { why = QStringLiteral("stopped"); return false; }
		Gs2Circle c;
		bool none = false;
		QString miss;
		//the same circle every time: the one nearest the first one found
		if (!gs2FindCircle(img, cf, c, miss, shots.isEmpty() ? nullptr : &follow, nullptr, &none)) {
			if (!none) { why = QStringLiteral("snap %1: %2").arg(i + 1).arg(miss); return false; }
		}
		else if (c.fitErr > kMaxFitErrPx) {
			none = true;
			miss = badFit(c.diaPx, c.fitErr);
		}
		if (none) {
			misses << miss;
			ct::logger::info("[GuidedSetup2] Stability snap %d: missed - %s", i + 1, qPrintable(miss));
		}
		else {
			if (shots.isEmpty()) follow = QPointF(c.px, c.py);
			shots << c;
			ct::logger::info("[GuidedSetup2] Stability snap %d: centre (%.2f, %.2f) px, dia %.2f px, score %.2f, fit error %.3f, %d in range",
				i + 1, c.px, c.py, c.diaPx, c.score, c.fitErr, c.count);
		}
		zsPause(kStillGapMs);
	}
	const int missed = misses.size();
	if (shots.size() < 3) {
		why = QStringLiteral("the circle was missed in %1 of %2 snaps (last: %3)").arg(missed).arg(kStabilitySnaps).arg(misses.back());
		return false;
	}

	QVector<double> xs, ys;
	for (const auto& c : shots) { xs << c.px; ys << c.py; }
	std::sort(xs.begin(), xs.end());
	std::sort(ys.begin(), ys.end());
	const double mx = xs[xs.size() / 2], my = ys[ys.size() / 2];
	double worst = 0.0, sum2 = 0.0, dMin = shots[0].diaPx, dMax = dMin, sMin = shots[0].score, sMax = sMin, fitMax = -1.0;
	int agree = 0;
	for (const auto& c : shots) {
		const double d = std::hypot(c.px - mx, c.py - my);
		worst = std::max(worst, d);
		sum2 += d * d;
		if (d <= kJitterPx) agree++;
		dMin = std::min(dMin, c.diaPx); dMax = std::max(dMax, c.diaPx);
		sMin = std::min(sMin, c.score); sMax = std::max(sMax, c.score);
		fitMax = std::max(fitMax, c.fitErr);
	}
	const double rms = std::sqrt(sum2 / shots.size());
	const double umPerPx = shots[0].diaPx > 0.0 ? 1000.0 * shots[0].diaMm / shots[0].diaPx : 0.0;
	const QString verdict = missed == 0 && worst <= kStillPx ? QStringLiteral("Steady: the settle takes three snaps in a row.")
		: missed <= 2 && agree >= shots.size() - 1 ? QStringLiteral("Usable: a stray snap is outvoted and a missed one snapped again, but tighter is better.")
		: QStringLiteral("Unsteady: tighten the diameter range round the real edge, raise Edge smoothness or try the other polarity.");
	report = QStringLiteral("%1 snaps without moving%2. ").arg(kStabilitySnaps)
		.arg(missed > 0 ? QStringLiteral(", the circle missed in %1 (last: %2)").arg(missed).arg(misses.back()) : QString())
		+ QStringLiteral("Centre: furthest %1 px (%2 um) from the median, typical %3 px; %4 of %5 within %6 px. "
		"Diameter %7 .. %8 px. Score %9 .. %10%11.\n%12")
		.arg(worst, 0, 'f', 2).arg(std::lround(worst * umPerPx)).arg(rms, 0, 'f', 2).arg(agree).arg(shots.size()).arg(kJitterPx)
		.arg(dMin, 0, 'f', 1).arg(dMax, 0, 'f', 1).arg(sMin, 0, 'f', 1).arg(sMax, 0, 'f', 1)
		.arg(fitMax >= 0.0 ? QStringLiteral(", fit error up to %1").arg(fitMax, 0, 'f', 2) : QString())
		.arg(verdict);
	ct::logger::info("[GuidedSetup2] Stability test: furthest %.2f px, rms %.2f px, %d of %d within %.1f px, dia %.1f..%.1f px, score %.1f..%.1f, %d missed",
		worst, rms, agree, shots.size(), kJitterPx, dMin, dMax, sMin, sMax, missed);
	return true;
}

void VisionApp::gs2RunTool(int action)
{
	if (!_gs2Page || _zsBusy) return;
	if (action == GuidedSetup2Page::ToolSaveMeasured) { gs2SaveMeasured(); return; }

	QString what;
	const gs2::CircleFinder cf = _gs2Page->toolCircle(&what);
	if (!cf.dia.valid()) {
		_gs2Page->showToolResult(QStringLiteral("Type the %1 diameter range first (min above 0, max above min).").arg(what), false);
		return;
	}
	//the circles are drawn on the camera image, so that is what the big image area shows
	if (!ui.graphicsViewFOV->isVisible()) toggleFOVView();

	auto describe = [&](const QString& title, const Gs2Circle& c) {
		return QStringLiteral("%1 (%2): %3 circle(s) in range; nearest %4 mm across (%5 px), score %6%7, %8 / %9 mm from the image centre.")
			.arg(title, what).arg(c.count).arg(mm(c.diaMm)).arg(c.diaPx, 0, 'f', 1).arg(c.score, 0, 'f', 2)
			.arg(c.fitErr >= 0.0 ? QStringLiteral(", fit error %1").arg(c.fitErr, 0, 'f', 2) : QString())
			.arg(signedMm(c.offX), signedMm(c.offY));
	};

	if (action == GuidedSetup2Page::ToolFindInImage) {
		Gs2Circle c;
		QString why;
		if (!gs2FindCircle(_imageFOV, cf, c, why)) {
			_gs2Page->showToolResult(QStringLiteral("Find in image (%1): %2.").arg(what, why), false);
			return;
		}
		QString text = describe(QStringLiteral("Find in image"), c);
		//the image may be a loaded file: the stage position only means something if it was taken here
		auto& mc = MotionController::instance();
		const auto x = mc.is_init(_motionID) ? mc.get_position_mm(_motionID, 0, (int)Axis::X) : std::nullopt;
		const auto y = mc.is_init(_motionID) ? mc.get_position_mm(_motionID, 0, (int)Axis::Y) : std::nullopt;
		if (x && y)
			text += QStringLiteral("\nIf this image was taken where the camera is now (X %1, Y %2), the circle is at X %3, Y %4.")
				.arg(mm(*x), mm(*y), mm(*x + c.offX), mm(*y + c.offY));
		_gs2Page->showToolResult(text, true);
		ct::logger::info("[GuidedSetup2] Find in image: %d in range, nearest dia %.1f px at (%.2f, %.2f) px, %+.3f, %+.3f mm off centre",
			c.count, c.diaPx, c.px, c.py, c.offX, c.offY);
		return;
	}

	const bool direction = action == GuidedSetup2Page::ToolDirection;
	const bool centre = action == GuidedSetup2Page::ToolCentre;
	const bool stability = action == GuidedSetup2Page::ToolStability;
	const QString title = direction ? QStringLiteral("Check camera direction") : stability ? QStringLiteral("Stability test")
		: (centre ? QStringLiteral("Centre on circle") : QStringLiteral("Snap + find"));
	if (!gs2MachineBusy(title, direction
		? QStringLiteral("The camera centres on the circle, snaps it from a 3 x 3 grid +/- %1 mm round it (about 30 s), and comes back.\n"
			"Everything else is locked until it finishes.").arg(kDirStepMm, 0, 'f', 0)
		: stability ? QStringLiteral("The camera snaps the circle %1 times without moving.\nEverything else is locked until it finishes.").arg(kStabilitySnaps)
		: QStringLiteral("The camera snaps and moves onto the circle if it needs to.\nEverything else is locked until it finishes."))) return;

	const int trolleyEpoch = _zsTrolleyEpoch;
	QString why, text;
	Gs2Circle c;
	int moves = 0;
	bool pass = false;
	bool ok = direction ? gs2CheckDirection(cf, pass, text, why)
		: stability ? gs2StabilityTest(cf, text, why) : gs2LocateCircle(cf, centre, c, moves, why);
	const bool stopped = _stopRun;
	//the trolley guard changed meanwhile: what was found belongs to a table that is gone
	if (ok && trolleyEpoch != _zsTrolleyEpoch) {
		ok = false;
		why = QStringLiteral("the trolley was removed or attached meanwhile - the result was discarded");
		_gs2LastMeasure = gs2::CameraCal();
	}
	gs2MachineIdle();

	if (!ok) {
		text = QStringLiteral("%1 (%2): %3.").arg(title, what, stopped ? QStringLiteral("stopped - nothing else was moved") : why);
		ct::logger::warn("[GuidedSetup2] %s failed: %s", qPrintable(title), qPrintable(stopped ? QStringLiteral("stopped") : why));
	}
	else if (stability) text = QStringLiteral("%1 (%2): %3").arg(title, what, text);
	else if (!direction) {
		text = describe(title, c) + QStringLiteral("\nCircle at X %1, Y %2 (camera at X %3, Y %4; %5 move(s)).")
			.arg(mm(c.wx), mm(c.wy), mm(c.stageX), mm(c.stageY)).arg(moves);
		AuditLog::instance().log(QStringLiteral("GS2_CIRCLE"), QStringLiteral("%1 %2: X %3, Y %4").arg(title, what, mm(c.wx), mm(c.wy)));
	}
	else text = QStringLiteral("%1 (%2): %3").arg(title, what, text);
	if (_gs2Page) _gs2Page->showToolResult(text, ok && (!direction || pass));
	gs2RefreshCalUi();
}

// ── build step 3a: section A (Safety check 1) and section B (fid0) ─────────────────────────────

void VisionApp::gs2ShowPatternBox(bool show)
{
	if (_gs2PatternBox) {
		if (show) {
			const QRectF roi = _gs2Page ? _gs2Page->setup().fid0.patternRoi : QRectF();
			if (!roi.isEmpty()) _gs2PatternBox->setGeometry(roi);
			if (!ui.graphicsViewFOV->isVisible()) toggleFOVView(); //the box is on the camera image
		}
		_gs2PatternBox->setVisible(show);
	}
	if (_gs2Page) _gs2Page->setPatternBoxShown(show && _gs2PatternBox);
}

//home is where homing leaves the machine (JobThread::homeAxesExcept sets 0, 0, 0); Z goes up there first
bool VisionApp::gs2GoHome(QString& why)
{
	if (!zsMove({ { (int)Axis::Z, 0.0 } }, why)) return false;
	return zsMove({ { (int)Axis::X, 0.0 }, { (int)Axis::Y, 0.0 } }, why);
}

bool VisionApp::gs2SnapHome(QImage& small, QString& why)
{
	setProgress(_zsBusyWindow, QStringLiteral("Going to machine home..."));
	if (!gs2GoHome(why)) return false;
	zsPause(kHomeSettleMs + std::max(0, (int)SystemData::instance()._snapDelay_ms));
	if (_stopRun) { why = QStringLiteral("stopped"); return false; }
	setProgress(_zsBusyWindow, QStringLiteral("Snapping at home..."));
	QImage img;
	if (!guidedSnapFov(img, kSnapTimeoutMs)) { why = QStringLiteral("no image from the camera"); return false; }
	small = safetySmall(img);
	return true;
}

bool VisionApp::gs2SafetyScores(const QImage& small, const QStringList& emptyImages, QVector<double>& scores, QString& why)
{
	scores.clear();
	const QString dir = _gs2Page ? _gs2Page->recipeDir() : QString();
	for (const auto& name : emptyImages) {
		const QImage empty = QImage(dir + name).convertToFormat(QImage::Format_Grayscale8);
		if (empty.isNull()) { why = QStringLiteral("cannot read the empty image %1").arg(name); return false; }
		const double d = meanDifference(small, empty);
		if (d < 0.0) {
			why = QStringLiteral("%1 is %2 x %3 px but the camera gives %4 x %5 now - snap the empty images again")
				.arg(name).arg(empty.width()).arg(empty.height()).arg(small.width()).arg(small.height());
			return false;
		}
		scores << d;
	}
	return true;
}

/*
* Every copy of the pattern from kPatternAcceptance up, best first. MIL directly rather than
* mtrx::find_patterns, which keeps four 10000-entry arrays on the stack: here the result arrays
* are sized by the number of matches.
*/
bool VisionApp::gs2MatchPatterns(const QImage& image, const QString& patternPath, QVector<Gs2PatternHit>& hits, QString& why)
{
	hits.clear();
	if (!QFile::exists(patternPath)) { why = QStringLiteral("the pattern file %1 is missing - learn the pattern again").arg(patternPath); return false; }
	MIL_ID mBuf = mtrx::to_milID(milReady(image));
	if (mBuf == M_NULL) { why = QStringLiteral("the image could not be handed to the pattern finder"); return false; }
	MIL_ID mMono = mtrx::to_mono(mBuf);

	MIL_ID ctx = MpatRestoreA(M_DEFAULT_HOST, patternPath.toStdString().c_str(), M_NULL);
	if (ctx == M_NULL) {
		mtrx::free_buffer(mMono);
		mtrx::free_buffer(mBuf);
		why = QStringLiteral("the pattern file %1 could not be read - learn the pattern again").arg(patternPath);
		return false;
	}
	MIL_ID res = MpatAllocResult(M_DEFAULT_HOST, M_DEFAULT, M_NULL);
	MpatControl(ctx, M_ALL, M_ACCEPTANCE, kPatternAcceptance);
	MpatControl(ctx, M_ALL, M_CERTAINTY, 100.0); //never stop at the first good one: every copy is wanted
	MpatControl(ctx, M_ALL, M_NUMBER, kMaxPatternHits);
	MpatPreprocess(ctx, M_DEFAULT, M_NULL);
	MpatFind(ctx, mMono, res);
	MIL_INT n = 0;
	MpatGetResult(res, M_DEFAULT, M_NUMBER + M_TYPE_MIL_INT, &n);
	if (n > 0) {
		std::vector<MIL_DOUBLE> x((size_t)n), y((size_t)n), score((size_t)n);
		MpatGetResult(res, M_POSITION_X, x.data());
		MpatGetResult(res, M_POSITION_Y, y.data());
		MpatGetResult(res, M_SCORE, score.data());
		for (MIL_INT i = 0; i < n; i++) hits.push_back({ x[i], y[i], score[i] / 100.0 });
	}
	MpatFree(res);
	MpatFree(ctx);
	mtrx::free_buffer(mMono);
	mtrx::free_buffer(mBuf);
	std::sort(hits.begin(), hits.end(), [](const Gs2PatternHit& a, const Gs2PatternHit& b) { return a.score > b.score; });
	return true;
}

/*
* START step 2 for this recipe's settings: to the search XY at Start Z, sweep Z down for the sharpest
* image (the Z search's 2D sweep, never past Stop Z), match the learnt pattern there - the right
* trolley, docked - and locate the fid0 circle (snap + convert, moving onto it only if it is outside
* the middle half). The caller holds the machine lock.
*/
bool VisionApp::gs2FindFid0(const gs2::Setup& s, const QString& dir, gs2::Fid0Found& out, QString& why)
{
	const gs2::Fid0& f = s.fid0;
	const gs2::CircleFinder& cf = s.circle[gs2::CircleFid0];
	out = gs2::Fid0Found();
	const double umPerPx = ScaleManager::instance().um_per_px();
	if (!(umPerPx > 0.0)) { why = QStringLiteral("the camera scale (um per pixel) is not set - run Camera Scaling"); return false; }
	ZSearch2DConfig c;
	c.xySet = c.startSet = c.stopSet = true;
	c.x = f.xy.x; c.y = f.xy.y;
	c.startZ = f.startZ; c.stopZ = f.stopZ;
	c.stepMm = f.stepMm;
	c.adaptiveStep = true; //Max step: smaller steps as the sharpness starts to change
	c.refineStages = f.refineStages;
	c.boxPx = std::max(16, (int)std::lround(f.focusBoxMm * 1000.0 / umPerPx));

	QLabel* progress = _zsBusyWindow ? _zsBusyWindow->findChild<QLabel*>(QStringLiteral("busyProgress")) : nullptr;
	double z = 0.0;
	if (!zsFind2D(c, c.x, c.y, false, z, why, progress)) { why = QStringLiteral("fid0: %1").arg(why); return false; }

	setProgress(_zsBusyWindow, QStringLiteral("Matching the trolley pattern at Z %1...").arg(mm(z)));
	QImage img;
	if (!guidedSnapFov(img, kSnapTimeoutMs)) { why = QStringLiteral("fid0: no image from the camera at the focus Z"); return false; }
	QVector<Gs2PatternHit> hits;
	if (!gs2MatchPatterns(img, dir + f.patternFile, hits, why)) return false;
	QVector<Gs2PatternHit> good;
	for (const auto& h : hits) if (h.score >= f.minScore) good << h;
	const double bestScore = hits.isEmpty() ? 0.0 : hits[0].score;
	ct::logger::info("[GuidedSetup2] fid0 pattern at Z %.3f: %d cop(ies) found, %d at or above %.2f (best %.3f)", z, hits.size(), good.size(), f.minScore, bestScore);
	if (good.isEmpty()) {
		why = QStringLiteral("fid0 pattern score %1 is below %2: wrong trolley, not docked, or the light has changed").arg(bestScore, 0, 'f', 2).arg(f.minScore, 0, 'f', 2);
		return false;
	}

	/*
	* Each copy found could be any of the copies taught (the same mark twice side by side), so every
	* pairing is tried: "found copy h is taught copy k" means the trolley shifted by h - k, which puts
	* fid0 at its taught spot + that shift. A pairing counts only when a circle sits on that spot, and
	* the smallest shift wins - the wrong pairings point a whole mark-spacing beside fid0. So one clean
	* copy is enough, whichever it is, and a neighbouring circle is never taken.
	*/
	QVector<QPointF> taught{ QPointF(f.patternAtPx.x, f.patternAtPx.y) };
	taught += f.patternOtherCopiesPx;
	const QPointF fid0Taught(f.patternAtPx.x + f.patternToCirclePx.x, f.patternAtPx.y + f.patternToCirclePx.y);
	struct Candidate { Gs2PatternHit hit; QPointF spot; double shiftPx; };
	QVector<Candidate> onCircle;
	//every circle in the image, found once: the pairings below are only distances to these
	Gs2Circle any;
	QVector<QPointF> circles;
	QString w;
	if (!gs2FindCircle(img, cf, any, w, &fid0Taught, &circles) || circles.isEmpty()) {
		why = QStringLiteral("no circle in the fid0 diameter range in the image");
		return false;
	}
	const double imgMmPerPx = any.diaPx > 0.0 ? any.diaMm / any.diaPx : 0.0;
	if (!(imgMmPerPx > 0.0)) { why = QStringLiteral("the pixel-to-mm conversion is empty"); return false; } //0 would put every spot on a circle
	double nearestMm = std::numeric_limits<double>::max();
	for (const auto& h : good) {
		for (const auto& k : taught) {
			const QPointF shift(h.x - k.x(), h.y - k.y());
			const QPointF spot = fid0Taught + shift;
			double dPx = std::numeric_limits<double>::max();
			for (const auto& c : circles) dPx = std::min(dPx, std::hypot(c.x() - spot.x(), c.y() - spot.y()));
			const double offMm = dPx * imgMmPerPx;
			nearestMm = std::min(nearestMm, offMm);
			ct::logger::info("[GuidedSetup2]   copy at (%.1f, %.1f) score %.3f as taught copy (%.1f, %.1f): shift (%+.1f, %+.1f) px -> fid0 spot (%.1f, %.1f), nearest circle %.3f mm from it",
				h.x, h.y, h.score, k.x(), k.y(), shift.x(), shift.y(), spot.x(), spot.y(), offMm);
			if (offMm <= kExpectTolMm) onCircle << Candidate{ h, spot, std::hypot(shift.x(), shift.y()) };
		}
	}
	if (onCircle.isEmpty()) {
		why = QStringLiteral("no circle where the pattern puts fid0 (%1 cop(ies) of the pattern found; the nearest circle is %2 mm from a predicted spot, limit %3 mm)")
			.arg(good.size()).arg(mm(nearestMm)).arg(kExpectTolMm, 0, 'f', 1);
		return false;
	}
	int pick = 0;
	for (int i = 1; i < onCircle.size(); i++)
		if (onCircle[i].shiftPx < onCircle[pick].shiftPx) pick = i;
	ct::logger::info("[GuidedSetup2] fid0 placed by the copy at (%.1f, %.1f): trolley shift %.1f px since teach (%d pairing(s) on a circle)",
		onCircle[pick].hit.x, onCircle[pick].hit.y, onCircle[pick].shiftPx, onCircle.size());
	const QPointF expect = onCircle[pick].spot;
	const double score = onCircle[pick].hit.score;

	//fid0 is the circle at that spot; it is located still and averaged, and moved onto if far out
	Gs2Circle circle;
	int moves = 0;
	//centred on fid0, as when the pattern was learnt: the most accurate fid0, and the view the check below expects
	if (!gs2LocateCircle(cf, true, circle, moves, why, &expect)) { why = QStringLiteral("fid0: %1").arg(why); return false; }

	/*
	* A second, independent look from fid0 itself (Jimmy's rule): standing on the real fid0 the
	* pattern is there, at the place it was taught relative to fid0; standing on any other circle it
	* is not. One copy where a taught copy should be, within kExpectTolMm, is enough.
	*/
	setProgress(_zsBusyWindow, QStringLiteral("Checking the pattern from fid0..."));
	QImage fromFid0;
	if (!guidedSnapFov(fromFid0, kSnapTimeoutMs)) { why = QStringLiteral("fid0: no image from the camera on fid0"); return false; }
	QVector<Gs2PatternHit> again;
	if (!gs2MatchPatterns(fromFid0, dir + f.patternFile, again, why)) return false;
	const double mmPerPx = circle.diaPx > 0.0 ? circle.diaMm / circle.diaPx : 0.0;
	double confirmScore = -1.0, nearestCopyMm = std::numeric_limits<double>::max();
	for (const auto& h : again) {
		if (h.score < f.minScore) continue;
		for (const auto& k : taught) {
			//where taught copy k sits when fid0 is at circle.px, circle.py in this image
			const double ex = circle.px - (fid0Taught.x() - k.x()), ey = circle.py - (fid0Taught.y() - k.y());
			const double offMm = std::hypot(h.x - ex, h.y - ey) * mmPerPx;
			nearestCopyMm = std::min(nearestCopyMm, offMm);
			if (offMm <= kExpectTolMm) confirmScore = std::max(confirmScore, h.score);
		}
	}
	ct::logger::info("[GuidedSetup2] pattern from fid0: %d cop(ies) found, %s (nearest copy %.3f mm from a taught place)",
		again.size(), confirmScore >= 0.0 ? "confirmed" : "NOT where taught", nearestCopyMm < 1e9 ? nearestCopyMm : -1.0);
	if (confirmScore < 0.0) {
		why = QStringLiteral("standing on the circle found, the pattern is not where it was taught next to fid0 - this is probably not fid0");
		return false;
	}

	out.ok = true;
	out.x = circle.wx;
	out.y = circle.wy;
	out.z = z;
	out.score = std::min(score, confirmScore);
	out.when = QDateTime::currentDateTime().toString(QStringLiteral("yyyy-MM-dd hh:mm"));
	ct::logger::info("[GuidedSetup2] fid0 found at %.3f, %.3f, 2D Z %.3f, pattern %.3f / %.3f from fid0 (%d move(s) onto the circle)",
		out.x, out.y, out.z, score, confirmScore, moves);
	return true;
}

// ── build step 3b: 3D Z (C), the four corners and Check (D), the Locate test ─────────────────────

bool VisionApp::gs2SessionFid0(const gs2::Setup& s, const QString& dir, QString& why) const
{
	if (!_gs2Session.fid0 || _gs2Session.recipeDir != dir) {
		why = QStringLiteral("run Find in section B first - this works from fid0 as found on the trolley now");
		return false;
	}
	if (_gs2Session.trolleyEpoch != _zsTrolleyEpoch) {
		why = QStringLiteral("the trolley was removed or attached since fid0 was found - run Find in section B again");
		return false;
	}
	if (!s.fid0.startSet || !s.fid0.stopSet || _gs2Session.z2D < s.fid0.startZ || _gs2Session.z2D > s.fid0.stopZ) {
		why = QStringLiteral("fid0's 2D Z %1 is outside the Start Z .. Stop Z set now - run Find in section B again").arg(mm(_gs2Session.z2D));
		return false;
	}
	return true;
}

/*
* A slot, unit or inspection point is stored in the trolley frame, so teaching one converts the
* camera position with the frame a Check or Locate found on this trolley in this session. A frame
* found with other corners or another drawing size is another frame: the corners define it.
*/
bool VisionApp::gs2SessionRect(const gs2::Setup& s, const QString& dir, QString& why) const
{
	if (!gs2SessionFid0(s, dir, why)) return false;
	const auto& by = _gs2Session.rectBy;
	bool same = _gs2Session.rect && _gs2Session.pose.valid && by.widthMm == s.fidRect.widthMm && by.lengthMm == s.fidRect.lengthMm;
	for (int i = 0; i < 4 && same; i++)
		same = by.corner[i].set == s.fidRect.corner[i].set && by.corner[i].x == s.fidRect.corner[i].x && by.corner[i].y == s.fidRect.corner[i].y;
	if (!same) {
		why = QStringLiteral("run Check in section D (or Locate) on this trolley first - this is taught in the trolley frame it finds");
		return false;
	}
	return true;
}

bool VisionApp::gs2SlotInWorld(const gs2::Setup& s, const gs2::Slot& slot, const gs2geo::Pose& pose, QString& why)
{
	gs2geo::Pt c[4];
	gs2::slotCornersWorld(s, pose, slot, c);
	for (int i = 0; i < 4; i++)
		if (!zsCheckTarget((int)Axis::X, c[i].x, why) || !zsCheckTarget((int)Axis::Y, c[i].y, why)) {
			why = QStringLiteral("%1 would leave the World at its %2 corner (%3)").arg(slot.name, QLatin1String(kCornerName[i] + 3), why);
			return false;
		}
	return true;
}

/*
* START step 5's reach check, run before the visits too: every place production will drive to for
* these points must be inside the soft limits - refused and named, never clamped (jogLaser clamps
* today). The camera for 2D; both barcode readers for Barcode + OCR (the first one can fail and the
* other then reads), each at its taught camera-to-reader XYZ; the laser for 3D, from the point plus the
* camera-to-laser XY (or not, as H's 3D laser XY says) for the scan length and production's overshoot.
* Z is the 2D Z (2D, Barcode + OCR) or the 3D Centre Z (3D) plus the point's Z offset - never the
* camera-to-laser Z, as the 3D Centre Z was found with the laser itself.
*/
bool VisionApp::gs2CheckReach(const gs2::Setup& s, const QVector<gs2::RunPoint>& points, double z2D, double z3D, QString& why)
{
	auto& sd = SystemData::instance();
	int bad = 0;
	QString first;
	auto check = [&](const gs2::RunPoint& p, const QString& what, double x, double y, double z) {
		QString w;
		if (zsCheckTarget((int)Axis::X, x, w) && zsCheckTarget((int)Axis::Y, y, w) && zsCheckTarget((int)Axis::Z, z, w)) return;
		if (bad++ == 0) first = QStringLiteral("%1 %2, %3: %4").arg(s.units[p.unit].name, gs2::inspTypeName(p.type), what, w);
	};
	const bool alongY = s.scanAxis == QLatin1String("Y");
	const double along = s.scanNegative ? -1.0 : 1.0;
	for (const auto& p : points) {
		const auto& in = s.insp[p.type];
		if (p.type == gs2::Insp2D) check(p, QStringLiteral("camera"), p.camera.x, p.camera.y, z2D + in.zOffsetMm);
		else if (p.type == gs2::InspBarcodeOcr) {
			for (int r = 1; r <= 2; r++) {
				const bool taught = r == 1 ? sd._brR1Taught.load() : sd._brR2Taught.load();
				const double dx = !taught ? 0.0 : r == 1 ? sd._brR1dx.load() : sd._brR2dx.load();
				const double dy = !taught ? 0.0 : r == 1 ? sd._brR1dy.load() : sd._brR2dy.load();
				const double dz = !taught ? 0.0 : r == 1 ? sd._brR1dz.load() : sd._brR2dz.load();
				check(p, QStringLiteral("reader %1").arg(r), p.camera.x + dx, p.camera.y + dy, z2D + in.zOffsetMm + dz);
			}
		}
		else {
			//the stage for the scan: where the laser measures + the camera-to-laser XY; never its Z (z3D is the laser's own)
			const gs2geo::Pt read = gs2::scanReadPoint(s, p.camera, _laserConfig.offset.wx, _laserConfig.offset.wy);
			const double lx = read.x + _laserConfig.offset.wx, ly = read.y + _laserConfig.offset.wy, z = z3D + in.zOffsetMm;
			const double run = (in.scanLengthMm + kScanOvershootMm) * along;
			check(p, QStringLiteral("scan start"), lx, ly, z);
			check(p, QStringLiteral("scan end + overshoot"), alongY ? lx : lx + run, alongY ? ly + run : ly, z);
		}
	}
	if (bad == 0) return true;
	why = bad == 1 ? QStringLiteral("out of reach - %1").arg(first)
		: QStringLiteral("%1 places out of reach, the first %2").arg(bad).arg(first);
	return false;
}

/*
* START step 3, no Z search: with the head at fid0's 2D Z, the laser to fid0 + the 3D spot and one
* profiler read. The 2D Z already puts the trolley in the profiler's range (on the IPC the 3D Centre Z
* sat 0.09 mm from the 2D Focus Z), so the reading says where the 3D Centre Z is; it must lie inside
* fid0's Start Z .. Stop Z. Nothing flat read = fail. Z never goes below the 2D Z - as low as the head
* is known to be clear of this flat trolley - and is left there for D. With a difference taught the
* result must agree with it: a profiler reading another surface does not.
*/
bool VisionApp::gs2Find3DZ(const gs2::Setup& s, gs2::Z3DFound& out, QString& why)
{
	out = gs2::Z3DFound();
	const double z2D = _gs2Session.z2D;
	const QPointF at = z3dReadPoint(s, _gs2Session.fid0X, _gs2Session.fid0Y, _laserConfig.offset);
	const double camX = at.x(), camY = at.y();
	if (!zsCheck3DRead(camX, camY, why)) { why = QStringLiteral("3D Z: %1").arg(why); return false; }

	ct::logger::info("[GuidedSetup2] 3D Z: one read at the 2D Z %.3f, laser on %.3f, %.3f (spot fid0 + %.3f, %.3f, camera-to-laser XY %s)",
		z2D, camX, camY, s.z3d.offset.x, s.z3d.offset.y, s.z3d.addLaserOffset ? "added" : "NOT added");

	setProgress(_zsBusyWindow, QStringLiteral("3D Z: the laser to the 3D spot at the 2D Z..."));
	QLabel* progress = _zsBusyWindow ? _zsBusyWindow->findChild<QLabel*>(QStringLiteral("busyProgress")) : nullptr;
	zsearch::ProfileStats st;
	bool flat = false;
	if (!zsRead3DAt(camX, camY, z2D, s.z3d.maxSpreadMm, st, flat, why, progress)) { why = QStringLiteral("3D Z: %1").arg(why); return false; }
	if (!flat) {
		why = st.valid > 0
			? QStringLiteral("3D Z: at the 2D Z %1 the profiler read %2 points with a spread of %3 mm - not one flat surface (Max spread %4, enough points needed): is the laser on the flat area?")
				.arg(mm(z2D)).arg(st.valid).arg(mm(st.spread), mm(s.z3d.maxSpreadMm))
			: QStringLiteral("3D Z: the profiler read nothing at the 2D Z %1 - is the laser on the trolley there?").arg(mm(z2D));
		return false;
	}

	//where the reading would be the target; it must lie inside the range fid0's Find may search
	const double z = z2D + (s.z3d.targetMm - st.median) / kReadingPerZ;
	const double diff = z2D - z;
	ct::logger::info("[GuidedSetup2] 3D reading %.4f at the 2D Z %.3f (target %.4f): 3D Centre Z %.3f, 2D Z - 3D Z = %.3f mm%s",
		st.median, z2D, s.z3d.targetMm, z, diff, !s.z3d.diffSet ? ""
		: qPrintable(QStringLiteral(" (taught %1, tolerance %2)").arg(mm(s.z3d.diffMm), mm(s.z3d.diffTolMm))));
	if (z < s.fid0.startZ || z > s.fid0.stopZ) {
		why = QStringLiteral("3D Z: the reading %1 mm at the 2D Z %2 puts the 3D Centre Z at %3, outside fid0's Start Z %4 .. Stop Z %5")
			.arg(mm(st.median), mm(z2D), mm(z), mm(s.fid0.startZ), mm(s.fid0.stopZ));
		return false;
	}
	if (s.z3d.diffSet && std::abs(diff - s.z3d.diffMm) > s.z3d.diffTolMm) {
		why = QStringLiteral("3D Z: 2D Z - 3D Z is %1 mm on this trolley but %2 mm was taught (tolerance %3) - the profiler may be reading another surface")
			.arg(mm(diff), mm(s.z3d.diffMm), mm(s.z3d.diffTolMm));
		return false;
	}
	out.ok = true;
	out.z = z;
	out.z2D = z2D;
	out.reading = st.median;
	out.when = QDateTime::currentDateTime().toString(QStringLiteral("yyyy-MM-dd hh:mm"));
	return true;
}

/*
* fid0 again, where it was last located and at the current Z (the caller keeps that at the 2D Z). A
* corner's offset is measured against fid0 as it is now, so a slow drift of the trolley or an axis
* between teaches (IPC 2026-11-03: Y crept 0.16 mm in 8 minutes) does not go into it.
*/
bool VisionApp::gs2RelocateFid0(const gs2::Setup& s, Gs2Circle& out, QString& why)
{
	setProgress(_zsBusyWindow, QStringLiteral("Locating fid0 again..."));
	//no sideways move over a trolley that has been taken off or put on since fid0 was found
	if (_gs2Session.trolleyEpoch != _zsTrolleyEpoch) { why = QStringLiteral("the trolley was removed or attached - nothing else was moved"); return false; }
	const int w = CAMManager::instance().getWidth(QStringLiteral("cam1")), h = CAMManager::instance().getHeight(QStringLiteral("cam1"));
	if (w <= 0 || h <= 0) { why = QStringLiteral("the camera image size is unknown - is the camera connected?"); return false; }
	if (!zsMove({ { (int)Axis::X, _gs2Session.fid0X }, { (int)Axis::Y, _gs2Session.fid0Y } }, why)) return false;
	//fid0 is where it was a few minutes ago, so at the image centre: a neighbour 12.7 mm away is never taken
	const QPointF centre(w / 2.0, h / 2.0);
	int moves = 0;
	if (!gs2LocateCircle(s.circle[gs2::CircleFid0], false, out, moves, why, &centre)) { why = QStringLiteral("fid0 again: %1").arg(why); return false; }
	ct::logger::info("[GuidedSetup2] fid0 located again at %.3f, %.3f: %+.3f, %+.3f mm from where it was last located",
		out.wx, out.wy, out.wx - _gs2Session.fid0X, out.wy - _gs2Session.fid0Y);
	_gs2Session.fid0X = out.wx;
	_gs2Session.fid0Y = out.wy;
	_gs2Session.fid0AtMs = QDateTime::currentMSecsSinceEpoch();
	return true;
}

/*
* START step 4: fid0 located again at the 2D Z, then the four corners at that Z, the one nearest
* fid0 first and on round the rectangle: until one corner is found the trolley's turn is unknown and
* the error grows with the distance from fid0. Each is looked for where fid0 + its offset puts it,
* turned by what the corners before it showed (predictCorner). The drawing rectangle is fitted onto
* the four (shift + turn) and every corner's residual and the angle are judged against the recipe's
* limits. All of it is logged on every run, so the limits can be set from real numbers.
*/
bool VisionApp::gs2CheckRect(const gs2::Setup& s, Gs2RectCheck& out, QString& why)
{
	out = Gs2RectCheck();
	const gs2::FidRect& r = s.fidRect;
	//the 2D Z is gone back down to: it must still be inside the Start Z .. Stop Z of these settings
	if (!s.fid0.startSet || !s.fid0.stopSet || _gs2Session.z2D < s.fid0.startZ || _gs2Session.z2D > s.fid0.stopZ) {
		why = QStringLiteral("fid0's 2D Z %1 is outside Start Z .. Stop Z - run Find in section B again").arg(mm(_gs2Session.z2D));
		return false;
	}
	//down to the 2D Z at fid0 only, where the Find swept; every move after that is sideways at that Z
	setProgress(_zsBusyWindow, QStringLiteral("To fid0 at the 2D Z..."));
	if (!zsMoveToXY(_gs2Session.fid0X, _gs2Session.fid0Y, s.fid0.startZ, why)) return false;
	if (!zsMove({ { (int)Axis::Z, _gs2Session.z2D } }, why)) return false;
	Gs2Circle f;
	if (!gs2RelocateFid0(s, f, why)) return false;
	out.fid0 = { f.wx, f.wy };

	const QPointF centre(f.imgW / 2.0, f.imgH / 2.0);
	gs2geo::Pt taught[4];
	int first = 0;
	for (int i = 0; i < 4; i++) {
		taught[i] = { f.wx + r.corner[i].x, f.wy + r.corner[i].y };
		if (std::hypot(r.corner[i].x, r.corner[i].y) < std::hypot(r.corner[first].x, r.corner[first].y)) first = i;
	}
	std::vector<gs2geo::Pt> taughtDone, foundDone;
	for (int k = 0; k < 4; k++) {
		const int i = (first + k) % 4;
		//no sideways move over a trolley that has been taken off or put on during the Check
		if (_gs2Session.trolleyEpoch != _zsTrolleyEpoch) { why = QStringLiteral("the trolley was removed or attached during the Check"); return false; }
		const gs2geo::Pt expect = predictCorner(out.fid0, taughtDone, foundDone, taught[i]);
		setProgress(_zsBusyWindow, QStringLiteral("Corner %1 of 4: %2 at X %3, Y %4...").arg(k + 1).arg(QLatin1String(kCornerName[i]), mm(expect.x), mm(expect.y)));
		Gs2Circle c;
		int moves = 0;
		if (!zsMove({ { (int)Axis::X, expect.x }, { (int)Axis::Y, expect.y } }, why)
			|| !gs2LocateCircle(s.circle[gs2::CircleCorner], false, c, moves, why, &centre, r.searchMm)) {
			why = QStringLiteral("%1 not found: %2%3").arg(QLatin1String(kCornerName[i]), why,
				k == 0 ? QStringLiteral(" - is the trolley turned more than when it was taught (raise Corner search in D), or not the one taught?") : QString());
			return false;
		}
		out.corner[i] = { c.wx, c.wy };
		taughtDone.push_back(taught[i]);
		foundDone.push_back(out.corner[i]);
		ct::logger::info("[GuidedSetup2] %s at %.3f, %.3f: %.3f mm from where expected, %.3f mm from fid0 + its taught offset (%d move(s))",
			kCornerName[i], c.wx, c.wy, gs2geo::dist(out.corner[i], expect), gs2geo::dist(out.corner[i], taught[i]), moves);
	}

	out.fit = gs2geo::fitRect(out.corner, r.widthMm, r.lengthMm);
	const double angleDeg = out.fit.pose.angle * 180.0 / kPi;
	int worst = 0;
	for (int i = 1; i < 4; i++) if (out.fit.residual[i] > out.fit.residual[worst]) worst = i;
	const bool residualOk = out.fit.residual[worst] <= r.residualLimitMm;
	const bool angleOk = std::abs(angleDeg) <= r.maxAngleDeg;
	out.pass = residualOk && angleOk;
	ct::logger::info("[GuidedSetup2] fidRect %s: centre %.3f, %.3f, angle %.4f deg (limit %.2f); residuals TL %.4f TR %.4f BR %.4f BL %.4f mm (limit %.3f); "
		"measured %.3f x %.3f mm, drawing %.3f x %.3f", out.pass ? "PASS" : "FAIL", out.fit.pose.cx, out.fit.pose.cy, angleDeg, r.maxAngleDeg,
		out.fit.residual[0], out.fit.residual[1], out.fit.residual[2], out.fit.residual[3], r.residualLimitMm,
		out.fit.widthMeasured, out.fit.lengthMeasured, r.widthMm, r.lengthMm);

	QStringList failed;
	if (!residualOk) failed << QStringLiteral("%1 is %2 mm off the drawing rectangle (limit %3)").arg(QLatin1String(kCornerName[worst]), mm(out.fit.residual[worst]), mm(r.residualLimitMm));
	if (!angleOk) failed << QStringLiteral("the trolley is turned %1 deg (limit %2)").arg(QString::number(angleDeg, 'f', 3), QString::number(r.maxAngleDeg, 'f', 2));
	out.report = (failed.isEmpty() ? QString() : failed.join(QStringLiteral("; ")) + QStringLiteral(". "))
		+ QStringLiteral("Residuals TL %1, TR %2, BR %3, BL %4 mm (limit %5); angle %6 deg (limit %7); measured %8 x %9 mm, drawing %10 x %11")
		.arg(mm(out.fit.residual[0]), mm(out.fit.residual[1]), mm(out.fit.residual[2]), mm(out.fit.residual[3]), mm(r.residualLimitMm))
		.arg(QString::number(angleDeg, 'f', 3), QString::number(r.maxAngleDeg, 'f', 2), mm(out.fit.widthMeasured), mm(out.fit.lengthMeasured))
		.arg(mm(r.widthMm), mm(r.lengthMm));
	return true;
}

//a passed Check as section D keeps it
static gs2::FidRectFound rectFound(const gs2geo::RectFit& fit)
{
	gs2::FidRectFound f;
	f.ok = true;
	f.cx = fit.pose.cx;
	f.cy = fit.pose.cy;
	f.angleDeg = fit.pose.angle * 180.0 / kPi;
	for (int i = 0; i < 4; i++) f.residual[i] = fit.residual[i];
	f.widthMeasured = fit.widthMeasured;
	f.lengthMeasured = fit.lengthMeasured;
	f.when = QDateTime::currentDateTime().toString(QStringLiteral("yyyy-MM-dd hh:mm"));
	return f;
}

void VisionApp::gs2Teach(int action)
{
	if (!_gs2Page || _zsBusy) return;
	using P = GuidedSetup2Page;
	if (_gs2Page->recipeDir().isEmpty()) { showTeachResult(_gs2Page, action, QStringLiteral("Open a recipe first."), false); return; }
	//a copy: the page's working copy can be replaced while an action pumps events (a recipe load)
	const gs2::Setup s = _gs2Page->setup();
	const QString dir = _gs2Page->recipeDir();
	//to the millisecond: two Learns or Snaps in one second must never share (and later delete) a file
	const QString stamp = QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd-HHmmss-zzz"));

	//── no motion: Set Start / Stop Z, Learn pattern
	if (action == P::TeachSetStart || action == P::TeachSetStop) {
		const auto z = MotionController::instance().get_position_mm(_motionID, 0, (int)Axis::Z);
		if (!z) { _gs2Page->showFid0Result(QStringLiteral("Could not read the Z position - is motion connected?"), false); return; }
		const bool start = action == P::TeachSetStart;
		_gs2Page->modify([&](gs2::Setup& w) {
			if (start) { w.fid0.startSet = true; w.fid0.startZ = *z; }
			else { w.fid0.stopSet = true; w.fid0.stopZ = *z; }
		});
		_gs2Page->showFid0Result(QStringLiteral("%1 Z set to %2.").arg(start ? QStringLiteral("Start") : QStringLiteral("Stop"), mm(*z)), false);
		AuditLog::instance().log(QStringLiteral("GS2_FID0_Z"), QStringLiteral("%1 %2").arg(start ? QStringLiteral("start") : QStringLiteral("stop"), mm(*z)));
		return;
	}
	if (action == P::TeachLearnPattern) {
		if (!_gs2PatternBox || !_gs2PatternBox->isVisible()) {
			_gs2Page->showFid0Result(QStringLiteral("Press Pattern box, place the box on the camera image, then Learn pattern."), false);
			return;
		}
		const QImage fov = _imageFOV;
		const QRect roi = _gs2PatternBox->getGeometry().toRect().intersected(fov.rect());
		if (fov.isNull() || roi.width() < 32 || roi.height() < 32) {
			_gs2Page->showFid0Result(QStringLiteral("The box must lie on the camera image and be at least 32 x 32 px."), false);
			return;
		}
		//the pattern is learnt WITH where fid0 sits from it, so the camera must be on fid0 now
		if (!s.circle[gs2::CircleFid0].dia.valid()) { _gs2Page->showFid0Result(QStringLiteral("Set the fid0 circle diameter in Camera and circle finder first."), false); return; }
		Gs2Circle fid0Here;
		QString circleWhy;
		if (!gs2FindCircle(fov, s.circle[gs2::CircleFid0], fid0Here, circleWhy)) {
			_gs2Page->showFid0Result(QStringLiteral("Centre the camera on fid0 first (Teach XY): %1.").arg(circleWhy), false);
			return;
		}
		if (std::hypot(fid0Here.offX, fid0Here.offY) > kExpectTolMm) {
			_gs2Page->showFid0Result(QStringLiteral("The nearest circle is %1 mm from the image centre - centre the camera on fid0 first (Teach XY), then learn.")
				.arg(mm(std::hypot(fid0Here.offX, fid0Here.offY))), false);
			return;
		}
		const QString name = QStringLiteral("gs2_fid0_%1.mmf").arg(stamp);
		const QImage crop = fov.copy(roi).convertToFormat(QImage::Format_Grayscale8);
		MIL_ID mBuf = mtrx::to_milID(crop);
		bool learnt = false;
		if (mBuf != M_NULL) {
			MIL_ID mMono = mtrx::to_mono(mBuf);
			mtrx::PatternInput in;
			in.filename = (dir + name).toStdString();
			in.min_score = 0.0;
			in.learn_x = 0;
			in.learn_y = 0;
			in.learn_w = crop.width();
			in.learn_h = crop.height();
			in.smoothness = 85;
			in.enable_angle = false; //the trolley turns at most a degree
			mtrx::PatternOutput out;
			learnt = mtrx::learn_pattern(mMono, in, out);
			mtrx::free_buffer(mMono);
			mtrx::free_buffer(mBuf);
		}
		//every copy in this image; the learnt one is the copy inside the box (an identical neighbour
		//can score almost the same, so "the best match" is not good enough here)
		QVector<Gs2PatternHit> hits;
		QString why;
		if (learnt) learnt = gs2MatchPatterns(fov, dir + name, hits, why);
		const QPointF boxCentre = QRectF(roi).center();
		int self = -1;
		for (int i = 0; i < hits.size(); i++)
			if (self < 0 || std::hypot(hits[i].x - boxCentre.x(), hits[i].y - boxCentre.y()) < std::hypot(hits[self].x - boxCentre.x(), hits[self].y - boxCentre.y())) self = i;
		const bool selfOk = self >= 0 && hits[self].score >= 0.9 && std::hypot(hits[self].x - boxCentre.x(), hits[self].y - boxCentre.y()) <= 0.25 * std::min(roi.width(), roi.height());
		if (!learnt || !selfOk) {
			QFile::remove(dir + name);
			_gs2Page->showFid0Result(!why.isEmpty() ? QStringLiteral("The pattern could not be learnt: %1.").arg(why)
				: QStringLiteral("The pattern could not be learnt (it does not find itself in its own box) - try a more distinctive area."), false);
			return;
		}
		const double score = hits[self].score;
		const QPointF patternAt(hits[self].x, hits[self].y);
		int copies = 0;
		QVector<QPointF> others;
		for (int i = 0; i < hits.size(); i++) {
			if (hits[i].score < s.fid0.minScore) continue;
			copies++;
			if (i != self) others << QPointF(hits[i].x, hits[i].y);
		}
		const QPointF toCircle(fid0Here.px - patternAt.x(), fid0Here.py - patternAt.y());
		_gs2Page->modify([&](gs2::Setup& w) {
			w.fid0.patternFile = name;
			w.fid0.patternRoi = QRectF(roi);
			w.fid0.patternToCirclePx = { true, toCircle.x(), toCircle.y() };
			w.fid0.patternAtPx = { true, patternAt.x(), patternAt.y() };
			w.fid0.patternOtherCopiesPx = others;
		});
		_gs2Page->showFid0Result(QStringLiteral("Pattern learnt from a %1 x %2 px box (score on this image %3); fid0 is %4 mm from it. %5Run Find to prove it on the trolley.")
			.arg(roi.width()).arg(roi.height()).arg(score, 0, 'f', 2).arg(mm(std::hypot(toCircle.x(), toCircle.y()) * fid0Here.diaMm / fid0Here.diaPx))
			.arg(copies > 1 ? QStringLiteral("%1 copies of it are in view, all remembered: at Find any one of them is enough. ").arg(copies) : QString()), false);
		ct::logger::info("[GuidedSetup2] fid0 pattern learnt: %s, box %d,%d %dx%d, self score %.3f at (%.1f, %.1f) px, %d cop(ies) >= %.2f in view, fid0 circle at (%.1f, %.1f) px -> offset (%.1f, %.1f) px",
			qPrintable(name), roi.x(), roi.y(), roi.width(), roi.height(), score, patternAt.x(), patternAt.y(), copies, s.fid0.minScore,
			fid0Here.px, fid0Here.py, toCircle.x(), toCircle.y());
		AuditLog::instance().log(QStringLiteral("GS2_FID0_PATTERN"), name);
		return;
	}

	//── no motion: C's Set spot and Set Difference
	if (action == P::TeachZ3DSpot) {
		QString w;
		if (!gs2SessionFid0(s, dir, w)) { _gs2Page->showZ3DResult(QStringLiteral("Set spot: %1.").arg(w), false); return; }
		auto& mc = MotionController::instance();
		const auto x = mc.get_position_mm(_motionID, 0, (int)Axis::X);
		const auto y = mc.get_position_mm(_motionID, 0, (int)Axis::Y);
		if (!x || !y) { _gs2Page->showZ3DResult(QStringLiteral("Could not read the X / Y position - is motion connected?"), false); return; }
		const bool addLaser = s.z3d.addLaserOffset;
		const double wx = _laserConfig.offset.wx, wy = _laserConfig.offset.wy;
		const double lx = addLaser ? *x + wx : *x, ly = addLaser ? *y + wy : *y; //the stage for the 3D read
		if (!zsCheckTarget((int)Axis::X, lx, w) || !zsCheckTarget((int)Axis::Y, ly, w)) {
			_gs2Page->showZ3DResult(QStringLiteral("Set spot: the laser cannot reach this spot - %1 (camera point + laser offset).").arg(w), false);
			return;
		}
		const double dx = *x - _gs2Session.fid0X, dy = *y - _gs2Session.fid0Y;
		_gs2Page->modify([&](gs2::Setup& ws) { ws.z3d.offset = { true, dx, dy }; });
		_gs2Session.has3D = false; //a 3D Find at the old spot is not this spot's difference
		_gs2Page->showZ3DResult(QStringLiteral("3D spot set: the camera at X %1, Y %2 = fid0 %3, %4 mm; %5 Run Find.")
			.arg(mm(*x), mm(*y), signedMm(dx), signedMm(dy), addLaser
				? QStringLiteral("the stage goes to X %1, Y %2 so the laser reads here.").arg(mm(lx), mm(ly))
				: QStringLiteral("the stage comes back here (camera-to-laser XY not added), so the laser reads X %1, Y %2.").arg(mm(*x - wx), mm(*y - wy))), false);
		ct::logger::info("[GuidedSetup2] 3D spot set: camera %.3f, %.3f = fid0 %+.3f, %+.3f; stage for the read %.3f, %.3f (camera-to-laser XY %s)",
			*x, *y, dx, dy, lx, ly, addLaser ? "added" : "NOT added");
		AuditLog::instance().log(QStringLiteral("GS2_Z3D_SPOT"), QStringLiteral("fid0 %1, %2").arg(signedMm(dx), signedMm(dy)));
		return;
	}
	if (action == P::TeachZ3DDiff) {
		QString w;
		if (!gs2SessionFid0(s, dir, w)) { _gs2Page->showZ3DResult(QStringLiteral("Set Difference: %1.").arg(w), false); return; }
		if (!_gs2Session.has3D || _gs2Session.z3DAddLaser != s.z3d.addLaserOffset) {
			_gs2Page->showZ3DResult(QStringLiteral("Run Find here first, with Laser XY as it is now: the difference is taken from it."), false);
			return;
		}
		const double diff = std::round((_gs2Session.z3DFrom2D - _gs2Session.z3D) * 1000.0) / 1000.0;
		_gs2Page->modify([&](gs2::Setup& ws) { ws.z3d.diffSet = true; ws.z3d.diffMm = diff; });
		_gs2Page->showZ3DResult(QStringLiteral("Difference set: 2D Z %1 - 3D Z %2 = %3 mm. Run Find again: from now on it must agree within the tolerance.")
			.arg(mm(_gs2Session.z3DFrom2D), mm(_gs2Session.z3D), mm(diff)), false);
		ct::logger::info("[GuidedSetup2] 2D - 3D difference set: %.3f - %.3f = %.3f mm", _gs2Session.z3DFrom2D, _gs2Session.z3D, diff);
		AuditLog::instance().log(QStringLiteral("GS2_Z3D_DIFF"), mm(diff));
		return;
	}

	//── sections E, F and H (build step 4a): positions go into the trolley frame the session holds
	static const char* const kSlotCorner[4] = { "TL", "TR", "BR", "BL" };   //gs2geo::Corner order
	const int addCorner = _gs2Page->slotAddCorner();                        //read once: the busy window is modal
	auto r3 = [](double v) { return std::round(v * 1000.0) / 1000.0; };
	//the frame the Check found, moved by what fid0 has moved since: the trolley or an axis creeps (IPC
	//2026-11-03: 0.09-0.16 mm in 8 min). The circle teaches locate fid0 again first, so theirs is fresh
	auto framePose = [&]() {
		gs2geo::Pose p = _gs2Session.pose;
		p.cx += _gs2Session.fid0X - _gs2Session.rectFid0.x;
		p.cy += _gs2Session.fid0Y - _gs2Session.rectFid0.y;
		return p;
	};
	auto toFrame = [&](double x, double y) { return gs2geo::toLocal(framePose(), { x, y }); };
	//for what takes the camera position as it is (no fid0 trip): how old the frame's fid0 is
	auto fid0Age = [&]() -> QString {
		const qint64 min = (QDateTime::currentMSecsSinceEpoch() - _gs2Session.fid0AtMs) / 60000;
		return min < 5 ? QString()
			: QStringLiteral(" fid0 was last located %1 min ago: if the trolley may have crept since, run Find in B and do this again.").arg(min);
	};
	auto camera = [&](gs2geo::Pt& here, QString& w) {
		auto& mc = MotionController::instance();
		const auto x = mc.get_position_mm(_motionID, 0, (int)Axis::X);
		const auto y = mc.get_position_mm(_motionID, 0, (int)Axis::Y);
		if (!x || !y) { w = QStringLiteral("could not read the X / Y position - is motion connected?"); return false; }
		here = { *x, *y };
		return true;
	};
	//a slot circle is centred sideways at the current Z: at the 2D Z or a little above, never below it (+ Z is down)
	auto atTwoDZ = [&](QString& w) {
		const auto z = MotionController::instance().get_position_mm(_motionID, 0, (int)Axis::Z);
		if (!z) { w = QStringLiteral("could not read the Z position - is motion connected?"); return false; }
		if (*z < _gs2Session.z2D - kTeachZTolMm || *z > _gs2Session.z2D + kTeachZBelowMm) {
			w = QStringLiteral("the camera is at Z %1 but fid0's 2D Z is %2: jog Z to %2 (not lower) so the circle is in focus").arg(mm(*z), mm(_gs2Session.z2D));
			return false;
		}
		return true;
	};
	//E: one of the slot's two circles, found at world (x, y); the size once both are there
	auto keepSlotCircle = [&](bool br, double x, double y) -> QString {
		const gs2geo::Pt p = toFrame(x, y);
		const gs2::TaughtXY xy{ true, r3(p.x), r3(p.y) };
		gs2::SlotSize z;
		bool restarted = false;
		_gs2Page->modify([&](gs2::Setup& w) {
			auto& ss = w.slotSize;
			//with a size already measured the other circle may be another slot's: this starts a new pair
			restarted = ss.set;
			if (restarted) (br ? ss.tl : ss.br) = gs2::TaughtXY();
			(br ? ss.br : ss.tl) = xy;
			ss.set = false;
			ss.widthMm = ss.lengthMm = 0.0;
			if (ss.tl.set && ss.br.set && ss.br.x > ss.tl.x && ss.br.y > ss.tl.y) {
				ss.set = true;
				ss.widthMm = r3(ss.br.x - ss.tl.x);
				ss.lengthMm = r3(ss.br.y - ss.tl.y);
			}
			z = ss;
		});
		ct::logger::info("[GuidedSetup2] slot %s circle at %.3f, %.3f = trolley frame %.3f, %.3f; slot size %s", br ? "BR" : "TL", x, y, xy.x, xy.y,
			z.set ? qPrintable(QStringLiteral("%1 x %2 mm").arg(mm(z.widthMm), mm(z.lengthMm))) : "not set");
		AuditLog::instance().log(QStringLiteral("GS2_SLOT_SIZE"), QStringLiteral("%1 %2, %3").arg(br ? QStringLiteral("BR") : QStringLiteral("TL"), mm(xy.x), mm(xy.y)));
		QString t = QStringLiteral("%1 circle at x %2, y %3 (trolley frame). ").arg(br ? QStringLiteral("BR") : QStringLiteral("TL"), mm(xy.x), mm(xy.y));
		if (z.set) return t + QStringLiteral("Slot size %1 x %2 mm.").arg(mm(z.widthMm), mm(z.lengthMm));
		if (!z.tl.set || !z.br.set) return t + QStringLiteral("Now Set %1 on the same slot%2.").arg(br ? QStringLiteral("TL") : QStringLiteral("BR"),
			restarted ? QStringLiteral(" (the size measured before is cleared)") : QString());
		return t + QStringLiteral("BR is not right of and below TL (BR - TL = %1, %2 mm): set the TL and BR circles of one slot.")
			.arg(mm(z.br.x - z.tl.x), mm(z.br.y - z.tl.y));
	};
	//F: the slot whose `corner` is at world (x, y), formed from E's size; w = what happened
	auto addSlot = [&](int corner, double x, double y, QString& w) {
		static const double kx[4] = { 0.5, -0.5, -0.5, 0.5 }, ky[4] = { 0.5, 0.5, -0.5, -0.5 };
		gs2::Setup t = _gs2Page->setup();
		const double sw = t.slotSize.widthMm, sl = t.slotSize.lengthMm;
		const gs2geo::Pt p = toFrame(x, y);
		const gs2::Slot slot = gs2::newSlot(t, r3(p.x + kx[corner] * sw), r3(p.y + ky[corner] * sl));
		for (const auto& o : t.slotList)
			if (gs2geo::slotsTouch({ slot.cx, slot.cy }, { o.cx, o.cy }, sw, sl)) {
				w = QStringLiteral("this slot would touch %1 - is it already added, or the wrong corner picked?").arg(o.name);
				return false;
			}
		if (!gs2SlotInWorld(t, slot, framePose(), w)) return false;
		_gs2Page->modify([&](gs2::Setup& ws) {
			ws.slotList.append(slot);
			ws.nextSlotId = t.nextSlotId;
			gs2::syncUnits(ws);
		});
		w = QStringLiteral("%1 added from its %2 corner: centre x %3, y %4 (trolley frame), %5 units.")
			.arg(slot.name, QLatin1String(kSlotCorner[corner]), mm(slot.cx), mm(slot.cy)).arg(t.unitLayout.rows * t.unitLayout.cols);
		ct::logger::info("[GuidedSetup2] slot %s (%s) added from its %s corner at %.3f, %.3f: centre %.3f, %.3f (trolley frame)",
			qPrintable(slot.name), qPrintable(slot.id), kSlotCorner[corner], x, y, slot.cx, slot.cy);
		AuditLog::instance().log(QStringLiteral("GS2_SLOT_ADD"), QStringLiteral("%1 %2, %3").arg(slot.name, mm(slot.cx), mm(slot.cy)));
		return true;
	};

	//F: Add from a corner taken by eye (TR, BL); TL and BR are circles and move the machine below
	if (action == P::TeachSlotAdd) {
		QString w;
		if (!gs2SessionRect(s, dir, w)) { _gs2Page->showSlotsResult(QStringLiteral("Add: %1.").arg(w), false); return; }
		if (!s.slotSize.set) { _gs2Page->showSlotsResult(QStringLiteral("Add: measure the slot size (E) first - a slot is formed from it."), false); return; }
		if (addCorner == gs2geo::TR || addCorner == gs2geo::BL) {
			gs2geo::Pt here;
			const bool added = camera(here, w) && addSlot(addCorner, here.x, here.y, w);
			_gs2Page->showSlotsResult(added ? w + fid0Age() : QStringLiteral("Add: %1.").arg(w), false);
			return;
		}
	}
	//F: Interpolate between the 2 slots selected in the list - an edit, but the World check needs the soft limits
	if (action == P::TeachSlotInterpolate) {
		auto say = [&](const QString& t) { _gs2Page->showSlotsResult(QStringLiteral("Interpolate: %1.").arg(t), false); };
		const QStringList ids = _gs2Page->selectedSlotIds();
		if (ids.size() != 2) { say(QStringLiteral("select exactly 2 slots of one row in the list")); return; }
		if (!s.slotSize.set) { say(QStringLiteral("measure the slot size (E) first")); return; }
		const gs2::Slot* a = gs2::findSlot(s, ids[0]);
		const gs2::Slot* b = gs2::findSlot(s, ids[1]);
		if (!a || !b) { say(QStringLiteral("the selected slots are no longer there - select them again")); return; }
		if (a->cx > b->cx) std::swap(a, b);
		const double sw = s.slotSize.widthMm, sl = s.slotSize.lengthMm;
		const double rowDy = std::abs(a->cy - b->cy);
		if (rowDy > sl / 2) {
			say(QStringLiteral("%1 and %2 are not in one row - they are %3 mm apart in Y").arg(a->name, b->name, mm(rowDy)));
			return;
		}
		//both ends move to the average Y: only by a little, or one of them was taught on the wrong spot
		if (rowDy > kRowYTolMm) {
			say(QStringLiteral("%1 and %2 are %3 mm apart in Y, and a row lines up within %4 mm - teach one of them again")
				.arg(a->name, b->name, mm(rowDy)).arg(kRowYTolMm, 0, 'f', 1));
			return;
		}
		const int n = _gs2Page->slotsBetween();
		const double pitch = (b->cx - a->cx) / (n + 1);
		if (pitch <= sw) {
			say(QStringLiteral("%1 slots between %2 and %3 would touch: %4 mm apart, and a slot is %5 mm wide").arg(n).arg(a->name, b->name, mm(pitch), mm(sw)));
			return;
		}
		const double y = r3((a->cy + b->cy) / 2);
		gs2::Setup t = s;
		QStringList row{ a->id, b->id }, added;
		for (auto& o : t.slotList) if (row.contains(o.id)) o.cy = y;
		for (int k = 1; k <= n; k++) {
			const gs2::Slot ns = gs2::newSlot(t, r3(a->cx + k * pitch), y);
			t.slotList.append(ns);
			row << ns.id;
			added << ns.name;
		}
		for (const auto& p : t.slotList) {
			if (!row.contains(p.id)) continue;
			for (const auto& o : t.slotList)
				if (!row.contains(o.id) && gs2geo::slotsTouch({ p.cx, p.cy }, { o.cx, o.cy }, sw, sl)) { say(QStringLiteral("%1 would touch %2").arg(p.name, o.name)); return; }
		}
		//the World check where the trolley was found this session, else where it was taught
		QString w;
		const gs2geo::Pose pose = gs2SessionRect(s, dir, w) ? framePose() : gs2::taughtFit(s).pose;
		if (!pose.valid) { say(QStringLiteral("teach fid0 and the four corners first - the World check needs the trolley")); return; }
		for (const auto& p : t.slotList) if (row.contains(p.id) && !gs2SlotInWorld(t, p, pose, w)) { say(w); return; }
		_gs2Page->modify([&](gs2::Setup& ws) {
			ws.slotList = t.slotList;
			ws.nextSlotId = t.nextSlotId;
			gs2::syncUnits(ws);
		});
		_gs2Page->showSlotsResult(QStringLiteral("%1 added between %2 and %3, %4 mm apart; all of them at y %5 (trolley frame) - %2 and %3 moved %6 mm in Y to get there.")
			.arg(added.join(QStringLiteral(", ")), a->name, b->name, mm(pitch), mm(y), mm(rowDy / 2)), false);
		ct::logger::info("[GuidedSetup2] slots %s interpolated between %s and %s: pitch %.3f, y %.3f", qPrintable(added.join(QStringLiteral(", "))),
			qPrintable(a->name), qPrintable(b->name), pitch, y);
		AuditLog::instance().log(QStringLiteral("GS2_SLOT_INTERPOLATE"), QStringLiteral("%1 between %2 and %3").arg(added.join(QStringLiteral(", ")), a->name, b->name));
		return;
	}
	//H: Set - the camera position as the offset from the nearest unit's reference
	if (action == P::TeachInsp2D || action == P::TeachInspBarcodeOcr || action == P::TeachInsp3D) {
		const int type = action - P::TeachInsp2D;
		const QString what = gs2::inspTypeName((gs2::InspType)type);
		auto say = [&](const QString& t) { _gs2Page->showInspResult(QStringLiteral("Set %1: %2.").arg(what, t), false); };
		QString w;
		const auto& u = s.unitLayout;
		if (!gs2SessionRect(s, dir, w)) { say(w); return; }
		if (s.units.isEmpty() || u.widthMm <= 0.0 || u.lengthMm <= 0.0) { say(QStringLiteral("add slots (F) and type the unit size (G) first - the point is taken from the nearest unit")); return; }
		gs2geo::Pt here;
		if (!camera(here, w)) { say(w); return; }
		const gs2geo::Pt p = toFrame(here.x, here.y);
		int best = -1;
		double bestD = 0.0;
		gs2geo::Pt centre;
		for (int i = 0; i < s.units.size(); i++) {
			const gs2::Slot* sl = gs2::findSlot(s, s.units[i].slotId);
			if (!sl) continue;
			const gs2geo::Pt c = gs2::unitCentreLocal(s, *sl, s.units[i].row, s.units[i].col);
			const double d = std::hypot(p.x - c.x, p.y - c.y);
			if (best < 0 || d < bestD) { best = i; bestD = d; centre = c; }
		}
		if (best < 0) { say(QStringLiteral("no unit has a slot")); return; }
		const int ref = s.insp[type].ref;
		const gs2geo::Pt r = gs2geo::unitRefPoint(ref, u.widthMm, u.lengthMm);
		const double dx = r3(p.x - (centre.x + r.x)), dy = r3(p.y - (centre.y + r.y));
		if (std::abs(dx) > 500.0 || std::abs(dy) > 500.0) {   //the page's offset boxes hold +/-500
			say(QStringLiteral("the camera is %1, %2 mm from %3's %4 - an offset holds at most 500 mm").arg(mm(dx), mm(dy), s.units[best].name, gs2::refName(ref)));
			return;
		}
		_gs2Page->modify([&](gs2::Setup& ws) {
			auto& in = ws.insp[type];
			in.enabled = true;
			in.dx = dx;
			in.dy = dy;
		});
		_gs2Page->showInspResult(QStringLiteral("%1 point set from %2 (%3): offset %4, %5 mm along the trolley axes; the camera is %6 mm from that unit's centre. %1 is switched on.")
			.arg(what, s.units[best].name, gs2::refName(ref), signedMm(dx), signedMm(dy), mm(bestD)) + fid0Age(), false);
		ct::logger::info("[GuidedSetup2] %s point set: camera %.3f, %.3f = trolley frame %.3f, %.3f; from %s (%s) offset %+.3f, %+.3f",
			qPrintable(what), here.x, here.y, p.x, p.y, qPrintable(s.units[best].name), qPrintable(gs2::refName(ref)), dx, dy);
		AuditLog::instance().log(QStringLiteral("GS2_INSP_SET"), QStringLiteral("%1 %2, %3").arg(what, signedMm(dx), signedMm(dy)));
		return;
	}

	//── the rest moves the machine
	//what Find fid0 needs before it moves: B's Find and Locate
	auto fid0Ready = [&](QString& w) {
		if (!readyToRun(s, gs2::SecFid0, w)) return false;
		if (s.fid0.stepMm < 0.1 || s.fid0.stepMm > s.fid0.stopZ - s.fid0.startZ) {
			w = QStringLiteral("the max step must be between 0.1 mm and the Start-Stop range");
			return false;
		}
		return zsCheckTarget((int)Axis::X, s.fid0.xy.x, w) && zsCheckTarget((int)Axis::Y, s.fid0.xy.y, w)
			&& zsCheckTarget((int)Axis::Z, s.fid0.startZ, w) && zsCheckTarget((int)Axis::Z, s.fid0.stopZ, w);
	};
	QString why, title, note;
	QVector<gs2::RunPoint> visit;   //I's Move only: the points, gathered by the check before the machine moves
	gs2geo::Pt goTo, goStage;       //I's Go to: where the camera or laser looks, the stage for it, its Z, what it is
	double goZ = 0.0;
	bool goProfiler = false;
	QString goWhat;
	switch (action) {
	case P::TeachSnapEmpty:
		if (MachineController::instance().trolleyAttached()) {
			ct::logger::warn("[GuidedSetup2] Snap empty refused: the trolley guard reads a trolley docked");
			_gs2Page->showSafetyScore(QStringLiteral("The trolley guard says a trolley is docked. Undock it first: an empty image must show "
				"the bare table, or Safety check 1 would learn the trolley as \"empty\"."), false);
			return;
		}
		title = QStringLiteral("Snap empty");
		note = QStringLiteral("The machine goes to home (X 0, Y 0, Z 0) and snaps the empty table.");
		break;
	case P::TeachScore:
		if (s.safety1.emptyImages.isEmpty()) { _gs2Page->showSafetyScore(QStringLiteral("Snap at least one empty image first."), false); return; }
		title = QStringLiteral("Score now");
		note = QStringLiteral("The machine goes to home (X 0, Y 0, Z 0) and snaps.");
		break;
	case P::TeachXY:
		if (!s.circle[gs2::CircleFid0].dia.valid()) { _gs2Page->showFid0Result(QStringLiteral("Set the fid0 circle diameter in Camera and circle finder first."), false); return; }
		title = QStringLiteral("Teach fid0 XY");
		note = QStringLiteral("The camera centres on the fid0 circle.");
		break;
	case P::TeachJogXY:
		if (!s.fid0.xy.set) { _gs2Page->showFid0Result(QStringLiteral("Teach the search XY first."), false); return; }
		if (!s.fid0.startSet) { _gs2Page->showFid0Result(QStringLiteral("Set Start Z first: Jog to XY lifts Z to it before moving sideways."), false); return; }
		title = QStringLiteral("Moving");
		note = QStringLiteral("Z goes up to Start Z if it is lower, then the camera moves to the fid0 search XY.");
		break;
	case P::TeachFind: {
		QString w;
		if (!fid0Ready(w)) { _gs2Page->showFid0Result(QStringLiteral("Find cannot start: %1.").arg(w), false); return; }
		title = QStringLiteral("Find fid0");
		note = QStringLiteral("To the search XY at Start Z, Z sweep for the sharpest image (never past Stop Z), pattern, circle.");
		break;
	}
	case P::TeachZ3DFind: {
		QString w;
		const QPointF at = z3dReadPoint(s, _gs2Session.fid0X, _gs2Session.fid0Y, _laserConfig.offset);
		if (!gs2SessionFid0(s, dir, w) || !readyToRun(s, gs2::Sec3DZ, w) || !zsCheck3DRead(at.x(), at.y(), w)) {
			_gs2Page->showZ3DResult(QStringLiteral("Find cannot start: %1.").arg(w), false);
			return;
		}
		title = QStringLiteral("Find 3D Z");
		note = QStringLiteral("Z up to fid0's 2D Z if it is lower, the laser to the 3D spot, Z to the 2D Z and one profiler read - no Z search.");
		break;
	}
	case P::TeachCornerTL: case P::TeachCornerTR: case P::TeachCornerBR: case P::TeachCornerBL: {
		QString w;
		if (!gs2SessionFid0(s, dir, w)) { _gs2Page->showRectResult(QStringLiteral("Teach: %1.").arg(w), false); return; }
		if (!s.circle[gs2::CircleCorner].dia.valid()) { _gs2Page->showRectResult(QStringLiteral("Set the corner circle diameter in Camera and circle finder first."), false); return; }
		const auto z = MotionController::instance().get_position_mm(_motionID, 0, (int)Axis::Z);
		if (!z) { _gs2Page->showRectResult(QStringLiteral("Could not read the Z position - is motion connected?"), false); return; }
		//at the 2D Z or a little above, never below it: the teach moves sideways at this Z, and the 2D Z is as
		//low as the head is known to be clear of the trolley (+ Z is down)
		if (*z < _gs2Session.z2D - kTeachZTolMm || *z > _gs2Session.z2D + kTeachZBelowMm) {
			_gs2Page->showRectResult(QStringLiteral("The camera is at Z %1 but fid0's 2D Z is %2: jog Z to %2 (not lower) so the corner circle is in focus, then teach.")
				.arg(mm(*z), mm(_gs2Session.z2D)), false);
			return;
		}
		title = QStringLiteral("Teach %1").arg(QLatin1String(kCornerName[action - P::TeachCornerTL]));
		note = QStringLiteral("The camera centres on the corner circle, then goes to fid0 at the same Z and locates it again.");
		break;
	}
	case P::TeachJogCornerTL: case P::TeachJogCornerTR: case P::TeachJogCornerBR: case P::TeachJogCornerBL: {
		const int i = action - P::TeachJogCornerTL;
		QString w;
		if (!s.fidRect.corner[i].set) { _gs2Page->showRectResult(QStringLiteral("Teach %1 first.").arg(QLatin1String(kCornerName[i])), false); return; }
		if (!gs2SessionFid0(s, dir, w)) { _gs2Page->showRectResult(QStringLiteral("Jog: %1.").arg(w), false); return; }
		if (!zsCheckTarget((int)Axis::X, _gs2Session.fid0X + s.fidRect.corner[i].x, w) || !zsCheckTarget((int)Axis::Y, _gs2Session.fid0Y + s.fidRect.corner[i].y, w)) {
			_gs2Page->showRectResult(QStringLiteral("Jog cannot start: %1.").arg(w), false);
			return;
		}
		title = QStringLiteral("Jog to %1").arg(QLatin1String(kCornerName[i]));
		note = QStringLiteral("Z up to fid0's 2D Z if it is lower, the camera to fid0 (as found now) + the taught %1 offset, Z to the 2D Z.")
			.arg(QLatin1String(kCornerName[i]));
		break;
	}
	//E's Set TL / BR and F's Add on a TL / BR circle (Add from TR / BL never gets here)
	case P::TeachSlotTL: case P::TeachSlotBR: case P::TeachSlotAdd: {
		const bool add = action == P::TeachSlotAdd;
		const QString which = QLatin1String(add ? kSlotCorner[addCorner] : action == P::TeachSlotTL ? "TL" : "BR");
		auto say = [&](const QString& t) { if (add) _gs2Page->showSlotsResult(t, false); else _gs2Page->showSlotSizeResult(t, false); };
		const QString verb = add ? QStringLiteral("Add") : QStringLiteral("Set %1").arg(which);
		QString w;
		if (!gs2SessionRect(s, dir, w)) { say(QStringLiteral("%1: %2.").arg(verb, w)); return; }
		if (!s.circle[gs2::CircleSlot].dia.valid()) { say(QStringLiteral("Set the slot circle diameter in Camera and circle finder first.")); return; }
		if (!atTwoDZ(w)) { say(QStringLiteral("%1: %2.").arg(verb, w)); return; }
		title = add ? QStringLiteral("Add slot from its %1 circle").arg(which) : QStringLiteral("Set slot %1").arg(which);
		note = QStringLiteral("The camera centres on the slot circle nearest the image centre (the slot's %1 circle), "
			"goes to fid0 at the same Z to locate it again, and comes back over the circle.").arg(which);
		break;
	}
	//I's Move only: the points of the selected slots' units (all units when none is selected) in run order -
	//the 2D pass, then the 3D pass, as production will run them
	//...and Snap and scan: the same points, taken with production's own code (build step 5)
	case P::TeachMoveOnly: case P::TeachSnapScan: {
		static const char* const kWhatName[3] = { "2D", "3D", "2D + 3D" };
		const bool capture = action == P::TeachSnapScan;
		const int what = capture ? _gs2Page->snapScanWhat() : _gs2Page->moveOnlyWhat();
		const QString name = QStringLiteral("%1 (%2)").arg(capture ? QStringLiteral("Snap and scan") : QStringLiteral("Move only"), QLatin1String(kWhatName[what]));
		auto say = [&](const QString& t) { _gs2Page->showLocateResult(QStringLiteral("%1: %2.").arg(name, t), false); };
		QString w;
		if (!gs2SessionRect(s, dir, w)) { say(w); return; }
		const QStringList only = _gs2Page->selectedSlotIds();
		const QVector<gs2::RunPoint> all = gs2::runPoints(s, framePose(), only);
		if (what != P::Move3D) for (const auto& p : all) if (p.type != gs2::Insp3D) visit << p;
		if (what != P::Move2D) for (const auto& p : all) if (p.type == gs2::Insp3D) visit << p;
		if (visit.isEmpty()) {
			say(QStringLiteral("no such point is switched on in H%1").arg(only.isEmpty() ? QString() : QStringLiteral(" for the units of the selected slots")));
			return;
		}
		const bool any3D = visit.last().type == gs2::Insp3D;
		if (any3D && !_gs2Session.has3D) { say(QStringLiteral("run Find in section C (or Locate) first - the 3D scans run at the 3D Centre Z")); return; }
		if (any3D && s.insp[gs2::Insp3D].scanLengthMm <= 0.0) { say(QStringLiteral("type the 3D scan length in H first")); return; }
		if (!gs2CheckReach(s, visit, _gs2Session.z2D, _gs2Session.z3D, w)) { say(w); return; }
		if (capture && any3D) {
			//production's scan move travels X only (JobThread::jog sends no Y for a 3D move)
			if (s.scanAxis == QLatin1String("Y")) { say(QStringLiteral("production's 3D scans run along X only - set H's 3D scan direction to +X or -X")); return; }
			const gs2geo::Pt read = gs2::scanReadPoint(s, visit.last().camera, _laserConfig.offset.wx, _laserConfig.offset.wy);
			if (!zsCheck3DRead(read.x, read.y, w)) { say(w); return; }
		}
		QSet<int> units;
		for (const auto& p : visit) units.insert(p.unit);
		title = name;
		const QString scope = QStringLiteral("%1 points of %2 units%3 in run order").arg(visit.size()).arg(units.size())
			.arg(only.isEmpty() ? QString() : QStringLiteral(" (the selected slots)"));
		note = capture
			? QStringLiteral("%1, with production's own code: a 2D snap saved as <unit>_2D, the barcode readers + OCR, the 3D scans - into a new "
				"production folder, inspected as in production.").arg(scope)
			: QStringLiteral("%1, no snap and no scan: the camera to each 2D and Barcode + OCR point, the laser along each 3D scan at the "
				"3D Centre Z. Sideways moves are never below the 2D Z.").arg(scope);
		break;
	}
	//I's Go to: a position of the unit selected in the layout (else the one selected slot), with the camera or the profiler
	case P::TeachGoTo: {
		static const char* const kPosName[] = { "centre", "top left", "top", "top right", "left", "right", "bottom left", "bottom", "bottom right",
			"2D point", "Barcode + OCR point", "3D scan start" };
		const int pos = _gs2Page->goToPosition();
		goProfiler = _gs2Page->goToWithProfiler();
		auto say = [&](const QString& t) { _gs2Page->showLocateResult(QStringLiteral("Go to: %1.").arg(t), false); };
		QString w;
		if (!gs2SessionRect(s, dir, w)) { say(w); return; }
		if (goProfiler && !_gs2Session.has3D) { say(QStringLiteral("run Find in section C (or Locate) first - the profiler goes to the 3D Centre Z")); return; }
		const gs2geo::Pose pose = framePose();
		const QString unitId = _gs2View ? _gs2View->selectedUnit() : QString();
		const QStringList picked = _gs2Page->selectedSlotIds();
		const gs2::Unit* unit = nullptr;
		for (const auto& u : s.units) if (u.id == unitId) unit = &u;
		const gs2::Slot* slot = unit ? gs2::findSlot(s, unit->slotId) : picked.size() == 1 ? gs2::findSlot(s, picked.first()) : nullptr;
		if (!slot) { say(QStringLiteral("select one unit (click it in the layout) or one slot")); return; }
		double zOff = 0.0;
		if (pos >= P::Go2DPoint) {
			if (!unit) { say(QStringLiteral("the inspection points are a unit's - click a unit in the layout")); return; }
			const gs2::InspType t = pos == P::Go2DPoint ? gs2::Insp2D : pos == P::GoBarcodePoint ? gs2::InspBarcodeOcr : gs2::Insp3D;
			if (!s.insp[t].enabled) { say(QStringLiteral("the %1 point is switched off in H").arg(gs2::inspTypeName(t))); return; }
			goTo = gs2::inspectionWorld(s, pose, *slot, unit->row, unit->col, t);
			if (t == gs2::Insp3D) goTo = gs2::scanReadPoint(s, goTo, _laserConfig.offset.wx, _laserConfig.offset.wy);
			if ((t == gs2::Insp3D) == goProfiler) zOff = s.insp[t].zOffsetMm;   //its own Z offset when its own tool goes there
		}
		else {
			//on the unit's or the slot's rectangle; top = far side, as in the layout
			static const double fx[9] = { 0.0, -0.5, 0.0, 0.5, -0.5, 0.5, -0.5, 0.0, 0.5 }, fy[9] = { 0.0, -0.5, -0.5, -0.5, 0.0, 0.0, 0.5, 0.5, 0.5 };
			const double rw = unit ? s.unitLayout.widthMm : s.slotSize.widthMm, rl = unit ? s.unitLayout.lengthMm : s.slotSize.lengthMm;
			if (pos != P::GoCentre && (rw <= 0.0 || rl <= 0.0)) { say(unit ? QStringLiteral("type the unit size in G first") : QStringLiteral("measure the slot size in E first")); return; }
			const gs2geo::Pt c = unit ? gs2::unitCentreLocal(s, *slot, unit->row, unit->col) : gs2geo::Pt{ slot->cx, slot->cy };
			goTo = gs2geo::toWorld(pose, { c.x + fx[pos] * rw, c.y + fy[pos] * rl });
		}
		goWhat = QStringLiteral("the %1 of %2").arg(QLatin1String(kPosName[pos]), unit ? unit->name : slot->name);
		goZ = (goProfiler ? _gs2Session.z3D : _gs2Session.z2D) + zOff;
		goStage = goProfiler ? gs2geo::Pt{ goTo.x + _laserConfig.offset.wx, goTo.y + _laserConfig.offset.wy } : goTo;
		if (!zsCheckTarget((int)Axis::X, goStage.x, w) || !zsCheckTarget((int)Axis::Y, goStage.y, w) || !zsCheckTarget((int)Axis::Z, goZ, w)) { say(w); return; }
		title = QStringLiteral("Go to %1").arg(goWhat);
		note = goProfiler
			? QStringLiteral("Sideways at the 2D Z or higher, the laser over %1 where the trolley is now (the stage moves by the camera-to-laser XY), Z to the 3D Centre Z.").arg(goWhat)
			: QStringLiteral("Sideways at the 2D Z or higher, the camera over %1 where the trolley is now, Z to the 2D Z.").arg(goWhat);
		break;
	}
	case P::TeachRectCheck: {
		QString w;
		if (!gs2SessionFid0(s, dir, w) || !readyToRun(s, gs2::SecFidRect, w)) { _gs2Page->showRectResult(QStringLiteral("Check cannot start: %1.").arg(w), false); return; }
		for (int i = 0; i < 4 && w.isEmpty(); i++)
			if (!zsCheckTarget((int)Axis::X, _gs2Session.fid0X + s.fidRect.corner[i].x, w) || !zsCheckTarget((int)Axis::Y, _gs2Session.fid0Y + s.fidRect.corner[i].y, w))
				w = QStringLiteral("%1: %2").arg(QLatin1String(kCornerName[i]), w);
		if (!w.isEmpty()) { _gs2Page->showRectResult(QStringLiteral("Check cannot start: %1.").arg(w), false); return; }
		title = QStringLiteral("Check fidRect");
		note = QStringLiteral("Z up to Start Z if it is lower, to fid0 and down to its 2D Z, fid0 located again, then each corner at that Z.");
		break;
	}
	case P::TeachLocate: {
		QString w;
		const QPointF at = z3dReadPoint(s, s.fid0.xy.x, s.fid0.xy.y, _laserConfig.offset);
		if (!gs2::sectionDone(s, gs2::SecSafety1, &w)) w = QStringLiteral("A: %1").arg(w);
		else if (!fid0Ready(w)) w = QStringLiteral("B: %1").arg(w);
		else if (!readyToRun(s, gs2::Sec3DZ, w)) w = QStringLiteral("C: %1").arg(w);
		else if (!readyToRun(s, gs2::SecFidRect, w)) w = QStringLiteral("D: %1").arg(w);
		else if (!zsCheck3DRead(at.x(), at.y(), w)) w = QStringLiteral("C: %1").arg(w);
		else w.clear();
		if (!w.isEmpty()) { _gs2Page->showLocateResult(QStringLiteral("Locate cannot start: %1.").arg(w), false); return; }
		title = QStringLiteral("Locate");
		note = QStringLiteral("START steps 1-5 as production will run them: home and Safety check 1, Find fid0 (Z sweep, pattern, circle), 3D Z (one read at the 2D Z), the four corners.");
		break;
	}
	default:
		return;
	}
	if (!ui.graphicsViewFOV->isVisible()) toggleFOVView();
	if (!gs2MachineBusy(title, note + QStringLiteral("\nEverything else is locked until it finishes."))) return;

	const int trolleyEpoch = _zsTrolleyEpoch;
	bool ok = false;
	QString result;
	//a result belongs to the trolley and the recipe the action started with
	auto stillHere = [&](QString& w) {
		if (trolleyEpoch != _zsTrolleyEpoch) { w = QStringLiteral("the trolley was removed or attached meanwhile - the result was discarded"); return false; }
		if (_gs2Page->recipeDir() != dir) { w = QStringLiteral("the recipe changed meanwhile - the result was discarded"); return false; }
		return true;
	};
	//passed START steps: into the page (and the file, see recordResult) and into this session
	auto keepFid0 = [&](const gs2::Fid0Found& found) {
		_gs2Page->recordFid0Found(found);
		//a 3D Find or a trolley frame from another trolley or recipe is not this one's
		if (_gs2Session.trolleyEpoch != _zsTrolleyEpoch || _gs2Session.recipeDir != dir) _gs2Session.has3D = _gs2Session.rect = false;
		_gs2Session.fid0 = true;
		_gs2Session.fid0X = found.x;
		_gs2Session.fid0Y = found.y;
		_gs2Session.fid0AtMs = QDateTime::currentMSecsSinceEpoch();
		_gs2Session.z2D = found.z;
		_gs2Session.trolleyEpoch = _zsTrolleyEpoch;
		_gs2Session.recipeDir = dir;
		AuditLog::instance().log(QStringLiteral("GS2_FID0_FIND"), QStringLiteral("X %1, Y %2, Z %3, score %4")
			.arg(mm(found.x), mm(found.y), mm(found.z)).arg(found.score, 0, 'f', 2));
	};
	auto keep3D = [&](const gs2::Z3DFound& found) {
		_gs2Page->recordZ3DFound(found);
		_gs2Session.has3D = true;
		_gs2Session.z3D = found.z;
		_gs2Session.z3DFrom2D = found.z2D;
		_gs2Session.z3DAddLaser = s.z3d.addLaserOffset;
		AuditLog::instance().log(QStringLiteral("GS2_Z3D_FIND"), QStringLiteral("Z %1 from 2D Z %2").arg(mm(found.z), mm(found.z2D)));
	};
	auto keepRect = [&](const Gs2RectCheck& r) {
		_gs2Page->recordFidRectFound(rectFound(r.fit));
		_gs2Session.rect = true;          //sections E, F and H teach in this frame
		_gs2Session.pose = r.fit.pose;
		_gs2Session.rectBy = s.fidRect;
		_gs2Session.rectFid0 = r.fid0;
		AuditLog::instance().log(QStringLiteral("GS2_FIDRECT_CHECK"), QStringLiteral("centre %1, %2, angle %3, worst residual %4")
			.arg(mm(r.fit.pose.cx), mm(r.fit.pose.cy), QString::number(r.fit.pose.angle * 180.0 / kPi, 'f', 3), mm(r.fit.maxResidual())));
	};
	//the layout drawn where the corners were found, passed or not: a failed corner shows where it is
	auto showFound = [&](const Gs2RectCheck& r) {
		if (_gs2View) _gs2View->setLocated(r.fit, r.fid0, r.corner, QStringLiteral("%1 %2%3").arg(title, QTime::currentTime().toString(QStringLiteral("hh:mm")),
			r.pass ? QString() : QStringLiteral(" (FAILED)")));
	};
	switch (action) {
	case P::TeachSnapEmpty: {
		QImage small;
		ok = gs2SnapHome(small, why);
		if (ok && (MachineController::instance().trolleyAttached() || trolleyEpoch != _zsTrolleyEpoch)) { ok = false; why = QStringLiteral("a trolley was attached meanwhile - nothing kept"); }
		if (ok && _gs2Page->recipeDir() != dir) { ok = false; why = QStringLiteral("the recipe changed meanwhile - nothing kept"); }
		QVector<double> others;
		if (ok) ok = gs2SafetyScores(small, s.safety1.emptyImages, others, why);
		const QString name = QStringLiteral("gs2_empty_%1.png").arg(stamp);
		if (ok && !small.save(dir + name)) { ok = false; why = QStringLiteral("cannot write %1").arg(dir + name); }
		if (ok) {
			_gs2Page->modify([&](gs2::Setup& w) { w.safety1.emptyImages << name; });
			QStringList d;
			for (double v : others) d << QString::number(v, 'f', 1);
			ct::logger::info("[GuidedSetup2] Empty image %s saved; against the other empty images: %s", qPrintable(name), qPrintable(d.join(QStringLiteral(", "))));
			result = QStringLiteral("Saved %1.%2").arg(name, d.isEmpty() ? QString()
				: QStringLiteral(" It differs from the other empty images by %1 - the threshold must be well above that.").arg(d.join(QStringLiteral(", "))));
			AuditLog::instance().log(QStringLiteral("GS2_SAFETY_EMPTY"), name);
		}
		break;
	}
	case P::TeachScore: {
		QImage small;
		QVector<double> scores;
		ok = gs2SnapHome(small, why) && gs2SafetyScores(small, s.safety1.emptyImages, scores, why);
		if (ok) {
			double lowest = scores[0];
			QStringList d;
			for (int i = 0; i < scores.size(); i++) { lowest = std::min(lowest, scores[i]); d << QString::number(scores[i], 'f', 1); }
			const double t = s.safety1.threshold;
			result = QStringLiteral("Score now: %1 (one per empty image; lowest %2). %3").arg(d.join(QStringLiteral(", "))).arg(lowest, 0, 'f', 1)
				.arg(t > 0.0 ? (lowest > t ? QStringLiteral("Above the threshold %1: the table is NOT empty - START would go on.").arg(t, 0, 'f', 1)
					: QStringLiteral("Not above the threshold %1: the table looks EMPTY - START would stop.").arg(t, 0, 'f', 1))
					: QStringLiteral("Set the threshold between the empty and loaded scores."));
			ct::logger::info("[GuidedSetup2] Safety check 1 score: %s (threshold %.1f)", qPrintable(d.join(QStringLiteral(", "))), t);
		}
		break;
	}
	case P::TeachXY: {
		Gs2Circle c;
		int moves = 0;
		ok = gs2LocateCircle(s.circle[gs2::CircleFid0], true, c, moves, why);
		if (ok && trolleyEpoch != _zsTrolleyEpoch) { ok = false; why = QStringLiteral("the trolley was removed or attached meanwhile - nothing kept"); }
		if (ok && _gs2Page->recipeDir() != dir) { ok = false; why = QStringLiteral("the recipe changed meanwhile - nothing kept"); }
		if (ok) {
			_gs2Page->modify([&](gs2::Setup& w) { w.fid0.xy = { true, c.wx, c.wy }; });
			result = QStringLiteral("Search XY taught: the fid0 circle at X %1, Y %2 (%3 mm across). Run Find to prove the settings.")
				.arg(mm(c.wx), mm(c.wy), mm(c.diaMm));
			AuditLog::instance().log(QStringLiteral("GS2_FID0_XY"), QStringLiteral("%1, %2").arg(mm(c.wx), mm(c.wy)));
		}
		break;
	}
	case P::TeachJogXY:
		ok = zsMoveToXY(s.fid0.xy.x, s.fid0.xy.y, s.fid0.startZ, why);
		if (ok) result = QStringLiteral("At the fid0 search XY.");
		break;
	//sideways at the 2D Z (or higher), as Check moves between corners: the corner arrives in focus
	case P::TeachJogCornerTL: case P::TeachJogCornerTR: case P::TeachJogCornerBR: case P::TeachJogCornerBL: {
		const int i = action - P::TeachJogCornerTL;
		const double x = _gs2Session.fid0X + s.fidRect.corner[i].x, y = _gs2Session.fid0Y + s.fidRect.corner[i].y;
		ok = zsMoveToXY(x, y, _gs2Session.z2D, why) && zsMove({ { (int)Axis::Z, _gs2Session.z2D } }, why);
		if (ok) result = QStringLiteral("At %1: X %2, Y %3, Z %4 (fid0 as found now + its taught offset, at the 2D Z). Teach %5 here to teach it again.")
			.arg(QLatin1String(kCornerName[i]), mm(x), mm(y), mm(_gs2Session.z2D), QLatin1String(kCornerName[i] + 3));
		break;
	}
	case P::TeachSlotTL: case P::TeachSlotBR: case P::TeachSlotAdd: {
		Gs2Circle c, f;
		int moves = 0;
		const double wasX = _gs2Session.fid0X, wasY = _gs2Session.fid0Y;
		ok = gs2LocateCircle(s.circle[gs2::CircleSlot], true, c, moves, why);
		if (!ok) { why = QStringLiteral("the slot circle: %1").arg(why); break; }
		ct::logger::info("[GuidedSetup2] slot circle at %.3f, %.3f (%.3f mm across, %d move(s))", c.wx, c.wy, c.diaMm, moves);
		//fid0 again, as the corner teach does: what crept since the Check moves the frame, not the slot
		ok = gs2RelocateFid0(s, f, why) && stillHere(why);
		if (!ok) break;
		const QString fid0Note = QStringLiteral(" fid0 was located again %1, %2 mm from before.").arg(signedMm(f.wx - wasX), signedMm(f.wy - wasY));
		if (action != P::TeachSlotAdd) result = keepSlotCircle(action == P::TeachSlotBR, c.wx, c.wy) + fid0Note;
		else if (!(ok = addSlot(addCorner, c.wx, c.wy, why))) why = QStringLiteral("not added - %1").arg(why);
		else result = why + fid0Note;
		//back over the circle at the same Z, so the next one is taught from near here
		QString back;
		if (ok && trolleyEpoch == _zsTrolleyEpoch && !zsMove({ { (int)Axis::X, c.wx }, { (int)Axis::Y, c.wy } }, back))
			result += QStringLiteral(" The camera did not go back over the circle: %1.").arg(back);
		break;
	}
	/*
	* Move only: the points one after another, as production's 2D pass and then its 3D pass will reach them,
	* without snapping or scanning. Each move goes sideways at the higher of the 2D Z and the point's own Z
	* (+ Z is down), then down to the point's Z; a 3D point then runs along its scan. Z is the 2D Z or the
	* 3D Centre Z plus the point's Z offset - never the camera-to-laser Z.
	*/
	case P::TeachMoveOnly: {
		QElapsedTimer timer;
		timer.start();
		const bool alongY = s.scanAxis == QLatin1String("Y");
		const double along = s.scanNegative ? -1.0 : 1.0;
		int done = 0;
		ok = true;
		for (int k = 0; k < visit.size() && ok; k++) {
			const auto& p = visit[k];
			const auto& in = s.insp[p.type];
			const QString name = s.units[p.unit].name, type = gs2::inspTypeName(p.type);
			if (trolleyEpoch != _zsTrolleyEpoch) { ok = false; why = QStringLiteral("the trolley was removed or attached - nothing else was moved"); break; }
			setProgress(_zsBusyWindow, QStringLiteral("%1 of %2: %3 %4").arg(k + 1).arg(visit.size()).arg(name, type));
			const bool threeD = p.type == gs2::Insp3D;
			const double z = (threeD ? _gs2Session.z3D : _gs2Session.z2D) + in.zOffsetMm;
			const double clearZ = std::min(_gs2Session.z2D, z);
			if (!threeD) {
				ok = zsMoveToXY(p.camera.x, p.camera.y, clearZ, why) && zsMove({ { (int)Axis::Z, z } }, why);
				if (ok) ct::logger::info("[GuidedSetup2] Move only %d/%d: %s %s, camera at %.3f, %.3f, Z %.3f", k + 1, visit.size(), qPrintable(name),
					qPrintable(type), p.camera.x, p.camera.y, z);
			}
			else {
				//the stage = where the laser measures (H's 3D laser XY) + the camera-to-laser XY
				const gs2geo::Pt read = gs2::scanReadPoint(s, p.camera, _laserConfig.offset.wx, _laserConfig.offset.wy);
				const double lx = read.x + _laserConfig.offset.wx, ly = read.y + _laserConfig.offset.wy;
				const double end = (alongY ? ly : lx) + in.scanLengthMm * along;
				ok = zsMoveToXY(lx, ly, clearZ, why) && zsMove({ { (int)Axis::Z, z } }, why)
					&& zsMove({ { alongY ? (int)Axis::Y : (int)Axis::X, end } }, why);
				if (ok) ct::logger::info("[GuidedSetup2] Move only %d/%d: %s 3D, laser on %.3f, %.3f (stage %.3f, %.3f) to stage %s %.3f, Z %.3f",
					k + 1, visit.size(), qPrintable(name), read.x, read.y, lx, ly, alongY ? "Y" : "X", end, z);
			}
			if (!ok) { why = QStringLiteral("%1 %2: %3").arg(name, type, why); break; }
			zsPause(kVisitDwellMs);
			done++;
		}
		if (ok) {
			result = QStringLiteral("%1 went through %2 points in %3 s.").arg(title).arg(done).arg(timer.elapsed() / 1000.0, 0, 'f', 1);
			ct::logger::info("[GuidedSetup2] %s", qPrintable(result));
		}
		else if (!_stopRun) why = QStringLiteral("point %1 of %2, %3. %4").arg(done + 1).arg(visit.size()).arg(why, zsLiftAfterFailure(s.fid0.startZ, QStringLiteral("Start Z")));
		break;
	}
	/*
	* Snap and scan: production's own acquisition (JobThread::runGs2, on the job thread) for the points
	* gathered above, into a new production folder as a START makes one. The busy window stays up until
	* the run is over, and its Stop stops the run the way production's Stop does. The recipe's line scan
	* is set to H's 3D scan direction first: scan() and the height-map orientation follow it.
	*/
	case P::TeachSnapScan: {
		QElapsedTimer timer;
		timer.start();
		const double along = s.scanNegative ? -1.0 : 1.0;
		QVector<JobThread::Gs2Shot> shots;
		for (const auto& p : visit) {
			const auto& in = s.insp[p.type];
			JobThread::Gs2Shot shot;
			shot.unitID = s.units[p.unit].name;
			if (p.type == gs2::Insp3D) {
				//where the laser measures (H's 3D laser XY); the job thread adds the camera-to-laser XY and takes its Z back out
				const gs2geo::Pt read = gs2::scanReadPoint(s, p.camera, _laserConfig.offset.wx, _laserConfig.offset.wy);
				shot.kind = 2;
				shot.x = read.x;
				shot.y = read.y;
				shot.z = _gs2Session.z3D + in.zOffsetMm;
				shot.scanEndX = read.x + in.scanLengthMm * along;
			}
			else {
				shot.kind = p.type == gs2::Insp2D ? 0 : 1;
				shot.x = p.camera.x;
				shot.y = p.camera.y;
				shot.z = _gs2Session.z2D + in.zOffsetMm;
			}
			shot.clearZ = std::min(_gs2Session.z2D, shot.z);   //sideways never below the 2D Z, as Move only
			shots << shot;
		}
		bool any2DSnap = false, anyBarcode = false, anyScan = false;
		for (const auto& sh : shots) { any2DSnap = any2DSnap || sh.kind == 0; anyBarcode = anyBarcode || sh.kind == 1; anyScan = anyScan || sh.kind == 2; }
		//before anything is changed or made: the optics production's code will ask for
		if ((any2DSnap && _recipeOptics.isEmpty()) || (anyScan && _recipeOptics3D.isEmpty())) {
			ok = false;
			why = QStringLiteral("the recipe has no %1 optics").arg(any2DSnap && _recipeOptics.isEmpty() ? QStringLiteral("2D") : QStringLiteral("3D"));
			break;
		}
		auto& sd = SystemData::instance();
		const int dirIndex = s.scanNegative ? 1 : 0;
		//only a run that scans changes the recipe's line scan
		if (anyScan && (sd._lineScanAxis != 0 || sd._lineScanDirection != dirIndex)) {
			ct::logger::info("[GuidedSetup2] recipe line scan set to H's 3D scan direction: along X, %s (was along %s, %s)",
				dirIndex ? "negative" : "positive", sd._lineScanAxis == 1 ? "Y" : "X", sd._lineScanDirection == 1 ? "negative" : "positive");
			sd._lineScanAxis = 0;
			sd._lineScanDirection = dirIndex;
			{
				QSignalBlocker b1(ui.comboBox_lineScanAxis), b2(ui.comboBox_lineScanDirection);   //their handlers would ask questions mid-run
				ui.comboBox_lineScanAxis->setCurrentIndex(0);
				ui.comboBox_lineScanDirection->setCurrentIndex(dirIndex);
			}
			saveRecipeConfig();
			AuditLog::instance().log(QStringLiteral("LINESCAN_DIRECTION_CHANGED"), QStringLiteral("Guided Setup (new): along X, %1").arg(dirIndex ? QStringLiteral("-") : QStringLiteral("+")));
		}
		gs2PrepareProductionRun();
		_gs2Run.active = true;
		_gs2Run.barcode = anyBarcode;
		_gs2Run.height = anyScan;
		_gs2Run.capturing = _gs2Run.holdPrompts = true;
		_gs2Run.heldPrompts.clear();
		const QString folder = Common::Directory::getProductionImageSetPath();
		bool finished = false, completed = false;
		int taken = 0, missing2D = 0;
		const auto c1 = connect(&_jobThread, &JobThread::gs2RunFinished, this, [&](bool all, int n, int missing) { finished = true; completed = all; taken = n; missing2D = missing; });
		const auto c2 = connect(&_jobThread, &JobThread::unitStarted, this, [&](QString unit) {
			setProgress(_zsBusyWindow, QStringLiteral("At %1 (%2 shots in all)...").arg(unit).arg(shots.size()));
		});
		QTimer::singleShot(0, &_jobThread, [this, shots]() { _jobThread.runGs2(shots); });
		bool stopSent = false, trolleyChanged = false;
		while (!finished) {
			zsPause(50);
			//a trolley taken out or put in stops it too - in debug mode the guard does not
			if (trolleyEpoch != _zsTrolleyEpoch) trolleyChanged = true;
			//the busy window's Stop ends the run as production's Stop does; the job thread's flag is set again on
			//every pass, as runGs2 clears it when it starts
			if (_stopRun || trolleyChanged) {
				if (!stopSent) { stopSent = true; stopRun(); }
				_jobThread.stopRun();
			}
		}
		disconnect(c1);
		disconnect(c2);
		if (trolleyChanged) _stopRun = false;   //stopRun() set it: the result says why instead of a plain "stopped"
		_gs2Run.capturing = _gs2Run.holdPrompts = false;
		ok = completed && !trolleyChanged;
		QString tail;
		if (missing2D) tail += QStringLiteral(" %1 2D snap(s) brought no image - see the log.").arg(missing2D);
		if (!SystemData::instance()._saveInspImages && (anyBarcode || anyScan))
			tail += QStringLiteral(" Save Inspection Images is off, so reader images and height maps were not saved.");
		if (!_gs2Run.heldPrompts.isEmpty()) tail += QStringLiteral(" During the run: %1").arg(_gs2Run.heldPrompts.join(QStringLiteral(" | ")));
		if (ok) {
			result = QStringLiteral("%1 took %2 shots in %3 s, into %4. OCR and 3D results come into the production table as their inspection finishes.%5")
				.arg(title).arg(taken).arg(timer.elapsed() / 1000.0, 0, 'f', 1).arg(folder, tail);
			ct::logger::info("[GuidedSetup2] %s", qPrintable(result));
		}
		else if (trolleyChanged)
			why = QStringLiteral("the trolley was removed or attached - the run was stopped after %1 of %2 shots.%3").arg(taken).arg(shots.size()).arg(tail);
		else if (!_stopRun)
			why = QStringLiteral("the run ended after %1 of %2 shots - see the log.%3 %4").arg(taken).arg(shots.size())
				.arg(tail, zsLiftAfterFailure(s.fid0.startZ, QStringLiteral("Start Z")));
		else if (!tail.isEmpty()) ct::logger::info("[GuidedSetup2] Snap and scan stopped.%s", qPrintable(tail));
		break;
	}
	case P::TeachGoTo:
		ok = zsMoveToXY(goStage.x, goStage.y, std::min(_gs2Session.z2D, goZ), why) && zsMove({ { (int)Axis::Z, goZ } }, why);
		if (ok) result = goProfiler
			? QStringLiteral("The 3D profiler is over %1: the laser on X %2, Y %3 (stage X %4, Y %5), Z %6.").arg(goWhat, mm(goTo.x), mm(goTo.y), mm(goStage.x), mm(goStage.y), mm(goZ))
			: QStringLiteral("The 2D camera is over %1: X %2, Y %3, Z %4.").arg(goWhat, mm(goTo.x), mm(goTo.y), mm(goZ));
		if (ok) ct::logger::info("[GuidedSetup2] %s", qPrintable(result));
		break;
	case P::TeachFind: {
		gs2::Fid0Found found;
		ok = gs2FindFid0(s, dir, found, why) && stillHere(why);
		if (ok) keepFid0(found);
		//after a failed Find Z goes back up to Start Z, as the Z search does - up only, never down to it
		else if (!_stopRun) why += QStringLiteral(". ") + zsLiftAfterFailure(s.fid0.startZ, QStringLiteral("Start Z"));
		break;
	}
	case P::TeachZ3DFind: {
		gs2::Z3DFound found;
		ok = gs2Find3DZ(s, found, why) && stillHere(why);
		if (ok) keep3D(found);
		else if (!_stopRun) why += QStringLiteral(". ") + zsLiftAfterFailure(s.fid0.startZ, QStringLiteral("Start Z"));
		break;
	}
	case P::TeachCornerTL: case P::TeachCornerTR: case P::TeachCornerBR: case P::TeachCornerBL: {
		const int i = action - P::TeachCornerTL;
		const double wasX = _gs2Session.fid0X, wasY = _gs2Session.fid0Y;
		Gs2Circle corner, f;
		int moves = 0;
		ok = gs2LocateCircle(s.circle[gs2::CircleCorner], true, corner, moves, why);
		if (!ok) why = QStringLiteral("the corner circle: %1").arg(why);
		else ok = gs2RelocateFid0(s, f, why) && stillHere(why);
		if (ok) {
			const double dx = corner.wx - f.wx, dy = corner.wy - f.wy;
			_gs2Page->modify([&](gs2::Setup& ws) { ws.fidRect.corner[i] = { true, dx, dy }; });
			result = QStringLiteral("%1 taught: the circle at X %2, Y %3 (%4 mm across) = fid0 %5, %6 mm. fid0 was located again at X %7, Y %8, %9 mm from before. %10")
				.arg(QLatin1String(kCornerName[i]), mm(corner.wx), mm(corner.wy), mm(corner.diaMm), signedMm(dx), signedMm(dy), mm(f.wx), mm(f.wy),
					mm(std::hypot(f.wx - wasX, f.wy - wasY)))
				.arg(QStringLiteral("Teach the other corners, then Check."));
			ct::logger::info("[GuidedSetup2] %s taught: circle %.3f, %.3f (%.3f mm across, %d move(s)), fid0 %.3f, %.3f -> offset %+.3f, %+.3f",
				kCornerName[i], corner.wx, corner.wy, corner.diaMm, moves, f.wx, f.wy, dx, dy);
			AuditLog::instance().log(QStringLiteral("GS2_CORNER"), QStringLiteral("%1 fid0 %2, %3").arg(QLatin1String(kCornerName[i]), signedMm(dx), signedMm(dy)));
		}
		break;
	}
	case P::TeachRectCheck: {
		Gs2RectCheck r;
		_gs2Session.rect = false; //only a Check that passes gives the frame E, F and H teach in
		ok = gs2CheckRect(s, r, why) && stillHere(why);
		if (ok) {
			showFound(r);
			if (r.pass) keepRect(r);
			else { ok = false; why = r.report; }
		}
		if (!ok && !_stopRun) why += QStringLiteral(". ") + zsLiftAfterFailure(s.fid0.startZ, QStringLiteral("Start Z"));
		break;
	}
	case P::TeachLocate: {
		QElapsedTimer timer;
		timer.start();
		QStringList steps;
		_gs2Session.rect = false;
		//1: Safety check 1 at home
		QImage small;
		QVector<double> scores;
		ok = gs2SnapHome(small, why) && gs2SafetyScores(small, s.safety1.emptyImages, scores, why);
		if (ok) {
			const double lowest = *std::min_element(scores.begin(), scores.end());
			ct::logger::info("[GuidedSetup2] Locate: Safety check 1 lowest score %.1f, threshold %.1f", lowest, s.safety1.threshold);
			if (lowest <= s.safety1.threshold) {
				ok = false;
				why = QStringLiteral("the table looks empty (score %1, threshold %2) - load a trolley").arg(lowest, 0, 'f', 1).arg(s.safety1.threshold, 0, 'f', 1);
			}
			else steps << QStringLiteral("Safety check 1 score %1 (threshold %2)").arg(lowest, 0, 'f', 1).arg(s.safety1.threshold, 0, 'f', 1);
		}
		if (!ok) why = QStringLiteral("Safety check 1: %1").arg(why);
		//2: fid0
		gs2::Fid0Found found;
		if (ok) ok = gs2FindFid0(s, dir, found, why) && stillHere(why);
		if (ok) {
			keepFid0(found);
			steps << QStringLiteral("fid0 at X %1, Y %2, 2D Z %3 (pattern %4)").arg(mm(found.x), mm(found.y), mm(found.z)).arg(found.score, 0, 'f', 2);
		}
		//3: 3D Z
		gs2::Z3DFound z3;
		if (ok) ok = gs2Find3DZ(s, z3, why) && stillHere(why);
		if (ok) {
			keep3D(z3);
			steps << QStringLiteral("3D Z %1").arg(mm(z3.z));
		}
		//4: the four corners
		Gs2RectCheck r;
		if (ok) ok = gs2CheckRect(s, r, why) && stillHere(why);
		if (ok) {
			showFound(r);
			if (r.pass) {
				keepRect(r);
				steps << QStringLiteral("trolley centre X %1, Y %2, angle %3 deg, worst residual %4 mm")
					.arg(mm(r.fit.pose.cx), mm(r.fit.pose.cy), QString::number(r.fit.pose.angle * 180.0 / kPi, 'f', 3), mm(r.fit.maxResidual()));
			}
			else { ok = false; why = QStringLiteral("the corners: %1").arg(r.report); }
		}
		//5: the layout where the trolley is (the view draws it there, showFound), and every place production
		//will drive to for it inside the soft limits
		if (ok) {
			const QVector<gs2::RunPoint> pts = gs2::runPoints(s, r.fit.pose);
			ok = gs2CheckReach(s, pts, _gs2Session.z2D, z3.z, why);
			if (ok) steps << QStringLiteral("%1 slots, %2 units, %3 inspection points, all within reach").arg(s.slotList.size()).arg(s.units.size()).arg(pts.size());
			else why = QStringLiteral("the layout: %1").arg(why);
			ct::logger::info("[GuidedSetup2] Locate step 5: %d inspection points, %s", pts.size(), ok ? "all within reach" : qPrintable(why));
		}
		if (ok) {
			result = QStringLiteral("Locate passed in %1 s: %2. The layout shows the trolley where it was found.")
				.arg(timer.elapsed() / 1000.0, 0, 'f', 1).arg(steps.join(QStringLiteral("; ")));
			ct::logger::info("[GuidedSetup2] Locate passed in %.1f s", timer.elapsed() / 1000.0);
			AuditLog::instance().log(QStringLiteral("GS2_LOCATE"), QStringLiteral("passed"));
		}
		else if (!_stopRun) why += QStringLiteral(". ") + zsLiftAfterFailure(s.fid0.startZ, QStringLiteral("Start Z"));
		break;
	}
	default:
		break;
	}
	const bool stopped = _stopRun;
	gs2MachineIdle();

	if (!ok) {
		result = QStringLiteral("%1: %2.").arg(title, stopped ? QStringLiteral("stopped - nothing else was moved") : why);
		ct::logger::warn("[GuidedSetup2] %s failed: %s", qPrintable(title), qPrintable(stopped ? QStringLiteral("stopped") : why));
	}
	if (!_gs2Page) return;
	//a Find or Check that passed is shown by the page from the working copy; a taught value is not a
	//section done, so it is shown as still to do
	const bool runResult = action == P::TeachFind || action == P::TeachZ3DFind || action == P::TeachRectCheck;
	if (!ok || !runResult)
		showTeachResult(_gs2Page, action, result, ok && (action == P::TeachSnapEmpty || action == P::TeachScore || action == P::TeachLocate
			|| action == P::TeachMoveOnly || action == P::TeachSnapScan));
	if (ok && action == P::TeachLocate) gs2ShowLayoutView();
}
