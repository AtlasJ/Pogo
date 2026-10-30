//
//  Guided Setup (new): the glue between VisionApp and the new page / layout view.
//
//  Built alongside the old Guided Setup, which it never touches: its own right-panel page (added
//  to stackedWidget in code, so VisionApp.ui is not edited), its own layout view in the workspace
//  stack, and its own recipe file guidedSetup2.json. Build step 1 = the page, Save / Discard and
//  the layout view. Build step 2 = the circle finder and the camera direction check, which move
//  the machine the way a Z search does: zsMove, one modal busy window with a Stop, START refused.
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
#include <QFile>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QTimer>
#include <algorithm>
#include <cmath>
#include <limits>

namespace {
	constexpr int kSnapTimeoutMs = 4000;
	constexpr double kCentreTolMm = 0.1;     //Centre on circle: done once the circle is this close to the image centre
	constexpr int kCentreMoves = 3;          //a move lands within 0.05 mm, so one is normally enough
	constexpr double kDirStepMm = 5.0;       //direction check: +/- this on each axis, about 180 px at 27 um/px
	constexpr double kDirMaxScalePct = 0.3;  //0.3 % is 0.05 mm at a quarter of the image from the centre
	constexpr double kDirMaxAngleDeg = 0.15; //0.15 deg is 0.045 mm there
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
	constexpr int kQuietMs = 1500;
	constexpr int kQuietSnaps = 5;
	constexpr double kJitterPx = 1.0;
	constexpr int kStillMaxMs = 6000;        //the IPC took 0.8-4.7 s
	constexpr int kStillGapMs = 50;          //between snaps, so a Stop gets through
	constexpr double kExpectTolMm = 1.0;     //fid0 must be this close to where the pattern puts it; its neighbours are ~12.7 mm away
	const double kPi = 3.14159265358979323846;

	QString mm(double v) { return QString::number(v, 'f', 3); }
	QString signedMm(double v) { return (v >= 0.0 ? QStringLiteral("+") : QString()) + QString::number(v, 'f', 3); }
	QString signedNum(double v, int decimals) { return (v >= 0.0 ? QStringLiteral("+") : QString()) + QString::number(v, 'f', decimals); }

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

	_gs2CameraTimer = new QTimer(this);
	_gs2CameraTimer->setInterval(200);
	connect(_gs2CameraTimer, &QTimer::timeout, this, [=]() { gs2RefreshCamera(); });

	connect(_gs2Page, &GuidedSetup2Page::setupChanged, this, [=]() { if (_gs2View && _gs2Page) _gs2View->setSetup(_gs2Page->setup()); });
	connect(_gs2Page, &GuidedSetup2Page::showLayoutToggled, this, [=](bool on) { gs2ShowLayout(on); });
	connect(_gs2Page, &GuidedSetup2Page::toolRequested, this, [=](int action) { gs2RunTool(action); });
	connect(_gs2Page, &GuidedSetup2Page::conversionChosen, this, [=](bool measured) { gs2ChooseConversion(measured); });
	connect(_gs2Page, &GuidedSetup2Page::teachRequested, this, [=](int action) { gs2Teach(action); });
	connect(_gs2Page, &GuidedSetup2Page::patternBoxToggled, this, [=](bool on) { gs2ShowPatternBox(on); });
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
	_zsBusy = false;
}

/*
* Every circle in the diameter range, drawn on the FOV (green = the one picked); the one nearest
* the image centre is returned with its offset from the centre in world mm. Where the camera stood
* is the caller's: only it knows where this image was taken.
*/
bool VisionApp::gs2FindCircle(const QImage& image, double minDiaMm, double maxDiaMm, Gs2Circle& out, QString& why, const QPointF* nearPx,
	QVector<QPointF>* allPx)
{
	out = Gs2Circle();
	if (allPx) allPx->clear();
	if (image.isNull()) { why = QStringLiteral("there is no image"); return false; }
	double a[2][2];
	if (!gs2Conversion(image.width(), image.height(), a, why)) return false;
	const double mmPerPx = std::sqrt(std::abs(a[0][0] * a[1][1] - a[0][1] * a[1][0])); //the conversion's mean scale
	if (!(mmPerPx > 0.0)) { why = QStringLiteral("the pixel-to-mm conversion is empty"); return false; }
	if (!(minDiaMm > 0.0) || !(maxDiaMm > minDiaMm)) { why = QStringLiteral("the diameter range is not valid"); return false; }

	const QImage img = milReady(image);

	const double minPx = minDiaMm / mmPerPx;
	const double maxPx = maxDiaMm / mmPerPx;
	std::vector<mtrx::Circle> circles;
	MIL_ID mBuf = mtrx::to_milID(img);
	if (mBuf == M_NULL) { why = QStringLiteral("the image could not be handed to the circle finder"); return false; }
	MIL_ID mMono = mtrx::to_mono(mBuf);
	mtrx::find_circles(circles, mMono, minPx / 2.0, maxPx / 2.0, mtrx::FOREGROUND_ANY);
	mtrx::free_buffer(mMono);
	mtrx::free_buffer(mBuf);

	if (circles.empty()) {
		clearGuidedMarks();
		why = QStringLiteral("no circle between %1 and %2 mm (%3 .. %4 px) in the image")
			.arg(mm(minDiaMm), mm(maxDiaMm)).arg(std::lround(minPx)).arg(std::lround(maxPx));
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
bool VisionApp::gs2SnapCircle(double minDiaMm, double maxDiaMm, Gs2Circle& out, QString& why, const QPointF* nearPx)
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
	int snaps = 0;
	while (true) {
		if (_stopRun) { why = QStringLiteral("stopped"); return false; }
		const auto x = mc.get_position_mm(_motionID, 0, (int)Axis::X);
		const auto y = mc.get_position_mm(_motionID, 0, (int)Axis::Y);
		if (!x || !y) { why = QStringLiteral("could not read the X / Y position"); return false; }

		QImage img;
		if (!guidedSnapFov(img, kSnapTimeoutMs)) { why = QStringLiteral("no image from the camera"); return false; }
		if (_stopRun) { why = QStringLiteral("stopped"); return false; }
		Gs2Circle cur;
		if (!gs2FindCircle(img, minDiaMm, maxDiaMm, cur, why, nearPx)) return false;
		cur.haveStage = true;
		cur.stageX = *x;
		cur.stageY = *y;
		cur.wx = cur.stageX + cur.offX;
		cur.wy = cur.stageY + cur.offY;
		snaps++;

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
		//the encoders have been still a while: the jitter left is the finder's, averaged away over more snaps
		if (n >= kQuietSnaps && timer.elapsed() - quietSince >= kQuietMs) {
			const double spread = average(quiet, kQuietSnaps);
			if (spread <= kJitterPx) {
				how = QStringLiteral("encoders still %1 ms, last %2 within %3 px of their average")
					.arg(timer.elapsed() - quietSince).arg(kQuietSnaps).arg(spread, 0, 'f', 2);
				break;
			}
		}
		if (timer.elapsed() > kStillMaxMs) {
			if (n >= kQuietSnaps && timer.elapsed() - quietSince >= kQuietMs) {
				const double spread = average(quiet, kQuietSnaps); //out is not used after a failure
				QStringList at;
				for (int i = n - kQuietSnaps; i < n; i++) at << QStringLiteral("(%1, %2)").arg(quiet[i].px, 0, 'f', 2).arg(quiet[i].py, 0, 'f', 2);
				ct::logger::warn("[GuidedSetup2] Circle would not hold still with the encoders still: last %d at %s px",
					kQuietSnaps, qPrintable(at.join(QStringLiteral(" "))));
				why = QStringLiteral("the circle measurement would not hold still: with the encoders still, the last %1 snaps "
					"spread up to %2 px from their average (limit %3) - check the circle is clean, in focus and evenly lit")
					.arg(kQuietSnaps).arg(spread, 0, 'f', 2).arg(kJitterPx);
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
* that point is taken, and it must be within kExpectTolMm of it - with rows of identical circles on
* the trolley, "nearest the image centre" took a neighbour 13 mm away (IPC 2026-10-30 17:50).
* After a move the camera stands on that circle, so the centre is right again.
*/
bool VisionApp::gs2LocateCircle(double minDiaMm, double maxDiaMm, bool centre, Gs2Circle& out, int& moves, QString& why, const QPointF* expectPx)
{
	moves = 0;
	setProgress(_zsBusyWindow, QStringLiteral("Snap and find the circle..."));
	if (!gs2SnapCircle(minDiaMm, maxDiaMm, out, why, expectPx)) return false;
	if (expectPx) {
		const double mmPerPx = out.diaPx > 0.0 ? out.diaMm / out.diaPx : 0.0;
		const double offMm = std::hypot(out.px - expectPx->x(), out.py - expectPx->y()) * mmPerPx;
		ct::logger::info("[GuidedSetup2] Circle where the pattern puts it: expected (%.1f, %.1f) px, nearest at (%.1f, %.1f) px, %.3f mm away (limit %.1f)",
			expectPx->x(), expectPx->y(), out.px, out.py, offMm, kExpectTolMm);
		if (offMm > kExpectTolMm) {
			why = QStringLiteral("no circle where the pattern puts fid0 - the nearest is %1 mm from that spot (limit %2 mm)").arg(mm(offMm)).arg(kExpectTolMm, 0, 'f', 1);
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
		if (!gs2SnapCircle(minDiaMm, maxDiaMm, out, why)) return false;

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
bool VisionApp::gs2CheckDirection(double minDiaMm, double maxDiaMm, bool& pass, QString& report, QString& why)
{
	pass = false;
	_gs2LastMeasure = gs2::CameraCal();
	Gs2Circle c0;
	int moves = 0;
	if (!gs2LocateCircle(minDiaMm, maxDiaMm, true, c0, moves, why)) { why = QStringLiteral("centring on the circle: %1").arg(why); return false; }
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
		if (!zsMove({ { (int)Axis::X, x0 + s.dx }, { (int)Axis::Y, y0 + s.dy } }, why) || !gs2SnapCircle(minDiaMm, maxDiaMm, s.c, why)) { ok = false; break; }
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

void VisionApp::gs2RunTool(int action)
{
	if (!_gs2Page || _zsBusy) return;
	if (action == GuidedSetup2Page::ToolSaveMeasured) { gs2SaveMeasured(); return; }

	QString what;
	const gs2::DiaRange dia = _gs2Page->toolCircle(&what);
	if (!dia.valid()) {
		_gs2Page->showToolResult(QStringLiteral("Type the %1 diameter range first (min above 0, max above min).").arg(what), false);
		return;
	}
	//the circles are drawn on the camera image, so that is what the big image area shows
	if (!ui.graphicsViewFOV->isVisible()) toggleFOVView();

	auto describe = [&](const QString& title, const Gs2Circle& c) {
		return QStringLiteral("%1 (%2): %3 circle(s) in range; nearest %4 mm across (%5 px), score %6, %7 / %8 mm from the image centre.")
			.arg(title, what).arg(c.count).arg(mm(c.diaMm)).arg(c.diaPx, 0, 'f', 1).arg(c.score, 0, 'f', 2)
			.arg(signedMm(c.offX), signedMm(c.offY));
	};

	if (action == GuidedSetup2Page::ToolFindInImage) {
		Gs2Circle c;
		QString why;
		if (!gs2FindCircle(_imageFOV, dia.minMm, dia.maxMm, c, why)) {
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
	const QString title = direction ? QStringLiteral("Check camera direction") : (centre ? QStringLiteral("Centre on circle") : QStringLiteral("Snap + find"));
	if (!gs2MachineBusy(title, direction
		? QStringLiteral("The camera centres on the circle, snaps it from a 3 x 3 grid +/- %1 mm round it (about 20 s), and comes back.\n"
			"Everything else is locked until it finishes.").arg(kDirStepMm, 0, 'f', 0)
		: QStringLiteral("The camera snaps and moves onto the circle if it needs to.\nEverything else is locked until it finishes."))) return;

	const int trolleyEpoch = _zsTrolleyEpoch;
	QString why, text;
	Gs2Circle c;
	int moves = 0;
	bool pass = false;
	bool ok = direction ? gs2CheckDirection(dia.minMm, dia.maxMm, pass, text, why) : gs2LocateCircle(dia.minMm, dia.maxMm, centre, c, moves, why);
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
bool VisionApp::gs2FindFid0(const gs2::Fid0& f, const QString& dir, gs2::Fid0Found& out, QString& why)
{
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
	if (!gs2FindCircle(img, f.dia.minMm, f.dia.maxMm, any, w, &fid0Taught, &circles) || circles.isEmpty()) {
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
	if (!gs2LocateCircle(f.dia.minMm, f.dia.maxMm, true, circle, moves, why, &expect)) { why = QStringLiteral("fid0: %1").arg(why); return false; }

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

void VisionApp::gs2Teach(int action)
{
	if (!_gs2Page || _zsBusy) return;
	using P = GuidedSetup2Page;
	if (_gs2Page->recipeDir().isEmpty()) { _gs2Page->showFid0Result(QStringLiteral("Open a recipe first."), false); return; }
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
		if (!s.fid0.dia.valid()) { _gs2Page->showFid0Result(QStringLiteral("Type the fid0 circle diameter range first."), false); return; }
		Gs2Circle fid0Here;
		QString circleWhy;
		if (!gs2FindCircle(fov, s.fid0.dia.minMm, s.fid0.dia.maxMm, fid0Here, circleWhy)) {
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

	//── the rest moves the machine
	QString why, title, note;
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
		if (!s.fid0.dia.valid()) { _gs2Page->showFid0Result(QStringLiteral("Type the fid0 circle diameter range first."), false); return; }
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
		if (!gs2::sectionDone(s, gs2::SecFid0, &w) && w != QStringLiteral("Find has not passed with these settings")) {
			_gs2Page->showFid0Result(QStringLiteral("Find cannot start: %1.").arg(w), false);
			return;
		}
		if (s.fid0.stepMm < 0.1 || s.fid0.stepMm > s.fid0.stopZ - s.fid0.startZ) {
			_gs2Page->showFid0Result(QStringLiteral("Find cannot start: the max step must be between 0.1 mm and the Start-Stop range."), false);
			return;
		}
		if (!zsCheckTarget((int)Axis::X, s.fid0.xy.x, w) || !zsCheckTarget((int)Axis::Y, s.fid0.xy.y, w)
			|| !zsCheckTarget((int)Axis::Z, s.fid0.startZ, w) || !zsCheckTarget((int)Axis::Z, s.fid0.stopZ, w)) {
			_gs2Page->showFid0Result(QStringLiteral("Find cannot start: %1.").arg(w), false);
			return;
		}
		title = QStringLiteral("Find fid0");
		note = QStringLiteral("To the search XY at Start Z, Z sweep for the sharpest image (never past Stop Z), pattern, circle.");
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
		ok = gs2LocateCircle(s.fid0.dia.minMm, s.fid0.dia.maxMm, true, c, moves, why);
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
	case P::TeachFind: {
		gs2::Fid0Found found;
		ok = gs2FindFid0(s.fid0, dir, found, why);
		if (ok && trolleyEpoch != _zsTrolleyEpoch) { ok = false; why = QStringLiteral("the trolley was removed or attached meanwhile - the result was discarded"); }
		if (ok && _gs2Page->recipeDir() != dir) { ok = false; why = QStringLiteral("the recipe changed meanwhile - the result was discarded"); }
		if (ok) {
			_gs2Page->recordFid0Found(found);
			_gs2Session.fid0 = true;
			_gs2Session.fid0X = found.x;
			_gs2Session.fid0Y = found.y;
			_gs2Session.z2D = found.z;
			_gs2Session.trolleyEpoch = _zsTrolleyEpoch;
			_gs2Session.recipeDir = dir;
			AuditLog::instance().log(QStringLiteral("GS2_FID0_FIND"), QStringLiteral("X %1, Y %2, Z %3, score %4")
				.arg(mm(found.x), mm(found.y), mm(found.z)).arg(found.score, 0, 'f', 2));
		}
		else if (!_stopRun) {
			//after a failed Find Z goes back up to Start Z, as the Z search does - up only, never down to it
			why += QStringLiteral(". ") + zsLiftAfterFailure(s.fid0.startZ, QStringLiteral("Start Z"));
		}
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
	if (action == P::TeachSnapEmpty || action == P::TeachScore) _gs2Page->showSafetyScore(result, ok);
	else if (action != P::TeachFind || !ok) _gs2Page->showFid0Result(result, ok && action == P::TeachFind);
	//a Find that passed is shown by the page from the working copy
}
