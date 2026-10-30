// =============================================================================
//  VisionApp_ZSearch.cpp
//  Z search on the Guided Setup page: find this table's 2D Focus Z (camera
//  sharpness sweep) and 3D Centre Z (profiler centred on a flat area).
//
//  Phase 1 of the Z search work: the searches, their settings and the
//  camera-to-laser calibration. Production does not use any of it yet.
// =============================================================================

#include "VisionApp.h"
#include "AuditLog.h"
#include "3DOpticsTab.h"
#include "AxisMoveSequence.h"

#include <QSaveFile>
#include <QDir>
#include <QInputDialog>
#include <QThread>
#include <QTimer>
#include <atomic>
#include <cmath>
#include <memory>

namespace {
	/*
	* While a search waits inside its own event loop, user input is held off - except a click on
	* the on-screen Stop, which now lands at once (before, it was held until the search ended).
	* Everything else - other buttons, keys, shortcuts, closing the window - is dropped rather than
	* queued, so nothing fires after the search either. Application event filters only see objects
	* of the GUI thread. Window-level mouse events pass so Qt can route them to the widget under
	* the cursor; it is the widget-level event that is let through or eaten. Everything inside the
	* busy window (its own Stop) always passes - that window is modal and ignores Esc and close.
	*/
	class StopOnlyInput : public QObject {
	public:
		StopOnlyInput(QObject* stopButton, QWidget* busyWindow) : m_stop(stopButton), m_window(busyWindow) { qApp->installEventFilter(this); }
		~StopOnlyInput() override { qApp->removeEventFilter(this); }

	protected:
		bool eventFilter(QObject* obj, QEvent* e) override
		{
			if (m_window && obj->isWidgetType() && static_cast<QWidget*>(obj)->window() == m_window) return false;
			switch (e->type()) {
			case QEvent::MouseButtonPress:
			case QEvent::MouseButtonRelease:
			case QEvent::MouseButtonDblClick:
				return !obj->isWindowType() && obj != m_stop;
			case QEvent::KeyPress:
			case QEvent::KeyRelease:
			case QEvent::Shortcut:
			case QEvent::ShortcutOverride:
			case QEvent::Wheel:
			case QEvent::ContextMenu:
			case QEvent::TouchBegin:
			case QEvent::TouchUpdate:
			case QEvent::TouchEnd:
			case QEvent::TabletPress:
			case QEvent::TabletRelease:
			case QEvent::DragEnter:
			case QEvent::Drop:
				return true;
			case QEvent::Close:
				return e->spontaneous(); //the close box / Alt+F4 only - a close the program makes itself still happens
			default:
				return false;
			}
		}

	private:
		QObject* m_stop;
		QPointer<QWidget> m_window;
	};

	constexpr int kMotionReadDelayMs = 150;  //the pause JobThread gives the card before trusting move_done
	constexpr int kMoveTimeoutMs = 60000;
	constexpr int kSettleMs = 500;
	/*
	* "Landed" means within this of the target, by the card's feedback. X and Y use the same 0.05 mm
	* as every other move in the app (JobThread::settleAxis): their drives settle anywhere within about
	* +/-0.03 mm, and asking for 0.02 failed moves that were fine (2026-10-27 IPC log, X 0.022 past
	* every time - re-issuing cannot help, the card already counts itself there). Z keeps 0.02: it
	* landed within 0.013 mm, and the focus search steps it in fractions of a millimetre.
	*/
	constexpr double kOnTargetXYMm = 0.05;
	constexpr double kOnTargetZMm = 0.02;
	constexpr int kMaxCorrections = 2;

	constexpr double kMinCoarseStepMm = 0.1;
	constexpr double kMinFineStepMm = 0.05;  //well above kOnTargetZMm, or a fine step could be "already there"
	constexpr double kDropRatio = 0.7;       //peak passed: two frames in a row below 70% of the sharpest
	constexpr double kMinPeakRatio = 1.3;    //the sharpest must beat the dullest by 30% to be a focus peak at all
	constexpr int kMaxRefineStages = 5;
	constexpr double kFitFraction = 0.8;     //a stage's peak is fitted through every frame within 80% of its sharpest
	constexpr double kFlatRatio = 0.03;      //a stage whose frames differ by under 3% sees only noise: zooming further cannot help
	constexpr double kConvergedMm = 0.02;    //a stage that moved the peak less than a move can land is the last one worth doing

	constexpr int kProfileShots = 5;
	constexpr double kProfileCentreFraction = 1.0 / 3.0; //only the middle third of the laser line is read
	constexpr int kMinProfilePoints = 20;
	constexpr double kSlopeStepMm = 0.5;
	constexpr double kCentreTolMm = 0.02;
	constexpr int kCentreMoves = 4;

	const char* const kAxisName[3] = { "X", "Y", "Z" };

	QString axisName(int axis) { return (axis >= 0 && axis < 3) ? kAxisName[axis] : QStringLiteral("?"); }
	double onTargetMm(int axis) { return axis == (int)Axis::Z ? kOnTargetZMm : kOnTargetXYMm; }
	QString mm(double v) { return QString::number(v, 'f', 3); }

	QString calibrationPath()
	{
		return QDir(Common::Directory::ConfigPath()).filePath(QStringLiteral("z_calibration.json"));
	}
}

/*
* Two frames on Guided Setup, above the fiducials: the Z has to be known before anything on the
* board can be seen in focus. Built in code rather than in VisionApp.ui, which is shared with
* Designer, and styled from the page's own focus frame so they look like the rest of the page.
*/
void VisionApp::initZSearch()
{
	loadZCalibration();

	const QString style = ui.frame_gsFocus->styleSheet()
		+ QStringLiteral("QLineEdit{background-color:rgb(27,29,35); color:#F0F0F0; border:1px solid rgb(27,29,35); border-radius:5px; padding-left:6px; min-height:22px;}");

	auto makeFrame = [&](const QString& title, QGridLayout*& grid) {
		auto* f = new QFrame(ui.GuidedSetupPage);
		f->setFrameShape(QFrame::StyledPanel);
		f->setStyleSheet(style);
		auto* v = new QVBoxLayout(f);
		auto* t = new QLabel(title, f);
		QFont font = t->font();
		font.setPointSize(10);
		font.setBold(true);
		t->setFont(font);
		v->addWidget(t);
		grid = new QGridLayout();
		grid->setHorizontalSpacing(6);
		v->addLayout(grid);
		return f;
	};
	auto spin = [](QWidget* parent, double lo, double hi, int decimals, double step, const QString& tip) {
		auto* s = new QDoubleSpinBox(parent);
		s->setRange(lo, hi);
		s->setDecimals(decimals);
		s->setSingleStep(step);
		s->setFixedWidth(95);
		s->setKeyboardTracking(false); //one save when the edit is finished, not one per keystroke
		s->setToolTip(tip);
		return s;
	};
	auto ispin = [](QWidget* parent, int lo, int hi, const QString& tip) {
		auto* s = new QSpinBox(parent);
		s->setRange(lo, hi);
		s->setFixedWidth(95);
		s->setKeyboardTracking(false);
		s->setToolTip(tip);
		return s;
	};
	auto button = [](QWidget* parent, const QString& text, const QString& tip) {
		auto* b = new QToolButton(parent);
		b->setText(text);
		b->setToolTip(tip);
		b->setMinimumHeight(28);
		return b;
	};
	auto row = [](QWidget* parent, std::initializer_list<QWidget*> widgets) {
		auto* h = new QHBoxLayout();
		for (auto* w : widgets) h->addWidget(w);
		h->addStretch();
		return h;
	};
	/*
	* Positions are shown, never typed: they only come from Set (or Teach XY), which reads where the
	* machine actually is - so a stored position is one the machine has physically reached.
	*/
	auto readOnly = [](QWidget* parent, int width = 95) {
		auto* e = new QLineEdit(parent);
		e->setReadOnly(true);
		e->setFixedWidth(width);
		e->setText(QStringLiteral("-"));
		return e;
	};
	auto status = [](QWidget* parent) {
		auto* l = new QLabel(parent);
		l->setWordWrap(true);
		return l;
	};

	//current machine position from the card, not the UI's 300 ms poll
	auto readPosition = [=](double& x, double& y, double& z) {
		auto& mc = MotionController::instance();
		const auto px = mc.get_position_mm(_motionID, 0, (int)Axis::X);
		const auto py = mc.get_position_mm(_motionID, 0, (int)Axis::Y);
		const auto pz = mc.get_position_mm(_motionID, 0, (int)Axis::Z);
		if (!px || !py || !pz) { showMsg("Failed to read the current position. Is motion connected?"); return false; }
		x = *px; y = *py; z = *pz;
		return true;
	};
	auto changed = [=]() { saveRecipeConfig(); refreshZSearchUi(); };
	auto bind = [=](QDoubleSpinBox* s, std::function<void(double)> apply) {
		connect(s, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [=](double v) { apply(v); changed(); });
	};
	auto bindInt = [=](QSpinBox* s, std::function<void(int)> apply) {
		connect(s, QOverload<int>::of(&QSpinBox::valueChanged), this, [=](int v) { apply(v); changed(); });
	};

	//Set XY / Set Z / Jog to XY / Jog to Z for one frame
	auto setXY = [=](ZSearch2DConfig* c2, ZSearch3DConfig* c3) {
		if (_zsBusy) return;
		double x, y, z;
		if (!readPosition(x, y, z)) return;
		if (c2) { c2->x = x; c2->y = y; c2->xySet = true; }
		if (c3) { c3->x = x; c3->y = y; c3->xySet = true; }
		changed();
		AuditLog::instance().log(QStringLiteral("ZSEARCH_SET_XY"), QStringLiteral("%1 %2, %3").arg(c2 ? "2D" : "3D").arg(mm(x)).arg(mm(y)));
	};
	auto setZ = [=](double* target, bool* flag, const QString& what) {
		if (_zsBusy) return;
		double x, y, z;
		if (!readPosition(x, y, z)) return;
		*target = z;
		*flag = true;
		changed();
		AuditLog::instance().log(QStringLiteral("ZSEARCH_SET_Z"), QStringLiteral("%1 %2").arg(what).arg(mm(z)));
	};
	/*
	* While the machine moves, a modal "moving" window with its own Stop covers the screen, so the
	* lock is visible; other input is dropped (StopOnlyInput). Anything still queued is let through
	* while the frames are disabled, so it cannot run the same command again afterwards.
	*/
	auto busy = [=](bool on) {
		if (on) _zsBusyWindow = showBusyWindow(QStringLiteral("Moving"), QStringLiteral(
			"The machine is moving for the Z search. Everything else is locked until it arrives.\n"
			"Stop ends the move at once; the panel Stop and the e-stop also stop it."));
		else closeBusyWindow(_zsBusyWindow);
		if (!on) QCoreApplication::processEvents();
		_zsw.frame2D->setEnabled(!on);
		_zsw.frame3D->setEnabled(!on);
		_zsBusy = on;
		if (on) _stopRun = false;
	};
	auto jog = [=](std::function<bool(QString&)> move) {
		if (zsRefuseIfBusy()) return;
		busy(true);
		QString why;
		const bool ok = move(why);
		busy(false);
		if (!ok) showMsg(QStringLiteral("The machine did not move there.\n\n%1").arg(why));
	};
	auto jogXY = [=](bool set, double x, double y, bool clearSet, double clearZ) {
		if (!set) { showMsg("Set the Search XY first."); return; }
		if (!clearSet) { showMsg("Set Start Z first: Jog to XY lifts Z to it before moving sideways."); return; }
		jog([=](QString& why) { return zsMoveToXY(x, y, clearZ, why); });
	};
	auto jogZ = [=](bool set, double z, const QString& what) {
		if (!set) { showMsg(QStringLiteral("Set %1 first.").arg(what)); return; }
		jog([=](QString& why) { return zsMove({ { (int)Axis::Z, z } }, why); });
	};

	//── 2D Focus Z
	QGridLayout* g = nullptr;
	_zsw.frame2D = makeFrame(QStringLiteral("2D Focus Z"), g);
	QFrame* f2 = _zsw.frame2D;
	ZSearch2DConfig* const c2 = &_zSearch.s2d; //a member, so the lambdas below can outlive this function

	_zsw.xy2 = readOnly(f2, 196);
	_zsw.xy2->setToolTip(QStringLiteral("Search X, Y of the camera, mm - set with Set XY or Teach XY only"));
	auto* bSetXY2 = button(f2, QStringLiteral("Set XY"), QStringLiteral("Store where the camera is now as the 2D Search XY."));
	auto* bTeachXY2 = button(f2, QStringLiteral("Teach XY"), QStringLiteral("Centre the camera on the screw head with the circle finder (Circle size below), then store that XY.\nThe image must already be roughly in focus."));
	auto* bJogXY2 = button(f2, QStringLiteral("Jog to XY"), QStringLiteral("Lift Z to Start Z if it is lower, then move to the Search XY."));
	g->addWidget(new QLabel(QStringLiteral("Search XY"), f2), 0, 0);
	g->addWidget(_zsw.xy2, 0, 1, 1, 2);
	g->addLayout(row(f2, { bSetXY2, bTeachXY2, bJogXY2 }), 0, 3);

	_zsw.start2 = readOnly(f2);
	_zsw.start2->setToolTip(QStringLiteral("Top of the search (+ Z is DOWN, so this is the smaller Z). The sweep starts here. Set with Set Z only."));
	auto* bSetStart2 = button(f2, QStringLiteral("Set Z"), QStringLiteral("Store the current Z as Start Z."));
	auto* bJogStart2 = button(f2, QStringLiteral("Jog to Z"), QStringLiteral("Move Z only, at the current XY."));
	g->addWidget(new QLabel(QStringLiteral("Start Z"), f2), 1, 0);
	g->addWidget(_zsw.start2, 1, 1);
	g->addLayout(row(f2, { bSetStart2, bJogStart2 }), 1, 3);

	_zsw.stop2 = readOnly(f2);
	_zsw.stop2->setToolTip(QStringLiteral("Bottom of the search and its collision limit: the head never goes below this.\nIt must clear the HIGHEST table this machine can see. Set with Set Z only."));
	auto* bSetStop2 = button(f2, QStringLiteral("Set Z"), QStringLiteral("Store the current Z as Stop Z."));
	auto* bJogStop2 = button(f2, QStringLiteral("Jog to Z"), QStringLiteral("Move Z only, at the current XY."));
	g->addWidget(new QLabel(QStringLiteral("Stop Z"), f2), 2, 0);
	g->addWidget(_zsw.stop2, 2, 1);
	g->addLayout(row(f2, { bSetStop2, bJogStop2 }), 2, 3);

	//a short grey line beside a setting, saying what it is without opening the tooltip
	auto hint = [=](QWidget* parent, const QString& text) {
		auto* l = status(parent);
		l->setText(text);
		l->setStyleSheet(QStringLiteral("color:#9AA0AA;"));
		return l;
	};

	_zsw.step2 = spin(f2, kMinCoarseStepMm, 20, 2, 0.1, QStringLiteral("Step of the first, coarse sweep down from Start Z. It only has to land on the sharpness hill, not its top -\nthe refine stages find the top - but keep it under about a third of the hill's width or the hill can be stepped over."));
	_zsw.step2->setSuffix(QStringLiteral(" mm"));
	g->addWidget(new QLabel(QStringLiteral("Coarse step"), f2), 3, 0);
	g->addWidget(_zsw.step2, 3, 1);
	g->addWidget(hint(f2, QStringLiteral("first sweep, from Start Z down")), 3, 2, 1, 2);

	_zsw.stages2 = ispin(f2, 1, kMaxRefineStages, QStringLiteral("At most this many zoom stages after the coarse sweep. Each goes back over +/- one step of the stage before\nat a fifth of its step, and the search stops early once a stage changes nothing (moved under 0.02 mm, frames\nall within 3% of each other, or the step would go under 0.05 mm)."));
	g->addWidget(new QLabel(QStringLiteral("Refine stages"), f2), 4, 0);
	g->addWidget(_zsw.stages2, 4, 1);
	g->addWidget(hint(f2, QStringLiteral("each a fifth of the last step; stops early")), 4, 2, 1, 2);

	_zsw.box2 = ispin(f2, 32, 4000, QStringLiteral("Sharpness is scored only inside a square of this size at the centre of the camera image.\nSize it to about 1.2 - 1.5x the screw head, so the table and other heights stay out of it."));
	_zsw.box2->setSuffix(QStringLiteral(" px"));
	g->addWidget(new QLabel(QStringLiteral("Focus box"), f2), 5, 0);
	g->addWidget(_zsw.box2, 5, 1);
	g->addWidget(hint(f2, QStringLiteral("square at the image centre where sharpness is scored")), 5, 2, 1, 2);

	_zsw.circleMin2 = ispin(f2, 4, 4000, QStringLiteral("Teach XY: smallest screw head DIAMETER the circle finder accepts, px"));
	_zsw.circleMax2 = ispin(f2, 4, 4000, QStringLiteral("Teach XY: largest screw head DIAMETER the circle finder accepts, px"));
	_zsw.circleMin2->setSuffix(QStringLiteral(" px"));
	_zsw.circleMax2->setSuffix(QStringLiteral(" px"));
	g->addWidget(new QLabel(QStringLiteral("Circle diameter"), f2), 6, 0);
	g->addWidget(_zsw.circleMin2, 6, 1);
	g->addWidget(_zsw.circleMax2, 6, 2);

	_zsw.confirm2 = new QCheckBox(QStringLiteral("Confirm with Safety Check"), f2);
	_zsw.confirm2->setToolTip(QStringLiteral("A Z only counts as in focus if this recipe's Safety Check passes on that frame."));
	g->addWidget(_zsw.confirm2, 7, 0, 1, 4);

	auto* bFind2 = button(f2, QStringLiteral("Find Z"), QStringLiteral("Move to the Search XY at Start Z, sweep down until the sharpness has peaked, then zoom in on the peak.\nNever passes Stop Z. On failure Z goes straight back up to Start Z."));
	_zsw.found2 = readOnly(f2);
	_zsw.found2->setToolTip(QStringLiteral("2D Focus Z found on this trolley (not saved)"));
	g->addWidget(bFind2, 8, 0);
	g->addWidget(_zsw.found2, 8, 1);
	_zsw.status2 = status(f2);
	g->addWidget(_zsw.status2, 9, 0, 1, 4);

	bind(_zsw.step2, [c2](double v) { c2->stepMm = v; });
	bindInt(_zsw.stages2, [c2](int v) { c2->refineStages = v; });
	bindInt(_zsw.box2, [c2](int v) { c2->boxPx = v; });
	bindInt(_zsw.circleMin2, [c2](int v) { c2->circleMinPx = v; });
	bindInt(_zsw.circleMax2, [c2](int v) { c2->circleMaxPx = v; });
	connect(_zsw.confirm2, &QCheckBox::toggled, this, [=](bool on) { c2->confirmSafetyCheck = on; changed(); });

	connect(bSetXY2, &QToolButton::clicked, this, [=]() { setXY(c2, nullptr); });
	connect(bTeachXY2, &QToolButton::clicked, this, [=]() {
		if (zsRefuseIfBusy()) return;
		if (c2->circleMinPx > c2->circleMaxPx) { showMsg("The circle size range is empty (min is above max)."); return; }
		busy(true);
		mtrx::Circle circle;
		QString why;
		const bool ok = guidedCentreOnCircle(c2->circleMinPx, c2->circleMaxPx, circle, why);
		busy(false);
		if (!ok) { showMsg(QStringLiteral("The Search XY was not taught.\n\n%1").arg(why)); return; }
		setXY(c2, nullptr);
	});
	connect(bJogXY2, &QToolButton::clicked, this, [=]() { jogXY(c2->xySet, c2->x, c2->y, c2->startSet, c2->startZ); });
	connect(bSetStart2, &QToolButton::clicked, this, [=]() { setZ(&c2->startZ, &c2->startSet, QStringLiteral("2D start")); });
	connect(bJogStart2, &QToolButton::clicked, this, [=]() { jogZ(c2->startSet, c2->startZ, QStringLiteral("Start Z")); });
	connect(bSetStop2, &QToolButton::clicked, this, [=]() { setZ(&c2->stopZ, &c2->stopSet, QStringLiteral("2D stop")); });
	connect(bJogStop2, &QToolButton::clicked, this, [=]() { jogZ(c2->stopSet, c2->stopZ, QStringLiteral("Stop Z")); });
	connect(bFind2, &QToolButton::clicked, this, [=]() { zsRunFind(false); });

	//── 3D Centre Z
	_zsw.frame3D = makeFrame(QStringLiteral("3D Centre Z"), g);
	QFrame* f3 = _zsw.frame3D;
	ZSearch3DConfig* const c3 = &_zSearch.s3d;

	_zsw.xy3 = readOnly(f3, 196);
	_zsw.xy3->setToolTip(QStringLiteral("Search X, Y of the CAMERA over the flat area, mm - the laser goes there plus the laser offset. Set with Set XY only."));
	auto* bSetXY3 = button(f3, QStringLiteral("Set XY"), QStringLiteral("Store where the CAMERA is now. The search puts the laser line on this point."));
	auto* bJogXY3 = button(f3, QStringLiteral("Jog to XY"), QStringLiteral("Lift Z to Start Z if it is lower, then move the CAMERA to the Search XY."));
	g->addWidget(new QLabel(QStringLiteral("Search XY"), f3), 0, 0);
	g->addWidget(_zsw.xy3, 0, 1, 1, 2);
	g->addLayout(row(f3, { bSetXY3, bJogXY3 }), 0, 3);
	_zsw.laserAt3 = status(f3);
	g->addWidget(_zsw.laserAt3, 1, 1, 1, 3);

	_zsw.start3 = readOnly(f3);
	_zsw.start3->setToolTip(QStringLiteral("Top of the search (+ Z is DOWN, so this is the smaller Z). Set with Set Z only."));
	auto* bSetStart3 = button(f3, QStringLiteral("Set Z"), QStringLiteral("Store the current Z as Start Z."));
	auto* bJogStart3 = button(f3, QStringLiteral("Jog to Z"), QStringLiteral("Move Z only, at the current XY."));
	g->addWidget(new QLabel(QStringLiteral("Start Z"), f3), 2, 0);
	g->addWidget(_zsw.start3, 2, 1);
	g->addLayout(row(f3, { bSetStart3, bJogStart3 }), 2, 3);

	_zsw.stop3 = readOnly(f3);
	_zsw.stop3->setToolTip(QStringLiteral("Bottom of the search and its collision limit: the head never goes below this. Set with Set Z only."));
	auto* bSetStop3 = button(f3, QStringLiteral("Set Z"), QStringLiteral("Store the current Z as Stop Z."));
	auto* bJogStop3 = button(f3, QStringLiteral("Jog to Z"), QStringLiteral("Move Z only, at the current XY."));
	g->addWidget(new QLabel(QStringLiteral("Stop Z"), f3), 3, 0);
	g->addWidget(_zsw.stop3, 3, 1);
	g->addLayout(row(f3, { bSetStop3, bJogStop3 }), 3, 3);

	_zsw.step3 = spin(f3, kMinCoarseStepMm, 20, 2, 0.1, QStringLiteral("Step while the profiler reads nothing. Must stay well inside the profiler's range or a surface can be stepped over."));
	_zsw.step3->setSuffix(QStringLiteral(" mm"));
	g->addWidget(new QLabel(QStringLiteral("Step"), f3), 4, 0);
	g->addWidget(_zsw.step3, 4, 1);
	g->addWidget(hint(f3, QStringLiteral("down from Start Z until the profiler reads")), 4, 2, 1, 2);

	_zsw.target3 = spin(f3, -20, 20, 3, 0.1, QStringLiteral("The profiler reading that counts as centred. 0 = the head's reference distance, the middle of its range."));
	_zsw.target3->setSuffix(QStringLiteral(" mm"));
	g->addWidget(new QLabel(QStringLiteral("Target reading"), f3), 5, 0);
	g->addWidget(_zsw.target3, 5, 1);
	g->addWidget(hint(f3, QStringLiteral("0 = middle of the profiler's range")), 5, 2, 1, 2);

	_zsw.spread3 = spin(f3, 0.005, 5, 3, 0.01, QStringLiteral("A read counts as a flat surface only if the middle 50% of the points in the centre of the laser line lie within this."));
	_zsw.spread3->setSuffix(QStringLiteral(" mm"));
	g->addWidget(new QLabel(QStringLiteral("Max spread"), f3), 6, 0);
	g->addWidget(_zsw.spread3, 6, 1);
	g->addWidget(hint(f3, QStringLiteral("how flat a read must be to count")), 6, 2, 1, 2);

	auto* bFind3 = button(f3, QStringLiteral("Find Z"), QStringLiteral("Put the laser on the Search XY, step down from Start Z until the profiler reads a flat surface, then centre it.\nNever passes Stop Z. On failure Z goes straight back up to Start Z."));
	_zsw.found3 = readOnly(f3);
	_zsw.found3->setToolTip(QStringLiteral("3D Centre Z found on this trolley (not saved)"));
	g->addWidget(bFind3, 7, 0);
	g->addWidget(_zsw.found3, 7, 1);
	_zsw.status3 = status(f3);
	g->addWidget(_zsw.status3, 8, 0, 1, 4);

	//the 2D - 3D difference and the machine's camera-to-laser constant
	_zsw.diffNow = readOnly(f3);
	_zsw.diffNow->setToolTip(QStringLiteral("2D Focus Z - 3D Centre Z found now"));
	_zsw.diffTaught = readOnly(f3);
	_zsw.diffTaught->setToolTip(QStringLiteral("2D Focus Z - 3D Centre Z as taught for this recipe"));
	_zsw.diffTol = spin(f3, 0.01, 10, 2, 0.05, QStringLiteral("How far a run's difference may stray from the taught one before it is refused (from Phase 3)."));
	//"±" by its code point: the file is UTF-8 without a BOM, so MSVC reads a literal "±" in the
	//Windows code page and the page showed "Â±"
	_zsw.diffTol->setPrefix(QString(QChar(0x00B1)) + QStringLiteral(" "));
	_zsw.diffTol->setSuffix(QStringLiteral(" mm"));
	auto* bSetDiff = button(f3, QStringLiteral("Set Difference"), QStringLiteral("Store 2D Focus Z - 3D Centre Z, both found on this trolley, as this recipe's taught difference."));
	g->addWidget(new QLabel(QStringLiteral("2D - 3D"), f3), 9, 0);
	g->addWidget(_zsw.diffNow, 9, 1);
	g->addWidget(_zsw.diffTaught, 9, 2);
	g->addLayout(row(f3, { _zsw.diffTol, bSetDiff }), 9, 3);

	_zsw.calibration = status(f3);
	auto* bCalibrate = button(f3, QStringLiteral("Calibrate Here"), QStringLiteral("Machine-wide: run the 2D and the 3D search at the CURRENT XY, over one flat textured surface,\nand store 2D Focus Z - 3D Centre Z as the camera-to-laser Z. Engineer login."));
	auto* bCopy = button(f3, QStringLiteral("Copy From Recipe..."), QStringLiteral("Replace this recipe's Z search settings with another recipe's (the trolley is the same)."));
	g->addWidget(_zsw.calibration, 10, 0, 1, 3);
	g->addLayout(row(f3, { bCalibrate, bCopy }), 10, 3);

	bind(_zsw.step3, [c3](double v) { c3->stepMm = v; });
	bind(_zsw.target3, [c3](double v) { c3->targetMm = v; });
	bind(_zsw.spread3, [c3](double v) { c3->maxSpreadMm = v; });
	bind(_zsw.diffTol, [=](double v) { _zSearch.diffTolMm = v; });

	connect(bSetXY3, &QToolButton::clicked, this, [=]() { setXY(nullptr, c3); });
	connect(bJogXY3, &QToolButton::clicked, this, [=]() { jogXY(c3->xySet, c3->x, c3->y, c3->startSet, c3->startZ); });
	connect(bSetStart3, &QToolButton::clicked, this, [=]() { setZ(&c3->startZ, &c3->startSet, QStringLiteral("3D start")); });
	connect(bJogStart3, &QToolButton::clicked, this, [=]() { jogZ(c3->startSet, c3->startZ, QStringLiteral("Start Z")); });
	connect(bSetStop3, &QToolButton::clicked, this, [=]() { setZ(&c3->stopZ, &c3->stopSet, QStringLiteral("3D stop")); });
	connect(bJogStop3, &QToolButton::clicked, this, [=]() { jogZ(c3->stopSet, c3->stopZ, QStringLiteral("Stop Z")); });
	connect(bFind3, &QToolButton::clicked, this, [=]() { zsRunFind(true); });
	connect(bSetDiff, &QToolButton::clicked, this, [=]() { zsSetDifference(); });
	connect(bCalibrate, &QToolButton::clicked, this, [=]() { zsCalibrateCamLaser(); });
	connect(bCopy, &QToolButton::clicked, this, [=]() { zsCopyFromRecipe(); });

	//before the fiducial frame: Z first, then everything that needs it in focus
	int at = ui.vl_guidedSetupPage->indexOf(ui.frame_gsFiducial);
	if (at < 0) at = ui.vl_guidedSetupPage->count();
	ui.vl_guidedSetupPage->insertWidget(at, _zsw.frame2D);
	ui.vl_guidedSetupPage->insertWidget(at + 1, _zsw.frame3D);

	/*
	* The 3D Optics live profile reads the profiler from the job thread every 200 ms. A 3D search
	* reading it at the same time from here would share the controller with it, and would switch live
	* mode off under it when it finishes - so the search refuses while that view is on.
	*/
	if (_optics3DTab)
		connect(_optics3DTab, &Optics3DTab::alignLiveProfile, this, [=](bool on) { _zsLiveProfileOn = on; });

	//what was found belongs to the trolley it was found on
	connect(&MachineController::instance(), &MachineController::signalTrolleyGuard, this, [=](bool on) {
		_zsTrolleyEpoch++; //a search running now measured a table that has changed - see zsRunFind
		if (on || (!_zFound.has2D && !_zFound.has3D)) return;
		_zFound = ZFound();
		ct::logger::info("[ZSearch] Trolley removed - the found 2D/3D Z values are cleared");
		refreshZSearchUi();
	}, Qt::QueuedConnection);

	refreshZSearchUi();
}

void VisionApp::refreshZSearchUi()
{
	if (!_zsw.frame2D || !_zsw.frame3D) return; //before initZSearch: a recipe can load first

	const auto& c2 = _zSearch.s2d;
	const auto& c3 = _zSearch.s3d;
	{
		const QList<QAbstractSpinBox*> spins = {
			_zsw.step2, _zsw.stages2, _zsw.box2, _zsw.circleMin2, _zsw.circleMax2,
			_zsw.step3, _zsw.target3, _zsw.spread3, _zsw.diffTol };
		std::vector<std::unique_ptr<QSignalBlocker>> blockers;
		for (auto* s : spins) blockers.emplace_back(new QSignalBlocker(s));
		QSignalBlocker bc(_zsw.confirm2);

		_zsw.step2->setValue(c2.stepMm);
		_zsw.stages2->setValue(c2.refineStages);
		_zsw.box2->setValue(c2.boxPx);
		_zsw.circleMin2->setValue(c2.circleMinPx);
		_zsw.circleMax2->setValue(c2.circleMaxPx);
		_zsw.confirm2->setChecked(c2.confirmSafetyCheck);

		_zsw.step3->setValue(c3.stepMm);
		_zsw.target3->setValue(c3.targetMm);
		_zsw.spread3->setValue(c3.maxSpreadMm);
		_zsw.diffTol->setValue(_zSearch.diffTolMm);
	}

	const QString notSet = QStringLiteral("not set");
	_zsw.xy2->setText(c2.xySet ? QStringLiteral("%1, %2").arg(mm(c2.x)).arg(mm(c2.y)) : notSet);
	_zsw.start2->setText(c2.startSet ? mm(c2.startZ) : notSet);
	_zsw.stop2->setText(c2.stopSet ? mm(c2.stopZ) : notSet);
	_zsw.xy3->setText(c3.xySet ? QStringLiteral("%1, %2").arg(mm(c3.x)).arg(mm(c3.y)) : notSet);
	_zsw.start3->setText(c3.startSet ? mm(c3.startZ) : notSet);
	_zsw.stop3->setText(c3.stopSet ? mm(c3.stopZ) : notSet);

	_zsw.found2->setText(_zFound.has2D ? mm(_zFound.z2D) : QStringLiteral("-"));
	_zsw.found3->setText(_zFound.has3D ? mm(_zFound.z3D) : QStringLiteral("-"));
	_zsw.laserAt3->setText(c3.xySet
		? QStringLiteral("Laser lands at %1, %2 (laser offset %3, %4)")
			.arg(mm(c3.x + _laserConfig.offset.wx)).arg(mm(c3.y + _laserConfig.offset.wy))
			.arg(mm(_laserConfig.offset.wx)).arg(mm(_laserConfig.offset.wy))
		: QStringLiteral("Laser lands at the Search XY + the laser offset"));
	_zsw.diffNow->setText(_zFound.has2D && _zFound.has3D ? mm(_zFound.z2D - _zFound.z3D) : QStringLiteral("-"));
	_zsw.diffTaught->setText(_zSearch.diffSet ? mm(_zSearch.diffMm) : QStringLiteral("not set"));
	_zsw.calibration->setText(_zCalibration.set
		? QStringLiteral("Camera-to-laser Z (machine): %1 mm, %2").arg(mm(_zCalibration.camLaserDzMm)).arg(_zCalibration.when)
		: QStringLiteral("Camera-to-laser Z (machine): not calibrated"));

	//while idle the status lines say what is still missing; during a search they show progress
	if (!_zsBusy) {
		auto missing = [](bool xy, bool start, bool stop) {
			QStringList m;
			if (!xy) m << QStringLiteral("Search XY");
			if (!start) m << QStringLiteral("Start Z");
			if (!stop) m << QStringLiteral("Stop Z");
			return m.isEmpty() ? QString() : QStringLiteral("Not set: %1").arg(m.join(QStringLiteral(", ")));
		};
		const QString m2 = missing(c2.xySet, c2.startSet, c2.stopSet);
		const QString m3 = missing(c3.xySet, c3.startSet, c3.stopSet);
		if (!m2.isEmpty() || _zsw.status2->text().isEmpty() || _zsw.status2->text().startsWith(QStringLiteral("Not set")))
			_zsw.status2->setText(m2);
		if (!m3.isEmpty() || _zsw.status3->text().isEmpty() || _zsw.status3->text().startsWith(QStringLiteral("Not set")))
			_zsw.status3->setText(m3);
	}
}

QJsonObject VisionApp::zSearchToJson() const
{
	const auto& c2 = _zSearch.s2d;
	const auto& c3 = _zSearch.s3d;

	QJsonObject o2;
	o2.insert(QStringLiteral("xySet"), c2.xySet);
	o2.insert(QStringLiteral("x"), c2.x);
	o2.insert(QStringLiteral("y"), c2.y);
	o2.insert(QStringLiteral("startSet"), c2.startSet);
	o2.insert(QStringLiteral("startZ"), c2.startZ);
	o2.insert(QStringLiteral("stopSet"), c2.stopSet);
	o2.insert(QStringLiteral("stopZ"), c2.stopZ);
	o2.insert(QStringLiteral("stepMm"), c2.stepMm);
	o2.insert(QStringLiteral("refineStages"), c2.refineStages);
	o2.insert(QStringLiteral("boxPx"), c2.boxPx);
	o2.insert(QStringLiteral("circleMinPx"), c2.circleMinPx);
	o2.insert(QStringLiteral("circleMaxPx"), c2.circleMaxPx);
	o2.insert(QStringLiteral("confirmSafetyCheck"), c2.confirmSafetyCheck);

	QJsonObject o3;
	o3.insert(QStringLiteral("xySet"), c3.xySet);
	o3.insert(QStringLiteral("x"), c3.x);
	o3.insert(QStringLiteral("y"), c3.y);
	o3.insert(QStringLiteral("startSet"), c3.startSet);
	o3.insert(QStringLiteral("startZ"), c3.startZ);
	o3.insert(QStringLiteral("stopSet"), c3.stopSet);
	o3.insert(QStringLiteral("stopZ"), c3.stopZ);
	o3.insert(QStringLiteral("stepMm"), c3.stepMm);
	o3.insert(QStringLiteral("targetMm"), c3.targetMm);
	o3.insert(QStringLiteral("maxSpreadMm"), c3.maxSpreadMm);

	QJsonObject o;
	o.insert(QStringLiteral("2d"), o2);
	o.insert(QStringLiteral("3d"), o3);
	o.insert(QStringLiteral("diffSet"), _zSearch.diffSet);
	o.insert(QStringLiteral("diffMm"), _zSearch.diffMm);
	o.insert(QStringLiteral("diffTolMm"), _zSearch.diffTolMm);
	return o;
}

void VisionApp::zSearchFromJson(const QJsonObject& o)
{
	_zSearch = ZSearchConfig(); //absent keys = defaults, nothing set
	const ZSearchConfig d;

	const QJsonObject o2 = o.value(QStringLiteral("2d")).toObject();
	auto& c2 = _zSearch.s2d;
	c2.xySet = jsonHelper::getBool(o2, QStringLiteral("xySet"), false);
	c2.x = jsonHelper::getDouble(o2, QStringLiteral("x"), 0.0);
	c2.y = jsonHelper::getDouble(o2, QStringLiteral("y"), 0.0);
	c2.startSet = jsonHelper::getBool(o2, QStringLiteral("startSet"), false);
	c2.startZ = jsonHelper::getDouble(o2, QStringLiteral("startZ"), 0.0);
	c2.stopSet = jsonHelper::getBool(o2, QStringLiteral("stopSet"), false);
	c2.stopZ = jsonHelper::getDouble(o2, QStringLiteral("stopZ"), 0.0);
	c2.stepMm = std::max(kMinCoarseStepMm, jsonHelper::getDouble(o2, QStringLiteral("stepMm"), d.s2d.stepMm));
	c2.refineStages = std::max(1, std::min(kMaxRefineStages, jsonHelper::getInteger(o2, QStringLiteral("refineStages"), d.s2d.refineStages)));
	c2.boxPx = std::max(32, jsonHelper::getInteger(o2, QStringLiteral("boxPx"), d.s2d.boxPx));
	c2.circleMinPx = jsonHelper::getInteger(o2, QStringLiteral("circleMinPx"), d.s2d.circleMinPx);
	c2.circleMaxPx = jsonHelper::getInteger(o2, QStringLiteral("circleMaxPx"), d.s2d.circleMaxPx);
	c2.confirmSafetyCheck = jsonHelper::getBool(o2, QStringLiteral("confirmSafetyCheck"), false);

	const QJsonObject o3 = o.value(QStringLiteral("3d")).toObject();
	auto& c3 = _zSearch.s3d;
	c3.xySet = jsonHelper::getBool(o3, QStringLiteral("xySet"), false);
	c3.x = jsonHelper::getDouble(o3, QStringLiteral("x"), 0.0);
	c3.y = jsonHelper::getDouble(o3, QStringLiteral("y"), 0.0);
	c3.startSet = jsonHelper::getBool(o3, QStringLiteral("startSet"), false);
	c3.startZ = jsonHelper::getDouble(o3, QStringLiteral("startZ"), 0.0);
	c3.stopSet = jsonHelper::getBool(o3, QStringLiteral("stopSet"), false);
	c3.stopZ = jsonHelper::getDouble(o3, QStringLiteral("stopZ"), 0.0);
	c3.stepMm = std::max(kMinCoarseStepMm, jsonHelper::getDouble(o3, QStringLiteral("stepMm"), d.s3d.stepMm));
	c3.targetMm = jsonHelper::getDouble(o3, QStringLiteral("targetMm"), d.s3d.targetMm);
	c3.maxSpreadMm = jsonHelper::getDouble(o3, QStringLiteral("maxSpreadMm"), d.s3d.maxSpreadMm);

	_zSearch.diffSet = jsonHelper::getBool(o, QStringLiteral("diffSet"), false);
	_zSearch.diffMm = jsonHelper::getDouble(o, QStringLiteral("diffMm"), 0.0);
	_zSearch.diffTolMm = jsonHelper::getDouble(o, QStringLiteral("diffTolMm"), d.diffTolMm);
}

void VisionApp::loadZCalibration()
{
	_zCalibration = ZCalibration();
	QJsonObject o;
	if (!QFile::exists(calibrationPath()) || !loadJson(calibrationPath(), o)) return; //not calibrated yet
	_zCalibration.set = jsonHelper::getBool(o, QStringLiteral("set"), false);
	_zCalibration.camLaserDzMm = jsonHelper::getDouble(o, QStringLiteral("camLaserDzMm"), 0.0);
	_zCalibration.when = jsonHelper::getString(o, QStringLiteral("when"));
}

bool VisionApp::saveZCalibration()
{
	QJsonObject o;
	o.insert(QStringLiteral("set"), _zCalibration.set);
	o.insert(QStringLiteral("camLaserDzMm"), _zCalibration.camLaserDzMm);
	o.insert(QStringLiteral("when"), _zCalibration.when);

	//all or nothing: a half-written calibration would place every 3D scan wrongly
	QSaveFile file(calibrationPath());
	if (!file.open(QIODevice::WriteOnly)) return false;
	file.write(QJsonDocument(o).toJson(QJsonDocument::Indented));
	return file.commit();
}

/*
* True when the job thread picks up a queued no-op within waitMs - i.e. it is not in the middle of
* a run. A Guided Test, Test Run or the motion test holds it for their whole length, including the
* pauses between moves where no axis is moving. Blocks the UI thread for at most waitMs; a no-op
* still queued behind a run just runs later and does nothing.
*/
bool VisionApp::jobThreadIdle(int waitMs)
{
	auto ran = std::make_shared<std::atomic<bool>>(false);
	QTimer::singleShot(0, &_jobThread, [ran]() { ran->store(true); });
	QElapsedTimer t;
	t.start();
	while (!ran->load() && t.elapsed() < waitMs) QThread::msleep(5);
	return ran->load();
}

/*
* A search drives the machine from the UI thread, as the guided teach does, so anything else that
* could also be driving it has to be ruled out first: an axis still in motion (a jog), and the job
* thread being busy (a run between two of its moves, when nothing is moving).
*/
bool VisionApp::zsRefuseIfBusy()
{
	if (_zsBusy) return true;
	if (_productionSetupLocked || isStartTimer) { showMsg("Not while a production run is in progress."); return true; }
	if (_motionTestRunning) { showMsg("The Motion Accuracy Test is using the machine."); return true; }

	auto& mc = MotionController::instance();
	if (!mc.is_init(_motionID)) { showMsg("Motion is not initialised."); return true; }
	if (!mc.motionEnabled()) { showMsg("Motion is disabled - home the machine or clear the active error."); return true; }
	for (int axis = (int)Axis::X; axis <= (int)Axis::Z; axis++) {
		if (!mc.move_done(_motionID, 0, axis)) {
			showMsg(QStringLiteral("Axis %1 is still moving - wait for the machine to stop (is a Guided Test running?).").arg(axisName(axis)));
			return true;
		}
	}
	if (!jobThreadIdle(300)) {
		showMsg("A run is using the machine (Test Run, Guided Test or a jog sequence) - wait for it to finish.");
		return true;
	}
	return false;
}

bool VisionApp::zsCheckTarget(int axis, double target, QString& why)
{
	double lo = 0.0, hi = 0.0;
	if (!MotionController::instance().limits_mm(_motionID, axis, lo, hi)) {
		why = QStringLiteral("the %1 soft limits are unknown (motion not loaded)").arg(axisName(axis));
		return false;
	}
	if (!(lo < hi)) {
		why = QStringLiteral("motion.json has no %1 soft limits").arg(axisName(axis));
		return false;
	}
	if (target < lo || target > hi) {
		why = QStringLiteral("%1 %2 is outside the %1 soft limits %3 .. %4").arg(axisName(axis)).arg(mm(target)).arg(mm(lo)).arg(mm(hi));
		return false;
	}
	return true;
}

/*
* Everything a search needs, checked before the first move - a search that discovers half way down
* that its Stop Z is out of travel has already moved for nothing. x, y are where the search runs:
* the camera point for 2D, the camera point the laser is put on for 3D.
*/
bool VisionApp::zsCheckSetup(bool threeD, double x, double y, QString& why)
{
	const bool xySet = threeD ? _zSearch.s3d.xySet : _zSearch.s2d.xySet;
	const bool startSet = threeD ? _zSearch.s3d.startSet : _zSearch.s2d.startSet;
	const bool stopSet = threeD ? _zSearch.s3d.stopSet : _zSearch.s2d.stopSet;
	const double startZ = threeD ? _zSearch.s3d.startZ : _zSearch.s2d.startZ;
	const double stopZ = threeD ? _zSearch.s3d.stopZ : _zSearch.s2d.stopZ;
	const double step = threeD ? _zSearch.s3d.stepMm : _zSearch.s2d.stepMm;
	const QString which = threeD ? QStringLiteral("3D") : QStringLiteral("2D");

	if (!xySet || !startSet || !stopSet) { why = QStringLiteral("set the %1 Search XY, Start Z and Stop Z first").arg(which); return false; }
	if (!(startZ < stopZ)) {
		why = QStringLiteral("%1 Start Z must be ABOVE Stop Z - the smaller number, since + Z is down").arg(which);
		return false;
	}
	if (step < kMinCoarseStepMm || step > stopZ - startZ) {
		why = QStringLiteral("the %1 step must be between %2 mm and the Start-Stop range").arg(which).arg(kMinCoarseStepMm);
		return false;
	}

	double lx = x, ly = y;
	if (threeD) {
		lx += _laserConfig.offset.wx;
		ly += _laserConfig.offset.wy;

		auto& pm = ProfilerManager::instance();
		if (!pm.isConnected(_profilerID)) { why = QStringLiteral("the 3D profiler is not connected"); return false; }
		if (_zsLiveProfileOn) { why = QStringLiteral("the live profile on the 3D Optics page is running - stop it first"); return false; }
		const double range = pm.liveZRangeMm(_profilerID);
		if (range > 0.0 && step > range / 4.0) {
			why = QStringLiteral("the 3D step %1 mm could step over a surface - keep it under %2 mm (a quarter of the profiler's %3 mm range)")
				.arg(mm(step)).arg(mm(range / 4.0)).arg(mm(range));
			return false;
		}
	}

	if (!zsCheckTarget((int)Axis::X, lx, why) || !zsCheckTarget((int)Axis::Y, ly, why)) {
		if (threeD) why += QStringLiteral(" (the laser point: Search XY + laser offset)");
		return false;
	}
	return zsCheckTarget((int)Axis::Z, startZ, why) && zsCheckTarget((int)Axis::Z, stopZ, why);
}

void VisionApp::zsPause(int ms)
{
	StopOnlyInput gate(ui.toolButton_stopRun, _zsBusyWindow);
	QElapsedTimer t;
	t.start();
	while (t.elapsed() < ms) {
		QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
		QThread::msleep(5);
	}
}

/*
* A move that reports. Every target is checked against the soft limits BEFORE anything is sent, a
* refused move stops whatever was already sent, and "done" means the card's feedback counter is on
* target - the same test JobThread::settleAxis applies, with up to two re-issues. The event loop
* keeps running (only the on-screen Stop among user input - see StopOnlyInput) so a Stop from the
* panel or the screen lands: VisionApp::stopRun stops the axes and sets _stopRun, which ends the wait.
* Several axes go out in the groups the Axis Move Sequence setting gives (AxisMoveSequence.h), each
* landed before the next is sent.
*/
bool VisionApp::zsMove(const QVector<QPair<int, double>>& targets, QString& why)
{
	for (const auto& t : targets)
		if (!zsCheckTarget(t.first, t.second, why)) return false;
	if (targets.size() < 2) return zsMoveGroup(targets, why);

	QVector<int> axes;
	bool zDown = false;
	for (const auto& t : targets) {
		axes.push_back(t.first);
		if (t.first != (int)Axis::Z) continue;
		const auto pz = MotionController::instance().get_position_mm(_motionID, 0, (int)Axis::Z);
		zDown = pz && t.second > *pz;
	}
	const auto groups = axisseq::groups(SystemData::instance()._axisMoveSequence.load(), axes, zDown, (int)Axis::X, (int)Axis::Y, (int)Axis::Z);
	for (const auto& g : groups) {
		QVector<QPair<int, double>> part;
		for (const auto& t : targets) if (g.contains(t.first)) part.push_back(t);
		if (!zsMoveGroup(part, why)) return false;
	}
	return true;
}

bool VisionApp::zsMoveGroup(const QVector<QPair<int, double>>& targets, QString& why)
{
	auto& mc = MotionController::instance();
	for (const auto& t : targets)
		if (!zsCheckTarget(t.first, t.second, why)) return false;

	auto stopAll = [&]() { for (const auto& t : targets) mc.stop_move(_motionID, t.first); };

	for (int attempt = 0; attempt <= kMaxCorrections; attempt++) {
		//a Stop that landed while the previous move was finishing must not let this one go out
		if (_stopRun) { why = QStringLiteral("stopped"); return false; }

		QVector<int> sent;
		for (const auto& t : targets) {
			if (attempt > 0) {
				const auto p = mc.get_position_mm(_motionID, 0, t.first);
				if (p && std::abs(*p - t.second) <= onTargetMm(t.first)) continue;
			}
			if (!mc.absolute_move(_motionID, t.first, t.second)) {
				why = mc.motionEnabled()
					? QStringLiteral("the motion card refused the %1 move to %2 (see the log)").arg(axisName(t.first)).arg(mm(t.second))
					: QStringLiteral("motion is disabled - home the machine or clear the active error");
				stopAll();
				return false;
			}
			sent.push_back(t.first);
		}
		if (sent.isEmpty()) return true;

		zsPause(kMotionReadDelayMs);
		QElapsedTimer timer;
		timer.start();
		while (true) {
			bool done = true;
			for (int axis : sent) if (!mc.move_done(_motionID, 0, axis)) { done = false; break; }
			if (done) break;
			if (_stopRun) { stopAll(); why = QStringLiteral("stopped"); return false; }
			if (timer.elapsed() > kMoveTimeoutMs) {
				stopAll();
				why = QStringLiteral("the move did not finish within %1 s").arg(kMoveTimeoutMs / 1000);
				return false;
			}
			zsPause(20);
		}

		//on target by the card's own feedback, given half a second to close the last fraction
		QElapsedTimer settle;
		settle.start();
		bool onTarget = false;
		QString off;
		while (!onTarget) {
			onTarget = true;
			off.clear();
			for (const auto& t : targets) {
				const auto p = mc.get_position_mm(_motionID, 0, t.first);
				if (!p) { why = QStringLiteral("could not read the %1 position").arg(axisName(t.first)); return false; }
				if (std::abs(*p - t.second) > onTargetMm(t.first)) {
					onTarget = false;
					off = QStringLiteral("%1 is at %2, %3 mm from %4").arg(axisName(t.first)).arg(mm(*p)).arg(mm(*p - t.second)).arg(mm(t.second));
				}
			}
			if (onTarget || settle.elapsed() >= kSettleMs) break;
			if (_stopRun) { why = QStringLiteral("stopped"); return false; }
			zsPause(20);
		}
		if (onTarget) {
			if (attempt > 0) ct::logger::warn("[ZSearch] On target after %d correction(s)", attempt);
			return true;
		}
		why = off;
		if (attempt < kMaxCorrections)
			ct::logger::warn("[ZSearch] Not on target (%s) - re-issuing (%d of %d)", qPrintable(off), attempt + 1, kMaxCorrections);
		else
			ct::logger::warn("[ZSearch] Not on target (%s) after %d corrections - giving up", qPrintable(off), kMaxCorrections);
	}

	why = QStringLiteral("%1 after %2 corrections").arg(why).arg(kMaxCorrections);
	return false;
}

//never slides sideways below clearZ (+ Z is down): lifts to it first when lower, then moves XY
bool VisionApp::zsMoveToXY(double x, double y, double clearZ, QString& why)
{
	const auto pz = MotionController::instance().get_position_mm(_motionID, 0, (int)Axis::Z);
	if (!pz) { why = QStringLiteral("could not read the Z position"); return false; }
	if (*pz > clearZ + kOnTargetZMm && !zsMove({ { (int)Axis::Z, clearZ } }, why)) return false;
	return zsMove({ { (int)Axis::X, x }, { (int)Axis::Y, y } }, why);
}

bool VisionApp::zsSample2D(double z, bool confirm, zsearch::FocusSample& out, QString& why)
{
	if (!zsMove({ { (int)Axis::Z, z } }, why)) return false;
	if (SystemData::instance()._snapDelay_ms > 0) zsPause(SystemData::instance()._snapDelay_ms);

	QImage img;
	if (!guidedSnapFov(img, 4000)) { why = QStringLiteral("no image from the camera at Z %1").arg(mm(z)); return false; }
	if (_stopRun) { why = QStringLiteral("stopped"); return false; }

	out.z = z;
	out.score = zsearch::focusScore(img, _zSearch.s2d.boxPx);
	out.confirmed = true;
	if (confirm) {
		SafetyCheckResult r;
		out.confirmed = runSafetyCheck(img, r);
	}

	ct::logger::info("[ZSearch] 2D z=%.3f score=%.3f%s", z, out.score,
		confirm ? (out.confirmed ? " safety=PASS" : " safety=fail") : "");
	_zsw.status2->setText(QStringLiteral("Z %1  sharpness %2%3").arg(mm(z)).arg(out.score, 0, 'f', 2)
		.arg(confirm ? (out.confirmed ? QStringLiteral("  Safety Check PASS") : QStringLiteral("  Safety Check fail")) : QString()));
	return true;
}

/*
* Coarse sweep down from Start Z until the sharpness has clearly peaked (or Stop Z). Then up to
* Refine stages zoom stages: each goes back over +/- one step of the stage before at a fifth of that
* step, fits a parabola through the top of the hill, and is the last one once it changes nothing
* worth moving for. Every frame is approached from above, and so is the final move onto the result.
*/
bool VisionApp::zsFind2D(double x, double y, bool confirm, double& foundZ, QString& why)
{
	const auto& c = _zSearch.s2d;
	if (!zsMoveToXY(x, y, c.startZ, why)) return false;

	ct::logger::info("[ZSearch] 2D search at %.3f, %.3f: Z %.3f -> %.3f, step %.3f, up to %d refine stage(s), box %d px%s",
		x, y, c.startZ, c.stopZ, c.stepMm, c.refineStages, c.boxPx, confirm ? ", Safety Check confirms" : "");

	QVector<zsearch::FocusSample> coarse;
	bool passed = false;
	for (int i = 0; ; i++) {
		const double z = c.startZ + i * c.stepMm;
		if (z > c.stopZ + 1e-6) break;
		zsearch::FocusSample s;
		if (!zsSample2D(z, confirm, s, why)) return false;
		coarse.push_back(s);
		const int best = zsearch::bestConfirmed(coarse);
		if (best >= 0 && zsearch::peakPassed(coarse, best, kDropRatio)) { passed = true; break; }
	}

	const int best = zsearch::bestConfirmed(coarse);
	if (best < 0) {
		why = confirm ? QStringLiteral("the Safety Check did not pass at any Z between Start Z and Stop Z")
			: QStringLiteral("no frame was scored");
		return false;
	}
	double dullest = coarse[0].score;
	for (const auto& s : coarse) dullest = std::min(dullest, s.score);
	if (best == 0) {
		why = QStringLiteral("the sharpest frame is at Start Z - the focus may be above it: raise Start Z (a smaller Z)");
		return false;
	}
	if (!passed && best == coarse.size() - 1) {
		why = QStringLiteral("the sharpest frame is at Stop Z - the focus may be below it: lower Stop Z, if the machine clears it");
		return false;
	}
	if (coarse[best].score < kMinPeakRatio * dullest) {
		why = QStringLiteral("no clear focus peak (sharpest %1, dullest %2) - check the light, the box size and that the Search XY is on the screw head")
			.arg(coarse[best].score, 0, 'f', 2).arg(dullest, 0, 'f', 2);
		return false;
	}

	//zoom stages: every Z below stays inside the stage before's window, so inside Start Z .. Stop Z
	double centre = coarse[best].z;
	double lastStep = c.stepMm;
	int frames = coarse.size();
	QString stopReason = QStringLiteral("all %1 stage(s) done").arg(c.refineStages);
	for (int stage = 1; stage <= c.refineStages; stage++) {
		if (lastStep <= kMinFineStepMm + 1e-9) {
			stopReason = QStringLiteral("the step is at its %1 mm floor").arg(kMinFineStepMm);
			break;
		}
		const double step = std::max(kMinFineStepMm, lastStep / 5.0);
		const double zA = std::max(c.startZ, centre - lastStep);
		const double zB = std::min(c.stopZ, centre + lastStep);

		//arrive at the window's first Z from above, like every frame after it
		if (!zsMove({ { (int)Axis::Z, std::max(c.startZ, zA - step) } }, why)) return false;
		QVector<zsearch::FocusSample> samples;
		for (int i = 0; ; i++) {
			const double z = zA + i * step;
			if (z > zB + 1e-6) break;
			zsearch::FocusSample s;
			if (!zsSample2D(z, confirm, s, why)) return false;
			samples.push_back(s);
		}
		frames += samples.size();

		const int sb = zsearch::bestConfirmed(samples);
		if (sb < 0) {
			why = QStringLiteral("the Safety Check did not pass near the focus peak (refine stage %1)").arg(stage);
			return false;
		}

		double hi = samples[0].score, lo = hi;
		for (const auto& s : samples) { hi = std::max(hi, s.score); lo = std::min(lo, s.score); }
		const double spread = hi > 0.0 ? (hi - lo) / hi : 0.0;
		if (spread < kFlatRatio) {
			ct::logger::info("[ZSearch] 2D stage %d: step %.3f over %.3f..%.3f, %d frames all within %.1f%% - flat, keeping %.3f",
				stage, step, zA, zB, samples.size(), 100.0 * spread, centre);
			stopReason = QStringLiteral("stage %1 was flat - only noise left at that scale").arg(stage);
			break;
		}

		int used = 0;
		const double estimate = zsearch::fitPeak(samples, sb, kFitFraction, &used);
		const double moved = estimate - centre;
		ct::logger::info("[ZSearch] 2D stage %d: step %.3f over %.3f..%.3f, %d frames, peak %.3f fitted through %d (moved %+.3f)",
			stage, step, zA, zB, samples.size(), estimate, used, moved);
		centre = estimate;
		lastStep = step;
		if (std::abs(moved) < kConvergedMm) {
			stopReason = QStringLiteral("stage %1 moved the peak under %2 mm - converged").arg(stage).arg(kConvergedMm);
			break;
		}
	}
	ct::logger::info("[ZSearch] 2D refine stopped: %s", qPrintable(stopReason));

	//onto the result from above, the same way every frame was approached
	if (!zsMove({ { (int)Axis::Z, std::max(c.startZ, centre - lastStep) } }, why)) return false;
	zsearch::FocusSample last;
	if (!zsSample2D(centre, confirm, last, why)) return false;
	if (confirm && !last.confirmed) { why = QStringLiteral("the Safety Check failed at the focus Z %1").arg(mm(centre)); return false; }

	ct::logger::info("[ZSearch] 2D Focus Z %.3f (sharpness %.3f, %d frames)", centre, last.score, frames + 1);
	foundZ = centre;
	return true;
}

//five live profiles at the current position, middle third of the line only
bool VisionApp::zsReadProfile(zsearch::ProfileStats& st, int& offered, QString& why)
{
	auto& pm = ProfilerManager::instance();
	std::vector<double> values;
	offered = 0;
	for (int i = 0; i < kProfileShots; i++) {
		if (pm.snapShot(_profilerID)) {
			if (const auto* f = pm.getFrame(_profilerID)) {
				const auto& p = f->profiles;
				const size_t keep = (size_t)(p.size() * kProfileCentreFraction);
				const size_t from = (p.size() - keep) / 2;
				values.insert(values.end(), p.begin() + from, p.begin() + from + keep);
				offered += (int)keep;
			}
		}
		zsPause(60);
		if (_stopRun) { why = QStringLiteral("stopped"); return false; }
	}
	st = zsearch::profileStats(values);
	return true;
}

/*
* Step down until the profiler reads a flat surface, measure how the reading moves with Z (sign and
* scale - neither is assumed), then move until the reading is the target. The controller is put in
* live mode for the reads and ALWAYS put back in encoder batch mode, which production scans need.
*/
bool VisionApp::zsFind3D(double camX, double camY, double& foundZ, QString& why)
{
	const auto& c = _zSearch.s3d;
	auto& pm = ProfilerManager::instance();

	const double lx = camX + _laserConfig.offset.wx;
	const double ly = camY + _laserConfig.offset.wy;
	if (!zsMoveToXY(lx, ly, c.startZ, why)) return false;

	ct::logger::info("[ZSearch] 3D search, laser at %.3f, %.3f (camera %.3f, %.3f): Z %.3f -> %.3f, step %.3f, target %.3f, max spread %.3f",
		lx, ly, camX, camY, c.startZ, c.stopZ, c.stepMm, c.targetMm, c.maxSpreadMm);

	pm.stop(_profilerID); //live reads need scan acquisition stopped
	if (!pm.startLive(_profilerID)) { why = QStringLiteral("the profiler could not enter live mode"); return false; }

	auto search = [&]() -> bool {
		auto readAt = [&](double z, zsearch::ProfileStats& st, bool& valid) {
			if (!zsMove({ { (int)Axis::Z, z } }, why)) return false;
			int offered = 0;
			if (!zsReadProfile(st, offered, why)) return false;
			valid = st.valid >= std::max(kMinProfilePoints, offered / 2) && st.spread <= c.maxSpreadMm;
			ct::logger::info("[ZSearch] 3D z=%.3f points=%d/%d reading=%.4f spread=%.4f %s",
				z, st.valid, offered, st.median, st.spread, valid ? "FLAT" : "-");
			_zsw.status3->setText(st.valid > 0
				? QStringLiteral("Z %1  reading %2 mm  spread %3 mm  (%4 of %5 points)%6").arg(mm(z)).arg(st.median, 0, 'f', 3)
					.arg(st.spread, 0, 'f', 3).arg(st.valid).arg(offered).arg(valid ? QString() : QStringLiteral("  - not flat enough"))
				: QStringLiteral("Z %1  nothing in range").arg(mm(z)));
			return true;
		};

		double z0 = 0.0;
		zsearch::ProfileStats s0;
		bool valid = false, found = false;

		//first where the 2D result says it should be, when this recipe's difference is taught
		if (_zFound.has2D && _zSearch.diffSet) {
			const double predicted = _zFound.z2D - _zSearch.diffMm;
			if (predicted >= c.startZ && predicted <= c.stopZ) {
				if (!readAt(predicted, s0, valid)) return false;
				if (valid) { z0 = predicted; found = true; }
				ct::logger::info("[ZSearch] 3D predicted from 2D Focus Z: %.3f - %s", predicted, valid ? "surface there" : "nothing, stepping from Start Z");
			}
		}
		for (int i = 0; !found; i++) {
			const double z = c.startZ + i * c.stepMm;
			if (z > c.stopZ + 1e-6) break;
			if (!readAt(z, s0, valid)) return false;
			if (valid) { z0 = z; found = true; }
		}
		if (!found) {
			why = QStringLiteral("no flat surface read between Start Z and Stop Z - check that the laser lands on the flat area, and Max spread");
			return false;
		}

		//the reading's slope against machine Z, measured half a millimetre away
		double dz = kSlopeStepMm;
		if (z0 + dz > c.stopZ) dz = -kSlopeStepMm;
		if (z0 + dz < c.startZ) { why = QStringLiteral("the Start-Stop range is too short to measure the slope"); return false; }
		zsearch::ProfileStats s1;
		if (!readAt(z0 + dz, s1, valid)) return false;
		if (!valid) { why = QStringLiteral("the surface was lost %1 mm away - is it really flat there?").arg(mm(dz)); return false; }
		const double slope = (s1.median - s0.median) / dz;
		ct::logger::info("[ZSearch] 3D slope %.4f mm of reading per mm of Z", slope);
		if (std::abs(slope) < 0.5 || std::abs(slope) > 2.0) {
			why = QStringLiteral("the reading moved %1 mm per mm of Z (about 1 expected) - not one flat surface under the laser?").arg(slope, 0, 'f', 3);
			return false;
		}

		double zc = z0 + (c.targetMm - s0.median) / slope;
		for (int i = 0; i < kCentreMoves; i++) {
			if (zc < c.startZ || zc > c.stopZ) {
				why = QStringLiteral("centring needs Z %1, outside Start Z .. Stop Z").arg(mm(zc));
				return false;
			}
			zsearch::ProfileStats st;
			if (!readAt(zc, st, valid)) return false;
			if (!valid) { why = QStringLiteral("the surface was lost while centring at Z %1").arg(mm(zc)); return false; }
			const double err = st.median - c.targetMm;
			if (std::abs(err) <= kCentreTolMm) {
				ct::logger::info("[ZSearch] 3D Centre Z %.3f (reading %.4f, target %.4f)", zc, st.median, c.targetMm);
				foundZ = zc;
				return true;
			}
			zc -= err / slope;
		}
		why = QStringLiteral("the reading did not settle within %1 mm of the target after %2 moves").arg(kCentreTolMm).arg(kCentreMoves);
		return false;
	};

	bool ok = search();

	if (!pm.stopLive(_profilerID)) {
		ct::logger::error("[ZSearch] FAILED to restore the profiler's encoder batch mode after the 3D search");
		why = (ok ? QString() : why + QStringLiteral("\n\n")) + QStringLiteral(
			"The profiler did NOT go back to encoder batch mode - press Start/Stop Live on the 3D Optics page before scanning.");
		ok = false;
	}
	return ok;
}

/*
* Find Z for one frame. On failure Z goes straight back up to Start Z and nothing else moves; a Stop
* from the panel moves nothing at all.
*/
void VisionApp::zsRunFind(bool threeD)
{
	if (zsRefuseIfBusy()) return;

	const double x = threeD ? _zSearch.s3d.x : _zSearch.s2d.x;
	const double y = threeD ? _zSearch.s3d.y : _zSearch.s2d.y;
	const double startZ = threeD ? _zSearch.s3d.startZ : _zSearch.s2d.startZ;
	const QString which = threeD ? QStringLiteral("3D") : QStringLiteral("2D");
	QLabel* statusLabel = threeD ? _zsw.status3 : _zsw.status2;

	QString why;
	if (!zsCheckSetup(threeD, x, y, why)) { showMsg(QStringLiteral("%1 Find Z cannot start: %2.").arg(which).arg(why)); return; }

	_zsBusy = true;
	_stopRun = false;
	_zsw.frame2D->setEnabled(false);
	_zsw.frame3D->setEnabled(false);
	statusLabel->setText(QStringLiteral("Searching..."));
	_zsBusyWindow = showBusyWindow(threeD ? QStringLiteral("Finding 3D Centre Z") : QStringLiteral("Finding 2D Focus Z"), QStringLiteral(
		"The machine is searching for Z. Everything else is locked until it finishes.\n"
		"Stop ends it at once; the panel Stop and the e-stop also stop it."));
	if (auto* progress = _zsBusyWindow->findChild<QLabel*>(QStringLiteral("busyProgress")))
		progress->setText(QStringLiteral("Searching Z %1 to %2 at X %3, Y %4...").arg(mm(startZ))
			.arg(mm(threeD ? _zSearch.s3d.stopZ : _zSearch.s2d.stopZ)).arg(mm(x)).arg(mm(y)));

	double z = 0.0;
	const int trolleyEpoch = _zsTrolleyEpoch;
	bool ok = threeD ? zsFind3D(x, y, z, why) : zsFind2D(x, y, _zSearch.s2d.confirmSafetyCheck, z, why);
	const bool stopped = _stopRun;
	//the trolley guard changed while it searched: the Z it found belongs to a table that is gone
	if (ok && trolleyEpoch != _zsTrolleyEpoch) {
		ok = false;
		why = QStringLiteral("the trolley was removed or attached during the search - the result was discarded");
	}

	QString after;
	if (ok) {
		if (threeD) { _zFound.has3D = true; _zFound.z3D = z; }
		else { _zFound.has2D = true; _zFound.z2D = z; }
		statusLabel->setText(QStringLiteral("Found %1 Z %2").arg(which).arg(mm(z)));
		AuditLog::instance().log(threeD ? QStringLiteral("ZSEARCH_FIND_3D") : QStringLiteral("ZSEARCH_FIND_2D"), QStringLiteral("z=%1").arg(mm(z)));
	}
	else {
		if (threeD) _zFound.has3D = false; else _zFound.has2D = false;
		if (stopped) after = QStringLiteral("Stopped - nothing else was moved.");
		else {
			QString w;
			after = zsMove({ { (int)Axis::Z, startZ } }, w)
				? QStringLiteral("Z went back up to Start Z.")
				: QStringLiteral("Z could NOT go back up to Start Z: %1").arg(w);
		}
		statusLabel->setText(QStringLiteral("Not found: %1").arg(why));
		ct::logger::warn("[ZSearch] %s Find Z failed: %s. %s", qPrintable(which), qPrintable(why), qPrintable(after));
		AuditLog::instance().log(threeD ? QStringLiteral("ZSEARCH_FIND_3D") : QStringLiteral("ZSEARCH_FIND_2D"), QStringLiteral("FAILED: %1").arg(why));
	}

	closeBusyWindow(_zsBusyWindow);
	//anything still queued from the search lands on the still-disabled frames, not as a new command
	QCoreApplication::processEvents();
	_zsw.frame2D->setEnabled(true);
	_zsw.frame3D->setEnabled(true);
	_zsBusy = false;
	refreshZSearchUi();

	if (!ok) showMsg(QStringLiteral("%1 Find Z failed.\n\n%2.\n\n%3").arg(which).arg(why).arg(after));
}

/*
* Machine-wide: with the camera and the laser on the SAME flat surface, the difference between the
* two found Zs is how the two are mounted. Run at the current XY, which the engineer has parked over
* a flat surface with enough texture for the camera to focus on.
*/
void VisionApp::zsCalibrateCamLaser()
{
	if (zsRefuseIfBusy()) return;
	if (notAllowToAccess(AccessLevel::OPERATOR)) { showMsg("Calibrating the camera-to-laser Z needs an engineer or admin login."); return; }

	double x = 0.0, y = 0.0;
	{
		auto& mc = MotionController::instance();
		const auto px = mc.get_position_mm(_motionID, 0, (int)Axis::X);
		const auto py = mc.get_position_mm(_motionID, 0, (int)Axis::Y);
		if (!px || !py) { showMsg("Failed to read the current position. Is motion connected?"); return; }
		x = *px; y = *py;
	}

	QString why;
	if (!zsCheckSetup(false, x, y, why) || !zsCheckSetup(true, x, y, why)) {
		showMsg(QStringLiteral("Calibration cannot start: %1.\n\nIt uses the Start Z, Stop Z and step of both frames.").arg(why));
		return;
	}
	if (QMessageBox::question(this, QStringLiteral("Calibrate camera-to-laser Z"),
		QStringLiteral("The camera must be over a FLAT surface with some texture, here at %1, %2.\n\n"
			"The 2D search runs here, then the 3D search puts the laser on the same spot. "
			"The result is used for every recipe on this machine.\n\nContinue?").arg(mm(x)).arg(mm(y)))
		!= QMessageBox::Yes) return;

	_zsBusy = true;
	_stopRun = false;
	_zsw.frame2D->setEnabled(false);
	_zsw.frame3D->setEnabled(false);
	_zsBusyWindow = showBusyWindow(QStringLiteral("Calibrating camera-to-laser Z"), QStringLiteral(
		"The machine runs the 2D search, then the 3D search on the same spot. Everything else is locked\n"
		"until it finishes. Stop ends it at once; the panel Stop and the e-stop also stop it."));

	double z2 = 0.0, z3 = 0.0;
	const bool ok = zsFind2D(x, y, false, z2, why) && zsFind3D(x, y, z3, why);
	QString after;
	if (ok) {
		_zCalibration.set = true;
		_zCalibration.camLaserDzMm = z2 - z3;
		_zCalibration.when = QDateTime::currentDateTime().toString(QStringLiteral("yyyy-MM-dd hh:mm"));
		if (!saveZCalibration()) after = QStringLiteral("\n\nBut it could NOT be saved to %1.").arg(calibrationPath());
		ct::logger::info("[ZSearch] Camera-to-laser Z calibrated: 2D %.3f - 3D %.3f = %.3f mm", z2, z3, z2 - z3);
		AuditLog::instance().log(QStringLiteral("ZSEARCH_CALIBRATE"),
			QStringLiteral("2D %1 - 3D %2 = %3 mm").arg(mm(z2)).arg(mm(z3)).arg(mm(z2 - z3)));
	}
	else if (!_stopRun) {
		//back above both searches: the smaller Start Z is the higher one
		QString w;
		const double clearZ = std::min(_zSearch.s2d.startZ, _zSearch.s3d.startZ);
		after = zsMove({ { (int)Axis::Z, clearZ } }, w)
			? QStringLiteral("\n\nZ went back up to Start Z.")
			: QStringLiteral("\n\nZ could NOT go back up to Start Z: %1").arg(w);
		AuditLog::instance().log(QStringLiteral("ZSEARCH_CALIBRATE"), QStringLiteral("FAILED: %1").arg(why));
	}

	closeBusyWindow(_zsBusyWindow);
	QCoreApplication::processEvents();
	_zsw.frame2D->setEnabled(true);
	_zsw.frame3D->setEnabled(true);
	_zsBusy = false;
	refreshZSearchUi();

	showMsg(ok
		? QStringLiteral("Camera-to-laser Z: %1 mm (2D Focus Z %2, 3D Centre Z %3).%4").arg(mm(z2 - z3)).arg(mm(z2)).arg(mm(z3)).arg(after)
		: QStringLiteral("Calibration failed.\n\n%1.%2").arg(why).arg(after));
}

void VisionApp::zsSetDifference()
{
	if (_zsBusy) return;
	if (!_zFound.has2D || !_zFound.has3D) {
		showMsg("Run Find Z for both 2D and 3D on this trolley first.");
		return;
	}
	_zSearch.diffSet = true;
	_zSearch.diffMm = _zFound.z2D - _zFound.z3D;
	saveRecipeConfig();
	refreshZSearchUi();
	AuditLog::instance().log(QStringLiteral("ZSEARCH_SET_DIFF"),
		QStringLiteral("2D %1 - 3D %2 = %3 mm").arg(mm(_zFound.z2D)).arg(mm(_zFound.z3D)).arg(mm(_zSearch.diffMm)));
}

void VisionApp::zsCopyFromRecipe()
{
	if (_zsBusy) return;

	const QString root = QStringLiteral("%1recipe/").arg(Common::Directory::LocalPath);
	QStringList names;
	for (const QString& name : QDir(root).entryList(QDir::Dirs | QDir::NoDotAndDotDot, QDir::Name)) {
		if (name == Common::Directory::CurrentRecipe) continue;
		if (QFile::exists(root + name + QStringLiteral("/recipeConfig.json"))) names << name;
	}
	if (names.isEmpty()) { showMsg("No other recipe to copy from."); return; }

	bool picked = false;
	const QString name = QInputDialog::getItem(this, QStringLiteral("Copy Z search"),
		QStringLiteral("Copy the Z search settings from:"), names, 0, false, &picked);
	if (!picked || name.isEmpty()) return;

	QJsonObject other;
	if (!loadJson(root + name + QStringLiteral("/recipeConfig.json"), other) || !other.contains(QStringLiteral("zSearch"))) {
		showMsg(QStringLiteral("%1 has no Z search set up.").arg(name));
		return;
	}
	if (QMessageBox::question(this, QStringLiteral("Copy Z search"),
		QStringLiteral("Replace this recipe's Z search settings (both frames and the taught 2D - 3D difference) with %1's?").arg(name))
		!= QMessageBox::Yes) return;

	zSearchFromJson(other.value(QStringLiteral("zSearch")).toObject());
	_zFound = ZFound(); //found at the old settings' points
	saveRecipeConfig();
	refreshZSearchUi();
	AuditLog::instance().log(QStringLiteral("ZSEARCH_COPY"), QStringLiteral("from %1").arg(name));
}
