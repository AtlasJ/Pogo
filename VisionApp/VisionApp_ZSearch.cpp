// =============================================================================
//  VisionApp_ZSearch.cpp
//  The Z search helpers the Guided Setup runs: checked moves, the 2D focus
//  sweep (fid0's 2D Z) and the one-read 3D Z. The old Z search frames
//  (Find 2D / 3D Z, camera-to-laser, difference) went at the cutover, step 6.
// =============================================================================

#include "VisionApp.h"
#include "AuditLog.h"
#include "3DOpticsTab.h"
#include "AxisMoveSequence.h"
#include "GantryLog.h"

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
	//Guided Setup (new) accepts X / Y this far off once the settle is over: it reads where the camera
	//really stopped before it measures anything, and on 3 Nov 2026 the IPC's X parked 0.063-0.064 mm off
	//where re-issuing never moved it - a failed Teach or Find for nothing. Still warned in the log.
	//0.1 -> 0.3 on 4 Nov 2026: X parked 0.116-0.155 mm off and every Teach XY / Find failed (Jimmy's call)
	constexpr double kOnTargetXYGs2Mm = 0.3;
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
	//NaN fails both comparisons below, so it is refused on its own
	if (!std::isfinite(target) || target < lo || target > hi) {
		why = QStringLiteral("%1 %2 is outside the %1 soft limits %3 .. %4").arg(axisName(axis)).arg(mm(target)).arg(mm(lo)).arg(mm(hi));
		return false;
	}
	return true;
}

bool VisionApp::zsCheck3DRead(double camX, double camY, QString& why)
{
	auto& pm = ProfilerManager::instance();
	if (!pm.isConnected(_profilerID)) { why = QStringLiteral("the 3D profiler is not connected"); return false; }
	if (_zsLiveProfileOn) { why = QStringLiteral("the live profile on the 3D Optics page is running - stop it first"); return false; }
	if (!zsCheckTarget((int)Axis::X, camX + _laserConfig.offset.wx, why) || !zsCheckTarget((int)Axis::Y, camY + _laserConfig.offset.wy, why)) {
		why += QStringLiteral(" (the laser point: camera point + laser offset)");
		return false;
	}
	return true;
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

		//X / Y aimed past by the card's gap to its own encoder, so the ENCODER lands on target (aim_for_encoder) -
		//every axis read BEFORE any is sent: once one moves, the other's gap is disturbed
		QVector<QPair<int, double>> sends;
		for (const auto& t : targets) {
			if (attempt > 0) {
				const auto p = mc.get_position_mm(_motionID, 0, t.first);
				if (p && std::abs(*p - t.second) <= onTargetMm(t.first)) continue;
			}
			double send = mc.aim_for_encoder(_motionID, t.first, t.second);
			QString aimWhy;
			if (send != t.second && !zsCheckTarget(t.first, send, aimWhy)) send = t.second;   //the plain target was checked above
			sends.push_back({ t.first, send });
		}
		QVector<int> sent;
		for (const auto& s : sends) {
			if (!mc.absolute_move(_motionID, s.first, s.second)) {
				why = mc.motionEnabled()
					? QStringLiteral("the motion card refused the %1 move to %2 (see the log)").arg(axisName(s.first)).arg(mm(s.second))
					: QStringLiteral("motion is disabled - home the machine or clear the active error");
				stopAll();
				return false;
			}
			sent.push_back(s.first);
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
		bool onTarget = false, withinGs2 = false;
		QString off;
		while (!onTarget) {
			onTarget = withinGs2 = true;
			off.clear();
			for (const auto& t : targets) {
				const auto p = mc.get_position_mm(_motionID, 0, t.first);
				if (!p) { why = QStringLiteral("could not read the %1 position").arg(axisName(t.first)); return false; }
				const double miss = std::abs(*p - t.second);
				if (miss > onTargetMm(t.first)) {
					onTarget = false;
					off = QStringLiteral("%1 is at %2, %3 mm from %4").arg(axisName(t.first)).arg(mm(*p)).arg(mm(*p - t.second)).arg(mm(t.second));
					if (t.first == (int)Axis::Z || miss > kOnTargetXYGs2Mm) withinGs2 = false;
				}
			}
			if (onTarget || settle.elapsed() >= kSettleMs) break;
			if (_stopRun) { why = QStringLiteral("stopped"); return false; }
			zsPause(20);
		}
		//the gantry log: every axis of the group, where it settled against its target
		QStringList at;
		for (const auto& t : targets) {
			const auto p = mc.get_position_mm(_motionID, 0, t.first);
			at << (p ? QStringLiteral("%1 %2 at %3 (%4)").arg(QLatin1String(gantrylog::axisName(t.first)))
				.arg(t.second, 0, 'f', 4).arg(*p, 0, 'f', 4).arg(*p - t.second, 0, 'f', 4) : QStringLiteral("%1 unreadable").arg(QLatin1String(gantrylog::axisName(t.first))));
		}
		if (onTarget) {
			if (attempt > 0) ct::logger::warn("[ZSearch] On target after %d correction(s)", attempt);
			gantrylog::write("SETTLE %s: on target (XY %.2f, Z %.2f) after %d correction(s), %lld ms", qPrintable(at.join(QStringLiteral(", "))),
				kOnTargetXYMm, kOnTargetZMm, attempt, (long long)settle.elapsed());
			return true;
		}
		//the corrections first - each aims past by the gap again; only what they could not close is accepted
		if (_zsLooseXYLanding && withinGs2 && attempt == kMaxCorrections) {
			ct::logger::warn("[ZSearch] Landed past %.2f mm (%s) - within Guided Setup's %.2f mm, accepted",
				kOnTargetXYMm, qPrintable(off), kOnTargetXYGs2Mm);
			gantrylog::write("SETTLE %s: past %.2f but within Guided Setup's %.2f - ACCEPTED", qPrintable(at.join(QStringLiteral(", "))),
				kOnTargetXYMm, kOnTargetXYGs2Mm);
			return true;
		}
		why = off;
		if (attempt < kMaxCorrections) {
			ct::logger::warn("[ZSearch] Not on target (%s) - re-issuing (%d of %d)", qPrintable(off), attempt + 1, kMaxCorrections);
			gantrylog::write("SETTLE %s: NOT on target after %d ms - re-issuing (%d of %d)", qPrintable(at.join(QStringLiteral(", "))),
				kSettleMs, attempt + 1, kMaxCorrections);
		}
		else {
			ct::logger::warn("[ZSearch] Not on target (%s) after %d corrections - giving up", qPrintable(off), kMaxCorrections);
			gantrylog::write("SETTLE %s: GIVEN UP after %d corrections", qPrintable(at.join(QStringLiteral(", "))), kMaxCorrections);
		}
	}

	why = QStringLiteral("%1 after %2 corrections").arg(why).arg(kMaxCorrections);
	return false;
}

//never slides sideways below clearZ (+ Z is down): lifts to it first when lower, then moves XY
/*
* The host's load-recipe arrives as a queued signal, so it can land inside any event pump of a Z
* search or Guided Setup action. Opening a recipe then would swap the page under the action (its
* result written into the other recipe) and the unsaved-changes question would come up while every
* click outside the busy window is eaten - a box nobody can answer. So it waits for the action.
*/
void VisionApp::openRecipeWhenIdle(const QString& name, bool create, bool waited)
{
	if (_zsBusy) {
		if (!waited) ct::logger::warn("[Recipe] Host asked for recipe %s while a Z search / Guided Setup action drives the machine - it opens when that ends", qPrintable(name));
		QTimer::singleShot(500, this, [=]() { openRecipeWhenIdle(name, create, true); });
		return;
	}
	if (waited) ct::logger::info("[Recipe] Opening recipe %s, held back by the machine action", qPrintable(name));
	if (create) createRecipe(name);
	else openRecipe(name);
}

/*
* A failure can come before any Z went down - an XY move that did not land, a missing scale - with
* the head ABOVE the Start Z it was taught at another XY. "Back to Start Z" must not lower it there.
*/
QString VisionApp::zsLiftAfterFailure(double z, const QString& name)
{
	const auto pz = MotionController::instance().get_position_mm(_motionID, 0, (int)Axis::Z);
	if (!pz) return QStringLiteral("Z was left where it is: its position could not be read");
	if (*pz <= z + kOnTargetZMm) return QStringLiteral("Z stayed at %1 (already at or above %2)").arg(mm(*pz), name);
	QString w;
	return zsMove({ { (int)Axis::Z, z } }, w)
		? QStringLiteral("Z went back up to %1").arg(name)
		: QStringLiteral("Z could NOT go back up to %1: %2").arg(name, w);
}

bool VisionApp::zsMoveToXY(double x, double y, double clearZ, QString& why)
{
	const auto pz = MotionController::instance().get_position_mm(_motionID, 0, (int)Axis::Z);
	if (!pz) { why = QStringLiteral("could not read the Z position"); return false; }
	if (*pz > clearZ + kOnTargetZMm && !zsMove({ { (int)Axis::Z, clearZ } }, why)) return false;
	return zsMove({ { (int)Axis::X, x }, { (int)Axis::Y, y } }, why);
}

bool VisionApp::zsSample2D(double z, int boxPx, bool confirm, zsearch::FocusSample& out, QString& why, QLabel* status)
{
	if (!zsMove({ { (int)Axis::Z, z } }, why)) return false;
	if (SystemData::instance()._snapDelay_ms > 0) zsPause(SystemData::instance()._snapDelay_ms);

	QImage img;
	if (!guidedSnapFov(img, 4000)) { why = QStringLiteral("no image from the camera at Z %1").arg(mm(z)); return false; }
	if (_stopRun) { why = QStringLiteral("stopped"); return false; }

	out.z = z;
	out.score = zsearch::focusScore(img, boxPx);
	out.confirmed = true;
	if (confirm) {
		SafetyCheckResult r;
		out.confirmed = runSafetyCheck(img, r);
	}

	ct::logger::info("[ZSearch] 2D z=%.3f score=%.3f%s", z, out.score,
		confirm ? (out.confirmed ? " safety=PASS" : " safety=fail") : "");
	if (status) status->setText(QStringLiteral("Z %1  sharpness %2%3").arg(mm(z)).arg(out.score, 0, 'f', 2)
		.arg(confirm ? (out.confirmed ? QStringLiteral("  Safety Check PASS") : QStringLiteral("  Safety Check fail")) : QString()));
	return true;
}

/*
* Coarse sweep down from Start Z until the sharpness has clearly peaked (or Stop Z). Then up to
* Refine stages zoom stages: each goes back over +/- one step of the stage before at a fifth of that
* step, fits a parabola through the top of the hill, and is the last one once it changes nothing
* worth moving for. Every frame is approached from above, and so is the final move onto the result.
*/
bool VisionApp::zsFind2D(const ZSearch2DConfig& c, double x, double y, bool confirm, double& foundZ, QString& why, QLabel* status)
{
	//a step of 0 (or NaN) would sample the same Z for ever
	if (!(c.stepMm > 0.0)) { why = QStringLiteral("the 2D step must be above 0 mm"); return false; }
	if (!zsMoveToXY(x, y, c.startZ, why)) return false;

	ct::logger::info("[ZSearch] 2D search at %.3f, %.3f: Z %.3f -> %.3f, %s %.3f, up to %d refine stage(s), box %d px%s",
		x, y, c.startZ, c.stopZ, c.adaptiveStep ? "adaptive step up to" : "step", c.stepMm, c.refineStages, c.boxPx,
		confirm ? ", Safety Check confirms" : "");

	QVector<zsearch::FocusSample> coarse;
	bool passed = false;
	double z = c.startZ, step = c.stepMm;
	for (int i = 0; ; i++) {
		if (!c.adaptiveStep) z = c.startZ + i * c.stepMm;
		if (z > c.stopZ + 1e-6) break;
		zsearch::FocusSample s;
		if (!zsSample2D(z, c.boxPx, confirm, s, why, status)) return false;
		coarse.push_back(s);
		const int best = zsearch::bestConfirmed(coarse);
		if (best >= 0 && zsearch::peakPassed(coarse, best, kDropRatio)) { passed = true; break; }
		if (c.adaptiveStep) {
			step = zsearch::nextAdaptiveStep(coarse, c.stepMm, step);
			//the last step lands ON Stop Z rather than past it, so the bottom of the range is looked at
			z = z < c.stopZ - 1e-6 ? std::min(z + step, c.stopZ) : z + step;
		}
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
	if (c.adaptiveStep) {
		//the peak lies between the sharpest frame's neighbours (both exist: it is neither the first
		//frame nor the last one, see above), so the first window spans the wider of the two gaps
		lastStep = std::max(coarse[best].z - coarse[best - 1].z, coarse[best + 1].z - coarse[best].z);
		ct::logger::info("[ZSearch] 2D coarse: %d frames, sharpest %.3f at Z %.3f, neighbours %.3f / %.3f mm away",
			coarse.size(), coarse[best].score, coarse[best].z, coarse[best].z - coarse[best - 1].z, coarse[best + 1].z - coarse[best].z);
	}
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
			if (!zsSample2D(z, c.boxPx, confirm, s, why, status)) return false;
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
	if (!zsSample2D(centre, c.boxPx, confirm, last, why, status)) return false;
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
* One read where the head already is clear: Z up to z first when the head is lower, the laser over the
* camera point, Z to z, five live profiles. No search - the caller knows z puts the surface in range.
* The controller is ALWAYS put back in encoder batch mode, which production scans need.
*/
bool VisionApp::zsRead3DAt(double camX, double camY, double z, double maxSpreadMm, zsearch::ProfileStats& st, bool& valid, QString& why, QLabel* status)
{
	auto& pm = ProfilerManager::instance();
	valid = false;
	const double lx = camX + _laserConfig.offset.wx;
	const double ly = camY + _laserConfig.offset.wy;
	if (!zsMoveToXY(lx, ly, z, why)) return false;
	if (!zsMove({ { (int)Axis::Z, z } }, why)) return false;

	pm.stop(_profilerID); //live reads need scan acquisition stopped
	if (!pm.startLive(_profilerID)) { why = QStringLiteral("the profiler could not enter live mode"); return false; }
	int offered = 0;
	bool ok = zsReadProfile(st, offered, why);
	if (ok) {
		valid = st.valid >= std::max(kMinProfilePoints, offered / 2) && st.spread <= maxSpreadMm;
		ct::logger::info("[ZSearch] 3D read, laser at %.3f, %.3f (camera %.3f, %.3f), z=%.3f points=%d/%d reading=%.4f spread=%.4f %s",
			lx, ly, camX, camY, z, st.valid, offered, st.median, st.spread, valid ? "FLAT" : "-");
		if (status) status->setText(st.valid > 0
			? QStringLiteral("Z %1  reading %2 mm  spread %3 mm  (%4 of %5 points)%6").arg(mm(z)).arg(st.median, 0, 'f', 3)
				.arg(st.spread, 0, 'f', 3).arg(st.valid).arg(offered).arg(valid ? QString() : QStringLiteral("  - not flat enough"))
			: QStringLiteral("Z %1  nothing in range").arg(mm(z)));
	}

	if (!pm.stopLive(_profilerID)) {
		ct::logger::error("[ZSearch] FAILED to restore the profiler's encoder batch mode after the 3D read");
		why = (ok ? QString() : why + QStringLiteral("\n\n")) + QStringLiteral(
			"The profiler did NOT go back to encoder batch mode - press Start/Stop Live on the 3D Optics page before scanning.");
		ok = false;
	}
	return ok;
}
