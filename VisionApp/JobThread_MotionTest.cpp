// =============================================================================
//  JobThread_MotionTest.cpp
//  Motion Accuracy Test (Test Run page): landing accuracy per axis, random X/Y/Z
//  moves like a real run, and what the axes and their drives do through a curtain
//  trip / e-stop and reset.
//  See the declaration in JobThread.h. The dialog is VisionApp_MotionTest.cpp.
// =============================================================================

#include "JobThread.h"
#include "CommonDir.h"
#include "Logger.h"
#include "QOSTool.h"
#include "MotionController.h"
#include "MachineController.h"

#include <QDir>
#include <QFile>
#include <QDateTime>
#include <QElapsedTimer>
#include <QHostInfo>
#include <QTextStream>
#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

namespace {
	const char* const kAxisNames[3] = { "X", "Y", "Z" };
	const int kSampleMs[] = { 0, 100, 250, 500, 1000, 2000 }; //after "move done"
	constexpr int kSamples = sizeof(kSampleMs) / sizeof(kSampleMs[0]);
	constexpr int kMoveTimeoutMs = 60000;
	constexpr double kGapWarnMm = 0.1;         //command vs encoder: beyond this the axis moved without torque
	constexpr double kMovedMm = 0.02;          //Part B: an encoder change this big counts as the axis moving
	constexpr double kOnTargetMm = 0.005;      //Part A: a mean error this small is simply on target
	constexpr int kWatchAfterHoldMs = 30000;   //Part B: keep watching this long after the drives hold again
	constexpr int kWatchPeriodMs = 50;

	//Part C
	constexpr double kRandomMarginMm = 1.0;    //targets stay this far inside each soft limit
	const int kFollowMarkMs[] = { 0, 100, 250, 500, 1000 }; //after the axis's own "move done"
	constexpr int kFollowMarks = sizeof(kFollowMarkMs) / sizeof(kFollowMarkMs[0]);
	constexpr int kFollowMs = 1000;
	constexpr int kFollowPeriodMs = 5;
	constexpr double kSettleWideMm = 0.05;     //JobThread::settleAxis's tolerance
	constexpr double kSettleTightMm = 0.01;
	constexpr int kFrozenWarnMs = 50;          //a position read standing still this long while off target

	//bits as Motion_APS::get_motion_io_status() orders them
	enum IoBit { IO_ALM = 0, IO_EMG = 4, IO_INP = 6, IO_SVON = 7, IO_RDY = 8 };

	QString f3(double v) { return QString::number(v, 'f', 3); }
	QString f4(double v) { return QString::number(v, 'f', 4); }
	QString sgn(double v) { return (v >= 0.0 ? QStringLiteral("+") : QString()) + QString::number(v, 'f', 3); }

	bool bitOf(const std::vector<bool>& io, int b) { return (int)io.size() > b && io[b]; }

	struct AxisNow {
		bool ok = false;
		double enc = 0.0;          //drive encoder (APS_get_position)
		double cmd = 0.0;          //card command counter (APS_get_command)
		std::vector<bool> io;
	};

	struct Landing {
		int axis = 0;
		int speedPct = 100;
		bool positive = true;      //arrived travelling in +
		double err[kSamples] = {}; //encoder - target
		bool inp[kSamples] = {};
	};

	struct RandomLanding {
		int move = 0;
		int axis = 0;
		double from = 0.0, target = 0.0;
		bool inpAtDone = false;
		double err[kFollowMarks] = {}; //encoder - target
		int settledWideMs = -1;        //from here on within kSettleWideMm; -1 = not by kFollowMs
		int settledTightMs = -1;
		int frozenMs = 0;              //longest the position read stood still while > kSettleWideMm off, then moved again
		double cmdErr = 0.0;           //command - target at the end
		double travel() const { return std::abs(target - from); }
	};

	double median(std::vector<double> v)
	{
		if (v.empty()) return 0.0;
		std::sort(v.begin(), v.end());
		return v[v.size() / 2];
	}
}

void JobThread::motionAccuracyTest(int axesMask, double distanceMm, int repeats, int slowPercent, int randomSeconds, bool watchReset, int watchSeconds)
{
	m_stopRun = false; //explicit new intent, as every other manual entry point
	auto& mc = MotionController::instance();

	QString report, landingsCsv, randomCsv, watchCsv;
	QTextStream out(&report), lc(&landingsCsv), rc(&randomCsv), wc(&watchCsv);

	auto say = [&](const QString& line) {
		ct::logger::info("[MotionTest] %s", line.toUtf8().constData());
		emit profilerScanTestProgress(line); //the Test Run status box, shared with the other test runs
	};
	auto section = [&](const QString& title) {
		out << "\n" << title << "\n" << QString(title.size(), QLatin1Char('-')) << "\n";
	};

	const QDateTime now = QDateTime::currentDateTime();
	const QString dir = QStringLiteral("%1MotionTest/%2/").arg(Common::Directory::LocalPath).arg(now.toString(QStringLiteral("yyyyMMdd_HHmmss")));
	const QString reportPath = dir + QStringLiteral("report.txt");
	if (!QDir().mkpath(dir)) ct::logger::error("[MotionTest] Could not create %s", qPrintable(dir));

	auto writeFile = [&](const QString& name, const QString& text) {
		if (text.isEmpty()) return;
		QFile f(dir + name);
		if (f.open(QIODevice::WriteOnly | QIODevice::Text)) { QTextStream ts(&f); ts << text; }
		else ct::logger::error("[MotionTest] Could not write %s", qPrintable(dir + name));
	};
	//single exit: the report is written whatever happened
	auto finish = [&](bool ran, const QString& summary) {
		out << "\n" << QString(78, QLatin1Char('=')) << "\n" << "VERDICT: " << summary << "\n";
		out.flush(); lc.flush(); rc.flush(); wc.flush();
		writeFile(QStringLiteral("report.txt"), report);
		writeFile(QStringLiteral("landings.csv"), landingsCsv);
		writeFile(QStringLiteral("random_moves.csv"), randomCsv);
		writeFile(QStringLiteral("reset_watch.csv"), watchCsv);
		ct::logger::info("[MotionTest] Report written: %s", qPrintable(reportPath));
		emit motionAccuracyTestDone(ran, reportPath, summary);
	};

	auto readAxis = [&](int axis) {
		AxisNow a;
		const auto e = mc.get_position_mm(m_motionID, 0, axis);
		const auto c = mc.get_command_mm(m_motionID, 0, axis);
		const auto io = mc.get_motion_io_status(m_motionID, axis);
		a.ok = e.has_value() && c.has_value();
		if (e) a.enc = *e;
		if (c) a.cmd = *c;
		if (io) a.io = *io;
		return a;
	};
	auto bitsText = [](const std::vector<bool>& io) {
		if (io.empty()) return QStringLiteral("(drive bits unreadable)");
		return QStringLiteral("ALM %1  EMG %2  SVON %3  RDY %4  INP %5")
			.arg(bitOf(io, IO_ALM)).arg(bitOf(io, IO_EMG)).arg(bitOf(io, IO_SVON)).arg(bitOf(io, IO_RDY)).arg(bitOf(io, IO_INP));
	};

	//------------------------------------------------------------------ header
	out << QString(78, QLatin1Char('=')) << "\n" << "MOTION ACCURACY TEST\n" << QString(78, QLatin1Char('=')) << "\n";
	out << "  When            : " << now.toString(QStringLiteral("yyyy-MM-dd HH:mm:ss")) << "\n";
	out << "  Machine         : " << QHostInfo::localHostName() << "\n";
	out << "  Recipe          : " << Common::Directory::CurrentRecipe << "\n";
	out << "  Output folder   : " << dir << "\n";
	out << "  Part A axes     : " << (axesMask & 1 ? "X " : "") << (axesMask & 2 ? "Y " : "") << (axesMask & 4 ? "Z" : "")
		<< (axesMask == 0 ? "(none)" : "") << "\n";
	out << "  Distance        : " << f3(distanceMm) << " mm, " << repeats << " repeats each way"
		<< (slowPercent > 0 ? QStringLiteral(", then again at %1 % speed").arg(slowPercent) : QString()) << "\n";
	out << "  Part C          : " << (randomSeconds > 0 ? QStringLiteral("random X/Y/Z moves for %1 s").arg(randomSeconds) : QStringLiteral("no")) << "\n";
	out << "  Part B          : " << (watchReset ? QStringLiteral("yes, up to %1 s").arg(watchSeconds) : QStringLiteral("no")) << "\n";
	out << "  Reading         : encoder = APS_get_position (the drive's own encoder over EtherCAT),\n"
		<< "                    command = APS_get_command (where the card last sent the axis).\n";

	if (!mc.is_init(m_motionID)) { finish(false, QStringLiteral("Motion is not initialised - nothing tested.")); return; }

	//------------------------------------------------------------- counters now
	section(QStringLiteral("COUNTERS BEFORE THE TEST"));
	for (int axis = 0; axis < 3; axis++) {
		const auto a = readAxis(axis);
		if (!a.ok) { out << "  " << kAxisNames[axis] << "  unreadable\n"; continue; }
		const double gap = a.enc - a.cmd;
		out << "  " << kAxisNames[axis] << "  encoder " << f4(a.enc) << "  command " << f4(a.cmd) << "  encoder-command " << sgn(gap)
			<< "   " << bitsText(a.io) << (std::abs(gap) > kGapWarnMm ? "   <- MOVED WITHOUT TORQUE since the last re-sync" : "") << "\n";
	}

	QStringList verdicts;
	bool stoppedByUser = false;

	//================================================================== PART A
	if (axesMask != 0) {
		section(QStringLiteral("PART A - LANDING ACCURACY"));
		out << "  Error = encoder - target after the card reports the move done. Read at "
			<< "0, 100, 250, 500, 1000, 2000 ms.\n"
			<< "  STATIC = stays after landing (drive parks off its command: in-position width / tuning).\n"
			<< "  SETTLING = shrinks after landing (the move ends before the axis does: speed / gains).\n";

		if (!mc.motionEnabled()) {
			out << "  NOT RUN: motion is disabled - home the machine or clear the active error.\n";
			verdicts << QStringLiteral("Part A not run: motion disabled");
		}
		else {
			lc << "axis,speed_pct,repeat,direction,target_mm,t_ms,encoder_mm,command_mm,error_mm,inp\n";
			std::vector<Landing> landings;
			bool aborted = false;
			QString abortWhy;

			for (int axis = 0; axis < 3 && !aborted; axis++) {
				if (!(axesMask & (1 << axis))) continue;
				const QString name = kAxisNames[axis];

				const auto start = readAxis(axis);
				if (!start.ok) { out << "  " << name << ": position unreadable - skipped\n"; continue; }
				if (std::abs(start.enc - start.cmd) > kGapWarnMm) {
					//an absolute move would land off by the gap - and for Z that could be below the start
					out << "  " << name << ": SKIPPED - command and encoder differ by " << f3(start.enc - start.cmd)
						<< " mm. Press Reset (it re-syncs them) and run again; Part B shows how this happens.\n";
					verdicts << QStringLiteral("%1 skipped: counters %2 mm apart").arg(name).arg(f3(start.enc - start.cmd));
					continue;
				}

				//Z never goes below where it starts: up by the distance, and back down to the start
				const double p0 = start.enc;
				const double farEnd = (axis == (int)Axis::Z) ? p0 - distanceMm : p0 + distanceMm;
				double lo = 0.0, hi = 0.0;
				if (!mc.limits_mm(m_motionID, axis, lo, hi) || !(lo < hi) || farEnd < lo || farEnd > hi || p0 < lo || p0 > hi) {
					out << "  " << name << ": SKIPPED - " << f3(p0) << " .. " << f3(farEnd) << " is not inside the soft limits\n";
					verdicts << QStringLiteral("%1 skipped: outside the soft limits").arg(name);
					continue;
				}

				const auto normalSpeed = mc.get_move_max_velocity_mm(m_motionID, axis);
				out << "  " << name << ": between " << f3(p0) << " and " << f3(farEnd) << ", speed "
					<< (normalSpeed ? f3(*normalSpeed) + QStringLiteral(" mm/s") : QStringLiteral("unknown")) << "\n";

				std::vector<int> speeds{ 100 };
				if (slowPercent > 0) speeds.push_back(slowPercent);

				for (int pct : speeds) {
					if (aborted) break;
					if (pct != 100) {
						if (!normalSpeed || *normalSpeed <= 0.0) { out << "  " << name << ": slow pass skipped - the speed cannot be read back to restore\n"; continue; }
						mc.set_move_max_velocity(m_motionID, axis, *normalSpeed * pct / 100.0);
					}
					say(QStringLiteral("%1 at %2 %: %3 x back and forth over %4 mm").arg(name).arg(pct).arg(repeats).arg(f3(distanceMm)));

					double previous = p0;
					for (int r = 0; r < repeats && !aborted; r++) {
						for (double target : { farEnd, p0 }) {
							if (m_stopRun) { aborted = true; abortWhy = QStringLiteral("stopped"); break; }

							Landing L;
							L.axis = axis;
							L.speedPct = pct;
							L.positive = target > previous;
							previous = target;

							if (!mc.absolute_move(m_motionID, axis, target)) {
								aborted = true;
								abortWhy = QStringLiteral("the %1 move to %2 was refused").arg(name).arg(f3(target));
								break;
							}
							os_tool::doNothing(m_motionReadDelay_ms);
							QElapsedTimer t;
							t.start();
							while (!mc.move_done(m_motionID, 0, axis) && !m_stopRun && t.elapsed() < kMoveTimeoutMs)
								os_tool::goSleep(10);
							if (m_stopRun) { mc.stop_move(m_motionID, axis); aborted = true; abortWhy = QStringLiteral("stopped"); break; }
							if (!mc.move_done(m_motionID, 0, axis)) {
								mc.stop_move(m_motionID, axis);
								aborted = true;
								abortWhy = QStringLiteral("a %1 move did not finish within %2 s").arg(name).arg(kMoveTimeoutMs / 1000);
								break;
							}

							QElapsedTimer s;
							s.start();
							for (int k = 0; k < kSamples; k++) {
								while (s.elapsed() < kSampleMs[k]) os_tool::goSleep(5);
								const auto a = readAxis(axis);
								L.err[k] = a.enc - target;
								L.inp[k] = bitOf(a.io, IO_INP);
								lc << name << "," << pct << "," << (r + 1) << "," << (L.positive ? "+" : "-") << "," << f4(target) << ","
									<< kSampleMs[k] << "," << f4(a.enc) << "," << f4(a.cmd) << "," << f4(L.err[k]) << "," << (L.inp[k] ? 1 : 0) << "\n";
							}
							landings.push_back(L);
						}
						if (!landings.empty() && !aborted)
							say(QStringLiteral("%1 %2 % repeat %3/%4: error after 2 s %5 mm").arg(name).arg(pct).arg(r + 1).arg(repeats)
								.arg(sgn(landings.back().err[kSamples - 1])));
					}

					//the speed goes back whatever happened in the pass
					if (pct != 100 && normalSpeed) mc.set_move_max_velocity(m_motionID, axis, *normalSpeed);
				}
			}

			//-------------------------------------------------------- analysis
			out << "\n  " << QStringLiteral("axis speed dir  n   at landing: mean (min..max)        after 2 s: mean (min..max)         INP@0  verdict") << "\n";
			for (int axis = 0; axis < 3; axis++) {
				if (!(axesMask & (1 << axis))) continue;
				double meanAtSpeed[2] = { 0.0, 0.0 };
				int countAtSpeed[2] = { 0, 0 };

				std::vector<int> speeds{ 100 };
				if (slowPercent > 0) speeds.push_back(slowPercent);
				for (size_t si = 0; si < speeds.size(); si++) {
					for (bool positive : { true, false }) {
						std::vector<const Landing*> set;
						for (const auto& L : landings)
							if (L.axis == axis && L.speedPct == speeds[si] && L.positive == positive) set.push_back(&L);
						if (set.empty()) continue;

						auto stat = [&](int k, double& mean, double& mn, double& mx) {
							mean = 0.0; mn = set[0]->err[k]; mx = mn;
							for (auto* L : set) { mean += L->err[k]; mn = std::min(mn, L->err[k]); mx = std::max(mx, L->err[k]); }
							mean /= set.size();
						};
						double m0, n0, x0, m2, n2, x2;
						stat(0, m0, n0, x0);
						stat(kSamples - 1, m2, n2, x2);
						int inp0 = 0;
						for (auto* L : set) inp0 += L->inp[0] ? 1 : 0;

						QString verdict;
						if (std::abs(m2) <= kOnTargetMm && (x2 - n2) <= 2 * kOnTargetMm) verdict = QStringLiteral("on target");
						else if (std::abs(m0) - std::abs(m2) > 0.5 * std::abs(m0)) verdict = QStringLiteral("SETTLING");
						else verdict = QStringLiteral("STATIC");

						out << "  " << kAxisNames[axis] << "    " << QString::number(speeds[si]).rightJustified(3) << "% "
							<< (positive ? " + " : " - ") << QString::number(set.size()).rightJustified(3) << "   "
							<< sgn(m0) << " (" << sgn(n0) << ".." << sgn(x0) << ")      "
							<< sgn(m2) << " (" << sgn(n2) << ".." << sgn(x2) << ")      "
							<< inp0 << "/" << set.size() << "   " << verdict << "\n";

						meanAtSpeed[si] += std::abs(m2) * set.size();
						countAtSpeed[si] += (int)set.size();

						if (verdict != QStringLiteral("on target") && si == 0)
							verdicts << QStringLiteral("%1 %2: %3, %4 mm after 2 s").arg(kAxisNames[axis]).arg(positive ? "+" : "-").arg(verdict).arg(sgn(m2));
					}
				}
				if (countAtSpeed[0] > 0 && countAtSpeed[1] > 0) {
					const double fast = meanAtSpeed[0] / countAtSpeed[0], slow = meanAtSpeed[1] / countAtSpeed[1];
					out << "  " << kAxisNames[axis] << "    speed: mean |error| after 2 s " << f3(fast) << " mm at 100 %, " << f3(slow) << " mm at " << slowPercent << " % -> "
						<< (std::abs(fast - slow) <= kOnTargetMm ? "speed makes no difference"
							: (slow < fast ? "slower lands closer: speed matters" : "slower is no better")) << "\n";
				}
			}

			if (aborted) {
				out << "\n  PART A ENDED EARLY: " << abortWhy << "\n";
				verdicts << QStringLiteral("Part A ended early: %1").arg(abortWhy);
				stoppedByUser = (abortWhy == QStringLiteral("stopped"));
			}
		}
	}

	//================================================================== PART C
	if (randomSeconds > 0) {
		section(QStringLiteral("PART C - RANDOM MOVES"));
		out << "  X, Y and Z go together (as jog() sends them) to random points inside the soft limits, " << f3(kRandomMarginMm) << " mm in\n"
			<< "  from each, at the speeds set now. From each axis's own \"move done\" its encoder is read every few ms for 1 s.\n"
			<< "  Error = encoder - target. Settled = within the tolerance from then on, to the end of that second.\n"
			<< "  Frozen = the position read stood still while more than " << f3(kSettleWideMm) << " mm off, then moved again\n"
			<< "  (the axis paused, or the reading was stale).\n";

		QString why;
		AxisNow start[3];
		double lo[3] = {}, hi[3] = {};
		if (stoppedByUser) why = QStringLiteral("the test was stopped during Part A");
		else if (!mc.motionEnabled()) why = QStringLiteral("motion is disabled - home the machine or clear the active error");
		else if (MachineController::instance().trolleyAttached())
			why = QStringLiteral("a trolley is attached - Z goes down to its lowest soft limit, so this runs only with no trolley");
		for (int axis = 0; axis < 3 && why.isEmpty(); axis++) {
			const QString name = kAxisNames[axis];
			start[axis] = readAxis(axis);
			if (!start[axis].ok) why = QStringLiteral("the %1 position is unreadable").arg(name);
			else if (std::abs(start[axis].enc - start[axis].cmd) > kGapWarnMm)
				why = QStringLiteral("%1 command and encoder differ by %2 mm - press Reset (it re-syncs them) and run again").arg(name).arg(f3(start[axis].enc - start[axis].cmd));
			else if (!mc.limits_mm(m_motionID, axis, lo[axis], hi[axis]) || hi[axis] - lo[axis] <= 2 * kRandomMarginMm)
				why = QStringLiteral("the %1 soft limits are unreadable or too close together").arg(name);
		}

		if (!why.isEmpty()) {
			out << "  NOT RUN: " << why << ".\n";
			verdicts << QStringLiteral("Part C not run: %1").arg(why);
		}
		else {
			const unsigned seed = (unsigned)QDateTime::currentMSecsSinceEpoch();
			std::mt19937 rng(seed);
			std::uniform_real_distribution<double> pick[3];
			for (int axis = 0; axis < 3; axis++) {
				pick[axis] = std::uniform_real_distribution<double>(lo[axis] + kRandomMarginMm, hi[axis] - kRandomMarginMm);
				const auto speed = mc.get_move_max_velocity_mm(m_motionID, axis);
				out << "  " << kAxisNames[axis] << ": " << f3(lo[axis] + kRandomMarginMm) << " .. " << f3(hi[axis] - kRandomMarginMm) << " mm, speed "
					<< (speed ? f3(*speed) + QStringLiteral(" mm/s") : QStringLiteral("unknown")) << ", start " << f3(start[axis].enc) << "\n";
			}
			out << "  Random seed: " << seed << "\n";
			rc << "move,axis,from_mm,target_mm,travel_mm,move_ms,inp_at_done,err_0,err_100,err_250,err_500,err_1000,"
				<< "settled_0.05_ms,settled_0.01_ms,frozen_ms,command_minus_target\n";
			say(QStringLiteral("Part C: random X/Y/Z moves for %1 s - Z goes down to its lowest soft limit").arg(randomSeconds));

			const int order[3] = { (int)Axis::Z, (int)Axis::X, (int)Axis::Y }; //as jog() sends them
			auto stopAll = [&]() { for (int axis = 0; axis < 3; axis++) mc.stop_move(m_motionID, axis); };
			auto allDone = [&]() {
				for (int axis = 0; axis < 3; axis++) if (!mc.move_done(m_motionID, 0, axis)) return false;
				return true;
			};

			std::vector<RandomLanding> rl;
			std::vector<double> moveMs;
			double from[3] = { start[0].enc, start[1].enc, start[2].enc };
			bool cAborted = false;
			QString cWhy;
			QElapsedTimer total;
			total.start();
			int move = 0;
			while (total.elapsed() < (qint64)randomSeconds * 1000) {
				if (m_stopRun) { cAborted = true; cWhy = QStringLiteral("stopped"); break; }
				move++;
				double target[3];
				for (int axis = 0; axis < 3; axis++) target[axis] = std::round(pick[axis](rng) * 1000.0) / 1000.0;

				for (int axis : order) {
					if (!mc.absolute_move(m_motionID, axis, target[axis])) {
						cAborted = true;
						cWhy = QStringLiteral("the %1 move to %2 was refused").arg(kAxisNames[axis]).arg(f3(target[axis]));
						break;
					}
				}
				if (cAborted) { stopAll(); break; }
				os_tool::doNothing(m_motionReadDelay_ms);

				RandomLanding L[3];
				qint64 doneAt[3] = { -1, -1, -1 };
				bool followed[3] = { false, false, false };
				int nextMark[3] = { 0, 0, 0 };
				bool havePos[3] = { false, false, false };
				double lastPos[3] = {};
				qint64 lastChange[3] = {};
				for (int axis = 0; axis < 3; axis++) { L[axis].move = move; L[axis].axis = axis; L[axis].from = from[axis]; L[axis].target = target[axis]; }

				//each axis is followed from its OWN done: they finish at different times
				QElapsedTimer t;
				t.start();
				while (true) {
					if (m_stopRun) { cAborted = true; cWhy = QStringLiteral("stopped"); break; }
					const qint64 ms = t.elapsed();
					bool finished = true;
					for (int axis = 0; axis < 3; axis++) {
						if (followed[axis]) continue;
						finished = false;
						auto& a = L[axis];
						if (doneAt[axis] < 0) {
							if (!mc.move_done(m_motionID, 0, axis)) continue;
							doneAt[axis] = ms;
							const auto io = mc.get_motion_io_status(m_motionID, axis);
							a.inpAtDone = io && bitOf(*io, IO_INP);
						}
						const auto p = mc.get_position_mm(m_motionID, 0, axis);
						if (!p) continue;
						const qint64 since = ms - doneAt[axis];
						const double err = *p - target[axis];

						while (nextMark[axis] < kFollowMarks && since >= kFollowMarkMs[nextMark[axis]]) a.err[nextMark[axis]++] = err;
						if (std::abs(err) > kSettleWideMm) a.settledWideMs = -1; else if (a.settledWideMs < 0) a.settledWideMs = (int)since;
						if (std::abs(err) > kSettleTightMm) a.settledTightMs = -1; else if (a.settledTightMs < 0) a.settledTightMs = (int)since;

						if (!havePos[axis] || *p != lastPos[axis]) {
							//a read that stood still while well off target and has now moved again
							if (havePos[axis] && std::abs(lastPos[axis] - target[axis]) > kSettleWideMm)
								a.frozenMs = std::max(a.frozenMs, (int)(since - lastChange[axis]));
							havePos[axis] = true;
							lastPos[axis] = *p;
							lastChange[axis] = since;
						}

						if (since >= kFollowMs) {
							followed[axis] = true;
							const auto c = mc.get_command_mm(m_motionID, 0, axis);
							a.cmdErr = c.value_or(target[axis]) - target[axis];
						}
					}
					if (finished) break;
					if (ms > kMoveTimeoutMs + kFollowMs) { cAborted = true; cWhy = QStringLiteral("a move did not finish within %1 s").arg(kMoveTimeoutMs / 1000); break; }
					os_tool::goSleep(kFollowPeriodMs);
				}
				if (cAborted) { stopAll(); break; }

				moveMs.push_back((double)*std::max_element(doneAt, doneAt + 3));
				for (int axis = 0; axis < 3; axis++) {
					const auto& a = L[axis];
					rc << move << "," << kAxisNames[axis] << "," << f4(a.from) << "," << f4(a.target) << "," << f4(a.travel()) << ","
						<< doneAt[axis] << "," << (a.inpAtDone ? 1 : 0);
					for (int k = 0; k < kFollowMarks; k++) rc << "," << f4(a.err[k]);
					rc << "," << a.settledWideMs << "," << a.settledTightMs << "," << a.frozenMs << "," << f4(a.cmdErr) << "\n";
					rl.push_back(a);
					from[axis] = target[axis];
				}
				if (move % 10 == 0)
					say(QStringLiteral("Part C: %1 moves, %2 s left").arg(move).arg(std::max<qint64>(0, randomSeconds - total.elapsed() / 1000)));
			}
			const qint64 runMs = total.elapsed();

			//back where the operator left it - only after a clean finish: a refusal or timeout says the machine is not fit to move
			if (!cAborted) {
				say(QStringLiteral("Part C: back to the start point"));
				bool back = true;
				for (int axis : order) back = mc.absolute_move(m_motionID, axis, start[axis].enc) && back;
				os_tool::doNothing(m_motionReadDelay_ms);
				QElapsedTimer t;
				t.start();
				while (!allDone() && !m_stopRun && t.elapsed() < kMoveTimeoutMs) os_tool::goSleep(10);
				if (!allDone()) { stopAll(); back = false; }
				if (!back) verdicts << QStringLiteral("Part C could not return to the start point");
			}

			//-------------------------------------------------------- analysis
			const int n = (int)rl.size() / 3;
			out << "\n  Moves: " << n << " in " << f3(runMs / 1000.0) << " s";
			if (!moveMs.empty())
				out << ", move time (sent -> last axis done) median " << median(moveMs) << " ms, longest " << *std::max_element(moveMs.begin(), moveMs.end()) << " ms";
			out << "\n";

			if (n > 0) {
				out << "\n  |error| from each axis's own \"move done\", mm - mean / max\n";
				out << "  axis     n   at done          0.1 s            0.25 s           0.5 s            1 s              INP@done\n";
				for (int axis = 0; axis < 3; axis++) {
					std::vector<const RandomLanding*> set;
					for (const auto& a : rl) if (a.axis == axis) set.push_back(&a);
					out << "  " << kAxisNames[axis] << "    " << QString::number(set.size()).rightJustified(4) << "   ";
					for (int k = 0; k < kFollowMarks; k++) {
						double mean = 0.0, mx = 0.0;
						for (auto* a : set) { mean += std::abs(a->err[k]); mx = std::max(mx, std::abs(a->err[k])); }
						mean /= set.size();
						out << (f3(mean) + " / " + f3(mx)).leftJustified(17);
					}
					int inp = 0;
					for (auto* a : set) inp += a->inpAtDone ? 1 : 0;
					out << inp << "/" << set.size() << "\n";
				}

				out << "\n  Settle time from \"move done\", ms - median / max (landings not settled within 1 s)\n";
				out << "  axis   within " << f3(kSettleWideMm) << "              within " << f3(kSettleTightMm) << "              frozen >= " << kFrozenWarnMs
					<< " ms   command off target\n";
				for (int axis = 0; axis < 3; axis++) {
					std::vector<double> wide, tight;
					int wideMiss = 0, tightMiss = 0, frozen = 0, frozenMax = 0, cmdOff = 0;
					for (const auto& a : rl) {
						if (a.axis != axis) continue;
						if (a.settledWideMs >= 0) wide.push_back(a.settledWideMs); else wideMiss++;
						if (a.settledTightMs >= 0) tight.push_back(a.settledTightMs); else tightMiss++;
						if (a.frozenMs >= kFrozenWarnMs) frozen++;
						frozenMax = std::max(frozenMax, a.frozenMs);
						if (std::abs(a.cmdErr) > 0.0005) cmdOff++;
					}
					auto cell = [&](const std::vector<double>& v, int miss) {
						const QString s = v.empty() ? QStringLiteral("-") : QStringLiteral("%1 / %2").arg(median(v)).arg(*std::max_element(v.begin(), v.end()));
						return (s + QStringLiteral(" (%1)").arg(miss)).leftJustified(23);
					};
					out << "  " << kAxisNames[axis] << "      " << cell(wide, wideMiss) << cell(tight, tightMiss)
						<< (QString::number(frozen) + QStringLiteral(" (longest %1 ms)").arg(frozenMax)).leftJustified(23) << cmdOff << "\n";

					const QString name = kAxisNames[axis];
					if (wideMiss > 0) verdicts << QStringLiteral("%1: %2 of %3 random landings not within %4 mm 1 s after done").arg(name).arg(wideMiss).arg(n).arg(f3(kSettleWideMm));
					if (frozen > 0) verdicts << QStringLiteral("%1: position read stood still up to %2 ms while > %3 mm off (%4 landings)").arg(name).arg(frozenMax).arg(f3(kSettleWideMm)).arg(frozen);
					if (cmdOff > 0) verdicts << QStringLiteral("%1: command counter not at the target after %2 landings").arg(name).arg(cmdOff);
				}

				out << "\n  By travel - mean |error| at done / at 0.25 s, median settle within " << f3(kSettleTightMm) << " mm\n";
				const double bands[] = { 0.0, 10.0, 100.0, 1e9 };
				for (int axis = 0; axis < 3; axis++) {
					for (int b = 0; b < 3; b++) {
						std::vector<double> tight;
						double e0 = 0.0, e250 = 0.0;
						int count = 0;
						for (const auto& a : rl) {
							if (a.axis != axis || a.travel() < bands[b] || a.travel() >= bands[b + 1]) continue;
							count++;
							e0 += std::abs(a.err[0]);
							e250 += std::abs(a.err[2]);
							if (a.settledTightMs >= 0) tight.push_back(a.settledTightMs);
						}
						if (count == 0) continue;
						const QString band = b == 2 ? QStringLiteral(">= 100 mm") : QStringLiteral("%1-%2 mm").arg(bands[b]).arg(bands[b + 1]);
						out << "  " << kAxisNames[axis] << "  " << band.leftJustified(10) << QString::number(count).rightJustified(4) << " moves   "
							<< f3(e0 / count) << " / " << f3(e250 / count) << "   settle " << (tight.empty() ? QStringLiteral("-") : QString::number(median(tight)) + QStringLiteral(" ms")) << "\n";
					}
				}

				//a counter that drifts, or a drive that warms up, shows as the two halves differing
				out << "\n  Error 1 s after done, signed mean - first half of the run vs second half\n";
				for (int axis = 0; axis < 3; axis++) {
					double sum[2] = {}, mx = 0.0;
					int cnt[2] = {};
					for (const auto& a : rl) {
						if (a.axis != axis) continue;
						const int h = a.move <= n / 2 ? 0 : 1;
						sum[h] += a.err[kFollowMarks - 1];
						cnt[h]++;
						mx = std::max(mx, std::abs(a.err[kFollowMarks - 1]));
					}
					out << "  " << kAxisNames[axis] << "  " << sgn(cnt[0] ? sum[0] / cnt[0] : 0.0) << "  ->  " << sgn(cnt[1] ? sum[1] / cnt[1] : 0.0)
						<< "   (largest " << f3(mx) << ")\n";
					if (mx > kSettleTightMm) verdicts << QStringLiteral("%1: up to %2 mm off 1 s after done").arg(kAxisNames[axis]).arg(f3(mx));
				}
			}

			if (cAborted) {
				out << "\n  PART C ENDED EARLY: " << cWhy << "\n";
				verdicts << QStringLiteral("Part C ended early: %1").arg(cWhy);
				if (cWhy == QStringLiteral("stopped")) stoppedByUser = true;
			}
		}
	}

	//================================================================== PART B
	if (watchReset && stoppedByUser) {
		section(QStringLiteral("PART B - CURTAIN TRIP / E-STOP AND RESET"));
		out << "  NOT RUN: the test was stopped earlier.\n";
	}
	else if (watchReset) {
		section(QStringLiteral("PART B - CURTAIN TRIP / E-STOP AND RESET"));
		out << "  Nothing moves. 'Holding' = every axis reports SVON 1, RDY 1, ALM 0 and the curtain is not tripped.\n"
			<< "  The trip itself raises Stop - that is expected and ignored here.\n";
		say(QStringLiteral("Part B: trip the curtain or press an e-stop, then press Reset as usual. Watching up to %1 s - nothing moves.").arg(watchSeconds));

		wc << "t_ms,axis,encoder_mm,command_mm,encoder_minus_command_mm,alm,emg,svon,rdy,inp,curtain,phase\n";

		struct Track {
			AxisNow first, atLoss, atHold, last;
			double lastLoggedEnc = 0.0, lastCmd = 0.0;
			std::vector<bool> lastIo;
			double movedUnpowered = 0.0;  //largest encoder change while not holding
			double movedAfterHold = 0.0;  //largest encoder change after holding again
			int cmdJumps = 0;             //command counter changes: nothing moves, so each one is a re-sync
			double cmdJumpTotal = 0.0;
			qint64 lastLogMs = -1000;
		} track[3];

		enum Phase { WAIT_LOSS, LOST, HELD } phase = WAIT_LOSS;
		const char* phaseName[3] = { "before", "no torque", "holding" };
		qint64 lossMs = -1, holdMs = -1;

		QElapsedTimer t;
		t.start();
		bool first = true;
		while (t.elapsed() < (qint64)watchSeconds * 1000) {
			const qint64 ms = t.elapsed();
			const bool curtain = MachineController::instance().curtainTripped();
			AxisNow now3[3];
			bool allHold = !curtain;
			for (int axis = 0; axis < 3; axis++) {
				now3[axis] = readAxis(axis);
				const auto& io = now3[axis].io;
				if (!now3[axis].ok || !bitOf(io, IO_SVON) || !bitOf(io, IO_RDY) || bitOf(io, IO_ALM)) allHold = false;
			}

			if (first) {
				for (int axis = 0; axis < 3; axis++) {
					track[axis].first = now3[axis];
					track[axis].lastCmd = now3[axis].cmd;
					track[axis].lastLoggedEnc = now3[axis].enc;
				}
				first = false;
			}

			if (phase == WAIT_LOSS && !allHold) {
				phase = LOST;
				lossMs = ms;
				for (int axis = 0; axis < 3; axis++) track[axis].atLoss = now3[axis];
				say(QStringLiteral("Part B: torque lost at %1 s - now press Reset.").arg(ms / 1000.0, 0, 'f', 1));
			}
			else if (phase == LOST && allHold) {
				phase = HELD;
				holdMs = ms;
				for (int axis = 0; axis < 3; axis++) track[axis].atHold = now3[axis];
				say(QStringLiteral("Part B: drives holding again at %1 s - watching %2 s more.").arg(ms / 1000.0, 0, 'f', 1).arg(kWatchAfterHoldMs / 1000));
			}

			for (int axis = 0; axis < 3; axis++) {
				auto& tr = track[axis];
				const auto& a = now3[axis];
				if (!a.ok) continue;
				if (phase == LOST) tr.movedUnpowered = std::max(tr.movedUnpowered, std::abs(a.enc - tr.atLoss.enc));
				if (phase == HELD) tr.movedAfterHold = std::max(tr.movedAfterHold, std::abs(a.enc - tr.atHold.enc));
				const bool cmdChanged = std::abs(a.cmd - tr.lastCmd) > 0.0005;
				if (cmdChanged) { tr.cmdJumps++; tr.cmdJumpTotal += a.cmd - tr.lastCmd; tr.lastCmd = a.cmd; }

				//a row when anything changes, otherwise once a second
				if (a.io != tr.lastIo || cmdChanged || std::abs(a.enc - tr.lastLoggedEnc) >= 0.002 || ms - tr.lastLogMs >= 1000) {
					wc << ms << "," << kAxisNames[axis] << "," << f4(a.enc) << "," << f4(a.cmd) << "," << f4(a.enc - a.cmd) << ","
						<< bitOf(a.io, IO_ALM) << "," << bitOf(a.io, IO_EMG) << "," << bitOf(a.io, IO_SVON) << "," << bitOf(a.io, IO_RDY) << ","
						<< bitOf(a.io, IO_INP) << "," << (curtain ? 1 : 0) << "," << phaseName[phase] << "\n";
					tr.lastIo = a.io;
					tr.lastLoggedEnc = a.enc;
					tr.lastLogMs = ms;
				}
				tr.last = a;
			}

			if (phase == HELD && ms - holdMs >= kWatchAfterHoldMs) break;
			os_tool::goSleep(kWatchPeriodMs);
		}
		const qint64 endMs = t.elapsed();

		//the trip raised Stop on purpose; it must not be left set for the next run
		m_stopRun = false;

		//-------------------------------------------------------- analysis
		out << "  Watched " << f3(endMs / 1000.0) << " s. Torque lost at "
			<< (lossMs >= 0 ? f3(lossMs / 1000.0) + QStringLiteral(" s") : QStringLiteral("(never)"))
			<< ", holding again at " << (holdMs >= 0 ? f3(holdMs / 1000.0) + QStringLiteral(" s") : QStringLiteral("(never)")) << ".\n\n";

		if (phase == WAIT_LOSS) {
			out << "  No curtain trip or e-stop was seen - nothing to report.\n";
			verdicts << QStringLiteral("Part B saw no curtain trip or e-stop");
		}
		else {
			for (int axis = 0; axis < 3; axis++) {
				const auto& tr = track[axis];
				const QString name = kAxisNames[axis];
				out << "  " << name << "  encoder: start " << f4(tr.first.enc) << ", at loss " << f4(tr.atLoss.enc)
					<< (phase == HELD ? QStringLiteral(", at holding ") + f4(tr.atHold.enc) : QString()) << ", end " << f4(tr.last.enc) << "\n";
				out << "     moved while not holding : " << f3(tr.movedUnpowered) << " mm\n";
				if (phase == HELD) out << "     moved AFTER holding again: " << f3(tr.movedAfterHold) << " mm\n";
				out << "     command re-syncs        : " << tr.cmdJumps << " (total " << sgn(tr.cmdJumpTotal) << " mm)\n";
				out << "     encoder-command at end  : " << sgn(tr.last.enc - tr.last.cmd) << " mm\n";

				if (phase == HELD && tr.movedAfterHold > kMovedMm)
					verdicts << QStringLiteral("%1 moved %2 mm AFTER its drive reported holding").arg(name).arg(f3(tr.movedAfterHold));
				if (std::abs(tr.last.enc - tr.last.cmd) > kGapWarnMm)
					verdicts << QStringLiteral("%1 counters %2 mm apart after the reset - the next move lands that far off").arg(name).arg(sgn(tr.last.enc - tr.last.cmd));
				if (tr.movedUnpowered > kMovedMm)
					verdicts << QStringLiteral("%1 moved %2 mm without torque").arg(name).arg(f3(tr.movedUnpowered));
			}
			if (phase == LOST) {
				out << "\n  The drives did not report holding again before the time limit.\n";
				verdicts << QStringLiteral("Part B: drives never held again within %1 s").arg(watchSeconds);
			}
		}
	}

	finish(true, verdicts.isEmpty() ? QStringLiteral("No problem seen.") : verdicts.join(QStringLiteral("; ")));
}
