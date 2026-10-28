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
	//Command vs encoder: beyond this the axis moved without torque (unpowered slides measure 2-4 mm). Not 0.1:
	//X holds up to ~0.15 mm off its command after mixed moves, and that must not stop the later parts.
	constexpr double kGapWarnMm = 0.5;
	constexpr double kMovedMm = 0.02;          //Part B: an encoder change this big counts as the axis moving
	constexpr double kOnTargetMm = 0.005;      //Part A: a mean error this small is simply on target
	constexpr int kWatchAfterHoldMs = 30000;   //Part B: keep watching this long after the drives hold again
	constexpr int kWatchPeriodMs = 50;

	//Part C
	constexpr double kBufferFraction = 0.01;   //every target stays this share of the travel inside each soft limit (Jimmy: middle 98 %)
	const int kFollowMarkMs[] = { 0, 100, 250, 500, 1000, 2000 }; //after the axis's own "move done"
	constexpr int kFollowMarks = sizeof(kFollowMarkMs) / sizeof(kFollowMarkMs[0]);
	constexpr int kFollowMs = 2000;            //Y was still closing in at 1 s (2026-10-28 run)
	constexpr int kFollowPeriodMs = 5;
	constexpr double kSettleWideMm = 0.05;     //JobThread::settleAxis's tolerance
	constexpr double kSettleTightMm = 0.01;
	constexpr int kFrozenWarnMs = 50;          //a position read standing still this long while off target

	//Part D
	constexpr double kShortMinMm = 2.0;
	constexpr double kShortMaxMm = 20.0;
	constexpr double kLongFraction = 0.4;      //a long move covers at least this share of the axis's travel
	constexpr double kStepMm = 0.005;          //a settled error this much different from the previous move's is a step
	constexpr int kRegions = 5;                //the travel split into this many equal stretches for the by-position table
	const char* const kBlockNames[3] = { "SHORT", "LONG", "SHORT" };

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
		double tSec = 0.0;             //when the move was sent, from the start of Part C
		double from = 0.0, target = 0.0;
		bool inpAtDone = false;
		double err[kFollowMarks] = {}; //encoder - target
		int settledWideMs = -1;        //from here on within kSettleWideMm; -1 = not by kFollowMs
		int settledTightMs = -1;
		int frozenMs = 0;              //longest the position read stood still while > kSettleWideMm off, then moved again
		double cmdErr = 0.0;           //command - target at the end
		double travel() const { return std::abs(target - from); }
	};

	//Part D: one landing of one axis moving on its own
	struct TravelLanding {
		int block = 0;                 //0 short, 1 long, 2 short again
		RandomLanding L;
		double settled = 0.0;          //error kFollowMs after done
		double step = 0.0;             //settled minus the previous landing's settled
		//the two axes standing still: encoder - command after the follow, and how much this move changed it
		bool heldOk[3] = {};
		double held[3] = {};
		double heldShift[3] = {};
	};

	double median(std::vector<double> v)
	{
		if (v.empty()) return 0.0;
		std::sort(v.begin(), v.end());
		return v[v.size() / 2];
	}
}

void JobThread::motionAccuracyTest(int axesMask, double distanceMm, int repeats, int slowPercent, int randomSeconds, int randomSlowPercent,
	int travelAxesMask, int travelBlockSeconds, int pairSeconds, bool watchReset, int watchSeconds)
{
	m_stopRun = false; //explicit new intent, as every other manual entry point
	auto& mc = MotionController::instance();

	QString report, landingsCsv, randomCsv, travelCsv, pairCsv, watchCsv;
	QTextStream out(&report), lc(&landingsCsv), rc(&randomCsv), tc(&travelCsv), ec(&pairCsv), wc(&watchCsv);

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
		//one finding per line: the full test produces dozens
		out << "\n" << QString(78, QLatin1Char('=')) << "\n" << "VERDICT\n";
		for (const QString& line : summary.split(QStringLiteral("; "))) out << "  - " << line << "\n";
		out.flush(); lc.flush(); rc.flush(); tc.flush(); ec.flush(); wc.flush();
		writeFile(QStringLiteral("report.txt"), report);
		writeFile(QStringLiteral("landings.csv"), landingsCsv);
		writeFile(QStringLiteral("random_moves.csv"), randomCsv);
		writeFile(QStringLiteral("travel_moves.csv"), travelCsv);
		writeFile(QStringLiteral("pair_moves.csv"), pairCsv);
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

	//The soft limits less kBufferFraction of the travel at each end: the only range any part moves in.
	auto testRange = [&](int axis, double& lo, double& hi) -> bool {
		if (!mc.limits_mm(m_motionID, axis, lo, hi) || !(lo < hi)) return false;
		const double buffer = (hi - lo) * kBufferFraction;
		lo += buffer;
		hi -= buffer;
		return true;
	};

	/*
	* Parts C, D and E: follow each axis in `moving` from its OWN "move done" for kFollowMs - axes
	* finish at different times - filling L[axis], whose target must already be set: error at the
	* marks, settle times, frozen reads and the command counter at the end. doneAt[axis] = ms from
	* this call to that axis's done. False when stopped or timed out; why says which.
	*/
	auto followLandings = [&](const bool moving[3], RandomLanding L[3], qint64 doneAt[3], QString& why) -> bool {
		bool followed[3] = { !moving[0], !moving[1], !moving[2] };
		int nextMark[3] = { 0, 0, 0 };
		bool havePos[3] = { false, false, false };
		double lastPos[3] = {};
		qint64 lastChange[3] = {};
		for (int axis = 0; axis < 3; axis++) doneAt[axis] = -1;

		QElapsedTimer t;
		t.start();
		while (true) {
			if (m_stopRun) { why = QStringLiteral("stopped"); return false; }
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
				const double err = *p - a.target;

				while (nextMark[axis] < kFollowMarks && since >= kFollowMarkMs[nextMark[axis]]) a.err[nextMark[axis]++] = err;
				if (std::abs(err) > kSettleWideMm) a.settledWideMs = -1; else if (a.settledWideMs < 0) a.settledWideMs = (int)since;
				if (std::abs(err) > kSettleTightMm) a.settledTightMs = -1; else if (a.settledTightMs < 0) a.settledTightMs = (int)since;

				if (!havePos[axis] || *p != lastPos[axis]) {
					//a read that stood still while well off target and has now moved again
					if (havePos[axis] && std::abs(lastPos[axis] - a.target) > kSettleWideMm)
						a.frozenMs = std::max(a.frozenMs, (int)(since - lastChange[axis]));
					havePos[axis] = true;
					lastPos[axis] = *p;
					lastChange[axis] = since;
				}

				if (since >= kFollowMs) {
					followed[axis] = true;
					const auto c = mc.get_command_mm(m_motionID, 0, axis);
					a.cmdErr = c.value_or(a.target) - a.target;
				}
			}
			if (finished) return true;
			if (ms > kMoveTimeoutMs + kFollowMs) { why = QStringLiteral("a move did not finish within %1 s").arg(kMoveTimeoutMs / 1000); return false; }
			os_tool::goSleep(kFollowPeriodMs);
		}
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
	out << "  Part C          : " << (randomSeconds > 0
		? QStringLiteral("random X/Y/Z moves for %1 s%2").arg(randomSeconds)
			.arg(randomSlowPercent > 0 ? QStringLiteral(", then %1 s at %2 % speed").arg(std::max(60, randomSeconds / 2)).arg(randomSlowPercent) : QString())
		: QStringLiteral("no")) << "\n";
	out << "  Part E          : " << (pairSeconds > 0 ? QStringLiteral("X+Y, X+Z, Y+Z together, the third still, %1 s each").arg(pairSeconds) : QStringLiteral("no")) << "\n";
	out << "  Run order       : A, D, E, C, B. Every target stays in the middle 98 % of the soft limits.\n";
	out << "  Part D          : " << (travelAxesMask != 0 && travelBlockSeconds > 0
		? QStringLiteral("short vs long moves, %1%2%3one at a time, %4 s per block")
			.arg(travelAxesMask & 1 ? "X " : "").arg(travelAxesMask & 2 ? "Y " : "").arg(travelAxesMask & 4 ? "Z " : "").arg(travelBlockSeconds)
		: QStringLiteral("no")) << "\n";
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
				if (!testRange(axis, lo, hi) || farEnd < lo || farEnd > hi || p0 < lo || p0 > hi) {
					out << "  " << name << ": SKIPPED - " << f3(p0) << " .. " << f3(farEnd) << " is not inside the middle 98 % of the soft limits\n";
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
	/*
	* Random X/Y/Z moves for `seconds` at `pct` % of the speeds set now. A lambda because the full
	* test runs it twice (100 % and slower); it is called after Parts D and E, so that the parts
	* moving fewer axes run while X is still clean.
	*/
	bool rcHeader = false;
	auto runRandom = [&](int pct, int seconds) {
		const QString tag = pct == 100 ? QStringLiteral("Part C") : QStringLiteral("Part C %1 %").arg(pct);
		section(QStringLiteral("PART C - RANDOM MOVES, ALL THREE AXES, %1 % SPEED").arg(pct));
		out << "  X, Y and Z go together (as jog() sends them) to random points inside the middle 98 % of the soft limits,\n"
			<< "  at " << pct << " % of the speeds set now, for " << seconds << " s. From each axis's own \"move done\" its encoder is read every\n"
			<< "  few ms for " << kFollowMs / 1000 << " s.\n"
			<< "  Error = encoder - target. Settled = within the tolerance from then on, to the end of that second.\n"
			<< "  Frozen = the position read stood still while more than " << f3(kSettleWideMm) << " mm off, then moved again\n"
			<< "  (the axis paused, or the reading was stale).\n";

		QString why;
		AxisNow start[3];
		double lo[3] = {}, hi[3] = {};
		double normal[3] = {};
		if (stoppedByUser) why = QStringLiteral("the test was stopped earlier");
		else if (!mc.motionEnabled()) why = QStringLiteral("motion is disabled - home the machine or clear the active error");
		else if (MachineController::instance().trolleyAttached())
			why = QStringLiteral("a trolley is attached - Z goes down to its lowest soft limit, so this runs only with no trolley");
		for (int axis = 0; axis < 3 && why.isEmpty(); axis++) {
			const QString name = kAxisNames[axis];
			start[axis] = readAxis(axis);
			const auto speed = mc.get_move_max_velocity_mm(m_motionID, axis);
			if (!start[axis].ok) why = QStringLiteral("the %1 position is unreadable").arg(name);
			else if (std::abs(start[axis].enc - start[axis].cmd) > kGapWarnMm)
				why = QStringLiteral("%1 command and encoder differ by %2 mm - press Reset (it re-syncs them) and run again").arg(name).arg(f3(start[axis].enc - start[axis].cmd));
			else if (!testRange(axis, lo[axis], hi[axis]))
				why = QStringLiteral("the %1 soft limits are unreadable").arg(name);
			else if (pct != 100 && (!speed || *speed <= 0.0))
				why = QStringLiteral("the %1 speed cannot be read back to restore after the slow pass").arg(name);
			else normal[axis] = speed.value_or(0.0);
		}

		if (!why.isEmpty()) {
			out << "  NOT RUN: " << why << ".\n";
			verdicts << QStringLiteral("%1 not run: %2").arg(tag).arg(why);
		}
		else {
			if (pct != 100)
				for (int axis = 0; axis < 3; axis++) mc.set_move_max_velocity(m_motionID, axis, normal[axis] * pct / 100.0);

			const unsigned seed = (unsigned)QDateTime::currentMSecsSinceEpoch();
			std::mt19937 rng(seed);
			std::uniform_real_distribution<double> pick[3];
			for (int axis = 0; axis < 3; axis++) {
				pick[axis] = std::uniform_real_distribution<double>(lo[axis], hi[axis]);
				const auto speed = mc.get_move_max_velocity_mm(m_motionID, axis);
				out << "  " << kAxisNames[axis] << ": " << f3(lo[axis]) << " .. " << f3(hi[axis]) << " mm, speed "
					<< (speed ? f3(*speed) + QStringLiteral(" mm/s") : QStringLiteral("unknown")) << ", start " << f3(start[axis].enc) << "\n";
			}
			out << "  Random seed: " << seed << "\n";
			if (!rcHeader) {
				rc << "speed_pct,move,t_s,axis,from_mm,target_mm,travel_mm,move_ms,inp_at_done,err_0,err_100,err_250,err_500,err_1000,err_2000,"
					<< "settled_0.05_ms,settled_0.01_ms,frozen_ms,command_minus_target\n";
				rcHeader = true;
			}
			say(QStringLiteral("%1: random X/Y/Z moves for %2 s - Z goes down to its lowest soft limit").arg(tag).arg(seconds));

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
			while (total.elapsed() < (qint64)seconds * 1000) {
				if (m_stopRun) { cAborted = true; cWhy = QStringLiteral("stopped"); break; }
				move++;
				double target[3];
				for (int axis = 0; axis < 3; axis++) target[axis] = std::round(pick[axis](rng) * 1000.0) / 1000.0;

				const double sentSec = total.elapsed() / 1000.0;
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
				qint64 doneAt[3];
				for (int axis = 0; axis < 3; axis++) {
					L[axis].move = move; L[axis].axis = axis; L[axis].tSec = sentSec; L[axis].from = from[axis]; L[axis].target = target[axis];
				}
				const bool all3[3] = { true, true, true };
				if (!followLandings(all3, L, doneAt, cWhy)) { cAborted = true; stopAll(); break; }

				moveMs.push_back((double)*std::max_element(doneAt, doneAt + 3));
				for (int axis = 0; axis < 3; axis++) {
					const auto& a = L[axis];
					rc << pct << "," << move << "," << QString::number(a.tSec, 'f', 1) << "," << kAxisNames[axis] << "," << f4(a.from) << "," << f4(a.target) << "," << f4(a.travel()) << ","
						<< doneAt[axis] << "," << (a.inpAtDone ? 1 : 0);
					for (int k = 0; k < kFollowMarks; k++) rc << "," << f4(a.err[k]);
					rc << "," << a.settledWideMs << "," << a.settledTightMs << "," << a.frozenMs << "," << f4(a.cmdErr) << "\n";
					rl.push_back(a);
					from[axis] = target[axis];
				}
				if (move % 10 == 0)
					say(QStringLiteral("%1: %2 moves, %3 s left").arg(tag).arg(move).arg(std::max<qint64>(0, seconds - total.elapsed() / 1000)));
			}
			const qint64 runMs = total.elapsed();

			//back where the operator left it - only after a clean finish: a refusal or timeout says the machine is not fit to move
			if (!cAborted) {
				say(QStringLiteral("%1: back to the start point").arg(tag));
				bool back = true;
				for (int axis : order) back = mc.absolute_move(m_motionID, axis, start[axis].enc) && back;
				os_tool::doNothing(m_motionReadDelay_ms);
				QElapsedTimer t;
				t.start();
				while (!allDone() && !m_stopRun && t.elapsed() < kMoveTimeoutMs) os_tool::goSleep(10);
				if (!allDone()) { stopAll(); back = false; }
				if (!back) verdicts << QStringLiteral("%1 could not return to the start point").arg(tag);
			}
			//the speeds go back whatever happened in the pass
			if (pct != 100)
				for (int axis = 0; axis < 3; axis++) mc.set_move_max_velocity(m_motionID, axis, normal[axis]);

			//-------------------------------------------------------- analysis
			const int n = (int)rl.size() / 3;
			out << "\n  Moves: " << n << " in " << f3(runMs / 1000.0) << " s";
			if (!moveMs.empty())
				out << ", move time (sent -> last axis done) median " << median(moveMs) << " ms, longest " << *std::max_element(moveMs.begin(), moveMs.end()) << " ms";
			out << "\n";

			if (n > 0) {
				out << "\n  |error| from each axis's own \"move done\", mm - mean / max\n";
				out << "  axis     n   at done          0.1 s            0.25 s           0.5 s            1 s              2 s              INP@done\n";
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

				out << "\n  Settle time from \"move done\", ms - median / max (landings not settled within " << kFollowMs / 1000 << " s)\n";
				out << "  axis   within " << f3(kSettleWideMm) << "              within " << f3(kSettleTightMm) << "              frozen >= " << kFrozenWarnMs
					<< " ms   command off target (> 1.5 motor steps)\n";
				for (int axis = 0; axis < 3; axis++) {
					std::vector<double> wide, tight;
					int wideMiss = 0, tightMiss = 0, frozen = 0, frozenMax = 0, cmdOff = 0;
					//the card can only hold whole pulses, so a target lands up to a step away in the command counter
					const double ppm = mc.pulse_per_mm(m_motionID, axis);
					const double cmdTol = ppm > 1.0 ? 1.5 / ppm : 0.0015;
					for (const auto& a : rl) {
						if (a.axis != axis) continue;
						if (a.settledWideMs >= 0) wide.push_back(a.settledWideMs); else wideMiss++;
						if (a.settledTightMs >= 0) tight.push_back(a.settledTightMs); else tightMiss++;
						if (a.frozenMs >= kFrozenWarnMs) frozen++;
						frozenMax = std::max(frozenMax, a.frozenMs);
						if (std::abs(a.cmdErr) > cmdTol) cmdOff++;
					}
					auto cell = [&](const std::vector<double>& v, int miss) {
						const QString s = v.empty() ? QStringLiteral("-") : QStringLiteral("%1 / %2").arg(median(v)).arg(*std::max_element(v.begin(), v.end()));
						return (s + QStringLiteral(" (%1)").arg(miss)).leftJustified(23);
					};
					out << "  " << kAxisNames[axis] << "      " << cell(wide, wideMiss) << cell(tight, tightMiss)
						<< (QString::number(frozen) + QStringLiteral(" (longest %1 ms)").arg(frozenMax)).leftJustified(23) << cmdOff << "\n";

					const QString name = kAxisNames[axis];
					if (wideMiss > 0) verdicts << QStringLiteral("%1 %2: %3 of %4 random landings not within %5 mm %6 s after done").arg(tag).arg(name).arg(wideMiss).arg(n).arg(f3(kSettleWideMm)).arg(kFollowMs / 1000);
					if (frozen > 0) verdicts << QStringLiteral("%1 %2: position read stood still up to %3 ms while > %4 mm off (%5 landings)").arg(tag).arg(name).arg(frozenMax).arg(f3(kSettleWideMm)).arg(frozen);
					if (cmdOff > 0) verdicts << QStringLiteral("%1 %2: command counter not at the target after %3 landings").arg(tag).arg(name).arg(cmdOff);
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

				/*
				* Final error over time: on 2026-10-28 X's resting point moved from 0 to about -0.017 mm
				* some 40 s into the run. Minute by minute shows whether that keeps growing, levels off
				* or comes back. Signed mean, with the largest |error| of that minute.
				*/
				const int last = kFollowMarks - 1;
				out << "\n  Final error (" << kFollowMs / 1000 << " s after done) by minute of the run, mm - signed mean [largest |error|]\n";
				out << "  minute  moves   X                     Y                     Z\n";
				const int minutes = (int)(rl.back().tSec / 60.0) + 1;
				for (int m = 0; m < minutes; m++) {
					double sum[3] = {}, mx[3] = {};
					int cnt[3] = {};
					for (const auto& a : rl) {
						if ((int)(a.tSec / 60.0) != m) continue;
						sum[a.axis] += a.err[last];
						mx[a.axis] = std::max(mx[a.axis], std::abs(a.err[last]));
						cnt[a.axis]++;
					}
					if (cnt[0] == 0) continue;
					out << "  " << QString::number(m + 1).rightJustified(6) << "  " << QString::number(cnt[0]).rightJustified(5) << "   ";
					for (int axis = 0; axis < 3; axis++)
						out << (sgn(sum[axis] / cnt[axis]) + QStringLiteral(" [") + f3(mx[axis]) + QStringLiteral("]")).leftJustified(22);
					out << "\n";
				}
				for (int axis = 0; axis < 3; axis++) {
					double mx = 0.0;
					for (const auto& a : rl) if (a.axis == axis) mx = std::max(mx, std::abs(a.err[last]));
					out << "  " << kAxisNames[axis] << " largest final error: " << f3(mx) << " mm\n";
					if (mx > kSettleTightMm) verdicts << QStringLiteral("%1 %2: up to %3 mm off %4 s after done").arg(tag).arg(kAxisNames[axis]).arg(f3(mx)).arg(kFollowMs / 1000);
				}
			}

			if (cAborted) {
				out << "\n  PART C ENDED EARLY: " << cWhy << "\n";
				verdicts << QStringLiteral("%1 ended early: %2").arg(tag).arg(cWhy);
				if (cWhy == QStringLiteral("stopped")) stoppedByUser = true;
			}
		}
	};

	//================================================================== PART D
	/*
	* The random runs left X holding 0.02-0.15 mm off target, in steps that each came with one
	* move. This asks whether short or long moves bring them, one axis at a time with the other
	* two standing still: blocks of SHORT moves, LONG moves, SHORT again, and a count of the
	* steps in the settled error per block.
	*/
	if (travelAxesMask != 0 && travelBlockSeconds > 0) {
		section(QStringLiteral("PART D - SHORT vs LONG MOVES, ONE AXIS AT A TIME"));
		out << "  Each ticked axis on its own, the other two standing still, in three blocks of " << travelBlockSeconds << " s:\n"
			<< "  SHORT moves (" << kShortMinMm << " to " << kShortMaxMm << " mm), LONG moves (at least " << (int)(kLongFraction * 100)
			<< " % of the axis's travel), SHORT again. Speeds as set now.\n"
			<< "  SHORT moves sweep the whole travel: each one carries on the same way, turning round at each end, so every\n"
			<< "  region is visited (one full sweep of X or Y takes about 3 min). LONG moves land anywhere in the travel.\n"
			<< "  Every landing is followed for " << kFollowMs / 1000 << " s; settled error = encoder - target at the end of that.\n"
			<< "  STEP = the settled error changed by " << f3(kStepMm) << " mm or more since the previous landing. Steps are how the\n"
			<< "  offset built up in the random runs, so the block they fall in shows which moves cause it.\n"
			<< "  HELD = the two axes standing still, encoder - command, read just before each move is sent and again after\n"
			<< "  its follow. A change of " << f3(kStepMm) << " mm or more is a step on THAT axis, caused by the one moving.\n";

		QString why;
		if (stoppedByUser) why = QStringLiteral("the test was stopped earlier");
		else if (!mc.motionEnabled()) why = QStringLiteral("motion is disabled - home the machine or clear the active error");
		else if (MachineController::instance().trolleyAttached())
			why = QStringLiteral("a trolley is attached - Z can go down to its lowest soft limit and X/Y move at the current Z, so this runs only with no trolley");

		if (!why.isEmpty()) {
			out << "  NOT RUN: " << why << ".\n";
			verdicts << QStringLiteral("Part D not run: %1").arg(why);
		}
		else {
			const unsigned seed = (unsigned)QDateTime::currentMSecsSinceEpoch();
			std::mt19937 rng(seed);
			out << "  Random seed: " << seed << "\n";
			tc << "axis,block,kind,move,t_s,from_mm,target_mm,travel_mm,direction,move_ms,inp_at_done,err_0,err_100,err_250,err_500,err_1000,err_2000,"
				<< "settled_0.05_ms,settled_0.01_ms,frozen_ms,step_mm,X_held_mm,Y_held_mm,Z_held_mm,X_held_shift_mm,Y_held_shift_mm,Z_held_shift_mm\n";

			std::vector<TravelLanding> tl;
			double testLo[3] = {}, testHi[3] = {}; //each tested axis's travel, for the by-position table
			bool dAborted = false;
			QString dWhy;
			for (int axis = 0; axis < 3 && !dAborted; axis++) {
				if (!(travelAxesMask & (1 << axis))) continue;
				const QString name = kAxisNames[axis];

				const auto start = readAxis(axis);
				double lo = 0.0, hi = 0.0;
				QString skip;
				if (!start.ok) skip = QStringLiteral("position unreadable");
				else if (std::abs(start.enc - start.cmd) > kGapWarnMm)
					skip = QStringLiteral("command and encoder differ by %1 mm - press Reset (it re-syncs them) and run again").arg(f3(start.enc - start.cmd));
				else if (!testRange(axis, lo, hi) || hi - lo <= 4 * kShortMaxMm)
					skip = QStringLiteral("soft limits unreadable or too close together");
				else if (start.enc < lo || start.enc > hi)
					skip = QStringLiteral("it starts outside %1 .. %2 mm - jog it inside and run again").arg(f3(lo)).arg(f3(hi));
				if (!skip.isEmpty()) {
					out << "  " << name << ": SKIPPED - " << skip << "\n";
					verdicts << QStringLiteral("Part D %1 skipped: %2").arg(name).arg(skip);
					continue;
				}

				testLo[axis] = lo;
				testHi[axis] = hi;
				const double longMin = kLongFraction * (hi - lo);
				const auto speed = mc.get_move_max_velocity_mm(m_motionID, axis);
				out << "  " << name << ": " << f3(lo) << " .. " << f3(hi) << " mm, long moves at least " << f3(longMin) << " mm, speed "
					<< (speed ? f3(*speed) + QStringLiteral(" mm/s") : QStringLiteral("unknown")) << ", start " << f3(start.enc) << "\n";
				say(QStringLiteral("Part D: %1 alone - short, long, short moves, %2 s each").arg(name).arg(travelBlockSeconds));

				bool moving[3] = { false, false, false };
				moving[axis] = true;
				double pos = start.enc;
				double prevSettled = start.enc - start.cmd; //where it stood before the first move
				int move = 0;
				QElapsedTimer total;
				total.start();
				int sweepDir = 1;
				for (int block = 0; block < 3 && !dAborted; block++) {
					const bool isLong = (block == 1);
					//a short block sweeps towards the far end first, so it covers the most ground
					if (!isLong) sweepDir = (hi - pos) >= (pos - lo) ? 1 : -1;
					QElapsedTimer bt;
					bt.start();
					while (bt.elapsed() < (qint64)travelBlockSeconds * 1000) {
						if (m_stopRun) { dAborted = true; dWhy = QStringLiteral("stopped"); break; }

						double target;
						if (!isLong) {
							//carry on the same way, turning round at the end of the travel
							const double d = std::uniform_real_distribution<double>(kShortMinMm, kShortMaxMm)(rng);
							target = pos + sweepDir * d;
							if (target > hi || target < lo) {
								sweepDir = -sweepDir;
								target = pos + sweepDir * d;
							}
						}
						else {
							//anywhere at least longMin away: the stretch above pos and the one below, picked by length
							const double upLen = std::max(0.0, hi - (pos + longMin));
							const double downLen = std::max(0.0, (pos - longMin) - lo);
							const double u = std::uniform_real_distribution<double>(0.0, upLen + downLen)(rng);
							target = u < upLen ? pos + longMin + u : pos - longMin - (u - upLen);
						}
						target = std::min(hi, std::max(lo, std::round(target * 1000.0) / 1000.0));

						//where the still axes are holding just before this move
						bool heldBeforeOk[3] = {};
						double heldBefore[3] = {};
						for (int o = 0; o < 3; o++) {
							if (o == axis) continue;
							const auto h = readAxis(o);
							heldBeforeOk[o] = h.ok;
							heldBefore[o] = h.enc - h.cmd;
						}

						move++;
						const double sentSec = total.elapsed() / 1000.0;
						if (!mc.absolute_move(m_motionID, axis, target)) {
							dAborted = true;
							dWhy = QStringLiteral("the %1 move to %2 was refused").arg(name).arg(f3(target));
							break;
						}
						os_tool::doNothing(m_motionReadDelay_ms);

						RandomLanding L[3];
						L[axis].move = move; L[axis].axis = axis; L[axis].tSec = sentSec; L[axis].from = pos; L[axis].target = target;
						qint64 doneAt[3];
						if (!followLandings(moving, L, doneAt, dWhy)) { dAborted = true; mc.stop_move(m_motionID, axis); break; }

						TravelLanding T;
						T.block = block;
						T.L = L[axis];
						T.settled = L[axis].err[kFollowMarks - 1];
						T.step = T.settled - prevSettled;
						prevSettled = T.settled;
						for (int o = 0; o < 3; o++) {
							if (o == axis || !heldBeforeOk[o]) continue;
							const auto h = readAxis(o);
							if (!h.ok) continue;
							T.heldOk[o] = true;
							T.held[o] = h.enc - h.cmd;
							T.heldShift[o] = T.held[o] - heldBefore[o];
						}
						tl.push_back(T);

						const auto& a = T.L;
						tc << name << "," << (block + 1) << "," << kBlockNames[block] << "," << move << "," << QString::number(a.tSec, 'f', 1) << ","
							<< f4(a.from) << "," << f4(a.target) << "," << f4(a.travel()) << "," << (a.target > a.from ? "+" : "-") << ","
							<< doneAt[axis] << "," << (a.inpAtDone ? 1 : 0);
						for (int k = 0; k < kFollowMarks; k++) tc << "," << f4(a.err[k]);
						tc << "," << a.settledWideMs << "," << a.settledTightMs << "," << a.frozenMs << "," << f4(T.step);
						for (int o = 0; o < 3; o++) tc << "," << (T.heldOk[o] ? f4(T.held[o]) : QString());
						for (int o = 0; o < 3; o++) tc << "," << (T.heldOk[o] ? f4(T.heldShift[o]) : QString());
						tc << "\n";

						pos = target;
						if (move % 10 == 0)
							say(QStringLiteral("Part D: %1 %2 block, %3 moves, settled %4 mm").arg(name).arg(kBlockNames[block]).arg(move).arg(sgn(T.settled)));
					}
				}

				//back where it started - only after a clean finish, as Part C
				if (!dAborted) {
					bool back = mc.absolute_move(m_motionID, axis, start.enc);
					os_tool::doNothing(m_motionReadDelay_ms);
					QElapsedTimer t;
					t.start();
					while (back && !mc.move_done(m_motionID, 0, axis) && !m_stopRun && t.elapsed() < kMoveTimeoutMs) os_tool::goSleep(10);
					if (!back || !mc.move_done(m_motionID, 0, axis)) {
						mc.stop_move(m_motionID, axis);
						verdicts << QStringLiteral("Part D: %1 could not return to its start point").arg(name);
					}
				}
			}

			//-------------------------------------------------------- analysis
			for (int axis = 0; axis < 3; axis++) {
				std::vector<const TravelLanding*> mine;
				for (const auto& T : tl) if (T.L.axis == axis) mine.push_back(&T);
				if (mine.empty()) continue;
				const QString name = kAxisNames[axis];

				out << "\n  " << name << "  block  kind    moves  travel mm (min-max)    settled error start -> end    steps   sum of steps   largest step\n";
				int steps[2] = { 0, 0 };           //[0] short blocks, [1] long block
				double stepSum[2] = { 0.0, 0.0 };
				for (int block = 0; block < 3; block++) {
					std::vector<const TravelLanding*> set;
					for (auto* T : mine) if (T->block == block) set.push_back(T);
					if (set.empty()) continue;
					double tmin = set[0]->L.travel(), tmax = tmin, sum = 0.0, largest = 0.0;
					int count = 0;
					for (auto* T : set) {
						tmin = std::min(tmin, T->L.travel());
						tmax = std::max(tmax, T->L.travel());
						if (std::abs(T->step) >= kStepMm) {
							count++;
							sum += T->step;
							if (std::abs(T->step) > std::abs(largest)) largest = T->step;
						}
					}
					const double startErr = set.front()->settled - set.front()->step;
					out << "     " << QString::number(block + 1).rightJustified(5) << "  " << QString(kBlockNames[block]).leftJustified(6)
						<< QString::number(set.size()).rightJustified(7) << "  " << (f3(tmin) + QStringLiteral(" - ") + f3(tmax)).leftJustified(21)
						<< (sgn(startErr) + QStringLiteral(" -> ") + sgn(set.back()->settled)).leftJustified(30)
						<< QString::number(count).rightJustified(5) << "   " << sgn(sum).leftJustified(15) << (count ? sgn(largest) : QStringLiteral("-")) << "\n";
					const int kind = block == 1 ? 1 : 0;
					steps[kind] += count;
					stepSum[kind] += sum;
				}

				QString verdict;
				if (steps[0] == 0 && steps[1] == 0) verdict = QStringLiteral("no steps from short or long moves");
				else if (steps[0] == 0) verdict = QStringLiteral("steps on LONG moves only (%1, total %2 mm)").arg(steps[1]).arg(sgn(stepSum[1]));
				else if (steps[1] == 0) verdict = QStringLiteral("steps on SHORT moves only (%1, total %2 mm)").arg(steps[0]).arg(sgn(stepSum[0]));
				else verdict = QStringLiteral("steps on BOTH - short %1 (total %2 mm), long %3 (total %4 mm)")
					.arg(steps[0]).arg(sgn(stepSum[0])).arg(steps[1]).arg(sgn(stepSum[1]));
				out << "  " << name << " verdict: " << verdict << "\n";
				verdicts << QStringLiteral("Part D %1: %2").arg(name).arg(verdict);

				//every step, so each can be matched to its move in travel_moves.csv
				int listed = 0;
				for (auto* T : mine) {
					if (std::abs(T->step) < kStepMm) continue;
					if (listed++ == 0) out << "  " << name << " steps:\n";
					if (listed > 40) { out << "    ... more in travel_moves.csv\n"; break; }
					out << "    move " << QString::number(T->L.move).rightJustified(4) << "  " << QString(kBlockNames[T->block]).leftJustified(6)
						<< f3(T->L.from) << " -> " << f3(T->L.target) << " (" << f3(T->L.travel()) << " mm " << (T->L.target > T->L.from ? "+" : "-") << ")   settled "
						<< sgn(T->settled - T->step) << " -> " << sgn(T->settled) << "\n";
				}

				//by position: accurate everywhere along the axis, or only in some stretches?
				out << "\n  " << name << " by position (landing target)   kind    region mm              landings   settled mean   largest |err|   steps";
				for (int o = 0; o < 3; o++) if (o != axis) out << "   " << kAxisNames[o] << " held steps";
				out << "\n";
				for (int kind = 0; kind < 2; kind++) {
					for (int r = 0; r < kRegions; r++) {
						const double span = testHi[axis] - testLo[axis];
						const double r0 = testLo[axis] + span * r / kRegions;
						const double r1 = testLo[axis] + span * (r + 1) / kRegions;
						std::vector<const TravelLanding*> set;
						for (auto* T : mine) {
							if ((T->block == 1) != (kind == 1)) continue;
							const double x = T->L.target;
							if (x >= r0 && (x < r1 || (r == kRegions - 1 && x <= r1))) set.push_back(T);
						}
						double sum = 0.0, largest = 0.0;
						int count = 0;
						for (auto* T : set) {
							sum += T->settled;
							largest = std::max(largest, std::abs(T->settled));
							if (std::abs(T->step) >= kStepMm) count++;
						}
						out << "                                    " << (kind == 1 ? "LONG  " : "SHORT ") << "  "
							<< (f3(r0) + QStringLiteral(" .. ") + f3(r1)).leftJustified(23) << QString::number(set.size()).rightJustified(8) << "   "
							<< (set.empty() ? QStringLiteral("-") : sgn(sum / set.size())).leftJustified(15)
							<< (set.empty() ? QStringLiteral("-") : f3(largest)).leftJustified(16) << QString::number(count).rightJustified(5);
						for (int o = 0; o < 3; o++) {
							if (o == axis) continue;
							int held = 0;
							for (auto* T : set) if (T->heldOk[o] && std::abs(T->heldShift[o]) >= kStepMm) held++;
							out << QString::number(held).rightJustified(15);
						}
						out << "\n";
					}
				}

				//the axes standing still while this one moved: did its moves push them?
				for (int o = 0; o < 3; o++) {
					if (o == axis) continue;
					std::vector<const TravelLanding*> seen;
					for (auto* T : mine) if (T->heldOk[o]) seen.push_back(T);
					if (seen.empty()) continue;
					const QString held = kAxisNames[o];

					out << "\n  " << held << " HELD while " << name << " moved   block  kind    held start -> end            steps   sum of steps   largest step\n";
					int total = 0;
					double totalSum = 0.0;
					for (int block = 0; block < 3; block++) {
						std::vector<const TravelLanding*> set;
						for (auto* T : seen) if (T->block == block) set.push_back(T);
						if (set.empty()) continue;
						int count = 0;
						double sum = 0.0, largest = 0.0;
						for (auto* T : set) {
							if (std::abs(T->heldShift[o]) < kStepMm) continue;
							count++;
							sum += T->heldShift[o];
							if (std::abs(T->heldShift[o]) > std::abs(largest)) largest = T->heldShift[o];
						}
						const double heldStart = set.front()->held[o] - set.front()->heldShift[o];
						out << "                            " << QString::number(block + 1).rightJustified(5) << "  " << QString(kBlockNames[block]).leftJustified(6)
							<< "  " << (sgn(heldStart) + QStringLiteral(" -> ") + sgn(set.back()->held[o])).leftJustified(28)
							<< QString::number(count).rightJustified(5) << "   " << sgn(sum).leftJustified(15) << (count ? sgn(largest) : QStringLiteral("-")) << "\n";
						total += count;
						totalSum += sum;
					}

					//by the moving axis's direction: in the random runs X's steps followed Z going up or down
					for (bool positive : { true, false }) {
						double sum = 0.0;
						int n = 0, up = 0, down = 0;
						for (auto* T : seen) {
							if ((T->L.target > T->L.from) != positive) continue;
							n++;
							sum += T->heldShift[o];
							if (T->heldShift[o] >= kStepMm) up++;
							else if (T->heldShift[o] <= -kStepMm) down++;
						}
						if (n == 0) continue;
						out << "    while " << name << " moved " << (positive ? "+" : "-")
							<< (axis == (int)Axis::Z ? (positive ? " (down)" : " (up)  ") : "") << ": " << n << " moves, mean change "
							<< (sum / n >= 0.0 ? "+" : "") << QString::number(sum / n * 1000.0, 'f', 2) << " um, steps up " << up << ", down " << down << "\n";
					}

					const QString heldVerdict = total == 0
						? QStringLiteral("%1 held still while %2 moved alone (no steps)").arg(held).arg(name)
						: QStringLiteral("%1 SHIFTED while %2 moved alone: %3 steps, total %4 mm").arg(held).arg(name).arg(total).arg(sgn(totalSum));
					out << "  " << heldVerdict << "\n";
					verdicts << QStringLiteral("Part D: %1").arg(heldVerdict);
				}
			}

			if (dAborted) {
				out << "\n  PART D ENDED EARLY: " << dWhy << "\n";
				verdicts << QStringLiteral("Part D ended early: %1").arg(dWhy);
				if (dWhy == QStringLiteral("stopped")) stoppedByUser = true;
			}
		}
	}

	//================================================================== PART E
	/*
	* Two axes moving together with the third standing still and watched: X+Y, X+Z, Y+Z. Each axis
	* alone lands exactly (Part D, 28 Oct 2026) while all three together let X's settled error step,
	* the steps following Z's direction - this shows which partner does it.
	*/
	if (pairSeconds > 0) {
		section(QStringLiteral("PART E - TWO AXES TOGETHER, THE THIRD STANDING STILL"));
		out << "  X+Y, then X+Z, then Y+Z, " << pairSeconds << " s each. The two go together (as jog() sends them) to random points in\n"
			<< "  the middle 98 % of their soft limits at the speeds set now; every landing is followed for " << kFollowMs / 1000 << " s. The third\n"
			<< "  axis is read (encoder - command) before and after every move.\n"
			<< "  STEP = a settled error, or the still axis's reading, that changed by " << f3(kStepMm) << " mm or more with one move.\n"
			<< "  Offsets carry over from part to part until the next home, so each pair's start value is shown.\n";

		QString why;
		if (stoppedByUser) why = QStringLiteral("the test was stopped earlier");
		else if (!mc.motionEnabled()) why = QStringLiteral("motion is disabled - home the machine or clear the active error");
		else if (MachineController::instance().trolleyAttached())
			why = QStringLiteral("a trolley is attached - Z goes down to its lowest soft limit, so this runs only with no trolley");

		if (!why.isEmpty()) {
			out << "  NOT RUN: " << why << ".\n";
			verdicts << QStringLiteral("Part E not run: %1").arg(why);
		}
		else {
			const unsigned seed = (unsigned)QDateTime::currentMSecsSinceEpoch();
			std::mt19937 rng(seed);
			out << "  Random seed: " << seed << "\n";
			ec << "pair,move,t_s,axis,role,from_mm,target_mm,travel_mm,direction,settled_mm,step_mm\n";

			struct PairMove {
				bool ok[3] = {};
				double settled[3] = {};   //moving axis: error kFollowMs after done; still axis: encoder - command after the move
				double step[3] = {};
				int dir[3] = {};          //+1 / -1 for a moving axis, 0 for the still one
			};
			const int pairs[3][2] = { { (int)Axis::X, (int)Axis::Y }, { (int)Axis::X, (int)Axis::Z }, { (int)Axis::Y, (int)Axis::Z } };
			const int order[3] = { (int)Axis::Z, (int)Axis::X, (int)Axis::Y }; //as jog() sends them
			const int last = kFollowMarks - 1;
			bool eAborted = false;
			QString eWhy;

			for (int p = 0; p < 3 && !eAborted; p++) {
				bool moving[3] = { false, false, false };
				moving[pairs[p][0]] = true;
				moving[pairs[p][1]] = true;
				const int still = 3 - pairs[p][0] - pairs[p][1];
				const QString pairName = QStringLiteral("%1+%2").arg(kAxisNames[pairs[p][0]]).arg(kAxisNames[pairs[p][1]]);
				auto stopPair = [&]() { for (int axis = 0; axis < 3; axis++) if (moving[axis]) mc.stop_move(m_motionID, axis); };

				AxisNow start[3];
				double lo[3] = {}, hi[3] = {};
				QString skip;
				for (int axis = 0; axis < 3 && skip.isEmpty(); axis++) {
					start[axis] = readAxis(axis);
					if (!start[axis].ok) skip = QStringLiteral("the %1 position is unreadable").arg(kAxisNames[axis]);
					else if (std::abs(start[axis].enc - start[axis].cmd) > kGapWarnMm)
						skip = QStringLiteral("%1 command and encoder differ by %2 mm").arg(kAxisNames[axis]).arg(f3(start[axis].enc - start[axis].cmd));
					else if (moving[axis] && !testRange(axis, lo[axis], hi[axis]))
						skip = QStringLiteral("the %1 soft limits are unreadable").arg(kAxisNames[axis]);
				}
				out << "\n  " << pairName << " together, " << kAxisNames[still] << " still";
				if (!skip.isEmpty()) {
					out << ": SKIPPED - " << skip << "\n";
					verdicts << QStringLiteral("Part E %1 skipped: %2").arg(pairName).arg(skip);
					continue;
				}
				out << " (start X " << f3(start[0].enc) << ", Y " << f3(start[1].enc) << ", Z " << f3(start[2].enc) << ")\n";
				say(QStringLiteral("Part E: %1 together, %2 still, %3 s").arg(pairName).arg(kAxisNames[still]).arg(pairSeconds));

				std::vector<PairMove> pm;
				double prev[3], from[3];
				for (int axis = 0; axis < 3; axis++) {
					prev[axis] = start[axis].enc - start[axis].cmd;
					from[axis] = start[axis].enc;
				}
				QElapsedTimer total;
				total.start();
				int move = 0;
				while (total.elapsed() < (qint64)pairSeconds * 1000) {
					if (m_stopRun) { eAborted = true; eWhy = QStringLiteral("stopped"); break; }
					double target[3];
					for (int axis = 0; axis < 3; axis++)
						target[axis] = moving[axis] ? std::round(std::uniform_real_distribution<double>(lo[axis], hi[axis])(rng) * 1000.0) / 1000.0 : from[axis];

					const auto stillBefore = readAxis(still);
					move++;
					const double sentSec = total.elapsed() / 1000.0;
					for (int axis : order) {
						if (!moving[axis] || mc.absolute_move(m_motionID, axis, target[axis])) continue;
						eAborted = true;
						eWhy = QStringLiteral("the %1 move to %2 was refused").arg(kAxisNames[axis]).arg(f3(target[axis]));
						break;
					}
					if (eAborted) { stopPair(); break; }
					os_tool::doNothing(m_motionReadDelay_ms);

					RandomLanding L[3];
					for (int axis = 0; axis < 3; axis++) {
						L[axis].move = move; L[axis].axis = axis; L[axis].tSec = sentSec; L[axis].from = from[axis]; L[axis].target = target[axis];
					}
					qint64 doneAt[3];
					if (!followLandings(moving, L, doneAt, eWhy)) { eAborted = true; stopPair(); break; }

					PairMove m;
					for (int axis = 0; axis < 3; axis++) {
						if (!moving[axis]) continue;
						m.ok[axis] = true;
						m.settled[axis] = L[axis].err[last];
						m.step[axis] = m.settled[axis] - prev[axis];
						prev[axis] = m.settled[axis];
						m.dir[axis] = target[axis] > from[axis] ? 1 : -1;
					}
					const auto stillAfter = readAxis(still);
					if (stillBefore.ok && stillAfter.ok) {
						m.ok[still] = true;
						m.settled[still] = stillAfter.enc - stillAfter.cmd;
						m.step[still] = m.settled[still] - (stillBefore.enc - stillBefore.cmd);
					}
					pm.push_back(m);

					for (int axis = 0; axis < 3; axis++) {
						ec << pairName << "," << move << "," << QString::number(sentSec, 'f', 1) << "," << kAxisNames[axis] << ","
							<< (moving[axis] ? "moving" : "still") << "," << f4(from[axis]) << "," << f4(target[axis]) << ","
							<< f4(std::abs(target[axis] - from[axis])) << "," << (moving[axis] ? (m.dir[axis] > 0 ? "+" : "-") : "") << ","
							<< (m.ok[axis] ? f4(m.settled[axis]) : QString()) << "," << (m.ok[axis] ? f4(m.step[axis]) : QString()) << "\n";
						if (moving[axis]) from[axis] = target[axis];
					}
					if (move % 10 == 0)
						say(QStringLiteral("Part E: %1, %2 moves, %3 s left").arg(pairName).arg(move).arg(std::max<qint64>(0, pairSeconds - total.elapsed() / 1000)));
				}

				//back where this pair started - only after a clean finish
				if (!eAborted) {
					bool back = true;
					for (int axis : order) if (moving[axis]) back = mc.absolute_move(m_motionID, axis, start[axis].enc) && back;
					os_tool::doNothing(m_motionReadDelay_ms);
					auto pairDone = [&]() {
						for (int axis = 0; axis < 3; axis++) if (moving[axis] && !mc.move_done(m_motionID, 0, axis)) return false;
						return true;
					};
					QElapsedTimer t;
					t.start();
					while (back && !pairDone() && !m_stopRun && t.elapsed() < kMoveTimeoutMs) os_tool::goSleep(10);
					if (!back || !pairDone()) { stopPair(); verdicts << QStringLiteral("Part E %1 could not return to its start point").arg(pairName); }
				}

				//-------------------------------------------------- this pair's analysis
				if (pm.empty()) continue;
				out << "    " << pm.size() << " moves\n";
				out << "    axis  role     start -> end              steps up / down   sum of steps   largest step\n";
				QStringList stepped;
				for (int axis = 0; axis < 3; axis++) {
					bool any = false;
					double first = 0.0, lastVal = 0.0, sum = 0.0, largest = 0.0;
					int up = 0, down = 0;
					for (const auto& m : pm) {
						if (!m.ok[axis]) continue;
						if (!any) { first = m.settled[axis] - m.step[axis]; any = true; }
						lastVal = m.settled[axis];
						if (std::abs(m.step[axis]) < kStepMm) continue;
						(m.step[axis] > 0 ? up : down)++;
						sum += m.step[axis];
						if (std::abs(m.step[axis]) > std::abs(largest)) largest = m.step[axis];
					}
					if (!any) continue;
					out << "    " << kAxisNames[axis] << "     " << (moving[axis] ? "moving " : "STILL  ") << "  "
						<< (sgn(first) + QStringLiteral(" -> ") + sgn(lastVal)).leftJustified(26)
						<< (QString::number(up) + QStringLiteral(" / ") + QString::number(down)).leftJustified(18)
						<< sgn(sum).leftJustified(15) << (up + down ? sgn(largest) : QStringLiteral("-")) << "\n";
					if (up + down) stepped << QStringLiteral("%1%2 %3 steps (total %4 mm)").arg(kAxisNames[axis])
						.arg(moving[axis] ? QString() : QStringLiteral(" (still)")).arg(up + down).arg(sgn(sum));
				}

				//each axis's change split by the direction of each OTHER moving axis - the random runs tied X to Z's direction
				for (int axis = 0; axis < 3; axis++) {
					for (int other = 0; other < 3; other++) {
						if (other == axis || !moving[other]) continue;
						for (int dir : { 1, -1 }) {
							double sum = 0.0;
							int n = 0, up = 0, down = 0;
							for (const auto& m : pm) {
								if (!m.ok[axis] || m.dir[other] != dir) continue;
								n++;
								sum += m.step[axis];
								if (m.step[axis] >= kStepMm) up++;
								else if (m.step[axis] <= -kStepMm) down++;
							}
							if (n == 0) continue;
							out << "    " << kAxisNames[axis] << " while " << kAxisNames[other] << " moved " << (dir > 0 ? "+" : "-")
								<< (other == (int)Axis::Z ? (dir > 0 ? " (down)" : " (up)  ") : "       ") << ": " << QString::number(n).rightJustified(3)
								<< " moves, mean change " << (sum / n >= 0.0 ? "+" : "") << QString::number(sum / n * 1000.0, 'f', 2)
								<< " um, steps up " << up << ", down " << down << "\n";
						}
					}
				}

				const QString pairVerdict = stepped.isEmpty() ? QStringLiteral("no steps on any axis") : stepped.join(QStringLiteral(", "));
				out << "    " << pairName << " verdict: " << pairVerdict << "\n";
				verdicts << QStringLiteral("Part E %1: %2").arg(pairName).arg(pairVerdict);
			}

			if (eAborted) {
				out << "\n  PART E ENDED EARLY: " << eWhy << "\n";
				verdicts << QStringLiteral("Part E ended early: %1").arg(eWhy);
				if (eWhy == QStringLiteral("stopped")) stoppedByUser = true;
			}
		}
	}

	//--------------------------------------------- Part C last: all three axes together, after the fewer-axis parts
	if (randomSeconds > 0) {
		runRandom(100, randomSeconds);
		if (randomSlowPercent > 0) runRandom(randomSlowPercent, std::max(60, randomSeconds / 2));
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
