// =============================================================================
//  External3DViewer.cpp
//  The external 3D display: ImageJ's Interactive 3D Surface Plot, launched and closed
//  from the V3 page. A port of Jimmy's Python/tkinter launcher - same macro, same plot
//  settings, same close-then-open behaviour - with the UI left to Pogo.
// =============================================================================

#include "External3DViewer.h"
#include "Logger.h"

#include <QDir>
#include <QFileInfo>
#include <QProcess>
#include <QStringList>
#include <algorithm>
#include <chrono>
#include <set>
#include <vector>
#include <opencv2/imgcodecs.hpp>

//last, so its min/max macros can never reach the headers above
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <tlhelp32.h>

namespace {

//where the Python launcher found ImageJ; the IPC has it in the same place
const char* kImageJExe = "C:/Program Files/ImageJ/ImageJ.exe";

//the plot window's title contains this whatever file it shows - compared lower-case
const char* kPlotTitle = "interactive 3d surface plot";

constexpr int kCloseTimeoutMs = 0;  //polite WM_CLOSE before terminating, as in the Python
constexpr int kPlotTimeoutMs = 20000;  //the Python waited 10 s; a JVM start plus a 64 MB map wants headroom
constexpr int kPollMs = 200;

/*
* Interactive 3D Surface Plot settings, copied verbatim from the Python launcher - this is
* the configuration that was tested.
*/
const char* kPlotSettings =
	"plotType=2 "           // 0=Dots, 1=Lines, 2=Mesh, 3=Filled, 4=Isolines
	"colorType=4 "          // 0=Original, 1=Grayscale, 2=Spectrum, 3=Fire,
	                        // 4=Thermal, 5=Gradient, 6=Blue, 7=Orange

	"grid=1024 "
	"smooth=1.0 "

	"perspective=0 "
	"light=0.2 "

	"drawAxes=1 "
	"drawLines=1 "
	"drawText=1 "
	"drawLegend=1 "

	"invertZ=0 "
	"isEqualxyzRatio=0 "

	"rotationX=45 "
	"rotationZ=45 "

	"scale=1.2 "
	"scaleZ=1.0 "

	"min=0 "
	"max=100 "

	"snapshot=0 "

	"backgroundColor=808080 "
	"lineColor=FFFFFF "

	"windowWidth=720 "
	"windowHeight=600";

//the Python build_macro_code(): open the map, plot it, then close the source image so only
//the plot is left
QString buildMacro(const QString& tiffPath)
{
	const QString path = QDir::fromNativeSeparators(tiffPath); //ImageJ macro strings take '/'
	return QStringLiteral(
		"\n"
		"open(\"%1\");\n"
		"sourceTitle = getTitle();\n"
		"\n"
		"run(\"Interactive 3D Surface Plot\", \"%2\");\n"
		"\n"
		"selectWindow(sourceTitle);\n"
		"close();\n").arg(path, QString::fromLatin1(kPlotSettings));
}

//the single form every path comparison here uses: '/' separators, lower case
QString normPath(const QString& p)
{
	return QDir::cleanPath(QDir::fromNativeSeparators(p)).toLower();
}

QString imageJDirPrefix()
{
	return normPath(QFileInfo(QString::fromLatin1(kImageJExe)).absolutePath()) + QLatin1Char('/');
}

QString processImagePath(DWORD pid)
{
	HANDLE h = OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, FALSE, pid);
	if (!h) return QString();

	wchar_t buf[4096];
	DWORD size = sizeof(buf) / sizeof(buf[0]);
	QString path;
	if (QueryFullProcessImageNameW(h, 0, buf, &size)) path = QString::fromWCharArray(buf, (int)size);
	CloseHandle(h);
	return path;
}

/*
* Every process whose executable lives under the ImageJ folder. That is ImageJ.exe AND the
* jre\bin\javaw.exe it starts (ImageJ.cfg on the laptop points there), whichever of the two
* turns out to own the windows - and it can never be another Java program, whose javaw lives
* somewhere else.
*/
std::set<DWORD> imageJProcesses()
{
	std::set<DWORD> pids;
	const QString prefix = imageJDirPrefix();

	HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
	if (snap == INVALID_HANDLE_VALUE) return pids;

	PROCESSENTRY32W pe;
	pe.dwSize = sizeof(pe);
	for (BOOL more = Process32FirstW(snap, &pe); more; more = Process32NextW(snap, &pe)) {
		if (pe.th32ProcessID == 0 || pe.th32ProcessID == GetCurrentProcessId()) continue;
		const QString path = processImagePath(pe.th32ProcessID);
		if (!path.isEmpty() && normPath(path).startsWith(prefix)) pids.insert(pe.th32ProcessID);
	}

	CloseHandle(snap);
	return pids;
}

//for a window of another process this reads the cached caption, so a hung JVM cannot stall it
QString windowTitle(HWND h)
{
	const int len = GetWindowTextLengthW(h);
	if (len <= 0) return QString();

	std::vector<wchar_t> buf(len + 1);
	const int n = GetWindowTextW(h, buf.data(), len + 1);
	return QString::fromWCharArray(buf.data(), n);
}

struct WinInfo {
	HWND hwnd;
	bool visible;
	QString title;
};

BOOL CALLBACK collectWindow(HWND hwnd, LPARAM lp)
{
	reinterpret_cast<std::vector<HWND>*>(lp)->push_back(hwnd);
	return TRUE;
}

//top-level windows owned by those processes, hidden ones included
std::vector<WinInfo> imageJWindows(const std::set<DWORD>& pids)
{
	std::vector<WinInfo> out;
	if (pids.empty()) return out;

	std::vector<HWND> all;
	EnumWindows(collectWindow, reinterpret_cast<LPARAM>(&all));

	for (HWND h : all) {
		DWORD pid = 0;
		GetWindowThreadProcessId(h, &pid);
		if (!pids.count(pid)) continue;
		out.push_back({ h, IsWindowVisible(h) != FALSE, windowTitle(h) });
	}
	return out;
}

bool isPlotWindow(const WinInfo& w)
{
	return w.title.toLower().contains(QLatin1String(kPlotTitle));
}

//sleeps in short slices; false once the viewer is being destroyed and the job must stop
bool pause(int ms, const std::atomic<bool>& abort)
{
	for (int t = 0; t < ms && !abort; t += 50)
		std::this_thread::sleep_for(std::chrono::milliseconds(std::min(50, ms - t)));
	return !abort;
}

/*
* The Python close_imagej_windows(): polite first, forceful after the timeout.
*
* WM_CLOSE goes to every TITLED ImageJ window, hidden ones included - after an open the
* main ImageJ window is hidden, and closing it is what makes ImageJ quit. Untitled windows
* are Java's own hidden helpers and are left alone. "Gone" is judged by the PROCESSES, not
* the windows: an empty window list can just mean it is still on its way out.
*
* `headline` is what the please-wait box says while this runs; nothing is said at all when
* no ImageJ is running, which is the common case for an open.
*
* Returns how many ImageJ processes there were, or -1 if aborted part-way.
*/
int closeImageJ(const std::atomic<bool>& abort, const std::function<void(const QString&)>& say,
	const QString& headline)
{
	const std::set<DWORD> pids = imageJProcesses();
	if (pids.empty()) return 0;

	say(headline);

	int posted = 0;
	for (const auto& w : imageJWindows(pids)) {
		if (w.title.isEmpty()) continue;
		PostMessageW(w.hwnd, WM_CLOSE, 0, 0);
		posted++;
	}
	ct::logger::info("[External3D] Closing ImageJ: %d process(es), WM_CLOSE sent to %d window(s)",
		(int)pids.size(), posted);

	//nothing to ask politely means nothing to wait for
	if (posted > 0) {
		for (int waited = 0; waited < kCloseTimeoutMs; waited += kPollMs) {
			if (imageJProcesses().empty()) return (int)pids.size();
			if (waited > 0 && waited % 1000 == 0)
				say(QStringLiteral("%1\n\nWaiting for ImageJ to quit... %2 s").arg(headline).arg(waited / 1000));
			if (!pause(kPollMs, abort)) return -1;
		}
	}

	//the Python's taskkill /F /T - but only for processes that really are ImageJ's
	say(QStringLiteral("%1\n\nImageJ did not quit by itself - forcing it to close...").arg(headline));
	for (DWORD pid : imageJProcesses()) {
		HANDLE h = OpenProcess(PROCESS_TERMINATE | SYNCHRONIZE, FALSE, pid);
		if (!h) {
			ct::logger::warn("[External3D] Cannot open ImageJ process %lu to terminate it", pid);
			continue;
		}
		TerminateProcess(h, 1);
		WaitForSingleObject(h, 2000);
		CloseHandle(h);
		ct::logger::warn("[External3D] ImageJ process %lu did not quit on WM_CLOSE - terminated", pid);
	}
	return (int)pids.size();
}

} // namespace

External3DViewer::External3DViewer(QObject* parent)
	: QObject(parent)
{
}

External3DViewer::~External3DViewer()
{
	//every wait inside a job checks this, so the join returns within a poll interval
	m_abort = true;
	if (m_worker.joinable()) m_worker.join();
}

QString External3DViewer::imageJExe()
{
	return QDir::toNativeSeparators(QString::fromLatin1(kImageJExe));
}

QString External3DViewer::tempTiffPath()
{
	return QDir::toNativeSeparators(QDir::tempPath() + QStringLiteral("/Pogo/external3d_height.tiff"));
}

bool External3DViewer::open(const cv::Mat& height16)
{
	const cv::Mat map = height16; //shares the caller's deep copy; nothing else holds it
	return start([this, map]() { return runOpen(map); });
}

bool External3DViewer::close()
{
	return start([this]() { return runClose(); });
}

void External3DViewer::say(const QString& text)
{
	if (!m_abort) emit progress(text);
}

bool External3DViewer::start(std::function<Result()> job)
{
	if (m_busy.exchange(true)) return false;

	//busy was clear, so the previous worker has already finished its job: this join is immediate
	if (m_worker.joinable()) m_worker.join();

	m_worker = std::thread([this, job]() {
		Result r{ false, QString() };
		try {
			r = job();
		}
		catch (const std::exception& e) {
			r = { false, QStringLiteral("External 3D display failed: %1").arg(QString::fromLocal8Bit(e.what())) };
		}

		//cleared BEFORE the signal: the GUI's handler re-reads busy() to re-enable the buttons,
		//and it can run the moment the signal is posted
		m_busy = false;
		if (!m_abort) emit finished(r.ok, r.message);
	});
	return true;
}

External3DViewer::Result External3DViewer::runOpen(const cv::Mat& height16)
{
	if (height16.empty() || height16.type() != CV_16UC1)
		return { false, QStringLiteral("The external 3D display needs a 16-bit height map.") };

	const QString exe = imageJExe();
	const QString tiff = tempTiffPath();

	//1. one plot at a time: whatever ImageJ is up now goes first
	const int closed = closeImageJ(m_abort, [this](const QString& text) { say(text); },
		QStringLiteral("Closing the previous 3D plot..."));
	if (closed < 0) return { false, QStringLiteral("Cancelled.") };

	/*
	* 2. The map, to the one fixed file. Overwritten every time, so it never accumulates.
	* Uncompressed on purpose: OpenCV defaults to LZW, and this file is written, read once
	* and thrown away - speed on both ends matters more than its size.
	*/
	if (!QDir().mkpath(QFileInfo(tiff).absolutePath()))
		return { false, QStringLiteral("Cannot create the folder for the temporary height map:\n%1").arg(tiff) };

	say(QStringLiteral("Saving the height map (%1 x %2 px)...").arg(height16.cols).arg(height16.rows));
	bool written = false;
	try {
		const std::vector<int> params = { cv::IMWRITE_TIFF_COMPRESSION, cv::IMWRITE_TIFF_COMPRESSION_NONE };
		written = cv::imwrite(tiff.toLocal8Bit().toStdString(), height16, params);
	}
	catch (const cv::Exception& e) {
		ct::logger::error("[External3D] imwrite threw: %s", e.what());
	}
	if (!written)
		return { false, QStringLiteral("Cannot write the temporary height map:\n%1").arg(tiff) };

	//3. ImageJ, running the plot macro from its own folder - the Python's Popen(cwd=...)
	say(QStringLiteral("Starting ImageJ..."));
	qint64 pid = 0;
	if (!QProcess::startDetached(exe, { QStringLiteral("-eval"), buildMacro(tiff) },
		QFileInfo(exe).absolutePath(), &pid)) {
		return { false, QStringLiteral("Cannot start ImageJ:\n%1").arg(exe) };
	}
	ct::logger::info("[External3D] Closed %d old ImageJ process(es); started ImageJ (pid %lld) on %s (%d x %d)",
		closed, pid, tiff.toStdString().c_str(), height16.cols, height16.rows);

	//4. wait for the plot, then make it the only ImageJ window showing. The process list is
	//re-read every poll because the JVM that owns the windows starts after ImageJ.exe does.
	for (int waited = 0; waited < kPlotTimeoutMs; waited += kPollMs) {
		const auto pids = imageJProcesses();
		const auto windows = imageJWindows(pids);

		HWND plot = nullptr;
		for (const auto& w : windows)
			if (w.visible && isPlotWindow(w)) plot = w.hwnd; //the last, as the Python took

		if (plot) {
			//Async: a synchronous ShowWindow on another process's window blocks while that
			//process is busy, and a stuck worker would also stall Pogo's shutdown (the join)
			ShowWindowAsync(plot, SW_MAXIMIZE);

			int hidden = 0;
			for (const auto& w : windows) {
				if (!w.visible || isPlotWindow(w)) continue;
				ShowWindowAsync(w.hwnd, SW_HIDE);
				hidden++;
			}
			ct::logger::info("[External3D] Plot window up after ~%d ms; hid %d other ImageJ window(s)",
				waited, hidden);
			return { true, QStringLiteral("External 3D display opened (%1 x %2 px).")
				.arg(height16.cols).arg(height16.rows) };
		}

		//the longest wait of the job, so it counts - a number that moves says nothing is stuck
		if (waited > 0 && waited % 1000 == 0)
			say(QStringLiteral("ImageJ is loading the map and building the 3D plot... %1 s").arg(waited / 1000));

		if (!pause(kPollMs, m_abort)) return { false, QStringLiteral("Cancelled.") };
	}

	//not an error to pop up about: ImageJ is running and may simply still be loading
	ct::logger::warn("[External3D] ImageJ started but no plot window appeared within %d ms", kPlotTimeoutMs);
	return { true, QStringLiteral("ImageJ started, but its 3D plot window did not appear within %1 s - "
		"it may still be loading.").arg(kPlotTimeoutMs / 1000) };
}

External3DViewer::Result External3DViewer::runClose()
{
	const int closed = closeImageJ(m_abort, [this](const QString& text) { say(text); },
		QStringLiteral("Closing the 3D plot..."));
	if (closed < 0) return { false, QStringLiteral("Cancelled.") };

	if (closed == 0) return { true, QStringLiteral("No external 3D display was open.") };
	return { true, QStringLiteral("External 3D display closed.") };
}
