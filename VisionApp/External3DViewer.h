#pragma once

#include <QObject>
#include <QString>
#include <atomic>
#include <functional>
#include <thread>
#include <opencv2/core.hpp>

/*
* External 3D display: hands a height map to ImageJ's "Interactive 3D Surface Plot" in a
* window of its own. A port of Jimmy's Python launcher (open / close), minus its tkinter UI.
*
* Open  = close any ImageJ still running -> write the map to ONE fixed temp TIFF (overwritten
*         every time, so it never accumulates) -> launch ImageJ with the plot macro -> wait for
*         the plot window, maximise it, and hide ImageJ's other windows.
* Close = WM_CLOSE to every ImageJ window, wait up to 5 s, then terminate what is left.
*
* Both run on a worker thread, because either can wait for seconds on another process. They
* report each step through progress() and the outcome through finished(), both arriving
* queued on the GUI thread. One job at a time: open()/close() refuse while busy() rather
* than queue.
*
* "ImageJ" means a process whose EXECUTABLE lives under the ImageJ install folder - which
* covers ImageJ.exe and the javaw.exe it starts from its bundled jre\bin (ImageJ.cfg), and
* nothing else. The Python version also matched windows by TITLE, which would have posted
* WM_CLOSE to, say, a browser window showing the plugin's wiki page.
*/
class External3DViewer : public QObject
{
	Q_OBJECT

public:
	explicit External3DViewer(QObject* parent = nullptr);
	~External3DViewer() override; //aborts a running job and joins it

	static QString imageJExe();     //where ImageJ is expected - the same path on the laptop and the IPC
	static QString tempTiffPath();  //the one file every open() overwrites

	bool busy() const { return m_busy; }

	//16-bit single-channel height map, deep-copied by the caller (the worker owns it)
	bool open(const cv::Mat& height16);
	bool close();

signals:
	//what the job is doing right now, for the please-wait box - several per job
	void progress(QString text);

	//ok=false means the operator should be told; ok=true may still carry a warning
	void finished(bool ok, QString message);

private:
	struct Result { bool ok; QString message; };
	bool start(std::function<Result()> job);

	Result runOpen(const cv::Mat& height16);
	Result runClose();
	void say(const QString& text); //progress(), unless the viewer is being torn down

	std::thread m_worker;
	std::atomic<bool> m_busy{ false };
	std::atomic<bool> m_abort{ false };
};
