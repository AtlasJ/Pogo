#ifndef POSTINSPECTIONTHREAD_H
#define POSTINSPECTIONTHREAD_H

#include <QThread>
#include "MessageQue.h"
#include "PostInspectionInfo.h"
#include "QOStool.h"
#include "WinSharedMem.h"

extern TMessageQue<PostInspectionInfo> g_postInspectionQueue;

class PostInspectionThread : public QThread
{
	Q_OBJECT

private:
	bool _stopRun;
	WinSharedMem _appSharedMem;
	PostInspectionInfo* _pPostInspectionInfo;
public:
	explicit PostInspectionThread(QObject *parent = 0);
	~PostInspectionThread();

	void run();
	void startRun();
	void stopRun();

signals:
	void startPostInspection();
};

#endif // POSTINSPECTIONTHREAD_H