#include "PostInspectionThread.h"
#include "Logger.h"

PostInspectionThread::PostInspectionThread(QObject *parent): QThread(parent)
{
	_stopRun = false;
	_appSharedMem.createMemory(std::string("PostInspectionInfo"), sizeof(PostInspectionInfo));

	_pPostInspectionInfo = reinterpret_cast<PostInspectionInfo*>(_appSharedMem.getMemory(std::string("PostInspectionInfo")));
}

void PostInspectionThread::startRun()
{
	if (isRunning() == false)
	{
		start(QThread::HighestPriority);
	}
}

void PostInspectionThread::run()
{
	ct::logger::info("[QThread] Post inspection thread started");
	while (!_stopRun)
	{
		PostInspectionInfo ppInfo;

		if (g_postInspectionQueue.get(ppInfo) == false)
		{
			os_tool::goSleep(1);
			continue;
		}

		_pPostInspectionInfo->_isRunFail = ppInfo._isRunFail;
		_pPostInspectionInfo->_imgWidth = ppInfo._imgWidth;
		_pPostInspectionInfo->_imgHeight = ppInfo._imgHeight;
		_pPostInspectionInfo->_cycleTime = ppInfo._cycleTime;
		_pPostInspectionInfo->_viewIndex = ppInfo._viewIndex;
		_pPostInspectionInfo->_redBufferID = ppInfo._redBufferID;
		_pPostInspectionInfo->_greenBufferID = ppInfo._greenBufferID;
		_pPostInspectionInfo->_blueBufferID = ppInfo._blueBufferID;
		strcpy_s(_pPostInspectionInfo->_frameID, 1024, ppInfo._frameID);
		strcpy_s(_pPostInspectionInfo->_error, 1024, ppInfo._error);
		strcpy_s(_pPostInspectionInfo->_viewID, 1024, ppInfo._viewID);

		emit startPostInspection();
	}
}

void PostInspectionThread::stopRun()
{
	if (isRunning() == true)
	{
		_stopRun = true;
		wait();
	}
}

PostInspectionThread::~PostInspectionThread()
{
}