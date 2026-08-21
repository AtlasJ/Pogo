#pragma once

#include <QString>
#include <QHash>
#include <QJsonObject>

#include "QCommonStruct.h" //BufferInfo
#include <mil.h>

/*
* In-app replacement for the display-buffer surface of the removed Algo
* library's Algo class. VisionApp used Algo as a named MIL buffer registry
* for its main display pipeline (Red/Green/Blue channel buffers plus the
* O_Buffer overlay); the inspection-engine surface died with InspectionThread.
*
* Buffer semantics match Algo::createBuffer: one 8-bit MIL buffer per entry
* of the config's "Buffer" array (16-bit for names containing "HeightMap"),
* cleared to 50, plus an "O_Buffer" overlay sized to the largest buffer,
* cleared to 0.
*/
class AlgoBuffers {
public:
	~AlgoBuffers();

	//allocates the default MIL application + host system (the removed Algo.dll
	//used to do this in Algo::init); must run before any MIL call in the app
	bool init();
	bool release(); //frees the buffers and the MIL application/system

	bool createBuffer(const QJsonObject& bufferInfoObj);
	void bufferInfo(const QString& bufName, BufferInfo& info) const;
	void clearOverlay();
	void releaseBuffer();

private:
	QHash<QString, BufferInfo> _bufHash;
	MIL_ID _milApplication = M_NULL;
	MIL_ID _milSystem = M_NULL;
};
