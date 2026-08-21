#include "AlgoBuffers.h"
#include "QJsonHelper.h"
#include "Logger.h"
#include "mtrx.h"

#include <QJsonArray>
#include <mil.h>

AlgoBuffers::~AlgoBuffers()
{
	release();
}

bool AlgoBuffers::init()
{
	if (_milApplication != M_NULL) return true; //already initialized

	MappAllocDefault(M_DEFAULT, &_milApplication, &_milSystem, M_NULL, M_NULL, M_NULL);

	if (_milApplication == M_NULL || _milSystem == M_NULL) {
		ct::logger::error("[AlgoBuffers] Failed to allocate MIL application/host system");
		return false;
	}

	ct::logger::info("[AlgoBuffers] MIL application + host system allocated");
	return true;
}

bool AlgoBuffers::release()
{
	releaseBuffer();

	if (_milApplication != M_NULL) {
		MappFreeDefault(_milApplication, _milSystem, M_NULL, M_NULL, M_NULL);
		_milApplication = M_NULL;
		_milSystem = M_NULL;
	}

	return true;
}

bool AlgoBuffers::createBuffer(const QJsonObject& bufferInfoObj)
{
	bool flag = true;
	MIL_INT maxImageWidth = -1;
	MIL_INT maxImageHeight = -1;

	const QJsonArray bufferList = jsonHelper::getArray(bufferInfoObj, QStringLiteral("Buffer"));

	for (int i = 0; i < bufferList.count(); i++) {
		const QJsonObject bufferObj = bufferList[i].toObject();
		const QString name = jsonHelper::getString(bufferObj, QStringLiteral("Buffer_Name"));
		const MIL_INT imageWidth = jsonHelper::getInteger(bufferObj, QStringLiteral("Buffer_Width"));
		const MIL_INT imageHeight = jsonHelper::getInteger(bufferObj, QStringLiteral("Buffer_Height"));

		BufferInfo info;
		info.camera = jsonHelper::getString(bufferObj, QStringLiteral("Camera"));
		info.umPerPixel = jsonHelper::getDouble(bufferObj, QStringLiteral("Micrometer_Per_Pixel"));

		maxImageWidth = qMax(imageWidth, maxImageWidth);
		maxImageHeight = qMax(imageHeight, maxImageHeight);

		if (name.contains(QStringLiteral("HeightMap"))) {
			info.bufID = MbufAlloc2d(M_DEFAULT_HOST, imageWidth, imageHeight, 16 + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);
			if (info.bufID != M_NULL) MbufInquire(info.bufID, M_HOST_ADDRESS, &info.pHeightMap);
			info.pBuf = nullptr;
		}
		else {
			info.bufID = MbufAlloc2d(M_DEFAULT_HOST, imageWidth, imageHeight, 8 + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);
			if (info.bufID != M_NULL) MbufInquire(info.bufID, M_HOST_ADDRESS, &info.pBuf);
			info.pHeightMap = nullptr;
		}

		if (info.bufID != M_NULL) {
			MbufInquire(info.bufID, M_PITCH_BYTE, &info.pitch);
			MbufInquire(info.bufID, M_SIZE_X, &info.width);
			MbufInquire(info.bufID, M_SIZE_Y, &info.height);
			_bufHash.insert(name, info);
			MbufClear(info.bufID, 50);
		}
		else {
			ct::logger::error("[AlgoBuffers] Failed to allocate buffer '%s' (%lld x %lld)",
				name.toStdString().c_str(), imageWidth, imageHeight);
			flag = false;
		}
	}

	//overlay buffer sized to the largest channel buffer
	if (maxImageWidth > 0 && maxImageHeight > 0) {
		BufferInfo info;
		info.bufID = MbufAlloc2d(M_DEFAULT_HOST, maxImageWidth, maxImageHeight, 8 + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);

		if (info.bufID != M_NULL) {
			MbufInquire(info.bufID, M_PITCH_BYTE, &info.pitch);
			MbufInquire(info.bufID, M_HOST_ADDRESS, &info.pBuf);
			MbufInquire(info.bufID, M_SIZE_X, &info.width);
			MbufInquire(info.bufID, M_SIZE_Y, &info.height);
			info.camera = QStringLiteral("NIL");
			info.umPerPixel = 1;
			_bufHash.insert(QStringLiteral("O_Buffer"), info);
			MbufClear(info.bufID, 0);
		}
		else {
			flag = false;
		}
	}

	return flag;
}

void AlgoBuffers::bufferInfo(const QString& bufName, BufferInfo& info) const
{
	if (_bufHash.contains(bufName)) {
		info = _bufHash.value(bufName);
	}
	else {
		info = BufferInfo();
		info.bufID = M_NULL;
		info.pitch = 0;
		info.pBuf = nullptr;
		info.width = 0;
		info.height = 0;
	}
}

void AlgoBuffers::clearOverlay()
{
	BufferInfo oBufInfo;
	bufferInfo(QStringLiteral("O_Buffer"), oBufInfo);
	if (oBufInfo.bufID) MbufClear(oBufInfo.bufID, 0);
}

void AlgoBuffers::releaseBuffer()
{
	for (auto it = _bufHash.begin(); it != _bufHash.end(); ++it) {
		mtrx::free_buffer(it.value().childBufID);
		mtrx::free_buffer(it.value().mChildHeightMap);
		mtrx::free_buffer(it.value().bufID);
		mtrx::free_buffer(it.value().mHeightMap);
	}
	_bufHash.clear();
}
