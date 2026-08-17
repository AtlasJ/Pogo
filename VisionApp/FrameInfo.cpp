#include "FrameInfo.h"

FrameInfo::FrameInfo()
{
}

FrameInfo::~FrameInfo()
{
}

FrameInfo::FrameInfo(const FrameInfo& other)
{
	pImage = other.pImage;
	pHeightMap = other.pHeightMap;
	profiles = other.profiles;

	bufferSize = other.bufferSize;
	width = other.width;
	height = other.height;
	channel = other.channel;
	pixelFormat = other.pixelFormat;
	timeStamp = other.timeStamp;

	cameraID = other.cameraID;
	viewID = other.viewID;
	baseOpticID = other.baseOpticID;
	opticID = other.opticID;
	stitchID = other.stitchID;
	index = other.index;
	row = other.row;
	col = other.col;

	type = other.type;

	postTask = other.postTask;
}

FrameInfo& FrameInfo::operator=(const FrameInfo& other)
{
	pImage = other.pImage;
	pHeightMap = other.pHeightMap;
	profiles = other.profiles;

	bufferSize = other.bufferSize;
	width = other.width;
	height = other.height;
	channel = other.channel;
	pixelFormat = other.pixelFormat;
	timeStamp = other.timeStamp;

	cameraID = other.cameraID;
	viewID = other.viewID;
	baseOpticID = other.baseOpticID;
	opticID = other.opticID;
	stitchID = other.stitchID;
	index = other.index;
	row = other.row;
	col = other.col;

	type = other.type;

	postTask = other.postTask;
	return *this;
}
