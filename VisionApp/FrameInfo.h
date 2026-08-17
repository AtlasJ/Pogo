#pragma once
#include "mil.h"
#include "PostAcquisitionTask.h"
#include "MbufWrapper.h"

enum class ICAM_pixelFormat {
	Unknown, RGB8, Mono8, Mono12, BayerGB8, BayerRG8
};

struct FrameInfo {
	FrameInfo();
	~FrameInfo();
	FrameInfo(const FrameInfo& other);
	FrameInfo& operator=(const FrameInfo& other);

	// Move
	FrameInfo(FrameInfo&&) = default;
	FrameInfo& operator=(FrameInfo&&) = default;


	//buffer
	mtrx::SharedMilID pImage = nullptr;
	mtrx::SharedMilID pHeightMap = nullptr;
	std::vector<double> profiles;

	//generic info
	int	bufferSize = 0;
	int	width = 0;
	int	height = 0;
	int channel = 1;
	ICAM_pixelFormat pixelFormat;
	uint64_t timeStamp;
	QString type = "";

	//linkage
	QString cameraID = "";
	QString viewID = "";

	QString baseOpticID = ""; //for cases where optic was split
	QString opticID = "";

	QString stitchID = "";
	int index = -1;
	int row = 0;
	int col = 0;

	PostAcquisitionTask postTask;
};