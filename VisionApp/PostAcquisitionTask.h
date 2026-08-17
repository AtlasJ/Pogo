#pragma once
#include "OpticsInfo.h"

struct PostAcquisitionTask {
	PostAcquisitionTask();
	~PostAcquisitionTask();
	PostAcquisitionTask(const PostAcquisitionTask& other);
	PostAcquisitionTask& operator=(const PostAcquisitionTask& other);

	//rotation task
	double rotationalAngle = 0.0;

	//combine rgb task
	bool combineRGB = false;
	BandType bandType = BandType::M;

	//rgb offset
	RGBOffset rgbOffset;

	//zstack
	bool stackImage = false;
};
