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

	//3D scan travelled in the negative direction along the scan axis, so the profiles arrived
	//in reverse order and the raw frame is mirrored along that axis until ImageManager flips
	//it back. NOTE: the copy constructor and operator= below are hand-written, so a member
	//missing from them is dropped silently on every copy - add new fields to BOTH.
	bool scanReversed = false;

	//The unit this frame shows sits INVERTED on its trolley (its Template Mapping cell says so):
	//ImageManager turns the finished image - and a 3D frame's height + intensity maps - by exactly
	//180 degrees (two flips, no resampling) before anything displays, saves or inspects it, so the
	//unit always comes out upright. Written on every snap / scan (JobThread::m_unitTurn180), as the
	//driver frames are persistent and must not carry one unit's turn to the next frame.
	bool turn180 = false;
};
