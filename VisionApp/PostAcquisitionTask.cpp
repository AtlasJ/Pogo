#include "PostAcquisitionTask.h"

PostAcquisitionTask::PostAcquisitionTask()
{
}

PostAcquisitionTask::~PostAcquisitionTask()
{
}

PostAcquisitionTask::PostAcquisitionTask(const PostAcquisitionTask& other)
{
	rotationalAngle = other.rotationalAngle;
	bandType = other.bandType;
	combineRGB = other.combineRGB;
	rgbOffset = other.rgbOffset;
	stackImage = other.stackImage;
	scanReversed = other.scanReversed;
}

PostAcquisitionTask& PostAcquisitionTask::operator=(const PostAcquisitionTask& other)
{
	rotationalAngle = other.rotationalAngle;
	bandType = other.bandType;
	combineRGB = other.combineRGB;
	rgbOffset = other.rgbOffset;
	stackImage = other.stackImage;
	scanReversed = other.scanReversed;
	return *this;
}
