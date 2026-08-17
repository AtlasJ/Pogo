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
}

PostAcquisitionTask& PostAcquisitionTask::operator=(const PostAcquisitionTask& other)
{
	rotationalAngle = other.rotationalAngle;
	bandType = other.bandType;
	combineRGB = other.combineRGB;
	rgbOffset = other.rgbOffset;
	stackImage = other.stackImage;
	return *this;
}
