#ifndef POSTINSPECTIONINFO_H
#define POSTINSPECTIONINFO_H

#include <QString>

class PostInspectionInfo
{
public:
	PostInspectionInfo();
	~PostInspectionInfo();

	bool _isRunFail;
	
	int _imgWidth;
	int _imgHeight;
	int _cycleTime;
	int _redBufferID;
	int _greenBufferID;
	int _blueBufferID;
	int _viewIndex;

	char _frameID[1024] = { 0 };
	char _error[1024] = { 0 };
	char _viewID[1024] = {0};
};
#endif // POSTINSPECTIONINFO_H