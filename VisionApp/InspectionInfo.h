#ifndef INSPECTIONINFO_H
#define INSPECTIONINFO_H

class InspectionInfo
{
public:
	InspectionInfo();
	~InspectionInfo();

	bool _isCollectImage;
	bool _productionMode;
	char _ocr1[1024] = { 0 };
	char _ocr2[1024] = { 0 };
	char _ocr3[1024] = { 0 };
	char _imagePath[1024] = {0};
};
#endif // INSPECTIONINFO_H