#pragma once
#include "Mil.h"
#include <memory>

class ImageRegistration
{
private:
	struct Impl;
	std::unique_ptr<Impl> p;

public:
	enum class AlignmentType { NONE, ECC, ORB };
	enum class ImageType { RED, BLUE, GREEN, GRAYSCALE, BLUE1GREEN2, BLUE2GREEN1, RED_MINUS_BLUE };
	ImageRegistration();
	~ImageRegistration();

	bool ECCImageAlignment(MIL_ID & srcImageColour, MIL_ID & templateImageColour, int imgChannel);
	bool ORBImageAlignment(MIL_ID & srcImageColour, MIL_ID & templateImageColour, int imgChannel, int distanceFilter, bool fast);
	MIL_ID warpImage(MIL_ID & inputImage, MIL_ID & templateImage);
	bool getOffset(double &offset_x, double &offset_y);
};

