#pragma once
#include <QString>
#include "WorldCoordinate.h"
#include "Box2D.h"

struct BarcodeInfo {
	QString id;
	dat::WorldCoordinate teach_point;
	ct::Box2D search_region;
	int image_channel = 0;
	int foreground_type = 0;
	int barcode_type = 0;
	bool hasTeachPoint = false;
	bool generate_2D_stack = false;
	bool generate_3D_stack = false;
	QString acq_type = "";
	int preset_iteration = 5;
	int step_um = 100;
	int encoder_range_um = 1000;
	int recognition_type = 0;
	int registration_method = 0;
};
