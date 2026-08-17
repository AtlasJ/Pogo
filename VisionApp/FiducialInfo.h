#pragma once
#include <QString>
#include "WorldCoordinate.h"
#include "Box2D.h"

struct FiducialInfo {
	QString id;
	dat::WorldCoordinate teach_point;
	ct::Box2D inspect_region;
	ct::Box2D search_region;
	bool hasTeachPoint = false;
	int fiducial_method = 0;
	int score = 50;
	int min_diameter = 280;
	int max_diameter = 320;
	bool generate_2D_stack = false;
	bool generate_3D_stack = false;
	QString acq_type = ct::s_preset;
	int preset_iteration = 5;
	int step_um = 100;
	int encoder_range_um = 5000;

};
