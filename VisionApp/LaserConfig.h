#pragma once
#include "WorldCoordinate.h"

struct LaserConfig {
	dat::WorldCoordinate camera_center;
	dat::WorldCoordinate laser_center;
	dat::WorldCoordinate offset;
};
