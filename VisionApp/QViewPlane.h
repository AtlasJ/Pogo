#pragma once

#include <QString>
#include <array>
#include <vector>
#include "Box2d.h"
#include "WorldCoordinate.h"
#include "QView.h"

enum class Corner {
	BACKRIGHT, FRONTLEFT, BACKLEFT, FRONTRIGHT
};

struct QViewPlane {
	QString id = "";
	QString name = "";
	QString created_by = "";
	int horizontal_num = 0;
	int vertical_num = 0;
	double distance_to_plane = 0.0;
	std::string corner_mode = "";
	double width_px = 0.0;
	double height_px = 0.0;
	double width_mm = 0.0;
	double height_mm = 0.0;
	double horizontal_overlap_percentage = 0.0;
	double vertical_overlap_percentage = 0.0;
	double horizontal_overlap_px = 0.0;
	double vertical_overlap_px = 0.0;
	double horizontal_overlap_mm = 0.0;
	double vertical_overlap_mm = 0.0;
	std::array<dat::WorldCoordinate, 4> corner_points;
	std::vector<QView> views;

	QViewPlane();
	~QViewPlane();
};
