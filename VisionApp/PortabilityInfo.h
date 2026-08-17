#pragma once
#include <array>
#include <vector>
#include <QHash>
#include "WorldCoordinate.h"
#include "Box2D.h"

using GVTable = QHash <QString, std::array<double, 256>>;

struct BrightnessInfo {
	int exposure = 20000;
	int gain = 1;
	double average_gap = 0.0;
};

struct LightingCalibrationInfo {
	bool is_main = true;
	GVTable main_GVTable;
	GVTable local_GVTable;
	QHash <QString, BrightnessInfo> brightness;
	dat::WorldCoordinate graycard_point;
};

struct PositionPortabilityInfo {
	QString id;
	QString machine_name;
	QString PIC;
	QString date_created;
	ct::Box2D learn_region;
	ct::Box2D search_region;
	double min_diameter;
	double max_diameter;
	int feature_searching_method = 0;
	double width;
	double height;
	dat::WorldCoordinate portability_point;
	dat::WorldCoordinate offset_point;
	bool learnt_status = false;
};

struct PortabilityInfo {
	LightingCalibrationInfo lightingCalibrationInfo;
};


/*
Portability
=> Exposure and gain
- exposure: collect profile for 14000, 16000, 18000, 20000, 22000
- gain: collect profile for 2, 4, 6, 8
- understand the steps on gray value
*/
