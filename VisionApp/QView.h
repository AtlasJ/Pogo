#pragma once

#include <QString>
#include <QVector>
#include "Box2d.h"
#include "WorldCoordinate.h"
#include "Def.h"

struct QROI {
	std::string id = "";
	ct::Box2D box;
	std::string assigned_to = "";
};

struct ZStackInfo {
	QString acq_type = ct::s_preset;

	int step_um = 500;
	
	//preset
	int preset_iteration = 3;

	//encoder
	int encoder_range_um = 100;

	//time
	int time_interval_ms = 50;

	bool generate_2D_stack = false;
	bool generate_3D_stack = false;

	// Assignment operator
	ZStackInfo& operator=(const ZStackInfo& other);

	// Copy constructor
	ZStackInfo(const ZStackInfo& other);

	// Equality operator
	bool operator==(const ZStackInfo& other) const;

	ZStackInfo();
	~ZStackInfo();
};

struct PreprocessViewInfo {
	bool crop = false;
	bool resize = false;
	bool flatfield = false;

	QRect cropRect;
	QSize resizeRect;

	// Assignment operator
	PreprocessViewInfo& operator=(const PreprocessViewInfo& other);

	// Copy constructor
	PreprocessViewInfo(const PreprocessViewInfo& other);

	// Equality operator
	bool operator==(const PreprocessViewInfo& other) const;

	PreprocessViewInfo();
	~PreprocessViewInfo();
};

class QDragBox;
class QView {
public:
	QView();
	~QView();

	QString id = "";
	QString name = "";
	QString type = "";
	QString created_by = "";
	QString camID = "cam1";
	QSet<QString> opticIDs;
	double horizontal_scale = 0.0;
	double vertical_scale = 0.0;
	QString map_to_sview = "";
	dat::WorldCoordinate world;
	ct::Box2D px;
	QVector<QString> vision_obj_IDs;

	QDragBox *pDragBox = nullptr;

	PreprocessViewInfo preprocess;
	ZStackInfo zstack;
};