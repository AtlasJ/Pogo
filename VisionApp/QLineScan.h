#pragma once

#include <QString>
#include <QVector>
#include "Box2d.h"
#include "WorldCoordinate.h"

class QDragBox;
class QLineScan {
public:
	QLineScan() {}
	~QLineScan() {}

	QString id = "";
	QString name = "";
	QString created_by = "";
	QString type = "";
	QString map_to_slinescan = "";
	dat::WorldCoordinate start_point;
	dat::WorldCoordinate end_point;
	ct::Box2D px;
	QVector<QString> vision_obj_IDs;

	QDragBox *pDragBox = nullptr;
};