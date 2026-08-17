#pragma once
#include <vector>
#include <QJsonObject>
#include <QHash>
#include "QOrderedHash.h"
#include "WorldCoordinate.h"
#include "PortabilityInfo.h"
#include "QDragBox.h"

/*
* Note: This is a singleton for system related data.
* The data is usually tied to a machine
*/

class RecipeData {
private:
	RecipeData();
	~RecipeData();
	RecipeData(const RecipeData&) = delete;
	RecipeData& operator=(const RecipeData&) = delete;

	static RecipeData m_instance;

	//portability
	void toJson(const ct::Box2D& obj, QJsonObject& j);
	void fromJson(const QJsonObject& j, ct::Box2D& obj);
	void toJson(const dat::WorldCoordinate& obj, QJsonObject& j, bool isRelative);
	void fromJson(const QJsonObject& j, dat::WorldCoordinate& obj, bool isAbsolute);
	void getCurrentMachinePortabilityPointOffset(dat::WorldCoordinate& offset);
	dat::WorldCoordinate getAbsoluteRobotPoint(dat::WorldCoordinate point);
	dat::WorldCoordinate getRelativeRobotPoint(dat::WorldCoordinate point);

public:
	static RecipeData& instance();

	enum class Type {
		PORTABILITY
	};

	bool save(QString path);
	bool load(QString path);

	bool save(Type type);
	bool load(Type type);

	//camera
	int _camWidth;
	int _camHeight;

	//portability 
	struct Portability {
		QPointF pattern_size;
		dat::WorldCoordinate position_point;
		bool done = false;
		PositionPortabilityInfo ref_info;
		PositionPortabilityInfo current_info;
		QDragBox learn_region;
		QDragBox search_region;
		QDragBox located_region;
	};

	Portability _portability;
};
