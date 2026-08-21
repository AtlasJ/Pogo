#pragma once
#include <vector>
#include <shared_mutex>

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

enum class Axis {
	X = 0, Y = 1, Z = 2, CY1 = 3, CY2 = 4, FR1 = 5, FR2 = 6, FR3 = 7
};

enum class DOA { //card0
	START_BTN = 0,
	STOP_BTN = 1,
	RESET_BTN = 2,
	RED_TOWER_LIGHT = 3,
	AMBER_TOWER_LIGHT = 4,
	GREEN_TOWER_LIGHT = 5,
	POWER_DRIVE = 6,
	SAFETY_DOOR_LOCK = 7,
	BRAKE_RELEASE = 11,
	LAST_INDEX
};

enum class DIA {
	START_BTN = 0,
	STOP_BTN = 1,
	RESET_BTN = 2,
	AIR_PRESSURE = 3,
	ESTOP = 5,
	CONTACTOR_1 = 6,
	CONTACTOR_2 = 7,
	ENTRY_SENSOR = 8,
	SLOW_SENSOR = 9,
	EXIT_SENSOR = 10,
	CLAMPER_1 = 11,
	CLAMPER_2 = 12,
	CLAMPER_3 = 13,
	CLAMPER_4 = 14,
	DOWNSTREAM = 18,
	UPSTREAM = 19,
	POS1_SENSOR = 20,
	POS2_SENSOR = 21,
	DOOR_INTERLOCK = 22,
	DOOR_SWITCH = 24,
	DOOR_LOCK = 25,
	LAST_INDEX
};

class SystemData {
private:
	SystemData();
	~SystemData();
	SystemData(const SystemData&) = delete;
	SystemData& operator=(const SystemData&) = delete;

	static SystemData m_instance;

	//portability
	void toJson(const ct::Box2D& obj, QJsonObject& j);
	void fromJson(const QJsonObject& j, ct::Box2D& obj);
	void toJson(const dat::WorldCoordinate& obj, QJsonObject& j, bool isRelative);
	void fromJson(const QJsonObject& j, dat::WorldCoordinate& obj, bool isAbsolute);
	void getCurrentMachinePortabilityPointOffset(dat::WorldCoordinate& offset);
	dat::WorldCoordinate getAbsoluteRobotPoint(dat::WorldCoordinate point);
	dat::WorldCoordinate getRelativeRobotPoint(dat::WorldCoordinate point);

	//coordinate
	mutable std::shared_mutex mtx_currentCoordinate;
	dat::WorldCoordinate m_currentCoordinate;

public:
	static SystemData& instance();

	enum class Type {
		PORTABILITY
	};

	bool save(QString path);
	bool load(QString path);

	bool save(Type type);
	bool load(Type type);

	//flow
	std::atomic<bool> _saveUnstackedImages = false;
	std::atomic<bool> _saveUnstitchedImages = false;
	QString _workingPath = "";
	int _subRecipeIndex = 0;

	int _stitchingMethod = 3;
	std::atomic<bool> _controlMotion = false;

	double _maxAllowRamUsage = 80.0;

	int _snapDelay_ms = 0;

	bool _offlineRun = true;
	bool _loadProductionAfterRun = true;

	bool _enable_multi_thread = true;
	int _num_threads = 4;

	//camera
	int _camWidth;
	int _camHeight;
	QHash<QString, double> _camAngles;

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
		int num_z_offset_performed;
	};

	Portability _portability;

	std::string _currentBarcode;

	int _lscTriggerMode = 1;

	//linescan scan direction: 0 = X axis, 1 = Y axis (persisted in recipeConfig.json)
	std::atomic<int> _lineScanAxis = 0;
	bool isLineScanAxisY() const { return _lineScanAxis == 1; }

	std::atomic<bool> _bypassInspection = false;

	std::atomic<bool> _Machine_Ready = true;     // Refer to Machine

	std::atomic<int> _index;

	std::atomic<bool> _InspectionCompleted = true;
	std::atomic<int> _BoardEntryQty = 0;

	std::atomic<bool> _useRecipeScale = false;
	std::atomic<bool> _switchingRecipe = false;

	std::atomic<bool> _MotoIsMoving = false;
	std::atomic<bool> _enableFiducialRotate = true;
	std::atomic<bool> _doubleFiducialChecking = false;   // two-island fiducials: fid1/2 = island 1, fid3/4 = island 2 (recipe-based, persisted in recipeConfig.json)

	QString getLaserType() const;
	bool saveLaserType(QString laser);

	//coordinate
	void setCurrentCoordinate(double x, double y, double z);
	const dat::WorldCoordinate& currentCoordinate() const;

	//PSP
	bool _psp = false;
	bool _machineDebugMode = false;
	void triggerPSP();
	const std::array<QString, 8> _projectNames = { "s0", "s90", "s180", "s270", "l0", "l90", "l180", "l270"};

	//3D jog
	double m_extraMoveFor3DLaser = 0.00;

	QDateTime StartInspectionTimer;


};

//SystemData::instance()._enableFiducialRotate
