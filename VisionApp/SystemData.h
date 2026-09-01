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
	X = 0, Y = 1, Z = 2
};

enum class DOA { //card0
	START_BTN_LED = 0,       //Y100
	STOP_BTN_LED = 1,        //Y101
	RESET_BTN_LED = 2,       //Y102
	RED_TOWER_LIGHT = 3,     //Y103
	AMBER_TOWER_LIGHT = 4,   //Y104
	GREEN_TOWER_LIGHT = 5,   //Y105
	BUZZER = 6,              //Y106
	TROLLEY_LOCK_RELEASE = 7,//Y107
	BRAKE_RELEASE = 8,       //Y108 Z axis motor brake release
	LAST_INDEX
};

enum class DIA {
	START_BTN = 0,           //X100
	STOP_BTN = 1,            //X101
	RESET_BTN = 2,           //X102
	ESTOP_1 = 3,             //X103 emergency stop 1 trigger
	ESTOP_2 = 4,             //X104 emergency stop 2 trigger
	TROLLEY_LOCK_GUARD = 5,  //X105 trolley lock guard switch on
	ESTOP_SAFETY_RELAY = 6,  //X106 e-stop safety relay OK
	CURTAIN_SAFETY_RELAY = 7,//X107 curtain sensor safety relay OK
	PRODUCTION_MODE = 8,     //X108 production mode selector on
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
	std::atomic<bool> _saveUnstitchedImages = false;
	QString _workingPath = "";

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

	/*
	* Build the 3D height map at the SENSOR's own pixel pitch instead of ScaleManager's shared
	* world scale (recipeConfig.json, default false = world scale, i.e. unchanged behaviour).
	*
	* World scale exists so a height map lines up with 2D views. On this machine the profiler
	* delivers 5 um pixels and the world scale is ~27.5 um, so that alignment costs a 5.5x
	* resample. Turning this on keeps the full resolution - at roughly 32x the pixels, and at the
	* price of the alignment: stitching maths and any ROI taught in world pixels both assume the
	* old scale. Off by default for exactly that reason.
	*/
	std::atomic<bool> _heightMapNativeScale = false;

	//which way along that axis the gantry travels: 0 = positive, 1 = negative (recipeConfig.json).
	//Defaults to positive, which is what every recipe taught before scan() supported the other
	//direction did - a missing key therefore keeps existing recipes behaving exactly as before.
	std::atomic<int> _lineScanDirection = 0;
	bool isLineScanDirectionNegative() const { return _lineScanDirection == 1; }

	std::atomic<bool> _lscStrobeMode = false; //light controller default mode: strobe (trigger) vs continuous

	std::atomic<int> _camImageRotation = 0; //camera alignment rotation option: 0/90/180/270, applied in ImageManager preprocessing

	std::atomic<bool> _homeOnStartup = true; //auto home the machine after login

	//barcode reader production flow (recipe settings + taught camera-to-reader offsets)
	std::atomic<int> _brFirstReader = 1;        //which reader scans first: 1 or 2
	std::atomic<int> _brR1Duration_ms = 2000;   //read duration per reader
	std::atomic<int> _brR2Duration_ms = 2000;
	std::atomic<bool> _brR1Taught = false;
	std::atomic<bool> _brR2Taught = false;
	std::atomic<double> _brR1dx = 0.0, _brR1dy = 0.0, _brR1dz = 0.0; //camera-to-reader offsets (mm)
	std::atomic<double> _brR2dx = 0.0, _brR2dy = 0.0, _brR2dz = 0.0;

	//setup region pitch mode (recipe): barcode flow iterates a unit grid from
	//point 1 (top left) using the taught XY pitch, instead of the 3D mid point
	std::atomic<bool> _setupRegionPitchMode = false;
	std::atomic<bool> _pitchP1Set = false;
	std::atomic<double> _pitchP1x = 0.0, _pitchP1y = 0.0, _pitchP1z = 0.0;
	std::atomic<double> _pitchX = 0.0, _pitchY = 0.0; //signed, direction from point 1 to point 2
	std::atomic<int> _unitsX = 1, _unitsY = 1;
	std::string _currentUnitID = "board"; //unit currently in the barcode/OCR flow (written by JobThread)
	std::atomic<bool> _saveInspImages = false; //mirror of the Save Inspection Images toggle for worker threads
	std::atomic<bool> _pitchEnableBarcode = true; //production runs the barcode reader flow
	std::atomic<bool> _pitchEnable3D = true;      //production runs the 3D scan
	std::atomic<double> _pitchScanLen_mm = 10.0;  //3D scan length, centered on each unit's mid point

	std::atomic<bool> _bypassInspection = false;

	std::atomic<bool> _Machine_Ready = true;     // Refer to Machine

	std::atomic<int> _index;

	std::atomic<bool> _InspectionCompleted = true;
	std::atomic<int> _BoardEntryQty = 0;

	std::atomic<bool> _useRecipeScale = false;

	std::atomic<bool> _MotoIsMoving = false;
	std::atomic<bool> _enableFiducialRotate = true;
	std::atomic<bool> _doubleFiducialChecking = false;   // two-island fiducials: fid1/2 = island 1, fid3/4 = island 2 (recipe-based, persisted in recipeConfig.json)

	QString getLaserType() const;
	bool saveLaserType(QString laser);

	//coordinate
	void setCurrentCoordinate(double x, double y, double z);
	const dat::WorldCoordinate& currentCoordinate() const;

	bool _machineDebugMode = false;

	//3D jog
	double m_extraMoveFor3DLaser = 0.00;

	QDateTime StartInspectionTimer;


};

//SystemData::instance()._enableFiducialRotate
