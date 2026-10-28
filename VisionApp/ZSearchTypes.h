#pragma once
#include <QString>

class QFrame;
class QLabel;
class QLineEdit;
class QDoubleSpinBox;
class QSpinBox;
class QCheckBox;

/*
* Z search: find THIS table's Z before anything is taught or run against it.
*
* The trolley's height varies by up to a few centimetres from one load to the next, so a taught Z
* puts the camera out of focus and the profiler out of range. Instead the machine looks for the
* table each time: the 2D search sweeps Z down from Start Z, scoring how sharp the camera image is,
* and keeps the sharpest Z (2D Focus Z); the 3D search steps Z down until the profiler reads a flat
* surface, then moves until that surface sits at the centre of the profiler's range (3D Centre Z).
*
* Machine Z is POSITIVE DOWNWARDS, towards the table. So Start Z is the top of a search and the
* smaller number; Stop Z is the bottom and the collision limit - a search never passes it.
*
* Everything here is per recipe (recipeConfig.json, "zSearch"), like the safety check, except the
* camera-to-laser constant, which is the machine's own mounting.
*/
struct ZSearch2DConfig {
	bool xySet = false;    double x = 0.0, y = 0.0;   //camera over the screw head
	bool startSet = false; double startZ = 0.0;        //top of the search (smaller Z)
	bool stopSet = false;  double stopZ = 0.0;         //bottom of the search and its collision limit
	double stepMm = 0.5;                               //coarse step: only has to land on the sharpness hill, not its top
	int refineStages = 2;                              //at most this many zoom stages after the coarse pass, each a fifth of the last step
	int boxPx = 400;                                   //sharpness is scored in a square this size at the image centre
	int circleMinPx = 200, circleMaxPx = 400;          //Teach XY: the screw head's diameter range for the circle finder
	bool confirmSafetyCheck = false;                   //a Z only counts when the recipe's Safety Check passes there
};

struct ZSearch3DConfig {
	bool xySet = false;    double x = 0.0, y = 0.0;   //CAMERA position over the flat area; the laser goes there + the laser offset
	bool startSet = false; double startZ = 0.0;
	bool stopSet = false;  double stopZ = 0.0;
	double stepMm = 2.0;                               //coarse step while nothing reads - must stay well inside the profiler's range
	double targetMm = 0.0;                             //the profiler reading that counts as centred (0 = the head's reference distance)
	double maxSpreadMm = 0.2;                          //a read is a flat surface only if its middle 50% lies within this
};

struct ZSearchConfig {
	ZSearch2DConfig s2d;
	ZSearch3DConfig s3d;

	//2D Focus Z - 3D Centre Z on this trolley, as taught; a run that measures something else has
	//locked onto the wrong thing (checked from Phase 3)
	bool diffSet = false;
	double diffMm = 0.0;
	double diffTolMm = 0.3;
};

/*
* Machine-wide (config/z_calibration.json): 2D Focus Z - 3D Centre Z with the camera and the laser
* on the SAME flat surface. That is the mounting of the two, and what will later place a 3D scan
* from where the camera found the board.
*/
struct ZCalibration {
	bool set = false;
	double camLaserDzMm = 0.0;
	QString when;
};

//What was found on the table in front of the machine now. Never saved: cleared on recipe change
//and when the trolley is taken off.
struct ZFound {
	bool has2D = false; double z2D = 0.0;
	bool has3D = false; double z3D = 0.0;
};

//the widgets the Z search frames are refreshed through; built in code, owned by their frames
struct ZSearchWidgets {
	QFrame* frame2D = nullptr;
	QFrame* frame3D = nullptr;

	//positions are read-only: only Set / Teach XY (where the machine actually is) can change them
	QLineEdit* xy2 = nullptr;
	QLineEdit* start2 = nullptr;      QLineEdit* stop2 = nullptr;
	QDoubleSpinBox* step2 = nullptr;  QSpinBox* stages2 = nullptr;
	QSpinBox* box2 = nullptr;
	QSpinBox* circleMin2 = nullptr;   QSpinBox* circleMax2 = nullptr;
	QCheckBox* confirm2 = nullptr;
	QLineEdit* found2 = nullptr;      QLabel* status2 = nullptr;

	QLineEdit* xy3 = nullptr;
	QLineEdit* start3 = nullptr;      QLineEdit* stop3 = nullptr;
	QDoubleSpinBox* step3 = nullptr;  QDoubleSpinBox* target3 = nullptr;
	QDoubleSpinBox* spread3 = nullptr;
	QLabel* laserAt3 = nullptr;
	QLineEdit* found3 = nullptr;      QLabel* status3 = nullptr;

	QLineEdit* diffNow = nullptr;     QLineEdit* diffTaught = nullptr;
	QDoubleSpinBox* diffTol = nullptr;
	QLabel* calibration = nullptr;
};
