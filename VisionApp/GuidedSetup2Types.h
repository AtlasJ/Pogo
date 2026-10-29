#pragma once
#include <QString>
#include <QStringList>
#include <QVector>
#include <QJsonObject>
#include "GuidedSetup2Geometry.h"

/*
* Guided Setup (new): everything one recipe teaches, kept in <recipe>/guidedSetup2.json.
*
* It is built alongside the old Guided Setup and never touches the old page's data (fiducials,
* pitch region 1, setup-region mode). A recipe runs the new way only once this file says it is
* complete (production integration comes in a later build step).
*
* The chain is World > fid0 > fidRect > slots > units > inspection points:
*   - fid0's search XY, Start Z and Stop Z are absolute machine positions;
*   - the four corner search positions and the 3D Z spot are offsets from fid0 (world axes);
*   - slot centres live in the trolley frame (fidRect centre + angle, see GuidedSetup2Geometry.h);
*   - units are a rows x cols grid centred in every slot;
*   - inspection points are a unit's centre or corner plus an offset.
* Every XY is where the 2D camera looks; the machine adds the camera->laser / camera->reader offsets.
* Machine values (soft limits, pixel size, optic offsets, camera->stage calibration) stay machine-wide.
*/
namespace gs2 {

struct TaughtXY { bool set = false; double x = 0.0, y = 0.0; };

//a circle finder diameter range in mm
struct DiaRange {
	double minMm = 0.0, maxMm = 0.0;
	bool valid() const { return minMm > 0.0 && maxMm > minMm; }
};

//A: the table-not-empty check at the machine home position
struct Safety1 {
	QStringList emptyImages;   //file names in the recipe folder, saved with no trolley loaded
	double threshold = 0.0;    //a snap must differ from every empty image by more than this
};

//B: safety check 2 + fid0 - the 2D Z sweep, the trolley pattern and the fid0 circle
struct Fid0 {
	TaughtXY xy;                                  //search XY, absolute; the fid0 circle at teach
	bool startSet = false; double startZ = 0.0;   //top of the sweep (smaller Z)
	bool stopSet = false;  double stopZ = 0.0;    //bottom of the sweep and its collision limit
	double stepMm = 0.5;                          //coarse step
	int refineStages = 2;
	double focusBoxMm = 11.0;                     //sharpness is scored in a square this size at the image centre
	QString patternFile;                          //learnt pattern in the recipe folder; empty = not learnt
	double minScore = 0.70;
	DiaRange dia;
};

//C: 3D Centre Z, read by the profiler at fid0 + an offset, starting at the 2D Z
struct Z3D {
	TaughtXY offset;           //camera position of the flat spot, as an offset from fid0
	double stepMm = 2.0;
	double targetMm = 0.0;     //the profiler reading that counts as centred
	double maxSpreadMm = 0.2;
	bool diffSet = false; double diffMm = 0.0;   //taught 2D Z - 3D Z
	double diffTolMm = 0.3;
};

//D: the fidRect - four corner fiducials, judged against the drawing rectangle
struct FidRect {
	double widthMm = 0.0, lengthMm = 0.0;   //typed from the drawing
	TaughtXY corner[4];                     //TL TR BR BL, offsets from fid0
	DiaRange dia;
	double residualLimitMm = 0.10;
	double maxAngleDeg = 1.0;
};

//E: slot size, measured once from one slot's TL and BR circles
struct SlotSize {
	bool set = false;
	double widthMm = 0.0, lengthMm = 0.0;
	DiaRange dia;               //the slot circles
};

//F: one slot; its centre is in the trolley frame (the midpoint of its TL and BR circles)
struct Slot { QString id, name; double cx = 0.0, cy = 0.0; };

//G: the unit grid every slot shares
struct UnitLayout {
	double widthMm = 0.0, lengthMm = 0.0;
	int rows = 1, cols = 1;
	double marginMm = 0.0;
};

//one unit: a cell of one slot's grid; row 0 = far side, col 0 = left
struct Unit { QString id, slotId; int row = 0, col = 0; QString name; };

//H: inspections, at most one of each type per unit
enum InspType { Insp2D = 0, InspBarcodeOcr, Insp3D, InspTypeCount };
enum InspRef { RefCentre = 0, RefTL, RefTR, RefBR, RefBL };
struct Inspection {
	bool enabled = false;
	int ref = RefCentre;
	double dx = 0.0, dy = 0.0;   //offset from the reference, camera position
	double zOffsetMm = 0.0;      //+ = down, from the found 2D Z (2D, Barcode + OCR) or 3D Centre Z (3D)
	double scanLengthMm = 0.0;   //3D only
};

struct Setup {
	Safety1 safety1;
	Fid0 fid0;
	Z3D z3d;
	FidRect fidRect;
	SlotSize slotSize;
	QVector<Slot> slotList;   //"slots" is a Qt macro
	UnitLayout unitLayout;
	QVector<Unit> units;
	Inspection insp[InspTypeCount];
	QString scanAxis = QStringLiteral("X");   //3D scan direction for the whole recipe
	bool scanNegative = false;
	int nextSlotId = 1, nextUnitId = 1;       //ids are handed out once and never reused
};

//the page's sections; the order is the teaching order
enum Section { SecSafety1 = 0, SecFid0, Sec3DZ, SecFidRect, SecSlotSize, SecSlots, SecUnits, SecInspections, SectionCount };
QString sectionTitle(Section s);
bool sectionDone(const Setup& s, Section sec, QString* why = nullptr);
bool isComplete(const Setup& s);
QString inspTypeName(InspType t);
QString refName(int ref);

QJsonObject toJson(const Setup& s);
Setup fromJson(const QJsonObject& o);

enum class LoadResult { Missing, Loaded, Unreadable };
QString filePath(const QString& recipeDir);   //recipeDir ends with '/'
LoadResult load(const QString& recipeDir, Setup& out, QString& why);
bool save(const QString& recipeDir, const Setup& s, QString& why);

/*
* Where the fidRect sat when it was taught: fid0's search XY plus the four corner offsets, fitted
* with the drawing size. Invalid until fid0 and all four corners are taught. The layout view draws
* at this pose until a Locate finds the trolley where it is now.
*/
gs2geo::RectFit taughtFit(const Setup& s);

//world positions of the parts, for a given trolley pose
gs2geo::Pt fid0Taught(const Setup& s);
void slotCornersWorld(const Setup& s, const gs2geo::Pose& pose, const Slot& slot, gs2geo::Pt out[4]);
gs2geo::Pt unitCentreLocal(const Setup& s, const Slot& slot, int row, int col);   //trolley frame
void unitCornersWorld(const Setup& s, const gs2geo::Pose& pose, const Slot& slot, int row, int col, gs2geo::Pt out[4]);
gs2geo::Pt inspectionWorld(const Setup& s, const gs2geo::Pose& pose, const Slot& slot, int row, int col, InspType t);
const Slot* findSlot(const Setup& s, const QString& id);

/*
* Keeps the unit list in step with slots x layout: existing units keep their id and name, units of
* removed cells go, new cells get a new id and their grid name - or a temporary name when that name
* is already taken, which keeps section G not done until it is resolved.
*/
void syncUnits(Setup& s);
//the trolley-wide grid name of every unit, in s.units order: X counts unit columns to the right,
//Y counts unit rows towards the operator, X1Y1 at the far-left corner
QStringList gridNames(const Setup& s);
//Name all by position: every unit gets its grid name, slots S1, S2... far row first, left to right
void nameAllByPosition(Setup& s);
bool isTemporaryName(const QString& name);
//X2Y1 before X10Y1: digits compare as numbers
bool naturalLess(const QString& a, const QString& b);

} // namespace gs2
