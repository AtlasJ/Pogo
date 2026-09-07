#pragma once

#include <QString>
#include <QVector>
#include <QRectF>
#include <QPointF>
#include <QColor>
#include <QMetaType>

/*
* 3D Height Measurement 3 - the data contract.
*
* ONE input struct (AlgoHeight3Params) carries every setting on the eight QToolBox
* sections of widget_algoHeight3Page; ONE output struct (AlgoHeight3Output) carries
* every number the page displays plus everything a later process would export.
* Nothing else crosses the boundary, so the algorithm never touches a widget and the
* page never reaches into the pipeline.
*
* COORDINATE SPACES - the one thing to get right here:
*   - The SOURCE maps (height + intensity) are in map pixels, origin top-left.
*   - Segmentation finds the part and produces a STRAIGHTENED CROP: the part rotated
*     upright and cut to its own bounding box, resampled nearest-neighbour so a height
*     sample is never blended with its neighbour.
*   - Every ROI (datum and measurement) lives in the PART FRAME: crop pixels with the
*     origin at the CENTRE of the crop. So an ROI keeps its meaning when the part is
*     placed slightly differently, and an ROI is meaningless until a crop exists -
*     which is exactly why the page refuses to show ROIs before segmentation succeeds.
*     partFrame -> cropPixels is rect.translated(cropW/2, cropH/2).
*/

// ── enums ────────────────────────────────────────────────────────────────────

//which QToolBox section's work to run; All = the page's top Run button
enum class AlgoH3Stage {
	Preprocess = 0,
	Segment,
	Datum,
	Measure,
	Overall,
	All
};

/*
* Which image the page is showing. Order matches comboBox_algoH3Display. This is pure
* view state and is never persisted, so unlike AlgoH3Method it can be reordered freely.
*/
enum class AlgoH3Display {
	HeightColor = 0,
	HeightGray,
	Intensity,
	Surface3D
};

//order matches comboBox_algoH3PreprocessMethod and stackedWidget_algoH3Preprocess
enum class AlgoH3Preprocess {
	None = 0,
	Median,
	Gaussian,
	Bilateral,
	Opening,
	Closing
};

//order matches comboBox_algoH3DatumMethod
enum class AlgoH3DatumMethod {
	LeastSquares = 0, //vertical (z) error, the QAlgoHeightMeasurement fit
	PcaSvd = 1        //orthogonal (perpendicular) error, smallest-eigenvector normal
};

/*
* Height measurement methods. The VALUE IS THE METHOD ID the operator types into an
* ROI type's Method ID column, and it is deliberately the same number as the row index
* in comboBox_algoH3Method ("0 - Mean Height", "1 - Median Height", ...). Insert a new
* method anywhere but the end and every recipe's Method IDs silently change meaning -
* so only ever APPEND.
*/
enum class AlgoH3Method {
	Mean = 0,
	Median = 1,
	Maximum = 2,
	Minimum = 3,
	Percentile = 4
};
constexpr int kAlgoH3MethodCount = 5;

QString algoH3MethodName(int methodId);   //"Mean Height" etc, "" when out of range
bool algoH3MethodValid(int methodId);

// ── input ────────────────────────────────────────────────────────────────────

//one row of tableWidget_algoH3RoiTypes
struct AlgoH3RoiType {
	QString name;                        //unique, chosen once at Add time, never edited after
	QColor color = QColor(0, 200, 0);
	double minUm = 0.0;                  //criteria band; max <= min means "no limit"
	double maxUm = 0.0;
	int methodId = (int)AlgoH3Method::Mean;
};

//one measurement ROI. Every ROI belongs to a type - deleting a type deletes its ROIs,
//so there is no such thing as an untyped ROI and no branch anywhere has to handle one.
struct AlgoH3Roi {
	QString typeName;
	QRectF rel;      //part frame (origin = crop centre), px
};

struct AlgoHeight3Params {
	// ── section 0: input, scaling, valid ranges ──
	double xScaleUmPx = 5.0;        //um per pixel across the map
	double yScaleUmPx = 5.0;        //um per pixel along the map
	double zScaleRawPerUm = 1.25;   //raw grey levels per um: um = raw / this
	int minValidRaw = 1;            //raw values outside [min,max] are dropouts: never
	int maxValidRaw = 65535;        //segmented, never fitted, never measured
	double minValidHeightUm = 0.0;  //applied AFTER the datum plane, to measurement samples
	double maxValidHeightUm = 0.0;  //max <= min means "no valid-height limit"

	// ── section 1: data preprocessing ──
	AlgoH3Preprocess preprocess = AlgoH3Preprocess::None;
	int medianKernel = 3;           //16-bit medianBlur accepts 3 or 5 only
	int gaussianKernel = 3;
	double gaussianSigma = 1.0;
	int bilateralDiameter = 5;
	double bilateralSpatialSigma = 5.0;
	double bilateralHeightSigma = 50.0;  //in RAW grey levels, same units as the map
	int openingKernel = 3;
	int closingKernel = 3;

	// ── section 2: segmentation ──
	bool segCheckWidth = false;
	double segMinWidthUm = 0.0, segMaxWidthUm = 0.0;
	bool segCheckHeight = false;
	double segMinHeightUm = 0.0, segMaxHeightUm = 0.0;
	bool segCheckAngle = false;
	double segMinAngleDeg = 0.0, segMaxAngleDeg = 0.0;

	// ── section 3: datum plane ──
	AlgoH3DatumMethod datumMethod = AlgoH3DatumMethod::LeastSquares;
	bool datumCheckTilt = false;
	double datumMaxTiltDeg = 0.0;   //tilt is an absolute angle, so there is no minimum
	QVector<QRectF> datumRois;      //part frame, px

	// ── section 4: height measurement settings ──
	int methodId = (int)AlgoH3Method::Mean; //the page's own selection (which help page shows)
	double percentile = 50.0;               //shared by every ROI type using method 4

	// ── section 5: ROI types & criteria ──
	QVector<AlgoH3RoiType> roiTypes;
	QVector<AlgoH3Roi> rois;

	// ── section 7: overall result ──
	bool overallCheckCount = false;
	int overallMaxCount = 0;
	bool overallCheckRate = false;
	double overallMaxRatePct = 0.0;

	//lookup helpers (linear - the type list is a handful of rows, not a data structure)
	int indexOfType(const QString& name) const;
	bool hasType(const QString& name) const { return indexOfType(name) >= 0; }
};

// ── output ───────────────────────────────────────────────────────────────────

//every section shows the same trio: Result / Fail Reason / Time Taken
struct AlgoH3StageResult {
	bool ran = false;         //false = never attempted since the last invalidation
	bool pass = false;
	QString failReason;
	qint64 elapsedMs = 0;
};

struct AlgoH3RoiResult {
	int id = 0;               //1-based, matches the ROI's on-screen name
	QString typeName;
	QColor color;
	double criteriaMinUm = 0.0;
	double criteriaMaxUm = 0.0;
	int methodId = 0;
	QString methodName;
	QRectF rel;               //part frame, as taught
	QRectF measuredRect;      //crop px actually read, after clipping to the image
	bool clipped = false;     //partially outside the crop - measured on what was left
	bool valid = false;       //had at least one usable sample
	bool pass = false;
	double heightUm = 0.0;
	qint64 sampleCount = 0;
	QString failReason;
};

struct AlgoHeight3Output {
	//NOTE: registered as a queued-connection metatype in AlgoManager::init()

	AlgoH3StageResult preprocess;
	AlgoH3StageResult segment;
	AlgoH3StageResult datum;
	AlgoH3StageResult measure;
	AlgoH3StageResult overall;

	// ── segmentation ──
	double segWidthUm = 0.0;
	double segHeightUm = 0.0;
	double segAngleDeg = 0.0;
	QRectF segRectMap;          //axis-aligned bounds of the part in MAP px (for the overlay)
	QVector<QPointF> segCorners;//the four rotated-rect corners in MAP px
	int cropWidthPx = 0;
	int cropHeightPx = 0;

	// ── datum plane: z = a*x + b*y + c, raw grey levels, crop px ──
	bool planeValid = false;
	double planeA = 0.0, planeB = 0.0, planeC = 0.0;
	double planeTiltDeg = 0.0;  //absolute angle between the plane normal and the map normal
	double planeRmsUm = 0.0;    //not shown on the page yet - kept for export
	qint64 datumPoints = 0;

	// ── measurement ──
	QVector<AlgoH3RoiResult> roiResults;

	// ── overall ──
	int totalPins = 0;
	int passedPins = 0;
	int failedPins = 0;
	double failedRatePct = 0.0;
	bool overallPass = false;

	qint64 totalElapsedMs = 0;  //filled by a Run All only

	const AlgoH3RoiResult* roiById(int id) const;
};

/*
* One readable line describing a Run All: which stage stopped it, or the pin tally when it
* got all the way through. Lives here rather than in the page because production reports the
* same thing to the inspection log, and a unit's verdict must not be described two different
* ways depending on who is looking.
*/
QString algoH3RunSummary(const AlgoHeight3Output& out);

Q_DECLARE_METATYPE(AlgoHeight3Output)
