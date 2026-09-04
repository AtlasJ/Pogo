#pragma once

#include <QString>
#include <QStringList>
#include <QVector>
#include <QRectF>
#include <QPolygonF>
#include <QPointF>
#include <QPoint>
#include <QColor>
#include <QImage>

/*
* Shared data types for the Algo Setup page (AlgoManager).
* OCR types follow IM430's ocrInspection2 core; height measurement follows
* the Algo project's QAlgoHeightMeasurement plane-fit method; the locator
* follows QAlgoLocator.
*/

//which algo the setup page combobox is on (name avoids the Algo lib's AlgoType)
enum class AlgoPageAlgo {
	OCR_READ = 0,
	HEIGHT_3D = 1,
	HEIGHT_3D_V2 = 2,     //new pipeline, built beside HEIGHT_3D - UI shell only for now
	HEIGHT_3D_V3 = 3      //second layout attempt, also a shell - no locator, no algorithm
};

/*
* How many algos own a Locator config. HEIGHT_3D_V2 and HEIGHT_3D_V3 deliberately do NOT -
* their segmentation stage locates the part itself, on the height data rather than on an 8-bit
* render of it, and returns width, height and angle instead of only a match score.
*
* This constant exists because AlgoPageAlgo indexes AlgoManager::m_locator[] directly and the
* enum is now LARGER than that array. Anything converting an AlgoPageAlgo to an index must
* bounds-check against this, or selecting one of those pages writes past the end of the array -
* silent memory corruption, not a crash, because captureAlgoParamsFromUI() runs on both Save
* and Run.
*/
constexpr int kAlgoLocatorSlots = 2;

inline bool algoHasLocator(AlgoPageAlgo a)
{
	const int i = (int)a;
	return i >= 0 && i < kAlgoLocatorSlots;
}

/*
* Which algos actually have an implementation behind them. The V2 and V3 pages are layout only.
* Written as a WHITELIST rather than a list of exclusions so that the next page added to the
* enum refuses by default, instead of falling through to whatever the last else branch runs.
*/
inline bool algoIsImplemented(AlgoPageAlgo a)
{
	return a == AlgoPageAlgo::OCR_READ || a == AlgoPageAlgo::HEIGHT_3D;
}

//one PaddleOCR text detection (matches IM430's OcrResult)
struct AlgoOcrBox {
	QString text;
	float score = 0.f;
	QVector<QPoint> box; //4 corners, OCR image space

	QRect boundingRect() const {
		if (box.isEmpty()) return QRect();
		int minX = box[0].x(), minY = box[0].y(), maxX = box[0].x(), maxY = box[0].y();
		for (const auto& p : box) {
			minX = qMin(minX, p.x()); minY = qMin(minY, p.y());
			maxX = qMax(maxX, p.x()); maxY = qMax(maxY, p.y());
		}
		return QRect(QPoint(minX, minY), QPoint(maxX, maxY));
	}
};

//overlay primitives the worker returns for the UI thread to draw on the FOV scene
struct AlgoOverlayItem {
	enum Type { Rect, Polygon, Text, Cross };
	Type type = Rect;
	QRectF rect;
	QPolygonF poly;
	QString text;
	QPointF pos;
	QColor color = QColor(0, 255, 127);
	QColor fill = Qt::transparent;
	int pointSize = 12;

	static AlgoOverlayItem makeRect(const QRectF& r, const QColor& c, const QColor& f = Qt::transparent) {
		AlgoOverlayItem i; i.type = Rect; i.rect = r; i.color = c; i.fill = f; return i;
	}
	static AlgoOverlayItem makePoly(const QPolygonF& p, const QColor& c) {
		AlgoOverlayItem i; i.type = Polygon; i.poly = p; i.color = c; return i;
	}
	static AlgoOverlayItem makeText(const QString& t, const QPointF& at, const QColor& c, int size = 12) {
		AlgoOverlayItem i; i.type = Text; i.text = t; i.pos = at; i.color = c; i.pointSize = size; return i;
	}
};

// ── OCR pattern library (per-character MIL pattern matching) ────────────────

struct OcrPatternSample {
	QString filePath; //absolute .mpat path in memory, recipe-relative on disk
};

struct OcrPatternLabel {
	QString label; //single character, e.g. "0", "O", "8"
	QVector<OcrPatternSample> samples;
	bool enabled = true;
};

struct OcrPatternConfig {
	bool enabled = false;
	double scoreThreshold = 70.0; //MIL score 0-100
	QVector<OcrPatternLabel> labels;
};

// ── OCR inspection (core of IM430 ocrInspection2) ───────────────────────────

struct AlgoOcrParams {
	int orientation = 0;          //0 / 90 / 180 / 270
	int roi1Rows = 1;
	int roi1Columns = 0;          //0 = use PaddleOCR charCount; >0 = fixed column split
	bool removeSpecialChars = false;
	bool paddleOcrEnabled = true; //false = rows/columns grid + pattern matching only

	//geometries captured from the UI drag boxes at run time (FOV px)
	QRectF roi1Geo;
};

struct AlgoOcrOutput {
	bool ok = false;
	QString message;
	QString roi1Text;  //rows joined with ','
	QString roi1Key;   //first token per row, joined (IM430's m_inspBar.start)
	qint64 elapsedMs = 0;
	QVector<AlgoOverlayItem> overlay;
};

// ── Locator (QAlgoLocator style) ────────────────────────────────────────────

struct AlgoLocatorConfig {
	bool enabled = false;
	double scoreThreshold = 70.0;
	double searchAngle = 10.0;   //± degrees (baked into the .mpat at learn time)
	double angleOffset = 0.0;    //manual tuning offset added to found delta angle
	double maskMarginW = 0.0;    //% of pattern width kept as border (interior filled with mean)
	double maskMarginH = 0.0;
	QString modelPath;           //.mpat, absolute in memory / recipe-relative on disk
	double learnX = 0;           //model centre at learn time (FOV px, from full-image find)
	double learnY = 0;
	double learnAngle = 0;
	QRectF learnRoi;             //locator learn ROI geometry at learn time
	QRectF searchRoi;            //region searched at run time (empty = whole image)
};

struct AlgoLocatorResult {
	bool ran = false;     //locator enabled and attempted
	bool found = false;
	double score = 0.0;
	double deltaX = 0.0;  //found - learn (FOV px)
	double deltaY = 0.0;
	double deltaAngle = 0.0;
	QPointF foundPos;
};

// ── 3D height measurement (QAlgoHeightMeasurement plane-fit style) ──────────


struct AlgoHeightParams {
	double intensityPerMicron = 11.0; //gray levels per um (Algo convention: raw / ipm = um)
	double minHeightUm = 0.0;         //pass/fail limits; both 0 = no limit
	double maxHeightUm = 0.0;
	bool removeOutliers = true;       //plane-residual outlier rejection before the datum fit
	QVector<QRectF> planeRois;        //datum plane-fit ROIs (any number, at least 3 to fit)
	QVector<QRectF> heightRois;       //measurement ROIs (each measured against the datum plane)
};

//one measurement ROI's result, relative to the shared datum plane
struct AlgoHeightRoiResult {
	QRectF roi;
	double avgHeightUm = 0.0;
	double minHeightUm = 0.0;
	double maxHeightUm = 0.0;
	bool pass = false;
	bool valid = false; //false = no valid pixels in the ROI
};

struct AlgoHeightOutput {
	//NOTE: registered as a queued-connection metatype in AlgoManager::init()
	bool ok = false;
	QString message;
	bool planeValid = false;
	double planeA = 0, planeB = 0, planeC = 0, planeD = 0;
	double tiltXDeg = 0.0;
	double tiltYDeg = 0.0;
	QVector<AlgoHeightRoiResult> roiResults; //one per measurement ROI
	bool pass = false;                       //all ROIs pass
	qint64 elapsedMs = 0;
	QVector<AlgoOverlayItem> overlay;
};

#include <QMetaType>
Q_DECLARE_METATYPE(AlgoOcrOutput)
Q_DECLARE_METATYPE(AlgoHeightOutput)
