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
* OCR types follow IM430's ocrInspection2 core; the locator follows QAlgoLocator.
* 3D height measurement has its own types in AlgoHeight3Types.h.
*/

/*
* Which algo the setup page combobox is on (name avoids the Algo lib's AlgoType).
*
* The value is the combo row AND the stacked page AND what the Template Library saves as a
* template's "algo" - so it is persisted, and only ever APPEND. HEIGHT_3D_V3 is 1 because 1 has
* always meant "3D Height Measurement" in a saved template: it used to be the old plane-fit algo
* (V1), and every template that linked to it now links to V3 with nothing to convert.
*/
enum class AlgoPageAlgo {
	OCR_READ = 0,
	HEIGHT_3D_V3 = 1      //AlgoHeight3Pipeline, no locator
};

/*
* How many algos own a Locator config: OCR only. HEIGHT_3D_V3 deliberately does NOT - its
* segmentation stage locates the part itself, on the height data rather than on an 8-bit render
* of it, and returns width, height and angle instead of only a match score.
*
* AlgoPageAlgo indexes AlgoManager::m_locator[] directly, and the enum is LARGER than that
* array. Anything converting an AlgoPageAlgo to an index must bounds-check against this, or
* selecting the V3 page writes past the end of the array - silent memory corruption, not a
* crash, because captureAlgoParamsFromUI() runs on both Save and Run.
*/
constexpr int kAlgoLocatorSlots = 1;

inline bool algoHasLocator(AlgoPageAlgo a)
{
	const int i = (int)a;
	return i >= 0 && i < kAlgoLocatorSlots;
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
	//Rect / Polygon outline. Image pixels by default, so it scales with the zoom; a cosmetic
	//pen is in SCREEN pixels instead and keeps its width however far the image is shrunk
	qreal penWidth = 2.0;
	bool cosmeticPen = false;

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
	bool superResolution = false; //4x Real-ESRGAN on small crops (runs in the Paddle server)

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
	QImage srImage; //super-resolved crop sized back to the ROI, for display (null = SR did not run)
	QRectF srRect;  //where srImage belongs in the FOV
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

#include <QMetaType>
Q_DECLARE_METATYPE(AlgoOcrOutput)
