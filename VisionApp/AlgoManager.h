#pragma once

#include <QObject>
#include <QThread>
#include <QHash>
#include <QStringList>
#include <mutex>
#include <atomic>
#include <opencv2/opencv.hpp>

#include "AlgoSetupTypes.h"
#include "mtrx.h"
#include "MbufWrapper.h"

class PaddleOcrClient;

/*
* Runs the Algo Setup page's inspections on a dedicated worker thread so the
* UI stays live and production can trigger runs in the background.
*
* Algos:
*  - OCR inspection: port of IM430 ocrInspection2's core — PaddleOCR text
*    detection (python server via PaddleOcrClient) plus per-character MIL
*    pattern-matching correction from a learned pattern library. With
*    PaddleOCR disabled, the ROI is grid-split by rows/columns and read by
*    pattern matching alone.
*  - 3D height measurement: plane-fit datum ROIs + height ROI relative to the
*    fitted plane, following the Algo project's QAlgoHeightMeasurement.
*
* An optional locator (QAlgoLocator style) runs before either algo and shifts
* the ROIs by the found offset; when no locator is assigned the algo runs on
* the ROIs as-is.
*
* Results come back on signals; the payload carries overlay primitives for
* the UI thread to draw. Settings persist per recipe in algoSetup.json and
* ocr_patterns/config.json.
*/
class AlgoManager : public QObject {
	Q_OBJECT

public:
	static AlgoManager& instance();

	void init();     //start worker thread (call once from GUI thread)
	void release();

	//── config (thread-safe copies) ──
	AlgoOcrParams ocrParams() const;
	void setOcrParams(const AlgoOcrParams& p);

	AlgoHeightParams heightParams() const;
	void setHeightParams(const AlgoHeightParams& p);

	AlgoLocatorConfig locatorConfig(AlgoPageAlgo algo) const;
	void setLocatorConfig(AlgoPageAlgo algo, const AlgoLocatorConfig& cfg);

	OcrPatternConfig patternConfig() const;
	void setPatternEnabled(bool enabled);
	void setPatternThreshold(double threshold);
	void setPatternLabelEnabled(const QString& label, bool enabled);

	//── per-recipe persistence ──
	bool loadRecipeConfig(); //call on recipe open
	bool saveRecipeConfig();

	//── pattern library (UI thread; internally guarded) ──
	bool learnPatternSample(const QImage& fov, const QRectF& roi, const QString& label, QString& error);
	bool deletePatternSample(const QString& label, int sampleIndex);

	//── locator learn ──
	bool learnLocatorModel(AlgoPageAlgo algo, const QImage& fov, const QRectF& learnRoi, QString& error);

	//── heightmap source ──
	void setHeightMap(mtrx::SharedMilID heightMap);         //e.g. last scanned map
	bool loadHeightMapFromFile(const QString& tiffPath, QString& error);
	mtrx::SharedMilID heightMap() const;
	QImage heightMapImage(bool colorMapped) const;          //for 2D/3D display

	//── runs (queued to the worker thread; results come by signal) ──
	void runOcr(const QImage& fov);
	void runHeight();

	bool isBusy() const { return m_busy; }

signals:
	void ocrFinished(AlgoOcrOutput output);
	void heightFinished(AlgoHeightOutput output);
	void busyChanged(bool busy);
	void patternsChanged();

private slots:
	void doRunOcr(QImage fov);
	void doRunHeight();

private:
	AlgoManager();
	~AlgoManager();
	AlgoManager(const AlgoManager&) = delete;
	AlgoManager& operator=(const AlgoManager&) = delete;

	//OCR internals (worker thread)
	struct OcrRoiTransform {
		QRectF roiGeo;      //crop geometry in FOV space
		double scale = 1.0; //scale applied before OCR (currently always 1)
		QPoint canvasPad;   //centering pad applied before OCR
		int rotation = 0;   //0/90/180/270 applied before OCR
	};
	QPointF ocrToFov(const QPointF& pt, const OcrRoiTransform& t) const;
	QVector<AlgoOcrBox> runOcrOnRoi(const cv::Mat& fovBgr, const QRectF& roiGeo, int rows, int cols,
		const AlgoOcrParams& param, OcrRoiTransform& transform, QVector<AlgoOverlayItem>& overlay);
	void applyPatternMatching(const cv::Mat& fovGray, QVector<AlgoOcrBox>& results,
		int startIdx, int endIdx, int columnsOverride, const AlgoOcrParams& param,
		const OcrRoiTransform& transform, QVector<AlgoOverlayItem>& overlay);

	//locator (worker thread) — returns offset applied to ROI geometries
	AlgoLocatorResult runLocator(const cv::Mat& fovBgr, const AlgoLocatorConfig& cfg,
		QVector<AlgoOverlayItem>& overlay);

	//pattern library storage
	QString patternDir() const;
	void loadPatterns();
	void savePatterns();

	QString algoConfigPath() const;

	QThread m_thread;
	bool m_initialized = false;
	std::atomic<bool> m_busy = false;

	mutable std::mutex m_mutex; //guards params, configs, pattern maps, heightmap

	AlgoOcrParams m_ocrParams;
	AlgoHeightParams m_heightParams;
	AlgoLocatorConfig m_locator[2]; //indexed by AlgoPageAlgo

	OcrPatternConfig m_patternConfig;
	QHash<QString, QStringList> m_patternFiles; //label -> .mpat paths

	mtrx::SharedMilID m_heightMap;

	PaddleOcrClient* m_paddle = nullptr; //created on worker thread
};
