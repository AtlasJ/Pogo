#pragma once

#include <QObject>
#include <QThread>
#include <QHash>
#include <QMap>
#include <QJsonObject>
#include <QStringList>
#include <mutex>
#include <atomic>
#include <opencv2/opencv.hpp>

#include "AlgoSetupTypes.h"
#include "AlgoHeight3.h"
#include "mtrx.h"
#include "MbufWrapper.h"

class PaddleOcrClient;
class AlgoManager;

/*
* V3's own worker: a queue on a thread of its own, so its stages never wait behind anything on
* the shared algo thread - above all the PaddleOCR warm-up, which can hold that thread for two
* minutes after every launch (and in production a 3D unit gives up after 60 s). A QObject only
* so a queued call can be posted to its thread; the work itself is AlgoManager's.
*/
class AlgoH3Runner : public QObject {
	Q_OBJECT
public:
	explicit AlgoH3Runner(AlgoManager* mgr) : m_mgr(mgr) {}
public slots:
	void run(int stage);
	void runUpTo(int stage);
	void runNamed(QString algoName);
private:
	AlgoManager* m_mgr;
};

//one pixel of the 3D maps behind the FOV, for the hover readout
struct FovMapPixel {
	int raw = 0;                //16-bit height; 0 is a dropout
	bool hasIntensity = false;  //false when there is no intensity map that lines up
	int intensity = 0;
	bool hasDatum = false;      //true only on the V3 crop with a fitted datum and a valid pixel
	double aboveDatumUm = 0.0;  //exactly what Measure computes for this pixel
};

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
*  - 3D Height Measurement 3 (AlgoHeight3Pipeline): five stages from the raw
*    height map to a unit verdict, on a thread of its own - see AlgoHeight3.h.
*
* An optional locator (QAlgoLocator style) runs before OCR and shifts the ROI
* by the found offset; when no locator is assigned OCR runs on the ROI as-is.
*
* Results come back on signals; the payload carries overlay primitives for
* the UI thread to draw. OCR settings persist per recipe in algoSetup.json and
* ocr_patterns/config.json; V3's are machine-wide named algos (see below).
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

	//the algo the V3 page is tuning (height3EditAlgo); defaults while the library is empty
	AlgoHeight3Params height3Params() const;
	void setHeight3Params(const AlgoHeight3Params& p);

	/*
	* ── V3's named algos: machine-wide, not per recipe ──
	*
	* One file per algo, <config>/algo_height3/<name>.json, each holding EVERY V3 setting, plus
	* _default.json naming the default. Production runs the algo whose name is the unit's cable
	* name from the PMF (ignoring case), and the default when there is none. Names are matched
	* and kept unique ignoring case, and have to be usable as a file name.
	*
	* The first launch with no algo files imports every recipe's old "height3" block as an algo
	* named after the recipe, the current recipe's as the default - so production measures
	* exactly as before until cable-named algos exist. GUI thread only, except the lookups.
	*/
	QStringList height3AlgoNames() const;          //sorted, as typed
	QString height3DefaultAlgo() const;
	QString height3EditAlgo() const;
	static QString height3AlgoDir();
	//"" when the name is usable for a new algo, else why not (rename passes its own old name)
	QString height3AlgoNameProblem(const QString& name, const QString& renaming = QString()) const;
	bool height3SelectAlgo(const QString& name, QString& error);  //the page tunes this one now
	bool height3CreateAlgo(const QString& name, QString& error);  //a copy of the one being tuned
	bool height3RenameAlgo(const QString& newName, QString& error);  //the one being tuned
	bool height3DeleteAlgo(const QString& name, QString& error);  //never the default
	bool height3SetDefaultAlgo(const QString& name, QString& error);
	bool saveHeight3Algo();   //writes the tuned algo's file, only when it changed
	//the algo production runs for a cable: its own, else the default (usedDefault), else ""
	QString height3AlgoForCable(const QString& cable, bool* usedDefault = nullptr) const;

	AlgoLocatorConfig locatorConfig(AlgoPageAlgo algo) const;
	void setLocatorConfig(AlgoPageAlgo algo, const AlgoLocatorConfig& cfg);

	OcrPatternConfig patternConfig() const;
	void setPatternEnabled(bool enabled);
	void setPatternThreshold(double threshold);
	void setPatternLabelEnabled(const QString& label, bool enabled);

	//── per-recipe persistence (OCR; the first call also loads V3's algo library) ──
	bool loadRecipeConfig(); //call on recipe open
	bool saveRecipeConfig();

	//── pattern library (UI thread; internally guarded) ──
	bool learnPatternSample(const QImage& fov, const QRectF& roi, const QString& label, QString& error);
	bool deletePatternSample(const QString& label, int sampleIndex);

	//── locator learn ──
	bool learnLocatorModel(AlgoPageAlgo algo, const QImage& fov, const QRectF& learnRoi, QString& error);

	//── the last scan's maps: production's display, the FOV readout, V3's "Use Last Scan" ──
	void setLastScanMaps(mtrx::SharedMilID heightMap, mtrx::SharedMilID intensityMap);
	mtrx::SharedMilID heightMap() const;
	mtrx::SharedMilID lastScanIntensityMap() const;
	QImage heightMapImage(bool colorMapped) const;          //for 2D/3D display
	//the pixel under the FOV cursor while heightMapImage() is shown; false outside the map
	bool lastScanPixel(int x, int y, FovMapPixel& out) const;

	// ── 3D Height Measurement 3 (AlgoHeight3Pipeline) ──
	//sources: either the last scan's pair, or files loaded on the page for offline work
	bool height3UseLastScan(QString& error);

	/*
	* Production entry point: take V3's source maps straight from the frame being inspected.
	* Deliberately NOT height3UseLastScan() - that reads whatever the GUI thread last stored,
	* which is a race against the next unit's scan. `note` carries a non-fatal remark (e.g. a
	* mismatched intensity map that was dropped); false means the height map was unusable.
	*/
	bool height3SetSourceMaps(mtrx::SharedMilID heightMap, mtrx::SharedMilID intensityMap,
		QString& note);
	bool height3LoadHeightFile(const QString& path, QString& error);
	bool height3LoadIntensityFile(const QString& path, QString& error);
	void height3Clear();

	bool height3HasHeight() const;
	bool height3HasIntensity() const;
	bool height3PreprocessReady() const;
	bool height3SegmentReady() const;
	bool height3DatumReady() const;
	//Place one ROI per pin. seed = a taught ROI to lay the grid out from, or null.
	bool height3FindPins(const QRectF* seed, AlgoH3PinFind& out, QString& why) const;

	/*
	* Put a reopened algo back where it left off: run every stage up to and including the
	* furthest one that passed when it was last saved. -1 when nothing had.
	*
	* Only the stage NUMBER is kept in the algo file, never the results - see
	* AlgoHeight3Pipeline::furthestPassed for why.
	*/
	int height3SavedStage() const { return m_height3SavedStage; }
	//how far the PAGE last got - what the algo's last_stage records. Set only when a run the
	//operator started on the page finishes; every save writes it (see saveHeight3Algo)
	void setHeight3LastStage(int stage) { m_height3LastStage = stage; }
	void runHeight3UpTo(int stage);
	QSize height3CropSize() const;
	AlgoHeight3Output height3Output() const;

	//the pixel under the FOV cursor on a flat V3 view; false outside the map, and while a stage
	//holds the pipeline - it never waits, because it runs on every mouse move
	bool height3Pixel(bool preprocessed, bool segmented, int x, int y, FovMapPixel& out) const;

	//display renders, taken under the same lock the worker writes under
	QImage height3Image(bool intensity, bool preprocessed, bool segmented, bool colorMapped) const;
	QImage height3Surface(bool preprocessed, bool segmented,
		double yawDeg, double pitchDeg, double zExaggeration, const QSize& outSize,
		AlgoH3SurfaceStyle style = AlgoH3SurfaceStyle::Filled,
		bool fillHoles = true,
		//draw the fitted datum over the part. Only means anything on the segmented view -
		//the plane is fitted in the crop's coordinates and has no place on the full map.
		bool showPlane = false) const;
	QImage height3Relief(bool preprocessed, bool segmented,
		double zExaggeration, bool colorMapped) const;

	/*
	* A DEEP copy of the height map the page is showing, for handing to another thread or
	* process (the external 3D display). try_lock like the renders above: false means a stage
	* is running right now; true with an empty `out` means no map is loaded.
	*/
	bool height3CopyDisplayHeight(bool preprocessed, bool segmented, cv::Mat& out) const;

	//── runs (queued to the worker thread; results come by signal) ──
	void runOcr(const QImage& fov);
	void runHeight3(AlgoH3Stage stage);           //the algo the page is tuning
	void runHeight3Named(const QString& algoName); //production: a Run All of that algo

	bool isBusy() const { return m_busyCount > 0; }

signals:
	void ocrFinished(AlgoOcrOutput output);
	void height3Finished(int stage, AlgoHeight3Output output);
	void busyChanged(bool busy);
	void patternsChanged();

private slots:
	void doRunOcr(QImage fov);
	void doRunHeight3(int stage);
	void doRunHeight3UpTo(int stage);
	void doRunHeight3Named(QString algoName);

private:
	friend class AlgoH3Runner;   //calls doRunHeight3 / doRunHeight3UpTo on V3's own thread

	AlgoManager();
	~AlgoManager();
	AlgoManager(const AlgoManager&) = delete;
	AlgoManager& operator=(const AlgoManager&) = delete;

	//a COUNT, not a flag: OCR and V3 run on different threads, and one finishing must not
	//report the manager idle while the other is still going
	void setBusy(bool on);

	//OCR internals (worker thread)
	struct OcrRoiTransform {
		QRectF roiGeo;      //crop geometry in FOV space
		double scale = 1.0; //scale applied before OCR (currently always 1)
		QPoint canvasPad;   //centering pad applied before OCR
		int rotation = 0;   //0/90/180/270 applied before OCR
	};
	QPointF ocrToFov(const QPointF& pt, const OcrRoiTransform& t) const;
	QVector<AlgoOcrBox> runOcrOnRoi(const cv::Mat& fovBgr, const QRectF& roiGeo, int rows, int cols,
		const AlgoOcrParams& param, OcrRoiTransform& transform, QVector<AlgoOverlayItem>& overlay,
		QImage* srImage = nullptr);
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

	//V3's algo library (AlgoManager_Height3.cpp)
	struct H3Algo {
		QString name;              //as typed; the map key is its lower case
		AlgoHeight3Params params;
		int lastStage = -1;
		QJsonObject savedJson;     //what its file holds, so a save that changes nothing writes nothing
	};
	void loadHeight3Library();
	void importHeight3FromRecipes();
	bool writeHeight3Algo(const QString& name, const QJsonObject& json, QString& error); //GUI thread, no lock held
	bool writeHeight3Default(const QString& name, QString& error);
	void selectHeight3AlgoLocked(const QString& key);   //m_mutex held
	//params of the algo whose results the pipeline holds, for drawing them (else the tuned one)
	AlgoHeight3Params height3DisplayParams() const;
	QString height3EditSnapshot(AlgoHeight3Params& p) const;
	//run on V3's thread: the stage results it leaves belong to `algoName`
	AlgoHeight3Output runHeight3Locked(AlgoH3Stage stage, bool upTo, const AlgoHeight3Params& p,
		const QString& algoName);

	QThread m_thread;          //OCR and its PaddleOCR warm-up
	QThread m_height3Thread;   //V3 only - see AlgoH3Runner
	AlgoH3Runner* m_height3Runner = nullptr;
	bool m_initialized = false;
	std::atomic<int> m_busyCount{ 0 };

	mutable std::mutex m_mutex; //guards params, configs, pattern maps, heightmap

	AlgoOcrParams m_ocrParams;
	AlgoLocatorConfig m_locator[kAlgoLocatorSlots]; //indexed by AlgoPageAlgo - see algoHasLocator

	OcrPatternConfig m_patternConfig;
	QHash<QString, QStringList> m_patternFiles; //label -> .mpat paths

	mtrx::SharedMilID m_heightMap;         //the last scan's height map
	mtrx::SharedMilID m_lastScanIntensity; //its companion, for "Use Last Scan"

	/*
	* 3D Height Measurement 3. The algos live under m_mutex like every other config,
	* but the PIPELINE gets its own lock: a stage can run for seconds, and holding the
	* config mutex that long would stall every unrelated caller. The display accessors
	* only try_lock it, so dragging the 3D view while a stage runs keeps the last frame
	* instead of freezing the GUI thread. The two are never held together.
	*/
	QMap<QString, H3Algo> m_height3Algos;   //key: name in lower case
	QString m_height3DefaultKey;
	QString m_height3EditKey;               //the algo the page tunes
	QString m_height3ResultKey;             //the algo the pipeline's results came from; "" = none
	bool m_height3LibraryLoaded = false;
	QJsonObject m_recipeHeight3Block;       //the recipe's old block, written back untouched
	mutable std::mutex m_height3Mutex;
	int m_height3SavedStage = -1;   //from the algo file; -1 = nothing had passed
	std::atomic<int> m_height3LastStage{ -1 };   //written as last_stage; GUI sets, saves read
	AlgoHeight3Pipeline m_height3;

	PaddleOcrClient* m_paddle = nullptr; //created on worker thread
};
