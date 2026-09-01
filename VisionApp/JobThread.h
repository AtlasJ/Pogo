#pragma once
#include <QCoreApplication>
#include <QThread>
#include <QVector>
#include <QMutex>
#include <QWaitCondition>
#include <QSet>
#include <QHash>
#include <QListWidget>
#include <QCoreApplication>
#include <opencv2/opencv.hpp>
#include <queue>
#include <QObject>
#include "QView.h"
#include "QViewPlane.h"
#include "QLineScan.h"
#include "OpticsInfo.h"
#include <QDebug>
#include "WinEvents.h"
#include "Fiducial.h"
#include "FiducialInfo.h"
#include "BarcodeInfo.h"
#include "Utilities.h"
#include "PortabilityInfo.h"
#include "QServer.h"
#include "FrameInfo.h"
#include "MessageQue.h"
#include "Logger.h"
#include "SystemData.h"
#include "MotionController.h"
#include <QTcpSocket>
#include <QByteArray>
#include <QElapsedTimer>
#include <QVector3D>

#include "LSCManager.h"

#define PROFILER_TIMEOUT 60000

//feature-finding setup for camera alignment/scaling: circle or learned pattern
struct AlignFeatureParams {
	bool usePattern = false;

	//circle: expected diameter comes from the on-screen ROI +- tolerance
	int minDiameter = 0;
	int maxDiameter = 0;
	mtrx::ForegoundType foreground = mtrx::ForegoundType::FOREGROUND_WHITE;

	//pattern
	QString modelPath;      //learned .mpat
	QRectF searchRoi;       //px, empty = whole FOV
	double minScore = 70.0;
};
Q_DECLARE_METATYPE(AlignFeatureParams)

Q_DECLARE_METATYPE(InspStatus::FiducialDetail)
Q_DECLARE_METATYPE(MIL_ID)
Q_DECLARE_METATYPE(LaserAlignmentImage)
class JobThread : public QThread {
	Q_OBJECT

public:

	JobThread(QObject* parent = nullptr) : QThread(parent) {
	}
	~JobThread() {
		// The SR-X sockets live in this thread, so they must NOT be deleted from
		// here (the destructor runs on the caller's thread). Stop the event loop
		// and let run() tear them down, then wait for it to finish.
		quit();
		if (!wait(3000)) {
			ct::logger::error("[JobThread] Thread did not exit within 3s; SR-X sockets left in place");
		}

		// Existing V1 cleanup
		if (m_server) {
			// cleanup server if needed
		}
	}

	void run() override;
	void release();
	
	//revisit
	void attach(QListWidget* viewSequence);
	void attach(Fiducial* fiducialAlgo);
	void attach2ndFiducial(Fiducial* fiducialAlgo);
	void attach(std::vector<FiducialInfo>* fiducialInfos);
	void attach(std::array<BarcodeInfo, 2>* barcodeInfos);
	void attach(InspStatus* inspStatus);
	void attach(CSAInfo* csa);
	void attach(QHash <QString, QView>* views, QHash<QString, OpticsInfo>* optics2D);
	void attach(QHash <QString, QLineScan>* linescans, QHash<QString, OpticsInfo3D>* optics3D); 
	void attach(dat::WorldCoordinate* laserOffset);
	void attach(QViewPlane* viewPlane);
	void attach(PortabilityInfo* portabilityInfo);
	void setRootPath(QString rootPath); 
	void setWarpageMethod(QString method);
	void setXSpeed(int speed, int speed3d);
	void setXDecel(int decel);
	void getXSpeed(int& speed, int& speed3d);

	void enableFiducial(bool enable);
	void enableBarcode(bool enable);
	void enableWarpageCompensation(bool enable);
	void enableRun1stFOVOnly(bool enable);

	void resetFiducial();

	//job
	void stopRun();

	// SR-X barcode readers are owned by SRXManager; these thin wrappers keep the
	// existing queued signal connections working
	void triggerSRX();
	void stopSRX();

	//3D Optics alignment: continuous single-profile polling for the live graph.
	//Runs on a timer in this thread's event loop so jogs interleave between ticks.
	void liveProfile(bool enable);
private:
	TimeLogger m_timeLogger;

	bool m_imageReadyFlag = false;
	bool m_imagePreprocessedFlag = false;
	bool m_run1stFOVOnly = false;

	bool m_encoderCheck = false;

	int m_xDecel = 1500;
	int m_xSpeed = 300;
	int m_xSpeed3d = 20;
	int m_motionReadDelay_ms = 150;
	int m_index = 0;

	struct PostResult {
		FrameInfo frame;
		QString bufferPath;
	};
	QVector<PostResult> m_postResults;

	MIL_ID m_buffers;

	WinEvents m_appEvents;
	QServer* m_server = nullptr;

	TimeLogger m_timer;

	QString m_camID = "cam1";
	QString m_motionID = "motion1";
	QString m_profilerID = "profiler1";
	int m_camTriggerIO = 2;
	int m_camResetIO = 1;
	bool m_lscFastMode = false;

	QTimer* m_liveProfileTimer = nullptr; //created lazily in the job thread
	void liveProfileTick();

	QListWidget* m_viewSequence = nullptr;
	Fiducial* m_fiducialAlgo = nullptr;
	Fiducial* m_fiducialAlgo2 = nullptr;   // second island (fid3/fid4) when double fiducial checking is enabled
	std::vector<FiducialInfo>* m_fiducialInfos = nullptr;
	std::array<BarcodeInfo, 2>* m_barcodeInfos = nullptr;
	InspStatus* m_inspStatus = nullptr;
	dat::WorldCoordinate* m_laserOffset = nullptr;
	CSAInfo* m_csa = nullptr;
	QViewPlane* m_viewPlane = nullptr;
	PortabilityInfo* m_portabilityInfo = nullptr;
	QString m_rootPath = "";


	QHash <QString, QView>* m_views = nullptr;
	QHash <QString, QLineScan>* m_linescans = nullptr;

	QHash <QString, OpticsInfo>* m_optics = nullptr;
	QHash <QString, OpticsInfo3D>* m_optics3D = nullptr;

	QHash<QString, RGBOffset> m_rgbOverrides;

	std::thread m_thread;
	int m_snapDelay_ms = 0;
	std::atomic<bool> m_stopZstack = false;

	//scale
	double m_scaleStep;
	
	//jog
	int m_minDiameter, m_maxDiameter;
	dat::WorldCoordinate m_coordinate;
	QString m_type;
	OpticsInfo m_optic;
	mtrx::ForegoundType m_foregroundType = mtrx::ForegoundType::FOREGROUND_ANY;

	QImage m_qimg;
	int m_camThreshold, m_laserThreshold;
	QRectF m_roi;
	int m_method = 0;

	//booleans
	bool m_stopRun = false;
	bool m_enableWarpageCompensation = false;

	//fid
	bool m_enableFiducial = false;
	QSet<QString> m_locatedFidID;
	int m_currentFidIndex = 0;
	bool m_testFidOnline = true;

	//barcode
	bool m_enableBarcode = false;
	int m_currentBarcodeIndex = 0;
	bool m_testBarcodeOnline = true;

	void jogBasedOnFiducial(double x, double y, double z, QString type, bool forceEnable = false);
	void jogView(const QView& view, double z_offset = 0.0);
	void jogLaser(double x, double y, double z, QString type);
	void jogLaserBasedOnFiducial(double x, double y, double z, QString type, bool forceEnable = false);


	void switchToContinuousModeLSC();
	void switchToFastModeLSC();
	const OpticsInfo& getMainOptics();
	const OpticsInfo3D& getMainOptics3D();
	void snapBand(const OpticsInfo& optic, QString viewID, QString stitchID, BandType bandType); //wait for raw image
	//void snapOptic(QString camID, const OpticsInfo& optic);
	void snapView(QString viewID, bool resetFrame = true);
	void triggerCamera(QString camID);

	FrameInfo scan(QString preID, dat::WorldCoordinate start, dat::WorldCoordinate end, const OpticsInfo3D& optic, bool waitImage = true);

	//camera process
	double m_cameraAlignment = 0.0;

	//fiducial
	bool fiducialExists(int index);
	em::V2d getFiducialPointInMM(int index, int x_px, int y_px);
	bool locateFiducial(int index, int fidIndex, InspStatus::FiducialDetail& fDetail, bool saveImg,const dat::WorldCoordinate & curCoordinate, int crossFinderScore = 85, Fiducial* algo = nullptr);
	void searchFiducial();
	void searchDoubleFiducial();
	Fiducial* fiducialForPoint(double x, double y);   // routes a target point to the nearest island's transform
	void saveFiducialResult();

	//barcode
	const QString msg_failed_barcode = "Fail_to_read_barcode";
	bool barcodeExists(int index);
	void searchBarcode();

	/*
	Warpage compensation concept
	1. Teach focus is to obtain the ideal offset of camera to laser. Ideal being that the laser scan should give an average profile of 0
	2. Therefore the offset to compensate the warpage is -average. Since for gantry, z up is negative while z down is positive.
	*/
	QString m_warpageMethod = "None";
	QHash<QString, double> m_compensateZMap;
	void warpageCompensation();
	void subWarpageCompensation();
	void fullWarpageCompensation();
	void generateWarpageMap();
	void centerLaserZ();


	//Color compensation
	void colorCompensation();

	//Portability
	/*QPointF m_positionPortabilityPatternSize = QPointF();
	int m_num_of_Z_Offset_Performed = 0;
	dat::WorldCoordinate m_positionPortabilityPoint;
	bool m_donePortability = false;
	PositionPortabilityInfo mSystemData::instance()._portability.ref_info;
	PositionPortabilityInfo mSystemData::instance()._portability.current_info;*/
	em::V2d getPositionPortabilityPointInMM(int x_px, int y_px);
	bool testPortabilityPatternFeature(QRectF& outputFeature);
	bool testPortabilityCircleFeature(QRectF& outputFeature);
	bool getCorrectedPortabilityPoint(dat::WorldCoordinate& portabilityPoint, QPointF& PatternSize);
	bool getPortabilitySizeDifference(double difference, double& offsetZ);
	bool savePositionPortabilityInfo(PositionPortabilityType type);
	void getCurrentMachinePortabilityPointOffset(dat::WorldCoordinate& offset);
	void toJson(const dat::WorldCoordinate& obj, QJsonObject& j, bool isRelative = false);
	void fromJson(const QJsonObject& j, dat::WorldCoordinate& obj, bool isAbsolute = false);
	void toJson(const ct::Box2D& obj, QJsonObject& j);
	void fromJson(const QJsonObject& j, ct::Box2D& obj);
	dat::WorldCoordinate getRelativeRobotPoint(dat::WorldCoordinate point);
	dat::WorldCoordinate getAbsoluteRobotPoint(dat::WorldCoordinate point);

	//Acquisition
	void preAcquisition();
	void postAcquisition();
	void continuousSnap();
	void savePostResult();
	void acquire2DImages();
	bool acquireBarcodeAndOcr(); //SR-X barcode read + OCR-side capture (pitch unit grid, or 3D mid point in plane mode)
	bool acquireBarcodeAndOcrAt(double baseX, double baseY, double baseZ, const QString& unitID = QStringLiteral("board")); //one read cycle at a base camera position
	void acquire3DImagesPitch(); //pitch mode: one scan of the recipe scan length centered on each unit
	void acquire3DImages();
	void collectPlane();

	void save3DExtraOffset();

	//Lighting
	int m_expectedRedGV, m_expectedGreenGV, m_expectedBlueGV; 
	QString m_opticType = "";
	int getIntensityFromIdealGV(QString camID, QString channel, double idealGV);
	int getIntensityFromIdealGV(QString camID, const QVector<QString>& channels, double idealGV);
	void calibrateOptimumBrightness(QString camID);
	void getGVTable(QString camID, GVTable& gvt, QRectF roi);

	//Communication
	TMessageQue<QByteArray> m_byteArrayQue;

	QString m_currentTriggerSequence = "";
	QString m_expectedViewID;
	QString m_expectedIndexID;
	QVector<FrameInfo> m_frameInfos;
	void saveFrame(QString rootPath, QVector<FrameInfo> frames);

	bool updateTriggerSequence(QString viewID1, QString viewID2);
	bool updateTriggerSequence(QString viewID);
	bool appendSequence(const OpticsInfo& optic, QVector<LSCManager::SequenceData>& datas);

	bool m_isTest = false;
	bool m_isSetup = false;


	void getEncoder(const QString& data);

	MIL_ID preprocessImage(MIL_ID mColor);

	void clearEmptyView();
	void clearEmptyLineScan();

	bool safeGuardView();
	bool safeGuardLineScan();

	QRectF m_locatedPortabilityPos;

	QStringList m_extraMoveLog;
	QString m_laserOffsetInfo;

public slots:
	void incomingJob(QByteArray);
	bool sendToClient(QString msg);

	void snapOptic(const OpticsInfo& optic, QString viewID, QString stitchID, bool resetFrame = true);
	void snapOpticFastMode(const OpticsInfo& optic, QString viewID, QString stitchID, bool resetFrame = true);

	//wait func
	QPointF waitForLocator(QString viewID);
	QPointF waitForLocator(QString viewID, QString indexID);
	QPointF waitForLocator(QString viewID, int row, int col);

	ct::UnitResultInfo waitForUnitResult(QString viewID, QString indexID);
	QVector<FrameInfo> waitForImageReady();
	FrameInfo waitForImagePreprocessed();
	FrameInfo waitForImagePreprocessed(int timeoutMs);

	void waitAxis(int axis);
	void waitEncoderCheck(double x_mm, double y_mm, double z_mm);
	void jog(double x, double y, double z, QString type = "2D", bool waitJogDone = true);
	void jogUser(double x, double y, double z, QString type = "2D", bool waitJogDone = true); //user-initiated: clears a stale stop flag first
	void dryRun(QVector<QVector3D> coords, int loops);
	void reconnectMotion();
	void jogSnap(double x, double y, double z, const OpticsInfo& optic);
	void jogLeft(double mm, const OpticsInfo& optic);
	void jogRight(double mm, const OpticsInfo& optic);
	void jogBack(double mm, const OpticsInfo& optic);
	void jogFront(double mm, const OpticsInfo& optic);
	void jogUp(double mm, const OpticsInfo& optic);
	void jogDown(double mm, const OpticsInfo& optic);

	void homeX();
	void homeY();
	void homeZ();
	void homeXYZ();
	void homeAll();
	//setup
	void autoSetFiducialPoint(int currentFid);
	void testFiducial(int index, bool online);

	QString readBarcode(int index, bool online = true);

	//calibration
	void performCameraAlignment(dat::WorldCoordinate currentPoint, double step_mm, AlignFeatureParams featureParams);
	void performCameraScaling(dat::WorldCoordinate currentPoint, double step_mm, AlignFeatureParams featureParams);
	bool findAlignFeature(MIL_ID mMono, const AlignFeatureParams& p, mtrx::PatternOutput& out);

	void performLaserAlignment(dat::WorldCoordinate currentPoint, QRectF roi, int camThreshold, int laserThreshold);
	void captureAlignmentImages(dat::WorldCoordinate currentPoint, int camThreshold, int laserThreshold);
	void performGuidedLaserAlignment(dat::WorldCoordinate currentPoint);
	void verifyLaserAlignment(dat::WorldCoordinate currentPoint);

	/*
	* Profiler Scan Test - Test Run page, online run type "Profiler Scan Test".
	*
	* A 3D-only acquisition with no 2D camera in the loop, no recipe views and no production
	* error handling. Scans from wherever the gantry is now, distance_mm along X in the chosen
	* direction, and writes a full settings-and-results report under the current recipe.
	*
	* Deliberately NOT built on scan(): that path calls stopRun() on an acquisition timeout,
	* which leaves m_stopRun set and aborts the NEXT run's axis wait. A diagnostic must leave
	* the machine exactly as it found it. Any failure aborts the scan, never the report.
	*/
	/*
	* Production Scan Check - Test Run page, online run type "Production Scan Check".
	*
	* Unlike profilerScanTest, this drives the REAL production path: JobThread::scan(), with
	* jogLaser and the laser offset, the recipe's scan direction, and the same configuration
	* calls a production run makes. It exists because the two paths configure the profiler
	* separately, and a setting wired into one and not the other looks fine until something
	* measures it - which is exactly how peak sensitivity went a day without being sent.
	*
	* Only the distance is asked for. Direction comes from the recipe's Scan Direction, and the
	* optic from getMainOptics3D(), so the test uses what production would use.
	*/
	void productionScanTest(double distance_mm);

	/*
	* Position readback that has stopped moving. SystemData::currentCoordinate() is polled, so
	* reading it straight after a jog can return a sample from mid-move - measured at 5 to 30 mm
	* out on 2026-08-28, because a stale reading at the 300 mm/s jog speed is metres per second
	* of error. Samples until two consecutive reads agree, so the caller gets a position the
	* gantry is actually at.
	*/
	dat::WorldCoordinate settledCoordinate(int timeoutMs = 3000, double tol_mm = 0.002);

	void profilerScanTest(double distance_mm, bool positiveDir, QString optic3DId,
		bool saveImages, bool returnToStart);

	void getAllIntensityFromExpectedGV(QString camID, QString opticType, int idealR, int idealG, int idealB, QRectF roi);
	void calibrateGoldenLightingProfile(QString camID, QRectF roi);
	void calibrateCurrentLightingProfile(QString camID, QRectF roi);
	void calibrateMaxCurrent(QString camID, QRectF roi, double plateauDiffThreshold, double maxCurrentAmp);


	void setPositionPortabilityPoint(PositionPortabilityType type);
	bool findPortabilityPattern();
	bool findPortabilityCircle();

	//acquisition
	void runPlaneCollection();
	void run2D();
	void run3D();
	void run2D3D();

	//simulation
	void simulateOnlineStitching();

	void test();

	void processImageReady(QVector<FrameInfo> infos);
	void processImagePreprocessed(FrameInfo info);

	void collectZImages(double x, double y, double step_mm, double firstStep, double finalStep, OpticsInfo optic);

	//UI
	void displayFOV_fnc(MIL_ID mBuf);

	//load sequence
	void loadToPositionSensor(int index);

	void unloadBoard();


signals:

	void acquisitionDone();
	void encoderReceived(dat::WorldCoordinate coordinate);
	void imageReady(QVector<FrameInfo> infos);
	void imagePreprocessed(FrameInfo info);
	void locatorReceived(QPointF locatorOffsets, double locatorAngle, QString viewID, QString indexID, bool locatorFail, bool locatorAngleFail);
	void resultReceived(QVector<FrameInfo> infos, QHash<QString, ct::UnitResultInfo> results);

	//UI
	void promptMsg(QString msg);
	void dryRunStatus(QString msg, bool running);
	void reconnectMotionDone(bool ok);
	void displayFOV(MIL_ID mBuf);
	void drawRectFOV(QString name, QRectF rect, QColor color);
	void startProgressBar(QString title, int count, bool enableCancel);
	void incrementProgress();
	void stopProgressBar();

	void cameraAlignmentDone(double angle);
	void cameraAlignmentFailed(QString msg);
	void cameraScalingDone(double horizontal_scale, double vertical_scale);

	void locatedFiducial(QRectF roi);
	void updateFiducialStatus(InspStatus::FiducialDetail detail);
	void updateFiducialRegion();
	void teachFiducialPoint();
	void fiducialDone();
	void fiducialFailed();

	void barcodeDecoded(QString code);
	void unitBarcode(QString unitID, QString code); //per pitch unit (or "board"), for the live status table
	void locatedBarcode(QRectF roi, int index, bool pass, QString code);
	void barcodeFailed();

	void updateLaserOffset();

	void planeCollectionDone();

	void appendLaserAlignmentImage(LaserAlignmentImage);
	void verifyLaserAlignmentDone();

	//Profiler Scan Test: a running commentary, then the verdict plus the report path
	void profilerScanTestProgress(QString line);
	void profilerScanTestDone(bool scanRan, QString reportPath, QString summary);
	void productionScanTestDone(bool scanRan, QString reportPath, QString summary);
	void captureAlignmentDone();
	void laserAlignmentDone();
	void guidedLaserAlignmentDone();

	void obtainedIdealIntensity(int R, int G, int B);
	void savePortabilityInfo();
	void loadPortabilityInfo();

	void startLot();
	void setLotSize(int size);
	void endLot();
	void unloadStrip();
	void uploadRecipe(QString);
	void downloadRecipe(QString);
	void frameReady();

	void stackImages(QString id);
	void openRecipe(QString recipeName);
	void createRecipe(QString recipeName);
	void onLive(QString camID);
	void offLive();

	//rail
	void signalSetRailWidthDone();

	//load sequence
	void signalLoadSequenceFail(QString msg);

	void signalBoardInPosition(int index);
	void signalBoardUnloaded();

	void liveProfileData(QVector<double> profile, double xFovMm, double zRangeMm);

	//gvTable
	void calibrationFinished(QString summaryMsg, QHash<QString, double> proposedLimits);
};
