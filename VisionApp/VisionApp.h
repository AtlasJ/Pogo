#ifndef QVISIONAPP_H
#define QVISIONAPP_H

#include "Utilities.h"
#include <QtWidgets/QMainWindow>
#include "ui_VisionApp.h"
#include <QStandardItemModel> 
#include <QMessagebox>
#include <QtWidgets>
#include <QGraphicsItem>
#include <QObject>
#include <QProgressDialog>
#include <QMetaObject>
#include <QElapsedTimer>

#include <fstream>
#include <functional>
#include <queue>
#include <set>
#include <array>
#include "QMainDelegate.h"
#include "WinSharedMem.h"
#include "QServer.h"
#include "QClient.h"
#include "ResultCache.h"
#include "AdvantechDigitalIO.h"
#include "ILSC.h"
#include "FiducialInfo.h"
#include "BarcodeInfo.h"
#include "AlgoSetupTypes.h"
#include "InspectionThread.h"
#include "AlgoDefectResult.h"
#include "ErrorInfo.h"
#include "OpenWorkspaceInfo.h"
#include "QOSTool.h"
#include "QVisionObject.h"
#include "VIsionAppQDragBox.h"
#include "DatasetQDragBox.h"
#include "VidiToolResult.h"
#include "CodeConfig.h"
#include "MessageQue.h"
#include "FrameInfo.h"
#include "ResultCache.h"
#include "WinSharedMem.h"
#include "WinEvents.h"
#include "InspectionInfo.h"
#include "ResultInfo.h"
#include "QDragBox.h"
#include "QView.h"
#include "QLineScan.h"
#include "QViewPlane.h"
#include "QImageGrabber.h"
#include "QMainGraphicsScene.h"
#include "SQLiteDatabase.h"
#include "DataBaseThread.h"
#include "NetworkPathChecker.h"
#include "FileRemovingThread.h"
#include "GoldenRecipeDialog.h"
//#include "InactivityHandler.h"

#include "IOInfo.h"
#include "PathInfo.h"

#include "TimeLogger.h"
#include "ScopedTimeLogger.h"
#include "Timer.h"

#include "CT_Client.h"
#include "Logger.h"

#include "AlgoBuffers.h"
#include "AlgoTemplate.h"
#include "mtrx.h"
#include "CommonDir.h"
#include "Fiducial.h"

#include "LaserConfig.h"
#include "ProfilerManager.h"

#include "FunctionQueue.h"
#include "OpticsInfo.h"
#include "PortabilityInfo.h"
#include "UserAccount.h"
#include "LSCManager.h"
#include "ImageManager.h"
#include "JobThread.h"
#include "OpticsControl.h"
#include "OpticsInfo.h"

#include "OnnxInference.h"

#include "MbufWrapper.h"
#include "MbufPoolManager.h"
#include "ImageSavingThread.h"

#include "SystemData.h"
#include "dbscan.h"

#include "Motion_APS.h"
#include "MachineController.h"

extern int g_viewMode;
extern MIL_INT g_imgType;
extern QString g_imgExtension;
extern int g_viewIndex;
extern bool g_forceStopInspLoop;
extern bool g_enableClassificationDataCollection;
extern QString g_bufferID;
extern ResultCache g_resultCache;
extern TMessageQue<FrameInfo> g_imageQueue;
extern Timer g_time;
extern QHash<QString, QPointF> g_locatorOffsets;
extern QVector< Onnx::InferenceEngine*> g_ODModels;
extern Onnx::InferenceEngine* g_segModel;
extern ObjectDetectionTilingSettings g_odTilingSettings;

class TemplateLibraryTab;
class DatasetPage;
class ProductionPage;
class Motion;
class Guided_2D3D_AlignmentTab;
class ImageViewerTab;
class UnitConfigTab;
class Optics3DTab;

class QDragBox;
class VisionAppQDragBox;
class QRectItem;
class QLineItem;
class QEllipseItem;
class QCrossItem;
class QMainGraphicsScene;
class QGraphicsPixmapItem;
class QItemSelectionModel;
class ExtendedMenu;

enum class EditMode {
	SELECT,  
	VISION_OBJECT,
	PATH_ASSIGNMENT,
	RULER,
	IMAGE_FILTERING,
	NAVIGATE_TO,
	POSITION_PORTABILITY_MODE
};

enum class Direction {
	UP, DOWN, LEFT, RIGHT
};

enum class UIPage {
	RECIPE, ROI_EDITOR, SCALING, PATH, LIGHTING, TEMPLATE_LIB, RECIPE_SETUP, NAMING_CONVENTION,
	CONFIG, ANALYSIS, TESTRUN, LASER, PORTABILITY, AIMODEL, COLOR_SEGMENT, ZSTACK, UNIT_CONFIG,
	OPTICS3D, MOTION, BARCODE_READER, ALGO_SETUP, DRY_RUN
};

enum class UIHierarchy {
	IMAGE, VIEW, DRAGGABLES, SHAPE
};

enum class Representation {
	ASSIGNED_VIEW, UNASSIGNED_VIEW, ASSIGNED_VO, UNASSIGNED_VO
};

enum class LaserAction {
	NONE, ALIGNMENT, INSP, VERIFY, COLLECT
};

enum class ViewMode {
	PLANE, SINGLE
};

struct SegmentResult {
	bool overTolerance = false;
	double stdDev = 0.0;
	std::array<int, 3> targetedIntensity;
	std::array<int, 3> adjustedIntensity;
	std::array<int, 3> initialExposure;
	std::array<int, 3> adjustedExposure;
	std::array<int, 3> initialGain;
	std::array<int, 3> adjustedGain;
};

struct GroupedOverSizedVO {
	QVector<VisionAppQDragBox*> VOs;
	QRectF rect;
};

const int g_min_intensity = 0;
const int g_max_intensity = 255;

//temp:
const int g_jog_distance = 100000; //um
const int g_start_velocity = 5000; //um
const int g_end_velocity = 150000; //um 1000pulse -> 1mm max: 400000
const double g_acceleration = 0.1; //0.01;
const double g_deceleration = 0.1; //0.01;

//UI Macros
#define UI_COLLAPSE "COLLAPSE"
#define UI_EXPANDED "EXPANDED"
#define UI_SETUP_BTN "SETUP"
#define UI_EXECUTE_BTN "EXECUTE"

#define SNAP_TIMEOUT 5000
#define MAX_BRIGHTNESS 252
#define MID_BRIGHTNESS 128

class VisionApp : public QMainWindow
{
	Q_OBJECT


public:
	VisionApp(QWidget *parent = Q_NULLPTR);
	~VisionApp();
	SegmentResult g_segmentResult;
	bool _hasWorkspace = false;

	QElapsedTimer vs_elapsedTimer;
	bool isStartTimer = false;

protected:
	bool eventFilter(QObject *obj, QEvent *event) override;
	void rightMenuMouseMoveEvent(QPoint pos, bool blockEventFilter);
	void moveWidgetAnimation(QWidget* widget, QRect EndValue, bool enable);
	void resizeWidgetAnimation(QWidget* widget, int minValue, int maxValue, bool enable, int rightTabIndex, QStackedWidget* stackWidget);
	void fadeWidgetAnimation(QWidget* widget, bool enable);

private:
	Ui::VisionAppClass ui;

	//test case
	void testcase_fiducialLogic(); 
	void testcase_mbufpool();
	
	bool _switchingSubRecipe = false;

	AccountInfo _curUserAccInfo;

	int _loop = 0;
	bool _testRunLoopingNoUnload = false;

	//event filter
	bool _blockEventFilter = false;
	bool _isAutoMode = false;

	//top Menu Bar
	bool _maximizedState = true;

	//system
	bool _enableClassificationDataCollection = false;
	QString _camID = "cam1";
	QString _profilerID = "profiler1";
	QString _motionID = "motion1";
	//QString _motion2ID = "motion2";
	std::thread _ioThread;
	QSet<int> _subrecipesToRun;
	QString _currentProductionID;
	bool hasSubrecipe();

	QTimer* _motionTimer = nullptr;

	QString _stateColor = "#5A7863";
	QTimer* _stateTimer = nullptr;

	//DragBoxIcon
	QIcon* _noViewIcon = nullptr;
	QPixmap _noViewPixmap;

	//progress Dialog
	QProgressDialog* _progressDialog = nullptr;
	int _progressValue = 0;
	void progressBarSetup(QString displayText, int maxValue, bool enableCancel = false); //enable cancel works for run inspection only
	void incrementProgressBar();
	void progressBarRelease(bool directCancel = false);

	QProgressDialog* _loadingDialog = nullptr;
	void loadingBarSetup(QString title);
	void loadingBarRelease();

	struct RenderInfo {
		QMainGraphicsScene* scene = nullptr;
		QGraphicsItem* item = nullptr;
	};

	//InactivityHandler* _inactivityHandler;
	// App Variable 
	bool _fastMode = false;
	double _prevCamAlignedAngle = 0.0;
	LaserConfig _laserConfig;
	int _currentLaserAlignmentIndex = 0;
	QVector<LaserAlignmentImage> _laserAlignmentImages;
	AdvantechDigitalIO _ioCard;
	int _workingMode;
	QSize _imageSize;
	WinEvents _appEvents;
	WinSharedMem _appSharedMem;
	QString _currentUser;
	//QString Common::Directory::CurrentRecipe; changed to static
	QString _productionID = "";
	QString _currentInspPoint = "view";
	QString _currentObjectID;
	QPixmap _pixmapMain;
	QImage _imageMain;
	QPixmap _pixmapFOV;
	QImage _imageFOV;
	QImage _qRedBuffer;
	QImage _qGreenBuffer;
	QImage _qBlueBuffer;
	QImage _imageWorld;
	QImage _imageWorld3D;
	QImage _dummyImage;
	QRectF _sceneBound;
	QRectF _sceneFOV;
	QJsonObject _gpIOObj;
	QJsonObject _userObj;
	QJsonObject _systemObj;
	QJsonObject _recipeMotionObj;
	QJsonObject _bufferInfoObj;
	QJsonObject _streamMappingObj;
	QJsonDocument _finalProcessDoc;
	QGraphicsPixmapItem* _pPixmapItemMain;
	QGraphicsPixmapItem* _pPixmapItemFOV;
	QMainGraphicsScene* _pGraphicsSceneMain;
	QMainGraphicsScene* _pGraphicsSceneFOV;
	QVector <QGraphicsItem*> _renderedShape;
	QHash<QString, QVector<RenderInfo>> _renderedMaps;
	QVector <QRectItem*> _defectRectShape;
	QVector <VisionAppQDragBox*> _cropGuidingRoi;
	QHash <QString, QVisionObject> _visionObject;
	QHash <QString, QString> _visionObjectCircuitId; // key: circuitId, value: visionObject Id
	QHash <QString, QView> _views;
	QHash <QString, QLineScan> _lineScans;
	double m_currentZOffset = 0.0;
	QHash <QString, nvs::motion::MotionConfig> _motions;
	QHash<QString, bool> _eMapHash; // key: row[@]col[@]island, value: ignore
	QViewPlane _plane;
	QStandardItemModel _recipeModel;
	QStandardItemModel _objectModel;
	QStandardItemModel _resultModel;
	QItemSelectionModel* _recipeSelectionModel;
	QItemSelectionModel* _resultSelectionModel;
	QStandardItem* _pRecipeItem;
	QStandardItem* _pPassItem;
	QStandardItem* _pFailItem;
	QStandardItem* _pSkipItem;
	QStandardItem* _pUninspectItem;
	QMainDelegate _objectDelegate;
	AlgoBuffers _algo;
	QPoint _scenePos;
	QPointF _startDragPos;
	QPointF _endDragPos;
	dat::WorldCoordinate _currentOriginInMM;
	QLineItem* _ruler = nullptr;
	QLineItem* _crossHairX = nullptr;
	QLineItem* _crossHairY = nullptr;
	QPointF _lastMousePressPos;
	QCursor _cursor;
	QMessageBox _msg;
	QPointF _pressedDragPos;
	QStringList _copiedVisionObjectIDs;
	QVector <DragBoxInfo> _visionObjectInfo;
	QVector <VisionAppQDragBox*> _dragROI;
	QVector <QDragBox*> _viewROI;
	QVector <QDragBox*> _lineScanROI;
	QHash <QString, QString> _displayBuffer;
	QClient _client;
	CT::CT_Client* _ctClient;
	SQLiteDatabase _sqliteDatabase;
	double _angle = 0.22;

	QDragBox _commonDragBox;

	bool _isIOCardOpened;

	CSAInfo _CSA;

	QHash<QString, OpticsInfo> _mainOptics;
	OpticsInfo3D _mainOptics3D;
	QHash<QString, OpticsInfo> _recipeOptics;
	QHash<QString, OpticsInfo3D> _recipeOptics3D;
	std::array<int, 3> _bestIntensities;
	std::array<int, 3> _channelRatio;
	QHash<QString, QSlider*> _channelSliders;
	QHash<QString, QLineEdit*> _channelLineEdits;
	QHash<QString, QCheckBox*> _channelToggle;
	QHash<QString, MIL_ID> _mIntensityMaps;
	
	double _recipeZ = 0.0;

	struct ImageFilterInfo {
		QString baseName = "";
		QString currentPath = "";
		QImage qimg;
		MIL_ID mbuf;
	};

	enum EmapType
	{
		CSV01_EMAP,
		CSV34_EMAP,
		TEXT_FILE_EMAP
	};
	enum EmapMode
	{
		AUTO,
		CSV01,
		CSV34,
		TEXT_FILE
	};
	struct EmapInfo
	{
		QString templateName;
		EmapMode mode;
		EmapType topInspEmap;
		EmapType botInspEmap;
		QStringList csvEmapDir;
		QStringList textFileEmapDir;
		QString incomingEmapPath;
	};
	EmapInfo _emapInfo;
	LotInfo _lotInfo;

	EmapInfo _emapLocalSetting;
	QHash<QString, EmapInfo >_emapTemplateList; //key: templateName
	QString _emapTemplate;


	QString _warpageMethod;

	bool _isUseEmapTemplate; // true = emap template: false = local emap setting

	GoldenRecipeDialog* _grDialog;
	InspStatus _inspStatus;

	std::set<int> _unfilteredImages;
	QVector<int> _lastFilteredImages;
	int _filterIndex = 0;
	QVector<ImageFilterInfo> _imagesToFilter;
	void resetFilterInfo();
	void filterImage(QString root);
	
	ImageManager _imageManager;
	JobThread _jobThread;

	unsigned char* _camAlpha = nullptr;
	void createCamAlpha();

	EditMode _editMode = EditMode::SELECT; 
	void setEditMode(EditMode);
	bool _pointJog = false;

	QColor getColor(Representation);

	//dragMode
	bool _dragMode = false;

	bool _inspectionThreadBusy = false;
	bool _autoCalPending = false; // set while an auto-calibration offline run is in flight; triggers the report on completion

	NetworkPathChecker _networkPathChecker;
	QTimer _networkPathCheckerTimer;
	bool _isVerificationConnected;
	QTimer _fileRemoverTimer;
	QTimer _emapReEnableTimer;
	double _emapReEnableTimerSeconds = 60;

	//PostInspectionInfo* _pPostInspectionInfo;
	ErrorInfo* _pErrorInfo = nullptr;
	OpenWorkspaceInfo* _pOpenWorkspaceInfo = nullptr;
	//BYPASS:VIDI
	// App Variable 

	//Component Cad
	bool loadComponentCadRois(const QString &filePath, QVector<CadRoiInfo> & cadRois);

	void clearEmptyViewKey();

	//readLightingUsed
	QStringList get2DLightingUsed();
	QStringList get3DLightingUsed();

	QVector<QString> _defectPriorityList;
	QHash<QString, QString> _defectMappingHash;
	QHash<QString, DefectTag> _tagNameHash;
	bool readDefectPriorityList();

	//ExtendedMenu
	ExtendedMenu* _recipeSettingsMenu;
	ExtendedMenu* _systemSettingsMenu;
	ExtendedMenu* _rightMenu;

	//ActionMenu
	QMenu actionMenu;

	//Editor
	QUndoStack* _undoStack = nullptr;

	//Tabs
	TemplateLibraryTab* _templateLibraryTab;
	Motion* _motionControl = nullptr;
	Guided_2D3D_AlignmentTab* _guided_2D3D_AlignmentTab = nullptr;

	DatasetPage* _datasetPage;
	ProductionPage* _productionPage;
	ImageViewerTab* _imageViewerTab = nullptr;
	UnitConfigTab* _unitConfigTab = nullptr;
	Optics3DTab* _optics3DTab = nullptr;

	// database thread
	DataBaseThread _databaseThread;

	//datasetPage
	QStringList _datasetIndexIds;

	QVector<IOInfo> _lightingIOs;
	QVector<IOInfo> _cameraIOs;
	double _jogDistance = 1;
	double _inFocusSnapShotZ = 1.51;
	double _inFocusGantryZ = 0.0;
	double _offsetZ = 0.0;
	dat::WorldCoordinate _toCompensate;
	QHash<QString, double> _compensateMap;

	QIcon* _viewIcon = nullptr;
	QIcon* _objectIcon = nullptr;
	QIcon* _passIcon = nullptr;
	QIcon* _failIcon = nullptr;

	PortabilityInfo _portabilityInfo;

	QMainWindow mwindowR;
	QMainWindow mwindowG;
	QMainWindow mwindowB;

	QMovie* _movieMainUi;
	QMovie* _movieBrain;
	QMovie* _movieProductionMode;

	// golden recipe
	bool _enableGoldenRecipeChecking = false;
	QString _goldenRecipeCheckListPath = "C:/Advanced/Data/GoldenRecipeCheckList.json";
	struct GoldenRecipeCheckList
	{
		QString recipeName;
		bool runStatus;
	};

	int _barcodeRegistrationMethod = 1;

	bool _od_enableTensortRt = false;
	bool _seg_enableTensortRt = false;

	void editGoldenRecipeCheckList(QString recipeName, bool runStatus, bool reset = false);
	void setupGoldenRecipeTimer();
	void triggerOnNewDay();
	bool checkGoldenRecipeRunStatus(QString recipeName); 
	void runGoldenRecipe();
	// golden recipe

	QRect getQRectBasedOnCam(int percentage);
	QRectF getQRectFBasedOnCam(int percentage);


	void initVariable();
	void initWidget();
	void initImageDisplayWidget();
	void initTCPIP();
	void initStartupState();

	void connectMachineController();
	void connectJobThread();
	void simulateImageSaving();
	void simulateOnlineInspection();
	void simulateOnlineStitching();

	QDragBox* addDragBoxToScene(QMainGraphicsScene* scene, QRect rect, const QColor& color, const QString& name, const QString& id);
	void deleteDragBox(QMainGraphicsScene* scene, QDragBox* p);
	
	QStringList m_logStatus;
	void addLogLine(const QString& line);
	void clearErrorLogs();
	void setXAxisVelocity();

	void clearLiveDefectTableWidget();
	void addDefectToTableWidget();

	void connectSignalAndSlot();
	void connectShortcuts();
	void createEventAndSharedMemory(const QJsonObject& bufferInfoObj);
	void iniCamera();
	void iniIOCard();
	void initLSC();
	void initMotion();
	void initProductionUI();
	void updateOpticComboBoxUI();
	void updateViewComboBoxUI();
	void updateSegmentComboBoxUI();
	void showSegmentReferenceUI();
	void showCSALocatorReferenceUI();
	int getNumOfSingleViewstoProcess();
	int getNumOfViewToProcess(QStringList datasetIndexIds = QStringList());
	int getNumOfLineScanToProcess(QStringList datasetIndexIds = QStringList());
	QString getPathToCSALocatorImage();
	void showSegmentPriorityUI(int index);
	void updateCSAStatus(int priority);
	void showCSADragBox(bool show);
	bool isTypeRGB(const OpticsInfo& opt);
	void assignOpticBasedOnLightingType(OpticsInfo& opt);
	void updateOpticBasedOnLightingType(const OpticsInfo& opt);
	void updateOpticBandUI(const QHash<QString, int>& band, QLineEdit* lineEdit);
	void addDefaultOptic();
	void assignMainOptics();
	void initConfig();
	void initLaserUI();
	void iniBufferQueue();
	void initQImageKeys();
	void initImageFiltering();
	void initAnalysis();
	void loadConfig();
	std::string charsToStr(char* pCharArray);
	void strToChars(char* pCharArr, int arrSize, std::string src);
	void displayImage(const QImage& img);
	void displayFOV(const QImage& img);
	void clearDrawingFromFOV();
	void drawFOVInWorld(double cx, double cy);
	void displayPlane();
	void displayWorld();
	void clearImageView();
	void clearBufferQueue();
	void drawResult(const bool& result, const QByteArray& jsonData);
	bool createStreamMapping(const QString& recipeName, const QString& workspaceName);
	void getStreamMapping(const QString& recipeName, QString& workspaceName);
	bool openWorkspace(const QString& workspaceName);
	void setVisionIO(bool state);

	void enableUIControl(const bool& flag);

	// AccessLevel::ADMIN
	// AccessLevel::ENGINEER
	// AccessLevel::OPERATOR
	bool notAllowToAccess(AccessLevel accessLevel);

	//CadReader
	//struct fidCAD {
	//	double x;
	//	double y;
	//};

	//struct padCAD {
	//	double x;
	//	double y;
	//};
	QVector<QPointF> fidList;
	QVector<QPointF> padList;
	QVector<QPointF>   m_overlayPads;
	QString           m_overlayBoxID; 
	bool exportSingleFiducialAndPad(const QString& txtPath, const QString& jsonPath);
	void overlayPadCircles(QDragBox* box,QVector<QGraphicsEllipseItem*>& circleItemsVec,const QVector<QPointF>& padData,const QRectF& originalDataBoundingRect,const QRectF& targetContentAreaInBox, qreal circleRadius,const QColor& circleColor);
	bool loadFiducialsAndPads(const QString& txtPath, QVector<QPointF>& fidList, QVector<QPointF>& padList);
	void redrawCirclesForVO(const QString& voId);

	//CT_Client
	void initCT_Client();

	//json
	bool loadJson(QString path, QJsonObject& root);
	bool loadJson(QString path, QJsonDocument& doc);
	bool saveWorldEnv();
	bool loadWorldEnv();
	QString getMainOpticsID;

	//view editor
	void initViewEditor();
	void checkSelectedView();
	void updateViewEditorSettingUI(QString viewID);
	void updateViewEditorSetting(QVector<QString> views);
	void logViews(QString msg);

	QString greyCardConfigFile() const;
	void loadGreyCardPathIfAny();
	bool saveGreyCardPath(const QString& path);
	void loadGreyCard();

	//Path
	QVector<QGraphicsItem*> _pathGraphicItems;
	QString _lastViewPressed = "";
	QString _lastSelectedViewHovered = "";
	QString _lastViewAddedToPath = "";
	QString _currentSetPoint = "start";
	//PathInfo _pathInfo;
	QStateMachine _pathSM;
	void initPathSM();
	void addViewToPath(QString viewID);
	void setViewSelectionCheckState(Qt::CheckState state);
	bool savePathInfo();
	bool loadPathInfo();

	struct IslandInfo {
		int totalRow = 0;
		int totalCol = 0;
		int totalIsland = 0;

		int rowStartingIndex = 0;
		int colStartingIndex = 0;

		double rowPitch = 0.00;
		double colPitch =0.00;
		double rotation = 0.00;

		QString prefix = "-";
		QString postfix = "-";
	};
	IslandInfo _islandInfo;

	
	bool _inspMode = false;

	

	//Scaling
	struct ManualScalingInfo {
		QPointF origin_pos;
		QPointF horizontal_pos;
		QPointF vertical_pos;
	} _manualScalingInfo;
	QStateMachine _manualScalingSM;
	void initManualScalingSM();

	enum class TeachPointType {
		FIDUCIAL, BARCODE
	};
	//teachPoint
	TeachPointType _currentTeachPointType = TeachPointType::FIDUCIAL;
	void initTeachPoint();
	void teachPoint(TeachPointType type, int index, dat::WorldCoordinate point, bool promptPassword = true);
	void jogToTeachPoint();
	void teachTopleft(dat::WorldCoordinate point);
	void teachBtmright(dat::WorldCoordinate point);
	
	bool _enableSingleViewRecipe = false;
	bool _enablePreProcessImg;
	bool _saveDefectVoImg;
	bool _saveDefectRectVoImg;
	bool _saveInspImg;
	bool _enableEmap;

	bool _autoDeleteProductionFile;
	bool _enableRmsRecipe;
	bool _enableMounterChecking;
	QStringList _clearingPathList;
	int _storageLimit;
	FileRemovingThread* _fileRemovingThread;

	bool _enable3D = true;
	bool _enable2D = true;
	bool _enableVisionObjectSampling = false;
	double _passYieldPerc;
	QString laserApi = "";
	//fiducial
	std::vector<QImage> _fid_image;
	std::vector<FiducialInfo> _fiducialInfos;
	bool _useFiducial = true;
	int _numOfFidLocated = 0;
	Fiducial _fiducial;
	Fiducial _fiducial2;   // second island transform (fid3/fid4) for double fiducial checking
	int _currentFidIndex = 0;
	QDragBox _fidInspectionRegion;
	QDragBox _fidSearchRegion;
	QDragBox _fidLocatedRegion;
	double fidWidth;
	double fidHeight;
	bool fiducialExistTest(int index);
	void toggleFiducialUI(bool enable);
	void initFiducial();
	void setDefaultFiducialInfos();
	void showFiducial(int index);
	void showFiducialFeature(int index);
	void showFiducialMethodUI(int index);
	void learnFiducial();
	void clearFiducialFeature(int index);
	bool saveFiducial();
	bool loadFiducial();
	void displayFiducialImage(int index);
	void toggleFidROISetupMode(bool state);
	void updateFiducialSettingsJson(int index);
	void fiducialSize();
	void backupFiducial();

	//barcode
	std::array<QImage, 2> _barcode_image;
	bool _enableBarcode = false;
	int _currentBarcodeIndex = 0;
	std::array<BarcodeInfo, 2> _barcodeInfos;
	QDragBox _barcodeSearchRegion;

	//algo setup page ROIs (shown only on the algo page for the selected algo)
	QDragBox* _algoOcrRoi1Box = nullptr;
	QDragBox* _algoOcrRoi2Box = nullptr;
	QDragBox* _algoOcrLearnBox = nullptr;
	QDragBox* _algoLocLearnBox = nullptr;

	//camera alignment/scaling feature ROIs (laser/alignment page) - VisionApp_Laser.cpp
	QDragBox* _alignCircleRoi = nullptr;
	QDragBox* _alignLearnBox = nullptr;
	QDragBox* _alignSearchBox = nullptr;
	QDragBox* _algoLocSearchBox = nullptr;
	QVector<QDragBox*> _algoPlaneBoxes;
	QVector<QDragBox*> _algoHeightBoxes;
	QVector<QGraphicsItem*> _algoOverlayItems;
	bool _algoHeightView3D = false;
	QDragBox _barcodeLocatedRegion;
	bool barcodeExistTest(int index);
	void initBarcode();

	//external barcode reader setup page (SRXManager)
	void initBarcodeReaderPage();
	void initBarcodeReaderAlignment();
	void refreshBarcodeReaderPage();
	void updateSRXImagePreview(const QString& readerID);

	//camera alignment method (circle/pattern) - VisionApp_Laser.cpp
	void initAlignmentMethodUI();
	void updateAlignMethodWidgets();
	void updateAlignRoiVisibility();
	void hideAlignRois();
	void saveAlignmentConfig();
	void loadAlignmentConfig();
	AlignFeatureParams buildAlignFeatureParams(bool& ok);

	//setup region pitch mode UI refreshers (set up in the connect block, reused on recipe load)
	std::function<void()> _applySetupRegionMode;
	std::function<void()> _refreshPitchLabels;

	//dry run page - VisionApp_DryRun.cpp
	void initDryRunPage();
	void saveDryRunPoints();
	void loadDryRunPoints();

	//algo setup page (AlgoManager) - VisionApp_AlgoSetup.cpp
	void initAlgoSetupPage();
	void refreshAlgoSetupPage();
	void refreshAlgoLocatorUI();
	void refreshAlgoPatternList();
	void updateAlgoRoiVisibility();
	void updateAlgoHRoiCounts();
	void hideAlgoSetupRois();
	void captureAlgoParamsFromUI();
	void showAlgoHeightMap(bool view3D);
	void clearAlgoOverlay();
	void renderAlgoOverlay(const QVector<AlgoOverlayItem>& overlay);
	AlgoPageAlgo currentAlgoPageAlgo() const;
	void showBarcode(int index);
	void showBarcodeDebugImage(int index);
	bool saveBarcode();
	bool loadBarcode();
	void saveBarcodeResult();
	void updateBarcodeSettingsUI(int index);
	void updateBarcodeSettingsJson(int index);
	void displayBarcodeImage(int index);
	void toggleBarcodeROISetupMode(bool state);

	//recipe setup ZStack
	void initRecipeSetupZStack();
	void saveRecipeSetupZStack();
	void loadRecipeSetupZStack();

	//PositionPortabiliy
	void initPortability();
	void togglePositionPortabilitySetupROIMode(bool state);
	void displayPortabilityFeatureImage();
	void savePortabilityBaseImage();
	void learnPortabilityPatternFeature();
	void showPortabilityPatternFeature();
	bool testPortabilityPatternFeature();
	bool testPortabilityCircleFeature();
	bool savePositionPortabilityInfo(PositionPortabilityType type);
	bool loadRefPositionPortabilityInfo(QString refPortabilityPath = QString());
	bool loadCurPositionPortabilityInfo(QString curPortabilityPath = QString());
	void getCurrentMachinePortabilityPointOffset(dat::WorldCoordinate & offset);
	dat::WorldCoordinate getAbsoluteRobotPoint(dat::WorldCoordinate point);
	QPointF getAbsoluteFOVCoordinates(const QPointF & FOVcoordinates);
	QPointF getAbsoluteWorldCoordinates(const QPointF & Worldcoordinates);
	dat::WorldCoordinate getRelativeRobotPoint(dat::WorldCoordinate point);
	QPointF getRelativeFOVCoordinates(const QPointF & FOVcoordinates);
	QPointF getRelativeWorldCoordinates(const QPointF & Worldcoordinates);
	void setPositionPortabilityPoint(PositionPortabilityType type);
	void jogToPositionPortability(PositionPortabilityType type);
	em::V2d getPositionPortabilityPointInMM(int x_px, int y_px);
	bool getPortabilitySizeDifference(double difference, double & offsetZ);
	void guidedAlignPositionPortabilitySetup();

	//AIModels
	std::vector<QLineEdit*> _objectDetectionModelPaths;
	std::vector<QCheckBox*> _objectDetectionCheckBoxes;
	QHash<QString, int> _objectDetectionClassSize; // key modelID
	QString _curSegmentationModel;
	void clearGridLayout(QGridLayout* layout);

	//Naming convention
	struct NamingConvention {
		QString prefix, postfix;
		QString row, col;
	} _namingConvention;
	void initNamingConvention();
	QString getIndexNaming(int currentIndex);
	QString getRowColumnNaming(QString island,int row, int col);
	void updateNamingPreview();
	void allowOnlyIslandNamingConvention();

	//dummy func
	bool _debug = false;
	void displayDummyImage(int w, int h);
	void recordMemory(QString msg);
	void resetLoopFlags();

	void startAcquisition();
	void startProduction();
	void startProductionS();

	//helper
	TimeLogger _timer;
	bool _dryRun = false;
	bool _runGrr = false;
	TimeLogger _timelogger;
	TimeLogger _enqueueTimer;
	QMovie* _movieShowLineScan;
	void initMovieIcons();
	void log(std::string msg, const dat::WorldCoordinate& w);
	QPointF getCenterPointFrom4Side(QRectF rect);
	MIL_ID getCameraMilMono();
	std::string generateTimeStampID();

	std::atomic<bool> _stopRun = false;

	QHash<QString, OpticsInfo> _brightnessOverrides; //brightness override takes precedent over recipe brightness
	QHash<QString, OpticsInfo> _recipeBandBrightness;
	QHash<QString, RGBOffset> _rgbOverrides;

	struct UtilityInfo {
		int num = 0;
		em::V2d center = { 0.0, 0.0 };
		double distance = 0.0;
		double angle = 0.0;
	} _utilityInfo;
	
	void processUtilityInfo();
	void updateUtilityInfoUI();

	QDragBox _worldFOV;
	QImageGrabber* _grabber = nullptr;
	void startThread(QObject* worker);
	void getEncoder(const QString& data, dat::WorldCoordinate& encoder);
	void removeWhitespace(QString& str);

	// wc
	InspectionInfo* _pInspectionInfo;
	//systemTrayIcon
	QSystemTrayIcon *trayIcon;
	QMenu *trayIconMenu;
	QAction *_minimizeAction;
	QAction *_maximizeAction;
	QAction *_quitAction;
	void createTrayIcon();

	void updateVoStatus(QVector<ct::DefectResult>& defectResults, QVector<BarcodeDecoderInfo>& barcodeInfoVector);
	void updateRowColIslandID(QVector<ct::DefectResult>& defectResults);


	QString _dataBasePath = "C:/Advanced/Data/database_3df.db";


	//Inspection
	std::queue<QString> _inspQueue;
	enum class ProcessType {
		NONE,
		IMAGE_COLLECTION,
		PRODUCTION,
		LIVE_INSPECT
	}_processType = ProcessType::NONE;

	void arrangeTagNameBasedOnPriority(QStringList& tagNames);


	//rtr
	std::array<QString, 3> _rtr_unitID;
	std::array<bool, 3>_rtr_teachMode;
	bool _first = true;
	bool _toggle = true;
	void rtrComms(QString data);

	//unitConfigTab
	QHash<QString, QColor> _savedBorderColors;
	bool roiLocked;

	//bareBoardAnalysis
	bool _inbbaInspection=false;

	bool patchTemplateKeys(const QString& recipeDir, const QStringList& keyValueRules);
	bool copyAndPatchRecipe(const QString& sourceDir, const QString& destinationDir);

	QGraphicsPathItem* _no3DZoneItem = nullptr;
	bool _show2DOnlyOverlay = true;
	void update2DOnlyOverlay();
	void clear2DOnlyOverlay();



public slots:

	void enableFiducial(bool enable);
	void enableSaveInspectionImage(bool enable);

//Barcode
	void updateBarcodeSearchRegion();
	void showBarcodeRegistrationMethodUI(int methodIdx);

//Fiducial
	void updateFiducialRegions();
	void fiducialAlignment();

//portability
	void updatePositionPortabilityInfos();
	
//DatasetPage
	void refreshDatasetView();
	void displayCurrentView(QString viewID, QString opticID, QString indexID);
	void runStoredUnitsInspection(QStringList storedIndexIDs);

//Helper
	void executeUIFunc(std::function<void()> func);
	void imageReceived(FrameInfo infos);
	void imagePreprocessed(FrameInfo infos);
	void imageReady(QVector<FrameInfo> infos);

//UI navigation
	bool isPage(UIPage);
	bool toPage(UIPage);
	void prepareRecipeSetupPage();

//left Menu Fnc
	void toggleMenu();
	void showSetupPage();
	void showDatasetPage();
	void showProductionPage();
	void showRecipeSettingsMenu();
	void showSystemSettingsMenu();
	void toggleRecipeSettingsMenu(bool enable);
	void toggleSystemSettingsMenu(bool enable);
	void toggleRightMenu();

//show left Tab

	void toPageLeft();
	bool showLeftTab(int index, QString status);

//extendedMenuRecipeSettings
	void recipeSettingsMenuBtnPressed(int btn);
	void systemSettingsMenuBtnPressed(int btn);

//right Menu Fnc
	bool showRightTab(int index, QString status);
	void showLogTab();
	void showLogTab(int index, bool isShow);
	void hideRightMenu(bool blockEventFilter);
	void blockRightMenu(bool block);
	void resizeLogTab();

//top Menu Bar
	void maximize_restoreWindow();
	void maximizedWindow();
	void closeWindow();
	void fadeIn();

//action Menu
	void visionObjectMode();
	void selectMode();
	void selectAssigned();
	void guidedPositionPortabilityMode();

//graphicViewMain
	void setdragMode(bool flag);
	void setRightMousePressed(QPoint point);
	void toggleDualView();
	void toggleWorldView();
	void toggleFOVView();
	void showRightTabFOV();
	void showProductionFOV(const int& viewSize);
	void wheelEventStart();
	void wheelEventEnd();

	void toggleOfflineRun();
	void toggleOnlineRun();

//templateLibraryTab
	void setVisionObjectAsDefaultTemplate();
	void deleteVisionObjectTemplate(const QString & templateId);
	void updateVisionObjectTemplate(AlgoTemplate* algoTemplate);
	void updateVisionObjectSize(AlgoTemplate* algoTemplate);
	void updateVisionObjectColor(AlgoTemplate* algoTemplate);
	void generateVIDIImages(AlgoTemplate* algoTemplate, bool enablePreprocess);
	void addVisionObjectPadding(AlgoTemplate* algoTemplate, int paddingSize);
	void saveTemplateReferenceImage(AlgoTemplate* algoTemplate);
	bool referenceImageExistTest();

//unitConfigTab
	void runStoredUnits();
	void storeSkippedUnits();
	void removeSkippedUnits();
	void saveBorderColors();
	void applySkipColors();
	void restoreBorderColors();

//ImageViewerTab
	void displayCurrentView(QString viewID, QString opticID);
	void displayCurrentView();

//singleViewRecipeFnc
	void showPreviousImage();
	void showNextImage();
	
//treeViewExplorerFunction
	void updateTreeViewExplorer(QString&  recipeName, QHash<QString, QView> views, QHash<QString, QVisionObject> visionObjects, QVector<ct::DefectResult> defectResults = QVector<ct::DefectResult>());

//CT_Client
	void connectToServer();
	void sendReplyReceived(QString message);
	void showExe();
	void hideExe();
	void quitExe();
	bool EXE_ExistTest(LPCWSTR exeName);

//SQLiteDataBase
	void insertProductionToDataBase(QVector<ct::DefectResult>& defectResults);


//CustomResult
	void outputCustomResultJson(QVector<ct::DefectResult>& defectResults);
	QHash<QString, QString> getDefectCode(QString &filePath);
	void outputVoInfo();
	void outputViewInfo();
	void outputLineScanInfo();
	void outputUnitIndexInfo(QVector<ct::DefectResult>& defectResults, QVector<BarcodeDecoderInfo>& barcodeInfoVector);
	void outputOpticInfo();
	void outputBoardInfo();
	void outputRecipeSettings();
	bool readOutputVoInfo(QString voInfoListPath = QString());
	
	

//inspectionThread
	void inspectionDone(QVector<ct::DefectResult>& defectResults, QVector<BarcodeDecoderInfo>& barcodeInfoVector);
	void updateInspectionProgressBar();
	void runLooping();
	void locatorInfo(QPointF locatorOffsets,double locatorAngle, QString viewID, QString indexID, bool locatorFail, bool locatorAngleFail);
	void displayLiveImage(QVector<FrameInfo> frameInfos, QHash<QString, ct::UnitResultInfo> unitResultInfo);

//DefectRectShape
	void drawDefectResults(QVector<ct::DefectResult> & defectResults);
	void clearAllDefectRectShape();
	void clearCropGuidingRoi();

//VisionObjectFnc
	void storeVisionObjectInfo(bool forceCheck = false);
	void updateVisionObjectInfo(bool forceCheck = false);
	bool includeVisionObject_into_View(VisionAppQDragBox* dragBox, bool forceCheck = false);
	bool includeVisionObject_into_HeightMap(VisionAppQDragBox* dragBox, bool forceCheck = false);
	void copyVisionObject();
	void pasteVisionObject();

// App Function
	void updateWindowMask(int borderRadius);
	bool readUserInfo(QJsonObject& userObj);
	QVector<AccountInfo> loadUserAccounts();
	bool saveUserAccounts(const QVector<AccountInfo>& accounts);
	void openUserManagementDialog();
	bool readSystemInfo(QJsonObject& systemObj);
	bool updateSystemInfo(const QJsonObject& systemObj);
	bool readBufferInfo(QJsonObject& bufferInfoObj);
	bool createBufferInfo(QJsonObject& bufferInfoObj, QSize & imgSize);
	void assignDisplayBuffer(const QJsonObject& bufferInfoObj);
	void formImage(QImage& img, int w, int h, const unsigned char* pRedBuf, const unsigned char* pGreenBuf, const unsigned char* pBlueBuf, const unsigned char* pAlphaBuf = nullptr);
	void loadImage2Mem(const QJsonObject& bufferInfoObj);
	void showBufOnMainDisp(const QRgb& oColor = qRgb(0, 0, 255));
	void initDisplaySize();
	void updateMainDisp();
	void promptLoadImageType();
	void loadWorldView();
	void loadPlaneView(bool autoLoad);
	void loadHeatMap();
	void loadFOV();
	void loadImage(const QString& fileName);
	void loadImageSet(QString imageSetPath);
	void loadProcessedImage(QString imageSetPath, QString savePath);
	void saveImage();
	bool saveJson(const QString& fileName, const QJsonDocument& doc);
	void showMsg(const QString& msg, QMessageBox::StandardButtons buttons = QMessageBox::Close);
	void showStatus(const QString& msg, int timeout=3000);
	void logMsg(QString msg, bool reset = false, QColor color = QColor(0, 255, 127), qreal size = 9);
	void showInfo(const QString& info=QString());
	void clearLogMsg();
	void clearInfoMsg();
	void openSystemSetting();
	void showRecipeInExplorer();
	void goSleep(int msSleep);
	void setMessageBoxTitleColor(QMessageBox& messageBox, QColor color);
	QString dragROINameGenerator(QString name);
	bool dragROIExistTest(QString name);
	QString getNextSamplePath();
	QString getViewCollectionPath();
	void lockAllROIs();
	void unlockAllROIs();
	void reloadStyleSheet();


	void clear2DImages(QString folder);
	void clear3DImages(QString folder);

	void drawCropGuidingRoi();
	

	QRectItem* drawDefectRect(const QRectF& rect, const QString & defectID,const QString & defectName,const QString & viewId = QString(), const QString & indexId = QString(), const QString & opticId = QString(), const QColor& borderColor = QColor(255, 0, 0), const QColor& innerColor = Qt::transparent);
	QRectItem* drawRect(const QRectF& rect, const QColor& borderColor = QColor(0, 255, 127), const QColor& innerColor = Qt::transparent);
	QLineItem* drawLine(const QLineF& line, const QColor& color = QColor(0, 255, 127), int width = 3);
	QEllipseItem* drawEllipse(const qreal& x, const qreal& y, const qreal& radiusX, const qreal& radiusY, const QColor& borderColor = QColor(0, 255, 127), const QColor& innerColor = Qt::transparent);
	QGraphicsTextItem * drawText(const QString& text, const QPointF& pos, const QColor& color = QColor(0, 255, 127), const int& pointSize = 12);
	QCrossItem* drawCross(const QRectF& rect, const QColor& color = QColor(0, 255, 127));
	VisionAppQDragBox* drawVisionAppDragBox(const QRectF& rect, const QColor& color = QColor(0, 255, 127), const QString& name = QString("DRAG_BOX"), const QString & viewID = QString());
	QDragBox* drawViewBox(const QRectF& rect, const QColor& color = QColor(0, 255, 127), const QString& name = QString("VIEW"), QString id = QString());
	QDragBox* drawLineScan(const QRectF& rect, const QColor& color = QColor(0, 255, 127), const QString& name = QString("LineScan"), QString id = QString());

	QRectItem* drawRect(QMainGraphicsScene* scene, QString mapKey, const QRectF& rect, const QColor& borderColor = QColor(0, 255, 127), const QColor& innerColor = Qt::transparent);
	QLineItem* drawLine(QMainGraphicsScene* scene, QString mapKey, const QLineF& line, const QColor& color = QColor(0, 255, 127), int width = 3);
	QEllipseItem* drawEllipse(QMainGraphicsScene* scene, QString mapKey, const qreal& x, const qreal& y, const qreal& radiusX, const qreal& radiusY, const QColor& borderColor = QColor(0, 255, 127), const QColor& innerColor = Qt::transparent);
	QGraphicsTextItem * drawText(QMainGraphicsScene* scene, QString mapKey, const QString& text, const QPointF& pos, const QColor& color = QColor(0, 255, 127), const int& pointSize = 12);
	QCrossItem* drawCross(QMainGraphicsScene* scene, QString mapKey, const QRectF& rect, const QColor& color = QColor(0, 255, 127));
	void clearRenderMap(QString mapKey);
	void clearAllRenderMaps();
	
	void dragBoxCreatedEvent(QRectF rect);
	void dragBoxMouseReleasedEvent(QDragBox* pDragBox, QString name, QPointF pos);
	void dragBoxMousePressedEvent(QDragBox* pDragBox, QString name, QPointF pos);
	void dragBoxMouseHoverEntered(QDragBox* pDragBox);
	void dragBoxMouseHoverLeaved(QDragBox* pDragBox, QString name);
	void dragBoxContextMenuEvent(QDragBox* pDragBox, QString name, QPointF pos);
	void dragBoxResized(QDragBox* pDragBox, QString name);
	void grabberReleased(QDragBox* pDragBox);
	void grabberPressed(QDragBox* pDragBox);
	void refreshDragBoxSequence();

	void viewBoxMousePressedEvent(QDragBox* pDragBox, QString name, QPointF pos);
	void viewBoxMouseHoverEntered(QDragBox* pDragBox);

	void mouseMove(QPoint pt);
	
	void doDrawRect(const QRectF& rect, const QColor& borderColor, const QString& toolTip);
	void doDrawLine(const QLineF& line, const QColor& borderColor, const QString& toolTip);
	void doDrawCross(const QRectF& rect, const QColor& borderColor, const QString& toolTip);
	void doDrawText(const QString& text, const QPointF& pos, const QColor& color, const int& pointSize);
	void doDrawCircle(const QPointF& pt, const qreal& radius, const QColor& color, const QString& toolTip);

	void clearDrawing(QGraphicsItem *pShape);
	void excludeFromRenderedShape(QGraphicsItem *pShape);
	void clearDragBox(QDragBox* pDragBox);
	void clearAllDrawings();
	void clearVisionObject();
	void clearView();
	void clearLineScans();
	void processEvents();
	void updateVisionObjectGeometry();
	void showActiveObject(QModelIndex &index);
	void graphicsViewMousePress(QPointF pt, bool isLeftClick);
	void graphicsViewMouseReleased(QPointF pt, bool isLeftClick);
	void onCustomContextMenu(const QPoint &point);
	void processROIOption(QRect rubberBandRect, QPointF fromScenePoint, QPointF toScenePoint);
	void recipeSelectionChangedSlot(const QItemSelection & /*newSelection*/, const QItemSelection & /*oldSelection*/);
	void resultSelectionChangedSlot(const QItemSelection & /*newSelection*/, const QItemSelection & /*oldSelection*/);
	void treeViewRecipeExplorerClicked(QModelIndex);
	void treeViewObjectExplorerClicked(QModelIndex);
	void treeViewRecipeExplorerEntered(QModelIndex);
	void objectModelItemChanged(QStandardItem *item);
	void modelResetSlot();
	void rowsRemovedSlot(const QModelIndex &parent, int first, int last);
	void newRecipe();
	void createRecipe(const QString& recipeName);
	void createSubRecipe();
	void addObject();
	void addObjectFromView();
	QString addVisionObject(QRectF rect, bool setSelected = true); //return vision object's key
	void saveVisionObject();
	void saveView();
	void saveLineScans();
	void saveRecipe();
	bool saveRecipeConfig();
	bool saveLaserConfig();
	bool saveIslandInfo();
	//bool saveSetupConfig();
	bool loadVisionObject();
	bool loadView();
	bool loadIslandInfo();
	bool loadLineScans();
	bool loadRecipeConfig();
	bool loadLSCConfig();
	bool loadLaserConfig();
	void updateLaserOffsetUI(dat::WorldCoordinate offset);
	void duplicateRecipe();
	bool openRecipe(const QString& recipeName = QString(""), bool autoLoad=false);
	void openAutoCalibrationRecipe();
	void generateAutoCalReport();
	void switchSubRecipe(const QString& subrecipe);
	void switchToMainRecipe();
	void switchToSubRecipe();
	void archiveRecipe();
	void restoreRecipe();
	void recipeChanged();
	void clearDir(const QString& path);
	void editTemplate();
	void findVisionObject();
	void resetResult();
	bool createResultFolder(QString& path);
	void toggleImageView();
	void toggleDrawingAndRois();
	void closeApp();
	void showImageView();
	void showChartView();
	int numBoxSelected();
	QVector <VisionAppQDragBox*> getSelectedVisionObject();
	QVector <QString> getSelectedViewIDs();
	void selectAll();
	void workingMode();
	void checkRecipeFacing(QString recipeName, bool &isTop);
	void setUserEnvironment(AccessLevel);
	void setUIVisibility();
	void setUIVisibilityMotionControl(AccessLevel accessLevel);
	void loginMode();
	void initGifIcon();

	void triggerCamera(); //display cam without lighting control
	void toggleLiveView();
	void startLiveView();
	void stopLiveView();
	void toggleCircleCrosshair();
	bool verifyLogin();
	void initMachine();
	void escapeKeyPressed();

	//prompts
	bool passwordPromptCorrect();
	QString promptComboBox(QStringList items, QString title, QString msg);
	QStringList promptFolderSelection(const QString& defaultPath = QString(), QWidget* parent = nullptr);
	bool promptQuestion(QString title, QString msg);

	void showAllGraphicItems(bool show);
	void showCrossHair(bool show);
	void showView(bool show);
	void showLineScans(bool show);
	void showVisionObject(bool show);
	void showDefectRect(bool show);
	void showPath(bool show);
	void executePathAssignment();
	void redrawPath();
	void selectAllPath();
	void unselectAllPath();
	void clearPath();
	void toggleCommonDragBox(QToolButton* btn);

	int numOfCheckedListItems(QListWidget* lists);

	//singleViewRecipeFunction
	void assignSingleViewForWholePlane();
	void generateSingleViewImages();

	//Editor
	void highlightViewInWorld(QString id);
	void assignViewsToOversizedVO(VisionAppQDragBox* vo, int padding_wpx, int overlap_percentage);
	void assignViews(double padding_mm, int overlap_percentage);
	void addView();
	void generateBestPath();
	void verifyUnassignedVisionObject();
	void duplicateROI(Direction direction);
	void duplicateFromSelectedVisionObject();
	void duplicateFromSelectedVisionObject_cropped();
	bool updateAllChannels();
	bool validChannel(QString ch);
	void printBand(QString title, const ct::Band& band);
	void sampleGVFromRegion();
	void getAllIntensityFromExpectedGV();
	QListWidgetItem* getViewOpticListItem(QString id);
	void jogToGrayCard();
	int getMaxIntensityOfBand(const OpticsInfo& opticID, BandType bandType);
	int getAverageBasedOnSegment(const MIL_ID& mMono, const std::vector<cv::Point>& segment);
	int getAverageBasedOnInlierSegment(const MIL_ID& mMono, const std::vector<cv::Point>& segment);
	void generateGVTableReport();
	void backupPortabilityFile();

	//runInspection
	void promptInspSelection();
	void runQueuedInsp();
	void run();
	void testRun(); 
	void stopRun(bool clearInspQueue = true);
	void userClickStopRun();
	void resetStopRunFlags();
	void runOffline();
	void load_2dOffline_image();
	void load_2dOffline_image_parallel();
	void load_3dOffline_image();
	void load_3dOffline_image_parallel(QHash <QString, QLineScan> linescan, QHash<QString, OpticsInfo3D> optic3D);
	void copyAllFilesToProduction(QString sourcePath, QString destPath);
	bool copyFileToFolder(const QString& sourceFilePath, const QString& destFolderPath, bool overwrite = true);
	void loadImagesForOfflineInspection(QHash <QString, QLineScan> linescan, QHash<QString, OpticsInfo3D> optic3D);
	void GenerateVidiWorkspaceInfo();
	void openVidiWorkSpace();
	void closeVidiWorkSpace();

	//JSON
	bool saveRecipeOptics();
	bool loadRecipeOptics();
	bool saveOpticBand(QJsonObject& obj, const QHash<QString, int>& opt);
	bool loadOpticBand(const QJsonValue& doc, QHash<QString, int>& opt);
	bool savePortabilityInfo();
	bool loadPortabilityInfo();
	bool saveMotion();
	bool loadMotion();
	bool saveRecipeMotion();
	bool loadRecipeMotion();
	void toJson(const QView& view, QJsonObject& obj, bool isRelative = false);
	void fromJson(const QJsonObject& obj, QView& view, bool isAbsolute = false);
	void toJson(const dat::WorldCoordinate& obj, QJsonObject& j, bool isRelative = false);
	void fromJson(const QJsonObject& j, dat::WorldCoordinate& obj, bool isAbsolute = false);
	void toJson(const ct::Box2D& obj, QJsonObject& j);
	void fromJson(const QJsonObject& j, ct::Box2D& obj);
	void toJson(const QDragBox* p, QJsonObject& obj);
	void fromJson(QDragBox* p, const QJsonObject& obj);
	void toJson(const nvs::motion::AxisConfig& a, QJsonObject& j);
	void fromJson(const QJsonObject& j, nvs::motion::AxisConfig& a);
	void toJson(const nvs::motion::MotionConfig& m, QJsonObject& j);
	void fromJson(const QJsonObject& j, nvs::motion::MotionConfig& m);

	void recipeSanitaryCheck();

	//Plane
	bool isPlaneValid(const QViewPlane& plane);
	void generatePlane(QViewPlane& plane);
	void collectPlaneViews(const QViewPlane& plane);
	void stitchPlaneImage(const QViewPlane& plane);
	bool savePlane();
	bool loadPlane();

	//linescan
	void reassignVoIntoLineScans();
	QVector<GroupedOverSizedVO> groupOverSizedVOForLineScan(QVector <VisionAppQDragBox*> bigROIs);
	void assignLineScansToOversizedVO(VisionAppQDragBox* vo, int padding_wpx, int overlap_percentage);
	void assignLineScansToGroupedOversizedVO(const GroupedOverSizedVO& gvo, int padding_wpx, int overlap_percentage);
	void assignLineScans();
	int getScanOrientation(const dat::WorldCoordinate& start, const dat::WorldCoordinate& end);
	QImage get3DImage(QImage& qimg);
	MIL_ID get3DImage(MIL_ID mbuf);
	void warpPerspective(MIL_ID & milImg);
	void alignCameraAndLaser();
	void enhancedAlignCameraAndLaser();
	void guidedAlignCameraAndLaser();
	void guidedAlignCameraAndLaserSetup();
	void verifyLaserAlignment();
	void drawCrossOnQImage(int cx, int cy, int size, QImage& qimg);
	void saveQImageWithCrossSection(QImage qimg, QString path);
	void displayCurrentAlignmentImage();
	void scanDone();
	static void terminated(int signum);

	void collectImages();
	void collect2DView(QString rootPath);
	void collect3DView(QString rootPath);
	void collect2D3DView(QString rootPath);
	void inspect2D3D();

	//Path
	void generatePath();

	//Scaling
	bool autoScaling(double step_mm, double& h_scale, double& v_scale);
	void setCameraAngle(double angle);
	void cameraAlignment();
	void performScaling();

	//AIModel
	void addObjectDetectionModels();
	void load_unload_ODModels();
	void loadODModels();
	void unloadODModels();
	void loadODModelListJson();
	void saveODModelListJson();
	void oDModelCheckBoxChanged();
	void loadSegmentationModel();
	void unloadSegmentationModel();

	//Motion control
	double mm_to_pulse(double mm);
	double um_to_mm(double um);
	QString getJogCommand(double x, double y, double z);
	QString getJogCommand(dat::WorldCoordinate world);
	QString getFidCompensatedJogCommand(double x, double y, double z);
	QString getFidCompensatedJogCommand(dat::WorldCoordinate world);
	QString getLaserFidCompensatedJogCommand(dat::WorldCoordinate world);
	bool jogToView(const QView& v);
	bool jogToFidCompensatedXYZ(double x, double y, double z, QString type = "2D");
	bool jogToLaserFidCompensatedXYZ(double x, double y, double z, QString type = "2D");
	bool jogToCamView(double x, double y, double z, QString type = "2D");
	bool jogToLaserView(double x, double y, double z, QString type = "2D");
	bool getCurrentPoint(double &x, double &y, double& z);

	//EZ mode
	void generateRecipeSetupTemplate();

	// Testing Code
	void createPieChart();

	void testFunction();


	//vidi Image 
	void saveOffsettedVIDIImage(const QString & visionObjectID, const QString & lightingID, const QString &imagePath,const QPointF & offset);


	//createNewDirectory();
	QString createTemplateImagesDirectory();
	void setupProductionDir();
	void saveProductionBarcodeInfo();
	void saveProductionInfoJson();
	bool loadProductionInfoJson();
	void saveFiducialResultJson(QString rootPath);

	void updateMsgBoxBorder();


	// read E-map
	void readEmap();
	// upload E-map
	void updateEmap();
	// read IVEmap
	void readIVEmap(QString productionFolderPath = QString());
	// save IVEmap
	void saveComparisonEmap();
	// read CSV Emap
	bool readEmap_Csv01();
	bool readEmap_Csv34();
	// read txt File Emap
	bool readEmap_textFile();
	//sampling Vision Object
	void visionObjectSampling();
	//test EmapID
	bool testEmapID();

	bool copyFolderRecursively(const QString &srcFolderPath, const QString &destFolderPath);
	void clearDirectory(const QString& path);

	// check verificaiton ip address
	void checkVerificationIp();
	void updateConnection(bool);


	
	void searchVo();
	void updateSetupCheckList();
	void writeSetupCheckListTextEdit(QString header, bool info, QString extraInfo = "");

	void refreshClearingPathListWidget();
	void iniFileRemover();
	void updateDriveSpace();


	// golden ligting template
	void importOptic();
	void loadLightingTemplate();
	void saveAsLightingTemplate();
	void deleteLightingTemplate();
	void updateLightingTemplate();
	void refreshLightingTemplateComboBox();
	void saveLightingTemplateJson(QString filePath);

	bool pullFromRmsRecipe(QString recipeName);
	bool pushToRmsRecipe(QString recipeName);
	void rmsRecipeUpdate(const QString& sourceFolder, const QString& destFolder);
	void udpateRecipeVersion(const QString& folderPath);

	//QXlsx
	bool csv_readCircuitIdMapping(QString &csvPath);
	bool csv_readMounterIdMapping(QString &csvPath);

	void loadEmapSetting();
	void refreshEmapSettingUi();

	void slotDatabaseStatus(bool status);

	// new golden recipe
	void openGoldenRecipeDialog();
	void goldenRecipeRunComplete();
	void productionRunGoldenRecipeComplete();
	void slotRunGoldenRecipeComplete(bool, QString);
	//

	void guidingRoiResize(QDragBox*, QString, QPointF);

	void updateCameraTypeUI(const QString& camID);
	void updatePortabilityFeatureUI(int index);
	void clearCacheFolder();
	bool deleteAllFilesInFolder(const QString& folderPath);

	//Bareboard Analysis
	void runBareBoardAnalysis();

	//Stiching Method
	void saveStitchingMethod();
	/*bool loadStitchingMethod();*/
	void initStitchingMethod();

	bool cropVOFromViewImage(const QString& viewImagePath, const QRectF& voRectWorldPx, const QPointF& fovOffsetPx, MIL_ID& outParent, MIL_ID& outChild, QRect& outCropRect, QString& outRectString) const;

	void boardInPosition(int pos);
	void runProdS();
	void unloadBoard();

	void clearSubRecipe();

	void vs_updateUptimer();
	void vs_startElapseTimer();
	void vs_stopElapseTimer();
	bool blockJogSignal();

	void handleCalibrationFinished(QString msg, QHash<QString, double> limits);

signals:
	void updateMsgBoxBorderSignal();
	void viewBoxPressed(QDragBox* pDragBox, QString name, QPointF pos);
	void signalEncoderChanged(double x, double y, double z);
	void snapImage(const OpticsInfo& optic, QString viewID, QString stitchID, bool resetFrame = true);
	void snapImageFastMode(const OpticsInfo& optic, QString viewID, QString stitchID, bool resetFrame = true);
	void testJob();

	void jogTo(double x, double y, double z, QString type = "2D", bool waitJogDone = true);
	void signalDryRun(QVector<QVector3D> coords, int loops);
	void signalReconnectMotion();
	bool jogSnap(double x, double y, double z, const OpticsInfo& optic);
	void jogLeft(double mm, const OpticsInfo& optic);
	void jogRight(double mm, const OpticsInfo& optic);
	void jogFront(double mm, const OpticsInfo& optic);
	void jogBack(double mm, const OpticsInfo& optic);
	void jogUp(double mm, const OpticsInfo& optic);
	void jogDown(double mm, const OpticsInfo& optic);

	void homeX();
	void homeY();
	void homeZ();
	void homeXYZ();
	void homeAll();


	void signalOnlineStitchingSimulation();

	//setup
	void autoSetFiducialPoint(int currentFid);
	void testFiducial(int index, bool online);

	QString readBarcode(int index, bool online = true);

	//calibration
	void performCameraAlignment(dat::WorldCoordinate currentPoint, double step_mm, AlignFeatureParams featureParams);
	void performCameraScaling(dat::WorldCoordinate currentPoint, double step_mm, AlignFeatureParams featureParams);

	void performLaserAlignment(dat::WorldCoordinate currentPoint, QRectF roi, int camThreshold, int laserThreshold);
	void captureAlignmentImages(dat::WorldCoordinate currentPoint, int camThreshold, int laserThreshold);
	void performGuidedLaserAlignment(dat::WorldCoordinate currentPoint);
	void signalVerifyLaserAlignment(dat::WorldCoordinate currentPoint);

	void signalGetAllIntensityFromExpectedGV(QString camID, QString opticType, int idealR, int idealG, int idealB, QRectF roi);
	void calibrateGoldenLightingProfile(QString camID, QRectF roi);
	void calibrateCurrentLightingProfile(QString camID, QRectF roi);
	void triggerMaxCurrentCalibration(QString camID, QRectF roi, double plateauDiffThreshold, double maxCurrentAmp);

	void signalSetPortabilityPoint(PositionPortabilityType type);
	void signalFindPortabilityPattern();
	void signalFindPortabilityCircle();

	bool sendToClient(QString msg);

	void collectZImages(double x, double y, double step_mm, double firstStep, double finalStep, OpticsInfo optic);

	void signalLoadToPosition(int index);
	void barcodeReceived(QString payload);
	void signalUnloadBoard();
	void signalRunProdS();

	//SR-X readers: queued so the socket I/O runs in the job thread that owns the sockets
	void signalTriggerSRX();
	void signalStopSRX();
};




#endif // QVISIONAPP_H


/*
* => Resources for motion (APS_FunctionLibrary_V2.0.pdf): https://drive.google.com/drive/u/1/folders/1h5BAl9NtQk3XFxBIUn8lKX5N696naa8Y
* pg 892: Motion profile
* pg 984: Axis parameter table
* 
* => Flow 
* 1. VisionApp_Production::startProduction()
* Handles barcode logic, trigger load recipe and load pallet to inspection position.
* Jobthread::loadToPositionSensor() will trigger signalBoardInPosition() to VisionApp_Production::boardInPosition() if successful
* Loading position will timeout after 15s and stop production
* 
* 2. Subrecipe
* When trying to trace how subrecipe works, you can trace with _subrecipesToRun variable.
* During start production, it will be insert with 1, which tells the system, after VisionApp::inspectionDone(), it needs to trigger another subrecipe run
* 
* => Possible issues
* 1. Jog transition from 2d to 3d have possibility of having motion error. Reason unknown, could be due to speed profile being changed and need to wait around 100ms
* 2. Move log: unknown error, mostly due to speed profile being set with invalid parameters. (Ex. Acceleration = 0, Max speed over limit)
* 3. Set S-curve for unrelated axis like conveyor and rail, will result it them not being able to move.
* 4. Loading to Pos1 and Pos2 did not reach sensor, mostly due to material being unable to reflect sensor, putting tape under the pallet where sensor scan through should solve this issue.
* 5. Sometimes image capture might not get passed to inspection thread, look for "[ImageReady] In image" log in VisionApp.cpp to ensure it has been processed by Imagemanager. 
* 6. Conveyor acceleration need to be 1000
* 7. Homing speed cannot too fast, otherwise will overshoot sensor
*/
