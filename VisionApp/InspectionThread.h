#ifndef INSPECTIONTHREAD_H
#define INSPECTIONTHREAD_H

#include "CodeConfig.h"

#include <QThread>
#include <QDebug>
#include <QRect>
#include <QFileInfo>
#include <QJsonDocument>
#include <QtConcurrent/QtConcurrentMap>
#include <fstream>
#include <QDateTime>
#include "QJsonHelper.h"
#include "WinEvents.h"
#include "WinSharedMem.h"
#include "FrameInfo.h"
#include "ErrorInfo.h"
#include "OpenWorkspaceInfo.h"
#include "InspectionInfo.h"
#include "ResultInfo.h"
#include "MessageQue.h"
#include "PostInspectionInfo.h"
#include "ResultCache.h"
#include "QOSTool.h"
#include "VidiToolResult.h"
#include "QVisionObject.h"
#include "QView.h"
#include "QLineScan.h"
#include "VisionAppQDragBox.h"
#include "Algo.h"
#include "AlgoDefectResult.h"
#include "Timer.h"
#include <QXmlStreamReader>
#include "QCommonStruct.h"
#include "OpticsInfo.h"
#include "mtrx.h"
#include "AlgoCommonStruct.h"
#include "OnnxCommonStruct.h"
#include <omp.h>
#include "Utilities.h"

#include "OnnxInference.h"

#if HAS_VIDI_LICENSE
#include "vidi_runtime.h"
#include "vidi_training.h"
#endif

extern int g_viewMode;
extern MIL_INT g_imgType;
extern QString g_imgExtension;
extern int g_viewIndex;
extern bool g_forceStopInspLoop;
extern bool g_enableClassificationDataCollection;
extern ResultCache g_resultCache;
extern TMessageQue<PostInspectionInfo> g_postInspectionQueue;
extern TMessageQue<QVector<FrameInfo>> g_inspectionQueue;
extern Timer g_time;
extern QHash<QString, QPointF> g_locatorOffsets;
extern QVector< Onnx::InferenceEngine*> g_ODModels;
extern Onnx::InferenceEngine* g_segModel;
extern ObjectDetectionTilingSettings g_odTilingSettings;

class InspectionThread : public QThread
{
	Q_OBJECT

	enum Event {
		PRODUCTION,
		OFFLINE
	};

	enum PreprocessMethod {
		DIFF_OF_MEDIAN,
		HIGHLIGHT_DEFECT
	};

	enum class ViewMode {
		PLANE, SINGLE
	};

private:
	struct ODModelResults {
		QString modelName;
		QString opticName;
		QString opticID;
		QString channel;
		QString imageRotation;
		QString locatorId;
		QString algoType;
		bool enableSegmentation = false;
		std::unordered_map<std::string, std::vector<OnnxResult>> od_result;
	};

	struct PostInspResult {
		QString viewID;
		QJsonArray measurements;
		QVector<DistanceMeasurementInfo> distances;
		QVector<BarcodeDecoderInfo> barcodes;
		QVector<ct::AlgoDefectResult> defects;
		QVector<ct::AlgoDefectResult> debugResults;
	};

	struct WorkerCtx {
		std::unique_ptr<Algo>      algo;
		AlgoGraph* graph = nullptr;

		// per-iteration outputs
		QVector<QJsonObject> meas;
		QVector<ct::AlgoDefectResult>    defects;
		QVector<DistanceMeasurementInfo> dists;
		QVector <BarcodeDecoderInfo>     barcodeInfo;
		QVector<ct::AlgoDefectResult>    debugItems;
		QVector<DynamicDataObject>		 dataObjects;

		void resetForNext() {
			meas.clear(); defects.clear(); dists.clear(); debugItems.clear();
			if (graph)
			{
				graph->clearDefects();
				graph->clearMeasurementDatas();
				graph->clearDistanceMeasurement();
				graph->clearBarcodeDecoderData();
				graph->clearDynamicDataObjects();
			}		
		}
	};

	bool _inspResult;
	bool _isRunFail;
	bool _isVidiInit;
	int _cycleTime;
	std::atomic<bool> _scanInspBufferQueue;
	bool _isWorkspaceOpened;
	bool _saveInspImg = true;
	bool _saveDefectVoImg = true;
	bool _saveDefectRectVoImg = true;
	bool _dryRun = false;
	bool _isOffline = false;
	bool _saveBMPImg = false;
	QString _frameID;
	QString _workspaceOpened;
	WinEvents _appEvents;
	WinSharedMem _appSharedMem;
	ErrorInfo* _pErrorInfo;
	OpenWorkspaceInfo* _pOpenWorkspaceInfo;
	InspectionInfo* _pInspectionInfo;
	ResultInfo* _pResultInfo;
	FrameInfo* _pFrameInfo;
	QByteArray _resultBufferData;

	int _frameCount=0;
	FrameInfo _redFrameInfo;
	FrameInfo _greenFrameInfo;
	FrameInfo _blueFrameInfo;

	QString _errorMsg;
	QString _workspaceName;
	QString _workspacePath;

	QVector<VisionAppQDragBox*> _dragROI;
	QHash<QString, QVisionObject> _visionObjects;
 	QHash<QString, QView> _views;
	QHash<QString, QLineScan> _heightMaps;
	QPointF _planeOffset;
	Algo* _algo = nullptr;

	int _emptyPocketIndex = 1;
	QSet<QString> _viewsToRun;

	//defect
	QVector<ct::AlgoDefectResult> _algoDefectResults;
	QVector<ct::DefectResult> _defectResults;
	QHash<QString, bool> _locResults;
	QHash<QString, QHash<QString, QPointF>> _locatorOffsets; //QHash<roiID, QHash<locID, xyOffset>>
	QHash<QString, QHash<QString, LocAngle>> _locatorAngles;
	QHash<QString, QJsonObject> _vidiResults;
	QJsonArray _measArray;

	QVector<DistanceMeasurementInfo> _distanceMeasurementVector;
	QVector<BarcodeDecoderInfo> _barcodeDecoderInfoVector;


	QHash<QString, QJsonObject> _objectDetectionResult; //vor, odResult

	QHash<QString, QVector<DynamicMaskObject>> _segmentationResult; //vor, dynamicMaskObjects

	QHash<QString, QVector<DynamicDataObject>> _dataResult;


	//cad Variables
	QHash<QString, QVector<CadRoiInfo>> _templateCadRois;

	//defectPriorityList
	QVector<QString> _defectPriorityList;
	QHash<QString, DefectTag> _tagNameHash;
	QHash<QString, QString> _defectMappingHash;

	//opticsSettings
	OpticsInfo _mainOptics;
	QHash<QString, OpticsInfo> _recipeOptics;
	QHash<QString, OpticsInfo3D> _recipeOptics3D;
	QVector<OpticsInfo> _allOptics_2D_3D;

	//preprocessSettings
	bool _enableImagePreprocess = true;

	//run image Node Settings
	QStringList _imgPaths;
	QStringList _lightingIDs;
	QHash<QString, QPointF> _runImageOffset;
	QHash<QString, LocAngle> _runImageAngles;
	int _runImageVidiAlgoType = 1;
	AlgoGraph* _runImageAlgoGraph = nullptr;

	//ODModelsOpticsSettings
	QString _templateODOpticsPath;
	QHash<QString, QVector<AiModelInfo::opticInfo>> _odModelsOptics;

	//unitResultInfo
	QHash<QString, ct::UnitResultInfo> _unitResultInfos; //<opticID, unitResultInfo>

	//imgSize
	int _imgWidth = 0;
	int _imgHeight = 0;

	int _camChannel = 1;

	//segmentation Settings
	bool _enableSegmentation = false;
	double _segmentationScore = -1;

	int _expectedIndex = 0;
	int _expectedRow = 0;
	int _expectedCol = 0;
	int _numOfViewReceived = 0;

	QString _defectCollectorPath;

	bool vidiInit();
	bool openWorkspace();
	QStringList getResult(const QByteArray& val);
	bool vidiInspectionFromMemory(FrameInfo* pRedFrameInfo, FrameInfo* pGreenFrameInfo, FrameInfo* pBlueFrameInfo);
	
	bool vidiInspectionFromFile();
	bool vidiInspectionFromFiles(QVector<QStringList> imgPreprocessPaths, QStringList lightingIDs);
	bool preprocessFromFiles(QStringList imgPaths, QStringList lightingIDs, QVector<QStringList> & imgPreprocessPaths);
	void getRedToolHeatMap(QString uuid, QString heatMapImagePath);
	void strToChars(char* pCharArr, int arrSize, std::string src);
	std::string charsToStr(char* pCharArray);

	//ObjectDetection Fnc
	void loadObjectDetectionModelSettings(QString templateODOpticsPath);
	bool objectDetectionInspectionFromFiles(QStringList imgPaths, QStringList lightingIDs, QString templateODOpticsPath);
	bool samInspectionFromFiles(QStringList imgPaths, QStringList lightingIDs, AlgoGraph* algoGraph, const QHash<QString, QPointF>& offsets, const QHash<QString, LocAngle>& locAngles);
	bool wirebondInspectionFromFiles(QStringList imgPaths, QStringList lightingIDs, AlgoGraph* algoGraph, const QHash<QString, QPointF>& offsets, const QHash<QString, LocAngle>& locAngles);
	bool samInspectionTest();

	//optic Fnc
	bool alloptics2D3D_exist(QString opticName);

	//img Fnc
	void formImage(QImage& img, const unsigned char* pRedBuf, const unsigned char* pGreenBuf, const unsigned char* pBlueBuf);
	void formImage(QImage& img, const unsigned char* pImageBuf);

	//defect fnc
	void clearResults();
	void addDefects(QVector<ct::AlgoDefectResult> res, QView& view, bool locatorDefect = false);
	void generateDefectImages(QImage & img, QStringList & defectTagNames, const QString & voName, const QRect & voRect);
	void addDistanceMeasurement(QVector<DistanceMeasurementInfo> res, QView& view, bool locatorDefect = false);
	void addBarcodeDecoderInfoData(QVector<BarcodeDecoderInfo> info);
	bool defectFound(QString viewID);

	//unit defect for Live
	void addUnitBarcodeResults(QVector<BarcodeDecoderInfo> bInfoVector);
	void addUnitDebugResults(const QVector<ct::AlgoDefectResult> & debugResults, QView& view, bool locatorDefect = false);
	void addUnitDefectResults(const QVector<ct::AlgoDefectResult>& defectResults, QView& view, bool locatorDefect = false);
	void setUnitInspectionTime(double inspTime);

	//utility fnc
	void getVisionObjects(QHash<QString, QRectF> &visionObjectsRect, const QString & viewID);
	void getVisionObjectsForHeightMap(QHash<QString, QRectF> &visionObjectsRect, const QString & id, MIL_ID mImap = M_NULL);
	void offsetVisionObject(const QHash<QString, QPointF> & locatorOffsets, QString visionObjectName, QRect & rect);
	void removeLocOffsetfromRect(const QHash<QString, QPointF> & locatorOffsets, QString visionObjectName, QRectF & rect);

	//mil Fnc
	void performImagePreprocess(util::ImagePreprocess & imagePreprocess, const QRect rect, MIL_ID & src, MIL_ID & milChild, QString &preprocessOptic);
	
	//
	void performObjectDetectionPreprocess(const QRect rect, MIL_ID& src, cv::Mat& cvVoImage);
	void getImageChannel(const QString& channel, cv::Mat& inputImage, cv::Mat& outputImage);
	void getImageChannel(const QString& channel, const QString& imageRotation,const std::unordered_map<std::string, cv::Mat>& inputImages, std::unordered_map<std::string, cv::Mat>& outputImages);
	
	//Cad Fnc
	bool loadComponentCadRois(const QString &filePath, QVector<CadRoiInfo> & cadRois);
	//QString getCadTypeName(const QString& id);
	void insertDefectTagNames(QStringList &defectTagNames,const QStringList & tagNames);

	//ClassificationFnc
	void saveAllVisionObjectImages(const QHash<QString, QRectF> & visionObjectsRect, const  QHash<QString, QHash<QString, QPointF>> & locatorOffsets, QVector<FrameInfo> & pFrameInfos, QVector<MIL_ID> & milImgs, MIL_ID & mil_IMap);
	void classifyDefectImages();
	void generateDefectImages(QStringList & defectTagNames, QString& voName, QVector<OpticsInfo> & opticsInfos);

	//defectTagNames Fnc
	void changeSelectedTagNametoUnknown(QStringList &tagNames,const QString & selectedTagName);
	void arrangeTagNameBasedOnPriority(QStringList &tagNames);

	//getVIDIWorkSpaceInfo fnc
	void getVidiTools(QVector<ToolInfo> & toolInfos, const QString & streamName);
	void getVidiLabels(QVector<ToolInfo> & toolInfos, const QString & streamName);
	void getVidiToolParameters(QVector<ToolInfo> & toolInfos);
	bool serializeToolIInfos(QString fileName, QVector<ToolInfo> & toolInfos);
	bool deserializeToolInfos(QString fileName, QVector<ToolInfo> & toolInfos);

	//Algorithm fnc
	bool locatorInspection(const QHash<QString, QRectF> & visionObjectsRect, QHash<QString, QHash<QString, QPointF>> & locatorOffsets, QHash<QString, QHash<QString, LocAngle>>& locatorAngles, const QString & viewID);
	bool postInspection(const QHash<QString, QRectF> & visionObjectsRect,const QHash<QString, QHash<QString, QPointF>> & locatorOffsets, const QString & imgType);
	bool multi_postInspection(const QHash<QString, QRectF>& visionObjectsRect, const QHash<QString, QHash<QString, QPointF>>& locatorOffsets, const QString& imgType);
	bool multi_postInspection_v2(const QHash<QString, QRectF>& visionObjectsRect, const QHash<QString, QHash<QString, QPointF>>& locatorOffsets, const QString& imgType);
	bool generateIgnoreDefect(const QHash<QString, QRectF> & visionObjectsRect, QString skipDefectName);
	QPointF getAlgoLocatorOffset(const QJsonObject& algoObj, const QHash<QString, QPointF>& offsets);

	//Events Fnc
	void VidiNodeInit();
	void VidiNodeDeInit();
	void VidiNodeOpenWorkspace();
	void SoftTriggerProductionInspection();
	void OfflineHybridInspection();
	void GenerateVidiWorkspaceInfo();
	void SoftTrigger();
	void VidiNodeRunImage();
	void AIWireBondInspectionRunImage();

	//Thread Inspection Loop
	void mainInspectionLoop(int mode);

	//utility fnc
	bool loadJson(QString path, QJsonObject& root);
	std::unordered_map<std::string, std::list<cv::Vec6f>> removeOutOfBoundLinesMap(std::unordered_map<std::string, std::list<cv::Vec6f>> linesMap, QSize imageSize);
	std::vector<LineData> removeOutOfBoundLineData(std::vector<LineData> linesData, QSize imageSize);


	bool objectDetectionFromMemory(const QHash<QString, QRectF>& visionObjectsRect, const  QHash<QString, QHash<QString, QPointF>>& locatorOffsets, QVector<FrameInfo>& pFrameInfos, QVector<MIL_ID>& milImgs, bool locatorODFlag = false);
	bool segmentationInspectionFromMemory(const QHash<QString, QRectF>& visionObjectsRect, const  QHash<QString, QHash<QString, QPointF>>& locatorOffsets,const QHash<QString, QHash<QString, LocAngle>>& locatorAngles, QVector<FrameInfo>& pFrameInfos, QVector<MIL_ID>& milImgs);
	bool wirebondInspectionPointsFromMemory(const QHash<QString, QRectF>& visionObjectsRect, const  QHash<QString, QHash<QString, QPointF>>& locatorOffsets, const QHash<QString, QHash<QString, LocAngle>>& locatorAngles, QVector<FrameInfo>& pFrameInfos, QVector<MIL_ID>& milImgs);
	QString defectMapping(QString& tagName);
	void cadTagNameMapping(ct::DefectResult& dResult, QString cadFamily);
	QVector<CadFamilyInfo>_cadFamilyInfos;
	
#if HAS_VIDI_LICENSE
	//VIDI fnc
	VIDI_UINT _status;
	VIDI_BUFFER _buffer;
	bool formVidiImage(FrameInfo* pFrameInfo, VIDI_IMAGE* img);
	bool releaseVidiImage(VIDI_IMAGE* img);
	bool cropVidiImages(const QRect rect, VIDI_IMAGE* src, VIDI_IMAGE* croppedImage);
	bool childVidiImages(const QRect rect, MIL_ID & src, VIDI_IMAGE* childImage);
	bool childVidiImagestoJPG(const QRect rect, MIL_ID & src, VIDI_IMAGE* childImage);
	bool childVidiImagePreprocess(util::ImagePreprocess & imagePreprocess, const QRect rect, MIL_ID & src, VIDI_IMAGE* childImage, PreprocessMethod method = DIFF_OF_MEDIAN);
	bool vidiInspectionFromMemory(const QHash<QString, QRectF> & visionObjectsRect, const  QHash<QString, QPointF> & locatorOffsets, QVector<FrameInfo> & pFrameInfos, QVector<MIL_ID> & milImgs);
	bool vidiInspection(VIDI_IMAGE* Image, QString objectID, bool saveResult, QJsonObject & vidiResult, QString opticID = "default", QString streamName = "default");
	bool checkVidiStreamExist(QString streamName);
	void cadClassificationInspection(const QHash<QString, QRectF> & visionObjectsRect, const  QHash<QString, QPointF> & locatorOffsets, QVector<QImage> & qImgs, QVector<FrameInfo> & pFrameInfos, QVector<MIL_ID> & milImgs);
	QStringList classificationInspection(QString & streamName, const QString & voName, VIDI_IMAGE * vidiImg);
#endif

	void cadDefectMapping(const QHash<QString, QRectF> & visionObjectsRect, const  QHash<QString, QHash<QString, QPointF>> & locatorOffsets);

public:
	explicit InspectionThread(QObject *parent = 0);
	~InspectionThread();

	enum class CountMode {
		VIEW, INDEX, ROWCOL
	};

	CountMode _countMode = CountMode::VIEW;

	void run();
	void release();
	void startRun();
	void stopRun();
	void initMultiThread();

	void startScanInspBufferQueue(int mode); //0: production, 1: offline
	void stopScanInspBufferQueue();	//stop inspection Run 
	void closeVidiWorkSpace();
	bool loadComponentCadTypeLibrary();

	//set VisionObject Infos
	void clearLocatorResults();
	void clearAllOpticsInfos_2D3D();
	void setVisionAppDragBox(const QVector <VisionAppQDragBox*> dragROI);
	void setVisionObject(const QHash<QString, QVisionObject> & visionObjects);
	void setPlaneOffset(const QPointF &planeOffset);
	void setViews(const QHash<QString, QView> & views);
	void setHeightMap(const QHash<QString, QLineScan> & heightMaps);
	void setAlgo(Algo* algo);
	void setCadRoiInfos(QHash<QString, QVector<CadRoiInfo>> & templateCadRois);
	void setDefectPriorityList(QVector<QString> & defectPriorityList, QHash<QString, DefectTag> tagNameHash, QHash<QString, QString> defectMappingHash);
	void setOpticInfos(OpticsInfo & mainOptics, QHash<QString, OpticsInfo> & recipeOptics, QHash<QString, OpticsInfo3D> & recipeOptics3D);
	void setEnableImagePreprocess(bool enable);
	void setSaveInspImg(bool enable);
	void setsaveDefectVoImg(bool enable);
	void setsaveDefectRectVoImg(bool enable);
	void setDryRunFlag(bool enable);
	void setCamChannel(int camChannel);
	void setSegmentationSettings(bool enableSegmentation, double segmentScore);
	void setIsOffline(bool flag);
	void setSaveBMPImg(bool enable);
	//bool loadComponentCadTypeLibrary();

	//set Run Single Image Node Settings
	void setImagePaths(QStringList imagePaths, QStringList lightingIDs);
	void setODModelOpticSettingsPath(QString filePath);
	void setRunImageVidiAlgoType(int type);
	void setRunImageOffset(QHash<QString, QPointF> offsets);
	void setRunImageAlgoGraph(AlgoGraph* algoGraph);
	void setDefectCollectorPath(QString path);

	QJsonArray measurementArray();

	void objectDetectionResultToJsonObject(QVector<ODModelResults> odModelResults, QStringList totalVoNames);
	void objectDetectionResultToJsonObject(std::unordered_map<std::string, std::vector<OnnxResult>>& odResult);
	void objectDetectionResultToJsonObject(QVector<std::vector<OnnxResult>> & odResult, QVector<AiModelInfo::opticInfo> & odOpticsList, QStringList modelList);

	void setCountMode(CountMode mode);
	void setExpectedIndex(int expectedIndex);
	int getExpectedIndex();

	bool isStarted();

signals:
	void inspectionDone(QVector<ct::DefectResult>& defectResults, QVector<BarcodeDecoderInfo>& barcodeInfoVector);
	void locatorInfo(QPointF locatorOffsets, double locatorAngle, QString viewID, QString indexID, bool locatorFail, bool locatorAngleFail);
	void updateInspectionProgressBar();
	void runLooping();
	void displayLiveImage(QVector<FrameInfo> frameInfos, QHash<QString, ct::UnitResultInfo>  unitResultInfos);
};

#endif // INSPECTIONTHREAD_H