#include "InspectionThread.h"
#include "CommonDir.h"
#include "uidGenerator.h"
#include <QPainter>
#include "utilities.h"
#include "Logger.h"
#include "ProfilerManager.h"
#include "SystemData.h"
#include "ScaleManager.h"
#include "ImageSavingThread.h"
#include "MachineController.h"

InspectionThread::InspectionThread(QObject* parent) : QThread(parent), _scanInspBufferQueue(false)
{
	_appEvents.createEvent(std::string("VidiNodeInit"));
	_appEvents.createEvent(std::string("VidiNodeOpenWorkspace"));
	_appEvents.createEvent(std::string("VidiNodeRunImage"));
	_appEvents.createEvent(std::string("AIWireBondInspectionRunImage"));
	_appEvents.createEvent(std::string("SoftTriggerProductionInspection"));
	_appEvents.createEvent(std::string("VidiNodeDeInit"));
	_appEvents.createEvent(std::string("VidiNodeReturn"));
	_appEvents.createEvent(std::string("OfflineHybridInspection"));
	_appEvents.createEvent(std::string("GenerateVidiWorkspaceInfo"));

	_appSharedMem.createMemory(std::string("FrameInfo"), sizeof(FrameInfo));
	_appSharedMem.createMemory(std::string("ErrorInfo"), sizeof(ErrorInfo));
	_appSharedMem.createMemory(std::string("OpenWorkspaceInfo"), sizeof(OpenWorkspaceInfo));
	_appSharedMem.createMemory(std::string("InspectionInfo"), sizeof(InspectionInfo));
	_appSharedMem.createMemory(std::string("ResultInfo"), sizeof(ResultInfo));

	_pFrameInfo = reinterpret_cast<FrameInfo*>(_appSharedMem.getMemory(std::string("FrameInfo")));
	_pErrorInfo = reinterpret_cast<ErrorInfo*>(_appSharedMem.getMemory(std::string("ErrorInfo")));
	_pOpenWorkspaceInfo = reinterpret_cast<OpenWorkspaceInfo*>(_appSharedMem.getMemory(std::string("OpenWorkspaceInfo")));
	_pInspectionInfo = reinterpret_cast<InspectionInfo*>(_appSharedMem.getMemory(std::string("InspectionInfo")));
	_pResultInfo = reinterpret_cast<ResultInfo*>(_appSharedMem.getMemory(std::string("ResultInfo")));
}

void InspectionThread::startRun()
{

	if (isRunning() == false)
	{
		start(QThread::HighestPriority);
	}
}

void InspectionThread::startScanInspBufferQueue(int mode)
{
	ct::logger::info("[InspectionThread] START scan inspection buffer queue: %d", mode);

	clearResults();
	g_inspectionQueue.clear();
	_scanInspBufferQueue = true;
	if (mode == PRODUCTION) SetEvent(_appEvents.getEvent(std::string("SoftTriggerProductionInspection")));
	else if (mode == OFFLINE) SetEvent(_appEvents.getEvent(std::string("OfflineHybridInspection")));

	if (!_scanInspBufferQueue)
	{

	}

	_viewsToRun.clear();
	for (const auto& v : _views) {
		if (v.type == ct::s_child_view) continue;
		if (v.id == "") continue;
		_viewsToRun.insert(v.id);
	}
	for (const auto& v : _heightMaps) {
		if (v.type == ct::s_child_linescan) continue;
		if (v.id == "") continue;
		_viewsToRun.insert(v.id);
	}
}

void InspectionThread::stopScanInspBufferQueue()
{
	ct::logger::info("[InspectionThread] STOP scan inspection buffer queue: %d", g_inspectionQueue.size());

	_scanInspBufferQueue = false;
	g_inspectionQueue.clear();

	ct::logger::info("[InspectionThread] STOP flag set");
}

void InspectionThread::closeVidiWorkSpace()
{
#if HAS_VIDI_LICENSE
	if (!_workspaceOpened.isEmpty())
	{
		vidi_runtime_close_workspace(_workspaceOpened.toStdString().c_str());
		_workspaceOpened.clear();
		_isWorkspaceOpened = false;
	}
#endif
}

void InspectionThread::clearLocatorResults()
{
	_locatorOffsets.clear();
	_locResults.clear();
	_objectDetectionResult.clear();
	_dataResult.clear();
}

void InspectionThread::clearAllOpticsInfos_2D3D()
{
	_allOptics_2D_3D.clear();
}

void InspectionThread::setVisionAppDragBox(const QVector<VisionAppQDragBox*> dragROI)
{
	_dragROI = dragROI;
}

void InspectionThread::setVisionObject(const QHash<QString, QVisionObject>& visionObjects)
{
	_visionObjects = visionObjects;
	for (auto& vo : _visionObjects)
	{
		if (vo.ignore) qDebug() << "[INSPECTION THREAD] Incoming VO: " << vo.objectName;
	}
}

void InspectionThread::setPlaneOffset(const QPointF& planeOffset)
{
	_planeOffset = planeOffset;
}

void InspectionThread::setViews(const QHash<QString, QView>& views)
{
	_views = views;
}

void InspectionThread::setAlgo(Algo* algo)
{
	_algo = algo;
	_algo->releaseBuffer();
}

void InspectionThread::setCadRoiInfos(QHash<QString, QVector<CadRoiInfo>>& templateCadRois)
{
	_templateCadRois = templateCadRois;
	loadComponentCadTypeLibrary();
}

void InspectionThread::setDefectPriorityList(QVector<QString>& defectPriorityList, QHash<QString, DefectTag> tagNameHash, QHash<QString, QString> defectMappingHash)
{
	_defectPriorityList = defectPriorityList;
	_tagNameHash = tagNameHash;
	_defectMappingHash = defectMappingHash;
}

void InspectionThread::setOpticInfos(OpticsInfo& mainOptics, QHash<QString, OpticsInfo>& recipeOptics, QHash<QString, OpticsInfo3D>& recipeOptics3D)
{
	_mainOptics = mainOptics;
	_recipeOptics = recipeOptics;
	_recipeOptics3D = recipeOptics3D;
}

void InspectionThread::setEnableImagePreprocess(bool enable)
{
	_enableImagePreprocess = enable;
}

void InspectionThread::setSaveInspImg(bool enable)
{
	_saveInspImg = enable;
}

void InspectionThread::setsaveDefectVoImg(bool enable)
{
	_saveDefectVoImg = enable;
}

void InspectionThread::setsaveDefectRectVoImg(bool enable)
{
	_saveDefectRectVoImg = enable;
}

void InspectionThread::setDryRunFlag(bool enable)
{
	_dryRun = enable;
}

void InspectionThread::setCamChannel(int camChannel)
{
	_camChannel = camChannel;
}

void InspectionThread::setSegmentationSettings(bool enableSegmentation, double segmentScore)
{
	_enableSegmentation = enableSegmentation;
	_segmentationScore = segmentScore;
}

void InspectionThread::setIsOffline(bool flag)
{
	_isOffline = true;
}

void InspectionThread::setSaveBMPImg(bool enable)
{
	_saveBMPImg = enable;
}

void InspectionThread::run()
{
	ct::logger::info("[QThread] Inspection thread started");

	HANDLE hThr[8];
	hThr[0] = _appEvents.getEvent(std::string("VidiNodeInit"));
	hThr[1] = _appEvents.getEvent(std::string("VidiNodeOpenWorkspace"));
	hThr[2] = _appEvents.getEvent(std::string("SoftTriggerProductionInspection"));
	hThr[3] = _appEvents.getEvent(std::string("AIWireBondInspectionRunImage"));
	hThr[4] = _appEvents.getEvent(std::string("VidiNodeRunImage"));
	hThr[5] = _appEvents.getEvent(std::string("OfflineHybridInspection"));
	hThr[6] = _appEvents.getEvent(std::string("GenerateVidiWorkspaceInfo"));
	hThr[7] = _appEvents.getEvent(std::string("VidiNodeDeInit"));

	_isVidiInit = false;
	_workspaceOpened.clear();

	for (;;)
	{
		DWORD ret = WaitForMultipleObjects(8, hThr, FALSE, INFINITE);
		ct::logger::debug("[InspectionThread Run] ret: %d", ret);
		if (ret == WAIT_OBJECT_0) //vidiNodeInit
		{
			qDebug() << "vidiNodeInit";
			VidiNodeInit();
		}
		else if (ret == WAIT_OBJECT_0 + 1) //vidiOpenWorkSpace
		{
			qDebug() << "vidiOpenWorkSpace";
			VidiNodeOpenWorkspace();
		}
		else if (ret == WAIT_OBJECT_0 + 2) //softTriggerProductionInspection
		{
			SoftTriggerProductionInspection();
		}
		else if (ret == WAIT_OBJECT_0 + 3) //AIWireBondInspectionRunImage
		{
			AIWireBondInspectionRunImage();
		}
		else if (ret == WAIT_OBJECT_0 + 4) //vidiNodeRunImage
		{
			VidiNodeRunImage();
		}
		else if (ret == WAIT_OBJECT_0 + 5) //OfflineHybridInspection
		{
			qDebug() << "OfflineHybridInspection";
			OfflineHybridInspection();
		}
		else if (ret == WAIT_OBJECT_0 + 6) //GenerateVidiWorkspaceInfo
		{
			qDebug() << "GenerateVidiWorkspaceInfo";
			GenerateVidiWorkspaceInfo();
		}
		else if (ret == WAIT_OBJECT_0 + 7) //vidiNodeDeinitialize
		{
			qDebug() << "vidiNodeDeinitialize";
			VidiNodeDeInit();
			qDebug() << "vidiNodeDeinitialize - end";
			break;
		}

		ct::logger::debug("finish one run");
	}
	
	ct::logger::info("Inspection thread fully released");
}

void InspectionThread::release()
{
	SetEvent(_appEvents.getEvent(std::string("VidiNodeDeInit")));
	ct::logger::info("[InspectionThread] Releasing...");
}

bool InspectionThread::vidiInit()
{
#if HAS_VIDI_LICENSE
	// send debug info to a message file
	_status = vidi_debug_infos(VIDI_DEBUG_SINK_FILE, "vidi_messages.log");
	if (_status != VIDI_SUCCESS)
	{
		_errorMsg = QStringLiteral("Fail to startup debug info");
		return false;
	}

	// initialize the libary to run with one GPU per tool
	_status = vidi_initialize(VIDI_GPU_SINGLE_DEVICE_PER_TOOL, "");
	if (_status != VIDI_SUCCESS)
	{
		_errorMsg = QStringLiteral("Fail to startup library");
		return false;
	}

	// create and initialize a buffer to be used whenever data is returned from the library
	_status = vidi_init_buffer(&_buffer);

	if (_status != VIDI_SUCCESS)
	{
		_errorMsg = QStringLiteral("Fail to startup buffer");
		qDebug() << _errorMsg;
		return false;
	}

	/*_status = vidi_optimized_gpu_memory(0);
	if (_status != VIDI_SUCCESS)
	{
		_errorMsg = QStringLiteral("Fail to close optimized gpu memory");
		return false;
	}*/
#endif

	return true;
}

bool InspectionThread::openWorkspace()
{
#if HAS_VIDI_LICENSE
	_isWorkspaceOpened = false;
	if (!_workspaceOpened.isEmpty())
	{
		vidi_runtime_close_workspace(_workspaceOpened.toStdString().c_str());
		_workspaceOpened.clear();
	}

	_workspaceName = QString::fromStdString(charsToStr(_pOpenWorkspaceInfo->_workspaceName));
	_workspacePath = QString::fromStdString(charsToStr(_pOpenWorkspaceInfo->_workspacePath));
	qDebug() << "_workspaceName" << _workspaceName;
	qDebug() << "_workspacePath" << _workspacePath;

	_status = vidi_runtime_open_workspace_from_file(_workspaceName.toStdString().c_str(), _workspacePath.toStdString().c_str());

	if (_status != VIDI_SUCCESS)
	{
		vidi_get_error_message(_status, &_buffer);
		_errorMsg = QStringLiteral("Fail to open workspace, error message: %1").arg(QString(_buffer.data));

		return false;
	}

	_workspaceOpened = _workspaceName;
	_isWorkspaceOpened = true;
#endif
	return true;
}

QStringList InspectionThread::getResult(const QByteArray& val)
{
	QStringList blueResultList;
	//
	//QJsonDocument doc = QJsonDocument::fromJson(val);
	//QJsonObject root = doc.object();
	//QJsonObject sampleObj = root[QStringLiteral("sample")].toObject();

	//QJsonObject imageObj = jsonHelper::getObject(sampleObj, QStringLiteral("image"));
	//QJsonArray markingArr = jsonHelper::getArray(imageObj, QStringLiteral("marking"));
	//for (int i = 0; i < markingArr.count(); i++)
	//{
	//	QJsonObject markingArrObj = markingArr[i].toObject();
	//	QString toolType = jsonHelper::getString(markingArrObj, QStringLiteral("tool_type"));

	//	QJsonArray viewArr = jsonHelper::getArray(markingArrObj, QStringLiteral("view"));
	//	if (viewArr != QJsonArray())
	//	{
	//		for (int j = 0; j < viewArr.count(); j++)
	//		{
	//			QJsonObject viewArrObj = viewArr[j].toObject();
	//			QJsonObject toolTypeObj = jsonHelper::getObject(viewArrObj, toolType);

	//			if (toolType == QStringLiteral("blue_read"))
	//			{
	//				QJsonArray matchArr = jsonHelper::getArray(toolTypeObj, QStringLiteral("match"));
	//				if (matchArr != QJsonArray())
	//				{
	//					for (int l = 0; l < matchArr.count(); l++)
	//					{
	//						QJsonObject matchArrObj = matchArr[l].toObject();
	//						blueResultList.append(jsonHelper::getString(matchArrObj, QStringLiteral("string")).remove(QStringLiteral(" ")));
	//					}
	//				}
	//				else
	//				{
	//					// might need to handle if nothing matched
	//				}
	//			}
	//		}
	//	}
	//}

	return blueResultList;
}

bool InspectionThread::vidiInspectionFromMemory(FrameInfo* pRedFrameInfo, FrameInfo* pGreenFrameInfo, FrameInfo* pBlueFrameInfo)
{
#if HAS_VIDI_LICENSE
	VIDI_IMAGE image;
	_status = vidi_init_image(&image);
	if (_status != VIDI_SUCCESS)
	{
		vidi_get_error_message(_status, &_buffer);
		_errorMsg = QStringLiteral("Fail to initialise inspection image, error message: %1").arg(QString(_buffer.data));
		return false;
	}

	// Construct color image
	_status = vidi_create_image(pRedFrameInfo->_width, pRedFrameInfo->_height, pRedFrameInfo->_width, VIDI_IMG_8U, reinterpret_cast<void*>(pBlueFrameInfo->_pImageBuf), reinterpret_cast<void*>(pGreenFrameInfo->_pImageBuf), reinterpret_cast<void*>(pRedFrameInfo->_pImageBuf), nullptr, &image);
	if (_status != VIDI_SUCCESS)
	{
		vidi_get_error_message(_status, &_buffer);
		_errorMsg = QStringLiteral("Fail to create color image, error message: %1").arg(QString(_buffer.data));
		return false;
	}

	//collect Image
	if (_pInspectionInfo->_isCollectImage)
	{
		QString imagePath = QStringLiteral("C:/SNIPER/Train Image/%1.bmp").arg(_frameID);
		vidi_save_image(imagePath.toStdString().c_str(), &image);
		return true;
	}

	VIDI_BUFFER result_buffer;
	_status = vidi_init_buffer(&result_buffer);
	if (_status != VIDI_SUCCESS)
	{
		_errorMsg = QStringLiteral("Fail to initialise inspection result buffer");
		return false;
	}

	// as of ViDi Suite 3.0.0, samples are processed in a few steps instead of just calling vidi_runtime_process
	// the first step is to initialize the sample
	_status = vidi_runtime_create_sample(_workspaceName.toStdString().c_str(), "default", "my_sample");
	if (_status != VIDI_SUCCESS)
	{
		_errorMsg = QStringLiteral("Fail to initialise inspection sample");
		return false;
	}

	// then add the image to be processed
	_status = vidi_runtime_sample_add_image(_workspaceName.toStdString().c_str(), "default", "my_sample", &image);
	if (_status != VIDI_SUCCESS)
	{
		_errorMsg = QStringLiteral("Fail to add inspection image");
		return false;
	}

	// process image
	_status = vidi_runtime_sample_process(_workspaceName.toStdString().c_str(), "default", "", "my_sample", "");
	if (_status != VIDI_SUCCESS)
	{
		vidi_get_error_message(_status, &_buffer);
		_errorMsg = QStringLiteral("Fail to process inspection sample, error message: %1").arg(QString(_buffer.data));
		return false;
	}

	// the next step is to get the results
	_status = vidi_runtime_get_sample_json(_workspaceName.toStdString().c_str(), "default", "my_sample", &result_buffer);
	if (_status != VIDI_SUCCESS)
	{
		_errorMsg = QStringLiteral("Fail to get inspection result");
		return false;
	}

	// process result
	_resultBufferData = QByteArray(result_buffer.data);



	// free resources
	_status = vidi_free_buffer(&result_buffer);
	if (_status != VIDI_SUCCESS)
	{
		_errorMsg = QStringLiteral("Fail to free inspection result buffer");
		return false;
	}

	_status = vidi_free_image(&image);
	if (_status != VIDI_SUCCESS)
	{
		_errorMsg = QStringLiteral("Fail to free inspection image");
		vidi_runtime_free_sample(_workspaceName.toStdString().c_str(), "default", "my_sample");
		return false;
	}

	_status = vidi_runtime_free_sample(_workspaceName.toStdString().c_str(), "default", "my_sample");
	if (_status != VIDI_SUCCESS)
	{
		_errorMsg = QStringLiteral("Fail to free inspection sample");
		return false;
	}

#endif
	return true;
}

void InspectionThread::performImagePreprocess(util::ImagePreprocess& imagePreprocess, const QRect rect, MIL_ID& src, MIL_ID& milChild, QString& preprocessOptic)
{
	MIL_INT bandSize;
	MbufInquire(src, M_SIZE_BAND, &bandSize);

	milChild = MbufAllocColor(M_DEFAULT, bandSize, rect.width(), rect.height(), 8 + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);
	//MbufCopyColor2d(src, milChild, M_ALL_BANDS, rect.x(), rect.y(), M_ALL_BANDS, 0, 0, rect.width(), rect.height());
	MbufClear(milChild, M_BLACK);
	auto imgW = mtrx::get_width(src);
	auto imgH = mtrx::get_height(src);

	int endX = rect.x() + rect.width();
	int endY = rect.y() + rect.height();

	int cappedWidth = rect.width();
	int cappedHeight = rect.height();
	if (endX > imgW) cappedWidth = imgW - rect.x();
	if (endY > imgH) cappedHeight = imgH - rect.y();

	int startX = 0;
	int startY = 0;
	int srcX = rect.x();
	int srcY = rect.y();
	if (rect.x() < 0)
	{
		startX = -rect.x();
		srcX = 0;
		cappedWidth = imgW - startX;
	}
	if (rect.y() < 0)
	{
		startY = -rect.y();
		srcY = 0;
		cappedHeight = imgH - startY;
	}

	//calculate exceed how much, then cap the copy
	MbufCopyColor2d(src, milChild, M_ALL_BANDS, srcX, srcY, M_ALL_BANDS, startX, startY, cappedWidth, cappedHeight);
	//qDebug() << "CAPPPED IMAGE:srcX: " << srcX << ", srcY:" << srcY << ", startX" << startX << ", startY:" << startY << " cappedWidth:" << cappedWidth << " cappedHeight:" << cappedHeight;
	//MbufSaveA("cappedImg.jpg", milChild);


	if (_enableImagePreprocess)
	{
		if (preprocessOptic == "RB_DiffOfMedianFilter")
		{
			imagePreprocess.Diff_of_medianFilter(milChild, milChild);
		}
		else if (preprocessOptic == "RGB_HighlightDefects")
		{
			imagePreprocess.HighlightDarkDefects(milChild, milChild);
		}
		else if (preprocessOptic == "DieLight")
		{
			imagePreprocess.medianFilter(milChild, milChild);
		}
		else if (preprocessOptic == "DieLight_DiffOfMedianFilter")
		{
			imagePreprocess.Diff_of_medianFilter(milChild, milChild, 8, 255, 0, 3, 15);
		}
	}

}

void InspectionThread::performObjectDetectionPreprocess(const QRect rect, MIL_ID& src, cv::Mat& cvVoImage)
{
	MIL_INT bandSize;
	MbufInquire(src, M_SIZE_BAND, &bandSize);
	

	auto imgW = mtrx::get_width(src);
	auto imgH = mtrx::get_height(src);

	int endX = rect.x() + rect.width();
	int endY = rect.y() + rect.height();

	int cappedWidth = rect.width();
	int cappedHeight = rect.height();
	if (endX > imgW) cappedWidth = imgW - rect.x();
	if (endY > imgH) cappedHeight = imgH - rect.y();

	int startX = 0;
	int startY = 0;
	int srcX = rect.x();
	int srcY = rect.y();
	if (rect.x() < 0)
	{
		startX = -rect.x();
		srcX = 0;
		cappedWidth = imgW - startX;
	}
	if (rect.y() < 0)
	{
		startY = -rect.y();
		srcY = 0;
		cappedHeight = imgH - startY;
	}


	//MbufCopyColor2d(src, milChild, M_ALL_BANDS, srcX, srcY, M_ALL_BANDS, startX, startY, cappedWidth, cappedHeight);

	MIL_ID croppedMilImg = MbufChild2d(src, srcX, srcY, cappedWidth, cappedHeight, M_NULL);

	cv::Mat cvVoImg;
	util::Mil_to_cv(croppedMilImg, cvVoImg);
	cvVoImage = cvVoImg;
	MbufFree(croppedMilImg);
}

void InspectionThread::getImageChannel(const QString& channel, cv::Mat& inputImage, cv::Mat& outputImage)
{
	qDebug() << "[VisionApp] " << channel;
	std::vector<cv::Mat> channels;
	cv::split(inputImage, channels); // Split once
	if (channel == "Red")
	{
		// Use the Red channel (index 2)
		std::vector<cv::Mat> redChannels = { channels[2], channels[2], channels[2] };
		cv::merge(redChannels, outputImage);
	}
	else if (channel == "Green")
	{
		// Use the Green channel (index 1)
		std::vector<cv::Mat> greenChannels = { channels[1], channels[1], channels[1] };
		cv::merge(greenChannels, outputImage);
	}
	else if (channel == "Blue")
	{
		// Use the Blue channel (index 0)
		std::vector<cv::Mat> blueChannels = { channels[0], channels[0], channels[0] };
		cv::merge(blueChannels, outputImage);
	}
}

void InspectionThread::getImageChannel(const QString& channel, const QString& imageRotation, const std::unordered_map<std::string, cv::Mat>& inputImages, std::unordered_map<std::string, cv::Mat>& outputImages)
{
	for (auto i : inputImages)
	{
		std::string imageId = i.first;
		cv::Mat inputImage = i.second;

		cv::Mat outputImage;

		std::vector<cv::Mat> channels;
		cv::split(inputImage, channels); // Split once
		if (channel == "Red")
		{
			// Use the Red channel (index 2)
			std::vector<cv::Mat> redChannels = { channels[2], channels[2], channels[2] };
			cv::merge(redChannels, outputImage);
		}
		else if (channel == "Green")
		{
			// Use the Green channel (index 1)
			std::vector<cv::Mat> greenChannels = { channels[1], channels[1], channels[1] };
			cv::merge(greenChannels, outputImage);
		}
		else if (channel == "Blue")
		{
			// Use the Blue channel (index 0)
			std::vector<cv::Mat> blueChannels = { channels[0], channels[0], channels[0] };
			cv::merge(blueChannels, outputImage);
		}
		else outputImage = inputImage;

		if (imageRotation == "90") cv::rotate(outputImage, outputImage, cv::ROTATE_90_CLOCKWISE);
		else if (imageRotation == "180") cv::rotate(outputImage, outputImage, cv::ROTATE_180);
		else if (imageRotation == "270") cv::rotate(outputImage, outputImage, cv::ROTATE_90_COUNTERCLOCKWISE);

		outputImages[imageId] = outputImage;
	}
}

void InspectionThread::setImagePaths(QStringList imagePaths, QStringList lightingIDs)
{
	_imgPaths = imagePaths;
	_lightingIDs = lightingIDs;
}

void InspectionThread::setODModelOpticSettingsPath(QString filePath)
{
	_templateODOpticsPath = filePath;
}

void InspectionThread::setRunImageVidiAlgoType(int type)
{
	_runImageVidiAlgoType = type;
}

void InspectionThread::setRunImageOffset(QHash<QString,QPointF> offsets)
{
	_runImageOffset = offsets;
}

void InspectionThread::setRunImageAlgoGraph(AlgoGraph* algoGraph)
{
	_runImageAlgoGraph = algoGraph;
}

QJsonArray InspectionThread::measurementArray()
{
	return _measArray;
}

bool InspectionThread::loadComponentCadRois(const QString& filePath, QVector<CadRoiInfo>& cadRois)
{
	QJsonObject root;

	QString val;
	QFile file;
	QJsonDocument doc;

	bool flag = false;
	if (QFile::exists(filePath))
	{
		file.setFileName(filePath);
		if (file.open(QIODevice::ReadOnly | QIODevice::Text))
		{
			val = file.readAll();
			file.close();

			doc = QJsonDocument::fromJson(val.toUtf8());
			root = doc.object();

			flag = true;
		}
	}

	if (!root.contains("CadRois"))
	{
		flag = false;
	}

	cadRois.clear();
	auto cadRoiInfos = root["CadRois"].toArray();

	for (int i = 0; i < cadRoiInfos.size(); i++) {
		auto cadRoiInfo = cadRoiInfos[i].toObject();
		CadRoiInfo cadRoi;
		cadRoi.familyId = jsonHelper::getString(cadRoiInfo, QStringLiteral("familyId"));
		cadRoi.familyName = jsonHelper::getString(cadRoiInfo, QStringLiteral("familyName"));
		cadRoi.name = jsonHelper::getString(cadRoiInfo, QStringLiteral("name"));
		cadRoi.id = jsonHelper::getString(cadRoiInfo, QStringLiteral("id"));
		cadRoi.x = jsonHelper::getDouble(cadRoiInfo, QStringLiteral("x"));
		cadRoi.y = jsonHelper::getDouble(cadRoiInfo, QStringLiteral("y"));
		cadRoi.w = jsonHelper::getDouble(cadRoiInfo, QStringLiteral("w"));
		cadRoi.h = jsonHelper::getDouble(cadRoiInfo, QStringLiteral("h"));
	
		cadRois.append(cadRoi);
	}


	return flag;
}

bool InspectionThread::loadComponentCadTypeLibrary()
{
	//auto jsonPath = Common::Directory::ConfigPath() + "ComponentCadTypeLibrary.json";
	auto jsonPath = Common::Directory::getRecipeCurrentPath() + "/ComponentCadTypeLibrary.json";

	QJsonObject root;

	QString val;
	QFile file;
	QJsonDocument doc;

	bool flag = false;
	if (QFile::exists(jsonPath))
	{
		file.setFileName(jsonPath);
		if (file.open(QIODevice::ReadOnly | QIODevice::Text))
		{
			val = file.readAll();
			file.close();

			doc = QJsonDocument::fromJson(val.toUtf8());
			root = doc.object();

			flag = true;
		}
	}

	if (!root.contains("CadInfos"))
	{
		flag = false;
	}



	_cadFamilyInfos.clear();

	auto cadFamilyInfos = root["CadFamilyInfos"].toArray();
	for (int i = 0; i < cadFamilyInfos.size(); i++) {
		auto cadFamilyInfo = cadFamilyInfos[i].toObject();
		CadFamilyInfo cFamilyInfo;

		// Basic Info
		cFamilyInfo.familyId = jsonHelper::getString(cadFamilyInfo, "familyId", "Unassigned");
		cFamilyInfo.familyName = jsonHelper::getString(cadFamilyInfo, "familyName", "Unassigned");
		cFamilyInfo.color = jsonHelper::getString(cadFamilyInfo, "color");
		cFamilyInfo.isCuPillar = jsonHelper::getBool(cadFamilyInfo, "cuPillar", false);
		cFamilyInfo.isForcePass = jsonHelper::getBool(cadFamilyInfo, "isForcePass", false);
		cFamilyInfo.ranking = jsonHelper::getInteger(cadFamilyInfo, "ranking", _cadFamilyInfos.size() + 1);

		// 3D Setting Parameters
		cFamilyInfo.upperHeightRange = jsonHelper::getInteger(cadFamilyInfo, "upperHeightRange", 0);
		cFamilyInfo.lowerHeightRange = jsonHelper::getInteger(cadFamilyInfo, "lowerHeightRange", 0);
		cFamilyInfo.cornerSizePercentage = jsonHelper::getInteger(cadFamilyInfo, "cornerSizePercentage", 0);
		cFamilyInfo.horizontalCornerShift_px = jsonHelper::getInteger(cadFamilyInfo, "horizontalCornerShift_px", 0);
		cFamilyInfo.verticalCornerShift_px = jsonHelper::getInteger(cadFamilyInfo, "verticalCornerShift_px", 0);
		cFamilyInfo.registrationMethod = jsonHelper::getString(cadFamilyInfo, "registrationMethod");
		cFamilyInfo.measurememntMethod = jsonHelper::getString(cadFamilyInfo, "measurememntMethod");
		cFamilyInfo.planeFittingPercentage = jsonHelper::getInteger(cadFamilyInfo, "planeFittingPercentage", 0);
		cFamilyInfo.bias = jsonHelper::getInteger(cadFamilyInfo, "bias", 0);

		// 3D Criteria
		cFamilyInfo.checkHeight = jsonHelper::getBool(cadFamilyInfo, "checkHeight", false);
		cFamilyInfo.checkTilt = jsonHelper::getBool(cadFamilyInfo, "checkTilt", false);
		cFamilyInfo.checkVolume = jsonHelper::getBool(cadFamilyInfo, "checkVolume", false);
		cFamilyInfo.upperAllowHeight = jsonHelper::getInteger(cadFamilyInfo, "upperAllowHeight", 0);
		cFamilyInfo.lowerAllowHeight = jsonHelper::getInteger(cadFamilyInfo, "lowerAllowHeight", 0);
		cFamilyInfo.maxAllowTilt = jsonHelper::getDouble(cadFamilyInfo, "maxAllowTilt", 0);
		cFamilyInfo.nominalVolume = jsonHelper::getInteger(cadFamilyInfo, "nominalVolume", 0);
		cFamilyInfo.lowerVolumePercentage = jsonHelper::getInteger(cadFamilyInfo, "lowerVolumePercentage", 0);
		cFamilyInfo.upperVolumePercentage = jsonHelper::getInteger(cadFamilyInfo, "upperVolumePercentage", 0);

		_cadFamilyInfos.append(cFamilyInfo);
	}



	qDebug() << "Load cad family Json end";
	return flag;
}

void InspectionThread::insertDefectTagNames(QStringList& defectTagNames, const QStringList& tagNames)
{
	for (int i = 0; i < tagNames.size(); i++)
	{
		if (!defectTagNames.contains(tagNames[i]))
		{
			defectTagNames.append(tagNames[i]);
		}
	}

	return;
}

void InspectionThread::saveAllVisionObjectImages(const QHash<QString, QRectF>& visionObjectsRect, const QHash<QString, QHash<QString, QPointF>>& locatorOffsets, QVector<FrameInfo>& pFrameInfos, QVector<MIL_ID>& milImgs, MIL_ID& mil_IMap)
{
	// can use to debug if VO cropped image is correct by turning off locator offset

	QHash<QString, QRectF>::const_iterator vor = visionObjectsRect.constBegin();
	while (vor != visionObjectsRect.constEnd())
	{
		QRect voRect = vor.value().toRect();

		//get first value of loc offset for all locator hashMap
		QHash<QString, QPointF> offsets;
		QHash<QString, QHash<QString, QPointF>>::const_iterator lo = locatorOffsets.constBegin();
		while (lo != locatorOffsets.constEnd())
		{
			if (lo.value().size() > 0) offsets.insert(lo.key(), lo.value().begin().value());
			++lo;
		}
		offsetVisionObject(offsets, vor.key(), voRect);

		for (int i = 0; i < pFrameInfos.size(); i++)
		{
			auto frameInfo = pFrameInfos[i];
			QString opticName = _recipeOptics[frameInfo.opticID].name;
			QString imgPath = Common::Directory::getProductionDefectPath() + vor.key() + "_" + opticName;
			QString imgType = frameInfo.type;
			MIL_ID croppedMilImg = mtrx::crop(milImgs[i], voRect.x(), voRect.y(), voRect.width(), voRect.height());
			//SEETHIS:

			auto sharedCrop = mtrx::MPM::instance().attach(croppedMilImg);

			if (imgType == ct::s_height_map)
			{
			}
			else
			{
				if (_saveDefectVoImg) {
					QString extension = ".jpg";
					auto filename = QString(imgPath + extension);
					IST::instance().enqueue(filename.toStdString(), sharedCrop, _defectCollectorPath.toStdString());
				}
			}
		}

		//save intensityMap
		if (mil_IMap && _saveDefectVoImg)
		{//SEETHIS:
			QString imgPath = Common::Directory::getProductionDefectPath() + vor.key() + "_IntensityMap.jpg";
			MIL_ID croppedMilImg = mtrx::crop(mil_IMap, voRect.x(), voRect.y(), voRect.width(), voRect.height());
			auto sharedCrop = mtrx::MPM::instance().attach(croppedMilImg);
			IST::instance().enqueue(imgPath.toStdString(), sharedCrop, _defectCollectorPath.toStdString());
		}

		vor++;
	}
}

void InspectionThread::classifyDefectImages()
{
	for (int i = 0; i < _dragROI.size(); i++)
	{
		bool pass = true;
		QStringList voTagNames;
		for (int j = 0; j < _defectResults.size(); j++)
		{
			if (_defectResults[j].algoDefResult.vo_name == _dragROI[i]->getName().toStdString())
			{
				pass = false;

				if (_defectResults[j].algoDefResult.enableCustomTagName) //custom TagName
				{
					_defectResults[j].tagNames.append(_defectResults[j].algoDefResult.customTagName);
				}
				_defectResults[j].tagNames.removeDuplicates();
				arrangeTagNameBasedOnPriority(_defectResults[j].tagNames);
				voTagNames.append(_defectResults[j].tagNames);
			}
		}

		voTagNames.removeDuplicates();
		if (!pass) arrangeTagNameBasedOnPriority(voTagNames);

		QString s = _dragROI[i]->getName();
		if (_saveDefectVoImg) generateDefectImages(voTagNames, s, _allOptics_2D_3D);
	}
}

void InspectionThread::generateDefectImages(QStringList& defectTagNames, QString& voName, QVector<OpticsInfo>& opticsInfos)
{
	//for (int i = 0; i < opticsInfos.size(); i++)
	//{
	//	qDebug() << "name:" << opticsInfos[i].name << " type:" << opticsInfos[i].type;
	//}

	QString defectTagName = "Good";
	if (defectTagNames.size() > 0) defectTagName = defectTagNames[0];

	//defectFolder
	QString defectFolderPath = Common::Directory::getProductionDefectPath() + defectTagName + "//";
	CreateDirectoryA(defectFolderPath.toStdString().c_str(), NULL);

	//defectRectFolder
	QString defectRectFolderPath = Common::Directory::getProductionDefectPath() + defectTagName + "_defRect\\";
	if (defectTagName != "Good") CreateDirectoryA(defectRectFolderPath.toStdString().c_str(), NULL);

	QVector<QRect> defRects;
	for (int j = 0; j < _defectResults.size(); j++)
	{
		if (_defectResults[j].algoDefResult.vo_name == voName.toStdString())
		{
			auto x = _defectResults[j].algoDefResult.def_x - _defectResults[j].algoDefResult.loc_offset_x;
			auto y = _defectResults[j].algoDefResult.def_y - _defectResults[j].algoDefResult.loc_offset_y;
			auto w = _defectResults[j].algoDefResult.def_w;
			auto h = _defectResults[j].algoDefResult.def_h;
			defRects.append(QRect(x, y, w, h));
		}
	}

	for (int i = 0; i < opticsInfos.size(); i++)
	{
		bool saveDefectRect = true;
		QString extension;
		if (opticsInfos[i].type == ct::s_color || opticsInfos[i].type == ct::s_mono || opticsInfos[i].type == ct::s_intensity_map) extension = ".jpg";
		else
		{
			extension = ".tiff";
			saveDefectRect = false;
		}

		QString imgPath = Common::Directory::getProductionDefectPath() + voName + "_" + opticsInfos[i].name + extension;
		QString destinationImgPath = defectFolderPath + voName + "_" + opticsInfos[i].name + extension;

		if (defectTagName == "Good")
		{
			if (false) DeleteFileA(imgPath.toStdString().c_str());
			else MoveFileA(imgPath.toStdString().c_str(), destinationImgPath.toStdString().c_str());
		}
		else if (saveDefectRect)
		{
			MoveFileA(imgPath.toStdString().c_str(), destinationImgPath.toStdString().c_str());
		}
	}
}

void InspectionThread::changeSelectedTagNametoUnknown(QStringList& tagNames, const QString& selectedTagName)
{
	for (int i = 0; i < tagNames.size(); i++)
	{
		if (tagNames[i] == selectedTagName) tagNames[i] = "Unknown";
	}
}

void InspectionThread::arrangeTagNameBasedOnPriority(QStringList& tagNames)
{
	QStringList reorderedTagNames;


	for (int i = 0; i < _defectPriorityList.size(); i++)
	{
		for (int j = 0; j < tagNames.size(); j++)
		{

			if (tagNames[j] == _defectPriorityList[i]) reorderedTagNames.append(tagNames[j]);
		}
	}

	tagNames.clear();
	if (reorderedTagNames.size() == 0) reorderedTagNames.append("Unknown");
	tagNames = reorderedTagNames;


}

void InspectionThread::getVidiTools(QVector<ToolInfo>& toolInfos, const QString& streamName)
{
#if HAS_VIDI_LICENSE
	bool flag = true;
	VIDI_BUFFER buffer;
	_status = vidi_init_buffer(&buffer);
	if (_status != VIDI_SUCCESS)
	{
		_errorMsg = QStringLiteral("Fail to initialise inspection result buffer");
		flag = false;
	}

	vidi_runtime_list_tools(_workspaceName.toStdString().c_str(), streamName.toStdString().c_str(), &buffer);
	QByteArray bufferData = QByteArray(buffer.data);
	QXmlStreamReader xmlReader(bufferData);

	ToolInfo toolInfo;
	while (!xmlReader.atEnd() && !xmlReader.hasError())
	{
		QXmlStreamReader::TokenType token = xmlReader.readNext();

		if (token == QXmlStreamReader::StartElement && xmlReader.name() == "tool")
		{
			toolInfo.toolId = xmlReader.attributes().value("id").toString();
			toolInfo.toolType = xmlReader.attributes().value("type").toString();
			toolInfo.toolUuid = xmlReader.attributes().value("uuid").toString();
		}
		if (token == QXmlStreamReader::EndElement && xmlReader.name() == "tool")
		{
			if (!toolInfo.exist(toolInfos, toolInfo)) toolInfos.append(toolInfo);
		}
	}

	/*for (int i = 0; i < toolInfos.size(); i++)
	{
		qDebug() << "Tool ID:" << toolInfos[i].toolId;
		qDebug() << "Tool Type:" << toolInfos[i].toolType;
		qDebug() << "Tool UUID:" << toolInfos[i].toolUuid;
	}*/
	if (xmlReader.hasError())
	{
		// Handle XML parsing error
		qDebug() << "XML Parsing Error:" << xmlReader.errorString();
	}

	vidi_free_buffer(&buffer);
#endif
}

void InspectionThread::getVidiLabels(QVector<ToolInfo>& toolInfos, const QString& streamName)
{
#if HAS_VIDI_LICENSE
	bool flag = true;
	for (int i = 0; i < toolInfos.size(); i++)
	{
		VIDI_BUFFER buffer;
		_status = vidi_init_buffer(&buffer);
		if (_status != VIDI_SUCCESS)
		{
			_errorMsg = QStringLiteral("Fail to initialise inspection result buffer");
			flag = false;
		}

		vidi_runtime_tool_get_info(_workspaceName.toStdString().c_str(), streamName.toStdString().c_str(), toolInfos[i].toolId.toStdString().c_str(), &buffer);
		QByteArray bufferData = QByteArray(buffer.data);
		QXmlStreamReader xmlReader(bufferData);

		while (!xmlReader.atEnd() && !xmlReader.hasError())
		{
			QXmlStreamReader::TokenType token = xmlReader.readNext();

			if (token == QXmlStreamReader::StartElement && xmlReader.name() == "tool_info")
			{
				QString toolType = xmlReader.attributes().value("type").toString();

				if (toolType == "red")
				{
					// Parse known_classes for red tool
					while (!xmlReader.atEnd() && !xmlReader.hasError())
					{
						token = xmlReader.readNext();

						if (token == QXmlStreamReader::StartElement && xmlReader.name() == "class")
						{
							QString className = xmlReader.attributes().value("name").toString();
							toolInfos[i].labels.append(className);
						}
						else if (token == QXmlStreamReader::EndElement && xmlReader.name() == "known_classes")
						{
							// End of known_classes, exit the loop
							break;
						}
					}
				}
				else if (toolType == "blue")
				{
					// Parse known_tags for green tool
					while (!xmlReader.atEnd() && !xmlReader.hasError())
					{
						token = xmlReader.readNext();

						if (token == QXmlStreamReader::StartElement && xmlReader.name() == "feature")
						{
							QString tagName = xmlReader.attributes().value("name").toString();
							toolInfos[i].labels.append(tagName);
						}
						else if (token == QXmlStreamReader::EndElement && xmlReader.name() == "known_features")
						{
							// End of known_tags, exit the loop
							break;
						}
					}
				}
				else if (toolType == "green")
				{
					// Parse known_tags for green tool
					while (!xmlReader.atEnd() && !xmlReader.hasError())
					{
						token = xmlReader.readNext();

						if (token == QXmlStreamReader::StartElement && xmlReader.name() == "tag")
						{
							QString tagName = xmlReader.attributes().value("name").toString();
							toolInfos[i].labels.append(tagName);
						}
						else if (token == QXmlStreamReader::EndElement && xmlReader.name() == "known_tags")
						{
							// End of known_tags, exit the loop
							break;
						}
					}
				}
			}
		}
		vidi_free_buffer(&buffer);
	}

	/*for (int i = 0; i < toolInfos.size(); i++)
	{
		qDebug() << "Tool ID:" << toolInfos[i].toolId;
		qDebug() << "Tool Type:" << toolInfos[i].toolType;
		qDebug() << "Tool UUID:" << toolInfos[i].toolUuid;
		qDebug() << "Tool labels:" << toolInfos[i].labels;
	}*/
#endif
}

void InspectionThread::getVidiToolParameters(QVector<ToolInfo>& toolInfos)
{
#if HAS_VIDI_LICENSE
	bool flag = true;
	for (int i = 0; i < toolInfos.size(); i++)
	{
		VIDI_BUFFER buffer;
		_status = vidi_init_buffer(&buffer);
		if (_status != VIDI_SUCCESS)
		{
			_errorMsg = QStringLiteral("Fail to initialise inspection result buffer");
			flag = false;
		}

		vidi_runtime_tool_list_parameters(_workspaceName.toStdString().c_str(), "default", toolInfos[i].toolId.toStdString().c_str(), &buffer);
		QByteArray bufferData = QByteArray(buffer.data);

		qDebug() << bufferData.toStdString().c_str();
		vidi_free_buffer(&buffer);
	}
#endif
}

bool InspectionThread::serializeToolIInfos(QString fileName, QVector<ToolInfo>& toolInfos)
{
	QJsonObject tools;
	QJsonArray toolInfoArray;
	for (int i = 0; i < toolInfos.size(); i++)
	{
		QJsonObject toolInfoObj;
		toolInfoObj.insert("id", toolInfos[i].toolId);
		toolInfoObj.insert("type", toolInfos[i].toolType);
		toolInfoObj.insert("uuid", toolInfos[i].toolUuid);

		QJsonArray labelArray;
		for (int j = 0; j < toolInfos[i].labels.size(); j++)
		{
			labelArray.push_back(toolInfos[i].labels[j]);
		}

		toolInfoObj.insert("labels", labelArray);

		toolInfoArray.push_back(toolInfoObj);
	}

	tools.insert("ToolInfos", toolInfoArray);

	QJsonDocument doc(tools);
	bool flag = false;

	QFile file(fileName);
	if (file.open(QIODevice::WriteOnly))
	{
		file.write(doc.toJson());
		file.flush();
		file.close();

		flag = true;
	}

	return flag;
}

bool InspectionThread::deserializeToolInfos(QString fileName, QVector<ToolInfo>& toolInfos)
{
	QString val;
	QFile file;

	file.setFileName(fileName);

	if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
	{
		return false;
	}

	val = file.readAll();
	file.close();

	QJsonDocument doc = QJsonDocument::fromJson(val.toUtf8());
	QJsonObject root = doc.object();

	QJsonArray toolInfosArray = jsonHelper::getArray(root, QStringLiteral("ToolInfos"));
	for (int i = 0; i < toolInfosArray.size(); i++)
	{
		auto jsonObj = toolInfosArray[i].toObject();
		ToolInfo t;
		t.toolId = jsonHelper::getString(jsonObj, QStringLiteral("id")).toStdString().c_str();
		t.toolType = jsonHelper::getString(jsonObj, QStringLiteral("type")).toStdString().c_str();
		t.toolUuid = jsonHelper::getString(jsonObj, QStringLiteral("uuid")).toStdString().c_str();

		QJsonArray labelsArray = jsonHelper::getArray(jsonObj, QStringLiteral("labels"));
		for (int j = 0; j < labelsArray.size(); j++)
		{
			t.labels.append(labelsArray[i].toString());
		}

		toolInfos.append(t);
	}
	return true;
}

bool InspectionThread::vidiInspectionFromFile()
{
#if HAS_VIDI_LICENSE
	QString imagePath = QString::fromStdString(charsToStr(_pInspectionInfo->_imagePath));
	QString workspaceName = _workspaceOpened;
	VIDI_IMAGE image;
	_status = vidi_init_image(&image);
	if (_status != VIDI_SUCCESS)
	{
		vidi_get_error_message(_status, &_buffer);
		_errorMsg = QStringLiteral("Fail to initialise inspection image, error message: %1").arg(QString(_buffer.data));
		return false;
	}

	_status = vidi_load_image(imagePath.toStdString().c_str(), &image);
	if (_status != VIDI_SUCCESS)
	{
		vidi_get_error_message(_status, &_buffer);
		_errorMsg = QStringLiteral("Fail to load inspection image, error message: %1").arg(QString(_buffer.data));

		vidi_free_image(&image);
		return false;
	}

	VIDI_BUFFER result_buffer;
	_status = vidi_init_buffer(&result_buffer);
	if (_status != VIDI_SUCCESS)
	{
		_errorMsg = QStringLiteral("Fail to initialise inspection result buffer");

		vidi_free_image(&image);
		return false;
	}

	// as of ViDi Suite 3.0.0, samples are processed in a few steps instead of just calling vidi_runtime_process
	// the first step is to initialize the sample
	_status = vidi_runtime_create_sample(workspaceName.toStdString().c_str(), "default", "my_sample");
	if (_status != VIDI_SUCCESS)
	{
		_errorMsg = QStringLiteral("Fail to initialise inspection sample");

		vidi_free_buffer(&result_buffer);
		vidi_free_image(&image);
		return false;
	}

	// then add the image to be processed
	_status = vidi_runtime_sample_add_image(workspaceName.toStdString().c_str(), "default", "my_sample", &image);
	if (_status != VIDI_SUCCESS)
	{
		_errorMsg = QStringLiteral("Fail to add inspection image");

		vidi_free_buffer(&result_buffer);
		vidi_free_image(&image);
		vidi_runtime_free_sample(workspaceName.toStdString().c_str(), "default", "my_sample");

		return false;
	}

	_status = vidi_runtime_sample_process(workspaceName.toStdString().c_str(), "default", "", "my_sample", "");
	if (_status != VIDI_SUCCESS)
	{
		vidi_get_error_message(_status, &_buffer);
		_errorMsg = QStringLiteral("Fail to process inspection sample, error message: %1").arg(QString(_buffer.data));

		vidi_free_buffer(&result_buffer);
		vidi_free_image(&image);
		vidi_runtime_free_sample(workspaceName.toStdString().c_str(), "default", "my_sample");
		return false;
	}

	// the next step is to get the results
	_status = vidi_runtime_get_sample_json(workspaceName.toStdString().c_str(), "default", "my_sample", &result_buffer);
	if (_status != VIDI_SUCCESS)
	{
		_errorMsg = QStringLiteral("Fail to get inspection result");

		vidi_free_buffer(&result_buffer);
		vidi_free_image(&image);
		vidi_runtime_free_sample(workspaceName.toStdString().c_str(), "default", "my_sample");
		return false;
	}

	// New Update - 27/2/2023
	QByteArray resultBufferData = QByteArray(result_buffer.data);
	QString jsonPath = Common::Directory::CachePath + "VidiResult.json";
	VidiToolResult vidiResult;
	QString heatMapUUID;
	QString scoreMapUUID;

	vidiResult.attachResult(resultBufferData);
	for (int i = 0; i < vidiResult._redToolList.size(); i++)
	{
		heatMapUUID = vidiResult._redToolList[i].heatMapUUID;
		scoreMapUUID = vidiResult._redToolList[i].scoreMapUUID;
	}

	QString heatmapPath = Common::Directory::CachePath + "heatmap.jpg";

	getRedToolHeatMap(heatMapUUID, heatmapPath);

	std::ofstream ofs(jsonPath.toStdString());
	ofs << resultBufferData.data() << std::endl;
	ofs.close();
	_errorMsg = "Succesfully opened!";


	// free resources
	_status = vidi_free_buffer(&result_buffer);
	if (_status != VIDI_SUCCESS)
	{
		_errorMsg = QStringLiteral("Fail to free inspection result buffer");
		return false;
	}

	_status = vidi_free_image(&image);
	if (_status != VIDI_SUCCESS)
	{
		_errorMsg = QStringLiteral("Fail to free inspection image");
		vidi_runtime_free_sample(_workspaceName.toStdString().c_str(), "default", "my_sample");
		return false;
	}

	_status = vidi_runtime_free_sample(_workspaceName.toStdString().c_str(), "default", "my_sample");
	if (_status != VIDI_SUCCESS)
	{
		_errorMsg = QStringLiteral("Fail to free inspection sample");
		return false;
	}
#endif

	return true;
}

bool InspectionThread::vidiInspectionFromFiles(QVector<QStringList> imgPreprocessPaths, QStringList lightingIDs)
{
	if (imgPreprocessPaths.size() != lightingIDs.size()) return false;

	QJsonObject res;
	QJsonArray streamVidiResults;

#if HAS_VIDI_LICENSE
	for (int i = 0; i < imgPreprocessPaths.size(); i++)
	{
		for (int j = 0; j < imgPreprocessPaths[i].size(); j++)
		{
			auto imagePath = imgPreprocessPaths[i][j];

			//get preprocess name
			QFileInfo fileInfo(imagePath);
			QStringList imgInfos = fileInfo.completeBaseName().split("_");
			QString preprocessName = QString();
			if (imgInfos.size() > 2) preprocessName = imgInfos[2];


			QString opticName = _recipeOptics[lightingIDs[i]].name;
			if (!preprocessName.isEmpty()) opticName = opticName + "_" + preprocessName;
			if (checkVidiStreamExist(opticName))
			{
				//init image
				VIDI_IMAGE image;
				_status = vidi_init_image(&image);
				if (_status != VIDI_SUCCESS)
				{
					vidi_get_error_message(_status, &_buffer);
					_errorMsg = QStringLiteral("Fail to initialise inspection image, error message: %1").arg(QString(_buffer.data));
					return false;
				}

				//load image
				_status = vidi_load_image(imagePath.toStdString().c_str(), &image);
				if (_status != VIDI_SUCCESS)
				{
					vidi_get_error_message(_status, &_buffer);
					_errorMsg = QStringLiteral("Fail to load inspection image, error message: %1").arg(QString(_buffer.data));

					vidi_free_image(&image);
					return false;
				}

				//inspect image
				QJsonObject vidiResult;
				Timer time;
				vidiInspection(&image, "", false, vidiResult, lightingIDs[i], opticName);
				qDebug() << "VidiInspectionFromFileDuration:" << time.duration();
				streamVidiResults.push_back(vidiResult);

				//free image
				_status = vidi_free_image(&image);
				if (_status != VIDI_SUCCESS)
				{
					_errorMsg = QStringLiteral("Fail to free inspection image");
					return false;
				}
			}
		}
	}
#endif

	res.insert("vidiResults", streamVidiResults);
	QString jsonPath = Common::Directory::CachePath + "VidiResult.json";
	QJsonDocument doc(res);
	bool flag = false;

	QFile file(jsonPath);
	if (file.open(QIODevice::WriteOnly))
	{
		file.write(doc.toJson());
		file.flush();
		file.close();

		flag = true;
	}
	qDebug() << "vidiResultSuccessfullyGenerated:" << flag;
	return flag;
}

bool InspectionThread::preprocessFromFiles(QStringList imgPaths, QStringList lightingIDs, QVector<QStringList>& imgPreprocessPaths)
{
	if (imgPaths.size() != lightingIDs.size()) return false;

	//temp hardcoded
	QHash<QString, util::ImagePreprocess*> imagePreprocessTools;
	auto optic = _recipeOptics.constBegin();
	while (optic != _recipeOptics.constEnd())
	{
		QString refImgPath = Common::Directory::getRecipeVidiImagePath() + "ref_" + optic.value().name + g_imgExtension;
		QString maskImgPath = Common::Directory::getRecipeVidiImagePath() + "mask" + g_imgExtension;
		util::ImagePreprocess* imagePreprocess;
		imagePreprocess = new util::ImagePreprocess(refImgPath, maskImgPath);
		imagePreprocessTools.insert(optic.value().name, imagePreprocess);
		optic++;
	}
	//temp hardcoded - end

	for (int i = 0; i < imgPaths.size(); i++)
	{
		QStringList imgPreprocessPath;
		auto imagePath = imgPaths[i];
		QString opticName = _recipeOptics[lightingIDs[i]].name;


		MIL_INT sizeX, sizeY, bandSize;
		MbufDiskInquireA(imagePath.toStdString().c_str(), M_SIZE_X, &sizeX);
		MbufDiskInquireA(imagePath.toStdString().c_str(), M_SIZE_Y, &sizeY);
		MbufDiskInquireA(imagePath.toStdString().c_str(), M_SIZE_BAND, &bandSize);

		MIL_ID img = M_NULL;
		MbufAllocColor(M_DEFAULT, bandSize, sizeX, sizeY, 8 + M_UNSIGNED, M_IMAGE + M_PROC, &img);
		MIL_INT imgType = M_JPEG_LOSSY;
		if (util::isPNG(imagePath)) imgType = M_PNG;
		if (util::isBMP(imagePath)) imgType = M_BMP;
		MbufImportA(imagePath.toStdString().c_str(), imgType, M_LOAD, M_DEFAULT_HOST, &img);

		if (img)
		{
			auto imagePreprocess = imagePreprocessTools[opticName];
			if (opticName == "RB")
			{
				imgPreprocessPath.append(imagePath);
				MbufExportA(imagePath.toStdString().c_str(), g_imgType, img);

				Timer time;
				QFileInfo fileInfo(imagePath);
				QString fullPathWithoutExtension = fileInfo.absolutePath() + "/" + fileInfo.completeBaseName() + "_DiffOfMedianFilter" + g_imgExtension;
				imgPreprocessPath.append(fullPathWithoutExtension);
				imagePreprocess->Diff_of_medianFilter(img, img);
				qDebug() << "Diff_of_medianFilter:" << time.duration();
				MbufExportA(fullPathWithoutExtension.toStdString().c_str(), g_imgType, img);
			}
			else if (opticName == "RGB")
			{
				imgPreprocessPath.append(imagePath);
				MbufExportA(imagePath.toStdString().c_str(), g_imgType, img);

				//highlightDefects (FM on background)
				QFileInfo fileInfo(imagePath);
				QString fullPathWithoutExtension = fileInfo.absolutePath() + "/" + fileInfo.completeBaseName() + "_HighlightDefects" + g_imgExtension;
				imgPreprocessPath.append(fullPathWithoutExtension);
				Timer time;
				imagePreprocess->HighlightDarkDefects(img, img);
				qDebug() << "HighlightDarkDefects:" << time.duration();
				MbufExportA(fullPathWithoutExtension.toStdString().c_str(), g_imgType, img);
			}
			else if (opticName == "DieLight")
			{
				Timer time;
				imagePreprocess->medianFilter(img, img);
				qDebug() << "medianFilter:" << time.duration();
				imgPreprocessPath.append(imagePath);
				MbufExportA(imagePath.toStdString().c_str(), g_imgType, img);

				//diff of median filter (Die Chip)
				QFileInfo fileInfo(imagePath);
				QString fullPathWithoutExtension = fileInfo.absolutePath() + "/" + fileInfo.completeBaseName() + "_DiffOfMedianFilter" + g_imgExtension;
				imgPreprocessPath.append(fullPathWithoutExtension);
				imagePreprocess->Diff_of_medianFilter(img, img, 8, 255, 0, 3, 15);
				MbufExportA(fullPathWithoutExtension.toStdString().c_str(), g_imgType, img);
			}
		}
		imgPreprocessPaths.append(imgPreprocessPath);

		MbufFree(img);
	}

	//temp hardcoded - end
	auto imageP = imagePreprocessTools.constBegin();
	while (imageP != imagePreprocessTools.constEnd())
	{
		delete imageP.value();
		imageP++;
	}
	//temp hardcoded - end
}

void InspectionThread::strToChars(char* pCharArr, int arrSize, std::string src)
{
	std::string str = src + "~";
	strcpy_s(pCharArr, arrSize, str.c_str());
}

std::string InspectionThread::charsToStr(char* pCharArray)
{
	std::string str;
	str.clear();

	for (int i = 0;; i++)
	{
		if (pCharArray[i] != '~')
		{
			str = str + pCharArray[i];
		}
		else
		{
			break;
		}
	}

	return str;
}

void InspectionThread::loadObjectDetectionModelSettings(QString templateODOpticsPath)
{
	auto jsonPath = Common::Directory::getRecipeObjectDetectionModelSettingsPath() + templateODOpticsPath;

	QJsonObject root;
	if (loadJson(jsonPath, root)) {

		_odModelsOptics.clear();
		auto odModelsArray = jsonHelper::getArray(root, QStringLiteral("OD_model_list"));
		for (int i = 0; i < odModelsArray.size(); i++)
		{
			auto model = odModelsArray[i].toObject();
			auto modelName = jsonHelper::getString(model, "model_name");
			auto modelEnabled = jsonHelper::getBool(model, "enable");

			if (!modelEnabled) continue;

			auto opticArray = jsonHelper::getArray(model, QStringLiteral("optics"));
			QVector<AiModelInfo::opticInfo> opticList;
			for (int j = 0; j < opticArray.size(); j++)
			{
				AiModelInfo::opticInfo opticInfo;
				QJsonObject opticObj = opticArray[j].toObject();
				opticInfo.enableSegmentation = jsonHelper::getBool(opticObj, "enableSegmentation");
				opticInfo.opticId = jsonHelper::getString(opticObj, "opticId");
				opticInfo.opticName = jsonHelper::getString(opticObj, "opticName");
				opticInfo.channel = jsonHelper::getString(opticObj, "channel");
				opticInfo.imageRotation = jsonHelper::getString(opticObj, "imageRotation");
				opticInfo.locatorId = jsonHelper::getString(opticObj, "LocatorID");
				opticInfo.algoType = jsonHelper::getString(opticObj, "algoType");
				opticList.append(opticInfo);
			}

			_odModelsOptics.insert(modelName, opticList);
		}
	}
}

bool InspectionThread::objectDetectionInspectionFromFiles(QStringList imgPaths, QStringList lightingIDs, QString templateODOpticsPath)
{
	qDebug() << "objectDetectionInspectionFromFiles";

	loadObjectDetectionModelSettings(templateODOpticsPath);

	QStringList lightingNames;
	for (int i = 0; i < lightingIDs.size(); i++)
	{
		lightingNames.append(_recipeOptics[lightingIDs[i]].name);
	}

	QVector<AiModelInfo::opticInfo> odOpticsList;
	QStringList modelList;
	QVector<std::vector<OnnxResult>> odResultList;

	for (int i = 0; i < g_ODModels.size(); i++)
	{
		for (auto it = _odModelsOptics.constBegin(); it != _odModelsOptics.constEnd(); ++it)
		{
			QString modelName = it.key();
			auto opticList = it.value();

			if (modelName == g_ODModels[i]->getModelID().c_str())
			{
				if (imgPaths.size() == lightingIDs.size())
				{
					for (int j = 0; j < imgPaths.size(); j++)
					{
						for (auto& optic : opticList)
						{

							if (optic.opticId == lightingIDs[j])
							{

								cv::Mat img = cv::imread(imgPaths[j].toStdString());

								// do rotation
								if (optic.imageRotation == "90") cv::rotate(img, img, cv::ROTATE_90_CLOCKWISE);
								else if (optic.imageRotation == "180") cv::rotate(img, img, cv::ROTATE_180);
								else if (optic.imageRotation == "270") cv::rotate(img, img, cv::ROTATE_90_COUNTERCLOCKWISE);

								getImageChannel(optic.channel, img, img);



								// when running from algo Editor
								std::vector<OnnxResult>od_result;
								if (g_odTilingSettings.enableTiling)
								{

									ct::logger::info("Running OD Model With Tiling: %s ", g_ODModels[i]->getModelID().c_str());
									od_result = g_ODModels[i]->ct_runModel_tiling(img, 0.1, 0.5, g_odTilingSettings.tilingSize, g_odTilingSettings.tilingPaddingPerc, g_odTilingSettings.tilingIou);
									ct::logger::info("Finished running OD Model Tiling: %s ", g_ODModels[i]->getModelID().c_str());
								}
								else
								{
									ct::logger::info("Running OD Model: %s ", g_ODModels[i]->getModelID().c_str());
									od_result = g_ODModels[i]->ct_runModel(img, 0.1, 0.5);
									ct::logger::info("Finished running OD Model: %s ", g_ODModels[i]->getModelID().c_str());
								}
								cv::cvtColor(img, img, cv::COLOR_RGB2BGR);

								// kernel size must be ODD numbers
								cv::GaussianBlur(
									img,        // src
									img,        // dst (in-place is OK)
									cv::Size(5, 5),  // kernel size
									0           // sigmaX = 0 → auto-compute
								);

								/*std::vector<cv::Mat> channels;
								cv::split(img, channels);
								std::unordered_map<std::string, cv::Mat> segImgs;
								cv::Mat green = channels[1];
								cv::Mat green3;
								cv::merge(std::vector<cv::Mat>{ green, green, green }, green3);
								cv::GaussianBlur(green3, green3, cv::Size(5, 5), 1.2);*/
								/*cv::imwrite("channel_B.png", channels[0]);
								cv::imwrite("channel_G.png", channels[1]);
								cv::imwrite("channel_R.png", channels[2]);*/
								// === segmentation ===
								if (optic.enableSegmentation && g_segModel)
								{
									qDebug() << "@@@@@@@@@@@@ SEGMENTATION ENABLED @@@@@@@@@@@@";
									std::unordered_map<std::string, cv::Mat> segImgs;
									segImgs["Img"] = img;
									//segImgs["Img"] = green3;
									//cv::imwrite("channel_low_contrast.png", green_low_contrast);

									std::unordered_map<std::string, std::list<cv::Rect>> segRectMap;
									std::list<cv::Rect> segRects;
									std::vector<std::string> segRectScore;

									std::unordered_map<std::string, std::list<cv::Point>> segPointMap;
									
									for (auto& r : od_result)
									{

										if (r.accuracy < 0.5) continue;
										cv::Rect  rect = cv::Rect(r.x1, r.y1, r.x2 - r.x1, r.y2 - r.y1);

										// --- Shrink height ---
										float hScale = 0.8f;
										int newHeight = static_cast<int>(rect.height * hScale);
										int offsetY = (rect.height - newHeight) / 2;

										// --- Extend width ---
										float wScale = 1.05f;
										int newWidth = static_cast<int>(rect.width * wScale);
										int offsetX = (rect.width - newWidth) / 2;

										// --- Final rect ---
										cv::Rect finalRect(
											rect.x + offsetX,
											rect.y + offsetY,
											newWidth,
											newHeight
										);

										segRects.push_back(finalRect);
										segRectMap["Img"] = segRects;

										segRectScore.push_back(QString::number(r.accuracy).toStdString());

									
										// ========================= segPoints ==========================
									
									}

									//std::list<cv::Point> segPoints;
									//for (const auto& r : od_result) {
									//	if (r.accuracy < 0.6) continue;

									//	// Fixed mid-Y scanline, X goes from left -> right
									//	int interval = 4;           // e.g. 5, 10, 20 (must be >= 1)
									//	interval = std::max(1, interval);

									//	int xL = std::min(r.x1, r.x2);
									//	int xR = std::max(r.x1, r.x2);
									//	int yT = std::min(r.y1, r.y2);
									//	int yB = std::max(r.y1, r.y2);

									//	int yMid = yT + (yB - yT) / 2;         // same as (y1 + y2) / 2, but avoids overflow

									//	for (int i = 1; i < 3; i++)
									//	{
									//		int width = (xR - xL) * (i + 1) / 4;
									//		segPoints.emplace_back(xL + width, yMid);
									//	}

									//}

									//segPointMap["Img"] = segPoints;

									ct::logger::info("SegmentationScore:%.5f", _segmentationScore);
									g_segModel->runModel_segmentation(segImgs, segRectMap, _segmentationScore, true);
									//g_segModel->runModel_segmentation(segImgs, segPointMap, 2.0, true);
									
									//saveSegRectMapToJson(segRectMap, "AlgoEditorODResult.json");

									std::unordered_map<std::string, cv::Mat> segMaskMap;
									segMaskMap = g_segModel->getSegmentationMaskResult();

									QString odSegmentationFolder = Common::Directory::CachePath + "OdSegmentationMask/";
									CreateDirectoryA(odSegmentationFolder.toStdString().c_str(), NULL);
									for (auto& i : segMaskMap)
									{
										QString maskImgPath = odSegmentationFolder + optic.opticId + optic.channel + ".jpg";

										MIL_ID milImg = M_NULL;
										util::cv_to_Mil(i.second, milImg);
										MbufExportA(maskImgPath.toStdString().c_str(), M_JPEG_LOSSY, milImg);
										mtrx::free_buffer(milImg);
									}
									// ==
									
									QString odSegmentationFolder1 = Common::Directory::CachePath + "OdSegmentationImage/";
									CreateDirectoryA(odSegmentationFolder1.toStdString().c_str(), NULL);
									cv::Mat drawImg = img.clone();
									//cv::Mat drawImg = green3.clone();

									int index = 0;
									for (const auto& r : segRects)
									{
										cv::rectangle(drawImg, r, cv::Scalar(0, 255, 0), 2);   // green, thickness = 2
										std::string score = segRectScore[index];
							
										int fontFace = cv::FONT_HERSHEY_SIMPLEX;
										double fontScale = 0.6;                 // Adjust size
										int thickness = 2;
										cv::Scalar color(0, 255, 0);             // Green (BGR)
										int lineType = cv::LINE_AA;
										cv::Point textPos(r.x + 5, r.y + 20);

										cv::putText(
											drawImg,
											score,
											textPos,          // Bottom-left corner of text
											fontFace,
											fontScale,
											color,
											thickness,
											lineType
										);
										index++;
									}
									
									// Save result
									std::string savePath = odSegmentationFolder1.toStdString()
										+ optic.opticId.toStdString() + "_" + optic.channel.toStdString() + "seg_result.jpg";
									cv::imwrite(savePath, drawImg);
									// ===
								}
								else
								{
									qDebug() << "@@@@@@@@@@@@ NO SEGMENTATION ENABLED @@@@@@@@@@@@";
								}
								// ==- segmentation ===


								odOpticsList.append(optic);
								odResultList.append(od_result);
								modelList.append(modelName);
							}
						}
					}
				}
			}
		}
	}

	objectDetectionResultToJsonObject(odResultList, odOpticsList, modelList);
	return false;
}

bool InspectionThread::samInspectionFromFiles(QStringList imgPaths, QStringList lightingIDs, AlgoGraph* algoGraph, const QHash<QString, QPointF>& offsets, const QHash<QString, LocAngle>& locAngles)
{
	qDebug() << "samInspectionFromFiles";

	std::unordered_map<std::string, cv::Mat> images;
	std::unordered_map<double, std::unordered_map<std::string, std::list<cv::Vec6f>>> lines;

	algoGraph->needSAM(algoGraph->templateId(), lines, offsets, locAngles, true);

	qDebug() << "imgPaths:" << imgPaths;
	if (imgPaths.size() == lightingIDs.size())
	{
		for (int i = 0; i < imgPaths.size(); i++)
		{

			for (const auto& angleEntry : lines) {  // Loop through the outer map (angle -> inner map)
				double angle = angleEntry.first;    // This is the angle
				const auto& opticMap = angleEntry.second;  // Inner map (opticID -> list of rects)

				// Print the angle

				// Loop through the inner map (opticID -> list of rects)
				for (const auto& opticEntry : opticMap) {
					const std::string& opticID = opticEntry.first;  // This is the opticID
					if (opticID == lightingIDs[i].toStdString())
					{
						auto imgPath = imgPaths[i];
						if (QFileInfo::exists(imgPath))
						{
							MIL_INT sizeX = 0, sizeY = 0, bandSize = 0;

							bandSize = mtrx::get_band(imgPath);
							sizeX = mtrx::get_width(imgPath);
							sizeY = mtrx::get_height(imgPath);

							MIL_ID milImg = M_NULL;
							if (bandSize == 1) MbufAlloc2d(M_DEFAULT_HOST, sizeX, sizeY, 8 + M_UNSIGNED, M_IMAGE + M_PROC, &milImg);
							else MbufAllocColor(M_DEFAULT_HOST, 3, sizeX, sizeY, 8 + M_UNSIGNED, M_IMAGE + M_PROC, &milImg);

							MIL_INT imgType = M_BMP;
							if (util::isPNG(imgPath)) imgType = M_PNG;
							if (util::isBMP(imgPath)) imgType = M_BMP;
							MbufImportA(imgPath.toStdString().c_str(), imgType, M_LOAD, M_DEFAULT_HOST, &milImg);
							MimConvolve(milImg, milImg, M_SMOOTH);
							cv::Mat img;
							util::Mil_to_cv(milImg, img);

							/*	QString imgChannel = "Green";
								getImageChannel(imgChannel, img, img);*/
							if (images.find(lightingIDs[i].toStdString()) == images.end())
							{
								images[lightingIDs[i].toStdString()] = img;
							}

							if (true)
							{
								// Make a copy to draw on (to preserve original rotatedImage)
								cv::Mat displayImage = img.clone();

								// Draw each rotated rect
								auto lineList = opticEntry.second;
								for (const auto& l : lineList) {
									auto rect = cv::Rect(cv::Point2i(l[0], l[1]), cv::Point2i(l[2], l[3]));
									cv::rectangle(displayImage, rect, cv::Scalar(0, 255, 0), 2);  // Green rects

									cv::line(displayImage, cv::Point2i(l[0], l[1]), cv::Point2i(l[2], l[3]), cv::Scalar(0, 255, 0), 2);
								}


								double scaleFactor = 0.15; // or adjust to something smaller like 0.3, 0.2, etc.

								cv::Mat resizedDisplay;
								//cv::resize(displayImage, resizedDisplay, cv::Size(), scaleFactor, scaleFactor);

								//cv::imshow("Rotated Image with Annotations", resizedDisplay);
								//cv::waitKey(0);

								//std::string savePath = opticID + "angle" + std::to_string(int(angle)) + "_rotated.png";
								//cv::imwrite(savePath, displayImage);  // save full-sized annotated image
							}

							mtrx::free_buffer(milImg);
						}
					}
				}
			}
		}
	}

	// sample data --
	//g_segModel->runModel_segmentation(images, rectsMap, _segmentationScore, true);
	g_segModel->runModel_segmentation_wireBond(images, lines, _segmentationScore, true);

	std::unordered_map<std::string, cv::Mat> resultMasks;
	resultMasks = g_segModel->getSegmentationMaskResult();

	QString segmentationMaskFolderPath = Common::Directory::CachePath + "AIWireBondSegmentationMask/";
	CreateDirectoryA(segmentationMaskFolderPath.toStdString().c_str(), NULL);
	for (auto& i : resultMasks)
	{
		std::string opticId = i.first;
		QString maskImgPath = segmentationMaskFolderPath + opticId.c_str() + ".jpg";

		MIL_ID milImg = M_NULL;
		util::cv_to_Mil(i.second, milImg);
		MbufExportA(maskImgPath.toStdString().c_str(), M_JPEG_LOSSY, milImg);
		mtrx::free_buffer(milImg);
	}

	//MIL_ID maskMil;
	//util::cv_to_Mil(resultMask[imgPaths[1].toStdString()], maskMil);
	return true;


}

bool InspectionThread::wirebondInspectionFromFiles(QStringList imgPaths, QStringList lightingIDs, AlgoGraph* algoGraph, const QHash<QString, QPointF>& offsets, const QHash<QString, LocAngle>& locAngles)
{
	qDebug() << "wirebondInspectionFromFiles";

	std::unordered_map<std::string, cv::Mat> images;
	QHash<QString, MIL_ID>  milImages;
	std::unordered_map<std::string, std::vector<LineData>> lines;
	double bondSize = 0;
	double padding = 0;

	if (imgPaths.size() == lightingIDs.size())
	{
		for (int i = 0; i < imgPaths.size(); i++)
		{
			auto imgPath = imgPaths[i];

			if (QFileInfo::exists(imgPath))
			{
				MIL_INT sizeX = 0, sizeY = 0, bandSize = 0;

				bandSize = mtrx::get_band(imgPath);
				sizeX = mtrx::get_width(imgPath);
				sizeY = mtrx::get_height(imgPath);

				MIL_ID milImg = M_NULL;
				if (bandSize == 1) MbufAlloc2d(M_DEFAULT_HOST, sizeX, sizeY, 8 + M_UNSIGNED, M_IMAGE + M_PROC, &milImg);
				else MbufAllocColor(M_DEFAULT_HOST, 3, sizeX, sizeY, 8 + M_UNSIGNED, M_IMAGE + M_PROC, &milImg);

				MIL_INT imgType = M_BMP;
				if (util::isPNG(imgPath)) imgType = M_PNG;
				if (util::isBMP(imgPath)) imgType = M_BMP;
				MbufImportA(imgPath.toStdString().c_str(), imgType, M_LOAD, M_DEFAULT_HOST, &milImg);
				//MimConvolve(milImg, milImg, M_SMOOTH);

				milImages[lightingIDs[i]] = milImg;

			}
		}
	}

	qDebug() << "getWireLines";
	algoGraph->getWireLines(algoGraph->templateId(), milImages, lines, offsets, locAngles, true);


	for (auto it = milImages.begin(); it != milImages.end(); ++it) {
		const QString& milImg_opticID = it.key();
		MIL_ID milImg = it.value();

		for (const auto& l : lines)
		{
			const std::string& l_opticID = l.first; //this is the opticID
			if (l_opticID == milImg_opticID.toStdString())
			{
				cv::Mat img;
				util::Mil_to_cv(milImg, img);

				if (images.find(l_opticID) == images.end())
				{
					images[l_opticID] = img;
				}

				if (true)
				{
					// Make a copy to draw on (to preserve original rotatedImage)
					cv::Mat displayImage = img.clone();

					// Draw each rotated rect
					auto lineList = l.second;
					for (const auto& line : lineList) {

						auto l = line.points;
						auto rect = cv::Rect(cv::Point2i(l.front().x, l.front().y), cv::Point2i(l.back().x, l.back().y));
						cv::rectangle(displayImage, rect, cv::Scalar(0, 255, 0), 2);  // Green rects

						for (int i = 0; i < l.size(); i++)
						{
							if (i < l.size() - 1)
							{
								auto p1 = l[i];
								auto p2 = l[i + 1];
								cv::line(displayImage, cv::Point2i(p1.x, p1.y), cv::Point2i(p2.x, p2.y), cv::Scalar(0, 255, 0), 2);
							}
						}

					}

					std::string savePath = l_opticID + "_wirebondLines.png";
					cv::imwrite(savePath, displayImage);  // save full-sized annotated image
				}
			}
		}
	}

	qDebug() << "freeMilBuffer";
	for (auto it = milImages.begin(); it != milImages.end(); ++it) {
		const QString& milImg_opticID = it.key();
		MIL_ID milImg = it.value();
		if (milImg) mtrx::free_buffer(milImg);
	}

	qDebug() << "done";
	// sample data --
	auto start = std::chrono::high_resolution_clock::now();
	g_segModel->runModel_segmentation_wireBond_v2(images, lines, -0.5, true);
	auto end = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double, std::milli> duration = end - start;
	std::cout << "runModel_segmentation_wireBond_v2 execution time: " << duration.count() << " ms" << std::endl;

	std::unordered_map<std::string, cv::Mat> resultMasks;
	resultMasks = g_segModel->getSegmentationMaskResult();

	QString segmentationMaskFolderPath = Common::Directory::CachePath + "AIWireBondSegmentationMask/";
	CreateDirectoryA(segmentationMaskFolderPath.toStdString().c_str(), NULL);
	for (auto& i : resultMasks)
	{
		std::string opticId = i.first;
		QString maskImgPath = segmentationMaskFolderPath + opticId.c_str() + ".jpg";

		MIL_ID milImg = M_NULL;
		util::cv_to_Mil(i.second, milImg);
		MbufExportA(maskImgPath.toStdString().c_str(), M_JPEG_LOSSY, milImg);
		mtrx::free_buffer(milImg);
	}

	return true;
}

bool InspectionThread::samInspectionTest()
{
	std::unordered_map<std::string, cv::Mat> images;
	std::unordered_map<std::string, std::list<cv::Rect>> rectsMap;

	g_segModel->runModel_segmentation(images, rectsMap, _segmentationScore, true);
	return false;
}

bool InspectionThread::alloptics2D3D_exist(QString opticName)
{
	for (int i = 0; i < _allOptics_2D_3D.size(); i++)
	{
		if (_allOptics_2D_3D[i].name == opticName) return true;
	}
	return false;
}

void InspectionThread::formImage(QImage& img, const unsigned char* pRedBuf, const unsigned char* pGreenBuf, const unsigned char* pBlueBuf)
{
	QRgb* pBuf;
	int offset = 0;

	for (int i = 0; i < img.height(); ++i)
	{
		pBuf = reinterpret_cast<QRgb*>(img.scanLine(i));
		offset = i * img.width();

		for (int j = 0; j < img.width(); ++j)
		{
			pBuf[j] = qRgb(pRedBuf[offset + j], pGreenBuf[offset + j], pBlueBuf[offset + j]);
		}
	}
}

void InspectionThread::formImage(QImage& img, const unsigned char* pImageBuf)
{
	uchar* pBuf;
	int offset = 0;

	for (int i = 0; i < img.height(); ++i)
	{
		pBuf = img.scanLine(i);
		offset = i * img.width();

		for (int j = 0; j < img.width(); ++j)
		{
			pBuf[j] = pImageBuf[offset + j];
		}
	}
}

void InspectionThread::clearResults()
{
	_algoDefectResults.clear();
	_defectResults.clear();
	_distanceMeasurementVector.clear();
	_barcodeDecoderInfoVector.clear();
}
//
//void InspectionThread::addDefects(QVector<ct::AlgoDefectResult> res, QView& view, bool locatorDefect)
//{
//	bool dieShearUnit = false;
//
//	auto worldScale = ScaleManager::instance().world_scale();
//
//	for (int i = 0; i < res.size(); i++)
//	{
//		if (res[i].dieShear)
//		{
//			dieShearUnit = true;
//
//			ct::DefectResult def;
//			def.view_id = view.id.toStdString();
//			def.view_name = view.name.toStdString();
//			def.view_x = view.pDragBox->getGeometry().x();
//			def.view_y = view.pDragBox->getGeometry().y();
//			def.view_w = view.pDragBox->getGeometry().width();
//			def.view_h = view.pDragBox->getGeometry().height();
//			def.plane_x = _planeOffset.x();
//			def.plane_y = _planeOffset.y();
//			def.worldScale = worldScale;
//
//			for (int j = 0; j < _dragROI.size(); j++)
//			{
//				if (_dragROI[j]->getId() == res[i].vo_id.c_str())
//				{
//					res[i].vo_x = _dragROI[j]->getGeometry().x();
//					res[i].vo_y = _dragROI[j]->getGeometry().y();
//					res[i].vo_w = _dragROI[j]->getGeometry().width();
//					res[i].vo_h = _dragROI[j]->getGeometry().height();
//				}
//			}
//
//			QString& tagName = res[i].customTagName;
//			tagName = defectMapping(tagName);
//
//
//			def.algoDefResult = res[i];
//			_defectResults.append(def);
//
//			break;
//		}
//	}
//	if (!dieShearUnit)
//	{
//		for (int i = 0; i < res.size(); i++)
//		{
//			ct::DefectResult def;
//			def.view_id = view.id.toStdString();
//			def.view_name = view.name.toStdString();
//			def.view_x = view.pDragBox->getGeometry().x();
//			def.view_y = view.pDragBox->getGeometry().y();
//			def.view_w = view.pDragBox->getGeometry().width();
//			def.view_h = view.pDragBox->getGeometry().height();
//			def.plane_x = _planeOffset.x();
//			def.plane_y = _planeOffset.y();
//			def.worldScale = worldScale;
//
//			for (int j = 0; j < _dragROI.size(); j++)
//			{
//				if (_dragROI[j]->getId() == res[i].vo_id.c_str())
//				{
//					res[i].vo_x = _dragROI[j]->getGeometry().x();
//					res[i].vo_y = _dragROI[j]->getGeometry().y();
//					res[i].vo_w = _dragROI[j]->getGeometry().width();
//					res[i].vo_h = _dragROI[j]->getGeometry().height();
//				}
//			}
//			QString& tagName = res[i].customTagName;
//			tagName = defectMapping(tagName);
//
//			def.algoDefResult = res[i];
//			_defectResults.append(def);
//		}
//	}
//	
//}

void InspectionThread::addDefects(QVector<ct::AlgoDefectResult> res, QView& view, bool locatorDefect)
{
	if (res.isEmpty()) return;

	// Cache frequently used values
	const auto worldScale = ScaleManager::instance().world_scale();
	const auto& viewGeometry = view.pDragBox->getGeometry();
	const auto& planeOffset = _planeOffset;

	// Create lookup map for dragROI to avoid O(n²) complexity
	QHash<QString, QRect> roiGeometryMap;
	roiGeometryMap.reserve(_dragROI.size());
	for (const auto& roi : _dragROI) {
		roiGeometryMap.insert(roi->getId(), roi->getGeometry().toRect());
	}

	// Check if we have die shear defects and should process only the first one
	bool foundDieShear = false;
	for (const auto& defectResult : res) {
		if (defectResult.dieShear) {
			foundDieShear = true;
			break;
		}
	}

	// Process defects - either first die shear only or all non-die shear
	for (int i = 0; i < res.size(); i++) {
		// Skip if we found die shear but this isn't one, or if this isn't the first die shear
		/*if (foundDieShear && (!res[i].dieShear || (res[i].dieShear && i > 0))) {
			continue;
		}*/
		if (foundDieShear && !res[i].dieShear ) {
			continue;
		}

		ct::DefectResult def;

		// Set view properties (cached values)
		def.view_id = view.id.toStdString();
		def.view_name = view.name.toStdString();
		def.view_x = viewGeometry.x();
		def.view_y = viewGeometry.y();
		def.view_w = viewGeometry.width();
		def.view_h = viewGeometry.height();
		def.plane_x = planeOffset.x();
		def.plane_y = planeOffset.y();
		def.worldScale = worldScale;

		// Update VO geometry using lookup map (O(1) instead of O(n))
		const QString voId = QString::fromStdString(res[i].vo_id);
		if (roiGeometryMap.contains(voId)) {
			const QRect& geometry = roiGeometryMap[voId];
			res[i].vo_x = geometry.x();
			res[i].vo_y = geometry.y();
			res[i].vo_w = geometry.width();
			res[i].vo_h = geometry.height();
		}

		// Apply defect mapping
		res[i].customTagName = defectMapping(res[i].customTagName);

		def.algoDefResult = res[i];
		_defectResults.append(def);

		// processed a die shear defect
		if (res[i].dieShear) {
			break;
		}
	}

}

void InspectionThread::generateDefectImages(QImage& img, QStringList& defectTagNames, const QString& voName, const QRect& voRect)
{
	QString defectTagName = "Good";
	if (defectTagNames.size() > 0) defectTagName = defectTagNames[0];
	QString defectFolderPath = Common::Directory::getProductionDefectPath() + defectTagName + "\\";
	CreateDirectoryA(defectFolderPath.toStdString().c_str(), NULL);

	QString defectImgPath = defectFolderPath + voName + ".jpg";
	QImage defImg = img.copy(voRect);

	IST::instance().enqueue(defectImgPath.toStdString(), defImg);

	if (defectTagName == "Good") return;

	QPainter painter(&defImg);
	QPen pen(Qt::red);
	painter.setPen(pen);
	QBrush brush(Qt::transparent);
	painter.setBrush(brush);

	for (int j = 0; j < _defectResults.size(); j++)
	{
		if (_defectResults[j].algoDefResult.vo_name == voName.toStdString())
		{
			_defectResults[j].tagNames = defectTagNames;
			auto x = _defectResults[j].algoDefResult.def_x;
			auto y = _defectResults[j].algoDefResult.def_y;
			auto w = _defectResults[j].algoDefResult.def_w;
			auto h = _defectResults[j].algoDefResult.def_h;
			painter.drawRect(x, y, w, h);
		}
	}

	// End painting
	painter.end();

	QString defectRectFolderPath = Common::Directory::getProductionDefectPath() + defectTagName + "_defRect\\";
	CreateDirectoryA(defectRectFolderPath.toStdString().c_str(), NULL);
	QString defectRectImgPath = defectRectFolderPath + voName + "_def.jpg";
	IST::instance().enqueue(defectRectImgPath.toStdString(), defImg);
}

void InspectionThread::getVisionObjects(QHash<QString, QRectF>& visionObjectsRect, const QString& viewID)
{
	QPointF viewOffset = { 0,0 };

	if (g_viewMode == int(ViewMode::PLANE))
	{
		QHash<QString, QView>::const_iterator v = _views.constBegin();
		while (v != _views.constEnd())
		{
			if (v.value().id == viewID)
			{
				QPointF fovView = ScaleManager::instance().to_fov_px(v.value());
				viewOffset.setX(fovView.x());
				viewOffset.setY(fovView.y());
				break;
			}
			++v;
		}
	}


	QHash<QString, QVisionObject>::const_iterator vo = _visionObjects.constBegin();
	while (vo != _visionObjects.constEnd())
	{
		if (vo.value().viewID == viewID)
		{
			QRect voRect = QRect(vo.value().rect.x() - viewOffset.x(), vo.value().rect.y() - viewOffset.y(), vo.value().rect.width(), vo.value().rect.height());
			visionObjectsRect.insert(vo.value().objectName, voRect);
		}
		++vo;
	}
}

void InspectionThread::getVisionObjectsForHeightMap(QHash<QString, QRectF>& visionObjectsRect, const QString& id, MIL_ID mImap)
{
	QPointF offset;

	if (_heightMaps.contains(id)) {
		auto h = _heightMaps.find(id);
		//QPointF fovLine = mmtoFovPX(h.value());
		QPointF fovLine;
		if (h.value().type == ct::s_stitch_linescan) {
			for (auto& childL : _heightMaps) {
				if (childL.map_to_slinescan == h.value().id) {
					if (childL.id.contains("-0")) {
						fovLine = ScaleManager::instance().to_fov_px(childL);
					}
				}
			}
		}
		else {
			fovLine = ScaleManager::instance().to_fov_px(h.value());
		}
		offset.setX(fovLine.x());
		offset.setY(fovLine.y());

		ct::logger::trace("[InspThread] FOV Line: %f, %f", fovLine.x(), fovLine.y());
		ct::logger::trace("[InspThread] Offset: %f, %f", offset.x(), offset.y());

		for (const auto& voID : h->vision_obj_IDs) {

			if (_visionObjects.contains(voID)) {
				auto vo = _visionObjects.find(voID);

				ct::logger::trace("[InspThread] VO: %f, %f, %f, %f", vo.value().rect.x(), vo.value().rect.y(), vo.value().rect.width(), vo.value().rect.height());
				QRect voRect = QRect(vo.value().rect.x() - offset.x(), vo.value().rect.y() - offset.y(), vo.value().rect.width(), vo.value().rect.height()); //TJ_smartRay offset.y()+550 (removed)
				ct::logger::trace("[InspThread] Before insert: %d, %d, %d, %d", voRect.x(), voRect.y(), voRect.width(), voRect.height());
				visionObjectsRect.insert(vo.value().objectName, voRect);

				auto r = visionObjectsRect[vo.value().objectName];
				ct::logger::trace("[InspThread] After insert(%s): %f, %f, %f, %f", vo.value().objectName.toStdString().c_str(), r.x(), r.y(), r.width(), r.height());

				if (mImap) {
					auto cropped = mtrx::crop(mImap, r.x(), r.y(), r.width(), r.height());
					MbufSaveA(QString("crop%1.jpg").arg(vo.value().objectName).toStdString().c_str(), cropped);
				}
			}
		}
	}
}

void InspectionThread::offsetVisionObject(const QHash<QString, QPointF>& locatorOffsets, QString visionObjectName, QRect& rect)
{
	QHash<QString, QPointF>::const_iterator lo = locatorOffsets.constBegin();
	while (lo != locatorOffsets.constEnd())
	{
		if (lo.key() == visionObjectName)
		{
			int width = rect.width();
			int height = rect.height();
			int offsetedX = rect.x() + (int)lo.value().x();
			int offsetedY = rect.y() + (int)lo.value().y();

			/*if (offsetedX + width > _imgWidth) offsetedX = offsetedX - (_imgWidth - (offsetedX + width));
			if (offsetedX < 0) offsetedX = 0;

			if (offsetedY + height > _imgHeight) offsetedY = offsetedY - (_imgHeight - (offsetedY + height));
			if (offsetedY < 0) offsetedY = 0;*/

			rect = QRect(offsetedX, offsetedY, width, height);
			return;
		}
		++lo;
	}

}

void InspectionThread::removeLocOffsetfromRect(const QHash<QString, QPointF>& locatorOffsets, QString visionObjectName, QRectF& rect)
{
	QHash<QString, QPointF>::const_iterator lo = locatorOffsets.constBegin();
	while (lo != locatorOffsets.constEnd())
	{
		if (lo.key() == visionObjectName)
		{
			int width = rect.width();
			int height = rect.height();
			int offsetedX = rect.x() - lo.value().x();
			int offsetedY = rect.y() - lo.value().y();

			rect = QRect(offsetedX, offsetedY, width, height);
			return;
		}
		++lo;
	}

}

bool InspectionThread::locatorInspection(const QHash<QString, QRectF>& visionObjectsRect, QHash<QString, QHash<QString, QPointF>>& locatorOffsets, QHash<QString, QHash<QString, LocAngle>>& locatorAngles, const QString& viewID)
{
	QHash<QString, QHash<QString, QPointF>> curViewLocatorOffsets;
	QHash<QString, QHash<QString, LocAngle>> curViewLocatorAngles;

	QHash<QString, QRectF>::const_iterator vor = visionObjectsRect.constBegin();
	while (vor != visionObjectsRect.constEnd())
	{
		for (int i = 0; i < _dragROI.size(); i++)
		{
			if (_dragROI[i]->getName() == vor.key())
			{
				if (_locResults.contains(_dragROI[i]->getName())) break;
				//ignore vision Object
				if (_visionObjects.find(_dragROI[i]->getId()).value().ignore) break;
				//skip vision Object
				if (_visionObjects.find(_dragROI[i]->getId()).value().skip) break;
				if (_visionObjects.find(_dragROI[i]->getId()).value().forcedSkip) break;
				_algo->setCroppedBuffer(vor.value().toRect());
				//_algo->loadImage(pFrameInfo->_pRedImageBuf, pFrameInfo->_pGreenImageBuf, pFrameInfo->_pBlueImageBuf, QSize(pFrameInfo->_width, pFrameInfo->_height), vor.value().toRect());
				auto algoGraph = _dragROI[i]->algoGraph();
				if (algoGraph)
				{
					algoGraph->setAlgoInspector(_algo);
					algoGraph->setVisionObject(&_visionObjects.find(_dragROI[i]->getId()).value());
					QString processMsg;
					QJsonObject processData;
					QHash<QString, QPointF> offsets;
					QHash<QString, LocAngle> angles;

					//get OD Locator Bbox
					algoGraph->clearDefects();
					algoGraph->clearDebugResults();

					QJsonObject odResult;
					auto odr = _objectDetectionResult.find(vor.key());
					if (odr != _objectDetectionResult.end()) odResult = odr.value();

					QJsonObject vidiObj;
					algoGraph->processLocator(processMsg, processData, vidiObj, odResult, offsets, angles);

					auto locDef = algoGraph->getDefects();

					if (!_views.contains(viewID)) {
						ct::logger::error("[InspThread] Failed to inspect locator. Invalid view ID: %s", viewID.toStdString().c_str());
						continue;
					}

					addDefects(locDef, _views.find(viewID).value(), true);
					addUnitDefectResults(locDef, _views.find(viewID).value());


					auto locDebugRes = algoGraph->getDebugResults();
					addUnitDebugResults(locDebugRes, _views.find(viewID).value());

					if (locDef.size() > 0)
					{
						_locResults.insert(_dragROI[i]->getName(), true);
					}
					else
					{
						_locResults.insert(_dragROI[i]->getName(), false);
					}

					g_locatorOffsets = offsets;
					locatorOffsets.insert(_dragROI[i]->getName(), offsets);
					locatorAngles.insert(_dragROI[i]->getName(), angles);

					curViewLocatorOffsets.insert(_dragROI[i]->getName(), offsets);
					curViewLocatorAngles.insert(_dragROI[i]->getName(), angles);

					algoGraph->clearDynamicMaskObjects();
				}
				break;
			}
		}
		++vor;
	}

	/*ct::logger::info("[InspectionThread] LocatorInfoEmitted: %d ,%s, %s", locatorOffsets.size(), viewID.toStdString());
	bool locatorFail = true;
	QPointF curlocOffset;

	for (const auto& roiID : locatorOffsets.keys()) {
		const auto& multiLocatorHash = locatorOffsets[roiID];
		for (const auto& locatorID : multiLocatorHash.keys()) {
			const auto& locOffset = multiLocatorHash[locatorID];
			locatorFail = false;
			curlocOffset = locOffset;
			break;
		}
	}

	bool locatorAngleFail = true;
	double curlocAngle;
	for (const auto& roiID : curViewLocatorAngles.keys()) {
		const auto& multiLocatorAnglesHash = curViewLocatorAngles[roiID];
		for (const auto& locatorID : multiLocatorAnglesHash.keys()) {
			const auto& locAngle = multiLocatorAnglesHash[locatorID];
			locatorAngleFail = false;
			curlocAngle = locAngle;
			if(locAngle != 0) break;
		}
	}*/

	//emit locatorInfo(curlocOffset, curlocAngle, viewID, indexID, locatorFail, locatorAngleFail);


	return true;
}

bool InspectionThread::postInspection(const QHash<QString, QRectF>& visionObjectsRect, const QHash<QString, QHash<QString, QPointF>>& locatorOffsets, const QString& imgType)
{
	QHash<QString, QRectF>::const_iterator vor = visionObjectsRect.constBegin();

	while (vor != visionObjectsRect.constEnd())
	{
		bool runPostInsp = false;
		if (_locResults.find(vor.key()) != _locResults.end())
		{
			runPostInsp = !_locResults.find(vor.key()).value();

		}
		else runPostInsp = true;

		runPostInsp = true;
		if (runPostInsp)
		{
			for (int i = 0; i < _dragROI.size(); i++)
			{
				if (_dragROI[i]->getName() == vor.key())
				{
					//ignore all vision Object
					if (_visionObjects.find(_dragROI[i]->getId()).value().ignore) {
						ct::logger::info("[Inspection] Vision object ignored: %s", _dragROI[i]->getId().toStdString().c_str());
						break;
					}

					//skip all vision Object
					if (_visionObjects.find(_dragROI[i]->getId()).value().skip|| _visionObjects.find(_dragROI[i]->getId()).value().forcedSkip) {
						ct::logger::info("[Inspection] Vision object skipped: %s", _dragROI[i]->getId().toStdString().c_str());
						break;
					}

					auto r = vor.value().toRect();
					ct::logger::trace("Crop buffer(%s) %d, %d, %d, %d", vor.key().toStdString().c_str(), r.x(), r.y(), r.width(), r.height());
		
					auto algoGraph = _dragROI[i]->algoGraph();
					if (algoGraph)
					{
						_algo->setCroppedBuffer(vor.value().toRect());
						algoGraph->setAlgoInspector(_algo);
						QVisionObject* visionObj = nullptr;
						auto vo = _visionObjects.find(_dragROI[i]->getId());
						if (vo != _visionObjects.end()) visionObj = &vo.value();
						algoGraph->setVisionObject(visionObj);

						QString processMsg;
						QJsonObject processData;
						auto l = locatorOffsets.find(vor.key());
						QHash<QString, QPointF> offsets;
						if (l != locatorOffsets.end()) offsets = l.value();
						algoGraph->clearDefects();
						algoGraph->clearMeasurementDatas();
						algoGraph->clearDistanceMeasurement();
						algoGraph->clearBarcodeDecoderData();

						QJsonObject vidiResult;
						auto vr = _vidiResults.find(vor.key());
						if (vr != _vidiResults.end()) vidiResult = vr.value();
						 
						QJsonObject odResult;
						auto odr = _objectDetectionResult.find(vor.key());
						if (odr != _objectDetectionResult.end()) odResult = odr.value();

						QVector<DynamicMaskObject> srResult;
						auto sr = _segmentationResult.find(vor.key());
						if (sr != _segmentationResult.end()) srResult = sr.value();

						QVector<DynamicDataObject> dataResult;
						auto dr = _dataResult.find(vor.key());
						if (dr != _dataResult.end()) dataResult = dr.value();

						ct::logger::info("[Post Inspection]3");
						algoGraph->processGraph(processMsg, processData, vidiResult, odResult, srResult, dataResult, imgType, true, offsets);
						for (auto& meas : algoGraph->getMeasurementDatas()) {
							_measArray.append(meas);
						}
						ct::logger::info("[Post Inspection]4");
						QView view;
						QHash<QString, QView>::iterator v;

						if (!_views.contains(visionObj->viewID)) {
							ct::logger::error("[InspThread] Failed to inspect. Invalid view ID: %s", visionObj->viewID.toStdString().c_str());
							continue;
						}

						if (visionObj) v = _views.find(visionObj->viewID);
						if (v != _views.end()) view = _views.find(visionObj->viewID).value();
						addDefects(algoGraph->getDefects(), view);
						addDistanceMeasurement(algoGraph->getDistanceMeasurementResults(), view);
						addBarcodeDecoderInfoData(algoGraph->getBarcodeDecoderData());
						_dataResult.insert(vor.key(), algoGraph->getDynamicDataObjects());

						addUnitBarcodeResults(algoGraph->getBarcodeDecoderData());
						addUnitDefectResults(algoGraph->getDefects(), view);
						addUnitDebugResults(algoGraph->getDebugResults(), view);

						algoGraph->clearDynamicMaskObjects();
					}
					break;
				}
			}
		}
		else {
			ct::logger::warn("[Inspection] Locator failed on vision object: %s", vor.key().toStdString().c_str());
		}
		++vor;
	}
	return true;
}

bool InspectionThread::multi_postInspection(const QHash<QString, QRectF>& visionObjectsRect, const QHash<QString, QHash<QString, QPointF>>& locatorOffsets, const QString& imgType)
{
	Timer time;
	
	auto keys = visionObjectsRect.keys();

#pragma omp parallel for
	for (int i = 0; i < keys.size(); ++i)
	{
		const QString& key = keys[i];
		bool runPostInsp = false;
		if (_locResults.find(key) != _locResults.end())
		{
			runPostInsp = !_locResults.find(key).value();

		}
		else runPostInsp = true;
		
		runPostInsp = true;
		if (runPostInsp)
		{
		
			// Skip ignored or skipped objects
			for (int j = 0; j < _dragROI.size(); ++j) {
				if (_dragROI[j]->getName() != key)
					continue;

				const QRectF& rect = visionObjectsRect[key];
				QHash<QString, QPointF> offsets = locatorOffsets.value(key);
				QJsonObject vidiResult = _vidiResults.value(key);
				QJsonObject odResult = _objectDetectionResult.value(key);
				QVector<DynamicMaskObject> srResult = _segmentationResult.value(key);
				QVector<DynamicDataObject> dataResult = _dataResult.value(key);
				QVisionObject* visionObj = _visionObjects.contains(_dragROI[j]->getId()) ? &_visionObjects[_dragROI[j]->getId()] : nullptr;

				const auto& vObj = _visionObjects.value(_dragROI[j]->getId());
				if (vObj.ignore || vObj.skip || vObj.forcedSkip)
					continue;

				AlgoGraph* graph = _dragROI[j]->algoGraph();
				if (!graph) continue;

				//ct::logger::info("Algo Clone - start");
				Algo* algoClone = _algo->clone();
				//ct::logger::info("Algo Clone - done");
				AlgoGraph* graphClone = graph->clone();
				//ct::logger::info("Graph Clone - done");

				/*Algo* algoClone = _algo;
				AlgoGraph* graphClone = graph;*/

				ct::logger::info("Algo Set Cropped Buffer - start");
				algoClone->setCroppedBuffer(rect.toRect());
				ct::logger::info("Algo Set Cropped Buffer - Done");

				ct::logger::info("Algo Set Algo Inspector - start");
				graphClone->setAlgoInspector(algoClone);
				ct::logger::info("Algo Set Algo Inspector - Done");
				graphClone->setVisionObject(visionObj);
				ct::logger::info("Algo Set Vision Object - Done");

				graphClone->clearDefects();
				graphClone->clearMeasurementDatas();
				graphClone->clearDistanceMeasurement();
				graphClone->clearBarcodeDecoderData();

				QString processMsg;
				QJsonObject processData;

				ct::logger::info("processGraph-start");
				graphClone->processGraph(processMsg, processData, vidiResult, odResult, srResult, dataResult, imgType, true, offsets);
				ct::logger::info("processGraph-end");

				//Use mutex for shared resource write
				#pragma omp critical
				{
					for (auto& meas : graphClone->getMeasurementDatas()) {
						_measArray.append(meas);
					}

					if (visionObj && _views.contains(visionObj->viewID)) {
						QView view = _views[visionObj->viewID];
						addDefects(graphClone->getDefects(), view);
						addDistanceMeasurement(graphClone->getDistanceMeasurementResults(), view);
						addBarcodeDecoderInfoData(graphClone->getBarcodeDecoderData());
						addUnitBarcodeResults(graphClone->getBarcodeDecoderData());
						addUnitDefectResults(graphClone->getDefects(), view);
						addUnitDebugResults(graphClone->getDebugResults(), view);
						_dataResult.insert(key, graphClone->getDynamicDataObjects());
					}
				}
				//ct::logger::info("algoCloneRelease");
				algoClone->releaseClone();
				//ct::logger::info("algoCloneRelease - done");
				delete algoClone;
				//ct::logger::info("algoClone delete - done");
				delete graphClone;
				//ct::logger::info("algoGraph delete - done");
				break;  // stop after matching ROI
			}
		}
		else
		{
			ct::logger::warn("[Inspection] Locator failed on vision object: %s", key.toStdString().c_str());
		}
	}
	ct::logger::info("multiPostInsp Duration: %.5f", time.duration());
	return true;
}

bool InspectionThread::multi_postInspection_v2(const QHash<QString, QRectF>& visionObjectsRect, const QHash<QString, QHash<QString, QPointF>>& locatorOffsets, const QString& imgType)
{
	Timer time;

	// --- 1) Build a per-thread pool once ---
	const int T = omp_get_max_threads();
	std::vector<WorkerCtx> pool(T);

	for (int t = 0; t < T; ++t) {
		pool[t].algo = std::unique_ptr<Algo>(_algo->clone());
	}

	auto keys = visionObjectsRect.keys();

#pragma omp parallel for
	for (int i = 0; i < keys.size(); ++i)
	{
		const QString& key = keys[i];
		bool runPostInsp = false;
		if (_locResults.find(key) != _locResults.end())
		{
			runPostInsp = !_locResults.find(key).value();

		}
		else runPostInsp = true;

		runPostInsp = true;
		if (runPostInsp)
		{

			// Skip ignored or skipped objects
			for (int j = 0; j < _dragROI.size(); ++j) {
				if (_dragROI[j]->getName() != key)
					continue;

				const QRectF& rect = visionObjectsRect[key];
				QHash<QString, QPointF> offsets = locatorOffsets.value(key);
				QJsonObject vidiResult = _vidiResults.value(key);
				QJsonObject odResult = _objectDetectionResult.value(key);
				QVector<DynamicMaskObject> srResult = _segmentationResult.value(key);
				QVector<DynamicDataObject> dataResult = _dataResult.value(key);
				QVisionObject* visionObj = _visionObjects.contains(_dragROI[j]->getId()) ? &_visionObjects[_dragROI[j]->getId()] : nullptr;

				const auto& vObj = _visionObjects.value(_dragROI[j]->getId());
				if (vObj.ignore || vObj.skip || vObj.forcedSkip)
					continue;

				AlgoGraph* graph = _dragROI[j]->algoGraph();
				if (!graph) continue;

			
				AlgoGraph* graphClone = graph->clone();
				// --- 2) Grab this thread's worker and prepare it ---
				const int tid = omp_get_thread_num();
				auto& W = pool[tid];
				W.graph = graphClone;
				W.resetForNext();

				ct::logger::info("Algo Set Cropped Buffer - start");
				W.algo->setCroppedBuffer(rect.toRect());
				ct::logger::info("Algo Set Cropped Buffer - Done");

				ct::logger::info("Algo Set Algo Inspector - start");
				W.graph->setAlgoInspector(W.algo.get());
				ct::logger::info("Algo Set Algo Inspector - Done");
				W.graph->setVisionObject(visionObj);
				ct::logger::info("Algo Set Vision Object - Done");

				W.graph->clearDefects();
				W.graph->clearMeasurementDatas();
				W.graph->clearDistanceMeasurement();
				W.graph->clearBarcodeDecoderData();
				W.graph->clearDynamicDataObjects();

				QString processMsg;
				QJsonObject processData;

				ct::logger::info("processGraph-start");
				graphClone->processGraph(processMsg, processData, vidiResult, odResult, srResult, dataResult, imgType, true, offsets);
				ct::logger::info("processGraph-end");

				// Collect per-thread outputs (avoid holding references)
				W.meas = W.graph->getMeasurementDatas();          // returns by value or copy
				W.defects = W.graph->getDefects();
				W.dists = W.graph->getDistanceMeasurementResults();
				W.barcodeInfo = W.graph->getBarcodeDecoderData();
				W.debugItems = W.graph->getDebugResults();
				W.dataObjects = W.graph->getDynamicDataObjects();
#pragma omp critical
				{
					
					for (const auto& m : W.meas) _measArray.append(m);

					if (visionObj && _views.contains(visionObj->viewID)) {
						QView view = _views[visionObj->viewID];
						addDefects(W.defects, view);
						addDistanceMeasurement(W.dists, view);
						addBarcodeDecoderInfoData(W.barcodeInfo);
						addUnitBarcodeResults(W.barcodeInfo);
						addUnitDefectResults(W.defects, view);
						addUnitDebugResults(W.debugItems, view);
						_dataResult.insert(key, W.dataObjects);
					}
				}

				W.graph = nullptr;
				delete graphClone;
				break;  // stop after matching ROI
			}
		}
		else
		{
			ct::logger::warn("[Inspection] Locator failed on vision object: %s", key.toStdString().c_str());
		}
	}

	for (auto& w : pool) {
		if (w.algo) {
			w.algo->releaseClone(); 
			w.algo.reset();           // unique_ptr deletes
		}
	}

	ct::logger::info("multiPostInsp Duration: %.5f", time.duration());
	return true;
}

void InspectionThread::addDistanceMeasurement(QVector<DistanceMeasurementInfo> dRes, QView& view, bool locatorDefect)
{
	auto worldScale = ScaleManager::instance().world_scale();

	for (int i = 0; i < dRes.size(); i++)
	{
		dRes[i].viewId = view.id;
		dRes[i].viewName = view.name;
		dRes[i].viewX = view.pDragBox->getGeometry().x();
		dRes[i].viewY = view.pDragBox->getGeometry().y();
		dRes[i].viewW = view.pDragBox->getGeometry().width();
		dRes[i].viewH = view.pDragBox->getGeometry().height();
		dRes[i].planeX = _planeOffset.x();
		dRes[i].planeY = _planeOffset.y();
		dRes[i].worldScale = worldScale;

		for (int j = 0; j < _dragROI.size(); j++)
		{
			if (_dragROI[j]->getId() == dRes[i].voId)
			{
				dRes[i].voX = _dragROI[j]->getGeometry().x();
				dRes[i].voY = _dragROI[j]->getGeometry().y();
				dRes[i].voW = _dragROI[j]->getGeometry().width();
				dRes[i].voH = _dragROI[j]->getGeometry().height();
			}
		}

		_distanceMeasurementVector.append(dRes[i]);

	}
}

void InspectionThread::addBarcodeDecoderInfoData(QVector<BarcodeDecoderInfo> bInfoVector)
{
	/*if (!index.isEmpty())
	{
		for (auto& bInfo : bInfoVector)
		{
			bInfo.indexId = index;
		}
	}*/
	_barcodeDecoderInfoVector.append(bInfoVector);
}

bool InspectionThread::defectFound(QString viewID)
{
	bool defectFound = false;

	int defCount = 0;
	bool crossBoard = false;
	for (auto def : _defectResults)
	{
		if (def.view_id == viewID.toStdString())
		{
			defCount++;

			QString algoTagName = def.algoDefResult.customTagName;
			/*qDebug() << "algoTagName:" << algoTagName;
			getchar();*/
			if (algoTagName.contains("Cross Board")) crossBoard = true;
		}
	}

	if (!crossBoard && defCount > 0) defectFound = true;

	return defectFound;
}

void InspectionThread::addUnitBarcodeResults(QVector<BarcodeDecoderInfo> bInfoVector)
{
	for (int i = 0; i < bInfoVector.size(); i++)
	{
		for (QString opticID : _unitResultInfos.keys())
		{
			auto& unitResult = _unitResultInfos[opticID];
			unitResult.barcodeId = bInfoVector[i].decodedString;
		}
		break;
	}
}

void InspectionThread::addUnitDebugResults(const QVector<ct::AlgoDefectResult>& debugResults, QView& view, bool locatorDefect)
{
	for (int i = 0; i < debugResults.size(); i++)
	{
		auto opticID = debugResults[i].optic_id.c_str();
		if (_unitResultInfos.contains(opticID))
		{
			auto& unitResult = _unitResultInfos[opticID];
			auto debugRect = QRect(debugResults[i].def_x + debugResults[i].vo_x, debugResults[i].def_y + debugResults[i].vo_y, debugResults[i].def_w, debugResults[i].def_h);
			//ct::logger::info("Added Unit Debug Resutls: x:%d, y:%d, w:%d, h:%d", debugRect.x(), debugRect.y(), debugRect.width(), debugRect.height());
			unitResult.debugBoxes.append(debugRect);
		}
	}
}

void InspectionThread::addUnitDefectResults(const QVector<ct::AlgoDefectResult>& defectResults, QView& view, bool locatorDefect)
{
	for (QString unitOptic : _unitResultInfos.keys())
	{
		auto& unitResult = _unitResultInfos[unitOptic];
	
		bool missingUnitFlag = false;
		int failCount = 0;
		for (int i = 0; i < defectResults.size(); i++)
		{
			auto opticID = defectResults[i].optic_id.c_str();

			if (unitOptic == opticID)
			{

				auto defectRect = QRect(defectResults[i].def_x + defectResults[i].vo_x, defectResults[i].def_y + defectResults[i].vo_y, defectResults[i].def_w, defectResults[i].def_h);
				unitResult.defectBoxes.append(defectRect);

				QString tagName = defectResults[i].customTagName;
			
				tagName = defectMapping(tagName);
			

				unitResult.defectName = tagName;
				QString defectCode;
				if (_tagNameHash.contains(unitResult.defectName))
				{
					defectCode = _tagNameHash[unitResult.defectName].tCode;
					
				}
				unitResult.defectCode = defectCode;

				//// ---
				if (defectResults[i].customTagName == "Missing Unit")
				{
					qDebug() << "Missing Unit Found!!";
					missingUnitFlag = true;
					unitResult.defectCode = defectResults[i].customTagName;
				}
				else
				{
					failCount++;
				}
				//// ---
			}
		}

		if (failCount > 0)
		{
			unitResult.pass = false;
		}
		else if (failCount == 0)
		{
			unitResult.pass = true;
		}
	}
}

void InspectionThread::setUnitInspectionTime(double inspTime)
{
	double unitInspTime = inspTime / (double)_unitResultInfos.size();
	for (auto optic : _unitResultInfos.keys())
	{
		_unitResultInfos[optic].inspTime = QString::number(unitInspTime * 1000);
	}
}


bool InspectionThread::generateIgnoreDefect(const QHash<QString, QRectF>& visionObjectsRect, QString ignoreDefectName)
{
	QString defectName;

	QHash<QString, QRectF>::const_iterator vor = visionObjectsRect.constBegin();
	while (vor != visionObjectsRect.constEnd())
	{
		for (int i = 0; i < _dragROI.size(); i++)
		{
			if (_dragROI[i]->getName() == vor.key())
			{
				auto vo = _visionObjects.find(_dragROI[i]->getId()).value();
				if (vo.ignore || vo.skip||vo.forcedSkip)
				{
					if (vo.skip||vo.forcedSkip)
					{

						defectName = "Skip";

					}
					else
					{
						defectName = ignoreDefectName;
					}

					QVector<ct::AlgoDefectResult> resList;
					ct::AlgoDefectResult res;
					res.enableCustomTagName = true;
					res.customTagName = defectName;
					res.optic_id = _mainOptics.id.toStdString();

					res.vo_id = vo.objectID.toStdString();
					res.vo_name = vo.objectName.toStdString();
					res.vo_x = vo.rect.x();
					res.vo_y = vo.rect.y();
					res.vo_w = vo.rect.width();
					res.vo_h = vo.rect.height();

					res.algo_id = "None";

					uidGenerator uidGen;
					res.def_id = QString("def" + QString(uidGen.id().c_str())).toStdString();
					res.def_name = defectName.toStdString();
					res.def_x = 0;
					res.def_y = 0;
					res.def_w = vo.rect.width();
					res.def_h = vo.rect.height();

					if (vo.skip||vo.forcedSkip) res.skip = true;
					else res.ignore = true;


					QView view;

					if (!_views.contains(vo.viewID)) {
						ct::logger::error("[InspThread] Failed to ignore defect. Invalid view ID: %s", vo.viewID.toStdString().c_str());
						continue;
					}

					auto v = _views.find(vo.viewID);
					if (v != _views.end()) view = _views.find(vo.viewID).value();

					resList.append(res);
					addDefects(resList, view, false);
				}
				break;
			}
		}
		vor++;
	}
	return false;
}

void InspectionThread::VidiNodeInit()
{
	if (!_isVidiInit)
	{
		if (vidiInit())
		{
			_isVidiInit = true;
			_pErrorInfo->_isRunFail = false;
			strToChars(_pErrorInfo->_error, 1024, "No error");
		}
		else
		{
			_pErrorInfo->_isRunFail = true;
			strToChars(_pErrorInfo->_error, 1024, _errorMsg.toStdString());
		}
	}
	else
	{
		_isVidiInit = true;
		_pErrorInfo->_isRunFail = false;
		strToChars(_pErrorInfo->_error, 1024, "No error");
	}

	SetEvent(_appEvents.getEvent(std::string("VidiNodeReturn")));
}

void InspectionThread::VidiNodeOpenWorkspace()
{
	if (_isVidiInit)
	{
		if (openWorkspace())
		{
			_pErrorInfo->_isRunFail = false;
			strToChars(_pErrorInfo->_error, 1024, "No error");
		}
		else
		{
			_pErrorInfo->_isRunFail = true;
			strToChars(_pErrorInfo->_error, 1024, _errorMsg.toStdString());
		}
	}
	else
	{
		_pErrorInfo->_isRunFail = true;
		strToChars(_pErrorInfo->_error, 1024, "VIDI not yet initialised");
	}

	SetEvent(_appEvents.getEvent(std::string("VidiNodeReturn")));
}

void InspectionThread::SoftTriggerProductionInspection()
{
	mainInspectionLoop(PRODUCTION);
}

void InspectionThread::SoftTrigger()
{
	// Non production

	// contain current image info _pFrameInfo
	_frameCount++;
	if (_frameCount == 1)
	{
		_redFrameInfo = *_pFrameInfo;
		return;
	}
	else if (_frameCount == 2)
	{
		_greenFrameInfo = *_pFrameInfo;
		return;
	}
	else if (_frameCount == 3)
	{
		_blueFrameInfo = *_pFrameInfo;
	}

	// start vidi inspection
	{
		if (!_workspaceOpened.isEmpty())
		{
			_isRunFail = !vidiInspectionFromMemory(&_redFrameInfo, &_greenFrameInfo, &_blueFrameInfo);
		}
		else
		{
			_isRunFail = true;
			_errorMsg = QStringLiteral("Workspace not yet opened");
		}
	}

}

void InspectionThread::VidiNodeRunImage()
{
	qDebug() << "Running vidi Node run Image";
	qDebug() << "OpenedWorkSpace: " << _workspaceOpened;


	//if (_isWorkspaceOpened)
	if (false)
	{
		//deprecated
		QVector<QStringList> imgPreprocessPaths;
		preprocessFromFiles(_imgPaths, _lightingIDs, imgPreprocessPaths);
		vidiInspectionFromFiles(imgPreprocessPaths, _lightingIDs);
	}

	if (!_templateODOpticsPath.isEmpty())
	{
		qDebug() << "runODInspection";
		objectDetectionInspectionFromFiles(_imgPaths, _lightingIDs, _templateODOpticsPath);
	}

	qDebug() << "Vidi Message: " << _errorMsg;

	SetEvent(_appEvents.getEvent(std::string("VidiNodeReturn")));
}

void InspectionThread::AIWireBondInspectionRunImage()
{
	qDebug() << "AIWireBondInspectionRunImage";

	//algoGraph
	//samInspectionFromFiles(_imgPaths, _lightingIDs, _runImageAlgoGraph, _runImageOffset, _runImageAngles);
	wirebondInspectionFromFiles(_imgPaths, _lightingIDs, _runImageAlgoGraph, _runImageOffset, _runImageAngles);	

	_runImageAlgoGraph = nullptr;

	SetEvent(_appEvents.getEvent(std::string("VidiNodeReturn")));
}

QPointF InspectionThread::getAlgoLocatorOffset(const QJsonObject& algoObj, const QHash<QString, QPointF>& offsets)
{
	QString locID = jsonHelper::getString(algoObj, QStringLiteral("Locator_ID"));
	QPointF offset = { 0,0 };
	if (offsets.contains(locID))
	{
		offset = offsets.value(locID);
	}
	else
	{
		if (offsets.size() > 0)
		{
			offset = offsets.begin().value();
		}
	}
	return offset;
}

void InspectionThread::OfflineHybridInspection()
{
	mainInspectionLoop(OFFLINE);
}

void InspectionThread::GenerateVidiWorkspaceInfo()
{
	QVector<StreamInfo> streamInfos;

	auto optic = _recipeOptics.constBegin();
#if HAS_VIDI_LICENSE
	while (optic != _recipeOptics.constEnd())
	{
		if (optic.value().name == "RGB")
		{
			QVector<ToolInfo> toolInfos;
			QString opticWithPreprocess = optic.value().name + "_HighlightDefects";
			bool streamExist = checkVidiStreamExist(opticWithPreprocess);
			if (streamExist)
			{
				getVidiTools(toolInfos, opticWithPreprocess);
				getVidiLabels(toolInfos, opticWithPreprocess);
			}

			StreamInfo streamInfo;
			streamInfo._streamName = opticWithPreprocess;
			streamInfo._toolInfos = toolInfos;
			streamInfos.append(streamInfo);
		}
		else if (optic.value().name == "RB")
		{
			QVector<ToolInfo> toolInfos;
			QString opticWithPreprocess = optic.value().name + "_DiffOfMedianFilter";
			bool streamExist = checkVidiStreamExist(opticWithPreprocess);
			if (streamExist)
			{
				getVidiTools(toolInfos, opticWithPreprocess);
				getVidiLabels(toolInfos, opticWithPreprocess);
			}

			StreamInfo streamInfo;
			streamInfo._streamName = opticWithPreprocess;
			streamInfo._toolInfos = toolInfos;
			streamInfos.append(streamInfo);
		}
		else if (optic.value().name == "DieLight")
		{
			QVector<ToolInfo> toolInfos;
			QString opticWithPreprocess = optic.value().name + "_DiffOfMedianFilter";
			bool streamExist = checkVidiStreamExist(opticWithPreprocess);
			if (streamExist)
			{
				getVidiTools(toolInfos, opticWithPreprocess);
				getVidiLabels(toolInfos, opticWithPreprocess);
			}

			StreamInfo streamInfo;
			streamInfo._streamName = opticWithPreprocess;
			streamInfo._toolInfos = toolInfos;
			streamInfos.append(streamInfo);
		}

		QVector<ToolInfo> toolInfos;
		bool streamExist = checkVidiStreamExist(optic.value().name);
		if (streamExist)
		{
			getVidiTools(toolInfos, optic.value().name);
			getVidiLabels(toolInfos, optic.value().name);
		}

		StreamInfo streamInfo;
		streamInfo._streamName = optic.value().name;
		streamInfo._toolInfos = toolInfos;
		streamInfos.append(streamInfo);
		optic++;
	}
#endif
	StreamInfo s;
	QString fileName = Common::Directory::getRecipeCurrentPath() + "VIDI_WorkSpaceInfos.json";
	s.serializeStreamInfos(fileName, streamInfos);

	SetEvent(_appEvents.getEvent(std::string("VidiNodeReturn")));
}

void InspectionThread::VidiNodeDeInit()
{
	// Close workspace & deinitialize Vidi
#if HAS_VIDI_LICENSE
	if (!_workspaceOpened.isEmpty())
	{
		vidi_runtime_close_workspace(_workspaceOpened.toStdString().c_str());
		_workspaceOpened.clear();
		_isWorkspaceOpened = false;
	}

	if (_isVidiInit)
	{
		vidi_deinitialize();
		_isVidiInit = false;
	}
#endif
	SetEvent(_appEvents.getEvent(std::string("VidiNodeReturn")));
}

void InspectionThread::mainInspectionLoop(int mode)
{
	int viewIndex = 0;
	bool doneInspection = false;
	_measArray = QJsonArray();
	bool stopLooping = false;
	_numOfViewReceived = 0;
	_emptyPocketIndex = 1;

	ct::logger::info("[QThread] Main inspection loop started");
	ct::logger::info("[InspectionThread] Inspection mode: %d", mode);

	const int EMIT_INTERVAL_MS = 50; // 20 Hz
	QElapsedTimer progressTimer;
	progressTimer.start();

	while (_scanInspBufferQueue || g_inspectionQueue.size() > 0)
	{
		//ct::logger::error("Scan Insp: %d, Insp Queue: %d", _scanInspBufferQueue, g_inspectionQueue.size());
		//ct::logger::debug("Start Inspection Queue");

		//os_tool::doNothing(1);

		if (g_inspectionQueue.size() == 0) continue;

		MachineController::instance().trackTime("Inspection");

		Timer singleViewInspectionTime;

		//clearLocator
		if (g_viewMode == (int)ViewMode::SINGLE)  clearLocatorResults();

		//get Images
		QVector<FrameInfo> frameInfos;
		g_inspectionQueue.get(frameInfos);

		//get ViewID
		QString viewID = frameInfos[0].viewID;
		ct::logger::info("[InspectionThread] Received inspection request: %s", viewID.toStdString().c_str());

		//getUnitResultInfos
		_unitResultInfos.clear();
		for (int i = 0; i < frameInfos.size(); i++)
		{
			ct::UnitResultInfo info;
			info.opticID = frameInfos[i].opticID;
			info.viewID = frameInfos[i].viewID;
			info.indexID = QString::number(frameInfos[i].index);

			_unitResultInfos.insert(info.opticID, info);
		}

		//convert frameInfos to MIL
		QVector<MIL_ID> milImgs;
		for (int i = 0; i < frameInfos.size(); i++)
		{
			auto frameInfo = frameInfos[i];

			QString imgType = frameInfo.type;

			if (imgType == ct::s_height_map) milImgs.append(frameInfo.pHeightMap->id());
			else if (imgType == ct::s_mono) milImgs.append(frameInfo.pImage->id());
			else if (imgType == ct::s_color || _camChannel == 3) milImgs.append(frameInfo.pImage->id());
		}

		//get milImgIMap
		MIL_ID milImgIMap = M_NULL;
		for (int i = 0; i < frameInfos.size(); i++)
		{
			auto frameInfo = frameInfos[i];
			QString imgType = frameInfo.type;
			if (imgType == ct::s_height_map)
			{
				milImgIMap = frameInfo.pImage->id();
				break;
			}
		}

		//get OpticsInfos2D3D
		for (int i = 0; i < frameInfos.size(); i++)
		{
			auto frameInfo = frameInfos[i];
			QString imgType = frameInfo.type;
			QString opticID = frameInfo.opticID;
			QString opticName = _recipeOptics[opticID].name;

			OpticsInfo o;
			if (imgType == ct::s_height_map)
			{
				if (!alloptics2D3D_exist(opticID))
				{
					o.type = imgType;
					o.name = opticID;
					_allOptics_2D_3D.append(o);
				}

				if (!alloptics2D3D_exist("IntensityMap"))
				{
					OpticsInfo o;
					o.type = ct::s_intensity_map;
					o.name = "IntensityMap";
					_allOptics_2D_3D.append(o);
				}
			}
			else
			{
				if (!alloptics2D3D_exist(opticName))
				{
					OpticsInfo o;
					o.type = imgType;
					o.name = opticName;
					_allOptics_2D_3D.append(o);
				}
			}
		}

		//COMMENT BY KEAN
		//load Images to Algo
		bool is3D = false;
		QVector<MilImageInfo> milImageInfos;
		for (int i = 0; i < frameInfos.size(); i++)
		{
			auto frameInfo = frameInfos[i];
			MilImageInfo milImageInfo;
			milImageInfo.imgSize = QSize(frameInfo.width, frameInfo.height);

			milImageInfo._opticID = frameInfo.opticID;
			milImageInfo._opticName = _recipeOptics[frameInfo.opticID].name;
			milImageInfo._imageType = frameInfo.type;

			ct::logger::info("Loading Image to Algo...Image ID: %s, Optic: %s",
				frameInfo.viewID.toStdString().c_str(), frameInfo.opticID.toStdString().c_str());

			auto band = mtrx::get_band(frameInfo.pImage->id());

			if (milImageInfo._imageType == ct::s_height_map) {
				auto optID = frameInfo.opticID;
				QString name = "";

				if (!frameInfo.baseOpticID.isEmpty()) {
					optID = frameInfo.baseOpticID;
					name = _recipeOptics3D[optID].name + frameInfo.opticID.right(2);
				}
				else {
					name = _recipeOptics3D[optID].name;
				}

				milImageInfo._opticName = name;
				milImageInfo._pHeightMap = frameInfo.pHeightMap->id();
				milImageInfo._pIntensityMap = frameInfo.pImage->id();
				is3D = true;
			}
			//else if (milImageInfo._imageType == ct::s_mono) {
			else if (band == 1) {
				milImageInfo._pMonoImageBuf = frameInfo.pImage->id();
			}
			//else if (milImageInfo._imageType == ct::s_color || _camChannel == 3) {
			else if (band == 3) {
				if (frameInfo.pImage == M_NULL)
				{
					ct::logger::info("Null ptr detected");
				}
				//SEETHIS:
				milImageInfo._pRedImageBuf = mtrx::extract_channel(frameInfo.pImage->id(), mtrx::Channel::RED);
				milImageInfo._pGreenImageBuf = mtrx::extract_channel(frameInfo.pImage->id(), mtrx::Channel::GREEN);
				milImageInfo._pBlueImageBuf = mtrx::extract_channel(frameInfo.pImage->id(), mtrx::Channel::BLUE);
				ct::logger::info("MbufChildColor done");
			}
			milImageInfos.append(milImageInfo);
		}

		//get imgType, img width and img Height;
		QString mainImgType = ct::s_color;
		_imgWidth = 0;
		_imgHeight = 0;
		for (int i = 0; i < frameInfos.size(); i++)
		{
			auto frameInfo = frameInfos[i];
			mainImgType = frameInfo.type;
			_imgWidth = frameInfo.width;
			_imgHeight = frameInfo.height;
			break;
		}

		_algo->loadImages(milImageInfos); //X3D: Make sure here get images

		//get Vision Objects
		QHash<QString, QRectF> visionObjectsRect;
		if (!is3D) getVisionObjects(visionObjectsRect, viewID);
		else getVisionObjectsForHeightMap(visionObjectsRect, viewID);
		//get Vision Objects

		_cycleTime = 0;
		_isRunFail = false;
		_inspResult = true;
		_resultBufferData.clear();
		_errorMsg = QStringLiteral("No error");

		//start ignore defect generation
		if (true)
		{
			if (generateIgnoreDefect(visionObjectsRect, "Incoming")) {
				ct::logger::info("Generate Ignore Defect");
			}
		}

		//pre OD
		if (!is3D && !_dryRun)
		{

			Timer timeOD;
			//_isRunFail = !objectDetectionFromMemory(visionObjectsRect, _locatorOffsets, frameInfos, milImgs, true);
			ct::logger::info("Pre Object Detection inspection duration: %fs", timeOD.duration());
		}

		// start locator inspection

		if (!is3D && !_dryRun)
		{
			MachineController::instance().trackTime("Locator");
			Timer time;
			_isRunFail = !locatorInspection(visionObjectsRect, _locatorOffsets,_locatorAngles, viewID);
			ct::logger::info("Locator inspection duration: %fs", time.duration());
			MachineController::instance().logTime("Locator");
		}

		// start vidi inspection
		if (!is3D && !_dryRun)
		{
			MachineController::instance().trackTime("AI");

			Timer time;
			_isRunFail = !objectDetectionFromMemory(visionObjectsRect, _locatorOffsets, frameInfos, milImgs);
			ct::logger::info("Object Detection inspection duration: %fs", time.duration());
			if (!_workspaceOpened.isEmpty())
			{
				Timer time;
#if HAS_VIDI_LICENSE
				_isRunFail = !vidiInspectionFromMemory(visionObjectsRect, _locatorOffsets, frameInfos, milImgs);
#endif
				// here Run Onnx Model


				ct::logger::info("VIDI inspection duration: %fs", time.duration());
			}
			else
			{
				_isRunFail = true;
				_errorMsg = QStringLiteral("Workspace not yet opened");
			}

			MachineController::instance().logTime("AI");
		}

		//start AI Segmentation
		if (!is3D && !_dryRun && _enableSegmentation)
		{		
			Timer time;
			ct::logger::info("Start Wire bond AI Inspection");
			//_isRunFail = !wirebondInspectionPointsFromMemory(visionObjectsRect, _locatorOffsets, _locatorAngles, frameInfos, milImgs);
			ct::logger::info("Wire bond AI Inspection duration: %fs", time.duration());
		}

		//start post inspection
		if (!_dryRun)
		{
			MachineController::instance().trackTime("Algo");
			Timer time;
			ct::logger::info("Start post inspection");
			if(SystemData::instance()._enable_multi_thread)_isRunFail = !multi_postInspection_v2(visionObjectsRect, _locatorOffsets, mainImgType);
			else _isRunFail = !postInspection(visionObjectsRect, _locatorOffsets, mainImgType);
			ct::logger::info("Post inspection duration: %fs", time.duration());
			MachineController::instance().logTime("Algo");
		}

		if (true)
		{
			Timer time;
			cadDefectMapping(visionObjectsRect, _locatorOffsets);
		
		}

		ct::logger::debug("Single View Inspection duration: %fs", singleViewInspectionTime.duration());
		setUnitInspectionTime(singleViewInspectionTime.duration());


		//save All Vision Object Images
		if (_saveDefectVoImg && !_dryRun) saveAllVisionObjectImages(visionObjectsRect, _locatorOffsets, frameInfos, milImgs, milImgIMap);


		//collect Image
		if (_pInspectionInfo->_isCollectImage && _saveInspImg)
		{
			for (auto frameInfo : frameInfos) {

				auto root = Common::Directory::getProductionImageSetPath().toStdString();

				QString id = frameInfo.viewID;
				QString extension = g_imgExtension;

				if (is3D) {
					extension = ".tiff";
					id = frameInfo.viewID + "_HeightMap";
				}

				QString defectCollectorImagePath = "none";
				if (_defectCollectorPath.contains("C:/"))
				{
					defectCollectorImagePath = "None";
				}
				else
				{

					QString defectCollectorImageDir = _defectCollectorPath + "/" + Common::Directory::productionName + "/Images/";
					QDir destinationFolder(defectCollectorImageDir);
					if (!destinationFolder.exists()) {

						if (!destinationFolder.mkpath(".")) {
							ct::logger::error("Failed to create defectCollectorImageDir folder: %s", defectCollectorImageDir.toStdString().c_str());

						}
						else
						{
							defectCollectorImagePath = defectCollectorImageDir + "/" + id + "_" + frameInfo.opticID + extension;
						}
					}
					else
					{
						defectCollectorImagePath = defectCollectorImageDir + "/" + id + "_" + frameInfo.opticID + extension;
					}

				}

				IST::instance().enqueue(root, frameInfo, defectCollectorImagePath.toStdString());
				
				if (!is3D && _saveBMPImg) IST::instance().enqueue(root, frameInfo, defectCollectorImagePath.toStdString(), "bmp");
			}
		}

		//free child buffer in algo
		_algo->releaseBuffer();
		for (auto& milImageInfo : milImageInfos) {
			if (milImageInfo._imageType == ct::s_color || _camChannel == 3) {
				MbufFree(milImageInfo._pRedImageBuf);
				MbufFree(milImageInfo._pGreenImageBuf);
				MbufFree(milImageInfo._pBlueImageBuf);
			}
		}

		//clear Segmentation Results
		_segmentationResult.clear();

		//UNCOMMENT HERE

		QDateTime startInspTime = QDateTime::fromString(_frameID, QStringLiteral("dd-MM-yyyy hh-mm-ss-zzz"));
		QDateTime endInspTime = QDateTime::currentDateTime();
		_cycleTime = startInspTime.msecsTo(endInspTime);

		if (progressTimer.elapsed() >= EMIT_INTERVAL_MS)
		{
			emit updateInspectionProgressBar();
			progressTimer.restart();
		}
		viewIndex++;

		_viewsToRun.remove(viewID);


		bool conditionMet = false;
		if (_countMode == CountMode::VIEW) {
			ct::logger::info("View index: %d, GView index: %d, Remaining views to run: %d", viewIndex, g_viewIndex, _viewsToRun.size());
			conditionMet = (viewIndex == g_viewIndex);
		}

		MachineController::instance().logTime("Inspection");

		if (conditionMet || g_forceStopInspLoop)
		{

			g_viewIndex = 0;
			viewIndex = 0;

			_scanInspBufferQueue = false;
			doneInspection = true;

			//ClassifyDefectImages
			classifyDefectImages();

			//saveResult
			ct::ResultSerializer resSerializer(_algoDefectResults, _defectResults, _distanceMeasurementVector);

			QString resultDir = Common::Directory::getProductionResultPath() + "/ResultsJson/";
			QDir dir(resultDir);
			if (!dir.exists())
				dir.mkpath(resultDir);
			//resSerializer.serializeDefectResult(QString(Common::Directory::getProductionResultPath() + "result.json"));
			resSerializer.serializeDefectResult(resultDir);
			resSerializer.serializeDistanceMeasurementResult(QString(Common::Directory::getProductionResultPath() + "distanceMeasurement.json"));

			auto measResultPath = Common::Directory::getProductionResultPath() + "measurement.json";
			QJsonDocument doc(_measArray);
			bool flag = false;

			QFile file(measResultPath);
			if (file.open(QIODevice::WriteOnly))
			{
				file.write(doc.toJson());
				file.flush();
				file.close();
			}

			SystemData::instance()._InspectionCompleted = false;
			if (g_forceStopInspLoop)
			{
				stopLooping = true;
				g_forceStopInspLoop = false;
				g_inspectionQueue.clear();
				ct::logger::info("Inspection Forced Stop");
			}
			else
			{
				SystemData::instance()._InspectionCompleted = true;
				g_inspectionQueue.clear();
				ct::logger::info("Inspection Completed");
			}

			_isOffline = false;

			/*for (const QString& key : _dataResult.keys()) {
				ct::logger::info("key: %s", key.toStdString().c_str());
			}

			for (auto dataResult : _dataResult)
			{
				ct::logger::info("dataResultQuantity: %d", dataResult.size());
				for (auto dr : dataResult)
				{
					QString className = dr.dataObj.value("ClassName").toString();
					ct::logger::info("DataID: %s, DataName: %s, ClassName: %s", dr.data_id.toStdString().c_str(), dr.data_name.toStdString().c_str(), className.toStdString().c_str());
				}
			}*/


			emit inspectionDone(_defectResults, _barcodeDecoderInfoVector);
		}
	}

	if (!stopLooping) {
		ct::logger::info("[InspectionThread] Emit run looping");
		emit runLooping();
	}
	if (!doneInspection) SetEvent(_appEvents.getEvent(std::string("VidiNodeReturn")));
}

bool InspectionThread::loadJson(QString path, QJsonObject& root)
{
	QString val;
	QFile file;
	QJsonDocument doc;

	if (QFile::exists(path))
	{
		file.setFileName(path);
		if (file.open(QIODevice::ReadOnly | QIODevice::Text))
		{
			val = file.readAll();
			file.close();

			doc = QJsonDocument::fromJson(val.toUtf8());
			root = doc.object();

			return true;
		}
	}

	return false;
}

std::unordered_map<std::string, std::list<cv::Vec6f>> InspectionThread::removeOutOfBoundLinesMap(
	std::unordered_map<std::string, std::list<cv::Vec6f>> linesMap, QSize imageSize)
{
	std::unordered_map<std::string, std::list<cv::Vec6f>> filteredMap;

	int imgWidth = imageSize.width();
	int imgHeight = imageSize.height();

	for (auto& [key, lineList] : linesMap)
	{
		std::list<cv::Vec6f> filteredLines;

		for (const auto& line : lineList)
		{
			float x1 = line[0];
			float y1 = line[1];
			float x2 = line[2];
			float y2 = line[3];
			float bondSize = line[4];

			// Check if both endpoints are within the image boundaries
			bool inBounds = (x1 >= 0 && x1 < imgWidth &&
				y1 >= 0 && y1 < imgHeight &&
				x2 >= 0 && x2 < imgWidth &&
				y2 >= 0 && y2 < imgHeight);

			if (inBounds)
			{
				filteredLines.push_back(line);
			}
		}

		if (!filteredLines.empty())
		{
			filteredMap[key] = std::move(filteredLines);
		}
	}

	return filteredMap;
}

std::vector<LineData> InspectionThread::removeOutOfBoundLineData(std::vector<LineData> linesData, QSize imageSize)
{
	std::vector<LineData> filteredLineData;

	int imgWidth = imageSize.width();
	int imgHeight = imageSize.height();

	for (auto& lineData : linesData)
	{
		auto line_point2d = lineData.points;
		std::vector<cv::Point> line;
		line.reserve(line_point2d.size());
		for (const auto& pt : line_point2d) {
			line.emplace_back(cv::Point(cvRound(pt.x), cvRound(pt.y)));  // rounding to nearest integer
		}

		cv::Rect boundingRect = cv::boundingRect(line);

		if ((boundingRect & cv::Rect(0, 0, imgWidth, imgHeight)) == boundingRect) {
			// Safe to use boundingRect
			filteredLineData.push_back(lineData);
		}
		else {
			std::cout << "Warning: boundingRect is out of image bounds: " << boundingRect << std::endl;
		}
	}

	return filteredLineData;
}

void InspectionThread::stopRun()
{
	for (auto id : _viewsToRun) {
		ct::logger::trace("Remaining view to run: %s", id.toStdString().c_str());
	}

	stopScanInspBufferQueue();

}

void InspectionThread::initMultiThread()
{
	// Check if OpenMP is enabled
#ifdef _OPENMP
	std::cout << "OpenMP is supported! Version: " << _OPENMP << std::endl;
#else
	std::cout << "OpenMP is not supported!" << std::endl;
#endif

	printf("ideal thread count: %d \n", QThread::idealThreadCount());
	// Set the number of threads
	omp_set_num_threads(SystemData::instance()._num_threads);

	// Parallel region
#pragma omp parallel
	{
		int thread_id = omp_get_thread_num();
		int num_threads = omp_get_num_threads();
		std::cout << "Hello from thread " << thread_id << " out of " << num_threads << std::endl;
	}

}

void InspectionThread::getRedToolHeatMap(QString uuid, QString heatMapImagePath)
{
#if HAS_VIDI_LICENSE
	VIDI_BUFFER buffer;
	VIDI_UINT status;
	VIDI_IMAGE vidiHeatMapImg;

	status = vidi_init_image(&vidiHeatMapImg);

	status = vidi_init_buffer(&buffer);

	status = vidi_resource_get_image(uuid.toStdString().c_str(), &vidiHeatMapImg);

	status = vidi_save_image(heatMapImagePath.toStdString().c_str(), &vidiHeatMapImg);

	//qDebug() << "channels:" << vidiHeatMapImg.channels;
	//for (size_t y = 0; y < vidiHeatMapImg.height; y++)
	//{
	//	for (size_t x = 0; x < vidiHeatMapImg.width; x++)
	//	{
	//			// Get the original pixel value
	//			uint16_t* pixel_ptr = (uint16_t*)vidiHeatMapImg.data + y *  vidiHeatMapImg.step + x *  vidiHeatMapImg.channels;

	//			qDebug() << "x:" << x << " y:" << y << "value:" << pixel_ptr[0];
	//	}
	//}

	vidi_get_error_message(status, &buffer);
	QString errorMsg;
	errorMsg = QStringLiteral("Fail to compile heat map image, error message: %1").arg(QString(buffer.data));

	// free resources
	status = vidi_free_buffer(&buffer);

	status = vidi_free_image(&vidiHeatMapImg);
#endif
}

InspectionThread::~InspectionThread()
{
}

void InspectionThread::setHeightMap(const QHash<QString, QLineScan>& heightMaps)
{
	_heightMaps = heightMaps;
}


void InspectionThread::cadDefectMapping(const QHash<QString, QRectF>& visionObjectsRect, const QHash<QString, QHash<QString, QPointF>>& locatorOffsets)
{
	QHash<QString, QRectF>::const_iterator vor = visionObjectsRect.constBegin();
	while (vor != visionObjectsRect.constEnd())
	{
		//get cadRois
		QVector<CadRoiInfo> cadRois;
		for (int i = 0; i < _dragROI.size(); i++)
		{
			if (_dragROI[i]->getName() == vor.key())
			{
				auto algoGraph = _dragROI[i]->algoGraph();
				if (!algoGraph)
				{
					ct::logger::error("No AlgoGraph %s, viewID: %s", _dragROI[i]->getName().toStdString().c_str(), _dragROI[i]->viewID().toStdString().c_str());
				}
				else
				{
					auto c = _templateCadRois.find(algoGraph->templateId());
					if (c != _templateCadRois.end()) cadRois = c.value();
					break;
				}
			}
		}

		bool pass = true;
		for (int i = 0; i < _defectResults.size(); i++)
		{
			//temp hardcode do not add defect cad mapping if these 4 defects appear
			if (_defectResults[i].algoDefResult.customTagName == "Incoming" || _defectResults[i].algoDefResult.customTagName == "Skip"
				|| _defectResults[i].algoDefResult.customTagName == "Locator Not Found" || _defectResults[i].algoDefResult.customTagName == "Die Shear") continue;

			if (_defectResults[i].algoDefResult.vo_name == vor.key().toStdString())
			{

				pass = false;
				QRectF defRect(_defectResults[i].algoDefResult.def_x, _defectResults[i].algoDefResult.def_y, _defectResults[i].algoDefResult.def_w, _defectResults[i].algoDefResult.def_h);

				//get first value of loc offset for all locator hashMap
				QHash<QString, QPointF> offsets;
				QHash<QString, QHash<QString, QPointF>>::const_iterator lo = locatorOffsets.constBegin();
				while (lo != locatorOffsets.constEnd())
				{
					if (lo.value().size() > 0) offsets.insert(lo.key(), lo.value().begin().value());
					++lo;
				}
				removeLocOffsetfromRect(offsets, vor.key(), defRect);

				double totalIntersectPercentage = 0;

				//cadClassification			
				for (int j = 0; j < cadRois.size(); j++)
				{
					//calculate percentage
					auto cadRoi = cadRois[j];
					QRectF cadRect(cadRoi.x, cadRoi.y, cadRoi.w, cadRoi.h);

					// -- intesect percentage --//
					//QRectF intersectedRect = defRect.intersected(cadRect);
					//double intersectArea = intersectedRect.width() * intersectedRect.height();
					//double cadArea = cadRoi.w * cadRoi.h;
					//double defectArea = defRect.width() * defRect.height();

					//if (cadArea == 0) cadArea = 1;
					//double percentage = intersectArea / cadArea * 100;
					//totalIntersectPercentage = totalIntersectPercentage + (intersectArea) / defectArea * 100;

					////if percentage exceed threshold
					//double cadThreshold = 50;
					//if (percentage > cadThreshold)
					//{
					//
					//	_defectResults[i].cadRoiInfos.append(cadRoi);

					//	cadTagNameMapping(_defectResults[i], cadRoi.familyName);
					//	
					//}
					// -- intersect percentage

					// -- now use center point inclusion
					if (cadRect.contains(defRect.center()))
					{
						_defectResults[i].cadRoiInfos.append(cadRoi);
						cadTagNameMapping(_defectResults[i], cadRoi.familyName);

					}
					// -- now use center point inclusion

				}

			}	//cad classification - end
		}
		vor++;
	}
}

bool InspectionThread::objectDetectionFromMemory(const QHash<QString, QRectF>& visionObjectsRect,
	const  QHash<QString, QHash<QString, QPointF>>& locatorOffsets, QVector<FrameInfo>& pFrameInfos,
	QVector<MIL_ID>& milImgs, bool locatorODFlag)
{
	bool flag = true;

	//_objectDetectionResult.clear();

	//get number of templates used [templateID][odModelOptics]
	QHash<QString, QHash<QString, QVector<AiModelInfo::opticInfo>>> odmodelOpticList;
	for (auto dragRoi : _dragROI)
	{
		if (dragRoi->algoGraph())
		{
			QHash<QString, QRectF>::const_iterator vor = visionObjectsRect.constBegin();
			while (vor != visionObjectsRect.constEnd())
			{
				if (dragRoi->getName() == vor.key())
				{
					if (dragRoi->algoGraph())
					{
						QString templateID = dragRoi->algoGraph()->templateId();
						QString templateODOpticsPath = dragRoi->algoGraph()->ObjectDetectionModelSettingsFilePath();

						if (!odmodelOpticList.contains(templateID))
						{
							loadObjectDetectionModelSettings(templateODOpticsPath);
							odmodelOpticList.insert(templateID, _odModelsOptics);
						}

					}
				}
				vor++;
			}
		}
	}



	auto odModelOptics = odmodelOpticList.constBegin();
	while (odModelOptics != odmodelOpticList.constEnd())
	{
		//1.perform OD for current templateID
		QString templateID = odModelOptics.key();
		QStringList totalVoNames;

		//2.getTotalLightingIDs_LocatorIDs that will be used
		QStringList totalLightingNames_LocatorIDs_algoTypes;
		for (auto& opticInfo : odModelOptics.value())
		{
			for (auto o : opticInfo)
			{

				if ((o.algoType != "2" && locatorODFlag == false) || (o.algoType == "2" && locatorODFlag == true))
				{
					QString lightingName_LocatorID_algoType = o.opticName + "[@]" + o.locatorId + "[@]" + o.algoType;
					if (!totalLightingNames_LocatorIDs_algoTypes.contains(lightingName_LocatorID_algoType)) totalLightingNames_LocatorIDs_algoTypes.append(lightingName_LocatorID_algoType);
				}
			}
		}

		//3.Form cvVoImageMaps using lighting ID and locatorID
		QVector<std::unordered_map<std::string, cv::Mat>> cvVoImageMaps;
		cvVoImageMaps.resize(totalLightingNames_LocatorIDs_algoTypes.size());
		QHash<QString, QRectF>::const_iterator vor = visionObjectsRect.constBegin();
		while (vor != visionObjectsRect.constEnd())
		{
			//get current voID
			QString voID;
			for (int i = 0; i < _dragROI.size(); i++)
			{
				if (_dragROI[i]->algoGraph() == nullptr) continue;

				if (_dragROI[i]->getName() == vor.key() && _dragROI[i]->algoGraph()->templateId() == templateID)
				{
					voID = _dragROI[i]->getId();
					break;
				}
			}

			//logic for ignore vision Object
			if (!_visionObjects.find(voID).value().ignore && !_visionObjects.find(voID).value().skip && !_visionObjects.find(voID).value().forcedSkip)
			{
				for (int i = 0; i < pFrameInfos.size(); i++)
				{
					QString opticName = _recipeOptics[pFrameInfos[i].opticID].name;

					for (int j = 0; j < totalLightingNames_LocatorIDs_algoTypes.size(); j++)
					{
						QStringList idList = totalLightingNames_LocatorIDs_algoTypes[j].split("[@]");
						QString curOpticName = idList[0];
						QString curLocatorID = idList[1];
						QString curAlgoType = idList[2];

						//get current vo offset
						QHash<QString, QPointF> offsets;
						QHash<QString, QHash<QString, QPointF>>::const_iterator lo = locatorOffsets.constBegin();
						while (lo != locatorOffsets.constEnd())
						{
							if (lo.value().size() > 0 && lo.value().contains(curLocatorID))
							{
								offsets.insert(lo.key(), lo.value()[curLocatorID]);
							}
							++lo;
						}


						QRect voRect = vor.value().toRect();
						QString voName = vor.key();
						//ct::logger::info("vo before locator offset: %s, xywh: %d, %d, %d, %d", voName.toStdString().c_str(), voRect.x(), voRect.y(), voRect.width(), voRect.height());
						offsetVisionObject(offsets, vor.key(), voRect);
						//ct::logger::info("vo after locator offset: %s, xywh: %d, %d, %d, %d", voName.toStdString().c_str(), voRect.x(), voRect.y(), voRect.width(), voRect.height());
						if (curOpticName == opticName)
						{
							auto& cvVoImageMap = cvVoImageMaps[j];
							cv::Mat cvVoImage;
							performObjectDetectionPreprocess(voRect, milImgs[i], cvVoImage);
							cvVoImageMap[voName.toStdString()] = cvVoImage;
							if (!totalVoNames.contains(voName)) totalVoNames.append(voName);
						}
					}
				}

			}
			++vor;
		}

		//4.perform objectDetection
		QVector<ODModelResults> oDModelResults;
		for (int i = 0; i < g_ODModels.size(); i++)
		{
			QHash<QString, QVector<AiModelInfo::opticInfo>>::const_iterator odOptics = odModelOptics.value().constBegin();
			while (odOptics != odModelOptics.value().constEnd())
			{
				auto modelName = odOptics.key();
				auto opticList = odOptics.value();

				if (modelName == g_ODModels[i]->getModelID().c_str())
				{
					for (int j = 0; j < opticList.size(); j++)
					{
						for (int k = 0; k < totalLightingNames_LocatorIDs_algoTypes.size(); k++)
						{
							QStringList idList = totalLightingNames_LocatorIDs_algoTypes[k].split("[@]");
							QString opticName = idList[0];
							QString locatorID = idList[1];
							QString algoType = idList[2];

							if (opticList[j].opticName == opticName && opticList[j].locatorId == locatorID && opticList[j].algoType == algoType)
							{
								ODModelResults odModelResult;
								std::unordered_map<std::string, cv::Mat> outputImages;
								getImageChannel(opticList[j].channel, opticList[j].imageRotation, cvVoImageMaps[k], outputImages);


								// when inspection Run
								if (g_odTilingSettings.enableTiling)
								{
									ct::logger::info("Production Running OD Model With Tiling: %s ", g_ODModels[i]->getModelID().c_str());
									g_ODModels[i]->ct_runModel_tiling(outputImages, 0.1, 0.5, g_odTilingSettings.tilingSize, g_odTilingSettings.tilingPaddingPerc, g_odTilingSettings.tilingIou);
									ct::logger::info("Finsihed running OD Model With Tiling");
								}
								else
								{
									QString timerMsg = QStringLiteral("OD-Timer");
									ct::ScopedTimeLogger stl(timerMsg.toStdString().c_str());
									ct::logger::info("Production Running OD Model: %s ", g_ODModels[i]->getModelID().c_str());
									g_ODModels[i]->ct_runModel(outputImages, 0.1, 0.5);
									ct::logger::info("Finsihed running OD Model");


								}


								odModelResult.modelName = g_ODModels[i]->getModelID().c_str();
								odModelResult.opticName = opticList[j].opticName;
								odModelResult.opticID = opticList[j].opticId;
								odModelResult.algoType = opticList[j].algoType;
								odModelResult.channel = opticList[j].channel;
								odModelResult.imageRotation = opticList[j].imageRotation;
								odModelResult.enableSegmentation = opticList[j].enableSegmentation;
								odModelResult.locatorId = opticList[j].locatorId;
								odModelResult.od_result = g_ODModels[i]->getResults();

								for (auto l : _recipeOptics)
								{
									if (l.name == opticList[j].opticName)
									{
										odModelResult.opticID = l.id;
										break;
									}
								}
								oDModelResults.push_back(odModelResult);

								// == sam segmentation == //
								if (opticList[j].enableSegmentation && g_segModel)
								{
									
									std::unordered_map<std::string, std::list<cv::Rect>> segRectMap;
									std::unordered_map<std::string, std::list<cv::Point>> segPointMap;
									for (const auto& it : outputImages) {
										const std::string& imgId = it.first;

										auto it = odModelResult.od_result.find(imgId);
										if (it != odModelResult.od_result.end()) {
											std::vector<OnnxResult>& results = it->second;

											std::list<cv::Rect> segRects;
											for (const auto& r : results) {
												if (r.accuracy < 0.5) continue;
												cv::Rect  rect = cv::Rect(r.x1, r.y1, r.x2 - r.x1, r.y2 - r.y1);


												// --- Shrink height ---
												float hScale = 0.8f;
												int newHeight = static_cast<int>(rect.height * hScale);
												int offsetY = (rect.height - newHeight) / 2;

												// --- Extend width ---
												float wScale = 1.05f;
												int newWidth = static_cast<int>(rect.width * wScale);
												int offsetX = (rect.width - newWidth) / 2;

												// --- Final rect ---
												cv::Rect finalRect(
													rect.x + offsetX,
													rect.y + offsetY,
													newWidth,
													newHeight
												);

												segRects.push_back(finalRect);
											}

											segRectMap[imgId] = segRects;

											// ========================= segPoints ==========================
											/*std::list<cv::Point> segPoints;
											for (const auto& r : results) {
												if (r.accuracy < 0.6) continue;
												cv::Point  point = cv::Point(r.x1 + (r.x2 - r.x1)/2, r.y1 + (r.y2 - r.y1)/2);

												segPoints.push_back(point);
											}

											segPointMap[imgId] = segPoints;*/

										}
										else {
											// Key doesn't exist → handle this case
										}


									}

									Timer time;
									for (auto& i : outputImages)
									{
										cv::cvtColor(i.second, i.second, cv::COLOR_RGB2BGR);

										// kernel size must be ODD numbers
										cv::GaussianBlur(
											i.second,        // src
											i.second,        // dst (in-place is OK)
											cv::Size(5, 5),  // kernel size
											0           // sigmaX = 0 → auto-compute
										);
									/*	std::vector<cv::Mat> channels;
										cv::split(i.second, channels);


										cv::Mat green = channels[1];
										cv::Mat green3;
										cv::merge(std::vector<cv::Mat>{ green, green, green }, green3);
										cv::GaussianBlur(green3, green3, cv::Size(5, 5), 1.2);
										i.second = green3;*/
									}
									ct::logger::info("SAM convert image duration: %fs", time.duration());

									time.reset_timer();
									//saveSegRectMapToJson(segRectMap, "VisionAppODResult.json");
									ct::logger::info("SegmentationScore:%.5f", _segmentationScore);
									g_segModel->runModel_segmentation(outputImages, segRectMap, _segmentationScore, true);

									//g_segModel->runModel_segmentation(outputImages, segPointMap, 2.0, true);
									ct::logger::info("SAM run model segmentation duration: %fs", time.duration());

									ct::logger::info("runModel_segmentation");

									time.reset_timer();
									std::unordered_map<std::string, cv::Mat> resultMasks;
									resultMasks = g_segModel->getSegmentationMaskResult();
									ct::logger::info("resultMasks");

									for (auto& m : resultMasks)
									{
										QVector<DynamicMaskObject> dynamicMaskObjects;

										QString voId = m.first.c_str();
										QString opticID = opticList[j].opticId;
										QString opticChannel = opticList[j].channel;
										DynamicMaskObject dynamicMaskObject;
										dynamicMaskObject.algo_id = "OD_Segmentation[@]" + opticID;
										dynamicMaskObject.id = "OD_Segmentation[@]" + opticID;
										dynamicMaskObject.name = "OD_Segmentation[@]" + opticID;
										dynamicMaskObject.type = AlgoType::OD_SEGMENTATION;
										dynamicMaskObject.optic_id = opticID;

										util::cv_to_Mil(m.second, dynamicMaskObject.dynamicMaskImg);

										dynamicMaskObjects.append(dynamicMaskObject);

										//QString segmentMaskName = "segmentMask_" + voId + "_" + opticID + ".jpg";
										//MbufExportA(segmentMaskName.toStdString().c_str(), M_JPEG_LOSSY, dynamicMaskObject.dynamicMaskImg);

										if (!voId.isEmpty())
										{
											auto it = _segmentationResult.find(voId);
											if (it == _segmentationResult.end()) {
												it = _segmentationResult.insert(voId, QVector<DynamicMaskObject>{});
											}
											// Append one or many:
											it.value().append(dynamicMaskObjects);

										}
									}
									ct::logger::info("SAM get Result duration: %fs", time.duration());

								}
							}
						}
					}
				}
				odOptics++;
			}
		}
		objectDetectionResultToJsonObject(oDModelResults, totalVoNames);
		++odModelOptics;
	}

	return flag;
}

bool InspectionThread::segmentationInspectionFromMemory(const QHash<QString, QRectF>& visionObjectsRect, const QHash<QString, QHash<QString, QPointF>>& locatorOffsets, const QHash<QString, QHash<QString, LocAngle>>& locatorAngles, QVector<FrameInfo>& pFrameInfos, QVector<MIL_ID>& milImgs)
{
	//loop through all VisionObjectsRect
	//get DragROI
	//get AlgoGraph
	//perform need SAM to get wirebond ROI
	//crop and input image into hashMap
	//perform sam Inspection and store the dynamic MaskObjects hashMap with voRect ID;

	//_segmentationResult.clear();

	QHash<QString, QRectF>::const_iterator vor = visionObjectsRect.constBegin();
	while (vor != visionObjectsRect.constEnd())
	{
		//get current Offset
		QHash<QString, QPointF> offsets;
		QPointF firstOffset;
		if (locatorOffsets.contains(vor.key())) {
			offsets = locatorOffsets.value(vor.key());

			if (!offsets.isEmpty()) {
				auto firstOffsetIt = offsets.begin();
				firstOffset = firstOffsetIt.value();
			}
		}

		QHash<QString, LocAngle> angles;
		LocAngle firstAngle;
		if (locatorAngles.contains(vor.key())) {
			angles = locatorAngles.value(vor.key());

			if (!angles.isEmpty()) {
				auto firstAngleIt = angles.begin();
				firstAngle = firstAngleIt.value();
			}
		}

		//get current voID
		QString voID;
		for (int i = 0; i < _dragROI.size(); i++)
		{
			if (_dragROI[i]->algoGraph() == nullptr) continue;

			if (_dragROI[i]->getName() == vor.key())
			{
				std::unordered_map<std::string, cv::Mat> images;
				std::unordered_map<double, std::unordered_map<std::string, std::list<cv::Vec6f>>> lines;

				auto algoGraph = _dragROI[i]->algoGraph();
				voID = _dragROI[i]->getId();
				
				bool needSam = algoGraph->needSAM(algoGraph->templateId(), lines, offsets, angles, false);

				if (needSam)
				{
					for (int j = 0; j < pFrameInfos.size(); j++)
					{
						std::string optic_id = pFrameInfos[j].opticID.toStdString();
						//loop rects
						for (const auto& angleEntry : lines) {  // Loop through the outer map (angle -> inner map)
							double angle = angleEntry.first;    // This is the angle
							const auto& opticMap = angleEntry.second;  // Inner map (opticID -> list of rects)

							if (opticMap.find(optic_id) != opticMap.end())
							{
								auto voRect = vor.value().toRect();
								MIL_ID croppedImg;
								MIL_INT bandSize = mtrx::get_band(milImgs[j]);

								if (bandSize == 1) croppedImg = MbufAlloc2d(M_DEFAULT, voRect.width(), voRect.height(), 8, M_IMAGE + M_PROC + M_DISP, M_NULL);
								else croppedImg = MbufAllocColor(M_DEFAULT, 3, voRect.width(), voRect.height(), 8 + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);

								MbufCopyColor2d(milImgs[j], croppedImg, M_ALL_BANDS, voRect.x(), voRect.y(), M_ALL_BANDS, 0, 0, voRect.width(), voRect.height());

								MimConvolve(croppedImg, croppedImg, M_SMOOTH);
								//QString templateImgName = pFrameInfos[0].viewID + "_templateImage.jpg";
								//MbufExportA(templateImgName.toStdString().c_str(), M_JPEG_LOSSY, croppedImg);

								cv::Mat cvVoImg;
								util::Mil_to_cv(croppedImg, cvVoImg);

								if (images.find(optic_id) == images.end()) images[optic_id] = cvVoImg;
								MbufFree(croppedImg);

							}
						}
						
					}

					auto voRect = vor.value().toRect();
					//remove out of bound rectsMap
					for (auto& angleEntry : lines) {
						auto& opticMap = angleEntry.second;  // Inner map (opticID -> list of rects)
						opticMap = removeOutOfBoundLinesMap(opticMap, QSize(voRect.width(), voRect.height()));
					}
					

					// sample data --
					//g_segModel->runModel_segmentation(images, rectsMap, _segmentationScore, true);
					g_segModel->runModel_segmentation_wireBond(images, lines, _segmentationScore, true);

					std::unordered_map<std::string, cv::Mat> resultMasks;
					resultMasks = g_segModel->getSegmentationMaskResult();

					QVector<DynamicMaskObject> dynamicMaskObjects;
					for (auto& m : resultMasks)
					{
						QString opticID = m.first.c_str();
						DynamicMaskObject dynamicMaskObject;
						dynamicMaskObject.algo_id = "WireBondInspection_1[@]" + opticID;
						dynamicMaskObject.id = "WireBondInspection_1[@]" + opticID;
						dynamicMaskObject.name = "WireBondInspection_1[@]" + opticID;
						dynamicMaskObject.type = AlgoType::WIREBOND;
						dynamicMaskObject.optic_id = opticID;
						util::cv_to_Mil(m.second, dynamicMaskObject.dynamicMaskImg);
						dynamicMaskObjects.append(dynamicMaskObject);

					/*	QString segmentMaskName = pFrameInfos[0].viewID + "_segmentMask.jpg";
						MbufExportA(segmentMaskName.toStdString().c_str(), M_JPEG_LOSSY, dynamicMaskObject.dynamicMaskImg);*/
					}


					_segmentationResult.insert(vor.key(), dynamicMaskObjects);
				}

				break;
			}
		}
		++vor;	
	}
	return false;
}

bool InspectionThread::wirebondInspectionPointsFromMemory(const QHash<QString, QRectF>& visionObjectsRect, const QHash<QString, QHash<QString, QPointF>>& locatorOffsets, const QHash<QString, QHash<QString, LocAngle>>& locatorAngles, QVector<FrameInfo>& pFrameInfos, QVector<MIL_ID>& milImgs)
{
	//loop through all VisionObjectsRect
	//get DragROI
	//get AlgoGraph
	//perform need SAM to get wirebond ROI
	//crop and input image into hashMap
	//perform sam Inspection and store the dynamic MaskObjects hashMap with voRect ID;

	//_segmentationResult.clear();

	QHash<QString, QRectF>::const_iterator vor = visionObjectsRect.constBegin();
	while (vor != visionObjectsRect.constEnd())
	{
		//get current Offset
		QHash<QString, QPointF> offsets;
		QPointF firstOffset;
		if (locatorOffsets.contains(vor.key())) {
			offsets = locatorOffsets.value(vor.key());

			if (!offsets.isEmpty()) {
				auto firstOffsetIt = offsets.begin();
				firstOffset = firstOffsetIt.value();
			}
		}

		QHash<QString, LocAngle> angles;
		LocAngle firstAngle;
		if (locatorAngles.contains(vor.key())) {
			angles = locatorAngles.value(vor.key());

			if (!angles.isEmpty()) {
				auto firstAngleIt = angles.begin();
				firstAngle = firstAngleIt.value();
			}
		}

		//get current voID
		QString voID;
		for (int i = 0; i < _dragROI.size(); i++)
		{
			if (_dragROI[i]->algoGraph() == nullptr) continue;

			if (_dragROI[i]->getName() == vor.key())
			{
				std::unordered_map<std::string, cv::Mat> images;
				std::unordered_map<std::string, std::vector<LineData>>lines;

				auto algoGraph = _dragROI[i]->algoGraph();
				voID = _dragROI[i]->getId();

				QHash<QString, MIL_ID>  croppedMilImages;
				for (int j = 0; j < pFrameInfos.size(); j++)
				{
					QString optic_id = pFrameInfos[j].opticID;
					//loop rects

					auto voRect = vor.value().toRect();
					MIL_ID croppedImg;
					MIL_INT bandSize = mtrx::get_band(milImgs[j]);

					if (bandSize == 1) croppedImg = MbufAlloc2d(M_DEFAULT, voRect.width(), voRect.height(), 8, M_IMAGE + M_PROC + M_DISP, M_NULL);
					else croppedImg = MbufAllocColor(M_DEFAULT, 3, voRect.width(), voRect.height(), 8 + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);

					MbufCopyColor2d(milImgs[j], croppedImg, M_ALL_BANDS, voRect.x(), voRect.y(), M_ALL_BANDS, 0, 0, voRect.width(), voRect.height());

					croppedMilImages[optic_id] = croppedImg;
				}

				bool needSam = algoGraph->getWireLines(algoGraph->templateId(), croppedMilImages, lines, offsets, angles, false);

				if (needSam)
				{
					for (const auto& lineData : lines)
					{
						const std::string& opticID = lineData.first;
						QString opticID_q = QString::fromStdString(opticID);

						if (!croppedMilImages.contains(opticID_q)) {
							qWarning() << "Missing croppedMilImage for opticID:" << opticID_q;
							continue;
						}

						MIL_ID milImg = croppedMilImages[opticID_q];
						cv::Mat img;
						util::Mil_to_cv(milImg, img);

						// Store the image once for segmentation input
						if (images.find(opticID) == images.end()) {
							images[opticID] = img;
						}

						//VISUALIZATION ONLY
						if (false)
						{
							// Visualize and save debug image
							cv::Mat displayImage = img.clone();
							const auto& lineList = lineData.second;

							for (const auto& line : lineList) {
								const auto& l = line.points;
								if (l.size() >= 2) {
									cv::Rect rect(cv::Point2i(l.front().x, l.front().y), cv::Point2i(l.back().x, l.back().y));
									cv::rectangle(displayImage, rect, cv::Scalar(0, 255, 0), 2);

									for (size_t i = 0; i < l.size() - 1; ++i) {
										cv::Point2i p1(l[i].x, l[i].y);
										cv::Point2i p2(l[i + 1].x, l[i + 1].y);
										cv::line(displayImage, p1, p2, cv::Scalar(0, 255, 0), 2);
									}
								}
							}

							std::string savePath = opticID + "_" + algoGraph->templateId().toStdString() + "_wirebondLines.png";
							cv::imwrite(savePath, displayImage);
						}				
					}
				
					auto voRect = vor.value().toRect();
					//remove out of bound rectsMap
					for (auto& line : lines)
					{
						auto & lineData = line.second;
						lineData = removeOutOfBoundLineData(lineData, QSize(voRect.width(), voRect.height()));
					}


					// sample data --
					//g_segModel->runModel_segmentation(images, rectsMap, _segmentationScore, true);
					Timer time;
					ct::logger::info("Start Wire bond AI V2 Inspection");
					g_segModel->runModel_segmentation_wireBond_v2(images, lines, -0.5, true);
					ct::logger::info("Wire bond AI Inspection V2 duration: %fs", time.duration());

					std::unordered_map<std::string, cv::Mat> resultMasks;
					resultMasks = g_segModel->getSegmentationMaskResult();

					QVector<DynamicMaskObject> dynamicMaskObjects;
					for (auto& m : resultMasks)
					{
						QString opticID = m.first.c_str();
						DynamicMaskObject dynamicMaskObject;
						dynamicMaskObject.algo_id = "WireBondInspection_1[@]" + opticID;
						dynamicMaskObject.id = "WireBondInspection_1[@]" + opticID;
						dynamicMaskObject.name = "WireBondInspection_1[@]" + opticID;
						dynamicMaskObject.type = AlgoType::WIREBOND;
						dynamicMaskObject.optic_id = opticID;
						util::cv_to_Mil(m.second, dynamicMaskObject.dynamicMaskImg);
						dynamicMaskObjects.append(dynamicMaskObject);

						/*	QString segmentMaskName = pFrameInfos[0].viewID + "_segmentMask.jpg";
							MbufExportA(segmentMaskName.toStdString().c_str(), M_JPEG_LOSSY, dynamicMaskObject.dynamicMaskImg);*/
					}


					_segmentationResult.insert(vor.key(), dynamicMaskObjects);
				}

				for (auto it = croppedMilImages.begin(); it != croppedMilImages.end(); ++it) {
					const QString& milImg_opticID = it.key();
					MIL_ID milImg = it.value();
					if (milImg) mtrx::free_buffer(milImg);
				}

				break;
			}
		}
		++vor;
	}
	return false;
}

void InspectionThread::objectDetectionResultToJsonObject(std::unordered_map<std::string, std::vector<OnnxResult>>& od_result)
{
	for (auto i : od_result)
	{
		std::string voName = i.first;
		std::vector<OnnxResult> result = i.second;

		QJsonObject voJsonObj;
		QJsonArray voResultJsonArray;

		for (auto r : result)
		{
			QJsonObject resultJsonObj;
			int t = 0;
			for (auto tPoint : r.transformedPoints)
			{
				QString tName_x = "transformed_point_x_" + QString::number(t);
				QString tName_y = "transformed_point_y_" + QString::number(t);
				resultJsonObj.insert(tName_x, tPoint.x);
				resultJsonObj.insert(tName_y, tPoint.y);
				t++;
			}
			resultJsonObj.insert("x1", r.x1);
			resultJsonObj.insert("y1", r.y1);
			resultJsonObj.insert("x2", r.x2);
			resultJsonObj.insert("y2", r.y2);

			resultJsonObj.insert("object_id", r.obj_id);
			resultJsonObj.insert("accuracy", r.accuracy);
			resultJsonObj.insert("angle", r.angle);

			voResultJsonArray.append(resultJsonObj);
		}

		voJsonObj.insert("vo_name", QString::fromStdString(voName));
		voJsonObj.insert("result_list", voResultJsonArray);

		_objectDetectionResult.insert(QString::fromStdString(voName), voJsonObj);
	}


}

//Test Run Algo Editor
void InspectionThread::objectDetectionResultToJsonObject(QVector<std::vector<OnnxResult>>& odResult, QVector<AiModelInfo::opticInfo>& odOpticsList, QStringList modelList)
{
	

	QJsonObject odResultObj;
	QJsonArray odResultArray;
	for (int i = 0; i < odResult.size(); i++)
	{
		QJsonObject voJsonObj;
		QJsonArray voResultJsonArray;
		for (auto r : odResult[i])
		{
			QJsonObject resultJsonObj;
			int t = 0;
			for (auto tPoint : r.transformedPoints)
			{
				QString tName_x = "transformed_point_x_" + QString::number(t);
				QString tName_y = "transformed_point_y_" + QString::number(t);
				resultJsonObj.insert(tName_x, tPoint.x);
				resultJsonObj.insert(tName_y, tPoint.y);
				t++;
			}
			resultJsonObj.insert("x1", r.x1);
			resultJsonObj.insert("y1", r.y1);
			resultJsonObj.insert("x2", r.x2);
			resultJsonObj.insert("y2", r.y2);

			resultJsonObj.insert("object_id", r.obj_id);
			resultJsonObj.insert("accuracy", r.accuracy);
			resultJsonObj.insert("angle", r.angle);

			voResultJsonArray.append(resultJsonObj);
		}

		voJsonObj.insert("vo_name", QString::fromStdString(""));
		voJsonObj.insert("optic_id", odOpticsList[i].opticId);
		voJsonObj.insert("locator_id", odOpticsList[i].locatorId);
		voJsonObj.insert("algoType", odOpticsList[i].algoType);
		voJsonObj.insert("optic_name", odOpticsList[i].opticName);
		voJsonObj.insert("channel", odOpticsList[i].channel);
		voJsonObj.insert("rotation", odOpticsList[i].imageRotation);
		voJsonObj.insert("model_name", modelList[i]);
		voJsonObj.insert("result_list", voResultJsonArray);

		odResultArray.append(voJsonObj);
	}

	odResultObj.insert("odResults", odResultArray);

	QString jsonPath = Common::Directory::CachePath + "ODResult.json";
	QJsonDocument doc(odResultObj);
	bool flag = false;

	// Check if the file exists and delete it
	if (QFile::exists(jsonPath)) {
		if (!QFile::remove(jsonPath)) {
			qDebug() << "Failed to delete existing file:" << jsonPath;
		}
	}

	QFile file(jsonPath);
	if (file.open(QIODevice::WriteOnly))
	{
		file.write(doc.toJson());
		file.flush();
		file.close();

		flag = true;
	}
	qDebug() << "odResultSuccessfullyGenerated:" << flag;
	//return flag;
}

void InspectionThread::setCountMode(CountMode mode)
{
	_countMode = mode;
}

void InspectionThread::setExpectedIndex(int expectedIndex)
{
	_expectedIndex = expectedIndex;
}

int InspectionThread::getExpectedIndex()
{
	return _expectedIndex;
}

bool InspectionThread::isStarted()
{
	return _scanInspBufferQueue;
}

//InspectionThread run result
void InspectionThread::objectDetectionResultToJsonObject(QVector<ODModelResults> odModelResults, QStringList totalVoNames)
{
	for (int i = 0; i < totalVoNames.size(); i++)
	{
		QJsonObject odResultObj;
		QJsonArray odResultArray;
		for (int j = 0; j < odModelResults.size(); j++)
		{
			QJsonObject voJsonObj;
			QJsonArray voResultJsonArray;

			auto od_result = odModelResults[j].od_result;

			for (auto o : od_result)
			{
				std::string voName = o.first;
				if (voName.c_str() == totalVoNames[i])
				{
					std::vector<OnnxResult> result = o.second;

					QJsonObject voJsonObj;
					QJsonArray voResultJsonArray;

					for (auto r : result)
					{
						QJsonObject resultJsonObj;
						int t = 0;
						for (auto tPoint : r.transformedPoints)
						{
							QString tName_x = "transformed_point_x_" + QString::number(t);
							QString tName_y = "transformed_point_y_" + QString::number(t);
							resultJsonObj.insert(tName_x, tPoint.x);
							resultJsonObj.insert(tName_y, tPoint.y);
							t++;
						}
						resultJsonObj.insert("x1", r.x1);
						resultJsonObj.insert("y1", r.y1);
						resultJsonObj.insert("x2", r.x2);
						resultJsonObj.insert("y2", r.y2);

						resultJsonObj.insert("object_id", r.obj_id);
						resultJsonObj.insert("accuracy", r.accuracy);
						resultJsonObj.insert("angle", r.angle);

						voResultJsonArray.append(resultJsonObj);
					}

					voJsonObj.insert("vo_name", QString::fromStdString(voName));
					voJsonObj.insert("result_list", voResultJsonArray);
					voJsonObj.insert("model_name", odModelResults[j].modelName);
					voJsonObj.insert("optic_name", odModelResults[j].opticName);
					voJsonObj.insert("optic_id", odModelResults[j].opticID);
					voJsonObj.insert("rotation", odModelResults[j].imageRotation);
					voJsonObj.insert("channel", odModelResults[j].channel);
					voJsonObj.insert("locator_id", odModelResults[j].locatorId);
					voJsonObj.insert("algoType", odModelResults[j].algoType);
					odResultArray.append(voJsonObj);
				}
			}
		}

		odResultObj.insert("odResults", odResultArray);

		_objectDetectionResult.insert(totalVoNames[i], odResultObj);
	}
}


QString InspectionThread::defectMapping(QString& tagName)
{
	QString nDefectName = tagName;
	if (tagName.contains("[!]"))
	{
		for (int j = 0; j < _defectMappingHash.size(); j++)
		{
			QString systemDefect = _defectMappingHash.keys()[j];
			QString mTagName = _defectMappingHash[systemDefect];

			if (tagName == systemDefect)nDefectName = mTagName;
		}
	}
	return nDefectName;
}

void InspectionThread::cadTagNameMapping(ct::DefectResult& dResult, QString cadFamily)
{
	if (cadFamily.toLower().contains("die"))
	{
		bool isCupillar = false;
		for (auto& cFam : _cadFamilyInfos)
		{
			if (cFam.familyName == cadFamily)
				isCupillar = cFam.isCuPillar;
		}

		if (dResult.algoDefResult.systemDefectName == QString("[!]Object Missing"))
		{
			if (isCupillar)	dResult.algoDefResult.customTagName = QString("Missing CuP Die");
			else 	dResult.algoDefResult.customTagName = QString("Missing FBar Die");

		}
		else if (dResult.algoDefResult.systemDefectName == QString("[!]Object Misalignment"))
		{
			if (isCupillar)	dResult.algoDefResult.customTagName = QString("CuP Die Misalignment");
			else 	dResult.algoDefResult.customTagName = QString("FBar Die Misalignment");
		}
		else if (dResult.algoDefResult.systemDefectName == QString("[!]Object Failed Height"))
		{
			if (isCupillar)	dResult.algoDefResult.customTagName = QString("CuP Die Height");
			else 	dResult.algoDefResult.customTagName = QString("FBar Die Height");
		}
		else if (dResult.algoDefResult.systemDefectName == QString("[!]Object Failed Tilt"))
		{
			if (isCupillar)	dResult.algoDefResult.customTagName = QString("CuP Die Tilt");
			else 	dResult.algoDefResult.customTagName = QString("FBar Die Tilt");
		}
		else if (dResult.algoDefResult.systemDefectName == QString("[!]Object Touching"))
		{
			if (isCupillar)	dResult.algoDefResult.customTagName = QString("CuP Die Misalignment");
			else 	dResult.algoDefResult.customTagName = QString("FBar Die Misalignment");
		}
		else if (dResult.algoDefResult.systemDefectName == QString("[!]Object Failed Volume"))
		{

		}
	}
	
}

void InspectionThread::setDefectCollectorPath(QString path)
{
	_defectCollectorPath = path;

}

//mainInspectionLoop
#if HAS_VIDI_LICENSE
bool InspectionThread::vidiInspectionFromMemory(const QHash<QString, QRectF>& visionObjectsRect, const  QHash<QString, QPointF>& locatorOffsets, QVector<FrameInfo>& pFrameInfos, QVector<MIL_ID>& milImgs)
{
	qDebug() << "vidi inspection from memory";
	//temp hardcoded
	QHash<QString, util::ImagePreprocess*> imagePreprocessTools;
	auto optic = _recipeOptics.constBegin();
	while (optic != _recipeOptics.constEnd())
	{
		/*QString refImgPath = Common::Directory::getRecipeVidiImagePath() + algoGraph->templateId() + "\\ref_" + optic.value().name + g_imgExtension;
		QString maskImgPath = Common::Directory::getRecipeVidiImagePath() + algoGraph->templateId() + "\\mask" + g_imgExtension;*/
		QString refImgPath = Common::Directory::getRecipeVidiImagePath() + "ref_" + optic.value().name + g_imgExtension;
		QString maskImgPath = Common::Directory::getRecipeVidiImagePath() + "mask" + g_imgExtension;
		util::ImagePreprocess* imagePreprocess;
		imagePreprocess = new util::ImagePreprocess(refImgPath, maskImgPath);
		imagePreprocessTools.insert(optic.value().name, imagePreprocess);
		optic++;
	}

	bool flag = true;

	_vidiResults.clear();
	QHash<QString, QRectF>::const_iterator vor = visionObjectsRect.constBegin();
	while (vor != visionObjectsRect.constEnd())
	{
		//ignore vision Object
		QString voID;
		for (int i = 0; i < _dragROI.size(); i++)
		{
			if (_dragROI[i]->getName() == vor.key())
			{
				voID = _dragROI[i]->getId();
				break;
			}
		}

		if (!_visionObjects.find(voID).value().ignore && !_visionObjects.find(voID).value().skip)
		{
			QRect voRect = vor.value().toRect();
			offsetVisionObject(locatorOffsets, vor.key(), voRect);

			QJsonObject res;
			QJsonArray streamVidiResults;
			for (int i = 0; i < pFrameInfos.size(); i++)
			{
				QString opticName = _recipeOptics[pFrameInfos[i]._opticID].name;

				QStringList opticPreprocessList;
				opticPreprocessList.append(opticName);
				if (opticName == "RGB") opticPreprocessList.append("RGB_HighlightDefects");
				else if (opticName == "RB") opticPreprocessList.append("RB_DiffOfMedianFilter");
				else if (opticName == "DieLight") opticPreprocessList.append("DieLight_DiffOfMedianFilter");

				QJsonObject vidiResult;
				for (int j = 0; j < opticPreprocessList.size(); j++)
				{
					auto opticPreprocessor = opticPreprocessList[j];

					if (checkVidiStreamExist(opticPreprocessor))
					{
						VIDI_IMAGE croppedImg;

						//temp harcode jet
						auto imagePreprocess = imagePreprocessTools[opticName];

						MIL_ID childBuffer;
						performImagePreprocess(*imagePreprocess, voRect, milImgs[i], childBuffer, opticPreprocessor);

						MIL_INT SizeX = 0;
						MIL_INT SizeY = 0;
						MbufInquire(childBuffer, M_SIZE_X, &SizeX);
						MbufInquire(childBuffer, M_SIZE_Y, &SizeY);

						childVidiImagestoJPG(QRect(0, 0, SizeX, SizeY), childBuffer, &croppedImg);
						MbufFree(childBuffer);

						/*if (opticPreprocessor == "RGB_HighlightDefects") childVidiImagePreprocess(*imagePreprocess, voRect, milImgs[i], &croppedImg, HIGHLIGHT_DEFECT);
						else if (opticPreprocessor == "RB") childVidiImagePreprocess(*imagePreprocess, voRect, milImgs[i], &croppedImg, DIFF_OF_MEDIAN);
						else if (opticPreprocessor == "DieLight") childVidiImagePreprocess(*imagePreprocess, voRect, milImgs[i], &croppedImg, DIFF_OF_MEDIAN);
						else childVidiImagestoJPG(voRect, milImgs[i], &croppedImg);*/

						/*			if (opticPreprocessor == "RB" && vor.key() == "I1R6C33")
									{
										QString filePath = "C:/Advanced/Data/BUYOFF PRODUCTION/analyze/test1.jpg";
										MbufSaveA(filePath.toStdString().c_str(), childBuffer);
										QString fileName = Common::Directory::getRecipeCurrentPath() + "image/" + vor.key() + "_" + opticName + ".jpeg";
										int ret = vidi_save_image(fileName.toStdString().c_str(), &croppedImg);
									}

									if (opticPreprocessor == "DieLight" && vor.key() == "I1R5C40")
									{
										QString fileName = Common::Directory::getRecipeCurrentPath() + "image/" + vor.key() + "_" + opticName + ".jpeg";
										int ret = vidi_save_image(fileName.toStdString().c_str(), &croppedImg);
									}
								*/

						Timer time;
						vidiInspection(&croppedImg, vor.key(), false, vidiResult, pFrameInfos[i]._opticID, opticPreprocessor);

						if (false)
						{
							QString fileName = Common::Directory::getRecipeCurrentPath() + "image/" + vor.key() + "_" + opticPreprocessor + ".json";
							QJsonDocument doc(vidiResult);

							QFile file(fileName);
							if (file.open(QIODevice::WriteOnly))
							{
								file.write(doc.toJson());
								file.flush();
								file.close();
							}

						}


						//qDebug() << "voName:" << vor.key() << " singleVidiInsp:" << time.duration();


						streamVidiResults.push_back(vidiResult);

						//vidi_free_image(&croppedImg);
						delete[]((uint8_t*)croppedImg.data);
					}
				}

			}

			res.insert("vidiResults", streamVidiResults);
			_vidiResults.insert(vor.key(), res);
		}

		++vor;
	}

	auto imageP = imagePreprocessTools.constBegin();
	while (imageP != imagePreprocessTools.constEnd())
	{
		delete imageP.value();
		imageP++;
	}

	return flag;
}

bool InspectionThread::vidiInspection(VIDI_IMAGE* Image, QString objectID, bool saveResult, QJsonObject& vidiResult, QString opticID, QString streamName)
{
	bool flag = true;
	//setup Result Buffer
	VIDI_BUFFER result_buffer;
	_status = vidi_init_buffer(&result_buffer);
	if (_status != VIDI_SUCCESS)
	{
		_errorMsg = QStringLiteral("Fail to initialise inspection result buffer");
		qDebug() << "vidiInspection ErroMsg:" << _errorMsg;
		flag = false;
	}

	// the first step is to initialize the sample
	_status = vidi_runtime_create_sample(_workspaceName.toStdString().c_str(), streamName.toStdString().c_str(), "my_sample");
	if (_status != VIDI_SUCCESS)
	{
		_errorMsg = QStringLiteral("Fail to initialise inspection sample");
		qDebug() << "vidiInspection ErroMsg:" << _errorMsg;
		flag = false;
	}

	// then add the image to be processed
	_status = vidi_runtime_sample_add_image(_workspaceName.toStdString().c_str(), streamName.toStdString().c_str(), "my_sample", Image);
	if (_status != VIDI_SUCCESS)
	{
		_errorMsg = QStringLiteral("Fail to add inspection image");
		qDebug() << _errorMsg;
		flag = false;
	}



	// process image
	_status = vidi_runtime_sample_process(_workspaceName.toStdString().c_str(), streamName.toStdString().c_str(), "", "my_sample", "");
	if (_status != VIDI_SUCCESS)
	{
		vidi_get_error_message(_status, &_buffer);
		_errorMsg = QStringLiteral("Fail to process inspection sample, error message: %1").arg(QString(_buffer.data));
		qDebug() << "vidiInspection ErroMsg:" << _errorMsg;
		flag = false;
	}

	// the next step is to get the results
	_status = vidi_runtime_get_sample_json(_workspaceName.toStdString().c_str(), streamName.toStdString().c_str(), "my_sample", &result_buffer);
	if (_status != VIDI_SUCCESS)
	{
		_errorMsg = QStringLiteral("Fail to get inspection result");
		qDebug() << "vidiInspection ErroMsg:" << _errorMsg;
		flag = false;
	}

	// process result
	_resultBufferData.clear();
	_resultBufferData = QByteArray(result_buffer.data);
	QJsonDocument doc = QJsonDocument::fromJson(_resultBufferData);
	QJsonObject res = doc.object();

	//if(saveResult) _vidiResults.insert(objectID, res);

	vidiResult = res;
	vidiResult.insert(QStringLiteral("optic_name"), streamName);
	vidiResult.insert(QStringLiteral("optic_id"), opticID);

	/*if (saveResult)
	{
		QString jsonPath = Common::Directory::getRecipeCurrentPath() + "VIDI_result.json";
		std::ofstream ofs(jsonPath.toStdString());
		ofs << _resultBufferData.data() << std::endl;
		ofs.close();
	}*/


	//free resource
	_status = vidi_free_buffer(&result_buffer);
	if (_status != VIDI_SUCCESS)
	{
		_errorMsg = QStringLiteral("Fail to free inspection result buffer");
		qDebug() << "vidiInspection ErroMsg:" << _errorMsg;
		flag = false;
	}

	_status = vidi_runtime_free_sample(_workspaceName.toStdString().c_str(), streamName.toStdString().c_str(), "my_sample");
	if (_status != VIDI_SUCCESS)
	{
		_errorMsg = QStringLiteral("Fail to free inspection sample");
		qDebug() << "vidiInspection ErroMsg:" << _errorMsg;
		flag = false;
	}

	return flag;
}

bool InspectionThread::checkVidiStreamExist(QString streamName)
{
	bool flag = true;
	VIDI_BUFFER buffer;
	_status = vidi_init_buffer(&buffer);
	if (_status != VIDI_SUCCESS)
	{
		_errorMsg = QStringLiteral("Fail to initialise inspection result buffer");
		flag = false;
	}
	vidi_runtime_list_streams(_workspaceName.toStdString().c_str(), &buffer);

	QByteArray bufferData = QByteArray(buffer.data);

	QStringList streamIds;
	QXmlStreamReader xmlReader(bufferData);
	while (!xmlReader.atEnd() && !xmlReader.hasError()) {
		if (xmlReader.readNextStartElement()) {
			if (xmlReader.name() == "stream") {
				QString id = xmlReader.attributes().value("id").toString();
				streamIds.append(id);
			}
		}
	}

	vidi_free_buffer(&buffer);
	return streamIds.contains(streamName);
}

void InspectionThread::cadClassificationInspection(const QHash<QString, QRectF>& visionObjectsRect, const QHash<QString, QPointF>& locatorOffsets, QVector<QImage>& qImgs, QVector<FrameInfo>& pFrameInfos, QVector<MIL_ID>& milImgs)
{
	bool flag = true;

	//1. load Cad Type
	//2. getViewDefects
	//3. getCadRoiInfos for the Vo of the defects.
	//4. calculate the percentage of intersection between defect and component to the component size
	//5. if percentage exceeds the threshold then register the cad into defect
	//6. using the Cad Type name as the stream Name, perform classification on the cad that is registered into the defects if the model exist, if not set tagName to -
	//7. collect the tagNames from the cad and add it into vision Object, based on the priority list arrange the defect tagNames
	//8. store the cad defect information in an array and store them into the database after inspection done
	//9. generate good VO, defect VO into folder

	//1. load Cad Type
	QHash<QString, QRectF>::const_iterator vor = visionObjectsRect.constBegin();
	while (vor != visionObjectsRect.constEnd())
	{
		QRect voRect = vor.value().toRect();
		offsetVisionObject(locatorOffsets, vor.key(), voRect);

		QVector<VIDI_IMAGE> croppedImgs;
		for (int i = 0; i < milImgs.size(); i++)
		{
			VIDI_IMAGE croppedImg;
			childVidiImages(voRect, milImgs[i], &croppedImg);
			croppedImgs.append(croppedImg);
		}

		QStringList croppedImgFileNames;
		QStringList croppedImgFilePaths;
		for (int i = 0; i < pFrameInfos.size(); i++)
		{
			uidGenerator uid;
			QString croppedImgFileName = QString("vo_") + _recipeOptics[pFrameInfos[i]._opticID].name + "_" + uid.id().c_str() + ".jpg";
			QString croppedImgFilePath = Common::Directory::ClassificationWorkspaceImagePath() + croppedImgFileName;
			croppedImgFileNames.append(croppedImgFileName);
			croppedImgFilePaths.append(croppedImgFilePath);
		}

		//get cadRois
		QVector<CadRoiInfo> cadRois;
		for (int i = 0; i < _dragROI.size(); i++)
		{
			if (_dragROI[i]->getName() == vor.key())
			{
				auto algoGraph = _dragROI[i]->algoGraph();
				auto c = _templateCadRois.find(algoGraph->templateId());
				if (c != _templateCadRois.end()) cadRois = c.value();
				break;
			}
		}

		//crop cadRois
		QHash<QString, QVector<VIDI_IMAGE>> cadVidiImages;
		for (int i = 0; i < cadRois.size(); i++)
		{
			auto cadRoi = cadRois[i];
			QRect FOVcadRect = QRect(cadRoi.x + voRect.x(), cadRoi.y + voRect.y(), cadRoi.w, cadRoi.h);

			QVector<VIDI_IMAGE> opticVidiImages;
			for (int j = 0; j < milImgs.size(); j++)
			{
				VIDI_IMAGE opticImg;
				childVidiImages(FOVcadRect, milImgs[j], &opticImg);
				opticVidiImages.append(opticImg);
			}
			cadVidiImages.insert(cadRoi.id, opticVidiImages);
		}

		bool pass = true;
		QStringList voTagNames;
		for (int i = 0; i < _defectResults.size(); i++)
		{
			if (_defectResults[i].algoDefResult.vo_name == vor.key().toStdString())
			{

				pass = false;

				QString opticID = _defectResults[i].algoDefResult.optic_id.c_str();
				if (opticID.isEmpty()) opticID = _mainOptics.id;
				int opticIndex = 0;
				for (int j = 0; j < pFrameInfos.size(); j++)
				{
					if (pFrameInfos[j]._opticID == opticID)
					{
						opticIndex = j;
					}
				}


				//if (_defectResults[i].algoDefResult.enableCustomTagName) //custom TagName
				//{
				//	_defectResults[i].tagNames.append(_defectResults[i].algoDefResult.customTagName);
				//}
				//else //cad classification - start
				{
					QRectF defRect(_defectResults[i].algoDefResult.def_x, _defectResults[i].algoDefResult.def_y, _defectResults[i].algoDefResult.def_w, _defectResults[i].algoDefResult.def_h);
					removeLocOffsetfromRect(locatorOffsets, vor.key(), defRect);

					double totalIntersectPercentage = 0;

					//cadClassification			
					for (int j = 0; j < cadRois.size(); j++)
					{
						//calculate percentage
						auto cadRoi = cadRois[j];
						QRectF cadRect(cadRoi.x, cadRoi.y, cadRoi.w, cadRoi.h);
						QRectF intersectedRect = defRect.intersected(cadRect);

						double intersectArea = intersectedRect.width() * intersectedRect.height();
						double cadArea = cadRoi.w * cadRoi.h;
						double defectArea = defRect.width() * defRect.height();

						if (cadArea == 0) cadArea = 1;
						double percentage = intersectArea / cadArea * 100;
						totalIntersectPercentage = totalIntersectPercentage + (intersectArea) / defectArea * 100;

						//if percentage exceed threshold
						double cadThreshold = 0;
						if (percentage > cadThreshold)
						{
							cadRoi.vo_name = _defectResults[i].algoDefResult.vo_name.c_str();
							cadRoi.vo_id = _defectResults[i].algoDefResult.vo_id.c_str();

							//perform classification inspection
							QVector<VIDI_IMAGE> cadImgs = cadVidiImages.find(cadRoi.id).value();
							QString streamName = getCadTypeName(cadRoi.type_id);

							if (false)
							{
								QStringList tagNames = classificationInspection(streamName, vor.key(), &cadImgs[opticIndex]);
								if (tagNames.isEmpty()) tagNames << "Unknown";
								changeSelectedTagNametoUnknown(tagNames, "Good");

								insertDefectTagNames(_defectResults[i].tagNames, tagNames);
								cadRoi.tagNames.append(tagNames);
								cadRoi.tagIDs.append(tagNames);
								cadRoi.image_path = croppedImgFileNames[opticIndex];
							}

							_defectResults[i].cadRoiInfos.append(cadRoi);
						}
					}

					//backgroundClassification
					if (false)
					{
						double backgroundPercentage = 100 - totalIntersectPercentage;
						double backgroundThreshold = 20;
						if (backgroundPercentage > backgroundThreshold)
						{
							CadRoiInfo cadRoi;

							cadRoi.name = "background";
							cadRoi.id = "background";
							cadRoi.type_name = "Background";
							cadRoi.type_id = "Background";
							cadRoi.x = defRect.x();
							cadRoi.y = defRect.y();
							cadRoi.w = defRect.width();
							cadRoi.h = defRect.height();
							cadRoi.image_path = croppedImgFileNames[opticIndex];
							cadRoi.vo_name = _defectResults[i].algoDefResult.vo_name.c_str();
							cadRoi.vo_id = _defectResults[i].algoDefResult.vo_id.c_str();

							QRect FOVdefRect = QRect(defRect.x() + voRect.x(), defRect.y() + voRect.y(), defRect.width(), defRect.height());
							VIDI_IMAGE defImg;
							childVidiImages(FOVdefRect, milImgs[opticIndex], &defImg);

							QString streamName = "Background";
							QStringList tagNames = classificationInspection(streamName, vor.key(), &defImg);
							if (tagNames.isEmpty()) tagNames << "Unknown";

							changeSelectedTagNametoUnknown(tagNames, "Good");
							insertDefectTagNames(_defectResults[i].tagNames, tagNames);
							cadRoi.tagNames.append(tagNames);
							cadRoi.tagIDs.append(tagNames);
							_defectResults[i].cadRoiInfos.append(cadRoi);

							delete[]((uint8_t*)defImg.data);
						}
					}
				}

				if (false)
				{
					_defectResults[i].tagNames.removeDuplicates();
					arrangeTagNameBasedOnPriority(_defectResults[i].tagNames);
					voTagNames.append(_defectResults[i].tagNames);
				}


			}	//cad classification - end
		}


		//skip
		if (false)
		{
			voTagNames.removeDuplicates();
			if (!pass) arrangeTagNameBasedOnPriority(voTagNames);
			//qDebug() << "voName:" << vor.key() << "voTagNames:" << voTagNames;
			if (_saveDefectVoImg) generateDefectImages(milImgs, pFrameInfos, voTagNames, vor.key(), voRect);
		}




		//skip temporarily
		if (false)
		{
			if (!pass && g_enableClassificationDataCollection)
			{
				//Timer time;
				for (int i = 0; i < croppedImgs.size(); i++)
				{
					int ret = vidi_save_image(croppedImgFilePaths[i].toStdString().c_str(), &croppedImgs[i]);
				}

				//qDebug() << "vidiSaveImageDuration:" << time.duration();
			}
		}

		QHash<QString, QVector<VIDI_IMAGE>>::const_iterator cv = cadVidiImages.constBegin();
		while (cv != cadVidiImages.constEnd())
		{
			for (int i = 0; i < cv.value().size(); i++)
			{
				delete[]((uint8_t*)cv.value()[i].data);
			}

			cv++;
		}

		for (int i = 0; i < croppedImgs.size(); i++)
		{
			delete[]((uint8_t*)croppedImgs[i].data);
		}


		vor++;
	}
}

QStringList InspectionThread::classificationInspection(QString& streamName, const QString& voName, VIDI_IMAGE* vidiImg)
{
	bool skipVIDI_Classification = false;
	if (_workspaceOpened.isEmpty()) skipVIDI_Classification = true;
	if (!skipVIDI_Classification)
	{
		if (!checkVidiStreamExist(streamName))
		{
			skipVIDI_Classification = true;
		}
	}

	QStringList tagNames;
	//check if workspace is opened, check if stream exist or not
	if (!skipVIDI_Classification)
	{
		QJsonObject vidiResult;
		vidiInspection(vidiImg, voName, false, vidiResult, streamName.toStdString().c_str());
		VidiToolResult vidiToolResult;
		vidiToolResult.attachResult(vidiResult);
		tagNames = vidiToolResult.getTagNames();
	}
	else
	{
		tagNames.append("Unknown");
	}

	return tagNames;
}

bool InspectionThread::formVidiImage(FrameInfo* pFrameInfo, VIDI_IMAGE* img)
{
	bool flag = true;
	_status = vidi_init_image(img);
	if (_status != VIDI_SUCCESS)
	{
		vidi_get_error_message(_status, &_buffer);
		_errorMsg = QStringLiteral("Fail to initialise inspection image, error message: %1").arg(QString(_buffer.data));
		qDebug() << _errorMsg;
		flag = false;
	}

	// Construct color image
	if (_recipeOptics[pFrameInfo->_opticID].type == ct::s_color)
	{
		_status = vidi_create_image(pFrameInfo->_width, pFrameInfo->_height, pFrameInfo->_width, VIDI_IMG_8U, reinterpret_cast<void*>(pFrameInfo->_pBlueImageBuf), reinterpret_cast<void*>(pFrameInfo->_pGreenImageBuf), reinterpret_cast<void*>(pFrameInfo->_pRedImageBuf), nullptr, img);
	}
	else if (_recipeOptics[pFrameInfo->_opticID].type == ct::s_mono)
	{
		_status = vidi_create_image(pFrameInfo->_width, pFrameInfo->_height, pFrameInfo->_width, VIDI_IMG_8U, reinterpret_cast<void*>(pFrameInfo->_pImageBuf), nullptr, nullptr, nullptr, img);
	}

	//_status = vidi_create_image(pFrameInfo->_width, pFrameInfo->_height, pFrameInfo->_width, VIDI_IMG_8U, reinterpret_cast<void*>(pBlue), reinterpret_cast<void*>(pGreen), reinterpret_cast<void*>(pRed), nullptr, img);
	if (_status != VIDI_SUCCESS)
	{
		vidi_get_error_message(_status, &_buffer);
		_errorMsg = QStringLiteral("Fail to create color image, error message: %1").arg(QString(_buffer.data));
		qDebug() << _errorMsg;
		flag = false;
	}

	return flag;
}

bool InspectionThread::releaseVidiImage(VIDI_IMAGE* img)
{
	bool flag = true;
	// free resource
	_status = vidi_free_image(img);
	if (_status != VIDI_SUCCESS)
	{
		_errorMsg = QStringLiteral("Fail to free inspection image");
		//vidi_runtime_free_sample(_workspaceName.toStdString().c_str(), "default", "my_sample");
		qDebug() << _errorMsg;
		flag = false;
	}

	return flag;
}

bool InspectionThread::cropVidiImages(const QRect rect, VIDI_IMAGE* src, VIDI_IMAGE* croppedImage)
{
	if (!src) return false;
	croppedImage->channels = src->channels;
	croppedImage->channel_depth = VIDI_IMG_8U;
	croppedImage->height = rect.height();
	croppedImage->width = rect.width();
	croppedImage->step = croppedImage->width * croppedImage->channels;

	croppedImage->data = new uint8_t[croppedImage->height * croppedImage->step];

	for (size_t y = 0; y < croppedImage->height; y++)
	{
		for (size_t x = 0; x < croppedImage->width; x++)
		{
			size_t originalX = x + rect.x();
			size_t originalY = y + rect.y();

			// Check if the original pixel is within the bounds of the original image
			if (originalX >= 0 && originalX < src->width && originalY >= 0 && originalY < src->height)
			{
				// Get the original pixel value
				uint8_t* pixel_ptr = (uint8_t*)src->data + originalY * src->step + originalX * src->channels;

				// Copy the pixel value to the cropped image data
				uint8_t* croppedPixel_ptr = (uint8_t*)croppedImage->data + y * croppedImage->step + x * croppedImage->channels;
				croppedPixel_ptr[0] = pixel_ptr[0];
				croppedPixel_ptr[1] = pixel_ptr[1];
				croppedPixel_ptr[2] = pixel_ptr[2];
			}
			else
			{
				// Set the pixel to black if it is outside the bounds of the original image
				uint8_t* croppedPixel_ptr = (uint8_t*)croppedImage->data + y * croppedImage->step + x * croppedImage->channels;
				croppedPixel_ptr[0] = 0;
				croppedPixel_ptr[1] = 0;
				croppedPixel_ptr[2] = 0;
			}
		}
	}

	//int ret = vidi_save_image("C:/Advanced/Data/recipe/2SZ1_1703/Images/Testing/vidiCroppedImage.jpeg", croppedImage);
	return true;
}

bool InspectionThread::childVidiImages(const QRect rect, MIL_ID& src, VIDI_IMAGE* childImage)
{
	MIL_INT bandSize;

	MbufInquire(src, M_SIZE_BAND, &bandSize);

	if (!src) return false;
	childImage->channels = bandSize;
	childImage->channel_depth = VIDI_IMG_8U;
	childImage->height = rect.height();
	childImage->width = rect.width();
	childImage->step = childImage->width * childImage->channels;
	childImage->data = new uint8_t[childImage->height * childImage->step];

	MbufGetColor2d(src, M_PACKED + M_BGR24, M_ALL_BANDS, rect.x(), rect.y(), rect.width(), rect.height(), childImage->data);

	/*util::saveCroppedMilImg(src, rect, QString("vidiCroppedImageMil.jpeg"));*/
	return true;
}

bool InspectionThread::childVidiImagestoJPG(const QRect rect, MIL_ID& src, VIDI_IMAGE* childImage)
{
	MIL_INT bandSize;

	MbufInquire(src, M_SIZE_BAND, &bandSize);
	//MIL_ID jpegBuffer = M_NULL;
	//if(bandSize == 1) jpegBuffer = MbufAlloc2d(M_DEFAULT, rect.width(), rect.height(), 8 + M_UNSIGNED, M_IMAGE + M_COMPRESS + M_JPEG_LOSSY, M_NULL);
	//else if (bandSize == 3) jpegBuffer = MbufAllocColor(M_DEFAULT, bandSize, rect.width(), rect.height(), 8 + M_UNSIGNED, M_IMAGE + M_COMPRESS + M_JPEG_LOSSY, M_NULL);
	////MbufControl(jpegBuffer, M_Q_FACTOR, 50);
	//MbufCopy(src, jpegBuffer);
	//MbufExportA("Cropped_mil.jpg", M_PNG, jpegBuffer);


	if (!src) return false;
	childImage->channels = bandSize;
	childImage->channel_depth = VIDI_IMG_8U;
	childImage->height = rect.height();
	childImage->width = rect.width();
	childImage->step = childImage->width * childImage->channels;
	childImage->data = new uint8_t[childImage->height * childImage->step];

	MbufGetColor2d(src, M_PACKED + M_BGR24, M_ALL_BANDS, rect.x(), rect.y(), rect.width(), rect.height(), childImage->data);
	//int ret = vidi_save_image("Cropped_vidi.jpg", childImage);
	//convert to jpg
	/*int ret = vidi_save_image("vidiCroppedImage.jpeg", childImage);
	delete[]((uint8_t*)childImage->data);
	_status = vidi_init_image(childImage);
	_status = vidi_load_image("vidiCroppedImage.jpeg", childImage);*/

	//convert to jpg

	return true;
}

bool InspectionThread::childVidiImagePreprocess(util::ImagePreprocess& imagePreprocess, const QRect rect, MIL_ID& src, VIDI_IMAGE* childImage, PreprocessMethod method)
{
	MIL_INT bandSize;
	MbufInquire(src, M_SIZE_BAND, &bandSize);

	MIL_ID croppedImg = MbufAllocColor(M_DEFAULT, bandSize, rect.width(), rect.height(), 8 + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);
	MbufClear(croppedImg, M_BLACK);
	auto imgW = mtrx::get_width(src);
	auto imgH = mtrx::get_height(src);

	int endX = rect.x() + rect.width();
	int endY = rect.y() + rect.height();

	int cappedWidth = rect.width();
	int cappedHeight = rect.height();
	if (endX > imgW) cappedWidth = imgW - rect.x();
	if (endY > imgH) cappedHeight = imgH - rect.y();

	int startX = 0;
	int startY = 0;
	int srcX = rect.x();
	int srcY = rect.y();
	if (rect.x() < 0)
	{
		startX = -rect.x();
		srcX = 0;
		cappedWidth = imgW - startX;
	}
	if (rect.y() < 0)
	{
		startY = -rect.y();
		srcY = 0;
		cappedHeight = imgH - startY;
	}

	//calculate exceed how much, then cap the copy
	MbufCopyColor2d(src, croppedImg, M_ALL_BANDS, srcX, srcY, M_ALL_BANDS, startX, startY, cappedWidth, cappedHeight);
	//qDebug() << "CAPPPED IMAGE:srcX: " << srcX << ", srcY:" << srcY << ", startX" << startX << ", startY:" << startY << " cappedWidth:" << cappedWidth << " cappedHeight:" << cappedHeight;
	//MbufSaveA("cappedImg.jpg", milChild);

	//perform Difference of Median Filter
	if (method == DIFF_OF_MEDIAN) imagePreprocess.Diff_of_medianFilter(croppedImg, croppedImg);
	else if (method == HIGHLIGHT_DEFECT) imagePreprocess.HighlightDarkDefects(croppedImg, croppedImg);

	//put after preprocess image back to vidi child Image

	if (!src) return false;
	childImage->channels = bandSize;
	childImage->channel_depth = VIDI_IMG_8U;
	childImage->height = rect.height();
	childImage->width = rect.width();
	childImage->step = childImage->width * childImage->channels;
	childImage->data = new uint8_t[childImage->height * childImage->step];

	MbufGetColor(croppedImg, M_PACKED + M_BGR24, M_ALL_BANDS, childImage->data);
	MbufFree(croppedImg);

	//convert to jpg
	int ret = vidi_save_image("vidiCroppedImage.jpeg", childImage);
	delete[]((uint8_t*)childImage->data);
	_status = vidi_init_image(childImage);
	_status = vidi_load_image("vidiCroppedImage.jpeg", childImage);
	//convert to jpg


	return true;

}
#endif