#include "VisionApp.h"


void VisionApp::addObjectDetectionModels()
{
	QString ObjectDetectionPath = Common::Directory::LocalPath + "AIModel\\ObjectDetection\\";
	QDir dir(ObjectDetectionPath);
	dir.mkpath(ObjectDetectionPath);

	QStringList filters;
	filters << "*.onnx";
	QStringList odModels = dir.entryList(filters, QDir::NoDotAndDotDot | QDir::AllEntries);
	qDebug() << "odModels:" << odModels;

    clearGridLayout(ui.gridLayout_ModelList);
    _objectDetectionModelPaths.clear();
    _objectDetectionCheckBoxes.clear();

	for (int i = 0; i < odModels.size(); i++)
	{
		QLineEdit* odModel = new QLineEdit(odModels[i]);
        odModel->setReadOnly(true);
        _objectDetectionModelPaths.push_back(odModel);

        QCheckBox* odCheckBox = new QCheckBox();
        odCheckBox->setWhatsThis(odModels[i]);
        _objectDetectionCheckBoxes.push_back(odCheckBox);
        connect(odCheckBox, SIGNAL(clicked()), this, SLOT(oDModelCheckBoxChanged()));

        ui.gridLayout_ModelList->addWidget(odCheckBox, i, 0);
        ui.gridLayout_ModelList->addWidget(odModel, i, 1);     
	}
}

void VisionApp::load_unload_ODModels()
{
    if (ui.comboBox_loadODModels->currentText() == "Load Models") loadODModels();
    else unloadODModels();
}

void VisionApp::loadODModels()
{
    if (_objectDetectionModelPaths.size() != _objectDetectionCheckBoxes.size()) return;
    unloadODModels();

    for (int i = 0; i < _objectDetectionModelPaths.size(); i++)
    {
        if (_objectDetectionCheckBoxes[i]->isChecked())
        {
            QString ObjectDetectionPath = Common::Directory::LocalPath + "AIModel\\ObjectDetection\\";
            ObjectDetectionPath = ObjectDetectionPath + _objectDetectionModelPaths[i]->text();
            Onnx::InferenceEngine* ODModel = new Onnx::InferenceEngine(_od_enableTensortRt);

            qDebug() << "Loading Object Detection Model: "<<QString::fromStdString(ODModel->getModelID());
            ODModel->ct_loadModel(ObjectDetectionPath.toStdString(), _objectDetectionModelPaths[i]->text().toStdString());
            qDebug() << "Model Loaded successfully: " << QString::fromStdString(ODModel->getModelID());

         
            // warmup gpu
            cv::Mat warmupImage = cv::Mat::zeros(640, 640, CV_8UC3);
            //cv::Mat warmupImage = cv::imread("C:/Advanced/Data/recipe/GrianT/Images/VidiImages/ref_RGB.jpg");
           
            if (warmupImage.empty())
            {
                QMessageBox::warning(this, tr("Error warming up GPU"),
                    tr("Reference Image not found. Load Model Failed"));
                ODModel->ct_releaseModel();
                delete ODModel;
                ODModel = nullptr;
            }
            else
            {
                ct::logger::info("Warming up Object Detection Model...");    
                ODModel->ct_runModel(warmupImage,0.5,0.5);
                ct::logger::info("Warming up Object Detection model complete...");
                g_ODModels.append(ODModel);

                //int classSize = ODModel->getClassSize();
               int classSize = ODModel->getClassSize();
               qDebug() << "Class size: " << classSize;
               if (classSize > 50) classSize = 0; // prevent sizing when warming up fail;

               _objectDetectionClassSize.insert(_objectDetectionModelPaths[i]->text(), classSize);
            }     
        }
      
    }

    if (_objectDetectionModelPaths.size() > 0)
    {
        ui.label_modelStatus->setStyleSheet("color: green");
        ui.label_modelStatus->setText("Models loaded!");
    }
    saveODModelListJson();
}

void VisionApp::unloadODModels()
{
    ct::logger::info("Unload Model");
    for (int i = 0; i < g_ODModels.size(); i++)
    {
        qDebug() << "Unloading model: " << QString::fromStdString(g_ODModels[i]->getModelID());
        g_ODModels[i]->ct_releaseModel();
      
        delete g_ODModels[i];
        g_ODModels[i] = nullptr;
    }

    g_ODModels.clear();

    ui.label_modelStatus->setStyleSheet("color: red");
    ui.label_modelStatus->setText("Models are not loaded!");
}

void VisionApp::loadODModelListJson()
{
    if (_objectDetectionModelPaths.size() != _objectDetectionCheckBoxes.size()) return;

    auto jsonPath = Common::Directory::getRecipeCurrentPath() + "ODModelList.json";

    QJsonObject root;
    if (loadJson(jsonPath, root)) {
        auto modelList = jsonHelper::getArray(root, QStringLiteral("OD_model_list"));

        bool loadModel = false;
        for (int i = 0; i < modelList.size(); i++)
        {
            auto modelName = jsonHelper::getString(modelList[i].toObject(), QStringLiteral("model_name"));
            auto modelEnabled = jsonHelper::getBool(modelList[i].toObject(), QStringLiteral("enable"));

            for (int j = 0; j < _objectDetectionModelPaths.size(); j++)
            {
                if (_objectDetectionModelPaths[j]->text() == modelName)
                {
                    _objectDetectionCheckBoxes[j]->setChecked(modelEnabled);
                    if (modelEnabled) loadModel = true;
                    break;
                }
            }
        }

        g_odTilingSettings.enableTiling = jsonHelper::getBool(root, QStringLiteral("OD_tiling_enable"));
        g_odTilingSettings.tilingSize = jsonHelper::getInteger(root, QStringLiteral("OD_tiling_size"));
        g_odTilingSettings.tilingPaddingPerc = jsonHelper::getDouble(root, QStringLiteral("OD_tiling_padding_percentage"));
        g_odTilingSettings.tilingIou = jsonHelper::getDouble(root, QStringLiteral("OD_tiling_iou"));
      
        _od_enableTensortRt = jsonHelper::getBool(root, QStringLiteral("OD_Enable_TensorRT"));
        _seg_enableTensortRt = jsonHelper::getBool(root, QStringLiteral("Seg_Enable_TensorRT"));

        ui.checkBox_odEnableFastMode->setChecked(_od_enableTensortRt);
        ui.checkBox_segEnableFastMode->setChecked(_seg_enableTensortRt);

        auto segObj = jsonHelper::getObject(root, QStringLiteral("Segmentation"));
        _curSegmentationModel = jsonHelper::getString(segObj, QStringLiteral("segmentation_model"));
        ui.lineEdit_segmentationScore->setText(QString::number(jsonHelper::getDouble(segObj, QStringLiteral("segmentation_score"), 1.0)));
        ui.checkBox_loadSegmentationModel->setChecked(jsonHelper::getBool(segObj, QStringLiteral("segmenation_enable")));
       

        if (loadModel) loadODModels();
        if(_curSegmentationModel == "Tiny")	ui.radioButton_segTiny->setChecked(true);
        if (_curSegmentationModel == "Small")	ui.radioButton_segSmall->setChecked(true);
        if (_curSegmentationModel == "Medium")	ui.radioButton_segMedium->setChecked(true);
        if (_curSegmentationModel == "Large")	ui.radioButton_segLarge->setChecked(true);

    }

    ui.checkBox_odTiling->setChecked(g_odTilingSettings.enableTiling);
    ui.spinBox_odTilingSize->setValue(g_odTilingSettings.tilingSize);
    ui.doubleSpinBox_odTilingPaddingPerc->setValue(g_odTilingSettings.tilingPaddingPerc);
    ui.doubleSpinBox_odTilingIou->setValue(g_odTilingSettings.tilingIou);

    if (g_odTilingSettings.enableTiling) ui.frame_odTiling->show();
    else  ui.frame_odTiling->hide();

}

void VisionApp::saveODModelListJson()
{
    
    auto jsonPath = Common::Directory::getRecipeCurrentPath() + "ODModelList.json";
    qDebug() << "saveMODModelList.json:" << jsonPath;

    if (_objectDetectionModelPaths.size() != _objectDetectionCheckBoxes.size()) return;

    QJsonObject j_main;
    QJsonArray j_array;
    for (int i = 0; i < _objectDetectionModelPaths.size(); i++)
    {
        int classSize = _objectDetectionClassSize[_objectDetectionModelPaths[i]->text()];
       
        QJsonObject j_root;
        j_root.insert(QStringLiteral("model_name"), _objectDetectionModelPaths[i]->text());
        j_root.insert(QStringLiteral("enable"), _objectDetectionCheckBoxes[i]->isChecked());
        j_root.insert(QStringLiteral("class_size"), classSize);

        j_array.push_back(j_root);
    }

    j_main.insert(QStringLiteral("OD_tiling_enable"), g_odTilingSettings.enableTiling);
    j_main.insert(QStringLiteral("OD_tiling_size"), g_odTilingSettings.tilingSize);
    j_main.insert(QStringLiteral("OD_tiling_padding_percentage"), g_odTilingSettings.tilingPaddingPerc);
    j_main.insert(QStringLiteral("OD_tiling_iou"), g_odTilingSettings.tilingIou);
    j_main.insert(QStringLiteral("OD_Enable_TensorRT"), _od_enableTensortRt);
    j_main.insert(QStringLiteral("Seg_Enable_TensorRT"), _seg_enableTensortRt);
    QJsonObject j_segObj;
    j_segObj.insert(QStringLiteral("segmenation_enable"), ui.checkBox_loadSegmentationModel->isChecked());
    j_segObj.insert(QStringLiteral("segmentation_model"), _curSegmentationModel);
    j_segObj.insert(QStringLiteral("segmentation_score"), ui.lineEdit_segmentationScore->text().toDouble());

    j_main.insert(QStringLiteral("OD_model_list"), j_array);
    j_main.insert(QStringLiteral("Segmentation"), j_segObj);
 
    auto ret = saveJson(jsonPath, QJsonDocument(j_main));

    if (ret) showStatus(QStringLiteral("Successfully saved OD Model List!"));
    else showStatus(QStringLiteral("Failed to save OD Model List!"));
}

void VisionApp::oDModelCheckBoxChanged()
{
    QObject* senderObj = sender();
  /*  for (int i = 0; i < _objectDetectionCheckBoxes.size(); i++)
    {
        if (senderObj != _objectDetectionCheckBoxes[i])
        {
            _objectDetectionCheckBoxes[i]->setChecked(false);
        }
    }*/
    unloadODModels();
    saveODModelListJson();
    ui.comboBox_loadODModels->setCurrentText("Unload Models");
}

void VisionApp::clearGridLayout(QGridLayout* layout)
{
    if (!layout) return;

    while (layout->count() > 0) {
        // Get the item at the top
        QLayoutItem* item = layout->takeAt(0);

        // Check if the item is a widget
        if (QWidget* widget = item->widget()) {
            layout->removeWidget(widget);
            delete widget; // Delete the widget to prevent memory leak
        }

        // Check if the item is another layout
        if (QLayout* childLayout = item->layout()) {
            clearGridLayout(static_cast<QGridLayout*>(childLayout));
        }

        delete item; // Delete the layout item to prevent memory leak
    }
}

void VisionApp::loadSegmentationModel()
{
    unloadSegmentationModel();

    QString encoderPath;
    QString decoderPath;

    QString segDir = Common::Directory::LocalPath + "AIModel/Segmentation/";

    if (_curSegmentationModel == "Tiny")
    {
        encoderPath = segDir + "/tiny/tiny_preprocess.onnx";
        decoderPath = segDir + "/tiny/tiny.onnx";

    }
    else if (_curSegmentationModel == "Small")
    {
        encoderPath = segDir + "/small/small_preprocess.onnx";
        decoderPath = segDir + "/small/small.onnx";

      
    }
    else if (_curSegmentationModel == "Medium")
    {
        encoderPath = segDir + "/medium/medium_preprocess.onnx";
        decoderPath = segDir + "/medium/medium.onnx";


    }
    else if (_curSegmentationModel == "Large")
    {
        encoderPath = segDir + "/large/large_preprocess.onnx";
        decoderPath = segDir + "/large/large.onnx";

    }

    g_segModel = new Onnx::InferenceEngine(_seg_enableTensortRt);
    g_segModel->laodModel_segmentation(
        _curSegmentationModel.toStdString(),
        encoderPath.toStdString(),
        decoderPath.toStdString()
    );

}


void VisionApp::unloadSegmentationModel()
{
    if (g_segModel) {
        g_segModel->releaseModel_segmentation();
        delete g_segModel;
        g_segModel = nullptr;
    }
}