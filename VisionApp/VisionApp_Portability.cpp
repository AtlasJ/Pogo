#include "VisionApp.h"
#include "CAMManager.h"
#include "ScaleManager.h"
#include "Guided_2D3D_AlignmentTab.h"

void VisionApp::initPortability()
{
	ui.lineEdit_currentMachineName->setReadOnly(true);
	ui.lineEdit_currentMachineName->setText(QHostInfo::localHostName());
	QIntValidator* validator_minDiameter = new QIntValidator(0, 100000, ui.lineEdit_minDiameter);
	ui.lineEdit_minDiameter->setValidator(validator_minDiameter);
	ui.lineEdit_minDiameter->setText("280");

	QIntValidator* validator_maxDiameter = new QIntValidator(0, 100000, ui.lineEdit_maxDiameter);
	ui.lineEdit_maxDiameter->setValidator(validator_maxDiameter);
	ui.lineEdit_maxDiameter->setText("320 ");

	connect(ui.lineEdit_minDiameter, &QLineEdit::textEdited, this, [=](const QString &text) { updatePositionPortabilityInfos(); });
	connect(ui.lineEdit_maxDiameter, &QLineEdit::textEdited, this, [=](const QString &text) { updatePositionPortabilityInfos(); });

	_pGraphicsSceneFOV->addItem(&SystemData::instance()._portability.learn_region);

	_pGraphicsSceneFOV->addItem(&SystemData::instance()._portability.search_region);
	_pGraphicsSceneFOV->addItem(&SystemData::instance()._portability.located_region);

	auto setRegion = [=](const QRectF & rect, const QColor & color, const QString & name, const int & ZValue, QDragBox& db) {
		db.setOutterBarrier(_pGraphicsSceneFOV->sceneRect());
		db.setup(rect, color, name);
		db.setDragable(true);
		db.setZValue(ZValue);
	};

	auto pr = QRectF(_imageSize.width() /2 - _imageSize.width() * 0.1, _imageSize.height() / 2 - _imageSize.height() * 0.1, _imageSize.width() * 0.2, _imageSize.height() * 0.2);

	setRegion(pr, Qt::magenta, "Learning Region", 2, SystemData::instance()._portability.learn_region);
	connect(&SystemData::instance()._portability.learn_region, SIGNAL(dragBoxMouseReleased(QDragBox*, QString, QPointF)), this, SLOT(updatePositionPortabilityInfos()));
	connect(&SystemData::instance()._portability.learn_region, SIGNAL(grabberReleased(QDragBox*)), this, SLOT(updatePositionPortabilityInfos()));

	auto sr = QRectF(0, 0, _imageSize.width(), _imageSize.height());
	setRegion(sr, Qt::green, "Search Region",1, SystemData::instance()._portability.search_region);
	connect(&SystemData::instance()._portability.search_region, SIGNAL(dragBoxMouseReleased(QDragBox*, QString, QPointF)), this, SLOT(updatePositionPortabilityInfos()));
	connect(&SystemData::instance()._portability.search_region, SIGNAL(grabberReleased(QDragBox*)), this, SLOT(updatePositionPortabilityInfos()));

	updatePositionPortabilityInfos();

	auto lr = QRectF(0, 0, 10, 10);
	setRegion(lr, QColor(247, 227, 41), "Located Region", 1, SystemData::instance()._portability.located_region);
	SystemData::instance()._portability.located_region.setDragable(false);
	SystemData::instance()._portability.located_region.setZValue((int)UIHierarchy::VIEW);
	SystemData::instance()._portability.located_region.hide();
	ct::logger::info("1");
	showPortabilityPatternFeature(); ct::logger::info("2");
	loadRefPositionPortabilityInfo(); ct::logger::info("3");
	loadCurPositionPortabilityInfo(); ct::logger::info("4");
	togglePositionPortabilitySetupROIMode(ui.toolButton_portabilitySetupROI->isChecked());

	connect(ui.toolButton_portabilitySetupROI, &QToolButton::toggled, this, [=](bool state) {togglePositionPortabilitySetupROIMode(state); });
	connect(ui.toolButton_portabilityBaseImage, &QToolButton::clicked, this, [=]() { savePortabilityBaseImage(); });
	connect(ui.toolButton_portabilityLearnPattern, &QToolButton::clicked, this, [=]() { learnPortabilityPatternFeature(); });
	connect(ui.toolButton_portabilityFindPattern, &QToolButton::clicked, this, [=]() { testPortabilityPatternFeature(); });
	connect(ui.toolButton_portabilityFindCircle, &QToolButton::clicked, this, [=]() { testPortabilityCircleFeature(); });
	connect(ui.comboBox_FeatureLearning, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [=](int index) { 
		updatePortabilityFeatureUI(index);
		SystemData::instance()._portability.ref_info.feature_searching_method = index;
		SystemData::instance()._portability.current_info.feature_searching_method = index;
	});
	ct::logger::info("5");
	connect(ui.toolButton_setRefPoint, &QToolButton::clicked, this, [=]() { setPositionPortabilityPoint(PositionPortabilityType::REFERENCE); });
	connect(ui.toolButton_setCurPoint, &QToolButton::clicked, this, [=]() { setPositionPortabilityPoint(PositionPortabilityType::CURRENT); });
	connect(ui.toolButton_jogToRefPoint, &QToolButton::clicked, this, [=]() { jogToPositionPortability(PositionPortabilityType::REFERENCE); });
	connect(ui.toolButton_jogToCurrentPoint, &QToolButton::clicked, this, [=]() { jogToPositionPortability(PositionPortabilityType::CURRENT); });

	ui.frame_featureLearning->hide();

	ui.frame_mainReferencePortabilityPoint->hide();
	ui.toolButton_setRefPoint->hide();
	ui.toolButton_jogToRefPoint->hide();

	ui.toolButton_setCurPoint->hide();
	ui.toolButton_jogToCurrentPoint->hide();
}

void VisionApp::updatePositionPortabilityInfos()
{
	SystemData::instance()._portability.ref_info.id = "Ref_Portability_Info";

	SystemData::instance()._portability.ref_info.search_region.cx = SystemData::instance()._portability.search_region.getGeometry().center().x();
	SystemData::instance()._portability.ref_info.search_region.cy = SystemData::instance()._portability.search_region.getGeometry().center().y();
	SystemData::instance()._portability.ref_info.search_region.w = SystemData::instance()._portability.search_region.getGeometry().width();
	SystemData::instance()._portability.ref_info.search_region.h = SystemData::instance()._portability.search_region.getGeometry().height();
	SystemData::instance()._portability.ref_info.search_region.compute_extremum();

	SystemData::instance()._portability.ref_info.learn_region.cx = SystemData::instance()._portability.learn_region.getGeometry().center().x();
	SystemData::instance()._portability.ref_info.learn_region.cy = SystemData::instance()._portability.learn_region.getGeometry().center().y();
	SystemData::instance()._portability.ref_info.learn_region.w = SystemData::instance()._portability.learn_region.getGeometry().width();
	SystemData::instance()._portability.ref_info.learn_region.h = SystemData::instance()._portability.learn_region.getGeometry().height();
	SystemData::instance()._portability.ref_info.learn_region.compute_extremum();
	
	SystemData::instance()._portability.ref_info.min_diameter = ui.lineEdit_minDiameter->text().toDouble();
	SystemData::instance()._portability.ref_info.max_diameter = ui.lineEdit_maxDiameter->text().toDouble();;
}

void VisionApp::togglePositionPortabilitySetupROIMode(bool state)
{
	if (state) {
		auto cam_w = CAMManager::instance().getWidth(_camID);
		auto cam_h = CAMManager::instance().getHeight(_camID);

		if (SystemData::instance()._portability.learn_region.getGeometry().width() < 1 || SystemData::instance()._portability.learn_region.getGeometry().x() < 0 || SystemData::instance()._portability.learn_region.getGeometry().y() < 0) {
			SystemData::instance()._portability.learn_region.setGeometry(QRectF(cam_w / 2 - cam_w * 0.1, cam_h / 2 - cam_h * 0.1, cam_w * 0.2, cam_h * 0.2));
			SystemData::instance()._portability.search_region.setGeometry(QRectF(0, 0, cam_w, cam_h));
		}

		SystemData::instance()._portability.learn_region.show();
		SystemData::instance()._portability.search_region.show();
	 
		ui.toolButton_toggleFovView->animateClick();
		displayPortabilityFeatureImage();

		ui.toolButton_portabilitySetupROI->setChecked(true);
	}
	else {
		SystemData::instance()._portability.search_region.hide();
		SystemData::instance()._portability.learn_region.hide();

		ui.toolButton_portabilitySetupROI->setChecked(false);
	}

	showPortabilityPatternFeature();
	savePositionPortabilityInfo(PositionPortabilityType::REFERENCE);
}

void VisionApp::displayPortabilityFeatureImage()
{
	auto path_fid = Common::Directory::PortabilityPath() + QString("PortabilityBaseImage.jpg");

	if (QFile::exists(path_fid)) {
		displayFOV(QImage(path_fid));
		ui.graphicsViewFOV->fitInView(_pPixmapItemFOV, Qt::KeepAspectRatio);
		ui.graphicsViewFOV->centerOn(_pPixmapItemFOV);
	}
}

void VisionApp::savePortabilityBaseImage()
{
	snapImage(_mainOptics[_camID], "", "");

	auto path = Common::Directory::PortabilityPath() + QString("PortabilityBaseImage.jpg");
	_imageFOV.save(path);

	QMessageBox::information(this, tr("Save Base Image"),
		tr("Portability Base Image has been saved."));
}

void VisionApp::learnPortabilityPatternFeature()
{ 
	QImage qimg = _pixmapFOV.toImage();
	QImage croppedImage = qimg.copy(
		SystemData::instance()._portability.learn_region.getGeometry().x()
		, SystemData::instance()._portability.learn_region.getGeometry().y()
		, SystemData::instance()._portability.learn_region.getGeometry().width()
		, SystemData::instance()._portability.learn_region.getGeometry().height());

	auto path_feature = Common::Directory::PortabilityPath() + "PortabilityFeature.jpg";
	croppedImage.save(path_feature);

	MIL_ID mBuf = mtrx::to_milID(croppedImage);
	MIL_ID mMono = mtrx::to_mono(mBuf);
	mtrx::PatternInput input;
	mtrx::PatternOutput output;

	auto w = mtrx::get_width(mMono);
	auto h = mtrx::get_height(mMono);
	input.learn_x = 0;
	input.learn_y = 0;
	input.learn_w = w;
	input.learn_h = h;
	input.min_score = 50;
	input.angle_step = 0.0;

	//learn Geometry Model
	input.filename = QString(Common::Directory::PortabilityPath() + "PortabilityFeature.mod").toStdString();
	mtrx::learn_pattern(mMono, input, output);

	mtrx::free_buffer(mBuf);
	mtrx::free_buffer(mMono);

	showPortabilityPatternFeature();

	togglePositionPortabilitySetupROIMode(false);
	updateSetupCheckList();

	auto modelPath = Common::Directory::PortabilityPath() + "PortabilityFeature.mod";
	if (QFileInfo::exists(modelPath))
	{
		ui.lineEdit_featureLearningStatus->setText("Pattern Learnt"); // Set text
		ui.lineEdit_featureLearningStatus->setStyleSheet("color: green;"); // Set text color to green
	}
}

void VisionApp::showPortabilityPatternFeature()
{
	auto path_feature = Common::Directory::PortabilityPath() + "PortabilityFeature.jpg";
	if (QFile::exists(path_feature)) {
		QPixmap pixmap = QPixmap(path_feature).scaledToHeight(300, Qt::SmoothTransformation);
		ui.label_positionPortabilityFeature->setPixmap(pixmap);
	}
	else {
		ui.label_positionPortabilityFeature->setPixmap(QPixmap());
	}
}

bool VisionApp::testPortabilityPatternFeature()
{
	emit signalFindPortabilityPattern();
	return true;
}

bool VisionApp::testPortabilityCircleFeature()
{
	emit signalFindPortabilityCircle();
	return true;
}

bool VisionApp::savePositionPortabilityInfo(PositionPortabilityType type)
{
	if (type == PositionPortabilityType::REFERENCE)
	{
		auto jsonPath = Common::Directory::PortabilityPath() + "RefPositionPortability.json";

		QJsonObject j_root;
		QJsonArray j_array;

		QJsonObject obj;
		QJsonObject srObj, prObj;
		SystemData::instance()._portability.ref_info.search_region.compute_extremum();
		SystemData::instance()._portability.ref_info.learn_region.compute_extremum();
		toJson(SystemData::instance()._portability.ref_info.search_region, srObj);
		toJson(SystemData::instance()._portability.ref_info.learn_region, prObj);

		obj.insert(QStringLiteral("id"), SystemData::instance()._portability.ref_info.id);
		obj.insert(QStringLiteral("search_region"), srObj);
		obj.insert(QStringLiteral("learn_region"), prObj);
		obj.insert(QStringLiteral("feature_searching_method"), SystemData::instance()._portability.ref_info.feature_searching_method);
		obj.insert(QStringLiteral("min_diameter"), SystemData::instance()._portability.ref_info.min_diameter);
		obj.insert(QStringLiteral("max_diameter"), SystemData::instance()._portability.ref_info.max_diameter);
		obj.insert(QStringLiteral("width"), SystemData::instance()._portability.ref_info.width);
		obj.insert(QStringLiteral("height"), SystemData::instance()._portability.ref_info.height);
		//obj.insert(QStringLiteral("machine_name"), SystemData::instance()._portability.ref_info.machine_name);
		obj.insert(QStringLiteral("machine_name"), "Reference_Machine");
		obj.insert(QStringLiteral("PIC"), SystemData::instance()._portability.ref_info.PIC);
		obj.insert(QStringLiteral("date_created"), SystemData::instance()._portability.ref_info.date_created);
		obj.insert(QStringLiteral("learnt_status"), SystemData::instance()._portability.ref_info.learnt_status);

		QJsonObject pObj;
		toJson(SystemData::instance()._portability.ref_info.portability_point, pObj, true);
		obj.insert(QStringLiteral("ref_portability_point"), pObj);

		j_root.insert(QStringLiteral("RefPositionPortabilityInfos"), obj);

		auto ret = saveJson(jsonPath, QJsonDocument(j_root));

		if (ret) showStatus(QStringLiteral("Successfully saved position portability info!"));
		else showStatus(QStringLiteral("Failed to save position portability info!"));

		return ret;
	}
	else
	{
		auto jsonPath = Common::Directory::PortabilityPath() + "CurPositionPortability.json";
		if (Common::Directory::CurrentRecipe.contains("subrecipe1"))
		{
			jsonPath = Common::Directory::PortabilityPath() + "CurPositionPortability-subRecipe.json";
		}

		QJsonObject j_root;
		QJsonArray j_array;

		QJsonObject obj;

		obj.insert(QStringLiteral("id"), SystemData::instance()._portability.current_info.id);
		obj.insert(QStringLiteral("machine_name"), SystemData::instance()._portability.current_info.machine_name);
		obj.insert(QStringLiteral("PIC"), SystemData::instance()._portability.current_info.PIC);
		obj.insert(QStringLiteral("date_created"), SystemData::instance()._portability.current_info.date_created);

		SystemData::instance()._portability.current_info.offset_point =
			SystemData::instance()._portability.current_info.portability_point -
			SystemData::instance()._portability.ref_info.portability_point;

		QJsonObject pObj;
		toJson(SystemData::instance()._portability.current_info.portability_point, pObj, true);
		obj.insert(QStringLiteral("cur_portability_point"), pObj);

		QJsonObject offsetObj;
		toJson(SystemData::instance()._portability.current_info.offset_point, offsetObj, true);
		obj.insert(QStringLiteral("cur_offset"), offsetObj);

		j_root.insert(QStringLiteral("CurPositionPortabilityInfos"), obj);

		auto ret = saveJson(jsonPath, QJsonDocument(j_root));

		if (ret) showStatus(QStringLiteral("Successfully saved position portability info!"));
		else showStatus(QStringLiteral("Failed to save position portability info!"));

		return ret;
	}
	
}

bool VisionApp::loadRefPositionPortabilityInfo(QString refPortabilityPath)
{
	ct::logger::debug("Load Ref Position Portability Info");

	QString portabilityPath = Common::Directory::PortabilityPath();
	if (refPortabilityPath != QString()) portabilityPath = refPortabilityPath;
	auto jsonPath = portabilityPath + "RefPositionPortability.json";
	qDebug() << "loadRefPortabilityInfo:" << jsonPath;
	QJsonObject root;

	//guard
	if (!loadJson(jsonPath, root)) {
		auto cam_w = CAMManager::instance().getWidth(_camID);
		auto cam_h = CAMManager::instance().getHeight(_camID);

		SystemData::instance()._portability.ref_info.learn_region.cx = cam_w / 2;
		SystemData::instance()._portability.ref_info.learn_region.cy = cam_h / 2;
		SystemData::instance()._portability.ref_info.learn_region.w = cam_w * 0.2;
		SystemData::instance()._portability.ref_info.learn_region.h = cam_h * 0.2;
		SystemData::instance()._portability.ref_info.learn_region.compute_extremum();
		SystemData::instance()._portability.learn_region.setGeometry(QRectF(cam_w / 2 - cam_w * 0.1, cam_h / 2 - cam_h * 0.1, cam_w * 0.2, cam_h * 0.2));

		SystemData::instance()._portability.ref_info.search_region.cx = cam_w / 2;
		SystemData::instance()._portability.ref_info.search_region.cy = cam_h / 2;
		SystemData::instance()._portability.ref_info.search_region.w = cam_w;
		SystemData::instance()._portability.ref_info.search_region.h = cam_h;
		SystemData::instance()._portability.ref_info.search_region.compute_extremum();
		SystemData::instance()._portability.search_region.setGeometry(QRectF(SystemData::instance()._portability.ref_info.search_region.xmin, SystemData::instance()._portability.ref_info.search_region.ymin, SystemData::instance()._portability.ref_info.search_region.w, SystemData::instance()._portability.ref_info.search_region.h));	
	}


	if (!root.contains("RefPositionPortabilityInfos")) return false;
	auto pInfos = root["RefPositionPortabilityInfos"].toObject();

	SystemData::instance()._portability.ref_info.id = jsonHelper::getString(pInfos, QStringLiteral("id"));
	//SystemData::instance()._portability.ref_info.machine_name = jsonHelper::getString(pInfos, QStringLiteral("machine_name"));
	SystemData::instance()._portability.ref_info.machine_name = "ReferenceMachine";
	SystemData::instance()._portability.ref_info.PIC = jsonHelper::getString(pInfos, QStringLiteral("PIC"));
	SystemData::instance()._portability.ref_info.date_created = jsonHelper::getString(pInfos, QStringLiteral("date_created"));
	SystemData::instance()._portability.ref_info.feature_searching_method = jsonHelper::getInteger(pInfos, QStringLiteral("feature_searching_method"), 0);
	SystemData::instance()._portability.ref_info.min_diameter = jsonHelper::getDouble(pInfos, QStringLiteral("min_diameter"), 280);
	SystemData::instance()._portability.ref_info.max_diameter = jsonHelper::getDouble(pInfos, QStringLiteral("max_diameter"), 320);
	//SystemData::instance()._portability.ref_info.learnt_status = jsonHelper::getBool(pInfos, QStringLiteral("learnt_status"), true);
	SystemData::instance()._portability.ref_info.learnt_status = true;

	if (SystemData::instance()._portability.ref_info.learnt_status)
	{
		ui.lineEdit_refPointLearntStatus->setText("Ref Portability Point Learnt");
		ui.lineEdit_refPointLearntStatus->setStyleSheet("color: green;"); // Set text color to green
		ui.toolButton_setRefPoint->setDisabled(true);
	}
	else
	{
		ui.lineEdit_refPointLearntStatus->setText("Ref Portability Point Not Learnt");
		ui.lineEdit_refPointLearntStatus->setStyleSheet("color: red;"); // Set text color to green
		ui.toolButton_setRefPoint->setDisabled(false);
	}

	fromJson(pInfos["search_region"].toObject(), SystemData::instance()._portability.ref_info.search_region);
	fromJson(pInfos["learn_region"].toObject(), SystemData::instance()._portability.ref_info.learn_region);
	fromJson(pInfos["ref_portability_point"].toObject(), SystemData::instance()._portability.ref_info.portability_point, true);
	SystemData::instance()._portability.ref_info.portability_point.wx = 0;
	SystemData::instance()._portability.ref_info.portability_point.wy = 0;
	SystemData::instance()._portability.ref_info.portability_point.wz = 0;

	ui.lineEdit_refPointStatus->setText(QString("X:%1   Y:%2   Z:%3")
		.arg(SystemData::instance()._portability.ref_info.portability_point.wx)
		.arg(SystemData::instance()._portability.ref_info.portability_point.wy)
		.arg(SystemData::instance()._portability.ref_info.portability_point.wz));

	ui.lineEdit_refPointLearntFromMachine->setText(SystemData::instance()._portability.ref_info.machine_name);
	ui.lineEdit_refPointDateCreated->setText(SystemData::instance()._portability.ref_info.date_created);
	ui.lineEdit_refPointPIC->setText(SystemData::instance()._portability.ref_info.PIC);

	ui.comboBox_FeatureLearning->setCurrentIndex(SystemData::instance()._portability.ref_info.feature_searching_method);

	SystemData::instance()._portability.learn_region.setGeometry(QRectF(SystemData::instance()._portability.ref_info.learn_region.xmin, SystemData::instance()._portability.ref_info.learn_region.ymin, SystemData::instance()._portability.ref_info.learn_region.w, SystemData::instance()._portability.ref_info.learn_region.h));
	SystemData::instance()._portability.search_region.setGeometry(QRectF(SystemData::instance()._portability.ref_info.search_region.xmin, SystemData::instance()._portability.ref_info.search_region.ymin, SystemData::instance()._portability.ref_info.search_region.w, SystemData::instance()._portability.ref_info.search_region.h));

	ui.lineEdit_minDiameter->setText(QString::number(SystemData::instance()._portability.ref_info.min_diameter));
	ui.lineEdit_maxDiameter->setText(QString::number(SystemData::instance()._portability.ref_info.max_diameter));

	auto modelPath = Common::Directory::PortabilityPath() + "PortabilityFeature.mod";
	if (QFileInfo::exists(modelPath))
	{
		ui.lineEdit_featureLearningStatus->setText("Pattern Learnt"); // Set text
		ui.lineEdit_featureLearningStatus->setStyleSheet("color: green;"); // Set text color to green
	}
	else
	{
		ui.lineEdit_featureLearningStatus->setText("Pattern Not Learnt"); // Set text
		ui.lineEdit_featureLearningStatus->setStyleSheet("color: red;"); // Set text color to green
	}
	return true;
}

bool VisionApp::loadCurPositionPortabilityInfo(QString curPortabilityPath)
{
	ct::logger::debug("Load Cur Position Portability Info");

	if (SystemData::instance()._portability.ref_info.machine_name == QHostInfo::localHostName())
	{
		ui.toolButton_setCurPoint->setDisabled(true);
		ui.toolButton_jogToCurrentPoint->setDisabled(true);
		return false;
	}

	QString portabilityPath = Common::Directory::PortabilityPath();
	if (curPortabilityPath != QString()) portabilityPath = curPortabilityPath;
	auto jsonPath = portabilityPath + "CurPositionPortability.json";
	qDebug() << "loadCurPositionPortabilityInfo:" << jsonPath;

	ui.label_105->setText("Current Portability Point - Main Recipe");
	if (Common::Directory::CurrentRecipe.contains("subrecipe1"))
	{
		jsonPath = portabilityPath + "CurPositionPortability-subRecipe.json";
		ui.label_105->setText("Current Portability Point - Sub Recipe 1");
	}

	auto& currentInfo = SystemData::instance()._portability.current_info;

	QJsonObject root;
	if (!loadJson(jsonPath, root))
	{
		currentInfo = PositionPortabilityInfo();
		ct::logger::error("Failed to load current position portability info");
		return false;
	}

	if (!root.contains("CurPositionPortabilityInfos")) {
		currentInfo = PositionPortabilityInfo();
		ct::logger::error("Root object in current position portability info not found");
		return false;
	}
	auto pInfos = root["CurPositionPortabilityInfos"].toObject();

	currentInfo.id = jsonHelper::getString(pInfos, QStringLiteral("id"));
	currentInfo.machine_name = jsonHelper::getString(pInfos, QStringLiteral("machine_name"));
	currentInfo.PIC = jsonHelper::getString(pInfos, QStringLiteral("PIC"));
	currentInfo.date_created = jsonHelper::getString(pInfos, QStringLiteral("date_created"));

	fromJson(pInfos["cur_portability_point"].toObject(), currentInfo.portability_point, true);
	ui.lineEdit_setCurPointStatus->setText(QString("X:%1   Y:%2   Z:%3")
		.arg(currentInfo.portability_point.wx)
		.arg(currentInfo.portability_point.wy)
		.arg(currentInfo.portability_point.wz));

	currentInfo.offset_point =
		currentInfo.portability_point -
		SystemData::instance()._portability.ref_info.portability_point;

	ui.lineEdit_offsetFromRefPoint->setText(QString("(offset) X:%1   Y:%2   Z:%3")
		.arg(currentInfo.offset_point.wx)
		.arg(currentInfo.offset_point.wy)
		.arg(currentInfo.offset_point.wz));

	ui.lineEdit_curPointLearntFromMachine->setText(currentInfo.machine_name);
	ui.lineEdit_CurPointDateCreated->setText(currentInfo.date_created);
	ui.lineEdit_curPointPIC->setText(currentInfo.PIC);

	qDebug() << "Machine name: " << currentInfo.machine_name;
	if (currentInfo.machine_name == QHostInfo::localHostName())
	{
		ui.lineEdit_curPointLearntStatus->setText("Cur Portability Point Learnt from current machine");
		ui.lineEdit_curPointLearntStatus->setStyleSheet("color: green;"); // Set text color to green
		ui.toolButton_setCurPoint->setDisabled(true);
	}
	else
	{
		ui.lineEdit_curPointLearntStatus->setText("Cur Portability Point Not Learnt from current machine");
		ui.lineEdit_curPointLearntStatus->setStyleSheet("color: red;"); // Set text color to green
		ui.toolButton_setCurPoint->setDisabled(false);
	}

	return true;
}

void VisionApp::getCurrentMachinePortabilityPointOffset(dat::WorldCoordinate & offset)
{
	if (SystemData::instance()._portability.ref_info.machine_name == QHostInfo::localHostName())
	{
		offset = dat::WorldCoordinate();
	}
	else if (SystemData::instance()._portability.current_info.machine_name == QHostInfo::localHostName())
	{
		offset = SystemData::instance()._portability.current_info.portability_point - SystemData::instance()._portability.ref_info.portability_point;
		qDebug() << "[getCurrentMachinePortabilityPointOffset] portability_point:" << offset.wx << "," << offset.wy << "," << offset.wz;
	}
}

dat::WorldCoordinate VisionApp::getAbsoluteRobotPoint(dat::WorldCoordinate point)
{
	dat::WorldCoordinate offset;
	getCurrentMachinePortabilityPointOffset(offset);
	dat::WorldCoordinate absolutePoint = point + offset;
	return absolutePoint;
}

QPointF VisionApp::getAbsoluteFOVCoordinates(const QPointF & FOVcoordinates)
{
	auto scale = ScaleManager::instance().um_per_px();

	dat::WorldCoordinate offset;
	getCurrentMachinePortabilityPointOffset(offset);
	auto offsetX = util::mm_to_px(offset.wx, scale);
	auto offsetY = util::mm_to_px(offset.wy, scale);
	QPointF absoluteCoordinates = FOVcoordinates + QPointF(offsetX, offsetY);
	return absoluteCoordinates;
}

QPointF VisionApp::getAbsoluteWorldCoordinates(const QPointF & Worldcoordinates)
{
	dat::WorldCoordinate offset;
	getCurrentMachinePortabilityPointOffset(offset);
	auto worldOffset = ScaleManager::instance().to_world_px(QPointF(offset.wx, offset.wy));
	QPointF absoluteCoordinates = Worldcoordinates + worldOffset;
	return absoluteCoordinates;
}

dat::WorldCoordinate VisionApp::getRelativeRobotPoint(dat::WorldCoordinate point)
{
	dat::WorldCoordinate offset;
	getCurrentMachinePortabilityPointOffset(offset);
	dat::WorldCoordinate relativePoint = point - offset;
	return relativePoint;
}

QPointF VisionApp::getRelativeFOVCoordinates(const QPointF & FOVcoordinates)
{
	auto scale = ScaleManager::instance().um_per_px();

	dat::WorldCoordinate offset;
	getCurrentMachinePortabilityPointOffset(offset);
	auto offsetX = util::mm_to_px(offset.wx, scale);
	auto offsetY = util::mm_to_px(offset.wy, scale);
	QPointF relativeCoordinates = FOVcoordinates - QPointF(offsetX, offsetY);
	return relativeCoordinates;
}

QPointF VisionApp::getRelativeWorldCoordinates(const QPointF & Worldcoordinates)
{
	dat::WorldCoordinate offset;
	getCurrentMachinePortabilityPointOffset(offset);
	auto worldOffset = ScaleManager::instance().to_world_px(QPointF(offset.wx, offset.wy));
	QPointF relativeCoordinates = Worldcoordinates - worldOffset;
	return relativeCoordinates;
}

void VisionApp::setPositionPortabilityPoint(PositionPortabilityType type)
{
	QString typestr;
	if (type == PositionPortabilityType::REFERENCE) typestr = "REFERENCE";
	else if (type == PositionPortabilityType::CURRENT) typestr = "CURRENT";
	QString msg = "Are you sure you want to set the " + typestr + " portability point? Click Yes to confirm set " + typestr + " portability point";
	QMessageBox::StandardButton reply = QMessageBox::question(this, "Set Portability Point", msg, QMessageBox::Yes | QMessageBox::No);
	if (reply == QMessageBox::No)
	{
		return;
	}

	QString PIC = QInputDialog::getText(this, tr("PIC"), tr("Person In Charge:"));

	if (type == PositionPortabilityType::REFERENCE) // save XYZ offset for reference portability
	{
		SystemData::instance()._portability.ref_info.PIC = PIC;
	}
	else if (type == PositionPortabilityType::CURRENT) //correct Z offset
	{
		SystemData::instance()._portability.current_info.PIC = PIC;
	}
	
	emit signalSetPortabilityPoint(type);
}

void VisionApp::jogToPositionPortability(PositionPortabilityType type)
{
	if (type == PositionPortabilityType::REFERENCE)
	{
		const auto& p = SystemData::instance()._portability.ref_info.portability_point;
		jogSnap(p.wx, p.wy, p.wz, _mainOptics[_camID]);
	}
	else if (type == PositionPortabilityType::CURRENT)
	{ 
		const auto& p = SystemData::instance()._portability.current_info.portability_point ;
		jogSnap(p.wx, p.wy, p.wz, _mainOptics[_camID]);
	}
}

em::V2d VisionApp::getPositionPortabilityPointInMM(int x_px, int y_px)
{
	auto h_scale = ScaleManager::instance().horizontal_um_per_px();
	auto v_scale = ScaleManager::instance().vertical_um_per_px();

	em::V2d point;
	printf("px: %d, %d\n", x_px, y_px);
	printf("mm: %f, %f\n", util::px_to_mm(x_px, h_scale), util::px_to_mm(y_px, v_scale));

	point.x() = SystemData::instance().currentCoordinate().wx + util::px_to_mm(x_px, h_scale);
	point.y() = SystemData::instance().currentCoordinate().wy + util::px_to_mm(y_px, v_scale);

	//printf("fid: %d, %d\n", _fiducialInfos[index].inspect_region.cx, _fiducialInfos[index].inspect_region.cy);
	printf("point: %f, %f\n", point.x(), point.y());
	return point;
}

bool VisionApp::getPortabilitySizeDifference(double difference, double & offsetZ)
{
	//need to verify if Z positive is going up, or going down
	if (abs(difference) > 1)
	{
		//if current size is more zoomed in than ref size move camera upwards
		if (difference < 0) offsetZ = 0.01;

		//if current size is less zoomed in than ref size move camera downwards
		if (difference >= 0) offsetZ = -0.01;

		return true;
	}
	else return false;
}

void VisionApp::guidedAlignPositionPortabilitySetup()
{
	//prompt message box ask to draw region of interest for alignment
	QMessageBox::StandardButton reply = QMessageBox::question(this, "Draw Alignment ROI", "Please draw a region of interest for alignment on the graphic scene.\n\nClick and drag on the image to define the alignment area.", QMessageBox::Yes | QMessageBox::No);

	if (reply == QMessageBox::Yes)
	{
		guidedPositionPortabilityMode();
	}
}
