#include "VisionApp.h"
#include <QLabel>
#include "CAMManager.h"

bool VisionApp::barcodeExistTest(int index)
{
	if (index > 1) return false;

	auto barcodeJsonPath = QStringLiteral("%1recipe/%2/barcode.json").arg(Common::Directory::LocalPath).arg(Common::Directory::CurrentRecipe);
	if (!QFileInfo(barcodeJsonPath).exists()) return false;

	qDebug() << "_barcodeInfos[index].hasTeachPoint:" << _barcodeInfos[index].hasTeachPoint;
	return _barcodeInfos[index].hasTeachPoint;
}

void VisionApp::initBarcode()
{
	QStringList imageChannels;
	imageChannels << "Red" << "Green" << "Blue" << "Grayscale";
	ui.comboBox_imageChannel->addItems(imageChannels);

	QStringList foregroundTypes;
	foregroundTypes << "Any" << "Black" << "White";
	ui.comboBox_foregroundType->addItems(foregroundTypes);

	QStringList barcodeTypes;
	barcodeTypes << "Code39" << "Code93" << "Code128" << "DataMatrix" << "Aztec" << "QRCode" << "MicroQRCode";
	ui.comboBox_barcodeType->addItems(barcodeTypes);

	QStringList recognitionTypes;
	recognitionTypes << "Improved" << "Typical";
	ui.comboBox_recognitionType->addItems(recognitionTypes);

	ui.comboBox_barcodeRegistrationMethod->clear();
	ui.comboBox_barcodeRegistrationMethod->addItems(QStringList()
		<< "Handheld"
		<< "Camera"
		<< "External"
	);

	_pGraphicsSceneFOV->addItem(&_barcodeSearchRegion);
	_pGraphicsSceneFOV->addItem(&_barcodeLocatedRegion);

	auto setRegion = [=](const QRectF & rect, const QColor & color, const QString & name, QDragBox& db) {
		db.setOutterBarrier(_pGraphicsSceneFOV->sceneRect());
		db.setup(rect, color, name);
		db.setDragable(true);
		db.setZValue((int)UIHierarchy::DRAGGABLES);
	};

	auto sr = QRectF(0, 0, _imageSize.width(), _imageSize.height());
	setRegion(sr, QColor(0, 255, 127), "Search Region", _barcodeSearchRegion);
	updateBarcodeSearchRegion();

	connect(&_barcodeSearchRegion, SIGNAL(dragBoxMouseReleased(QDragBox*, QString, QPointF)), this, SLOT(updateBarcodeSearchRegion()));
	connect(&_barcodeSearchRegion, SIGNAL(grabberReleased(QDragBox*)), this, SLOT(updateBarcodeSearchRegion()));

	auto lr = QRectF(0, 0, 10, 10);
	setRegion(lr, QColor(247, 227, 41), "Located Region", _barcodeLocatedRegion);

	_barcodeLocatedRegion.setDragable(false);
	_barcodeLocatedRegion.setZValue((int)UIHierarchy::VIEW);
	_barcodeLocatedRegion.hide();

	//_barcodeSearchRegion
	connect(ui.toolButton_barcode, &QToolButton::clicked, this, [=]() { showBarcode(0); });
	connect(ui.toolButton_Barcode2, &QToolButton::clicked, this, [=]() {showBarcode(1);  });
	connect(ui.toolButton_ReadBarcode, &QToolButton::clicked, this, [=]() { readBarcode(_currentBarcodeIndex); });
	connect(ui.toolButton_readBarcodeOffline, &QToolButton::clicked, this, [=]() { readBarcode(_currentBarcodeIndex, false); });
	connect(ui.toolButton_setupBarcodeROI, &QToolButton::toggled, this, [=](bool state) {toggleBarcodeROISetupMode(state); saveBarcode(); });

	connect(ui.comboBox_imageChannel, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [=](int index) { updateBarcodeSettingsJson(_currentBarcodeIndex);});
	connect(ui.comboBox_foregroundType, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [=](int index) { updateBarcodeSettingsJson(_currentBarcodeIndex); });
	connect(ui.comboBox_barcodeType, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [=](int index) { updateBarcodeSettingsJson(_currentBarcodeIndex); });
	connect(ui.comboBox_recognitionType, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [=](int index) { updateBarcodeSettingsJson(_currentBarcodeIndex); });

	connect(ui.checkBox_enablebarcode, &QCheckBox::toggled, this, [=]() {
		_enableBarcode = ui.checkBox_enablebarcode->isChecked(); 
		_jobThread.enableBarcode(_enableBarcode);
		saveBarcode();
	});

	connect(ui.comboBox_barcodeRegistrationMethod,
		QOverload<int>::of(&QComboBox::currentIndexChanged),
		this,
		[=](int methodIdx)
		{
			if (_currentBarcodeIndex > 1) return;

			_barcodeInfos[_currentBarcodeIndex].registration_method = methodIdx;

			showBarcodeRegistrationMethodUI(methodIdx); // show/hide UI for this barcode
			saveBarcode();
		}
	);



}

void VisionApp::showBarcode(int index)
{
	ui.frame_fiducial->hide();
	ui.frame_barcode->show();

	_currentTeachPointType = TeachPointType::BARCODE;
	_currentBarcodeIndex = index;

	QSignalBlocker sb(ui.comboBox_barcodeRegistrationMethod);
	ui.comboBox_barcodeRegistrationMethod->setCurrentIndex(_barcodeInfos[index].registration_method);
	showBarcodeRegistrationMethodUI(_barcodeInfos[index].registration_method);

	if (index == 0) {
		ui.toolButton_fid1->setChecked(false);
		ui.toolButton_barcode->setChecked(true);
		ui.toolButton_Barcode2->setChecked(false);
	}
	else {
		ui.toolButton_fid1->setChecked(false);
		ui.toolButton_barcode->setChecked(false);
		ui.toolButton_Barcode2->setChecked(true);
	}

	//load barcode view teach point
	if (_barcodeInfos.size() > 0)
	{
		auto& barcode = _barcodeInfos[index];
		ui.lineEdit_teachPoint->setText(QString("X: %1	Y: %2	Z: %3")
			.arg(barcode.teach_point.wx)
			.arg(barcode.teach_point.wy)
			.arg(barcode.teach_point.wz));

		//load barcode view image
		QString fileIndex = QString::number(index + 1);

		auto path_fid = Common::Directory::CurrentImageSetPath + QString("barcode%1.jpg").arg(fileIndex);
		/*if (!QFile::exists(path_fid)) auto path_fid = Common::Directory::getRecipeImagesPath() + QString("Barcode/barcode%1.jpg").arg(fileIndex);*/

		if (!QFile::exists(path_fid))
			path_fid = Common::Directory::getRecipeImagesPath() + QString("Barcode/barcode%1.jpg").arg(fileIndex);

		if (QFile::exists(path_fid)) {
			auto desired_width = ui.label_teachPoint->width();
			QPixmap pixmap = QPixmap(path_fid).scaledToWidth(300, Qt::SmoothTransformation);
			ui.label_teachPoint->setPixmap(pixmap);
		}
		else {
			ui.label_teachPoint->setPixmap(QPixmap());
		}

		toggleBarcodeROISetupMode(false);
		toggleFidROISetupMode(false);

		auto& sr = barcode.search_region;
		sr.compute_extremum();
	
		_barcodeSearchRegion.setGeometry(QRectF(sr.xmin, sr.ymin, sr.w, sr.h));

		updateBarcodeSettingsUI(index);
	}
}

void VisionApp::showBarcodeDebugImage(int index)
{
	if (index > 1) {
		ct::logger::error("[UB] Invalid barcode index: %d\n", index);
		return;
	}

	auto path_debugImage = Common::Directory::getRecipeImagesPath() + QString("Barcode/debugImage%1.jpg").arg(index + 1);
	if (QFile::exists(path_debugImage)) {
		QPixmap pixmap = QPixmap(path_debugImage).scaledToHeight(250, Qt::SmoothTransformation);
		ui.label_barcode->setPixmap(pixmap);
	}
	else {
		ui.label_barcode->setPixmap(QPixmap());
	}
}

bool VisionApp::saveBarcode()
{
	auto jsonPath = QStringLiteral("%1recipe/%2/barcode.json").arg(Common::Directory::LocalPath).arg(Common::Directory::CurrentRecipe);

	QJsonObject j_root;
	QJsonArray j_array;
	int index = 1;
	for (auto barcode : _barcodeInfos) {
		QJsonObject obj;
		barcode.id = "barcode" + QString::number(index);

		QJsonObject srObj;
		barcode.search_region.compute_extremum();
		toJson(barcode.search_region, srObj);

		QJsonObject tObj;
		toJson(barcode.teach_point, tObj);

		obj.insert(QStringLiteral("id"), barcode.id);
		obj.insert(QStringLiteral("teach_point"), tObj);
		obj.insert(QStringLiteral("teach_point_flag"), barcode.hasTeachPoint);
		obj.insert(QStringLiteral("search_region"), srObj);
		obj.insert(QStringLiteral("image_channel"), barcode.image_channel);
		obj.insert(QStringLiteral("foreground_type"), barcode.foreground_type);
		obj.insert(QStringLiteral("barcode_type"), barcode.barcode_type);
		obj.insert(QStringLiteral("recognition_type"), barcode.recognition_type);
		obj.insert(QStringLiteral("registration_method"), barcode.registration_method);

		j_array.append(obj);
		index++;
	}

	//need to add enable barcode UI
	j_root.insert(QStringLiteral("enable_barcode"), _enableBarcode);
	j_root.insert(QStringLiteral("barcodes"), j_array);

	auto ret = saveJson(jsonPath, QJsonDocument(j_root));

	if (ret) showStatus(QStringLiteral("Successfully saved barcode!"));
	else showStatus(QStringLiteral("Failed to save barcode!"));

	return ret;
}

bool VisionApp::loadBarcode()
{
	auto jsonPath = QStringLiteral("%1recipe/%2/barcode.json")
		.arg(Common::Directory::LocalPath)
		.arg(Common::Directory::CurrentRecipe);

	QJsonObject root;

	// guard
	if (!loadJson(jsonPath, root)) {
		for (auto& barcode : _barcodeInfos) {
			BarcodeInfo b;
			barcode = b;

			auto w = CAMManager::instance().getWidth(_camID);
			auto h = CAMManager::instance().getHeight(_camID);

			barcode.search_region.cx = w / 2;
			barcode.search_region.cy = h / 2;
			barcode.search_region.w = w;
			barcode.search_region.h = h;
			barcode.search_region.compute_extremum();
			_barcodeSearchRegion.setGeometry(QRectF(barcode.search_region.xmin, barcode.search_region.ymin,
				barcode.search_region.w, barcode.search_region.h));
		}
		return false;
	}
	int rootFallbackMethod = jsonHelper::getInteger(root, QStringLiteral("registration_method"), 1);

	if (!root.contains("barcodes")) return false;
	auto barcodes = root["barcodes"].toArray();

	int n = std::min((int)barcodes.size(), (int)_barcodeInfos.size());

	for (int i = 0; i < n; i++) {
		auto barcode = barcodes[i].toObject();

		_barcodeInfos[i].id = jsonHelper::getString(barcode, QStringLiteral("id"));
		_barcodeInfos[i].image_channel = jsonHelper::getInteger(barcode, QStringLiteral("image_channel"));
		_barcodeInfos[i].foreground_type = jsonHelper::getInteger(barcode, QStringLiteral("foreground_type"));
		_barcodeInfos[i].barcode_type = jsonHelper::getInteger(barcode, QStringLiteral("barcode_type"));
		_barcodeInfos[i].recognition_type = jsonHelper::getInteger(barcode, QStringLiteral("recognition_type"));
		_barcodeInfos[i].hasTeachPoint = jsonHelper::getBool(barcode, QStringLiteral("teach_point_flag"));
		_barcodeInfos[i].registration_method = jsonHelper::getInteger(barcode, QStringLiteral("registration_method"), rootFallbackMethod);

		fromJson(barcode["search_region"].toObject(), _barcodeInfos[i].search_region);
		fromJson(barcode["teach_point"].toObject(), _barcodeInfos[i].teach_point);

		ct::logger::debug("id: %s, x: %f, y: %f, z: %f",
			_barcodeInfos[i].id.toStdString().c_str(),
			_barcodeInfos[i].teach_point.wx,
			_barcodeInfos[i].teach_point.wy,
			_barcodeInfos[i].teach_point.wz);
	}

	_enableBarcode = jsonHelper::getBool(root, QStringLiteral("enable_barcode"), false);
	int idx = std::clamp(_currentBarcodeIndex, 0, (int)_barcodeInfos.size() - 1);

	{
		QSignalBlocker sb(ui.comboBox_barcodeRegistrationMethod);
		ui.comboBox_barcodeRegistrationMethod->setCurrentIndex(_barcodeInfos[idx].registration_method);
	}
	showBarcodeRegistrationMethodUI(_barcodeInfos[idx].registration_method);

	_jobThread.enableBarcode(_enableBarcode);
	ui.checkBox_enablebarcode->setChecked(_enableBarcode);

	return true;
}

void VisionApp::saveBarcodeResult()
{
	auto jsonPath = SystemData::instance()._workingPath + "/Barcode.json";

	QJsonObject j_root;
	j_root.insert(QStringLiteral("code"), SystemData::instance()._currentBarcode.c_str());

	auto ret = jsonHelper::saveJson(jsonPath, QJsonDocument(j_root));
	if (ret) ct::logger::info("Successfully saved barcode result!");
	else ct::logger::error("Failed to save barcode result!");
}

void VisionApp::updateBarcodeSettingsUI(int index)
{
	QSignalBlocker sb1(ui.label_barcodeID);
	QSignalBlocker sb2(ui.lineEdit_barcode_teachPoint);
	QSignalBlocker sb3(ui.comboBox_imageChannel);
	QSignalBlocker sb4(ui.comboBox_foregroundType);
	QSignalBlocker sb5(ui.comboBox_barcodeType);
	QSignalBlocker sb6(ui.comboBox_recognitionType);
	QSignalBlocker sb7(ui.comboBox_barcodeRegistrationMethod);

	ct::logger::error("Updatebarcode UI: %d", _barcodeInfos[index].barcode_type);
	auto& barcode = _barcodeInfos[index];
	ui.label_barcodeID->setText(QString(QString("Barcode-ID: ") + barcode.id));
	ui.lineEdit_barcode_teachPoint->setText(QString("X: %1    Y: %2    Z: %3")
		.arg(barcode.teach_point.wx)
		.arg(barcode.teach_point.wy)
		.arg(barcode.teach_point.wz));
	ui.comboBox_imageChannel->setCurrentIndex(_barcodeInfos[index].image_channel);
	ui.comboBox_foregroundType->setCurrentIndex(_barcodeInfos[index].foreground_type);
	ui.comboBox_barcodeType->setCurrentIndex(_barcodeInfos[index].barcode_type);
	ui.comboBox_recognitionType->setCurrentIndex(_barcodeInfos[index].recognition_type);
	ui.comboBox_barcodeRegistrationMethod->setCurrentIndex(_barcodeInfos[index].registration_method);
	showBarcodeRegistrationMethodUI(_barcodeInfos[index].registration_method);
}

void VisionApp::updateBarcodeSettingsJson(int index)
{
	qDebug() << "updateBarcodeSettingsJson()";
	if (index > 1) {
		printf("[UB] Invalid barcode index: %d\n", index);
		return;
	}

	_barcodeInfos[index].image_channel = ui.comboBox_imageChannel->currentIndex();
	_barcodeInfos[index].foreground_type = ui.comboBox_foregroundType->currentIndex();
	_barcodeInfos[index].barcode_type = ui.comboBox_barcodeType->currentIndex();
	_barcodeInfos[index].recognition_type = ui.comboBox_recognitionType->currentIndex();

	saveBarcode();
}

void VisionApp::displayBarcodeImage(int index)
{
	auto path_fid = Common::Directory::getRecipeImagesPath() + QString("Barcode/barcode%1.jpg").arg(index + 1);

	if (QFile::exists(path_fid)) {
		displayFOV(QImage(path_fid));
		ui.graphicsViewFOV->fitInView(_pPixmapItemFOV, Qt::KeepAspectRatio);
		ui.graphicsViewFOV->centerOn(_pPixmapItemFOV);
	}
}

void VisionApp::toggleBarcodeROISetupMode(bool state)
{
	if (state) {
		if (_barcodeSearchRegion.getGeometry().width() < 1) {
			auto w = CAMManager::instance().getWidth(_camID);
			auto h = CAMManager::instance().getHeight(_camID);
			_barcodeSearchRegion.setGeometry(QRectF(0, 0, w, h));
		}

		_barcodeSearchRegion.show();
		ui.toolButton_toggleFovView->animateClick();
		displayBarcodeImage(_currentBarcodeIndex);

		ui.toolButton_setupBarcodeROI->setChecked(true);
	}
	else {
		_barcodeSearchRegion.hide();
	
		ui.toolButton_setupBarcodeROI->setChecked(false);
	}
}

void VisionApp::updateBarcodeSearchRegion()
{
	if (_currentBarcodeIndex > 1)
	{
		qDebug() << "invalid barcode index";
		return;
	}

	_barcodeInfos[_currentBarcodeIndex].search_region.cx = _barcodeSearchRegion.getGeometry().center().x();
	_barcodeInfos[_currentBarcodeIndex].search_region.cy = _barcodeSearchRegion.getGeometry().center().y();
	_barcodeInfos[_currentBarcodeIndex].search_region.w = _barcodeSearchRegion.getGeometry().width();
	_barcodeInfos[_currentBarcodeIndex].search_region.h = _barcodeSearchRegion.getGeometry().height();
	_barcodeInfos[_currentBarcodeIndex].search_region.compute_extremum();
}


void VisionApp::showBarcodeRegistrationMethodUI(int methodIdx)
{
	const bool isCamera = (methodIdx == 1); // 0=Handheld, 1=Camera, 2=External

	// ---- 1) Force turn off ROI / overlays when not Camera ----
	if (!isCamera) {
		toggleBarcodeROISetupMode(false);   // will hide _barcodeSearchRegion + uncheck button
		_barcodeLocatedRegion.hide();
		_barcodeSearchRegion.hide();
	}

	// ---- 2) Hide/show camera-only widgets ----
	// Add EVERYTHING you want hidden for Handheld/External here.
	const QList<QWidget*> cameraOnly = {
		ui.lineEdit_barcode_teachPoint,
		ui.toolButton_setupBarcodeROI,
		ui.toolButton_ReadBarcode,
		ui.toolButton_readBarcodeOffline,
		ui.comboBox_imageChannel,
		ui.comboBox_foregroundType,
		ui.comboBox_barcodeType,
		ui.comboBox_recognitionType,
		ui.label_imagechannel,
		ui.label_foregroundType,
		ui.label_barcodeType,
		ui.label_RecognitionType,
		ui.label_DecodedBarcode,
		ui.label_barcode,
		ui.lineEdit_decodedBarcode

	};

	for (QWidget* w : cameraOnly) {
		if (w) w->setVisible(isCamera);
	}

}