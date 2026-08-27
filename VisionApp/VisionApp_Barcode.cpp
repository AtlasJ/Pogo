#include "VisionApp.h"
#include <QLabel>
#include <QPainter>
#include "CAMManager.h"
#include "SRXManager.h"
#include "MotionController.h"
#include "AuditLog.h"

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
	j_root.insert(QStringLiteral("first_reader"), (int)SystemData::instance()._brFirstReader);
	j_root.insert(QStringLiteral("r1_duration_ms"), (int)SystemData::instance()._brR1Duration_ms);
	j_root.insert(QStringLiteral("r2_duration_ms"), (int)SystemData::instance()._brR2Duration_ms);
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
	//recipe settings: reader scan order + per-reader read duration
	SystemData::instance()._brFirstReader = jsonHelper::getInteger(root, QStringLiteral("first_reader"), 1);
	SystemData::instance()._brR1Duration_ms = jsonHelper::getInteger(root, QStringLiteral("r1_duration_ms"), 2000);
	SystemData::instance()._brR2Duration_ms = jsonHelper::getInteger(root, QStringLiteral("r2_duration_ms"), 2000);
	{
		QSignalBlocker b1(ui.radioButton_brFirstR1);
		QSignalBlocker b2(ui.radioButton_brFirstR2);
		QSignalBlocker b3(ui.lineEdit_brR1Duration);
		QSignalBlocker b4(ui.lineEdit_brR2Duration);
		if (SystemData::instance()._brFirstReader == 2) ui.radioButton_brFirstR2->setChecked(true);
		else ui.radioButton_brFirstR1->setChecked(true);
		ui.lineEdit_brR1Duration->setText(QString::number(SystemData::instance()._brR1Duration_ms / 1000.0));
		ui.lineEdit_brR2Duration->setText(QString::number(SystemData::instance()._brR2Duration_ms / 1000.0));
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


//---------------------------------------------------------------------------
// External barcode reader setup page (Keyence SR-X100 via SRXManager)
//---------------------------------------------------------------------------

void VisionApp::initBarcodeReaderPage()
{
	auto& srx = SRXManager::instance();

	struct Row {
		QString id;
		QCheckBox* enable;
		QLineEdit* ip;
		QLineEdit* port;
		QToolButton* connectBtn;
		QToolButton* status;
		QToolButton* trigger;
		QToolButton* stop;
		QLineEdit* code;
		QLabel* readTime;
		QCheckBox* live;
	};

	const QVector<Row> rows = {
		{ SRXManager::SRX1, ui.checkBox_srx1Enable, ui.lineEdit_srx1Ip, ui.lineEdit_srx1Port,
		  ui.toolButton_srx1Connect, ui.toolButton_srx1Status, ui.toolButton_srx1Trigger,
		  ui.toolButton_srx1Stop, ui.lineEdit_srx1Code, ui.label_srx1ReadTime, ui.checkBox_srx1Live },
		{ SRXManager::SRX2, ui.checkBox_srx2Enable, ui.lineEdit_srx2Ip, ui.lineEdit_srx2Port,
		  ui.toolButton_srx2Connect, ui.toolButton_srx2Status, ui.toolButton_srx2Trigger,
		  ui.toolButton_srx2Stop, ui.lineEdit_srx2Code, ui.label_srx2ReadTime, ui.checkBox_srx2Live },
	};

	for (const auto& row : rows) {
		const QString id = row.id;

		nvs::set_background_color(row.status, srx.isConnected(id) ? Qt::green : Qt::red);

		connect(row.connectBtn, &QToolButton::clicked, this, [=]() {
			//apply what is currently typed, then reconnect
			SRXManager::ReaderConfig cfg = SRXManager::instance().readerConfig(id);
			cfg.enabled = row.enable->isChecked();
			cfg.ip = row.ip->text().trimmed();
			cfg.port = row.port->text().toInt();
			SRXManager::instance().setReaderConfig(cfg);
			AuditLog::instance().log(QStringLiteral("SRX_CONNECT"), id);
		});

		connect(row.trigger, &QToolButton::clicked, this, [=]() {
			row.code->clear();
			row.readTime->setText("-");
			SRXManager::instance().trigger(id);
			AuditLog::instance().log(QStringLiteral("SRX_TEST_READ"), id);
		});

		connect(row.stop, &QToolButton::clicked, this, [=]() {
			//stop also ends live read - reflect that in the checkbox without re-firing setLiveRead
			QSignalBlocker blocker(row.live);
			row.live->setChecked(false);
			SRXManager::instance().stopReader(id);
		});

		connect(row.live, &QCheckBox::toggled, this, [=](bool checked) {
			row.code->clear();
			row.readTime->setText("-");
			SRXManager::instance().setLiveRead(id, checked);
			AuditLog::instance().log(QStringLiteral("SRX_LIVE_READ"), id, checked ? QStringLiteral("ON") : QStringLiteral("OFF"));
		});
	}

	initBarcodeReaderAlignment();

	//recipe settings: scan order + read durations (persisted in barcode.json)
	connect(ui.radioButton_brFirstR1, &QRadioButton::toggled, this, [=](bool checked) {
		if (!checked) return;
		SystemData::instance()._brFirstReader = 1;
		saveBarcode();
	});
	connect(ui.radioButton_brFirstR2, &QRadioButton::toggled, this, [=](bool checked) {
		if (!checked) return;
		SystemData::instance()._brFirstReader = 2;
		saveBarcode();
	});
	connect(ui.lineEdit_brR1Duration, &QLineEdit::editingFinished, this, [=]() {
		auto s = ui.lineEdit_brR1Duration->text().toDouble();
		if (s <= 0) { s = 2.0; ui.lineEdit_brR1Duration->setText("2"); }
		SystemData::instance()._brR1Duration_ms = (int)(s * 1000);
		saveBarcode();
	});
	connect(ui.lineEdit_brR2Duration, &QLineEdit::editingFinished, this, [=]() {
		auto s = ui.lineEdit_brR2Duration->text().toDouble();
		if (s <= 0) { s = 2.0; ui.lineEdit_brR2Duration->setText("2"); }
		SystemData::instance()._brR2Duration_ms = (int)(s * 1000);
		saveBarcode();
	});

	connect(ui.toolButton_srxFtpRestart, &QToolButton::clicked, this, [=]() {
		SRXManager::FtpConfig cfg = SRXManager::instance().ftpConfig();
		cfg.port = ui.lineEdit_srxFtpPort->text().toInt();
		cfg.dropPath = ui.lineEdit_srxFtpDrop->text().trimmed();
		SRXManager::instance().setFtpConfig(cfg);
	});

	connect(ui.toolButton_srxSave, &QToolButton::clicked, this, [=]() {
		for (const auto& row : rows) {
			SRXManager::ReaderConfig cfg = SRXManager::instance().readerConfig(row.id);
			cfg.enabled = row.enable->isChecked();
			cfg.ip = row.ip->text().trimmed();
			cfg.port = row.port->text().toInt();
			SRXManager::instance().setReaderConfig(cfg);
		}

		SRXManager::FtpConfig ftp = SRXManager::instance().ftpConfig();
		ftp.port = ui.lineEdit_srxFtpPort->text().toInt();
		ftp.dropPath = ui.lineEdit_srxFtpDrop->text().trimmed();
		SRXManager::instance().setFtpConfig(ftp);

		if (SRXManager::instance().saveConfig()) showStatus(QStringLiteral("Barcode reader settings saved!"));
		else showMsg(QStringLiteral("Failed to save barcode reader settings!"));

		AuditLog::instance().log(QStringLiteral("SRX_CONFIG_SAVE"));
	});

	//live updates from the manager (signals come from its worker thread)
	connect(&srx, &SRXManager::connectionChanged, this, [=](const QString& id, bool connected) {
		for (const auto& row : rows) {
			if (row.id == id) nvs::set_background_color(row.status, connected ? Qt::green : Qt::red);
		}
	});

	connect(&srx, &SRXManager::barcodeReceived, this, [=](const QString& id, const QString& code) {
		for (const auto& row : rows) {
			if (row.id == id) row.code->setText(code);
		}
	});

	connect(&srx, &SRXManager::readResultReceived, this, [=](const QString& id) {
		auto result = SRXManager::instance().lastResult(id);
		for (const auto& row : rows) {
			if (row.id != id) continue;
			if (result.readTimeMs >= 0) row.readTime->setText(QString("%1 ms").arg(result.readTimeMs));
			if (!result.code.isEmpty()) row.code->setText(result.code);
		}
		updateSRXImagePreview(id);
	});

	connect(&srx, &SRXManager::imageReceived, this, [=](const QString& id, const QString&) {
		updateSRXImagePreview(id);

		//production: show the reader capture on the main FOV so the operator
		//sees what each unit snap looks like
		if (_processType == ProcessType::PRODUCTION) {
			QImage img = SRXManager::instance().lastImage(id);
			if (!img.isNull()) displayFOV(img);
		}
	});

	connect(&srx, &SRXManager::ftpStateChanged, this, [=](bool running) {
		nvs::set_background_color(ui.toolButton_srxFtpStatus, running ? Qt::green : Qt::red);
	});

	refreshBarcodeReaderPage();
}

void VisionApp::refreshBarcodeReaderPage()
{
	auto& srx = SRXManager::instance();

	auto cfg1 = srx.readerConfig(SRXManager::SRX1);
	ui.checkBox_srx1Enable->setChecked(cfg1.enabled);
	ui.lineEdit_srx1Ip->setText(cfg1.ip);
	ui.lineEdit_srx1Port->setText(QString::number(cfg1.port));

	auto cfg2 = srx.readerConfig(SRXManager::SRX2);
	ui.checkBox_srx2Enable->setChecked(cfg2.enabled);
	ui.lineEdit_srx2Ip->setText(cfg2.ip);
	ui.lineEdit_srx2Port->setText(QString::number(cfg2.port));

	auto ftp = srx.ftpConfig();
	ui.lineEdit_srxFtpPort->setText(QString::number(ftp.port));
	ui.lineEdit_srxFtpDrop->setText(ftp.dropPath);

	nvs::set_background_color(ui.toolButton_srx1Status, srx.isConnected(SRXManager::SRX1) ? Qt::green : Qt::red);
	nvs::set_background_color(ui.toolButton_srx2Status, srx.isConnected(SRXManager::SRX2) ? Qt::green : Qt::red);
	nvs::set_background_color(ui.toolButton_srxFtpStatus, srx.isFtpRunning() ? Qt::green : Qt::red);
}

void VisionApp::updateSRXImagePreview(const QString& readerID)
{
	QLabel* target = nullptr;
	if (readerID == SRXManager::SRX1) target = ui.label_srx1Image;
	else if (readerID == SRXManager::SRX2) target = ui.label_srx2Image;
	if (!target) return;

	//the pixmap must not drive the layout: with a default size policy the label
	//grows to the scaled pixmap, which enlarges the label, which enlarges the
	//next scale - stretching the page a little on every image
	target->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);

	auto result = SRXManager::instance().lastResult(readerID);
	QImage image = SRXManager::instance().lastImage(readerID);
	if (image.isNull()) return;

	//overlay the decoded code corners from the historical data
	if (!result.corners.isEmpty()) {
		QPainter painter(&image);
		painter.setRenderHint(QPainter::Antialiasing);
		painter.setPen(QPen(result.ok ? Qt::green : Qt::red, qMax(2, image.width() / 400)));
		for (const auto& quad : result.corners) painter.drawPolygon(quad);
	}

	//center crosshair, follows the main FOV crosshair toggle
	if (ui.toolButton_showCrossHair->isChecked()) {
		QPainter painter(&image);
		painter.setPen(QPen(QColor(0, 255, 127), qMax(1, image.width() / 800)));
		painter.drawLine(0, image.height() / 2, image.width(), image.height() / 2);
		painter.drawLine(image.width() / 2, 0, image.width() / 2, image.height());
	}

	target->setPixmap(QPixmap::fromImage(image).scaled(
		target->width(), target->height(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
}


// ── reader alignment: camera-to-reader offsets (XYZ) ─────────────────────────

namespace {
	struct BrPoint {
		bool set = false;
		double x = 0, y = 0, z = 0;
	};
	struct BrAlign {
		BrPoint cam, r1, r2;
		int lastOffsetReader = 0; //last reader offset applied, for "Offset to Camera"
	};
	BrAlign g_brAlign;
}

void VisionApp::initBarcodeReaderAlignment()
{
	const QString alignPath = QStringLiteral("%1/barcodeAlign.json").arg(Common::Directory::ConfigPath());

	//offsets are derived from the taught absolute centers: reader - camera
	auto offsetOf = [](const BrPoint& reader) {
		return BrPoint{ reader.set && g_brAlign.cam.set,
			reader.x - g_brAlign.cam.x, reader.y - g_brAlign.cam.y, reader.z - g_brAlign.cam.z };
	};

	//keep the job-thread copy in SystemData up to date
	auto syncToSystemData = [=]() {
		auto o1 = offsetOf(g_brAlign.r1);
		auto o2 = offsetOf(g_brAlign.r2);
		SystemData::instance()._brR1Taught = o1.set;
		SystemData::instance()._brR1dx = o1.x;
		SystemData::instance()._brR1dy = o1.y;
		SystemData::instance()._brR1dz = o1.z;
		SystemData::instance()._brR2Taught = o2.set;
		SystemData::instance()._brR2dx = o2.x;
		SystemData::instance()._brR2dy = o2.y;
		SystemData::instance()._brR2dz = o2.z;
	};

	auto refreshLabels = [=]() {
		ui.label_brCamCenter->setText(g_brAlign.cam.set
			? QString("Camera center: %1, %2, %3").arg(g_brAlign.cam.x, 0, 'f', 3).arg(g_brAlign.cam.y, 0, 'f', 3).arg(g_brAlign.cam.z, 0, 'f', 3)
			: QStringLiteral("Camera center: not set"));

		auto offsetText = [&](const BrPoint& reader) {
			auto o = offsetOf(reader);
			return o.set
				? QString("Offset: %1, %2, %3").arg(o.x, 0, 'f', 3).arg(o.y, 0, 'f', 3).arg(o.z, 0, 'f', 3)
				: QStringLiteral("Offset: 0.000, 0.000, 0.000 (NOT TAUGHT)");
		};
		ui.label_brR1Offset->setText(offsetText(g_brAlign.r1));
		ui.label_brR2Offset->setText(offsetText(g_brAlign.r2));
		ui.label_brR1Offset->setStyleSheet(offsetOf(g_brAlign.r1).set ? "" : "color: orange;");
		ui.label_brR2Offset->setStyleSheet(offsetOf(g_brAlign.r2).set ? "" : "color: orange;");
	};

	auto pointToJson = [](const BrPoint& p) {
		QJsonObject o;
		o.insert("set", p.set);
		o.insert("x", p.x);
		o.insert("y", p.y);
		o.insert("z", p.z);
		return o;
	};
	auto pointFromJson = [](const QJsonObject& o) {
		BrPoint p;
		p.set = jsonHelper::getBool(o, "set", false);
		p.x = jsonHelper::getDouble(o, "x", 0.0);
		p.y = jsonHelper::getDouble(o, "y", 0.0);
		p.z = jsonHelper::getDouble(o, "z", 0.0);
		return p;
	};

	auto save = [=]() {
		QJsonObject obj;
		obj.insert("camera", pointToJson(g_brAlign.cam));
		obj.insert("reader1", pointToJson(g_brAlign.r1));
		obj.insert("reader2", pointToJson(g_brAlign.r2));
		saveJson(alignPath, QJsonDocument(obj));
		syncToSystemData();
	};

	auto currentXYZ = [=](double& x, double& y, double& z) -> bool {
		auto ox = MotionController::instance().get_position_mm(_motionID, 0, (int)Axis::X);
		auto oy = MotionController::instance().get_position_mm(_motionID, 0, (int)Axis::Y);
		auto oz = MotionController::instance().get_position_mm(_motionID, 0, (int)Axis::Z);
		if (!ox.has_value() || !oy.has_value() || !oz.has_value()) {
			showMsg("Failed to read the current position. Is motion connected?");
			return false;
		}
		x = ox.value();
		y = oy.value();
		z = oz.value();
		return true;
	};

	//load taught centers
	{
		QJsonObject root;
		if (loadJson(alignPath, root)) {
			g_brAlign.cam = pointFromJson(root.value("camera").toObject());
			g_brAlign.r1 = pointFromJson(root.value("reader1").toObject());
			g_brAlign.r2 = pointFromJson(root.value("reader2").toObject());
		}
	}
	syncToSystemData();
	refreshLabels();

	//── teach ──
	auto setCenter = [=](BrPoint& p, const QString& what) {
		double x = 0, y = 0, z = 0;
		if (!currentXYZ(x, y, z)) return;
		p.set = true;
		p.x = x;
		p.y = y;
		p.z = z;
		AuditLog::instance().log(QStringLiteral("BR_ALIGN_SET"), what);
		save();
		refreshLabels();
	};

	connect(ui.toolButton_brSetCamCenter, &QToolButton::clicked, this, [=]() { setCenter(g_brAlign.cam, "CAMERA"); });
	connect(ui.toolButton_brSetR1Center, &QToolButton::clicked, this, [=]() {
		if (!g_brAlign.cam.set) { showMsg("Set the camera center first."); return; }
		setCenter(g_brAlign.r1, "READER1");
	});
	connect(ui.toolButton_brSetR2Center, &QToolButton::clicked, this, [=]() {
		if (!g_brAlign.cam.set) { showMsg("Set the camera center first."); return; }
		setCenter(g_brAlign.r2, "READER2");
	});

	//── jog to a taught center (absolute) ──
	auto jogToCenter = [=](const BrPoint& p, const QString& what) {
		if (!p.set) {
			showMsg(QString("%1 center has not been taught.").arg(what));
			return;
		}
		AuditLog::instance().log(QStringLiteral("BR_ALIGN_JOG"), what);
		emit jogTo(p.x, p.y, p.z);
	};

	connect(ui.toolButton_brJogCam, &QToolButton::clicked, this, [=]() { jogToCenter(g_brAlign.cam, "Camera"); });
	connect(ui.toolButton_brJogR1, &QToolButton::clicked, this, [=]() { jogToCenter(g_brAlign.r1, "Reader 1"); });
	connect(ui.toolButton_brJogR2, &QToolButton::clicked, this, [=]() { jogToCenter(g_brAlign.r2, "Reader 2"); });

	//── move by the taught offset: the point under the camera goes under the reader ──
	auto offsetToReader = [=](int reader) {
		auto o = offsetOf(reader == 1 ? g_brAlign.r1 : g_brAlign.r2);
		if (!o.set) {
			showMsg(QString("Reader %1 offset has not been taught.").arg(reader));
			return;
		}

		double x = 0, y = 0, z = 0;
		if (!currentXYZ(x, y, z)) return;

		g_brAlign.lastOffsetReader = reader;
		AuditLog::instance().log(QStringLiteral("BR_ALIGN_GOTO_READER"), QString::number(reader));
		emit jogTo(x + o.x, y + o.y, z + o.z);
	};

	connect(ui.toolButton_brGoR1, &QToolButton::clicked, this, [=]() { offsetToReader(1); });
	connect(ui.toolButton_brGoR2, &QToolButton::clicked, this, [=]() { offsetToReader(2); });

	//reverse of the last applied reader offset: back from the reader to the camera
	connect(ui.toolButton_brGoCam, &QToolButton::clicked, this, [=]() {
		if (g_brAlign.lastOffsetReader == 0) {
			showMsg("Offset to a reader first - Offset to Camera reverses that move.");
			return;
		}

		auto o = offsetOf(g_brAlign.lastOffsetReader == 1 ? g_brAlign.r1 : g_brAlign.r2);
		if (!o.set) {
			showMsg("The reader offset is no longer taught.");
			return;
		}

		double x = 0, y = 0, z = 0;
		if (!currentXYZ(x, y, z)) return;

		AuditLog::instance().log(QStringLiteral("BR_ALIGN_GOTO_CAMERA"), QString::number(g_brAlign.lastOffsetReader));
		g_brAlign.lastOffsetReader = 0;
		emit jogTo(x - o.x, y - o.y, z - o.z);
	});
}
