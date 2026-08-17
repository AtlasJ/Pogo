#include "VisionApp.h"
#include <QLabel>
#include "CAMManager.h"
#include "ScaleManager.h"
#include "AuditLog.h"

static QString fiducialAuditValue(const QJsonValue& value)
{
	if (value.isUndefined()) return QStringLiteral("-");
	if (value.isBool()) return value.toBool() ? QStringLiteral("true") : QStringLiteral("false");
	if (value.isNull()) return QStringLiteral("null");
	return value.toVariant().toString();
}

static void collectFiducialChanges(const QString& path,
	const QJsonValue& before,
	const QJsonValue& after,
	QStringList& changes)
{
	if (before.isObject() && after.isObject())
	{
		const QJsonObject beforeObj = before.toObject();
		const QJsonObject afterObj = after.toObject();
		QStringList keys = beforeObj.keys();
		for (const QString& key : afterObj.keys())
			if (!keys.contains(key)) keys.append(key);

		for (const QString& key : keys)
		{
			// These region extrema are derived from cx/cy/w/h and would duplicate the useful changes.
			if ((path == QStringLiteral("search_region") || path == QStringLiteral("inspect_region"))
				&& (key == QStringLiteral("xmin") || key == QStringLiteral("ymin")
					|| key == QStringLiteral("xmax") || key == QStringLiteral("ymax")))
				continue;

			const QString childPath = path.isEmpty() ? key : path + QStringLiteral("/") + key;
			collectFiducialChanges(childPath, beforeObj.value(key), afterObj.value(key), changes);
		}
		return;
	}

	if (before != after)
	{
		changes.append(QStringLiteral("%1: %2 -> %3")
			.arg(path, fiducialAuditValue(before), fiducialAuditValue(after)));
	}
}

bool VisionApp::fiducialExistTest(int index)
{
	if (index >= _fiducialInfos.size()) return false;
	auto fiducialJsonPath = QStringLiteral("%1recipe/%2/fiducial.json").arg(Common::Directory::LocalPath).arg(Common::Directory::CurrentRecipe);
	if (!QFileInfo(fiducialJsonPath).exists()) return false;

	qDebug() << "_fiducialInfos[index].hasTeachPoint:" << _fiducialInfos[index].hasTeachPoint;
	return _fiducialInfos[index].hasTeachPoint;
}

void VisionApp::toggleFiducialUI(bool enable)
{
	if (enable)
	{
		ui.frame_fiducial->show();
		ui.frame_viewSetup->show();
		ui.frame_barcode->hide();
	}
	else
	{
		ui.frame_fiducial->hide();
		ui.frame_viewSetup->hide();
		ui.frame_barcode->hide();
	}
}

void VisionApp::initFiducial()
{
	setDefaultFiducialInfos();

	_pGraphicsSceneFOV->addItem(&_fidSearchRegion);
	_pGraphicsSceneFOV->addItem(&_fidInspectionRegion);
	_pGraphicsSceneFOV->addItem(&_fidLocatedRegion);

	auto setRegion = [=](const QRectF& rect, const QColor& color, const QString& name, QDragBox& db) {
		db.setup(rect, color, name);
		db.setOutterBarrier(_pGraphicsSceneFOV->sceneRect());
		db.setDragable(true);
		db.setZValue((int)UIHierarchy::DRAGGABLES);
		};

	QIntValidator* validator_minDiameter = new QIntValidator(0, 100000, ui.lineEdit_fiducial_minDiameter);
	ui.lineEdit_fiducial_minDiameter->setValidator(validator_minDiameter);
	ui.lineEdit_fiducial_minDiameter->setText("280");

	QIntValidator* validator_maxDiameter = new QIntValidator(0, 100000, ui.lineEdit_fiducial_maxDiameter);
	ui.lineEdit_fiducial_maxDiameter->setValidator(validator_maxDiameter);
	ui.lineEdit_fiducial_maxDiameter->setText("320");

	QIntValidator* validator_score = new QIntValidator(0, 100, ui.lineEdit_fiducial_score);
	ui.lineEdit_fiducial_score->setValidator(validator_score);
	ui.lineEdit_fiducial_score->setText("50");

	QStringList fiducialMethods;
	fiducialMethods << "Pattern Matching" << "Geometry Model Finder" << "Circle Finder" << "Cross Finder";
	ui.comboBox_fiducialMethod->addItems(fiducialMethods);

	auto sr = QRectF(0, 0, _imageSize.width(), _imageSize.height());
	setRegion(sr, QColor(0, 255, 127), "Search Region", _fidSearchRegion);
	connect(&_fidSearchRegion, SIGNAL(dragBoxMouseReleased(QDragBox*, QString, QPointF)), this, SLOT(updateFiducialRegions()));
	connect(&_fidSearchRegion, SIGNAL(grabberReleased(QDragBox*)), this, SLOT(updateFiducialRegions()));

	auto ir = QRectF(_imageSize.width() * 0.3, _imageSize.height() * 0.3, _imageSize.width() * 0.7, _imageSize.height() * 0.7);
	setRegion(ir, QColor(0, 127, 127), "Inspection Region", _fidInspectionRegion);
	connect(&_fidInspectionRegion, SIGNAL(dragBoxMouseReleased(QDragBox*, QString, QPointF)), this, SLOT(updateFiducialRegions()));
	connect(&_fidInspectionRegion, SIGNAL(grabberReleased(QDragBox*)), this, SLOT(updateFiducialRegions()));

	updateFiducialRegions();

	auto lr = QRectF(0, 0, 10, 10);
	setRegion(lr, QColor(247, 227, 41), "Located Region", _fidLocatedRegion);
	_fidLocatedRegion.setDragable(false);
	_fidLocatedRegion.setZValue((int)UIHierarchy::VIEW);
	_fidLocatedRegion.hide();

	//_fidSearchRegion
	connect(ui.toolButton_fid1, &QToolButton::clicked, this, [=]() { showFiducial(_currentFidIndex); });
	connect(ui.toolButton_addFid, &QToolButton::clicked, this, [=]() {

		int index = 1;

		for (const auto& f : _fiducialInfos) {
			if (f.id.size() != 4) {
				ct::logger::error("Invalid Fiducial ID.");
				continue;
			}

			auto num = f.id.at(3).digitValue();

			if (num == index) {
				index++;
			}
			else {
				break;
			}
		}

		if (index > 10) {
			showMsg("Exceeded maximum allowable fiducials");
			return;
		}

		FiducialInfo newFid = _fiducialInfos[0];
		newFid.id = QString("fid%1").arg(index);
		ui.comboBox_fids->addItem(newFid.id);
		AuditLog::instance().log(QStringLiteral("FIDUCIAL_ADD"), newFid.id);

		_fiducialInfos.push_back(newFid);
		_currentFidIndex = _fiducialInfos.size() - 1;

		ui.comboBox_fids->setCurrentIndex(_currentFidIndex);
		});
	connect(ui.toolButton_deleteFid, &QToolButton::clicked, this, [=]() {
		if (_currentFidIndex < 2) {
			showMsg("First two fiducial is not removable.");
			return;
		}

		AuditLog::instance().log(QStringLiteral("FIDUCIAL_DELETE"), QStringLiteral("index=%1").arg(_currentFidIndex));
		_fiducialInfos.erase(_fiducialInfos.begin() + _currentFidIndex);
		ui.comboBox_fids->removeItem(_currentFidIndex);

		if (_currentFidIndex >= _fiducialInfos.size()) {
			_currentFidIndex = _fiducialInfos.size() - 1;
		}

		showFiducial(_currentFidIndex);
		});
	QObject::connect(ui.comboBox_fids, QOverload<int>::of(&QComboBox::currentIndexChanged), [&](int index) {
		_currentFidIndex = index;
		showFiducial(index);
		});

	connect(ui.toolButton_learnFiducial, &QToolButton::clicked, this, [=]() {learnFiducial(); });
	connect(ui.toolButton_testFiducial, &QToolButton::clicked, this, [=]() { emit testFiducial(_currentFidIndex, true); });
	connect(ui.toolButton_testFiducialOffline, &QToolButton::clicked, this, [=]() { emit testFiducial(_currentFidIndex, false); });
	connect(ui.toolButton_setFiducialPoint, &QToolButton::clicked, this, [=]() { emit autoSetFiducialPoint(_currentFidIndex); });
	connect(ui.toolButton_fiducialAlignment, &QToolButton::clicked, this, [=]() { fiducialAlignment(); });
	connect(ui.toolButton_setupFidROI, &QToolButton::toggled, this, [=](bool state) { if (state) { if (!passwordPromptCorrect()) return;} toggleFidROISetupMode(state); saveFiducial(); });
	connect(ui.comboBox_fiducialMethod, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [=](int index) { updateFiducialSettingsJson(_currentFidIndex); showFiducialMethodUI(index); });
	connect(ui.lineEdit_fiducial_score, &QLineEdit::editingFinished, this, [=]() { updateFiducialSettingsJson(_currentFidIndex); });
	connect(ui.lineEdit_fiducial_minDiameter, &QLineEdit::editingFinished, this, [=]() { updateFiducialSettingsJson(_currentFidIndex); });
	connect(ui.lineEdit_fiducial_maxDiameter, &QLineEdit::editingFinished, this, [=]() { updateFiducialSettingsJson(_currentFidIndex); });

	connect(ui.toolButton_runFiducialPrecisionTest, &QToolButton::clicked, this, [=]() {
		auto num = ui.lineEdit_numOfRuns->text().toInt();

		/*clearAcquisitionQueue();

		for (int i = 0; i < num; i++) {
			appendAcquisitionQueue(AcquisitionType::FID1);
			appendAcquisitionQueue(AcquisitionType::FID2);
		}

		startAcquisition();*/
		});
}

void VisionApp::setDefaultFiducialInfos()
{
	_fiducialInfos.resize(2);
	_fiducialInfos[0] = FiducialInfo();
	_fiducialInfos[1] = FiducialInfo();
	_fiducialInfos[0].id = "fid1";
	_fiducialInfos[1].id = "fid2";

	ui.comboBox_fids->clear();
	ui.comboBox_fids->addItem(_fiducialInfos[0].id);
	ui.comboBox_fids->addItem(_fiducialInfos[1].id);
}

void VisionApp::showFiducial(int index)
{
	ui.frame_fiducial->show();
	ui.frame_barcode->hide();

	if (index >= _fiducialInfos.size()) {
		ct::logger::warn("[UB] Invalid fiducial index: %d", index);
		return;
	}

	_currentTeachPointType = TeachPointType::FIDUCIAL;
	_currentFidIndex = index;

	ui.toolButton_fid1->setChecked(true);
	ui.toolButton_barcode->setChecked(false);
	ui.toolButton_Barcode2->setChecked(false);


	auto& fid = _fiducialInfos[index];
	ui.lineEdit_teachPoint->setText(QString("X: %1	Y: %2	Z: %3")
		.arg(fid.teach_point.wx)
		.arg(fid.teach_point.wy)
		.arg(fid.teach_point.wz));

	QString fileIndex = QString::number(index + 1);

	auto path_fid = Common::Directory::CurrentImageSetPath + QString("fid%1.jpg").arg(fileIndex);
	if (!QFile::exists(path_fid)) path_fid = Common::Directory::getRecipeImagesPath() + QString("Fiducial/fid%1.jpg").arg(fileIndex);

	if (QFile::exists(path_fid)) {
		auto desired_width = ui.label_teachPoint->width();
		QPixmap pixmap = QPixmap(path_fid).scaledToWidth(300, Qt::SmoothTransformation);
		ui.label_teachPoint->setPixmap(pixmap);
	}
	else {
		ui.label_teachPoint->setPixmap(QPixmap());
	}

	toggleFidROISetupMode(false);
	toggleBarcodeROISetupMode(false);

	auto& ir = fid.inspect_region;
	ir.compute_extremum();

	_fidInspectionRegion.setGeometry(QRectF(ir.xmin, ir.ymin, ir.w, ir.h));
	auto& sr = fid.search_region;
	sr.compute_extremum();

	_fidSearchRegion.setGeometry(QRectF(sr.xmin, sr.ymin, sr.w, sr.h));

	showFiducialFeature(index);

	ui.comboBox_fiducialMethod->blockSignals(true);
	ui.lineEdit_fiducial_score->blockSignals(true);
	ui.lineEdit_fiducial_minDiameter->blockSignals(true);
	ui.lineEdit_fiducial_maxDiameter->blockSignals(true);

	ui.comboBox_fiducialMethod->setCurrentIndex(fid.fiducial_method);
	ui.lineEdit_fiducial_score->setText(QString::number(fid.score));
	ui.lineEdit_fiducial_minDiameter->setText(QString::number(fid.min_diameter));
	ui.lineEdit_fiducial_maxDiameter->setText(QString::number(fid.max_diameter));

	ui.comboBox_fiducialMethod->blockSignals(false);
	ui.lineEdit_fiducial_score->blockSignals(false);
	ui.lineEdit_fiducial_minDiameter->blockSignals(false);
	ui.lineEdit_fiducial_maxDiameter->blockSignals(false);

	showFiducialMethodUI(index);
}

void VisionApp::showFiducialFeature(int index)
{
	if (index >= _fiducialInfos.size()) {
		ct::logger::warn("[UB] Invalid fiducial index: %d", index);
		return;
	}

	auto path_feature = Common::Directory::getRecipeImagesPath() + QString("Fiducial/feature%1.jpg").arg(index + 1);
	if (QFile::exists(path_feature)) {
		QPixmap pixmap = QPixmap(path_feature).scaledToHeight(300, Qt::SmoothTransformation);
		ui.label_fiducial->setPixmap(pixmap);
	}
	else {
		ui.label_fiducial->setPixmap(QPixmap());
	}
}

void VisionApp::showFiducialMethodUI(int index)
{
	if (ui.comboBox_fiducialMethod->currentIndex() == 2)
	{
		ui.label_121->hide();
		ui.lineEdit_fiducial_score->hide();

		ui.label_122->show();
		ui.lineEdit_fiducial_minDiameter->show();
		ui.label_123->show();
		ui.lineEdit_fiducial_maxDiameter->show();
	}
	else
	{
		ui.label_121->show();
		ui.lineEdit_fiducial_score->show();

		ui.label_122->hide();
		ui.lineEdit_fiducial_minDiameter->hide();
		ui.label_123->hide();
		ui.lineEdit_fiducial_maxDiameter->hide();

	}
}

void VisionApp::learnFiducial()
{
	if (passwordPromptCorrect())
	{
		backupFiducial();

		if (_currentFidIndex >= _fiducialInfos.size()) {
			ct::logger::warn("[UB] Invalid fiducial index: %d", _currentFidIndex);
			return;
		}

		QString fileIndex = QString::number(_currentFidIndex + 1);

		QImage qimg = _pixmapFOV.toImage();
		QImage croppedImage = qimg.copy(
			_fidInspectionRegion.getGeometry().x()
			, _fidInspectionRegion.getGeometry().y()
			, _fidInspectionRegion.getGeometry().width()
			, _fidInspectionRegion.getGeometry().height());

		auto root = Common::Directory::getRecipeImagesPath() + QString("Fiducial/");
		auto path_feature = root + QString("feature%1.jpg").arg(fileIndex);
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

		auto& fid = _fiducialInfos[_currentFidIndex];

		//learn Pattern Matching Model
		input.filename = root.toStdString() + "feature" + fileIndex.toStdString() + ".pat";
		mtrx::learn_pattern(mMono, input, output);

		//learn Geometry Model
		input.filename = root.toStdString() + "feature" + fileIndex.toStdString() + ".mod";
		mtrx::learn_geometryModel(mMono, input, output);

		fid.search_region.cx = _fidSearchRegion.getGeometry().x() + _fidSearchRegion.getGeometry().width() / 2;
		fid.search_region.cy = _fidSearchRegion.getGeometry().y() + _fidSearchRegion.getGeometry().height() / 2;
		fid.search_region.w = _fidSearchRegion.getGeometry().width();
		fid.search_region.h = _fidSearchRegion.getGeometry().height();

		fid.inspect_region.cx = _fidInspectionRegion.getGeometry().x() + _fidInspectionRegion.getGeometry().width() / 2;
		fid.inspect_region.cy = _fidInspectionRegion.getGeometry().y() + _fidInspectionRegion.getGeometry().height() / 2;
		fid.inspect_region.w = _fidInspectionRegion.getGeometry().width();
		fid.inspect_region.h = _fidInspectionRegion.getGeometry().height();

		mtrx::free_buffer(mBuf);
		mtrx::free_buffer(mMono);

		saveFiducial();
		showFiducialFeature(_currentFidIndex);

		toggleFidROISetupMode(false);
		updateSetupCheckList();
	}
	
}

void VisionApp::clearFiducialFeature(int index)
{
	if (index >= _fiducialInfos.size()) {
		ct::logger::warn("[UB] Invalid fiducial index: %d\n", index);
		return;
	}

	QString fileIndex = QString::number(index + 1);
	auto path_feature = Common::Directory::getRecipeImagesPath() + QString("Fiducial/feature%1.jpg").arg(fileIndex);
	auto path_pat = Common::Directory::getRecipeImagesPath() + QString("Fiducial/feature%1.pat").arg(fileIndex);
	QFile::remove(path_feature);
	QFile::remove(path_pat);
}

bool VisionApp::saveFiducial()
{
	auto jsonPath = QStringLiteral("%1recipe/%2/fiducial.json").arg(Common::Directory::LocalPath).arg(Common::Directory::CurrentRecipe);
	QJsonObject previousRoot;
	bool hadPrevious = false;
	QFile previousFile(jsonPath);
	if (previousFile.open(QIODevice::ReadOnly | QIODevice::Text))
	{
		const QJsonDocument previousDocument = QJsonDocument::fromJson(previousFile.readAll());
		previousFile.close();
		if (previousDocument.isObject())
		{
			previousRoot = previousDocument.object();
			hadPrevious = true;
		}
	}

	//save view
	QJsonObject j_root;
	QJsonArray j_array;

	bool emptyFiducialFlag = false;
	for (auto fid : _fiducialInfos) {
		QJsonObject obj;
		QJsonObject srObj, irObj;
		fid.search_region.compute_extremum();
		fid.inspect_region.compute_extremum();
		toJson(fid.search_region, srObj);
		toJson(fid.inspect_region, irObj);

		QJsonObject tObj;
		toJson(fid.teach_point, tObj);

		obj.insert(QStringLiteral("id"), fid.id);
		obj.insert(QStringLiteral("teach_point"), tObj);
		obj.insert(QStringLiteral("teach_point_flag"), fid.hasTeachPoint);
		obj.insert(QStringLiteral("search_region"), srObj);
		obj.insert(QStringLiteral("inspect_region"), irObj);
		obj.insert(QStringLiteral("fiducial_method"), fid.fiducial_method);
		obj.insert(QStringLiteral("score"), fid.score);
		obj.insert(QStringLiteral("min_diameter"), fid.min_diameter);
		obj.insert(QStringLiteral("max_diameter"), fid.max_diameter);

		if ((fid.inspect_region.cx == 0 && fid.inspect_region.cy == 0 && fid.inspect_region.h == 0 && fid.inspect_region.w == 0)
			|| (fid.search_region.cx == 0 && fid.search_region.cy == 0 && fid.search_region.w == 0 && fid.search_region.h == 0)
			|| (fid.teach_point.wx == 0 && fid.teach_point.wy == 0 && fid.teach_point.wz == 0) || fid.hasTeachPoint == false)
		{
			emptyFiducialFlag = true;
		}

		j_array.append(obj);
	}

	j_root.insert(QStringLiteral("fiducials"), j_array);
	QJsonObject j_board_dims_obj;
	j_board_dims_obj.insert(QStringLiteral("width_mm"), fidWidth);   
	j_board_dims_obj.insert(QStringLiteral("height_mm"), fidHeight);  
	j_root.insert(QStringLiteral("fiducial_board_dimensions"), j_board_dims_obj);


	emptyFiducialFlag = false;
	int ret = 0;
	if (emptyFiducialFlag)
	{	
		QMessageBox::warning(this, tr("Missing Fiducial Info"),
			tr("One or more Fiducial does not have any info!!! Failed to save Fiducial Info."));
		ct::logger::error("Missing Fiducial Info, fail to save, fiducial path%s", jsonPath.toStdString().c_str());
	}
	else ret = saveJson(jsonPath, QJsonDocument(j_root));

	if (ret)
	{
		if (hadPrevious)
		{
			QHash<QString, QJsonObject> previousFiducials;
			for (const QJsonValue& value : previousRoot.value(QStringLiteral("fiducials")).toArray())
			{
				const QJsonObject fiducial = value.toObject();
				previousFiducials.insert(fiducial.value(QStringLiteral("id")).toString(), fiducial);
			}

			for (const QJsonValue& value : j_root.value(QStringLiteral("fiducials")).toArray())
			{
				const QJsonObject fiducial = value.toObject();
				const QString id = fiducial.value(QStringLiteral("id")).toString();
				if (!previousFiducials.contains(id)) continue;

				QStringList changes;
				collectFiducialChanges(QString(), previousFiducials.value(id), fiducial, changes);
				for (const QString& change : changes)
				{
					AuditLog::instance().log(QStringLiteral("FIDUCIAL_CONFIG_CHANGE"),
						QStringLiteral("fid=%1; %2").arg(id, change));
				}
			}

			QStringList boardChanges;
			collectFiducialChanges(QString(),
				previousRoot.value(QStringLiteral("fiducial_board_dimensions")),
				j_root.value(QStringLiteral("fiducial_board_dimensions")),
				boardChanges);
			for (const QString& change : boardChanges)
			{
				AuditLog::instance().log(QStringLiteral("FIDUCIAL_CONFIG_CHANGE"),
					QStringLiteral("fid=board; %1").arg(change));
			}
		}

		showStatus(QStringLiteral("Successfully saved fiducial!"));
	}
	else showStatus(QStringLiteral("Failed to save fiducial!"));

	return ret;
}

bool VisionApp::loadFiducial()
{
	ct::logger::debug("Load Fiducial");

	auto jsonPath = QStringLiteral("%1recipe/%2/fiducial.json").arg(Common::Directory::LocalPath).arg(Common::Directory::CurrentRecipe);

	QJsonObject root;

	auto cam_w = CAMManager::instance().getWidth(_camID);
	auto cam_h = CAMManager::instance().getHeight(_camID);

	//guard
	if (!loadJson(jsonPath, root)) {
		for (auto& fid : _fiducialInfos) {
			fid.inspect_region.cx = cam_w / 2;
			fid.inspect_region.cy = cam_h / 2;
			fid.inspect_region.w = cam_w * 0.7;
			fid.inspect_region.h = cam_h * 0.7;
			fid.inspect_region.compute_extremum();
			_fidInspectionRegion.setGeometry(QRectF(fid.inspect_region.xmin, fid.inspect_region.ymin, fid.inspect_region.w, fid.inspect_region.h));

			fid.search_region.cx = cam_w / 2;
			fid.search_region.cy = cam_h / 2;
			fid.search_region.w = cam_w;
			fid.search_region.h = cam_h;
			fid.search_region.compute_extremum();
			_fidSearchRegion.setGeometry(QRectF(fid.search_region.xmin, fid.search_region.ymin, fid.search_region.w, fid.search_region.h));
		}
		return false;
	}

	if (!root.contains("fiducials")) return false;
	auto fids = root["fiducials"].toArray();

	ui.comboBox_fids->clear();
	_fiducialInfos.clear();

	for (int i = 0; i < fids.size(); i++) {
		auto fid = fids[i].toObject();

		FiducialInfo fidInfo;
		fidInfo.id = jsonHelper::getString(fid, QStringLiteral("id"));
		fidInfo.hasTeachPoint = jsonHelper::getBool(fid, QStringLiteral("teach_point_flag"));
		fidInfo.fiducial_method = jsonHelper::getInteger(fid, QStringLiteral("fiducial_method"), 0);
		fidInfo.score = jsonHelper::getInteger(fid, QStringLiteral("score"), 50);
		fidInfo.min_diameter = jsonHelper::getInteger(fid, QStringLiteral("min_diameter"), 280);
		fidInfo.max_diameter = jsonHelper::getInteger(fid, QStringLiteral("max_diameter"), 320);
		fromJson(fid["search_region"].toObject(), fidInfo.search_region);
		fromJson(fid["inspect_region"].toObject(), fidInfo.inspect_region);
		fromJson(fid["teach_point"].toObject(), fidInfo.teach_point);

		ui.comboBox_fids->addItem(fidInfo.id);
		_fiducialInfos.push_back(fidInfo);
	}
	if (root.contains(QStringLiteral("fiducial_board_dimensions")) && root[QStringLiteral("fiducial_board_dimensions")].isObject()) {
		QJsonObject boardDimsObj = root[QStringLiteral("fiducial_board_dimensions")].toObject();


		fidWidth = jsonHelper::getDouble(boardDimsObj, QStringLiteral("width_mm"), 0.0);
		fidHeight = jsonHelper::getDouble(boardDimsObj, QStringLiteral("height_mm"), 0.0);

		ct::logger::debug(QString("Loaded board dimensions into globals: Width=%1mm, Height=%2mm").arg(fidWidth).arg(fidHeight).toStdString().c_str());
	}
	else {
		ct::logger::debug("No 'fiducial_board_dimensions' object found in JSON. Global dimensions not updated from file.");
	}

	return true;
}

void VisionApp::displayFiducialImage(int index)
{
	auto path_fid = Common::Directory::getRecipeImagesPath() + QString("Fiducial/fid%1.jpg").arg(index + 1);

	if (QFile::exists(path_fid)) {
		displayFOV(QImage(path_fid));
		ui.graphicsViewFOV->fitInView(_pPixmapItemFOV, Qt::KeepAspectRatio);
		ui.graphicsViewFOV->centerOn(_pPixmapItemFOV);
	}
}

void VisionApp::toggleFidROISetupMode(bool state)
{
	if (_currentFidIndex >= _fiducialInfos.size()) {
		ct::logger::warn("[UB] Invalid fiducial index: %d", _currentFidIndex);
		return;
	}

	if (state) {
		auto cam_w = CAMManager::instance().getWidth(_camID);
		auto cam_h = CAMManager::instance().getHeight(_camID);

		if (_fidSearchRegion.getGeometry().width() < 1) {
			_fidInspectionRegion.setGeometry(QRectF(cam_w * 0.3, cam_h * 0.3, cam_w * 0.7, cam_h * 0.7));
			_fidSearchRegion.setGeometry(QRectF(0, 0, cam_w, cam_h));
		}

		_fidSearchRegion.show();
		_fidInspectionRegion.show();

		ui.toolButton_toggleFovView->animateClick();
		displayFiducialImage(_currentFidIndex);

		ui.toolButton_setupFidROI->setChecked(true);
	}
	else {
		_fidSearchRegion.hide();
		_fidInspectionRegion.hide();

		ui.toolButton_setupFidROI->setChecked(false);
	}
}

void VisionApp::updateFiducialSettingsJson(int index)
{
	if (index >= _fiducialInfos.size()) {
		ct::logger::warn("[UB] Invalid Fid index: %d\n", index);
		return;
	}

	_fiducialInfos[index].fiducial_method = ui.comboBox_fiducialMethod->currentIndex();
	_fiducialInfos[index].score = ui.lineEdit_fiducial_score->text().toInt();
	_fiducialInfos[index].min_diameter = ui.lineEdit_fiducial_minDiameter->text().toInt();
	_fiducialInfos[index].max_diameter = ui.lineEdit_fiducial_maxDiameter->text().toInt();
	saveFiducial();
}

void VisionApp::updateFiducialRegions()
{
	backupFiducial();

	if (_currentFidIndex >= _fiducialInfos.size())
	{
		ct::logger::warn("[UB] Invalid Fid index: %d\n", _currentFidIndex);
		return;
	}

	_fiducialInfos[_currentFidIndex].search_region.cx = _fidSearchRegion.getGeometry().center().x();
	_fiducialInfos[_currentFidIndex].search_region.cy = _fidSearchRegion.getGeometry().center().y();
	_fiducialInfos[_currentFidIndex].search_region.w = _fidSearchRegion.getGeometry().width();
	_fiducialInfos[_currentFidIndex].search_region.h = _fidSearchRegion.getGeometry().height();
	_fiducialInfos[_currentFidIndex].search_region.compute_extremum();

	_fiducialInfos[_currentFidIndex].inspect_region.cx = _fidInspectionRegion.getGeometry().center().x();
	_fiducialInfos[_currentFidIndex].inspect_region.cy = _fidInspectionRegion.getGeometry().center().y();
	_fiducialInfos[_currentFidIndex].inspect_region.w = _fidInspectionRegion.getGeometry().width();
	_fiducialInfos[_currentFidIndex].inspect_region.h = _fidInspectionRegion.getGeometry().height();
	_fiducialInfos[_currentFidIndex].inspect_region.compute_extremum();
}

void VisionApp::fiducialAlignment()
{
	if (!passwordPromptCorrect()) return;

	backupFiducial();
	auto h_scale = ScaleManager::instance().horizontal_um_per_px();
	auto v_scale = ScaleManager::instance().vertical_um_per_px();

	//collect all 4 point ficuial infos
	QVector<em::V2d> pointVectorOri;
	QVector<em::V2d> pointVector;
	if (_fiducialInfos.size() == 4)
	{
		for (auto fInfo : _fiducialInfos)
		{
			em::V2d point;
			int x_px = fInfo.inspect_region.cx - _imageSize.width() / 2;
			int y_px = fInfo.inspect_region.cy - _imageSize.height() / 2;
			point.x() = fInfo.teach_point.wx + util::px_to_mm(x_px, h_scale);
			point.y() = fInfo.teach_point.wy + util::px_to_mm(y_px, v_scale);

			pointVectorOri.append(point);

			qDebug() << "Original Fiducial Info: cx = " << fInfo.inspect_region.cx
				<< ", cy = " << fInfo.inspect_region.cy;
		}
	}

	pointVector = pointVectorOri;
	// classified this four points into Top Left, Top Right, Botton Left, Bottom right
	if (pointVector.size() == 4)
	{
		// Sorting the points by y-coordinate first, so we can classify Top and Bottom points
		std::sort(pointVector.begin(), pointVector.end(), [](const em::V2d& a, const em::V2d& b) {
			return a.y() < b.y(); // Sort by y-axis
			});

		// Now we have the two top points (index 0 and 1) and the two bottom points (index 2 and 3)
		em::V2d topLeft, topRight, bottomLeft, bottomRight;

		// Among the top two points, the one with the smaller x-coordinate is the top-left
		if (pointVector[0].x() < pointVector[1].x())
		{
			topLeft = pointVector[0];
			topRight = pointVector[1];
		}
		else
		{
			topLeft = pointVector[1];
			topRight = pointVector[0];
		}

		// Among the bottom two points, the one with the smaller x-coordinate is the bottom-left
		if (pointVector[2].x() < pointVector[3].x())
		{
			bottomLeft = pointVector[2];
			bottomRight = pointVector[3];
		}
		else
		{
			bottomLeft = pointVector[3];
			bottomRight = pointVector[2];
		}

		// Now you have the points classified
		qDebug() << "----------";
		qDebug() << "Top Left: " << topLeft.x() << "," << topLeft.y();
		qDebug() << "Top Right: " << topRight.x() << "," << topRight.y();
		qDebug() << "Bottom Left: " << bottomLeft.x() << "," << bottomLeft.y();
		qDebug() << "Bottom Right: " << bottomRight.x() << "," << bottomRight.y();


		// Compute the original width (distance between top-left and top-right)
		double originalWidth = std::sqrt(std::pow(topRight.x() - topLeft.x(), 2) +
			std::pow(topRight.y() - topLeft.y(), 2));

		// Compute the original height (distance between top-left and bottom-left)
		double originalHeight = std::sqrt(std::pow(bottomLeft.x() - topLeft.x(), 2) +
			std::pow(bottomLeft.y() - topLeft.y(), 2));

		// Adjust the points to make the rectangle perfectly horizontal
		// Fix the top-left point

		// Top-right: Set the y to the top-left y, but compute the new x based on the original width
		topRight.y() = topLeft.y();
		topRight.x() = topLeft.x() + originalWidth;

		// Bottom-left: Adjust y to reflect the original height, keep the same x as the top-left
		bottomLeft.x() = topLeft.x();
		bottomLeft.y() = topLeft.y() + originalHeight;

		// Bottom-right: Adjust both x and y based on the new top-right and bottom-left coordinates
		bottomRight.x() = topRight.x();
		bottomRight.y() = bottomLeft.y();

		// Now pointVector needs to be updated with the modified points
		pointVector[0] = topLeft;
		pointVector[1] = topRight;
		pointVector[2] = bottomLeft;
		pointVector[3] = bottomRight;

		// Debug or return the results
		qDebug() << "----------";
		qDebug() << "Top Left: " << topLeft.x() << "," << topLeft.y();
		qDebug() << "Top Right: " << topRight.x() << "," << topRight.y();
		qDebug() << "Bottom Left: " << bottomLeft.x() << "," << bottomLeft.y();
		qDebug() << "Bottom Right: " << bottomRight.x() << "," << bottomRight.y();


		// find back ori vector index
		int topRightIndex;
		int topLeftIndex;
		int botRightIndex;
		int botLeftIndex;

		
		for (int l = 0; l < pointVector.size(); l++)
		{
			int index;
			double shortestDistance = 999999999;
			em::V2d p = pointVector[l];
			for (int k = 0; k < pointVectorOri.size(); k++)
			{
				em::V2d oriP = pointVectorOri[k];

				double distance = std::sqrt(std::pow(oriP.x() - p.x(), 2) +
					std::pow(oriP.y() - p.y(), 2));

				if (distance < shortestDistance)
				{
					shortestDistance = distance;
					index = k;
				}

			}
			if (l == 0) topLeftIndex = index;
			else if (l == 1) topRightIndex = index;
			else if (l == 2) botLeftIndex = index;
			else if (l == 3) botRightIndex = index;
		}

		qDebug() << "Top left index: " << topLeftIndex;
		qDebug() << "Top right index: " << topRightIndex;
		qDebug() << "Bot left index: " << botLeftIndex;
		qDebug() << "Boit right index: " << botRightIndex;


		// Back-calculate to pixel values and update _fiducialInfos
		for (int i = 0; i < 4; ++i)
		{
			em::V2d point = pointVector[i];
			
			int oriIndex;
			if (i == 0) oriIndex = topLeftIndex;
			else if (i == 1) oriIndex = topRightIndex;
			else if (i == 2) oriIndex = botLeftIndex;
			else if (i == 3) oriIndex = botRightIndex;

			// Convert world coordinates (mm) back to pixel values
			int x_px = util::mm_to_px(point.x() - _fiducialInfos[oriIndex].teach_point.wx, h_scale);
			int y_px = util::mm_to_px(point.y() - _fiducialInfos[oriIndex].teach_point.wy, v_scale);

			// Update the center of the inspection region (cx, cy) in pixel coordinates
			_fiducialInfos[oriIndex].inspect_region.cx = x_px + _imageSize.width() / 2;
			_fiducialInfos[oriIndex].inspect_region.cy = y_px + _imageSize.height() / 2;

			// Debug the updated fiducial info
			qDebug() << "Updated Fiducial Info [" << oriIndex << "]: cx = " << _fiducialInfos[oriIndex].inspect_region.cx
				<< ", cy = " << _fiducialInfos[oriIndex].inspect_region.cy;
		}

		saveFiducial();

	}


}
void VisionApp::fiducialSize()
{
	QVector<em::V2d> fiducialPoints;
	fiducialPoints.clear();
	fiducialPoints.reserve(_fiducialInfos.size());

	if (_fiducialInfos.size() >= 2)
	{
		for (const auto& fInfo : _fiducialInfos)
		{
			em::V2d point;

			point.x() = fInfo.teach_point.wx;
			point.y() = fInfo.teach_point.wy;

			fiducialPoints.append(point);

			qDebug() << "Original Fiducial Info: cx =" << fInfo.inspect_region.cx
				<< ", cy =" << fInfo.inspect_region.cy;
		}
	}
	else
	{
		qWarning() << "Need at least 2 fiducials, but only have"
			<< _fiducialInfos.size();
	}
	double minX = std::numeric_limits<double>::infinity();
	double maxX = -std::numeric_limits<double>::infinity();
	double minY = std::numeric_limits<double>::infinity();
	double maxY = -std::numeric_limits<double>::infinity();
	for (auto& p : fiducialPoints) {
		minX = std::min(minX, p.x());
		maxX = std::max(maxX, p.x());
		minY = std::min(minY, p.y());
		maxY = std::max(maxY, p.y());
	}
	double width_mm = maxX - minX;
	double height_mm = maxY - minY;
	qDebug() << "Dimension (mm): width =" << width_mm
		<< ", height =" << height_mm;
	fidWidth = width_mm;
	fidHeight = height_mm;
	saveFiducial();
}

void VisionApp::backupFiducial()
{
	QString fiducialBackupPath = Common::Directory::getRecipeCurrentPath() + "FiducialBackup/";

	//create Directory
	qDebug() << "backup:" << fiducialBackupPath;
	CreateDirectoryA(fiducialBackupPath.toStdString().c_str(), NULL);

	QString fiducialBackupIDPath = Common::Directory::getRecipeCurrentPath() + "FiducialBackup/fiducial_" + generateTimeStampID().c_str();
	CreateDirectoryA(fiducialBackupIDPath.toStdString().c_str(), NULL);

	QString fiducialJsonPath = Common::Directory::getRecipeCurrentPath() + "fiducial.json";
	QString fiducialImagePath = Common::Directory::getRecipeImagesPath() + QString("Fiducial/");

	// Copy fiducial.json
	QString targetJsonPath = fiducialBackupIDPath + "/fiducial.json";
	if (!QFile::copy(fiducialJsonPath, targetJsonPath)) {
		qWarning() << "Failed to copy fiducial.json to backup folder.";
	}

	// Copy all files from Fiducial image path to backup
	QDir sourceDir(fiducialImagePath);
	if (!sourceDir.exists()) {
		qWarning() << "Fiducial image source directory does not exist:" << fiducialImagePath;
		return;
	}

	QDirIterator it(sourceDir.absolutePath(), QDir::Files, QDirIterator::Subdirectories);
	while (it.hasNext()) {
		QString sourceFilePath = it.next();
		QFileInfo fileInfo(sourceFilePath);
		QString destFilePath = fiducialBackupIDPath + "/" + fileInfo.fileName();

		if (!QFile::copy(sourceFilePath, destFilePath)) {
			qWarning() << "Failed to copy file:" << sourceFilePath << "to" << destFilePath;
		}
	}

	qDebug() << "Backup completed at:" << fiducialBackupIDPath;


}
