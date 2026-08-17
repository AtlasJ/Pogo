#include "VisionApp.h"
#include "CAMManager.h"
#include "ImageViewerTab.h"

void VisionApp::initViewEditor() {
	
	//General
	connect(ui.toolButton_highlightSameViews, &QToolButton::clicked, this, [=]() {

		auto viewIDs = getSelectedViewIDs();

		if (viewIDs.size() == 0) {
			showMsg("No views selected.");
			return;
		}

		auto viewID = viewIDs[0];

		if (!_views.contains(viewID)) {
			ct::logger::error("[IM] Failed to highlight same view. Invalid view ID: %s", viewID.toStdString().c_str());
			return;
		}

		const auto& mainView = _views[viewID];

		for (const auto& v : _views) {
			if (mainView.zstack == v.zstack && mainView.opticIDs == v.opticIDs) {
				v.pDragBox->setSelected(true);
			}
			else {
				v.pDragBox->setSelected(false);
			}
		}
	});

	//Zstack
	ui.lineEdit_zstack_selectedView->setText("-");
	ui.lineEdit_zstack_step_um->setText("0");
	ui.lineEdit_zstack_iteration->setText("0");
	ui.lineEdit_zstack_range_um->setText("0");
	ui.comboBox_zstack_acqType->setCurrentIndex(0);
	ui.groupBox_acqType_preset->hide();
	ui.groupBox_acqType_encoder->hide();


	connect(ui.toolButton_zstack_save, &QToolButton::clicked, this, [=]() {
		auto viewIDs = getSelectedViewIDs();
		updateViewEditorSetting(viewIDs); 
		showMsg("Changes saved!");
	});

	QObject::connect(ui.comboBox_zstack_acqType, QOverload<int>::of(&QComboBox::currentIndexChanged), [&](int index) {
		if (ui.comboBox_zstack_acqType->currentText() == "Preset") {
			ui.groupBox_acqType_preset->show();
			ui.groupBox_acqType_encoder->hide();
			ui.groupBox_acqType_time->hide();
		}
		else if (ui.comboBox_zstack_acqType->currentText() == "Encoder") {
			ui.groupBox_acqType_preset->hide();
			ui.groupBox_acqType_encoder->show();
			ui.groupBox_acqType_time->hide();
		}
		else if (ui.comboBox_zstack_acqType->currentText() == "Time") {
			ui.groupBox_acqType_preset->hide();
			ui.groupBox_acqType_encoder->hide();
			ui.groupBox_acqType_time->show();
		}
	});

	connect(ui.toolButton_cropGuidingRoi, &QToolButton::clicked, this, [=]() {
		drawCropGuidingRoi();
	});


	connect(ui.toolButton_loadGreyCard, &QToolButton::clicked, this, [=]() {
		loadGreyCard();
		});


	loadGreyCardPathIfAny();

}

void VisionApp::checkSelectedView()
{	
	auto viewIDs = getSelectedViewIDs();
	
	if (viewIDs.size() == 0) {
		ui.lineEdit_zstack_selectedView->setText("-");
		updateViewEditorSettingUI("-");
		return;
	}

	if (!_views.contains(viewIDs[0])) {
		showMsg("Selected ROI does not linked to any view");
		return;
	}
	
	if (viewIDs.size() == 1) {
		const auto& view = _views[viewIDs[0]];

		ui.lineEdit_zstack_selectedView->setText(view.name);
		updateViewEditorSettingUI(view.id);
	}
	else if (viewIDs.size() > 1) {
		const auto& view = _views[viewIDs[0]];
		ui.lineEdit_zstack_selectedView->setText("Multiple views selected. Showing " + view.name);
		updateViewEditorSettingUI(view.id);
	}
}

void VisionApp::updateViewEditorSettingUI(QString viewID)
{
	if (viewID == "-") {
		for (int i = 0; i < ui.listWidget_viewOpticSelection->count(); i++) {
			auto item = ui.listWidget_viewOpticSelection->item(i);
			item->setCheckState(Qt::Unchecked);
		}

		ui.label_cameraInfo->setText(QString("Camera Size: 0, 0"));
	
		ui.checkBox_ve_crop->setChecked(false);
		ui.checkBox_ve_resize->setChecked(false);
		ui.lineEdit_ve_cropX->setText(0);
		ui.lineEdit_ve_cropY->setText(0);
		ui.lineEdit_ve_cropW->setText(0);
		ui.lineEdit_ve_cropH->setText(0);
		ui.lineEdit_ve_resizeW->setText(0);
		ui.lineEdit_ve_resizeH->setText(0);
		ui.checkBox_enableFlatField->setChecked(false);

		ui.lineEdit_zstack_step_um->setText("0");
		ui.lineEdit_zstack_iteration->setText("0");
		ui.lineEdit_zstack_range_um->setText("0");
		ui.lineEdit_zstack_time_interval_ms->setText("0");
		ui.checkBox_zstack_2D->setChecked(false);
		ui.checkBox_zstack_3D->setChecked(false);
	}
	else {
		if (!_views.contains(viewID)) {
			showMsg("Selected ROI does not linked to any view");
			return;
		}

		const auto& v = _views[viewID];
		const auto& zs = _views[viewID].zstack;
		auto camID = _views[viewID].camID;
		auto cam = CAMManager::instance().camera(camID);
		if (cam) {
			ui.label_cameraInfo->setText(QString("Camera Size: %1, %2").arg(cam->getWidth()).arg(cam->getHeight()));
		}
		
		for (int i = 0; i < ui.listWidget_viewOpticSelection->count(); i++) {
			auto item = ui.listWidget_viewOpticSelection->item(i);
			if (v.opticIDs.contains(item->whatsThis())) {
				item->setCheckState(Qt::Checked);
			}
			else {
				item->setCheckState(Qt::Unchecked);
			}
		}

		ui.checkBox_ve_crop->setChecked(v.preprocess.crop);
		ui.checkBox_ve_resize->setChecked(v.preprocess.resize);
		ui.lineEdit_ve_cropX->setText(QString::number(v.preprocess.cropRect.x()));
		ui.lineEdit_ve_cropY->setText(QString::number(v.preprocess.cropRect.y()));
		ui.lineEdit_ve_cropW->setText(QString::number(v.preprocess.cropRect.width()));
		ui.lineEdit_ve_cropH->setText(QString::number(v.preprocess.cropRect.height()));
		ui.lineEdit_ve_resizeW->setText(QString::number(v.preprocess.resizeRect.width()));
		ui.lineEdit_ve_resizeH->setText(QString::number(v.preprocess.resizeRect.height()));
		ui.checkBox_enableFlatField->setChecked(v.preprocess.flatfield);

		if (zs.acq_type == ct::s_preset) {
			ui.comboBox_zstack_acqType->setCurrentIndex(0);
			ui.groupBox_acqType_preset->show();
			ui.groupBox_acqType_encoder->hide();
			ui.groupBox_acqType_time->hide();
		}
		else if(zs.acq_type == ct::s_encoder) {
			ui.comboBox_zstack_acqType->setCurrentIndex(1);
			ui.groupBox_acqType_preset->hide();
			ui.groupBox_acqType_encoder->show();
			ui.groupBox_acqType_time->hide();
		}
		else if (zs.acq_type == ct::s_time) {
			ui.comboBox_zstack_acqType->setCurrentIndex(2);
			ui.groupBox_acqType_preset->hide();
			ui.groupBox_acqType_encoder->hide();
			ui.groupBox_acqType_time->show();
		}

		ui.lineEdit_zstack_step_um->setText(QString::number(zs.step_um));
		ui.lineEdit_zstack_iteration->setText(QString::number(zs.preset_iteration));
		ui.lineEdit_zstack_range_um->setText(QString::number(zs.encoder_range_um));
		ui.lineEdit_zstack_time_interval_ms->setText(QString::number(zs.time_interval_ms));
		ui.checkBox_zstack_2D->setChecked(zs.generate_2D_stack);
		ui.checkBox_zstack_3D->setChecked(zs.generate_3D_stack);


	}
}

void VisionApp::updateViewEditorSetting(QVector<QString> viewIDs)
{
	QString acq_type = ct::s_preset;
	if (ui.comboBox_zstack_acqType->currentIndex() == 1) acq_type = ct::s_encoder;
	else if (ui.comboBox_zstack_acqType->currentIndex() == 2) acq_type = ct::s_time;

	bool hasChecked = false;
	for (int i = 0; i < ui.listWidget_viewOpticSelection->count(); i++) {
		auto item = ui.listWidget_viewOpticSelection->item(i);
		if (item->checkState() == Qt::Checked) {
			hasChecked = true;
			break;
		}
	}

	if (!hasChecked) {
		showMsg("Unable to save! Need at least one optic enable.");
		return;
	}

	for (const auto& viewID : viewIDs) {

		if (!_views.contains(viewID)) {
			ct::logger::error("[IM] Failed to update view settings. Invalid view ID: %s", viewID.toStdString().c_str());
			continue;
		}

		auto& view = _views[viewID];

		view.opticIDs.clear();

		for (int i = 0; i < ui.listWidget_viewOpticSelection->count(); i++) {
			auto item = ui.listWidget_viewOpticSelection->item(i);
			if (item->checkState() == Qt::Checked) {
				auto opticID = item->whatsThis();
				view.opticIDs.insert(opticID);
			}
		}

		view.preprocess.crop = ui.checkBox_ve_crop->isChecked();
		view.preprocess.resize = ui.checkBox_ve_resize->isChecked();
		view.preprocess.cropRect.setX(ui.lineEdit_ve_cropX->text().toInt());
		view.preprocess.cropRect.setY(ui.lineEdit_ve_cropY->text().toInt());
		view.preprocess.cropRect.setWidth(ui.lineEdit_ve_cropW->text().toInt());
		view.preprocess.cropRect.setHeight(ui.lineEdit_ve_cropH->text().toInt());
		view.preprocess.resizeRect.setWidth(ui.lineEdit_ve_resizeW->text().toInt());
		view.preprocess.resizeRect.setHeight(ui.lineEdit_ve_resizeH->text().toInt());

		view.zstack.acq_type = acq_type;
		view.zstack.step_um = ui.lineEdit_zstack_step_um->text().toInt();
		view.zstack.preset_iteration = ui.lineEdit_zstack_iteration->text().toInt();
		view.zstack.encoder_range_um = ui.lineEdit_zstack_range_um->text().toInt();
		view.zstack.time_interval_ms = ui.lineEdit_zstack_time_interval_ms->text().toInt();
		view.zstack.generate_2D_stack = ui.checkBox_zstack_2D->isChecked();
		view.zstack.generate_3D_stack = ui.checkBox_zstack_3D->isChecked();

		view.preprocess.flatfield = ui.checkBox_enableFlatField->isChecked();
	}

	saveView();
}

void VisionApp::logViews(QString msg)
{
	ct::logger::debug("%s", msg.toStdString().c_str());
	for (auto v : _views) {
		ct::logger::debug("View: %s", v.id.toStdString().c_str());
	}
}


void VisionApp::loadGreyCard()
{
	QString fileName = QFileDialog::getOpenFileName(
		this,
		tr("Select Grey Card Image"),
		QString(),
		tr("Images (*.png *.jpg *.jpeg *.bmp *.tif *.tiff)")
	);

	if (fileName.isEmpty()) {
		QMessageBox::information(this, tr("No File Selected"), tr("You did not select any image."));
		return;
	}

	ui.lineEdit_greycardPath->setText(fileName);
	if (!saveGreyCardPath(fileName)) {
		ct::logger::error("[IM] Failed to save grey card config.");
	}

}


QString VisionApp::greyCardConfigFile() const
{
	QDir dir(Common::Directory::ConfigPath());
	return dir.filePath("greycard.json");
}

void VisionApp::loadGreyCardPathIfAny()
{
	const QString cfg = greyCardConfigFile();
	QFile f(cfg);
	if (!f.exists()) return;
	if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) return;

	const auto doc = QJsonDocument::fromJson(f.readAll());
	f.close();
	if (!doc.isObject()) return;

	const QJsonObject obj = doc.object();
	const QString savedPath = obj.value("greycardPath").toString().trimmed();
	if (savedPath.isEmpty()) return;

	// Update the UI
	ui.lineEdit_greycardPath->setText(savedPath);


	if (!QFileInfo::exists(savedPath)) {
		 ui.lineEdit_greycardPath->clear();
	}
}


bool VisionApp::saveGreyCardPath(const QString& path)
{
	QDir dir(Common::Directory::ConfigPath());
	if (!dir.exists()) {
		if (!dir.mkpath(".")) return false;
	}

	const QString cfg = dir.filePath("greycard.json");
	QFile f(cfg);
	if (!f.open(QIODevice::WriteOnly | QIODevice::Text)) return false;

	QJsonObject obj;
	obj.insert("greycardPath", QDir::toNativeSeparators(path));
	QJsonDocument doc(obj);
	f.write(doc.toJson(QJsonDocument::Indented));
	f.close();
	return true;
}