#include "VisionApp.h"
#include "LSC_VLP.h"
#include "LSC_OPT.h"
#include "LSC_CST_MVL.h"
#include "uidGenerator.h"
#include "ImagePathManager.h"
#include "cvUtil.h"
#include "QOrderedSet.h"
#include "CAMManager.h"
#include "UnitConfigTab.h"
#include "AuditLog.h"

void VisionApp::initLSC()
{
	loadLSCConfig();

	//main optic
	_mainOptics[_camID].id = "optic123";
	_mainOptics[_camID].name = "RGB";
	_mainOptics[_camID].type = ct::s_color;
	_mainOptics[_camID].tag = "Main";
		
	//adaptive ui
	//get number of optic type
	ui.comboBox_lightingType->clear();
	while (ui.stackedWidget_lightingType->count() > 0) {
		QWidget* widget = ui.stackedWidget_lightingType->widget(0);
		ui.stackedWidget_lightingType->removeWidget(widget);
		delete widget;  // Optionally delete the widget if you want to free memory
	}
	ct::logger::debug("S1");
	QOrderedSet<QString> optics;
	for (const auto& key : LSCManager::instance().channels()) {
		auto ch = LSCManager::instance().channels()[key];
		optics.insert(ch.optic);
	}
	ct::logger::debug("S2");
	for (const auto& opt : optics.toList()) {
		ui.comboBox_lightingType->addItem(opt);
		
		QWidget* page = new QWidget();
		QGridLayout* layout = new QGridLayout(page);
		page->setLayout(layout);
		ui.stackedWidget_lightingType->addWidget(page);

		int rowIndex = -1;

		for (const auto& key : LSCManager::instance().channels()) {

			auto ch = LSCManager::instance().channels()[key];

			if (ch.optic == opt) {
				QLabel* label = new QLabel(ch.name);
				QSlider* slider = new QSlider(Qt::Horizontal);
				QLineEdit* lineEdit = new QLineEdit();
				QCheckBox* checkBox = new QCheckBox();

				slider->setWhatsThis(ch.id);
				slider->setRange(g_min_intensity, g_max_intensity);
				connect(slider, &QSlider::sliderMoved, this, [=](int value) {

					auto ch = slider->whatsThis();

					OpticsControl::instance().setBrightness(_camID, slider->whatsThis());
					OpticsControl::instance().setIntensity(ch, value);

					if (isPage(UIPage::LIGHTING)) {
						_channelSliders[ch]->setValue(value);
						_channelLineEdits[ch]->setText(QString::number(value));
					}
				});

				QIntValidator* validator_lineEdit = new QIntValidator(g_min_intensity, g_max_intensity, lineEdit);
				lineEdit->setValidator(validator_lineEdit);
				lineEdit->setWhatsThis(ch.id);
				connect(lineEdit, &QLineEdit::textEdited, this, [=](const QString& text) {
					
					auto ch = lineEdit->whatsThis();
					auto value = text.toInt();

					OpticsControl::instance().setBrightness(_camID, lineEdit->whatsThis());
					OpticsControl::instance().setIntensity(ch, value);

					if (isPage(UIPage::LIGHTING)) {
						_channelSliders[ch]->setValue(value);
						_channelLineEdits[ch]->setText(QString::number(value));
					}
				});

				checkBox->setWhatsThis(ch.id);
				connect(checkBox, &QCheckBox::stateChanged, this, [=](int state) {
					OpticsControl::instance().toggleChannel(checkBox->whatsThis(), state);
				});

				if (opt.contains("RGB")) {
					if (ch.lighting_type == "R") rowIndex = 0;
					else if (ch.lighting_type == "G") rowIndex = 1;
					else if (ch.lighting_type == "B") rowIndex = 2;
				}
				else {
					rowIndex++;
				}

				layout->addWidget(label, rowIndex, 0);
				layout->addWidget(slider, rowIndex, 1);
				layout->addWidget(lineEdit, rowIndex, 2);
				layout->addWidget(checkBox, rowIndex, 3);

				_channelLineEdits.insert(ch.id, lineEdit);
				_channelSliders.insert(ch.id, slider);
				_channelToggle.insert(ch.id, checkBox);
			}
		}
	}

	ct::logger::debug("S3");

	connect(ui.comboBox_lightingType, QOverload<int>::of(&QComboBox::activated), [&](int index) {
		ui.stackedWidget_lightingType->setCurrentIndex(index);
	});

	//recipe optics
	connect(ui.toolButton_addColorOptic, &QToolButton::clicked, this, [=]() {

		OpticsInfo optic;

		optic.type = ct::s_color;
		
		uidGenerator idGen;
		optic.id = QString("optic") + idGen.id().c_str();

		int index = 1;
		for (auto& r : _recipeOptics) {
			if (r.name.contains("RGB")) {
				auto nums = util::getNumsFromString(r.name);
				if (nums.size() == 1) {
					if (index <= nums.at(0)) {
						index = nums.at(0) + 1;
					}
				}
			}
		}

		optic.name = QString("RGB%1").arg(index);

		_recipeOptics.insert(optic.id, optic);

		auto item = new QListWidgetItem(optic.name);
		item->setWhatsThis(optic.id);

		ui.listWidget_recipeOptics->addItem(item);

		auto item2 = new QListWidgetItem(optic.name);
		item2->setWhatsThis(optic.id);
		item2->setCheckState(Qt::Checked);
		ui.listWidget_viewOpticSelection->addItem(item2);

		item->setSelected(true);

		for (auto& view : _views) {
			view.opticIDs.insert(optic.id);
		}

		saveView();
	});

	connect(ui.toolButton_addMonoOptic, &QToolButton::clicked, this, [=]() {

		QStringList cams;
		cams.append("cam1");

		auto camID = promptComboBox(cams, "Camera Selection", "Please select the camera for the current optics");

		OpticsInfo optic;

		optic.camID = camID;
		optic.type = ct::s_mono;
		if (CAMManager::instance().getChannel(_camID) == 3) optic.type = ct::s_color;

		uidGenerator idGen;
		optic.id = QString("optic") + idGen.id().c_str();
		
		int index = 1;
		for (auto& r : _recipeOptics) {
			if (r.name.contains("Mono")) {
				auto nums = util::getNumsFromString(r.name);
				if (nums.size() == 1) {
					if (index <= nums.at(0)) {
						index = nums.at(0) + 1;
					}
				}
			}
		}

		optic.name = QString("Mono%1").arg(index);

		_recipeOptics.insert(optic.id, optic);

		auto item = new QListWidgetItem(optic.name);
		item->setWhatsThis(optic.id);

		ui.listWidget_recipeOptics->addItem(item);

		auto item2 = new QListWidgetItem(optic.name);
		item2->setWhatsThis(optic.id);
		item2->setCheckState(Qt::Checked);
		ui.listWidget_viewOpticSelection->addItem(item2);

		item->setSelected(true);

		for (auto& view : _views) {
			view.opticIDs.insert(optic.id);
		}

		saveView();
	});

	connect(ui.toolButton_deleteRecipeOptic, &QToolButton::clicked, this, [=]() {
		QList<QListWidgetItem*> selectedItems = ui.listWidget_recipeOptics->selectedItems();
		for (QListWidgetItem* item : selectedItems) {
			auto id = item->whatsThis();

			if (_recipeOptics[id].tag == "Main") {
				showMsg("Main optic cannot be deleted!");
				return;
			}

			AuditLog::instance().log(QStringLiteral("OPTIC_DELETE"), id);
			_recipeOptics.remove(id);
			delete ui.listWidget_recipeOptics->takeItem(ui.listWidget_recipeOptics->row(item));

			auto item2 = getViewOpticListItem(id);
			if (item2) delete ui.listWidget_viewOpticSelection->takeItem(ui.listWidget_viewOpticSelection->row(item2));

			for (auto& view : _views) {
				view.opticIDs.remove(id);
			}
		}
		
		saveView();
	});

	connect(ui.toolButton_setOpticAsMain, &QToolButton::clicked, this, [=]() {

		//Demote previous Main
		for (int i = 0; i < ui.listWidget_recipeOptics->count(); ++i) {
			auto item = ui.listWidget_recipeOptics->item(i);
			auto id = item->whatsThis();

			if (_recipeOptics[id].tag == "Main") {
				_recipeOptics[id].tag = "";
				item->setIcon(QIcon());
			}
		}

		//Promote selected to Main
		QList<QListWidgetItem*> selectedItems = ui.listWidget_recipeOptics->selectedItems();
		for (QListWidgetItem* item : selectedItems) {
			auto id = item->whatsThis();

			_recipeOptics[id].tag = "Main";
			item->setIcon(QIcon(":/8Icon/Icon/icon8/icons8-m-100.png"));
			_mainOptics[_camID] = _recipeOptics[id];

			break;
		}
	});

	connect(ui.listWidget_recipeOptics, &QListWidget::itemSelectionChanged, [&]() {
		QList<QListWidgetItem*> selectedItems = ui.listWidget_recipeOptics->selectedItems();
		for (QListWidgetItem* item : selectedItems) {

			if (_recipeOptics.contains(item->whatsThis())) {
				auto& opt = _recipeOptics[item->whatsThis()];

				ui.lineEdit_opticName->setText(opt.name);

				ui.lineEdit_recipeExposure->setText(QString::number(opt.exposure));
				ui.lineEdit_recipeGain->setText(QString::number(opt.gain));

				//show frame type
				if (opt.type == ct::s_color && CAMManager::instance().getChannel(_camID) == 1) {
					ui.frame_opticColor->show();
					ui.frame_opticMono->hide();

					updateOpticBandUI(opt.R, ui.lineEdit_redBuffer);
					updateOpticBandUI(opt.G, ui.lineEdit_greenBuffer);
					updateOpticBandUI(opt.B, ui.lineEdit_blueBuffer);
				}
				else {
					ui.frame_opticColor->hide();
					ui.frame_opticMono->show();

					updateOpticBandUI(opt.M, ui.lineEdit_monoBuffer);
				}
			}
		}
	});

	connect(ui.toolButton_updateExposureAndGain, &QToolButton::clicked, this, [=]() {
		QList<QListWidgetItem*> selectedItems = ui.listWidget_recipeOptics->selectedItems();
		for (QListWidgetItem* item : selectedItems) {
			auto id = item->whatsThis();

			_recipeOptics[id].exposure = ui.lineEdit_exposure->text().toDouble();
			_recipeOptics[id].gain = ui.lineEdit_gain->text().toDouble();
			AuditLog::instance().log(QStringLiteral("OPTIC_EXPOSURE_GAIN"), QStringLiteral("%1 exp=%2 gain=%3").arg(id).arg(_recipeOptics[id].exposure).arg(_recipeOptics[id].gain));
			ui.lineEdit_recipeExposure->setText(QString::number(_recipeOptics[id].exposure));
			ui.lineEdit_recipeGain->setText(QString::number(_recipeOptics[id].gain));

			if (_recipeOptics[id].tag == "Main") {
				_mainOptics[_camID] = _recipeOptics[id];
			}
		}
	});

	connect(ui.toolButton_updateRedBuffer, &QToolButton::clicked, this, [=]() {
		QList<QListWidgetItem*> selectedItems = ui.listWidget_recipeOptics->selectedItems();
		for (QListWidgetItem* item : selectedItems) {
			auto id = item->whatsThis();
			_recipeOptics[id].R.clear();
			
			for (auto& i : _channelLineEdits) {
				int intensity = i->text().toInt();
				_recipeOptics[id].R.insert(i->whatsThis(), intensity);
			}

			if (_recipeOptics[id].tag == "Main") {
				_mainOptics[_camID] = _recipeOptics[id];
			}

			updateOpticBandUI(_recipeOptics[id].R, ui.lineEdit_redBuffer);
		}
	});

	connect(ui.toolButton_updateGreenBuffer, &QToolButton::clicked, this, [=]() {
		QList<QListWidgetItem*> selectedItems = ui.listWidget_recipeOptics->selectedItems();
		for (QListWidgetItem* item : selectedItems) {
			auto id = item->whatsThis();
			_recipeOptics[id].G.clear();

			for (auto& i : _channelLineEdits) {
				int intensity = i->text().toInt();
				_recipeOptics[id].G.insert(i->whatsThis(), intensity);
			}

			if (_recipeOptics[id].tag == "Main") {
				_mainOptics[_camID] = _recipeOptics[id];
			}

			updateOpticBandUI(_recipeOptics[id].G, ui.lineEdit_greenBuffer);
		}
	});

	connect(ui.toolButton_updateBlueBuffer, &QToolButton::clicked, this, [=]() {
		QList<QListWidgetItem*> selectedItems = ui.listWidget_recipeOptics->selectedItems();
		for (QListWidgetItem* item : selectedItems) {
			auto id = item->whatsThis();
			_recipeOptics[id].B.clear();

			for (auto& i : _channelLineEdits) {
				int intensity = i->text().toInt();
				_recipeOptics[id].B.insert(i->whatsThis(), intensity);
			}

			if (_recipeOptics[id].tag == "Main") {
				_mainOptics[_camID] = _recipeOptics[id];
			}

			updateOpticBandUI(_recipeOptics[id].B, ui.lineEdit_blueBuffer);
		}
	});

	connect(ui.toolButton_updateAllBuffer, &QToolButton::clicked, this, [=]() {

		QList<QListWidgetItem*> selectedItems = ui.listWidget_recipeOptics->selectedItems();
		for (QListWidgetItem* item : selectedItems) {
			auto id = item->whatsThis();

			assignOpticBasedOnLightingType(_recipeOptics[id]);
			
			if (_recipeOptics[id].tag == "Main") {
				_mainOptics[_camID] = _recipeOptics[id];
			}

			updateOpticBandUI(_recipeOptics[id].R, ui.lineEdit_redBuffer);
			updateOpticBandUI(_recipeOptics[id].G, ui.lineEdit_greenBuffer);
			updateOpticBandUI(_recipeOptics[id].B, ui.lineEdit_blueBuffer);
		}
	});

	connect(ui.toolButton_updateMonoBuffer, &QToolButton::clicked, this, [=]() {
		QList<QListWidgetItem*> selectedItems = ui.listWidget_recipeOptics->selectedItems();
		for (QListWidgetItem* item : selectedItems) {
			auto id = item->whatsThis();
			
			_recipeOptics[id].M.clear();

			for (auto& i : _channelLineEdits) {
				int intensity = i->text().toInt();
				_recipeOptics[id].M.insert(i->whatsThis(), intensity);
			}

			updateOpticBandUI(_recipeOptics[id].M, ui.lineEdit_monoBuffer);
		}
	});

	connect(ui.toolButton_updateToAllChannels, &QToolButton::clicked, this, [=]() {
		QList<QListWidgetItem*> selectedItems = ui.listWidget_recipeOptics->selectedItems();
		for (QListWidgetItem* item : selectedItems) {
			auto id = item->whatsThis();
			auto opt = _recipeOptics[id];
			
			ui.lineEdit_exposure->setText(QString::number(opt.exposure));
			ui.lineEdit_gain->setText(QString::number(opt.gain));

			if (opt.type == ct::s_color && CAMManager::instance().getChannel(_camID) == 1) {
				updateOpticBasedOnLightingType(opt);
			}
			else {
				for (const auto& key : opt.M.keys()) {
					OpticsControl::instance().setIntensity(key, opt.M[key]);
				}
			}
		}
	});

	connect(ui.toolButton_snapMono, &QToolButton::clicked, this, [=]() {
		OpticsControl::instance().toggleAllChannels(false);

		OpticsInfo opt;
		opt.type = ct::s_mono;
		if (CAMManager::instance().getChannel(_camID) == 3) opt.type = ct::s_color;

		for (auto& i : _channelLineEdits) {
			opt.M.insert(i->whatsThis(), i->text().toInt());
		}

		opt.exposure = ui.lineEdit_exposure->text().toDouble();
		opt.gain = ui.lineEdit_gain->text().toDouble();

		printBand("M", opt.M);

		CAMManager::instance().resetFrame(_camID);
		CAMManager::instance().frame(_camID)->viewID = "main";
		emit snapImage(opt, "", "");
	});

	connect(ui.toolButton_snapColor, &QToolButton::clicked, this, [=]() {
		OpticsControl::instance().toggleAllChannels(false);

		OpticsInfo opt;
		opt.type = ct::s_color;

		opt.exposure = ui.lineEdit_exposure->text().toDouble();
		opt.gain = ui.lineEdit_gain->text().toDouble();

		assignOpticBasedOnLightingType(opt);

		printBand("R", opt.R);
		printBand("G", opt.G);
		printBand("B", opt.B);

		CAMManager::instance().resetFrame(_camID);
		CAMManager::instance().frame(_camID)->viewID = "main";
		emit snapImage(opt, "", "");
	});

	connect(ui.toolButton_snapBuffer, &QToolButton::clicked, this, [=]() {
		stopLiveView();

		OpticsControl::instance().toggleAllChannels(false);

		QList<QListWidgetItem*> selectedItems = ui.listWidget_recipeOptics->selectedItems();
		for (QListWidgetItem* item : selectedItems) {
			auto id = item->whatsThis();
			ct::logger::info("Snap buffer: %s", id.toStdString().c_str());

			CAMManager::instance().resetFrame(_camID);
			CAMManager::instance().frame(_camID)->viewID = "main";
			emit snapImage(_recipeOptics[id], "", "");
		}
	});

	connect(ui.lineEdit_opticName, &QLineEdit::textEdited, [&](const QString& newText) {
		QList<QListWidgetItem*> selectedItems = ui.listWidget_recipeOptics->selectedItems();
		for (QListWidgetItem* item : selectedItems) {
			auto id = item->whatsThis();
			_recipeOptics[id].name = newText;
			item->setText(newText);

			auto item2 = getViewOpticListItem(id);
			item2->setText(newText);
		}
	});

	//calibration
	ui.toolButton_calibrationDropDown->setWhatsThis(UI_COLLAPSE);
	ui.frame_whiteBalance->hide();

	connect(ui.toolButton_calibrationDropDown, &QToolButton::clicked, this, [=]() {

		if (ui.toolButton_calibrationDropDown->whatsThis() == UI_COLLAPSE) {
			ui.toolButton_calibrationDropDown->setWhatsThis(UI_EXPANDED);
			ui.toolButton_calibrationDropDown->setIcon(QIcon(":/24x24/Icon/24x24/cil-arrow-circle-bottom.png"));
			ui.frame_whiteBalance->show();
		}
		else {
			ui.toolButton_calibrationDropDown->setWhatsThis(UI_COLLAPSE);
			ui.toolButton_calibrationDropDown->setIcon(QIcon(":/24x24/Icon/24x24/cil-arrow-circle-right.png"));
			ui.frame_whiteBalance->hide();
		}

	});


	ui.toolButton_setAwbRegion->setWhatsThis(UI_SETUP_BTN);

	//set region logic
	auto cam_w = CAMManager::instance().getWidth(_camID);
	auto cam_h = CAMManager::instance().getHeight(_camID);
	_commonDragBox.setGeometry(QRectF(cam_w * 0.3, cam_h * 0.3, cam_w * 0.4, cam_h * 0.4));

	connect(ui.toolButton_setAwbRegion, &QToolButton::clicked, this, [=]() {
		if (ui.toolButton_setAwbRegion->whatsThis() == UI_SETUP_BTN) {
			ui.toolButton_setAwbRegion->setWhatsThis(UI_EXECUTE_BTN);
			ui.toolButton_setAwbRegion->setText("Hide Region");

			ui.toolButton_toggleFovView->animateClick();

			_commonDragBox.show();
		}
		else {
			ui.toolButton_setAwbRegion->setWhatsThis(UI_SETUP_BTN);
			ui.toolButton_setAwbRegion->setText("Set Region");
			_commonDragBox.hide();
		}
	});


	connect(ui.toolButton_learnGoldenLightingProfile, &QToolButton::clicked, this, [=]() {
		//password protected
		if (!passwordPromptCorrect()) return;
		AuditLog::instance().log(QStringLiteral("CALIB_GOLDEN_LIGHTING"));
		_stopRun = false;
		backupPortabilityFile();
		_portabilityInfo.lightingCalibrationInfo.is_main = true;
		//jogToGrayCard();
		emit calibrateGoldenLightingProfile(_camID, _commonDragBox.getGeometry());
	});
	connect(ui.toolButton_learnCurrentLightingProfile, &QToolButton::clicked, this, [=]() {
		AuditLog::instance().log(QStringLiteral("CALIB_CURRENT_LIGHTING"));
		_stopRun = false;
		backupPortabilityFile();
		_portabilityInfo.lightingCalibrationInfo.is_main = false;
		//jogToGrayCard();
		emit calibrateCurrentLightingProfile(_camID, _commonDragBox.getGeometry());
	});

	connect(ui.toolButton_sampleFromRegion, &QToolButton::clicked, this, [=]() { sampleGVFromRegion(); });
	connect(ui.toolButton_getIntensityFromExpectedGV, &QToolButton::clicked, this, [=]() { getAllIntensityFromExpectedGV(); });


	connect(ui.toolButton_assignColorSegment, &QToolButton::clicked, this, [=]() {
		auto items = ui.listWidget_recipeOptics->selectedItems();
		if (items.size()) {
			auto item = items.at(0);
			ui.lineEdit_selectedOptics->setText(item->text());
			ui.lineEdit_selectedOptics->setWhatsThis(item->whatsThis());

			auto optic = _recipeOptics[item->whatsThis()];
			showSegmentReferenceUI();
			updateSegmentComboBoxUI();
			showSegmentPriorityUI(optic.segmentPriority);

			updateCSAStatus(optic.segmentPriority);

			processEvents();

			showCSADragBox(false);
			toPage(UIPage::COLOR_SEGMENT);
		}
	});

	connect(ui.toolButton_backToOpticsPage, &QToolButton::clicked, this, [=]() {
		showCSADragBox(false);
		toPage(UIPage::LIGHTING);
	});

	connect(ui.comboBox_views, QOverload<int>::of(&QComboBox::activated), [&](int index) {
		auto optic_id = ui.lineEdit_selectedOptics->whatsThis();
		auto view_id = ui.comboBox_views->itemData(index, Qt::WhatsThisRole).toString();

		ct::logger::debug("ID: %s, %s", view_id.toStdString().c_str(), optic_id.toStdString().c_str());

		if (!_views.contains(view_id)) {
			ct::logger::error("[IM] Failed to toggle view. Invalid view ID: %s", view_id.toStdString().c_str());
			return;
		}

		auto v = _views[view_id];
		highlightViewInWorld(view_id);
		auto optic = _recipeOptics[optic_id];
		auto ipf = path::getViewPath(Common::Directory::getRecipeSetupImagePath().toStdString(), v, optic, _recipeOptics);

		auto path = ipf.getOpticPath(optic_id.toStdString());

		ct::logger::debug("Path: %s", path.c_str());

		if (QFile::exists(path.c_str())) {
			QPixmap pixmap = QPixmap(path.c_str()).scaledToWidth(500, Qt::SmoothTransformation);
			ui.label_viewReference->setPixmap(pixmap);
		}
		else {
			ui.label_viewReference->setPixmap(QPixmap());
		}
	});

	connect(ui.toolButton_showColorSegmentReference, &QToolButton::clicked, this, [=]() {
		showSegmentReferenceUI();
	});

	connect(ui.toolButton_setColorSegmentReference, &QToolButton::clicked, this, [=]() {
		auto optic_id = ui.lineEdit_selectedOptics->whatsThis();
		auto& optic = _recipeOptics[optic_id];
		
		auto index = ui.comboBox_views->currentIndex();
		auto view_id = ui.comboBox_views->itemData(index, Qt::WhatsThisRole).toString();
		_CSA.viewRef = view_id;
		
		Common::Directory::createDir(Common::Directory::getRecipeImagesPath() + "\\CSA\\");
		for (auto& optic : _recipeOptics) {
			auto src = Common::Directory::getRecipeSetupImagePath() + QString("\\%1_%2.jpg").arg(view_id).arg(optic.id);
			auto dst = Common::Directory::getRecipeImagesPath() + QString("\\CSA\\%1_segmentReference.png").arg(optic.id);
			util::copyTo(src, dst);
		}

		saveRecipeOptics();
	});

	connect(ui.toolButton_showSegmentPriority, &QToolButton::clicked, this, [=]() {
		auto optic_id = ui.lineEdit_selectedOptics->whatsThis();
		auto optic = _recipeOptics[optic_id];
		showSegmentPriorityUI(optic.segmentPriority);

		if (optic.segmentPriority == -1) {
			showMsg("No priority assigned!");
		}
	});

	connect(ui.toolButton_setSegmentPriority, &QToolButton::clicked, this, [=]() {
		auto optic_id = ui.lineEdit_selectedOptics->whatsThis();
		auto& optic = _recipeOptics[optic_id];

		auto index = ui.comboBox_colorSegment->currentIndex();
		optic.segmentPriority = index;
		updateCSAStatus(optic.segmentPriority);
		saveRecipeOptics();
	});

	connect(ui.toolButton_deprioritizeSegment, &QToolButton::clicked, this, [=]() {
		auto optic_id = ui.lineEdit_selectedOptics->whatsThis();
		auto& optic = _recipeOptics[optic_id];

		optic.segmentPriority = -1;
		updateCSAStatus(optic.segmentPriority);
		saveRecipeOptics();
	});

	connect(ui.toolButton_segmentColors, &QToolButton::clicked, this, [=]() {
		auto optic_id = ui.lineEdit_selectedOptics->whatsThis();
		auto& optic = _recipeOptics[optic_id];
		auto brightnessOffset = ui.lineEdit_brightnessOffset->text().toInt();
		auto neighbouringOffset = ui.lineEdit_hueOffset->text().toInt();

		if (!_views.contains(_CSA.viewRef)) return;

		auto v = _views[_CSA.viewRef];
		auto ipf = path::getViewPath(Common::Directory::getRecipeSetupImagePath().toStdString(), v, optic, _recipeOptics);

		optic.segmentNum = ui.lineEdit_numDistinctColors->text().toInt();

		auto path = ipf.getOpticPath(optic_id.toStdString());
		if (QFile::exists(path.c_str())) {
			
			progressBarSetup("Segmenting Colors...", 5);

			cv::Mat img = cv::imread(path);
			
			if (img.empty()) {
				progressBarRelease();
				showMsg("Reference image not found");
				return;
			}
			
			incrementProgressBar();


			std::vector<cv::Vec3b> mainColors = cvUtil::extractMainColors(img, optic.segmentNum);

			if (mainColors.empty()) {
				progressBarRelease();
				showMsg("Failed to segment colors from reference image");
				return;
			}

			incrementProgressBar();

			std::vector<std::vector<cv::Point>> segments;

			cvUtil::segmentImageByColors(img, mainColors, segments, brightnessOffset, neighbouringOffset);

			incrementProgressBar();

			auto seq = cvUtil::getAscendingSequence(segments);

			std::vector<std::vector<cv::Point>> eroded_segments;

			for (const auto& s : segments) {
				cv::Mat mask;
				cvUtil::getMaskFromSegments(s, mask, cv::Size(img.cols, img.rows));
				cvUtil::erode(mask, mask, 3);

				std::vector<cv::Point> e_segment;
				cvUtil::getSegmentFromMask(mask, e_segment);
				eroded_segments.push_back(e_segment);
			}


			cvUtil::saveImageSegments(img, eroded_segments, seq, Common::Directory::getRecipeImagesPath().toStdString() + QString("\\CSA\\%1_segment").arg(optic_id).toStdString());

			incrementProgressBar();


			optic.segmentRGBs.clear();

			int index = 1;
			ui.comboBox_colorSegment->clear();
			for (auto s : seq) {
				std::array<int, 3> rgb;
				rgb[0] = mainColors[s][2]; //Convert BGR to RGB
				rgb[1] = mainColors[s][1];
				rgb[2] = mainColors[s][0];
				optic.segmentRGBs.push_back(rgb);

				ui.comboBox_colorSegment->addItem(QString("segment%1").arg(index));
				index++;
			}

			progressBarRelease();

			saveRecipeOptics();
		}
		else {
			showMsg("Reference Image Not Found");
		}
	});

	connect(ui.comboBox_colorSegment, QOverload<int>::of(&QComboBox::activated), [&](int index) {
		showSegmentPriorityUI(index);
	});

	connect(ui.toolButton_setCSALocatorOptic, &QToolButton::clicked, this, [=]() {
		auto index = ui.comboBox_CSALocatorOptic->currentIndex();
		auto optic_id = ui.comboBox_CSALocatorOptic->itemData(index, Qt::WhatsThisRole).toString();
		_CSA.opticID = optic_id;
		saveRecipeOptics();
	});

	connect(ui.toolButton_setupCSALocator, &QToolButton::toggled, this, [=](bool state) {
		showCSADragBox(state);
		showCSALocatorReferenceUI();
	});
	
	connect(ui.toolButton_addCSASearchRegion, &QToolButton::clicked, this, [=]() {
		uidGenerator uid;
		QString id = uid.id().c_str();
		_CSA.searchLocator.append(addDragBoxToScene(_pGraphicsSceneFOV, getQRectBasedOnCam(70), Qt::green, "Search Region", id));
	});

	connect(ui.toolButton_learnCSALocator, &QToolButton::clicked, this, [=]() {

		if (_CSA.learnLocator == nullptr) return;

		auto roi = _CSA.learnLocator->getGeometry();
		
		auto setupPath = getPathToCSALocatorImage();
		auto mBuf = MbufRestoreA(setupPath.toStdString().c_str(), M_DEFAULT, M_NULL);
		auto mCrop = mtrx::crop(mBuf, roi.x(), roi.y(), roi.width(), roi.height());

		auto root = Common::Directory::getRecipeImagesPath() + "\\CSA\\";
		auto imgpath = root + "locator.png";
		auto refpath = root + "locatorView.png";

		MbufSaveA(imgpath.toStdString().c_str(), mCrop);
		MbufSaveA(refpath.toStdString().c_str(), mBuf);

		MIL_ID mMono = mtrx::to_mono(mCrop);
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

		//learn Pattern Matching Model
		input.filename = root.toStdString() + "locator.pat";
		mtrx::learn_pattern(mMono, input, output);

		mtrx::free_buffer(mBuf);
		mtrx::free_buffer(mCrop);
		mtrx::free_buffer(mMono);
	});

	connect(ui.toolButton_setCSALocator, &QToolButton::clicked, this, [=]() {

		auto root = Common::Directory::getRecipeImagesPath() + "\\CSA\\";
		auto modPath = root.toStdString() + "locator.pat";
		auto setupPath = getPathToCSALocatorImage();
		auto mBuf = MbufRestoreA(setupPath.toStdString().c_str(), M_DEFAULT, M_NULL);

		for (int i = 0; i < _CSA.searchLocator.size(); i++) {

			if (_CSA.searchLocator[i] == nullptr) continue;

			auto id = _CSA.searchLocator[i]->getId();
			auto roi = _CSA.searchLocator[i]->getGeometry();

			auto mCrop = mtrx::crop(mBuf, roi.x(), roi.y(), roi.width(), roi.height());
			MIL_ID mMono = mtrx::to_mono(mCrop);

			MbufSaveA("test.jpg", mMono);

			mtrx::PatternOutput output;
			output.acceptance_min_score = 50;
			output.certainty_min_score = 80;

			mtrx::find_pattern(mMono, modPath, output);

			printf("output: %f, %f, %f, %f, score:%f center: %f, %f\n", output.x, output.y, output.w, output.h, output.score, output.cx, output.cy);
			
			if (output.score > 50) {
				drawCross(_pGraphicsSceneFOV, "locator", QRectF(roi.x() + output.cx, roi.y() + output.cy, output.w, output.h), Qt::yellow);
				_CSA.teachPoints.insert(id, QPointF(roi.x() + output.cx, roi.y() + output.cy));
			}

			mtrx::free_buffer(mMono);
			mtrx::free_buffer(mCrop);
		}

		mtrx::free_buffer(mBuf);
		saveRecipeOptics();
	});
}

void VisionApp::updateOpticComboBoxUI()
{
	ui.comboBox_CSALocatorOptic->clear();

	auto keys = _recipeOptics.keys();
	qSort(keys);

	for (auto& key : keys) {
		auto& v = _recipeOptics[key];
		ui.comboBox_CSALocatorOptic->addItem(v.name);
		int index = ui.comboBox_CSALocatorOptic->count() - 1;
		ui.comboBox_CSALocatorOptic->setItemData(index, v.id, Qt::WhatsThisRole);
	}
}

void VisionApp::updateViewComboBoxUI()
{
	ui.comboBox_views->clear();

	auto keys = _views.keys();
	qSort(keys);

	for (auto& key : keys) {
		auto& v = _views[key];

		if (v.type == ct::s_child_view) continue;

		ui.comboBox_views->addItem(v.name);
		int index = ui.comboBox_views->count() - 1;
		ui.comboBox_views->setItemData(index, v.id, Qt::WhatsThisRole);
	}
}

void VisionApp::updateSegmentComboBoxUI()
{
	ui.comboBox_colorSegment->clear();
	auto optic_id = ui.lineEdit_selectedOptics->whatsThis();
	auto optic = _recipeOptics[optic_id];

	int index = 1;
	for (auto& s : optic.segmentRGBs) {
		ui.comboBox_colorSegment->addItem(QString("segment%1").arg(index));
		index++;
	}
}

void VisionApp::showSegmentReferenceUI()
{
	auto optic_id = ui.lineEdit_selectedOptics->whatsThis();
	auto optic = _recipeOptics[optic_id];

	if (!_views.contains(_CSA.viewRef)) {
		ui.label_viewReference->setPixmap(QPixmap());
		return;
	}

	auto v = _views[_CSA.viewRef];
	ui.comboBox_views->setCurrentText(v.name);
	highlightViewInWorld(v.id);

	auto ipf = path::getViewPath(Common::Directory::getRecipeSetupImagePath().toStdString(), v, optic, _recipeOptics);

	auto path = ipf.getOpticPath(optic_id.toStdString());
	if (QFile::exists(path.c_str())) {
		QPixmap pixmap = QPixmap(path.c_str()).scaledToWidth(500, Qt::SmoothTransformation);
		ui.label_viewReference->setPixmap(pixmap);
	}
	else {
		ui.label_viewReference->setPixmap(QPixmap());
	}
}

void VisionApp::showCSALocatorReferenceUI()
{
	auto path = getPathToCSALocatorImage();

	if (QFile::exists(path)) {
		_imageFOV.load(path);
		displayFOV(_imageFOV);
	}
}

int VisionApp::getNumOfSingleViewstoProcess()
{
	int viewIndex = 0;

	if (g_viewMode == int(ViewMode::SINGLE))
	{
		for (const auto& v : _views) {
			if (v.type == ct::s_stitch_view) continue;
			if (v.id == "") continue;
			viewIndex = viewIndex + _unitConfigTab->getTotalIndex(v.id);
		}
	}
	else
	{
		for (const auto& v : _views) {
			if (v.type == ct::s_stitch_view) continue;
			if (v.id == "") continue;
			viewIndex++;
		}
	}
		
	ct::logger::info("Number of single View to process: %d", viewIndex);
	return viewIndex;
}

int VisionApp::getNumOfViewToProcess(QStringList datasetIndexIds)
{
	int viewIndex = 0;

	if (g_viewMode == int(ViewMode::SINGLE))
	{
		for (const auto& v : _views) {
			if (v.type == ct::s_stitch_view) continue;
			if (v.id == "") continue;

			int size = _unitConfigTab->getTotalIndex(v.id);

			if (datasetIndexIds.size() > 0)
			{
				QStringList idList = _unitConfigTab->getIDList(v.id);
				QStringList commonList;
				for (const QString& item : datasetIndexIds) {
					if (idList.contains(item)) {
						commonList.append(item);
					}
				}

				size = commonList.size();
			}
			

			viewIndex = viewIndex + size;
		}
	}
	else
	{
		for (const auto& v : _views) {
			if (v.type == ct::s_child_view) continue;
			if (v.id == "") continue;

			if (datasetIndexIds.size() > 0)
			{
				bool voExistInView = false;
				for (const QString& voID : datasetIndexIds) {
					auto vo = _visionObject[voID];
					if (vo.viewID == v.id)
					{
						voExistInView = true;
						break;
					}
				}
				if (!voExistInView) continue;
			}
			
			viewIndex++;
		}
	}
	
	ct::logger::info("Number of view to process: %d", viewIndex);
	return viewIndex;
}

int VisionApp::getNumOfLineScanToProcess(QStringList datasetIndexIds)
{
	int viewIndex = 0;

	for (const auto& v : _lineScans) {
		if (v.type == ct::s_child_linescan) continue;
		if (v.id == "") continue;

		if (datasetIndexIds.size() > 0)
		{
			bool voExistInView = false;
			for (const QString& voID : datasetIndexIds) {
				auto vo = _visionObject[voID];
				if (vo.lineScanID == v.id)
				{
					voExistInView = true;
					break;
				}
			}
			if (!voExistInView) continue;
		}

		viewIndex++;
	}
	ct::logger::info("Number of lineScans to process: %d", viewIndex);
	return viewIndex;
}

QString VisionApp::getPathToCSALocatorImage()
{
	if (_CSA.opticID.isEmpty()) {
		showMsg("Locator's Optic not set!");
		return "";
	}

	auto optic = _recipeOptics[_CSA.opticID];

	if (!_views.contains(_CSA.viewRef)) {
		return "";
	}

	auto v = _views[_CSA.viewRef];

	auto ipf = path::getViewPath(Common::Directory::getRecipeSetupImagePath().toStdString(), v, optic, _recipeOptics);

	QString path = ipf.getOpticPath(optic.id.toStdString()).c_str();

	return path;
}

void VisionApp::showSegmentPriorityUI(int index)
{
	auto optic_id = ui.lineEdit_selectedOptics->whatsThis();
	auto optic = _recipeOptics[optic_id];

	auto path = Common::Directory::getRecipeImagesPath() + QString("\\CSA\\%1_segment%2.png").arg(optic_id).arg(index + 1);

	if (QFile::exists(path)) {
		QPixmap pixmap = QPixmap(path).scaledToWidth(500, Qt::SmoothTransformation);
		ui.label_colorSegment->setPixmap(pixmap);

		QSignalBlocker block(ui.comboBox_colorSegment);
		ui.comboBox_colorSegment->setCurrentIndex(index);

		if (optic.segmentRGBs.size() > index) {
			auto& rgb = optic.segmentRGBs[index];

			ct::logger::debug("Segment RGB GV: %d, %d, %d", rgb[0], rgb[1], rgb[2]);
			ui.label_colorSegmentPalette->setStyleSheet(QString("background-color : rgb(%1, %2, %3);")
				.arg(rgb[0]).arg(rgb[1]).arg(rgb[2]));
		}
	}
	else {
		ui.label_colorSegmentPalette->setStyleSheet("");
		ui.label_colorSegment->setPixmap(QPixmap());
	}
}

void VisionApp::updateCSAStatus(int priority)
{
	if (priority != -1) {
		ui.status_CSA->setStyleSheet("background-color: green;");
	}
	else {
		ui.status_CSA->setStyleSheet("background-color: red;");
	}
}

void VisionApp::showCSADragBox(bool show)
{
	ui.toolButton_addCSASearchRegion->setEnabled(show);
	ui.toolButton_learnCSALocator->setEnabled(show);
	ui.toolButton_setCSALocator->setEnabled(show);

	if (_CSA.learnLocator) _CSA.learnLocator->setVisible(show);
	auto pos = _CSA.learnLocator->getGeometry();
	ct::logger::debug("Pos: %f, %f, %f, %f", pos.x(), pos.y(), pos.width(), pos.height());

	for (int i = 0; i < _CSA.searchLocator.size(); i++) {
		auto p = _CSA.searchLocator[i];
		if (p) p->setVisible(show);
	}
}

bool VisionApp::isTypeRGB(const OpticsInfo& opt)
{
	return false;
}

void VisionApp::assignOpticBasedOnLightingType(OpticsInfo & opt)
{
	auto optic = ui.comboBox_lightingType->currentText();

	if (optic.contains("RGB")) {
		opt.R.clear();
		opt.G.clear();
		opt.B.clear();

		for (const auto lineEdit : _channelLineEdits) {
			auto id = lineEdit->whatsThis();
			auto intensity = lineEdit->text().toInt();

			auto channel = LSCManager::instance().channels()[id];
			
			opt.R.insert(id, 0);
			opt.G.insert(id, 0);
			opt.B.insert(id, 0);

			if (channel.optic == optic) {
				if (channel.lighting_type == "R") opt.R.insert(id, intensity);
				else if (channel.lighting_type == "G") opt.G.insert(id, intensity);
				else if (channel.lighting_type == "B") opt.B.insert(id, intensity);
			}
		}
	}
	else {
		opt.M.clear();

		for (const auto lineEdit : _channelLineEdits) {
			auto id = lineEdit->whatsThis();
			auto intensity = lineEdit->text().toInt();

			auto channel = LSCManager::instance().channels()[id];

			//if (channel.optic == optic) {
				opt.M.insert(id, intensity);
			//}
		}
	}
}

void VisionApp::updateOpticBasedOnLightingType(const OpticsInfo & opt)
{
	auto optic = ui.comboBox_lightingType->currentText();
	
	if (optic.contains("RGB")) {
		for (const auto& key : opt.R.keys()) {
			auto channel = LSCManager::instance().channels()[key];
			
			if (channel.optic == optic) {
				if (channel.lighting_type == "R" && opt.R.contains(key))  OpticsControl::instance().setIntensity(key, opt.R[key]);
			}
		}

		for (const auto& key : opt.G.keys()) {
			auto channel = LSCManager::instance().channels()[key];
			
			if (channel.optic == optic) {
				if (channel.lighting_type == "G" && opt.G.contains(key))  OpticsControl::instance().setIntensity(key, opt.G[key]);
			}
		}

		for (const auto& key : opt.B.keys()) {
			auto channel = LSCManager::instance().channels()[key];
			
			if (channel.optic == optic) {
				if (channel.lighting_type == "B" && opt.B.contains(key))  OpticsControl::instance().setIntensity(key, opt.B[key]);
			}
			
		}
	}
	else {
		for (const auto& key : opt.M.keys()) {
			if (opt.M.contains(key)) OpticsControl::instance().setIntensity(key, opt.M[key]);
		}
	}
}

void VisionApp::updateOpticBandUI(const QHash<QString, int>& band, QLineEdit * lineEdit)
{
	QString s;

	for (const auto& key : LSCManager::instance().channels()) {
		if (!band.contains(key)) continue;
		s += QString::number(band[key]);
		s += ", ";
	}
	
	lineEdit->setText(s);
}

void VisionApp::addDefaultOptic()
{
	if (_recipeOptics.size() == 0) {

		_recipeOptics.clear();

		//HARDCODE:
		OpticsInfo optic;
		optic.id = "RGB";
		optic.name = optic.id;
		optic.camID = _camID;
		optic.exposure = 10000;
		optic.gain = 1;
		optic.type = ct::s_color;

		for (auto chID : LSCManager::instance().channels()) {
			optic.R.insert(chID, 0);
			optic.G.insert(chID, 0);
			optic.B.insert(chID, 0);
		}

		optic.R.insert("Coaxial Blue Btm", 160);
		optic.R.insert("Coaxial Red", 10);
		optic.R.insert("Coaxial Red Btm", 8);
		optic.G.insert("Coaxial Blue Btm", 160);
		optic.G.insert("Coaxial Red", 10);
		optic.B.insert("Coaxial Blue Btm", 160);
		optic.B.insert("Coaxial Red", 10);
		_recipeOptics.insert(optic.id, optic);

		////Create default optics
		//QSet<QString> uniqueOptics;
		//for (auto chID : LSCManager::instance().channels()) {
		//	auto& channel = LSCManager::instance().channel(chID);

		//	if (uniqueOptics.contains(channel.optic)) continue;
		//	uniqueOptics.insert(channel.optic);

		//	QString id, camID;
		//	id = channel.optic;
		//	camID = channel.camID;

		//	OpticsInfo optic;
		//	optic.id = id;
		//	optic.name = id;
		//	optic.camID = camID;

		//	if (camID == "cam1") {
		//		optic.exposure = 10000; //TODO-610: Auto assign exposure, now need manual decide
		//		optic.gain = 1;
		//	}
		//	else {
		//		optic.exposure = 10000;
		//		optic.gain = 10;
		//	}

		//	optic.type = (CAMManager::instance().getChannel(camID) == 3) ? ct::s_color : ct::s_mono;

		//	for (auto chID2 : LSCManager::instance().channels()) {
		//		auto& channel2 = LSCManager::instance().channel(chID2);

		//		if (channel2.optic == id) {
		//			optic.M.insert(channel2.id, 128);
		//		}
		//		else {
		//			optic.M.insert(channel2.id, 0);
		//		}
		//	}

		//	_recipeOptics.insert(id, optic);
		//}
	}

	assignMainOptics();

	if (_recipeOptics3D.size() == 0) {
		_recipeOptics3D.clear();

		//give default if none found
		OpticsInfo3D opt30;
		opt30.id = "E30";
		opt30.name = "E30";
		opt30.exposure = 30.0;
		opt30.intensity = true;

		OpticsInfo3D opt375;
		opt375.id = "E300";
		opt375.name = "E300";
		opt375.exposure = 300;
		opt375.intensity = false;

		_recipeOptics3D.insert(opt30.id, opt30);
		_recipeOptics3D.insert(opt375.id, opt375);
	}
}

void VisionApp::assignMainOptics()
{
	_mainOptics.clear();

	auto keys = _recipeOptics.keys();
	qSort(keys);

	for (auto& camID : CAMManager::instance().cameras().keys()) {
		for (auto key : keys) {
			auto& optic = _recipeOptics[key];

			if (optic.camID == camID) {
				if (!_mainOptics.contains(camID)) {
					_mainOptics.insert(camID, optic);
					optic.tag = "Main";
				}
			}
		}
	}
}


bool VisionApp::updateAllChannels()
{
	if (_mainOptics[_camID].type == ct::s_color && CAMManager::instance().getChannel(_camID) == 1) {
		updateOpticBasedOnLightingType(_mainOptics[_camID]);
	}
	else {
		OpticsControl::instance().setAllChannels(_camID, _mainOptics[_camID].M);
	}

	return true;
}

bool VisionApp::validChannel(QString ch)
{
	return LSCManager::instance().isChannelValid(ch);
}

void VisionApp::printBand(const QString title, const ct::Band& band)
{
	std::string msg = "";

	for (const auto& b : band) {
		msg += std::to_string(b) + ",";
	}

	ct::logger::debug("%s: %s", title.toStdString().c_str(), msg.c_str());
}

void VisionApp::sampleGVFromRegion()
{
	auto mBuf = mtrx::to_milID(_imageFOV);
	mtrx::BufferCollector bc_mBuf(mBuf);

	auto w = mtrx::get_width(mBuf);
	auto h = mtrx::get_height(mBuf);

	int start_x = _commonDragBox.getGeometry().x();
	int start_y = _commonDragBox.getGeometry().y();
	int end_x = _commonDragBox.getGeometry().width() + start_x;
	int end_y = _commonDragBox.getGeometry().height() + start_y;

	int region_w = _commonDragBox.getGeometry().width();
	int region_h = _commonDragBox.getGeometry().height();

	if (mtrx::is_color(mBuf)) {
		auto mR = MbufChildColor(mBuf, M_RED, M_NULL);
		auto mG = MbufChildColor(mBuf, M_GREEN, M_NULL);
		auto mB = MbufChildColor(mBuf, M_BLUE, M_NULL);
		mtrx::BufferCollector bc_mR(mR);
		mtrx::BufferCollector bc_mG(mG);
		mtrx::BufferCollector bc_mB(mB);

		double avgR = 0.0, avgG = 0.0, avgB = 0.0;
		avgR = mtrx::get_mean(mR, start_x, start_y, region_w, region_h);
		avgG = mtrx::get_mean(mG, start_x, start_y, region_w, region_h);
		avgB = mtrx::get_mean(mB, start_x, start_y, region_w, region_h);

		ui.lineEdit_expectedCH1->setText(QString::number(avgR));
		ui.lineEdit_expectedCH2->setText(QString::number(avgG));
		ui.lineEdit_expectedCH3->setText(QString::number(avgB));
	}
	else {
		auto avg = mtrx::get_mean(mBuf, start_x, start_y, region_w, region_h);
		ui.lineEdit_expectedCH1->setText(QString::number(avg));
		ui.lineEdit_expectedCH2->setText(QString::number(avg));
		ui.lineEdit_expectedCH3->setText(QString::number(avg));
	}
}

void VisionApp::getAllIntensityFromExpectedGV()
{
	auto idealGV_R = ui.lineEdit_expectedCH1->text().toDouble();
	auto idealGV_G = ui.lineEdit_expectedCH2->text().toDouble();
	auto idealGV_B = ui.lineEdit_expectedCH3->text().toDouble();

	int intensity_R = 0, intensity_G = 0, intensity_B = 0;

	auto opticType = ui.comboBox_lightingType->currentText();

	emit signalGetAllIntensityFromExpectedGV(_camID, opticType, idealGV_R, idealGV_G, idealGV_B, _commonDragBox.getGeometry());
}

QListWidgetItem* VisionApp::getViewOpticListItem(QString id)
{
	for (int i = 0; i < ui.listWidget_viewOpticSelection->count(); i++) {
		auto item = ui.listWidget_viewOpticSelection->item(i);
		if (item->whatsThis() == id) {
			return item;
		}
	}
	return nullptr;
}

void VisionApp::jogToGrayCard()
{
	auto point = _portabilityInfo.lightingCalibrationInfo.graycard_point;
	
}

int VisionApp::getMaxIntensityOfBand(const OpticsInfo& optic, BandType bandType)
{
	int intensity = 0;

	auto band = OpticsControl::instance().getBand(optic, bandType);
	for (const auto& b : band) {
		if (b > intensity) intensity = b;
	}

	return intensity;
}

int VisionApp::getAverageBasedOnSegment(const MIL_ID& mMono, const std::vector<cv::Point>& segment)
{
	if (mMono == M_NULL) return 0;

	MIL_UINT8* hostPtr = M_NULL;
	MIL_ID pitch = M_NULL;
	MbufInquire(mMono, M_HOST_ADDRESS, &hostPtr);
	MbufInquire(mMono, M_PITCH, &pitch);

	auto w = mtrx::get_width(mMono);
	auto h = mtrx::get_height(mMono);

	int avg = 0;
	int count = 0;

	for (auto& pos : segment) {
		if (pos.x < 0 || pos.x >= w || pos.y < 0 || pos.y >= h) {
			ct::logger::error("Invalid point access when getting average based on segment");
			continue;
		}

		avg += hostPtr[pos.x + (pos.y * pitch)];
		count++;
	}

	avg /= count;

	return avg;
}

int VisionApp::getAverageBasedOnInlierSegment(const MIL_ID& mMono, const std::vector<cv::Point>& segment)
{
	if (mMono == M_NULL) return 0;

	MIL_UINT8* hostPtr = M_NULL;
	MIL_ID pitch = M_NULL;
	MbufInquire(mMono, M_HOST_ADDRESS, &hostPtr);
	MbufInquire(mMono, M_PITCH, &pitch);

	auto w = mtrx::get_width(mMono);
	auto h = mtrx::get_height(mMono);

	std::vector <int> inliers;

	for (auto& pos : segment) {

		int x = pos.x;
		int y = pos.y;

		if (x < 0 || x >= w || y < 0 || y >= h) {
			//ct::logger::error("Invalid point access when getting average based on segment");
			continue;
		}

		inliers.push_back(hostPtr[x + (y * pitch)]);
	}

	util::removeOutliers(inliers);
	int avg = 0;

	for (const auto& i : inliers) {
		avg += i;
	}

	avg /= inliers.size();

	return avg;
}

void VisionApp::generateGVTableReport()
{
	const auto& mgvt = _portabilityInfo.lightingCalibrationInfo.main_GVTable;
	const auto& lgvt = _portabilityInfo.lightingCalibrationInfo.local_GVTable;

	if (mgvt.size() != lgvt.size()) {
		showMsg("Master and Local Channel not match.");
		return;
	}

	//ch | master | local | offset
	struct IntensityGV {
		int master_intensity = 0;
		int local_intensity = 0;
		double gv = 0.0;
	};

	QHash<QString, IntensityGV> max_differences;
	QHash<QString, double> average_differences;
	QHash<QString, double> similarities; //

	for (const auto& channel : LSCManager::instance().channels().keys()) {

		IntensityGV max_dif;
		double avg_dif = 0.0, similarity = 0.0;

		for (int mi = 0; mi < mgvt[channel].size(); mi++) {

			const auto& mgv = mgvt[channel][mi];

			int master_intensity = 0;
			int local_intensity = 0;
			double min_dif = 999.9;

			for (int li = 0; li < lgvt[channel].size(); li++) {
				
				const auto& lgv = lgvt[channel][li];

				auto dif = abs(mgv - lgv);


				if (dif < min_dif) {
					master_intensity = mi;
					local_intensity = li;
					min_dif = dif;
				}
			}

			avg_dif += min_dif;

			if (min_dif > max_dif.gv) {
				ct::logger::debug("%d - %d", mi, local_intensity);
				ct::logger::debug("%f - %f = %f", mgv, lgvt[channel][local_intensity], min_dif);

				max_dif.gv = min_dif;
				max_dif.master_intensity = master_intensity;
				max_dif.local_intensity = local_intensity;
			}
		}

		avg_dif /= mgvt[channel].size();
		average_differences.insert(channel, avg_dif);
		max_differences.insert(channel, max_dif);
	}

	for (const auto& key : max_differences.keys()) {
		const auto& m = max_differences[key];
		int offset = abs(m.master_intensity - m.local_intensity);
		ct::logger::info("Channel: %s", key.toStdString().c_str());
		ct::logger::info("Average difference: %f", average_differences[key]);
		ct::logger::info("Max offset (%d -> %d): %d", m.master_intensity, m.local_intensity, offset);
		ct::logger::info("Master Channel intensity: %d -> RGB intensity: %f", m.master_intensity, mgvt[key][m.master_intensity]);
		ct::logger::info("Local Channel intensity: %d -> RGB intensity: %f", m.local_intensity, lgvt[key][m.local_intensity]);
		ct::logger::info("Max difference: %f", m.gv);
	}
}

void VisionApp::backupPortabilityFile()
{
	auto srcPath = QStringLiteral("%1portability.json").arg(Common::Directory::ConfigPath());
	auto dstFolder = Common::Directory::ConfigPath() + "backup";
	auto dstPath = QStringLiteral("%1/portability.json").arg(dstFolder);

	util::createFolder(dstFolder);
	util::copyTo(srcPath, dstPath);
	util::renameFileWithTimestamp(dstPath);
}
