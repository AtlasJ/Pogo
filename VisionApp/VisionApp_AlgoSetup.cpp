// =============================================================================
//  VisionApp_AlgoSetup.cpp
//  Algo Setup page: OCR inspection (PaddleOCR + pattern matching, ported from
//  IM430 ocrInspection2 core) and 3D height measurement (plane-fit datum,
//  following Algo QAlgoHeightMeasurement), with an optional locator per algo.
//
//  All processing runs on AlgoManager's worker thread; this file is UI glue:
//  ROI drag boxes, parameter round-tripping, overlay rendering, pattern
//  library management.
// =============================================================================

#include "VisionApp.h"
#include "AlgoManager.h"
#include "AuditLog.h"

#include <QFileDialog>
#include <QMessageBox>
#include <QTableWidgetItem>
#include <QHeaderView>
#include <QPushButton>

static const QColor kAlgoRoiColor(0, 150, 255);
static const QColor kAlgoLearnColor(66, 135, 245);
static const QColor kAlgoPlaneColor(255, 165, 0);   //plane ROIs: orange
static const QColor kAlgoHeightColor(0, 200, 0);    //height ROIs: green
static const QColor kAlgoSearchColor(255, 165, 0);

// ── setup ────────────────────────────────────────────────────────────────────

void VisionApp::initAlgoSetupPage()
{
	//── create the ROI drag boxes (hidden until the page + matching algo is active)
	auto makeBox = [=](const QRectF& rect, const QColor& color, const QString& name) -> QDragBox* {
		auto box = new QDragBox();
		_pGraphicsSceneFOV->addItem(box);
		box->setOutterBarrier(_pGraphicsSceneFOV->sceneRect());
		box->setup(rect, color, name);
		box->setDragable(true);
		box->setZValue((int)UIHierarchy::DRAGGABLES);
		box->hide();
		connect(box, SIGNAL(dragBoxMouseReleased(QDragBox*, QString, QPointF)), this, SLOT(algoSettingsTouched()));
		connect(box, SIGNAL(grabberReleased(QDragBox*)), this, SLOT(algoSettingsTouched()));
		return box;
	};

	_algoOcrRoi1Box = makeBox(QRectF(100, 100, 400, 150), kAlgoRoiColor, "OCR ROI 1");
	_algoOcrLearnBox = makeBox(QRectF(100, 100, 60, 60), kAlgoLearnColor, "Learn Char");
	_algoLocLearnBox = makeBox(QRectF(50, 50, 150, 150), kAlgoLearnColor, "Locator Learn");
	_algoLocSearchBox = makeBox(QRectF(0, 0, 800, 600), kAlgoSearchColor, "Locator Search");

	//── header: algo selection + run
	connect(ui.comboBox_algoType, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [=](int index) {
		ui.stackedWidget_algoParams->setCurrentIndex(index);
		refreshAlgoLocatorUI();
		updateAlgoRoiVisibility();
	});

	connect(ui.toolButton_algoRun, &QToolButton::clicked, this, [=]() {
		if (AlgoManager::instance().isBusy()) {
			showMsg("Algo is still running, please wait.");
			return;
		}

		captureAlgoParamsFromUI();
		clearAlgoOverlay();
		ui.label_algoStatus->setText("Running...");

		if (currentAlgoPageAlgo() == AlgoPageAlgo::OCR_READ) {
			AlgoManager::instance().runOcr(_imageFOV);
			AuditLog::instance().log(QStringLiteral("ALGO_RUN_OCR"));
		}
		else {
			AlgoManager::instance().runHeight();
			AuditLog::instance().log(QStringLiteral("ALGO_RUN_HEIGHT"));
		}
	});

	//── OCR page controls
	connect(ui.toolButton_algoOcrRoi1, &QToolButton::toggled, this, [=](bool) { updateAlgoRoiVisibility(); });

	/*
	* Reference IMAGE: "Save as Ref" stores the current image (OCR: the FOV image,
	* 3D: the loaded height map) as a machine-level reference; "Load Ref" brings it
	* back into the display. Settings are not part of the reference.
	*/
	const QString ocrRefPath = QStringLiteral("%1/algoRef_ocr.png").arg(Common::Directory::ConfigPath());
	const QString hRefPath = QStringLiteral("%1/algoRef_height.tif").arg(Common::Directory::ConfigPath());

	auto refreshRefButtons = [=]() {
		ui.toolButton_algoOcrLoadRef->setEnabled(QFile::exists(ocrRefPath));
		ui.toolButton_algoHLoadRef->setEnabled(QFile::exists(hRefPath));
	};

	connect(ui.toolButton_algoOcrSaveRef, &QToolButton::clicked, this, [=]() {
		if (_imageFOV.isNull()) { showMsg("No image loaded to save as reference."); return; }
		if (_imageFOV.save(ocrRefPath)) {
			showStatus("OCR reference image saved");
			AuditLog::instance().log(QStringLiteral("ALGO_REF_SAVE"), QStringLiteral("ocr"));
		}
		else showMsg("Failed to save the reference image.");
		refreshRefButtons();
	});

	connect(ui.toolButton_algoOcrLoadRef, &QToolButton::clicked, this, [=]() {
		QImage img(ocrRefPath);
		if (img.isNull()) { showMsg("No reference image saved yet."); refreshRefButtons(); return; }
		_imageFOV = img;
		displayFOV(_imageFOV);
		ui.graphicsViewFOV->fitInView(_pPixmapItemFOV, Qt::KeepAspectRatio);
		showStatus("OCR reference image loaded");
		AuditLog::instance().log(QStringLiteral("ALGO_REF_LOAD"), QStringLiteral("ocr"));
	});

	connect(ui.toolButton_algoHSaveRef, &QToolButton::clicked, this, [=]() {
		auto hm = AlgoManager::instance().heightMap();
		if (!hm || hm->id() == M_NULL) { showMsg("No height map loaded to save as reference."); return; }
		MbufExportA(hRefPath.toStdString().c_str(), M_TIFF, hm->id());
		showStatus("3D reference height map saved");
		AuditLog::instance().log(QStringLiteral("ALGO_REF_SAVE"), QStringLiteral("height"));
		refreshRefButtons();
	});

	connect(ui.toolButton_algoHLoadRef, &QToolButton::clicked, this, [=]() {
		if (!QFile::exists(hRefPath)) { showMsg("No reference height map saved yet."); refreshRefButtons(); return; }
		MIL_ID hm = M_NULL;
		MbufImportA(hRefPath.toStdString().c_str(), M_TIFF, M_RESTORE, M_DEFAULT_HOST, &hm);
		if (hm == M_NULL) { showMsg("Failed to load the reference height map."); return; }
		AlgoManager::instance().setHeightMap(mtrx::MPM::instance().attach(hm));
		showAlgoHeightMap(_algoHeightView3D);
		showStatus("3D reference height map loaded");
		AuditLog::instance().log(QStringLiteral("ALGO_REF_LOAD"), QStringLiteral("height"));
	});

	refreshRefButtons();

	connect(ui.toolButton_algoOcrLearnRoi, &QToolButton::toggled, this, [=](bool) { updateAlgoRoiVisibility(); });

	connect(ui.toolButton_algoOcrLearnSample, &QToolButton::clicked, this, [=]() {
		QString error;
		const QString label = ui.lineEdit_algoOcrLearnLabel->text().trimmed().toUpper();
		if (!AlgoManager::instance().learnPatternSample(_imageFOV, _algoOcrLearnBox->getGeometry(), label, error)) {
			showMsg(error);
			return;
		}
		showStatus(QStringLiteral("Learned sample for '%1'").arg(label));
	});

	connect(ui.checkBox_algoOcrPatternEnable, &QCheckBox::toggled, this, [=](bool on) {
		AlgoManager::instance().setPatternEnabled(on);
	});
	connect(ui.dspin_algoOcrPatternScore, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [=](double v) {
		AlgoManager::instance().setPatternThreshold(v);
	});

	connect(ui.tableWidget_algoOcrPatternLabels, &QTableWidget::itemChanged, this, [=](QTableWidgetItem* item) {
		if (!item || item->column() != 2) return;
		const auto labelItem = ui.tableWidget_algoOcrPatternLabels->item(item->row(), 0);
		if (!labelItem) return;
		AlgoManager::instance().setPatternLabelEnabled(labelItem->text(), item->checkState() == Qt::Checked);
	});

	//── height page controls
	connect(ui.toolButton_algoH2D, &QToolButton::clicked, this, [=]() { showAlgoHeightMap(false); });
	connect(ui.toolButton_algoH3D, &QToolButton::clicked, this, [=]() { showAlgoHeightMap(true); });

	connect(ui.toolButton_algoHLoadMap, &QToolButton::clicked, this, [=]() {
		const QString path = QFileDialog::getOpenFileName(this, "Load Heightmap",
			Common::Directory::getRecipeCurrentPath(), "Heightmap (*.tiff *.tif)");
		if (path.isEmpty()) return;

		QString error;
		if (!AlgoManager::instance().loadHeightMapFromFile(path, error)) {
			showMsg(error);
			return;
		}
		showAlgoHeightMap(_algoHeightView3D);
	});

	connect(ui.toolButton_algoHUseLast, &QToolButton::clicked, this, [=]() {
		if (!AlgoManager::instance().heightMap()) {
			showMsg("No heightmap captured yet - run a 3D scan first.");
			return;
		}
		showAlgoHeightMap(_algoHeightView3D);
	});

	//rotate the loaded heightmap by the set angle - the map itself rotates, so taught
	//ROIs and the algo stay in the same (rotated) coordinate space. Exact multiples of
	//90 use the lossless transpose path (and resize the canvas); other angles warp
	//within the same canvas, nearest-neighbour so heights are never blended.
	auto rotateHeightmap = [=](bool clockwise) {
		auto hm = AlgoManager::instance().heightMap();
		if (!hm) { showMsg("No heightmap loaded."); return; }

		const double angle = ui.dspin_algoHRotAngle->value();

		cv::Mat src, dst;
		util::Mil_to_cv(hm->id(), src);
		if (src.empty()) { showMsg("Failed to read the heightmap."); return; }

		if (std::fmod(angle, 90.0) == 0.0) {
			int quarters = (int)(angle / 90.0) % 4;
			if (!clockwise) quarters = (4 - quarters) % 4;
			dst = src;
			for (int i = 0; i < quarters; i++) {
				cv::Mat tmp;
				cv::rotate(dst, tmp, cv::ROTATE_90_CLOCKWISE);
				dst = tmp;
			}
		}
		else {
			//OpenCV: positive angle = counter-clockwise
			const double cvAngle = clockwise ? -angle : angle;
			const cv::Point2f center(src.cols / 2.0f, src.rows / 2.0f);
			const cv::Mat m = cv::getRotationMatrix2D(center, cvAngle, 1.0);
			cv::warpAffine(src, dst, m, src.size(), cv::INTER_NEAREST,
				cv::BORDER_CONSTANT, cv::Scalar(0)); //0 = invalid height
		}

		MIL_ID rotated = M_NULL;
		util::cv_to_Mil(dst, rotated);
		if (rotated == M_NULL) { showMsg("Rotate failed."); return; }

		AlgoManager::instance().setHeightMap(mtrx::MPM::instance().attach(rotated));
		showAlgoHeightMap(_algoHeightView3D);
	};
	connect(ui.toolButton_algoHRotCW, &QToolButton::clicked, this, [=]() { rotateHeightmap(true); });
	connect(ui.toolButton_algoHRotCCW, &QToolButton::clicked, this, [=]() { rotateHeightmap(false); });

	//── ROI tools: duplicate at pitch, selection diff, copy/paste ──
	auto selected3DBoxes = [=]() {
		QVector<QPair<bool, QDragBox*>> sel; //isPlane, box
		for (auto b : _algoPlaneBoxes) if (b->getSelected()) sel.append({ true, b });
		for (auto b : _algoHeightBoxes) if (b->getSelected()) sel.append({ false, b });
		return sel;
	};

	auto duplicateSelected = [=](bool horizontal) {
		const auto sel = selected3DBoxes();
		if (sel.size() != 1) { showMsg("Select exactly one ROI to duplicate."); return; }

		const bool isPlane = sel[0].first;
		const QRectF r = sel[0].second->getGeometry();
		const int count = ui.spinBox_algoHDupCount->value();
		const double step = horizontal ? ui.spinBox_algoHPitchX->value() : ui.spinBox_algoHPitchY->value();

		for (int i = 1; i <= count; i++) {
			addAlgoHRoiBox(isPlane, r.translated(horizontal ? step * i : 0.0, horizontal ? 0.0 : step * i));
		}
	};
	connect(ui.toolButton_algoHDupH, &QToolButton::clicked, this, [=]() { duplicateSelected(true); });
	connect(ui.toolButton_algoHDupV, &QToolButton::clicked, this, [=]() { duplicateSelected(false); });

	//selection watcher: enables Duplicate for exactly one selected ROI, and shows the
	//center-to-center X/Y difference when exactly two are selected
	auto* selTimer = new QTimer(this);
	connect(selTimer, &QTimer::timeout, this, [=]() {
		if (!isPage(UIPage::ALGO_SETUP)) return;

		const auto sel = selected3DBoxes();
		ui.toolButton_algoHDupH->setEnabled(sel.size() == 1);
		ui.toolButton_algoHDupV->setEnabled(sel.size() == 1);

		if (sel.size() == 2) {
			const QPointF c1 = sel[0].second->getGeometry().center();
			const QPointF c2 = sel[1].second->getGeometry().center();
			ui.label_algoHDiff->setText(QStringLiteral("Diff X: %1 px   Y: %2 px")
				.arg(std::abs(c2.x() - c1.x()), 0, 'f', 1)
				.arg(std::abs(c2.y() - c1.y()), 0, 'f', 1));
		}
		else {
			ui.label_algoHDiff->setText(QStringLiteral("Diff: select two ROIs"));
		}
	});
	selTimer->start(250);

	//Ctrl+C / Ctrl+V are handled in the global event filter (see VisionApp::eventFilter):
	//shortcuts scoped to the FOV view need it focused, which the operator rarely does

	connect(ui.toolButton_algoHAddPlane, &QToolButton::clicked, this, [=]() {
		const int n = _algoPlaneBoxes.size();
		auto box = makeBox(QRectF(80 + n * 60, 80 + n * 60, 120, 120), kAlgoPlaneColor,
			QStringLiteral("Plane %1").arg(n + 1));
		_algoPlaneBoxes.append(box);
		updateAlgoHRoiCounts();
		updateAlgoRoiVisibility();
	});

	connect(ui.toolButton_algoHHeightRoi, &QToolButton::clicked, this, [=]() {
		const int n = _algoHeightBoxes.size();
		auto box = makeBox(QRectF(220 + n * 60, 220 + n * 60, 200, 200), kAlgoHeightColor,
			QStringLiteral("Height %1").arg(n + 1));
		_algoHeightBoxes.append(box);
		updateAlgoHRoiCounts();
		updateAlgoRoiVisibility();
	});

	//delete whichever plane/height ROI the user has highlighted (selected) on the scene
	connect(ui.toolButton_algoHRemovePlane, &QToolButton::clicked, this, [=]() {
		bool removed = false;

		for (int i = _algoPlaneBoxes.size() - 1; i >= 0; i--) {
			if (!_algoPlaneBoxes[i]->getSelected()) continue;
			_pGraphicsSceneFOV->removeItem(_algoPlaneBoxes[i]);
			delete _algoPlaneBoxes[i];
			_algoPlaneBoxes.removeAt(i);
			removed = true;
		}
		for (int i = _algoHeightBoxes.size() - 1; i >= 0; i--) {
			if (!_algoHeightBoxes[i]->getSelected()) continue;
			_pGraphicsSceneFOV->removeItem(_algoHeightBoxes[i]);
			delete _algoHeightBoxes[i];
			_algoHeightBoxes.removeAt(i);
			removed = true;
		}

		if (!removed) {
			showMsg("Click a plane or height ROI on the image first, then Remove Selected.");
			return;
		}

		//renumber so names stay Plane 1..N / Height 1..N
		for (int i = 0; i < _algoPlaneBoxes.size(); i++) {
			_algoPlaneBoxes[i]->setup(_algoPlaneBoxes[i]->getGeometry(), kAlgoPlaneColor,
				QStringLiteral("Plane %1").arg(i + 1));
		}
		for (int i = 0; i < _algoHeightBoxes.size(); i++) {
			_algoHeightBoxes[i]->setup(_algoHeightBoxes[i]->getGeometry(), kAlgoHeightColor,
				QStringLiteral("Height %1").arg(i + 1));
		}
		updateAlgoHRoiCounts();
	});

	//── locator controls (bound to the current algo's locator config)
	connect(ui.toolButton_algoLocLearnRoi, &QToolButton::toggled, this, [=](bool) { updateAlgoRoiVisibility(); });
	connect(ui.toolButton_algoLocSearchRoi, &QToolButton::toggled, this, [=](bool) { updateAlgoRoiVisibility(); });

	connect(ui.toolButton_algoLocLearn, &QToolButton::clicked, this, [=]() {
		captureAlgoParamsFromUI(); //so searchAngle/mask margins apply to the learn

		QString error;
		if (!AlgoManager::instance().learnLocatorModel(currentAlgoPageAlgo(), _imageFOV,
			_algoLocLearnBox->getGeometry(), error)) {
			showMsg(error);
			ui.label_algoLocStatus->setText("Learn failed");
			return;
		}
		ui.label_algoLocStatus->setText("Model learned");
		AuditLog::instance().log(QStringLiteral("ALGO_LOCATOR_LEARN"));
	});

	//── auto-save: every edit (widgets and ROI moves) saves the settings after a short
	//debounce; the manual Save Settings button is gone
	_algoAutoSaveTimer = new QTimer(this);
	_algoAutoSaveTimer->setSingleShot(true);
	_algoAutoSaveTimer->setInterval(600);
	connect(_algoAutoSaveTimer, &QTimer::timeout, this, [=]() {
		captureAlgoParamsFromUI();
		if (AlgoManager::instance().saveRecipeConfig()) showStatus("Algo settings saved");
		else showMsg("Failed to save algo settings!");
	});

	{
		QWidget* root = ui.scrollAreaContents_algoPage;
		for (auto* sb : root->findChildren<QSpinBox*>())
			connect(sb, QOverload<int>::of(&QSpinBox::valueChanged), this, &VisionApp::algoSettingsTouched);
		for (auto* db : root->findChildren<QDoubleSpinBox*>())
			connect(db, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &VisionApp::algoSettingsTouched);
		for (auto* cb : root->findChildren<QComboBox*>())
			connect(cb, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &VisionApp::algoSettingsTouched);
		for (auto* ch : root->findChildren<QCheckBox*>())
			connect(ch, &QCheckBox::toggled, this, &VisionApp::algoSettingsTouched);
		for (auto* le : root->findChildren<QLineEdit*>())
			connect(le, &QLineEdit::editingFinished, this, &VisionApp::algoSettingsTouched);
	}

	//── results from the worker thread
	connect(&AlgoManager::instance(), &AlgoManager::ocrFinished, this, [=](const AlgoOcrOutput& out) {
		ui.lineEdit_algoOcr1Result->setText(out.roi1Key.isEmpty() ? out.roi1Text : out.roi1Key + "   (" + out.roi1Text + ")");
		ui.label_algoOcrTime->setText(QStringLiteral("%1 ms").arg(out.elapsedMs));
		ui.label_algoStatus->setText(out.ok ? out.message : "Failed: " + out.message);
		renderAlgoOverlay(out.overlay);
	});

	connect(&AlgoManager::instance(), &AlgoManager::heightFinished, this, [=](const AlgoHeightOutput& out) {
		if (out.ok) {
			QStringList avgLines, rangeLines;
			for (int i = 0; i < out.roiResults.size(); i++) {
				const auto& r = out.roiResults[i];
				if (r.valid) {
					avgLines << QStringLiteral("H%1: %2 um %3").arg(i + 1)
						.arg(r.avgHeightUm, 0, 'f', 2).arg(r.pass ? "" : "(FAIL)");
					rangeLines << QStringLiteral("H%1: %2 .. %3 um").arg(i + 1)
						.arg(r.minHeightUm, 0, 'f', 2).arg(r.maxHeightUm, 0, 'f', 2);
				}
				else {
					avgLines << QStringLiteral("H%1: no valid pixels").arg(i + 1);
				}
			}
			ui.label_algoHAvg->setText(QStringLiteral("Average height:\n%1").arg(avgLines.join("\n")));
			ui.label_algoHMinMax->setText(QStringLiteral("Min / Max:\n%1").arg(rangeLines.join("\n")));
			ui.label_algoHTilt->setText(QStringLiteral("Plane tilt: X %1 deg, Y %2 deg")
				.arg(out.tiltXDeg, 0, 'f', 3).arg(out.tiltYDeg, 0, 'f', 3));
			ui.label_algoHPass->setText(out.pass ? "PASS" : "FAIL");
			ui.label_algoHPass->setStyleSheet(out.pass ? "color: #00C853; font-weight: bold;"
			                                           : "color: #FF5252; font-weight: bold;");
		}
		else {
			ui.label_algoHPass->setText("-");
			ui.label_algoHPass->setStyleSheet("");
		}
		ui.label_algoHTime->setText(QStringLiteral("%1 ms").arg(out.elapsedMs));
		ui.label_algoStatus->setText(out.ok ? out.message : "Failed: " + out.message);
		renderAlgoOverlay(out.overlay);
	});

	connect(&AlgoManager::instance(), &AlgoManager::busyChanged, this, [=](bool busy) {
		ui.toolButton_algoRun->setEnabled(!busy);
		if (busy) ui.label_algoStatus->setText("Running...");
	});

	connect(&AlgoManager::instance(), &AlgoManager::patternsChanged, this, [=]() {
		refreshAlgoPatternList();
	});

	ui.stackedWidget_algoParams->setCurrentIndex(0);
	ui.toolButton_algoH2D->setChecked(true);
}

//Ctrl+C: snapshot the selected 3D ROIs (called from the global event filter)
void VisionApp::algoHCopySelectedRois()
{
	_algoHClipboard.clear();
	for (auto b : _algoPlaneBoxes) if (b->getSelected()) _algoHClipboard.append({ true, b->getGeometry() });
	for (auto b : _algoHeightBoxes) if (b->getSelected()) _algoHClipboard.append({ false, b->getGeometry() });
	if (!_algoHClipboard.isEmpty())
		showStatus(QStringLiteral("%1 ROI(s) copied").arg(_algoHClipboard.size()));
}

//Ctrl+V: paste the snapshot offset by 10 px so the copies are visibly separate
void VisionApp::algoHPasteRois()
{
	if (_algoHClipboard.isEmpty()) return;
	for (const auto& c : _algoHClipboard)
		addAlgoHRoiBox(c.first, c.second.translated(10, 10));
	showStatus(QStringLiteral("%1 ROI(s) pasted").arg(_algoHClipboard.size()));
}

QDragBox* VisionApp::addAlgoHRoiBox(bool isPlane, const QRectF& rect)
{
	auto box = new QDragBox();
	_pGraphicsSceneFOV->addItem(box);
	box->setOutterBarrier(_pGraphicsSceneFOV->sceneRect());
	const int n = (isPlane ? _algoPlaneBoxes.size() : _algoHeightBoxes.size()) + 1;
	box->setup(rect, isPlane ? kAlgoPlaneColor : kAlgoHeightColor,
		QStringLiteral("%1 %2").arg(isPlane ? "Plane" : "Height").arg(n));
	box->setDragable(true);
	box->setZValue((int)UIHierarchy::DRAGGABLES);
	connect(box, SIGNAL(dragBoxMouseReleased(QDragBox*, QString, QPointF)), this, SLOT(algoSettingsTouched()));
	connect(box, SIGNAL(grabberReleased(QDragBox*)), this, SLOT(algoSettingsTouched()));

	if (isPlane) _algoPlaneBoxes.append(box);
	else _algoHeightBoxes.append(box);

	updateAlgoHRoiCounts();
	updateAlgoRoiVisibility();
	return box;
}

AlgoPageAlgo VisionApp::currentAlgoPageAlgo() const
{
	return (AlgoPageAlgo)ui.comboBox_algoType->currentIndex();
}

// ── ROI visibility: only on the algo page, only for the selected algo ────────

void VisionApp::updateAlgoHRoiCounts()
{
	ui.label_algoHPlaneCount->setText(QStringLiteral("Plane ROIs: %1").arg(_algoPlaneBoxes.size()));
	ui.label_algoHHeightCount->setText(QStringLiteral("Height ROIs: %1").arg(_algoHeightBoxes.size()));
}

void VisionApp::updateAlgoRoiVisibility()
{
	const bool onPage = (ui.stackedWidget->currentIndex() == (int)UIPage::ALGO_SETUP)
		&& ui.frame_rightTab->isVisible();
	const AlgoPageAlgo algo = currentAlgoPageAlgo();
	const bool ocr = onPage && (algo == AlgoPageAlgo::OCR_READ);
	const bool height = onPage && (algo == AlgoPageAlgo::HEIGHT_3D);

	if (_algoOcrRoi1Box) _algoOcrRoi1Box->setVisible(ocr && ui.toolButton_algoOcrRoi1->isChecked());
	if (_algoOcrLearnBox) _algoOcrLearnBox->setVisible(ocr && ui.toolButton_algoOcrLearnRoi->isChecked());

	for (auto box : _algoPlaneBoxes) box->setVisible(height);
	for (auto box : _algoHeightBoxes) box->setVisible(height);

	if (_algoLocLearnBox) _algoLocLearnBox->setVisible(onPage && ui.toolButton_algoLocLearnRoi->isChecked());
	if (_algoLocSearchBox) _algoLocSearchBox->setVisible(onPage && ui.toolButton_algoLocSearchRoi->isChecked());

	if (!onPage) clearAlgoOverlay();
}

void VisionApp::hideAlgoSetupRois()
{
	if (_algoOcrRoi1Box) _algoOcrRoi1Box->hide();
	if (_algoOcrLearnBox) _algoOcrLearnBox->hide();
	if (_algoLocLearnBox) _algoLocLearnBox->hide();
	if (_algoLocSearchBox) _algoLocSearchBox->hide();
	for (auto box : _algoPlaneBoxes) box->hide();
	for (auto box : _algoHeightBoxes) box->hide();
	clearAlgoOverlay();
}

// ── params round trip ────────────────────────────────────────────────────────

void VisionApp::captureAlgoParamsFromUI()
{
	auto& mgr = AlgoManager::instance();

	AlgoOcrParams ocr = mgr.ocrParams();
	ocr.orientation = ui.comboBox_algoOcrOrientation->currentText().toInt();
	ocr.roi1Rows = ui.spin_algoOcrRoi1Rows->value();
	ocr.roi1Columns = ui.spin_algoOcrRoi1Cols->value();
	ocr.removeSpecialChars = ui.checkBox_algoOcrRemoveSpecial->isChecked();
	ocr.paddleOcrEnabled = ui.checkBox_algoOcrPaddle->isChecked();
	if (_algoOcrRoi1Box) ocr.roi1Geo = _algoOcrRoi1Box->getGeometry();
	mgr.setOcrParams(ocr);

	AlgoHeightParams h = mgr.heightParams();
	h.intensityPerMicron = ui.dspin_algoHIpm->value();
	h.minHeightUm = ui.dspin_algoHMin->value();
	h.maxHeightUm = ui.dspin_algoHMax->value();
	h.removeOutliers = ui.checkBox_algoHOutliers->isChecked();
	h.planeRois.clear();
	for (auto box : _algoPlaneBoxes) h.planeRois.append(box->getGeometry());
	h.heightRois.clear();
	for (auto box : _algoHeightBoxes) h.heightRois.append(box->getGeometry());
	mgr.setHeightParams(h);

	//locator widgets edit the CURRENT algo's config
	const AlgoPageAlgo algo = currentAlgoPageAlgo();
	AlgoLocatorConfig loc = mgr.locatorConfig(algo);
	loc.enabled = ui.checkBox_algoLocEnable->isChecked();
	loc.scoreThreshold = ui.dspin_algoLocScore->value();
	loc.searchAngle = ui.dspin_algoLocSearchAngle->value();
	loc.angleOffset = ui.dspin_algoLocAngleOffset->value();
	loc.maskMarginW = ui.dspin_algoLocMaskW->value();
	loc.maskMarginH = ui.dspin_algoLocMaskH->value();
	if (_algoLocLearnBox) loc.learnRoi = _algoLocLearnBox->getGeometry();
	if (_algoLocSearchBox) loc.searchRoi = _algoLocSearchBox->getGeometry();
	mgr.setLocatorConfig(algo, loc);
}

void VisionApp::refreshAlgoLocatorUI()
{
	const AlgoLocatorConfig loc = AlgoManager::instance().locatorConfig(currentAlgoPageAlgo());

	QSignalBlocker b1(ui.checkBox_algoLocEnable);
	QSignalBlocker b2(ui.dspin_algoLocScore);
	QSignalBlocker b3(ui.dspin_algoLocSearchAngle);
	QSignalBlocker b4(ui.dspin_algoLocAngleOffset);
	QSignalBlocker b5(ui.dspin_algoLocMaskW);
	QSignalBlocker b6(ui.dspin_algoLocMaskH);

	ui.checkBox_algoLocEnable->setChecked(loc.enabled);
	ui.dspin_algoLocScore->setValue(loc.scoreThreshold);
	ui.dspin_algoLocSearchAngle->setValue(loc.searchAngle);
	ui.dspin_algoLocAngleOffset->setValue(loc.angleOffset);
	ui.dspin_algoLocMaskW->setValue(loc.maskMarginW);
	ui.dspin_algoLocMaskH->setValue(loc.maskMarginH);
	ui.label_algoLocStatus->setText(loc.modelPath.isEmpty() ? "No model" : "Model ready");

	if (!loc.learnRoi.isEmpty() && _algoLocLearnBox) _algoLocLearnBox->setGeometry(loc.learnRoi);
	if (!loc.searchRoi.isEmpty() && _algoLocSearchBox) _algoLocSearchBox->setGeometry(loc.searchRoi);
}

void VisionApp::refreshAlgoSetupPage()
{
	auto& mgr = AlgoManager::instance();

	const AlgoOcrParams ocr = mgr.ocrParams();
	{
		QSignalBlocker b1(ui.comboBox_algoOcrOrientation);
		QSignalBlocker b3(ui.spin_algoOcrRoi1Rows);
		QSignalBlocker b4(ui.spin_algoOcrRoi1Cols);
		QSignalBlocker b9(ui.checkBox_algoOcrRemoveSpecial);
		QSignalBlocker b10(ui.checkBox_algoOcrPaddle);

		ui.comboBox_algoOcrOrientation->setCurrentText(QString::number(ocr.orientation));
		ui.spin_algoOcrRoi1Rows->setValue(ocr.roi1Rows);
		ui.spin_algoOcrRoi1Cols->setValue(ocr.roi1Columns);
		ui.checkBox_algoOcrRemoveSpecial->setChecked(ocr.removeSpecialChars);
		ui.checkBox_algoOcrPaddle->setChecked(ocr.paddleOcrEnabled);
	}
	if (!ocr.roi1Geo.isEmpty() && _algoOcrRoi1Box) _algoOcrRoi1Box->setGeometry(ocr.roi1Geo);

	const OcrPatternConfig pat = mgr.patternConfig();
	{
		QSignalBlocker b1(ui.checkBox_algoOcrPatternEnable);
		QSignalBlocker b2(ui.dspin_algoOcrPatternScore);
		ui.checkBox_algoOcrPatternEnable->setChecked(pat.enabled);
		ui.dspin_algoOcrPatternScore->setValue(pat.scoreThreshold);
	}

	const AlgoHeightParams h = mgr.heightParams();
	{
		QSignalBlocker b1(ui.dspin_algoHIpm);
		QSignalBlocker b2(ui.dspin_algoHMin);
		QSignalBlocker b3(ui.dspin_algoHMax);
		QSignalBlocker b4(ui.checkBox_algoHOutliers);
		ui.dspin_algoHIpm->setValue(h.intensityPerMicron);
		ui.dspin_algoHMin->setValue(h.minHeightUm);
		ui.dspin_algoHMax->setValue(h.maxHeightUm);
		ui.checkBox_algoHOutliers->setChecked(h.removeOutliers);
	}

	//rebuild plane + height ROI boxes from the stored geometries
	auto rebuildBoxes = [=](QVector<QDragBox*>& boxes, const QVector<QRectF>& rois,
		const QColor& color, const QString& prefix) {
		for (auto box : boxes) {
			_pGraphicsSceneFOV->removeItem(box);
			delete box;
		}
		boxes.clear();

		int idx = 0;
		for (const auto& r : rois) {
			auto box = new QDragBox();
			_pGraphicsSceneFOV->addItem(box);
			box->setOutterBarrier(_pGraphicsSceneFOV->sceneRect());
			box->setup(r, color, QStringLiteral("%1 %2").arg(prefix).arg(++idx));
			box->setDragable(true);
			box->setZValue((int)UIHierarchy::DRAGGABLES);
			box->hide();
			boxes.append(box);
		}
	};

	rebuildBoxes(_algoPlaneBoxes, h.planeRois, kAlgoPlaneColor, QStringLiteral("Plane"));
	rebuildBoxes(_algoHeightBoxes, h.heightRois, kAlgoHeightColor, QStringLiteral("Height"));
	updateAlgoHRoiCounts();

	refreshAlgoLocatorUI();
	refreshAlgoPatternList();
	updateAlgoRoiVisibility();
}

// ── heightmap display (2D grayscale for setup, 3D colormap for viewing) ─────

void VisionApp::showAlgoHeightMap(bool view3D)
{
	_algoHeightView3D = view3D;

	QSignalBlocker b1(ui.toolButton_algoH2D);
	QSignalBlocker b2(ui.toolButton_algoH3D);
	ui.toolButton_algoH2D->setChecked(!view3D);
	ui.toolButton_algoH3D->setChecked(view3D);

	const QImage img = AlgoManager::instance().heightMapImage(view3D);
	if (img.isNull()) {
		showStatus("No heightmap loaded.");
		return;
	}

	displayFOV(img);
	ui.graphicsViewFOV->fitInView(_pPixmapItemFOV, Qt::KeepAspectRatio);
}

// ── overlay rendering ────────────────────────────────────────────────────────

void VisionApp::clearAlgoOverlay()
{
	for (auto item : _algoOverlayItems) {
		_pGraphicsSceneFOV->removeItem(item);
		delete item;
	}
	_algoOverlayItems.clear();
}

void VisionApp::renderAlgoOverlay(const QVector<AlgoOverlayItem>& overlay)
{
	clearAlgoOverlay();

	for (const auto& item : overlay) {
		QGraphicsItem* gfx = nullptr;

		switch (item.type) {
		case AlgoOverlayItem::Rect: {
			auto rect = new QGraphicsRectItem(item.rect);
			rect->setPen(QPen(item.color, 2));
			rect->setBrush(item.fill);
			gfx = rect;
			break;
		}
		case AlgoOverlayItem::Polygon: {
			auto poly = new QGraphicsPolygonItem(item.poly);
			poly->setPen(QPen(item.color, 2));
			gfx = poly;
			break;
		}
		case AlgoOverlayItem::Text: {
			auto text = new QGraphicsTextItem(item.text);
			text->setDefaultTextColor(item.color);
			QFont font;
			font.setPointSize(item.pointSize);
			font.setBold(true);
			text->setFont(font);
			text->setPos(item.pos);
			gfx = text;
			break;
		}
		case AlgoOverlayItem::Cross:
		default:
			break;
		}

		if (!gfx) continue;
		gfx->setZValue((int)UIHierarchy::SHAPE);
		_pGraphicsSceneFOV->addItem(gfx);
		_algoOverlayItems.append(gfx);
	}
}

// ── pattern library list (port of IM430 refreshOcrLabelList) ────────────────

void VisionApp::refreshAlgoPatternList()
{
	const OcrPatternConfig cfg = AlgoManager::instance().patternConfig();

	auto* tbl = ui.tableWidget_algoOcrPatternLabels;
	tbl->blockSignals(true);
	tbl->setColumnCount(3);
	tbl->setHorizontalHeaderLabels({ "Label", "Samples", "Enable" });
	tbl->setRowCount(0);
	tbl->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
	tbl->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
	tbl->horizontalHeader()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
	tbl->verticalHeader()->setVisible(false);

	QStringList unlearned;
	for (const auto& lbl : cfg.labels) {
		const int row = tbl->rowCount();
		tbl->insertRow(row);
		const bool hasNoSamples = lbl.samples.isEmpty();

		auto* itemLabel = new QTableWidgetItem(lbl.label);
		itemLabel->setFlags(itemLabel->flags() & ~Qt::ItemIsEditable);
		if (hasNoSamples) itemLabel->setBackground(QColor(180, 50, 50));
		tbl->setItem(row, 0, itemLabel);

		auto* itemCount = new QTableWidgetItem(QString::number(lbl.samples.size()));
		itemCount->setFlags(itemCount->flags() & ~Qt::ItemIsEditable);
		if (hasNoSamples) itemCount->setBackground(QColor(180, 50, 50));
		tbl->setItem(row, 1, itemCount);

		auto* itemEnable = new QTableWidgetItem();
		itemEnable->setFlags(Qt::ItemIsUserCheckable | Qt::ItemIsEnabled);
		itemEnable->setCheckState(lbl.enabled ? Qt::Checked : Qt::Unchecked);
		itemEnable->setTextAlignment(Qt::AlignCenter);
		tbl->setItem(row, 2, itemEnable);

		if (hasNoSamples) unlearned.append(lbl.label);
	}
	tbl->blockSignals(false);

	//break the enumeration into short lines so the label never stretches the page
	QString unlearnedText;
	if (!unlearned.isEmpty()) {
		QStringList lines;
		for (int i = 0; i < unlearned.size(); i += 12) {
			lines << QStringList(unlearned.mid(i, 12)).join(", ");
		}
		unlearnedText = QStringLiteral("Unlearned (%1):\n%2").arg(unlearned.size()).arg(lines.join("\n"));
	}
	ui.label_algoOcrUnlearned->setText(unlearnedText);

	//── per-label sample thumbnails (click to delete)
	auto* vLayout = ui.verticalLayout_algoOcrPatternList;
	while (vLayout->count() > 0) {
		QLayoutItem* item = vLayout->takeAt(0);
		if (item->widget()) item->widget()->deleteLater();
		delete item;
	}

	constexpr int THUMB = 50;

	for (const auto& lbl : cfg.labels) {
		if (lbl.samples.isEmpty()) continue;

		auto* section = new QFrame();
		section->setFrameShape(QFrame::StyledPanel);
		section->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
		section->setStyleSheet(QStringLiteral("QFrame { background:#1e2028; border-radius:6px; }"));
		auto* secLayout = new QVBoxLayout(section);
		secLayout->setContentsMargins(4, 4, 4, 4);
		secLayout->setSpacing(3);

		auto* title = new QLabel(QStringLiteral("Character: '%1'  (%2 samples)")
			.arg(lbl.label).arg(lbl.samples.size()));
		title->setStyleSheet(QStringLiteral("color:white; font-size:12px; font-weight:bold; background:transparent;"));
		secLayout->addWidget(title);

		auto* rowWidget = new QWidget();
		auto* rowLayout = new QHBoxLayout(rowWidget);
		rowLayout->setContentsMargins(2, 2, 2, 2);
		rowLayout->setSpacing(4);

		for (int si = 0; si < lbl.samples.size(); ++si) {
			QString jpgPath = lbl.samples[si].filePath;
			jpgPath.replace(QStringLiteral(".mpat"), QStringLiteral(".jpg"));

			auto* imgBtn = new QPushButton();
			imgBtn->setFixedSize(THUMB, THUMB);
			imgBtn->setFlat(true);
			imgBtn->setToolTip(QStringLiteral("Click to delete this sample"));
			imgBtn->setStyleSheet(QStringLiteral(
				"QPushButton { background:#333; border:none; padding:0; }"
				"QPushButton:hover { background:#555; border:1px solid orange; }"));

			QPixmap px(jpgPath);
			if (!px.isNull()) {
				imgBtn->setIcon(QIcon(px.scaled(THUMB, THUMB, Qt::KeepAspectRatio, Qt::SmoothTransformation)));
				imgBtn->setIconSize(QSize(THUMB, THUMB));
			}

			const QString labelName = lbl.label;
			connect(imgBtn, &QPushButton::clicked, this, [this, labelName, si]() {
				auto reply = QMessageBox::question(this, "Delete Sample",
					QStringLiteral("Delete sample %1 of character '%2'?").arg(si + 1).arg(labelName),
					QMessageBox::Yes | QMessageBox::No);
				if (reply != QMessageBox::Yes) return;
				AlgoManager::instance().deletePatternSample(labelName, si);
			});

			rowLayout->addWidget(imgBtn);
		}
		rowLayout->addStretch();

		secLayout->addWidget(rowWidget);
		vLayout->addWidget(section);
	}
}

//any algo-setup edit lands here: debounce, then capture + save
void VisionApp::algoSettingsTouched()
{
	if (_algoAutoSaveTimer) _algoAutoSaveTimer->start();
}

