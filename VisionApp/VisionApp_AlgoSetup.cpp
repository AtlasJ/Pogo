// =============================================================================
//  VisionApp_AlgoSetup.cpp
//  Algo Setup page: OCR inspection (PaddleOCR + pattern matching, ported from
//  IM430 ocrInspection2 core) with an optional locator, and the page shell the
//  3D Height Measurement 3 page lives in (its own wiring is in
//  VisionApp_AlgoHeight3.cpp).
//
//  All processing runs on AlgoManager's worker threads; this file is UI glue:
//  ROI drag boxes, parameter round-tripping, overlay rendering, pattern
//  library management.
// =============================================================================

#include "VisionApp.h"
#include "AlgoManager.h"
#include "AuditLog.h"

#include <QAbstractSpinBox>
#include <QApplication>
#include <QPainter>
#include <QHeaderView>
#include <QLineEdit>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QTableWidgetItem>
#include <QTextEdit>

static const QColor kAlgoRoiColor(0, 150, 255);
static const QColor kAlgoLearnColor(66, 135, 245);
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
		//the Locator frame is shared by all pages, but the V3 page has none - hide it there
		//rather than leave a set of controls on screen that write nowhere
		ui.frame_algoLocator->setVisible(algoHasLocator(currentAlgoPageAlgo()));
		refreshAlgoLocatorUI();
		updateAlgoRoiVisibility();

		if (currentAlgoPageAlgo() == AlgoPageAlgo::HEIGHT_3D_V3) {
			//first visit this session: put back whatever maps were loaded when the app was
			//last closed. Before the two updates below, so they see the restored maps.
			restoreAlgoH3Inputs();
			updateAlgoH3Enables();
			updateAlgoH3Display();
		}
	});

	connect(ui.toolButton_algoRun, &QToolButton::clicked, this, [=]() {
		if (AlgoManager::instance().isBusy()) {
			showMsg("Algo is still running, please wait.");
			return;
		}

		//V3's Run is a Run All: every stage in order, stopping at the first failure. It
		//does its own capture and validation, so hand straight over.
		if (currentAlgoPageAlgo() == AlgoPageAlgo::HEIGHT_3D_V3) {
			algoH3RunStage(AlgoH3Stage::All);
			return;
		}

		captureAlgoParamsFromUI();
		clearAlgoOverlay();
		setAlgoStatus("Running...");

		AlgoManager::instance().runOcr(_imageFOV);
		AuditLog::instance().log(QStringLiteral("ALGO_RUN_OCR"));
	});

	//── OCR page controls
	connect(ui.toolButton_algoOcrRoi1, &QToolButton::toggled, this, [=](bool) { updateAlgoRoiVisibility(); });

	/*
	* Reference IMAGE: "Save as Ref" stores the current FOV image as a machine-level
	* reference; "Load Ref" brings it back into the display. Settings are not part of
	* the reference.
	*/
	const QString ocrRefPath = QStringLiteral("%1/algoRef_ocr.png").arg(Common::Directory::ConfigPath());

	auto refreshRefButtons = [=]() {
		ui.toolButton_algoOcrLoadRef->setEnabled(QFile::exists(ocrRefPath));
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
		setAlgoStatus(out.ok ? out.message : "Failed: " + out.message);

		//super-resolution preview: paint the enhanced crop over its ROI in the display.
		//_imageFOV itself stays raw, so every run starts from the original image.
		if (!out.srImage.isNull() && !_imageFOV.isNull() && isPage(UIPage::ALGO_SETUP)) {
			QImage disp = _imageFOV.convertToFormat(QImage::Format_RGB32);
			QPainter p(&disp);
			p.drawImage(out.srRect, out.srImage);
			p.end();
			displayFOV(disp);
		}

		renderAlgoOverlay(out.overlay);
	});

	connect(&AlgoManager::instance(), &AlgoManager::busyChanged, this, [=](bool busy) {
		ui.toolButton_algoRun->setEnabled(!busy);
		if (busy) setAlgoStatus("Running...");
		updateAlgoH3Enables(); //V3's per-section Run buttons follow the same busy state
	});

	connect(&AlgoManager::instance(), &AlgoManager::patternsChanged, this, [=]() {
		refreshAlgoPatternList();
	});

	ui.stackedWidget_algoParams->setCurrentIndex(0);

	//the 3D Height Measurement 3 page owns its own wiring - see VisionApp_AlgoHeight3.cpp
	initAlgoHeight3Page();
}

/*
* THE ONE PLACE Ctrl+C / Ctrl+V IS DECIDED. Both keys arrive here from the QShortcuts in
* VisionApp_Shortcuts.cpp, and this picks the handler by which page is open.
*
* It has to be a shortcut and not an event filter, and that is worth spelling out because
* it was got wrong once: Qt dispatches shortcuts BEFORE key events exist. A key press first
* goes out as QEvent::ShortcutOverride, and if nobody accepts it the shortcut map fires the
* QShortcut and NO QEvent::KeyPress is ever generated. So while a QShortcut owns Ctrl+C,
* an eventFilter watching for KeyPress can never see it - which is exactly why the algo
* pages' copy/paste looked implemented but had never once run, on any of them.
*
* Text editing is left alone. QLineEdit and the spin boxes accept ShortcutOverride for the
* standard Copy/Paste sequences and so would win anyway, but the guard is kept explicit
* rather than resting on that.
*/
bool VisionApp::copyPasteGoesToText() const
{
	QWidget* fw = QApplication::focusWidget();
	return qobject_cast<QLineEdit*>(fw) || qobject_cast<QTextEdit*>(fw)
		|| qobject_cast<QPlainTextEdit*>(fw) || qobject_cast<QAbstractSpinBox*>(fw);
}

void VisionApp::copyShortcutPressed()
{
	if (copyPasteGoesToText()) return;

	if (isPage(UIPage::ALGO_SETUP)) {
		//only V3 has ROIs to copy; the OCR page's boxes are fixed, one of each
		if (currentAlgoPageAlgo() == AlgoPageAlgo::HEIGHT_3D_V3) algoH3CopySelectedRois();
		return;
	}

	copyVisionObject();
}

/*
* Delete. Only the algo pages claim it, and only for ROIs - everywhere else the key keeps
* whatever meaning the focused widget gives it, which is why the text guard comes first.
*
* There is deliberately no confirmation. An ROI is cheap to put back, Delete is a key people
* press on purpose, and a dialog on every press would make deleting fifty of them a chore.
*/
bool VisionApp::deleteShortcutPressed()
{
	if (copyPasteGoesToText()) return false;
	if (!isPage(UIPage::ALGO_SETUP)) return false;
	if (currentAlgoPageAlgo() != AlgoPageAlgo::HEIGHT_3D_V3) return false;

	algoH3DeleteSelectedRois();
	//consumed either way: on this page Delete means the page's ROIs, and falling through to
	//the recipe page's handler would delete vision objects the operator cannot even see
	return true;
}

/*
* Select all. Only the algo pages claim it - anywhere else Ctrl+A does nothing, rather than
* reaching some other page's function. A text field keeps its own select-all.
*
* Visible boxes only: setSelected() is a no-op on a hidden item anyway, and a box the page
* is hiding belongs to another algo or another section.
*/
void VisionApp::selectAllShortcutPressed()
{
	if (copyPasteGoesToText()) return;
	if (!isPage(UIPage::ALGO_SETUP)) return;

	if (currentAlgoPageAlgo() == AlgoPageAlgo::HEIGHT_3D_V3) algoH3SelectAllRois();
}

void VisionApp::pasteShortcutPressed()
{
	if (copyPasteGoesToText()) return;

	if (isPage(UIPage::ALGO_SETUP)) {
		if (currentAlgoPageAlgo() == AlgoPageAlgo::HEIGHT_3D_V3) algoH3PasteRois();
		return;
	}

	//the page test above is also what stops a vision-object clipboard from being pasted
	//into the recipe while the operator is looking at the Algo Setup page
	pasteVisionObject();
}

/*
* ALWAYS ASK A BOX `isSelected()`, NEVER `getSelected()`. Every algo page reads selection
* through Qt's own QGraphicsItem::isSelected(), and that is not a style preference:
*
* QDragBox::_isSelected is declared WITHOUT an initialiser (QDragBox.h) and is written in
* exactly one place - paintFunction(), from option->state. So on a box that has not been
* painted yet the value is INDETERMINATE, and Qt does not paint items outside the viewport,
* so a box that has never been scrolled into view keeps that indeterminate value for good.
*
* What that looked like in practice (on the old V1 height page, whose Run destroyed and rebuilt
* every ROI box): the new boxes landed in the heap blocks just freed by the old ones, and _isSelected
* often came back as the OLD box's value. Copy then reported 18 ROIs copied with nothing
* selected, and the results section counted phantom selections and refused to show a result
* for the one ROI actually clicked. isSelected() has none of these problems - it is the real
* state, correct immediately, independent of painting and of the viewport.
*
* This is why copyVisionObject() was never affected: it already used isSelected().
*/

AlgoPageAlgo VisionApp::currentAlgoPageAlgo() const
{
	return (AlgoPageAlgo)ui.comboBox_algoType->currentIndex();
}

// ── ROI visibility: only on the algo page, only for the selected algo ────────

void VisionApp::updateAlgoRoiVisibility()
{
	const bool onPage = (ui.stackedWidget->currentIndex() == (int)UIPage::ALGO_SETUP)
		&& ui.frame_rightTab->isVisible();
	const AlgoPageAlgo algo = currentAlgoPageAlgo();
	const bool ocr = onPage && (algo == AlgoPageAlgo::OCR_READ);

	if (_algoOcrRoi1Box) _algoOcrRoi1Box->setVisible(ocr && ui.toolButton_algoOcrRoi1->isChecked());
	if (_algoOcrLearnBox) _algoOcrLearnBox->setVisible(ocr && ui.toolButton_algoOcrLearnRoi->isChecked());

	//gated on algoHasLocator as well: frame_algoLocator is hidden on the pages that have no
	//locator, but its toggle buttons keep whatever checked state they were left in, so without
	//this the learn/search boxes stay on the FOV with no visible control owning them
	const bool loc = onPage && algoHasLocator(algo);
	if (_algoLocLearnBox) _algoLocLearnBox->setVisible(loc && ui.toolButton_algoLocLearnRoi->isChecked());
	if (_algoLocSearchBox) _algoLocSearchBox->setVisible(loc && ui.toolButton_algoLocSearchRoi->isChecked());

	//V3's ROIs have their own rule - the open section decides which set is shown
	updateAlgoH3RoiVisibility();

	if (!onPage) clearAlgoOverlay();
}

void VisionApp::hideAlgoSetupRois()
{
	if (_algoOcrRoi1Box) _algoOcrRoi1Box->hide();
	if (_algoOcrLearnBox) _algoOcrLearnBox->hide();
	if (_algoLocLearnBox) _algoLocLearnBox->hide();
	if (_algoLocSearchBox) _algoLocSearchBox->hide();
	hideAlgoH3Rois();
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
	ocr.superResolution = ui.checkBox_algoOcrSR->isChecked();
	if (_algoOcrRoi1Box) ocr.roi1Geo = _algoOcrRoi1Box->getGeometry();
	mgr.setOcrParams(ocr);

	//3D Height Measurement 3 lives on its own page with its own params struct; capture it
	//BEFORE the locator early-return below, or nothing on that page would ever be saved
	captureAlgoH3ParamsFromUI();

	//locator widgets edit the CURRENT algo's config - but HEIGHT_3D_V3 has no locator slot,
	//so skip rather than round-trip a config that setLocatorConfig() would discard anyway
	const AlgoPageAlgo algo = currentAlgoPageAlgo();
	if (!algoHasLocator(algo)) return;

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
		QSignalBlocker b11(ui.checkBox_algoOcrSR);

		ui.comboBox_algoOcrOrientation->setCurrentText(QString::number(ocr.orientation));
		ui.spin_algoOcrRoi1Rows->setValue(ocr.roi1Rows);
		ui.spin_algoOcrRoi1Cols->setValue(ocr.roi1Columns);
		ui.checkBox_algoOcrRemoveSpecial->setChecked(ocr.removeSpecialChars);
		ui.checkBox_algoOcrPaddle->setChecked(ocr.paddleOcrEnabled);
		ui.checkBox_algoOcrSR->setChecked(ocr.superResolution);
	}
	if (!ocr.roi1Geo.isEmpty() && _algoOcrRoi1Box) _algoOcrRoi1Box->setGeometry(ocr.roi1Geo);

	const OcrPatternConfig pat = mgr.patternConfig();
	{
		QSignalBlocker b1(ui.checkBox_algoOcrPatternEnable);
		QSignalBlocker b2(ui.dspin_algoOcrPatternScore);
		ui.checkBox_algoOcrPatternEnable->setChecked(pat.enabled);
		ui.dspin_algoOcrPatternScore->setValue(pat.scoreThreshold);
	}

	refreshAlgoLocatorUI();
	refreshAlgoPatternList();
	refreshAlgoHeight3Page();
	updateAlgoRoiVisibility();
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
			QPen pen(item.color, item.penWidth);
			pen.setCosmetic(item.cosmeticPen);
			rect->setPen(pen);
			rect->setBrush(item.fill);
			gfx = rect;
			break;
		}
		case AlgoOverlayItem::Polygon: {
			auto poly = new QGraphicsPolygonItem(item.poly);
			QPen pen(item.color, item.penWidth);
			pen.setCosmetic(item.cosmeticPen);
			poly->setPen(pen);
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

/*
* The status line in the algo page header, shared by every algo. A flat read-only line edit
* rather than a label: it never resizes the header, and a message longer than the room it has
* can still be read to the end (Home/End, drag) or copied - a label just cut it off. The whole
* text is on the tooltip too.
*/
void VisionApp::setAlgoStatus(const QString& text)
{
	ui.lineEdit_algoStatus->setText(text);
	ui.lineEdit_algoStatus->setToolTip(text);
	//setText leaves the cursor at the end, and a line edit too narrow for its text then shows
	//the TAIL - keep the start of the message in view
	ui.lineEdit_algoStatus->setCursorPosition(0);
}

