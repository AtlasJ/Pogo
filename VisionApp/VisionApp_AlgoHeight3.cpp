// =============================================================================
//  VisionApp_AlgoHeight3.cpp
//  The "3D Height Measurement 3" page of Algo Setup: everything between the eight
//  QToolBox sections and AlgoHeight3Pipeline.
//
//  The organising idea of the page is that THE OPEN SECTION DECIDES WHAT YOU SEE.
//  Each section owns one step of the pipeline, so it shows the image that step works
//  on and only the ROIs that step owns:
//
//     0 Input        full raw map          no ROIs
//     1 Preprocess   full raw map          no ROIs
//     2 Segmentation full processed map    no ROIs (the found rectangle is drawn on it)
//     3 Datum plane  straightened crop     datum ROIs only (orange)
//     4 Method       straightened crop     no ROIs
//     5 ROI types    straightened crop     measurement ROIs only
//     6 Results      straightened crop     measurement ROIs only (pick one to read it)
//     7 Overall      straightened crop     no ROIs
//
//  The 3D Surface View never shows an ROI at all - it is a viewing aid, and an ROI
//  dragged in a projection would land somewhere nobody intended.
//
//  No ROI can be shown or dragged before segmentation has succeeded, because every ROI
//  is stored relative to the CENTRE OF THE SEGMENTED CROP. Without a crop there is no
//  origin to be relative to, so an ROI is not merely unhelpful, it is undefined.
// =============================================================================

#include "VisionApp.h"
#include "AlgoManager.h"
#include "AuditLog.h"

#include <QColorDialog>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QHeaderView>
#include <QInputDialog>
#include <QMessageBox>
#include <QSpinBox>
#include <QTableWidgetItem>
#include <QToolButton>

#include <cmath>

// ── the datum plane's ROIs are always orange; the operator does not get to pick, so
// ── that one colour always means "this is a datum patch" on every recipe
static const QColor kAlgoH3DatumColor(255, 165, 0);

//the page's verdict colours, in one place because they are now painted twice: on the
//PASS / FAIL fields and on the per-ROI labels drawn over the image
static const QColor kAlgoH3PassColor(0, 200, 83);
static const QColor kAlgoH3FailColor(255, 82, 82);

/*
* Per-ROI result labels. The size is in SCENE units, so a label scales with the part as
* the view is zoomed - the same behaviour as the V1 height overlay. That is deliberate:
* fit-to-view on a 9000 px crop is far too small to read a few hundred numbers anyway, and
* zooming in to look at a pin is exactly when its number becomes legible.
* The offset lifts the text clear of the box's top edge at that size.
*/
static const int kAlgoH3LabelPointSize = 24;
static const double kAlgoH3LabelOffsetPx = kAlgoH3LabelPointSize * 5.0 / 3.0;

//fixed canvas for the software 3D render: a constant size keeps the scene rect stable,
//so spinning the surface never makes the view jump or refit
static const QSize kAlgoH3SurfaceCanvas(1200, 900);

namespace {

//section indices of toolBox_algoH3Sections
enum AlgoH3Section {
	SEC_INPUT = 0,
	SEC_PREPROCESS,
	SEC_SEG,
	SEC_DATUM,
	SEC_METHOD,
	SEC_ROI,
	SEC_RESULT,
	SEC_OVERALL
};

/*
* Which kind of ROI the open section owns - the page's organising rule, made explicit so
* copy and paste are gated on the same thing the display is. Only section 3 owns datum
* ROIs; only 5 and 6 own measurement ROIs; every other section owns none, and copy/paste
* has no meaning there.
*/
enum class H3RoiOwner { None, Datum, Measurement };

static H3RoiOwner h3SectionOwner(int section)
{
	if (section == SEC_DATUM) return H3RoiOwner::Datum;
	if (section == SEC_ROI || section == SEC_RESULT) return H3RoiOwner::Measurement;
	return H3RoiOwner::None;
}

/*
* Is the page showing a 3D projection rather than a flat map?
*
* The two 3D modes differ ONLY in how the render is painted. Everything else about them
* is identical: no ROI drag boxes, no overlays, and drag-to-spin. Six separate places
* used to compare against Surface3D by name; ask this instead, so adding a third 3D
* style is one enum value and not another six-way edit that one site will be missed in.
*/
static bool h3IsSurfaceMode(AlgoH3Display m)
{
	return algoH3IsSurfaceDisplay(m);
}

//which paint style each 3D mode asks the one shared renderer for
static AlgoH3SurfaceStyle h3StyleFor(AlgoH3Display m)
{
	switch (m) {
	case AlgoH3Display::Mesh3D:       return AlgoH3SurfaceStyle::ShadedMesh;
	case AlgoH3Display::Smooth3D:     return AlgoH3SurfaceStyle::SmoothShaded;
	case AlgoH3Display::Wireframe3D:  return AlgoH3SurfaceStyle::Wireframe;
	case AlgoH3Display::PointCloud3D: return AlgoH3SurfaceStyle::PointCloud;
	case AlgoH3Display::Textured3D:   return AlgoH3SurfaceStyle::Textured;
	default:                          return AlgoH3SurfaceStyle::Filled;
	}
}

//what an ROI's label reads, shared by the measurement sections and the overall section so
//the same measurement never appears formatted two different ways
static QString h3RoiLabelText(const AlgoH3RoiResult& r)
{
	return r.valid ? QString::number(r.heightUm, 'f', 1) : QStringLiteral("NO DATA");
}

//the same sentence in both refusals, so the operator is told where the ROIs DO belong
static const char* kH3RoiSectionHint =
	"Open the Datum Plane section to work with datum ROIs, or ROI Types & Criteria / "
	"Measurement Results for measurement ROIs.";

const char* kH3ColorProp = "algoH3Color"; //QColor carried by a type row's colour button

static void h3SetReadonly(QLineEdit* le)
{
	if (le) le->setReadOnly(true);
}

static void h3ShowVerdict(QLineEdit* le, bool ran, bool pass)
{
	if (!le) return;
	if (!ran) {
		le->setText(QStringLiteral("-"));
		le->setStyleSheet(QString());
		return;
	}
	le->setText(pass ? QStringLiteral("PASS") : QStringLiteral("FAIL"));
	le->setStyleSheet(QStringLiteral("color:%1; font-weight:bold;")
		.arg((pass ? kAlgoH3PassColor : kAlgoH3FailColor).name()));
}

//the Result / Fail Reason / Time trio every section shares
static void h3ShowStage(QLineEdit* result, QLineEdit* reason, QLineEdit* time,
	const AlgoH3StageResult& s)
{
	h3ShowVerdict(result, s.ran, s.pass);
	if (reason) reason->setText(s.ran ? s.failReason : QString());
	if (time) time->setText(s.ran ? QString::number(s.elapsedMs) : QString());
}

static void h3ShowNumber(QLineEdit* le, bool have, double v, int decimals)
{
	if (!le) return;
	le->setText(have ? QString::number(v, 'f', decimals) : QStringLiteral("-"));
}

} //namespace

// =============================================================================
// Setup
// =============================================================================

void VisionApp::initAlgoHeight3Page()
{
	configureAlgoH3Ranges();

	//every readout on this page is output, never input
	const QStringList readouts = {
		"lineEdit_algoH3InputLoadHeightStatus", "lineEdit_algoH3InputLoadIntensityStatus",
		"lineEdit_algoH3InputUseLastScanStatus",
		"lineEdit_algoH3PreprocessResult", "lineEdit_algoH3PreprocessFailReason", "lineEdit_algoH3PreprocessTimeMs",
		"lineEdit_algoH3SegMeasuredWidthUm", "lineEdit_algoH3SegMeasuredHeightUm", "lineEdit_algoH3SegMeasuredAngleDeg",
		"lineEdit_algoH3SegResult", "lineEdit_algoH3SegFailReason", "lineEdit_algoH3SegTimeMs",
		"lineEdit_algoH3DatumRoiCount", "lineEdit_algoH3DatumCoeffA", "lineEdit_algoH3DatumCoeffB",
		"lineEdit_algoH3DatumCoeffC", "lineEdit_algoH3DatumMeasuredTiltDeg", "lineEdit_algoH3DatumResult",
		"lineEdit_algoH3DatumFailReason", "lineEdit_algoH3DatumTimeMs",
		"lineEdit_algoH3RoiCount",
		"lineEdit_algoH3ResultTimeMs", "lineEdit_algoH3ResultRoiId", "lineEdit_algoH3ResultRoiType",
		"lineEdit_algoH3ResultCriteriaMinUm", "lineEdit_algoH3ResultCriteriaMaxUm",
		"lineEdit_algoH3ResultMethodId", "lineEdit_algoH3ResultMethodName", "lineEdit_algoH3ResultZHeightUm",
		"lineEdit_algoH3ResultPassFail", "lineEdit_algoH3ResultFailReason",
		"lineEdit_algoH3OverallTotalPins", "lineEdit_algoH3OverallPassedPins",
		"lineEdit_algoH3OverallFailedPins", "lineEdit_algoH3OverallFailedRate",
		"lineEdit_algoH3OverallPassFail", "lineEdit_algoH3OverallFailReason",
		"lineEdit_algoH3OverallTimeMs", "lineEdit_algoH3OverallTotalTimeMs"
	};
	for (const QString& name : readouts)
		h3SetReadonly(ui.widget_algoHeight3Page->findChild<QLineEdit*>(name));

	//Ctrl+C / Ctrl+V has no button of its own, so say so where the operator is looking
	const QString copyHint = QStringLiteral(
		"\n\nSelect ROIs on the image and press Ctrl+C to copy, Ctrl+V to paste them offset by 10 px.");
	ui.toolButton_algoH3RoiAdd->setToolTip(
		"Add one ROI of the selected type, at the centre of the segmented part." + copyHint);
	ui.toolButton_algoH3DatumAddRoi->setToolTip(
		"Add one datum ROI, at the centre of the segmented part." + copyHint);

	// ── display mode + section: both decide what is on screen ──
	connect(ui.comboBox_algoH3Display, QOverload<int>::of(&QComboBox::currentIndexChanged),
		this, [=](int) { updateAlgoH3Display(); });

	connect(ui.toolBox_algoH3Sections, &QToolBox::currentChanged, this, [=](int) {
		updateAlgoH3Display();
		updateAlgoH3RoiVisibility();
		refreshAlgoH3ResultSection();
	});

	// ── section 0: input ──
	connect(ui.toolButton_algoH3InputLoadHeight, &QToolButton::clicked, this, [=]() {
		const QString path = QFileDialog::getOpenFileName(this, "Load Height Map",
			Common::Directory::getRecipeCurrentPath(), "Height map (*.tiff *.tif)");
		if (path.isEmpty()) return;

		QString error;
		if (!AlgoManager::instance().height3LoadHeightFile(path, error)) {
			ui.lineEdit_algoH3InputLoadHeightStatus->setText(error);
			showMsg(error);
			return;
		}
		ui.lineEdit_algoH3InputLoadHeightStatus->setText(QFileInfo(path).fileName());
		//a new map invalidates everything taught against the old one
		_algoH3Output = AlgoHeight3Output();
		refreshAlgoH3RoiBoxes();
		applyAlgoH3Output(_algoH3Output);
		updateAlgoH3Display();
		updateAlgoH3Enables(); //a map is loaded now, so Preprocess and Segmentation can run
		AuditLog::instance().log(QStringLiteral("ALGO_H3_LOAD_HEIGHT"), QFileInfo(path).fileName());
	});

	connect(ui.toolButton_algoH3InputLoadIntensity, &QToolButton::clicked, this, [=]() {
		const QString path = QFileDialog::getOpenFileName(this, "Load Intensity Map",
			Common::Directory::getRecipeCurrentPath(),
			"Intensity map (*.png *.jpg *.jpeg *.bmp *.tif *.tiff)");
		if (path.isEmpty()) return;

		QString error;
		if (!AlgoManager::instance().height3LoadIntensityFile(path, error)) {
			ui.lineEdit_algoH3InputLoadIntensityStatus->setText(error);
			showMsg(error);
			return;
		}
		ui.lineEdit_algoH3InputLoadIntensityStatus->setText(QFileInfo(path).fileName());
		updateAlgoH3Display();
		updateAlgoH3Enables();
		AuditLog::instance().log(QStringLiteral("ALGO_H3_LOAD_INTENSITY"), QFileInfo(path).fileName());
	});

	connect(ui.toolButton_algoH3InputUseLastScan, &QToolButton::clicked, this, [=]() {
		QString note;
		if (!AlgoManager::instance().height3UseLastScan(note)) {
			ui.lineEdit_algoH3InputUseLastScanStatus->setText(note);
			showMsg(note);
			return;
		}
		ui.lineEdit_algoH3InputUseLastScanStatus->setText(
			note.isEmpty() ? QStringLiteral("Last scan loaded") : note);
		ui.lineEdit_algoH3InputLoadHeightStatus->setText(QStringLiteral("(from last scan)"));
		ui.lineEdit_algoH3InputLoadIntensityStatus->setText(
			AlgoManager::instance().height3HasIntensity() ? QStringLiteral("(from last scan)") : QString());

		_algoH3Output = AlgoHeight3Output();
		refreshAlgoH3RoiBoxes();
		applyAlgoH3Output(_algoH3Output);
		updateAlgoH3Display();
		updateAlgoH3Enables(); //a map is loaded now, so Preprocess and Segmentation can run
		AuditLog::instance().log(QStringLiteral("ALGO_H3_USE_LAST_SCAN"));
	});

	// ── section 1: preprocessing ──
	connect(ui.comboBox_algoH3PreprocessMethod, QOverload<int>::of(&QComboBox::currentIndexChanged),
		this, [=](int index) {
			//one parameter page per method, in the same order as the combo
			if (index >= 0 && index < ui.stackedWidget_algoH3Preprocess->count())
				ui.stackedWidget_algoH3Preprocess->setCurrentIndex(index);
		});

	connect(ui.toolButton_algoH3PreprocessRun, &QToolButton::clicked, this, [=]() {
		algoH3RunStage(AlgoH3Stage::Preprocess);
	});

	// ── section 2: segmentation ──
	connect(ui.toolButton_algoH3SegRun, &QToolButton::clicked, this, [=]() {
		algoH3RunStage(AlgoH3Stage::Segment);
	});

	// ── section 3: datum plane ──
	connect(ui.toolButton_algoH3DatumAddRoi, &QToolButton::clicked, this, [=]() {
		if (!AlgoManager::instance().height3SegmentReady()) {
			showMsg("Run segmentation first - a datum ROI is positioned relative to the segmented part.");
			return;
		}
		const int n = _algoH3DatumBoxes.size();
		const QSize crop = AlgoManager::instance().height3CropSize();
		//stagger new boxes so several added in a row do not stack invisibly
		const QRectF sceneRect(crop.width() / 2.0 - 60 + n * 24, crop.height() / 2.0 - 60 + n * 24, 120, 120);

		auto* box = makeAlgoH3Box(sceneRect, kAlgoH3DatumColor, QStringLiteral("Datum %1").arg(n + 1));
		_algoH3DatumBoxes.append(box);
		updateAlgoH3RoiVisibility();
		algoSettingsTouched();
		ui.lineEdit_algoH3DatumRoiCount->setText(QString::number(_algoH3DatumBoxes.size()));
	});

	connect(ui.toolButton_algoH3DatumDeleteRoi, &QToolButton::clicked, this, [=]() {
		bool removed = false;
		for (int i = _algoH3DatumBoxes.size() - 1; i >= 0; i--) {
			if (!_algoH3DatumBoxes[i]->isSelected()) continue;
			_pGraphicsSceneFOV->removeItem(_algoH3DatumBoxes[i]);
			delete _algoH3DatumBoxes[i];
			_algoH3DatumBoxes.removeAt(i);
			removed = true;
		}
		if (!removed) {
			showMsg("Click a datum ROI on the image first, then Delete Selected Datum ROI.");
			return;
		}
		for (int i = 0; i < _algoH3DatumBoxes.size(); i++)
			_algoH3DatumBoxes[i]->setName(QStringLiteral("Datum %1").arg(i + 1));

		ui.lineEdit_algoH3DatumRoiCount->setText(QString::number(_algoH3DatumBoxes.size()));
		algoSettingsTouched();
	});

	connect(ui.toolButton_algoH3DatumRun, &QToolButton::clicked, this, [=]() {
		algoH3RunStage(AlgoH3Stage::Datum);
	});

	// ── section 4: height measurement settings ──
	connect(ui.comboBox_algoH3Method, QOverload<int>::of(&QComboBox::currentIndexChanged),
		this, [=](int index) {
			if (index >= 0 && index < ui.stackedWidget_algoH3Method->count())
				ui.stackedWidget_algoH3Method->setCurrentIndex(index);
		});

	// ── section 5: ROI types and ROIs ──
	connect(ui.toolButton_algoH3RoiAddType, &QToolButton::clicked, this, [=]() {
		bool ok = false;
		const QString name = QInputDialog::getText(this, "Add ROI Type",
			"Type name:", QLineEdit::Normal, QString(), &ok).trimmed();
		if (!ok || name.isEmpty()) return;

		//capture first: the table's cell widgets are the truth for the existing types,
		//and appending to a stale copy would throw away whatever was just edited
		captureAlgoH3ParamsFromUI();
		AlgoHeight3Params p = AlgoManager::instance().height3Params();

		//the name IS the key that ties an ROI to its criteria, so it has to be unique
		//and it is never editable afterwards
		if (p.hasType(name)) {
			showMsg(QStringLiteral("An ROI type called '%1' already exists.").arg(name));
			return;
		}

		AlgoH3RoiType t;
		t.name = name;
		//walk a small palette so consecutive types are visually distinct by default
		static const QColor palette[] = { QColor(0,200,0), QColor(0,150,255), QColor(255,215,0),
			QColor(255,0,255), QColor(0,255,255), QColor(255,120,0) };
		t.color = palette[p.roiTypes.size() % 6];
		p.roiTypes.append(t);
		AlgoManager::instance().setHeight3Params(p);

		refreshAlgoH3TypeTable();
		ui.tableWidget_algoH3RoiTypes->selectRow(p.roiTypes.size() - 1);
		algoSettingsTouched();
	});

	connect(ui.toolButton_algoH3RoiDeleteType, &QToolButton::clicked, this, [=]() {
		auto* tbl = ui.tableWidget_algoH3RoiTypes;
		const int row = tbl->currentRow();
		if (row < 0 || row >= tbl->rowCount() || !tbl->item(row, 0)) {
			showMsg("Select an ROI type row first.");
			return;
		}
		const QString name = tbl->item(row, 0)->text();

		//count the ROIs that would go with it - deleting a type has to take its ROIs too,
		//otherwise the recipe would carry ROIs with no criteria and no method
		int owned = 0;
		for (auto* b : _algoH3RoiBoxes) if (b->getTag() == name) owned++;

		if (owned > 0) {
			const auto reply = QMessageBox::warning(this, "Delete ROI Type",
				QStringLiteral("%1 ROI(s) still use the type '%2'.\n\n"
					"Deleting the type deletes those ROIs as well. Continue?").arg(owned).arg(name),
				QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
			if (reply != QMessageBox::Yes) return;
		}

		captureAlgoH3ParamsFromUI();

		for (int i = _algoH3RoiBoxes.size() - 1; i >= 0; i--) {
			if (_algoH3RoiBoxes[i]->getTag() != name) continue;
			_pGraphicsSceneFOV->removeItem(_algoH3RoiBoxes[i]);
			delete _algoH3RoiBoxes[i];
			_algoH3RoiBoxes.removeAt(i);
		}

		AlgoHeight3Params p = AlgoManager::instance().height3Params();
		const int idx = p.indexOfType(name);
		if (idx >= 0) p.roiTypes.remove(idx);
		for (int i = p.rois.size() - 1; i >= 0; i--)
			if (p.rois[i].typeName == name) p.rois.remove(i);
		AlgoManager::instance().setHeight3Params(p);

		refreshAlgoH3TypeTable();
		//ROI ids shift when one is removed, so the boxes have to be renamed
		for (int i = 0; i < _algoH3RoiBoxes.size(); i++)
			_algoH3RoiBoxes[i]->setName(QStringLiteral("R%1 %2").arg(i + 1).arg(_algoH3RoiBoxes[i]->getTag()));

		ui.lineEdit_algoH3RoiCount->setText(QString::number(_algoH3RoiBoxes.size()));
		_algoH3Output.roiResults.clear();
		refreshAlgoH3ResultSection();
		algoSettingsTouched();
	});

	connect(ui.toolButton_algoH3RoiAdd, &QToolButton::clicked, this, [=]() {
		if (!AlgoManager::instance().height3SegmentReady()) {
			showMsg("Run segmentation first - an ROI is positioned relative to the segmented part.");
			return;
		}
		auto* tbl = ui.tableWidget_algoH3RoiTypes;
		const int row = tbl->currentRow();
		if (row < 0 || row >= tbl->rowCount() || !tbl->item(row, 0)) {
			//an ROI with no type would have no criteria and no method, so there is
			//nothing sensible to do with it - require the type up front
			showMsg("Select exactly one ROI type first - a new ROI is created with that type.");
			return;
		}

		const QString typeName = tbl->item(row, 0)->text();
		QColor color(0, 200, 0);
		if (auto* btn = qobject_cast<QToolButton*>(tbl->cellWidget(row, 1))) {
			const QVariant v = btn->property(kH3ColorProp);
			if (v.canConvert<QColor>()) color = v.value<QColor>();
		}

		const int n = _algoH3RoiBoxes.size();
		const QSize crop = AlgoManager::instance().height3CropSize();
		const QRectF sceneRect(crop.width() / 2.0 - 30 + n * 18, crop.height() / 2.0 - 30 + n * 18, 60, 60);

		auto* box = makeAlgoH3Box(sceneRect, color, QStringLiteral("R%1 %2").arg(n + 1).arg(typeName));
		box->setTag(typeName);
		_algoH3RoiBoxes.append(box);

		ui.lineEdit_algoH3RoiCount->setText(QString::number(_algoH3RoiBoxes.size()));
		updateAlgoH3RoiVisibility();
		algoSettingsTouched();
	});

	connect(ui.toolButton_algoH3RoiDelete, &QToolButton::clicked, this, [=]() {
		bool removed = false;
		for (int i = _algoH3RoiBoxes.size() - 1; i >= 0; i--) {
			if (!_algoH3RoiBoxes[i]->isSelected()) continue;
			_pGraphicsSceneFOV->removeItem(_algoH3RoiBoxes[i]);
			delete _algoH3RoiBoxes[i];
			_algoH3RoiBoxes.removeAt(i);
			removed = true;
		}
		if (!removed) {
			showMsg("Click one or more ROIs on the image first, then Delete Selected ROI.");
			return;
		}
		for (int i = 0; i < _algoH3RoiBoxes.size(); i++)
			_algoH3RoiBoxes[i]->setName(QStringLiteral("R%1 %2").arg(i + 1).arg(_algoH3RoiBoxes[i]->getTag()));

		ui.lineEdit_algoH3RoiCount->setText(QString::number(_algoH3RoiBoxes.size()));
		_algoH3Output.roiResults.clear();
		refreshAlgoH3ResultSection();
		algoSettingsTouched();
	});

	connect(ui.toolButton_algoH3RoiAssignType, &QToolButton::clicked, this, [=]() {
		auto* tbl = ui.tableWidget_algoH3RoiTypes;
		const int row = tbl->currentRow();
		if (row < 0 || row >= tbl->rowCount() || !tbl->item(row, 0)) {
			showMsg("Select the ROI type to assign to first.");
			return;
		}
		const QString typeName = tbl->item(row, 0)->text();
		QColor color(0, 200, 0);
		if (auto* btn = qobject_cast<QToolButton*>(tbl->cellWidget(row, 1))) {
			const QVariant v = btn->property(kH3ColorProp);
			if (v.canConvert<QColor>()) color = v.value<QColor>();
		}

		int changed = 0;
		for (int i = 0; i < _algoH3RoiBoxes.size(); i++) {
			auto* b = _algoH3RoiBoxes[i];
			if (!b->isSelected()) continue;
			b->setTag(typeName);
			b->setBorderColor(color);
			b->setName(QStringLiteral("R%1 %2").arg(i + 1).arg(typeName));
			b->update();
			changed++;
		}

		if (changed == 0) {
			showMsg("Select one or more ROIs on the image first.");
			return;
		}
		showStatus(QStringLiteral("%1 ROI(s) assigned to '%2'").arg(changed).arg(typeName));
		algoSettingsTouched();
	});

	// ── section 6: measurement results ──
	connect(ui.toolButton_algoH3ResultRun, &QToolButton::clicked, this, [=]() {
		algoH3RunStage(AlgoH3Stage::Measure);
	});

	//a swatch, not a control - the colour is set on the type, not on the individual ROI
	ui.toolButton_algoH3ResultColor->setToolTip("The colour of this ROI's type (set it in ROI Types & Criteria)");

	/*
	* Which ROI the results section is describing follows the scene selection. QDragBox
	* has no "selection changed" signal, so this polls - the same 250 ms watcher pattern
	* the V1 page already uses for its duplicate/diff tools - and only while section 6 is
	* actually open, so it costs nothing the rest of the time.
	*/
	_algoH3SelTimer = new QTimer(this);
	connect(_algoH3SelTimer, &QTimer::timeout, this, [=]() {
		if (!isPage(UIPage::ALGO_SETUP)) return;
		if (currentAlgoPageAlgo() != AlgoPageAlgo::HEIGHT_3D_V3) return;
		if (algoH3CurrentSection() != SEC_RESULT) return;

		//only repaint when the selection actually changed - this ticks four times a
		//second and the section is a dozen line edits
		int selected = -1, count = 0;
		for (int i = 0; i < _algoH3RoiBoxes.size(); i++) {
			if (!_algoH3RoiBoxes[i] || !_algoH3RoiBoxes[i]->isSelected()) continue;
			if (selected < 0) selected = i;
			count++;
		}
		const int now = (count == 1) ? selected : ((count == 0) ? -1 : -2);
		if (now == _algoH3ShownRoi) return;

		refreshAlgoH3ResultSection();
	});
	_algoH3SelTimer->start(250);

	// ── section 7: overall result ──
	connect(ui.toolButton_algoH3OverallRun, &QToolButton::clicked, this, [=]() {
		algoH3RunStage(AlgoH3Stage::Overall);
	});

	// ── results from the worker thread ──
	connect(&AlgoManager::instance(), &AlgoManager::height3Finished, this,
		[=](int stage, const AlgoHeight3Output& out) {
			_algoH3Output = out;

			//boxes first: a stage that changed the crop moves every ROI, and the results
			//section reads the box list, so it has to be rebuilt before anything is shown
			refreshAlgoH3RoiBoxes();
			applyAlgoH3Output(out);
			updateAlgoH3Display();
			updateAlgoH3RoiVisibility();
			updateAlgoH3Enables();

			QString status;
			switch ((AlgoH3Stage)stage) {
			case AlgoH3Stage::Preprocess: status = out.preprocess.pass ? "Preprocessing done" : "Preprocessing failed: " + out.preprocess.failReason; break;
			case AlgoH3Stage::Segment:    status = out.segment.pass ? "Segmentation done" : "Segmentation failed: " + out.segment.failReason; break;
			case AlgoH3Stage::Datum:      status = out.datum.pass ? "Plane fitted" : "Plane fit failed: " + out.datum.failReason; break;
			case AlgoH3Stage::Measure:    status = out.measure.pass ? QStringLiteral("Measured %1 ROI(s)").arg(out.roiResults.size()) : "Measurement failed: " + out.measure.failReason; break;
			case AlgoH3Stage::Overall:    status = out.overall.ran ? (out.overallPass ? "Overall: PASS" : "Overall: FAIL - " + out.overall.failReason) : QString(); break;
			case AlgoH3Stage::All:
			default:
				//a Run All stops at the first stage that fails, so report that one
				if (!out.preprocess.pass) status = "Preprocessing failed: " + out.preprocess.failReason;
				else if (!out.segment.pass) status = "Segmentation failed: " + out.segment.failReason;
				else if (!out.datum.pass) status = "Plane fit failed: " + out.datum.failReason;
				else if (!out.measure.pass) status = "Measurement failed: " + out.measure.failReason;
				else status = out.overallPass ? "Overall: PASS" : "Overall: FAIL - " + out.overall.failReason;
				break;
			}
			ui.label_algoStatus->setText(status);
		});
}

/*
* Spin box limits deliberately live here and not in the .ui: Qt Designer's defaults are
* 0..99.99 / 0..99 on every one of them, which is wrong for a micron and absurd for a raw
* 16-bit value, and keeping them in code means the limits sit beside the algorithm that
* has to honour them.
*/
void VisionApp::configureAlgoH3Ranges()
{
	/*
	* Signals blocked while the limits are applied. setRange() CLAMPS the current value,
	* and a clamp would emit valueChanged -> algoSettingsTouched -> a debounced save of
	* the clamped value. This runs at startup, before any recipe has been read into the
	* widgets, so that save would be writing defaults over a real recipe.
	*/
	auto setD = [](QDoubleSpinBox* sb, double lo, double hi, int decimals) {
		if (!sb) return;
		QSignalBlocker block(sb);
		sb->setDecimals(decimals);
		sb->setRange(lo, hi);
		sb->setKeyboardTracking(false); //otherwise every keystroke fires a save
	};
	auto setI = [](QSpinBox* sb, int lo, int hi) {
		if (!sb) return;
		QSignalBlocker block(sb);
		sb->setRange(lo, hi);
		sb->setKeyboardTracking(false);
	};

	// ── section 0 ──
	setD(ui.doubleSpinBox_algoH3InputXScaleUmPx, 0.0001, 10000.0, 4);
	setD(ui.doubleSpinBox_algoH3InputYScaleUmPx, 0.0001, 10000.0, 4);
	setD(ui.doubleSpinBox_algoH3InputZScaleIntensityUm, 0.0001, 10000.0, 4);
	//minimum 1, never 0: raw 0 is the profiler's "no measurement" marker, and letting it
	//count as a valid height is how a dropout becomes a datum point
	setI(ui.spinBox_algoH3InputMinValidRaw, 1, 65535);
	setI(ui.spinBox_algoH3InputMaxValidRaw, 1, 65535);
	setD(ui.doubleSpinBox_algoH3InputMinValidHeightUm, -1000000.0, 1000000.0, 2);
	setD(ui.doubleSpinBox_algoH3InputMaxValidHeightUm, -1000000.0, 1000000.0, 2);

	// ── section 1 ──
	setI(ui.spinBox_algoH3PreprocessMedianKernelSize, 3, 5);   //OpenCV's 16-bit median limit
	setI(ui.spinBox_algoH3PreprocessGaussianKernelSize, 3, 99);
	setD(ui.doubleSpinBox_algoH3PreprocessGaussianSigma, 0.0, 100.0, 2);
	setI(ui.spinBox_algoH3PreprocessBilateralDiameter, 1, 31);
	setD(ui.doubleSpinBox_algoH3PreprocessBilateralSpatialSigma, 0.1, 500.0, 2);
	setD(ui.doubleSpinBox_algoH3PreprocessBilateralHeightSigma, 0.1, 65535.0, 2);
	setI(ui.spinBox_algoH3PreprocessOpeningKernelSize, 3, 99);
	setI(ui.spinBox_algoH3PreprocessClosingKernelSize, 3, 99);

	// ── section 2 ──
	setD(ui.doubleSpinBox_algoH3SegMinWidthUm, 0.0, 10000000.0, 2);
	setD(ui.doubleSpinBox_algoH3SegMaxWidthUm, 0.0, 10000000.0, 2);
	setD(ui.doubleSpinBox_algoH3SegMinHeightUm, 0.0, 10000000.0, 2);
	setD(ui.doubleSpinBox_algoH3SegMaxHeightUm, 0.0, 10000000.0, 2);
	//the reported angle is folded into (-45, 45], so the limits match that range
	setD(ui.doubleSpinBox_algoH3SegMinAngleDeg, -45.0, 45.0, 3);
	setD(ui.doubleSpinBox_algoH3SegMaxAngleDeg, -45.0, 45.0, 3);

	// ── section 3 ──
	setD(ui.doubleSpinBox_algoH3DatumMaxTiltDeg, 0.0, 90.0, 3);

	// ── section 4 ──
	setD(ui.doubleSpinBox_algoH3MethodPercentileValue, 0.0, 100.0, 2);

	// ── section 7 ──
	setI(ui.spinBox_algoH3OverallMaxCount, 0, 1000000);
	setD(ui.doubleSpinBox_algoH3OverallMaxRate, 0.0, 100.0, 2);
}

// =============================================================================
// ROI boxes
// =============================================================================

QDragBox* VisionApp::makeAlgoH3Box(const QRectF& sceneRect, const QColor& color, const QString& name)
{
	auto* box = new QDragBox();
	_pGraphicsSceneFOV->addItem(box);

	/*
	* The barrier is the CROP, never "whatever happens to be on screen".
	*
	* These boxes live in part-frame coordinates, so the crop rect IS the space they are
	* confined to. Taking it from the scene rect instead meant that rebuilding the boxes
	* while a 3D view was showing handed every box the 1200x900 projection canvas as its
	* barrier - and QDragBox::itemChange clamps to that barrier, so every ROI at a large
	* x/y was silently dragged toward the origin and capture then wrote the wreckage back
	* into the recipe. Pressing Run in any stage while in a 3D view destroyed the teach.
	*/
	const QRectF barrier = (_algoH3BoxCropW > 0 && _algoH3BoxCropH > 0)
		? QRectF(0, 0, _algoH3BoxCropW, _algoH3BoxCropH)
		: _pGraphicsSceneFOV->sceneRect();
	box->setOutterBarrier(barrier);

	box->setup(sceneRect, color, name);
	box->setDragable(true);
	box->setZValue((int)UIHierarchy::DRAGGABLES);
	box->hide();
	connect(box, SIGNAL(dragBoxMouseReleased(QDragBox*, QString, QPointF)), this, SLOT(algoSettingsTouched()));
	connect(box, SIGNAL(grabberReleased(QDragBox*)), this, SLOT(algoSettingsTouched()));
	//a moved or resized box has to take its result label with it, or the number is left
	//floating over where the ROI used to be
	connect(box, SIGNAL(dragBoxMouseReleased(QDragBox*, QString, QPointF)), this, SLOT(refreshAlgoH3Overlay()));
	connect(box, SIGNAL(grabberReleased(QDragBox*)), this, SLOT(refreshAlgoH3Overlay()));
	return box;
}

/*
* Rebuild every ROI box from the stored part-frame geometry, against the crop that is
* currently on screen. Called after any run that could have changed the crop, and after
* a recipe load.
*
* _algoH3BoxCropW/H is the contract with capture: while it is non-zero the boxes are in
* part-frame space and their positions may be written back; while it is zero they are
* not, and capture must leave the recipe's geometry alone.
*/
void VisionApp::refreshAlgoH3RoiBoxes()
{
	auto& mgr = AlgoManager::instance();
	const AlgoHeight3Params p = mgr.height3Params();

	auto dropBoxes = [&](QVector<QDragBox*>& boxes) {
		for (auto* b : boxes) {
			if (!b) continue;
			_pGraphicsSceneFOV->removeItem(b);
			delete b;
		}
		boxes.clear();
	};

	dropBoxes(_algoH3DatumBoxes);
	dropBoxes(_algoH3RoiBoxes);

	const QSize crop = mgr.height3SegmentReady() ? mgr.height3CropSize() : QSize();
	if (crop.width() <= 0 || crop.height() <= 0) {
		//no crop, so no part frame, so nothing can be placed. The recipe still holds the
		//ROIs; they come back the moment segmentation succeeds again.
		_algoH3BoxCropW = 0;
		_algoH3BoxCropH = 0;
		ui.lineEdit_algoH3DatumRoiCount->setText(QString::number(p.datumRois.size()));
		ui.lineEdit_algoH3RoiCount->setText(QString::number(p.rois.size()));
		return;
	}

	_algoH3BoxCropW = crop.width();
	_algoH3BoxCropH = crop.height();
	const double cx = crop.width() / 2.0;
	const double cy = crop.height() / 2.0;

	for (int i = 0; i < p.datumRois.size(); i++) {
		auto* b = makeAlgoH3Box(p.datumRois[i].translated(cx, cy), kAlgoH3DatumColor,
			QStringLiteral("Datum %1").arg(i + 1));
		_algoH3DatumBoxes.append(b);
	}

	for (int i = 0; i < p.rois.size(); i++) {
		const AlgoH3Roi& roi = p.rois[i];
		QColor color(0, 200, 0);
		const int ti = p.indexOfType(roi.typeName);
		if (ti >= 0) color = p.roiTypes[ti].color;

		auto* b = makeAlgoH3Box(roi.rel.translated(cx, cy), color,
			QStringLiteral("R%1 %2").arg(i + 1).arg(roi.typeName));
		b->setTag(roi.typeName);
		_algoH3RoiBoxes.append(b);
	}

	ui.lineEdit_algoH3DatumRoiCount->setText(QString::number(_algoH3DatumBoxes.size()));
	ui.lineEdit_algoH3RoiCount->setText(QString::number(_algoH3RoiBoxes.size()));
	updateAlgoH3RoiVisibility();
}

/*
* Ctrl+C / Ctrl+V for V3's ROIs. Both are gated on the OPEN SECTION, which is the page's
* own rule: section 3 owns the datum ROIs, 5 and 6 own the measurement ROIs, and no other
* section owns either. So a copy can only take the kind the section owns, and a paste is
* refused outright anywhere that kind could not be shown - otherwise the ROIs are created
* perfectly correctly somewhere the operator cannot see them, the count ticks up, and
* nothing appears on the image.
*
* The clipboard holds part-frame geometry, not scene geometry, so a copy taken before a
* re-segmentation still pastes to the same place on the part afterwards.
*
* Selection is read with Qt's isSelected(), never QDragBox::getSelected() - see the note
* above algoHCopySelectedRois() in VisionApp_AlgoSetup.cpp for why that matters.
*/
void VisionApp::algoH3CopySelectedRois()
{
	if (_algoH3BoxCropW <= 0 || _algoH3BoxCropH <= 0) {
		showMsg("Run segmentation first - an ROI only exists relative to the segmented part.");
		return;
	}

	const H3RoiOwner owner = h3SectionOwner(algoH3CurrentSection());
	if (owner == H3RoiOwner::None) {
		showMsg(QStringLiteral("This section has no ROIs to copy. %1").arg(kH3RoiSectionHint));
		return;
	}

	const double cx = _algoH3BoxCropW / 2.0;
	const double cy = _algoH3BoxCropH / 2.0;

	_algoH3Clipboard.clear();

	//one kind only, decided by the section rather than inferred from which boxes happen to
	//be visible - that keeps the clipboard homogeneous, which paste relies on
	if (owner == H3RoiOwner::Datum) {
		for (auto* b : _algoH3DatumBoxes) {
			if (!b || !b->isVisible() || !b->isSelected()) continue;
			AlgoH3ClipRoi c;
			c.datum = true;
			c.rel = b->getGeometry().translated(-cx, -cy);
			_algoH3Clipboard.append(c);
		}
	}
	else {
		for (auto* b : _algoH3RoiBoxes) {
			if (!b || !b->isVisible() || !b->isSelected()) continue;
			AlgoH3ClipRoi c;
			c.datum = false;
			c.typeName = b->getTag();
			c.rel = b->getGeometry().translated(-cx, -cy);
			_algoH3Clipboard.append(c);
		}
	}

	if (_algoH3Clipboard.isEmpty()) {
		showMsg(owner == H3RoiOwner::Datum
			? "Click one or more datum ROIs on the image first, then Ctrl+C."
			: "Click one or more measurement ROIs on the image first, then Ctrl+C.");
		return;
	}
	//a fresh clipboard starts the paste offset over, so the first paste of a new copy lands
	//10 px off its own original rather than wherever the last run of pastes had got to
	_algoH3PasteCount = 0;
	showStatus(QStringLiteral("%1 ROI(s) copied").arg(_algoH3Clipboard.size()));
}

void VisionApp::algoH3PasteRois()
{
	if (_algoH3Clipboard.isEmpty()) return;

	if (_algoH3BoxCropW <= 0 || _algoH3BoxCropH <= 0) {
		showMsg("Run segmentation first - an ROI only exists relative to the segmented part.");
		return;
	}

	/*
	* Section checks BEFORE anything else, so a refused paste is a true no-op: no capture,
	* no bump of the paste offset, no clearing of the current selection.
	*/
	const H3RoiOwner owner = h3SectionOwner(algoH3CurrentSection());
	if (owner == H3RoiOwner::None) {
		showMsg(QStringLiteral("Nothing here can show a pasted ROI. %1").arg(kH3RoiSectionHint));
		return;
	}
	const bool wantDatum = (owner == H3RoiOwner::Datum);

	//copy takes one kind at a time, so the clipboard is homogeneous and its first entry
	//speaks for all of it
	if (_algoH3Clipboard.first().datum != wantDatum) {
		showMsg(wantDatum
			? "The clipboard holds measurement ROIs - open ROI Types & Criteria or Measurement Results to paste them."
			: "The clipboard holds datum ROIs - open the Datum Plane section to paste them.");
		return;
	}

	//the type table is the truth for colours and for which types still exist, so push it
	//down before reading any of it back
	captureAlgoH3ParamsFromUI();
	const AlgoHeight3Params p = AlgoManager::instance().height3Params();

	const double cx = _algoH3BoxCropW / 2.0;
	const double cy = _algoH3BoxCropH / 2.0;

	/*
	* The offset STEPS with each paste of the same clipboard. It has to: the clipboard holds
	* the original geometry, so a fixed 10 px would put the second Ctrl+V exactly on top of
	* the first and the operator would be stacking invisible duplicates. Counting pastes
	* instead of mutating the clipboard keeps the clipboard meaning what was copied.
	*/
	_algoH3PasteCount++;
	const double step = 10.0 * _algoH3PasteCount;

	//the paste owns the selection when it finishes, so clear what was selected first -
	//otherwise the sources stay selected too and the next Ctrl+C would copy six ROIs
	//when the operator can only see three highlighted
	for (auto* b : _algoH3DatumBoxes) if (b) b->setSelected(false);
	for (auto* b : _algoH3RoiBoxes) if (b) b->setSelected(false);

	int pasted = 0;
	int skipped = 0;
	QVector<QDragBox*> fresh; //selected at the end - see the note below

	for (const auto& c : _algoH3Clipboard) {
		//offset so a copy is visibly separate from its original, the same gesture the V1
		//height page uses
		const QRectF rel = c.rel.translated(step, step);

		if (c.datum) {
			auto* b = makeAlgoH3Box(rel.translated(cx, cy), kAlgoH3DatumColor,
				QStringLiteral("Datum %1").arg(_algoH3DatumBoxes.size() + 1));
			_algoH3DatumBoxes.append(b);
			fresh.append(b);
			pasted++;
			continue;
		}

		//its type may have been deleted between the copy and the paste; an ROI with no
		//type has no criteria and no method, so skip it rather than invent one
		const int ti = p.indexOfType(c.typeName);
		if (ti < 0) { skipped++; continue; }

		auto* b = makeAlgoH3Box(rel.translated(cx, cy), p.roiTypes[ti].color,
			QStringLiteral("R%1 %2").arg(_algoH3RoiBoxes.size() + 1).arg(c.typeName));
		b->setTag(c.typeName);
		_algoH3RoiBoxes.append(b);
		fresh.append(b);
		pasted++;
	}

	ui.lineEdit_algoH3DatumRoiCount->setText(QString::number(_algoH3DatumBoxes.size()));
	ui.lineEdit_algoH3RoiCount->setText(QString::number(_algoH3RoiBoxes.size()));

	/*
	* VISIBILITY FIRST, THEN SELECTION, and the order is not cosmetic:
	* QGraphicsItem::setSelected() DOES NOTHING when the item is not visible, and
	* makeAlgoH3Box() creates every box hidden. Selecting inside the loop above therefore
	* gets silently discarded - the paste looked like it cleared the selection instead of
	* moving it. Anything that selects a box it just created has to show it first.
	*/
	updateAlgoH3RoiVisibility();
	for (auto* b : fresh) if (b) b->setSelected(true);

	//a paste adds ROIs the last measurement knows nothing about, so its per-ROI numbers
	//no longer line up with the ROI ids on screen
	_algoH3Output.roiResults.clear();
	refreshAlgoH3ResultSection();

	algoSettingsTouched();

	if (skipped > 0) {
		showMsg(QStringLiteral("%1 ROI(s) pasted. %2 skipped - their ROI type no longer exists.")
			.arg(pasted).arg(skipped));
	}
	else {
		showStatus(QStringLiteral("%1 ROI(s) pasted").arg(pasted));
	}
}

void VisionApp::hideAlgoH3Rois()
{
	for (auto* b : _algoH3DatumBoxes) if (b) b->hide();
	for (auto* b : _algoH3RoiBoxes) if (b) b->hide();
}

/*
* One section owns the image at a time. Datum ROIs belong to section 3 and measurement
* ROIs to sections 5 and 6 - 6 as well as 5 because that is where an ROI is picked to
* read its result, and a section that asks you to select an ROI has to show them.
*/
void VisionApp::updateAlgoH3RoiVisibility()
{
	const bool onPage = isPage(UIPage::ALGO_SETUP) && ui.frame_rightTab->isVisible()
		&& (currentAlgoPageAlgo() == AlgoPageAlgo::HEIGHT_3D_V3);

	//the 3D view is a projection - an ROI dragged on it would not mean anything
	const bool flatView = !h3IsSurfaceMode(algoH3DisplayMode());
	const bool cropOnScreen = onPage && flatView && (_algoH3BoxCropW > 0);
	const int section = algoH3CurrentSection();

	const bool showDatum = cropOnScreen && (section == SEC_DATUM);
	const bool showRois = cropOnScreen && (section == SEC_ROI || section == SEC_RESULT);

	for (auto* b : _algoH3DatumBoxes) if (b) b->setVisible(showDatum);
	for (auto* b : _algoH3RoiBoxes) if (b) b->setVisible(showRois);
}

// =============================================================================
// Display
// =============================================================================

int VisionApp::algoH3CurrentSection() const
{
	return ui.toolBox_algoH3Sections ? ui.toolBox_algoH3Sections->currentIndex() : 0;
}

/*
* An out-of-range index can only mean the combo and AlgoH3Display have drifted apart, so
* fall back to the height view rather than pick a mode the operator did not ask for.
*/
AlgoH3Display VisionApp::algoH3DisplayMode() const
{
	if (!ui.comboBox_algoH3Display) return AlgoH3Display::HeightColor;

	const int i = ui.comboBox_algoH3Display->currentIndex();
	if (i < static_cast<int>(AlgoH3Display::HeightColor)
		|| i >= kAlgoH3DisplayCount) {
		return AlgoH3Display::HeightColor;
	}
	return static_cast<AlgoH3Display>(i);
}

void VisionApp::updateAlgoH3Display()
{
	if (!isPage(UIPage::ALGO_SETUP)) return;
	if (currentAlgoPageAlgo() != AlgoPageAlgo::HEIGHT_3D_V3) return;

	auto& mgr = AlgoManager::instance();
	const int section = algoH3CurrentSection();

	//section 0 looks at the raw map; 1 onwards look at the FILTERED one - the preprocessing
	//section is where the filter and kernel are chosen, so it is the one place the cleaned
	//result has to be visible to judge them; 3 onwards look at the straightened crop.
	//Each only takes effect once that map exists: heightForDisplay falls back to the raw
	//map while m_work is empty, so nothing special is needed before the stage has run.
	//KEEP IN SYNC with updateAlgoH3Surface() - the 2D and 3D views must show the same map.
	const bool preprocessed = (section >= SEC_PREPROCESS);
	const bool segmented = (section >= SEC_DATUM) && mgr.height3SegmentReady();

	const AlgoH3Display mode = algoH3DisplayMode();
	if (!h3IsSurfaceMode(mode)) _algoH3Dragging = false;

	QImage img;
	switch (mode) {
	//every 3D mode is the same projection; only the paint style differs
	case AlgoH3Display::Surface3D:
	case AlgoH3Display::Mesh3D:
	case AlgoH3Display::Smooth3D:
	case AlgoH3Display::Wireframe3D:
	case AlgoH3Display::PointCloud3D:
	case AlgoH3Display::Textured3D:
		img = mgr.height3Surface(preprocessed, segmented, _algoH3Yaw, _algoH3Pitch,
			_algoH3ZExaggeration, kAlgoH3SurfaceCanvas, h3StyleFor(mode));
		break;

	//lit but flat, and rendered at full resolution - so unlike the 3D views this one is
	//still a map: the ROI boxes and overlays sit on it exactly where they belong
	case AlgoH3Display::Relief2D:
		img = mgr.height3Relief(preprocessed, segmented, _algoH3ZExaggeration, true);
		break;

	case AlgoH3Display::Intensity:
		img = mgr.height3Image(true, preprocessed, segmented, false);
		if (img.isNull() && mgr.height3HasHeight() && !mgr.height3HasIntensity())
			showStatus("No intensity map loaded - load one to view it.");
		break;

	//both height modes read the same map and differ only in how it is painted:
	//the JET ramp makes small steps obvious, grey keeps the surface readable
	case AlgoH3Display::HeightColor:
	case AlgoH3Display::HeightGray:
	default:
		img = mgr.height3Image(false, preprocessed, segmented,
			mode == AlgoH3Display::HeightColor);
		break;
	}

	if (img.isNull()) return; //nothing loaded yet, or a stage is running - keep the view

	const bool sizeChanged = (_pixmapFOV.width() != img.width() || _pixmapFOV.height() != img.height());
	displayFOV(img);

	//the scene rect moved with the image, so the boxes' movement limits must follow it
	for (auto* b : _algoH3DatumBoxes) if (b) b->setOutterBarrier(_pGraphicsSceneFOV->sceneRect());
	for (auto* b : _algoH3RoiBoxes) if (b) b->setOutterBarrier(_pGraphicsSceneFOV->sceneRect());

	if (sizeChanged) ui.graphicsViewFOV->fitInView(_pPixmapItemFOV, Qt::KeepAspectRatio);

	//BEFORE the overlay, not after: the ROI labels below are anchored to the boxes and read
	//their visibility to decide whether this section owns them at all
	updateAlgoH3RoiVisibility();

	refreshAlgoH3Overlay();
}

/*
* Two overlays, and never both at once because the sections that want them are different:
* segmentation draws its found rectangle on the full map so the operator can see what was
* found before deciding to trust the crop, and the measurement sections draw each ROI's
* own result over the crop.
*
* Split out of updateAlgoH3Display() so a dragged ROI can take its label with it without
* repainting the image - rebuilding a 36 MB crop into a QImage on every box release would
* be a lot of work to move a few pieces of text.
*
* renderAlgoOverlay() clears before it draws, so an empty list is how the overlay is taken
* down; that is what replaced the bare clearAlgoOverlay() call this used to make.
*/
void VisionApp::refreshAlgoH3Overlay()
{
	//guarded independently of the caller: this is a slot on every V3 box's release signal,
	//and the shared overlay list belongs to whichever algo page is actually showing
	if (!isPage(UIPage::ALGO_SETUP)) return;
	if (currentAlgoPageAlgo() != AlgoPageAlgo::HEIGHT_3D_V3) return;

	/*
	* Nothing in the overlay means anything on a 3D projection. Every item here is placed
	* in map or crop coordinates - the segmentation outline, the per-ROI labels, the
	* overall pass/fail rects - and the 3D canvas is neither of those spaces, so they would
	* land at arbitrary spots on the render. Take the overlay down instead of drawing it.
	*
	* Handled once here rather than per branch, so a future overlay item cannot forget to
	* opt out and leak onto the 3D view.
	*/
	if (h3IsSurfaceMode(algoH3DisplayMode())) {
		renderAlgoOverlay({});
		return;
	}

	const int section = algoH3CurrentSection();
	const bool segmented = (section >= SEC_DATUM) && AlgoManager::instance().height3SegmentReady();

	QVector<AlgoOverlayItem> overlay;

	if (section == SEC_SEG && !segmented && _algoH3Output.segment.ran && _algoH3Output.segment.pass
		&& _algoH3Output.segCorners.size() == 4) {
		QPolygonF poly;
		for (const auto& pt : _algoH3Output.segCorners) poly << pt;
		overlay.append(AlgoOverlayItem::makePoly(poly, QColor(0, 255, 127)));
	}
	else if (section == SEC_OVERALL && _algoH3Output.measure.ran && _algoH3BoxCropW > 0) {
		//no flat-view test needed any more: the 3D early-return above covers it
		//the overall section draws the ROIs itself, as overlay - see the note on the function
		appendAlgoH3OverallRois(overlay);
	}
	else if (_algoH3Output.measure.ran) {
		//no section test of its own: the labels are anchored to the boxes and skip a hidden
		//one, so they follow whatever updateAlgoH3RoiVisibility() already decided
		appendAlgoH3RoiLabels(overlay);
	}

	renderAlgoOverlay(overlay);
}

/*
* The overall section's view of the ROIs: PASS/FAIL only, and NOT editable.
*
* Drawn as overlay rectangles rather than by showing the QDragBoxes, which is what makes
* "not editable" true by construction - an overlay item has no grabbers, cannot be dragged,
* cannot be selected, and so cannot be copied either. Re-using the drag boxes would have
* meant stripping their movable/selectable flags and then RECOLOURING boxes that sections 3,
* 5 and 6 share, which would have destroyed the per-type colours those sections exist to
* show. Nothing has to be undone on the way out of this section either.
*
* Colour carries the whole message here: green for pass, red for fail, on both the rectangle
* and its height. The ROI type is deliberately not shown - by this section the operator has
* stopped asking which type a pin is and is only asking whether the unit passed.
*
* Geometry comes from the stored part-frame rect rather than from the boxes, because the
* boxes are hidden in this section. That is also why this branch needs its own crop and
* flat-view guards: it has no box visibility to piggyback on.
*/
void VisionApp::appendAlgoH3OverallRois(QVector<AlgoOverlayItem>& overlay) const
{
	const double cx = _algoH3BoxCropW / 2.0;
	const double cy = _algoH3BoxCropH / 2.0;

	for (const auto& r : _algoH3Output.roiResults) {
		const QColor c = r.pass ? kAlgoH3PassColor : kAlgoH3FailColor;

		//a light wash makes a failing pin findable at a glance on a part with hundreds of
		//them, without hiding the surface underneath
		QColor fill = c;
		fill.setAlpha(40);

		const QRectF rect = r.rel.translated(cx, cy);
		overlay.append(AlgoOverlayItem::makeRect(rect, c, fill));
		overlay.append(AlgoOverlayItem::makeText(h3RoiLabelText(r),
			rect.topLeft() - QPointF(0, kAlgoH3LabelOffsetPx), c, kAlgoH3LabelPointSize));
	}
}

/*
* One label per measured ROI, just outside its top-left corner: the height in microns,
* green when the ROI passed and red when it did not. Section 6 can only describe one ROI
* at a time - the right panel would be an unreadable wall of numbers otherwise - so the
* image is the only place the whole result can be taken in at once.
*
* Labels are index-matched to the boxes exactly the way the results section is, box i
* carrying ROI id i+1, so an ROI added or pasted since the last run has no result and gets
* no label rather than borrowing its neighbour's number.
*/
void VisionApp::appendAlgoH3RoiLabels(QVector<AlgoOverlayItem>& overlay) const
{
	for (int i = 0; i < _algoH3RoiBoxes.size(); i++) {
		auto* b = _algoH3RoiBoxes[i];
		//visibility is the section gate: a hidden box means this section does not own the
		//measurement ROIs, and a label with no box under it would point at nothing
		if (!b || !b->isVisible()) continue;

		const AlgoH3RoiResult* r = _algoH3Output.roiById(i + 1);
		if (!r) continue;

		//NO DATA is not a number, but it is still a verdict - and design decision 12 makes
		//it not-pass - so it reads red like any other failure rather than going blank
		overlay.append(AlgoOverlayItem::makeText(h3RoiLabelText(*r),
			b->getGeometry().topLeft() - QPointF(0, kAlgoH3LabelOffsetPx),
			r->pass ? kAlgoH3PassColor : kAlgoH3FailColor, kAlgoH3LabelPointSize));
	}
}

//re-render just the 3D surface, throttled, so a drag stays smooth without queueing frames
void VisionApp::updateAlgoH3Surface()
{
	const AlgoH3Display mode = algoH3DisplayMode();
	if (!h3IsSurfaceMode(mode)) return;

	if (_algoH3DragClock.isValid() && _algoH3DragClock.elapsed() < 40) return;
	_algoH3DragClock.restart();

	auto& mgr = AlgoManager::instance();
	const int section = algoH3CurrentSection();
	//KEEP IN SYNC with updateAlgoH3Display() - same rule, or 2D and 3D disagree
	const bool preprocessed = (section >= SEC_PREPROCESS);
	const bool segmented = (section >= SEC_DATUM) && mgr.height3SegmentReady();

	const QImage img = mgr.height3Surface(preprocessed, segmented, _algoH3Yaw, _algoH3Pitch,
		_algoH3ZExaggeration, kAlgoH3SurfaceCanvas, h3StyleFor(mode));
	if (img.isNull()) return;

	//the canvas size never changes, so the pixmap can be swapped in place - going through
	//displayFOV() would rebuild the scene and its crosshairs on every mouse move
	_pixmapFOV = QPixmap::fromImage(img);
	if (_pPixmapItemFOV) _pPixmapItemFOV->setPixmap(_pixmapFOV);
}

/*
* Drag-to-spin for the 3D surface. Installed on the FOV viewport through VisionApp's
* app-wide event filter; it only ever claims events while the V3 page is showing the 3D
* view, and returns false everywhere else so nothing about the normal FOV behaviour
* changes.
*/
bool VisionApp::algoH3HandleViewMouse(QObject* obj, QEvent* ev)
{
	if (!ui.graphicsViewFOV || obj != ui.graphicsViewFOV->viewport()) return false;
	if (!isPage(UIPage::ALGO_SETUP)) return false;
	if (currentAlgoPageAlgo() != AlgoPageAlgo::HEIGHT_3D_V3) return false;
	if (!h3IsSurfaceMode(algoH3DisplayMode())) { _algoH3Dragging = false; return false; }

	switch (ev->type()) {
	case QEvent::MouseButtonPress: {
		auto* me = static_cast<QMouseEvent*>(ev);
		if (me->button() != Qt::LeftButton) return false;
		_algoH3Dragging = true;
		_algoH3DragFrom = me->pos();
		_algoH3DragClock.start();
		return true;
	}
	case QEvent::MouseMove: {
		if (!_algoH3Dragging) return false;
		auto* me = static_cast<QMouseEvent*>(ev);
		const QPoint d = me->pos() - _algoH3DragFrom;
		_algoH3DragFrom = me->pos();

		/*
		* Drag GRABS THE OBJECT, it does not fly the camera. Pull the mouse up and the far
		* side lifts toward you (the view goes edge-on); push it down and the part lays
		* flat under you (top-down).
		*
		* pitchDeg is a camera ELEVATION - 89 is straight down, 2 is edge-on - so tilting
		* the object up means DECREASING it. Mouse-up gives a negative d.y(), so the sign
		* here is +, and it deliberately differs from the yaw line above: for yaw the two
		* conventions are indistinguishable (spinning the camera left and pushing the
		* object left look the same), for pitch they are exact opposites. Do not "fix"
		* this to match the line above.
		*/
		_algoH3Yaw = std::fmod(_algoH3Yaw - d.x() * 0.4, 360.0);
		_algoH3Pitch = std::max(2.0, std::min(89.0, _algoH3Pitch + d.y() * 0.4));
		updateAlgoH3Surface();
		return true;
	}
	case QEvent::MouseButtonRelease: {
		if (!_algoH3Dragging) return false;
		_algoH3Dragging = false;
		_algoH3DragClock.invalidate();
		updateAlgoH3Surface();
		return true;
	}
	default:
		return false;
	}
}

// =============================================================================
// Enables
// =============================================================================

void VisionApp::updateAlgoH3Enables()
{
	auto& mgr = AlgoManager::instance();
	const bool busy = mgr.isBusy();
	const bool haveHeight = mgr.height3HasHeight();
	const bool segReady = mgr.height3SegmentReady();
	const bool datumReady = mgr.height3DatumReady();
	const bool measured = _algoH3Output.measure.ran && _algoH3Output.measure.pass;

	//each Run is gated on the stage before it having actually passed, so the pipeline
	//can never be entered halfway with stale state behind it
	ui.toolButton_algoH3PreprocessRun->setEnabled(!busy && haveHeight);
	ui.toolButton_algoH3SegRun->setEnabled(!busy && haveHeight);
	ui.toolButton_algoH3DatumRun->setEnabled(!busy && segReady);
	ui.toolButton_algoH3ResultRun->setEnabled(!busy && datumReady);
	ui.toolButton_algoH3OverallRun->setEnabled(!busy && measured);

	ui.toolButton_algoH3DatumAddRoi->setEnabled(!busy && segReady);
	ui.toolButton_algoH3DatumDeleteRoi->setEnabled(!busy && segReady);
	ui.toolButton_algoH3RoiAdd->setEnabled(!busy && segReady);
	ui.toolButton_algoH3RoiDelete->setEnabled(!busy && segReady);
	ui.toolButton_algoH3RoiAssignType->setEnabled(!busy && segReady);
}

// =============================================================================
// ROI type table
// =============================================================================

void VisionApp::refreshAlgoH3TypeTable()
{
	auto* tbl = ui.tableWidget_algoH3RoiTypes;
	if (!tbl) return;

	const AlgoHeight3Params p = AlgoManager::instance().height3Params();

	_algoH3Updating = true;
	const int keepRow = tbl->currentRow();

	//setRowCount(0) first: QTableWidget owns the cell widgets, and this is what deletes
	//the previous set. clearContents() would leave them behind, connected and orphaned.
	tbl->setRowCount(0);
	tbl->setColumnCount(5);
	tbl->setHorizontalHeaderLabels({ "Type", "Color", "Min (um)", "Max (um)", "Method ID" });
	tbl->verticalHeader()->setVisible(false);
	tbl->setSelectionBehavior(QAbstractItemView::SelectRows);
	tbl->setSelectionMode(QAbstractItemView::SingleSelection);
	tbl->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
	for (int c = 1; c < 5; c++)
		tbl->horizontalHeader()->setSectionResizeMode(c, QHeaderView::ResizeToContents);

	tbl->setRowCount(p.roiTypes.size());

	for (int row = 0; row < p.roiTypes.size(); row++) {
		const AlgoH3RoiType& t = p.roiTypes[row];

		//the name is the key ROIs refer to, so it is fixed once the type is created
		auto* nameItem = new QTableWidgetItem(t.name);
		nameItem->setFlags(nameItem->flags() & ~Qt::ItemIsEditable);
		tbl->setItem(row, 0, nameItem);

		auto* colorBtn = new QToolButton();
		colorBtn->setProperty(kH3ColorProp, t.color);
		colorBtn->setMinimumWidth(48);
		colorBtn->setToolTip(QStringLiteral("Click to choose the ROI colour for '%1'").arg(t.name));
		colorBtn->setStyleSheet(QStringLiteral(
			"QToolButton { background:%1; border:1px solid #777; }").arg(t.color.name()));
		const QString typeName = t.name;
		connect(colorBtn, &QToolButton::clicked, this, [this, colorBtn, typeName]() {
			const QVariant v = colorBtn->property(kH3ColorProp);
			const QColor current = v.canConvert<QColor>() ? v.value<QColor>() : QColor(0, 200, 0);
			const QColor picked = QColorDialog::getColor(current, this, "ROI Type Colour");
			if (!picked.isValid()) return;

			colorBtn->setProperty(kH3ColorProp, picked);
			colorBtn->setStyleSheet(QStringLiteral(
				"QToolButton { background:%1; border:1px solid #777; }").arg(picked.name()));

			//recolour the ROIs of this type immediately - the colour is how the operator
			//tells one type from another on the image
			for (auto* b : _algoH3RoiBoxes) {
				if (!b || b->getTag() != typeName) continue;
				b->setBorderColor(picked);
				b->update();
			}
			algoSettingsTouched();
		});
		tbl->setCellWidget(row, 1, colorBtn);

		auto makeDouble = [&](double value) {
			auto* sb = new QDoubleSpinBox();
			sb->setDecimals(2);
			sb->setRange(-1000000.0, 1000000.0);
			sb->setKeyboardTracking(false);
			sb->setValue(value);
			connect(sb, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double) {
				if (_algoH3Updating) return;
				algoSettingsTouched();
			});
			return sb;
		};

		tbl->setCellWidget(row, 2, makeDouble(t.minUm));
		tbl->setCellWidget(row, 3, makeDouble(t.maxUm));

		auto* methodSpin = new QSpinBox();
		methodSpin->setRange(0, kAlgoH3MethodCount - 1);
		methodSpin->setKeyboardTracking(false);
		methodSpin->setValue(t.methodId);
		methodSpin->setToolTip(QStringLiteral("Method ID from the Height Measurement Settings section"));
		connect(methodSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, [this](int) {
			if (_algoH3Updating) return;
			algoSettingsTouched();
		});
		tbl->setCellWidget(row, 4, methodSpin);
	}

	if (keepRow >= 0 && keepRow < tbl->rowCount()) tbl->selectRow(keepRow);
	_algoH3Updating = false;
}

// =============================================================================
// Params round trip
// =============================================================================

void VisionApp::captureAlgoH3ParamsFromUI()
{
	auto& mgr = AlgoManager::instance();
	AlgoHeight3Params p = mgr.height3Params();

	// ── section 0 ──
	p.xScaleUmPx = ui.doubleSpinBox_algoH3InputXScaleUmPx->value();
	p.yScaleUmPx = ui.doubleSpinBox_algoH3InputYScaleUmPx->value();
	p.zScaleRawPerUm = ui.doubleSpinBox_algoH3InputZScaleIntensityUm->value();
	p.minValidRaw = ui.spinBox_algoH3InputMinValidRaw->value();
	p.maxValidRaw = ui.spinBox_algoH3InputMaxValidRaw->value();
	p.minValidHeightUm = ui.doubleSpinBox_algoH3InputMinValidHeightUm->value();
	p.maxValidHeightUm = ui.doubleSpinBox_algoH3InputMaxValidHeightUm->value();

	// ── section 1 ──
	p.preprocess = (AlgoH3Preprocess)ui.comboBox_algoH3PreprocessMethod->currentIndex();
	p.medianKernel = ui.spinBox_algoH3PreprocessMedianKernelSize->value();
	p.gaussianKernel = ui.spinBox_algoH3PreprocessGaussianKernelSize->value();
	p.gaussianSigma = ui.doubleSpinBox_algoH3PreprocessGaussianSigma->value();
	p.bilateralDiameter = ui.spinBox_algoH3PreprocessBilateralDiameter->value();
	p.bilateralSpatialSigma = ui.doubleSpinBox_algoH3PreprocessBilateralSpatialSigma->value();
	p.bilateralHeightSigma = ui.doubleSpinBox_algoH3PreprocessBilateralHeightSigma->value();
	p.openingKernel = ui.spinBox_algoH3PreprocessOpeningKernelSize->value();
	p.closingKernel = ui.spinBox_algoH3PreprocessClosingKernelSize->value();

	// ── section 2 ──
	p.segMethod = (AlgoH3SegMethod)ui.comboBox_algoH3SegMethod->currentIndex();
	p.segCheckWidth = ui.checkBox_algoH3SegEnableWidthCheck->isChecked();
	p.segMinWidthUm = ui.doubleSpinBox_algoH3SegMinWidthUm->value();
	p.segMaxWidthUm = ui.doubleSpinBox_algoH3SegMaxWidthUm->value();
	p.segCheckHeight = ui.checkBox_algoH3SegEnableHeightCheck->isChecked();
	p.segMinHeightUm = ui.doubleSpinBox_algoH3SegMinHeightUm->value();
	p.segMaxHeightUm = ui.doubleSpinBox_algoH3SegMaxHeightUm->value();
	p.segCheckAngle = ui.checkBox_algoH3SegEnableAngleCheck->isChecked();
	p.segMinAngleDeg = ui.doubleSpinBox_algoH3SegMinAngleDeg->value();
	p.segMaxAngleDeg = ui.doubleSpinBox_algoH3SegMaxAngleDeg->value();

	// ── section 3 ──
	p.datumMethod = (AlgoH3DatumMethod)ui.comboBox_algoH3DatumMethod->currentIndex();
	p.datumCheckTilt = ui.checkBox_algoH3DatumEnableTiltCheck->isChecked();
	p.datumMaxTiltDeg = ui.doubleSpinBox_algoH3DatumMaxTiltDeg->value();

	// ── section 4 ──
	p.methodId = ui.comboBox_algoH3Method->currentIndex();
	p.percentile = ui.doubleSpinBox_algoH3MethodPercentileValue->value();

	// ── section 5: types come from the table's cell widgets ──
	auto* tbl = ui.tableWidget_algoH3RoiTypes;
	if (tbl && tbl->columnCount() >= 5) {
		QVector<AlgoH3RoiType> types;
		for (int row = 0; row < tbl->rowCount(); row++) {
			auto* nameItem = tbl->item(row, 0);
			if (!nameItem || nameItem->text().isEmpty()) continue;

			AlgoH3RoiType t;
			t.name = nameItem->text();
			if (auto* btn = qobject_cast<QToolButton*>(tbl->cellWidget(row, 1))) {
				const QVariant v = btn->property(kH3ColorProp);
				if (v.canConvert<QColor>()) t.color = v.value<QColor>();
			}
			if (auto* sb = qobject_cast<QDoubleSpinBox*>(tbl->cellWidget(row, 2))) t.minUm = sb->value();
			if (auto* sb = qobject_cast<QDoubleSpinBox*>(tbl->cellWidget(row, 3))) t.maxUm = sb->value();
			if (auto* sb = qobject_cast<QSpinBox*>(tbl->cellWidget(row, 4))) t.methodId = sb->value();
			types.append(t);
		}
		p.roiTypes = types;
	}

	/*
	* ROI geometry is only readable while the boxes are laid out against a live crop.
	* Off the crop there is no part frame, so a box's scene position means nothing - and
	* writing it back would silently move every taught ROI. Keep what the recipe has.
	*/
	if (_algoH3BoxCropW > 0 && _algoH3BoxCropH > 0) {
		const double cx = _algoH3BoxCropW / 2.0;
		const double cy = _algoH3BoxCropH / 2.0;

		p.datumRois.clear();
		for (auto* b : _algoH3DatumBoxes) {
			if (!b) continue;
			p.datumRois.append(b->getGeometry().translated(-cx, -cy));
		}

		p.rois.clear();
		for (auto* b : _algoH3RoiBoxes) {
			if (!b) continue;
			AlgoH3Roi r;
			r.typeName = b->getTag();
			r.rel = b->getGeometry().translated(-cx, -cy);
			if (r.typeName.isEmpty()) continue; //never persist an ROI with no type
			p.rois.append(r);
		}
	}

	// ── section 7 ──
	p.overallCheckCount = ui.checkBox_algoH3OverallEnableCountCheck->isChecked();
	p.overallMaxCount = ui.spinBox_algoH3OverallMaxCount->value();
	p.overallCheckRate = ui.checkBox_algoH3OverallEnableRateCheck->isChecked();
	p.overallMaxRatePct = ui.doubleSpinBox_algoH3OverallMaxRate->value();

	mgr.setHeight3Params(p);
}

void VisionApp::refreshAlgoHeight3Page()
{
	const AlgoHeight3Params p = AlgoManager::instance().height3Params();

	// ── section 0 ──
	{
		QSignalBlocker b1(ui.doubleSpinBox_algoH3InputXScaleUmPx);
		QSignalBlocker b2(ui.doubleSpinBox_algoH3InputYScaleUmPx);
		QSignalBlocker b3(ui.doubleSpinBox_algoH3InputZScaleIntensityUm);
		QSignalBlocker b4(ui.spinBox_algoH3InputMinValidRaw);
		QSignalBlocker b5(ui.spinBox_algoH3InputMaxValidRaw);
		QSignalBlocker b6(ui.doubleSpinBox_algoH3InputMinValidHeightUm);
		QSignalBlocker b7(ui.doubleSpinBox_algoH3InputMaxValidHeightUm);

		ui.doubleSpinBox_algoH3InputXScaleUmPx->setValue(p.xScaleUmPx);
		ui.doubleSpinBox_algoH3InputYScaleUmPx->setValue(p.yScaleUmPx);
		ui.doubleSpinBox_algoH3InputZScaleIntensityUm->setValue(p.zScaleRawPerUm);
		ui.spinBox_algoH3InputMinValidRaw->setValue(p.minValidRaw);
		ui.spinBox_algoH3InputMaxValidRaw->setValue(p.maxValidRaw);
		ui.doubleSpinBox_algoH3InputMinValidHeightUm->setValue(p.minValidHeightUm);
		ui.doubleSpinBox_algoH3InputMaxValidHeightUm->setValue(p.maxValidHeightUm);
	}

	// ── section 1 ──
	{
		QSignalBlocker b1(ui.comboBox_algoH3PreprocessMethod);
		QSignalBlocker b2(ui.spinBox_algoH3PreprocessMedianKernelSize);
		QSignalBlocker b3(ui.spinBox_algoH3PreprocessGaussianKernelSize);
		QSignalBlocker b4(ui.doubleSpinBox_algoH3PreprocessGaussianSigma);
		QSignalBlocker b5(ui.spinBox_algoH3PreprocessBilateralDiameter);
		QSignalBlocker b6(ui.doubleSpinBox_algoH3PreprocessBilateralSpatialSigma);
		QSignalBlocker b7(ui.doubleSpinBox_algoH3PreprocessBilateralHeightSigma);
		QSignalBlocker b8(ui.spinBox_algoH3PreprocessOpeningKernelSize);
		QSignalBlocker b9(ui.spinBox_algoH3PreprocessClosingKernelSize);

		const int m = (int)p.preprocess;
		ui.comboBox_algoH3PreprocessMethod->setCurrentIndex(
			(m >= 0 && m < ui.comboBox_algoH3PreprocessMethod->count()) ? m : 0);
		ui.spinBox_algoH3PreprocessMedianKernelSize->setValue(p.medianKernel);
		ui.spinBox_algoH3PreprocessGaussianKernelSize->setValue(p.gaussianKernel);
		ui.doubleSpinBox_algoH3PreprocessGaussianSigma->setValue(p.gaussianSigma);
		ui.spinBox_algoH3PreprocessBilateralDiameter->setValue(p.bilateralDiameter);
		ui.doubleSpinBox_algoH3PreprocessBilateralSpatialSigma->setValue(p.bilateralSpatialSigma);
		ui.doubleSpinBox_algoH3PreprocessBilateralHeightSigma->setValue(p.bilateralHeightSigma);
		ui.spinBox_algoH3PreprocessOpeningKernelSize->setValue(p.openingKernel);
		ui.spinBox_algoH3PreprocessClosingKernelSize->setValue(p.closingKernel);
	}
	if ((int)p.preprocess >= 0 && (int)p.preprocess < ui.stackedWidget_algoH3Preprocess->count())
		ui.stackedWidget_algoH3Preprocess->setCurrentIndex((int)p.preprocess);

	// ── section 2 ──
	{
		QSignalBlocker b0(ui.comboBox_algoH3SegMethod);
		QSignalBlocker b1(ui.checkBox_algoH3SegEnableWidthCheck);
		QSignalBlocker b2(ui.doubleSpinBox_algoH3SegMinWidthUm);
		QSignalBlocker b3(ui.doubleSpinBox_algoH3SegMaxWidthUm);
		QSignalBlocker b4(ui.checkBox_algoH3SegEnableHeightCheck);
		QSignalBlocker b5(ui.doubleSpinBox_algoH3SegMinHeightUm);
		QSignalBlocker b6(ui.doubleSpinBox_algoH3SegMaxHeightUm);
		QSignalBlocker b7(ui.checkBox_algoH3SegEnableAngleCheck);
		QSignalBlocker b8(ui.doubleSpinBox_algoH3SegMinAngleDeg);
		QSignalBlocker b9(ui.doubleSpinBox_algoH3SegMaxAngleDeg);

		const int sm = (int)p.segMethod;
		ui.comboBox_algoH3SegMethod->setCurrentIndex(
			(sm >= 0 && sm < ui.comboBox_algoH3SegMethod->count()) ? sm : 0);
		ui.checkBox_algoH3SegEnableWidthCheck->setChecked(p.segCheckWidth);
		ui.doubleSpinBox_algoH3SegMinWidthUm->setValue(p.segMinWidthUm);
		ui.doubleSpinBox_algoH3SegMaxWidthUm->setValue(p.segMaxWidthUm);
		ui.checkBox_algoH3SegEnableHeightCheck->setChecked(p.segCheckHeight);
		ui.doubleSpinBox_algoH3SegMinHeightUm->setValue(p.segMinHeightUm);
		ui.doubleSpinBox_algoH3SegMaxHeightUm->setValue(p.segMaxHeightUm);
		ui.checkBox_algoH3SegEnableAngleCheck->setChecked(p.segCheckAngle);
		ui.doubleSpinBox_algoH3SegMinAngleDeg->setValue(p.segMinAngleDeg);
		ui.doubleSpinBox_algoH3SegMaxAngleDeg->setValue(p.segMaxAngleDeg);
	}

	// ── section 3 ──
	{
		QSignalBlocker b1(ui.comboBox_algoH3DatumMethod);
		QSignalBlocker b2(ui.checkBox_algoH3DatumEnableTiltCheck);
		QSignalBlocker b3(ui.doubleSpinBox_algoH3DatumMaxTiltDeg);

		const int m = (int)p.datumMethod;
		ui.comboBox_algoH3DatumMethod->setCurrentIndex(
			(m >= 0 && m < ui.comboBox_algoH3DatumMethod->count()) ? m : 0);
		ui.checkBox_algoH3DatumEnableTiltCheck->setChecked(p.datumCheckTilt);
		ui.doubleSpinBox_algoH3DatumMaxTiltDeg->setValue(p.datumMaxTiltDeg);
	}

	// ── section 4 ──
	{
		QSignalBlocker b1(ui.comboBox_algoH3Method);
		QSignalBlocker b2(ui.doubleSpinBox_algoH3MethodPercentileValue);

		ui.comboBox_algoH3Method->setCurrentIndex(
			algoH3MethodValid(p.methodId) ? p.methodId : 0);
		ui.doubleSpinBox_algoH3MethodPercentileValue->setValue(p.percentile);
	}
	if (algoH3MethodValid(p.methodId) && p.methodId < ui.stackedWidget_algoH3Method->count())
		ui.stackedWidget_algoH3Method->setCurrentIndex(p.methodId);

	// ── section 7 ──
	{
		QSignalBlocker b1(ui.checkBox_algoH3OverallEnableCountCheck);
		QSignalBlocker b2(ui.spinBox_algoH3OverallMaxCount);
		QSignalBlocker b3(ui.checkBox_algoH3OverallEnableRateCheck);
		QSignalBlocker b4(ui.doubleSpinBox_algoH3OverallMaxRate);

		ui.checkBox_algoH3OverallEnableCountCheck->setChecked(p.overallCheckCount);
		ui.spinBox_algoH3OverallMaxCount->setValue(p.overallMaxCount);
		ui.checkBox_algoH3OverallEnableRateCheck->setChecked(p.overallCheckRate);
		ui.doubleSpinBox_algoH3OverallMaxRate->setValue(p.overallMaxRatePct);
	}

	refreshAlgoH3TypeTable();
	refreshAlgoH3RoiBoxes();

	//a freshly opened recipe has run nothing yet
	_algoH3Output = AlgoHeight3Output();
	applyAlgoH3Output(_algoH3Output);
	updateAlgoH3Enables();
}

// =============================================================================
// Results
// =============================================================================

void VisionApp::applyAlgoH3Output(const AlgoHeight3Output& out)
{
	// ── section 1 ──
	h3ShowStage(ui.lineEdit_algoH3PreprocessResult, ui.lineEdit_algoH3PreprocessFailReason,
		ui.lineEdit_algoH3PreprocessTimeMs, out.preprocess);

	// ── section 2 ──
	const bool segRan = out.segment.ran;
	h3ShowNumber(ui.lineEdit_algoH3SegMeasuredWidthUm, segRan, out.segWidthUm, 2);
	h3ShowNumber(ui.lineEdit_algoH3SegMeasuredHeightUm, segRan, out.segHeightUm, 2);
	h3ShowNumber(ui.lineEdit_algoH3SegMeasuredAngleDeg, segRan, out.segAngleDeg, 3);
	h3ShowStage(ui.lineEdit_algoH3SegResult, ui.lineEdit_algoH3SegFailReason,
		ui.lineEdit_algoH3SegTimeMs, out.segment);

	// ── section 3 ──
	h3ShowNumber(ui.lineEdit_algoH3DatumCoeffA, out.planeValid, out.planeA, 6);
	h3ShowNumber(ui.lineEdit_algoH3DatumCoeffB, out.planeValid, out.planeB, 6);
	h3ShowNumber(ui.lineEdit_algoH3DatumCoeffC, out.planeValid, out.planeC, 3);
	h3ShowNumber(ui.lineEdit_algoH3DatumMeasuredTiltDeg, out.planeValid, out.planeTiltDeg, 3);
	h3ShowStage(ui.lineEdit_algoH3DatumResult, ui.lineEdit_algoH3DatumFailReason,
		ui.lineEdit_algoH3DatumTimeMs, out.datum);

	// ── section 6 ──
	ui.lineEdit_algoH3ResultTimeMs->setText(out.measure.ran ? QString::number(out.measure.elapsedMs) : QString());
	refreshAlgoH3ResultSection();

	// ── section 7 ──
	const bool overallRan = out.overall.ran;
	ui.lineEdit_algoH3OverallTotalPins->setText(overallRan ? QString::number(out.totalPins) : QString());
	ui.lineEdit_algoH3OverallPassedPins->setText(overallRan ? QString::number(out.passedPins) : QString());
	ui.lineEdit_algoH3OverallFailedPins->setText(overallRan ? QString::number(out.failedPins) : QString());
	h3ShowNumber(ui.lineEdit_algoH3OverallFailedRate, overallRan, out.failedRatePct, 2);
	h3ShowVerdict(ui.lineEdit_algoH3OverallPassFail, overallRan, out.overallPass);
	ui.lineEdit_algoH3OverallFailReason->setText(overallRan ? out.overall.failReason : QString());
	ui.lineEdit_algoH3OverallTimeMs->setText(overallRan ? QString::number(out.overall.elapsedMs) : QString());
	//total time is a Run All measurement only - a single stage has no "total"
	ui.lineEdit_algoH3OverallTotalTimeMs->setText(
		out.totalElapsedMs > 0 ? QString::number(out.totalElapsedMs) : QString());
}

/*
* Show the selected ROI's result. Showing every ROI at once would be an unreadable wall
* of numbers on a part with hundreds of pins, so the section describes exactly one - the
* one selected on the image - and says so plainly when nothing is selected.
*/
void VisionApp::refreshAlgoH3ResultSection()
{
	int selected = -1;
	int selectedCount = 0;
	for (int i = 0; i < _algoH3RoiBoxes.size(); i++) {
		if (!_algoH3RoiBoxes[i] || !_algoH3RoiBoxes[i]->isSelected()) continue;
		if (selected < 0) selected = i;
		selectedCount++;
	}

	auto clearAll = [&](const QString& instruction) {
		ui.label_algoH3ResultInstruction->setText(instruction);
		ui.lineEdit_algoH3ResultRoiId->clear();
		ui.lineEdit_algoH3ResultRoiType->clear();
		ui.lineEdit_algoH3ResultCriteriaMinUm->clear();
		ui.lineEdit_algoH3ResultCriteriaMaxUm->clear();
		ui.lineEdit_algoH3ResultMethodId->clear();
		ui.lineEdit_algoH3ResultMethodName->clear();
		ui.lineEdit_algoH3ResultZHeightUm->clear();
		h3ShowVerdict(ui.lineEdit_algoH3ResultPassFail, false, false);
		ui.lineEdit_algoH3ResultFailReason->setText(
			(_algoH3Output.measure.ran && !_algoH3Output.measure.pass)
				? _algoH3Output.measure.failReason : QString());
		ui.toolButton_algoH3ResultColor->setStyleSheet(QString());
	};

	//kMultiple keeps "several selected" distinct from "none selected", so the poll below
	//still notices the change between those two states
	constexpr int kMultiple = -2;
	_algoH3ShownRoi = (selectedCount == 1) ? selected : ((selectedCount == 0) ? -1 : kMultiple);

	if (_algoH3RoiBoxes.isEmpty()) { clearAll("No ROIs yet - add them in the ROI Types & Criteria section."); return; }
	if (selectedCount == 0) { clearAll("Select an ROI to view its measurement result."); return; }
	if (selectedCount > 1) { clearAll("Select a single ROI to view its measurement result."); return; }

	const int id = selected + 1;

	ui.lineEdit_algoH3ResultRoiId->setText(QString::number(id));
	ui.lineEdit_algoH3ResultRoiType->setText(_algoH3RoiBoxes[selected]->getTag());

	const AlgoH3RoiResult* r = _algoH3Output.roiById(id);
	if (!r) {
		//taught but not measured since - say so rather than show the previous run's number
		ui.label_algoH3ResultInstruction->setText(
			QStringLiteral("ROI %1 has not been measured yet - press Run Height Measurement.").arg(id));
		ui.lineEdit_algoH3ResultCriteriaMinUm->clear();
		ui.lineEdit_algoH3ResultCriteriaMaxUm->clear();
		ui.lineEdit_algoH3ResultMethodId->clear();
		ui.lineEdit_algoH3ResultMethodName->clear();
		ui.lineEdit_algoH3ResultZHeightUm->clear();
		h3ShowVerdict(ui.lineEdit_algoH3ResultPassFail, false, false);
		ui.lineEdit_algoH3ResultFailReason->clear();
		ui.toolButton_algoH3ResultColor->setStyleSheet(QString());
		return;
	}

	ui.label_algoH3ResultInstruction->setText(
		QStringLiteral("Showing ROI %1 of %2.").arg(id).arg(_algoH3RoiBoxes.size()));
	//the type as MEASURED, which can differ from the box's tag if it was retyped since
	ui.lineEdit_algoH3ResultRoiType->setText(r->typeName);
	ui.lineEdit_algoH3ResultCriteriaMinUm->setText(QString::number(r->criteriaMinUm, 'f', 2));
	ui.lineEdit_algoH3ResultCriteriaMaxUm->setText(QString::number(r->criteriaMaxUm, 'f', 2));
	ui.lineEdit_algoH3ResultMethodId->setText(QString::number(r->methodId));
	ui.lineEdit_algoH3ResultMethodName->setText(r->methodName);
	ui.lineEdit_algoH3ResultZHeightUm->setText(
		r->valid ? QString::number(r->heightUm, 'f', 3) : QStringLiteral("NO DATA"));
	h3ShowVerdict(ui.lineEdit_algoH3ResultPassFail, true, r->pass);

	QString reason = r->failReason;
	//a clipped ROI still produces a number, but the operator has to know it was measured
	//on less than the taught area
	if (r->clipped) {
		const QString note = QStringLiteral("ROI is partly outside the image - measured on the visible part");
		reason = reason.isEmpty() ? note : reason + "; " + note;
	}
	ui.lineEdit_algoH3ResultFailReason->setText(reason);

	if (r->color.isValid()) {
		ui.toolButton_algoH3ResultColor->setStyleSheet(QStringLiteral(
			"QToolButton { background:%1; border:1px solid #777; }").arg(r->color.name()));
	}
}

// =============================================================================
// Running
// =============================================================================

void VisionApp::algoH3RunStage(AlgoH3Stage stage)
{
	auto& mgr = AlgoManager::instance();

	if (mgr.isBusy()) {
		showMsg("Algo is still running, please wait.");
		return;
	}
	if (!mgr.height3HasHeight()) {
		showMsg("Load a height map first (Input section: Load Height Map, or Use Last Scan).");
		return;
	}
	//both maps are required to run, per the page's own contract
	if (!mgr.height3HasIntensity()) {
		showMsg("Load an intensity map first - it must match the height map's size.");
		return;
	}

	//the widgets are the source of truth, so push them down before anything runs
	captureAlgoParamsFromUI();
	clearAlgoOverlay();
	ui.label_algoStatus->setText("Running...");

	mgr.runHeight3(stage);

	static const char* kStageNames[] = { "PREPROCESS", "SEGMENT", "DATUM", "MEASURE", "OVERALL", "ALL" };
	const int s = (int)stage;
	AuditLog::instance().log(QStringLiteral("ALGO_H3_RUN"),
		QString::fromLatin1((s >= 0 && s <= (int)AlgoH3Stage::All) ? kStageNames[s] : "?"));
}
