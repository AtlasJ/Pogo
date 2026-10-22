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
#include "External3DViewer.h"

#include <QColorDialog>
#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QInputDialog>
#include <QLabel>
#include <QMenu>
#include <QMessageBox>
#include <QPainter>
#include <QPixmap>
#include <QProgressBar>
#include <QRadioButton>
#include <QScrollArea>
#include <QSpinBox>
#include <QTableWidgetItem>
#include <QToolButton>
#include <QVBoxLayout>

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
* the view is zoomed. That is deliberate:
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

/*
* One log line for a stage the page just ran. Until this existed the page logged only the
* button press - ALGO_H3_RUN in the audit log, written at dispatch with no result, so it
* reads "OK" whatever happened - and every number went to the widgets and nowhere else.
* Production logs its own line per unit (InspectionThread); this is the page's equivalent.
*/
static void h3LogStage(const char* stage, const AlgoH3StageResult& s, const QString& details)
{
	if (!s.ran) return; //a Run All that stopped early never reached the later stages

	QString line = s.pass
		? QStringLiteral("OK in %1 ms").arg(s.elapsedMs)
		: QStringLiteral("FAILED in %1 ms: %2").arg(s.elapsedMs).arg(s.failReason);
	if (!details.isEmpty()) line += QStringLiteral(" - ") + details;
	if (!s.note.isEmpty()) line += QStringLiteral(" [note: %1]").arg(s.note);

	if (s.pass) ct::logger::info("[Algo H3] Page %s: %s", stage, line.toStdString().c_str());
	else ct::logger::warn("[Algo H3] Page %s: %s", stage, line.toStdString().c_str());
}

//the same sentence in both refusals, so the operator is told where the ROIs DO belong
static const char* kH3RoiSectionHint =
	"Open the Datum Plane section to work with datum ROIs, or ROI Types & Criteria / "
	"Measurement Results for measurement ROIs.";

/*
* The please-wait box while the external 3D display opens or closes. The same look as
* loadingBarSetup() - a moving bar, no Cancel, no close button - but its OWN instance: that
* one is a single shared member driven by synchronous flows, and this job runs on a worker
* while the GUI keeps going. APPLICATION MODAL, so nothing else can be clicked until ImageJ
* is up or gone. Escape and Alt+F4 are swallowed: the job cannot be cancelled, so the box
* must not look as if it can. Code takes it down with hide(), which sends no close event.
*/
class H3BusyDialog : public QDialog
{
public:
	H3BusyDialog(const QString& text, QWidget* parent)
		: QDialog(parent, Qt::Dialog | Qt::CustomizeWindowHint | Qt::WindowTitleHint)
	{
		setWindowTitle(QStringLiteral("External 3D Display"));
		setWindowModality(Qt::ApplicationModal);

		auto* layout = new QVBoxLayout(this);
		m_label = new QLabel(text, this);
		m_label->setMinimumWidth(420);
		auto* bar = new QProgressBar(this);
		bar->setRange(0, 0); //indeterminate: it keeps moving, which is the point
		bar->setTextVisible(false);
		layout->addWidget(m_label);
		layout->addWidget(bar);
	}

	void setText(const QString& text) { m_label->setText(text); }

protected:
	void keyPressEvent(QKeyEvent* e) override
	{
		if (e->key() == Qt::Key_Escape) return; //QDialog would reject() and hide it
		QDialog::keyPressEvent(e);
	}
	void closeEvent(QCloseEvent* e) override { e->ignore(); }

private:
	QLabel* m_label = nullptr;
};

const char* kH3ColorProp = "algoH3Color"; //QColor carried by a type row's colour button

static void h3SetReadonly(QLineEdit* le)
{
	if (le) le->setReadOnly(true);
}

/*
* Stop a readout from sizing the page.
*
* Several of these carry a sentence rather than a number - a fail reason, the instruction line,
* the type status - and the sentence is as long as whatever happened. Left to its own size hint
* a widget like that widens the whole section the moment the message grows, so the panel moves
* under the operator's hand for no reason they can see.
*
* Ignored is the policy that says it outright: the layout gives the widget the room there is
* and never asks what it would like. A line edit scrolls its own text, so nothing is lost; a
* label would clip, so the ones carrying messages are elided explicitly below and keep the full
* text on the tooltip.
*/
static void h3SetNonExpanding(QWidget* w, int minWidth = 60)
{
	if (!w) return;
	w->setSizePolicy(QSizePolicy::Ignored, w->sizePolicy().verticalPolicy());
	w->setMinimumWidth(minWidth);
	//a wrapped label grows DOWNWARD instead, which is the same problem turned on its side
	if (auto* lbl = qobject_cast<QLabel*>(w)) lbl->setWordWrap(false);
}

//A message in a widget that must not grow: show what fits, keep the whole of it on the
//tooltip. Elided against the width it has NOW, which on this page is a fixed panel.
static void h3SetMessage(QLabel* lbl, const QString& text)
{
	if (!lbl) return;
	lbl->setToolTip(text);
	const int room = std::max(40, lbl->width() - 4);
	lbl->setText(lbl->fontMetrics().elidedText(text, Qt::ElideRight, room));
}

static void h3SetMessage(QLineEdit* le, const QString& text)
{
	//a line edit scrolls, so it needs no elision - only the tooltip, for the tail it is hiding
	if (!le) return;
	le->setToolTip(text);
	le->setText(text);
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
	if (reason) {
		//a stage that passed clears failReason, so the note is the only thing left to show -
		//and a part cropped to fit the canvas has to be visible, not just logged
		QString text = s.failReason;
		if (text.isEmpty() && !s.note.isEmpty()) text = QStringLiteral("Note: ") + s.note;
		reason->setText(s.ran ? text : QString());
	}
	if (time) time->setText(s.ran ? QString::number(s.elapsedMs) : QString());
}

static void h3ShowNumber(QLineEdit* le, bool have, double v, int decimals)
{
	if (!le) return;
	le->setText(have ? QString::number(v, 'f', decimals) : QStringLiteral("-"));
}

//a small filled square, so the type combo says which colour a type is without being opened
static QIcon h3ColorIcon(const QColor& c)
{
	QPixmap pm(14, 14);
	pm.fill(c);
	QPainter pr(&pm);
	pr.setPen(QColor(80, 80, 80));
	pr.drawRect(0, 0, 13, 13);
	pr.end();
	return QIcon(pm);
}

} //namespace

// =============================================================================
// Setup
// =============================================================================

void VisionApp::initAlgoHeight3Page()
{
	configureAlgoH3Ranges();
	buildAlgoH3AlgoRow();
	buildAlgoH3SegMethodRows();
	buildAlgoH3DatumMethodRows();

	connect(ui.comboBox_algoH3DatumMethod, QOverload<int>::of(&QComboBox::currentIndexChanged),
		this, [=](int) {
			updateAlgoH3DatumMethodUi();
			captureAlgoH3ParamsFromUI();
		});
	updateAlgoH3DatumMethodUi();

	//a method change swaps which settings are on screen, and the new ones have to be captured
	//into the params before any Run reads them
	connect(ui.comboBox_algoH3SegMethod, QOverload<int>::of(&QComboBox::currentIndexChanged),
		this, [=](int) {
			updateAlgoH3SegMethodUi();
			captureAlgoH3ParamsFromUI();
		});
	updateAlgoH3SegMethodUi();

	//every readout on this page is output, never input
	const QStringList readouts = {
		"lineEdit_algoH3InputLoadHeightStatus", "lineEdit_algoH3InputLoadIntensityStatus",
		"lineEdit_algoH3InputUseLastScanStatus",
		"lineEdit_algoH3PreprocessResult", "lineEdit_algoH3PreprocessFailReason", "lineEdit_algoH3PreprocessTimeMs",
		"lineEdit_algoH3SegCanvasPx",
		"lineEdit_algoH3SegMeasuredWidthUm", "lineEdit_algoH3SegMeasuredHeightUm", "lineEdit_algoH3SegMeasuredAngleDeg",
		"lineEdit_algoH3SegResult", "lineEdit_algoH3SegFailReason", "lineEdit_algoH3SegTimeMs",
		"lineEdit_algoH3DatumCoeffA", "lineEdit_algoH3DatumCoeffB",
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
	for (const QString& name : readouts) {
		auto* le = ui.widget_algoHeight3Page->findChild<QLineEdit*>(name);
		h3SetReadonly(le);
		h3SetNonExpanding(le);
	}

	/*
	* The two datum counts share a line with a stretching spacer, and there the Ignored policy
	* above backfires: Qt sizes an Ignored box from a 0 width hint, and with no stretch of its
	* own it can be crushed to a sliver. They only ever hold a small number, so a fixed width.
	*/
	for (QLineEdit* le : { ui.lineEdit_algoH3DatumRoiCount, ui.lineEdit_algoH3DatumRoiRequired }) {
		h3SetReadonly(le);
		le->setSizePolicy(QSizePolicy::Fixed, le->sizePolicy().verticalPolicy());
		le->setFixedWidth(60);
	}

	//the rest of the page's variable text: the type status, the result instruction, and the
	//shared algo status line that every page writes to
	h3SetReadonly(ui.lineEdit_algoH3RoiTypeStatus);
	h3SetNonExpanding(ui.lineEdit_algoH3RoiTypeStatus, 120);
	h3SetNonExpanding(ui.label_algoH3ResultInstruction, 120);
	h3SetNonExpanding(ui.label_algoH3DatumEquation, 120);
	h3SetNonExpanding(ui.lineEdit_algoStatus, 120);

	//Ctrl+C / Ctrl+V has no button of its own, so say so where the operator is looking
	const QString copyHint = QStringLiteral(
		"\n\nSelect ROIs on the image and press Ctrl+C to copy, Ctrl+V to paste them offset by 10 px.");
	ui.toolButton_algoH3RoiAdd->setToolTip(
		"Add one ROI of the selected type, in the middle of the part of the image you are looking at." + copyHint);
	ui.toolButton_algoH3DatumAddRoi->setToolTip(
		"Add one datum ROI, in the middle of the part of the image you are looking at." + copyHint);

	// ── display mode + section: both decide what is on screen ──
	connect(ui.comboBox_algoH3Display, QOverload<int>::of(&QComboBox::currentIndexChanged),
		this, [=](int) {
			//the fill only reaches the 3D renderer, so the toggle is dead weight on a flat view
			ui.checkBox_algoH3FillHoles->setEnabled(h3IsSurfaceMode(algoH3DisplayMode()));
			ui.checkBox_algoH3ShowPlane->setEnabled(h3IsSurfaceMode(algoH3DisplayMode()));
			updateAlgoH3Display();
		});

	connect(ui.checkBox_algoH3FillHoles, &QCheckBox::toggled, this, [=](bool) {
		updateAlgoH3Display();
	});
	connect(ui.checkBox_algoH3ShowPlane, &QCheckBox::toggled, this, [=](bool) {
		updateAlgoH3Display();
	});
	ui.checkBox_algoH3FillHoles->setEnabled(h3IsSurfaceMode(algoH3DisplayMode()));
	ui.checkBox_algoH3ShowPlane->setEnabled(h3IsSurfaceMode(algoH3DisplayMode()));

	/*
	* ── external 3D display: ImageJ's Interactive 3D Surface Plot, in a window of its own ──
	* Open hands ImageJ the map the page is showing for the open section, through one temp
	* TIFF that every open overwrites. Both jobs wait on another process for seconds, so they
	* run on the viewer's worker and report back here; the two buttons stay disabled until
	* they do. Close is always offered: ImageJ can outlive the plot window (closing the plot
	* by hand leaves ImageJ running hidden), and Close is how that gets cleared up.
	*/
	_external3D = new External3DViewer(this);
	connect(_external3D, &External3DViewer::progress, this, [=](QString text) {
		if (_external3DBusy) static_cast<H3BusyDialog*>(_external3DBusy.data())->setText(text);
	});
	connect(_external3D, &External3DViewer::finished, this, [=](bool ok, QString message) {
		//the box goes BEFORE any message: showMsg() is modal too, and should not sit under it
		hideExternal3DBusy();
		updateAlgoH3Enables(); //idle again - both buttons come back
		if (ok) showStatus(message, 8000);
		else showMsg(message);
	});

	connect(ui.toolButton_algoH3DisplayExternalOpen, &QToolButton::clicked, this, [=]() {
		const QString exe = External3DViewer::imageJExe();
		if (!QFileInfo::exists(exe)) {
			showMsg(QStringLiteral("ImageJ was not found:\n%1").arg(exe));
			return;
		}

		//the same map the 2D and 3D views are showing, so the external one agrees with them
		bool preprocessed = false, segmented = false;
		algoH3ShownStage(preprocessed, segmented);

		cv::Mat height;
		if (!AlgoManager::instance().height3CopyDisplayHeight(preprocessed, segmented, height)) {
			showMsg(QStringLiteral("A stage is still running - open the external 3D display once it has finished."));
			return;
		}
		if (height.empty()) {
			showMsg("Load a height map first (Input section: Load Height Map, or Use Last Scan).");
			return;
		}

		if (!_external3D->open(height)) return; //already working; the buttons say so
		updateAlgoH3Enables();
		showStatus(QStringLiteral("Opening the external 3D display..."), 0);
		showExternal3DBusy(QStringLiteral("Opening the external 3D display..."));
		AuditLog::instance().log(QStringLiteral("ALGO_H3_EXTERNAL_3D_OPEN"),
			QStringLiteral("%1 x %2 %3").arg(height.cols).arg(height.rows)
			.arg(segmented ? "crop" : preprocessed ? "preprocessed" : "raw"));
	});

	connect(ui.toolButton_algoH3DisplayExternalClose, &QToolButton::clicked, this, [=]() {
		if (!_external3D->close()) return;
		updateAlgoH3Enables();
		showStatus(QStringLiteral("Closing the external 3D display..."), 0);
		showExternal3DBusy(QStringLiteral("Closing the external 3D display..."));
		AuditLog::instance().log(QStringLiteral("ALGO_H3_EXTERNAL_3D_CLOSE"));
	});

	connect(ui.toolBox_algoH3Sections, &QToolBox::currentChanged, this, [=](int) {
		updateAlgoH3Display();
		updateAlgoH3RoiVisibility();
		refreshAlgoH3ResultSection();
		//the newly opened section decides the height now, and a stacked sub-page may have
		//changed since it was last measured
		fitAlgoH3Sections();
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
		rememberAlgoH3Input(false, path);
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
		rememberAlgoH3Input(true, path);
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
		//the maps now come from the scan, not from a pair of files: forget the remembered
		//ones rather than have the next restart silently replace a scan with stale files
		rememberAlgoH3Input(false, QString());
		rememberAlgoH3Input(true, QString());

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
			//a taller parameter page needs a taller section
			fitAlgoH3Sections();
		});

	connect(ui.toolButton_algoH3PreprocessRun, &QToolButton::clicked, this, [=]() {
		algoH3RunStage(AlgoH3Stage::Preprocess);
	});

	// ── section 2: segmentation ──
	connect(ui.toolButton_algoH3SegRun, &QToolButton::clicked, this, [=]() {
		algoH3RunStage(AlgoH3Stage::Segment);
	});

	//the px readout is the only place the um -> even-px conversion is visible
	connect(ui.doubleSpinBox_algoH3SegCanvasWidthUm,
		QOverload<double>::of(&QDoubleSpinBox::valueChanged),
		this, [=](double) { updateAlgoH3CanvasPxLabel(); });
	connect(ui.doubleSpinBox_algoH3SegCanvasHeightUm,
		QOverload<double>::of(&QDoubleSpinBox::valueChanged),
		this, [=](double) { updateAlgoH3CanvasPxLabel(); });

	// ── section 3: datum plane ──
	connect(ui.toolButton_algoH3DatumAddRoi, &QToolButton::clicked, this, [=]() {
		if (!AlgoManager::instance().height3SegmentReady()) {
			showMsg("Run segmentation first - a datum ROI is positioned relative to the segmented part.");
			return;
		}
		const int n = _algoH3DatumBoxes.size();
		const QRectF sceneRect = algoH3NewBoxRect(QSizeF(120, 120), _algoH3DatumBoxes);

		auto* box = makeAlgoH3Box(sceneRect, kAlgoH3DatumColor, QStringLiteral("Datum %1").arg(n + 1));
		_algoH3DatumBoxes.append(box);
		updateAlgoH3RoiVisibility();
		algoH3SelectOnly(box);
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

	/*
	* The per-type method list is built from algoH3MethodName() rather than typed into the
	* .ui, so it can never drift from the AlgoH3Method enum - the index IS the Method ID,
	* which is the number stored in the recipe.
	*/
	for (int m = 0; m < kAlgoH3MethodCount; m++)
		ui.comboBox_algoH3RoiTypeMethod->addItem(
			QStringLiteral("%1 - %2").arg(m).arg(algoH3MethodName(m)));

	//switching type commits the one being left BEFORE loading the new one - capture keys
	//off _algoH3TypeIndex, which still points at the outgoing type at this moment
	connect(ui.comboBox_algoH3RoiType, QOverload<int>::of(&QComboBox::currentIndexChanged),
		this, [=](int) {
			if (_algoH3Updating) return;
			captureAlgoH3ParamsFromUI();
			loadAlgoH3TypeFields();
		});

	connect(ui.toolButton_algoH3RoiTypeColor, &QToolButton::clicked, this, [=]() {
		if (_algoH3TypeIndex < 0) return;
		const QColor picked = QColorDialog::getColor(algoH3SelectedTypeColor(), this,
			"ROI Type Colour");
		if (!picked.isValid()) return;

		const QString typeName = ui.comboBox_algoH3RoiType->currentText();
		ui.toolButton_algoH3RoiTypeColor->setProperty(kH3ColorProp, picked);
		ui.toolButton_algoH3RoiTypeColor->setStyleSheet(QStringLiteral(
			"QToolButton { background:%1; border:1px solid #777; }").arg(picked.name()));
		//and the swatch in the list, so the combo still tells the truth when it is closed
		ui.comboBox_algoH3RoiType->setItemIcon(
			ui.comboBox_algoH3RoiType->currentIndex(), h3ColorIcon(picked));

		//recolour this type's ROIs immediately - the colour is how the operator tells one
		//type from another on the image
		for (auto* b : _algoH3RoiBoxes) {
			if (!b || b->getTag() != typeName) continue;
			b->setBorderColor(picked);
			b->update();
		}
		algoSettingsTouched();
	});

	//the three value widgets belong to the SELECTED type, so an edit is a recipe change
	connect(ui.doubleSpinBox_algoH3RoiTypeMinUm,
		QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [=](double) {
			if (!_algoH3Updating) algoSettingsTouched();
		});
	connect(ui.doubleSpinBox_algoH3RoiTypeMaxUm,
		QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [=](double) {
			if (!_algoH3Updating) algoSettingsTouched();
		});
	connect(ui.comboBox_algoH3RoiTypeMethod, QOverload<int>::of(&QComboBox::currentIndexChanged),
		this, [=](int) {
			if (!_algoH3Updating) algoSettingsTouched();
		});

	connect(ui.toolButton_algoH3RoiAddType, &QToolButton::clicked, this, [=]() {
		bool ok = false;
		const QString name = QInputDialog::getText(this, "Add ROI Type",
			"Type name:", QLineEdit::Normal, QString(), &ok).trimmed();
		if (!ok || name.isEmpty()) return;

		//capture first: the per-type widgets are the truth for the type on screen, and
		//appending to a stale copy would throw away whatever was just edited
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

		refreshAlgoH3TypeList();
		//select what was just added, by name - the list is rebuilt, so an index would be
		//a guess about ordering that refreshAlgoH3TypeList does not promise
		{
			QSignalBlocker b(ui.comboBox_algoH3RoiType);
			ui.comboBox_algoH3RoiType->setCurrentIndex(
				ui.comboBox_algoH3RoiType->findText(name));
		}
		loadAlgoH3TypeFields();
		algoSettingsTouched();
	});

	connect(ui.toolButton_algoH3RoiDeleteType, &QToolButton::clicked, this, [=]() {
		const QString name = ui.comboBox_algoH3RoiType->currentText();
		if (name.isEmpty()) {
			showMsg("Select an ROI type first.");
			return;
		}

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

		refreshAlgoH3TypeList();
		//ROI ids shift when one is removed, so the boxes have to be renamed
		for (int i = 0; i < _algoH3RoiBoxes.size(); i++)
			_algoH3RoiBoxes[i]->setName(QStringLiteral("R%1 %2").arg(i + 1).arg(_algoH3RoiBoxes[i]->getTag()));

		ui.lineEdit_algoH3RoiCount->setText(QString::number(_algoH3RoiBoxes.size()));
		updateAlgoH3TypeStatus();
		_algoH3Output.roiResults.clear();
		refreshAlgoH3ResultSection();
		algoSettingsTouched();
	});

	connect(ui.toolButton_algoH3RoiAdd, &QToolButton::clicked, this, [=]() {
		if (!AlgoManager::instance().height3SegmentReady()) {
			showMsg("Run segmentation first - an ROI is positioned relative to the segmented part.");
			return;
		}
		const QString typeName = ui.comboBox_algoH3RoiType->currentText();
		if (typeName.isEmpty()) {
			//an ROI with no type would have no criteria and no method, so there is
			//nothing sensible to do with it - require the type up front
			showMsg("Add an ROI type first - a new ROI is created with the selected type.");
			return;
		}
		const QColor color = algoH3SelectedTypeColor();

		const int n = _algoH3RoiBoxes.size();
		const QRectF sceneRect = algoH3NewBoxRect(QSizeF(60, 60), _algoH3RoiBoxes);

		auto* box = makeAlgoH3Box(sceneRect, color, QStringLiteral("R%1 %2").arg(n + 1).arg(typeName));
		box->setTag(typeName);
		_algoH3RoiBoxes.append(box);

		ui.lineEdit_algoH3RoiCount->setText(QString::number(_algoH3RoiBoxes.size()));
		updateAlgoH3TypeStatus();
		updateAlgoH3RoiVisibility();
		algoH3SelectOnly(box);
		algoSettingsTouched();
	});

	/*
	* ── Auto Assignment: By Pin Detection ──
	*
	* Two hundred boxes placed by hand is the job this replaces. The pipeline finds the pins and
	* the lattice; this turns what it found into ROIs of the operator's own existing types.
	*
	* A single SELECTED ROI is passed down as a seed - its size and position lay the grid out,
	* which is the way through when the field is too broken for the pattern to be read on its
	* own. Selected rather than "the only one", because on a part already half taught the
	* operator still needs to be able to say WHICH box is the good one.
	*/
	connect(ui.toolButton_algoH3RoiAutoAssign, &QToolButton::clicked, this, [=]() {
		auto& mgr = AlgoManager::instance();
		//this one waits on the pipeline's lock rather than skipping a turn like the display
		//renders do, so a stage still running would freeze the window instead of answering
		if (mgr.isBusy()) {
			showMsg("Algo is still running, please wait.");
			return;
		}
		if (!mgr.height3DatumReady()) {
			showMsg("Fit the datum plane first - a pin is found by how far it stands above it.");
			return;
		}
		captureAlgoH3ParamsFromUI();
		//the new ROIs take the operator's own types, criteria and all - so there must be one
		if (mgr.height3Params().roiTypes.isEmpty()) {
			showMsg("Add an ROI type first - By Pin Detection gives the new ROIs one of your "
				"existing types.");
			return;
		}

		QRectF seedRect;
		QString seedType;
		bool haveSeed = false;
		int selected = 0;
		for (auto* b : _algoH3RoiBoxes) if (b && b->isSelected()) selected++;
		if (selected == 1) {
			const QSize crop = mgr.height3CropSize();
			for (auto* b : _algoH3RoiBoxes) {
				if (!b || !b->isSelected()) continue;
				seedRect = b->getGeometry().translated(-crop.width() / 2.0, -crop.height() / 2.0);
				seedType = b->getTag();
				haveSeed = true;
			}
		}
		else if (selected > 1) {
			showMsg("Select a single ROI to lay the grid out from, or none to find the pins "
				"automatically.");
			return;
		}

		AlgoH3PinFind found;
		QString why;
		if (!mgr.height3FindPins(haveSeed ? &seedRect : nullptr, found, why)) {
			showMsg(why);
			setAlgoStatus(QStringLiteral("By Pin Detection: ") + why);
			return;
		}

		AlgoHeight3Params p = mgr.height3Params();

		/*
		* Which EXISTING type the new ROIs get - one choice per size of pin found. No type is
		* created any more: a made-up "Pin" type has no criteria, so it judged nothing until the
		* operator went and filled it in, and a second run sprouted "Pin 1", "Pin 2"... beside
		* the types they had actually set up. The replace warning lives in the same dialog, so
		* there is one prompt, not two.
		*
		* Groups come sorted by how much pin each box holds, smallest first (findPins), so they
		* are labelled by size. Default: the seed ROI's type if one was given, else the type
		* chosen in the ROI Type combo.
		*/
		QVector<int> groupCount(found.groups, 0);
		for (int g : found.group) if (g >= 0 && g < found.groups) groupCount[g]++;

		QStringList existingTypes;
		for (const auto& t : p.roiTypes) existingTypes << t.name;
		const QString defaultType = (haveSeed && existingTypes.contains(seedType))
			? seedType : ui.comboBox_algoH3RoiType->currentText();

		QDialog dlg(this);
		dlg.setWindowTitle(QStringLiteral("Auto Assignment - By Pin Detection"));
		auto* form = new QFormLayout(&dlg);
		form->addRow(new QLabel(QStringLiteral("Found %1.").arg(found.note), &dlg));
		if (found.groups > 1)
			form->addRow(new QLabel(QStringLiteral("The pins come in %1 sizes - choose a type for each:")
				.arg(found.groups), &dlg));

		QVector<QComboBox*> typeBoxes;
		for (int g = 0; g < found.groups; g++) {
			auto* combo = new QComboBox(&dlg);
			combo->addItems(existingTypes);
			const int at = combo->findText(defaultType);
			combo->setCurrentIndex(at >= 0 ? at : 0);
			typeBoxes << combo;

			QString label = QStringLiteral("Type for the %1 new ROIs").arg(groupCount[g]);
			if (found.groups > 1) {
				const QString size = (g == 0) ? QStringLiteral("smallest")
					: (g == found.groups - 1) ? QStringLiteral("largest") : QStringLiteral("middle");
				label = QStringLiteral("Size %1 (%2, %3 pins)").arg(g + 1).arg(size).arg(groupCount[g]);
			}
			form->addRow(label, combo);
		}

		if (!_algoH3RoiBoxes.isEmpty())
			form->addRow(new QLabel(QStringLiteral("This replaces all %1 existing ROI(s).")
				.arg(_algoH3RoiBoxes.size()), &dlg));

		auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dlg);
		connect(buttons, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
		connect(buttons, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
		form->addRow(buttons);
		if (dlg.exec() != QDialog::Accepted) return;

		QStringList typeNames;
		for (auto* combo : typeBoxes) typeNames << combo->currentText();

		//re-read after the dialog's event loop, so a stale snapshot is never written back
		p = mgr.height3Params();
		p.rois.clear();
		for (int i = 0; i < found.boxes.size(); i++) {
			AlgoH3Roi roi;
			roi.rel = found.boxes[i];
			const int g = (i < found.group.size()) ? found.group[i] : 0;
			roi.typeName = typeNames.value(qBound(0, g, typeNames.size() - 1));
			p.rois.append(roi);
		}
		mgr.setHeight3Params(p);

		//the old results are index-matched to the old boxes, so they would label the new ones
		//wrongly until Measure ran again - the same reason By Rows / Columns clears them
		_algoH3Output.roiResults.clear();
		refreshAlgoH3TypeList();
		refreshAlgoH3RoiBoxes();
		updateAlgoH3RoiVisibility();
		refreshAlgoH3ResultSection();
		refreshAlgoH3Overlay();
		algoSettingsTouched();

		setAlgoStatus(QStringLiteral("By Pin Detection: ") + found.note);
		ct::logger::info("[Algo H3] Auto Assign by pin detection: %s", found.note.toStdString().c_str());
		AuditLog::instance().log(QStringLiteral("ALGO_H3_AUTO_ASSIGN"),
			QStringLiteral("%1 ROIs").arg(found.boxes.size()));
	});

	connect(ui.toolButton_algoH3RoiAutoGrid, &QToolButton::clicked, this, [=]() { algoH3AutoAssignGrid(); });

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
			showMsg("Click one or more ROIs on the image first, then press Delete Selected.");
			return;
		}
		for (int i = 0; i < _algoH3RoiBoxes.size(); i++)
			_algoH3RoiBoxes[i]->setName(QStringLiteral("R%1 %2").arg(i + 1).arg(_algoH3RoiBoxes[i]->getTag()));

		ui.lineEdit_algoH3RoiCount->setText(QString::number(_algoH3RoiBoxes.size()));
		updateAlgoH3TypeStatus();
		_algoH3Output.roiResults.clear();
		refreshAlgoH3ResultSection();
		algoSettingsTouched();
	});

	connect(ui.toolButton_algoH3RoiAssignType, &QToolButton::clicked, this, [=]() {
		const QString typeName = ui.comboBox_algoH3RoiType->currentText();
		if (typeName.isEmpty()) {
			showMsg("Select the ROI type to assign to first.");
			return;
		}
		const QColor color = algoH3SelectedTypeColor();

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
	* has no "selection changed" signal, so this polls every 250 ms - and only while
	* section 6 is actually open, so it costs nothing the rest of the time.
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
			/*
			* Production runs each unit's own algo. A result from any other than the one this page
			* tunes would put its numbers under this algo's ROIs, so the page shows none - and the
			* pipeline starts this algo's next single-stage run from scratch (runHeight3Locked).
			*/
			if (out.algoName.compare(AlgoManager::instance().height3EditAlgo(), Qt::CaseInsensitive) != 0) {
				if (_algoH3Output.preprocess.ran) {
					_algoH3Output = AlgoHeight3Output();
					refreshAlgoH3RoiBoxes();
					applyAlgoH3Output(_algoH3Output);
					updateAlgoH3Display();
					updateAlgoH3RoiVisibility();
				}
				updateAlgoH3Enables();
				return;
			}

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
			setAlgoStatus(status);

			/*
			* One log line per stage THIS job ran. A single-stage run carries the earlier
			* stages' results too, from whenever they last ran, so only a Run All logs more
			* than the one stage - and h3LogStage skips any stage it never reached.
			*/
			const bool all = ((AlgoH3Stage)stage == AlgoH3Stage::All);
			auto thisJob = [&](AlgoH3Stage s) { return all || (AlgoH3Stage)stage == s; };

			if (thisJob(AlgoH3Stage::Preprocess))
				h3LogStage("Preprocess", out.preprocess, QString());

			//the size is measured before the canvas check, so a refusal still reports it
			if (thisJob(AlgoH3Stage::Segment))
				h3LogStage("Segment", out.segment, out.segWidthUm > 0.0
					? QStringLiteral("part %1 x %2 um at %3 deg")
						.arg(out.segWidthUm, 0, 'f', 2).arg(out.segHeightUm, 0, 'f', 2)
						.arg(out.segAngleDeg, 0, 'f', 3)
					: QString());

			if (thisJob(AlgoH3Stage::Datum))
				h3LogStage("Datum", out.datum, out.planeValid
					? QStringLiteral("tilt %1 deg, RMS %2 um, %3 points")
						.arg(out.planeTiltDeg, 0, 'f', 3).arg(out.planeRmsUm, 0, 'f', 2)
						.arg(out.datumPoints)
					: QString());

			if (thisJob(AlgoH3Stage::Measure)) {
				int pass = 0, fail = 0, noData = 0;
				for (const auto& r : out.roiResults) {
					if (!r.valid) noData++;
					else if (r.pass) pass++;
					else fail++;
				}
				h3LogStage("Measure", out.measure, out.measure.pass
					? QStringLiteral("%1 ROI(s): %2 pass, %3 fail, %4 no data")
						.arg(out.roiResults.size()).arg(pass).arg(fail).arg(noData)
					: QString());
			}

			//unlike the others, this stage's pass IS the unit's verdict (doOverall sets it from
			//overallPass), so a FAIL here is a failed unit and its reasons are the failReason -
			//the tally is wanted either way, and only a measurement that never ran has none
			if (thisJob(AlgoH3Stage::Overall))
				h3LogStage("Overall", out.overall, out.totalPins > 0
					? QStringLiteral("%1 of %2 pins passed, %3 failed (%4%)")
						.arg(out.passedPins).arg(out.totalPins).arg(out.failedPins)
						.arg(out.failedRatePct, 0, 'f', 2)
					: QString());

			if (all)
				ct::logger::info("[Algo H3] Page Run All: pass=%d, %s (%lld ms)",
					out.overallPass ? 1 : 0, algoH3RunSummary(out).toStdString().c_str(),
					(long long)out.totalElapsedMs);

			/*
			* Remember how far the page got, and save it. last_stage used to be read off the live
			* pipeline at whatever moment the recipe happened to be saved, so a run on its own
			* recorded nothing, and a save while the page was still reopening wrote -1 over the
			* stage the last session had reached. Now only a finished run the operator started
			* here moves it. Not production's own runs (this page is not showing), and not the
			* reopen's re-run, which only re-creates the stage it was given.
			*/
			if (_algoH3RestoreRunPending) {
				_algoH3RestoreRunPending = false;
			}
			else if (isPage(UIPage::ALGO_SETUP) && currentAlgoPageAlgo() == AlgoPageAlgo::HEIGHT_3D_V3) {
				AlgoManager::instance().setHeight3LastStage(algoH3FurthestReached(out));
				algoSettingsTouched();
			}
		});

	//last: every section's contents are configured by now, so their heights are meaningful
	fitAlgoH3Sections();
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
	//deliberately NOT constrained to even values: the box holds um, and what has to be even
	//is the PIXEL count, which the scale conversion decides. Rounded up in doSegment instead.
	setD(ui.doubleSpinBox_algoH3SegCanvasWidthUm, 0.0, 10000000.0, 2);
	setD(ui.doubleSpinBox_algoH3SegCanvasHeightUm, 0.0, 10000000.0, 2);
	setD(ui.doubleSpinBox_algoH3SegMinWidthUm, 0.0, 10000000.0, 2);
	setD(ui.doubleSpinBox_algoH3SegMaxWidthUm, 0.0, 10000000.0, 2);
	setD(ui.doubleSpinBox_algoH3SegMinHeightUm, 0.0, 10000000.0, 2);
	setD(ui.doubleSpinBox_algoH3SegMaxHeightUm, 0.0, 10000000.0, 2);
	//the reported angle is folded into (-45, 45], so the limits match that range
	setD(ui.doubleSpinBox_algoH3SegMinAngleDeg, -45.0, 45.0, 3);
	setD(ui.doubleSpinBox_algoH3SegMaxAngleDeg, -45.0, 45.0, 3);

	// ── section 5: the per-type criteria ──
	setD(ui.doubleSpinBox_algoH3RoiTypeMinUm, -1000000.0, 1000000.0, 2);
	setD(ui.doubleSpinBox_algoH3RoiTypeMaxUm, -1000000.0, 1000000.0, 2);
	//an offset is a magnitude, so it starts at 0 rather than going negative
	setD(ui.doubleSpinBox_algoH3RoiTypeMaxOffsetUm, 0.0, 1000000.0, 2);

	// ── section 3 ──
	setD(ui.doubleSpinBox_algoH3DatumMaxTiltDeg, 0.0, 90.0, 3);

	// ── section 4 ──
	setD(ui.doubleSpinBox_algoH3MethodPercentileValue, 0.0, 100.0, 2);

	// ── section 7 ──
	setI(ui.spinBox_algoH3OverallMaxCount, 0, 1000000);
	setD(ui.doubleSpinBox_algoH3OverallMaxRate, 0.0, 100.0, 2);
}

/*
* Let each toolbox section be as tall as its contents instead of scrolling inside itself.
*
* QToolBox gives every page its OWN QScrollArea. Qt creates them internally - the V3 page
* declares none in the .ui - so each section could scroll independently of the algo page's
* outer scroll area, and the operator got a second, hidden scrollbar for one column of
* settings. The outer bar already handles a tall page; the inner one just buried the bottom
* of a section behind a scroll nobody expects.
*
* Turning the bar off is NOT enough on its own. A QScrollArea is a size firewall - its own
* minimumSizeHint is deliberately tiny, which is exactly what stops a page's height from
* reaching the outer layout (the same trick shields the whole AlgoSetupPage). With the bar
* off and no minimum, the content would CLIP rather than expand, which is worse than the
* scrollbar was. Pushing the minimum height up from the page's sizeHint is what actually
* makes the section grow and hands the scrolling to the outer area where it belongs.
*
* Re-run whenever a section's height can change: opening a different section, or switching
* the preprocessing method to a taller parameter page.
*/
void VisionApp::fitAlgoH3Sections()
{
	auto* tb = ui.toolBox_algoH3Sections;
	if (!tb) return;

	//direct children only - these are the per-page areas QToolBox owns, never one that a
	//page might contain itself
	const QList<QScrollArea*> areas =
		tb->findChildren<QScrollArea*>(QString(), Qt::FindDirectChildrenOnly);

	for (QScrollArea* sa : areas) {
		QWidget* page = sa->widget();
		if (!page) continue;

		//horizontal is left alone on purpose: width minimums on this page are driven from
		//elsewhere (see the notes on the right panel's bars), and forcing it off here would
		//clip a wide row instead of letting it scroll
		sa->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
		sa->setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContents);
		sa->setMinimumHeight(page->sizeHint().height());
	}
}

/*
* Show what the canvas actually becomes in pixels.
*
* The operator types um, but the part frame is pixels, and the two are related by the X/Y
* scale plus a round UP to an even count. Without this the rounding is invisible and the
* size of the frame every ROI lives in has to be worked out by hand.
*/
void VisionApp::updateAlgoH3CanvasPxLabel()
{
	if (!ui.lineEdit_algoH3SegCanvasPx) return;

	const double wUm = ui.doubleSpinBox_algoH3SegCanvasWidthUm->value();
	const double hUm = ui.doubleSpinBox_algoH3SegCanvasHeightUm->value();
	const double sx = ui.doubleSpinBox_algoH3InputXScaleUmPx->value();
	const double sy = ui.doubleSpinBox_algoH3InputYScaleUmPx->value();

	if (wUm <= 0.0 || hUm <= 0.0) {
		const bool band = ((AlgoH3SegMethod)ui.comboBox_algoH3SegMethod->currentIndex()
			== AlgoH3SegMethod::CompleteBand);
		h3SetMessage(ui.lineEdit_algoH3SegCanvasPx, band
			? QStringLiteral("not set - sized to the block found")
			: QStringLiteral("not set"));
		return;
	}
	if (sx <= 0.0 || sy <= 0.0) {
		h3SetMessage(ui.lineEdit_algoH3SegCanvasPx, QStringLiteral("set the X/Y scale first"));
		return;
	}

	//same rounding doSegment uses - keep the two in step
	auto evenPx = [](double um, double umPerPx) {
		int px = (int)std::lround(um / umPerPx);
		if (px < 2) px = 2;
		if (px % 2) px++;
		return px;
	};
	h3SetMessage(ui.lineEdit_algoH3SegCanvasPx, QStringLiteral("%1 x %2 px")
		.arg(evenPx(wUm, sx)).arg(evenPx(hUm, sy)));
}

// =============================================================================
// ROI boxes
// =============================================================================

/*
* Where a new ROI goes: the middle of what the operator is LOOKING at, not the middle of the
* crop. Zoomed in on one corner of a 10000 px part, a box dropped at the centre is off-screen
* and has to be hunted for - and the old "+18 px per existing box" stagger put the 69th box
* 1200 px away from even that.
*
* The view centre only means something while the crop itself is on screen. A 3D render, or a
* section still showing the full map, falls back to the crop centre (boxes are hidden there
* anyway). The box is kept wholly inside the crop, and only if another box of the same kind
* already sits exactly there - Add pressed twice without moving the view - is it stepped by a
* quarter of its size, so the earlier one stays visible underneath.
*/
QRectF VisionApp::algoH3NewBoxRect(const QSizeF& size, const QVector<QDragBox*>& existing) const
{
	const QSize crop = (_algoH3BoxCropW > 0 && _algoH3BoxCropH > 0)
		? QSize(_algoH3BoxCropW, _algoH3BoxCropH)
		: AlgoManager::instance().height3CropSize();
	const double cw = crop.width(), ch = crop.height();

	QPointF centre(cw / 2.0, ch / 2.0);
	const bool cropOnScreen = _fovPixelSource == FovPixelSource::Height3 && _fovH3Segmented
		&& !h3IsSurfaceMode(algoH3DisplayMode());
	if (cropOnScreen)
		centre = ui.graphicsViewFOV->mapToScene(ui.graphicsViewFOV->viewport()->rect().center());

	//whole box inside the crop; a box bigger than the crop is simply centred on it
	auto clampToCrop = [&](QPointF c) {
		const double hx = size.width() / 2.0, hy = size.height() / 2.0;
		c.setX(cw > size.width() ? qBound(hx, c.x(), cw - hx) : cw / 2.0);
		c.setY(ch > size.height() ? qBound(hy, c.y(), ch - hy) : ch / 2.0);
		return c;
	};
	centre = clampToCrop(centre);

	const double step = std::max(size.width(), size.height()) / 4.0;
	for (int tries = 0; tries < 50; tries++) {
		bool taken = false;
		for (auto* b : existing) {
			if (!b) continue;
			const QPointF d = b->getGeometry().center() - centre;
			if (std::fabs(d.x()) < 0.5 && std::fabs(d.y()) < 0.5) { taken = true; break; }
		}
		if (!taken) break;
		const QPointF next = clampToCrop(centre + QPointF(step, step));
		if (next == centre) break;   //pinned in a corner: stacking beats looping forever
		centre = next;
	}

	return QRectF(centre.x() - size.width() / 2.0, centre.y() - size.height() / 2.0,
		size.width(), size.height());
}

/*
* Make a freshly added box the selection, so it is highlighted, the Box readout shows it, and
* Delete or Ctrl+C act on it straight away. Called AFTER updateAlgoH3RoiVisibility(): a hidden
* item ignores setSelected().
*/
void VisionApp::algoH3SelectOnly(QDragBox* box)
{
	if (!box) return;
	_pGraphicsSceneFOV->clearSelection();
	box->setSelected(true);
}

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
		updateAlgoH3TypeStatus();
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
	updateAlgoH3TypeStatus();
	updateAlgoH3RoiVisibility();
}

/*
* Delete the selected ROIs of whichever kind the open section owns.
*
* The same rule copy and paste use, and for the same reason: which boxes are on screen is not
* a safe guide, because a section can show one kind while another is selected underneath. The
* section decides.
*
* It presses the section's own Delete button rather than repeating what that button does. The
* button already handles renumbering the remaining boxes, the type counts and the auto-save,
* and a second copy of that would be a second thing to keep in step - and it inherits the
* button's enabled state for free, so the key cannot do what the button would refuse.
*/
void VisionApp::algoH3DeleteSelectedRois()
{
	const H3RoiOwner owner = h3SectionOwner(algoH3CurrentSection());
	if (owner == H3RoiOwner::None) {
		showStatus(QStringLiteral("This section has no ROIs to delete. %1").arg(kH3RoiSectionHint));
		return;
	}

	/*
	* Selection alone, with no visibility test - the same thing the button asks. A box can be
	* selected while the section that owns it has scrolled it out of view, and an extra
	* condition here would make the key quietly refuse where the button works.
	*
	* Nothing selected is silent: that is a miss, not a mistake.
	*/
	const auto& boxes = (owner == H3RoiOwner::Datum) ? _algoH3DatumBoxes : _algoH3RoiBoxes;
	int selected = 0;
	for (auto* b : boxes) if (b && b->isSelected()) selected++;
	if (selected == 0) return;

	if (owner == H3RoiOwner::Datum) ui.toolButton_algoH3DatumDeleteRoi->click();
	else ui.toolButton_algoH3RoiDelete->click();
}

//Ctrl+A: the kind the open section owns, like Copy - never both kinds at once
void VisionApp::algoH3SelectAllRois()
{
	const H3RoiOwner owner = h3SectionOwner(algoH3CurrentSection());
	if (owner == H3RoiOwner::None) {
		showStatus(QStringLiteral("This section has no ROIs to select. %1").arg(kH3RoiSectionHint));
		return;
	}

	const auto& boxes = (owner == H3RoiOwner::Datum) ? _algoH3DatumBoxes : _algoH3RoiBoxes;
	int selected = 0;
	for (auto* b : boxes) {
		if (!b || !b->isVisible()) continue;
		b->setSelected(true);
		selected++;
	}
	if (selected > 0) showStatus(QStringLiteral("%1 ROI(s) selected").arg(selected));
}

/*
* Auto Assignment: By Rows / Columns. The operator puts an ROI on the first and on the last
* pin of the field - opposite corners - selects both and gives the row and column count; the
* spacing comes from the two ROIs, so nothing has to be measured or typed in um.
*
* The crop is already straightened by segmentation, so the grid is a plain rectangle. Which
* corner pair was used, and which ROI was clicked first, does not matter: the grid spans the
* rectangle between the two centres.
*
* EVERY cell gets an ROI, unlike By Pin Detection. On an inspection that is the point - a
* missing pin must show up as a failed ROI, not as a cell that was quietly left out.
*/
void VisionApp::algoH3AutoAssignGrid()
{
	auto& mgr = AlgoManager::instance();
	if (mgr.isBusy()) {
		showMsg("Algo is still running, please wait.");
		return;
	}
	if (_algoH3BoxCropW <= 0 || _algoH3BoxCropH <= 0) {
		showMsg("Run segmentation first - an ROI only exists relative to the segmented part.");
		return;
	}

	QVector<QDragBox*> picked;
	for (auto* b : _algoH3RoiBoxes) if (b && b->isVisible() && b->isSelected()) picked.append(b);
	if (picked.size() != 2) {
		showMsg("Select exactly 2 ROIs first (Ctrl+click): one on the first pin and one on the "
			"last pin, at opposite corners of the field.");
		return;
	}
	//the upper one first, so the dialog lists them in the order they appear on the image.
	//QPointer, because the dialog below runs its own event loop: anything that rebuilds the
	//boxes meanwhile (a stage finishing, a recipe reload) deletes these two under us
	QPointer<QDragBox> a = picked[0];
	QPointer<QDragBox> b = picked[1];
	if (b->getGeometry().center().y() < a->getGeometry().center().y()) std::swap(a, b);

	//before anything is read from the params: the widgets are the source of truth
	captureAlgoH3ParamsFromUI();
	AlgoHeight3Params p = mgr.height3Params();

	const QRectF ra = a->getGeometry(), rb = b->getGeometry();
	const bool sameSize = std::fabs(ra.width() - rb.width()) < 0.5 && std::fabs(ra.height() - rb.height()) < 0.5;
	const bool sameType = (a->getTag() == b->getTag());

	auto describe = [&](QDragBox* box) {
		const QRectF r = box->getGeometry();
		return QStringLiteral("%1  -  %2 x %3 um")
			.arg(box->getName())
			.arg(r.width() * p.xScaleUmPx, 0, 'f', 0).arg(r.height() * p.yScaleUmPx, 0, 'f', 0);
	};

	// ── the dialog: rows, columns, and which ROI the new ones copy when the two differ ──
	QDialog dlg(this);
	dlg.setWindowTitle(QStringLiteral("Auto Assignment - By Rows / Columns"));
	auto* form = new QFormLayout(&dlg);

	auto* rowsBox = new QSpinBox(&dlg);
	rowsBox->setRange(1, 1000);
	rowsBox->setValue(_algoH3GridRows);
	auto* colsBox = new QSpinBox(&dlg);
	colsBox->setRange(1, 1000);
	colsBox->setValue(_algoH3GridCols);
	form->addRow(QStringLiteral("Rows"), rowsBox);
	form->addRow(QStringLiteral("Columns"), colsBox);

	QRadioButton* followA = nullptr;
	QRadioButton* followB = nullptr;
	if (!sameSize || !sameType) {
		form->addRow(new QLabel(QStringLiteral("The two ROIs differ. Every new ROI takes the size and "
			"type of:"), &dlg));
		followA = new QRadioButton(describe(a), &dlg);
		followB = new QRadioButton(describe(b), &dlg);
		followA->setChecked(true);
		form->addRow(followA);
		form->addRow(followB);
	}

	auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dlg);
	connect(buttons, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
	connect(buttons, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
	form->addRow(buttons);

	if (dlg.exec() != QDialog::Accepted) return;

	if (!a || !b || !_algoH3RoiBoxes.contains(a.data()) || !_algoH3RoiBoxes.contains(b.data())) {
		showMsg("The ROIs were rebuilt while the dialog was open - select the two ROIs again "
			"and press By Rows / Columns again.");
		return;
	}

	const int rows = rowsBox->value();
	const int cols = colsBox->value();
	_algoH3GridRows = rows;
	_algoH3GridCols = cols;

	if (rows * cols < 2) {
		showMsg("A grid needs at least 2 cells - set more than one row or column.");
		return;
	}
	//a hard stop, not a warning: a slipped digit (1000 rows) would otherwise build a million
	//ROI boxes and hang the window. Far above any pin field this page has seen.
	const int kMaxGridRois = 5000;
	if (rows * cols > kMaxGridRois) {
		showMsg(QStringLiteral("%1 x %2 = %3 ROIs is more than the %4 this page allows - check the "
			"row and column count.").arg(rows).arg(cols).arg(rows * cols).arg(kMaxGridRois));
		return;
	}
	//re-read after the dialog's event loop, so a stale snapshot is never written back below
	p = mgr.height3Params();

	QDragBox* follow = (followB && followB->isChecked()) ? b : a;
	const double bw = follow->getGeometry().width();
	const double bh = follow->getGeometry().height();
	const QString typeName = follow->getTag();

	// ── the lattice between the two centres ──
	const QPointF ca = ra.center(), cb = rb.center();
	const double x0 = std::min(ca.x(), cb.x()), x1 = std::max(ca.x(), cb.x());
	const double y0 = std::min(ca.y(), cb.y()), y1 = std::max(ca.y(), cb.y());

	//a single row or column cannot absorb a spread in that direction - say so rather than lay
	//the line through the middle of two pins that are not on it
	if (cols == 1 && (x1 - x0) > bw / 2.0) {
		showMsg(QStringLiteral("With 1 column the two ROIs must be in the same column, but they are "
			"%1 um apart across.").arg((x1 - x0) * p.xScaleUmPx, 0, 'f', 0));
		return;
	}
	if (rows == 1 && (y1 - y0) > bh / 2.0) {
		showMsg(QStringLiteral("With 1 row the two ROIs must be on the same row, but they are "
			"%1 um apart down.").arg((y1 - y0) * p.yScaleUmPx, 0, 'f', 0));
		return;
	}

	const double pitchX = (cols > 1) ? (x1 - x0) / (cols - 1) : 0.0;
	const double pitchY = (rows > 1) ? (y1 - y0) / (rows - 1) : 0.0;
	const double startX = (cols > 1) ? x0 : (x0 + x1) / 2.0;
	const double startY = (rows > 1) ? y0 : (y0 + y1) / 2.0;

	const bool overlap = (cols > 1 && pitchX < bw) || (rows > 1 && pitchY < bh);

	QVector<QRectF> cells;   //scene (crop) coordinates, row by row from the top left
	int outside = 0;
	const QRectF crop(0, 0, _algoH3BoxCropW, _algoH3BoxCropH);
	for (int r = 0; r < rows; r++) {
		for (int c = 0; c < cols; c++) {
			const QRectF cell(startX + c * pitchX - bw / 2.0, startY + r * pitchY - bh / 2.0, bw, bh);
			if (!crop.contains(cell)) outside++;
			cells.append(cell);
		}
	}

	// ── say what is about to happen, warnings included, and let the operator back out ──
	auto pitchText = [](int n, double pitchPx, double scale) {
		return (n > 1) ? QString::number(pitchPx * scale, 'f', 0) : QStringLiteral("-");
	};
	QString text = QStringLiteral("%1 x %2 = %3 ROIs\nPitch %4 x %5 um, box %6 x %7 um, type '%8'.")
		.arg(rows).arg(cols).arg(cells.size())
		.arg(pitchText(cols, pitchX, p.xScaleUmPx)).arg(pitchText(rows, pitchY, p.yScaleUmPx))
		.arg(bw * p.xScaleUmPx, 0, 'f', 0).arg(bh * p.yScaleUmPx, 0, 'f', 0).arg(typeName);
	if (overlap)
		text += QStringLiteral("\n\nWARNING: the pitch is smaller than the box, so neighbouring ROIs "
			"will overlap. Check the row and column count.");
	if (outside > 0)
		text += QStringLiteral("\n\nWARNING: %1 ROI(s) reach past the edge of the segmented part and "
			"will measure less of their pin.").arg(outside);
	text += QStringLiteral("\n\nThis replaces all %1 existing ROI(s). Continue?").arg(_algoH3RoiBoxes.size());

	const auto reply = (overlap || outside > 0)
		? QMessageBox::warning(this, dlg.windowTitle(), text, QMessageBox::Yes | QMessageBox::No, QMessageBox::No)
		: QMessageBox::question(this, dlg.windowTitle(), text, QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
	if (reply != QMessageBox::Yes) return;

	// ── replace the ROIs, in the part frame like every other ROI ──
	const double ox = _algoH3BoxCropW / 2.0, oy = _algoH3BoxCropH / 2.0;
	p.rois.clear();
	for (const QRectF& cell : cells) {
		AlgoH3Roi roi;
		roi.rel = cell.translated(-ox, -oy);
		roi.typeName = typeName;
		p.rois.append(roi);
	}
	mgr.setHeight3Params(p);

	//the old results are index-matched to the old boxes, so they would label the new ones wrongly
	_algoH3Output.roiResults.clear();
	refreshAlgoH3RoiBoxes();
	updateAlgoH3RoiVisibility();
	refreshAlgoH3ResultSection();
	refreshAlgoH3Overlay();
	algoSettingsTouched();

	const QString note = QStringLiteral("%1 x %2 = %3 ROIs on a %4 x %5 um pitch, type '%6'")
		.arg(rows).arg(cols).arg(cells.size())
		.arg(pitchText(cols, pitchX, p.xScaleUmPx)).arg(pitchText(rows, pitchY, p.yScaleUmPx)).arg(typeName);
	setAlgoStatus(QStringLiteral("By Rows / Columns: ") + note);
	ct::logger::info("[Algo H3] Auto Assign by rows/columns: %s", note.toStdString().c_str());
	AuditLog::instance().log(QStringLiteral("ALGO_H3_AUTO_GRID"), QStringLiteral("%1 ROIs").arg(cells.size()));
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
* Selection is read with Qt's isSelected(), never QDragBox::getSelected() - see the "ALWAYS
* ASK A BOX isSelected()" note in VisionApp_AlgoSetup.cpp for why that matters.
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
		//offset so a copy is visibly separate from its original
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
	updateAlgoH3TypeStatus();

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

/*
* Which map the page is showing. Section 0 looks at the raw map; 1 onwards at the FILTERED
* one - the preprocessing section is where the filter and kernel are chosen, so it is the
* one place the cleaned result has to be visible to judge them; 3 onwards at the
* straightened crop. Each only takes effect once that map exists: heightForDisplay falls
* back to the raw map while m_work is empty, so nothing special is needed before the stage
* has run.
*
* ONE rule for every view - the 2D map, the 3D projection and the external 3D display. It
* was written out twice under a "keep in sync" note until the external display needed a
* third copy.
*/
void VisionApp::algoH3ShownStage(bool& preprocessed, bool& segmented) const
{
	const int section = algoH3CurrentSection();
	preprocessed = (section >= SEC_PREPROCESS);
	segmented = (section >= SEC_DATUM) && AlgoManager::instance().height3SegmentReady();
}

// =============================================================================
// Named algos - which one this page tunes
// =============================================================================

/*
* The algo picker, above the sections. In code rather than the .ui for the same reason as the
* method rows below, and because VisionApp.ui is shared and edited in Designer.
*
* One combo and one menu: the panel is narrow, and five buttons in a row would push it into a
* horizontal scrollbar. The default is marked in the list itself, so there is nothing else to
* read to know which one an unmatched cable runs.
*/
void VisionApp::buildAlgoH3AlgoRow()
{
	if (_algoH3AlgoCombo) return;

	auto* row = new QHBoxLayout();
	auto* label = new QLabel(tr("Algo:"), ui.widget_algoHeight3Page);

	_algoH3AlgoCombo = new QComboBox(ui.widget_algoHeight3Page);
	_algoH3AlgoCombo->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
	//sized by a fixed count, not by the longest name: a long one would widen the whole panel
	_algoH3AlgoCombo->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
	_algoH3AlgoCombo->setMinimumContentsLength(12);
	_algoH3AlgoCombo->setToolTip(tr("The algo this page is tuning. In production each unit runs the algo "
		"named after its cable in the PMF, and the default one when there is none."));

	_algoH3AlgoMenuButton = new QToolButton(ui.widget_algoHeight3Page);
	_algoH3AlgoMenuButton->setText(tr("Manage"));
	_algoH3AlgoMenuButton->setPopupMode(QToolButton::InstantPopup);
	auto* menu = new QMenu(_algoH3AlgoMenuButton);
	connect(menu->addAction(tr("New (copy of this one)...")), &QAction::triggered, this, [=]() { algoH3NewAlgo(); });
	connect(menu->addAction(tr("Rename...")), &QAction::triggered, this, [=]() { algoH3RenameAlgo(); });
	connect(menu->addAction(tr("Delete")), &QAction::triggered, this, [=]() { algoH3DeleteAlgo(); });
	menu->addSeparator();
	connect(menu->addAction(tr("Set as Default")), &QAction::triggered, this, [=]() { algoH3SetDefaultAlgo(); });
	_algoH3AlgoMenuButton->setMenu(menu);

	row->addWidget(label);
	row->addWidget(_algoH3AlgoCombo, 1);
	row->addWidget(_algoH3AlgoMenuButton);
	ui.verticalLayout_algoH3Page->insertLayout(0, row);

	//activated, not currentIndexChanged: only the operator's own pick switches algo, never a refill
	connect(_algoH3AlgoCombo, QOverload<int>::of(&QComboBox::activated), this, [=](int index) {
		algoH3SwitchAlgo(_algoH3AlgoCombo->itemData(index).toString());
	});
}

void VisionApp::refreshAlgoH3AlgoCombo()
{
	if (!_algoH3AlgoCombo) return;

	auto& mgr = AlgoManager::instance();
	const QString def = mgr.height3DefaultAlgo();

	QSignalBlocker b(_algoH3AlgoCombo);
	_algoH3AlgoCombo->clear();
	for (const QString& name : mgr.height3AlgoNames()) {
		_algoH3AlgoCombo->addItem(name.compare(def, Qt::CaseInsensitive) == 0
			? tr("%1  (default)").arg(name) : name, name);
	}
	_algoH3AlgoCombo->setCurrentIndex(_algoH3AlgoCombo->findData(mgr.height3EditAlgo()));
}

/*
* Production runs every unit's algo by name and the default for the rest, so neither may change
* under a run - and switching which one is tuned clears the pipeline, which a stage that is
* running holds. Managing them (anything that renames, removes or re-points what production
* runs) is an engineer's job; choosing which one to look at is anybody's.
*/
bool VisionApp::algoH3AlgoActionAllowed(bool manage)
{
	if (_productionSetupLocked || !InspectionThread::instance().isIdle()) {
		showMsg(tr("A production run is using the 3D algos - wait for it to finish."));
		return false;
	}
	if (AlgoManager::instance().isBusy()) {
		showMsg(tr("Algo is still running, please wait."));
		return false;
	}
	if (manage && notAllowToAccess(AccessLevel::OPERATOR)) {
		showMsg(tr("Managing 3D algos needs an engineer login."));
		return false;
	}
	return true;
}

/*
* Save the algo being tuned before anything moves the page off it. The auto-save is a timer, and
* one still pending after a switch would capture the NEXT algo's widgets - harmless - or, after a
* delete, the page's last widgets into whichever algo is tuned by then. Stopping it here and
* saving at once closes both.
*/
void VisionApp::algoH3FlushAlgo()
{
	if (_algoAutoSaveTimer) _algoAutoSaveTimer->stop();
	captureAlgoParamsFromUI();
	auto& mgr = AlgoManager::instance();
	mgr.saveRecipeConfig();
	mgr.saveHeight3Algo();
}

bool VisionApp::algoH3SwitchAlgo(const QString& name)
{
	auto& mgr = AlgoManager::instance();
	if (name.compare(mgr.height3EditAlgo(), Qt::CaseInsensitive) == 0) return true;

	if (!algoH3AlgoActionAllowed(false)) {
		refreshAlgoH3AlgoCombo();   //put the combo back on the algo still being tuned
		return false;
	}

	algoH3FlushAlgo();
	QString error;
	if (!mgr.height3SelectAlgo(name, error)) {
		showMsg(error);
		refreshAlgoH3AlgoCombo();
		return false;
	}
	AuditLog::instance().log(QStringLiteral("ALGO_H3_ALGO_SELECT"), name);
	algoH3AlgoChanged();
	return true;
}

//every setting on the page belongs to the algo, so all of it is refilled; the maps stay loaded
void VisionApp::algoH3AlgoChanged()
{
	_algoH3Output = AlgoHeight3Output();
	refreshAlgoHeight3Page();   //the combo, the settings, the types and the ROI boxes
	applyAlgoH3Output(_algoH3Output);
	updateAlgoH3Display();
	updateAlgoH3RoiVisibility();
	updateAlgoH3Enables();
	setAlgoStatus(tr("Tuning algo '%1' - press Run to run it on the loaded maps")
		.arg(AlgoManager::instance().height3EditAlgo()));
}

void VisionApp::algoH3NewAlgo()
{
	if (!algoH3AlgoActionAllowed(true)) return;

	auto& mgr = AlgoManager::instance();
	const QString from = mgr.height3EditAlgo();
	bool ok = false;
	const QString name = QInputDialog::getText(this, tr("New 3D Algo"),
		tr("Name it after the cable it is for - production runs it for every unit with that cable.\n"
			"It starts as a copy of '%1'.").arg(from), QLineEdit::Normal, QString(), &ok);
	if (!ok) return;

	algoH3FlushAlgo();
	QString error;
	if (!mgr.height3CreateAlgo(name, error)) {
		showMsg(error);
		return;
	}
	AuditLog::instance().log(QStringLiteral("ALGO_H3_ALGO_NEW"), name.trimmed() + QStringLiteral(" from ") + from);
	algoH3AlgoChanged();
}

void VisionApp::algoH3RenameAlgo()
{
	if (!algoH3AlgoActionAllowed(true)) return;

	auto& mgr = AlgoManager::instance();
	const QString old = mgr.height3EditAlgo();
	bool ok = false;
	const QString name = QInputDialog::getText(this, tr("Rename 3D Algo"),
		tr("New name for '%1':").arg(old), QLineEdit::Normal, old, &ok);
	if (!ok || name.trimmed() == old) return;

	algoH3FlushAlgo();
	QString error;
	if (!mgr.height3RenameAlgo(name, error)) {
		showMsg(error);
		return;
	}
	AuditLog::instance().log(QStringLiteral("ALGO_H3_ALGO_RENAME"), old + QStringLiteral(" -> ") + name.trimmed());
	refreshAlgoH3AlgoCombo();
	setAlgoStatus(tr("Renamed algo '%1' to '%2'").arg(old, name.trimmed()));
}

void VisionApp::algoH3DeleteAlgo()
{
	if (!algoH3AlgoActionAllowed(true)) return;

	auto& mgr = AlgoManager::instance();
	const QString name = mgr.height3EditAlgo();
	if (name.compare(mgr.height3DefaultAlgo(), Qt::CaseInsensitive) == 0) {
		showMsg(tr("'%1' is the default algo - set another one as the default first.").arg(name));
		return;
	}
	if (QMessageBox::question(this, tr("Delete 3D Algo"),
			tr("Delete algo '%1'?\n\nUnits with this cable will run the default algo '%2' instead. "
				"Its file is kept in config\\algo_height3\\deleted and can be moved back.")
				.arg(name, mgr.height3DefaultAlgo()),
			QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes)
		return;

	algoH3FlushAlgo();
	QString error;
	if (!mgr.height3DeleteAlgo(name, error)) {
		showMsg(error);
		return;
	}
	AuditLog::instance().log(QStringLiteral("ALGO_H3_ALGO_DELETE"), name);
	algoH3AlgoChanged();
}

void VisionApp::algoH3SetDefaultAlgo()
{
	if (!algoH3AlgoActionAllowed(true)) return;

	auto& mgr = AlgoManager::instance();
	const QString name = mgr.height3EditAlgo();
	QString error;
	if (!mgr.height3SetDefaultAlgo(name, error)) {
		showMsg(error);
		return;
	}
	AuditLog::instance().log(QStringLiteral("ALGO_H3_ALGO_DEFAULT"), name);
	refreshAlgoH3AlgoCombo();
	setAlgoStatus(tr("'%1' is now the default algo - units whose cable has no algo of its own run it").arg(name));
}

/*
* Remember, or forget, one of the two input files.
*
* Alongside Recent_Open_Recipe in the system JSON, because that is already the app's answer to
* "pick up where you left off" and these belong to the installation rather than to the recipe -
* the maps are sample data the operator teaches against, not something the recipe owns.
*
* An empty path clears the key, which is what "Use Last Scan" does: the maps come from the
* machine then, and leaving the old files recorded would have the next restart quietly put a
* stale pair back in their place.
*/
/*
* The settings that belong to one segmentation method only.
*
* Inserted into the form in code rather than placed in the .ui: that form numbers its rows
* explicitly, so adding a row in the middle there means renumbering every row below it, and a
* mis-numbered QFormLayout fails quietly by stacking two widgets in the same cell.
*
* They go directly under the method combo, so the settings that change with the method sit
* next to it - above the canvas and the width, height and angle checks, which every method
* shares.
*/
void VisionApp::buildAlgoH3SegMethodRows()
{
	auto* form = ui.formLayout_algoH3Seg;
	if (!form || _algoH3SegBandStep) return;

	const QString kWhite = QStringLiteral("color: #F0F0F0;");

	_algoH3SegBandStepLabel = new QLabel(tr("Plate Step (raw)"), ui.widget_algoHeight3Page);
	_algoH3SegBandStepLabel->setStyleSheet(kWhite);
	_algoH3SegBandStep = new QDoubleSpinBox(ui.widget_algoHeight3Page);
	_algoH3SegBandStep->setRange(1.0, 60000.0);
	_algoH3SegBandStep->setDecimals(0);
	_algoH3SegBandStep->setSingleStep(25.0);
	_algoH3SegBandStep->setValue(250.0);
	_algoH3SegBandStep->setToolTip(tr(
		"The smallest step in plate height that counts as a boundary between two blocks, in raw "
		"grey levels - the same units as the height map.\n\n"
		"It has to clear the tilt ALONG a plate, which on a long block can total as much as the "
		"step between two of them. Too high and the blocks are never separated; too low and the "
		"tilt itself gets cut into slivers.\n\n"
		"Run once and read the Fail Reason: it says how many boundaries were found."));

	_algoH3SegBandMinLabel = new QLabel(tr("Min Block Height (um)"), ui.widget_algoHeight3Page);
	_algoH3SegBandMinLabel->setStyleSheet(kWhite);
	_algoH3SegBandMin = new QDoubleSpinBox(ui.widget_algoHeight3Page);
	_algoH3SegBandMin->setRange(0.0, 1000000.0);
	_algoH3SegBandMin->setDecimals(1);
	_algoH3SegBandMin->setSingleStep(100.0);
	_algoH3SegBandMin->setValue(0.0);
	_algoH3SegBandMin->setToolTip(tr(
		"A band shorter than this is not a block. It stops a sliver between two boundaries that "
		"landed close together from being taken for one.\n\n"
		"0 = an eighth of the map."));

	//row 0 is the method combo, so 1 puts these directly under it (Step first, then Min)
	form->insertRow(1, _algoH3SegBandMinLabel, _algoH3SegBandMin);
	form->insertRow(1, _algoH3SegBandStepLabel, _algoH3SegBandStep);

	/*
	* Both hooks, and the auto-save one is not optional.
	*
	* initAlgoSetupPage() wires every spin box, combo and checkbox on the algo page to
	* algoSettingsTouched() by walking its children - but it does that walk BEFORE this
	* function runs, so a widget built here is never found by it. Left to the walk, these two
	* would save only when some OTHER control on the page happened to be nudged, which is a
	* worse failure than not saving at all: the setting survives or vanishes depending on what
	* else was touched afterwards.
	*/
	auto edited = [=](double) {
		captureAlgoH3ParamsFromUI();   //live immediately, for a Run before the debounce fires
		algoSettingsTouched();         //and into the recipe 600 ms later
	};
	connect(_algoH3SegBandStep, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, edited);
	connect(_algoH3SegBandMin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, edited);
}

/*
* Show only the settings the chosen method actually reads.
*
* Only Plate Step and Min Block Height belong to one method. The canvas does not: both methods
* use it when it is set, and a typed canvas is the stronger guarantee - it cannot move unit to
* unit. What differs is a canvas left at 0: Largest Connected Region refuses, Complete Block
* falls back to the block it found. So the canvas stays on screen for both, and its readout
* says which of the two a 0 means.
*/
void VisionApp::updateAlgoH3SegMethodUi()
{
	const auto method = (AlgoH3SegMethod)ui.comboBox_algoH3SegMethod->currentIndex();
	const bool band = (method == AlgoH3SegMethod::CompleteBand);

	if (_algoH3SegBandStepLabel) _algoH3SegBandStepLabel->setVisible(band);
	if (_algoH3SegBandStep) _algoH3SegBandStep->setVisible(band);
	if (_algoH3SegBandMinLabel) _algoH3SegBandMinLabel->setVisible(band);
	if (_algoH3SegBandMin) _algoH3SegBandMin->setVisible(band);

	//what a canvas of 0 means depends on the method
	updateAlgoH3CanvasPxLabel();

	//the section just changed height, so the toolbox has to be remeasured
	fitAlgoH3Sections();
}

/*
* The datum settings that belong to one method only - built in code for the same reason the
* segmentation ones are: formLayout_algoH3Datum numbers its rows explicitly, and inserting one
* in the middle of the .ui means renumbering every row below it.
*/
void VisionApp::buildAlgoH3DatumMethodRows()
{
	auto* form = ui.formLayout_algoH3Datum;
	if (!form || _algoH3DatumFlatness) return;

	_algoH3DatumFlatnessLabel = new QLabel(tr("Flatness (um)"), ui.widget_algoHeight3Page);
	_algoH3DatumFlatnessLabel->setStyleSheet(QStringLiteral("color: #F0F0F0;"));
	_algoH3DatumFlatness = new QDoubleSpinBox(ui.widget_algoHeight3Page);
	_algoH3DatumFlatness->setRange(0.1, 10000.0);
	_algoH3DatumFlatness->setDecimals(1);
	_algoH3DatumFlatness->setSingleStep(1.0);
	_algoH3DatumFlatness->setValue(15.0);
	_algoH3DatumFlatness->setToolTip(tr(
		"How much a patch of surface may vary, top to bottom, and still count as flat.\n\n"
		"This is only the first sieve - a pin cap is flat too and passes it easily. What separates "
		"the datum from the caps is that the plate is the surface everything else stands on, so it "
		"is the biggest flat population, and the fit settles on it by repeatedly discarding whatever "
		"sits too far off.\n\n"
		"That makes the value far less critical than it looks: on a sample part 10 um and 20 um "
		"agreed to four decimal places. Raise it if too few flat regions are found."));

	//row 1 is the method combo, so 2 puts this straight under it
	form->insertRow(2, _algoH3DatumFlatnessLabel, _algoH3DatumFlatness);

	//both hooks: initAlgoSetupPage wires the page's widgets to the auto-save by walking its
	//children, and that walk has already run by the time this builds them
	connect(_algoH3DatumFlatness, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
		[=](double) { captureAlgoH3ParamsFromUI(); algoSettingsTouched(); });
}

/*
* Only the Flatness setting is method-specific. The datum ROIs are not: under Auto Flat they
* are optional rather than ignored - with none it reads the whole part, with some it reads only
* inside them and still picks the flat pixels itself. So the ROI controls stay on screen for
* every method. The labels are the same for every method too; only the ROI Required value
* changes, so switching method never moves the layout.
*/
void VisionApp::updateAlgoH3DatumMethodUi()
{
	const auto method = (AlgoH3DatumMethod)ui.comboBox_algoH3DatumMethod->currentIndex();
	const bool autoFlat = (method == AlgoH3DatumMethod::AutoFlat);

	if (_algoH3DatumFlatnessLabel) _algoH3DatumFlatnessLabel->setVisible(autoFlat);
	if (_algoH3DatumFlatness) _algoH3DatumFlatness->setVisible(autoFlat);

	//the least number of datum ROIs a Run needs - Auto Flat reads the whole part without any
	ui.lineEdit_algoH3DatumRoiRequired->setText(autoFlat ? QStringLiteral("0") : QStringLiteral("1"));
	const QString why = autoFlat
		? tr("Auto Flat needs no datum ROIs: with none it reads the whole part, with some it reads only inside them.")
		: tr("This method fits the plane inside the datum ROIs, so it needs at least one.");
	ui.label_algoH3DatumRoiRequired->setToolTip(why);
	ui.lineEdit_algoH3DatumRoiRequired->setToolTip(why);

	fitAlgoH3Sections();
}

void VisionApp::rememberAlgoH3Input(bool intensity, const QString& path)
{
	const QString key = intensity ? QStringLiteral("Algo_H3_Recent_Intensity_Map")
		: QStringLiteral("Algo_H3_Recent_Height_Map");
	_systemObj.insert(key, path);
	updateSystemInfo(_systemObj);
}

/*
* Reload the remembered pair, once, the first time the page is opened in this session.
*
* Height first and intensity second, because loading a height map of a different size drops the
* intensity map that no longer lines up with it - do it the other way round and the restore
* throws away the thing it just loaded.
*
* A file that has since been moved or deleted is reported in the status line and nothing more:
* this runs because the operator opened a page, not because they asked for anything, so it has
* no business putting a modal in front of them. The key is left alone so the path stays visible
* and comes back if the folder is remounted.
*/
void VisionApp::restoreAlgoH3Inputs()
{
	if (_algoH3InputsRestored) return;
	_algoH3InputsRestored = true;

	const QString heightPath = jsonHelper::getString(_systemObj,
		QStringLiteral("Algo_H3_Recent_Height_Map"));
	const QString intensityPath = jsonHelper::getString(_systemObj,
		QStringLiteral("Algo_H3_Recent_Intensity_Map"));
	if (heightPath.isEmpty() && intensityPath.isEmpty()) return;

	//a scan loaded in this session outranks anything a previous one left on disk
	if (AlgoManager::instance().height3HasHeight()) return;

	auto& mgr = AlgoManager::instance();
	bool loadedAny = false;

	if (!heightPath.isEmpty()) {
		QString error;
		if (mgr.height3LoadHeightFile(heightPath, error)) {
			ui.lineEdit_algoH3InputLoadHeightStatus->setText(QFileInfo(heightPath).fileName());
			loadedAny = true;
		}
		else {
			ui.lineEdit_algoH3InputLoadHeightStatus->setText(error);
			ct::logger::warn("[Algo H3] Could not reopen the last height map %s: %s",
				heightPath.toStdString().c_str(), error.toStdString().c_str());
		}
	}

	//no height map means nothing for an intensity map to line up against, so do not try
	if (!intensityPath.isEmpty() && mgr.height3HasHeight()) {
		QString error;
		if (mgr.height3LoadIntensityFile(intensityPath, error)) {
			ui.lineEdit_algoH3InputLoadIntensityStatus->setText(QFileInfo(intensityPath).fileName());
			loadedAny = true;
		}
		else {
			ui.lineEdit_algoH3InputLoadIntensityStatus->setText(error);
			ct::logger::warn("[Algo H3] Could not reopen the last intensity map %s: %s",
				intensityPath.toStdString().c_str(), error.toStdString().c_str());
		}
	}

	if (!loadedAny) return;

	_algoH3Output = AlgoHeight3Output();
	refreshAlgoH3RoiBoxes();
	applyAlgoH3Output(_algoH3Output);
	AuditLog::instance().log(QStringLiteral("ALGO_H3_RESTORE_INPUT"),
		QFileInfo(heightPath).fileName());

	/*
	* Put the pipeline back where it left off, rather than making the operator press Run on
	* every section again.
	*
	* RE-RUN, not restore. Only how far it got is kept in the recipe, never the results - a
	* stored result could outlive the settings it was computed from, and the whole pipeline is
	* built on the opposite rule: running a stage invalidates every stage after it, so a number
	* on screen is never older than the data behind it. Re-running reproduces the state and
	* cannot go stale; on the sample part it costs about three seconds.
	*
	* On the worker, like any other run, so the window stays alive while it happens - and it
	* arrives back through the same finished signal, so nothing here has to show it.
	*/
	const int stage = mgr.height3SavedStage();
	if (stage >= 0 && !mgr.isBusy()) {
		showStatus(QStringLiteral("Re-running the stages this recipe had completed..."));
		setAlgoStatus(QStringLiteral("Restoring the last run..."));
		_algoH3RestoreRunPending = true;   //its result must not trigger a save - see height3Finished
		mgr.runHeight3UpTo(stage);
	}
}

/*
* Put one production unit's saved scan on this page, as a pair of files - which is what they
* are, so they are remembered for the next restart like any pair loaded by hand.
*
* Cleared first. Loading a height map keeps an intensity map of the same size, and every unit
* of a run is the same size, so without the clear a unit saved with no intensity map would show
* the previous unit's - a picture of a different part under this one's heights.
*
* The intensity map is optional here: the caller decides what to do without one.
*/
bool VisionApp::algoH3LoadUnitMaps(const QString& heightPath, const QString& intensityPath)
{
	auto& mgr = AlgoManager::instance();
	mgr.height3Clear();
	ui.lineEdit_algoH3InputUseLastScanStatus->clear();

	QString error;
	const bool heightOk = mgr.height3LoadHeightFile(heightPath, error);
	ui.lineEdit_algoH3InputLoadHeightStatus->setText(heightOk ? QFileInfo(heightPath).fileName() : error);

	bool intensityOk = false;
	if (heightOk && QFile::exists(intensityPath)) {
		intensityOk = mgr.height3LoadIntensityFile(intensityPath, error);
		ui.lineEdit_algoH3InputLoadIntensityStatus->setText(intensityOk ? QFileInfo(intensityPath).fileName() : error);
	}
	else {
		ui.lineEdit_algoH3InputLoadIntensityStatus->setText(heightOk
			? QStringLiteral("No intensity map saved for this unit") : QString());
	}

	//a unit that failed to load leaves the remembered pair alone, for the next restart to reopen
	if (heightOk) {
		rememberAlgoH3Input(false, heightPath);
		rememberAlgoH3Input(true, intensityOk ? intensityPath : QString());
	}

	//whatever was on the page belonged to another map
	_algoH3Output = AlgoHeight3Output();
	refreshAlgoH3RoiBoxes();
	applyAlgoH3Output(_algoH3Output);
	updateAlgoH3Display();
	updateAlgoH3Enables();

	if (!heightOk) {
		showMsg(error);
		return false;
	}
	AuditLog::instance().log(QStringLiteral("ALGO_H3_LOAD_UNIT"), QFileInfo(heightPath).fileName());
	return true;
}

void VisionApp::updateAlgoH3Display()
{
	if (!isPage(UIPage::ALGO_SETUP)) return;
	if (currentAlgoPageAlgo() != AlgoPageAlgo::HEIGHT_3D_V3) return;

	auto& mgr = AlgoManager::instance();

	bool preprocessed = false, segmented = false;
	algoH3ShownStage(preprocessed, segmented);

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
			_algoH3ZExaggeration, kAlgoH3SurfaceCanvas, h3StyleFor(mode),
			ui.checkBox_algoH3FillHoles->isChecked(),
			ui.checkBox_algoH3ShowPlane->isChecked());
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
	//the hover readout reads the same map this view was drawn from; a 3D render is a
	//projection, so it has no map pixels to read
	_fovH3Preprocessed = preprocessed;
	_fovH3Segmented = segmented;
	displayFOV(img, h3IsSurfaceMode(mode) ? FovPixelSource::None : FovPixelSource::Height3);

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

	/*
	* Drawn on a FAIL too, in red. doSegment publishes the outline it found BEFORE it applies the
	* canvas and the width / height / angle checks, so a part that failed one of those still has
	* corners - and seeing what was found is how the operator tells a half-found part from a wrong
	* limit. Only a method that found nothing at all leaves no corners, and then nothing is drawn.
	*/
	if (section == SEC_SEG && !segmented && _algoH3Output.segment.ran
		&& _algoH3Output.segCorners.size() == 4) {
		QPolygonF poly;
		for (const auto& pt : _algoH3Output.segCorners) poly << pt;
		//a 10000 px map is shown at a tenth of that or less, where a 2 px image line is a
		//fraction of a screen pixel - so this outline is sized in screen pixels instead
		auto outline = AlgoOverlayItem::makePoly(poly,
			_algoH3Output.segment.pass ? QColor(0, 255, 127) : kAlgoH3FailColor);
		outline.penWidth = 3.0;
		outline.cosmeticPen = true;
		overlay.append(outline);
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

	bool preprocessed = false, segmented = false;
	algoH3ShownStage(preprocessed, segmented);

	const QImage img = mgr.height3Surface(preprocessed, segmented, _algoH3Yaw, _algoH3Pitch,
		_algoH3ZExaggeration, kAlgoH3SurfaceCanvas, h3StyleFor(mode),
		ui.checkBox_algoH3FillHoles->isChecked(),
		ui.checkBox_algoH3ShowPlane->isChecked());
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
	const bool preReady = mgr.height3PreprocessReady();
	const bool segReady = mgr.height3SegmentReady();
	const bool datumReady = mgr.height3DatumReady();
	const bool measured = _algoH3Output.measure.ran && _algoH3Output.measure.pass;

	//each Run is gated on the stage before it having actually passed, so the pipeline
	//can never be entered halfway with stale state behind it
	ui.toolButton_algoH3PreprocessRun->setEnabled(!busy && haveHeight);
	ui.toolButton_algoH3SegRun->setEnabled(!busy && preReady);
	ui.toolButton_algoH3DatumRun->setEnabled(!busy && segReady);
	ui.toolButton_algoH3ResultRun->setEnabled(!busy && datumReady);
	ui.toolButton_algoH3OverallRun->setEnabled(!busy && measured);

	ui.toolButton_algoH3DatumAddRoi->setEnabled(!busy && segReady);
	ui.toolButton_algoH3DatumDeleteRoi->setEnabled(!busy && segReady);
	ui.toolButton_algoH3RoiAdd->setEnabled(!busy && segReady);
	ui.toolButton_algoH3RoiDelete->setEnabled(!busy && segReady);
	ui.toolButton_algoH3RoiAssignType->setEnabled(!busy && segReady);
	ui.toolButton_algoH3RoiAutoGrid->setEnabled(!busy && segReady);

	//the external display shows whichever map is on screen, so Open needs one; while a job
	//is running (it can take ~25 s) neither button may start a second
	const bool extBusy = _external3D && _external3D->busy();
	ui.toolButton_algoH3DisplayExternalOpen->setEnabled(!busy && haveHeight && !extBusy);
	ui.toolButton_algoH3DisplayExternalClose->setEnabled(!extBusy);
}

/*
* The external 3D display's please-wait box. Shown a beat AFTER the job starts, so a job
* that is over at once - Close with nothing open - does not flash a box. The timer checks it
* is still the current box before showing it: finished() can land inside that beat, and a
* box taken down there must not pop up afterwards.
*/
void VisionApp::showExternal3DBusy(const QString& text)
{
	hideExternal3DBusy();

	auto* dlg = new H3BusyDialog(text, this);
	_external3DBusy = dlg;
	QTimer::singleShot(300, dlg, [this, dlg]() {
		if (_external3DBusy.data() == dlg) dlg->show();
	});
}

void VisionApp::hideExternal3DBusy()
{
	if (!_external3DBusy) return;
	_external3DBusy->hide();
	_external3DBusy->deleteLater();
	_external3DBusy = nullptr;
}

// =============================================================================
// ROI type table
// =============================================================================

//the colour button's property is the single source both capture and Add ROI read, exactly
//as the table's cell widget used to be
QColor VisionApp::algoH3SelectedTypeColor() const
{
	const QVariant v = ui.toolButton_algoH3RoiTypeColor->property(kH3ColorProp);
	return v.canConvert<QColor>() ? v.value<QColor>() : QColor(0, 200, 0);
}

int VisionApp::algoH3RoiCountForType(const QString& name) const
{
	//the boxes are the live truth while a crop is on screen; before segmentation there are
	//no boxes at all and the recipe is all there is
	if (_algoH3BoxCropW > 0) {
		int n = 0;
		for (auto* b : _algoH3RoiBoxes) if (b && b->getTag() == name) n++;
		return n;
	}
	int n = 0;
	for (const auto& r : AlgoManager::instance().height3Params().rois)
		if (r.typeName == name) n++;
	return n;
}

void VisionApp::updateAlgoH3TypeStatus()
{
	if (!ui.lineEdit_algoH3RoiTypeStatus) return;

	const QString name = ui.comboBox_algoH3RoiType->currentText();
	ui.lineEdit_algoH3RoiTypeCount->setText(
		name.isEmpty() ? QString() : QString::number(algoH3RoiCountForType(name)));

	//say the next useful thing rather than a fixed caption - this line is the only place
	//left to explain a section that no longer has a table to read
	QString msg;
	if (ui.comboBox_algoH3RoiType->count() == 0)
		msg = QStringLiteral("Add an ROI type first - every ROI belongs to one.");
	else if (!AlgoManager::instance().height3SegmentReady())
		msg = QStringLiteral("Run segmentation before adding ROIs.");
	else
		msg = QStringLiteral("Add ROI creates one of '%1'. Select ROIs on the image to "
			"re-assign, copy (Ctrl+C) or delete them.").arg(name);

	h3SetMessage(ui.lineEdit_algoH3RoiTypeStatus, msg);
}

void VisionApp::loadAlgoH3TypeFields()
{
	const AlgoHeight3Params p = AlgoManager::instance().height3Params();
	const int i = ui.comboBox_algoH3RoiType->currentIndex();
	_algoH3TypeIndex = (i >= 0 && i < p.roiTypes.size()) ? i : -1;

	const bool have = (_algoH3TypeIndex >= 0);
	ui.toolButton_algoH3RoiTypeColor->setEnabled(have);
	ui.checkBox_algoH3RoiTypeEnableHeightCheck->setEnabled(have);
	ui.doubleSpinBox_algoH3RoiTypeMinUm->setEnabled(have);
	ui.doubleSpinBox_algoH3RoiTypeMaxUm->setEnabled(have);
	ui.checkBox_algoH3RoiTypeEnableOffsetCheck->setEnabled(have);
	ui.doubleSpinBox_algoH3RoiTypeMaxOffsetUm->setEnabled(have);
	ui.comboBox_algoH3RoiTypeMethod->setEnabled(have);

	_algoH3Updating = true;
	{
		//every one of these is also wired to the generic auto-save sweep in
		//VisionApp_AlgoSetup, which does not know about _algoH3Updating - blocking is what
		//stops merely SHOWING a type from marking the recipe dirty
		QSignalBlocker b1(ui.doubleSpinBox_algoH3RoiTypeMinUm);
		QSignalBlocker b2(ui.doubleSpinBox_algoH3RoiTypeMaxUm);
		QSignalBlocker b3(ui.comboBox_algoH3RoiTypeMethod);
		QSignalBlocker b4(ui.checkBox_algoH3RoiTypeEnableHeightCheck);
		QSignalBlocker b5(ui.checkBox_algoH3RoiTypeEnableOffsetCheck);
		QSignalBlocker b6(ui.doubleSpinBox_algoH3RoiTypeMaxOffsetUm);

		const AlgoH3RoiType t = have ? p.roiTypes[_algoH3TypeIndex] : AlgoH3RoiType();
		ui.checkBox_algoH3RoiTypeEnableHeightCheck->setChecked(have && t.checkHeight);
		ui.doubleSpinBox_algoH3RoiTypeMinUm->setValue(have ? t.minUm : 0.0);
		ui.doubleSpinBox_algoH3RoiTypeMaxUm->setValue(have ? t.maxUm : 0.0);
		ui.checkBox_algoH3RoiTypeEnableOffsetCheck->setChecked(have && t.checkOffset);
		ui.doubleSpinBox_algoH3RoiTypeMaxOffsetUm->setValue(have ? t.maxOffsetUm : 0.0);
		ui.comboBox_algoH3RoiTypeMethod->setCurrentIndex(
			(t.methodId >= 0 && t.methodId < ui.comboBox_algoH3RoiTypeMethod->count())
			? t.methodId : 0);

		//the colour lives on the button as a property, the way it used to live on the
		//table's cell widget - it is the single source capture and Add ROI both read
		ui.toolButton_algoH3RoiTypeColor->setProperty(kH3ColorProp, t.color);
		ui.toolButton_algoH3RoiTypeColor->setStyleSheet(have
			? QStringLiteral("QToolButton { background:%1; border:1px solid #777; }").arg(t.color.name())
			: QString());
	}
	_algoH3Updating = false;

	updateAlgoH3TypeStatus();
}

void VisionApp::refreshAlgoH3TypeList()
{
	auto* cb = ui.comboBox_algoH3RoiType;
	if (!cb) return;

	const AlgoHeight3Params p = AlgoManager::instance().height3Params();

	//keep the selection by NAME, not by index: deleting a type shifts every index after it
	const QString keep = cb->currentText();

	_algoH3Updating = true;
	{
		QSignalBlocker b(cb);
		cb->clear();
		for (const auto& t : p.roiTypes) cb->addItem(h3ColorIcon(t.color), t.name);

		const int at = cb->findText(keep);
		cb->setCurrentIndex(at >= 0 ? at : (cb->count() > 0 ? 0 : -1));
	}
	_algoH3Updating = false;

	loadAlgoH3TypeFields();
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
	p.segCanvasWidthUm = ui.doubleSpinBox_algoH3SegCanvasWidthUm->value();
	p.segCanvasHeightUm = ui.doubleSpinBox_algoH3SegCanvasHeightUm->value();
	if (_algoH3SegBandStep) p.segBandStepRaw = _algoH3SegBandStep->value();
	if (_algoH3SegBandMin) p.segBandMinUm = _algoH3SegBandMin->value();
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
	if (_algoH3DatumFlatness) p.datumFlatnessUm = _algoH3DatumFlatness->value();
	p.datumCheckTilt = ui.checkBox_algoH3DatumEnableTiltCheck->isChecked();
	p.datumMaxTiltDeg = ui.doubleSpinBox_algoH3DatumMaxTiltDeg->value();

	// ── section 4 ──
	p.methodId = ui.comboBox_algoH3Method->currentIndex();
	p.percentile = ui.doubleSpinBox_algoH3MethodPercentileValue->value();

	/*
	* ── section 5: only ONE type is on screen, so only that one can be written back ──
	*
	* The others are left exactly as the recipe holds them. This is why _algoH3TypeIndex
	* exists rather than reading the combo: currentIndexChanged fires AFTER the index has
	* moved, so a capture triggered by switching types has to write the type the widgets
	* still hold - the OUTGOING one - or it would stamp its values onto the incoming type.
	*/
	if (_algoH3TypeIndex >= 0 && _algoH3TypeIndex < p.roiTypes.size()) {
		AlgoH3RoiType& t = p.roiTypes[_algoH3TypeIndex];
		//the name is the key ROIs refer to and is never editable, so it is not touched here
		t.color = algoH3SelectedTypeColor();
		t.checkHeight = ui.checkBox_algoH3RoiTypeEnableHeightCheck->isChecked();
		t.minUm = ui.doubleSpinBox_algoH3RoiTypeMinUm->value();
		t.maxUm = ui.doubleSpinBox_algoH3RoiTypeMaxUm->value();
		t.checkOffset = ui.checkBox_algoH3RoiTypeEnableOffsetCheck->isChecked();
		t.maxOffsetUm = ui.doubleSpinBox_algoH3RoiTypeMaxOffsetUm->value();
		t.methodId = ui.comboBox_algoH3RoiTypeMethod->currentIndex();
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
	refreshAlgoH3AlgoCombo();
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
		QSignalBlocker bc1(ui.doubleSpinBox_algoH3SegCanvasWidthUm);
		QSignalBlocker bc2(ui.doubleSpinBox_algoH3SegCanvasHeightUm);
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
		ui.doubleSpinBox_algoH3SegCanvasWidthUm->setValue(p.segCanvasWidthUm);
		ui.doubleSpinBox_algoH3SegCanvasHeightUm->setValue(p.segCanvasHeightUm);
		if (_algoH3SegBandStep) {
			QSignalBlocker bb1(_algoH3SegBandStep);
			_algoH3SegBandStep->setValue(p.segBandStepRaw);
		}
		if (_algoH3SegBandMin) {
			QSignalBlocker bb2(_algoH3SegBandMin);
			_algoH3SegBandMin->setValue(p.segBandMinUm);
		}
		updateAlgoH3CanvasPxLabel();
		updateAlgoH3SegMethodUi();
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
		if (_algoH3DatumFlatness) {
			QSignalBlocker b4(_algoH3DatumFlatness);
			_algoH3DatumFlatness->setValue(p.datumFlatnessUm);
		}
	}
	updateAlgoH3DatumMethodUi();

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

	refreshAlgoH3TypeList();
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
		h3SetMessage(ui.label_algoH3ResultInstruction,
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

	h3SetMessage(ui.label_algoH3ResultInstruction,
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
	setAlgoStatus("Running...");

	mgr.runHeight3(stage);

	static const char* kStageNames[] = { "PREPROCESS", "SEGMENT", "DATUM", "MEASURE", "OVERALL", "ALL" };
	const int s = (int)stage;
	AuditLog::instance().log(QStringLiteral("ALGO_H3_RUN"),
		QString::fromLatin1((s >= 0 && s <= (int)AlgoH3Stage::All) ? kStageNames[s] : "?"));
}
