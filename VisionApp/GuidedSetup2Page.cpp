//
//  Guided Setup (new): the right-panel page. Build step 1 = the recipe file, Save / Discard and the
//  typed settings; build step 2 = the Camera and circle finder tools; build step 3 = the teach
//  buttons of sections A-D and the Locate test; build step 4a = sections E, F and H's Set. The
//  visits arrive in build step 4b.
//
#include "GuidedSetup2Page.h"
#include "ScaleManager.h"
#include "Logger.h"

#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QInputDialog>
#include <QMessageBox>
#include <QPushButton>
#include <QSpinBox>
#include <algorithm>
#include <cmath>

namespace {

const double kPi = 3.14159265358979323846;
const QString kDoneColour = QStringLiteral("color:#7CD992;");
const QString kTodoColour = QStringLiteral("color:#FFB347;");

QString mm3(double v) { return QString::number(v, 'f', 3); }

//a spin box value rounded to the decimals it shows: 0.5 plus ten 0.05 steps reads 1.0000000000000004
double shown(const QDoubleSpinBox* w)
{
	const double f = std::pow(10.0, w->decimals());
	return std::round(w->value() * f) / f;
}

QString xyText(const gs2::TaughtXY& p, const QString& prefix = QString())
{
	if (!p.set) return QStringLiteral("not taught");
	return QStringLiteral("%1X %2   %1Y %3").arg(prefix, mm3(p.x), mm3(p.y));
}

} // namespace

GuidedSetup2Page::GuidedSetup2Page(QWidget* parent)
	: QWidget(parent)
{
	ui.setupUi(this);
	ui.label_banner->hide();
	connectEdits();

	connect(ui.toolButton_showLayout, &QToolButton::toggled, this, [=](bool on) {
		ui.toolButton_showLayout->setText(on ? QStringLiteral("Show camera") : QStringLiteral("Show layout"));
		emit showLayoutToggled(on);
	});
	connect(ui.toolButton_save, &QToolButton::clicked, this, [=]() { save(); });
	connect(ui.toolButton_discard, &QToolButton::clicked, this, [=]() { discard(); });
	connect(ui.toolButton_gNameAll, &QToolButton::clicked, this, [=]() {
		if (_work.units.isEmpty() && _work.slotList.isEmpty()) return;
		const auto reply = QMessageBox::question(this, QStringLiteral("Name all by position"),
			QStringLiteral("Rename every slot (S1, S2...) and every unit (X1Y1...) by its position?\n\nCustom names will be lost."),
			QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
		if (reply != QMessageBox::Yes) return;
		gs2::nameAllByPosition(_work);
		refreshStatus();
		emit setupChanged();
	});

	//tools: not part of the recipe, so they never mark the page as edited
	//the settings shown are the picked kind's; switching kinds is not an edit (fillWidgets blocks it)
	connect(ui.comboBox_tCircle, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [=]() { fillWidgets(); });
	connect(ui.toolButton_tStability, &QToolButton::clicked, this, [=]() { emit toolRequested(ToolStability); });
	connect(ui.toolButton_tFindImage, &QToolButton::clicked, this, [=]() { emit toolRequested(ToolFindInImage); });
	connect(ui.toolButton_tSnapFind, &QToolButton::clicked, this, [=]() { emit toolRequested(ToolSnapFind); });
	connect(ui.toolButton_tCentre, &QToolButton::clicked, this, [=]() { emit toolRequested(ToolCentre); });
	connect(ui.toolButton_tDirection, &QToolButton::clicked, this, [=]() { emit toolRequested(ToolDirection); });
	connect(ui.toolButton_tSaveMeasured, &QToolButton::clicked, this, [=]() { emit toolRequested(ToolSaveMeasured); });
	//activated = the operator's pick only; VisionApp confirms, saves and sets the combo back if refused
	connect(ui.comboBox_tConversion, QOverload<int>::of(&QComboBox::activated), this, [=](int i) { emit conversionChosen(i == 1); });
	ui.toolButton_tSaveMeasured->setEnabled(false);

	//A and B: the machine does the teaching; deleting an empty image is only an edit (the file goes on Save)
	connect(ui.toolButton_aSnapEmpty, &QToolButton::clicked, this, [=]() { emit teachRequested(TeachSnapEmpty); });
	connect(ui.toolButton_aScore, &QToolButton::clicked, this, [=]() { emit teachRequested(TeachScore); });
	connect(ui.toolButton_aDelete, &QToolButton::clicked, this, [=]() {
		const int row = ui.listWidget_aImages->currentRow();
		if (row < 0 || row >= _work.safety1.emptyImages.size()) return;
		_work.safety1.emptyImages.removeAt(row);
		refreshStatus();
		emit setupChanged();
	});
	connect(ui.toolButton_bTeachXY, &QToolButton::clicked, this, [=]() { emit teachRequested(TeachXY); });
	connect(ui.toolButton_bJogXY, &QToolButton::clicked, this, [=]() { emit teachRequested(TeachJogXY); });
	connect(ui.toolButton_bSetStart, &QToolButton::clicked, this, [=]() { emit teachRequested(TeachSetStart); });
	connect(ui.toolButton_bSetStop, &QToolButton::clicked, this, [=]() { emit teachRequested(TeachSetStop); });
	connect(ui.toolButton_bLearn, &QToolButton::clicked, this, [=]() { emit teachRequested(TeachLearnPattern); });
	connect(ui.toolButton_bFind, &QToolButton::clicked, this, [=]() { emit teachRequested(TeachFind); });
	connect(ui.toolButton_bPatternBox, &QToolButton::toggled, this, [=](bool on) { emit patternBoxToggled(on); });

	//C, D and the Locate test
	connect(ui.toolButton_cSetSpot, &QToolButton::clicked, this, [=]() { emit teachRequested(TeachZ3DSpot); });
	connect(ui.toolButton_cFind, &QToolButton::clicked, this, [=]() { emit teachRequested(TeachZ3DFind); });
	connect(ui.toolButton_cSetDiff, &QToolButton::clicked, this, [=]() { emit teachRequested(TeachZ3DDiff); });
	connect(ui.toolButton_dTeachTL, &QToolButton::clicked, this, [=]() { emit teachRequested(TeachCornerTL); });
	connect(ui.toolButton_dTeachTR, &QToolButton::clicked, this, [=]() { emit teachRequested(TeachCornerTR); });
	connect(ui.toolButton_dTeachBR, &QToolButton::clicked, this, [=]() { emit teachRequested(TeachCornerBR); });
	connect(ui.toolButton_dTeachBL, &QToolButton::clicked, this, [=]() { emit teachRequested(TeachCornerBL); });
	connect(ui.toolButton_dJogTL, &QToolButton::clicked, this, [=]() { emit teachRequested(TeachJogCornerTL); });
	connect(ui.toolButton_dJogTR, &QToolButton::clicked, this, [=]() { emit teachRequested(TeachJogCornerTR); });
	connect(ui.toolButton_dJogBR, &QToolButton::clicked, this, [=]() { emit teachRequested(TeachJogCornerBR); });
	connect(ui.toolButton_dJogBL, &QToolButton::clicked, this, [=]() { emit teachRequested(TeachJogCornerBL); });
	connect(ui.toolButton_dCheck, &QToolButton::clicked, this, [=]() { emit teachRequested(TeachRectCheck); });
	connect(ui.toolButton_iLocate, &QToolButton::clicked, this, [=]() { emit teachRequested(TeachLocate); });

	//E, F and H: the machine measures; Delete and Rename are only edits
	connect(ui.toolButton_eSetTL, &QToolButton::clicked, this, [=]() { emit teachRequested(TeachSlotTL); });
	connect(ui.toolButton_eSetBR, &QToolButton::clicked, this, [=]() { emit teachRequested(TeachSlotBR); });
	connect(ui.toolButton_fAdd, &QToolButton::clicked, this, [=]() { emit teachRequested(TeachSlotAdd); });
	connect(ui.toolButton_fInterp, &QToolButton::clicked, this, [=]() { emit teachRequested(TeachSlotInterpolate); });
	connect(ui.toolButton_fDelete, &QToolButton::clicked, this, [=]() { deleteSlots(); });
	connect(ui.toolButton_fRename, &QToolButton::clicked, this, [=]() { renameSlot(); });
	connect(ui.toolButton_hSet2D, &QToolButton::clicked, this, [=]() { emit teachRequested(TeachInsp2D); });
	connect(ui.toolButton_hSetBc, &QToolButton::clicked, this, [=]() { emit teachRequested(TeachInspBarcodeOcr); });
	connect(ui.toolButton_hSet3D, &QToolButton::clicked, this, [=]() { emit teachRequested(TeachInsp3D); });
	ui.listWidget_fSlots->setSelectionMode(QAbstractItemView::ExtendedSelection);
	connect(ui.listWidget_fSlots, &QListWidget::itemSelectionChanged, this, [=]() { emit slotSelectionChanged(selectedSlotIds()); });

	//a number typed into a box counts only on Enter or when the box loses focus, and a tool button does not
	//take the focus: take it as the button goes down, so Save, Check or Interpolate never use the old number
	for (auto* b : findChildren<QToolButton*>())
		connect(b, &QToolButton::pressed, this, [=]() {
			if (auto* box = qobject_cast<QAbstractSpinBox*>(QApplication::focusWidget()))
				if (isAncestorOf(box)) box->interpretText();
		});

	fillWidgets();
}

int GuidedSetup2Page::slotAddCorner() const
{
	return std::clamp(ui.comboBox_fCorner->currentIndex(), 0, 3);
}

QStringList GuidedSetup2Page::selectedSlotIds() const
{
	QStringList ids;
	for (const auto* item : ui.listWidget_fSlots->selectedItems()) {
		const QString id = item->data(Qt::UserRole).toString();
		if (!id.isEmpty()) ids << id;
	}
	return ids;
}

int GuidedSetup2Page::slotsBetween() const
{
	return ui.spin_fBetween->value();
}

void GuidedSetup2Page::selectSlots(const QStringList& ids)
{
	QSignalBlocker b(ui.listWidget_fSlots);
	QListWidgetItem* first = nullptr;
	for (int i = 0; i < ui.listWidget_fSlots->count(); i++) {
		auto* item = ui.listWidget_fSlots->item(i);
		const bool on = ids.contains(item->data(Qt::UserRole).toString());
		item->setSelected(on);
		if (on && !first) first = item;
	}
	if (first) ui.listWidget_fSlots->scrollToItem(first);
}

void GuidedSetup2Page::deleteSlots()
{
	const QStringList ids = selectedSlotIds();
	if (ids.isEmpty()) { showSlotsResult(QStringLiteral("Select the slots to delete in the list first."), false); return; }
	QStringList names;
	for (const auto& id : ids) if (const auto* sl = gs2::findSlot(_work, id)) names << sl->name;
	const auto reply = QMessageBox::question(this, QStringLiteral("Delete slots"),
		QStringLiteral("Delete %1 and their units?\n\nDiscard brings them back until you Save.").arg(names.join(QStringLiteral(", "))),
		QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
	if (reply != QMessageBox::Yes) return;
	modify([&](gs2::Setup& s) {
		for (int i = s.slotList.size() - 1; i >= 0; i--) if (ids.contains(s.slotList[i].id)) s.slotList.removeAt(i);
		gs2::syncUnits(s);
	});
	showSlotsResult(QStringLiteral("Deleted %1.").arg(names.join(QStringLiteral(", "))), false);
	ct::logger::info("[GuidedSetup2] Slots deleted: %s", qPrintable(names.join(QStringLiteral(", "))));
}

void GuidedSetup2Page::renameSlot()
{
	const QStringList ids = selectedSlotIds();
	if (ids.size() != 1) { showSlotsResult(QStringLiteral("Select one slot in the list to rename."), false); return; }
	const auto* sl = gs2::findSlot(_work, ids.first());
	if (!sl) return;
	const QString was = sl->name;
	bool ok = false;
	const QString name = QInputDialog::getText(this, QStringLiteral("Rename slot"), QStringLiteral("New name for %1:").arg(was),
		QLineEdit::Normal, was, &ok).trimmed();
	if (!ok || name == was) return;
	if (name.isEmpty()) { showSlotsResult(QStringLiteral("A slot needs a name."), false); return; }
	for (const auto& other : _work.slotList)
		if (other.id != ids.first() && other.name == name) { showSlotsResult(QStringLiteral("%1 is already a slot's name.").arg(name), false); return; }
	modify([&](gs2::Setup& s) { for (auto& other : s.slotList) if (other.id == ids.first()) other.name = name; });
	showSlotsResult(QStringLiteral("%1 renamed %2.").arg(was, name), false);
}

void GuidedSetup2Page::modify(const std::function<void(gs2::Setup&)>& change)
{
	const gs2::Setup before = _work;
	change(_work);
	keepResults(before, _work);
	fillWidgets();
	emit setupChanged();
}

void GuidedSetup2Page::keepResults(const gs2::Setup& before, gs2::Setup& s) const
{
	if (!gs2::sameFid0Settings(before, s)) s.fid0.found = gs2::sameFid0Settings(s, _saved) ? _saved.fid0.found : gs2::Fid0Found();
	if (!gs2::sameZ3DSettings(before, s)) s.z3d.found = gs2::sameZ3DSettings(s, _saved) ? _saved.z3d.found : gs2::Z3DFound();
	if (!gs2::sameFidRectSettings(before, s)) s.fidRect.found = gs2::sameFidRectSettings(s, _saved) ? _saved.fidRect.found : gs2::FidRectFound();
}

int GuidedSetup2Page::circleKind() const
{
	return std::clamp(ui.comboBox_tCircle->currentIndex(), 0, (int)gs2::CircleKindCount - 1);
}

QString GuidedSetup2Page::circleSummary(int kind) const
{
	const gs2::CircleFinder& c = _work.circle[kind];
	if (!c.dia.valid()) return QStringLiteral("Diameter not set: Camera and circle finder, %1").arg(gs2::circleKindName(kind));
	static const char* const polarity[3] = { "bright", "dark", "bright or dark" };
	return QStringLiteral("%1 .. %2 mm %3, %4, min score %5 (Camera and circle finder)")
		.arg(QString::number(c.dia.minMm, 'f', 2), QString::number(c.dia.maxMm, 'f', 2), pxText(c.dia.minMm, c.dia.maxMm),
			QLatin1String(polarity[(c.polarity >= 0 && c.polarity < 3) ? c.polarity : 0])).arg(c.minScore);
}

void GuidedSetup2Page::recordResult(const char* what, const std::function<void(gs2::Setup&)>& put, bool (*same)(const gs2::Setup&, const gs2::Setup&))
{
	put(_work);
	//only over a file that loaded (or was saved) cleanly, and only when the file holds these very settings
	if (_loadResult == gs2::LoadResult::Loaded && !_recipeDir.isEmpty() && same(_work, _saved)) {
		gs2::Setup s = _saved; //the file as it is, plus this result - other unsaved edits stay unsaved
		put(s);
		QString why;
		if (gs2::save(_recipeDir, s, why)) {
			_saved = s;
			ct::logger::info("[GuidedSetup2] %s result saved to %s", what, gs2::filePath(_recipeDir).toStdString().c_str());
		}
		else ct::logger::error("[GuidedSetup2] %s result NOT saved (%s): it stays on the page as an unsaved change", what, why.toStdString().c_str());
	}
	else ct::logger::info("[GuidedSetup2] %s result kept with the unsaved settings it ran with: Save or Discard decides both", what);
	fillWidgets();
	emit setupChanged();
}

void GuidedSetup2Page::recordFid0Found(const gs2::Fid0Found& found)
{
	recordResult("fid0 Find", [&](gs2::Setup& s) { s.fid0.found = found; }, gs2::sameFid0Settings);
}

void GuidedSetup2Page::recordZ3DFound(const gs2::Z3DFound& found)
{
	recordResult("3D Z Find", [&](gs2::Setup& s) { s.z3d.found = found; }, gs2::sameZ3DSettings);
}

void GuidedSetup2Page::recordFidRectFound(const gs2::FidRectFound& found)
{
	recordResult("fidRect Check", [&](gs2::Setup& s) { s.fidRect.found = found; }, gs2::sameFidRectSettings);
}

void GuidedSetup2Page::showSafetyScore(const QString& text, bool ok)
{
	ui.label_aResult->setText(text);
	ui.label_aResult->setStyleSheet(ok ? QString() : kTodoColour);
}

void GuidedSetup2Page::showFid0Result(const QString& text, bool ok)
{
	ui.label_bFound->setText(text);
	ui.label_bFound->setStyleSheet(ok ? kDoneColour : kTodoColour);
}

void GuidedSetup2Page::showZ3DResult(const QString& text, bool ok)
{
	ui.label_cFound->setText(text);
	ui.label_cFound->setStyleSheet(ok ? kDoneColour : kTodoColour);
}

void GuidedSetup2Page::showRectResult(const QString& text, bool ok)
{
	ui.label_dFound->setText(text);
	ui.label_dFound->setStyleSheet(ok ? kDoneColour : kTodoColour);
}

void GuidedSetup2Page::showLocateResult(const QString& text, bool ok)
{
	ui.label_iResult->setText(text);
	ui.label_iResult->setStyleSheet(ok ? kDoneColour : kTodoColour);
}

void GuidedSetup2Page::showSlotSizeResult(const QString& text, bool ok)
{
	ui.label_eFound->setText(text);
	ui.label_eFound->setStyleSheet(ok ? kDoneColour : kTodoColour);
}

void GuidedSetup2Page::showSlotsResult(const QString& text, bool ok)
{
	ui.label_fFound->setText(text);
	ui.label_fFound->setStyleSheet(ok ? kDoneColour : kTodoColour);
}

void GuidedSetup2Page::showInspResult(const QString& text, bool ok)
{
	ui.label_hFound->setText(text);
	ui.label_hFound->setStyleSheet(ok ? kDoneColour : kTodoColour);
}

void GuidedSetup2Page::setPatternBoxShown(bool shown)
{
	QSignalBlocker b(ui.toolButton_bPatternBox);
	ui.toolButton_bPatternBox->setChecked(shown);
}

void GuidedSetup2Page::setCameraConversion(bool useMeasured, const QString& text, bool warn, bool canSaveMeasured)
{
	{
		QSignalBlocker b(ui.comboBox_tConversion);
		ui.comboBox_tConversion->setCurrentIndex(useMeasured ? 1 : 0);
	}
	ui.label_tConversion->setText(text);
	ui.label_tConversion->setStyleSheet(warn ? kTodoColour : QString());
	_canSaveMeasured = canSaveMeasured;
	ui.toolButton_tSaveMeasured->setEnabled(canSaveMeasured && ui.toolButton_tDirection->isEnabled());
}

gs2::CircleFinder GuidedSetup2Page::toolCircle(QString* what) const
{
	const int kind = circleKind();
	if (what) *what = gs2::circleKindName(kind);
	return _work.circle[kind];
}

void GuidedSetup2Page::showToolResult(const QString& text, bool ok)
{
	ui.label_tResult->setText(text);
	ui.label_tResult->setStyleSheet(ok ? QString() : kTodoColour);
}

void GuidedSetup2Page::setMachineBusy(bool busy)
{
	ui.comboBox_tCircle->setEnabled(!busy);
	ui.toolButton_tFindImage->setEnabled(!busy);
	ui.toolButton_tSnapFind->setEnabled(!busy);
	ui.toolButton_tCentre->setEnabled(!busy);
	ui.toolButton_tDirection->setEnabled(!busy);
	ui.comboBox_tConversion->setEnabled(!busy);
	ui.toolButton_tSaveMeasured->setEnabled(!busy && _canSaveMeasured);
	ui.toolButton_tStability->setEnabled(!busy);
	for (auto* b : { ui.toolButton_aSnapEmpty, ui.toolButton_aScore, ui.toolButton_aDelete, ui.toolButton_bTeachXY, ui.toolButton_bJogXY,
		ui.toolButton_bSetStart, ui.toolButton_bSetStop, ui.toolButton_bLearn, ui.toolButton_bFind,
		ui.toolButton_cSetSpot, ui.toolButton_cFind, ui.toolButton_cSetDiff,
		ui.toolButton_dTeachTL, ui.toolButton_dTeachTR, ui.toolButton_dTeachBR, ui.toolButton_dTeachBL, ui.toolButton_dCheck,
		ui.toolButton_dJogTL, ui.toolButton_dJogTR, ui.toolButton_dJogBR, ui.toolButton_dJogBL,
		ui.toolButton_eSetTL, ui.toolButton_eSetBR, ui.toolButton_fAdd, ui.toolButton_fInterp, ui.toolButton_fDelete, ui.toolButton_fRename,
		ui.toolButton_hSet2D, ui.toolButton_hSetBc, ui.toolButton_hSet3D,
		ui.toolButton_iLocate, ui.toolButton_save, ui.toolButton_discard })
		b->setEnabled(!busy);
	if (!busy) refreshStatus(); //Save / Discard follow the edits again
}

void GuidedSetup2Page::connectEdits()
{
	for (auto* w : findChildren<QDoubleSpinBox*>())
		connect(w, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [=]() { edited(); });
	//F's corner and count say what Add / Interpolate do next; they are not recipe settings
	for (auto* w : findChildren<QSpinBox*>())
		if (w != ui.spin_fBetween)
			connect(w, QOverload<int>::of(&QSpinBox::valueChanged), this, [=]() { edited(); });
	for (auto* w : findChildren<QCheckBox*>())
		connect(w, &QCheckBox::toggled, this, [=]() { edited(); });
	for (auto* w : findChildren<QComboBox*>())
		if (w != ui.comboBox_tCircle && w != ui.comboBox_tConversion && w != ui.comboBox_fCorner)
			connect(w, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [=]() { edited(); });
}

void GuidedSetup2Page::loadRecipe(const QString& recipeDir, const QString& recipeName)
{
	_recipeDir = recipeDir;
	_recipeName = recipeName;
	_loadResult = gs2::load(recipeDir, _saved, _loadWhy);
	_mayClean = _loadResult == gs2::LoadResult::Loaded;
	if (_loadResult == gs2::LoadResult::Unreadable)
		ct::logger::error("[GuidedSetup2] %s", _loadWhy.toStdString().c_str());
	else
		ct::logger::info("[GuidedSetup2] Recipe %s: %s", recipeName.toStdString().c_str(),
			_loadResult == gs2::LoadResult::Loaded
				? (gs2::isComplete(_saved) ? "guidedSetup2.json loaded, complete" : "guidedSetup2.json loaded, not complete")
				: "no guidedSetup2.json yet");
	_work = _saved;
	//results shown for the previous recipe do not belong to this one
	ui.label_aResult->setText(QStringLiteral("No score yet."));
	ui.label_aResult->setStyleSheet(QString());
	ui.label_bFound->setText(QStringLiteral("Find has not run with these settings."));
	ui.label_bFound->setStyleSheet(QString());
	ui.label_cFound->setText(QStringLiteral("Find has not run with these settings."));
	ui.label_cFound->setStyleSheet(QString());
	ui.label_dFound->setText(QStringLiteral("Check has not run with these corners and settings."));
	ui.label_dFound->setStyleSheet(QString());
	ui.label_iResult->setText(QStringLiteral("Locate has not run."));
	ui.label_iResult->setStyleSheet(QString());
	for (auto* l : { ui.label_eFound, ui.label_fFound, ui.label_hFound }) { l->clear(); l->setStyleSheet(QString()); }
	fillWidgets();
	emit setupChanged();
}

bool GuidedSetup2Page::isDirty() const
{
	//a passed Find is saved as it happens (recordFid0Found), so it only shows here when that save failed
	return gs2::toJson(_work) != gs2::toJson(_saved);
}

void GuidedSetup2Page::confirmLeave()
{
	if (!isDirty()) return;
	QMessageBox box(this);
	box.setIcon(QMessageBox::Question);
	box.setWindowTitle(QStringLiteral("Guided Setup (new)"));
	box.setText(QStringLiteral("Recipe %1 has unsaved Guided Setup (new) changes.").arg(_recipeName));
	QPushButton* bSave = box.addButton(QStringLiteral("Save"), QMessageBox::AcceptRole);
	box.addButton(QStringLiteral("Discard"), QMessageBox::DestructiveRole);
	box.setDefaultButton(bSave);
	box.exec();
	//a Save that fails has already said why; the edits stay on the page rather than being thrown
	//away (a recipe change still replaces them - the page holds one recipe)
	if (box.clickedButton() == bSave) { save(); return; }
	discard();
}

bool GuidedSetup2Page::confirmClose()
{
	QMessageBox box(this);
	box.setIcon(QMessageBox::Question);
	box.setWindowTitle(QStringLiteral("Close"));
	box.setText(QStringLiteral("Recipe %1 has unsaved Guided Setup (new) changes.").arg(_recipeName));
	box.setInformativeText(QStringLiteral("Save them before the application closes?"));
	QPushButton* bSave = box.addButton(QStringLiteral("Save and close"), QMessageBox::AcceptRole);
	QPushButton* bDiscard = box.addButton(QStringLiteral("Discard and close"), QMessageBox::DestructiveRole);
	QPushButton* bCancel = box.addButton(QStringLiteral("Cancel"), QMessageBox::RejectRole);
	box.setDefaultButton(bCancel);
	box.setEscapeButton(bCancel);
	box.exec();
	if (box.clickedButton() == bSave) return save(); //a failed Save has said why
	if (box.clickedButton() == bDiscard) { discard(); return true; }
	return false;
}

void GuidedSetup2Page::setLayoutShown(bool shown)
{
	QSignalBlocker b(ui.toolButton_showLayout);
	ui.toolButton_showLayout->setChecked(shown);
	ui.toolButton_showLayout->setText(shown ? QStringLiteral("Show camera") : QStringLiteral("Show layout"));
}

void GuidedSetup2Page::refreshScale()
{
	refreshStatus();
}

void GuidedSetup2Page::fillWidgets()
{
	_filling = true;
	const auto& s = _work;
	ui.dspin_aThreshold->setValue(s.safety1.threshold);

	ui.dspin_bStep->setValue(s.fid0.stepMm);
	ui.spin_bStages->setValue(s.fid0.refineStages);
	ui.dspin_bBox->setValue(s.fid0.focusBoxMm);
	ui.dspin_bScore->setValue(s.fid0.minScore);

	ui.checkBox_cAddLaser->setChecked(s.z3d.addLaserOffset);
	ui.dspin_cTarget->setValue(s.z3d.targetMm);
	ui.dspin_cSpread->setValue(s.z3d.maxSpreadMm);
	ui.dspin_cDiffTol->setValue(s.z3d.diffTolMm);

	ui.dspin_dWidth->setValue(s.fidRect.widthMm);
	ui.dspin_dLength->setValue(s.fidRect.lengthMm);
	ui.dspin_dResidual->setValue(s.fidRect.residualLimitMm);
	ui.dspin_dAngle->setValue(s.fidRect.maxAngleDeg);
	ui.dspin_dSearch->setValue(s.fidRect.searchMm);

	//the circle finder settings of the kind picked in Circle
	const gs2::CircleFinder& cf = s.circle[circleKind()];
	ui.dspin_tDiaMin->setValue(cf.dia.minMm);
	ui.dspin_tDiaMax->setValue(cf.dia.maxMm);
	ui.comboBox_tPolarity->setCurrentIndex(cf.polarity);
	ui.spin_tScore->setValue(cf.minScore);
	ui.spin_tSmooth->setValue(cf.smoothness);
	ui.comboBox_tDetail->setCurrentIndex(cf.detail);

	ui.dspin_gWidth->setValue(s.unitLayout.widthMm);
	ui.dspin_gLength->setValue(s.unitLayout.lengthMm);
	ui.spin_gRows->setValue(s.unitLayout.rows);
	ui.spin_gCols->setValue(s.unitLayout.cols);
	ui.dspin_gMargin->setValue(s.unitLayout.marginMm);

	QCheckBox* on[] = { ui.checkBox_h2DOn, ui.checkBox_hBcOn, ui.checkBox_h3DOn };
	QComboBox* ref[] = { ui.comboBox_h2DRef, ui.comboBox_hBcRef, ui.comboBox_h3DRef };
	QDoubleSpinBox* dx[] = { ui.dspin_h2DDx, ui.dspin_hBcDx, ui.dspin_h3DDx };
	QDoubleSpinBox* dy[] = { ui.dspin_h2DDy, ui.dspin_hBcDy, ui.dspin_h3DDy };
	QDoubleSpinBox* dz[] = { ui.dspin_h2DZ, ui.dspin_hBcZ, ui.dspin_h3DZ };
	for (int t = 0; t < gs2::InspTypeCount; t++) {
		on[t]->setChecked(s.insp[t].enabled);
		ref[t]->setCurrentIndex(s.insp[t].ref);
		dx[t]->setValue(s.insp[t].dx);
		dy[t]->setValue(s.insp[t].dy);
		dz[t]->setValue(s.insp[t].zOffsetMm);
	}
	ui.dspin_h3DLen->setValue(s.insp[gs2::Insp3D].scanLengthMm);
	ui.comboBox_hScanDir->setCurrentIndex((s.scanAxis == QLatin1String("Y") ? 2 : 0) + (s.scanNegative ? 1 : 0));
	_filling = false;
	refreshStatus();
}

void GuidedSetup2Page::edited()
{
	if (_filling) return;
	auto& s = _work;
	const gs2::Setup setupBefore = s;
	s.safety1.threshold = shown(ui.dspin_aThreshold);

	s.fid0.stepMm = shown(ui.dspin_bStep);
	s.fid0.refineStages = ui.spin_bStages->value();
	s.fid0.focusBoxMm = shown(ui.dspin_bBox);
	s.fid0.minScore = shown(ui.dspin_bScore);

	gs2::CircleFinder& cf = s.circle[circleKind()];
	cf.dia = { shown(ui.dspin_tDiaMin), shown(ui.dspin_tDiaMax) };
	cf.polarity = std::max(0, ui.comboBox_tPolarity->currentIndex());
	cf.minScore = ui.spin_tScore->value();
	cf.smoothness = ui.spin_tSmooth->value();
	cf.detail = std::max(0, ui.comboBox_tDetail->currentIndex());

	s.z3d.addLaserOffset = ui.checkBox_cAddLaser->isChecked();
	s.z3d.targetMm = shown(ui.dspin_cTarget);
	s.z3d.maxSpreadMm = shown(ui.dspin_cSpread);
	s.z3d.diffTolMm = shown(ui.dspin_cDiffTol);

	s.fidRect.widthMm = shown(ui.dspin_dWidth);
	s.fidRect.lengthMm = shown(ui.dspin_dLength);
	s.fidRect.residualLimitMm = shown(ui.dspin_dResidual);
	s.fidRect.maxAngleDeg = shown(ui.dspin_dAngle);
	s.fidRect.searchMm = shown(ui.dspin_dSearch);

	const gs2::UnitLayout before = s.unitLayout;
	s.unitLayout.widthMm = shown(ui.dspin_gWidth);
	s.unitLayout.lengthMm = shown(ui.dspin_gLength);
	s.unitLayout.rows = ui.spin_gRows->value();
	s.unitLayout.cols = ui.spin_gCols->value();
	s.unitLayout.marginMm = shown(ui.dspin_gMargin);
	//a different grid adds or removes cells; existing units keep their names
	if (before.rows != s.unitLayout.rows || before.cols != s.unitLayout.cols) gs2::syncUnits(s);

	QCheckBox* on[] = { ui.checkBox_h2DOn, ui.checkBox_hBcOn, ui.checkBox_h3DOn };
	QComboBox* ref[] = { ui.comboBox_h2DRef, ui.comboBox_hBcRef, ui.comboBox_h3DRef };
	QDoubleSpinBox* dx[] = { ui.dspin_h2DDx, ui.dspin_hBcDx, ui.dspin_h3DDx };
	QDoubleSpinBox* dy[] = { ui.dspin_h2DDy, ui.dspin_hBcDy, ui.dspin_h3DDy };
	QDoubleSpinBox* dz[] = { ui.dspin_h2DZ, ui.dspin_hBcZ, ui.dspin_h3DZ };
	for (int t = 0; t < gs2::InspTypeCount; t++) {
		s.insp[t].enabled = on[t]->isChecked();
		s.insp[t].ref = ref[t]->currentIndex();
		s.insp[t].dx = shown(dx[t]);
		s.insp[t].dy = shown(dy[t]);
		s.insp[t].zOffsetMm = shown(dz[t]);
	}
	s.insp[gs2::Insp3D].scanLengthMm = shown(ui.dspin_h3DLen);
	const int dir = ui.comboBox_hScanDir->currentIndex();
	s.scanAxis = dir >= 2 ? QStringLiteral("Y") : QStringLiteral("X");
	s.scanNegative = (dir % 2) == 1;

	//a new Ref keeps the point where it is: the offset becomes the one from the new reference
	bool refChanged = false;
	const double uw = s.unitLayout.widthMm, ul = s.unitLayout.lengthMm;
	for (int t = 0; t < gs2::InspTypeCount; t++) {
		auto& in = s.insp[t];
		if (in.ref == setupBefore.insp[t].ref || uw <= 0.0 || ul <= 0.0) continue;
		const gs2geo::Pt was = gs2geo::unitRefPoint(setupBefore.insp[t].ref, uw, ul), now = gs2geo::unitRefPoint(in.ref, uw, ul);
		in.dx = std::clamp(std::round((in.dx + was.x - now.x) * 1000.0) / 1000.0, -500.0, 500.0);   //the offset boxes hold +/-500
		in.dy = std::clamp(std::round((in.dy + was.y - now.y) * 1000.0) / 1000.0, -500.0, 500.0);
		refChanged = true;
	}

	//a Find or Check proves the settings it ran with, not these
	keepResults(setupBefore, s);

	if (refChanged) fillWidgets(); //shows the new offsets (and refreshes)
	else refreshStatus();
	emit setupChanged();
}

QString GuidedSetup2Page::pxText(double minMm, double maxMm) const
{
	const double umPerPx = ScaleManager::instance().um_per_px();
	if (!(umPerPx > 0.0) || maxMm <= 0.0) return QString();
	return QStringLiteral("= %1 .. %2 px").arg(std::lround(minMm * 1000.0 / umPerPx)).arg(std::lround(maxMm * 1000.0 / umPerPx));
}

void GuidedSetup2Page::refreshStatus()
{
	const auto& s = _work;

	QString fileState;
	switch (_loadResult) {
	case gs2::LoadResult::Missing:    fileState = QStringLiteral("not set up yet"); break;
	case gs2::LoadResult::Loaded:     fileState = QStringLiteral("guidedSetup2.json"); break;
	case gs2::LoadResult::Unreadable: fileState = QStringLiteral("guidedSetup2.json UNREADABLE"); break;
	}
	ui.label_recipe->setText(_recipeName.isEmpty() ? QStringLiteral("No recipe open")
		: QStringLiteral("Recipe: %1   (%2)").arg(_recipeName, fileState));
	ui.label_banner->setVisible(_loadResult == gs2::LoadResult::Unreadable);
	if (_loadResult == gs2::LoadResult::Unreadable)
		ui.label_banner->setText(QStringLiteral("%1. Save keeps that file aside (.unreadable-<time>) and writes a new one.").arg(_loadWhy));

	//read-only values; the list is only rebuilt when it changed, so a selection survives typing elsewhere
	QStringList listed;
	for (int i = 0; i < ui.listWidget_aImages->count(); i++) listed << ui.listWidget_aImages->item(i)->text();
	if (listed != s.safety1.emptyImages) {
		ui.listWidget_aImages->clear();
		ui.listWidget_aImages->addItems(s.safety1.emptyImages);
	}
	ui.toolButton_aDelete->setEnabled(!s.safety1.emptyImages.isEmpty() && ui.toolButton_aSnapEmpty->isEnabled());

	if (s.fid0.found.ok)
		showFid0Result(QStringLiteral("Find passed %1: fid0 at X %2, Y %3, 2D Z %4, pattern score %5")
			.arg(s.fid0.found.when, mm3(s.fid0.found.x), mm3(s.fid0.found.y), mm3(s.fid0.found.z)).arg(s.fid0.found.score, 0, 'f', 2), true);
	else if (ui.label_bFound->styleSheet() == kDoneColour)
		showFid0Result(QStringLiteral("Find has not run with these settings."), false);

	ui.label_bXY->setText(xyText(s.fid0.xy));
	ui.label_bZ->setText(QStringLiteral("Start %1   Stop %2")
		.arg(s.fid0.startSet ? mm3(s.fid0.startZ) : QStringLiteral("not set"), s.fid0.stopSet ? mm3(s.fid0.stopZ) : QStringLiteral("not set")));
	ui.label_bFocusZ->setText(s.fid0.found.ok ? QStringLiteral("%1   (Find %2)").arg(mm3(s.fid0.found.z), s.fid0.found.when)
		: QStringLiteral("- (Find has not passed with these settings)"));
	ui.label_bPattern->setText(s.fid0.patternFile.isEmpty() ? QStringLiteral("not learnt") : s.fid0.patternFile);
	const double umPerPx = ScaleManager::instance().um_per_px();
	ui.label_bBoxPx->setText(umPerPx > 0.0 ? QStringLiteral("= %1 px").arg(std::lround(s.fid0.focusBoxMm * 1000.0 / umPerPx)) : QString());
	ui.label_bCircle->setText(circleSummary(gs2::CircleFid0));

	ui.label_cSpot->setText(xyText(s.z3d.offset, QStringLiteral("d")));
	ui.label_c3DZ->setText(s.z3d.found.ok ? QStringLiteral("%1   (reading %2 mm at the 2D Z %3, Find %4)")
		.arg(mm3(s.z3d.found.z), mm3(s.z3d.found.reading), mm3(s.z3d.found.z2D), s.z3d.found.when)
		: QStringLiteral("- (Find has not passed with these settings)"));
	ui.label_cDiff->setText(s.z3d.diffSet ? QStringLiteral("%1 mm").arg(mm3(s.z3d.diffMm)) : QStringLiteral("not set"));
	if (s.z3d.found.ok)
		showZ3DResult(QStringLiteral("Find passed %1: reading %2 mm at the 2D Z %3, so 3D Centre Z %4 (2D - 3D = %5 mm)")
			.arg(s.z3d.found.when, mm3(s.z3d.found.reading), mm3(s.z3d.found.z2D), mm3(s.z3d.found.z), mm3(s.z3d.found.z2D - s.z3d.found.z)), true);
	else if (ui.label_cFound->styleSheet() == kDoneColour)
		showZ3DResult(QStringLiteral("Find has not run with these settings."), false);

	static const char* const cornerName[4] = { "TL", "TR", "BR", "BL" };
	QStringList corners;
	for (int i = 0; i < 4; i++)
		corners << QStringLiteral("%1   %2").arg(QLatin1String(cornerName[i]), xyText(s.fidRect.corner[i], QStringLiteral("d")));
	ui.label_dCorners->setText(corners.join(QLatin1Char('\n')));
	ui.label_dCircle->setText(circleSummary(gs2::CircleCorner));
	//a corner's Jog only once it is taught, and like Teach never while the machine is busy
	QToolButton* jog[4] = { ui.toolButton_dJogTL, ui.toolButton_dJogTR, ui.toolButton_dJogBR, ui.toolButton_dJogBL };
	for (int i = 0; i < 4; i++)
		jog[i]->setEnabled(s.fidRect.corner[i].set && ui.toolButton_dTeachTL->isEnabled());

	const gs2geo::RectFit fit = gs2::taughtFit(s);
	if (!fit.pose.valid) {
		ui.label_dFit->setText(QStringLiteral("Teach fid0 and all four corners to see the fit."));
	}
	else {
		QString t = QStringLiteral("Measured %1 x %2 mm").arg(mm3(fit.widthMeasured), mm3(fit.lengthMeasured));
		if (s.fidRect.widthMm > 0.0 && s.fidRect.lengthMm > 0.0)
			t += QStringLiteral(" (drawing %1 x %2)").arg(mm3(s.fidRect.widthMm), mm3(s.fidRect.lengthMm));
		t += QStringLiteral("\nAngle %1 deg").arg(QString::number(fit.pose.angle * 180.0 / kPi, 'f', 3));
		if (s.fidRect.widthMm > 0.0 && s.fidRect.lengthMm > 0.0)
			t += QStringLiteral("\nResiduals TL %1, TR %2, BR %3, BL %4 mm (limit %5)")
				.arg(mm3(fit.residual[0]), mm3(fit.residual[1]), mm3(fit.residual[2]), mm3(fit.residual[3]), mm3(s.fidRect.residualLimitMm));
		ui.label_dFit->setText(t);
	}
	if (s.fidRect.found.ok) {
		const auto& r = s.fidRect.found;
		showRectResult(QStringLiteral("Check passed %1: centre X %2, Y %3, angle %4 deg. Residuals TL %5, TR %6, BR %7, BL %8 mm. Measured %9 x %10 mm")
			.arg(r.when, mm3(r.cx), mm3(r.cy), QString::number(r.angleDeg, 'f', 3), mm3(r.residual[0]), mm3(r.residual[1]), mm3(r.residual[2]), mm3(r.residual[3]))
			.arg(mm3(r.widthMeasured), mm3(r.lengthMeasured)), true);
	}
	else if (ui.label_dFound->styleSheet() == kDoneColour)
		showRectResult(QStringLiteral("Check has not run with these corners and settings."), false);

	{
		const auto& z = s.slotSize;
		QString t = z.set ? QStringLiteral("%1 x %2 mm").arg(mm3(z.widthMm), mm3(z.lengthMm)) : QStringLiteral("not measured");
		if (z.tl.set || z.br.set)
			t += QStringLiteral("\nTL circle %1\nBR circle %2  (trolley frame)").arg(xyText(z.tl), xyText(z.br));
		ui.label_eSize->setText(t);
	}
	ui.label_eCircle->setText(circleSummary(gs2::CircleSlot));

	//rebuilt only when it changed, keeping the selection (by slot id) that Delete, Rename and Interpolate use
	{
		QStringList lines, ids;
		for (const auto& sl : s.slotList) {
			lines << QStringLiteral("%1     centre x %2   y %3   (trolley frame)").arg(sl.name, mm3(sl.cx), mm3(sl.cy));
			ids << sl.id;
		}
		QStringList shownLines, shownIds;
		for (int i = 0; i < ui.listWidget_fSlots->count(); i++) {
			shownLines << ui.listWidget_fSlots->item(i)->text();
			shownIds << ui.listWidget_fSlots->item(i)->data(Qt::UserRole).toString();
		}
		//the ids too: a slot deleted and added again can read the same with a new id
		if (s.slotList.isEmpty() ? shownLines != QStringList{ QStringLiteral("No slots yet.") } : (shownLines != lines || shownIds != ids)) {
			const QStringList keep = selectedSlotIds();
			QSignalBlocker b(ui.listWidget_fSlots);   //the view drops slots that are gone itself (setSetup)
			ui.listWidget_fSlots->clear();
			for (int i = 0; i < lines.size(); i++) {
				auto* item = new QListWidgetItem(lines[i], ui.listWidget_fSlots);
				item->setData(Qt::UserRole, ids[i]);
				item->setSelected(keep.contains(ids[i]));
			}
			if (s.slotList.isEmpty()) ui.listWidget_fSlots->addItem(QStringLiteral("No slots yet."));
		}
	}

	const auto& u = s.unitLayout;
	if (!s.slotSize.set) {
		ui.label_gFit->setText(QStringLiteral("Measure the slot size (E) to check the fit."));
	}
	else {
		const double ex = (u.cols - 1) * (u.widthMm + 2 * u.marginMm) + u.widthMm;
		const double ey = (u.rows - 1) * (u.lengthMm + 2 * u.marginMm) + u.lengthMm;
		const bool fits = gs2geo::unitsFit(u.rows, u.cols, u.widthMm, u.lengthMm, u.marginMm, s.slotSize.widthMm, s.slotSize.lengthMm);
		ui.label_gFit->setText(QStringLiteral("%1: the units cover %2 x %3 mm of a %4 x %5 mm slot")
			.arg(fits ? QStringLiteral("Fits") : QStringLiteral("Does not fit"), mm3(ex), mm3(ey), mm3(s.slotSize.widthMm), mm3(s.slotSize.lengthMm)));
	}
	if (s.units.isEmpty()) {
		ui.label_gNames->setText(QStringLiteral("No units yet (they come with the slots)."));
	}
	else {
		QStringList names;
		int temporary = 0;
		for (const auto& unit : s.units) { names << unit.name; if (gs2::isTemporaryName(unit.name)) temporary++; }
		std::sort(names.begin(), names.end(), gs2::naturalLess);
		QString t = QStringLiteral("%1 units, %2 .. %3").arg(names.size()).arg(names.first(), names.last());
		if (temporary) t += QStringLiteral("; %1 need a name (Name all by position)").arg(temporary);
		ui.label_gNames->setText(t);
	}

	//section ticks
	QLabel* state[gs2::SectionCount] = { ui.label_aState, ui.label_bState, ui.label_cState, ui.label_dState,
										 ui.label_eState, ui.label_fState, ui.label_gState, ui.label_hState };
	for (int i = 0; i < gs2::SectionCount; i++) {
		QString why;
		const bool done = gs2::sectionDone(s, (gs2::Section)i, &why);
		state[i]->setText(done ? QString(QChar(0x2713)) + QStringLiteral(" Done") : QStringLiteral("Not done: %1").arg(why));
		state[i]->setStyleSheet(done ? kDoneColour : kTodoColour);
	}
	ui.label_iState->setText(QStringLiteral("Locate; the visits come in build step 4b"));
	ui.label_iState->setStyleSheet(QStringLiteral("color:#A8B0BF;"));

	static const char* const usedBy[gs2::CircleKindCount] = { "used by section B: Teach XY, Learn pattern, Find",
		"used by section D: the four corners", "used by section E: the slot circles" };
	const gs2::CircleFinder& cf = s.circle[circleKind()];
	ui.label_tRange->setText(QLatin1String(usedBy[circleKind()]));
	ui.label_tDiaPx->setText(cf.dia.valid() ? pxText(cf.dia.minMm, cf.dia.maxMm) : QStringLiteral("type the diameter range"));

	const bool dirty = isDirty();
	ui.toolButton_save->setEnabled(dirty && !_recipeDir.isEmpty());
	ui.toolButton_discard->setEnabled(dirty);
	if (dirty) {
		ui.label_saveState->setText(QStringLiteral("Unsaved changes"));
		ui.label_saveState->setStyleSheet(kTodoColour);
	}
	else {
		ui.label_saveState->setText(_loadResult == gs2::LoadResult::Loaded
			? (gs2::isComplete(_saved) ? QStringLiteral("Saved - complete") : QStringLiteral("Saved - not complete yet"))
			: QStringLiteral("Nothing saved yet"));
		ui.label_saveState->setStyleSheet(QStringLiteral("color:#A8B0BF;"));
	}
}

bool GuidedSetup2Page::save()
{
	if (_recipeDir.isEmpty()) {
		QMessageBox::warning(this, QStringLiteral("Guided Setup (new)"), QStringLiteral("Not saved: no recipe is open."));
		return false;
	}
	QString why;
	if (!gs2::save(_recipeDir, _work, why)) {
		ct::logger::error("[GuidedSetup2] Save failed: %s", why.toStdString().c_str());
		QMessageBox::warning(this, QStringLiteral("Guided Setup (new)"), QStringLiteral("Not saved: %1").arg(why));
		return false;
	}
	_saved = _work;
	_loadResult = gs2::LoadResult::Loaded;
	_loadWhy.clear();
	if (_mayClean) gs2::removeUnusedFiles(_recipeDir, _saved, _saved); //images and patterns replaced or deleted before this Save
	ct::logger::info("[GuidedSetup2] Saved %s (%s, %d slots, %d units)", gs2::filePath(_recipeDir).toStdString().c_str(),
		gs2::isComplete(_saved) ? "complete" : "not complete", _saved.slotList.size(), _saved.units.size());
	refreshStatus();
	emit setupChanged();
	return true;
}

void GuidedSetup2Page::discard()
{
	if (!_recipeDir.isEmpty() && _mayClean) gs2::removeUnusedFiles(_recipeDir, _saved, _saved); //what the discarded edits wrote
	_work = _saved;
	fillWidgets();
	emit setupChanged();
}
