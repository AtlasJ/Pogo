// =============================================================================
//  VisionApp_LiveMapping.cpp
//  Live Mapping on the production page: the Template Mapping named in the
//  production Template Mapping box, drawn as a grid of pogo block positions and
//  coloured by each position's state.
//
//    - the template mapping is the layout: which position sits in which cell
//    - the PMF says which positions this testhead has (_pmfUnits). A position on
//      the template that the PMF does not list is Not exist, grey
//    - a PMF position with no cell on the template should never happen, and is
//      reported beside the title, as are positions the PMF lists twice
//
//  Position numbers are the key: getUnitInfo() says what is behind one, and
//  updateUnitState() recolours its cell - from any thread.
//
//  The PMF path and the template mapping name are remembered in
//  production_setup.json, so a restart comes back to the same Live Mapping.
// =============================================================================

#include "VisionApp.h"
#include "AuditLog.h"
#include "CellFillDelegate.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QHeaderView>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QRegularExpression>
#include <QSaveFile>
#include <QStyle>
#include <QThread>
#include <QTimer>
#include <algorithm>

namespace {

//what the production page had selected - the PMF's path and the template mapping's name -
//so a restart comes back to it. App-wide, like the template mappings themselves.
const QString kSetupFile = QStringLiteral("C:/Advanced/Data/config/production_setup.json");

struct StateLook { QColor fill; QColor text; };

StateLook stateLook(UnitState state)
{
	switch (state) {
	case UnitState::NotExist: return { QColor(0x3A, 0x3D, 0x44), QColor(0x8A, 0x93, 0xA3) };
	case UnitState::Pending:  return { QColor(0x3D, 0x5A, 0x80), QColor(0xF0, 0xF0, 0xF0) };
	case UnitState::Scanning: return { QColor(0xF2, 0xB7, 0x05), QColor(0x1B, 0x1D, 0x23) };
	case UnitState::Pass:     return { QColor(0x2E, 0x7D, 0x32), QColor(0xF0, 0xF0, 0xF0) };
	case UnitState::Fail:     return { QColor(0xC6, 0x28, 0x28), QColor(0xF0, 0xF0, 0xF0) };
	}
	return {};
}

//"101, 102, ..." sorted; capped unless the full list is wanted (for a tooltip)
QString positionList(QVector<int> positions, bool full = false)
{
	std::sort(positions.begin(), positions.end());
	const int kShown = 12;
	QStringList texts;
	for (int i = 0; i < positions.size() && (full || i < kShown); i++)
		texts << TemplateMappingStore::valueText(positions[i]);
	QString text = texts.join(QStringLiteral(", "));
	if (!full && positions.size() > kShown) text += QStringLiteral(" and %1 more").arg(positions.size() - kShown);
	return text;
}

} // namespace

void VisionApp::initLiveMapping()
{
	auto* t = ui.tableWidget_liveMapping;
	t->setItemDelegate(new CellFillDelegate(t));
	t->setEditTriggers(QAbstractItemView::NoEditTriggers);
	t->setSelectionMode(QAbstractItemView::NoSelection); //a selection colour would cover the state colour
	t->setFocusPolicy(Qt::NoFocus);
	t->horizontalHeader()->setMinimumSectionSize(48);
	t->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);

	//the legend is coloured from the same table as the cells, so the two cannot drift apart
	const QVector<QPair<QLabel*, UnitState>> legend = {
		{ ui.label_liveMappingLegendNotExist, UnitState::NotExist },
		{ ui.label_liveMappingLegendPending, UnitState::Pending },
		{ ui.label_liveMappingLegendScanning, UnitState::Scanning },
		{ ui.label_liveMappingLegendPass, UnitState::Pass },
		{ ui.label_liveMappingLegendFail, UnitState::Fail },
	};
	for (const auto& entry : legend) {
		const StateLook look = stateLook(entry.second);
		entry.first->setText(unitStateName(entry.second));
		entry.first->setStyleSheet(QStringLiteral("background-color:%1; color:%2; border-radius:4px; padding:2px 8px;")
			.arg(look.fill.name(), look.text.name()));
	}

	connect(this, &VisionApp::unitStateRequested, this,
		[=](int position, int state) { applyUnitState(position, UnitState(state)); }, Qt::QueuedConnection);

	refreshProdTemplateMapping(); //the lamp, and the first build of the grid
}

void VisionApp::rebuildLiveMapping()
{
	auto* t = ui.tableWidget_liveMapping;
	_liveMappingCells.clear();

	auto note = [=](const QString& text, const QString& tip, bool warning) {
		ui.label_liveMappingNote->setText(text);
		ui.label_liveMappingNote->setToolTip(tip);
		ui.label_liveMappingNote->setStyleSheet(warning ? QStringLiteral("color:#FFB300;") : QStringLiteral("color:#8A93A3;"));
	};

	const QString name = ui.lineEdit_prodTemplateMapping->text().trimmed();
	const int mi = _templateMappings.indexOfName(name);
	if (mi < 0) {
		t->setRowCount(0);
		t->setColumnCount(0);
		const QString why = name.isEmpty() ? tr("Enter a Template Mapping to show its layout.")
			: tr("There is no template mapping called \"%1\".").arg(name);
		note(why, why, false);
		return;
	}

	const TemplateMapping& m = _templateMappings.mappings()[mi];
	t->clear();
	t->setColumnCount(m.xCount);
	t->setRowCount(m.yCount);
	QStringList xHeaders, yHeaders;
	for (int x = 1; x <= m.xCount; x++) xHeaders << QStringLiteral("X%1").arg(x);
	for (int y = 1; y <= m.yCount; y++) yHeaders << QStringLiteral("Y%1").arg(y);
	t->setHorizontalHeaderLabels(xHeaders);
	t->setVerticalHeaderLabels(yHeaders);

	for (int y = 0; y < m.yCount; y++) {
		for (int x = 0; x < m.xCount; x++) {
			auto* item = new QTableWidgetItem();
			item->setTextAlignment(Qt::AlignCenter);
			const std::optional<int>& value = m.values[y][x];
			if (value) {
				item->setText(TemplateMappingStore::valueText(*value));
				_liveMappingCells.insert(*value, QPoint(x, y));
			}
			else {
				//a blank cell can only come from a hand-edited file: no position, so nothing fitted
				item->setData(CellFillDelegate::kFillRole, stateLook(UnitState::NotExist).fill);
			}
			t->setItem(y, x, item);
		}
	}
	for (auto it = _liveMappingCells.cbegin(); it != _liveMappingCells.cend(); ++it) paintLiveMappingCell(it.key());

	//what the grid cannot show: a PMF position with no cell, or one the PMF lists twice
	QVector<int> missing;
	for (auto it = _pmfUnits.cbegin(); it != _pmfUnits.cend(); ++it)
		if (!_liveMappingCells.contains(it.key())) missing << it.key();

	QStringList warnings, tips;
	if (!missing.isEmpty()) {
		warnings << tr("%1 PMF position(s) have no cell in \"%2\": %3.").arg(missing.size()).arg(m.name, positionList(missing));
		tips << tr("PMF positions with no cell in \"%1\":\n%2").arg(m.name, positionList(missing, true));
	}
	if (!_pmfDuplicates.isEmpty()) {
		warnings << tr("The PMF lists %1 more than once.").arg(positionList(_pmfDuplicates));
		tips << tr("Positions the PMF lists more than once (the first listing is used):\n%1").arg(positionList(_pmfDuplicates, true));
	}

	if (!warnings.isEmpty()) {
		note(warnings.join(QLatin1Char(' ')), tips.join(QStringLiteral("\n\n")), true);
	}
	else if (_pmfUnits.isEmpty()) {
		const QString text = tr("No PMF loaded - every position shows as Not exist.");
		note(text, text, false);
	}
	else {
		int fitted = 0;
		for (auto it = _liveMappingCells.cbegin(); it != _liveMappingCells.cend(); ++it)
			if (_pmfUnits.contains(it.key())) fitted++;
		const QString text = tr("%1 of %2 positions are in the PMF").arg(fitted).arg(_liveMappingCells.size());
		note(text, text, false);
	}
}

void VisionApp::paintLiveMappingCell(int position)
{
	const auto cell = _liveMappingCells.constFind(position);
	if (cell == _liveMappingCells.cend()) return;
	QTableWidgetItem* item = ui.tableWidget_liveMapping->item(cell->y(), cell->x());
	if (!item) return;

	const UnitState state = unitState(position);
	const StateLook look = stateLook(state);
	item->setData(CellFillDelegate::kFillRole, look.fill);
	item->setForeground(look.text);

	const QString key = TemplateMappingStore::valueText(position);
	if (const auto info = getUnitInfo(position)) {
		item->setToolTip(tr("%1 - %2\nSlot %3, OCR %4\n%5 (%6)\nCable %7")
			.arg(key, unitStateName(state), info->slot, QString(info->ocr), info->board, info->description, info->cable));
	}
	else {
		item->setToolTip(tr("%1 - not in the PMF").arg(key));
	}
}

std::optional<PogoUnitInfo> VisionApp::getUnitInfo(int position) const
{
	const auto it = _pmfUnits.constFind(position);
	if (it == _pmfUnits.cend()) return std::nullopt;
	return *it;
}

UnitState VisionApp::unitState(int position) const
{
	if (!_pmfUnits.contains(position)) return UnitState::NotExist;
	return _unitStates.value(position, UnitState::Pending);
}

void VisionApp::updateUnitState(int position, UnitState state)
{
	//widgets belong to the GUI thread: from anywhere else, hand the change over and return
	if (QThread::currentThread() != thread()) {
		emit unitStateRequested(position, int(state));
		return;
	}
	applyUnitState(position, state);
}

void VisionApp::applyUnitState(int position, UnitState state)
{
	if (!_pmfUnits.contains(position)) {
		ct::logger::warn("[LiveMapping] Position %d is not in the PMF, so it stays Not exist", position);
		return;
	}
	if (state == UnitState::NotExist) {
		ct::logger::warn("[LiveMapping] Position %d is in the PMF, so it cannot be set to Not exist", position);
		return;
	}
	_unitStates[position] = state;
	paintLiveMappingCell(position);
}

void VisionApp::resetUnitStates()
{
	for (auto it = _unitStates.begin(); it != _unitStates.end(); ++it) it.value() = UnitState::Pending;
	for (auto it = _liveMappingCells.cbegin(); it != _liveMappingCells.cend(); ++it) paintLiveMappingCell(it.key());
}

QVector<int> VisionApp::liveMappingPositions() const
{
	QVector<int> positions;
	const int mi = _templateMappings.indexOfName(ui.lineEdit_prodTemplateMapping->text());
	if (mi < 0) return positions;

	const TemplateMapping& m = _templateMappings.mappings()[mi];
	for (int y = 0; y < m.yCount; y++)
		for (int x = 0; x < m.xCount; x++)
			if (m.values[y][x] && _pmfUnits.contains(*m.values[y][x])) positions << *m.values[y][x];
	return positions;
}

/*
* Startup: put back the template mapping name and the PMF from the last session, so the
* production page does not come up blank. Only the PDF's path is kept; the PDF is read again
* through the reader, exactly as Load would.
*/
void VisionApp::restoreProductionSetup()
{
	QFile file(kSetupFile);
	if (file.open(QIODevice::ReadOnly)) {
		const QJsonObject root = QJsonDocument::fromJson(file.readAll()).object(); //unreadable = start blank
		file.close();
		_pmfFile = root.value(QStringLiteral("pmfFile")).toString();
		ui.lineEdit_prodTemplateMapping->setText(root.value(QStringLiteral("templateMapping")).toString());
	}

	//connected after the restore above, so putting the name back does not count as typing it
	_setupSaveTimer = new QTimer(this);
	_setupSaveTimer->setSingleShot(true);
	_setupSaveTimer->setInterval(500);
	connect(_setupSaveTimer, &QTimer::timeout, this, &VisionApp::saveProductionSetup);
	connect(ui.lineEdit_prodTemplateMapping, &QLineEdit::textChanged, _setupSaveTimer, QOverload<>::of(&QTimer::start));

	if (_pmfFile.isEmpty()) return;
	_pmfLastDir = QFileInfo(_pmfFile).absolutePath();

	//the reader takes a second or two, so let the window come up first
	const QString path = _pmfFile;
	QTimer::singleShot(1000, this, [=]() {
		if (_pmfFile != path) return; //a PMF was loaded or cleared in the meantime
		if (_productionSetupLocked) return; //never under a run that is already using the PMF

		if (!QFileInfo::exists(path)) {
			//kept, not forgotten: a share that is offline now may be back next time
			const QString why = tr("The PMF used last time is no longer there: %1").arg(QDir::toNativeSeparators(path));
			ui.label_pmfFile->setText(QFileInfo(path).fileName());
			ui.lineEdit_prodPmfFile->setText(QFileInfo(path).fileName());
			showPmfStatus(PmfStatus::Failed, tr("Load failed"), why);
			ct::logger::warn("[PMF] %s", why.toStdString().c_str());
			return;
		}
		ct::logger::info("[PMF] Reloading the PMF used last time: %s", path.toStdString().c_str());
		loadPmfFile(path);
	});
}

void VisionApp::saveProductionSetup()
{
	QJsonObject root;
	root.insert(QStringLiteral("pmfFile"), _pmfFile);
	root.insert(QStringLiteral("templateMapping"), ui.lineEdit_prodTemplateMapping->text().trimmed());

	QDir().mkpath(QFileInfo(kSetupFile).absolutePath());
	QSaveFile out(kSetupFile);
	bool ok = out.open(QIODevice::WriteOnly);
	if (ok) ok = out.write(QJsonDocument(root).toJson(QJsonDocument::Indented)) >= 0 && out.commit();
	if (!ok) ct::logger::warn("[Production] Could not save %s: %s",
		kSetupFile.toStdString().c_str(), out.errorString().toStdString().c_str());
}

/*
* What stops production from starting: the same three things the production page shows - the
* PMF lamp, the Template Mapping lamp, and the "no cell" warning beside the Live Mapping title.
* A PMF position the PMF lists twice is not here: the first listing is used and the run is sound.
*/
QStringList VisionApp::liveMappingProblems() const
{
	QStringList problems;

	switch (_pmfStatus) {
	case PmfStatus::Ready:
		break;
	case PmfStatus::Empty:
		problems << tr("No PMF is loaded.");
		break;
	case PmfStatus::Busy:
		problems << tr("The PMF is still being read.");
		break;
	case PmfStatus::Failed:
		problems << tr("The PMF is not usable (%1).").arg(ui.toolButton_prodPmfReady->toolTip());
		break;
	}

	const QString name = ui.lineEdit_prodTemplateMapping->text().trimmed();
	const int mi = _templateMappings.indexOfName(name);
	if (name.isEmpty()) problems << tr("No Template Mapping is entered.");
	else if (mi < 0) problems << tr("There is no template mapping called \"%1\".").arg(name);

	if (_pmfStatus == PmfStatus::Ready && mi >= 0) {
		QVector<int> missing;
		for (auto it = _pmfUnits.cbegin(); it != _pmfUnits.cend(); ++it)
			if (!_liveMappingCells.contains(it.key())) missing << it.key();
		if (!missing.isEmpty())
			problems << tr("%1 PMF position(s) have no cell in \"%2\": %3.")
				.arg(missing.size()).arg(_templateMappings.mappings()[mi].name, positionList(missing));
	}
	return problems;
}

/*
* The production start gate for the Live Mapping: refuse, saying what is missing, unless the
* machine is in debug mode - which may run without, as it may without the production selector.
* Both production starts call it before the safety check, so a run that cannot start never moves.
*/
bool VisionApp::liveMappingReadyForProduction()
{
	const QStringList missing = liveMappingProblems();
	if (missing.isEmpty()) return true;

	if (SystemData::instance()._machineDebugMode) {
		ct::logger::warn("[Production] Debug mode - starting without a ready Live Mapping: %s",
			missing.join(QLatin1Char(' ')).toStdString().c_str());
		return true;
	}

	showMsg(tr("Production cannot start:\n\n- %1").arg(missing.join(QStringLiteral("\n- "))));
	AuditLog::instance().log(QStringLiteral("PRODUCTION_BLOCKED"),
		QStringLiteral("live mapping: %1").arg(missing.join(QLatin1Char(' '))));
	return false;
}

/*
* Where a production unit sits on the Live Mapping. A run names its units X#Y# (1-based,
* JobThread::pitchUnitID) and the template mapping's cells are headed X1.. and Y1.., so unit
* X3Y2 is the template's cell X3 Y2 and holds that cell's position. -1 when the unit is outside
* the template or its cell holds no position. This is the one place that ties the two together.
*/
int VisionApp::positionForUnit(const QString& unitID) const
{
	//a doubled backslash: C++ drops an unknown escape, and a single one left "^X(d+)Y(d+)$",
	//which matched no unit at all
	static const QRegularExpression re(QStringLiteral("^X(\\d+)Y(\\d+)$"));
	const QRegularExpressionMatch match = re.match(unitID);
	if (!match.hasMatch()) return -1;

	const int mi = _templateMappings.indexOfName(ui.lineEdit_prodTemplateMapping->text());
	if (mi < 0) return -1;
	const TemplateMapping& m = _templateMappings.mappings()[mi];
	const int x = match.captured(1).toInt() - 1;
	const int y = match.captured(2).toInt() - 1;
	if (x < 0 || y < 0 || x >= m.xCount || y >= m.yCount) return -1;
	return m.values[y][x] ? *m.values[y][x] : -1;
}

/*
* The units a production run visits, by the rule above: those whose cell holds a position the
* PMF lists. A recipe unit outside the template, or on a cell the PMF does not have, is not in
* the set, so the run never goes there. std::nullopt - visit every unit, as before - when there
* is no usable Live Mapping, which only a debug-mode run can start with.
*/
std::optional<QSet<QString>> VisionApp::fittedUnits() const
{
	const int mi = _templateMappings.indexOfName(ui.lineEdit_prodTemplateMapping->text());
	if (mi < 0 || _pmfStatus != PmfStatus::Ready) return std::nullopt;

	QSet<QString> units;
	const TemplateMapping& m = _templateMappings.mappings()[mi];
	for (int y = 0; y < m.yCount; y++)
		for (int x = 0; x < m.xCount; x++)
			if (m.values[y][x] && _pmfUnits.contains(*m.values[y][x]))
				units.insert(QStringLiteral("X%1Y%2").arg(x + 1).arg(y + 1));
	return units;
}

/*
* A production unit's progress onto its cell. Scanning only ever moves a Pending cell: in a
* "2D to 3D" run the 3D pass revisits units that may already have failed, and that must not
* turn back to Scanning.
*/
void VisionApp::showUnitOnLiveMapping(const QString& unitID, UnitState state)
{
	const int position = positionForUnit(unitID);
	if (position < 0 || !_pmfUnits.contains(position)) return;
	if (state == UnitState::Scanning && unitState(position) != UnitState::Pending) return;
	updateUnitState(position, state);
}

/*
* A production run uses the PMF, the template mapping named for it and that mapping's cells the
* whole way through - which units to visit, which cell each result colours. Changing any of them
* mid-run would put results on the wrong positions, so they are locked from the moment the run
* is queued until its last result is in (the production progress bar is released) or it is
* stopped. Choosing a different mapping to look at on the PMF page stays allowed.
*/
void VisionApp::setProductionSetupLocked(bool locked)
{
	_productionSetupLocked = locked;
	for (QToolButton* b : { ui.toolButton_pmfLoad, ui.toolButton_pmfClear, ui.toolButton_prodPmfLoad, ui.toolButton_prodPmfClear })
		b->setEnabled(!locked);

	auto* name = ui.lineEdit_prodTemplateMapping;
	name->setReadOnly(locked);
	name->style()->unpolish(name); //the [readOnly=true] stylesheet look follows only on a re-polish
	name->style()->polish(name);

	setTemplateMappingEditing(_tmEditing); //re-applies the Setup buttons with the lock taken into account
}
