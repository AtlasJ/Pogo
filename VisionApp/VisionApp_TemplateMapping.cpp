// =============================================================================
//  VisionApp_TemplateMapping.cpp
//  Template Mapping Setup, on the PMF page: named X x Y grids of whole numbers,
//  kept app-wide in template_mapping.json (see TemplateMapping.h).
//
//  A mapping changes only through Edit -> Save; Discard puts back what was saved.
//  Save refuses a mapping with an empty cell or a value used twice.
//  While editing, the mapping list and Add / Rename / Remove are locked, so what
//  is on screen is always the selected mapping.
//
//  The production page's Template Mapping row lights green while the name typed
//  there matches a mapping here, and follows every add, rename, remove and save -
//  as does the Live Mapping, which takes that mapping as its layout.
// =============================================================================

#include "VisionApp.h"
#include "AuditLog.h"
#include "CellFillDelegate.h"
#include "nvsutil/nvs_qt.h"

#include <QInputDialog>
#include <QIntValidator>
#include <QLineEdit>
#include <QMap>
#include <QMessageBox>
#include <QPoint>
#include <QSignalBlocker>
#include <QTableWidgetItem>

namespace {

const QColor kProblemFill(0x8B, 0x2E, 0x2E);   //a cell a refused Save complained about

//cell editor: whole numbers in range and nothing else; clearing the text empties the cell
class MappingCellDelegate : public CellFillDelegate
{
public:
	using CellFillDelegate::CellFillDelegate;

	QWidget* createEditor(QWidget* parent, const QStyleOptionViewItem& option, const QModelIndex& index) const override
	{
		QWidget* editor = CellFillDelegate::createEditor(parent, option, index);
		if (auto* line = qobject_cast<QLineEdit*>(editor))
			line->setValidator(new QIntValidator(0, TemplateMappingStore::kMaxValue, line));
		return editor;
	}
};

QString cellName(const QPoint& cell)
{
	return QStringLiteral("X%1 Y%2").arg(cell.x() + 1).arg(cell.y() + 1);
}

//"X1 Y1, X2 Y1, ..." - capped so a big grid cannot make the message taller than the screen
QString cellList(const QVector<QPoint>& cells)
{
	const int kShown = 10;
	QStringList names;
	for (int i = 0; i < cells.size() && i < kShown; i++) names << cellName(cells[i]);
	QString text = names.join(QStringLiteral(", "));
	if (cells.size() > kShown) text += QStringLiteral(" and %1 more").arg(cells.size() - kShown);
	return text;
}

} // namespace

void VisionApp::initTemplateMappingSetup()
{
	ui.tableWidget_tmValues->setItemDelegate(new MappingCellDelegate(ui.tableWidget_tmValues));
	for (QSpinBox* s : { ui.spinBox_tmXCount, ui.spinBox_tmYCount }) {
		s->setRange(TemplateMappingStore::kMinCount, TemplateMappingStore::kMaxCount);
		s->setKeyboardTracking(false); //typing "12" resizes once, not to 1 and then 12
	}

	QString message;
	if (!_templateMappings.load(&message)) ct::logger::error("[TemplateMapping] %s", message.toStdString().c_str());
	else if (!message.isEmpty()) ct::logger::warn("[TemplateMapping] %s", message.toStdString().c_str());
	ui.label_tmStatus->setText(message);

	auto selected = [=]() { return _templateMappings.indexOfId(ui.comboBox_tmMapping->currentData().toString()); };

	connect(ui.comboBox_tmMapping, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
		[=](int) { showTemplateMapping(); });

	connect(ui.toolButton_tmAdd, &QToolButton::clicked, this, [=]() {
		const QString name = askTemplateMappingName(tr("Add Mapping"), QString(), QString());
		if (name.isEmpty()) return;

		const TemplateMappingStore before = _templateMappings;
		const int added = _templateMappings.add(name);
		const QString id = _templateMappings.mappings()[added].id;
		if (!saveTemplateMappings(before)) return;

		AuditLog::instance().log(QStringLiteral("TEMPLATE_MAPPING_ADD"), QStringLiteral("%1 (%2)").arg(name, id));
		listTemplateMappings(id);
	});

	connect(ui.toolButton_tmRename, &QToolButton::clicked, this, [=]() {
		const int i = selected();
		if (i < 0) return;
		const QString id = _templateMappings.mappings()[i].id;
		const QString old = _templateMappings.mappings()[i].name;

		const QString name = askTemplateMappingName(tr("Rename Mapping"), old, id);
		if (name.isEmpty() || name == old) return;

		const TemplateMappingStore before = _templateMappings;
		_templateMappings.rename(i, name);
		if (!saveTemplateMappings(before)) return;

		AuditLog::instance().log(QStringLiteral("TEMPLATE_MAPPING_RENAME"), QStringLiteral("%1 -> %2 (%3)").arg(old, name, id));
		listTemplateMappings(id);
	});

	connect(ui.toolButton_tmRemove, &QToolButton::clicked, this, [=]() {
		const int i = selected();
		if (i < 0) return;
		const QString id = _templateMappings.mappings()[i].id;
		const QString name = _templateMappings.mappings()[i].name;

		if (QMessageBox::question(this, tr("Remove Mapping"),
			tr("Remove the mapping \"%1\" and all its values?").arg(name)) != QMessageBox::Yes) return;

		const TemplateMappingStore before = _templateMappings;
		_templateMappings.remove(i);
		if (!saveTemplateMappings(before)) return;

		AuditLog::instance().log(QStringLiteral("TEMPLATE_MAPPING_REMOVE"), QStringLiteral("%1 (%2)").arg(name, id));
		//select the mapping that moved into its place, or the one before it when it was the last
		const auto& left = _templateMappings.mappings();
		listTemplateMappings(left.isEmpty() ? QString() : left[std::min(i, left.size() - 1)].id);
	});

	connect(ui.toolButton_tmEdit, &QToolButton::clicked, this, [=]() { setTemplateMappingEditing(true); });
	connect(ui.toolButton_tmDiscard, &QToolButton::clicked, this, [=]() { showTemplateMapping(); });

	connect(ui.toolButton_tmSave, &QToolButton::clicked, this, [=]() {
		const int i = selected();
		if (i < 0) return;
		keepTemplateMappingEdits();

		/*
		* Every cell must hold a number, and no number may appear twice - a mapping with a gap or
		* two cells pointing at the same position is refused. All problems are reported at once,
		* and the offending cells stay red until the next Save, Discard or count change.
		* Only what is inside the new X x Y counts; anything beyond it is dropped here.
		*/
		const int xCount = ui.spinBox_tmXCount->value();
		const int yCount = ui.spinBox_tmYCount->value();
		QVector<QVector<std::optional<int>>> values(yCount, QVector<std::optional<int>>(xCount));
		QVector<QPoint> empty, invalid;
		QMap<int, QVector<QPoint>> cellsOf;   //value -> every cell holding it
		for (int y = 0; y < yCount; y++) {
			for (int x = 0; x < xCount; x++) {
				const QString text = (y < _tmScratch.size() && x < _tmScratch[y].size()) ? _tmScratch[y][x] : QString();
				if (text.isEmpty()) {
					empty << QPoint(x, y);
					continue;
				}
				bool ok = false;
				const int v = text.toInt(&ok);
				if (!ok || v < 0 || v > TemplateMappingStore::kMaxValue) {
					invalid << QPoint(x, y);
					continue;
				}
				values[y][x] = v;
				cellsOf[v] << QPoint(x, y);
			}
		}

		QStringList problems;
		QVector<QPoint> marked = empty + invalid;
		if (!empty.isEmpty())
			problems << tr("Empty cells (%1): %2").arg(empty.size()).arg(cellList(empty));
		if (!invalid.isEmpty())
			problems << tr("Not a whole number from 0 to %1: %2").arg(TemplateMappingStore::kMaxValue).arg(cellList(invalid));
		QStringList duplicates;
		for (auto it = cellsOf.cbegin(); it != cellsOf.cend(); ++it) {
			if (it.value().size() < 2) continue;
			duplicates << tr("  %1 is in %2").arg(TemplateMappingStore::valueText(it.key()), cellList(it.value()));
			marked += it.value();
		}
		if (!duplicates.isEmpty())
			problems << tr("Duplicate values:\n%1").arg(duplicates.join(QLatin1Char('\n')));

		auto* t = ui.tableWidget_tmValues;
		for (int y = 0; y < t->rowCount(); y++)
			for (int x = 0; x < t->columnCount(); x++)
				if (QTableWidgetItem* item = t->item(y, x)) item->setData(CellFillDelegate::kFillRole, QVariant());
		for (const QPoint& p : marked)
			if (QTableWidgetItem* item = t->item(p.y(), p.x())) item->setData(CellFillDelegate::kFillRole, kProblemFill);
		if (!marked.isEmpty()) t->scrollToItem(t->item(marked.first().y(), marked.first().x()));

		if (!problems.isEmpty()) {
			QMessageBox::warning(this, tr("Save Not Successful"),
				tr("The mapping was not saved:\n\n%1").arg(problems.join(QStringLiteral("\n\n"))));
			return; //stays in Edit, so the cells can be fixed
		}

		const TemplateMappingStore before = _templateMappings;
		_templateMappings.setGrid(i, xCount, yCount, values);
		if (!saveTemplateMappings(before)) return; //stays in Edit, with the edits still on screen

		const TemplateMapping& m = _templateMappings.mappings()[i];
		AuditLog::instance().log(QStringLiteral("TEMPLATE_MAPPING_SAVE"),
			QStringLiteral("%1 (%2) %3 x %4").arg(m.name, m.id).arg(m.xCount).arg(m.yCount));
		showTemplateMapping();
	});

	for (QSpinBox* s : { ui.spinBox_tmXCount, ui.spinBox_tmYCount }) {
		connect(s, QOverload<int>::of(&QSpinBox::valueChanged), this, [=](int) {
			if (!_tmEditing) return;
			keepTemplateMappingEdits();
			fillTemplateMappingTable(ui.spinBox_tmXCount->value(), ui.spinBox_tmYCount->value());
		});
	}

	connect(ui.lineEdit_prodTemplateMapping, &QLineEdit::textChanged, this, [=]() { refreshProdTemplateMapping(); });

	listTemplateMappings(QString());
}

void VisionApp::listTemplateMappings(const QString& selectId)
{
	{
		const QSignalBlocker block(ui.comboBox_tmMapping);
		ui.comboBox_tmMapping->clear();
		for (const TemplateMapping& m : _templateMappings.mappings())
			ui.comboBox_tmMapping->addItem(m.name, m.id);
		ui.comboBox_tmMapping->setCurrentIndex(std::max(0, ui.comboBox_tmMapping->findData(selectId)));
	}
	showTemplateMapping();
}

void VisionApp::showTemplateMapping()
{
	const int i = _templateMappings.indexOfId(ui.comboBox_tmMapping->currentData().toString());
	ui.stackedWidget_tmData->setCurrentWidget(i >= 0 ? ui.page_tmData : ui.page_tmEmpty);

	if (i >= 0) {
		const TemplateMapping& m = _templateMappings.mappings()[i];
		_tmScratch.clear();
		for (const auto& row : m.values) {
			QVector<QString> texts;
			for (const auto& cell : row) texts.append(cell ? TemplateMappingStore::valueText(*cell) : QString());
			_tmScratch.append(texts);
		}
		{
			const QSignalBlocker bx(ui.spinBox_tmXCount), by(ui.spinBox_tmYCount);
			ui.spinBox_tmXCount->setValue(m.xCount);
			ui.spinBox_tmYCount->setValue(m.yCount);
		}
		fillTemplateMappingTable(m.xCount, m.yCount);
	}
	setTemplateMappingEditing(false);
}

void VisionApp::setTemplateMappingEditing(bool editing)
{
	_tmEditing = editing;
	const bool any = ui.comboBox_tmMapping->count() > 0;

	//while editing, nothing may switch the mapping out from under the edits
	ui.comboBox_tmMapping->setEnabled(any && !editing);
	ui.toolButton_tmAdd->setEnabled(!editing);
	ui.toolButton_tmRename->setEnabled(any && !editing);
	ui.toolButton_tmRemove->setEnabled(any && !editing);

	ui.spinBox_tmXCount->setEnabled(editing);
	ui.spinBox_tmYCount->setEnabled(editing);
	ui.toolButton_tmEdit->setEnabled(any && !editing);
	ui.toolButton_tmSave->setEnabled(editing);
	ui.toolButton_tmDiscard->setEnabled(editing);
	ui.tableWidget_tmValues->setEditTriggers(editing
		? QAbstractItemView::DoubleClicked | QAbstractItemView::EditKeyPressed | QAbstractItemView::AnyKeyPressed
		: QAbstractItemView::NoEditTriggers);
}

void VisionApp::fillTemplateMappingTable(int xCount, int yCount)
{
	auto* t = ui.tableWidget_tmValues;
	t->setColumnCount(xCount);
	t->setRowCount(yCount);

	QStringList xHeaders, yHeaders;
	for (int x = 1; x <= xCount; x++) xHeaders << QStringLiteral("X%1").arg(x);
	for (int y = 1; y <= yCount; y++) yHeaders << QStringLiteral("Y%1").arg(y);
	t->setHorizontalHeaderLabels(xHeaders);
	t->setVerticalHeaderLabels(yHeaders);

	for (int y = 0; y < yCount; y++) {
		for (int x = 0; x < xCount; x++) {
			const QString text = (y < _tmScratch.size() && x < _tmScratch[y].size()) ? _tmScratch[y][x] : QString();
			auto* item = new QTableWidgetItem(text);
			item->setTextAlignment(Qt::AlignCenter);
			t->setItem(y, x, item);
		}
	}
}

/*
* Copy the table into _tmScratch. The scratch only ever grows during an edit, so shrinking the
* table and growing it back brings the old values back; Save takes just the X x Y it ends with.
*/
void VisionApp::keepTemplateMappingEdits()
{
	auto* t = ui.tableWidget_tmValues;
	t->setCurrentIndex(QModelIndex()); //a cell still being typed in commits when the current cell moves

	if (_tmScratch.size() < t->rowCount()) _tmScratch.resize(t->rowCount());
	for (int y = 0; y < t->rowCount(); y++) {
		if (_tmScratch[y].size() < t->columnCount()) _tmScratch[y].resize(t->columnCount());
		for (int x = 0; x < t->columnCount(); x++) {
			const QTableWidgetItem* item = t->item(y, x);
			_tmScratch[y][x] = item ? item->text().trimmed() : QString();
		}
	}
}

QString VisionApp::askTemplateMappingName(const QString& title, const QString& current, const QString& ignoreId)
{
	//ask again until the name is usable or the user cancels, keeping what they typed
	QString name = current;
	for (;;) {
		bool ok = false;
		name = QInputDialog::getText(this, title, tr("Mapping name:"), QLineEdit::Normal, name, &ok);
		if (!ok) return QString();

		const QString problem = _templateMappings.nameProblem(name, ignoreId);
		if (problem.isEmpty()) return name.trimmed();
		QMessageBox::warning(this, title, problem);
	}
}

/*
* Write the file; when that fails, put the in-memory mappings back to `before`, so what the
* page shows is always what is on disk.
*/
bool VisionApp::saveTemplateMappings(const TemplateMappingStore& before)
{
	QString error;
	if (_templateMappings.save(&error)) {
		ui.label_tmStatus->clear();
		refreshProdTemplateMapping(); //every add, rename, remove and save passes here
		return true;
	}

	_templateMappings = before;
	ct::logger::error("[TemplateMapping] Save failed: %s", error.toStdString().c_str());
	QMessageBox::warning(this, tr("Template Mapping"),
		tr("Could not save %1:\n%2").arg(TemplateMappingStore::filePath(), error));
	return false;
}

void VisionApp::refreshProdTemplateMapping()
{
	const bool found = _templateMappings.indexOfName(ui.lineEdit_prodTemplateMapping->text()) >= 0;
	nvs::set_background_color(ui.toolButton_prodTemplateMappingReady, found ? Qt::green : Qt::red);
	ui.toolButton_prodTemplateMappingReady->setToolTip(found ? tr("Found") : tr("Not found"));
	rebuildLiveMapping();   //the mapping it names is the live mapping's layout
}
