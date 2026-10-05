// =============================================================================
//  VisionApp_PMF.cpp
//  Pogo Mapping File page: load a PMF PDF and show its DUT-IF pogo block
//  positions (columns A..P) in a table.
//
//  The PDF itself is read by Scripts/pmfExtract.py, which prints JSON. Qt has no
//  PDF reader, and the file is a real PDF - compressed content streams with the
//  table geometry recovered from text positions - so parsing it natively would
//  mean a new third-party dependency. The app already ships a Python environment
//  for PaddleOCR, so this costs a process launch instead.
// =============================================================================

#include "VisionApp.h"
#include "AuditLog.h"

#include <QFileDialog>
#include <QFileInfo>
#include <QHeaderView>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QProcess>
#include <QTableWidgetItem>

//same environment the OCR server runs in - one Python to install and keep working
static const QString kPmfPython = QStringLiteral("C:/Advanced/Scripts/VirtualEnv/Scripts/python.exe");
static const QString kPmfScript = QStringLiteral("C:/Advanced/Scripts/pmfExtract.py");

void VisionApp::initPmfPage()
{
	ui.tableWidget_pmf->setEditTriggers(QAbstractItemView::NoEditTriggers);
	ui.tableWidget_pmf->setSelectionBehavior(QAbstractItemView::SelectRows);
	ui.tableWidget_pmf->horizontalHeader()->setStretchLastSection(true);

	connect(ui.toolButton_pmfLoad, &QToolButton::clicked, this, [=]() {
		const QString path = QFileDialog::getOpenFileName(this,
			tr("Open Pogo Mapping File"), _pmfLastDir, tr("PMF documents (*.pdf);;All files (*)"));
		if (path.isEmpty()) return;

		_pmfLastDir = QFileInfo(path).absolutePath();
		loadPmfFile(path);
	});

	connect(ui.toolButton_pmfClear, &QToolButton::clicked, this, [=]() {
		ui.tableWidget_pmf->setRowCount(0);
		ui.tableWidget_pmf->setColumnCount(0);
		ui.tableWidget_pmfXY->setRowCount(0);
		ui.tableWidget_pmfXY->setColumnCount(0);
		ui.label_pmfFile->setText(tr("No file loaded"));
		ui.label_pmfStatus->clear();
		ui.label_pmfXYCap->setText(tr("XY coordinates - select a row above"));
	});

	ui.tableWidget_pmfXY->setEditTriggers(QAbstractItemView::NoEditTriggers);

	//the grid belongs to ONE board, so it follows the selection rather than trying to merge
	//every slot into a single map - two slots can both own an X1Y1 and neither is wrong
	connect(ui.tableWidget_pmf, &QTableWidget::currentCellChanged, this,
		[=](int row, int, int, int) { showPmfXyForRow(row); });
}

void VisionApp::loadPmfFile(const QString& path)
{
	ui.label_pmfFile->setText(QFileInfo(path).fileName());
	ui.label_pmfStatus->setStyleSheet(QStringLiteral("color:#F0F0F0;"));
	ui.label_pmfStatus->setText(tr("Reading..."));
	QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents, 50);

	if (!QFileInfo::exists(kPmfPython) || !QFileInfo::exists(kPmfScript)) {
		const QString why = tr("The PMF reader is not installed on this machine "
			"(expected %1 and %2).").arg(kPmfPython, kPmfScript);
		ui.label_pmfStatus->setStyleSheet(QStringLiteral("color:#E53935;"));
		ui.label_pmfStatus->setText(why);
		ct::logger::error("[PMF] %s", why.toStdString().c_str());
		return;
	}

	/*
	* Run to completion rather than streaming: a PMF is a handful of pages and the
	* table is useless until every page has been read, so there is nothing to show
	* in the meantime. Bounded so a wedged interpreter cannot hang the UI thread.
	*/
	QProcess proc;
	proc.start(kPmfPython, { kPmfScript, path });
	if (!proc.waitForStarted(5000)) {
		ui.label_pmfStatus->setStyleSheet(QStringLiteral("color:#E53935;"));
		ui.label_pmfStatus->setText(tr("Could not start the PMF reader."));
		ct::logger::error("[PMF] Failed to start: %s", proc.errorString().toStdString().c_str());
		return;
	}
	if (!proc.waitForFinished(30000)) {
		proc.kill();
		ui.label_pmfStatus->setStyleSheet(QStringLiteral("color:#E53935;"));
		ui.label_pmfStatus->setText(tr("The PMF reader timed out."));
		ct::logger::error("[PMF] Reader timed out on %s", path.toStdString().c_str());
		return;
	}

	const QByteArray out = proc.readAllStandardOutput();
	const QString err = QString::fromUtf8(proc.readAllStandardError()).trimmed();

	if (proc.exitCode() != 0 || out.isEmpty()) {
		//the script writes the reason to stderr and exits non-zero, so show that rather
		//than a generic failure - "pdfplumber is not installed" is actionable, "failed" is not
		const QString why = err.isEmpty() ? tr("The PMF could not be read.") : err;
		ui.label_pmfStatus->setStyleSheet(QStringLiteral("color:#E53935;"));
		ui.label_pmfStatus->setText(why);
		ct::logger::error("[PMF] %s", why.toStdString().c_str());
		return;
	}

	QJsonParseError parseErr;
	const QJsonDocument doc = QJsonDocument::fromJson(out, &parseErr);
	if (doc.isNull() || !doc.isObject()) {
		ui.label_pmfStatus->setStyleSheet(QStringLiteral("color:#E53935;"));
		ui.label_pmfStatus->setText(tr("The PMF reader returned something unreadable."));
		ct::logger::error("[PMF] Bad JSON: %s", parseErr.errorString().toStdString().c_str());
		return;
	}

	const QJsonObject root = doc.object();
	const QJsonArray cols = root.value(QStringLiteral("columns")).toArray();
	const QJsonArray rows = root.value(QStringLiteral("rows")).toArray();

	auto* t = ui.tableWidget_pmf;
	t->clear();
	t->setColumnCount(cols.size());
	t->setRowCount(rows.size());

	QStringList headers;
	for (const auto& c : cols) headers << c.toString();
	t->setHorizontalHeaderLabels(headers);

	for (int r = 0; r < rows.size(); r++) {
		const QJsonArray cells = rows[r].toArray();
		for (int c = 0; c < cols.size(); c++) {
			const QString text = (c < cells.size()) ? cells[c].toString() : QString();
			auto* item = new QTableWidgetItem(text);

			//the pogo columns are the point of the page: dim the context columns ahead of
			//them so A..P reads as the data and Slot/Board/Cable as the label for it
			if (c < kPmfContextColumns) item->setForeground(QBrush(QColor(0xA8, 0xB0, 0xBF)));
			else item->setForeground(QBrush(QColor(0xF0, 0xF0, 0xF0)));

			t->setItem(r, c, item);
		}
	}

	t->resizeColumnsToContents();

	const int pages = root.value(QStringLiteral("pages")).toInt();
	ui.label_pmfStatus->setStyleSheet(QStringLiteral("color:#4CAF50;"));
	ui.label_pmfStatus->setText(tr("%1 row(s) from %2 page(s)").arg(rows.size()).arg(pages));

	ct::logger::info("[PMF] Loaded %d row(s) from %s", rows.size(), path.toStdString().c_str());
	AuditLog::instance().log(QStringLiteral("PMF_LOAD"),
		QStringLiteral("%1 (%2 rows)").arg(QFileInfo(path).fileName()).arg(rows.size()));
}

/*
* The XY grid for one board.
*
* The PMF stores a pogo block as sixteen lettered columns, each holding the positions for
* that column top to bottom: column A reads "226, 626, 258, 658". The letter is the X index
* and the place in the list is the Y index, so A is X1 and its first entry is Y1 - giving
* X1Y1 = 226, X2Y1 = 230 (column B's first entry) and X1Y2 = 626 (column A's second).
*
* Only the columns that actually carry values are shown. A board using four of the sixteen
* would otherwise sit behind twelve empty columns, with the real data squeezed off to the left.
*/
void VisionApp::showPmfXyForRow(int row)
{
	auto* src = ui.tableWidget_pmf;
	auto* grid = ui.tableWidget_pmfXY;

	grid->clear();
	grid->setRowCount(0);
	grid->setColumnCount(0);

	if (row < 0 || row >= src->rowCount()) {
		ui.label_pmfXYCap->setText(tr("XY coordinates - select a row above"));
		return;
	}

	//split every pogo column of this row into its Y values
	QVector<QStringList> byX;
	int maxY = 0;
	int lastUsedX = -1;

	for (int c = kPmfContextColumns; c < src->columnCount(); c++) {
		auto* item = src->item(row, c);
		const QString text = item ? item->text().trimmed() : QString();

		QStringList values;
		for (const QString& part : text.split(QLatin1Char(','), QString::SkipEmptyParts)) {
			const QString v = part.trimmed();
			if (!v.isEmpty()) values << v;
		}

		if (!values.isEmpty()) lastUsedX = byX.size();
		maxY = std::max(maxY, values.size());
		byX.append(values);
	}

	if (lastUsedX < 0 || maxY == 0) {
		ui.label_pmfXYCap->setText(tr("XY coordinates - this row has no pogo positions"));
		return;
	}

	const int cols = lastUsedX + 1;
	grid->setColumnCount(cols);
	grid->setRowCount(maxY);

	QStringList xHeaders, yHeaders;
	for (int x = 0; x < cols; x++) xHeaders << QStringLiteral("X%1").arg(x + 1);
	for (int y = 0; y < maxY; y++) yHeaders << QStringLiteral("Y%1").arg(y + 1);
	grid->setHorizontalHeaderLabels(xHeaders);
	grid->setVerticalHeaderLabels(yHeaders);

	for (int x = 0; x < cols; x++) {
		for (int y = 0; y < maxY; y++) {
			const QString v = (y < byX[x].size()) ? byX[x][y] : QString();
			auto* cell = new QTableWidgetItem(v);
			cell->setTextAlignment(Qt::AlignCenter);
			cell->setForeground(QBrush(v.isEmpty() ? QColor(0x6A, 0x72, 0x80) : QColor(0xF0, 0xF0, 0xF0)));
			grid->setItem(y, x, cell);
		}
	}

	grid->resizeColumnsToContents();

	//name the board the grid belongs to - "X1Y1" means nothing without it
	auto* slot = src->item(row, 0);
	auto* board = src->item(row, 1);
	ui.label_pmfXYCap->setText(tr("XY coordinates - slot %1, %2")
		.arg(slot ? slot->text() : QString("?"), board ? board->text() : QString("?")));
}
