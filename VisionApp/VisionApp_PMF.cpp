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

	/*
	* The grid spans the whole file, so selecting a board cannot rebuild it - instead it
	* scrolls to that board's band of Y rows, which is the slow thing to find by hand.
	*/
	connect(ui.tableWidget_pmf, &QTableWidget::currentCellChanged, this,
		[=](int row, int, int, int) {
			if (row < 0 || row >= _pmfRowFirstY.size()) return;
			const int y = _pmfRowFirstY[row];
			if (y > 0) ui.tableWidget_pmfXY->scrollToItem(
				ui.tableWidget_pmfXY->item(y - 1, 0), QAbstractItemView::PositionAtCenter);
		});
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

	//the XY grid is derived from what just landed in the table above
	buildPmfXyTable();

	const int pages = root.value(QStringLiteral("pages")).toInt();
	ui.label_pmfStatus->setStyleSheet(QStringLiteral("color:#4CAF50;"));
	ui.label_pmfStatus->setText(tr("%1 row(s) from %2 page(s)").arg(rows.size()).arg(pages));

	ct::logger::info("[PMF] Loaded %d row(s) from %s", rows.size(), path.toStdString().c_str());
	AuditLog::instance().log(QStringLiteral("PMF_LOAD"),
		QStringLiteral("%1 (%2 rows)").arg(QFileInfo(path).fileName()).arg(rows.size()));
}

/*
* The XY grid for the WHOLE file.
*
* Each board stores sixteen lettered columns, every one holding that column's positions top
* to bottom: slot 101 column A reads "226, 626, 258, 658".
*
*   X is the LETTER within its own board - A is X1, B is X2 - and restarts on every board.
*   Y runs CONTINUOUSLY down the file: each board occupies a band of Y rows as deep as its
*   deepest column, and the next board starts below it.
*
* Slot 101 is four deep, so it fills Y1..Y4 and the next board (slot 103) begins at Y5 -
* giving X1Y1 = 226, X2Y1 = 230, X1Y2 = 626 and X1Y5 = 212.
*
* Bands are sized by the board's DEEPEST column, not by each column separately, so a board
* whose columns differ in length still occupies one rectangular band and the boards below it
* keep their Y numbers. Sizing per column would make Y mean something different in every
* column, which is no longer a coordinate.
*/
void VisionApp::buildPmfXyTable()
{
	auto* src = ui.tableWidget_pmf;
	auto* grid = ui.tableWidget_pmfXY;

	grid->clear();
	grid->setRowCount(0);
	grid->setColumnCount(0);
	_pmfRowFirstY.clear();
	_pmfRowFirstY.resize(src->rowCount());

	struct Cell { QString value; QString slot; QChar letter; };
	QHash<int, Cell> cells;   //key: y * 100 + x, both 1-based
	int maxX = 0;
	int yOffset = 0;

	for (int r = 0; r < src->rowCount(); r++) {
		_pmfRowFirstY[r] = -1;

		auto* slotItem = src->item(r, 0);
		const QString slot = slotItem ? slotItem->text() : QString();

		//split this board's columns first - the band depth is the deepest of them
		QVector<QStringList> byLetter;
		int depth = 0;
		for (int c = kPmfContextColumns; c < src->columnCount(); c++) {
			auto* item = src->item(r, c);
			const QString text = item ? item->text().trimmed() : QString();

			QStringList values;
			for (const QString& part : text.split(QLatin1Char(','), QString::SkipEmptyParts)) {
				const QString v = part.trimmed();
				if (!v.isEmpty()) values << v;
			}
			depth = std::max(depth, values.size());
			byLetter.append(values);
		}

		if (depth == 0) continue; //no positions on this board, so it takes no Y rows

		_pmfRowFirstY[r] = yOffset + 1;

		for (int x = 0; x < byLetter.size(); x++) {
			if (byLetter[x].isEmpty()) continue;
			maxX = std::max(maxX, x + 1);
			for (int j = 0; j < byLetter[x].size(); j++) {
				Cell cell;
				cell.value = byLetter[x][j];
				cell.slot = slot;
				cell.letter = QChar('A' + x);
				cells.insert((yOffset + j + 1) * 100 + (x + 1), cell);
			}
		}

		yOffset += depth;
	}

	if (maxX == 0 || yOffset == 0) {
		ui.label_pmfXYCap->setText(tr("XY coordinates - nothing to show"));
		return;
	}

	grid->setColumnCount(maxX);
	grid->setRowCount(yOffset);

	QStringList xHeaders, yHeaders;
	for (int x = 0; x < maxX; x++) xHeaders << QStringLiteral("X%1").arg(x + 1);
	for (int y = 0; y < yOffset; y++) yHeaders << QStringLiteral("Y%1").arg(y + 1);
	grid->setHorizontalHeaderLabels(xHeaders);
	grid->setVerticalHeaderLabels(yHeaders);

	for (int y = 1; y <= yOffset; y++) {
		for (int x = 1; x <= maxX; x++) {
			const Cell cell = cells.value(y * 100 + x);
			auto* item = new QTableWidgetItem(cell.value);
			item->setTextAlignment(Qt::AlignCenter);
			item->setForeground(QBrush(cell.value.isEmpty()
				? QColor(0x6A, 0x72, 0x80) : QColor(0xF0, 0xF0, 0xF0)));
			if (!cell.value.isEmpty()) {
				//a coordinate on its own cannot say which board it belongs to
				item->setToolTip(tr("X%1Y%2 - slot %3, column %4")
					.arg(x).arg(y).arg(cell.slot, QString(cell.letter)));
			}
			grid->setItem(y - 1, x - 1, item);
		}
	}

	grid->resizeColumnsToContents();
	ui.label_pmfXYCap->setText(tr("XY coordinates - %1 x %2 across %3 board(s)")
		.arg(maxX).arg(yOffset).arg(src->rowCount()));
}
