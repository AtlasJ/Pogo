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
		ui.label_pmfFile->setText(tr("No file loaded"));
		ui.label_pmfStatus->clear();
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

	const int pages = root.value(QStringLiteral("pages")).toInt();
	ui.label_pmfStatus->setStyleSheet(QStringLiteral("color:#4CAF50;"));
	ui.label_pmfStatus->setText(tr("%1 row(s) from %2 page(s)").arg(rows.size()).arg(pages));

	ct::logger::info("[PMF] Loaded %d row(s) from %s", rows.size(), path.toStdString().c_str());
	AuditLog::instance().log(QStringLiteral("PMF_LOAD"),
		QStringLiteral("%1 (%2 rows)").arg(QFileInfo(path).fileName()).arg(rows.size()));
}
