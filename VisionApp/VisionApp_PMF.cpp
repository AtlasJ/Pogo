// =============================================================================
//  VisionApp_PMF.cpp
//  Pogo Mapping File page: load a PMF PDF and show its DUT-IF pogo block
//  positions (columns A..P) in a table. The production page's PMF row loads and
//  clears the same file, and shows its state as a lamp. Every position the PMF lists
//  is indexed in _pmfUnits for the Live Mapping (VisionApp_LiveMapping.cpp).
//  The Template Mapping Setup section under the table is VisionApp_TemplateMapping.cpp.
//
//  The PDF itself is read by Scripts/pmfExtract.py, which prints JSON. Qt has no
//  PDF reader, and the file is a real PDF - compressed content streams with the
//  table geometry recovered from text positions - so parsing it natively would
//  mean a new third-party dependency. The app already ships a Python environment
//  for PaddleOCR, so this costs a process launch instead.
// =============================================================================

#include "VisionApp.h"
#include "AuditLog.h"
#include "nvsutil/nvs_qt.h"

#include <QFileDialog>
#include <QFileInfo>
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

	//one PMF for the whole app: Load and Clear do the same thing from either page
	for (QToolButton* b : { ui.toolButton_pmfLoad, ui.toolButton_prodPmfLoad })
		connect(b, &QToolButton::clicked, this, &VisionApp::browsePmfFile);
	for (QToolButton* b : { ui.toolButton_pmfClear, ui.toolButton_prodPmfClear })
		connect(b, &QToolButton::clicked, this, &VisionApp::clearPmf);
	showPmfStatus(PmfStatus::Empty, tr("Not loaded"), QString());
}

void VisionApp::browsePmfFile()
{
	const QString path = QFileDialog::getOpenFileName(this,
		tr("Open Pogo Mapping File"), _pmfLastDir, tr("PMF documents (*.pdf);;All files (*)"));
	if (path.isEmpty()) return;

	_pmfLastDir = QFileInfo(path).absolutePath();
	loadPmfFile(path);
}

void VisionApp::clearPmf()
{
	ui.tableWidget_pmf->setRowCount(0);
	ui.tableWidget_pmf->setColumnCount(0);
	ui.label_pmfFile->setText(tr("No file loaded"));
	ui.lineEdit_prodPmfFile->clear();
	showPmfStatus(PmfStatus::Empty, tr("Not loaded"), QString());

	_pmfUnits.clear();
	_pmfDuplicates.clear();
	_unitStates.clear();
	rebuildLiveMapping();

	_pmfFile.clear(); //cleared on purpose, so the next start comes up without one
	saveProductionSetup();
}

/*
* One PMF state, shown on both pages: the PMF page gets the full sentence, the production page
* only a lamp, with the words in its tooltip. The lamp is green only for Ready - a file read
* with at least one board in it.
*/
void VisionApp::showPmfStatus(PmfStatus status, const QString& summary, const QString& detail)
{
	_pmfStatus = status;
	const QString color = status == PmfStatus::Ready ? QStringLiteral("#4CAF50")
		: status == PmfStatus::Failed ? QStringLiteral("#E53935") : QStringLiteral("#F0F0F0");
	ui.label_pmfStatus->setStyleSheet(QStringLiteral("color:%1;").arg(color));
	ui.label_pmfStatus->setText(detail);

	ui.toolButton_prodPmfReady->setToolTip(detail.isEmpty() || detail == summary
		? summary : QStringLiteral("%1 - %2").arg(summary, detail));
	nvs::set_background_color(ui.toolButton_prodPmfReady, status == PmfStatus::Ready ? Qt::green : Qt::red);
}

void VisionApp::loadPmfFile(const QString& path)
{
	ui.label_pmfFile->setText(QFileInfo(path).fileName());
	ui.lineEdit_prodPmfFile->setText(QFileInfo(path).fileName());
	showPmfStatus(PmfStatus::Busy, tr("Reading..."), tr("Reading..."));
	QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents, 50);

	if (!QFileInfo::exists(kPmfPython) || !QFileInfo::exists(kPmfScript)) {
		const QString why = tr("The PMF reader is not installed on this machine "
			"(expected %1 and %2).").arg(kPmfPython, kPmfScript);
		showPmfStatus(PmfStatus::Failed, tr("Load failed"), why);
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
		showPmfStatus(PmfStatus::Failed, tr("Load failed"), tr("Could not start the PMF reader."));
		ct::logger::error("[PMF] Failed to start: %s", proc.errorString().toStdString().c_str());
		return;
	}
	if (!proc.waitForFinished(30000)) {
		proc.kill();
		showPmfStatus(PmfStatus::Failed, tr("Load failed"), tr("The PMF reader timed out."));
		ct::logger::error("[PMF] Reader timed out on %s", path.toStdString().c_str());
		return;
	}

	const QByteArray out = proc.readAllStandardOutput();
	const QString err = QString::fromUtf8(proc.readAllStandardError()).trimmed();

	if (proc.exitCode() != 0 || out.isEmpty()) {
		//the script writes the reason to stderr and exits non-zero, so show that rather
		//than a generic failure - "pdfplumber is not installed" is actionable, "failed" is not
		const QString why = err.isEmpty() ? tr("The PMF could not be read.") : err;
		showPmfStatus(PmfStatus::Failed, tr("Load failed"), why);
		ct::logger::error("[PMF] %s", why.toStdString().c_str());
		return;
	}

	QJsonParseError parseErr;
	const QJsonDocument doc = QJsonDocument::fromJson(out, &parseErr);
	if (doc.isNull() || !doc.isObject()) {
		showPmfStatus(PmfStatus::Failed, tr("Load failed"), tr("The PMF reader returned something unreadable."));
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

	/*
	* Index every position by its number - the Live Mapping, and anything that needs to know
	* what is behind a position, look it up here. A cell normally holds one number, but the
	* extractor keeps a "226, 626" list as it is, so every number in it counts. A new PMF
	* starts every position at Pending.
	*/
	_pmfUnits.clear();
	_pmfDuplicates.clear();
	int ignored = 0;
	for (int r = 0; r < rows.size(); r++) {
		const QJsonArray cells = rows[r].toArray();
		PogoUnitInfo info;
		info.slot = cells.at(0).toString();
		info.board = cells.at(1).toString();
		info.description = cells.at(2).toString();
		info.cable = cells.at(3).toString();
		for (int c = kPmfContextColumns; c < cols.size() && c < cells.size(); c++) {
			const QString letter = cols[c].toString();
			info.ocr = letter.isEmpty() ? QChar() : letter.at(0);
			for (const QString& part : cells[c].toString().split(QLatin1Char(','), QString::SkipEmptyParts)) {
				bool ok = false;
				const int position = part.trimmed().toInt(&ok);
				if (!ok) {
					ignored++;
					continue;
				}
				if (_pmfUnits.contains(position)) {
					if (!_pmfDuplicates.contains(position)) _pmfDuplicates << position;
					continue;
				}
				info.position = position;
				_pmfUnits.insert(position, info);
			}
		}
	}
	if (ignored > 0) ct::logger::warn("[PMF] %d value(s) under A..P are not position numbers and were ignored", ignored);
	_unitStates.clear();
	for (auto it = _pmfUnits.cbegin(); it != _pmfUnits.cend(); ++it) _unitStates.insert(it.key(), UnitState::Pending);
	rebuildLiveMapping();

	//a PDF that reads cleanly but holds no board rows is not a usable mapping
	const int pages = root.value(QStringLiteral("pages")).toInt();
	const QString detail = tr("%1 row(s) from %2 page(s)").arg(rows.size()).arg(pages);
	if (rows.isEmpty()) {
		showPmfStatus(PmfStatus::Failed, tr("No boards"), detail);
	}
	else {
		showPmfStatus(PmfStatus::Ready, tr("Loaded"), detail);
		_pmfFile = path; //only a file that loaded is worth coming back to
		saveProductionSetup();
	}

	ct::logger::info("[PMF] Loaded %d row(s), %d position(s) from %s", rows.size(), _pmfUnits.size(), path.toStdString().c_str());
	AuditLog::instance().log(QStringLiteral("PMF_LOAD"),
		QStringLiteral("%1 (%2 rows)").arg(QFileInfo(path).fileName()).arg(rows.size()));
}
