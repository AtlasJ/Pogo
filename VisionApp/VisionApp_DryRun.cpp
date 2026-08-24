#include "VisionApp.h"
#include "MotionController.h"
#include "AuditLog.h"

//Dry Run page: a table of XYZ coordinates the user can add (from the current
//position), drag to reorder, and select to delete. Start moves through the
//sequence for the requested number of loops on the job thread.

static bool g_dryRunActive = false;

void VisionApp::initDryRunPage()
{
	auto* table = ui.tableWidget_dryRun;
	table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
	table->verticalHeader()->setVisible(true);

	qRegisterMetaType<QVector<QVector3D>>("QVector<QVector3D>");
	QObject::connect(this, &VisionApp::signalDryRun, &_jobThread, &JobThread::dryRun, Qt::QueuedConnection);

	QObject::connect(&_jobThread, &JobThread::dryRunStatus, this, [=](QString msg, bool running) {
		g_dryRunActive = running;
		ui.label_dryRunStatus->setText(msg);
	}, Qt::QueuedConnection);

	//add the current machine position as a new row
	connect(ui.pushButton_dryAddCoord, &QPushButton::clicked, this, [=]() {
		double x = 0.0, y = 0.0, z = 0.0;

		auto optional_x = MotionController::instance().get_position_mm(_motionID, 0, (int)Axis::X);
		auto optional_y = MotionController::instance().get_position_mm(_motionID, 0, (int)Axis::Y);
		auto optional_z = MotionController::instance().get_position_mm(_motionID, 0, (int)Axis::Z);

		if (optional_x.has_value()) x = optional_x.value();
		if (optional_y.has_value()) y = optional_y.value();
		if (optional_z.has_value()) z = optional_z.value();

		auto* table = ui.tableWidget_dryRun;
		int row = table->rowCount();
		table->insertRow(row);
		table->setItem(row, 0, new QTableWidgetItem(QString::number(x, 'f', 3)));
		table->setItem(row, 1, new QTableWidgetItem(QString::number(y, 'f', 3)));
		table->setItem(row, 2, new QTableWidgetItem(QString::number(z, 'f', 3)));
	});

	//delete highlighted rows
	connect(ui.pushButton_dryRemoveCoord, &QPushButton::clicked, this, [=]() {
		auto* table = ui.tableWidget_dryRun;

		QList<int> rows;
		for (const auto& range : table->selectedRanges())
			for (int r = range.topRow(); r <= range.bottomRow(); r++)
				if (!rows.contains(r)) rows.append(r);

		if (rows.isEmpty()) {
			showMsg("Select the rows to remove first.");
			return;
		}

		std::sort(rows.begin(), rows.end(), std::greater<int>());
		for (int r : rows) table->removeRow(r);
	});

	connect(ui.pushButton_dryStartRun, &QPushButton::clicked, this, [=]() {
		if (g_dryRunActive) {
			showMsg("Dry run is already running.");
			return;
		}

		auto* table = ui.tableWidget_dryRun;
		if (table->rowCount() == 0) {
			showMsg("No sequence to run. Add coordinates first.");
			return;
		}

		QVector<QVector3D> coords;
		for (int r = 0; r < table->rowCount(); r++) {
			bool okX = false, okY = false, okZ = false;
			double x = table->item(r, 0) ? table->item(r, 0)->text().toDouble(&okX) : 0.0;
			double y = table->item(r, 1) ? table->item(r, 1)->text().toDouble(&okY) : 0.0;
			double z = table->item(r, 2) ? table->item(r, 2)->text().toDouble(&okZ) : 0.0;

			if (!okX || !okY || !okZ) {
				showMsg(QString("Row %1 has an invalid coordinate.").arg(r + 1));
				return;
			}

			coords.append(QVector3D((float)x, (float)y, (float)z));
		}

		g_dryRunActive = true;
		ui.label_dryRunStatus->setText("Dry run starting...");
		AuditLog::instance().log(QStringLiteral("DRY_RUN"), QStringLiteral("%1 points x %2 loops").arg(coords.size()).arg(ui.spinBox_dryLoops->value()));
		emit signalDryRun(coords, ui.spinBox_dryLoops->value());
	});

	connect(ui.pushButton_dryStopRun, &QPushButton::clicked, this, [=]() {
		_jobThread.stopRun();
		ui.label_dryRunStatus->setText("Stopping...");
	});
}
