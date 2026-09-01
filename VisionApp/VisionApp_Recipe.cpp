#include "VisionApp.h"
#include "AlgoManager.h"
#include "ProductionPage.h"
#include "UnitConfigTab.h"
#include "DatasetPage.h"
#include "ScaleManager.h"
#include "TemplateLibraryTab.h"
#include "ImageViewerTab.h"
#include "3DOpticsTab.h"
#include <QPdfWriter>
#include <QTextDocument>
#include <QPageSize>
#include <QPageLayout>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <algorithm>
#include <limits>
#include "AuditLog.h"

void VisionApp::newRecipe()
{
	bool ok1;
	bool ok2;
	bool flag = true;
	QString recipeName;

	recipeName = QInputDialog::getText(this, tr("New Recipe"), tr("Recipe:"), QLineEdit::Normal, "", &ok1, Qt::CoverWindow);

	if (ok1 && ok2 && !recipeName.isEmpty())
	{
		QDir dir(Common::Directory::LocalPath + QStringLiteral("recipe/"));

		if (dir.exists(recipeName) == true)
		{
			QMessageBox::StandardButton reply = QMessageBox::question(this, "New recipe", "Overwrite Recipe?", QMessageBox::Yes | QMessageBox::No);

			if (reply == QMessageBox::Yes)
			{
				AuditLog::instance().log(QStringLiteral("RECIPE_OVERWRITE_NEW"), recipeName);
				dir.cd(Common::Directory::LocalPath + QStringLiteral("recipe/%1").arg(recipeName));
				dir.removeRecursively();
			}
			else
			{
				flag = false;
			}
		}

		if (flag == true)
		{
			createRecipe(recipeName);

			openRecipe(recipeName);

			showStatus(QStringLiteral("Recipe created"));
		}
	}
	updateSetupCheckList();
}

void VisionApp::createRecipe(const QString& recipeName)
{
	_objectModel.clear();
	_recipeModel.clear();
	clearVisionObject();
	clearView();
	clearPath();
	clearLineScans();
	clearAllDrawings();

	_recipeOptics.clear();
	addDefaultOptic();

	QStringList header;
	header.append(QStringLiteral(""));
	_recipeModel.setHorizontalHeaderLabels(header);

	QStandardItem* parentItem = _recipeModel.invisibleRootItem();

	Common::Directory::CurrentRecipe = recipeName;
	Common::Directory::setupRecipeDir(Common::Directory::CurrentRecipe);
	Common::Directory::CurrentImageSetPath = Common::Directory::getRecipeSetupImagePath();
	_pRecipeItem = new QStandardItem(recipeName);
	_pRecipeItem->setEditable(false);

	parentItem->appendRow(_pRecipeItem);

	QDir dir;
	dir.cd(Common::Directory::LocalPath + QStringLiteral("recipe/"));
	dir.mkdir(recipeName);

	dir.cd(Common::Directory::LocalPath + QStringLiteral("recipe/%1/").arg(recipeName));
	dir.mkdir(QStringLiteral("algorithm"));
	dir.mkdir(QStringLiteral("image"));
	dir.mkdir(QStringLiteral("cache"));
	dir.mkdir(QStringLiteral("archive"));

	displayWorld();

	setDefaultFiducialInfos();
	loadRecipeOptics();
	loadPlane();
	loadBarcode();
	AlgoManager::instance().loadRecipeConfig();
	refreshAlgoSetupPage();

	//loadUnitConfigInfos
	_unitConfigTab->loadUnitConfig(_views);
	//loadProductionPageCamInfo
	_productionPage->setCamInfo(_views, _recipeOptics);

	//loadViewsIntoImageViewerTab
	_imageViewerTab->setViewInfoList(_views);

	//load 3D optics
	_optics3DTab->loadOptics3DToUi();
	_optics3DTab->refreshCurrentOptics3DFromJson();

	//loadDatasetPage
	_datasetPage->updateDatasetView(_unitConfigTab->getUnifConfigInfos(), _unitConfigTab, _views, _recipeOptics);
	_templateLibraryTab->loadTemplateList();

	_systemObj.insert(QStringLiteral("Recent_Open_Recipe"), Common::Directory::CurrentRecipe);
	updateSystemInfo(_systemObj);
	updateSetupCheckList();

	auto initJsonPath = QStringLiteral("%1recipe/%2/init.json").arg(Common::Directory::LocalPath).arg(Common::Directory::CurrentRecipe);
	saveJson(initJsonPath, QJsonDocument(ScaleManager::instance().json_object()));

	saveRecipe();

	showMsg("Call-To-Action: Reminder to recalibrate lighting when creating a new recipe to ensure the images obtained are not affected by the degration of the LED.");
	toPage(UIPage::LIGHTING);
}

bool VisionApp::openRecipe(const QString& recentOpenRecipe, bool autoLoad)
{
	TimeLogger timer;

	QDir dir;
	QString val;
	QFile file;
	bool ok = true;
	QJsonObject root;
	QJsonDocument doc;
	QString jsonPath;
	QString imagePath;
	QString recipeName;
	if (autoLoad) {
		const QString suffix = "_BBA";
		QString name = Common::Directory::CurrentRecipe.trimmed();
		if (name.endsWith(suffix)) name.chop(suffix.size());  // remove "_BBA"
		else name += suffix;                                  // add "_BBA"
		recipeName = name;
		qDebug() << "autoLoad toggled recipeName:" << recipeName;
	}
	else
	{
		if (recentOpenRecipe.isEmpty())
		{
			dir.setPath(QStringLiteral("%1recipe/").arg(Common::Directory::LocalPath));
			QStringList recipes = dir.entryList(QDir::NoDotAndDotDot | QDir::AllEntries);

			if (recipes.count() > 0)
			{
				recipeName = QInputDialog::getItem(this, tr("Open Recipe"), tr("Recipe:"), recipes, 0, false, &ok, Qt::CoverWindow);
				qDebug() << "recipeName:" << recipeName;
				if (!ok)
				{
					qDebug() << "Load Recipe Cancelled";
					return false;
				}
			}
			else
			{
				showMsg(QStringLiteral("No recipe to open"));
				return false;
			}
		}
		else
		{
			recipeName = recentOpenRecipe;
		}
	}
	unloadODModels();
	_objectModel.clear();
	_recipeModel.clear();
	clearVisionObject();
	clearView();
	clearLineScans();
	clearPath();
	clearAllDrawings();

	Common::Directory::CurrentRecipe = recipeName;
	Common::Directory::setupRecipeDir(Common::Directory::CurrentRecipe);
	Common::Directory::CurrentImageSetPath = Common::Directory::getRecipeSetupImagePath();

	loadRefPositionPortabilityInfo();
	loadCurPositionPortabilityInfo();

	bool isTop = true;
	checkRecipeFacing(Common::Directory::CurrentRecipe, isTop);

	auto facing = isTop ?
		"<b> + Top recipe Detected!"
		: "<b> + Bot recipe Detected!";
	ui.label_recipeFacing->setText(facing);

	_systemObj.insert(QStringLiteral("Recent_Open_Recipe"), Common::Directory::CurrentRecipe);
	updateSystemInfo(_systemObj);

	int maxCount = 9;
	progressBarSetup("Open Recipe", maxCount);
	timer.log_duration("{openRecipe} Update system info", true);

	//loadTemplateLibrary
	_templateLibraryTab->loadTemplateList();
	incrementProgressBar();
	timer.log_duration("{openRecipe} Load template", true);

	_enableSingleViewRecipe = false;
	if (_enableSingleViewRecipe)
	{
		loadPlaneView(true);
	}
	else
	{
		loadWorldEnv();
		loadPlane();
	}
	initStitchingMethod();
	loadRecipeConfig();
	timer.log_duration("{openRecipe} Load recipe config", true);

	loadRecipeOptics();
	incrementProgressBar();
	qDebug() << "done recipe";
	loadVisionObject();
	incrementProgressBar();
	timer.log_duration("{openRecipe} Load vision object", true);

	loadView();
	incrementProgressBar();
	timer.log_duration("{openRecipe} Load view", true);

	//loadUnitConfigInfos
	_unitConfigTab->loadUnitConfig(_views);

	_optics3DTab->loadOptics3DToUi();
	_optics3DTab->refreshCurrentOptics3DFromJson();

	//reset CamWidthHeight and bufferQueue
	if (_enableSingleViewRecipe)
	{
		clearBufferQueue();
	}

	loadIslandInfo();
	loadLineScans();
	//updateTreeViewExplorer
	updateTreeViewExplorer(Common::Directory::CurrentRecipe, _views, _visionObject);
	incrementProgressBar();
	timer.log_duration("{openRecipe} Update tree view", true);

	if (g_viewMode == (int)ViewMode::PLANE) createCamAlpha();

	if (g_viewMode == (int)ViewMode::PLANE) loadPathInfo();
	incrementProgressBar();

	updateAllChannels();
	incrementProgressBar();

	//loadProductionPageCamInfo
	//_productionPage->setCamInfo(_views, _recipeOptics);

	//loadViewsIntoImageViewerTab
	_imageViewerTab->setViewInfoList(_views);

	//loadDatasetPage
	//_datasetPage->updateDatasetView(_unitConfigTab->getUnifConfigInfos(), _unitConfigTab, _views, _recipeOptics);

	if (loadFiducial()) {
		showFiducial(0);
		displayFiducialImage(0);
		toggleFidROISetupMode(false);
	}

	loadBarcode();
	AlgoManager::instance().loadRecipeConfig();
	refreshAlgoSetupPage();

	initRecipeSetupZStack();
	loadRecipeSetupZStack();



	_imageManager.attach(&_views, &_recipeOptics);
	_imageManager.attach(&_lineScans, &_recipeOptics3D);
	_imageManager.reset();

	_jobThread.attach(&_views, &_recipeOptics);
	_jobThread.attach(&_lineScans, &_recipeOptics3D);

	incrementProgressBar();

	toggleWorldView();

	//refreshDragBox_Z
	refreshDragBoxSequence();
	timer.log_duration("{openRecipe} Loading remaining files", true);

	//loadODModels
	addObjectDetectionModels();
	loadODModelListJson();

	recipeSanitaryCheck();

	progressBarRelease();

	updateSetupCheckList();

	udpateRecipeVersion(Common::Directory::getRecipeCurrentPath());
	//loadStitchingMethod();
	ui.label_recipeName->setText(QString("Recipe: %1").arg(Common::Directory::CurrentRecipe));

	OpticsControl::instance().toggleAllChannels(false);

	clearCacheFolder();
	_optics3DTab->refreshCurrentOptics3DFromJson();

	showPath(false);
	showView(false);
	showLineScans(false);
	showVisionObject(true);

	return true;
}

void VisionApp::openAutoCalibrationRecipe()
{
	const QString autoCalibrationRecipeName = QStringLiteral("3D Jig cal");
	const QDir recipeRoot(Common::Directory::RecipePath());
	const QStringList recipes = recipeRoot.entryList(QDir::Dirs | QDir::NoDotAndDotDot);

	QString matchedRecipeName;
	for (const QString& recipe : recipes)
	{
		if (recipe.compare(autoCalibrationRecipeName, Qt::CaseInsensitive) == 0)
		{
			matchedRecipeName = recipe;
			break;
		}
	}

	if (matchedRecipeName.isEmpty())
	{
		showMsg(QStringLiteral("Recipe \"%1\" not found.").arg(autoCalibrationRecipeName));
		return;
	}

	openRecipe(matchedRecipeName);

	// Ask the operator whether to run live (online) or on the loaded image set (offline).
	const QString mode = promptComboBox(QStringList() << "Offline" << "Online",
		QStringLiteral("Auto Calibration"), QStringLiteral("Run inspection mode:"));
	if (mode.isEmpty()) return; // cancelled

	// The inspection is asynchronous; the report is generated from inspectionDone() once
	// the run completes (guarded by _autoCalPending, covers both online and offline).
	_autoCalPending = true;

	if (mode == QStringLiteral("Online"))
	{
		startProduction();
	}
	else
	{
		_datasetIndexIds.clear();
		resetLoopFlags();
		_inspQueue = {};
		_inspQueue.push(Common::Directory::CurrentImageSetPath);
		runQueuedInsp();
	}
}

void VisionApp::generateAutoCalReport()
{
	const QString calDataDir = QStringLiteral("C:/Advanced/Data/3DCalReadings");
	QDir dataDir(calDataDir);

	if (!dataDir.exists())
	{
		showMsg(QStringLiteral("Auto cal report: folder not found - %1").arg(calDataDir));
		return;
	}

	const QStringList csvFiles = dataDir.entryList(QStringList() << "GridIntensity*.csv", QDir::Files, QDir::Name);
	const QStringList jsonFiles = dataDir.entryList(QStringList() << "HeightMeasurement*.json", QDir::Files, QDir::Name);

	if (csvFiles.isEmpty() && jsonFiles.isEmpty())
	{
		showMsg(QStringLiteral("Auto cal report: no calibration readings found in %1").arg(calDataDir));
		return;
	}

	QString html;
	QTextStream out(&html);
	out << QStringLiteral("<h2>3D Auto Calibration Report</h2>");
	out << QStringLiteral("<p><b>Recipe:</b> %1<br><b>Generated:</b> %2</p>")
		.arg(Common::Directory::CurrentRecipe.toHtmlEscaped())
		.arg(QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss"));

	// 3D calibration block reference (fixed): block letter -> expected height (mm).
	struct BlockRef { const char* block; double expected_mm; };
	static const BlockRef kBlockRefs[] = {
		{ "A", 5.0 }, { "B", 4.0 }, { "C", 3.0 }, { "D", 2.0 },
		{ "E", 1.0 }, { "F", 0.75 }, { "G", 0.5 }, { "H", 0.25 }
	};
	const int kBlockCount = static_cast<int>(sizeof(kBlockRefs) / sizeof(kBlockRefs[0]));
	const double acceptanceRange_mm = 0.05;

	for (const QString& jsonFile : jsonFiles)
	{
		QFile file(dataDir.absoluteFilePath(jsonFile));
		if (!file.open(QIODevice::ReadOnly)) continue;

		QJsonParseError parseError;
		QJsonDocument doc = QJsonDocument::fromJson(file.readAll(), &parseError);
		file.close();
		if (parseError.error != QJsonParseError::NoError || !doc.isObject()) continue;

		QJsonArray readings = doc.object().value(QStringLiteral("3DCalReadings")).toArray();
		if (readings.isEmpty()) continue;

		out << QStringLiteral("<h3>3D Calibration (%1)</h3>").arg(jsonFile.toHtmlEscaped());
		out << QStringLiteral("<p>Acceptable Range: &plusmn;%1 mm</p>").arg(acceptanceRange_mm);
		out << QStringLiteral("<table border='1' cellspacing='0' cellpadding='4' width='100%'>");
		out << QStringLiteral(
			"<tr>"
			"<th rowspan='2'>Block</th>"
			"<th rowspan='2'>Expected Height (mm)</th>"
			"<th colspan='2'>3DF (mm)</th>"
			"<th rowspan='2'>Acceptance Error (&plusmn;%1mm)</th>"
			"</tr>"
			"<tr><th>Height</th><th>Error to Expected Height</th></tr>").arg(acceptanceRange_mm);

		int idx = 0;
		for (const auto& v : readings)
		{
			QJsonObject o = v.toObject();
			double measured_mm = o.value(QStringLiteral("averageHeight_um")).toDouble() / 1000.0;

			// Resolve which block this reading is: prefer a name that matches A..H,
			// otherwise fall back to row order.
			QString name = o.value(QStringLiteral("name")).toString().trimmed();
			QString block;
			double expected_mm = 0.0;
			bool haveExpected = false;
			for (int b = 0; b < kBlockCount; ++b)
			{
				if (name.compare(QLatin1String(kBlockRefs[b].block), Qt::CaseInsensitive) == 0)
				{
					block = QString(kBlockRefs[b].block);
					expected_mm = kBlockRefs[b].expected_mm;
					haveExpected = true;
					break;
				}
			}
			if (!haveExpected && idx < kBlockCount)
			{
				block = QString(kBlockRefs[idx].block);
				expected_mm = kBlockRefs[idx].expected_mm;
				haveExpected = true;
			}
			if (block.isEmpty()) block = name.isEmpty() ? QStringLiteral("-") : name;

			QString expectedStr = haveExpected ? QString::number(expected_mm) : QStringLiteral("-");
			QString heightStr = QString::number(measured_mm, 'f', 3);
			QString errorStr = QStringLiteral("-");
			QString acceptanceStr = QStringLiteral("-");
			QString acceptanceColor = QStringLiteral("#000");
			if (haveExpected)
			{
				double error_mm = measured_mm - expected_mm;
				bool pass = qAbs(error_mm) <= acceptanceRange_mm;
				errorStr = QString::number(error_mm, 'f', 3);
				acceptanceStr = pass ? QStringLiteral("Pass") : QStringLiteral("Fail");
				acceptanceColor = pass ? QStringLiteral("#0a0") : QStringLiteral("#c00");
			}

			out << QStringLiteral("<tr><td align='center'>%1</td><td align='center'>%2</td>"
				"<td align='center'>%3</td><td align='center'>%4</td>"
				"<td align='center' style='color:%5'><b>%6</b></td></tr>")
				.arg(block.toHtmlEscaped())
				.arg(expectedStr)
				.arg(heightStr)
				.arg(errorStr)
				.arg(acceptanceColor)
				.arg(acceptanceStr);
			++idx;
		}
		out << QStringLiteral("</table>");
	}

	for (const QString& csvFile : csvFiles)
	{
		QFile file(dataDir.absoluteFilePath(csvFile));
		if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) continue;

		QVector<QVector<double>> grid;
		QTextStream in(&file);
		while (!in.atEnd())
		{
			QString line = in.readLine().trimmed();
			if (line.isEmpty()) continue;

			QVector<double> row;
			for (const QString& cell : line.split(','))
				row.push_back(cell.toDouble());
			grid.push_back(row);
		}
		file.close();
		if (grid.isEmpty()) continue;

		out << QStringLiteral("<h3>Grid Intensity (%1)</h3>").arg(csvFile.toHtmlEscaped());
		out << QStringLiteral("<table class='grid' border='1' cellspacing='0' cellpadding='2' width='100%'>");

		double sum = 0.0, minV = std::numeric_limits<double>::max(), maxV = std::numeric_limits<double>::lowest();
		int count = 0;
		for (const auto& row : grid)
		{
			out << QStringLiteral("<tr>");
			for (double v : row)
			{
				out << QStringLiteral("<td align='center'>%1</td>").arg(v, 0, 'f', 2);
				sum += v;
				count++;
				minV = std::min(minV, v);
				maxV = std::max(maxV, v);
			}
			out << QStringLiteral("</tr>");
		}
		out << QStringLiteral("</table>");

		double avg = count ? sum / count : 0.0;
		// Full precision to match the calibration report (e.g. Min: 44.9488, Range: 7.66125).
		// QString::arg(double) defaults to 'g' with 6 significant digits.
		out << QStringLiteral("<p>Min: %1 &nbsp; Max: %2 &nbsp; Range: %3 &nbsp; Average: %4</p>")
			.arg(minV).arg(maxV).arg(maxV - minV).arg(avg);
	}

	const QString reportDir = calDataDir + "/Report";
	util::createFolder(reportDir);
	const QString reportPath = QStringLiteral("%1/AutoCalReport_%2.pdf")
		.arg(reportDir)
		.arg(QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss"));

	QPdfWriter pdfWriter(reportPath);
	pdfWriter.setPageSize(QPageSize(QPageSize::A4));
	pdfWriter.setPageMargins(QMarginsF(12, 12, 12, 12), QPageLayout::Millimeter);
	// QPdfWriter defaults to 1200 DPI while QTextDocument lays out fonts at 96 DPI.
	// That mismatch makes the page ~8800px wide and shrinks the text to a tiny strip.
	// Render at 96 DPI so the document's point sizes map to the page correctly.
	pdfWriter.setResolution(96);

	QTextDocument doc;
	doc.setDefaultStyleSheet(QStringLiteral(
		"body { font-family: Arial, sans-serif; font-size: 10pt; color: #000; }"
		"h2 { font-size: 15pt; margin: 0 0 6px 0; }"
		"h3 { font-size: 11pt; margin: 12px 0 4px 0; }"
		"p { font-size: 10pt; margin: 4px 0; }"
		"table { border-collapse: collapse; }"
		"th, td { border: 1px solid #888; padding: 3px 5px; font-size: 9pt; }"
		"th { background-color: #e8e8f0; }"
		"table.grid td { padding: 1px 2px; font-size: 7pt; text-align: center; }"
	));
	doc.setHtml(html);
	doc.setPageSize(pdfWriter.pageLayout().paintRectPixels(pdfWriter.resolution()).size());
	doc.print(&pdfWriter);

	ct::logger::info("Auto cal report generated: %s", reportPath.toStdString().c_str());
	showMsg(QStringLiteral("Auto cal report generated:\n%1").arg(reportPath));
}

