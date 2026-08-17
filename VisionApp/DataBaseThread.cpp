#include "DataBaseThread.h"
#include "Logger.h"
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>

DataBaseThread::DataBaseThread()
{
}

void DataBaseThread::setSetting(SQLiteDatabase& sqliteDatabase, QVector<ct::DefectResult>& defectResults,
	const ProductionInfo& productionInfo, QStringList templateIdList, bool isReadyToPackaged, const PackageInfo& packageInfo)
{
	_sqliteDatabase = sqliteDatabase;
	_defectResults = defectResults;
	_productionInfo =productionInfo;
	_templateIdList = templateIdList;
	_isReadyToPackage = isReadyToPackaged;
	_packageInfo = packageInfo;
}

void DataBaseThread::run()
{
	ct::logger::info("[QThread] Database thread started");
	
	// Run both in parallel
	QFuture<void> futureCad = QtConcurrent::run(this, &DataBaseThread::compileCadOffsetFile);
	QFuture<void> futureLaser = QtConcurrent::run(this, &DataBaseThread::compileLaserMeasFile);

	futureCad.waitForFinished();
	futureLaser.waitForFinished();

	writeDefectVoInfo(_productionInfo.timestamp, _productionInfo.productionID, _productionInfo.recipeID, _productionInfo.recipeName);
	

	QString productionDataPath = Common::Directory::getProductionResultPath()+"productionData.json";
	writeProductionDataJson(productionDataPath);

	bool databaseStatus = true;
	if (_productionExportMode == ProductionExportMode::Normal)
	{
		// mini defect packager
		runRecipeCollector();
		runDefectCollector(); // here copy all the file

		databaseStatus = insertProductionToDataBase();
		if (_isReadyToPackage)
		{
			_sqliteDatabase.insertPackageData(_packageInfo);
		}
	}
	else
	{
		runDefectCollector(); // keep Images/Results on the normal ResultViewer collector path

		const QString dbPayloadPath = QDir(recipeProductionRootPath()).filePath("production_db_insert.json");
		databaseStatus = writeProductionDbInsertJson(dbPayloadPath);
	}

	emit signalDatabaseStatus(databaseStatus);
}

bool DataBaseThread::writeProductionDataJson(QString path)
{
	ProductionInfo p = _productionInfo;
	QElapsedTimer timer;
	timer.start();
	bool flag = true;

	// Create a QJsonObject to store the production data
	QJsonObject productionData;

	// Populate the QJsonObject with production data
	productionData["timeStamp"] = p.timestamp;
	productionData["recipeID"] = p.recipeID;
	productionData["recipeName"] = p.recipeName;
	productionData["productionID"] = p.productionID;
	productionData["productionFileName"] = p.productionFileName;
	productionData["cycleTime"] = p.cycleTime;
	productionData["totalDefect"] = p.totalDefect;
	productionData["isPass"] = p.isPass;
	productionData["isError"] = p.isError;
	productionData["report"] = p.report;
	productionData["reportTimeStamp"] = p.reportTimestamp;

	productionData["machineID"] = p.machineID;
	productionData["lotNumber"] = p.lotNumber;
	productionData["packageName"] = p.packageName;
	productionData["operator"] = p._operator;
	productionData["productNumber"] = p.productNumber;
	productionData["deviceGroup"] = p.deviceGroup;
	productionData["inspectionType"] = p.inspectionType;
	productionData["stripeID"] = p.stripeID;
	productionData["totalUnit"] = p.totalUnit;

	productionData["totalRow"] = p.totalRow;
	productionData["totalCol"] = p.totalCol;
	productionData["totalIsland"] = p.totalIsland;
	productionData["rowStartingIndex"] = p.rowStartingIndex;
	productionData["colStartingIndex"] = p.colStartingIndex;

	productionData["verification"] = false;
	productionData["fiducial1"] = p.fiducialStatus.join(",");
	productionData["fiducial2"] = "-";
	productionData["productionMode"] = p.productionMode;
	productionData["defectUnits"] = p.defectUnits;
	productionData["buyOffDefectUnits"] = p.defectUnits;
	productionData["passYieldPerc"] = p.passYieldPerc;
	productionData["yield"] = p.yieldPerc;
	productionData["isSampling"] = p.isSampling;
	productionData["isMounterPass"] = p.isMounterPass;
	productionData["incomingEmapPath"] = p.incomingEmapPath;
	productionData["trueCall"] = 0;
	productionData["falseCall"] = 0;
	productionData["emapTemplate"] = p.emapTemplate;

	productionData["assemblyNumber"] = p.assemblyNumber;

	productionData["startDateTime"] = p.inspectionStartDate;
	productionData["endDateTime"] = p.inspectionEndDate;

	// Convert the QJsonObject into a QJsonDocument
	QJsonDocument jsonDoc(productionData);

	
	QFile jsonFile(path);

	// Open the file for writing
	if (!jsonFile.open(QIODevice::WriteOnly)) {
		qDebug() << "Failed to open file for writing:" << jsonFile.errorString();
		return false;
	}

	// Write the JSON document to the file
	jsonFile.write(jsonDoc.toJson());
	jsonFile.close();

	qint64 elapsedTime = timer.elapsed();
	qDebug() << "Time taken for writing JSON output (ms):" << elapsedTime;
	qDebug() << "Production data written to file:" << path;

	return flag;
}

bool DataBaseThread::writeProductionDbInsertJson(QString path)
{
	ProductionInfo p = _productionInfo;
	QDir dir = QFileInfo(path).dir();
	if (!dir.exists() && !dir.mkpath(".")) {
		qDebug() << "Failed to create production DB payload folder:" << dir.absolutePath();
		return false;
	}

	QJsonObject values;
	values.insert("TIME_STAMP", p.timestamp);
	values.insert("RECIPE_ID", p.recipeID);
	values.insert("RECIPE_NAME", p.recipeName);
	values.insert("PRODUCTION_ID", p.productionID);
	values.insert("PRODUCTION_FILE_NAME", p.productionFileName);
	values.insert("CYCLE_TIME", p.cycleTime);
	values.insert("TOTAL_DEFECT", p.totalDefect);
	values.insert("IS_PASS", p.isPass ? 1 : 0);
	values.insert("IS_ERROR", p.isError ? 1 : 0);
	values.insert("REPORT", p.report ? 1 : 0);
	values.insert("REPORT_TIME_STAMP", p.reportTimestamp);
	values.insert("MACHINE_ID", p.machineID);
	values.insert("LOT_NUMBER", p.lotNumber);
	values.insert("PACKAGE_NAME", p.packageName);
	values.insert("OPERATOR", p._operator);
	values.insert("INSPECTION_TYPE", p.inspectionType);
	values.insert("BARCODE", p.stripeID);
	values.insert("TOTAL_UNIT", p.totalUnit);
	values.insert("PACKAGE_UUID", 0);
	values.insert("RECIPE_TYPE", 0);
	values.insert("FIDUCIAL_STATUS", p.fiducialStatus.join(","));
	values.insert("PRODUCTION_MODE", p.productionMode ? 1 : 0);
	values.insert("DEFECT_UNITS", p.defectUnits);
	values.insert("BUYOFF_DEFECT_UNITS", 0);
	values.insert("PASS_YIELD_PERCENTAGE", p.passYieldPerc);
	values.insert("YIELD", p.yieldPerc);
	values.insert("IS_VERIFIED", false);
	values.insert("FALSE_CALL", 0);
	values.insert("TRUE_CALL", 0);

	const QString productionPath = QDir(Common::Directory::ProductionPath()).filePath(p.productionID);
	const QString imagesPath = Common::Directory::getProductionImageSetPath();
	const QString resultsPath = Common::Directory::getProductionResultPath();
	const QString defectCollectorProductionPath = QDir(_defectCollectorPath).filePath(p.productionID);

	QJsonObject paths;
	paths.insert("recipePayloadPath", path);
	paths.insert("productionPath", productionPath);
	paths.insert("imagesPath", imagesPath);
	paths.insert("resultsPath", resultsPath);
	paths.insert("defectCollectorPath", _defectCollectorPath);
	paths.insert("defectCollectorProductionPath", defectCollectorProductionPath);
	paths.insert("defectCollectorImagesPath", QDir(defectCollectorProductionPath).filePath("Images"));
	paths.insert("defectCollectorResultsPath", QDir(defectCollectorProductionPath).filePath("Results"));

	QJsonObject root;
	root.insert("mode", productionExportModeName());
	root.insert("table", "PRODUCTION_DATA");
	root.insert("paths", paths);
	root.insert("values", values);

	QFile file(path);
	if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
		qDebug() << "Failed to write production DB payload json:" << path;
		return false;
	}

	file.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
	file.close();
	return true;
}

bool DataBaseThread::insertProductionToDataBase()
{
	qDebug() << "InsertProductionToDataBase";
	// arrange defect into Vision object based
	
	if (!_productionInfo.productionID.isEmpty())
	{
		// Constants for maximum attempts
		const int maxAttempts = 5;
		const int retryDelayMs = 1000; // 0.5-second delay

		// Try to insert production data up to 3 times
		bool insertProductionStatus = false;
		int productionAttemptCount = 0;

		while (!insertProductionStatus && productionAttemptCount < maxAttempts)
		{
			insertProductionStatus = _sqliteDatabase.insertProductionData(_productionInfo);
			productionAttemptCount++;

			if (!insertProductionStatus)
			{
				qDebug() << "Insert Production Status: failed on attempt " << productionAttemptCount << ", retrying...";
				QThread::msleep(retryDelayMs); // Delay before retrying
			}
		}

		// Final status messages
		qDebug() << "Insert Production Status: " << (insertProductionStatus ? "successful" : "failed after 3 attempts");

		bool overallStatus = true;
		//if (!insertDefectDataStatus)overallStatus = false;
		if(!insertProductionStatus)overallStatus = false;
		return overallStatus;
	}
	else
	{
		return false;
	}
	
}

void DataBaseThread::writeDefectVoInfo(QString& timeStamp, QString& productionID, QString& recipeID, QString& recipeName)
{
	QHash<QString, QVisionObject> visionObject;
	for (auto d : _defectResults)
	{
		QString voID = QString::fromStdString(d.algoDefResult.vo_id);
		QString tagName = d.tagNames.isEmpty() ? "Unknown" : d.tagNames[0];
		if (visionObject.contains(voID))
		{
			QString defectID = QString::fromStdString(d.algoDefResult.def_id);

			visionObject[voID].defIDs.append(defectID);
			if (!visionObject[voID].tagNameList.contains(tagName))
			{
				visionObject[voID].tagNameList.append(tagName);
			}

		}
		else
		{
			QVisionObject vo;
			vo.objectID = voID;
			vo.objectName = QString::fromStdString(d.algoDefResult.vo_name);
			vo.viewID = QString::fromStdString(d.view_id);
			vo.viewName = QString::fromStdString(d.view_name);
			vo.row_id = d.algoDefResult.vo_row_id;
			vo.col_id = d.algoDefResult.vo_col_id;;
			vo.island_id = d.algoDefResult.vo_island_id;
			vo.defIDs.append(QString::fromStdString(d.algoDefResult.def_id));
			vo.tagNameList.append(tagName);
			visionObject.insert(voID, vo);
		}
	}

	QJsonDocument jsonDoc;
	QJsonObject rootObj;
	QJsonArray voArr;
	for (auto vo : visionObject)
	{

		QJsonObject voObj;
		QString defectID = vo.defIDs.join(",");
		QString tagNameList = vo.tagNameList.join(",");
		QString voID = vo.objectID;
		QString voName = vo.objectName;
		QString viewID = vo.viewID;
		QString viewName = vo.viewName;

		int rowID = vo.row_id;
		int colID = vo.col_id;
		int islandID = vo.island_id;


		voObj.insert("timestamp", timeStamp);
		voObj.insert("productionID", productionID);
		voObj.insert("recipeID", recipeID);
		voObj.insert("recipeName", recipeName);
		voObj.insert("verificationStatus", 0);
		voObj.insert("defectId", defectID);
		voObj.insert("tagNameList", tagNameList);
		voObj.insert("voId", voID);
		voObj.insert("voName", voName);
		voObj.insert("ViewId", viewID);
		voObj.insert("viewName", viewName);

		voObj.insert("rowId", rowID);
		voObj.insert("colId", colID);
		voObj.insert("islandId", islandID);

		voArr.append(voObj);
	}

	rootObj.insert("DefectVoList", voArr);

	jsonDoc.setObject(rootObj);

	QString resultPath = Common::Directory::getProductionResultPath() + "/DefectVoInfo.json";
	QFile file(resultPath);
	if (file.open(QIODevice::WriteOnly))
	{
		file.write(jsonDoc.toJson());
		file.flush();
		file.close();
	}
	else
	{
	}
}

void DataBaseThread::runRecipeCollector()
{
	qDebug() << "RunRecipeCollector";
	qDebug() << "Template List: " << _templateIdList;

	QString recipeCollectorDir = _recipeCollectorPath + "/" + _productionInfo.recipeName;

	QDir dir(recipeCollectorDir);
	if (!dir.exists()) dir.mkpath(recipeCollectorDir);

	// copy full board image if not collected yet
	QString fullImgSource = Common::Directory::getRecipeImagesPath() + "/plane.jpg";
	QString fullImgDestination  = recipeCollectorDir + "/plane.jpg";

	if (!QFile::exists(fullImgDestination))
	{
		bool fullImgCopy = QFile::copy(fullImgSource, fullImgDestination);
		qDebug() << "Copy Full Img status: " << fullImgCopy;
	}

	// copy good image per template if not collected yet
	for (int i = 0; i < _templateIdList.size(); i++)
	{
		QString destinationFilePath = recipeCollectorDir + "/" + _templateIdList[i] + ".jpg" ;
		if (QFile::exists(destinationFilePath)) continue;

		QDir sourceDir(Common::Directory::getRecipeVidiImagePath()+"/"+ _templateIdList[i]);
		QStringList imageFilters;
		imageFilters << "*.jpg" << "*.png" << "*.bmp";
		QStringList imageFiles = sourceDir.entryList(imageFilters, QDir::Files);

		bool copySuccess = true;

		for (const QString& imageFile : imageFiles) {
			QString sourceFilePath = sourceDir.filePath(imageFile);

			qDebug() << "DESTINATION PATH: " << destinationFilePath;

			// Copy each image from source to destination
			if (!QFile::copy(sourceFilePath, destinationFilePath)) {
				copySuccess = false;  // Handle error if copy fails
				qWarning() << "Failed to copy" << sourceFilePath << "to" << destinationFilePath;
			}
		}

		if (copySuccess) {
			qDebug() << "All images copied successfully!";
		}
		else {
			qDebug() << "Some images failed to copy.";
		}
	}


	//QString goodImgSource = Common::Directory::getRecipeVidiImagePath() + "/ref_RGB.jpg";
	//QString goodImgDestination = recipeCollectorDir + "/ref_RGB.jpg";

	//bool goodImgCopy = QFile::copy(goodImgSource, goodImgDestination);
	//qDebug() << "Copy Good Img status: " << goodImgCopy;

	QString localRecipeCollector = "C:/Advanced/Data/RecipeCollector/" + _productionInfo.recipeName;
	bool copyToLocal = localRecipeCollector == recipeCollectorDir ? false:true;

	if (copyToLocal)
	{
		QDir dir1(localRecipeCollector);
		if (!dir1.exists()) dir.mkpath(localRecipeCollector);

		// copy good image if not collected yet
		QString fullImgSource = Common::Directory::getRecipeImagesPath() + "/plane.jpg";
		QString fullImgDestination = localRecipeCollector + "/plane.jpg";

		QString goodImgSource = Common::Directory::getRecipeVidiImagePath() + "/ref_RGB.jpg";
		QString goodImgDestination = localRecipeCollector + "/ref_RGB.jpg";

		if (!QFile::exists(fullImgDestination)) QFile::copy(fullImgSource, fullImgDestination);

		if (!QFile::exists(goodImgDestination)) QFile::copy(goodImgSource, goodImgDestination);
	}
}

void DataBaseThread::runDefectCollector()
{
	qDebug() << "RunDefectCollector";

	if (_defectCollectorPath.contains("C:/")) return;

	QString imgPathSource = Common::Directory::getProductionImageSetPath();
	QString resultPathSource = Common::Directory::getProductionResultPath();
	
	QString imgPathDestination=  _defectCollectorPath +"/" + _productionInfo.productionID +  "/Images/";
	QString resultPathDestination = _defectCollectorPath + "/" + _productionInfo.productionID + "/Results/";

	bool copyImages = copyFolderRecursively(imgPathSource, imgPathDestination);
	bool copyResult = copyFolderRecursively(resultPathSource, resultPathDestination);

	//qDebug() << "copyImages Status: " << copyImages;
	qDebug() << "copyResult Status: " << copyResult;
	
}

void DataBaseThread::setDefectCollectorPath(QString defectCollectorPath)
{
	_defectCollectorPath = defectCollectorPath;
}

void DataBaseThread::setRecipeCollectorPath(QString recipeCollectorPath)
{
	_recipeCollectorPath = recipeCollectorPath;
}

void DataBaseThread::setProductionExportMode(ProductionExportMode mode)
{
	_productionExportMode = mode;
}

QString DataBaseThread::productionExportModeName() const
{
	switch (_productionExportMode)
	{
	case ProductionExportMode::Recipe1:
		return "Recipe1";
	case ProductionExportMode::Recipe2:
		return "Recipe2";
	default:
		return "Normal";
	}
}

QString DataBaseThread::recipeProductionRootPath() const
{
	QString basePath;
	switch (_productionExportMode)
	{
	case ProductionExportMode::Recipe1:
		basePath = "C:/Advanced/Data/Recipe1";
		break;
	case ProductionExportMode::Recipe2:
		basePath = "C:/Advanced/Data/Recipe2";
		break;
	default:
		return "";
	}

	return QDir(basePath).filePath(_productionInfo.productionID);
}

bool DataBaseThread::copyFolderRecursively(const QString &srcFolderPath, const QString &destFolderPath) {
	QTime timer;
	timer.start();

	QDir sourceFolder(srcFolderPath);
	QDir destinationFolder(destFolderPath);

	// Ensure the source folder exists
	if (!sourceFolder.exists()) {
		qDebug() << "Source folder does not exist: " << srcFolderPath;
		return false;
	}

	// Create the destination folder if it doesn't exist
	if (!destinationFolder.exists()) {
		if (!destinationFolder.mkpath(".")) {
			qDebug() << "Failed to create destination folder: " << destFolderPath;
			return false;
		}
	}

	//QStringList files = sourceFolder.entryList(QDir::Files);
	//foreach(const QString &file, files) {
	//	if (file.contains("HeightMap") || file.contains("IMap")) continue; // heightmap transfer too slow



	//	QString srcFilePath = sourceFolder.filePath(file);
	//	QString destFilePath = destinationFolder.filePath(file);

	//	// Copy the file from source to destination
	//	if (!QFile::copy(srcFilePath, destFilePath)) {
	//		qDebug() << "Failed to copy file: " << srcFilePath;
	//		return false;
	//	}
	//}
	int imageIndex = 0;
	QStringList files = sourceFolder.entryList(QDir::Files);

	qDebug() << "Total Image waited to be copy: " << files.size();

	foreach(const QString & file, files) {
		if (file.contains("HeightMap") || file.contains("IMap")) continue; // heightmap transfer too slow
		if (file.contains("optic", Qt::CaseInsensitive) && file.contains("view", Qt::CaseInsensitive)) continue;
	
		// Condition: if contains "optic" and "view", must also contain "display"
		if (file.contains("optic", Qt::CaseInsensitive) && file.contains("view", Qt::CaseInsensitive)) {
			imageIndex++;
			if (!file.contains("_display", Qt::CaseInsensitive)) continue;

			// Remove "display" from filename
			QString modifiedFileName = file;
			modifiedFileName.replace("_display", "", Qt::CaseInsensitive).replace("  ", " ").trimmed();

			QString srcFilePath = sourceFolder.filePath(file);
			QString destFilePath = destinationFolder.filePath(modifiedFileName);

			qDebug() << "Copy Image Index: " << imageIndex;
			qDebug() << "Image: " << srcFilePath;

			if (!QFile::copy(srcFilePath, destFilePath)) {
				qDebug() << "Failed to copy file: " << srcFilePath;
				return false;
			}
			continue;
		}

		// Normal copy for other files
		QString srcFilePath = sourceFolder.filePath(file);
		QString destFilePath = destinationFolder.filePath(file);

		if (!QFile::copy(srcFilePath, destFilePath)) {
			qDebug() << "Failed to copy file: " << srcFilePath;
			return false;
		}
	}

	QStringList folders = sourceFolder.entryList(QDir::Dirs | QDir::NoDotAndDotDot);
	foreach(const QString &folder, folders) {
		QString srcFolderPath = sourceFolder.filePath(folder);
		QString destFolderPath = destinationFolder.filePath(folder);

		// Recursively copy subfolders
		if (!copyFolderRecursively(srcFolderPath, destFolderPath)) {
			return false;
		}	
	}

	int elapsedTime = timer.elapsed();
	qDebug() << "Copy  time: " << elapsedTime << "ms";

	return true;
}

void DataBaseThread::compileLaserMeasFile()
{
	
	QString laserMeasDir = "C:/Advanced/Cache/MeasurementData/";
	QDir directory(laserMeasDir);

	// Make sure the directory exists
	if (!directory.exists()) {
		qDebug() << "Directory does not exist.";
		return;
	}

	QHash<QString, QVector<LaserMeasurementInfo> > voLaserMeasInfo;

	// Get a list of all files in the directory
	QStringList files = directory.entryList(QStringList() << "*.json", QDir::Files);

	// Iterate through each file
	foreach(QString filename, files) {

		QString voName = filename;
		voName = voName.remove(".json");
		QString laserMeasFilePath = directory.absoluteFilePath(filename);

		if (voName.split("_").size() == 3) voName = voName.split("_")[2];


		QVector<LaserMeasurementInfo> laserMeasVector;
		bool readSuc = readLaserMeasJson(laserMeasFilePath, laserMeasVector);
		if (readSuc)
		{
			if (voLaserMeasInfo.contains(voName))
			{	
				voLaserMeasInfo[voName].append(laserMeasVector);		
			}
			else
			{
				voLaserMeasInfo.insert(voName, laserMeasVector);
			}
		
			QFile::remove(laserMeasFilePath);
		}

	}

	QJsonDocument jsonDoc;
	QJsonObject rootObj;
	QJsonArray voArr;
	for (auto it = voLaserMeasInfo.constBegin(); it != voLaserMeasInfo.constEnd(); ++it) {
		const QString& voName = it.key();
		QVector<LaserMeasurementInfo> cadOffsetVector = it.value();

		QJsonObject voObj;
	
		if (voName.isEmpty() || voName == QString("None")) continue;

		QVector<LaserMeasurementInfo> laserMeasVector = voLaserMeasInfo.value(voName);

		QJsonArray laserMeasArr;
		for (int i = 0; i < laserMeasVector.size(); i++)
		{
			QString cadName = laserMeasVector[i].cadName;
			QString cadFamily = laserMeasVector[i].cadFamily;
			double averageHeight_um = laserMeasVector[i].averageHeight_um;
			double tilt_um = laserMeasVector[i].tilt_um;
			bool isHeightPass = laserMeasVector[i].isHeightPass;
			bool isTiltPass = laserMeasVector[i].isTiltPass;
			double minHeight = laserMeasVector[i].minHeight;
			double maxHeight = laserMeasVector[i].maxHeight;
			double maxTilt = laserMeasVector[i].maxTilt;
			double area = laserMeasVector[i].area;
			double volume = laserMeasVector[i].volume;
			QString message = laserMeasVector[i].message;

			QJsonObject measObj;
			measObj.insert(QStringLiteral("cadName"), cadName);
			measObj.insert(QStringLiteral("cadFamily"), cadFamily);
			measObj.insert(QStringLiteral("averageHeight_um"), averageHeight_um);
			measObj.insert(QStringLiteral("tilt_um"), tilt_um);
			measObj.insert(QStringLiteral("isHeightPass"), isHeightPass);
			measObj.insert(QStringLiteral("isTiltPass"), isTiltPass);
			measObj.insert(QStringLiteral("minHeight"), minHeight);
			measObj.insert(QStringLiteral("maxHeight"), maxHeight);
			measObj.insert(QStringLiteral("maxTilt"), maxTilt);
			measObj.insert(QStringLiteral("message"), message);

			measObj.insert(QStringLiteral("topLeftHeight"), laserMeasVector[i].topLeftHeight);
			measObj.insert(QStringLiteral("topRightHeight"), laserMeasVector[i].topRightHeight);
			measObj.insert(QStringLiteral("bottomLeftHeight"), laserMeasVector[i].bottomLeftHeight);
			measObj.insert(QStringLiteral("bottomRightHeight"), laserMeasVector[i].bottomRightHeight);
			measObj.insert(QStringLiteral("measuringMethod"), laserMeasVector[i].measuringMethod);

			measObj.insert(QStringLiteral("area"), area);
			measObj.insert(QStringLiteral("volume"), volume);

			laserMeasArr.append(measObj);
		}
		voObj.insert(QStringLiteral("Vo_Name"), voName);
		voObj.insert(QStringLiteral("Laser_Measurement_List"), laserMeasArr);

		voArr.append(voObj);
	}

	rootObj.insert("Vo_Laser_Measurement_List", voArr);

	jsonDoc.setObject(rootObj);

	QString resultPath = Common::Directory::getProductionResultPath() + "/LaserMeasurement.json";
	QFile file(resultPath);
	if (file.open(QIODevice::WriteOnly))
	{
		file.write(jsonDoc.toJson());
		file.flush();
		file.close();

		qDebug() << "CompileLaserMeasFile success";

	}
	else
	{
		qDebug() << "CompileLaserMeasFile failed";
	}
}

void DataBaseThread::compileCadOffsetFile()
{
	QString cadOffsetDir = "C:/Advanced/Cache/CadOffset/";
	QDir directory(cadOffsetDir);

	// Make sure the directory exists
	if (!directory.exists()) {
		qDebug() << "Directory does not exist.";
		return;
	}

	QHash<QString, QVector<CadOffsetInfo> > voCadOffsetInfo;

	// Get a list of all files in the directory
	QStringList files = directory.entryList(QStringList() << "*.json", QDir::Files);

	// Iterate through each file
	foreach(QString filename, files) {

		QString voName = filename;
		voName = voName.remove(".json");
		QString cadFilePath = directory.absoluteFilePath(filename);
	

		QVector<CadOffsetInfo> cadOffsetVector;
		bool readSuc = readCadOffsetJson(cadFilePath, cadOffsetVector);
		if (readSuc)
		{
			voCadOffsetInfo.insert(voName, cadOffsetVector);
			QFile::remove(cadFilePath);
		}
		
	}

	QJsonDocument jsonDoc;
	QJsonObject rootObj;
	QJsonArray voArr;
	for (auto it = voCadOffsetInfo.constBegin(); it != voCadOffsetInfo.constEnd(); ++it) {
		const QString& voName = it.key();
		QVector<CadOffsetInfo> cadOffsetVector = it.value();

		if (voName.isEmpty() || voName == QString("None")) continue;

		QJsonObject voObj;

		QJsonArray cadOffsetArr;
		for (int i = 0; i < cadOffsetVector.size(); i++)
		{
			bool found = cadOffsetVector[i].isFound;
			QString cadName = cadOffsetVector[i].cadName;
			QString cadFamily = cadOffsetVector[i].cadFamily;
			double offSetX = cadOffsetVector[i].offsetX_perc;
			double offSetY = cadOffsetVector[i].offsetY_perc;
			double offSetX_um = cadOffsetVector[i].offsetX_um;
			double offSetY_um = cadOffsetVector[i].offsetY_um;

			QJsonObject cadObj;
			cadObj.insert(QStringLiteral("found"), found);
			cadObj.insert(QStringLiteral("cadName"), cadName);
			cadObj.insert(QStringLiteral("cadFamily"), cadFamily);
			cadObj.insert(QStringLiteral("offSetX_perc"), offSetX);
			cadObj.insert(QStringLiteral("offSetY_perc"), offSetY);
			cadObj.insert(QStringLiteral("offSetX_um"), offSetX_um);
			cadObj.insert(QStringLiteral("offSetY_um"), offSetY_um);

			cadObj.insert(QStringLiteral("goldenX"), cadOffsetVector[i].goldenX);
			cadObj.insert(QStringLiteral("goldenY"), cadOffsetVector[i].goldenY);
			cadObj.insert(QStringLiteral("goldenW"), cadOffsetVector[i].goldenW);
			cadObj.insert(QStringLiteral("goldenH"), cadOffsetVector[i].goldenH);

			cadOffsetArr.append(cadObj);
		}
		voObj.insert(QStringLiteral("Vo_Name"), voName);
		voObj.insert(QStringLiteral("Cad_Offset_List"), cadOffsetArr);

		voArr.append(voObj);
	}

	rootObj.insert("Vo_Cad_Offset_List",voArr);

	jsonDoc.setObject(rootObj);

	QString resultPath = Common::Directory::getProductionResultPath() + "/CadOffset.json";
	QFile file(resultPath);
	if (file.open(QIODevice::WriteOnly))
	{
		file.write(jsonDoc.toJson());
		file.flush();
		file.close();

		qDebug() << "CompileCadOffsetFile success";
	}
	else
	{
		qDebug() << "CompileCadOffsetFile failed";
	}

}

bool DataBaseThread::readCadOffsetJson(QString path, QVector<CadOffsetInfo>& cadOffsetVector)
{
	QVector<CadOffsetInfo> cOffsetVector;

	// Open the JSON file
	QFile file(path);
	if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
		qDebug() << "Failed to open JSON file: " << file.errorString();
		return false;
	}

	// Read the JSON data from the file
	QByteArray jsonData = file.readAll();
	file.close();

	// Parse the JSON data
	QJsonDocument jsonDoc = QJsonDocument::fromJson(jsonData);
	if (!jsonDoc.isObject()) {
		qDebug() << "Invalid JSON format.";
		return false;
	}

	QJsonObject jsonObj = jsonDoc.object();
	QJsonArray cadArray = jsonObj["Cad_Offset_List"].toArray();

	// Iterate through the array and populate the vector
	for (const QJsonValue &cadValue : cadArray) {
		QJsonObject cadObj = cadValue.toObject();
		CadOffsetInfo cadInfo;

		cadInfo.isFound = cadObj["found"].toBool();
		cadInfo.cadName = cadObj["cadName"].toString();
		cadInfo.cadFamily = cadObj["cadFamily"].toString();
		cadInfo.offsetX_perc = cadObj["offSetX_perc"].toDouble();
		cadInfo.offsetY_perc = cadObj["offSetY_perc"].toDouble();

		cadInfo.goldenX = cadObj["goldenX"].toDouble();
		cadInfo.goldenY = cadObj["goldenY"].toDouble();
		cadInfo.goldenW = cadObj["goldenW"].toDouble();
		cadInfo.goldenH = cadObj["goldenH"].toDouble();

		cadInfo.offsetX_um = cadInfo.offsetX_perc * _productionInfo.scalingUmPixel;
		cadInfo.offsetY_um = cadInfo.offsetY_perc * _productionInfo.scalingUmPixel;


		//cadInfo.offsetX_um = cadInfo.offSetX *1;
		//cadInfo.offsetY_um = cadInfo.offSetY * 1;

		cOffsetVector.append(cadInfo);
	}
	cadOffsetVector = cOffsetVector;
	return true;
}

bool DataBaseThread::readLaserMeasJson(QString path, QVector<LaserMeasurementInfo>& laserMeasVector)
{
	QVector<LaserMeasurementInfo> lMeasVector;

	// Open the JSON file
	QFile file(path);
	if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
		qDebug() << "Failed to open JSON file: " << file.errorString();
		return false;
	}

	// Read the JSON data from the file
	QByteArray jsonData = file.readAll();
	file.close();

	// Parse the JSON data
	QJsonDocument jsonDoc = QJsonDocument::fromJson(jsonData);
	if (!jsonDoc.isObject()) {
		qDebug() << "Invalid JSON format.";
		return false;
	}

	QJsonObject jsonObj = jsonDoc.object();
	QJsonArray measArray = jsonObj["Laser_Measurement_Info"].toArray();

	// Iterate through the array and populate the vector
	for (const QJsonValue &measValue : measArray) {
		QJsonObject measObj = measValue.toObject();
		LaserMeasurementInfo lMeas;

		lMeas.cadName = measObj["cadName"].toString();
		lMeas.cadFamily = measObj["cadFamily"].toString();
		lMeas.averageHeight_um = measObj["averageHeight_um"].toDouble();
		lMeas.tilt_um = measObj["tilt_um"].toDouble();
		lMeas.isHeightPass = measObj["isHeightPass"].toBool();
		lMeas.isTiltPass = measObj["isTiltPass"].toBool();
		lMeas.minHeight = measObj["minHeight"].toDouble();
		lMeas.maxHeight = measObj["maxHeight"].toDouble();
		lMeas.maxTilt = measObj["maxTilt"].toDouble();
		lMeas.message = measObj["message"].toString();

		lMeas.topLeftHeight = measObj["topLeftHeight"].toDouble();
		lMeas.topRightHeight = measObj["topRightHeight"].toDouble();
		lMeas.bottomLeftHeight = measObj["bottomLeftHeight"].toDouble();
		lMeas.bottomRightHeight = measObj["bottomRightHeight"].toDouble();
		lMeas.measuringMethod = measObj["measuringMethod"].toString();
		lMeas.area = measObj["area"].toDouble();
		lMeas.volume = measObj["volume"].toDouble();

		lMeasVector.append(lMeas);
	}
	laserMeasVector = lMeasVector;
	return true;
}

void DataBaseThread::updateConnectionStatus(bool connection)
{
	_connectionToVStation = connection;
}

DataBaseThread::~DataBaseThread()
{
}
