#include "SQLiteDatabase.h"
#include <QDebug>
#include "uidGenerator.h"
#include "Timer.h"

static QSet<QString> existingColumns(QSqlDatabase& db, const QString& table)
{
	QSet<QString> cols;
	QSqlQuery q(db);
	if (!q.exec(QString("PRAGMA table_info(%1);").arg(table))) {
		qDebug() << "PRAGMA table_info failed:" << table << q.lastError().text();
		return cols;
	}

	while (q.next()) {
		cols.insert(q.value(1).toString().toUpper()); // column name
	}
	return cols;
}

static bool addColumnIfMissing(QSqlDatabase& db,
	const QString& table,
	const QString& colName,
	const QString& colType,            // "TEXT" / "INTEGER" / "REAL"
	const QString& defaultSql = "")     // e.g. "0", "''", "CURRENT_TIMESTAMP"
{
	const QSet<QString> cols = existingColumns(db, table);
	qDebug() << "cols:" << cols;
	if (cols.contains(colName.toUpper()))
		return true;

	QString sql = QString("ALTER TABLE %1 ADD COLUMN %2 %3")
		.arg(table, colName, colType);

	if (!defaultSql.isEmpty())
		sql += QString(" DEFAULT %1").arg(defaultSql);

	QSqlQuery q(db);
	if (!q.exec(sql)) {
		qDebug() << "ALTER TABLE add column failed:" << sql << q.lastError().text();
		return false;
	}

	qDebug() << "Added missing column:" << table << colName << colType;
	return true;
}


SQLiteDatabase::SQLiteDatabase()
{
}

SQLiteDatabase::~SQLiteDatabase()
{
}


bool SQLiteDatabase::open(const QString& path)
{
	const QString connectionName = "general";

	if (QSqlDatabase::contains(connectionName)) {
		{
			QSqlDatabase oldDb = QSqlDatabase::database(connectionName);
			if (oldDb.isOpen()) oldDb.close();
		} // oldDb destroyed here
		QSqlDatabase::removeDatabase(connectionName);
	}

	_db = QSqlDatabase::addDatabase("QSQLITE", connectionName);
	_db.setDatabaseName(path);

	// Strongly recommended for concurrency with other apps:
	_db.setConnectOptions("QSQLITE_BUSY_TIMEOUT=3000");

	if (!_db.open()) {
		qDebug() << "Failed to open Database:" << _db.lastError().text();
		return false;
	}

	QSqlQuery query(_db);

	//// Recommended WAL (better read/write concurrency)
	//if (!query.exec("PRAGMA journal_mode=WAL;"))
	//	qDebug() << "WAL failed:" << query.lastError().text();
	//query.exec("PRAGMA synchronous=NORMAL;"); // optional

	//1. check if table exist
	//2. check if every column exist, if not add the column in

	const QString tableName = "PRODUCTION_DATA";

	QString createTable =
		"CREATE TABLE IF NOT EXISTS PRODUCTION_DATA ("
		"TIME_STAMP TEXT, RECIPE_ID TEXT, RECIPE_NAME TEXT, PRODUCTION_ID TEXT, PRODUCTION_FILE_NAME TEXT, "
		"CYCLE_TIME REAL, TOTAL_DEFECT INT, IS_PASS INT, IS_ERROR INT, REPORT INT, REPORT_TIME_STAMP TEXT, "
		"MACHINE_ID TEXT, LOT_NUMBER TEXT, PACKAGE_NAME TEXT, OPERATOR TEXT, INSPECTION_TYPE TEXT, BARCODE TEXT, TOTAL_UNIT INT, "
		"PACKAGE_UUID TEXT, RECIPE_TYPE INT, FIDUCIAL_STATUS TEXT, PRODUCTION_MODE INT, DEFECT_UNITS INT, BUYOFF_DEFECT_UNITS INT, "
		"PASS_YIELD_PERCENTAGE REAL, YIELD REAL, IS_VERIFIED INT, FALSE_CALL INT, TRUE_CALL INT"
		");";

	if (!query.exec(createTable)) {
		qDebug() << "Failed to create table:" << query.lastError().text();
		return false;
	}

	if (!query.exec("CREATE INDEX IF NOT EXISTS idx_production_id ON PRODUCTION_DATA(PRODUCTION_ID);")) {
		qDebug() << "Failed to create index:" << query.lastError().text();
		return false;
	}

	// 2) Ensure all columns exist (migrate-in-place)
	struct ColDef {
		const char* name;
		const char* type;        // SQLite types: TEXT, INTEGER, REAL, BLOB, etc.
		const char* def;         // DEFAULT SQL literal, e.g. "0", "''", "CURRENT_TIMESTAMP". empty = none
	};

	const ColDef requiredCols[] = {
	   {"MACHINE_ID", "TEXT", ""},
	   {"TIME_STAMP", "TEXT", ""},
	   {"RECIPE_ID", "TEXT", ""},
	   {"RECIPE_NAME", "TEXT", ""},
	   {"PRODUCTION_ID", "TEXT", ""},
	   {"PRODUCTION_FILE_NAME", "TEXT", ""},
	   {"CYCLE_TIME", "REAL", "0"},
	   {"TOTAL_DEFECT", "INTEGER", "0"},
	   {"IS_PASS", "INTEGER", "0"},
	   {"IS_ERROR", "INTEGER", "0"},
	   {"REPORT", "INTEGER", "0"},
	   {"REPORT_TIME_STAMP", "TEXT", ""},
	   {"MACHINE_ID", "TEXT", ""},
	   {"LOT_NUMBER", "TEXT", ""},
	   {"PACKAGE_NAME", "TEXT", ""},
	   {"OPERATOR", "TEXT", ""},
	   {"INSPECTION_TYPE", "TEXT", ""},
	   {"BARCODE", "TEXT", ""},
	   {"TOTAL_UNIT", "INTEGER", "0"},
	   {"PACKAGE_UUID", "TEXT", ""},
	   {"RECIPE_TYPE", "INTEGER", "0"},
	   {"FIDUCIAL_STATUS", "TEXT", ""},
	   {"PRODUCTION_MODE", "INTEGER", "0"},
	   {"DEFECT_UNITS", "INTEGER", "0"},
	   {"BUYOFF_DEFECT_UNITS", "INTEGER", "0"},
	   {"PASS_YIELD_PERCENTAGE", "REAL", "0"},
	   {"YIELD", "REAL", "0"},
	   {"IS_VERIFIED", "INTEGER", "0"},
	   {"FALSE_CALL", "INTEGER", "0"},
	   {"TRUE_CALL", "INTEGER", "0"},
	};

	for (const auto& c : requiredCols) {
		if (!addColumnIfMissing(_db, tableName, c.name, c.type, c.def)) {
			return false;
		}
	}
	qDebug() << "add Column if missing";


	QString createPackageDataTable =
		"CREATE TABLE IF NOT EXISTS PACKAGE_DATA ("
		"TIME_STAMP TEXT, "
		"PACKAGE_UUID TEXT, "
		"RECIPE_LIST TEXT, "
		"BARCODE_LIST TEXT, "
		"PRODUCTION_ID_LIST TEXT, "
		"PACKAGE_ID_LIST TEXT, "
		"CYCLE_TIME REAL, "
		"IS_PACKAGED INT, "
		"PACKAGED_TIME_STAMP TEXT"
		");";


	if (!query.exec(createPackageDataTable)) {
		qDebug() << "Failed to create table:" << query.lastError().text();
		return false;
	}

	if (!query.exec("CREATE INDEX IF NOT EXISTS idx_package_id ON PACKAGE_DATA(PACKAGE_UUID);")) {
		qDebug() << "Failed to create index:" << query.lastError().text();
		return false;
	}


	
	QString createPackagedProductionDataTable =
		"CREATE TABLE IF NOT EXISTS PACKAGED_PRODUCTION_DATA ("
		"TIME_STAMP TEXT, "
		"PRODUCTION_ID TEXT, "
		"BARCODE TEXT, "
		"PACKAGE_UUID TEXT, "
		"MAIN_RECIPE TEXT, "
		"SUB_RECIPE TEXT, "
		"CYCLE_TIME_MAIN REAL, "
		"CYCLE_TIME_SUB REAL, "
		"CYCLE_TIME_TOTAL REAL, "
		"DEFECT_COUNT_MAIN INT, "
		"DEFECT_COUNT_SUB INT, "
		"DEFECT_COUNT_TOTAL INT, "
		"IS_VERIFIED INT, "
		"REPORT INT "
		");";

	//addColumnIfNotExist(_db, QString("PACKAGED_PRODUCTION_DATA"), QString("REPORT"));

	if (!query.exec(createPackagedProductionDataTable)) {
		qDebug() << "Failed to create table:" << query.lastError().text();
		return false;
	}

	if (!query.exec("CREATE INDEX IF NOT EXISTS idx_production_id ON PACKAGED_PRODUCTION_DATA(PRODUCTION_ID);")) {
		qDebug() << "Failed to create index:" << query.lastError().text();
		return false;
	}

	qDebug() << "Successfully open Database";
	return true;
}

bool SQLiteDatabase::addColumnIfNotExist(QSqlDatabase& db, QString tableName, QString columnName)
{
	QSqlQuery query(db);

	bool querySuc = query.prepare("PRAGMA table_info(" + tableName + ")");
	query.exec();
	bool columnExists = false;
	while (query.next()) {
		if (query.value(1).toString() == columnName) {
			columnExists = true;
			break;
		}
	}

	// If the column doesn't exist, add it
	if (!columnExists) {
		QSqlQuery alterQuery(_db);
		if (!alterQuery.exec("ALTER TABLE "+tableName+" ADD COLUMN " + columnName +" INT")) {
			qDebug() << "Error adding column:" << alterQuery.lastError().text();
			return false;
		}
	}
	else { qDebug() << "Column existed:" << columnName; }
	
	return true;
}

bool SQLiteDatabase::insertProductionData(const ProductionInfo& p)
{
	QElapsedTimer timer;
	timer.start();
	bool flag = true;

	if (_db.isOpen())
	{
		QSqlQuery query(_db);

		query.prepare(
			"INSERT INTO PRODUCTION_DATA ("
			"TIME_STAMP, "
			"RECIPE_ID, "
			"RECIPE_NAME, "
			"PRODUCTION_ID, "
			"PRODUCTION_FILE_NAME, "
			"CYCLE_TIME, "
			"TOTAL_DEFECT, "
			"IS_PASS, "
			"IS_ERROR, "
			"REPORT, "
			"REPORT_TIME_STAMP, "
			"MACHINE_ID, "
			"LOT_NUMBER, "
			"PACKAGE_NAME, "
			"OPERATOR, "
			"INSPECTION_TYPE, "
			"BARCODE, "
			"TOTAL_UNIT, "
			"PACKAGE_UUID, "
			"RECIPE_TYPE, "
			"FIDUCIAL_STATUS, "
			"PRODUCTION_MODE, "
			"DEFECT_UNITS, "
			"BUYOFF_DEFECT_UNITS, "
			"PASS_YIELD_PERCENTAGE, "
			"YIELD, "
			"IS_VERIFIED, "
			"FALSE_CALL, "
			"TRUE_CALL"
			") VALUES ("
			":timeStamp, "
			":recipeID, "
			":recipeName, "
			":productionID, "
			":productionFileName, "
			":cycleTime, "
			":totalDefect, "
			":isPass, "
			":isError, "
			":report, "
			":reportTimeStamp, "
			":machineID, "
			":lotNumber, "
			":packageName, "
			":operator, "
			":inspectionType, "
			":barcode, "
			":totalUnit, "
			":packageUUID, "
			":recipeType, "
			":fiducialStatus, "
			":productionMode, "
			":defectUnits, "
			":buyOffDefectUnits, "
			":passYieldPerc, "
			":yield, "
			":isVerified, "
			":falseCall, "
			":trueCall"
			")"
		);


		query.bindValue(":timeStamp", p.timestamp);
		query.bindValue(":recipeID", p.recipeID);
		query.bindValue(":recipeName", p.recipeName);
		query.bindValue(":productionID", p.productionID);
		query.bindValue(":productionFileName", p.productionFileName);
		query.bindValue(":cycleTime", p.cycleTime);
		query.bindValue(":totalDefect", p.totalDefect);
		query.bindValue(":isPass", p.isPass ? 1 : 0);
		query.bindValue(":isError", p.isError ? 1 : 0);
		query.bindValue(":report", p.report ? 1 : 0);
		query.bindValue(":reportTimeStamp", p.reportTimestamp);

		query.bindValue(":machineID", p.machineID);
		query.bindValue(":lotNumber", p.lotNumber);
		query.bindValue(":packageName", p.packageName);
		query.bindValue(":operator", p._operator);
		query.bindValue(":inspectionType", p.inspectionType);
		query.bindValue(":barcode", p.stripeID); // CHANGE HERE

		query.bindValue(":totalUnit", p.totalUnit);
		query.bindValue(":packageUUID", 0); // CHANGE HERE
		query.bindValue(":recipeType", 0); 

		query.bindValue(":fiducialStatus", p.fiducialStatus.join(",")); // TEXT
		query.bindValue(":productionMode", p.productionMode ? 1 : 0);

		query.bindValue(":defectUnits", p.defectUnits);
		query.bindValue(":buyOffDefectUnits", 0);

		query.bindValue(":passYieldPerc", p.passYieldPerc);
		query.bindValue(":yield", p.yieldPerc);

		query.bindValue(":isVerified", false);
		query.bindValue(":falseCall", 0);
		query.bindValue(":trueCall", 0);
	
		// Execute the query and check for errors
		if (!query.exec()) {
			qDebug() << "insert production data failed:" << query.lastError().text();
			flag = false;
		}
		else
		{
			qDebug() << "insert production data suc";
		}
	}
	else
	{
		flag = false;
		//_lastError = QStringLiteral("Database not opened");
	}
	qint64 elapsedTime = timer.elapsed();
	qDebug() << "Time taken for insert PRODUCTION_DATA operation (ms):" << elapsedTime;
	return flag;
}

bool SQLiteDatabase::insertPackageData(const PackageInfo& p)
{
	if (!_db.isOpen()) {
		qDebug() << "Database not open";
		return false;
	}

	QSqlQuery query(_db);

	query.prepare(
		"INSERT INTO PACKAGE_DATA ("
		"TIME_STAMP, "
		"PACKAGE_UUID, "
		"RECIPE_LIST, "
		"BARCODE_LIST, "
		"PRODUCTION_ID_LIST, "
		"PACKAGE_ID_LIST, "
		"CYCLE_TIME, "
		"IS_PACKAGED"
		") VALUES ("
		":timeStamp, "
		":packageUuid, "
		":recipeList, "
		":barcodeList, "
		":productionIdList, "
		":packageIdList, "
		":cycleTime, "
		":isPackaged"
		")"
	);
	query.bindValue(":timeStamp", p.timeStamp);
	query.bindValue(":packageUuid", p.packageUuid);
	query.bindValue(":recipeList", p.recipeList.join(","));   // CSV
	query.bindValue(":barcodeList", p.barcodeList.join(",")); // CSV
	query.bindValue(":productionIdList", p.productionIdList);
	query.bindValue(":packageIdList", p.packageUuid); // reuse UUID unless you have another ID
	query.bindValue(":cycleTime", p.cycleTime);
	query.bindValue(":isPackaged", 0); // default false

	if (!query.exec()) {
		qDebug() << "insertPackageData failed:" << query.lastError().text();
		return false;
	}

	return true;
}

// user account info
bool SQLiteDatabase::openAccountInfoDatabase(QString path)
{
	QString connectionName = "userDatabaseConnection";
	if (_userDb.contains(connectionName))
	{
		_userDb.close();
		_userDb.removeDatabase(connectionName);
		_userDb = QSqlDatabase();
	}

	_userDb = QSqlDatabase::addDatabase("QSQLITE", connectionName);
	_userDb.setDatabaseName(path);
	_userDbDriver = _userDb.driver();

	bool success = _userDb.open();

	if (success) {

		QString createUserTableQuery = "CREATE TABLE IF NOT EXISTS USER_ACCOUNT_DATA "
			"(USER_ID TEXT,USER_NAME TEXT,PASSWORD TEXT, ACCESS_LEVEL INT)";

		QSqlQuery query(_userDb);
		query.prepare(createUserTableQuery);
		success = query.exec();
		if (!success) {
			qDebug() << "Failed to create user table";
		}

	}

	return success;
}

bool SQLiteDatabase::getAccountInfo(AccountInfo receivedAccInfo, AccountInfo& authenticatedAccInfo, bool& accountExisted)
{
	QSqlQuery query(_userDb);
	query.prepare("SELECT USER_ID, USER_NAME ,PASSWORD, ACCESS_LEVEL FROM USER_ACCOUNT_DATA WHERE USER_NAME = :userName AND PASSWORD= :password ");
	query.bindValue(":userName", receivedAccInfo.userName);
	query.bindValue(":password", receivedAccInfo.password);

	accountExisted = false;

	if (query.exec()) {
		while (query.next()) {
			authenticatedAccInfo.userID = query.value("USER_ID").toString();
			authenticatedAccInfo.userName = query.value("USER_NAME").toString();
			authenticatedAccInfo.password = query.value("PASSWORD").toString();
			authenticatedAccInfo.accessLevel = static_cast<AccessLevel>(query.value("ACCESS_LEVEL").toInt());
			accountExisted = true;

		}
		return true;
	}
	else
	{
		return false;
	}
}
