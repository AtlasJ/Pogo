#pragma once

#include <QString>
#include <QSqlDatabase>
#include <QSqlQuery>
#include <QSqlError>
#include <QVariantList>
#include <QVector>
#include <QDate>
#include <QElapsedTimer>
#include "AlgoDefectResult.h"
#include "QCommonStruct.h"
#include "UserAccount.h"


class SQLiteDatabase
{

public:
	SQLiteDatabase();
	~SQLiteDatabase();


	bool open(const QString& path);
	bool insertProductionData(const ProductionInfo& pInfo);
	bool insertPackageData(const PackageInfo& p);



	// user account
	bool getAccountInfo(AccountInfo receivedAccInfo, AccountInfo& authenticatedAccInfo, bool& accountExisted);
	bool openAccountInfoDatabase(QString path);


	bool addColumnIfNotExist(QSqlDatabase& db, QString tableName, QString columnName);
private:
	QSqlDatabase _db;
	//QSqlDatabase _dbCad;
	QSqlDatabase _userDb;
	QSqlDriver* _userDbDriver;

};

