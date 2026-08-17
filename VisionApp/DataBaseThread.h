#include <QObject>
#include <QThread>
#include <QFile>
#include <QDir>
#include <QDebug>
#include <QtConcurrent/QtConcurrent>

#include "CommonDir.h"
#include "SQLiteDatabase.h"



#pragma once
class DataBaseThread : public QThread
{
	Q_OBJECT
public:
	enum class ProductionExportMode {
		Normal,
		Recipe1,
		Recipe2
	};

	DataBaseThread();
	~DataBaseThread();


	void setSetting(
		SQLiteDatabase& sqliteDatabase,
		QVector<ct::DefectResult>& defectResults,
		const ProductionInfo& productionInfo,
		QStringList templateIDList,
		bool isReadyToPackaged,
		const PackageInfo& packageInfo
	);
	void run();
	bool insertProductionToDataBase();
	void runDefectCollector();
	void runRecipeCollector();
	void updateConnectionStatus(bool connection);
	bool copyFolderRecursively(const QString &srcFolderPath, const QString &destFolderPath);
	void setDefectCollectorPath(QString defectCollectorPath);
	void setRecipeCollectorPath(QString recipeCollectorPath);
	void setProductionExportMode(ProductionExportMode mode);

	void compileLaserMeasFile();
	void compileCadOffsetFile();
	bool readCadOffsetJson(QString path, QVector<CadOffsetInfo>& cadOffsetVector);
	bool readLaserMeasJson(QString path, QVector<LaserMeasurementInfo>& laserMeasVector);
	bool writeProductionDataJson(QString path);
	bool writeProductionDbInsertJson(QString path);

	void writeDefectVoInfo(QString& timeStamp, QString& productionID, QString& recipeID, QString& recipeName);


	
private:
	SQLiteDatabase _sqliteDatabase;
	QVector<ct::DefectResult> _defectResults;
	ProductionInfo _productionInfo;
	bool _connectionToVStation;
	QStringList _templateIdList;
	QString _defectCollectorPath;
	QString _recipeCollectorPath;
	bool _isReadyToPackage;
	PackageInfo _packageInfo;
	ProductionExportMode _productionExportMode = ProductionExportMode::Normal;

	QString recipeProductionRootPath() const;
	QString productionExportModeName() const;

signals:
	void signalDatabaseStatus(bool);


};

