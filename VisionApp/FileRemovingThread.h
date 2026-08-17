#pragma once

#include <QObject>
#include <QThread>

#include <QFile>
#include <QDebug>

#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QTimer>
#include <QEventLoop>
#include <QtConcurrent/QtConcurrent>
#include <QStorageInfo>
#include <QTimer>

class FileRemovingThread : public QThread
{
	Q_OBJECT
public:
	FileRemovingThread();
	~FileRemovingThread();

private:
	bool _autoRemoving;
	QStringList _clearingPathList;
	int _storageLimit;


	

public:
	

	double _availableSpace;
	double _percentageFilled;

	QStringList getClearingPathList();
	void setClearingPathList();
	void setSetting(bool autoRemoving, QStringList clearingPathList, int storageLimit);
	
	void manageRunResultFolder(QString resultPath);
	bool isDriveFullPercentageWise(QString drivePath, double percentage);
	void run();

signals:
	void updateDriveSpace();

public slots:
	void cleanUpFolder();


};

