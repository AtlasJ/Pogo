#pragma once

#include <QObject>
#include <QThread>

#include <QFile>
#include <QDebug>

#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QTcpSocket>
#include <QTimer>
#include <QEventLoop>

class NetworkPathChecker : public QThread
{
	Q_OBJECT

public:
	NetworkPathChecker();
	~NetworkPathChecker();

	void setIpAddress(QString ipAddress);
	void run();

	//Fast reachability probe (TCP connect to SMB port 445). Bounded by timeoutMs,
	//unlike QFile::exists on a dead UNC path which blocks ~20s in the SMB stack.
	//Local paths (no leading //) return true so they fall through to file checks.
	static bool isReachable(QString ipAddress, int timeoutMs = 500);

private:
	
	QString _ipAddress;
	bool _isConnected = false;
	bool _isFirstAttempt = true;
	bool _forceCheckPath = false;
	


public slots:
	void checkPathExist();

signals:
	void updateConnection(bool);

};
