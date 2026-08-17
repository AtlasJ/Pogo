#include "NetworkPathChecker.h"
#include "Logger.h"

NetworkPathChecker::NetworkPathChecker()	
{

}

NetworkPathChecker::~NetworkPathChecker()
{
}

void NetworkPathChecker::setIpAddress(QString ipAddress)
{

	_ipAddress = ipAddress;
	_forceCheckPath = true;
}

void NetworkPathChecker::run()
{
	//ct::logger::info("[QThread] Network path checker started");
	checkPathExist();
}

bool NetworkPathChecker::isReachable(QString ipAddress, int timeoutMs)
{
	QString host = ipAddress;
	host.replace('\\', '/');

	if (!host.startsWith("//")) return true; //local path, nothing to probe

	host = host.mid(2);
	int slash = host.indexOf('/');
	if (slash != -1) host = host.left(slash);
	if (host.isEmpty()) return false;

	QTcpSocket socket;
	socket.connectToHost(host, 445); //SMB
	bool connected = socket.waitForConnected(timeoutMs);
	socket.abort();
	return connected;
}

void NetworkPathChecker::checkPathExist()
{
	bool prevConnection = _isConnected;

	if (!isReachable(_ipAddress))
	{
		_isConnected = false;
	}
	else
	{
		QString defectPackagePath = _ipAddress + "/Advanced";

		QFile file(defectPackagePath);
		_isConnected = file.exists();
	}

	if (prevConnection != _isConnected || _isFirstAttempt || _forceCheckPath)
	{	
		_isFirstAttempt = false;
		_forceCheckPath = false;
		emit updateConnection(_isConnected);
	}

}

//void NetworkPathChecker::checkPathExist()
//{
//	//bool prevConnection = _isConnected;
//	//qDebug() << "Ipaddress: " << _ipAddress;
//	//// Use QNetworkAccessManager to check if the IP address is reachable
//	//QUrl url(_ipAddress + "/Advanced");
//	//QNetworkAccessManager manager;
//	//QNetworkRequest request(url);
//	//QNetworkReply* reply = manager.get(request);
//
//	//qDebug() << "URL: " << url.url();
//
//	//// Use a QTimer for a timeout in milliseconds (adjust the timeout value as needed)
//	//QTimer timeoutTimer;
//	//timeoutTimer.setSingleShot(true);
//	//timeoutTimer.start(2500);  // Set a 5-second timeout
//
//	//						   // Create an event loop to wait for the network request or timeout
//	//QEventLoop loop;
//	//connect(reply, &QNetworkReply::finished, &loop, &QEventLoop::quit);
//	//connect(&timeoutTimer, &QTimer::timeout, &loop, &QEventLoop::quit);
//
//	//// Wait for either the network request to complete or the timeout to occur
//	//loop.exec();
//
//	//// Check the result of the network request
//	//if (reply->error() == QNetworkReply::NoError)
//	//{
//	//	// The IP address is reachable
//	//	_isConnected = true;
//	//	qDebug() << "No error";
//	//}
//	//else
//	//{
//	//	// The IP address is not reachable or there was an error
//	//	_isConnected = false;
//	//	qDebug() << "Error";
//	//}
//
//	//reply->deleteLater();
//
//	//if (prevConnection != _isConnected)
//	//{
//	//	emit updateConnection(_isConnected);
//	//}
//}

