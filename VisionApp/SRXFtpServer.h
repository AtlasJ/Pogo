#pragma once

#include <QObject>
#include <QTcpServer>
#include <QTcpSocket>
#include <QFile>
#include <QHostAddress>

/*
* Minimal embedded FTP server for the Keyence SR-X readers' image push.
* The reader connects in, logs in (any user/pass accepted), and STORs
* the capture JPG plus the historical-data JSON. Files are written flat
* into the drop folder; fileReceived() fires per completed upload.
*
* Supports both active (PORT) and passive (PASV) data connections -
* the reader defaults to active mode.
*/
class SRXFtpServer : public QObject {
	Q_OBJECT

public:
	explicit SRXFtpServer(QObject* parent = nullptr);
	~SRXFtpServer();

	bool start(int port, const QString& dropPath);
	void stop();
	bool isRunning() const;

signals:
	void fileReceived(QString peerIp, QString filePath);

private:
	struct Session {
		QTcpSocket* control = nullptr;
		QTcpSocket* data = nullptr;
		QTcpServer* pasvServer = nullptr;
		QHostAddress activeAddr;   //PORT target
		quint16 activePort = 0;
		bool passive = false;
		QFile file;
		QString fileName;
		QByteArray lineBuffer;
	};

	void onNewControlConnection();
	void onControlReadyRead(Session* s);
	void handleCommand(Session* s, const QString& line);
	void beginStor(Session* s, const QString& name);
	void openDataChannel(Session* s);
	void onDataReadyRead(Session* s);
	void finishStor(Session* s, bool ok);
	void closeSession(Session* s);
	void reply(Session* s, const QString& line);

	QTcpServer* m_server = nullptr;
	QString m_dropPath;
	QList<Session*> m_sessions;
};
