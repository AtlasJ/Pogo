#include "SRXFtpServer.h"
#include "Logger.h"

#include <QDir>
#include <QFileInfo>

SRXFtpServer::SRXFtpServer(QObject* parent)
	: QObject(parent)
{
}

SRXFtpServer::~SRXFtpServer()
{
	stop();
}

bool SRXFtpServer::start(int port, const QString& dropPath)
{
	stop();

	m_dropPath = dropPath;
	QDir().mkpath(m_dropPath);

	m_server = new QTcpServer(this);
	connect(m_server, &QTcpServer::newConnection, this, &SRXFtpServer::onNewControlConnection);

	if (!m_server->listen(QHostAddress::Any, port)) {
		ct::logger::error("[SRX FTP] Failed to listen on port %d: %s. Is another FTP service (e.g. SR Management Tool) running?",
			port, m_server->errorString().toStdString().c_str());
		m_server->deleteLater();
		m_server = nullptr;
		return false;
	}

	ct::logger::info("[SRX FTP] Listening on port %d, drop folder: %s", port, m_dropPath.toStdString().c_str());
	return true;
}

void SRXFtpServer::stop()
{
	while (!m_sessions.isEmpty()) closeSession(m_sessions.first());

	if (m_server) {
		m_server->close();
		m_server->deleteLater();
		m_server = nullptr;
	}
}

bool SRXFtpServer::isRunning() const
{
	return m_server && m_server->isListening();
}

void SRXFtpServer::onNewControlConnection()
{
	while (m_server && m_server->hasPendingConnections()) {
		auto socket = m_server->nextPendingConnection();

		auto s = new Session();
		s->control = socket;
		m_sessions.append(s);

		ct::logger::info("[SRX FTP] Control connection from %s", socket->peerAddress().toString().toStdString().c_str());

		//every handler re-checks membership: closeSession() deletes s, but queued
		//socket signals may still fire afterwards
		connect(socket, &QTcpSocket::readyRead, this, [=]() { if (m_sessions.contains(s)) onControlReadyRead(s); });
		connect(socket, &QTcpSocket::disconnected, this, [=]() { if (m_sessions.contains(s)) closeSession(s); });

		reply(s, "220 VisionApp FTP ready");
	}
}

void SRXFtpServer::onControlReadyRead(Session* s)
{
	s->lineBuffer.append(s->control->readAll());

	int pos;
	while ((pos = s->lineBuffer.indexOf('\n')) >= 0) {
		QString line = QString::fromUtf8(s->lineBuffer.left(pos)).remove('\r').trimmed();
		s->lineBuffer.remove(0, pos + 1);
		if (!line.isEmpty()) handleCommand(s, line);
	}
}

void SRXFtpServer::handleCommand(Session* s, const QString& line)
{
	const QString cmd = line.section(' ', 0, 0).toUpper();
	const QString arg = line.section(' ', 1).trimmed();

	if (cmd != "PASS") ct::logger::debug("[SRX FTP] << %s", line.toStdString().c_str());

	if (cmd == "USER") { reply(s, "331 Password required"); return; }
	if (cmd == "PASS") { reply(s, "230 Logged in"); return; }
	if (cmd == "SYST") { reply(s, "215 UNIX Type: L8"); return; }
	if (cmd == "FEAT") { reply(s, "211 END"); return; }
	if (cmd == "TYPE") { reply(s, "200 Type set"); return; }
	if (cmd == "NOOP") { reply(s, "200 OK"); return; }
	if (cmd == "PWD" || cmd == "XPWD") { reply(s, "257 \"/\""); return; }
	if (cmd == "CWD" || cmd == "XCWD") { reply(s, "250 OK"); return; } //flat store, directories ignored
	if (cmd == "MKD" || cmd == "XMKD") { reply(s, "257 \"/\" created"); return; }
	if (cmd == "DELE") { reply(s, "250 OK"); return; }
	if (cmd == "QUIT") { reply(s, "221 Bye"); s->control->disconnectFromHost(); return; }

	if (cmd == "PORT") {
		auto parts = arg.split(',');
		if (parts.size() != 6) { reply(s, "501 Bad PORT"); return; }

		s->activeAddr = QHostAddress(QString("%1.%2.%3.%4").arg(parts[0], parts[1], parts[2], parts[3]));
		s->activePort = (quint16)(parts[4].toInt() * 256 + parts[5].toInt());
		s->passive = false;
		reply(s, "200 PORT OK");
		return;
	}

	if (cmd == "PASV") {
		if (s->pasvServer) { s->pasvServer->deleteLater(); s->pasvServer = nullptr; }

		s->pasvServer = new QTcpServer(this);
		if (!s->pasvServer->listen(QHostAddress::Any, 0)) {
			reply(s, "425 Cannot open passive port");
			s->pasvServer->deleteLater();
			s->pasvServer = nullptr;
			return;
		}

		s->passive = true;

		//the address the client should dial back: our end of the control connection
		const quint32 ip = s->control->localAddress().toIPv4Address();
		const quint16 port = s->pasvServer->serverPort();
		reply(s, QString("227 Entering Passive Mode (%1,%2,%3,%4,%5,%6)")
			.arg((ip >> 24) & 0xFF).arg((ip >> 16) & 0xFF).arg((ip >> 8) & 0xFF).arg(ip & 0xFF)
			.arg(port / 256).arg(port % 256));
		return;
	}

	if (cmd == "STOR") {
		if (arg.isEmpty()) { reply(s, "501 Missing file name"); return; }
		beginStor(s, arg);
		return;
	}

	reply(s, "502 Not implemented");
}

void SRXFtpServer::beginStor(Session* s, const QString& name)
{
	//strip any path the reader sends; store flat in the drop folder
	s->fileName = QFileInfo(name.trimmed().replace('\\', '/')).fileName();

	if (s->fileName.isEmpty()) { reply(s, "553 Bad file name"); return; }

	s->file.setFileName(m_dropPath + "/" + s->fileName);
	if (!s->file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
		ct::logger::error("[SRX FTP] Cannot open %s for writing", s->file.fileName().toStdString().c_str());
		reply(s, "550 Cannot create file");
		return;
	}

	reply(s, "150 Opening data connection");
	openDataChannel(s);
}

void SRXFtpServer::openDataChannel(Session* s)
{
	if (s->passive) {
		if (!s->pasvServer) { finishStor(s, false); return; }

		auto adopt = [=](QTcpSocket* dataSocket) {
			s->data = dataSocket;
			connect(s->data, &QTcpSocket::readyRead, this, [=]() { if (m_sessions.contains(s)) onDataReadyRead(s); });
			connect(s->data, &QTcpSocket::disconnected, this, [=]() { if (m_sessions.contains(s)) finishStor(s, true); });
			onDataReadyRead(s); //flush anything already buffered
		};

		if (s->pasvServer->hasPendingConnections()) {
			adopt(s->pasvServer->nextPendingConnection());
		}
		else {
			connect(s->pasvServer, &QTcpServer::newConnection, this, [=]() {
				if (m_sessions.contains(s) && !s->data && s->pasvServer && s->pasvServer->hasPendingConnections()) {
					adopt(s->pasvServer->nextPendingConnection());
				}
			});
		}
	}
	else {
		//active mode: we dial the client's data port
		s->data = new QTcpSocket(this);
		connect(s->data, &QTcpSocket::readyRead, this, [=]() { if (m_sessions.contains(s)) onDataReadyRead(s); });
		connect(s->data, &QTcpSocket::disconnected, this, [=]() { if (m_sessions.contains(s)) finishStor(s, true); });
		s->data->connectToHost(s->activeAddr, s->activePort);

		if (!s->data->waitForConnected(3000)) {
			ct::logger::error("[SRX FTP] Active data connect to %s:%d failed",
				s->activeAddr.toString().toStdString().c_str(), s->activePort);
			finishStor(s, false);
		}
	}
}

void SRXFtpServer::onDataReadyRead(Session* s)
{
	if (!s->data || !s->file.isOpen()) return;
	s->file.write(s->data->readAll());
}

void SRXFtpServer::finishStor(Session* s, bool ok)
{
	if (!s->file.isOpen()) return; //already finished

	if (s->data) {
		s->file.write(s->data->readAll()); //drain remaining bytes
		s->data->deleteLater();
		s->data = nullptr;
	}

	const QString path = s->file.fileName();
	s->file.close();

	if (s->pasvServer) { s->pasvServer->deleteLater(); s->pasvServer = nullptr; }

	if (ok) {
		reply(s, "226 Transfer complete");
		ct::logger::info("[SRX FTP] Received %s (%lld bytes) from %s",
			s->fileName.toStdString().c_str(), QFileInfo(path).size(),
			s->control->peerAddress().toString().toStdString().c_str());
		emit fileReceived(s->control->peerAddress().toString(), path);
	}
	else {
		reply(s, "426 Transfer failed");
		QFile::remove(path);
	}
}

void SRXFtpServer::closeSession(Session* s)
{
	m_sessions.removeAll(s);

	if (s->file.isOpen()) s->file.close();
	if (s->data) s->data->deleteLater();
	if (s->pasvServer) s->pasvServer->deleteLater();
	if (s->control) s->control->deleteLater();

	delete s;
}

void SRXFtpServer::reply(Session* s, const QString& line)
{
	if (!s->control) return;
	s->control->write((line + "\r\n").toUtf8());
	s->control->flush();
}
