#include "LSC_VLP.h"
#include <sstream>
#include <algorithm>
#include "QOSTool.h"
#include "TimeLogger.h"
#include "Logger.h"
#include "QSocketWorker.h"

std::string LSC_VLP::getCheckSum(std::string msg)
{
	int ascii = 0;

	for (auto chr : msg)
	{
		ascii += int(chr);
	}

	std::ostringstream ss;
	ss << std::hex << ascii; 
	std::string result = ss.str(); //+: convert int to hex
	if (result.length() == 3) result.erase(0, 1); //+: remove 1st string
	std::transform(result.begin(), result.end(), result.begin(), ::toupper); //+: convert string to upper case

	return result;
}


std::string LSC_VLP::getChannelStr(int ch)
{
	if (ch == 0) return "00";
	else if (ch == 1) return "01";
	else if (ch == 2) return "02";
	else if (ch == 3) return "03";
	return "00";
}

std::string LSC_VLP::getIntensityStr(int intensity)
{
	std::string s = std::to_string(intensity);
	while (s.length() < 3) {
		s.insert(0, "0"); //+: insert in front of string
	}
	return s;
}

LSC_VLP::LSC_VLP()
{
}

LSC_VLP::~LSC_VLP()
{
	ct::logger::info("~LSC_VLP");
}

const std::string & LSC_VLP::id() const
{
	return m_id;
}

std::string & LSC_VLP::id()
{
	return m_id;
}

const std::string & LSC_VLP::name() const
{
	return m_name;
}

std::string & LSC_VLP::name()
{
	return m_name;
}

int LSC_VLP::enable(bool toggle)
{
	m_enable = toggle;
	return (int)LSC_RC::PASS;
}

int LSC_VLP::connect()
{
	if (!m_enable) return (int)LSC_RC::PASS;

	//if (m_socket != nullptr) {
	//	delete m_socket;
	//	m_socket = nullptr;
	//}

	//m_socket = new QTcpSocket(); // create a new socket

	QThread* netThread = new QThread;
	m_socketWorker = new QSocketWorker;
	m_socketWorker->moveToThread(netThread);
	QObject::connect(netThread, &QThread::started, m_socketWorker, &QSocketWorker::init);
	QObject::connect(netThread, &QThread::finished, m_socketWorker, &QObject::deleteLater);
	netThread->start();

	reconnect();
	return (int)LSC_RC::PASS;
}

int LSC_VLP::disconnect()
{
	if (!m_enable) return (int)LSC_RC::PASS;

	ct::logger::info("LSC disconnecting...");
	if (!isConnected()) {
		ct::logger::info("LSC disconnected!");
		return (int)LSC_RC::PASS;
	}

	//m_socket->disconnectFromHost();
	//m_socket->waitForDisconnected(m_connectionTimeout);
	QMetaObject::invokeMethod(m_socketWorker, "disconnectFromHost", Qt::QueuedConnection);

	if (!isConnected()) {
		ct::logger::info("LSC disconnected!");
		return (int)LSC_RC::PASS;
	}
	else return (int)LSC_RC::INVALID_CONNECTION;
}

int LSC_VLP::reconnect()
{
	if (!m_enable) return (int)LSC_RC::PASS;

	QMetaObject::invokeMethod(m_socketWorker, "connectToHost", Qt::QueuedConnection, Q_ARG(QString, m_ip.c_str()), Q_ARG(quint16, m_port));
	/*m_socket->connectToHost(m_ip.c_str(), m_port);
	m_socket->waitForConnected(m_connectionTimeout);*/

	if (isConnected()) return (int)LSC_RC::PASS;
	else return (int)LSC_RC::INVALID_CONNECTION;
}

bool LSC_VLP::isConnected() const
{
	if (!m_enable) return true;
	//return (m_socket->state() == QAbstractSocket::ConnectedState);
	return true;
}

void LSC_VLP::setConnectionTimeOut(int ms)
{
	m_connectionTimeout = ms;
}

void LSC_VLP::setResponseTimeOut(int ms)
{
	m_responseTimeout = ms;
}

int LSC_VLP::numChannel() const
{
	return m_numChannel;
}

int & LSC_VLP::numChannel()
{
	return m_numChannel;
}

int LSC_VLP::toggle(int ch, bool on)
{
	if (!m_enable) return (int)LSC_RC::PASS;

	if (!isConnected()) return (int)LSC_RC::INVALID_CONNECTION;
	if (ch >= numChannel()) return (int)LSC_RC::INVALID_CHANNEL;

	if (on) {
		writeToLSC(ch, m_intensity[ch]);
	}
	else {
		writeToLSC(ch, -1);
	}

	return (int)LSC_RC::PASS;
}

int LSC_VLP::setIntensity(int ch, int intensity)
{
	ct::logger::debug("VLP set: %d", intensity);
	if (!m_enable) return (int)LSC_RC::PASS;

	if (ch >= numChannel()) {
		ct::logger::warn("Failed to set intensity: Invalid channel");
		return (int)LSC_RC::INVALID_CHANNEL;
	}

#if !HAS_LSC
	return (int)LSC_RC::PASS;
#endif

	m_intensity[ch] = intensity;

	return (int)LSC_RC::PASS;
}

int LSC_VLP::setMultiIntensity(lsc::IntensityData* idata, int size)
{
	return 0;
}

int LSC_VLP::getIntensity(int ch, int & intensity)
{
	return (int)LSC_RC::PASS;
}

int LSC_VLP::setIP(std::string ip)
{
	if (!m_enable) return (int)LSC_RC::PASS;

	m_ip = ip;
	return (int)LSC_RC::PASS;
}

int LSC_VLP::setPort(int port)
{
	if (!m_enable) return (int)LSC_RC::PASS;

	m_port = port;
	return (int)LSC_RC::PASS;
}

int LSC_VLP::setMode(lsc::MODE mode)
{
	return (int)LSC_RC::PASS;
}

int LSC_VLP::setTriggerDuration(int ch, int us)
{
	return (int)LSC_RC::PASS;
}

int LSC_VLP::setTriggerSequence(const std::vector<lsc::SequenceData>& datas)
{
	return (int)LSC_RC::PASS;
}

std::string LSC_VLP::codeString(int returnCode)
{
	return std::string();
}

void LSC_VLP::writeToLSC(int ch, int intensity)
{
	if (!isConnected()) {
		reconnect();
		if (!isConnected()) {
			ct::logger::warn("Failed to set intensity: Connection failure");
			return;
		}
	}
	if (ch >= numChannel()) {
		ct::logger::warn("Failed to set intensity: Invalid channel");
		return;
	}

	int real_intensity = intensity;
	if (real_intensity < 0) real_intensity = 0;

	std::string msg = "@";
	msg += getChannelStr(ch);
	msg += "F";
	msg += getIntensityStr(real_intensity);
	msg += getCheckSum(msg);
	msg += "\r\n";

	QMetaObject::invokeMethod(m_socketWorker, "writeData", Qt::BlockingQueuedConnection, Q_ARG(QByteArray, QByteArray(msg.c_str())));

	//m_socket->write(msg.c_str()); // send the message to the server
	//m_socket->flush();

	//m_socket->waitForBytesWritten(m_responseTimeout);
	//if (m_socket->waitForReadyRead(m_responseTimeout)) { // wait for the server to respond
	//	QString response = QString::fromUtf8(m_socket->readAll()); // read the response
	//	//qDebug() << "LSC Server response:" << response;
	//	if (intensity != -1) m_lastIntensity[ch] = real_intensity;
	//}
	//else {
	//	ct::logger::warn("LSC response timeout");
	//}

	os_tool::doNothing(15); //WARNING: This is needed as if there's no delay, the lighting will not be accurate

	//disconnect();
}

int LSC_VLP::setMaxCurrent(int ch, double dCurrent)
{
	return (int)LSC_RC::PASS;
}