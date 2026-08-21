#include "PaddleOcrClient.h"
#include "Logger.h"

#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>
#include <QElapsedTimer>
#include <QFileInfo>
#include <QDataStream>
#include <QCoreApplication>

static const quint32 MSG_TEXT = 1;
static const quint32 MSG_FRAME = 2;

PaddleOcrClient::PaddleOcrClient(QObject* parent)
	: QObject(parent)
{
}

PaddleOcrClient::~PaddleOcrClient()
{
	shutdown();
}

void PaddleOcrClient::configure(const QString& venvPath, const QString& scriptPath, int port)
{
	m_venvPath = venvPath;
	m_scriptPath = scriptPath;
	m_port = port;
}

bool PaddleOcrClient::ensureStarted(int timeoutMs)
{
	if (m_initialized && m_socket && m_socket->state() == QAbstractSocket::ConnectedState) return true;

	m_initialized = false;
	m_rxBuffer.clear();

	//1) spawn the python server if it is not running
	if (!m_process || m_process->state() == QProcess::NotRunning) {
		const QString python = m_venvPath + QStringLiteral("Scripts/python.exe");

		if (!QFileInfo::exists(python) || !QFileInfo::exists(m_scriptPath)) {
			ct::logger::error("[PaddleOCR] Python or script missing (%s | %s)",
				python.toStdString().c_str(), m_scriptPath.toStdString().c_str());
			return false;
		}

		if (!m_process) {
			m_process = new QProcess(this);
			m_process->setProcessChannelMode(QProcess::MergedChannels);
			connect(m_process, &QProcess::readyReadStandardOutput, this, [this]() {
				for (const auto& line : m_process->readAllStandardOutput().split('\n')) {
					const QString msg = QString::fromUtf8(line).trimmed();
					if (!msg.isEmpty()) ct::logger::info("[PaddleOCR] %s", msg.toStdString().c_str());
				}
			});
		}

		ct::logger::info("[PaddleOCR] Starting server: %s %s", python.toStdString().c_str(), m_scriptPath.toStdString().c_str());
		m_process->start(python, { m_scriptPath });

		if (!m_process->waitForStarted(10000)) {
			ct::logger::error("[PaddleOCR] Failed to start python: %s", m_process->errorString().toStdString().c_str());
			return false;
		}
	}

	//2) connect — the server binds after import (can take a while on first run)
	if (!m_socket) m_socket = new QTcpSocket(this);

	QElapsedTimer timer;
	timer.start();

	while (m_socket->state() != QAbstractSocket::ConnectedState) {
		if (timer.elapsed() > timeoutMs) {
			ct::logger::error("[PaddleOCR] Timed out connecting to 127.0.0.1:%d", m_port);
			return false;
		}

		m_socket->abort();
		m_socket->connectToHost(QStringLiteral("127.0.0.1"), m_port);
		if (m_socket->waitForConnected(2000)) break;
	}

	//3) wait for "Initialization Complete" (model load — slow on first connect)
	while (timer.elapsed() < timeoutMs) {
		quint32 type = 0;
		QByteArray payload;
		if (!readPacket(type, payload, timeoutMs - (int)timer.elapsed())) return false;

		if (type == MSG_TEXT) {
			const QString text = QString::fromUtf8(payload);
			ct::logger::info("[PaddleOCR] << %s", text.left(120).toStdString().c_str());
			if (text.startsWith(QStringLiteral("Initialization Complete"))) {
				m_initialized = true;
				ct::logger::info("[PaddleOCR] Ready (%.1fs)", timer.elapsed() / 1000.0);
				return true;
			}
		}
	}

	ct::logger::error("[PaddleOCR] Timed out waiting for initialization");
	return false;
}

bool PaddleOcrClient::runOcr(const cv::Mat& imageBgr, QVector<AlgoOcrBox>& results, int timeoutMs)
{
	results.clear();

	if (imageBgr.empty()) return false;
	if (!ensureStarted()) return false;

	//encode as PNG (lossless — OCR quality over bandwidth on loopback)
	std::vector<uchar> encoded;
	if (!cv::imencode(".png", imageBgr, encoded)) {
		ct::logger::error("[PaddleOCR] imencode failed");
		return false;
	}

	if (!sendPacket(MSG_FRAME, QByteArray(reinterpret_cast<const char*>(encoded.data()), (int)encoded.size()))) {
		m_initialized = false; //force re-handshake next time
		return false;
	}

	QElapsedTimer timer;
	timer.start();
	bool gotResult = false;

	while (timer.elapsed() < timeoutMs) {
		quint32 type = 0;
		QByteArray payload;
		if (!readPacket(type, payload, timeoutMs - (int)timer.elapsed())) {
			m_initialized = false;
			return false;
		}

		if (type != MSG_TEXT) continue;
		const QString text = QString::fromUtf8(payload);

		if (text.startsWith(QStringLiteral("OCR_RESULT:"))) {
			results = parseResults(payload.mid(11));
			gotResult = true;
		}
		else if (text.startsWith(QStringLiteral("OCR_ERROR:"))) {
			ct::logger::error("[PaddleOCR] %s", text.toStdString().c_str());
		}
		else if (text.startsWith(QStringLiteral("Prediction Complete"))) {
			return gotResult;
		}
	}

	ct::logger::error("[PaddleOCR] runOcr timed out after %dms", (int)timer.elapsed());
	m_initialized = false;
	return false;
}

void PaddleOcrClient::shutdown()
{
	if (m_socket) {
		m_socket->abort();
	}
	if (m_process && m_process->state() != QProcess::NotRunning) {
		m_process->kill();
		m_process->waitForFinished(3000);
	}
	m_initialized = false;
}

bool PaddleOcrClient::sendPacket(quint32 msgType, const QByteArray& payload)
{
	if (!m_socket || m_socket->state() != QAbstractSocket::ConnectedState) return false;

	QByteArray header(8, 0);
	quint32 size = (quint32)payload.size();
	memcpy(header.data(), &msgType, 4);      //little-endian x64
	memcpy(header.data() + 4, &size, 4);

	m_socket->write(header);
	m_socket->write(payload);
	return m_socket->waitForBytesWritten(10000);
}

bool PaddleOcrClient::readPacket(quint32& msgType, QByteArray& payload, int timeoutMs)
{
	QElapsedTimer timer;
	timer.start();

	auto waitFor = [&](int needed) -> bool {
		while (m_rxBuffer.size() < needed) {
			int remaining = timeoutMs - (int)timer.elapsed();
			if (remaining <= 0) return false;
			if (!m_socket->waitForReadyRead(qMin(remaining, 1000))) {
				if (m_socket->state() != QAbstractSocket::ConnectedState) return false;
				continue;
			}
			m_rxBuffer.append(m_socket->readAll());
		}
		return true;
	};

	if (!waitFor(8)) return false;

	quint32 size = 0;
	memcpy(&msgType, m_rxBuffer.constData(), 4);
	memcpy(&size, m_rxBuffer.constData() + 4, 4);

	if (size > 64u * 1024u * 1024u) { //corrupt stream guard
		ct::logger::error("[PaddleOCR] Bad packet size %u, resetting", size);
		m_rxBuffer.clear();
		return false;
	}

	if (!waitFor(8 + (int)size)) return false;

	payload = m_rxBuffer.mid(8, size);
	m_rxBuffer.remove(0, 8 + size);
	return true;
}

QVector<AlgoOcrBox> PaddleOcrClient::parseResults(const QByteArray& json)
{
	QVector<AlgoOcrBox> out;

	const auto doc = QJsonDocument::fromJson(json);
	if (!doc.isArray()) return out;

	for (const auto& v : doc.array()) {
		const auto obj = v.toObject();
		AlgoOcrBox r;
		r.text = obj.value(QStringLiteral("text")).toString();
		r.score = (float)obj.value(QStringLiteral("score")).toDouble();

		for (const auto& ptVal : obj.value(QStringLiteral("box")).toArray()) {
			const auto pt = ptVal.toArray();
			if (pt.size() >= 2) r.box.append(QPoint((int)pt[0].toDouble(), (int)pt[1].toDouble()));
		}

		out.append(r);
	}

	return out;
}
