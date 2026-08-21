#pragma once

#include <QObject>
#include <QProcess>
#include <QTcpSocket>
#include <QVector>
#include <opencv2/opencv.hpp>

#include "AlgoSetupTypes.h"

/*
* Native client for the PaddleOCR python server (C:/Advanced/Scripts/pyPaddleAPI.py).
* Replaces IM430's NvsOcr.lib, whose import library is not available here — the
* protocol is simple enough to speak directly:
*
*   packet = 8-byte little-endian header (uint32 msgType, uint32 payloadSize) + payload
*   msgType 1 = TEXT (utf-8), msgType 2 = FRAME (encoded image bytes)
*   server replies with TEXT packets: "Initialization Complete",
*   "OCR_RESULT:<json>" then "Prediction Complete" per frame,
*   or "OCR_ERROR:<msg>" then "Prediction Complete".
*
* The server initialises PaddleOCR only after a client connects, which takes
* many seconds (model load onto GPU) — ensureStarted() handles the whole
* spawn/connect/init handshake and can be called repeatedly.
*
* All calls must come from the owning thread (sockets have thread affinity);
* AlgoManager owns one instance on its worker thread.
*/
class PaddleOcrClient : public QObject {
	Q_OBJECT

public:
	explicit PaddleOcrClient(QObject* parent = nullptr);
	~PaddleOcrClient();

	void configure(const QString& venvPath, const QString& scriptPath, int port = 8888);

	//spawns the python server (if not running) and completes the init handshake.
	//Safe to call repeatedly; returns false if the server cannot be reached.
	bool ensureStarted(int timeoutMs = 120000);

	//blocking OCR round trip. Returns false on transport error/timeout;
	//an empty result list with true is a valid "nothing read".
	bool runOcr(const cv::Mat& imageBgr, QVector<AlgoOcrBox>& results, int timeoutMs = 30000);

	bool isReady() const { return m_initialized; }
	void shutdown();

private:
	bool sendPacket(quint32 msgType, const QByteArray& payload);
	bool readPacket(quint32& msgType, QByteArray& payload, int timeoutMs);
	static QVector<AlgoOcrBox> parseResults(const QByteArray& json);

	QString m_venvPath = QStringLiteral("C:/Advanced/Scripts/VirtualEnv/");
	QString m_scriptPath = QStringLiteral("C:/Advanced/Scripts/pyPaddleAPI.py");
	int m_port = 8888;

	QProcess* m_process = nullptr;
	QTcpSocket* m_socket = nullptr;
	bool m_initialized = false;
	QByteArray m_rxBuffer;
};
