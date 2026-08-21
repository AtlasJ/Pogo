#pragma once

#include <QObject>
#include <QThread>
#include <QTcpSocket>
#include <QElapsedTimer>
#include <QPolygonF>
#include <QDateTime>
#include <QImage>
#include <QVector>
#include <mutex>

class SRXFtpServer;

/*
* Manager for the two Keyence SR-X100 external barcode readers.
* Follows the CAMManager singleton pattern: fixed IDs ("SRX1"/"SRX2"),
* one owner for the reader connections (the readers serve a single
* client only, so nothing else may open sockets to them).
*
* All socket work runs on a dedicated worker thread. Public command
* methods are safe to call from any thread (queued internally); data
* getters are mutex-guarded snapshots.
*
* Two independent paths per the SR-X handover doc:
*  - Trigger/decode: raw TCP, "LON\r" to trigger, CR-terminated ASCII reply
*  - Image + code corners: the reader FTP-pushes the capture JPG and a
*    historical-data JSON into our embedded FTP server (SRXFtpServer)
*/

struct SRXReadResult {
	QString id;                   //SRX1 / SRX2
	QString code;                 //segmented barcode, or "ERROR"
	QString rawCode;              //payload as received
	bool ok = false;
	int readTimeMs = -1;          //reader-side read time from historical JSON
	QVector<QPolygonF> corners;   //one quad per code, in image pixel space
	int imageWidth = 0;
	int imageHeight = 0;
	QString imagePath;            //last pushed capture for this reader
	QDateTime timestamp;
};

class SRXManager : public QObject {
	Q_OBJECT

public:
	static const int READER_COUNT = 2;
	static const QString SRX1;
	static const QString SRX2;

	static SRXManager& instance();

	struct ReaderConfig {
		QString id;
		bool enabled = true;
		QString ip;
		int port = 9004;
	};

	struct FtpConfig {
		bool enabled = true;
		int port = 21;
		QString dropPath;
	};

	//lifecycle (call from GUI thread)
	void init();     //start worker thread, load config, connect readers, start FTP
	void release();

	//config
	QStringList ids() const;
	ReaderConfig readerConfig(const QString& id) const;
	FtpConfig ftpConfig() const;
	void setReaderConfig(const ReaderConfig& cfg);  //applies + reconnects that reader
	void setFtpConfig(const FtpConfig& cfg);        //applies + restarts the FTP server
	bool saveConfig();

	//status
	bool isConnected(const QString& id) const;
	bool isFtpRunning() const;

	//commands (queued to the worker thread, safe from any thread)
	void connectReader(const QString& id);
	void disconnectReader(const QString& id);
	void trigger(const QString& id);   //LON
	void triggerAll();
	void stopReader(const QString& id); //LOFF
	void stopAll();

	//data
	QString lastBarcode(const QString& id) const;
	SRXReadResult lastResult(const QString& id) const;
	QString lastImagePath(const QString& id) const;
	QImage lastImage(const QString& id) const;
	void clearResults();

	static int indexOf(const QString& id);           //SRX1 -> 0, SRX2 -> 1, else -1
	static QString idOf(int index);

signals:
	void connectionChanged(QString id, bool connected);
	void barcodeReceived(QString id, QString code);   //segmented decode, or "ERROR"
	void readResultReceived(QString id);              //lastResult(id) updated from historical JSON
	void imageReceived(QString id, QString imagePath);
	void ftpStateChanged(bool running);

private slots:
	//worker-thread internals
	void onInit();
	void doConnect(int index);
	void doDisconnect(int index);
	void doTrigger(int index);
	void doStop(int index);
	void doApplyFtp();
	void onSocketData(int index);
	void onFtpFile(QString peerIp, QString filePath);

private:
	SRXManager();
	~SRXManager();
	SRXManager(const SRXManager&) = delete;
	SRXManager& operator=(const SRXManager&) = delete;

	void loadConfig();
	bool writeCommand(int index, const QString& cmd, const char* label);
	void processMessage(int index, const QString& msg);
	void parseHistoricalJson(int index, const QString& path);
	static QString segmentBarcode(const QString& raw);

	QThread m_thread;
	bool m_initialized = false;

	mutable std::mutex m_mutex; //guards config, connected flags and results

	ReaderConfig m_config[READER_COUNT];
	FtpConfig m_ftpConfig;

	QTcpSocket* m_socket[READER_COUNT] = { nullptr, nullptr };
	bool m_connected[READER_COUNT] = { false, false };
	QByteArray m_buffer[READER_COUNT]; //per-reader receive buffer, messages framed on CR
	QElapsedTimer m_triggerElapsed;

	SRXReadResult m_result[READER_COUNT];

	SRXFtpServer* m_ftpServer = nullptr;
};
