#include "SRXManager.h"
#include "SRXFtpServer.h"
#include "Logger.h"
#include "QJsonHelper.h"

#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QFileInfo>
#include <QDir>
#include <QTimer>
#include <QTextCodec>
#include <QMetaObject>

const QString SRXManager::SRX1 = QStringLiteral("SRX1");
const QString SRXManager::SRX2 = QStringLiteral("SRX2");

static const QString SRX_CONFIG_PATH = QStringLiteral("C:/Advanced/Data/config/barcodeReader.json");
static const QString SRX_DEFAULT_DROP = QStringLiteral("C:/Advanced/Data/barcode_ftp");

// The readers append a trailing field whose ":xx" suffix is not part of the
// barcode, e.g.
//   "KEGE0248BA06JJA 2615-B3R1 25:01"  ->  "KEGE0248BA06JJA 2615-B3R1 25"
// Keep the leading fields only, and cut the appended field at ':'. The full raw
// payload is logged before this runs, so nothing is lost for troubleshooting.
static const int SRX_BARCODE_FIELDS = 3;

QString SRXManager::segmentBarcode(const QString& raw)
{
	QStringList parts = raw.split(' ', QString::SkipEmptyParts);

	if (parts.size() > SRX_BARCODE_FIELDS) parts = parts.mid(0, SRX_BARCODE_FIELDS);

	// Only the appended field is trimmed; the leading data fields are kept verbatim.
	if (parts.size() == SRX_BARCODE_FIELDS) {
		QString& tail = parts.last();
		const int colon = tail.indexOf(':');
		if (colon >= 0) tail = tail.left(colon);
		if (tail.isEmpty()) parts.removeLast();
	}

	return parts.join(' ');
}

static const char* srxStateName(QAbstractSocket::SocketState state)
{
	switch (state) {
	case QAbstractSocket::UnconnectedState: return "Unconnected";
	case QAbstractSocket::HostLookupState:  return "HostLookup";
	case QAbstractSocket::ConnectingState:  return "Connecting";
	case QAbstractSocket::ConnectedState:   return "Connected";
	case QAbstractSocket::BoundState:       return "Bound";
	case QAbstractSocket::ClosingState:     return "Closing";
	case QAbstractSocket::ListeningState:   return "Listening";
	default:                                return "Unknown";
	}
}

//recursively search a JSON tree for the first value under the given key
static QJsonValue findJsonKey(const QJsonValue& node, const QString& key)
{
	if (node.isObject()) {
		auto obj = node.toObject();
		if (obj.contains(key)) return obj.value(key);

		for (auto it = obj.begin(); it != obj.end(); ++it) {
			auto found = findJsonKey(it.value(), key);
			if (!found.isUndefined()) return found;
		}
	}
	else if (node.isArray()) {
		for (const auto& item : node.toArray()) {
			auto found = findJsonKey(item, key);
			if (!found.isUndefined()) return found;
		}
	}

	return QJsonValue(QJsonValue::Undefined);
}

SRXManager& SRXManager::instance()
{
	static SRXManager inst;
	return inst;
}

SRXManager::SRXManager()
{
	for (int i = 0; i < READER_COUNT; i++) {
		m_config[i].id = idOf(i);
		m_result[i].id = idOf(i);
	}
	m_ftpConfig.dropPath = SRX_DEFAULT_DROP;
}

SRXManager::~SRXManager()
{
	release();
}

int SRXManager::indexOf(const QString& id)
{
	if (id == SRX1) return 0;
	if (id == SRX2) return 1;
	return -1;
}

QString SRXManager::idOf(int index)
{
	if (index == 0) return SRX1;
	if (index == 1) return SRX2;
	return QString();
}

QStringList SRXManager::ids() const
{
	return { SRX1, SRX2 };
}

void SRXManager::init()
{
	if (m_initialized) return;
	m_initialized = true;

	loadConfig();

	moveToThread(&m_thread);
	m_thread.start();

	QMetaObject::invokeMethod(this, "onInit", Qt::QueuedConnection);
}

void SRXManager::release()
{
	if (!m_thread.isRunning()) return;
	m_thread.quit();
	m_thread.wait(3000);
}

void SRXManager::onInit()
{
	ct::logger::info("[SRX] Manager started on worker thread");

	for (int i = 0; i < READER_COUNT; i++) {
		m_socket[i] = new QTcpSocket(this);

		connect(m_socket[i], &QTcpSocket::readyRead, this, [=]() { onSocketData(i); });
		connect(m_socket[i], &QTcpSocket::disconnected, this, [=]() {
			ct::logger::error("[SRX] %s disconnected (was %s:%d)",
				idOf(i).toStdString().c_str(), m_config[i].ip.toStdString().c_str(), m_config[i].port);
			{
				std::lock_guard<std::mutex> lock(m_mutex);
				m_connected[i] = false;
			}
			emit connectionChanged(idOf(i), false);
		});
		connect(m_socket[i], &QTcpSocket::stateChanged, this, [=](QAbstractSocket::SocketState state) {
			ct::logger::debug("[SRX] %s state -> %s", idOf(i).toStdString().c_str(), srxStateName(state));
		});
	}

	m_ftpServer = new SRXFtpServer(this);
	connect(m_ftpServer, &SRXFtpServer::fileReceived, this, &SRXManager::onFtpFile);

	doApplyFtp();

	for (int i = 0; i < READER_COUNT; i++) {
		if (m_config[i].enabled) doConnect(i);
	}
}

//---------------------------------------------------------------- config

void SRXManager::loadConfig()
{
	QJsonObject root;
	if (!jsonHelper::loadJson(SRX_CONFIG_PATH, root)) {
		ct::logger::warn("[SRX] %s not found, using defaults (readers unconfigured)",
			SRX_CONFIG_PATH.toStdString().c_str());
		return;
	}

	std::lock_guard<std::mutex> lock(m_mutex);

	if (root.contains("readers")) {
		auto readers = root["readers"].toArray();
		for (const auto& r : readers) {
			auto obj = r.toObject();
			int index = indexOf(jsonHelper::getString(obj, "id"));
			if (index < 0) continue;

			m_config[index].enabled = jsonHelper::getBool(obj, "enabled", true);
			m_config[index].ip = jsonHelper::getString(obj, "ip");
			m_config[index].port = jsonHelper::getInteger(obj, "port", 9004);
		}
	}
	else {
		//legacy schema: single "IP"/"Port", reader 2 derived as IP+1 on the same port
		auto ip = jsonHelper::getString(root, "IP");
		auto port = jsonHelper::getInteger(root, "Port", 9004);

		m_config[0].ip = ip;
		m_config[0].port = port;

		QStringList octets = ip.split('.');
		bool ok = false;
		int last = (octets.size() == 4) ? octets[3].toInt(&ok) : -1;
		if (ok && last >= 0 && last + 1 <= 254) {
			octets[3] = QString::number(last + 1);
			m_config[1].ip = octets.join('.');
			m_config[1].port = port;
		}

		ct::logger::info("[SRX] Migrated legacy config: %s -> SRX1 %s:%d, SRX2 %s:%d",
			SRX_CONFIG_PATH.toStdString().c_str(),
			m_config[0].ip.toStdString().c_str(), m_config[0].port,
			m_config[1].ip.toStdString().c_str(), m_config[1].port);
	}

	if (root.contains("ftp")) {
		auto ftp = root["ftp"].toObject();
		m_ftpConfig.enabled = jsonHelper::getBool(ftp, "enabled", true);
		m_ftpConfig.port = jsonHelper::getInteger(ftp, "port", 21);
		m_ftpConfig.dropPath = jsonHelper::getString(ftp, "drop_path");
		if (m_ftpConfig.dropPath.isEmpty()) m_ftpConfig.dropPath = SRX_DEFAULT_DROP;
	}

	for (int i = 0; i < READER_COUNT; i++) {
		ct::logger::info("[SRX] Config %s: %s:%d enabled=%d",
			idOf(i).toStdString().c_str(), m_config[i].ip.toStdString().c_str(),
			m_config[i].port, m_config[i].enabled ? 1 : 0);
	}
}

bool SRXManager::saveConfig()
{
	QJsonObject root;
	QJsonArray readers;

	std::unique_lock<std::mutex> lock(m_mutex);

	for (int i = 0; i < READER_COUNT; i++) {
		QJsonObject obj;
		obj.insert("id", m_config[i].id);
		obj.insert("enabled", m_config[i].enabled);
		obj.insert("ip", m_config[i].ip);
		obj.insert("port", m_config[i].port);
		readers.append(obj);
	}

	QJsonObject ftp;
	ftp.insert("enabled", m_ftpConfig.enabled);
	ftp.insert("port", m_ftpConfig.port);
	ftp.insert("drop_path", m_ftpConfig.dropPath);

	//keep legacy keys so older builds can still read reader 1
	root.insert("IP", m_config[0].ip);
	root.insert("Port", m_config[0].port);
	root.insert("readers", readers);
	root.insert("ftp", ftp);

	lock.unlock();

	auto ret = jsonHelper::saveJson(SRX_CONFIG_PATH, QJsonDocument(root));
	if (ret) ct::logger::info("[SRX] Config saved: %s", SRX_CONFIG_PATH.toStdString().c_str());
	else ct::logger::error("[SRX] Failed to save config: %s", SRX_CONFIG_PATH.toStdString().c_str());

	return ret;
}

SRXManager::ReaderConfig SRXManager::readerConfig(const QString& id) const
{
	std::lock_guard<std::mutex> lock(m_mutex);
	int index = indexOf(id);
	return (index >= 0) ? m_config[index] : ReaderConfig();
}

SRXManager::FtpConfig SRXManager::ftpConfig() const
{
	std::lock_guard<std::mutex> lock(m_mutex);
	return m_ftpConfig;
}

void SRXManager::setReaderConfig(const ReaderConfig& cfg)
{
	int index = indexOf(cfg.id);
	if (index < 0) return;

	ReaderConfig fixed = cfg;
	if (fixed.port <= 0) fixed.port = 9004; //SR-X non-procedural command port

	{
		std::lock_guard<std::mutex> lock(m_mutex);
		m_config[index] = fixed;
	}

	saveConfig(); //persist page edits across restarts

	if (fixed.enabled) connectReader(fixed.id);
	else disconnectReader(fixed.id);
}

void SRXManager::setFtpConfig(const FtpConfig& cfg)
{
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		m_ftpConfig = cfg;
		if (m_ftpConfig.dropPath.isEmpty()) m_ftpConfig.dropPath = SRX_DEFAULT_DROP;
	}

	saveConfig(); //persist page edits across restarts

	QMetaObject::invokeMethod(this, "doApplyFtp", Qt::QueuedConnection);
}

//---------------------------------------------------------------- status

bool SRXManager::isConnected(const QString& id) const
{
	std::lock_guard<std::mutex> lock(m_mutex);
	int index = indexOf(id);
	return (index >= 0) ? m_connected[index] : false;
}

bool SRXManager::isFtpRunning() const
{
	return m_ftpServer && m_ftpServer->isRunning();
}

//---------------------------------------------------------------- commands

void SRXManager::connectReader(const QString& id)
{
	int index = indexOf(id);
	if (index < 0) return;
	QMetaObject::invokeMethod(this, "doConnect", Qt::QueuedConnection, Q_ARG(int, index));
}

void SRXManager::disconnectReader(const QString& id)
{
	int index = indexOf(id);
	if (index < 0) return;
	QMetaObject::invokeMethod(this, "doDisconnect", Qt::QueuedConnection, Q_ARG(int, index));
}

void SRXManager::trigger(const QString& id)
{
	int index = indexOf(id);
	if (index < 0) return;
	QMetaObject::invokeMethod(this, "doTrigger", Qt::QueuedConnection, Q_ARG(int, index));
}

void SRXManager::setLiveRead(const QString& id, bool on)
{
	int index = indexOf(id);
	if (index < 0) return;

	m_liveRead[index] = on;
	ct::logger::info("[SRX] %s live read %s", id.toStdString().c_str(), on ? "ON" : "OFF");

	if (on) trigger(id);
	else stopReader(id);
}

void SRXManager::triggerAll()
{
	for (int i = 0; i < READER_COUNT; i++) {
		QMetaObject::invokeMethod(this, "doTrigger", Qt::QueuedConnection, Q_ARG(int, i));
	}
}

void SRXManager::stopReader(const QString& id)
{
	int index = indexOf(id);
	if (index < 0) return;
	QMetaObject::invokeMethod(this, "doStop", Qt::QueuedConnection, Q_ARG(int, index));
}

void SRXManager::stopAll()
{
	for (int i = 0; i < READER_COUNT; i++) {
		QMetaObject::invokeMethod(this, "doStop", Qt::QueuedConnection, Q_ARG(int, i));
	}
}

//---------------------------------------------------------------- worker-thread slots

void SRXManager::doConnect(int index)
{
	if (index < 0 || index >= READER_COUNT || !m_socket[index]) return;

	ReaderConfig cfg;
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		cfg = m_config[index];
		m_connected[index] = false;
	}

	if (!cfg.enabled) return;

	if (cfg.ip.isEmpty() || cfg.port <= 0) {
		ct::logger::warn("[SRX] %s endpoint not configured (%s:%d), skipping connect",
			cfg.id.toStdString().c_str(),
			cfg.ip.isEmpty() ? "<unset>" : cfg.ip.toStdString().c_str(), cfg.port);
		return;
	}

	if (m_socket[index]->state() != QAbstractSocket::UnconnectedState) {
		m_socket[index]->abort();
	}

	ct::logger::info("[SRX] %s connecting to %s:%d",
		cfg.id.toStdString().c_str(), cfg.ip.toStdString().c_str(), cfg.port);

	m_socket[index]->connectToHost(QHostAddress(cfg.ip), cfg.port);

	if (!m_socket[index]->waitForConnected(3000)) {
		ct::logger::error("[SRX] %s connect FAILED (%s:%d) state=%s err=%s",
			cfg.id.toStdString().c_str(), cfg.ip.toStdString().c_str(), cfg.port,
			srxStateName(m_socket[index]->state()),
			m_socket[index]->errorString().toStdString().c_str());
		emit connectionChanged(cfg.id, false);
	}
	else {
		ct::logger::info("[SRX] %s connected (%s:%d)",
			cfg.id.toStdString().c_str(), cfg.ip.toStdString().c_str(), cfg.port);
		{
			std::lock_guard<std::mutex> lock(m_mutex);
			m_connected[index] = true;
		}
		emit connectionChanged(cfg.id, true);
	}
}

void SRXManager::doDisconnect(int index)
{
	if (index < 0 || index >= READER_COUNT || !m_socket[index]) return;

	if (m_socket[index]->state() != QAbstractSocket::UnconnectedState) {
		m_socket[index]->disconnectFromHost();
	}

	std::lock_guard<std::mutex> lock(m_mutex);
	m_connected[index] = false;
}

bool SRXManager::writeCommand(int index, const QString& cmd, const char* label)
{
	if (!m_socket[index]) {
		ct::logger::warn("[SRX] %s -> %s skipped: no socket", label, idOf(index).toStdString().c_str());
		return false;
	}

	const auto state = m_socket[index]->state();
	if (state != QTcpSocket::ConnectedState) {
		ct::logger::error("[SRX] %s -> %s skipped: not connected (state=%s, %s:%d)",
			label, idOf(index).toStdString().c_str(), srxStateName(state),
			m_config[index].ip.toStdString().c_str(), m_config[index].port);
		return false;
	}

	const QByteArray payload = cmd.toUtf8();
	const qint64 written = m_socket[index]->write(payload);
	const bool flushed = m_socket[index]->flush();

	if (written != payload.size() || !flushed) {
		ct::logger::error("[SRX] %s -> %s FAILED: wrote %d/%d bytes, flush=%d, err=%s",
			label, idOf(index).toStdString().c_str(), (int)written, (int)payload.size(), flushed ? 1 : 0,
			m_socket[index]->errorString().toStdString().c_str());
		return false;
	}

	ct::logger::info("[SRX] %s -> %s OK (%d bytes)", label, idOf(index).toStdString().c_str(), (int)written);
	return true;
}

void SRXManager::doTrigger(int index)
{
	if (index < 0 || index >= READER_COUNT) return;

	if (!m_buffer[index].isEmpty()) {
		ct::logger::warn("[SRX] %s discarding %d stale buffered byte(s) before trigger: %s",
			idOf(index).toStdString().c_str(), (int)m_buffer[index].size(),
			QString::fromUtf8(m_buffer[index]).trimmed().toStdString().c_str());
		m_buffer[index].clear();
	}

	//auto-reconnect: a reader power cycle drops the socket silently
	if (m_socket[index] && m_socket[index]->state() != QTcpSocket::ConnectedState && m_config[index].enabled) {
		doConnect(index);
	}

	if (writeCommand(index, "LON\r", "Trigger LON")) {
		//reply latency is measured from here; see the "rx ... after trigger" line on receive
		m_triggerElapsed.start();

		//Live read: the reader-side read timeout can be very long (or unlimited),
		//in which case a no-decode cycle never ends on its own - no result, no
		//image, no retrigger. Cap the cycle: if no reply arrives in time, force
		//it closed with LOFF; the reader then reports ERROR and pushes its image,
		//and the ERROR reply schedules the next live trigger.
		if (m_liveRead[index]) {
			constexpr int liveCycleTimeoutMs = 100;
			const int seq = m_cycleSeq[index].load();
			QTimer::singleShot(liveCycleTimeoutMs, this, [this, index, seq]() {
				if (m_liveRead[index] && m_cycleSeq[index].load() == seq) {
					ct::logger::info("[SRX] %s live read: no decode within cycle window, closing cycle",
						idOf(index).toStdString().c_str());
					writeCommand(index, "LOFF\r", "Live cycle LOFF");
				}
			});
		}
	}
}

void SRXManager::doStop(int index)
{
	if (index < 0 || index >= READER_COUNT) return;

	m_liveRead[index] = false; //stop always ends live read

	writeCommand(index, "LOFF\r", "Stop LOFF");

	if (!m_buffer[index].isEmpty()) {
		ct::logger::warn("[SRX] %s had %d unparsed byte(s) at stop: %s",
			idOf(index).toStdString().c_str(), (int)m_buffer[index].size(),
			QString::fromUtf8(m_buffer[index]).trimmed().toStdString().c_str());
	}
}

void SRXManager::doApplyFtp()
{
	if (!m_ftpServer) return;

	FtpConfig cfg;
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		cfg = m_ftpConfig;
	}

	m_ftpServer->stop();

	bool running = false;
	if (cfg.enabled) {
		running = m_ftpServer->start(cfg.port, cfg.dropPath);
	}
	else {
		ct::logger::info("[SRX FTP] Disabled by config");
	}

	emit ftpStateChanged(running);
}

//---------------------------------------------------------------- decode path

void SRXManager::onSocketData(int index)
{
	if (index < 0 || index >= READER_COUNT || !m_socket[index]) return;

	const QByteArray byteArray = m_socket[index]->readAll();
	const qint64 sinceTrigger = m_triggerElapsed.isValid() ? m_triggerElapsed.elapsed() : -1;

	ct::logger::info("[SRX] %s rx %d byte(s) at %dms after trigger: %s",
		idOf(index).toStdString().c_str(), (int)byteArray.size(), (int)sinceTrigger,
		QString::fromUtf8(byteArray).trimmed().toStdString().c_str());

	if (sinceTrigger < 0) {
		ct::logger::warn("[SRX] %s sent data with no trigger outstanding (unsolicited/late)",
			idOf(index).toStdString().c_str());
	}

	auto& buffer = m_buffer[index];
	buffer.append(byteArray);

	//process only complete CR-terminated messages; keep the unterminated tail buffered
	int pos;
	while ((pos = buffer.indexOf('\r')) >= 0) {
		QString msg = QString::fromUtf8(buffer.left(pos)).remove('\n').trimmed();
		buffer.remove(0, pos + 1);

		if (msg.isEmpty()) continue;
		if (msg == "LON" || msg == "LOFF") continue; //command echoes

		processMessage(index, msg);
	}

	if (!buffer.isEmpty()) {
		ct::logger::debug("[SRX] %s partial message buffered (%d bytes)",
			idOf(index).toStdString().c_str(), (int)buffer.size());
	}
}

void SRXManager::processMessage(int index, const QString& msg)
{
	const QString id = idOf(index);

	m_cycleSeq[index]++; //a reply arrived - the live cycle watchdog for this trigger stands down

	if (msg.contains("ERROR")) {
		ct::logger::error("[SRX] %s reported ERROR", id.toStdString().c_str());
		{
			std::lock_guard<std::mutex> lock(m_mutex);
			m_result[index].code = "ERROR";
			m_result[index].rawCode = msg;
			m_result[index].ok = false;
			m_result[index].timestamp = QDateTime::currentDateTime();
		}
		emit barcodeReceived(id, "ERROR");
		scheduleLiveRetrigger(index);
		return;
	}

	const QString barcode = segmentBarcode(msg);

	if (barcode != msg) {
		ct::logger::info("[SRX] %s code: %s [segmented from raw '%s']",
			id.toStdString().c_str(), barcode.toStdString().c_str(), msg.toStdString().c_str());
	}
	else {
		ct::logger::info("[SRX] %s code: %s", id.toStdString().c_str(), barcode.toStdString().c_str());
	}

	{
		std::lock_guard<std::mutex> lock(m_mutex);
		m_result[index].code = barcode;
		m_result[index].rawCode = msg;
		m_result[index].ok = true;
		m_result[index].timestamp = QDateTime::currentDateTime();
	}

	emit barcodeReceived(id, barcode);
	scheduleLiveRetrigger(index);
}

void SRXManager::scheduleLiveRetrigger(int index)
{
	if (!m_liveRead[index]) return;

	//short pause between cycles so the reader can finish its output (image
	//FTP push) before the next LON
	QTimer::singleShot(300, this, [this, index]() {
		if (m_liveRead[index]) doTrigger(index);
	});
}

//---------------------------------------------------------------- image path

void SRXManager::onFtpFile(QString peerIp, QString filePath)
{
	//attribute the upload to a reader by its source address
	int index = -1;
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		for (int i = 0; i < READER_COUNT; i++) {
			if (!m_config[i].ip.isEmpty() && peerIp.endsWith(m_config[i].ip)) { //peer may be "::ffff:x.x.x.x"
				index = i;
				break;
			}
		}
	}

	if (index < 0) {
		ct::logger::warn("[SRX FTP] File from unknown peer %s ignored: %s",
			peerIp.toStdString().c_str(), filePath.toStdString().c_str());
		return;
	}

	const QString id = idOf(index);
	const QString suffix = QFileInfo(filePath).suffix().toLower();

	if (suffix == "jpg" || suffix == "jpeg" || suffix == "png" || suffix == "bmp") {
		QImage img(filePath);
		if (img.isNull()) {
			ct::logger::warn("[SRX] %s pushed unreadable image: %s", id.toStdString().c_str(), filePath.toStdString().c_str());
		}
		else {
			{
				std::lock_guard<std::mutex> lock(m_mutex);
				m_result[index].imagePath = filePath;
				m_result[index].image = img;
			}
			ct::logger::info("[SRX] %s image received: %s", id.toStdString().c_str(), filePath.toStdString().c_str());
		}

		QFile::remove(filePath); //loaded into memory - keep the drop folder clean

		if (!img.isNull()) emit imageReceived(id, filePath);
	}
	else if (suffix == "json") {
		parseHistoricalJson(index, filePath);
		QFile::remove(filePath);
	}
	else {
		ct::logger::debug("[SRX FTP] %s pushed unhandled file type: %s",
			id.toStdString().c_str(), filePath.toStdString().c_str());
	}
}

void SRXManager::parseHistoricalJson(int index, const QString& path)
{
	QJsonObject root;
	if (!jsonHelper::loadJson(path, root)) {
		ct::logger::error("[SRX] Failed to parse historical JSON: %s", path.toStdString().c_str());
		return;
	}

	const QString id = idOf(index);

	//keys are searched recursively - the schema nests differently across SR families
	const QJsonValue result = findJsonKey(root, "result");
	const QJsonValue readtime = findJsonKey(root, "readtime");
	const QJsonValue outdata = findJsonKey(root, "outdata");
	const QJsonValue width = findJsonKey(root, "imagewidth");
	const QJsonValue height = findJsonKey(root, "imageheight");
	const QJsonValue corners = findJsonKey(root, "codecorner");

	//outdata is hex-encoded shift-jis; a failed read decodes to "ERROR"
	QString decoded;
	if (outdata.isString()) {
		QByteArray bytes = QByteArray::fromHex(outdata.toString().toLatin1());
		QTextCodec* codec = QTextCodec::codecForName("Shift-JIS");
		decoded = codec ? codec->toUnicode(bytes) : QString::fromLatin1(bytes);
	}

	QVector<QPolygonF> quads;
	if (corners.isArray()) {
		for (const auto& quadVal : corners.toArray()) {
			QPolygonF poly;
			for (const auto& ptVal : quadVal.toArray()) {
				auto pt = ptVal.toArray();
				if (pt.size() >= 2) poly << QPointF(pt[0].toDouble(), pt[1].toDouble());
			}
			if (!poly.isEmpty()) quads.append(poly);
		}
	}

	{
		std::lock_guard<std::mutex> lock(m_mutex);
		auto& r = m_result[index];
		r.ok = (result.toString() == "OK");
		r.readTimeMs = readtime.isDouble() ? (int)readtime.toDouble() : -1;
		r.imageWidth = (int)width.toDouble();
		r.imageHeight = (int)height.toDouble();
		r.corners = quads;
		r.timestamp = QDateTime::currentDateTime();
		if (!decoded.isEmpty() && decoded != "ERROR") r.code = segmentBarcode(decoded);
	}

	ct::logger::info("[SRX] %s historical data: result=%s readtime=%dms codes=%d",
		id.toStdString().c_str(), result.toString().toStdString().c_str(),
		readtime.isDouble() ? (int)readtime.toDouble() : -1, quads.size());

	emit readResultReceived(id);
}

//---------------------------------------------------------------- data getters

QString SRXManager::lastBarcode(const QString& id) const
{
	std::lock_guard<std::mutex> lock(m_mutex);
	int index = indexOf(id);
	return (index >= 0) ? m_result[index].code : QString();
}

SRXReadResult SRXManager::lastResult(const QString& id) const
{
	std::lock_guard<std::mutex> lock(m_mutex);
	int index = indexOf(id);
	return (index >= 0) ? m_result[index] : SRXReadResult();
}

QString SRXManager::lastImagePath(const QString& id) const
{
	std::lock_guard<std::mutex> lock(m_mutex);
	int index = indexOf(id);
	return (index >= 0) ? m_result[index].imagePath : QString();
}

QImage SRXManager::lastImage(const QString& id) const
{
	std::lock_guard<std::mutex> lock(m_mutex);
	int index = indexOf(id);
	return (index >= 0) ? m_result[index].image : QImage();
}

void SRXManager::clearResults()
{
	std::lock_guard<std::mutex> lock(m_mutex);
	for (int i = 0; i < READER_COUNT; i++) {
		QString imagePath = m_result[i].imagePath; //keep the last image on screen
		QImage image = m_result[i].image;
		m_result[i] = SRXReadResult();
		m_result[i].id = idOf(i);
		m_result[i].imagePath = imagePath;
		m_result[i].image = image;
	}
}
