#pragma once
#include "Profiler_Keyence.h"
#include "Logger.h"
#include "mtrx.h"
#include "MessageQue.h"
#include "MbufPoolManager.h"
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QStringList>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <cstring>

extern TMessageQue<FrameInfo> g_imageQueue;

/*
* Setting addresses. Every one of these is from section 11.3 of the LJ-X8000A
* Communication Library Reference Manual - keep the manual page next to the constant so
* the next person can check it without hunting.
*/
namespace {

	constexpr unsigned char KY_TYPE_COMMON = 0x02;   // common measurement settings

	//Common/environment settings have no category of their own - the manual says to send 0
	//(p.67). Same numeric value as KY_CAT_TRIGGER, spelled differently so the call sites read
	//honestly.
	constexpr unsigned char KY_CAT_NONE = 0x00;

	//Category 0x00 - trigger settings (manual p.70-72)
	constexpr unsigned char KY_CAT_TRIGGER = 0x00;
	constexpr unsigned char KY_IT_TRIGGER_MODE = 0x01;   // 0 continuous, 1 external, 2 encoder
	constexpr unsigned char KY_IT_SAMPLING_CYCLE = 0x02;
	constexpr unsigned char KY_IT_BATCH_ENABLE = 0x03;   // 0 off, 1 on
	constexpr unsigned char KY_IT_PITCH_ENABLE = 0x04;   // inter-trigger pitch on/off
	constexpr unsigned char KY_IT_PITCH_VALUE = 0x05;   // 0.0001 mm units, 1..500000
	constexpr unsigned char KY_IT_ENCODER_MODE = 0x07;   // 0 1-phase, 1 2-phase x1, 2 x2, 3 x4
	constexpr unsigned char KY_IT_SUBSAMPLE_EN = 0x08;   // 0 off, 1 on
	constexpr unsigned char KY_IT_SUBSAMPLE_CNT = 0x09;   // 2..1000
	constexpr unsigned char KY_IT_BATCH_COUNT = 0x0A;   // 50..60000 profiles

	//Category 0x01 - imaging settings (manual p.74-76)
	constexpr unsigned char KY_CAT_IMAGING = 0x01;
	constexpr unsigned char KY_IT_DYNAMIC_RANGE = 0x05;   // LJ-X: 1..9
	constexpr unsigned char KY_IT_EXPOSURE_TIME = 0x06;   // index into the table below
	constexpr unsigned char KY_IT_EXPOSURE_MODE = 0x07;   // 0 standard, 1 multi-synth, 2 multi-opt
	constexpr unsigned char KY_IT_LIGHT_CTRL_MODE = 0x0B;   // 0 auto, 1 manual
	constexpr unsigned char KY_IT_LIGHT_UPPER = 0x0C;   // 1..99
	constexpr unsigned char KY_IT_LIGHT_LOWER = 0x0D;   // 1..99
	constexpr unsigned char KY_IT_PEAK_SENSITIVITY = 0x0F;   // 1..5
	constexpr unsigned char KY_IT_INVALID_INTERP = 0x10;   // 0..255
	constexpr unsigned char KY_IT_PEAK_SELECTION = 0x11;   // 0 standard, 1 near, 2 far

	//Common settings (manual p.70)
	constexpr unsigned char KY_IT_ENCODER_MIN_TIME = 0x07;
	constexpr unsigned char KY_IT_LUMINANCE_OUTPUT = 0x0B;   // 0 height only, 1 height+luminance

	//dwNotify bits (manual p.84)
	constexpr DWORD KY_NOTIFY_STOPPED = 0x00000001;
	constexpr DWORD KY_NOTIFY_BATCH_DONE = 0x00010000;

	constexpr int KY_MAX_DEVICES = 6;
	constexpr int KY_BATCH_MIN = 50;
	constexpr int KY_BATCH_MAX = 60000;

	//LJ-X exposure indices, manual p.74. Deliberately NOT sorted - 10..15 interleave with
	//0..9, which is why the nearest-match below scans the whole table instead of bisecting.
	//Single definition; setExposure() reports the actual value from the same array.
	constexpr int kExposureUs[] = {
		15, 30, 60, 120, 240, 480, 960, 1700, 4800, 9600,
		80, 160, 210, 320, 380, 640
	};
	constexpr int kExposureCount = sizeof(kExposureUs) / sizeof(kExposureUs[0]);

	// dwUser carries the device id; this is how a C callback finds its instance.
	// Deliberately not the single s_instance shortcut Profiler_SSZN uses - the LJ-X API
	// supports six devices and hands us the id back, so there is no reason to give that up.
	Profiler_Keyence* g_devices[KY_MAX_DEVICES] = { nullptr };
	std::mutex        g_deviceMutex;

} // namespace


Profiler_Keyence::Profiler_Keyence()
{
	ct::logger::info("[Profiler_Keyence] Constructed");
}

Profiler_Keyence::~Profiler_Keyence()
{
	// Teardown lives in disconnect(); IProfiler has no virtual destructor and
	// ProfilerManager never deletes its pointers, so this may never run.
	ct::logger::info("[Profiler_Keyence] Destroyed");
}


/* ------------------------------------------------------------------ helpers */

bool Profiler_Keyence::safeGuard() const
{
	if (!m_enable) {
		ct::logger::warn("[Profiler_Keyence] Profiler not enabled");
		return false;
	}
	return true;
}

bool Profiler_Keyence::parseIp(const QString& ip, unsigned char out[4]) const
{
	const QStringList parts = ip.trimmed().split('.');
	if (parts.size() != 4) {
		ct::logger::error("[Profiler_Keyence] Invalid IP format: %s", ip.toUtf8().constData());
		return false;
	}
	for (int i = 0; i < 4; ++i) {
		bool ok = false;
		const int val = parts[i].toInt(&ok);
		if (!ok || val < 0 || val > 255) {
			ct::logger::error("[Profiler_Keyence] Invalid IP octet: %s (index %d)",
				parts[i].toUtf8().constData(), i);
			return false;
		}
		out[i] = static_cast<unsigned char>(val);
	}
	return true;
}

/*
* Every documented setting is a 4-byte block: the value in the low bytes, the rest
* reserved and fixed to zero (manual p.68 onward). Sending a zero-initialised int32
* satisfies both the 1-byte and the 2/4-byte items on little-endian.
*/
bool Profiler_Keyence::setSetting(unsigned char type, unsigned char category, unsigned char item,
	const void* data, unsigned int size, const char* what,
	unsigned char target1)
{
	LJX8IF_TARGET_SETTING ts{};
	ts.byType = type;
	ts.byCategory = category;
	ts.byItem = item;
	ts.byTarget1 = target1;

	DWORD detailedError = 0;
	const LONG rc = LJX8IF_SetSetting(m_deviceId,
		LJX8IF_SETTING_DEPTH_RUNNING,
		ts,
		const_cast<void*>(data),
		size,
		&detailedError);

	if (rc != LJX8IF_RC_OK) {
		ct::logger::error("[Profiler_Keyence] SetSetting(%s) failed rc=0x%X detail=0x%X "
			"(type=0x%02X cat=0x%02X item=0x%02X)",
			what, rc, detailedError, type, category, item);
		m_errorMsg = QStringLiteral("SetSetting(%1) rc=0x%2").arg(what).arg(rc, 0, 16);
		return false;
	}

	ct::logger::info("[Profiler_Keyence] SetSetting(%s) OK", what);
	return true;
}

/*
* LJ-X exposure is a discrete index, and the table is NOT monotonic - indices 10..15
* interleave with 0..9 (manual p.74). Scan the whole table; do not binary search.
*/
int Profiler_Keyence::nearestExposureIndex(int us)
{
	int bestIdx = 0;
	int bestDiff = std::abs(us - kExposureUs[0]);
	for (int i = 1; i < kExposureCount; ++i) {
		const int diff = std::abs(us - kExposureUs[i]);
		if (diff < bestDiff) {
			bestDiff = diff;
			bestIdx = i;
		}
	}
	return bestIdx;
}

/*
* Grey level -> micron factor, per head. Manual p.40 / p.54.
* Reading it from the controller means the operator never has to configure it.
*/
double Profiler_Keyence::zPitchForHead(const QString& model)
{
	const QString m = model.toUpper();
	if (m.contains("8020")) return 0.4;
	if (m.contains("8030")) return 0.8;
	if (m.contains("8060")) return 0.8;
	if (m.contains("8070")) return 1.6;
	if (m.contains("8080")) return 1.6;
	if (m.contains("8100")) return 4.0;
	if (m.contains("8200")) return 4.0;
	if (m.contains("8300")) return 4.0;
	if (m.contains("8400")) return 8.0;
	if (m.contains("8900")) return 16.0;

	ct::logger::warn("[Profiler_Keyence] Unknown head model '%s' - Z pitch defaulting to 1.6 um/level. "
		"Heights will be wrong by a scale factor until this is corrected.",
		model.toUtf8().constData());
	return 1.6;
}

void Profiler_Keyence::readHeadIdentity()
{
	CHAR headModel[32] = { 0 };
	if (LJX8IF_GetHeadModel(m_deviceId, headModel) == LJX8IF_RC_OK) {
		m_headModel = QString::fromLatin1(headModel).trimmed();
	}
	else {
		m_headModel = QStringLiteral("UNKNOWN");
		ct::logger::warn("[Profiler_Keyence] GetHeadModel failed");
	}

	CHAR ctrlSerial[32] = { 0 };
	CHAR headSerial[32] = { 0 };
	if (LJX8IF_GetSerialNumber(m_deviceId, ctrlSerial, headSerial) == LJX8IF_RC_OK) {
		m_serialNumber = QStringLiteral("%1/%2")
			.arg(QString::fromLatin1(ctrlSerial).trimmed())
			.arg(QString::fromLatin1(headSerial).trimmed());
	}
	else {
		m_serialNumber = QStringLiteral("UNKNOWN");
	}

	const LJX8IF_VERSION_INFO v = LJX8IF_GetVersion();
	m_firmwareVersion = QStringLiteral("DLL %1.%2.%3.%4")
		.arg(v.nMajorNumber).arg(v.nMinorNumber)
		.arg(v.nRevisionNumber).arg(v.nBuildNumber);

	m_zPitchUm = zPitchForHead(m_headModel);

	ct::logger::info("[Profiler_Keyence] Head model  : %s", m_headModel.toUtf8().constData());
	ct::logger::info("[Profiler_Keyence] Serial      : %s", m_serialNumber.toUtf8().constData());
	ct::logger::info("[Profiler_Keyence] Library     : %s", m_firmwareVersion.toUtf8().constData());
	ct::logger::info("[Profiler_Keyence] Z pitch     : %.2f um per grey level", m_zPitchUm);
}


/* --------------------------------------------------------- high-speed comms */

void Profiler_Keyence::finalizeHighSpeed()
{
	if (!m_highSpeedReady) return;
	LJX8IF_FinalizeHighSpeedDataCommunication(m_deviceId);
	m_highSpeedReady = false;
}

/*
* Manual 12.3 (6): to restart high-speed communication you must go all the way back
* through Finalize -> Close -> Open -> Initialize -> PreStart. The callback delivery size
* is fixed at Initialize time, so any change to the batch count has to come through here.
*/
bool Profiler_Keyence::initHighSpeed()
{
	unsigned char octets[4] = { 0 };
	if (!parseIp(m_ip, octets)) return false;

	// Manual p.69: "Do not set the command port number and the high-speed port number to the
	// same number." Equal ports fail in a way that reads like a network fault rather than a
	// config error - EthernetOpen succeeds (the controller is listening on that port) and then
	// InitializeHighSpeed... returns RC_ERR_OPEN because our own command socket already holds
	// it. Refuse up front and say so, rather than letting that play out.
	if (m_commandPort == m_highSpeedPort) {
		ct::logger::error("[Profiler_Keyence] commandPort and highSpeedPort are both %d - they "
			"must differ (manual p.69). Fix keyence.json.", m_commandPort);
		m_errorMsg = QStringLiteral("commandPort and highSpeedPort must differ (both %1)")
			.arg(m_commandPort);
		m_connectionStatus = false;
		return false;
	}

	finalizeHighSpeed();
	LJX8IF_CommunicationClose(m_deviceId);

	LJX8IF_ETHERNET_CONFIG cfg{};
	for (int i = 0; i < 4; ++i) cfg.abyIpAddress[i] = octets[i];
	cfg.wPortNo = static_cast<WORD>(m_commandPort);   // command channel, NOT the high-speed one

	LONG rc = LJX8IF_EthernetOpen(m_deviceId, &cfg);
	if (rc != LJX8IF_RC_OK) {
		ct::logger::error("[Profiler_Keyence] EthernetOpen failed rc=0x%X", rc);
		m_connectionStatus = false;
		return false;
	}

	rc = LJX8IF_InitializeHighSpeedDataCommunicationSimpleArray(
		m_deviceId,
		&cfg,
		static_cast<WORD>(m_highSpeedPort),
		&Profiler_Keyence::SimpleArrayCallback,
		static_cast<DWORD>(m_batchCount),   // one callback == one whole batch
		static_cast<DWORD>(m_deviceId));    // comes back as dwUser

	if (rc != LJX8IF_RC_OK) {
		ct::logger::error("[Profiler_Keyence] InitializeHighSpeedSimpleArray failed rc=0x%X", rc);
		return false;
	}

	m_highSpeedReady = true;
	m_highSpeedDirty = false;
	ct::logger::info("[Profiler_Keyence] High-speed comms initialised (batch=%d, cmdPort=%d, hsPort=%d)",
		m_batchCount, m_commandPort, m_highSpeedPort);
	return true;
}


/* ------------------------------------------------------------- the callback */

void _cdecl Profiler_Keyence::SimpleArrayCallback(LJX8IF_PROFILE_HEADER* /*pHeaderArray*/,
	WORD* pHeightArray,
	WORD* pLuminanceArray,
	DWORD dwLuminanceEnable,
	DWORD dwProfileDataCount,
	DWORD dwCount,
	DWORD dwNotify,
	DWORD dwUser)
{
	if (dwUser >= KY_MAX_DEVICES) return;

	Profiler_Keyence* self = nullptr;
	{
		std::lock_guard<std::mutex> lock(g_deviceMutex);
		self = g_devices[dwUser];
	}
	if (!self || self->m_release.load()) return;

	if (dwNotify & KY_NOTIFY_STOPPED) {
		ct::logger::info("[Profiler_Keyence] Callback: high-speed communication stopped");
	}
	if (dwNotify & KY_NOTIFY_BATCH_DONE) {
		ct::logger::info("[Profiler_Keyence] Callback: batch complete");
	}

	if (dwCount == 0 || pHeightArray == nullptr) return;
	if (!self->m_softTriggered) {
		ct::logger::warn("[Profiler_Keyence] Callback fired while not armed - discarding %u profiles", dwCount);
		return;
	}

	const int rows = static_cast<int>(dwCount);
	const int cols = static_cast<int>(dwProfileDataCount);
	if (rows <= 0 || cols <= 0) {
		ct::logger::warn("[Profiler_Keyence] Empty batch (rows=%d cols=%d) - skipping", rows, cols);
		return;
	}

	ct::logger::info("[Profiler_Keyence] Batch received: %d profiles x %d points (luminance=%u)",
		rows, cols, dwLuminanceEnable);

	FrameInfo& frame = self->m_frameInfo;
	frame.profiles.clear();

	frame.type = ct::s_height_map;
	frame.width = cols;
	frame.height = rows;
	frame.bufferSize = cols * rows;
	frame.channel = 1;

	frame.pHeightMap = mtrx::MPM::instance().acquire(cols, rows, 1, 16 + M_UNSIGNED);
	frame.pImage = mtrx::MPM::instance().acquire(cols, rows, 1, 8 + M_UNSIGNED);

	if (!frame.pHeightMap || !frame.pImage) {
		ct::logger::error("[Profiler_Keyence] shared buffers not allocated");
		return;
	}

	MIL_UINT16* hPtr = M_NULL; MIL_INT pitch16 = 0;
	MIL_UINT8* gPtr = M_NULL; MIL_INT pitch8 = 0;

	MbufInquire(frame.pHeightMap->id(), M_HOST_ADDRESS, &hPtr);
	MbufInquire(frame.pHeightMap->id(), M_PITCH, &pitch16);
	MbufInquire(frame.pImage->id(), M_HOST_ADDRESS, &gPtr);
	MbufInquire(frame.pImage->id(), M_PITCH, &pitch8);

	if (!hPtr || !gPtr) {
		ct::logger::error("[Profiler_Keyence] MbufInquire returned null host address");
		return;
	}

	/*
	* Straight copy. SimpleArray height is already 16-bit unsigned with 0 == invalid,
	* which is exactly what FrameInfo::pHeightMap wants (manual p.40). Downstream code
	* recovers microns with:  height_um = (grey - 32768) * m_zPitchUm
	*
	* Luminance is 0..1023 in a 16-bit word; pImage is 8-bit, so shift down by 2.
	*/
	const bool haveLuminance = (dwLuminanceEnable == 1) && (pLuminanceArray != nullptr);

	for (int r = 0; r < rows; ++r) {
		MIL_UINT16* hLine = hPtr + r * pitch16;
		MIL_UINT8* gLine = gPtr + r * pitch8;
		const WORD* hSrc = pHeightArray + (static_cast<size_t>(r) * cols);

		for (int c = 0; c < cols; ++c) {
			hLine[c] = static_cast<MIL_UINT16>(hSrc[c]);
		}

		if (haveLuminance) {
			const WORD* lSrc = pLuminanceArray + (static_cast<size_t>(r) * cols);
			for (int c = 0; c < cols; ++c) {
				gLine[c] = static_cast<MIL_UINT8>(lSrc[c] >> 2);   // 0..1023 -> 0..255
			}
		}
		else {
			std::memset(gLine, 0, static_cast<size_t>(cols));
		}
	}

	g_imageQueue.push_back(frame);
	frame = FrameInfo();

	ct::logger::info("[Profiler_Keyence] Enqueued frame (%dx%d)", cols, rows);

	{
		std::lock_guard<std::mutex> lock(self->m_mutex);
		self->m_softTriggered = false;
	}
	self->m_conditionVariable.notify_one();
}


/* -------------------------------------------------------------------- Query */

const double Profiler_Keyence::getExposure() const
{
	return m_exposure;
}

const double Profiler_Keyence::getYResolution() const
{
	// mm per acquired profile line, i.e. after sub-sampling.
	return (m_yPitchUm * m_divider) / 1000.0;
}

const QString& Profiler_Keyence::getFirmwareVersion() const
{
	// Returns a member, not a temporary - Profiler_SSZN::getFirmwareVersion() returns a
	// dangling reference here and that bug is not worth inheriting.
	return m_firmwareVersion;
}

const QString& Profiler_Keyence::getSerialNumber() const
{
	return m_serialNumber;
}

bool Profiler_Keyence::isConnected() const
{
	if (!safeGuard()) return false;
	return m_connectionStatus;
}

const bool Profiler_Keyence::isGrabbing() const
{
	return m_softTriggered;
}


/* --------------------------------------------------------------- Connection */

bool Profiler_Keyence::enable(bool enable)
{
	m_enable = enable;
	ct::logger::info("[Profiler_Keyence] enable = %d", enable);
	return true;
}

bool Profiler_Keyence::connect(QString ip)
{
	if (!safeGuard()) return false;

	// The parameter is named 'sn' in IProfiler but every backend receives the IP string
	// from profiler.json.
	m_ip = ip.trimmed();

	unsigned char octets[4] = { 0 };
	if (!parseIp(m_ip, octets)) {
		m_connectionStatus = false;
		return false;
	}

	if (LJX8IF_Initialize() != LJX8IF_RC_OK) {
		ct::logger::error("[Profiler_Keyence] LJX8IF_Initialize failed");
		m_connectionStatus = false;
		return false;
	}

	{
		std::lock_guard<std::mutex> lock(g_deviceMutex);
		if (m_deviceId < 0 || m_deviceId >= KY_MAX_DEVICES) {
			ct::logger::error("[Profiler_Keyence] Invalid device id %d", m_deviceId);
			return false;
		}
		g_devices[m_deviceId] = this;
	}
	m_release = false;

	if (!initHighSpeed()) {
		m_connectionStatus = false;
		return false;
	}

	m_connectionStatus = true;
	ct::logger::info("[Profiler_Keyence] Connected to: %s (device %d)",
		m_ip.toUtf8().constData(), m_deviceId);

	readHeadIdentity();
	return true;
}

bool Profiler_Keyence::disconnect()
{
	if (!safeGuard()) return false;
	ct::logger::info("[Profiler_Keyence] Disconnecting");

	// Stop the world before tearing down, so no callback can land on a dead object.
	m_release = true;
	{
		std::lock_guard<std::mutex> lock(g_deviceMutex);
		if (m_deviceId >= 0 && m_deviceId < KY_MAX_DEVICES) g_devices[m_deviceId] = nullptr;
	}

	// Only the two stop commands need a live connection - they are command round-trips, and
	// against a dead or half-open path each one blocks for the SDK's internal timeout (there
	// is no API to shorten it). A failed connect that still managed EthernetOpen therefore
	// costs ~2 timeouts on every shutdown, which is what made this feel hung. stop() has
	// always guarded on m_connectionStatus for the same reason; this was missing it.
	//
	// Deliberately NOT an early return like CAM_HIK::disconnect() does: CommunicationClose
	// and the Finalize bookkeeping below must still run, or the socket EthernetOpen left
	// behind leaks until the process exits.
	if (m_connectionStatus) {
		LJX8IF_StopMeasure(m_deviceId);
		LJX8IF_StopHighSpeedDataCommunication(m_deviceId);
	}
	else {
		ct::logger::info("[Profiler_Keyence] Not connected - skipping stop commands");
	}

	finalizeHighSpeed();

	const LONG rc = LJX8IF_CommunicationClose(m_deviceId);
	if (rc != LJX8IF_RC_OK) {
		ct::logger::error("[Profiler_Keyence] CommunicationClose failed rc=0x%X", rc);
	}

	// LJX8IF_Finalize() tears down the whole DLL, not just this device, so only the last
	// profiler standing may call it.
	bool othersStillOpen = false;
	{
		std::lock_guard<std::mutex> lock(g_deviceMutex);
		for (int i = 0; i < KY_MAX_DEVICES; ++i) {
			if (g_devices[i]) { othersStillOpen = true; break; }
		}
	}
	if (othersStillOpen) {
		ct::logger::info("[Profiler_Keyence] Other devices still open - skipping LJX8IF_Finalize");
	}
	else {
		LJX8IF_Finalize();
	}

	m_connectionStatus = false;
	m_conditionVariable.notify_all();
	return true;
}


/* -------------------------------------------------------------- Acquisition */

bool Profiler_Keyence::start()
{
	if (!safeGuard()) return false;
	if (!m_connectionStatus) {
		ct::logger::error("[Profiler_Keyence] Cannot start: not connected");
		return false;
	}

	// The callback delivery size is baked in at Initialize time, so a changed batch
	// count means a full re-init before we arm.
	if (m_highSpeedDirty && !initHighSpeed()) {
		ct::logger::error("[Profiler_Keyence] Cannot start: high-speed re-init failed");
		return false;
	}

	LJX8IF_HIGH_SPEED_PRE_START_REQ req{};
	req.bySendPosition = 2;                 // from current after batch commitment
	LJX8IF_PROFILE_INFO info{};

	LONG rc = LJX8IF_PreStartHighSpeedDataCommunication(m_deviceId, &req, &info);
	if (rc != LJX8IF_RC_OK) {
		ct::logger::error("[Profiler_Keyence] PreStartHighSpeed failed rc=0x%X", rc);
		return false;
	}

	m_xPoints = info.wProfileDataCount;
	m_luminanceEnabled = (info.byLuminanceOutput != 0);

	// lXPitch is in 0.01 um units. This is the true X field of view of the head as
	// currently configured - the number the FOV tables in VisionApp_CRUD.cpp and
	// VisionApp_JSON.cpp need. Logged loudly because it cannot be known before first
	// connection and is currently a placeholder in those tables.
	m_measuredFovMm = (info.lXPitch / 100.0) * m_xPoints / 1000.0;
	ct::logger::info("[Profiler_Keyence] ===== MEASURED LASER FOV = %.4f mm (%d points @ %.4f um) =====",
		m_measuredFovMm, m_xPoints, info.lXPitch / 100.0);

	{
		std::lock_guard<std::mutex> lock(m_mutex);
		m_softTriggered = true;
	}

	rc = LJX8IF_StartHighSpeedDataCommunication(m_deviceId);
	if (rc != LJX8IF_RC_OK) {
		ct::logger::error("[Profiler_Keyence] StartHighSpeed failed rc=0x%X", rc);
		std::lock_guard<std::mutex> lock(m_mutex);
		m_softTriggered = false;
		return false;
	}

	// Batch is armed here but no profile is emitted until the encoder clocks one, which
	// happens as JobThread jogs the gantry across the part. Returning promptly is
	// mandatory: block here and the gantry never moves.
	rc = LJX8IF_StartMeasure(m_deviceId);
	if (rc != LJX8IF_RC_OK) {
		ct::logger::error("[Profiler_Keyence] StartMeasure failed rc=0x%X", rc);
		LJX8IF_StopHighSpeedDataCommunication(m_deviceId);
		std::lock_guard<std::mutex> lock(m_mutex);
		m_softTriggered = false;
		return false;
	}

	ct::logger::info("[Profiler_Keyence] Armed (batch=%d profiles, %d points/profile)",
		m_batchCount, m_xPoints);
	return true;
}

bool Profiler_Keyence::stop()
{
	if (!safeGuard()) return false;
	ct::logger::info("[Profiler_Keyence] Stopping acquisition");

	// stop() is called before every scan as well as after, so a stop with nothing
	// running must succeed quietly.
	if (!m_connectionStatus) return true;

	LONG rc = LJX8IF_StopMeasure(m_deviceId);
	if (rc != LJX8IF_RC_OK) {
		ct::logger::info("[Profiler_Keyence] StopMeasure rc=0x%X (harmless if not measuring)", rc);
	}

	rc = LJX8IF_StopHighSpeedDataCommunication(m_deviceId);
	if (rc != LJX8IF_RC_OK) {
		ct::logger::info("[Profiler_Keyence] StopHighSpeed rc=0x%X (harmless if not started)", rc);
	}

	{
		std::lock_guard<std::mutex> lock(m_mutex);
		m_softTriggered = false;
	}
	m_conditionVariable.notify_one();
	return true;
}

bool Profiler_Keyence::snapShot()
{
	// NOT a hardware limitation. The controller and the SDK both support single-shot
	// capture: LJX8IF_Trigger() fires a software trigger (LJX8_IF.h:356) and
	// LJX8IF_GetProfile() fetches one profile over the command channel, no batch and no
	// high-speed stream. Neither is called anywhere in this tree. The manual (p.5-4) lists
	// "communication command" as a documented input for External trigger.
	//
	// The blocker is the mode WE impose: loadConfig writes triggerMode = 2 (encoder) on
	// connect and nothing ever changes it, so the trigger source is the encoder terminal
	// and a stationary gantry generates no profile to fetch. Implementing this means
	// switching mode (continuous for a live preview, external for a true one-shot),
	// capturing, and switching back - where the hard part is not the API calls but state
	// restoration: a failed switch-back leaves the controller free-running at the sampling
	// clock, and the next production scan is silently wrong.
	//
	// Returning false rather than faking success the way Profiler_SSZN does: a fake would
	// feed a stale or empty frame into warpage compensation and corrupt the height map.
	//
	// But be aware what false actually buys, because it is less than it looks. EVERY caller
	// ignores this return value (JobThread.cpp:1753, 1800, 1845, 1913) and then calls
	// waitAcquisition(id, PROFILER_TIMEOUT) unconditionally - 60 s (JobThread.h:38) - before
	// carrying on with whatever frame was already in the buffer. In fullWarpageCompensation
	// that pair sits inside a per-view loop, so it is 60 s PER VIEW.
	//
	// Harmless today only because nothing live calls it: centerLaserZ has no caller at all,
	// and warpageCompensation is reached from preAcquisition() but only when _warpageMethod
	// is "Subsampling" or "Fullsampling" - it is "None" or absent in every recipe. Note the
	// Guided 2D/3D Alignment tab does NOT go through here, so laserConfig.json's offset is
	// not blocked by this.
	//
	// REVISIT WHEN: someone sets _warpageMethod to Subsampling or Fullsampling on a Keyence
	// recipe. That needs both the capture above and a fast-fail in waitAcquisition, or
	// production stalls a minute per view and then measures against a stale frame.
	ct::logger::error("[Profiler_Keyence] snapShot not available in encoder batch mode");
	m_errorMsg = QStringLiteral("snapShot not supported in encoder batch mode");
	return false;
}


/* ------------------------------------------------------------------ Control */

bool Profiler_Keyence::enableIntensityMap(bool enable)
{
	if (!safeGuard()) return false;

	// Luminance output is a COMMON setting, not per-program (manual p.70).
	int32_t v = enable ? 1 : 0;
	if (!setSetting(KY_TYPE_COMMON, KY_CAT_NONE, KY_IT_LUMINANCE_OUTPUT, &v, 4, "luminanceOutput"))
		return false;

	m_luminanceEnabled = enable;
	return true;
}

bool Profiler_Keyence::setScanLength(double mm)
{
	if (!safeGuard()) return false;

	// Y pitch per acquired line is the encoder pitch multiplied by the sub-sampling count.
	const double effectivePitchMm = (m_yPitchUm * m_divider) / 1000.0;
	if (effectivePitchMm <= 0.0) {
		ct::logger::error("[Profiler_Keyence] setScanLength: invalid Y pitch (%.4f um, divider %d)",
			m_yPitchUm, m_divider);
		return false;
	}

	int batch = static_cast<int>(std::lround(mm / effectivePitchMm));
	if (batch < KY_BATCH_MIN || batch > KY_BATCH_MAX) {
		ct::logger::warn("[Profiler_Keyence] Batch count %d out of range [%d..%d] - clamping",
			batch, KY_BATCH_MIN, KY_BATCH_MAX);
		batch = std::max(KY_BATCH_MIN, std::min(KY_BATCH_MAX, batch));
	}

	ct::logger::info("[Profiler_Keyence] setScanLength %.3f mm -> %d profiles (pitch %.4f mm)",
		mm, batch, effectivePitchMm);

	int32_t v = batch;
	if (!setSetting(0x10 + m_programNo, KY_CAT_TRIGGER, KY_IT_BATCH_COUNT, &v, 4, "batchCount"))
		return false;

	if (batch != m_batchCount) {
		m_batchCount = batch;
		m_highSpeedDirty = true;   // callback delivery size must follow the batch size
	}
	return true;
}

/*
* The LJ-X has no gain. The closest analogue is Dynamic Range (1..9), which is what the
* 3D Optics tab's gain field is mapped onto. This is a deliberate product decision, not
* an equivalence - logged every time so it is visible in the field.
*/
bool Profiler_Keyence::setGain(double gain)
{
	if (!safeGuard()) return false;

	int dr = static_cast<int>(std::lround(gain));
	dr = std::max(1, std::min(9, dr));

	ct::logger::info("[Profiler_Keyence] setGain %.2f -> dynamic range %d (LJ-X has no gain setting)",
		gain, dr);

	int32_t v = dr;
	return setSetting(0x10 + m_programNo, KY_CAT_IMAGING, KY_IT_DYNAMIC_RANGE, &v, 4, "dynamicRange");
}

bool Profiler_Keyence::setDuoHeadGain(double gain, double gain2)
{
	ct::logger::error("[Profiler_Keyence] setDuoHeadGain not available - LJ-X8000A is single head");
	return false;
}

bool Profiler_Keyence::setDivider(int divider)
{
	if (!safeGuard()) return false;

	m_divider = std::max(1, divider);

	// Sub-sampling has to be switched off entirely for a divider of 1; the count itself
	// only accepts 2..1000 (manual p.72).
	int32_t enable = (m_divider > 1) ? 1 : 0;
	if (!setSetting(0x10 + m_programNo, KY_CAT_TRIGGER, KY_IT_SUBSAMPLE_EN, &enable, 4, "subSampleEnable"))
		return false;

	if (m_divider > 1) {
		int32_t count = std::min(1000, m_divider);
		if (!setSetting(0x10 + m_programNo, KY_CAT_TRIGGER, KY_IT_SUBSAMPLE_CNT, &count, 4, "subSampleCount"))
			return false;
	}

	ct::logger::info("[Profiler_Keyence] setDivider OK (divider = %d)", m_divider);
	return true;
}

bool Profiler_Keyence::setExposureMode(ExposureMode mode)
{
	if (!safeGuard()) return false;

	int32_t v = 0;
	switch (mode) {
	case SINGLE:   v = 0; break;   // standard
	case MULTI:    v = 1; break;   // multi emission (synthesis)
	case DYNAMIC:  v = 2; break;   // multi emission (optimized light intensity)
	case PARALLEL:
		ct::logger::error("[Profiler_Keyence] PARALLEL exposure needs a dual head - not available");
		return false;
	default:
		ct::logger::error("[Profiler_Keyence] Unknown exposure mode %d", (int)mode);
		return false;
	}

	return setSetting(0x10 + m_programNo, KY_CAT_IMAGING, KY_IT_EXPOSURE_MODE, &v, 4, "exposureMode");
}

bool Profiler_Keyence::setExposure(double us)
{
	if (!safeGuard()) return false;

	const int idx = nearestExposureIndex(static_cast<int>(std::lround(us)));

	ct::logger::info("[Profiler_Keyence] setExposure %.1f us -> index %d (%d us actual)",
		us, idx, kExposureUs[idx]);

	int32_t v = idx;
	if (!setSetting(0x10 + m_programNo, KY_CAT_IMAGING, KY_IT_EXPOSURE_TIME, &v, 4, "exposureTime"))
		return false;

	m_exposure = kExposureUs[idx];
	return true;
}

bool Profiler_Keyence::setMultiExposure(double us, double us2)
{
	// Multi emission is configured by emission COUNT (2/4 or 3/5), not by two exposure
	// times, so the pair of microsecond values has nowhere to go.
	ct::logger::error("[Profiler_Keyence] setMultiExposure not available - LJ-X multi emission "
		"takes an emission count, not two exposure times");
	return false;
}

bool Profiler_Keyence::setDynamicExposure(double min_us, double max_us)
{
	ct::logger::error("[Profiler_Keyence] setDynamicExposure not available on LJ-X8000A");
	return false;
}

bool Profiler_Keyence::setParallelExposure(double min_us, double max_us)
{
	ct::logger::error("[Profiler_Keyence] setParallelExposure needs a dual head - not available");
	return false;
}

bool Profiler_Keyence::waitAcquisition(int ms)
{
	std::unique_lock<std::mutex> lock(m_mutex);

	if (!m_softTriggered) {
		// Nothing was armed, so there is nothing to wait for. Returning false here would
		// be read by JobThread::scan() as a hard fault and unload the board.
		ct::logger::info("[Profiler_Keyence] waitAcquisition: not armed, nothing to wait for");
		return true;
	}

	const bool signalled = m_conditionVariable.wait_for(
		lock, std::chrono::milliseconds(ms), [this] { return !m_softTriggered; });

	if (signalled) {
		ct::logger::info("[Profiler_Keyence] waitAcquisition complete");
		return true;
	}

	ct::logger::error("[Profiler_Keyence] waitAcquisition TIMED OUT after %d ms", ms);
	m_errorMsg = QStringLiteral("Acquisition timed out after %1 ms").arg(ms);
	m_softTriggered = false;
	return false;
}

bool Profiler_Keyence::setMSR(bool enable)
{
	// MSR is a SmartRay high-resolution mode. Nothing to do here, but returning true so
	// ProfilerManager's startup chain is not tripped by a setting that does not apply.
	if (enable) {
		ct::logger::warn("[Profiler_Keyence] MSR requested but is SmartRay-specific - ignoring");
	}
	return true;
}

/*
* No binarisation threshold on the LJ-X. Peak detection sensitivity (1..5) is the nearest
* control over which reflections become a profile point. Same class of decision as setGain.
*/
bool Profiler_Keyence::setLaserLineThreshold(double threshold)
{
	if (!safeGuard()) return false;

	int level = static_cast<int>(std::lround(threshold));
	level = std::max(1, std::min(5, level));

	ct::logger::info("[Profiler_Keyence] setLaserLineThreshold %.2f -> peak sensitivity %d",
		threshold, level);

	int32_t v = level;
	return setSetting(0x10 + m_programNo, KY_CAT_IMAGING, KY_IT_PEAK_SENSITIVITY, &v, 4, "peakSensitivity");
}


/* --------------------------------------------------------------------- Data */

const FrameInfo& Profiler_Keyence::getFrame() const
{
	return m_frameInfo;
}

FrameInfo& Profiler_Keyence::getFrame()
{
	return m_frameInfo;
}

void Profiler_Keyence::resetFrame()
{
	m_frameInfo = FrameInfo();
}


/* --------------------------------------------------------------------- Misc */

/*
* Called once at startup by ProfilerManager with the path built from profiler.json's
* Config_File field. Unlike Profiler_SSZN this actually reads the file; every key is
* optional and falls back to the value already in the member.
*/
bool Profiler_Keyence::loadConfig(QString path)
{
	if (!safeGuard()) return false;
	ct::logger::info("[Profiler_Keyence] loadConfig: %s", path.toUtf8().constData());

	// ProfilerManager's startup chain is create -> connect -> loadConfig -> setMSR, so by the
	// time we get here the connection has already been made using the default device id and
	// port. Remember them; if the file changes either one we have to reconnect below, or the
	// setting would silently do nothing until the next app restart (and not even then).
	const int prevDeviceId = m_deviceId;
	const int prevPort = m_highSpeedPort;
	const int prevCmdPort = m_commandPort;

	//--- defaults, overridden by the file if present
	int triggerMode = 2;    // encoder
	int batchEnable = 1;    // batch measurement ON - required for the batch callback
	int samplingCycle = -1;   // -1 = leave the controller's own setting alone
	int encoderMode = 1;    // 2-phase 1x
	int encoderMinTime = 0;    // 120 ns
	int dynamicRange = 1;
	int lightCtrlMode = 1;    // manual
	int lightUpper = 90;
	int lightLower = 90;
	int peakSensitivity = 1;
	int peakSelection = 0;
	int invalidInterp = 0;
	int luminance = m_luminanceEnabled ? 1 : 0;

	QFile f(path);
	if (f.open(QIODevice::ReadOnly)) {
		const QJsonObject o = QJsonDocument::fromJson(f.readAll()).object();
		f.close();

		auto num = [&o](const char* k, int def) {
			return o.contains(k) ? o.value(k).toInt(def) : def;
		};

		m_programNo = static_cast<unsigned char>(std::max(0, std::min(15, num("programNo", m_programNo))));
		m_commandPort = num("commandPort", m_commandPort);
		m_highSpeedPort = num("highSpeedPort", m_highSpeedPort);
		m_deviceId = std::max(0, std::min(KY_MAX_DEVICES - 1, num("deviceId", m_deviceId)));

		// Reject an equal pair here rather than at connect time, so the message names the file
		// the operator has to edit. Keeping the previous values means a typo degrades to the
		// working defaults instead of taking the sensor offline.
		if (m_commandPort == m_highSpeedPort) {
			ct::logger::error("[Profiler_Keyence] keyence.json sets commandPort and highSpeedPort "
				"both to %d - they must differ (manual p.69). Reverting to %d/%d.",
				m_commandPort, prevCmdPort, prevPort);
			m_commandPort = prevCmdPort;
			m_highSpeedPort = prevPort;
		}

		if (o.contains("yPitchUm"))  m_yPitchUm = o.value("yPitchUm").toDouble(m_yPitchUm);
		if (o.contains("zPitchUmOverride")) {
			const double z = o.value("zPitchUmOverride").toDouble(0.0);
			if (z > 0.0) {
				m_zPitchUm = z;
				ct::logger::warn("[Profiler_Keyence] Z pitch overridden from config: %.3f um", z);
			}
		}

		triggerMode = num("triggerMode", triggerMode);
		batchEnable = num("batchMeasurement", batchEnable);
		samplingCycle = num("samplingCycle", samplingCycle);
		encoderMode = num("encoderInputMode", encoderMode);
		encoderMinTime = num("encoderMinInputTime", encoderMinTime);
		dynamicRange = num("dynamicRange", dynamicRange);
		lightCtrlMode = num("lightControlMode", lightCtrlMode);
		lightUpper = num("lightUpperLimit", lightUpper);
		lightLower = num("lightLowerLimit", lightLower);
		peakSensitivity = num("peakSensitivity", peakSensitivity);
		peakSelection = num("peakSelection", peakSelection);
		invalidInterp = num("invalidInterpolation", invalidInterp);
		luminance = num("luminanceOutput", luminance);

		ct::logger::info("[Profiler_Keyence] Config loaded from file");
	}
	else {
		ct::logger::warn("[Profiler_Keyence] Config file not found (%s) - using built-in defaults",
			path.toUtf8().constData());
	}

	if (m_connectionStatus && (m_deviceId != prevDeviceId || m_highSpeedPort != prevPort
		|| m_commandPort != prevCmdPort)) {
		ct::logger::warn("[Profiler_Keyence] Config changed device/ports (%d, %d/%d -> %d, %d/%d) "
			"after connect - reconnecting", prevDeviceId, prevCmdPort, prevPort,
			m_deviceId, m_commandPort, m_highSpeedPort);

		{
			std::lock_guard<std::mutex> lock(g_deviceMutex);
			if (prevDeviceId >= 0 && prevDeviceId < KY_MAX_DEVICES) g_devices[prevDeviceId] = nullptr;
			g_devices[m_deviceId] = this;
		}

		LJX8IF_FinalizeHighSpeedDataCommunication(prevDeviceId);
		LJX8IF_CommunicationClose(prevDeviceId);
		m_highSpeedReady = false;

		if (!initHighSpeed()) {
			ct::logger::error("[Profiler_Keyence] Reconnect after config change FAILED");
			m_connectionStatus = false;
			return false;
		}
		readHeadIdentity();
	}

	const unsigned char prog = 0x10 + m_programNo;
	bool ok = true;
	int32_t v = 0;

	v = triggerMode;    ok &= setSetting(prog, KY_CAT_TRIGGER, KY_IT_TRIGGER_MODE, &v, 4, "triggerMode");
	v = batchEnable;    ok &= setSetting(prog, KY_CAT_TRIGGER, KY_IT_BATCH_ENABLE, &v, 4, "batchMeasurement");
	v = encoderMode;    ok &= setSetting(prog, KY_CAT_TRIGGER, KY_IT_ENCODER_MODE, &v, 4, "encoderInputMode");

	if (samplingCycle >= 0) {
		v = samplingCycle;
		ok &= setSetting(prog, KY_CAT_TRIGGER, KY_IT_SAMPLING_CYCLE, &v, 4, "samplingCycle");
	}

	v = dynamicRange;    ok &= setSetting(prog, KY_CAT_IMAGING, KY_IT_DYNAMIC_RANGE, &v, 4, "dynamicRange");
	v = lightCtrlMode;   ok &= setSetting(prog, KY_CAT_IMAGING, KY_IT_LIGHT_CTRL_MODE, &v, 4, "lightControlMode");
	v = std::max(1, std::min(99, lightUpper));
	ok &= setSetting(prog, KY_CAT_IMAGING, KY_IT_LIGHT_UPPER, &v, 4, "lightUpperLimit");
	v = std::max(1, std::min(99, lightLower));
	ok &= setSetting(prog, KY_CAT_IMAGING, KY_IT_LIGHT_LOWER, &v, 4, "lightLowerLimit");
	v = std::max(1, std::min(5, peakSensitivity));
	ok &= setSetting(prog, KY_CAT_IMAGING, KY_IT_PEAK_SENSITIVITY, &v, 4, "peakSensitivity");
	v = peakSelection;   ok &= setSetting(prog, KY_CAT_IMAGING, KY_IT_PEAK_SELECTION, &v, 4, "peakSelection");
	v = std::max(0, std::min(255, invalidInterp));
	ok &= setSetting(prog, KY_CAT_IMAGING, KY_IT_INVALID_INTERP, &v, 4, "invalidInterpolation");

	//--- common settings
	v = encoderMinTime;  ok &= setSetting(KY_TYPE_COMMON, KY_CAT_NONE, KY_IT_ENCODER_MIN_TIME, &v, 4, "encoderMinInputTime");
	v = luminance;       ok &= setSetting(KY_TYPE_COMMON, KY_CAT_NONE, KY_IT_LUMINANCE_OUTPUT, &v, 4, "luminanceOutput");
	m_luminanceEnabled = (luminance == 1);

	ct::logger::info("[Profiler_Keyence] loadConfig %s (programNo=%d, yPitch=%.3f um, cmdPort=%d, hsPort=%d)",
		ok ? "OK" : "completed WITH ERRORS", m_programNo, m_yPitchUm, m_commandPort, m_highSpeedPort);

	return ok;
}

QString Profiler_Keyence::errorMsg()
{
	return m_errorMsg;
}
