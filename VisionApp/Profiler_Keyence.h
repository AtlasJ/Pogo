#pragma once
#include "IProfiler.h"
#include <QString>
#include "FrameInfo.h"
#include <mutex>
#include <condition_variable>
#include <atomic>
#include "mtrx.h"

#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include "Keyence\include\LJX8_IF.h"
#include "Keyence\include\LJX8_ErrorCode.h"


/*
* Keyence LJ-X8000A backend.
*
* Reference: "LJ-X8000A Communication Library Reference Manual" (N06GB), bundled with
* LJ-X Navigator at lib\Manual\en\. Section 11.3 carries the Type/Category/Item codes
* used by every setSetting() call below; section 12.3 carries the batch acquisition
* sequence that start()/stop() implement.
*
* Data path is the SimpleArray high-speed callback, NOT the LJXA_ACQ sample wrapper:
* the wrapper busy-waits on a polling loop and mallocs per acquisition, neither of which
* is acceptable inside JobThread::scan(). We register our own callback, convert straight
* into the shared MIL pool, and release JobThread through a condition variable.
*
* Height format is a direct match for Pogo's. SimpleArray stores 16-bit unsigned where
*     height_um = (stored - 32768) * zPitchUm
* and reserves 0 for invalid / dead-zone / judgment-standby points (manual p.40).
* FrameInfo::pHeightMap is 16 + M_UNSIGNED with 0 meaning invalid, so the conversion is a
* straight copy - no clamping window like Profiler_SSZN needs.
*/


class Profiler_Keyence : public IProfiler {
public:
	Profiler_Keyence();
	~Profiler_Keyence();

	//Query
	const double getExposure() const;
	const double getYResolution() const;
	const QString& getFirmwareVersion() const;
	const QString& getSerialNumber() const;
	bool isConnected() const;
	const bool isGrabbing() const;

	//Connection
	bool enable(bool enable);
	bool connect(QString ip);
	bool disconnect();

	//Acquisition
	bool start();
	bool stop();
	bool snapShot();

	//Control
	bool enableIntensityMap(bool enable);
	bool setScanLength(double mm);
	bool setGain(double gain);
	bool setDuoHeadGain(double gain, double gain2);
	bool setDivider(int divider);
	bool setExposureMode(ExposureMode mode);
	bool setExposure(double us);
	bool setMultiExposure(double us, double us2);
	bool setDynamicExposure(double min_us, double max_us);
	bool setParallelExposure(double min_us, double max_us);
	bool waitAcquisition(int ms);
	bool setMSR(bool enable);
	bool setLaserLineThreshold(double threshold);

	//Data
	const FrameInfo& getFrame() const;
	FrameInfo& getFrame();
	void resetFrame();

	//Misc
	bool loadConfig(QString path);
	QString errorMsg();

	//Keyence specific - useful during bring-up, not part of IProfiler
	const double getZPitchUm() const { return m_zPitchUm; }
	const double getMeasuredFovMm() const { return m_measuredFovMm; }

	// dwUser carries the device id, so one static entry point serves every instance.
	static void _cdecl SimpleArrayCallback(LJX8IF_PROFILE_HEADER* pHeaderArray,
		WORD* pHeightArray,
		WORD* pLuminanceArray,
		DWORD dwLuminanceEnable,
		DWORD dwProfileDataCount,
		DWORD dwCount,
		DWORD dwNotify,
		DWORD dwUser);

private:
	//--- identity / connection
	int      m_deviceId = 0;              // 0..5, LJX8 supports up to 6 devices
	QString  m_ip = "";
	// Two DIFFERENT TCP ports, and the manual (p.69) forbids making them equal:
	// "Do not set the command port number and the high-speed port number to the same
	// number." These mirror the controller's own setting items 07h and 08h.
	int      m_commandPort = 24691;       // controller default; LJX8IF_ETHERNET_CONFIG.wPortNo
	int      m_highSpeedPort = 24692;     // controller default; wHighSpeedPortNo
	bool     m_connectionStatus = false;
	bool     m_enable = true;
	std::atomic<bool> m_release = false;

	//--- program / geometry
	unsigned char m_programNo = 0;        // Type byte is 0x10 + programNo
	int      m_divider = 1;               // encoder sub-sampling count
	double   m_yPitchUm = 10.0;           // per-trigger Y pitch BEFORE sub-sampling
	double   m_zPitchUm = 1.6;            // grey-level -> micron factor, from head model
	double   m_measuredFovMm = 0.0;       // lXPitch * wProfileDataCount, logged at PreStart
	int      m_batchCount = 1000;         // profiles per batch == image height
	int      m_xPoints = 0;               // profile data count reported by the controller
	bool     m_luminanceEnabled = true;

	//--- settings mirrored for the getters
	double   m_exposure = 0.0;
	QString  m_headModel = "";
	QString  m_serialNumber = "";
	QString  m_firmwareVersion = "";
	QString  m_errorMsg = "";

	//--- acquisition state
	FrameInfo m_frameInfo;
	std::mutex m_mutex;
	std::condition_variable m_conditionVariable;
	bool     m_softTriggered = false;
	bool     m_highSpeedReady = false;
	bool     m_highSpeedDirty = true;     // batch count changed since last init

	//--- helpers
	bool safeGuard() const;
	bool setSetting(unsigned char type, unsigned char category, unsigned char item,
		const void* data, unsigned int size, const char* what,
		unsigned char target1 = 0);
	bool parseIp(const QString& ip, unsigned char out[4]) const;
	bool initHighSpeed();                 // Finalize -> Close -> Open -> Initialize
	void finalizeHighSpeed();
	void readHeadIdentity();              // model / serial, and the Z factor that follows
	static int  nearestExposureIndex(int us);
	static double zPitchForHead(const QString& model);
};
