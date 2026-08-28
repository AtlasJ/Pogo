#pragma once
#include "IProfiler.h"
#include <QString>
#include <QVector>
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
	bool setPeakSensitivity(int level) override;
	bool setPeakSelection(int mode) override;
	bool setLightLimits(int lower, int upper) override;

	//Data
	const FrameInfo& getFrame() const;
	FrameInfo& getFrame();
	void resetFrame();

	//Misc
	bool loadConfig(QString path);
	QString errorMsg();

	//Diagnostics (IProfiler optional overrides) - see the Profiler Scan Test report
	QString diagnostics() const override;
	bool getCounters(quint32& triggerCount, qint32& encoderCount) override;
	bool isSafeToScan(QString& reason) const override;
	bool takeLastRawFrame(mtrx::SharedMilID& height, mtrx::SharedMilID& intensity) override;
	bool getLastBatchSize(int& profiles, int& pointsPerProfile) const override {
		profiles = m_lastProfiles;
		pointsPerProfile = m_lastPoints;
		return true;
	}

	//Keyence specific - useful during bring-up, not part of IProfiler
	double getZPitchUm() const override { return m_zPitchUm; }
	const double getMeasuredFovMm() const { return m_measuredFovMm; }
	const QString& getHeadModel() const { return m_headModel; }

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
	int      m_lastProfiles = 0;          // rows in the last batch the callback actually got
	int      m_lastPoints = 0;            // and its points per profile
	int      m_lastDiscarded = 0;         // profiles that arrived too late to be kept
	bool     m_luminanceEnabled = true;

	//--- what loadConfig last pushed to the controller, kept for diagnostics()
	// loadConfig ANDs 13 setSetting results into one bool, so a single rejection is invisible
	// unless you read the log line by line. Recording them makes the report able to say which.
	struct AppliedSetting {
		QString name;
		int     value = 0;
		bool    ok = false;
	};
	QVector<AppliedSetting> m_appliedSettings;
	QString  m_configPath = "";

	/*
	* Last values pushed for the per-optic peak/light settings, so an unchanged setting costs
	* nothing. Each setSetting is a separate apply cycle on the controller and these four would
	* otherwise add roughly half a second to EVERY scan, for values that almost never change
	* between line scans. -1 means "not pushed yet", so the first scan after a connect always
	* writes. loadConfig() seeds them with what it pushed from keyence.json, which is what makes
	* the cache safe across a controller power cycle: connect re-pushes and re-seeds.
	*/
	/*
	* The controller's own sampling cycle, read back at connect. Pogo does not set this when
	* keyence.json says -1, so the controller keeps whatever Navigator last saved - and that
	* value caps the trigger rate, and therefore the maximum scan speed. Read so the report can
	* state it instead of printing the -1 that only means "we did not send one".
	* -1 here means the read-back itself failed or has not run.
	*
	* Stored RAW. The encoding of item 0x02 is not confirmed on this machine - the name says
	* cycle, which would be a period, but Navigator presents kHz - so nothing converts it.
	*/
	int      m_samplingCycleRunning = -1;

	int      m_appliedPeakSensitivity = -1;
	int      m_appliedPeakSelection = -1;
	int      m_appliedLightLower = -1;
	int      m_appliedLightUpper = -1;

	//--- settings mirrored for the getters
	double   m_exposure = 0.0;
	QString  m_headModel = "";
	QString  m_serialNumber = "";
	QString  m_firmwareVersion = "";
	QString  m_errorMsg = "";

	//--- last raw batch, kept only as shared references so a diagnostic can save the data
	//    as the controller delivered it. Handed over by takeLastRawFrame(), which clears
	//    these so the pool buffers are not pinned for the life of the process.
	mtrx::SharedMilID m_lastRawHeight = nullptr;
	mtrx::SharedMilID m_lastRawIntensity = nullptr;

	//--- acquisition state
	FrameInfo m_frameInfo;
	std::mutex m_mutex;
	std::condition_variable m_conditionVariable;
	bool     m_softTriggered = false;
	bool     m_highSpeedReady = false;
	bool     m_highSpeedDirty = true;     // batch count changed since last init

	//--- helpers
	//Largest batch point this controller will accept, from User's Manual Table A-10: it
	//depends on luminance output and the profile point count, not on one constant.
	int  batchPointMax() const;
	bool safeGuard() const;
	bool setSetting(unsigned char type, unsigned char category, unsigned char item,
		const void* data, unsigned int size, const char* what,
		unsigned char target1 = 0);
	//Read one setting back out of the controller's Running area. Read-only and never fatal:
	//a failure logs and returns false, leaving the caller's buffer untouched.
	bool getSetting(unsigned char type, unsigned char category, unsigned char item,
		void* data, unsigned int size, const char* what,
		unsigned char target1 = 0);
	bool parseIp(const QString& ip, unsigned char out[4]) const;
	bool initHighSpeed();                 // Finalize -> Close -> Open -> Initialize
	void finalizeHighSpeed();
	void readHeadIdentity();              // model / serial, and the Z factor that follows
	static int  nearestExposureIndex(int us);
	static double zPitchForHead(const QString& model);
};
