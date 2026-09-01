#pragma once
#include "IProfiler.h"
#include <QHash>



class ProfilerManager {

public:


	static ProfilerManager& instance();

	void loadConfig(QString path);

	QList<QString> keys() const;

	//Query
	const double getExposure(QString id) const;
	const double getYResolution(QString id)const;
	const QString& getFirmwareVersion(QString id) const;
	const QString& getSerialNumber(QString id) const;

	bool isConnected(QString id) const;
	const bool isGrabbing(QString id) const;

	/*
	* Y pitch per acquired profile line, in MICRONS - the encoder pitch multiplied by the
	* sub-sampling count the driver actually applied. Returns 0.0 when no profiler object
	* exists, which is legitimate: profiler.json sets the API even offline.
	*
	* Id-less on purpose, matching getAPI()/getInvertX()/getInvertY(): the only caller is
	* ImageManager::rotate_heightMap, which has no profiler id to hand. This exists so the
	* image maths and the driver cannot disagree about the pitch - see the comment there.
	*/
	double getLinePitchUm() const;

	const QString& getAPI() const { return m_sensortype; }
	const bool& getMSR() const { return m_enableMSR; }
	const bool& getInvertX() const { return m_invertX; }
	const bool& getInvertY() const { return m_invertY; }
	const bool& getConnectionStatus() const { return m_connectionStatus; }

	//Connection
	bool enable(QString id, bool enable);
	bool connect(QString id, QString ip);
	bool disconnect(QString id);


	//Acquisition
	bool start(QString id);
	bool stop(QString id);
	bool snapShot(QString id);
	bool startLive(QString id);
	bool stopLive(QString id);
	double liveXFovMm(QString id) const;
	double liveZRangeMm(QString id) const;


	//Control
	bool enableIntensityMap(QString id, bool enable);
	bool setScanLength(QString id, double mm);
	bool setGain(QString id, double gain);
	bool setDuoHeadGain(QString id, double gain, double gain2);
	bool setExposureMode(QString id, IProfiler::ExposureMode mode);
	bool setExposure(QString id, double us);
	bool setMultiExposure(QString id, double us, double us2);
	bool setDynamicExposure(QString id, double min_us, double max_us);
	bool setParallelExposure(QString id, double us, double us2);
	bool waitAcquisition(QString id, int ms);
	bool setMSR(QString id, bool enable);
	bool setLaserLineThreshold(QString id, double threshold);
	bool setDivider(QString id, int divider);

	//Per-optic peak and light controls. Plain pass-throughs on purpose - the backend does its
	//own change detection, because unlike m_currentSettings here it can reset that cache when
	//the controller is reconnected and its Running area has been re-pushed.
	bool setPeakSensitivity(QString id, int level);
	bool setPeakSelection(QString id, int mode);
	bool setLightLimits(QString id, int lower, int upper);

	//Data
	const FrameInfo* getFrame(QString id) const;
	FrameInfo* getFrame(QString id);
	bool resetFrame(QString id);

	//Misc
	bool loadConfig(QString id, QString path);
	QString errorMsg(QString id);

	//Access
	QHash<QString, IProfiler*>& profilers();
	IProfiler* profiler(QString id);
	const IProfiler* profiler(QString id) const;

private:
	ProfilerManager();
	~ProfilerManager();
	ProfilerManager(const ProfilerManager&) = delete;
	ProfilerManager& operator=(const ProfilerManager&) = delete;

	static ProfilerManager m_instance;

	bool m_enable = false;

	QHash<QString, IProfiler*> m_profilers;

	QHash<QString, OpticsInfo3D> m_currentSettings;


	bool create(QString id, QString api);
	bool valid(QString id) const;

	/*
	* Mark every cached setting as "not applied", so the next scan re-pushes all of them.
	*
	* m_currentSettings exists to skip a driver call when the value has not changed, but nothing
	* used to reset it. A reconnect re-runs the backend's loadConfig, which pushes the driver
	* config's own defaults into the controller - while this cache still believed the recipe's
	* values were in force, so the following scan skipped re-sending them and the controller
	* quietly ran on the config defaults.
	*
	* Clearing the hash is NOT enough, which is the subtle part: the cache is an OpticsInfo3D,
	* whose defaults are gain 0.0, lineThreshold 0.0, divider 1 and exposure 0.0 - all of them
	* perfectly ordinary recipe values. A default-constructed entry therefore MATCHES such a
	* recipe and skips the very first push. Hence sentinels no real setting can equal, rather
	* than a reset to defaults.
	*/
	void invalidateSettings(QString id);

	//Error msg
	const QString error_invalid_id = "INVALID_ID";

	QString m_sensortype;
	bool m_enableMSR;
	bool m_connectionStatus;
	bool m_invertX;
	bool m_invertY;
	
};