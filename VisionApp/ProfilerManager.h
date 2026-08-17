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

	//Error msg
	const QString error_invalid_id = "INVALID_ID";

	QString m_sensortype;
	bool m_enableMSR;
	bool m_connectionStatus;
	bool m_invertX;
	bool m_invertY;
	
};