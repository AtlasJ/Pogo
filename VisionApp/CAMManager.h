#pragma once
#include "ICamera.h"
#include <QHash>

class CAMManager {

public:
	static CAMManager& instance();

	struct LSCInfo {
		int id = 0;
		int triggerSource = 1;
	};

	void loadConfig(QString path);

	QList<QString> keys() const;

	const int getWidth(QString id) const;
	const int getHeight(QString id) const;
	const int getChannel(QString id) const;

	const double getExposure(QString id) const;
	const double getGain(QString id) const;

	//last exposure set through setExposure - saved state, no camera API round trip
	double currentExposure(QString id) const { return m_currentExposure.value(id, 0.0); }

	const QString& getName(QString id) const;
	const QString& getSerialNumber(QString id) const;

	bool isConnected(QString id) const;
	const bool isGrabbing(QString id) const;


	//Connection
	bool enable(QString id, bool enable);
	bool connect(QString id, QString sn);
	bool disconnect(QString id);
	bool reconnect(QString id);


	//Acquisition
	bool startGrab(QString id);
	bool stopGrab(QString id);


	//Control
	bool setExposure(QString id, double exposure);
	bool setGain(QString id, double gain);
	bool softTrigger(QString id);
	bool waitAcquisition(QString id, int ms);
	bool setTriggerOutput(QString id, QString line, QString source);
	
	bool setDO(QString id, int DO, bool on);

	//Misc
	QString errorMsg(QString id);
	bool loadConfig(QString id, QString path);

	//Data
	const FrameInfo* frame(QString id) const;
	FrameInfo* frame(QString id);
	bool resetFrame(QString id);


	//Default
	void setDefaultWidth(int w);
	void setDefaultHeight(int h);
	void setDefaultChannel(int c);

	//Access
	QHash<QString, ICamera*>& cameras();
	ICamera* camera(QString id);
	const ICamera* camera(QString id) const;
	LSCInfo* lsc(QString id); //camID

private:
	CAMManager();
	~CAMManager();
	CAMManager(const CAMManager&) = delete;
	CAMManager& operator=(const CAMManager&) = delete;

	static CAMManager m_instance;

	bool m_enable = false;

	QHash<QString, ICamera*> m_cam;
	QHash<QString, CAMManager::LSCInfo> m_lsc;

	QHash<QString, double> m_currentExposure;
	QHash<QString, double> m_currentGain;

	int m_defaultW = 5120;
	int m_defaultH = 5120;
	int m_defaultChannel = 3;

	bool create(QString id, QString api);
	bool valid(QString index) const;
	bool verifyKey(QString sn, int key) const;
};