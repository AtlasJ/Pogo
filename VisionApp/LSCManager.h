#pragma once
#include <vector>
#include <QJsonObject>
#include <QHash>
#include "QOrderedHash.h"
#include "ILSC.h"
#include "AdvantechDigitalIO.h"

#include "OpticsInfo.h"

class LSCManager {
private:
	LSCManager();
	~LSCManager();
	LSCManager(const LSCManager&) = delete;
	LSCManager& operator=(const LSCManager&) = delete;

	static LSCManager m_instance;

	bool m_enable = false;
	int m_connectionTimeout = 3000;
	int m_responseTimeout = 1000;
	lsc::MODE m_mode = lsc::MODE::CONTINUOUS;
	int m_currentPulseWidth = -1;
	bool m_strobeInternalTrigger = true; //from the channel trigger_type in lsc.json
	int m_strobeInternalCycleUs = 0;     //strobe_internal_cycle from lsc.json

	struct Channel {
		QString id;
		QString name;
		QString optic;
		QString group;
		QString trigger_type;
		QString lighting_type;
		QString camID;
		int io_bit;
		int io_port;
		int lsc_index;
		int channel_index;
	};

	struct ChannelSetting {
		QString id;
		int channelIndex;
		int intensity;
	};

	std::vector<int> m_lastIntensity;
	std::vector<ILSC*> m_lsc;
	QOrderedHash<QString, Channel> m_channels;
	QHash<QString, int> m_currentIntensity;

	bool createLSC(QString id);
	//bool validChannel(int index) const;
	bool validLSC(int index) const;
	bool validIO() const;

	//3rd party
	AdvantechDigitalIO* m_io = nullptr;

	std::map<QString, double> m_maxCurrentLimit;

public:
	static LSCManager& instance();

	struct SequenceData {
		int triggerSource = 1;
		int exposure_us = 5000;
		ct::Band band;
	};

	void loadConfig(QJsonObject obj);

	int connect();
	int disconnect();
	int reconnect();
	bool isConnected() const;
	void resetLatch();

	int numChannel() const;
	bool isChannelValid(QString ch) const;
	const QOrderedHash<QString, Channel>& channels() const;
	Channel& channel(QString id);
	const Channel& channel(QString id) const;

	int toggle(QString ch, bool on);
	int setIntensity(QString ch, int intensity);
	int setMultiIntensity(const QHash<QString, int>& intensityMap);
	int getIntensity(QString ch, int& intensity);

	int setMode(lsc::MODE mode);
	lsc::MODE getMode();
	int setTriggerDuration(QString ch, int us);
	int setStrobePulseWidth(int us); //all channels, latched, no-op unless in strobe mode
	bool strobeInternalTrigger() const { return m_strobeInternalTrigger; } //LAN = internal, IO = external (Y110)
	int strobePulseWidthUs() const { return m_currentPulseWidth; } //last latched pulse width, -1 = none

	int setTriggerSequence(const QVector<LSCManager::SequenceData>& datas);

	std::string codeString(int returnCode);

	void attachIO(AdvantechDigitalIO* io);

	int setMaxCurrent(QString ch, double dCurrent);
};