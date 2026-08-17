#pragma once
#include <vector>
#include <QJsonObject>
#include <QHash>
#include "QOrderedHash.h"
#include "ILSC.h"
#include "AdvantechDigitalIO.h"
#include "OpticsInfo.h"
#include "PortabilityInfo.h"

class OpticsControl {
private:
	OpticsControl();
	~OpticsControl();
	OpticsControl(const OpticsControl&) = delete;
	OpticsControl& operator=(const OpticsControl&) = delete;

	static OpticsControl m_instance;

	bool m_enableOffset = true;
	PortabilityInfo* m_portabilityInfo = nullptr;

public:
	static OpticsControl& instance();

	void attach(PortabilityInfo* pi);

	bool validChannel(QString id);
	void printBand(QString title, const ct::Band& band);

	void enableOffset(bool enable);

	bool toggleAllChannels(bool toggle);
	bool toggleChannel(QString ch, bool toggle);
	bool toggleBand(const ct::Band& band, bool toggle);
	bool toggleGroupedOptic(QString key, const QHash<QString, QVector<QString>>& groupedOptics, bool toggle);

	QString getBandID(QString opticID, BandType bandType);
	const ct::Band& getBand(const OpticsInfo& optic, BandType bandType);
	QHash<QString, QVector<QString>> getGroupedOptics();

	bool setIntensity(QString ch, int value);
	bool setAllChannels(QString camID, const ct::Band& channels);
	bool setBand(QString camID, const OpticsInfo& optic, BandType bandType);
	bool adjustBand(const OpticsInfo& optic, BandType bandType, int adjustValue);
	bool setGroupedOpticIntensity(QString key, const QHash<QString, QVector<QString>>& groupedOptics, int value);
	bool setBrightness(QString camID, QString ch);
	bool setBrightnessBasedOnBand(QString camID, const OpticsInfo& optic, BandType bandType);
	bool setMaxCurrent(QString ch, double dCurrent);
	bool setGroupedOpticMaxCurrent(QString key, const QHash<QString, QVector<QString>>& groupedOptics, double dCurrent);
};