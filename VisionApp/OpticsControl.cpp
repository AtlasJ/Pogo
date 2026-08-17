#include "OpticsControl.h"
#include "LSCManager.h"
#include "Logger.h"
#include "ScopedTimeLogger.h"
#include "CAMManager.h"

OpticsControl OpticsControl::m_instance;

OpticsControl::OpticsControl()
{
}

OpticsControl::~OpticsControl()
{
}

OpticsControl& OpticsControl::instance()
{
	return m_instance;
}

void OpticsControl::attach(PortabilityInfo* pi)
{
	m_portabilityInfo = pi;
}

bool OpticsControl::validChannel(QString id)
{
	return LSCManager::instance().isChannelValid(id);
}

void OpticsControl::printBand(QString title, const ct::Band& band)
{
	std::string msg = "";

	for (const auto& b : band) {
		msg += std::to_string(b) + ",";
	}

	ct::logger::debug("[OC] %s: %s", title.toStdString().c_str(), msg.c_str());
}

void OpticsControl::enableOffset(bool enable)
{
	m_enableOffset = enable;
}

bool OpticsControl::toggleAllChannels(bool toggle)
{
	bool ret = true;

	for (const auto& id : LSCManager::instance().channels()) {
		ret &= toggleChannel(id, toggle);
	}

	return ret;
}

bool OpticsControl::toggleChannel(QString ch, bool toggle)
{
	if (!validChannel(ch)) return false;

	//_channelToggle[ch]->setCheckState((toggle) ? Qt::Checked : Qt::Unchecked); UNFINISH
	if (LSCManager::instance().toggle(ch, toggle) != (int)LSC_RC::PASS) return false;
	return true;
}

bool OpticsControl::toggleBand(const ct::Band& band, bool toggle)
{
	bool ret = true;
	for (const auto& key : band.keys()) {
		const auto& intensity = band[key];
		if (intensity == 0) continue;
		ret &= toggleChannel(key, toggle);
	}
	return ret;
}

bool OpticsControl::toggleGroupedOptic(QString key, const QHash<QString, QVector<QString>>& groupedOptics, bool toggle)
{
	bool ret = true;
	for (const auto& key : groupedOptics[key]) {
		ret &= toggleChannel(key, toggle);
	}
	return ret;
}

QString OpticsControl::getBandID(QString opticID, BandType bandType)
{
	QString id = opticID;

	if (bandType == BandType::R) id += "_R";
	else if (bandType == BandType::G) id += "_G";
	else if (bandType == BandType::B) id += "_B";
	else if (bandType == BandType::M) id += "_M";

	return id;
}

const ct::Band& OpticsControl::getBand(const OpticsInfo& optic, BandType bandType)
{
	if (bandType == BandType::R) return optic.R;
	else if (bandType == BandType::G) return optic.G;
	else if (bandType == BandType::B) return optic.B;
	return optic.M;
}

QHash<QString, QVector<QString>> OpticsControl::getGroupedOptics()
{
	QHash<QString, QVector<QString>> groupedOptics;

	for (const auto& key : LSCManager::instance().channels().keys()) {

		const auto& ch = LSCManager::instance().channels()[key];

		auto groupkey = key;

		if (!ch.group.isEmpty()) groupkey = ch.group;

		if (groupedOptics.contains(groupkey)) {
			groupedOptics[groupkey].append(key);
		}
		else {
			groupedOptics.insert(groupkey, QVector<QString>());
			groupedOptics[groupkey].append(key);
		}
	}

	return groupedOptics;
}

bool OpticsControl::setIntensity(QString ch, int value)
{
	if (!validChannel(ch)) return false;

	//intensity offset
	auto lci = m_portabilityInfo->lightingCalibrationInfo;

	int offsetedValue = value;

	if (!m_portabilityInfo->lightingCalibrationInfo.is_main && m_enableOffset) {
		
		if (!lci.main_GVTable.contains(ch) || !lci.local_GVTable.contains(ch)) {
			ct::logger::error("GV table invalid access key : % s", ch.toStdString().c_str());
			return false;
		}
		
		if (lci.local_GVTable[ch].size() <= value || lci.main_GVTable[ch].size() <= value) {
			ct::logger::error("GV table invalid index: %d", value);
			return false;
		}
		
		const auto& expectedGV = lci.main_GVTable[ch][value];
		const auto& localGV = lci.local_GVTable[ch][value];
		
		if (abs(expectedGV - localGV) > 3) {
			int closestIndex = 0;
			double minDif = 999;
			
			for (int i = 0; i < lci.local_GVTable[ch].size(); i++) {
				const auto& gv = lci.local_GVTable[ch][i];
				auto dif = abs(expectedGV - gv);
				if (dif < minDif) {
					minDif = dif;
					closestIndex = i;
				}
			}

			offsetedValue = closestIndex;
		}
	}

	//ct::logger::trace("[lsc] Set intensity (%s): %d -> %d", ch.toStdString().c_str(), value, offsetedValue);

	auto ret = LSCManager::instance().setIntensity(ch, offsetedValue);
	
	/*if (isPage(UIPage::LIGHTING)) {
		_channelSliders[ch]->setValue(value);
		_channelLineEdits[ch]->setText(QString::number(value));
	}*/ //UNFINISH

	if (ret != (int)LSC_RC::PASS) return false;
	return true;
}

bool OpticsControl::setAllChannels(QString camID, const ct::Band& channels)
{
	for (const auto& key : channels.keys()) {
		int intensity = channels[key];
		if (intensity != 0) {
			setBrightness(camID, key);
			setIntensity(key, intensity);
			toggleChannel(key, true);
		}
	}

	return true;
}

bool OpticsControl::setBand(QString camID, const OpticsInfo& optic, BandType bandType)
{
	const auto& band = getBand(optic, bandType);

	setBrightnessBasedOnBand(camID, optic, bandType);

	bool ret = true;
	for (const auto& key : band.keys()) {
		const auto& intensity = band[key];
		if (intensity == 0) {
			if (LSCManager::instance().getMode() == lsc::MODE::CONTINUOUS) {
				//ret &= toggleChannel(key, false); //TEMPORARY:
			}
			continue;
		}

		ret &= setIntensity(key, intensity);

		if (LSCManager::instance().getMode() == lsc::MODE::CONTINUOUS) {
			ret &= toggleChannel(key, true);
		}
	}
	return ret;
}

bool OpticsControl::adjustBand(const OpticsInfo& optic, BandType bandType, int adjustValue)
{
	/*
	The logic of adjust band takes the adjustValue and adjust every channel that is not 0. Either increase or decrease based on the value given.
	If one of the channel is less than 0 or over 255. The adjustment will not be carry out and returns false
	*/

	const auto& band = getBand(optic, bandType);

	bool ret = true;

	for (const auto& key : band.keys()) {
		auto intensity = band[key];
		if (intensity == 0) continue;

		intensity += adjustValue;

		if (intensity < 0 || intensity > 255) {
			return false;
		}
	}

	for (const auto& key : band.keys()) {
		auto intensity = band[key];
		if (intensity == 0) continue;

		intensity += adjustValue;
		ret &= setIntensity(key, intensity);
	}

	return ret;
}

bool OpticsControl::setGroupedOpticIntensity(QString key, const QHash<QString, QVector<QString>>& groupedOptics, int value)
{
	bool ret = true;
	for (const auto& key : groupedOptics[key]) {
		ret &= setIntensity(key, value);
	}
	return ret;
}

bool OpticsControl::setBrightness(QString camID, QString ch)
{
	const auto& bs = m_portabilityInfo->lightingCalibrationInfo.brightness;

	ct::logger::trace("Set brightness -> Cam: %s, Channel: %s", camID.toStdString().c_str(), ch.toStdString().c_str());
	if (!bs.contains(ch) && LSCManager::instance().channels().contains(ch)) {
		ch = LSCManager::instance().channels()[ch].group;
	}

	if (!bs.contains(ch)) {
		ct::logger::error("Attempting to adjust brightness on invalid channel: %s", ch.toStdString().c_str());
		return false;
	}

	const auto& b = bs[ch];

	auto ret = CAMManager::instance().setExposure(camID, b.exposure);
	ret &= CAMManager::instance().setGain(camID, b.gain);

	if (ret) ct::logger::info("Based on: %s, Set %s exposure: %d, gain: %d", ch.toStdString().c_str(), camID.toStdString().c_str(), b.exposure, b.gain);

	return ret;
}

bool OpticsControl::setBrightnessBasedOnBand(QString camID, const OpticsInfo& optic, BandType bandType)
{
	bool ret = true;

	const auto& band = getBand(optic, bandType);

	auto keys = band.keys();
	qSort(keys);

	for (const auto& key : keys) {
		const auto& intensity = band[key];
		if (intensity == 0) continue;

		ret &= setBrightness(camID, key);
		break;
	}

	return ret;
}

bool OpticsControl::setMaxCurrent(QString ch, double dCurrent)
{
	if (!validChannel(ch)) return false;
	if (LSCManager::instance().setMaxCurrent(ch, dCurrent) != (int)LSC_RC::PASS) return false;
	return true;
}

bool OpticsControl::setGroupedOpticMaxCurrent(QString key, const QHash<QString, QVector<QString>>& groupedOptics, double dCurrent)
{
	bool ret = true;
	for (const auto& ch : groupedOptics[key]) {
		ret &= setMaxCurrent(ch, dCurrent);
	}
	return ret;
}

