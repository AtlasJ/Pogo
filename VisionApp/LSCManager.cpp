#include "LSCManager.h"
#include "QJsonHelper.h"
#include "Logger.h"

#include "LSC_VLP.h"
#include "LSC_OPT.h"
#include "LSC_CST_MVL.h"
#include "LSC_VIE.h"
#include "ScopedTimeLogger.h"

LSCManager LSCManager::m_instance;

bool LSCManager::createLSC(QString id)
{
	if (id == "VLP") {
		auto* lsc = new LSC_VLP;
		m_lsc.push_back(lsc);
	}
	else if (id == "OPT") {
		auto* lsc = new LSC_OPT;
		m_lsc.push_back(lsc);
	}
	else if (id == "CST_MVL") {
		auto* lsc = new LSC_CST_MVL;
		m_lsc.push_back(lsc);
	}
	else if (id == "VIE") {
		auto* lsc = new LSC_VIE;
		m_lsc.push_back(lsc);
	}
	else {
		return false;
	}

	ct::logger::info("Created LSC: %s", id.toStdString().c_str());
	return true;
}

bool LSCManager::validLSC(int index) const
{
	if ((m_lsc.size() > index && index >= 0)) {
		return (m_lsc[index] != nullptr);
	}
	return false;
}

bool LSCManager::validIO() const
{
	return (m_io != nullptr);
}

LSCManager::LSCManager()
{
}

LSCManager::~LSCManager()
{
}

LSCManager & LSCManager::instance()
{
	return m_instance;
}

void LSCManager::loadConfig(QJsonObject obj)
{
	m_enable = jsonHelper::getBool(obj, "enable");
	if (!m_enable) {
		ct::logger::warn("[LSC][strobe] LSC manager DISABLED by config (top-level \"enable\" in lsc.json is false/missing) - "
			"ALL lighting calls (mode, intensity, strobe pulse) will be silent no-ops");
	}
	m_connectionTimeout = jsonHelper::getInteger(obj, "connection_timeout(ms)");
	m_responseTimeout = jsonHelper::getInteger(obj, "response_timeout(ms)");

	if (obj.contains("LSCs")) {
		auto LSCs = obj["LSCs"].toArray();

		for (auto lscDoc : LSCs) {
			auto lscObj = lscDoc.toObject();

			auto id = jsonHelper::getString(lscObj, "id");

			if (createLSC(id)) {
				auto name = jsonHelper::getString(lscObj, "name");
				auto ip = jsonHelper::getString(lscObj, "ip");
				auto port = jsonHelper::getInteger(lscObj, "port");
				auto enable = jsonHelper::getBool(lscObj, "enable", false);
				if (!enable) ct::logger::warn("[LSC][strobe] Controller '%s' DISABLED by its \"enable\" flag in lsc.json", id.toStdString().c_str());

				auto lsc = m_lsc.back();
				lsc->id() = id.toStdString();
				lsc->name() = name.toStdString();
				lsc->setIP(ip.toStdString());
				lsc->setPort(port);
				lsc->enable(enable);

				QString lscTriggerType;

				if (lscObj.contains("channels")) {
					auto channels = lscObj["channels"].toArray();

					if (id == "VLP") { //VLP does not provide their own API to check channels
						lsc->numChannel() = channels.size();
					}

					int channel_index = 0;
					for (auto chDoc : channels) {
						auto chObj = chDoc.toObject();
			
						Channel channel;
						channel.id = jsonHelper::getString(chObj, "id");
						channel.name = jsonHelper::getString(chObj, "name");
						ct::logger::info("Channel: %s", channel.name.toStdString().c_str());
						channel.camID = jsonHelper::getString(chObj, "camID");
						channel.optic = jsonHelper::getString(chObj, "optic");
						channel.group = jsonHelper::getString(chObj, "group");
						channel.trigger_type = jsonHelper::getString(chObj, "trigger_type");
						if (lscTriggerType.isEmpty()) lscTriggerType = channel.trigger_type;
						channel.lighting_type = jsonHelper::getString(chObj, "lighting_type");
						channel.io_bit = jsonHelper::getInteger(chObj, "io_bit");
						channel.io_port = jsonHelper::getInteger(chObj, "io_port");
						channel.lsc_index = m_lsc.size() - 1;
						channel.channel_index = jsonHelper::getInteger(chObj, "channel_index");

						m_channels.insert(channel.id, channel);
						m_currentIntensity.insert(channel.id, -1);

						channel_index++;
					}
				}

				if (id == "CST_MVL") {
					//strobe trigger source follows the channel trigger_type:
					//LAN = internal trigger, IO = external trigger (Y110 pulses per snap)
					m_strobeInternalTrigger = (lscTriggerType == "LAN");

					//internal cycle is fixed in code: pulse = cycle gives steady output, so
					//the exact period does not matter and never tracks the exposure
					m_strobeInternalCycleUs = m_strobeInternalTrigger ? 10000 : 0; //10 ms period, 100% duty
					static_cast<LSC_CST_MVL*>(lsc)->setStrobeConfig(
						m_strobeInternalTrigger,
						m_strobeInternalCycleUs);
				}
			}
			else {
				ct::logger::error("Failed to create LSC: %s", id.toStdString().c_str());
			}
		}
	}
}

int LSCManager::connect()
{
	if (!m_enable) return (int)LSC_RC::PASS;

	int ret = 0;
	for (auto& lsc : m_lsc) {
		auto rc = lsc->connect();

		if (rc != 0) ct::logger::error("Failed to connect LSC %s: %d", lsc->name().c_str(), ret);
		if (ret == 0) ret = rc;
	}
	return ret;
}

int LSCManager::disconnect()
{
	if (!m_enable) return (int)LSC_RC::PASS;

	int ret = 0;
	for (auto& lsc : m_lsc) {
		auto rc = lsc->disconnect();

		if (rc != 0) ct::logger::error("Failed to disconnect LSC %s: %d", lsc->name().c_str(), rc);
		if (ret == 0) ret = rc;
	}
	return ret;
}

int LSCManager::reconnect()
{
	if (!m_enable) return (int)LSC_RC::PASS;

	int ret = 0;
	for (auto& lsc : m_lsc) {
		auto rc = lsc->reconnect();

		if (rc != 0) ct::logger::error("Failed to reconnect LSC %s: %d", lsc->name().c_str(), ret);
		if (ret == 0) ret = rc;
	}
	return ret;
}

bool LSCManager::isConnected() const
{
	if (!m_enable) return (int)LSC_RC::PASS;

	bool isConnect;
	for (auto& lsc : m_lsc) {
		isConnect &= lsc->isConnected();
		if (!lsc->isConnected()) ct::logger::error("LSC %s is not connected", lsc->name().c_str());
	}

	return isConnect;
}

void LSCManager::resetLatch()
{
	for (auto& c : m_currentIntensity) {
		c = -1;
	}
}

int LSCManager::numChannel() const
{
	return m_channels.size();
}

bool LSCManager::isChannelValid(QString ch) const 
{
	return m_channels.contains(ch);
}

const QOrderedHash<QString, LSCManager::Channel>& LSCManager::channels() const
{
	return m_channels;
}

LSCManager::Channel& LSCManager::channel(QString id)
{
	return m_channels[id];
}

const LSCManager::Channel& LSCManager::channel(QString id) const
{
	return m_channels[id];
}

int LSCManager::toggle(QString ch, bool on)
{
	ScopedTimeLogger stl("[LSC] toggle light");

	if (!m_enable) return (int)LSC_RC::PASS;

	if (!isChannelValid(ch)) return (int)LSC_RC::FAIL;

	int ret = 0;
	auto& channel = m_channels[ch];
	auto lsc_index = channel.lsc_index;
	auto ch_index = channel.channel_index;
	auto bit = channel.io_bit;
	auto port = channel.io_port;

	if (!validLSC(lsc_index)) return (int)LSC_RC::FAIL;
	
	if (channel.trigger_type == "LAN") {
		ret = m_lsc[lsc_index]->toggle(ch_index, on);

		//the CST toggle is EMULATED via the intensity register (OFF writes 0, ON
		//restores the driver's cached value), so this cache goes stale: OFF leaves it
		//claiming the old intensity and the next setIntensity latches out, keeping the
		//light dark. Track OFF as 0 and ON as unknown so the next write goes through.
		//With EXTERNAL strobe the toggle is a no-op (light fires per Y110 pulse) and the
		//cache stays; continuous and INTERNAL strobe really write 0/restore.
		if (ret == (int)LSC_RC::PASS && (m_mode != lsc::MODE::TRIGGER || m_strobeInternalTrigger))
			m_currentIntensity[ch] = on ? -1 : 0;
	}
	else if (channel.trigger_type == "IO") {
		if (!validIO()) return (int)LSC_RC::FAIL;
		m_io->writeBit(port, bit, !on); //need low to trigger on
	}

	return ret;
}

int LSCManager::setIntensity(QString ch, int intensity)
{
	std::string title = "[LSC] Set intensity " + ch.toStdString() + ": " + std::to_string(intensity);
	ScopedTimeLogger stl(title);

	if (!m_enable) return (int)LSC_RC::PASS;

	if (!isChannelValid(ch)) return (int)LSC_RC::FAIL;

	if (m_currentIntensity[ch] == intensity) {
		ct::logger::trace("[LSC][strobe] intensity %s=%d already latched (mode %s) - skipped",
			ch.toStdString().c_str(), intensity, m_mode == lsc::MODE::TRIGGER ? "STROBE" : "CONTINUOUS");
		return (int)LSC_RC::PASS;
	}

	ct::logger::debug("[LSC][strobe] setIntensity %s=%d writing to %s register",
		ch.toStdString().c_str(), intensity, m_mode == lsc::MODE::TRIGGER ? "STROBE" : "CONTINUOUS");

	int ret = 0;
	auto& channel = m_channels[ch];
	auto lsc_index = channel.lsc_index;
	auto ch_index = channel.channel_index;

	if (!validLSC(lsc_index)) return (int)LSC_RC::FAIL;

	ret = m_lsc[lsc_index]->setIntensity(ch_index, intensity);
	if (ret == (int)LSC_RC::PASS) m_currentIntensity[ch] = intensity;
	if (ret == (int)LSC_RC::FAIL) {
		ct::logger::error("[LSC] Failed to set intensity: %s, %d", ch.toStdString().c_str(), intensity);
	}

	return ret;
}

int LSCManager::setMultiIntensity(const QHash<QString, int>& intensityMap)
{
	ScopedTimeLogger stl("[LSC] Set multi intensity");

	int ret = (int)LSC_RC::PASS;
	if (!m_enable) return ret;

	bool isSet = true;
	for (auto key : intensityMap.keys()) {
		if (m_currentIntensity[key] != intensityMap[key]) {
			isSet = false;
		}
	}

	if (isSet) {
		ct::logger::trace("[LSC][strobe] multi intensity already latched (mode %s) - skipped",
			m_mode == lsc::MODE::TRIGGER ? "STROBE" : "CONTINUOUS");
		return ret;
	}

	{
		QStringList vals;
		for (auto key : intensityMap.keys()) vals << QString("%1=%2").arg(key).arg(intensityMap[key]);
		ct::logger::info("[LSC][strobe] setMultiIntensity (%s register): %s",
			m_mode == lsc::MODE::TRIGGER ? "STROBE" : "CONTINUOUS",
			vals.join(", ").toStdString().c_str());
	}

	//safe guard
	QVector<QVector<ChannelSetting>> LSCs;
	LSCs.resize(m_lsc.size());
	for (auto key : intensityMap.keys()) {
		if (!isChannelValid(key)) {
			ct::logger::error("[LSC] Failed to set multi intensity. Invalid channel ID: %s", key.toStdString().c_str());
			return (int)LSC_RC::FAIL;
		}

		int ret = 0;
		auto& channel = m_channels[key];
		auto lsc_index = channel.lsc_index;
		auto ch_index = channel.channel_index;

		if (!validLSC(lsc_index)) {
			ct::logger::error("[LSC] Failed to set multi intensity. Invalid LSC index: %d", lsc_index);
			return (int)LSC_RC::FAIL;
		}

		ChannelSetting cs;
		cs.id = key;
		cs.intensity = intensityMap[key];
		cs.channelIndex = ch_index;

		LSCs[lsc_index].append(cs);
	}

	//Allocate based on each lsc
	lsc::IntensityData** ptrData = new lsc::IntensityData*[LSCs.size()];
	for (int i = 0; i < LSCs.size(); i++) {
		ptrData[i] = new lsc::IntensityData[LSCs[i].size()];

		for (int j = 0; j < LSCs[i].size(); j++) {
			ptrData[i][j].channelIndex = LSCs[i][j].channelIndex;
			ptrData[i][j].intensity = LSCs[i][j].intensity;
		}

		ret = m_lsc[i]->setMultiIntensity(ptrData[i], LSCs[i].size());
		if (ret == (int)LSC_RC::FAIL) {
			ct::logger::error("[LSC] Failed to set multi intensity");
			return ret;
		}
	}

	for (auto key : intensityMap.keys()) {
		m_currentIntensity[key] = intensityMap[key];
	}

	return ret;
}

int LSCManager::getIntensity(QString ch, int & intensity)
{
	if (!m_enable) return (int)LSC_RC::PASS;

	if (!isChannelValid(ch)) return (int)LSC_RC::FAIL;

	int ret = 0;
	auto& channel = m_channels[ch];
	auto lsc_index = channel.lsc_index;
	auto ch_index = channel.channel_index;

	if (!validLSC(lsc_index)) return (int)LSC_RC::FAIL;

	ret = m_lsc[lsc_index]->getIntensity(ch_index, intensity);

	return ret;
}

int LSCManager::setMode(lsc::MODE mode)
{
	ScopedTimeLogger stl("[LSC] Set mode");

	ct::logger::info("[LSC][strobe] setMode requested: %s -> %s (enable=%d, lsc count=%d)",
		m_mode == lsc::MODE::TRIGGER ? "STROBE" : "CONTINUOUS",
		mode == lsc::MODE::TRIGGER ? "STROBE" : "CONTINUOUS",
		m_enable ? 1 : 0, (int)m_lsc.size());

	if (!m_enable) {
		ct::logger::warn("[LSC][strobe] setMode SKIPPED - LSC manager disabled");
		return (int)LSC_RC::PASS;
	}

	int ret = 0;

	for (auto lsc : m_lsc) {
		ret = lsc->setMode(mode);
		ct::logger::info("[LSC][strobe] setMode on controller returned %s",
			ret == (int)LSC_RC::PASS ? "PASS" : "FAIL");
		if (ret == (int)LSC_RC::PASS) m_mode = mode;
	}

	//continuous and strobe intensities live in different controller registers,
	//so the cached values no longer reflect the active register after a mode
	//change - reset so the next setIntensity writes through
	resetLatch();
	m_currentPulseWidth = -1;
	ct::logger::info("[LSC][strobe] intensity latch + pulse width reset after mode change (mode now %s)",
		m_mode == lsc::MODE::TRIGGER ? "STROBE" : "CONTINUOUS");

	/*
	* INTERNAL trigger fix: free-running flashes are unsynchronized with the camera, so
	* an exposure shorter than the cycle catches a random slice of it (randomly dark
	* frames). Pulse width = cycle makes the duty 100% - the flashes butt together and
	* the light is effectively steady, consistent for ANY exposure. Mind the intensity:
	* 100% duty means continuous-rated drive, not strobe overdrive.
	*/
	if (m_mode == lsc::MODE::TRIGGER && m_strobeInternalTrigger && m_strobeInternalCycleUs > 0) {
		for (const auto& channel : m_channels.values()) {
			if (!validLSC(channel.lsc_index)) continue;
			m_lsc[channel.lsc_index]->setTriggerDuration(channel.channel_index, m_strobeInternalCycleUs);
		}
		m_currentPulseWidth = m_strobeInternalCycleUs;
		ct::logger::info("[LSC][strobe] internal trigger: pulse = cycle (%d us) -> steady output for any exposure",
			m_strobeInternalCycleUs);
	}

	return ret;
}

int LSCManager::setStrobePulseWidth(int us)
{
	if (!m_enable) {
		ct::logger::debug("[LSC][strobe] pulse width %d us SKIPPED - manager disabled", us);
		return (int)LSC_RC::PASS;
	}
	if (m_mode != lsc::MODE::TRIGGER) {
		ct::logger::debug("[LSC][strobe] pulse width %d us SKIPPED - mode is CONTINUOUS, not strobe", us);
		return (int)LSC_RC::PASS;
	}
	if (us <= 0) {
		ct::logger::warn("[LSC][strobe] pulse width SKIPPED - invalid value %d us (camera exposure not known?)", us);
		return (int)LSC_RC::PASS;
	}
	if (m_currentPulseWidth == us) {
		ct::logger::trace("[LSC][strobe] pulse width %d us already latched - skipped", us);
		return (int)LSC_RC::PASS;
	}

	int ret = (int)LSC_RC::PASS;

	for (const auto& channel : m_channels.values()) {
		if (!validLSC(channel.lsc_index)) {
			ct::logger::warn("[LSC][strobe] pulse width: channel %s has invalid LSC index %d - skipped",
				channel.id.toStdString().c_str(), channel.lsc_index);
			continue;
		}
		const int r = m_lsc[channel.lsc_index]->setTriggerDuration(channel.channel_index, us);
		ct::logger::info("[LSC][strobe] pulse width %d us -> channel %s (ch idx %d): %s",
			us, channel.id.toStdString().c_str(), channel.channel_index,
			r == (int)LSC_RC::PASS ? "PASS" : "FAIL");
		ret = r;
	}

	m_currentPulseWidth = us;
	ct::logger::info("[LSC] Strobe pulse width set to %d (camera exposure)", us);

	return ret;
}

lsc::MODE LSCManager::getMode()
{
	return m_mode;
}

int LSCManager::setTriggerDuration(QString ch, int us)
{
	ScopedTimeLogger stl("[LSC] Set trigger duration");

	if (!m_enable) return (int)LSC_RC::PASS;

	if (!isChannelValid(ch)) return (int)LSC_RC::FAIL;

	int ret = 0;
	auto& channel = m_channels[ch];
	auto lsc_index = channel.lsc_index;
	auto ch_index = channel.channel_index;

	if (!validLSC(lsc_index)) return (int)LSC_RC::FAIL;

	ret = m_lsc[lsc_index]->setTriggerDuration(ch_index, us);

	return ret;
}

int LSCManager::setTriggerSequence(const QVector<LSCManager::SequenceData>& datas)
{
	ct::logger::info("Set trigger sequence data size: %d", datas.size());
	ScopedTimeLogger stl("[LSC] Set trigger sequence");

	int ret = (int)LSC_RC::PASS;

	if (!m_enable) return ret;

	if (datas.size() == 0) return (int)LSC_RC::FAIL;
	
	//TODO: Only tested with OPT, need to 
	// 
	// 
	// 
	// if this format works for other LSC
	std::vector<lsc::SequenceData> sdatas;
	int lsc_index = 0;

	for (auto& data : datas) {

		lsc::SequenceData sdata;
		sdata.triggerSource = data.triggerSource;
		sdata.exposure_us = data.exposure_us;
	
		for (auto& ch : data.band.keys()) {

			if (!isChannelValid(ch)) return (int)LSC_RC::FAIL;

			auto& channel = m_channels[ch];
			lsc_index = channel.lsc_index; //This is currently global, need to handle when multiple lsc
			auto ch_index = channel.channel_index;

			if (!validLSC(lsc_index)) return (int)LSC_RC::FAIL;

			//TODO: Only this code is not develop to support multi LSC, need to support this
			lsc::IntensityData intensityData;
			intensityData.channelIndex = ch_index;
			intensityData.intensity = data.band[ch];
			sdata.intensityDatas.push_back(intensityData);
		}

		sdatas.push_back(sdata);
	}

	m_lsc[lsc_index]->setTriggerSequence(sdatas);

	return ret;
}

std::string LSCManager::codeString(int returnCode)
{
	return std::string();
}

void LSCManager::attachIO(AdvantechDigitalIO * io)
{
	m_io = io;
}

int LSCManager::setMaxCurrent(QString ch, double dCurrent)
{
	std::string title = "[LSC] Set Max Current " + ch.toStdString() + ": " + std::to_string(dCurrent);
	ScopedTimeLogger stl(title);

	if (!m_enable) return (int)LSC_RC::PASS;

	if (!isChannelValid(ch)) return (int)LSC_RC::FAIL;

	if (m_maxCurrentLimit[ch] == dCurrent) return (int)LSC_RC::PASS;

	int ret = 0;
	auto& channel = m_channels[ch];
	auto lsc_index = channel.lsc_index;
	auto ch_index = channel.channel_index;


	if (!validLSC(lsc_index)) return (int)LSC_RC::FAIL;

	ret = m_lsc[lsc_index]->setMaxCurrent(ch_index, dCurrent);

	if (ret == (int)LSC_RC::PASS) {
		m_maxCurrentLimit[ch] = dCurrent;
	}
	else {
		ct::logger::error("[LSC] Failed to set Max Current: %s, %.2f", ch.toStdString().c_str(), dCurrent);
	}

	return ret;
}