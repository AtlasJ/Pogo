#include "LSC_VIE.h"
#include "Logger.h"
#include <set>
#include <QHash>
#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QtGlobal>
#include <QVector>

#include "CommonDir.h"
#include "nvsutil/nvs_common.h"

#define VIE_RUN_MODE 1
#define VIE_CONFIG_MODE 2
#define VIE_CONTINUOUS_MODE 1
#define VIE_SEQUENCE_MODE 2
#define VIE_PULSEWIDTH_MODE 3
#define VIE_PULSE_MODE 4
#define VIE_ON 1
#define VIE_OFF 0

namespace {
	constexpr double VIE_DEFAULT_MAX_CURRENT = 2.0;
	constexpr int VIE_APP_MIN_INTENSITY = 0;
	constexpr int VIE_APP_MAX_INTENSITY = 255;
}

std::string LSC_VIE::getStringRetCode(VI32 ret)
{
	if (ret == VT_SUCCESS) return "Success";
	else if (ret == VERR_TIMEOUT) return "Response time out";
	else if (ret == VERR_NO_BOARD) return "No board detected";
	else if (ret == VERR_INVALID_PARAM) return "Invalid parameter being passed to function";
	else if (ret == VERR_BOARD_ID_OVERFLOW) return "Exceed maximum supported board";
	else if (ret == VERR_API_NOT_AVAILABLE) return "API not available";
	else if (ret == VERR_API_NOT_SUPPORTED) return "API not supported";
	else if (ret == VERR_COM_FAILED) return "Serial communication failed";
	else if (ret == VERR_SYSTEM_NOT_INITIALED) return "System not initialized";
	else if (ret == VERR_BOARD_NOT_CONNECTED) return "Board not connected";
	else if (ret == VERR_BOARD_CONNECTED) return "Board already connected";
	else if (ret == VERR_DATA_INVALID) return "Invalid data received from board";
	else if (ret == VERR_SYSTEM_PROGRAM) return "System error from board";
	else if (ret == VERR_OVER_PRODUCT_SPEC) return "Setting beyond specification";
	return "Unknown error code";
}

QString LSC_VIE::maxCurrentJsonPath() const
{
	return QStringLiteral("%1vie_lsc_max_current.json").arg(Common::Directory::ConfigPath());
}

bool LSC_VIE::isValidChannelIndex(int ch) const
{
	return ch >= 0 && ch < static_cast<int>(m_maxCurrent.size());
}

double LSC_VIE::maxCurrentFor(int ch) const
{
	if (isValidChannelIndex(ch) && m_hasMaxCurrent[ch] && m_maxCurrent[ch] > 0.0) {
		return m_maxCurrent[ch];
	}

	return VIE_DEFAULT_MAX_CURRENT;
}

double LSC_VIE::intensityToCurrent(int ch, int intensity) const
{
	const int clampedIntensity = qBound(VIE_APP_MIN_INTENSITY, intensity, VIE_APP_MAX_INTENSITY);
	return maxCurrentFor(ch) * static_cast<double>(clampedIntensity) / static_cast<double>(VIE_APP_MAX_INTENSITY);
}

int LSC_VIE::currentToIntensity(int ch, double current) const
{
	const double maxCurrent = maxCurrentFor(ch);
	if (maxCurrent <= 0.0) return 0;

	const int intensity = qRound(current / maxCurrent * static_cast<double>(VIE_APP_MAX_INTENSITY));
	return qBound(VIE_APP_MIN_INTENSITY, intensity, VIE_APP_MAX_INTENSITY);
}

void LSC_VIE::loadMaxCurrentJson()
{
	for (std::size_t i = 0; i < m_hasMaxCurrent.size(); ++i) {
		m_hasMaxCurrent[i] = false;
		m_maxCurrent[i] = VIE_DEFAULT_MAX_CURRENT;
	}

	QFile file(maxCurrentJsonPath());
	if (!file.exists()) {
		ct::logger::info("[LSC_VIE] Max current JSON does not exist: %s", maxCurrentJsonPath().toStdString().c_str());
		return;
	}

	if (!file.open(QIODevice::ReadOnly)) {
		ct::logger::error("[LSC_VIE] Failed to open max current JSON: %s", maxCurrentJsonPath().toStdString().c_str());
		return;
	}

	QJsonParseError parseError;
	const auto doc = QJsonDocument::fromJson(file.readAll(), &parseError);
	if (parseError.error != QJsonParseError::NoError || !doc.isObject()) {
		ct::logger::error("[LSC_VIE] Failed to parse max current JSON: %s", parseError.errorString().toStdString().c_str());
		return;
	}

	const auto root = doc.object();
	const auto channels = root.value(QStringLiteral("channels")).toArray();
	for (const auto& channelDoc : channels) {
		const auto channelObj = channelDoc.toObject();
		const int ch = channelObj.value(QStringLiteral("channel_index")).toInt(-1);
		const double current = channelObj.value(QStringLiteral("max_current")).toDouble(0.0);

		if (!isValidChannelIndex(ch) || current <= 0.0) {
			ct::logger::warn("[LSC_VIE] Ignoring invalid max current entry. Channel: %d, Current: %.4f", ch, current);
			continue;
		}

		m_maxCurrent[ch] = current;
		m_hasMaxCurrent[ch] = true;
	}
}

bool LSC_VIE::saveMaxCurrentJson() const
{
	QDir().mkpath(Common::Directory::ConfigPath());

	QJsonArray channels;
	for (int ch = 0; ch < static_cast<int>(m_maxCurrent.size()); ++ch) {
		if (!m_hasMaxCurrent[ch]) continue;

		QJsonObject channelObj;
		channelObj.insert(QStringLiteral("channel_index"), ch);
		channelObj.insert(QStringLiteral("max_current"), m_maxCurrent[ch]);
		channels.append(channelObj);
	}

	QJsonObject root;
	root.insert(QStringLiteral("channels"), channels);

	QFile file(maxCurrentJsonPath());
	if (!file.open(QIODevice::WriteOnly)) {
		ct::logger::error("[LSC_VIE] Failed to save max current JSON: %s", maxCurrentJsonPath().toStdString().c_str());
		return false;
	}

	file.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
	return true;
}

int LSC_VIE::applySavedMaxCurrent()
{
	const int channelCount = qMin(m_numChannel, static_cast<int>(m_maxCurrent.size()));
	for (int ch = 0; ch < channelCount; ++ch) {
		if (!m_hasMaxCurrent[ch]) continue;

		auto ret = m_controller.SetMaxCurrent(ch, m_maxCurrent[ch]);
		if (ret != VT_SUCCESS) {
			ct::logger::error("[LSC_VIE] Failed to apply saved max current for channel%d: %s", ch, getStringRetCode(ret).c_str());
			return (int)LSC_RC::FAIL;
		}

		ct::logger::info("[LSC_VIE] Applied saved max current. Channel: %d, Current: %.4f", ch, m_maxCurrent[ch]);
	}

	return (int)LSC_RC::PASS;
}

int LSC_VIE::setContinuousMode()
{
	int operationMode = VIE_CONFIG_MODE;
	int lightMode = VIE_CONTINUOUS_MODE;

	auto ret = m_controller.SetOperationMode(operationMode);
	if (ret != VT_SUCCESS) {
		ct::logger::error("[LSC_VIE] Failed to enter config mode: %s", getStringRetCode(ret).c_str());
		return (int)LSC_RC::FAIL;
	}

	//Release all existing groups
	int totalGroup = 0;
	std::vector<DWORD> groups(8);

	m_controller.GetChannelGrouping(&totalGroup, groups.data());

	for (int i = 0; i < totalGroup; i++) {
		m_controller.ReleaseChannelGrouping(groups[i]);
	}

	//Set default groups to continuous mode
	m_controller.GetChannelGrouping(&totalGroup, groups.data());

	for (int i = 0; i < totalGroup; i++) {
		m_controller.ReleaseChannelGrouping(groups[i]);
		m_controller.SetLightModeByGroup(groups[i], lightMode);
		//m_controller.SetContinuousChannelGroupOn(groups[i], i);
	}

	return (int)LSC_RC::PASS;
}

int LSC_VIE::setTriggerMode()
{
	int operationMode = VIE_CONFIG_MODE;

	auto ret = m_controller.SetOperationMode(operationMode);
	if (ret != VT_SUCCESS) {
		ct::logger::error("[LSC_VIE] Failed to enter config mode: %s", getStringRetCode(ret).c_str());
		return (int)LSC_RC::FAIL;
	}

	int exposure_us = 100000; //always set to max, the lsc will have issue if trigger signal is longer than output width limit

	//Get all channels
	std::set<int> channels;
	for (const auto& data : m_sequenceDatas) {
		for (const auto& intensities : data.intensityDatas) {
			ct::logger::info("Channel index: %d", intensities.channelIndex);
			auto channelIndex = intensities.channelIndex + 1;
			channels.insert(channelIndex);
		}
	}

	auto groupID = getGroupID(channels);
	ct::logger::info("Group ID: %lu", groupID);
	m_controller.SetChannelGrouping(groupID);
	m_controller.SetLightModeByGroup(groupID, VIE_PULSE_MODE);

	int totalSettings = m_sequenceDatas.size();
	if (totalSettings <= 0) return (int)LSC_RC::FAIL;

	QVector<int> settingIDs(totalSettings);
	QVector<int> activeFlags(totalSettings);

	QHash<int, QVector<double>> channelSettings;

	int settingID = 0;

	for (const auto& data : m_sequenceDatas) {
		for (const auto& intensities : data.intensityDatas) {
			double current = intensityToCurrent(intensities.channelIndex, intensities.intensity);

			ct::logger::info("Channel index: %d, SettingID: %d, Current: %.4f", intensities.channelIndex, settingID, current);

			auto& currents = channelSettings[intensities.channelIndex];

			if (currents.size() != totalSettings) {
				currents.fill(0.0, totalSettings);
			}

			currents[settingID] = current;

			// Legacy scale mode:
			// int value = nvs::scale_int(intensities.intensity, 0, 255, 0, 13107 / 2);
			// cs.intensity[settingID] = value;
			//m_controller.SetIntensityByScale(intensities.channelIndex, settingID, value);
			//m_controller.SetActiveFlagByGroup(groupID, settingID, VIE_ON);
		}

		settingIDs[settingID] = settingID;
		activeFlags[settingID] = VIE_ON;

		settingID++;
	}

	for (const auto& key : channelSettings.keys()) {
		auto retIntensity = m_controller.SetMultipleIntensityByCurrent(key, totalSettings, settingIDs.data(), channelSettings[key].data());
		// Legacy scale mode:
		// auto retIntensity = m_controller.SetMultipleIntensityByScale(key, totalSettings, settingIDs.data(), channelSettings[key].intensity);
		if (retIntensity != VT_SUCCESS) {
			ct::logger::error("[LSC_VIE] Failed to set multiple intensity by current for channel%d: %s", key, getStringRetCode(retIntensity).c_str());
			return (int)LSC_RC::FAIL;
		}
	}

	m_controller.SetMultipleActiveFlagByGroup(groupID, totalSettings, settingIDs.data(), activeFlags.data());

	//Set back to run mode
	operationMode = VIE_RUN_MODE;
	ret = m_controller.SetOperationMode(operationMode);
	if (ret != VT_SUCCESS) {
		ct::logger::error("[LSC_VIE] Failed to enter config mode: %s", getStringRetCode(ret).c_str());
		return (int)LSC_RC::FAIL;
	}

	return (int)LSC_RC::PASS;
}

DWORD LSC_VIE::getGroupID(const std::set<int>& channels)
{
	DWORD group = 0;
	for (int ch : channels) {
		if (ch >= 1 && ch <= 32) group |= (1u << (ch - 1));
	}
	return group;
}

LSC_VIE::LSC_VIE()
{
	m_maxCurrent.fill(VIE_DEFAULT_MAX_CURRENT);
	m_hasMaxCurrent.fill(false);

	for (std::size_t i = 0; i < m_continuousGroupID.size(); ++i) {
		m_continuousGroupID[i] = (DWORD{ 1u } << i);
		//ct::logger::info("Group ID: %lu", m_continuousGroupID[i]);
	}
}

LSC_VIE::~LSC_VIE()
{
}

const std::string& LSC_VIE::id() const
{
	return m_id;
}

std::string& LSC_VIE::id()
{
	return m_id;
}

const std::string& LSC_VIE::name() const
{
	return m_name;
}

std::string& LSC_VIE::name()
{
	return m_name;
}

int LSC_VIE::enable(bool enable)
{
	m_enable = true;
	return (int)LSC_RC::PASS;
}

int LSC_VIE::connect()
{
	m_isConnect = false;

	int totalBoard = 0;
	auto ret = m_manager.InitScan(&totalBoard);
	if (ret != VT_SUCCESS) {
		ct::logger::error("[LSC_VIE] Fail to initialize LSC manager: %s", getStringRetCode(ret).c_str());
		return (int)LSC_RC::FAIL;
	}

	if (totalBoard == 0) {
		ct::logger::error("[LSC_VIE] No board found");
		return (int)LSC_RC::FAIL;
	}

	m_lsc.resize(totalBoard);


	ret = m_manager.GetDetectController(m_lsc.data());
	if (ret != VT_SUCCESS) {
		ct::logger::error("[LSC_VIE] Fail to get controllers: %s", getStringRetCode(ret).c_str());
		return (int)LSC_RC::FAIL;
	}

	if (m_lsc.size() == 0) {
		ct::logger::error("[LSC_VIE] No controller found");
		return (int)LSC_RC::FAIL;
	}

	ret = m_controller.Connect(m_lsc[0]);
	if (ret != VT_SUCCESS) {
		ct::logger::error("[LSC_VIE] Fail to connect controller: %s", getStringRetCode(ret).c_str());
		return (int)LSC_RC::FAIL;
	}
	ct::logger::info("[LSC_VIE] Connected to LSC");


	ret = m_controller.GetTotalChannelPerBoard(&m_numChannel);
	if (ret != VT_SUCCESS) {
		ct::logger::error("[LSC_VIE] Fail to get total channel: %s", getStringRetCode(ret).c_str());
		return (int)LSC_RC::FAIL;
	}
	ct::logger::info("[LSC_VIE] Num of channel per board: %d", m_numChannel);


	//get operation mode
	int operationMode = 0;
	ret = m_controller.GetOperationMode(&operationMode);
	if (ret != VT_SUCCESS) {
		ct::logger::error("[LSC_VIE] Fail to get operation mode: %s", getStringRetCode(ret).c_str());
		return (int)LSC_RC::FAIL;
	}
	ct::logger::info("[LSC_VIE] Operation mode: %d", operationMode);

	loadMaxCurrentJson();
	if (applySavedMaxCurrent() != (int)LSC_RC::PASS) {
		return (int)LSC_RC::FAIL;
	}

	//=> Access intensity
	/*for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			int intensity = 0;
			m_controller.GetIntensityByScale(i, j, &intensity);
			ct::logger::info("Channel ID: %d, Setting ID: %d, Intensity: %d", i, j, intensity);
		}
	}*/

	//=> Access group data
	/*int totalGroup = 0;
	std::vector<DWORD> groups(8);

	m_controller.GetChannelGrouping(&totalGroup, groups.data());

	for (int i = 0; i < totalGroup; i++) {
		ct::logger::info("Group ID: %lu", groups[i]);
	}*/

	m_isConnect = true;

	return (int)LSC_RC::PASS;
}

int LSC_VIE::disconnect()
{
	m_lsc.clear();

	auto ret = m_controller.Disconnect();
	if (ret != VT_SUCCESS) {
		ct::logger::error("[LSC_VIE] Fail to disconnect: %s", getStringRetCode(ret).c_str());
		return (int)LSC_RC::FAIL;
	}

	m_isConnect = false;

	return (int)LSC_RC::PASS;
}

int LSC_VIE::reconnect()
{
	disconnect();
	return connect();
}

bool LSC_VIE::isConnected() const
{
	return m_isConnect;
}

void LSC_VIE::setConnectionTimeOut(int ms)
{
	m_connectionTimeout = ms;
}

void LSC_VIE::setResponseTimeOut(int ms)
{
	m_responseTimeout = ms;
}

int LSC_VIE::numChannel() const
{
	return m_numChannel;
}

int& LSC_VIE::numChannel()
{
	return m_numChannel;
}

int LSC_VIE::toggle(int ch, bool on)
{
	VI32 ret;

	DWORD group = m_continuousGroupID[ch];
	if (on) ret = m_controller.SetContinuousChannelGroupOn(group, ch);
	else ret = m_controller.SetContinuousChannelGroupOff(group);

	if (ret != VT_SUCCESS) {
		ct::logger::error("[LSC_VIE] Fail to toggle groupID(%lu) channel%d: %s", group, ch, getStringRetCode(ret).c_str());
		return (int)LSC_RC::FAIL;
	}

	return (int)LSC_RC::PASS;
}

int LSC_VIE::setIntensity(int ch, int intensity)
{
	double current = intensityToCurrent(ch, intensity);

	auto ret = m_controller.SetIntensityByCurrent(ch, ch, current);
	// Legacy scale mode:
	// int value = nvs::scale_int(intensity, 0, 255, 0, 13107/2);
	// auto ret = m_controller.SetIntensityByScale(ch, ch, value);
	if (ret != VT_SUCCESS) {
		ct::logger::error("[LSC_VIE] Fail to set channel%d intensity by current %.4f: %s", ch, current, getStringRetCode(ret).c_str());
		return (int)LSC_RC::FAIL;
	}
	//ct::logger::info("[LSC_VIE] Set channel%d intensity: %d", ch, intensity);

	return (int)LSC_RC::PASS;
}

int LSC_VIE::setMultiIntensity(lsc::IntensityData* idata, int size)
{
	int ret = 0;

	for (int i = 0; i < size; i++) {
		int ch = idata[i].channelIndex;
		int intensity = idata[i].intensity;
		ret += setIntensity(ch, intensity);
		//m_controller.SetMultipleIntensityByScale(); does not support the way we want.
	}

	if (ret != 0) {
		ct::logger::error("[LSC_VIE] Fail to set multi-intensity: %s", getStringRetCode(ret).c_str());
		return (int)LSC_RC::FAIL;
	}

	return (int)LSC_RC::PASS;
}

int LSC_VIE::getIntensity(int ch, int& intensity)
{
	double current = 0.0;
	auto ret = m_controller.GetIntensityByCurrent(ch, ch, &current);
	// Legacy scale mode:
	// auto ret = m_controller.GetIntensityByScale(ch, ch, &intensity);
	if (ret != VT_SUCCESS) {
		ct::logger::error("[LSC_VIE] Fail to get channel%d intensity by current: %s", ch, getStringRetCode(ret).c_str());
		return (int)LSC_RC::FAIL;
	}

	intensity = currentToIntensity(ch, current);
	// Legacy scale mode:
	// intensity = nvs::scale_int(intensity, 0, 13107/2, 0, 255);

	return (int)LSC_RC::PASS;
}

int LSC_VIE::setIP(std::string ip)
{
	return 0; //not needed
}

int LSC_VIE::setPort(int port)
{
	return 0; //not needed
}

int LSC_VIE::setMode(lsc::MODE mode)
{
	int ret = 0;

	if (mode == lsc::MODE::CONTINUOUS) {
		ct::logger::info("[LSC_VIE] Set continuous mode");
		ret = setContinuousMode();
	}
	else if (mode == lsc::MODE::TRIGGER) {
		ct::logger::info("[LSC_VIE] Set trigger mode");
		ret = setTriggerMode();
	}

	return ret;
}

int LSC_VIE::setTriggerDuration(int ch, int us)
{
	auto ret = m_controller.SetOutputPulseWidthLimit(ch, us);
	if (ret != VT_SUCCESS) {
		ct::logger::error("[LSC_VIE] Fail to set trigger duration for group %d: %d", ch, ret);
		return (int)LSC_RC::FAIL;
	}

	return (int)LSC_RC::PASS;
}

int LSC_VIE::setTriggerSequence(const std::vector<lsc::SequenceData>& datas)
{
	m_sequenceDatas = datas;
	return (int)LSC_RC::PASS;
}

std::string LSC_VIE::codeString(int returnCode)
{
	return std::string();
}


int LSC_VIE::setMaxCurrent(int ch, double intensity)
{


	auto ret = m_controller.SetMaxCurrent(ch, intensity);
	if (ret != VT_SUCCESS) {
		ct::logger::error("[LSC_VIE] Fail to set channel%d Max Current: %s", ch, getStringRetCode(ret).c_str());
		return (int)LSC_RC::FAIL;
	}

	if (isValidChannelIndex(ch)) {
		m_maxCurrent[ch] = intensity;
		m_hasMaxCurrent[ch] = true;
		saveMaxCurrentJson();
	}
	else {
		ct::logger::warn("[LSC_VIE] Max current was set for invalid cache channel index: %d", ch);
	}
	//ct::logger::info("[LSC_VIE] Set channel%d max current: %.4f", ch, intensity);

	return (int)LSC_RC::PASS;
}
