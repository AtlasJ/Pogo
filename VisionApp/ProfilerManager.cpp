#include "ProfilerManager.h"
#include "QJsonHelper.h"
#include "Logger.h"
#include "Utilities.h"
#include "Profiler_Gocator.h"
#include "Profiler_SmartRay.h"
#include "Profiler_SSZN.h"

ProfilerManager ProfilerManager::m_instance;

ProfilerManager& ProfilerManager::instance()
{
	return m_instance;
}

void ProfilerManager::loadConfig(QString path)
{
	QJsonObject obj;

	if (!jsonHelper::loadJson(path, obj)) {
		ct::logger::error("[Profiler] Failed to load profiler.json");
		return;
	}
	
	if (obj.contains("Profilers")) {
		auto profilers = obj["Profilers"].toArray();

		for (auto doc : profilers) {
			auto obj = doc.toObject();

			auto id = jsonHelper::getString(obj, "ID");
			auto api = jsonHelper::getString(obj, "API");
			auto ip = jsonHelper::getString(obj, "IP");
			auto configPath = jsonHelper::getString(obj, "Config_File");
			auto msr = jsonHelper::getBool(obj, "MSR");
			auto invertX = jsonHelper::getBool(obj, "InvertX");
			auto invertY = jsonHelper::getBool(obj, "InvertY");
			m_sensortype = api;
			m_enableMSR = msr;
			m_invertX = invertX;
			m_invertY = invertY;


			if (!create(id, api)) continue;

			if (!connect(id, ip)) continue;

			if (!loadConfig(id, "C:/Advanced/Data/config/" + configPath)) continue;

			if (!setMSR(id, msr)) continue;


		}
	}
}

QList<QString> ProfilerManager::keys() const
{
	return m_profilers.keys();
}

const double ProfilerManager::getExposure(QString id) const
{
	if (!valid(id)) return 0.0;
	return m_profilers[id]->getExposure();
}

const double ProfilerManager::getYResolution(QString id) const
{
	if (!valid(id)) return 0.0;
	return m_profilers[id]->getYResolution();
}

const QString& ProfilerManager::getFirmwareVersion(QString id) const
{
	if (!valid(id)) return error_invalid_id;
	return m_profilers[id]->getFirmwareVersion();
}

const QString& ProfilerManager::getSerialNumber(QString id) const
{
	if (!valid(id)) return error_invalid_id;
	return m_profilers[id]->getSerialNumber();
}

bool ProfilerManager::isConnected(QString id) const
{
	if (!valid(id)) return false;
	return m_profilers[id]->isConnected();
}

const bool ProfilerManager::isGrabbing(QString id) const
{
	if (!valid(id)) return false;
	return m_profilers[id]->isGrabbing();
}

bool ProfilerManager::enable(QString id, bool enable)
{
	if (!valid(id)) return false;
	return m_profilers[id]->enable(enable);
}

bool ProfilerManager::connect(QString id, QString ip)
{
	if (!valid(id)) return false;
	auto ret = m_profilers[id]->connect(ip);
	if (!ret)
	{
		ct::logger::error("[Profiler] Failed to connect profiler: %s", id.toStdString().c_str());
		m_connectionStatus = false;
	}
	else
	{
		ct::logger::info("[Profiler] Connected to: %s", ip.toStdString().c_str());
		m_connectionStatus = true;
	}
	return ret;
}

bool ProfilerManager::disconnect(QString id)
{
	if (!valid(id)) return false;
	auto ret = m_profilers[id]->disconnect();
	if (!ret) ct::logger::error("[Profiler] Failed to disconnect profiler: %s", id.toStdString().c_str());
	ct::logger::info("[Profiler] Disconnected from: %s", id.toStdString().c_str());
	return ret;
}

bool ProfilerManager::start(QString id)
{
	if (!valid(id)) return false;
	auto ret = m_profilers[id]->start();
	if (!ret) ct::logger::error("[Profiler] Failed to start grabbing: %s", id.toStdString().c_str());
	return ret;
}

bool ProfilerManager::stop(QString id)
{
	if (!valid(id)) return false;
	auto ret = m_profilers[id]->stop();
	if (!ret) ct::logger::error("[Profiler] Failed to stop grabbing: %s", id.toStdString().c_str());
	return ret;
}

bool ProfilerManager::snapShot(QString id)
{
	if (!valid(id)) return false;
	auto ret = m_profilers[id]->snapShot();
	if (!ret) ct::logger::error("[Profiler] Failed to snap: %s", id.toStdString().c_str());
	return ret;
}

bool ProfilerManager::enableIntensityMap(QString id, bool enable)
{
	if (!valid(id)) return false;
	auto ret = m_profilers[id]->enableIntensityMap(enable);
	if (!ret) ct::logger::error("[Profiler] Failed to enable intensity map: %s", id.toStdString().c_str());
	return ret;
}

bool ProfilerManager::setScanLength(QString id, double mm)
{
	if (!valid(id)) return false;
	auto ret = m_profilers[id]->setScanLength(mm);
	if (!ret) ct::logger::error("[Profiler] Failed to set scan length: %s", id.toStdString().c_str());
	return ret;
}

bool ProfilerManager::setGain(QString id, double gain)
{
	if (!valid(id)) return false;

	if (util::is_equal(m_currentSettings[id].gain, gain, 0.1)) {
		ct::logger::trace("[Profiler] Gain already set");
		return true;
	}

	auto ret = m_profilers[id]->setGain(gain);

	if (ret) m_currentSettings[id].gain = gain;
	if (!ret) ct::logger::error("[Profiler] Failed to set gain: %s, %f", id.toStdString().c_str(), gain);
	return ret;
}

bool ProfilerManager::setDuoHeadGain(QString id, double gain, double gain2)
{
	if (!valid(id)) return false;

	if (util::is_equal(m_currentSettings[id].gain, gain, 0.1)&& util::is_equal(m_currentSettings[id].gain2, gain2, 0.1)) {
		ct::logger::trace("[Profiler] Gain already set");
		return true;
	}

	auto ret = m_profilers[id]->setDuoHeadGain(gain, gain2);

	if (ret)
	{
		m_currentSettings[id].gain = gain;
		m_currentSettings[id].gain2 = gain2;
	}
	if (!ret)
	{
		ct::logger::error("[Profiler] Failed to set duo head gain: %s, %f, %f", id.toStdString().c_str(), gain, gain2);
	}
	return ret;
}

bool ProfilerManager::setDivider(QString id, int divider)
{
	if (!valid(id)) return false;

	if (m_currentSettings[id].divider == divider) {
		ct::logger::trace("[Profiler] Divider already set");
		return true;
	}

	auto ret = m_profilers[id]->setDivider(divider);

	if (ret) m_currentSettings[id].divider = divider;
	if (!ret) ct::logger::error("[Profiler] Failed to set divider: %s, %d", id.toStdString().c_str(), divider);
	return ret;
}

bool ProfilerManager::setExposureMode(QString id, IProfiler::ExposureMode mode)
{
	if (!valid(id)) return false;

	QString smode = ct::s_single;
	if (mode == IProfiler::MULTI) smode = ct::s_multi;
	else if (mode == IProfiler::DYNAMIC) smode = ct::s_dynamic;

	if (m_currentSettings[id].exposureMode == smode) {
		ct::logger::trace("[Profiler] Exposure mode already set");
		return true;
	}

	auto ret = m_profilers[id]->setExposureMode(mode);

	if (ret) {
		m_currentSettings[id].exposureMode = smode;
		m_currentSettings[id].exposure = 0.0;
		m_currentSettings[id].exposure2 = 0.0;
	}
	if (!ret) ct::logger::error("[Profiler] Failed to set exposure mode: %s, %f", id.toStdString().c_str(), smode.toStdString().c_str());
	return ret;
}

bool ProfilerManager::setExposure(QString id, double us)
{
	if (!valid(id)) return false;

	if (util::is_equal(m_currentSettings[id].exposure, us, 0.1)) {
		ct::logger::info("[Profiler] Exposure already set");
		return true;
	}

	auto ret = m_profilers[id]->setExposure(us);

	if (ret) m_currentSettings[id].exposure = us;
	if (!ret) ct::logger::error("[Profiler] Failed to set exposure: %s, %f", id.toStdString().c_str(), us);
	return ret;
}

bool ProfilerManager::setMultiExposure(QString id, double us, double us2)
{
	if (!valid(id)) return false;

	if (util::is_equal(m_currentSettings[id].exposure, us, 0.1) && util::is_equal(m_currentSettings[id].exposure2, us2, 0.1))
	{
		ct::logger::trace("[Profiler] Multi exposure already set");
		return true;
	}
	

	auto ret = m_profilers[id]->setMultiExposure(us,us2);

	if (ret) {
		m_currentSettings[id].exposure = us;
		m_currentSettings[id].exposure2 = us2;
	}
	if (!ret) ct::logger::error("[Profiler] Failed to set multi exposure: %s, %f & %f", id.toStdString().c_str(), us,us2);
	return ret;
}

bool ProfilerManager::setDynamicExposure(QString id, double min_us, double max_us)
{
	if (!valid(id)) return false;

	if (util::is_equal(m_currentSettings[id].exposure, min_us, 0.1) && util::is_equal(m_currentSettings[id].exposure2, max_us, 0.1))
	{
		ct::logger::trace("[Profiler] Dynamic exposure already set");
		return true;
	}


	auto ret = m_profilers[id]->setDynamicExposure(min_us, max_us);

	if (ret) {
		m_currentSettings[id].exposure = min_us;
		m_currentSettings[id].exposure2 = max_us;
	}
	if (!ret) ct::logger::error("[Profiler] Failed to set dynamic exposure: %s, %f & %f", id.toStdString().c_str(), min_us, max_us);
	return ret;
}

bool ProfilerManager::setParallelExposure(QString id, double us, double us2)
{
	if (!valid(id)) return false;

	if (util::is_equal(m_currentSettings[id].exposure, us, 0.1) && util::is_equal(m_currentSettings[id].exposure2, us2, 0.1))
	{
		ct::logger::trace("[Profiler] Parallel exposure already set");
		return true;
	}

	auto ret = m_profilers[id]->setParallelExposure(us, us2);

	if (ret) {
		m_currentSettings[id].exposure = us;
		m_currentSettings[id].exposure2 = us2;
	}
	if (!ret) ct::logger::error("[Profiler] Failed to set parallel exposure: %s, %f & %f", id.toStdString().c_str(), us, us2);
	return ret;
}

bool ProfilerManager::setMSR(QString id, bool enable)
{
	if (!valid(id)) return false;
	auto ret = m_profilers[id]->setMSR(enable);
	if (!ret)
	{
		ct::logger::error("[Profiler] Failed to set MSR: %s", id.toStdString().c_str());
		m_enableMSR = false;
	}
	return ret;
}

bool ProfilerManager::setLaserLineThreshold(QString id, double threshold)
{
	if (!valid(id)) return false;
	
	if (util::is_equal(m_currentSettings[id].lineThreshold, threshold, 0.1)) {
		ct::logger::trace("[Profiler] Line threshold already set");
		return true;
	}

	auto ret = m_profilers[id]->setLaserLineThreshold(threshold);

	if (ret) m_currentSettings[id].lineThreshold = threshold;
	if (!ret) ct::logger::error("[Profiler] Failed to set laser line threshold: %s, %f", id.toStdString().c_str(), threshold);
	return ret;
}

bool ProfilerManager::waitAcquisition(QString id, int ms)
{
	if (!valid(id)) return false;
	auto ret = m_profilers[id]->waitAcquisition(ms);
	if (!ret) ct::logger::error("[Profiler] Acquisition timeout: %s", id.toStdString().c_str());
	
	return ret;
}

const FrameInfo* ProfilerManager::getFrame(QString id) const
{
	if (!valid(id)) return nullptr;
	return &m_profilers[id]->getFrame();
}

FrameInfo* ProfilerManager::getFrame(QString id)
{
	if (!valid(id)) return nullptr;
	return &m_profilers[id]->getFrame();
}

bool ProfilerManager::resetFrame(QString id)
{
	if (!valid(id)) return false;
	m_profilers[id]->resetFrame();
	return true;
}

bool ProfilerManager::loadConfig(QString id, QString path)
{
	if (!valid(id)) return false;
	auto ret = m_profilers[id]->loadConfig(path);
	if (!ret) ct::logger::error("[Profiler] Fail to load profiler config: %s", path.toStdString().c_str());
	else {
		ct::logger::info("[Profiler] Loaded config: %s, %s", id.toStdString().c_str(), path.toStdString().c_str());
	}
	return ret;
}

QString ProfilerManager::errorMsg(QString id)
{
	if (!valid(id)) return "INVALID_ID";
	return m_profilers[id]->errorMsg();
}

QHash<QString, IProfiler*>& ProfilerManager::profilers()
{
	return m_profilers;
}

IProfiler* ProfilerManager::profiler(QString id)
{
	if (m_profilers.contains(id)) return m_profilers[id];
	ct::logger::warn("[Profiler] Trying to access invalid profiler: %s", id.toStdString().c_str());
	return nullptr;
}

const IProfiler* ProfilerManager::profiler(QString id) const
{
	if (m_profilers.contains(id)) return m_profilers[id];
	ct::logger::warn("[Profiler] Trying to access invalid profiler: %s", id.toStdString().c_str());
	return nullptr;
}

ProfilerManager::ProfilerManager()
{
}

ProfilerManager::~ProfilerManager()
{
	for (auto profiler : m_profilers) {
		profiler->stop();
		profiler->disconnect();
	}
}

bool ProfilerManager::create(QString id, QString api)
{
	if (api == "Gocator") {
		auto* profiler = new Profiler_Gocator();
		m_profilers.insert(id, profiler);
	}
	else if (api == "SmartRay") {
		auto* profiler = new Profiler_SmartRay();
		m_profilers.insert(id, profiler);
	}
	else if (api == "SSZN") {
		auto* profiler = new Profiler_SSZN();
		m_profilers.insert(id, profiler);
	}
	else {
		ct::logger::error("[Profiler] Failed to create profiler: %s", api.toStdString().c_str());
		return false;
	}

	m_currentSettings.insert(id, OpticsInfo3D());
	ct::logger::info("[Profiler] Created profiler: %s", api.toStdString().c_str());
	return true;
}

bool ProfilerManager::valid(QString id) const
{
	if (!m_profilers.contains(id)) ct::logger::warn("[Profiler] Trying to access invalid profiler: %s", id.toStdString().c_str());
	return m_profilers.contains(id);
}

