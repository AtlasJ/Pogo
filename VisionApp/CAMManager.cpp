#include "CAMManager.h"
#include "CAM_IRayple.h"
#include "CAM_IRayple_FG.h"
#include "CAM_OPT.h"
#include "CAM_HIK.h"
#include "CAM_HIK_FG.h"

#include "Logger.h"
#include "QJsonHelper.h"
#include "Utilities.h"

CAMManager CAMManager::m_instance;

CAMManager::CAMManager()
{
}

CAMManager::~CAMManager()
{
}

bool CAMManager::create(QString id, QString api)
{
	if (api == "IRayple") {
		auto* cam = new CAM_IRayple();
		m_cam.insert(id, cam);
	}
	else if (api == "IRayple_FG") {
		auto* cam = new CAM_IRayple_FG();
		m_cam.insert(id, cam);
	}
	else if (api == "OPT") {
		auto* cam = new CAM_OPT();
		m_cam.insert(id, cam);
	}
	else if (api == "HIK") {
		auto* cam = new CAM_HIK();
		m_cam.insert(id, cam);
	}
	else if (api == "HIK_FG") {
		auto* cam = new CAM_HIK_FG();
		m_cam.insert(id, cam);
	}
	else {
		ct::logger::error("[CAM] Failed to create camera: %s", api.toStdString().c_str());
		return false;
	}

	m_currentExposure.insert(id, 0.0);
	m_currentGain.insert(id, 0.0);
	ct::logger::info("[CAM] Created camera: %s", api.toStdString().c_str());
	return true;
}

bool CAMManager::valid(QString id) const
{
	if (!m_cam.contains(id)) ct::logger::warn("[CAM_valid] Trying to access invalid camera: %s", id.toStdString().c_str());
	return m_cam.contains(id);
}

bool CAMManager::verifyKey(QString sn, int key) const
{
	int securityKey = 1;
	int notDigit = 0;

	for (auto x : sn)
	{
		int value = x.digitValue();

		if (value == 0) continue;

		if (value == -1)
		{
			notDigit++;
		}
		else
		{
			securityKey *= value;
		}
	}

	securityKey -= notDigit;

	if (key != securityKey)
	{
		//ct::logger::error("[CAM] Invalid camera key. Expected key: %d, Receive: %d", securityKey, key);
		// Don't print the info
		ct::logger::error("[CAM] Licensing Error: Invalid license.");
		return false;
	}

	return true;
}

CAMManager& CAMManager::instance()
{
	return m_instance;
}

void CAMManager::loadConfig(QString path)
{
	QJsonObject obj;

	if (!jsonHelper::loadJson(path, obj)) {
		ct::logger::error("[CAM] Failed to load camera.json");
		return;
	}

	if (obj.contains("Camera")) {
		auto cams = obj["Camera"].toArray();

		for (auto doc : cams) {
			auto obj = doc.toObject();

			auto id = jsonHelper::getString(obj, "ID");
			auto api = jsonHelper::getString(obj, "API");
			auto sn = jsonHelper::getString(obj, "Serial_Number");
			auto configPath = jsonHelper::getString(obj, "Config_File");
			auto skey = jsonHelper::getInteger(obj, "Security_Key");

			auto lscObj = jsonHelper::getObject(obj, "LSC");
			auto lscID = jsonHelper::getInteger(lscObj, "ID");
			auto triggerSource = jsonHelper::getInteger(lscObj, "Trigger_Source");
			m_lsc[id].id = lscID;
			m_lsc[id].triggerSource = triggerSource;
			ct::logger::info("[CAM] Trigger source: %d", m_lsc[id].triggerSource);

			if (!verifyKey(sn, skey)) continue;

			if (!create(id, api)) continue;

			if (!connect(id, sn)) continue;

			if (!loadConfig(id, "C:/Advanced/Data/config/" + configPath)) continue;

			if (!startGrab(id)) continue;
		}
	}
}

QList<QString> CAMManager::keys() const
{
	return m_cam.keys();
}

const int CAMManager::getWidth(QString id) const
{
	if (!valid(id)) return m_defaultW;
	return m_cam[id]->getWidth();
}

const int CAMManager::getHeight(QString id) const
{
	if (!valid(id)) return m_defaultH;
	return m_cam[id]->getHeight();
}

const int CAMManager::getChannel(QString id) const
{
	if (!valid(id)) return m_defaultChannel;
	return m_cam[id]->getChannel();
}

const double CAMManager::getExposure(QString id) const
{
	if (!valid(id)) return 0.0;
	return m_cam[id]->getExposure();
}

const double CAMManager::getGain(QString id) const
{
	if (!valid(id)) return 0.0;
	return m_cam[id]->getGain();
}

const QString& CAMManager::getName(QString id) const
{
	if (!valid(id)) return "";
	return m_cam[id]->getName();
}

const QString& CAMManager::getSerialNumber(QString id) const
{
	if (!valid(id)) return "";
	return m_cam[id]->getSerialNumber();
}

bool CAMManager::isConnected(QString id) const
{
	if (!valid(id)) return false;
	return m_cam[id]->isConnected();
}

const bool CAMManager::isGrabbing(QString id) const
{
	if (!valid(id)) return false;
	return m_cam[id]->isGrabbing();
}

bool CAMManager::enable(QString id, bool enable)
{
	if (!valid(id)) return false;
	return m_cam[id]->enable(enable);
}

bool CAMManager::connect(QString id, QString sn)
{
	if (!valid(id)) return false;
	auto ret =  m_cam[id]->connect(sn);
	if (!ret) ct::logger::error("Failed to connect camera: %s", sn.toStdString().c_str());
	else ct::logger::info("[CAM] Connected to: %s", sn.toStdString().c_str());
	return ret;
}

bool CAMManager::disconnect(QString id)
{
	if (!valid(id)) return false;
	auto ret = m_cam[id]->disconnect();
	if (!ret) ct::logger::error("Failed to disconnect camera: %s", id.toStdString().c_str());
	else ct::logger::info("[CAM] Discconected from: %s", id.toStdString().c_str());
	return ret;
}

bool CAMManager::reconnect(QString id)
{
	if (!connect(id, getSerialNumber(id))) return false;

	//if (!loadConfig(id, "C:/Advanced/Data/config/" + configPath))  return false;

	if (!startGrab(id)) return false;
}

bool CAMManager::startGrab(QString id)
{
	if (!valid(id)) return false;
	auto ret = m_cam[id]->startGrab();
	if (!ret) ct::logger::error("Failed to start grabbing: %s", id.toStdString().c_str());
	return ret;
}

bool CAMManager::stopGrab(QString id)
{
	if (!valid(id)) return false;
	auto ret = m_cam[id]->stopGrab();
	if (!ret) ct::logger::error("Failed to stop grabbing: %s", id.toStdString().c_str());
	return ret;
}

bool CAMManager::setExposure(QString id, double exposure)
{
	if (!valid(id)) return false;

	/*if (util::is_equal(m_currentExposure[id], exposure, 0.1)) {
		ct::logger::info("Exposure already set");
		return true;
	}*/

	auto ret = m_cam[id]->setExposure(exposure);

	if (ret) m_currentExposure[id] = exposure;
	if (!ret) ct::logger::error("Failed to set exposure: %s, %f", id.toStdString().c_str(), exposure);
	return ret;
}

bool CAMManager::setGain(QString id, double gain)
{
	if (!valid(id)) return false;

	if (util::is_equal(m_currentGain[id], gain, 0.1)) {
		ct::logger::info("Gain already set");
		return true;
	}

	auto ret = m_cam[id]->setGain(gain);

	if (ret) m_currentGain[id] = gain;
	if (!ret) ct::logger::error("Failed to set gain: %s", id.toStdString().c_str(), gain);
	return ret;
}

bool CAMManager::softTrigger(QString id)
{
	if (!valid(id)) return false;
	auto ret = m_cam[id]->softTrigger();
	if (!ret) ct::logger::error("Failed to soft trigger: %s", id.toStdString().c_str());
	return ret;
}

bool CAMManager::waitAcquisition(QString id, int ms)
{
	if (!valid(id)) return false;
	auto ret = m_cam[id]->waitAcquisition(ms);
	if (!ret) ct::logger::error("Acquisition timeout: %s", id.toStdString().c_str());
	return ret;
}

bool CAMManager::setTriggerOutput(QString id, QString line, QString source)
{
	if (!valid(id)) return false;
	auto ret = m_cam[id]->setTriggerOutput(line, source);
	if (!ret) ct::logger::error("Failed to set trigger output: %s -> %s, %s"
		, id.toStdString().c_str()
		, line.toStdString().c_str()
		, source.toStdString().c_str());
	return ret;
}

bool CAMManager::setDO(QString id, int DO, bool on)
{
	ScopedTimeLogger stl("[CAM] Set DO");
	if (!valid(id)) return false;
	auto ret = m_cam[id]->setDO(DO, on);
	if (!ret) ct::logger::error("Failed to set DO: %d -> %d", DO, on);
	return ret;
}

QString CAMManager::errorMsg(QString id)
{
	if (!valid(id)) return "INVALID_ID";
	return m_cam[id]->errorMsg();
}

bool CAMManager::loadConfig(QString id, QString path)
{
	if (!valid(id)) return false;
	auto ret = m_cam[id]->loadConfig(path);
	if (!ret) ct::logger::error("Fail to load camera config: %s", path.toStdString().c_str());
	else ct::logger::info("[CAM] Loaded config: %s, %s", id.toStdString().c_str(), path.toStdString().c_str());
	return ret;
}

const FrameInfo* CAMManager::frame(QString id) const
{
	if (!valid(id)) return nullptr;
	return &m_cam[id]->frame();
}

FrameInfo* CAMManager::frame(QString id)
{
	if (!valid(id)) return nullptr;
	return &m_cam[id]->frame();
}

bool CAMManager::resetFrame(QString id)
{
	if (!valid(id)) return false;
	m_cam[id]->resetFrame();
	return true;
}

void CAMManager::setDefaultWidth(int w)
{
	m_defaultW = w;

}

void CAMManager::setDefaultHeight(int h)
{
	m_defaultH = h;

}

void CAMManager::setDefaultChannel(int c)
{
	m_defaultChannel = c;

}

QHash<QString, ICamera*>& CAMManager::cameras()
{
	return m_cam;
}

ICamera* CAMManager::camera(QString id)
{
	if (m_cam.contains(id)) return m_cam[id];
	ct::logger::warn("[CAM_1] Trying to access invalid camera: %s", id.toStdString().c_str());
	return nullptr;
}

const ICamera* CAMManager::camera(QString id) const
{
	if (m_cam.contains(id)) return m_cam[id];
	ct::logger::warn("[CAM_2] Trying to access invalid camera: %s", id.toStdString().c_str());
	return nullptr;
}

CAMManager::LSCInfo* CAMManager::lsc(QString id) 
{
	if (m_lsc.contains(id)) {
		ct::logger::info("[CAM] 1Trigger source: %d", m_lsc[id].triggerSource);
		return &m_lsc[id];
	}
	ct::logger::warn("[CAM] Trying to access invalid camera's LSC info: %s", id.toStdString().c_str());
	return nullptr;
}
