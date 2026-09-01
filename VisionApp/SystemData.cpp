#include "SystemData.h"
#include "CommonDir.h"
#include "QJsonHelper.h"
#include "QHostInfo.h"
#include "Logger.h"
#include "AuditLog.h"

static bool loadPersistedMachineName(const QString& path, const QString& rootKey, QString& machineName)
{
	QJsonObject root;
	if (!jsonHelper::loadJson(path, root)) return false;

	const QJsonObject info = root.value(rootKey).toObject();
	if (!info.contains(QStringLiteral("machine_name"))) return false;

	machineName = info.value(QStringLiteral("machine_name")).toString();
	return true;
}

static void auditMachineNameChange(const QString& scope,
	bool hadPrevious,
	const QString& previousName,
	const QString& currentName,
	bool saved)
{
	if (!hadPrevious || previousName == currentName) return;

	AuditLog::instance().log(QStringLiteral("MACHINE_NAME_CHANGE"),
		QStringLiteral("scope=%1; old=%2; new=%3").arg(scope, previousName, currentName),
		saved ? QStringLiteral("OK") : QStringLiteral("FAILED"));
}

SystemData SystemData::m_instance;

SystemData::SystemData()
{
}

SystemData::~SystemData()
{
}

void SystemData::toJson(const ct::Box2D& obj, QJsonObject& j)
{
	double cx = obj.cx;
	double cy = obj.cy;
	double xmin = obj.xmin;
	double ymin = obj.ymin;
	double xmax = obj.xmax;
	double ymax = obj.ymax;

	j.insert(QStringLiteral("id"), obj.id.c_str());
	j.insert(QStringLiteral("cx"), cx);
	j.insert(QStringLiteral("cy"), cy);
	j.insert(QStringLiteral("w"), obj.w);
	j.insert(QStringLiteral("h"), obj.h);
	j.insert(QStringLiteral("xmin"), xmin);
	j.insert(QStringLiteral("ymin"), ymin);
	j.insert(QStringLiteral("xmax"), xmax);
	j.insert(QStringLiteral("ymax"), ymax);
}

void SystemData::fromJson(const QJsonObject& j, ct::Box2D& obj)
{
	obj.id = jsonHelper::getString(j, "id", "").toStdString();
	obj.cx = jsonHelper::getInteger(j, "cx");
	obj.cy = jsonHelper::getInteger(j, "cy");
	obj.w = jsonHelper::getInteger(j, "w");
	obj.h = jsonHelper::getInteger(j, "h");
	obj.xmin = jsonHelper::getInteger(j, "xmin");
	obj.ymin = jsonHelper::getInteger(j, "ymin");
	obj.xmax = jsonHelper::getInteger(j, "xmax");
	obj.ymax = jsonHelper::getInteger(j, "ymax");

	obj.compute_extremum();
}

void SystemData::toJson(const dat::WorldCoordinate& obj, QJsonObject& j, bool isRelative)
{
	dat::WorldCoordinate relativeObj = obj;
	if (!isRelative) relativeObj = getRelativeRobotPoint(obj);

	j.insert(QStringLiteral("wx"), relativeObj.wx);
	j.insert(QStringLiteral("wy"), relativeObj.wy);
	j.insert(QStringLiteral("wz"), relativeObj.wz);
}

void SystemData::fromJson(const QJsonObject& j, dat::WorldCoordinate& obj, bool isAbsolute)
{
	obj.wx = jsonHelper::getDouble(j, "wx");
	obj.wy = jsonHelper::getDouble(j, "wy");
	obj.wz = jsonHelper::getDouble(j, "wz");

	if (!isAbsolute) obj = getAbsoluteRobotPoint(obj);
}

void SystemData::getCurrentMachinePortabilityPointOffset(dat::WorldCoordinate& offset)
{
	if (_portability.ref_info.machine_name == QHostInfo::localHostName())
	{
		offset = dat::WorldCoordinate();
	}
	else if (_portability.current_info.machine_name == QHostInfo::localHostName())
	{
		offset = _portability.current_info.portability_point - _portability.ref_info.portability_point;
	}
}

dat::WorldCoordinate SystemData::getAbsoluteRobotPoint(dat::WorldCoordinate point)
{
	dat::WorldCoordinate offset;
	getCurrentMachinePortabilityPointOffset(offset);
	dat::WorldCoordinate absolutePoint = point + offset;
	return absolutePoint;
}

dat::WorldCoordinate SystemData::getRelativeRobotPoint(dat::WorldCoordinate point)
{
	dat::WorldCoordinate offset;
	getCurrentMachinePortabilityPointOffset(offset);
	dat::WorldCoordinate relativePoint = point - offset;
	return relativePoint;
}

SystemData& SystemData::instance()
{
	return m_instance;
}

bool SystemData::save(Type type)
{
	bool ret = true;

    if (type == Type::PORTABILITY) {
		auto jsonPath = Common::Directory::PortabilityPath() + "RefPositionPortability.json";
		QString previousRefMachineName;
		const bool hadPreviousRefMachineName = loadPersistedMachineName(jsonPath,
			QStringLiteral("RefPositionPortabilityInfos"), previousRefMachineName);

		QJsonObject j_root;
		QJsonArray j_array;

		QJsonObject obj;
		QJsonObject srObj, prObj;

		auto& ref = _portability.ref_info;
		ref.search_region.compute_extremum();
		ref.learn_region.compute_extremum();
		toJson(ref.search_region, srObj);
		toJson(ref.learn_region, prObj);

		obj.insert(QStringLiteral("id"), ref.id);
		obj.insert(QStringLiteral("search_region"), srObj);
		obj.insert(QStringLiteral("learn_region"), prObj);
		obj.insert(QStringLiteral("feature_searching_method"), ref.feature_searching_method);
		obj.insert(QStringLiteral("min_diameter"), ref.min_diameter);
		obj.insert(QStringLiteral("max_diameter"), ref.max_diameter);
		obj.insert(QStringLiteral("width"), ref.width);
		obj.insert(QStringLiteral("height"), ref.height);
		obj.insert(QStringLiteral("machine_name"), ref.machine_name);
		obj.insert(QStringLiteral("PIC"), ref.PIC);
		obj.insert(QStringLiteral("date_created"), ref.date_created);
		obj.insert(QStringLiteral("learnt_status"), ref.learnt_status);

		QJsonObject pObj;
		toJson(ref.portability_point, pObj, true);
		obj.insert(QStringLiteral("ref_portability_point"), pObj);

		j_root.insert(QStringLiteral("RefPositionPortabilityInfos"), obj);

		const bool refSaved = jsonHelper::saveJson(jsonPath, QJsonDocument(j_root));
		ret = refSaved;
		auditMachineNameChange(QStringLiteral("reference"), hadPreviousRefMachineName,
			previousRefMachineName, ref.machine_name, refSaved);

		{
			auto jsonPath = Common::Directory::PortabilityPath() + "CurPositionPortability.json";
			QString previousCurMachineName;
			const bool hadPreviousCurMachineName = loadPersistedMachineName(jsonPath,
				QStringLiteral("CurPositionPortabilityInfos"), previousCurMachineName);

			QJsonObject j_root;
			QJsonArray j_array;

			QJsonObject obj;
			auto& cur = _portability.current_info;

			obj.insert(QStringLiteral("id"), cur.id);
			obj.insert(QStringLiteral("machine_name"), cur.machine_name);
			obj.insert(QStringLiteral("PIC"), cur.PIC);
			obj.insert(QStringLiteral("date_created"), cur.date_created);

			cur.offset_point = cur.portability_point - _portability.ref_info.portability_point;

			QJsonObject pObj;
			toJson(cur.portability_point, pObj, true);
			obj.insert(QStringLiteral("cur_portability_point"), pObj);

			QJsonObject offsetObj;
			toJson(cur.offset_point, offsetObj, true);
			obj.insert(QStringLiteral("cur_offset"), offsetObj);

			j_root.insert(QStringLiteral("CurPositionPortabilityInfos"), obj);

			const bool curSaved = jsonHelper::saveJson(jsonPath, QJsonDocument(j_root));
			ret &= curSaved;
			auditMachineNameChange(QStringLiteral("current"), hadPreviousCurMachineName,
				previousCurMachineName, cur.machine_name, curSaved);
		}
    }

    return ret;
}

bool SystemData::load(Type type)
{
	bool ret = true;

	if (type == Type::PORTABILITY) {
		QString portabilityPath = Common::Directory::PortabilityPath();
		auto jsonPath = portabilityPath + "RefPositionPortability.json";

		QJsonObject root;

		auto& ref = _portability.ref_info;

		//guard
		if (!jsonHelper::loadJson(jsonPath, root)) {
			ref.learn_region.cx = _camWidth / 2;
			ref.learn_region.cy = _camHeight / 2;
			ref.learn_region.w = _camWidth * 0.2;
			ref.learn_region.h = _camHeight * 0.2;
			ref.learn_region.compute_extremum();
			_portability.learn_region.setGeometry(QRectF(_camWidth / 2 - _camWidth * 0.1, _camHeight / 2 - _camHeight * 0.1, _camWidth * 0.2, _camHeight * 0.2));

			ref.search_region.cx = _camWidth / 2;
			ref.search_region.cy = _camHeight / 2;
			ref.search_region.w = _camWidth;
			ref.search_region.h = _camHeight;
			ref.search_region.compute_extremum();
			_portability.search_region.setGeometry(QRectF(ref.search_region.xmin, ref.search_region.ymin, ref.search_region.w, ref.search_region.h));
		}


		if (!root.contains("RefPositionPortabilityInfos")) return false;
		auto pInfos = root["RefPositionPortabilityInfos"].toObject();

		ref.id = jsonHelper::getString(pInfos, QStringLiteral("id"));
		ref.machine_name = jsonHelper::getString(pInfos, QStringLiteral("machine_name"));
		ref.PIC = jsonHelper::getString(pInfos, QStringLiteral("PIC"));
		ref.date_created = jsonHelper::getString(pInfos, QStringLiteral("date_created"));
		ref.feature_searching_method = jsonHelper::getInteger(pInfos, QStringLiteral("feature_searching_method"), 0);
		ref.min_diameter = jsonHelper::getDouble(pInfos, QStringLiteral("min_diameter"), 280);
		ref.max_diameter = jsonHelper::getDouble(pInfos, QStringLiteral("max_diameter"), 320);
		ref.learnt_status = jsonHelper::getBool(pInfos, QStringLiteral("learnt_status"), false);

		//if (ref.learnt_status) //TODO:
		//{
		//	ui.lineEdit_refPointLearntStatus->setText("Ref Portability Point Learnt");
		//	ui.lineEdit_refPointLearntStatus->setStyleSheet("color: green;"); // Set text color to green
		//	ui.toolButton_setRefPoint->setDisabled(true);
		//}
		//else
		//{
		//	ui.lineEdit_refPointLearntStatus->setText("Ref Portability Point Not Learnt");
		//	ui.lineEdit_refPointLearntStatus->setStyleSheet("color: red;"); // Set text color to green
		//	ui.toolButton_setRefPoint->setDisabled(false);
		//}

		fromJson(pInfos["search_region"].toObject(), ref.search_region);
		fromJson(pInfos["learn_region"].toObject(), ref.learn_region);
		fromJson(pInfos["ref_portability_point"].toObject(), ref.portability_point, true);

		/*ui.lineEdit_refPointStatus->setText(QString("X:%1   Y:%2   Z:%3")
			.arg(ref.portability_point.wx)
			.arg(ref.portability_point.wy)
			.arg(ref.portability_point.wz));

		ui.lineEdit_refPointLearntFromMachine->setText(ref.machine_name);
		ui.lineEdit_refPointDateCreated->setText(ref.date_created);
		ui.lineEdit_refPointPIC->setText(ref.PIC);

		ui.comboBox_FeatureLearning->setCurrentIndex(ref.feature_searching_method);*/

		_portability.learn_region.setGeometry(QRectF(ref.learn_region.xmin, ref.learn_region.ymin, ref.learn_region.w, ref.learn_region.h));
		_portability.search_region.setGeometry(QRectF(ref.search_region.xmin, ref.search_region.ymin, ref.search_region.w, ref.search_region.h));

		//ui.lineEdit_minDiameter->setText(QString::number(ref.min_diameter));
		//ui.lineEdit_maxDiameter->setText(QString::number(ref.max_diameter));

		//auto modelPath = Common::Directory::PortabilityPath() + "PortabilityFeature.mod";
		//if (QFileInfo::exists(modelPath))
		//{
		//	ui.lineEdit_featureLearningStatus->setText("Pattern Learnt"); // Set text
		//	ui.lineEdit_featureLearningStatus->setStyleSheet("color: green;"); // Set text color to green
		//}
		//else
		//{
		//	ui.lineEdit_featureLearningStatus->setText("Pattern Not Learnt"); // Set text
		//	ui.lineEdit_featureLearningStatus->setStyleSheet("color: red;"); // Set text color to green
		//}

		{
			/*ct::logger::debug("Load Cur Position Portability Info");

			if (SystemData::instance()._portability.ref_info.machine_name == QHostInfo::localHostName())
			{
				ui.toolButton_setCurPoint->setDisabled(true);
				ui.toolButton_jogToCurrentPoint->setDisabled(true);
				return false;
			}*/

			QString portabilityPath = Common::Directory::PortabilityPath();
			auto jsonPath = portabilityPath + "CurPositionPortability.json";
			QJsonObject root;
			auto& cur = _portability.current_info;
			if (!jsonHelper::loadJson(jsonPath, root))
			{
				cur = PositionPortabilityInfo();
				return false;
			}

			if (!root.contains("CurPositionPortabilityInfos")) {
				cur = PositionPortabilityInfo();
				return false;
			}
			auto pInfos = root["CurPositionPortabilityInfos"].toObject();

			cur.id = jsonHelper::getString(pInfos, QStringLiteral("id"));
			cur.machine_name = jsonHelper::getString(pInfos, QStringLiteral("machine_name"));
			cur.PIC = jsonHelper::getString(pInfos, QStringLiteral("PIC"));
			cur.date_created = jsonHelper::getString(pInfos, QStringLiteral("date_created"));

			fromJson(pInfos["cur_portability_point"].toObject(), cur.portability_point, true);
			cur.offset_point = cur.portability_point - ref.portability_point;

			//ui.lineEdit_setCurPointStatus->setText(QString("X:%1   Y:%2   Z:%3")
			//	.arg(cur.portability_point.wx)
			//	.arg(cur.portability_point.wy)
			//	.arg(cur.portability_point.wz));

			//ui.lineEdit_offsetFromRefPoint->setText(QString("(offset) X:%1   Y:%2   Z:%3")
			//	.arg(cur.offset_point.wx)
			//	.arg(cur.offset_point.wy)
			//	.arg(cur.offset_point.wz));

			//ui.lineEdit_curPointLearntFromMachine->setText(cur.machine_name);
			//ui.lineEdit_CurPointDateCreated->setText(cur.date_created);
			//ui.lineEdit_curPointPIC->setText(cur.PIC);

			//if (cur.machine_name == QHostInfo::localHostName())
			//{
			//	ui.lineEdit_curPointLearntStatus->setText("Cur Portability Point Learnt from current machine");
			//	ui.lineEdit_curPointLearntStatus->setStyleSheet("color: green;"); // Set text color to green
			//	ui.toolButton_setCurPoint->setDisabled(true);
			//}
			//else
			//{
			//	ui.lineEdit_curPointLearntStatus->setText("Cur Portability Point Not Learnt from current machine");
			//	ui.lineEdit_curPointLearntStatus->setStyleSheet("color: red;"); // Set text color to green
			//	ui.toolButton_setCurPoint->setDisabled(false);
			//}
		}
	}

    return ret;
}

void SystemData::setCurrentCoordinate(double x, double y, double z)
{
	std::unique_lock<std::shared_mutex> lock(mtx_currentCoordinate);
	m_currentCoordinate.wx = x;
	m_currentCoordinate.wy = y;
	m_currentCoordinate.wz = z;
}

const dat::WorldCoordinate& SystemData::currentCoordinate() const
{
	std::shared_lock<std::shared_mutex> lock(mtx_currentCoordinate);
	return m_currentCoordinate;
}


QString SystemData::getLaserType() const
{
	auto jsonPath = QStringLiteral("%1recipe\\%2\\recipeConfig.json").arg(Common::Directory::LocalPath).arg(Common::Directory::CurrentRecipe);
	qDebug() << jsonPath;
	QString laserApi = "";
	QJsonObject root;
	if (!jsonHelper::loadJson(jsonPath, root)) {
		ct::logger::info("Fail Getting LaserType, emtpy");
		return laserApi;  // return empty

	}
	else {
		ct::logger::info("Getting LaserType");
		laserApi = jsonHelper::getString(root, QStringLiteral("laserApi"));
	}

	return laserApi;
}

bool SystemData::saveLaserType(QString laser)
{
	auto jsonPath = QStringLiteral("%1recipe\\%2\\recipeConfig.json").arg(Common::Directory::LocalPath).arg(Common::Directory::CurrentRecipe);

	QJsonObject obj;
	obj.insert(QStringLiteral("laserApi"), laser);
	auto ret = jsonHelper::saveJson(jsonPath, QJsonDocument(obj));

	return ret;
}

