#include "ScaleManager.h"
#include <mutex>
#include "Utilities.h"
#include "Logger.h"
#include "SystemData.h"

void ScaleManager::update_um_per_px()
{
	m_um_per_px = (m_worldEnv.horizontal_scale + m_worldEnv.vertical_scale) / 2;
}

void ScaleManager::set_horizontal_um_per_px(double um_per_px)
{
	std::unique_lock lock(mutex_h);
	m_worldEnv.horizontal_scale = um_per_px;
	ct::logger::info("[ScaleManager] Horizontal um per px: %.6f", um_per_px);
	update_um_per_px();
}

double ScaleManager::horizontal_um_per_px() const
{
	std::shared_lock lock(mutex_h);
	return m_worldEnv.horizontal_scale;
}

void ScaleManager::set_vertical_um_per_px(double um_per_px)
{
	std::unique_lock lock(mutex_v);
	m_worldEnv.vertical_scale = um_per_px;
	ct::logger::info("[ScaleManager] Vertical um per px: %.6f", um_per_px);
	update_um_per_px();
}

double ScaleManager::vertical_um_per_px() const
{
	std::shared_lock lock(mutex_v);
	return m_worldEnv.vertical_scale;
}

double ScaleManager::um_per_px() const
{
	return m_um_per_px;
}

void ScaleManager::set_world_scale(double world_scale)
{
	std::unique_lock lock(mutex_worldScale);
	m_worldScale = world_scale;
	ct::logger::info("[ScaleManager] World scale: %.6f", world_scale);
}

double ScaleManager::world_scale() const
{
	std::shared_lock lock(mutex_worldScale);
	return m_worldScale;
}

void ScaleManager::set_world_env(const dat::WorldEnvironment& env)
{
	std::unique_lock lock(mutex_worldEnv);
	m_worldEnv = env;
	update_um_per_px();

	ct::logger::info("[ScaleManager] Horizontal um per px: %.6f", env.horizontal_scale);
	ct::logger::info("[ScaleManager] Vertical um per px: %.6f", env.vertical_scale);
	ct::logger::info("[ScaleManager] World scale: %.6f", m_worldScale);
}

dat::WorldEnvironment ScaleManager::world_env() const
{
	std::shared_lock lock(mutex_worldEnv);
	return m_worldEnv;
}

void ScaleManager::set_laser_fov_mm(double fov_mm)
{
	m_laserFov_mm = fov_mm;
}

double ScaleManager::mm_to_px(double mm)
{
	return mm / m_um_per_px * 1000;
}

double ScaleManager::um_to_px(double um)
{
	return um / m_um_per_px;
}

double ScaleManager::to_mm(double px)
{
	return px * m_um_per_px / 1000;
}

double ScaleManager::to_um(double px)
{
	return px * m_um_per_px;
}

double ScaleManager::mm_to_horizontal_px(double mm)
{
	return mm / m_worldEnv.horizontal_scale * 1000;
}

double ScaleManager::um_to_horizontal_px(double um)
{
	return um / m_worldEnv.horizontal_scale;
}

double ScaleManager::mm_to_vertical_px(double mm)
{
	return mm / m_worldEnv.vertical_scale * 1000;
}

double ScaleManager::um_to_vertical_px(double um)
{
	return um / m_worldEnv.vertical_scale;
}

double ScaleManager::to_horizontal_mm(double px)
{
	return px * m_worldEnv.horizontal_scale / 1000;
}

double ScaleManager::to_horizontal_um(double px)
{
	return px * m_worldEnv.horizontal_scale;
}

double ScaleManager::to_vertical_mm(double px)
{
	return px * m_worldEnv.vertical_scale / 1000;
}

double ScaleManager::to_vertical_um(double px)
{
	return px * m_worldEnv.vertical_scale;
}

QPointF ScaleManager::to_world_px(QPointF world_mm)
{
	double x_origin_px = 0;
	double y_origin_px = 0;
	double x_origin_mm = m_worldEnv.left;
	double y_origin_mm = m_worldEnv.front;

	QPointF point_px;
	auto horizontal_distance_mm = abs(world_mm.x() - x_origin_mm); //get distance in mm
	auto horizontal_distance_px = mm_to_px(horizontal_distance_mm);
	point_px.setX((horizontal_distance_px + x_origin_px) * m_worldScale); //offset from origin

	auto vertical_distance_mm = abs(world_mm.y() - y_origin_mm); //get distance in mm
	auto vertical_distance_px = mm_to_px(vertical_distance_mm);
	point_px.setY((vertical_distance_px + y_origin_px) * m_worldScale); //offset from origin

	return point_px;
}

QPointF ScaleManager::to_world_mm(QPointF world_px)
{
	double x_origin_px = 0;
	double y_origin_px = 0;
	double x_origin_mm = 0;
	double y_origin_mm = 0;

	QPointF point_mm;

	//TODO: Can add a vertical & horizontal axis for x and y, currently harcode x_mm as vertical, bad practice
	auto x_distance_px = abs(world_px.x() - x_origin_px) / m_worldScale; //get distance in px
	auto x_distance_mm = to_mm(x_distance_px);

	auto y_distance_px = abs(world_px.y() - y_origin_px) / m_worldScale; //get distance in px
	auto y_distance_mm = to_mm(y_distance_px);

	//horizontal
	point_mm.setX(x_origin_mm + x_distance_mm);

	//vertical
	point_mm.setY(y_origin_mm + y_distance_mm);

	return point_mm;
}

QRectF ScaleManager::world_to_fov(const QRectF& rect)
{
	if (util::is_equal(m_worldScale, 0, allowable_double_error)) return rect;
	QRectF FOVRect = QRectF(rect.x() / m_worldScale, rect.y() / m_worldScale, rect.width() / m_worldScale, rect.height() / m_worldScale);
	return FOVRect;
}

QRectF ScaleManager::fov_to_world(const QRectF& rect)
{
	if (util::is_equal(m_worldScale, 0, allowable_double_error)) return rect;
	QRectF WorldRect = QRectF(rect.x() * m_worldScale, rect.y() * m_worldScale, rect.width() * m_worldScale, rect.height() * m_worldScale);
	return WorldRect;
}

double ScaleManager::world_to_fov(const double& px)
{
	if (util::is_equal(m_worldScale, 0, allowable_double_error)) return px;
	double FOVpx = px / m_worldScale;
	return FOVpx;
}

double ScaleManager::fov_to_world(const double& px)
{
	if (util::is_equal(m_worldScale, 0, allowable_double_error)) return px;
	double Worldpx = px * m_worldScale;
	return Worldpx;
}

ct::Box2D ScaleManager::world_to_fov(const ct::Box2D& box)
{
	if (util::is_equal(m_worldScale, 0, allowable_double_error)) return box;

	ct::Box2D FOVBox = box;
	FOVBox.cx = (double)box.cx / m_worldScale;
	FOVBox.cy = (double)box.cy / m_worldScale;
	FOVBox.w = std::ceil((double)box.w / m_worldScale);
	FOVBox.h = std::ceil((double)box.h / m_worldScale);
	FOVBox.xmin = (double)box.xmin / m_worldScale;
	FOVBox.ymin = (double)box.ymin / m_worldScale;
	FOVBox.xmax = (double)box.xmax / m_worldScale;
	FOVBox.xmin = (double)box.xmin / m_worldScale;
	FOVBox.compute_extremum();
	return FOVBox; 
}

ct::Box2D ScaleManager::fov_to_world(const ct::Box2D& box)
{
	if (util::is_equal(m_worldScale, 0, allowable_double_error)) return box;

	ct::Box2D worldBox = box;
	worldBox.cx = (double)box.cx * m_worldScale;
	worldBox.cy = (double)box.cy * m_worldScale;
	worldBox.w = (double)box.w * m_worldScale;
	worldBox.h = (double)box.h * m_worldScale;
	worldBox.xmin = (double)box.xmin * m_worldScale;
	worldBox.ymin = (double)box.ymin * m_worldScale;
	worldBox.xmax = (double)box.xmax * m_worldScale;
	worldBox.xmin = (double)box.xmin * m_worldScale;
	worldBox.compute_extremum();
	return worldBox;
}

QPointF ScaleManager::to_fov_px(QLineScan linescan)
{
	//start_point sits on the scan axis edge and is centered on the laser FOV along the step axis
	auto topLeft_mm = SystemData::instance().isLineScanAxisY()
		? QPointF(linescan.start_point.wx - m_laserFov_mm / 2, linescan.start_point.wy)
		: QPointF(linescan.start_point.wx, linescan.start_point.wy - m_laserFov_mm / 2);
	auto wpx = to_world_px(topLeft_mm);
	auto xmin = wpx.x();
	auto ymin = wpx.y();
	xmin = world_to_fov(xmin);
	ymin = world_to_fov(ymin);
	return QPointF(xmin, ymin);
}

QPointF ScaleManager::to_world_px(QLineScan linescan)
{
	auto wpx = to_world_px(QPointF(linescan.start_point.wx, linescan.start_point.wy));
	auto xmin = wpx.x();
	auto ymin = wpx.y();
	return QPointF(xmin, ymin);
}

QPointF ScaleManager::to_fov_px(QView view)
{
	auto wpx = to_world_px(QPointF(view.world.wx, view.world.wy));
	auto px = fov_to_world(view.px);
	auto xmin = wpx.x() - px.w / 2;
	auto ymin = wpx.y() - px.h / 2;
	xmin = world_to_fov(xmin);
	ymin = world_to_fov(ymin);
	return QPointF(xmin, ymin);
}

QPointF ScaleManager::to_world_px(QView view)
{
	auto wpx = to_world_px(QPointF(view.world.wx, view.world.wy));
	auto px = fov_to_world(view.px);
	auto xmin = wpx.x() - px.w / 2;
	auto ymin = wpx.y() - px.h / 2;
	return QPointF(xmin, ymin);
}

bool ScaleManager::extract_json_object(const QJsonObject& root)
{
	auto assignWorldOrientation = [](QString value) -> dat::WorldOrientation {
		if (value == "right") return dat::WorldOrientation::RIGHT;
		else if (value == "left") return dat::WorldOrientation::LEFT;
		else if (value == "front") return dat::WorldOrientation::FRONT;
		else if (value == "back") return dat::WorldOrientation::BACK;
		else if (value == "top") return dat::WorldOrientation::TOP;
		else if (value == "bottom") return dat::WorldOrientation::BOTTOM;
		return dat::WorldOrientation::INVALID;
	};

	if (!root.contains("scale")) {
		ct::logger::error("[ScaleManager] Failed to extract json. Key not found: scale");
		return false;
	}

	auto scaleObj = root["scale"].toObject();

	if (!scaleObj.contains("world_environment")) {
		ct::logger::error("[ScaleManager] Failed to extract json. Key not found: world_environment");
		return false;
	}

	auto envObj = scaleObj["world_environment"].toObject();

	dat::WorldEnvironment env;
	env.left = jsonHelper::getDouble(envObj, "left");
	env.front = jsonHelper::getDouble(envObj, "front");
	env.right = jsonHelper::getDouble(envObj, "right");
	env.back = jsonHelper::getDouble(envObj, "back");
	env.top = jsonHelper::getDouble(envObj, "top");
	env.bottom = jsonHelper::getDouble(envObj, "bottom");
	env.x_axis = assignWorldOrientation(jsonHelper::getString(envObj, "x_axis"));
	env.y_axis = assignWorldOrientation(jsonHelper::getString(envObj, "y_axis"));
	env.z_axis = assignWorldOrientation(jsonHelper::getString(envObj, "z_axis"));

	env.horizontal_scale = jsonHelper::getDouble(scaleObj, "horizontal_um_per_px");
	env.vertical_scale = jsonHelper::getDouble(scaleObj, "vertical_um_per_px");

	m_worldScale = jsonHelper::getDouble(scaleObj, "world_scale");
	m_laserFov_mm = jsonHelper::getDouble(scaleObj, "laser_fov(mm)");

	set_world_env(env);
	
	return true;
}

QJsonObject ScaleManager::json_object()
{
	auto toQString = [](dat::WorldOrientation value) -> QString {
		if (value == dat::WorldOrientation::RIGHT) return "right";
		else if (value == dat::WorldOrientation::LEFT) return "left";
		else if (value == dat::WorldOrientation::FRONT) return "front";
		else if (value == dat::WorldOrientation::BACK) return "back";
		else if (value == dat::WorldOrientation::TOP) return "top";
		else if (value == dat::WorldOrientation::BOTTOM) return "bottom";
		return "INVALID";
	};

	QJsonObject obj;
	obj.insert(QStringLiteral("world_scale"), m_worldScale);
	obj.insert(QStringLiteral("laser_fov(mm)"), m_laserFov_mm);
	obj.insert(QStringLiteral("horizontal_um_per_px"), m_worldEnv.horizontal_scale);
	obj.insert(QStringLiteral("vertical_um_per_px"), m_worldEnv.vertical_scale);

	QJsonObject objEnv;
	objEnv.insert(QStringLiteral("left"), m_worldEnv.left);
	objEnv.insert(QStringLiteral("front"), m_worldEnv.front);
	objEnv.insert(QStringLiteral("right"), m_worldEnv.right);
	objEnv.insert(QStringLiteral("back"), m_worldEnv.back);
	objEnv.insert(QStringLiteral("top"), m_worldEnv.top);
	objEnv.insert(QStringLiteral("bottom"), m_worldEnv.bottom);
	objEnv.insert(QStringLiteral("x_axis"), toQString(m_worldEnv.x_axis));
	objEnv.insert(QStringLiteral("y_axis"), toQString(m_worldEnv.y_axis));
	objEnv.insert(QStringLiteral("z_axis"), toQString(m_worldEnv.z_axis));
	

	QJsonObject root;
	obj.insert(QStringLiteral("world_environment"), objEnv);
	root.insert(QStringLiteral("scale"), obj);

	return root;
}
