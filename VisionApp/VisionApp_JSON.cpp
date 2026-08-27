#include "VisionApp.h"
#include "CommonDir.h"
#include "AuditLog.h"
#include "TemplateLibraryTab.h"
#include "ImageViewerTab.h"
#include "UnitConfigTab.h"
#include "DatasetPage.h"
#include "ProductionPage.h"
#include "VIsionAppQDragBox.h"
#include "uidGenerator.h"
#include "QDragBox.h"
#include "CAMManager.h"
#include "ScaleManager.h"
#include"3DOpticsTab.h"
#include <algorithm>


void VisionApp::saveVisionObject()
{
	QJsonObject visionObj;
	QJsonObject recipeObj;
	QList<QVariant> visionObjList;
	QMap<QString, QVariant> recipeMap;

	//save vision object
	auto vo_keys = _visionObject.keys();
	qSort(vo_keys);

	for (auto key : vo_keys) {
		auto vo = _visionObject[key];

		visionObj.insert(QStringLiteral("objectName"), vo.objectName);
		visionObj.insert(QStringLiteral("objectID"), vo.objectID);

		visionObj.insert(QStringLiteral("templateName"), vo.templateName);
		visionObj.insert(QStringLiteral("templateID"), vo.templateID);

		visionObj.insert(QStringLiteral("View"), vo.viewID);
		visionObj.insert(QStringLiteral("lineScanID"), vo.lineScanID);

		visionObj.insert(QStringLiteral("skip"), vo.skip);
		visionObj.insert(QStringLiteral("ForcedSkip"), vo.forcedSkip);
		visionObj.insert(QStringLiteral("locked"), vo.locked);
		visionObj.insert(QStringLiteral("Angle"), vo.angle);

		QRectF voRect = ScaleManager::instance().world_to_fov(vo.pDragBox->getGeometry());

		if (g_viewMode == int(ViewMode::PLANE))
		{
			auto relativeCoordinates = getRelativeFOVCoordinates(QPointF(voRect.left(), voRect.top()));
			visionObj.insert(QStringLiteral("Roi_Left"), relativeCoordinates.x());
			visionObj.insert(QStringLiteral("Roi_Top"), relativeCoordinates.y());
		}
		else if(g_viewMode == int(ViewMode::SINGLE))
		{
			visionObj.insert(QStringLiteral("Roi_Left"), voRect.x());
			visionObj.insert(QStringLiteral("Roi_Top"), voRect.y());
		}
		

		if (vo.pDragBox->getGeometry().width() == 0)
			showMsg("Oops... object roi corrupted");

		visionObj.insert(QStringLiteral("Roi_Width"), voRect.width());
		visionObj.insert(QStringLiteral("Roi_Height"), voRect.height());
		
		visionObj.insert(QStringLiteral("Row"), vo.row);
		visionObj.insert(QStringLiteral("Col"), vo.col);
		visionObj.insert(QStringLiteral("Row_ID"), vo.row_id);
		visionObj.insert(QStringLiteral("Col_ID"), vo.col_id);
		visionObj.insert(QStringLiteral("Island"), vo.island);
		visionObj.insert(QStringLiteral("Island_ID"), vo.island_id);

		visionObjList.append(visionObj);
	}

	recipeObj.insert(QStringLiteral("Object"), QJsonArray::fromVariantList(visionObjList));
	recipeMap.insert(QStringLiteral("Recipe"), recipeObj);

	auto path = Common::Directory::LocalPath + QString("recipe/%1/visionObject.json").arg(Common::Directory::CurrentRecipe);
	saveJson(path, QJsonDocument::fromVariant(recipeMap));

	ct::logger::info("Save vision object: %s", path.toStdString().c_str());
}

void VisionApp::saveView()
{
	//save view
	QJsonObject viewObj;
	QJsonObject pxObj;
	QJsonObject worldObj;
	QList<QVariant> viewList;
	QJsonObject recipeViewObj;
	QList<QVariant> objList;

	auto view_keys = _views.keys();
	qSort(view_keys);

	for (auto key : view_keys) {

		auto view = _views[key];
		if (view.id.isEmpty()) continue;

		viewObj.insert(QStringLiteral("id"), view.id);
		viewObj.insert(QStringLiteral("name"), view.name);
		viewObj.insert(QStringLiteral("camID"), view.camID);
		viewObj.insert(QStringLiteral("type"), view.type);
		viewObj.insert(QStringLiteral("created_by"), view.created_by);
		viewObj.insert(QStringLiteral("map_to_sview"), view.map_to_sview);
		viewObj.insert(QStringLiteral("horizontal_scale"), view.horizontal_scale);
		viewObj.insert(QStringLiteral("vertical_scale"), view.vertical_scale);

		double cx = view.px.cx;
		double cy = view.px.cy;
		double w = view.px.w;
		double h = view.px.h;
		double xmin = view.px.xmin;
		double ymin = view.px.ymin;
		double xmax = view.px.xmax;
		double ymax = view.px.ymax;

		if (g_viewMode == int(ViewMode::PLANE))
		{
			auto vpxc = getRelativeFOVCoordinates(QPointF(view.px.cx, view.px.cy));
			cx = vpxc.x();
			cy = vpxc.y();
			auto vpxmin = getRelativeFOVCoordinates(QPointF(view.px.xmin, view.px.ymin));
			xmin = vpxmin.x();
			ymin = vpxmin.y();
			auto vpxmax = getRelativeFOVCoordinates(QPointF(view.px.xmax, view.px.ymax));
			xmax = vpxmax.x();
			ymax = vpxmax.y();
		}

		pxObj.insert(QStringLiteral("cx"), cx);
		pxObj.insert(QStringLiteral("cy"), cy);
		pxObj.insert(QStringLiteral("w"), w);
		pxObj.insert(QStringLiteral("h"), h);
		pxObj.insert(QStringLiteral("xmin"), xmin);
		pxObj.insert(QStringLiteral("ymin"), ymin);
		pxObj.insert(QStringLiteral("xmax"), xmax);
		pxObj.insert(QStringLiteral("ymax"), ymax);
		viewObj.insert(QStringLiteral("px_coordinate"), pxObj);

		toJson(view.world, worldObj);
		/*worldObj.insert(QStringLiteral("wx"), view.world.wx);
		worldObj.insert(QStringLiteral("wy"), view.world.wy);
		worldObj.insert(QStringLiteral("wz"), view.world.wz);*/
		worldObj.insert(QStringLiteral("rx"), view.world.rx);
		worldObj.insert(QStringLiteral("ry"), view.world.ry);
		worldObj.insert(QStringLiteral("rz"), view.world.rz);
		viewObj.insert(QStringLiteral("world_coordinate"), worldObj);

		QList<QVariant> objList;
		for (auto obj : view.vision_obj_IDs)
		{
			objList.append(obj);
		}
		viewObj.insert(QStringLiteral("vision_obj_ids"), QJsonArray::fromVariantList(objList));

		//zstack
		QJsonObject preObj;
		preObj.insert(QStringLiteral("crop"), view.preprocess.crop);
		preObj.insert(QStringLiteral("resize"), view.preprocess.resize);
		preObj.insert(QStringLiteral("flatfield"), view.preprocess.flatfield);
		preObj.insert(QStringLiteral("crop_x"), view.preprocess.cropRect.x());
		preObj.insert(QStringLiteral("crop_y"), view.preprocess.cropRect.y());
		preObj.insert(QStringLiteral("crop_w"), view.preprocess.cropRect.width());
		preObj.insert(QStringLiteral("crop_h"), view.preprocess.cropRect.height());
		preObj.insert(QStringLiteral("resize_w"), view.preprocess.resizeRect.width());
		preObj.insert(QStringLiteral("resize_h"), view.preprocess.resizeRect.height());
		viewObj.insert(QStringLiteral("preprocess"), preObj);

		//zstack
		QJsonObject zstackObj;
		zstackObj.insert(QStringLiteral("acq_type"), view.zstack.acq_type);
		zstackObj.insert(QStringLiteral("step_um"), view.zstack.step_um);
		zstackObj.insert(QStringLiteral("preset_iteration"), view.zstack.preset_iteration);
		zstackObj.insert(QStringLiteral("encoder_range_um"), view.zstack.encoder_range_um);
		zstackObj.insert(QStringLiteral("time_interval_ms"), view.zstack.time_interval_ms);
		zstackObj.insert(QStringLiteral("generate_2D_stack"), view.zstack.generate_2D_stack);
		zstackObj.insert(QStringLiteral("generate_3D_stack"), view.zstack.generate_3D_stack);
		viewObj.insert(QStringLiteral("zstack"), zstackObj);

		//opticIDs
		QJsonArray opticIDsArr;
		for (auto opticID : view.opticIDs)
		{
			opticIDsArr.push_back(opticID);
			
		}
		viewObj.insert(QStringLiteral("opticIDs"), opticIDsArr);

		viewList.append(viewObj);
	}

	recipeViewObj.insert(QStringLiteral("views"), QJsonArray::fromVariantList(viewList));

	auto path = Common::Directory::LocalPath + QString("recipe/%1/view.json").arg(Common::Directory::CurrentRecipe);
	saveJson(path, QJsonDocument(recipeViewObj));

	ct::logger::info("Save view: %s", path.toStdString().c_str());
}

void VisionApp::saveLineScans()
{
	//save view
	QJsonObject obj;
	QJsonObject pxObj;
	QJsonObject startPointObj;
	QJsonObject endPointObj;
	QList<QVariant> list;
	QJsonObject recipeViewObj;
	QList<QVariant> objList;

	auto keys = _lineScans.keys();
	qSort(keys);

	for (const auto& key : keys) {

		auto data = _lineScans[key];

		obj.insert(QStringLiteral("id"), data.id);
		obj.insert(QStringLiteral("name"), data.name);
		obj.insert(QStringLiteral("created_by"), data.created_by);
		obj.insert(QStringLiteral("type"), data.type);
		obj.insert(QStringLiteral("map_to_slinescan"), data.map_to_slinescan);

		toJson(data.px, pxObj);
		obj.insert(QStringLiteral("px_coordinate"), pxObj);

		toJson(data.start_point, startPointObj);
		obj.insert(QStringLiteral("start_point"), startPointObj);

		toJson(data.end_point, endPointObj);
		obj.insert(QStringLiteral("end_point"), endPointObj);

		QList<QVariant> objList;
		for (auto obj : data.vision_obj_IDs)
		{
			objList.append(obj);
		}
		obj.insert(QStringLiteral("vision_obj_ids"), QJsonArray::fromVariantList(objList));

		list.append(obj);
	}

	recipeViewObj.insert(QStringLiteral("line_scans"), QJsonArray::fromVariantList(list));
	// RECIPE_Z_CONVEYOR_DISABLED_BEGIN
	// Recipe-based 3D Z offset persistence is disabled. Re-enable this insert to
	// save z_offset into linescans.json again.
	//recipeViewObj.insert(QStringLiteral("z_offset"), m_currentZOffset);
	// RECIPE_Z_CONVEYOR_DISABLED_END
	auto path = Common::Directory::LocalPath + QString("recipe/%1/linescans.json").arg(Common::Directory::CurrentRecipe);
	saveJson(path, QJsonDocument(recipeViewObj));

	ct::logger::info("Save linescan: %s", path.toStdString().c_str());
}

bool VisionApp::saveIslandInfo()
{
	QJsonObject islandInfoObj;

	islandInfoObj.insert(QStringLiteral("prefix"), _islandInfo.prefix);
	islandInfoObj.insert(QStringLiteral("postfix"), _islandInfo.postfix);
	islandInfoObj.insert(QStringLiteral("rowStartingIndex"), QString::number(_islandInfo.rowStartingIndex));
	islandInfoObj.insert(QStringLiteral("colStartingIndex"), QString::number(_islandInfo.colStartingIndex));
	islandInfoObj.insert(QStringLiteral("totalRow"), QString::number(_islandInfo.totalRow));
	islandInfoObj.insert(QStringLiteral("rowPitch"), QString::number(_islandInfo.rowPitch));
	islandInfoObj.insert(QStringLiteral("totalCol"), QString::number(_islandInfo.totalCol));
	islandInfoObj.insert(QStringLiteral("colPitch"), QString::number(_islandInfo.colPitch));
	islandInfoObj.insert(QStringLiteral("rotation"), QString::number(_islandInfo.rotation));
	islandInfoObj.insert(QStringLiteral("totalIsland"), QString::number(_islandInfo.totalIsland));

	saveJson(Common::Directory::LocalPath + QString("recipe/%1/islandInfo.json").arg(Common::Directory::CurrentRecipe), QJsonDocument(islandInfoObj));

	return true;
}

void VisionApp::saveRecipe()
{
	if (!Common::Directory::CurrentRecipe.isEmpty())
	{
		
		saveVisionObject();
		saveView();
		saveLineScans();
		saveIslandInfo();
		savePathInfo();
		savePlane();
		saveBarcode();
		saveFiducial();
		saveRecipeOptics();
		saveRecipeConfig();
		saveRecipeMotion();
		AuditLog::instance().log(QStringLiteral("RECIPE_SAVE"), Common::Directory::CurrentRecipe);
		showStatus(QStringLiteral("Recipe saved"));

		udpateRecipeVersion(Common::Directory::getRecipeCurrentPath());

	}
	else
	{
		showMsg(QStringLiteral("Open a recipe to continue"));
	}
}

bool VisionApp::saveWorldEnv()
{
	auto jsonPath = QStringLiteral("%1config/world.json").arg(Common::Directory::LocalPath);

	auto toQString = [](dat::WorldOrientation value) -> QString {
		if (value == dat::WorldOrientation::RIGHT) return "right";
		else if (value == dat::WorldOrientation::LEFT) return "left";
		else if (value == dat::WorldOrientation::FRONT) return "front";
		else if (value == dat::WorldOrientation::BACK) return "back";
		else if (value == dat::WorldOrientation::TOP) return "top";
		else if (value == dat::WorldOrientation::BOTTOM) return "bottom";
		return "INVALID";
	};

	auto env = ScaleManager::instance().world_env();

	//save view
	QJsonObject obj;
	obj.insert(QStringLiteral("left"), env.left);
	obj.insert(QStringLiteral("front"), env.front);
	obj.insert(QStringLiteral("right"), env.right);
	obj.insert(QStringLiteral("back"), env.back);
	obj.insert(QStringLiteral("top"), env.top);
	obj.insert(QStringLiteral("bottom"), env.bottom);
	obj.insert(QStringLiteral("horizontal_scale"), env.horizontal_scale);
	obj.insert(QStringLiteral("vertical_scale"), env.vertical_scale);
	obj.insert(QStringLiteral("x_axis"), toQString(env.x_axis));
	obj.insert(QStringLiteral("y_axis"), toQString(env.y_axis));
	obj.insert(QStringLiteral("z_axis"), toQString(env.z_axis));

	auto ret = saveJson(jsonPath, QJsonDocument(obj));

	if (ret) showStatus(QStringLiteral("Successfully saved world's environment!"));
	else showStatus(QStringLiteral("Failed to save world's environment!"));

	return ret;
}

bool VisionApp::loadWorldEnv()
{
	double maxSize = 20000;

	qDebug() << "SystemData::instance()._useRecipeScale:" << SystemData::instance()._useRecipeScale;

	if (SystemData::instance()._useRecipeScale) {

		auto initJsonPath = QStringLiteral("%1recipe/%2/init.json").arg(Common::Directory::LocalPath).arg(Common::Directory::CurrentRecipe);
		qDebug() << "initJsonPath:" << initJsonPath;
		QJsonObject rootInit;

		if (loadJson(initJsonPath, rootInit)) {
			if (ScaleManager::instance().extract_json_object(rootInit)) {

				auto worldScale = ScaleManager::instance().world_scale();
				auto env = ScaleManager::instance().world_env();

				auto worldW = abs(env.left - env.right);
				auto worldH = abs(env.front - env.back);
	
				auto imageW = ScaleManager::instance().mm_to_px(worldW);
				auto imageH = ScaleManager::instance().mm_to_px(worldH);
				
				ct::logger::info("[ScaleManager] World scale: %f", worldScale);

				ScaleManager::instance().set_world_scale(worldScale);

				imageW = imageW * worldScale;
				imageH = imageH * worldScale;
				
				_imageWorld = QImage(imageW, imageH, QImage::Format_RGB32);
				_imageWorld.fill(Qt::black);

				displayImage(_imageWorld);

				_worldFOV.setOutterBarrier(_sceneBound);

				return true;
			}
			else {
				ct::logger::error("Failed to extract scale from recipe init.json");
			}
		}
		else {
			ct::logger::error("Failed to open: %s, load world.json instead", initJsonPath.toStdString().c_str());
		}
	}


	qDebug() << "Loading world env...";
	auto jsonPath = QStringLiteral("%1config/world.json").arg(Common::Directory::LocalPath);
	QJsonObject root;

	auto assignWorldOrientation = [](QString value) -> dat::WorldOrientation {
		if (value == "right") return dat::WorldOrientation::RIGHT;
		else if (value == "left") return dat::WorldOrientation::LEFT;
		else if (value == "front") return dat::WorldOrientation::FRONT;
		else if (value == "back") return dat::WorldOrientation::BACK;
		else if (value == "top") return dat::WorldOrientation::TOP;
		else if (value == "bottom") return dat::WorldOrientation::BOTTOM;
		return dat::WorldOrientation::INVALID;
	};

	if (loadJson(jsonPath, root)) {
		dat::WorldEnvironment env;
		env.left = jsonHelper::getDouble(root, "left");
		env.front = jsonHelper::getDouble(root, "front");
		env.right = jsonHelper::getDouble(root, "right");
		env.back = jsonHelper::getDouble(root, "back");
		env.top = jsonHelper::getDouble(root, "top");
		env.bottom = jsonHelper::getDouble(root, "bottom");
		env.horizontal_scale = jsonHelper::getDouble(root, "horizontal_scale");
		env.vertical_scale = jsonHelper::getDouble(root, "vertical_scale");
		env.x_axis = assignWorldOrientation(jsonHelper::getString(root, "x_axis"));
		env.y_axis = assignWorldOrientation(jsonHelper::getString(root, "y_axis"));
		env.z_axis = assignWorldOrientation(jsonHelper::getString(root, "z_axis"));

		ScaleManager::instance().set_world_env(env);

		ui.lineEdit_horizontalScale->setText(QString::number(env.horizontal_scale));
		ui.lineEdit_verticalScale->setText(QString::number(env.vertical_scale));

		//image
		auto worldW = abs(env.left - env.right);
		auto worldH = abs(env.front - env.back);
		printf("World (mm): %f, %f", worldW, worldH);
		auto imageW = ScaleManager::instance().mm_to_px(worldW);
		auto imageH = ScaleManager::instance().mm_to_px(worldH);
		printf("World (px): %f, %f", imageW, imageH);

		double worldScale = 1.0;

		if (g_viewMode == int(ViewMode::SINGLE))
		{
			ScaleManager::instance().set_world_scale(1);
		}
		else
		{
			if (imageW > maxSize || imageH > maxSize) {
				if (imageW > imageH) {
					worldScale = maxSize / imageW;
				}
				else {
					worldScale = maxSize / imageH;
				}
			}
			else {
				worldScale = 1;
			}

			ScaleManager::instance().set_world_scale(worldScale);
		}

		auto userdefined_worldScale = jsonHelper::getDouble(root, "world_scale", 0.0);
		
		qDebug() << "Not User Defined WorldScale:" << worldScale;

		if (userdefined_worldScale > 0.00001) {
			worldScale = userdefined_worldScale;
			ScaleManager::instance().set_world_scale(worldScale);
		}

		//_worldScale = 0.291716; 12MP
		//_worldScale = 0.108155; 25MP
		qDebug() << "imageW:" << imageW;
		qDebug() << "imageH:" << imageH;
		qDebug() << "WorldScale:" << worldScale;

		imageW = imageW * worldScale;
		imageH = imageH * worldScale;
		printf("World scaled (px): %f, %f", imageW, imageH);

		_imageWorld = QImage(imageW, imageH, QImage::Format_RGB32);
		_imageWorld.fill(Qt::black);

		displayImage(_imageWorld);

		_worldFOV.setOutterBarrier(_sceneBound);
	}
	else {
		ct::logger::error("Failed to open: world.json");
		ct::logger::info("[ScaleManager] World scale: %f", ScaleManager::instance().world_scale());
		return false;
	}

	ct::logger::info("[ScaleManager] World scale: %f", ScaleManager::instance().world_scale());
	return true;
}

bool VisionApp::savePathInfo()
{
	auto jsonPath = QStringLiteral("%1recipe/%2/path.json").arg(Common::Directory::LocalPath).arg(Common::Directory::CurrentRecipe);

	//save view
	QJsonArray ivArr;
	for (int i = 0; i < ui.listWidget_viewSelection->count(); i++) {
		auto item = ui.listWidget_viewSelection->item(i);
		if (item->checkState() == Qt::Checked) {
			auto id = item->whatsThis();
			ivArr.push_back(id);
		}
	}

	QJsonArray pArr;
	for (int i = 0; i < ui.listWidget_paths->count(); i++) {
		auto id = ui.listWidget_paths->item(i)->whatsThis();
		pArr.push_back(id);
	}

	QJsonObject obj;
	obj.insert(QStringLiteral("startID"), ui.tb_setStartPoint->whatsThis());
	obj.insert(QStringLiteral("endID"), ui.tb_setEndPoint->whatsThis());
	obj.insert(QStringLiteral("included_views"), ivArr);
	obj.insert(QStringLiteral("paths"), pArr);

	auto ret = saveJson(jsonPath, QJsonDocument(obj));

	if (ret) showStatus(QStringLiteral("Successfully saved path's sequence!"));
	else showStatus(QStringLiteral("Failed to save path's sequence!"));

	updateTreeViewExplorer(Common::Directory::CurrentRecipe, _views, _visionObject);

	return ret;
}

bool VisionApp::loadPathInfo()
{
	auto jsonPath = QStringLiteral("%1recipe/%2/path.json").arg(Common::Directory::LocalPath).arg(Common::Directory::CurrentRecipe);

	QJsonObject root;

	if (loadJson(jsonPath, root)) {
		auto startID = jsonHelper::getString(root, "startID", "");
		auto endID = jsonHelper::getString(root, "endID", "");

		if (!_views.contains(startID)) {
			ct::logger::warn("Starting view not found, please reassign path before running!");
			return false;
		}

		if (!_views.contains(endID)) {
			ct::logger::warn("Ending view not found, please reassign path before running!");
			return false;
		}

		if (startID != "") {
			ct::logger::debug("Start ID: %s", startID.toStdString().c_str());
			ui.tb_setStartPoint->setWhatsThis(startID);
			ui.tb_setStartPoint->setText(_views[startID].name);
		}

		if (endID != "") {
			ct::logger::debug("End ID: %s", endID.toStdString().c_str());
			ui.tb_setEndPoint->setWhatsThis(endID);
			ui.tb_setEndPoint->setText(_views[endID].name);
		}
		auto ivArr = jsonHelper::getArray(root, "included_views");
		auto pArr = jsonHelper::getArray(root, "paths");

		ui.listWidget_viewSelection->clear();
		ui.listWidget_paths->clear();

		QSet<QString> sets;
		for (auto iv : ivArr) {
			if (iv.isString()) {
				sets.insert(iv.toString());
			}
		}
		for (const auto& v : _views) {
			QListWidgetItem* item = new QListWidgetItem;
			item->setWhatsThis(v.id);
			item->setText(v.name);

			if (sets.contains(v.id)) {
				item->setCheckState(Qt::Checked);
			}
			else {
				item->setCheckState(Qt::Unchecked);
			}
			
			ui.listWidget_viewSelection->addItem(item);
		}

		for (auto p : pArr) {
			if (p.isString()) {
				addViewToPath(p.toString());
			}
		}
		//updatePathToUI();
	}
	else {
		ct::logger::warn("Failed to open: path.json");
		return false;
	}

	ct::logger::info("Loaded path info");
	return true;
}


bool VisionApp::savePlane()
{
	auto jsonPath = QStringLiteral("%1recipe/%2/plane.json").arg(Common::Directory::LocalPath).arg(Common::Directory::CurrentRecipe);

	//save view
	QJsonObject obj;
	obj.insert(QStringLiteral("id"), _plane.id);
	obj.insert(QStringLiteral("name"), _plane.name);
	obj.insert(QStringLiteral("created_by"), _plane.created_by);
	obj.insert(QStringLiteral("horizontal_num"), _plane.horizontal_num);
	obj.insert(QStringLiteral("vertical_num"), _plane.vertical_num);
	obj.insert(QStringLiteral("width_px"), ScaleManager::instance().world_to_fov(_plane.width_px));
	obj.insert(QStringLiteral("height_px"), ScaleManager::instance().world_to_fov(_plane.height_px));
	obj.insert(QStringLiteral("width_mm"), _plane.width_mm);
	obj.insert(QStringLiteral("height_mm"), _plane.height_mm);
	obj.insert(QStringLiteral("horizontal_overlap_percentage"), _plane.horizontal_overlap_percentage);
	obj.insert(QStringLiteral("vertical_overlap_percentage"), _plane.vertical_overlap_percentage);
	obj.insert(QStringLiteral("horizontal_overlap_px"), ScaleManager::instance().world_to_fov(_plane.horizontal_overlap_px));
	obj.insert(QStringLiteral("vertical_overlap_px"), ScaleManager::instance().world_to_fov(_plane.vertical_overlap_px));
	obj.insert(QStringLiteral("horizontal_overlap_mm"), _plane.horizontal_overlap_mm);
	obj.insert(QStringLiteral("vertical_overlap_mm"), _plane.vertical_overlap_mm);

	QJsonObject fl;
	toJson(_plane.corner_points[(int)Corner::FRONTLEFT], fl);
	/*fl.insert(QStringLiteral("wx"), _plane.corner_points[(int)Corner::FRONTLEFT].wx);
	fl.insert(QStringLiteral("wy"), _plane.corner_points[(int)Corner::FRONTLEFT].wy);
	fl.insert(QStringLiteral("wz"), _plane.corner_points[(int)Corner::FRONTLEFT].wz);*/

	QJsonObject br;
	toJson(_plane.corner_points[(int)Corner::BACKRIGHT], br);
	/*br.insert(QStringLiteral("wx"), _plane.corner_points[(int)Corner::BACKRIGHT].wx);
	br.insert(QStringLiteral("wy"), _plane.corner_points[(int)Corner::BACKRIGHT].wy);
	br.insert(QStringLiteral("wz"), _plane.corner_points[(int)Corner::BACKRIGHT].wz);*/

	obj.insert(QStringLiteral("front_left"), fl);
	obj.insert(QStringLiteral("back_right"), br);

	QList<QVariant> viewList;
	for (auto v : _plane.views) {
		QJsonObject vObj;
		toJson(v, vObj);
		viewList.append(vObj);
	}

	obj.insert(QStringLiteral("views"), QJsonArray::fromVariantList(viewList));

	auto ret = saveJson(jsonPath, QJsonDocument(obj));

	if (ret) showStatus(QStringLiteral("Successfully saved setup region"));
	else showStatus(QStringLiteral("Failed to save setup region"));

	return ret;
}

bool VisionApp::loadPlane()
{
	qDebug() << "loading plane...";
	auto jsonPath = QStringLiteral("%1recipe/%2/plane.json").arg(Common::Directory::LocalPath).arg(Common::Directory::CurrentRecipe);

	QJsonObject root;

	if (loadJson(jsonPath, root)) {
		_plane.id = jsonHelper::getString(root, QStringLiteral("id"));
		_plane.name = jsonHelper::getString(root, QStringLiteral("name"));
		_plane.created_by = jsonHelper::getString(root, QStringLiteral("created_by"));
		_plane.horizontal_num = jsonHelper::getInteger(root, QStringLiteral("horizontal_num"));
		_plane.vertical_num = jsonHelper::getInteger(root, QStringLiteral("vertical_num"));
		_plane.width_px = ScaleManager::instance().fov_to_world(jsonHelper::getDouble(root, QStringLiteral("width_px")));
		_plane.height_px = ScaleManager::instance().fov_to_world(jsonHelper::getDouble(root, QStringLiteral("height_px")));
		_plane.width_mm = jsonHelper::getDouble(root, QStringLiteral("width_mm"));
		_plane.height_mm = jsonHelper::getDouble(root, QStringLiteral("height_mm"));
		_plane.horizontal_overlap_percentage = jsonHelper::getDouble(root, QStringLiteral("horizontal_overlap_percentage"));
		_plane.vertical_overlap_percentage = jsonHelper::getDouble(root, QStringLiteral("vertical_overlap_percentage"));
		_plane.horizontal_overlap_px = ScaleManager::instance().fov_to_world(jsonHelper::getDouble(root, QStringLiteral("horizontal_overlap_px")));
		_plane.vertical_overlap_px = ScaleManager::instance().fov_to_world(jsonHelper::getDouble(root, QStringLiteral("vertical_overlap_px")));
		_plane.horizontal_overlap_mm = jsonHelper::getDouble(root, QStringLiteral("horizontal_overlap_mm"));
		_plane.vertical_overlap_mm = jsonHelper::getDouble(root, QStringLiteral("vertical_overlap_mm"));
	
		if (root.contains("front_left")) {
			auto fl = root["front_left"].toObject();
			fromJson(fl, _plane.corner_points[(int)Corner::FRONTLEFT]);
		/*	_plane.corner_points[(int)Corner::FRONTLEFT].wx = jsonHelper::getDouble(fl, QStringLiteral("wx"));
			_plane.corner_points[(int)Corner::FRONTLEFT].wy = jsonHelper::getDouble(fl, QStringLiteral("wy"));
			_plane.corner_points[(int)Corner::FRONTLEFT].wz = jsonHelper::getDouble(fl, QStringLiteral("wz"));*/
			ui.lineEdit_viewZ->setText(QString::number(_plane.corner_points[(int)Corner::FRONTLEFT].wz));
			_recipeZ = _plane.corner_points[(int)Corner::FRONTLEFT].wz;
		}

		if (root.contains("back_right")) {
			auto br = root["back_right"].toObject();
			fromJson(br, _plane.corner_points[(int)Corner::BACKRIGHT]);
			/*_plane.corner_points[(int)Corner::BACKRIGHT].wx = jsonHelper::getDouble(br, QStringLiteral("wx"));
			_plane.corner_points[(int)Corner::BACKRIGHT].wy = jsonHelper::getDouble(br, QStringLiteral("wy"));
			_plane.corner_points[(int)Corner::BACKRIGHT].wz = jsonHelper::getDouble(br, QStringLiteral("wz"));*/
		}

		if (root.contains("views")) {

			_plane.views.clear();
			QSet<QString> planeMap;
			auto viewsArray = root["views"].toArray();
			for (auto vObj : viewsArray)
			{
				QView v;
				fromJson(vObj.toObject(), v);

				if (planeMap.contains(v.id)) continue;

				planeMap.insert(v.id);
				_plane.views.emplace_back(v);
			}

			/*auto viewsArray = root["views"].toArray();
			for (auto vObj : viewsArray)
			{
				QView v;
				fromJson(vObj.toObject(), v);
				_plane.views.emplace_back(v);
			}*/
		}

		displayWorld();
	}
	else {
		//showMsg(QStringLiteral("Failed to open: plane.json"));
		// Regresh _plane 
		QViewPlane p;
		_plane = p;

		displayWorld();
		ct::logger::error("Failed to open: plane.json\n");
		return false;
	}

	return true;
}

bool VisionApp::saveRecipeOptics()
{
	auto jsonPath = QStringLiteral("%1recipe/%2/optics.json").arg(Common::Directory::LocalPath).arg(Common::Directory::CurrentRecipe);

	QJsonArray j_optics;

	for (auto o : _recipeOptics) {
		QJsonObject j_optic;
		j_optic.insert(QStringLiteral("id"), o.id);
		j_optic.insert(QStringLiteral("name"), o.name);
		j_optic.insert(QStringLiteral("type"), o.type);
		j_optic.insert(QStringLiteral("tag"), o.tag);
		j_optic.insert(QStringLiteral("exposure"), o.exposure);
		j_optic.insert(QStringLiteral("gain"), o.gain);
		j_optic.insert(QStringLiteral("camID"), o.camID);

		j_optic.insert(QStringLiteral("segmentPriority"), o.segmentPriority);
		//j_optic.insert(QStringLiteral("segmentReference"), o.segmentReference);
		j_optic.insert(QStringLiteral("segmentNum"), o.segmentNum);

		QJsonArray j_segmentRGBs;
		for (auto s : o.segmentRGBs) {
			QJsonObject j_RGB;
			j_RGB.insert(QStringLiteral("R"), s[0]);
			j_RGB.insert(QStringLiteral("G"), s[1]);
			j_RGB.insert(QStringLiteral("B"), s[2]);
			j_segmentRGBs.append(j_RGB);
		}
		j_optic.insert(QStringLiteral("segmentRGBs"), j_segmentRGBs);

		QJsonObject j_R;
		saveOpticBand(j_R, o.R);
		j_optic.insert(QStringLiteral("R"), j_R);

		QJsonObject j_G;
		saveOpticBand(j_G, o.G);
		j_optic.insert(QStringLiteral("G"), j_G);

		QJsonObject j_B;
		saveOpticBand(j_B, o.B);
		j_optic.insert(QStringLiteral("B"), j_B);

		QJsonObject j_M;
		saveOpticBand(j_M, o.M);
		j_optic.insert(QStringLiteral("M"), j_M);

		j_optics.append(j_optic);
	}

	QJsonArray j_bandBrightness;

	for (auto key : _recipeBandBrightness.keys()) {

		const auto& o = _recipeBandBrightness[key];

		QJsonObject j_optic;
		j_optic.insert(QStringLiteral("id"), key);
		j_optic.insert(QStringLiteral("exposure"), o.exposure);
		j_optic.insert(QStringLiteral("gain"), o.gain);

		j_bandBrightness.append(j_optic);
	}

	//CSA
	QJsonObject j_CSA;
	j_CSA.insert(QStringLiteral("opticID"), _CSA.opticID);
	j_CSA.insert(QStringLiteral("viewRef"), _CSA.viewRef);

	QJsonObject j_learnLocator;
	toJson(_CSA.learnLocator, j_learnLocator);

	QJsonArray j_searchLocators;
	for (int i = 0; i < _CSA.searchLocator.size(); i++) {
		QJsonObject j_searchLocator;
		toJson(_CSA.searchLocator[i], j_searchLocator);

		auto id = _CSA.searchLocator[i]->getId();

		if (_CSA.teachPoints.contains(id)) {
			auto pos = _CSA.teachPoints[id];
			j_searchLocator.insert("teachPointX", pos.x());
			j_searchLocator.insert("teachPointY", pos.y());
		}

		j_searchLocators.append(j_searchLocator);
	}

	j_CSA.insert(QStringLiteral("learnLocator"), j_learnLocator);
	j_CSA.insert(QStringLiteral("searchLocators"), j_searchLocators);

	//3D
	QJsonArray j_optics3D;
	for (auto o : _recipeOptics3D) {
		QJsonObject j_optic;
		j_optic.insert(QStringLiteral("id"), o.id);
		j_optic.insert(QStringLiteral("name"), o.name);
		j_optic.insert(QStringLiteral("tag"), o.tag);
		j_optic.insert(QStringLiteral("exposure"), o.exposure);
		j_optic.insert(QStringLiteral("intensity"), o.intensity);
		j_optic.insert(QStringLiteral("exposureMode"), o.exposureMode);
		j_optic.insert(QStringLiteral("exposure2"), o.exposure2);
		j_optic.insert(QStringLiteral("gain"), o.gain);
		j_optic.insert(QStringLiteral("gain2"), o.gain2);
		j_optic.insert(QStringLiteral("lineThreshold"), o.lineThreshold);
		j_optic.insert(QStringLiteral("divider"), o.divider);
		j_optic.insert(QStringLiteral("lowerLaserLimit"), o.lowerLaserLimit);
		j_optic.insert(QStringLiteral("upperLaserLimit"), o.upperLaserLimit);
		j_optic.insert(QStringLiteral("lightSensitivity"), o.lightSensitivity);
		j_optic.insert(QStringLiteral("peakSensitivity"), o.peakSensitivity);
		j_optic.insert(QStringLiteral("peakSelection"), o.peakSelection);

		
		j_optics3D.append(j_optic);
	}

	QJsonObject obj;
	obj.insert(QStringLiteral("optics"), j_optics);
	obj.insert(QStringLiteral("bandBrightness"), j_bandBrightness);
	obj.insert(QStringLiteral("optics3D"), j_optics3D);
	obj.insert(QStringLiteral("CSA"), j_CSA);

	auto ret = saveJson(jsonPath, QJsonDocument(obj));

	if (ret) showStatus(QStringLiteral("Successfully saved optics!"));
	else showStatus(QStringLiteral("Failed to save optics!"));

	return ret;
}

bool VisionApp::loadRecipeOptics()
{
	auto jsonPath = QStringLiteral("%1recipe/%2/optics.json").arg(Common::Directory::LocalPath).arg(Common::Directory::CurrentRecipe);

	QJsonObject root;

	if (loadJson(jsonPath, root)) {
		if (root.contains("optics")) {
			auto j_optics = root["optics"].toArray();

			_recipeOptics.clear();
			ui.listWidget_recipeOptics->clear();
			ui.listWidget_viewOpticSelection->clear();

			bool hasMainOptic = false;

			for (auto j_optic : j_optics)
			{
				auto obj = j_optic.toObject();
				
				OpticsInfo optic;
				optic.id = jsonHelper::getString(obj, QStringLiteral("id"));
				optic.name = jsonHelper::getString(obj, QStringLiteral("name"));
				optic.type = jsonHelper::getString(obj, QStringLiteral("type"));
				optic.tag = jsonHelper::getString(obj, QStringLiteral("tag"));
				optic.exposure = jsonHelper::getDouble(obj, QStringLiteral("exposure"), 20000);
				optic.gain = jsonHelper::getDouble(obj, QStringLiteral("gain"), 2);
				optic.camID = jsonHelper::getString(obj, QStringLiteral("camID"), "cam1");
				if (optic.camID.isEmpty()) {
					optic.camID = "cam1";
				}

				optic.segmentPriority = jsonHelper::getInteger(obj, QStringLiteral("segmentPriority"), -1);
				//optic.segmentReference = jsonHelper::getString(obj, QStringLiteral("segmentReference"), "");
				optic.segmentNum = jsonHelper::getInteger(obj, QStringLiteral("segmentNum"), 3);

				if (obj.contains("segmentRGBs")) {
					auto j_segmentRGBs = obj["segmentRGBs"].toArray();
					for (auto doc_RGB : j_segmentRGBs) {
						auto j_RGB = doc_RGB.toObject();
						
						std::array<int, 3> rgb;
						rgb[0] = jsonHelper::getInteger(j_RGB, QStringLiteral("R"), 0);
						rgb[1] = jsonHelper::getInteger(j_RGB, QStringLiteral("G"), 0);
						rgb[2] = jsonHelper::getInteger(j_RGB, QStringLiteral("B"), 0);

						optic.segmentRGBs.push_back(rgb);
					}
				}

				if (obj.contains("R")) loadOpticBand(obj["R"], optic.R);
				if (obj.contains("G")) loadOpticBand(obj["G"], optic.G);
				if (obj.contains("B")) loadOpticBand(obj["B"], optic.B);
				if (obj.contains("M")) loadOpticBand(obj["M"], optic.M);

				auto item = new QListWidgetItem();
				
				item->setText(optic.name);
				item->setWhatsThis(optic.id);
				if (optic.tag == "Main") {
					_mainOptics[_camID] = optic;
					item->setIcon(QIcon(":/8Icon/Icon/icon8/icons8-m-100.png"));
					item->setSelected(true);
					hasMainOptic = true;
					getMainOpticsID = optic.id;
				}

				auto item2 = new QListWidgetItem();
				item2->setText(optic.name);
				item2->setWhatsThis(optic.id);
				item2->setCheckState(Qt::Checked);

				ui.listWidget_recipeOptics->addItem(item);
				ui.listWidget_viewOpticSelection->addItem(item2);
				_recipeOptics.insert(optic.id, optic);
			}

			if (!hasMainOptic) {
				addDefaultOptic();
			}
		}

		if (root.contains("CSA")) {
			auto j_CSA = root["CSA"].toObject();
			_CSA.opticID = jsonHelper::getString(j_CSA, QStringLiteral("opticID"));
			_CSA.viewRef = jsonHelper::getString(j_CSA, QStringLiteral("viewRef"));

			if (j_CSA.contains("learnLocator")) {
				auto j_learnLocator = j_CSA["learnLocator"].toObject();
				deleteDragBox(nullptr, _CSA.learnLocator);

				_CSA.learnLocator = addDragBoxToScene(_pGraphicsSceneFOV, getQRectBasedOnCam(30), Qt::blue, "Learn Region", "learnLocator");
				_CSA.learnLocator->hide();
				fromJson(_CSA.learnLocator, j_learnLocator);
			}

			if (j_CSA.contains("searchLocators")) {
				auto j_searchLocator = j_CSA["searchLocators"].toArray();

				for (int i = 0; i < _CSA.searchLocator.size(); i++) {
					deleteDragBox(nullptr, _CSA.searchLocator[i]);
				}

				_CSA.searchLocator.clear();
				_CSA.teachPoints.clear();

				for (auto j_locator : j_searchLocator) {
					auto p = addDragBoxToScene(_pGraphicsSceneFOV, getQRectBasedOnCam(70), Qt::green, "Search Region", "");
					auto obj = j_locator.toObject();
					fromJson(p, obj);
					_CSA.searchLocator.append(p);
					_CSA.searchLocator.back()->hide();

					auto cx = jsonHelper::getDouble(obj, "teachPointX");
					auto cy = jsonHelper::getDouble(obj, "teachPointY");

					_CSA.teachPoints.insert(p->getId(), QPointF(cx, cy));
				}
			}
		}
		else {
			deleteDragBox(nullptr, _CSA.learnLocator); 
			_CSA.learnLocator = addDragBoxToScene(_pGraphicsSceneFOV, getQRectBasedOnCam(30), Qt::blue, "Learn Region", "learnLocator");
			_CSA.learnLocator->hide();
		
			for (int i = 0; i < _CSA.searchLocator.size(); i++) {
				deleteDragBox(nullptr, _CSA.searchLocator[i]);
			}

			_CSA.searchLocator.clear();
			
			uidGenerator uidGen;
			_CSA.searchLocator.append(addDragBoxToScene(_pGraphicsSceneFOV, getQRectBasedOnCam(70), Qt::green, "Search Region", uidGen.id().c_str()));
			_CSA.searchLocator.back()->hide();
		}

		if (root.contains("bandBrightness")) {
			auto j_bandBrightness = root["bandBrightness"].toArray();

			_recipeBandBrightness.clear();

			for (auto j_optic : j_bandBrightness)
			{
				auto obj = j_optic.toObject();

				OpticsInfo optic;
				optic.id = jsonHelper::getString(obj, QStringLiteral("id"));
				optic.exposure = jsonHelper::getDouble(obj, QStringLiteral("exposure"), 20000);
				optic.gain = jsonHelper::getDouble(obj, QStringLiteral("gain"), 2);

				_recipeBandBrightness.insert(optic.id, optic);
			}
		}

		//load 3d
		if (root.contains("optics3D")) {
			auto j_optics = root["optics3D"].toArray();

			_recipeOptics3D.clear();

			for (auto j_optic : j_optics)
			{
				auto obj = j_optic.toObject();

				OpticsInfo3D optic;
				optic.id = jsonHelper::getString(obj, QStringLiteral("id"));
				optic.name = jsonHelper::getString(obj, QStringLiteral("name"));
				optic.tag = jsonHelper::getString(obj, QStringLiteral("tag"));
				optic.intensity = jsonHelper::getBool(obj, QStringLiteral("intensity"));
				optic.exposureMode = jsonHelper::getString(obj, QStringLiteral("exposureMode"), ct::s_single);
				optic.exposure = jsonHelper::getDouble(obj, QStringLiteral("exposure"));
				optic.exposure2 = jsonHelper::getDouble(obj, QStringLiteral("exposure2"));
				optic.gain = jsonHelper::getDouble(obj, QStringLiteral("gain"));
				optic.gain2 = jsonHelper::getDouble(obj, QStringLiteral("gain2"));
				optic.lineThreshold = jsonHelper::getDouble(obj, QStringLiteral("lineThreshold"));
				optic.divider = jsonHelper::getInteger(obj, QStringLiteral("divider"));
				optic.lowerLaserLimit = jsonHelper::getInteger(obj, QStringLiteral("lowerLaserLimit"), 90);
				optic.upperLaserLimit = jsonHelper::getInteger(obj, QStringLiteral("upperLaserLimit"), 90);
				optic.lightSensitivity = jsonHelper::getString(obj, QStringLiteral("lightSensitivity"), "High Precision");
				optic.peakSensitivity = jsonHelper::getString(obj, QStringLiteral("peakSensitivity"), "5");
				optic.peakSelection = jsonHelper::getString(obj, QStringLiteral("peakSelection"), "Standard");



				auto item = new QListWidgetItem();

				item->setText(optic.name);
				item->setWhatsThis(optic.id);
				if (optic.tag == "Main") {
					_mainOptics3D = optic;
					item->setIcon(QIcon(":/8Icon/Icon/icon8/icons8-m-100.png"));
					item->setSelected(true);
				}

				//ui.listWidget_recipeOptics->addItem(item);
				_recipeOptics3D.insert(optic.id, optic);
			}
		}
		else {
			_recipeOptics3D.clear();

			//give default if none found
			OpticsInfo3D opt30;
			opt30.id = "E30";
			opt30.name = "E30";
			opt30.exposure = 30.0;
			opt30.intensity = true;

			OpticsInfo3D opt375;
			opt375.id = "E375";
			opt375.name = "E375";
			opt375.exposure = 375;
			opt375.intensity = false;

			_recipeOptics3D.insert(opt30.id, opt30);
			_recipeOptics3D.insert(opt375.id, opt375);

			saveRecipeOptics();
		}
	}
	else {
		_recipeOptics.clear();
		ui.listWidget_recipeOptics->clear();
		ui.listWidget_viewOpticSelection->clear();
		addDefaultOptic();
		ct::logger::warn("Failed to open: optics.json\n");
		OpticsControl::instance().toggleAllChannels(false);
		return false;
	}

	updateOpticComboBoxUI();

	OpticsControl::instance().toggleAllChannels(false);

	ct::logger::info("Loaded recipe optics");
	return true;
}

bool VisionApp::saveOpticBand(QJsonObject& obj, const QHash<QString, int>& opt)
{
	for (const auto& key : opt.keys()) {
		obj.insert(key, opt[key]);
	}

	return true;
}

bool VisionApp::loadOpticBand(const QJsonValue& doc, QHash<QString, int>& opt)
{
	//old format
	if (doc.isArray()) {
		int index = 0;
		int validIndex = LSCManager::instance().channels().size();

		for (const auto& d : doc.toArray()) {
			if (index >= validIndex) break;

			auto itr = LSCManager::instance().channels().begin() + index;
			auto key = *itr;
			auto intensity = d.toInt();

			//if (intensity == 0) continue;

			opt.insert(key, intensity);
			index++;
		}
	}
	else if (doc.isObject()) {
		auto obj = doc.toObject();
		
		// Iterate through the QJsonObject
		for (auto it = obj.begin(); it != obj.end(); ++it) {
			QString key = it.key();
			QJsonValue value = it.value();

			opt.insert(key, jsonHelper::getInteger(obj, key, 0));
		}
	}
	else {
		return false;
	}
	
	return true;
}

bool VisionApp::savePortabilityInfo()
{
	auto jsonPath = QStringLiteral("%1portability.json").arg(Common::Directory::ConfigPath());


	auto& lci = _portabilityInfo.lightingCalibrationInfo;

	QJsonArray j_brightness;
	for (const auto& key : lci.brightness.keys()) {
		const auto& b = lci.brightness[key];

		QJsonObject bobj;
		bobj.insert(QStringLiteral("id"), key);
		bobj.insert(QStringLiteral("exposure"), b.exposure);
		bobj.insert(QStringLiteral("gain"), b.gain);
		bobj.insert(QStringLiteral("average_gap"), b.average_gap);
		j_brightness.append(bobj);
	}


	QJsonArray j_MGVTable;
	QJsonArray j_LGVTable;

	for (const auto& key : lci.main_GVTable.keys()) {

		const auto& gvt = lci.main_GVTable[key];

		QJsonObject tableObj;
		QJsonArray j_IList;

		for (const auto& gv : gvt) {
			j_IList.append(gv);
		}

		tableObj.insert("id", key);
		tableObj.insert("profile", j_IList);

		j_MGVTable.append(tableObj);
	}

	for (const auto& key : lci.local_GVTable.keys()) {

		const auto& gvt = lci.local_GVTable[key];

		QJsonObject tableObj;
		QJsonArray j_IList;

		for (const auto& gv : gvt) {
			j_IList.append(gv);
		}

		tableObj.insert("id", key);
		tableObj.insert("profile", j_IList);

		j_LGVTable.append(tableObj);
	}

	QJsonObject obj;
	obj.insert(QStringLiteral("main"), lci.is_main);
	obj.insert(QStringLiteral("main_gvtable"), j_MGVTable);
	obj.insert(QStringLiteral("local_gvtable"), j_LGVTable);
	obj.insert(QStringLiteral("brightness"), j_brightness);

	auto ret = saveJson(jsonPath, QJsonDocument(obj));

	if (ret) showStatus(QStringLiteral("Successfully saved portability!"));
	else showStatus(QStringLiteral("Failed to save portability!"));

	return ret;
}

bool VisionApp::loadPortabilityInfo()
{
	auto jsonPath = QStringLiteral("%1portability.json").arg(Common::Directory::ConfigPath());

	QJsonObject root;

	//reset info
	auto& lci = _portabilityInfo.lightingCalibrationInfo;
	lci.main_GVTable.clear();
	lci.local_GVTable.clear();
	lci.graycard_point = dat::WorldCoordinate();
	lci.brightness.clear();

	if (!loadJson(jsonPath, root)) return false;

	lci.is_main = jsonHelper::getBool(root, "main", true);

	if (root.contains("graycard_point")) {
		fromJson(root["graycard_point"].toObject(), lci.graycard_point, true);
	}

	if (root.contains("main_gvtable")) {
		if (root["main_gvtable"].isArray()) {

			auto j_GVTable = root["main_gvtable"].toArray();

			for (auto tableDoc : j_GVTable) {
				auto tableObj = tableDoc.toObject();

				auto id = jsonHelper::getString(tableObj, "id", "");
				auto profile = jsonHelper::getArray(tableObj, "profile");

				auto& gvt = lci.main_GVTable[id];
				std::fill(gvt.begin(), gvt.end(), 0);
				const int profileSize = qMin(profile.size(), static_cast<int>(gvt.size()));
				for (int i = 0; i < profileSize; i++)
				{
					gvt[i] = profile[i].toDouble();
				}
			}
		}
	}

	if (root.contains("local_gvtable")) {
		if (root["local_gvtable"].isArray()) {

			auto j_GVTable = root["local_gvtable"].toArray();

			for (auto tableDoc : j_GVTable) {
				auto tableObj = tableDoc.toObject();

				auto id = jsonHelper::getString(tableObj, "id", "");
				auto profile = jsonHelper::getArray(tableObj, "profile");

				auto& gvt = lci.local_GVTable[id];
				std::fill(gvt.begin(), gvt.end(), 0);
				const int profileSize = qMin(profile.size(), static_cast<int>(gvt.size()));
				for (int i = 0; i < profileSize; i++)
				{
					gvt[i] = profile[i].toDouble();
				}
			}
		}
	}

	if (root.contains("brightness")) {
		auto j_brightness = root["brightness"].toArray();

		BrightnessInfo b;
		for (const auto& bdoc : j_brightness) {
			auto bobj = bdoc.toObject();
			
			auto id = jsonHelper::getString(bobj, "id", "");
			b.exposure = jsonHelper::getInteger(bobj, "exposure", 20000);
			b.gain = jsonHelper::getInteger(bobj, "gain", 2);
			b.average_gap = jsonHelper::getDouble(bobj, "average_gap", -1);

			lci.brightness.insert(id, b);
		}
	}

	return true;
}

bool VisionApp::saveMotion()
{
	QJsonArray j_motion;

	for (const auto& m : _motions) {
		QJsonObject j_ctrl;
		toJson(m, j_ctrl);
		j_motion.append(j_ctrl);
	}

	QJsonObject root;
	root["Motion"] = j_motion;

	auto motionPath = QStringLiteral("%1/motion.json").arg(Common::Directory::ConfigPath());

	return saveJson(
		motionPath,
		QJsonDocument(root)
	);
}

bool VisionApp::loadMotion()
{
	QJsonObject root;
	auto motionPath = QStringLiteral("%1/motion.json").arg(Common::Directory::ConfigPath());

	if (!loadJson(motionPath, root))
		return false;

	_motions.clear();

	if (!root.contains("Motion"))
		return false;

	for (const auto& v : root["Motion"].toArray()) {
		nvs::motion::MotionConfig m;
		fromJson(v.toObject(), m);
		_motions.insert(m.id.c_str(), m);
	}

	return true;
}

bool VisionApp::saveRecipeMotion()
{
	return false;
}

bool VisionApp::loadRecipeMotion()
{
	return false;
}

bool VisionApp::saveRecipeConfig()
{
	auto jsonPath = QStringLiteral("%1recipe/%2/recipeConfig.json").arg(Common::Directory::LocalPath).arg(Common::Directory::CurrentRecipe);

	QJsonObject obj;
	obj.insert(QStringLiteral("camera_angle"), SystemData::instance()._camAngles[_camID]);
	obj.insert(QStringLiteral("_enable3D"), _enable3D);
	obj.insert(QStringLiteral("_disable2D"), !_enable2D);
	obj.insert(QStringLiteral("_enablePreProcessImg"), _enablePreProcessImg);
	obj.insert(QStringLiteral("_enableVisionObjectSampling"), _enableVisionObjectSampling);
	obj.insert(QStringLiteral("_enableSingleViewRecipe"), _enableSingleViewRecipe);
	obj.insert(QStringLiteral("_passYieldPercentage"), _passYieldPerc);
	obj.insert(QStringLiteral("worldScale"), ScaleManager::instance().world_scale());
	obj.insert(QStringLiteral("laserApi"), laserApi);
	obj.insert(QStringLiteral("_warpageMethod"), _warpageMethod);
	obj.insert(QStringLiteral("stitchingMethod"), SystemData::instance()._stitchingMethod);
	obj.insert(QStringLiteral("lineScanAxis"), (int)SystemData::instance()._lineScanAxis);
	obj.insert(QStringLiteral("lscStrobeMode"), (bool)SystemData::instance()._lscStrobeMode);
	obj.insert(QStringLiteral("camImageRotation"), (int)SystemData::instance()._camImageRotation);
	obj.insert(QStringLiteral("_doubleFiducialChecking"), (bool)SystemData::instance()._doubleFiducialChecking);

	int speed, speed3d;
	_jobThread.getXSpeed(speed, speed3d);
	obj.insert(QStringLiteral("x_2d_velocity"), speed);
	obj.insert(QStringLiteral("x_3d_velocity"), speed3d);
	obj.insert(QStringLiteral("x_acceleration"), ui.lineEdit_x_acceleration->text().toInt());
	// RECIPE_Z_CONVEYOR_DISABLED_BEGIN
	// Recipe-based conveyor width persistence is disabled. Re-enable this insert
	// to save conveyorWidth into recipeConfig.json again.
	//obj.insert(QStringLiteral("conveyorWidth"), ui.lineEdit_railWidth1->text().toDouble());
	// RECIPE_Z_CONVEYOR_DISABLED_END
	
	

	auto ret = saveJson(jsonPath, QJsonDocument(obj));

	if (ret) showStatus(QStringLiteral("Successfully saved recipe config!"));
	else showStatus(QStringLiteral("Failed to save recipe config!"));

	return ret;
}

bool VisionApp::saveLaserConfig()
{
	auto jsonPath = QStringLiteral("%1/laserConfig.json").arg(Common::Directory::ConfigPath());
	
	QJsonObject obj, camObj, laserObj, offsetObj;

	toJson(_laserConfig.camera_center, camObj, true);
	toJson(_laserConfig.laser_center, laserObj, true);
	toJson(_laserConfig.offset, offsetObj, true);
	
	const bool& msr = ProfilerManager::instance().getMSR();
	if (msr == true) {
		obj.insert(QStringLiteral("camera_center"), camObj);
		obj.insert(QStringLiteral("laser_center"), laserObj);
		obj.insert(QStringLiteral("offsetMSR"), offsetObj);
	}
	else
	{
		obj.insert(QStringLiteral("camera_center"), camObj);
		obj.insert(QStringLiteral("laser_center"), laserObj);
		obj.insert(QStringLiteral("offset"), offsetObj);
	}

	
	auto ret = saveJson(jsonPath, QJsonDocument(obj));

	//if (ret) showStatus(QStringLiteral("Successfully saved laser config!"));
	//else showStatus(QStringLiteral("Failed to save laser config!"));

	return ret;
}

bool VisionApp::loadVisionObject()
{
	qDebug() << "loading vision object...";
	QDir dir;
	QString val;
	QFile file;
	QJsonObject root;
	QJsonDocument doc;
	QString imagePath;
	QString recipeName;

	//load VisionObject
	auto jsonPath = QStringLiteral("%1recipe/%2/visionObject.json").arg(Common::Directory::LocalPath).arg(Common::Directory::CurrentRecipe);
	ct::logger::info("Load vision object: %s", jsonPath.toStdString().c_str());

	if (QFile::exists(jsonPath))
	{
		file.setFileName(jsonPath);
		if (file.open(QIODevice::ReadOnly | QIODevice::Text))
		{
			QVisionObject visionObject;

			clearVisionObject();
			clearView();
			clearPath();
			clearAllDrawings();

			imagePath = QStringLiteral("%1recipe/%2/image/%3").arg(Common::Directory::LocalPath).arg(recipeName).arg(jsonHelper::getString(_systemObj, QStringLiteral("Recent_Open_Image")));
			if (QFile::exists(imagePath))
			{
				loadImage(imagePath);
			}


			//load VisionObject - start
			val = file.readAll();
			file.close();

			doc = QJsonDocument::fromJson(val.toUtf8());
			root = doc.object();

			QJsonObject recipeObj = root[QStringLiteral("Recipe")].toObject();
			QJsonArray objectArray = recipeObj[QStringLiteral("Object")].toArray();
		
			for (int i = 0; i < objectArray.count(); i++)
			{
				QJsonObject object = objectArray[i].toObject();

				visionObject.objectName = jsonHelper::getString(object, QStringLiteral("objectName"));
				visionObject.objectID = jsonHelper::getString(object, QStringLiteral("objectID"));
				if (visionObject.objectID.isEmpty())
				{
					uidGenerator uidGen;
					visionObject.objectID = "object" + QString(uidGen.id().c_str());
				}

				visionObject.templateName = jsonHelper::getString(object, QStringLiteral("templateName"));
				visionObject.templateID = jsonHelper::getString(object, QStringLiteral("templateID"));

				visionObject.viewID = jsonHelper::getString(object, QStringLiteral("View"));
				visionObject.lineScanID = jsonHelper::getString(object, QStringLiteral("lineScanID"));

				visionObject.skip = jsonHelper::getBool(object, QStringLiteral("skip"));
				visionObject.forcedSkip = jsonHelper::getBool(object, QStringLiteral("ForcedSkip"));
				visionObject.locked = jsonHelper::getBool(object, QStringLiteral("Locked"));
				visionObject.angle = jsonHelper::getDouble(object, QStringLiteral("Angle"));

				visionObject.row = jsonHelper::getInteger(object, QStringLiteral("Row"));
				visionObject.col = jsonHelper::getInteger(object, QStringLiteral("Col"));
				visionObject.row_id = jsonHelper::getInteger(object, QStringLiteral("Row_ID"));
				visionObject.col_id = jsonHelper::getInteger(object, QStringLiteral("Col_ID"));
				visionObject.island = jsonHelper::getString(object, QStringLiteral("Island"), "1");
				visionObject.island_id = jsonHelper::getInteger(object, QStringLiteral("Island_ID"));

				QRectF roi;
				roi.setLeft(jsonHelper::getDouble(object, QStringLiteral("Roi_Left")));
				roi.setTop(jsonHelper::getDouble(object, QStringLiteral("Roi_Top")));

				if (g_viewMode == int(ViewMode::PLANE))
				{
					auto absoluteCoordinates = getAbsoluteFOVCoordinates(QPointF(roi.left(), roi.top()));
					roi.setLeft(absoluteCoordinates.x());
					roi.setTop(absoluteCoordinates.y());
				}
				

				roi.setWidth(jsonHelper::getDouble(object, QStringLiteral("Roi_Width")));
				roi.setHeight(jsonHelper::getDouble(object, QStringLiteral("Roi_Height")));
				visionObject.rect = roi;

				roi = ScaleManager::instance().fov_to_world(roi);

				QColor color = _templateLibraryTab->getTemplateColor(visionObject.templateID);
				auto visionAppDragBox = drawVisionAppDragBox(roi, color, visionObject.objectName, visionObject.viewID);
				visionAppDragBox->algoTemplate(_templateLibraryTab->getAlgoTemplate(visionObject.templateID));
				visionAppDragBox->setID(visionObject.objectID);
				visionAppDragBox->viewID(visionObject.viewID);
				visionAppDragBox->lineScanID(visionObject.lineScanID);
				visionObject.pDragBox = visionAppDragBox;
				visionObject.pDragBox->type((int)DragBoxType::VISIONOBJECT);
				visionObject.pDragBox->setZValue((int)UIHierarchy::DRAGGABLES);
				if (visionObject.locked) {
					visionObject.pDragBox->setDragable(false);
				}

				_visionObject.insert(visionObject.objectID, visionObject);
			}

			ct::logger::info("Successfully loaded vision object\n");
		}
		else
		{
			ct::logger::warn("Failed to open: visionObject.json");
			return false;
		}
	}
	else
	{
		ct::logger::warn("Failed to open: visionObject.json");
		return false;
	}

	return true;
}

bool VisionApp::loadIslandInfo()
{
	QFile file;
	QString val;
	QJsonDocument doc;
	QJsonObject obj;


	auto islandInfoPath = QStringLiteral("%1recipe/%2/islandInfo.json").arg(Common::Directory::LocalPath).arg(Common::Directory::CurrentRecipe);
	if (QFile::exists(islandInfoPath))
	{
		
		file.setFileName(islandInfoPath);
		if (file.open(QIODevice::ReadOnly | QIODevice::Text))
		{
			
			val = file.readAll();
			
			file.close();
			doc = QJsonDocument::fromJson(val.toUtf8());
			obj = doc.object();
		
			_islandInfo.colPitch = jsonHelper::getString(obj, QStringLiteral("colPitch")).toDouble();
			_islandInfo.postfix = jsonHelper::getString(obj, QStringLiteral("postfix"));
			_islandInfo.prefix = jsonHelper::getString(obj, QStringLiteral("prefix"));
			_islandInfo.rotation = jsonHelper::getString(obj, QStringLiteral("rotation")).toDouble();
			_islandInfo.rowPitch = jsonHelper::getString(obj, QStringLiteral("rowPitch")).toDouble();
			_islandInfo.rowStartingIndex = jsonHelper::getString(obj, QStringLiteral("rowStartingIndex")).toInt();
			_islandInfo.colStartingIndex = jsonHelper::getString(obj, QStringLiteral("colStartingIndex")).toInt();
			_islandInfo.totalCol = jsonHelper::getString(obj, QStringLiteral("totalCol")).toInt();
			_islandInfo.totalIsland = jsonHelper::getString(obj, QStringLiteral("totalIsland")).toInt();
			_islandInfo.totalRow = jsonHelper::getString(obj, QStringLiteral("totalRow")).toInt();
		}

		ui.lineEdit_namingPrefix->setText(_islandInfo.prefix);
		ui.lineEdit_namingPostfix->setText(_islandInfo.postfix);
		ui.lineEdit_rowStartingIndex->setText(QString::number(_islandInfo.rowStartingIndex));
		ui.lineEdit_colStartingIndex->setText(QString::number(_islandInfo.colStartingIndex));
		ui.lineEdit_row->setText(QString::number(_islandInfo.totalRow));
		ui.lineEdit_rowPitch->setText(QString::number(_islandInfo.rowPitch));
		ui.lineEdit_column->setText(QString::number(_islandInfo.totalCol));
		ui.lineEdit_columnPitch->setText(QString::number(_islandInfo.colPitch));
		ui.lineEdit_rotation->setText(QString::number(_islandInfo.rotation)); 
		ui.lineEdit_totalIsland->setText(QString::number(_islandInfo.totalIsland));
	}

	return true;
}

bool VisionApp::loadView()
{
	QDir dir;
	QString val;
	QFile file;
	QJsonObject root;
	QJsonDocument doc;
	QString jsonPath;
	
	auto viewPath = QStringLiteral("%1recipe/%2/view.json").arg(Common::Directory::LocalPath).arg(Common::Directory::CurrentRecipe);
	ct::logger::info("Load view: %s", viewPath.toStdString().c_str());

	if (QFile::exists(viewPath))
	{
		file.setFileName(viewPath);
		if (file.open(QIODevice::ReadOnly | QIODevice::Text))
		{

			_views.clear();

			val = file.readAll();
			file.close();

			doc = QJsonDocument::fromJson(val.toUtf8());
			root = doc.object();

			QJsonArray j_array = root[QStringLiteral("views")].toArray();

			for (int i = 0; i < j_array.count(); i++)
			{

				QJsonObject obj = j_array[i].toObject();
				QView v;
				v.id = jsonHelper::getString(obj, QStringLiteral("id"));
				if (v.id.isEmpty()) continue;

				v.name = jsonHelper::getString(obj, QStringLiteral("name"));
				v.camID = jsonHelper::getString(obj, QStringLiteral("camID"), "cam1");
				v.type = jsonHelper::getString(obj, QStringLiteral("type"));
				v.created_by = jsonHelper::getString(obj, QStringLiteral("created_by"));
				v.map_to_sview = jsonHelper::getString(obj, QStringLiteral("map_to_sview"));
				v.horizontal_scale = jsonHelper::getDouble(obj, QStringLiteral("horizontal_scale"));
				v.vertical_scale = jsonHelper::getDouble(obj, QStringLiteral("vertical_scale"));

				if (obj.contains("px_coordinate")) {
					auto pxObj = obj["px_coordinate"].toObject();
					v.px.cx = jsonHelper::getDouble(pxObj, QStringLiteral("cx"));
					v.px.cy = jsonHelper::getDouble(pxObj, QStringLiteral("cy"));
					v.px.w = jsonHelper::getDouble(pxObj, QStringLiteral("w"));
					v.px.h = jsonHelper::getDouble(pxObj, QStringLiteral("h"));
					v.px.xmin = jsonHelper::getDouble(pxObj, QStringLiteral("xmin"));
					v.px.ymin = jsonHelper::getDouble(pxObj, QStringLiteral("ymin"));
					v.px.xmax = jsonHelper::getDouble(pxObj, QStringLiteral("xmax"));
					v.px.ymax = jsonHelper::getDouble(pxObj, QStringLiteral("ymax"));

					if (g_viewMode == int(ViewMode::PLANE))
					{
						auto vpxc = getAbsoluteFOVCoordinates(QPointF(v.px.cx, v.px.cy));
						v.px.cx = vpxc.x();
						v.px.cy = vpxc.y();
						auto vpxmin = getAbsoluteFOVCoordinates(QPointF(v.px.xmin, v.px.ymin));
						v.px.xmin = vpxmin.x();
						v.px.ymin = vpxmin.y();
						auto vpxmax = getAbsoluteFOVCoordinates(QPointF(v.px.xmax, v.px.ymax));
						v.px.xmax = vpxmax.x();
						v.px.ymax = vpxmax.y();
					}
					
				}

				if (obj.contains("world_coordinate")) {
					auto worldObj = obj["world_coordinate"].toObject();
					fromJson(worldObj, v.world);
					/*v.world.wx = jsonHelper::getDouble(worldObj, QStringLiteral("wx"));
					v.world.wy = jsonHelper::getDouble(worldObj, QStringLiteral("wy"));
					v.world.wz = jsonHelper::getDouble(worldObj, QStringLiteral("wz"));*/
					v.world.rx = jsonHelper::getDouble(worldObj, QStringLiteral("rx"));
					v.world.ry = jsonHelper::getDouble(worldObj, QStringLiteral("ry"));
					v.world.rz = jsonHelper::getDouble(worldObj, QStringLiteral("rz"));
				}

				if (obj.contains("vision_obj_ids")) {
					auto objVisionArray = obj["vision_obj_ids"].toArray();
					for (auto obj : objVisionArray)
					{
						v.vision_obj_IDs.append(obj.toString());
					}
				}

				if (obj.contains("preprocess")) {
					auto preObj = obj["preprocess"].toObject();
					v.preprocess.crop = jsonHelper::getBool(preObj, QStringLiteral("crop"));
					v.preprocess.resize = jsonHelper::getBool(preObj, QStringLiteral("resize"));
					v.preprocess.flatfield = jsonHelper::getBool(preObj, QStringLiteral("flatfield"));
					v.preprocess.cropRect.setX(jsonHelper::getInteger(preObj, QStringLiteral("crop_x")));
					v.preprocess.cropRect.setY(jsonHelper::getInteger(preObj, QStringLiteral("crop_y")));
					v.preprocess.cropRect.setWidth(jsonHelper::getInteger(preObj, QStringLiteral("crop_w")));
					v.preprocess.cropRect.setHeight(jsonHelper::getInteger(preObj, QStringLiteral("crop_h")));
					v.preprocess.resizeRect.setWidth(jsonHelper::getInteger(preObj, QStringLiteral("resize_w")));
					v.preprocess.resizeRect.setHeight(jsonHelper::getInteger(preObj, QStringLiteral("resize_h")));
				}

				if (obj.contains("zstack")) {
					auto zstackObj = obj["zstack"].toObject();
					v.zstack.acq_type = jsonHelper::getString(zstackObj, QStringLiteral("acq_type"));
					v.zstack.step_um = jsonHelper::getInteger(zstackObj, QStringLiteral("step_um"));
					v.zstack.preset_iteration = jsonHelper::getInteger(zstackObj, QStringLiteral("preset_iteration"));
					v.zstack.encoder_range_um = jsonHelper::getInteger(zstackObj, QStringLiteral("encoder_range_um"));
					v.zstack.time_interval_ms = jsonHelper::getInteger(zstackObj, QStringLiteral("time_interval_ms"));
					v.zstack.generate_2D_stack = jsonHelper::getBool(zstackObj, QStringLiteral("generate_2D_stack"));
					v.zstack.generate_3D_stack = jsonHelper::getBool(zstackObj, QStringLiteral("generate_3D_stack"));
				}

				if (obj.contains("opticIDs")) {
					auto objOpticIDsArray = obj["opticIDs"].toArray();
					for (auto obj : objOpticIDsArray)
					{
						for (auto opt : _recipeOptics) {
							if (opt.id == obj.toString())
							{
								v.opticIDs.insert(obj.toString());
							}
						}
						
					}
				}
				else {
					for (auto opt : _recipeOptics) {
						v.opticIDs.insert(opt.id);
					}
				}

				//convert to world

				auto wpx = ScaleManager::instance().to_world_px(QPointF(v.world.wx, v.world.wy));
				ct::Box2D worldView;
				worldView.cx = wpx.x();
				worldView.cy = wpx.y();
				worldView.w = ScaleManager::instance().fov_to_world(v.px.w);
				worldView.h = ScaleManager::instance().fov_to_world(v.px.h);
				worldView.xmin = wpx.x() - worldView.w / 2;
				worldView.ymin = wpx.y() - worldView.h / 2;
				worldView.xmax = wpx.x() + worldView.w / 2;
				worldView.ymax = wpx.y() + worldView.h / 2;

				v.pDragBox = drawViewBox(QRectF(worldView.xmin, worldView.ymin, worldView.w, worldView.h), getColor(Representation::ASSIGNED_VIEW), v.name, v.id);
				_views.insert(v.id, v);
			}

			updateViewComboBoxUI();
			ct::logger::info("Successfully loaded view\n");
		}
		else
		{
			ct::logger::warn("Failed to open: view.json");
			return false;
		}
	}
	else
	{
		ct::logger::warn("Failed to open: view.json");
		return false;
	}
	
	return true;
}

bool VisionApp::loadLineScans()
{
	QDir dir;
	QString val;
	QFile file;
	QJsonObject root;
	QJsonDocument doc;

	auto jsonPath = QStringLiteral("%1recipe/%2/linescans.json").arg(Common::Directory::LocalPath).arg(Common::Directory::CurrentRecipe);
	ct::logger::info("Load linescan: %s", jsonPath.toStdString().c_str());

	if (QFile::exists(jsonPath))
	{
		file.setFileName(jsonPath);
		if (file.open(QIODevice::ReadOnly | QIODevice::Text))
		{

			_lineScans.clear();

			val = file.readAll();
			file.close();

			doc = QJsonDocument::fromJson(val.toUtf8());
			root = doc.object();

#if 0
			// RECIPE_Z_CONVEYOR_DISABLED_BEGIN
			// Recipe-based 3D Z offset loading is disabled. Re-enable this block
			// to load z_offset from linescans.json into the setup UI again.
			if (root.contains("z_offset")) {
				m_currentZOffset = root.value("z_offset").toDouble();
				ui.lineEdit_3DZoffset->setText(QString::number(m_currentZOffset));
			}
			else {
				m_currentZOffset = 0.0;
				ui.lineEdit_3DZoffset->setText("0");
			}
			// RECIPE_Z_CONVEYOR_DISABLED_END
#endif

			QJsonArray j_array = root[QStringLiteral("line_scans")].toArray();

			for (int i = 0; i < j_array.count(); i++)
			{

				QJsonObject obj = j_array[i].toObject();
				QLineScan v;
				v.id = jsonHelper::getString(obj, QStringLiteral("id"));
				v.name = jsonHelper::getString(obj, QStringLiteral("name"));
				v.created_by = jsonHelper::getString(obj, QStringLiteral("created_by"));
				v.type = jsonHelper::getString(obj, QStringLiteral("type"));
				v.map_to_slinescan = jsonHelper::getString(obj, QStringLiteral("map_to_slinescan"));

				if (obj.contains("px_coordinate")) {
					auto pxObj = obj["px_coordinate"].toObject();
					fromJson(pxObj, v.px);
				}

				if (obj.contains("start_point")) {
					auto worldObj = obj["start_point"].toObject();
					fromJson(worldObj, v.start_point);
				}

				if (obj.contains("end_point")) {
					auto worldObj = obj["end_point"].toObject();
					fromJson(worldObj, v.end_point);
				}

				if (obj.contains("vision_obj_ids")) {
					auto objVisionArray = obj["vision_obj_ids"].toArray();
					for (auto obj : objVisionArray)
					{
						v.vision_obj_IDs.append(obj.toString());
					}
				}

				auto wpx = ScaleManager::instance().to_world_px(QPointF((v.start_point.wx + v.end_point.wx)/2, (v.start_point.wy + v.end_point.wy)/2));
				ct::Box2D worldView;
				worldView.cx = wpx.x();
				worldView.cy = wpx.y();
				worldView.w = ScaleManager::instance().fov_to_world(v.px.w);
				worldView.h = ScaleManager::instance().fov_to_world(v.px.h);
				worldView.xmin = wpx.x() - worldView.w / 2;
				worldView.ymin = wpx.y() - worldView.h / 2;
				worldView.xmax = wpx.x() + worldView.w / 2;
				worldView.ymax = wpx.y() + worldView.h / 2;

				v.pDragBox = drawLineScan(QRectF(worldView.xmin, worldView.ymin, worldView.w, worldView.h), getColor(Representation::ASSIGNED_VIEW), v.name, v.id);
				_lineScans.insert(v.id, v);
			}

			ct::logger::info("Successfully loaded line scans\n");
		}
		else
		{
			ct::logger::warn("Failed to open: linescans.json");
			return false;
		}
	}
	else
	{
		ct::logger::warn("Failed to open: linescans.json");
		return false;
	}

	return true;
}

bool VisionApp::loadRecipeConfig()
{
	auto jsonPath = QStringLiteral("%1recipe/%2/recipeConfig.json").arg(Common::Directory::LocalPath).arg(Common::Directory::CurrentRecipe);

	QJsonObject root;
	const bool& connected = ProfilerManager::instance().getConnectionStatus();
	QString currentSensorApi = ProfilerManager::instance().getAPI();
	if (loadJson(jsonPath, root)) {
		_enable3D = jsonHelper::getBool(root, QStringLiteral("_enable3D"), false);
		_enable2D = !jsonHelper::getBool(root, QStringLiteral("_disable2D"), false);
		_enablePreProcessImg = jsonHelper::getBool(root, QStringLiteral("_enablePreProcessImg"), true);
		_enableVisionObjectSampling = jsonHelper::getBool(root, QStringLiteral("_enableVisionObjectSampling"), false);
		_enableSingleViewRecipe = jsonHelper::getBool(root, QStringLiteral("_enableSingleViewRecipe"), false);
		_passYieldPerc = jsonHelper::getDouble(root, QStringLiteral("_passYieldPercentage"), 100.00);
		_warpageMethod = jsonHelper::getString(root, QStringLiteral("_warpageMethod"), "None");
		SystemData::instance()._doubleFiducialChecking = jsonHelper::getBool(root, QStringLiteral("_doubleFiducialChecking"), false);

		double defaultSpeed = 0;
		if (_motions.contains(_motionID)) defaultSpeed = _motions[_motionID].axes[(int)Axis::X].move_max_velocity;
		auto defaultSpeed3d = defaultSpeed / 10;

		auto speed2d = jsonHelper::getInteger(root, QStringLiteral("x_2d_velocity"), defaultSpeed);
		auto speed3d = jsonHelper::getInteger(root, QStringLiteral("x_3d_velocity"), defaultSpeed3d);
		auto accel = jsonHelper::getInteger(root, QStringLiteral("x_acceleration"), defaultSpeed);

		ui.lineEdit_x_velocity->setText(QString::number(speed2d));
		ui.lineEdit_x_velocity3d->setText(QString::number(speed3d));
		ui.lineEdit_x_acceleration->setText(QString::number(accel));

		_jobThread.setXSpeed(speed2d, speed3d);
		_jobThread.setXDecel(accel);

		MotionController::instance().set_move_acceleration(_motionID, (int)Axis::X, accel);
		MotionController::instance().set_move_deceleration(_motionID, (int)Axis::X, accel);

		// for laser API
		SystemData::instance()._stitchingMethod = jsonHelper::getInteger(root, QStringLiteral("stitchingMethod"), 2);
		ui.comboBox_stitchingMethod ->setCurrentIndex(SystemData::instance()._stitchingMethod-1);

		SystemData::instance()._lineScanAxis = jsonHelper::getInteger(root, QStringLiteral("lineScanAxis"), 0);
		{
			QSignalBlocker blocker(ui.comboBox_lineScanAxis);
			ui.comboBox_lineScanAxis->setCurrentIndex(SystemData::instance()._lineScanAxis);
		}

		SystemData::instance()._lscStrobeMode = jsonHelper::getBool(root, QStringLiteral("lscStrobeMode"), false);
		{
			QSignalBlocker blocker(ui.checkBox_lscStrobeMode);
			ui.checkBox_lscStrobeMode->setChecked(SystemData::instance()._lscStrobeMode);
		}
		//LSC connects before this config loads, so apply the configured mode here
		LSCManager::instance().setMode(SystemData::instance()._lscStrobeMode ? lsc::MODE::TRIGGER : lsc::MODE::CONTINUOUS);

		SystemData::instance()._camImageRotation = jsonHelper::getInteger(root, QStringLiteral("camImageRotation"), 0);
		{
			QSignalBlocker blocker(ui.comboBox_camRotation);
			ui.comboBox_camRotation->setCurrentIndex(SystemData::instance()._camImageRotation / 90);
		}

		{
		}
		//ScaleManager::instance().set_world_scale(jsonHelper::getDouble(root, QStringLiteral("worldScale"), 0.291716));
		if (!root.contains(QStringLiteral("laserApi")) || root.value(QStringLiteral("laserApi")).toString().isEmpty()) {
			laserApi = currentSensorApi;
			root.insert(QStringLiteral("laserApi"), laserApi);
			ct::logger::info("[InspectionThread] 'laserApi' not found in JSON, connected sensor value set.");
			saveRecipeConfig();
		}
		else if (connected && root.value(QStringLiteral("laserApi")).toString() != currentSensorApi)
		{

			QMessageBox questionBox;
			questionBox.setWindowFlags(questionBox.windowFlags() | Qt::WindowStaysOnTopHint);
			questionBox.setWindowTitle(QObject::tr("Sensor API Mismatch"));
			questionBox.setText(QObject::tr("The sensor API in recipeConfig.json does not match with the current connected sensor. Do you want to switch to the connected sensor?"));
			questionBox.setIcon(QMessageBox::Question);
			questionBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
			questionBox.setDefaultButton(QMessageBox::Yes); 
			QMessageBox::StandardButton response = static_cast<QMessageBox::StandardButton>(questionBox.exec());

			if (response == QMessageBox::Yes)
			{
				laserApi = currentSensorApi;
				ct::logger::info("[InspectionThread] 'laserApi' connected sensor value set.");
				saveRecipeConfig();

				// Create a warning message box to advise reassigning the linescan
				QMessageBox warnBox;
				warnBox.setWindowFlags(warnBox.windowFlags() | Qt::WindowStaysOnTopHint);
				warnBox.setWindowTitle(QObject::tr("Laser FoV Mistmatch"));
				warnBox.setText(QObject::tr("Warning: Please Reassign Line Scan for proper configuration."));
				warnBox.setIcon(QMessageBox::Warning);
				warnBox.setStandardButtons(QMessageBox::Ok);
				warnBox.exec();
			}
			else if (response == QMessageBox::No)
			{
				ct::logger::info("[InspectionThread] No changes to 'laserApi'.");

				QMessageBox warnBox;
				warnBox.setWindowFlags(warnBox.windowFlags() | Qt::WindowStaysOnTopHint);
				warnBox.setWindowTitle(QObject::tr("Laser FoV Mistmatch"));
				warnBox.setText(QObject::tr("Warning: Laser FoV not in sync for image acquisition."));
				warnBox.setIcon(QMessageBox::Warning);
				warnBox.setStandardButtons(QMessageBox::Ok);
				warnBox.exec();
				laserApi = jsonHelper::getString(root, QStringLiteral("laserApi"));
				saveRecipeConfig();
			}
		}
		else 
		{
			laserApi = jsonHelper::getString(root, QStringLiteral("laserApi"));
		}
		
		ui.checkBox_enable3D->setChecked(_enable3D);
		ui.checkBox_disable2D->setChecked(!_enable2D);
		ui.checkBox_enableVisionObjectSampling->setChecked(_enableVisionObjectSampling);
		ui.checkBox_doubleFiducialChecking->setChecked(SystemData::instance()._doubleFiducialChecking);
		ui.doubleSpinBox_passYieldPerc->setValue(_passYieldPerc);
	

		ui.comboBox_warpageMethod->setCurrentIndex(ui.comboBox_warpageMethod->findText(_warpageMethod));
		_jobThread.setWarpageMethod(ui.comboBox_warpageMethod->currentText());
		//ui.lineEdit_assemblyNumber->setText(_assemblyNumber); //TODO: WC

	}
	else {
		ct::logger::warn("Failed to open: recipeConfig.json");
		return false;
	}

	return true;
}

bool VisionApp::loadLSCConfig()
{
	auto jsonPath = QStringLiteral("%1/lsc.json").arg(Common::Directory::ConfigPath());

	QJsonObject root;

	if (loadJson(jsonPath, root)) {
		LSCManager::instance().loadConfig(root);
		int ret = LSCManager::instance().connect();
		LSCManager::instance().setMode(SystemData::instance()._lscStrobeMode ? lsc::MODE::TRIGGER : lsc::MODE::CONTINUOUS);

		nvs::set_background_color(ui.toolButton_lscStatus, ret == (int)LSC_RC::PASS ? Qt::green : Qt::red);
	}
	else {
		ct::logger::warn("Failed to open: lsc.json");
		return false;
	}

	return true;
}

bool VisionApp::loadLaserConfig()
{
	auto jsonPath = QStringLiteral("%1/laserConfig.json").arg(Common::Directory::ConfigPath());

	QJsonObject root;
	const bool& msr = ProfilerManager::instance().getMSR();

	if (loadJson(jsonPath, root)) {
		if(msr == true)
		{
			ct::logger::info("Loaded Dual Head laserConfig.json");
			if (root.contains("camera_center")) fromJson(root["camera_center"].toObject(), _laserConfig.camera_center);
			if (root.contains("laser_center")) fromJson(root["laser_center"].toObject(), _laserConfig.laser_center);
			if (root.contains("offsetMSR")) 
			{
				fromJson(root["offsetMSR"].toObject(), _laserConfig.offset, true);
				updateLaserOffsetUI(_laserConfig.offset);
			}
		}
		else
		{
			ct::logger::info("Loaded Single Head laserConfig.json");
			if (root.contains("camera_center")) fromJson(root["camera_center"].toObject(), _laserConfig.camera_center);
			if (root.contains("laser_center")) fromJson(root["laser_center"].toObject(), _laserConfig.laser_center);
			if (root.contains("offset")) 
			{
				fromJson(root["offset"].toObject(), _laserConfig.offset, true);
				updateLaserOffsetUI(_laserConfig.offset);
			}
		}

		double laser_fov = 21.76;
		QString sensorType = SystemData::instance().getLaserType();
		qDebug() << sensorType;
		if (sensorType == "Gocator") {
			ct::logger::info("[VisionAppJSON] Using Gocator FOV");
			laser_fov = 14.5;
		}
		else if (sensorType == "SmartRay") {
			ct::logger::info("[VisionAppJSON] Using SmartRay FOV");
			// Determine FOV based on the msr flag.
			if (msr) {
				ct::logger::info("[VisionAppJSON] MSR enabled, using MSR FOV");
				laser_fov = 11.277;
			}
			else {
				ct::logger::info("[VisionAppJSON] MSR disabled, using default FOV");
				laser_fov = 13.85;
			}
		}
		else if (sensorType == "SSZN") {
			ct::logger::info("[VisionAppJSON] Using SSZN FOV");
			laser_fov = 21.76;//21.76;
		}
		else if (sensorType == "KeyenceLJ") {
			ct::logger::info("[VisionAppJSON] Using KeyenceLJ FOV");
			//LJ-X8060: 3200 profile points at the default 5 um interval = 16.0 mm, which is also
			//the FAR-side X measurement range on the data sheet. Verify against Profiler_Keyence's
			//per-scan log "MEASURED LASER FOV = <mm> mm (<N> points @ <P> um)" - if the profile
			//data interval has been changed in Navigator this figure moves with it, and the
			//controller wins. Must be kept identical to the copy in VisionApp_CRUD.cpp.
			laser_fov = 16.0;
		}
		else {
			ct::logger::error("[VisionApp Json] Failed to get sensor type", sensorType.toStdString().c_str());
		}

		ScaleManager::instance().set_laser_fov_mm(laser_fov);
	}
	else {
		ct::logger::warn("Failed to open: laserConfig.json");
		return false;
	}
	/*update2DOnlyOverlay();*/
	return true;
}

void VisionApp::updateLaserOffsetUI(dat::WorldCoordinate offset)
{
	ui.lineEdit_laserOffset->setText(QString("%1, %2, %3").arg(offset.wx).arg(offset.wy).arg(offset.wz));
}

//converter
void VisionApp::toJson(const QView& view, QJsonObject& obj, bool isRelative)
{
	obj.insert(QStringLiteral("id"), view.id);
	obj.insert(QStringLiteral("name"), view.name);
	obj.insert(QStringLiteral("created_by"), view.created_by);
	obj.insert(QStringLiteral("map_to_sview"), view.map_to_sview);
	obj.insert(QStringLiteral("horizontal_scale"), view.horizontal_scale);
	obj.insert(QStringLiteral("vertical_scale"), view.vertical_scale);
	
	double cx = ScaleManager::instance().world_to_fov(view.px.cx);
	double cy = ScaleManager::instance().world_to_fov(view.px.cy);
	double w = ScaleManager::instance().world_to_fov(view.px.w);
	double h = ScaleManager::instance().world_to_fov(view.px.h);
	double xmin = ScaleManager::instance().world_to_fov(view.px.xmin);
	double ymin = ScaleManager::instance().world_to_fov(view.px.ymin);
	double xmax = ScaleManager::instance().world_to_fov(view.px.xmax);
	double ymax = ScaleManager::instance().world_to_fov(view.px.ymax);
	
	if (!isRelative)
	{
		auto vpxc = getRelativeFOVCoordinates(QPointF(view.px.cx, view.px.cy));
		cx = vpxc.x();
		cy = vpxc.y();
		auto vpxmin = getRelativeFOVCoordinates(QPointF(view.px.xmin, view.px.ymin));
		xmin = vpxmin.x();
		ymin = vpxmin.y();
		auto vpxmax = getRelativeFOVCoordinates(QPointF(view.px.xmax, view.px.ymax));
		xmax = vpxmax.x();
		ymax = vpxmax.y();
	}
	

	QJsonObject pxObj;
	pxObj.insert(QStringLiteral("cx"), cx);
	pxObj.insert(QStringLiteral("cy"), cy);
	pxObj.insert(QStringLiteral("w"), w);
	pxObj.insert(QStringLiteral("h"), h);
	pxObj.insert(QStringLiteral("xmin"), xmin);
	pxObj.insert(QStringLiteral("ymin"), ymin);
	pxObj.insert(QStringLiteral("xmax"), xmax);
	pxObj.insert(QStringLiteral("ymax"), ymax);
	obj.insert(QStringLiteral("px_coordinate"), pxObj);
	
	QJsonObject worldObj;
	toJson(view.world, worldObj, isRelative);
	//worldObj.insert(QStringLiteral("wx"), view.world.wx);
	//worldObj.insert(QStringLiteral("wy"), view.world.wy);
	//worldObj.insert(QStringLiteral("wz"), view.world.wz);
	worldObj.insert(QStringLiteral("rx"), view.world.rx);
	worldObj.insert(QStringLiteral("ry"), view.world.ry);
	worldObj.insert(QStringLiteral("rz"), view.world.rz);
	obj.insert(QStringLiteral("world_coordinate"), worldObj);
	
	QList<QVariant> objList;
	for (auto obj : view.vision_obj_IDs)
	{
		objList.append(obj);
	}
	
	obj.insert(QStringLiteral("vision_obj_ids"), QJsonArray::fromVariantList(objList));
}

void VisionApp::fromJson(const QJsonObject & obj, QView & view, bool isAbsolute)
{
	view.id = jsonHelper::getString(obj, QStringLiteral("id"));
	view.name = jsonHelper::getString(obj, QStringLiteral("name"));
	view.created_by = jsonHelper::getString(obj, QStringLiteral("created_by"));
	view.map_to_sview = jsonHelper::getString(obj, QStringLiteral("map_to_sview"));
	view.horizontal_scale = jsonHelper::getDouble(obj, QStringLiteral("horizontal_scale"));
	view.vertical_scale = jsonHelper::getDouble(obj, QStringLiteral("vertical_scale"));

	if (obj.contains("px_coordinate")) {
		auto pxObj = obj["px_coordinate"].toObject();
		view.px.cx = ScaleManager::instance().fov_to_world(jsonHelper::getDouble(pxObj, QStringLiteral("cx")));
		view.px.cy = ScaleManager::instance().fov_to_world(jsonHelper::getDouble(pxObj, QStringLiteral("cy")));
		view.px.w = ScaleManager::instance().fov_to_world(jsonHelper::getDouble(pxObj, QStringLiteral("w")));
		view.px.h = ScaleManager::instance().fov_to_world(jsonHelper::getDouble(pxObj, QStringLiteral("h")));
		view.px.xmin = ScaleManager::instance().fov_to_world(jsonHelper::getDouble(pxObj, QStringLiteral("xmin")));
		view.px.ymin = ScaleManager::instance().fov_to_world(jsonHelper::getDouble(pxObj, QStringLiteral("ymin")));
		view.px.xmax = ScaleManager::instance().fov_to_world(jsonHelper::getDouble(pxObj, QStringLiteral("xmax")));
		view.px.ymax = ScaleManager::instance().fov_to_world(jsonHelper::getDouble(pxObj, QStringLiteral("ymax")));

		if (!isAbsolute)
		{
			auto vpxc = getAbsoluteFOVCoordinates(QPointF(view.px.cx, view.px.cy));
			view.px.cx = vpxc.x();
			view.px.cy = vpxc.y();
			auto vpxmin = getAbsoluteFOVCoordinates(QPointF(view.px.xmin, view.px.ymin));
			view.px.xmin = vpxmin.x();
			view.px.ymin = vpxmin.y();
			auto vpxmax = getAbsoluteFOVCoordinates(QPointF(view.px.xmax, view.px.ymax));
			view.px.xmax = vpxmax.x();
			view.px.ymax = vpxmax.y();
		}
	
	}

	if (obj.contains("world_coordinate")) {
		auto worldObj = obj["world_coordinate"].toObject();
		fromJson(worldObj, view.world, isAbsolute);
		/*view.world.wx = jsonHelper::getDouble(worldObj, QStringLiteral("wx"));
		view.world.wy = jsonHelper::getDouble(worldObj, QStringLiteral("wy"));
		view.world.wz = jsonHelper::getDouble(worldObj, QStringLiteral("wz"));*/
		view.world.rx = jsonHelper::getDouble(worldObj, QStringLiteral("rx"));
		view.world.ry = jsonHelper::getDouble(worldObj, QStringLiteral("ry"));
		view.world.rz = jsonHelper::getDouble(worldObj, QStringLiteral("rz"));
	}

	if (obj.contains("vision_obj_ids")) {
		auto objVisionArray = obj["vision_obj_ids"].toArray();
		for (auto obj : objVisionArray)
		{
			view.vision_obj_IDs.append(obj.toString());
		}
	}
}

void VisionApp::toJson(const ct::Box2D & obj, QJsonObject & j)
{
	double cx = obj.cx;
	double cy = obj.cy;
	double xmin = obj.xmin;
	double ymin = obj.ymin;
	double xmax = obj.xmax;
	double ymax = obj.ymax;

	/*if (!isRelative)
	{
		auto c = getRelativeFOVCoordinates(QPointF(obj.cx, obj.cy));
		cx = c.x();
		cy = c.y();
		auto min = getRelativeFOVCoordinates(QPointF(obj.xmin, obj.ymin));
		xmin = min.x();
		ymin = min.y();
		auto max = getRelativeFOVCoordinates(QPointF(obj.xmax, obj.ymax));
		xmax = max.x();
		ymax = max.y();
	}*/
	

	j.insert(QStringLiteral("id"), obj.id.c_str());
	j.insert(QStringLiteral("cx"), cx);
	j.insert(QStringLiteral("cy"), cy);
	j.insert(QStringLiteral("w"), obj.w);
	j.insert(QStringLiteral("h"), obj.h);
	j.insert(QStringLiteral("xmin"),xmin);
	j.insert(QStringLiteral("ymin"), ymin);
	j.insert(QStringLiteral("xmax"), xmax);
	j.insert(QStringLiteral("ymax"), ymax);
}

void VisionApp::fromJson(const QJsonObject & j, ct::Box2D & obj)
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

	/*if (!isAbsolute)
	{
		auto c = getAbsoluteFOVCoordinates(QPointF(obj.cx, obj.cy));
		obj.cx = c.x();
		obj.cy = c.y();
		auto min = getAbsoluteFOVCoordinates(QPointF(obj.xmin, obj.ymin));
		obj.xmin = min.x();
		obj.ymin = min.y();
		auto max = getAbsoluteFOVCoordinates(QPointF(obj.xmax, obj.ymax));
		obj.xmax = max.x();
		obj.ymax = max.y();
	}*/

	obj.compute_extremum();
}

void VisionApp::toJson(const QDragBox* p, QJsonObject& obj)
{
	if (p == nullptr) return;

	obj.insert(QStringLiteral("id"), p->getId());
	obj.insert(QStringLiteral("name"), p->getName());

	auto rect = p->getGeometry();
	obj.insert(QStringLiteral("x"), rect.x());
	obj.insert(QStringLiteral("y"), rect.y());
	obj.insert(QStringLiteral("w"), rect.width());
	obj.insert(QStringLiteral("h"), rect.height());
}

void VisionApp::fromJson(QDragBox* p, const QJsonObject& obj)
{
	if (p == nullptr) return;

	auto id = jsonHelper::getString(obj, "id", "");
	auto name = jsonHelper::getString(obj, "name", "");
	auto x = jsonHelper::getDouble(obj, "x", 0);
	auto y = jsonHelper::getDouble(obj, "y", 0);
	auto w = jsonHelper::getDouble(obj, "w", 0);
	auto h = jsonHelper::getDouble(obj, "h", 0);

	p->setGeometry(QRectF(x, y, w, h));
	p->setID(id);
	p->setName(name);
}

void VisionApp::toJson(const nvs::motion::AxisConfig& a, QJsonObject& j)
{
	j["id"] = a.id;
	j["name"] = a.name.c_str();
	j["pulse_per(mm)"] = a.pulse_per_mm;

	j["move_acceleration"] = a.move_acceleration;
	j["move_deceleration"] = a.move_deceleration;
	j["move_start_velocity"] = a.move_start_velocity;
	j["move_max_velocity"] = a.move_max_velocity;

	j["home_acceleration"] = a.home_acceleration;
	j["home_deceleration"] = a.home_deceleration;
	j["home_to_origin_velocity"] = a.home_to_origin_velocity;
	j["home_start_velocity"] = a.home_start_velocity;
	j["home_max_velocity"] = a.home_max_velocity;

	j["home_mode"] = a.home_mode;

	j["positive_limit(mm)"] = a.positive_limit_mm;
	j["negative_limit(mm)"] = a.negative_limit_mm;
	j["max_allowable_velocity"] = a.max_allowable_velocity;
}

void VisionApp::fromJson(const QJsonObject& j, nvs::motion::AxisConfig& a)
{
	a.id = j["id"].toInt();
	a.name = j["name"].toString().toStdString();
	a.pulse_per_mm = j["pulse_per(mm)"].toDouble();

	a.move_acceleration = j["move_acceleration"].toDouble();
	a.move_deceleration = j["move_deceleration"].toDouble();
	a.move_start_velocity = j["move_start_velocity"].toDouble();
	a.move_max_velocity = j["move_max_velocity"].toDouble();

	a.home_acceleration = j["home_acceleration"].toDouble();
	a.home_deceleration = j["home_deceleration"].toDouble();
	a.home_to_origin_velocity = j["home_to_origin_velocity"].toDouble();
	a.home_start_velocity = j["home_start_velocity"].toDouble();
	a.home_max_velocity = j["home_max_velocity"].toDouble();

	a.home_mode = j["home_mode"].toInt();

	a.positive_limit_mm = j["positive_limit(mm)"].toDouble();
	a.negative_limit_mm = j["negative_limit(mm)"].toDouble();
	a.max_allowable_velocity = j["max_allowable_velocity"].toDouble();
}

void VisionApp::toJson(const nvs::motion::MotionConfig& m, QJsonObject& j)
{
	j["id"] = m.id.c_str();
	j["api"] = m.api.c_str();
	j["enable"] = m.enable;
	j["config_file"] = m.config_file.c_str();

	QJsonArray j_axes;
	for (const auto& axis : m.axes) {
		QJsonObject j_axis;
		toJson(axis, j_axis);
		j_axes.append(j_axis);
	}
	j["axes"] = j_axes;
}

void VisionApp::fromJson(const QJsonObject& j, nvs::motion::MotionConfig& m)
{
	m.id = j["id"].toString().toStdString();
	m.api = j["api"].toString().toStdString();
	m.enable = j["enable"].toBool();
	m.config_file = j["config_file"].toString().toStdString();

	m.axes.clear();
	for (const auto& v : j["axes"].toArray()) {
		nvs::motion::AxisConfig a;
		fromJson(v.toObject(), a);
		m.axes.push_back(a);
	}
}

void VisionApp::toJson(const dat::WorldCoordinate & obj, QJsonObject & j, bool isRelative)
{
	dat::WorldCoordinate relativeObj = obj;
	if(!isRelative) relativeObj = getRelativeRobotPoint(obj);

	j.insert(QStringLiteral("wx"), relativeObj.wx);
	j.insert(QStringLiteral("wy"), relativeObj.wy);
	j.insert(QStringLiteral("wz"), relativeObj.wz);
}

void VisionApp::fromJson(const QJsonObject & j, dat::WorldCoordinate & obj, bool isAbsolute)
{
	obj.wx = jsonHelper::getDouble(j, "wx");
	obj.wy = jsonHelper::getDouble(j, "wy");
	obj.wz = jsonHelper::getDouble(j, "wz");

	if(!isAbsolute) obj = getAbsoluteRobotPoint(obj);
}

void VisionApp::recipeSanitaryCheck()
{
	//check optic
	for (auto& opt : _recipeOptics) {

		bool R_is_empty = true;
		bool G_is_empty = true;
		bool B_is_empty = true;

		bool M_is_empty = true;

		for (const auto& intensity : opt.R) {
			if (intensity != 0) {
				R_is_empty = false;
				break;
			}
		}

		for (const auto& intensity : opt.G) {
			if (intensity != 0) {
				G_is_empty = false;
				break;
			}
		}

		for (const auto& intensity : opt.B) {
			if (intensity != 0) {
				B_is_empty = false;
				break;
			}
		}

		for (const auto& intensity : opt.M) {
			if (intensity != 0) {
				M_is_empty = false;
				break;
			}
		}

		if (R_is_empty && G_is_empty && B_is_empty && M_is_empty) {
			ct::logger::error("Optics is empty: %s", opt.name.toStdString().c_str());
			showMsg(QString("Optic %1 is empty, please check if the settings are correct").arg(opt.name));
			break;
		}
	}
}

//bool VisionApp::saveSetupConfig()
//{
//	auto jsonPath = SystemData::instance()._workingPath + "/setupConfig.json";
//
//	QJsonObject j_root;
//	j_root.insert(QStringLiteral("code"), SystemData::instance()._currentBarcode.c_str());
//
//	auto ret = jsonHelper::saveJson(jsonPath, QJsonDocument(j_root));
//	if (ret) ct::logger::info("Successfully saved barcode result!");
//	else ct::logger::error("Failed to save barcode result!");
//}
