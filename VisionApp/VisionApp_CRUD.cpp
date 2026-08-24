#include "VisionApp.h"
#include "QLineItem.h"
#include "BoxCluster.h"
#include "ImagePathManager.h" 
#include "uidGenerator.h"
#include "EM_TSP.h"
#include "TemplateLibraryTab.h"
#include "CAMManager.h"
#include "QRectItem.h"
#include "ScaleManager.h"

void VisionApp::assignSingleViewForWholePlane()
{
	clearView();

	auto w = _imageWorld.width();
	auto h = _imageWorld.height();
	
	std::string viewID = "view_1";
	auto viewBox = drawViewBox(QRectF(0, 0, w, h), QColor(0, 255, 127), viewID.c_str());
	ct::Box2D box;
	box.id = viewID;
	box.cx = w / 2;
	box.cy = h / 2;
	box.w = w;
	box.h = h;
	box.compute_extremum();

	QView v;
	uidGenerator idGen;
	v.id = QString("view") + idGen.id().c_str();
	v.name = QString("view_1");
	v.horizontal_scale = ScaleManager::instance().horizontal_um_per_px();
	v.vertical_scale = ScaleManager::instance().vertical_um_per_px();
	v.created_by = "";
	v.map_to_sview = "";
	v.px = ScaleManager::instance().world_to_fov(box);

	auto wmm = ScaleManager::instance().to_world_mm(QPointF(box.cx, box.cy));
	v.world.wx = wmm.x();
	v.world.wy = wmm.y();
	v.world.wz = _plane.corner_points[(int)Corner::FRONTLEFT].wz;

	viewBox->setID(v.id);
	v.pDragBox = viewBox;
	_views.insert(v.id, v);

	clearBufferQueue();
	createCamAlpha();

	//set start and end Path to the single View then auto generate path
	ui.tb_setStartPoint->setText(v.name);
	ui.tb_setStartPoint->setWhatsThis(v.id);
	ui.tb_setEndPoint->setText(v.name);
	ui.tb_setEndPoint->setWhatsThis(v.id);
	generatePath();

	//add VisionDragBox that covers the whole view and create and set a template for it
	clearVisionObject();

	auto cam_w = CAMManager::instance().getWidth(_camID);
	auto cam_h = CAMManager::instance().getHeight(_camID);

	auto worldScale = ScaleManager::instance().world_scale();
	int viewWidth = cam_w * worldScale;
	int viewHeight = cam_h * worldScale;
	addVisionObject(QRectF(0,0, viewWidth, viewHeight));

	//add New Template list
	_templateLibraryTab->addTemplate();
	//auto set new vision Object as default vision Object for the templateList
	setVisionObjectAsDefaultTemplate();

}

void VisionApp::generateSingleViewImages()
{

	if (_views.size() != 1)
	{
		QMessageBox::warning(this, ("Unable to generate Single View ImageSet!"),
			"There are more than one views in this recipe!!! Unable to generate single view imageset");
		return;
	}

	bool ok  = false;
	QStringList imageSetType;
	imageSetType << "Sample" << "Setup";
	QString imageSetDirectory = QInputDialog::getItem(this, tr("ImageSet Directory"), tr("Directory:"), imageSetType, 0, false, &ok, Qt::CoverWindow);

	if (imageSetDirectory == "Sample")
	{
		ui.comboBox_collectView->setCurrentIndex(0);
	}
	else if (imageSetDirectory == "Setup")
	{
		ui.comboBox_collectView->setCurrentIndex(1);
	}

	QView view;
	for (auto v : _views)
	{
		view = v;
	}

	_mainOptics[_camID].id;
	QString imgPath = Common::Directory::getRecipeImagesPath();
	QStringList fileNames = QFileDialog::getOpenFileNames(this, tr("Select images to inspect"), imgPath, "Image File (*.png *.jpg *.bmp *.tiff)");

	auto cam_w = CAMManager::instance().getWidth(_camID);
	auto cam_h = CAMManager::instance().getHeight(_camID);

	for (int i = 0; i < fileNames.size(); i++)
	{
		auto w = mtrx::get_width(fileNames[i]);
		auto h = mtrx::get_height(fileNames[i]);
		
		if (cam_w == w && cam_h == h)
		{
			auto ipf = path::getViewPath(getViewCollectionPath().toStdString(), view, _mainOptics[_camID], _recipeOptics);
			QString sourceFilePath = fileNames[i];
			QString destinationFilePath = ipf.getMainOpticPath().c_str();
			QImage viewImage(sourceFilePath);
			viewImage.save(destinationFilePath);
			
			qDebug() << ipf.getMainOpticPath().c_str();
		}
	}
	//create SampleImages
	// - check if all image selected are from the same size with the CAM_WIDTH and CAM_HEIGHT
	// 1. create sample folder in recipe imageset path
	// 2. get the viewID of the single View
	// 3. generate the img name based on the viewID and opticID selected
	// 4. make a copy of the images into the new directory using the imgName generated
}

void VisionApp::highlightViewInWorld(QString id)
{
	for (auto& p : _viewROI) {
		if (p->getId() != id) {
			p->setSelected(false);
		}
		else {
			p->setSelected(true);
		}
	}
}

void VisionApp::assignViewsToOversizedVO(VisionAppQDragBox* vo, int padding_wpx, int overlap_percentage)
{
	auto voID = vo->getId();

	ct::logger::info("Start assigning to oversized VO");

	/*
	* Condition to fulfill:
	* - To support padding
	* - To support minimum overlap percentage
	* - Able to center FOV when there's not a need for stitching
	*/

	//convert all into horizontal and vertical axis
	double h_scale = ScaleManager::instance().horizontal_um_per_px();
	double v_scale = ScaleManager::instance().vertical_um_per_px();

	//Add padding 
	auto rect = vo->getGeometry();
	rect.setBottomRight(QPointF(rect.bottomRight().x() + padding_wpx, rect.bottomRight().y() + padding_wpx));
	rect.setTopLeft(QPointF(rect.topLeft().x() - padding_wpx, rect.topLeft().y() - padding_wpx));
	auto br = ScaleManager::instance().to_world_mm(rect.bottomRight());
	auto fl = ScaleManager::instance().to_world_mm(rect.topLeft());
	
	//Get FOV
	auto cam_w = CAMManager::instance().getWidth(_camID);
	auto cam_h = CAMManager::instance().getHeight(_camID);
	double h_cam_mm = util::px_to_mm(cam_w, h_scale);
	double v_cam_mm = util::px_to_mm(cam_h, v_scale);

	//Get Target Size to fill up
	double h_target_mm = abs(br.x() - fl.x());
	double v_target_mm = abs(br.y() - fl.y());

	//Set minimum overlap percentage
	double h_percentage = (double)overlap_percentage / 100.0;
	double v_percentage = (double)overlap_percentage / 100.0;
	double h_overlap_mm = h_cam_mm * h_percentage;
	double v_overlap_mm = v_cam_mm * v_percentage;

	/*
	* n: Minimum number of FOV
	* t: Target size in mm
	* o: Minimum overlap in mm
	* f: FOV size in mm
	* 
	* To get minimum number of FOV
	* n = ceil((t - o)/(f - o))
	* 
	* To get minimum overlap in mm
	* o = (n * f - t) / n - 1
	*/
	//Calculate minimum number of FOV required
	int h_num = std::ceil((h_target_mm - h_overlap_mm) / (h_cam_mm - h_overlap_mm));
	int v_num = std::ceil((v_target_mm - v_overlap_mm) / (v_cam_mm - v_overlap_mm));

	//Recalculate minimum overlap based on minimum number of FOV
	h_overlap_mm = ((h_num * h_cam_mm) - h_target_mm) / (h_num - 1);
	v_overlap_mm = ((v_num * v_cam_mm) - v_target_mm) / (v_num - 1);

	//Handle target that is smaller than FOV
	double h_centerOffset_mm = 0.0;
	if (h_num == 1) {
		h_overlap_mm = 0.0;
		h_centerOffset_mm = (h_target_mm / 2) - (h_cam_mm / 2);
	}

	double v_centerOffset_mm = 0.0;
	if (v_num == 1) {
		v_overlap_mm = 0.0;
		v_centerOffset_mm = (v_target_mm / 2) - (v_cam_mm / 2);
	}

	//Calculate each iteration's offset
	double h_offset_mm = h_cam_mm - h_overlap_mm;
	double v_offset_mm = v_cam_mm - v_overlap_mm;

	ct::logger::trace("h_num: %d", h_num);
	ct::logger::trace("v_num: %d", v_num);

	ct::logger::trace("h_overlap: %f", h_overlap_mm);
	ct::logger::trace("v_overlap: %f", v_overlap_mm);

	ct::logger::trace("h_offset: %f", h_offset_mm);
	ct::logger::trace("v_offset: %f", v_offset_mm);

	ct::logger::trace("h_cam_mm: %f", h_cam_mm);
	ct::logger::trace("v_cam_mm: %f", v_cam_mm);

	ct::logger::trace("h_target_mm: %f", h_target_mm);
	ct::logger::trace("v_target_mm: %f", v_target_mm);
	
	ct::logger::trace("h_centerOffset_mm: %f", h_centerOffset_mm);
	ct::logger::trace("v_centerOffset_mm: %f", v_centerOffset_mm);

	uidGenerator sgen;
	auto sview_id = QString("sview") + sgen.id().c_str();

	for (int r = 0; r < v_num; r++) {
		for (int c = 0; c < h_num; c++) {

			QView v;
			uidGenerator idGen;
			v.id = QString("%1-%2-%3").arg(sview_id).arg(r).arg(c);
			v.name = QString("view_") + QString::number(_views.size());
			v.horizontal_scale = h_scale;
			v.vertical_scale = v_scale;
			v.created_by = "";
			v.type = ct::s_child_view;
			v.map_to_sview = sview_id;
			v.world.wx = h_offset_mm * c + fl.x() + (h_cam_mm / 2) + h_centerOffset_mm;
			v.world.wy = fl.y() + (v_cam_mm / 2) + (v_offset_mm * r) + v_centerOffset_mm;
			v.world.wz = _plane.corner_points[(int)Corner::FRONTLEFT].wz;
			
			auto wpx = getAbsoluteFOVCoordinates(QPointF(v.world.wx, v.world.wy));
			v.px.cx = ScaleManager::instance().fov_to_world(util::mm_to_px(v.world.wx, h_scale));
			v.px.cy = ScaleManager::instance().fov_to_world(util::mm_to_px(v.world.wy, v_scale));
			v.px.w = ScaleManager::instance().fov_to_world(cam_w);
			v.px.h = ScaleManager::instance().fov_to_world(cam_h);
			v.px.compute_extremum();

			auto dragBox = drawViewBox(QRectF(v.px.cx - (v.px.w/2), v.px.cy - (v.px.h/2), v.px.w, v.px.h), QColor(0, 255, 127), v.id);
			dragBox->setID(v.id);

			v.px = ScaleManager::instance().world_to_fov(v.px);

			for (const auto& optic : _recipeOptics) {
				v.opticIDs.insert(optic.id);
			}

			v.pDragBox = dragBox;

			v.vision_obj_IDs.append(voID);

			auto ipf = path::getViewPath(Common::Directory::getRecipeSetupImagePath().toStdString(), v);
			auto croppedView = _imageWorld.copy(QRect(v.px.cx, v.px.cy, v.px.w, v.px.h));
			QImage scaledCroppedView = croppedView.scaled(_imageSize.rwidth(), _imageSize.rheight(), Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
			scaledCroppedView.save(ipf.GetPath().c_str());

			_views.insert(v.id, v);

			processEvents();
		}
	}

	//add data for final stitch view
	auto center_mm = ScaleManager::instance().to_world_mm(rect.center());

	QView v;
	v.id = sview_id;
	v.name = sview_id;
	v.horizontal_scale = h_scale;
	v.vertical_scale = v_scale;
	v.created_by = "";
	v.type = ct::s_stitch_view;
	v.map_to_sview = "";
	v.world.wx = center_mm.x();
	v.world.wy = center_mm.y();
	v.world.wz = _plane.corner_points[(int)Corner::FRONTLEFT].wz;

	auto wpx = getAbsoluteFOVCoordinates(QPointF(v.world.wx, v.world.wy));
	v.px.cx = ScaleManager::instance().fov_to_world(util::mm_to_px(v.world.wx, h_scale));
	v.px.cy = ScaleManager::instance().fov_to_world(util::mm_to_px(v.world.wy, v_scale));
	v.px.w = ScaleManager::instance().fov_to_world(util::mm_to_px(h_target_mm, h_scale));
	v.px.h = ScaleManager::instance().fov_to_world(util::mm_to_px(v_target_mm, v_scale));
	if (h_num == 1) v.px.w = ScaleManager::instance().fov_to_world(cam_w);
	if (v_num == 1) v.px.h = ScaleManager::instance().fov_to_world(cam_h);
	v.px.compute_extremum();

	auto dragBox = drawViewBox(QRectF(v.px.cx - (v.px.w / 2), v.px.cy - (v.px.h / 2), v.px.w, v.px.h), QColor(0, 255, 127), v.id);
	dragBox->setID(v.id);

	v.px = ScaleManager::instance().world_to_fov(v.px);

	for (const auto& optic : _recipeOptics) {
		v.opticIDs.insert(optic.id);
	}

	v.pDragBox = dragBox;

	v.vision_obj_IDs.append(voID);
	_views.insert(v.id, v);


	_visionObject[voID].locked = false;
	_visionObject[voID].viewID = sview_id;

	if (_visionObject[voID].pDragBox) {
		_visionObject[voID].pDragBox->viewID(sview_id);
		_visionObject[voID].pDragBox->setDragable(true);
	}

	return;
}

/* Create related */
void VisionApp::assignViews(double padding_mm, int overlap_percentage)
{
	ScopedTimeLogger t("Assign views");

	auto svo = getSelectedVisionObject();

	if (svo.size() == 0) {
		showMsg("No vision object selected!");
		return;
	}

	bool unassignOnly = false;
	if (ui.comboBox_viewAssignmentMethod->currentText() == "All Selected Region") clearView();
	else if (ui.comboBox_viewAssignmentMethod->currentText() == "Unassigned Selected Region") unassignOnly = true;


	/*
	* Assignment priority
	* Big ROI assign first, remaining unassigned ROI will be assigned by clustering
	*/
	auto cam_w = CAMManager::instance().getWidth(_camID);
	auto cam_h = CAMManager::instance().getHeight(_camID);

	auto worldScale = ScaleManager::instance().world_scale();
	auto h_scale = ScaleManager::instance().horizontal_um_per_px();
	auto v_scale = ScaleManager::instance().vertical_um_per_px();

	int viewWidth = cam_w * worldScale;
	int viewHeight = cam_h * worldScale;

	QVector<VisionAppQDragBox*> bigROIs;
	std::vector<ct::SmallBox> rois;
	std::vector<ct::ClusterBox> views;

	for (auto& vo : svo) {
		auto rect = vo->getGeometry();

		if (vo->viewID() != "" && unassignOnly) continue;

		if (rect.width() > viewWidth || rect.height() > viewHeight) {
			bigROIs.push_back(vo);
		}
		else {
			auto x = vo->getGeometry().x();
			auto y = vo->getGeometry().y();
			auto w = vo->getGeometry().width();
			auto h = vo->getGeometry().height();

			ct::SmallBox roi;
			roi.id = vo->getId().toStdString();
			roi.box.cx = x + w / 2;
			roi.box.cy = y + h / 2;
			roi.box.w = w;
			roi.box.h = h;

			roi.box.compute_extremum();

			rois.emplace_back(roi);
		}
	}

	int padding_wpx = util::mm_to_px(padding_mm, h_scale) * worldScale;

	for (auto& roi : bigROIs) {
		assignViewsToOversizedVO(roi, padding_wpx, overlap_percentage);
	}

	ct::cluster_boxes(cam_w * worldScale, cam_h * worldScale, padding_wpx, rois, views);

	ct::logger::debug("1");

	for (auto wpx : views) {
		auto& b = wpx.box;
		auto dragBox = drawViewBox(QRectF(b.xmin, b.ymin, b.w, b.h), QColor(0, 255, 127), wpx.id.c_str());

		QView v;
		uidGenerator idGen;
		v.id = QString("view") + idGen.id().c_str();
		v.name = QString("view_") + QString::number(_views.size());
		v.horizontal_scale = h_scale;
		v.vertical_scale = v_scale;
		v.created_by = "";
		v.map_to_sview = "";
		v.type = ct::s_view;
		v.px = ScaleManager::instance().world_to_fov(wpx.box);
		ct::logger::debug("2");
		auto wmm = ScaleManager::instance().to_world_mm(QPointF(wpx.box.cx, wpx.box.cy));
		v.world.wx = wmm.x(); 
		v.world.wy = wmm.y(); 
		v.world.wz = _plane.corner_points[(int)Corner::FRONTLEFT].wz;

		for (const auto& optic : _recipeOptics) {
			v.opticIDs.insert(optic.id);
		}

		dragBox->setID(v.id);
		v.pDragBox = dragBox;
		ct::logger::debug("3");
		for (auto box : wpx.boxes) {
			QString id = box.id.c_str();

			v.vision_obj_IDs.append(id);

			_visionObject[id].locked = false;
			_visionObject[id].viewID = v.id;

			if (_visionObject[id].pDragBox) {
				_visionObject[id].pDragBox->viewID(v.id);
				_visionObject[id].pDragBox->setDragable(true);
			}
		}
		ct::logger::debug("4");
		auto ipf = path::getViewPath(Common::Directory::getRecipeSetupImagePath().toStdString(), v);
		auto croppedView = _imageWorld.copy(QRect(wpx.box.xmin, wpx.box.ymin, wpx.box.w, wpx.box.h));
		QImage scaledCroppedView = croppedView.scaled(_imageSize.rwidth(), _imageSize.rheight(), Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
		scaledCroppedView.save(ipf.GetPath().c_str());
		ct::logger::debug("5: %s", v.id.toStdString().c_str());
		_views.insert(v.id, v);

		processEvents();
	}

	updateTreeViewExplorer(Common::Directory::CurrentRecipe, _views, _visionObject);
	updateViewComboBoxUI();

	refreshDragBoxSequence();

	generateBestPath();

	saveRecipe();
}

void VisionApp::addView()
{
	auto viewID = ui.lineEdit_viewName->text();

	auto& wmm = SystemData::instance().currentCoordinate();

	auto wpx = ScaleManager::instance().to_world_px(QPointF(wmm.wx, wmm.wy));

	auto cam_w = CAMManager::instance().getWidth(_camID);
	auto cam_h = CAMManager::instance().getHeight(_camID);

	auto wpx_cam_w = ScaleManager::instance().fov_to_world(cam_w);
	auto wpx_cam_h = ScaleManager::instance().fov_to_world(cam_h);
	auto half_wpx_cam_w = wpx_cam_w / 2;
	auto half_wpx_cam_h = wpx_cam_h / 2;

	ct::Box2D wpx_box;
	wpx_box.cx = wpx.x();
	wpx_box.cy = wpx.y();
	wpx_box.w = wpx_cam_w;
	wpx_box.h = wpx_cam_h;
	wpx_box.compute_extremum();

	auto dragBox = drawViewBox(QRectF(wpx_box.xmin, wpx_box.ymin, wpx_box.w, wpx_box.h), QColor(0, 255, 127), viewID);

	QView v;
	v.id = viewID;
	v.name = v.id;
	v.horizontal_scale = ScaleManager::instance().horizontal_um_per_px();
	v.vertical_scale = ScaleManager::instance().vertical_um_per_px();
	v.created_by = "";
	v.map_to_sview = "";
	v.type = ct::s_view;
	v.px = ScaleManager::instance().world_to_fov(wpx_box); 

	for (const auto& optic : _recipeOptics) {
		v.opticIDs.insert(optic.id);
	}

	dragBox->setID(v.id);
	v.pDragBox = dragBox;

	SystemData::instance()._workingPath = Common::Directory::getRecipeSetupImagePath();

	_processType = ProcessType::IMAGE_COLLECTION;

	_views.insert(v.id, v);

	CAMManager::instance().frame(_camID)->viewID = v.id;
	for (const auto& optic : _recipeOptics) {
		emit snapImage(optic, "", "", false);
	}

	processEvents();

	//crash somewhere here
	updateTreeViewExplorer(Common::Directory::CurrentRecipe, _views, _visionObject);
	updateViewComboBoxUI();

	refreshDragBoxSequence();

	saveRecipe();

	generateBestPath();
}

void VisionApp::generateBestPath()
{
	ScopedTimeLogger t("Generate Best Path");

	QString TLid, TRid, BLid, BRid;
	QPointF TL, TR, BL, BR;
	TL = QPointF(std::numeric_limits<int>::max(), std::numeric_limits<int>::max());
	TR = QPointF(std::numeric_limits<int>::min(), std::numeric_limits<int>::max());
	BL = QPointF(std::numeric_limits<int>::max(), std::numeric_limits<int>::min());
	BR = QPointF(std::numeric_limits<int>::min(), std::numeric_limits<int>::min());

	for (auto& v : _views) {
		if (v.type == ct::s_stitch_view) continue;

		auto p = v.pDragBox->pos();
		// Top-left: Smallest x and smallest y
		if (p.x() <= TL.x() && p.y() <= TL.y()) {
			TL = p;
			TLid = v.id;
		}

		// Top-right: Largest x and smallest y
		if (p.x() >= TR.x() && p.y() <= TR.y()) {
			TR = p;
			TRid = v.id;
		}

		// Bottom-left: Smallest x and largest y
		if (p.x() <= BL.x() && p.y() >= BL.y()) {
			BL = p;
			BLid = v.id;
		}

		// Bottom-right: Largest x and largest y
		if (p.x() >= BR.x() && p.y() >= BR.y()) {
			BR = p;
			BRid = v.id;
		}
	}

	struct Key {
		Key(QString s, QString e) {
			startID = s;
			endID = e;
		}

		QString startID, endID;
	};

	std::vector<Key> keys;
	keys.push_back(Key(TLid, TRid));
	keys.push_back(Key(TLid, BLid));
	keys.push_back(Key(TLid, BRid));
	keys.push_back(Key(TRid, BLid));
	keys.push_back(Key(TRid, BRid));
	keys.push_back(Key(BLid, BRid));

	//Get shortest path
	Key shortestKey("","");
	double shortestDistance = std::numeric_limits<double>::max();
	QVector<QString> sequenceViewID;

	for (auto& key : keys) {
		em::TourGenerator tg;
		em::TSPAlgorithm algo = em::TSPAlgorithm::CHEAPEST_INSERTION;
		em::TSPDirectionPriority priority = em::TSPDirectionPriority::VERTICAL;

		tg.set_direction_priority(priority);

		QHash<QString, int> idToIndexMap;
		QHash<int, QString> indexToIDMap;
		int index = 0;
		for (auto view : _views) {
			if (view.type == ct::s_stitch_view) continue;

			idToIndexMap.insert(view.id, index);
			indexToIDMap.insert(index, view.id);
			tg.vertices().emplace_back(em::Vertex(index, view.px.cx, view.px.cy));
			index++;
		}
		ct::logger::debug("2");
		if (!idToIndexMap.contains(key.startID) || !idToIndexMap.contains(key.endID)) {
			printf("[UB] Path generator invalid start/end key.\n");
			return;
		}

		tg.set_start_point(idToIndexMap[key.startID]); //set first view
		tg.set_end_point(idToIndexMap[key.endID]);
		tg.compute_tour(algo);
		ct::logger::debug("3");
		auto& sequence = tg.sequence();

		//add to preview list
		double totalDistance = 0;
		for (int i = 0; i < sequence.size() - 1; i++) {
			auto s1 = sequence[i];
			auto s2 = sequence[i + 1];
			auto& vertex1 = tg.vertices()[s1];
			auto& vertex2 = tg.vertices()[s2];
			totalDistance += em::distance(vertex1, vertex2);
		}

		ct::logger::info("Start: %s, End: %s, Distance: %.2f", key.startID.toStdString().c_str(), key.endID.toStdString().c_str(), totalDistance);

		if (totalDistance < shortestDistance) {
			shortestKey.startID = key.startID;
			shortestKey.endID = key.endID;
			shortestDistance = totalDistance;

			sequenceViewID.clear();

			for (auto s : sequence) {
				auto& vertex = tg.vertices()[s];

				if (!_views.contains(indexToIDMap[vertex.index()])) {
					ct::logger::error("[IM] Failed to generate best path. Invalid view ID: %s", indexToIDMap[vertex.index()].toStdString().c_str());
					continue;
				}

				auto& view = _views[indexToIDMap[vertex.index()]];
				sequenceViewID.push_back(view.id);
			}
		}
	}

	ct::logger::info("Shortest Start: %s, End: %s, Distance: %.2f", shortestKey.startID.toStdString().c_str(), shortestKey.endID.toStdString().c_str(), shortestDistance);
	clearPath();

	for (auto viewID : sequenceViewID) {
		addViewToPath(viewID);
	}

	ct::logger::debug("4");
	updateTreeViewExplorer(Common::Directory::CurrentRecipe, _views, _visionObject);
	ct::logger::debug("5");
	saveRecipe();
}

void VisionApp::reassignVoIntoLineScans()
{
	//clear all VO from linescan and clear all linescanIDs from VO
	//check which VO falls into which linescan

	for (auto& lineScan : _lineScans)
	{
		lineScan.vision_obj_IDs.clear();
	}

	for (auto& dragBox : _dragROI)
	{
		bool voIncludedFlag = false;
		for (int i = 0; i < _lineScanROI.size(); i++)
		{
			int rectX = dragBox->getGeometry().x();
			int rectY = dragBox->getGeometry().y();
			int rectW = dragBox->getGeometry().width();
			int rectH = dragBox->getGeometry().height();

			auto lineScanRect = _lineScanROI[i]->getGeometry();
			if (rectX >= (int)lineScanRect.x() && rectX + rectW <= (int)lineScanRect.x() + (int)lineScanRect.width() &&
				rectY >= (int)lineScanRect.y() && rectY + rectH <= (int)lineScanRect.y() + (int)lineScanRect.height())
			{
				dragBox->lineScanID(_lineScanROI[i]->getId());
				auto vo = _visionObject.find(dragBox->getId());
				if (vo != _visionObject.end())
				{
					vo.value().lineScanID = _lineScanROI[i]->getId();
					voIncludedFlag = true;
				}

				_lineScans[_lineScanROI[i]->getId()].vision_obj_IDs.append(dragBox->getId());			
			}
		}

		if (!voIncludedFlag)
		{
			auto vo = _visionObject.find(dragBox->getId());
			dragBox->lineScanID("");
			vo.value().lineScanID = "";
		}
	}

	saveLineScans();
	ct::logger::info("reassignVoIntoLineScans");
}

QVector<GroupedOverSizedVO> VisionApp::groupOverSizedVOForLineScan(QVector <VisionAppQDragBox*> bigVOs) {

	QVector<GroupedOverSizedVO> groupedVOs;
	QSet<QString> assignedVOs;

	//group along the axis the FOV strips stack on: Y for X-axis scans, X for Y-axis scans
	const bool scanAlongY = SystemData::instance().isLineScanAxisY();

	for (auto vo : bigVOs) {

		auto id = vo->getId();

		if (assignedVOs.contains(id)) continue;

		assignedVOs.insert(id);

		GroupedOverSizedVO gvo;
		gvo.VOs.append(vo);
		//ct::logger::trace("parent vo: %s", id.toStdString().c_str());

		auto rect = vo->getGeometry();
		auto start = scanAlongY ? rect.x() : rect.y();
		auto end = start + (scanAlongY ? rect.width() : rect.height());
		auto range = end - start;
		auto pad = range * 0.35;
		start += pad;
		end -= pad;

		/*ct::logger::trace("start: %f", start);
		ct::logger::trace("end: %f", end);
		ct::logger::trace("pad: %f", pad);*/

		for (auto childVO : bigVOs) {
			auto childID = childVO->getId();

			if (assignedVOs.contains(childID)) continue;

			//ct::logger::trace("child vo: %s", childID.toStdString().c_str());

			auto childRect = childVO->getGeometry();
			auto childStart = scanAlongY ? childRect.x() : childRect.y();
			auto childEnd = childStart + (scanAlongY ? childRect.width() : childRect.height());
			auto childCenter = (childStart + childEnd) / 2;

			//ct::logger::trace("child center: %f", childCenter);

			if (childCenter > start && childCenter < end) {
				assignedVOs.insert(childID);
				gvo.VOs.append(childVO);
			}
		}

		groupedVOs.append(gvo);
	}

	for (auto& gvo : groupedVOs) {

		double sx = 999999999999; //smallest x and y
		double sy = 999999999999;
		double lx = 0; //largest x and y
		double ly = 0;

		for (auto vo : gvo.VOs) {
			auto rect = vo->getGeometry();

			if (rect.x() < sx) sx = rect.x();
			if (rect.y() < sy) sy = rect.y();

			auto current_lx = rect.x() + rect.width();
			auto current_ly = rect.y() + rect.height();

			if (current_lx > lx) lx = current_lx;
			if (current_ly > ly) ly = current_ly;
		}

		//gvo.rect.setTopLeft(QPointF(sx, sy));
		//gvo.rect.setBottomRight(QPointF(lx, ly));
		/*gvo.rect.setY(sy);
		gvo.rect.setWidth(lx - sx);
		gvo.rect.setHeight(ly - sy);*/

		ct::logger::trace("sx: %f, sy: %f, lx: %f, ly: %f", sx, sy, lx, ly);
		gvo.rect = QRectF(sx, sy, lx - sx, ly - sy);

		qDebug() << "BASAL: " << gvo.rect;
	}

	return groupedVOs;
}

void VisionApp::assignLineScansToOversizedVO(VisionAppQDragBox* vo, int padding_wpx, int overlap_percentage)
{
	auto voID = vo->getId();

	ct::logger::info("Start assigning to oversized VO");

	/*
	* Condition to fulfill:
	* - To support padding
	* - To support minimum overlap percentage
	* - Able to center FOV when there's not a need for stitching
	*/

	//convert all into horizontal and vertical axis
	double h_scale = ScaleManager::instance().horizontal_um_per_px();
	double v_scale = ScaleManager::instance().vertical_um_per_px();

	//Add padding 
	auto rect = vo->getGeometry();
	
	rect.setBottomRight(QPointF(rect.bottomRight().x() + padding_wpx, rect.bottomRight().y() + padding_wpx));
	rect.setTopLeft(QPointF(rect.topLeft().x() - padding_wpx, rect.topLeft().y() - padding_wpx));
	auto br = ScaleManager::instance().to_world_mm(rect.bottomRight());
	auto fl = ScaleManager::instance().to_world_mm(rect.topLeft());

	//Get Target Size to fill up
	double h_target_mm = abs(br.x() - fl.x());
	double h_center_mm = (br.x() + fl.x()) / 2;
	double v_target_mm = abs(br.y() - fl.y());
	double v_center_mm = (br.y() + fl.y()) / 2;

	//scan axis: direction the gantry travels; step axis: direction the FOV strips stack
	const bool scanAlongY = SystemData::instance().isLineScanAxisY();
	double scan_target_mm = scanAlongY ? v_target_mm : h_target_mm;
	double scan_center_mm = scanAlongY ? v_center_mm : h_center_mm;
	double step_target_mm = scanAlongY ? h_target_mm : v_target_mm;
	double step_start_mm = scanAlongY ? fl.x() : fl.y();

	//Get FOV (laser FOV perpendicular to the scan direction)
	double step_cam_mm = 14.5;

	//Set minimum overlap percentage
	double step_percentage = (double)overlap_percentage / 100.0;
	double step_overlap_mm = step_cam_mm * step_percentage;

	/*
	* n: Minimum number of FOV
	* t: Target size in mm
	* o: Minimum overlap in mm
	* f: FOV size in mm
	*
	* To get minimum number of FOV
	* n = ceil((t - o)/(f - o))
	*
	* To get minimum overlap in mm
	* o = (n * f - t) / n - 1
	*/
	//Calculate minimum number of FOV required
	int step_num = std::ceil((step_target_mm - step_overlap_mm) / (step_cam_mm - step_overlap_mm));

	//Recalculate minimum overlap based on minimum number of FOV
	step_overlap_mm = ((step_num * step_cam_mm) - step_target_mm) / (step_num - 1);

	double step_centerOffset_mm = 0.0;
	if (step_num == 1) {
		step_overlap_mm = 0.0;
		step_centerOffset_mm = (step_target_mm / 2) - (step_cam_mm / 2);
	}

	//Calculate each iteration's offset
	double step_offset_mm = step_cam_mm - step_overlap_mm;

	ct::logger::trace("step_num: %d", step_num);

	ct::logger::trace("step_overlap: %f", step_overlap_mm);

	ct::logger::trace("step_offset: %f", step_offset_mm);

	ct::logger::trace("step_cam_mm: %f", step_cam_mm);

	ct::logger::trace("scan_target_mm: %f", scan_target_mm);
	ct::logger::trace("step_target_mm: %f", step_target_mm);

	ct::logger::trace("step_centerOffset_mm: %f", step_centerOffset_mm);

	uidGenerator sgen;
	auto slinescan_id = QString("slinescan") + sgen.id().c_str();

	auto wz = _plane.corner_points[(int)Corner::FRONTLEFT].wz;

	for (int r = 0; r < step_num; r++) {

		QLineScan v;
		v.id = slinescan_id + "-" + QString::number(r);
		v.name = QString("linescan_") + QString::number(_lineScans.size());
		v.created_by = "";
		v.type = ct::s_child_linescan;
		v.map_to_slinescan = slinescan_id;
		auto step_mm = step_start_mm + (step_cam_mm / 2) + (step_offset_mm * r) + step_centerOffset_mm;
		auto wx = scanAlongY ? step_mm : scan_center_mm;
		auto wy = scanAlongY ? scan_center_mm : step_mm;

		auto wpx = getAbsoluteFOVCoordinates(QPointF(wx, wy));
		v.px.cx = ScaleManager::instance().fov_to_world(util::mm_to_px(wx, h_scale));
		v.px.cy = ScaleManager::instance().fov_to_world(util::mm_to_px(wy, h_scale));
		v.px.w = ScaleManager::instance().fov_to_world(util::mm_to_px(scanAlongY ? step_cam_mm : scan_target_mm, h_scale));
		v.px.h = ScaleManager::instance().fov_to_world(util::mm_to_px(scanAlongY ? scan_target_mm : step_cam_mm, h_scale));
		v.px.compute_extremum();

		auto dragBox = drawLineScan(QRectF(v.px.cx - (v.px.w / 2), v.px.cy - (v.px.h / 2), v.px.w, v.px.h), QColor(0, 255, 127), v.id);
		dragBox->setID(v.id);

		v.px = ScaleManager::instance().world_to_fov(v.px);

		v.pDragBox = dragBox;

		v.vision_obj_IDs.append(voID);

		if (scanAlongY) {
			v.start_point.wx = wx;
			v.start_point.wy = wy - scan_target_mm / 2;

			v.end_point.wx = wx;
			v.end_point.wy = wy + scan_target_mm / 2;
		}
		else {
			v.start_point.wx = wx - scan_target_mm / 2;
			v.start_point.wy = wy;

			v.end_point.wx = wx + scan_target_mm / 2;
			v.end_point.wy = wy;
		}
		v.start_point.wz = wz;
		v.end_point.wz = wz;

		/*ct::logger::debug("World: %d, %d, %d, %d", b.xmin, b.ymin, b.w, b.h);
		ct::logger::debug("FOV: %d, %d, %d, %d", v.px.xmin, v.px.ymin, v.px.w, v.px.h);
		ct::logger::debug("Start (mm): %f, %f, %f", v.start_point.wx, v.start_point.wy, v.start_point.wz);
		ct::logger::debug("End (mm): %f, %f, %f", v.end_point.wx, v.end_point.wy, v.end_point.wz);
		ct::logger::debug("Center (mm): %f, %f", wmm.x(), wmm.y());
		ct::logger::debug("New Center (mm): %f, %f", wcx_mm, wcy_mm);
		ct::logger::debug("Size (mm): %f, %f", w_mm, h_mm);*/

		dragBox->setID(v.id);
		v.pDragBox = dragBox;

		vo->getId();
		v.vision_obj_IDs.append(voID);
		_visionObject[voID].locked = true;
		_visionObject[voID].lineScanID = v.id;

		if (_visionObject[voID].pDragBox) {
			_visionObject[voID].pDragBox->setDragable(false);
			_visionObject[voID].pDragBox->lineScanID(v.id);
		}

		_lineScans.insert(v.id, v);
	}

	//add data for final stitch view
	auto center_mm = ScaleManager::instance().to_world_mm(rect.center());

	QLineScan v;
	v.id = slinescan_id;
	v.name = slinescan_id;
	v.created_by = "";
	v.type = ct::s_stitch_linescan;
	v.map_to_slinescan = "";
	if (scanAlongY) {
		v.start_point.wx = center_mm.x();
		v.start_point.wy = center_mm.y() - scan_target_mm / 2;
		v.end_point.wx = center_mm.x();
		v.end_point.wy = center_mm.y() + scan_target_mm / 2;
	}
	else {
		v.start_point.wx = center_mm.x() - scan_target_mm / 2;
		v.start_point.wy = center_mm.y();
		v.end_point.wx = center_mm.x() + scan_target_mm / 2;
		v.end_point.wy = center_mm.y();
	}
	v.start_point.wz = wz;
	v.end_point.wz = wz;

	auto wpx = getAbsoluteFOVCoordinates(QPointF(center_mm.x(), center_mm.y()));
	v.px.cx = ScaleManager::instance().fov_to_world(util::mm_to_px(center_mm.x(), h_scale));
	v.px.cy = ScaleManager::instance().fov_to_world(util::mm_to_px(center_mm.y(), h_scale));
	v.px.w = ScaleManager::instance().fov_to_world(util::mm_to_px(h_target_mm, h_scale));
	v.px.h = ScaleManager::instance().fov_to_world(util::mm_to_px(v_target_mm, h_scale));
	if (step_num == 1) {
		if (scanAlongY) v.px.w = ScaleManager::instance().fov_to_world(util::mm_to_px(step_cam_mm, h_scale));
		else v.px.h = ScaleManager::instance().fov_to_world(util::mm_to_px(step_cam_mm, v_scale));
	}
	v.px.compute_extremum();

	auto dragBox = drawLineScan(QRectF(v.px.cx - (v.px.w / 2), v.px.cy - (v.px.h / 2), v.px.w, v.px.h), QColor(0, 255, 127), v.id);
	dragBox->setID(v.id);

	v.px = ScaleManager::instance().world_to_fov(v.px);

	v.pDragBox = dragBox;

	v.vision_obj_IDs.append(voID);
	_lineScans.insert(v.id, v);


	_visionObject[voID].locked = false;
	_visionObject[voID].lineScanID = slinescan_id;

	if (_visionObject[voID].pDragBox) {
		_visionObject[voID].pDragBox->lineScanID(slinescan_id);
		_visionObject[voID].pDragBox->setDragable(true);
	}

	return;
}

void VisionApp::assignLineScansToGroupedOversizedVO(const GroupedOverSizedVO& gvo, int padding_wpx, int overlap_percentage) {
	ct::logger::info("Start assigning to oversized VO");

	/*
	* Condition to fulfill:
	* - To support padding
	* - To support minimum overlap percentage
	* - Able to center FOV when there's not a need for stitching
	*/

	//convert all into horizontal and vertical axis
	double h_scale = ScaleManager::instance().horizontal_um_per_px();
	double v_scale = ScaleManager::instance().vertical_um_per_px();

	//Add padding 
	auto rect = gvo.rect;
	rect.setBottomRight(QPointF(rect.bottomRight().x() + padding_wpx, rect.bottomRight().y() + padding_wpx));
	rect.setTopLeft(QPointF(rect.topLeft().x() - padding_wpx, rect.topLeft().y() - padding_wpx));
	auto br = ScaleManager::instance().to_world_mm(rect.bottomRight());
	auto fl = ScaleManager::instance().to_world_mm(rect.topLeft());

	//Get Target Size to fill up
	double h_target_mm = abs(br.x() - fl.x());
	double h_center_mm = (br.x() + fl.x()) / 2;
	double v_target_mm = abs(br.y() - fl.y());
	double v_center_mm = (br.y() + fl.y()) / 2;

	//scan axis: direction the gantry travels; step axis: direction the FOV strips stack
	const bool scanAlongY = SystemData::instance().isLineScanAxisY();
	double scan_target_mm = scanAlongY ? v_target_mm : h_target_mm;
	double scan_center_mm = scanAlongY ? v_center_mm : h_center_mm;
	double step_target_mm = scanAlongY ? h_target_mm : v_target_mm;
	double step_start_mm = scanAlongY ? fl.x() : fl.y();

	//Get FOV (laser FOV perpendicular to the scan direction)
	double step_cam_mm = 14.5;

	//Set minimum overlap percentage
	double step_percentage = (double)overlap_percentage / 100.0;
	double step_overlap_mm = step_cam_mm * step_percentage;

	/*
	* n: Minimum number of FOV
	* t: Target size in mm
	* o: Minimum overlap in mm
	* f: FOV size in mm
	*
	* To get minimum number of FOV
	* n = ceil((t - o)/(f - o))
	*
	* To get minimum overlap in mm
	* o = (n * f - t) / n - 1
	*/
	//Calculate minimum number of FOV required
	int step_num = std::ceil((step_target_mm - step_overlap_mm) / (step_cam_mm - step_overlap_mm));

	//Recalculate minimum overlap based on minimum number of FOV
	step_overlap_mm = ((step_num * step_cam_mm) - step_target_mm) / (step_num - 1);

	double step_centerOffset_mm = 0.0;
	if (step_num == 1) {
		step_overlap_mm = 0.0;
		step_centerOffset_mm = (step_target_mm / 2) - (step_cam_mm / 2);
	}

	//Calculate each iteration's offset
	double step_offset_mm = step_cam_mm - step_overlap_mm;

	ct::logger::trace("step_num: %d", step_num);

	ct::logger::trace("step_overlap: %f", step_overlap_mm);

	ct::logger::trace("step_offset: %f", step_offset_mm);

	ct::logger::trace("step_cam_mm: %f", step_cam_mm);

	ct::logger::trace("scan_target_mm: %f", scan_target_mm);
	ct::logger::trace("step_target_mm: %f", step_target_mm);

	ct::logger::trace("step_centerOffset_mm: %f", step_centerOffset_mm);

	uidGenerator sgen;
	auto slinescan_id = QString("slinescan") + sgen.id().c_str();

	auto wz = _plane.corner_points[(int)Corner::FRONTLEFT].wz;

	for (int r = 0; r < step_num; r++) {

		QLineScan v;
		v.id = slinescan_id + "-" + QString::number(r);
		v.name = QString("linescan_") + QString::number(_lineScans.size());
		v.created_by = "";
		v.type = ct::s_child_linescan;
		v.map_to_slinescan = slinescan_id;
		auto step_mm = step_start_mm + (step_cam_mm / 2) + (step_offset_mm * r) + step_centerOffset_mm;
		auto wx = scanAlongY ? step_mm : scan_center_mm;
		auto wy = scanAlongY ? scan_center_mm : step_mm;

		auto wpx = getAbsoluteFOVCoordinates(QPointF(wx, wy));
		v.px.cx = ScaleManager::instance().fov_to_world(util::mm_to_px(wx, h_scale));
		v.px.cy = ScaleManager::instance().fov_to_world(util::mm_to_px(wy, h_scale));
		v.px.w = ScaleManager::instance().fov_to_world(util::mm_to_px(scanAlongY ? step_cam_mm : scan_target_mm, h_scale));
		v.px.h = ScaleManager::instance().fov_to_world(util::mm_to_px(scanAlongY ? scan_target_mm : step_cam_mm, h_scale));
		v.px.compute_extremum();

		auto dragBox = drawLineScan(QRectF(v.px.cx - (v.px.w / 2), v.px.cy - (v.px.h / 2), v.px.w, v.px.h), QColor(0, 255, 127), v.id);
		dragBox->setID(v.id);

		v.px = ScaleManager::instance().world_to_fov(v.px);

		v.pDragBox = dragBox;

		if (scanAlongY) {
			v.start_point.wx = wx;
			v.start_point.wy = wy - scan_target_mm / 2;

			v.end_point.wx = wx;
			v.end_point.wy = wy + scan_target_mm / 2;
		}
		else {
			v.start_point.wx = wx - scan_target_mm / 2;
			v.start_point.wy = wy;

			v.end_point.wx = wx + scan_target_mm / 2;
			v.end_point.wy = wy;
		}
		v.start_point.wz = wz;
		v.end_point.wz = wz;

		dragBox->setID(v.id);
		v.pDragBox = dragBox;

		_lineScans.insert(v.id, v);
	}

	//add data for final stitch view
	auto center_mm = ScaleManager::instance().to_world_mm(rect.center());

	QLineScan v;
	v.id = slinescan_id;
	v.name = slinescan_id;
	v.created_by = "";
	v.type = ct::s_stitch_linescan;
	v.map_to_slinescan = "";
	if (scanAlongY) {
		v.start_point.wx = center_mm.x();
		v.start_point.wy = center_mm.y() - scan_target_mm / 2;
		v.end_point.wx = center_mm.x();
		v.end_point.wy = center_mm.y() + scan_target_mm / 2;
	}
	else {
		v.start_point.wx = center_mm.x() - scan_target_mm / 2;
		v.start_point.wy = center_mm.y();
		v.end_point.wx = center_mm.x() + scan_target_mm / 2;
		v.end_point.wy = center_mm.y();
	}
	v.start_point.wz = wz;
	v.end_point.wz = wz;

	auto wpx = getAbsoluteFOVCoordinates(QPointF(center_mm.x(), center_mm.y()));
	v.px.cx = ScaleManager::instance().fov_to_world(util::mm_to_px(center_mm.x(), h_scale));
	v.px.cy = ScaleManager::instance().fov_to_world(util::mm_to_px(center_mm.y(), h_scale));
	v.px.w = ScaleManager::instance().fov_to_world(util::mm_to_px(h_target_mm, h_scale));
	v.px.h = ScaleManager::instance().fov_to_world(util::mm_to_px(v_target_mm, h_scale));
	if (step_num == 1) {
		if (scanAlongY) v.px.w = ScaleManager::instance().fov_to_world(util::mm_to_px(step_cam_mm, h_scale));
		else v.px.h = ScaleManager::instance().fov_to_world(util::mm_to_px(step_cam_mm, v_scale));
	}
	v.px.compute_extremum();

	auto dragBox = drawLineScan(QRectF(v.px.cx - (v.px.w / 2), v.px.cy - (v.px.h / 2), v.px.w, v.px.h), QColor(0, 255, 127), v.id);
	dragBox->setID(v.id);

	v.px = ScaleManager::instance().world_to_fov(v.px);

	v.pDragBox = dragBox;

	for (auto vo : gvo.VOs) {
		auto voID = vo->getId();

		v.vision_obj_IDs.append(voID);

		_visionObject[voID].locked = true;
		_visionObject[voID].lineScanID = slinescan_id;
		if (_visionObject[voID].pDragBox) {
			_visionObject[voID].pDragBox->setDragable(true);
			_visionObject[voID].pDragBox->lineScanID(slinescan_id);
		}
	}
	
	_lineScans.insert(v.id, v);

	return;
}

void VisionApp::assignLineScans()
{
	ScopedTimeLogger t("Assign Line Scans");

	QVector<VisionAppQDragBox*> bigROIs;
	std::vector<ct::SmallBox> rois;
	std::vector<ct::ClusterBox> scans;

	auto svo = getSelectedVisionObject();

	if (svo.size() == 0) {
		showMsg("No vision object selected!");
		return;
	}

	double leftmost = 9999999.99;
	double rightmost = 0.0;
	double topmost = 9999999.99;
	double btmmost = 0.0;
	double scale = ScaleManager::instance().um_per_px();
	const QString& api = ProfilerManager::instance().getAPI();
	const bool& msr = ProfilerManager::instance().getMSR();
	double laser_fov_mm= 21.76;
	//const bool& connected = ProfilerManager::instance().getConnectionStatus();
	//QString sensorType = connected ? api : SystemData::instance().getLaserType();
	QString sensorType = SystemData::instance().getLaserType();
	if (sensorType == "Gocator") {
		ct::logger::info("[VisionAppCRUD] Using Gocator FOV");
		laser_fov_mm = 14.5;  
	}
	else if (sensorType == "SmartRay") {
		ct::logger::info("[VisionAppCRUD] Using SmartRay FOV");
		// Determine FOV based on the msr flag.
		if (msr) {
			ct::logger::info("[VisionAppCRUD] MSR enabled, using MSR FOV");
			laser_fov_mm = 11.277;
		}
		else {
			ct::logger::info("[VisionAppCRUD] MSR disabled, using default FOV");
			laser_fov_mm = 13.85;
		}
	}
	else if (sensorType == "SSZN") {
		ct::logger::info("[VisionAppCRUD] Using SSZN FOV");
		laser_fov_mm = 21.76;
	}
	else if (sensorType == "KeyenceLJ") {
		ct::logger::info("[VisionAppCRUD] Using KeyenceLJ FOV");
		//LJ-X8060: 3200 profile points at the default 5 um interval = 16.0 mm, which is also
		//the FAR-side X measurement range on the data sheet. Verify against Profiler_Keyence's
		//per-scan log "MEASURED LASER FOV = <mm> mm (<N> points @ <P> um)" - if the profile
		//data interval has been changed in Navigator this figure moves with it, and the
		//controller wins. Must be kept identical to the copy in VisionApp_JSON.cpp.
		laser_fov_mm = 16.0;
	}
	else {
		ct::logger::error("[VisionAppCRUD] Failed to get sensor type", sensorType.toStdString().c_str());
	}

	double fov_wpx = ScaleManager::instance().fov_to_world(util::mm_to_px(laser_fov_mm, scale));

	int padding_wpx = ScaleManager::instance().fov_to_world(util::mm_to_px(ui.lineEdit_lineScanPaddingMM->text().toDouble(), scale));

	const bool scanAlongY = SystemData::instance().isLineScanAxisY();
	ct::logger::info("[VisionAppCRUD] Line scan axis: %s", scanAlongY ? "Y" : "X");


	bool unassignOnly = false;
	if (ui.comboBox_lineScanAssignmentMethod->currentText() == "All Selected Region") clearLineScans();
	else if (ui.comboBox_lineScanAssignmentMethod->currentText() == "Unassigned Selected Region") unassignOnly = true;

	for (auto& vo : svo) {

		auto rect = vo->getGeometry();

		if (vo->lineScanID() != "" && unassignOnly) continue;

		//oversized when the VO exceeds the laser FOV perpendicular to the scan direction
		if ((scanAlongY ? rect.width() : rect.height()) > fov_wpx) {
			bigROIs.push_back(vo);
		}
		else {
			if (leftmost > vo->getGeometry().topLeft().x()) {
				leftmost = vo->getGeometry().topLeft().x();
			}
			if (rightmost < vo->getGeometry().bottomRight().x()) {
				rightmost = vo->getGeometry().bottomRight().x();
			}
			if (topmost > vo->getGeometry().topLeft().y()) {
				topmost = vo->getGeometry().topLeft().y();
			}
			if (btmmost < vo->getGeometry().bottomRight().y()) {
				btmmost = vo->getGeometry().bottomRight().y();
			}

			auto x = vo->getGeometry().x();
			auto y = vo->getGeometry().y();
			auto w = vo->getGeometry().width();
			auto h = vo->getGeometry().height();

			ct::SmallBox roi;
			roi.id = vo->getId().toStdString();
			roi.box.cx = x + w / 2;
			roi.box.cy = y + h / 2;
			roi.box.w = w;
			roi.box.h = h;

			roi.box.compute_extremum();

			rois.emplace_back(roi);
		}
	}

	auto w_wpx = abs(leftmost - rightmost);
	auto h_wpx = abs(topmost - btmmost);

	ct::logger::debug("W: %f, H: %f", w_wpx, h_wpx);
	ct::logger::debug("Top: %f, Left: %f", topmost, leftmost);
	ct::logger::debug("Btm: %f, Right: %f", btmmost, rightmost);

	int overlap_percentage = ui.lineEdit_lineScanOverlapPercentage->text().toInt();

	auto gvos = groupOverSizedVOForLineScan(bigROIs);

	for (auto& gvo : gvos) {
		assignLineScansToGroupedOversizedVO(gvo, padding_wpx, overlap_percentage);
	}

	/*for (auto& roi : bigROIs) {
		assignLineScansToOversizedVO(roi, padding_wpx, overlap_percentage);
	}*/

	//FOV of laser must be maintain, while scan start and end point can be padded
	if (scanAlongY) {
		ct::cluster_boxes(fov_wpx, h_wpx + padding_wpx * 2, padding_wpx, rois, scans, ct::ClusterPriority::X);
		ct::fit_boxes_height(scans, padding_wpx);
	}
	else {
		ct::cluster_boxes(w_wpx + padding_wpx * 2, fov_wpx, padding_wpx, rois, scans, ct::ClusterPriority::Y);
		ct::fit_boxes_width(scans, padding_wpx);
	}

	//world z 
	auto wz = _plane.corner_points[(int)Corner::FRONTLEFT].wz;

	ct::logger::debug("Linescan world z (mm): %f", wz);

	for (auto wpx : scans) {
		auto& b = wpx.box;

		QLineScan v;
		uidGenerator idGen;
		v.id = QString("linescan") + idGen.id().c_str();
		v.name = QString("linescan_") + QString::number(_lineScans.size());
		v.created_by = "";
		v.px = ScaleManager::instance().world_to_fov(wpx.box);

		auto dragBox = drawLineScan(QRectF(b.xmin, b.ymin, b.w, b.h), QColor(0, 255, 127), v.name);

		auto wmm = ScaleManager::instance().to_world_mm(QPointF(wpx.box.cx, wpx.box.cy));
		auto wcx_mm = util::px_to_mm(ScaleManager::instance().world_to_fov(wpx.box.cx), scale);
		auto wcy_mm = util::px_to_mm(ScaleManager::instance().world_to_fov(wpx.box.cy), scale);

		auto w_mm = util::px_to_mm(ScaleManager::instance().world_to_fov(b.w), scale);
		auto h_mm = util::px_to_mm(ScaleManager::instance().world_to_fov(b.h), scale);

		if (scanAlongY) {
			v.start_point.wx = wmm.x();
			v.start_point.wy = wmm.y() - h_mm / 2;
			v.start_point.wz = wz;

			v.end_point.wx = wmm.x();
			v.end_point.wy = wmm.y() + h_mm / 2;
			v.end_point.wz = wz;
		}
		else {
			v.start_point.wx = wmm.x() - w_mm / 2;
			v.start_point.wy = wmm.y();
			v.start_point.wz = wz;

			v.end_point.wx = wmm.x() + w_mm / 2;
			v.end_point.wy = wmm.y();
			v.end_point.wz = wz;
		}

		ct::logger::debug("World: %d, %d, %d, %d", b.xmin, b.ymin, b.w, b.h);
		ct::logger::debug("FOV: %d, %d, %d, %d", v.px.xmin, v.px.ymin, v.px.w, v.px.h);
		ct::logger::debug("Start (mm): %f, %f, %f", v.start_point.wx, v.start_point.wy, v.start_point.wz);
		ct::logger::debug("End (mm): %f, %f, %f", v.end_point.wx, v.end_point.wy, v.end_point.wz);
		ct::logger::debug("Center (mm): %f, %f", wmm.x(), wmm.y());
		ct::logger::debug("New Center (mm): %f, %f", wcx_mm, wcy_mm);
		ct::logger::debug("Size (mm): %f, %f", w_mm, h_mm);
		
		dragBox->setID(v.id);
		v.pDragBox = dragBox;

		for (auto box : wpx.boxes) {
			QString id = box.id.c_str();

			v.vision_obj_IDs.append(id);
			_visionObject[id].locked = true;
			_visionObject[id].lineScanID = v.id;

			if (_visionObject[id].pDragBox) {
				_visionObject[id].pDragBox->setDragable(false);
				_visionObject[id].pDragBox->lineScanID(v.id);
			}
		}

		_lineScans.insert(v.id, v);

		processEvents();
	}

	/*
	1. Generate line scan
	2. Get data
	3. Crop unit
	4. Use algoeditor to see accuracy
	5. Test rotation
	*/

	saveRecipe();

	return;
}

void VisionApp::generatePath()
{
	ScopedTimeLogger t("Generate Path");

	QString start_key = ui.tb_setStartPoint->whatsThis();
	QString end_key = ui.tb_setEndPoint->whatsThis();
	if (start_key == "" || end_key == "") return;

	em::TourGenerator tg;
	em::TSPAlgorithm algo = em::TSPAlgorithm::CHEAPEST_INSERTION;
	em::TSPDirectionPriority priority = em::TSPDirectionPriority::HORIZONTAL;

	if (ui.checkBox_uniformlyDistancedViews->isChecked()) {
		algo = em::TSPAlgorithm::NEAREST_NEIGHBOR;
	}

	if (ui.comboBox_prioritizeDirection->currentText() == "Vertical") {
		priority = em::TSPDirectionPriority::VERTICAL;
	}

	tg.set_direction_priority(priority);
	ct::logger::debug("1");
	//QHash<QString, int> idToIndexMap;
	//QHash<int, QString> indexToIDMap;
	//int index = 0;
	//for (int i = 0; i < ui.listWidget_viewSelection->count(); i++) {
	//	auto item = ui.listWidget_viewSelection->item(i);
	//	if (item->checkState() == Qt::Checked) {
	//		auto& view = _views[item->whatsThis()];
	//		idToIndexMap.insert(view.id, index);
	//		indexToIDMap.insert(index, view.id);
	//		tg.vertices().emplace_back(em::Vertex(index, view.px.cx, view.px.cy));
	//		index++;
	//	}
	//}

	QHash<QString, int> idToIndexMap;
	QHash<int, QString> indexToIDMap;
	int index = 0;
	for (auto view : _views) {
		if (view.type == ct::s_stitch_view) continue;

		idToIndexMap.insert(view.id, index);
		indexToIDMap.insert(index, view.id);
		tg.vertices().emplace_back(em::Vertex(index, view.px.cx, view.px.cy));
		index++;
	}
	ct::logger::debug("2");
	if (!idToIndexMap.contains(start_key) || !idToIndexMap.contains(end_key)) {
		printf("[UB] Path generator invalid start/end key.\n");
		return;
	}

	tg.set_start_point(idToIndexMap[start_key]); //set first view
	tg.set_end_point(idToIndexMap[end_key]);
	tg.compute_tour(algo);
	ct::logger::debug("3");
	auto& sequence = tg.sequence();

	clearPath();
	//add to preview list
	for (auto s : sequence) {
		auto& vertex = tg.vertices()[s];

		if (!_views.contains(indexToIDMap[vertex.index()])) {
			ct::logger::error("[IM] Failed to generate path. Invalid view ID: %s", indexToIDMap[vertex.index()].toStdString().c_str());
			continue;
		}

		auto& view = _views[indexToIDMap[vertex.index()]];
		addViewToPath(view.id);
	}
	ct::logger::debug("4");
	updateTreeViewExplorer(Common::Directory::CurrentRecipe, _views, _visionObject);
	ct::logger::debug("5");
	saveRecipe();
}

/* Read related */
void VisionApp::showAllGraphicItems(bool show)
{
	showView(show);
	showLineScans(show);
	showVisionObject(show);
	showPath(show);
}

void VisionApp::showCrossHair(bool show)
{
	ui.toolButton_showCrossHair->setChecked(show);

	if (_crossHairX) {
		if (show) _crossHairX->show();
		else _crossHairX->hide();
	}

	if (_crossHairY) {
		if (show)_crossHairY->show();
		else _crossHairY->hide();
	}
}

void VisionApp::showView(bool show)
{
	ui.toolButton_showView->setChecked(show);

	for (auto roi : _viewROI) {
		if (show) {
			roi->show();
			if (roi->text) roi->text->show();
		}
		else {
			roi->hide();
			if (roi->text) roi->text->hide();
		}
	}
}

void VisionApp::showLineScans(bool show)
{
	ui.toolButton_showLineScan->setChecked(show);

	for (auto roi : _lineScanROI) {
		if (show) {
			roi->show();
			if (roi->text) roi->text->show();
		}
		else {
			roi->hide();
			if (roi->text) roi->text->hide();
		}
	}
}

void VisionApp::showVisionObject(bool show)
{
	ui.toolButton_showVisionObject->setChecked(show);

	if (g_viewMode == int(ViewMode::SINGLE))
	{
		for (auto roi : _dragROI) {
			if (show && roi->viewID() == ui.label_curViewName->whatsThis())
			{
				roi->show();
				roi->setOutterBarrier(_sceneBound);
				roi->setOutterBarrier(_pGraphicsSceneMain->sceneRect());

			}
			else roi->hide();
		}
	}
	else
	{
		for (auto roi : _dragROI) {
			if (show) roi->show();
			else roi->hide();
		}
	}
	
}

void VisionApp::showDefectRect(bool show)
{
	if (g_viewMode == int(ViewMode::SINGLE))
	{
		for (auto defRect : _defectRectShape) {
			auto defID = defRect->data(0).toString();
			auto indexID = defRect->data(1).toString();
			auto viewID = defRect->data(2).toString();
			if (show && viewID == ui.label_curViewName->whatsThis() && indexID == ui.lineEdit_currentImageIndex->text())
			{
				defRect->show();
			}
			else defRect->hide();
		}
	}
	else
	{
		for (auto defRect : _defectRectShape) {
			if (show) defRect->show();
			else defRect->hide();
		}
	}
}

void VisionApp::showPath(bool show)
{
	ui.toolButton_showPath->setChecked(show);

	for (auto item : _pathGraphicItems) {
		if (show) item->show();
		else item->hide();
	}
}

/* Update related */


/* Delete related */
// Delete both data and UI
void VisionApp::clearVisionObject()
{
	for (int i = 0; i < _dragROI.count(); i++)
	{
		_dragROI.at(i)->deleteLater();
		_dragROI[i] = nullptr;
	}

	_dragROI.clear();
	_visionObject.clear();
}

void VisionApp::clearView()
{
	for (int i = 0; i < _viewROI.count(); i++)
	{
		if (_viewROI.at(i)->text) _viewROI.at(i)->text->deleteLater();
		_viewROI.at(i)->deleteLater();
		_viewROI[i] = nullptr;
	}

	_viewROI.clear();
	_views.clear();

	for (auto& vo : _visionObject) {
		vo.viewID = "";
		vo.pDragBox->viewID("");
	}
}

void VisionApp::clearLineScans()
{
	for (auto& p : _lineScanROI) {
		if (p->text) p->text->deleteLater();
		p->deleteLater();
		p = nullptr;
	}
	
	_lineScanROI.clear();
	_lineScans.clear();

	for (auto& vo : _visionObject) {
		vo.lineScanID = "";
		vo.pDragBox->lineScanID("");
	}
}

void VisionApp::clearPath()
{
	_lastViewAddedToPath = "";

	for (auto item : _pathGraphicItems) {
		if (item != nullptr) {
			_pGraphicsSceneMain->removeItem(item);
			delete item;
			item = nullptr;
		}
	}
	_pathGraphicItems.clear();
	ui.listWidget_paths->clear();

	for (auto view : _viewROI) {
		view->setBorderColor(getColor(Representation::UNASSIGNED_VIEW));
	}
}

void VisionApp::toggleCommonDragBox(QToolButton* btn)
{
	if (_commonDragBox.isVisible()) {
		_commonDragBox.hide();
		btn->setChecked(false);
	} else {
		_commonDragBox.show();
		btn->setChecked(true);
	}
}

