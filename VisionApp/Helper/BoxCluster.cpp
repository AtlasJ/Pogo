#include "BoxCluster.h"
#include <map>

#include "Utilities.h"
#include "ScopedTimeLogger.h"

void ct::cluster_boxes(int view_width, int view_height, int padding, std::vector<SmallBox>& rois, std::vector<ClusterBox>& views, bool prioritizeY)
{
	ScopedTimeLogger timer("Cluster ROI");

	//offset padding
	view_width -= padding;
	view_height -= padding;

	if (rois.size() == 0) return;

	//Sort roi from top left
	//assign distance from top left of every ROI
	std::vector<int> distances;
	for (auto& roi : rois) {
		roi.is_assigned = false;
		roi.assignedTo = -1;
		ct::Box2D topleft;
		topleft.cx = 0;
		topleft.cy = 0;
		if(prioritizeY) distances.emplace_back(int(ct::distancePrioritizeY(topleft, roi.box, view_width)));
		else distances.emplace_back(int(ct::distance(topleft, roi.box)));
	}

	std::vector<int> sorted_indexes;
	algo::bubbleSort(distances, sorted_indexes, true); //optimize: use merge sort

	auto assignROItoView = [](ClusterBox& view, SmallBox& roi, int& numAssigned) {
		roi.is_assigned = true;
		view.boxes.emplace_back(roi);
		numAssigned++;
	};

	auto get_xmin = [](const std::vector<SmallBox>& rois) -> int {
		if (rois.size() == 0) return -1;

		int value = rois[0].box.xmin;

		for (auto roi : rois) {
			if (value > roi.box.xmin) {
				value = roi.box.xmin;
			}
		}

		return value;
	};

	auto get_xmax = [](const std::vector<SmallBox>& rois) -> int {
		if (rois.size() == 0) return -1;

		int value = rois[0].box.xmax;

		for (auto roi : rois) {
			if (value < roi.box.xmax) {
				value = roi.box.xmax;
			}
		}

		return value;
	};

	auto get_ymin = [](const std::vector<SmallBox>& rois) -> int {
		if (rois.size() == 0) return -1;

		int value = rois[0].box.ymin;

		for (auto roi : rois) {
			if (value > roi.box.ymin) {
				value = roi.box.ymin;
			}
		}

		return value;
	};

	auto get_ymax = [](const std::vector<SmallBox>& rois) -> int {
		if (rois.size() == 0) return -1;

		int value = rois[0].box.ymax;

		for (auto roi : rois) {
			if (value < roi.box.ymax) {
				value = roi.box.ymax;
			}
		}

		return value;
	};

	auto adjustViewCenter = [=](ClusterBox& view) {
		view.box.cx = (get_xmin(view.boxes) + get_xmax(view.boxes)) / 2;
		view.box.cy = (get_ymin(view.boxes) + get_ymax(view.boxes)) / 2;
		view.box.compute_extremum();
	};

	int numAssigned = 0;
	while (numAssigned != rois.size()) { //keep loop until all roi is assigned
										 //allocate view
		ClusterBox v;
		v.id = "view_" + std::to_string(views.size());
		v.box.w = view_width;
		v.box.h = view_height;
		v.box.cx = 0 + v.box.w / 2;
		v.box.cy = 0 + v.box.h / 2;
		//v.box.compute_extremum(); not needed since it will be shift later

		for (int i = 0; i < rois.size(); i++) {
			auto si = sorted_indexes[i];
			if (rois[si].is_assigned) continue;

			if (v.boxes.size() == 0) { //if no roi assign, use first one as center
				v.box.cx = rois[si].box.cx;
				v.box.cy = rois[si].box.cy;
				v.box.compute_extremum();

				assignROItoView(v, rois[si], numAssigned);
				rois[si].assignedTo = views.size();
				continue;
			}

			//within range
			bool in_range = true;
			if (v.box.cx < rois[si].box.cx) {
				if (abs(get_xmin(v.boxes) - rois[si].box.xmax) > view_width) {
					in_range = false;
				}
			}
			else {
				if (abs(get_xmax(v.boxes) - rois[si].box.xmin) > view_width) {
					in_range = false;
				}
			}
			if (v.box.cy < rois[si].box.cy) {
				if (abs(get_ymin(v.boxes) - rois[si].box.ymax) > view_height) {
					in_range = false;
				}
			}
			else {
				if (abs(get_ymax(v.boxes) - rois[si].box.ymin) > view_height) {
					in_range = false;
				}
			}

			//if in range, center the view
			if (in_range) {
				assignROItoView(v, rois[si], numAssigned);
				rois[si].assignedTo = views.size();
				adjustViewCenter(v);
			}
		}

		views.emplace_back(v);
	}

	for (auto& v : views) {
		//assign back true view's w and h
		v.box.w += padding;
		v.box.h += padding;
		v.box.compute_extremum();
	}

	//check which roi is not inside a view
	//for (auto sr : srs) {
	//	bool isAssigned = false;
	//	for (auto v : views) {
	//		//if (sr.box.is_inside(v.box)) {
	//		if (is_in_box(v.box, sr.box)) {
	//			isAssigned = true;
	//			break;
	//		}
	//	}
	//	if (!isAssigned) {
	//		printf("Not in any view:\n");
	//		int i = sr.assignedTo;
	//		printf("Assigned [%d]:  %d, %d, %d, %d\n", i,
	//			views[i].box.xmin,
	//			views[i].box.ymin,
	//			views[i].box.xmax,
	//			views[i].box.ymax);
	//		printf("ROI [%d]:  %d, %d, %d, %d\n", i,
	//			sr.box.xmin,
	//			sr.box.ymin,
	//			sr.box.xmax,
	//			sr.box.ymax);
	//	}
	//}
}

void ct::fit_boxes_width(std::vector<ClusterBox>& gb, int padding)
{
	for (auto& b : gb)
	{
		//get the rect of box 2d and put it into QRectF, get QRectF of svo. then check if any svo fall in the QRectF of the box 2d and then store them into another vector
		b.box.compute_extremum();
		QRectF box(b.box.xmin, b.box.ymin, b.box.w, b.box.h);

		//calculate the most left and most right from the VO
		double leftmost = 9999999.99;
		double rightmost = 0.0;
		double topmost = 9999999.99;
		double btmmost = 0.0;
		for (auto vo_box : b.boxes)
		{
			QRect rect(vo_box.box.xmin, vo_box.box.ymin, vo_box.box.w, vo_box.box.h);
			if (leftmost > rect.topLeft().x()) {
				leftmost = rect.topLeft().x();
			}
			if (rightmost < rect.bottomRight().x()) {
				rightmost = rect.bottomRight().x();
			}
			if (topmost > rect.y()) {
				topmost = rect.y();
			}
			if (btmmost < rect.y()) {
				btmmost = rect.y();
			}
		}

		b.box.w = rightmost - leftmost + 2 * padding;
		double center = leftmost - padding + b.box.w / 2;
		b.box.cx = center;
		b.box.compute_extremum();

	}
}
