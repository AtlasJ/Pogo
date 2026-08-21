#pragma once
#include "Box2D.h"
#include <vector>
#include <QVector>

namespace ct {
	struct SmallBox {
		std::string id = "";
		ct::Box2D box;
		bool is_assigned = false;
		int assignedTo = 0;
	};

	struct ClusterBox {
		std::string id = "";
		ct::Box2D box;
		std::vector<SmallBox> boxes;
	};

	//ordering used when seeding clusters: NONE = plain distance from top-left,
	//Y = row-major (horizontal bands, X-axis line scan), X = column-major (vertical strips, Y-axis line scan)
	enum class ClusterPriority { NONE, Y, X };

	void cluster_boxes(int group_width, int group_height, int padding, std::vector<SmallBox>& sb, std::vector<ClusterBox>& gb, ClusterPriority priority = ClusterPriority::NONE);
	void fit_boxes_width(std::vector<ClusterBox>& gb, int padding);
	void fit_boxes_height(std::vector<ClusterBox>& gb, int padding);
}


