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

	void cluster_boxes(int group_width, int group_height, int padding, std::vector<SmallBox>& sb, std::vector<ClusterBox>& gb, bool prioritizeY = false);
	void fit_boxes_width(std::vector<ClusterBox>& gb, int padding);
}


