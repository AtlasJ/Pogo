#include "VisionApp.h"
#include "AuditLog.h"
#include <queue>
#include <set>

QString VisionApp::getIndexNaming(int currentIndex) {
	//auto name = ui.lineEdit_n->text();
	//auto start_index = ui.lineEdit_startingIndex->text().toInt();
	//name.replace("?", QString::number(currentIndex));

	//QRegExp regex("\\d*");  // a digit (\d), zero or more times (*)
	//std::vector<QString> nums;
	//std::vector<QString> ops;
	//std::vector<QString> test;

	////123+34*12*12+4234
	//QString n = "";

	//for (auto chr : name)
	//{
	//	if (chr.isDigit()) {
	//		n += chr;
	//	}
	//	else {
	//		if (regex.exactMatch(n)) {
	//			nums.emplace_back(n);
	//			n = "";
	//		}

	//		ops.emplace_back(chr);
	//	}
	//}

	//if (regex.exactMatch(n)) {
	//	nums.emplace_back(n);
	//	n = "";
	//}

	//for (auto num : nums) {
	//	printf("%s,", num.toStdString().c_str());
	//}printf("\n");

	//for (auto op : ops) {
	//	printf("%s,", op.toStdString().c_str());
	//}printf("\n");

	//QSet<int> unique_ops;
	//QSet<int> unique_nums;
	//std::queue<QString> seq;
	//for (int i = 0; i < ops.size(); i++) {
	//	if (nums.size() <= (i + 1)) {
	//		return "INVALID";
	//	}

	//	if (ops[i] == "*") {
	//		if (!unique_nums.contains(i)) {
	//			unique_nums.insert(i);
	//			seq.emplace(nums[i]);
	//		}

	//		unique_ops.insert(i);
	//		seq.emplace(ops[i]);
	//		
	//		if (!unique_nums.contains(i+1)) {
	//			unique_nums.insert(i+1);
	//			seq.emplace(nums[i+1]);
	//		}
	//	}
	//}

	//for (int i = 0; i < ops.size(); i++) {
	//	if (nums.size() <= (i + 1)) {
	//		return "INVALID";
	//	}

	//	if (!unique_nums.contains(i)) {
	//		unique_nums.insert(i);
	//		seq.emplace(nums[i]);
	//	}

	//	unique_ops.insert(i);
	//	seq.emplace(ops[i]);

	//	if (!unique_nums.contains(i + 1)) {
	//		unique_nums.insert(i + 1);
	//		seq.emplace(nums[i + 1]);
	//	}
	//}

	//while (!seq.empty()) {
	//	printf("%s", seq.front().toStdString().c_str());
	//	seq.pop();
	//}
	//printf("\n");

	auto& nc = _namingConvention;
	return nc.prefix + currentIndex + nc.postfix;
}

QString VisionApp::getRowColumnNaming(QString island, int row, int col) {
	auto name = ui.lineEdit_rowColumnRepresentation->text();
	name.replace("I?", island);
	name.replace("R?", QString::number(row));
	name.replace("C?", QString::number(col));

	auto& nc = _namingConvention;
	return nc.prefix + name + nc.postfix;
}

void VisionApp::updateNamingPreview()
{
	QString preview = "";
	auto& nc = _namingConvention;
	auto method = ui.comboBox_namingMethod->currentText();
	auto row_start_index = ui.lineEdit_rowStartingIndex->text().toInt();
	auto col_start_index = ui.lineEdit_colStartingIndex->text().toInt();
	auto island = ui.lineEdit_island->text();

	if (method == "Indexing") {
		//preview = nc.prefix + QString::number(start_index) + nc.postfix;
	}
	else if (method == "Row Column") {
		preview = getRowColumnNaming(island, row_start_index, col_start_index);
	}

	ui.label_namingPreview->setText("Preview: " + preview);
}

void VisionApp::allowOnlyIslandNamingConvention()
{
	ui.stackedWidget_namingConvention->setCurrentIndex(2);
	ui.frame_voNamingSetting->hide();
	ui.comboBox_namingMethod->setEnabled(false);
	ui.toolButton_convertBotNamingConvention->hide();
	ui.toolButton_convertToIndexing->hide();
	ui.label_recipeFacing->hide();
	ui.label_159->hide();

}

void VisionApp::initNamingConvention()
{
	connect(ui.lineEdit_namingPrefix, &QLineEdit::textEdited, this, [=](const QString& text) {
		_namingConvention.prefix = text;
		removeWhitespace(_namingConvention.prefix);
		updateNamingPreview();
		});

	connect(ui.lineEdit_namingPostfix, &QLineEdit::textEdited, this, [=](const QString& text) {
		_namingConvention.postfix = text;
		removeWhitespace(_namingConvention.postfix);
		updateNamingPreview();
		});

	connect(ui.lineEdit_rowColumnRepresentation, &QLineEdit::textEdited, this, [=](const QString& text) { updateNamingPreview(); });
	connect(ui.lineEdit_rowStartingIndex, &QLineEdit::textEdited, this, [=](const QString& text) { updateNamingPreview(); });
	connect(ui.lineEdit_colStartingIndex, &QLineEdit::textEdited, this, [=](const QString& text) { updateNamingPreview(); });

	connect(ui.comboBox_namingMethod, QOverload<const QString&>::of(&QComboBox::currentTextChanged), [=](const QString& text) {
		ui.frame_voNamingSetting->show();
		ui.toolButton_convertBotNamingConvention->show();

		if (text == "Indexing") ui.stackedWidget_namingConvention->setCurrentIndex(0);
		else if (text == "Row Column") ui.stackedWidget_namingConvention->setCurrentIndex(1);
		else if (text == "Island Unit")
		{
			ui.stackedWidget_namingConvention->setCurrentIndex(2);
			ui.frame_voNamingSetting->hide();
			ui.spinBox_voSerialNumber->setValue(1);
			ui.toolButton_convertBotNamingConvention->hide();
		}
		updateNamingPreview();
		});

	connect(ui.toolButton_updateColumn, &QToolButton::clicked, this, [=]() { ui.lineEdit_column->setText(QString::number(_utilityInfo.num)); });
	connect(ui.toolButton_updateRow, &QToolButton::clicked, this, [=]() { ui.lineEdit_row->setText(QString::number(_utilityInfo.num)); });
	connect(ui.toolButton_updateColumnPitch, &QToolButton::clicked, this, [=]() { ui.lineEdit_columnPitch->setText(QString::number(_utilityInfo.distance)); });
	connect(ui.toolButton_updateRowPitch, &QToolButton::clicked, this, [=]() { ui.lineEdit_rowPitch->setText(QString::number(_utilityInfo.distance)); });
	connect(ui.toolButton_updateRotation, &QToolButton::clicked, this, [=]() { ui.lineEdit_rotation->setText(QString::number(_utilityInfo.angle)); });

	connect(ui.toolButton_assignToRight, &QToolButton::toggled, [=](bool state) {
		QSignalBlocker sb1(ui.toolButton_assignToRight);
		QSignalBlocker sb2(ui.toolButton_assignToLeft);
		if (state) ui.toolButton_assignToLeft->setChecked(false);
		else ui.toolButton_assignToLeft->setChecked(true);
		});
	connect(ui.toolButton_assignToLeft, &QToolButton::toggled, [=](bool state) {
		QSignalBlocker sb1(ui.toolButton_assignToRight);
		QSignalBlocker sb2(ui.toolButton_assignToLeft);
		if (state) ui.toolButton_assignToRight->setChecked(false);
		else ui.toolButton_assignToRight->setChecked(true);
		});
	connect(ui.toolButton_assignUpwards, &QToolButton::toggled, [=](bool state) {
		QSignalBlocker sb1(ui.toolButton_assignUpwards);
		QSignalBlocker sb2(ui.toolButton_assignDownwards);
		if (state) ui.toolButton_assignDownwards->setChecked(false);
		else ui.toolButton_assignDownwards->setChecked(true);
		});
	connect(ui.toolButton_assignDownwards, &QToolButton::toggled, [=](bool state) {
		QSignalBlocker sb1(ui.toolButton_assignUpwards);
		QSignalBlocker sb2(ui.toolButton_assignDownwards);
		if (state) ui.toolButton_assignUpwards->setChecked(false);
		else ui.toolButton_assignUpwards->setChecked(true);
		});

	connect(ui.toolButton_applyNamingConvention, &QToolButton::clicked, this, [=]() {
		AuditLog::instance().log(QStringLiteral("NAMING_CONVENTION_APPLY"), ui.comboBox_namingMethod->currentText());

		auto method = ui.comboBox_namingMethod->currentText();
		int row_start_index = ui.lineEdit_rowStartingIndex->text().toInt();
		int col_start_index = ui.lineEdit_colStartingIndex->text().toInt();

		/*if (method == "Indexing") {
			auto increment = ui.lineEdit_namingIncrement->text().toInt();
			auto currentIndex = start_index;

			for (auto& vo : _visionObject) {
				if (vo.pDragBox->isSelected()) {
					auto& nc = _namingConvention;
					vo.objectName = nc.prefix + QString::number(currentIndex) + nc.postfix;
					vo.pDragBox->setName(vo.objectName);
					currentIndex += increment;
				}
			}
		}*/
		if (method == "Island Unit")
		{
			int serialNumber = ui.spinBox_voSerialNumber->value();
			bool isAssign = false;
			for (auto& vo : _visionObject) {
				if (vo.pDragBox->isSelected()) {
					isAssign = true;
					vo.row = 1;
					vo.col = serialNumber;
					vo.row_id = 1;
					vo.col_id = serialNumber;
					vo.island = QString::number(serialNumber);
					vo.island_id = serialNumber;
				}
			}
			if (isAssign)
			{
				serialNumber++;
				ui.spinBox_voSerialNumber->setValue(serialNumber);
			}

		}
		else if (method == "Row Column") {


			auto col = ui.lineEdit_column->text().toInt();
			auto row = ui.lineEdit_row->text().toInt();
			auto colPitch = ui.lineEdit_columnPitch->text().toDouble();
			auto rowPitch = ui.lineEdit_rowPitch->text().toDouble();
			auto rotation = ui.lineEdit_rotation->text().toDouble();
			auto island = ui.lineEdit_island->text();

			//get selected vision object
			std::vector<int> distances;
			std::vector<QVisionObject*> selectedVO;
			bool hasSelectedVo = false;
			for (auto& vo : _visionObject) {
				if (vo.pDragBox->isSelected()) {
					int cx = vo.pDragBox->getGeometry().center().x();
					int cy = vo.pDragBox->getGeometry().center().y();

					distances.emplace_back(int(em::distance(0, 0, cx, cy)));
					selectedVO.push_back(&vo);
					hasSelectedVo = true;
				}
			}

			if (!hasSelectedVo)
			{
				QMessageBox::warning(this, tr("No Unit Selected!"),
					"Please select Units for naming");
				return;
			}

			loadingBarSetup(QString("Assigning Unit Object Naming..."));
			std::vector<int> sorted_indexes;
			algo::bubbleSort(distances, sorted_indexes, true); //optimize: use merge sort

			//get top most left vision object
			std::set<QString> assignedRC;
			if (sorted_indexes.size()) {
				auto topleft_vo = selectedVO[sorted_indexes[0]];
				auto topleft_cx = topleft_vo->pDragBox->getGeometry().center().x();
				auto topleft_cy = topleft_vo->pDragBox->getGeometry().center().y();
				auto width = topleft_vo->pDragBox->getGeometry().width();
				auto height = topleft_vo->pDragBox->getGeometry().height();

				auto R = em::compute_2DRotationMatrix(em::to_radian(rotation));
				em::V2d O(topleft_cx, topleft_cy);

				//populate expected row col
				std::set<QString> expectedRC;
				for (int r = 0 + row_start_index; r < row + row_start_index; r++) {
					for (int c = 0 + col_start_index; c < col + col_start_index; c++) {
						QString name = QString("%1_%2").arg(r).arg(c);
						expectedRC.insert(name);

						auto cx = topleft_cx + ((c - col_start_index) * colPitch);
						auto cy = topleft_cy + ((r - row_start_index) * rowPitch);

						if (_debug) drawRect(QRectF(cx - width / 2, cy - height / 2, width, height));
					}
				}

				//assign naming
				for (auto& svo : selectedVO) {

					em::V2d V(svo->pDragBox->getGeometry().center().x(), svo->pDragBox->getGeometry().center().y());
					V = V - O;
					V = R * V;
					V = V + O;

					if (_debug) drawCross(QRectF(V.x(), V.y(), 30, 30));

					double col_center = (V.x() - topleft_cx) / colPitch + col_start_index;
					double row_center = (V.y() - topleft_cy) / rowPitch + row_start_index;

					int trunc_col = std::trunc(col_center);
					int trunc_row = std::trunc(row_center);

					double col_fraction = col_center - trunc_col;
					double row_fraction = row_center - trunc_row;

					int col_index = 0;
					int row_index = 0;
					//if (col_fraction <= 0.25) col_index = trunc_col - 1;
					if (col_fraction >= 0.75) col_index = trunc_col + 1;
					else col_index = trunc_col;

					//if (row_fraction <= 0.25) row_index = trunc_row - 1;
					if (row_fraction >= 0.75) row_index = trunc_row + 1;
					else row_index = trunc_row;


					auto name = getRowColumnNaming(island, row_index, col_index);
					svo->pDragBox->setName(name);
					svo->objectName = name;
					if (name.contains("I") && name.contains("R") && name.contains("C"))
					{
						svo->island = island;
						svo->row = row_index;
						svo->col = col_index;
						svo->row_id = row_index - row_start_index;
						svo->col_id = col_index - col_start_index;
					}

					name = QString("%1_%2").arg(row_index).arg(col_index);
					assignedRC.insert(name);
				}

				//get row col that are not assigned
				for (auto& a : assignedRC) {
					if (expectedRC.find(a) != expectedRC.end()) {
						expectedRC.erase(a);
					}
				}

				//handle unassigned vision object
				for (auto e : expectedRC) {
					auto list = e.split("_");
					int row_index = list.at(0).toInt();
					int col_index = list.at(1).toInt();

					double cx = (col_index - col_start_index) * colPitch + topleft_cx;
					double cy = (row_index - row_start_index) * rowPitch + topleft_cy;

					em::V2d V(cx, cy);
					V = V - O;
					V = R.inverse() * V;
					V = V + O;

					double x = V.x() - (width / 2);
					double y = V.y() - (height / 2);
					ct::logger::debug("Unpopulated[%s]: %f, %f", e.toStdString().c_str(), V.x(), V.y());

					auto id = addVisionObject(QRectF(x, y, width, height));

					if (id != "") {
						auto name = getRowColumnNaming(island, row_index, col_index);
						_visionObject[id].pDragBox->setName(name);
						_visionObject[id].objectName = name;

						if (name.contains("I") && name.contains("R") && name.contains("C"))
						{
							_visionObject[id].island = island;
							_visionObject[id].row = row_index;
							_visionObject[id].col = col_index;
							_visionObject[id].row_id = row_index - row_start_index;
							_visionObject[id].col_id = col_index - col_start_index;
						}

						selectedVO.push_back(&_visionObject[id]);
					}
					else {
						ct::logger::debug("Failed to add vision object");
					}
				}
			}


			//Flip logic
			auto assignToLeft = ui.toolButton_assignToLeft->isChecked();
			auto assignUpwards = ui.toolButton_assignUpwards->isChecked();

			if (assignToLeft || assignUpwards) {
				for (auto& svo : selectedVO) {
					if (assignUpwards) svo->row = row - 1 - svo->row + row_start_index * 2;
					if (assignToLeft) svo->col = col - 1 - svo->col + col_start_index * 2;

					auto name = getRowColumnNaming(island, svo->row, svo->col);
					svo->pDragBox->setName(name);
					svo->objectName = name;
				}
			}

			//=>assign island ID
			//get unique islands
			QHash<QString, QVector<QVisionObject*>> uniqueIsland;
			for (auto& vo : _visionObject) {
				if (uniqueIsland.contains(vo.island)) {
					uniqueIsland[vo.island].append(&vo);
				}
				else {
					uniqueIsland.insert(vo.island, QVector<QVisionObject*>());
					uniqueIsland[vo.island].append(&vo);
				}
			}

			//get centroid of islands
			QHash<QString, QPointF> islandCentroids;
			for (const auto& vos : uniqueIsland) {

				QPointF centroid;
				QString island;

				for (const auto& vo : vos) {
					island = vo->island;
					centroid.rx() += vo->pDragBox->getGeometry().center().x();
					centroid.ry() += vo->pDragBox->getGeometry().center().y();
				}

				auto size = vos.size();
				centroid.rx() /= size;
				centroid.ry() /= size;

				islandCentroids.insert(island, centroid);
			}

			//assign id from left to right of centroid
			QVector<std::pair<QString, QPointF>> sortedIslands;
			for (const auto& key : islandCentroids.keys()) {
				sortedIslands.append({ key, islandCentroids.value(key) });
			}

			std::sort(sortedIslands.begin(), sortedIslands.end(), [](const auto& a, const auto& b) {
				return a.second.x() < b.second.x();
				});

			//assign id to island
			int index = 0;
			for (const auto& s : sortedIslands) {
				for (auto& vo : uniqueIsland[s.first]) {
					vo->island_id = index;
				}
				index++;
			}


			refreshDragBoxSequence();
			updateTreeViewExplorer(Common::Directory::CurrentRecipe, _views, _visionObject);

			// save all info
			_islandInfo.prefix = ui.lineEdit_namingPrefix->text();
			_islandInfo.postfix = ui.lineEdit_namingPostfix->text();
			_islandInfo.rowStartingIndex = ui.lineEdit_rowStartingIndex->text().toInt();
			_islandInfo.colStartingIndex = ui.lineEdit_colStartingIndex->text().toInt();

			_islandInfo.totalRow = ui.lineEdit_row->text().toInt();
			_islandInfo.rowPitch = ui.lineEdit_rowPitch->text().toDouble();

			_islandInfo.totalCol = ui.lineEdit_column->text().toInt();
			_islandInfo.colPitch = ui.lineEdit_columnPitch->text().toDouble();

			_islandInfo.rotation = ui.lineEdit_rotation->text().toDouble();
			_islandInfo.totalIsland = ui.lineEdit_totalIsland->text().toInt();


			//showMsg("Object name updated!");



			loadingBarRelease();
			return;
		}



		updateTreeViewExplorer(Common::Directory::CurrentRecipe, _views, _visionObject);
		showMsg("Object name updated!");

		});

	connect(ui.toolButton_convertBotNamingConvention, &QToolButton::clicked, [=]() {
		int totalRow = _islandInfo.totalRow;

		for (auto& vo : _visionObject)
		{
			int oriRowID = vo.row_id;
			int oriRowName = vo.row;
			vo.row_id = totalRow - 1 - oriRowID;
			vo.row = totalRow - 1 - oriRowID + _islandInfo.rowStartingIndex;

			QString voName = "I" + vo.island + "R" + QString::number(vo.row) + "C" + QString::number(vo.col);
			vo.objectName = voName;
			vo.pDragBox->setName(voName);


		}
		QMessageBox::information(this, ("Success"),
			"Rows successfully inverted!");
		});

	connect(ui.toolButton_convertToIndexing, &QToolButton::clicked, [=]() {
		auto col = ui.lineEdit_column->text().toInt();
		auto row = ui.lineEdit_row->text().toInt();

		auto totalIsland = ui.lineEdit_totalIsland->text();

		int totalCols = totalIsland.toInt() * col;
		for (auto& svo : _visionObject)
		{
			// Compute the absolute column index across islands
			int absoluteCol = (svo.island_id * col) + svo.col_id;

			// Compute final linear index (left-to-right, top-to-bottom)
			int indexingName = ((svo.row_id * totalCols) + absoluteCol)+1;

			svo.objectName = QString::number(indexingName); // Assign to your QDragBox
		}


		QMessageBox::information(this, ("Success"),
			"Indexing Applied!");
		});


	//ui.comboBox_namingMethod->setCurrentText("Island Unit");
	//ui.stackedWidget_namingConvention->setCurrentIndex(2);
	//ui.frame_voNamingSetting->hide();
	//ui.spinBox_voSerialNumber->setValue(1);
	//ui.toolButton_convertBotNamingConvention->hide();
}

/* First row col assignment algo
1. Get largest y
2. Loop through all to calculate key for map
key formula: y * max_x + x, where x = col, y = row
3. Add into map<int, *>
4. Since ordered map is already sorted, there's no need to sort in ascending anymore
5. Assign row and col accordingly
6. Row is assign by division of max y, while col is just an increment


//1.
int max_x = 0;
for (auto roi : _dragROI) {
	if (roi->x() > max_x && roi->isSelected()) {
		max_x = roi->x();
	}

}

//2, 3, 4
std::map<int, QDragBox*> roi_map;
for (auto& roi : _dragROI) {
	if (!roi->isSelected()) continue;

	int key = roi->y() * max_x + roi->x();

	if (roi_map.find(key) != roi_map.end()) {
		//printf("%d, %d == %d, %d\n", roi_map[key]->x(), roi_map[key]->y(), roi->x(), roi->y());
	}
	else {
		roi_map.insert({ key, roi });
	}
}

//5, 6
int index = 0;
int start_index = 1;
int col_index = start_index;
int row_index = start_index;

auto roi1 = *roi_map.begin();
int allowable_range = roi1.second->x() / 2;
double prev_div = (roi1.first - roi1.second->y()) / max_x;

for (auto& roi : roi_map) {
	double div = (roi.first - roi.second->y()) / max_x; //formula to adjust according to start index
	printf("Key: %d, Div: %f, Prev Div: %f -> %f, %f | Index: %d, %d\n", roi.first, div, prev_div, roi.second->x(), roi.second->y(), row_index, col_index);

	if (div - prev_div > allowable_range) {
		prev_div = div;
		row_index++;
		col_index = start_index;
	}

	roi.second->setName(getRowColumnNaming(row_index, col_index));
	col_index++;
	index++;
}

for (auto roi : roi_map) {
	if (_visionObject.contains(roi.second->getId())) {
		_visionObject[roi.second->getId()].objectName = roi.second->getName();
	}
}

*/