#include "VisionApp.h"
#include "ScaleManager.h"

void get_segment_details(const double& totalSize, const double& size, double& offset, int& num) {
	num = std::ceil(totalSize / size);
	double remainder = size * num - totalSize;
	double overlap = remainder / (num - 1);
	offset = size - overlap;
}

void VisionApp::initAnalysis()
{
	connect(ui.toolButton_analyseGridIntensity, &QToolButton::clicked, this, [=]() {
		auto expectedSize_um = ui.lineEdit_gridSegment_um->text().toDouble();
		if (expectedSize_um < 450) expectedSize_um = 450;
		expectedSize_um = 1000;
		expectedSize_um = 3000;

		auto expectedSize_px = ScaleManager::instance().um_to_px(expectedSize_um);

		ct::logger::info("Expected size: %.2f, %.2f", expectedSize_px, expectedSize_um);
		clearRenderMap("IntensityGrid");
		
		MIL_ID mBuf = mtrx::to_milID(_imageFOV); 
		MIL_ID mMono = mtrx::to_mono(mBuf);
		mtrx::BufferCollector bc_mBuf(mBuf);
		mtrx::BufferCollector bc_mMono(mMono);

		MIL_UINT8 *hostPtr = M_NULL;
		MIL_ID pitch = M_NULL;
		MbufInquire(mMono, M_HOST_ADDRESS, &hostPtr);
		MbufInquire(mMono, M_PITCH, &pitch);

		auto w = mtrx::get_width(mMono);
		auto h = mtrx::get_height(mMono);

		double h_offset, v_offset;
		int h_num, v_num;

		get_segment_details(w, expectedSize_px, h_offset, h_num);
		get_segment_details(h, expectedSize_px, v_offset, v_num);
		ct::logger::info("Offset(h,v): %.2f, %.2f", h_offset, v_offset);
		ct::logger::info("Num(h,v): %d, %d", h_num, v_num);

		QHash<QString, double> avgMap;

		auto offset = expectedSize_px * 0.25;
		auto fontSize = expectedSize_px / 8;


		QImage qimg = _imageFOV;
		QPainter painter(&qimg);

		QPen pen;
		pen.setWidth(3);
		pen.setColor(QColor(0, 255, 127));
		painter.setPen(pen);
		QBrush brush;
		brush.setColor(QColor(0, 255, 127));
		painter.setBrush(brush);
		QFont font = painter.font();
		font.setPointSize(fontSize);
		painter.setFont(font);

		ct::logger::info("x1");
		double min = 9999999;
		double max = 0.0;
		double range = 0.0;
		double totalAvg = 0.0;

		//auto c1 = ui.lineEdit_ch1->text();
		//auto c2 = ui.lineEdit_ch2->text();
		//auto c3 = ui.lineEdit_ch3->text();
		QString filename = "gridAnalysis.txt";
		QString imgPath = "gridAnalysis.png";
		QString oimgPath = "ogridAnalysis.png";
		ct::logger::info("x2");
		std::ofstream fout(filename.toStdString());
		int totalCount = 0;

		for (int x_idx = 0; x_idx < h_num; x_idx++) {
			for (int y_idx = 0; y_idx < v_num; y_idx++) {

				//skip edge grids
				if (x_idx == 0 || y_idx == 0 || x_idx == h_num - 1 || y_idx == v_num - 1) continue;

				auto start_x = x_idx * h_offset;
				auto start_y = y_idx * v_offset;
				auto end_x = start_x + expectedSize_px;
				auto end_y = start_y + expectedSize_px;

				QString key = "R" + QString::number(y_idx) + "C" + QString::number(x_idx);

				double avg = 0.0;
				int count = 0;
				//ct::logger::info("xy: %d, %d", x_idx, y_idx);
				for (int x = start_x; x < end_x; x++) {
					for (int y = start_y; y < end_y; y++) {

						if (x >= w || y >= h || x < 0 || y < 0) continue;

						avg += hostPtr[x + (y * pitch)];
						count++;
					}
				}
				//ct::logger::info(">>xy: %d, %d", x_idx, y_idx);
				avg = avg / count;
				
				fout << key.toStdString() << ", " << avg << std::endl;

				if (min > avg) min = avg;
				if (max < avg) max = avg;
				totalAvg += avg;
				totalCount++;

				avgMap.insert(key, avg);


				auto rect = QRectF(start_x, start_y, expectedSize_px, expectedSize_px);
				auto textPos = QPointF(start_x + offset, start_y + offset);
				auto text = QString::number(avg, 'f', 2);

				drawRect(_pGraphicsSceneFOV, "IntensityGrid", rect);
				drawText(_pGraphicsSceneFOV, "IntensityGrid", text, textPos, QColor(0, 255, 127), fontSize);


				painter.fillRect(rect, painter.brush());
				painter.drawRect(rect);
				painter.drawText(textPos, text);
			}
		}
		
		ct::logger::info("x3");
		range = max - min;
		totalAvg /= totalCount;

		ui.textEdit_utility->clear();
		ui.textEdit_utility->append(QStringLiteral("Min: %1").arg(min));
		ui.textEdit_utility->append(QStringLiteral("Max: %1").arg(max));
		ui.textEdit_utility->append(QStringLiteral("Range: %1").arg(range));
		ui.textEdit_utility->append(QStringLiteral("Average: %1").arg(totalAvg));

		fout << "Min, " << min << std::endl;
		fout << "Max, " << max << std::endl;
		fout << "Range, " << range << std::endl;
		fout << "Average, " << totalAvg << std::endl;

		// Also emit a numeric grid matrix that the auto-cal report reader consumes
		// (C:/Advanced/Data/3DCalReadings/GridIntensity*.csv). Only the analysed
		// (non-edge) cells are written, so the report's recomputed Min/Max/Range/Average
		// match the values shown here. Cells are full precision; the report rounds them
		// for display but keeps precision for the stats.
		const int rows = v_num - 2;
		const int cols = h_num - 2;
		if (rows > 0 && cols > 0) {
			const QString calDataDir = QStringLiteral("C:/Advanced/Data/3DCalReadings");
			QDir().mkpath(calDataDir);
			QFile csvFile(calDataDir + "/GridIntensity.csv");
			if (csvFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
				QTextStream csvOut(&csvFile);
				for (int y_idx = 1; y_idx <= v_num - 2; y_idx++) {
					QStringList rowVals;
					for (int x_idx = 1; x_idx <= h_num - 2; x_idx++) {
						QString key = "R" + QString::number(y_idx) + "C" + QString::number(x_idx);
						rowVals << QString::number(avgMap.value(key, 0.0), 'g', 12);
					}
					csvOut << rowVals.join(',') << '\n';
				}
				csvFile.close();
			}
			else {
				ct::logger::error("[GridAnalysis] Failed to write %s", (calDataDir + "/GridIntensity.csv").toStdString().c_str());
			}
		}

		painter.end();
		qimg.save(imgPath);
		_imageFOV.save(oimgPath);
	});

	connect(ui.toolButton_clearGridRenders, &QToolButton::clicked, this, [=]() {
		clearRenderMap("IntensityGrid");
	});
}