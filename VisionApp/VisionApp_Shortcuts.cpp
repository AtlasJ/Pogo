//#include <opencv2/opencv.hpp> //NOTE: reorder this to the first include will solve ambiguos symbol error
#include "VisionApp.h"
#include <iostream>
#include "Utilities.h"
#include "mtrx.h"
#include "BoxCluster.h"
#include "TimeLogger.h"
#include "EM_TSP.h"
#include "QClient.h"
#include <sstream>
#include <algorithm>
#include "QImageGrabber.h"
#include "AuditLog.h"
#include "VisionAppQDragBox.h"
#include "CommonDir.h"
#include "ImagePathManager.h"
#include "Gocator\Include\GoSdk\GoSdk.h"
#include <csignal>
#include "ImageRegistration.h"
#include "cvUtil.h"
#include <opencv2/opencv.hpp>
#include <opencv2/stitching.hpp>
#include "CAMManager.h"
#include "focus_stacking.h"
#include "ProfilerManager.h"
#include "ScopedFlag.h"
#include "ScaleManager.h"
#include <QInputDialog>
#include <vips/vips8>
#include "libvips/VImage8.h"
#include "Depth_from_focus.h"
#include "heightmap/height_map_wrapper.h"
#include "nvsutil/nvs_math.h"

using namespace vips;

HANDLE hEvent;

bool compareRectanglesByY_ascending(VisionAppQDragBox * rect1, VisionAppQDragBox * rect2);


bool compareRectanglesByX_ascending(VisionAppQDragBox * rect1, VisionAppQDragBox * rect2);

// Utility to sort filenames naturally
bool naturalSort(const QFileInfo& a, const QFileInfo& b) {
	return a.fileName().toLower() < b.fileName().toLower();  // Optional: Use QCollator for smarter sorting
}

bool naturalSortByNumber(const QFileInfo& a, const QFileInfo& b) {
	static const QRegularExpression re("(\\d+)");

	QRegularExpressionMatch matchA = re.match(a.baseName());
	QRegularExpressionMatch matchB = re.match(b.baseName());

	if (matchA.hasMatch() && matchB.hasMatch()) {
		int numA = matchA.captured(1).toInt();
		int numB = matchB.captured(1).toInt();
		return numA < numB;
	}

	return a.fileName().toLower() < b.fileName().toLower();
}

static void loadInputImagesCPU(const std::string& folderPath, const std::string& extension, std::vector<cv::Mat>& input_images) {
	std::cout << "Loading Images..." << std::endl;
	auto start = std::chrono::high_resolution_clock::now();

	int i = 0; // Start with the first image
	while (true) {
		std::string filePath = folderPath + "/" + std::to_string(i) + "." + extension;

		// Check if the file exists
		std::ifstream file(filePath);
		if (!file.good()) {
			if (i == 0) {
				std::cerr << "Error: No images found in folder: " << folderPath << " with extension: " << extension << std::endl;
			}
			break;
		}

		cv::Mat img = cv::imread(filePath, cv::IMREAD_UNCHANGED);	// 3-channels BGR
		if (img.empty()) {
			std::cerr << "Failed to load image: " << filePath << std::endl;
			continue; // Skip to the next image
		}

		//Mat grayImg;									// GRAY
		//cvtColor(img, grayImg, COLOR_BGR2GRAY);		//
		//img = grayImg;								//

		//cv::Mat singleChannel;						// single color channel B/G/R
		//cv::extractChannel(img, singleChannel, 2);	// 0 = blue, 1 = green, 2 = red
		//img = singleChannel;							//

		input_images.push_back(img); // Add the Mat to the vector

		std::cout << std::to_string(i) << std::endl;
		++i; // Move to the next image index
		//++i; // Move to the next image index
	}
	auto end = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double, std::milli> duration = end - start;
	std::cout << "Time taken: " << duration.count() << " ms\n";
}

static void focus_stacking_saveImage(const cv::cuda::GpuMat& image, const std::string& filePath) {
	cv::Mat output;
	image.download(output);
	cv::imwrite(filePath, output);
	std::cout << "Image saved to " << filePath << std::endl;
}

void VisionApp::connectShortcuts()
{
	QShortcut *shortcutImageView = new QShortcut(QKeySequence(Qt::Key_Space), this);
	connect(shortcutImageView, SIGNAL(activated()), this, SLOT(toggleImageView()));

	QShortcut *shortcutEscape = new QShortcut(QKeySequence(Qt::Key_Escape), this);
	connect(shortcutEscape, SIGNAL(activated()), this, SLOT(escapeKeyPressed()));

	QShortcut *shortcutToggleDrawingAndRois = new QShortcut(tr("`"), this);
	connect(shortcutToggleDrawingAndRois, SIGNAL(activated()), this, SLOT(toggleDrawingAndRois()));

	QShortcut * shortcutEnter = new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_Return), this);
	connect(shortcutEnter, SIGNAL(activated()), this, SLOT(editTemplate()));

	QShortcut *shortcut_alt1 = new QShortcut(QKeySequence(Qt::ALT + Qt::Key_1), this);
	connect(shortcut_alt1, &QShortcut::activated, [=]() {
		//testJob();
	});

	QShortcut *shortcut_alt2 = new QShortcut(QKeySequence(Qt::ALT + Qt::Key_2), this);
	connect(shortcut_alt2, &QShortcut::activated, [=]() {

	});

	QShortcut *shortcut_alt3 = new QShortcut(QKeySequence(Qt::ALT + Qt::Key_3), this);
	connect(shortcut_alt3, &QShortcut::activated, [=]() {
	});

	QShortcut *shortcut_alt4 = new QShortcut(QKeySequence(Qt::ALT + Qt::Key_4), this);
	connect(shortcut_alt4, &QShortcut::activated, [=]() {
	});

	QShortcut *shortcut_alt5 = new QShortcut(QKeySequence(Qt::ALT + Qt::Key_5), this);
	connect(shortcut_alt5, &QShortcut::activated, [=]() { 
	});

	QShortcut *shortcut_alt6 = new QShortcut(QKeySequence(Qt::ALT + Qt::Key_6), this);
	connect(shortcut_alt6, &QShortcut::activated, [=]() {
		//ct::logger::info("LSC: Set programmable trigger mode");
		//
		//for (auto& v : _views) {
		//	QVector<LSCManager::SequenceData> datas;
		//	int triggerSource = 1;
		//
		//	for (auto& opticID : v.opticIDs) {
		//
		//		auto& optic = _recipeOptics[opticID];
		//
		//		//each setting
		//		auto exposure = optic.exposure;
		//		/*auto camLSC = CAMManager::instance().lsc(optic.camID);
		//		if (camLSC == nullptr) {
		//			ct::logger::error("Invalid LSC info for camera: %s", optic.camID.toStdString().c_str());
		//			continue;
		//		}*/
		//
		//		LSCManager::SequenceData data;
		//		data.exposure_us = exposure;
		//
		//		if (optic.type == ct::s_mono) {
		//			data.band = optic.M;
		//			datas.push_back(data);
		//		}
		//		else {
		//			data.band = optic.R;
		//			datas.push_back(data);
		//			data.band = optic.G;
		//			datas.push_back(data);
		//			data.band = optic.B;
		//			datas.push_back(data);
		//		}
		//		//triggerSource = data.triggerSource = camLSC->triggerSource;
		//	}
		//
		//	LSCManager::instance().setTriggerSequence(datas);
		//
		//	break;
		//}
		//
		//LSCManager::instance().setMode(lsc::MODE::TRIGGER);
	});

	QShortcut *shortcut_alt7 = new QShortcut(QKeySequence(Qt::ALT + Qt::Key_7), this);
	connect(shortcut_alt7, &QShortcut::activated, [=]() {
		//bool ok;
		//QString text = QInputDialog::getText(this, tr("Delay"), "Enter wait time (ms)", QLineEdit::Normal, QString("1"), &ok);
		//int ms = text.toInt();
		//int resetIO = 1;
		//int triggerIO = 2;
		//
		////Simulate lighting
		//CAMManager::instance().setDO(_camID, resetIO, true);
		//os_tool::doNothing(ms);
		//CAMManager::instance().setDO(_camID, triggerIO, true);
		//os_tool::doNothing(ms);
		//CAMManager::instance().setDO(_camID, triggerIO, false);
		//
		//os_tool::doNothing(ms);
		//
		//CAMManager::instance().setDO(_camID, triggerIO, true);
		//os_tool::doNothing(ms);
		//CAMManager::instance().setDO(_camID, triggerIO, false);
		//
		//os_tool::doNothing(ms);
		//
		//CAMManager::instance().setDO(_camID, triggerIO, true);
		//os_tool::doNothing(ms);
		//CAMManager::instance().setDO(_camID, triggerIO, false);
		//
		//CAMManager::instance().setDO(_camID, resetIO, false);
	});

	QShortcut *shortcut_alt8 = new QShortcut(QKeySequence(Qt::ALT + Qt::Key_8), this);
	connect(shortcut_alt8, &QShortcut::activated, [=]() {
		if (notAllowToAccess(AccessLevel::ENGINEER)) return;
		if (notAllowToAccess(AccessLevel::OPERATOR)) return;
		if (!passwordPromptCorrect()) return;

		const auto roi = _commonDragBox.getGeometry();
		bool ok = false;
		const double maxCurrentAmp = QInputDialog::getDouble(
			this,
			QStringLiteral("Max Current Calibration"),
			QStringLiteral("Max current (A):"),
			2.0,
			0.001,
			2.0,
			4,
			&ok);

		if (!ok) {
			ct::logger::info("[Alt8][MaxCurrent] Shortcut cancelled at max current prompt. Cam: %s",
				_camID.toStdString().c_str());
			return;
		}

		const double plateauDiffThreshold = QInputDialog::getDouble(
			this,
			QStringLiteral("Max Current Calibration"),
			QStringLiteral("Plateau diff threshold:"),
			0.03,
			0.0,
			255.0,
			6,
			&ok);

		if (!ok) {
			ct::logger::info("[Alt8][MaxCurrent] Shortcut cancelled at threshold prompt. Cam: %s",
				_camID.toStdString().c_str());
			return;
		}

		const QString password = QInputDialog::getText(
			this,
			QStringLiteral("Max Current Calibration"),
			QStringLiteral("Password:"),
			QLineEdit::Password,
			QString(),
			&ok);

		if (!ok) {
			ct::logger::info("[Alt8][MaxCurrent] Shortcut cancelled at password prompt. Cam: %s",
				_camID.toStdString().c_str());
			return;
		}

		if (password != QStringLiteral("3df")) {
			QMessageBox::warning(this, QStringLiteral("Max Current Calibration"), QStringLiteral("Invalid password."));
			ct::logger::warn("[Alt8][MaxCurrent] Shortcut blocked by invalid password. Cam: %s",
				_camID.toStdString().c_str());
			return;
		}

		ct::logger::info("[Alt8][MaxCurrent] Shortcut triggered. Cam: %s, ROI: %.2f, %.2f, %.2f, %.2f, Threshold: %.6f, MaxCurrent: %.4f A",
			_camID.toStdString().c_str(), roi.x(), roi.y(), roi.width(), roi.height(), plateauDiffThreshold, maxCurrentAmp);
		emit triggerMaxCurrentCalibration(_camID, roi, plateauDiffThreshold, maxCurrentAmp);

	});

	QShortcut* shortcut_alt9 = new QShortcut(QKeySequence(Qt::ALT + Qt::Key_9), this);
	connect(shortcut_alt9, &QShortcut::activated, [=]() {
		stopLiveView();
		ct::logger::info("[Alt9][FastSnap] Camera: %s", _camID.toStdString().c_str());

		CAMManager::instance().resetFrame(_camID);

		if (!ui.toolButton_toggleDualView->isChecked() && !ui.toolButton_toggleFovView->isChecked()) {
			toggleFOVView();
		}

		emit snapImageFastMode(_mainOptics[_camID], "main", "");
	});

	QShortcut *shortcut_altl = new QShortcut(QKeySequence(Qt::ALT + Qt::Key_L), this);
	connect(shortcut_altl, &QShortcut::activated, [=]() {
		////lock vision object
		//for (auto& vo : _visionObject) {
		//	if (vo.pDragBox->isSelected()) {
		//		if (vo.locked) {
		//			vo.locked = false;
		//			vo.pDragBox->setDragable(true);
		//		}
		//		else {
		//			vo.locked = true;
		//			vo.pDragBox->setDragable(false);
		//		}
		//	}
		//}
	});

	QShortcut *shortcut_dlt = new QShortcut(QKeySequence(Qt::Key_Delete), this);
	connect(shortcut_dlt, &QShortcut::activated, [=]() {
		QVector<QDragBox*> _selectedROIs;
		for (auto roi : _dragROI) {
			if (roi->isSelected()) {
				_selectedROIs.push_back(roi);
			}
		}
		
		for (auto roi : _viewROI) {
			if (roi->isSelected()) {
				_selectedROIs.push_back(roi);
			}
		}

		for (auto roi : _lineScanROI) {
			if (roi->isSelected()) {
				_selectedROIs.push_back(roi);
			}
		}

		for (auto roi : _selectedROIs) {
			auto key = roi->getId();
			AuditLog::instance().log(QStringLiteral("DELETE_OBJECT"), key);

			clearDragBox(roi);
			
			if (_visionObject.contains(key)) _visionObject.remove(key);
			if (_views.contains(key)) _views.remove(key);
			if (_lineScans.contains(key)) _lineScans.remove(key);

			for (auto& obj : _visionObject)   
			{				
				if (obj.viewID == key)
				{
					obj.viewID = "";
				}
			}

			for (auto& view : _views)
			{
				for (int j = 0; j < view.vision_obj_IDs.size(); j++)
				{
					if (view.vision_obj_IDs[j] == key)
					{
						view.vision_obj_IDs.remove(j);
						j--;
					}
				}
			}

			for (auto& l : _lineScans)
			{
				for (int j = 0; j < l.vision_obj_IDs.size(); j++)
				{
					if (l.vision_obj_IDs[j] == key)
					{
						l.vision_obj_IDs.remove(j);
						j--;
					}
				}
			}
		}

		clearEmptyViewKey();

		updateTreeViewExplorer(Common::Directory::CurrentRecipe, _views, _visionObject);


		//FOV
		for (int i = 0; i < _CSA.searchLocator.size(); i++) {

			auto p = _CSA.searchLocator[i];

			if (p->isSelected() && _CSA.searchLocator.size() > 1) {
				deleteDragBox(nullptr, p);
				_CSA.searchLocator.erase(_CSA.searchLocator.begin() + i);
				i--;
			}
		}
	});

	QShortcut *shortcut_cr = new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_R), this);
	connect(shortcut_cr, &QShortcut::activated, [=]() { 
		//SystemData::instance()._index = 0;
		//CAMManager::instance().reconnect("cam1");
	});

	QShortcut* shortcut_ce = new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_E), this);
	connect(shortcut_ce, &QShortcut::activated, [=]() {
		CAMManager::instance().reconnect("cam1");

		bool ok;
		QString text = QInputDialog::getText(this, tr("Exposure"), "Enter exposure (us)", QLineEdit::Normal, QString("1"), &ok);

		if (!ok) return "USER_CANCELED";

		CAMManager::instance().setExposure("cam1", text.simplified().toDouble());
	});

	QShortcut* shortcut_cm = new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_M), this);
	connect(shortcut_cm, &QShortcut::activated, [=]() {
		/*ui.stackedWidgetViewSelection->setCurrentIndex(5);
		ui.stackedWidget->setCurrentIndex(17);
		ui.frame_leftTab->hide();*/
	});
	
	QShortcut *shortcut_1 = new QShortcut(QKeySequence(Qt::Key_1 ), this);
	connect(shortcut_1, &QShortcut::activated, [=]() {
		
		//// block operator access
		//if (notAllowToAccess(AccessLevel::OPERATOR)) return;
		//
		//// block engineer access
		//if (notAllowToAccess(AccessLevel::ENGINEER)) return;
		//if (notAllowToAccess(AccessLevel::OPERATOR)) return;

	});

	QShortcut* shortcut_2 = new QShortcut(QKeySequence(Qt::Key_2), this);
	connect(shortcut_2, &QShortcut::activated, [=]() {

		qDebug() << "Width: " << ui.centralWidget->size().width();
		qDebug() << "Height: " << ui.centralWidget->size().height();
	});

	QShortcut* shortcut_c0 = new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_0), this);
	connect(shortcut_c0, &QShortcut::activated, [=]() {
	});

	QShortcut *shortcut_c1 = new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_1), this);
	connect(shortcut_c1, &QShortcut::activated, [=]() {
		//emit testJob();
	});

	QShortcut *shortcut_c2 = new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_2), this);
	connect(shortcut_c2, &QShortcut::activated, [=]() { 
		guidedAlignCameraAndLaserSetup();
	});

	QShortcut *shortcut_c3 = new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_3), this);
	connect(shortcut_c3, &QShortcut::activated, [=]() {
		guidedAlignPositionPortabilitySetup();
	});

	QShortcut *shortcut_c4 = new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_4), this);
	connect(shortcut_c4, &QShortcut::activated, [=]() { 
		if (notAllowToAccess(AccessLevel::OPERATOR)) return;

		auto c = SystemData::instance().currentCoordinate();
		auto d = c;
		d.wx = d.wx - 40;
		d.wy = d.wy + 40;
		emit jogTo(d.wx, d.wy, d.wz);
		emit jogSnap(c.wx, c.wy, c.wz, _mainOptics[_camID]);
	});

	QShortcut *shortcut_c5 = new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_5), this);
	connect(shortcut_c5, &QShortcut::activated, [=]() { 
		
		if (notAllowToAccess(AccessLevel::OPERATOR)) return;
		struct Meas {
			QString sourceFile;   // relative file name
			int rowIndex = -1;    // data row index (starting from 1 after header)
			double height = 0.0;
			double length = 0.0;
		};

		auto normalizeHeader = [](QString s) -> QString {
			s = s.trimmed().toLower();
			s.remove(QRegularExpression("[^a-z0-9]+")); // keep only alnum
			return s;
			};

		// CSV parser (comma + quotes)
		auto parseCsvLine = [](const QString& line) -> QStringList {
			QStringList out;
			QString cur;
			bool inQuotes = false;

			for (int i = 0; i < line.size(); ++i) {
				const QChar c = line[i];
				if (c == '"') {
					if (inQuotes && i + 1 < line.size() && line[i + 1] == '"') {
						cur += '"';
						++i;
					}
					else {
						inQuotes = !inQuotes;
					}
				}
				else if (c == ',' && !inQuotes) {
					out << cur;
					cur.clear();
				}
				else {
					cur += c;
				}
			}
			out << cur;
			return out;
			};

		auto toDoubleSafe = [](const QString& s, bool* okOut = nullptr) -> double {
			bool ok = false;
			double v = s.trimmed().toDouble(&ok);
			if (okOut) *okOut = ok;
			return ok ? v : 0.0;
			};

		auto sanitize = [](QString s) -> QString {
			s.replace(QRegularExpression(R"([\\/:*?"<>|\s]+)"), "_");
			return s;
			};

		auto csvEscape = [](QString s) -> QString {
			if (s.contains('"')) s.replace("\"", "\"\"");
			const bool needQuotes = s.contains(',') || s.contains('"') || s.contains('\n') || s.contains('\r');
			return needQuotes ? QString("\"%1\"").arg(s) : s;
			};

		// ---- 1) Pick folder ----
		const QString folder = QFileDialog::getExistingDirectory(
			this,
			"Select folder containing CSV reports",
			QString(),
			QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks
		);
		if (folder.isEmpty())
			return;

		// ---- 2) Scan CSV files ----
		// source_file -> pin -> meas (first occurrence)
		QMap<QString, QMap<QString, Meas>> filePinFirst;
		QSet<QString> allPins;
		QStringList fileOrder;
		int dupTotal = 0;

		QDirIterator it(folder, QStringList() << "*.csv", QDir::Files, QDirIterator::Subdirectories);

		int filesRead = 0;
		while (it.hasNext()) {
			const QString filePath = it.next();
			const QString fileKey = QDir(folder).relativeFilePath(filePath);
			if (!fileOrder.contains(fileKey))
				fileOrder << fileKey;

			QFile f(filePath);
			if (!f.open(QIODevice::ReadOnly | QIODevice::Text))
				continue;

			QTextStream ts(&f);
			if (ts.atEnd()) { f.close(); continue; }

			const QString headerLine = ts.readLine();
			const QStringList headers = parseCsvLine(headerLine);

			int colPin = -1;
			int colHeight = -1;
			int colLength = -1;

			// Pass 1: prefer NON-nominal
			for (int c = 0; c < headers.size(); ++c) {
				const QString rawLower = headers[c].trimmed().toLower();
				const QString h = normalizeHeader(headers[c]);
				const bool isNominal = rawLower.contains("nominal") || h.startsWith("nominal");

				if (colPin < 0 && (h.contains("pinname") || (h.contains("pin") && h.contains("name"))))
					colPin = c;

				if (!isNominal && colHeight < 0 &&
					(h.contains("3dheight") || (h.contains("height") && h.contains("3d"))))
					colHeight = c;

				if (!isNominal && colLength < 0 && h.contains("length"))
					colLength = c;
			}

			// Pass 2: fallback (only if still not found)
			for (int c = 0; c < headers.size() && (colHeight < 0 || colLength < 0); ++c) {
				const QString h = normalizeHeader(headers[c]);

				if (colHeight < 0 && (h.contains("3dheight") || (h.contains("height") && h.contains("3d"))))
					colHeight = c;

				if (colLength < 0 && h.contains("length"))
					colLength = c;
			}

			if (colPin < 0 || (colHeight < 0 && colLength < 0)) {
				f.close();
				continue;
			}

			int rowIdx = 0;
			while (!ts.atEnd()) {
				const QString line = ts.readLine();
				if (line.trimmed().isEmpty())
					continue;

				++rowIdx;
				const QStringList cols = parseCsvLine(line);

				auto getCol = [&](int idx) -> QString {
					if (idx < 0 || idx >= cols.size()) return QString();
					return cols[idx];
					};

				const QString pin = getCol(colPin).trimmed();
				if (pin.isEmpty())
					continue;

				bool okH = true, okL = true;
				const double hVal = (colHeight >= 0) ? toDoubleSafe(getCol(colHeight), &okH) : 0.0;
				const double lVal = (colLength >= 0) ? toDoubleSafe(getCol(colLength), &okL) : 0.0;

				// If both missing/unparseable, skip
				if ((colHeight >= 0 && !okH) && (colLength >= 0 && !okL))
					continue;

				Meas m;
				m.sourceFile = fileKey;
				m.rowIndex = rowIdx;
				m.height = hVal;
				m.length = lVal;

				auto& pinMap = filePinFirst[fileKey];
				if (!pinMap.contains(pin)) {
					pinMap.insert(pin, m);   // keep FIRST occurrence per file
				}
				else {
					++dupTotal;
				}

				allPins.insert(pin);
			}

			f.close();
			++filesRead;
		}

		if (filesRead == 0 || allPins.isEmpty()) {
			QMessageBox::warning(this, "CTRL+5 Extract", "No readable CSV files found (or no Pin Name data).");
			return;
		}

		// ---- 3) Checkbox dialog for multi-select pins ----
		QStringList pinList = allPins.values();
		pinList.sort(Qt::CaseInsensitive);

		QDialog dlg(this);
		dlg.setWindowTitle("Select Pin Names");

		auto* lay = new QVBoxLayout(&dlg);
		lay->addWidget(new QLabel(
			QString("Found %1 unique pins across %2 file(s).\nTick the pins you want to export:")
			.arg(pinList.size()).arg(filesRead),
			&dlg
		));

		auto* listWidget = new QListWidget(&dlg);
		listWidget->setSelectionMode(QAbstractItemView::NoSelection);

		for (const auto& p : pinList) {
			auto* itItem = new QListWidgetItem(p, listWidget);
			itItem->setFlags(itItem->flags() | Qt::ItemIsUserCheckable);
			itItem->setCheckState(Qt::Unchecked);
		}
		lay->addWidget(listWidget);

		auto* row = new QHBoxLayout();
		auto* btnAll = new QPushButton("Select All", &dlg);
		auto* btnNone = new QPushButton("Select None", &dlg);
		row->addWidget(btnAll);
		row->addWidget(btnNone);
		row->addStretch(1);
		lay->addLayout(row);

		QObject::connect(btnAll, &QPushButton::clicked, [&]() {
			for (int i = 0; i < listWidget->count(); ++i)
				listWidget->item(i)->setCheckState(Qt::Checked);
			});
		QObject::connect(btnNone, &QPushButton::clicked, [&]() {
			for (int i = 0; i < listWidget->count(); ++i)
				listWidget->item(i)->setCheckState(Qt::Unchecked);
			});

		auto* bb = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dlg);
		QObject::connect(bb, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
		QObject::connect(bb, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
		lay->addWidget(bb);

		if (dlg.exec() != QDialog::Accepted)
			return;

		QStringList selectedPins;
		for (int i = 0; i < listWidget->count(); ++i) {
			if (listWidget->item(i)->checkState() == Qt::Checked)
				selectedPins << listWidget->item(i)->text();
		}

		if (selectedPins.isEmpty()) {
			QMessageBox::warning(this, "CTRL+5 Extract", "No pins selected.");
			return;
		}

		// ---- 4) Export wide CSV ----
		QString outPath = QFileDialog::getSaveFileName(
			this,
			"Save extracted CSV",
			folder + "/extract_pins.csv",
			"CSV Files (*.csv)"
		);
		if (outPath.isEmpty())
			return;

		QFile outFile(outPath);
		if (!outFile.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
			QMessageBox::warning(this, "CTRL+5 Extract", "Failed to write output CSV:\n" + outPath);
			return;
		}

		QTextStream out(&outFile);

		// unique safe column names
		QMap<QString, QString> pinToSafe;
		QSet<QString> used;
		for (const auto& p : selectedPins) {
			QString base = sanitize(p);
			QString name = base;
			int k = 2;
			while (used.contains(name))
				name = QString("%1_%2").arg(base).arg(k++);
			used.insert(name);
			pinToSafe[p] = name;
		}

		// header
		out << "source_file";
		for (const auto& p : selectedPins) {
			const QString safe = pinToSafe[p];
			out << "," << safe << "_height" << "," << safe << "_length";
		}
		out << "\n";

		// rows: 1 row per file
		for (const auto& fileKey : fileOrder) {
			out << csvEscape(fileKey);

			const auto pinMap = filePinFirst.value(fileKey);
			for (const auto& p : selectedPins) {
				if (pinMap.contains(p)) {
					const auto& m = pinMap[p];
					out << "," << QString::number(m.height, 'g', 16)
						<< "," << QString::number(m.length, 'g', 16);
				}
				else {
					out << ",,";
				}
			}
			out << "\n";
		}

		outFile.close();

		QString note = (dupTotal > 0)
			? QString("\n\nNote: %1 duplicate pin rows were ignored (kept first occurrence per file).").arg(dupTotal)
			: QString();

		QMessageBox::information(
			this,
			"CTRL+5 Extract",
			QString("Done.\nPins selected: %1\nFiles: %2\nOutput:\n%3%4")
			.arg(selectedPins.size())
			.arg(filesRead)
			.arg(outPath)
			.arg(note)
		);
	});

	QShortcut *shortcut_c6 = new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_6), this);
	connect(shortcut_c6, &QShortcut::activated, [=]() {

		// 1) Prompt select root folder
		const QString root = QFileDialog::getExistingDirectory(
			this,
			"Select root folder (contains many production folders)",
			QDir::homePath()
		);
		if (root.isEmpty()) return;

		// 2) Prompt select main or sub
		bool ok = false;
		const QStringList modes = { "main", "subrecipe1" };
		const QString mode = QInputDialog::getItem(
			this,
			"Select recipe type",
			"Read which LaserMeasurement file?",
			modes,
			0,
			false,
			&ok
		);
		if (!ok || mode.isEmpty()) return;

		const QString targetFile =
			(mode == "main") ? "LaserMeasurement[#]main.json"
			: "LaserMeasurement[#]subrecipe1.json";

		// Find all matching JSON files under root (recursively)
		QStringList jsonPaths;

		// Case 1: user selected ONE production folder directly
		{
			QStringList resultsNames = { "Results", "results" };
			for (const QString& rn : resultsNames) {
				const QString p = QDir(root).filePath(rn + "/" + targetFile);
				if (QFileInfo::exists(p)) {
					jsonPaths << p;
					break;
				}
			}
		}

		// Case 2: user selected a folder that contains MANY production folders
		if (jsonPaths.isEmpty())
		{
			QDir rootDir(root);
			QFileInfoList prodDirs = rootDir.entryInfoList(QDir::Dirs | QDir::NoDotAndDotDot);

			for (const QFileInfo& pfi : prodDirs)
			{
				const QString prodPath = pfi.absoluteFilePath();

				// results folder could be "Results" or "results"
				QString resultsPath;
				if (QDir(prodPath + "/Results").exists()) resultsPath = prodPath + "/Results";
				else if (QDir(prodPath + "/results").exists()) resultsPath = prodPath + "/results";
				else continue;

				const QString jsonPath = resultsPath + "/" + targetFile;
				if (QFileInfo::exists(jsonPath))
					jsonPaths << jsonPath;
			}
		}

		if (jsonPaths.isEmpty()) {
			QMessageBox::warning(this, "Not found",
				"No " + targetFile + " found in:\n" + root + "\n\nExpected:\nroot/<production>/Results/" + targetFile);
			return;
		}

		// Storage:
		// per file(row) -> vo -> unit -> list of avg heights
		struct RowCache {
			QString jsonPath;
			QString rowName; // display name (relative folder)
			QHash<QString, QHash<QString, QVector<double>>> values;
		};

		QVector<RowCache> rows;
		rows.reserve(jsonPaths.size());

		QSet<QString> allVos;
		QSet<QString> allUnits;

		// Parse each JSON
		for (const QString& jp : jsonPaths)
		{
			QFile f(jp);
			if (!f.open(QIODevice::ReadOnly)) continue;
			const QByteArray data = f.readAll();
			f.close();

			QJsonParseError pe;
			QJsonDocument doc = QJsonDocument::fromJson(data, &pe);
			if (pe.error != QJsonParseError::NoError) continue;

			QJsonArray arr;

			if (doc.isArray()) {
				arr = doc.array();
			}
			else if (doc.isObject()) {
				const QJsonObject o = doc.object();
				// try common array keys (in case your file is wrapped)
				const QStringList keys = { "items", "data", "measurements", "MeasurementList", "LaserMeasurements", "LaserMeasurement" };
				for (const QString& k : keys) {
					if (o.value(k).isArray()) {
						arr = o.value(k).toArray();
						break;
					}
				}
			}

			if (arr.isEmpty()) continue;

			RowCache rc;
			rc.jsonPath = jp;

			QFileInfo fi(jp);
			QDir d = fi.dir();           // .../Results
			d.cdUp();                    // .../<production>
			const QString prodFolder = d.absolutePath();
			rc.rowName = QDir(root).relativeFilePath(prodFolder);

			for (const QJsonValue& v : arr)
			{
				if (!v.isObject()) continue;
				const QJsonObject o = v.toObject();

				const QString vo = o.value("VoName").toString().trimmed();
				const QString unit = o.value("UnitName").toString().trimmed();
				if (vo.isEmpty() || unit.isEmpty()) continue;

				bool numOk = false;
				double avg = 0.0;
				const QJsonValue av = o.value("AverageHeight_um");
				if (av.isDouble()) { avg = av.toDouble(); numOk = true; }
				else if (av.isString()) { avg = av.toString().toDouble(&numOk); }

				if (!numOk) continue;

				allVos.insert(vo);
				allUnits.insert(unit);
				rc.values[vo][unit].push_back(avg);
			}

			// keep only if it has any parsed content
			if (!rc.values.isEmpty())
				rows.push_back(std::move(rc));
		}

		if (rows.isEmpty() || allVos.isEmpty() || allUnits.isEmpty()) {
			QMessageBox::warning(this, "No usable data",
				"Found files, but couldn't extract VoName/UnitName/AverageHeight_um.");
			return;
		}

		QStringList voList = allVos.values();
		voList.sort(Qt::CaseInsensitive);

		QStringList unitList = allUnits.values();
		unitList.sort(Qt::CaseInsensitive);

		// 3) Prompt multi-select VoName (QDialog inline)
		QStringList selectedVos;
		{
			QDialog dlg(this);
			dlg.setWindowTitle("Select VoName(s)");
			dlg.setMinimumWidth(420);

			auto* layout = new QVBoxLayout(&dlg);
			layout->addWidget(new QLabel("Select one or more VoName(s):", &dlg));

			auto* list = new QListWidget(&dlg);
			list->addItems(voList);
			list->setSelectionMode(QAbstractItemView::ExtendedSelection);
			layout->addWidget(list, 1);

			auto* rowBtns = new QHBoxLayout();
			auto* btnAll = new QPushButton("Select All", &dlg);
			auto* btnNone = new QPushButton("Select None", &dlg);
			rowBtns->addWidget(btnAll);
			rowBtns->addWidget(btnNone);
			rowBtns->addStretch(1);
			layout->addLayout(rowBtns);

			QObject::connect(btnAll, &QPushButton::clicked, &dlg, [list]() { list->selectAll(); });
			QObject::connect(btnNone, &QPushButton::clicked, &dlg, [list]() { list->clearSelection(); });

			auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dlg);
			layout->addWidget(buttons);
			QObject::connect(buttons, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
			QObject::connect(buttons, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);

			if (dlg.exec() != QDialog::Accepted) return;

			const auto sel = list->selectedItems();
			for (auto* it : sel) selectedVos << it->text();
		}

		if (selectedVos.isEmpty()) {
			QMessageBox::information(this, "Nothing selected", "No VoName selected.");
			return;
		}

		// 4) Prompt UnitName (single select)
		const QString selectedUnit = QInputDialog::getItem(
			this,
			"Select UnitName",
			"Pick UnitName:",
			unitList,
			0,
			false,
			&ok
		);
		if (!ok || selectedUnit.isEmpty()) return;

		// 5) Export CSV in same root folder
		const QString outName = QString("LaserMeasurement_%1_%2_%3.csv")
			.arg(mode)
			.arg(selectedUnit)
			.arg(QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss"));

		const QString outPath = QDir(root).filePath(outName);

		QFile out(outPath);
		if (!out.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
			QMessageBox::warning(this, "Error", "Cannot write:\n" + outPath);
			return;
		}

		QTextStream ts(&out);

		// Write CSV header (always quote fields, escape quotes)
		{
			// Folder
			QString fld = "Folder"; fld.replace("\"", "\"\"");
			ts << "\"" << fld << "\"";

			for (const QString& vo : selectedVos) {
				QString h = vo + "_" + selectedUnit;
				h.replace("\"", "\"\"");
				ts << ",\"" << h << "\"";
			}
			ts << "\n";
		}

		// Write rows
		for (const auto& rc : rows)
		{
			QString folder = rc.rowName;
			folder.replace("\"", "\"\"");
			ts << "\"" << folder << "\"";

			for (const QString& vo : selectedVos)
			{
				const auto unitMap = rc.values.value(vo);
				const QVector<double> vals = unitMap.value(selectedUnit);

				if (vals.isEmpty()) {
					ts << ",\"\"";
				}
				else {
					// if duplicates exist, average them
					double sum = 0.0;
					for (double x : vals) sum += x;
					const double mean = sum / double(vals.size());
					QString v = QString::number(mean, 'f', 6);
					v.replace("\"", "\"\"");
					ts << ",\"" << v << "\"";
				}
			}
			ts << "\n";
		}

		out.close();

		QMessageBox::information(this, "Done", "CSV exported:\n" + outPath);
	});

	QShortcut *shortcut_c7 = new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_7), this);
	connect(shortcut_c7, &QShortcut::activated, [=]() {
	});

	QShortcut *shortcut_c8 = new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_8), this);
	connect(shortcut_c8, &QShortcut::activated, [=]() {
	});

	QShortcut *shortcut_c9 = new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_9), this);
	connect(shortcut_c9, &QShortcut::activated, [=]() {
	});

	QShortcut* shortcut_ctrlp = new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_P), this);
	connect(shortcut_ctrlp, &QShortcut::activated, [=]() {
		SystemData::instance().triggerPSP();
	});

	QShortcut *shortcut_ctrlj = new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_J), this);
	connect(shortcut_ctrlj, &QShortcut::activated, [=]() { 
		if (notAllowToAccess(AccessLevel::OPERATOR)) return;

		bool ok;
		QString text = QInputDialog::getText(this, tr("Jog Step"), "Enter step (mm)", QLineEdit::Normal, QString("1"), &ok);

		// if (!ok) return "USER_CANCELED";
		if (!ok) return;

		_jogDistance = text.simplified().toDouble();
	});

	QShortcut *shortcut_ctrlk = new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_K), this);
	connect(shortcut_ctrlk, &QShortcut::activated, [=]() {
		if (notAllowToAccess(AccessLevel::OPERATOR)) return;
		bool ok;

		QString x = QInputDialog::getText(this, tr("Jog To"), "X coordinate", QLineEdit::Normal, QString("250"), &ok);
		if (!ok) return;

		QString y = QInputDialog::getText(this, tr("Jog To"), "Y coordinate", QLineEdit::Normal, QString("250"), &ok);
		if (!ok) return;

		QString z = QInputDialog::getText(this, tr("Jog To"), "Z coordinate", QLineEdit::Normal, QString("0"), &ok);
		if (!ok) return;

		auto x_mm = x.simplified().toDouble();
		auto y_mm = y.simplified().toDouble();
		auto z_mm = z.simplified().toDouble();

		//jogTo(x_mm, y_mm, z_mm);
		emit jogSnap(x_mm, y_mm, z_mm, _mainOptics[_camID]);
	});

	QShortcut *shortcut_ctrln = new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_N), this);
	connect(shortcut_ctrln, &QShortcut::activated, [=]() {
		if (notAllowToAccess(AccessLevel::OPERATOR)) return;
		
		_editMode = EditMode::NAVIGATE_TO;
		_cursor.setShape(Qt::CrossCursor);
		ui.graphicsViewMain->setCursor(_cursor);
	});

	QShortcut* shortcut_ctrll = new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_L), this);
	connect(shortcut_ctrll, &QShortcut::activated, [=]() {
		QStringList list;
		list.append("trace");
		list.append("debug");
		list.append("info");
		list.append("warn");
		list.append("error");

		QString logLevel = promptComboBox(list, "Log Levels", "Set the log levels");

		if (logLevel == "trace") ct::logger::set_level(ct::logger::Level::TRACE);
		else if (logLevel == "debug") ct::logger::set_level(ct::logger::Level::DEBUG);
		else if (logLevel == "info") ct::logger::set_level(ct::logger::Level::INFO);
		else if (logLevel == "warn") ct::logger::set_level(ct::logger::Level::WARN);
		else if (logLevel == "error") ct::logger::set_level(ct::logger::Level::L_ERROR);
	});

	QShortcut *shortcut_up = new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_Up), this);
	connect(shortcut_up, &QShortcut::activated, [=]() { 
		if (notAllowToAccess(AccessLevel::OPERATOR)) return;
		
		if (!blockJogSignal()) return; emit jogBack(_jogDistance, _mainOptics[_camID]); });

	QShortcut *shortcut_down = new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_Down), this);
	connect(shortcut_down, &QShortcut::activated, [=]() {
		if (notAllowToAccess(AccessLevel::OPERATOR)) return;

		if (!blockJogSignal()) return; emit jogFront(_jogDistance, _mainOptics[_camID]); });

	QShortcut *shortcut_left = new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_Left), this);
	connect(shortcut_left, &QShortcut::activated, [=]() { 
		if (notAllowToAccess(AccessLevel::OPERATOR)) return; 
		
		if (!blockJogSignal()) return; emit jogLeft(_jogDistance, _mainOptics[_camID]); });

	QShortcut *shortcut_right = new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_Right), this);
	connect(shortcut_right, &QShortcut::activated, [=]() { 
		if (notAllowToAccess(AccessLevel::OPERATOR)) return; 
		
		if (!blockJogSignal()) return; emit jogRight(_jogDistance, _mainOptics[_camID]); });

	QShortcut *shortcut_arrowleft = new QShortcut(QKeySequence(Qt::Key_Left), this);
	connect(shortcut_arrowleft, &QShortcut::activated, [=]() { 
		ct::logger::trace("LEFT"); 
		if (notAllowToAccess(AccessLevel::OPERATOR)) return;

		if (_editMode == EditMode::IMAGE_FILTERING && !_imagesToFilter.empty()) {
			filterImage(Common::Directory::getRecipeVidiImageBadPath());
		}
	});

	QShortcut *shortcut_arrowright = new QShortcut(QKeySequence(Qt::Key_Right), this);
	connect(shortcut_arrowright, &QShortcut::activated, [=]() { 
		ct::logger::trace("RIGHT");
		if (notAllowToAccess(AccessLevel::OPERATOR)) return;

		if (_editMode == EditMode::IMAGE_FILTERING && !_imagesToFilter.empty()) {
			filterImage(Common::Directory::getRecipeVidiImageGoodPath());
		}
	});

	QShortcut *shortcut_arrowup = new QShortcut(QKeySequence(Qt::Key_Up), this);
	connect(shortcut_arrowup, &QShortcut::activated, [=]() {
		ct::logger::trace("UP");
		if (notAllowToAccess(AccessLevel::OPERATOR)) return;

		if (_editMode == EditMode::IMAGE_FILTERING && !_imagesToFilter.empty()) {
			auto root = ui.lineEdit_vidiImagePath->text();

			progressBarSetup("Undo action...", _lastFilteredImages.size());

			for (auto f : _lastFilteredImages) {
				auto& ifi = _imagesToFilter[f];
				auto dstPath = root + "/" + ifi.baseName + ".jpg";

				QFile::rename(ifi.currentPath, dstPath); //move file
				ifi.currentPath = dstPath;

				_unfilteredImages.insert(f);
				incrementProgressBar();
			}

			ui.lineEdit_filterStatus->setText(QString::number(_unfilteredImages.size()));
			ui.progressBar_filterStatus->setValue(ui.progressBar_filterStatus->maximum() - _unfilteredImages.size());

			_filterIndex = *_lastFilteredImages.begin();
			auto& nextImage = _imagesToFilter[_filterIndex];
			displayFOV(nextImage.qimg);

			progressBarRelease();
			_lastFilteredImages.clear();
		}
	});

	QShortcut *shortcut_top = new QShortcut(QKeySequence(Qt::Key_PageUp), this);
	connect(shortcut_top, &QShortcut::activated, [=]() { 
		if (notAllowToAccess(AccessLevel::OPERATOR)) return; 
		
		if (!blockJogSignal()) return; emit jogDown(_jogDistance, _mainOptics[_camID]); });

	QShortcut *shortcut_btm = new QShortcut(QKeySequence(Qt::Key_PageDown), this);
	connect(shortcut_btm, &QShortcut::activated, [=]() {
		if (notAllowToAccess(AccessLevel::OPERATOR)) return;

		if (!blockJogSignal()) return; emit jogUp(_jogDistance, _mainOptics[_camID]); });

	QShortcut *shortcut_ctrlS = new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_S), this);
	connect(shortcut_ctrlS, &QShortcut::activated, [=]() { if (notAllowToAccess(AccessLevel::OPERATOR)) return; saveRecipe(); });

	QShortcut *shortcut_undo = new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_Z), this);
	connect(shortcut_undo, &QShortcut::activated, [=]() { if (notAllowToAccess(AccessLevel::OPERATOR)) return; if (_undoStack->canUndo()) _undoStack->undo(); });

	QShortcut *shortcut_redo = new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_Y), this);
	connect(shortcut_redo, &QShortcut::activated, [=]() { if (notAllowToAccess(AccessLevel::OPERATOR)) return; if (_undoStack->canRedo()) _undoStack->redo(); });

	QShortcut *shortcut_test = new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_T), this);
	connect(shortcut_test, &QShortcut::activated, [=]() { testFunction(); });

	QShortcut *shortcut_ctrlF = new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_F), this);
	connect(shortcut_ctrlF, &QShortcut::activated, [=]() {
		if (notAllowToAccess(AccessLevel::ENGINEER)) return;
		if (notAllowToAccess(AccessLevel::OPERATOR)) return;

		QStringList list;
		list.append("Collect Image");
		list.append("Inspection");

		QString ret = promptComboBox(list, "Mode", "Set the mode");

		if (ret == "Collect Image") {
			_imageManager.reset();
			_processType = ProcessType::IMAGE_COLLECTION;
			auto rootPath = getNextSamplePath();
			clear2DImages(rootPath);
			SystemData::instance()._workingPath = rootPath;
			ct::logger::info("Root Path: %s", SystemData::instance()._workingPath.toStdString().c_str());
			_jobThread.setRootPath(rootPath);
		}
		else if (ret == "Inspection") {
			_imageManager.attach(&_views, &_recipeOptics);
			_imageManager.attach(&_lineScans, &_recipeOptics3D);
			_imageManager.reset();

			_processType = ProcessType::PRODUCTION;

			setupProductionDir();
		}
		//auto groupedKeys = OpticsControl::instance().getGroupedOptics();
		//progressBarSetup("Generating Lighting Report", groupedKeys.size() * 30, true);
		//_stopRun = false;

		//auto w = CAMManager::instance().getWidth(_camID);
		//auto h = CAMManager::instance().getHeight(_camID);
		////We can direct divide by 10 because all camera resolution is divisible by 10
		//int w_num = 10, h_num = 10; //10%
		//int w_segment = w / w_num;
		//int h_segment = h / h_num;

		//for (const auto& groupedKey : groupedKeys.keys()) {
		//	
		//	Common::Directory::createDir("data/");
		//	Common::Directory::createDir("data/" + groupedKey + "/");

		//	std::ofstream fout(QString("data/%1/data.csv").arg(groupedKey).toStdString());

		//	for (int w_idx = 0; w_idx < w_num; w_idx++) {
		//		for (int h_idx = 0; h_idx < h_num; h_idx++) {
		//			fout << std::to_string(w_idx) + "_" + std::to_string(h_idx) + ", ";
		//		}
		//	}
		//	fout << "Total";

		//	for (int i = 0; i < 30; i++) {

		//		QString imagePath = QString("data/%1/%2.jpg").arg(groupedKey).arg(i);

		//		if (_stopRun) return;
		//		//find the right exposure and gain

		//		OpticsControl::instance().toggleGroupedOptic(groupedKey, groupedKeys, true);
		//		OpticsControl::instance().setGroupedOpticIntensity(groupedKey, groupedKeys, MID_BRIGHTNESS);

		//		triggerCamera();

		//		MIL_ID mBuf = _cam.getBuffer();
		//		MIL_ID mMono = mtrx::to_mono(mBuf);
		//		mtrx::BufferCollector bc_mMono(mMono);

		//		MIL_UINT8* hostPtr = M_NULL;
		//		MIL_ID pitch = M_NULL;
		//		MbufInquire(mMono, M_HOST_ADDRESS, &hostPtr);
		//		MbufInquire(mMono, M_PITCH, &pitch);

		//		QImage qimg = mtrx::to_qimg(_cam.getBuffer());
		//		QPainter painter(&qimg);

		//		QPen pen;
		//		pen.setWidth(3);
		//		pen.setColor(QColor(0, 255, 127));
		//		painter.setPen(pen);
		//		QBrush brush;
		//		brush.setColor(QColor(0, 255, 127));
		//		painter.setBrush(brush);
		//		QFont font = painter.font();
		//		font.setPointSize(w_segment / 3);
		//		painter.setFont(font);

		//		fout << std::endl;
		//		int total = 0;
		//		int totalCount = 0;

		//		for (int w_idx = 0; w_idx < w_num; w_idx++) {
		//			for (int h_idx = 0; h_idx < h_num; h_idx++) {

		//				int start_x = w_idx * w_segment;
		//				int start_y = h_idx * h_segment;
		//				int end_x = start_x + w_segment;
		//				int end_y = start_y + h_segment;

		//				int avg = 0;
		//				int count = 0;

		//				for (int x = start_x; x < end_x; x++) {
		//					for (int y = start_y; y < end_y; y++) {
		//						if (x >= w || y >= h || x < 0 || y < 0) continue;

		//						avg += hostPtr[x + (y * pitch)];
		//						count++;
		//					}
		//				}

		//				avg = avg / count;
		//				total += avg;
		//				totalCount++;

		//				auto rect = QRectF(start_x, start_y, w_segment, h_segment);
		//				auto textPos = QPointF(start_x, start_y + h_segment);
		//				auto text = QString::number(avg);

		//				painter.fillRect(rect, painter.brush());
		//				painter.drawRect(rect);
		//				painter.drawText(textPos, text);

		//				fout << std::to_string(avg) + ", ";
		//			}
		//		}

		//		fout << std::to_string(total / totalCount);

		//		painter.end();
		//		qimg.save(imagePath);

		//		OpticsControl::instance().toggleGroupedOptic(groupedKey, groupedKeys, false);

		//		incrementProgressBar();
		//	}
		//}

		//progressBarRelease();
	});

	QShortcut *shortcut_ctrlA = new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_A), this);
	connect(shortcut_ctrlA, &QShortcut::activated, [=]() {
		if (notAllowToAccess(AccessLevel::ENGINEER)) return;
		if (notAllowToAccess(AccessLevel::OPERATOR)) return;

		ui.toolButton_analyseGridIntensity->animateClick();
	});

	//copyVisionObject
	QShortcut *shortcut_ctrlC = new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_C), this);
	connect(shortcut_ctrlC, &QShortcut::activated, [=]() { if (notAllowToAccess(AccessLevel::OPERATOR)) return; copyVisionObject(); });

	//pasteVisionObject
	QShortcut *shortcut_ctrlV = new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_V), this);
	connect(shortcut_ctrlV, &QShortcut::activated, [=]() { if (notAllowToAccess(AccessLevel::OPERATOR)) return; pasteVisionObject(); });

	QShortcut *shortcut_ctrlB = new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_B), this);
	connect(shortcut_ctrlB, &QShortcut::activated, [=]() {
		if (notAllowToAccess(AccessLevel::ENGINEER)) return;
		if (notAllowToAccess(AccessLevel::OPERATOR)) return;

		if (_commonDragBox.isVisible()) _commonDragBox.hide();
		else _commonDragBox.show();

	});

	// User management (Admin only)
	QShortcut *shortcut_ctrlU = new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_U), this);
	connect(shortcut_ctrlU, &QShortcut::activated, [=]() {
		if (notAllowToAccess(AccessLevel::ENGINEER)) return;
		if (notAllowToAccess(AccessLevel::OPERATOR)) return;
		openUserManagementDialog();
	});

	QShortcut* shortcut_altM = new QShortcut(QKeySequence(Qt::ALT + Qt::Key_M), this);
	connect(shortcut_altM, &QShortcut::activated, [=]() {

		ct::logger::info("Toggle Display Machine Status - Production Page");
		
		ui.frame_hardwareStatus->setVisible(!ui.frame_hardwareStatus->isVisible());

		});
}


/*
Archive 

// ===> DBSCAN example
ct::logger::info("Start clustering");

		QHash<QString, util::DBRect> rects;
		for (auto& roi : _dragROI) {
			util::DBRect dbrect;
			dbrect.rect = roi->getGeometry();
			rects.insert(roi->getId(), dbrect);
		}

		auto r = _dragROI.at(0)->getGeometry();
		auto eps = r.width() + (r.width() / 2);

		auto clusters = util::dbscan(rects, eps, 4);

		// Generate 1000 distinct colors
		QVector<QColor> colors;
		for (int i = 0; i < clusters.size(); ++i) {
			// Generate RGB values between 0 and 255
			int red = qrand() % 256;
			int green = qrand() % 256;
			int blue = qrand() % 256;

			// Create a QColor object with the generated RGB values
			QColor color(red, green, blue);

			// Add the color to the array
			colors.append(color);
		}

		int colorIdx = 0;

		for (auto& c : clusters) {
			for (auto& id : c) {
				for (auto& roi : _dragROI) {
					if (id == roi->getId()) {
						roi->setBorderColor(colors[colorIdx]);
					}
				}
			}

			colorIdx++;
		}
		processEvents();

		ct::logger::info("Done clustering");

*/

//QString root = "C:\\Atlas\\Data\\Case Studies\\Exposure and Gain Behavior on Lighting Profile\\";

		//auto plotGraph = [&](QString seriesName, const std::array<double, 255>& data, const std::array<double, 255>& data2, QMainWindow& window) {
		//	// Create a line series and populate it with data
		//	QLineSeries *series = new QLineSeries();
		//	series->setName("Golden " + seriesName);

		//	QLineSeries *series2 = new QLineSeries();
		//	series2->setName("Local " + seriesName);

		//	//auto& mgvt = _portabilityInfo.lightingCalibrationInfo.main_GVTable[0];
		//	for (int x = 0; x < data.size(); x++) {
		//		series->append(x, data[x]); // X and Y values
		//	}

		//	for (int x = 0; x < data2.size(); x++) {
		//		series2->append(x, data2[x]); // X and Y values
		//	}

		//	// Create a chart and add the series to it
		//	QChart *chart = new QChart();
		//	chart->addSeries(series);
		//	chart->addSeries(series2);

		//	// Create and set up the X and Y axes
		//	QValueAxis *axisX = new QValueAxis;
		//	axisX->setTitleText("Channel Intensity");
		//	chart->setAxisX(axisX, series);

		//	QValueAxis *axisY = new QValueAxis;
		//	axisY->setTitleText("Brightness");
		//	chart->setAxisY(axisY, series);

		//	// Create a chart view and set the chart on it
		//	QChartView *chartView = new QChartView(chart);
		//	chartView->setRenderHint(QPainter::Antialiasing);

		//	// Set up the main window
		//	window.setCentralWidget(chartView);
		//	window.setGeometry(100, 100, 800, 600);

		//	// Show the main window
		//	window.show();
		//};

		//struct Series {
		//	QString name;
		//	std::array<double, 255>* pData = nullptr;
		//};

		//auto plotMultiSeries = [&](QString title, const std::vector<Series>& series, QMainWindow& window) {
		//	// Create a chart and add the series to it
		//	QChart *chart = new QChart();
		//	chart->setTitle(title);

		//	QValueAxis *customAxis = new QValueAxis;
		//	customAxis->setTitleText("Custom Axis Title");
		//	customAxis->setRange(0, 255);

		//	// Create and set up the X and Y axes
		//	QValueAxis *axisX = new QValueAxis;
		//	axisX->setTitleText("Channel Intensity");
		//	axisX->setRange(0.0, 255.0);
		//	axisX->setTickCount(10);
		//	chart->setAxisX(axisX);

		//	QValueAxis *axisY = new QValueAxis;
		//	axisY->setTitleText("Brightness");
		//	axisY->setRange(0.0, 255.0);
		//	axisY->setTickCount(10);
		//	chart->setAxisY(axisY);

		//	QLineSeries* lastLine = nullptr;
		//	for (auto& s : series) {
		//		if (s.pData == nullptr) continue;

		//		QLineSeries *line = new QLineSeries();
		//		lastLine = line;
		//		line->setName(s.name);

		//		//auto& mgvt = _portabilityInfo.lightingCalibrationInfo.main_GVTable[0];
		//		for (int x = 0; x < s.pData->size(); x++) {
		//			qreal y = s.pData->at(x);
		//			//ct::logger::info("%s-%s[%d] : %f", title.toStdString().c_str(), s.name.toStdString().c_str(), x, y);
		//			
		//			if (x == 254) line->append(x + 1, 255); // X and Y values
		//			else line->append(x + 1, y);
		//		}

		//		chart->addSeries(line);
		//		
		//		//connect(line, &QLineSeries::hovered, this, [](const QPointF &point, bool state) {
		//		//	if (state) { // If the mouse is hovering over the series
		//		//		qreal xValue = point.x(); // Extract the x value
		//		//		qreal yValue = point.y(); // Extract the y value

		//		//		 // Display the x and y values (you can use a tooltip or custom widget)
		//		//		qDebug() << "Series Hovered - X:" << xValue << ", Y:" << yValue;
		//		//	}
		//		//	else {
		//		//		// Clear or hide the tooltip or custom widget when the mouse moves away from the series
		//		//		qDebug() << "Mouse moved away from series";
		//		//	}
		//		//});
		//	}

		//	// Create a chart view and set the chart on it
		//	QChartView *chartView = new QChartView(chart);
		//	chartView->setRenderHint(QPainter::Antialiasing);
		//	chartView->setRubberBand({ QChartView::RectangleRubberBand });

		//	//// Connect the mouse wheel event to a custom slot/function
		//	//connect(chartView, &QChartView::wheelEvent, this, &YourClass::handleWheelEvent);

		//	//// Custom slot/function to handle mouse wheel event
		//	//void YourClass::handleWheelEvent(QWheelEvent *event) {
		//	//	// Determine the direction of the scroll (up or down)
		//	//	int numDegrees = event->angleDelta().y() / 8;
		//	//	int numSteps = numDegrees / 15;

		//	//	// Adjust the visible range of the axes based on the scroll direction
		//	//	if (numSteps > 0) {
		//	//		// Zoom in (decrease visible range)
		//	//		xAxis->setRange(xAxis->min() * 0.9, xAxis->max() * 0.9);
		//	//		yAxis->setRange(yAxis->min() * 0.9, yAxis->max() * 0.9);
		//	//	}
		//	//	else {
		//	//		// Zoom out (increase visible range)
		//	//		xAxis->setRange(xAxis->min() * 1.1, xAxis->max() * 1.1);
		//	//		yAxis->setRange(yAxis->min() * 1.1, yAxis->max() * 1.1);
		//	//	}

		//	//	// Update the range of the axes to reflect the zoomed view
		//	//	chartView->chart()->axisX()->setRange(xAxis->min(), xAxis->max());
		//	//	chartView->chart()->axisY()->setRange(yAxis->min(), yAxis->max());
		//	//}

		//	// Set up the main window
		//	window.setCentralWidget(chartView);
		//	//window.setGeometry(0, 0, 800, 600);
		//	window.setGeometry(100, 100, 1024, 768);

		//	// Show the main window
		//	window.show();
		//};

		//auto loadTable = [&](QString path, GVTable& table) {

		//	QJsonObject root;

		//	if (!loadJson(path, root)) return false;
		//	
		//	if (root.contains("local_gvtable")) {
		//		auto j_GVTable = root["local_gvtable"].toArray();

		//		table.resize(j_GVTable.size());

		//		int channel_index = 0;
		//		for (const auto& gtv : j_GVTable) {

		//			auto& list = gtv.toArray();

		//			for (int i = 0; i < list.size(); i++) {
		//				table[channel_index][i] = list[i].toDouble();
		//			}

		//			channel_index++;
		//		}
		//	}
		//};

		//std::vector<QString> titles;
		//titles.push_back("Red Dome");
		//titles.push_back("Green Dome");
		//titles.push_back("Blue Dome");
		//titles.push_back("Red Ring");
		//titles.push_back("Green Ring");
		//titles.push_back("Blue Ring");
		//titles.push_back("White Ring1");
		//titles.push_back("White Ring2");
		//titles.push_back("White Ring3");
		//titles.push_back("White Ring4");
		//titles.push_back("White Ring5");

		//GVTable g1, g2, g3, g4, g5, g6, g7;

		//loadTable(root + "portability-g1.json", g1);
		//loadTable(root + "portability-g2.json", g2);
		//loadTable(root + "portability-g3.json", g3);
		//loadTable(root + "portability-g4.json", g4);
		//loadTable(root + "portability-g5.json", g5);
		//
		//for (int i = 0; i < 10; i++) {
		//	std::vector<Series> series;
		//	series.emplace_back();
		//	series.emplace_back();
		//	series.emplace_back();
		//	series.emplace_back();
		//	series.emplace_back();

		//	series[0].name = "G1";
		//	series[1].name = "G2";
		//	series[2].name = "G3";
		//	series[3].name = "G4";
		//	series[4].name = "G5";

		//	series[0].pData = &g1[i];
		//	series[1].pData = &g2[i];
		//	series[2].pData = &g3[i];
		//	series[3].pData = &g4[i];
		//	series[4].pData = &g5[i];

		//	QMainWindow* window = new QMainWindow();

		//	plotMultiSeries(titles[i], series, *window);
		//}


		/*loadTable(root + "portability-e14000.json", g1);
		loadTable(root + "portability-e16000.json", g2);
		loadTable(root + "portability-e18000.json", g3);
		loadTable(root + "portability-e20000.json", g4);
		loadTable(root + "portability-e22000.json", g5);
		loadTable(root + "portability-e30000.json", g6);
		loadTable(root + "portability-e50000.json", g7);

		for (int i = 0; i < 10; i++) {
			std::vector<Series> series;
			series.emplace_back();
			series.emplace_back();
			series.emplace_back();
			series.emplace_back();
			series.emplace_back();
			series.emplace_back();
			series.emplace_back();

			series[0].name = "E14000";
			series[1].name = "E16000";
			series[2].name = "E18000";
			series[3].name = "E20000";
			series[4].name = "E22000";
			series[5].name = "E30000";
			series[6].name = "E50000";

			series[0].pData = &g1[i];
			series[1].pData = &g2[i];
			series[2].pData = &g3[i];
			series[3].pData = &g4[i];
			series[4].pData = &g5[i];
			series[5].pData = &g6[i];
			series[6].pData = &g7[i];

			QMainWindow* window = new QMainWindow();

			plotMultiSeries(titles[i], series, *window);
		}*/
