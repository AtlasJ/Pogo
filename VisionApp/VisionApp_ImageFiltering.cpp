#include "VisionApp.h"

void VisionApp::initImageFiltering()
{
	connect(ui.toolButton_loadVIDIImages, &QToolButton::clicked, this, [=]() { 
		QString folderPath = QFileDialog::getExistingDirectory(nullptr, "Select Folder", Common::Directory::getRecipeVidiImagePath());

		ui.lineEdit_vidiImagePath->setText(folderPath);

		if (!folderPath.isEmpty()) {
			QDir directory(folderPath);
			QStringList imageFilters;
			imageFilters << "*.png" << "*.jpg" << "*.jpeg" << "*.bmp"; // Add more image formats if needed
			QStringList files = directory.entryList(imageFilters, QDir::Files);
		
			progressBarSetup("Loading Images...", files.size());

			resetFilterInfo();

			int index = 0;
			for (auto f : files) {

				auto path = folderPath + "/" + f;
				ct::logger::trace("Loading: %s", path.toStdString().c_str());

				ImageFilterInfo ifi;
				ifi.currentPath = path;

				QFileInfo fileInfo(path);
				ifi.baseName = fileInfo.baseName();

				ifi.qimg.load(path);
				//MbufAllocColor(M_DEFAULT, 3, ifi.qimg.width(), ifi.qimg.height(), 8 + M_UNSIGNED, M_IMAGE + M_PROC, &ifi.mbuf);
				//MbufLoadA(path.toStdString().c_str(), ifi.mbuf);
				ifi.mbuf = mtrx::to_milID(ifi.qimg);

				_unfilteredImages.insert(index);
				_imagesToFilter.append(ifi);

				incrementProgressBar();
				index++;
			}

			toggleFOVView();
			_filterIndex = 0;

			ui.lineEdit_filterStatus->setText(QString::number(_unfilteredImages.size()));
			ui.progressBar_filterStatus->setMaximum(_unfilteredImages.size());
			ui.progressBar_filterStatus->setValue(0);

			displayFOV(_imagesToFilter[_filterIndex].qimg);
			
			//align drag box to current image size
			_commonDragBox.setGeometry(QRectF(0, 0, _pixmapFOV.width(), _pixmapFOV.height()));

			_editMode = EditMode::IMAGE_FILTERING;

			QDir dir(Common::Directory::getRecipeVidiImageGoodPath());
			if (!dir.exists()) dir.mkpath(Common::Directory::getRecipeVidiImageGoodPath());

			QDir dir2(Common::Directory::getRecipeVidiImageBadPath());
			if (!dir2.exists()) dir2.mkpath(Common::Directory::getRecipeVidiImageBadPath());
		}

	});

	connect(ui.checkBox_vidiSimilarity, &QCheckBox::stateChanged, this, [=](int state) {
		auto check = (bool)state;

		if (check) {
			_commonDragBox.show();
		}
		else {
			_commonDragBox.hide();
		}
		
	});

	connect(ui.toolButton_stopVIDIFiltering, &QToolButton::clicked, this, [=]() {
		resetFilterInfo();
		_editMode = EditMode::SELECT;
		ui.lineEdit_filterStatus->setText(QString::number(_unfilteredImages.size()));
		ui.progressBar_filterStatus->setValue(0);
	});
}

void VisionApp::resetFilterInfo()
{
	_unfilteredImages.clear();
	_lastFilteredImages.clear();
	_filterIndex = 0;
	for (auto& f : _imagesToFilter) {
		mtrx::free_buffer(f.mbuf);
	}
	_imagesToFilter.clear();
}

void VisionApp::filterImage(QString root) {
	auto& currentImage = _imagesToFilter[_filterIndex];

	_lastFilteredImages.clear();

	auto dstPath = root + currentImage.baseName + ".jpg";
	QFile::rename(currentImage.currentPath, dstPath); //move file
	currentImage.currentPath = dstPath;
	_lastFilteredImages.append(_filterIndex);


	if (ui.checkBox_vidiSimilarity->isChecked()) {
		//using current image to further filter good and bad
		auto score = ui.lineEdit_vidiSimilarityScore->text().toDouble();

		auto sx = _commonDragBox.getGeometry().x();
		auto sy = _commonDragBox.getGeometry().y();
		auto sw = _commonDragBox.getGeometry().width();
		auto sh = _commonDragBox.getGeometry().height();


		MIL_ID mMono = mtrx::to_mono(currentImage.mbuf);
		MIL_ID mCrop = MbufChild2d(mMono, sx, sy, sw, sh, M_NULL);
		mtrx::BufferCollector bc(mMono);
		mtrx::BufferCollector bc2(mCrop);
		mtrx::PatternInput input;
		mtrx::PatternOutput output;

		auto w = mtrx::get_width(mCrop);
		auto h = mtrx::get_height(mCrop);
		input.filename = "temp.pat";
		input.learn_x = 0;
		input.learn_y = 0;
		input.learn_w = w;
		input.learn_h = h;
		input.min_score = score;
		input.angle_step = 0;
		mtrx::learn_pattern(mCrop, input, output);

		progressBarSetup("Analysing...", _unfilteredImages.size());
		
		for (auto u : _unfilteredImages) {

			auto & uIF = _imagesToFilter[u];

			MIL_ID mMonoU = mtrx::to_mono(uIF.mbuf);
			MIL_ID mCropU = MbufChild2d(mMonoU, sx, sy, sw, sh, M_NULL);
			mtrx::BufferCollector bc(mMonoU);
			mtrx::BufferCollector bc2(mCropU);

			if (mtrx::find_pattern(mCropU, input.filename, output)) {
				if (output.score >= score) {
					dstPath = root + uIF.baseName + ".jpg";

					QFile::rename(uIF.currentPath, dstPath);
					uIF.currentPath = dstPath;

					_lastFilteredImages.append(u);
				}
			}

			incrementProgressBar();
		}

		progressBarRelease();
	}

	for (auto f : _lastFilteredImages) {
		_unfilteredImages.erase(f);
	}


	ui.lineEdit_filterStatus->setText(QString::number(_unfilteredImages.size()));
	ui.progressBar_filterStatus->setValue(ui.progressBar_filterStatus->maximum() - _unfilteredImages.size());


	if (!_unfilteredImages.empty()) {
		_filterIndex = *_unfilteredImages.begin();
		auto& nextImage = _imagesToFilter[_filterIndex];
		displayFOV(nextImage.qimg);
	}
	else {
		_editMode = EditMode::SELECT;
	}
}