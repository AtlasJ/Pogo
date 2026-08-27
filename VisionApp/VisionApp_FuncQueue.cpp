#include "VisionApp.h"
#include "uidGenerator.h"
#include "ImagePathManager.h"
#include "cvUtil.h"

void VisionApp::collectImages()
{
	if (_enable2D && _enable3D) {
		collect2D3DView(getNextSamplePath());
	}
	else if (_enable2D) {
		collect2DView(getNextSamplePath());
	}
	else if (_enable3D) {
		collect3DView(getNextSamplePath());
	}
}

void VisionApp::collect2DView(QString rootPath)
{
	toggleDualView();

	_imageManager.attach(&_views, &_recipeOptics);
	_imageManager.attach(&_lineScans, &_recipeOptics3D);
	_imageManager.reset();
	clear2DImages(rootPath);

	SystemData::instance()._workingPath = rootPath;
	_processType = ProcessType::IMAGE_COLLECTION;

	_jobThread.setRootPath(rootPath);
	QMetaObject::invokeMethod(&_jobThread, "run2D", Qt::QueuedConnection);
	return;
}

void VisionApp::collect3DView(QString rootPath)
{
	toggleDualView();

	_imageManager.attach(&_views, &_recipeOptics);
	_imageManager.attach(&_lineScans, &_recipeOptics3D);
	_imageManager.reset();
	clear3DImages(rootPath);

	SystemData::instance()._workingPath = rootPath;
	_processType = ProcessType::IMAGE_COLLECTION;

	_jobThread.setRootPath(rootPath);
	QMetaObject::invokeMethod(&_jobThread, "run3D", Qt::QueuedConnection);
	return;
}

void VisionApp::collect2D3DView(QString rootPath)
{
	toggleDualView();

	_imageManager.attach(&_views, &_recipeOptics);
	_imageManager.attach(&_lineScans, &_recipeOptics3D);
	_imageManager.reset();
	clear2DImages(rootPath);
	clear3DImages(rootPath);

	SystemData::instance()._workingPath = rootPath;
	_processType = ProcessType::IMAGE_COLLECTION;

	_jobThread.setRootPath(rootPath);
	QMetaObject::invokeMethod(&_jobThread, "run2D3D", Qt::QueuedConnection);
	return;
}

void VisionApp::inspect2D3D()
{
	//toggleDualView();

	int numView = 0;
	int multiplier = 3;

	auto& sd = SystemData::instance();

	if (sd._setupRegionPitchMode) {
		//pitch mode: one step per unit for the barcode cycle, its OCR result,
		//the 3D scan, and its height result (skipped steps are credited by the flow)
		const int units = std::max(1, (int)sd._unitsX) * std::max(1, (int)sd._unitsY);
		if (sd._pitchEnableBarcode) numView += units * 2;
		if (sd._pitchEnable3D) numView += units * 2;
		if (numView == 0) numView = 1;
	}
	else if (_enable2D && _enable3D) numView = getNumOfViewToProcess() + getNumOfLineScanToProcess();
	else if (_enable2D)  numView = getNumOfViewToProcess();
	else if (_enable3D)  numView = getNumOfLineScanToProcess();

	if (!sd._setupRegionPitchMode && ui.checkBox_runOneFOVonly->isChecked()) numView = 2;

	progressBarSetup("Running Inspection...", numView, true);

	_imageManager.attach(&_views, &_recipeOptics);
	_imageManager.attach(&_lineScans, &_recipeOptics3D);
	_imageManager.reset();

	_processType = ProcessType::PRODUCTION;

	//setupProductionDir();
	
	if (_enable2D && _enable3D)	QMetaObject::invokeMethod(&_jobThread, "run2D3D", Qt::QueuedConnection);
	else if (_enable2D) QMetaObject::invokeMethod(&_jobThread, "run2D", Qt::QueuedConnection);
	else if (_enable3D)	QMetaObject::invokeMethod(&_jobThread, "run3D", Qt::QueuedConnection);
	return;
}