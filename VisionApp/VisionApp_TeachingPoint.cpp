#include "VisionApp.h"
#include "AuditLog.h"

void VisionApp::initTeachPoint()
{
	connect(ui.toolButton_teachPoint, &QToolButton::clicked, this, [=]() { 
		auto& cc = SystemData::instance().currentCoordinate();
		if (_currentTeachPointType == TeachPointType::FIDUCIAL) teachPoint(_currentTeachPointType, _currentFidIndex, cc);
		else if (_currentTeachPointType == TeachPointType::BARCODE) teachPoint(_currentTeachPointType, _currentBarcodeIndex, cc);
	});
	connect(ui.toolButton_jogToTeachPoint, &QToolButton::clicked, this, [=]() { jogToTeachPoint(); });
}

void VisionApp::teachPoint(TeachPointType type, int index, dat::WorldCoordinate point, bool promptPassword)
{
	if (type == TeachPointType::FIDUCIAL)
	{
		if (index >= _fiducialInfos.size()) {
			printf("[UB] Invalid fiducial index: %d\n", index);
			return;
		}

		auto& fid = _fiducialInfos[index];

		if (fid.teach_point.wx > 0.1 && fid.teach_point.wy > 0.1 && fid.teach_point.wz > 0.1 && promptPassword) {
			if (promptQuestion("Teach", "Fiducial position have been teach, confirm to reteach?")) {
				if (!passwordPromptCorrect()) return;
			}
			else {
				return;
			}
		}

		fid.teach_point = point;
		fid.hasTeachPoint = true;

		snapImage(_mainOptics[_camID], "", "");

		QString fileIndex = QString::number(index + 1);
		auto root = Common::Directory::getRecipeImagesPath() + QString("Barcode/");
		Common::Directory::createDir(root);

		auto path = Common::Directory::getRecipeImagesPath() + QString("Fiducial/fid%1.jpg").arg(fileIndex);
		_imageFOV.save(path);

		saveFiducial();
		AuditLog::instance().log(QStringLiteral("TEACH_POINT_FIDUCIAL"), QStringLiteral("index=%1").arg(index));

		showFiducial(index);
	}
	else if (type == TeachPointType::BARCODE)
	{
		if (index > 1)
		{
			printf("[UB] Invalid barcode index: %d\n", index);
			return;
		}

		auto& barcode = _barcodeInfos[index];

		if (barcode.teach_point.wx > 0.1 && barcode.teach_point.wy > 0.1 && barcode.teach_point.wz > 0.1 && promptPassword) {
			if (promptQuestion("Teach", "Barcode position have been teach, confirm to reteach?")) {
				if (!passwordPromptCorrect()) return;
			}
			else {
				return;
			}
		}

		barcode.teach_point = point;
		barcode.hasTeachPoint = true;

		snapImage(_mainOptics[_camID], "", "");

		QString fileIndex = QString::number(index + 1);
		auto root = Common::Directory::getRecipeImagesPath() + QString("Barcode/");
		Common::Directory::createDir(root);

		auto path = Common::Directory::getRecipeImagesPath() + QString("Barcode/barcode%1.jpg").arg(fileIndex);
		_imageFOV.save(path);

		saveBarcode();
		AuditLog::instance().log(QStringLiteral("TEACH_POINT_BARCODE"), QStringLiteral("index=%1").arg(index));

		showBarcode(index);
	}
}

void VisionApp::jogToTeachPoint()
{
	if (_currentTeachPointType == TeachPointType::FIDUCIAL)
	{
		if (_currentFidIndex >= _fiducialInfos.size()) {
			printf("[UB] Invalid fiducial index: %d\n", _currentFidIndex);
			return;
		}

		const auto& fid = _fiducialInfos[_currentFidIndex].teach_point;
		emit jogSnap(fid.wx, fid.wy, fid.wz, _mainOptics[_camID]);
	}
	else if (_currentTeachPointType == TeachPointType::BARCODE)
	{
		if (_currentBarcodeIndex > 1) {
			printf("[UB] Invalid barcode index: %d\n", _currentBarcodeIndex);
			return;
		}

		const auto& barcode = _barcodeInfos[_currentBarcodeIndex].teach_point;
		emit jogSnap(barcode.wx, barcode.wy, barcode.wz, _mainOptics[_camID]);
	}

}

void VisionApp::teachTopleft(dat::WorldCoordinate point)
{
	ui.label_frontLeft->setText(QString("x: %1\ny: %2\nz: %3\n").arg(point.wx).arg(point.wy).arg(point.wz));
	updateSetupCheckList();
	saveRecipe();
}

void VisionApp::teachBtmright(dat::WorldCoordinate point)
{
	ui.label_backRight->setText(QString("x: %1\ny: %2\nz: %3\n").arg(point.wx).arg(point.wy).arg(point.wz));
	updateSetupCheckList();
	saveRecipe();
}
