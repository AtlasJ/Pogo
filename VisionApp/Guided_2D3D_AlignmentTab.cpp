#include "Guided_2D3D_AlignmentTab.h"
#include <QFileInfo>
#include <math.h>
#include <QElapsedTimer>
#include <QDebug>
#include "mil.h"
#include "mtrx.h"
#include "ScaleManager.h"
#include "SystemData.h"
#include "QHostInfo.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

Guided_2D3D_AlignmentTab::Guided_2D3D_AlignmentTab(QWidget *parent)
	: QWidget(parent)
{
	ui.setupUi(this);

	connect(ui.horizontalSlider_toggle2D3D, SIGNAL(valueChanged(int)), this, SLOT(toggle2D3D_Opacity(int)));
	connect(ui.toolButton_jogFront_2, SIGNAL(clicked()), this, SLOT(jogFront()));
	connect(ui.toolButton_jogRight_2, SIGNAL(clicked()), this, SLOT(jogRight()));
	connect(ui.toolButton_jogLeft_2, SIGNAL(clicked()), this, SLOT(jogLeft()));
	connect(ui.toolButton_jogBack_2, SIGNAL(clicked()), this, SLOT(jogBack()));
	connect(ui.toolButton_confirm, SIGNAL(clicked()), this, SLOT(confirmLaserOffset()));

	connect(ui.toolButton_Z_up, SIGNAL(clicked()), this, SLOT(jogUp()));
	connect(ui.toolButton_Z_down, SIGNAL(clicked()), this, SLOT(jogDown()));


	connect(ui.toolButton_confirm_PortabilityOffset, SIGNAL(clicked()), this, SLOT(confirmPortabilityOffset()));

	connect(ui.toolButton_point1, SIGNAL(clicked()), this, SLOT(showPoint1()));
	connect(ui.toolButton_point2, SIGNAL(clicked()), this, SLOT(showPoint2()));
	connect(ui.toolButton_load2dImage, SIGNAL(clicked()), this, SLOT(load2dImage()));
	connect(ui.toolButton_load3dImage, SIGNAL(clicked()), this, SLOT(load3dImage()));
	connect(ui.toolButton_rotateImages, SIGNAL(clicked()), this, SLOT(rotateImages()));
	connect(ui.toolButton_offsetImages, SIGNAL(clicked()), this, SLOT(offsetImages()));

	connect(ui.graphicsView_Aligner, SIGNAL(mouseMove(QPoint)), this, SLOT(mouseMove(QPoint)));

	ui.progressBar_offsetImage->setValue(0);
	ui.progressBar_rotateImage->setValue(0);

	//setup graphics View
	ui.graphicsView_Aligner->setFrameShape(QFrame::NoFrame);
	ui.graphicsView_Aligner->setRenderHint(QPainter::Antialiasing, false);
	ui.graphicsView_Aligner->setDragMode(QGraphicsView::RubberBandDrag);
	ui.graphicsView_Aligner->setOptimizationFlags(QGraphicsView::DontSavePainterState);
	ui.graphicsView_Aligner->setCacheMode(QGraphicsView::CacheBackground);
	ui.graphicsView_Aligner->setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
	ui.graphicsView_Aligner->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);

	//setup graphics scene
	_pGraphicsSceneMain = new QMainGraphicsScene(QRectF());
	_sceneBound =  QRectF(0, 0, 5120, 5120);
	_pGraphicsSceneMain->setSceneRect(_sceneBound);
	ui.graphicsView_Aligner->setScene(_pGraphicsSceneMain);

	//setup 2d pixmap Item
	QPixmap pixmap2d = QPixmap(5120, 5120);
	pixmap2d.fill(QColor(50, 50, 50));
	_pPixmapItem2D = _pGraphicsSceneMain->addPixmap(pixmap2d);
	ui.graphicsView_Aligner->fitInView(_pPixmapItem2D, Qt::KeepAspectRatio);

	//setup 3d pixmap Item
	QPixmap pixmap3d = QPixmap(5120, 5120);
	pixmap3d.fill(QColor(100, 100, 100));
	_pPixmapItem3D = _pGraphicsSceneMain->addPixmap(pixmap3d);
	ui.graphicsView_Aligner->fitInView(_pPixmapItem3D, Qt::KeepAspectRatio);

	//create cross hair for point 1
	_point1Cross = drawCross(QRectF(5120/4, 5120/4, 100, 100), Qt::red);
	_point1Cross->setFlag(QGraphicsItem::ItemIsMovable, true);
	_point1Cross->setFlag(QGraphicsItem::ItemIsSelectable, true);

	//create cross hair for point 2
	_point2Cross = drawCross(QRectF(5120 * 3 / 4, 5120 / 4, 100, 100), Qt::red);
	_point2Cross->setFlag(QGraphicsItem::ItemIsMovable, true);
	_point2Cross->setFlag(QGraphicsItem::ItemIsSelectable, true);


}

Guided_2D3D_AlignmentTab::~Guided_2D3D_AlignmentTab()
{}

void Guided_2D3D_AlignmentTab::reloadAlignmentImages()
{
	QString guidedLaserPath = Common::Directory::LocalPath + "laser/guidedLaser/";

	QString camImgPath = guidedLaserPath + "camImage.png";

	QString imapImgPath = guidedLaserPath + "laser_Intensity.png";

	//if both file exists then proceed to load the images into graphic View
	if (QFileInfo::exists(camImgPath) && QFileInfo::exists(imapImgPath))
	{
		_zOffset = 0;
		_offset = { 0, 0 };
		_pPixmapItem3D->setPos(_offset);

		QImage img2d;
		img2d.load(camImgPath);

		QImage img3d;
		img3d.load(imapImgPath);

		int maxWidth = std::max(img2d.width(), img3d.width());
		int maxHeight = std::max(img2d.height(), img3d.height());

		QPixmap pixmap2d = QPixmap::fromImage(img2d);
		_pPixmapItem2D->setPixmap(pixmap2d);

		QPixmap pixmap3d = QPixmap::fromImage(img3d);
		_pPixmapItem3D->setPixmap(pixmap3d);

		_sceneBound = QRectF(0, 0, maxWidth, maxHeight);
		_pGraphicsSceneMain->setSceneRect(_sceneBound);
		ui.graphicsView_Aligner->setScene(_pGraphicsSceneMain);

		ui.graphicsView_Aligner->show();
		ui.graphicsView_Aligner->resetMatrix();
		ui.graphicsView_Aligner->scale(1, 1);
		QGuiApplication::processEvents();
	}
}

void Guided_2D3D_AlignmentTab::setCurrentLaserOffset(dat::WorldCoordinate offset)
{
	_laserOffset = offset;

	ui.lineEdit_x_2->setText(QString::number(offset.wx));
	ui.lineEdit_y_2->setText(QString::number(offset.wy));
	ui.lineEdit_z_2->setText(QString::number(offset.wz));
}

void Guided_2D3D_AlignmentTab::setCurrentLaserOffsetUI(dat::WorldCoordinate offset)
{
	ui.lineEdit_x_2->setText(QString::number(offset.wx));
	ui.lineEdit_y_2->setText(QString::number(offset.wy));
	ui.lineEdit_z_2->setText(QString::number(offset.wz));
}

void Guided_2D3D_AlignmentTab::setCurrentPositionPortabilityOffset(dat::WorldCoordinate offset)
{
	_positionPortabilityOffset = offset;

	ui.lineEdit_x_portability->setText(QString::number(offset.wx));
	ui.lineEdit_y_portability->setText(QString::number(offset.wy));
	ui.lineEdit_z_portability->setText(QString::number(offset.wz));
}

void Guided_2D3D_AlignmentTab::setCurrentPositionPortabilityOffsetUI(dat::WorldCoordinate offset)
{
	ui.lineEdit_x_portability->setText(QString::number(offset.wx));
	ui.lineEdit_y_portability->setText(QString::number(offset.wy));
	ui.lineEdit_z_portability->setText(QString::number(offset.wz));
}

void Guided_2D3D_AlignmentTab::setMode(AlignmentMode mode)
{
	_mode = mode;
	if (_mode == AlignmentMode::OFFSET_2D3D)
	{
		ui.frameLaserOffset->show();
		ui.framePortabilityOffset->hide();
	}
	else
	{
		ui.frameLaserOffset->hide();
		ui.framePortabilityOffset->show();
	}
}

void Guided_2D3D_AlignmentTab::loadPositionPortabilityImages(QImage& plane, QImage& Vo)
{
	//if both file exists then proceed to load the images into graphic View
	if (!plane.isNull() && !Vo.isNull())
	{
		_zOffset = 0;
		_offset = { 0, 0 };
		_pPixmapItem3D->setPos(_offset);

		int maxWidth = std::max(plane.width(), Vo.width());
		int maxHeight = std::max(plane.height(), Vo.height());

		QPixmap pixmap2d = QPixmap::fromImage(plane);
		_pPixmapItem2D->setPixmap(pixmap2d);

		QPixmap pixmap3d = QPixmap::fromImage(Vo);
		_pPixmapItem3D->setPixmap(pixmap3d);

		_sceneBound = QRectF(0, 0, maxWidth, maxHeight);
		_pGraphicsSceneMain->setSceneRect(_sceneBound);
		ui.graphicsView_Aligner->setScene(_pGraphicsSceneMain);

		ui.graphicsView_Aligner->show();
		ui.graphicsView_Aligner->resetMatrix();
		ui.graphicsView_Aligner->scale(1, 1);
		QGuiApplication::processEvents();
	}
}

void Guided_2D3D_AlignmentTab::jogFront()
{
	double h_scale = ScaleManager::instance().horizontal_um_per_px();
	double v_scale = ScaleManager::instance().vertical_um_per_px();

	_offset = _offset - QPointF(0, ui.lineEdit_stepMM_2->text().toDouble());
	ui.lineEdit_offsetX->setText(QString::number(_offset.x()));
	ui.lineEdit_offsetY->setText(QString::number(_offset.y()));
	_pPixmapItem3D->setPos(_offset);

	if (_mode == AlignmentMode::OFFSET_2D3D)
	{	
		dat::WorldCoordinate newLaserOffset;
		newLaserOffset.wx = _laserOffset.wx - _offset.x() * h_scale / 1000;
		newLaserOffset.wy = _laserOffset.wy - _offset.y() * h_scale / 1000;
		newLaserOffset.wz = _laserOffset.wz + _zOffset;
		setCurrentLaserOffsetUI(newLaserOffset);
	}
	else if (_mode == AlignmentMode::OFFSET_POSITION_PORTABILITY)
	{
		double offsetX = ScaleManager::instance().world_to_fov(_offset.x()) * h_scale / 1000;
		double offsetY = ScaleManager::instance().world_to_fov(_offset.y()) * h_scale / 1000;

		dat::WorldCoordinate newPositionPortabilityOffset;
		newPositionPortabilityOffset.wx = _positionPortabilityOffset.wx + offsetX;
		newPositionPortabilityOffset.wy = _positionPortabilityOffset.wy + offsetY;
		newPositionPortabilityOffset.wz = _positionPortabilityOffset.wz + _zOffset;
		setCurrentPositionPortabilityOffsetUI(newPositionPortabilityOffset);
	}
	
}

void Guided_2D3D_AlignmentTab::jogBack()
{
	double h_scale = ScaleManager::instance().horizontal_um_per_px();
	double v_scale = ScaleManager::instance().vertical_um_per_px();

	_offset = _offset + QPointF(0, ui.lineEdit_stepMM_2->text().toDouble());
	ui.lineEdit_offsetX->setText(QString::number(_offset.x()));
	ui.lineEdit_offsetY->setText(QString::number(_offset.y()));
	_pPixmapItem3D->setPos(_offset);

	if (_mode == AlignmentMode::OFFSET_2D3D)
	{
		dat::WorldCoordinate newLaserOffset;
		newLaserOffset.wx = _laserOffset.wx - _offset.x() * h_scale / 1000;
		newLaserOffset.wy = _laserOffset.wy - _offset.y() * h_scale / 1000;
		newLaserOffset.wz = _laserOffset.wz + _zOffset;
		setCurrentLaserOffsetUI(newLaserOffset);
	}
	else if (_mode == AlignmentMode::OFFSET_POSITION_PORTABILITY)
	{
		double offsetX = ScaleManager::instance().world_to_fov(_offset.x()) * h_scale / 1000;
		double offsetY = ScaleManager::instance().world_to_fov(_offset.y()) * h_scale / 1000;

		dat::WorldCoordinate newPositionPortabilityOffset;
		newPositionPortabilityOffset.wx = _positionPortabilityOffset.wx + offsetX;
		newPositionPortabilityOffset.wy = _positionPortabilityOffset.wy + offsetY;
		newPositionPortabilityOffset.wz = _positionPortabilityOffset.wz + _zOffset;
		setCurrentPositionPortabilityOffsetUI(newPositionPortabilityOffset);
	}
}

void Guided_2D3D_AlignmentTab::jogRight()
{
	double h_scale = ScaleManager::instance().horizontal_um_per_px();
	double v_scale = ScaleManager::instance().vertical_um_per_px();

	_offset = _offset + QPointF(ui.lineEdit_stepMM_2->text().toDouble(), 0);
	ui.lineEdit_offsetX->setText(QString::number(_offset.x()));
	ui.lineEdit_offsetY->setText(QString::number(_offset.y()));
	_pPixmapItem3D->setPos(_offset);

	if (_mode == AlignmentMode::OFFSET_2D3D)
	{

		dat::WorldCoordinate newLaserOffset;
		newLaserOffset.wx = _laserOffset.wx - _offset.x() * h_scale / 1000;
		newLaserOffset.wy = _laserOffset.wy - _offset.y() * h_scale / 1000;
		newLaserOffset.wz = _laserOffset.wz + _zOffset;
		setCurrentLaserOffsetUI(newLaserOffset);
	}
	else if (_mode == AlignmentMode::OFFSET_POSITION_PORTABILITY)
	{
		double offsetX = ScaleManager::instance().world_to_fov(_offset.x()) * h_scale / 1000;
		double offsetY = ScaleManager::instance().world_to_fov(_offset.y()) * h_scale / 1000;

		dat::WorldCoordinate newPositionPortabilityOffset;
		newPositionPortabilityOffset.wx = _positionPortabilityOffset.wx + offsetX;
		newPositionPortabilityOffset.wy = _positionPortabilityOffset.wy + offsetY;
		newPositionPortabilityOffset.wz = _positionPortabilityOffset.wz + _zOffset;
		setCurrentPositionPortabilityOffsetUI(newPositionPortabilityOffset);
	}
}

void Guided_2D3D_AlignmentTab::jogLeft()
{
	double h_scale = ScaleManager::instance().horizontal_um_per_px();
	double v_scale = ScaleManager::instance().vertical_um_per_px();

	_offset = _offset - QPointF(ui.lineEdit_stepMM_2->text().toDouble(), 0);
	ui.lineEdit_offsetX->setText(QString::number(_offset.x()));
	ui.lineEdit_offsetY->setText(QString::number(_offset.y()));
	_pPixmapItem3D->setPos(_offset);

	if (_mode == AlignmentMode::OFFSET_2D3D)
	{
		dat::WorldCoordinate newLaserOffset;
		newLaserOffset.wx = _laserOffset.wx - _offset.x() * h_scale / 1000;
		newLaserOffset.wy = _laserOffset.wy - _offset.y() * h_scale / 1000;
		newLaserOffset.wz = _laserOffset.wz + _zOffset;
		setCurrentLaserOffsetUI(newLaserOffset);
	}
	else if (_mode == AlignmentMode::OFFSET_POSITION_PORTABILITY)
	{
		double offsetX = ScaleManager::instance().world_to_fov(_offset.x()) * h_scale / 1000;
		double offsetY = ScaleManager::instance().world_to_fov(_offset.y()) * h_scale / 1000;

		dat::WorldCoordinate newPositionPortabilityOffset;
		newPositionPortabilityOffset.wx = _positionPortabilityOffset.wx + offsetX;
		newPositionPortabilityOffset.wy = _positionPortabilityOffset.wy + offsetY;
		newPositionPortabilityOffset.wz = _positionPortabilityOffset.wz + _zOffset;
		setCurrentPositionPortabilityOffsetUI(newPositionPortabilityOffset);
	}
}

void Guided_2D3D_AlignmentTab::jogUp()
{
	_zOffset = _zOffset + ui.lineEdit_ZStepmm->text().toDouble();
	if (_mode == AlignmentMode::OFFSET_2D3D)
	{
		dat::WorldCoordinate newLaserOffset;
		newLaserOffset.wx = ui.lineEdit_x_2->text().toDouble();
		newLaserOffset.wy = ui.lineEdit_y_2->text().toDouble();
		newLaserOffset.wz = _laserOffset.wz + _zOffset;
		setCurrentLaserOffsetUI(newLaserOffset);
	}
	else if (_mode == AlignmentMode::OFFSET_POSITION_PORTABILITY)
	{
		dat::WorldCoordinate newPositionPortabilityOffset;
		newPositionPortabilityOffset.wx = ui.lineEdit_x_portability->text().toDouble();
		newPositionPortabilityOffset.wy = ui.lineEdit_y_portability->text().toDouble();
		newPositionPortabilityOffset.wz = _positionPortabilityOffset.wz + _zOffset;
		setCurrentPositionPortabilityOffsetUI(newPositionPortabilityOffset);
	}

	emit motionJogUp(ui.lineEdit_ZStepmm->text().toDouble());
}

void Guided_2D3D_AlignmentTab::jogDown()
{
	_zOffset = _zOffset - ui.lineEdit_ZStepmm->text().toDouble();
	if (_mode == AlignmentMode::OFFSET_2D3D)
	{
		dat::WorldCoordinate newLaserOffset;
		newLaserOffset.wx = ui.lineEdit_x_2->text().toDouble();
		newLaserOffset.wy = ui.lineEdit_y_2->text().toDouble();
		newLaserOffset.wz = _laserOffset.wz + _zOffset;
		setCurrentLaserOffsetUI(newLaserOffset);
	}
	else if (_mode == AlignmentMode::OFFSET_POSITION_PORTABILITY)
	{
		dat::WorldCoordinate newPositionPortabilityOffset;
		newPositionPortabilityOffset.wx = ui.lineEdit_x_portability->text().toDouble();
		newPositionPortabilityOffset.wy = ui.lineEdit_y_portability->text().toDouble();
		newPositionPortabilityOffset.wz = _positionPortabilityOffset.wz + _zOffset;
		setCurrentPositionPortabilityOffsetUI(newPositionPortabilityOffset);
	}

	emit motionJogDown(ui.lineEdit_ZStepmm->text().toDouble());
}

void Guided_2D3D_AlignmentTab::confirmLaserOffset()
{
	//update laser offset
	dat::WorldCoordinate laserOffset;
	laserOffset.wx = ui.lineEdit_x_2->text().toDouble();
	laserOffset.wy = ui.lineEdit_y_2->text().toDouble();
	laserOffset.wz = ui.lineEdit_z_2->text().toDouble();

	emit updateLaserOffset(laserOffset);

}

void Guided_2D3D_AlignmentTab::confirmPortabilityOffset()
{
	//update laser offset
	dat::WorldCoordinate positionPortabilityOffset;
	positionPortabilityOffset.wx = ui.lineEdit_x_portability->text().toDouble();
	positionPortabilityOffset.wy = ui.lineEdit_y_portability->text().toDouble();
	positionPortabilityOffset.wz = ui.lineEdit_z_portability->text().toDouble();

	QString PIC = QInputDialog::getText(this, tr("PIC"), tr("Person In Charge:"));
	SystemData::instance()._portability.current_info.PIC = PIC;
	SystemData::instance()._portability.current_info.date_created = QDateTime::currentDateTime().toString().replace(":", "-");
	SystemData::instance()._portability.current_info.machine_name = QHostInfo::localHostName();

	emit updatePositionPortabilityOffset(positionPortabilityOffset);

}

void Guided_2D3D_AlignmentTab::showPoint1()
{
	if (ui.toolButton_point1->isChecked()) _point1Cross->show();
	else  _point1Cross->hide();
}

void Guided_2D3D_AlignmentTab::showPoint2()
{
	if (ui.toolButton_point2->isChecked()) _point2Cross->show();
	else  _point2Cross->hide();
}

void Guided_2D3D_AlignmentTab::load2dImage()
{
	QFileDialog dialog(this, tr("Open 2D Image File"), Common::Directory::getRecipeImagesPath(), tr("Image Files (*.jpg *.png *.bmp)"));
	// Set the file dialog to allow only single file selection
	dialog.setFileMode(QFileDialog::ExistingFile);

	// Show the file dialog and wait for the user's selection
	if (dialog.exec() == QDialog::Accepted) {
		// Get the selected file path
		QString filePath = dialog.selectedFiles().at(0);

		QImage img2d;
		img2d.load(filePath);

		QPixmap pixmap2d = QPixmap::fromImage(img2d);
		_pPixmapItem2D->setPixmap(pixmap2d);

		_zOffset = 0;
		_offset = { 0, 0 };
		_pPixmapItem3D->setPos(_offset);

		int maxWidth = std::max(img2d.width(), (int)_sceneBound.width());
		int maxHeight = std::max(img2d.height(), (int)_sceneBound.height());

		_sceneBound = QRectF(0, 0, maxWidth, maxHeight);
		_pGraphicsSceneMain->setSceneRect(_sceneBound);
		ui.graphicsView_Aligner->setScene(_pGraphicsSceneMain);

		ui.graphicsView_Aligner->show();
		ui.graphicsView_Aligner->resetMatrix();
		ui.graphicsView_Aligner->scale(1, 1);
		QGuiApplication::processEvents();
	}
}

void Guided_2D3D_AlignmentTab::load3dImage()
{
	QFileDialog dialog(this, tr("Open 3D Image File"), Common::Directory::getRecipeImagesPath(), tr("Image Files (*.jpg *.png *.bmp)"));
	// Set the file dialog to allow only single file selection
	dialog.setFileMode(QFileDialog::ExistingFile);

	// Show the file dialog and wait for the user's selection
	if (dialog.exec() == QDialog::Accepted) {
		// Get the selected file path
		QString filePath = dialog.selectedFiles().at(0);

		QImage img3d;
		img3d.load(filePath);

		QPixmap pixmap3d = QPixmap::fromImage(img3d);
		_pPixmapItem3D->setPixmap(pixmap3d);

		_zOffset = 0;
		_offset = { 0, 0 };
		_pPixmapItem3D->setPos(_offset);

		int maxWidth = std::max(img3d.width(), (int)_sceneBound.width());
		int maxHeight = std::max(img3d.height(), (int)_sceneBound.height());

		_sceneBound = QRectF(0, 0, maxWidth, maxHeight);
		_pGraphicsSceneMain->setSceneRect(_sceneBound);
		ui.graphicsView_Aligner->setScene(_pGraphicsSceneMain);

		ui.graphicsView_Aligner->show();
		ui.graphicsView_Aligner->resetMatrix();
		ui.graphicsView_Aligner->scale(1, 1);
		QGuiApplication::processEvents();
	}
}

void Guided_2D3D_AlignmentTab::rotateImages()
{
	QString imgPath = Common::Directory::getRecipeImagesPath();
	QStringList fileNames = QFileDialog::getOpenFileNames(this, tr("Select images to inspect"), imgPath, "Image File (*.png *.jpg *.jpeg *.bmp *.tiff)");

	if (fileNames.isEmpty()) return;

	// Get rotation angle (replace with your own source if needed)
	double angleDeg = ui.lineEdit_rotation->text().toDouble();  // placeholder

	// Create timestamp ID
	QString timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss_zzz");
	// Create output folder: <first image folder>/rotated
	QFileInfo firstFile(fileNames.first());
	QString outputDirPath = firstFile.absolutePath() + "/rotated_" + timestamp;
	QDir().mkpath(outputDirPath); // create if not exists

	// Set up progress bar
	ui.progressBar_rotateImage->setMinimum(0);
	ui.progressBar_rotateImage->setMaximum(fileNames.size());
	ui.progressBar_rotateImage->setValue(0);

	int count = 0;

	for (const QString& filePath : fileNames)
	{
		QElapsedTimer timer;
		timer.start();

		QImage original(filePath);
		if (original.isNull()) continue;

		int w = original.width();
		int h = original.height();

		// Create a transparent image of the same size
		QImage rotatedSameSize(w, h, original.format());
		rotatedSameSize.fill(Qt::transparent); // optional: fill white or black if needed

		//// Prepare painter and transformation
		//QPainter painter(&rotatedSameSize);
		//QTransform transform;
		//transform.translate(w / 2.0, h / 2.0);   // Move origin to center
		//transform.rotate(angleDeg);             // Rotate
		//transform.translate(-w / 2.0, -h / 2.0); // Move back

		//painter.setTransform(transform);
		//painter.drawImage(0, 0, original);
		//painter.end();

		MIL_ID milImg = MbufRestoreA(filePath.toStdString().c_str(), M_DEFAULT_HOST, M_NULL);
		mtrx::BufferCollector bc_b(milImg);
		if (milImg == M_NULL) {
			qDebug() << "Failed to load image: " << filePath;
			continue;
		}
		

		int width = mtrx::get_width(milImg);
		int height = mtrx::get_height(milImg);
		MimRotate(milImg, milImg, angleDeg, width/2, height/2, width / 2, height / 2, M_BILINEAR + M_OVERSCAN_CLEAR);

		// Compose output path
		QFileInfo fileInfo(filePath);
		QString baseName = fileInfo.completeBaseName();
		QString extension = fileInfo.suffix();
		QString outputPath = QString("%1/%2.%3").arg(outputDirPath, baseName, extension);

		// Save
		MbufExportA(outputPath.toStdString().c_str(), M_JPEG_LOSSY, milImg);
		//rotatedSameSize.save(outputPath);

		qint64 elapsed = timer.elapsed(); // in milliseconds
		qDebug() << "Rotated and saved:" << outputPath << "| Time:" << elapsed << "ms";

		ui.progressBar_rotateImage->setValue(++count);
	}
}

void Guided_2D3D_AlignmentTab::offsetImages()
{
	QPoint offset = QPoint(ui.lineEdit_offsetX->text().toInt(),
		ui.lineEdit_offsetY->text().toInt());

	QString imgPath = Common::Directory::getRecipeImagesPath();
	QStringList fileNames = QFileDialog::getOpenFileNames(this, tr("Select images to inspect"), imgPath, "Image File (*.png *.jpg *.jpeg *.bmp *.tiff)");
	if (fileNames.isEmpty()) return;

	// Create timestamp ID
	QString timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss_zzz");

	// Create output folder: <first image folder>/offset
	QFileInfo firstFile(fileNames.first());
	QString outputDirPath = firstFile.absolutePath() + "/offset_" + timestamp;
	QDir().mkpath(outputDirPath);

	// Set up progress bar
	ui.progressBar_offsetImage->setMinimum(0);
	ui.progressBar_offsetImage->setMaximum(fileNames.size());
	ui.progressBar_offsetImage->setValue(0);

	int count = 0;

	for (const QString& filePath : fileNames)
	{
		QElapsedTimer timer;
		timer.start();

		MIL_ID milImg = MbufRestoreA(filePath.toStdString().c_str(), M_DEFAULT_HOST, M_NULL);
		mtrx::BufferCollector bc_b(milImg);
		if (milImg == M_NULL) {
			qDebug() << "Failed to load image: " << filePath;
			continue;
		}
	
		MimTranslate(milImg, milImg, offset.x(), offset.y(), M_BILINEAR + M_OVERSCAN_CLEAR);
		
		// Compose output path
		QFileInfo fileInfo(filePath);
		QString baseName = fileInfo.completeBaseName();
		QString extension = fileInfo.suffix();
		QString outputPath = QString("%1/%2.%3").arg(outputDirPath, baseName, extension);

		MbufExportA(outputPath.toStdString().c_str(), M_JPEG_LOSSY, milImg);

		/*MIL_ID croppedBuf = MbufChild2d(milImg, 0, 0, 3480, 2112, M_NULL);
		MbufSaveA(outputPath.toStdString().c_str(), croppedBuf);
		MbufFree(croppedBuf);*/

		qint64 elapsed = timer.elapsed();
		qDebug() << "Offset and saved:" << outputPath << "| Time:" << elapsed << "ms";

		ui.progressBar_offsetImage->setValue(++count);
	}
}

QCrossItem* Guided_2D3D_AlignmentTab::drawCross(const QRectF& rect, const QColor& color)
{
	QCrossItem* pShape = new QCrossItem();

	_pGraphicsSceneMain->addItem(pShape);

	pShape->setup(rect, color);

	return pShape;
}

void Guided_2D3D_AlignmentTab::mouseMove(QPoint point)
{
	double point1_x = _point1Cross->pos().x() + _point1Cross->boundingRect().width() / 2;
	double point1_y = _point1Cross->pos().y() + _point1Cross->boundingRect().height() / 2;
	ui.lineEdit_point1_x->setText(QString::number(point1_x));
	ui.lineEdit_point1_y->setText(QString::number(point1_y));

	double point2_x = _point2Cross->pos().x() + _point2Cross->boundingRect().width() / 2;
	double point2_y = _point2Cross->pos().y() + _point2Cross->boundingRect().height() / 2;
	ui.lineEdit_point2_x->setText(QString::number(point2_x));
	ui.lineEdit_point2_y->setText(QString::number(point2_y));

	//get rotation value
	double dx = point2_x - point1_x;
	double dy = point2_y - point1_y;

	// Calculate angle in radians and convert to degrees
	double angleRad = atan2(dy, dx);
	double angleDeg = angleRad * (180.0 / M_PI);

	// Normalize to [0, 360)
	/*if (angleDeg < 0)
		angleDeg += 360.0;*/

	ui.lineEdit_rotation->setText(QString::number(angleDeg));
}

void Guided_2D3D_AlignmentTab::toggle2D3D_Opacity(int value)
{
	double opacity = (double)value / 100.0;
	_pPixmapItem3D->setOpacity(opacity);
}
