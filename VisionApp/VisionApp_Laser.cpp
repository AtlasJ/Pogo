#include "VisionApp.h"
#include "CAMManager.h"
#include "ScaleManager.h"
#include "Guided_2D3D_AlignmentTab.h"

void VisionApp::initLaserUI()
{
	connect(ui.toolButton_verifyLaserAlignment, &QToolButton::clicked, this, [=]() { verifyLaserAlignment(); });
	connect(ui.toolButton_alignLaser, &QToolButton::clicked, this, [=]() { alignCameraAndLaser(); });
	connect(ui.toolButton_manualAlignLaser, &QToolButton::clicked, this, [=]() { guidedAlignCameraAndLaser(); });
	connect(ui.toolButton_jogToLaserView, &QToolButton::clicked, this, [=]() { 
		auto& cc = SystemData::instance().currentCoordinate();
		jogToLaserView(cc.wx, cc.wy, cc.wz, "2D"); 
		snapImage(_mainOptics[_camID], "", "");
	});
	connect(ui.toolButton_jogToCamView, &QToolButton::clicked, this, [=]() {
		auto& cc = SystemData::instance().currentCoordinate();
		jogToCamView(cc.wx, cc.wy, cc.wz, "2D"); 
		snapImage(_mainOptics[_camID], "", "");
	});
	connect(ui.toolButton_setCircleDiameter, &QToolButton::clicked, this, [=]() { toggleCommonDragBox(ui.toolButton_setCircleDiameter); });
	connect(ui.toolButton_prevLaserAlignmentImage, &QToolButton::clicked, this, [=]() { 
		_currentLaserAlignmentIndex--;
		if (_laserAlignmentImages.isEmpty()) return;
		if (_currentLaserAlignmentIndex < 0) _currentLaserAlignmentIndex = _laserAlignmentImages.size() - 1;
		if (_currentLaserAlignmentIndex >= _laserAlignmentImages.size()) _currentLaserAlignmentIndex = 0;

		auto& l = _laserAlignmentImages[_currentLaserAlignmentIndex];
		QImage scaledImage = l.qimg.scaled(380, 380, Qt::KeepAspectRatio);
		ui.label_laserAlignmentImage->setPixmap(QPixmap::fromImage(scaledImage));
		ui.label_laserAlignmentStatus->setText(l.info);
	});
	connect(ui.toolButton_nextLaserAlignmentImage, &QToolButton::clicked, this, [=]() { 
		_currentLaserAlignmentIndex++;
		if (_laserAlignmentImages.isEmpty()) return;
		if (_currentLaserAlignmentIndex < 0) _currentLaserAlignmentIndex = _laserAlignmentImages.size() - 1;
		if (_currentLaserAlignmentIndex >= _laserAlignmentImages.size()) _currentLaserAlignmentIndex = 0;

		auto& l = _laserAlignmentImages[_currentLaserAlignmentIndex];
		QImage scaledImage = l.qimg.scaled(380, 380, Qt::KeepAspectRatio);
		ui.label_laserAlignmentImage->setPixmap(QPixmap::fromImage(scaledImage));
		ui.label_laserAlignmentStatus->setText(l.info);
	});


	//warpage
	connect(ui.toolButton_generateWarpageMap, &QToolButton::clicked, this, [=]() { 
		//TODO: JObthread
	});
}

int VisionApp::getScanOrientation(const dat::WorldCoordinate & start, const dat::WorldCoordinate & end)
{
	//if (start.wx < end.wx) return GO_ORIENTATION_REVERSE;
	//return GO_ORIENTATION_WIDE;
	return 0;
}

QImage VisionApp::get3DImage(QImage & qimg)
{
	auto rotated = util::rotateQImage(qimg, -90);
	auto cam_w = CAMManager::instance().getWidth(_camID);
	auto cam_h = CAMManager::instance().getHeight(_camID);
	auto scale = rotated.scaled(QSize(cam_w, cam_h), Qt::KeepAspectRatio, Qt::SmoothTransformation);
	return scale;
}

MIL_ID VisionApp::get3DImage(MIL_ID mbuf)
{
	if (mbuf == M_NULL) return M_NULL;

	auto w = mtrx::get_width(mbuf);
	auto h = mtrx::get_height(mbuf);

	ct::logger::debug("[SpeedUp] Initial size: %d, %d", h, w);

	auto scale = ScaleManager::instance().um_per_px();

	MIL_INT sw = w * 7 / scale;
	MIL_INT sh = h * 7 / scale;

	ct::logger::debug("[SpeedUp] Scaled size: %d, %d", sh, sw);

	auto type = mtrx::get_type(mbuf);

	auto mRotate = MbufAlloc2d(M_DEFAULT, h, w, type + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);
	auto mScaled = MbufAlloc2d(M_DEFAULT, sh, sw, type + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);
	MimRotate(mbuf, mRotate, 90, w / 2, h / 2, h / 2, w / 2, M_BICUBIC + M_OVERSCAN_ENABLE);
	MimRotate(mRotate, mRotate, (-_fiducial.getAngle()), M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_BICUBIC + M_OVERSCAN_CLEAR);
	MimResize(mRotate, mScaled, M_FILL_DESTINATION, M_FILL_DESTINATION, M_BICUBIC + M_OVERSCAN_ENABLE);
	//warpPerspective(mScaled);
	mtrx::BufferCollector bc(mRotate);


	return mScaled;
}

void VisionApp::warpPerspective(MIL_ID & milImg)
{
	if (milImg == M_NULL) return;
	// Define four points on the output image
	cv::Point2f output_points[] = { cv::Point2f(357, 235), cv::Point2f(875, 235), cv::Point2f(357, 629), cv::Point2f(875, 629) };

	// Define four points on the input image
	cv::Point2f input_points[] = { cv::Point2f(341, 220), cv::Point2f(863, 220), cv::Point2f(341, 619), cv::Point2f(863, 619) };

	// Find the transformation matrix
	cv::Mat M = cv::getPerspectiveTransform(input_points, output_points);

	cv::Mat cvImg;
	util::Mil_to_cv(milImg, cvImg);

	//MbufSaveA("TransformedBefore.tiff", milImg);

	cv::warpPerspective(cvImg, cvImg, M, cvImg.size());

	util::cv_to_Mil(cvImg, milImg);
	//MbufSaveA("TransformedBefore.tiff", milImg);
}

void VisionApp::alignCameraAndLaser()
{
	auto camThreshold = ui.lineEdit_cameraThreshold->text().toInt();
	auto laserThreshold = ui.lineEdit_laserThreshold->text().toInt();
	emit performLaserAlignment(SystemData::instance().currentCoordinate(), _commonDragBox.getGeometry(), camThreshold, laserThreshold);
}

void VisionApp::enhancedAlignCameraAndLaser()
{
	auto camThreshold = ui.lineEdit_cameraThreshold->text().toInt();
	auto laserThreshold = ui.lineEdit_laserThreshold->text().toInt();
	emit captureAlignmentImages(SystemData::instance().currentCoordinate(), camThreshold, laserThreshold);
}

void VisionApp::guidedAlignCameraAndLaser()
{
	emit performGuidedLaserAlignment(SystemData::instance().currentCoordinate());
}

void VisionApp::guidedAlignCameraAndLaserSetup()
{
	//load 2d and 3d image into graphics View, create two graphic items where 3d is overlayed on top of 2d image
	_guided_2D3D_AlignmentTab->reloadAlignmentImages();
	//set current laser offset
	_guided_2D3D_AlignmentTab->setCurrentLaserOffset(_laserConfig.offset);
	//able to manual move 3d image to map it nicely on the 2d image
	//or can set points for 2d and points for 3d, then click confirm to map them
	_guided_2D3D_AlignmentTab->setMode(Guided_2D3D_AlignmentTab::AlignmentMode::OFFSET_2D3D);
	_guided_2D3D_AlignmentTab->show();
}

void VisionApp::verifyLaserAlignment()
{
	_laserAlignmentImages.clear();
	emit signalVerifyLaserAlignment(SystemData::instance().currentCoordinate());
}

void VisionApp::drawCrossOnQImage(int cx, int cy, int size, QImage & qimg)
{
	QPainter painter(&qimg);
	QPen pen(Qt::red);
	pen.setWidth(30);
	painter.setPen(Qt::red);

	// Draw a horizontal line (cross)
	painter.drawLine(cx - size, cy, cx + size, cy);

	// Draw a vertical line (cross)
	painter.drawLine(cx, cy - size, cx, cy + size);

	// End the painting
	painter.end();
}

void VisionApp::saveQImageWithCrossSection(QImage qimg, QString path)
{
	auto cxh = qimg.width() / 2;
	auto cyh = qimg.height() / 2;
	drawCrossOnQImage(cxh, cyh, 60, qimg);

	qimg.save(path);
}

void VisionApp::displayCurrentAlignmentImage()
{
	if (_laserAlignmentImages.isEmpty()) return;
	if (_laserAlignmentImages.size() <= _currentLaserAlignmentIndex) _currentLaserAlignmentIndex = 0;

	auto& l = _laserAlignmentImages[_currentLaserAlignmentIndex];
	QImage scaledImage = l.qimg.scaled(380, 380, Qt::KeepAspectRatio);
	ui.label_laserAlignmentImage->setPixmap(QPixmap::fromImage(scaledImage));
	ui.label_laserAlignmentStatus->setText(l.info);
}

void VisionApp::scanDone() //WARNING: Dont put UI related stuff here. Emit it thru signal slot
{
	ct::logger::debug("Scan done");
	ct::logger::debug("DONE SCAN ACTIONS");
}

void VisionApp::terminated(int signum)
{
	MachineController::instance().turnOnBrake();

	std::string messageCode = "Unknown";
	if (signum == 2) messageCode = "Process Interrupted";
	else if (signum == 4) messageCode = "Illegal Instruction";
	else if (signum == 6) messageCode = "Abort Signal Received";
	else if (signum == 8) messageCode = "Floating point exception (Arithmetic logic error)";
	else if (signum == 11) messageCode = "Segment Violation (Invalid memory access)";
	else if (signum == 15) messageCode = "Kill Signal Received";
	else if (signum == 21) messageCode = "Ctrl Break Sequence";
	else if (signum == 22) messageCode = "Software Abort";

	if (signum == 22) ct::logger::info("Terminate Signal %d: %s", signum, messageCode.c_str());
	else ct::logger::error("Terminate Signal %d: %s", signum, messageCode.c_str());

	if (signum != 999) exit(signum);
}
