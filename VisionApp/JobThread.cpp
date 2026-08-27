#include "JobThread.h"
#include "SRXManager.h"
#include "InspectionThread.h"
#include "TimeLogger.h"
#include "ScopedTimeLogger.h"
#include "Utilities.h"
#include "Logger.h"
#include "OpticsControl.h"
#include "CommonDir.h"
#include <QThreadPool>
#include "cvUtil.h"
#include "ImagePathManager.h"
#include <QHostInfo>
#include <QDir>
#include <QFile>
#include <QDateTime>
#include <QTextStream>
#include "SystemData.h"
#include "CAMManager.h"
#include "ProfilerManager.h"
#include <algorithm>
#include <cmath>
#include <deque>
#include "QOSTool.h"
#include "ScaleManager.h"

#include "ImageSavingThread.h"

#include "MotionController.h"
#include "MachineController.h"

#include "ProjectBased_Definition.h"

extern QString g_imgExtension;
extern TMessageQue<FrameInfo> g_imageQueue;

const QString WAIT_IPP = "WAIT_IPP";

void JobThread::run()
{
	ct::logger::info("[QThread] Job thread started");
	m_server = new QServer();
	m_server->run(QHostAddress("127.0.0.1"), 700);
	connect(m_server, SIGNAL(dataReceived(QByteArray)), this, SLOT(incomingJob(QByteArray)));
	connect(this, &JobThread::imageReady, this, &JobThread::processImageReady);
	connect(this, &JobThread::imagePreprocessed, this, &JobThread::processImagePreprocessed);

	exec();

	ct::logger::info("[QThread] Job thread stopped");
}

void JobThread::release()
{
	quit();  // Exits the event loop
}

void JobThread::attach(QListWidget* viewSequence)
{
	m_viewSequence = viewSequence;
}

void JobThread::attach(Fiducial* fiducialAlgo)
{
	m_fiducialAlgo = fiducialAlgo;
}

void JobThread::attach2ndFiducial(Fiducial* fiducialAlgo)
{
	m_fiducialAlgo2 = fiducialAlgo;
}

void JobThread::attach(std::vector<FiducialInfo>* fiducialInfo)
{
	m_fiducialInfos = fiducialInfo;
}

void JobThread::attach(std::array<BarcodeInfo, 2>* barcodeInfos)
{
	m_barcodeInfos = barcodeInfos;
}

void JobThread::attach(InspStatus* inspStatus)
{
	m_inspStatus = inspStatus;
}

void JobThread::attach(CSAInfo* csa)
{
	m_csa = csa;
}

void JobThread::attach(QHash<QString, QView>* views, QHash<QString, OpticsInfo>* optics2D)
{
	m_views = views;
	m_optics = optics2D;
}

void JobThread::attach(QHash<QString, QLineScan>* linescans, QHash<QString, OpticsInfo3D>* optics3D)
{
	m_linescans = linescans;
	m_optics3D = optics3D;
}

void JobThread::attach(dat::WorldCoordinate* laserOffset)
{
	m_laserOffset = laserOffset;
}

void JobThread::attach(QViewPlane* viewPlane)
{
	m_viewPlane = viewPlane;
}

void JobThread::attach(PortabilityInfo* portabilityInfo)
{
	m_portabilityInfo = portabilityInfo;
}

void JobThread::setRootPath(QString rootPath)
{
	m_rootPath = rootPath;
	m_index = 0;
}

void JobThread::setWarpageMethod(QString method)
{
	m_warpageMethod = method;
}

void JobThread::setXDecel(int decel)
{
	m_xDecel = decel;
}

void JobThread::setXSpeed(int speed, int speed3d)
{
	m_xSpeed = speed;
	m_xSpeed3d = speed3d;
}

void JobThread::getXSpeed(int& speed, int& speed3d)
{
	speed = m_xSpeed;
	speed3d = m_xSpeed3d;
}


void JobThread::enableFiducial(bool enable)
{
	m_enableFiducial = enable;
}

void JobThread::enableBarcode(bool enable)
{
	m_enableBarcode = enable;
}

void JobThread::enableWarpageCompensation(bool enable)
{
	m_enableWarpageCompensation = enable;
}

void JobThread::enableRun1stFOVOnly(bool enable)
{
	m_run1stFOVOnly = enable;
}

void JobThread::resetFiducial()
{
	m_locatedFidID.clear();
	m_fiducialAlgo->reset();
	if (m_fiducialAlgo2) m_fiducialAlgo2->reset();
}

void JobThread::switchToContinuousModeLSC() {
	m_lscFastMode = false;
	//restore the configured default mode (strobe if enabled in config)
	LSCManager::instance().setMode(SystemData::instance()._lscStrobeMode ? lsc::MODE::TRIGGER : lsc::MODE::CONTINUOUS);
	LSCManager::instance().resetLatch();
}

void JobThread::switchToFastModeLSC() {
	m_lscFastMode = true;
	for (auto& v : (*m_views)) {
		QVector<LSCManager::SequenceData> datas;
		int triggerSource = 1;

		for (auto& opticID : v.opticIDs) {

			auto& optic = (*m_optics)[opticID];

			//each setting
			auto exposure = optic.exposure;

			LSCManager::SequenceData data;
			data.exposure_us = exposure;

			if (optic.type == ct::s_mono) {
				data.band = optic.M;
				datas.push_back(data);
			}
			else {
				data.band = optic.R;
				datas.push_back(data);
				data.band = optic.G;
				datas.push_back(data);
				data.band = optic.B;
				datas.push_back(data);
			}
		}

		LSCManager::instance().setTriggerSequence(datas);

		break;
	}

	LSCManager::instance().setMode(lsc::MODE::TRIGGER);
}

const OpticsInfo& JobThread::getMainOptics()
{
	for (const auto& optic : *m_optics) {
		if (optic.tag == "Main") {
			return optic;
		}
	}

	if (m_optics->size()) {
		return (*m_optics)[0];
	}
	else {
		ct::logger::error("[JobThread] Unable to find main optics, apps will crash now :)");
	}
}

const OpticsInfo3D& JobThread::getMainOptics3D()
{
	for (const auto& optic : *m_optics3D) {
		if (optic.intensity) {
			return optic;
		}
	}
}

void JobThread::snapBand(const OpticsInfo& optic, QString viewID, QString stitchID, BandType bandType)
{
	if (!CAMManager::instance().isConnected(optic.camID)) {
		ct::logger::error("[Acq] snapBand skipped: camera '%s' is not connected", optic.camID.toStdString().c_str());
		return;
	}

	MachineController::instance().trackTime("Snap + Light");

	TimeLogger timer;

	//strobe mode: pulse width must cover the camera exposure. Uses the saved
	//exposure state (no camera API round trip); latched inside, no-op unless
	//the LSC is in strobe mode.
	LSCManager::instance().setStrobePulseWidth((int)CAMManager::instance().currentExposure(optic.camID));

	if (m_lscFastMode) {
		CAMManager::instance().setDO(m_camID, m_camTriggerIO, true);
	}
	else {
		OpticsControl::instance().setBand(optic.camID, optic, bandType);
	}

	if (bandType != BandType::M) CAMManager::instance().frame(optic.camID)->postTask.combineRGB = true;
	else CAMManager::instance().frame(optic.camID)->postTask.combineRGB = false;
	CAMManager::instance().frame(optic.camID)->viewID = viewID;
	CAMManager::instance().frame(optic.camID)->stitchID = stitchID;
	CAMManager::instance().frame(optic.camID)->postTask.bandType = bandType;
	CAMManager::instance().frame(optic.camID)->opticID = optic.id;
	CAMManager::instance().frame(optic.camID)->type = optic.type;
	CAMManager::instance().frame(optic.camID)->postTask.rotationalAngle = SystemData::instance()._camAngles[optic.camID];

	timer.log_duration("Set channels");

	MachineController::instance().trackTime("Snap");
	CAMManager::instance().softTrigger(optic.camID);
	CAMManager::instance().waitAcquisition(optic.camID, 5000);
	MachineController::instance().logTime("Snap");

	timer.log_duration("Snap image");

	if (m_lscFastMode) {
		CAMManager::instance().setDO(m_camID, m_camTriggerIO, false);
	}
	else {
		const auto& band = OpticsControl::instance().getBand(optic, bandType);
		OpticsControl::instance().toggleBand(band, false);
	}

	timer.log_duration("Turn off channels");
	MachineController::instance().logTime("Snap + Light");
}

void JobThread::snapOptic(const OpticsInfo& optic, QString viewID, QString stitchID, bool resetFrame)
{
	if (SystemData::instance()._psp) {
		if (SystemData::instance()._snapDelay_ms != 0) os_tool::doNothing(SystemData::instance()._snapDelay_ms);
		SystemData::instance().triggerPSP();
		return;
	}

	QString camID = optic.camID;

	if (!CAMManager::instance().isConnected(camID)) {
		ct::logger::error("[Acq] snapOptic skipped: camera '%s' is not connected", camID.toStdString().c_str());
		return;
	}

	if (resetFrame) CAMManager::instance().resetFrame(camID);

	if (m_rgbOverrides.contains(optic.id)) {
		CAMManager::instance().frame(camID)->postTask.rgbOffset = m_rgbOverrides[optic.id];
	}
	else {
		CAMManager::instance().frame(camID)->postTask.rgbOffset = RGBOffset();
	}

	if (optic.type == ct::s_color && CAMManager::instance().getChannel(camID) == 1) {
		//color img, mono camera
		ct::logger::trace("[Acq] Start acquiring R band");
		snapBand(optic, viewID, stitchID, BandType::R);

		ct::logger::trace("[Acq] Start acquiring G band");
		snapBand(optic, viewID, stitchID, BandType::G);

		ct::logger::trace("[Acq] Start acquiring B band");
		snapBand(optic, viewID, stitchID, BandType::B);
	}
	else {
		CAMManager::instance().frame(camID)->postTask.combineRGB = false;

		//color img, color camera
		ct::logger::trace("[Acq] Start acquiring M band");
		snapBand(optic, viewID, stitchID, BandType::M);
	}
}

void JobThread::snapOpticFastMode(const OpticsInfo& optic, QString viewID, QString stitchID, bool resetFrame)
{
	ScopedTimeLogger stl("[JobThread] Snap optic fast mode");

	auto camLSC = CAMManager::instance().lsc(optic.camID);
	if (camLSC == nullptr) {
		ct::logger::error("[JobThread] Failed to snap fast mode. Invalid LSC info for camera: %s", optic.camID.toStdString().c_str());
		return;
	}

	QVector<LSCManager::SequenceData> datas;
	const int exposure = CAMManager::instance().getExposure(optic.camID);
	const int triggerSource = camLSC->triggerSource;

	auto appendBand = [&](const ct::Band& band) {
		LSCManager::SequenceData data;
		data.exposure_us = exposure;
		data.triggerSource = triggerSource;
		data.band = band;
		datas.push_back(data);
	};

	if (optic.type == ct::s_color && CAMManager::instance().getChannel(optic.camID) == 1) {
		appendBand(optic.R);
		appendBand(optic.G);
		appendBand(optic.B);
	}
	else {
		appendBand(optic.M);
	}

	if (datas.isEmpty()) {
		ct::logger::error("[JobThread] Failed to snap fast mode. Empty trigger sequence. Optic: %s", optic.id.toStdString().c_str());
		return;
	}

	if (LSCManager::instance().setTriggerSequence(datas) != (int)LSC_RC::PASS) {
		ct::logger::error("[JobThread] Failed to snap fast mode. Could not set trigger sequence. Optic: %s", optic.id.toStdString().c_str());
		return;
	}
	m_currentTriggerSequence.clear();

	const QString previousCamID = m_camID;
	m_camID = optic.camID;
	m_lscFastMode = true;
	if (LSCManager::instance().setMode(lsc::MODE::TRIGGER) != (int)LSC_RC::PASS) {
		ct::logger::error("[JobThread] Failed to snap fast mode. Could not switch LSC to trigger mode. Optic: %s", optic.id.toStdString().c_str());
		switchToContinuousModeLSC();
		m_camID = previousCamID;
		return;
	}

	CAMManager::instance().setDO(m_camID, m_camResetIO, true);
	snapOptic(optic, viewID, stitchID, resetFrame);
	CAMManager::instance().setDO(m_camID, m_camResetIO, false);

	switchToContinuousModeLSC();
	m_camID = previousCamID;
}

void JobThread::snapView(QString viewID, bool resetFrame)
{
	MachineController::instance().trackTime("View");

	if (!(*m_views).contains(viewID)) {
		ct::logger::error("[JobThread] Failed to snap view: %s", viewID.toStdString().c_str());
		return;
	}

	auto v = (*m_views)[viewID];

	if (m_lscFastMode) {
		CAMManager::instance().setDO(m_camID, m_camResetIO, true);
	}

	for (auto& optID : v.opticIDs) {

		auto cid = util::combineID(viewID, optID);

		ScopedTimeLogger Stimer("Snap view: " + cid.toStdString());

		if (resetFrame) CAMManager::instance().resetFrame(v.camID);

		if (!m_optics->contains(optID)) {
			ct::logger::error("Failed to snap optic. Invalid optic ID: %s", optID.toStdString().c_str());
			continue;
		}

		snapOptic((*m_optics)[optID], viewID, v.map_to_sview, false);
	}

	if (m_lscFastMode) {
		CAMManager::instance().setDO(m_camID, m_camResetIO, false);
	}

	MachineController::instance().logTime("View");
}

void JobThread::triggerCamera(QString camID) //TODO: Check if the g buffer id is correct for code that use this
{
	CAMManager::instance().softTrigger(camID);
	CAMManager::instance().waitAcquisition(camID, 1000);
}

FrameInfo JobThread::scan(QString id, dat::WorldCoordinate start, dat::WorldCoordinate end, const OpticsInfo3D& optic, bool waitImage)
{
	MachineController::instance().trackTime("Scan");

	FrameInfo info;

	ct::logger::trace("Start set laser config");
	ProfilerManager::instance().stop(m_profilerID);
	ProfilerManager::instance().enableIntensityMap(m_profilerID, optic.intensity);

	if (optic.exposureMode == ct::s_single) {
		ProfilerManager::instance().setExposureMode(m_profilerID, IProfiler::SINGLE);
		ProfilerManager::instance().setExposure(m_profilerID, optic.exposure);
		ProfilerManager::instance().setGain(m_profilerID, optic.gain);
	}
	else if (optic.exposureMode == ct::s_multi) {
		ProfilerManager::instance().setExposureMode(m_profilerID, IProfiler::MULTI);
		ProfilerManager::instance().setMultiExposure(m_profilerID, optic.exposure, optic.exposure2);
		ProfilerManager::instance().setDuoHeadGain(m_profilerID, optic.gain, optic.gain2);
	}
	else if (optic.exposureMode == ct::s_dynamic) {
		ProfilerManager::instance().setExposureMode(m_profilerID, IProfiler::DYNAMIC);
		ProfilerManager::instance().setDynamicExposure(m_profilerID, optic.exposure, optic.exposure2);
		ProfilerManager::instance().setGain(m_profilerID, optic.gain);
	}
	else if (optic.exposureMode == ct::s_parallel) {
		ProfilerManager::instance().setExposureMode(m_profilerID, IProfiler::PARALLEL);
		ProfilerManager::instance().setParallelExposure(m_profilerID, optic.exposure, optic.exposure2);
		ProfilerManager::instance().setDuoHeadGain(m_profilerID, optic.gain, optic.gain2);
	}
	ProfilerManager::instance().setDivider(m_profilerID, optic.divider);
	ProfilerManager::instance().setLaserLineThreshold(m_profilerID, optic.lineThreshold);
	ct::logger::trace("Done set laser config");

	ProfilerManager::instance().getFrame(m_profilerID)->type = ct::s_height_map;
	ProfilerManager::instance().getFrame(m_profilerID)->viewID = id;
	ProfilerManager::instance().getFrame(m_profilerID)->opticID = optic.id;
	ProfilerManager::instance().getFrame(m_profilerID)->baseOpticID = QString();
	if (optic.exposureMode == ct::s_parallel) ProfilerManager::instance().getFrame(m_profilerID)->baseOpticID = optic.id;

	auto cid = util::combineID(id, optic.id);

	const bool scanAlongY = SystemData::instance().isLineScanAxisY();

	ct::logger::trace("Start set scan info");
	auto fixedLength = scanAlongY ? abs(start.wy - end.wy) : abs(start.wx - end.wx);
	ProfilerManager::instance().setScanLength(m_profilerID, fixedLength);
	ct::logger::trace("Done set scan info");

	SystemData::instance().m_extraMoveFor3DLaser = 0.0;
	jogLaser(start.wx, start.wy, start.wz, "2D");

	if (SystemData::instance().m_extraMoveFor3DLaser != 0.0) {
		m_extraMoveLog.append(m_laserOffsetInfo);
		m_extraMoveLog.append(QStringLiteral("ID_OpticID: %1, Extra Move: %2").arg(cid).arg(SystemData::instance().m_extraMoveFor3DLaser));
		m_extraMoveLog.append("\n");
	}

	/*if (!waitImage) {
		ProfilerManager::instance().waitAcquisition(m_profilerID, PROFILER_TIMEOUT);
		ct::logger::info("Done wait for laser");
	}*/


	if (!ProfilerManager::instance().start(m_profilerID)) {
		ct::logger::error("Failed to start scanning");
		emit promptMsg("Failed to start scanning");
		stopRun();
		return info;
	}

	if (scanAlongY) jogLaser(end.wx, end.wy + 6 + SystemData::instance().m_extraMoveFor3DLaser, start.wz, "3D");
	else jogLaser(end.wx + 6 + SystemData::instance().m_extraMoveFor3DLaser, end.wy, start.wz, "3D");
	//jogLaser(end.wx + 10 + SystemData::instance().m_extraMoveFor3DLaser, end.wy, start.wz, "3D");

	if (!ProfilerManager::instance().waitAcquisition(m_profilerID, PROFILER_TIMEOUT)) {
		if (!ProfilerManager::instance().stop(m_profilerID)) {
			ct::logger::error("Failed to stop scanning");
			emit promptMsg("Failed to stop scanning");
			stopRun();
			return info;
		}
		emit promptMsg("3D Profiler is not responding. Please restart the 3D Profiler.");
		stopRun();
		auto error = MachineController::instance().getErrorStatus();
		if(!error.contains((int)MachineError::ESTOP_PRESSED))unloadBoard();
	}
	ct::logger::info("Done wait for laser");

	if (waitImage) {
		info = waitForImagePreprocessed(5000);
		ct::logger::info("Done wait for heightmap");
	}


	if (!ProfilerManager::instance().stop(m_profilerID)) {
		ct::logger::error("Failed to stop scanning");
		emit promptMsg("Failed to stop scanning");
		stopRun();
		return info;
	}

	MachineController::instance().logTime("Scan");

	return info;
}

bool JobThread::findAlignFeature(MIL_ID mMono, const AlignFeatureParams& p, mtrx::PatternOutput& out)
{
	if (p.usePattern) {
		if (p.modelPath.isEmpty() || !QFile::exists(p.modelPath)) {
			ct::logger::error("[Align] Pattern model not learned: %s", p.modelPath.toStdString().c_str());
			return false;
		}

		//constrain to the search ROI when one is drawn
		MIL_ID mSearch = mMono;
		double offX = 0, offY = 0;
		bool cropped = false;

		if (!p.searchRoi.isEmpty()) {
			const MIL_INT w = MbufInquire(mMono, M_SIZE_X, M_NULL);
			const MIL_INT h = MbufInquire(mMono, M_SIZE_Y, M_NULL);
			QRectF sr = p.searchRoi.intersected(QRectF(0, 0, (double)w, (double)h));
			if (sr.width() > 10 && sr.height() > 10) {
				mSearch = mtrx::crop(mMono, sr.x(), sr.y(), sr.width(), sr.height());
				offX = sr.x();
				offY = sr.y();
				cropped = true;
			}
		}

		mtrx::PatternOutput patOut;
		patOut.acceptance_min_score = p.minScore;
		const bool found = mtrx::find_pattern(mSearch, p.modelPath.toStdString(), patOut);
		if (cropped) MbufFree(mSearch);

		if (!found || patOut.score < p.minScore) {
			ct::logger::warn("[Align] Pattern not found (best %.1f, min %.1f)", patOut.score, p.minScore);
			return false;
		}

		out = patOut;
		out.x += offX;
		out.y += offY;
		out.cx += offX;
		out.cy += offY;
		ct::logger::info("[Align] Pattern found at (%.1f, %.1f), score %.1f", out.cx, out.cy, out.score);
		return true;
	}

	mtrx::Circle circle;
	if (!mtrx::find_circle(circle, mMono, (double)p.minDiameter / 2.0, (double)p.maxDiameter / 2.0,
		mtrx::CircleType::LARGEST_RADIUS, p.foreground)) {
		ct::logger::warn("[Align] Circle not found (diameter %d-%d px)", p.minDiameter, p.maxDiameter);
		return false;
	}

	out.x = circle.cx - circle.radius;
	out.y = circle.cy - circle.radius;
	out.w = circle.radius * 2;
	out.h = circle.radius * 2;
	out.cx = circle.cx;
	out.cy = circle.cy;
	ct::logger::info("[Align] Circle found at (%.1f, %.1f), diameter %.1f px", out.cx, out.cy, out.w);
	return true;
}

void JobThread::performCameraAlignment(dat::WorldCoordinate currentPoint, double step_mm, AlignFeatureParams featureParams)
{
	//NOTE: Inconsistency is usually due to gantry error and not the algorithm itself. Can easily be proven by loading the same image to test if the circle found is different. 
	auto origin = currentPoint;

	auto horizontal_step = origin;
	horizontal_step.wx += step_mm;

	auto mainOptic = getMainOptics();
	snapOptic(mainOptic, "O", "", false);

	auto info = waitForImagePreprocessed();

	//reference
	MIL_ID mBuf = info.pImage->id();
	MIL_ID mMono = mtrx::to_mono(mBuf);
	mtrx::BufferCollector bc2(mMono);

	mtrx::PatternOutput output;

	if (findAlignFeature(mMono, featureParams, output)) {
		emit drawRectFOV("cam_align", QRectF(output.x, output.y, output.w, output.h), Qt::yellow);
	}else {
		emit cameraAlignmentFailed("Camera Alignment Failed: Origin feature not found!");
		jog(origin.wx, origin.wy, origin.wz, "2D");
		snapOptic(mainOptic, "O", "", true);
		return;
	}

	jog(horizontal_step.wx, horizontal_step.wy, horizontal_step.wz, "2D");

	os_tool::goSleep(1500);

	snapOptic(mainOptic, "H", "", false);

	auto info2 = waitForImagePreprocessed();
	
	//horizontal`
	MIL_ID mBufH = info2.pImage->id();
	MIL_ID mMonoH = mtrx::to_mono(mBufH);
	mtrx::BufferCollector bc4(mMonoH);


	mtrx::PatternOutput outputH;
	if (findAlignFeature(mMonoH, featureParams, outputH)) {
		emit drawRectFOV("cam_align", QRectF(outputH.x, outputH.y, outputH.w, outputH.h), Qt::yellow);
	}else {
		emit cameraAlignmentFailed("Camera Alignment Failed: Horizontal feature not found!");
		jog(origin.wx, origin.wy, origin.wz, "2D");
		snapOptic(mainOptic, "H", "", true);
		return;
	}

	auto cameraAngle = em::to_degree(atan((outputH.y - output.y) / (outputH.x - output.x)));

	if (std::isnan(cameraAngle)) {
		emit cameraAlignmentFailed("Invalid camera angle calculated!");
		jog(origin.wx, origin.wy, origin.wz, "2D");
		snapOptic(mainOptic, "H", "", true);
		return;
	}

	ct::logger::debug("[Camera Alignment] Output: %f, %f | H: %f, %f | Angle: %f", output.x, output.y, outputH.x, outputH.y, cameraAngle);

	emit cameraAlignmentDone(cameraAngle);

	jog(origin.wx, origin.wy, origin.wz, "2D");
	snapOptic(mainOptic, "H", "", true);
}

void JobThread::performCameraScaling(dat::WorldCoordinate currentPoint, double step_mm, AlignFeatureParams featureParams)
{
	auto origin = currentPoint;
	auto horizontal_step = origin;
	auto vertical_step = origin;

	horizontal_step.wx += step_mm;
	vertical_step.wy += step_mm;

	auto mainOptic = getMainOptics();

	std::string path;

	//reference
	jog(origin.wx, origin.wy, origin.wz, "2D");
	snapOptic(mainOptic, "O", "", false);

	auto oInfo = waitForImagePreprocessed();
	auto oID = util::combineID(oInfo.viewID, oInfo.opticID);

	MIL_ID mBuf = oInfo.pImage->id();
	MIL_ID mMono = mtrx::to_mono(mBuf);
	mtrx::BufferCollector bc2(mMono);

	bool ret1 = false, retH = false, retV = false;

	mtrx::PatternOutput output;
	if (findAlignFeature(mMono, featureParams, output)) {
		emit drawRectFOV("cam_scaling", QRectF(output.x, output.y, output.w, output.h), Qt::yellow);
	}
	else {
		emit promptMsg("Camera Scaling Failed: Origin feature not found!");
		jog(origin.wx, origin.wy, origin.wz, "2D");
		snapOptic(mainOptic, "O", "", true);
		return;
	}

	//horizontal
	jog(horizontal_step.wx, horizontal_step.wy, horizontal_step.wz, "2D");
	os_tool::goSleep(1500);


	snapOptic(mainOptic, "H", "", false);

	auto hInfo = waitForImagePreprocessed();

	MIL_ID mBufH = hInfo.pImage->id();
	MIL_ID mMonoH = mtrx::to_mono(mBufH);
	mtrx::BufferCollector bc4(mMonoH);


	mtrx::PatternOutput outputH;
	if (findAlignFeature(mMonoH, featureParams, outputH)) {
		emit drawRectFOV("cam_scaling", QRectF(outputH.x, outputH.y, outputH.w, outputH.h), Qt::yellow);
	}
	else {
		emit promptMsg("Camera Alignment Failed: Horizontal feature not found!");
		jog(origin.wx, origin.wy, origin.wz, "2D");
		snapOptic(mainOptic, "H", "", true);
		return;
	}

	double horizontal_scale = step_mm * 1000 / em::distance(output.x, output.y, outputH.x, outputH.y);
	ct::logger::info("[camera] Horizontal scale: %f\n", horizontal_scale);


	//vertical
	jog(vertical_step.wx, vertical_step.wy, vertical_step.wz, "2D");
	os_tool::goSleep(1500);


	snapOptic(mainOptic, "V", "", false);

	auto vInfo = waitForImagePreprocessed();

	MIL_ID mBufV = vInfo.pImage->id();
	MIL_ID mMonoV = mtrx::to_mono(mBufV);
	mtrx::BufferCollector bc6(mMonoV);

	mtrx::PatternOutput outputV;
	if (findAlignFeature(mMonoV, featureParams, outputV)) {
		emit drawRectFOV("cam_scaling", QRectF(outputV.x, outputV.y, outputV.w, outputV.h), Qt::yellow);
	}
	else {
		emit promptMsg("Camera Scaling Failed: Vertical feature not found!");
		jog(origin.wx, origin.wy, origin.wz, "2D");
		snapOptic(mainOptic, "V", "", true);
		return;
	}

	double vertical_scale = step_mm * 1000 / em::distance(output.x, output.y, outputV.x, outputV.y);
	ct::logger::info("[camera] Vertical scale: %f\n", vertical_scale);

	emit cameraScalingDone(horizontal_scale, vertical_scale);

	jog(origin.wx, origin.wy, origin.wz, "2D");
	snapOptic(mainOptic, "V", "", true);
}

bool JobThread::fiducialExists(int index)
{
	auto& fidInfo = *m_fiducialInfos;

	if (index >= fidInfo.size()) {
		ct::logger::error("Fiducial do not exist");
		return false;
	}
	auto fiducialJsonPath = QStringLiteral("%1recipe/%2/fiducial.json").arg(Common::Directory::LocalPath).arg(Common::Directory::CurrentRecipe);
	if (!QFileInfo(fiducialJsonPath).exists()) return false;

	return fidInfo[index].hasTeachPoint;
}

em::V2d JobThread::getFiducialPointInMM(int index, int x_px, int y_px)
{
	auto& fidInfo = *m_fiducialInfos;

	em::V2d point;

	point.x() = fidInfo[index].teach_point.wx + ScaleManager::instance().to_horizontal_mm(x_px);
	point.y() = fidInfo[index].teach_point.wy + ScaleManager::instance().to_vertical_mm(y_px);

	return point;
}

bool JobThread::locateFiducial(int index, int fidIndex, InspStatus::FiducialDetail& fDetail, bool saveImg, const dat::WorldCoordinate& curCoordinate, int crossFinderScore, Fiducial* algo)
{
	auto& fidInfo = *m_fiducialInfos;

	//route to the requested island transform (defaults to the primary one)
	Fiducial* fid = algo ? algo : m_fiducialAlgo;

	if (index >= fidInfo.size()) {
		ct::logger::warn("[Fid] Invalid fiducial index: %d", index);
		return false;
	}

	QString fileIndex = QString::number(index + 1);
	auto root = Common::Directory::getRecipeImagesPath() + QString("Fiducial/");
	std::string path = root.toStdString() + "feature" + fileIndex.toStdString() + ".pat";

	const auto& mainOptic = getMainOptics();

	CAMManager::instance().resetFrame(mainOptic.camID);

	QString viewID = QString("fid%1").arg(fileIndex);
	//need to add Z stack function here

	bool generate_2D_stack = fidInfo.at(index).generate_2D_stack;
	bool generate_3D_stack = fidInfo.at(index).generate_3D_stack;
	QString acq_type = fidInfo.at(index).acq_type;
	int preset_iteration = fidInfo.at(index).preset_iteration;
	int step_um = fidInfo.at(index).step_um;
	int encoder_range_um = fidInfo.at(index).encoder_range_um;
	if (generate_2D_stack || generate_3D_stack) {

		CAMManager::instance().frame(mainOptic.camID)->postTask.stackImage = true; //SEETHIS:

		if (acq_type == ct::s_preset) {
			ct::logger::info("Preset iteration: %d, %dum", preset_iteration, step_um);

			snapOptic(mainOptic, viewID, "", false);

			for (int i = 1; i < preset_iteration; i++) {
				ct::logger::info("Step: %.2fum", -((double)i * (double)step_um) / 1000);
				

				double new_z = curCoordinate.wz + -((double)i * (double)step_um) / 1000;
				jog(curCoordinate.wx, curCoordinate.wy, new_z, "2D", true);
				snapOptic(mainOptic, viewID, "", false);
			}

			auto cid = util::combineID(viewID, mainOptic.id);
			emit stackImages(cid);
		}
		else if (acq_type == ct::s_encoder) {

		}
		/*else if (acq_type == ct::s_time) {
			auto offset_mm = -(double)encoder_range_um / 1000.0;
			jogView(v);
			m_thread = std::thread(&JobThread::continuousSnap, this);
			jogView(v, offset_mm);
			m_stopZstack = true;
			m_thread.join();
			emit stackImages(id);
		}*/
	}
	else snapOptic(mainOptic, viewID, "", false);

	auto info = waitForImagePreprocessed();

	fidInfo.at(index).search_region.compute_extremum();
	auto sr_x = fidInfo.at(index).search_region.xmin;
	auto sr_y = fidInfo.at(index).search_region.ymin;
	auto sr_w = fidInfo.at(index).search_region.w;
	auto sr_h = fidInfo.at(index).search_region.h;

	MIL_ID mFid = info.pImage->id();

	if (saveImg) {
		PostResult pr;
		pr.frame = info;
		pr.bufferPath = "fid_" + fileIndex + g_imgExtension;
		m_postResults.push_back(pr);
	}

	auto w = mtrx::get_width(mFid);
	auto h = mtrx::get_height(mFid);

	if (sr_x < 0 || sr_y < 0 || sr_w > w || sr_h > h) {
		promptMsg("Fiducial Search Region out of bounds. Reposition the Fiducial search region to proceed.");
		return false;
	}

	MIL_ID mCrop;
	if (sr_w == w && sr_h == h && sr_x == 0 && sr_y == 0) {
		mCrop = mtrx::alloc_buffer(mFid);
		MbufCopy(mFid, mCrop);
	}
	else {
		mCrop = mtrx::crop(mFid, sr_x, sr_y, sr_w, sr_h);
	}

	MIL_ID mMono = mtrx::to_mono(mCrop);
	mtrx::BufferCollector bc_mBuf(mCrop);
	mtrx::BufferCollector bc_mMono(mMono);

	mtrx::PatternOutput output;
	mtrx::Circle circle;
	mtrx::Cross cross;
	int ret = 0;

	QString fidType = "unknown";
	output.acceptance_min_score = fidInfo.at(index).score;
	output.certainty_min_score = 80;

	if (fidInfo[index].fiducial_method == 0)
	{
		path = root.toStdString() + "feature" + fileIndex.toStdString() + ".pat";
		ret = mtrx::find_pattern(mMono, path, output);

		fidType = "Pattern Matching";
	}
	else if (fidInfo[index].fiducial_method == 1)
	{
		path = root.toStdString() + "feature" + fileIndex.toStdString() + ".mod";
		ret = mtrx::find_geometryModel(mMono, path, output);

		fidType = "Geometry Finder";
	}
	else if (fidInfo[index].fiducial_method == 2)
	{

		if (!mtrx::find_circle(circle, mMono, fidInfo.at(index).min_diameter / 2, fidInfo.at(index).max_diameter / 2, mtrx::CircleType::HIGHEST_SCORE))
		{
			ct::logger::debug("circle not found");
		}
		else
		{
			ret = 1;
			output.x = circle.cx - circle.radius;
			output.y = circle.cy - circle.radius;
			output.w = circle.radius * 2;
			output.h = circle.radius * 2;
			output.score = circle.score;
		}
		fidType = "Circle Finder";
	}
	else if (fidInfo[index].fiducial_method == 3)
	{
		if (!mtrx::find_cross(cross, mMono, 250, 250, 80, 80, crossFinderScore))
		{
			ct::logger::debug("cross not found");
		}
		else
		{
			ret = 1;
			output.x = cross.x;
			output.y = cross.y;
			output.w = cross.width;
			output.h = cross.height;
			output.score = cross.score;
		}
		fidType = "Cross Finder";
	}


	if (ret) {
		auto locatedPoint = QRectF(sr_x + output.x, sr_y + output.y, output.w, output.h);
		emit locatedFiducial(locatedPoint);

		auto learnt = getFiducialPointInMM(index, fidInfo[index].inspect_region.cx, fidInfo[index].inspect_region.cy);
		auto located = getFiducialPointInMM(index, locatedPoint.x() + (output.w / 2 + 1), locatedPoint.y() + (output.h / 2 + 1)); //center the point with additional 1px, tested with direct learn and inspect. 
		fid->setLearntFid(fidIndex, learnt);
		fid->setShiftedFid(fidIndex, located);
		fid->compute();

		fDetail.fiducialScore = output.score;
		fDetail.fiducialRect = locatedPoint;
		fDetail.type = fidType;

		// Angle check is only meaningful when fiducial rotation is enabled; the
		// same checkbox gates both applying the rotation and this sanity guard.
		if (SystemData::instance()._enableFiducialRotate && abs(fid->getAngle()) > 3)
		{
			ret = 0;
			std::string msg = "Angle: " + QString::number(fid->getAngle()).toStdString() + " exceed 3 degree!!! Locator Failed!!!";
			ct::logger::warn(msg.c_str());
			promptMsg(msg.c_str());
		}
	}

	return ret;
}

void JobThread::searchFiducial()
{
	if (SystemData::instance()._doubleFiducialChecking) {
		searchDoubleFiducial();
		return;
	}

	//reset anyway to avoid having residue offset
	resetFiducial();
	auto& fidInfo = *m_fiducialInfos;

	int index = 0;
	for (const auto& f : fidInfo) {

		if (m_stopRun) return;

		bool stopFid = false;

		auto& p = f.teach_point;

		jog(p.wx, p.wy, p.wz, "2D");

		InspStatus::FiducialDetail fDetail;

		if (!fiducialExists(index)) {
			stopFid = true;
		}
		else
		{
			if (!locateFiducial(index, m_locatedFidID.size(), fDetail, true, p)) stopFid = true;
		}

		if (stopFid) {
			std::string msg = "Failed to locate " + f.id.toStdString() + "!";
			ct::logger::warn(msg.c_str());

			fDetail.fiducialStatus = QString::number(index) + "_false";
			fDetail.index = index;
			fDetail.isPass = false;
		}
		else {
			fDetail.fiducialStatus = QString::number(index) + "_true";
			fDetail.index = index;
			fDetail.isPass = true;
			m_locatedFidID.insert(f.id);
		}

		m_inspStatus->fiducialHash.insert(fDetail.index, fDetail);
		//emit updateFiducialStatus(fDetail);

		if (m_locatedFidID.size() == 2) {
			emit fiducialDone();
			return;
		}

		index++;
	}

	if (m_locatedFidID.size() < 2) {
		stopRun();
		emit fiducialFailed();
	}
}

void JobThread::searchDoubleFiducial()
{
	//reset anyway to avoid having residue offset (resets both island transforms)
	resetFiducial();
	auto& fidInfo = *m_fiducialInfos;

	if (fidInfo.size() < 4 || !m_fiducialAlgo2) {
		ct::logger::warn("[Fid] Double fiducial checking requires 4 fiducials (found %d). Aborting run.", (int)fidInfo.size());
		stopRun();
		emit fiducialFailed();
		return;
	}

	//island 0 = fid1/fid2 (index 0,1) ; island 1 = fid3/fid4 (index 2,3)
	int islandLocated[2] = { 0, 0 };

	for (int index = 0; index < 4; index++) {

		if (m_stopRun) return;

		const auto& f = fidInfo[index];
		auto& p = f.teach_point;

		jog(p.wx, p.wy, p.wz, "2D");

		int island = index / 2;                                       //0,1 -> island 0 ; 2,3 -> island 1
		int slot = index % 2;                                         //within-island slot (0 or 1)
		Fiducial* algo = (island == 0) ? m_fiducialAlgo : m_fiducialAlgo2;

		InspStatus::FiducialDetail fDetail;
		bool stopFid = false;

		if (!fiducialExists(index)) {
			stopFid = true;
		}
		else {
			if (!locateFiducial(index, slot, fDetail, true, p, 85, algo)) stopFid = true;
		}

		if (stopFid) {
			std::string msg = "Failed to locate " + f.id.toStdString() + "!";
			ct::logger::warn(msg.c_str());

			fDetail.fiducialStatus = QString::number(index) + "_false";
			fDetail.index = index;
			fDetail.isPass = false;
		}
		else {
			fDetail.fiducialStatus = QString::number(index) + "_true";
			fDetail.index = index;
			fDetail.isPass = true;
			m_locatedFidID.insert(f.id);
			islandLocated[island]++;
		}

		m_inspStatus->fiducialHash.insert(fDetail.index, fDetail);
	}

	//both islands must have both fiducials located; if either fails, abort the whole run
	if (islandLocated[0] < 2 || islandLocated[1] < 2) {
		ct::logger::warn("[Fid] Double fiducial checking failed (island 1: %d/2, island 2: %d/2). Aborting run.", islandLocated[0], islandLocated[1]);
		stopRun();
		emit fiducialFailed();
		return;
	}

	emit fiducialDone();
}

Fiducial* JobThread::fiducialForPoint(double x, double y)
{
	if (!SystemData::instance()._doubleFiducialChecking || !m_fiducialAlgo2) return m_fiducialAlgo;

	auto& fidInfo = *m_fiducialInfos;
	if (fidInfo.size() < 4) return m_fiducialAlgo;

	//island centroids from the taught fiducial points (same mm world frame as the target point)
	double c0x = (fidInfo[0].teach_point.wx + fidInfo[1].teach_point.wx) / 2.0;
	double c0y = (fidInfo[0].teach_point.wy + fidInfo[1].teach_point.wy) / 2.0;
	double c1x = (fidInfo[2].teach_point.wx + fidInfo[3].teach_point.wx) / 2.0;
	double c1y = (fidInfo[2].teach_point.wy + fidInfo[3].teach_point.wy) / 2.0;

	double d0 = (x - c0x) * (x - c0x) + (y - c0y) * (y - c0y);
	double d1 = (x - c1x) * (x - c1x) + (y - c1y) * (y - c1y);

	return (d1 < d0) ? m_fiducialAlgo2 : m_fiducialAlgo;
}

void JobThread::saveFiducialResult()
{
	auto jsonPath = m_rootPath + "/FiducialInfo.json";

	QJsonObject j_root;
	
	QJsonArray j_array;
	for (auto locatedID : m_locatedFidID) {
		j_array.append(locatedID);
	}

	j_root.insert(QStringLiteral("located_fiducials"), j_array);
	j_root.insert(QStringLiteral("fiducial_angle"), m_fiducialAlgo->getAngle());
	j_root.insert(QStringLiteral("fiducial_offset_x"), m_fiducialAlgo->getOffset().x());
	j_root.insert(QStringLiteral("fiducial_offset_y"), m_fiducialAlgo->getOffset().y());

	if (SystemData::instance()._doubleFiducialChecking && m_fiducialAlgo2) {
		j_root.insert(QStringLiteral("fiducial_angle_2"), m_fiducialAlgo2->getAngle());
		j_root.insert(QStringLiteral("fiducial_offset_x_2"), m_fiducialAlgo2->getOffset().x());
		j_root.insert(QStringLiteral("fiducial_offset_y_2"), m_fiducialAlgo2->getOffset().y());
	}

	auto ret = jsonHelper::saveJson(jsonPath, QJsonDocument(j_root));

	if (ret) ct::logger::info("Successfully saved fiducial result!");
	else ct::logger::error("Failed to save fiducial result!");
}

void JobThread::save3DExtraOffset()
{
	if (m_extraMoveLog.isEmpty()) return;

	QFile file(m_rootPath + "/Extra3DMoveLog.txt");
	if (!file.open(QIODevice::Append | QIODevice::Text)) {
		ct::logger::error("Failed to open log file.");
		return;
	}

	QTextStream out(&file);
	out.setCodec("UTF-8");

	for (const QString& line : m_extraMoveLog) {
		out << line << "\n";
	}

	file.close();

}

void JobThread::autoSetFiducialPoint(int currentFid)
{
	ct::logger::info("User trigger auto set fiducial point.");

	m_locatedFidID.clear();

	m_currentFidIndex = currentFid;

	auto& f = (*m_fiducialInfos)[m_currentFidIndex];
	auto& p = f.teach_point;

	//locate and jog must use the same island transform (primary when double fiducial checking is off)
	Fiducial* algo = fiducialForPoint(p.wx, p.wy);
	algo->reset();

	int w = f.inspect_region.w;
	int h = f.inspect_region.h;
	int cx = CAMManager::instance().getWidth(m_camID) / 2;
	int cy = CAMManager::instance().getHeight(m_camID) / 2;
	f.inspect_region.cx = cx;
	f.inspect_region.cy = cy;
	f.inspect_region.compute_extremum();

	//emit updateFiducialRegion();
	//emit teachFiducialPoint();

	//1 jog to current locator position
	jog(p.wx, p.wy, p.wz, "2D");

	//2 perform fiducial checking to get offset value
	bool stopInsp = false;
	InspStatus::FiducialDetail fDetail;
	if (!fiducialExists(m_currentFidIndex)) stopInsp = true;
	else
	{
		if (!locateFiducial(m_currentFidIndex, m_locatedFidID.size(), fDetail, false, p, 50, algo)) stopInsp = true;
	}

	if (stopInsp) {
		std::string msg = "Failed to locate " + f.id.toStdString() + "!";
		ct::logger::warn(msg.c_str());

		emit fiducialFailed();
		return;
	}
	else {

		//3 use offset to jog to center point
		jogBasedOnFiducial(p.wx, p.wy, p.wz, "2D", true);

		//4 perform locate fiducial again to set the new learnt position
		InspStatus::FiducialDetail fDetail;
		bool stopInsp = false;
		if (!fiducialExists(m_currentFidIndex)) stopInsp = true;
		else
		{
			if (!locateFiducial(m_currentFidIndex, m_locatedFidID.size(), fDetail, false, p, 50, algo)) stopInsp = true;
		}

		if (stopInsp) {
			std::string msg = "Failed to locate " + f.id.toStdString() + "!";
			ct::logger::warn(msg.c_str());

			emit fiducialFailed();
			return;
		}
		else {
			emit updateFiducialRegion();
			emit teachFiducialPoint();
		}
	}
}

void JobThread::testFiducial(int index, bool online)
{
	auto& fidInfos = *m_fiducialInfos;

	if (index >= fidInfos.size()) {
		ct::logger::warn("[UB] Invalid fiducial index: %d", index);
		return;
	}

	QString fileIndex = QString::number(index + 1);
	auto root = Common::Directory::getRecipeImagesPath() + QString("Fiducial/");
	std::string path = root.toStdString() + "feature" + fileIndex.toStdString() + ".pat";

	MIL_ID mBuf;

	if (online) {
		snapOptic(getMainOptics(), "", "");
		auto info = waitForImagePreprocessed();
		mBuf = info.pImage->id();
	}
	else {
		auto path_fid = Common::Directory::CurrentImageSetPath + QString("fid%1.jpg").arg(fileIndex);
		if (!QFile::exists(path_fid)) path_fid = Common::Directory::getRecipeImagesPath() + QString("Fiducial/fid%1.jpg").arg(fileIndex);
		if (!QFile::exists(path_fid)) {
			promptMsg("Failed to test offline as there are no image collected for fiducial. Teach Point to proceed");
			return;
		}

		mBuf = MbufRestoreA(path_fid.toStdString().c_str(), M_DEFAULT_HOST, M_NULL);
		displayFOV_fnc(mBuf);
	}

	fidInfos.at(index).search_region.compute_extremum();
	auto sr_x = fidInfos.at(index).search_region.xmin;
	auto sr_y = fidInfos.at(index).search_region.ymin;
	auto sr_w = fidInfos.at(index).search_region.w;
	auto sr_h = fidInfos.at(index).search_region.h;

	auto w = mtrx::get_width(mBuf);
	auto h = mtrx::get_height(mBuf);

	if (sr_x < 0 || sr_y < 0 || sr_w > w || sr_h > h) {
		promptMsg("Search Region out of bounds. Reposition the search region to proceed.");
		return;
	}
	
	MIL_ID mCrop;
	if (sr_w == w && sr_h == h && sr_x == 0 && sr_y == 0) {
		mCrop = mtrx::alloc_buffer(mBuf);
		MbufCopy(mBuf, mCrop);
	}
	else {
		mCrop = mtrx::crop(mBuf, sr_x, sr_y, sr_w, sr_h);
	}
	
	MIL_ID mMono = mtrx::to_mono(mCrop);

	mtrx::BufferCollector bc_mCrop(mCrop);
	mtrx::BufferCollector bc_mMono(mMono);

	mtrx::PatternOutput output;
	output.acceptance_min_score = fidInfos.at(index).score;
	output.certainty_min_score = 80;
	mtrx::Circle circle;
	mtrx::Cross cross;
	if (fidInfos[index].fiducial_method == 0)
	{
		path = root.toStdString() + "feature" + fileIndex.toStdString() + ".pat";
		if (!mtrx::find_pattern(mMono, path, output)) {
			ct::logger::error("[Fiducial] Pattern not found");
		}
	}
	else if (fidInfos[index].fiducial_method == 1)
	{
		path = root.toStdString() + "feature" + fileIndex.toStdString() + ".mod";
		if (!mtrx::find_geometryModel(mMono, path, output)) {
			ct::logger::error("[Fiducial] Model not found");
		}
	}
	else if (fidInfos[index].fiducial_method == 2)
	{
		if (!mtrx::find_circle(circle, mMono, fidInfos.at(index).min_diameter / 2, fidInfos.at(index).max_diameter / 2, mtrx::CircleType::HIGHEST_SCORE, mtrx::FOREGROUND_ANY))
		{
			ct::logger::error("[Fiducial] Circle not found");
		}
		else
		{
			output.x = circle.cx - circle.radius;
			output.y = circle.cy - circle.radius;
			output.w = circle.radius * 2;
			output.h = circle.radius * 2;
			output.score = circle.score;
		}
	}
	else if (fidInfos[index].fiducial_method == 3)
	{
		if (!mtrx::find_cross(cross, mMono, 250, 250, 80, 80, 85))
		{
			ct::logger::error("[Fiducial] Cross not found");
		}
		else
		{
			output.x = cross.x;
			output.y = cross.y;
			output.w = cross.width;
			output.h = cross.height;
			output.score = cross.score;
		}
	}

	ct::logger::debug("output: %f, %f, %f, %f, score:%f", output.x, output.y, output.w, output.h, output.score);

	auto locatedPoint = QRectF(sr_x + output.x, sr_y + output.y, output.w, output.h);
	
	auto learnt = getFiducialPointInMM(index, fidInfos[index].inspect_region.cx, fidInfos[index].inspect_region.cy);
	auto fid = getFiducialPointInMM(index, locatedPoint.x() + (output.w / 2 + 1), locatedPoint.y() + (output.h / 2 + 1));
	ct::logger::debug("Learnt point(mm): %f, %f", learnt.x(), learnt.y());
	ct::logger::debug("Located point(mm): %f, %f", fid.x(), fid.y());
	
	emit locatedFiducial(locatedPoint);
}

bool JobThread::barcodeExists(int index)
{
	if (index > 1) return false;
	auto barcode = (*m_barcodeInfos)[index];

	auto barcodeJsonPath = QStringLiteral("%1recipe/%2/barcode.json").arg(Common::Directory::LocalPath).arg(Common::Directory::CurrentRecipe);
	if (!QFileInfo(barcodeJsonPath).exists()) return false;

	return barcode.hasTeachPoint;
}

QString JobThread::readBarcode(int index, bool online)
{
	if (!barcodeExists(index)) return msg_failed_barcode;

	auto barcode = (*m_barcodeInfos)[index];

	QString fileIndex = QString::number(index + 1);
	auto root = Common::Directory::getRecipeImagesPath() + QString("Barcode/");

	auto mainOptic = getMainOptics();

	FrameInfo info;
	MIL_ID mBuf;

	QString barcodeID = QString("barcode%1").arg(fileIndex);

	if (online) {
		auto& p = barcode.teach_point;

		bool generate_2D_stack = barcode.generate_2D_stack;
		bool generate_3D_stack = barcode.generate_3D_stack;
		QString acq_type = barcode.acq_type;
		int preset_iteration = barcode.preset_iteration;
		int step_um = barcode.step_um;
		int encoder_range_um = barcode.encoder_range_um;

		CAMManager::instance().resetFrame(mainOptic.camID);
		QString viewID = QString("barcode%1").arg(fileIndex);

		if (generate_2D_stack || generate_3D_stack) {

			CAMManager::instance().frame(mainOptic.camID)->postTask.stackImage = true; //SEETHIS:

			if (acq_type == ct::s_preset) {
				ct::logger::info("Preset iteration: %d, %dum", preset_iteration, step_um);

				for (int i = 0; i < preset_iteration; i++) {
					ct::logger::info("Step: %.2fum", -((double)i * (double)step_um) / 1000);


					double new_z = p.wz + -((double)i * (double)step_um) / 1000;
					jog(p.wx, p.wy, new_z, "2D", true);
					snapOptic(mainOptic, barcodeID, "", false);
				}

				auto cid = util::combineID(viewID, mainOptic.id);
				emit stackImages(cid);
			}
			else if (acq_type == ct::s_encoder) {

			}
			/*else if (acq_type == ct::s_time) {
				auto offset_mm = -(double)encoder_range_um / 1000.0;
				jogView(v);
				m_thread = std::thread(&JobThread::continuousSnap, this);
				jogView(v, offset_mm);
				m_stopZstack = true;
				m_thread.join();
				emit stackImages(id);
			}*/
		}
		else
		{
			jog(p.wx, p.wy, p.wz, "2D");
			snapOptic(getMainOptics(), barcodeID, "", false);
		}

		info = waitForImagePreprocessed();
		mBuf = info.pImage->id();
	}
	else
	{
		auto path_barcode = Common::Directory::CurrentImageSetPath + QString("barcode%1.jpg").arg(fileIndex);
		if (!QFile::exists(path_barcode)) path_barcode = Common::Directory::getRecipeImagesPath() + QString("Barcode\\barcode%1.jpg").arg(fileIndex);
		if (!QFile::exists(path_barcode)) {
			promptMsg(QString("Failed to load barcode image for offline testing: %1").arg(path_barcode));
			return "FAILED_TO_LOAD_BARCODE_OFFLINE";
		}

		mBuf = MbufRestoreA(path_barcode.toStdString().c_str(), M_DEFAULT_HOST, M_NULL);
		displayFOV_fnc(mBuf);
	}

	barcode.search_region.compute_extremum();
	auto sr_x = barcode.search_region.xmin;
	auto sr_y = barcode.search_region.ymin;
	auto sr_w = barcode.search_region.w;
	auto sr_h = barcode.search_region.h;


	auto w = mtrx::get_width(mBuf);
	auto h = mtrx::get_height(mBuf);

	if (sr_x < 0 || sr_y < 0 || sr_w > w || sr_h > h) {
		emit promptMsg("Search Region out of bounds. Reposition the search region to proceed.");
		return "BARCODE_SEARCH_REGION_OUT_OF_BOUND";
	}

	MIL_ID mColor;
	if (sr_w == w && sr_h == h && sr_x == 0 && sr_y == 0) {
		mColor = mtrx::alloc_buffer(mBuf);
		MbufCopy(mBuf, mColor);
	}
	else {
		mColor = mtrx::crop(mBuf, sr_x, sr_y, sr_w, sr_h);
	}

	mtrx::BarcodeInput input;
	mtrx::BarcodeOutput output;
	MIL_ID debugImage = M_NULL;

	input.image_channel = barcode.image_channel;
	input.barcode_type = barcode.barcode_type;
	input.foreground_type = barcode.foreground_type; 
	//input.recognitionType = M_IMPROVED_RECOGNITION;

	input.recognitionType =
		(barcode.recognition_type == 0)
		? M_IMPROVED_RECOGNITION
		: M_TYPICAL_RECOGNITION;

	bool pass = mtrx::find_barcode(mColor, input, output, debugImage, false);

	if (!pass)
	{
		MIL_INT SizeX = 0;
		MIL_INT SizeY = 0;
		MIL_INT SizeB = 0;
		MbufInquire(mColor, M_SIZE_X, &SizeX);
		MbufInquire(mColor, M_SIZE_Y, &SizeY);
		MbufInquire(mColor, M_SIZE_BAND, &SizeB);

		MIL_UINT8* RedSrcPtr, * GreenSrcPtr, * BlueSrcPtr, * MonoSrcPtr;
		MIL_ID PitchPtr;
		if (SizeB == 3)
		{
			MbufInquire(mColor, M_HOST_ADDRESS_BAND + 0, &RedSrcPtr);
			MbufInquire(mColor, M_HOST_ADDRESS_BAND + 1, &GreenSrcPtr);
			MbufInquire(mColor, M_HOST_ADDRESS_BAND + 2, &BlueSrcPtr);
			MbufInquire(mColor, M_PITCH, &PitchPtr);

			int blackThreshold = 80;
			for (int y = 0; y < SizeY; ++y) {
				for (int x = 0; x < SizeX; ++x) {

					int R = RedSrcPtr[x + (y * PitchPtr)];
					int G = GreenSrcPtr[x + (y * PitchPtr)];
					int B = BlueSrcPtr[x + (y * PitchPtr)];

					int count = 0;
					if (R <= blackThreshold) count++;
					if (G <= blackThreshold) count++;
					if (B <= blackThreshold) count++;

					if (count > 1)
					{
						RedSrcPtr[x + (y * PitchPtr)] = 255;
						GreenSrcPtr[x + (y * PitchPtr)] = 255;
						BlueSrcPtr[x + (y * PitchPtr)] = 255;
					}
				}
			}
		}
		else if (SizeB == 1)
		{
			MbufInquire(mColor, M_HOST_ADDRESS, &MonoSrcPtr);
			MbufInquire(mColor, M_PITCH, &PitchPtr);
		}

		if (debugImage) mtrx::free_buffer(debugImage);
		pass = mtrx::find_barcode(mColor, input, output, debugImage, false);
	}

	auto path_debugImage = Common::Directory::getRecipeImagesPath() + QString("Barcode/debugImage%1.jpg").arg(index + 1);
	if (debugImage)
	{
		MbufExportA(path_debugImage.toStdString().c_str(), M_JPEG_LOSSY, debugImage);
		mtrx::free_buffer(debugImage);
	}

	QString decodedBarcode = msg_failed_barcode;
	if (pass) {
		decodedBarcode = output.decoded_barcode.c_str();
	}
	else {
		if (online) {
			PostResult pr;
			pr.frame = info;
			pr.bufferPath = QString("barcode%1.jpg").arg(index + 1);
			m_postResults.push_back(pr);
		}
	}
	
	emit locatedBarcode(QRectF(sr_x + output.x, sr_y + output.y, output.w, output.h), index, pass, decodedBarcode);

	if (pass) {
		ct::logger::info("[Barcode] Camera read %d PASS: %s", index + 1,
			decodedBarcode.toStdString().c_str());
	}
	else {
		ct::logger::warn("[Barcode] Camera read %d FAILED to decode (debug image: %s)",
			index + 1, path_debugImage.toStdString().c_str());
	}

	mtrx::free_buffer(mColor);
	if (!online) mtrx::free_buffer(mBuf);

	return decodedBarcode;
}

void JobThread::searchBarcode()
{
	qDebug() << "Search Barcode";
	qDebug() << "m stop run:" << m_stopRun;
	qDebug() << "m_enableBarcode:" << m_enableBarcode;

	if (m_stopRun) return;

	QString decodedBarcode = "No_Barcode";
	 //SystemData::instance()._currentBarcode = decodedBarcode.toStdString();

	// External slots are filled over TCP by the SR-X readers; their values are consumed
	// from the barcode line edits, so skip the camera read entirely.
	const bool hasExternal = m_barcodeInfos &&
		(((*m_barcodeInfos)[0].registration_method == 2) ||
		 ((*m_barcodeInfos)[1].registration_method == 2));   // 2=External

	if (hasExternal) {
		ct::logger::info("[Barcode] External registration, consuming TCP reader values");
		emit barcodeDecoded("External_Barcode");
		return;
	}

	if (m_enableBarcode) {
		//read barcode 1
		decodedBarcode = readBarcode(0);
		if (decodedBarcode != msg_failed_barcode) {
			ct::logger::info("[Barcode] Board barcode from slot 1: %s",
				decodedBarcode.toStdString().c_str());
			emit barcodeDecoded(decodedBarcode);
			return;
		}
		else {
			//read barcode 2 if barcode 1 fail
			ct::logger::warn("[Barcode] Slot 1 failed to read, retrying with slot 2");
			decodedBarcode = readBarcode(1);
			if (decodedBarcode != msg_failed_barcode) {
				ct::logger::info("[Barcode] Board barcode from slot 2: %s",
					decodedBarcode.toStdString().c_str());
				emit barcodeDecoded(decodedBarcode);
			}
			else {
				ct::logger::error("[Barcode] Both slots failed to read - board has no barcode");
				emit barcodeDecoded(decodedBarcode);
			}
		}
	}
	else
	{
		emit barcodeDecoded("External_Barcode");
	}
}

void JobThread::warpageCompensation()
{
	if (m_stopRun) return;

	if (!ProfilerManager::instance().getFrame(m_profilerID)) return;
	ProfilerManager::instance().getFrame(m_profilerID)->type = ct::s_height_snapshot;
	if (m_warpageMethod == "Subsampling") subWarpageCompensation();
	else if (m_warpageMethod == "Fullsampling") fullWarpageCompensation();
}

void JobThread::subWarpageCompensation()
{
	m_compensateZMap.clear();

	//compute subsampling region
	double xmin = 99999999.0;
	double ymin = 99999999.0;
	double xmax = -999.0;
	double ymax = -999.0;
	double currentZ = 0.0;

	for (const auto& v : *m_views) {
		if (xmin > v.world.wx) xmin = v.world.wx;
		if (xmax < v.world.wx) xmax = v.world.wx;
		if (ymin > v.world.wy) ymin = v.world.wy;
		if (ymax < v.world.wy) ymax = v.world.wy;
		currentZ = v.world.wz;
	}


	int camWidth = CAMManager::instance().getWidth("cam1");
	int camHeight = CAMManager::instance().getHeight("cam1");
	auto half_width_mm = ScaleManager::instance().to_horizontal_mm(camWidth) / 2;
	auto half_height_mm = ScaleManager::instance().to_vertical_mm(camHeight) / 2;
	auto width_mm = ScaleManager::instance().to_horizontal_mm(camWidth);
	auto height_mm = ScaleManager::instance().to_vertical_mm(camHeight);


	//get first row
	auto isSameRow = [](const QView& v, double centerY, double range) -> bool {
		return (abs(centerY - v.world.wy) < range);
		};

	auto allowable_range = width_mm * 0.3; //30% of FOV

	int numRow = 0;
	for (const auto v : *m_views) {
		if (isSameRow(v, ymin, allowable_range)) {
			numRow++;
		}
	}
	ct::logger::trace("Num rows: %d", numRow);

	//shift to edge, previously is the extremum of center point
	xmin -= half_width_mm;
	xmax += half_width_mm;
	ymin -= half_height_mm;
	ymax += half_height_mm;


	double offsetX = abs(xmax - xmin) / numRow; //offset by width of FOV in x direction
	double offsetY = abs(ymax - ymin) / 4; //every column will measure 2 points in y direction evenly
	std::array<double, 2> Ypoints;
	Ypoints[0] = ymin + abs(ymax - ymin) / 4;
	Ypoints[1] = ymax - abs(ymax - ymin) / 4;

	ct::logger::trace("X range: %f, %f", xmin, xmax);
	ct::logger::trace("Y range: %f, %f", ymin, ymax);
	ct::logger::trace("Offset: %f, %f", offsetX, offsetY);

	struct SampleInfo {
		double wx, wy, wz;
		double offsetZ;
	};

	std::vector<SampleInfo> sampleInfos;

	double currentX = xmin + half_width_mm;
	while (currentX < xmax) {
		for (const auto& currentY : Ypoints) {
			ct::logger::trace("Point: %f, %f", currentX, currentY);

			SampleInfo s;
			s.wx = currentX;
			s.wy = currentY;
			s.wz = currentZ;
			sampleInfos.push_back(s);
		}
		currentX += offsetX;
	}


	//get offset z 
	//IGocator::instance().stop();
	ProfilerManager::instance().stop(m_profilerID);
	ProfilerManager::instance().getFrame(m_profilerID)->type = ct::s_height_map;

	//assign offset to neighbouring sample
	auto isSameColumn = [](const QView& v, const SampleInfo& s) -> bool {
		return (abs(s.wx - v.world.wx) < 1);
		};

	auto isWithinDistance = [](const QView& v, const SampleInfo& s, double distance) -> bool {
		return (abs(s.wy - v.world.wy) < distance);
		};

	auto isAssigned = [=](const QView& v) -> bool {
		return m_compensateZMap.contains(v.id);
		};

	int index = 0;
	for (const auto& s : sampleInfos) {
		if (m_stopRun) return;

		dat::WorldCoordinate point;
		point.wx = s.wx;
		point.wy = s.wy;
		point.wz = s.wz;

		jogLaserBasedOnFiducial(s.wx, s.wy, s.wz, "2D");
		//IGocator::instance().snapshot();
		ProfilerManager::instance().snapShot(m_profilerID);
		ProfilerManager::instance().waitAcquisition(m_profilerID, PROFILER_TIMEOUT);

		/*if (IGocator::instance().go_info().mode != GO_MODE_PROFILE) {
			ct::logger::error("Expected profile mode, but receive other mode: %d", IGocator::instance().go_info().mode);
			return;
		}*/

		auto& profiles = ProfilerManager::instance().getFrame(m_profilerID)->profiles;
		auto average = std::accumulate(profiles.begin(), profiles.end(), 0.0) / profiles.size();

		auto offset = -average;

		for (const auto& v : *m_views) {
			if (isSameColumn(v, s) && isWithinDistance(v, s, offsetY) && !isAssigned(v)) {
				ct::logger::debug("[%d] ID: %s", index, v.name.toStdString().c_str());
				if (isAssigned(v)) m_compensateZMap[v.id] = s.offsetZ;
			}
		}

		index++;
	}
}

void JobThread::fullWarpageCompensation()
{
	m_compensateZMap.clear();

	//IGocator::instance().stop();
	ProfilerManager::instance().stop(m_profilerID);

	for (int i = 0; i < m_viewSequence->count(); i++) {
		if (m_stopRun) return;

		auto item = m_viewSequence->item(i);
		auto id = item->whatsThis();

		if (!(*m_views).contains(id)) {
			ct::logger::error("[JobThread] Failed to perform warpage compensation. Invalid view ID: %s", id.toStdString().c_str());
			continue;
		}

		auto v = (*m_views)[id];

		jogLaserBasedOnFiducial(v.world.wx, v.world.wy, v.world.wz, "2D");
		
		//IGocator::instance().snapshot();
		ProfilerManager::instance().snapShot(m_profilerID);
		ProfilerManager::instance().waitAcquisition(m_profilerID, PROFILER_TIMEOUT);

		/*if (IGocator::instance().go_info().mode != GO_MODE_PROFILE) {
			ct::logger::error("Expected profile mode, but receive other mode: %d", IGocator::instance().go_info().mode);
			return;
		}*/

		auto& profiles = ProfilerManager::instance().getFrame(m_profilerID)->profiles;
		auto average = std::accumulate(profiles.begin(), profiles.end(), 0.0) / profiles.size();
		if (std::isnan(average))
		{
			ct::logger::warn("[Warpage Compenastion] Average value is NAN");
		}
		else
		{
			auto offset = -average;
			m_compensateZMap.insert(v.id, offset);
		}
		
	}
}

void JobThread::generateWarpageMap() //TODO: Get qimg from world
{
	//IGocator::instance().stop();
	ProfilerManager::instance().stop(m_profilerID);

	QImage world3Dimg = m_qimg;
	world3Dimg.fill(Qt::black);

	for (int i = 0; i < m_viewSequence->count(); i++) {
		auto item = m_viewSequence->item(i);
		auto id = item->whatsThis();

		if (!(*m_views).contains(id)) {
			ct::logger::error("[JobThread] Failed to generate warpage map. Invalid view ID: %s", id.toStdString().c_str());
			continue;
		}

		auto v = (*m_views)[id];

		jogLaserBasedOnFiducial(v.world.wx, v.world.wy, v.world.wz, "2D");

		//IGocator::instance().snapshot();
		ProfilerManager::instance().snapShot(m_profilerID);
		ProfilerManager::instance().waitAcquisition(m_profilerID, PROFILER_TIMEOUT);

		
		/*if (IGocator::instance().go_info().mode != GO_MODE_PROFILE) {
			ct::logger::error("Expected profile mode, but receive other mode: %d", IGocator::instance().go_info().mode);
			return;
		}*/

		auto wpx = ScaleManager::instance().fov_to_world(v.px);
		auto fontSize = wpx.w / 10;
		auto& profiles = ProfilerManager::instance().getFrame(m_profilerID)->profiles;
		auto average = std::accumulate(profiles.begin(), profiles.end(), 0.0) / profiles.size();
		QString avg_z = QString::number(average, 'f', 2);

		QPainter painter(&m_qimg);
		QPainter painter2(&world3Dimg);

		QPen pen;
		pen.setWidth(5);
		pen.setColor(QColor(0, 255, 127));
		painter.setPen(pen);
		painter2.setPen(pen);
		QBrush brush;
		brush.setColor(QColor(0, 255, 127));
		painter.setBrush(brush);
		painter2.setBrush(brush);
		QFont font = painter.font();
		font.setPointSize(fontSize);
		painter.setFont(font);
		painter2.setFont(font);

		painter.drawText(QPointF((wpx.xmin + wpx.cx) / 2, wpx.cy), avg_z);
		painter2.drawText(QPointF((wpx.xmin + wpx.cx) / 2, wpx.cy), avg_z);
		painter.drawRect(QRect(wpx.xmin, wpx.ymin, wpx.w, wpx.h));
		painter2.drawRect(QRect(wpx.xmin, wpx.ymin, wpx.w, wpx.h));
		painter.end();
		painter2.end();
	}

	m_qimg.save("warp.png");
	world3Dimg.save("warp2.png");
}

void JobThread::centerLaserZ()
{
	auto origin = m_coordinate;

	jogLaser(origin.wx, origin.wy, origin.wz, "2D");

	//IGocator::instance().snapshot();
	ProfilerManager::instance().snapShot(m_profilerID);
	ProfilerManager::instance().waitAcquisition(m_profilerID, PROFILER_TIMEOUT);

	/*if (IGocator::instance().go_info().mode != GO_MODE_PROFILE) {
		ct::logger::error("Expected profile mode, but receive other mode: %d", IGocator::instance().go_info().mode);
		return;
	}*/

	auto& profiles = ProfilerManager::instance().getFrame(m_profilerID)->profiles;
	auto average = std::accumulate(profiles.begin(), profiles.end(), 0.0) / profiles.size();
	m_laserOffset->wz = m_laserOffset->wz - average;

	emit updateLaserOffset();

	//verify
	jogLaser(origin.wx, origin.wy, origin.wz, "2D");
	//IGocator::instance().snapshot();
	ProfilerManager::instance().snapShot(m_profilerID);
	ProfilerManager::instance().waitAcquisition(m_profilerID, PROFILER_TIMEOUT);

	//if (IGocator::instance().go_info().mode != GO_MODE_PROFILE) {
	//	ct::logger::error("Expected profile mode, but receive other mode: %d", IGocator::instance().go_info().mode);
	//	return;
	//}

	auto& profiles2 = ProfilerManager::instance().getFrame(m_profilerID)->profiles;
	auto average2 = std::accumulate(profiles2.begin(), profiles2.end(), 0.0) / profiles2.size();
	ct::logger::info("Laser distance from center: %f", average2);

	jog(origin.wx, origin.wy, origin.wz, "2D");
}

void JobThread::colorCompensation()
{
	if (m_stopRun) return;

	m_rgbOverrides.clear();

	auto& csa = *m_csa;
	auto& optics = *m_optics;
	auto& views = *m_views;

	csa.offset.setX(0);
	csa.offset.setY(0);

	bool autoAdjust = false;
	int intensityDif = 11;

	//check if got one lighting needed to auto adjust
	for (auto& opt : optics) {
		//safe guard
		if (opt.segmentPriority == -1) continue;
		if (!views.contains(csa.viewRef)) continue;
		if (opt.segmentRGBs.size() <= opt.segmentPriority) continue;

		const auto& v = views[csa.viewRef];

		jogView(v);
		autoAdjust = true;
		break;
	}

	if (!autoAdjust) return;

	//get offset
	if (optics.contains(csa.opticID)) {
		auto& opt = optics[csa.opticID];
		
		snapOptic(opt, "", "");
		auto info = waitForImagePreprocessed();
		auto mBuf = info.pImage->id();

		auto root = Common::Directory::getRecipeImagesPath() + "\\CSA\\";
		auto modPath = root.toStdString() + "locator.pat";

		for (int i = 0; i < csa.searchLocator.size(); i++) {

			if (csa.searchLocator[i] == nullptr) continue;

			auto id = csa.searchLocator[i]->getId();
			auto roi = csa.searchLocator[i]->getGeometry();

			MIL_ID mCrop = mtrx::crop(mBuf, roi.x(), roi.y(), roi.width(), roi.height());
			MIL_ID mMono = mtrx::to_mono(mCrop);
			mtrx::BufferCollector bc_mCrop(mCrop);
			mtrx::BufferCollector bc_mMono(mMono);

			mtrx::PatternOutput output;
			output.acceptance_min_score = 50;
			output.certainty_min_score = 80;

			mtrx::find_pattern(mMono, modPath, output);

			if (output.score > 50) {
				auto cx = csa.teachPoints[id].x();
				auto cy = csa.teachPoints[id].y();

				csa.offset.setX(cx - (roi.x() + output.cx));
				csa.offset.setY(cy - (roi.y() + output.cy));

				ct::logger::debug("CSA offset: %f, %f", csa.offset.x(), csa.offset.y());

				emit drawRectFOV("locator", QRectF(roi.x() + output.x, roi.y() + output.y, output.w, output.h), Qt::yellow);
				break;
			}
		}
	}

	for (auto& opt : optics) {
		if (m_stopRun) return;

		//safe guard
		if (opt.segmentPriority == -1) continue;
		if (opt.segmentRGBs.size() <= opt.segmentPriority) continue;

		snapOptic(getMainOptics(), "", "");
		auto info = waitForImagePreprocessed();
		auto mBuf = info.pImage->id();


		auto path = Common::Directory::getRecipeImagesPath().toStdString() + "\\CSA\\" + opt.id.toStdString() + "_segmentReference.png";
		auto segmentPath = Common::Directory::getRecipeImagesPath() + "\\CSA\\" + opt.id + "_segment" + QString::number(opt.segmentPriority + 1) + ".png";

		ct::logger::debug("[CSA] Ref path: %s", path.c_str());
		ct::logger::debug("[CSA] Segment path: %s", segmentPath.toStdString().c_str());

		if (QFile::exists(path.c_str()) && QFile::exists(segmentPath)) {

			ct::logger::debug("[CSA] Get ref image");
			cv::Mat currentImg;
			util::Mil_to_cv(mBuf, currentImg);
			ct::logger::debug("[CSA] Get current image");

			std::ofstream fout("colorCompensation.txt");

			QString currentImgPath = m_rootPath + "/CSA-" + opt.id + "_currentImage.png";
			cv::imwrite(currentImgPath.toStdString(), currentImg);

			cv::Mat segmentImg = cv::imread(segmentPath.toStdString());
			std::vector<cv::Point> segment;
			cvUtil::getSegmentFromColor(segmentImg, segment, cv::Vec3b(255, 0, 0));

			//offset
			for (auto& pos : segment) {
				auto x = pos.x;
				auto y = pos.y;
				pos.x -= csa.offset.x();
				pos.y -= csa.offset.y();
			}

			ct::logger::debug("[CSA] Segment size: %d", segment.size());

			auto stdDev = cvUtil::removeOutliersInSegment(currentImg, segment);

			auto segmentColor = cvUtil::getColorFromSegment(currentImg, segment);

			const auto& segmentOfInterest = segment;

			auto currentColor = segmentColor;
			currentColor[2] = segmentColor[0];
			currentColor[0] = segmentColor[2];

			auto colorToMatch = opt.segmentRGBs[opt.segmentPriority];

			if (stdDev > 0) { //always check for intensity being overtolerance

				if (abs(colorToMatch[0] - currentColor[0]) >= intensityDif || abs(colorToMatch[1] - currentColor[1]) >= intensityDif || abs(colorToMatch[2] - currentColor[2]) >= intensityDif) {
					ct::logger::info("[CSA] Analyzing lighting adjustment for optic: %s - %s", opt.id.toStdString().c_str(), opt.name.toStdString().c_str());
					ct::logger::info("[CSA] Expected colors: %d, %d, %d", colorToMatch[0], colorToMatch[1], colorToMatch[2]);
					ct::logger::info("[CSA] Current colors: %d, %d, %d", currentColor[0], currentColor[1], currentColor[2]);

					QString segmentPath = m_rootPath + "/CSA-" + opt.id + "_segment.png";
					cvUtil::saveSegmentImage(segmentPath.toStdString(), currentImg, segment, cv::Vec3b(255, 0, 0));

					OpticsControl::instance().toggleAllChannels(false);

					//override rgb
					m_rgbOverrides[opt.id].R = colorToMatch[0] - currentColor[0];
					m_rgbOverrides[opt.id].G = colorToMatch[1] - currentColor[1];
					m_rgbOverrides[opt.id].B = colorToMatch[2] - currentColor[2];

					snapOptic(opt, "", "");
					auto cinfo = waitForImagePreprocessed();
					auto mCorrected = cinfo.pImage->id();
					mtrx::offset_intensity(mCorrected, M_RED, m_rgbOverrides[opt.id].R);
					mtrx::offset_intensity(mCorrected, M_GREEN, m_rgbOverrides[opt.id].G);
					mtrx::offset_intensity(mCorrected, M_BLUE, m_rgbOverrides[opt.id].B);

					QString adjustedPath = m_rootPath + "/CSA-" + opt.id + "_adjustedImage.png";

					MbufExportA(adjustedPath.toStdString().c_str(), M_PNG, mCorrected);
				}
			}
			else {
				fout << "No correction needed" << std::endl;
			}

			fout << "Expected colors: " << colorToMatch[0] << ", " << colorToMatch[1] << ", " << colorToMatch[2] << std::endl;
			fout << "Current colors: " << currentColor[0] << ", " << currentColor[1] << ", " << currentColor[2] << std::endl;
		}
	}
}

em::V2d JobThread::getPositionPortabilityPointInMM(int x_px, int y_px)
{
	em::V2d point;

	auto x_mm = ScaleManager::instance().to_horizontal_mm(x_px);
	auto y_mm = ScaleManager::instance().to_vertical_mm(y_px);
	ct::logger::debug("px: %d, %d\n", x_px, y_px);
	ct::logger::debug("mm: %f, %f\n", x_mm, y_mm);

	point.x() = m_coordinate.wx + x_mm;
	point.y() = m_coordinate.wy + y_mm;

	ct::logger::debug("point: %f, %f\n", point.x(), point.y());
	return point;
}

bool JobThread::testPortabilityPatternFeature(QRectF& outputFeature)
{
	auto mainOptics = getMainOptics();

	snapOptic(mainOptics, "portability", "");
	auto info = waitForImagePreprocessed();
	auto mBuf = info.pImage->id();

	auto& por = SystemData::instance()._portability;

	por.ref_info.search_region.compute_extremum();
	auto sr_x = por.ref_info.search_region.xmin;
	auto sr_y = por.ref_info.search_region.ymin;
	auto sr_w = por.ref_info.search_region.w;
	auto sr_h = por.ref_info.search_region.h;

	auto mCrop = mtrx::crop(mBuf, sr_x, sr_y, sr_w, sr_h);
	MIL_ID mMono = mtrx::to_mono(mCrop);

	mtrx::PatternOutput output;
	std::string path = QString(Common::Directory::PortabilityPath() + "PortabilityFeature.mod").toStdString();
	if (!mtrx::find_geometryModel(mMono, path, output))
	{
		ct::logger::debug("pattern not found");
		mtrx::free_buffer(mCrop);
		mtrx::free_buffer(mMono);
		return false;
	}

	printf("output: %f, %f, %f, %f\n", output.x, output.y, output.w, output.h);

	auto locatedPoint = QRectF(sr_x + output.x, sr_y + output.y, output.w, output.h);
	por.located_region.setGeometry(locatedPoint);
	por.located_region.show(); //TODO:

	mtrx::free_buffer(mMono);

	outputFeature = QRectF(locatedPoint.x() + (output.w / 2 + 1), locatedPoint.y() + (output.h / 2 + 1), locatedPoint.width(), locatedPoint.height());
	return true;
}

bool JobThread::testPortabilityCircleFeature(QRectF& outputFeature)
{
	auto mainOptics = getMainOptics();

	CAMManager::instance().frame(m_camID)->viewID = "portability";
	snapOptic(mainOptics, "portability", "");
	auto info = waitForImagePreprocessed();
	auto mBuf = info.pImage->id();

	auto& por = SystemData::instance()._portability;

	por.ref_info.search_region.compute_extremum();
	auto sr_x = por.ref_info.search_region.xmin;
	auto sr_y = por.ref_info.search_region.ymin;
	auto sr_w = por.ref_info.search_region.w;
	auto sr_h = por.ref_info.search_region.h;

	auto mCrop = mtrx::crop(mBuf, sr_x, sr_y, sr_w, sr_h);
	MIL_ID mMono = mtrx::to_mono(mCrop);

	mtrx::Circle circle;
	if (!mtrx::find_circle(circle, mMono, por.ref_info.min_diameter / 2, por.ref_info.max_diameter / 2, mtrx::CircleType::HIGHEST_SCORE))
	{
		ct::logger::debug("circle not found");
		mtrx::free_buffer(mCrop);
		mtrx::free_buffer(mMono);
		return false;
	}

	auto locatedPoint = QRectF(sr_x + circle.cx - circle.radius, sr_y + circle.cy - circle.radius, circle.radius * 2, circle.radius * 2);
	printf("circleFeature: %f, %f, %f\n", circle.x, circle.y, circle.radius * 2);
	por.located_region.setGeometry(locatedPoint);
	SystemData::instance()._portability.located_region.show();

	mtrx::free_buffer(mCrop);
	mtrx::free_buffer(mMono);

	outputFeature = QRectF(sr_x + circle.cx, sr_y + circle.cy, locatedPoint.width(), locatedPoint.height());

	return true;
}

bool JobThread::getCorrectedPortabilityPoint(dat::WorldCoordinate& portabilityPoint, QPointF& PatternSize)
{
	bool featureFound = false;

	if (m_method == 0) //Geometry Model Finder
	{
		featureFound = findPortabilityPattern();
	}
	else if (m_method == 1) //circle finder
	{
		featureFound = findPortabilityCircle();
	}

	if (!featureFound) return false;

	//get offset x,y
	double offsetX = m_locatedPortabilityPos.x() - CAMManager::instance().getWidth("cam1") / 2;
	double offsetY = m_locatedPortabilityPos.y() - CAMManager::instance().getHeight("cam1") / 2;
	auto offset = getPositionPortabilityPointInMM(offsetX, offsetY);

	//offset current point with x,y
	ct::logger::debug("[Portability] Current encoder: %f, %f, %f", m_coordinate.wx, m_coordinate.wy, m_coordinate.wz);

	portabilityPoint = m_coordinate;
	portabilityPoint.wx = offset.x();
	portabilityPoint.wy = offset.y();
	ct::logger::debug("[Portability] Offseted encoder: %f, %f, %f", portabilityPoint.wx, portabilityPoint.wy, portabilityPoint.wz);

	PatternSize.setX(m_locatedPortabilityPos.width());
	PatternSize.setY(m_locatedPortabilityPos.height());
	return true;
}

bool JobThread::getPortabilitySizeDifference(double difference, double& offsetZ)
{
	//need to verify if Z positive is going up, or going down
	if (abs(difference) > 1)
	{
		//if current size is more zoomed in than ref size move camera upwards
		if (difference < 0) offsetZ = 0.01;

		//if current size is less zoomed in than ref size move camera downwards
		if (difference >= 0) offsetZ = -0.01;

		return true;
	}
	else return false;
}

bool JobThread::savePositionPortabilityInfo(PositionPortabilityType type)
{
	if (type == PositionPortabilityType::REFERENCE)
	{
		auto jsonPath = Common::Directory::PortabilityPath() + "RefPositionPortability.json";

		QJsonObject j_root;
		QJsonArray j_array;

		QJsonObject obj;
		QJsonObject srObj, prObj;
		SystemData::instance()._portability.ref_info.search_region.compute_extremum();
		SystemData::instance()._portability.ref_info.learn_region.compute_extremum();
		toJson(SystemData::instance()._portability.ref_info.search_region, srObj);
		toJson(SystemData::instance()._portability.ref_info.learn_region, prObj);

		obj.insert(QStringLiteral("id"), SystemData::instance()._portability.ref_info.id);
		obj.insert(QStringLiteral("search_region"), srObj);
		obj.insert(QStringLiteral("learn_region"), prObj);
		obj.insert(QStringLiteral("feature_searching_method"), SystemData::instance()._portability.ref_info.feature_searching_method);
		obj.insert(QStringLiteral("min_diameter"), SystemData::instance()._portability.ref_info.min_diameter);
		obj.insert(QStringLiteral("max_diameter"), SystemData::instance()._portability.ref_info.max_diameter);
		obj.insert(QStringLiteral("width"), SystemData::instance()._portability.ref_info.width);
		obj.insert(QStringLiteral("height"), SystemData::instance()._portability.ref_info.height);
		obj.insert(QStringLiteral("machine_name"), SystemData::instance()._portability.ref_info.machine_name);
		obj.insert(QStringLiteral("PIC"), SystemData::instance()._portability.ref_info.PIC);
		obj.insert(QStringLiteral("date_created"), SystemData::instance()._portability.ref_info.date_created);
		obj.insert(QStringLiteral("learnt_status"), SystemData::instance()._portability.ref_info.learnt_status);

		QJsonObject pObj;
		toJson(SystemData::instance()._portability.ref_info.portability_point, pObj, true);
		obj.insert(QStringLiteral("ref_portability_point"), pObj);

		j_root.insert(QStringLiteral("RefPositionPortabilityInfos"), obj);

		auto ret = jsonHelper::saveJson(jsonPath, QJsonDocument(j_root));

		if (ret) promptMsg(QStringLiteral("Successfully saved position portability info!"));
		else promptMsg(QStringLiteral("Failed to save position portability info!"));

		return ret;
	}
	else
	{
		auto jsonPath = Common::Directory::PortabilityPath() + "CurPositionPortability.json";

		QJsonObject j_root;
		QJsonArray j_array;

		QJsonObject obj;

		obj.insert(QStringLiteral("id"), SystemData::instance()._portability.current_info.id);
		obj.insert(QStringLiteral("machine_name"), SystemData::instance()._portability.current_info.machine_name);
		obj.insert(QStringLiteral("PIC"), SystemData::instance()._portability.current_info.PIC);
		obj.insert(QStringLiteral("date_created"), SystemData::instance()._portability.current_info.date_created);

		SystemData::instance()._portability.current_info.offset_point =
			SystemData::instance()._portability.current_info.portability_point -
			SystemData::instance()._portability.ref_info.portability_point;

		QJsonObject pObj;
		toJson(SystemData::instance()._portability.current_info.portability_point, pObj, true);
		obj.insert(QStringLiteral("cur_portability_point"), pObj);

		QJsonObject offsetObj;
		toJson(SystemData::instance()._portability.current_info.offset_point, offsetObj, true);
		obj.insert(QStringLiteral("cur_offset"), offsetObj);

		j_root.insert(QStringLiteral("CurPositionPortabilityInfos"), obj);

		auto ret = jsonHelper::saveJson(jsonPath, QJsonDocument(j_root));

		if (ret) promptMsg(QStringLiteral("Successfully saved position portability info!"));
		else promptMsg(QStringLiteral("Failed to save position portability info!"));

		return ret;
	}

}

void JobThread::setPositionPortabilityPoint(PositionPortabilityType type)
{
	auto& por = SystemData::instance()._portability;

	if (type == PositionPortabilityType::REFERENCE) {
		m_method = por.ref_info.feature_searching_method;
	}
	else {
		m_method = por.current_info.feature_searching_method;
	}

	ct::logger::debug("Portability Type: %d", (int)type);

	// 1. correct XY offset - start
	//if feature cant be found we need to prompt warning and stop the process
	dat::WorldCoordinate p;
	QPointF patternSize;
	por.pattern_size = QPointF();
	if (!getCorrectedPortabilityPoint(p, patternSize))
	{
		//emit promptMsg("Feature not found, unable to perform portability!!!");
		return;
	}

	ct::logger::debug("{correctXY (1)}: %f, %f, %f", p.wx, p.wy, p.wz);

	jog(p.wx, p.wy, p.wz, "2D");

	if (!getCorrectedPortabilityPoint(por.position_point, por.pattern_size))
	{
		//emit promptMsg("Feature not found, unable to perform portability!!!");
		return;
	}

	ct::logger::debug("{correctXY (2)}: %f, %f, %f", por.position_point.wx, por.position_point.wy, por.position_point.wz);
	jog(por.position_point.wx, por.position_point.wy, por.position_point.wz, "2D");
	
	// 1. correct XY offset - done
	if (type == PositionPortabilityType::REFERENCE) // save XYZ offset for reference portability
	{
		por.ref_info.width = patternSize.x();
		 por.ref_info.height = patternSize.y();
		 por.ref_info.portability_point = por.position_point;

		 por.ref_info.date_created = QDateTime::currentDateTime().toString().replace(":", "-");
		 por.ref_info.machine_name = QHostInfo::localHostName();
		 por.ref_info.learnt_status = true;

		 SystemData::instance().save(SystemData::Type::PORTABILITY);
		 emit loadPortabilityInfo();
	}
	else if (type == PositionPortabilityType::CURRENT) //correct Z offset
	{
		//portability success and register current portability position to current portability point
		por.current_info.portability_point = por.position_point;
		por.current_info.id = "Cur_Portability_Info";

		por.current_info.date_created = QDateTime::currentDateTime().toString().replace(":", "-");
		por.current_info.machine_name = QHostInfo::localHostName();
		por.current_info.offset_point.wx = por.current_info.portability_point.wx - por.ref_info.portability_point.wx;
		por.current_info.offset_point.wy = por.current_info.portability_point.wy - por.ref_info.portability_point.wy;
		por.current_info.offset_point.wz = por.current_info.portability_point.wz - por.ref_info.portability_point.wz;

		ct::logger::debug("{offset (curPortability)}: %f, %f, %f", por.current_info.offset_point.wx, por.current_info.offset_point.wy, por.current_info.offset_point.wz);
		SystemData::instance().save(SystemData::Type::PORTABILITY);
		emit loadPortabilityInfo();

		auto& por = SystemData::instance()._portability;
		por.num_z_offset_performed = 0;

		for (int i = 0; i < 5; i++)
		{
			if (getCorrectedPortabilityPoint(por.position_point, por.pattern_size))
			{
				double offsetZ = 0;
				if (getPortabilitySizeDifference((por.pattern_size.x() - por.ref_info.width), offsetZ))
				{
					por.num_z_offset_performed++;
					por.position_point.wz = por.position_point.wz + offsetZ;
					ct::logger::debug("{correctZ (%d)}: %f, %f, %f, curSize:%f, refSize:%f", por.num_z_offset_performed, p.wx, p.wy, p.wz, por.pattern_size.x(), SystemData::instance()._portability.ref_info.width);
				}
				else {
					//promptMsg(tr("Feature not found, unable to perform portability!!!"));
					return;
				}
			}
			else
			{
				//promptMsg(tr("Feature not found, unable to perform portability!!!"));
				return;
			}

			jog(por.position_point.wx, por.position_point.wy, por.position_point.wz);
		}
		
		if (por.num_z_offset_performed == 5)
		{
			por.num_z_offset_performed = 0;

			double offsetZ;
			if (getPortabilitySizeDifference((por.pattern_size.x() - SystemData::instance()._portability.ref_info.width), offsetZ))
			{
				// portability failed
				promptMsg("Unable to calibrate feature size to match reference, please make sure the image is focused!!!");
				return;
			}
			else
			{
				//portability success and register current portability position to current portability point
				SystemData::instance()._portability.current_info.id = "Cur_Portability_Info";
				savePositionPortabilityInfo(PositionPortabilityType::CURRENT);
				ct::logger::debug("Done save current Portability Point");
				promptMsg("Performed portability offset successfully!");
			}
		}
	}
}

void JobThread::performLaserAlignment(dat::WorldCoordinate currentPoint, QRectF roi, int camThreshold, int laserThreshold)
{
	//=> Get circle at world respective to camera
	auto origin = currentPoint;

	Common::Directory::createDir("data");
	Common::Directory::createDir("data/laser");

	auto& mainOptics = getMainOptics();
	snapOptic(mainOptics, "", "");
	auto info = waitForImagePreprocessed();
	auto mColor = info.pImage->id();

	auto _blobCtx = MblobAlloc(M_DEFAULT_HOST, M_DEFAULT, M_DEFAULT, M_NULL);
	auto _blobResult = MblobAllocResult(M_DEFAULT_HOST, M_NULL);
	MblobControl(_blobCtx, M_BOX, M_ENABLE);
	MblobControl(_blobCtx, M_FERETS, M_ENABLE);
	MblobControl(_blobCtx, M_CENTER_OF_GRAVITY, M_ENABLE);

	ct::logger::info("[Alignment] Align camera");
	MbufExportA("data/laser/align_camera.png", M_PNG, mColor);

	auto qimg = mtrx::to_qimg(mColor);
	LaserAlignmentImage l;
	l.qimg = qimg;
	l.info = "Alignment: Camera View";
	appendLaserAlignmentImage(l);

	MIL_ID mMono = M_NULL;
	if (mtrx::is_color(mColor)) {
		mMono = mtrx::to_mono(mColor);
	}
	else {
		mMono = mtrx::alloc_buffer(mColor);
		MbufCopy(mColor, mMono);
	}

	auto mBin = mtrx::alloc_buffer(mMono);
	mtrx::BufferCollector bc2(mMono);
	mtrx::BufferCollector bc3(mBin);

	auto threshold = camThreshold;
	MimBinarize(mMono, mBin, M_FIXED + M_LESS, threshold, M_NULL);
	ct::logger::info("[Alignment] Save Binarized camera image");
	MbufExportA("data/laser/align_cameraBin.png", M_PNG, mBin);


	LaserAlignmentImage l2;
	l2.qimg = mtrx::to_qimg(mBin);
	l2.info = "Alignment: Camera Threshold";
	appendLaserAlignmentImage(l2);

	MIL_INT numBlobs = 0;
	MblobCalculate(_blobCtx, mBin, M_NULL, _blobResult);

	auto rect = roi;
	auto diameter = (rect.width() + rect.height()) / 2;
	auto min_diameter = diameter - 100;
	auto max_diameter = diameter + 100;
	MblobSelect(_blobResult, M_EXCLUDE, M_FERET_X, M_LESS_OR_EQUAL, min_diameter, M_NULL);
	MblobSelect(_blobResult, M_EXCLUDE, M_FERET_Y, M_LESS_OR_EQUAL, min_diameter, M_NULL);
	MblobSelect(_blobResult, M_EXCLUDE, M_FERET_X, M_GREATER_OR_EQUAL, max_diameter, M_NULL);
	MblobSelect(_blobResult, M_EXCLUDE, M_FERET_Y, M_GREATER_OR_EQUAL, max_diameter, M_NULL);

	MblobGetResult(_blobResult, M_DEFAULT, M_NUMBER + M_TYPE_MIL_INT, &numBlobs);

	//map to world
	auto img_cx = (float)qimg.width() / 2;
	auto img_cy = (float)qimg.height() / 2;

	double cx = img_cx, cy = img_cy, fx, fy;

	if (numBlobs != 0) {
		MIL_DOUBLE* CogX = new MIL_DOUBLE[numBlobs];
		MIL_DOUBLE* CogY = new MIL_DOUBLE[numBlobs];
		MIL_DOUBLE* FeretX = new MIL_DOUBLE[numBlobs];
		MIL_DOUBLE* FeretY = new MIL_DOUBLE[numBlobs];

		/* Get the results. */
		MblobGetResult(_blobResult, M_DEFAULT, M_CENTER_OF_GRAVITY_X + M_BINARY, CogX);
		MblobGetResult(_blobResult, M_DEFAULT, M_CENTER_OF_GRAVITY_Y + M_BINARY, CogY);
		MblobGetResult(_blobResult, M_DEFAULT, M_FERET_X + M_BINARY, FeretX);
		MblobGetResult(_blobResult, M_DEFAULT, M_FERET_Y + M_BINARY, FeretY);

		for (auto n = 0; n < numBlobs; n++) {
			MosPrintf(MIL_TEXT("Blob #%d: X=%5.1f, Y=%5.1f\n"), (int)n, CogX[n], CogY[n]);
			MosPrintf(MIL_TEXT("Feret #%d: X=%5.1f, Y=%5.1f\n"), (int)n, FeretX[n], FeretY[n]);
			cx = CogX[n];
			cy = CogY[n];
			fx = FeretX[n];
			fy = FeretY[n];
		}

		delete[] CogX; CogX = NULL;
		delete[] CogY; CogY = NULL;
		delete[] FeretX; FeretX = NULL;
		delete[] FeretY; FeretY = NULL;
	}
	else {
		emit promptMsg("Failed to find circle to align for camera!");
		MblobFree(_blobCtx);
		MblobFree(_blobResult);
		return;
	}

	MblobFree(_blobCtx);
	MblobFree(_blobResult);

	auto x_offset = cx - img_cx;
	auto y_offset = cy - img_cy;

	auto x_offset_mm = ScaleManager::instance().to_horizontal_mm(x_offset);
	auto y_offset_mm = ScaleManager::instance().to_vertical_mm(y_offset);

	ct::logger::info("[Camera] Offset: %f, %f\n", x_offset_mm, y_offset_mm);

	auto h_scale = ScaleManager::instance().horizontal_um_per_px();
	auto v_scale = ScaleManager::instance().vertical_um_per_px();

	{
	//=> Get circle at world respective to laser

	if (!ProfilerManager::instance().stop(m_profilerID)) {
		promptMsg("Failed to stop scanning");
		return;
	}

	//IGocator::instance().load_job("test.job");

	auto half_w_mm = ScaleManager::instance().to_mm(CAMManager::instance().getWidth("cam1")) / 2.0;
	auto start = origin;
	start.wx = start.wx - half_w_mm;

	auto end = origin;
	end.wx = end.wx + half_w_mm;

	auto& mainOptics3D = getMainOptics3D();
	auto info = scan("pla", start, end, mainOptics3D);

	/*auto iid = "pla_IMap";
	auto hid = "pla_HeightMap_" + mainOptics3D.id;

	if (mainOptics3D.exposureMode == ct::s_parallel) {
		hid = "pla_HeightMap_" + mainOptics3D.id + "E1";
	}*/

	auto mHeightMap = info.pHeightMap->id();
	auto mIntensity = info.pImage->id();

	auto _blobCtx2 = MblobAlloc(M_DEFAULT_HOST, M_DEFAULT, M_DEFAULT, M_NULL);
	auto _blobResult2 = MblobAllocResult(M_DEFAULT_HOST, M_NULL);
	MblobControl(_blobCtx2, M_BOX, M_ENABLE);
	MblobControl(_blobCtx2, M_FERETS, M_ENABLE);
	MblobControl(_blobCtx2, M_CENTER_OF_GRAVITY, M_ENABLE);

	auto w = mtrx::get_width(mHeightMap);
	auto h = mtrx::get_height(mHeightMap);

	ct::logger::info("[Alignment] Save laser image");
	MbufExportA("data/laser/align_laser.png", M_PNG, mHeightMap);
	LaserAlignmentImage l3;
	l3.qimg.load("data/laser/align_laser.png");
	l3.info = "Alignment: Laser View";
	appendLaserAlignmentImage(l3);

	MIL_ID mMono2 = M_NULL;
	if (mtrx::is_color(mHeightMap)) {
		mMono2 = mtrx::to_mono(mHeightMap);
	}
	else {
		mMono2 = mtrx::alloc_buffer(mHeightMap);
		MbufCopy(mHeightMap, mMono2);
	}

	auto mBin2 = mtrx::alloc_buffer(mMono2);

	mtrx::BufferCollector b2c2(mMono2);
	mtrx::BufferCollector b2c3(mBin2);

	MimBinarize(mMono2, mBin2, M_FIXED + M_LESS, threshold, M_NULL);

	ct::logger::info("[Alignment] Save binarized laser image");
	MbufExportA("data/laser/align_laserBin.png", M_PNG, mBin2);
	LaserAlignmentImage l4;
	l4.qimg.load("data/laser/align_laserBin.png");
	l4.info = "Alignment: Laser Threshold";
	appendLaserAlignmentImage(l4);


	MIL_INT numBlobs2 = 0;
	MblobCalculate(_blobCtx2, mBin2, M_NULL, _blobResult2);

	auto diameter = (rect.width() + rect.height()) / 2;
	auto min_diameter = diameter - 100;
	auto max_diameter = diameter + 100;
	MblobSelect(_blobResult2, M_EXCLUDE, M_FERET_X, M_LESS_OR_EQUAL, min_diameter, M_NULL);
	MblobSelect(_blobResult2, M_EXCLUDE, M_FERET_Y, M_LESS_OR_EQUAL, min_diameter, M_NULL);
	MblobSelect(_blobResult2, M_EXCLUDE, M_FERET_X, M_GREATER_OR_EQUAL, max_diameter, M_NULL);
	MblobSelect(_blobResult2, M_EXCLUDE, M_FERET_Y, M_GREATER_OR_EQUAL, max_diameter, M_NULL);

	MblobGetResult(_blobResult2, M_DEFAULT, M_NUMBER + M_TYPE_MIL_INT, &numBlobs2);

	auto img_cx = (float)w / 2;
	auto img_cy = (float)h / 2;

	double cx = img_cx, cy = img_cy, fx, fy;

	if (numBlobs2 != 0) {
		MIL_DOUBLE* CogX = new MIL_DOUBLE[numBlobs2];
		MIL_DOUBLE* CogY = new MIL_DOUBLE[numBlobs2];
		MIL_DOUBLE* FeretX = new MIL_DOUBLE[numBlobs2];
		MIL_DOUBLE* FeretY = new MIL_DOUBLE[numBlobs2];

		/* Get the results. */
		MblobGetResult(_blobResult2, M_DEFAULT, M_CENTER_OF_GRAVITY_X + M_BINARY, CogX);
		MblobGetResult(_blobResult2, M_DEFAULT, M_CENTER_OF_GRAVITY_Y + M_BINARY, CogY);
		MblobGetResult(_blobResult2, M_DEFAULT, M_FERET_X + M_BINARY, FeretX);
		MblobGetResult(_blobResult2, M_DEFAULT, M_FERET_Y + M_BINARY, FeretY);

		for (auto n = 0; n < numBlobs2; n++) {
			MosPrintf(MIL_TEXT("Blob #%d: X=%5.1f, Y=%5.1f\n"), (int)n, CogX[n], CogY[n]);
			MosPrintf(MIL_TEXT("Feret #%d: X=%5.1f, Y=%5.1f\n"), (int)n, FeretX[n], FeretY[n]);
			cx = CogX[n];
			cy = CogY[n];
			fx = FeretX[n];
			fy = FeretY[n];
		}

		delete[] CogX; CogX = NULL;
		delete[] CogY; CogY = NULL;
		delete[] FeretX; FeretX = NULL;
		delete[] FeretY; FeretY = NULL;
	}
	else {
		promptMsg("Failed to find circle to align for laser!");
		MblobFree(_blobCtx2);
		MblobFree(_blobResult2);
		return;
	}

	MblobFree(_blobCtx2);
	MblobFree(_blobResult2);

	//map to world
	auto x_offset = cx - img_cx; 
	auto y_offset = cy - img_cy;

	auto x_offset_mm = util::px_to_mm(x_offset, h_scale);
	auto y_offset_mm = util::px_to_mm(y_offset, v_scale);

	ct::logger::info("[Laser] Offset: %f, %f\n", x_offset_mm, y_offset_mm);

	//=>Update offset
	auto& laserOffset = *m_laserOffset;
	laserOffset.wx += x_offset_mm;
	laserOffset.wy += y_offset_mm;

	ct::logger::info("[Laser] New offset (mm): %f, %f\n", laserOffset.wx, laserOffset.wy);

	}

	jog(origin.wx, origin.wy, origin.wz, "2D");

	emit laserAlignmentDone();
}

void JobThread::captureAlignmentImages(dat::WorldCoordinate currentPoint,int camThreshold,int laserThreshold)
{
	auto origin = currentPoint;

	Common::Directory::createDir("data");
	Common::Directory::createDir("data/laser");

	// =========================
	// 1) Capture 2D camera image
	// =========================
	auto& mainOptics = getMainOptics();
	snapOptic(mainOptics, "", "");

	auto info2D = waitForImagePreprocessed();
	if (!info2D.pImage) {
		emit promptMsg("captureAlignmentImages: 2D image is null");
		return;
	}

	MIL_ID mColor = info2D.pImage->id();

	ct::logger::info("[captureAlignmentImages] Save camera image");
	MbufExportA("data/laser/align_camera.png", M_PNG, mColor);

	// UI preview: camera
	{
		LaserAlignmentImage l;
		l.qimg = mtrx::to_qimg(mColor);
		l.info = "Alignment: Camera View";
		appendLaserAlignmentImage(l);
	}

	// Camera binarized save + preview
	{
		MIL_ID mMono = M_NULL;
		if (mtrx::is_color(mColor)) {
			mMono = mtrx::to_mono(mColor);
		}
		else {
			mMono = mtrx::alloc_buffer(mColor);
			MbufCopy(mColor, mMono);
		}

		auto mBin = mtrx::alloc_buffer(mMono);
		mtrx::BufferCollector bcMono(mMono);
		mtrx::BufferCollector bcBin(mBin);

		MimBinarize(mMono, mBin, M_FIXED + M_LESS, camThreshold, M_NULL);

		ct::logger::info("[captureAlignmentImages] Save camera bin image");
		MbufExportA("data/laser/align_cameraBin.png", M_PNG, mBin);

		LaserAlignmentImage l2;
		l2.qimg = mtrx::to_qimg(mBin);
		l2.info = "Alignment: Camera Threshold";
		appendLaserAlignmentImage(l2);
	}

	// =========================
	// 2) Capture 3D laser scan (heightmap)
	// =========================
	if (!ProfilerManager::instance().stop(m_profilerID)) {
		emit promptMsg("captureAlignmentImages: Failed to stop scanning");
		return;
	}

	auto half_w_mm =
		ScaleManager::instance().to_mm(CAMManager::instance().getWidth("cam1")) / 2.0;

	auto start = origin; start.wx -= half_w_mm;
	auto end = origin; end.wx += half_w_mm;

	auto& mainOptics3D = getMainOptics3D();
	auto info3D = scan("pla", start, end, mainOptics3D);

	if (!info3D.pHeightMap) {
		emit promptMsg("captureAlignmentImages: HeightMap is null");
		return;
	}

	MIL_ID mHeightMap = info3D.pHeightMap->id();

	ct::logger::info("[captureAlignmentImages] Save laser image");
	MbufExportA("data/laser/align_laser.png", M_PNG, mHeightMap);

	// UI preview: laser
	{
		LaserAlignmentImage l3;
		l3.qimg.load("data/laser/align_laser.png");
		l3.info = "Alignment: Laser View";
		appendLaserAlignmentImage(l3);
	}

	// Laser binarized save + preview
	{
		MIL_ID mMono2 = M_NULL;
		if (mtrx::is_color(mHeightMap)) {
			mMono2 = mtrx::to_mono(mHeightMap);
		}
		else {
			mMono2 = mtrx::alloc_buffer(mHeightMap);
			MbufCopy(mHeightMap, mMono2);
		}

		auto mBin2 = mtrx::alloc_buffer(mMono2);
		mtrx::BufferCollector bcMono2(mMono2);
		mtrx::BufferCollector bcBin2(mBin2);

		MimBinarize(mMono2, mBin2, M_FIXED + M_LESS, laserThreshold, M_NULL);

		ct::logger::info("[captureAlignmentImages] Save laser bin image");
		MbufExportA("data/laser/align_laserBin.png", M_PNG, mBin2);

		LaserAlignmentImage l4;
		l4.qimg.load("data/laser/align_laserBin.png");
		l4.info = "Alignment: Laser Threshold";
		appendLaserAlignmentImage(l4);
	}

	// =========================
	// 3) Go back + done
	// =========================
	jog(origin.wx, origin.wy, origin.wz, "2D");

	ct::logger::info("[captureAlignmentImages] Done");
	emit captureAlignmentDone();
}

void JobThread::performGuidedLaserAlignment(dat::WorldCoordinate currentPoint)
{
	// get Camera view
	auto origin = currentPoint;

	Common::Directory::createDir(Common::Directory::LocalPath);
	Common::Directory::createDir(Common::Directory::LocalPath + "laser");
	Common::Directory::createDir(Common::Directory::LocalPath + "laser/guidedLaser");
	QString guidedLaserPath = Common::Directory::LocalPath + "laser/guidedLaser/";

	auto& mainOptics = getMainOptics();
	snapOptic(mainOptics, "", "");
	auto info = waitForImagePreprocessed();
	auto mColor = info.pImage->id();

	ct::logger::info("[Guided_Alignment] Save camera Img");
	QString camImgPath = guidedLaserPath + "camImage.png";
	MbufExportA(camImgPath.toStdString().c_str(), M_PNG, mColor);

	//get Laser View
	auto half_w_mm = ScaleManager::instance().to_mm(CAMManager::instance().getWidth("cam1")) / 2.0;
	auto start = origin;
	start.wx -= half_w_mm;

	auto end = origin;
	end.wx += half_w_mm;

	auto& mainOptics3D = getMainOptics3D();
	auto info3d = scan("vla", start, end, mainOptics3D);
	if (info3d.viewID.isEmpty()) {
		return;
	}

	/*auto iid = "vla_IMap";
	auto hid = "vla_HeightMap_" + mainOptics3D.id;

	if (mainOptics3D.exposureMode == ct::s_parallel) {
		hid = "pla_HeightMap_" + mainOptics3D.id + "E1";
	}*/

	MIL_ID mIntensity = info3d.pImage->id();

	ct::logger::info("[Guided_Alignment] Save Laser Imap");
	QString imapImgPath = guidedLaserPath + "laser_Intensity.png";
	MbufSaveA(imapImgPath.toStdString().c_str(), mIntensity);

	ct::logger::info("[Guided_Alignment] Save Laser HeightMap");
	MIL_ID mHeightMap = info3d.pHeightMap->id();
	QString hmapImgPath = guidedLaserPath + "laser_heightMap.tiff";
	MbufSaveA(hmapImgPath.toStdString().c_str(), mHeightMap);

	if (!ProfilerManager::instance().stop(m_profilerID)) {
		emit promptMsg("Failed to stop scanning");
	}

	jog(origin.wx, origin.wy, origin.wz, "2D");

	// after done display guided alignment Tab
	emit guidedLaserAlignmentDone();
}

void JobThread::verifyLaserAlignment(dat::WorldCoordinate currentPoint)
{
	QDir dir;
	QString path = "data/laser/";
	dir.mkdir(path);

	//=> Verify 2D to 3D alignment

	auto origin = currentPoint;

	auto& mainOptics = getMainOptics();
	snapOptic(mainOptics, "", "");
	auto info = waitForImagePreprocessed();
	auto mBuf = info.pImage->id();

	auto w = mtrx::get_width(mBuf);
	auto h = mtrx::get_height(mBuf);
	auto cx = w / 2;
	auto cy = h / 2;
	int crossSize = 100;

	auto qimg = mtrx::to_qimg(mBuf);
	util::drawCross(cx, cy, crossSize, qimg);
	qimg.save(path + "camera_view.png");


	LaserAlignmentImage l;
	l.qimg = qimg;
	l.info = "Camera View";
	appendLaserAlignmentImage(l);

	//IGocator::instance().load_job("test.job");

	auto half_w_mm = ScaleManager::instance().to_mm(CAMManager::instance().getWidth("cam1")) / 2.0;
	auto start = origin;
	start.wx -= half_w_mm;

	auto end = origin;
	end.wx += half_w_mm;

	auto& mainOptics3D = getMainOptics3D();
	auto info3d = scan("vla", start, end, mainOptics3D);
	if (info3d.viewID.isEmpty()) {
		return;
	}
	
	auto iid = "vla_IMap";
	auto hid = "vla_HeightMap_" + mainOptics3D.id;

	if (mainOptics3D.exposureMode == ct::s_parallel) {
		hid = "pla_HeightMap_" + mainOptics3D.id + "E1";
	}

	auto mIntensity = info3d.pImage->id();
	MbufSaveA((path + "laser_Intensity.tiff").toUtf8().constData(), mIntensity);
	auto qimgi = mtrx::to_qimg(mIntensity);
	auto cxi = qimgi.width() / 2;
	auto cyi = qimgi.height() / 2;
	util::drawCross(cxi, cyi, 60, qimgi);

	qimgi.save(path + "laser_Intensity.png");
	LaserAlignmentImage l3;
	l3.qimg = qimgi;
	l3.info = "Laser Intensity";
	appendLaserAlignmentImage(l3);


	auto mHeightMap = info3d.pHeightMap->id();
	MbufSaveA((path + "laser_heightMap.tiff").toUtf8().constData(), mHeightMap);
	auto qimgh = mtrx::to_qimg(mHeightMap);
	auto cxh = qimgh.width() / 2;
	auto cyh = qimgh.height() / 2;
	util::drawCross(cxh, cyh, 60, qimgh);

	qimgh.save(path + "laser_heightMap.png");

	LaserAlignmentImage l2;
	l2.qimg = qimgh;
	l2.info = "Laser HeightMap";
	appendLaserAlignmentImage(l2);

	if (!ProfilerManager::instance().stop(m_profilerID)) {
		emit promptMsg("Failed to stop scanning");
	}

	jog(origin.wx, origin.wy, origin.wz, "2D");

	emit verifyLaserAlignmentDone();
}


/* ------------------------------------------------------- Profiler Scan Test */

namespace {

	/*
	* Height statistics taken from the RAW batch, where 0 means "no measurement".
	*
	* This exists because min/max off the PROCESSED height map is not a measurement of
	* anything. rotate_heightMap resizes with M_BICUBIC, and interpolating between a real
	* height (~28000) and an invalid marker (0) rings past both ends of the real range - on
	* Codetrace-CK that turned a true maximum of 32865 into a reported 45572, i.e. +10.2 mm
	* on a sensor with +/-7.3 mm of range. Percentiles off the raw buffer are immune to that.
	*
	* A 65536-bin histogram is the cheapest exact way to get percentiles out of uint16, and
	* one pass over a 56 Mpx batch costs tens of milliseconds - irrelevant for a diagnostic.
	*/
	struct HeightStats {
		bool    ok = false;
		quint64 total = 0;
		quint64 valid = 0;
		int     lo = 0, p01 = 0, p50 = 0, p99 = 0, hi = 0;   //grey levels
	};

	HeightStats rawHeightStats(MIL_ID mBuf)
	{
		HeightStats s;
		if (mBuf == M_NULL) return s;

		const MIL_INT w = mtrx::get_width(mBuf);
		const MIL_INT h = mtrx::get_height(mBuf);
		if (w <= 0 || h <= 0) return s;

		MIL_UINT16* base = M_NULL;
		MIL_INT pitch = 0;
		MbufInquire(mBuf, M_HOST_ADDRESS, &base);
		MbufInquire(mBuf, M_PITCH, &pitch);
		if (!base) return s;

		std::vector<quint64> hist(65536, 0);
		for (MIL_INT r = 0; r < h; ++r) {
			const MIL_UINT16* line = base + r * pitch;
			for (MIL_INT c = 0; c < w; ++c) ++hist[line[c]];
		}

		s.total = quint64(w) * quint64(h);
		s.valid = s.total - hist[0];
		s.ok = true;
		if (s.valid == 0) return s;

		auto pct = [&](double frac) {
			const quint64 target = quint64(double(s.valid) * frac);
			quint64 run = 0;
			for (int g = 1; g < 65536; ++g) {
				run += hist[g];
				if (run >= target) return g;
			}
			return 65535;
		};

		s.p01 = pct(0.01);
		s.p50 = pct(0.50);
		s.p99 = pct(0.99);
		for (int g = 1; g < 65536; ++g)  { if (hist[g]) { s.lo = g; break; } }
		for (int g = 65535; g >= 1; --g) { if (hist[g]) { s.hi = g; break; } }
		return s;
	}

} // namespace

void JobThread::profilerScanTest(double distance_mm, bool positiveDir, QString optic3DId,
	bool saveImages, bool returnToStart)
{
	QString report;
	QTextStream out(&report);

	//Stands in for the profiler diagnostics block until finish(), so the values it prints
	//are the ones in force when the run ended rather than before it was configured.
	static const QString KY_DIAG_TOKEN = QStringLiteral("<<PROFILER_DIAGNOSTICS>>");

	auto say = [&](const QString& line) {
		ct::logger::info("[ProfilerTest] %s", line.toUtf8().constData());
		emit profilerScanTestProgress(line);
	};
	auto section = [&](const QString& title) {
		out << "\n" << title << "\n" << QString(title.size(), QLatin1Char('-')) << "\n";
	};
	auto yesNo = [](bool b) { return b ? QStringLiteral("yes") : QStringLiteral("no"); };

	// The output folder is created FIRST so that even a refusal has somewhere to land.
	const QDateTime now = QDateTime::currentDateTime();
	const QString stamp = now.toString(QStringLiteral("yyyyMMdd_HHmmss"));
	const QString dir = Common::Directory::getRecipeImagesPath() + QStringLiteral("ProfilerTest/") + stamp + "/";
	const QString reportPath = dir + QStringLiteral("report.txt");

	if (!QDir().mkpath(dir)) {
		ct::logger::error("[ProfilerTest] Could not create output folder: %s", dir.toUtf8().constData());
	}

	//Resolved in the PROFILER section below; declared here so finish() can reach it.
	IProfiler* profiler = nullptr;

	// Single exit point: writes the report and reports back, whatever happened.
	auto finish = [&](bool scanRan, const QString& summary) {
		section(QStringLiteral("WHERE TO CHANGE WHAT"));
		out << "  Scan speed (mm/s)      Config page -> X 3D velocity -> Update Velocity X.\n"
			<< "                         Applies immediately and saves to recipeConfig.json\n"
			<< "                         ('x_3d_velocity'). No restart needed.\n"
			<< "  Scan axis              Config page -> Recipe Configuration -> Line Scan Axis.\n"
			<< "                         Must be saved with the recipe or it reverts.\n"
			<< "  Exposure, gain,        3D Optics page, per optics ID. Applied on every scan.\n"
			<< "  divider, threshold\n"
			<< "  IP, ports, encoder     3D Optics page -> Profiler Hardware. Applied on connect.\n"
			<< "  mode, min input time\n"
			<< "  Trigger mode           Read-only by design. Exactly one correct value (2 =\n"
			<< "                         encoder) for this machine. Edit keyence.json only if\n"
			<< "                         you mean it, then reconnect.\n"
			<< "  Sampling frequency,    Navigator only. Persists into every later Pogo run and\n"
			<< "  profile data interval  nothing here detects a change. The interval changes the\n"
			<< "                         point count, which invalidates X pitch and laser FOV.\n"
			<< "  Y pitch (um/trigger)   3D Optics page -> Profiler Hardware -> Y Pitch. Applies\n"
			<< "                         on connect, and the image maths reads the same value.\n"
			<< "  X pitch, laser FOV,    Code constants - ImageManager.cpp and the two copies of\n"
			<< "  batch max              laser_fov_mm. Need a rebuild.\n"
			<< "  lightSensitivity,      On the 3D Optics page and saved to the recipe, but DEAD\n"
			<< "  peakSensitivity,       on the Keyence path - they reach Profiler_SSZN only.\n"
			<< "  peakSelection,         Tuning them here has no effect on this sensor.\n"
			<< "  upper/lowerLaserLimit\n";

		out << "\n" << QString(78, QLatin1Char('=')) << "\n";
		out << "VERDICT: " << summary << "\n";

		report.replace(KY_DIAG_TOKEN, profiler
			? profiler->diagnostics()
			: QStringLiteral("  (no profiler - nothing to report)"));

		QFile f(reportPath);
		if (f.open(QIODevice::WriteOnly | QIODevice::Text)) {
			QTextStream ts(&f);
			ts << report;
			f.close();
			ct::logger::info("[ProfilerTest] Report written: %s", reportPath.toUtf8().constData());
		}
		else {
			ct::logger::error("[ProfilerTest] Could not write %s", reportPath.toUtf8().constData());
		}

		emit profilerScanTestDone(scanRan, reportPath, summary);
	};

	//--------------------------------------------------------------- header
	out << QString(78, QLatin1Char('=')) << "\n";
	out << "PROFILER SCAN TEST\n";
	out << QString(78, QLatin1Char('=')) << "\n";
	out << "  When            : " << now.toString(QStringLiteral("yyyy-MM-dd HH:mm:ss")) << "\n";
	out << "  Machine         : " << QHostInfo::localHostName() << "\n";
	out << "  Recipe          : " << Common::Directory::CurrentRecipe << "\n";
	out << "  Output folder   : " << dir << "\n";

	section(QStringLiteral("WHAT WAS REQUESTED"));
	out << "  Scan distance   : " << QString::number(distance_mm, 'f', 3) << " mm\n";
	out << "  Direction       : " << (positiveDir ? QStringLiteral("+X") : QStringLiteral("-X")) << "\n";
	out << "  Optics 3D ID    : " << (optic3DId.isEmpty() ? QStringLiteral("(auto)") : optic3DId) << "\n";
	out << "  Save images     : " << yesNo(saveImages) << "\n";
	out << "  Return to start : " << yesNo(returnToStart) << "\n";

	//------------------------------------------------------------- profiler
	section(QStringLiteral("PROFILER"));

	const QString api = ProfilerManager::instance().getAPI();
	out << "  API             : " << (api.isEmpty() ? QStringLiteral("(none)") : api) << "\n";
	out << "  Profiler ID     : " << m_profilerID << "\n";

	QStringList blockers;

	if (ProfilerManager::instance().keys().contains(m_profilerID)) {
		profiler = ProfilerManager::instance().profiler(m_profilerID);
	}

	if (!profiler) {
		out << "  Status          : NOT CONFIGURED - no profiler with this ID exists\n";
		blockers << QStringLiteral("no profiler is configured under ID '%1'").arg(m_profilerID);
	}
	else {
		const bool connected = ProfilerManager::instance().isConnected(m_profilerID);
		out << "  Status          : " << (connected ? QStringLiteral("connected") : QStringLiteral("DISCONNECTED")) << "\n";

		/*
		* Placeholder, substituted in finish(). Batch count, points per profile and the
		* measured FOV are only learned when the profiler is configured and armed, which
		* happens further down - printing diagnostics() here reported them as 0 and 1000,
		* which reads as a fault rather than as "not known yet". Filling it in at the end
		* means a completed scan shows the real values and a refusal shows the pre-scan
		* state, which is the truth in both cases.
		*/
		out << "\n" << KY_DIAG_TOKEN << "\n";

		if (!connected) blockers << QStringLiteral("the profiler is not connected");
	}

	//------------------------------------------------- recipe optics3D entry
	section(QStringLiteral("RECIPE OPTICS 3D"));

	/*
	* A POINTER, never a copy. OpticsInfo3D::operator= (Common/OpticsInfo.h) silently omits
	* divider, lightSensitivity, peakSensitivity and peakSelection, so `optic = o` would leave
	* divider at its default of 1 and push that to the controller no matter what the recipe
	* says - and the report would print the 1 as though it were the recipe's value.
	* JobThread::scan() takes a const reference for the same reason.
	*/
	const OpticsInfo3D* optic = nullptr;

	if (!m_optics3D || m_optics3D->isEmpty()) {
		out << "  No optics3D entries exist in this recipe.\n";
		out << "  JobThread::getMainOptics3D() has no return path when nothing matches, so the\n";
		out << "  scan path would be undefined behaviour. Create at least one entry on the\n";
		out << "  3D Optics page.\n";
		blockers << QStringLiteral("the recipe has no optics3D entries");
	}
	else {
		// What the scan would have used, and what rotate_heightMap independently uses. These
		// can disagree, and when they do the divider that reaches the image maths is not the
		// one you set on the entry you were looking at.
		QString autoPick;
		for (const auto& o : *m_optics3D) {
			if (o.intensity) { autoPick = o.id; break; }
		}
		const QString firstInHash = m_optics3D->begin()->id;

		if (optic3DId.isEmpty()) optic3DId = autoPick;

		for (const auto& o : *m_optics3D) {
			if (o.id == optic3DId) { optic = &o; break; }
		}

		out << "  Entries in recipe        : " << m_optics3D->size() << "\n";
		out << "  Used for this scan       : " << (optic ? optic->id : QStringLiteral("(not found)")) << "\n";
		out << "  getMainOptics3D() picks  : " << (autoPick.isEmpty() ? QStringLiteral("(none has intensity=true)") : autoPick) << "\n";
		out << "  rotate_heightMap uses    : " << firstInHash << "   (first entry in the hash, for the divider)\n";

		if (autoPick.isEmpty()) {
			out << "\n  WARNING: no optics3D entry has intensity=true. getMainOptics3D() falls off\n";
			out << "  the end of the function in that case (the C4715 in every build).\n";
			blockers << QStringLiteral("no optics3D entry has intensity=true");
		}
		if (!optic) {
			blockers << QStringLiteral("optics3D ID '%1' was not found in this recipe").arg(optic3DId);
		}
		if (optic && !autoPick.isEmpty() && autoPick != optic->id) {
			out << "\n  NOTE: a normal scan would use '" << autoPick << "', not '" << optic->id << "'.\n";
		}
		if (optic && firstInHash != optic->id) {
			out << "  NOTE: the height map is stretched using '" << firstInHash << "' divider, not '"
				<< optic->id << "' divider. Set the divider on every entry.\n";
		}

		if (optic) {
			out << "\n  exposure        : " << QString::number(optic->exposure, 'f', 1) << " us\n";
			out << "  exposureMode    : " << optic->exposureMode << "\n";
			out << "  gain            : " << QString::number(optic->gain, 'f', 1) << "   (sent as Dynamic Range 1-9)\n";
			out << "  lineThreshold   : " << QString::number(optic->lineThreshold, 'f', 1) << "   (sent as Peak Sensitivity 1-5)\n";
			out << "  divider         : " << optic->divider << "\n";
			out << "  intensity       : " << yesNo(optic->intensity) << "\n";

			if (optic->divider < 1) {
				out << "\n  REFUSING: divider is " << optic->divider << ". ImageManager computes the height map\n";
				out << "  height as h * pitch * divider, so 0 gives a zero-height MbufAlloc2d, a null\n";
				out << "  buffer, and a dead ImageManager thread. Set it to 1 or more on EVERY entry.\n";
				blockers << QStringLiteral("optics3D '%1' has divider %2 (must be 1 or more)").arg(optic->id).arg(optic->divider);
			}
		}
	}

	//----------------------------------------------------------- geometry
	section(QStringLiteral("GEOMETRY AND MOTION"));

	const bool scanAlongY = SystemData::instance().isLineScanAxisY();
	int speed2d = 0, speed3d = 0;
	getXSpeed(speed2d, speed3d);

	out << "  Line scan axis  : " << (scanAlongY ? QStringLiteral("Y") : QStringLiteral("X")) << "\n";
	out << "  3D scan speed   : " << speed3d << " mm/s   (X 3D velocity, from the recipe config)\n";
	out << "  X pitch         : 5.000 um    (KEYENCE_X_PITCH_UM, ImageManager.cpp - code constant)\n";

	// Read from the driver, which is the same source the image maths uses - so this figure
	// cannot drift from the one the height map was actually stretched by.
	const double linePitchUm = ProfilerManager::instance().getLinePitchUm();
	out << "  Line pitch      : " << QString::number(linePitchUm, 'f', 3)
		<< " um   (yPitchUm x divider, from keyence.json)\n";
	out << "                                 Editable on the 3D Optics page under Profiler\n";
	out << "                                 Hardware. Single source of truth: rotate_heightMap\n";
	out << "                                 reads this same value, so they cannot disagree.\n";
	out << "  Laser offset    : not applied - this test jogs from the current position with no\n";
	out << "                    2D/3D offset, so an uncalibrated laserConfig.json cannot skew it.\n";

	if (scanAlongY) {
		out << "\n  NOTE: line scan axis is Y but this test always scans along X. On this machine\n";
		out << "  the axis should be X - check the Config page.\n";
	}

	//---------------------------------------------------------- pre-flight
	section(QStringLiteral("PRE-FLIGHT"));

	if (distance_mm <= 0.0) {
		blockers << QStringLiteral("scan distance must be greater than zero");
	}

	if (profiler) {
		QString reason;
		if (!profiler->isSafeToScan(reason)) {
			blockers << reason;
		}
	}

	if (!blockers.isEmpty()) {
		out << "  SCAN NOT RUN. " << blockers.size() << " blocker(s):\n\n";
		for (int i = 0; i < blockers.size(); ++i) {
			out << "    " << (i + 1) << ". " << blockers[i] << "\n";
		}
		out << "\n  Nothing was moved, armed or written apart from this report.\n";

		say(QStringLiteral("Scan not run - %1 blocker(s). Report: %2").arg(blockers.size()).arg(reportPath));
		finish(false, QStringLiteral("SCAN NOT RUN - %1").arg(blockers.first()));
		return;
	}

	out << "  All checks passed.\n";

	//--------------------------------------------------------------- scan
	section(QStringLiteral("SCAN"));

	/*
	* Clear any stale stop flag before moving. m_stopRun survives a failed or aborted run, and
	* jog() -> waitAxis() honours it - so the scan move would return early, the gantry would
	* travel less than asked, and every distance-derived number in the report below would be
	* quietly wrong. That is worse here than in production: this tool exists to measure.
	*
	* Pressing Run is explicit new intent, the same reasoning as JobThread::jogUser() and the
	* jog helpers, which clear it for exactly this reason.
	*/
	if (m_stopRun) {
		ct::logger::info("[ProfilerTest] Clearing a stale stop flag left by an earlier run");
		out << "  NOTE: a stop flag from an earlier run was still set; cleared so the scan move\n";
		out << "  is not cut short. Nothing else about that earlier run is affected.\n\n";
	}
	m_stopRun = false;

	const dat::WorldCoordinate origin = SystemData::instance().currentCoordinate();
	const double sign = positiveDir ? 1.0 : -1.0;

	// scan() hardcodes a POSITIVE 6 mm overshoot, so a -X scan there stops 6 mm short and the
	// batch can never fill. Here the overshoot follows the direction. Its purpose is to
	// guarantee the batch fills before the move ends, which is why it is past the end point.
	const double overshoot_mm = 6.0;
	const double endX = origin.wx + sign * distance_mm;
	const double targetX = endX + sign * overshoot_mm;

	out << "  Start (encoder) : X=" << QString::number(origin.wx, 'f', 3)
		<< "  Y=" << QString::number(origin.wy, 'f', 3)
		<< "  Z=" << QString::number(origin.wz, 'f', 3) << "\n";
	out << "  Scan end        : X=" << QString::number(endX, 'f', 3) << "\n";
	out << "  Move target     : X=" << QString::number(targetX, 'f', 3)
		<< "   (scan end plus a " << QString::number(overshoot_mm, 'f', 1) << " mm overshoot so the batch always fills)\n";

	say(QStringLiteral("Configuring profiler..."));

	QElapsedTimer step;
	step.start();

	ProfilerManager::instance().stop(m_profilerID);
	const bool okIntensity = ProfilerManager::instance().enableIntensityMap(m_profilerID, optic->intensity);
	const bool okMode = ProfilerManager::instance().setExposureMode(m_profilerID, IProfiler::SINGLE);
	const bool okExposure = ProfilerManager::instance().setExposure(m_profilerID, optic->exposure);
	const bool okGain = ProfilerManager::instance().setGain(m_profilerID, optic->gain);
	const bool okDivider = ProfilerManager::instance().setDivider(m_profilerID, optic->divider);
	const bool okThreshold = ProfilerManager::instance().setLaserLineThreshold(m_profilerID, optic->lineThreshold);
	const bool okLength = ProfilerManager::instance().setScanLength(m_profilerID, distance_mm);
	const qint64 configMs = step.elapsed();

	out << "\n  Configuration (all at RUNNING depth, applied per scan):\n";
	out << "    enableIntensityMap    : " << yesNo(okIntensity) << "\n";
	out << "    setExposureMode SINGLE: " << yesNo(okMode) << "\n";
	out << "    setExposure           : " << yesNo(okExposure) << "\n";
	out << "    setGain               : " << yesNo(okGain) << "\n";
	out << "    setDivider            : " << yesNo(okDivider) << "\n";
	out << "    setLaserLineThreshold : " << yesNo(okThreshold) << "\n";
	out << "    setScanLength         : " << yesNo(okLength) << "\n";
	out << "    took                  : " << configMs << " ms\n";

	// Tag the frame so ImageManager routes it and waitForImagePreprocessed can match it.
	FrameInfo* pending = ProfilerManager::instance().getFrame(m_profilerID);
	if (pending) {
		pending->type = ct::s_height_map;
		pending->viewID = QStringLiteral("pst");
		pending->opticID = optic->id;
		pending->baseOpticID = QString();
	}

	quint32 trig0 = 0, trig1 = 0;
	qint32  enc0 = 0, enc1 = 0;
	const bool haveCounters = profiler->getCounters(trig0, enc0);

	say(QStringLiteral("Arming..."));
	step.restart();

	if (!ProfilerManager::instance().start(m_profilerID)) {
		out << "\n  ARM FAILED - start() returned false. Nothing was moved.\n";
		out << "  Driver error: " << ProfilerManager::instance().errorMsg(m_profilerID) << "\n";
		say(QStringLiteral("Arm failed. Report: %1").arg(reportPath));
		finish(false, QStringLiteral("ARM FAILED - the profiler did not start"));
		return;
	}
	const qint64 armMs = step.elapsed();

	say(QStringLiteral("Scanning %1 mm %2X at %3 mm/s...")
		.arg(distance_mm).arg(positiveDir ? "+" : "-").arg(speed3d));

	step.restart();
	jog(targetX, origin.wy, origin.wz, QStringLiteral("3D"));
	const qint64 moveMs = step.elapsed();

	step.restart();
	const bool acquired = ProfilerManager::instance().waitAcquisition(m_profilerID, PROFILER_TIMEOUT);
	const qint64 waitMs = step.elapsed();

	// Deliberately NOT stopRun() / unloadBoard() here, unlike scan(). A timeout during a
	// diagnostic must not leave m_stopRun set for the next run to trip over.
	if (!acquired) {
		out << "\n  ACQUISITION TIMED OUT after " << waitMs << " ms.\n";
		out << "  The batch did not fill. Nothing was force-stopped, so the next run starts clean.\n";
		out << "\n  Before blaming the sensor, check the arithmetic. The batch count is derived\n";
		out << "  from yPitchUm, so if triggers actually arrive further apart than yPitchUm says,\n";
		out << "  the batch needs MORE travel than the scan length asked for and can never fill.\n";
		out << "  Compare 'Implied pitch' below against the Y pitch in the profiler block.\n";
	}

	const dat::WorldCoordinate landed = SystemData::instance().currentCoordinate();
	if (haveCounters) profiler->getCounters(trig1, enc1);

	say(QStringLiteral("Waiting for the height map..."));
	step.restart();
	FrameInfo info = waitForImagePreprocessed(5000);
	const qint64 imageMs = step.elapsed();

	ProfilerManager::instance().stop(m_profilerID);

	//------------------------------------------------------------ results
	section(QStringLiteral("RESULTS"));

	const double travelled_mm = std::abs(landed.wx - origin.wx);

	out << "  Acquisition     : " << (acquired ? QStringLiteral("completed") : QStringLiteral("TIMED OUT")) << "\n";
	out << "  Landed at       : X=" << QString::number(landed.wx, 'f', 3) << "\n";
	out << "  Gantry travel   : " << QString::number(travelled_mm, 'f', 3) << " mm"
		<< "   (requested " << QString::number(distance_mm + overshoot_mm, 'f', 3) << " mm including overshoot)\n";

	if (std::abs(travelled_mm - (distance_mm + overshoot_mm)) > 0.5) {
		out << "\n  WARNING: the gantry did not travel what was asked. A soft limit may have\n";
		out << "  clamped the move. Every distance-derived number below is unreliable.\n";
	}

	out << "\n  Timing:\n";
	out << "    configure     : " << configMs << " ms\n";
	out << "    arm           : " << armMs << " ms\n";
	out << "    move          : " << moveMs << " ms\n";
	out << "    wait for batch: " << waitMs << " ms\n";
	out << "    wait for image: " << imageMs << " ms\n";

	// The RAW batch, straight from the driver's callback. The processed height map below is
	// stretched by the pitch constants and the divider, so its dimensions are NOT the profile
	// count and must never be used as one.
	int profiles = 0;
	int pointsPerProfile = 0;
	const bool haveBatchSize = profiler->getLastBatchSize(profiles, pointsPerProfile);

	if (haveBatchSize) {
		out << "\n  Batch received  : " << profiles << " profiles x " << pointsPerProfile
			<< " points   (raw, before any resize)\n";
		if (profiles == 0) {
			out << "  No profiles arrived at all - the controller never delivered a batch.\n";
		}
	}

	/*
	* The RAW batch, saved before ImageManager touches it. Deliberately done first and
	* independently of the processed image: the raw data is the only copy that is not
	* stretched by the pitch constants or rotated, so it is what any real measurement or
	* re-analysis should start from - and it is worth having even when the height map
	* downstream failed to build.
	*/
	//Taken unconditionally - the statistics below are worth having even when images are not
	//being written, and takeLastRawFrame() has to be called once either way to release the
	//driver's hold on the pool buffers.
	{
		mtrx::SharedMilID rawHeight, rawIntensity;

		if (profiler->takeLastRawFrame(rawHeight, rawIntensity) && rawHeight) {
			const MIL_ID mRawH = rawHeight->id();

			out << "\n  Raw height map  : " << mtrx::get_width(mRawH) << " x " << mtrx::get_height(mRawH)
				<< " px, 16-bit\n";

			const HeightStats hs = rawHeightStats(mRawH);
			const double zPitch = profiler->getZPitchUm();

			if (hs.ok && hs.valid > 0) {
				auto um = [&](int grey) { return (double(grey) - 32768.0) * zPitch; };

				out << "\n  Valid points    : " << hs.valid << " of " << hs.total
					<< "   (" << QString::number(100.0 * double(hs.valid) / double(hs.total), 'f', 1) << "%)\n";
				out << "                    0 means no measurement, so empty space around the part\n";
				out << "                    counts here too - judge this against how much of the\n";
				out << "                    scan the part actually occupies.\n";

				if (zPitch > 0.0) {
					out << "\n  Height spread (raw, 0 excluded; um relative to reference distance):\n";
					out << "    min           : " << QString::number(um(hs.lo), 'f', 1) << " um\n";
					out << "    1st pct       : " << QString::number(um(hs.p01), 'f', 1) << " um\n";
					out << "    median        : " << QString::number(um(hs.p50), 'f', 1) << " um\n";
					out << "    99th pct      : " << QString::number(um(hs.p99), 'f', 1) << " um\n";
					out << "    max           : " << QString::number(um(hs.hi), 'f', 1) << " um\n";
					out << "                    Percentiles, not min/max, are the honest summary: a\n";
					out << "                    handful of stray peaks moves the extremes a long way.\n";

					//The 8060 measures +/-7.3 mm about the reference distance. Anything beyond that
					//cannot be a real height, so it is either a stray peak or the head is mounted
					//at the wrong standoff - which is worth saying out loud, not leaving to be read
					//off a number.
					const double limit = 7300.0;
					if (std::abs(um(hs.p50)) > limit) {
						out << "\n  WARNING: the median height is outside the sensor's +/-7.3 mm range.\n";
						out << "  The head standoff is wrong - it wants 64 mm to the target.\n";
					}
					else if (std::abs(um(hs.p01)) > limit || std::abs(um(hs.p99)) > limit) {
						out << "\n  WARNING: more than 1% of measured points fall outside +/-7.3 mm.\n";
						out << "  Part height variation is exceeding the measuring window.\n";
					}
				}
			}
			else if (hs.ok) {
				out << "\n  Valid points    : NONE - every point in the batch is 0 (no measurement).\n";
				out << "                    The laser line never came back. Check standoff first,\n";
				out << "                    then exposure.\n";
			}

			if (saveImages) {
				MbufSaveA((dir + QStringLiteral("raw_heightmap.tiff")).toUtf8().constData(), mRawH);
				out << "\n  Saved           : raw_heightmap.tiff\n";
				out << "                    grey = height; recover microns with\n";
				out << "                    (grey - 32768) * " << QString::number(zPitch, 'f', 2) << ", and 0 means INVALID.\n";
			}

			if (rawIntensity && saveImages) {
				const MIL_ID mRawI = rawIntensity->id();
				MbufSaveA((dir + QStringLiteral("raw_intensity.tiff")).toUtf8().constData(), mRawI);
				out << "                    raw_intensity.tiff, " << mtrx::get_width(mRawI)
					<< " x " << mtrx::get_height(mRawI) << " px, 8-bit\n";
			}

			out << "                    One row per profile, one column per sample point, in\n";
			out << "                    acquisition order. No resize, no rotate, no divider\n";
			out << "                    stretch - so a pitch constant being wrong cannot\n";
			out << "                    corrupt this copy.\n";
		}
		else {
			out << "\n  Raw batch       : not available.\n";
			out << "                    The driver only holds raw buffers for a batch that\n";
			out << "                    COMPLETED. A batch abandoned on timeout is discarded\n";
			out << "                    before the buffers are ever allocated, so there is\n";
			out << "                    nothing to save - see the discarded count above.\n";
		}
	}

	if (info.viewID.isEmpty() || !info.pHeightMap) {
		out << "\n  NO IMAGE. waitForImagePreprocessed returned an empty frame.\n";
		out << "  If the acquisition completed, the batch arrived but the height map was never\n";
		out << "  built - check the ImageManager thread in the log.\n";
	}
	else {
		const MIL_ID mHeight = info.pHeightMap->id();
		const MIL_INT hw = mtrx::get_width(mHeight);
		const MIL_INT hh = mtrx::get_height(mHeight);

		out << "\n  Height map      : " << hw << " x " << hh << " px   (after resize and rotate)\n";

		/*
		* Deliberately NOT reporting min/max here. rotate_heightMap resizes with M_BICUBIC,
		* and interpolating between a real height and the 0 that means "no measurement" rings
		* past both ends of the real range: a batch whose true maximum was 32865 reported
		* 45572 through this path, which converts to +10.2 mm on a sensor with +/-7.3 mm of
		* range. Use the raw percentiles above; they cannot be distorted that way.
		*/
		if (mtrx::get_max(mHeight) <= 0.0) {
			out << "  WARNING: the height map is entirely zero. 0 means invalid on this sensor,\n";
			out << "  so the laser saw nothing - a different fault from receiving no triggers.\n";
		}

		if (info.pImage) {
			out << "  Intensity map   : " << mtrx::get_width(info.pImage->id())
				<< " x " << mtrx::get_height(info.pImage->id()) << " px   (after resize and rotate)\n";
		}

		if (saveImages) {
			MbufSaveA((dir + QStringLiteral("heightmap.tiff")).toUtf8().constData(), mHeight);
			mtrx::to_qimg(mHeight).save(dir + QStringLiteral("heightmap.png"));
			if (info.pImage) {
				MbufSaveA((dir + QStringLiteral("intensity.tiff")).toUtf8().constData(), info.pImage->id());
				mtrx::to_qimg(info.pImage->id()).save(dir + QStringLiteral("intensity.png"));
			}
			out << "  Images saved    : yes\n";
		}
	}

	//-------------------------------------------- the encoder question
	section(QStringLiteral("TRIGGER AND ENCODER"));

	if (!haveCounters) {
		out << "  The controller's counters could not be read on this backend.\n";
	}
	else {
		const qint64 trigDelta = qint64(trig1) - qint64(trig0);
		const qint64 encDelta = qint64(enc1) - qint64(enc0);
		/*
		* Rates are computed over the MOVE, not the whole counter window. The counters are
		* read just before start() and just after the batch wait, so their window includes
		* arming and any timeout spent standing still - and a 60 s timeout dilutes the rate
		* by a factor of five, which reads as a slow trigger rate rather than a stalled one.
		* Pulses only happen while the gantry moves, so the move is the honest denominator.
		*/
		const double windowSec = double(armMs + moveMs + waitMs) / 1000.0;
		const double moveSec = double(moveMs) / 1000.0;

		out << "  Trigger count   : " << trig0 << " -> " << trig1 << "   delta " << trigDelta << "\n";
		out << "  Encoder count   : " << enc0 << " -> " << enc1 << "   delta " << encDelta << "\n";
		out << "  Counter window  : " << QString::number(windowSec, 'f', 3) << " s   (arm + move + wait)\n";
		out << "  Move time       : " << QString::number(moveSec, 'f', 3) << " s\n";

		if (moveSec > 0.0) {
			const double trigRate = double(trigDelta) / moveSec;
			const double encRate = std::abs(double(encDelta)) / moveSec;

			out << "\n  During the move:\n";
			out << "    encoder pulses : " << QString::number(encRate, 'f', 1) << " /s   (what the gantry demanded)\n";
			out << "    triggers taken : " << QString::number(trigRate, 'f', 1) << " /s   (what the controller accepted)\n";

			if (trigRate > 0.0) {
				const double ratio = encRate / trigRate;
				const int div = std::max(1, optic->divider);

				out << "    ratio          : " << QString::number(ratio, 'f', 3)
					<< " encoder counts per trigger   (expected " << div << " at divider " << div << ")\n";

				/*
				* The expected ratio is the DIVIDER, not 1.0 - sub-sampling deliberately takes
				* one encoder count in N. Comparing against 1.0 made every correct run at
				* divider > 1 look like a fault.
				*/
				if (ratio > double(div) * 1.15) {
					out << "\n  The controller accepted FEWER triggers than the divider asks for.\n";
					out << "  That is saturation: the demanded rate is above the sampling frequency,\n";
					out << "  so the surplus is discarded and TRG_PASS fires (manual p.5-5). The pitch\n";
					out << "  you actually got is coarser than the one you set, and it moves with\n";
					out << "  speed - so it cannot be written into a constant.\n";
					out << "  Fix by raising the divider or lowering the speed: keep\n";
					out << "  speed_mm_s * 1000 / (yPitchUm * divider) under the sampling frequency.\n";
				}

				/*
				* Triggers outnumbering encoder counts is impossible for encoder-clocked
				* triggering measured against NET displacement - unless the axis reverses.
				* With 2-phase decoding a dither nets out in the counter while every accepted
				* edge still fires a trigger, so the profiles end up UNEVENLY spaced: locally
				* bunched where the gantry hesitated. That is worse than saturation, which at
				* least drops profiles evenly and leaves a correctable scale error.
				*
				* Observed on Codetrace-CK at 1 mm/s: 21462 triggers against 17968 counts, and
				* the resulting height map measured 2x wrong along the scan while looking
				* entirely plausible. It took a raw-buffer analysis to find, hence this check.
				*/
				if (encDelta != 0 && trigDelta > qint64(std::abs(double(encDelta)) * 1.02)) {
					const double excess = 100.0 * (double(trigDelta) / std::abs(double(encDelta)) - 1.0);
					out << "\n  *** WARNING: " << QString::number(excess, 'f', 1)
						<< "% MORE TRIGGERS THAN ENCODER COUNTS ***\n";
					out << "  Encoder-clocked triggers cannot outnumber net encoder counts unless the\n";
					out << "  axis changed direction. That means the gantry dithered - stick-slip at\n";
					out << "  low speed is the usual cause - and the extra profiles are NOT evenly\n";
					out << "  spaced, so this height map is locally stretched and squeezed.\n";
					out << "  It will look plausible and measure wrong. RAISE THE SCAN SPEED and\n";
					out << "  compare a feature's size between runs before trusting this scan.\n";
				}
			}
		}

		/*
		* Only trustworthy when the batch did NOT fill before the move ended. Once the batch
		* completes the counters stop advancing while the gantry keeps going, so dividing the
		* FULL travel by a delta that covers only part of it inflates the answer. That is what
		* produced 4.33 um/count on runs whose true pitch was 4.00 - the encoder delta had
		* landed on the batch size (17553 against a batch of 17500) while travel included the
		* 6 mm overshoot. Withhold it rather than print a number I know is high.
		*/
		if (encDelta != 0 && travelled_mm > 0.0 && !acquired) {
			out << "  Encoder scaling : " << QString::number(travelled_mm * 1000.0 / double(encDelta), 'f', 4)
				<< " um per encoder count   (MEASURED over the whole move)\n";
		}
		else if (encDelta != 0 && travelled_mm > 0.0) {
			out << "  Encoder scaling : not measurable this run - the batch filled before the move\n";
			out << "                    ended, so the counters stopped part way while the gantry\n";
			out << "                    carried on. Measure it from a run that times out: ask for a\n";
			out << "                    scan longer than the travel available, then this figure\n";
			out << "                    covers the whole move and is exact.\n";
		}
		else if (travelled_mm > 0.0) {
			out << "\n  The encoder counter did not move while the gantry travelled "
				<< QString::number(travelled_mm, 'f', 3) << " mm.\n";
			out << "  The controller is not seeing the encoder at all.\n";
		}

		if (trigDelta > 0 && travelled_mm > 0.0) {
			out << "  Implied pitch   : " << QString::number(travelled_mm * 1000.0 / double(trigDelta), 'f', 4)
				<< " um per trigger\n";
		}

		out << "\n  How to read this: run the same distance twice at two clearly different\n";
		out << "  speeds (Config page -> X 3D velocity -> Update Velocity X).\n";
		out << "    Same trigger delta both times      -> triggers follow DISTANCE. Encoder real.\n";
		out << "    Same elapsed time both times       -> triggers follow TIME. Sampling clock.\n";
		out << "    Trigger rate near the sampling\n";
		out << "    frequency regardless of speed      -> saturated: something is triggering\n";
		out << "                                          faster than the controller accepts.\n";
	}

	if (profiles > 0) {
		out << "\n  Profiles vs the requested scan length:\n";
		out << "    " << QString::number(distance_mm * 1000.0 / double(profiles), 'f', 4)
			<< " um per profile, if the batch really spanned the requested "
			<< QString::number(distance_mm, 'f', 3) << " mm.\n";
		out << "    Treat that as an upper bound, not a measurement: the batch fills whenever\n";
		out << "    enough triggers arrive, which is why the move overshoots the end point.\n";
		out << "    The encoder scaling above is the number to trust.\n";
	}

	//--------------------------------------------------------------- tidy
	if (returnToStart) {
		say(QStringLiteral("Returning to start..."));
		jog(origin.wx, origin.wy, origin.wz, QStringLiteral("2D"));
		out << "\n  Returned to the start position.\n";
	}
	else {
		out << "\n  Left parked at X=" << QString::number(landed.wx, 'f', 3)
			<< " so the travel can be measured by hand.\n";
	}

	const bool gotImage = !info.viewID.isEmpty() && info.pHeightMap;
	QString summary;
	if (acquired && gotImage)      summary = QStringLiteral("OK - batch acquired and height map built");
	else if (acquired && !gotImage) summary = QStringLiteral("PARTIAL - batch acquired but no height map was produced");
	else                            summary = QStringLiteral("FAILED - the acquisition timed out");

	say(QStringLiteral("%1. Report: %2").arg(summary).arg(reportPath));
	finish(true, summary);
}


void JobThread::preAcquisition()
{
	MachineController::instance().trackTime("Fiducial");
	m_postResults.clear();
	if (m_enableFiducial) searchFiducial();
	MachineController::instance().logTime("Fiducial");
	searchBarcode();
	ct::logger::info("preacq-3");
	colorCompensation(); ct::logger::info("preacq-4");
	warpageCompensation(); ct::logger::info("preacq-5");
}

void JobThread::postAcquisition()
{
	m_rgbOverrides.clear();
	savePostResult();
	save3DExtraOffset();
}

void JobThread::continuousSnap() {
	QString camID = "cam1";

	ScopedTimeLogger stl("Acquire Zstack");
	while (!m_stopZstack) {
		ScopedTimeLogger stl("Snap Z");
		CAMManager::instance().softTrigger(camID);
		CAMManager::instance().waitAcquisition(camID, 5000);
		if (m_snapDelay_ms > 0) os_tool::doNothing(m_snapDelay_ms);
	}
}

void JobThread::acquire2DImages()
{
	ScopedTimeLogger Stimer("[Acq] Total 2D Acquisition Time");
	MachineController::instance().trackTime("2D Acquisition");

	QString camID = "cam1";

	clearEmptyView();

	switchToFastModeLSC();

	ct::logger::info("acq2d");
	for (int i = 0; i < m_viewSequence->count(); i++) {
		if (m_stopRun) break;

		if (util::isRAMOver(SystemData::instance()._maxAllowRamUsage)) {
			ct::logger::warn("[Acq] Memory overload, waiting for more memory to proceed 2D acquisition...");

			while (util::isRAMOver(SystemData::instance()._maxAllowRamUsage)) {
				if (m_stopRun) break;
				os_tool::goSleep(1000);
			}

			ct::logger::warn("[Acq] Sufficient memory, proceed 2D acquisition.");
		}

		auto item = m_viewSequence->item(i);
		if (item->checkState() == Qt::Unchecked) continue;
		ct::logger::info("acq2d-1");
		auto id = item->whatsThis();

		if (!(*m_views).contains(id)) {
			ct::logger::error("[JobThread] Failed to acquire 2D. Invalid view ID: %s", id.toStdString().c_str());
			continue;
		}

		auto v = (*m_views)[id];

		if (v.type == ct::s_stitch_view) continue;

		CAMManager::instance().resetFrame(camID);

		if (v.zstack.generate_2D_stack || v.zstack.generate_3D_stack) {

			CAMManager::instance().frame(camID)->postTask.stackImage = true;

			if (v.zstack.acq_type == ct::s_preset) {
				ct::logger::info("Preset iteration: %d, %dum", v.zstack.preset_iteration, v.zstack.step_um);
				for (int i = 0; i < v.zstack.preset_iteration; i++) {
					ct::logger::info("Step: %.2fum", -((double)i * (double)v.zstack.step_um) / 1000);
					jogView(v, -((double)i* (double)v.zstack.step_um)/1000);
					snapView(v.id, false);
				}

				for (auto& optID : v.opticIDs) {
					auto cid = util::combineID(v.id, optID);
					emit stackImages(cid);
				}
			}
			else if (v.zstack.acq_type == ct::s_encoder) {

			}
			else if (v.zstack.acq_type == ct::s_time) {
				auto offset_mm = - (double)v.zstack.encoder_range_um / 1000.0;
				jogView(v);
				m_thread = std::thread(&JobThread::continuousSnap, this);
				jogView(v, offset_mm);
				m_stopZstack = true;
				m_thread.join();
				emit stackImages(id);
			}
		}
		else {
			jogView(v);
			snapView(v.id);
		}

		if (m_run1stFOVOnly) break;
	}

	m_compensateZMap.clear();

	switchToContinuousModeLSC();

	MachineController::instance().logTime("2D Acquisition");
	ct::logger::info("[Acq] Done acquiring 2D images");
}

bool JobThread::acquireBarcodeAndOcr()
{
	if (m_stopRun) return false;

	auto& sd = SystemData::instance();

	//Pitch mode: iterate the taught unit grid - base per unit = point 1 (top
	//left) + unit index * pitch. The line scan mid point is not used.
	if (sd._setupRegionPitchMode) {
		if (!sd._pitchP1Set) {
			ct::logger::error("[Acq] Pitch mode: point 1 not taught - cannot run barcode flow");
			return false;
		}

		const int unitsX = std::max(1, (int)sd._unitsX);
		const int unitsY = std::max(1, (int)sd._unitsY);
		ct::logger::info("[Acq] Pitch mode: %dx%d units, pitch %.3f / %.3f mm",
			unitsX, unitsY, sd._pitchX.load(), sd._pitchY.load());

		for (int iy = 0; iy < unitsY && !m_stopRun; iy++) {
			for (int ix = 0; ix < unitsX && !m_stopRun; ix++) {
				ct::logger::info("[Acq] Unit (%d, %d)", ix + 1, iy + 1);
				acquireBarcodeAndOcrAt(
					sd._pitchP1x + ix * sd._pitchX,
					sd._pitchP1y + iy * sd._pitchY,
					sd._pitchP1z,
					QString("unit_%1_%2").arg(ix + 1).arg(iy + 1));
			}
		}

		return !m_stopRun;
	}

	//Plane mode: single read at the middle of the 3D scan area
	if (!safeGuardLineScan()) return false;

	double sumX = 0, sumY = 0, z = 0;
	int n = 0;
	for (const auto& l : *m_linescans) {
		if (l.type == ct::s_stitch_linescan || l.id.isEmpty()) continue;
		sumX += (l.start_point.wx + l.end_point.wx) / 2.0;
		sumY += (l.start_point.wy + l.end_point.wy) / 2.0;
		z = l.start_point.wz;
		n++;
	}

	if (n == 0) {
		ct::logger::error("[Acq] No line scans assigned - cannot locate the barcode read position");
		return false;
	}

	return acquireBarcodeAndOcrAt(sumX / n, sumY / n, z);
}

bool JobThread::acquireBarcodeAndOcrAt(double baseX, double baseY, double baseZ, const QString& unitID)
{
	if (m_stopRun) return false;

	ScopedTimeLogger stl("[Acq] Barcode + OCR acquisition");
	auto& srx = SRXManager::instance();
	auto& sd = SystemData::instance();

	//jog the base camera position, then apply the taught camera-to-reader offset (XYZ)
	auto jogToReader = [&](int reader) {
		const bool taught = (reader == 1) ? (bool)sd._brR1Taught : (bool)sd._brR2Taught;
		const double dx = (reader == 1) ? sd._brR1dx.load() : sd._brR2dx.load();
		const double dy = (reader == 1) ? sd._brR1dy.load() : sd._brR2dy.load();
		const double dz = (reader == 1) ? sd._brR1dz.load() : sd._brR2dz.load();
		if (!taught) ct::logger::warn("[Acq] Reader %d offset not taught - scanning at the base point", reader);
		jog(baseX + (taught ? dx : 0), baseY + (taught ? dy : 0), baseZ + (taught ? dz : 0), "2D", true);
	};

	auto readerID = [](int reader) { return reader == 1 ? SRXManager::SRX1 : SRXManager::SRX2; };
	auto readerDuration = [&](int reader) { return (reader == 1) ? (int)sd._brR1Duration_ms : (int)sd._brR2Duration_ms; };

	//scan sequentially: first reader from recipe settings, fall back to the other
	const int first = (sd._brFirstReader == 2) ? 2 : 1;
	const int second = (first == 1) ? 2 : 1;

	const QString prevImg1 = srx.lastImagePath(SRXManager::SRX1);
	const QString prevImg2 = srx.lastImagePath(SRXManager::SRX2);

	int winner = 0;
	QString barcode;
	bool loserTriggered = false; //true when the non-barcode reader already ran a cycle

	for (int reader : { first, second }) {
		if (m_stopRun) return false;

		jogToReader(reader);
		if (m_stopRun) return false;

		const QString id = readerID(reader);
		const QDateTime t0 = QDateTime::currentDateTime();
		srx.trigger(id);

		const int durationMs = readerDuration(reader);
		QElapsedTimer timer;
		timer.start();
		bool decoded = false;

		while (timer.elapsed() < durationMs && !m_stopRun) {
			auto r = srx.lastResult(id);
			if (r.ok && r.timestamp >= t0) {
				barcode = r.code;
				decoded = true;
				break;
			}
			os_tool::goSleep(20);
		}

		if (decoded) {
			winner = reader;
			ct::logger::info("[Acq] Barcode read by reader %d: %s", reader, barcode.toStdString().c_str());
			break;
		}

		//close the cycle - the reader then reports and pushes its capture,
		//which doubles as the OCR image if the other reader finds the barcode
		ct::logger::warn("[Acq] Reader %d: no barcode within %dms", reader, durationMs);
		srx.stopReader(id);
		if (reader == first) loserTriggered = true;
	}

	if (winner == 0) {
		ct::logger::error("[Acq] No barcode found on either reader - saving as No_Barcode");
		emit barcodeDecoded("No_Barcode");
		emit unitBarcode(unitID, "No_Barcode");
		if (sd._setupRegionPitchMode) emit incrementProgress(); //barcode step
		if (InspectionThread::instance().isActive())
			InspectionThread::instance().reportSkipped(unitID, "OCR", "no barcode");
		else if (sd._setupRegionPitchMode)
			emit incrementProgress(); //keep the accounting when inspection is inactive
		return true; //run continues, no OCR image for this board
	}

	emit barcodeDecoded(barcode);
	emit unitBarcode(unitID, barcode);

	//the non-barcode side faces the label text: get its capture for OCR
	const int loser = (winner == 1) ? 2 : 1;
	const QString loserID = readerID(loser);
	const QString prevLoserImg = (loser == 1) ? prevImg1 : prevImg2;

	if (!loserTriggered) {
		//winner decoded on the first try - the other side has not run a cycle yet
		if (m_stopRun) return true;
		jogToReader(loser);
		if (m_stopRun) return true;

		srx.trigger(loserID);
		os_tool::doNothing(500); //let it capture
		srx.stopReader(loserID); //close the cycle so the image pushes
	}

	QImage ocrImg;
	QElapsedTimer imgTimer;
	imgTimer.start();
	constexpr int imageTimeoutMs = 5000;

	while (imgTimer.elapsed() < imageTimeoutMs && !m_stopRun) {
		if (srx.lastImagePath(loserID) != prevLoserImg) {
			ocrImg = srx.lastImage(loserID);
			break;
		}
		os_tool::goSleep(20);
	}

	bool ocrEnqueued = false;

	if (ocrImg.isNull()) {
		ct::logger::error("[Acq] No OCR image received from reader %d within %dms - OCR skipped for this board",
			loser, imageTimeoutMs);
	}
	else if (InspectionThread::instance().isActive()) {
		FrameInfo info;
		info.type = "srx_ocr";
		info.viewID = unitID; //ties the OCR result back to its unit
		info.opticID = loserID;
		info.cameraID = loserID;
		info.width = ocrImg.width();
		info.height = ocrImg.height();
		InspectionThread::instance().enqueue(info, ocrImg);
		ocrEnqueued = true;
	}
	else {
		ct::logger::info("[Acq] OCR image from reader %d captured (inspection inactive, image saved only)", loser);
	}

	if (sd._setupRegionPitchMode) emit incrementProgress(); //barcode step done

	if (!ocrEnqueued) {
		//no OCR result will come for this unit - close it out
		if (InspectionThread::instance().isActive())
			InspectionThread::instance().reportSkipped(unitID, "OCR", "no OCR image");
		else if (sd._setupRegionPitchMode)
			emit incrementProgress();
	}

	return true;
}

void JobThread::acquire3DImages()
{
	ScopedTimeLogger Stimer("[Acq] Total 3D Acquisition Time");
	MachineController::instance().trackTime("3D Acquisition");

	std::map<QString, QString> scanSequence;

	auto& ls = *m_linescans;

	const bool scanAlongY = SystemData::instance().isLineScanAxisY();

	clearEmptyLineScan();

	for (const auto& l : ls) {
		if (l.type == ct::s_stitch_linescan) continue;
		if (l.id == "") continue;

		//order lines by their position along the step axis
		scanSequence.insert({ QString("%1_%2").arg(scanAlongY ? l.px.cx : l.px.cy).arg(l.id), l.id});

		if (m_run1stFOVOnly) break;
	}

	//arrange so that always scan intensity true first
	std::deque<QString> opticsSeq;
	int smallestExposure = 999999999;
	QString smallestKey = "";

	for (auto o : *m_optics3D) {
		if (o.exposure < smallestExposure) {
			smallestExposure = o.exposure;
			smallestKey = o.id;
		}

		if (o.intensity) {
			opticsSeq.push_front(o.id);
		}
		else {
			opticsSeq.push_back(o.id);
		}
	}

	if (opticsSeq.size()) {
		//if none have intensity, assign smallest exposure as true
		if (!(*m_optics3D)[opticsSeq.front()].intensity) {
			(*m_optics3D)[smallestKey].intensity = true;
		}
	}

	for (const auto& seq : scanSequence) {

		auto l = ls[seq.second];

		auto mid = l.start_point;
		auto halfLength_mm = scanAlongY
			? abs(l.start_point.wy - l.end_point.wy) / 2
			: abs(l.start_point.wx - l.end_point.wx) / 2;
		mid.wx = (mid.wx + l.end_point.wx) / 2;
		mid.wy = (mid.wy + l.end_point.wy) / 2;

		//shift according to fiducial location (route to this line's nearest island transform)
		Fiducial* lineAlgo = m_fiducialAlgo;
		if (m_enableFiducial) {
			lineAlgo = fiducialForPoint(mid.wx, mid.wy);
			auto shifted = lineAlgo->getShiftedPoint(em::V2d(mid.wx, mid.wy));

			if (scanAlongY) {
				l.start_point.wx = shifted.x();
				l.end_point.wx = shifted.x();
				l.start_point.wy = shifted.y() - halfLength_mm;
				l.end_point.wy = shifted.y() + halfLength_mm;
			}
			else {
				l.start_point.wx = shifted.x() - halfLength_mm;
				l.end_point.wx = shifted.x() + halfLength_mm;
				l.start_point.wy = shifted.y();
				l.end_point.wy = shifted.y();
			}

			mid.wx = shifted.x();
			mid.wy = shifted.y();
		}

		bool firstRun = true;
		for (auto o : opticsSeq) {
			if (m_stopRun) break;

			if (util::isRAMOver(SystemData::instance()._maxAllowRamUsage)) {
				ct::logger::warn("[Acq] Memory overload, waiting for more memory to proceed 3D acquisition...");

				while (util::isRAMOver(SystemData::instance()._maxAllowRamUsage)) {
					if (m_stopRun) break;
					os_tool::goSleep(1000);
				}

				ct::logger::warn("[Acq] Sufficient memory, proceed 3D acquisition.");
			}

			ProfilerManager::instance().getFrame(m_profilerID)->stitchID = l.map_to_slinescan;
			ProfilerManager::instance().getFrame(m_profilerID)->postTask.rotationalAngle = -(lineAlgo->getAngle());

			//First run mainly to avoid collecting more than one intensity
			auto optic = (*m_optics3D)[o];
			if (!firstRun) {
				optic.intensity = false;
			}

			bool waitImage = true;
			if (SystemData::instance().getLaserType() == "SmartRay") {
				//SmartRay converts and pushes the frame inline inside waitAcquisition(),
				//so the image is already on the queue and there is nothing left to wait for.
				//Backends that push from an SDK callback (Gocator, SSZN, KeyenceLJ) still
				//need waitForImagePreprocessed, so they keep the default.
				waitImage = false;
			}

			scan(l.id, l.start_point, l.end_point, optic, waitImage);
			firstRun = false;
		}
	}

	MachineController::instance().logTime("3D Acquisition");
}

void JobThread::acquire3DImagesPitch()
{
	ScopedTimeLogger Stimer("[Acq] Total 3D Acquisition Time (pitch)");
	MachineController::instance().trackTime("3D Acquisition");

	auto& sd = SystemData::instance();
	if (!sd._pitchP1Set) {
		ct::logger::error("[Acq] Pitch mode: point 1 not taught - 3D scan skipped");
		return;
	}

	const bool scanAlongY = sd.isLineScanAxisY();
	const double halfLen = std::max(0.1, sd._pitchScanLen_mm.load()) / 2.0;
	const int unitsX = std::max(1, (int)sd._unitsX);
	const int unitsY = std::max(1, (int)sd._unitsY);

	//scan the intensity-carrying optic first, same rule as acquire3DImages
	std::deque<QString> opticsSeq;
	int smallestExposure = 999999999;
	QString smallestKey = "";

	for (auto o : *m_optics3D) {
		if (o.exposure < smallestExposure) {
			smallestExposure = o.exposure;
			smallestKey = o.id;
		}

		if (o.intensity) opticsSeq.push_front(o.id);
		else opticsSeq.push_back(o.id);
	}

	if (opticsSeq.size()) {
		if (!(*m_optics3D)[opticsSeq.front()].intensity) {
			(*m_optics3D)[smallestKey].intensity = true;
		}
	}

	for (int iy = 0; iy < unitsY && !m_stopRun; iy++) {
		for (int ix = 0; ix < unitsX && !m_stopRun; ix++) {

			const double baseX = sd._pitchP1x + ix * sd._pitchX;
			const double baseY = sd._pitchP1y + iy * sd._pitchY;

			dat::WorldCoordinate start, end;
			start.wx = end.wx = baseX;
			start.wy = end.wy = baseY;
			start.wz = end.wz = sd._pitchP1z;

			//scan of the recipe length, centered on the unit
			if (scanAlongY) {
				start.wy = baseY - halfLen;
				end.wy = baseY + halfLen;
			}
			else {
				start.wx = baseX - halfLen;
				end.wx = baseX + halfLen;
			}

			const QString unitID = QString("unit_%1_%2").arg(ix + 1).arg(iy + 1);
			ct::logger::info("[Acq] 3D scan %s: %.3f..%.3f", unitID.toStdString().c_str(),
				scanAlongY ? start.wy : start.wx, scanAlongY ? end.wy : end.wx);

			bool firstRun = true;
			for (auto o : opticsSeq) {
				if (m_stopRun) break;

				ProfilerManager::instance().getFrame(m_profilerID)->stitchID = "";
				ProfilerManager::instance().getFrame(m_profilerID)->postTask.rotationalAngle = 0.0;

				auto optic = (*m_optics3D)[o];
				if (!firstRun) optic.intensity = false;

				bool waitImage = (SystemData::instance().getLaserType() != "SmartRay");
				scan(unitID, start, end, optic, waitImage);
				firstRun = false;
			}

			if (!m_stopRun) emit incrementProgress(); //3D scan step done for this unit
		}
	}

	MachineController::instance().logTime("3D Acquisition");
}

void JobThread::collectPlane()
{
	auto& mainOptics = getMainOptics();

	for (const auto& v : m_viewPlane->views) {

		if (m_stopRun) return;

		auto root = Common::Directory::getRecipeImagesPath() + "PlaneImages";
		auto path = path::getViewPath(root.toStdString(), v);
		path.extension = "jpg";
		auto finalPath = path.GetPath();

		//Note: Not using jog to view, plane image should be collected using fiducial only. Since the warpage compensation only applies to assigned views
		// plane view do not count
		jogBasedOnFiducial(v.world.wx, v.world.wy, v.world.wz, "2D", true);

		CAMManager::instance().resetFrame(m_camID);
		snapOptic(mainOptics, v.id, "", false);
		auto info = waitForImagePreprocessed();
		auto mBuf = info.pImage->id();
		if (mBuf == M_NULL) {
			ct::logger::error("[Collect Plane] Invalid buffer obtained: %s", v.id.toStdString().c_str());
			continue;
		}
		MbufExportA(finalPath.c_str(), M_JPEG_LOSSY, mBuf);
	}

	if (!m_stopRun) emit planeCollectionDone();
}

void JobThread::test()
{		
	//TEST: Toggle between fast and continuous mode
	switchToFastModeLSC();
	switchToContinuousModeLSC();

	//TEST: VIE switch light speed

	/*CAMManager::instance().setDO(m_camID, m_camResetIO, true);
	for (auto v : (*m_views)) {
		snapView(v.id);
		break;
	}
	CAMManager::instance().setDO(m_camID, m_camResetIO, false);*/
	
	//TEST: Test snapshot transition to linescan issue
	////do snapshot on first view only
	//ProfilerManager::instance().stop(m_profilerID);

	//for (int i = 0; i < m_viewSequence->count(); i++) {
	//	auto item = m_viewSequence->item(i);
	//	auto id = item->whatsThis();

	//	if (!(*m_views).contains(id)) {
	//		ct::logger::error("[JobThread] Failed to test. Invalid view ID: %s", id.toStdString().c_str());
	//		continue;
	//	}

	//	auto v = (*m_views)[id];

	//	jogLaserBasedOnFiducial(v.world.wx, v.world.wy, v.world.wz, "2D");

	//	ProfilerManager::instance().snapShot(m_profilerID);
	//	ProfilerManager::instance().waitAcquisition(m_profilerID, PROFILER_TIMEOUT);

	//	auto& profiles = ProfilerManager::instance().getFrame(m_profilerID)->profiles;
	//	auto average = std::accumulate(profiles.begin(), profiles.end(), 0.0) / profiles.size();
	//	if (std::isnan(average))
	//	{
	//		ct::logger::warn("[Warpage Compenastion] Average value is NAN");
	//	}
	//	else
	//	{
	//		auto offset = -average;
	//	}

	//	break;
	//}

	////scan first linescan only
	//std::map<QString, QString> scanSequence;

	//auto& ls = *m_linescans;

	//clearEmptyLineScan();

	//for (const auto& l : ls) {
	//	if (l.type == ct::s_stitch_linescan) continue;
	//	if (l.id == "") continue;

	//	scanSequence.insert({ QString("%1_%2").arg(l.px.cy).arg(l.id), l.id });

	//	if (m_run1stFOVOnly) break;
	//}

	//for (const auto& seq : scanSequence) {

	//	auto l = ls[seq.second];

	//	auto mid = l.start_point;
	//	auto halfWidth_mm = abs(l.start_point.wx - l.end_point.wx) / 2;
	//	mid.wx = (mid.wx + l.end_point.wx) / 2;
	//	mid.wy = (mid.wy + l.end_point.wy) / 2;

	//	//shift according to fiducial location
	//	if (m_enableFiducial) {
	//		auto shifted = m_fiducialAlgo->getShiftedPoint(em::V2d(mid.wx, mid.wy));

	//		l.start_point.wx = shifted.x() - halfWidth_mm;
	//		l.end_point.wx = shifted.x() + halfWidth_mm;
	//		l.start_point.wy = shifted.y();
	//		l.end_point.wy = shifted.y();

	//		mid.wx = shifted.x();
	//		mid.wy = shifted.y();
	//	}

	//	for (auto& o : *m_optics3D) {
	//		if (m_stopRun) break;

	//		if (util::isRAMOver(SystemData::instance()._maxAllowRamUsage)) {
	//			ct::logger::warn("[Acq] Memory overload, waiting for more memory to proceed 3D acquisition...");

	//			while (util::isRAMOver(SystemData::instance()._maxAllowRamUsage)) {
	//				if (m_stopRun) break;
	//				os_tool::goSleep(1000);
	//			}

	//			ct::logger::warn("[Acq] Sufficient memory, proceed 3D acquisition.");
	//		}

	//		ProfilerManager::instance().getFrame(m_profilerID)->stitchID = l.map_to_slinescan;
	//		ProfilerManager::instance().getFrame(m_profilerID)->postTask.rotationalAngle = -(m_fiducialAlgo->getAngle());
	//		scan(l.id, l.start_point, l.end_point, o);
	//	}

	//	break;
	//}
}

void JobThread::stopRun()
{
	m_stopRun = true;
}

void JobThread::getAllIntensityFromExpectedGV(QString camID, QString opticType, int idealR, int idealG, int idealB, QRectF roi)
{
	QString info;

	auto idealGV_R = idealR;
	auto idealGV_G = idealG;
	auto idealGV_B = idealB;

	int intensity_R = 0, intensity_G = 0, intensity_B = 0;

	auto cam = CAMManager::instance().camera(camID);

	if (opticType.contains("RGB")) {
		for (const auto& key : LSCManager::instance().channels()) {
			auto ch = LSCManager::instance().channels()[key];

			if (opticType == ch.optic) {
				if (ch.lighting_type == "R") intensity_R = getIntensityFromIdealGV(camID, ch.id, idealGV_R);
				else if (ch.lighting_type == "G") intensity_G = getIntensityFromIdealGV(camID, ch.id, idealGV_G);
				else if (ch.lighting_type == "B") intensity_B = getIntensityFromIdealGV(camID, ch.id, idealGV_B);
			}
		}
	}
	else {
		QVector<QString> channels;

		for (const auto& key : LSCManager::instance().channels()) {
			auto ch = LSCManager::instance().channels()[key];

			if (opticType == ch.optic) {
				channels.push_back(ch.id);
			}
		}

		intensity_R = getIntensityFromIdealGV(camID, channels, idealGV_R);
		intensity_G = getIntensityFromIdealGV(camID, channels, idealGV_G);
		intensity_B = getIntensityFromIdealGV(camID, channels, idealGV_B);
	}

	emit obtainedIdealIntensity(intensity_R, intensity_G, intensity_B);
}

int JobThread::getIntensityFromIdealGV(QString camID, QString channel, double idealGV)
{
	if (!LSCManager::instance().isChannelValid(channel)) return 0;

	OpticsControl::instance().toggleAllChannels(false);
	OpticsControl::instance().toggleChannel(channel, true);
	
	int bestIntensity = 0;
	double lowestDiff = 255.0;
	double closestGV = 0.0;
	double prevDif = 255.0;

	OpticsControl::instance().setBrightness(camID, channel);

	CAMManager::instance().resetFrame(camID);
	CAMManager::instance().frame(camID)->type = ct::s_mono; //currently only support mono camera

	for (int i = 1; i < 255; i++) {
		if (m_stopRun) break;

		OpticsControl::instance().setIntensity(channel, i);
		CAMManager::instance().frame(camID)->postTask.combineRGB = false;
		triggerCamera(camID);

		auto info = waitForImagePreprocessed();

		MIL_ID mBuf = info.pImage->id();
		MIL_ID mMono = mtrx::to_mono(mBuf);
		mtrx::BufferCollector bc_mMono(mMono);

		MIL_UINT8* hostPtr = M_NULL;
		MIL_ID pitch = M_NULL;
		MbufInquire(mMono, M_HOST_ADDRESS, &hostPtr);
		MbufInquire(mMono, M_PITCH, &pitch);

		auto w = mtrx::get_width(mMono);
		auto h = mtrx::get_height(mMono);

		int start_x = m_roi.x();
		int start_y = m_roi.y();
		int end_x = m_roi.width() + start_x;
		int end_y = m_roi.height() + start_y;

		double avg = 0.0;
		double count = 0.0;

		for (int x = start_x; x < end_x; x++) {
			for (int y = start_y; y < end_y; y++) {

				avg += hostPtr[x + (y * pitch)];
				count++;
			}
		}

		avg /= count;

		auto difGV = abs(idealGV - avg);

		if (lowestDiff > difGV) {
			lowestDiff = difGV;
			closestGV = avg;
			bestIntensity = i;
		}

		if (prevDif + 1 < difGV) break;
		prevDif = difGV;
	}

	ct::logger::info("=============================> Channel: %s", channel.toStdString().c_str());
	ct::logger::info("Best Intensity: %d", bestIntensity);
	ct::logger::info("Closest Gray Value: %f", closestGV);
	ct::logger::info("Lowest Difference from Ideal Gray Value: %f", lowestDiff);

	OpticsControl::instance().toggleChannel(channel, false);

	return bestIntensity;
}

int JobThread::getIntensityFromIdealGV(QString camID, const QVector<QString>& channels, double idealGV) {

	for (auto channel : channels) {
		if (!LSCManager::instance().isChannelValid(channel)) return 0;
	}

	OpticsControl::instance().toggleAllChannels(false);

	for (auto channel : channels) {
		OpticsControl::instance().setBrightness(camID, channel);
		OpticsControl::instance().toggleChannel(channel, true);
	}

	int bestIntensity = 0;
	double lowestDiff = 255.0;
	double closestGV = 0.0;
	double prevDif = 255.0;

	CAMManager::instance().resetFrame(camID);
	CAMManager::instance().frame(camID)->type = ct::s_mono; //currently only support mono camera

	for (int i = 1; i < 255; i++) {

		for (auto channel : channels) {
			OpticsControl::instance().setIntensity(channel, i);
		}
		CAMManager::instance().frame(camID)->postTask.combineRGB = false;
		triggerCamera(camID);

		auto info = waitForImagePreprocessed();

		MIL_ID mBuf = info.pImage->id();
		MIL_ID mMono = mtrx::to_mono(mBuf);
		mtrx::BufferCollector bc_mMono(mMono);

		MIL_UINT8* hostPtr = M_NULL;
		MIL_ID pitch = M_NULL;
		MbufInquire(mMono, M_HOST_ADDRESS, &hostPtr);
		MbufInquire(mMono, M_PITCH, &pitch);

		auto w = mtrx::get_width(mMono);
		auto h = mtrx::get_height(mMono);

		int start_x = m_roi.x();
		int start_y = m_roi.y();
		int end_x = m_roi.width() + start_x;
		int end_y = m_roi.height() + start_y;

		double avg = 0.0;
		double count = 0.0;

		for (int x = start_x; x < end_x; x++) {
			for (int y = start_y; y < end_y; y++) {

				avg += hostPtr[x + (y * pitch)];
				count++;
			}
		}

		avg /= count;

		auto difGV = abs(idealGV - avg);

		if (lowestDiff > difGV) {
			lowestDiff = difGV;
			closestGV = avg;
			bestIntensity = i;
		}

		if (prevDif + 1 < difGV) break;
		prevDif = difGV;
	}

	ct::logger::info("Best Intensity: %d", bestIntensity);
	ct::logger::info("Closest Gray Value: %f", closestGV);
	ct::logger::info("Lowest Difference from Ideal Gray Value: %f", lowestDiff);

	for (auto channel : channels) {
		OpticsControl::instance().toggleChannel(channel, false);
	}

	return bestIntensity;
}

void JobThread::calibrateOptimumBrightness(QString camID) {
	int camIndex = 0;

	int minExposure = 1000;
	int maxExposure = 25000;

	int minGain = 1;
	int maxGain = 15;

	int targetedDif = 1;
	double tolerance = 0.1;
	double distancePerStepInIntensity = 0.0;

	int midBrightness = 128;

	auto w = CAMManager::instance().getWidth(camID);
	auto h = CAMManager::instance().getHeight(camID);
	CAMManager::instance().resetFrame(camID);

	int percentage = 70;

	int newW = w * percentage / 100;
	int newH = h * percentage / 100;

	int startX = (w - newW) / 2;
	int startY = (h - newH) / 2;

	OpticsControl::instance().toggleAllChannels(false);

	auto& brightness = m_portabilityInfo->lightingCalibrationInfo.brightness;
	brightness.clear();
	
	auto groupedKeys = OpticsControl::instance().getGroupedOptics();

	emit startProgressBar("Obtaining Optimum Brightness", groupedKeys.size(), true);

	for (const auto& groupedKey : groupedKeys.keys()) {

		bool stop = false;
		//find the right exposure and gain

		OpticsControl::instance().toggleGroupedOptic(groupedKey, groupedKeys, true);
		OpticsControl::instance().setGroupedOpticIntensity(groupedKey, groupedKeys, midBrightness);

		BrightnessInfo b;

		maxExposure = 25000;
		double prevDif = 255;

		for (int gain = minGain; gain <= maxGain; gain++) {

			if (m_stopRun) break;

			CAMManager::instance().setGain(camID, gain);

			double prevAvg = 0;

			int exposureStep = 1000;
			bool stepIsOptimum = false;

			if (gain == 5) {
				maxExposure = 50000;
			}

			for (int exposure = minExposure; exposure <= maxExposure; exposure += exposureStep) {

				if (m_stopRun) break;

				// Ensure setting does not exceed the maximum allowed value
				if (exposure > maxExposure) {
					exposure = maxExposure;
				}

				CAMManager::instance().setExposure(camID, exposure);
				CAMManager::instance().frame(camID)->postTask.combineRGB = false;

				triggerCamera(camID);

				auto info = waitForImagePreprocessed();

				MIL_ID mBuf = info.pImage->id();

				auto avg = mtrx::get_mean(mBuf, startX, startY, newW, newH);
				ct::logger::debug("[Auto brightness] Intensity: %f", avg);

				//if (avg > 252 && abs(prevAvg-avg) < 0.01) {
				if (util::is_equal(avg, midBrightness, 0.55)) {
					ct::logger::debug("[Auto brightness] Channel %s: Exposure %d, Gain %d, Intensity Gap: %f", groupedKey.toStdString().c_str(), exposure, gain, abs(avg - prevAvg));
					b.gain = gain;
					b.exposure = exposure;
					stop = true;
					break;
				}

				double dif = avg - prevAvg;

				if (dif < prevDif) {
					b.gain = gain;
					b.exposure = exposure;
					prevDif = dif;
				}

				// Adjust step size based on difference
				if (std::abs(dif - targetedDif) > tolerance && !stepIsOptimum) {
					// Reduce step size to get closer to the target feedback increase
					exposureStep *= (targetedDif / std::abs(dif));
					ct::logger::debug("Exposure step: %d", exposureStep);
				}
				else {

					if (!stepIsOptimum) {
						distancePerStepInIntensity = dif;
						b.average_gap = distancePerStepInIntensity;
					}

					stepIsOptimum = true;

					//calculate the required number of step to get to the optimum exposure
					//T: target, C: current value, D: distance per step (intensity), S: Optimum step to get close to 1 intensity per step increase, E: Current exposure
					//Closest exposure to get targeted value: (T-C)/D*S+E
					exposure = (midBrightness - avg) / distancePerStepInIntensity * exposureStep + exposure;
					ct::logger::debug("Optimum exposure step: %d", exposureStep);
					ct::logger::debug("Optimum exposure: %d", exposure);
					exposure = exposure - exposureStep; //mainly to remove the sincrement that will happen when looping

					if (exposure < minExposure) break;
				}

				prevAvg = avg;
			}

			if (stop) break;
		}

		brightness.insert(groupedKey, b);

		OpticsControl::instance().toggleGroupedOptic(groupedKey, groupedKeys, false);

		emit incrementProgress();
	}

	if (m_stopRun) {
		emit loadPortabilityInfo();
	}
	else {
		emit savePortabilityInfo();
	}

	emit stopProgressBar();
}

void JobThread::getGVTable(QString camID, GVTable& gvt, QRectF roi) {

	OpticsControl::instance().enableOffset(false);
	OpticsControl::instance().toggleAllChannels(false);

	gvt.clear();

	auto groupedKeys = OpticsControl::instance().getGroupedOptics();
	CAMManager::instance().resetFrame(camID);
	emit startProgressBar("Learning Profile", groupedKeys.size(), true);

	qDebug() << "All Grouped Keys:" << groupedKeys;
	qDebug() << "Grouped Keys Size:" << groupedKeys.size();

	for (const auto& groupedKey : groupedKeys.keys()) {

		if (m_stopRun) break;

		OpticsControl::instance().setBrightness(camID, groupedKey);
		OpticsControl::instance().toggleGroupedOptic(groupedKey, groupedKeys, true);
		auto& profile = gvt[groupedKey];
		std::fill(profile.begin(), profile.end(), 0);

		for (int i = 0; i < static_cast<int>(profile.size()); i++) {

			if (m_stopRun) break;

			OpticsControl::instance().setGroupedOpticIntensity(groupedKey, groupedKeys, i);
			CAMManager::instance().frame(camID)->postTask.combineRGB = false;
			CAMManager::instance().frame(camID)->opticID = "optic123";
			CAMManager::instance().frame(camID)->type = ct::s_mono;
			qDebug() << "Before trigger camera";
			triggerCamera(camID);

			auto infos = waitForImageReady();
			if (infos.isEmpty()) {
				ct::logger::error("[getGVTable] Image capture timed out or returned no frames.");
				m_stopRun = true;
				break;
			}

			auto info = infos[0];
			if (!info.pImage) {
				ct::logger::error("[getGVTable] Image capture returned no image.");
				m_stopRun = true;
				break;
			}

			MIL_ID mBuf = info.pImage->id();
			if (mBuf == M_NULL) {
				ct::logger::error("[getGVTable] Image buffer is null.");
				m_stopRun = true;
				break;
			}

			auto w = mtrx::get_width(mBuf);
			auto h = mtrx::get_height(mBuf);
			if (w <= 0 || h <= 0) {
				ct::logger::error("[getGVTable] Invalid image buffer.");
				m_stopRun = true;
				break;
			}

			int start_x = std::max(0, static_cast<int>(std::floor(roi.x())));
			int start_y = std::max(0, static_cast<int>(std::floor(roi.y())));
			int end_x = std::min(static_cast<int>(w), static_cast<int>(std::ceil(roi.x() + roi.width())));
			int end_y = std::min(static_cast<int>(h), static_cast<int>(std::ceil(roi.y() + roi.height())));

			if (start_x >= end_x || start_y >= end_y) {
				ct::logger::error("[getGVTable] Invalid ROI: %.2f, %.2f, %.2f, %.2f",
					roi.x(), roi.y(), roi.width(), roi.height());
				m_stopRun = true;
				break;
			}

			double avg = mtrx::get_mean(mBuf, start_x, start_y, end_x - start_x, end_y - start_y);

			if (avg >= 255) break;

			profile[i] = avg;
			ct::logger::info("Calibrated intensity for %s(%d): %.2f", groupedKey.toStdString().c_str(), i, avg);
		}

		OpticsControl::instance().toggleGroupedOptic(groupedKey, groupedKeys, false);

		emit incrementProgress();
	}

	OpticsControl::instance().enableOffset(true);

	if (m_stopRun) {
		loadPortabilityInfo();
	}
	else {
		savePortabilityInfo();
	}

	emit stopProgressBar();
}

void JobThread::calibrateMaxCurrent(QString camID, QRectF roi, double plateauDiffThreshold, double maxCurrentAmp) {

	if (!std::isfinite(maxCurrentAmp) || maxCurrentAmp <= 0.0) {
		ct::logger::error("[Alt8][MaxCurrent] Invalid max current. Cam: %s, MaxCurrent: %.6f A",
			camID.toStdString().c_str(), maxCurrentAmp);
		return;
	}

	ct::logger::info("[Alt8][MaxCurrent] Start. Cam: %s, ROI: %.2f, %.2f, %.2f, %.2f, Threshold: %.6f, MaxCurrent: %.4f A",
		camID.toStdString().c_str(), roi.x(), roi.y(), roi.width(), roi.height(), plateauDiffThreshold, maxCurrentAmp);
	m_stopRun = false;
	OpticsControl::instance().enableOffset(false);
	OpticsControl::instance().toggleAllChannels(false);

	auto groupedKeys = OpticsControl::instance().getGroupedOptics();
	ct::logger::info("[Alt8][MaxCurrent] Grouped optic count: %d", groupedKeys.size());
	CAMManager::instance().resetFrame(camID);
	ct::logger::info("[Alt8][MaxCurrent] Camera frame reset done. Cam: %s", camID.toStdString().c_str());

	// 1. Set all grouped optics to the requested max current to begin testing
	for (const auto& groupedKey : groupedKeys.keys()) {
		ct::logger::info("[Alt8][MaxCurrent] Init group max current to %.4f A. Group: %s", maxCurrentAmp, groupedKey.toStdString().c_str());
		const bool initMaxCurrentOk = OpticsControl::instance().setGroupedOpticMaxCurrent(groupedKey, groupedKeys, maxCurrentAmp);
		ct::logger::info("[Alt8][MaxCurrent] Init max current result. Group: %s, Result: %s",
			groupedKey.toStdString().c_str(), initMaxCurrentOk ? "PASS" : "FAIL");
	}

	emit startProgressBar("Calibrating Max Current Limits", groupedKeys.size(), true);
	ct::logger::info("[Alt8][MaxCurrent] Progress bar started.");

	QHash<QString, double> proposedLimits;
	int groupIndex = 0;

	for (const auto& groupedKey : groupedKeys.keys()) {
		if (m_stopRun) {
			ct::logger::warn("[Alt8][MaxCurrent] Stop requested before group start. Next group: %s", groupedKey.toStdString().c_str());
			break;
		}

		ct::logger::info("[Alt8][MaxCurrent] Begin group %d/%d: %s",
			groupIndex + 1, groupedKeys.size(), groupedKey.toStdString().c_str());
		const bool setBrightnessOk = OpticsControl::instance().setBrightness(camID, groupedKey);
		ct::logger::info("[Alt8][MaxCurrent] setBrightness result. Group: %s, Result: %s",
			groupedKey.toStdString().c_str(), setBrightnessOk ? "PASS" : "FAIL");
		const bool toggleOnOk = OpticsControl::instance().toggleGroupedOptic(groupedKey, groupedKeys, true);
		ct::logger::info("[Alt8][MaxCurrent] Toggle group ON result. Group: %s, Result: %s",
			groupedKey.toStdString().c_str(), toggleOnOk ? "PASS" : "FAIL");

		std::vector<double> gvHistory;
		int plateauIntensityIndex = -1;

		for (int i = 0; i <= 255; i++) {
			if (m_stopRun) {
				ct::logger::warn("[Alt8][MaxCurrent] Stop requested during group. Group: %s, Intensity: %d",
					groupedKey.toStdString().c_str(), i);
				break;
			}

			ct::logger::info("[Alt8][MaxCurrent] Set intensity. Group: %s, Intensity: %d",
				groupedKey.toStdString().c_str(), i);
			const bool setIntensityOk = OpticsControl::instance().setGroupedOpticIntensity(groupedKey, groupedKeys, i);
			ct::logger::info("[Alt8][MaxCurrent] Set intensity result. Group: %s, Intensity: %d, Result: %s",
				groupedKey.toStdString().c_str(), i, setIntensityOk ? "PASS" : "FAIL");
			ct::logger::info("[Alt8][MaxCurrent] Prepare camera frame. Cam: %s, Group: %s, Intensity: %d",
				camID.toStdString().c_str(), groupedKey.toStdString().c_str(), i);
			CAMManager::instance().frame(camID)->postTask.combineRGB = false;
			CAMManager::instance().frame(camID)->opticID = "optic123";
			CAMManager::instance().frame(camID)->type = ct::s_mono;

			ct::logger::info("[Alt8][MaxCurrent] Trigger camera. Cam: %s, Group: %s, Intensity: %d",
				camID.toStdString().c_str(), groupedKey.toStdString().c_str(), i);
			triggerCamera(camID);
			ct::logger::info("[Alt8][MaxCurrent] Wait for preprocessed image. Cam: %s, Group: %s, Intensity: %d",
				camID.toStdString().c_str(), groupedKey.toStdString().c_str(), i);
			auto info = waitForImagePreprocessed();
			ct::logger::info("[Alt8][MaxCurrent] Image wait returned. Cam: %s, Group: %s, Intensity: %d, HasImage: %s",
				camID.toStdString().c_str(), groupedKey.toStdString().c_str(), i, info.pImage ? "YES" : "NO");

			if (!info.pImage) {
				ct::logger::error("[calibrateMaxCurrent] Image preprocessing returned no image. Group: %s, Intensity: %d",
					groupedKey.toStdString().c_str(), i);
				m_stopRun = true;
				break;
			}

			MIL_ID mBuf = info.pImage->id();
			if (mBuf == M_NULL) {
				ct::logger::error("[calibrateMaxCurrent] Image buffer is null. Group: %s, Intensity: %d",
					groupedKey.toStdString().c_str(), i);
				m_stopRun = true;
				break;
			}

			ct::logger::info("[Alt8][MaxCurrent] Convert image to mono. Group: %s, Intensity: %d",
				groupedKey.toStdString().c_str(), i);
			MIL_ID mMono = mtrx::to_mono(mBuf);
			if (mMono == M_NULL) {
				ct::logger::error("[calibrateMaxCurrent] Failed to convert image to mono. Group: %s, Intensity: %d",
					groupedKey.toStdString().c_str(), i);
				m_stopRun = true;
				break;
			}
			mtrx::BufferCollector bc_mMono(mMono);

			auto w = mtrx::get_width(mMono);
			auto h = mtrx::get_height(mMono);
			ct::logger::info("[Alt8][MaxCurrent] Mono image size. Group: %s, Intensity: %d, W: %d, H: %d",
				groupedKey.toStdString().c_str(), i, static_cast<int>(w), static_cast<int>(h));

			if (w <= 0 || h <= 0) {
				ct::logger::error("[calibrateMaxCurrent] Invalid image buffer. Group: %s, Intensity: %d, W: %d, H: %d",
					groupedKey.toStdString().c_str(), i, static_cast<int>(w), static_cast<int>(h));
				m_stopRun = true;
				break;
			}

			int start_x = std::max(0, static_cast<int>(std::floor(roi.x())));
			int start_y = std::max(0, static_cast<int>(std::floor(roi.y())));
			int end_x = std::min(static_cast<int>(w), static_cast<int>(std::ceil(roi.x() + roi.width())));
			int end_y = std::min(static_cast<int>(h), static_cast<int>(std::ceil(roi.y() + roi.height())));
			ct::logger::info("[Alt8][MaxCurrent] Clipped ROI. Group: %s, Intensity: %d, X: %d, Y: %d, W: %d, H: %d",
				groupedKey.toStdString().c_str(), i, start_x, start_y, end_x - start_x, end_y - start_y);

			if (start_x >= end_x || start_y >= end_y) {
				ct::logger::error("[calibrateMaxCurrent] Invalid ROI: %.2f, %.2f, %.2f, %.2f. Group: %s, Intensity: %d",
					roi.x(), roi.y(), roi.width(), roi.height(), groupedKey.toStdString().c_str(), i);
				m_stopRun = true;
				break;
			}

			ct::logger::info("[Alt8][MaxCurrent] Calculate average GV. Group: %s, Intensity: %d",
				groupedKey.toStdString().c_str(), i);
			double avg = mtrx::get_mean(mMono, start_x, start_y, end_x - start_x, end_y - start_y);
			if (!std::isfinite(avg)) {
				ct::logger::error("[calibrateMaxCurrent] Invalid average GV. Group: %s, Intensity: %d",
					groupedKey.toStdString().c_str(), i);
				m_stopRun = true;
				break;
			}
			gvHistory.push_back(avg);
			ct::logger::info("[Alt8][MaxCurrent] Average GV. Group: %s, Intensity: %d, GV: %.6f, History: %d",
				groupedKey.toStdString().c_str(), i, avg, static_cast<int>(gvHistory.size()));

			if (gvHistory.size() >= 10) {
				auto start_it = gvHistory.end() - 10;
				auto end_it = gvHistory.end();

				auto minmax = std::minmax_element(start_it, end_it);
				double diff = *(minmax.second) - *(minmax.first);
				ct::logger::info("[Alt8][MaxCurrent] Plateau check. Group: %s, Intensity: %d, Last10Diff: %.6f, Threshold: %.6f",
					groupedKey.toStdString().c_str(), i, diff, plateauDiffThreshold);

				if (diff < plateauDiffThreshold) {
					plateauIntensityIndex = i - 9;
					ct::logger::info("[Alt8][MaxCurrent] Plateau detected. Group: %s, CurrentIntensity: %d, PlateauIntensity: %d",
						groupedKey.toStdString().c_str(), i, plateauIntensityIndex);
					break;
				}
			}
		}

		// Save the calculated limit into our temporary QHash
		if (plateauIntensityIndex != -1) {
			double calculatedCurrent = ((double)(plateauIntensityIndex) / 255.0) * maxCurrentAmp;
			proposedLimits.insert(groupedKey, calculatedCurrent);
			ct::logger::info("Optic %s Plateaued. Calculated Current: %.2f A", groupedKey.toStdString().c_str(), calculatedCurrent);
		}
		else {
			// Fallback: If it never plateaus, keep it at the requested max current.
			proposedLimits.insert(groupedKey, maxCurrentAmp);
			ct::logger::warn("Optic %s reached 255 without plateauing! Keeping at %.4f A", groupedKey.toStdString().c_str(), maxCurrentAmp);
		}

		const bool toggleOffOk = OpticsControl::instance().toggleGroupedOptic(groupedKey, groupedKeys, false);
		ct::logger::info("[Alt8][MaxCurrent] Toggle group OFF result. Group: %s, Result: %s",
			groupedKey.toStdString().c_str(), toggleOffOk ? "PASS" : "FAIL");
		emit incrementProgress();
		ct::logger::info("[Alt8][MaxCurrent] Group complete. Group: %s", groupedKey.toStdString().c_str());
		groupIndex++;
	}

	if (!m_stopRun) {
		qDebug() << "Applying new Max Currents...";
		ct::logger::info("[Alt8][MaxCurrent] Applying proposed max currents. Count: %d", proposedLimits.size());

		for (const auto& key : proposedLimits.keys()) {
			ct::logger::info("[Alt8][MaxCurrent] Apply max current. Group: %s, Current: %.6f A",
				key.toStdString().c_str(), proposedLimits[key]);
			const bool applyOk = OpticsControl::instance().setGroupedOpticMaxCurrent(key, groupedKeys, proposedLimits[key]);
			ct::logger::info("[Alt8][MaxCurrent] Apply max current result. Group: %s, Result: %s",
				key.toStdString().c_str(), applyOk ? "PASS" : "FAIL");
		}
	}
	else {
		ct::logger::warn("[Alt8][MaxCurrent] Calibration stopped before applying proposed max currents.");
	}

	const bool allOffOk = OpticsControl::instance().toggleAllChannels(false);
	ct::logger::info("[Alt8][MaxCurrent] Toggle all channels OFF result: %s", allOffOk ? "PASS" : "FAIL");
	OpticsControl::instance().enableOffset(true);
	ct::logger::info("[Alt8][MaxCurrent] Offset re-enabled.");

	emit stopProgressBar();
	ct::logger::info("[Alt8][MaxCurrent] Finished. StopRun: %s", m_stopRun ? "TRUE" : "FALSE");
}


void JobThread::savePostResult()
{
	for (const auto& res : m_postResults) {
		auto filename = m_rootPath + res.bufferPath; 
		ImageSavingThread::instance().enqueue(filename.toStdString().c_str(), res.frame);
	}
	m_postResults.clear();

	saveFiducialResult();
}

void JobThread::incomingJob(QByteArray byteArray)
{
	QString data = QString(byteArray);

	QString error = QStringLiteral("NO_ERROR");
	QString response = QStringLiteral("R\r");
	QString separator = "#@#";
	bool externalResponse = false;

	ct::logger::info("[com] Receive: %s", data.toStdString().c_str());

	if (data.contains(QStringLiteral("V99OPENLOT")))
	{	//Start Lot
		auto datas = data.split(separator);
		if (datas.size() >= 4) {
			ct::logger::debug("{incomingJob} [%s %s] Lot number: %s",
				datas.at(2).toStdString().c_str(),
				datas.at(3).toStdString().c_str(),
				datas.at(1).toStdString().c_str());

			externalResponse = true;
			emit startLot();
		}
		else {
			response = QStringLiteral("F\r");
		}
	}
	else if (data.contains(QStringLiteral("V99CLOSELOT")))
	{	//End Lot
		auto datas = data.split(separator);
		if (datas.size() >= 3) {
			ct::logger::debug("{incomingJob} [%s %s] Close lot",
				datas.at(1).toStdString().c_str(),
				datas.at(2).toStdString().c_str());

			externalResponse = true;
			emit endLot();
		}
		else {
			response = QStringLiteral("F\r");
		}
	}
	else if (data.contains(QStringLiteral("V99PACKAGE")))
	{	//Inform Recipe Name
		auto datas = data.split(separator);
		if (datas.size() >= 2) {
			auto recipeName = datas.at(1);

			if (true) // for handler recipe name GrianT, VisionApp recipe name GrianT (new handler)
			{
				recipeName = recipeName.remove("\r\n");
			}
			if (false) // for handler recipe name Grian, VisionApp recipe name Grian[@]Top (old handler)
			{
				QString facing = datas.at(2);
				facing = facing.remove("\r\n");   // Top || Bottom 
				recipeName = recipeName + "[@]" + facing;
			}


			ct::logger::debug("{incomingJob} Load recipe: %s", recipeName.toStdString().c_str());
			if (Common::Directory::CurrentRecipe != recipeName) {
				Common::Directory::CurrentRecipe = recipeName;
				qDebug() << "Common::Directory::CurrentRecipe: " << Common::Directory::CurrentRecipe;
				qDebug() << "Common::Directory::getRecipeCurrentPath(): " << Common::Directory::getRecipeCurrentPath();

				if (QFile::exists(Common::Directory::getRecipeCurrentPath())) {
					ct::logger::debug("{incomingJob} open recipe: %s", recipeName.toStdString().c_str());
					emit openRecipe(recipeName);
				}
				else {
					ct::logger::debug("{incomingJob} create recipe: %s", recipeName.toStdString().c_str());
					emit createRecipe(recipeName);
				}
			}
		}
		else {
			response = QStringLiteral("F\r");
		}
	}
	else if (data.contains(QStringLiteral("V99ID")))
	{
		data.chop(2);
		SystemData::instance()._currentBarcode = data.remove(0, 8).toStdString();
	}
	else if (data.contains(QStringLiteral("V01LIVE")))
	{	
		ct::logger::debug("{incomingJob} Turn on live mode");
		emit onLive(m_camID);
	}
	else if (data.contains(QStringLiteral("V01UNLIVE")))
	{	
		ct::logger::debug("{incomingJob} Turn off live mode");
		emit offLive();
	}
	else if (data.contains(QStringLiteral("V01FRAMEREADY")))
	{	//Frame loaded
		m_run1stFOVOnly = false;
		externalResponse = true;
		emit frameReady();
	}
	else if (data.contains(QStringLiteral("V01ENCODER")))
	{
		//Current encoder coordinate
		auto s = data.remove("V01ENCODER");

		getEncoder(s);
		auto& cc = SystemData::instance().currentCoordinate();
		ct::logger::debug("{incomingJob} Current encoder: %f, %f, %f", cc.wx, cc.wy, cc.wz);

		emit encoderReceived(cc);

		//frameready -> jog -> wait encoder -> snap -> jog -> wait encoder
	}
	else if (data.contains(QStringLiteral("V01ONJOGSNAP")))
	{
		snapOptic(m_optic, "", "");
	}
	else if (data.contains(QStringLiteral("V01UNLOADSTRIP")))
	{
		emit unloadStrip();
	}
	else if (data.contains(QStringLiteral("V99POSTREFLOW")))
	{
		auto datas = data.split(separator);
		if (datas.size() == 2) {
			auto postReflowNumber = datas.at(1);
			//TODO: Enable skip FM checking
			//do something with number
		}
	}
	else if (data.contains(QStringLiteral("V99RCPUPLOAD")))
	{
		auto datas = data.split(separator);
		if (datas.size() == 2) {
			auto recipeName = datas.at(1);
			recipeName = recipeName.remove("\r\n");
			externalResponse = true;
			emit uploadRecipe(recipeName);
		}
	}
	else if (data.contains(QStringLiteral("V99RCPDOWNLOAD")))
	{
		auto datas = data.split(separator);
		if (datas.size() == 2) {
			auto recipeName = datas.at(1);
			recipeName = recipeName.remove("\r\n");
			externalResponse = true;
			emit downloadRecipe(recipeName);
		}
	}
	// if running golden recipe, respond will only make during "inspectionDone" function;

	if (data.size() == 3) {
		if (data[0] == "R") {
			return;
		}
	}

	if (!externalResponse) sendToClient(response);
}

QPointF JobThread::waitForLocator(QString expectedViewID) {
	QPointF offset_mm;

	ct::logger::info("[JobThread] Wait for locator: %s", expectedViewID.toStdString().c_str());

	QEventLoop loop;
	QTimer timeoutTimer;
	bool timedOut = false;

	auto connection = QObject::connect(this, &JobThread::locatorReceived, this, [&](QPointF offset, double angle, QString viewID, QString indexID, bool locatorFail, bool locatorAngleFail) {
		if (viewID == expectedViewID) {
			if (!locatorFail) {
				offset_mm.setX(-offset.x() * 1.67 / 1000);
				offset_mm.setY(offset.y() * 1.67 / 1000);
			}
			loop.quit();
		}
	});

	QObject::connect(&timeoutTimer, &QTimer::timeout, [&]() {
		timedOut = true;
		loop.quit();
	});

	timeoutTimer.start(5000);

	loop.exec();

	QObject::disconnect(connection);

	if (timedOut) {
		ct::logger::error("[JobThread] Timeout occurred while waiting for locator: %s", expectedViewID.toStdString().c_str());
	}
	else {
		ct::logger::info("[JobThread] Locator Receive from %s (mm): x:%f, y: %f", expectedViewID.toStdString().c_str(), offset_mm.x(), offset_mm.y());
	}

	return offset_mm;
}

QPointF JobThread::waitForLocator(QString expectedViewID, QString expectedIndexID) {
	QPointF offset_mm;
	ScopedTimeLogger stl("Total locator wait time");

	ct::logger::info("[JobThread] Wait for locator: %s_%s", expectedViewID.toStdString().c_str(), expectedIndexID.toStdString().c_str());


	QEventLoop loop;
	QTimer timeoutTimer;
	bool timedOut = false;

	auto connection = QObject::connect(this, &JobThread::locatorReceived, this, [&](QPointF offset, double angle, QString viewID, QString indexID, bool locatorFail, bool locatorAngleFail) {
		if (viewID == expectedViewID && indexID == expectedIndexID) {
			if (!locatorFail) {
				offset_mm.setX(-offset.x() * 1.67 / 1000);
				offset_mm.setY(offset.y() * 1.67 / 1000);
			}
			loop.quit();
		}
	});

	QObject::connect(&timeoutTimer, &QTimer::timeout, [&]() {
		timedOut = true;
		loop.quit();
	});

	timeoutTimer.start(1000);
	ct::logger::info("[JobThread] before exec");

	loop.exec();

	QObject::disconnect(connection);

	if (timedOut) {
		ct::logger::error("[JobThread] Timeout occurred while waiting for locator: %s_%s", expectedViewID.toStdString().c_str(), expectedIndexID.toStdString().c_str());
	}
	else {
		ct::logger::info("[JobThread] Locator Receive from %s_%s (mm): x:%f, y: %f", expectedViewID.toStdString().c_str(), expectedIndexID.toStdString().c_str(), offset_mm.x(), offset_mm.y());
	}

	return offset_mm;
}

QPointF JobThread::waitForLocator(QString expectedViewID, int expectedRow, int expectedCol) {
	QPointF offset_mm;

	ct::logger::info("[JobThread] Wait for locator: %s_%s", expectedViewID.toStdString().c_str(), expectedRow, expectedCol);

	QEventLoop loop;
	QTimer timeoutTimer;
	bool timedOut = false;

	auto connection = QObject::connect(this, &JobThread::locatorReceived, this, [&](QPointF offset, double angle, QString viewID, QString indexID, bool locatorFail, bool locatorAngleFail) {
		auto indexes = indexID.split("_");
		auto row = indexes.at(0);
		auto col = indexes.at(1);

		if (viewID == expectedViewID && row == expectedRow && col == expectedCol) {
			if (!locatorFail) {
				offset_mm.setX(-offset.x() * 1.67 / 1000);
				offset_mm.setY(offset.y() * 1.67 / 1000);
			}
			loop.quit();
		}
	});

	QObject::connect(&timeoutTimer, &QTimer::timeout, [&]() {
		timedOut = true;
		loop.quit();
	});

	timeoutTimer.start(5000);

	loop.exec();

	QObject::disconnect(connection);

	if (timedOut) {
		ct::logger::error("[JobThread] Timeout occurred while waiting for locator: %s_%s", expectedViewID.toStdString().c_str(), expectedRow, expectedCol);
	}
	else {
		ct::logger::info("[JobThread] Locator Receive from %s_%s (mm): x:%f, y: %f", expectedViewID.toStdString().c_str(), expectedRow, expectedCol, offset_mm.x(), offset_mm.y());
	}

	return offset_mm;
}

bool JobThread::sendToClient(QString msg)
{
	ct::logger::info("[com] Send: %s", msg.toStdString().c_str());
	return m_server->send(msg);
}

ct::UnitResultInfo JobThread::waitForUnitResult(QString expectedViewID, QString expectedIndexID)
{
	ct::UnitResultInfo result;

	ct::logger::info("[JobThread] Wait for result: %s_%s", expectedViewID.toStdString().c_str(), expectedIndexID.toStdString().c_str());

	QString defectCode;

	QEventLoop loop;
	QTimer timeoutTimer;
	bool timedOut = false;

	auto connection = QObject::connect(this, &JobThread::resultReceived, this, [&](QVector<FrameInfo> infos, QHash<QString, ct::UnitResultInfo> results) {

		for (int i = 0; i < infos.size(); i++)
		{
			auto frameInfo = infos[i];
			auto viewID = infos[i].viewID;
			auto indexID = QString("%1_%2").arg(infos[i].row).arg(infos[i].col);

			if (viewID == expectedViewID && indexID == expectedIndexID) {

				if (!(*m_views).contains(viewID)) {
					ct::logger::error("[JobThread] Failed to check unit reuslt. Invalid view ID: %s", viewID.toStdString().c_str());
					continue;
				}

				for (auto opticID : (*m_views)[viewID].opticIDs) {
					result = results[opticID];
				}

				loop.quit();
			}
		}
	});

	QObject::connect(&timeoutTimer, &QTimer::timeout, [&]() {
		timedOut = true;
		loop.quit();
	});

	timeoutTimer.start(1000);

	loop.exec();

	QObject::disconnect(connection);

	if (timedOut) {
		ct::logger::info("[JobThread] Waiting for result timeout : %s_%s", expectedViewID.toStdString().c_str(), expectedIndexID.toStdString().c_str());
	}
	else {
		ct::logger::info("[JobThread] Result Receive for %s_%s: %s", expectedViewID.toStdString().c_str(), expectedIndexID.toStdString().c_str(), defectCode.toStdString().c_str());
	}

	return result;
}

QVector<FrameInfo> JobThread::waitForImageReady()
{
	QVector<FrameInfo> frames;

	ct::logger::info("[JobThread] Wait for image...");

	QEventLoop loop;
	QTimer timeoutTimer;
	bool timedOut = false;
	auto connection = QObject::connect(this, &JobThread::imageReady, this, [&](QVector<FrameInfo> infos) {
		for (int i = 0; i < infos.size(); i++)
		{
			auto frameInfo = infos[i];
			auto viewID = infos[i].viewID;
			auto indexID = QString::number(infos[i].index);
			
			frames = infos;
			loop.quit();
		}
	});

	QObject::connect(&timeoutTimer, &QTimer::timeout, this, [&]() {
		timedOut = true;
		loop.quit();
		});

	timeoutTimer.start(60000);   
	loop.exec();

	QObject::disconnect(connection);

	ct::logger::info("[JobThread] Image Receive");
	
	return frames;
}

FrameInfo JobThread::waitForImagePreprocessed()
{
	FrameInfo frame;

	ct::logger::info("[JobThread] Wait for image...");

	QEventLoop loop;
	auto connection = QObject::connect(this, &JobThread::imagePreprocessed, this, [&](FrameInfo info) {
		frame = info;
		loop.quit();
	});
	loop.exec();

	QObject::disconnect(connection);

	ct::logger::info("[JobThread] Image Receive");

	return frame;
}

FrameInfo JobThread::waitForImagePreprocessed(int timeoutMs)
{
	FrameInfo frame;
	bool received = false;

	ct::logger::info("[JobThread] Wait for image...");

	QEventLoop loop;
	QTimer timer;
	timer.setSingleShot(true);

	auto connection = QObject::connect(this, &JobThread::imagePreprocessed, this, [&](FrameInfo info) {
		frame = info;

		received = true;
		timer.stop();
		loop.quit();
		});

	QObject::connect(&timer, &QTimer::timeout, this, [&]() {
		ct::logger::error("[JobThread] Wait image timeout!");
		loop.quit();
		});


	timer.start(timeoutMs);
	loop.exec();

	QObject::disconnect(connection);


	if (!received) {
		ct::logger::warn("[JobThread] Returning empty FrameInfo.");
		return FrameInfo();
	}

	ct::logger::info("[JobThread] Image Receive");
	return frame;
}

void JobThread::saveFrame(QString rootPath, QVector<FrameInfo> frames)
{
	for (auto& info : frames) {
		QString cid = util::combineID(info.viewID, info.opticID);
		cid = cid + "_" + util::getRowColID(info.row, info.col);

		auto filename = rootPath + cid + g_imgExtension;
		ct::logger::info("Saving: %s", filename.toStdString().c_str());
		
		ImageSavingThread::instance().enqueue(filename.toStdString().c_str(), info);
	}
}

bool JobThread::updateTriggerSequence(QString viewID1, QString viewID2)
{
	QString seqID = QString("%1_%2").arg(viewID1).arg(viewID2);

	if (m_currentTriggerSequence == seqID) return true;

	ScopedTimeLogger stl(QString("[JobThread] Update trigger sequence: %1_%2").arg(viewID1).arg(viewID2).toStdString());

	m_currentTriggerSequence = seqID;

	QVector<LSCManager::SequenceData> datas;

	if (!(*m_views).contains(viewID1)) {
		ct::logger::error("[JobThread] Failed to update trigger sequence: %s", viewID1.toStdString().c_str());
		return false;
	}

	if (!(*m_views).contains(viewID2)) {
		ct::logger::error("[JobThread] Failed to update trigger sequence: %s", viewID2.toStdString().c_str());
		return false;
	}

	auto& v1 = (*m_views)[viewID1];
	auto& v2 = (*m_views)[viewID2];

	auto opticIDs1 = v1.opticIDs;
	auto opticIDs2 = v2.opticIDs;
	QVector<QString> opticIDs1_vec;
	QVector<QString> opticIDs2_vec;

	for (auto opticID : opticIDs1) opticIDs1_vec.append(opticID);
	for (auto opticID : opticIDs2) opticIDs2_vec.append(opticID);

	int maxIndex = opticIDs1.size();
	if (opticIDs2.size() > maxIndex) maxIndex = opticIDs2.size();


	for (int i = 0; i < maxIndex; i++) {
		if (opticIDs1.size() > i && opticIDs2.size() > i) {
			auto opt1 = (*m_optics)[opticIDs1_vec[i]];
			auto opt2 = (*m_optics)[opticIDs2_vec[i]];
			OpticsInfo optic;
			optic.id = "";
			optic.name = "";
			optic.camID = opt1.camID;

			for (auto& key : opt1.M.keys()) {
				auto intensity = opt1.M[key];
				if (intensity == 0) continue;
				optic.M[key] = intensity;
			}

			for (auto& key : opt2.M.keys()) {
				auto intensity = opt2.M[key];
				if (intensity == 0) continue;
				optic.M[key] = intensity;
			}

			if (!appendSequence(optic, datas)) return false;
		}
		else if (opticIDs1.size() > i) {
			if (!appendSequence((*m_optics)[opticIDs1_vec[i]], datas)) return false;
		}
		else if (opticIDs2.size() > i) {
			if (!appendSequence((*m_optics)[opticIDs2_vec[i]], datas)) return false;
		}
	}

	ct::logger::info("Update trigger sequence: %s", m_currentTriggerSequence.toStdString().c_str());
	LSCManager::instance().setTriggerSequence(datas);

	return true;
}

bool JobThread::updateTriggerSequence(QString viewID)
{
	if (m_currentTriggerSequence == viewID) return true;

	ScopedTimeLogger stl(QString("[JobThread] Update trigger sequence: %1").arg(viewID).toStdString());

	m_currentTriggerSequence = viewID;

	if (!(*m_views).contains(viewID)) {
		ct::logger::error("[JobThread] Failed to update trigger sequence: %s", viewID.toStdString().c_str());
		return false;
	}

	auto& v = (*m_views)[viewID];

	QVector<LSCManager::SequenceData> datas;

	for (auto& opticID : v.opticIDs) {
		if (!appendSequence((*m_optics)[opticID], datas)) return false;
	}

	ct::logger::info("Update trigger sequence: %s", m_currentTriggerSequence.toStdString().c_str());
	LSCManager::instance().setTriggerSequence(datas);

	return true;
}

bool JobThread::appendSequence(const OpticsInfo& optic, QVector<LSCManager::SequenceData>& datas)
{
	//each setting
	auto exposure = CAMManager::instance().getExposure(optic.camID);
	auto camLSC = CAMManager::instance().lsc(optic.camID);
	if (camLSC == nullptr) {
		ct::logger::error("Invalid LSC info for camera: %s", optic.camID.toStdString().c_str());
		return false;
	}

	LSCManager::SequenceData data;
	data.exposure_us = exposure;
	data.band = optic.M;
	data.triggerSource = camLSC->triggerSource;

	datas.push_back(data);

	return true;
}

void JobThread::getEncoder(const QString& data)
{
	QString x_direction = data.mid(0, 1);
	QString x_value = data.mid(1, 7);
	QString y_direction = data.mid(8, 1);
	QString y_value = data.mid(9, 7);
	QString z_direction = data.mid(16, 1);
	QString z_value = data.mid(17, 7);

	double wx = x_value.toDouble() / 1000;
	double wy = y_value.toDouble() / 1000;
	double wz = z_value.toDouble() / 1000;

	if (x_direction == "N") wx = -wx;
	if (y_direction == "N") wy = -wy;
	if (z_direction == "N") wz = -wz;
	
	SystemData::instance().setCurrentCoordinate(wx, wy, wz);
}

void JobThread::runPlaneCollection() {
	m_stopRun = false;
	//preAcquisition();
	m_postResults.clear();

	if (m_enableFiducial) searchFiducial();

	collectPlane();
	//postAcquisition();
}

void JobThread::run2D() {
	SystemData::instance().StartInspectionTimer = QDateTime::currentDateTime();
	m_stopRun = false;
	preAcquisition();
	acquire2DImages();
	postAcquisition();
	if (!m_stopRun) emit acquisitionDone();
}

void JobThread::run3D() {
	SystemData::instance().StartInspectionTimer = QDateTime::currentDateTime();
	m_stopRun = false;
	preAcquisition();
	acquire3DImages();
	postAcquisition();
	if (!m_stopRun) emit acquisitionDone();
}

void JobThread::run2D3D() {

	SystemData::instance().StartInspectionTimer = QDateTime::currentDateTime();
	m_extraMoveLog.clear();

	m_stopRun = false;
	preAcquisition();

	auto& sd = SystemData::instance();

	//2D camera acquisition replaced for Pogo: barcode via the SR-X readers,
	//then the 3D scan - each gated by its recipe checkbox
	bool barcodeOk = true;
	if (sd._pitchEnableBarcode) barcodeOk = acquireBarcodeAndOcr();
	else ct::logger::info("[Acq] Barcode flow disabled in recipe, skipped");

	if (sd._pitchEnable3D && barcodeOk) {
		if (sd._setupRegionPitchMode) acquire3DImagesPitch();
		else acquire3DImages();
	}
	else if (!sd._pitchEnable3D) {
		ct::logger::info("[Acq] 3D scan disabled in recipe, skipped");
	}

	postAcquisition();
	if (!m_stopRun) emit acquisitionDone();
}

void JobThread::calibrateGoldenLightingProfile(QString camID, QRectF roi) {
	m_stopRun = false;
	//calibrateOptimumBrightness(m_camID); //TODO: Color camera cant use this method
	getGVTable(camID, m_portabilityInfo->lightingCalibrationInfo.main_GVTable, roi);
}

void JobThread::calibrateCurrentLightingProfile(QString camID, QRectF roi) {
	m_stopRun = false;
	//calibrateOptimumBrightness(m_camID);
	getGVTable(camID, m_portabilityInfo->lightingCalibrationInfo.local_GVTable, roi);
}

void JobThread::processImageReady(QVector<FrameInfo> infos) {
	m_imageReadyFlag = true;
	//if (m_isSetup) saveFrame(Common::Directory::getRecipeSetupImagePath(), infos);
}

void JobThread::processImagePreprocessed(FrameInfo info) {
	m_imagePreprocessedFlag = true;
}

void JobThread::collectZImages(double x, double y, double step_mm, double firstStep, double finalStep, OpticsInfo optic) {
	int index = 0;
	for (double i = firstStep; i > finalStep; i = i - step_mm) {
		jog(x, y, i);

		snapOptic(optic, QString::number(index), "");
		auto info = waitForImagePreprocessed();
		auto mBuf = info.pImage->id();

		auto path = QString("DFF/%1.bmp").arg(index);
		MbufExportA(path.toStdString().c_str(), M_BMP, mBuf);
		index++;
	}
}

void JobThread::displayFOV_fnc(MIL_ID mBuf)
{
	MIL_ID imgClone = M_NULL;
	MbufClone(mBuf, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, &imgClone);
	MbufCopy(mBuf, imgClone);

	emit displayFOV(imgClone);
}

MIL_ID JobThread::preprocessImage(MIL_ID mColor)
{
	auto mBlue = mtrx::extract_channel(mColor, mtrx::Channel::BLUE);
	auto mRed = mtrx::extract_channel(mColor, mtrx::Channel::RED);
	MIL_ID mResult = mtrx::alloc_buffer(mBlue);
	mtrx::BufferCollector bc_mBlue(mBlue);
	mtrx::BufferCollector bc_mRed(mRed);

	////Find circle
	MimArith(mBlue, mRed, mResult, M_SUB + M_SATURATION);
	mtrx::gaussian_filter(mResult, mResult, 3);
	MimBinarize(mResult, mResult, M_BIMODAL + M_GREATER, M_NULL, M_NULL);

	MbufSaveA("test.jpg", mResult);

	return mResult;
}

void JobThread::clearEmptyView() {
	if (!safeGuardView()) return;

	auto views = *m_views;
	
	for (auto it = views.begin(); it != views.end(); ) {
		if (it.value().id.isEmpty()) {
			it = views.erase(it);  // erase returns the next iterator
		}
		else {
			++it;
		}
	}
}

void JobThread::clearEmptyLineScan() {
	if (!safeGuardLineScan()) return;

	auto linescans = *m_linescans;

	for (auto it = linescans.begin(); it != linescans.end(); ) {
		if (it.value().id.isEmpty()) {
			it = linescans.erase(it);  // erase returns the next iterator
		}
		else {
			++it;
		}
	}
}

bool JobThread::safeGuardView() {
	if (m_views == nullptr) {
		ct::logger::error("[JobThread] View is nullptr");
		return false;
	}

	return true;
}

bool JobThread::safeGuardLineScan() {
	if (m_linescans == nullptr) {
		ct::logger::error("[JobThread] Linescan is nullptr");
		return false;
	}

	return true;
}

void JobThread::toJson(const ct::Box2D& obj, QJsonObject& j)
{
	double cx = obj.cx;
	double cy = obj.cy;
	double xmin = obj.xmin;
	double ymin = obj.ymin;
	double xmax = obj.xmax;
	double ymax = obj.ymax;

	j.insert(QStringLiteral("id"), obj.id.c_str());
	j.insert(QStringLiteral("cx"), cx);
	j.insert(QStringLiteral("cy"), cy);
	j.insert(QStringLiteral("w"), obj.w);
	j.insert(QStringLiteral("h"), obj.h);
	j.insert(QStringLiteral("xmin"), xmin);
	j.insert(QStringLiteral("ymin"), ymin);
	j.insert(QStringLiteral("xmax"), xmax);
	j.insert(QStringLiteral("ymax"), ymax);
}

void JobThread::fromJson(const QJsonObject& j, ct::Box2D& obj)
{
	obj.id = jsonHelper::getString(j, "id", "").toStdString();
	obj.cx = jsonHelper::getInteger(j, "cx");
	obj.cy = jsonHelper::getInteger(j, "cy");
	obj.w = jsonHelper::getInteger(j, "w");
	obj.h = jsonHelper::getInteger(j, "h");
	obj.xmin = jsonHelper::getInteger(j, "xmin");
	obj.ymin = jsonHelper::getInteger(j, "ymin");
	obj.xmax = jsonHelper::getInteger(j, "xmax");
	obj.ymax = jsonHelper::getInteger(j, "ymax");

	obj.compute_extremum();
}

void JobThread::toJson(const dat::WorldCoordinate& obj, QJsonObject& j, bool isRelative)
{
	dat::WorldCoordinate relativeObj = obj;
	if (!isRelative) relativeObj = getRelativeRobotPoint(obj);

	j.insert(QStringLiteral("wx"), relativeObj.wx);
	j.insert(QStringLiteral("wy"), relativeObj.wy);
	j.insert(QStringLiteral("wz"), relativeObj.wz);
}

void JobThread::fromJson(const QJsonObject& j, dat::WorldCoordinate& obj, bool isAbsolute)
{
	obj.wx = jsonHelper::getDouble(j, "wx");
	obj.wy = jsonHelper::getDouble(j, "wy");
	obj.wz = jsonHelper::getDouble(j, "wz");

	if (!isAbsolute) obj = getAbsoluteRobotPoint(obj);
}

dat::WorldCoordinate JobThread::getRelativeRobotPoint(dat::WorldCoordinate point)
{
	dat::WorldCoordinate offset;
	getCurrentMachinePortabilityPointOffset(offset);
	dat::WorldCoordinate relativePoint = point - offset;
	return relativePoint;
}

dat::WorldCoordinate JobThread::getAbsoluteRobotPoint(dat::WorldCoordinate point)
{
	dat::WorldCoordinate offset;
	getCurrentMachinePortabilityPointOffset(offset);
	dat::WorldCoordinate absolutePoint = point + offset;
	return absolutePoint;
}

void JobThread::getCurrentMachinePortabilityPointOffset(dat::WorldCoordinate& offset)
{
	if (SystemData::instance()._portability.ref_info.machine_name == QHostInfo::localHostName())
	{
		offset = dat::WorldCoordinate();
	}
	else if (SystemData::instance()._portability.current_info.machine_name == QHostInfo::localHostName())
	{
		offset = SystemData::instance()._portability.current_info.portability_point - SystemData::instance()._portability.ref_info.portability_point;
	}
}

bool JobThread::findPortabilityPattern()
{
	snapOptic(getMainOptics(), "", "");
	auto info = waitForImagePreprocessed();
	auto mSrc = info.pImage->id();

	QImage qimg = mtrx::to_qimg(mSrc);

	SystemData::instance()._portability.ref_info.search_region.compute_extremum();
	auto sr_x = SystemData::instance()._portability.ref_info.search_region.xmin;
	auto sr_y = SystemData::instance()._portability.ref_info.search_region.ymin;
	auto sr_w = SystemData::instance()._portability.ref_info.search_region.w;
	auto sr_h = SystemData::instance()._portability.ref_info.search_region.h;

	QImage croppedImage = qimg.copy(sr_x, sr_y, sr_w, sr_h);

	MIL_ID mBuf = mtrx::to_milID(croppedImage);
	MIL_ID mMono = mtrx::to_mono(mBuf);

	mtrx::BufferCollector bc_mMono(mMono);

	mtrx::PatternOutput output;
	std::string path = QString(Common::Directory::PortabilityPath() + "PortabilityFeature.mod").toStdString();
	if (!mtrx::find_pattern(mMono, path, output))
	{
		ct::logger::debug("pattern not found");
		emit promptMsg("Portability pattern not found!");
		return false;
	}

	ct::logger::debug("output: %f, %f, %f, %f", output.x, output.y, output.w, output.h);

	m_locatedPortabilityPos = QRectF(sr_x + output.x, sr_y + output.y, output.w, output.h);
	//SystemData::instance()._portability.located_region.setGeometry(locatedPoint);
	//SystemData::instance()._portability.located_region.show();

	//auto outputFeature = QRectF(locatedPoint.x() + (output.w / 2 + 1), locatedPoint.y() + (output.h / 2 + 1), locatedPoint.width(), locatedPoint.height());

	emit drawRectFOV("portability", m_locatedPortabilityPos, Qt::yellow);

	return true;
}

bool JobThread::findPortabilityCircle()
{
	snapOptic(getMainOptics(), "", "");
	auto info = waitForImagePreprocessed();
	auto mSrc = info.pImage->id();

	QImage qimg = mtrx::to_qimg(mSrc);

	SystemData::instance()._portability.ref_info.search_region.compute_extremum();
	auto sr_x = SystemData::instance()._portability.ref_info.search_region.xmin;
	auto sr_y = SystemData::instance()._portability.ref_info.search_region.ymin;
	auto sr_w = SystemData::instance()._portability.ref_info.search_region.w;
	auto sr_h = SystemData::instance()._portability.ref_info.search_region.h;

	QImage croppedImage = qimg.copy(sr_x, sr_y, sr_w, sr_h);

	MIL_ID mBuf = mtrx::to_milID(croppedImage);
	MIL_ID mMono = mtrx::to_mono(mBuf);

	mtrx::BufferCollector bc_mMono(mMono);

	mtrx::Circle circle;

	if (!mtrx::find_circle(circle, mMono, SystemData::instance()._portability.ref_info.min_diameter / 2, SystemData::instance()._portability.ref_info.max_diameter / 2, mtrx::CircleType::HIGHEST_SCORE))
	{
		ct::logger::error("circle not found");
		emit promptMsg("Portability circle not found!");
		return false;
	}

	m_locatedPortabilityPos = QRectF(sr_x + circle.cx - circle.radius, sr_y + circle.cy - circle.radius, circle.radius * 2, circle.radius * 2);
	//printf("circleFeature: %f, %f, %f\n", circle.x, circle.y, circle.radius * 2);
	//SystemData::instance()._portability.located_region.setGeometry(locatedPoint);
	//SystemData::instance()._portability.located_region.show();

	//auto outputFeature = QRectF(sr_x + circle.cx, sr_y + circle.cy, locatedPoint.width(), locatedPoint.height());

	emit drawRectFOV("portability", m_locatedPortabilityPos, Qt::yellow);

	return true;
}

void JobThread::simulateOnlineStitching()
{
	const QString basePath = Common::Directory::CurrentImageSetPath + "\\Unstitched\\";
	const QString outputRoot = Common::Directory::CurrentImageSetPath + "\\Restitched\\";

	Common::Directory::createDir(outputRoot);
	SystemData::instance()._workingPath = outputRoot;

	QSet<QString> stitchIDs;

	for (auto v : *m_views) {
		if (v.type == ct::s_child_view) stitchIDs.insert(v.map_to_sview);
	}

	for (auto stitchID : stitchIDs) {
		for (auto v : *m_views) {
			for (auto optID : v.opticIDs) {

				if (v.type != ct::s_child_view) continue;
				if (stitchID != v.map_to_sview) continue;

				auto cid = util::combineID(v.id, optID);
				auto filepath = basePath + cid + ".jpg";

				ct::logger::info("[Offline Unstitched Test] Loading view image: %s", filepath.toStdString().c_str());

				auto& opt = (*m_optics)[optID];

				FrameInfo frame;
				frame.viewID = v.id;
				frame.stitchID = v.map_to_sview;
				frame.opticID = opt.id;
				frame.type = opt.type;
				auto mbuf = MbufRestoreA(filepath.toStdString().c_str(), M_DEFAULT_HOST, M_NULL);
				auto sharedBuf = mtrx::MPM::instance().attach(mbuf);
				frame.pImage = sharedBuf;
				frame.width = mtrx::get_width(frame.pImage->id());
				frame.height = mtrx::get_height(frame.pImage->id());
				frame.channel = mtrx::get_band(frame.pImage->id());
				frame.bufferSize = frame.width * frame.height * frame.channel;
				frame.pixelFormat = ICAM_pixelFormat::RGB8;

				g_imageQueue.push_back(frame);
			}
		}

		//break;

		if (util::isRAMOver(SystemData::instance()._maxAllowRamUsage)) {
			ct::logger::warn("[Acq] Memory overload, waiting for more memory to proceed 2D acquisition...");

			while (util::isRAMOver(SystemData::instance()._maxAllowRamUsage)) {
				if (m_stopRun) break;
				os_tool::goSleep(1000);
			}

			ct::logger::warn("[Acq] Sufficient memory, proceed 2D acquisition.");
		}
	}
}



void JobThread::jog(double x_mm, double y_mm, double z_mm, QString type, bool waitJogDone)
{
	ScopedTimeLogger stl(QString("Jog to: %1, %2, %3").arg(x_mm).arg(y_mm).arg(z_mm).toStdString());
	MachineController::instance().trackTime(QString("Jog %1").arg(type));

	auto axisX = (int)im390::Axis::X;
	auto axisY = (int)im390::Axis::Y;
	auto axisZ = (int)im390::Axis::Z;

	dat::WorldCoordinate c;
	c.wx = x_mm;
	c.wy = y_mm;
	c.wz = z_mm;

	//HARDCODE:
	if (type == "3D") {
		MotionController::instance().set_move_deceleration(m_motionID, axisX, m_xSpeed3d * 3.333);
		MotionController::instance().set_move_max_velocity(m_motionID, axisX, m_xSpeed3d);
	}
	
	std::vector<nvs::motion::MoveParam> moveParams; //Note: EMX100 only support 2 axis for multi move
	nvs::motion::MoveParam x_param, y_param;
	x_param.axis = axisX;
	x_param.position_mm = x_mm;
	y_param.axis = axisY;
	y_param.position_mm = y_mm;
	moveParams.push_back(x_param);
	moveParams.push_back(y_param);

	MotionController::instance().absolute_move(m_motionID, axisZ, z_mm);
	if (type == "3D") {
		MotionController::instance().absolute_move(m_motionID, axisX, x_mm);
	}
	else {
		MotionController::instance().absolute_move(m_motionID, axisX, x_mm);
		MotionController::instance().absolute_move(m_motionID, axisY, y_mm);
		//MotionController::instance().absolute_multi_move(m_motionID, moveParams);
	}

	if (waitJogDone) {
		ct::logger::info("Waiting for jog done...");
		os_tool::doNothing(m_motionReadDelay_ms);
		waitAxis(axisX);
		if (type != "3D") waitAxis(axisY);
		waitAxis(axisZ);

		if (m_encoderCheck) {
			waitEncoderCheck(x_mm, y_mm, z_mm);
		}

		auto c = SystemData::instance().currentCoordinate();
		ct::logger::info("Jog done: %f, %f, %f", c.wx, c.wy, c.wz);
	}

	if (type == "3D") {
		MotionController::instance().set_move_deceleration(m_motionID, axisX, m_xDecel);
		MotionController::instance().set_move_max_velocity(m_motionID, axisX, m_xSpeed);
	}
	
	encoderReceived(c);

	MachineController::instance().logTime(QString("Jog %1").arg(type));
}

void JobThread::jogView(const QView& v, double z_offset)
{
	double new_z = v.world.wz + z_offset;

	if (m_compensateZMap.contains(v.id)) {
		new_z += m_compensateZMap[v.id];
	}

	jogBasedOnFiducial(v.world.wx, v.world.wy, new_z, "2D");

	if (SystemData::instance()._snapDelay_ms != 0) os_tool::doNothing(SystemData::instance()._snapDelay_ms);
}

void JobThread::jogLaser(double x, double y, double z, QString type)
{
	x += m_laserOffset->wx;
	y += m_laserOffset->wy;
	z += m_laserOffset->wz;

	ct::logger::info("Laser offset: %.2f, %.2f, %.2f", m_laserOffset->wx, m_laserOffset->wy, m_laserOffset->wz);
	m_laserOffsetInfo = QStringLiteral("Laser offset: %1, %2, %3").arg(m_laserOffset->wx).arg(m_laserOffset->wy).arg(m_laserOffset->wz);

	bool waitJogDone = true;
	// if (type == "3D") waitJogDone = false;

	double newX = x;
	double newY = y;
	if (type == "2D")
	{
		//clamp the scan-axis target to the soft limit; the shortfall is added back at the scan end move
		double softLimit = 0.0;
		if (SystemData::instance().isLineScanAxisY())
		{
			if (!MotionController::instance().get_soft_limit(m_motionID, (int)Axis::Y, y, softLimit))
			{
				SystemData::instance().m_extraMoveFor3DLaser = std::abs(y) - std::abs(softLimit);
				newY = softLimit;
				ct::logger::warn("[JobThread] Axis %d extra move: %.2f", (int)Axis::Y, SystemData::instance().m_extraMoveFor3DLaser);
			}
		}
		else if (!MotionController::instance().get_soft_limit(m_motionID, (int)Axis::X, x, softLimit))
		{
			SystemData::instance().m_extraMoveFor3DLaser = std::abs(x) - std::abs(softLimit);
			newX = softLimit;
			ct::logger::warn("[JobThread] Axis %d extra move: %.2f", (int)Axis::X, SystemData::instance().m_extraMoveFor3DLaser);
		}
	}

	return jog(newX, newY, z, type, waitJogDone);
}

void JobThread::jogLaserBasedOnFiducial(double x, double y, double z, QString type, bool forceEnable)
{
	if (m_enableFiducial || forceEnable) {
		auto shifted = fiducialForPoint(x, y)->getShiftedPoint(em::V2d(x, y));
		x = shifted.x();
		y = shifted.y();
	}

	x += m_laserOffset->wx;
	y += m_laserOffset->wy;
	z += m_laserOffset->wz;

	return jog(x, y, z, type);
}

void JobThread::jogUser(double x, double y, double z, QString type, bool waitJogDone)
{
	m_stopRun = false; //stale stop flag must not abort waitAxis
	jog(x, y, z, type, waitJogDone);
}

void JobThread::jogSnap(double x, double y, double z, const OpticsInfo& optic)
{
	m_stopRun = false; //stale stop flag must not abort waitAxis - snap needs the jog finished

	jog(x, y, z, "2D");
	if (SystemData::instance()._snapDelay_ms != 0) os_tool::doNothing(SystemData::instance()._snapDelay_ms);
	snapOptic(optic, "", "");
}

void JobThread::jogBasedOnFiducial(double x, double y, double z, QString type, bool forceEnable)
{
	if (m_enableFiducial || forceEnable) {
		auto shifted = fiducialForPoint(x, y)->getShiftedPoint(em::V2d(x, y));
		jog(shifted.x(), shifted.y(), z, type);
		return;
	}

	jog(x, y, z, type);
}

void JobThread::jogLeft(double mm, const OpticsInfo& optic)
{
	m_stopRun = false; //stale stop flag must not abort waitAxis - snap needs the jog finished

	MachineController::instance().trackTime("Jog 2D");
	m_timeLogger.reset_timer();
	auto axis = (int)im390::Axis::X;
	MotionController::instance().relative_move(m_motionID, axis, -mm);
	os_tool::doNothing(m_motionReadDelay_ms);
	waitAxis(axis);
	m_timeLogger.log_duration("[Motion] Jog left");
	MachineController::instance().logTime("Jog 2D");
	snapOptic(optic, "", "");
}

void JobThread::jogRight(double mm, const OpticsInfo& optic)
{
	m_stopRun = false; //stale stop flag must not abort waitAxis - snap needs the jog finished

	MachineController::instance().trackTime("Jog 2D");
	m_timeLogger.reset_timer();
	auto axis = (int)im390::Axis::X;
	MotionController::instance().relative_move(m_motionID, axis, mm);
	os_tool::doNothing(m_motionReadDelay_ms);
	waitAxis(axis);
	m_timeLogger.log_duration("[Motion] Jog right");
	MachineController::instance().logTime("Jog 2D");
	snapOptic(optic, "", "");
}

void JobThread::jogBack(double mm, const OpticsInfo& optic)
{
	m_stopRun = false; //stale stop flag must not abort waitAxis - snap needs the jog finished

	MachineController::instance().trackTime("Jog 2D");
	m_timeLogger.reset_timer();
	auto axis = (int)im390::Axis::Y;
	MotionController::instance().relative_move(m_motionID, axis, -mm);
	os_tool::doNothing(m_motionReadDelay_ms);
	waitAxis(axis);
	m_timeLogger.log_duration("[Motion] Jog back");
	MachineController::instance().logTime("Jog 2D");
	snapOptic(optic, "", "");
}

void JobThread::jogFront(double mm, const OpticsInfo& optic)
{
	m_stopRun = false; //stale stop flag must not abort waitAxis - snap needs the jog finished

	MachineController::instance().trackTime("Jog 2D");
	m_timeLogger.reset_timer();
	auto axis = (int)im390::Axis::Y;
	MotionController::instance().relative_move(m_motionID, axis, mm);
	os_tool::doNothing(m_motionReadDelay_ms);
	waitAxis(axis);
	m_timeLogger.log_duration("[Motion] Jog front");
	MachineController::instance().logTime("Jog 2D");
	snapOptic(optic, "", "");
}

void JobThread::jogUp(double mm, const OpticsInfo& optic)
{
	m_stopRun = false; //stale stop flag must not abort waitAxis - snap needs the jog finished

	MachineController::instance().trackTime("Jog 2D");
	m_timeLogger.reset_timer();
	auto axis = (int)im390::Axis::Z;
	MotionController::instance().relative_move(m_motionID, axis, mm);
	os_tool::doNothing(m_motionReadDelay_ms);
	waitAxis(axis);
	m_timeLogger.log_duration("[Motion] Jog up");
	MachineController::instance().logTime("Jog 2D");
	snapOptic(optic, "", "");
}

void JobThread::jogDown(double mm, const OpticsInfo& optic)
{
	m_stopRun = false; //stale stop flag must not abort waitAxis - snap needs the jog finished

	MachineController::instance().trackTime("Jog 2D");
	m_timeLogger.reset_timer();
	auto axis = (int)im390::Axis::Z;
	MotionController::instance().relative_move(m_motionID, axis, -mm);
	os_tool::doNothing(m_motionReadDelay_ms);
	waitAxis(axis);
	m_timeLogger.log_duration("[Motion] Jog down");
	MachineController::instance().logTime("Jog 2D");
	snapOptic(optic, "", "");
}

void JobThread::homeX()
{
	m_stopRun = false;

	if (!MachineController::instance().isServoOn(Axis::X)) {
		MachineController::instance().notifyError(MachineError::X_SERVO_OFF);
		return;
	}

	MachineController::instance().notifyEvent(MachineEvent::HOMING);

	auto axis = (int)im390::Axis::X;
	MotionController::instance().home(m_motionID, axis);
	os_tool::doNothing(m_motionReadDelay_ms);
	waitAxis(axis);
	MotionController::instance().set_position_mm(m_motionID, 0, axis, 0.0);

	MachineController::instance().notifyEvent(MachineEvent::IDLE);
}

void JobThread::homeY()
{
	m_stopRun = false;

	if (!MachineController::instance().isServoOn(Axis::Y)) {
		MachineController::instance().notifyError(MachineError::Y_SERVO_OFF);
		return;
	}

	MachineController::instance().notifyEvent(MachineEvent::HOMING);

	auto axis = (int)im390::Axis::Y;
	MotionController::instance().home(m_motionID, axis);
	os_tool::doNothing(m_motionReadDelay_ms);
	waitAxis(axis);
	MotionController::instance().set_position_mm(m_motionID, 0, axis, 0.0);

	MachineController::instance().notifyEvent(MachineEvent::IDLE);
}

void JobThread::homeZ()
{
	m_stopRun = false;

	auto o = MotionController::instance().set_servo(m_motionID, 0, (int)Axis::Z, true);

	if (o)
	{
		MachineController::instance().notifyEvent(MachineEvent::Z_SERVO_ON);
	}
	

	if (!MachineController::instance().isServoOn(Axis::Z)) {
		MachineController::instance().notifyError(MachineError::Z_SERVO_OFF);
		return;
	}

	MachineController::instance().notifyEvent(MachineEvent::HOMING);

	os_tool::doNothing(3000);

	MachineController::instance().safelyReleaseBrake();

	auto axis = (int)im390::Axis::Z;
	MotionController::instance().home(m_motionID, axis);
	os_tool::doNothing(m_motionReadDelay_ms);
	waitAxis(axis);
	MotionController::instance().set_position_mm(m_motionID, 0, axis, 0.0);

	MachineController::instance().notifyEvent(MachineEvent::IDLE);
}

void JobThread::homeXYZ()
{
	m_stopRun = false;

	auto o = MotionController::instance().set_servo(m_motionID, 0, (int)Axis::Z, true);

	if (o)
	{
		MachineController::instance().notifyEvent(MachineEvent::Z_SERVO_ON);
	}

	if (!MachineController::instance().isServoOn(Axis::X)) MachineController::instance().notifyError(MachineError::X_SERVO_OFF);
	if (!MachineController::instance().isServoOn(Axis::Y)) MachineController::instance().notifyError(MachineError::Y_SERVO_OFF);
	if (!MachineController::instance().isServoOn(Axis::Z)) MachineController::instance().notifyError(MachineError::Z_SERVO_OFF);

	if (MachineController::instance().getMachineState() == MachineState::S_ERROR) return;

	MachineController::instance().notifyEvent(MachineEvent::HOMING);

	os_tool::doNothing(3000);

	MachineController::instance().safelyReleaseBrake();

	auto axisX = (int)im390::Axis::X;
	auto axisY = (int)im390::Axis::Y;
	auto axisZ = (int)im390::Axis::Z;
	
	auto ret = MotionController::instance().home(m_motionID, axisX);
	ret &= MotionController::instance().home(m_motionID, axisY);
	ret &= MotionController::instance().home(m_motionID, axisZ);

	os_tool::doNothing(m_motionReadDelay_ms);
	waitAxis(axisX);
	waitAxis(axisY);
	waitAxis(axisZ);

	MotionController::instance().set_position_mm(m_motionID, 0, axisX, 0.0);
	MotionController::instance().set_position_mm(m_motionID, 0, axisY, 0.0);
	MotionController::instance().set_position_mm(m_motionID, 0, axisZ, 0.0);

	MachineController::instance().notifyEvent(MachineEvent::HOME_SUCCESS);
}

void JobThread::homeAll()
{
	m_stopRun = false;

	if (!MachineController::instance().isServoOn(Axis::X)) MachineController::instance().notifyError(MachineError::X_SERVO_OFF);
	if (!MachineController::instance().isServoOn(Axis::Y)) MachineController::instance().notifyError(MachineError::Y_SERVO_OFF);
	if (!MachineController::instance().isServoOn(Axis::Z)) MachineController::instance().notifyError(MachineError::Z_SERVO_OFF);


	if (MachineController::instance().getMachineState() == MachineState::S_ERROR) return;

	MachineController::instance().notifyEvent(MachineEvent::HOMING);

	auto axisX = (int)im390::Axis::X;
	auto axisY = (int)im390::Axis::Y;
	auto axisZ = (int)im390::Axis::Z;

	MotionController::instance().home(m_motionID, axisX);
	MotionController::instance().home(m_motionID, axisY);
	MotionController::instance().home(m_motionID, axisZ);

	os_tool::doNothing(m_motionReadDelay_ms);
	waitAxis(axisX);
	waitAxis(axisY);
	waitAxis(axisZ);

	MotionController::instance().set_position_mm(m_motionID, 0, axisX, 0.0);
	MotionController::instance().set_position_mm(m_motionID, 0, axisY, 0.0);
	MotionController::instance().set_position_mm(m_motionID, 0, axisZ, 0.0);

	MachineController::instance().notifyEvent(MachineEvent::HOME_SUCCESS);
}

void JobThread::dryRun(QVector<QVector3D> coords, int loops)
{
	if (coords.isEmpty()) {
		emit dryRunStatus("No coordinates to run.", false);
		return;
	}

	if (!MachineController::instance().isServoOn(Axis::X) ||
		!MachineController::instance().isServoOn(Axis::Y) ||
		!MachineController::instance().isServoOn(Axis::Z)) {
		emit dryRunStatus("Servo is off. Initialize the machine first.", false);
		return;
	}

	m_stopRun = false;
	ct::logger::info("[DryRun] Start: %d points, %d loops", coords.size(), loops);

	for (int loop = 0; loop < loops && !m_stopRun; loop++) {
		for (int i = 0; i < coords.size(); i++) {
			if (m_stopRun) break;

			const auto& c = coords[i];
			emit dryRunStatus(QString("Loop %1/%2, point %3/%4: (%5, %6, %7)")
				.arg(loop + 1).arg(loops).arg(i + 1).arg(coords.size())
				.arg(c.x(), 0, 'f', 3).arg(c.y(), 0, 'f', 3).arg(c.z(), 0, 'f', 3), true);

			jog(c.x(), c.y(), c.z(), "2D", true);
		}
	}

	auto msg = m_stopRun ? QString("Dry run stopped.") : QString("Dry run completed.");
	ct::logger::info("[DryRun] %s", msg.toStdString().c_str());
	emit dryRunStatus(msg, false);
}

void JobThread::reconnectMotion()
{
	ct::logger::info("[Reconnect] Started (JobThread)");

	//1 park + 20 close-wait ticks + 1 re-init + 1 resume
	constexpr int closeWaitSec = 20; //EMX-100 requires ~20s between APS_close and re-init
	emit startProgressBar("Reconnecting Motion Controller...", closeWaitSec + 3, false);

	bool ok = false;

	//The APS re-init must not run concurrently with the machine state polling.
	ct::logger::info("[Reconnect] Parking machine state polling...");
	if (!MachineController::instance().pauseStatePolling(true)) {
		ct::logger::error("[Reconnect] State polling did not park - reconnect aborted");
	}
	else {
		emit incrementProgress();
		ct::logger::info("[Reconnect] State polling parked, releasing motion card...");

		//release here and sit out the EMX close-to-init interval with progress
		//ticks, so Motion_APS::init does not have to block silently
		MotionController::instance().release(m_motionID);
		for (int i = 0; i < closeWaitSec; i++) {
			os_tool::goSleep(1000);
			emit incrementProgress();
		}

		ok = MotionController::instance().reconnect(m_motionID, false /*already released*/);
		emit incrementProgress();
		ct::logger::info("[Reconnect] Motion card reconnect returned: %d", ok);
	}
	MachineController::instance().pauseStatePolling(false);
	emit incrementProgress();
	ct::logger::info("[Reconnect] State polling resumed");

	emit stopProgressBar();
	emit reconnectMotionDone(ok);
	ct::logger::info("[Reconnect] Done (ok=%d)", ok);
}

void JobThread::waitAxis(int axis)
{
	ct::logger::info("[Motion] Waiting for axis %d...", axis);

	// An offline/invalid motion controller makes move_done() return false forever.
	// The old hot spin then never returned, which froze the whole job thread - any
	// BlockingQueuedConnection into it (e.g. snapImage) hung the GUI. Bail out when
	// motion is not initialised, poll instead of spinning, and cap the wait.
	if (!MotionController::instance().is_init(m_motionID)) {
		ct::logger::error("[Motion] waitAxis(%d) skipped: motion controller not initialised", axis);
		SystemData::instance()._MotoIsMoving = false;
		return;
	}

	constexpr int timeoutMs = 120000; //homing can be slow, but never endless
	QElapsedTimer timer;
	timer.start();

	while (!MotionController::instance().move_done(m_motionID, 0, axis)) {
		if (m_stopRun) {
			ct::logger::warn("[Motion] waitAxis(%d) aborted by stop", axis);
			break;
		}
		if (timer.elapsed() > timeoutMs) {
			ct::logger::error("[Motion] waitAxis(%d) timed out after %dms", axis, timeoutMs);
			break;
		}
		os_tool::goSleep(20);
	}

	ct::logger::info("[Motion] Axis %d is in position", axis);
	SystemData::instance()._MotoIsMoving = false;
}

void JobThread::loadToPositionSensor(int index) {
	//NOTE: no board transport on this machine, the part is already in position
	m_stopRun = false;

	while (SystemData::instance()._switchingRecipe) {
		if (m_stopRun) return;
		os_tool::doNothing(100);
	}

	emit signalBoardInPosition(index);
}
// The SR-X reader sockets are owned by SRXManager. These wrappers keep the
// existing queued signalTriggerSRX/signalStopSRX connections working.
void JobThread::triggerSRX()
{
	SRXManager::instance().triggerAll();
}

void JobThread::stopSRX()
{
	SRXManager::instance().stopAll();
}

void JobThread::unloadBoard()
{
	//NOTE: no board transport on this machine, just complete the run
	emit signalBoardUnloaded();
}
void JobThread::waitEncoderCheck(double x_mm, double y_mm, double z_mm)
{
	double allowable_tolerance = 0.1;
	bool reach = false;

	constexpr auto timeout = std::chrono::seconds(15);
	const auto start = std::chrono::steady_clock::now();

	while (!reach) {

		auto c = SystemData::instance().currentCoordinate();

		auto x_reach = abs(c.wx - x_mm) < 0.1;
		auto y_reach = abs(c.wy - y_mm) < 0.1;
		auto z_reach = abs(c.wz - z_mm) < 0.1;

		reach = x_reach && y_reach && z_reach;
		if (reach) break;

		// timeout check
		if (std::chrono::steady_clock::now() - start >= timeout) {
			ct::logger::warn(
				"waitEncoderCheck timeout (x={}, y={}, z={})",
				x_mm, y_mm, z_mm
			);
			break;
		}

		os_tool::doNothing(10);
	}
}
