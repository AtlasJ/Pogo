// =============================================================================
//  AlgoManager_Height3.cpp
//  AlgoManager's 3D Height Measurement 3 half: the source maps, the worker-thread
//  stage runs, the display renders and the "height3" block of algoSetup.json.
//
//  Kept out of AlgoManager.cpp on purpose - V3 is built BESIDE the existing height
//  algo, and none of this should be able to disturb the one that is on the
//  production path.
// =============================================================================

#include "AlgoManager.h"
#include "Logger.h"
#include "QJsonHelper.h"
#include "CommonDir.h"
#include "Utilities.h"
#include "MbufPoolManager.h"

#include <QFile>
#include <QFileInfo>
#include <QImage>
#include <QJsonArray>
#include <QMetaObject>

namespace {

//QRectF <-> json, kept local so the V3 block never depends on AlgoManager.cpp's copies
static QRectF h3JsonToRect(const QJsonObject& o)
{
	return QRectF(o.value("x").toDouble(), o.value("y").toDouble(),
		o.value("w").toDouble(), o.value("h").toDouble());
}

static QJsonObject h3RectToJson(const QRectF& r)
{
	QJsonObject o;
	o.insert("x", r.x()); o.insert("y", r.y());
	o.insert("w", r.width()); o.insert("h", r.height());
	return o;
}

//8-bit QImage -> single channel cv::Mat copy (the QImage's own buffer does not outlive this)
static cv::Mat qimageToGrayMat(const QImage& img)
{
	if (img.isNull()) return cv::Mat();

	const QImage gray = img.convertToFormat(QImage::Format_Grayscale8);
	if (gray.isNull()) return cv::Mat();

	cv::Mat wrapped(gray.height(), gray.width(), CV_8UC1,
		const_cast<uchar*>(gray.bits()), (size_t)gray.bytesPerLine());
	return wrapped.clone();
}

//MIL buffer -> cv::Mat, or an empty Mat when there is nothing attached.
//util::Mil_to_cv allocates a fresh Mat and MbufGet copies into it, so what comes back
//already owns its pixels - cloning it again would be a second copy of 60 MB for nothing.
static cv::Mat milToMatCopy(const mtrx::SharedMilID& buf)
{
	if (!buf || buf->id() == M_NULL) return cv::Mat();

	cv::Mat m;
	util::Mil_to_cv(buf->id(), m);
	return m;
}

} //namespace

// =============================================================================
// Params
// =============================================================================

AlgoHeight3Params AlgoManager::height3Params() const
{
	std::lock_guard<std::mutex> lock(m_mutex);
	return m_height3Params;
}

void AlgoManager::setHeight3Params(const AlgoHeight3Params& p)
{
	std::lock_guard<std::mutex> lock(m_mutex);
	m_height3Params = p;
}

// =============================================================================
// Source maps
// =============================================================================

/*
* The scan's height map AND its intensity map, captured together. V3 needs both -
* the height map is what every number is computed from, the intensity map is what the
* operator looks at to recognise what they are measuring - and they only stay
* pixel-aligned if they come from the same frame.
*/
void AlgoManager::setLastScanMaps(mtrx::SharedMilID heightMap, mtrx::SharedMilID intensityMap)
{
	std::lock_guard<std::mutex> lock(m_mutex);
	m_heightMap = heightMap;
	m_lastScanIntensity = intensityMap;
}

mtrx::SharedMilID AlgoManager::lastScanIntensityMap() const
{
	std::lock_guard<std::mutex> lock(m_mutex);
	return m_lastScanIntensity;
}

bool AlgoManager::height3UseLastScan(QString& error)
{
	mtrx::SharedMilID h, i;
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		h = m_heightMap;
		i = m_lastScanIntensity;
	}

	const cv::Mat height = milToMatCopy(h);
	if (height.empty()) {
		error = QStringLiteral("No scan captured yet - run a 3D scan first, or load maps from file.");
		return false;
	}

	const cv::Mat intensity = milToMatCopy(i);
	if (!intensity.empty() && intensity.size() != height.size()) {
		//refuse rather than crop or stretch: a misaligned intensity map would draw ROIs
		//in the wrong place while looking perfectly reasonable
		error = QStringLiteral("The last scan's intensity map is %1 x %2 but its height map is %3 x %4.")
			.arg(intensity.cols).arg(intensity.rows).arg(height.cols).arg(height.rows);
		return false;
	}

	{
		std::lock_guard<std::mutex> lock(m_height3Mutex);
		m_height3.setSourceMaps(height, intensity);
	}

	if (intensity.empty())
		error = QStringLiteral("Height map taken from the last scan; that scan carried no intensity map.");

	ct::logger::info("[Algo H3] Using last scan: height %d x %d, intensity %s",
		height.cols, height.rows, intensity.empty() ? "none" : "yes");
	return true;
}

/*
* The production path's way in. Differs from height3UseLastScan in two ways that matter:
* it takes the buffers of the frame actually being inspected rather than whatever the GUI
* thread stored last, and a mismatched intensity map is DROPPED rather than treated as an
* error - the pipeline never measures from intensity, so failing a unit over a display-only
* map would be wrong. An unusable HEIGHT map is still a hard failure.
*/
bool AlgoManager::height3SetSourceMaps(mtrx::SharedMilID heightMap,
	mtrx::SharedMilID intensityMap, QString& note)
{
	const cv::Mat height = milToMatCopy(heightMap);
	if (height.empty()) {
		note = QStringLiteral("the frame carried no usable height map");
		return false;
	}

	cv::Mat intensity = milToMatCopy(intensityMap);
	if (!intensity.empty() && intensity.size() != height.size()) {
		note = QStringLiteral("intensity map %1 x %2 does not match the height map %3 x %4 - ignored")
			.arg(intensity.cols).arg(intensity.rows).arg(height.cols).arg(height.rows);
		intensity = cv::Mat();
	}

	{
		std::lock_guard<std::mutex> lock(m_height3Mutex);
		m_height3.setSourceMaps(height, intensity);
	}
	return true;
}

bool AlgoManager::height3LoadHeightFile(const QString& path, QString& error)
{
	if (!QFile::exists(path)) { error = "File not found: " + path; return false; }

	//restored through MIL like every other height map in the app, so a 16-bit Keyence
	//tiff behaves here exactly as it does on the production path
	MIL_ID mBuf = MbufRestoreA(path.toStdString().c_str(), M_DEFAULT_HOST, M_NULL);
	if (mBuf == M_NULL) {
		error = QStringLiteral("Failed to load the height map (a 16-bit tiff is expected).");
		return false;
	}

	cv::Mat height;
	util::Mil_to_cv(mBuf, height);
	//attach so the MIL buffer is freed when this returns; Mil_to_cv has already copied
	//the pixels into a Mat of its own, so nothing here points back at it
	mtrx::SharedMilID owner = mtrx::MPM::instance().attach(mBuf);
	if (height.empty()) {
		error = QStringLiteral("The height map loaded but contained no image data.");
		return false;
	}

	{
		std::lock_guard<std::mutex> lock(m_height3Mutex);
		m_height3.setHeightMap(height);

		//a height map of a different size makes the loaded intensity map meaningless -
		//drop it rather than keep a pair that no longer lines up pixel for pixel
		if (m_height3.hasIntensity() && m_height3.intensitySize() != QSize(height.cols, height.rows))
			m_height3.setIntensityMap(cv::Mat());
	}

	ct::logger::info("[Algo H3] Loaded height map %s (%d x %d)",
		path.toStdString().c_str(), height.cols, height.rows);
	return true;
}

bool AlgoManager::height3LoadIntensityFile(const QString& path, QString& error)
{
	if (!QFile::exists(path)) { error = "File not found: " + path; return false; }

	QImage img(path);
	if (img.isNull()) {
		error = QStringLiteral("Failed to load the intensity map (an 8-bit image is expected).");
		return false;
	}

	const cv::Mat intensity = qimageToGrayMat(img);
	if (intensity.empty()) {
		error = QStringLiteral("The intensity map loaded but contained no image data.");
		return false;
	}

	std::lock_guard<std::mutex> lock(m_height3Mutex);
	if (!m_height3.hasHeight()) {
		error = QStringLiteral("Load the height map first - the intensity map has to match its size.");
		return false;
	}

	const QSize hs = m_height3.sourceSize();
	if (hs != QSize(intensity.cols, intensity.rows)) {
		error = QStringLiteral("Intensity map is %1 x %2 but the height map is %3 x %4 - they must match.")
			.arg(intensity.cols).arg(intensity.rows).arg(hs.width()).arg(hs.height());
		return false;
	}

	m_height3.setIntensityMap(intensity);
	ct::logger::info("[Algo H3] Loaded intensity map %s (%d x %d)",
		path.toStdString().c_str(), intensity.cols, intensity.rows);
	return true;
}

void AlgoManager::height3Clear()
{
	std::lock_guard<std::mutex> lock(m_height3Mutex);
	m_height3.clearAll();
}

// =============================================================================
// State queries
// =============================================================================

bool AlgoManager::height3HasHeight() const
{
	std::lock_guard<std::mutex> lock(m_height3Mutex);
	return m_height3.hasHeight();
}

bool AlgoManager::height3HasIntensity() const
{
	std::lock_guard<std::mutex> lock(m_height3Mutex);
	return m_height3.hasIntensity();
}

bool AlgoManager::height3SegmentReady() const
{
	std::lock_guard<std::mutex> lock(m_height3Mutex);
	return m_height3.segmentReady();
}

bool AlgoManager::height3DatumReady() const
{
	std::lock_guard<std::mutex> lock(m_height3Mutex);
	return m_height3.datumReady();
}

QSize AlgoManager::height3CropSize() const
{
	std::lock_guard<std::mutex> lock(m_height3Mutex);
	return m_height3.cropSize();
}

AlgoHeight3Output AlgoManager::height3Output() const
{
	std::lock_guard<std::mutex> lock(m_height3Mutex);
	return m_height3.output();
}

// =============================================================================
// Display renders
//
// try_lock, never lock: these are called from the GUI thread, including on every
// mouse-move of a 3D drag. If a stage is mid-run the caller gets a null image and
// simply keeps the frame it already has, instead of the whole UI stalling.
// =============================================================================

QImage AlgoManager::height3Image(bool intensity, bool preprocessed, bool segmented, bool colorMapped) const
{
	//params first, then the pipeline: the two locks are never held at the same time,
	//so no call path here can ever be half of a deadlock
	const AlgoHeight3Params p = height3Params();

	std::unique_lock<std::mutex> lock(m_height3Mutex, std::try_to_lock);
	if (!lock.owns_lock()) return QImage();

	if (intensity) return algoH3GrayToQImage(m_height3.intensityForDisplay(segmented));

	return algoH3HeightToQImage(m_height3.heightForDisplay(preprocessed, segmented),
		p.minValidRaw, p.maxValidRaw, colorMapped);
}

QImage AlgoManager::height3Surface(bool preprocessed, bool segmented,
	double yawDeg, double pitchDeg, double zExaggeration, const QSize& outSize,
	AlgoH3SurfaceStyle style) const
{
	const AlgoHeight3Params p = height3Params();

	std::unique_lock<std::mutex> lock(m_height3Mutex, std::try_to_lock);
	if (!lock.owns_lock()) return QImage();

	//the texture has to come from the SAME segmentation state as the geometry, or a
	//straightened crop would be painted with the uncropped map and slide off the part
	return algoH3RenderSurface3D(m_height3.heightForDisplay(preprocessed, segmented),
		m_height3.intensityForDisplay(segmented),
		p.minValidRaw, p.maxValidRaw, yawDeg, pitchDeg, zExaggeration, outSize, style);
}

QImage AlgoManager::height3Relief(bool preprocessed, bool segmented,
	double zExaggeration, bool colorMapped) const
{
	const AlgoHeight3Params p = height3Params();

	std::unique_lock<std::mutex> lock(m_height3Mutex, std::try_to_lock);
	if (!lock.owns_lock()) return QImage();

	return algoH3RenderRelief2D(m_height3.heightForDisplay(preprocessed, segmented),
		p.minValidRaw, p.maxValidRaw, zExaggeration, colorMapped);
}

// =============================================================================
// Runs
// =============================================================================

void AlgoManager::runHeight3(AlgoH3Stage stage)
{
	QMetaObject::invokeMethod(this, "doRunHeight3", Qt::QueuedConnection,
		Q_ARG(int, (int)stage));
}

void AlgoManager::doRunHeight3(int stage)
{
	//the slot takes an int so it can be queued across threads; validate before casting
	//back, because an out-of-range value would index the switch in runStage with garbage
	if (stage < 0 || stage >(int)AlgoH3Stage::All) {
		ct::logger::error("[Algo H3] Ignoring a run request for unknown stage %d", stage);
		return;
	}

	m_busy = true;
	emit busyChanged(true);

	const AlgoHeight3Params p = height3Params();

	AlgoHeight3Output out;
	{
		std::lock_guard<std::mutex> lock(m_height3Mutex);
		m_height3.runStage((AlgoH3Stage)stage, p);
		out = m_height3.output();
	}

	m_busy = false;
	emit busyChanged(false);
	emit height3Finished(stage, out);
}

// =============================================================================
// Persistence - the "height3" block of algoSetup.json
// =============================================================================

void AlgoManager::height3FromJson(const QJsonObject& root)
{
	AlgoHeight3Params p; //defaults, so a recipe with no height3 block is simply untaught

	const QJsonObject h = root.value("height3").toObject();
	if (!h.isEmpty()) {
		// ── section 0 ──
		p.xScaleUmPx = jsonHelper::getDouble(h, "x_scale_um_px", p.xScaleUmPx);
		p.yScaleUmPx = jsonHelper::getDouble(h, "y_scale_um_px", p.yScaleUmPx);
		p.zScaleRawPerUm = jsonHelper::getDouble(h, "z_scale_raw_per_um", p.zScaleRawPerUm);
		p.minValidRaw = jsonHelper::getInteger(h, "min_valid_raw", p.minValidRaw);
		p.maxValidRaw = jsonHelper::getInteger(h, "max_valid_raw", p.maxValidRaw);
		p.minValidHeightUm = jsonHelper::getDouble(h, "min_valid_height_um", p.minValidHeightUm);
		p.maxValidHeightUm = jsonHelper::getDouble(h, "max_valid_height_um", p.maxValidHeightUm);

		// ── section 1 ──
		p.preprocess = (AlgoH3Preprocess)jsonHelper::getInteger(h, "preprocess_method", 0);
		p.medianKernel = jsonHelper::getInteger(h, "median_kernel", p.medianKernel);
		p.gaussianKernel = jsonHelper::getInteger(h, "gaussian_kernel", p.gaussianKernel);
		p.gaussianSigma = jsonHelper::getDouble(h, "gaussian_sigma", p.gaussianSigma);
		p.bilateralDiameter = jsonHelper::getInteger(h, "bilateral_diameter", p.bilateralDiameter);
		p.bilateralSpatialSigma = jsonHelper::getDouble(h, "bilateral_spatial_sigma", p.bilateralSpatialSigma);
		p.bilateralHeightSigma = jsonHelper::getDouble(h, "bilateral_height_sigma", p.bilateralHeightSigma);
		p.openingKernel = jsonHelper::getInteger(h, "opening_kernel", p.openingKernel);
		p.closingKernel = jsonHelper::getInteger(h, "closing_kernel", p.closingKernel);

		// ── section 2 ──
		p.segMethod = (AlgoH3SegMethod)jsonHelper::getInteger(h, "seg_method", 0);
		p.segCheckWidth = jsonHelper::getBool(h, "seg_check_width", false);
		p.segMinWidthUm = jsonHelper::getDouble(h, "seg_min_width_um", 0.0);
		p.segMaxWidthUm = jsonHelper::getDouble(h, "seg_max_width_um", 0.0);
		p.segCheckHeight = jsonHelper::getBool(h, "seg_check_height", false);
		p.segMinHeightUm = jsonHelper::getDouble(h, "seg_min_height_um", 0.0);
		p.segMaxHeightUm = jsonHelper::getDouble(h, "seg_max_height_um", 0.0);
		p.segCheckAngle = jsonHelper::getBool(h, "seg_check_angle", false);
		p.segMinAngleDeg = jsonHelper::getDouble(h, "seg_min_angle_deg", 0.0);
		p.segMaxAngleDeg = jsonHelper::getDouble(h, "seg_max_angle_deg", 0.0);

		// ── section 3 ──
		p.datumMethod = (AlgoH3DatumMethod)jsonHelper::getInteger(h, "datum_method", 0);
		p.datumCheckTilt = jsonHelper::getBool(h, "datum_check_tilt", false);
		p.datumMaxTiltDeg = jsonHelper::getDouble(h, "datum_max_tilt_deg", 0.0);
		p.datumRois.clear();
		for (const auto& v : h.value("datum_rois").toArray())
			p.datumRois.append(h3JsonToRect(v.toObject()));

		// ── section 4 ──
		p.methodId = jsonHelper::getInteger(h, "method_id", 0);
		p.percentile = jsonHelper::getDouble(h, "percentile", p.percentile);

		// ── section 5 ──
		p.roiTypes.clear();
		for (const auto& v : h.value("roi_types").toArray()) {
			const QJsonObject o = v.toObject();
			AlgoH3RoiType t;
			t.name = jsonHelper::getString(o, "name");
			if (t.name.isEmpty()) continue;              //an unnamed type cannot be referenced
			if (p.hasType(t.name)) continue;             //names are the key; never load a duplicate
			const QString colorName = jsonHelper::getString(o, "color");
			if (QColor::isValidColor(colorName)) t.color = QColor(colorName);
			t.minUm = jsonHelper::getDouble(o, "min_um", 0.0);
			t.maxUm = jsonHelper::getDouble(o, "max_um", 0.0);
			t.methodId = jsonHelper::getInteger(o, "method_id", 0);
			p.roiTypes.append(t);
		}

		p.rois.clear();
		for (const auto& v : h.value("rois").toArray()) {
			const QJsonObject o = v.toObject();
			AlgoH3Roi r;
			r.typeName = jsonHelper::getString(o, "type");
			r.rel = h3JsonToRect(o.value("rect").toObject());
			//drop ROIs whose type is gone: the page's own delete keeps these in step, so
			//this only fires for a hand-edited file, and an untyped ROI has no criteria
			if (r.typeName.isEmpty() || !p.hasType(r.typeName)) continue;
			if (r.rel.width() <= 0.0 || r.rel.height() <= 0.0) continue;
			p.rois.append(r);
		}

		// ── section 7 ──
		p.overallCheckCount = jsonHelper::getBool(h, "overall_check_count", false);
		p.overallMaxCount = jsonHelper::getInteger(h, "overall_max_count", 0);
		p.overallCheckRate = jsonHelper::getBool(h, "overall_check_rate", false);
		p.overallMaxRatePct = jsonHelper::getDouble(h, "overall_max_rate_pct", 0.0);
	}

	//clamp anything a hand-edited file could have put out of range, once, here - so no
	//stage has to defend itself against a nonsense recipe
	if (p.minValidRaw < 1) p.minValidRaw = 1;
	if (p.maxValidRaw > 65535) p.maxValidRaw = 65535;
	if (p.maxValidRaw < p.minValidRaw) p.maxValidRaw = p.minValidRaw;
	if (p.xScaleUmPx <= 0.0) p.xScaleUmPx = 5.0;
	if (p.yScaleUmPx <= 0.0) p.yScaleUmPx = 5.0;
	if (p.zScaleRawPerUm <= 0.0) p.zScaleRawPerUm = 1.25;
	if (p.percentile < 0.0) p.percentile = 0.0;
	if (p.percentile > 100.0) p.percentile = 100.0;
	if ((int)p.preprocess < 0 || (int)p.preprocess > (int)AlgoH3Preprocess::Closing)
		p.preprocess = AlgoH3Preprocess::None;
	if ((int)p.segMethod < 0 || (int)p.segMethod >= kAlgoH3SegMethodCount)
		p.segMethod = AlgoH3SegMethod::LargestRegion;
	if ((int)p.datumMethod < 0 || (int)p.datumMethod > (int)AlgoH3DatumMethod::PcaSvd)
		p.datumMethod = AlgoH3DatumMethod::LeastSquares;
	if (!algoH3MethodValid(p.methodId)) p.methodId = 0;

	{
		std::lock_guard<std::mutex> lock(m_mutex);
		m_height3Params = p;
	}

	//the taught geometry just changed, so nothing computed from the old one is still true
	{
		std::lock_guard<std::mutex> lock(m_height3Mutex);
		m_height3.invalidateFrom(AlgoH3Stage::Segment);
	}
}

QJsonObject AlgoManager::height3ToJson() const
{
	const AlgoHeight3Params p = height3Params();

	QJsonObject h;

	h.insert("x_scale_um_px", p.xScaleUmPx);
	h.insert("y_scale_um_px", p.yScaleUmPx);
	h.insert("z_scale_raw_per_um", p.zScaleRawPerUm);
	h.insert("min_valid_raw", p.minValidRaw);
	h.insert("max_valid_raw", p.maxValidRaw);
	h.insert("min_valid_height_um", p.minValidHeightUm);
	h.insert("max_valid_height_um", p.maxValidHeightUm);

	h.insert("preprocess_method", (int)p.preprocess);
	h.insert("median_kernel", p.medianKernel);
	h.insert("gaussian_kernel", p.gaussianKernel);
	h.insert("gaussian_sigma", p.gaussianSigma);
	h.insert("bilateral_diameter", p.bilateralDiameter);
	h.insert("bilateral_spatial_sigma", p.bilateralSpatialSigma);
	h.insert("bilateral_height_sigma", p.bilateralHeightSigma);
	h.insert("opening_kernel", p.openingKernel);
	h.insert("closing_kernel", p.closingKernel);

	h.insert("seg_method", (int)p.segMethod);
	h.insert("seg_check_width", p.segCheckWidth);
	h.insert("seg_min_width_um", p.segMinWidthUm);
	h.insert("seg_max_width_um", p.segMaxWidthUm);
	h.insert("seg_check_height", p.segCheckHeight);
	h.insert("seg_min_height_um", p.segMinHeightUm);
	h.insert("seg_max_height_um", p.segMaxHeightUm);
	h.insert("seg_check_angle", p.segCheckAngle);
	h.insert("seg_min_angle_deg", p.segMinAngleDeg);
	h.insert("seg_max_angle_deg", p.segMaxAngleDeg);

	h.insert("datum_method", (int)p.datumMethod);
	h.insert("datum_check_tilt", p.datumCheckTilt);
	h.insert("datum_max_tilt_deg", p.datumMaxTiltDeg);
	QJsonArray datumRois;
	for (const auto& r : p.datumRois) datumRois.append(h3RectToJson(r));
	h.insert("datum_rois", datumRois);

	h.insert("method_id", p.methodId);
	h.insert("percentile", p.percentile);

	QJsonArray types;
	for (const auto& t : p.roiTypes) {
		QJsonObject o;
		o.insert("name", t.name);
		o.insert("color", t.color.name(QColor::HexRgb));
		o.insert("min_um", t.minUm);
		o.insert("max_um", t.maxUm);
		o.insert("method_id", t.methodId);
		types.append(o);
	}
	h.insert("roi_types", types);

	QJsonArray rois;
	for (const auto& r : p.rois) {
		QJsonObject o;
		o.insert("type", r.typeName);
		o.insert("rect", h3RectToJson(r.rel));
		rois.append(o);
	}
	h.insert("rois", rois);

	h.insert("overall_check_count", p.overallCheckCount);
	h.insert("overall_max_count", p.overallMaxCount);
	h.insert("overall_check_rate", p.overallCheckRate);
	h.insert("overall_max_rate_pct", p.overallMaxRatePct);

	return h;
}
