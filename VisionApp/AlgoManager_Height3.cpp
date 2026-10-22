// =============================================================================
//  AlgoManager_Height3.cpp
//  AlgoManager's 3D Height Measurement 3 half: the source maps, the worker-thread
//  stage runs, the display renders and the machine-wide library of named algos
//  (config/algo_height3/<name>.json).
// =============================================================================

#include "AlgoManager.h"
#include "Logger.h"
#include "QJsonHelper.h"
#include "CommonDir.h"
#include "Utilities.h"
#include "MbufPoolManager.h"

#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QImage>
#include <QJsonArray>
#include <QJsonDocument>
#include <QMetaObject>
#include <QRegularExpression>
#include <QSaveFile>

namespace {

//algo names are matched and kept unique ignoring case and surrounding spaces, like a cable name
static QString h3Key(const QString& name)
{
	return name.trimmed().toLower();
}

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
	const auto it = m_height3Algos.constFind(m_height3EditKey);
	return it != m_height3Algos.cend() ? it->params : AlgoHeight3Params();
}

void AlgoManager::setHeight3Params(const AlgoHeight3Params& p)
{
	std::lock_guard<std::mutex> lock(m_mutex);
	const auto it = m_height3Algos.find(m_height3EditKey);
	if (it != m_height3Algos.end()) it->params = p;
}

/*
* The params of the algo whose results the pipeline holds, for DRAWING them: the valid range
* that colours a crop and the Z scale that reads a pixel off it have to be the ones it was made
* with. Production leaves its unit's algo there, which need not be the one being tuned.
*/
AlgoHeight3Params AlgoManager::height3DisplayParams() const
{
	std::lock_guard<std::mutex> lock(m_mutex);
	auto it = m_height3Algos.constFind(m_height3ResultKey);
	if (it == m_height3Algos.cend()) it = m_height3Algos.constFind(m_height3EditKey);
	return it != m_height3Algos.cend() ? it->params : AlgoHeight3Params();
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

bool AlgoManager::height3PreprocessReady() const
{
	std::lock_guard<std::mutex> lock(m_height3Mutex);
	return m_height3.preprocessReady();
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

bool AlgoManager::height3FindPins(const QRectF* seed, AlgoH3PinFind& out, QString& why) const
{
	const AlgoHeight3Params p = height3Params();
	{
		//the boxes are laid on the tuned algo's crop; another algo's crop and datum are not it
		std::lock_guard<std::mutex> lock(m_mutex);
		if (m_height3ResultKey != m_height3EditKey) {
			why = QStringLiteral("Run this algo first - the results in the pipeline belong to another one.");
			return false;
		}
	}

	//blocking, not try_to_lock: this runs because the operator pressed a button and is waiting
	//for the answer, where the display renders can simply skip a frame
	std::lock_guard<std::mutex> lock(m_height3Mutex);
	return m_height3.findPins(p, seed, out, why);
}

QImage AlgoManager::height3Image(bool intensity, bool preprocessed, bool segmented, bool colorMapped) const
{
	//params first, then the pipeline: the two locks are never held at the same time,
	//so no call path here can ever be half of a deadlock
	const AlgoHeight3Params p = height3DisplayParams();

	std::unique_lock<std::mutex> lock(m_height3Mutex, std::try_to_lock);
	if (!lock.owns_lock()) return QImage();

	if (intensity) return algoH3GrayToQImage(m_height3.intensityForDisplay(segmented));

	return algoH3HeightToQImage(m_height3.heightForDisplay(preprocessed, segmented),
		p.minValidRaw, p.maxValidRaw, colorMapped);
}

bool AlgoManager::height3Pixel(bool preprocessed, bool segmented, int x, int y, FovMapPixel& out) const
{
	out = FovMapPixel();

	//params first, then the pipeline - the two locks are never held together (see height3Image)
	const AlgoHeight3Params p = height3DisplayParams();

	std::unique_lock<std::mutex> lock(m_height3Mutex, std::try_to_lock);
	if (!lock.owns_lock()) return false;

	//the same maps height3Image() draws from, so the pixel read is the pixel shown
	const cv::Mat h = m_height3.heightForDisplay(preprocessed, segmented);
	if (h.empty() || h.type() != CV_16U || x < 0 || y < 0 || x >= h.cols || y >= h.rows) return false;
	out.raw = h.at<ushort>(y, x);

	const cv::Mat in = m_height3.intensityForDisplay(segmented);
	if (!in.empty() && in.type() == CV_8U && in.size() == h.size()) {
		out.hasIntensity = true;
		out.intensity = in.at<uchar>(y, x);
	}

	/*
	* Height above the datum, the way doMeasure computes it: vertical distance to the plane,
	* valid-range pixels only. The plane is fitted in CROP pixels, so this exists only while the
	* crop is what is shown - and it is read under the same lock as the pixel, so the plane and
	* the crop can never come from two different runs. A re-run of Preprocess or Segment wipes
	* the datum, and the value simply disappears until Datum passes again.
	*/
	double a = 0.0, b = 0.0, c = 0.0;
	if (segmented && m_height3.segmentReady() && m_height3.datumPlane(a, b, c)
		&& p.zScaleRawPerUm > 0.0 && out.raw >= p.minValidRaw && out.raw <= p.maxValidRaw) {
		out.hasDatum = true;
		out.aboveDatumUm = (out.raw - (a * x + b * y + c)) / p.zScaleRawPerUm;
	}
	return true;
}

QImage AlgoManager::height3Surface(bool preprocessed, bool segmented,
	double yawDeg, double pitchDeg, double zExaggeration, const QSize& outSize,
	AlgoH3SurfaceStyle style, bool fillHoles, bool showPlane) const
{
	const AlgoHeight3Params p = height3DisplayParams();

	std::unique_lock<std::mutex> lock(m_height3Mutex, std::try_to_lock);
	if (!lock.owns_lock()) return QImage();

	//the texture has to come from the SAME segmentation state as the geometry, or a
	//straightened crop would be painted with the uncropped map and slide off the part
	//the plane lives in the crop's coordinate system, so it can only be drawn on the crop
	const AlgoHeight3Output out = m_height3.output();
	double abc[3] = { out.planeA, out.planeB, out.planeC };
	const bool drawPlane = showPlane && segmented && out.planeValid;

	return algoH3RenderSurface3D(m_height3.heightForDisplay(preprocessed, segmented),
		m_height3.intensityForDisplay(segmented),
		p.minValidRaw, p.maxValidRaw, yawDeg, pitchDeg, zExaggeration, outSize, style, fillHoles,
		drawPlane ? abc : nullptr);
}

QImage AlgoManager::height3Relief(bool preprocessed, bool segmented,
	double zExaggeration, bool colorMapped) const
{
	const AlgoHeight3Params p = height3DisplayParams();

	std::unique_lock<std::mutex> lock(m_height3Mutex, std::try_to_lock);
	if (!lock.owns_lock()) return QImage();

	return algoH3RenderRelief2D(m_height3.heightForDisplay(preprocessed, segmented),
		p.minValidRaw, p.maxValidRaw, zExaggeration, colorMapped);
}

bool AlgoManager::height3CopyDisplayHeight(bool preprocessed, bool segmented, cv::Mat& out) const
{
	std::unique_lock<std::mutex> lock(m_height3Mutex, std::try_to_lock);
	if (!lock.owns_lock()) return false;

	//cloned UNDER the lock: heightForDisplay hands back a header onto the pipeline's own
	//buffer, and the copy is what lets another thread own its map outright, whatever the
	//pipeline does to its buffers next
	out = m_height3.heightForDisplay(preprocessed, segmented).clone();
	return true;
}

// =============================================================================
// Runs
// =============================================================================

//queued to V3's own thread (see AlgoManager::init); before init() it falls back to the shared one
void AlgoManager::runHeight3(AlgoH3Stage stage)
{
	if (m_height3Runner)
		QMetaObject::invokeMethod(m_height3Runner, "run", Qt::QueuedConnection, Q_ARG(int, (int)stage));
	else
		QMetaObject::invokeMethod(this, "doRunHeight3", Qt::QueuedConnection, Q_ARG(int, (int)stage));
}

void AlgoManager::runHeight3UpTo(int stage)
{
	if (m_height3Runner)
		QMetaObject::invokeMethod(m_height3Runner, "runUpTo", Qt::QueuedConnection, Q_ARG(int, stage));
	else
		QMetaObject::invokeMethod(this, "doRunHeight3UpTo", Qt::QueuedConnection, Q_ARG(int, stage));
}

void AlgoManager::runHeight3Named(const QString& algoName)
{
	if (m_height3Runner)
		QMetaObject::invokeMethod(m_height3Runner, "runNamed", Qt::QueuedConnection, Q_ARG(QString, algoName));
	else
		QMetaObject::invokeMethod(this, "doRunHeight3Named", Qt::QueuedConnection, Q_ARG(QString, algoName));
}

//on V3's thread: the work is AlgoManager's, under the same pipeline lock as always
void AlgoH3Runner::run(int stage) { m_mgr->doRunHeight3(stage); }
void AlgoH3Runner::runUpTo(int stage) { m_mgr->doRunHeight3UpTo(stage); }
void AlgoH3Runner::runNamed(QString algoName) { m_mgr->doRunHeight3Named(algoName); }

/*
* Every run ends here, on V3's thread, and the stage results it leaves belong to ONE algo's
* settings - which is recorded. A single-stage run of a different algo must not build on them:
* Measure with this algo's ROIs on another algo's crop and datum would mix two sets of settings
* into one number. So it starts from nothing instead, and its stage says what it needs, exactly
* as on a fresh map. Run All and the reopen re-run start at Preprocess anyway.
*/
AlgoHeight3Output AlgoManager::runHeight3Locked(AlgoH3Stage stage, bool upTo,
	const AlgoHeight3Params& p, const QString& algoName)
{
	const QString key = h3Key(algoName);
	QString resultKey;
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		resultKey = m_height3ResultKey;
	}

	AlgoHeight3Output out;
	{
		std::lock_guard<std::mutex> lock(m_height3Mutex);
		if (resultKey != key) m_height3.invalidateFrom(AlgoH3Stage::Preprocess);
		if (upTo) m_height3.runUpTo(stage, p);
		else m_height3.runStage(stage, p);
		out = m_height3.output();
	}
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		m_height3ResultKey = key;
	}
	out.algoName = algoName;
	return out;
}

//the algo being tuned, name and params read under ONE lock so the two can never be from two algos
QString AlgoManager::height3EditSnapshot(AlgoHeight3Params& p) const
{
	std::lock_guard<std::mutex> lock(m_mutex);
	const auto it = m_height3Algos.constFind(m_height3EditKey);
	p = it != m_height3Algos.cend() ? it->params : AlgoHeight3Params();
	return it != m_height3Algos.cend() ? it->name : QString();
}

/*
* Re-run every stage up to `stage`, which is how a reopened algo gets back to where it was.
*
* The same worker (V3's own thread), the same lock and the same finished signal as a stage the
* operator asked for - the page cannot tell the difference, and so needs no second path for
* showing results that arrived on their own.
*/
void AlgoManager::doRunHeight3UpTo(int stage)
{
	if (stage < 0 || stage >(int)AlgoH3Stage::Overall) return;

	setBusy(true);
	AlgoHeight3Params p;
	const QString name = height3EditSnapshot(p);
	const AlgoHeight3Output out = runHeight3Locked((AlgoH3Stage)stage, true, p, name);
	setBusy(false);
	emit height3Finished(stage, out);
}

void AlgoManager::doRunHeight3(int stage)
{
	//the slot takes an int so it can be queued across threads; validate before casting
	//back, because an out-of-range value would index the switch in runStage with garbage
	if (stage < 0 || stage >(int)AlgoH3Stage::All) {
		ct::logger::error("[Algo H3] Ignoring a run request for unknown stage %d", stage);
		return;
	}

	setBusy(true);
	AlgoHeight3Params p;
	const QString name = height3EditSnapshot(p);
	const AlgoHeight3Output out = runHeight3Locked((AlgoH3Stage)stage, false, p, name);
	setBusy(false);
	emit height3Finished(stage, out);
}

/*
* Production's run: a Run All with the named algo's settings, whichever one the page is tuning.
*
* The name was looked up just before this was queued, but the page could have deleted or renamed
* the algo since. That fails the unit rather than running somebody else's settings in its place.
*/
void AlgoManager::doRunHeight3Named(QString algoName)
{
	setBusy(true);

	bool found = false;
	AlgoHeight3Params p;
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		const auto it = m_height3Algos.constFind(h3Key(algoName));
		if (it != m_height3Algos.cend()) {
			found = true;
			p = it->params;
			algoName = it->name;
		}
	}

	AlgoHeight3Output out;
	if (found) {
		out = runHeight3Locked(AlgoH3Stage::All, false, p, algoName);
	}
	else {
		out.algoName = algoName;
		out.preprocess.ran = true;
		out.preprocess.failReason = QStringLiteral("there is no 3D algo named '%1'").arg(algoName);
	}

	setBusy(false);
	emit height3Finished((int)AlgoH3Stage::All, out);
}

// =============================================================================
// Named algos - machine-wide, one file each
// =============================================================================

namespace {

//one algo's settings from its file (or from a recipe's old "height3" block, which has the same keys)
static AlgoHeight3Params h3ParamsFromJson(const QJsonObject& h, int& lastStage)
{
	AlgoHeight3Params p; //defaults, so an empty block is simply untaught
	lastStage = -1;

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
		/*
		* Clamped, and defaulted to whatever the struct starts on rather than to 0.
		*
		* The clamp is what doSegment's "unknown method" branch assumes has happened: without
		* it a recipe carrying a method number this build does not have would reach the switch
		* and fail the stage, where falling back to the default and saying so is kinder. The
		* struct default means a recipe with no method recorded gets the current one rather
		* than being pinned to whichever happened to be written first.
		*/
		const int segM = jsonHelper::getInteger(h, "seg_method", (int)p.segMethod);
		if (segM >= 0 && segM < kAlgoH3SegMethodCount) p.segMethod = (AlgoH3SegMethod)segM;
		p.segCanvasWidthUm = jsonHelper::getDouble(h, "seg_canvas_width_um", 0.0);
		p.segCanvasHeightUm = jsonHelper::getDouble(h, "seg_canvas_height_um", 0.0);
		p.segBandStepRaw = jsonHelper::getDouble(h, "seg_band_step_raw", 250.0);
		p.segBandMinUm = jsonHelper::getDouble(h, "seg_band_min_um", 0.0);
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
		const int datM = jsonHelper::getInteger(h, "datum_method", (int)p.datumMethod);
		if (datM >= 0 && datM < kAlgoH3DatumMethodCount) p.datumMethod = (AlgoH3DatumMethod)datM;
		p.datumFlatnessUm = jsonHelper::getDouble(h, "datum_flatness_um", 15.0);
		lastStage = jsonHelper::getInteger(h, "last_stage", -1);
		if (lastStage > (int)AlgoH3Stage::Overall) lastStage = -1;
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
			/*
			* MIGRATION: before this key existed, "max > min" was what turned the height
			* criterion on. Defaulting a missing key to false would silently stop checking
			* every type in every recipe already out there, so fall back to the old rule.
			*/
			t.checkHeight = o.contains("check_height")
				? jsonHelper::getBool(o, "check_height", false)
				: (t.maxUm > t.minUm);
			t.checkOffset = jsonHelper::getBool(o, "check_offset", false);
			t.maxOffsetUm = jsonHelper::getDouble(o, "max_offset_um", 0.0);
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
	//negative is meaningless; 0 stays 0 because it means "not set" and must keep refusing
	if (p.segCanvasWidthUm < 0.0) p.segCanvasWidthUm = 0.0;
	if (p.segCanvasHeightUm < 0.0) p.segCanvasHeightUm = 0.0;
	if ((int)p.datumMethod < 0 || (int)p.datumMethod >= kAlgoH3DatumMethodCount)
		p.datumMethod = AlgoH3DatumMethod::LeastSquares;
	if (!algoH3MethodValid(p.methodId)) p.methodId = 0;

	return p;
}

static QJsonObject h3ParamsToJson(const AlgoHeight3Params& p, int lastStage)
{
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
	h.insert("seg_canvas_width_um", p.segCanvasWidthUm);
	h.insert("seg_canvas_height_um", p.segCanvasHeightUm);
	h.insert("seg_band_step_raw", p.segBandStepRaw);
	h.insert("seg_band_min_um", p.segBandMinUm);
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
	h.insert("datum_flatness_um", p.datumFlatnessUm);
	/*
	* How far the pipeline had got, so reopening the algo can put it back there.
	*
	* The stage NUMBER only, never the results. A stored result could outlive the settings it
	* was computed from, and this pipeline is built on the opposite rule - running a stage
	* invalidates every stage after it, so a number on screen is never older than the data
	* behind it. Re-running reproduces the state instead of restoring it, and cannot go stale.
	*
	* The REMEMBERED stage, not the live pipeline. Saves happen for many reasons - a setting
	* touched, the page opening - and reading the pipeline at that moment wrote whatever it held
	* just then: on a page that had only reloaded its maps that was -1, overwriting the stage the
	* last session reached before the reopen could even re-run it. Only a finished page run moves
	* the remembered stage (setHeight3LastStage); production's own runs never do.
	*/
	h.insert("last_stage", lastStage);
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
		o.insert("check_height", t.checkHeight);
		o.insert("min_um", t.minUm);
		o.insert("max_um", t.maxUm);
		o.insert("check_offset", t.checkOffset);
		o.insert("max_offset_um", t.maxOffsetUm);
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

//what an algo's file holds: every setting, plus its name as typed (the file name may lose the case)
static QJsonObject h3AlgoToJson(const QString& name, const AlgoHeight3Params& p, int lastStage)
{
	QJsonObject o = h3ParamsToJson(p, lastStage);
	o.insert("name", name);
	return o;
}

/*
* Why a name cannot be an algo's, or "" when it can. The name IS the file name, so anything
* Windows will not take in one is out - and a leading '_' is kept for files that are not algos
* (_default.json). Uniqueness is the caller's: it needs the library.
*/
static QString h3NameFormatProblem(const QString& name)
{
	const QString n = name.trimmed();
	if (n.isEmpty()) return QStringLiteral("Enter a name.");
	if (n.size() > 64) return QStringLiteral("A name can be at most 64 characters.");
	if (n.startsWith(QLatin1Char('_'))) return QStringLiteral("A name cannot start with '_'.");
	if (n.endsWith(QLatin1Char('.'))) return QStringLiteral("A name cannot end with '.'.");
	static const QString kBad = QStringLiteral("\\/:*?\"<>|");
	for (const QChar c : n) {
		if (c < QLatin1Char(' ') || kBad.contains(c))
			return QStringLiteral("A name cannot contain '%1'.").arg(c);
	}
	static const QRegularExpression kReserved(QStringLiteral("^(CON|PRN|AUX|NUL|COM[1-9]|LPT[1-9])$"),
		QRegularExpression::CaseInsensitiveOption);
	if (kReserved.match(n).hasMatch()) return QStringLiteral("'%1' is a name Windows reserves.").arg(n);
	return QString();
}

//a json file read strictly: unlike jsonHelper::loadJson, a damaged file is an error, not {}
static bool h3ReadJsonObject(const QString& path, QJsonObject& out, QString& error)
{
	QFile f(path);
	if (!f.open(QIODevice::ReadOnly)) { error = f.errorString(); return false; }
	QJsonParseError pe;
	const QJsonDocument doc = QJsonDocument::fromJson(f.readAll(), &pe);
	if (pe.error != QJsonParseError::NoError) { error = pe.errorString(); return false; }
	if (!doc.isObject()) { error = QStringLiteral("not a json object"); return false; }
	out = doc.object();
	return true;
}

//all or nothing: QSaveFile writes beside the target and swaps it in, so a crash mid-save cannot
//leave half an algo behind
static bool h3WriteJsonObject(const QString& path, const QJsonObject& o, QString& error)
{
	QSaveFile f(path);
	if (!f.open(QIODevice::WriteOnly)) { error = f.errorString(); return false; }
	f.write(QJsonDocument(o).toJson(QJsonDocument::Indented));
	if (!f.commit()) { error = f.errorString(); return false; }
	return true;
}

} //namespace

QString AlgoManager::height3AlgoDir()
{
	return QDir::cleanPath(Common::Directory::ConfigPath() + QStringLiteral("/algo_height3")) + QLatin1Char('/');
}

bool AlgoManager::writeHeight3Algo(const QString& name, const QJsonObject& json, QString& error)
{
	QDir().mkpath(height3AlgoDir());
	if (!h3WriteJsonObject(height3AlgoDir() + name + QStringLiteral(".json"), json, error)) {
		error = QStringLiteral("Could not save algo '%1': %2").arg(name, error);
		return false;
	}
	return true;
}

bool AlgoManager::writeHeight3Default(const QString& name, QString& error)
{
	QDir().mkpath(height3AlgoDir());
	QJsonObject o;
	o.insert("default", name);
	if (!h3WriteJsonObject(height3AlgoDir() + QStringLiteral("_default.json"), o, error)) {
		error = QStringLiteral("Could not save the default algo: %1").arg(error);
		return false;
	}
	return true;
}

/*
* Once per session, on the first recipe open. Everything after that is in memory, and each
* change is written to its own file as it is made.
*/
void AlgoManager::loadHeight3Library()
{
	m_height3LibraryLoaded = true;
	const QString dir = height3AlgoDir();

	QMap<QString, H3Algo> algos;
	for (const QFileInfo& fi : QDir(dir).entryInfoList({ QStringLiteral("*.json") }, QDir::Files, QDir::Name)) {
		if (fi.fileName().startsWith(QLatin1Char('_'))) continue;   //_default.json, not an algo

		QJsonObject o;
		QString error;
		if (!h3ReadJsonObject(fi.absoluteFilePath(), o, error)) {
			//left on disk exactly as it is - never overwritten by a save, never imported over
			ct::logger::error("[Algo H3] Skipping algo file %s: %s",
				fi.absoluteFilePath().toStdString().c_str(), error.toStdString().c_str());
			continue;
		}

		H3Algo a;
		//the file name is the name; the copy inside only restores the case Windows may have dropped
		a.name = fi.completeBaseName();
		const QString stored = jsonHelper::getString(o, QStringLiteral("name")).trimmed();
		if (h3Key(stored) == h3Key(a.name)) a.name = stored;
		const QString problem = h3NameFormatProblem(a.name);
		if (!problem.isEmpty()) {
			ct::logger::error("[Algo H3] Skipping algo file %s: %s",
				fi.absoluteFilePath().toStdString().c_str(), problem.toStdString().c_str());
			continue;
		}

		a.params = h3ParamsFromJson(o, a.lastStage);
		a.savedJson = h3AlgoToJson(a.name, a.params, a.lastStage);
		algos.insert(h3Key(a.name), a);
	}

	{
		std::lock_guard<std::mutex> lock(m_mutex);
		m_height3Algos = algos;
	}

	if (algos.isEmpty()) {
		importHeight3FromRecipes();
	}
	else {
		QJsonObject d;
		QString error;
		const QString defaultPath = dir + QStringLiteral("_default.json");
		QString defaultKey;
		if (h3ReadJsonObject(defaultPath, d, error))
			defaultKey = h3Key(jsonHelper::getString(d, QStringLiteral("default")));

		//a default that names no algo (or no default file) would leave every unmatched cable with
		//nothing to run: pick one and SAY so, rather than fail units until somebody notices
		if (!algos.contains(defaultKey)) {
			const QString current = h3Key(Common::Directory::CurrentRecipe);
			const QString chosen = algos.contains(current) ? current : algos.firstKey();
			ct::logger::warn("[Algo H3] Default algo '%s' not found - using '%s'",
				defaultKey.toStdString().c_str(), algos[chosen].name.toStdString().c_str());
			defaultKey = chosen;
			if (!writeHeight3Default(algos[chosen].name, error))
				ct::logger::error("[Algo H3] %s", error.toStdString().c_str());
		}

		std::lock_guard<std::mutex> lock(m_mutex);
		m_height3DefaultKey = defaultKey;
	}

	std::lock_guard<std::mutex> lock(m_mutex);
	selectHeight3AlgoLocked(m_height3DefaultKey);
	ct::logger::info("[Algo H3] %d algo(s) in %s, default '%s'", m_height3Algos.size(),
		dir.toStdString().c_str(), m_height3Algos.value(m_height3DefaultKey).name.toStdString().c_str());
}

/*
* The first launch with no algos: every recipe's old "height3" block becomes an algo named after
* the recipe, so nothing anybody taught is lost, and the open recipe's becomes the default - so
* every unit keeps measuring exactly as it did until a cable gets an algo of its own.
*
* No recipe with a block at all leaves an untaught "Default", so there is always one to tune.
*/
void AlgoManager::importHeight3FromRecipes()
{
	const QString recipeRoot = Common::Directory::LocalPath + QStringLiteral("recipe/");
	QMap<QString, H3Algo> algos;

	for (const QFileInfo& d : QDir(recipeRoot).entryInfoList(QDir::Dirs | QDir::NoDotAndDotDot, QDir::Name)) {
		QJsonObject root;
		QString error;
		if (!h3ReadJsonObject(d.absoluteFilePath() + QStringLiteral("/algoSetup.json"), root, error)) continue;
		const QJsonObject h = root.value(QStringLiteral("height3")).toObject();
		if (h.isEmpty()) continue;

		QString name = d.fileName().trimmed();
		while (name.startsWith(QLatin1Char('_'))) name.remove(0, 1);
		const QString problem = h3NameFormatProblem(name);
		if (!problem.isEmpty() || algos.contains(h3Key(name))) {
			ct::logger::warn("[Algo H3] Not importing recipe '%s': %s", d.fileName().toStdString().c_str(),
				(problem.isEmpty() ? QStringLiteral("an algo of that name exists") : problem).toStdString().c_str());
			continue;
		}

		H3Algo a;
		a.name = name;
		a.params = h3ParamsFromJson(h, a.lastStage);
		const QJsonObject json = h3AlgoToJson(a.name, a.params, a.lastStage);
		if (!writeHeight3Algo(a.name, json, error)) {
			ct::logger::error("[Algo H3] %s", error.toStdString().c_str());
			continue;
		}
		a.savedJson = json;
		algos.insert(h3Key(a.name), a);
		ct::logger::info("[Algo H3] Imported recipe '%s' V3 settings as algo '%s' (%d ROIs)",
			d.fileName().toStdString().c_str(), a.name.toStdString().c_str(), a.params.rois.size());
	}

	if (algos.isEmpty()) {
		H3Algo a;
		a.name = QStringLiteral("Default");
		const QJsonObject json = h3AlgoToJson(a.name, a.params, a.lastStage);
		QString error;
		if (writeHeight3Algo(a.name, json, error)) a.savedJson = json;
		else ct::logger::error("[Algo H3] %s", error.toStdString().c_str());
		algos.insert(h3Key(a.name), a);
		ct::logger::info("[Algo H3] No recipe had V3 settings - created an untaught algo 'Default'");
	}

	const QString current = h3Key(Common::Directory::CurrentRecipe);
	const QString defaultKey = algos.contains(current) ? current : algos.firstKey();
	QString error;
	if (!writeHeight3Default(algos[defaultKey].name, error))
		ct::logger::error("[Algo H3] %s", error.toStdString().c_str());

	std::lock_guard<std::mutex> lock(m_mutex);
	m_height3Algos = algos;
	m_height3DefaultKey = defaultKey;
}

//m_mutex held. The page and the reopen re-run follow the algo being tuned, last stage included.
void AlgoManager::selectHeight3AlgoLocked(const QString& key)
{
	const auto out = m_height3Algos.find(m_height3EditKey);
	if (out != m_height3Algos.end()) out->lastStage = m_height3LastStage;

	m_height3EditKey = key;
	const auto in = m_height3Algos.constFind(key);
	m_height3SavedStage = in != m_height3Algos.cend() ? in->lastStage : -1;
	m_height3LastStage = m_height3SavedStage;
}

QStringList AlgoManager::height3AlgoNames() const
{
	QStringList names;
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		for (const auto& a : m_height3Algos) names << a.name;
	}
	names.sort(Qt::CaseInsensitive);
	return names;
}

QString AlgoManager::height3DefaultAlgo() const
{
	std::lock_guard<std::mutex> lock(m_mutex);
	return m_height3Algos.value(m_height3DefaultKey).name;
}

QString AlgoManager::height3EditAlgo() const
{
	std::lock_guard<std::mutex> lock(m_mutex);
	return m_height3Algos.value(m_height3EditKey).name;
}

QString AlgoManager::height3AlgoNameProblem(const QString& name, const QString& renaming) const
{
	const QString problem = h3NameFormatProblem(name);
	if (!problem.isEmpty()) return problem;

	const QString key = h3Key(name);
	std::lock_guard<std::mutex> lock(m_mutex);
	const auto it = m_height3Algos.constFind(key);
	if (it != m_height3Algos.cend() && key != h3Key(renaming))
		return QStringLiteral("There is already an algo named '%1'.").arg(it->name);
	return QString();
}

/*
* Every setting can differ from one algo to the next, the scale included, so nothing the pipeline
* computed with the last one stands. The maps stay - they are the scan, not the settings.
*/
bool AlgoManager::height3SelectAlgo(const QString& name, QString& error)
{
	const QString key = h3Key(name);
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		if (!m_height3Algos.contains(key)) {
			error = QStringLiteral("There is no algo named '%1'.").arg(name);
			return false;
		}
		if (key == m_height3EditKey) return true;
		selectHeight3AlgoLocked(key);
		m_height3ResultKey.clear();
	}
	{
		std::lock_guard<std::mutex> lock(m_height3Mutex);
		m_height3.invalidateFrom(AlgoH3Stage::Preprocess);
	}
	ct::logger::info("[Algo H3] Tuning algo '%s'", name.toStdString().c_str());
	return true;
}

//a copy of the algo being tuned - a new cable's part is usually a variation on one already taught
bool AlgoManager::height3CreateAlgo(const QString& name, QString& error)
{
	error = height3AlgoNameProblem(name);
	if (!error.isEmpty()) return false;

	H3Algo a;
	a.name = name.trimmed();
	QString from;
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		const auto it = m_height3Algos.constFind(m_height3EditKey);
		if (it != m_height3Algos.cend()) { a.params = it->params; from = it->name; }
	}

	const QJsonObject json = h3AlgoToJson(a.name, a.params, a.lastStage);
	if (!writeHeight3Algo(a.name, json, error)) return false;
	a.savedJson = json;

	{
		std::lock_guard<std::mutex> lock(m_mutex);
		m_height3Algos.insert(h3Key(a.name), a);
		selectHeight3AlgoLocked(h3Key(a.name));
		m_height3ResultKey.clear();
	}
	{
		std::lock_guard<std::mutex> lock(m_height3Mutex);
		m_height3.invalidateFrom(AlgoH3Stage::Preprocess);
	}
	ct::logger::info("[Algo H3] Created algo '%s' from '%s'",
		a.name.toStdString().c_str(), from.toStdString().c_str());
	return true;
}

/*
* Written under the new name first and only then is the old file removed, so a failure part way
* leaves the algo where it was. A change of case only is the same file on this disk, so it is
* simply rewritten - the name inside is what keeps the case.
*/
bool AlgoManager::height3RenameAlgo(const QString& newName, QString& error)
{
	H3Algo a;
	bool wasDefault = false;
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		const auto it = m_height3Algos.constFind(m_height3EditKey);
		if (it == m_height3Algos.cend()) { error = QStringLiteral("No algo is selected."); return false; }
		a = *it;
		a.lastStage = m_height3LastStage;
		wasDefault = (m_height3EditKey == m_height3DefaultKey);
	}

	const QString n = newName.trimmed();
	if (n == a.name) return true;
	error = height3AlgoNameProblem(n, a.name);
	if (!error.isEmpty()) return false;

	const QString oldName = a.name;
	a.name = n;
	const QJsonObject json = h3AlgoToJson(a.name, a.params, a.lastStage);
	if (!writeHeight3Algo(a.name, json, error)) return false;
	a.savedJson = json;

	if (h3Key(oldName) != h3Key(n)) {
		const QString oldPath = height3AlgoDir() + oldName + QStringLiteral(".json");
		if (QFile::exists(oldPath) && !QFile::remove(oldPath)) {
			QFile::remove(height3AlgoDir() + n + QStringLiteral(".json"));
			error = QStringLiteral("Could not rename algo '%1': its file could not be replaced.").arg(oldName);
			return false;
		}
	}

	if (wasDefault && !writeHeight3Default(n, error)) {
		//the algo is renamed either way; only the pointer to it failed, and the next launch picks
		//a default and logs it
		ct::logger::error("[Algo H3] %s", error.toStdString().c_str());
	}

	{
		std::lock_guard<std::mutex> lock(m_mutex);
		const QString oldKey = m_height3EditKey;
		m_height3Algos.remove(oldKey);
		m_height3Algos.insert(h3Key(n), a);
		m_height3EditKey = h3Key(n);
		if (m_height3ResultKey == oldKey) m_height3ResultKey = m_height3EditKey;
		if (wasDefault) m_height3DefaultKey = m_height3EditKey;
	}
	error.clear();
	ct::logger::info("[Algo H3] Renamed algo '%s' to '%s'", oldName.toStdString().c_str(), n.toStdString().c_str());
	return true;
}

/*
* Never the default: every cable without an algo of its own runs it, so removing it would fail
* them all. Never gone for good either - a taught algo is hours of work, so its file moves to
* deleted/ with a timestamp, out of the library but recoverable by moving it back.
*/
bool AlgoManager::height3DeleteAlgo(const QString& name, QString& error)
{
	const QString key = h3Key(name);
	QString stored;
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		const auto it = m_height3Algos.constFind(key);
		if (it == m_height3Algos.cend()) { error = QStringLiteral("There is no algo named '%1'.").arg(name); return false; }
		if (key == m_height3DefaultKey) {
			error = QStringLiteral("'%1' is the default algo - set another one as the default first.").arg(it->name);
			return false;
		}
		stored = it->name;
	}

	const QString dir = height3AlgoDir();
	QDir().mkpath(dir + QStringLiteral("deleted"));
	const QString to = dir + QStringLiteral("deleted/") + stored + QLatin1Char(' ')
		+ QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd-HHmmss")) + QStringLiteral(".json");
	const QString from = dir + stored + QStringLiteral(".json");
	if (QFile::exists(from) && !QFile::rename(from, to)) {
		error = QStringLiteral("Could not delete algo '%1': its file could not be moved.").arg(stored);
		return false;
	}

	bool wasEdit = false;
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		wasEdit = (key == m_height3EditKey);
		if (wasEdit) {
			selectHeight3AlgoLocked(m_height3DefaultKey);
			m_height3ResultKey.clear();
		}
		m_height3Algos.remove(key);
	}
	if (wasEdit) {
		std::lock_guard<std::mutex> lock(m_height3Mutex);
		m_height3.invalidateFrom(AlgoH3Stage::Preprocess);
	}
	ct::logger::info("[Algo H3] Deleted algo '%s' (kept as %s)", stored.toStdString().c_str(), to.toStdString().c_str());
	return true;
}

bool AlgoManager::height3SetDefaultAlgo(const QString& name, QString& error)
{
	const QString key = h3Key(name);
	QString stored;
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		const auto it = m_height3Algos.constFind(key);
		if (it == m_height3Algos.cend()) { error = QStringLiteral("There is no algo named '%1'.").arg(name); return false; }
		stored = it->name;
	}
	if (!writeHeight3Default(stored, error)) return false;
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		m_height3DefaultKey = key;
	}
	ct::logger::info("[Algo H3] Default algo is now '%s'", stored.toStdString().c_str());
	return true;
}

//only when something changed: the auto-save fires for every edit on the algo page, OCR's included
bool AlgoManager::saveHeight3Algo()
{
	QString name;
	QJsonObject json;
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		const auto it = m_height3Algos.find(m_height3EditKey);
		if (it == m_height3Algos.end()) return true;
		it->lastStage = m_height3LastStage;
		json = h3AlgoToJson(it->name, it->params, it->lastStage);
		if (json == it->savedJson) return true;
		name = it->name;
	}

	QString error;
	if (!writeHeight3Algo(name, json, error)) {
		ct::logger::error("[Algo H3] %s", error.toStdString().c_str());
		return false;
	}

	std::lock_guard<std::mutex> lock(m_mutex);
	const auto it = m_height3Algos.find(h3Key(name));
	if (it != m_height3Algos.end()) it->savedJson = json;
	return true;
}

QString AlgoManager::height3AlgoForCable(const QString& cable, bool* usedDefault) const
{
	if (usedDefault) *usedDefault = false;

	std::lock_guard<std::mutex> lock(m_mutex);
	const QString key = h3Key(cable);
	if (!key.isEmpty()) {
		const auto it = m_height3Algos.constFind(key);
		if (it != m_height3Algos.cend()) return it->name;
	}
	const auto def = m_height3Algos.constFind(m_height3DefaultKey);
	if (def == m_height3Algos.cend()) return QString();
	if (usedDefault) *usedDefault = true;
	return def->name;
}
