#include "AlgoManager.h"
#include "PaddleOcrClient.h"
#include "Logger.h"
#include "QJsonHelper.h"
#include "CommonDir.h"
#include "Utilities.h"
#include "MbufPoolManager.h"

#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QTimer>
#include <QElapsedTimer>
#include <QMetaObject>
#include <QtMath>

#include <algorithm>
#include <cmath>

// =============================================================================
// Plane fitting (port of Algo QAlgoHeightMeasurement's live path)
// Convention: plane is z = a*x + b*y + d (c fixed at -1).
// The only linear algebra needed is one 3x3 symmetric solve, done by Cramer.
// =============================================================================

namespace {

struct Plane {
	double a = 0, b = 0, c = -1, d = 0;
	double avg = 0;
	bool valid = false;
};

struct P3 { double x, y, z; };

//solve the symmetric 3x3 system M * p = v by Cramer's rule
static bool solve3x3(const double M[3][3], const double v[3], double p[3])
{
	const double det =
		M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) -
		M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) +
		M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);

	if (std::abs(det) < 1e-12) return false;

	auto detReplaced = [&](int col) {
		double T[3][3];
		for (int r = 0; r < 3; r++)
			for (int cIdx = 0; cIdx < 3; cIdx++)
				T[r][cIdx] = (cIdx == col) ? v[r] : M[r][cIdx];
		return
			T[0][0] * (T[1][1] * T[2][2] - T[1][2] * T[2][1]) -
			T[0][1] * (T[1][0] * T[2][2] - T[1][2] * T[2][0]) +
			T[0][2] * (T[1][0] * T[2][1] - T[1][1] * T[2][0]);
	};

	p[0] = detReplaced(0) / det;
	p[1] = detReplaced(1) / det;
	p[2] = detReplaced(2) / det;
	return true;
}

//centroid-centered least-squares plane fit (QAlgoHeightMeasurement::computeLeastSquaredPlane)
static Plane computeLeastSquaredPlane(const std::vector<P3>& points)
{
	Plane plane;

	const int n = (int)points.size();
	if (n < 3) return plane;

	double meanX = 0, meanY = 0, meanZ = 0;
	for (const auto& p : points) { meanX += p.x; meanY += p.y; meanZ += p.z; }
	meanX /= n; meanY /= n; meanZ /= n;

	//normal equations (A^T A) p = A^T b with rows (X, Y, 1) -> Z, centered
	double ATA[3][3] = { {0,0,0},{0,0,0},{0,0,0} };
	double ATb[3] = { 0,0,0 };

	for (const auto& pt : points) {
		const double X = pt.x - meanX;
		const double Y = pt.y - meanY;
		const double Z = pt.z - meanZ;

		ATA[0][0] += X * X; ATA[0][1] += X * Y; ATA[0][2] += X;
		ATA[1][1] += Y * Y; ATA[1][2] += Y;
		ATA[2][2] += 1.0;

		ATb[0] += X * Z; ATb[1] += Y * Z; ATb[2] += Z;
	}
	ATA[1][0] = ATA[0][1]; ATA[2][0] = ATA[0][2]; ATA[2][1] = ATA[1][2];

	double p[3];
	if (!solve3x3(ATA, ATb, p)) return plane;

	//translate back to original coordinates
	p[2] += meanZ - (p[0] * meanX + p[1] * meanY);

	plane.a = p[0];
	plane.b = p[1];
	plane.c = -1.0;
	plane.d = p[2];
	plane.avg = meanZ;
	plane.valid = true;
	return plane;
}

static double getZFromPlane(const Plane& plane, double x, double y)
{
	return ((-plane.a * x) - (plane.b * y) - plane.d) / plane.c;
}

//robust plane-fit outlier removal: fit, drop points whose perpendicular residual
//exceeds k*sigma, refit - iterated (QAlgoHeightMeasurement removeOutliersByPlaneResidual)
static void removeOutliersByPlaneResidual(std::vector<P3>& points, double k = 2.0, int iterations = 3)
{
	for (int iter = 0; iter < iterations; ++iter) {
		if (points.size() < 4) return;

		Plane pl = computeLeastSquaredPlane(points);
		if (!pl.valid) return;

		const double nrm = std::sqrt(pl.a * pl.a + pl.b * pl.b + pl.c * pl.c);
		if (nrm < 1e-12) return;

		double sum = 0.0, sumSq = 0.0;
		for (const auto& p : points) {
			const double r = (pl.a * p.x + pl.b * p.y + pl.c * p.z + pl.d) / nrm;
			sum += r;
			sumSq += r * r;
		}
		const double n = (double)points.size();
		const double mean = sum / n;
		const double sd = std::sqrt(std::max(0.0, sumSq / n - mean * mean));
		if (sd < 1e-9) return;

		const double lo = mean - k * sd;
		const double hi = mean + k * sd;
		auto it = std::partition(points.begin(), points.end(), [&](const P3& p) {
			const double r = (pl.a * p.x + pl.b * p.y + pl.c * p.z + pl.d) / nrm;
			return r >= lo && r <= hi;
		});
		if (it == points.end()) return;
		points.erase(it, points.end());
	}
}

//signed tilt of the plane along each image axis, in degrees
static void planeTiltXY(const Plane& p, double& tiltX, double& tiltY)
{
	tiltX = qRadiansToDegrees(std::atan(p.a));
	tiltY = qRadiansToDegrees(std::atan(p.b));
}

static QRectF jsonToRect(const QJsonObject& o)
{
	return QRectF(o.value("x").toDouble(), o.value("y").toDouble(),
		o.value("w").toDouble(), o.value("h").toDouble());
}

static QJsonObject rectToJson(const QRectF& r)
{
	QJsonObject o;
	o.insert("x", r.x()); o.insert("y", r.y());
	o.insert("w", r.width()); o.insert("h", r.height());
	return o;
}

//QImage (any format) -> BGR cv::Mat copy
static cv::Mat qimageToBgr(const QImage& img)
{
	QImage rgb = img.convertToFormat(QImage::Format_RGB888);
	cv::Mat wrapped(rgb.height(), rgb.width(), CV_8UC3,
		const_cast<uchar*>(rgb.bits()), (size_t)rgb.bytesPerLine());
	cv::Mat bgr;
	cv::cvtColor(wrapped, bgr, cv::COLOR_RGB2BGR);
	return bgr;
}

} //namespace

// =============================================================================
// Lifecycle
// =============================================================================

AlgoManager& AlgoManager::instance()
{
	static AlgoManager inst;
	return inst;
}

AlgoManager::AlgoManager() {}

AlgoManager::~AlgoManager()
{
	release();
}

void AlgoManager::init()
{
	if (m_initialized) return;
	m_initialized = true;

	qRegisterMetaType<AlgoOcrOutput>("AlgoOcrOutput");
	qRegisterMetaType<AlgoHeightOutput>("AlgoHeightOutput");

	moveToThread(&m_thread);
	m_thread.start();

	ct::logger::info("[Algo] Manager worker thread started");

	//Paddle environment check + warm-up: the server import takes many seconds,
	//so start it now in the background instead of on the first OCR of the day.
	//This only STARTS an existing environment - it does not install anything;
	//the machine needs C:/Advanced/Scripts (script + VirtualEnv) and the
	//per-user .paddlex model cache in place.
	QTimer::singleShot(0, this, [this]() {
		if (!QFileInfo::exists("C:/Advanced/Scripts/pyPaddleAPI.py") ||
			!QFileInfo::exists("C:/Advanced/Scripts/VirtualEnv/Scripts/python.exe")) {
			ct::logger::error("[PaddleOCR] Environment MISSING on this machine "
				"(C:/Advanced/Scripts: pyPaddleAPI.py + VirtualEnv). OCR will fail until it is installed.");
			return;
		}

		if (!m_paddle) m_paddle = new PaddleOcrClient(this);
		ct::logger::info("[PaddleOCR] Warming up server at startup...");
		if (m_paddle->ensureStarted()) ct::logger::info("[PaddleOCR] Server ready");
		else ct::logger::error("[PaddleOCR] Warm-up failed - see previous [PaddleOCR] lines");
	});
}

void AlgoManager::release()
{
	if (!m_thread.isRunning()) return;
	m_thread.quit();
	m_thread.wait(3000);
}

// =============================================================================
// Config access
// =============================================================================

AlgoOcrParams AlgoManager::ocrParams() const
{
	std::lock_guard<std::mutex> lock(m_mutex);
	return m_ocrParams;
}

void AlgoManager::setOcrParams(const AlgoOcrParams& p)
{
	std::lock_guard<std::mutex> lock(m_mutex);
	m_ocrParams = p;
}

AlgoHeightParams AlgoManager::heightParams() const
{
	std::lock_guard<std::mutex> lock(m_mutex);
	return m_heightParams;
}

void AlgoManager::setHeightParams(const AlgoHeightParams& p)
{
	std::lock_guard<std::mutex> lock(m_mutex);
	m_heightParams = p;
}

AlgoLocatorConfig AlgoManager::locatorConfig(AlgoPageAlgo algo) const
{
	std::lock_guard<std::mutex> lock(m_mutex);
	return m_locator[(int)algo];
}

void AlgoManager::setLocatorConfig(AlgoPageAlgo algo, const AlgoLocatorConfig& cfg)
{
	std::lock_guard<std::mutex> lock(m_mutex);
	m_locator[(int)algo] = cfg;
}

OcrPatternConfig AlgoManager::patternConfig() const
{
	std::lock_guard<std::mutex> lock(m_mutex);
	return m_patternConfig;
}

void AlgoManager::setPatternEnabled(bool enabled)
{
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		m_patternConfig.enabled = enabled;
	}
	savePatterns();
}

void AlgoManager::setPatternThreshold(double threshold)
{
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		m_patternConfig.scoreThreshold = threshold;
	}
	savePatterns();
}

void AlgoManager::setPatternLabelEnabled(const QString& label, bool enabled)
{
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		for (auto& l : m_patternConfig.labels) {
			if (l.label == label) { l.enabled = enabled; break; }
		}
	}
	savePatterns();
}

// =============================================================================
// Persistence (per recipe)
// =============================================================================

QString AlgoManager::algoConfigPath() const
{
	return Common::Directory::getRecipeCurrentPath() + "algoSetup.json";
}

QString AlgoManager::patternDir() const
{
	return Common::Directory::getRecipeCurrentPath() + "ocr_patterns/";
}

bool AlgoManager::loadRecipeConfig()
{
	QJsonObject root;
	const bool exists = jsonHelper::loadJson(algoConfigPath(), root);

	{
		std::lock_guard<std::mutex> lock(m_mutex);

		m_ocrParams = AlgoOcrParams();
		m_heightParams = AlgoHeightParams();
		m_locator[0] = AlgoLocatorConfig();
		m_locator[1] = AlgoLocatorConfig();

		if (exists) {
			auto ocr = root.value("ocr").toObject();
			m_ocrParams.orientation = jsonHelper::getInteger(ocr, "orientation", 0);
			m_ocrParams.enlargeOcrImage = jsonHelper::getDouble(ocr, "enlarge", 4.0);
			m_ocrParams.roi1Rows = jsonHelper::getInteger(ocr, "roi1_rows", 1);
			m_ocrParams.roi2Rows = jsonHelper::getInteger(ocr, "roi2_rows", 1);
			m_ocrParams.roi1Columns = jsonHelper::getInteger(ocr, "roi1_columns", 0);
			m_ocrParams.roi2Columns = jsonHelper::getInteger(ocr, "roi2_columns", 0);
			m_ocrParams.patternSearchPadX = jsonHelper::getInteger(ocr, "pattern_search_pad_x", 0);
			m_ocrParams.patternSearchPadY = jsonHelper::getInteger(ocr, "pattern_search_pad_y", 0);
			m_ocrParams.removeSpecialChars = jsonHelper::getBool(ocr, "remove_special_chars", false);
			m_ocrParams.paddleOcrEnabled = jsonHelper::getBool(ocr, "paddle_enabled", true);
			m_ocrParams.roi2Enabled = jsonHelper::getBool(ocr, "roi2_enabled", false);
			m_ocrParams.roi1Geo = jsonToRect(ocr.value("roi1").toObject());
			m_ocrParams.roi2Geo = jsonToRect(ocr.value("roi2").toObject());

			auto h = root.value("height").toObject();
			m_heightParams.intensityPerMicron = jsonHelper::getDouble(h, "intensity_per_micron", 11.0);
			if (m_heightParams.intensityPerMicron <= 0.0) m_heightParams.intensityPerMicron = 11.0;
			m_heightParams.minHeightUm = jsonHelper::getDouble(h, "min_height_um", 0.0);
			m_heightParams.maxHeightUm = jsonHelper::getDouble(h, "max_height_um", 0.0);
			m_heightParams.removeOutliers = jsonHelper::getBool(h, "remove_outliers", true);
			for (const auto& rv : h.value("height_rois").toArray()) {
				m_heightParams.heightRois.append(jsonToRect(rv.toObject()));
			}
			//legacy single-ROI recipes
			if (m_heightParams.heightRois.isEmpty() && h.contains("height_roi")) {
				const QRectF legacy = jsonToRect(h.value("height_roi").toObject());
				if (!legacy.isEmpty()) m_heightParams.heightRois.append(legacy);
			}
			for (const auto& rv : h.value("plane_rois").toArray()) {
				m_heightParams.planeRois.append(jsonToRect(rv.toObject()));
			}

			auto locators = root.value("locators").toArray();
			for (int i = 0; i < locators.size() && i < 2; i++) {
				auto lo = locators[i].toObject();
				auto& cfg = m_locator[i];
				cfg.enabled = jsonHelper::getBool(lo, "enabled", false);
				cfg.scoreThreshold = jsonHelper::getDouble(lo, "score_threshold", 70.0);
				cfg.searchAngle = jsonHelper::getDouble(lo, "search_angle", 10.0);
				cfg.angleOffset = jsonHelper::getDouble(lo, "angle_offset", 0.0);
				cfg.maskMarginW = jsonHelper::getDouble(lo, "mask_margin_w", 0.0);
				cfg.maskMarginH = jsonHelper::getDouble(lo, "mask_margin_h", 0.0);
				cfg.learnX = jsonHelper::getDouble(lo, "learn_x", 0.0);
				cfg.learnY = jsonHelper::getDouble(lo, "learn_y", 0.0);
				cfg.learnAngle = jsonHelper::getDouble(lo, "learn_angle", 0.0);
				cfg.learnRoi = jsonToRect(lo.value("learn_roi").toObject());
				cfg.searchRoi = jsonToRect(lo.value("search_roi").toObject());

				auto file = jsonHelper::getString(lo, "model");
				if (!file.isEmpty()) {
					cfg.modelPath = Common::Directory::getRecipeCurrentPath() + "algo_locator/" + file;
				}
			}
		}
	}

	loadPatterns();

	ct::logger::info("[Algo] Recipe config %s: %s",
		exists ? "loaded" : "not found (defaults)", algoConfigPath().toStdString().c_str());
	return exists;
}

bool AlgoManager::saveRecipeConfig()
{
	QJsonObject root;

	{
		std::lock_guard<std::mutex> lock(m_mutex);

		QJsonObject ocr;
		ocr.insert("orientation", m_ocrParams.orientation);
		ocr.insert("enlarge", m_ocrParams.enlargeOcrImage);
		ocr.insert("roi1_rows", m_ocrParams.roi1Rows);
		ocr.insert("roi2_rows", m_ocrParams.roi2Rows);
		ocr.insert("roi1_columns", m_ocrParams.roi1Columns);
		ocr.insert("roi2_columns", m_ocrParams.roi2Columns);
		ocr.insert("pattern_search_pad_x", m_ocrParams.patternSearchPadX);
		ocr.insert("pattern_search_pad_y", m_ocrParams.patternSearchPadY);
		ocr.insert("remove_special_chars", m_ocrParams.removeSpecialChars);
		ocr.insert("paddle_enabled", m_ocrParams.paddleOcrEnabled);
		ocr.insert("roi2_enabled", m_ocrParams.roi2Enabled);
		ocr.insert("roi1", rectToJson(m_ocrParams.roi1Geo));
		ocr.insert("roi2", rectToJson(m_ocrParams.roi2Geo));
		root.insert("ocr", ocr);

		QJsonObject h;
		h.insert("intensity_per_micron", m_heightParams.intensityPerMicron);
		h.insert("min_height_um", m_heightParams.minHeightUm);
		h.insert("max_height_um", m_heightParams.maxHeightUm);
		h.insert("remove_outliers", m_heightParams.removeOutliers);
		QJsonArray heightRois;
		for (const auto& r : m_heightParams.heightRois) heightRois.append(rectToJson(r));
		h.insert("height_rois", heightRois);
		QJsonArray planes;
		for (const auto& r : m_heightParams.planeRois) planes.append(rectToJson(r));
		h.insert("plane_rois", planes);
		root.insert("height", h);

		QJsonArray locators;
		for (int i = 0; i < 2; i++) {
			const auto& cfg = m_locator[i];
			QJsonObject lo;
			lo.insert("enabled", cfg.enabled);
			lo.insert("score_threshold", cfg.scoreThreshold);
			lo.insert("search_angle", cfg.searchAngle);
			lo.insert("angle_offset", cfg.angleOffset);
			lo.insert("mask_margin_w", cfg.maskMarginW);
			lo.insert("mask_margin_h", cfg.maskMarginH);
			lo.insert("learn_x", cfg.learnX);
			lo.insert("learn_y", cfg.learnY);
			lo.insert("learn_angle", cfg.learnAngle);
			lo.insert("learn_roi", rectToJson(cfg.learnRoi));
			lo.insert("search_roi", rectToJson(cfg.searchRoi));
			lo.insert("model", QFileInfo(cfg.modelPath).fileName()); //recipe-relative
			locators.append(lo);
		}
		root.insert("locators", locators);
	}

	auto ret = jsonHelper::saveJson(algoConfigPath(), QJsonDocument(root));
	if (!ret) ct::logger::error("[Algo] Failed to save %s", algoConfigPath().toStdString().c_str());
	return ret;
}

// =============================================================================
// OCR pattern library (port of IM430 VisionApp_OcrPattern.cpp)
// =============================================================================

void AlgoManager::loadPatterns()
{
	std::lock_guard<std::mutex> lock(m_mutex);

	m_patternFiles.clear();
	m_patternConfig = OcrPatternConfig();

	QJsonObject root;
	if (jsonHelper::loadJson(patternDir() + "config.json", root)) {
		m_patternConfig.enabled = jsonHelper::getBool(root, "enabled", false);
		m_patternConfig.scoreThreshold = jsonHelper::getDouble(root, "score_threshold", 70.0);

		for (const auto& lv : root.value("labels").toArray()) {
			auto lo = lv.toObject();
			OcrPatternLabel lbl;
			lbl.label = jsonHelper::getString(lo, "label");
			lbl.enabled = jsonHelper::getBool(lo, "enabled", true);

			QStringList mpatPaths;
			for (const auto& sv : lo.value("samples").toArray()) {
				//paths are stored recipe-relative
				QString abs = QDir::cleanPath(patternDir() + sv.toString());
				OcrPatternSample s; s.filePath = abs;
				lbl.samples.push_back(s);
				if (QFile::exists(abs)) mpatPaths.push_back(abs);
			}

			m_patternConfig.labels.push_back(lbl);
			m_patternFiles[lbl.label] = mpatPaths;
		}
	}

	//ensure the default 0-9 A-Z labels exist
	QSet<QString> existing;
	for (const auto& l : m_patternConfig.labels) existing.insert(l.label);
	for (char c = '0'; c <= '9'; ++c) {
		if (!existing.contains(QString(c))) m_patternConfig.labels.push_back(OcrPatternLabel{ QString(c), {}, true });
	}
	for (char c = 'A'; c <= 'Z'; ++c) {
		if (!existing.contains(QString(c))) m_patternConfig.labels.push_back(OcrPatternLabel{ QString(c), {}, true });
	}
}

void AlgoManager::savePatterns()
{
	QDir().mkpath(patternDir());

	QJsonObject root;
	QJsonArray labelsArr;

	{
		std::lock_guard<std::mutex> lock(m_mutex);
		root.insert("enabled", m_patternConfig.enabled);
		root.insert("score_threshold", m_patternConfig.scoreThreshold);

		for (const auto& lbl : m_patternConfig.labels) {
			QJsonObject lo;
			lo.insert("label", lbl.label);
			lo.insert("enabled", lbl.enabled);
			QJsonArray samplesArr;
			for (const auto& s : lbl.samples) {
				samplesArr.append(QDir(QDir::cleanPath(patternDir())).relativeFilePath(s.filePath));
			}
			lo.insert("samples", samplesArr);
			labelsArr.append(lo);
		}
	}

	root.insert("labels", labelsArr);
	jsonHelper::saveJson(patternDir() + "config.json", QJsonDocument(root));
}

bool AlgoManager::learnPatternSample(const QImage& fov, const QRectF& roi, const QString& label, QString& error)
{
	if (label.isEmpty()) { error = "Enter the correct character label first."; return false; }
	if (fov.isNull()) { error = "No FOV image available. Load an image first."; return false; }

	cv::Mat fovBgr = qimageToBgr(fov);

	QRect charRect = roi.toRect() & QRect(0, 0, fovBgr.cols, fovBgr.rows);
	if (charRect.isEmpty() || charRect.width() < 2 || charRect.height() < 2) {
		error = "Learn ROI is outside image bounds or too small.";
		return false;
	}

	cv::Mat charCrop = fovBgr(cv::Rect(charRect.x(), charRect.y(), charRect.width(), charRect.height())).clone();
	cv::cvtColor(charCrop, charCrop, cv::COLOR_BGR2GRAY);

	MIL_ID mBuf = M_NULL;
	util::cv_to_Mil(charCrop, mBuf);
	if (mBuf == M_NULL) { error = "Failed to create MIL buffer from crop."; return false; }

	int sampleIdx = 0;
	QString mpatPath, jpgPath;
	{
		std::lock_guard<std::mutex> lock(m_mutex);

		OcrPatternLabel* lblEntry = nullptr;
		for (auto& l : m_patternConfig.labels) {
			if (l.label == label) { lblEntry = &l; break; }
		}
		if (!lblEntry) {
			m_patternConfig.labels.push_back(OcrPatternLabel{ label, {}, true });
			lblEntry = &m_patternConfig.labels.back();
		}

		sampleIdx = lblEntry->samples.size();
		QDir().mkpath(patternDir() + label);
		mpatPath = patternDir() + label + QStringLiteral("/sample_%1.mpat").arg(sampleIdx);
		jpgPath = patternDir() + label + QStringLiteral("/sample_%1.jpg").arg(sampleIdx);

		OcrPatternSample s; s.filePath = mpatPath;
		lblEntry->samples.push_back(s);
	}

	//50x50 thumbnail for the UI grid only; the model keeps the original resolution
	cv::Mat thumb;
	cv::resize(charCrop, thumb, cv::Size(50, 50), 0, 0, cv::INTER_CUBIC);
	cv::imwrite(jpgPath.toStdString(), thumb);

	mtrx::PatternInput patInput;
	patInput.filename = mpatPath.toStdString();
	patInput.min_score = 0.0;
	patInput.learn_x = 0;
	patInput.learn_y = 0;
	patInput.learn_w = charCrop.cols;
	patInput.learn_h = charCrop.rows;
	patInput.smoothness = 85;
	patInput.enable_angle = false; //characters don't need rotation search

	mtrx::PatternOutput patOutput;
	mtrx::learn_pattern(mBuf, patInput, patOutput);
	MbufFree(mBuf);

	{
		std::lock_guard<std::mutex> lock(m_mutex);
		m_patternFiles[label].push_back(mpatPath);
	}

	savePatterns();
	emit patternsChanged();

	ct::logger::info("[OcrPattern] Learned sample %d for '%s' -> %s",
		sampleIdx, label.toStdString().c_str(), mpatPath.toStdString().c_str());
	return true;
}

bool AlgoManager::deletePatternSample(const QString& label, int sampleIndex)
{
	QString mpatPath;
	{
		std::lock_guard<std::mutex> lock(m_mutex);

		for (auto& l : m_patternConfig.labels) {
			if (l.label != label) continue;
			if (sampleIndex < 0 || sampleIndex >= l.samples.size()) return false;
			mpatPath = l.samples[sampleIndex].filePath;
			l.samples.removeAt(sampleIndex);
			break;
		}
		if (mpatPath.isEmpty()) return false;

		m_patternFiles[label].removeAll(mpatPath);
	}

	QString jpgPath = mpatPath;
	jpgPath.replace(QStringLiteral(".mpat"), QStringLiteral(".jpg"));
	QFile::remove(mpatPath);
	QFile::remove(jpgPath);

	savePatterns();
	emit patternsChanged();
	return true;
}

// =============================================================================
// Locator (QAlgoLocator style via mtrx pattern helpers)
// =============================================================================

bool AlgoManager::learnLocatorModel(AlgoPageAlgo algo, const QImage& fov, const QRectF& learnRoi, QString& error)
{
	if (fov.isNull()) { error = "No FOV image available."; return false; }

	cv::Mat fovBgr = qimageToBgr(fov);
	cv::Mat fovGray;
	cv::cvtColor(fovBgr, fovGray, cv::COLOR_BGR2GRAY);

	QRect r = learnRoi.toRect() & QRect(0, 0, fovGray.cols, fovGray.rows);
	if (r.width() < 10 || r.height() < 10) { error = "Learn ROI too small."; return false; }

	cv::Mat crop = fovGray(cv::Rect(r.x(), r.y(), r.width(), r.height())).clone();

	AlgoLocatorConfig cfg = locatorConfig(algo);

	//mask margin: fill the interior with the mean so only the border matters
	const int marginW = (int)(crop.cols * cfg.maskMarginW / 100.0);
	const int marginH = (int)(crop.rows * cfg.maskMarginH / 100.0);
	if ((marginW > 0 || marginH > 0) && marginW < crop.cols / 2 && marginH < crop.rows / 2) {
		cv::Scalar meanVal = cv::mean(crop);
		cv::Rect interior(marginW, marginH, crop.cols - 2 * marginW, crop.rows - 2 * marginH);
		crop(interior).setTo(meanVal);
	}

	const QString dir = Common::Directory::getRecipeCurrentPath() + "algo_locator/";
	QDir().mkpath(dir);
	const QString mpatPath = dir + QStringLiteral("locator_%1.mpat").arg((int)algo);
	const QString jpgPath = dir + QStringLiteral("locator_%1.jpg").arg((int)algo);
	cv::imwrite(jpgPath.toStdString(), crop);

	MIL_ID mCrop = M_NULL;
	util::cv_to_Mil(crop, mCrop);
	if (mCrop == M_NULL) { error = "Failed to create MIL buffer."; return false; }

	mtrx::PatternInput patInput;
	patInput.filename = mpatPath.toStdString();
	patInput.min_score = 0.0;
	patInput.learn_x = 0;
	patInput.learn_y = 0;
	patInput.learn_w = crop.cols;
	patInput.learn_h = crop.rows;
	patInput.smoothness = 85;
	patInput.enable_angle = true;
	patInput.angle_step = 1.0;
	patInput.angleAccuracy = 0.05;         //fine accuracy for locator
	patInput.searchAngle = cfg.searchAngle; //baked into the .mpat

	mtrx::PatternOutput patOutput;
	mtrx::learn_pattern(mCrop, patInput, patOutput);
	MbufFree(mCrop);

	//find on the full FOV to establish the learn-time reference position (centre)
	MIL_ID mFov = M_NULL;
	util::cv_to_Mil(fovGray, mFov);
	mtrx::PatternOutput findOutput;
	findOutput.acceptance_min_score = 1.0;
	const bool found = mtrx::find_pattern(mFov, mpatPath.toStdString(), findOutput);
	if (mFov) MbufFree(mFov);

	if (!found || findOutput.score < 1.0) {
		QFile::remove(mpatPath);
		error = "Pattern was learned but could not be found in the current image. Try a more distinctive region.";
		return false;
	}

	cfg.modelPath = mpatPath;
	cfg.learnX = findOutput.cx;
	cfg.learnY = findOutput.cy;
	cfg.learnAngle = findOutput.angle;
	cfg.learnRoi = learnRoi;
	setLocatorConfig(algo, cfg);
	saveRecipeConfig();

	ct::logger::info("[AlgoLocator] Learned model for algo %d at (%.1f, %.1f) angle %.2f score %.1f",
		(int)algo, cfg.learnX, cfg.learnY, cfg.learnAngle, findOutput.score);
	return true;
}

AlgoLocatorResult AlgoManager::runLocator(const cv::Mat& fovBgr, const AlgoLocatorConfig& cfg,
	QVector<AlgoOverlayItem>& overlay)
{
	AlgoLocatorResult result;

	if (!cfg.enabled || cfg.modelPath.isEmpty()) return result; //not assigned - fine
	result.ran = true;

	if (!QFile::exists(cfg.modelPath)) {
		ct::logger::error("[AlgoLocator] Model missing: %s", cfg.modelPath.toStdString().c_str());
		return result;
	}

	cv::Mat fovGray;
	cv::cvtColor(fovBgr, fovGray, cv::COLOR_BGR2GRAY);

	//crop to search region if configured
	int searchOffX = 0, searchOffY = 0;
	cv::Mat searchImg = fovGray;
	if (!cfg.searchRoi.isEmpty()) {
		QRect sr = cfg.searchRoi.toRect() & QRect(0, 0, fovGray.cols, fovGray.rows);
		if (sr.width() > 10 && sr.height() > 10) {
			searchImg = fovGray(cv::Rect(sr.x(), sr.y(), sr.width(), sr.height())).clone();
			searchOffX = sr.x();
			searchOffY = sr.y();
		}
	}

	MIL_ID mSearch = M_NULL;
	util::cv_to_Mil(searchImg, mSearch);
	if (mSearch == M_NULL) return result;

	mtrx::PatternOutput patOut;
	patOut.acceptance_min_score = cfg.scoreThreshold;
	const bool found = mtrx::find_pattern(mSearch, cfg.modelPath.toStdString(), patOut);
	MbufFree(mSearch);

	if (!found || patOut.score < cfg.scoreThreshold) {
		ct::logger::warn("[AlgoLocator] No match (best %.1f, threshold %.1f) - running unshifted",
			patOut.score, cfg.scoreThreshold);
		overlay.append(AlgoOverlayItem::makeText(
			QStringLiteral("Locator: no match (%1)").arg(patOut.score, 0, 'f', 1),
			QPointF(10, 10), QColor(255, 165, 0), 16));
		return result;
	}

	result.found = true;
	result.score = patOut.score;
	result.foundPos = QPointF(patOut.cx + searchOffX, patOut.cy + searchOffY);
	result.deltaX = result.foundPos.x() - cfg.learnX;
	result.deltaY = result.foundPos.y() - cfg.learnY;
	result.deltaAngle = patOut.angle - cfg.learnAngle + cfg.angleOffset;

	overlay.append(AlgoOverlayItem::makeRect(
		QRectF(result.foundPos.x() - patOut.w / 2, result.foundPos.y() - patOut.h / 2, patOut.w, patOut.h),
		Qt::blue));
	overlay.append(AlgoOverlayItem::makeText(
		QStringLiteral("Locator %1 (d %2, %3, %4°)")
			.arg(patOut.score, 0, 'f', 1).arg(result.deltaX, 0, 'f', 1)
			.arg(result.deltaY, 0, 'f', 1).arg(result.deltaAngle, 0, 'f', 2),
		result.foundPos + QPointF(0, -patOut.h / 2 - 20), Qt::blue, 12));

	ct::logger::info("[AlgoLocator] Found at (%.1f, %.1f) score %.1f delta (%.1f, %.1f, %.2f deg)",
		result.foundPos.x(), result.foundPos.y(), result.score,
		result.deltaX, result.deltaY, result.deltaAngle);
	return result;
}

//apply the locator delta to an ROI: translate by the delta, then rotate the ROI
//centre about the found point (size-preserving, QAlgo::getLocatorOffsetedQRectF style)
static QRectF applyLocatorToRoi(const QRectF& roi, const AlgoLocatorResult& loc)
{
	if (!loc.found) return roi;

	const double rad = qDegreesToRadians(-loc.deltaAngle);
	const double s = std::sin(rad), c = std::cos(rad);

	const QPointF translated = roi.center() + QPointF(loc.deltaX, loc.deltaY);
	const QPointF rel = translated - loc.foundPos;
	const QPointF rotated(rel.x() * c - rel.y() * s, rel.x() * s + rel.y() * c);

	QRectF out = roi;
	out.moveCenter(rotated + loc.foundPos);
	return out;
}

// =============================================================================
// OCR inspection (port of IM430 ocrInspection2 core)
// =============================================================================

void AlgoManager::runOcr(const QImage& fov)
{
	QMetaObject::invokeMethod(this, "doRunOcr", Qt::QueuedConnection, Q_ARG(QImage, fov));
}

QPointF AlgoManager::ocrToFov(const QPointF& ocrPt, const OcrRoiTransform& t) const
{
	//inverse transform: OCR image coords -> FOV coords
	//(reverses: ROI crop -> cv::rotate -> enlarge -> canvas pad)
	double px = ocrPt.x() - t.canvasPad.x();
	double py = ocrPt.y() - t.canvasPad.y();

	if (t.scale > 0.0) {
		px /= t.scale;
		py /= t.scale;
	}

	const double W = t.roiGeo.width();
	const double H = t.roiGeo.height();
	double qx = px, qy = py;
	switch (t.rotation) {
	case 90:  //forward cv::ROTATE_90_CLOCKWISE : (x, y) -> (H - y, x)
		qx = py;
		qy = H - px;
		break;
	case 180:
		qx = W - px;
		qy = H - py;
		break;
	case 270: //forward cv::ROTATE_90_COUNTERCLOCKWISE : (x, y) -> (y, W - x)
		qx = W - py;
		qy = px;
		break;
	default:
		break;
	}

	return QPointF(t.roiGeo.x() + qx, t.roiGeo.y() + qy);
}

QVector<AlgoOcrBox> AlgoManager::runOcrOnRoi(const cv::Mat& fovBgr, const QRectF& roiGeo, int rows, int cols,
	const AlgoOcrParams& param, OcrRoiTransform& transform, QVector<AlgoOverlayItem>& overlay)
{
	QVector<AlgoOcrBox> results;

	QRect g = roiGeo.toRect() & QRect(0, 0, fovBgr.cols, fovBgr.rows);
	if (g.width() < 4 || g.height() < 4) return results;

	overlay.append(AlgoOverlayItem::makeRect(QRectF(g), QColor(0, 150, 255), QColor(0, 150, 255, 60)));

	cv::Mat cvImg = fovBgr(cv::Rect(g.x(), g.y(), g.width(), g.height())).clone();

	transform.roiGeo = QRectF(g);
	transform.scale = 1.0;
	transform.canvasPad = QPoint(0, 0);
	transform.rotation = 0;

	const int angle = param.orientation;

	if (!param.paddleOcrEnabled) {
		//── PaddleOCR-disabled path: no rotation; orientation controls reading order only.
		//  0:   rows = horizontal slices top-to-bottom,  cols left-to-right
		//  90:  rows = vertical slices right-to-left,    cols top-to-bottom
		//  180: rows = horizontal slices bottom-to-top,  cols right-to-left
		//  270: rows = vertical slices left-to-right,    cols bottom-to-top
		const int imgW = cvImg.cols;
		const int imgH = cvImg.rows;
		const int roiRows = std::max(1, rows);
		const int roiCols = std::max(1, cols);

		if (angle == 90 || angle == 270) {
			const int sliceW = std::max(1, imgW / roiRows);
			for (int ri = 0; ri < roiRows; ++ri) {
				int sliceIdx = (angle == 90) ? (roiRows - 1 - ri) : ri;
				int x0 = sliceIdx * sliceW;
				int x1 = (sliceIdx == roiRows - 1) ? imgW : (sliceIdx + 1) * sliceW;

				AlgoOcrBox r;
				r.text = QString(roiCols, QChar(' '));
				r.box = { QPoint(x0, 0), QPoint(x1, 0), QPoint(x1, imgH), QPoint(x0, imgH) };
				results.push_back(r);
			}
		}
		else {
			const int rowH = std::max(1, imgH / roiRows);
			for (int ri = 0; ri < roiRows; ++ri) {
				int rowIdx = (angle == 180) ? (roiRows - 1 - ri) : ri;
				int y0 = rowIdx * rowH;
				int y1 = (rowIdx == roiRows - 1) ? imgH : (rowIdx + 1) * rowH;

				AlgoOcrBox r;
				r.text = QString(roiCols, QChar(' '));
				r.box = { QPoint(0, y0), QPoint(imgW, y0), QPoint(imgW, y1), QPoint(0, y1) };
				results.push_back(r);
			}
		}

		ct::logger::info("[Algo OCR] PaddleOCR disabled - %d rows x %d cols, orientation=%d",
			roiRows, roiCols, angle);
		return results;
	}

	//── PaddleOCR-enabled path: rotate before OCR, enlarge small crops, pad to min canvas
	if (angle == 90)       cv::rotate(cvImg, cvImg, cv::ROTATE_90_CLOCKWISE);
	else if (angle == 180) cv::rotate(cvImg, cvImg, cv::ROTATE_180);
	else if (angle == 270) cv::rotate(cvImg, cvImg, cv::ROTATE_90_COUNTERCLOCKWISE);
	transform.rotation = angle;

	const int ocrMinWidth = 1200;
	const int ocrMinHeight = 1200;

	if (cvImg.cols < 200 || cvImg.rows < 200) {
		transform.scale = param.enlargeOcrImage > 0 ? param.enlargeOcrImage : 4.0;
		cv::resize(cvImg, cvImg, cv::Size(), transform.scale, transform.scale, cv::INTER_CUBIC);
	}

	if (cvImg.cols < ocrMinWidth || cvImg.rows < ocrMinHeight) {
		int canvasW = std::max(cvImg.cols, ocrMinWidth);
		int canvasH = std::max(cvImg.rows, ocrMinHeight);
		cv::Mat canvas(canvasH, canvasW, cvImg.type(), cv::Scalar(0, 0, 0));
		int offsetX = (canvasW - cvImg.cols) / 2;
		int offsetY = (canvasH - cvImg.rows) / 2;
		transform.canvasPad = QPoint(offsetX, offsetY);
		cvImg.copyTo(canvas(cv::Rect(offsetX, offsetY, cvImg.cols, cvImg.rows)));
		cvImg = canvas;
	}

	if (!m_paddle) {
		m_paddle = new PaddleOcrClient(this);
	}

	QElapsedTimer paddleTimer;
	paddleTimer.start();
	if (!m_paddle->runOcr(cvImg, results)) {
		ct::logger::error("[Algo OCR] PaddleOCR transport failure");
	}
	ct::logger::info("[Algo OCR] PaddleOCR returned %d row(s) in %lldms", results.size(), paddleTimer.elapsed());

	return results;
}

void AlgoManager::applyPatternMatching(const cv::Mat& fovGray, QVector<AlgoOcrBox>& results,
	int startIdx, int endIdx, int columnsOverride, const AlgoOcrParams& param,
	const OcrRoiTransform& transform, QVector<AlgoOverlayItem>& overlay)
{
	OcrPatternConfig patternCfg;
	QHash<QString, QStringList> patternFiles;
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		patternCfg = m_patternConfig;
		patternFiles = m_patternFiles;
	}

	//when PaddleOCR is disabled the whole result comes from pattern matching,
	//so force this pass on regardless of the enable checkbox
	const bool paddleDisabled = !param.paddleOcrEnabled;
	if (!paddleDisabled && !patternCfg.enabled) return;
	if (patternFiles.isEmpty()) {
		if (paddleDisabled) {
			ct::logger::warn("[OcrPattern] PaddleOCR disabled but no learned patterns available - result will be empty");
		}
		return;
	}

	const double threshold = patternCfg.scoreThreshold;

	for (int ri = startIdx; ri <= endIdx && ri < results.size(); ++ri) {
		auto& result = results[ri];
		const int charCount = result.text.length();
		if (charCount == 0) continue;

		QRect ocrBox = result.boundingRect();
		if (ocrBox.width() < 2 || ocrBox.height() < 2) continue;

		//divide the row into per-character slots.
		//0/180: split horizontally; 90/270: split vertically. Reading order follows orientation.
		int effectiveCols = (columnsOverride > 0) ? columnsOverride : charCount;
		effectiveCols = std::max(1, effectiveCols);

		const int ocrAngle = param.orientation;
		const bool verticalSplit = (ocrAngle == 90 || ocrAngle == 270);
		const bool reverseCols = (ocrAngle == 180 || ocrAngle == 270);
		const int splitDim = verticalSplit ? ocrBox.height() : ocrBox.width();
		const int charSlotSize = std::max(1, splitDim / effectiveCols);

		//corrected string sized to effectiveCols (pad/trim from PaddleOCR text as needed)
		QString corrected = result.text.leftJustified(effectiveCols, ' ').left(effectiveCols);

		for (int ci = 0; ci < effectiveCols; ci++) {
			const int colIdx = reverseCols ? (effectiveCols - 1 - ci) : ci;
			const int offset = colIdx * charSlotSize;
			int slotSize = (colIdx == effectiveCols - 1) ? splitDim - offset : charSlotSize;
			if (offset + slotSize > splitDim) slotSize = splitDim - offset;
			if (slotSize < 1) continue;

			//slot rect in OCR image coordinates
			int slotOcrX, slotOcrY, slotOcrW, slotOcrH;
			if (verticalSplit) {
				slotOcrX = ocrBox.x();
				slotOcrY = ocrBox.y() + offset;
				slotOcrW = ocrBox.width();
				slotOcrH = slotSize;
			}
			else {
				slotOcrX = ocrBox.x() + offset;
				slotOcrY = ocrBox.y();
				slotOcrW = slotSize;
				slotOcrH = ocrBox.height();
			}

			//map the 4 slot corners to FOV space, take the axis-aligned bounding rect
			//(rotations are multiples of 90 so this is exact)
			QPointF c1 = ocrToFov(QPointF(slotOcrX, slotOcrY), transform);
			QPointF c2 = ocrToFov(QPointF(slotOcrX + slotOcrW, slotOcrY), transform);
			QPointF c3 = ocrToFov(QPointF(slotOcrX + slotOcrW, slotOcrY + slotOcrH), transform);
			QPointF c4 = ocrToFov(QPointF(slotOcrX, slotOcrY + slotOcrH), transform);
			const double minX = std::min({ c1.x(), c2.x(), c3.x(), c4.x() });
			const double minY = std::min({ c1.y(), c2.y(), c3.y(), c4.y() });
			const double maxX = std::max({ c1.x(), c2.x(), c3.x(), c4.x() });
			const double maxY = std::max({ c1.y(), c2.y(), c3.y(), c4.y() });

			//expand by search pad and clamp to image bounds
			const int padX = std::max(0, (int)std::floor(minX) - param.patternSearchPadX);
			const int padY = std::max(0, (int)std::floor(minY) - param.patternSearchPadY);
			const int padR = std::min(fovGray.cols, (int)std::ceil(maxX) + param.patternSearchPadX);
			const int padB = std::min(fovGray.rows, (int)std::ceil(maxY) + param.patternSearchPadY);
			const int padW = padR - padX;
			const int padH = padB - padY;
			if (padW < 1 || padH < 1) continue;

			cv::Mat charCrop = fovGray(cv::Rect(padX, padY, padW, padH)).clone();

			//binarize and crop to the blob bounding box for tighter pattern matching
			MIL_ID mSearch = M_NULL;
			{
				MIL_ID mCharFull = M_NULL;
				util::cv_to_Mil(charCrop, mCharFull);
				if (mCharFull == M_NULL) continue;

				MIL_ID mBin = mtrx::alloc_buffer(mCharFull);
				MimBinarize(mCharFull, mBin, M_BIMODAL + M_GREATER, M_NULL, M_NULL);

				auto bCtx = MblobAlloc(M_DEFAULT_HOST, M_DEFAULT, M_DEFAULT, M_NULL);
				auto bRes = MblobAllocResult(M_DEFAULT_HOST, M_NULL);
				MblobControl(bCtx, M_BOX, M_ENABLE);
				MblobCalculate(bCtx, mBin, M_NULL, bRes);

				MIL_INT nBlobs = 0;
				MblobGetResult(bRes, M_DEFAULT, M_NUMBER + M_TYPE_MIL_INT, &nBlobs);

				if (nBlobs > 0) {
					MIL_INT bx = 0, by = 0, bx2 = 0, by2 = 0;
					MIL_INT bxMin = 999999, byMin = 999999, bxMax = 0, byMax = 0;
					for (MIL_INT bi = 0; bi < nBlobs; ++bi) {
						MblobGetResult(bRes, M_BLOB_INDEX(bi), M_BOX_X_MIN + M_TYPE_MIL_INT, &bx);
						MblobGetResult(bRes, M_BLOB_INDEX(bi), M_BOX_Y_MIN + M_TYPE_MIL_INT, &by);
						MblobGetResult(bRes, M_BLOB_INDEX(bi), M_BOX_X_MAX + M_TYPE_MIL_INT, &bx2);
						MblobGetResult(bRes, M_BLOB_INDEX(bi), M_BOX_Y_MAX + M_TYPE_MIL_INT, &by2);
						if (bx < bxMin) bxMin = bx;
						if (by < byMin) byMin = by;
						if (bx2 > bxMax) bxMax = bx2;
						if (by2 > byMax) byMax = by2;
					}

					const MIL_INT blobPad = 5;
					const MIL_INT imgW = MbufInquire(mCharFull, M_SIZE_X, M_NULL);
					const MIL_INT imgH = MbufInquire(mCharFull, M_SIZE_Y, M_NULL);
					bxMin = std::max((MIL_INT)0, bxMin - blobPad);
					byMin = std::max((MIL_INT)0, byMin - blobPad);
					bxMax = std::min(imgW - 1, bxMax + blobPad);
					byMax = std::min(imgH - 1, byMax + blobPad);
					const MIL_INT bw = bxMax - bxMin + 1;
					const MIL_INT bh = byMax - byMin + 1;

					//only trust the blob crop when it covers a plausible character area
					if (bw > imgW / 2 && bh > imgH / 2) {
						MIL_ID mChild = MbufChild2d(mCharFull, bxMin, byMin, bw, bh, M_NULL);
						const MIL_INT bands = MbufInquire(mChild, M_SIZE_BAND, M_NULL);
						const MIL_INT type = MbufInquire(mChild, M_TYPE, M_NULL);
						mSearch = MbufAllocColor(M_DEFAULT_HOST, bands, bw, bh, type, M_IMAGE + M_PROC, M_NULL);
						MbufCopy(mChild, mSearch);
						MbufFree(mChild);
					}
				}

				MblobFree(bRes);
				MblobFree(bCtx);
				mtrx::free_buffer(mBin);
				mtrx::free_buffer(mCharFull);

				if (mSearch == M_NULL) {
					util::cv_to_Mil(charCrop, mSearch); //fallback: full padded slot
				}
			}
			if (mSearch == M_NULL) continue;

			//unsharp mask before pattern matching
			{
				cv::Mat srcMat, blurred;
				util::Mil_to_cv(mSearch, srcMat);
				cv::GaussianBlur(srcMat, blurred, cv::Size(0, 0), 2);
				cv::addWeighted(srcMat, 2.0, blurred, -1.0, 0, srcMat);
				MbufPut(mSearch, srcMat.data);
			}

			QString bestLabel;
			double bestScore = 0;

			for (auto it = patternFiles.begin(); it != patternFiles.end(); ++it) {
				bool labelEnabled = true;
				for (const auto& lbl : patternCfg.labels) {
					if (lbl.label == it.key()) { labelEnabled = lbl.enabled; break; }
				}
				if (!labelEnabled) continue;

				for (const QString& mpatPath : it.value()) {
					mtrx::PatternOutput patOut;
					patOut.acceptance_min_score = 1.0; //must be > 0 so find_pattern applies MpatControl
					if (mtrx::find_pattern(mSearch, mpatPath.toStdString(), patOut)) {
						if (patOut.score > bestScore) {
							bestScore = patOut.score;
							bestLabel = it.key();
						}
					}
				}
			}

			MbufFree(mSearch);

			ct::logger::info("[OcrPattern] row %d col %d/%d: best='%s' score=%.1f threshold=%.1f -> %s",
				ri, ci + 1, effectiveCols, bestLabel.toStdString().c_str(),
				bestScore, threshold,
				(bestScore >= threshold && !bestLabel.isEmpty()) ? "MATCHED" : "no match");

			if (bestScore >= threshold && !bestLabel.isEmpty()) {
				corrected[ci] = bestLabel[0];

				const QRectF fovRect(QPointF(padX, padY), QPointF(padX + padW, padY + padH));
				static const QColor boxColor(255, 165, 0);
				overlay.append(AlgoOverlayItem::makeRect(fovRect, boxColor, QColor(255, 165, 0, 40)));
				overlay.append(AlgoOverlayItem::makeText(bestLabel, fovRect.topLeft() - QPointF(0, 16), boxColor, 11));
			}
		}

		if (corrected != result.text) {
			ct::logger::info("[OcrPattern] Row %d: corrected '%s' -> '%s'", ri,
				result.text.toStdString().c_str(), corrected.toStdString().c_str());
			result.text = corrected;
		}
	}
}

void AlgoManager::doRunOcr(QImage fov)
{
	m_busy = true;
	emit busyChanged(true);

	QElapsedTimer timer;
	timer.start();

	AlgoOcrOutput out;
	AlgoOcrParams param = ocrParams();

	do {
		if (fov.isNull()) { out.message = "No image loaded"; break; }

		//no ROI configured by the user: default to the centered 50% of the image
		//(with PaddleOCR, which is on by default)
		if (param.roi1Geo.isEmpty()) {
			param.roi1Geo = QRectF(fov.width() * 0.25, fov.height() * 0.25,
				fov.width() * 0.5, fov.height() * 0.5);
			ct::logger::info("[AlgoOCR] ROI 1 not set - using default centered 50%% ROI (%.0fx%.0f)",
				param.roi1Geo.width(), param.roi1Geo.height());
		}

		cv::Mat fovBgr = qimageToBgr(fov);
		cv::Mat fovGray;
		cv::cvtColor(fovBgr, fovGray, cv::COLOR_BGR2GRAY);

		//── locator first; when not assigned (or failed) the algo runs on the ROIs as-is
		const AlgoLocatorConfig locCfg = locatorConfig(AlgoPageAlgo::OCR_READ);
		AlgoLocatorResult loc = runLocator(fovBgr, locCfg, out.overlay);
		if (loc.ran && !loc.found) out.message = "Locator: no match - ran unshifted. ";

		const QRectF roi1 = applyLocatorToRoi(param.roi1Geo, loc);
		const QRectF roi2 = applyLocatorToRoi(param.roi2Geo, loc);

		//── ROI1
		OcrRoiTransform t1;
		auto results1 = runOcrOnRoi(fovBgr, roi1, param.roi1Rows, param.roi1Columns, param, t1, out.overlay);
		applyPatternMatching(fovGray, results1, 0, std::max(1, param.roi1Rows) - 1, param.roi1Columns, param, t1, out.overlay);

		QStringList roi1Rows, roi1Tokens;
		for (const auto& r : results1) {
			const QString text = r.text;
			if (text.trimmed().isEmpty()) continue;
			if ((int)roi1Tokens.size() >= std::max(1, param.roi1Rows)) break;

			roi1Rows << text;
			const QStringList parts = text.split(' ', QString::SkipEmptyParts);
			if (!parts.isEmpty()) roi1Tokens << parts.first();
		}
		out.roi1Text = roi1Rows.join(",");
		out.roi1Key = roi1Tokens.join("");
		if (param.removeSpecialChars) out.roi1Key.remove(QRegExp("[^a-zA-Z0-9]"));

		for (const auto& r : results1) {
			if (r.box.size() < 4) continue;
			QPolygonF poly;
			for (const auto& pt : r.box) poly << ocrToFov(pt, t1);
			out.overlay.append(AlgoOverlayItem::makePoly(poly, Qt::yellow));
		}
		out.overlay.append(AlgoOverlayItem::makeText(out.roi1Text, t1.roiGeo.topLeft() - QPointF(0, 40), Qt::green, 24));

		//── ROI2 (optional)
		if (param.roi2Enabled && !param.roi2Geo.isEmpty()) {
			OcrRoiTransform t2;
			auto results2 = runOcrOnRoi(fovBgr, roi2, param.roi2Rows, param.roi2Columns, param, t2, out.overlay);
			const int total = results2.size();
			const int startIdx = std::max(0, total - std::max(1, param.roi2Rows));
			applyPatternMatching(fovGray, results2, startIdx, total - 1, param.roi2Columns, param, t2, out.overlay);

			QStringList roi2Rows, roi2Tokens;
			for (const auto& r : results2) {
				const QString text = r.text;
				if (text.trimmed().isEmpty()) continue;
				roi2Rows << text;
			}
			const int tokenStart = std::max(0, roi2Rows.size() - std::max(1, param.roi2Rows));
			for (int i = tokenStart; i < roi2Rows.size(); i++) {
				const QStringList parts = roi2Rows[i].split(' ', QString::SkipEmptyParts);
				if (!parts.isEmpty()) roi2Tokens << parts.last();
			}
			out.roi2Text = roi2Rows.join(",");
			out.roi2Key = roi2Tokens.join("");
			if (param.removeSpecialChars) out.roi2Key.remove(QRegExp("[^a-zA-Z0-9]"));

			for (const auto& r : results2) {
				if (r.box.size() < 4) continue;
				QPolygonF poly;
				for (const auto& pt : r.box) poly << ocrToFov(pt, t2);
				out.overlay.append(AlgoOverlayItem::makePoly(poly, Qt::cyan));
			}
			out.overlay.append(AlgoOverlayItem::makeText(out.roi2Text, t2.roiGeo.topLeft() - QPointF(0, 40), Qt::green, 24));
		}

		//recognizing nothing is a fail, not a pass (e.g. Paddle server missing
		//or nothing readable in the ROI)
		out.ok = !out.roi1Text.trimmed().isEmpty();
		if (!out.ok && out.message.isEmpty()) out.message = "No text recognized";
	} while (false);

	out.elapsedMs = timer.elapsed();
	ct::logger::info("[Algo OCR] Done in %lldms: roi1='%s' roi2='%s' %s",
		out.elapsedMs, out.roi1Key.toStdString().c_str(), out.roi2Key.toStdString().c_str(),
		out.message.toStdString().c_str());

	m_busy = false;
	emit busyChanged(false);
	emit ocrFinished(out);
}

// =============================================================================
// 3D height measurement (QAlgoHeightMeasurement plane-fit core)
// =============================================================================

void AlgoManager::setHeightMap(mtrx::SharedMilID heightMap)
{
	std::lock_guard<std::mutex> lock(m_mutex);
	m_heightMap = heightMap;
}

mtrx::SharedMilID AlgoManager::heightMap() const
{
	std::lock_guard<std::mutex> lock(m_mutex);
	return m_heightMap;
}

bool AlgoManager::loadHeightMapFromFile(const QString& tiffPath, QString& error)
{
	if (!QFile::exists(tiffPath)) { error = "File not found: " + tiffPath; return false; }

	MIL_ID mBuf = MbufRestoreA(tiffPath.toStdString().c_str(), M_DEFAULT_HOST, M_NULL);
	if (mBuf == M_NULL) { error = "Failed to load heightmap (16-bit tiff expected)."; return false; }

	{
		std::lock_guard<std::mutex> lock(m_mutex);
		m_heightMap = mtrx::MPM::instance().attach(mBuf);
	}

	ct::logger::info("[Algo Height] Loaded heightmap %s (%lld x %lld)",
		tiffPath.toStdString().c_str(), mtrx::get_width(mBuf), mtrx::get_height(mBuf));
	return true;
}

QImage AlgoManager::heightMapImage(bool colorMapped) const
{
	mtrx::SharedMilID map = heightMap();
	if (!map || map->id() == M_NULL) return QImage();

	cv::Mat raw;
	util::Mil_to_cv(map->id(), raw);
	if (raw.empty()) return QImage();

	//normalize the valid (non-zero) range to 8 bit
	double minV = 0, maxV = 0;
	cv::Mat validMask = raw > 0;
	cv::minMaxLoc(raw, &minV, &maxV, nullptr, nullptr, validMask);
	if (maxV <= minV) { minV = 0; maxV = 65535; }

	cv::Mat gray8;
	raw.convertTo(gray8, CV_8U, 255.0 / (maxV - minV), -minV * 255.0 / (maxV - minV));
	gray8.setTo(0, ~validMask);

	cv::Mat display;
	if (colorMapped) {
		cv::applyColorMap(gray8, display, cv::COLORMAP_JET);
		display.setTo(cv::Scalar(20, 20, 20), ~validMask); //invalid px: near-black
		cv::cvtColor(display, display, cv::COLOR_BGR2RGB);
		QImage img((const uchar*)display.data, display.cols, display.rows, (int)display.step, QImage::Format_RGB888);
		return img.copy();
	}

	QImage img((const uchar*)gray8.data, gray8.cols, gray8.rows, (int)gray8.step, QImage::Format_Grayscale8);
	return img.copy();
}

void AlgoManager::runHeight()
{
	QMetaObject::invokeMethod(this, "doRunHeight", Qt::QueuedConnection);
}

void AlgoManager::doRunHeight()
{
	m_busy = true;
	emit busyChanged(true);

	QElapsedTimer timer;
	timer.start();

	AlgoHeightOutput out;
	const AlgoHeightParams param = heightParams();
	mtrx::SharedMilID map = heightMap();

	do {
		if (!map || map->id() == M_NULL) { out.message = "No heightmap loaded"; break; }
		if (param.planeRois.size() < 3) { out.message = "Add at least 3 plane ROIs for the datum fit"; break; }
		if (param.heightRois.isEmpty()) { out.message = "Add at least 1 height measurement ROI"; break; }
		if (param.intensityPerMicron <= 0.0) { out.message = "Intensity per micron must be > 0"; break; }

		MIL_UINT16* hostPtr = nullptr;
		MIL_INT pitch = 0;
		MbufInquire(map->id(), M_HOST_ADDRESS, &hostPtr);
		MbufInquire(map->id(), M_PITCH, &pitch);
		const MIL_INT mapW = mtrx::get_width(map->id());
		const MIL_INT mapH = mtrx::get_height(map->id());

		if (!hostPtr) { out.message = "Heightmap buffer has no host address"; break; }

		//── locator first (on the 8-bit heightmap view); ROIs run as-is when not assigned/found
		QVector<QRectF> planeRois = param.planeRois;
		QVector<QRectF> heightRois = param.heightRois;

		const AlgoLocatorConfig locCfg = locatorConfig(AlgoPageAlgo::HEIGHT_3D);
		if (locCfg.enabled) {
			const QImage gray = heightMapImage(false);
			if (!gray.isNull()) {
				cv::Mat grayBgr = qimageToBgr(gray);
				AlgoLocatorResult loc = runLocator(grayBgr, locCfg, out.overlay);
				if (loc.ran && !loc.found) out.message = "Locator: no match - ran unshifted. ";

				for (auto& r : planeRois) r = applyLocatorToRoi(r, loc);
				for (auto& r : heightRois) r = applyLocatorToRoi(r, loc);
			}
		}

		//── datum: collect every valid (z > 0) pixel from every plane ROI, absolute coords
		std::vector<P3> points;
		{
			size_t reserveSize = 0;
			for (const auto& r : planeRois) reserveSize += (size_t)(r.width() * r.height());
			points.reserve(reserveSize);
		}

		for (const auto& roiF : planeRois) {
			QRect r = roiF.toRect() & QRect(0, 0, (int)mapW, (int)mapH);
			for (int y = r.top(); y <= r.bottom(); y++) {
				const MIL_UINT16* row = hostPtr + (y * pitch);
				for (int x = r.left(); x <= r.right(); x++) {
					const int z = row[x];
					if (z <= 0) continue;
					points.push_back({ (double)x, (double)y, (double)z });
				}
			}
			out.overlay.append(AlgoOverlayItem::makeRect(roiF, QColor(0, 255, 127), QColor(0, 255, 127, 30)));
		}

		if (points.size() < 3) { out.message = "Not enough valid pixels in the plane ROIs"; break; }

		if (param.removeOutliers) removeOutliersByPlaneResidual(points, 2.0, 3);

		const Plane plane = computeLeastSquaredPlane(points);
		if (!plane.valid) { out.message = "Plane fit failed (degenerate points)"; break; }

		out.planeValid = true;
		out.planeA = plane.a; out.planeB = plane.b; out.planeC = plane.c; out.planeD = plane.d;
		planeTiltXY(plane, out.tiltXDeg, out.tiltYDeg);

		//── measure every ROI: per-pixel vertical distance to the datum plane, averaged (+min/max)
		const bool limitActive = (param.minHeightUm != 0.0 || param.maxHeightUm != 0.0);
		out.pass = true;

		for (int ri = 0; ri < heightRois.size(); ri++) {
			const QRectF roiF = heightRois[ri];
			AlgoHeightRoiResult res;
			res.roi = roiF;

			QRect hr = roiF.toRect() & QRect(0, 0, (int)mapW, (int)mapH);

			double sum = 0.0;
			double minDist = 1e18, maxDist = -1e18;
			qint64 count = 0;

			for (int y = hr.top(); y <= hr.bottom(); y++) {
				const MIL_UINT16* row = hostPtr + (y * pitch);
				for (int x = hr.left(); x <= hr.right(); x++) {
					const int z = row[x];
					if (z <= 0) continue;

					const double dist = (double)z - getZFromPlane(plane, x, y); //vertical, raw gray
					sum += dist;
					if (dist < minDist) minDist = dist;
					if (dist > maxDist) maxDist = dist;
					count++;
				}
			}

			if (count > 0) {
				//divide by scale at the very end (gray levels / (gray per um) = um)
				res.valid = true;
				res.avgHeightUm = (sum / count) / param.intensityPerMicron;
				res.minHeightUm = minDist / param.intensityPerMicron;
				res.maxHeightUm = maxDist / param.intensityPerMicron;
				res.pass = !limitActive ||
					(res.avgHeightUm >= param.minHeightUm && res.avgHeightUm <= param.maxHeightUm);
			}
			else {
				res.pass = false;
			}

			out.pass = out.pass && res.pass;

			out.overlay.append(AlgoOverlayItem::makeRect(roiF,
				res.pass ? QColor(0, 255, 127) : Qt::red,
				res.pass ? QColor(0, 255, 127, 30) : QColor(255, 0, 0, 30)));
			out.overlay.append(AlgoOverlayItem::makeText(
				res.valid
					? QStringLiteral("H%1: %2 um [%3 .. %4]").arg(ri + 1)
						.arg(res.avgHeightUm, 0, 'f', 1).arg(res.minHeightUm, 0, 'f', 1).arg(res.maxHeightUm, 0, 'f', 1)
					: QStringLiteral("H%1: no valid pixels").arg(ri + 1),
				roiF.topLeft() - QPointF(0, 24),
				res.pass ? QColor(0, 255, 127) : Qt::red, 14));

			out.roiResults.append(res);
		}

		out.ok = true;
	} while (false);

	out.elapsedMs = timer.elapsed();
	ct::logger::info("[Algo Height] Done in %lldms: %d ROI(s) tilt=(%.3f, %.3f) pass=%d %s",
		out.elapsedMs, out.roiResults.size(),
		out.tiltXDeg, out.tiltYDeg, out.pass ? 1 : 0, out.message.toStdString().c_str());

	m_busy = false;
	emit busyChanged(false);
	emit heightFinished(out);
}
