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

namespace {

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
	qRegisterMetaType<AlgoHeight3Output>("AlgoHeight3Output");

	moveToThread(&m_thread);
	m_thread.start();

	ct::logger::info("[Algo] Manager worker thread started");

	/*
	* V3 gets a thread of its own. Everything queued to m_thread runs in turn, and the first
	* job there is the PaddleOCR warm-up below, which can block for up to two minutes - every
	* V3 stage, the page's reopen re-run and production's per-unit 3D run used to wait behind
	* it, and production gives a 3D result 60 s. Nothing OCR does can delay V3 any more. V3's
	* data is behind its own m_height3Mutex and its results go out by signal, so its callers
	* see no difference. The runner has no parent: a child must live in its parent's thread.
	*/
	m_height3Runner = new AlgoH3Runner(this);
	m_height3Runner->moveToThread(&m_height3Thread);
	m_height3Thread.start();
	ct::logger::info("[Algo] V3 worker thread started");

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
	if (m_height3Thread.isRunning()) {
		m_height3Thread.quit();
		m_height3Thread.wait(3000);
	}
	//the thread has stopped, so the runner can be deleted from here
	delete m_height3Runner;
	m_height3Runner = nullptr;

	if (!m_thread.isRunning()) return;
	m_thread.quit();
	m_thread.wait(3000);
}

void AlgoManager::setBusy(bool on)
{
	if (on) {
		if (++m_busyCount == 1) emit busyChanged(true);
	}
	else {
		if (--m_busyCount == 0) emit busyChanged(false);
	}
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

/*
* Bounds-checked because AlgoPageAlgo is larger than m_locator[]: HEIGHT_3D_V3 has no Locator
* of its own. Without the check, selecting that page and pressing Save or Run would write past
* the end of the array - silent corruption of whatever follows it, not a crash.
*/
AlgoLocatorConfig AlgoManager::locatorConfig(AlgoPageAlgo algo) const
{
	if (!algoHasLocator(algo)) return AlgoLocatorConfig();

	std::lock_guard<std::mutex> lock(m_mutex);
	return m_locator[(int)algo];
}

void AlgoManager::setLocatorConfig(AlgoPageAlgo algo, const AlgoLocatorConfig& cfg)
{
	if (!algoHasLocator(algo)) return;

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
		for (auto& cfg : m_locator) cfg = AlgoLocatorConfig();

		if (exists) {
			auto ocr = root.value("ocr").toObject();
			m_ocrParams.orientation = jsonHelper::getInteger(ocr, "orientation", 0);
			m_ocrParams.roi1Rows = jsonHelper::getInteger(ocr, "roi1_rows", 1);
			m_ocrParams.roi1Columns = jsonHelper::getInteger(ocr, "roi1_columns", 0);
			m_ocrParams.removeSpecialChars = jsonHelper::getBool(ocr, "remove_special_chars", false);
			m_ocrParams.paddleOcrEnabled = jsonHelper::getBool(ocr, "paddle_enabled", true);
			m_ocrParams.superResolution = jsonHelper::getBool(ocr, "super_resolution", false);
			m_ocrParams.roi1Geo = jsonToRect(ocr.value("roi1").toObject());

			/*
			* Only the first entry is read: the second belonged to the old plane-fit height algo
			* (V1), which is gone. Its "height" block and that entry stay in the file until the
			* next save, which no longer writes them.
			*/
			auto locators = root.value("locators").toArray();
			for (int i = 0; i < locators.size() && i < kAlgoLocatorSlots; i++) {
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

	/*
	* 3D Height Measurement 3 is not per recipe: it is the machine-wide library of named algos,
	* loaded once, the first time a recipe opens (its first launch imports every recipe's old
	* "height3" block). The block is still kept and written back exactly as it was, so an older
	* build reopening this recipe finds the settings it measured with - nothing here reads it.
	*/
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		m_recipeHeight3Block = root.value("height3").toObject();
	}
	if (!m_height3LibraryLoaded) loadHeight3Library();

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
		ocr.insert("roi1_rows", m_ocrParams.roi1Rows);
		ocr.insert("roi1_columns", m_ocrParams.roi1Columns);
		ocr.insert("remove_special_chars", m_ocrParams.removeSpecialChars);
		ocr.insert("paddle_enabled", m_ocrParams.paddleOcrEnabled);
		ocr.insert("super_resolution", m_ocrParams.superResolution);
		ocr.insert("roi1", rectToJson(m_ocrParams.roi1Geo));
		root.insert("ocr", ocr);

		QJsonArray locators;
		for (int i = 0; i < kAlgoLocatorSlots; i++) {
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

		if (!m_recipeHeight3Block.isEmpty()) root.insert("height3", m_recipeHeight3Block);
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
	//(reverses: ROI crop -> cv::rotate -> canvas pad)
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
	const AlgoOcrParams& param, OcrRoiTransform& transform, QVector<AlgoOverlayItem>& overlay,
	QImage* srImage)
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

	//── PaddleOCR-enabled path: rotate before OCR. Super-resolution of small crops
	//and min-canvas padding both happen in the Paddle server (pyPaddleAPI.py), which
	//returns boxes in the coordinates of the image we send - so no pad/scale here.
	if (angle == 90)       cv::rotate(cvImg, cvImg, cv::ROTATE_90_CLOCKWISE);
	else if (angle == 180) cv::rotate(cvImg, cvImg, cv::ROTATE_180);
	else if (angle == 270) cv::rotate(cvImg, cvImg, cv::ROTATE_90_COUNTERCLOCKWISE);
	transform.rotation = angle;

	if (!m_paddle) {
		m_paddle = new PaddleOcrClient(this);
	}

	QElapsedTimer paddleTimer;
	paddleTimer.start();
	if (!m_paddle->runOcr(cvImg, results, 30000, param.superResolution, srImage)) {
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
			const int padX = std::max(0, (int)std::floor(minX));
			const int padY = std::max(0, (int)std::floor(minY));
			const int padR = std::min(fovGray.cols, (int)std::ceil(maxX));
			const int padB = std::min(fovGray.rows, (int)std::ceil(maxY));
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
	setBusy(true);

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

		//── ROI1
		OcrRoiTransform t1;
		QImage sr1;
		auto results1 = runOcrOnRoi(fovBgr, roi1, param.roi1Rows, param.roi1Columns, param, t1, out.overlay, &sr1);

		//super-resolution preview: undo the pre-OCR rotation and size the enhanced
		//crop back to its ROI so the UI can paint it over the FOV in place
		if (param.superResolution && sr1.isNull()) {
			ct::logger::warn("[Algo OCR] Super Resolution is ON but the server did not run it - "
				"check the Paddle server startup lines: torch installed in the venv, weights file "
				"models/realesr-general-x4v3.pth present. Crops with a side >= 500 px are never super-resolved.");
		}
		if (!sr1.isNull()) {
			QImage img = sr1;
			if (t1.rotation != 0) {
				QTransform tr;
				tr.rotate(-t1.rotation);
				img = img.transformed(tr);
			}
			out.srImage = img.scaled(t1.roiGeo.size().toSize(), Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
			out.srRect = t1.roiGeo;
		}
		applyPatternMatching(fovGray, results1, 0, std::max(1, param.roi1Rows) - 1, param.roi1Columns, param, t1, out.overlay);

		//every detected line goes into the displayed text (the overlay draws them all,
		//so the table must agree); the Rows setting only caps the KEY tokens
		QStringList roi1Rows, roi1Tokens;
		for (const auto& r : results1) {
			const QString text = r.text;
			if (text.trimmed().isEmpty()) continue;

			roi1Rows << text;
			if ((int)roi1Tokens.size() < std::max(1, param.roi1Rows)) {
				const QStringList parts = text.split(' ', QString::SkipEmptyParts);
				if (!parts.isEmpty()) roi1Tokens << parts.first();
			}
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


		//recognizing nothing is a fail, not a pass (e.g. Paddle server missing
		//or nothing readable in the ROI)
		out.ok = !out.roi1Text.trimmed().isEmpty();
		if (!out.ok && out.message.isEmpty()) out.message = "No text recognized";
	} while (false);

	out.elapsedMs = timer.elapsed();
	ct::logger::info("[Algo OCR] Done in %lldms: roi1='%s' %s",
		out.elapsedMs, out.roi1Key.toStdString().c_str(),
		out.message.toStdString().c_str());

	setBusy(false);
	emit ocrFinished(out);
}

// =============================================================================
// The last scan's maps (production display, FOV readout, V3 "Use Last Scan")
// =============================================================================

mtrx::SharedMilID AlgoManager::heightMap() const
{
	std::lock_guard<std::mutex> lock(m_mutex);
	return m_heightMap;
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

//one pixel of a single-band 8 or 16 bit MIL buffer, read in place - no copy of the map
static bool milPixel(const mtrx::SharedMilID& buf, int x, int y, int& value)
{
	if (!buf || buf->id() == M_NULL) return false;
	const MIL_ID id = buf->id();
	if (x < 0 || y < 0 || x >= (int)mtrx::get_width(id) || y >= (int)mtrx::get_height(id)) return false;
	if (MbufInquire(id, M_SIZE_BAND, M_NULL) != 1) return false;

	switch (MbufInquire(id, M_SIZE_BIT, M_NULL)) {
	case 8:  { MIL_UINT8 v = 0;  MbufGet2d(id, x, y, 1, 1, &v); value = v; return true; }
	case 16: { MIL_UINT16 v = 0; MbufGet2d(id, x, y, 1, 1, &v); value = v; return true; }
	default: return false;
	}
}

bool AlgoManager::lastScanPixel(int x, int y, FovMapPixel& out) const
{
	out = FovMapPixel();

	mtrx::SharedMilID h, i;
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		h = m_heightMap;
		i = m_lastScanIntensity;
	}
	if (!milPixel(h, x, y, out.raw)) return false;

	//a frame may carry no intensity map, or one of another size - use it only when it lines up
	if (i && i->id() != M_NULL
		&& mtrx::get_width(i->id()) == mtrx::get_width(h->id())
		&& mtrx::get_height(i->id()) == mtrx::get_height(h->id()))
		out.hasIntensity = milPixel(i, x, y, out.intensity);
	return true;
}
