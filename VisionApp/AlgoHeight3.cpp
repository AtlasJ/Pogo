// =============================================================================
//  AlgoHeight3.cpp
//  The 3D Height Measurement 3 pipeline: preprocess -> segment -> datum plane ->
//  measure -> overall result, plus the display renders the page needs (2D height,
//  2D intensity, and the software 3D surface view).
//
//  Pure processing. No widgets, no VisionApp, nothing that has to run on the GUI
//  thread - AlgoManager drives all of this from its worker thread.
// =============================================================================

#include "AlgoHeight3.h"

#include <QElapsedTimer>
#include <QPainter>
#include <QPolygonF>
#include <QStringList>
#include <QtMath>

#include <algorithm>
#include <cmath>
#include <vector>

// =============================================================================
// Small shared pieces
// =============================================================================

QString algoH3MethodName(int methodId)
{
	switch (methodId) {
	case (int)AlgoH3Method::Mean:       return QStringLiteral("Mean Height");
	case (int)AlgoH3Method::Median:     return QStringLiteral("Median Height");
	case (int)AlgoH3Method::Maximum:    return QStringLiteral("Maximum Height");
	case (int)AlgoH3Method::Minimum:    return QStringLiteral("Minimum Height");
	case (int)AlgoH3Method::Percentile: return QStringLiteral("Percentile Height");
	default: return QString();
	}
}

bool algoH3MethodValid(int methodId)
{
	return methodId >= 0 && methodId < kAlgoH3MethodCount;
}

int AlgoHeight3Params::indexOfType(const QString& name) const
{
	for (int i = 0; i < roiTypes.size(); i++)
		if (roiTypes[i].name == name) return i;
	return -1;
}

const AlgoH3RoiResult* AlgoHeight3Output::roiById(int id) const
{
	for (const auto& r : roiResults)
		if (r.id == id) return &r;
	return nullptr;
}

namespace {

// ── plane fitting ────────────────────────────────────────────────────────────
//
// Both methods produce the same shape - z = a*x + b*y + c, which is what the page's
// "z = ax + by + c" label promises - but they minimise different errors:
//   Least squares: vertical (z) distance. Right when x/y are exact and only z is noisy,
//                  which is what a height map is.
//   PCA/SVD:       perpendicular distance. Right when every axis is equally uncertain,
//                  and it degrades more gracefully when the datum patches are small.
// Neither can represent a vertical plane; both report that as degenerate rather than
// returning enormous coefficients.

struct H3Plane {
	double a = 0.0, b = 0.0, c = 0.0;
	bool valid = false;
};

struct H3Point { double x, y, z; };

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

//centroid-centred normal equations, so the solve stays conditioned even when the ROIs
//sit thousands of pixels from the image origin
static H3Plane fitPlaneLeastSquares(const std::vector<H3Point>& pts)
{
	H3Plane pl;
	const int n = (int)pts.size();
	if (n < 3) return pl;

	double mx = 0, my = 0, mz = 0;
	for (const auto& p : pts) { mx += p.x; my += p.y; mz += p.z; }
	mx /= n; my /= n; mz /= n;

	double ATA[3][3] = { {0,0,0},{0,0,0},{0,0,0} };
	double ATb[3] = { 0,0,0 };

	for (const auto& p : pts) {
		const double X = p.x - mx, Y = p.y - my, Z = p.z - mz;
		ATA[0][0] += X * X; ATA[0][1] += X * Y; ATA[0][2] += X;
		ATA[1][1] += Y * Y; ATA[1][2] += Y;
		ATA[2][2] += 1.0;
		ATb[0] += X * Z; ATb[1] += Y * Z; ATb[2] += Z;
	}
	ATA[1][0] = ATA[0][1]; ATA[2][0] = ATA[0][2]; ATA[2][1] = ATA[1][2];

	double p[3];
	if (!solve3x3(ATA, ATb, p)) return pl;

	pl.a = p[0];
	pl.b = p[1];
	pl.c = p[2] + mz - (p[0] * mx + p[1] * my); //back to absolute coordinates
	pl.valid = true;
	return pl;
}

//orthogonal fit: the plane normal is the eigenvector of the covariance matrix with the
//smallest eigenvalue (the direction the points vary in least)
static H3Plane fitPlanePcaSvd(const std::vector<H3Point>& pts)
{
	H3Plane pl;
	const int n = (int)pts.size();
	if (n < 3) return pl;

	double mx = 0, my = 0, mz = 0;
	for (const auto& p : pts) { mx += p.x; my += p.y; mz += p.z; }
	mx /= n; my /= n; mz /= n;

	double cxx = 0, cxy = 0, cxz = 0, cyy = 0, cyz = 0, czz = 0;
	for (const auto& p : pts) {
		const double X = p.x - mx, Y = p.y - my, Z = p.z - mz;
		cxx += X * X; cxy += X * Y; cxz += X * Z;
		cyy += Y * Y; cyz += Y * Z; czz += Z * Z;
	}

	cv::Mat cov = (cv::Mat_<double>(3, 3) <<
		cxx, cxy, cxz,
		cxy, cyy, cyz,
		cxz, cyz, czz);

	cv::Mat evals, evecs;
	if (!cv::eigen(cov, evals, evecs)) return pl;
	if (evecs.rows < 3 || evecs.cols < 3) return pl;

	//cv::eigen sorts descending, so the last row is the smallest - the normal
	const double nx = evecs.at<double>(2, 0);
	const double ny = evecs.at<double>(2, 1);
	const double nz = evecs.at<double>(2, 2);

	//a plane that is (near) vertical cannot be written as z = f(x,y) at all
	if (std::abs(nz) < 1e-9) return pl;

	pl.a = -nx / nz;
	pl.b = -ny / nz;
	pl.c = (nx * mx + ny * my + nz * mz) / nz;
	pl.valid = true;
	return pl;
}

static inline double planeZ(const H3Plane& pl, double x, double y)
{
	return pl.a * x + pl.b * y + pl.c;
}

// ── statistics ───────────────────────────────────────────────────────────────

//linear-interpolated percentile; v is sorted in place
static double percentileOf(std::vector<double>& v, double pct)
{
	if (v.empty()) return 0.0;
	std::sort(v.begin(), v.end());
	if (v.size() == 1) return v.front();

	const double p = std::min(100.0, std::max(0.0, pct)) / 100.0;
	const double pos = p * (double)(v.size() - 1);
	const size_t lo = (size_t)std::floor(pos);
	const size_t hi = (size_t)std::ceil(pos);
	if (lo == hi) return v[lo];
	const double f = pos - (double)lo;
	return v[lo] * (1.0 - f) + v[hi] * f;
}

// ── masks and kernels ────────────────────────────────────────────────────────

//"valid" everywhere in this pipeline means the RAW value is inside the section 0 band.
//Dropouts (raw 0) are excluded by that band, which is why minValidRaw is forced to >= 1.
static cv::Mat validMaskOf(const cv::Mat& height16, int minValidRaw, int maxValidRaw)
{
	cv::Mat mask;
	cv::inRange(height16, cv::Scalar(minValidRaw), cv::Scalar(maxValidRaw), mask);
	return mask;
}

static int oddKernel(int k, int lo, int hi)
{
	k = std::max(lo, std::min(hi, k));
	if ((k % 2) == 0) k++;               //an even kernel has no centre pixel
	return std::min(hi, k);
}

//min / max / median over the valid pixels only - used to pick a neutral fill value so a
//filter never drags a real height toward a dropout
static bool validStats(const cv::Mat& height16, const cv::Mat& mask,
	double& outMin, double& outMax)
{
	double mn = 0, mx = 0;
	cv::minMaxLoc(height16, &mn, &mx, nullptr, nullptr, mask);
	if (mx < mn) return false;
	outMin = mn; outMax = mx;
	return cv::countNonZero(mask) > 0;
}

} //namespace

// =============================================================================
// Source maps
// =============================================================================

void AlgoHeight3Pipeline::setSourceMaps(const cv::Mat& height16, const cv::Mat& intensity8)
{
	setHeightMap(height16);
	setIntensityMap(intensity8);
}

void AlgoHeight3Pipeline::setHeightMap(const cv::Mat& height16)
{
	m_height = cv::Mat();
	if (!height16.empty()) {
		//everything downstream indexes ushort directly, so normalise the depth once here
		if (height16.type() == CV_16U) m_height = height16.clone();
		else if (height16.channels() == 1) height16.convertTo(m_height, CV_16U);
		else {
			cv::Mat gray;
			cv::cvtColor(height16, gray, cv::COLOR_BGR2GRAY);
			gray.convertTo(m_height, CV_16U);
		}
	}
	invalidateFrom(AlgoH3Stage::Preprocess);
}

void AlgoHeight3Pipeline::setIntensityMap(const cv::Mat& intensity8)
{
	m_intensity = cv::Mat();
	if (!intensity8.empty()) {
		if (intensity8.type() == CV_8U) m_intensity = intensity8.clone();
		else if (intensity8.channels() == 1) intensity8.convertTo(m_intensity, CV_8U);
		else {
			cv::Mat gray;
			cv::cvtColor(intensity8, gray, cv::COLOR_BGR2GRAY);
			gray.convertTo(m_intensity, CV_8U);
		}
	}
	//the intensity map is never measured from, so only its crop needs rebuilding
	m_cropIntensity = cv::Mat();
}

void AlgoHeight3Pipeline::clearAll()
{
	m_height = cv::Mat();
	m_intensity = cv::Mat();
	invalidateFrom(AlgoH3Stage::Preprocess);
}

QSize AlgoHeight3Pipeline::sourceSize() const
{
	if (m_height.empty()) return QSize();
	return QSize(m_height.cols, m_height.rows);
}

QSize AlgoHeight3Pipeline::intensitySize() const
{
	if (m_intensity.empty()) return QSize();
	return QSize(m_intensity.cols, m_intensity.rows);
}

QSize AlgoHeight3Pipeline::cropSize() const
{
	if (m_cropHeight.empty()) return QSize();
	return QSize(m_cropHeight.cols, m_cropHeight.rows);
}

const cv::Mat& AlgoHeight3Pipeline::workingHeight() const
{
	return (m_preprocessDone && !m_work.empty()) ? m_work : m_height;
}

/*
* Invalidate this stage and every stage after it. Called before any stage runs and
* whenever the source maps change, so a displayed number is never older than the data
* it was computed from.
*/
void AlgoHeight3Pipeline::invalidateFrom(AlgoH3Stage stage)
{
	const int s = (int)stage;

	if (s <= (int)AlgoH3Stage::Preprocess) {
		m_preprocessDone = false;
		m_work = cv::Mat();
		m_out.preprocess = AlgoH3StageResult();
	}
	if (s <= (int)AlgoH3Stage::Segment) {
		m_segmentDone = false;
		m_cropHeight = cv::Mat();
		m_cropIntensity = cv::Mat();
		m_out.segment = AlgoH3StageResult();
		m_out.segWidthUm = m_out.segHeightUm = m_out.segAngleDeg = 0.0;
		m_out.segRectMap = QRectF();
		m_out.segCorners.clear();
		m_out.cropWidthPx = m_out.cropHeightPx = 0;
	}
	if (s <= (int)AlgoH3Stage::Datum) {
		m_datumDone = false;
		m_out.datum = AlgoH3StageResult();
		m_out.planeValid = false;
		m_out.planeA = m_out.planeB = m_out.planeC = 0.0;
		m_out.planeTiltDeg = m_out.planeRmsUm = 0.0;
		m_out.datumPoints = 0;
	}
	if (s <= (int)AlgoH3Stage::Measure) {
		m_measureDone = false;
		m_out.measure = AlgoH3StageResult();
		m_out.roiResults.clear();
	}
	if (s <= (int)AlgoH3Stage::Overall) {
		m_out.overall = AlgoH3StageResult();
		m_out.totalPins = m_out.passedPins = m_out.failedPins = 0;
		m_out.failedRatePct = 0.0;
		m_out.overallPass = false;
	}
}

// =============================================================================
// Stage dispatch
// =============================================================================

bool AlgoHeight3Pipeline::runStage(AlgoH3Stage stage, const AlgoHeight3Params& p)
{
	if (stage == AlgoH3Stage::All) {
		QElapsedTimer total;
		total.start();
		m_out.totalElapsedMs = 0;

		bool ok = doPreprocess(p)
			&& doSegment(p)
			&& doDatum(p)
			&& doMeasure(p)
			&& doOverall(p);

		m_out.totalElapsedMs = total.elapsed();
		return ok;
	}

	switch (stage) {
	case AlgoH3Stage::Preprocess: return doPreprocess(p);
	case AlgoH3Stage::Segment:    return doSegment(p);
	case AlgoH3Stage::Datum:      return doDatum(p);
	case AlgoH3Stage::Measure:    return doMeasure(p);
	case AlgoH3Stage::Overall:    return doOverall(p);
	default: return false;
	}
}

// =============================================================================
// Stage 1 - data preprocessing
// =============================================================================

/*
* Every filter here is MASKED: dropouts take no part in the result and never become
* valid. Filtering a profiler map naively is a real trap - a raw 0 is not "very low
* surface", it is "no measurement", and letting it into a mean or an erosion pulls
* genuine heights toward zero and quietly biases every height that follows.
*/
bool AlgoHeight3Pipeline::doPreprocess(const AlgoHeight3Params& p)
{
	invalidateFrom(AlgoH3Stage::Preprocess);

	QElapsedTimer t; t.start();
	auto& res = m_out.preprocess;
	res.ran = true;

	auto fail = [&](const QString& why) {
		res.pass = false;
		res.failReason = why;
		res.elapsedMs = t.elapsed();
		return false;
	};

	if (m_height.empty()) return fail(QStringLiteral("No height map loaded"));
	if (p.minValidRaw > p.maxValidRaw)
		return fail(QStringLiteral("Valid raw range is inverted (min > max)"));

	const cv::Mat mask = validMaskOf(m_height, p.minValidRaw, p.maxValidRaw);
	if (cv::countNonZero(mask) == 0)
		return fail(QStringLiteral("No pixel is inside the valid raw range"));

	double vMin = 0, vMax = 0;
	validStats(m_height, mask, vMin, vMax);

	cv::Mat dst;

	switch (p.preprocess) {
	case AlgoH3Preprocess::None:
		dst = m_height.clone();
		break;

	case AlgoH3Preprocess::Median: {
		//OpenCV's 16-bit medianBlur accepts an aperture of 3 or 5 only - refuse loudly
		//rather than silently filtering with a different kernel than the one on screen
		if (p.medianKernel > 5)
			return fail(QStringLiteral("Median on a 16-bit map supports kernel 3 or 5 only"));
		cv::medianBlur(m_height, dst, oddKernel(p.medianKernel, 3, 5));
		break;
	}

	case AlgoH3Preprocess::Gaussian: {
		//normalised convolution: blur value*mask and mask, then divide. That is a
		//Gaussian computed over the valid neighbours ONLY, with no zero-pull at a
		//dropout edge, and it still costs two separable blurs.
		const int k = oddKernel(p.gaussianKernel, 3, 99);
		const double sigma = std::max(0.0, p.gaussianSigma);

		cv::Mat val, w;
		m_height.convertTo(val, CV_32F);
		mask.convertTo(w, CV_32F, 1.0 / 255.0);
		cv::multiply(val, w, val);

		cv::GaussianBlur(val, val, cv::Size(k, k), sigma, sigma, cv::BORDER_REPLICATE);
		cv::GaussianBlur(w, w, cv::Size(k, k), sigma, sigma, cv::BORDER_REPLICATE);

		//floor the weights in place. A separate cv::max() result would be a third
		//full-size float image, and on a 32 Mpx map that is another 128 MB for nothing.
		cv::max(w, 1e-6f, w);
		cv::divide(val, w, val);
		val.convertTo(dst, CV_16U);
		break;
	}

	case AlgoH3Preprocess::Bilateral: {
		//bilateralFilter has no 16-bit overload, so this runs in float. Dropouts are
		//filled with the valid median first: the range term then treats them as ordinary
		//neighbours of a typical height instead of a huge edge, which is the least
		//damaging thing available without writing a masked bilateral by hand.
		const int d = std::max(1, std::min(31, p.bilateralDiameter));
		const double sSpace = std::max(0.1, p.bilateralSpatialSigma);
		const double sColor = std::max(0.1, p.bilateralHeightSigma);

		cv::Mat f;
		m_height.convertTo(f, CV_32F);
		const double fill = 0.5 * (vMin + vMax);
		f.setTo(cv::Scalar(fill), ~mask);

		cv::Mat filtered;
		cv::bilateralFilter(f, filtered, d, sColor, sSpace, cv::BORDER_REPLICATE);
		filtered.convertTo(dst, CV_16U);
		break;
	}

	case AlgoH3Preprocess::Opening:
	case AlgoH3Preprocess::Closing: {
		const bool opening = (p.preprocess == AlgoH3Preprocess::Opening);
		const int k = oddKernel(opening ? p.openingKernel : p.closingKernel, 3, 99);

		//opening erodes first, so a dropout would win every min in its neighbourhood;
		//fill it with the valid MAXIMUM so it cannot. Closing dilates first, so fill
		//with the valid MINIMUM for the mirror reason.
		cv::Mat src = m_height.clone();
		src.setTo(cv::Scalar(opening ? vMax : vMin), ~mask);

		const cv::Mat se = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(k, k));
		cv::morphologyEx(src, dst, opening ? cv::MORPH_OPEN : cv::MORPH_CLOSE, se,
			cv::Point(-1, -1), 1, cv::BORDER_REPLICATE);
		break;
	}

	default:
		return fail(QStringLiteral("Unknown preprocessing method"));
	}

	if (dst.empty() || dst.size() != m_height.size())
		return fail(QStringLiteral("Preprocessing produced no image"));

	//a dropout stays a dropout: no filter is allowed to invent a measurement
	dst.setTo(cv::Scalar(0), ~mask);

	m_work = dst;
	m_preprocessDone = true;

	res.pass = true;
	res.failReason.clear();
	res.elapsedMs = t.elapsed();
	return true;
}

// =============================================================================
// Stage 2 - segmentation
// =============================================================================

/*
* Pure geometry on a binary image, no pattern model and no blob library: everything
* inside the valid raw band is part, everything else is background, the largest
* connected region is the unit, and its minimum-area rectangle gives width, height and
* angle in one step.
*
* The same rectangle then defines the PART FRAME - the straightened crop every later
* stage works in.
*/
bool AlgoHeight3Pipeline::doSegment(const AlgoHeight3Params& p)
{
	invalidateFrom(AlgoH3Stage::Segment);

	QElapsedTimer t; t.start();
	auto& res = m_out.segment;
	res.ran = true;

	auto fail = [&](const QString& why) {
		res.pass = false;
		res.failReason = why;
		res.elapsedMs = t.elapsed();
		return false;
	};

	const cv::Mat& src = workingHeight();
	if (src.empty()) return fail(QStringLiteral("No height map loaded"));
	if (p.xScaleUmPx <= 0.0 || p.yScaleUmPx <= 0.0)
		return fail(QStringLiteral("X and Y scale must be greater than 0"));

	const cv::Mat mask = validMaskOf(src, p.minValidRaw, p.maxValidRaw);

	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
	if (contours.empty())
		return fail(QStringLiteral("No region found inside the valid raw range"));

	int best = -1;
	double bestArea = 0.0;
	for (int i = 0; i < (int)contours.size(); i++) {
		const double a = cv::contourArea(contours[i]);
		if (a > bestArea) { bestArea = a; best = i; }
	}
	if (best < 0 || bestArea < 4.0)
		return fail(QStringLiteral("Largest region is too small to be a part"));

	cv::RotatedRect rr = cv::minAreaRect(contours[best]);

	/*
	* Normalise to the SMALLEST rotation that straightens the part, in (-45, 45].
	* minAreaRect's own angle convention has changed between OpenCV versions; folding it
	* into this range makes the reported angle stable, keeps "width" the more horizontal
	* side, and stops a part that is upright reading as 90 degrees rotated.
	*/
	double ang = rr.angle;
	cv::Size2f sz = rr.size;
	for (int guard = 0; ang > 45.0 && guard < 8; guard++) { ang -= 90.0; std::swap(sz.width, sz.height); }
	for (int guard = 0; ang <= -45.0 && guard < 8; guard++) { ang += 90.0; std::swap(sz.width, sz.height); }

	const int cropW = (int)std::lround(sz.width);
	const int cropH = (int)std::lround(sz.height);
	if (cropW < 2 || cropH < 2)
		return fail(QStringLiteral("Segmented part is smaller than 2 px"));
	//a corrupt map can produce an absurd rectangle; refuse rather than try to allocate it
	if ((qint64)cropW * (qint64)cropH > 400000000LL)
		return fail(QStringLiteral("Segmented part is unreasonably large"));

	m_out.segWidthUm = sz.width * p.xScaleUmPx;
	m_out.segHeightUm = sz.height * p.yScaleUmPx;
	m_out.segAngleDeg = ang;
	m_out.cropWidthPx = cropW;
	m_out.cropHeightPx = cropH;

	{
		cv::Point2f pts[4];
		rr.points(pts);
		m_out.segCorners.clear();
		for (int i = 0; i < 4; i++) m_out.segCorners.append(QPointF(pts[i].x, pts[i].y));
		const cv::Rect br = rr.boundingRect();
		m_out.segRectMap = QRectF(br.x, br.y, br.width, br.height);
	}

	// ── the checks the operator enabled ──
	QStringList reasons;
	if (p.segCheckWidth && (m_out.segWidthUm < p.segMinWidthUm || m_out.segWidthUm > p.segMaxWidthUm))
		reasons << QStringLiteral("Width %1 um outside %2 .. %3")
			.arg(m_out.segWidthUm, 0, 'f', 1).arg(p.segMinWidthUm, 0, 'f', 1).arg(p.segMaxWidthUm, 0, 'f', 1);
	if (p.segCheckHeight && (m_out.segHeightUm < p.segMinHeightUm || m_out.segHeightUm > p.segMaxHeightUm))
		reasons << QStringLiteral("Height %1 um outside %2 .. %3")
			.arg(m_out.segHeightUm, 0, 'f', 1).arg(p.segMinHeightUm, 0, 'f', 1).arg(p.segMaxHeightUm, 0, 'f', 1);
	if (p.segCheckAngle && (m_out.segAngleDeg < p.segMinAngleDeg || m_out.segAngleDeg > p.segMaxAngleDeg))
		reasons << QStringLiteral("Angle %1 deg outside %2 .. %3")
			.arg(m_out.segAngleDeg, 0, 'f', 3).arg(p.segMinAngleDeg, 0, 'f', 3).arg(p.segMaxAngleDeg, 0, 'f', 3);

	if (!reasons.isEmpty()) return fail(reasons.join(QStringLiteral("; ")));

	/*
	* Build the straightened crop in ONE warp - rotate about the part centre and land the
	* part in a cropW x cropH canvas - so a 60 MB map is never rotated whole.
	*
	* INTER_NEAREST is not a performance choice, it is a correctness one: interpolating
	* two height samples invents a height that was never measured, and blending a real
	* height with a dropout invents a much worse one.
	*/
	cv::Mat M = cv::getRotationMatrix2D(rr.center, ang, 1.0);
	M.at<double>(0, 2) += cropW / 2.0 - rr.center.x;
	M.at<double>(1, 2) += cropH / 2.0 - rr.center.y;

	cv::warpAffine(src, m_cropHeight, M, cv::Size(cropW, cropH),
		cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(0));

	if (!m_intensity.empty() && m_intensity.size() == src.size()) {
		//same transform, so the intensity crop stays pixel-aligned with the height crop
		cv::warpAffine(m_intensity, m_cropIntensity, M, cv::Size(cropW, cropH),
			cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(0));
	}
	else {
		m_cropIntensity = cv::Mat();
	}

	if (m_cropHeight.empty()) return fail(QStringLiteral("Failed to build the segmented image"));

	m_segmentDone = true;
	res.pass = true;
	res.failReason.clear();
	res.elapsedMs = t.elapsed();
	return true;
}

// =============================================================================
// Stage 3 - datum plane
// =============================================================================

bool AlgoHeight3Pipeline::doDatum(const AlgoHeight3Params& p)
{
	invalidateFrom(AlgoH3Stage::Datum);

	QElapsedTimer t; t.start();
	auto& res = m_out.datum;
	res.ran = true;

	auto fail = [&](const QString& why) {
		res.pass = false;
		res.failReason = why;
		res.elapsedMs = t.elapsed();
		return false;
	};

	if (!segmentReady()) return fail(QStringLiteral("Run segmentation first"));
	if (p.datumRois.isEmpty()) return fail(QStringLiteral("Add at least one datum ROI"));
	if (p.zScaleRawPerUm <= 0.0) return fail(QStringLiteral("Z scale must be greater than 0"));

	const int w = m_cropHeight.cols, h = m_cropHeight.rows;
	const cv::Rect bounds(0, 0, w, h);

	std::vector<H3Point> pts;
	int usedRois = 0;

	for (const auto& rel : p.datumRois) {
		//part frame -> crop pixels
		const QRectF abs = rel.translated(w / 2.0, h / 2.0);
		cv::Rect r((int)std::floor(abs.left()), (int)std::floor(abs.top()),
			(int)std::lround(abs.width()), (int)std::lround(abs.height()));
		r &= bounds;                       //fully outside collapses to an empty rect
		if (r.width <= 0 || r.height <= 0) continue;

		usedRois++;
		for (int y = r.y; y < r.y + r.height; y++) {
			const ushort* row = m_cropHeight.ptr<ushort>(y);
			for (int x = r.x; x < r.x + r.width; x++) {
				const int z = row[x];
				if (z < p.minValidRaw || z > p.maxValidRaw) continue;
				pts.push_back({ (double)x, (double)y, (double)z });
			}
		}
	}

	if (usedRois == 0) return fail(QStringLiteral("Every datum ROI is outside the segmented image"));
	if (pts.size() < 3) return fail(QStringLiteral("Fewer than 3 valid points in the datum ROIs"));

	const H3Plane plane = (p.datumMethod == AlgoH3DatumMethod::PcaSvd)
		? fitPlanePcaSvd(pts)
		: fitPlaneLeastSquares(pts);

	if (!plane.valid)
		return fail(QStringLiteral("Plane fit failed - the datum points are degenerate"));

	m_out.planeValid = true;
	m_out.planeA = plane.a;
	m_out.planeB = plane.b;
	m_out.planeC = plane.c;
	m_out.datumPoints = (qint64)pts.size();

	//absolute angle between the fitted plane and the map plane; a tilt has no sign that
	//matters here, so the page asks for a maximum only
	m_out.planeTiltDeg = qRadiansToDegrees(std::atan(std::sqrt(plane.a * plane.a + plane.b * plane.b)));

	double sumSq = 0.0;
	for (const auto& pt : pts) {
		const double d = (pt.z - planeZ(plane, pt.x, pt.y)) / p.zScaleRawPerUm;
		sumSq += d * d;
	}
	m_out.planeRmsUm = std::sqrt(sumSq / (double)pts.size());

	if (p.datumCheckTilt && m_out.planeTiltDeg > p.datumMaxTiltDeg) {
		return fail(QStringLiteral("Plane tilt %1 deg exceeds %2 deg")
			.arg(m_out.planeTiltDeg, 0, 'f', 3).arg(p.datumMaxTiltDeg, 0, 'f', 3));
	}

	m_datumDone = true;
	res.pass = true;
	res.failReason.clear();
	res.elapsedMs = t.elapsed();
	return true;
}

// =============================================================================
// Stage 4 - height measurement
// =============================================================================

bool AlgoHeight3Pipeline::doMeasure(const AlgoHeight3Params& p)
{
	invalidateFrom(AlgoH3Stage::Measure);

	QElapsedTimer t; t.start();
	auto& res = m_out.measure;
	res.ran = true;

	auto fail = [&](const QString& why) {
		res.pass = false;
		res.failReason = why;
		res.elapsedMs = t.elapsed();
		return false;
	};

	if (!datumReady()) return fail(QStringLiteral("Fit the datum plane first"));
	if (p.rois.isEmpty()) return fail(QStringLiteral("Add at least one measurement ROI"));
	if (p.zScaleRawPerUm <= 0.0) return fail(QStringLiteral("Z scale must be greater than 0"));

	H3Plane plane;
	plane.a = m_out.planeA; plane.b = m_out.planeB; plane.c = m_out.planeC; plane.valid = true;

	const int w = m_cropHeight.cols, h = m_cropHeight.rows;
	const cv::Rect bounds(0, 0, w, h);
	const bool heightBandActive = (p.maxValidHeightUm > p.minValidHeightUm);

	std::vector<double> samples;

	for (int i = 0; i < p.rois.size(); i++) {
		const AlgoH3Roi& roi = p.rois[i];

		AlgoH3RoiResult r;
		r.id = i + 1;                      //1-based, same number the box shows on screen
		r.typeName = roi.typeName;
		r.rel = roi.rel;

		const int ti = p.indexOfType(roi.typeName);
		if (ti < 0) {
			//cannot happen through the page (deleting a type deletes its ROIs) but a
			//hand-edited recipe can produce it, and guessing a type would be worse
			r.failReason = QStringLiteral("ROI type '%1' does not exist").arg(roi.typeName);
			m_out.roiResults.append(r);
			continue;
		}

		const AlgoH3RoiType& type = p.roiTypes[ti];
		r.color = type.color;
		r.criteriaMinUm = type.minUm;
		r.criteriaMaxUm = type.maxUm;
		r.methodId = type.methodId;
		r.methodName = algoH3MethodName(type.methodId);

		if (!algoH3MethodValid(type.methodId)) {
			r.failReason = QStringLiteral("Type '%1' has Method ID %2, which does not exist")
				.arg(type.name).arg(type.methodId);
			m_out.roiResults.append(r);
			continue;
		}

		const QRectF abs = roi.rel.translated(w / 2.0, h / 2.0);
		cv::Rect want((int)std::floor(abs.left()), (int)std::floor(abs.top()),
			(int)std::lround(abs.width()), (int)std::lround(abs.height()));
		cv::Rect got = want & bounds;

		//fully outside: nothing to measure, so the ROI fails rather than reporting 0 um
		if (got.width <= 0 || got.height <= 0) {
			r.failReason = QStringLiteral("ROI is completely outside the segmented image");
			m_out.roiResults.append(r);
			continue;
		}
		//partially outside: measure what is left, and say so
		r.clipped = (got != want);
		r.measuredRect = QRectF(got.x, got.y, got.width, got.height);

		samples.clear();
		samples.reserve((size_t)got.width * (size_t)got.height);

		for (int y = got.y; y < got.y + got.height; y++) {
			const ushort* row = m_cropHeight.ptr<ushort>(y);
			for (int x = got.x; x < got.x + got.width; x++) {
				const int z = row[x];
				if (z < p.minValidRaw || z > p.maxValidRaw) continue;

				const double um = ((double)z - planeZ(plane, x, y)) / p.zScaleRawPerUm;
				//the valid-height band throws away noise, dropout shoulders and holes
				//BEFORE the statistic, so one bad pixel cannot decide a maximum
				if (heightBandActive && (um < p.minValidHeightUm || um > p.maxValidHeightUm)) continue;
				samples.push_back(um);
			}
		}

		r.sampleCount = (qint64)samples.size();
		if (samples.empty()) {
			r.failReason = r.clipped
				? QStringLiteral("No valid data in the part of the ROI inside the image")
				: QStringLiteral("No valid data in the ROI");
			m_out.roiResults.append(r);
			continue;
		}

		switch ((AlgoH3Method)type.methodId) {
		case AlgoH3Method::Mean: {
			double sum = 0.0;
			for (double v : samples) sum += v;
			r.heightUm = sum / (double)samples.size();
			break;
		}
		case AlgoH3Method::Median:
			r.heightUm = percentileOf(samples, 50.0);
			break;
		case AlgoH3Method::Maximum:
			r.heightUm = *std::max_element(samples.begin(), samples.end());
			break;
		case AlgoH3Method::Minimum:
			r.heightUm = *std::min_element(samples.begin(), samples.end());
			break;
		case AlgoH3Method::Percentile:
			r.heightUm = percentileOf(samples, p.percentile);
			break;
		}

		r.valid = true;

		//max <= min means the type carries no criteria, so the ROI simply reports
		const bool criteriaActive = (type.maxUm > type.minUm);
		if (!criteriaActive) {
			r.pass = true;
		}
		else if (r.heightUm < type.minUm) {
			r.pass = false;
			r.failReason = QStringLiteral("%1 um below %2 um").arg(r.heightUm, 0, 'f', 2).arg(type.minUm, 0, 'f', 2);
		}
		else if (r.heightUm > type.maxUm) {
			r.pass = false;
			r.failReason = QStringLiteral("%1 um above %2 um").arg(r.heightUm, 0, 'f', 2).arg(type.maxUm, 0, 'f', 2);
		}
		else {
			r.pass = true;
		}

		m_out.roiResults.append(r);
	}

	m_measureDone = true;
	res.pass = true;   //the stage itself succeeded; individual ROIs carry their own verdict
	res.failReason.clear();
	res.elapsedMs = t.elapsed();
	return true;
}

// =============================================================================
// Stage 5 - overall result
// =============================================================================

bool AlgoHeight3Pipeline::doOverall(const AlgoHeight3Params& p)
{
	invalidateFrom(AlgoH3Stage::Overall);

	QElapsedTimer t; t.start();
	auto& res = m_out.overall;
	res.ran = true;

	auto fail = [&](const QString& why) {
		res.pass = false;
		res.failReason = why;
		res.elapsedMs = t.elapsed();
		return false;
	};

	if (!m_measureDone) return fail(QStringLiteral("Run the height measurement first"));

	m_out.totalPins = m_out.roiResults.size();
	m_out.passedPins = 0;
	for (const auto& r : m_out.roiResults) if (r.pass) m_out.passedPins++;
	m_out.failedPins = m_out.totalPins - m_out.passedPins;
	m_out.failedRatePct = (m_out.totalPins > 0)
		? (100.0 * (double)m_out.failedPins / (double)m_out.totalPins)
		: 0.0;

	QStringList reasons;
	if (p.overallCheckCount && m_out.failedPins > p.overallMaxCount)
		reasons << QStringLiteral("%1 failed pins exceeds the maximum of %2")
			.arg(m_out.failedPins).arg(p.overallMaxCount);
	if (p.overallCheckRate && m_out.failedRatePct > p.overallMaxRatePct)
		reasons << QStringLiteral("Failed rate %1% exceeds the maximum of %2%")
			.arg(m_out.failedRatePct, 0, 'f', 2).arg(p.overallMaxRatePct, 0, 'f', 2);

	//with neither criterion enabled the unit passes only if every pin passed - a silent
	//"pass with 40 failed pins" would be the worst possible default
	if (!p.overallCheckCount && !p.overallCheckRate && m_out.failedPins > 0)
		reasons << QStringLiteral("%1 pin(s) failed and no failure criteria are enabled")
			.arg(m_out.failedPins);

	m_out.overallPass = reasons.isEmpty();

	res.pass = m_out.overallPass;
	res.failReason = reasons.join(QStringLiteral("; "));
	res.elapsedMs = t.elapsed();
	return true;   //the stage computed a verdict; the verdict itself is overallPass
}

// =============================================================================
// Display sources
// =============================================================================

cv::Mat AlgoHeight3Pipeline::heightForDisplay(bool preprocessed, bool segmented) const
{
	if (segmented && !m_cropHeight.empty()) return m_cropHeight;
	if (preprocessed && m_preprocessDone && !m_work.empty()) return m_work;
	return m_height;
}

cv::Mat AlgoHeight3Pipeline::intensityForDisplay(bool segmented) const
{
	if (segmented && !m_cropIntensity.empty()) return m_cropIntensity;
	return m_intensity;
}

// =============================================================================
// Renders
// =============================================================================

QImage algoH3HeightToQImage(const cv::Mat& height16, int minValidRaw, int maxValidRaw, bool colorMapped)
{
	if (height16.empty()) return QImage();

	cv::Mat src;
	if (height16.type() == CV_16U) src = height16;
	else height16.convertTo(src, CV_16U);

	const cv::Mat mask = validMaskOf(src, minValidRaw, maxValidRaw);

	double mn = 0, mx = 0;
	if (cv::countNonZero(mask) > 0) cv::minMaxLoc(src, &mn, &mx, nullptr, nullptr, mask);
	if (mx <= mn) { mn = 0; mx = 65535; }

	//valid heights map into 1..255 so that 0 is left to mean "no data". In the grey view
	//that is the only thing separating a dropout from the lowest real surface; the colour
	//view does not need it because it repaints dropouts below.
	cv::Mat gray8;
	src.convertTo(gray8, CV_8U, 254.0 / (mx - mn), 1.0 - mn * 254.0 / (mx - mn));
	gray8.setTo(0, ~mask);

	if (!colorMapped) {
		QImage img((const uchar*)gray8.data, gray8.cols, gray8.rows, (int)gray8.step,
			QImage::Format_Grayscale8);
		return img.copy();
	}

	cv::Mat color;
	cv::applyColorMap(gray8, color, cv::COLORMAP_JET);
	color.setTo(cv::Scalar(20, 20, 20), ~mask);   //dropouts read as near-black, not deep blue
	cv::cvtColor(color, color, cv::COLOR_BGR2RGB);
	QImage img((const uchar*)color.data, color.cols, color.rows, (int)color.step,
		QImage::Format_RGB888);
	return img.copy();
}

QImage algoH3GrayToQImage(const cv::Mat& gray8)
{
	if (gray8.empty()) return QImage();

	cv::Mat g;
	if (gray8.type() == CV_8U) g = gray8;
	else gray8.convertTo(g, CV_8U);

	QImage img((const uchar*)g.data, g.cols, g.rows, (int)g.step, QImage::Format_Grayscale8);
	return img.copy();
}

// ── 3D surface view ─────────────────────────────────────────────────────────

namespace {

//JET ramp as a 256-entry table. Built once through a function-local static, so the
//initialisation is thread-safe even if a render ever runs off the GUI thread.
static const QRgb* jetTable()
{
	static const std::vector<QRgb> table = []() {
		std::vector<QRgb> t(256, 0);
		cv::Mat ramp(1, 256, CV_8U);
		for (int i = 0; i < 256; i++) ramp.at<uchar>(0, i) = (uchar)i;
		cv::Mat color;
		cv::applyColorMap(ramp, color, cv::COLORMAP_JET);
		for (int i = 0; i < 256; i++) {
			const cv::Vec3b& b = color.at<cv::Vec3b>(0, i);
			t[i] = qRgb(b[2], b[1], b[0]);   //BGR -> RGB
		}
		return t;
	}();
	return table.data();
}

struct H3Quad {
	QPolygonF poly;
	double depth = 0.0;
	QRgb color = 0;
};

} //namespace

/*
* A genuine projected surface: the map becomes a mesh, the mesh is rotated by the yaw and
* pitch the operator has dragged to, and the quads are painted far-to-near (painter's
* algorithm) so the shape occludes itself the way a solid would. Height also drives the
* colour, so the JET ramp and the relief agree.
*
* Downsampled to a fixed grid budget: a drag has to stay responsive, and this is a
* viewing aid - nothing is ever measured from it.
*/
QImage algoH3RenderSurface3D(const cv::Mat& height16, int minValidRaw, int maxValidRaw,
	double yawDeg, double pitchDeg, double zExaggeration, const QSize& outSize)
{
	const QSize size = (outSize.width() >= 64 && outSize.height() >= 64) ? outSize : QSize(900, 700);

	QImage img(size, QImage::Format_RGB888);
	img.fill(QColor(24, 26, 32));
	if (height16.empty()) return img;

	cv::Mat src;
	if (height16.type() == CV_16U) src = height16;
	else height16.convertTo(src, CV_16U);

	// ── downsample to a grid we can rotate interactively ──
	constexpr int kGridBudget = 150;
	const double shrink = std::min(1.0,
		(double)kGridBudget / (double)std::max(src.cols, src.rows));
	const int gw = std::max(2, (int)std::lround(src.cols * shrink));
	const int gh = std::max(2, (int)std::lround(src.rows * shrink));

	cv::Mat grid;
	cv::resize(src, grid, cv::Size(gw, gh), 0, 0, cv::INTER_NEAREST);

	const cv::Mat gmask = validMaskOf(grid, minValidRaw, maxValidRaw);
	double zMin = 0, zMax = 0;
	if (cv::countNonZero(gmask) == 0) return img;
	cv::minMaxLoc(grid, &zMin, &zMax, nullptr, nullptr, gmask);
	if (zMax <= zMin) zMax = zMin + 1.0;

	// ── model space: x,y in [-1,1] scaled by aspect, z centred on 0 ──
	const double aspect = (double)gw / (double)std::max(1, gh);
	const double ax = (aspect >= 1.0) ? 1.0 : aspect;
	const double ay = (aspect >= 1.0) ? 1.0 / aspect : 1.0;
	const double zSpan = std::max(0.05, std::min(2.0, zExaggeration)) * 0.5;

	const double yaw = qDegreesToRadians(yawDeg);
	const double elev = qDegreesToRadians(std::max(2.0, std::min(89.0, pitchDeg)));
	const double cy = std::cos(yaw), sy = std::sin(yaw);
	const double ce = std::cos(elev), se = std::sin(elev);

	std::vector<double> px((size_t)gw * gh, 0.0), py((size_t)gw * gh, 0.0), pd((size_t)gw * gh, 0.0);
	std::vector<uchar> ok((size_t)gw * gh, 0);
	std::vector<uchar> shade((size_t)gw * gh, 0);

	double minX = 1e18, maxX = -1e18, minY = 1e18, maxY = -1e18;

	for (int j = 0; j < gh; j++) {
		const ushort* row = grid.ptr<ushort>(j);
		const uchar* mrow = gmask.ptr<uchar>(j);
		for (int i = 0; i < gw; i++) {
			const size_t idx = (size_t)j * gw + i;
			if (!mrow[i]) continue;

			const double t = ((double)row[i] - zMin) / (zMax - zMin);   //0..1
			const double x = (2.0 * i / (double)(gw - 1) - 1.0) * ax;
			const double y = (2.0 * j / (double)(gh - 1) - 1.0) * ay;
			const double z = (t - 0.5) * 2.0 * zSpan;

			//spin about the vertical axis, then tilt the whole thing toward the viewer
			const double xr = x * cy - y * sy;
			const double yr = x * sy + y * cy;

			px[idx] = xr;
			py[idx] = yr * se - z * ce;       //screen y grows downward, so height climbs
			pd[idx] = yr * ce + z * se;       //toward the viewer
			ok[idx] = 1;
			shade[idx] = (uchar)std::lround(std::max(0.0, std::min(1.0, t)) * 255.0);

			minX = std::min(minX, px[idx]); maxX = std::max(maxX, px[idx]);
			minY = std::min(minY, py[idx]); maxY = std::max(maxY, py[idx]);
		}
	}

	if (maxX <= minX || maxY <= minY) return img;

	// ── fit the projection to the canvas, whatever the angle ──
	const double margin = 18.0;
	const double sx = (size.width() - 2 * margin) / (maxX - minX);
	const double sYy = (size.height() - 2 * margin) / (maxY - minY);
	const double s = std::min(sx, sYy);
	const double offX = margin + ((size.width() - 2 * margin) - (maxX - minX) * s) / 2.0;
	const double offY = margin + ((size.height() - 2 * margin) - (maxY - minY) * s) / 2.0;

	auto toScreen = [&](size_t idx) {
		return QPointF(offX + (px[idx] - minX) * s, offY + (py[idx] - minY) * s);
	};

	// ── build the quads, skipping any cell with a dropout corner ──
	const QRgb* lut = jetTable();
	std::vector<H3Quad> quads;
	quads.reserve((size_t)(gw - 1) * (gh - 1));

	for (int j = 0; j < gh - 1; j++) {
		for (int i = 0; i < gw - 1; i++) {
			const size_t a = (size_t)j * gw + i;
			const size_t b = a + 1;
			const size_t c = a + gw;
			const size_t d = c + 1;
			if (!ok[a] || !ok[b] || !ok[c] || !ok[d]) continue;

			H3Quad q;
			q.poly << toScreen(a) << toScreen(b) << toScreen(d) << toScreen(c);
			q.depth = 0.25 * (pd[a] + pd[b] + pd[c] + pd[d]);
			const int level = (int)((shade[a] + shade[b] + shade[c] + shade[d]) / 4);
			q.color = lut[std::max(0, std::min(255, level))];
			quads.push_back(std::move(q));
		}
	}

	if (quads.empty()) return img;

	//painter's algorithm: far first, so nearer geometry paints over it
	std::sort(quads.begin(), quads.end(),
		[](const H3Quad& l, const H3Quad& r) { return l.depth < r.depth; });

	QPainter painter(&img);
	painter.setRenderHint(QPainter::Antialiasing, false);

	for (const auto& q : quads) {
		const QColor c(q.color);
		//the pen closes the hairline seams rounding leaves between neighbouring quads
		painter.setPen(QPen(c, 1));
		painter.setBrush(c);
		painter.drawPolygon(q.poly);
	}

	painter.end();
	return img;
}
