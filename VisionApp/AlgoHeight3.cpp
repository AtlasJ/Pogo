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
#include "AlgoH3GLSurface.h"
#include "AlgoH3HoleFill.h"

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

/*
* A Run All stops at the first stage that fails, so the useful summary is the name of that
* stage plus its own reason. Only when every stage passed is the pin tally meaningful - and
* the tally is what gets reported rather than a list of heights, because a part here has
* hundreds of pins and a comma-joined list of them is not a result anyone can read.
*/
QString algoH3RunSummary(const AlgoHeight3Output& out)
{
	if (!out.preprocess.ran) return QStringLiteral("did not run");
	if (!out.preprocess.pass) return QStringLiteral("preprocessing failed: ") + out.preprocess.failReason;
	if (!out.segment.pass)    return QStringLiteral("segmentation failed: ") + out.segment.failReason;
	if (!out.datum.pass)      return QStringLiteral("plane fit failed: ") + out.datum.failReason;
	if (!out.measure.pass)    return QStringLiteral("measurement failed: ") + out.measure.failReason;

	QString s = QStringLiteral("%1/%2 pins passed").arg(out.passedPins).arg(out.totalPins);
	if (!out.overallPass && !out.overall.failReason.isEmpty())
		s += QStringLiteral(" - ") + out.overall.failReason;
	//a unit whose part outgrew the canvas passed on the pins that were still in frame, which
	//is not the same thing as passing - say so on the production line too, not just the page
	if (out.segOversized) s += QStringLiteral(" [OVERSIZED: ") + out.segment.note + QLatin1Char(']');
	return s;
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

/*
* Orthogonal fit: the plane normal is the eigenvector of the covariance matrix with the
* smallest eigenvalue (the direction the points vary in least).
*
* A PERPENDICULAR distance only means something with one unit on every axis, so the points
* are taken to um first (sx, sy = um per px, sz = um per raw grey level). Fitted in raw
* px/grey, a grey level (0.8 um) and a pixel (5 um) counted as the same length: heights were
* stretched 6.25x and the "orthogonal" fit was not orthogonal in the real world. The plane is
* handed back in raw grey per px, the units every caller works in.
*
* Least squares needs no such care - a VERTICAL fit comes out the same plane in any units,
* only its coefficients change - which is why only this method takes the scales.
*/
static H3Plane fitPlanePcaSvd(const std::vector<H3Point>& pts, double sx, double sy, double sz)
{
	H3Plane pl;
	const int n = (int)pts.size();
	if (n < 3) return pl;

	double mx = 0, my = 0, mz = 0;
	for (const auto& p : pts) { mx += p.x; my += p.y; mz += p.z; }
	mx /= n; my /= n; mz /= n;

	double cxx = 0, cxy = 0, cxz = 0, cyy = 0, cyz = 0, czz = 0;
	for (const auto& p : pts) {
		const double X = (p.x - mx) * sx, Y = (p.y - my) * sy, Z = (p.z - mz) * sz;
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

	//slopes come out in um per um; back to raw grey per px, through the same centroid
	pl.a = (-nx / nz) * sx / sz;
	pl.b = (-ny / nz) * sy / sz;
	pl.c = mz - pl.a * mx - pl.b * my;
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

/*
* Median over the VALID neighbours only.
*
* cv::medianBlur has no concept of a mask, so on a height map it ranks the dropout
* zeros as if they were real measurements: a window that is majority-dropout returns
* 0 - turning measured pixels into dropouts - and even a minority of zeros drags the
* rank down. That matters here because the laser cannot see the sides of the pins, so
* ~19% of the part's own rectangle is interior dropout and every pin has a halo.
* Measured on 20260828_193223 at k=5: 28,551 valid pixels destroyed outright, 51,960
* wrong by more than 100 um, worst case 25 mm. Median was the only method in
* doPreprocess that was not dropout-aware; Gaussian, Bilateral and the morphology
* branches all already are.
*
* Where every pixel in the window is valid this returns exactly what cv::medianBlur
* returns - verified over 14,449,859 such pixels on that map, 100.00% identical - so
* it is a strict repair, not a different filter.
*
* On an even number of valid neighbours this takes the upper middle sample rather
* than averaging the middle two, because averaging would invent a height that no
* pixel actually measured. Same reason doPreprocess re-imposes the mask at the end.
*
* Doing it by hand also removes OpenCV's 3-or-5 aperture limit for 16-bit input: any
* odd kernel up to kMaxMedianK works. The 3/5 cap is kept at the call site so saved
* recipes keep their meaning - lifting it is a separate, deliberate decision.
*/
static const int kMaxMedianK = 11;

static void maskedMedian16(const cv::Mat& src, const cv::Mat& mask, int k, cv::Mat& dst)
{
	CV_Assert(src.type() == CV_16U && mask.type() == CV_8U && src.size() == mask.size());
	CV_Assert(k >= 1 && k <= kMaxMedianK && (k % 2) == 1);

	const int r = k / 2;
	const int rows = src.rows, cols = src.cols;
	dst.create(src.size(), CV_16U);

	cv::parallel_for_(cv::Range(0, rows), [&](const cv::Range& band) {
		//fixed buffer: no allocation per pixel, and k is bounded by kMaxMedianK
		unsigned short buf[kMaxMedianK * kMaxMedianK];

		for (int y = band.start; y < band.end; y++) {
			const unsigned char* mRow = mask.ptr<unsigned char>(y);
			unsigned short* dRow = dst.ptr<unsigned short>(y);
			const int y0 = std::max(0, y - r), y1 = std::min(rows - 1, y + r);

			for (int x = 0; x < cols; x++) {
				//a dropout stays a dropout: no filter is allowed to invent a measurement
				if (!mRow[x]) { dRow[x] = 0; continue; }

				const int x0 = std::max(0, x - r), x1 = std::min(cols - 1, x + r);
				int n = 0;
				for (int yy = y0; yy <= y1; yy++) {
					const unsigned short* s = src.ptr<unsigned short>(yy);
					const unsigned char* m = mask.ptr<unsigned char>(yy);
					for (int xx = x0; xx <= x1; xx++)
						if (m[xx]) buf[n++] = s[xx];
				}
				//the centre pixel is valid, so n >= 1 always
				const int mid = n / 2;
				std::nth_element(buf, buf + mid, buf + n);
				dRow[x] = buf[mid];
			}
		}
	});
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
		m_out.segOversized = false;
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
		//masked: the median is taken over the valid neighbours only, so a pin's dropout
		//halo cannot drag its top down or zero it out. See maskedMedian16.
		//The kernel stays capped at 3 or 5 so saved recipes keep their meaning, even
		//though maskedMedian16 itself has no such limit - refuse loudly rather than
		//silently filtering with a different kernel than the one on screen.
		if (p.medianKernel > 5)
			return fail(QStringLiteral("Median kernel must be 3 or 5"));
		maskedMedian16(m_height, mask, oddKernel(p.medianKernel, 3, 5), dst);
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
namespace {

/*
* Segmentation method 0 - the largest connected region of valid pixels, posed by the
* min-area rect of its outer contour.
*
* RETR_EXTERNAL matters: the laser cannot see the sides of the pins, so ~19% of this
* part's own rectangle is interior dropout. External contours ignore those holes, so
* the pose comes from the part's outline and nothing else.
*
* EVERY segmentation method ends the same way - producing ONE cv::RotatedRect for the
* unit. Everything after the dispatch in doSegment (angle folding, crop sizing, the
* warpAffine straighten, the operator's checks) is deliberately method-agnostic, so a
* new method is one function plus one enum value plus one combo item, and never a
* change to doSegment's tail.
*/
static bool h3SegLargestRegion(const cv::Mat& src, const AlgoHeight3Params& p,
	cv::RotatedRect& out, QString& why)
{
	const cv::Mat mask = validMaskOf(src, p.minValidRaw, p.maxValidRaw);

	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
	if (contours.empty()) {
		why = QStringLiteral("No region found inside the valid raw range");
		return false;
	}

	int best = -1;
	double bestArea = 0.0;
	for (int i = 0; i < (int)contours.size(); i++) {
		const double a = cv::contourArea(contours[i]);
		if (a > bestArea) { bestArea = a; best = i; }
	}
	if (best < 0 || bestArea < 4.0) {
		why = QStringLiteral("Largest region is too small to be a part");
		return false;
	}

	out = cv::minAreaRect(contours[best]);
	return true;
}

/*
* Estimate how far the part is rotated in the map, from the long thin lines of dropout that
* run across it.
*
* A pogo field is laid out in rows, and a row of pins casts a row of shadows: the sensor loses
* the surface in a broken line that runs the length of the block. The seams between blocks do
* the same. Both are features OF the part, so both carry its rotation - and unlike the plate
* level, which is a majority statistic and so steps sharply whatever the angle, a line has a
* direction that can simply be measured.
*
* The dropouts around the part are far larger than any of these lines, so the search is
* confined to the part's interior first; then an opening by a long flat kernel keeps only what
* runs a long way horizontally, which a pin's own shadow does not.
*
* Returns false when too little lines up to be sure, and the caller then works unrotated -
* which is right, because a part that is square to the scan has no rotation to find.
*/
static bool h3PartRotation(const cv::Mat& validMask, double maxDeg, double& thetaDeg)
{
	const int h = validMask.rows, w = validMask.cols;
	if (h < 64 || w < 64) return false;

	//the part's outline: close hard enough to bridge the pin shadows, then take the biggest
	//thing left. Everything outside it is background and has nothing to say about rotation.
	const int closeK = std::max(31, std::min(201, w / 96) | 1);
	cv::Mat closed;
	cv::morphologyEx(validMask, closed, cv::MORPH_CLOSE,
		cv::getStructuringElement(cv::MORPH_RECT, cv::Size(closeK, closeK)));

	cv::Mat labels, stats, centroids;
	const int nLab = cv::connectedComponentsWithStats(closed, labels, stats, centroids, 8, CV_32S);
	if (nLab < 2) return false;
	int interior = 1;
	for (int i = 2; i < nLab; i++)
		if (stats.at<int>(i, cv::CC_STAT_AREA) > stats.at<int>(interior, cv::CC_STAT_AREA)) interior = i;

	cv::Mat holes(h, w, CV_8U, cv::Scalar(0));
	for (int y = 0; y < h; y++) {
		const int* lrow = labels.ptr<int>(y);
		const uchar* vrow = validMask.ptr<uchar>(y);
		uchar* orow = holes.ptr<uchar>(y);
		for (int x = 0; x < w; x++) if (lrow[x] == interior && !vrow[x]) orow[x] = 255;
	}

	//long and flat: a pin's shadow is a blob a few hundred px across, a shadow ROW is many
	//times that. The kernel is a fraction of the width so it scales with the field.
	const int openK = std::max(81, std::min(601, w / 56) | 1);
	cv::Mat lines;
	cv::morphologyEx(holes, lines, cv::MORPH_OPEN,
		cv::getStructuringElement(cv::MORPH_RECT, cv::Size(openK, 1)));

	cv::Mat lLab, lStats, lCent;
	const int nSeg = cv::connectedComponentsWithStats(lines, lLab, lStats, lCent, 8, CV_32S);

	struct Frag { double x, y; };
	std::vector<Frag> frags;
	for (int i = 1; i < nSeg; i++) {
		const int x = lStats.at<int>(i, cv::CC_STAT_LEFT), y = lStats.at<int>(i, cv::CC_STAT_TOP);
		const int bw = lStats.at<int>(i, cv::CC_STAT_WIDTH), bh = lStats.at<int>(i, cv::CC_STAT_HEIGHT);
		const int area = lStats.at<int>(i, cv::CC_STAT_AREA);
		if (area < openK * 6 || bw < openK) continue;
		if (y <= 1 || y + bh >= h - 2) continue;       //the edge of the map is not a feature of the part
		if (bh > openK / 2) continue;                  //thick enough to be a blob rather than a line
		frags.push_back({ lCent.at<double>(i, 0), lCent.at<double>(i, 1) });
	}
	if (frags.size() < 6) return false;

	/*
	* Fit lines through the fragment centres: every pair far enough apart in x proposes a
	* slope, and the proposal with the most fragments on it wins. Exhaustive rather than
	* random - there are only ever a few dozen fragments, so every pair is cheap and the
	* answer does not change from run to run.
	*/
	const double maxSlope = std::tan(maxDeg * CV_PI / 180.0);
	const double minSpan = w * 0.25;
	const double tol = std::max(8.0, h * 0.008);

	std::vector<std::pair<int, double>> found;   //inliers, slope
	for (size_t i = 0; i < frags.size(); i++) {
		for (size_t j = i + 1; j < frags.size(); j++) {
			const double dx = frags[j].x - frags[i].x;
			if (std::fabs(dx) < minSpan) continue;
			const double m = (frags[j].y - frags[i].y) / dx;
			if (std::fabs(m) > maxSlope) continue;
			const double c = frags[i].y - m * frags[i].x;

			int n = 0; double sx = 0, sy = 0, sxx = 0, sxy = 0;
			for (const Frag& f : frags) {
				if (std::fabs(f.y - (m * f.x + c)) > tol) continue;
				n++; sx += f.x; sy += f.y; sxx += f.x * f.x; sxy += f.x * f.y;
			}
			if (n < 4) continue;
			//refit on the fragments that agreed, so the answer is not hostage to the two that
			//happened to propose it
			const double den = n * sxx - sx * sx;
			if (std::fabs(den) < 1e-9) continue;
			found.emplace_back(n, (n * sxy - sx * sy) / den);
		}
	}
	if (found.empty()) return false;

	//the median slope over every line found, weighted by nothing but its own vote: a pin row
	//and a block seam are parallel, so they should agree, and the median says so if they do
	std::sort(found.begin(), found.end(),
		[](const std::pair<int, double>& l, const std::pair<int, double>& r) { return l.first > r.first; });
	const size_t keep = std::min<size_t>(found.size(), 8);
	std::vector<double> slopes;
	for (size_t i = 0; i < keep; i++) slopes.push_back(found[i].second);
	std::nth_element(slopes.begin(), slopes.begin() + slopes.size() / 2, slopes.end());
	thetaDeg = std::atan(slopes[slopes.size() / 2]) * 180.0 / CV_PI;
	return true;
}

/*
* Find the block the scan captured WHOLE, and pose it.
*
* A pogo field is several blocks stacked along the scan direction, each sitting on its own
* plate at its own height. The scan covers one of them completely and clips the ones either
* side, and a part measured off a clipped block is measured off whatever fraction of it
* happened to land in frame.
*
*   1. How far the part is rotated, from the lines of dropout that run along it (above). This
*      has to come first. The boundaries between blocks are parallel to it, so a cut made
*      before the angle is known is a cut at the wrong angle - and on a long block that keeps
*      a wedge of the neighbour at one end while losing a wedge of the block at the other.
*   2. The plate level along each line at that angle: the MODE of the valid heights on it. The
*      mode and not the mean or the median, because the pins are a minority of the area but a
*      long way from the plate, and both of those get dragged by them.
*   3. Median filter, so a line that caught a row of pin tops cannot become a boundary alone.
*   4. The step response: the change in level across a short window. This is what separates a
*      block boundary from the tilt along a plate - on a long block the tilt totals as much as
*      the step does, but it is spread over the whole block and the step is not.
*   5. Cut at each step above the threshold, strongest first, suppressing anything within half
*      a band of a cut already taken, so one boundary yields one cut.
*   6. Of the bands between the cuts, keep the ones that reach neither end of the sweep - a
*      band that runs off an end was cut by the scan. Take the tallest, since a real block is
*      much taller than any sliver two nearby boundaries can enclose.
*   7. Pose it directly from the angle already known, sized to the block's own extent. Not by
*      min-area rect over a contour: the fixture rails that run down either side of the field
*      sit several thousand counts BELOW the plate and reach into the band like everything
*      else there, and a rect fitted around one of those is stretched sideways and square to
*      the image. Anything that far below the local plate is fixture, not part.
*/
static bool h3SegCompleteBand(const cv::Mat& src, const AlgoHeight3Params& p,
	cv::RotatedRect& out, QString& why)
{
	const cv::Mat mask = validMaskOf(src, p.minValidRaw, p.maxValidRaw);
	const int h = src.rows, w = src.cols;
	if (h < 32 || w < 8) { why = QStringLiteral("Map is too small to split into blocks"); return false; }

	cv::Mat src16;
	if (src.type() == CV_16U) src16 = src;
	else src.convertTo(src16, CV_16U);

	//the minimum a band can be, in rows. An eighth of the map by default: big enough that a
	//sliver cannot win, small enough not to rule out a genuinely short block.
	int minBand = (p.segBandMinUm > 0.0 && p.yScaleUmPx > 0.0)
		? (int)std::lround(p.segBandMinUm / p.yScaleUmPx) : h / 8;
	minBand = std::max(8, std::min(minBand, h / 2));

	// ── 1. how far the part is rotated ──
	//5 degrees is already far more than a part can sit out in its fixture; allowing more only
	//lets a chance alignment of unrelated dropouts pass for a pin row
	double thetaDeg = 0.0;
	h3PartRotation(mask, 5.0, thetaDeg);
	const double tanT = std::tan(thetaDeg * CV_PI / 180.0);

	/*
	* ── 2. the plate level along each line at that angle ──
	*
	* The sweep coordinate is u = y - x * tan(theta): lines of constant u are parallel to the
	* part, so a band in u is a band of the PART rather than of the image.
	*
	* Binned over the range the MAP actually occupies, not over the configured valid range.
	* The valid range is usually left wide open (0 .. 65535), and spreading the bins across
	* that would quantise the level to hundreds of counts - the same size as the step being
	* looked for, which would bury it. A real map covers a small part of that span, so
	* binning the span it uses puts the resolution where the signal is.
	*/
	const int kBins = 1024;
	double dataLo = 0.0, dataHi = 0.0;
	cv::minMaxLoc(src16, &dataLo, &dataHi, nullptr, nullptr, mask);
	const double lo = std::max<double>(dataLo, p.minValidRaw);
	const double hi = std::max(lo + 1.0, std::min<double>(dataHi, p.maxValidRaw));
	const double binW = (hi - lo) / kBins;
	if (binW <= 0.0) { why = QStringLiteral("The map has no height variation to split on"); return false; }

	const int uMin = (int)std::floor(std::min(0.0, -(w - 1) * tanT));
	const int uMax = (int)std::ceil((h - 1) - std::min(0.0, (w - 1) * tanT));
	const int uCount = uMax - uMin + 1;
	if (uCount < 32) { why = QStringLiteral("Map is too small to split into blocks"); return false; }

	std::vector<int> hist((size_t)uCount * kBins, 0);
	std::vector<int> total(uCount, 0);
	for (int y = 0; y < h; y++) {
		const ushort* row = src16.ptr<ushort>(y);
		const uchar* mrow = mask.ptr<uchar>(y);
		for (int x = 0; x < w; x++) {
			if (!mrow[x]) continue;
			int ui = (int)std::lround(y - x * tanT) - uMin;
			if (ui < 0) ui = 0; else if (ui >= uCount) ui = uCount - 1;
			int b = (int)((row[x] - lo) / binW);
			if (b < 0) b = 0; else if (b >= kBins) b = kBins - 1;
			hist[(size_t)ui * kBins + b]++;
			total[ui]++;
		}
	}

	int peakTotal = 0;
	for (int v : total) peakTotal = std::max(peakTotal, v);
	if (peakTotal == 0) { why = QStringLiteral("The map contains no valid data"); return false; }

	std::vector<double> level(uCount, 0.0);
	std::vector<char> hasLevel(uCount, 0);
	for (int u = 0; u < uCount; u++) {
		//a line that barely clips the part carries no level; it is held across below rather
		//than allowed to read as a step
		if (total[u] < peakTotal / 5) continue;
		const int* bins = &hist[(size_t)u * kBins];
		const int top = (int)(std::max_element(bins, bins + kBins) - bins);
		level[u] = lo + (top + 0.5) * binW;
		hasLevel[u] = 1;
	}

	double held = 0.0; bool any = false;
	for (int u = 0; u < uCount; u++) {
		if (hasLevel[u]) { held = level[u]; any = true; }
		else if (any) level[u] = held;
	}
	if (!any) { why = QStringLiteral("No line across the part carried enough data to find a plate level"); return false; }
	for (int u = 0; u < uCount && !hasLevel[u]; u++) level[u] = level[std::min(uCount - 1, u + 1)];

	// ── 3. median filter ──
	int k = std::min(51, (minBand | 1));
	if (k % 2 == 0) k++;
	const int half = k / 2;
	std::vector<double> smooth(uCount), window(k);
	for (int u = 0; u < uCount; u++) {
		for (int i = 0; i < k; i++) window[i] = level[std::max(0, std::min(uCount - 1, u - half + i))];
		std::nth_element(window.begin(), window.begin() + half, window.end());
		smooth[u] = window[half];
	}

	// ── 4. the step response ──
	const int W = std::max(4, std::min(25, minBand / 4));
	std::vector<double> resp(uCount, 0.0);
	for (int u = W; u < uCount - W; u++) {
		double aSum = 0.0, bSum = 0.0;
		for (int i = 0; i < W; i++) { aSum += smooth[u + i]; bSum += smooth[u - 1 - i]; }
		resp[u] = (aSum - bSum) / W;
	}

	// ── 5. cuts, strongest first ──
	std::vector<int> order(uCount);
	for (int u = 0; u < uCount; u++) order[u] = u;
	std::sort(order.begin(), order.end(),
		[&](int l, int r) { return std::fabs(resp[l]) > std::fabs(resp[r]); });

	const double thresh = std::max(1.0, p.segBandStepRaw);
	std::vector<char> taken(uCount, 0);
	std::vector<int> cuts;
	for (int u : order) {
		if (std::fabs(resp[u]) < thresh) break;
		bool near = false;
		for (int i = std::max(0, u - minBand / 2); i <= std::min(uCount - 1, u + minBand / 2); i++)
			if (taken[i]) { near = true; break; }
		if (near) continue;
		taken[u] = 1;
		cuts.push_back(u);
	}
	std::sort(cuts.begin(), cuts.end());

	// ── 6. the tallest band that reaches neither end of the sweep ──
	std::vector<int> edges;
	edges.push_back(0);
	for (int c : cuts) edges.push_back(c);
	edges.push_back(uCount);

	int bandU0 = -1, bandU1 = -1;
	for (size_t i = 0; i + 1 < edges.size(); i++) {
		const int a0 = edges[i], a1 = edges[i + 1] - 1;
		if (a0 == 0 || a1 == uCount - 1) continue;        //cut by the scan, not by a plate edge
		if (a1 - a0 + 1 < minBand) continue;
		if (bandU0 < 0 || (a1 - a0) > (bandU1 - bandU0)) { bandU0 = a0; bandU1 = a1; }
	}

	if (bandU0 < 0) {
		why = cuts.size() < 2
			? QStringLiteral("Found %1 block boundary across the map - a complete block needs one "
				"on each side of it. Lower the plate step if the blocks sit at similar heights.")
				.arg(cuts.size())
			: QStringLiteral("Every block runs off one end of the scan, so none of them was "
				"captured whole.");
		return false;
	}

	/*
	* ── 7. pose the block ──
	*
	* The angle is already known and the band is already parallel to the part, so the rect
	* follows from the block's extent along and across that angle - there is nothing left for
	* a min-area rect to work out, and a good deal for it to get wrong.
	*
	* The block is ONE PIECE, and that is what finally settles where it ends. The fixture rails
	* run down either side of the field separated from the plate by a strip the sensor sees
	* nothing of, so they are not connected to it - whereas no height threshold tells them apart
	* reliably, because a rail ramps down from its edge and its first columns read barely more
	* below the plate than a deep pin bore does. The height drop below is only a pre-filter, to
	* keep a rail that touches the plate somewhere from dragging the whole thing in with it; the
	* connectivity is what does the work.
	*
	* Ten times the plate step as that drop. It has to clear the pins' own bores while staying
	* well above them a rail does, and on real parts those two are orders apart.
	*/
	const double railDrop = 10.0 * thresh;
	const double cosT = std::cos(thetaDeg * CV_PI / 180.0);
	const double sinT = std::sin(thetaDeg * CV_PI / 180.0);

	cv::Mat blockMask(h, w, CV_8U, cv::Scalar(0));
	for (int y = 0; y < h; y++) {
		const ushort* row = src16.ptr<ushort>(y);
		const uchar* mrow = mask.ptr<uchar>(y);
		uchar* brow = blockMask.ptr<uchar>(y);
		for (int x = 0; x < w; x++) {
			if (!mrow[x]) continue;
			const int ui = (int)std::lround(y - x * tanT) - uMin;
			if (ui < bandU0 || ui > bandU1) continue;
			if (row[x] < smooth[ui] - railDrop) continue;
			brow[x] = 255;
		}
	}

	/*
	* Close before labelling: the pins shadow the plate into a lace of small holes, and a few of
	* those run together into a channel that would cut a corner of the plate off from the rest.
	* The kernel is sized to bridge a pin's shadow and no more, so it cannot reach across the
	* strip that separates the plate from a rail.
	*/
	const int bridge = std::max(3, std::min(41, w / 400) | 1);
	cv::Mat joined;
	cv::morphologyEx(blockMask, joined, cv::MORPH_CLOSE,
		cv::getStructuringElement(cv::MORPH_RECT, cv::Size(bridge, bridge)));

	cv::Mat bLabels, bStats, bCent;
	const int nBlocks = cv::connectedComponentsWithStats(joined, bLabels, bStats, bCent, 8, CV_32S);
	if (nBlocks < 2) {
		why = QStringLiteral("The complete block contains no surface to pose");
		return false;
	}
	int biggest = 1;
	for (int i = 2; i < nBlocks; i++)
		if (bStats.at<int>(i, cv::CC_STAT_AREA) > bStats.at<int>(biggest, cv::CC_STAT_AREA)) biggest = i;

	std::vector<float> along, across;
	along.reserve(1 << 16);
	across.reserve(1 << 16);
	for (int y = 0; y < h; y++) {
		const int* lrow = bLabels.ptr<int>(y);
		const uchar* brow = blockMask.ptr<uchar>(y);
		for (int x = 0; x < w; x++) {
			//the label comes from the closed mask, the membership from the original: the closing
			//is there to join the plate up, not to grow it over the gap around it
			if (!brow[x] || lrow[x] != biggest) continue;
			along.push_back((float)(x * cosT + y * sinT));
			across.push_back((float)(-x * sinT + y * cosT));
		}
	}
	if (along.size() < 64) {
		why = QStringLiteral("The complete block contains too little surface to pose");
		return false;
	}

	//a fiftieth of a percent off each end: enough that a thin line of stragglers cannot stretch
	//the frame, little enough that it never eats into the block itself
	auto pct = [](std::vector<float>& v, double q) {
		size_t i = (size_t)std::lround(q * (v.size() - 1));
		i = std::min(i, v.size() - 1);
		std::nth_element(v.begin(), v.begin() + i, v.end());
		return (double)v[i];
	};
	const double s0 = pct(along, 0.0002), s1 = pct(along, 0.9998);
	const double n0 = pct(across, 0.0002), n1 = pct(across, 0.9998);
	if (s1 - s0 < 2.0 || n1 - n0 < 2.0) {
		why = QStringLiteral("The complete block measured smaller than 2 px");
		return false;
	}

	const double cs = 0.5 * (s0 + s1), cn = 0.5 * (n0 + n1);
	out = cv::RotatedRect(
		cv::Point2f((float)(cs * cosT - cn * sinT), (float)(cs * sinT + cn * cosT)),
		cv::Size2f((float)(s1 - s0), (float)(n1 - n0)),
		(float)thetaDeg);
	return true;
}
} //namespace

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

	//── which method finds the part ──
	cv::RotatedRect rr;
	QString segWhy;
	switch (p.segMethod) {
	case AlgoH3SegMethod::LargestRegion:
		if (!h3SegLargestRegion(src, p, rr, segWhy)) return fail(segWhy);
		break;
	case AlgoH3SegMethod::CompleteBand:
		if (!h3SegCompleteBand(src, p, rr, segWhy)) return fail(segWhy);
		break;
	default:
		//loadRecipeConfig clamps seg_method, so this can only fire if an enum value was
		//added without its case here - say so rather than silently running method 0
		return fail(QStringLiteral("Unknown segmentation method %1").arg((int)p.segMethod));
	}

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

	if (sz.width < 2.0f || sz.height < 2.0f)
		return fail(QStringLiteral("Segmented part is smaller than 2 px"));

	/*
	* MEASURE AND PUBLISH BEFORE REFUSING ANYTHING.
	*
	* The canvas below is mandatory, and the only way to choose one is to know how big the
	* part actually is - so a stage that bailed out before filling these in would leave a
	* fresh recipe with no way to find out. Measuring first costs nothing and breaks that
	* circle: run once, read the size off Measured Width/Height or out of the message, type
	* a round number comfortably above it.
	*/
	m_out.segWidthUm = sz.width * p.xScaleUmPx;
	m_out.segHeightUm = sz.height * p.yScaleUmPx;
	m_out.segAngleDeg = ang;

	{
		cv::Point2f pts[4];
		rr.points(pts);
		m_out.segCorners.clear();
		for (int i = 0; i < 4; i++) m_out.segCorners.append(QPointF(pts[i].x, pts[i].y));
		const cv::Rect br = rr.boundingRect();
		m_out.segRectMap = QRectF(br.x, br.y, br.width, br.height);
	}

	/*
	* ── the FIXED canvas ──
	*
	* The part frame is a constant of the recipe, not a property of this particular unit.
	* That is the whole point: a taught ROI has to mean the same thing on every part, and it
	* cannot if the frame is sized to whatever this one happened to measure.
	*/
	double canvasWidthUm = p.segCanvasWidthUm;
	double canvasHeightUm = p.segCanvasHeightUm;

	/*
	* CompleteBand sizes the frame to the block it found, and the page hides the canvas boxes
	* for it. The block is bounded by its own plate edges rather than by whatever the scan
	* happened to include, so its extent is a property of the PART and repeats unit to unit -
	* which is the thing the fixed canvas exists to guarantee. It is not quite as strong a
	* guarantee: a block that measures a little differently still moves the frame a little,
	* where a typed canvas cannot move at all. Set a canvas anyway and it is honoured.
	*/
	if (p.segMethod == AlgoH3SegMethod::CompleteBand
		&& (canvasWidthUm <= 0.0 || canvasHeightUm <= 0.0)) {
		canvasWidthUm = m_out.segWidthUm;
		canvasHeightUm = m_out.segHeightUm;
	}

	if (canvasWidthUm <= 0.0 || canvasHeightUm <= 0.0) {
		return fail(QStringLiteral(
			"Canvas size is not set. This part measured %1 x %2 um - set a canvas "
			"comfortably larger than the biggest part the line will see.")
			.arg(m_out.segWidthUm, 0, 'f', 1).arg(m_out.segHeightUm, 0, 'f', 1));
	}

	//round UP to an even number of pixels so the centre lands on a pixel boundary rather
	//than half way across one. Enforced here and not in the spin box: the box holds um, and
	//the scale conversion breaks any relationship between an even um value and an even pixel
	//count (50002 um at 5 um/px is 10000 px, 50000 um is 10000 px too).
	auto toEvenPx = [](double um, double umPerPx) {
		int px = (int)std::lround(um / umPerPx);
		if (px < 2) px = 2;
		if (px % 2) px++;
		return px;
	};
	const int cropW = toEvenPx(canvasWidthUm, p.xScaleUmPx);
	const int cropH = toEvenPx(canvasHeightUm, p.yScaleUmPx);

	//a corrupt map or an absurd canvas could ask for an allocation that will not fit
	if ((qint64)cropW * (qint64)cropH > 400000000LL)
		return fail(QStringLiteral("Canvas is unreasonably large (%1 x %2 px)")
			.arg(cropW).arg(cropH));

	m_out.cropWidthPx = cropW;
	m_out.cropHeightPx = cropH;

	/*
	* Bigger than the canvas: the outer part is cropped away. Reported rather than silent -
	* a part that has outgrown its frame is a real anomaly (a double unit, a mis-load, a
	* mis-segmentation), and it must not look identical to a good one in the log. It does not
	* fail the unit on its own; the width/height checks below are what decide that.
	*/
	if (sz.width > cropW + 0.5f || sz.height > cropH + 0.5f) {
		m_out.segOversized = true;
		res.note = QStringLiteral(
			"Part %1 x %2 um is larger than the %3 x %4 um canvas - the outer part was cropped")
			.arg(m_out.segWidthUm, 0, 'f', 1).arg(m_out.segHeightUm, 0, 'f', 1)
			.arg(canvasWidthUm, 0, 'f', 1).arg(canvasHeightUm, 0, 'f', 1);
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
	* Build the straightened crop in ONE warp - rotate about the part centre and land that
	* centre at the centre of the canvas - so a 60 MB map is never rotated whole.
	*
	* The centring is what the two translation terms already did; they now target a FIXED
	* canvas instead of one sized to this part. Anything the part does not cover is filled
	* with BORDER_CONSTANT 0, and 0 is already "dropout" to the mask, the plane fit and the
	* measurement - so the letterboxing needs no special handling anywhere downstream.
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

/*
* Fit the datum to the part's own flat surface, with no ROIs to place.
*
* Placing datum ROIs by hand is the slow and fragile part of teaching this page: the operator
* has to find patches of bare substrate between the pins, and a patch that was bare on the
* unit it was taught on may have a pin shadow across it on the next. This finds them instead,
* and it finds ALL of them - on the sample part that is 4.96 million points spread over the
* whole plate, against 0.9 million in six hand-placed boxes.
*
* Flatness alone is not enough, because a pin cap is flat too. What distinguishes the datum is
* that it is the flat surface the rest of the part STANDS ON, which is to say the dominant one.
* So:
*
*   1. Split the crop into tiles and measure each one's spread - the 10th to 90th percentile of
*      its heights, which ignores the few stray samples a peak-to-peak would be set by. A tile
*      with too little data to judge is skipped rather than guessed at.
*   2. Keep the tiles flatter than the tolerance. Both the plate and the pin caps survive this.
*   3. Fit a plane to those tiles and throw out the ones that sit too far off it, repeatedly.
*      The plate is the biggest population, so it is what the fit settles on, and the caps -
*      which stand well above it - drop out within a couple of rounds. The cut is 2.5 MAD
*      rather than a fixed band, so a rough plate widens its own tolerance instead of
*      discarding itself, with a floor so a mirror-flat one does not close down to nothing.
*   4. Re-fit on every PIXEL within that band, not just the surviving tile centres. The tiles
*      found the surface; the pixels measure it, and there are three orders more of them.
*
* The result on the sample part is a plane with 4.4 um RMS where six hand-placed ROIs gave
* 37.6 um - not because the fit is better arithmetic, but because it is fitted to the whole
* plate rather than to three edges of it.
*
* roiMask confines the search. Empty, it reads the whole part, which is the method at its most
* automatic and needs nothing taught. Given the datum ROIs, it reads only inside them - the
* operator says WHERE the datum is and the method still decides WHICH pixels of it are flat
* enough to fit. That matters on a part where the substrate is not the largest flat thing in
* frame: left to itself the fit lands on whatever has the most area, and it reports a TIGHTER
* RMS while doing it, so a wrong surface looks more convincing rather than less. A box round
* the right surface removes the question.
*/
static bool h3FitDatumAutoFlat(const cv::Mat& crop16, const cv::Mat& roiMask,
	const AlgoHeight3Params& p, H3Plane& out, qint64& points, double& rmsUm,
	int& tilesUsed, QString& why)
{
	const int w = crop16.cols, h = crop16.rows;
	if (w < 64 || h < 64) { why = QStringLiteral("The segmented part is too small to find flat regions in"); return false; }

	//A tile has to be big enough for its spread to mean something and small enough that the
	//plate and a pin do not share one. Sixty-four across the short side puts it at 32 px on the
	//sample part, which is a fraction of a pin.
	int T = std::min(w, h) / 64;
	T = std::max(8, std::min(64, T));
	const int tw = w / T, th = h / T;
	if (tw < 4 || th < 4) { why = QStringLiteral("The segmented part is too small to find flat regions in"); return false; }

	const double zPerUm = p.zScaleRawPerUm;      //raw grey levels per um
	const double flatRaw = std::max(1.0, p.datumFlatnessUm) * zPerUm;

	struct Tile { double x, y, z; };
	std::vector<Tile> tiles;
	tiles.reserve((size_t)tw * th);

	const bool haveRoi = !roiMask.empty();

	std::vector<ushort> vals;
	vals.reserve((size_t)T * T);

	for (int tj = 0; tj < th; tj++) {
		for (int ti = 0; ti < tw; ti++) {
			vals.clear();
			int inRoi = 0;
			for (int y = tj * T; y < (tj + 1) * T; y++) {
				const ushort* row = crop16.ptr<ushort>(y);
				const uchar* mrow = haveRoi ? roiMask.ptr<uchar>(y) : nullptr;
				for (int x = ti * T; x < (ti + 1) * T; x++) {
					if (mrow && !mrow[x]) continue;
					inRoi++;
					const ushort z = row[x];
					if (z < p.minValidRaw || z > p.maxValidRaw) continue;
					vals.push_back(z);
				}
			}
			//A tile only half inside an ROI describes the edge of the box as much as the
			//surface, so it is skipped rather than allowed to vote.
			if (inRoi < T * T / 2) continue;
			//and half of what IS in the box has to be measured; below that the spread says
			//more about the dropouts than about the surface
			if ((int)vals.size() < inRoi / 2) continue;

			const size_t lo = vals.size() / 10, mid = vals.size() / 2;
			const size_t hiIdx = vals.size() - 1 - vals.size() / 10;
			std::nth_element(vals.begin(), vals.begin() + lo, vals.end());
			const double p10 = vals[lo];
			std::nth_element(vals.begin() + lo + 1, vals.begin() + hiIdx, vals.end());
			const double p90 = vals[hiIdx];
			if (p90 - p10 > flatRaw) continue;        //not flat: a pin flank, an edge, a bore

			std::nth_element(vals.begin(), vals.begin() + mid, vals.end());
			tiles.push_back({ ti * T + T / 2.0, tj * T + T / 2.0, (double)vals[mid] });
		}
	}

	if (tiles.size() < 16) {
		why = haveRoi
			? QStringLiteral("Only %1 flat region(s) found inside the datum ROIs - raise "
				"Flatness, or make the ROIs larger.").arg(tiles.size())
			: QStringLiteral("Only %1 flat region(s) found - raise Flatness, or check that the "
				"part was segmented correctly.").arg(tiles.size());
		return false;
	}

	// ── settle on the dominant flat surface ──
	std::vector<char> keep(tiles.size(), 1);
	H3Plane plane;
	double bandUm = 0.0;

	for (int iter = 0; iter < 8; iter++) {
		std::vector<H3Point> pts;
		pts.reserve(tiles.size());
		for (size_t i = 0; i < tiles.size(); i++)
			if (keep[i]) pts.push_back({ tiles[i].x, tiles[i].y, tiles[i].z });
		if (pts.size() < 16) break;

		plane = fitPlaneLeastSquares(pts);
		if (!plane.valid) { why = QStringLiteral("Plane fit failed on the flat regions"); return false; }

		//residuals of EVERY candidate against the current plane, so a tile thrown out in one
		//round can come back in the next if the plane moved towards it
		std::vector<double> resid(tiles.size());
		std::vector<double> kept;
		kept.reserve(pts.size());
		for (size_t i = 0; i < tiles.size(); i++) {
			resid[i] = (tiles[i].z - planeZ(plane, tiles[i].x, tiles[i].y)) / zPerUm;
			if (keep[i]) kept.push_back(resid[i]);
		}

		std::nth_element(kept.begin(), kept.begin() + kept.size() / 2, kept.end());
		const double centre = kept[kept.size() / 2];
		for (double& k : kept) k = std::fabs(k - centre);
		std::nth_element(kept.begin(), kept.begin() + kept.size() / 2, kept.end());
		const double mad = 1.4826 * kept[kept.size() / 2];

		//a floor under the band, so a plate that is flat to a micron does not keep tightening
		//until it has discarded itself
		bandUm = std::max(5.0, 2.5 * mad);

		std::vector<char> next(tiles.size(), 0);
		int n = 0;
		for (size_t i = 0; i < tiles.size(); i++)
			if (std::fabs(resid[i] - centre) < bandUm) { next[i] = 1; n++; }
		if (n < 16) break;
		const bool same = (next == keep);
		keep.swap(next);
		if (same) break;
	}

	tilesUsed = 0;
	for (char k : keep) if (k) tilesUsed++;
	if (tilesUsed < 16 || !plane.valid) {
		why = QStringLiteral("The flat regions did not settle on one surface - try a tighter Flatness.");
		return false;
	}

	/*
	* ── re-fit on the pixels ──
	*
	* Accumulated straight into the normal equations rather than collected into a vector: on the
	* sample part this band holds five million points, and a std::vector of them is 120 MB spent
	* to compute nine sums. Coordinates are taken relative to the crop centre so the sums stay
	* well inside double precision.
	*/
	const double cx = w / 2.0, cy = h / 2.0;
	double sxx = 0, sxy = 0, sx = 0, syy = 0, sy = 0, s1 = 0, sxz = 0, syz = 0, sz = 0;

	for (int y = 0; y < h; y++) {
		const ushort* row = crop16.ptr<ushort>(y);
		const uchar* mrow = haveRoi ? roiMask.ptr<uchar>(y) : nullptr;
		const double Y = y - cy;
		for (int x = 0; x < w; x++) {
			if (mrow && !mrow[x]) continue;
			const ushort z = row[x];
			if (z < p.minValidRaw || z > p.maxValidRaw) continue;
			if (std::fabs((z - planeZ(plane, x, y)) / zPerUm) >= bandUm) continue;
			const double X = x - cx, Z = z;
			sxx += X * X; sxy += X * Y; sx += X;
			syy += Y * Y; sy += Y; s1 += 1.0;
			sxz += X * Z; syz += Y * Z; sz += Z;
		}
	}
	if (s1 < 1000) {
		why = QStringLiteral("Too few points on the flat surface to fit a plane");
		return false;
	}

	double ATA[3][3] = { { sxx, sxy, sx }, { sxy, syy, sy }, { sx, sy, s1 } };
	double ATb[3] = { sxz, syz, sz };
	double sol[3];
	if (!solve3x3(ATA, ATb, sol)) {
		why = QStringLiteral("Plane fit failed - the flat surface is degenerate");
		return false;
	}

	out.a = sol[0];
	out.b = sol[1];
	out.c = sol[2] - sol[0] * cx - sol[1] * cy;    //back to absolute coordinates
	out.valid = true;

	double sumSq = 0.0;
	for (int y = 0; y < h; y++) {
		const ushort* row = crop16.ptr<ushort>(y);
		const uchar* mrow = haveRoi ? roiMask.ptr<uchar>(y) : nullptr;
		for (int x = 0; x < w; x++) {
			if (mrow && !mrow[x]) continue;
			const ushort z = row[x];
			if (z < p.minValidRaw || z > p.maxValidRaw) continue;
			if (std::fabs((z - planeZ(plane, x, y)) / zPerUm) >= bandUm) continue;
			const double d = (z - planeZ(out, x, y)) / zPerUm;
			sumSq += d * d;
		}
	}
	points = (qint64)s1;
	rmsUm = std::sqrt(sumSq / s1);
	return true;
}

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
	//AutoFlat finds its own surface, so an ROI is not merely unnecessary there - demanding one
	//would be asking for the thing the method exists to avoid
	const bool autoFlat = (p.datumMethod == AlgoH3DatumMethod::AutoFlat);
	if (!autoFlat && p.datumRois.isEmpty())
		return fail(QStringLiteral("Add at least one datum ROI"));
	if (p.zScaleRawPerUm <= 0.0) return fail(QStringLiteral("Z scale must be greater than 0"));
	//the tilt and the PCA/SVD fit are both worked out in um, so they need the XY scale too
	if (p.xScaleUmPx <= 0.0 || p.yScaleUmPx <= 0.0)
		return fail(QStringLiteral("X and Y scale must be greater than 0"));

	const int w = m_cropHeight.cols, h = m_cropHeight.rows;
	const cv::Rect bounds(0, 0, w, h);

	H3Plane autoPlane;
	qint64 autoPoints = 0;
	double autoRmsUm = 0.0;
	int autoTiles = 0;
	if (autoFlat) {
		/*
		* Every datum ROI, unioned into ONE mask and fitted as ONE plane.
		*
		* Not a plane per box averaged afterwards: three small boxes in a line would each fit
		* their own patch almost perfectly and say nothing about the tilt between them, where
		* the pooled fit is constrained by how far apart they are. Pooling is also what makes
		* boxes of very different sizes behave sensibly - a large one simply contributes more
		* points, which is the right weighting for a surface sampled unevenly.
		*
		* No ROIs at all leaves the mask empty, and the method reads the whole part.
		*/
		cv::Mat roiMask;
		int roisInside = 0;
		if (!p.datumRois.isEmpty()) {
			roiMask = cv::Mat::zeros(h, w, CV_8U);
			for (const auto& rel : p.datumRois) {
				const QRectF abs = rel.translated(w / 2.0, h / 2.0);
				cv::Rect r((int)std::floor(abs.left()), (int)std::floor(abs.top()),
					(int)std::lround(abs.width()), (int)std::lround(abs.height()));
				r &= bounds;
				if (r.width <= 0 || r.height <= 0) continue;
				roiMask(r).setTo(255);
				roisInside++;
			}
			if (roisInside == 0)
				return fail(QStringLiteral("Every datum ROI is outside the segmented image"));
		}

		QString autoWhy;
		if (!h3FitDatumAutoFlat(m_cropHeight, roiMask, p, autoPlane, autoPoints,
				autoRmsUm, autoTiles, autoWhy))
			return fail(autoWhy);

		//which it used is not obvious from the numbers, and a stale ROI quietly narrowing the
		//fit is exactly the kind of thing that should be on the record
		res.note = roisInside > 0
			? QStringLiteral("%1 flat regions in %2 datum ROI(s), %3 points")
				.arg(autoTiles).arg(roisInside).arg(autoPoints)
			: QStringLiteral("%1 flat regions across the whole part, %2 points")
				.arg(autoTiles).arg(autoPoints);
	}

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

	if (!autoFlat) {
		if (usedRois == 0) return fail(QStringLiteral("Every datum ROI is outside the segmented image"));
		if (pts.size() < 3) return fail(QStringLiteral("Fewer than 3 valid points in the datum ROIs"));
	}

	const H3Plane plane = autoFlat ? autoPlane
		: ((p.datumMethod == AlgoH3DatumMethod::PcaSvd)
			? fitPlanePcaSvd(pts, p.xScaleUmPx, p.yScaleUmPx, 1.0 / p.zScaleRawPerUm)
			: fitPlaneLeastSquares(pts));

	if (!plane.valid)
		return fail(QStringLiteral("Plane fit failed - the datum points are degenerate"));

	m_out.planeValid = true;
	m_out.planeA = plane.a;
	m_out.planeB = plane.b;
	m_out.planeC = plane.c;
	m_out.datumPoints = autoFlat ? autoPoints : (qint64)pts.size();

	/*
	* Absolute angle between the fitted plane and the map plane; a tilt has no sign that
	* matters here, so the page asks for a maximum only.
	*
	* From the PHYSICAL slopes. a and b are raw grey levels per pixel, and a grey level is
	* 1/zScale um while a pixel is x/yScale um, so each is taken to um per um first. Used
	* raw, they treated a 0.8 um grey level and a 5 um pixel as the same length and read
	* 6.25x too steep: 16.85 deg on 20260828_193223, whose substrate an independent fit of the
	* raw map puts at 2.78 deg (a 2.4 mm fall over 50 mm; 16.85 deg would need ~15 mm, and the
	* whole map spans only ~6 mm of height).
	*/
	const double slopeX = plane.a / (p.zScaleRawPerUm * p.xScaleUmPx);
	const double slopeY = plane.b / (p.zScaleRawPerUm * p.yScaleUmPx);
	m_out.planeTiltDeg = qRadiansToDegrees(std::atan(std::sqrt(slopeX * slopeX + slopeY * slopeY)));

	if (autoFlat) {
		m_out.planeRmsUm = autoRmsUm;
	}
	else {
		double sumSq = 0.0;
		for (const auto& pt : pts) {
			const double d = (pt.z - planeZ(plane, pt.x, pt.y)) / p.zScaleRawPerUm;
			sumSq += d * d;
		}
		m_out.planeRmsUm = std::sqrt(sumSq / (double)pts.size());
	}

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

		//an unchecked type simply reports its height without judging it
		if (!type.checkHeight) {
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

	/*
	* The XY offset criterion exists in the recipe but nothing measures an offset yet. Say so
	* loudly: a check the operator has ticked and which quietly does nothing is the worst
	* kind of dead setting, because the recipe claims a guarantee the machine is not giving.
	* Remove this the moment doMeasure learns to compute an offset.
	*/
	{
		QStringList pending;
		for (const auto& t : p.roiTypes)
			if (t.checkOffset) pending << t.name;
		if (!pending.isEmpty()) {
			res.note = QStringLiteral(
				"XY offset check is enabled on %1 but offset measurement is not implemented "
				"yet - that criterion is NOT being applied").arg(pending.join(", "));
		}
	}

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
	//the datum sheet: painted translucent and unstroked, so the part shows through it
	bool plane = false;
};

/*
* A fixed "headlight" for the shaded mesh: upper-left and angled toward the viewer.
* It lives in VIEW space, not model space, so the light stays put as the part is spun -
* which is what lets a drag read as rotating a lit object rather than a sliding texture.
* Unit length (sum of squares = 1.00004), so the diffuse dot product needs no rescale.
* Screen y grows DOWNWARD, hence the negative y for a light coming from above.
*/
const double kMeshLightX = -0.3990;
const double kMeshLightY = -0.6484;
const double kMeshLightZ = 0.6484;
const double kMeshAmbient = 0.32;   //floor, so a facet facing away is dim but never black

/*
* Close the shadows the pins cast on each other, on the display grid, before anything is
* drawn from it.
*
* A pin blocks the laser from reaching part of its neighbour, so the neighbour comes back
* with no height exactly where it is most crowded. A quad with an unmeasured corner cannot
* be drawn, so every one of those shadows is a bite taken out of a pin in the 3D view - and
* on a dense field that is most of what the operator is trying to look at.
*
* Only ENCLOSED gaps are filled, which is what separates a pin's own shadow from the space
* around the pins: see AlgoH3HoleFill.h. Filled heights are clamped back into the valid band
* so validMaskOf() accepts them - over-relaxation can overshoot a rim by a hair, and a value
* one count outside the band would come straight back as a dropout.
*
* Display only. Nothing is measured from this grid - the pipeline measures the full-resolution
* map, which this never touches.
*/
h3fill::Report algoH3FillDisplayGrid(cv::Mat& grid16, int minValidRaw, int maxValidRaw)
{
	if (grid16.empty() || grid16.type() != CV_16U) return h3fill::Report();

	const int w = grid16.cols, h = grid16.rows;
	const cv::Mat mask = validMaskOf(grid16, minValidRaw, maxValidRaw);

	std::vector<float> z((size_t)w * h, h3fill::kNaN);
	for (int j = 0; j < h; j++) {
		const ushort* row = grid16.ptr<ushort>(j);
		const uchar* mrow = mask.ptr<uchar>(j);
		for (int i = 0; i < w; i++) if (mrow[i]) z[(size_t)j * w + i] = (float)row[i];
	}

	h3fill::Params prm;
	prm.mode = h3fill::Mode::Interpolate;
	prm.tol = 0.5f;          //raw counts: half a count is already below anything that can be seen
	const h3fill::Report rep = h3fill::algoH3FillHoles(z, w, h, prm);
	if (rep.filledCells == 0) return rep;

	for (int j = 0; j < h; j++) {
		ushort* row = grid16.ptr<ushort>(j);
		const uchar* mrow = mask.ptr<uchar>(j);
		for (int i = 0; i < w; i++) {
			if (mrow[i]) continue;                    //already measured, leave it alone
			const float v = z[(size_t)j * w + i];
			if (!h3fill::valid(v)) continue;          //background, still a gap
			row[i] = (ushort)std::lround(std::max((double)minValidRaw,
				std::min((double)maxValidRaw, (double)v)));
		}
	}
	return rep;
}

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
QImage algoH3RenderSurface3D(const cv::Mat& height16, const cv::Mat& intensity8,
	int minValidRaw, int maxValidRaw,
	double yawDeg, double pitchDeg, double zExaggeration, const QSize& outSize,
	AlgoH3SurfaceStyle style, bool fillHoles, const double* planeABC)
{
	const bool mesh = (style == AlgoH3SurfaceStyle::ShadedMesh);
	const bool wire = (style == AlgoH3SurfaceStyle::Wireframe);
	const bool points = (style == AlgoH3SurfaceStyle::PointCloud);
	const bool textured = (style == AlgoH3SurfaceStyle::Textured);
	//every style except the original flat one is lit by the facet normal
	const bool lit = (style != AlgoH3SurfaceStyle::Filled && !wire && !points);

	const QSize size = (outSize.width() >= 64 && outSize.height() >= 64) ? outSize : QSize(900, 700);

	QImage img(size, QImage::Format_RGB888);
	img.fill(QColor(24, 26, 32));
	if (height16.empty()) return img;

	cv::Mat src;
	if (height16.type() == CV_16U) src = height16;
	else height16.convertTo(src, CV_16U);

	/*
	* Downsample to a grid we can still rotate interactively. Budget per style, because they
	* cost very different amounts per cell: at the CPU filled view's 150 a 1.27 mm pin on a
	* 10000 px map is under 4 cells wide and simply is not there, but a wireframe at 400 is an
	* unreadable ball of lines. Points are the cheapest thing to draw, so they get the most.
	*
	* The GPU column is several times denser, because those numbers were set by what a QPainter
	* could sort and fill per drag frame and a depth-buffered mesh has no such limit. Resolving
	* a pin at all is the whole point of this view, so that headroom goes straight into cells.
	* The wireframe barely moves either way: a dense hidden-line lattice is unreadable no matter
	* who draws it.
	*/
	const bool gpu = algoH3GLAvailable();
	const int kGridBudget = gpu
		? ((style == AlgoH3SurfaceStyle::Filled)       ?  800 :
		   (style == AlgoH3SurfaceStyle::ShadedMesh)   ?  800 :
		   (style == AlgoH3SurfaceStyle::SmoothShaded) ? 1100 :
		   (style == AlgoH3SurfaceStyle::Wireframe)    ?  320 :
		   (style == AlgoH3SurfaceStyle::PointCloud)   ? 1100 :
		   /* Textured */                                1100)
		: ((style == AlgoH3SurfaceStyle::Filled)       ?  150 :
		   (style == AlgoH3SurfaceStyle::ShadedMesh)   ?  240 :
		   (style == AlgoH3SurfaceStyle::SmoothShaded) ?  420 :
		   (style == AlgoH3SurfaceStyle::Wireframe)    ?  170 :
		   (style == AlgoH3SurfaceStyle::PointCloud)   ?  380 :
		   /* Textured */                                 420);
	const double shrink = std::min(1.0,
		(double)kGridBudget / (double)std::max(src.cols, src.rows));
	const int gw = std::max(2, (int)std::lround(src.cols * shrink));
	const int gh = std::max(2, (int)std::lround(src.rows * shrink));

	cv::Mat grid;
	cv::resize(src, grid, cv::Size(gw, gh), 0, 0, cv::INTER_NEAREST);

	//Before anything asks which cells are valid: reconstruct the pin surface the neighbouring
	//pins shadowed. Without this a quad with one unmeasured corner is skipped, and the pins come
	//out of every 3D view with bites taken out of them.
	//
	//The operator can turn it off to see exactly what the profiler returned - which is the
	//honest view when the question is how well the part was staged rather than what shape it is.
	if (fillHoles) algoH3FillDisplayGrid(grid, minValidRaw, maxValidRaw);

	//the intensity texture rides the SAME grid, so a cell's colour and its geometry come
	//from the same place on the part. INTER_AREA here, not NEAREST: this one is a picture,
	//not a measurement, so averaging is what we want when shrinking it hard.
	cv::Mat gtex;
	const bool haveTex = textured && !intensity8.empty()
		&& intensity8.size() == height16.size();
	if (haveTex) {
		cv::Mat tex8;
		if (intensity8.type() == CV_8U) tex8 = intensity8;
		else intensity8.convertTo(tex8, CV_8U);
		cv::resize(tex8, gtex, cv::Size(gw, gh), 0, 0, cv::INTER_AREA);
	}

	const cv::Mat gmask = validMaskOf(grid, minValidRaw, maxValidRaw);
	double zMin = 0, zMax = 0;
	if (cv::countNonZero(gmask) == 0) return img;
	cv::minMaxLoc(grid, &zMin, &zMax, nullptr, nullptr, gmask);
	if (zMax <= zMin) zMax = zMin + 1.0;

	/*
	* The GPU path. Everything below it is the CPU fallback and stays reachable: a machine PC
	* with no usable desktop GL driver still gets a picture rather than a black canvas.
	*
	* The two share this grid, this validity mask and this colour band, and project the same
	* way, so switching between them does not move the part on the canvas or change its colour.
	* What differs is that the GPU resolves occlusion with a depth buffer instead of sorting
	* quads far-to-near - and on a pin field the painter's algorithm genuinely fails, because
	* two pins at a similar distance interleave in depth and flicker through each other as the
	* view is spun.
	*/
	if (gpu) {
		AlgoH3GLScene scene;
		scene.w = gw;
		scene.h = gh;
		scene.zMin = zMin;
		scene.zMax = zMax;
		scene.ramp = reinterpret_cast<const unsigned int*>(jetTable());
		scene.z.resize((size_t)gw * gh, h3fill::kNaN);
		for (int j = 0; j < gh; j++) {
			const ushort* row = grid.ptr<ushort>(j);
			const uchar* mrow = gmask.ptr<uchar>(j);
			for (int i = 0; i < gw; i++) if (mrow[i]) scene.z[(size_t)j * gw + i] = (float)row[i];
		}
		if (haveTex) {
			scene.tex.resize((size_t)gw * gh);
			for (int j = 0; j < gh; j++) {
				const uchar* trow = gtex.ptr<uchar>(j);
				for (int i = 0; i < gw; i++) scene.tex[(size_t)j * gw + i] = trow[i];
			}
		}

		if (planeABC) {
			//the grid is a resize of the map, so a grid node at (i, j) is map pixel
			//(i * cols/gw, j * rows/gh) - evaluate the plane there and let the quad interpolate
			const double sx = (double)src.cols / gw, sy = (double)src.rows / gh;
			auto at = [&](int i, int j) {
				return (float)(planeABC[0] * i * sx + planeABC[1] * j * sy + planeABC[2]);
			};
			scene.hasPlane = true;
			scene.planeCorner[0] = at(0, 0);
			scene.planeCorner[1] = at(gw - 1, 0);
			scene.planeCorner[2] = at(gw - 1, gh - 1);
			scene.planeCorner[3] = at(0, gh - 1);
		}

		const QImage rendered = algoH3RenderSurfaceGL(scene, yawDeg, pitchDeg,
			zExaggeration, size, style);
		if (!rendered.isNull()) return rendered;
	}

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

	/*
	* The same projection the grid went through, for a point that is NOT on the grid - which is
	* what the datum sheet needs. Taking the sheet through the identical arithmetic is what
	* keeps it in register with the part; working it out a second way would leave it a pixel or
	* two out at every angle.
	*/
	auto project = [&](double gi, double gj, double rawZ, double& ox, double& oy, double& depth) {
		const double t = (rawZ - zMin) / (zMax - zMin);
		const double x = (2.0 * gi / (double)(gw - 1) - 1.0) * ax;
		const double y = (2.0 * gj / (double)(gh - 1) - 1.0) * ay;
		const double z = (t - 0.5) * 2.0 * zSpan;
		const double xr = x * cy - y * sy;
		const double yr = x * sy + y * cy;
		depth = yr * ce + z * se;
		ox = offX + (xr - minX) * s;
		oy = offY + (yr * se - z * ce - minY) * s;
	};

	const QRgb* lut = jetTable();

	/*
	* Point cloud: no surface at all, one dot per surviving sample. It is the only style
	* that shows the data the way the profiler actually delivers it - every dropout is a
	* visible gap rather than a quad that was quietly skipped - and it is the only one that
	* stays honest where the surface is too broken to triangulate. Dots are drawn far-first
	* so nearer samples cover farther ones, which is all the occlusion a cloud needs.
	*/
	if (points) {
		struct H3Dot { QPointF at; double depth; QRgb color; };
		std::vector<H3Dot> dots;
		dots.reserve((size_t)gw * gh);

		for (size_t idx = 0; idx < (size_t)gw * gh; idx++) {
			if (!ok[idx]) continue;
			dots.push_back({ toScreen(idx), pd[idx], lut[shade[idx]] });
		}
		if (dots.empty()) return img;

		std::sort(dots.begin(), dots.end(),
			[](const H3Dot& l, const H3Dot& r) { return l.depth < r.depth; });

		//size the dot to the cell pitch so the cloud reads as a surface when dense and as
		//separate samples when sparse, instead of always being a fixed speck
		const double pitch = s * 2.0 * ax / std::max(1, gw - 1);
		const double r = std::max(0.6, std::min(2.6, pitch * 0.62));

		QPainter painter(&img);
		painter.setRenderHint(QPainter::Antialiasing, true);
		painter.setPen(Qt::NoPen);
		for (const auto& d : dots) {
			painter.setBrush(QColor(d.color));
			painter.drawEllipse(d.at, r, r);
		}
		painter.end();
		return img;
	}

	// ── build the quads, skipping any cell with a dropout corner ──
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

			if (haveTex) {
				//the real surface appearance instead of a false-colour ramp: the JET ramp
				//answers "how high", the texture answers "what does it look like", and on a
				//part with hundreds of identical pins the second is often the useful one
				const int t = (int)((gtex.at<uchar>(j, i) + gtex.at<uchar>(j, i + 1)
					+ gtex.at<uchar>(j + 1, i) + gtex.at<uchar>(j + 1, i + 1)) / 4);
				q.color = qRgb(t, t, t);
			}

			if (lit) {
				/*
				* Light the facet by its own normal. Height alone cannot convey shape:
				* two facets at the same height but different slopes get the same colour,
				* which is exactly why the filled view reads flat. The cross product is
				* taken in view space (px, py, depth), so it is already oriented to the
				* camera and no separate model-to-view transform is needed.
				*/
				const double e1x = px[b] - px[a], e1y = py[b] - py[a], e1z = pd[b] - pd[a];
				const double e2x = px[c] - px[a], e2y = py[c] - py[a], e2z = pd[c] - pd[a];
				double nx = e1y * e2z - e1z * e2y;
				double ny = e1z * e2x - e1x * e2z;
				double nz = e1x * e2y - e1y * e2x;

				const double len = std::sqrt(nx * nx + ny * ny + nz * nz);
				if (len > 1e-12) {
					nx /= len; ny /= len; nz /= len;
					//fabs, not max(0,...): these quads are never back-face culled, so a
					//facet turned away from the light must still be shaded rather than
					//dropping to pure ambient and punching a hole in the surface
					const double diff = std::fabs(nx * kMeshLightX + ny * kMeshLightY
						+ nz * kMeshLightZ);
					const double lit = kMeshAmbient
						+ (1.0 - kMeshAmbient) * std::min(1.0, diff);

					const QRgb base = q.color;
					q.color = qRgb(
						std::min(255, (int)std::lround(qRed(base) * lit)),
						std::min(255, (int)std::lround(qGreen(base) * lit)),
						std::min(255, (int)std::lround(qBlue(base) * lit)));
				}
			}
			quads.push_back(std::move(q));
		}
	}

	/*
	* The datum, as a sheet of its own quads.
	*
	* Subdivided rather than drawn as one: this renderer resolves occlusion by painting
	* far-to-near, and a single quad carries a single depth - so one big sheet would land either
	* wholly in front of the part or wholly behind it. Cut into a grid it sorts with the part,
	* and the pins come through it where they stand above it.
	*/
	if (planeABC) {
		const int N = 24;
		const double psx = (double)src.cols / gw, psy = (double)src.rows / gh;
		auto planeRaw = [&](double gi, double gj) {
			return planeABC[0] * gi * psx + planeABC[1] * gj * psy + planeABC[2];
		};
		for (int j = 0; j < N; j++) {
			for (int i = 0; i < N; i++) {
				const double g0 = (double)i * (gw - 1) / N, g1 = (double)(i + 1) * (gw - 1) / N;
				const double h0 = (double)j * (gh - 1) / N, h1 = (double)(j + 1) * (gh - 1) / N;
				const double gx[4] = { g0, g1, g1, g0 };
				const double gy[4] = { h0, h0, h1, h1 };
				H3Quad q;
				q.plane = true;
				q.color = qRgba(255, 255, 255, 70);
				double ox, oy, dp, acc = 0.0;
				for (int k = 0; k < 4; k++) {
					project(gx[k], gy[k], planeRaw(gx[k], gy[k]), ox, oy, dp);
					q.poly << QPointF(ox, oy);
					acc += dp;
				}
				q.depth = acc / 4.0;
				quads.push_back(std::move(q));
			}
		}
	}

	if (quads.empty()) return img;

	//painter's algorithm: far first, so nearer geometry paints over it
	std::sort(quads.begin(), quads.end(),
		[](const H3Quad& l, const H3Quad& r) { return l.depth < r.depth; });

	QPainter painter(&img);
	painter.setRenderHint(QPainter::Antialiasing, false);

	const QColor bg(24, 26, 32);
	for (const auto& q : quads) {
		if (q.plane) {
			//translucent and unstroked: a stroked grid would read as a mesh of its own rather
			//than as the one flat reference surface it is
			painter.setPen(Qt::NoPen);
			painter.setBrush(QColor::fromRgba(q.color));
			painter.drawPolygon(q.poly);
			continue;
		}
		const QColor c(q.color);
		if (wire) {
			/*
			* HIDDEN-LINE wireframe, not a see-through one. Filling each cell with the
			* background before stroking it means a nearer facet erases the lines behind
			* it, so the lattice actually describes a solid. A transparent wireframe on a
			* surface this dense collapses into noise - every far line shows through every
			* near one and the shape disappears.
			*/
			painter.setPen(QPen(c, 1));
			painter.setBrush(bg);
		}
		else {
			//A darker edge of the facet's OWN colour, not a fixed black lattice: at a few
			//px per cell a black grid swamps the surface, while this reads as a mesh and
			//still carries the height colour. The smooth style has cells about a pixel
			//wide, so it strokes in the fill colour and shows no grid at all - which is
			//what makes it read as one continuous surface.
			painter.setPen(QPen(mesh ? c.darker(165) : c, 1));
			painter.setBrush(c);
		}
		painter.drawPolygon(q.poly);
	}

	painter.end();
	return img;
}

QImage algoH3RenderRelief2D(const cv::Mat& height16, int minValidRaw, int maxValidRaw,
	double zExaggeration, bool colorMapped)
{
	if (height16.empty()) return QImage();

	cv::Mat src;
	if (height16.type() == CV_16U) src = height16;
	else height16.convertTo(src, CV_16U);

	const cv::Mat mask = validMaskOf(src, minValidRaw, maxValidRaw);
	if (cv::countNonZero(mask) == 0) return QImage();

	double zMin = 0, zMax = 0;
	cv::minMaxLoc(src, &zMin, &zMax, nullptr, nullptr, mask);
	if (zMax <= zMin) zMax = zMin + 1.0;

	/*
	* Dropouts are 0, and a 0 beside a real height is a cliff that would dominate every
	* gradient near a pin - the relief would show the holes instead of the surface. Fill
	* them with the valid midpoint before differentiating, then paint them out again at
	* the end so nothing invented ever reaches the screen.
	*/
	cv::Mat f;
	src.convertTo(f, CV_32F);
	f.setTo(cv::Scalar(0.5 * (zMin + zMax)), ~mask);

	cv::Mat gx, gy;
	cv::Sobel(f, gx, CV_32F, 1, 0, 3);
	cv::Sobel(f, gy, CV_32F, 0, 1, 3);

	//normalise slope by the height range, so a part spanning 200 um and one spanning 6 mm
	//are lit the same way and the operator's Z exaggeration is what drives the relief
	const double gscale = std::max(0.05, std::min(2.0, zExaggeration)) * 6.0
		/ std::max(1.0, zMax - zMin);

	QImage img(src.cols, src.rows, QImage::Format_RGB888);
	const QRgb* lut = jetTable();

	//row pointers computed by hand: QImage::scanLine() detaches, which is not something to
	//do from several worker threads at once
	uchar* base = img.bits();
	const int stride = img.bytesPerLine();
	const double zRange = zMax - zMin;

	cv::parallel_for_(cv::Range(0, src.rows), [&](const cv::Range& band) {
		for (int y = band.start; y < band.end; y++) {
			const float* gxr = gx.ptr<float>(y);
			const float* gyr = gy.ptr<float>(y);
			const ushort* sr = src.ptr<ushort>(y);
			const uchar* mr = mask.ptr<uchar>(y);
			uchar* out = base + (size_t)y * stride;

			for (int x = 0; x < src.cols; x++) {
				if (!mr[x]) {   //same near-black the 3D views use for "no data"
					out[3 * x + 0] = 20; out[3 * x + 1] = 21; out[3 * x + 2] = 26;
					continue;
				}

				//normal of a height field is (-dz/dx, -dz/dy, 1); the light is the same
				//fixed one the 3D mesh uses, so the two views agree on where "up" is
				const double nx = -(double)gxr[x] * gscale;
				const double ny = -(double)gyr[x] * gscale;
				const double len = std::sqrt(nx * nx + ny * ny + 1.0);
				const double diff = (nx * kMeshLightX + ny * kMeshLightY + kMeshLightZ) / len;
				const double lit = kMeshAmbient
					+ (1.0 - kMeshAmbient) * std::max(0.0, std::min(1.0, diff));

				if (colorMapped) {
					const int lvl = (int)std::lround(
						((double)sr[x] - zMin) / zRange * 255.0);
					const QRgb c = lut[std::max(0, std::min(255, lvl))];
					out[3 * x + 0] = (uchar)std::min(255, (int)std::lround(qRed(c) * lit));
					out[3 * x + 1] = (uchar)std::min(255, (int)std::lround(qGreen(c) * lit));
					out[3 * x + 2] = (uchar)std::min(255, (int)std::lround(qBlue(c) * lit));
				}
				else {
					const uchar g = (uchar)std::min(255, (int)std::lround(lit * 255.0));
					out[3 * x + 0] = g; out[3 * x + 1] = g; out[3 * x + 2] = g;
				}
			}
		}
	});

	return img;
}

/*
* The strongest repeat in a 1-D profile, and where it starts.
*
* A pin field is periodic, so the column sums of its mask are too - and one Fourier
* coefficient per candidate period finds that repeat far more reliably than trying to group
* blobs into rows and columns. Grouping has to decide what counts as "the same row" before it
* knows the pitch, which on a field whose pins are nearly as tall as the gap between rows is a
* decision it gets wrong; the transform needs no such threshold.
*
* The coefficient's angle gives the phase, which is what turns a pitch into an actual lattice.
*
* Coarse pass then a fine one around the winner: the direct sum is O(period * samples) and the
* coarse grid alone would be a tenth of a pixel over a 9000 px profile.
*/
static bool h3StrongestPeriod(const std::vector<double>& sig, double loP, double hiP,
	double& period, double& phase, double& confidence)
{
	const int N = (int)sig.size();
	if (N < 32 || hiP <= loP) return false;

	double mean = 0.0;
	for (double v : sig) mean += v;
	mean /= N;

	auto coeff = [&](double P, double& re, double& im) {
		re = im = 0.0;
		const double k = 2.0 * CV_PI / P;
		for (int i = 0; i < N; i++) {
			const double s = sig[i] - mean;
			re += s * std::cos(k * i);
			im -= s * std::sin(k * i);
		}
	};

	double bestA = -1.0, bestP = 0.0, bestRe = 0, bestIm = 0;
	for (double P = loP; P <= hiP; P += 1.0) {
		double re, im; coeff(P, re, im);
		const double a = std::sqrt(re * re + im * im);
		if (a > bestA) { bestA = a; bestP = P; bestRe = re; bestIm = im; }
	}
	if (bestA <= 0.0) return false;

	for (double P = std::max(loP, bestP - 1.0); P <= std::min(hiP, bestP + 1.0); P += 0.05) {
		double re, im; coeff(P, re, im);
		const double a = std::sqrt(re * re + im * im);
		if (a > bestA) { bestA = a; bestP = P; bestRe = re; bestIm = im; }
	}

	/*
	* How much the winner stands out, ignoring its own harmonics - a period of P/2 or P/3 is the
	* same lattice described twice, not a rival. Without that exclusion every real lattice would
	* score as ambiguous against its own second harmonic.
	*/
	double rival = 0.0;
	for (double P = loP; P <= hiP; P += 1.0) {
		bool harmonic = false;
		for (int k = 1; k <= 4 && !harmonic; k++) {
			harmonic |= std::fabs(P - bestP / k) < 0.15 * bestP / k;
			harmonic |= std::fabs(P - bestP * k) < 0.15 * bestP * k;
		}
		if (harmonic) continue;
		double re, im; coeff(P, re, im);
		rival = std::max(rival, std::sqrt(re * re + im * im));
	}

	period = bestP;
	//angle -> the offset of the first lattice line, folded into [0, P)
	phase = std::fmod(std::atan2(bestIm, bestRe) / (2.0 * CV_PI) * bestP + bestP, bestP);
	confidence = (rival > 0.0) ? bestA / rival : 99.0;
	return true;
}

/*
* Teach every pin on the segmented part at once.
*
* Placing a box on each pin by hand means two hundred boxes. This places them.
*
* A pin is whatever stands proud of the datum, so the datum has to be fitted first - which also
* means heights are already relative to the part's own plate, and a tilted part needs no
* special handling.
*
*   1. Height above the datum, thresholded at a fraction of how tall the pins actually are.
*      A fraction rather than a number in um, because the field's own height sets the scale.
*   2. Close by a pin-sized amount, so one pin reads as one blob. The sensor sees a pin from
*      one side, so what comes back is a crescent with holes rather than a disc.
*   3. THE LATTICE, not the blobs, decides where the ROIs go. Blobs are not pins: a shadowed
*      pin breaks into two or three of them and a pair that touch become one, so an ROI per
*      blob gives duplicates stacked on one pin and single boxes straddling two. The pitch
*      comes from the strongest repeat in the mask's column and row sums, which is a property
*      of the whole field rather than of any blob, and every ROI then lands on a lattice
*      point - perfectly regular, and incapable of overlapping its neighbour.
*   4. A cell gets an ROI only if enough of it is actually pin. Nothing is invented: an ROI
*      with no pin under it measures the plate and passes every criterion, which is the worst
*      way for this to be wrong.
*
* seed, when given, is one ROI the operator has already placed on a pin. Its size becomes the
* box size and its centre fixes the lattice phase, which is the fallback when the field is too
* broken or too small for the transform to find the repeat on its own.
*/
bool AlgoHeight3Pipeline::findPins(const AlgoHeight3Params& p, const QRectF* seed,
	AlgoH3PinFind& out, QString& why) const
{
	out = AlgoH3PinFind();

	if (!segmentReady()) { why = QStringLiteral("Run segmentation first"); return false; }
	if (!m_datumDone || !m_out.planeValid) {
		why = QStringLiteral("Fit the datum plane first - a pin is found by how far it stands "
			"above the datum.");
		return false;
	}
	if (p.zScaleRawPerUm <= 0.0 || p.xScaleUmPx <= 0.0 || p.yScaleUmPx <= 0.0) {
		why = QStringLiteral("X, Y and Z scale must all be greater than 0");
		return false;
	}

	const int w = m_cropHeight.cols, h = m_cropHeight.rows;
	H3Plane plane;
	plane.a = m_out.planeA; plane.b = m_out.planeB; plane.c = m_out.planeC; plane.valid = true;

	// ── 1. what stands above the datum ──
	std::vector<float> above;
	above.reserve((size_t)w * h / 4);
	for (int y = 0; y < h; y++) {
		const ushort* row = m_cropHeight.ptr<ushort>(y);
		for (int x = 0; x < w; x++) {
			const ushort z = row[x];
			if (z < p.minValidRaw || z > p.maxValidRaw) continue;
			const double d = (z - planeZ(plane, x, y)) / p.zScaleRawPerUm;
			if (d > 0.0) above.push_back((float)d);
		}
	}
	if (above.size() < 1000) { why = QStringLiteral("Nothing stands above the datum plane"); return false; }

	//the 99th percentile, not the maximum, so one spike cannot set the scale for the whole part
	const size_t q = (size_t)(0.99 * (above.size() - 1));
	std::nth_element(above.begin(), above.begin() + q, above.end());
	const double pinTopUm = above[q];
	if (pinTopUm < 50.0) {
		why = QStringLiteral("Nothing stands more than %1 um above the datum - is the datum on "
			"the right surface?").arg(pinTopUm, 0, 'f', 1);
		return false;
	}
	//a sixth of the way up a pin: clear of the plate and of whatever lies on it, well below the
	//shortest pin worth teaching
	const double cutUm = std::max(25.0, pinTopUm / 6.0);

	/*
	* TWO masks, because they answer different questions.
	*
	* The low one, a sixth of the way up a pin, is everything that stands proud of the plate -
	* a pin's whole flank and the skirt of lower surface around it. That is the right thing for
	* asking WHETHER a lattice cell holds a pin, because a pin lost in its neighbour's shadow
	* still shows some of its flank.
	*
	* The high one is the pin TOP: the flat disc at the end of it. That is the right thing for
	* asking WHERE the pin is and HOW BIG to make its box. Sizing from the low mask was the
	* mistake in the first version - the skirt is as wide as the whole cell and lopsided with
	* it, so the boxes came out cell-sized and centred on the shadow rather than on the pin.
	*/
	const double topCutUm = 0.7 * pinTopUm;

	cv::Mat mask(h, w, CV_8U, cv::Scalar(0));
	cv::Mat tops(h, w, CV_8U, cv::Scalar(0));
	for (int y = 0; y < h; y++) {
		const ushort* row = m_cropHeight.ptr<ushort>(y);
		uchar* mrow = mask.ptr<uchar>(y);
		uchar* trow = tops.ptr<uchar>(y);
		for (int x = 0; x < w; x++) {
			const ushort z = row[x];
			if (z < p.minValidRaw || z > p.maxValidRaw) continue;
			const double d = (z - planeZ(plane, x, y)) / p.zScaleRawPerUm;
			if (d > cutUm) mrow[x] = 255;
			if (d > topCutUm) trow[x] = 255;
		}
	}

	// ── 2. one pin, one blob ──
	//150 um of closing: enough to bridge the holes across a pin's own face, far too little to
	//join two pins a millimetre apart
	const int closeK = std::max(3, std::min(81,
		(int)std::lround(150.0 / std::min(p.xScaleUmPx, p.yScaleUmPx)) | 1));
	cv::morphologyEx(mask, mask, cv::MORPH_CLOSE,
		cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(closeK, closeK)));
	cv::morphologyEx(tops, tops, cv::MORPH_CLOSE,
		cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(closeK, closeK)));

	//the pin tops as objects: centres to put the lattice through, extents to size the box by
	cv::Mat tLab, tStat, tCent;
	const int nTops = cv::connectedComponentsWithStats(tops, tLab, tStat, tCent, 8, CV_32S);
	std::vector<int> topAreas;
	for (int i = 1; i < nTops; i++) topAreas.push_back(tStat.at<int>(i, cv::CC_STAT_AREA));
	int topFloor = 0;
	if (!topAreas.empty()) {
		std::nth_element(topAreas.begin(), topAreas.begin() + topAreas.size() / 2, topAreas.end());
		//a third of a typical top: keeps a disc the sensor only half saw, drops the speckle
		topFloor = std::max(16, topAreas[topAreas.size() / 2] / 3);
	}
	struct Disc { double cx, cy; int w, h; };
	std::vector<Disc> discs;
	for (int i = 1; i < nTops; i++) {
		if (tStat.at<int>(i, cv::CC_STAT_AREA) < topFloor) continue;
		discs.push_back({ tCent.at<double>(i, 0), tCent.at<double>(i, 1),
			tStat.at<int>(i, cv::CC_STAT_WIDTH), tStat.at<int>(i, cv::CC_STAT_HEIGHT) });
	}

	// ── 3. the lattice ──
	//the tops, because they are compact and separated - the flanks run into each other and
	//blur the very repeat the transform is looking for
	const cv::Mat& periodic = discs.size() >= 8 ? tops : mask;
	std::vector<double> colSum(w, 0.0), rowSum(h, 0.0);
	for (int y = 0; y < h; y++) {
		const uchar* mrow = periodic.ptr<uchar>(y);
		for (int x = 0; x < w; x++) {
			if (!mrow[x]) continue;
			colSum[x] += 1.0; rowSum[y] += 1.0;
		}
	}

	//a lattice finer than 100 um is not a pin field, and one coarser than a third of the part
	//cannot be measured from the two or three repeats that would fit
	const double loX = std::max(8.0, 100.0 / p.xScaleUmPx), hiX = w / 3.0;
	const double loY = std::max(8.0, 100.0 / p.yScaleUmPx), hiY = h / 3.0;

	double px = 0, py = 0, phx = 0, phy = 0, confX = 0, confY = 0;
	const bool gotX = h3StrongestPeriod(colSum, loX, hiX, px, phx, confX);
	const bool gotY = h3StrongestPeriod(rowSum, loY, hiY, py, phy, confY);

	//1.3 is deliberately low. A real field beats its nearest non-harmonic rival by two or three
	//times, and anything near parity is a field the transform cannot read - better to say so and
	//let one taught ROI settle it than to lay a confident grid over nothing.
	const double kMinConfidence = 1.3;
	const bool latticeOk = gotX && gotY && confX >= kMinConfidence && confY >= kMinConfidence;

	if (!latticeOk && !seed) {
		why = QStringLiteral("Could not read a repeating pin pattern (confidence %1 across, %2 "
			"down; %3 needed). Place one ROI on a single pin and press Auto Assign again - its "
			"size and position are enough to lay out the rest.")
			.arg(confX, 0, 'f', 2).arg(confY, 0, 'f', 2).arg(kMinConfidence, 0, 'f', 2);
		return false;
	}

	/*
	* The box is sized to the pin TOP, not to the cell.
	*
	* The ninetieth percentile of the discs rather than the largest: a couple of them will have
	* run into a neighbour's top and measure double, and one of those must not set the size for
	* all two hundred. Plus a tenth as buffer, never less than 50 um.
	*/
	double discW = 0, discH = 0;
	if (discs.size() >= 8) {
		std::vector<int> ws, hs;
		for (const Disc& d : discs) { ws.push_back(d.w); hs.push_back(d.h); }
		const size_t k = (size_t)(0.9 * (ws.size() - 1));
		std::nth_element(ws.begin(), ws.begin() + k, ws.end()); discW = ws[k];
		std::nth_element(hs.begin(), hs.begin() + k, hs.end()); discH = hs[k];
		discW += 2.0 * std::max(50.0 / p.xScaleUmPx, discW / 10.0);
		discH += 2.0 * std::max(50.0 / p.yScaleUmPx, discH / 10.0);
	}

	double boxW = 0, boxH = 0;
	if (seed) {
		//the operator has shown it a pin: take the size from the box they drew, and put the
		//lattice through its centre so the grid is in step with the pin they chose
		boxW = seed->width();
		boxH = seed->height();
		const double sx = seed->center().x() + w / 2.0;
		const double sy = seed->center().y() + h / 2.0;
		if (!gotX || !gotY) {
			why = QStringLiteral("Could not measure the spacing between pins even with a taught "
				"ROI - check that the datum is on the plate and that pins stand above it.");
			return false;
		}
		phx = std::fmod(sx - px / 2.0 + px * 1000.0, px);
		phy = std::fmod(sy - py / 2.0 + py * 1000.0, py);
	}
	else {
		//the pin top plus its buffer, capped by the cell so two boxes can never touch. Falling
		//back to the cell only when there were too few tops to measure one.
		boxW = (discW > 1.0) ? std::min(discW, px - 4.0) : px - 4.0;
		boxH = (discH > 1.0) ? std::min(discH, py - 4.0) : py - 4.0;
	}
	if (boxW < 4.0 || boxH < 4.0) { why = QStringLiteral("The pin spacing is too small to place ROIs in"); return false; }

	/*
	* Put the cell centres on the pins.
	*
	* The transform's phase locks onto the repeat, but which part of the cycle it calls zero
	* depends on the shape of the profile, so the grid can land half a cell out - which is
	* exactly what it did on the sample part, cutting every pin in half.
	*
	* The fix is to take the phase from the pin TOPS. Their centres are what a box has to be
	* centred on, and averaging them has to be done on the circle: these are positions modulo
	* the pitch, so a straight mean of discs scattered either side of zero lands at the pitch's
	* midpoint, which is the one answer that is certainly wrong. Summing unit vectors and taking
	* the angle has no such seam.
	*/
	if (!seed && discs.size() >= 8) {
		double sxr = 0, sxi = 0, syr = 0, syi = 0;
		for (const Disc& d : discs) {
			const double ax = 2.0 * CV_PI * d.cx / px, ay = 2.0 * CV_PI * d.cy / py;
			sxr += std::cos(ax); sxi += std::sin(ax);
			syr += std::cos(ay); syi += std::sin(ay);
		}
		//the mean disc position within a cycle; the cell BOUNDARY is half a pitch before it
		const double mux = std::atan2(sxi, sxr) / (2.0 * CV_PI) * px;
		const double muy = std::atan2(syi, syr) / (2.0 * CV_PI) * py;
		phx = std::fmod(mux - px / 2.0 + px * 1000.0, px);
		phy = std::fmod(muy - py / 2.0 + py * 1000.0, py);
	}

	// ── 4. a box per cell that actually holds a pin ──
	const int cols = (int)std::ceil((w - phx) / px) + 1;
	const int rows = (int)std::ceil((h - phy) / py) + 1;
	if (cols < 1 || rows < 1 || (qint64)cols * rows > 100000) {
		why = QStringLiteral("The pin lattice came out implausible (%1 x %2 cells)").arg(cols).arg(rows);
		return false;
	}

	std::vector<int> fill((size_t)cols * rows, 0);
	for (int y = 0; y < h; y++) {
		const uchar* mrow = mask.ptr<uchar>(y);
		const int r = (int)std::floor((y - phy) / py);
		if (r < 0 || r >= rows) continue;
		for (int x = 0; x < w; x++) {
			if (!mrow[x]) continue;
			const int c = (int)std::floor((x - phx) / px);
			if (c < 0 || c >= cols) continue;
			fill[(size_t)r * cols + c]++;
		}
	}

	//an eighth of the cell. A pin shadowed almost to nothing still covers more than that, and
	//the speckle a threshold leaves behind covers far less.
	const int minFill = (int)std::max(16.0, 0.125 * px * py);

	struct Cell { double cx, cy; int fill; };
	std::vector<Cell> cells;
	int edgeDropped = 0;
	for (int r = 0; r < rows; r++) {
		for (int c = 0; c < cols; c++) {
			const int f = fill[(size_t)r * cols + c];
			if (f < minFill) continue;
			const double cx = phx + (c + 0.5) * px;
			const double cy = phy + (r + 0.5) * py;
			//a box hanging off the crop would measure less of its pin than the others do, and
			//silently - so it is dropped and counted rather than taught
			if (cx - boxW / 2 < 0 || cx + boxW / 2 > w || cy - boxH / 2 < 0 || cy + boxH / 2 > h) {
				edgeDropped++;
				continue;
			}
			cells.push_back({ cx, cy, f });
		}
	}
	if (cells.empty()) {
		why = QStringLiteral("The lattice fitted, but no cell holds enough pin to teach");
		return false;
	}

	/*
	* ── which pins count as the same kind ──
	*
	* By how much pin each box holds, and only split where the sorted amounts show a real gap -
	* a ratio of 1.8 between neighbours, with at least five pins on each side. Anything looser
	* invents types out of noise, because how much of a pin the sensor returns depends on how
	* deep it sits in its neighbours' shadow as much as on the pin itself.
	*/
	std::vector<std::pair<int, size_t>> sorted;
	for (size_t i = 0; i < cells.size(); i++) sorted.emplace_back(cells[i].fill, i);
	std::sort(sorted.begin(), sorted.end());

	std::vector<size_t> cuts;
	for (size_t i = 1; i < sorted.size(); i++) {
		if (sorted[i].first < sorted[i - 1].first * 1.8) continue;
		if (i < 5 || sorted.size() - i < 5) continue;
		cuts.push_back(i);
		if (cuts.size() >= 3) break;         //four kinds of pin is already more than plausible
	}

	std::vector<int> groupOf(cells.size(), 0);
	int g = 0; size_t nextCut = 0;
	for (size_t i = 0; i < sorted.size(); i++) {
		if (nextCut < cuts.size() && i == cuts[nextCut]) { g++; nextCut++; }
		groupOf[sorted[i].second] = g;
	}
	out.groups = g + 1;

	const double ox = w / 2.0, oy = h / 2.0;
	for (size_t i = 0; i < cells.size(); i++) {
		out.boxes.append(QRectF(cells[i].cx - ox - boxW / 2.0,
			cells[i].cy - oy - boxH / 2.0, boxW, boxH));
		out.group.append(groupOf[i]);
	}

	out.pitchXUm = px * p.xScaleUmPx;
	out.pitchYUm = py * p.yScaleUmPx;
	out.boxWidthUm = boxW * p.xScaleUmPx;
	out.boxHeightUm = boxH * p.yScaleUmPx;
	out.edgeDropped = edgeDropped;
	out.seeded = (seed != nullptr);
	out.note = QStringLiteral("%1 pins on a %2 x %3 um pitch, %4 x %5 um boxes")
		.arg(out.boxes.size())
		.arg(out.pitchXUm, 0, 'f', 0).arg(out.pitchYUm, 0, 'f', 0)
		.arg(out.boxWidthUm, 0, 'f', 0).arg(out.boxHeightUm, 0, 'f', 0);
	if (seed) out.note += QStringLiteral(", from a taught ROI");
	else out.note += QStringLiteral(", confidence %1/%2").arg(confX, 0, 'f', 1).arg(confY, 0, 'f', 1);
	if (edgeDropped > 0) out.note += QStringLiteral(", %1 dropped at the edge").arg(edgeDropped);
	if (out.groups > 1) out.note += QStringLiteral(", %1 sizes").arg(out.groups);
	return true;
}
