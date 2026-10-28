// =============================================================================
//  ZSearch.cpp
//  Z search arithmetic: focus score, peak detection and profiler read statistics.
//  See ZSearch.h.
// =============================================================================

#include "ZSearch.h"

#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>

namespace zsearch {

	double focusScore(const QImage& img, int boxPx)
	{
		if (img.isNull()) return 0.0;

		const int side = std::min({ boxPx, img.width(), img.height() });
		if (side < 16) return 0.0;

		const QRect box((img.width() - side) / 2, (img.height() - side) / 2, side, side);
		const QImage gray = img.copy(box).convertToFormat(QImage::Format_Grayscale8);
		const cv::Mat g(gray.height(), gray.width(), CV_8UC1, (void*)gray.constBits(), (size_t)gray.bytesPerLine());

		cv::Mat f, gx, gy;
		g.convertTo(f, CV_32F, 1.0 / 255.0);
		cv::Sobel(f, gx, CV_32F, 1, 0, 3);
		cv::Sobel(f, gy, CV_32F, 0, 1, 3);
		const cv::Mat energy = gx.mul(gx) + gy.mul(gy);
		return cv::mean(energy)[0] * 1000.0;
	}

	int bestConfirmed(const QVector<FocusSample>& s)
	{
		int best = -1;
		for (int i = 0; i < s.size(); i++) {
			if (!s[i].confirmed) continue;
			if (best < 0 || s[i].score > s[best].score) best = i;
		}
		return best;
	}

	bool peakPassed(const QVector<FocusSample>& s, int best, double dropRatio)
	{
		if (best < 0 || s.size() < best + 3) return false;
		const double threshold = dropRatio * s[best].score;
		const int last = s.size() - 1;
		return s[last].score < threshold && s[last - 1].score < threshold;
	}

	double fitPeak(const QVector<FocusSample>& s, int best, double fraction, int* used)
	{
		if (used) *used = 0;
		if (best < 0 || best >= s.size()) return 0.0;

		QVector<int> idx;
		const double floorScore = fraction * s[best].score;
		for (int i = 0; i < s.size(); i++)
			if (s[i].confirmed && s[i].score >= floorScore) idx.push_back(i);
		if (idx.size() < 3) {
			idx.clear();
			for (int i = std::max(0, best - 1); i <= std::min(s.size() - 1, best + 1); i++) idx.push_back(i);
		}
		if (used) *used = idx.size();
		if (idx.size() < 3) return s[best].z;

		/*
		* s = a x^2 + b x + c by least squares, x = z - mean(z) so the sums stay well conditioned.
		* Normal equations solved by Cramer's rule; vertex at x = -b / 2a, meaningful only for a < 0.
		*/
		double zMean = 0.0;
		for (int i : idx) zMean += s[i].z;
		zMean /= idx.size();

		double S0 = 0, S1 = 0, S2 = 0, S3 = 0, S4 = 0, T0 = 0, T1 = 0, T2 = 0;
		double zLo = s[idx[0]].z, zHi = zLo;
		for (int i : idx) {
			const double x = s[i].z - zMean, y = s[i].score, x2 = x * x;
			S0 += 1; S1 += x; S2 += x2; S3 += x2 * x; S4 += x2 * x2;
			T0 += y; T1 += x * y; T2 += x2 * y;
			zLo = std::min(zLo, s[i].z);
			zHi = std::max(zHi, s[i].z);
		}

		auto det3 = [](double a1, double b1, double c1, double a2, double b2, double c2, double a3, double b3, double c3) {
			return a1 * (b2 * c3 - b3 * c2) - b1 * (a2 * c3 - a3 * c2) + c1 * (a2 * b3 - a3 * b2);
		};
		const double D = det3(S4, S3, S2, S3, S2, S1, S2, S1, S0);
		if (!std::isfinite(D) || std::abs(D) < 1e-18) return s[best].z;
		const double a = det3(T2, S3, S2, T1, S2, S1, T0, S1, S0) / D;
		const double b = det3(S4, T2, S2, S3, T1, S1, S2, T0, S0) / D;
		if (!std::isfinite(a) || !std::isfinite(b) || !(a < 0.0)) return s[best].z;

		const double z = zMean - b / (2.0 * a);
		if (!std::isfinite(z)) return s[best].z;
		return std::max(zLo, std::min(zHi, z));
	}

	ProfileStats profileStats(std::vector<double> values)
	{
		values.erase(std::remove_if(values.begin(), values.end(),
			[](double v) { return !std::isfinite(v); }), values.end());

		ProfileStats st;
		st.valid = (int)values.size();
		if (values.empty()) return st;

		std::sort(values.begin(), values.end());
		const auto at = [&](double q) { return values[(size_t)std::lround(q * (values.size() - 1))]; };
		st.median = at(0.5);
		st.spread = at(0.75) - at(0.25);
		return st;
	}
}
