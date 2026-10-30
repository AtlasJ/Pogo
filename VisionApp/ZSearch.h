#pragma once
#include <QImage>
#include <QVector>
#include <vector>

/*
* Z search: the arithmetic, kept apart from the motion and the page so it can be read (and
* tested) on its own. Nothing here moves, snaps or touches a widget.
*
* The 2D half scores how sharp a frame is; the search moves Z, snaps, scores, and keeps the Z
* whose frame scored highest. The 3D half turns a set of profiler points into one height and a
* flatness figure, so a read can be trusted (or not) before the search acts on it.
*/
namespace zsearch {

	/*
	* Tenengrad focus measure: the mean squared Sobel gradient over a square of boxPx at the image
	* centre (clamped to the image). Higher is sharper. Only comparable between frames of the same
	* scene, lighting and box - which is exactly what a sweep at one XY produces. Scaled by 1000 so
	* the logged numbers are readable. 0 for a null or tiny image.
	*/
	double focusScore(const QImage& img, int boxPx);

	struct FocusSample {
		double z = 0.0;
		double score = 0.0;
		bool confirmed = true; //false when "Confirm with Safety Check" is on and it failed at this Z
	};

	//index of the sharpest CONFIRMED sample, -1 if none
	int bestConfirmed(const QVector<FocusSample>& s);

	/*
	* True once the peak is clearly behind the sweep: the last two samples, both after `best`, have
	* fallen below dropRatio x the best score. Two in a row, so one noisy frame cannot end it.
	*/
	bool peakPassed(const QVector<FocusSample>& s, int best, double dropRatio);

	/*
	* The coarse sweep's next step when it adapts. Far from focus the sharpness barely changes and
	* the step is maxStep; once it starts to climb (or fall) the step shrinks so each one changes it
	* by about half - 0.4 / |d ln(score) / dz| - never growing by more than half the step before (it
	* must not leap off the top of the hill) and never below a tenth of maxStep. The first step, with
	* one frame to go on, is a quarter of maxStep: Start Z may already be close to focus.
	*/
	double nextAdaptiveStep(const QVector<FocusSample>& s, double maxStep, double lastStep);

	/*
	* Sub-step peak: the vertex of a least-squares parabola through every confirmed sample scoring at
	* least `fraction` of the best - the top of the hill, not just its three highest points, so image
	* noise averages out instead of deciding the result. Too few points for that and it uses the best
	* and its two neighbours. Clamped to the Z span of the points used; falls back to the best
	* sample's Z when they do not curve downwards. `used` (optional) receives how many points it fitted.
	*/
	double fitPeak(const QVector<FocusSample>& s, int best, double fraction, int* used = nullptr);

	struct ProfileStats {
		int valid = 0;        //finite points used
		double median = 0.0;  //mm, the profiler's own sign
		double spread = 0.0;  //mm, interquartile range: the middle 50% of the points lie within this
	};

	//NaN (out of range / no return) points are dropped first
	ProfileStats profileStats(std::vector<double> values);
}
