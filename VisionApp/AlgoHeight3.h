#pragma once

#include <opencv2/opencv.hpp>
#include <QImage>
#include <QSize>
#include <QString>

#include "AlgoHeight3Types.h"

/*
* The 3D Height Measurement 3 pipeline.
*
* Five stages, each one runnable on its own from its section's Run button, or all of
* them in order from the page's Run button:
*
*   source maps -> [1] preprocess -> [2] segment -> [3] datum plane -> [4] measure -> [5] overall
*                        (full map)     (full map)      (crop)            (crop)
*
* A stage may only run when the stage before it has passed, and running a stage
* INVALIDATES every stage after it. That rule is the whole reason this is a class and
* not five free functions: re-running segmentation moves the part frame, which moves
* every ROI, which makes the plane and every height stale. Showing stale numbers as if
* they were current is the failure this design exists to prevent.
*
* No Qt widgets here, and nothing in this class touches the GUI thread's state, so the
* whole pipeline can run on AlgoManager's worker thread.
*/
class AlgoHeight3Pipeline {
public:
	// ── source ──
	//both maps are required to run; intensity is for the operator to look at, every
	//number in the output comes from the height map alone
	void setSourceMaps(const cv::Mat& height16, const cv::Mat& intensity8);
	void setHeightMap(const cv::Mat& height16);
	void setIntensityMap(const cv::Mat& intensity8);
	void clearAll();

	bool hasHeight() const { return !m_height.empty(); }
	bool hasIntensity() const { return !m_intensity.empty(); }
	QSize sourceSize() const;      //height map size - the size everything must match
	QSize intensitySize() const;
	QSize cropSize() const;

	// ── stage state ──
	bool preprocessReady() const { return m_preprocessDone; }
	bool segmentReady() const { return m_segmentDone && !m_cropHeight.empty(); }
	bool datumReady() const { return m_datumDone; }
	bool measureReady() const { return m_measureDone; }
	void invalidateFrom(AlgoH3Stage stage);

	// ── running ──
	//returns the stage's pass/fail; the reason is in output().<stage>.failReason
	bool runStage(AlgoH3Stage stage, const AlgoHeight3Params& p);

	AlgoHeight3Output output() const { return m_out; }

	// ── display sources ──
	//the map a given section should be looking at, already 8-bit for display.
	//segmented=false gives the full map, true gives the straightened crop.
	//preprocessed=false gives the raw height map, true the filtered one.
	cv::Mat heightForDisplay(bool preprocessed, bool segmented) const;
	cv::Mat intensityForDisplay(bool segmented) const;

private:
	bool doPreprocess(const AlgoHeight3Params& p);
	bool doSegment(const AlgoHeight3Params& p);
	bool doDatum(const AlgoHeight3Params& p);
	bool doMeasure(const AlgoHeight3Params& p);
	bool doOverall(const AlgoHeight3Params& p);

	//the height map a stage should read: preprocessed when it exists, raw otherwise
	const cv::Mat& workingHeight() const;

	cv::Mat m_height;        //CV_16U source height map
	cv::Mat m_intensity;     //CV_8U source intensity map, same size as m_height
	cv::Mat m_work;          //CV_16U preprocessed height map (empty until preprocess runs)
	cv::Mat m_cropHeight;    //CV_16U straightened part crop
	cv::Mat m_cropIntensity; //CV_8U straightened part crop, pixel-aligned with the above

	bool m_preprocessDone = false;
	bool m_segmentDone = false;
	bool m_datumDone = false;
	bool m_measureDone = false;

	AlgoHeight3Output m_out;
};

// ── shared helpers (used by the pipeline and by the page) ────────────────────

//8-bit render of a 16-bit height map: valid range stretched to 0..255, dropouts black.
//colorMapped adds the JET ramp used everywhere else in the app for height.
QImage algoH3HeightToQImage(const cv::Mat& height16, int minValidRaw, int maxValidRaw, bool colorMapped);
QImage algoH3GrayToQImage(const cv::Mat& gray8);

/*
* Software 3D surface render of a height map - a real projected surface the operator can
* drag to spin, not a colour ramp of a flat image. Deliberately CPU-only: the app has no
* OpenGL surface anywhere else, and a viewing aid is not worth a GL context on a machine
* PC. The grid is downsampled to keep a drag interactive.
*/
QImage algoH3RenderSurface3D(const cv::Mat& height16, int minValidRaw, int maxValidRaw,
	double yawDeg, double pitchDeg, double zExaggeration, const QSize& outSize);
