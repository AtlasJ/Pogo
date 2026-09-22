#pragma once
#include <QString>
#include <QRectF>
#include <QVector>

/*
* Safety check: a go/no-go look for an expected feature before a production run starts.
*
* Modelled on IM420's side alignment but deliberately simpler - that one measures an offset
* and feeds it back into the motion; this one only answers "is the feature there?". If it is
* not, production does not start.
*
* Two ways to recognise the feature, because the two failure modes are different: a coloured
* part is found by colour blobs with size filters (robust to position, weak on shape), and a
* printed or moulded feature is found by pattern matching (strong on shape, needs the learnt
* model). One or the other, never both - mixing them only makes the result harder to explain
* when it trips.
*/
struct SafetyCheckConfig {
	bool enabled = false;
	int method = 0; //0 = colour blobs, 1 = pattern matching

	/*
	* Where the machine must stand to see the feature. The check jogs here before snapping,
	* so the taught point is part of the recipe - a check run from wherever the gantry happened
	* to stop would be meaningless.
	*/
	bool pointSet = false;
	double pointX = 0.0, pointY = 0.0, pointZ = 0.0;

	QRectF roi; //search region in FOV pixels; empty = whole image

	//── colour blobs
	QVector<bool> colors;        //indexed by mtrx::Color, sized to Color::SIZE on load
	int chromaThreshold = 15;
	bool enableArea = false;   double areaMin = 0.0,   areaMax = 0.0;
	bool enableWidth = false;  double widthMin = 0.0,  widthMax = 0.0;
	bool enableHeight = false; double heightMin = 0.0, heightMax = 0.0;
	int minBlobs = 1;            //blobs passing every enabled filter for the check to pass

	//── pattern matching
	double patternScore = 70.0;  //acceptance score, same scale as the fiducial finders
};

struct SafetyCheckResult {
	bool ok = false;
	QString message;
	int found = 0;               //blobs passing the filters, or 1/0 for a pattern
	double score = 0.0;          //pattern score, unused for colour
	QVector<QRectF> marks;       //what was found, FOV pixels, for the overlay
};
