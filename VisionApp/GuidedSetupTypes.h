#pragma once
#include <QString>
#include <QRectF>

/*
* Guided setup: the short road from an empty recipe to a layout production can run.
*
* The operator teaches four fiducials and two units, every one of them by the same motion: park
* the camera roughly over a circle, press Teach, and the page finds the circle nearest the image
* centre, jogs until it IS the centre, and stores where the machine stands. Nothing here is a new
* kind of recipe data - the result is written into the fiducials (Circle Finder method), pitch
* region 1 and the setup-region mode that production already reads, so the run itself is the
* ordinary pitch-mode run. What this struct keeps is only what the page needs to do the teaching
* again: the circle size ranges, the reference box and the recipe type.
*/
struct GuidedSetupConfig {
	QString recipeType = QStringLiteral("Pogo"); //the only type so far; the combo is here for the next one

	//circle diameter ranges handed to the Matrox circle finder, in FOV pixels
	int fidMinDiaPx = 280;
	int fidMaxDiaPx = 320;
	int unitMinDiaPx = 200;
	int unitMaxDiaPx = 400;

	//the on-screen reference box: its size is shown live so the operator can read a diameter off it
	QRectF roi;

	//the 2D camera's in-focus height. Taught once and pushed into the plane, the 2D views and the
	//unit grid through applyViewZ(z, false); fiducials, barcode teach points and line scans are
	//NOT touched. Kept here so the page can show it and jog back to it.
	bool focusZSet = false;
	double focusZ = 0.0;
};
