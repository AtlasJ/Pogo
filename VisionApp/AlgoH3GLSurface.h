#pragma once
//OpenGL 3.3 core render of the Height 3 surface, drawn offscreen into a QImage.
//
//Why GL at all, when the CPU renderer in AlgoHeight3.cpp already works: it sorts quads
//far-to-near and paints them in order (the painter's algorithm). That is only correct when no
//two facets interleave in depth, and on a pin field they do constantly - a pin in front of
//another pin at a similar distance flickers through it as the view is spun. A depth buffer has
//no such failure mode. The second reason is density: the CPU path had to cap the mesh at a few
//hundred cells a side to stay draggable, and at that budget a 1.27 mm pin on a 10000 px map is
//under four cells wide and simply is not there. The GPU does not care, so the cap can be lifted
//to where a pin is actually resolved.
//
//Why OFFSCREEN rather than a QOpenGLWidget: the page shows this image in graphicsViewFOV, the
//same view every other algo page draws into, with the ROI boxes, the overlay, the zoom state
//and the drag-to-spin all built around a QImage. Returning a QImage keeps every one of those
//working and makes this a drop-in for the CPU renderer - the GL is an implementation detail of
//one function rather than a second display path to keep in step.
//
//There is always a way back: algoH3GLAvailable() is false when no desktop GL 3.3 context can be
//made (an ANGLE-only build, a machine PC with no usable driver, or a call from a worker thread),
//and the caller falls back to the CPU renderer rather than showing a black canvas.
#include "AlgoHeight3Types.h"

#include <QImage>
#include <QSize>
#include <QString>
#include <vector>

/*
* One frame's worth of geometry, already decimated to the display grid and already hole-filled.
*
* Heights are in the map's own raw units and NaN means no data: a triangle with an unmeasured
* corner is not emitted, so a genuine gap stays a gap. zMin/zMax are the band the colour ramp
* spans - the caller passes the same min/max it would give the CPU renderer, so the two colour
* the part identically.
*/
struct AlgoH3GLScene {
	std::vector<float> z;             //w * h, row major, NaN = no data
	int w = 0;
	int h = 0;
	double zMin = 0.0;
	double zMax = 1.0;
	//optional grey texture on the SAME grid, for AlgoH3SurfaceStyle::Textured. Empty = none.
	std::vector<unsigned char> tex;
	//256-entry colour ramp (0xAARRGGBB) used when the style is not textured; the caller passes
	//the app's own JET table so the 3D view and every flat height view agree.
	const unsigned int* ramp = nullptr;
};

//Can this thread render? Cheap after the first call; the context is made once and reused.
bool algoH3GLAvailable();

//Why the GPU path is unavailable, for the log. Empty while it is working.
QString algoH3GLError();

/*
* Returns a QImage of outSize, or a null QImage if the GPU path is not usable - the caller then
* falls back to the CPU renderer. yaw/pitch/zExaggeration mean exactly what they mean to
* algoH3RenderSurface3D, and the projection is the same orthographic fit, so switching between
* the two renderers does not move the part on the canvas.
*/
QImage algoH3RenderSurfaceGL(const AlgoH3GLScene& scene, double yawDeg, double pitchDeg,
	double zExaggeration, const QSize& outSize, AlgoH3SurfaceStyle style);
