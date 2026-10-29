#pragma once
#include <cmath>
#include <vector>
#include <algorithm>

/*
* Guided Setup (new): the geometry, kept free of Qt so it can be checked on its own.
*
* Machine axes: +X is to the operator's right and +Y moves the gantry TOWARDS the operator, so the
* layout view (operator side at the bottom) draws +Y downwards. Z is not used here.
*
* The trolley frame has its origin at the fidRect centre, +x from fidTL towards fidTR and +y from
* fidTL towards fidBL, turned by the fidRect angle. The angle is in radians and positive when +x
* turns towards +Y, which is clockwise on screen.
*/
namespace gs2geo {

struct Pt { double x = 0.0, y = 0.0; };

//where a frame sits in the world: its origin and its angle
struct Pose { bool valid = false; double cx = 0.0, cy = 0.0, angle = 0.0; };

inline Pt toWorld(const Pose& p, Pt local)
{
	const double c = std::cos(p.angle), s = std::sin(p.angle);
	return { p.cx + c * local.x - s * local.y, p.cy + s * local.x + c * local.y };
}

inline Pt toLocal(const Pose& p, Pt world)
{
	const double c = std::cos(p.angle), s = std::sin(p.angle);
	const double dx = world.x - p.cx, dy = world.y - p.cy;
	return { c * dx + s * dy, -s * dx + c * dy };
}

inline double dist(Pt a, Pt b) { return std::hypot(a.x - b.x, a.y - b.y); }

enum Corner { TL = 0, TR, BR, BL };

//the corners of a w x l rectangle centred on c in its own frame, in TL TR BR BL order
inline void rectCorners(Pt c, double w, double l, Pt out[4])
{
	out[TL] = { c.x - w / 2, c.y - l / 2 };
	out[TR] = { c.x + w / 2, c.y - l / 2 };
	out[BR] = { c.x + w / 2, c.y + l / 2 };
	out[BL] = { c.x - w / 2, c.y + l / 2 };
}

/*
* Rigid fit (shift + rotation, no scale) of the drawing's w x l rectangle onto the 4 found corners.
* The centre is the centroid of the found points (the nominal one is the origin); the angle is the
* least-squares rotation. Each residual is how far that corner sits from where the fitted rectangle
* puts it. With w or l unknown (<= 0) the angle comes from the two width sides and the residuals
* stay 0. The measured width and length are the mean of the two opposite sides.
*/
struct RectFit {
	Pose pose;
	double residual[4] = { 0.0, 0.0, 0.0, 0.0 };
	double widthMeasured = 0.0, lengthMeasured = 0.0;
	double maxResidual() const { return std::max(std::max(residual[0], residual[1]), std::max(residual[2], residual[3])); }
};

inline RectFit fitRect(const Pt found[4], double w, double l)
{
	RectFit f;
	Pt c;
	for (int i = 0; i < 4; i++) { c.x += found[i].x / 4; c.y += found[i].y / 4; }

	double angle = 0.0;
	if (w > 0 && l > 0) {
		Pt n[4];
		rectCorners({}, w, l, n);
		double sDot = 0.0, sCross = 0.0;
		for (int i = 0; i < 4; i++) {
			const double dx = found[i].x - c.x, dy = found[i].y - c.y;
			sDot += n[i].x * dx + n[i].y * dy;
			sCross += n[i].x * dy - n[i].y * dx;
		}
		angle = std::atan2(sCross, sDot);
		f.pose = { true, c.x, c.y, angle };
		for (int i = 0; i < 4; i++) f.residual[i] = dist(toWorld(f.pose, n[i]), found[i]);
	}
	else {
		const double vx = (found[TR].x - found[TL].x) + (found[BR].x - found[BL].x);
		const double vy = (found[TR].y - found[TL].y) + (found[BR].y - found[BL].y);
		angle = std::atan2(vy, vx);
		f.pose = { true, c.x, c.y, angle };
	}
	f.widthMeasured = (dist(found[TL], found[TR]) + dist(found[BL], found[BR])) / 2;
	f.lengthMeasured = (dist(found[TL], found[BL]) + dist(found[TR], found[BR])) / 2;
	return f;
}

//a unit's centre in its slot's frame: a rows x cols grid centred on the slot centre, pitch = size +
//2 x margin. Row 0 is the far side (smallest y), column 0 the left (smallest x).
inline Pt unitCentre(int row, int col, int rows, int cols, double w, double l, double margin)
{
	const double px = w + 2 * margin, py = l + 2 * margin;
	return { (col - (cols - 1) / 2.0) * px, (row - (rows - 1) / 2.0) * py };
}

//true when every unit lies inside a slotW x slotL slot; the margins may stick out past its edge
inline bool unitsFit(int rows, int cols, double w, double l, double margin, double slotW, double slotL)
{
	if (rows < 1 || cols < 1 || w <= 0 || l <= 0 || margin < 0) return false;
	const double ex = (cols - 1) * (w + 2 * margin) + w;
	const double ey = (rows - 1) * (l + 2 * margin) + l;
	return ex <= slotW + 1e-9 && ey <= slotL + 1e-9;
}

//true when two equal slots (same angle, compared in the trolley frame) touch or overlap
inline bool slotsTouch(Pt a, Pt b, double w, double l)
{
	return std::fabs(a.x - b.x) <= w && std::fabs(a.y - b.y) <= l;
}

//a unit's reference point in its own frame: 0 = centre, 1..4 = TL TR BR BL corner
inline Pt unitRefPoint(int ref, double w, double l)
{
	switch (ref) {
	case 1: return { -w / 2, -l / 2 };
	case 2: return { w / 2, -l / 2 };
	case 3: return { w / 2, l / 2 };
	case 4: return { -w / 2, l / 2 };
	default: return {};
	}
}

/*
* Groups values into bands: sorted ascending, a new band starts where the gap to the previous value
* is more than tol. Returns each value's band index (0-based) in input order. Used for the
* trolley-wide name grid (columns by x, rows by y) and for telling slot rows apart.
*/
inline std::vector<int> bandIndex(const std::vector<double>& v, double tol)
{
	std::vector<int> order(v.size());
	for (size_t i = 0; i < v.size(); i++) order[i] = (int)i;
	std::sort(order.begin(), order.end(), [&](int a, int b) { return v[a] < v[b]; });
	std::vector<int> band(v.size(), 0);
	int b = 0;
	for (size_t k = 0; k < order.size(); k++) {
		if (k > 0 && v[order[k]] - v[order[k - 1]] > tol) b++;
		band[order[k]] = b;
	}
	return band;
}

} // namespace gs2geo
