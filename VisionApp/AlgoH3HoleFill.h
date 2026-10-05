#pragma once
//Close the dropouts the profiler leaves on an obstructed surface.
//
//On a pin field the sensor cannot see every part of every pin: a neighbouring pin shadows
//the laser line, or the camera's view of it, so parts of a pin's own cap and flank come
//back with no height at all. Those gaps sit INSIDE the pin, ringed by valid pin surface.
//Left alone they take a bite out of the pin in every 3D view, because a quad with one
//unmeasured corner cannot be drawn - so the taller and more crowded the pins, the more of
//them is missing exactly where the operator is looking.
//
//The space BETWEEN pins is a different thing entirely. It is background: there is nothing
//there to measure, and filling it welds the pins into a slab. So a gap is only filled when
//it is enclosed - when it does not reach the edge of the grid - and, optionally, when it is
//no bigger than maxHoleArea. Everything else is left as no data and still renders as a gap.
//
//The fill is two dimensional and has no direction: each hole is solved from its whole rim at
//once. A row-wise "inherit the last valid pixel" rule is the obvious cheap alternative and it
//is wrong here - it drags a feature's edge sideways across the shadow and builds a wall,
//which on a pin field is the artefact it was meant to avoid.
//
//Pure C++ - no Qt, no OpenCV - so it can be exercised on its own. Heights are a row-major
//grid and NaN means no data.
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace h3fill {

const float kNaN = std::numeric_limits<float>::quiet_NaN();
inline bool valid(float v) { return v == v; }

enum class Mode {
	//Solve a smooth surface through the hole that meets the rim at the rim's own slope, so a
	//shadow on a domed or chamfered pin head keeps the curvature around it.
	Interpolate,
	//One flat level per hole, the median of its rim. A flat pin cap stays flat and cannot bow.
	RimMedian,
};

struct Params {
	Mode mode = Mode::Interpolate;
	//0 = fill an enclosed hole of any size. Set it when a large enclosed region is more likely
	//to be a genuinely unmeasured area than one pin's shadow.
	int maxHoleArea = 0;
	//A hole touching the border has no rim on that side, so there is nothing to interpolate
	//from; it is treated as background unless this says otherwise.
	bool fillBorderHoles = false;
	//0 = scaled from the widest hole: relaxation needs roughly as many sweeps as a hole is wide.
	int maxIters = 0;
	//stop once no cell in a sweep moves more than this, in the grid's own height units
	float tol = 0.05f;
};

struct Report {
	int holes = 0;          //components found, filled and left open
	int filled = 0;         //components actually filled
	int filledCells = 0;
	int openCells = 0;      //still no data afterwards - background, drawn as a gap
	int largestHole = 0;    //cells in the largest component seen
	int iterations = 0;
};

namespace detail {

//Nearest valid height, by a two-pass chamfer sweep. Only a starting guess - it is blocky, and
//the relaxation below is what smooths it - but it puts every hole cell near its answer, so the
//relaxation converges in a fraction of the sweeps a flat start would need.
inline void seedNearest(const std::vector<float>& z, int w, int h, std::vector<float>& seed)
{
	const size_t n = (size_t)w * h;
	std::vector<float> d(n, 1e30f);
	seed.assign(n, 0.0f);
	for (size_t i = 0; i < n; i++) if (valid(z[i])) { d[i] = 0.0f; seed[i] = z[i]; }

	auto relax = [&](int i, int j, float cost) {
		if (d[j] + cost < d[i]) { d[i] = d[j] + cost; seed[i] = seed[j]; }
	};
	for (int y = 0; y < h; y++) for (int x = 0; x < w; x++) {
		const int i = y * w + x;
		if (d[i] == 0.0f) continue;
		if (x > 0)               relax(i, i - 1, 1.0f);
		if (y > 0)               relax(i, i - w, 1.0f);
		if (x > 0 && y > 0)      relax(i, i - w - 1, 1.41421f);
		if (x < w - 1 && y > 0)  relax(i, i - w + 1, 1.41421f);
	}
	for (int y = h - 1; y >= 0; y--) for (int x = w - 1; x >= 0; x--) {
		const int i = y * w + x;
		if (d[i] == 0.0f) continue;
		if (x < w - 1)              relax(i, i + 1, 1.0f);
		if (y < h - 1)              relax(i, i + w, 1.0f);
		if (x < w - 1 && y < h - 1) relax(i, i + w + 1, 1.41421f);
		if (x > 0 && y < h - 1)     relax(i, i + w - 1, 1.41421f);
	}
}

} //namespace detail

/*
* Fills z in place and reports what it did.
*
* The report is worth showing: a part whose gaps are nearly all "open" is one the fill decided
* it could not help with, and that is a staging or exposure problem rather than a display one.
*/
inline Report algoH3FillHoles(std::vector<float>& z, int w, int h, const Params& prm = Params())
{
	Report rep;
	if (w <= 0 || h <= 0 || (long long)z.size() < (long long)w * h) return rep;
	const size_t n = (size_t)w * h;

	// ── 1. label the gaps ────────────────────────────────────────────────────
	//0 = valid data, -1 = gap left open, > 0 = fillable hole id
	std::vector<int> label(n, 0);
	std::vector<int> stack, cells;

	for (size_t s = 0; s < n; s++) {
		if (valid(z[s]) || label[s] != 0) continue;

		stack.clear(); cells.clear();
		stack.push_back((int)s);
		label[s] = -2;                        //claimed, decision pending
		bool touchesBorder = false;

		while (!stack.empty()) {
			const int i = stack.back(); stack.pop_back();
			cells.push_back(i);
			const int x = i % w, y = i / w;
			if (x == 0 || y == 0 || x == w - 1 || y == h - 1) touchesBorder = true;

			const int nb[4] = { x > 0 ? i - 1 : -1, x < w - 1 ? i + 1 : -1,
								y > 0 ? i - w : -1, y < h - 1 ? i + w : -1 };
			for (int j : nb) {
				if (j < 0 || valid(z[j]) || label[j] != 0) continue;
				label[j] = -2;
				stack.push_back(j);
			}
		}

		const int area = (int)cells.size();
		rep.holes++;
		rep.largestHole = std::max(rep.largestHole, area);

		const bool open = (touchesBorder && !prm.fillBorderHoles)
			|| (prm.maxHoleArea > 0 && area > prm.maxHoleArea);
		if (open) {
			for (int i : cells) label[i] = -1;
			rep.openCells += area;
		}
		else {
			const int id = ++rep.filled;
			for (int i : cells) label[i] = id;
			rep.filledCells += area;
		}
	}
	if (rep.filledCells == 0) return rep;

	// ── 2. one flat level per hole ───────────────────────────────────────────
	if (prm.mode == Mode::RimMedian) {
		//the median of the valid 8-neighbours, not the mean, so a single rim cell that caught
		//the flank below does not pull the whole cap down with it
		std::vector<std::vector<float>> rim((size_t)rep.filled + 1);
		for (size_t i = 0; i < n; i++) {
			if (label[i] <= 0) continue;
			const int x = (int)(i % w), y = (int)(i / w);
			for (int dy = -1; dy <= 1; dy++) for (int dx = -1; dx <= 1; dx++) {
				const int nx = x + dx, ny = y + dy;
				if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
				const float v = z[(size_t)ny * w + nx];
				if (valid(v)) rim[label[i]].push_back(v);
			}
		}
		for (size_t id = 1; id < rim.size(); id++) {
			if (rim[id].empty()) continue;
			auto mid = rim[id].begin() + rim[id].size() / 2;
			std::nth_element(rim[id].begin(), mid, rim[id].end());
			const float level = *mid;
			for (size_t i = 0; i < n; i++) if (label[i] == (int)id) z[i] = level;
		}
		return rep;
	}

	// ── 3. smooth surface through each hole ──────────────────────────────────
	std::vector<float> seed;
	detail::seedNearest(z, w, h, seed);

	std::vector<int> idx;
	idx.reserve(rep.filledCells);
	for (size_t i = 0; i < n; i++) if (label[i] > 0) { z[i] = seed[i]; idx.push_back((int)i); }

	//Laplace relaxation over the hole cells only, with the measured heights around them held
	//fixed as the boundary - the discrete form of "the smoothest surface that meets this rim".
	//Over-relaxed, because plain Gauss-Seidel needs the square of the sweeps on a wide hole.
	const int iters = prm.maxIters > 0 ? prm.maxIters
		: std::min(2000, std::max(32, (int)(10.0 * std::sqrt((double)rep.largestHole))));
	const float omega = 1.8f;

	for (int it = 0; it < iters; it++) {
		float moved = 0.0f;
		for (int i : idx) {
			const int x = i % w, y = i / w;
			float sum = 0.0f; int cnt = 0;
			//a neighbour that is still no data - an open gap next door - contributes nothing,
			//rather than poisoning the average with a NaN
			if (x > 0)     { const float v = z[i - 1]; if (valid(v)) { sum += v; cnt++; } }
			if (x < w - 1) { const float v = z[i + 1]; if (valid(v)) { sum += v; cnt++; } }
			if (y > 0)     { const float v = z[i - w]; if (valid(v)) { sum += v; cnt++; } }
			if (y < h - 1) { const float v = z[i + w]; if (valid(v)) { sum += v; cnt++; } }
			if (cnt == 0) continue;

			const float next = z[i] + omega * (sum / cnt - z[i]);
			moved = std::max(moved, std::fabs(next - z[i]));
			z[i] = next;
		}
		rep.iterations++;
		if (moved < prm.tol) break;
	}
	return rep;
}

} //namespace h3fill
