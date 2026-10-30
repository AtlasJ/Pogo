#pragma once
#include <QString>
#include <QVector>

/*
* Axis Move Sequence (Configurations page, machine-wide, system.json "Axis_Move_Sequence"): how a
* move that needs more than one axis goes out. Every group is sent, waited for and settled before
* the next one, so the axes of different groups never move at the same time.
*
* + Z is down. A move that takes Z down goes sideways first and Z last; any other move (Z up, or Z
* not changing) takes Z first and goes sideways after - the head never travels sideways lower than
* either end of the move.
*
* Why it exists: the motion accuracy test (28 Oct 2026) found X's reading stepping off by up to
* 0.4 mm when Z moved with another axis, so the order can be chosen per machine while that is
* chased with the motion team. XY together, Z alone is the default.
*/
namespace axisseq {

enum Sequence { OneAtATime = 0, XYTogetherZAlone = 1, AllTogether = 2, SequenceCount };

inline QString key(int s)
{
	switch (s) {
	case OneAtATime: return QStringLiteral("one_at_a_time");
	case AllTogether: return QStringLiteral("all_together");
	default: return QStringLiteral("xy_together_z_alone");
	}
}

inline int fromKey(const QString& k)
{
	for (int s = 0; s < SequenceCount; s++) if (key(s) == k) return s;
	return XYTogetherZAlone;
}

inline QString name(int s)
{
	switch (s) {
	case OneAtATime: return QStringLiteral("One axis at a time");
	case AllTogether: return QStringLiteral("All axes together");
	default: return QStringLiteral("XY together, Z alone");
	}
}

/*
* The axes of one move, in the groups they go out in. `axes` lists the axes that move (any of
* x, y, z); zDown = the move takes Z to a larger value. One at a time: X, Y, Z going down, else
* Z, Y, X. All together keeps the order given.
*/
inline QVector<QVector<int>> groups(int sequence, const QVector<int>& axes, bool zDown, int x, int y, int z)
{
	QVector<QVector<int>> out;
	if (sequence == AllTogether) {
		if (!axes.isEmpty()) out.push_back(axes);
		return out;
	}
	if (sequence == OneAtATime) {
		const QVector<int> order = zDown ? QVector<int>{ x, y, z } : QVector<int>{ z, y, x };
		for (int a : order) if (axes.contains(a)) out.push_back({ a });
		return out;
	}
	QVector<int> side;
	for (int a : axes) if (a != z) side.push_back(a);
	const bool hasZ = axes.contains(z);
	if (hasZ && !zDown) out.push_back({ z });
	if (!side.isEmpty()) out.push_back(side);
	if (hasZ && zDown) out.push_back({ z });
	return out;
}

} // namespace axisseq
