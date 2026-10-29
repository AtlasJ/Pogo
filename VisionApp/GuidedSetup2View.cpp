//
//  Guided Setup (new): the layout drawing. See GuidedSetup2View.h.
//
#include "GuidedSetup2View.h"

#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QWheelEvent>
#include <cmath>

namespace {

const QColor kBackground(22, 25, 31);
const QColor kWorldFill(29, 34, 43);
const QColor kWorldLine(74, 84, 102);
const QColor kText(230, 234, 240);
const QColor kHint(157, 167, 183);
const QColor kFid(255, 201, 51);
const QColor kFrame(136, 169, 224);
const QColor kSlot(111, 156, 226);
const QColor kUnit(79, 194, 180);
const QColor kInsp(242, 108, 176);
const QColor kCamera(255, 231, 35);
const QColor kSelected(255, 255, 255);

const double kMarginPx = 36.0;

QString mm3(double v) { return QString::number(v, 'f', 3); }

} // namespace

GuidedSetup2View::GuidedSetup2View(QWidget* parent)
	: QWidget(parent)
{
	setMouseTracking(false);
	setAutoFillBackground(false);
	setMinimumSize(200, 200);
}

void GuidedSetup2View::setSetup(const gs2::Setup& s)
{
	_s = s;
	_fit = gs2::taughtFit(s);
	//a selection that no longer exists is dropped
	bool still = false;
	if (_selKind == QLatin1String("slot")) still = gs2::findSlot(_s, _selId) != nullptr;
	if (_selKind == QLatin1String("unit")) for (const auto& u : _s.units) still = still || u.id == _selId;
	if (!still) { _selKind.clear(); _selId.clear(); }
	update();
}

void GuidedSetup2View::setWorld(bool known, double xLo, double xHi, double yLo, double yHi)
{
	_worldKnown = known && xLo < xHi && yLo < yHi;
	_world = _worldKnown ? QRectF(QPointF(xLo, yLo), QPointF(xHi, yHi)) : QRectF();
	update();
}

void GuidedSetup2View::setCamera(bool known, double x, double y, double fovWmm, double fovHmm)
{
	if (known == _camKnown && x == _camX && y == _camY && fovWmm == _fovW && fovHmm == _fovH) return;
	_camKnown = known;
	_camX = x; _camY = y; _fovW = fovWmm; _fovH = fovHmm;
	update();
}

QRectF GuidedSetup2View::contentMm() const
{
	QRectF r = _worldKnown ? _world : QRectF();
	auto add = [&r](double x, double y) {
		const QRectF p(QPointF(x, y), QSizeF(1e-6, 1e-6));
		r = r.isNull() ? p : r.united(p);
	};
	if (_s.fid0.xy.set) add(_s.fid0.xy.x, _s.fid0.xy.y);
	for (int i = 0; i < 4; i++)
		if (_s.fid0.xy.set && _s.fidRect.corner[i].set) add(_s.fid0.xy.x + _s.fidRect.corner[i].x, _s.fid0.xy.y + _s.fidRect.corner[i].y);
	if (_fit.pose.valid)
		for (const auto& sl : _s.slotList) {
			gs2geo::Pt c[4];
			gs2::slotCornersWorld(_s, _fit.pose, sl, c);
			for (const auto& p : c) add(p.x, p.y);
		}
	if (_camKnown) add(_camX, _camY);
	if (r.isNull()) r = QRectF(-100, -100, 200, 200);
	const double m = std::max(5.0, 0.05 * std::max(r.width(), r.height()));
	if (!_worldKnown) r.adjust(-m, -m, m, m);
	if (r.width() < 1.0) r.adjust(-50, 0, 50, 0);
	if (r.height() < 1.0) r.adjust(0, -50, 0, 50);
	return r;
}

double GuidedSetup2View::baseScale(const QRectF& r) const
{
	const double w = std::max(1.0, width() - 2 * kMarginPx);
	const double h = std::max(1.0, height() - 2 * kMarginPx - 24.0);   //room for the operator-side line
	return std::min(w / r.width(), h / r.height());
}

QPointF GuidedSetup2View::toScreen(gs2geo::Pt p) const
{
	const QRectF r = contentMm();
	const double s = baseScale(r);
	const double ox = (width() - r.width() * s) / 2, oy = (height() - 24.0 - r.height() * s) / 2;
	const QPointF base(ox + (p.x - r.left()) * s, oy + (p.y - r.top()) * s);   //+Y downwards
	const QPointF c(width() / 2.0, height() / 2.0);
	return c + (base - c) * _zoom + _pan;
}

QPolygonF GuidedSetup2View::screenPoly(const gs2geo::Pt pts[4]) const
{
	QPolygonF poly;
	for (int i = 0; i < 4; i++) poly << toScreen(pts[i]);
	return poly;
}

void GuidedSetup2View::paintEvent(QPaintEvent*)
{
	QPainter p(this);
	p.setRenderHint(QPainter::Antialiasing, true);
	p.fillRect(rect(), kBackground);

	QFont small = font();
	small.setPointSizeF(8.0);
	QFont normal = font();
	normal.setPointSizeF(9.0);
	QFont bold = normal;
	bold.setBold(true);

	//the World
	if (_worldKnown) {
		const QRectF wr(toScreen(_world.left(), _world.top()), toScreen(_world.right(), _world.bottom()));
		p.setPen(QPen(kWorldLine, 1.2));
		p.setBrush(kWorldFill);
		p.drawRect(wr.normalized());
	}

	const auto& pose = _fit.pose;
	auto label = [&](QPointF at, const QString& text, const QColor& colour, const QFont& f, int flags = Qt::AlignLeft | Qt::AlignBottom) {
		p.setFont(f);
		p.setPen(colour);
		QRectF box(at.x(), at.y() - 16, 220, 16);
		if (flags & Qt::AlignHCenter) box.moveLeft(at.x() - 110);
		if (flags & Qt::AlignRight) box.moveRight(at.x());
		if (flags & Qt::AlignTop) box.moveTop(at.y());
		if (flags & Qt::AlignVCenter) box.moveTop(at.y() - 8);
		p.drawText(box, flags, text);
	};

	//slots and units
	if (pose.valid) {
		for (const auto& sl : _s.slotList) {
			gs2geo::Pt c[4];
			gs2::slotCornersWorld(_s, pose, sl, c);
			const QPolygonF poly = screenPoly(c);
			const bool sel = _selKind == QLatin1String("slot") && _selId == sl.id;
			p.setPen(QPen(sel ? kSelected : kSlot, sel ? 2.5 : 1.5));
			QColor fill = kSlot; fill.setAlpha(45);
			p.setBrush(fill);
			p.drawPolygon(poly);
		}
		const auto& ul = _s.unitLayout;
		for (const auto& u : _s.units) {
			const gs2::Slot* sl = gs2::findSlot(_s, u.slotId);
			if (!sl || ul.widthMm <= 0.0 || ul.lengthMm <= 0.0) continue;
			gs2geo::Pt c[4];
			gs2::unitCornersWorld(_s, pose, *sl, u.row, u.col, c);
			const QPolygonF poly = screenPoly(c);
			const bool sel = _selKind == QLatin1String("unit") && _selId == u.id;
			p.setPen(QPen(sel ? kSelected : kUnit, sel ? 2.2 : 1.0));
			QColor fill = kUnit; fill.setAlpha(sel ? 110 : 60);
			p.setBrush(fill);
			p.drawPolygon(poly);
			const QRectF b = poly.boundingRect();
			if (b.width() > 38 && b.height() > 14) {
				p.setFont(small);
				p.setPen(kText);
				p.drawText(b, Qt::AlignCenter, u.name);
			}

			//inspection points
			p.setBrush(Qt::NoBrush);
			p.setPen(QPen(kInsp, 1.4));
			for (int t = 0; t < gs2::InspTypeCount; t++) {
				if (!_s.insp[t].enabled) continue;
				const gs2geo::Pt w = gs2::inspectionWorld(_s, pose, *sl, u.row, u.col, (gs2::InspType)t);
				const QPointF q = toScreen(w);
				if (t == gs2::Insp2D) {
					p.drawLine(q + QPointF(-4, 0), q + QPointF(4, 0));
					p.drawLine(q + QPointF(0, -4), q + QPointF(0, 4));
				}
				else if (t == gs2::InspBarcodeOcr) {
					p.drawRect(QRectF(q.x() - 3, q.y() - 3, 6, 6));
				}
				else {
					//the scan runs along a machine axis from its start point
					const double len = _s.insp[t].scanLengthMm * (_s.scanNegative ? -1.0 : 1.0);
					const gs2geo::Pt end = _s.scanAxis == QLatin1String("Y") ? gs2geo::Pt{ w.x, w.y + len } : gs2geo::Pt{ w.x + len, w.y };
					p.drawLine(q, toScreen(end));
					p.drawEllipse(q, 2.0, 2.0);
				}
			}
		}
		//slot names on top of their units
		for (const auto& sl : _s.slotList) {
			gs2geo::Pt c[4];
			gs2::slotCornersWorld(_s, pose, sl, c);
			const QRectF b = screenPoly(c).boundingRect();
			if (b.width() > 24) label(QPointF(b.center().x(), b.top() - 2), sl.name, kSlot, bold, Qt::AlignHCenter | Qt::AlignBottom);
		}

		//the fidRect and the trolley frame at its centre
		QPolygonF rectPoly;
		if (_s.fidRect.widthMm > 0.0 && _s.fidRect.lengthMm > 0.0) {
			gs2geo::Pt n[4];
			gs2geo::rectCorners({}, _s.fidRect.widthMm, _s.fidRect.lengthMm, n);
			for (auto& q : n) q = gs2geo::toWorld(pose, q);
			rectPoly = screenPoly(n);
		}
		else {
			for (int i = 0; i < 4; i++)
				rectPoly << toScreen(_s.fid0.xy.x + _s.fidRect.corner[i].x, _s.fid0.xy.y + _s.fidRect.corner[i].y);
		}
		p.setBrush(Qt::NoBrush);
		p.setPen(QPen(kFid, 1.2, Qt::DashLine));
		p.drawPolygon(rectPoly);

		const QPointF o = toScreen(gs2geo::Pt{ pose.cx, pose.cy });
		const double axisLen = 28.0;
		const QPointF ax(std::cos(pose.angle) * axisLen, std::sin(pose.angle) * axisLen);   //+x' on screen
		const QPointF ay(-std::sin(pose.angle) * axisLen, std::cos(pose.angle) * axisLen);  //+y' on screen (down)
		p.setPen(QPen(kFrame, 1.8));
		p.drawLine(o, o + ax);
		p.drawLine(o, o + ay);
		p.drawLine(o + QPointF(-5, 0), o + QPointF(5, 0));
		p.drawLine(o + QPointF(0, -5), o + QPointF(0, 5));
		label(o + ax + QPointF(3, 6), QStringLiteral("x'"), kFrame, small);
		label(o + ay + QPointF(3, 10), QStringLiteral("y'"), kFrame, small);
	}

	//fid0, the corners and the 3D Z spot
	if (_s.fid0.xy.set) {
		static const char* const cornerName[4] = { "fidTL", "fidTR", "fidBR", "fidBL" };
		for (int i = 0; i < 4; i++) {
			if (!_s.fidRect.corner[i].set) continue;
			const QPointF q = toScreen(_s.fid0.xy.x + _s.fidRect.corner[i].x, _s.fid0.xy.y + _s.fidRect.corner[i].y);
			p.setPen(QPen(kFid, 2.0));
			p.setBrush(kBackground);
			p.drawEllipse(q, 6.0, 6.0);
			p.setPen(QPen(kText, 1.0));
			p.drawLine(q + QPointF(-3, 0), q + QPointF(3, 0));
			p.drawLine(q + QPointF(0, -3), q + QPointF(0, 3));
			const bool right = i == gs2geo::TR || i == gs2geo::BR;
			const bool bottom = i == gs2geo::BR || i == gs2geo::BL;
			label(q + QPointF(right ? 9 : -9, bottom ? 18 : -6), QLatin1String(cornerName[i]), kFid, small,
				(right ? Qt::AlignLeft : Qt::AlignRight) | Qt::AlignBottom);
		}
		if (_s.z3d.offset.set) {
			const QPointF q = toScreen(_s.fid0.xy.x + _s.z3d.offset.x, _s.fid0.xy.y + _s.z3d.offset.y);
			QPolygonF d;
			d << q + QPointF(0, -6) << q + QPointF(6, 0) << q + QPointF(0, 6) << q + QPointF(-6, 0);
			p.setPen(QPen(kFid, 1.8));
			p.setBrush(kBackground);
			p.drawPolygon(d);
			label(q + QPointF(-9, 4), QStringLiteral("3D Z"), kFid, small, Qt::AlignRight | Qt::AlignVCenter);
		}
		const QPointF f = toScreen(_s.fid0.xy.x, _s.fid0.xy.y);
		p.setPen(QPen(kFid, 2.0));
		p.setBrush(kFid);
		p.drawEllipse(f, 7.0, 7.0);
		label(f + QPointF(-11, 4), QStringLiteral("fid0"), kFid, bold, Qt::AlignRight | Qt::AlignVCenter);
	}

	//the camera
	if (_camKnown) {
		const QPointF q = toScreen(_camX, _camY);
		p.setBrush(Qt::NoBrush);
		if (_fovW > 0.0 && _fovH > 0.0) {
			p.setPen(QPen(kCamera, 1.0, Qt::DashLine));
			p.drawRect(QRectF(toScreen(_camX - _fovW / 2, _camY - _fovH / 2), toScreen(_camX + _fovW / 2, _camY + _fovH / 2)).normalized());
		}
		p.setPen(QPen(kCamera, 1.6));
		p.drawLine(q + QPointF(-10, 0), q + QPointF(10, 0));
		p.drawLine(q + QPointF(0, -10), q + QPointF(0, 10));
		label(q + QPointF(12, -4), QStringLiteral("Camera %1, %2").arg(mm3(_camX), mm3(_camY)), kCamera, small);
	}

	//status text, top left
	QStringList lines;
	lines << (_worldKnown
		? QStringLiteral("World (soft limits): X %1 .. %2   Y %3 .. %4 mm").arg(mm3(_world.left()), mm3(_world.right()), mm3(_world.top()), mm3(_world.bottom()))
		: QStringLiteral("Soft limits unknown (motion not loaded): drawn around the taught layout"));
	if (!_s.fid0.xy.set) lines << QStringLiteral("Nothing taught yet: teach fid0 and the four corners first.");
	else if (!pose.valid) lines << QStringLiteral("Teach all four corners to place the slots and units.");
	else lines << QStringLiteral("Drawn where it was taught (trolley angle %1 deg)").arg(QString::number(pose.angle * 180.0 / 3.14159265358979323846, 'f', 3));
	const QString sel = selectionText();
	if (!sel.isEmpty()) lines << sel;
	p.setFont(normal);
	p.setPen(kHint);
	p.drawText(QRectF(10, 6, width() - 170, 60), Qt::AlignLeft | Qt::AlignTop | Qt::TextWordWrap, lines.join(QLatin1Char('\n')));   //clear of the legend

	//legend, top right
	struct Key { QColor c; const char* t; };
	const Key keys[] = { { kFid, "fiducials" }, { kSlot, "slots" }, { kUnit, "units" }, { kInsp, "inspection points" }, { kCamera, "camera" } };
	p.setFont(small);
	double ly = 10;
	for (const auto& k : keys) {
		p.setPen(Qt::NoPen);
		p.setBrush(k.c);
		p.drawRect(QRectF(width() - 150, ly + 2, 10, 8));
		p.setPen(kHint);
		p.drawText(QRectF(width() - 134, ly - 2, 130, 16), Qt::AlignLeft | Qt::AlignVCenter, QLatin1String(k.t));
		ly += 15;
	}

	//operator side and axes, bottom
	p.setFont(normal);
	p.setPen(kHint);
	p.drawText(QRectF(0, height() - 22, width(), 20), Qt::AlignCenter, QStringLiteral("Operator side"));
	const QPointF a0(18, height() - 50);
	p.setPen(QPen(kHint, 1.4));
	p.drawLine(a0, a0 + QPointF(30, 0));
	p.drawLine(a0 + QPointF(30, 0), a0 + QPointF(25, -3));
	p.drawLine(a0 + QPointF(30, 0), a0 + QPointF(25, 3));
	p.drawLine(a0, a0 + QPointF(0, 30));
	p.drawLine(a0 + QPointF(0, 30), a0 + QPointF(-3, 25));
	p.drawLine(a0 + QPointF(0, 30), a0 + QPointF(3, 25));
	p.setFont(small);
	p.drawText(QRectF(a0.x() + 34, a0.y() - 8, 30, 16), Qt::AlignLeft | Qt::AlignVCenter, QStringLiteral("+X"));
	p.drawText(QRectF(a0.x() + 6, a0.y() + 22, 40, 16), Qt::AlignLeft | Qt::AlignVCenter, QStringLiteral("+Y"));
}

void GuidedSetup2View::pick(QPointF screen)
{
	_selKind.clear();
	_selId.clear();
	if (_fit.pose.valid) {
		for (const auto& u : _s.units) {
			const gs2::Slot* sl = gs2::findSlot(_s, u.slotId);
			if (!sl) continue;
			gs2geo::Pt c[4];
			gs2::unitCornersWorld(_s, _fit.pose, *sl, u.row, u.col, c);
			if (screenPoly(c).containsPoint(screen, Qt::OddEvenFill)) { _selKind = QStringLiteral("unit"); _selId = u.id; break; }
		}
		if (_selKind.isEmpty())
			for (const auto& sl : _s.slotList) {
				gs2geo::Pt c[4];
				gs2::slotCornersWorld(_s, _fit.pose, sl, c);
				if (screenPoly(c).containsPoint(screen, Qt::OddEvenFill)) { _selKind = QStringLiteral("slot"); _selId = sl.id; break; }
			}
	}
	update();
}

QString GuidedSetup2View::selectionText() const
{
	if (!_fit.pose.valid) return QString();
	if (_selKind == QLatin1String("slot")) {
		const gs2::Slot* sl = gs2::findSlot(_s, _selId);
		if (!sl) return QString();
		const gs2geo::Pt w = gs2geo::toWorld(_fit.pose, { sl->cx, sl->cy });
		return QStringLiteral("Selected slot %1: centre X %2  Y %3 (as taught), x %4  y %5 in the trolley frame")
			.arg(sl->name, mm3(w.x), mm3(w.y), mm3(sl->cx), mm3(sl->cy));
	}
	if (_selKind == QLatin1String("unit")) {
		for (const auto& u : _s.units) {
			if (u.id != _selId) continue;
			const gs2::Slot* sl = gs2::findSlot(_s, u.slotId);
			if (!sl) return QString();
			const gs2geo::Pt w = gs2geo::toWorld(_fit.pose, gs2::unitCentreLocal(_s, *sl, u.row, u.col));
			return QStringLiteral("Selected unit %1 in %2 (row %3, column %4): centre X %5  Y %6 (as taught)")
				.arg(u.name, sl->name).arg(u.row + 1).arg(u.col + 1).arg(mm3(w.x), mm3(w.y));
		}
	}
	return QString();
}

void GuidedSetup2View::mousePressEvent(QMouseEvent* e)
{
	_pressed = true;
	_dragged = false;
	_pressPos = e->pos();
	_panAtPress = _pan;
}

void GuidedSetup2View::mouseMoveEvent(QMouseEvent* e)
{
	if (!_pressed) return;
	const QPoint d = e->pos() - _pressPos;
	if (!_dragged && d.manhattanLength() < 4) return;
	_dragged = true;
	_pan = _panAtPress + QPointF(d);
	setCursor(Qt::ClosedHandCursor);
	update();
}

void GuidedSetup2View::mouseReleaseEvent(QMouseEvent* e)
{
	if (_pressed && !_dragged && e->button() == Qt::LeftButton) pick(e->pos());
	_pressed = false;
	_dragged = false;
	unsetCursor();
}

void GuidedSetup2View::mouseDoubleClickEvent(QMouseEvent*)
{
	_zoom = 1.0;
	_pan = QPointF();
	update();
}

void GuidedSetup2View::wheelEvent(QWheelEvent* e)
{
	const double steps = e->angleDelta().y() / 120.0;
	if (steps == 0.0) return;
	const double next = std::min(200.0, std::max(0.2, _zoom * std::pow(1.25, steps)));
	//keep the point under the cursor where it is
	const QPointF c(width() / 2.0, height() / 2.0);
	const QPointF at = e->posF();
	_pan = at - c - (at - c - _pan) * (next / _zoom);
	_zoom = next;
	update();
}
