#pragma once
#include <QWidget>
#include <QPolygonF>
#include "GuidedSetup2Types.h"

/*
* Guided Setup (new): the layout drawing that the big image area switches to. It shows the World
* (the X/Y soft limits), fid0, the four corners, every slot, unit and inspection point, and the live
* camera position with its field of view.
*
* Drawn the way the operator stands: operator side at the bottom, +X to the right, +Y (towards the
* operator) downwards. Until a Locate finds the trolley it is drawn where it was taught.
*
* Nothing can be dragged. Click selects a slot or unit; the wheel zooms round the cursor, dragging
* pans, a double-click fits everything back in.
*/
class GuidedSetup2View : public QWidget
{
	Q_OBJECT

public:
	explicit GuidedSetup2View(QWidget* parent = nullptr);

	void setSetup(const gs2::Setup& s);
	void setWorld(bool known, double xLo, double xHi, double yLo, double yHi);
	void setCamera(bool known, double x, double y, double fovWmm, double fovHmm);

protected:
	void paintEvent(QPaintEvent*) override;
	void mousePressEvent(QMouseEvent* e) override;
	void mouseMoveEvent(QMouseEvent* e) override;
	void mouseReleaseEvent(QMouseEvent* e) override;
	void mouseDoubleClickEvent(QMouseEvent* e) override;
	void wheelEvent(QWheelEvent* e) override;

private:
	gs2::Setup _s;
	gs2geo::RectFit _fit;            //the taught fidRect pose
	bool _worldKnown = false;
	QRectF _world;                   //soft limits in mm (x = X, y = Y)
	bool _camKnown = false;
	double _camX = 0.0, _camY = 0.0, _fovW = 0.0, _fovH = 0.0;

	double _zoom = 1.0;
	QPointF _pan;                    //screen px added after the fit
	bool _pressed = false, _dragged = false;
	QPoint _pressPos;
	QPointF _panAtPress;

	QString _selKind, _selId;        //"slot" / "unit" and its id; empty = nothing selected

	QRectF contentMm() const;        //what has to fit on screen: the World, else everything drawn
	double baseScale(const QRectF& r) const;
	QPointF toScreen(gs2geo::Pt p) const;
	QPointF toScreen(double x, double y) const { return toScreen(gs2geo::Pt{ x, y }); }
	QPolygonF screenPoly(const gs2geo::Pt pts[4]) const;
	void pick(QPointF screen);
	QString selectionText() const;
};
