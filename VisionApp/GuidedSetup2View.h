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
* Nothing can be dragged. Click selects the unit or slot under the cursor, Ctrl+click adds or removes
* a slot - the page's slot list follows (slotsPicked) and is followed (setSelectedSlots). The wheel
* zooms round the cursor, dragging pans, a double-click fits everything back in.
*/
class GuidedSetup2View : public QWidget
{
	Q_OBJECT

public:
	explicit GuidedSetup2View(QWidget* parent = nullptr);

	void setSetup(const gs2::Setup& s);
	void setWorld(bool known, double xLo, double xHi, double yLo, double yHi);
	void setCamera(bool known, double x, double y, double fovWmm, double fovHmm);
	//a Check or Locate found the trolley: fid0, the corners (with their residuals) and everything
	//placed from the fidRect are drawn there, not where they were taught, until clearLocated()
	void setLocated(const gs2geo::RectFit& fit, gs2geo::Pt fid0, const gs2geo::Pt corners[4], const QString& what);
	void clearLocated();
	//the slots selected in the page's list; a selected unit stays only while its slot is one of them
	void setSelectedSlots(const QStringList& ids);
	QString selectedUnit() const { return _selUnit; }   //the unit a plain click picked, else empty
	//the machine's camera-to-laser XY: a 3D scan whose recipe does not add it is drawn where the laser measures
	void setLaserOffset(double wx, double wy) { if (wx != _laserWx || wy != _laserWy) { _laserWx = wx; _laserWy = wy; update(); } }

signals:
	void slotsPicked(const QStringList& ids);   //a click here changed which slots are selected

protected:
	void paintEvent(QPaintEvent*) override;
	void mousePressEvent(QMouseEvent* e) override;
	void mouseMoveEvent(QMouseEvent* e) override;
	void mouseReleaseEvent(QMouseEvent* e) override;
	void mouseDoubleClickEvent(QMouseEvent* e) override;
	void wheelEvent(QWheelEvent* e) override;

private:
	gs2::Setup _s;
	gs2geo::RectFit _fit;            //the fidRect pose drawn: where it was found, else where it was taught
	bool _located = false;
	gs2geo::RectFit _locFit;
	gs2geo::Pt _locFid0, _locCorner[4];
	QString _locWhat;                //"Check 10:41", "Locate 10:44 (FAILED)"
	bool fid0Shown() const { return _located || _s.fid0.xy.set; }
	gs2geo::Pt fid0At() const;       //where fid0 is drawn: found, else its search XY
	bool cornerShown(int i) const { return _located || (_s.fid0.xy.set && _s.fidRect.corner[i].set); }
	gs2geo::Pt cornerAt(int i) const; //where a corner is drawn: found, else fid0 + its offset
	bool _worldKnown = false;
	QRectF _world;                   //soft limits in mm (x = X, y = Y)
	bool _camKnown = false;
	double _camX = 0.0, _camY = 0.0, _fovW = 0.0, _fovH = 0.0;

	double _zoom = 1.0;
	QPointF _pan;                    //screen px added after the fit
	bool _pressed = false, _dragged = false;
	QPoint _pressPos;
	QPointF _panAtPress;

	double _laserWx = 0.0, _laserWy = 0.0;
	QStringList _selSlots;           //selected slot ids (Ctrl+click: several)
	QString _selUnit;                //a selected unit's id, picked by a plain click

	QRectF contentMm() const;        //what has to fit on screen: the World, else everything drawn
	double baseScale(const QRectF& r) const;
	QPointF toScreen(gs2geo::Pt p) const;
	QPointF toScreen(double x, double y) const { return toScreen(gs2geo::Pt{ x, y }); }
	QPolygonF screenPoly(const gs2geo::Pt pts[4]) const;
	void pick(QPointF screen, bool add);   //add = Ctrl: the slot under the cursor in or out of the selection
	QString selectionText() const;
};
