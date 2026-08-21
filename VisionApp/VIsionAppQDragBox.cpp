#include "VisionAppQDragBox.h"
#include "QGrabber.h"

VisionAppQDragBox::VisionAppQDragBox()
{
}

void VisionAppQDragBox::algoTemplate(AlgoTemplate * algoTemplate)
{
	_algoTemplate = algoTemplate;
}

AlgoTemplate * VisionAppQDragBox::algoTemplate()
{
	return _algoTemplate;
}

void VisionAppQDragBox::viewID(const QString & viewID)
{
	_viewID = viewID;
}

QString VisionAppQDragBox::viewID()
{
	return _viewID;
}

void VisionAppQDragBox::lineScanID(const QString & lineScanID)
{
	_lineScanID = lineScanID;
}

QString VisionAppQDragBox::lineScanID()
{
	return _lineScanID;
}

void VisionAppQDragBox::addNoViewPixmap(QPixmap* pixmap)
{
	_noViewPixmap = pixmap;
}

void VisionAppQDragBox::setFrozen(bool flag)
{
	_frozen = flag;
}

void VisionAppQDragBox::customHoverEnterEvent(QGraphicsSceneHoverEvent * event)
{
	bool algoTemplateExist = false;
	if (_algoTemplate) algoTemplateExist = true;

	if (getDragable())
	{
		if ((algoTemplateExist && !_algoTemplate->uniformBox()) || (!algoTemplateExist))
		{
			corners()[0] = new QGrabber(this, 0, grabSize());
			corners()[1] = new QGrabber(this, 1, grabSize());
			corners()[2] = new QGrabber(this, 2, grabSize());
			corners()[3] = new QGrabber(this, 3, grabSize());
			corners()[4] = new QGrabber(this, 4, grabSize());
			corners()[5] = new QGrabber(this, 5, grabSize());
			corners()[6] = new QGrabber(this, 6, grabSize());
			corners()[7] = new QGrabber(this, 7, grabSize());

			corners()[0]->installSceneEventFilter(this);
			corners()[1]->installSceneEventFilter(this);
			corners()[2]->installSceneEventFilter(this);
			corners()[3]->installSceneEventFilter(this);
			corners()[4]->installSceneEventFilter(this);
			corners()[5]->installSceneEventFilter(this);
			corners()[6]->installSceneEventFilter(this);
			corners()[7]->installSceneEventFilter(this);

			setCornerPositions();
		}
	}

	emit dragBoxMouseHoverEntered(this);
}

void VisionAppQDragBox::paintFunction(QPainter * painter, const QStyleOptionGraphicsItem * option, QWidget * widget)
{
	if (_frozen) return;

	QPen pen;
	pen.setWidth(2);
	pen.setColor(_borderColor);

	painter->setRenderHints(QPainter::Qt4CompatiblePainting);
	painter->setPen(pen);

	if (option->state & QStyle::State_Selected)
	{
		_isSelected = true;
		pen.setWidth(4);
		pen.setColor(QColor(55, 198, 255));
		painter->setPen(pen);
		if (_type == (int)DragBoxType::VISIONOBJECT) painter->setBrush(QColor(_borderColor.red(), _borderColor.green(), _borderColor.blue(), 50));
		//painter->setPen(QPen(QColor(255, 165, 0), 3));
	}
	else
	{
		_isSelected = false;
		if (_type == (int)DragBoxType::VISIONOBJECT) painter->setBrush(QColor(_borderColor.red(), _borderColor.green(), _borderColor.blue(), 50));
		//painter->setPen(QPen(_borderColor, 3));
	}

	painter->drawRect(QRectF(_left, _top, _width, _height));

	int offset = 0;
	if (_viewID.isEmpty() && _noViewPixmap!= nullptr)
	{
		painter->drawPixmap(QRectF(0 + offset, 0, 64, 64), *_noViewPixmap, QRectF(0, 0, 64, 64));
	}

	painter->drawText(QRectF(0, 0, _width, _height), QString::number(_index));
	_scale = scale() / painter->transform().m11();
	
}

void VisionAppQDragBox::setLocked(bool locked)
{
	_locked = locked;

	// disable the built-in QGraphicsItem movable/selectable flags
	setFlag(QGraphicsItem::ItemIsMovable, !locked);

}

bool VisionAppQDragBox::isLocked() const
{
	return _locked;
}


