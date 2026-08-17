#include "DatasetQDragBox.h"
#include "QGrabber.h"

DatasetQDragBox::DatasetQDragBox()
{
	// Set up the animation
	_animation = new QPropertyAnimation(this, "pixmapOffset");
	_animation->setDuration(300); // 300ms animation

	// Create and set up the opacity effect for fade animation
	_opacityEffect = new QGraphicsOpacityEffect(this);
	_opacityEffect->setOpacity(1.0);
	//setGraphicsEffect(_opacityEffect); // Apply effect to the entire item

	// Create fade animation
	_fadeAnimation = new QPropertyAnimation(_opacityEffect, "opacity");
	_fadeAnimation->setDuration(300);  // Fade duration (in milliseconds)
	_fadeAnimation->setStartValue(1);  // Start from fully transparent
	_fadeAnimation->setEndValue(0);    // End fully opaque
}


void DatasetQDragBox::addPassPixmap(QPixmap* pixmap, QPixmap* longPixmap)
{
	_passPixmap = pixmap;
	_longPassPixmap = longPixmap;
}

void DatasetQDragBox::addFailPixmap(QPixmap* pixmap, QPixmap* longPixmap)
{
	_failPixmap = pixmap;
	_longFailPixmap = longPixmap;
}

void DatasetQDragBox::addMissingPixmap(QPixmap* pixmap, QPixmap* longPixmap)
{
	_missingPixmap = pixmap;
	_longMissingPixmap = longPixmap;
}

void DatasetQDragBox::customHoverEnterEvent(QGraphicsSceneHoverEvent * event)
{
	// Start the animation to slide out the pixmap
	_animation->stop();
	_animation->setStartValue(-100);
	_animation->setEndValue(0); // Slide out to offset of 10px
	_animation->start();

	_isHighlighted = true;
	update();
	emit dragBoxMouseHoverEntered(this);
}

void DatasetQDragBox::customHoverLeaveEvent(QGraphicsSceneHoverEvent* event)
{
	if (!_isSelected)
	{
		// Start the animation to slide back the pixmap
		_animation->stop();
		_animation->setStartValue(_pixmapOffset);
		_animation->setEndValue(-100); // Slide back to original position
		_animation->start();
	}
	

	_isHighlighted = false;
	update();
	emit dragBoxMouseHoverLeaved(this, QString());
}

QVariant DatasetQDragBox::customItemChange(GraphicsItemChange change, const QVariant& value)
{
	if (change == QGraphicsItem::ItemSelectedChange) {
		if (value.toBool() && !_isHighlighted) {
			// Item is being selected
			// Start the animation to slide out the pixmap
			_isSelected = true;
			_animation->stop();
			_animation->setStartValue(-100);
			_animation->setEndValue(0); // Slide out to offset of 10px
			_animation->start();

			_checkboxChecked = true;
			update();  // Trigger a repaint
		}
		else {
			// Item is being deselected
			// Start the animation to slide back the pixmap
			_isSelected = false;
			_animation->stop();
			_animation->setStartValue(_pixmapOffset);
			_animation->setEndValue(-100); // Slide back to original position
			_animation->start();

			_checkboxChecked = false;
			update();
		}
	}

	return QGraphicsItem::itemChange(change, value);
}

//void DatasetQDragBox::customMousePressEvent(QGraphicsSceneMouseEvent* event)
//{
//	QRect checkboxRect(80, 10, 32, 32); // Same as the pixmap rect
//	if (checkboxRect.contains(event->pos().toPoint())) {
//	
//		qDebug() << "initialfadingValue:" << _fadeAnimation->currentValue();
//		// Fade out the current checkbox image first
//		update();  // Trigger a repaint
//		_fadeAnimation->setDirection(QAbstractAnimation::Forward); // Fade out
//		_fadeAnimation->start();
//		
//		// After the fade out, switch the pixmap
//		QTimer::singleShot(300, this, [this]() {  // Wait for fade out to complete
//			qDebug() << "backwardfadingValue:" << _fadeAnimation->currentValue();
//			// Toggle the checkbox checked state
//			_checkboxChecked = !_checkboxChecked;
//
//			// Change the pixmap based on the state
//			_fadeAnimation->setDirection(QAbstractAnimation::Backward); // Fade in
//			update();  // Trigger a repaint
//			_fadeAnimation->start();
//
//			QTimer::singleShot(300, this, [this]() {  // Wait for fade out to complete
//				update();
//				qDebug() << "forwardfadingValue:" << _fadeAnimation->currentValue();
//				});
//			});
//
//		
//	}
//	QGraphicsObject::mousePressEvent(event);
//}

void DatasetQDragBox::customMousePressEvent(QGraphicsSceneMouseEvent* event)
{
	QRect checkboxRect(80, 10, 32, 32); // Same as the pixmap rect
	if (checkboxRect.contains(event->pos().toPoint())) {
		_checkboxChecked = !_checkboxChecked;
		update();  // Trigger a repaint
	}
	QGraphicsObject::mousePressEvent(event);
}

void DatasetQDragBox::paintFunction(QPainter * painter, const QStyleOptionGraphicsItem * option, QWidget * widget)
{
	painter->setRenderHints(QPainter::Qt4CompatiblePainting);

	// Save the current painter state
	painter->save();

	QPen pen;
	pen.setWidth(2);
	pen.setColor(_borderColor);

	painter->setBrush(QColor(_borderColor.red(), _borderColor.green(), _borderColor.blue(), 50));
	painter->setPen(pen);

	QPainterPath path;
	path.addRoundedRect(QRectF(_left, _top, _width, _height), 10, 10);
	painter->setClipPath(path);

	QPixmap* pixmap = nullptr;
	QPixmap* longPixmap = nullptr;

	if (_status == PASS)
	{
		pixmap = _passPixmap;
		longPixmap = _longPassPixmap;
	}
	else if (_status == FAIL)
	{
		pixmap = _failPixmap;
		longPixmap = _longFailPixmap;
	}
	else if (_status == MISSING)
	{
		pixmap = _missingPixmap;
		longPixmap = _longMissingPixmap;
	}
	
	if (_isHighlighted || (option->state & QStyle::State_Selected))
	{
		if (option->state & QStyle::State_Selected)
		{
			pen.setWidth(4);
			painter->setPen(pen);
		}
		painter->drawRoundedRect(QRectF(_left, _top, _width, _height), 10, 10);

		if (longPixmap != nullptr)
		{
			QRectF pixmapRect(_pixmapOffset, _height - 32, 64, 32);
			painter->drawPixmap(pixmapRect, *longPixmap, QRectF(0, 0, 64, 32));
		}
	}
	else
	{
		if (pixmap != nullptr)
		{
			painter->drawPixmap(QRectF(0, _height - 32, 32, 32), *pixmap, QRectF(0, 0, 32, 32));
		}
	}

	//// Restore the painter state (important to avoid affecting other items)
	//painter->restore();

	
	// Draw the checkbox pixmap with fade effect
	QPixmap* checkboxPixmap = _checkboxChecked ? _checkedPixmap : _uncheckedPixmap;
	QRect pixmapRect(80, 10, 32, 32); // Position and size of the checkbox
	// Apply the opacity effect manually during the painting phase

	if (checkboxPixmap)
	{
		painter->drawPixmap(pixmapRect, *_uncheckedPixmap);
		//painter->setOpacity(_fadeAnimation ? _fadeAnimation->currentValue().toReal() : 1.0); // Use the fade animation value
		painter->drawPixmap(pixmapRect, *checkboxPixmap);
	}
	painter->restore();


	pen.setWidth(5);
	pen.setColor(_borderColor);
	painter->setPen(pen);
	QBrush brush;
	brush.setColor(_borderColor);
	painter->setBrush(brush);
	QFont font = painter->font();
	font.setPointSize(16);
	font.setBold(true);
	painter->setFont(font);
	painter->drawText(QPoint(15, 30), _indexID);


	_scale = scale() / painter->transform().m11();

	//if (option->state & QStyle::State_Selected)
	//{
	//	_isSelected = true;
	//	pen.setWidth(4);
	//	pen.setColor(QColor(55, 198, 255));
	//	painter->setPen(pen);
	//	if (_type == (int)DragBoxType::VISIONOBJECT) painter->setBrush(QColor(_borderColor.red(), _borderColor.green(), _borderColor.blue(), 50));
	//	//painter->setPen(QPen(QColor(255, 165, 0), 3));
	//}
	//else
	//{
	//	_isSelected = false;
	//	if (_type == (int)DragBoxType::VISIONOBJECT) painter->setBrush(QColor(_borderColor.red(), _borderColor.green(), _borderColor.blue(), 50));
	//	//painter->setPen(QPen(_borderColor, 3));
	//}

	//painter->drawRect(QRectF(_left, _top, _width, _height));

	

	//painter->drawText(QRectF(0, 0, _width, _height), QString::number(_index));
	//_scale = scale() / painter->transform().m11();
	
}

void DatasetQDragBox::setPassIcons(QPixmap* passPixmap, QPixmap* longPassPixmap)
{
	_passPixmap = passPixmap;
	_longPassPixmap = longPassPixmap;
}

void DatasetQDragBox::setFailIcons(QPixmap* failPixmap, QPixmap* longFailPixmap)
{
	_failPixmap = failPixmap;
	_longFailPixmap = longFailPixmap;
}

void DatasetQDragBox::setMissingIcons(QPixmap* missingPixmap, QPixmap* longMissingPixmap)
{
	_missingPixmap = missingPixmap;
	_longMissingPixmap = longMissingPixmap;
}

void DatasetQDragBox::setCheckBoxIcons(QPixmap* checkedPixmap, QPixmap* uncheckPixmap)
{
	_checkedPixmap = checkedPixmap;
	_uncheckedPixmap = uncheckPixmap;
}

qreal DatasetQDragBox::pixmapOffset() const
{
	return _pixmapOffset;
}

void DatasetQDragBox::setPixmapOffset(qreal offset)
{
	_pixmapOffset = offset;
	update(); // Repaint the item
}

void DatasetQDragBox::viewID(QString viewID)
{
	_viewID = viewID;
}

QString DatasetQDragBox::viewID()
{
	return _viewID;
}

void DatasetQDragBox::opticID(QString opticID)
{
	_opticID = opticID;
}

QString DatasetQDragBox::opticID()
{
	return _opticID;
}

void DatasetQDragBox::indexID(QString indexID)
{
	_indexID = indexID;
}

QString DatasetQDragBox::indexID()
{
	return _indexID;
}

void DatasetQDragBox::setDatasetStatus(DatasetStatus status)
{
	_status = status;
}

DatasetQDragBox::DatasetStatus DatasetQDragBox::getDatasetStatus()
{
	return _status;
}

bool DatasetQDragBox::checkBoxChecked()
{
	return _checkboxChecked;
}

