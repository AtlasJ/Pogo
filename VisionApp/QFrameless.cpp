#include "QFrameless.h"
#include <QDebug>

FrameLess::FrameLess(QWidget *target , VisionApp *visionApp) :
	_target(target),
	_visionApp(visionApp),
	_cursorchanged(false),
	_leftButtonPressed(false),
	_borderWidth(5),
	_dragPos(QPoint())
{
	_target->setMouseTracking(true);
	_target->setWindowFlags(Qt::FramelessWindowHint);
	_target->setAttribute(Qt::WA_TranslucentBackground);
	_target->setAttribute(Qt::WA_Hover);
	_target->installEventFilter(this);
	_rubberband = new QRubberBand(QRubberBand::Rectangle);
}

bool FrameLess::eventFilter(QObject *o, QEvent*e) {
	if (((e->type() == QEvent::MouseMove ||
		e->type() == QEvent::HoverMove ||
		e->type() == QEvent::Leave ||
		e->type() == QEvent::MouseButtonPress ||
		e->type() == QEvent::MouseButtonRelease) && _target->windowState() != Qt::WindowFullScreen) || e->type() == QEvent::MouseButtonDblClick){

		switch (e->type()) {
		case QEvent::MouseMove:
			mouseMove(static_cast<QMouseEvent*>(e));
			return true;
			break;
		case QEvent::HoverMove:
			mouseHover(static_cast<QHoverEvent*>(e));
			return true;
			break;
		case QEvent::Leave:
			mouseLeave(e);
			return true;
			break;
		case QEvent::MouseButtonPress:
			mousePress(static_cast<QMouseEvent*>(e));
			return true;
			break;
		case QEvent::MouseButtonRelease:
			mouseRelease(static_cast<QMouseEvent*>(e));
			return true;
			break;
		case QEvent::MouseButtonDblClick:
			mouseDoubleClickEvent(static_cast<QMouseEvent*>(e));
			return true;
			break;
		}
		return _target->eventFilter(o, e);
	}
	else {
		return _target->eventFilter(o, e);
	}
}

void FrameLess::mouseHover(QHoverEvent *e) {
	updateCursorShape(_target->mapToGlobal(e->pos()));
}

void FrameLess::mouseLeave(QEvent *e) {
	if (!_leftButtonPressed) {
		_target->unsetCursor();
	}
}

void FrameLess::mousePress(QMouseEvent *e) {

	QRect titleBarRect = QRect(_target->rect().x(), _target->rect().y(), _target->rect().width(), 30);
	
	if (e->button() == Qt::LeftButton) {
		_leftButtonPressed = true;
		calculateCursorPosition(e->globalPos(), _target->frameGeometry(), _mousePress);
		if (!_mousePress.testFlag(Edge::None)) {
			_rubberband->setGeometry(_target->frameGeometry());
			_visionApp->hideRightMenu(true);
		}
		//else if (_target->rect().marginsRemoved(QMargins(borderWidth(), borderWidth(), borderWidth(), borderWidth())).contains(e->pos())) {
		else if (titleBarRect.marginsRemoved(QMargins(borderWidth(), borderWidth(), borderWidth(), borderWidth())).contains(e->pos())) {
		/*	qDebug() << "ePos:" << e->pos();
			qDebug() << "targetRect:" << titleBarRect;
			qDebug() << "margins:" << QMargins(borderWidth(), borderWidth(), borderWidth(), borderWidth());
			qDebug() << "titleBarRect_MarginsRemoved:" << titleBarRect.marginsRemoved(QMargins(borderWidth(), borderWidth(), borderWidth(), borderWidth()));*/
			qDebug() << "mousePressed";
			_dragStart = true;
			_dragPos = e->pos();
		}
	}

	
}

void FrameLess::mouseRelease(QMouseEvent *e) {
	

	if (e->button() == Qt::LeftButton) {
		qDebug() << "MouseReleased";
		_leftButtonPressed = false;
		_dragStart = false;
		_visionApp->blockRightMenu(false);
	}
	
	
}

void FrameLess::mouseMove(QMouseEvent *e) {
	//qDebug() << "dragstart:" << _dragStart;
	if (_leftButtonPressed) {
		if (_dragStart && _target->windowState() != Qt::WindowFullScreen) {
			_target->move(_target->frameGeometry().topLeft() + (e->pos() - _dragPos));
		}
		else if (!_mousePress.testFlag(Edge::None)) {
			updateWindowMask(14);
			int left = _rubberband->frameGeometry().left();
			int top = _rubberband->frameGeometry().top();
			int right = _rubberband->frameGeometry().right();
			int bottom = _rubberband->frameGeometry().bottom();
			switch (_mousePress) {
			case Edge::Top:
				top = e->globalPos().y();
				break;
			case Edge::Bottom:
				bottom = e->globalPos().y();
				break;
			case Edge::Left:
				left = e->globalPos().x();
				break;
			case Edge::Right:
				right = e->globalPos().x();
				break;
			case Edge::TopLeft:
				top = e->globalPos().y();
				left = e->globalPos().x();
				break;
			case Edge::TopRight:
				right = e->globalPos().x();
				top = e->globalPos().y();
				break;
			case Edge::BottomLeft:
				bottom = e->globalPos().y();
				left = e->globalPos().x();
				break;
			case Edge::BottomRight:
				bottom = e->globalPos().y();
				right = e->globalPos().x();
				break;
			}

			QRect newRect(QPoint(left, top), QPoint(right, bottom));
			bool resize = true;
			if (newRect.width() <= _target->minimumWidth() + 150) {
				left = _target->frameGeometry().x();
				resize = false;
			}
			else if (newRect.height() <= _target->minimumHeight()) {
				top = _target->frameGeometry().y();
				resize = false;
			}

			if (resize)
			{
				_target->setGeometry(QRect(QPoint(left, top), QPoint(right, bottom)));
				_rubberband->setGeometry(QRect(QPoint(left, top), QPoint(right, bottom)));

				_visionApp->resizeLogTab();
			}	
		}
	}
	else {
		updateCursorShape(e->globalPos());
	}

	
}

void FrameLess::mouseDoubleClickEvent(QMouseEvent * e)
{
	if (e->button() == Qt::LeftButton) {
		calculateCursorPosition(e->globalPos(), _target->frameGeometry(), _mousePress);
		if(_topMenuBarClicked) _visionApp->maximize_restoreWindow();
	}
}

void FrameLess::updateCursorShape(const QPoint &pos) {

	if (_target->isFullScreen() || _target->isMaximized()) {
		if (_cursorchanged) {
			_target->unsetCursor();
		}
		return;
	}
	if (!_leftButtonPressed) {
		calculateCursorPosition(pos, _target->frameGeometry(), _mouseMove);
		_cursorchanged = true;
		if (_mouseMove.testFlag(Edge::Top) || _mouseMove.testFlag(Edge::Bottom)) {
			_target->setCursor(Qt::SizeVerCursor);
		}
		else if (_mouseMove.testFlag(Edge::Left) || _mouseMove.testFlag(Edge::Right)) {
			_target->setCursor(Qt::SizeHorCursor);
		}
		else if (_mouseMove.testFlag(Edge::TopLeft) || _mouseMove.testFlag(Edge::BottomRight)) {
			_target->setCursor(Qt::SizeFDiagCursor);
		}
		else if (_mouseMove.testFlag(Edge::TopRight) || _mouseMove.testFlag(Edge::BottomLeft)) {
			_target->setCursor(Qt::SizeBDiagCursor);
		}
		else if (_cursorchanged) {
			_target->unsetCursor();
			_cursorchanged = false;
		}
	}
}

void FrameLess::calculateCursorPosition(const QPoint &pos, const QRect &framerect, Edges &_edge) {
	bool onLeft = pos.x() >= framerect.x() - _borderWidth && pos.x() <= framerect.x() + _borderWidth &&
		pos.y() <= framerect.y() + framerect.height() - _borderWidth && pos.y() >= framerect.y() + _borderWidth;

	bool onRight = pos.x() >= framerect.x() + framerect.width() - _borderWidth && pos.x() <= framerect.x() + framerect.width() &&
		pos.y() >= framerect.y() + _borderWidth && pos.y() <= framerect.y() + framerect.height() - _borderWidth;

	bool onBottom = pos.x() >= framerect.x() + _borderWidth && pos.x() <= framerect.x() + framerect.width() - _borderWidth  &&
		pos.y() >= framerect.y() + framerect.height() - _borderWidth && pos.y() <= framerect.y() + framerect.height();

	bool onTop = pos.x() >= framerect.x() + _borderWidth && pos.x() <= framerect.x() + framerect.width() - _borderWidth &&
		pos.y() >= framerect.y() && pos.y() <= framerect.y() + _borderWidth;

	bool  onBottomLeft = pos.x() <= framerect.x() + _borderWidth && pos.x() >= framerect.x() &&
		pos.y() <= framerect.y() + framerect.height() && pos.y() >= framerect.y() + framerect.height() - _borderWidth;

	bool onBottomRight = pos.x() >= framerect.x() + framerect.width() - _borderWidth && pos.x() <= framerect.x() + framerect.width() &&
		pos.y() >= framerect.y() + framerect.height() - _borderWidth && pos.y() <= framerect.y() + framerect.height();

	bool onTopRight = pos.x() >= framerect.x() + framerect.width() - _borderWidth && pos.x() <= framerect.x() + framerect.width() &&
		pos.y() >= framerect.y() && pos.y() <= framerect.y() + _borderWidth;

	bool onTopLeft = pos.x() >= framerect.x() && pos.x() <= framerect.x() + _borderWidth &&
		pos.y() >= framerect.y() && pos.y() <= framerect.y() + _borderWidth;

	_topMenuBarClicked = pos.y() <= framerect.y() + 35; //35 is the height of the top menu bar

	if (onLeft) {
		_edge = Left;
	}
	else if (onRight) {
		_edge = Right;
	}
	else if (onBottom) {
		_edge = Bottom;
	}
	else if (onTop) {
		_edge = Top;
	}
	else if (onBottomLeft) {
		_edge = BottomLeft;
	}
	else if (onBottomRight) {
		_edge = BottomRight;
	}
	else if (onTopRight) {
		_edge = TopRight;
	}
	else if (onTopLeft) {
		_edge = TopLeft;
	}
	else {
		_edge = None;
	}
}

void FrameLess::updateWindowMask(int borderRadius)
{
	// Create a painter path to draw the rounded rectangle
	QPainterPath roundedPath;
	roundedPath.addRoundedRect(QRectF(0, 0, _target->width(), _target->height()), borderRadius, borderRadius);

	// Set the painter path as the shape of the window
	_target->setMask(roundedPath.toFillPolygon().toPolygon());
	_target->update();
}
