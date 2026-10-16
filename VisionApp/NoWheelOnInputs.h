#pragma once
#include <QObject>
#include <QEvent>
#include <QWheelEvent>
#include <QComboBox>
#include <QAbstractSpinBox>

/*
* The mouse wheel does not edit settings.
*
* A combo box or a spin box under the pointer swallows the wheel and steps its value, so an
* operator scrolling a settings page could change a recipe setting without noticing - and the
* right tab is a long scroll area full of exactly these widgets. Installed once on the
* application, this drops the wheel for every QComboBox and every QAbstractSpinBox (spin boxes,
* double spin boxes, date edits) in the program, present and future.
*
* The event is marked IGNORED before it is filtered out: QApplication::notify propagates an
* unaccepted wheel event to the parent widget, so the scroll area behind the control still
* scrolls. Consuming it accepted would freeze the page under the pointer instead.
*/
class NoWheelOnInputs : public QObject {
public:
	explicit NoWheelOnInputs(QObject* parent = nullptr) : QObject(parent) {}

protected:
	bool eventFilter(QObject* watched, QEvent* event) override
	{
		if (event->type() == QEvent::Wheel
			&& (qobject_cast<QComboBox*>(watched) || qobject_cast<QAbstractSpinBox*>(watched))) {
			event->ignore();
			return true;
		}
		return QObject::eventFilter(watched, event);
	}
};
