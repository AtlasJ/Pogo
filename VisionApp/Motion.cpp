#include "Motion.h"

Motion::Motion(QWidget *parent)
	: QWidget(parent)
{
	ui.setupUi(this);

	connect(ui.toolButton_jogBack, &QToolButton::pressed, this, [=]() { emit jogBack(); });
	connect(ui.toolButton_jogBottom, &QToolButton::pressed, this, [=]() { emit jogBottom(); });
	connect(ui.toolButton_jogFront, &QToolButton::pressed, this, [=]() { emit jogFront(); });
	connect(ui.toolButton_jogLeft, &QToolButton::pressed, this, [=]() { emit jogLeft(); });
	connect(ui.toolButton_jogRight, &QToolButton::pressed, this, [=]() { emit jogRight(); });
	connect(ui.toolButton_jogTop, &QToolButton::pressed, this, [=]() { emit jogTop(); });
	connect(ui.toolButton_jogTo, &QToolButton::pressed, this, [=]() {
		emit jogTo(
			ui.lineEdit_jogToX->text().toDouble(),
			ui.lineEdit_jogToY->text().toDouble(),
			ui.lineEdit_jogToZ->text().toDouble());
	});

	connect(ui.lineEdit_stepMM, &QLineEdit::textEdited, this, [=](QString text) {
		auto step = ui.lineEdit_stepMM->text().toDouble();
		if (step < 0) {
			step = 0.0;
			ui.lineEdit_stepMM->setText(QString::number(step));
		}
		if (step > 100) {
			step = 100;
			ui.lineEdit_stepMM->setText(QString::number(step));
		}

		stepChanged(step);
	});
}

Motion::~Motion()
{
}

double Motion::getJogToX()
{
	return ui.lineEdit_x->text().toDouble();
}

double Motion::getJogToY()
{
	return ui.lineEdit_y->text().toDouble();
}

double Motion::getJogToZ()
{
	return ui.lineEdit_z->text().toDouble();
}

void Motion::setCurrentX(double value)
{
	ui.lineEdit_x->setText(QString::number(value));
}

void Motion::setCurrentY(double value)
{
	ui.lineEdit_y->setText(QString::number(value));
}

void Motion::setCurrentZ(double value)
{
	ui.lineEdit_z->setText(QString::number(value));
}

void Motion::setCurrentPoint(double x, double y, double z)
{
	ui.lineEdit_x->setText(QString::number(x));
	ui.lineEdit_y->setText(QString::number(y));
	ui.lineEdit_z->setText(QString::number(z));
}