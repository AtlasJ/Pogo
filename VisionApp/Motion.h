#pragma once

#include <QWidget>
#include "ui_Motion.h"

class Motion : public QWidget
{
	Q_OBJECT

public:
	Motion(QWidget *parent = Q_NULLPTR);
	~Motion();
	double getJogToX();
	double getJogToY();
	double getJogToZ();

private:
	Ui::Motion ui;

	public Q_SLOTS:
	void setCurrentX(double);
	void setCurrentY(double);
	void setCurrentZ(double);
	void setCurrentPoint(double, double, double);

signals:
	void stepChanged(double);
	void jogTo(double, double, double);
	void jogLeft();
	void jogRight();
	void jogFront();
	void jogBack();
	void jogTop();
	void jogBottom();
};
