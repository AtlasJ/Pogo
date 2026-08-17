#pragma once

#include <QWidget>
#include "ui_ProductionPage.h"
#include "QMainGraphicsView.h"
#include "QMainGraphicsScene.h"

#include "QView.h"
#include "OpticsInfo.h"
#include "AlgoDefectResult.h"
#include <QImage>

class ProductionPage : public QWidget
{
	Q_OBJECT

public:
	ProductionPage(QWidget *parent = nullptr);
	~ProductionPage();

	void setCamInfo(QHash <QString, QView>& views, QHash < QString, OpticsInfo> & recipeOptics);
	void displayCamGraphicViews();
	void updateCamGraphicViews(QString viewID, QString opticID, QString indexID, QImage* img, QHash<QString, ct::UnitResultInfo> unitResultInfos);

	void inspectionDone();

	void startElapseTime();
	void stopElapseTime();

	
	LotInfo getLotInfos();

private:
	Ui::ProductionPageClass ui;

	QVector<QMainGraphicsView*> _camViews;
	QVector<QLabel*> _camNames;
	QVector<QTextEdit*> _camResults;
	QVector<QCheckBox*> _viewOpticList;

	QIcon* _passPixmapIcon;
	QIcon* _failPixmapIcon;

	QPixmap _passPixmap;
	QPixmap _failPixmap;

	int _totalInspectedUnit = 0;
	int _totalFailUnit = 0;
	int _totalPassUnit = 0;
	QHash<QString, bool> _unitPassHash;
	double _totalInspectedTimeMs = 0;
	double _uphTime = 0;

	QElapsedTimer elapsedTimer;

	void clearCamGraphicViews();
	void updateUptime();
};
