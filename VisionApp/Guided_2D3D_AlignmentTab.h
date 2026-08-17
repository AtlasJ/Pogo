#pragma once

#include <QWidget>
#include <QFileDialog>
#include <QDateTime>
#include "ui_Guided_2D3D_AlignmentTab.h"

#include "WorldCoordinate.h"
#include "CommonDir.h"
#include "QMainGraphicsView.h"
#include "QMainGraphicsScene.h"
#include "QGraphicsItem.h"
#include "QCrossItem.h"

class Guided_2D3D_AlignmentTab : public QWidget
{
	Q_OBJECT

public:
	Guided_2D3D_AlignmentTab(QWidget *parent = nullptr);
	~Guided_2D3D_AlignmentTab();

	enum class AlignmentMode {
		OFFSET_2D3D,
		OFFSET_POSITION_PORTABILITY
	};

	void reloadAlignmentImages();
	void setCurrentLaserOffset(dat::WorldCoordinate offset);
	void setCurrentLaserOffsetUI(dat::WorldCoordinate offset);
	void setCurrentPositionPortabilityOffset(dat::WorldCoordinate offset);
	void setCurrentPositionPortabilityOffsetUI(dat::WorldCoordinate offset);
	void setMode(AlignmentMode mode);

	void loadPositionPortabilityImages(QImage & plane, QImage & Vo);

private:
	Ui::Guided_2D3D_AlignmentTabClass ui;

	AlignmentMode _mode = AlignmentMode::OFFSET_2D3D;

	QMainGraphicsScene* _pGraphicsSceneMain = nullptr;
	QGraphicsPixmapItem* _pPixmapItem2D = nullptr;
	QGraphicsPixmapItem* _pPixmapItem3D = nullptr;

	QRectF _sceneBound;

	dat::WorldCoordinate _laserOffset;
	QPointF _offset;
	double _zOffset;

	dat::WorldCoordinate _positionPortabilityOffset;

	QCrossItem* _point1Cross = nullptr;
	QCrossItem* _point2Cross = nullptr;


public Q_SLOTS:
	void toggle2D3D_Opacity(int value);
	void jogFront();
	void jogBack();
	void jogRight();
	void jogLeft();
	void jogUp();
	void jogDown();
	void confirmLaserOffset();
	void confirmPortabilityOffset();

	void showPoint1();
	void showPoint2();
	void load2dImage();
	void load3dImage();
	void rotateImages();
	void offsetImages();

	QCrossItem* drawCross(const QRectF& rect, const  QColor& color);
	void mouseMove(QPoint point);


signals:
	void updateLaserOffset(dat::WorldCoordinate offset);
	void updatePositionPortabilityOffset(dat::WorldCoordinate offset);

	void motionJogUp(double jogStep);
	void motionJogDown(double jogStep);

};
