#pragma once

#include <QWidget>
#include "ui_DatasetPage.h"

#include "QMainGraphicsScene.h"
#include <QGraphicsPixmapItem>
#include "QRoundedPixmapItem.h"
#include "DatasetQDragBox.h"
#include "QDragObject.h"
#include "QCommonStruct.h"
#include "UnitConfigTab.h"
#include <atomic>

class DatasetPage : public QWidget
{
	Q_OBJECT

public:
	DatasetPage(QWidget *parent = nullptr);
	~DatasetPage();

	void updateDatasetView(QVector<UnitConfigInfo> unitConfigInfos, UnitConfigTab* unitConfigTab, const QHash <QString, QView> & views, QHash<QString, OpticsInfo> & recipeOptics);
	void addDefectResults(QVector<ct::DefectResult>& defectResults);


private:
	Ui::DatasetPageClass ui;

	QVector<ct::DefectResult> _defectResults;

	QIcon* _passPixmapIcon;
	QIcon* _failPixmapIcon;
	QIcon* _missingPixmapIcon;

	QIcon* _longPassPixmapIcon;
	QIcon* _longFailPixmapIcon;
	QIcon* _longMissingPixmapIcon;

	QIcon* _checkedPixmapIcon;
	QIcon* _uncheckedPixmapIcon;

	QPixmap _passPixmap;
	QPixmap _failPixmap;
	QPixmap _missingPixmap;

	QPixmap _longPassPixmap;
	QPixmap _longFailPixmap;
	QPixmap _longMissingPixmap;

	QPixmap _checkedPixmap;
	QPixmap _uncheckedPixmap;

	QPixmap _noImagePixmap;

	QMainGraphicsScene* _pGraphicsSceneMain;
	QRectF _sceneBound;
	QVector<DatasetQDragBox*> _datasetQDragBoxes;
	DatasetQDragBox* _currentDatasetQDragBox = nullptr;
	bool _dragMode = false;

	int _numOfItemPerPage = 0;

	int _currentPageItem = 1;
	int _currentPageTotalItem = 0;
	int _totalItem = 0;

	bool _renderingDatasetImage = false;

	QStringList _storedIndexIDs;

	std::atomic<bool> _cancelCurrentTask = false;
	

	void initGraphicsView();
	DatasetQDragBox* drawDatasetDragBox(const QRectF& rect, const QString& viewID, const QString& opticID, const QString& indexID, const QColor& color = Qt::magenta, const QString& name = QString("DRAG_BOX"));
	void clearDatasetDragBox();
	void updateDatasetDragBoxStatus();

	int extractInteger(const QString& text);

public slots:
	void updateImagesSelected();
	void selectAllImages();
	void goToPage();
	void showCurrentUnit();
	void filterDatasetView();

	//graphicView
	void setdragMode(bool flag);
	void setRightMousePressed(QPoint point);
	void onAddPixmapToScene(QGraphicsPixmapItem* pixmapItem);

	QStringList getFilteredUnitIndexIDs();

	//stored Units
	void runStoredUnits();
	void storeUnits();
	void removeStoredUnits();

signals:
	void refreshDatasetView();
	void addPixmapToScene(QGraphicsPixmapItem* pixmapItem);
	void displayCurrentView(QString viewID, QString opticID, QString indexID);
	void runStoredUnitsInspection(QStringList storedIndexIDs);
};
