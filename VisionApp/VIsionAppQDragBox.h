#ifndef VISIONAPPVisionAppQDragBox_H
#define VISIONAPPVisionAppQDragBox_H

#include <QtWidgets>
#include "QCommonStruct.h"
#include "QDragBox.h"
#include "AlgoGraphList.h"
#include <QIcon>
#include <QHash>
#include <QPixmap>

class QGrabber;

class VisionAppQDragBox : public QDragBox
{
public:
	VisionAppQDragBox();

	//VisionObject();
	void algoGraph(AlgoGraph* algoGraph);
	AlgoGraph* algoGraph();

	void viewID(const QString & viewID);
	QString viewID();

	void lineScanID(const QString & lineScanID);
	QString lineScanID();

	void addNoViewPixmap(QPixmap* pixmap);

	void setFrozen(bool flag);

	void setLocked(bool flag);
	bool isLocked() const;

	virtual void customHoverEnterEvent(QGraphicsSceneHoverEvent * event) override;
	virtual void paintFunction(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

	int _index = 0;
	QString _comparisonStatus;


private:

	//visionObject
	AlgoGraph* _algoGraph = nullptr;
	QString _viewID = "";
	QString _lineScanID = "";

	QPixmap* _noViewPixmap = nullptr;
	bool _frozen = false;
	bool _locked = false;

};

#endif // VisionAppQDragBox_H