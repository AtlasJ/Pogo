#ifndef DATASETQDragBox_H
#define DATASETQDragBox_H

#include <QtWidgets>
#include "QCommonStruct.h"
#include "QDragObject.h"
#include <QIcon>
#include <QHash>
#include <QPixmap>
#include <QString>
#include <QPropertyAnimation>
#include <QGraphicsObject>

class QGrabber;

class DatasetQDragBox : public QDragObject
{
	Q_OBJECT
		Q_PROPERTY(qreal pixmapOffset READ pixmapOffset WRITE setPixmapOffset)
		//Q_PROPERTY(qreal checkBoxOpacity READ checkBoxOpacity WRITE setCheckBoxOpacity)



public:
	DatasetQDragBox();

	enum DatasetStatus
	{
		PASS,
		FAIL,
		MISSING
	};

	void addPassPixmap(QPixmap* pixmap,QPixmap* longPixmap);
	void addFailPixmap(QPixmap* pixmap, QPixmap* longPixmap);
	void addMissingPixmap(QPixmap* pixmap, QPixmap* longPixmap);

	virtual void customHoverEnterEvent(QGraphicsSceneHoverEvent * event) override;
	virtual void customHoverLeaveEvent(QGraphicsSceneHoverEvent* event) override;
	virtual QVariant customItemChange(GraphicsItemChange change, const QVariant& value) override;
	virtual void  customMousePressEvent(QGraphicsSceneMouseEvent* event) override;
	virtual void paintFunction(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);


	void setPassIcons(QPixmap* passPixmap, QPixmap* longPassPixmap);
	void setFailIcons(QPixmap* failPixmap, QPixmap* longFailPixmap);
	void setMissingIcons(QPixmap* missingPixmap, QPixmap* longMissingPixmap);
	void setCheckBoxIcons(QPixmap* checkedPixmap, QPixmap* uncheckPixmap);

	qreal pixmapOffset() const;
	void setPixmapOffset(qreal offset);

	void viewID(QString viewID);
	QString viewID();

	void opticID(QString opticID);
	QString opticID();

	void indexID(QString indexID);
	QString indexID();

	void setDatasetStatus(DatasetStatus status);
	DatasetStatus getDatasetStatus();

	bool checkBoxChecked();


private:

	QPixmap* _passPixmap = nullptr;
	QPixmap* _failPixmap = nullptr;
	QPixmap* _missingPixmap = nullptr;

	QPixmap* _longPassPixmap = nullptr;
	QPixmap* _longFailPixmap = nullptr;
	QPixmap* _longMissingPixmap = nullptr;

	QPixmap* _checkedPixmap = nullptr;
	QPixmap* _uncheckedPixmap = nullptr;

	bool _isHighlighted = false;
	bool _isSelected = false;

	qreal _pixmapOffset;             // Offset for the pixmap's position
	QPropertyAnimation* _animation;  // Animation for sliding effect

	QGraphicsOpacityEffect* _opacityEffect;
	QPropertyAnimation* _fadeAnimation;
	bool _checkboxChecked = false;

	QString _viewID;
	QString _opticID;
	QString _indexID;

	DatasetStatus _status = PASS;

};

#endif // VisionAppQDragBox_H

//class QGrabber;
//
//class DatasetQDragBox : public QDragBox
//{
//	Q_PROPERTY(qreal pixmapOffset READ pixmapOffset WRITE setPixmapOffset)
//public:
//	DatasetQDragBox();
//
//	void addPassPixmap(QPixmap* pixmap, QPixmap* longPixmap);
//	void addFailPixmap(QPixmap* pixmap, QPixmap* longPixmap);
//	void addMissingPixmap(QPixmap* pixmap, QPixmap* longPixmap);
//
//	virtual void customHoverEnterEvent(QGraphicsSceneHoverEvent* event) override;
//	virtual void customHoverLeaveEvent(QGraphicsSceneHoverEvent* event) override;
//	virtual void paintFunction(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget);
//
//	int _index = 0;
//	QString _comparisonStatus;
//
//	void setPassIcons(QPixmap* passPixmap, QPixmap* longPassPixmap);
//
//	qreal pixmapOffset() const;
//	void setPixmapOffset(qreal offset);
//
//
//private:
//
//	QPixmap* _passPixmap = nullptr;
//	QPixmap* _failPixmap = nullptr;
//	QPixmap* _missingPixmap = nullptr;
//
//	QPixmap* _longPassPixmap = nullptr;
//	QPixmap* _longFailPixmap = nullptr;
//	QPixmap* _longMissingPixmap = nullptr;
//
//	QPropertyAnimation* _animation;
//	qreal _pixmapOffset = 0;
//
//	bool _isHighlighted = false;
//};
//
//#endif // VisionAppQDragBox_H