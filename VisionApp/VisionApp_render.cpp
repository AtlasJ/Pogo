#include "VisionApp.h"
#include "QRectItem.h"
#include "QLineItem.h"
#include "QEllipseItem.h"
#include "QCrossItem.h"
#include <QLineF>

QRectItem* VisionApp::drawRect(const QRectF& rect, const QColor& borderColor, const QColor& innerColor)
{
	QRectItem *pShape = new QRectItem();

	_pGraphicsSceneMain->addItem(pShape);
	pShape->setup(rect, borderColor, innerColor);

	_renderedShape.append(pShape);

	return pShape;
}

QLineItem* VisionApp::drawLine(const QLineF& line, const QColor& color, int width)
{
	QLineItem *pShape = new QLineItem();

	_pGraphicsSceneMain->addItem(pShape);
	pShape->setup(QRectF(line.p1(), line.p2()), color, width);

	_renderedShape.append(pShape);

	return pShape;
}

QEllipseItem* VisionApp::drawEllipse(const qreal& x, const qreal& y, const qreal& radiusX, const qreal& radiusY, const QColor& borderColor, const QColor& innerColor)
{
	QEllipseItem *pShape = new QEllipseItem();

	_pGraphicsSceneMain->addItem(pShape);

	pShape->setup(QRectF(x, y, radiusX, radiusY), borderColor, innerColor);

	_renderedShape.append(pShape);

	return pShape;
}

QGraphicsTextItem* VisionApp::drawText(const QString& text, const QPointF& pos, const QColor& color, const int& pointSize)
{
	QFont font;
	font.setPointSize(pointSize);

	QGraphicsTextItem *pShape = _pGraphicsSceneMain->addText(text, font);
	pShape->setDefaultTextColor(color);
	pShape->setPos(pos);

	_renderedShape.append(pShape);

	return pShape;
}

QCrossItem* VisionApp::drawCross(const QRectF& rect, const  QColor& color)
{
	QCrossItem *pShape = new QCrossItem();

	_pGraphicsSceneMain->addItem(pShape);

	pShape->setup(rect, color);

	_renderedShape.append(pShape);

	return pShape;
}

QRectItem* VisionApp::drawRect(QMainGraphicsScene* scene, QString mapKey, const QRectF& rect, const QColor& borderColor, const QColor& innerColor)
{
	QRectItem *pShape = new QRectItem();

	scene->addItem(pShape);
	pShape->setup(rect, borderColor, innerColor);

	RenderInfo info;
	info.scene = scene;
	info.item = pShape;

	if (_renderedMaps.contains(mapKey)) {
		_renderedMaps[mapKey].append(info);
	}
	else {
		QVector<RenderInfo> renders;
		_renderedMaps.insert(mapKey, renders);
		_renderedMaps[mapKey].append(info);
	}

	return pShape;
}

QLineItem* VisionApp::drawLine(QMainGraphicsScene* scene, QString mapKey, const QLineF& line, const QColor& color, int width)
{
	QLineItem *pShape = new QLineItem();

	scene->addItem(pShape);
	pShape->setup(QRectF(line.p1(), line.p2()), color, width);

	RenderInfo info;
	info.scene = scene;
	info.item = pShape;

	if (_renderedMaps.contains(mapKey)) {
		_renderedMaps[mapKey].append(info);
	}
	else {
		QVector<RenderInfo> renders;
		_renderedMaps.insert(mapKey, renders);
		_renderedMaps[mapKey].append(info);
	}

	return pShape;
}

QEllipseItem* VisionApp::drawEllipse(QMainGraphicsScene* scene, QString mapKey, const qreal& x, const qreal& y, const qreal& radiusX, const qreal& radiusY, const QColor& borderColor, const QColor& innerColor)
{
	QEllipseItem *pShape = new QEllipseItem();

	scene->addItem(pShape);

	pShape->setup(QRectF(x, y, radiusX, radiusY), borderColor, innerColor);

	RenderInfo info;
	info.scene = scene;
	info.item = pShape;

	if (_renderedMaps.contains(mapKey)) {
		_renderedMaps[mapKey].append(info);
	}
	else {
		QVector<RenderInfo> renders;
		_renderedMaps.insert(mapKey, renders);
		_renderedMaps[mapKey].append(info);
	}

	return pShape;
}

QGraphicsTextItem* VisionApp::drawText(QMainGraphicsScene* scene, QString mapKey, const QString& text, const QPointF& pos, const QColor& color, const int& pointSize)
{
	QFont font;
	font.setPointSize(pointSize);

	QGraphicsTextItem *pShape = scene->addText(text, font);
	pShape->setDefaultTextColor(color);
	pShape->setPos(pos);

	RenderInfo info;
	info.scene = scene;
	info.item = pShape;

	if (_renderedMaps.contains(mapKey)) {
		_renderedMaps[mapKey].append(info);
	}
	else {
		QVector<RenderInfo> renders;
		_renderedMaps.insert(mapKey, renders);
		_renderedMaps[mapKey].append(info);
	}

	return pShape;
}

QCrossItem* VisionApp::drawCross(QMainGraphicsScene* scene, QString mapKey, const QRectF& rect, const  QColor& color)
{
	QCrossItem *pShape = new QCrossItem();

	scene->addItem(pShape);

	pShape->setup(rect, color);

	RenderInfo info;
	info.scene = scene;
	info.item = pShape;

	if (_renderedMaps.contains(mapKey)) {
		_renderedMaps[mapKey].append(info);
	}
	else {
		QVector<RenderInfo> renders;
		_renderedMaps.insert(mapKey, renders);
		_renderedMaps[mapKey].append(info);
	}

	return pShape;
}

void VisionApp::clearRenderMap(QString mapKey)
{
	auto& renders = _renderedMaps[mapKey];
	for (int i = 0; i < renders.count(); i++)
	{
		if (renders[i].item != nullptr && renders[i].scene != nullptr) {
			renders[i].scene->removeItem(renders[i].item);
			delete renders[i].item;
			renders[i].item = nullptr;
		}
	}

	renders.clear();
	_renderedMaps.remove(mapKey);
}

void VisionApp::clearAllRenderMaps()
{
	for (auto& renders : _renderedMaps) {
		for (int i = 0; i < renders.count(); i++)
		{
			if (renders[i].item != nullptr && renders[i].scene != nullptr) {
				renders[i].scene->removeItem(renders[i].item);
				delete renders[i].item;
				renders[i].item = nullptr;
			}
		}

		renders.clear();
	}

	_renderedMaps.clear();
}
