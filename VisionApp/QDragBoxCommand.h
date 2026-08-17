#pragma once
#include <QUndoCommand>
#include <QGraphicsScene>
#include "QDragBox.h"

class AddDragBoxCommand : public QUndoCommand {
public :
	AddDragBoxCommand(QGraphicsScene* scene, QUndoCommand* parent = 0);

	void redo() override;
	void undo() override;

private:
	QDragBox* m_item = nullptr;
	QGraphicsScene* m_scene = nullptr;
	QPointF m_initPos;
};

class MoveDragBoxCommand : public QUndoCommand {
public:
	MoveDragBoxCommand(QDragBox* item, const QPointF oldPos, QUndoCommand* parent = 0);

	void redo() override;
	void undo() override;

private:
	QDragBox* m_item;
	QPointF m_oldPos;
	QPointF m_newPos;
};

class MultiMoveDragBoxCommand : public QUndoCommand {
public:
	MultiMoveDragBoxCommand(QVector<QDragBox*> items, QDragBox* refItem, const QPointF oldPos, QUndoCommand* parent = 0);

	void redo() override;
	void undo() override;

private:
	QVector<QDragBox*> m_items;
	QVector<QPointF> m_newPositions;
	QDragBox* m_refItem = nullptr; 
	QPointF m_oldPos;
};
