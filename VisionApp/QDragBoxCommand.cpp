#include "QDragBoxCommand.h"

AddDragBoxCommand::AddDragBoxCommand(QGraphicsScene* scene, QUndoCommand* parent) {
	m_scene = scene;
	m_item = new QDragBox();
	m_initPos = QPointF(10, 10);
	printf("Add item\n");
}

void AddDragBoxCommand::redo() {
	m_scene->addItem(m_item);
	m_item->setPos(m_initPos);
	m_scene->clearSelection();
	m_scene->update();
}

void AddDragBoxCommand::undo() {
	m_scene->removeItem(m_item);
	m_scene->update();
}

MoveDragBoxCommand::MoveDragBoxCommand(QDragBox* item, const QPointF oldPos, QUndoCommand* parent) {
	m_item = item;
	m_newPos = m_item->pos();
	m_oldPos = oldPos;
}

void MoveDragBoxCommand::redo() {
	if (m_item) {
		m_item->setPos(m_newPos);
	}
	printf("Move item: %f, %f\n", m_item->pos().rx(), m_item->pos().ry());
}

void MoveDragBoxCommand::undo() {
	if (m_item) {
		m_item->setPos(m_oldPos);
		m_item->scene()->update();
	}
	printf("Move item: %f, %f\n", m_item->pos().rx(), m_item->pos().ry());
}

MultiMoveDragBoxCommand::MultiMoveDragBoxCommand(QVector<QDragBox*> items, QDragBox* refItem, const QPointF oldPos, QUndoCommand * parent)
{
	m_items = items;
	m_refItem = refItem;
	m_oldPos = oldPos;

	for (auto item : m_items) {
		if (item) {
			m_newPositions.push_back(item->pos());
		}
	}
}

void MultiMoveDragBoxCommand::redo()
{
	int index = 0;

	for (auto item : m_items) {
		if (item) {
			item->setPos(m_newPositions[index]);
		}
		index++;
	}
}

void MultiMoveDragBoxCommand::undo()
{
	if (m_refItem == nullptr)  return;

	auto refPos = m_refItem->pos();

	for (auto item : m_items) {
		if (item) {
			auto offset = item->pos() - refPos;
			item->setPos(m_oldPos + offset);
			item->scene()->update();
		}
	}
}
