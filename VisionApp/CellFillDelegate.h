#pragma once

#include <QColor>
#include <QPainter>
#include <QStyledItemDelegate>

/*
* Paints a table cell's fill from kFillRole (a QColor) underneath the normal painting.
*
* The app's dark tables carry a stylesheet with a QTableWidget::item rule, and with that rule
* in place Qt ignores QTableWidgetItem::setBackground without a word. A cell's text colour
* (Qt::ForegroundRole) still works, so only the fill needs this.
*/
class CellFillDelegate : public QStyledItemDelegate
{
public:
	static constexpr int kFillRole = Qt::UserRole + 1;

	using QStyledItemDelegate::QStyledItemDelegate;

	void paint(QPainter* painter, const QStyleOptionViewItem& option, const QModelIndex& index) const override
	{
		const QVariant fill = index.data(kFillRole);
		if (fill.isValid()) painter->fillRect(option.rect, fill.value<QColor>());
		QStyledItemDelegate::paint(painter, option, index);
	}
};
