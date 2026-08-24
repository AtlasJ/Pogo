#pragma once
#include <QTableWidget>
#include <QDropEvent>
#include <QHeaderView>

//QTableWidget with working whole-row drag reorder. The stock InternalMove mode
//overwrites cells and leaves blank rows, so the drop is reimplemented to move
//the selected rows as a block to the drop position.
class DryRunTableWidget : public QTableWidget {
public:
	explicit DryRunTableWidget(QWidget* parent = nullptr) : QTableWidget(parent)
	{
		setSelectionBehavior(QAbstractItemView::SelectRows);
		setSelectionMode(QAbstractItemView::ExtendedSelection);
		setDragEnabled(true);
		setAcceptDrops(true);
		viewport()->setAcceptDrops(true);
		setDragDropMode(QAbstractItemView::InternalMove);
		setDropIndicatorShown(true);
		setDragDropOverwriteMode(false);
	}

protected:
	void dropEvent(QDropEvent* event) override
	{
		if (event->source() != this) { QTableWidget::dropEvent(event); return; }

		int dropRow = rowAt(event->pos().y());
		if (dropRow < 0) dropRow = rowCount();

		//collect selected rows in ascending order
		QList<int> rows;
		for (const auto& range : selectedRanges())
			for (int r = range.topRow(); r <= range.bottomRow(); r++)
				if (!rows.contains(r)) rows.append(r);
		std::sort(rows.begin(), rows.end());
		if (rows.isEmpty()) { event->ignore(); return; }

		//take the row contents
		QVector<QList<QTableWidgetItem*>> taken;
		for (int r : rows) {
			QList<QTableWidgetItem*> items;
			for (int c = 0; c < columnCount(); c++) items.append(takeItem(r, c));
			taken.append(items);
		}

		//remove from bottom up, adjusting the drop position
		for (int i = rows.size() - 1; i >= 0; i--) {
			removeRow(rows[i]);
			if (rows[i] < dropRow) dropRow--;
		}

		//reinsert as a block at the drop position
		clearSelection();
		for (int i = 0; i < taken.size(); i++) {
			int r = dropRow + i;
			insertRow(r);
			for (int c = 0; c < columnCount(); c++) setItem(r, c, taken[i][c]);
		}
		for (int i = 0; i < taken.size(); i++)
			for (int c = 0; c < columnCount(); c++)
				if (auto* it = item(dropRow + i, c)) it->setSelected(true);

		event->accept();
	}
};
