#pragma once

#include <QWidget>
#include "ui_ImageViewerTab.h"

#include "QView.h"

class ImageViewerTab : public QWidget
{
	Q_OBJECT

public:
	ImageViewerTab(QWidget* parent = nullptr);
	~ImageViewerTab();

	void setViewInfoList(QHash <QString, QView>& views);
	void updateImageViewerTableWidget();
	void selectFirstImage();

private:
	Ui::ImageViewerTabClass ui;
	QHash <QString, QView> _views;

public Q_SLOTS:
	void tableWidgetImageViewerCellClicked(int row, int column);

signals:
	void displayCurrentView(QString viewID, QString opticID);
};
