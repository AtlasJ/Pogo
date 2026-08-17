#include "ImageViewerTab.h"
#include "CommonDir.h"
#include <QFile>
#include <QDebug>

ImageViewerTab::ImageViewerTab(QWidget* parent)
	: QWidget(parent)
{
	ui.setupUi(this);

	QStringList horizontalLabel = { tr("Name") , tr(" "), tr("ID") };
	int col = 3;
	ui.tableWidget_imageViewer->setColumnCount(col);
	//ui.tableWidget_TemplateLibrary->horizontalHeader()->setSectionResizeMode(QHeaderView::Interactive);
	ui.tableWidget_imageViewer->horizontalHeader()->setStretchLastSection(true);
	ui.tableWidget_imageViewer->verticalHeader()->setVisible(false);
	ui.tableWidget_imageViewer->setHorizontalHeaderLabels(horizontalLabel);
	ui.tableWidget_imageViewer->setEditTriggers(QAbstractItemView::NoEditTriggers);
	ui.tableWidget_imageViewer->setContextMenuPolicy(Qt::CustomContextMenu);
	ui.tableWidget_imageViewer->resizeRowsToContents();
	ui.tableWidget_imageViewer->resizeColumnsToContents();

	connect(ui.tableWidget_imageViewer, SIGNAL(cellClicked(int, int)), this, SLOT(tableWidgetImageViewerCellClicked(int, int)));
}

ImageViewerTab::~ImageViewerTab()
{}

void ImageViewerTab::setViewInfoList(QHash<QString, QView>& views)
{
	return;
	_views = views;
	updateImageViewerTableWidget();
} 

void ImageViewerTab::updateImageViewerTableWidget()
{
	qDebug() << "updateImageViewerTableWidget";
	for (int i = 0; i < ui.tableWidget_imageViewer->rowCount(); i++)
	{
		ui.tableWidget_imageViewer->removeRow(i);
	}

	int i = 0;
	int size = 100;
	ui.tableWidget_imageViewer->setRowCount(_views.size());
	for (auto &v: _views)
	{
		ui.tableWidget_imageViewer->setItem(i, 0, new QTableWidgetItem(v.name));
		QTableWidgetItem* pImage = new QTableWidgetItem();
		QPixmap img;

		QString viewImagePath;
		for (auto o : v.opticIDs)
		{
			viewImagePath = Common::Directory::getRecipeSetupImagePath() + v.id + "_" + o + "_R0C0.jpg";
			qDebug() << "viewImagePath:" << viewImagePath;
			if (QFile::exists(viewImagePath)) break;
		}

		if (viewImagePath.isEmpty() || !img.load(viewImagePath))
		{
			qDebug() << "viewImageFailedToLoad";
			img.load(":/8Icon/Icon/icon8/no-pictures.png");
			QString noImage = tr("<p style = font-size:20px>Setup Image do not exist.<b><font color='red'></font></b> Please collect setup image for display<b><font color='#09ff00'></font></b>!</p>");
			pImage->setToolTip(noImage);
		}

		img = img.scaled(size, size, Qt::KeepAspectRatio, Qt::SmoothTransformation);
		pImage->setData(Qt::DecorationRole, img);

		ui.tableWidget_imageViewer->setItem(i, 1, pImage);
		ui.tableWidget_imageViewer->setItem(i, 2, new QTableWidgetItem(v.id)); // no of Vision Object
		i++;
	}
	ui.tableWidget_imageViewer->horizontalHeader()->setStretchLastSection(true);
	ui.tableWidget_imageViewer->resizeRowsToContents();
	ui.tableWidget_imageViewer->resizeColumnsToContents();

	if (ui.tableWidget_imageViewer->rowCount() > 0) tableWidgetImageViewerCellClicked(0, 2);
}

void ImageViewerTab::selectFirstImage()
{
	if(ui.tableWidget_imageViewer->rowCount() > 0) tableWidgetImageViewerCellClicked(0, 0);
}

void ImageViewerTab::tableWidgetImageViewerCellClicked(int row, int column)
{
	QString viewID = ui.tableWidget_imageViewer->item(row, 2)->text();
	for (auto & v : _views)
	{
		if (v.id == viewID)
		{
			emit displayCurrentView(viewID, QString());
			return;
		}
	}
}
