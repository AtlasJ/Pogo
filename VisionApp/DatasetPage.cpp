#include "DatasetPage.h"
#include "CommonDir.h"
#include "AuditLog.h"
#include "ImagePathManager.h"
#include <QRegularExpression>
#include <QtConcurrent/QtConcurrent>

DatasetPage::DatasetPage(QWidget *parent)
	: QWidget(parent)
{
	ui.setupUi(this);

	_passPixmapIcon = new QIcon(":/8Icon/Icon/icon8/pass.png");
	_passPixmap = _passPixmapIcon->pixmap(_passPixmapIcon->actualSize(QSize(32, 32)));

	_longPassPixmapIcon = new QIcon(":/8Icon/Icon/icon8/longPassFilled.png");
	_longPassPixmap = _longPassPixmapIcon->pixmap(_longPassPixmapIcon->actualSize(QSize(64, 32)));

	_failPixmapIcon = new QIcon(":/8Icon/Icon/icon8/Fail.png");
	_failPixmap = _failPixmapIcon->pixmap(_failPixmapIcon->actualSize(QSize(32, 32)));

	_longFailPixmapIcon = new QIcon(":/8Icon/Icon/icon8/LongFail.png");
	_longFailPixmap = _longFailPixmapIcon->pixmap(_longFailPixmapIcon->actualSize(QSize(64, 32)));

	_missingPixmapIcon = new QIcon(":/8Icon/Icon/icon8/Missing.png");
	_missingPixmap = _missingPixmapIcon->pixmap(_missingPixmapIcon->actualSize(QSize(32, 32)));

	_longMissingPixmapIcon = new QIcon(":/8Icon/Icon/icon8/LongWarning.png");
	_longMissingPixmap = _longMissingPixmapIcon->pixmap(_longMissingPixmapIcon->actualSize(QSize(64, 32)));

	_checkedPixmapIcon = new QIcon(":/8Icon/Icon/icon8/icons8-checkbox-50.png");
	_checkedPixmap = _checkedPixmapIcon->pixmap(_checkedPixmapIcon->actualSize(QSize(32, 32)));

	_uncheckedPixmapIcon = new QIcon(":/8Icon/Icon/icon8/icons8-uncheckbox-50.png");
	_uncheckedPixmap = _uncheckedPixmapIcon->pixmap(_uncheckedPixmapIcon->actualSize(QSize(32, 32)));

	_noImagePixmap = QPixmap(":/8Icon/Icon/icon8/no-pictures.png");

	connect(ui.graphicsView, SIGNAL(mouseDoubleClick(bool)), this, SLOT(setdragMode(bool)));
	connect(ui.graphicsView, SIGNAL(rightMouseBtnPressed(QPoint)), this, SLOT(setRightMousePressed(QPoint)));

	connect(this, SIGNAL(addPixmapToScene(QGraphicsPixmapItem*)),
		this, SLOT(onAddPixmapToScene(QGraphicsPixmapItem*)), Qt::QueuedConnection);

	connect(ui.checkBox_SelectAllImages, &QCheckBox::stateChanged, [=](int state) {
		selectAllImages();
		});

	QObject::connect(ui.comboBox_ImagePerPage, QOverload<int>::of(&QComboBox::currentIndexChanged), [&](int index) {
		emit refreshDatasetView();
		});

	connect(ui.toolButton_leftPage, SIGNAL(clicked()), this, SLOT(goToPage()));
	connect(ui.toolButton_rightPage, SIGNAL(clicked()), this, SLOT(goToPage()));

	ui.toolButton_filterMissingImages->setChecked(true);
	ui.toolButton_filterFailedImages->setChecked(true);
	ui.toolButton_filterPassImages->setChecked(true);

	connect(ui.toolButton_runStoreUnits, SIGNAL(clicked()), this, SLOT(runStoredUnits()));
	connect(ui.toolButton_storeUnits, SIGNAL(clicked()), this, SLOT(storeUnits()));
	connect(ui.toolButton_removeStoreUnits, SIGNAL(clicked()), this, SLOT(removeStoredUnits()));

	connect(ui.toolButton_filterMissingImages, SIGNAL(clicked()), this, SLOT(filterDatasetView()));
	connect(ui.toolButton_filterFailedImages, SIGNAL(clicked()), this, SLOT(filterDatasetView()));
	connect(ui.toolButton_filterPassImages, SIGNAL(clicked()), this, SLOT(filterDatasetView()));
	connect(ui.toolButton_filterStoredUnits, SIGNAL(clicked()), this, SLOT(filterDatasetView()));

	ui.toolButton_leftPage;
	ui.toolButton_rightPage;
	initGraphicsView();
}

DatasetPage::~DatasetPage()
{}

void DatasetPage::updateDatasetView(QVector<UnitConfigInfo> unitConfigInfos, UnitConfigTab* unitConfigTab, const QHash <QString, QView>& views, QHash<QString, OpticsInfo>& recipeOptics)
{
	_cancelCurrentTask = true;
	//if (_renderingDatasetImage) return;

	clearDatasetDragBox();
	QStringList imageTilesViewIDs;
	QStringList imageTilesOpticIDs;
	QStringList imageTilesIndexIDs;
	QStringList imageTilesPaths;
	QVector<DatasetQDragBox::DatasetStatus> unitStatus;
	for (auto unitConfig : unitConfigInfos)
	{
		QString viewID = unitConfig.viewId;

		QStringList indexIDs = unitConfigTab->getIDList(viewID);

	
		for (QString indexID : indexIDs)
		{
			bool indexIdExist = false;
			if (ui.toolButton_filterStoredUnits->isChecked())
			{
				if (_storedIndexIDs.contains(indexID))
				{
					indexIdExist = true;
				}
			}
			else indexIdExist = true;

			auto ipf = path::getViewPath(Common::Directory::CurrentImageSetPath.toStdString(), views[viewID], OpticsInfo(), recipeOptics);
			QHash<QString, QString> imgPaths;
			imgPaths = ipf.getAllOpticPaths(indexID);

			QHash<QString, QString>::const_iterator imgPath = imgPaths.constBegin();

			while (imgPath != imgPaths.constEnd())
			{
				std::string opticID = imgPath.key().toStdString();
				DatasetQDragBox::DatasetStatus status = DatasetQDragBox::DatasetStatus::PASS;
				//filter the images
				for (auto defectRes : _defectResults)
				{
					if (defectRes.view_id.c_str() == viewID && defectRes.index.c_str() == indexID && defectRes.algoDefResult.optic_id == opticID)
					{
						status = DatasetQDragBox::DatasetStatus::FAIL;
						break;
					}
				}
				if (!QFileInfo::exists(imgPath.value())) status = DatasetQDragBox::DatasetStatus::MISSING;

				if (status == DatasetQDragBox::DatasetStatus::MISSING && ui.toolButton_filterMissingImages->isChecked() || status == DatasetQDragBox::DatasetStatus::FAIL && ui.toolButton_filterFailedImages->isChecked()
					|| status == DatasetQDragBox::DatasetStatus::PASS && ui.toolButton_filterPassImages->isChecked())
				{
					if (indexIdExist)
					{
						unitStatus.append(status);
						imageTilesPaths.append(imgPath.value());
						imageTilesViewIDs.append(viewID);
						imageTilesOpticIDs.append(imgPath.key());
						imageTilesIndexIDs.append(indexID);
					}				
				}
				imgPath++;
			}

		}
	}

	_numOfItemPerPage = extractInteger(ui.comboBox_ImagePerPage->currentText());
	int rows = _numOfItemPerPage / 15;
	int cols = 15;
	int pixmapWidth = 100;
	int pixmapHeight = 100;
	int spacing = 20;

	int graphicW = 143 * cols;
	int graphicH = 145 * rows;
	_sceneBound.setRect(0, 0, graphicW, graphicH);
	_pGraphicsSceneMain->setSceneRect(_sceneBound);

	_totalItem = imageTilesPaths.size();
	_currentPageTotalItem = _currentPageItem + _numOfItemPerPage;
	if (_currentPageTotalItem > _totalItem) _currentPageTotalItem = _totalItem;
	QString currentPageShown = QString::number(_currentPageItem) + "_" + QString::number(_currentPageTotalItem) + " of " + QString::number(_totalItem);
	ui.label_currentPageCount->setText(currentPageShown);
	ui.label_currentPageCount->setWhatsThis(QString::number(1));

	for (QGraphicsItem* item : _pGraphicsSceneMain->items()) {
		_pGraphicsSceneMain->removeItem(item);
		delete item;
	}

	if(unitStatus.size() == 0) return;
	QStringList imgPaths;
	QVector<qreal> xVector;
	QVector<qreal> yVector;
	int index = _currentPageItem - 1;
	for (int row = 0; row < rows; ++row) {
		for (int col = 0; col < cols; ++col) {
			// Calculate the position for each pixmap item
			qreal x = col * (pixmapWidth + spacing * 2) + 50;
			qreal y = row * (pixmapHeight + spacing * 2) + 50;
			xVector.append(x);
			yVector.append(y);

			//get tile Info
			QString imgPath = imageTilesPaths[index];
			QString viewID = imageTilesViewIDs[index];
			QString opticID = imageTilesOpticIDs[index];
			QString indexID = imageTilesIndexIDs[index];

			QRect roiRect = QRect(x - spacing / 2, y - spacing / 2, pixmapWidth + spacing, pixmapHeight + spacing);
			auto roiItem = drawDatasetDragBox(roiRect, viewID, opticID, indexID, Qt::magenta);
			if (!QFileInfo::exists(imgPath)) roiItem->setDatasetStatus(DatasetQDragBox::DatasetStatus::MISSING);
			else roiItem->setDatasetStatus(unitStatus[index]);

			imgPaths.append(imgPath);
			//qDebug() << "roiRect:" << roiRect;
			index++;
			if (index == imageTilesPaths.size()) break;
		}
		if (index == imageTilesPaths.size()) break;
	}
	updateDatasetDragBoxStatus();

	QtConcurrent::run([this, imgPaths, pixmapWidth, pixmapHeight, xVector, yVector]() {

		_cancelCurrentTask = false;
		//_renderingDatasetImage = true;

	/*	if (imgPaths.size() == xVector.size() == yVector.size())
		{*/
			for (int i = 0; i < imgPaths.size(); i++)
			{
				if (_cancelCurrentTask) {
					qDebug() << "Task canceled!";
					break;
				}
				// Create and add the pixmap item to the scene
				QPixmap pixmap(imgPaths[i]);
				if (pixmap.isNull())
				{
					pixmap = _noImagePixmap;
				}
				QRoundedPixmapItem* pixmapItem = new QRoundedPixmapItem(pixmap.scaled(pixmapWidth, pixmapHeight));
				pixmapItem->setPos(xVector[i], yVector[i]);
				pixmapItem->setZValue(0);

				// Emit the signal to add the pixmap to the scene
				emit addPixmapToScene(pixmapItem);
			}
		//}
			
		//_renderingDatasetImage = false;


		});


	ui.graphicsView->show();
}

void DatasetPage::addDefectResults(QVector<ct::DefectResult>& defectResults)
{
	_defectResults = defectResults;
}

void DatasetPage::initGraphicsView()
{
	//Main view
	ui.graphicsView->setRenderHint(QPainter::Antialiasing, false);
	ui.graphicsView->setDragMode(QGraphicsView::RubberBandDrag);
	ui.graphicsView->setOptimizationFlags(QGraphicsView::DontSavePainterState);
	ui.graphicsView->setCacheMode(QGraphicsView::CacheBackground);
	ui.graphicsView->setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
	ui.graphicsView->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);

	_sceneBound.setRect(0, 0, 1430, 870);
	_pGraphicsSceneMain = new QMainGraphicsScene(QRectF());
	_pGraphicsSceneMain->setSceneRect(_sceneBound);
	ui.graphicsView->setScene(_pGraphicsSceneMain);

	connect(ui.graphicsView, SIGNAL(mouseMove(QPoint)), this, SLOT(updateImagesSelected()));

	//// Create a green pixmap
	//int rows = 6;
	//int cols = 10;
	//int pixmapWidth = 100;
	//int pixmapHeight = 100;
	//int spacing = 20;
	////QPixmap pixmap("C:/Advanced/Cache/runSingleVidiInspectionImg_optic123.jpg");
	//QPixmap pixmap("C:/Users/cheon/oneDrive/Desktop/OnGoing Projects/IM610/Die.png");
	//

	//for (int row = 0; row < rows; ++row) {
	//	for (int col = 0; col < cols; ++col) {
	//		// Calculate the position for each pixmap item
	//		qreal x = col * (pixmapWidth + spacing*2) + 50;
	//		qreal y = row * (pixmapHeight + spacing*2) + 50;

	//		// Create and add the pixmap item to the scene
	//		QRoundedPixmapItem* pixmapItem = new QRoundedPixmapItem(pixmap.scaled(pixmapWidth, pixmapHeight));
	//		pixmapItem->setPos(x, y);
	//		pixmapItem->setZValue(0);
	//		_pGraphicsSceneMain->addItem(pixmapItem);		

	//		QRect roiRect = QRect(x - spacing / 2, y - spacing / 2, pixmapWidth + spacing, pixmapHeight + spacing);
	//		auto roiItem = drawDatasetDragBox(roiRect, Qt::magenta);
	//	
	//		//qDebug() << "roiRect:" << roiRect;
	//	}
	//}

	///*AnimatedGraphicsItem* item = new AnimatedGraphicsItem(_longPassPixmap, 100, 100);
	//_pGraphicsSceneMain->addItem(item);*/
	//ui.graphicsView->show();
}

DatasetQDragBox* DatasetPage::drawDatasetDragBox(const QRectF& rect, const QString& viewID, const QString& opticID, const QString& indexID, const QColor& color, const QString& name)
{
	DatasetQDragBox* pShape = new DatasetQDragBox();

	pShape->setup(rect, color, name);
	pShape->setOutterBarrier(_sceneBound);
	pShape->setOutterBarrier(_pGraphicsSceneMain->sceneRect());

	auto r = _pGraphicsSceneMain->sceneRect();

	pShape->viewID(viewID);
	pShape->opticID(opticID);
	pShape->indexID(indexID);

	pShape->setDragable(false);
	pShape->setPassIcons(&_passPixmap, & _longPassPixmap);
	pShape->setFailIcons(&_failPixmap, &_longFailPixmap);
	pShape->setMissingIcons(&_missingPixmap, &_longMissingPixmap);
	pShape->setCheckBoxIcons(&_checkedPixmap, &_uncheckedPixmap);
	pShape->setZValue(2);

	_pGraphicsSceneMain->addItem(pShape);

	/*connect(pShape, SIGNAL(dragBoxCreated(QRectF)), this, SLOT(dragBoxCreatedEvent(QRectF)));
	connect(pShape, SIGNAL(dragBoxMousePressed(QDragBox*, QString, QPointF)), this, SLOT(dragBoxMousePressedEvent(QDragBox*, QString, QPointF)));
	connect(pShape, SIGNAL(dragBoxMouseReleased(QDragBox*, QString, QPointF)), this, SLOT(dragBoxMouseReleasedEvent(QDragBox*, QString, QPointF)));
	connect(pShape, SIGNAL(dragBoxMouseHoverEntered(QDragBox*)), this, SLOT(dragBoxMouseHoverEntered(QDragBox*)));
	connect(pShape, SIGNAL(dragBoxMouseHoverLeaved(QDragBox*, QString)), this, SLOT(dragBoxMouseHoverLeaved(QDragBox*, QString)));
	connect(pShape, SIGNAL(dragBoxContextMenuEvent(QDragBox*, QString, QPointF)), this, SLOT(dragBoxContextMenuEvent(QDragBox*, QString, QPointF)));
	connect(pShape, SIGNAL(dragBoxResized(QDragBox*, QString)), this, SLOT(dragBoxResized(QDragBox*, QString)));
	connect(pShape, SIGNAL(grabberPressed(QDragBox*)), this, SLOT(grabberPressed(QDragBox*)));
	connect(pShape, SIGNAL(grabberReleased(QDragBox*)), this, SLOT(grabberReleased(QDragBox*)));*/

	_datasetQDragBoxes.append(pShape);

	return pShape;

	return nullptr;
}

void DatasetPage::clearDatasetDragBox()
{
	for (int i = 0; i < _datasetQDragBoxes.count(); i++)
	{	
		_pGraphicsSceneMain->removeItem(_datasetQDragBoxes.at(i));
		delete _datasetQDragBoxes[i];

		_datasetQDragBoxes.removeAt(i);
		i--;
	}
}

void DatasetPage::updateDatasetDragBoxStatus()
{
	for (auto dragBox : _datasetQDragBoxes)
	{
		auto viewID = dragBox->viewID().toStdString();
		auto indexID = dragBox->indexID().toStdString();
		auto opticID = dragBox->opticID().toStdString();

		if (dragBox->getDatasetStatus() != DatasetQDragBox::DatasetStatus::MISSING)
		{
			bool isPass = true;
			for (auto defectRes : _defectResults)
			{
				if (defectRes.view_id == viewID && defectRes.index == indexID && defectRes.algoDefResult.optic_id == opticID)
				{
					dragBox->setDatasetStatus(DatasetQDragBox::DatasetStatus::FAIL);
					isPass = false;
					break;
				}
			}

			if (isPass)
			{
				{
					dragBox->setDatasetStatus(DatasetQDragBox::DatasetStatus::PASS);
				}
			}
		}
	
	}
}

int DatasetPage::extractInteger(const QString& text)
{
	QRegularExpression regex("(\\d+)");
	QRegularExpressionMatch match = regex.match(text);

	if (match.hasMatch()) {
		return match.captured(1).toInt(); // Extract the first matched group and convert to int
	}

	return 0; // Return -1 if no integer is found
}

void DatasetPage::selectAllImages()
{
	if (ui.checkBox_SelectAllImages->isChecked())
	{
		for (auto dragRoi : _datasetQDragBoxes)
		{
			dragRoi->setSelected(true);
		}
	}
	else
	{
		for (auto dragRoi : _datasetQDragBoxes)
		{
			dragRoi->setSelected(false);
		}
	}
	

	updateImagesSelected();
}

void DatasetPage::goToPage()
{
	_numOfItemPerPage;
	_currentPageItem;
	_currentPageTotalItem;
	_totalItem;

	QObject* senderObj = sender();
	if (senderObj == ui.toolButton_leftPage)
	{
		_currentPageItem = _currentPageItem - _numOfItemPerPage;
		if (_currentPageItem < 1) _currentPageItem = 1;
		emit refreshDatasetView();
	}
	else if (senderObj == ui.toolButton_rightPage)
	{
		_currentPageItem = _currentPageTotalItem;
		emit refreshDatasetView();
	}
}

void DatasetPage::showCurrentUnit()
{
	if (_currentDatasetQDragBox)
	{
		
		QString viewID = _currentDatasetQDragBox->viewID();
		QString opticID = _currentDatasetQDragBox->opticID();
		QString indexID = _currentDatasetQDragBox->indexID();

		qDebug() << "showCurrentUnit viewID:" << viewID << " opticID:" << opticID << " indexID:" << indexID;


		emit displayCurrentView(viewID, opticID, indexID);
	}
}

void DatasetPage::filterDatasetView()
{
	emit refreshDatasetView();
}

void DatasetPage::setdragMode(bool flag)
{
	_dragMode = flag;

	if (_dragMode)
	{
		for (int i = 0; i < _datasetQDragBoxes.size(); i++)
		{
			_datasetQDragBoxes.at(i)->setFlag(QGraphicsItem::ItemIsSelectable, false);
			_datasetQDragBoxes.at(i)->setFlag(QGraphicsItem::ItemIsMovable, false);
			_datasetQDragBoxes.at(i)->setDragable(false);
		}
	}
	else
	{
		for (int i = 0; i < _datasetQDragBoxes.size(); i++)
		{
			_datasetQDragBoxes.at(i)->setFlag(QGraphicsItem::ItemIsSelectable, true);
			_datasetQDragBoxes.at(i)->setFlag(QGraphicsItem::ItemIsMovable, true);
			_datasetQDragBoxes.at(i)->setDragable(true);
		}
	}
}

void DatasetPage::setRightMousePressed(QPoint point)
{
	QPoint pointCursor = QCursor::pos();

	for (int i = 0; i < _datasetQDragBoxes.size(); i++)
	{
		if (_datasetQDragBoxes[i]->isSelected())
		{
			_datasetQDragBoxes[i]->setSelected(false);
		}
	}

	DatasetQDragBox* dragbox = nullptr;
	int maxZ = -99999;
	for (int i = 0; i < _datasetQDragBoxes.size(); i++)
	{
		auto x = _datasetQDragBoxes[i]->pos().x();
		auto y = _datasetQDragBoxes[i]->pos().y();
		auto w = _datasetQDragBoxes[i]->getGeometry().width();
		auto h = _datasetQDragBoxes[i]->getGeometry().height();
		if ((point.x() < x + w) && (point.x() > x) && (point.y() < y + h) && (point.y() > y) && _datasetQDragBoxes[i]->isVisible())
		{
			if (_datasetQDragBoxes[i]->zValue() > maxZ)
			{
				maxZ = _datasetQDragBoxes[i]->zValue();
				dragbox = _datasetQDragBoxes[i];
			}
		}
	}

	_currentDatasetQDragBox = dragbox;

	if (dragbox != nullptr)
	{
		
		dragbox->setSelected(true);

		QMenu menu(this);

		menu.addAction(tr("Show Current Unit"), this, SLOT(showCurrentUnit()));
		menu.exec(QCursor::pos());
	}
}

void DatasetPage::onAddPixmapToScene(QGraphicsPixmapItem* pixmapItem)
{
	_pGraphicsSceneMain->addItem(pixmapItem); // Add the item to the scene
}

QStringList DatasetPage::getFilteredUnitIndexIDs()
{
	QStringList indexIDs;
	for (int i = 0; i < _datasetQDragBoxes.size(); i++)
	{
		indexIDs.append(_datasetQDragBoxes[i]->indexID());
	}
	return indexIDs;
}

void DatasetPage::runStoredUnits()
{
	emit runStoredUnitsInspection(_storedIndexIDs);
}

void DatasetPage::storeUnits()
{
	for (auto dataset : _datasetQDragBoxes)
	{
		if (dataset->isSelected())
		{
			if (!_storedIndexIDs.contains(dataset->indexID())) _storedIndexIDs.append(dataset->indexID());
		}
	}

	QString storedUnitsList = _storedIndexIDs.join(", ");
	ui.lineEdit_storedUnits->setText(storedUnitsList);
}

void DatasetPage::removeStoredUnits()
{
	qDebug() << "removeStoredUnits";
	AuditLog::instance().log(QStringLiteral("DATASET_REMOVE_STORED_UNITS"), _storedIndexIDs.join(", "));
	_storedIndexIDs.clear();
	ui.lineEdit_storedUnits->setText("");
}

void DatasetPage::updateImagesSelected()
{
	int itemSelected = 0;
	for (auto dragRoi : _datasetQDragBoxes)
	{
		if (dragRoi->checkBoxChecked()) itemSelected++;
	}

	QString text = QString::number(itemSelected) + " Images Selected";
	ui.checkBox_SelectAllImages->setText(text);
}
