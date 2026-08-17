#include "ProductionPage.h"
#include <QMovie>

ProductionPage::ProductionPage(QWidget *parent)
	: QWidget(parent)
{
	ui.setupUi(this);

	_passPixmapIcon = new QIcon(":/8Icon/Icon/icon8/checked.png");
	_passPixmap = _passPixmapIcon->pixmap(_passPixmapIcon->actualSize(QSize(32, 32)));

	_failPixmapIcon = new QIcon(":/8Icon/Icon/icon8/close.png");
	_failPixmap = _failPixmapIcon->pixmap(_failPixmapIcon->actualSize(QSize(32, 32)));

	QMovie* movieProductionMode = new QMovie(":/VisionApp/Icon/icon8/production-ezgif.com-gif-maker.gif");
	connect(movieProductionMode, &QMovie::frameChanged, this, [=](int frame) {
		ui.toolButton_runningIcon->setIcon(QIcon(movieProductionMode->currentPixmap()));
		});

	// if movie doesn't loop forever, force it to.
	if (movieProductionMode->loopCount() != -1) connect(movieProductionMode, SIGNAL(finished()), movieProductionMode, SLOT(start()));

	if (movieProductionMode->state() == QMovie::NotRunning)
	{
		movieProductionMode->start();
	}

	// Setup timer to update uptime every second
	QTimer* timer = new QTimer(this);
	connect(timer, &QTimer::timeout, this, &ProductionPage::updateUptime);
	timer->start(1000); // Update every second
	
}

ProductionPage::~ProductionPage()
{}

void ProductionPage::updateUptime() {
	// Calculate elapsed time
	qint64 elapsedSeconds = elapsedTimer.elapsed() / 1000;
	int hours = elapsedSeconds / 3600;
	int minutes = (elapsedSeconds % 3600) / 60;
	int seconds = elapsedSeconds % 60;

	// Update label
	if (elapsedTimer.isValid())
	{
		ui.lineEdit_timeElapsed->setText(QString("%1:%2:%3")
			.arg(hours, 2, 10, QChar('0'))
			.arg(minutes, 2, 10, QChar('0'))
			.arg(seconds, 2, 10, QChar('0')));
	}
	
}

void ProductionPage::setCamInfo(QHash<QString, QView>&views, QHash < QString, OpticsInfo>& recipeOptics)
{
	int row = 0;
	int col = 0;

	for (auto c : _viewOpticList)
	{
		delete c;
	}
	_viewOpticList.clear();
	for (auto& v : views)
	{
		for (auto o : v.opticIDs)
		{
			for (const auto& ro : recipeOptics)
			{
				if (ro.id == o)
				{
					QString view_optic = v.name + "_" + ro.name;
					QString viewID_opticID = v.id + "[@]" + ro.id;
					if (ro.id == "opt22") viewID_opticID = v.id + "[@]opt21";
					QCheckBox* checkbox = new QCheckBox;
					checkbox->setText(view_optic);
					checkbox->setWhatsThis(viewID_opticID);
					checkbox->setChecked(true);
					checkbox->setStyleSheet(QString("Color:White"));
					ui.gridLayout_4->addWidget(checkbox, row, col);
					_viewOpticList.append(checkbox);

					connect(checkbox, &QCheckBox::stateChanged, [=](int state) {
						displayCamGraphicViews();
						});

					// Update column and row
					++col;
					if (col == 1) { // Move to the next row after 2 columns
						col = 0;
						++row;
					}
					
				}
			}
		}
	
	}

	displayCamGraphicViews();
}

void ProductionPage::displayCamGraphicViews()
{
	clearCamGraphicViews();

	int count = 0;
	for (auto c : _viewOpticList)
	{
		if (c->isChecked())
		{
			QMainGraphicsView* graphicsView = new QMainGraphicsView();
			graphicsView->setFrameShape(QFrame::NoFrame);
			graphicsView->setRenderHint(QPainter::Antialiasing, false);
			graphicsView->setDragMode(QGraphicsView::RubberBandDrag);
			graphicsView->setOptimizationFlags(QGraphicsView::DontSavePainterState);
			graphicsView->setCacheMode(QGraphicsView::CacheBackground);
			graphicsView->setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
			graphicsView->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
			_camViews.append(graphicsView);

			QMainGraphicsScene* pGraphicsSceneMain = new QMainGraphicsScene(QRectF());
			QRectF sceneBound(0, 0, 500, 500);
			pGraphicsSceneMain->setSceneRect(sceneBound);
			graphicsView->setScene(pGraphicsSceneMain);
			graphicsView->setWhatsThis(c->whatsThis());

			QPixmap pixmapMain = QPixmap(500, 500);
			pixmapMain.fill(QColor(50, 50, 50));
			QGraphicsPixmapItem* pPixmapItemMain = pGraphicsSceneMain->addPixmap(pixmapMain);
			graphicsView->fitInView(pPixmapItemMain, Qt::KeepAspectRatio);

			QLabel* camName = new QLabel();
			camName->setText(c->text());
			camName->setWhatsThis(c->whatsThis());
			camName->setStyleSheet(QString("color:white; background-color: transparent"));
			_camNames.append(camName);


			QTextEdit* textEdit = new QTextEdit();
			textEdit->setStyleSheet(QString("color:white; background-color: transparent; border:none;"));
			textEdit->setReadOnly(true);
			textEdit->setFontPointSize(12);
			_camResults.append(textEdit);

			ui.gridLayout_CamViewer->addWidget(camName, 0, count);
			ui.gridLayout_CamViewer->addWidget(graphicsView, 1, count);
			ui.gridLayout_CamViewer->addWidget(textEdit, 2, count);
			count++;
		}
	}
}

void ProductionPage::updateCamGraphicViews(QString viewID, QString opticID, QString indexID, QImage* img, QHash<QString, ct::UnitResultInfo> unitResultInfos)
{
	//UPH
	//total Index
	//current index
	//yield



	qDebug() << "[ProductionPage] updateCamGraphicVies:" << viewID << "[@]" << opticID << "[@]" << indexID;
	for (int i = 0; i < _camViews.size(); i++)
	{
		QString viewID_opticID = _camViews[i]->whatsThis();

		if (viewID_opticID == QString(viewID + "[@]" + opticID))
		{
			if (img)
			{
				_camResults[i]->setText("");

				auto graphicScene = _camViews[i]->scene();
				for (QGraphicsItem* item : graphicScene->items()) {
					graphicScene->removeItem(item);
					delete item;
				}

				//insert image
				QImage resizedImage = img->scaled(500, 500, Qt::KeepAspectRatio, Qt::SmoothTransformation);
				QGraphicsPixmapItem* pPixmapItemMain = graphicScene->addPixmap(QPixmap::fromImage(resizedImage));

				_camViews[i]->fitInView(pPixmapItemMain, Qt::KeepAspectRatio);

				double scaleFactorWidth = double(resizedImage.width()) / double(img->width());
				double scaleFactorHeight = double(resizedImage.height()) / double(img->height());

				if (unitResultInfos.contains(opticID))
				{
					//insert debug Rect
					auto& unitResultInfo = unitResultInfos[opticID];

					//show Debug Rect
					for (auto debugRect : unitResultInfo.debugBoxes)
					{
						int x = debugRect.x()* scaleFactorWidth;
						int y = debugRect.y() * scaleFactorHeight;
						int w = debugRect.width() * scaleFactorWidth;
						int h = debugRect.height() * scaleFactorHeight;
					
						QPen pen;
						pen.setColor(Qt::blue);
						pen.setWidth(3);
						graphicScene->addRect(QRect(x,y,w,h), pen);
					}

					//show Defect Rect
					for (auto defectRect : unitResultInfo.defectBoxes)
					{
						int x = defectRect.x() * scaleFactorWidth;
						int y = defectRect.y() * scaleFactorHeight;
						int w = defectRect.width() * scaleFactorWidth;
						int h = defectRect.height() * scaleFactorHeight;

						QPen pen;
						pen.setColor(Qt::red);
						pen.setWidth(3);
						graphicScene->addRect(QRect(x, y, w, h), pen);
					}

					//insert time
					QString renderText;
					QString inspectionTime = "Inspection Time: " + unitResultInfo.inspTime + "ms\n";
					
					//show defectCode
					QString defectCode = "Defect Code: " + unitResultInfo.defectCode + "\n";
					QString defectName = "Defect Name: " + unitResultInfo.defectName + "\n";

				
					//show CurrentIndex
					QString indexID = "Index ID: " + unitResultInfo.indexID + "\n";

					//show BarcodeID
					QString barcodeID = "Barcode ID: " + unitResultInfo.barcodeId + "\n";

					renderText = indexID + barcodeID + inspectionTime + defectCode;
					_camResults[i]->setText(renderText);

					//insert pass/Fail Icon
					if (unitResultInfo.pass)
					{
						QGraphicsPixmapItem* statusIcon = graphicScene->addPixmap(_passPixmap);
						statusIcon->setPos(10, 10); // Set the position to (10, 10)
					}
					else
					{
						QGraphicsPixmapItem* statusIcon = graphicScene->addPixmap(_failPixmap);
						statusIcon->setPos(10, 10); // Set the position to (10, 10)
					}

					_totalInspectedTimeMs = _totalInspectedTimeMs +unitResultInfo.inspTime.toDouble();
					// result 
					bool passUnitFlag = true;
					for (auto& u : unitResultInfos)
					{
						if (!u.pass)
						{
							passUnitFlag = false;
							break;
						}
					}

					if (_unitPassHash.contains(indexID))
					{
						if (_unitPassHash[indexID] == true && !passUnitFlag)
						{
							// only take account for unit which is fail
							_unitPassHash[indexID] = passUnitFlag;
							_totalFailUnit++;
						}

					}
					else
					{
						_totalInspectedUnit++;
						_unitPassHash.insert(indexID, passUnitFlag);

						if (!passUnitFlag) _totalFailUnit++;
					}


					_totalPassUnit = _totalInspectedUnit - _totalFailUnit;

					double yield = double(_totalPassUnit) / double(_totalInspectedUnit) * 100.00;
					ui.lineEdit_Yield->setText(QString::number(yield, 'f', 2) + "%");
					ui.lineEdit_currentIndex->setText(indexID);

					
					double totalInspectedTimeHours = (double) elapsedTimer.elapsed() / (1000.0 * 60.0 * 60.0);
					_uphTime = _totalInspectedUnit / totalInspectedTimeHours;
					ui.lineEdit_UPH->setText(QString::number(_uphTime,'f',2));

				
				}

				_camViews[i]->update();
				QGuiApplication::processEvents();
				break;
			}
			
		}
	}
} 


LotInfo ProductionPage::getLotInfos()
{
	LotInfo lInfo;
	lInfo.operatorID = ui.lineEdit_operatorID->text();  
	lInfo.lotID = ui.lineEdit_lotID->text();
	lInfo.totalIndex = ui.lineEdit_totalIndex->text().toInt();
	lInfo.lotID = ui.lineEdit_totalIndex->text();

	return lInfo;
}


void ProductionPage::clearCamGraphicViews()
{
	for (auto camName : _camNames)
	{
		delete camName;
	}
	_camNames.clear();

	for (auto camView : _camViews)
	{
		delete camView;
	}
	_camViews.clear();

	for (auto camRes : _camResults)
	{
		delete camRes;
	}
	_camResults.clear();
	
}


void ProductionPage::inspectionDone()
{
	qDebug() << "Production page inspection done";
	qDebug() << "Total inspected Unit: " << _totalInspectedUnit;
	qDebug() << "Total pass unit: " << _totalPassUnit;
	qDebug() << "Total Failed Unit: " << _totalFailUnit;

	_totalInspectedUnit = 0;
	_totalFailUnit = 0;
	_totalPassUnit = 0;
	_uphTime = 0;
	_unitPassHash.clear();
}

void ProductionPage::startElapseTime()
{
	elapsedTimer.start();
}

void ProductionPage::stopElapseTime()
{
	elapsedTimer.invalidate();
}
