#include "TemplateLibraryTab.h"
#include <QFileDialog>
#include <QInputDialog>
#include <QMessageBox>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QFile>
#include "VisionApp.h"
#include "CommonDir.h"
#include "AuditLog.h"
#include "uidGenerator.h"

TemplateLibraryTab::TemplateLibraryTab(QWidget *parent)
	: QWidget(parent)
{
	ui.setupUi(this);

	ui.tableWidget_TemplateProperties->hide();


	connect(ui.toolButtonAddTemplate, SIGNAL(clicked()), this, SLOT(addTemplate()));
	connect(ui.toolButtonDeleteTemplate, SIGNAL(clicked()), this, SLOT(deleteTemplate()));
	connect(ui.toolButtonEditTemplate, SIGNAL(clicked()), this, SLOT(editTemplate()));
	connect(ui.checkBoxUniformBox, SIGNAL(clicked()), this, SLOT(updateUniformBoxFlag()));
	connect(ui.comboBox_templateAlgo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [=](int index) {
		auto algoTemplateList = _algoTemplateList.getAlgoTemplateList();
		for (auto tmpl : algoTemplateList) {
			if (tmpl->templateName() == ui.lineEditTemplateName->text()) {
				tmpl->algo((AlgoPageAlgo)index);
				saveTemplateList();
				break;
			}
		}
	});
	connect(ui.toolButtonGeneratePreprocessedTemplateImages, &QToolButton::clicked, this, [=]() {
		generateVIDIImages(true);
	});
	connect(ui.toolButtonGenerateVIDIImages, &QToolButton::clicked, this, [=]() {
		generateVIDIImages(false);
	});
	connect(ui.toolButton_goldenRecipe, &QToolButton::clicked, this, [=]() {
		emit  signalOpenGoldenRecipeDialog();
		});

	connect(ui.toolButtonPadding, SIGNAL(clicked()), this, SLOT(addPadding()));
	connect(ui.toolButtonResizeTemplate, SIGNAL(clicked()), this, SLOT(resizeTemplate()));
	connect(ui.toolButtonSaveRefImage, SIGNAL(clicked()), this, SLOT(saveRefImage()));
	connect(ui.toolButton_saveGoldenTemplateList, &QToolButton::clicked,this, &TemplateLibraryTab::saveGoldenTemplateList);
	connect(ui.toolButton_loadGoldenTemplateList, &QToolButton::clicked,this, &TemplateLibraryTab::loadGoldenTemplateList);

	QStringList horizontalLabel = { tr("Template Name") , tr(" "), tr("Template Image"), tr("No. Object"), tr("Status")};
	int col = 5;
	ui.tableWidget_TemplateLibrary->setColumnCount(col);
	//ui.tableWidget_TemplateLibrary->horizontalHeader()->setSectionResizeMode(QHeaderView::Interactive);
	ui.tableWidget_TemplateLibrary->horizontalHeader()->setStretchLastSection(true);
	ui.tableWidget_TemplateLibrary->verticalHeader()->setVisible(false);
	ui.tableWidget_TemplateLibrary->setHorizontalHeaderLabels(horizontalLabel);
	ui.tableWidget_TemplateLibrary->setEditTriggers(QAbstractItemView::NoEditTriggers);
	ui.tableWidget_TemplateLibrary->setContextMenuPolicy(Qt::CustomContextMenu);
	ui.tableWidget_TemplateLibrary->resizeRowsToContents();
	ui.tableWidget_TemplateLibrary->resizeColumnsToContents();

	ui.tableWidget_TemplateLibrary->hideColumn(3);
	ui.tableWidget_TemplateLibrary->hideColumn(4);

	QStringList statTablehorizontalLabel = { tr("No."), tr("Status")};
	col = 2;
	ui.tableWidget_TemplateProperties->setColumnCount(col);
	//ui.tableWidget_TemplateLibrary->horizontalHeader()->setSectionResizeMode(QHeaderView::Interactive);
	ui.tableWidget_TemplateProperties->horizontalHeader()->setStretchLastSection(true);
	ui.tableWidget_TemplateProperties->verticalHeader()->setVisible(false);
	ui.tableWidget_TemplateProperties->setHorizontalHeaderLabels(statTablehorizontalLabel);
	ui.tableWidget_TemplateProperties->setEditTriggers(QAbstractItemView::NoEditTriggers);
	ui.tableWidget_TemplateProperties->setContextMenuPolicy(Qt::CustomContextMenu);
	ui.tableWidget_TemplateProperties->resizeRowsToContents();
	ui.tableWidget_TemplateProperties->resizeColumnsToContents();

	connect(ui.tableWidget_TemplateLibrary, SIGNAL(cellClicked(int, int)), this, SLOT(tableWidgetTemplateLibCellClicked(int, int)));
	connect(ui.tableWidget_TemplateLibrary, SIGNAL(customContextMenuRequested(QPoint)), this, SLOT(customContextMenuRequested(QPoint)));
}

TemplateLibraryTab::~TemplateLibraryTab()
{
}

QString TemplateLibraryTab::currentTemplateId()
{
	auto algoTemplateList = _algoTemplateList.getAlgoTemplateList();

	for (int i = 0; i < algoTemplateList.size(); i++)
	{
		if (algoTemplateList[i]->templateName() == ui.lineEditTemplateName->text()) return algoTemplateList[i]->templateId();
	}
	return QString();
}

QString TemplateLibraryTab::currentTemplateName()
{
	return ui.lineEditTemplateName->text();
}

QColor TemplateLibraryTab::currentTemplateColor()
{
	auto algoTemplateList = _algoTemplateList.getAlgoTemplateList();
	for (int i = 0; i < algoTemplateList.size(); i++)
	{
		if (algoTemplateList[i]->templateName() == ui.lineEditTemplateName->text()) return QColor(algoTemplateList[i]->color());
	}
	return Qt::white;
}

QColor TemplateLibraryTab::getTemplateColor(QString & templateID)
{
	auto algoTemplateList = _algoTemplateList.getAlgoTemplateList();
	for (int i = 0; i < algoTemplateList.size(); i++)
	{
		if (algoTemplateList[i]->templateId() == templateID) return QColor(algoTemplateList[i]->color());
	}
	return Qt::white;
}

AlgoTemplate * TemplateLibraryTab::currentAlgoTemplate()
{
	auto algoTemplateList = _algoTemplateList.getAlgoTemplateList();
	for (int i = 0; i < algoTemplateList.size(); i++)
	{
		if (algoTemplateList[i]->templateName() == ui.lineEditTemplateName->text()) return algoTemplateList[i];
	}
	return nullptr;
}

AlgoTemplate * TemplateLibraryTab::getAlgoTemplate(QString & templateID)
{
	auto algoTemplateList = _algoTemplateList.getAlgoTemplateList();
	for (int i = 0; i < algoTemplateList.size(); i++)
	{
		if (algoTemplateList[i]->templateId() == templateID) return algoTemplateList[i];
	}
	return nullptr;
}

void TemplateLibraryTab::setTemplateImagePath(const QString & id, const QString & templateImagePath, const QSize& size)
{
	QString templateName;
	auto algoTemplateList = _algoTemplateList.getAlgoTemplateList();
	for (int i = 0; i < algoTemplateList.size(); i++)
	{
		if (algoTemplateList[i]->templateId() == id)
		{
			algoTemplateList[i]->templateImagePath(templateImagePath);
			algoTemplateList[i]->w(size.width());
			algoTemplateList[i]->h(size.height());
			templateName = algoTemplateList[i]->templateName();
		}
	}
	updateTemplateLibraryTableWidget();
	saveTemplateList();

	selectTemplateTableRow(templateName);
}

void TemplateLibraryTab::addTemplate()
{
	bool ok;
	QString templateName = QInputDialog::getText(this, tr("Add Template"), tr("Template:"), QLineEdit::Normal, "", &ok, Qt::CoverWindow);
	templateName.remove(" ");

	if (templateName.isEmpty())
	{
		QMessageBox::warning(this, tr("No Template Name"),
			tr("Please key in new template name!!!"));
		return;
	}
	else if (templateNameExist(templateName))
	{
		QMessageBox::warning(this, tr("Template Name repeated"),
			tr("Template Name: ") + templateName + tr(" -- has already existed!!!"));
		return;
	}
	else
	{
		_algoTemplateList.addAlgoTemplate(templateName, getUnusedColor());
		updateTemplateLibraryTableWidget();
		saveTemplateList();

		selectTemplateTableRow(templateName);
	}
}

bool TemplateLibraryTab::templateNameExist(const QString & templateName)
{
	return _algoTemplateList.algoTemplateExist(templateName);
}

QString TemplateLibraryTab::getUnusedColor()
{
	auto algoTemplateList = _algoTemplateList.getAlgoTemplateList();
	for (int i = 0; i < colorPallete.size(); i++)
	{
		QString color = colorPallete[i];
		bool colorExist = false;
		for (int j = 0; j < algoTemplateList.size(); j++)
		{
			QString algoTemplateColor = algoTemplateList[j]->color();
			if (algoTemplateColor == color) colorExist = true;
		}

		if (!colorExist) return color;
	}

	return QString();
}

bool TemplateLibraryTab::checkColourUsed(QColor colour)
{
	auto algoTemplateList = _algoTemplateList.getAlgoTemplateList();

	for (int i = 0; i < algoTemplateList.size(); i++)
	{
		QColor templateColour = algoTemplateList[i]->color();
		if (templateColour == colour) return true;
	}
	return false;
}

void TemplateLibraryTab::updateTemplateLibraryTableWidget()
{
	for (int i = 0; i < ui.tableWidget_TemplateLibrary->rowCount(); i++)
	{
		ui.tableWidget_TemplateLibrary->removeRow(i);
	}


	auto algoTemplateList = _algoTemplateList.getAlgoTemplateList();

	int size = 100;
	ui.tableWidget_TemplateLibrary->setRowCount(algoTemplateList.size());
	for (int i = 0; i < algoTemplateList.size(); i++)
	{
		auto algoTemplate = algoTemplateList[i];
		ui.tableWidget_TemplateLibrary->setItem(i, 0, new QTableWidgetItem(algoTemplate->templateName()));
		ui.tableWidget_TemplateLibrary->setItem(i, 1, new QTableWidgetItem(" "));
		QColor color = algoTemplate->color();
		ui.tableWidget_TemplateLibrary->item(i, 1)->setBackground(color);
		QTableWidgetItem* pImage = new QTableWidgetItem();
		QPixmap img;

		QString templateImagePath = Common::Directory::getRecipeCurrentPath() + "template_Images\\" + algoTemplate->templateImagePath();
		if (algoTemplate->templateImagePath().contains("c:\\Advanced\\Data\\recipe"))
		{
			int lastSlashIndex = algoTemplate->templateImagePath().lastIndexOf('\\');
			int firstSlashIndex = algoTemplate->templateImagePath().indexOf('\\', lastSlashIndex + 1);
			algoTemplate->templateImagePath(algoTemplate->templateImagePath().section('\\', firstSlashIndex));
			templateImagePath = Common::Directory::getRecipeCurrentPath() + "template_Images\\" + algoTemplate->templateImagePath();
			qDebug() << "templateImagePath:" << templateImagePath;
			
		}
		if (algoTemplate->templateImagePath().isEmpty() || !img.load(templateImagePath))
		{
			qDebug() << "templateImageFailedToLoad";
			img.load(":/8Icon/Icon/icon8/no-pictures.png");
			QString noImage = tr("<p style = font-size:20px>Default Vision Object<b><font color='red'> not set.</font></b> Right click on a Vision Object and select <b><font color='#09ff00'>add to template</font></b>!</p>");
			pImage->setToolTip(noImage);
			algoTemplate->w(0);
			algoTemplate->h(0);
		}

		algoTemplate->w(img.width());
		algoTemplate->h(img.height());
		img = img.scaled(size, size, Qt::KeepAspectRatio, Qt::SmoothTransformation);
		pImage->setData(Qt::DecorationRole, img);
		
		ui.tableWidget_TemplateLibrary->setItem(i, 2, pImage); //template Image
		ui.tableWidget_TemplateLibrary->setItem(i, 3, new QTableWidgetItem(0)); // no of Vision Object
		ui.tableWidget_TemplateLibrary->setItem(i, 4, new QTableWidgetItem()); //status
		if (i == algoTemplateList.size() - 1)
		{
			ui.tableWidget_TemplateLibrary->selectRow(i);
			ui.tableWidget_TemplateLibrary->item(i, 1)->setSelected(false);
		}
	}
	ui.tableWidget_TemplateLibrary->horizontalHeader()->setStretchLastSection(true);
	ui.tableWidget_TemplateLibrary->resizeRowsToContents();
	ui.tableWidget_TemplateLibrary->resizeColumnsToContents();

}


void TemplateLibraryTab::editTemplate()
{
	AuditLog::instance().log(QStringLiteral("TEMPLATE_EDIT"), ui.lineEditTemplateName->text());
	emit editTemplateSignal();
}

void TemplateLibraryTab::deleteTemplate()
{
	QString deleteTemplateMessage = tr("Are you sure you want to delete the template:") + ui.lineEditTemplateName->text() + "?";
	QMessageBox::StandardButton reply = QMessageBox::question(this, "Delete Template", deleteTemplateMessage, QMessageBox::Yes | QMessageBox::No);
	if (reply == QMessageBox::No) return;
	else
	{
		AuditLog::instance().log(QStringLiteral("TEMPLATE_DELETE"), ui.lineEditTemplateName->text());
		auto algoTemplateList = _algoTemplateList.getAlgoTemplateList();
		for (int i = 0; i < algoTemplateList.size(); i++)
		{
			if (algoTemplateList[i]->templateName() == ui.lineEditTemplateName->text())
			{
				emit deleteVisionObjectTemplate(algoTemplateList[i]->templateId());
				break;
			}
		}


		_algoTemplateList.deleteAlgoTemplate(ui.lineEditTemplateName->text());

		updateTemplateLibraryTableWidget();

		if (ui.tableWidget_TemplateLibrary->rowCount() > 0)
		{
			ui.tableWidget_TemplateLibrary->setCurrentCell(ui.tableWidget_TemplateLibrary->rowCount() - 1, 0);
			tableWidgetTemplateLibCellClicked(ui.tableWidget_TemplateLibrary->rowCount() - 1, 0);
		}

		backupTemplateList();
		saveTemplateList();
	}
}

void TemplateLibraryTab::duplicateTemplate()
{
	QString inputName = ui.lineEditTemplateName->text();
	bool ok;
	QString templateName = QInputDialog::getText(nullptr, "Duplicate Template Name Input",
		"Template name:", QLineEdit::Normal,
		inputName, &ok);

	if (templateNameExist(templateName) || !ok)
	{
		QString templateNameExistedErrorLog = "Unable to Duplciate Template with Name: " + templateName + " because it is already existed!!!";
		QMessageBox::warning(this, tr("Template Name Existed!!!"),
			templateNameExistedErrorLog);
		return;
	}
	else
	{
		_algoTemplateList.addAlgoTemplate(templateName, getUnusedColor());
		_algoTemplateList.copyAlgoTemplate(ui.lineEditTemplateName->text(), templateName);
		
		updateTemplateLibraryTableWidget();
		saveTemplateList();

		selectTemplateTableRow(templateName);
	}


}

void TemplateLibraryTab::renameTemplate()
{
}

void TemplateLibraryTab::generateVIDIImages(bool enablePreprocess)
{
	auto algoTemplateList = _algoTemplateList.getAlgoTemplateList();
	AlgoTemplate* algoTemplate;

	for (int i = 0; i < algoTemplateList.size(); i++)
	{
		if (algoTemplateList[i]->templateName() == ui.lineEditTemplateName->text())
		{
			algoTemplate = algoTemplateList[i];
		}
	}
	emit generateVIDIImages(algoTemplate, enablePreprocess);
}

void TemplateLibraryTab::addPadding()
{
	bool ok;
	int paddingSize = QInputDialog::getInt(this, tr("Padding"), tr("Padding Size:"), 0, -1000000, 1000000, 1, &ok, Qt::CoverWindow);

	if (ok)
	{
		qDebug() << "paddingSize:" << paddingSize;
		auto algoTemplateList = _algoTemplateList.getAlgoTemplateList();
		AlgoTemplate* algoTemplate;

		for (int i = 0; i < algoTemplateList.size(); i++)
		{
			if (algoTemplateList[i]->templateName() == ui.lineEditTemplateName->text())
			{
				algoTemplate = algoTemplateList[i];
			}
		}

		emit addVisionObjectPadding(algoTemplate, paddingSize);
	}
}

void TemplateLibraryTab::resizeTemplate()
{
	bool ok;
	int widthSize = QInputDialog::getInt(this, tr("Width"), tr("Width Size:"), 0, 0, 1000000, 1, &ok, Qt::CoverWindow);
	int heightSize = QInputDialog::getInt(this, tr("Height"), tr("Height Size:"), 0, 0, 1000000, 1, &ok, Qt::CoverWindow);

	if (ok)
	{
		qDebug() << "width:" << widthSize << " height:" << heightSize;
		auto algoTemplateList = _algoTemplateList.getAlgoTemplateList();
		AlgoTemplate* algoTemplate;

		for (int i = 0; i < algoTemplateList.size(); i++)
		{
			if (algoTemplateList[i]->templateName() == ui.lineEditTemplateName->text())
			{
				algoTemplate = algoTemplateList[i];
			}
		}

		algoTemplate->w(widthSize);
		algoTemplate->h(heightSize);

		QString imageFilePath = algoTemplate->templateImagePath();
		imageFilePath = Common::Directory::getRecipeCurrentPath() + "template_Images\\" + algoTemplate->templateImagePath();
		if (algoTemplate->templateImagePath().contains("c:\\Advanced\\Data\\recipe")) imageFilePath = algoTemplate->templateImagePath();
		QImage templateImg;
		if (templateImg.load(imageFilePath))
		{
			if (templateImg.size().width() < algoTemplate->w() && templateImg.size().height() < algoTemplate->h())
			{
				QImage paddedImage(algoTemplate->w(), algoTemplate->h(), QImage::Format_RGB32);
				paddedImage.fill(Qt::black); // Fill the image with black color

				QPainter painter(&paddedImage);
				painter.drawImage(QPoint(0, 0), templateImg);
				painter.end();
				paddedImage.save(imageFilePath);	
			}
			else
			{
				templateImg = templateImg.copy(QRect(0, 0, algoTemplate->w(), algoTemplate->h()));
				templateImg.save(imageFilePath);
			}
			setTemplateImagePath(algoTemplate->templateId(), imageFilePath, QSize(algoTemplate->w(), algoTemplate->h()));
		}

		emit updateVisionObjectSize(algoTemplate);
	}
}

void TemplateLibraryTab::saveRefImage()
{
	auto algoTemplateList = _algoTemplateList.getAlgoTemplateList();
	AlgoTemplate* algoTemplate;

	for (int i = 0; i < algoTemplateList.size(); i++)
	{
		if (algoTemplateList[i]->templateName() == ui.lineEditTemplateName->text())
		{
			algoTemplate = algoTemplateList[i];
		}
	}
	emit saveTemplateReferenceImage(algoTemplate);
}

void TemplateLibraryTab::updateUniformBoxFlag()
{
	auto algoTemplateList = _algoTemplateList.getAlgoTemplateList();
	AlgoTemplate* algoTemplate;

	for (int i = 0; i < algoTemplateList.size(); i++)
	{
		if (algoTemplateList[i]->templateName() == ui.lineEditTemplateName->text())
		{
			algoTemplate = algoTemplateList[i];
			algoTemplateList[i]->uniformBox(ui.checkBoxUniformBox->isChecked());
		}
	}

	saveTemplateList();

	if (ui.checkBoxUniformBox->isChecked()) emit updateVisionObjectSize(algoTemplate);
}

void TemplateLibraryTab::updateUniformBoxFlag(bool flag)
{
	ui.checkBoxUniformBox->setChecked(flag);
	updateUniformBoxFlag();
}

void TemplateLibraryTab::backupTemplateList()
{
	QString templateListPath = Common::Directory::getRecipeCurrentPath() + "templateList.json";
	QString backupTemplateListFolder = Common::Directory::getRecipeCurrentPath() + "TemplateListBackup\\";
	Common::Directory::createDir(backupTemplateListFolder);

	uidGenerator uidGen;
	QString backupTemplateListPath = backupTemplateListFolder + "templateList_" + QString(uidGen.id().c_str()) + ".json";
	if (QFileInfo::exists(templateListPath)) MoveFileA(templateListPath.toStdString().c_str(), backupTemplateListPath.toStdString().c_str());
	ct::logger::debug("backupTemplateListPath: %s", backupTemplateListPath);
}

// ---- helpers for auditing what changed inside a template JSON ----
static QString auditJsonScalar(const QJsonValue& v)
{
	if (v.isObject() || v.isArray()) return QStringLiteral("[...]");
	if (v.isBool()) return v.toBool() ? QStringLiteral("true") : QStringLiteral("false");
	if (v.isNull()) return QStringLiteral("null");
	return v.toVariant().toString();
}

// Recursively compares two JSON values and appends "path: old -> new" entries.
static void auditJsonDiff(const QString& path, const QJsonValue& a, const QJsonValue& b, QStringList& out, int maxItems)
{
	if (out.size() >= maxItems) return;

	if (a.isObject() && b.isObject())
	{
		QJsonObject ao = a.toObject(), bo = b.toObject();
		QStringList keys = ao.keys();
		for (const QString& k : bo.keys()) if (!keys.contains(k)) keys << k;
		for (const QString& k : keys)
		{
			if (out.size() >= maxItems) return;
			QString p = path.isEmpty() ? k : path + "/" + k;
			if (!ao.contains(k))      out << QStringLiteral("%1 (added)").arg(p);
			else if (!bo.contains(k)) out << QStringLiteral("%1 (removed)").arg(p);
			else                      auditJsonDiff(p, ao.value(k), bo.value(k), out, maxItems);
		}
		return;
	}
	if (a.isArray() && b.isArray())
	{
		QJsonArray aa = a.toArray(), ba = b.toArray();
		int n = qMax(aa.size(), ba.size());
		for (int i = 0; i < n; ++i)
		{
			if (out.size() >= maxItems) return;
			QString p = QStringLiteral("%1[%2]").arg(path).arg(i);
			if (i >= aa.size())      out << QStringLiteral("%1 (added)").arg(p);
			else if (i >= ba.size()) out << QStringLiteral("%1 (removed)").arg(p);
			else                     auditJsonDiff(p, aa.at(i), ba.at(i), out, maxItems);
		}
		return;
	}
	if (a != b)
		out << QStringLiteral("%1: %2 -> %3").arg(path, auditJsonScalar(a), auditJsonScalar(b));
}

static QJsonValue auditReadJson(const QString& path)
{
	QFile f(path);
	if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) return QJsonValue();
	QJsonDocument d = QJsonDocument::fromJson(f.readAll());
	f.close();
	if (d.isArray()) return QJsonValue(d.array());
	return QJsonValue(d.object());
}

void TemplateLibraryTab::saveTemplateList()
{
	QString algoTemplateListFileName = Common::Directory::LocalPath + QString("recipe/%1/templateList.json").arg(Common::Directory::CurrentRecipe);

	// capture before/after so we can log which fields inside the template changed
	QJsonValue before = auditReadJson(algoTemplateListFileName);
	bool existedBefore = !before.isNull() && (before.isObject() ? !before.toObject().isEmpty() : true);

	_algoTemplateList.saveAlgoTemplateList(algoTemplateListFileName);

	QJsonValue after = auditReadJson(algoTemplateListFileName);

	const int cap = 15;
	QStringList changes;
	auditJsonDiff(QString(), before, after, changes, cap);

	QString detail;
	if (!existedBefore)          detail = QStringLiteral("templateList.json created");
	else if (changes.isEmpty())  detail = QStringLiteral("no change");
	else if (changes.size() >= cap) detail = QStringLiteral("%1+ changes: %2, ...").arg(cap).arg(changes.join(", "));
	else                         detail = changes.join(", ");

	AuditLog::instance().log(QStringLiteral("TEMPLATE_SAVE"), detail);
}


void TemplateLibraryTab::loadTemplateList()
{
	QString algoTemplateListFileName = Common::Directory::LocalPath + QString("recipe/%1/templateList.json").arg(Common::Directory::CurrentRecipe);
	_algoTemplateList.loadAlgoTemplateList(algoTemplateListFileName);
	//algo node-graph masks removed with the Algo library

	updateTemplateLibraryTableWidget();

	if (ui.tableWidget_TemplateLibrary->rowCount() > 0)
	{
		ui.tableWidget_TemplateLibrary->setCurrentCell(0,0);
		tableWidgetTemplateLibCellClicked(0, 0);
	}
	
}

void TemplateLibraryTab::tableWidgetTemplateLibCellClicked(int row, int column)
{
	QColor color;
	QString templateId;
	QString templateName = ui.tableWidget_TemplateLibrary->item(row, 0)->text();;
	QString templateImagePath;
	QSize size;
	AlgoTemplate* algoTemplate;
	ui.lineEditTemplateName->setText(templateName);
	auto algoTemplateList = _algoTemplateList.getAlgoTemplateList();
	for (int i = 0; i < algoTemplateList.size(); i++)
	{
		algoTemplate = algoTemplateList[i];
		if (algoTemplate->templateName() == ui.lineEditTemplateName->text())
		{
			templateId = algoTemplate->templateId();
			templateImagePath = Common::Directory::getRecipeCurrentPath() + "template_Images\\" + algoTemplate->templateImagePath();
			if (algoTemplate->templateImagePath().contains("c:\\Advanced\\Data\\recipe")) templateImagePath = algoTemplate->templateImagePath();
			size.setWidth(algoTemplate->w());
			size.setHeight(algoTemplate->h());
			QString textColor = "black";
			color = QColor(algoTemplate->color());
			if (color.red() < 230 && color.green() < 230 && color.blue() < 230)
			{
				textColor = "white";
			}
			ui.lineEditTemplateName->setStyleSheet(QString("QLineEdit { color: %1; background-color: %2; }").arg(textColor).arg(algoTemplate->color()));
			ui.checkBoxUniformBox->setChecked(algoTemplate->uniformBox());
			{
				QSignalBlocker sb(ui.comboBox_templateAlgo);
				ui.comboBox_templateAlgo->setCurrentIndex((int)algoTemplate->algo());
			}
			break;		
		}	
	}
	
	//Change Vision Object template
	if (column == 1)
	{
		ui.tableWidget_TemplateLibrary->item(row, 1)->setSelected(false);
		qDebug() << "checkTemplateImagePath:" << templateImagePath;
		QFileInfo fileInfo(templateImagePath);
		if (!QFileInfo::exists(templateImagePath) || !fileInfo.isFile())
		{
			QString error = "Unable to attach template to Vision Object because Default Vision Object has not been set!";
			emit showMsg(error);
			return;
		}
		emit updateVisionObjectTemplate(algoTemplate);
	}
}

void TemplateLibraryTab::customContextMenuRequested(QPoint point)
{
	QModelIndex index = ui.tableWidget_TemplateLibrary->indexAt(point);
	auto item = ui.tableWidget_TemplateLibrary->itemAt(point);

	if (item == nullptr) return;

	tableWidgetTemplateLibCellClicked(item->row(), item->column());

	if (index.isValid()) {
		QMenu actionMenu;
		actionMenu.addAction("Change Template Color", this, SLOT(changeTemplateColor()));
		actionMenu.addAction("Delete Template", this, SLOT(deleteTemplate()));
		actionMenu.addAction("Duplicate Template", this, SLOT(duplicateTemplate()));
		//actionMenu.addAction("Rename Template", this, SLOT(renameTemplate()));
		actionMenu.exec(ui.tableWidget_TemplateLibrary->viewport()->mapToGlobal(point));
	}
}

void TemplateLibraryTab::changeTemplateColor()
{
	QItemSelectionModel *select = ui.tableWidget_TemplateLibrary->selectionModel();
	QModelIndexList selectedRows = select->selectedIndexes();

	for (int i = 0; i < selectedRows.size(); i++)
	{
		int row = selectedRows[i].row();
		QTableWidgetItem *item = ui.tableWidget_TemplateLibrary->item(row, 1);
		if (item)
		{
			QColor selectedClassColor = item->backgroundColor();
			QColor selectedColorCode = selectedClassColor;
			QColor color = QColorDialog::getColor(selectedColorCode, this);

			if (color.isValid())
			{
				item = ui.tableWidget_TemplateLibrary->item(row, 0);
				QString templateName = item->text();

				if (!checkColourUsed(color))
				{
					QString colorCode = color.name();
					ui.tableWidget_TemplateLibrary->setItem(row, 1, new QTableWidgetItem(""));
					ui.tableWidget_TemplateLibrary->item(row, 1)->setBackground(color);
					updateTemplateListSettings(colorCode, templateName);
					saveTemplateList();
					emit ui.tableWidget_TemplateLibrary->cellClicked(row, 1);
				}
			}
		}
	
	}
}

void TemplateLibraryTab::updateTemplateListSettings(QString color, QString templateName)
{
	auto algoTemplateList = _algoTemplateList.getAlgoTemplateList();

	for (int i = 0; i < algoTemplateList.size(); i++)
	{
		if (algoTemplateList[i]->templateName() == templateName)
		{
			algoTemplateList[i]->color(color);
			emit updateVisionObjectColor(algoTemplateList[i]);
		}
	}

	return;
}

void TemplateLibraryTab::selectTemplateTableRow(const QString & templateName)
{
	if (!templateName.isEmpty())
	{
		ui.tableWidget_TemplateLibrary->clearSelection();
		for (int i = 0; i < ui.tableWidget_TemplateLibrary->rowCount(); i++)
		{
			if (ui.tableWidget_TemplateLibrary->item(i, 0)->text() == templateName)
			{
				ui.tableWidget_TemplateLibrary->selectRow(i);
				ui.tableWidget_TemplateLibrary->item(i, 1)->setSelected(false);
				tableWidgetTemplateLibCellClicked(i, 0);
			}
		}

	}
}

void TemplateLibraryTab::loadAlgoTemplateListMask()
{
	//algo node-graph masks removed with the Algo library
}

void TemplateLibraryTab::reloadAlgoTemplateListMetaData()
{
	//algo node-graph metadata removed with the Algo library
}

void TemplateLibraryTab::setTagNameCount(QHash<QString, int> tagNameCountHash)
{
	qDebug() << "setTagNameCount";
	_tagNameCountHash.clear();
	_tagNameCountHash = tagNameCountHash;

	qDebug() << "tag name count size: " << _tagNameCountHash.size();
}

void TemplateLibraryTab::displayTagNameCount()
{
	//ui.textEdit_goldenRecipeTagName->clear(); TODO: WC

	QString header = "<b><font color = 'green'>Setup Golden recipe success!</font> </b> <br> <b><font color = 'white'>Golden Run Result: </font> </b> <br>";
	//ui.textEdit_goldenRecipeTagName->append(header); TODO: WC

	for (int i=0; i< _tagNameCountHash.size(); i++)
	{
		QString tagName = _tagNameCountHash.keys()[i];
		QString count = QString::number(_tagNameCountHash[tagName]);

		QString text = "<font color = 'white'>" + tagName + " : " + count + "</font>";

		//ui.textEdit_goldenRecipeTagName->append(text); TODO: WC
	}


}

QStringList TemplateLibraryTab::getAllTemplateID()
{
	auto algoTemplateList = _algoTemplateList.getAlgoTemplateList();

	QStringList templateIdList;
	for (auto t: algoTemplateList)
	{
		templateIdList.append(t->templateId());
	}
	return templateIdList;
}

void TemplateLibraryTab::releaseAlgoTemplates()
{
	_algoTemplateList.releaseAlgoTemplateList();
}


#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QFileDialog>
#include <QMessageBox>
#include <QStandardPaths>
#include <QDateTime>
#include <QDirIterator>

void TemplateLibraryTab::saveGoldenTemplateList()
{
	// ensure latest templateList.json written to recipe
	saveTemplateList();

	const QString recipePath = Common::Directory::getRecipeCurrentPath(); 

	// folders to bundle
	const QStringList folderNames = {
		"MaskImages",
		"template_Images",
		"bondInfos",
		"CadRois",
		"HeightMeasurement",
		"ObjectDetection",
		"OcrLibraries",      
		"PadRois",
		"PatternLocator"
	};

	// files to bundle
	const QStringList fileNames = {
		"templateList.json",
		"componentCadTypeLibrary.json",
		"OcrLibraries.json",
		"ODModelList.json"
	};



	// create bundle folder
	const QString ts = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
	const QString recipeName = Common::Directory::CurrentRecipe;
	const QString bundleName = QString("GoldenTemplate_%1_%2").arg(recipeName, ts);
	const QString destParent = "C:\\Advanced\\TemplateCollector\\";
	const QString bundleRoot = QDir(destParent).filePath(bundleName);
	QDir().mkpath(destParent);
	QDir().mkpath(bundleRoot);

	if (!QDir().mkpath(bundleRoot)) {
		QMessageBox::warning(this, tr("Export Failed"),
			tr("Failed to create folder:\n%1").arg(bundleRoot));
		return;
	}

	// copy dir recursively (overwrite)
	auto copyDir = [&](const QString& srcDirPath, const QString& dstDirPath) -> bool {
		QDir srcDir(srcDirPath);
		if (!srcDir.exists()) return true; // missing -> skip

		if (!QDir().mkpath(dstDirPath)) return false;

		QDirIterator it(srcDirPath, QDir::NoDotAndDotDot | QDir::AllEntries, QDirIterator::Subdirectories);
		while (it.hasNext()) {
			const QString srcPath = it.next();
			const QFileInfo fi(srcPath);

			const QString relPath = srcDir.relativeFilePath(srcPath);
			const QString dstPath = QDir(dstDirPath).filePath(relPath);

			if (fi.isDir()) {
				if (!QDir().mkpath(dstPath)) return false;
			}
			else if (fi.isFile()) {
				QDir().mkpath(QFileInfo(dstPath).absolutePath());
				if (QFileInfo::exists(dstPath)) QFile::remove(dstPath);
				if (!QFile::copy(srcPath, dstPath)) return false;
			}
		}
		return true;
		};

	// copy single file (overwrite). if required and missing -> fail
	auto copyFile = [&](const QString& srcFile, const QString& dstFile, bool required) -> bool {
		if (!QFileInfo::exists(srcFile)) return !required;
		QDir().mkpath(QFileInfo(dstFile).absolutePath());
		if (QFileInfo::exists(dstFile)) QFile::remove(dstFile);
		return QFile::copy(srcFile, dstFile);
		};

	// 1) files
	for (const QString& fn : fileNames) {
		const QString src = QDir(recipePath).filePath(fn);
		const QString dst = QDir(bundleRoot).filePath(fn);
		const bool required = (fn.compare("templateList.json", Qt::CaseInsensitive) == 0);

		if (!copyFile(src, dst, required)) {
			QMessageBox::warning(this, tr("Export Failed"),
				tr("Failed to export file:\n%1").arg(fn));
			return;
		}
	}

	// 2) folders
	for (const QString& dirName : folderNames) {
		const QString srcDir = QDir(recipePath).filePath(dirName);
		const QString dstDir = QDir(bundleRoot).filePath(dirName);

		if (!copyDir(srcDir, dstDir)) {
			QMessageBox::warning(this, tr("Export Failed"),
				tr("Failed to export folder:\n%1").arg(dirName));
			return;
		}
	}

	QMessageBox::information(this, tr("Export Success"),
		tr("Golden bundle exported to:\n%1").arg(bundleRoot));
}

void TemplateLibraryTab::loadGoldenTemplateList()
{

	const QString recipePath = Common::Directory::getRecipeCurrentPath();

	const QStringList folderNames = {
		"MaskImages",
		"template_Images",
		"bondInfos",
		"CadRois",
		"HeightMeasurement",
		"ObjectDetection",
		"OcrLibraries",      
		"PadRois",
		"PatternLocator"
	};

	const QStringList fileNames = {
		"templateList.json",
		"componentCadTypeLibrary.json",
		"OcrLibraries.json",
		"ODModelList.json"
	};

	// templateList.json must exist in bundle
	const QString defaultDir = "C:\\Advanced\\TemplateCollector\\";
	const QString bundleRoot = QFileDialog::getExistingDirectory(
		this,
		tr("Select Golden Template Bundle Folder"),
		defaultDir
	);
	if (bundleRoot.isEmpty())
		return;
	if (!QFileInfo::exists(QDir(bundleRoot).filePath("templateList.json"))) {
		QMessageBox::warning(this, tr("Import Failed"),
			tr("templateList.json not found in:\n%1").arg(bundleRoot));
		return;
	}

	// remove dir recursively
	auto removeDir = [&](const QString& dirPath) -> bool {
		QDir dir(dirPath);
		if (!dir.exists()) return true;
		return dir.removeRecursively();
		};

	// copy dir recursively (overwrite)
	auto copyDir = [&](const QString& srcDirPath, const QString& dstDirPath) -> bool {
		QDir srcDir(srcDirPath);
		if (!srcDir.exists()) return true; // missing in bundle -> skip

		if (!QDir().mkpath(dstDirPath)) return false;

		QDirIterator it(srcDirPath, QDir::NoDotAndDotDot | QDir::AllEntries, QDirIterator::Subdirectories);
		while (it.hasNext()) {
			const QString srcPath = it.next();
			const QFileInfo fi(srcPath);

			const QString relPath = srcDir.relativeFilePath(srcPath);
			const QString dstPath = QDir(dstDirPath).filePath(relPath);

			if (fi.isDir()) {
				if (!QDir().mkpath(dstPath)) return false;
			}
			else if (fi.isFile()) {
				QDir().mkpath(QFileInfo(dstPath).absolutePath());
				if (QFileInfo::exists(dstPath)) QFile::remove(dstPath);
				if (!QFile::copy(srcPath, dstPath)) return false;
			}
		}
		return true;
		};

	auto copyFile = [&](const QString& srcFile, const QString& dstFile, bool required) -> bool {
		if (!QFileInfo::exists(srcFile)) return !required;
		QDir().mkpath(QFileInfo(dstFile).absolutePath());
		if (QFileInfo::exists(dstFile)) QFile::remove(dstFile);
		return QFile::copy(srcFile, dstFile);
		};

	// --- backup current recipe before overwrite ---
	const QString ts = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
	const QString backupRoot = QDir(recipePath).filePath(QString("GoldenImportBackup_%1").arg(ts));

	if (!QDir().mkpath(backupRoot)) {
		QMessageBox::warning(this, tr("Import Failed"),
			tr("Failed to create backup folder:\n%1").arg(backupRoot));
		return;
	}

	// backup files 
	for (const QString& fn : fileNames) {
		const QString cur = QDir(recipePath).filePath(fn);
		const QString bak = QDir(backupRoot).filePath(fn);
		if (QFileInfo::exists(cur)) {
			QFile::remove(bak);
			QFile::copy(cur, bak);
		}
	}
	for (const QString& dirName : folderNames) {
		const QString curDir = QDir(recipePath).filePath(dirName);
		const QString bakDir = QDir(backupRoot).filePath(dirName);
		copyDir(curDir, bakDir);
	}

	// --- replace files from bundle ---
	for (const QString& fn : fileNames) {
		const QString src = QDir(bundleRoot).filePath(fn);
		const QString dst = QDir(recipePath).filePath(fn);

		const bool required = (fn.compare("templateList.json", Qt::CaseInsensitive) == 0);

		if (QFileInfo::exists(dst)) QFile::remove(dst);

		if (!copyFile(src, dst, required)) {
			QMessageBox::warning(this, tr("Import Failed"),
				tr("Failed to import file:\n%1\n\nBackup saved at:\n%2").arg(fn, backupRoot));
			return;
		}
	}

	// --- replace folders from bundle (only if present in bundle) ---
	for (const QString& dirName : folderNames) {
		const QString srcDir = QDir(bundleRoot).filePath(dirName);
		const QString dstDir = QDir(recipePath).filePath(dirName);

		if (!QDir(srcDir).exists())
			continue;

		if (!removeDir(dstDir)) {
			QMessageBox::warning(this, tr("Import Failed"),
				tr("Failed to clear existing folder:\n%1\n\nBackup saved at:\n%2").arg(dstDir, backupRoot));
			return;
		}

		if (!copyDir(srcDir, dstDir)) {
			QMessageBox::warning(this, tr("Import Failed"),
				tr("Failed to import folder:\n%1\n\nBackup saved at:\n%2").arg(dirName, backupRoot));
			return;
		}
	}

	// refresh UI + internal list
	loadTemplateList();

	QMessageBox::information(this, tr("Import Success"),
		tr("Golden bundle imported.\n\nBackup saved at:\n%1").arg(backupRoot));
}