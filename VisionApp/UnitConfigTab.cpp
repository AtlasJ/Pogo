#include "UnitConfigTab.h"
#include "CommonDir.h"
#include "Logger.h"
#include "AuditLog.h"



UnitConfigTab::UnitConfigTab(QWidget *parent)
	: QWidget(parent)
{
	ui.setupUi(this);

	QStringList mode;
	mode << "Index" << "Row_Column";

	ui.comboBox_viewMode->addItems(mode);
	ui.comboBox_unitMode->addItems(mode);

	connect(ui.comboBox_viewID, SIGNAL(currentIndexChanged(int)), this, SLOT(showCurrentViewUnitConfigSettings()));

	connect(ui.comboBox_viewMode, SIGNAL(currentIndexChanged(int)), this, SLOT(toggleUnitConfigSettings()));
	connect(ui.comboBox_unitMode, SIGNAL(currentIndexChanged(int)), this, SLOT(toggleUnitConfigSettings()));
	connect(ui.lineEdit_totalViewIndex, &QLineEdit::textEdited, this, [=](const QString& text) {updateUnitConfigSettings();});
	connect(ui.lineEdit_totalViewRow, &QLineEdit::textEdited, this, [=](const QString& text) {updateUnitConfigSettings(); });
	connect(ui.lineEdit_totalViewColumn, &QLineEdit::textEdited, this, [=](const QString& text) {updateUnitConfigSettings(); });
	connect(ui.lineEdit_totalUnitIndex, &QLineEdit::textEdited, this, [=](const QString& text) {updateUnitConfigSettings(); });
	connect(ui.lineEdit_totalUnitRow, &QLineEdit::textEdited, this, [=](const QString& text) {updateUnitConfigSettings(); });
	connect(ui.lineEdit_totalUnitColumn, &QLineEdit::textEdited, this, [=](const QString& text) {updateUnitConfigSettings(); });
	connect(ui.toolButton_runStoredUnits,&QToolButton::pressed,this,&UnitConfigTab::runStoredUnits);
	connect(ui.toolButton_storeSkippeddUnits, &QToolButton::pressed, this, &UnitConfigTab::storeSkippedUnits);
	connect(ui.toolButton_removeSkippedUnits, &QToolButton::pressed, this, &UnitConfigTab::removeSkippedUnits);

	
}

UnitConfigTab::~UnitConfigTab()
{}

void UnitConfigTab::loadUnitConfig(QHash <QString, QView>& views)
{
	_unitConfigInfos.clear();
	QString unitConfigFilePath = Common::Directory::getRecipeCurrentPath() + "unitConfig.json";

	bool validUnitConfigFile = false;
	//check if file exist, if file do not exist, initialize a new UnitConfig.json using QViews
	if (QFile::exists(unitConfigFilePath))
	{
		QJsonObject root;
		if (loadJson(unitConfigFilePath, root)) {
			auto unitConfigArray = jsonHelper::getArray(root, QStringLiteral("unit_config"));

			for (int i = 0; i < unitConfigArray.size(); i++)
			{
				auto unitConfigObj = unitConfigArray[i].toObject();
				UnitConfigInfo info;
				info.viewName = jsonHelper::getString(unitConfigObj, QStringLiteral("viewName"));
				info.viewId = jsonHelper::getString(unitConfigObj, QStringLiteral("viewId"));

				info.viewMode = jsonHelper::getInteger(unitConfigObj, QStringLiteral("viewMode"));
				info.totalViewIndex = jsonHelper::getInteger(unitConfigObj, QStringLiteral("totalViewIndex"));
				info.totalViewRow = jsonHelper::getInteger(unitConfigObj, QStringLiteral("totalViewRow"));
				info.totalViewCol = jsonHelper::getInteger(unitConfigObj, QStringLiteral("totalViewCol"));

				info.unitMode = jsonHelper::getInteger(unitConfigObj, QStringLiteral("unitMode"));
				info.totalUnitIndex = jsonHelper::getInteger(unitConfigObj, QStringLiteral("totalUnitIndex"));
				info.totalUnitRow = jsonHelper::getInteger(unitConfigObj, QStringLiteral("totalUnitRow"));
				info.totalUnitCol = jsonHelper::getInteger(unitConfigObj, QStringLiteral("totalUnitCol"));

				_unitConfigInfos.append(info);
				validUnitConfigFile = true;
			}
		}
	}
	else
	{	
		for (auto v : views)
		{
			UnitConfigInfo info;
			info.viewName = v.name;
			info.viewId = v.id;
		
			_unitConfigInfos.append(info);
		}

		saveUnitConfig();
		validUnitConfigFile = true;
	}

	if (validUnitConfigFile)
	{
		if (_unitConfigInfos.size() == views.size())
		{
			for (auto& info : _unitConfigInfos)
			{
				bool viewExist = false;
				for (auto v : views)
				{
					if (info.viewId == v.id)
					{
						viewExist = true;
						break;
					}
				}

				if (!viewExist) validUnitConfigFile = false;
			}
				
		}
		else validUnitConfigFile = false;
	}
	//check if unitConfig view match with QViews

	//if not the same than prompt warning error and ask to validate if the correct format recipe is used
	if (!validUnitConfigFile)
	{
		ct::logger::warn("Unit Config File.json does not match with the recipe, please Check!!!");
	}
	else
	{
		ui.comboBox_viewID->clear();
		for (auto info : _unitConfigInfos)
		{
			ui.comboBox_viewID->addItem(info.viewName, info.viewId);
		}
	}

	toggleUnitConfigSettings();
}

void UnitConfigTab::saveUnitConfig()
{
	AuditLog::instance().log(QStringLiteral("UNIT_CONFIG_SAVE"), Common::Directory::CurrentRecipe);
	QString unitConfigFilePath = Common::Directory::getRecipeCurrentPath() + "unitConfig.json";

	QJsonObject j_main;
	QJsonArray j_array;

	for (auto& info : _unitConfigInfos)
	{
		QJsonObject j_root;
		j_root.insert(QStringLiteral("viewName"), info.viewName);
		j_root.insert(QStringLiteral("viewId"), info.viewId);

		j_root.insert(QStringLiteral("viewMode"), info.viewMode);
		j_root.insert(QStringLiteral("totalViewIndex"), info.totalViewIndex);
		j_root.insert(QStringLiteral("totalViewRow"), info.totalViewRow);
		j_root.insert(QStringLiteral("totalViewCol"), info.totalViewCol);

		j_root.insert(QStringLiteral("unitMode"), info.unitMode);
		j_root.insert(QStringLiteral("totalUnitIndex"), info.totalUnitIndex);
		j_root.insert(QStringLiteral("totalUnitRow"), info.totalUnitRow);
		j_root.insert(QStringLiteral("totalUnitCol"), info.totalUnitCol);

		j_array.push_back(j_root);
	}

	j_main.insert(QStringLiteral("unit_config"), j_array);

	saveJson(unitConfigFilePath, QJsonDocument(j_main));
}

QVector<UnitConfigInfo> UnitConfigTab::getUnifConfigInfos()
{
	return _unitConfigInfos;
}

QString UnitConfigTab::getFirstID(QString viewID)
{
	for (auto& info : _unitConfigInfos)
	{
		if (info.viewId == viewID)
		{
			if (info.viewMode == (int)(UnitMode::INDEX))
			{
				return QString::number(1);
			}
			else
			{
				QString row_col_ID = "R0C0";
				return row_col_ID;
			}
		}
	}
	return QString();
}

QString UnitConfigTab::getNextID(QString currentID, QString viewID)
{
	for (auto& info : _unitConfigInfos)
	{
		if (info.viewId == viewID)
		{
			if (info.viewMode == (int)(UnitMode::INDEX))
			{
				int currentIdx = currentID.toInt();
				if(currentIdx < (info.totalViewIndex)) currentIdx++;
				return QString::number(currentIdx);
			}
			else
			{
				int row = 0;
				int col = 0;
				util::getRowColFromID(currentID, row, col);

				if (row < info.totalUnitRow)
				{
					row++;
					return util::getRowColID(row, col);
				}
				else
				{
					if (col < info.totalUnitCol)
					{
						col++;
						row = 1;
						return util::getRowColID(row, col);
					}
				}
				return currentID;
			}
		}
	}
	return QString();
}

QString UnitConfigTab::getPreviousID(QString currentID, QString viewID)
{
	for (auto& info : _unitConfigInfos)
	{
		if (info.viewId == viewID)
		{
			if (info.viewMode == (int)(UnitMode::INDEX))
			{				
				int currentIdx = currentID.toInt();
				if (currentIdx > 1) currentIdx--;
				return QString::number(currentIdx);				
			}
			else
			{
				int row = 0;
				int col = 0;
				util::getRowColFromID(currentID, row, col);

				if (row > 0)
				{
					row--;
					return util::getRowColID(row, col);
				}
				else
				{
					if (col > 0)
					{
						col--;
						row = info.totalUnitRow;
						return util::getRowColID(row, col);
					}
				}
				return currentID;
			}
		}
	}
	return QString();
}

int UnitConfigTab::getTotalIndex(QString viewID)
{
	int totalCount = 0;
	for (auto& info : _unitConfigInfos)
	{
		if (info.viewId == viewID)
		{
			if (info.viewMode == (int)(UnitMode::INDEX))
			{
				totalCount = info.totalViewIndex;		
			}
			else
			{
				totalCount = info.totalViewCol * info.totalViewRow;
			}
		}
	}
	return totalCount;
}

QStringList UnitConfigTab::getIDList(QString viewID)
{
	QStringList IDList;
	for (auto& info : _unitConfigInfos)
	{
		if (info.viewId == viewID)
		{
			if (info.viewMode == (int)(UnitMode::INDEX))
			{
				for (int i = 0; i < info.totalViewIndex; i++)
				{
					IDList.append(QString::number(i + 1));
				}
			}
			else
			{
				for (int r = 0; r < info.totalUnitRow; r++)
				{
					for (int c = 0; c < info.totalViewCol; c++)
					{
						int row = r;
						int col = c;
						IDList.append(util::getRowColID(row, col));
					}
				}
				
			}
		}
	}
	return IDList;
}

bool UnitConfigTab::loadJson(QString path, QJsonObject& root)
{
	QString val;
	QFile file;
	QJsonDocument doc;

	if (QFile::exists(path))
	{
		file.setFileName(path);
		if (file.open(QIODevice::ReadOnly | QIODevice::Text))
		{
			val = file.readAll();
			file.close();

			doc = QJsonDocument::fromJson(val.toUtf8());
			root = doc.object();

			return true;
		}
	}

	return false;
}

bool UnitConfigTab::loadJson(QString path, QJsonDocument& doc)
{
	QString val;
	QFile file;

	if (QFile::exists(path))
	{
		file.setFileName(path);
		if (file.open(QIODevice::ReadOnly | QIODevice::Text))
		{
			val = file.readAll();
			file.close();

			doc = QJsonDocument::fromJson(val.toUtf8());

			return true;
		}
	}

	return false;
}

bool UnitConfigTab::saveJson(const QString& fileName, const QJsonDocument& doc)
{
	bool flag = false;

	QFile file(fileName);
	if (file.open(QIODevice::WriteOnly))
	{
		file.write(doc.toJson());
		file.flush();
		file.close();

		flag = true;
	}

	return flag;
}

void UnitConfigTab::showCurrentViewUnitConfigSettings()
{
	for (auto info : _unitConfigInfos)
	{
		if (info.viewId == ui.comboBox_viewID->currentData())
		{
			
			ui.lineEdit_totalViewIndex->setText(QString::number(info.totalViewIndex));
			ui.lineEdit_totalViewRow->setText(QString::number(info.totalViewRow));
			ui.lineEdit_totalViewColumn->setText(QString::number(info.totalViewCol));

			
			ui.lineEdit_totalUnitIndex->setText(QString::number(info.totalUnitIndex));
			ui.lineEdit_totalUnitRow->setText(QString::number(info.totalUnitRow));
			ui.lineEdit_totalUnitColumn->setText(QString::number(info.totalUnitCol));

			ui.comboBox_viewMode->setCurrentIndex(info.viewMode);
			ui.comboBox_unitMode->setCurrentIndex(info.unitMode);
			break;
		}
	}

	toggleUnitConfigSettings();
}

void UnitConfigTab::updateUnitConfigSettings()
{
	for (auto & info : _unitConfigInfos)
	{
		if (info.viewId == ui.comboBox_viewID->currentData())
		{
			info.viewMode = ui.comboBox_viewMode->currentIndex();
			info.totalViewIndex = ui.lineEdit_totalViewIndex->text().toInt();
			info.totalViewRow = ui.lineEdit_totalViewRow->text().toInt();
			info.totalViewCol = ui.lineEdit_totalViewColumn->text().toInt();

			info.unitMode = ui.comboBox_unitMode->currentIndex();
			info.totalUnitIndex = ui.lineEdit_totalUnitIndex->text().toInt();
			info.totalUnitRow = ui.lineEdit_totalUnitRow->text().toInt();
			info.totalUnitCol = ui.lineEdit_totalUnitColumn->text().toInt();
		}
	}

	saveUnitConfig();
}

void UnitConfigTab::toggleUnitConfigSettings()
{
	if (ui.comboBox_viewID->count() == 0)
	{
		ui.label_3->hide();
		ui.lineEdit_totalViewIndex->hide();
		ui.label_4->hide();
		ui.lineEdit_totalViewRow->hide();
		ui.label_5->hide();
		ui.lineEdit_totalViewColumn->hide();

		ui.label_7->hide();
		ui.lineEdit_totalUnitIndex->hide();
		ui.label_8->hide();
		ui.lineEdit_totalUnitRow->hide();
		ui.label_9->hide();
		ui.lineEdit_totalUnitColumn->hide();
	}	
	else if (ui.comboBox_viewMode->currentIndex() == 0)
	{
		ui.label_3->show();
		ui.lineEdit_totalViewIndex->show();
		ui.label_4->hide();
		ui.lineEdit_totalViewRow->hide();
		ui.label_5->hide();
		ui.lineEdit_totalViewColumn->hide();
	}
	else 
	{
		ui.label_3->hide();
		ui.lineEdit_totalViewIndex->hide();
		ui.label_4->show();
		ui.lineEdit_totalViewRow->show();
		ui.label_5->show();
		ui.lineEdit_totalViewColumn->show();
	}

	if (ui.comboBox_unitMode->currentIndex() == 0)
	{
		ui.label_7->show();
		ui.lineEdit_totalUnitIndex->show();
		ui.label_8->hide();
		ui.lineEdit_totalUnitRow->hide();
		ui.label_9->hide();
		ui.lineEdit_totalUnitColumn->hide();
	}
	else
	{
		ui.label_7->hide();
		ui.lineEdit_totalUnitIndex->hide();
		ui.label_8->show();
		ui.lineEdit_totalUnitRow->show();
		ui.label_9->show();
		ui.lineEdit_totalUnitColumn->show();
	}

	updateUnitConfigSettings();
}

