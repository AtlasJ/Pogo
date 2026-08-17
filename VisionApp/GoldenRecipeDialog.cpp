#include "GoldenRecipeDialog.h"

GoldenRecipeDialog::GoldenRecipeDialog(QWidget* parent)
	: QDialog(parent)
{
	ui.setupUi(this);

	connectSignalAndSlot();



}


void GoldenRecipeDialog::setOptics(QHash<QString, OpticsInfo> optics_2d, QHash<QString, OpticsInfo3D> optics_3d)
{
	qDebug() << "setOptics";

	_opticsHash.clear();
	for (auto& o : optics_2d)
	{
		_opticsHash.insert(o.id, o.name);
	}

}

void GoldenRecipeDialog::runGoldenRecipe(bool isProduction)
{
	_isRunningProduction = isProduction;
	QString refPath = Common::Directory::getRecipeCurrentPath() + "/GoldenRecipe.json";
	readGoldenRecipeInfoJson(refPath, _refTagNameHash, _refTagNameCountHash, _refRecipeName);

	emit signalRequestGoldenRecipeResult();

}

void GoldenRecipeDialog::connectSignalAndSlot()
{
	connect(ui.toolButton_test, &QToolButton::clicked, this, [&]() {
		runGoldenRecipe(false);
		});
	connect(ui.toolButton_setReference, &QToolButton::clicked, this, [&]() {



		setReferenceGoldenRecipe();
		});
}

bool GoldenRecipeDialog::checkGoldenUnitCriteria()
{
	qDebug() << "getNumberGoldenUnit";
	QDir dir(_goldenRecipeImageDir);
	QStringList nameFilters;
	nameFilters << "*.png" << "*.jpg" << "*.jpeg" << "*.bmp" << "*.tif" << "*.tiff";

	QStringList fileList = dir.entryList(nameFilters, QDir::Files | QDir::NoSymLinks);
	QHash <QString, QStringList> unitOptics; // key: unitName // value: optics
	for (int i = 0; i < fileList.size(); ++i)
	{
		QString imageName = fileList[i];



		if (imageName.split("_").size() > 1)
		{
			QString unitName = imageName.split("_")[0];
			QString opticName = imageName.split("_")[1];
			unitOptics[unitName].append(opticName);
		}
	}

	bool isImagePass = true;
	QStringList failedUnits;

	for (auto it = unitOptics.constBegin(); it != unitOptics.constEnd(); ++it)
	{
		const QString& unitName = it.key();
		const QStringList& opticList = it.value();

		qDebug() << "_opticsHash.size: " << _opticsHash.size();
		qDebug() << "opticList.size: " << opticList.size();

		if (opticList.size() != _opticsHash.size())
		{
			isImagePass = false;
			failedUnits.append(unitName);
		}
	}

	if (unitOptics.isEmpty())
	{
		isImagePass = false;
		ui.label_message->setText("No Golden unit image found!");
		ui.label_message->setStyleSheet("QLabel { color : red; }");
	}
	else if (!isImagePass)
	{
		QString failedUnit = failedUnits.join(",");
		ui.label_message->setText(failedUnit + " Golden unit having incomplete optic image!");
		ui.label_message->setStyleSheet("QLabel { color : red; }");

	}
	else
	{
		ui.label_message->setText(QString::number(unitOptics.size())+" Golden unit image found!");
		ui.label_message->setStyleSheet("QLabel { color : green; }");
	}


	

	return isImagePass;
}

void GoldenRecipeDialog::goldenRecipeRunComplete()
{
	QString testPath = Common::Directory::CachePath + "/goldenRecipeInfo.json";
	readGoldenRecipeInfoJson(testPath, _testTagNameHash, _testTagNameCountHash, _testTecipeName);
	bool isMatchFlag = true;
	QString message;
	if (_refTagNameHash.isEmpty() || _refTagNameCountHash.isEmpty())
	{
		isMatchFlag = false;
		message = "Reference Unit not found";
	}
	else
	{
		isMatchFlag = isMatch(message);
	}

	if (!_isRunningProduction)
	{
		refreshTestDefCountTableWidget();
		refreshTestUnitDefectTableWidget();
		ui.label_matchingResult->setText(message);
		if (isMatchFlag)
		{
			// Set the text color to green
			ui.label_matchingResult->setStyleSheet("QLabel { color : green; }");
		}
		else
		{
			// Set the text color to red
			ui.label_matchingResult->setStyleSheet("QLabel { color : red; }");
		}
	}
	else
	{
		emit signalRunGoldenRecipeComplete(isMatchFlag, message);
	}


}
void GoldenRecipeDialog::initRefGoldenRecipe()
{
	qDebug() << "initRefGoldenRecipe";
	_goldenRecipeImageDir = Common::Directory::getRecipeImagesPath() + "GoldenRecipeImages\\";
	QDir dir;
	dir.mkdir(_goldenRecipeImageDir);

	ui.label_imageDir->setText(_goldenRecipeImageDir);
	QString refPath = Common::Directory::getRecipeCurrentPath() + "/GoldenRecipe.json";
	readGoldenRecipeInfoJson(refPath, _refTagNameHash, _refTagNameCountHash, _refRecipeName);
	refreshRefDefCountTableWidget();
	refreshRefUnitDefectTableWidget();

	checkGoldenUnitCriteria();
}
void GoldenRecipeDialog::setReferenceGoldenRecipe()
{
	writeRefGoldenRecipe(Common::Directory::CurrentRecipe, _testTagNameHash, _testTagNameCountHash);

	initRefGoldenRecipe();
}

void GoldenRecipeDialog::readGoldenRecipeInfoJson(QString& path, QHash<QString, QString>& tagNameHash, QHash<QString, int>& tagNameCountHash, QString& recipeName)
{
	qDebug() << "readGoldenRecipeInfoJson";

	QString jsonPath = path;

	QHash<QString, QString> this_tagNameHash;
	QHash<QString, int> this_tagNameCountHash;

	QFileInfo f(jsonPath);


	QFile file(jsonPath);
	if (!file.open(QIODevice::ReadOnly)) {

		return;
	}
	QByteArray jsonData = file.readAll();

	QJsonDocument jsonDoc = QJsonDocument::fromJson(jsonData);
	QJsonObject jsonObj = jsonDoc.object();
	QJsonArray tNameArr = jsonObj.value("voTagName").toArray();
	QJsonArray tCountArr = jsonObj.value("tagNameCount").toArray();
	for (const QJsonValue& v : tNameArr)
	{
		QJsonObject voObj = v.toObject();
		QString voName = voObj.value("voName").toString();
		QString tagName = voObj.value("tagName").toString();

		this_tagNameHash.insert(voName, tagName);
	}

	for (const QJsonValue& v : tCountArr)
	{
		QJsonObject voObj = v.toObject();
		QString tagName = voObj.value("tagName").toString();
		int count = voObj.value("count").toInt();

		this_tagNameCountHash.insert(tagName, count);
	}

	QString this_recipeName = jsonObj.value("recipeName").toString();

	file.close();
	tagNameHash = this_tagNameHash;
	tagNameCountHash = this_tagNameCountHash;
	recipeName = this_recipeName;

}

void GoldenRecipeDialog::refreshTestDefCountTableWidget()
{
	qDebug() << "refreshDistanceMeasurementTableWidget";

	ui.tableWidget_testDefectCount->blockSignals(true);
	ui.tableWidget_testDefectCount->clear();
	ui.tableWidget_testDefectCount->setRowCount(0); // reset


	QStringList horizontalLabel = { "Defect Name" ,"Count" };


	int col = horizontalLabel.size();
	ui.tableWidget_testDefectCount->setColumnCount(col);
	int row = 0;

	for (int i = 0; i < _testTagNameCountHash.size(); i++)
	{
		row++;

		QString tagName = _testTagNameCountHash.keys()[i];
		QString count = QString::number(_testTagNameCountHash[tagName]);

		ui.tableWidget_testDefectCount->setRowCount(row);

		QTableWidgetItem* tItem = new QTableWidgetItem(tagName);
		ui.tableWidget_testDefectCount->setItem(row - 1, 0, tItem);
		ui.tableWidget_testDefectCount->item(row - 1, 0)->setFlags(ui.tableWidget_testDefectCount->item(row - 1, 0)->flags() & ~Qt::ItemIsEditable);

		QTableWidgetItem* tItem1 = new QTableWidgetItem(count);
		ui.tableWidget_testDefectCount->setItem(row - 1, 1, tItem1);
		ui.tableWidget_testDefectCount->item(row - 1, 1)->setFlags(ui.tableWidget_testDefectCount->item(row - 1, 1)->flags() & ~Qt::ItemIsEditable);


	}

	ui.tableWidget_testDefectCount->setHorizontalHeaderLabels(horizontalLabel);
	ui.tableWidget_testDefectCount->sortItems(1, Qt::AscendingOrder);
	ui.tableWidget_testDefectCount->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);

	ui.tableWidget_testDefectCount->blockSignals(false);
}

void GoldenRecipeDialog::refreshTestUnitDefectTableWidget()
{
	qDebug() << "refreshTestUnitDefectTableWidget";

	ui.tableWidget_testUnitDefect->blockSignals(true);
	ui.tableWidget_testUnitDefect->clear();
	ui.tableWidget_testUnitDefect->setRowCount(0); // reset


	QStringList horizontalLabel = { "Unit Name" ,"Defect Name" };


	int col = horizontalLabel.size();
	ui.tableWidget_testUnitDefect->setColumnCount(col);
	int row = 0;

	for (int i = 0; i < _testTagNameHash.size(); i++)
	{
		row++;

		QString voName = _testTagNameHash.keys()[i];
		QString tagName = _testTagNameHash[voName];

		ui.tableWidget_testUnitDefect->setRowCount(row);

		QTableWidgetItem* tItem = new QTableWidgetItem(voName);
		ui.tableWidget_testUnitDefect->setItem(row - 1, 0, tItem);
		ui.tableWidget_testUnitDefect->item(row - 1, 0)->setFlags(ui.tableWidget_testUnitDefect->item(row - 1, 0)->flags() & ~Qt::ItemIsEditable);

		QTableWidgetItem* tItem1 = new QTableWidgetItem(tagName);
		ui.tableWidget_testUnitDefect->setItem(row - 1, 1, tItem1);
		ui.tableWidget_testUnitDefect->item(row - 1, 1)->setFlags(ui.tableWidget_testUnitDefect->item(row - 1, 1)->flags() & ~Qt::ItemIsEditable);


	}

	ui.tableWidget_testUnitDefect->setHorizontalHeaderLabels(horizontalLabel);
	ui.tableWidget_testUnitDefect->sortItems(1, Qt::AscendingOrder);
	ui.tableWidget_testUnitDefect->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);

	ui.tableWidget_testUnitDefect->blockSignals(false);
}


void GoldenRecipeDialog::refreshRefDefCountTableWidget()
{
	qDebug() << "refreshDistanceMeasurementTableWidget";

	ui.tableWidget_refDefectCount->blockSignals(true);
	ui.tableWidget_refDefectCount->clear();
	ui.tableWidget_refDefectCount->setRowCount(0); // reset


	QStringList horizontalLabel = { "Defect Name" ,"Count" };


	int col = horizontalLabel.size();
	ui.tableWidget_refDefectCount->setColumnCount(col);
	int row = 0;

	for (int i = 0; i < _refTagNameCountHash.size(); i++)
	{
		row++;

		QString tagName = _refTagNameCountHash.keys()[i];
		QString count = QString::number(_refTagNameCountHash[tagName]);

		ui.tableWidget_refDefectCount->setRowCount(row);

		QTableWidgetItem* tItem = new QTableWidgetItem(tagName);
		ui.tableWidget_refDefectCount->setItem(row - 1, 0, tItem);
		ui.tableWidget_refDefectCount->item(row - 1, 0)->setFlags(ui.tableWidget_refDefectCount->item(row - 1, 0)->flags() & ~Qt::ItemIsEditable);

		QTableWidgetItem* tItem1 = new QTableWidgetItem(count);
		ui.tableWidget_refDefectCount->setItem(row - 1, 1, tItem1);
		ui.tableWidget_refDefectCount->item(row - 1, 1)->setFlags(ui.tableWidget_refDefectCount->item(row - 1, 1)->flags() & ~Qt::ItemIsEditable);


	}

	ui.tableWidget_refDefectCount->setHorizontalHeaderLabels(horizontalLabel);
	ui.tableWidget_refDefectCount->sortItems(1, Qt::AscendingOrder);
	ui.tableWidget_refDefectCount->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);

	ui.tableWidget_refDefectCount->blockSignals(false);
}

void GoldenRecipeDialog::refreshRefUnitDefectTableWidget()
{
	qDebug() << "refreshTestUnitDefectTableWidget";

	ui.tableWidget_refUnitDefect->blockSignals(true);
	ui.tableWidget_refUnitDefect->clear();
	ui.tableWidget_refUnitDefect->setRowCount(0); // reset


	QStringList horizontalLabel = { "Unit Name" ,"Defect Name" };


	int col = horizontalLabel.size();
	ui.tableWidget_refUnitDefect->setColumnCount(col);
	int row = 0;

	for (int i = 0; i < _refTagNameHash.size(); i++)
	{
		row++;

		QString voName = _refTagNameHash.keys()[i];
		QString tagName = _refTagNameHash[voName];

		ui.tableWidget_refUnitDefect->setRowCount(row);

		QTableWidgetItem* tItem = new QTableWidgetItem(voName);
		ui.tableWidget_refUnitDefect->setItem(row - 1, 0, tItem);
		ui.tableWidget_refUnitDefect->item(row - 1, 0)->setFlags(ui.tableWidget_refUnitDefect->item(row - 1, 0)->flags() & ~Qt::ItemIsEditable);

		QTableWidgetItem* tItem1 = new QTableWidgetItem(tagName);
		ui.tableWidget_refUnitDefect->setItem(row - 1, 1, tItem1);
		ui.tableWidget_refUnitDefect->item(row - 1, 1)->setFlags(ui.tableWidget_refUnitDefect->item(row - 1, 1)->flags() & ~Qt::ItemIsEditable);
	}

	ui.tableWidget_refUnitDefect->setHorizontalHeaderLabels(horizontalLabel);
	ui.tableWidget_refUnitDefect->sortItems(1, Qt::AscendingOrder);
	ui.tableWidget_refUnitDefect->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);

	ui.tableWidget_refUnitDefect->blockSignals(false);
}

void GoldenRecipeDialog::writeRefGoldenRecipe(QString recipeName, QHash<QString, QString>& tagNameHash, QHash<QString, int>& tagNameCountHash)
{
	QJsonArray vArray;
	for (int i = 0; i < tagNameHash.size(); i++)
	{
		QString voName = tagNameHash.keys()[i];
		QString tagName = tagNameHash[voName];

		QJsonObject vObject;
		vObject.insert("voName", voName);
		vObject.insert("tagName", tagName);
		vArray.append(vObject);

	}
	QJsonArray cArray;
	for (int i = 0; i < tagNameCountHash.size(); i++)
	{
		QString tagName = tagNameCountHash.keys()[i];
		int count = tagNameCountHash[tagName];

		QJsonObject cObject;
		cObject.insert("tagName", tagName);
		cObject.insert("count", count);
		cArray.append(cObject);
	}

	QJsonObject vInfo;
	vInfo.insert("voTagName", vArray);
	vInfo.insert("tagNameCount", cArray);
	vInfo.insert("recipeName", recipeName);
	QJsonDocument jsonDoc(vInfo);
	QString jsonPath = Common::Directory::getRecipeCurrentPath() + "/GoldenRecipe.json";
	QFile file(jsonPath);
	if (file.open(QIODevice::WriteOnly))
	{
		file.write(jsonDoc.toJson());
		file.close();
	}
	else
	{
		qDebug() << "Write goldenRecipeInfo failed";
	}
}

bool GoldenRecipeDialog::isMatch(QString& message)
{
	// Check if both hashes contain the same keys and values for QString hashes
	if (_testTagNameHash.size() != _refTagNameHash.size()) {
		message = "Unit does not match";
		return false; // Size mismatch, no match
	}

	// Compare _testTagNameHash and _refTagNameHash
	for (auto it = _testTagNameHash.begin(); it != _testTagNameHash.end(); ++it) {
		const QString& key = it.key();
		if (!_refTagNameHash.contains(key) || _refTagNameHash.value(key) != it.value()) {
			message = "Unit Defect Name does not match";
			return false; // Either key is missing or value mismatch
		}
	}

	// Compare _testTagNameCountHash and _refTagNameCountHash
	if (_testTagNameCountHash.size() != _refTagNameCountHash.size()) {
		message = "Defect Count does not match";
		return false; // Size mismatch, no match
	}

	for (auto it = _testTagNameCountHash.begin(); it != _testTagNameCountHash.end(); ++it) {
		const QString& key = it.key();
		if (!_refTagNameCountHash.contains(key) || _refTagNameCountHash.value(key) != it.value()) {
			message = "Defect Count does not match";
			return false; // Either key is missing or value mismatch
		}
	}

	// All comparisons passed, the hashes are a match
	message = "MATCH";
	return true;
}

GoldenRecipeDialog::~GoldenRecipeDialog()
{
}
