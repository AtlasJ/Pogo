#pragma once

#include <QDir>
#include <QDialog>
#include <QDebug>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>

#include "ui_GoldenRecipeDialog.h"
#include "CommonDir.h"
#include "OpticsInfo.h"

class GoldenRecipeDialog : public QDialog
{
	Q_OBJECT

public:
	GoldenRecipeDialog(QWidget *parent = nullptr);
	~GoldenRecipeDialog();

	void goldenRecipeRunComplete();
	void initRefGoldenRecipe();
	void runGoldenRecipe(bool isProduction);
	void setOptics(QHash<QString, OpticsInfo> optics_2d, QHash<QString, OpticsInfo3D> optics_3d);
private:
	Ui::GoldenRecipeDialogClass ui;

	bool _isRunningProduction = false;
	QString _goldenRecipeImageDir;

	QHash<QString, QString> _testTagNameHash;
	QHash<QString, int> _testTagNameCountHash;
	QString _testTecipeName;

	QHash<QString, QString> _refTagNameHash;
	QHash<QString, int> _refTagNameCountHash;
	QString _refRecipeName;

	void readGoldenRecipeInfoJson(QString &path, QHash<QString, QString>& tagNameHash, QHash<QString, int>& tagNameCountHash, QString& recipeName);
	void setReferenceGoldenRecipe();
	void writeRefGoldenRecipe(QString recipeName, QHash<QString, QString>& tagNameHash, QHash<QString, int>& tagNameCountHash);

	void connectSignalAndSlot();
	void refreshTestDefCountTableWidget();
	void refreshTestUnitDefectTableWidget();

	void refreshRefDefCountTableWidget();
	void refreshRefUnitDefectTableWidget();

	bool isMatch(QString& message);

	bool checkGoldenUnitCriteria();

	QHash<QString, QString> _opticsHash;

public:


signals:
	void signalRequestGoldenRecipeResult();
	void signalRunGoldenRecipeComplete(bool isPass, QString message);
};
