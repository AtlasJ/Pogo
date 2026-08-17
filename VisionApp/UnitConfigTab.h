#pragma once

#include <QWidget>
#include <QFile>
#include <QMessageBox>
#include <QHash>
#include <QSet>
#include <QString>
#include "ui_UnitConfigTab.h"
#include "QView.h"
#include "QCommonStruct.h"
#include "utilities.h"
#include "VisionApp.h"
#include "VisionAppQDragBox.h"


class UnitConfigTab : public QWidget
{
	Q_OBJECT

	enum class UnitMode {
		INDEX, ROW_COL
	};

public:
	UnitConfigTab(QWidget *parent = nullptr);
	~UnitConfigTab();

	void loadUnitConfig(QHash <QString, QView>& views);
	void saveUnitConfig();

	QVector<UnitConfigInfo> getUnifConfigInfos();
	QString getFirstID(QString viewID);
	QString getNextID(QString currentID, QString viewID);
	QString getPreviousID(QString currentID, QString viewID);
	int getTotalIndex(QString viewID);
	QStringList getIDList(QString viewID);

private:
	Ui::UnitConfigTabClass ui;

	QVector<UnitConfigInfo> _unitConfigInfos;

	bool loadJson(QString path, QJsonObject& root);
	bool loadJson(QString path, QJsonDocument& doc);
	bool saveJson(const QString& fileName, const QJsonDocument& doc);

	QVector<VisionAppQDragBox*> _dragROIs;
	std::vector<QDragBox*> _selectedROIs; 

public Q_SLOTS:
	void toggleUnitConfigSettings();
	void showCurrentViewUnitConfigSettings();
	void updateUnitConfigSettings();

signals:

	void runStoredUnits();
	void storeSkippedUnits();
	void removeSkippedUnits();
};
