#pragma once

#include <QWidget>
#include <QFile>
#include <QMessageBox>
#include <QHash>
#include <QSet>
#include <QString>
#include "ui_3DOpticsTab.h"
#include "QView.h"
#include "QCommonStruct.h"
#include "utilities.h"
#include "VisionApp.h"
#include "VisionAppQDragBox.h"


class Optics3DTab : public QWidget
{
	Q_OBJECT


public:
	Optics3DTab(QWidget* parent = nullptr);
	~Optics3DTab();

    bool loadOptics3DToUi();
	bool reloadOptics3DIdCombo();     
    bool addOptics3DWithPrompt();
    bool deleteCurrentOptics3D();
    bool refreshCurrentOptics3DFromJson();

    void attachRecipeOptics3D(QHash<QString, OpticsInfo3D>* recipeOptics3D);
    bool syncToRecipeOptics3D();

private:
	Ui::Optics3DClass ui;

    struct Optics3DParams {
        int exposure = 0, exposure2 = 0, gain = 0, gain2 = 0, divider = 0;
        int lineThreshold = 0, lowerLaserLimit = 0, upperLaserLimit = 0;
        QString exposureMode; 
        QString lightSensitivity;   
        QString peakSensitivity;   
        QString peakSelection;
    };

    QHash<QString, Optics3DParams> _cacheById;
    QString _lastLoadedId;

    Optics3DParams readUiParams() const;
    void writeUiParams(const Optics3DParams& p);
    bool loadParamsFromJsonById(const QString& id, Optics3DParams& out) const;
    
    QHash<QString, OpticsInfo3D>* _recipeOptics3D = nullptr;


private slots:
    void onOpticsIdChanged();   


public:
    bool saveAllOptics3DByIdToJson();

public Q_SLOTS:

signals:

};