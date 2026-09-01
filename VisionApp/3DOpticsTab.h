#pragma once

#include <QWidget>
#include <QFile>
#include <QMessageBox>
#include <QHash>
#include <QSet>
#include <QString>
#include <QTimer>
#include <QJsonObject>
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

    //Camera -> profiler alignment (Set / Jog To / Offset To + live profile view).
    //The taught offset feeds the same laser offset production uses for offset-to-3D.
    void updateLiveProfile(QVector<double> profile, double xFovMm, double zRangeMm);

signals:
    void alignJogTo(double x, double y, double z);
    void alignLaserOffset(double dx, double dy, double dz);
    void alignLiveProfile(bool enable);

public:
    //Profiler hardware section. MACHINE level (config\profiler.json + the driver config it
    //names), NOT part of the recipe. Deliberately kept out of Optics3DParams, _cacheById and
    //saveAllOptics3DByIdToJson() - anything in those is per-optics-ID and would silently change
    //when the user picks a different ID from the combo.
    bool loadProfilerHwToUi();

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

    //--- Profiler hardware section ---
    QString profilerId() const;                 //live id from ProfilerManager, else "profiler1"
    QString profilerJsonPath() const;           //config\profiler.json
    QString driverConfigPath() const;           //config\<Config_File> named by profiler.json

    void initProfilerHwUi();                    //combo contents + signal wiring, called once
    void refreshProfilerStatus();               //status pill, info, enablement, message line
    bool saveProfilerHwToJson();                //read-modify-write, keeps every untouched key
    bool profilerBusy() const;                  //grabbing or gantry moving = refuse to touch it
    void runProfilerConnect(bool wantConnected); //blocking; wait cursor + section disabled
    bool readProfilerEntry(QJsonObject& entry) const; //this profiler's entry in profiler.json
    void markProfilerHwDirty();                 //an edit is pending a Save

    //--- Camera -> profiler alignment section ---
    void initAlign3DUi();
    void refreshAlignLabels();
    bool loadAlign3D();
    bool saveAlign3D() const;
    bool _alignCamSet = false, _alignProfSet = false;
    double _alignCamX = 0, _alignCamY = 0, _alignCamZ = 0;
    double _alignProfX = 0, _alignProfY = 0, _alignProfZ = 0;

    QTimer* _profStatusTimer = nullptr;
    QString _profilerConfigFile;                //Config_File value as written in profiler.json
    bool _profilerBusyUi = false;
    bool _profilerHwDirty = false;              //widgets differ from what is saved on disk

private slots:
    void onOpticsIdChanged();


public:
    bool saveAllOptics3DByIdToJson();

public Q_SLOTS:

signals:

};