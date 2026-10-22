#pragma once

#include <QString>
#include <QStringList>
#include <QVector>

/*
* An LJ-X Navigator measurement setting saved as TSV (Navigator: Local setting -> Save file, type
* *.tsv). The .mssx Navigator saves by default is Keyence's undocumented binary format with a
* checksum; the TSV holds the same settings as readable text, so it is the file Pogo imports.
*
* Layout, as Navigator 2026 writes it: UTF-16 LE, one setting per row. A section header in column 0
* ("Common measurement setting", "Program0".."Program15"), then seven label columns giving the
* setting's place in Navigator's tree - a blank label means "same as the row above" - then the
* value in column 8 (a few settings, like image masks, carry more values after it).
*
* Pure Qt - its only Pogo dependency is KeyenceSettings, also pure Qt - so it can be compiled and
* tested on its own against a real export.
*/
class NavigatorTsv
{
public:
    struct Item {
        QStringList labels;     //Navigator's tree path, outermost first
        QString value;          //first value column, trimmed
        QStringList values;     //every value column, trimmed (the image masks use six)
    };

    struct Section {
        QString name;           //"Common measurement setting", "Program0".."Program15"
        QVector<Item> items;
    };

    bool load(const QString& filePath, QString* error);

    const Section* common() const;
    const Section* program(int n) const;

    //Value of the first item whose path ENDS with these labels (whole labels, case-insensitive).
    //Navigator repeats leaf names - three "Measurement range", two "Median" - so pass enough of
    //the path to be unique. nth picks a later match of the same path, column a later value.
    static bool find(const Section* s, const QStringList& pathTail, QString* value, int nth = 0, int column = 0);

private:
    QVector<Section> m_sections;
};

/*
* One row of an import: a Navigator setting and what it becomes in Pogo, or why it does not.
*/
struct NavigatorSetting
{
    enum Target {
        None,                   //shown for information - stays on the controller
        OpticExposure,
        OpticDynamicRange,
        OpticDivider,
        OpticExposureMode,
        OpticPeakSensitivity,
        OpticPeakSelection,
        OpticLaserUpper,
        OpticLaserLower,
        DrvEncoderInputMode,
        DrvEncoderMinInputTime,
        DrvLightControlMode,
        DrvInvalidInterpolation,
        OpticLjx,               //the optic's "ljx" block - key names it (KeyenceSettings)
        DrvLjx                  //keyence.json - key names it (KeyenceSettings)
    };

    QString key;                //KeyenceSettings key, for OpticLjx / DrvLjx and the shown-only rows
    QString name;               //Navigator's name for the setting
    QString fileValue;          //as Navigator wrote it
    Target target = None;
    int value = 0;              //Pogo's value; meaningful only when usable
    bool usable = false;        //understood, in range, and has somewhere to go in Pogo
    QString note;               //why it is not imported, or what to watch for
};

//Turns one program of the file into import rows, and fills warnings with the settings Pogo
//depends on but never sets itself (trigger mode, batch, binning ...) when they look wrong. No
//file or UI access, so the mapping can be checked against a real export on its own.
QVector<NavigatorSetting> mapNavigatorProgram(const NavigatorTsv& tsv, int programNo, QStringList* warnings);
