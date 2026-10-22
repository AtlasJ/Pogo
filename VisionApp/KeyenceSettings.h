#pragma once

#include <QJsonObject>
#include <QString>
#include <QVector>
#include <climits>

/*
* Every LJ-X8000A measurement setting, defined once.
*
* The driver (what to send and where), the 3D Optics page (what to show and edit), the recipe and
* keyence.json (what to store) and the Navigator TSV import (what Navigator calls it) all read this
* one table, so a setting cannot be stored under one meaning and sent under another.
*
* Codes, byte layouts and value tables are from the LJ-X8000A Communication Library Reference
* Manual (N06GB, section 11.3, p.68-79), which installs with LJ-X Navigator at
* lib\Manual\en\. Navigator's own wording is from its TSV export.
*/
namespace ljx {

    //Who owns a setting in Pogo.
    enum class Level {
        Pogo,       //an existing Pogo field (3D Optics / Profiler Hardware) - shown here, edited there
        Optic,      //per 3D optic: the recipe's optics.json "ljx" block, sent before every scan
        Machine,    //keyence.json, sent at Connect
        Check,      //Pogo depends on one value and never changes it - shown, and checked on import
        Info        //Navigator's (or unused on this head) - shown, never sent
    };

    struct Option {
        int code;
        const char* label;      //display text, then '|'-separated aliases Navigator may write instead
    };

    struct Field {
        QString key;            //JSON key, in optics.json "ljx" or keyence.json
        QString group;          //tree group on the page
        QString name;           //Navigator's wording
        QString tsv;            //tail of the setting's path in Navigator's TSV, '|'-separated; empty = not in it
        int tsvNth = 0;         //which occurrence of that path (Navigator repeats "Pitch between triggers")
        int tsvColumn = 0;      //which value column (the six image masks share one row)
        Level level = Level::Info;
        bool legacy = false;    //sent by Pogo's existing code (loadConfig / JobThread::scan), not by applySettings
        bool perOptic = false;  //a Pogo-level setting that belongs to the optic rather than the machine
        bool common = false;    //common measurement setting (type 02h) rather than the program's (10h + n)
        unsigned char category = 0, item = 0, target1 = 0;
        int blockSize = 0;      //bytes in the whole SDK item; 0 = no SDK address (Navigator only)
        int offset = 0;         //this field's first byte inside the item, little-endian
        int size = 1;           //1, 2 or 4 bytes
        bool isSigned = false;
        int min = 0, max = 0;   //valid codes
        double scale = 1.0;     //shown value = code * scale
        QString unit;           //after the shown value: "%", "mm", "deg"
        QVector<Option> options;//empty = a number
        int required = INT_MIN; //Check: the one value Pogo works with; INT_MIN = none
        int fallback = INT_MIN; //a legacy key the driver sends with this value when the file lacks
                                //it, so it can never be "not set"; INT_MIN = absent means not sent
        QString note;
    };

    constexpr int kNotSet = INT_MIN;

    const QVector<Field>& fields();
    const Field* field(const QString& key);

    //"640us", "ON", "50.0%". Unknown enum codes show as "code N" rather than being guessed.
    QString display(const Field& f, int code);

    //Navigator's text -> code. False for "---" (does not apply in this mode), blanks, and
    //anything outside the field's options or range.
    bool parse(const Field& f, const QString& text, int* code);

    //Whether a code is one this field can be set to.
    bool valid(const Field& f, int code);

    //Byte packing for one field inside its SDK item.
    void writeField(QByteArray& block, const Field& f, int code);
    int readField(const QByteArray& block, const Field& f);

    /*
    * Per-optic values of the loaded recipe, by optic id. OpticsInfo3D lives in the shared Common
    * library, so the LJ-X block travels beside it here rather than inside it. Filled when the
    * recipe loads and when the 3D Optics page saves; read by JobThread::scan() from the worker
    * thread, hence the lock inside.
    */
    QJsonObject opticSettings(const QString& opticId);
    void setOpticSettings(const QString& opticId, const QJsonObject& values);
    void removeOpticSettings(const QString& opticId);
    void clearOpticSettings();

} // namespace ljx
