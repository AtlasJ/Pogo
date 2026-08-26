#include "3DOpticsTab.h"
#include "CommonDir.h"
#include "Logger.h"
#include "AuditLog.h"
#include "ProfilerManager.h"
#include "SystemData.h"
#include <QApplication>
#include <QStandardItemModel>
#include <QPushButton>
#include <QRegExp>
#include <QRegExpValidator>



Optics3DTab::Optics3DTab(QWidget* parent)
	: QWidget(parent)
{
	ui.setupUi(this);

    ui.comboBox_exposureMode->clear();
    ui.comboBox_exposureMode->addItems(QStringList() << "single" << "multi" << "dynamic" << "parallel");
    ui.comboBox_lightSensitivity->addItems(QStringList() << "High Precision" << "HDR 1" << "HDR 2" << "HDR 3");
    ui.comboBox_peakSensitivity->addItems(QStringList() << "1" << "2" << "3" << "4"<<"5");
    ui.comboBox_peakSelection->addItems(QStringList() << "Standard" << "Near" << "Far" << "Invalid");



    connect(ui.comboBox_opticsID, QOverload<int>::of(&QComboBox::currentIndexChanged),
        this, [=](int) { onOpticsIdChanged(); });

    connect(ui.toolButton_save3DOptics, &QToolButton::clicked,
        this, [=]() { syncToRecipeOptics3D();  saveAllOptics3DByIdToJson(); AuditLog::instance().log(QStringLiteral("OPTIC3D_SAVE")); });

    connect(ui.toolButton_add3DOptics, &QToolButton::clicked,
        this, [=]() { addOptics3DWithPrompt(); AuditLog::instance().log(QStringLiteral("OPTIC3D_ADD")); });

    connect(ui.toolButton_del3DOptics, &QToolButton::clicked,
        this, [=]() { deleteCurrentOptics3D(); });

    initProfilerHwUi();
}

Optics3DTab::~Optics3DTab()
{
}



bool Optics3DTab::reloadOptics3DIdCombo()
{
    const QString path = QDir(Common::Directory::getRecipeCurrentPath()).filePath("optics.json");

    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)) {
        ct::logger::warn("[Optics3DTab] Cannot open: %s", qPrintable(path));
        ui.comboBox_opticsID->clear();
        return false;
    }

    QJsonParseError err{};
    QJsonDocument doc = QJsonDocument::fromJson(f.readAll(), &err);
    f.close();

    if (err.error != QJsonParseError::NoError || !doc.isObject()) {
        ct::logger::warn("[Optics3DTab] JSON parse error: %s", qPrintable(err.errorString()));
        ui.comboBox_opticsID->clear();
        return false;
    }

    const QJsonArray arr = doc.object().value("optics3D").toArray();
    ui.comboBox_opticsID->blockSignals(true);
    ui.comboBox_opticsID->clear();

    for (const auto& it : arr) {
        if (!it.isObject()) continue;
        const QString id = it.toObject().value("id").toString();
        if (!id.isEmpty())
            ui.comboBox_opticsID->addItem(id, id);   // display=id, data=id
    }

    ui.comboBox_opticsID->blockSignals(false);

    if (ui.comboBox_opticsID->count() == 0) {
        ct::logger::warn("[Optics3DTab] No optics3D IDs found");
        return false;
    }

    ui.comboBox_opticsID->setCurrentIndex(0);

    _lastLoadedId.clear();   
    onOpticsIdChanged();     
    return true;
}


Optics3DTab::Optics3DParams Optics3DTab::readUiParams() const
{
    auto toIntSafe = [](const QString& s, int def = 0) {
        bool ok = false; int v = s.trimmed().toInt(&ok); return ok ? v : def;
        };

    Optics3DParams p;
    p.exposure = toIntSafe(ui.lineEdit_Exposure->text());
    p.exposure2 = toIntSafe(ui.lineEdit_Exposure2->text());
    p.gain = toIntSafe(ui.lineEdit_Gain->text());
    p.gain2 = toIntSafe(ui.lineEdit_Gain2->text());
    p.divider = toIntSafe(ui.lineEdit_Divider->text());
    p.lineThreshold = toIntSafe(ui.lineEdit_lineThreshold->text());
    p.lowerLaserLimit = toIntSafe(ui.lineEdit_laserLowerLimit->text());
    p.upperLaserLimit = toIntSafe(ui.lineEdit_laserUpperLimit->text());
    p.exposureMode = ui.comboBox_exposureMode->currentText().trimmed().toLower();
    p.lightSensitivity = ui.comboBox_lightSensitivity->currentText().trimmed();
    p.peakSensitivity = ui.comboBox_peakSensitivity->currentText().trimmed();
    p.peakSelection = ui.comboBox_peakSelection->currentText().trimmed();
    return p;
}

void Optics3DTab::writeUiParams(const Optics3DParams& p)
{
    QSignalBlocker b1(ui.lineEdit_Exposure);
    QSignalBlocker b2(ui.lineEdit_Exposure2);
    QSignalBlocker b3(ui.lineEdit_Gain);
    QSignalBlocker b4(ui.lineEdit_Gain2);
    QSignalBlocker b5(ui.lineEdit_Divider);
    QSignalBlocker b6(ui.lineEdit_lineThreshold);
    QSignalBlocker b7(ui.lineEdit_laserLowerLimit);
    QSignalBlocker b8(ui.lineEdit_laserUpperLimit);
    QSignalBlocker b9(ui.comboBox_exposureMode);
    QSignalBlocker b10(ui.comboBox_lightSensitivity);
    QSignalBlocker b11(ui.comboBox_peakSensitivity);
    QSignalBlocker b12(ui.comboBox_peakSelection);

    ui.lineEdit_Exposure->setText(QString::number(p.exposure));
    ui.lineEdit_Exposure2->setText(QString::number(p.exposure2));
    ui.lineEdit_Gain->setText(QString::number(p.gain));
    ui.lineEdit_Gain2->setText(QString::number(p.gain2));
    ui.lineEdit_Divider->setText(QString::number(p.divider));

    ui.lineEdit_lineThreshold->setText(QString::number(p.lineThreshold));
    ui.lineEdit_laserLowerLimit->setText(QString::number(p.lowerLaserLimit));
    ui.lineEdit_laserUpperLimit->setText(QString::number(p.upperLaserLimit));

    int idx = ui.comboBox_exposureMode->findText(p.exposureMode, Qt::MatchFixedString);
    ui.comboBox_exposureMode->setCurrentIndex(idx < 0 ? 0 : idx);

    auto setComboByText = [](QComboBox* cb, const QString& val) {
        if (!cb) return;
        if (cb->count() <= 0) return;
        if (val.trimmed().isEmpty()) { cb->setCurrentIndex(0); return; }
        int i = cb->findText(val, Qt::MatchFixedString);
        cb->setCurrentIndex(i < 0 ? 0 : i);
        };

    setComboByText(ui.comboBox_lightSensitivity, p.lightSensitivity);
    setComboByText(ui.comboBox_peakSensitivity, p.peakSensitivity);
    setComboByText(ui.comboBox_peakSelection, p.peakSelection);
}

bool Optics3DTab::loadParamsFromJsonById(const QString& id, Optics3DParams& out) const
{
    const QString path = QDir(Common::Directory::getRecipeCurrentPath()).filePath("optics.json");

    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)) return false;

    QJsonParseError err{};
    QJsonDocument doc = QJsonDocument::fromJson(f.readAll(), &err);
    f.close();

    if (err.error != QJsonParseError::NoError || !doc.isObject()) return false;

    const QJsonArray arr = doc.object().value("optics3D").toArray();
    for (const auto& it : arr) {
        if (!it.isObject()) continue;
        QJsonObject o = it.toObject();
        if (o.value("id").toString() != id) continue;

        out.exposure = o.value("exposure").toInt();
        out.exposure2 = o.value("exposure2").toInt();
        out.gain = o.value("gain").toInt();
        out.gain2 = o.value("gain2").toInt();
        out.divider = o.value("divider").toInt();
        out.lineThreshold = o.value("lineThreshold").toInt();
        out.lowerLaserLimit = o.value("lowerLaserLimit").toInt();
        out.upperLaserLimit = o.value("upperLaserLimit").toInt();
        out.exposureMode = o.value("exposureMode").toString().trimmed().toLower();
        out.lightSensitivity = o.value("lightSensitivity").toString();
        out.peakSensitivity = o.value("peakSensitivity").toString();
        out.peakSelection = o.value("peakSelection").toString();
        return true;
    }
    return false;
}

void Optics3DTab::onOpticsIdChanged()
{
    // 1) cache current UI into previous ID
    if (!_lastLoadedId.isEmpty()) {
        _cacheById[_lastLoadedId] = readUiParams();
    }

    // 2) get new selected ID
    QString newId = ui.comboBox_opticsID->currentData().toString();
    if (newId.isEmpty()) newId = ui.comboBox_opticsID->currentText();
    if (newId.isEmpty()) return;

    // 3) load new ID params: cache first, else json
    Optics3DParams p;
    if (_cacheById.contains(newId)) {
        p = _cacheById.value(newId);
    }
    else {
        if (!loadParamsFromJsonById(newId, p)) return;
        _cacheById.insert(newId, p);
    }

    // 4) write to UI
    writeUiParams(p);

    _lastLoadedId = newId;
}

bool Optics3DTab::saveAllOptics3DByIdToJson()
{
    // make sure current UI is cached too
    if (!_lastLoadedId.isEmpty()) {
        _cacheById[_lastLoadedId] = readUiParams();
    }

    const QString path = QDir(Common::Directory::getRecipeCurrentPath()).filePath("optics.json");

    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)) {
        ct::logger::warn("[Optics3DTab] SaveById failed: cannot open read: %s", qPrintable(path));
        return false;
    }

    QJsonParseError err{};
    QJsonDocument doc = QJsonDocument::fromJson(f.readAll(), &err);
    f.close();

    if (err.error != QJsonParseError::NoError || !doc.isObject()) {
        ct::logger::warn("[Optics3DTab] SaveById failed: JSON parse error: %s", qPrintable(err.errorString()));
        return false;
    }

    QJsonObject root = doc.object();
    QJsonArray arr = root.value("optics3D").toArray();
    if (arr.isEmpty()) return false;

    int updated = 0;
    for (int i = 0; i < arr.size(); ++i) {
        if (!arr[i].isObject()) continue;

        QJsonObject o = arr[i].toObject();
        const QString id = o.value("id").toString();
        if (!_cacheById.contains(id)) continue;

        const auto p = _cacheById.value(id);

        o["exposure"] = p.exposure;
        o["exposure2"] = p.exposure2;
        o["gain"] = p.gain;
        o["gain2"] = p.gain2;
        o["divider"] = p.divider;
        o["lineThreshold"] = p.lineThreshold;
        o["lowerLaserLimit"] = p.lowerLaserLimit;
        o["upperLaserLimit"] = p.upperLaserLimit;
        o["exposureMode"] = p.exposureMode;
        o["lightSensitivity"] = p.lightSensitivity;
        o["peakSensitivity"] = p.peakSensitivity;
        o["peakSelection"] = p.peakSelection;

        arr[i] = o;
        updated++;
    }

    root["optics3D"] = arr;

    QSaveFile out(path);
    if (!out.open(QIODevice::WriteOnly)) {
        ct::logger::warn("[Optics3DTab] SaveById failed: cannot open write: %s", qPrintable(path));
        return false;
    }

    out.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
    if (!out.commit()) {
        ct::logger::warn("[Optics3DTab] SaveById failed: commit failed: %s", qPrintable(path));
        return false;
    }

    ct::logger::info("[Optics3DTab] SaveById OK. Updated=%d, file=%s", updated, qPrintable(path));
    return true;
}

bool Optics3DTab::loadOptics3DToUi()
{
    _cacheById.clear();
    _lastLoadedId.clear();
    return reloadOptics3DIdCombo();
}

bool Optics3DTab::addOptics3DWithPrompt()
{
    // Cache current edits first (so switching doesn�t lose changes)
    if (!_lastLoadedId.isEmpty())
        _cacheById[_lastLoadedId] = readUiParams();

    const QString path = QDir(Common::Directory::getRecipeCurrentPath()).filePath("optics.json");

    // ---- Read JSON ----
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)) {
        ct::logger::warn("[Optics3DTab] Add failed: cannot open read: %s", qPrintable(path));
        return false;
    }

    QJsonParseError err{};
    QJsonDocument doc = QJsonDocument::fromJson(f.readAll(), &err);
    f.close();

    if (err.error != QJsonParseError::NoError || !doc.isObject()) {
        ct::logger::warn("[Optics3DTab] Add failed: JSON parse error: %s", qPrintable(err.errorString()));
        return false;
    }

    QJsonObject root = doc.object();
    QJsonArray arr = root.value("optics3D").toArray(); // if missing -> empty array is fine

    auto idExists = [&](const QString& id)->bool {
        for (const auto& it : arr) {
            if (!it.isObject()) continue;
            if (it.toObject().value("id").toString() == id) return true;
        }
        return false;
        };

    // ---- Prompt user for ID (loop until valid or cancelled) ----
    QString newId;
    while (true) {
        bool ok = false;
        newId = QInputDialog::getText(
            this,
            tr("Add 3D Optics"),
            tr("Enter new Optics 3D ID:"),
            QLineEdit::Normal,
            "",
            &ok
        );

        if (!ok) return false;                // user cancelled
        newId = newId.trimmed();
        if (newId.isEmpty()) {
            QMessageBox::warning(this, tr("Invalid ID"), tr("ID cannot be empty."));
            continue;
        }

        // Optional: restrict characters (avoid weird JSON/user typing)
        // Allow letters, numbers, underscore, dash
        static const QRegularExpression re("^[A-Za-z0-9_-]+$");
        if (!re.match(newId).hasMatch()) {
            QMessageBox::warning(this, tr("Invalid ID"),
                tr("ID can only contain letters, numbers, '_' and '-'."));
            continue;
        }

        if (idExists(newId)) {
            QMessageBox::warning(this, tr("Duplicate ID"),
                tr("This ID already exists. Please enter a different ID."));
            continue;
        }

        break; // valid
    }

    // ---- Default values (change if you want) ----
    Optics3DParams def;
    def.exposure = 60;
    def.exposure2 = 0;
    def.gain = 0;
    def.gain2 = 0;
    def.divider = 4;
    def.lineThreshold = 0;
    def.lowerLaserLimit = 80;
    def.upperLaserLimit = 80;
    def.exposureMode = "single";
    def.lightSensitivity = "High Precision";
    def.peakSensitivity = "5";
    def.peakSelection = "Standard";

    // ---- Create new JSON entry ----
    QJsonObject oNew;
    oNew["id"] = newId;
    oNew["name"] = newId;        // name same as ID
    oNew["tag"] = "";
    oNew["intensity"] = true;    // ALWAYS true

    oNew["exposure"] = def.exposure;
    oNew["exposure2"] = def.exposure2;
    oNew["gain"] = def.gain;
    oNew["gain2"] = def.gain2;
    oNew["divider"] = def.divider;
    oNew["lineThreshold"] = def.lineThreshold;
    oNew["lowerLaserLimit"] = def.lowerLaserLimit;
    oNew["upperLaserLimit"] = def.upperLaserLimit;
    oNew["exposureMode"] = def.exposureMode;
    oNew["lightSensitivity"] = def.lightSensitivity;
    oNew["peakSensitivity"] = def.peakSensitivity;
    oNew["peakSelection"] = def.peakSelection;

    arr.append(oNew);
    root["optics3D"] = arr;

    // ---- Write JSON safely ----
    QSaveFile out(path);
    if (!out.open(QIODevice::WriteOnly)) {
        ct::logger::warn("[Optics3DTab] Add failed: cannot open write: %s", qPrintable(path));
        return false;
    }

    out.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
    if (!out.commit()) {
        ct::logger::warn("[Optics3DTab] Add failed: commit failed: %s", qPrintable(path));
        return false;
    }

    // ---- Update cache + UI selection ----
    _cacheById.insert(newId, def);

    if (_recipeOptics3D) {
        OpticsInfo3D o;
        o.id = newId;
        o.name = newId;
        o.tag = "";
        o.intensity = true;
        o.exposureMode = def.exposureMode;
        o.exposure = def.exposure;
        o.exposure2 = def.exposure2;
        o.gain = def.gain;
        o.gain2 = def.gain2;
        o.divider = def.divider;
        o.lineThreshold = def.lineThreshold;
        o.lowerLaserLimit = def.lowerLaserLimit;
        o.upperLaserLimit = def.upperLaserLimit;
        o.lightSensitivity = def.lightSensitivity;
        o.peakSensitivity = def.peakSensitivity;
        o.peakSelection = def.peakSelection;

        _recipeOptics3D->insert(newId, o);
    }

    // Reload combo from disk, then select the new ID
    if (!reloadOptics3DIdCombo())
        return false;

    int idx = ui.comboBox_opticsID->findData(newId);
    if (idx < 0) idx = ui.comboBox_opticsID->findText(newId, Qt::MatchFixedString);

    if (idx >= 0) {
        ui.comboBox_opticsID->setCurrentIndex(idx); // triggers onOpticsIdChanged()
    }
    else {
        // fallback
        writeUiParams(def);
        _lastLoadedId = newId;
    }

    ct::logger::info("[Optics3DTab] Added optics3D id=%s", qPrintable(newId));
    return true;
}

bool Optics3DTab::deleteCurrentOptics3D()
{
    // current selected id
    int curIdx = ui.comboBox_opticsID->currentIndex();
    if (curIdx < 0 || ui.comboBox_opticsID->count() == 0) {
        ct::logger::warn("[Optics3DTab] Delete failed: no selection");
        return false;
    }

    QString id = ui.comboBox_opticsID->currentData().toString();
    if (id.isEmpty()) id = ui.comboBox_opticsID->currentText();
    if (id.isEmpty()) return false;

    // confirm
    const auto ret = QMessageBox::question(
        this,
        tr("Delete 3D Optics"),
        tr("Delete Optics3D ID '%1'?\nThis cannot be undone.").arg(id),
        QMessageBox::Yes | QMessageBox::No,
        QMessageBox::No
    );
    if (ret != QMessageBox::Yes) return false;

    AuditLog::instance().log(QStringLiteral("OPTIC3D_DELETE"), id);

    // decide what to select after delete (next/prev)
    int nextIdx = curIdx;
    if (nextIdx >= ui.comboBox_opticsID->count() - 1) nextIdx = curIdx - 1;

    // ---- read JSON ----
    const QString path = QDir(Common::Directory::getRecipeCurrentPath()).filePath("optics.json");

    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)) {
        ct::logger::warn("[Optics3DTab] Delete failed: cannot open read: %s", qPrintable(path));
        return false;
    }

    QJsonParseError err{};
    QJsonDocument doc = QJsonDocument::fromJson(f.readAll(), &err);
    f.close();

    if (err.error != QJsonParseError::NoError || !doc.isObject()) {
        ct::logger::warn("[Optics3DTab] Delete failed: JSON parse error: %s", qPrintable(err.errorString()));
        return false;
    }

    QJsonObject root = doc.object();
    QJsonValue vOptics3D = root.value("optics3D");
    if (!vOptics3D.isArray()) {
        ct::logger::warn("[Optics3DTab] Delete failed: optics3D missing/not array");
        return false;
    }

    QJsonArray arr = vOptics3D.toArray();

    // ---- remove by id ----
    QJsonArray newArr;
    bool removed = false;
    for (const auto& it : arr) {
        if (!it.isObject()) continue;
        QJsonObject o = it.toObject();
        if (o.value("id").toString() == id) {
            removed = true;
            continue; // skip this one
        }
        newArr.append(o);
    }

    if (!removed) {
        ct::logger::warn("[Optics3DTab] Delete failed: id not found in json: %s", qPrintable(id));
        return false;
    }

    root["optics3D"] = newArr;

    // ---- write JSON safely ----
    QSaveFile out(path);
    if (!out.open(QIODevice::WriteOnly)) {
        ct::logger::warn("[Optics3DTab] Delete failed: cannot open write: %s", qPrintable(path));
        return false;
    }
    out.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
    if (!out.commit()) {
        ct::logger::warn("[Optics3DTab] Delete failed: commit failed: %s", qPrintable(path));
        return false;
    }

    if (_recipeOptics3D) {
        _recipeOptics3D->remove(id);
    }

    // ---- clear cache for this id ----
    _cacheById.remove(id);
    if (_lastLoadedId == id) _lastLoadedId.clear();

    // ---- reload combo ----
    if (!reloadOptics3DIdCombo()) {
        // none left or reload failed -> clear UI
        ui.comboBox_opticsID->blockSignals(true);
        ui.comboBox_opticsID->clear();
        ui.comboBox_opticsID->blockSignals(false);

        writeUiParams(Optics3DParams{}); // sets zeros and mode index 0
        ct::logger::info("[Optics3DTab] Deleted id=%s. No entries left.", qPrintable(id));
        return true;
    }

    // select next/prev if valid
    if (nextIdx >= 0 && nextIdx < ui.comboBox_opticsID->count()) {
        ui.comboBox_opticsID->setCurrentIndex(nextIdx); // triggers onOpticsIdChanged()
    }
    else {
        ui.comboBox_opticsID->setCurrentIndex(0);
    }

    ct::logger::info("[Optics3DTab] Deleted optics3D id=%s", qPrintable(id));
    return true;
}

bool Optics3DTab::refreshCurrentOptics3DFromJson()
{
    // get current selected ID
    QString id = ui.comboBox_opticsID->currentData().toString();
    if (id.isEmpty()) id = ui.comboBox_opticsID->currentText();
    if (id.isEmpty()) return false;

    // load from json (always)
    Optics3DParams p;
    if (!loadParamsFromJsonById(id, p)) {
        ct::logger::warn("[Optics3DTab] RefreshFromJson failed: id=%s not found", qPrintable(id));
        return false;
    }

    // update cache + UI
    _cacheById[id] = p;

    // avoid overwriting _cacheById[_lastLoadedId] with current UI during onOpticsIdChanged
    QSignalBlocker b(ui.comboBox_opticsID);
    writeUiParams(p);

    _lastLoadedId = id;
    ct::logger::info("[Optics3DTab] RefreshFromJson OK: id=%s", qPrintable(id));
    return true;
}

void Optics3DTab::attachRecipeOptics3D(QHash<QString, OpticsInfo3D>* recipeOptics3D)
{
    _recipeOptics3D = recipeOptics3D;
}

bool Optics3DTab::syncToRecipeOptics3D()
{
    if (!_recipeOptics3D) return false;

    // ensure current UI is cached
    if (!_lastLoadedId.isEmpty())
        _cacheById[_lastLoadedId] = readUiParams();

    // push cache into VisionApp memory
    for (auto it = _cacheById.begin(); it != _cacheById.end(); ++it) {
        const QString id = it.key();
        const auto& p = it.value();

        OpticsInfo3D o;
        if (_recipeOptics3D->contains(id)) {
            o = _recipeOptics3D->value(id);   
        }
        else {
            o.id = id;
            o.name = id;
            o.tag = "";
            o.intensity = true;
            o.exposureMode = "single";
        }

        o.exposure = p.exposure;
        o.exposure2 = p.exposure2;
        o.gain = p.gain;
        o.gain2 = p.gain2;
        o.divider = p.divider;
        o.lineThreshold = p.lineThreshold;
        o.lowerLaserLimit = p.lowerLaserLimit;
        o.upperLaserLimit = p.upperLaserLimit;
        o.exposureMode = p.exposureMode;      
        o.lightSensitivity = p.lightSensitivity;
        o.peakSensitivity = p.peakSensitivity;
        o.peakSelection = p.peakSelection;
        _recipeOptics3D->insert(id, o);
    }

    qDebug() << "=== Synced _recipeOptics3D Contents ===";
    for (auto it = _recipeOptics3D->constBegin(); it != _recipeOptics3D->constEnd(); ++it) {
        const auto& obj = it.value();
        qDebug().nospace() << "ID: " << obj.id
            << " | Mode: " << obj.exposureMode
            << " | Exp1/2: " << obj.exposure << "/" << obj.exposure2
            << " | Gain1/2: " << obj.gain << "/" << obj.gain2
            << " | Divider: " << obj.divider
            << " | LineThresh: " << obj.lineThreshold
            << " | Laser(L/U): " << obj.lowerLaserLimit << "/" << obj.upperLaserLimit
            << " | LightSens: " << obj.lightSensitivity
            << " | PeakSens: " << obj.peakSensitivity
            << " | PeakSel: " << obj.peakSelection;
    }
    qDebug() << "=======================================";

    return true;
}


//---------------------------------------------------------------------------------------------
// Profiler hardware section - MACHINE level
//
// Reads/writes config\profiler.json (IP) and the driver config it names through Config_File
// (ports, trigger, encoder). None of this goes near Optics3DParams, _cacheById or
// saveAllOptics3DByIdToJson(): those are per-optics-ID recipe state, and a machine setting that
// leaked into them would silently change every time the user picked a different Optics ID.
//---------------------------------------------------------------------------------------------

namespace {

    bool readJsonObject(const QString& path, QJsonObject& out)
    {
        QFile f(path);
        if (!f.open(QIODevice::ReadOnly)) {
            ct::logger::warn("[Optics3DTab/Prof] Cannot open read: %s", qPrintable(path));
            return false;
        }

        QJsonParseError err{};
        const QJsonDocument doc = QJsonDocument::fromJson(f.readAll(), &err);
        f.close();

        if (err.error != QJsonParseError::NoError || !doc.isObject()) {
            ct::logger::warn("[Optics3DTab/Prof] JSON parse error in %s: %s",
                qPrintable(path), qPrintable(err.errorString()));
            return false;
        }

        out = doc.object();
        return true;
    }

    bool writeJsonObject(const QString& path, const QJsonObject& obj)
    {
        QSaveFile out(path);
        if (!out.open(QIODevice::WriteOnly)) {
            ct::logger::warn("[Optics3DTab/Prof] Cannot open write: %s", qPrintable(path));
            return false;
        }

        out.write(QJsonDocument(obj).toJson(QJsonDocument::Indented));
        if (!out.commit()) {
            ct::logger::warn("[Optics3DTab/Prof] Commit failed: %s", qPrintable(path));
            return false;
        }
        return true;
    }

}

QString Optics3DTab::profilerId() const
{
    const auto ids = ProfilerManager::instance().keys();
    if (!ids.isEmpty()) return ids.first();
    return QStringLiteral("profiler1");   //JobThread's default when profiler.json never loaded
}

QString Optics3DTab::profilerJsonPath() const
{
    return Common::Directory::ConfigPath() + QStringLiteral("profiler.json");
}

QString Optics3DTab::driverConfigPath() const
{
    if (_profilerConfigFile.isEmpty()) return QString();
    return Common::Directory::ConfigPath() + _profilerConfigFile;
}

void Optics3DTab::initProfilerHwUi()
{
    //Every one of these is an enumeration in the driver config, so carry the numeric code as
    //itemData instead of parsing it back out of the label.
    ui.comboBox_profTriggerMode->clear();
    ui.comboBox_profTriggerMode->addItem(tr("0  Continuous"), 0);
    ui.comboBox_profTriggerMode->addItem(tr("1  External"), 1);
    ui.comboBox_profTriggerMode->addItem(tr("2  Encoder"), 2);

    ui.comboBox_profEncoderMode->clear();
    ui.comboBox_profEncoderMode->addItem(tr("0  1-phase (no direction)"), 0);
    ui.comboBox_profEncoderMode->addItem(tr("1  2-phase x1"), 1);
    ui.comboBox_profEncoderMode->addItem(tr("2  2-phase x2"), 2);
    ui.comboBox_profEncoderMode->addItem(tr("3  2-phase x4"), 3);

    static const char* const minTimes[] = {
        "120 ns", "150 ns", "250 ns", "500 ns", "1 us", "2 us", "5 us", "10 us", "20 us" };

    ui.comboBox_profEncoderMinTime->clear();
    for (int i = 0; i < 9; ++i)
        ui.comboBox_profEncoderMinTime->addItem(QStringLiteral("%1  %2").arg(i).arg(minTimes[i]), i);

    //A typo'd octet is not a cheap mistake here: it gets past a mere is-empty check, and Connect
    //then freezes the GUI for the SDK's internal EthernetOpen timeout (~10 s, and LJX8_IF.h
    //exposes no way to shorten it). Reject it at the keystroke instead. The validator still
    //admits partial input ("192.168.") as Intermediate, so saveProfilerHwToJson() also asks
    //hasAcceptableInput() before writing.
    const QString octet = QStringLiteral("(25[0-5]|2[0-4][0-9]|1[0-9]{2}|[1-9]?[0-9])");
    const QString ipPattern = QStringLiteral("^") + octet + "\\." + octet + "\\."
                            + octet + "\\." + octet + QStringLiteral("$");
    ui.lineEdit_profIP->setValidator(new QRegExpValidator(QRegExp(ipPattern), this));
    ui.lineEdit_profIP->setPlaceholderText(QStringLiteral("192.168.0.1"));

    //yPitchUm is shown but not editable: it must stay equal to KEYENCE_Y_PITCH_UM in
    //ImageManager::rotate_heightMap, and nothing in the code detects a mismatch - parts just
    //measure long or short along the scan axis. Make it editable only once that duplication is
    //resolved in favour of a single source of truth.
    ui.label_profYPitch->setToolTip(
        tr("Read-only. Must match KEYENCE_Y_PITCH_UM in ImageManager.cpp - edit keyence.json and "
           "the constant together, or parts measure long/short along the scan axis."));

    //The LJ-X8000A is a single head. setMultiExposure, setDynamicExposure, setParallelExposure
    //and setDuoHeadGain all refuse in Profiler_Keyence, and JobThread::scan() ignores their
    //return values - so choosing one of those modes silently scans with whatever exposure was
    //set last. Only Single is real on this sensor. Disabled rather than removed so an existing
    //recipe value still displays instead of being quietly rewritten.
    if (ProfilerManager::instance().getAPI().compare(QStringLiteral("KeyenceLJ"), Qt::CaseInsensitive) == 0) {
        if (auto* model = qobject_cast<QStandardItemModel*>(ui.comboBox_exposureMode->model())) {
            for (int i = 0; i < model->rowCount(); ++i) {
                QStandardItem* item = model->item(i);
                if (!item) continue;
                if (item->text().compare(QStringLiteral("single"), Qt::CaseInsensitive) == 0) continue;
                item->setEnabled(false);
                item->setToolTip(tr("Not available on the LJ-X8000A (single head)."));
            }
        }
    }

    //Dirty tracking exists only to sequence Save before Connect. textEdited rather than
    //textChanged so loadProfilerHwToUi() cannot mark itself dirty.
    connect(ui.lineEdit_profIP, &QLineEdit::textEdited,
        this, [=](const QString&) { markProfilerHwDirty(); });
    connect(ui.spinBox_profCmdPort, QOverload<int>::of(&QSpinBox::valueChanged),
        this, [=](int) { markProfilerHwDirty(); });
    connect(ui.spinBox_profHsPort, QOverload<int>::of(&QSpinBox::valueChanged),
        this, [=](int) { markProfilerHwDirty(); });
    connect(ui.comboBox_profTriggerMode, QOverload<int>::of(&QComboBox::currentIndexChanged),
        this, [=](int) { markProfilerHwDirty(); });
    connect(ui.comboBox_profEncoderMode, QOverload<int>::of(&QComboBox::currentIndexChanged),
        this, [=](int) { markProfilerHwDirty(); });
    connect(ui.comboBox_profEncoderMinTime, QOverload<int>::of(&QComboBox::currentIndexChanged),
        this, [=](int) { markProfilerHwDirty(); });

    connect(ui.toolButton_profConnect, &QToolButton::clicked, this, [=]() {
        if (ProfilerManager::instance().isConnected(profilerId())) {
            runProfilerConnect(false);
            return;
        }

        //Editing is only possible while disconnected, so this is the one place an unsaved edit
        //can be stranded. Connect applies the SAVED config, so never silently drop the edit and
        //never apply it without persisting it first.
        if (_profilerHwDirty) {
            QMessageBox box(this);
            box.setIcon(QMessageBox::Question);
            box.setWindowTitle(tr("Unsaved changes"));
            box.setText(tr("The profiler settings on this page have not been saved."));
            box.setInformativeText(tr("Connect applies the saved configuration."));

            QPushButton* saveBtn = box.addButton(tr("Save && Connect"), QMessageBox::AcceptRole);
            QPushButton* discardBtn = box.addButton(tr("Discard && Connect"), QMessageBox::DestructiveRole);
            box.addButton(QMessageBox::Cancel);
            box.setDefaultButton(saveBtn);
            box.exec();

            if (box.clickedButton() == saveBtn) {
                if (!saveProfilerHwToJson()) return;
                _profilerHwDirty = false;
                AuditLog::instance().log(QStringLiteral("PROFILER_CONFIG_SAVE"), profilerId());
            }
            else if (box.clickedButton() == discardBtn) {
                loadProfilerHwToUi();   //pull the saved values back over the edits
            }
            else {
                return;                 //cancel
            }
        }

        runProfilerConnect(true);
        });

    //Save only. Applying is Connect's job, so the heavyweight, connection-dropping action is
    //never hidden behind the word "Save".
    connect(ui.toolButton_profSave, &QToolButton::clicked, this, [=]() {
        if (!saveProfilerHwToJson()) return;
        _profilerHwDirty = false;
        AuditLog::instance().log(QStringLiteral("PROFILER_CONFIG_SAVE"), profilerId());
        refreshProfilerStatus();
        });

    loadProfilerHwToUi();

    //Connection state changes underneath this page (startup, JobThread, a failed scan), so poll
    //rather than trying to hook every path that could change it.
    _profStatusTimer = new QTimer(this);
    connect(_profStatusTimer, &QTimer::timeout, this, [=]() { refreshProfilerStatus(); });
    _profStatusTimer->start(1000);

    refreshProfilerStatus();
}

void Optics3DTab::markProfilerHwDirty()
{
    if (_profilerHwDirty) return;
    _profilerHwDirty = true;
    refreshProfilerStatus();
}

bool Optics3DTab::readProfilerEntry(QJsonObject& entry) const
{
    QJsonObject root;
    if (!readJsonObject(profilerJsonPath(), root)) return false;

    const QJsonArray arr = root.value("Profilers").toArray();
    if (arr.isEmpty()) {
        ct::logger::warn("[Optics3DTab/Prof] No Profilers array in %s", qPrintable(profilerJsonPath()));
        return false;
    }

    //Match the live id when there is one, else fall back to the first entry so the section still
    //populates on a machine where the profiler never connected.
    const QString wantId = profilerId();
    entry = arr.first().toObject();
    for (const auto& it : arr) {
        if (!it.isObject()) continue;
        if (it.toObject().value("ID").toString() == wantId) {
            entry = it.toObject();
            break;
        }
    }
    return true;
}

bool Optics3DTab::loadProfilerHwToUi()
{
    QJsonObject entry;
    if (!readProfilerEntry(entry)) return false;

    const QString wantId = profilerId();
    _profilerConfigFile = entry.value("Config_File").toString();

    {
        QSignalBlocker b(ui.lineEdit_profIP);
        ui.lineEdit_profIP->setText(entry.value("IP").toString());
    }

    //Every driver-config key is optional, so fall back to the driver's own built-in defaults.
    QJsonObject drv;
    const QString drvPath = driverConfigPath();
    if (drvPath.isEmpty() || !readJsonObject(drvPath, drv)) {
        ct::logger::warn("[Optics3DTab/Prof] Driver config not read (Config_File='%s'), showing defaults",
            qPrintable(_profilerConfigFile));
    }

    const int cmdPort = drv.value("commandPort").toInt(24691);
    const int hsPort = drv.value("highSpeedPort").toInt(24692);
    const int trig = drv.value("triggerMode").toInt(2);
    const int encMode = drv.value("encoderInputMode").toInt(1);
    const int encTime = drv.value("encoderMinInputTime").toInt(0);
    const double yPitch = drv.value("yPitchUm").toDouble(10.0);

    QSignalBlocker b1(ui.spinBox_profCmdPort);
    QSignalBlocker b2(ui.spinBox_profHsPort);
    QSignalBlocker b3(ui.comboBox_profTriggerMode);
    QSignalBlocker b4(ui.comboBox_profEncoderMode);
    QSignalBlocker b5(ui.comboBox_profEncoderMinTime);

    ui.spinBox_profCmdPort->setValue(cmdPort);
    ui.spinBox_profHsPort->setValue(hsPort);

    auto selectByData = [](QComboBox* cb, int code) {
        const int i = cb->findData(code);
        cb->setCurrentIndex(i < 0 ? 0 : i);
        };

    selectByData(ui.comboBox_profTriggerMode, trig);
    selectByData(ui.comboBox_profEncoderMode, encMode);
    selectByData(ui.comboBox_profEncoderMinTime, encTime);

    ui.label_profYPitch->setText(QString::number(yPitch, 'f', 2));

    //Widgets now equal the file by construction.
    _profilerHwDirty = false;
    ct::logger::info("[Optics3DTab/Prof] Loaded hw config: id=%s ip=%s cfg=%s ports=%d/%d trig=%d enc=%d/%d",
        qPrintable(wantId), qPrintable(ui.lineEdit_profIP->text()),
        qPrintable(_profilerConfigFile), cmdPort, hsPort, trig, encMode, encTime);
    return true;
}

bool Optics3DTab::saveProfilerHwToJson()
{
    const int cmdPort = ui.spinBox_profCmdPort->value();
    const int hsPort = ui.spinBox_profHsPort->value();

    //Manual p.69 forbids equal ports, and getting it wrong is misleading to diagnose:
    //EthernetOpen succeeds and InitializeHighSpeedSimpleArray then fails rc=0x1000, which reads
    //like a network fault but is not one. Reject it here as well as in the driver's loader.
    if (cmdPort == hsPort) {
        QMessageBox::warning(this, tr("Invalid ports"),
            tr("Command port and high-speed port must be different."));
        return false;
    }

    //The validator admits partial input as Intermediate, so "192.168.0" reaches here happily.
    //hasAcceptableInput() is the one that insists on a complete address.
    const QString ip = ui.lineEdit_profIP->text().trimmed();
    if (ip.isEmpty() || !ui.lineEdit_profIP->hasAcceptableInput()) {
        QMessageBox::warning(this, tr("Invalid IP"),
            tr("Enter a complete IPv4 address, for example 192.168.0.1."));
        return false;
    }

    const int trig = ui.comboBox_profTriggerMode->currentData().toInt();
    const int encMode = ui.comboBox_profEncoderMode->currentData().toInt();
    const int encTime = ui.comboBox_profEncoderMinTime->currentData().toInt();

    //--- Read and validate BOTH files before writing EITHER. profiler.json used to be written
    //--- first, so an unreadable driver config was only discovered afterwards - leaving the IP on
    //--- disk while the page still said "Unsaved changes", with no way to see which half landed.
    //--- profiler.json: IP only. Read-modify-write so API/Config_File/MSR/InvertX/InvertY and
    //--- any other profiler entry survive untouched.
    QJsonObject root;
    if (!readJsonObject(profilerJsonPath(), root)) return false;

    QJsonArray arr = root.value("Profilers").toArray();
    if (arr.isEmpty()) {
        ct::logger::warn("[Optics3DTab/Prof] Save aborted: no Profilers array");
        return false;
    }

    const QString wantId = profilerId();
    int target = 0;
    for (int i = 0; i < arr.size(); ++i) {
        if (!arr[i].isObject()) continue;
        if (arr[i].toObject().value("ID").toString() == wantId) {
            target = i;
            break;
        }
    }

    QJsonObject entry = arr[target].toObject();
    entry["IP"] = ip;
    arr[target] = entry;
    root["Profilers"] = arr;

    //--- driver config: ports, trigger, encoder. Read-modify-write keeps every other key,
    //--- including the _comment_* documentation and yPitchUm, which this page never writes.
    const QString drvPath = driverConfigPath();
    QJsonObject drv;
    bool haveDrv = false;

    if (drvPath.isEmpty()) {
        ct::logger::warn("[Optics3DTab/Prof] No Config_File in profiler.json, driver config not saved");
    }
    else if (!readJsonObject(drvPath, drv)) {
        return false;                       //nothing written yet, so nothing to unwind
    }
    else {
        drv["commandPort"] = cmdPort;
        drv["highSpeedPort"] = hsPort;
        drv["triggerMode"] = trig;
        drv["encoderInputMode"] = encMode;
        drv["encoderMinInputTime"] = encTime;
        haveDrv = true;
    }

    //--- every read done and every value validated: only now write.
    if (haveDrv && !writeJsonObject(drvPath, drv)) return false;
    if (!writeJsonObject(profilerJsonPath(), root)) return false;

    ct::logger::info("[Optics3DTab/Prof] Saved hw config: ip=%s ports=%d/%d trig=%d enc=%d/%d",
        qPrintable(ip), cmdPort, hsPort, trig, encMode, encTime);
    return true;
}

bool Optics3DTab::profilerBusy() const
{
    //JobThread drives the profiler during a scan and the gantry during a move. Tearing the
    //connection down underneath either of those is the one way this section can do damage.
    //Ask whether the id is known first - see refreshProfilerStatus() for why.
    const QString id = profilerId();
    if (ProfilerManager::instance().keys().contains(id)
        && ProfilerManager::instance().isGrabbing(id)) return true;
    if (SystemData::instance()._MotoIsMoving) return true;
    return false;
}

void Optics3DTab::refreshProfilerStatus()
{
    const QString id = profilerId();

    //profilerId() falls back to the literal "profiler1" when no profiler was ever created, and
    //that id is not in m_profilers - so isConnected() and isGrabbing() would each log
    //"[Profiler] Trying to access invalid profiler" through ProfilerManager::valid()
    //(ProfilerManager.cpp:464). This function is on a 1 Hz timer, so that is two lines every
    //second for as long as the page exists, in the very log used to diagnose the sensor. Ask the
    //cheap question locally instead of letting the manager answer it noisily.
    const bool known = ProfilerManager::instance().keys().contains(id);
    const bool connected = known && ProfilerManager::instance().isConnected(id);
    const bool grabbing = known && ProfilerManager::instance().isGrabbing(id);

    QString text;
    QString bg;
    if (_profilerBusyUi)  { text = tr("Working...");     bg = QStringLiteral("#B98900"); }
    else if (!known)      { text = tr("Not configured"); bg = QStringLiteral("#5A5A5A"); }
    else if (grabbing)    { text = tr("Scanning");       bg = QStringLiteral("#1565C0"); }
    else if (connected)   { text = tr("Connected");      bg = QStringLiteral("#2E7D32"); }
    else                  { text = tr("Disconnected");   bg = QStringLiteral("#8E1B1B"); }

    ui.label_profStatus->setText(text);
    ui.label_profStatus->setStyleSheet(
        QStringLiteral("color:white; background-color:%1; border-radius:6px; padding:3px;").arg(bg));

    const QString api = ProfilerManager::instance().getAPI();
    ui.label_profApi->setText(api.isEmpty() ? QStringLiteral("-") : api);

    if (connected) {
        ui.label_profSerial->setText(ProfilerManager::instance().getSerialNumber(id));
        ui.label_profFirmware->setText(ProfilerManager::instance().getFirmwareVersion(id));
        ui.label_profYRes->setText(
            QString::number(ProfilerManager::instance().getYResolution(id), 'f', 4));
    }
    else {
        ui.label_profSerial->setText(QStringLiteral("-"));
        ui.label_profFirmware->setText(QStringLiteral("-"));
        ui.label_profYRes->setText(QStringLiteral("-"));
    }

    ui.toolButton_profConnect->setText(connected ? tr("Disconnect") : tr("Connect"));

    //Two modes. Connected = monitor: the inputs are read-only, so what is displayed is what was
    //pushed to the controller. Disconnected = configure. These settings genuinely only apply at
    //connect, so the enablement state is the honest way to say so - more reliable than a message
    //the user has to read and remember.
    const bool configurable = !connected && !_profilerBusyUi && !grabbing;

    ui.lineEdit_profIP->setEnabled(configurable);
    ui.spinBox_profCmdPort->setEnabled(configurable);
    ui.spinBox_profHsPort->setEnabled(configurable);
    ui.comboBox_profTriggerMode->setEnabled(configurable);
    ui.comboBox_profEncoderMode->setEnabled(configurable);
    ui.comboBox_profEncoderMinTime->setEnabled(configurable);
    ui.toolButton_profSave->setEnabled(configurable);

    //Editing stays available with no profiler created - the fields are backed by JSON files, not
    //by the sensor, so preparing a machine offline is legitimate. Connecting is not: there is no
    //driver object to connect. Disable the button rather than let it fail down in the manager.
    ui.toolButton_profConnect->setEnabled(known && !_profilerBusyUi && !grabbing);

    QString msg;
    if (_profilerBusyUi)       msg = tr("Working...");
    else if (!known)           msg = tr("No profiler '%1' was created - check API and ID in profiler.json. "
                                        "Editing and saving still work.").arg(id);
    else if (grabbing)         msg = tr("Scan in progress.");
    else if (connected)        msg = tr("Connected - these values are running on the controller. Disconnect to edit.");
    else if (_profilerHwDirty) msg = tr("Unsaved changes - press Save.");
    else                       msg = tr("Disconnected. Edit, Save, then Connect to apply.");

    //This runs at 1 Hz, and QLineEdit::setText() clears the selection and drops the cursor at the
    //end of the string. Writing unconditionally would therefore make the field impossible to
    //select for copying, and would show the tail of a message too long for the widget. So write
    //only on an actual change, then rewind to the start. Tooltip carries the full text either way.
    if (ui.lineEdit_profMessage->text() != msg) {
        ui.lineEdit_profMessage->setText(msg);
        ui.lineEdit_profMessage->setCursorPosition(0);
        ui.lineEdit_profMessage->setToolTip(msg);
    }
}

void Optics3DTab::runProfilerConnect(bool wantConnected)
{
    if (profilerBusy()) {
        QMessageBox::warning(this, tr("Profiler busy"),
            tr("The profiler is acquiring, or the gantry is moving.\n"
               "Wait for it to finish before changing the connection."));
        return;
    }

    const QString id = profilerId();

    //Connect applies the SAVED configuration, never the widgets. That is what makes the
    //read-only display after a connect provably equal to what was pushed: the ports, trigger and
    //encoder settings reach the controller through loadConfig() reading this same file, so
    //sourcing the IP from anywhere else would make one field behave differently from the rest.
    QString ip;
    if (wantConnected) {
        QJsonObject entry;
        if (!readProfilerEntry(entry)) {
            QMessageBox::warning(this, tr("Profiler config missing"),
                tr("Could not read %1.").arg(profilerJsonPath()));
            return;
        }

        ip = entry.value("IP").toString().trimmed();
        _profilerConfigFile = entry.value("Config_File").toString();

        if (ip.isEmpty()) {
            QMessageBox::warning(this, tr("Profiler config invalid"),
                tr("No IP address saved for '%1'.").arg(id));
            return;
        }
    }

    //This blocks the GUI thread. A failed EthernetOpen sits out the SDK's internal timeout
    //(~10 s, and LJX8_IF.h exposes no way to shorten it), and a successful connect spends
    //roughly 9 s applying the driver config one setting at a time. Show a wait cursor and lock
    //the section rather than letting it look hung.
    _profilerBusyUi = true;
    refreshProfilerStatus();
    QApplication::setOverrideCursor(Qt::WaitCursor);

    bool ok = true;
    bool applied = true;

    //Safe when already disconnected: disconnect() guards its stop commands on the connection
    //status, so this does not sit out a timeout.
    ProfilerManager::instance().disconnect(id);

    if (wantConnected) {
        ok = ProfilerManager::instance().connect(id, ip);

        if (ok) {
            //Mirror ProfilerManager::loadConfig's startup order, or the reconnected sensor keeps
            //whatever settings the controller happened to be holding.
            const QString drvPath = driverConfigPath();
            if (!drvPath.isEmpty()) applied = ProfilerManager::instance().loadConfig(id, drvPath);

            if (!applied) {
                ct::logger::warn("[Optics3DTab/Prof] Connected, but driver config failed to apply: %s",
                    qPrintable(drvPath));
            }

            ProfilerManager::instance().setMSR(id, ProfilerManager::instance().getMSR());
        }
    }

    QApplication::restoreOverrideCursor();
    _profilerBusyUi = false;

    //Re-read from the file so the now read-only fields show exactly what was just pushed.
    if (wantConnected && ok) loadProfilerHwToUi();

    refreshProfilerStatus();

    AuditLog::instance().log(
        wantConnected ? QStringLiteral("PROFILER_CONNECT") : QStringLiteral("PROFILER_DISCONNECT"),
        id, ok ? QStringLiteral("OK") : QStringLiteral("FAILED"));

    ct::logger::info("[Optics3DTab/Prof] %s id=%s ip=%s -> %s",
        wantConnected ? "Connect" : "Disconnect",
        qPrintable(id), qPrintable(ip), ok ? "OK" : "FAILED");

    if (wantConnected && !ok) {
        QMessageBox::warning(this, tr("Profiler connect failed"),
            tr("Could not connect to '%1' at %2.\n\n%3")
            .arg(id, ip, ProfilerManager::instance().errorMsg(id)));
        return;
    }

    //The read-only display is only trustworthy if every setting was accepted. loadConfig ANDs 13
    //setSetting results together, so a single rejection means the page can be showing a value the
    //controller is not running - which is exactly the kind of number that ends up transcribed
    //into a constant. Say so rather than leaving it in the log.
    if (wantConnected && !applied) {
        QMessageBox::warning(this, tr("Settings not fully applied"),
            tr("Connected to '%1', but at least one driver setting was rejected.\n\n"
               "The values shown here may not all be running on the controller. "
               "Check the log for [Profiler_Keyence] setSetting failures before trusting them.")
            .arg(id));
    }
}