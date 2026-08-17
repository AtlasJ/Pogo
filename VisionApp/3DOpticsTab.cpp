#include "3DOpticsTab.h"
#include "CommonDir.h"
#include "Logger.h"
#include "AuditLog.h"



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