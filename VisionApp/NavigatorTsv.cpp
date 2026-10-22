#include "NavigatorTsv.h"
#include "KeyenceSettings.h"

#include <QFile>
#include <QRegularExpression>
#include <cmath>

namespace {

    constexpr int kLabelColumns = 7;    //columns 1..7 are the tree path, column 8 the value

    //Navigator writes UTF-16 LE and repeats the byte-order mark (FF FE FF FE). fromUtf16 consumes
    //the first one, so strip whatever is left rather than assume there is exactly one.
    QString decodeText(const QByteArray& raw)
    {
        QString text;
        if (raw.size() >= 2 && uchar(raw[0]) == 0xFF && uchar(raw[1]) == 0xFE) {
            text = QString::fromUtf16(reinterpret_cast<const ushort*>(raw.constData()), raw.size() / 2);
        }
        else {
            text = QString::fromUtf8(raw);
        }
        while (text.startsWith(QChar(0xFEFF))) text.remove(0, 1);
        return text;
    }

    //"640<mu>s" / "1.7ms" / "120ns" -> lower case, no spaces, micro as 'u'. Navigator writes the
    //Greek mu (U+03BC); the micro sign (U+00B5) is accepted too in case another version uses it.
    QString normUnit(QString s)
    {
        s.replace(QChar(0x03BC), QLatin1Char('u'));
        s.replace(QChar(0x00B5), QLatin1Char('u'));
        s.remove(QLatin1Char(' '));
        return s.toLower();
    }

    bool parseMicroseconds(const QString& text, int* us)
    {
        static const QRegularExpression re(QStringLiteral("^([0-9]+(?:\\.[0-9]+)?)(ns|us|ms|s)?$"));
        const QRegularExpressionMatch m = re.match(normUnit(text));
        if (!m.hasMatch()) return false;

        double v = m.captured(1).toDouble();
        const QString unit = m.captured(2);
        if (unit == QLatin1String("ns"))      v /= 1000.0;
        else if (unit == QLatin1String("ms")) v *= 1000.0;
        else if (unit == QLatin1String("s"))  v *= 1000000.0;

        //The LJ-X tops out at 9.6 ms; anything near int range is a broken file, not an exposure.
        if (!(v >= 0.5 && v <= 1.0e7)) return false;
        *us = static_cast<int>(std::lround(v));
        return true;
    }

    bool parseInt(const QString& text, int lo, int hi, int* out)
    {
        bool ok = false;
        const int v = text.trimmed().toInt(&ok);
        if (!ok || v < lo || v > hi) return false;
        *out = v;
        return true;
    }

    bool isOn(const QString& v)  { return v.compare(QLatin1String("ON"), Qt::CaseInsensitive) == 0; }
    bool isOff(const QString& v) { return v.compare(QLatin1String("OFF"), Qt::CaseInsensitive) == 0; }

    //Navigator shows "---" for a setting that does not apply in the current mode, e.g. the encoder
    //details under Continuous trigger.
    bool notApplicable(const QString& v) { return v.isEmpty() || v == QLatin1String("---"); }

} // namespace

bool NavigatorTsv::load(const QString& filePath, QString* error)
{
    m_sections.clear();

    QFile f(filePath);
    if (!f.open(QIODevice::ReadOnly)) {
        if (error) *error = QStringLiteral("Cannot open %1").arg(filePath);
        return false;
    }
    //A Navigator export is about 90 KB. Refuse anything far bigger rather than read it whole on the
    //GUI thread - picking the wrong file must not stall the app.
    if (f.size() > 16 * 1024 * 1024) {
        if (error) *error = QStringLiteral("%1 is %2 MB - far too large to be an LJ-X Navigator settings file.")
            .arg(filePath).arg(f.size() / (1024 * 1024));
        return false;
    }
    const QString text = decodeText(f.readAll());
    f.close();

    QStringList context;
    for (QString line : text.split(QLatin1Char('\n'))) {
        if (line.endsWith(QLatin1Char('\r'))) line.chop(1);
        if (line.trimmed().isEmpty()) continue;

        const QStringList cols = line.split(QLatin1Char('\t'));
        if (!cols[0].trimmed().isEmpty()) {
            Section s;
            s.name = cols[0].trimmed();
            m_sections.append(s);
            context.clear();
        }
        if (m_sections.isEmpty()) continue;     //nothing before the first section header

        bool hasLabel = false;
        for (int i = 0; i < kLabelColumns && 1 + i < cols.size(); ++i) {
            const QString label = cols[1 + i].trimmed();
            if (label.isEmpty()) continue;
            context = context.mid(0, i);
            context << label;
            hasLabel = true;
        }
        if (!hasLabel) continue;                //a bare section header row

        Item item;
        item.labels = context;
        //Navigator marks some values as text with a leading apostrophe, spreadsheet style - "'1/2"
        //for a Z range, so Excel does not turn it into a date. The apostrophe is not the value.
        for (int c = 1 + kLabelColumns; c < cols.size(); ++c) {
            QString v = cols[c].trimmed();
            if (v.startsWith(QLatin1Char('\''))) v = v.mid(1).trimmed();
            item.values << v;
        }
        while (!item.values.isEmpty() && item.values.last().isEmpty()) item.values.removeLast();
        if (!item.values.isEmpty()) item.value = item.values.first();
        m_sections.last().items.append(item);
    }

    if (!common() || !program(0)) {
        if (error) *error = QStringLiteral("%1 is not an LJ-X Navigator settings file saved as TSV "
            "(no 'Common measurement setting' or 'Program0' section).").arg(filePath);
        m_sections.clear();
        return false;
    }
    return true;
}

const NavigatorTsv::Section* NavigatorTsv::common() const
{
    for (const Section& s : m_sections)
        if (s.name.startsWith(QLatin1String("Common"), Qt::CaseInsensitive)) return &s;
    return nullptr;
}

const NavigatorTsv::Section* NavigatorTsv::program(int n) const
{
    const QString name = QStringLiteral("Program%1").arg(n);
    for (const Section& s : m_sections)
        if (s.name.compare(name, Qt::CaseInsensitive) == 0) return &s;
    return nullptr;
}

bool NavigatorTsv::find(const Section* s, const QStringList& pathTail, QString* value, int nth, int column)
{
    if (!s || pathTail.isEmpty()) return false;

    int seen = 0;
    for (const Item& item : s->items) {
        const int offset = item.labels.size() - pathTail.size();
        if (offset < 0) continue;

        bool match = true;
        for (int k = 0; k < pathTail.size() && match; ++k)
            match = item.labels[offset + k].compare(pathTail[k], Qt::CaseInsensitive) == 0;

        if (match && seen++ == nth) {
            if (value) *value = column < item.values.size() ? item.values[column] : QString();
            return true;
        }
    }
    return false;
}

QVector<NavigatorSetting> mapNavigatorProgram(const NavigatorTsv& tsv, int programNo, QStringList* warnings)
{
    using S = NavigatorSetting;

    QVector<S> rows;
    QStringList warn;

    const NavigatorTsv::Section* prog = tsv.program(programNo);
    const NavigatorTsv::Section* common = tsv.common();
    if (!prog) {
        warn << QStringLiteral("The file has no Program%1 - the program Pogo runs (programNo in "
            "keyence.json).").arg(programNo);
        if (warnings) *warnings = warn;
        return rows;
    }

    auto get = [](const NavigatorTsv::Section* s, const QStringList& tail) {
        QString v;
        NavigatorTsv::find(s, tail, &v);
        return v;
    };

    auto add = [&rows](const QString& name, const QString& fileValue, S::Target target,
        bool usable, int value, const QString& note) {
        S s;
        s.name = name;
        s.fileValue = fileValue;
        s.target = target;
        s.usable = usable;
        s.value = value;
        s.note = note;
        rows.append(s);
    };

    auto notUnderstood = [](const QString& v) {
        return v.isEmpty() ? QStringLiteral("Not in the file") : QStringLiteral("Not understood - left unchanged");
    };

    //--- The selected 3D optic. Pushed to the controller before every scan (JobThread::scan), so
    //--- these are the values that actually run - anything sent to the controller from Navigator
    //--- is overwritten by them on the next scan.
    {
        const QString v = get(prog, { "Light intensity setting", "Exposure time" });
        int us = 0;
        const bool ok = parseMicroseconds(v, &us);
        add(QStringLiteral("Exposure time"), v, S::OpticExposure, ok, us, ok ? QString() : notUnderstood(v));
    }
    {
        const QString v = get(prog, { "Light receiving property", "CMOS Dynamic Range" });
        int dr = 0;
        const bool ok = parseInt(v, 1, 9, &dr);
        add(QStringLiteral("CMOS Dynamic Range"), v, S::OpticDynamicRange, ok, dr, ok ? QString() : notUnderstood(v));
    }
    {
        //One Pogo number becomes two controller settings: divider 1 is skipping OFF, not
        //"skip 1" - the controller's count only accepts 2..1000.
        const QString skip = get(prog, { "Encoder trigger detail", "Skipping" });
        const QString points = get(prog, { "Encoder trigger detail", "Points of skipping" });
        if (isOn(skip)) {
            int n = 0;
            const bool ok = parseInt(points, 2, 1000, &n);
            add(QStringLiteral("Skipping"), QStringLiteral("ON, %1 points").arg(points), S::OpticDivider,
                ok, n, ok ? QString() : QStringLiteral("Points of skipping not understood - left unchanged"));
        }
        else if (isOff(skip)) {
            add(QStringLiteral("Skipping"), skip, S::OpticDivider, true, 1, QStringLiteral("Divider 1 = skipping off"));
        }
        else {
            add(QStringLiteral("Skipping"), skip, S::OpticDivider, false, 0,
                notApplicable(skip) ? QStringLiteral("Only set under Encoder trigger - left unchanged") : notUnderstood(skip));
        }
    }
    {
        const QString v = get(prog, { "Exposure mode", "Exposure mode" });
        const bool ok = v.compare(QLatin1String("Standard"), Qt::CaseInsensitive) == 0;
        add(QStringLiteral("Exposure mode"), v, S::OpticExposureMode, ok, 0,
            ok ? QString() : QStringLiteral("Pogo runs the LJ-X in Standard (single) exposure only - left unchanged"));
    }
    {
        const QString v = get(prog, { "Peak process", "Detection Sensitivity" });
        int n = 0;
        const bool ok = parseInt(v, 1, 5, &n);
        add(QStringLiteral("Detection Sensitivity"), v, S::OpticPeakSensitivity, ok, n, ok ? QString() : notUnderstood(v));
    }
    {
        const QString v = get(prog, { "Multiple peak processing", "Detection Mode" });
        const QString lower = v.toLower();
        int sel = -1;
        if (lower.startsWith(QLatin1String("standard"))) sel = 0;
        else if (lower.startsWith(QLatin1String("near"))) sel = 1;
        else if (lower.startsWith(QLatin1String("far"))) sel = 2;
        add(QStringLiteral("Detection Mode"), v, S::OpticPeakSelection, sel >= 0, sel,
            sel >= 0 ? QString() : (v.isEmpty() ? notUnderstood(v)
                : QStringLiteral("Pogo's Peak Selection offers Standard, Near and Far only - left unchanged")));
    }
    {
        const QString v = get(prog, { "Control range", "Upper limit" });
        int n = 0;
        const bool ok = parseInt(v, 1, 99, &n);
        add(QStringLiteral("Light upper limit"), v, S::OpticLaserUpper, ok, n, ok ? QString() : notUnderstood(v));
    }
    {
        const QString v = get(prog, { "Control range", "Lower limit" });
        int n = 0;
        const bool ok = parseInt(v, 1, 99, &n);
        add(QStringLiteral("Light lower limit"), v, S::OpticLaserLower, ok, n, ok ? QString() : notUnderstood(v));
    }

    //--- Machine level: keyence.json, sent once at Connect.
    {
        //Same codes as comboBox_profEncoderMode: 0 1-phase, 1/2/3 = 2-phase x1/x2/x4.
        const QString v = get(prog, { "Encoder trigger detail", "Input mode" });
        const QString lower = v.toLower();
        int mode = -1;
        if (lower.startsWith(QLatin1String("1-phase"))) {
            mode = 0;
        }
        else {
            static const QRegularExpression re(QStringLiteral("^2-phase\\s*(\\d+)"));
            const QRegularExpressionMatch m = re.match(lower);
            if (m.hasMatch()) {
                const int times = m.captured(1).toInt();
                mode = times == 1 ? 1 : times == 2 ? 2 : times == 4 ? 3 : -1;
            }
        }
        add(QStringLiteral("Encoder input mode"), v, S::DrvEncoderInputMode, mode >= 0, mode,
            mode >= 0 ? QStringLiteral("Scales every scan - must match the encoder wiring")
                : (notApplicable(v) ? QStringLiteral("Only set under Encoder trigger - left unchanged") : notUnderstood(v)));
    }
    {
        //Same order as comboBox_profEncoderMinTime (Optics3DTab::initProfilerHwUi).
        static const char* const kTimes[] = {
            "120ns", "150ns", "250ns", "500ns", "1us", "2us", "5us", "10us", "20us" };
        const QString v = get(common, { "Minimum input time", "ENCODER" });
        const QString norm = normUnit(v);
        int idx = -1;
        for (int i = 0; i < 9; ++i)
            if (norm == QLatin1String(kTimes[i])) idx = i;
        add(QStringLiteral("Encoder minimum input time"), v, S::DrvEncoderMinInputTime, idx >= 0, idx,
            idx >= 0 ? QString() : notUnderstood(v));
    }
    {
        const QString v = get(prog, { "Control light intensity", "Control mode" });
        int mode = -1;
        if (v.compare(QLatin1String("MANUAL"), Qt::CaseInsensitive) == 0) mode = 1;
        else if (v.compare(QLatin1String("AUTO"), Qt::CaseInsensitive) == 0) mode = 0;
        //Manual p.5-9: Auto is Manual with the limits forced to 1..99, so the optic's limits
        //stop doing anything.
        add(QStringLiteral("Light control mode"), v, S::DrvLightControlMode, mode >= 0, mode,
            mode == 0 ? QStringLiteral("In Auto the controller ignores the optic's upper/lower limits")
                : mode == 1 ? QString() : notUnderstood(v));
    }
    {
        const QString v = get(prog, { "Peak process", "Intrpol pts inval data" });
        int n = 0;
        const bool ok = parseInt(v, 0, 255, &n);
        add(QStringLiteral("Interpolated points of invalid data"), v, S::DrvInvalidInterpolation, ok, n,
            ok ? QString() : notUnderstood(v));
    }

    //--- Shown for information. Pogo either sets these itself or never touches them.
    const QString head = get(common, { "Head sel" });
    if (!head.startsWith(QLatin1String("LJ-X8060"), Qt::CaseInsensitive)) {
        warn << QStringLiteral("The file is for a %1 head. Pogo's X pitch (5.0 um), laser FOV (16 mm) "
            "and Z scale are set for the LJ-X8060.").arg(head.isEmpty() ? QStringLiteral("unknown") : head);
    }
    {
        const QString v = get(prog, { "Trigger mode", "Trigger mode" });
        add(QStringLiteral("Trigger mode"), v, S::None, false, 0, QStringLiteral("Pogo always scans on Encoder trigger"));
        if (!v.startsWith(QLatin1String("Encoder"), Qt::CaseInsensitive)) {
            warn << QStringLiteral("Program%1 uses '%2', not Encoder trigger - this may not be the program "
                "you tuned. Pogo keeps Encoder trigger regardless.").arg(programNo).arg(v);
        }
    }
    {
        const QString v = get(prog, { "Batch setting", "Batch measurement" });
        add(QStringLiteral("Batch measurement"), v, S::None, false, 0, QStringLiteral("Pogo keeps batch measurement ON"));
        if (!isOn(v)) {
            warn << QStringLiteral("Batch measurement is '%1' in this program. Pogo needs it ON and "
                "keeps it ON regardless.").arg(v);
        }
    }
    add(QStringLiteral("Batch point"), get(prog, { "Batch setting", "Batch point" }), S::None, false, 0,
        QStringLiteral("Pogo sets this from the scan length on every scan"));
    {
        const QString v = get(common, { "Luminance output" });
        add(QStringLiteral("Luminance output"), v, S::None, false, 0, QStringLiteral("Pogo turns it on for every scan"));
    }

    //--- Everything else, from the KeyenceSettings table: what Pogo carries per optic or in
    //--- keyence.json, and what it only checks or shows. One table, so the import can never store a
    //--- setting under a different meaning from the one the driver sends.
    for (const ljx::Field& f : ljx::fields()) {
        if (f.tsv.isEmpty()) continue;

        const NavigatorTsv::Section* sec = f.group == QLatin1String("Common") ? common : prog;
        QString text;
        NavigatorTsv::find(sec, f.tsv.split(QLatin1Char('|')), &text, f.tsvNth, f.tsvColumn);
        int code = 0;
        const bool parsed = ljx::parse(f, text, &code);

        if (f.level == ljx::Level::Optic || f.level == ljx::Level::Machine) {
            add(f.name, text, f.level == ljx::Level::Optic ? S::OpticLjx : S::DrvLjx, parsed, code,
                parsed ? f.note
                    : notApplicable(text) ? QStringLiteral("Not set in this program - left unchanged") : notUnderstood(text));
        }
        else if (f.level == ljx::Level::Check) {
            add(f.name, text, S::None, false, 0, f.note);
            if (parsed && code != f.required) {
                warn << QStringLiteral("%1 is '%2' - Pogo works with '%3'. %4")
                    .arg(f.name, text, ljx::display(f, f.required), f.note);
            }
        }
        else {
            add(f.name, text, S::None, false, 0, f.note.isEmpty() ? QStringLiteral("Shown only - left to Navigator") : f.note);
        }
        rows.last().key = f.key;
    }

    if (warnings) *warnings = warn;
    return rows;
}
