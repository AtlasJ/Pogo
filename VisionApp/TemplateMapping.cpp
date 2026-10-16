#include "TemplateMapping.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QRegularExpression>
#include <QSaveFile>
#include <algorithm>
#include <cmath>

namespace {

QVector<QVector<std::optional<int>>> emptyGrid(int xCount, int yCount)
{
	return QVector<QVector<std::optional<int>>>(yCount, QVector<std::optional<int>>(xCount));
}

bool sameName(const QString& a, const QString& b)
{
	return QString::compare(a.trimmed(), b.trimmed(), Qt::CaseInsensitive) == 0;
}

//"map-007" -> 7, anything else -> 0
int idNumber(const QString& id)
{
	static const QRegularExpression re(QStringLiteral("^map-(\\d+)$"));
	const QRegularExpressionMatch m = re.match(id);
	return m.hasMatch() ? m.captured(1).toInt() : 0;
}

} // namespace

QString TemplateMappingStore::filePath()
{
	return QStringLiteral("C:/Advanced/Data/config/template_mapping.json");
}

QString TemplateMappingStore::valueText(int value)
{
	return QStringLiteral("%1").arg(value, 3, 10, QLatin1Char('0'));
}

bool TemplateMappingStore::load(QString* message)
{
	_mappings.clear();
	_nextId = 1;
	_blocked = false;

	QFile file(filePath());
	if (!file.exists()) {
		QString error;
		if (save(&error)) return true;
		if (message) *message = QStringLiteral("Could not create %1: %2").arg(filePath(), error);
		return false;
	}
	if (!file.open(QIODevice::ReadOnly)) {
		_blocked = true;
		if (message) *message = QStringLiteral("Could not open %1: %2").arg(filePath(), file.errorString());
		return false;
	}

	QJsonParseError parseError;
	const QJsonDocument doc = QJsonDocument::fromJson(file.readAll(), &parseError);
	file.close();

	if (!doc.isObject()) {
		//keep whatever is there for whoever wants it back, and start empty
		const QString aside = filePath() + QStringLiteral(".unreadable");
		QFile::remove(aside);
		if (!QFile::rename(filePath(), aside)) {
			_blocked = true;
			if (message) *message = QStringLiteral("%1 is unreadable (%2) and could not be moved aside, "
				"so it is left untouched and nothing can be saved.").arg(filePath(), parseError.errorString());
			return false;
		}
		if (message) *message = QStringLiteral("%1 was unreadable (%2); it was moved to %3 and the mappings start empty.")
			.arg(filePath(), parseError.errorString(), aside);
		save(nullptr);
		return false;
	}

	const QJsonObject root = doc.object();
	int skipped = 0;
	int highestId = 0;
	for (const QJsonValue& entry : root.value(QStringLiteral("mappings")).toArray()) {
		const QJsonObject o = entry.toObject();
		TemplateMapping m;
		m.id = o.value(QStringLiteral("id")).toString();
		m.name = o.value(QStringLiteral("name")).toString().trimmed();
		m.xCount = o.value(QStringLiteral("xCount")).toInt();
		m.yCount = o.value(QStringLiteral("yCount")).toInt();

		//one bad entry should not cost the others
		if (m.id.isEmpty() || m.name.isEmpty() || indexOfId(m.id) >= 0 || indexOfName(m.name) >= 0
			|| m.xCount < kMinCount || m.xCount > kMaxCount || m.yCount < kMinCount || m.yCount > kMaxCount) {
			skipped++;
			continue;
		}

		m.values = emptyGrid(m.xCount, m.yCount);
		const QJsonArray rows = o.value(QStringLiteral("values")).toArray();
		for (int y = 0; y < m.yCount && y < rows.size(); y++) {
			const QJsonArray row = rows.at(y).toArray();
			for (int x = 0; x < m.xCount && x < row.size(); x++) {
				const QJsonValue cell = row.at(x);
				if (!cell.isDouble()) continue;
				const double v = cell.toDouble();
				if (v == std::floor(v) && v >= 0 && v <= kMaxValue) m.values[y][x] = int(v);
			}
		}

		highestId = std::max(highestId, idNumber(m.id));
		_mappings.append(m);
	}
	_nextId = std::max(root.value(QStringLiteral("nextId")).toInt(1), highestId + 1);

	if (skipped > 0) {
		//the next save drops what was skipped, so keep the file as it was
		const QString backup = filePath() + QStringLiteral(".bak");
		QFile::remove(backup);
		QFile::copy(filePath(), backup);
		if (message) *message = QStringLiteral("%1 mapping(s) in %2 were incomplete or duplicated and were skipped "
			"(the file as it was is kept in %3).").arg(skipped).arg(filePath(), backup);
	}
	return true;
}

bool TemplateMappingStore::save(QString* error) const
{
	if (_blocked) {
		if (error) *error = QStringLiteral("the existing file could not be read, so it is not overwritten");
		return false;
	}

	QJsonArray list;
	for (const TemplateMapping& m : _mappings) {
		QJsonArray rows;
		for (const auto& row : m.values) {
			QJsonArray cells;
			for (const auto& cell : row) cells.append(cell ? QJsonValue(*cell) : QJsonValue(QJsonValue::Null));
			rows.append(cells);
		}
		QJsonObject o;
		o.insert(QStringLiteral("id"), m.id);
		o.insert(QStringLiteral("name"), m.name);
		o.insert(QStringLiteral("xCount"), m.xCount);
		o.insert(QStringLiteral("yCount"), m.yCount);
		o.insert(QStringLiteral("values"), rows);
		list.append(o);
	}
	QJsonObject root;
	root.insert(QStringLiteral("nextId"), _nextId);
	root.insert(QStringLiteral("mappings"), list);

	QDir().mkpath(QFileInfo(filePath()).absolutePath());
	QSaveFile out(filePath());   //written beside the real file and swapped in, so a crash mid-write leaves the old one
	if (!out.open(QIODevice::WriteOnly)) {
		if (error) *error = out.errorString();
		return false;
	}
	out.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
	if (!out.commit()) {
		if (error) *error = out.errorString();
		return false;
	}
	return true;
}

int TemplateMappingStore::indexOfId(const QString& id) const
{
	for (int i = 0; i < _mappings.size(); i++)
		if (_mappings[i].id == id) return i;
	return -1;
}

int TemplateMappingStore::indexOfName(const QString& name, const QString& ignoreId) const
{
	for (int i = 0; i < _mappings.size(); i++)
		if (_mappings[i].id != ignoreId && sameName(_mappings[i].name, name)) return i;
	return -1;
}

QString TemplateMappingStore::nameProblem(const QString& name, const QString& ignoreId) const
{
	if (name.trimmed().isEmpty()) return QStringLiteral("The name cannot be empty.");
	const int other = indexOfName(name, ignoreId);
	if (other >= 0) return QStringLiteral("A mapping called \"%1\" already exists.").arg(_mappings[other].name);
	return QString();
}

int TemplateMappingStore::add(const QString& name)
{
	TemplateMapping m;
	m.id = QStringLiteral("map-%1").arg(_nextId++, 3, 10, QLatin1Char('0'));
	m.name = name.trimmed();
	m.xCount = 1;
	m.yCount = 1;
	m.values = emptyGrid(m.xCount, m.yCount);
	m.values[0][0] = 0;
	_mappings.append(m);
	return _mappings.size() - 1;
}

void TemplateMappingStore::rename(int index, const QString& name)
{
	if (index >= 0 && index < _mappings.size()) _mappings[index].name = name.trimmed();
}

void TemplateMappingStore::remove(int index)
{
	if (index >= 0 && index < _mappings.size()) _mappings.removeAt(index);
}

void TemplateMappingStore::setGrid(int index, int xCount, int yCount, const QVector<QVector<std::optional<int>>>& values)
{
	if (index < 0 || index >= _mappings.size()) return;
	TemplateMapping& m = _mappings[index];
	m.xCount = std::clamp(xCount, kMinCount, kMaxCount);
	m.yCount = std::clamp(yCount, kMinCount, kMaxCount);
	m.values = emptyGrid(m.xCount, m.yCount);
	for (int y = 0; y < m.yCount && y < values.size(); y++)
		for (int x = 0; x < m.xCount && x < values[y].size(); x++)
			m.values[y][x] = values[y][x];
}
