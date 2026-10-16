#pragma once

#include <QString>
#include <QVector>
#include <optional>

/*
* Template Mapping: a named X x Y grid of whole numbers.
*
* The mappings are app-wide, not per recipe: one file, C:/Advanced/Data/config/template_mapping.json,
* next to the other machine configs. The store holds them in memory and writes the whole file
* on every change; nothing edits the file cell by cell.
*
*   {"nextId": 3,
*    "mappings": [{"id": "map-001", "name": "Input Pins", "xCount": 3, "yCount": 2,
*                  "values": [[10, 20, 30], [40, null, 60]]}]}
*
* values is [y][x] - the outer array is the rows - and an empty cell is null. nextId is kept in
* the file so a removed mapping's id is never handed out again.
*/
struct TemplateMapping
{
	QString id;                                  //permanent, survives renames, never reused
	QString name;                                //unique ignoring case and surrounding spaces
	int xCount = 1;
	int yCount = 1;
	QVector<QVector<std::optional<int>>> values; //[y][x], std::nullopt = empty cell
};

class TemplateMappingStore
{
public:
	static constexpr int kMinCount = 1;
	static constexpr int kMaxCount = 100;
	static constexpr int kMaxValue = 999999;     //cells hold 0..kMaxValue

	static QString filePath();
	static QString valueText(int value);         //three digits, like the PMF's pogo positions: 0 -> "000", 7 -> "007"

	//A missing file is the first-run case: no mappings, and an empty file is written so the
	//setup has somewhere to save. An unreadable file is moved aside rather than overwritten.
	//Returns false only when something needs the user's attention; message says what.
	bool load(QString* message);
	bool save(QString* error) const;

	const QVector<TemplateMapping>& mappings() const { return _mappings; }
	int indexOfId(const QString& id) const;
	int indexOfName(const QString& name, const QString& ignoreId = QString()) const;
	QString nameProblem(const QString& name, const QString& ignoreId = QString()) const; //empty when the name is fine

	int add(const QString& name);                //1 x 1 holding 0 (shown as 000); returns its index
	void rename(int index, const QString& name);
	void remove(int index);
	void setGrid(int index, int xCount, int yCount, const QVector<QVector<std::optional<int>>>& values);

private:
	QVector<TemplateMapping> _mappings;
	int _nextId = 1;
	bool _blocked = false;                       //the file on disk could not be read or moved aside: never overwrite it
};
