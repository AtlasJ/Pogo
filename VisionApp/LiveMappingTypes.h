#pragma once

#include <QChar>
#include <QString>

/*
* Live Mapping: the production page's grid of pogo block positions.
*
* The position number is the key everywhere. The selected Template Mapping says where each
* position sits on the grid; the loaded PMF says which positions this testhead actually has,
* and what is behind each one.
*/

//One position as the PMF describes it
struct PogoUnitInfo
{
	int position = 0;
	QString slot;
	QString board;
	QString description;
	QString cable;
	QChar ocr;        //the PMF column the position is listed under, A..P
};

enum class UnitState
{
	NotExist,         //on the template, but not in the PMF - nothing is fitted there
	Pending,          //in the PMF, not inspected yet
	Scanning,
	Pass,
	Fail
};

inline QString unitStateName(UnitState state)
{
	switch (state) {
	case UnitState::NotExist: return QStringLiteral("Not exist");
	case UnitState::Pending:  return QStringLiteral("Pending");
	case UnitState::Scanning: return QStringLiteral("Scanning");
	case UnitState::Pass:     return QStringLiteral("Pass");
	case UnitState::Fail:     return QStringLiteral("Fail");
	}
	return QString();
}
