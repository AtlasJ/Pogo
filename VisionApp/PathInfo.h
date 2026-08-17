#pragma once
#include <QString>
#include <QVector>
#include <QSet>

struct PathInfo {
	QString startID = "";
	QString endID = "";
	QSet<QString> includedViews;
	QVector<QString> paths;
};