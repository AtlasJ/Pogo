#pragma once

#include <QString>
#include <QVector>
#include <QJsonObject>

#include "AlgoSetupTypes.h"

/*
* In-app replacement for the Algo library's AlgoGraph/AlgoGraphList template
* metadata. A template is now a lightweight record that links a vision object
* to one of the AlgoManager algos (OCR inspection / 3D height measurement)
* instead of an external algo node graph.
*
* The JSON format keeps the legacy templateList.json structure and key names
* ({"TemplateList": [{templateName, templateId, uniformBox, color,
* templateImagePath, ...}]}) so templates from existing recipes keep their
* identity; the algo node data is simply ignored and a new "algo" key links
* the template to an AlgoManager algorithm.
*/
class AlgoTemplate {
public:
	QString templateName() const { return _templateName; }
	void templateName(const QString& name) { _templateName = name; }

	QString templateId() const { return _templateId; }
	void templateId(const QString& id) { _templateId = id; }

	QString color() const { return _color; }
	void color(QString color) { _color = color; }

	bool uniformBox() const { return _uniformBox; }
	void uniformBox(bool uniformBox) { _uniformBox = uniformBox; }

	QString templateImagePath() const { return _templateImagePath; }
	void templateImagePath(const QString& path) { _templateImagePath = path; }

	int w() const { return _w; }
	void w(int w) { _w = w; }

	int h() const { return _h; }
	void h(int h) { _h = h; }

	//which AlgoManager algorithm this template runs
	AlgoPageAlgo algo() const { return _algo; }
	void algo(AlgoPageAlgo algo) { _algo = algo; }

	void getGraphObject(QJsonObject& obj) const;
	void setGraphObject(const QJsonObject& obj);

private:
	QString _templateName = "Default";
	QString _templateId = "Default";
	QString _templateImagePath = "";
	QString _color = "0000000";
	bool _uniformBox = true;
	int _w = 0;
	int _h = 0;
	AlgoPageAlgo _algo = AlgoPageAlgo::OCR_READ;
};

class AlgoTemplateList {
public:
	~AlgoTemplateList();

	bool saveAlgoTemplateList(const QString& fileName);
	bool loadAlgoTemplateList(const QString& fileName);
	void releaseAlgoTemplateList();
	bool addAlgoTemplate(const QString& templateName, QString color);
	void copyAlgoTemplate(QString srcTemplateName, QString destTemplateName);
	void deleteAlgoTemplate(const QString& templateName);
	bool algoTemplateExist(const QString& templateName);
	QVector<AlgoTemplate*> getAlgoTemplateList();

private:
	QVector<AlgoTemplate*> _algoTemplateList;
};
