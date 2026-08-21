#include "AlgoTemplate.h"
#include "QJsonHelper.h"
#include "Logger.h"

#include <QJsonDocument>
#include <QJsonArray>
#include <QFile>
#include <chrono>

void AlgoTemplate::getGraphObject(QJsonObject& obj) const
{
	//legacy AlgoGraph key names, so old and new builds can read the same file
	obj.insert(QStringLiteral("templateName"), _templateName);
	obj.insert(QStringLiteral("templateId"), _templateId);
	obj.insert(QStringLiteral("uniformBox"), _uniformBox);
	obj.insert(QStringLiteral("color"), _color);
	obj.insert(QStringLiteral("templateImagePath"), _templateImagePath);
	obj.insert(QStringLiteral("w"), _w);
	obj.insert(QStringLiteral("h"), _h);
	obj.insert(QStringLiteral("algo"), (int)_algo); //AlgoManager algorithm link
}

void AlgoTemplate::setGraphObject(const QJsonObject& obj)
{
	_templateId = jsonHelper::getString(obj, QStringLiteral("templateId"));
	_templateName = jsonHelper::getString(obj, QStringLiteral("templateName"));
	_uniformBox = jsonHelper::getBool(obj, QStringLiteral("uniformBox"), true);
	_color = jsonHelper::getString(obj, QStringLiteral("color"));
	_templateImagePath = jsonHelper::getString(obj, QStringLiteral("templateImagePath"));
	_w = jsonHelper::getInteger(obj, QStringLiteral("w"), 0);
	_h = jsonHelper::getInteger(obj, QStringLiteral("h"), 0);

	const int algo = jsonHelper::getInteger(obj, QStringLiteral("algo"), 0);
	_algo = (algo == (int)AlgoPageAlgo::HEIGHT_3D) ? AlgoPageAlgo::HEIGHT_3D : AlgoPageAlgo::OCR_READ;
}

AlgoTemplateList::~AlgoTemplateList()
{
	releaseAlgoTemplateList();
}

bool AlgoTemplateList::saveAlgoTemplateList(const QString& fileName)
{
	QJsonObject root;
	QJsonArray templateArray;

	for (const auto tmpl : _algoTemplateList) {
		QJsonObject obj;
		tmpl->getGraphObject(obj);
		templateArray.append(obj);
	}

	root.insert(QStringLiteral("TemplateList"), templateArray);

	QFile file(fileName);
	if (!file.open(QIODevice::WriteOnly)) {
		ct::logger::error("[AlgoTemplate] Failed to save %s", fileName.toStdString().c_str());
		return false;
	}

	file.write(QJsonDocument(root).toJson());
	file.close();
	return true;
}

bool AlgoTemplateList::loadAlgoTemplateList(const QString& fileName)
{
	releaseAlgoTemplateList();

	QFile file(fileName);
	if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return false;

	const QJsonObject root = QJsonDocument::fromJson(file.readAll()).object();
	file.close();

	for (const auto& v : jsonHelper::getArray(root, QStringLiteral("TemplateList"))) {
		auto tmpl = new AlgoTemplate();
		tmpl->setGraphObject(v.toObject());
		_algoTemplateList.append(tmpl);
	}

	return true;
}

void AlgoTemplateList::releaseAlgoTemplateList()
{
	qDeleteAll(_algoTemplateList);
	_algoTemplateList.clear();
}

bool AlgoTemplateList::addAlgoTemplate(const QString& templateName, QString color)
{
	if (algoTemplateExist(templateName)) return false;

	auto tmpl = new AlgoTemplate();

	//same id scheme as the legacy AlgoGraphList
	const uint64_t us = std::chrono::duration_cast<std::chrono::microseconds>(
		std::chrono::system_clock::now().time_since_epoch()).count();
	tmpl->templateId(QStringLiteral("Template_") + QString::number(us));
	tmpl->templateName(templateName);
	tmpl->uniformBox(true);
	tmpl->color(color);

	_algoTemplateList.append(tmpl);
	return true;
}

void AlgoTemplateList::copyAlgoTemplate(QString srcTemplateName, QString destTemplateName)
{
	AlgoTemplate* src = nullptr;
	AlgoTemplate* dest = nullptr;

	for (auto tmpl : _algoTemplateList) {
		if (tmpl->templateName() == srcTemplateName) src = tmpl;
		if (tmpl->templateName() == destTemplateName) dest = tmpl;
	}
	if (!src || !dest) return;

	//copy everything except identity
	dest->uniformBox(src->uniformBox());
	dest->templateImagePath(src->templateImagePath());
	dest->w(src->w());
	dest->h(src->h());
	dest->algo(src->algo());
}

void AlgoTemplateList::deleteAlgoTemplate(const QString& templateName)
{
	for (int i = 0; i < _algoTemplateList.size(); i++) {
		if (_algoTemplateList[i]->templateName() == templateName) {
			delete _algoTemplateList[i];
			_algoTemplateList.removeAt(i);
			return;
		}
	}
}

bool AlgoTemplateList::algoTemplateExist(const QString& templateName)
{
	for (const auto tmpl : _algoTemplateList) {
		if (tmpl->templateName() == templateName) return true;
	}
	return false;
}

QVector<AlgoTemplate*> AlgoTemplateList::getAlgoTemplateList()
{
	return _algoTemplateList;
}
