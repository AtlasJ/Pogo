#ifndef RESULTCACHE_H
#define RESULTCACHE_H

#include <QHash>
#include <QMutex>

class ResultCache
{
public:
	ResultCache();
	~ResultCache();

	void insertResult(const QString& resutID, const QByteArray& result);
	QByteArray getResult(const QString& resutID);
	void clear();

private:
	QHash<QString, QByteArray> _resultHash;
	QMutex _mutex;
};

#endif // RESULTCACHE_H
