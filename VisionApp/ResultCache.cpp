#include "ResultCache.h"

ResultCache::ResultCache()
{
}

void ResultCache::insertResult(const QString& resutID, const QByteArray& result)
{
	_mutex.lock();
	_resultHash.insert(resutID, result);
	_mutex.unlock();
}

QByteArray ResultCache::getResult(const QString& resutID)
{
	_mutex.lock();
	QByteArray result = _resultHash.take(resutID);
	_mutex.unlock();

	return result;
}

void ResultCache::clear()
{
	_mutex.lock();
	_resultHash.clear();
	_mutex.unlock();
}

ResultCache::~ResultCache()
{
	clear();
}