#ifndef MILBUFFER_H
#define MILBUFFER_H

#include <QHash>
#include "mtrx.h"

class MilBuffer
{
public:
	MilBuffer();
	~MilBuffer();

	bool createBuffer(const QString& id, const int& width, const int& height, const int& channel, MIL_INT type);
	void insertBuffer(const QString& id, MIL_ID buffer);
	MIL_ID getBuffer(const QString& id) const;
	bool freeBuffer(const QString& id);
	int size();
	void clear();
	bool contains(const QString& id);
	void logRemainingBuffers();

private:
	QHash<QString, MIL_ID> m_buffers;
};

#endif // BUFFERQUE_H
