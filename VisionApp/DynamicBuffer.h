#ifndef DYNAMICBUFFER_H
#define DYNAMICBUFFER_H

#include <QHash>

enum class DataType {
	UNSIGNED_CHAR, UNSIGNED_SHORT
};

class DynamicBuffer
{
public:
	DynamicBuffer();
	~DynamicBuffer();

	bool createBuffer(const QString& id, const int& width, const int& height, const int& channel, DataType type);
	unsigned char* getUnsignedChar(const QString& id) const;
	unsigned short* getUnsignedShort(const QString& id) const;
	bool freeBuffer(const QString& id);
	int size();
	void clear();
	bool contains(const QString& id);

private:
	QHash<QString, unsigned char*> m_unsignedChar;
	QHash<QString, unsigned short*> m_unsignedShort;

	void free(unsigned char*);
	void free(unsigned short*);
};

#endif // BUFFERQUE_H
