#include "DynamicBuffer.h"

DynamicBuffer::DynamicBuffer()
{
}

DynamicBuffer::~DynamicBuffer()
{
}

bool DynamicBuffer::createBuffer(const QString& id, const int& width, const int& height, const int& channel, DataType type)
{
	bool flag = true;

	if (type == DataType::UNSIGNED_CHAR) {
		if (m_unsignedChar.contains(id)) {
			free(m_unsignedChar[id]);
			m_unsignedChar.remove(id);
		}

		unsigned char* pBuf = new unsigned char[width * height * channel];
		m_unsignedChar.insert(id, pBuf);
	}
	else if (type == DataType::UNSIGNED_SHORT) {
		if (m_unsignedShort.contains(id)) {
			free(m_unsignedShort[id]);
			m_unsignedShort.remove(id);
		}

		unsigned short* pBuf = new unsigned short[width * height * channel];
		m_unsignedShort.insert(id, pBuf);
	}

	return flag;
}

unsigned char* DynamicBuffer::getUnsignedChar(const QString& id) const
{
	if (m_unsignedChar.contains(id)) return m_unsignedChar[id];
	return nullptr;
}

unsigned short* DynamicBuffer::getUnsignedShort(const QString& id) const
{
	if (m_unsignedShort.contains(id)) return m_unsignedShort[id];
	return nullptr;
}

bool DynamicBuffer::freeBuffer(const QString& id)
{
	if (m_unsignedChar.contains(id)) {
		free(m_unsignedChar[id]);
		m_unsignedChar.remove(id);
		return true;
	}
	else if (m_unsignedShort.contains(id)) {
		free(m_unsignedShort[id]);
		m_unsignedShort.remove(id);
		return true;
	}

	return false;
}

int DynamicBuffer::size()
{
	return m_unsignedChar.size() + m_unsignedShort.size();
}

void DynamicBuffer::clear()
{
	for (auto p : m_unsignedChar) {
		free(p);
	}

	for (auto p : m_unsignedShort) {
		free(p);
	}

	m_unsignedChar.clear();
	m_unsignedShort.clear();
}

bool DynamicBuffer::contains(const QString& id)
{
	if (m_unsignedChar.contains(id)) return true;
	if (m_unsignedShort.contains(id)) return true;
	return false;
}

void DynamicBuffer::free(unsigned char* p)
{
	if (p != nullptr) {
		delete[] p;
		p = nullptr;
	}
}

void DynamicBuffer::free(unsigned short* p)
{
	if (p != nullptr) {
		delete[] p;
		p = nullptr;
	}
}
