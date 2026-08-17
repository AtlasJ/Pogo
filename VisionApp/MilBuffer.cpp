#include "MilBuffer.h"
#include "Logger.h"

MilBuffer::MilBuffer()
{
}

MilBuffer::~MilBuffer()
{
}

bool MilBuffer::createBuffer(const QString& id, const int& width, const int& height, const int& channel, MIL_INT type)
{
	MIL_ID mBuf = M_NULL;
	MbufAllocColor(M_DEFAULT_HOST, channel, width, height, type, M_IMAGE + M_PROC + M_DISP, &mBuf);
	return (mBuf != M_NULL);
}

void MilBuffer::insertBuffer(const QString& id, MIL_ID buffer)
{
	if (m_buffers.contains(id)) {
		ct::logger::warn("Attempting to insert buffer into existing ID: %s", id.toStdString().c_str());
		freeBuffer(id);
	}

	ct::logger::trace("Insert buffer: %s, %ld", id.toStdString().c_str(), buffer);
	m_buffers.insert(id, buffer);
}

MIL_ID MilBuffer::getBuffer(const QString& id) const
{
	ct::logger::trace("Get buffer: %s, %ld", id.toStdString().c_str(), m_buffers[id]);
	if (m_buffers.contains(id)) return m_buffers[id];
	ct::logger::warn("Accessing invalid buffer: %s", id.toStdString().c_str());
	return M_NULL;
}

bool MilBuffer::freeBuffer(const QString& id)
{
	if (m_buffers.contains(id)) {
		ct::logger::trace("Free buffer: %s, %ld", id.toStdString().c_str(), m_buffers[id]);
		mtrx::free_buffer(m_buffers[id]);
		m_buffers.remove(id);
		return true;
	}
	else {
		ct::logger::warn("Invalid mil buffer ID to free: %s", id.toStdString().c_str());
	}

	return false;
}

int MilBuffer::size()
{
	return m_buffers.size();
}

void MilBuffer::clear()
{
	for (auto p : m_buffers) {
		mtrx::free_buffer(p);
	}

	m_buffers.clear();
}

bool MilBuffer::contains(const QString& id)
{
	return m_buffers.contains(id);
}

void MilBuffer::logRemainingBuffers()
{
	for (const auto& key : m_buffers.keys()) {
		ct::logger::debug("Remain buffer ID: %s", key.toStdString().c_str());
	}
}
