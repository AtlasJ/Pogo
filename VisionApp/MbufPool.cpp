#include "MbufPool.h"
#include "mtrx.h"
#include "Logger.h"

using namespace mtrx;

//=> custom destructor
void MbufPoolIdleSetter::operator()(MbufWrapper* buf) const
{
    if (!buf) {
        ct::logger::error("[BufferPool] Failed to access buffer, received nullptr");
        return;
    }

    if (!pool) {
        ct::logger::error("[BufferPool] Failed to access pool for buffer %d, received nullptr", buf->id());
        return;
    }

    pool->set_idle(buf->id());
    delete buf;
}

void MbufPoolReleaser::operator()(MbufWrapper* buf) const
{
    if (!buf) {
        ct::logger::error("[BufferPool] Failed to free buffer, received nullptr");
        return;
    }

    if (!pool) {
        ct::logger::error("[BufferPool] Failed to access pool for buffer %d, received nullptr", buf->id());
        return;
    }


    auto id = buf->id();
    pool->free_buffer(id);
    delete buf;
    ct::logger::info("[BufferPool] Buffer is freed: %d", id);
}


//=> MbufPool
MbufPool::MbufPool(int width, int height, int channel, long long type, int count, PoolDestructorType destructorType)
    : m_width(width), m_height(height), m_channel(channel), m_type(type)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    for (int i = 0; i < count; i++) {
        alloc(width, height, channel, type, destructorType);
    }
}

MbufPool::~MbufPool()
{
    // Free all MIL buffers upon shutdown
    m_idle = {};

    for (auto buf : m_buffers) {
        mtrx::free_buffer(buf);
    }

    ct::logger::info("[BufferPool] Freeing pool");
}

std::shared_ptr<MbufWrapper> MbufPool::acquire()
{
    std::lock_guard<std::mutex> lock(m_mutex);

    if (!is_idle()) return nullptr;

    MIL_ID id = m_idle.front();
    m_idle.pop();

    if (m_dtypeMap[id] == PoolDestructorType::POOL_IDLE) {
        return std::shared_ptr<MbufWrapper>(new MbufWrapper(id), MbufPoolIdleSetter{ shared_from_this() });
    }
    else if (m_dtypeMap[id] == PoolDestructorType::FREE_BUFFER) {
        return std::shared_ptr<MbufWrapper>(new MbufWrapper(id), MbufPoolReleaser{ shared_from_this() });
    }
}

std::shared_ptr<MbufWrapper> mtrx::MbufPool::attach(MIL_ID id, PoolDestructorType destructorType)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    if (id == M_NULL) {
        ct::logger::error("[BufferPool] Trying to attach a NULL ID");
        return nullptr;
    }
    
    if (m_buffers.contains(id)) {
        ct::logger::error("[BufferPool] Trying to attach existing ID: %d", id);
        return nullptr;
    }

    m_dtypeMap[id] = destructorType;
    if (destructorType == PoolDestructorType::POOL_IDLE) m_buffers.insert(id);

    if (m_dtypeMap[id] == PoolDestructorType::POOL_IDLE) {
        return std::shared_ptr<MbufWrapper>(new MbufWrapper(id), MbufPoolIdleSetter{ shared_from_this() });
    }
    else if (m_dtypeMap[id] == PoolDestructorType::FREE_BUFFER) {
        return std::shared_ptr<MbufWrapper>(new MbufWrapper(id), MbufPoolReleaser{ shared_from_this() });
    }
}

void MbufPool::add_buffer(int count, PoolDestructorType destructorType)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    for (int i = 0; i < count; i++) {
        alloc(width(), height(), channel(), type(), destructorType);
    }
}

void MbufPool::set_idle(MIL_ID id)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_idle.push(id);
    ct::logger::info("[BufferPool] Buffer is now idle: %d", id);
}

bool MbufPool::is_idle() const
{
    return !m_idle.empty();
}

void MbufPool::alloc(int width, int height, int channel, long long type, PoolDestructorType destructorType)
{
    MIL_ID id;

    if (channel == 1) MbufAlloc2d(M_DEFAULT, width, height, type, M_IMAGE + M_PROC, &id);
    else if (channel == 3) MbufAllocColor(M_DEFAULT, channel, width, height, type, M_IMAGE + M_PROC, &id);
    else ct::logger::error("[BufferPool] Invalid channel: %d", channel);

    m_idle.push(id);
    m_dtypeMap[id] = destructorType;
    if (destructorType == PoolDestructorType::POOL_IDLE) m_buffers.insert(id);
}

void mtrx::MbufPool::free_buffer(MIL_ID id)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    mtrx::free_buffer(id);
    m_buffers.erase(id);
    m_dtypeMap.erase(id);
}

void MbufPool::release()
{
    std::lock_guard<std::mutex> lock(m_mutex);

    m_idle = {};
    m_dtypeMap.clear();

    for (auto buf : m_buffers) {
        mtrx::free_buffer(buf);
    }
}