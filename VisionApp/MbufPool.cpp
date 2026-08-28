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

    //m_mutex is held here and is not recursive - the unlocked form is mandatory.
    if (!is_idle_unlocked()) return nullptr;

    MIL_ID id = m_idle.front();
    m_idle.pop();

    if (m_dtypeMap[id] == PoolDestructorType::POOL_IDLE) {
        return std::shared_ptr<MbufWrapper>(new MbufWrapper(id), MbufPoolIdleSetter{ shared_from_this() });
    }
    else if (m_dtypeMap[id] == PoolDestructorType::FREE_BUFFER) {
        return std::shared_ptr<MbufWrapper>(new MbufWrapper(id), MbufPoolReleaser{ shared_from_this() });
    }

    //Falling off the end of a function returning shared_ptr is undefined behaviour, and the
    //id has already been popped, so the buffer would be lost as well. Callers all null-check.
    ct::logger::error("[BufferPool] Buffer %d has no destructor type recorded - not handing it out", id);
    m_idle.push(id);
    return nullptr;
}

std::shared_ptr<MbufWrapper> mtrx::MbufPool::attach(MIL_ID id, PoolDestructorType destructorType)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    if (id == M_NULL) {
        ct::logger::error("[BufferPool] Trying to attach a NULL ID");
        return nullptr;
    }
    
    //Covers POOL_IDLE ids; FREE_BUFFER ones are never in m_buffers, hence m_attachedFree below.
    if (m_buffers.contains(id) || m_attachedFree.count(id)) {
        ct::logger::error("[BufferPool] Trying to attach existing ID: %d", id);
        return nullptr;
    }

    m_dtypeMap[id] = destructorType;
    if (destructorType == PoolDestructorType::POOL_IDLE) m_buffers.insert(id);
    else m_attachedFree.insert(id);

    if (m_dtypeMap[id] == PoolDestructorType::POOL_IDLE) {
        return std::shared_ptr<MbufWrapper>(new MbufWrapper(id), MbufPoolIdleSetter{ shared_from_this() });
    }
    else if (m_dtypeMap[id] == PoolDestructorType::FREE_BUFFER) {
        return std::shared_ptr<MbufWrapper>(new MbufWrapper(id), MbufPoolReleaser{ shared_from_this() });
    }

    //Unreachable today - both enum values are covered - but falling off the end of a function
    //returning shared_ptr is undefined behaviour, not an empty pointer.
    ct::logger::error("[BufferPool] Unknown destructor type for attached ID %d", id);
    m_dtypeMap.erase(id);
    m_buffers.erase(id);
    return nullptr;
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

/*
* Locked. This used to read m_idle with no lock at all, while set_idle() and free_buffer()
* push and pop it under m_mutex - and those two are called from shared_ptr deleters, which run
* on whichever thread happens to drop the last reference to a buffer (the Keyence callback
* thread, ImageManager, or JobThread). Reading a std::queue concurrently with push/pop is a
* data race, so the answer could be stale or torn.
*
* MbufPoolManager::acquire() asks this question and then decides whether to allocate another
* buffer, so a wrong answer either wastes a buffer or hands back a null one.
*/
bool MbufPool::is_idle() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return is_idle_unlocked();
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
    m_attachedFree.erase(id);   //this id is dead; MIL may hand the same number out again
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