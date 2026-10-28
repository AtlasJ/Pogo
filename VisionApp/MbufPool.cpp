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

    pool->set_idle(buf->id(), generation);
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
    const bool freed = pool->free_buffer(id, generation);
    delete buf;
    if (freed) ct::logger::info("[BufferPool] Buffer is freed: %d", id);
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

    //every id that reaches m_idle got a generation from alloc() or attach()
    const unsigned long long gen = m_generation.count(id) ? m_generation[id] : new_generation_unlocked(id);

    if (m_dtypeMap[id] == PoolDestructorType::POOL_IDLE) {
        return std::shared_ptr<MbufWrapper>(new MbufWrapper(id), MbufPoolIdleSetter{ shared_from_this(), gen });
    }
    else if (m_dtypeMap[id] == PoolDestructorType::FREE_BUFFER) {
        return std::shared_ptr<MbufWrapper>(new MbufWrapper(id), MbufPoolReleaser{ shared_from_this(), gen });
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
    
    /*
    * An id this pool still lists. MIL never hands out the number of a LIVE buffer, and every
    * caller attaches a buffer it has just allocated (bayer_to_rgb, crop, rotate, restore, stitch
    * results...), so this entry is STALE: the buffer it described was freed outside the pool and
    * MIL has since reused the number - which it does for good once its counter wraps, after about
    * 131k ids. Refusing returned nullptr, the frame went on with no image, and the UI thread
    * dereferenced it (2026-10-27 IPC crash, 130806 -> 290 in two frames). The stale entry is
    * dropped instead; the new generation below keeps a wrapper still holding the old entry from
    * freeing or idling this buffer when it is released.
    * Covers POOL_IDLE ids (m_buffers) and FREE_BUFFER ones (m_attachedFree).
    */
    if (m_buffers.count(id) || m_attachedFree.count(id)) {
        ct::logger::warn("[BufferPool] ID %d came back from MIL while still listed - the old entry is stale (freed outside the pool), replacing it", id);
        forget_stale_unlocked(id);
    }

    m_dtypeMap[id] = destructorType;
    if (destructorType == PoolDestructorType::POOL_IDLE) m_buffers.insert(id);
    else m_attachedFree.insert(id);
    const unsigned long long gen = new_generation_unlocked(id);

    if (m_dtypeMap[id] == PoolDestructorType::POOL_IDLE) {
        return std::shared_ptr<MbufWrapper>(new MbufWrapper(id), MbufPoolIdleSetter{ shared_from_this(), gen });
    }
    else if (m_dtypeMap[id] == PoolDestructorType::FREE_BUFFER) {
        return std::shared_ptr<MbufWrapper>(new MbufWrapper(id), MbufPoolReleaser{ shared_from_this(), gen });
    }

    //Unreachable today - both enum values are covered - but falling off the end of a function
    //returning shared_ptr is undefined behaviour, not an empty pointer.
    ct::logger::error("[BufferPool] Unknown destructor type for attached ID %d", id);
    m_dtypeMap.erase(id);
    m_buffers.erase(id);
    m_attachedFree.erase(id);
    m_generation.erase(id);
    return nullptr;
}

unsigned long long MbufPool::new_generation_unlocked(MIL_ID id)
{
    const unsigned long long gen = m_nextGeneration++;
    m_generation[id] = gen;
    return gen;
}

bool MbufPool::is_current_unlocked(MIL_ID id, unsigned long long generation) const
{
    const auto it = m_generation.find(id);
    return it != m_generation.end() && it->second == generation;
}

//the old buffer is already gone in MIL: only the bookkeeping is dropped, nothing is freed
void MbufPool::forget_stale_unlocked(MIL_ID id)
{
    m_buffers.erase(id);
    m_attachedFree.erase(id);
    m_dtypeMap.erase(id);
    m_generation.erase(id);

    //a stale POOL_IDLE id waiting in m_idle would be handed out by acquire() as if it were free
    std::queue<MIL_ID> keep;
    while (!m_idle.empty()) {
        if (m_idle.front() != id) keep.push(m_idle.front());
        m_idle.pop();
    }
    m_idle.swap(keep);
}

void MbufPool::add_buffer(int count, PoolDestructorType destructorType)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    for (int i = 0; i < count; i++) {
        alloc(width(), height(), channel(), type(), destructorType);
    }
}

void MbufPool::set_idle(MIL_ID id, unsigned long long generation)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    //a wrapper whose entry was replaced: the id now belongs to a newer buffer, which is not idle
    if (!is_current_unlocked(id, generation)) {
        ct::logger::warn("[BufferPool] Not idling %d: stale wrapper (generation %llu) - the id belongs to a newer buffer", id, generation);
        return;
    }

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
    MIL_ID id = M_NULL; //stays M_NULL for an invalid channel count instead of pushing garbage

    if (channel == 1) MbufAlloc2d(M_DEFAULT, width, height, type, M_IMAGE + M_PROC, &id);
    else if (channel == 3) MbufAllocColor(M_DEFAULT, channel, width, height, type, M_IMAGE + M_PROC, &id);
    else ct::logger::error("[BufferPool] Invalid channel: %d", channel);

    if (id == M_NULL) return; //failed allocation: nothing to hand out

    //a fresh allocation cannot collide with a live id - anything listed under it is stale
    if (m_buffers.count(id) || m_attachedFree.count(id) || m_generation.count(id)) {
        ct::logger::warn("[BufferPool] Allocated ID %d is still listed - the old entry is stale, replacing it", id);
        forget_stale_unlocked(id);
    }

    m_idle.push(id);
    m_dtypeMap[id] = destructorType;
    if (destructorType == PoolDestructorType::POOL_IDLE) m_buffers.insert(id);
    new_generation_unlocked(id);
}

bool mtrx::MbufPool::free_buffer(MIL_ID id, unsigned long long generation)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    /*
    * A wrapper whose entry attach() replaced after MIL reused its number: the buffer it described
    * is already gone, and the id now names a NEWER buffer someone is using. Freeing here would pull
    * that buffer out from under its owner, so nothing is freed.
    */
    if (!is_current_unlocked(id, generation)) {
        ct::logger::warn("[BufferPool] Not freeing %d: stale wrapper (generation %llu) - the id belongs to a newer buffer", id, generation);
        return false;
    }

    mtrx::free_buffer(id);
    m_buffers.erase(id);
    m_attachedFree.erase(id);   //this id is dead; MIL may hand the same number out again
    m_dtypeMap.erase(id);
    m_generation.erase(id);
    return true;
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