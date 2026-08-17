#include "MbufPoolManager.h"
#include "Logger.h"

using namespace mtrx;

MbufPoolManager& MbufPoolManager::instance()
{
    static MbufPoolManager inst;
    return inst;
}

void MbufPoolManager::create_pool(int width, int height, int channel, long long type, int count, PoolDestructorType dType)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    auto key = get_key(width, height, channel, type);
    if (m_pools.contains(key)) {
        ct::logger::warn("Buffer Pool already created for format: %dx%dx%d, %d", width, height, channel, type);
        return;
    }

    m_pools[key] = std::make_shared<MbufPool>(width, height, channel, type, count, dType);

    ct::logger::info("[BufferPool] Created %d of size: %dx%dx%d, type: %d, dtype: %d", count, width, height, channel, type, dType);
}

std::shared_ptr<MbufWrapper> MbufPoolManager::acquire(int width, int height, int channel, long long type)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    auto key = get_key(width, height, channel, type);

    //if no pool available, create a self free pool
    if (!m_pools.contains(key)) {
        m_pools[key] = std::make_shared<MbufPool>(width, height, channel, type, 1, PoolDestructorType::FREE_BUFFER);
        ct::logger::info("[BufferPool] Requested format not available, created a new pool: %s", key.c_str());
    }
    
    auto it = m_pools.find(key);

    if (!it->second->is_idle()) {
        it->second->add_buffer(1, PoolDestructorType::FREE_BUFFER);
        ct::logger::info("[BufferPool] No idle pool, added new buffer: %s", key.c_str());
    }

    auto buf = it->second->acquire();

    if (buf) {
        ct::logger::info("[BufferPool] Acquire: %s, %d", key.c_str(), buf->id());
    }
    else {
        ct::logger::error("[BufferPool] Acquire return nullptr");
    }

    return buf;
}

std::shared_ptr<MbufWrapper> mtrx::MbufPoolManager::attach(MIL_ID id, PoolDestructorType destructorType)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    auto width = mtrx::get_width(id);
    auto height = mtrx::get_height(id);
    auto channel = mtrx::get_band(id);
    auto type = mtrx::get_type(id);

    auto key = get_key(width, height, channel, type);

    //if no pool available, create the object with no count
    if (!m_pools.contains(key)) {
        m_pools[key] = std::make_shared<MbufPool>(width, height, channel, type, 0, PoolDestructorType::POOL_IDLE);
        ct::logger::info("[BufferPool] Requested format not available, created a new pool: %s", key.c_str());
    }

    auto it = m_pools.find(key);

    auto buf = it->second->attach(id, destructorType);

    if (buf) {
        ct::logger::info("[BufferPool] Attach: %s, %d", key.c_str(), buf->id());
    }
    else {
        ct::logger::error("[BufferPool] Attach return nullptr");
    }

    return buf;
}

bool MbufPoolManager::is_idle(int width, int height, int channel, long long type)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    auto key = get_key(width, height, channel, type);

    auto it = m_pools.find(key);
    return it->second->is_idle();
}

void MbufPoolManager::release_pools()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_pools.clear();
}

MbufPoolManager::MbufPoolManager()
{
}

MbufPoolManager::~MbufPoolManager()
{
    release_pools();
}

std::string MbufPoolManager::get_key(int width, int height, int channel, long long type)
{
    return std::to_string(width) + "_" + std::to_string(height) + "_" + std::to_string(channel) + "_" + std::to_string(type);
}