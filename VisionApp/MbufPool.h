#pragma once
#include <mil.h>
#include <memory>
#include <queue>
#include <set>
#include <mutex>
#include <unordered_map>

#include "MbufWrapper.h"

namespace mtrx {
    // Forward declaration
    class MbufPool;

    enum class PoolDestructorType {
        POOL_IDLE, FREE_BUFFER
    };

    // Custom destructor: returns buffer to pool
    struct MbufPoolIdleSetter
    {
        std::shared_ptr<MbufPool> pool;

        void operator()(MbufWrapper* buf) const;
    };

    // Custom destructor: frees MIL buffer
    struct MbufPoolReleaser
    {
        std::shared_ptr<MbufPool> pool;

        void operator()(MbufWrapper* buf) const;
    };

    class MbufPool : public std::enable_shared_from_this<MbufPool>
    {
    public:
        MbufPool(int width, int height, int channel, long long type, int count, PoolDestructorType destructorType);
        ~MbufPool();


        std::shared_ptr<MbufWrapper> acquire();
        std::shared_ptr<MbufWrapper> attach(MIL_ID id, PoolDestructorType destructorType);
        void add_buffer(int count, PoolDestructorType destructorType);
        void set_idle(MIL_ID id);

        int width() const { return m_width; }
        int height() const { return m_height; }
        int channel() const { return m_channel; }
        long long type() const { return m_type; }

        bool is_idle() const;

        void free_buffer(MIL_ID id);
        void release();

    private:
        int m_width = 0;
        int m_height = 0;
        int m_channel = 0;
        long long m_type = 0;

        std::mutex m_mutex;
        std::queue<MIL_ID> m_idle;
        std::set<MIL_ID> m_buffers;
        std::unordered_map<MIL_ID, PoolDestructorType> m_dtypeMap;

        void alloc(int width, int height, int channel, long long type, PoolDestructorType destructorType);
    };
}