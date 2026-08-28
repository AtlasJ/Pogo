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

        //Mutable so the public is_idle() can lock: the buffer wrappers' deleters run on
        //whatever thread drops the last reference - the profiler callback, ImageManager or
        //JobThread - so set_idle() and free_buffer() mutate this pool from arbitrary threads.
        mutable std::mutex m_mutex;
        std::queue<MIL_ID> m_idle;
        std::set<MIL_ID> m_buffers;

        /*
        * Ids attached with FREE_BUFFER, tracked ONLY so attach() can reject a duplicate.
        * m_buffers cannot do that job: alloc() and attach() add to it for POOL_IDLE only, so
        * attach()'s "already attached" check never fires for a FREE_BUFFER id. Attach the same
        * MIL_ID twice and you get two wrappers, each carrying a MbufPoolReleaser, and MbufFree
        * runs twice on one buffer - reachable in a long session because MIL recycles ids after
        * a free. Deliberately a separate set: putting these into m_buffers would also make the
        * destructor free buffers a live shared_ptr still points at.
        */
        std::set<MIL_ID> m_attachedFree;

        std::unordered_map<MIL_ID, PoolDestructorType> m_dtypeMap;

        //Callers that already hold m_mutex must use this one. m_mutex is not recursive, so
        //calling the public is_idle() from acquire() would self-deadlock.
        bool is_idle_unlocked() const { return !m_idle.empty(); }

        void alloc(int width, int height, int channel, long long type, PoolDestructorType destructorType);
    };
}