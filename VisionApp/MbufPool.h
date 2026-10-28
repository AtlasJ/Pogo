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
        unsigned long long generation = 0; //which hand-out of this MIL_ID the wrapper belongs to - see MbufPool::m_generation

        void operator()(MbufWrapper* buf) const;
    };

    // Custom destructor: frees MIL buffer
    struct MbufPoolReleaser
    {
        std::shared_ptr<MbufPool> pool;
        unsigned long long generation = 0;

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
        void set_idle(MIL_ID id, unsigned long long generation);

        int width() const { return m_width; }
        int height() const { return m_height; }
        int channel() const { return m_channel; }
        long long type() const { return m_type; }

        bool is_idle() const;

        bool free_buffer(MIL_ID id, unsigned long long generation); //false: a stale wrapper, nothing freed
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

        /*
        * Which hand-out of each MIL_ID is the current one. MIL recycles ids - and wraps after
        * about 131k of them - so an id this pool still lists can come back attached to a NEW
        * buffer when the old one was freed outside the pool. attach() then replaces the stale
        * entry, and the generation is how a wrapper still holding the old entry is told apart
        * from the new one: its deleter must neither free nor idle the new buffer.
        */
        std::unordered_map<MIL_ID, unsigned long long> m_generation;
        unsigned long long m_nextGeneration = 1;

        //Callers that already hold m_mutex must use this one. m_mutex is not recursive, so
        //calling the public is_idle() from acquire() would self-deadlock.
        bool is_idle_unlocked() const { return !m_idle.empty(); }

        //all of these: m_mutex held
        unsigned long long new_generation_unlocked(MIL_ID id);
        bool is_current_unlocked(MIL_ID id, unsigned long long generation) const;
        void forget_stale_unlocked(MIL_ID id);

        void alloc(int width, int height, int channel, long long type, PoolDestructorType destructorType);
    };
}