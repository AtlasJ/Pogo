#pragma once

#include <string>
#include <queue>
#include <mutex>
#include <unordered_map>
#include "mil.h"
#include "mtrx.h"

#include "MbufWrapper.h"
#include "MbufPool.h"

namespace mtrx {

    class MbufPoolManager;

    using MPM = MbufPoolManager;

    class MbufPoolManager
    {
    public:
        static MbufPoolManager& instance();

        void create_pool(int width, int height, int channel, long long type, int count, PoolDestructorType destructorType);

        std::shared_ptr<MbufWrapper> acquire(int width, int height, int channel, long long type);

        std::shared_ptr<MbufWrapper> attach(MIL_ID id, PoolDestructorType destructorType = PoolDestructorType::FREE_BUFFER);

        bool is_idle(int width, int height, int channel, long long type);

        void release_pools();

        //modes


    private:
        MbufPoolManager();
        ~MbufPoolManager();

        std::string get_key(int width, int height, int channel, long long type);

        std::mutex m_mutex;
        std::unordered_map<std::string, std::shared_ptr<MbufPool>> m_pools;
    };
}

/*
=> To support few modes
Every mode should take note on RAM usage
Dynamic: Auto allocate buffer based on usage, only free buffers when over certain RAM usage
User Preset: User set the specific number of buffers, only free buffers when over certain RAM usage
Prioritize Memory: Always free once buffer is not in used
Fixed buffer: Will block thread if no available buffer
*/