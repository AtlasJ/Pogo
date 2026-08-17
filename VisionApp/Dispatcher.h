#pragma once

#include <unordered_map>
#include <vector>
#include <mutex>
#include <functional>
#include <typeindex>
#include <memory>
#include <algorithm>

namespace nvs {

    class Dispatcher {
    public:
        // Singleton pattern
        static Dispatcher& instance() {
            static Dispatcher inst;
            return inst;
        }

        template<typename EventT>
        int subscribe(std::function<void(const EventT&)> fn)
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            int handlerID = ++m_lastID;

            auto& vec = m_handlers[typeid(EventT)];
            vec.push_back(HandlerEntry{
                handlerID,
                [fn](const void* e) {
                    fn(*static_cast<const EventT*>(e));
                }
                });

            return handlerID;
        }

        template<typename EventT>
        void unsubscribe(int handlerID)
        {
            std::lock_guard<std::mutex> lock(m_mutex);

            auto it = m_handlers.find(typeid(EventT));
            if (it == m_handlers.end()) return;

            auto& vec = it->second;
            vec.erase(std::remove_if(vec.begin(), vec.end(),
                [&](const HandlerEntry& h) { return h.id == handlerID; }),
                vec.end());
        }

        template<typename EventT>
        void dispatch(const EventT& event)
        {
            std::vector<HandlerEntry> temp;

            {
                std::lock_guard<std::mutex> lock(m_mutex);
                auto it = m_handlers.find(typeid(EventT));
                if (it == m_handlers.end()) return;
                temp = it->second;  // copy handler list
            }

            // Call handlers outside the lock
            for (auto& h : temp)
                h.fn(&event);
        }

    private:
        Dispatcher() = default;
        ~Dispatcher() = default;
        Dispatcher(const Dispatcher&) = delete;
        Dispatcher& operator=(const Dispatcher&) = delete;

        struct HandlerEntry {
            int id;
            std::function<void(const void*)> fn;
        };

        std::mutex m_mutex;
        int m_lastID = 0;

        // key = typeid(EventT)
        std::unordered_map<std::type_index, std::vector<HandlerEntry>> m_handlers;
    };

}