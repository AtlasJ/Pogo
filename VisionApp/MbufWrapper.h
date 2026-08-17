#pragma once

#include <mil.h>
#include <memory>

namespace mtrx {
    class MbufWrapper;

    using SharedMilID = std::shared_ptr<MbufWrapper>;

    class MbufWrapper
    {
    public:
        explicit MbufWrapper(MIL_ID id = M_NULL)
            : m_id(id) {}

        MIL_ID id() const { return m_id; }
        operator MIL_ID() const { return m_id; }

    private:
        MIL_ID m_id = M_NULL;
    };
}