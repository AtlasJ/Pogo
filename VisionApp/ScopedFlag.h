#pragma once

class ScopedFlag {
private:
	bool* m_flag = nullptr;

public:
	ScopedFlag(bool* flag) {
		m_flag = flag;
	}

	~ScopedFlag() {
		*m_flag = !(*m_flag);
	}
};