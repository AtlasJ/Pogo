#pragma once
#include <chrono>
#include <string>

class ScopedTimeLogger {
private:
	std::chrono::time_point<std::chrono::system_clock> start;
	std::string message = "";

public:
	ScopedTimeLogger(std::string message);
	~ScopedTimeLogger();
};