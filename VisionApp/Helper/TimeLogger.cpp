#include "TimeLogger.h"
#include "Logger.h"

TimeLogger::TimeLogger()
{
	this->start = std::chrono::system_clock::now();
}

TimeLogger::~TimeLogger()
{
}

void TimeLogger::reset_timer()
{
	this->start = std::chrono::system_clock::now();
}

void TimeLogger::log_duration(std::string message, bool reset_timer)
{
	auto t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start).count();

	if (t > 1000) {
		ct::logger::warn("[duration over] %s: %lldms", message.c_str(), t);
	}
	else {
		ct::logger::debug("[duration] %s: %lldms", message.c_str(), t);
	}

	if (reset_timer) {
		this->reset_timer();
	}
}
