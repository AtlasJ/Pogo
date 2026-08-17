#include "ScopedTimeLogger.h"
#include "Logger.h"

ScopedTimeLogger::ScopedTimeLogger(std::string message)
{
	this->message = message;
	this->start = std::chrono::system_clock::now();
	ct::logger::trace("[duration] Start tracking: %s", message.c_str());
}

ScopedTimeLogger::~ScopedTimeLogger()
{
	auto t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start).count();
	ct::logger::debug("[duration] %s: %lldms", message.c_str(), t);
}
