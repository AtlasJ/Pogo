#pragma once
#include <chrono>
#include <string>

class TimeLogger {
private:
	std::chrono::time_point<std::chrono::system_clock> start;

public:
	TimeLogger();
	~TimeLogger();
	void reset_timer();
	void log_duration(std::string message, bool reset_timer = true);
};

/*
Description:
This class uses logger as a way to log the duration taken for certain tasks.
The moment the object is created, the timer will start. 
User can call log_duration() with first parameter as the message to be log for tracking purposes,
second paramater is for resetting the timer after the msg logged. This reduce the code from being
bloated by needing to call reset_timer() right after calling log_duration().

How to use:
TimeLogger timer;
//doing task1
timer.log_duration("Task1", true); //Duration = Task1
//doing task2
timer.log_duration("Task2"); //Duration = Task2
//doing task3
timer.log_duration("Task2 + Task3"); //Duration = Task2 and Task3 because the timer was not reset
*/