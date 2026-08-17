#pragma once
#include <string>

// Timer class declaration.
//class Timer {
//private:
//    std::chrono::time_point<std::chrono::steady_clock> start_time;
//public:
//    // Starts the timer.
//    void start();
//
//    // Resets the timer.
//    void reset();
//
//    // Prints the elapsed time in milliseconds with 4 decimal places.
//    void printDuration();
//};

bool fileExists(const std::string& filePath);
std::string dateTimeStr();
std::string type2str(const int type);
bool createDirectory(const std::string& directoryPath);