#include <iostream>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <opencv2/opencv.hpp>
#include <direct.h>
#include "utils.h"

//void Timer::start() 
//{
//    start_time = std::chrono::steady_clock::now();
//}
//
//void Timer::reset() 
//{
//    start();
//}
//
//void Timer::printDuration() 
//{
//    auto now = std::chrono::steady_clock::now();
//    double durationMs = std::chrono::duration<double, std::milli>(now - start_time).count();
//    std::cout << "Elapsed time: " << std::fixed << std::setprecision(4)
//        << durationMs << " ms" << std::endl;
//}

bool fileExists(const std::string& filePath) 
{
    std::ifstream file(filePath);
    return file.good();
}

std::string dateTimeStr() 
{
    // Get the current time point
    auto now = std::chrono::system_clock::now();

    // Convert to time_t to extract the seconds component for formatting
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::tm* now_tm = std::localtime(&now_c);

    // Format the time as YYYYMMDDHHMMSS (up to seconds)
    std::ostringstream oss;
    oss << std::put_time(now_tm, "%Y%m%d%H%M%S");

    return oss.str();
}

// Function to convert type to a readable string
std::string type2str(const int type) 
{
    std::string r;
    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);

    switch (depth) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
    }

    r += "C";
    r += std::to_string(chans);
    return r;
}

bool createDirectory(const std::string& directoryPath) {
    return _mkdir(directoryPath.c_str()) == 0;
}