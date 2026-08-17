#pragma once
#include <iostream>
#include <string>
#include <vector>
#include "utils.h"

enum class Channel {
    RED,
    GREEN,
    BLUE,
    GRAY
};

struct InputParam {
    std::string folderPath = "";
    std::string fileExtension = "";
    Channel channel = Channel::GRAY;
    uint16_t start = 0;
    uint16_t end = 0;
    uint16_t step = 1;
    int width = 0;
    int height = 0;
    cv::Size size = cv::Size(0, 0);
    uint16_t imageCount = 0;
    //uint16_t imageRange = 0;
    float umPerZStep = 50.0;
    float umPerXPixel = 2.5;
    float umPerYPixel = 2.5;
    bool inputFromMemory = false;
    const std::vector<cv::Mat>* inputImages = nullptr;

    bool init();
};

struct HeightmapParam {
    uint8_t pyrLayer = 0; // Lower layers produce more details, along with more noise
    int width = 0;
    int height = 0;
    int kernelSize = 5;
    float sigma = 1.0;
    cv::Size size = cv::Size(0, 0);
    std::vector<cv::Size> pyramidSizes;

    bool init(const InputParam& input);
};

struct PostProcessParam {
    bool reconstructFirst = false;
    bool wantRemoveByEnergy = false;
    bool wantRemoveByStdDev = false;
    float energyThreshold = 0.0;
    float stdDevThreshold = 0.0;
    bool wantRemoveByRadius = false;
    double searchRadius = 10.0;
    int minNeighborsInRadius = 10;

    bool init();
};


struct OutputParam {
    std::string folderPath = "";
    std::string fileName = dateTimeStr();
    bool wantColorMap = false;
    bool wanttiff = false;
    bool wantASC = false;
    bool wantInfoTXT = false;
    bool wantVideo = false;
    bool wantRaw = false;
    int width = 0;
    int height = 0;
    cv::Size size = cv::Size(0, 0);
    float colorMapInMin = 0.0;
    float colorMapInMax = 255.0;
    float colorMapOutMin = 0.0;
    float colorMapOutMax = 255.0;
    float tiffInMin = 0.0;
    float tiffInMax = 65535.0;
    float tiffOutMin = 0.0;
    float tiffOutMax = 65535.0;
    float ASCMin = 0.0;
    float ASCMax = 65535.0;
    float ASCxScale = 1.0;
    float ASCyScale = 1.0;
    float ASCzScale = 1.0;
    bool needScalingAndInvert = true;
    bool ASCxInvert = false;
    bool ASCyInvert = false;
    cv::Mat* colorMapOutput = nullptr;
    cv::Mat* rawOutput = nullptr;
    cv::Mat* tiffOutput = nullptr;

    std::string pythonEnginePath = "C:/Users/setup/PycharmProjects/my_Focus_Stacking/.venv/Scripts/python.exe";
    std::string pythonScriptPath = "C:/Users/setup/PycharmProjects/my_Focus_Stacking/images_to_video.py";
    
    std::string imageFolderPath = "";
    std::string videoPath = "";
    int videoFPS = 10;
    std::string videoCommand = "";

    std::string imageFolderPath2 = "";
    std::string videoPath2 = "";
    int videoFPS2 = 10;
    std::string videoCommand2 = "";
       
    bool init(const InputParam& input);
};