#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include "param.h"
#include "utils.h"


bool InputParam::init() 
{
    cv::Mat img;
    if (!inputFromMemory) {
        // Ensure the file extension starts with a dot
        std::string extension = fileExtension;
        if (!extension.empty() && extension.front() != '.') {
            extension = "." + extension;
        }

        // Construct full file path
        std::string filePath = folderPath + '/' + std::to_string(start) + extension;

        // Check if file exists
        if (!fileExists(filePath)) {
            std::cerr << filePath << " does not exist" << std::endl;
            return false;
        }

        // Load image
        img = cv::imread(filePath, cv::IMREAD_GRAYSCALE);

        if (img.empty()) {
            std::cerr << "Error loading image: " << filePath << std::endl;
            return false;
        }
    }
    else {
        img = inputImages->at(0).clone();
    }

    width = img.cols;
    height = img.rows;
    size = cv::Size(width, height);

    if (step == 0 || end < start) return false;
    imageCount = ((end - start) / step) + 1;

    return true;
}

bool HeightmapParam::init(const InputParam& input) 
{
    size = input.size;
    pyramidSizes.push_back(size);
    for (size_t i = 0; i < pyrLayer; i++) {
        size = cv::Size((size.width + 1) / 2, (size.height + 1) / 2);
        pyramidSizes.push_back(size);
    }

    width = size.width;
    height = size.height;

    if (kernelSize < 3 || kernelSize % 2 == 0) {
        std::cout << "kernelSize must be greater than or equal to 3 and must be an odd number" << std::endl;
        return false;
    }

    return true;
}

bool PostProcessParam::init()
{
    return true;
}

bool OutputParam::init(const InputParam& input)
{
    width = input.width;
    height = input.height;
    size = input.size;

    if (wantASC) {
        ASCxScale = input.umPerXPixel;
        ASCyScale = input.umPerYPixel;
        ASCzScale = input.umPerZStep;
    }

    if (wantColorMap) {
        colorMapInMin = input.start;
        colorMapInMax = input.end;
    }

    if (wantVideo) {
        videoCommand.clear();
        videoCommand2.clear();

        if (!pythonEnginePath.empty() &&
            !pythonScriptPath.empty() &&
            !imageFolderPath.empty() &&
            videoFPS > 0) 
        {
            videoCommand += pythonEnginePath;
            videoCommand += " \"" + pythonScriptPath + "\" ";
            videoCommand += " \"" + imageFolderPath + "\" ";
            videoCommand += " \"" + videoPath + "\" ";
            videoCommand += " " + std::to_string(videoFPS) + " ";
            if(!createDirectory(imageFolderPath)) return false;
        }

        if (!pythonEnginePath.empty() &&
            !pythonScriptPath.empty() &&
            !imageFolderPath2.empty() &&
            videoFPS2 > 0)
        {
            videoCommand2 += pythonEnginePath;
            videoCommand2 += " \"" + pythonScriptPath + "\" ";
            videoCommand2 += " \"" + imageFolderPath2 + "\" ";
            videoCommand2 += " \"" + videoPath2 + "\" ";
            videoCommand2 += " " + std::to_string(videoFPS2) + " ";
            if (!createDirectory(imageFolderPath2)) return false;
        }
    }

    return true;
}