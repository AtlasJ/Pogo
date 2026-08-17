#pragma once
#include <string>
#include <opencv2/opencv.hpp>
#include "param.h"

bool loadSingleInputImage(const InputParam& input, const size_t index, cv::cuda::GpuMat& gpuImg);
bool loadSingleInputImageFromMemory(const InputParam& input, const size_t index, cv::cuda::GpuMat& dst);
