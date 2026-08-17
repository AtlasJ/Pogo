// focus_stacking.h
#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>
#include <vector>

// Declare main focus stacking function
void focus_stacking_wrapper(const std::vector<cv::Mat>& input_images, cv::cuda::GpuMat& output_image, size_t availableMemory);
