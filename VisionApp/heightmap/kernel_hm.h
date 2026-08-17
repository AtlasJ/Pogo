#pragma once
#include <opencv2/opencv.hpp>
#include "device_launch_parameters.h"
#include "cuda_runtime.h"

//void callCombinedSquareAndFilterKernel(const cv::cuda::GpuMat& src, cv::cuda::GpuMat& dst);
//__global__ void combinedSquareAndFilterKernel(const float* src, float* dst, int srcWidth, int srcHeight, size_t srcStep, size_t dstStep);

std::vector<float> computeGaussianKernel(int kernelSize, float sigma);

void callCombinedSquareAndFilterKernel(
    const cv::cuda::GpuMat& src, 
    cv::cuda::GpuMat& dst,
    int kernelSize, 
    float sigma);

__global__ void combinedSquareAndFilterKernel(
    const float* src,
    float* dst,
    int srcWidth,
    int srcHeight,
    size_t srcStep,
    size_t dstStep,
    const float* d_kernel,
    int kernelSize);

void callUpdateMaxKernel(
    const cv::cuda::GpuMat& currentEnergy, 
    cv::cuda::GpuMat& energyMat, 
    cv::cuda::GpuMat& indexMat, 
    const size_t index);

__global__ void updateMaxKernel(
    const float* current_energy, 
    float* maxEnergy, 
    float* maxIndex, 
    int rows, 
    int cols, 
    int step, 
    unsigned short currentIteration);