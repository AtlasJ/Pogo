// kernel.h
#pragma once
#include <opencv2/core/cuda.hpp>
#include <vector>

// Declare wrapper functions for CUDA kernels
void callCombinedSquareAndFilterKernel(const cv::cuda::GpuMat& src, cv::cuda::GpuMat& dst, cv::cuda::Stream stream, dim3 block);
void callRegionalEnergyKernel(const std::vector<cv::cuda::GpuMat>& RE_l, cv::cuda::GpuMat& LP_l, const std::vector<std::vector<cv::cuda::GpuMat>>& listLapPyramids, int layer, int numImages, dim3 block);
void callCustomMergeKernel(const std::vector<cv::cuda::GpuMat>& channels, cv::cuda::GpuMat& output_image, dim3 block);
