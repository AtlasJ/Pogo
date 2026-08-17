#pragma once
#include <opencv2/opencv.hpp>
#include "param.h"

bool generateEnergyMat(const HeightmapParam& heightmap, const cv::cuda::GpuMat& src, cv::cuda::GpuMat& dst);
bool getLaplacianLayer(const uint8_t layer, const cv::cuda::GpuMat& src, cv::cuda::GpuMat& dst);
bool gpu_region_energy_hm(const cv::cuda::GpuMat& src, cv::cuda::GpuMat& dst, int kernelSize, float sigma);
bool updateMaxEnergyAndMaxIndex(const cv::cuda::GpuMat& currentEnergy,  cv::cuda::GpuMat& maxEnergyMat, cv::cuda::GpuMat& maxIndexMat, const size_t index);
bool reconstructPyr(const cv::cuda::GpuMat& src, const HeightmapParam& heightmap, cv::cuda::GpuMat& dst);