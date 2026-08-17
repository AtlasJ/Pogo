#pragma once
#include <opencv2/opencv.hpp>
#include "param.h"

cv::cuda::GpuMat scaleForSaving(const cv::cuda::GpuMat& src, double userMin, double userMax, double scaleMax);
cv::cuda::GpuMat scaleToRange(const cv::cuda::GpuMat& src, double inputMin, double inputMax, double outputMin, double outputMax, int outType = -1);
bool saveRaw(const cv::cuda::GpuMat& src, const OutputParam& output);
bool saveColorMap(const cv::cuda::GpuMat& src, const OutputParam& output);
bool saveColorMap(const cv::cuda::GpuMat& src, const float min, const float max, const std::string folderPath, const std::string fileName, cv::Mat* outImage = nullptr);
bool saveTiff(const cv::cuda::GpuMat& src, const OutputParam& output);
bool saveTiff(const cv::cuda::GpuMat& src, const float min, const float max, const std::string folderPath, const std::string fileName, cv::Mat* outImage = nullptr);
//bool saveASC(const cv::cuda::GpuMat& src, OutputParam& output);
//bool saveASC(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, const OutputParam& output);
bool saveInfoTXT(const InputParam& input, const HeightmapParam& heightmap, const PostProcessParam& postProcess, const OutputParam& output);
bool saveVideo(std::string command);
