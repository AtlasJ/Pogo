#pragma once
#include <opencv2/opencv.hpp>
//#include <pcl/point_types.h>          // For pcl::PointXYZ
//#include <pcl/point_cloud.h>        // For pcl::PointCloud
#include "param.h"

bool removeByEnergy(cv::cuda::GpuMat& maxIndexMat, cv::cuda::GpuMat& maxEnergyMat, float energyThreshold);
bool removeByStdDev(cv::cuda::GpuMat& maxIndexMat, cv::cuda::GpuMat& stdDevMat, float stdDevThreshold);
//bool convertToPointCloud(const cv::cuda::GpuMat& src, OutputParam& output, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in);
//bool removeByRadius(
//    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_in, // Use ConstPtr for input
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered, // Use Ptr for output
//    const PostProcessParam& postProcess);