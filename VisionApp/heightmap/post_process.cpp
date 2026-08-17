#include "post_process.h"
#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/cudaarithm.hpp>
//#include <pcl/point_types.h>        // For pcl::PointXYZ
//#include <pcl/point_cloud.h>        // For pcl::PointCloud
//#include <pcl/filters/radius_outlier_removal.h> // The filter header
#include "param.h"

bool removeByEnergy(cv::cuda::GpuMat& maxIndexMat, cv::cuda::GpuMat& maxEnergyMat, float energyThreshold)
{
    // For energy threshold:
    // Create a mask where maxEnergyMat is less than energyThreshold.
    cv::cuda::GpuMat maskEnergy;
    cv::cuda::compare(maxEnergyMat, cv::Scalar(energyThreshold), maskEnergy, cv::CMP_LT);
    // For pixels where the mask is true, set maxIndexMat to std::numeric_limits<float>::lowest().
    maxIndexMat.setTo(cv::Scalar(std::numeric_limits<float>::lowest()), maskEnergy);

    return true;
}

bool removeByStdDev(cv::cuda::GpuMat& maxIndexMat, cv::cuda::GpuMat& stdDevMat, float stdDevThreshold)
{
    // For standard deviation threshold:
    // Create a mask where stdDevMat is less than stdDevThreshold.
    cv::cuda::GpuMat maskStdDev;
    cv::cuda::compare(stdDevMat, cv::Scalar(stdDevThreshold), maskStdDev, cv::CMP_LT);
    // For those pixels, set maxIndexMat to std::numeric_limits<float>::lowest().
    maxIndexMat.setTo(cv::Scalar(std::numeric_limits<float>::lowest()), maskStdDev);

    return true;
}

//bool convertToPointCloud(const cv::cuda::GpuMat& src, OutputParam& output, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in)
//{
//	//cv::cuda::GpuMat scaled = scaleForSaving(src, output.ASCMin, output.ASCMax, 65535.0);
//	cv::Mat imgCPU;
//	src.download(imgCPU);
//
//    if (output.needScalingAndInvert) {
//        if (output.ASCxInvert) cv::flip(imgCPU, imgCPU, 1);
//        if (output.ASCyInvert) cv::flip(imgCPU, imgCPU, 0);
//    }
//
//    if (imgCPU.type() != CV_32FC1) {
//        std::cerr << "Error [convertToPointCloud]: Input image must be CV_32FC1. Got type: " << imgCPU.type() << std::endl;
//        // Make sure cloud metadata is set for empty case before returning
//        cloud_in->width = 0;
//        cloud_in->height = 1;
//        cloud_in->is_dense = true;
//        return false;
//    }
//
//    cloud_in->clear(); // This clears points, width, height, etc.
//    cloud_in->points.reserve(imgCPU.rows * imgCPU.cols);
//
//	for (int y = 0; y < imgCPU.rows; ++y) {
//		for (int x = 0; x < imgCPU.cols; ++x) {
//			float value = imgCPU.at<float>(y, x);
//			if (!std::isfinite(value) || value < 0) {
//				continue;
//			}
//
//            if (output.needScalingAndInvert) {
//                cloud_in->points.emplace_back(static_cast<float>(x * output.ASCxScale), static_cast<float>(y * output.ASCyScale), value * output.ASCzScale);
//            }
//            else {
//                cloud_in->points.emplace_back(static_cast<float>(x), static_cast<float>(y), value);
//            }
//		}
//	}
//
//    if (output.needScalingAndInvert) output.needScalingAndInvert = false;
//
//    if (cloud_in->points.empty()) return false;
//
//    cloud_in->width = cloud_in->points.size(); // Size is now directly available
//    cloud_in->height = 1;
//    cloud_in->is_dense = true;
//
//    return true;
//}
//
//bool removeByRadius(
//    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_in, // Use ConstPtr for input
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered, // Use Ptr for output
//    const PostProcessParam& postProcess)
//{
//    // --- Input Validation ---
//    if (!cloud_in) {
//        std::cerr << "Error [removeByRadius]: Input cloud pointer is null." << std::endl;
//        return false;
//    }
//    if (!cloud_filtered) {
//        std::cerr << "Error [removeByRadius]: Output cloud pointer is null." << std::endl;
//        return false;
//    }
//    if (postProcess.searchRadius <= 0.0) {
//        std::cerr << "Error [removeByRadius]: Search radius must be positive (was "
//            << postProcess.searchRadius << ")." << std::endl;
//        return false;
//    }
//    if (postProcess.minNeighborsInRadius <= 0) {
//        std::cerr << "Error [removeByRadius]: Minimum neighbors must be positive (was "
//            << postProcess.minNeighborsInRadius << ")." << std::endl;
//        return false;
//    }
//
//    // --- Handle Empty Input Cloud ---
//    if (cloud_in->empty()) {
//        std::cerr << "Warning [removeByRadius]: Input cloud is empty. Output will be empty." << std::endl;
//        cloud_filtered->clear(); // Ensure output is empty
//        cloud_filtered->width = 0;
//        cloud_filtered->height = 1;
//        cloud_filtered->is_dense = true;
//        return false; // Success (filtered nothing)
//    }
//
//    // --- Logging ---
//    std::cout << "Applying RadiusOutlierRemoval:" << std::endl;
//    std::cout << "  Input points: " << cloud_in->size() << std::endl;
//    std::cout << "  Search Radius: " << postProcess.searchRadius << std::endl;
//    std::cout << "  Min Neighbors in Radius: " << postProcess.minNeighborsInRadius << std::endl;
//
//    // --- Configure and Apply Filter ---
//    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outlier_remover;
//    outlier_remover.setInputCloud(cloud_in);
//    outlier_remover.setRadiusSearch(postProcess.searchRadius);
//    outlier_remover.setMinNeighborsInRadius(postProcess.minNeighborsInRadius);
//
//    try {
//        outlier_remover.filter(*cloud_filtered);
//    }
//    catch (const std::exception& e) {
//        std::cerr << "Error [removeByRadius]: Exception during PCL filtering: " << e.what() << std::endl;
//        cloud_filtered->clear(); // Ensure output is empty on error
//        cloud_filtered->width = 0;
//        cloud_filtered->height = 1;
//        cloud_filtered->is_dense = true;
//        return false; // Indicate failure
//    }
//
//    // --- Post-Filtering Logging ---
//    std::cout << "RadiusOutlierRemoval finished." << std::endl;
//    std::cout << "  Output points: " << cloud_filtered->size() << std::endl;
//    if (cloud_filtered->empty() && !cloud_in->empty()) {
//        std::cerr << "Warning [removeByRadius]: All points were removed by the filter." << std::endl;
//    }
//
//
//    return true; // Indicate success
//}