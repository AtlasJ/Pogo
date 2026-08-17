// Depth_from_focus.h
#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>
#include <vector>

struct SurfaceROI {
    cv::Rect surfaceRoi;
    std::vector<cv::Rect> planeRois;
};

struct FeatureROI {
    cv::Rect featureRoi;
};

// Declare main focus stacking function
void depth_from_focus_testing(const std::vector<cv::Mat>& input_images, cv::cuda::GpuMat& output_image, size_t availableMemory);
void depth_from_focus_surface(const std::vector<cv::Mat>& input_images, cv::Mat& output_image, const std::vector<SurfaceROI> surfaceRois);
void depth_from_focus_feature(const std::vector<cv::Mat>& input_images, cv::Mat& output_image, const std::vector<FeatureROI> featureRois);
void depth_from_focus(const std::vector<cv::Mat>& input_images, cv::Mat& output_image, const std::vector<SurfaceROI> surfaceRois, const std::vector<FeatureROI> featureRois);
