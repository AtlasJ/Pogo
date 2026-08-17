#pragma once
#include <iostream>
#include <string>
#include <limits>
#include <cstdlib>
#include <opencv2/opencv.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudawarping.hpp>
#include <fstream>
//#include <pcl/point_types.h>          // For pcl::PointXYZ
//#include <pcl/point_cloud.h>        // For pcl::PointCloud
#include "param.h"
#include "input_process.h"
#include "heightmap_process.h"
#include "post_process.h"
//#include "kernel_hm.h"
#include "output_process.h"
#include "utils.h"

int height_map_wrapper(InputParam& input, HeightmapParam& heightmap, PostProcessParam& postProcess, OutputParam& output);
int height_map_wrapper(const std::string& input_path, cv::Mat& output_image, InputParam& input, HeightmapParam& heightmap, PostProcessParam& postProcess, OutputParam& output);
int height_map_wrapper(const std::vector<cv::Mat>& input_images, cv::Mat& output_image, InputParam& input, HeightmapParam& heightmap, PostProcessParam& postProcess, OutputParam& output);
