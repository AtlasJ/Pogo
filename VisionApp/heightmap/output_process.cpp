#include <opencv2/opencv.hpp>
#include <opencv2/cudaarithm.hpp>
#include <fstream>
//#include <pcl/point_types.h>          // For pcl::PointXYZ
//#include <pcl/point_cloud.h>        // For pcl::PointCloud
#include "output_process.h"
#include "param.h"

// Clips and scales a GpuMat directly to desired range (e.g. 0–255 or 0–65535)
cv::cuda::GpuMat scaleForSaving(const cv::cuda::GpuMat& src, double userMin, double userMax, double scaleMax)
{
	cv::cuda::GpuMat clipped, normalized;

	// Step 1: Clip to [userMin, userMax]
	cv::cuda::min(src, userMax, clipped);
	cv::cuda::max(clipped, userMin, clipped);

	// Step 2: Scale directly to [0, scaleMax]
	cv::cuda::subtract(clipped, cv::Scalar(userMin), normalized);
	double scale = scaleMax / (userMax - userMin);

	cv::cuda::GpuMat result;
	normalized.convertTo(result, -1, scale);

	return result;
}

cv::cuda::GpuMat scaleToRange(const cv::cuda::GpuMat& src,
	double inputMin, double inputMax,
	double outputMin, double outputMax,
	int outType)
{
	// Normalize possibly swapped inputs
	if (inputMin > inputMax) std::swap(inputMin, inputMax);

	// 1) Clip to [inputMin, inputMax]
	cv::cuda::GpuMat clipped;
	cv::cuda::min(src, inputMax, clipped);
	cv::cuda::max(clipped, inputMin, clipped);

	cv::cuda::GpuMat dst;

	// 2) Handle degenerate range
	const double inRange = inputMax - inputMin;
	if (inRange <= DBL_EPSILON) {
		// Output constant = outputMin
		// alpha=0, beta=outputMin -> y = 0*x + outputMin
		clipped.convertTo(dst, outType, /*alpha=*/0.0, /*beta=*/outputMin);
		return dst;
	}

	// 3) Linear map to [outputMin, outputMax]
	// y = alpha * x + beta, where:
	// alpha = (outMax - outMin) / (inMax - inMin)
	// beta  = outMin - inMin * alpha
	const double alpha = (outputMax - outputMin) / inRange;
	const double beta = outputMin - inputMin * alpha;

	clipped.convertTo(dst, outType, alpha, beta);
	return dst;
}

bool saveRaw(const cv::cuda::GpuMat& src, const OutputParam& output)
{
	cv::Mat imgCPU;
	src.download(imgCPU);

	if (output.rawOutput) *output.rawOutput = imgCPU;

	if (output.fileName.empty() || output.folderPath.empty()) return false;
	return cv::imwrite(output.folderPath + '/' + output.fileName + "_raw.tiff", imgCPU);
}

bool saveColorMap(const cv::cuda::GpuMat& src, const OutputParam& output)
{
	//cv::cuda::GpuMat scaled = scaleForSaving(src, output.colorMapMin, output.colorMapMax, 255.0);
	cv::cuda::GpuMat scaled = scaleToRange(src, output.colorMapInMin, output.colorMapInMax, output.colorMapOutMin, output.colorMapOutMax);
	
	cv::cuda::GpuMat scaled8U;
	scaled.convertTo(scaled8U, CV_8U);

	cv::Mat imgCPU, colorMap;
	scaled8U.download(imgCPU);
	cv::applyColorMap(imgCPU, colorMap, cv::COLORMAP_RAINBOW);

	if (output.colorMapOutput) *output.colorMapOutput = colorMap;

	if (output.fileName.empty() || output.folderPath.empty()) return false;
	return cv::imwrite(output.folderPath + '/' + output.fileName + ".png", colorMap);
}

bool saveColorMap(const cv::cuda::GpuMat& src, const float min, const float max, const std::string folderPath, const std::string fileName, cv::Mat* outImage)
{
	//cv::cuda::GpuMat scaled = scaleForSaving(src, min, max, 255.0);
	cv::cuda::GpuMat scaled = scaleToRange(src, min, max, 0.0, 255.0);

	cv::cuda::GpuMat scaled8U;
	scaled.convertTo(scaled8U, CV_8U);

	cv::Mat imgCPU, colorMap;
	scaled8U.download(imgCPU);
	cv::applyColorMap(imgCPU, colorMap, cv::COLORMAP_RAINBOW);

	if (outImage) *outImage = colorMap;

	if (fileName.empty() || folderPath.empty()) return false;
	return cv::imwrite(folderPath + '/' + fileName + ".png", colorMap);
}

bool saveTiff(const cv::cuda::GpuMat& src, const float min, const float max, const std::string folderPath, const std::string fileName, cv::Mat* outImage)
{
	//cv::cuda::GpuMat scaled = scaleForSaving(src, min, max, 65535.0);
	cv::cuda::GpuMat scaled = scaleToRange(src, min, max, 0.0, 65535.0);

	cv::cuda::GpuMat scaled16U;
	scaled.convertTo(scaled16U, CV_16U);

	cv::Mat tiffImage;
	scaled16U.download(tiffImage);

	if (outImage) *outImage = tiffImage;

	if (fileName.empty() || folderPath.empty()) return false;
	return cv::imwrite(folderPath + '/' + fileName + ".tiff", tiffImage);
}

bool saveTiff(const cv::cuda::GpuMat& src, const OutputParam& output)
{
	//cv::cuda::GpuMat scaled = scaleForSaving(src, output.tiffMin, output.tiffMax, 65535.0);
	cv::cuda::GpuMat scaled = scaleToRange(src, output.tiffInMin, output.tiffInMax, output.tiffOutMin, output.tiffOutMax);

	cv::cuda::GpuMat scaled16U;
	scaled.convertTo(scaled16U, CV_16U);

	cv::Mat tiffImage;
	scaled16U.download(tiffImage);

	if (output.tiffOutput) *output.tiffOutput = tiffImage;

	if (output.fileName.empty() || output.folderPath.empty()) return false;
	return cv::imwrite(output.folderPath + '/' + output.fileName + ".tiff", tiffImage);
}

bool saveASC(const cv::cuda::GpuMat& src, OutputParam& output)
{
	//cv::cuda::GpuMat scaled = scaleForSaving(src, output.ASCMin, output.ASCMax, 65535.0);
	cv::Mat imgCPU;
	src.download(imgCPU);

	if (output.ASCxInvert) cv::flip(imgCPU, imgCPU, 1);
	if (output.ASCyInvert) cv::flip(imgCPU, imgCPU, 0);

	std::ofstream ofs(output.folderPath + "/" + output.fileName + ".asc");

	if (ofs.is_open()) {
		for (int y = 0; y < imgCPU.rows; ++y) {
			for (int x = 0; x < imgCPU.cols; ++x) {
				float value = imgCPU.at<float>(y, x);
				if (!std::isfinite(value) || value < 0) {
					continue;
				}
				
				if (output.needScalingAndInvert) {
					ofs << (x * output.ASCxScale) << " " << (y * output.ASCyScale) << " " << (value * output.ASCzScale) << "\n";
				}
				else {
					ofs << (x) << " " << (y) << " " << (value) << "\n";
				}
			}
		}

		if (output.needScalingAndInvert) output.needScalingAndInvert = false;

		ofs.close();
		return true;
	}
	else {
		return false;
	}
}

//bool saveASC(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, const OutputParam& output) // Pass by const reference
//{
//	// --- Input Validation ---
//	if (!cloud) {
//		std::cerr << "Error [saveASC - PointCloud]: Input cloud pointer is null." << std::endl;
//		return false;
//	}
//
//	std::string full_path = output.folderPath + "/" + output.fileName + ".asc";
//	std::ofstream ofs(full_path);
//
//	if (!ofs.is_open()) {
//		std::cerr << "Error [saveASC - PointCloud]: Could not open file for writing: " << full_path << std::endl;
//		return false;
//	}
//
//	std::cout << "Saving point cloud to ASC file: " << full_path << std::endl;
//	std::cout << "  Number of points to save: " << cloud->size() << std::endl;
//
//	// --- Iterate through points and write to file ---
//	// We assume points in 'cloud' are already filtered and scaled.
//	for (const auto& point : cloud->points) {
//		// Basic check for sanity, although we assume valid points from prior steps
//		if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
//			std::cerr << "Warning [saveASC - PointCloud]: Skipping non-finite point during save." << std::endl;
//			continue;
//		}
//
//		// Write X Y Z coordinates separated by space, followed by newline
//		// Uses default precision of the output stream
//		ofs << point.x << " " << point.y << " " << point.z << "\n";
//	}
//
//	// --- Check for stream errors after writing ---
//	if (!ofs.good()) {
//		std::cerr << "Error [saveASC - PointCloud]: IO error occurred while writing to file: " << full_path << std::endl;
//		ofs.close(); // Attempt to close even on error
//		return false;
//	}
//
//	ofs.close();
//
//	// Final check if closing failed (less common)
//	if (ofs.fail()) {
//		std::cerr << "Error [saveASC - PointCloud]: Failed to properly close file: " << full_path << std::endl;
//		return false;
//	}
//
//	std::cout << "Successfully saved point cloud to " << full_path << std::endl;
//	return true;
//}

bool saveInfoTXT(const InputParam& input, const HeightmapParam& heightmap, const PostProcessParam& postProcess, const OutputParam& output) 
{
	if (output.fileName.empty() || output.folderPath.empty()) return false;

	std::vector<std::pair<std::string, std::vector<std::pair<std::string, std::string>>>> data = {
		{"input", 
			{
				{"folderPath", input.folderPath},
				{"fileExtension", input.fileExtension},
				{"channel", (input.channel == Channel::RED) ? "RED" :
							(input.channel == Channel::GREEN) ? "GREEN" :
							(input.channel == Channel::BLUE) ? "BLUE" :
							(input.channel == Channel::GRAY) ? "GRAY" : "UNKNOWN"},
				{"start", std::to_string(input.start)},
				{"end", std::to_string(input.end)},
				{"step", std::to_string(input.step)},
				{"width", std::to_string(input.width)},
				{"height", std::to_string(input.height)},
				{"size", "[" + std::to_string(input.size.width) + " x " + std::to_string(input.size.height) + "]"},
				{"imageCount", std::to_string(input.imageCount)},
				{"umPerZStep", std::to_string(input.umPerZStep)},
				{"umPerXPixel", std::to_string(input.umPerXPixel)},
				{"umPerYPixel", std::to_string(input.umPerYPixel)},
				{"inputFromMemory", input.inputFromMemory ? "true" : "false" },
			}
		},

		{"heightmap", 
			{
				{"pyrLayer", std::to_string(heightmap.pyrLayer)},
				{"width", std::to_string(heightmap.width)},
				{"height", std::to_string(heightmap.height)},
				{"kernelSize", std::to_string(heightmap.kernelSize)},
				{"sigma", std::to_string(heightmap.sigma)},
				{"size", "[" + std::to_string(heightmap.size.width) + " x " + std::to_string(heightmap.size.height) + "]"},
				{"pyramidSizes",([&]() {
					std::stringstream ss;
					ss << "[";
					for (size_t i = 0; i < heightmap.pyramidSizes.size(); ++i) {
						ss << "[" << heightmap.pyramidSizes[i].width << " x " << heightmap.pyramidSizes[i].height << "]";
						if (i < heightmap.pyramidSizes.size() - 1) ss << ", ";
					}
					ss << "]";
					return ss.str();
					}())},
			}
		},

		{"postProcess",
			{
				{"reconstructFirst", postProcess.reconstructFirst ? "true" : "false" },
				{"wantRemoveByEnergy", postProcess.wantRemoveByEnergy ? "true" : "false" },
				{"wantRemoveByStdDev", postProcess.wantRemoveByStdDev ? "true" : "false" },
				{"energyThreshold", std::to_string(postProcess.energyThreshold)},
				{"stdDevThreshold", std::to_string(postProcess.stdDevThreshold)},
				{"wantRemoveByRadius", postProcess.wantRemoveByRadius ? "true" : "false" },
				{"searchRadius", std::to_string(postProcess.searchRadius)},
				{"minNeighborsInRadius", std::to_string(postProcess.minNeighborsInRadius)},
			}
		},

		{"output", 
			{
				{"folderPath", output.folderPath},
				{"fileName", output.fileName},
				{"wantColorMap", output.wantColorMap ? "true" : "false"},
				{"wanttiff", output.wanttiff ? "true" : "false"},
				{"wantASC", output.wantASC ? "true" : "false"},
				{"wantInfoTXT", output.wantInfoTXT ? "true" : "false"},
				{"wantVideo", output.wantVideo ? "true" : "false"},
				{"width", std::to_string(output.width)},
				{"height", std::to_string(output.height)},
				{"size", "[" + std::to_string(output.size.width) + " x " + std::to_string(output.size.height) + "]"},
				{"colorMapInMin", std::to_string(output.colorMapInMin)},
				{"colorMapInMax", std::to_string(output.colorMapInMax)},
				{"colorMapOutMin", std::to_string(output.colorMapOutMin)},
				{"colorMapOutMax", std::to_string(output.colorMapOutMax)},
				{"tiffInMin", std::to_string(output.tiffInMin)},
				{"tiffInMax", std::to_string(output.tiffInMax)},
				{"tiffOutMin", std::to_string(output.tiffOutMin)},
				{"tiffOutMax", std::to_string(output.tiffOutMax)},
				{"ASCMin", std::to_string(output.ASCMin)},
				{"ASCMax", std::to_string(output.ASCMax)},
				{"ASCxScale", std::to_string(output.ASCxScale)},
				{"ASCyScale", std::to_string(output.ASCyScale)},
				{"ASCzScale", std::to_string(output.ASCzScale)},
				{"needScalingAndInvert", output.needScalingAndInvert ? "true" : "false"},
				{"ASCxInvert", output.ASCxInvert ? "true" : "false"},
				{"ASCyInvert", output.ASCyInvert ? "true" : "false"},
				{"pythonEnginePath", output.pythonEnginePath},
				{"pythonScriptPath", output.pythonScriptPath},
				{"imageFolderPath", output.imageFolderPath},
				{"videoPath", output.videoPath},
				{"videoFPS", std::to_string(output.videoFPS)},
				{"videoCommand", output.videoCommand},
				{"imageFolderPath2", output.imageFolderPath2},
				{"videoPath2", output.videoPath2},
				{"videoFPS2", std::to_string(output.videoFPS2)},
				{"videoCommand2", output.videoCommand2},
			}
		}
	};

	std::ofstream outputFile(output.folderPath + "/" + output.fileName + ".txt");

	if (outputFile.is_open()) {
		for (const auto& section : data) {
			outputFile << section.first << ":" << std::endl; // Write section name
			for (const auto& field : section.second) {
				outputFile << field.first << " = " << field.second << std::endl; // Write field=value
			}
			outputFile << std::endl; // Add a blank line between sections
		}
		outputFile.close();
		return true;
	}
	else {
		return false;
	}
}

bool saveVideo(std::string command)
{
	int result = std::system(command.c_str());

	if (result == 0) {
		std::cout << "Python script executed successfully." << std::endl;
		return true;
	}
	else {
		std::cerr << "Python script execution failed." << std::endl;
		return false;
	}
}