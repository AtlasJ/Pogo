#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudawarping.hpp>
#include <omp.h>
#include <cuda_runtime.h>
#include "kernel.h"
#include "focus_stacking.h"

using namespace std;
using namespace cv;

// Function declarations
cuda::GpuMat lapFocusStacking(const vector<cuda::GpuMat>& images, int numImages, int numLayers, dim3 block);
void getLaplacianPyramid(const cuda::GpuMat& img,
	vector<cuda::GpuMat>& laplacianPyramid,
	cuda::GpuMat& currentBase,
	cuda::Stream& stream,
	const vector<Size>& pyramidSizes,
	int numLayers);

cuda::GpuMat gpu_region_energy(const cuda::GpuMat& laplacian, cuda::Stream stream, dim3 block);
void focus_stacking(const vector<cuda::GpuMat>& batch_input_images, cuda::GpuMat& batch_output_image, int numLayers, bool is_color, dim3 block);
dim3 calculateBlockSize();
void ROI_function(const vector<cuda::GpuMat>& input_images, cuda::GpuMat& output_image, int fullWidth, int fullHeight, int roiWidth, int roiHeight, int numLayers, dim3 block, bool is_color);
void batch_function(const vector<Mat>& input_images, cuda::GpuMat& final_output_image,
	int batch_size, int roiWidth, int roiHeight, int numLayers, dim3 block, bool is_color);

dim3 calculateBlockSize() {
	cudaDeviceProp deviceProp;
	cudaGetDeviceProperties(&deviceProp, 0); // Assuming the first GPU

	int maxThreadsPerBlock = deviceProp.maxThreadsPerBlock;

	// Default block size within known constraints
	int blockX = 32;
	int blockY = 32;

	if (blockX * blockY > maxThreadsPerBlock) {
		blockX /= 2;
		blockY /= 2;
	}

	return dim3(blockX, blockY);
}

void focus_stacking_wrapper(const vector<Mat>& input_images, cuda::GpuMat& final_output_image, size_t availableMemory) {
	int deviceCount = 0;
	cudaError_t err = cudaGetDeviceCount(&deviceCount);

	if (err != cudaSuccess || deviceCount == 0) {
		cerr << "Error: No CUDA-capable GPU detected or CUDA runtime is not properly installed." << endl;
		return;
	}

	// Check if the input images vector is empty
	if (input_images.empty()) {
		cerr << "Error: Input images vector is empty. Exiting focus stacking wrapper." << endl;
		return;
	}

	int numImages = input_images.size();

	if (numImages < 2) {
		cerr << "Error: At least 2 images are required." << endl;
		return;
	}

	// Validate input images
	for (size_t idx = 0; idx < numImages; ++idx) {
		if (input_images[idx].empty()) {
			cerr << "Error: Image at index " << idx << " is empty. Exiting focus stacking wrapper." << endl;
			return;
		}
	}

	// Validate channels
	int channels = 1 + (input_images[0].type() >> CV_CN_SHIFT);
	bool is_color = (channels == 3);
	if (channels != 1 && channels != 3) {
		cerr << "Error: Unsupported number of channels (" << channels << "). Exiting." << endl;
		return;
	}

	int imageWidth = input_images[0].cols;
	int imageHeight = input_images[0].rows;

	// assessPerformance 
	cout << endl << "Assess Performance:" << endl;

	size_t memoryPerImage = static_cast<size_t>(imageWidth) * imageHeight * channels;

	int maxImages = static_cast<int>(availableMemory / memoryPerImage);

	if (numImages > maxImages) {
		cout << "Slower than normal speed. Max suggested images: " << maxImages << endl;
	}
	else {
		cout << "Normal speed. Max suggested images: " << maxImages << endl;
	}
	cout << "Total number of images given: " << numImages << endl;

	// Configurable parameters
	cout << endl << "Configurable Parameters:" << endl;
	int batch_size, roiWidth, roiHeight, numLayers;
	numLayers = 9;

	// Determine batch size
	batch_size = (numImages > 15) ? 10 : numImages;

	// Determine ROI dimensions
	roiWidth = (imageWidth > 6000) ? 3900 : imageWidth;
	roiHeight = (imageHeight > 6000) ? 3900 : imageHeight;

	// can hardcode here for testing
	//roiWidth = 10000;
	//roiHeight = 10000;
	//batch_size = 10;

	// Print calculated values for verification
	cout << "Apply ROI: " << roiWidth << "x" << roiHeight << endl;
	cout << "Apply Batch Size: " << batch_size << endl;

	// Calculate block size for kernel
	dim3 block = calculateBlockSize();
	cout << "Apply Block Size: " << block.x << "x" << block.y << endl;

	batch_function(input_images, final_output_image, batch_size, roiWidth, roiHeight, numLayers, block, is_color);
}

void batch_function(const vector<Mat>& input_images, cuda::GpuMat& final_output_image,
	int batch_size, int roiWidth, int roiHeight, int numLayers, dim3 block, bool is_color) {
	int numImages = input_images.size();
	int fullWidth = input_images[0].cols;
	int fullHeight = input_images[0].rows;

	// Initialize the intermediate output as the first image
	cuda::GpuMat intermediate_output;
	intermediate_output.upload(input_images[0]); // First image starts as the initial output

	// Process each batch
	int batch_counter = 0;
	for (int i = 1; i < numImages; i += batch_size) {

		int remainingImages = numImages - i; // Images left to process
		int currentBatchSize = min(batch_size, remainingImages); // Adjust batch size

		// Preallocate space for the batch
		vector<cuda::GpuMat> current_batch(currentBatchSize + 1); // +1 for the intermediate output
		current_batch[0] = intermediate_output; // Include the previous output
		auto start_test = chrono::high_resolution_clock::now();

		//Parallelized upload loop
#pragma omp parallel for
		for (int j = 0; j < currentBatchSize; ++j) {
			current_batch[j + 1].upload(input_images[i + j]);
		}

		auto end_test = chrono::high_resolution_clock::now();
		chrono::duration<double, milli> duration = end_test - start_test;
		cout << endl << "Time taken (upload to gpu): " << duration.count() << " ms\n" << endl;

		// Create a GpuMat to store the result of the current batch
		cuda::GpuMat batch_output;

		// Apply ROI processing to the current batch
		ROI_function(current_batch, batch_output, fullWidth, fullHeight, roiWidth, roiHeight, numLayers, block, is_color);

		// Update the intermediate output
		intermediate_output = batch_output;

		cout << "batch_counter : " << batch_counter << endl;
		batch_counter++;
	}

	// Final output is stored in intermediate_output
	intermediate_output.copyTo(final_output_image);
}

void ROI_function(const vector<cuda::GpuMat>& input_images, cuda::GpuMat& output_image,
	int fullWidth, int fullHeight, int roiWidth, int roiHeight, int numLayers, dim3 block, bool is_color) {
	int numImages = input_images.size();
	if (numImages == 0) {
		cerr << "Error: input_images is empty." << endl;
		return;
	}

	int overlapSize = 98; // Set your desired overlap size
	overlapSize = max(0, min(overlapSize, min(roiWidth / 2, roiHeight / 2)));

	// Initialize the output GpuMat
	if (is_color) {
		output_image.create(fullHeight, fullWidth, CV_32FC3);
	}
	else {
		output_image.create(fullHeight, fullWidth, CV_32FC1);
	}

	output_image.setTo(Scalar(0)); // Optional: Initialize to zeros

	int ROI_counter = 0;
	cuda::GpuMat output_image_ROI_gpu; // Reuse buffer

	// Loop over ROIs
	for (int y = 0; y < fullHeight; y += roiHeight) {
		for (int x = 0; x < fullWidth; x += roiWidth) {
			int xStart = max(0, x - overlapSize);
			int yStart = max(0, y - overlapSize);
			int xEnd = min(fullWidth, x + roiWidth + overlapSize);
			int yEnd = min(fullHeight, y + roiHeight + overlapSize);

			if (xStart >= xEnd || yStart >= yEnd) continue; // Skip invalid regions
			Rect extendedRoi(xStart, yStart, xEnd - xStart, yEnd - yStart);

			Rect actualRoi(overlapSize, overlapSize, roiWidth, roiHeight);
			if (x == 0) actualRoi.x = 0;
			if (y == 0) actualRoi.y = 0;
			if (x + roiWidth >= fullWidth) actualRoi.width = min(roiWidth, fullWidth - x);
			if (y + roiHeight >= fullHeight) actualRoi.height = min(roiHeight, fullHeight - y);

			if (actualRoi.x + actualRoi.width > extendedRoi.width ||
				actualRoi.y + actualRoi.height > extendedRoi.height) continue;

			Rect roi(x, y, actualRoi.width, actualRoi.height);
			if (roi.x + roi.width > fullWidth || roi.y + roi.height > fullHeight) continue;

			vector<cuda::GpuMat> input_images_ROI_gpu(numImages);
			for (int i = 0; i < numImages; ++i) {
				int adjustedWidth = min(extendedRoi.width, input_images[i].cols - extendedRoi.x);
				int adjustedHeight = min(extendedRoi.height, input_images[i].rows - extendedRoi.y);
				Rect adjustedExtendedRoi(extendedRoi.x, extendedRoi.y, adjustedWidth, adjustedHeight);
				input_images_ROI_gpu[i] = input_images[i](adjustedExtendedRoi);
			}

			if (is_color) {
				output_image_ROI_gpu.create(extendedRoi.size(), CV_32FC3);
			}
			else {
				output_image_ROI_gpu.create(extendedRoi.size(), CV_32FC1);
			}
			focus_stacking(input_images_ROI_gpu, output_image_ROI_gpu, numLayers, is_color, block);

			cuda::GpuMat croppedOutput = output_image_ROI_gpu(actualRoi);
			croppedOutput.copyTo(output_image(roi));

			cout << "ROI_counter: " << ROI_counter
				<< " | x: " << x << ", y: " << y
				<< " | ROI: " << roi.width << "x" << roi.height
				<< endl << endl;
			ROI_counter++;
		}
	}
}

void focus_stacking(const vector<cuda::GpuMat>& batch_input_images, cuda::GpuMat& batch_output_image, int numLayers, bool is_color, dim3 block) {
	auto start_global = chrono::high_resolution_clock::now();
	int batch_numImages = batch_input_images.size();

	// Check if images are loaded
	if (batch_numImages == 0) {
		cerr << "No images loaded for focus stacking." << endl;
		return;
	}

	// Get image dimensions from the first image
	Size inputImageSize = batch_input_images[0].size();

	if (is_color) {
		vector<vector<cuda::GpuMat>> channels(3, vector<cuda::GpuMat>(batch_numImages));
		vector<cuda::GpuMat> lapFocusChannels(3);
#pragma omp parallel for
		for (int i = 0; i < batch_numImages; ++i) {
			// Convert input image to int16 type
			cuda::GpuMat convertedImage;
			batch_input_images[i].convertTo(convertedImage, CV_16S);

			// Split the converted image into 3 channels
			vector<cuda::GpuMat> d_channels(3);
			cuda::split(convertedImage, d_channels);

#pragma omp parallel for
			// Store each channel in the channels vector
			for (int c = 0; c < 3; ++c) {
				channels[c][i] = d_channels[c];
			}
		}
		cudaDeviceSynchronize();  // Ensure all channels are processed before continuing

		for (int c = 0; c < 3; ++c) {
			cout << "Lap channel " << to_string(c) << "..." << endl;
			lapFocusChannels[c] = lapFocusStacking(channels[c], batch_numImages, numLayers, block);
		}

		callCustomMergeKernel(lapFocusChannels, batch_output_image, block);
	}
	else {
		vector<cuda::GpuMat> singleChannelImages(batch_numImages);

#pragma omp parallel for
		for (int i = 0; i < batch_numImages; ++i) {
			// Convert input image to int16 type
			cuda::GpuMat convertedImage;
			batch_input_images[i].convertTo(convertedImage, CV_16S);

			// Store the converted single-channel image
			singleChannelImages[i] = convertedImage;
		}
		cudaDeviceSynchronize();  // Ensure all images are processed before continuing

		cout << "Lap channel " << to_string(0) << "..." << endl;
		batch_output_image = lapFocusStacking(singleChannelImages, batch_numImages, numLayers, block);
	}

	auto end_global = chrono::high_resolution_clock::now();

	chrono::duration<double, milli> duration_global = end_global - start_global;
	cout << "Execution time: " << duration_global.count() << " ms" << endl;

}

cuda::GpuMat lapFocusStacking(const vector<cuda::GpuMat>& images, int numImages, int numLayers, dim3 block) {
	vector<Size> pyramidSizes(numLayers + 1);

	int downsampledWidth = images[0].cols;
	int downsampledHeight = images[0].rows;

	for (int i = 0; i < numLayers; ++i) {
		pyramidSizes[i] = Size(downsampledWidth, downsampledHeight);
		downsampledWidth = (downsampledWidth + 1) / 2;
		downsampledHeight = (downsampledHeight + 1) / 2;
	}
	pyramidSizes[numLayers] = Size(downsampledWidth, downsampledHeight);

	cuda::GpuMat base(pyramidSizes[numLayers], CV_16S, Scalar::all(0));

	vector<vector<cuda::GpuMat>> listLapPyramids(numImages, vector<cuda::GpuMat>(numLayers));
	for (int i = 0; i < numImages; ++i) {
		for (int l = 0; l < numLayers; ++l) {
			listLapPyramids[i][l].create(pyramidSizes[l], CV_16S);  // Preallocate each GpuMat to the correct size and type
		}
	}

	vector<cuda::Stream> streams(numImages);

	//cout << "\tBuilding Laplacian Pyramid..." << endl;
	//auto start = chrono::high_resolution_clock::now();

	for (int i = 0; i < numImages; ++i) {
		cuda::GpuMat currentBase;
		// Pass the specific pyramid array for this image to getLaplacianPyramid
		getLaplacianPyramid(images[i], listLapPyramids[i], currentBase, streams[i], pyramidSizes, numLayers);

		// Accumulate currentBase into base asynchronously using cuda::Stream
		cuda::add(base, currentBase, base, cuda::GpuMat(), -1, streams[i]);
	}

	//auto end = chrono::high_resolution_clock::now();
	//chrono::duration<double, milli> duration = end - start;
	//cout << "\tTime taken: " << duration.count() << " ms\n" << endl;

	cuda::multiply(base, 1.0 / numImages, base);

	vector<cuda::GpuMat> LP_f(numLayers);   // For each layer
	vector<cuda::GpuMat> LP_l(numLayers);   // Reuse for each layer
	vector<cuda::GpuMat> RE_l(numImages);   // Reuse for regional energy

	for (int l = numLayers - 1; l >= 0; --l) {
		LP_l[l].create(pyramidSizes[l], CV_32F);
		LP_l[l].setTo(0);

		// Calculate regional energy for each image at the current level
		//cout << "\tcompute region energy..." << endl;
		//start = chrono::high_resolution_clock::now();

		// Launch processing in parallel on each stream
		for (int i = 0; i < numImages; ++i) {
			if (RE_l[i].empty()) {
				RE_l[i].create(listLapPyramids[i][l].size(), CV_32F);
			}
			RE_l[i] = gpu_region_energy(listLapPyramids[i][l], streams[i], block);  // Pass the stream to gpu_region_energy
		}
		cudaDeviceSynchronize();  // Ensure all regional energy calculations are complete

		//end = chrono::high_resolution_clock::now();
		//duration = end - start;
		//cout << "\tTime taken: " << duration.count() << " ms\n" << endl;

		// Loop through each pixel in the current layer
		//cout << "\tprocessing layer " << to_string(l) << "..." << pyramidSizes[l] << endl;
		//start = chrono::high_resolution_clock::now();

		callRegionalEnergyKernel(RE_l, LP_l[l], listLapPyramids, l, numImages, block);

		//end = chrono::high_resolution_clock::now();
		//duration = end - start;
		//cout << "\tTime taken: " << duration.count() << " ms\n" << endl;

		// Assign the fused layer LP_l to LP_f for the current layer
		LP_f[l] = LP_l[l];
	}

	//cout << "\tfused image..." << endl;
	//start = chrono::high_resolution_clock::now();

	// Create a stream for asynchronous execution
	cuda::Stream stream;
	base.convertTo(base, CV_32F, stream);

	for (int i = numLayers - 1; i >= 0; --i) {

		cuda::pyrUp(base, base, stream);

		// Resize only if necessary, using the precomputed target size of LP_f[i]
		if (base.size() != pyramidSizes[i]) {
			cuda::resize(base, base, pyramidSizes[i], 0, 0, INTER_LINEAR, stream);
		}

		// Add the current pyramid layer to fused_img
		cuda::add(base, LP_f[i], base, cuda::GpuMat(), -1, stream);
	}
	cudaDeviceSynchronize();  // Ensure the pyramid reconstruction is complete

	//end = chrono::high_resolution_clock::now();
	//duration = end - start;
	//cout << "\tTime taken: " << duration.count() << " ms\n" << endl;

	return base;
}

//Modified function to accept a 2D array pointer directly
void getLaplacianPyramid(const cuda::GpuMat& img,
	vector<cuda::GpuMat>& laplacianPyramid,
	cuda::GpuMat& currentBase,
	cuda::Stream& stream,
	const vector<Size>& pyramidSizes,
	int numLayers) {

	cuda::GpuMat curr_img = img;
	cuda::GpuMat downsampled, upsampled, lap;

	// Allocate once and reuse for each layer
	downsampled.create(curr_img.size(), curr_img.type());
	upsampled.create(curr_img.size(), curr_img.type());
	lap.create(curr_img.size(), curr_img.type());

	for (int i = 0; i < numLayers; ++i) {
		cuda::pyrDown(curr_img, downsampled, stream);
		cuda::pyrUp(downsampled, upsampled, stream);
		if (upsampled.size() != pyramidSizes[i]) {
			cuda::resize(upsampled, upsampled, pyramidSizes[i], 0, 0, INTER_LINEAR, stream);
		}

		// Perform the subtraction and directly store in laplacianPyramid[i]
		cuda::subtract(curr_img, upsampled, lap, cuda::GpuMat(), -1, stream);
		laplacianPyramid[i] = lap;  // Ensure separate storage for each level

		curr_img = downsampled;
	}
	currentBase = curr_img;
}

cuda::GpuMat gpu_region_energy(const cuda::GpuMat& laplacian, cuda::Stream stream, dim3 block) {
	cuda::GpuMat convolved_result;
	convolved_result.create(laplacian.size(), CV_32F);

	// Apply combined kernel for squaring and filtering in the provided stream
	callCombinedSquareAndFilterKernel(laplacian, convolved_result, stream, block);

	return convolved_result;
}
