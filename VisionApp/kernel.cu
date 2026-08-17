#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include "kernel.h"

using namespace std;
using namespace cv;

__global__ void combinedSquareAndFilterKernel(const int16_t* src, float* dst, int srcWidth, int srcHeight, size_t srcStep, size_t dstStep);
__global__ void regionalEnergyKernel(
	float* d_LP_l,             // Output image
	const float* const* d_RE_l,       // Array of pointers to regional energy images
	const short* const* d_listLapPyramids,  // Array of pointers to Laplacian pyramids
	size_t LP_step, size_t pyramid_step,
	int cols, int rows, int numImages);
__global__ void customMergeKernel(const float* ch1, const float* ch2, const float* ch3, float* output, int width, int height, size_t chStep, size_t outputStep);

__global__ void regionalEnergyKernel(
	float* d_LP_l,             // Output image
	const float* const* d_RE_l,       // Array of pointers to regional energy images
	const short* const* d_listLapPyramids,  // Array of pointers to Laplacian pyramids
	size_t LP_step, size_t pyramid_step,
	int cols, int rows, int numImages) {

	int rowInd = blockIdx.y * blockDim.y + threadIdx.y;
	int colInd = blockIdx.x * blockDim.x + threadIdx.x;

	if (rowInd >= rows || colInd >= cols)
		return;

	float max_energy = -FLT_MAX;
	int max_idx = 0;

	// Loop over images to find the maximum regional energy
	for (int i = 0; i < numImages; ++i) {
		const float* current_RE_row = d_RE_l[i] + rowInd * LP_step / sizeof(float);
		float energy = current_RE_row[colInd];
		if (energy > max_energy) {
			max_energy = energy;
			max_idx = i;
		}
	}

	// Assign the pixel from the image with the maximum energy to the output
	const short* pyramid_row = d_listLapPyramids[max_idx] + rowInd * pyramid_step / sizeof(short);
	d_LP_l[rowInd * LP_step / sizeof(float) + colInd] = static_cast<float>(pyramid_row[colInd]);
}

void callRegionalEnergyKernel(const vector<cuda::GpuMat>& RE_l,
	cuda::GpuMat& LP_l,
	const vector<vector<cuda::GpuMat>>& listLapPyramids,
	int layer,
	int numImages, dim3 block) {

	int rows = LP_l.rows;
	int cols = LP_l.cols;

	// Create device arrays of pointers using vector
	vector<const float*> d_RE_l_ptrs(numImages);
	vector<const short*> d_listLapPyramids_ptrs(numImages);

	// Fill the pointer arrays with addresses from GpuMat objects
	for (int i = 0; i < numImages; ++i) {
		d_RE_l_ptrs[i] = RE_l[i].ptr<float>();
		d_listLapPyramids_ptrs[i] = listLapPyramids[i][layer].ptr<short>();
	}

	// Allocate memory for device pointer arrays
	const float** d_RE_l_device;
	const short** d_listLapPyramids_device;
	cudaMalloc(&d_RE_l_device, numImages * sizeof(float*));
	cudaMalloc(&d_listLapPyramids_device, numImages * sizeof(short*));

	// Copy pointer arrays to device memory
	cudaMemcpy(d_RE_l_device, d_RE_l_ptrs.data(), numImages * sizeof(float*), cudaMemcpyHostToDevice);
	cudaMemcpy(d_listLapPyramids_device, d_listLapPyramids_ptrs.data(), numImages * sizeof(short*), cudaMemcpyHostToDevice);

	// Define grid and block sizes
	//dim3 block(32, 32);
	dim3 grid((cols + block.x - 1) / block.x, (rows + block.y - 1) / block.y);

	// Launch the kernel
	regionalEnergyKernel << <grid, block >> > (
		LP_l.ptr<float>(), d_RE_l_device, d_listLapPyramids_device,
		LP_l.step, listLapPyramids[0][layer].step, cols, rows, numImages);

	cudaDeviceSynchronize();

	// Free device memory
	cudaFree(d_RE_l_device);
	cudaFree(d_listLapPyramids_device);
}


// Define the 5x5 Gaussian kernel in constant memory at global scope
__constant__ float kernel[5][5] = {
	{0.0025f, 0.0125f, 0.02f, 0.0125f, 0.0025f},
	{0.0125f, 0.0625f,  0.1f, 0.0625f, 0.0125f},
	{  0.02f,    0.1f, 0.16f,    0.1f,   0.02f},
	{0.0125f, 0.0625f,  0.1f, 0.0625f, 0.0125f},
	{0.0025f, 0.0125f, 0.02f, 0.0125f, 0.0025f}
};

__global__ void combinedSquareAndFilterKernel(const int16_t* src, float* dst, int srcWidth, int srcHeight, size_t srcStep, size_t dstStep) {
	int row = blockIdx.y * blockDim.y + threadIdx.y;
	int col = blockIdx.x * blockDim.x + threadIdx.x;

	if (row >= srcHeight || col >= srcWidth) return;

	float sum = 0.0f;

	// Apply 5x5 convolution with squared values
	for (int ky = -2; ky <= 2; ++ky) {
		for (int kx = -2; kx <= 2; ++kx) {
			int x = col + kx;
			int y = row + ky;

			if (x >= 0 && x < srcWidth && y >= 0 && y < srcHeight) {
				const int16_t* srcRowPtr = (const int16_t*)((const char*)src + y * srcStep);
				sum += static_cast<float>(srcRowPtr[x] * srcRowPtr[x]) * kernel[ky + 2][kx + 2];
			}
		}
	}

	float* dstRowPtr = (float*)((char*)dst + row * dstStep);
	dstRowPtr[col] = sum;
}

void callCombinedSquareAndFilterKernel(const cuda::GpuMat& src, cuda::GpuMat& dst, cuda::Stream stream, dim3 block) {
	//dim3 block(32, 32);
	dim3 grid((src.cols + block.x - 1) / block.x, (src.rows + block.y - 1) / block.y);

	// Launch the kernel in the specified stream
	combinedSquareAndFilterKernel << <grid, block, 0, static_cast<cudaStream_t>(stream.cudaPtr()) >> > ((int16_t*)src.data, (float*)dst.data, src.cols, src.rows, src.step, dst.step);
}

__global__ void customMergeKernel(const float* ch1, const float* ch2, const float* ch3, float* output, int width, int height, size_t chStep, size_t outputStep) {
	int row = blockIdx.y * blockDim.y + threadIdx.y;
	int col = blockIdx.x * blockDim.x + threadIdx.x;

	if (row >= height || col >= width) return;

	// Calculate the linear index for the output image
	int idx = row * width + col;

	// Get pointers to the correct row in each channel and the output
	const float* ch1RowPtr = (const float*)((const char*)ch1 + row * chStep);
	const float* ch2RowPtr = (const float*)((const char*)ch2 + row * chStep);
	const float* ch3RowPtr = (const float*)((const char*)ch3 + row * chStep);
	float* outputRowPtr = (float*)((char*)output + row * outputStep);

	// Assign the merged values into the output image
	outputRowPtr[3 * col + 0] = ch1RowPtr[col];  // First channel
	outputRowPtr[3 * col + 1] = ch2RowPtr[col];  // Second channel
	outputRowPtr[3 * col + 2] = ch3RowPtr[col];  // Third channel
}

void callCustomMergeKernel(const std::vector<cuda::GpuMat>& channels, cuda::GpuMat& output_image, dim3 block) {
	// Ensure the output image is allocated with the correct size and type (3 channels, 32-bit float)
	if (output_image.empty() || output_image.size() != channels[0].size() || output_image.type() != CV_32FC3) {
		output_image.create(channels[0].size(), CV_32FC3);
	}

	//dim3 block(32, 32);
	dim3 grid((channels[0].cols + block.x - 1) / block.x, (channels[0].rows + block.y - 1) / block.y);

	// Launch the kernel
	customMergeKernel << <grid, block >> > (
		reinterpret_cast<const float*>(channels[0].data),
		reinterpret_cast<const float*>(channels[1].data),
		reinterpret_cast<const float*>(channels[2].data),
		reinterpret_cast<float*>(output_image.data),
		channels[0].cols, channels[0].rows,
		channels[0].step, output_image.step);
}