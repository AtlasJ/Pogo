#include <opencv2/opencv.hpp>
#include "kernel_hm.h"
#include "device_launch_parameters.h"
#include "cuda_runtime.h"
#include <opencv2/cudaarithm.hpp>
#include <iostream>
#include <vector>
#include <cmath>

//void callCombinedSquareAndFilterKernel(const cv::cuda::GpuMat& src, cv::cuda::GpuMat& dst) 
//{
//	dim3 block(32, 32);
//	dim3 grid((src.cols + block.x - 1) / block.x, (src.rows + block.y - 1) / block.y);
//
//	// Launch the kernel
//	combinedSquareAndFilterKernel << <grid, block >> > ((float*)src.data, (float*)dst.data, src.cols, src.rows, src.step, dst.step);
//	cudaDeviceSynchronize(); // Make sure the kernel finishes before using results
//}
//
//// Define the 5x5 Gaussian kernel in constant memory at global scope
//__constant__ float kernel[5][5] = 
//{
//	{0.0025f, 0.0125f, 0.02f, 0.0125f, 0.0025f},
//	{0.0125f, 0.0625f,  0.1f, 0.0625f, 0.0125f},
//	{  0.02f,    0.1f, 0.16f,    0.1f,   0.02f},
//	{0.0125f, 0.0625f,  0.1f, 0.0625f, 0.0125f},
//	{0.0025f, 0.0125f, 0.02f, 0.0125f, 0.0025f}
//};
//
//__global__ void combinedSquareAndFilterKernel(
//	const float* src, 
//	float* dst, 
//	int srcWidth, 
//	int srcHeight, 
//	size_t srcStep, 
//	size_t dstStep) 
//{
//	int row = blockIdx.y * blockDim.y + threadIdx.y;
//	int col = blockIdx.x * blockDim.x + threadIdx.x;
//
//	if (row >= srcHeight || col >= srcWidth) return;
//
//	float sum = 0.0f;
//
//	// Apply 5x5 convolution with squared values
//	for (int ky = -2; ky <= 2; ++ky) {
//		for (int kx = -2; kx <= 2; ++kx) {
//			int x = col + kx;
//			int y = row + ky;
//
//			if (x >= 0 && x < srcWidth && y >= 0 && y < srcHeight) {
//				const float* srcRowPtr = (const float*)((const char*)src + y * srcStep);
//				sum += static_cast<float>(srcRowPtr[x] * srcRowPtr[x]) * kernel[ky + 2][kx + 2];
//			}
//		}
//	}
//
//	float* dstRowPtr = (float*)((char*)dst + row * dstStep);
//	dstRowPtr[col] = sum;
//}

// Host function to compute a Gaussian kernel given kernelSize and sigma.
std::vector<float> computeGaussianKernel(int kernelSize, float sigma) {
    int half = kernelSize / 2;
    std::vector<float> kernel(kernelSize * kernelSize);
    float sum = 0.0f;
    float s = 2.0f * sigma * sigma;
    // Compute kernel values using the Gaussian formula
    for (int y = -half; y <= half; ++y) {
        for (int x = -half; x <= half; ++x) {
            float r = static_cast<float>(x * x + y * y);
            float val = expf(-r / s) / (3.14159265358979323846f * s);
            kernel[(y + half) * kernelSize + (x + half)] = val;
            sum += val;
        }
    }
    // Normalize the kernel so that the sum is 1.0
    for (size_t i = 0; i < kernel.size(); ++i) {
        kernel[i] /= sum;
    }
    return kernel;
}

// Host function wrapper that computes a Gaussian kernel with tunable size and sigma,
// uploads it to the device, and launches the kernel.
void callCombinedSquareAndFilterKernel(const cv::cuda::GpuMat& src, cv::cuda::GpuMat& dst,
    int kernelSize, float sigma)
{
    // Compute the Gaussian kernel on the host.
    std::vector<float> h_kernel = computeGaussianKernel(kernelSize, sigma);

    // Allocate device memory for the kernel.
    float* d_kernel = nullptr;
    size_t kernelBytes = h_kernel.size() * sizeof(float);
    cudaMalloc(&d_kernel, kernelBytes);

    // Copy the computed kernel from host to device.
    cudaMemcpy(d_kernel, h_kernel.data(), kernelBytes, cudaMemcpyHostToDevice);

    // Define the block and grid sizes for the kernel launch.
    dim3 block(32, 32);
    dim3 grid((src.cols + block.x - 1) / block.x, (src.rows + block.y - 1) / block.y);

    // Launch the CUDA kernel, passing the kernel pointer and kernel size.
    combinedSquareAndFilterKernel << <grid, block >> > (
        (float*)src.data,
        (float*)dst.data,
        src.cols,
        src.rows,
        src.step,
        dst.step,
        d_kernel,
        kernelSize);

    cudaDeviceSynchronize(); // Ensure kernel finishes before using results

    // Free the device memory allocated for the kernel.
    cudaFree(d_kernel);
}

// Modified CUDA kernel that accepts a Gaussian kernel of variable size.
__global__ void combinedSquareAndFilterKernel(
    const float* src,
    float* dst,
    int srcWidth,
    int srcHeight,
    size_t srcStep,
    size_t dstStep,
    const float* d_kernel,  // pointer to the Gaussian kernel in device memory
    int kernelSize)         // variable kernel size (must be odd)
{
    int row = blockIdx.y * blockDim.y + threadIdx.y;
    int col = blockIdx.x * blockDim.x + threadIdx.x;

    if (row >= srcHeight || col >= srcWidth)
        return;

    int half = kernelSize / 2;
    float sum = 0.0f;

    // Apply convolution over a (kernelSize x kernelSize) window
    for (int ky = -half; ky <= half; ++ky) {
        for (int kx = -half; kx <= half; ++kx) {
            int x = col + kx;
            int y = row + ky;

            if (x >= 0 && x < srcWidth && y >= 0 && y < srcHeight) {
                const float* srcRowPtr = (const float*)((const char*)src + y * srcStep);
                // Get the squared pixel value and weight it with the corresponding kernel value.
                float pixel = srcRowPtr[x];
                float weight = d_kernel[(ky + half) * kernelSize + (kx + half)];
                sum += pixel * pixel * weight;
            }
        }
    }

    float* dstRowPtr = (float*)((char*)dst + row * dstStep);
    dstRowPtr[col] = sum;
}

void callUpdateMaxKernel(const cv::cuda::GpuMat& currentEnergy, cv::cuda::GpuMat& maxEnergyMat, cv::cuda::GpuMat& maxIndexMat, const size_t index)
{
    // --- Update maxEnergyMat and maxIndexMat using custom kernel ---
    // Launch the kernel to update these matrices where current_energy is larger.
    int rows = maxEnergyMat.rows;
    int cols = maxEnergyMat.cols;
    // Assuming continuous memory (step = number of columns)
    int step = maxEnergyMat.step1();
    dim3 block(16, 16);
    dim3 grid((cols + block.x - 1) / block.x, (rows + block.y - 1) / block.y);

    // Launch the kernel. (current_energy.ptr<float>() returns device pointer)
    updateMaxKernel << <grid, block >> > (currentEnergy.ptr<float>(),
        maxEnergyMat.ptr<float>(),
        maxIndexMat.ptr<float>(),
        rows, cols, step,
        static_cast<unsigned short>(index));
    cudaDeviceSynchronize(); // Make sure the kernel finishes before using results
}

__global__ void updateMaxKernel(
    const float* current_energy,
    float* maxEnergy,
    float* maxIndex,
    int rows,
    int cols,
    int step,
    unsigned short currentIteration)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x < cols && y < rows)
    {
        int idx = y * step + x;
        float currE = current_energy[idx];
        float prevMax = maxEnergy[idx];
        if (currE > prevMax) {
            maxEnergy[idx] = currE;
            maxIndex[idx] = currentIteration;
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////
// cuda built-in example
//#include "cuda_runtime.h"
//#include "device_launch_parameters.h"
//
//#include <stdio.h>
//
//cudaError_t addWithCuda(int *c, const int *a, const int *b, unsigned int size);
//
//__global__ void addKernel(int *c, const int *a, const int *b)
//{
//    int i = threadIdx.x;
//    c[i] = a[i] + b[i];
//}
//
//int main()
//{
//    const int arraySize = 5;
//    const int a[arraySize] = { 1, 2, 3, 4, 5 };
//    const int b[arraySize] = { 10, 20, 30, 40, 50 };
//    int c[arraySize] = { 0 };
//
//    // Add vectors in parallel.
//    cudaError_t cudaStatus = addWithCuda(c, a, b, arraySize);
//    if (cudaStatus != cudaSuccess) {
//        fprintf(stderr, "addWithCuda failed!");
//        return 1;
//    }
//
//    printf("{1,2,3,4,5} + {10,20,30,40,50} = {%d,%d,%d,%d,%d}\n",
//        c[0], c[1], c[2], c[3], c[4]);
//
//    // cudaDeviceReset must be called before exiting in order for profiling and
//    // tracing tools such as Nsight and Visual Profiler to show complete traces.
//    cudaStatus = cudaDeviceReset();
//    if (cudaStatus != cudaSuccess) {
//        fprintf(stderr, "cudaDeviceReset failed!");
//        return 1;
//    }
//
//    return 0;
//}
//
//// Helper function for using CUDA to add vectors in parallel.
//cudaError_t addWithCuda(int *c, const int *a, const int *b, unsigned int size)
//{
//    int *dev_a = 0;
//    int *dev_b = 0;
//    int *dev_c = 0;
//    cudaError_t cudaStatus;
//
//    // Choose which GPU to run on, change this on a multi-GPU system.
//    cudaStatus = cudaSetDevice(0);
//    if (cudaStatus != cudaSuccess) {
//        fprintf(stderr, "cudaSetDevice failed!  Do you have a CUDA-capable GPU installed?");
//        goto Error;
//    }
//
//    // Allocate GPU buffers for three vectors (two input, one output)    .
//    cudaStatus = cudaMalloc((void**)&dev_c, size * sizeof(int));
//    if (cudaStatus != cudaSuccess) {
//        fprintf(stderr, "cudaMalloc failed!");
//        goto Error;
//    }
//
//    cudaStatus = cudaMalloc((void**)&dev_a, size * sizeof(int));
//    if (cudaStatus != cudaSuccess) {
//        fprintf(stderr, "cudaMalloc failed!");
//        goto Error;
//    }
//
//    cudaStatus = cudaMalloc((void**)&dev_b, size * sizeof(int));
//    if (cudaStatus != cudaSuccess) {
//        fprintf(stderr, "cudaMalloc failed!");
//        goto Error;
//    }
//
//    // Copy input vectors from host memory to GPU buffers.
//    cudaStatus = cudaMemcpy(dev_a, a, size * sizeof(int), cudaMemcpyHostToDevice);
//    if (cudaStatus != cudaSuccess) {
//        fprintf(stderr, "cudaMemcpy failed!");
//        goto Error;
//    }
//
//    cudaStatus = cudaMemcpy(dev_b, b, size * sizeof(int), cudaMemcpyHostToDevice);
//    if (cudaStatus != cudaSuccess) {
//        fprintf(stderr, "cudaMemcpy failed!");
//        goto Error;
//    }
//
//    // Launch a kernel on the GPU with one thread for each element.
//    addKernel<<<1, size>>>(dev_c, dev_a, dev_b);
//
//    // Check for any errors launching the kernel
//    cudaStatus = cudaGetLastError();
//    if (cudaStatus != cudaSuccess) {
//        fprintf(stderr, "addKernel launch failed: %s\n", cudaGetErrorString(cudaStatus));
//        goto Error;
//    }
//    
//    // cudaDeviceSynchronize waits for the kernel to finish, and returns
//    // any errors encountered during the launch.
//    cudaStatus = cudaDeviceSynchronize();
//    if (cudaStatus != cudaSuccess) {
//        fprintf(stderr, "cudaDeviceSynchronize returned error code %d after launching addKernel!\n", cudaStatus);
//        goto Error;
//    }
//
//    // Copy output vector from GPU buffer to host memory.
//    cudaStatus = cudaMemcpy(c, dev_c, size * sizeof(int), cudaMemcpyDeviceToHost);
//    if (cudaStatus != cudaSuccess) {
//        fprintf(stderr, "cudaMemcpy failed!");
//        goto Error;
//    }
//
//Error:
//    cudaFree(dev_c);
//    cudaFree(dev_a);
//    cudaFree(dev_b);
//    
//    return cudaStatus;
//}
