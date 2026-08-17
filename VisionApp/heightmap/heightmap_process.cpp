#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/cudaarithm.hpp>
#include "heightmap_process.h"
#include "kernel_hm.h"
#include "param.h"

bool generateEnergyMat(const HeightmapParam& heightmap, const cv::cuda::GpuMat& src, cv::cuda::GpuMat& dst) 
{
	bool ok = true;

	cv::cuda::GpuMat lapLayer(src.size(), CV_32F, cv::Scalar(0));

	ok = getLaplacianLayer(heightmap.pyrLayer, src, lapLayer);
	if (!ok) return false;

    ok = gpu_region_energy_hm(lapLayer, dst, heightmap.kernelSize, heightmap.sigma);
    if (!ok) return false;

	return ok;
}

bool getLaplacianLayer(const uint8_t layer, const cv::cuda::GpuMat& src, cv::cuda::GpuMat& dst) 
{
    cv::cuda::GpuMat curr_img, downsampled, upsampled;
    curr_img = src;

    // The loop now runs from i = 0 to i = layer (inclusive), performing layer+1 iterations.
    for (size_t i = 0; i <= layer; ++i) {
        
        cv::cuda::pyrDown(curr_img, downsampled);
        cv::cuda::pyrUp(downsampled, upsampled);

        if (upsampled.size() != curr_img.size()) {
            cv::cuda::resize(upsampled, upsampled, curr_img.size(), 0, 0, cv::INTER_LINEAR);
        }

        if(i != layer) curr_img = downsampled;
    }

    // Compute the Laplacian as the difference between the downsampled image and its upsampled version.
    cv::cuda::subtract(curr_img, upsampled, dst);

    return true;
}

bool gpu_region_energy_hm(const cv::cuda::GpuMat& src, cv::cuda::GpuMat& dst, int kernelSize, float sigma)
{
    dst.create(src.size(), CV_32F);
    //callCombinedSquareAndFilterKernel(src, dst);
    callCombinedSquareAndFilterKernel(src, dst, kernelSize, sigma);

    return true;
}

bool updateMaxEnergyAndMaxIndex(const cv::cuda::GpuMat& currentEnergy, cv::cuda::GpuMat& maxEnergyMat, cv::cuda::GpuMat& maxIndexMat, const size_t index) 
{
    callUpdateMaxKernel(currentEnergy, maxEnergyMat, maxIndexMat, index);
    return true;
}

bool reconstructPyr(const cv::cuda::GpuMat& src, const HeightmapParam& heightmap, cv::cuda::GpuMat& dst)
{
    // Handle base layer (no reconstruction needed)
    if (heightmap.pyrLayer == 0) {
        dst = src.clone();
        return true;
    }

    cv::cuda::GpuMat current = src.clone();
    cv::cuda::GpuMat upsampled;

    // Reconstruct from top layer down to base layer
    for (size_t i = heightmap.pyrLayer; i > 0; --i) {
        cv::cuda::pyrUp(current, upsampled);

        // Fix size mismatch due to pyrDown rounding
        if (upsampled.size() != heightmap.pyramidSizes[i - 1]) {
            cv::cuda::resize(upsampled, upsampled, heightmap.pyramidSizes[i - 1], 0, 0, cv::INTER_LINEAR);
        }

        current = upsampled;
    }

    dst = current;
    return true;
}
