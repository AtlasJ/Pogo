#include "height_map_wrapper.h"

int height_map_wrapper(const std::string& input_path, cv::Mat& output_image, InputParam& input, HeightmapParam& heightmap, PostProcessParam& postProcess, OutputParam& output)
{
    input.inputFromMemory = false;
    input.folderPath = input_path;
    output.tiffOutput = &output_image;

    return height_map_wrapper(input, heightmap, postProcess, output);
}

int height_map_wrapper(const std::vector<cv::Mat>& input_images, cv::Mat& output_image, InputParam& input, HeightmapParam& heightmap, PostProcessParam& postProcess, OutputParam& output)
{
    if (input_images.size() == 0) { 
        std::cout << "input_images is empty";
        return -99; 
    }

    input.inputFromMemory = true;
    input.inputImages = &input_images;
    output.tiffOutput = &output_image;

    input.start = 0;
    input.end = input_images.size() - 1;
    input.step = 1;

    return height_map_wrapper(input, heightmap, postProcess, output);
}

int height_map_wrapper(InputParam& input, HeightmapParam& heightmap, PostProcessParam& postProcess, OutputParam& output)
{
    std::cout << "Running...CudaRuntime_Height_Map_v2" << std::endl;

    //Timer globalTimer;
    //globalTimer.start();

    bool OK = true;

    OK = input.init();
    if (!OK) return -1;

    OK = heightmap.init(input);
    if (!OK) return -2;

    OK = postProcess.init();
    if (!OK) return -3;

    OK = output.init(input);
    if (!OK) return -4;

    // Initialize matrices on the GPU.
    cv::cuda::GpuMat maxIndexMat(heightmap.size, CV_32F, cv::Scalar(0.0));
    cv::cuda::GpuMat maxEnergyMat(heightmap.size, CV_32F, cv::Scalar(std::numeric_limits<float>::lowest()));
    cv::cuda::GpuMat meanMat(heightmap.size, CV_32F, cv::Scalar(0.0));
    cv::cuda::GpuMat M2Mat(heightmap.size, CV_32F, cv::Scalar(0.0));
    cv::cuda::GpuMat stdDevMat(heightmap.size, CV_32F, cv::Scalar(0.0));

    int count = 0;

    // Loop over images.
    for (size_t i = input.start; i <= input.end; i += input.step) {
        count++;
        std::cout << "Processing image " << i << std::endl;

        // Load current image.
        cv::cuda::GpuMat currentImg;
        bool OK = input.inputFromMemory
            ? loadSingleInputImageFromMemory(input, i, currentImg)
            : loadSingleInputImage(input, i, currentImg);
        if (!OK) return -5;

        // Generate energy matrix for current image.
        cv::cuda::GpuMat currentEnergy;
        OK = generateEnergyMat(heightmap, currentImg, currentEnergy);
        if (!OK) return -6;

        if (output.wantVideo) {
            saveTiff(currentEnergy, 0.0, 30.0, output.imageFolderPath, std::to_string(i));
        }

        OK = updateMaxEnergyAndMaxIndex(currentEnergy, maxEnergyMat, maxIndexMat, i);
        if (!OK) return -7;

        // --- Update Welford's algorithm (per-pixel) ---
        cv::cuda::GpuMat diff, diffDiv, newMean, diff2, deltaProduct, updatedM2;
        cv::cuda::subtract(currentEnergy, meanMat, diff);
        diff.convertTo(diffDiv, diff.type(), 1.0 / count);
        cv::cuda::add(meanMat, diffDiv, newMean);
        cv::cuda::subtract(currentEnergy, newMean, diff2);
        cv::cuda::multiply(diff, diff2, deltaProduct);
        cv::cuda::add(M2Mat, deltaProduct, updatedM2);
        M2Mat = updatedM2; // Update M2 for Welford's algorithm.
        meanMat = newMean; // Update running mean.
    }

    // Compute standard deviation from M2Mat using (count - 1) for sample variance.
    if (count <= 1) return -8;

    cv::cuda::GpuMat variance, divisor(heightmap.size, CV_32F, cv::Scalar(count - 1));
    cv::cuda::divide(M2Mat, divisor, variance);
    cv::cuda::sqrt(variance, stdDevMat);

    // At this point, the following GPU matrices have been updated:
    // - maxEnergyMat: per-pixel maximum energy.
    // - maxIndexMat: iteration at which maximum energy occurred.
    // - meanMat: per-pixel mean energy.
    // - M2Mat: accumulated squared differences.
    // - stdDevMat: per-pixel standard deviation.


    // Post-Processing
    cv::cuda::GpuMat reconstructedMaxIndexMat, reconstructedMaxEnergyMat, reconstructedStdDevMat;
    if (postProcess.reconstructFirst)
    {
        // reconstruct first then remove
        OK = reconstructPyr(maxIndexMat, heightmap, reconstructedMaxIndexMat);
        if (!OK) return -9;

        OK = reconstructPyr(maxEnergyMat, heightmap, reconstructedMaxEnergyMat);
        if (!OK) return -10;

        OK = reconstructPyr(stdDevMat, heightmap, reconstructedStdDevMat);
        if (!OK) return -11;

        if (postProcess.wantRemoveByEnergy && postProcess.energyThreshold > 0) {
            OK = removeByEnergy(reconstructedMaxIndexMat, reconstructedMaxEnergyMat, postProcess.energyThreshold);
            if (!OK) return -12;
        }

        if (postProcess.wantRemoveByStdDev && postProcess.stdDevThreshold > 0) {
            OK = removeByStdDev(reconstructedMaxIndexMat, reconstructedStdDevMat, postProcess.stdDevThreshold);
            if (!OK) return -13;
        }
    }
    else {
        // remove first then reconstruct
        if (postProcess.wantRemoveByEnergy && postProcess.energyThreshold > 0) {
            OK = removeByEnergy(maxIndexMat, maxEnergyMat, postProcess.energyThreshold);
            if (!OK) return -14;
        }

        if (postProcess.wantRemoveByStdDev && postProcess.stdDevThreshold > 0) {
            OK = removeByStdDev(maxIndexMat, stdDevMat, postProcess.stdDevThreshold);
            if (!OK) return -15;
        }

        OK = reconstructPyr(maxIndexMat, heightmap, reconstructedMaxIndexMat);
        if (!OK) return -16;
    }

    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    //if (output.wantASC && postProcess.wantRemoveByRadius) {
    //    OK = convertToPointCloud(reconstructedMaxIndexMat, output, cloud_in);
    //    if (!OK) return -17;

    //    OK = removeByRadius(cloud_in, cloud_filtered, postProcess);
    //    if (!OK) return -18;
    //}

    if (output.wantRaw) {
        std::cout << "running saveRaw..." << std::endl;
        if (!saveRaw(reconstructedMaxIndexMat, output)) std::cout << "saveRaw failed" << std::endl;
        else std::cout << "successfully saveRaw to " + output.folderPath + "/" + output.fileName + "_raw.tiff" << std::endl;
    }

    if (output.wantColorMap) {
        std::cout << "running saveColorMap..." << std::endl;
        if (!saveColorMap(reconstructedMaxIndexMat, output)) std::cout << "saveColorMap failed" << std::endl;
        else std::cout << "successfully saveColorMap to " + output.folderPath + "/" + output.fileName + ".png" << std::endl;
    }

    if (output.wanttiff) {
        std::cout << "running saveTiff..." << std::endl;
        if (!saveTiff(reconstructedMaxIndexMat, output)) std::cout << "saveTiff failed" << std::endl;
        else std::cout << "successfully saveTiff to " + output.folderPath + "/" + output.fileName + ".tiff" << std::endl;
    }

    //if (output.wantASC) {
    //    if (postProcess.wantRemoveByRadius) {
    //        std::cout << "running saveASC..." << std::endl;
    //        if (!saveASC(cloud_filtered, output)) std::cout << "saveASC failed" << std::endl;
    //        else std::cout << "successfully saveASC to " + output.folderPath + "/" + output.fileName + ".asc" << std::endl;
    //    }
    //    else {
    //        std::cout << "running saveASC..." << std::endl;
    //        if (!saveASC(reconstructedMaxIndexMat, output)) std::cout << "saveASC failed" << std::endl;
    //        else std::cout << "successfully saveASC to " + output.folderPath + "/" + output.fileName + ".asc" << std::endl;
    //    }
    //}

    if (output.wantVideo) {
        if (!output.videoCommand.empty()) {
            std::cout << "running saveVideo..." << std::endl;
            if (!saveVideo(output.videoCommand)) std::cout << "saveVideo1 failed" << std::endl;
            else std::cout << "successfully saveVideo1 to " + output.videoPath << std::endl;
        }

        if (!output.videoCommand2.empty()) {
            std::cout << "running saveVideo..." << std::endl;
            if (!saveVideo(output.videoCommand2)) std::cout << "saveVideo2 failed" << std::endl;
            else std::cout << "successfully saveVideo2 to " + output.videoPath2 << std::endl;
        }
    }

    if (output.wantInfoTXT) {
        std::cout << "running saveInfoTXT..." << std::endl;
        if (!saveInfoTXT(input, heightmap, postProcess, output)) std::cout << "saveInfoTXT failed" << std::endl;
        else std::cout << "successfully saveInfoTXT to " + output.folderPath + "/" + output.fileName + ".txt" << std::endl;
    }

    //globalTimer.printDuration();
    return 0;
}
