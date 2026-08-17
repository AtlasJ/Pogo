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
#include <QString>
#include <QDir>
#include <QDebug>
#include <opencv2/ximgproc/include/opencv2/ximgproc/edge_filter.hpp>
#include <cmath>
#include <Eigen/Dense>
#include <limits>
#include <numeric>
#include <fstream>
#include "Depth_from_focus.h"

using namespace std;
using namespace cv;



// Function declarations
void computeTenengradStack(const std::vector<cv::Mat>& inputImages,
    std::vector<cv::Mat>& focusMeasures,
    const QString& debugFolder = "",
    bool debugMode = false);

void computeLaplacianStack(const std::vector<cv::Mat>& inputImages,
    std::vector<cv::Mat>& focusMeasures,
    const QString& debugFolder,
    bool debugMode);

void computeLaplacianStack(const std::vector<cv::Mat>& inputImages,
    std::vector<cv::Mat>& focusMeasures,
    const std::vector<cv::Rect>& rois,
    const QString& debugFolder,
    bool debugMode);

cv::Mat computeDoGFocusMeasure(const cv::Mat& gray, double sigma1, double sigma2);

void computeDoGStack(const std::vector<cv::Mat>& inputImages,
    std::vector<cv::Mat>& focusMeasures,
    const QString& debugFolder,
    bool debugMode,
    double sigma1 = 2.0,
    double sigma2 = 8.0);

void computeLaplacianPyramidStack(const std::vector<cv::Mat>& inputImages,
    std::vector<cv::Mat>& focusMeasures,
    const QString& debugFolder,
    bool debugMode,
    int levels = 3);

std::vector<cv::Mat> cropImageStackWithROI(const std::vector<cv::Mat>& inputImages,
    const cv::Rect& roi,
    const QString& saveFolder = "");

cv::Mat computeInitialHeightMap(const std::vector<cv::Mat>& focusVolume,
    const QString& debugFolder = "",
    bool debugMode = false);

cv::Mat computeLocalPatchHeightMap(const std::vector<cv::Mat>& focusVolume,
    const QString& debugFolder,
    bool debugMode,
    int kernelSize = 4);

void analyzeFocusStackToGraphs(const std::vector<cv::Mat>& stack,
    const QString& debugFolderPath);

void plotFocusCurveAtPatch(const std::vector<cv::Mat>& focusVolume,
    int x, int y,
    int kernelSize,
    const QString& debugFolder,
    const QString& title = "Patch Mean Curve");

void plotAllPatchCurves(const std::vector<cv::Mat>& focusVolume, int kernelSize, const QString& debugFolder);

cv::Mat computeMADMask(const std::vector<cv::Mat>& focusVolume,
    const QString& debugFolder,
    bool debugMode,
    double threshold = 0.1);

cv::Mat computePeakCountMask(const std::vector<cv::Mat>& focusVolume,
    const QString& debugFolder,
    bool debugMode,
    int kernelSize = 10,
    int peakThreshold = 2,
    float peakProminence = 0.01f);

void applyGuidedFilter(const cv::Mat& guide, const cv::Mat& src, cv::Mat& dst,
    int radius = 8, double eps = 0.01);

int findBestFocusedSlice(const std::vector<cv::Mat>& images);

cv::Mat erodeHeightMap8U(const cv::Mat& heightMap8U, int iterations, int kernelSize = 3);

void preprocessFocusSlices(std::vector<cv::Mat>& focusVolume,
    const QString& debugFolder,
    bool debugMode,
    int dilateIter = 1,
    int erodeExtra = 1);

cv::Mat reconstructAndSaveFocusedImage(const std::vector<cv::Mat>& inputImages,
    const std::vector<cv::Mat>& focusMeasures,
    const QString& outputPath);

cv::Mat autoThresholdWireMask(const cv::Mat& dogMask, const QString& debugFolder = "");

cv::Mat smoothHeightMapDirectionally(const cv::Mat& heightMap,
    int kernelSize,
    int angleBins,
    const QString& debugFolder,
    bool debugMode);

void saveHeightMapDebug(const cv::Mat& heightMap, const QString& debugFolder, const QString& name = "final_height");
void save16BitHeightMapDebug(const cv::Mat& heightMap, const QString& debugFolder, const QString& name = "final_height");
void normalizeZSlice8UTo16U(
    const cv::Mat& heightMap8U,  // input CV_8UC1 Z-slice index map
    double zMin,                 // real-world min Z (e.g., 0.0)
    double zMax,                 // real-world max Z (e.g., 9.0)
    cv::Mat& out16U              // output CV_16UC1 normalized map
);

//get Surface Height
double computeFFTSharpness(const cv::Mat& patch, double freqThreshold = 15.0);
double computeStddevFocus(const cv::Mat& roi);
Eigen::Vector3d fitPlaneFromPoints(const std::vector<cv::Point3d>& points);
void renderPlaneToMat16U(
    const Eigen::Vector3d& plane,
    cv::Size fullSize,
    cv::Mat& out16U,
    bool normalize,
    int numZSlices = -1,
    const cv::Rect& roi = cv::Rect());
cv::Mat generateFittedPlaneFromZStack(
    const std::vector<cv::Mat>& zStack,
    const SurfaceROI surfaceRoi,
    double freqThreshold = 15.0,
    bool normalizeOutput = true);

void analyzeFocusStackToGraphsPerROI(
    const std::vector<cv::Mat>& stack,
    const std::vector<cv::Rect>& rois,
    const QString& debugFolderPath);

cv::Mat stackHeightMapsByMax(const std::vector<cv::Mat>& heightMaps);

//1. get Surface Region, One Surface Roi With a vector of other plane Rois => surfaceMap
//2. get Feature Region, give an ROI to perform the Feature HeightMap Generation => featureMap
//3. Add All Map together by selecting the highest Height for each pixel coordinate

void depth_from_focus_testing(const vector<Mat>& input_images, cuda::GpuMat& final_output_image, size_t availableMemory) {
	
    QString debugFolderTest20 = "C:/Users/leong/OneDrive/Desktop/OnGoing Projects/IM380/DebugDFF_test20/";
    QString debugFolderOct09 = "C:/Users/leong/OneDrive/Desktop/OnGoing Projects/IM380/DebugDFF_roiOct09_2/";
    cv::Rect roiOct09(3372, 3330, 624, 342); //partial wire
    cv::Rect roiOct09_2(1644, 5526, 1524, 1200); // small unit
    cv::Rect roiOct09_3(918, 1802, 274, 244); //background
    cv::Rect roiOct09_4(718, 1522, 246, 234); //background with fiducial

    cv::Rect roiTest20(3360, 2220, 612, 244);
    cv::Rect smallRoi(280, 60, 30, 30);

   
    auto croppedImages = cropImageStackWithROI(input_images, roiOct09_2, debugFolderOct09);

    // ========================================================= Surface Extration to HeightMap =================================================================
    QString debugFolderSurfaceHeightMap = "C:/Users/leong/OneDrive/Desktop/OnGoing Projects/IM380/DebugDFF_surfaceHeightMap/";
    cv::Rect roi1(50, 50, 50, 50);
    cv::Rect roi2(1350, 50, 50, 50);
    cv::Rect roi3(50, 1100, 50, 50);
    cv::Rect roi4(1350, 1100, 50, 50);

    SurfaceROI surfaceRoi;
    surfaceRoi.surfaceRoi = cv::Rect(0, 0, 1524, 1200);
    std::vector<cv::Rect> rois;
    rois.push_back(roi1);
    rois.push_back(roi2);
    rois.push_back(roi3);
    rois.push_back(roi4);
    surfaceRoi.planeRois = rois;
    auto surfaceHeight = generateFittedPlaneFromZStack(croppedImages, surfaceRoi);
    save16BitHeightMapDebug(surfaceHeight, debugFolderSurfaceHeightMap);



    analyzeFocusStackToGraphsPerROI(croppedImages, rois, debugFolderSurfaceHeightMap);

    // ========================================================= Feature Extration to HeightMap =================================================================
    //1. compute laplacian stack
    QString debugFolderFocusMeasure = "C:/Users/leong/OneDrive/Desktop/OnGoing Projects/IM380/DebugDFF_focusMeasures/";
    std::vector<cv::Mat> focusMeasures;
    //computeTenengradStack(croppedImages, focusMeasures, debugFolderFocusMeasure, true);
    computeLaplacianStack(croppedImages, focusMeasures, debugFolderFocusMeasure, true);
    //computeDoGStack(croppedImages, focusMeasures, debugFolderFocusMeasure, true);
    //computeLaplacianPyramidStack(croppedImages, focusMeasures, debugFolderFocusMeasure, true);

   /* QString debugFolderPreprocess = "C:/Users/leong/OneDrive/Desktop/OnGoing Projects/IM380/DebugDFF_preprocess/";
    preprocessFocusSlices(focusMeasures, debugFolderPreprocess, true);*/

    QString debugFolderFocusCurve = "C:/Users/leong/OneDrive/Desktop/OnGoing Projects/IM380/DebugDFF_FocusCurve/";
    //plotAllPatchCurves(focusMeasures, 5, debugFolderFocusCurve);
    //plotFocusCurveAtPatch(focusMeasures, 483, 27, 5, debugFolderFocusCurve);

    //2. get focus image and perform DOG to get mask
    QString debugFolderInitialMap = "C:/Users/leong/OneDrive/Desktop/OnGoing Projects/IM380/DebugDFF_initialHeightMap/";
    auto focusedImage = reconstructAndSaveFocusedImage(croppedImages, focusMeasures, debugFolderInitialMap);
    auto dogMask = computeDoGFocusMeasure(focusedImage, 2, 8);
    dogMask = autoThresholdWireMask(dogMask, debugFolderInitialMap);

    //3. get initial HeightMap and Mask using DoG
    auto initialHeightMap = computeInitialHeightMap(focusMeasures, debugFolderInitialMap, true);
    cv::Mat finalHeightMap;
    initialHeightMap.copyTo(finalHeightMap, dogMask);

    //4. guided filter
    cv::Mat guide = croppedImages[findBestFocusedSlice(croppedImages)];
    cv::Mat smoothed;
    applyGuidedFilter(guide, finalHeightMap, smoothed);
    //smoothed = smoothHeightMapDirectionally(smoothed, 5, 8, debugFolderInitialMap, true);
    saveHeightMapDebug(smoothed, debugFolderInitialMap);

    //QString debugFolderLocalPathMap = "C:/Users/leong/OneDrive/Desktop/OnGoing Projects/IM380/DebugDFF_localPatchHeightMap/";
    //auto patchHeightMap = computeLocalPatchHeightMap(focusMeasures, debugFolderLocalPathMap, true, 5);
    //patchHeightMap.copyTo(finalHeightMap, dogMask);
    //applyGuidedFilter(guide, finalHeightMap, smoothed);

    ////smoothed = smoothHeightMapDirectionally(smoothed, 5, 8, debugFolderLocalPathMap, true);
    //saveHeightMapDebug(smoothed, debugFolderLocalPathMap);

    QString debugFolderGraph = "C:/Users/leong/OneDrive/Desktop/OnGoing Projects/IM380/DebugDFF_Graph/";
    analyzeFocusStackToGraphs(focusMeasures, debugFolderGraph);
}

void depth_from_focus_surface(const std::vector<cv::Mat>& input_images, cv::Mat& output_image, const std::vector<SurfaceROI> surfaceRois)
{
    qDebug() << "depth_from_focus_surface";
    std::vector<cv::Mat> surfaceHeightMaps;
    for (int i = 0; i < surfaceRois.size(); i++)
    {
        auto surfaceHeightMap = generateFittedPlaneFromZStack(input_images, surfaceRois[i]);
        surfaceHeightMaps.push_back(surfaceHeightMap);
    }
   
    output_image = stackHeightMapsByMax(surfaceHeightMaps);

    QString debugFolderSurfaceHeightMap = "C:/Users/leong/OneDrive/Desktop/OnGoing Projects/IM380/DebugDFF_depth_from_focus_surface/";
    save16BitHeightMapDebug(output_image, debugFolderSurfaceHeightMap);
}

void depth_from_focus_feature(const std::vector<cv::Mat>& input_images, cv::Mat& output_image, const std::vector<FeatureROI> featureRois)
{
    std::vector<cv::Rect> rois;
    for (auto fRoi : featureRois)
    {
        rois.push_back(fRoi.featureRoi);
    }
    qDebug() << "depth_from_focus_feature";
    QString debugFolderFocusMeasure = "C:/Users/leong/OneDrive/Desktop/OnGoing Projects/IM380/DebugDFF_focusMeasures/";
    std::vector<cv::Mat> focusMeasures;
    computeLaplacianStack(input_images, focusMeasures, rois, debugFolderFocusMeasure, true);

    //2. get focus image and perform DOG to get mask
    QString debugFolderInitialMap = "C:/Users/leong/OneDrive/Desktop/OnGoing Projects/IM380/DebugDFF_initialHeightMap/";
    auto focusedImage = reconstructAndSaveFocusedImage(input_images, focusMeasures, debugFolderInitialMap);
    auto dogMask = computeDoGFocusMeasure(focusedImage, 2, 8);
    dogMask = autoThresholdWireMask(dogMask, debugFolderInitialMap);

    //3. get initial HeightMap and Mask using DoG
    auto initialHeightMap = computeInitialHeightMap(focusMeasures, debugFolderInitialMap, true);
    cv::Mat finalHeightMap;
    initialHeightMap.copyTo(finalHeightMap, dogMask);

    //4. guided filter
    cv::Mat guide = input_images[findBestFocusedSlice(input_images)];
    cv::Mat smoothed;
    applyGuidedFilter(guide, finalHeightMap, smoothed);

    normalizeZSlice8UTo16U(smoothed, 0, input_images.size() - 1, output_image);
    save16BitHeightMapDebug(output_image, debugFolderInitialMap);
}

void depth_from_focus(const std::vector<cv::Mat>& input_images, cv::Mat& output_image, const std::vector<SurfaceROI> surfaceRois, const std::vector<FeatureROI> featureRois)
{
    //cropped Big Images First, can be removed later
    QString debugFolder = "C:/Users/leong/OneDrive/Desktop/OnGoing Projects/IM380/DebugDFF_roiOct09_2/";
    //cv::Rect roi_unit(1644, 5526, 1524, 1200); // small unit
    cv::Rect roi_unit(2882, 2560, 870, 490); // stacked unit

    auto croppedImages = cropImageStackWithROI(input_images, roi_unit, debugFolder);

    


    //initialize Surface Rois, can be removed later
    SurfaceROI background;
    background.surfaceRoi = cv::Rect(0, 0, 1524, 1200);
    cv::Rect roi1(50, 50, 50, 50);
    cv::Rect roi2(1350, 50, 50, 50);
    cv::Rect roi3(50, 1100, 50, 50);
    cv::Rect roi4(1350, 1100, 50, 50);
    background.planeRois.push_back(roi1);
    background.planeRois.push_back(roi2);
    background.planeRois.push_back(roi3);
    background.planeRois.push_back(roi4);

    SurfaceROI dieSurface;
    dieSurface.surfaceRoi = cv::Rect(414, 227, 704, 748);
    cv::Rect roi5(462, 271, 50, 50);
    cv::Rect roi6(993, 277, 50, 50);
    cv::Rect roi7(1000, 848, 50, 50);
    cv::Rect roi8(488, 849, 50, 50);
    dieSurface.planeRois.push_back(roi5);
    dieSurface.planeRois.push_back(roi6);
    dieSurface.planeRois.push_back(roi7);
    dieSurface.planeRois.push_back(roi8);

    std::vector<SurfaceROI> tempSurfaceRois;
    tempSurfaceRois.push_back(background);
    tempSurfaceRois.push_back(dieSurface);

    cv::Mat surfaceHeightMap;
    //depth_from_focus_surface(croppedImages, surfaceHeightMap, tempSurfaceRois);

    cv::Mat featureHeightMap;
    FeatureROI featureRoi;
    //featureRoi.featureRoi = cv::Rect(0, 0, 1524, 1200); .//small unit
    featureRoi.featureRoi = cv::Rect(0, 0, 870, 490);

    std::vector<FeatureROI> tempFeatureRois;
    tempFeatureRois.push_back(featureRoi);

    depth_from_focus_feature(croppedImages, featureHeightMap, tempFeatureRois);

    std::vector<cv::Mat> HeightMaps;
    HeightMaps.push_back(surfaceHeightMap);
    HeightMaps.push_back(featureHeightMap);
   auto finalHeightMap = stackHeightMapsByMax(HeightMaps);

   QString debugFolderFinalMap = "C:/Users/leong/OneDrive/Desktop/OnGoing Projects/IM380/DebugDFF_FinalMap/";
   save16BitHeightMapDebug(featureHeightMap, debugFolderFinalMap);
}

std::vector<cv::Mat> cropImageStackWithROI(const std::vector<cv::Mat>& inputImages,
    const cv::Rect& roi,
    const QString& saveFolder) {
    std::vector<cv::Mat> croppedImages;

    if (!saveFolder.isEmpty()) {
        QDir().mkpath(saveFolder); // Create folder if it doesn't exist
    }

    for (size_t i = 0; i < inputImages.size(); ++i) {
        const cv::Mat& img = inputImages[i];

        // Validate ROI bounds
        cv::Rect validROI = roi & cv::Rect(0, 0, img.cols, img.rows);
        if (validROI.width <= 0 || validROI.height <= 0) {
            qDebug() << "Invalid ROI for index" << i << ": Skipping.";
            continue;
        }

        cv::Mat cropped = img(validROI).clone();
        croppedImages.push_back(cropped);

        if (!saveFolder.isEmpty()) {
            QString filename = QString("%1/cropped_%2.png").arg(saveFolder).arg(i, 3, 10, QChar('0'));
            cv::imwrite(filename.toStdString(), cropped);
        }
    }

    return croppedImages;
}


void computeTenengradStack(const std::vector<cv::Mat>& inputImages,
    std::vector<cv::Mat>& focusMeasures,
    const QString& debugFolder,
    bool debugMode) {
    QDir().mkpath(debugFolder); // Auto-create folder if it doesn't exist

    for (size_t i = 0; i < inputImages.size(); ++i) {
        const cv::Mat& img = inputImages[i];
        cv::Mat gradX, gradY, gradMag, floatImg;

        img.convertTo(floatImg, CV_32F, 1.0 / 255.0);
        cv::Sobel(floatImg, gradX, CV_32F, 1, 0, 3);
        cv::Sobel(floatImg, gradY, CV_32F, 0, 1, 3);
        cv::magnitude(gradX, gradY, gradMag);

        focusMeasures.push_back(gradMag);

        if (debugMode && !debugFolder.isEmpty()) {
            cv::Mat debugOut;
            cv::normalize(gradMag, debugOut, 0, 255, cv::NORM_MINMAX);
            debugOut.convertTo(debugOut, CV_8U);

            QString fileName = QString("tenengrad_%1.png").arg(i, 3, 10, QChar('0'));
            QString fullPath = debugFolder + "/" + fileName;
            cv::imwrite(fullPath.toStdString(), debugOut);
        }
    }
}

void computeLaplacianStack(const std::vector<cv::Mat>& inputImages,
    std::vector<cv::Mat>& focusMeasures,
    const QString& debugFolder,
    bool debugMode) {
    QDir().mkpath(debugFolder); // Auto-create folder if it doesn't exist

    for (size_t i = 0; i < inputImages.size(); ++i) {
        const cv::Mat& img = inputImages[i];
        cv::Mat floatImg, lap, absLap;

        // Convert to float
        img.convertTo(floatImg, CV_32F, 1.0 / 255.0);

        // Apply Laplacian
        cv::Laplacian(floatImg, lap, CV_32F, 11);  // kernel size = 3
        cv::absdiff(lap, 0, absLap);             // Absolute value of Laplacian

        focusMeasures.push_back(absLap);

        if (debugMode && !debugFolder.isEmpty()) {
            cv::Mat debugOut;
            cv::normalize(absLap, debugOut, 0, 255, cv::NORM_MINMAX);
            debugOut.convertTo(debugOut, CV_8U);

            QString fileName = QString("laplacian_%1.png").arg(i, 3, 10, QChar('0'));
            QString fullPath = debugFolder + "/" + fileName;
            cv::imwrite(fullPath.toStdString(), debugOut);
        }
    }
}

void computeLaplacianStack(const std::vector<cv::Mat>& inputImages,
    std::vector<cv::Mat>& focusMeasures,
    const std::vector<cv::Rect>& rois,
    const QString& debugFolder,
    bool debugMode)
{
    QDir().mkpath(debugFolder);
    focusMeasures.clear();

    for (size_t i = 0; i < inputImages.size(); ++i) {
        const cv::Mat& img = inputImages[i];
        cv::Mat floatImg, lap, absLap, roiOutput;

        // Convert to float
        img.convertTo(floatImg, CV_32F, 1.0 / 255.0);
        absLap = cv::Mat::zeros(img.size(), CV_32F);  // Init with zeros

        for (const auto& roi : rois) {
            if ((roi & cv::Rect(0, 0, img.cols, img.rows)).area() == 0)
                continue;

            cv::Mat roiPatch = floatImg(roi);
            cv::Mat lapROI;

            // Apply Laplacian to the ROI
            cv::Laplacian(roiPatch, lapROI, CV_32F, 11);  // Use your kernel size
            cv::absdiff(lapROI, 0, lapROI);

            // Copy back the result into the output
            lapROI.copyTo(absLap(roi));
        }

        focusMeasures.push_back(absLap);

        // Debug save
        if (debugMode && !debugFolder.isEmpty()) {
            cv::Mat debugOut;
            cv::normalize(absLap, debugOut, 0, 255, cv::NORM_MINMAX);
            debugOut.convertTo(debugOut, CV_8U);

            QString fileName = QString("laplacian_roi_%1.png").arg(i, 3, 10, QChar('0'));
            QString fullPath = debugFolder + "/" + fileName;
            cv::imwrite(fullPath.toStdString(), debugOut);
        }
    }
}

void computeDoGStack(const std::vector<cv::Mat>& inputImages,
    std::vector<cv::Mat>& focusMeasures,
    const QString& debugFolder,
    bool debugMode,
    double sigma1,
    double sigma2) {
    QDir().mkpath(debugFolder);

    for (size_t i = 0; i < inputImages.size(); ++i) {
        cv::Mat img = inputImages[i];
        img.convertTo(img, CV_32F, 1.0 / 255.0);
        cv::Mat dog = computeDoGFocusMeasure(img, sigma1, sigma2);
        focusMeasures.push_back(dog);

        if (debugMode) {
            cv::Mat vis;
            cv::normalize(dog, vis, 0, 255, cv::NORM_MINMAX);
            vis.convertTo(vis, CV_8U);
            QString path = debugFolder + QString("/dog_%1.png").arg(i, 3, 10, QChar('0'));
            cv::imwrite(path.toStdString(), vis);
        }
    }
}

cv::Mat computeDoGFocusMeasure(const cv::Mat& gray, double sigma1, double sigma2) {
    cv::Mat blur1, blur2, dog;
    cv::GaussianBlur(gray, blur1, cv::Size(0, 0), sigma1);
    cv::GaussianBlur(gray, blur2, cv::Size(0, 0), sigma2);
    cv::subtract(blur1, blur2, dog, cv::noArray(), CV_32F); // keep in float for precision
    cv::max(dog, 0, dog);
    return dog;
}

cv::Mat computeInitialHeightMap(const std::vector<cv::Mat>& focusVolume,
    const QString& debugFolder,
    bool debugMode) {
    if (focusVolume.empty()) return {};

    int rows = focusVolume[0].rows;
    int cols = focusVolume[0].cols;
    int depth = static_cast<int>(focusVolume.size());

    cv::Mat heightMap = cv::Mat::zeros(rows, cols, CV_8U);   // stores best Z index (0–255)
    cv::Mat maxResponse = cv::Mat::zeros(rows, cols, CV_32F);

    for (int z = 0; z < depth; ++z) {
        const cv::Mat& current = focusVolume[z];

        for (int y = 0; y < rows; ++y) {
            const float* currRow = current.ptr<float>(y);
            float* maxRow = maxResponse.ptr<float>(y);
            uchar* outRow = heightMap.ptr<uchar>(y);

            for (int x = 0; x < cols; ++x) {
                if (currRow[x] > maxRow[x]) {
                    maxRow[x] = currRow[x];
                    outRow[x] = static_cast<uchar>(z);  // Store best focus slice index
                }
            }
        }
    }

    // Optional debug output
    if (debugMode && !debugFolder.isEmpty()) {
        QDir().mkpath(debugFolder);

        // Color-mapped version
        Mat colorMap;
        normalize(heightMap, colorMap, 0, 255, NORM_MINMAX);
        applyColorMap(colorMap, colorMap, COLORMAP_RAINBOW);
        cv::imwrite((debugFolder + "/initial_height_colormap.png").toStdString(), colorMap);

        // Raw grayscale version
        cv::imwrite((debugFolder + "/initial_height.png").toStdString(), heightMap);
    }

    return heightMap;
}

cv::Mat computeLocalPatchHeightMap(const std::vector<cv::Mat>& focusVolume,
    const QString& debugFolder,
    bool debugMode,
    int kernelSize) {
    if (focusVolume.empty()) return {};

    int rows = focusVolume[0].rows;
    int cols = focusVolume[0].cols;
    int depth = static_cast<int>(focusVolume.size());
    int radius = kernelSize / 2;

    cv::Mat heightMap = cv::Mat::zeros(rows, cols, CV_8U);
    cv::Mat maxResponse = cv::Mat::zeros(rows, cols, CV_32F);

    for (int z = 0; z < depth; ++z) {
        const cv::Mat& slice = focusVolume[z];

        for (int y = 0; y < rows; ++y) {
            for (int x = 0; x < cols; ++x) {
                // Define the patch ROI, clamp to borders
                int x_start = std::max(0, x - radius);
                int x_end = std::min(cols, x + radius + 1);
                int y_start = std::max(0, y - radius);
                int y_end = std::min(rows, y + radius + 1);

                cv::Rect roi(x_start, y_start, x_end - x_start, y_end - y_start);
                cv::Scalar mean, stddev;
                cv::meanStdDev(slice(roi), mean, stddev);

                float avgValue = static_cast<float>(stddev[0]);
                if (avgValue > maxResponse.at<float>(y, x)) {
                    maxResponse.at<float>(y, x) = avgValue;
                    heightMap.at<uchar>(y, x) = static_cast<uchar>(z);
                }
            }
        }
    }

    // Debug output
    if (debugMode && !debugFolder.isEmpty()) {
        QDir().mkpath(debugFolder);

        // Pseudo-color
        cv::Mat colorMap;
        cv::normalize(heightMap, colorMap, 0, 255, cv::NORM_MINMAX);
        cv::applyColorMap(colorMap, colorMap, cv::COLORMAP_RAINBOW);
        cv::imwrite((debugFolder + "/initial_height_patch_colormap.png").toStdString(), colorMap);

        // Raw grayscale
        cv::imwrite((debugFolder + "/initial_height_patch.png").toStdString(), heightMap);
    }

    return heightMap;
}

void analyzeFocusStackToGraphs(const std::vector<cv::Mat>& stack,
    const QString& debugFolderPath) {
    if (stack.empty()) {
        qDebug() << "Image stack is empty!";
        return;
    }

    QDir().mkpath(debugFolderPath);

    std::vector<double> means, stddevs;

    for (const auto& img : stack) {
        cv::Scalar mean, stddev;
        cv::meanStdDev(img, mean, stddev);
        means.push_back(mean[0]);
        stddevs.push_back(stddev[0]);
    }

    auto drawGraph = [](const std::vector<double>& values, const std::string& label,
        int width = 800, int height = 400) -> cv::Mat {

            cv::Mat plot(height, width, CV_8UC3, cv::Scalar(255, 255, 255));
            double minVal = *std::min_element(values.begin(), values.end());
            double maxVal = *std::max_element(values.begin(), values.end());
            double range = maxVal - minVal + 1e-6;

            int margin = 50;
            int graphW = width - 2 * margin;
            int graphH = height - 2 * margin;
            int n = static_cast<int>(values.size());

            // Plot line
            for (int i = 1; i < n; ++i) {
                int x0 = margin + (i - 1) * graphW / (n - 1);
                int x1 = margin + i * graphW / (n - 1);
                int y0 = margin + static_cast<int>(graphH * (1.0 - (values[i - 1] - minVal) / range));
                int y1 = margin + static_cast<int>(graphH * (1.0 - (values[i] - minVal) / range));
                cv::line(plot, { x0, y0 }, { x1, y1 }, cv::Scalar(0, 0, 255), 2);
            }

            // Draw cross and label for each point
            for (int i = 0; i < n; ++i) {
                int x = margin + i * graphW / (n - 1);
                int y = margin + static_cast<int>(graphH * (1.0 - (values[i] - minVal) / range));

                // Draw cross
                int crossSize = 4;
                cv::line(plot, { x - crossSize, y - crossSize }, { x + crossSize, y + crossSize }, cv::Scalar(0, 0, 0), 1);
                cv::line(plot, { x - crossSize, y + crossSize }, { x + crossSize, y - crossSize }, cv::Scalar(0, 0, 0), 1);

                // Draw index and mean value
                std::string labelText = std::to_string(i) + " (" + cv::format("%.1f", values[i]*100) + ")";
                qDebug() << "value:" << values[i];
                cv::putText(plot, labelText, { x - 10, y - 8 }, cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(0, 0, 0), 1);
            }

            // Axes
            cv::line(plot, { margin, margin }, { margin, height - margin }, cv::Scalar(0, 0, 0), 1);
            cv::line(plot, { margin, height - margin }, { width - margin, height - margin }, cv::Scalar(0, 0, 0), 1);

            // Axis labels and title
            cv::putText(plot, "Z-Slice Index", { width / 2 - 50, height - 10 },
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
            cv::putText(plot, "Focus Value", { 5, margin - 10 },
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
            cv::putText(plot, label, { margin, margin - 25 },
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 1);

            return plot;
        };

    // Generate and save plots
    qDebug() << "Mean:";
    cv::Mat meanPlot = drawGraph(means, "Mean Intensity per Z-Slice");

    qDebug() << "================";
    qDebug() << "Std:";
    cv::Mat stdPlot = drawGraph(stddevs, "StdDev Intensity per Z-Slice");

    cv::imwrite((debugFolderPath + "/mean_plot.png").toStdString(), meanPlot);
    cv::imwrite((debugFolderPath + "/stddev_plot.png").toStdString(), stdPlot);

    qDebug() << "Graphs saved to:" << debugFolderPath;
}

cv::Mat computeMADMask(const std::vector<cv::Mat>& focusVolume,
    const QString& debugFolder,
    bool debugMode,
    double threshold) {
    if (focusVolume.empty()) return {};

    int rows = focusVolume[0].rows;
    int cols = focusVolume[0].cols;
    int depth = static_cast<int>(focusVolume.size());
    const float epsilon = 1e-6f;

    cv::Mat mask(rows, cols, CV_8U, cv::Scalar(0)); // 0 = unreliable, 255 = reliable

    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            std::vector<float> curve;
            curve.reserve(depth);

            // Extract curve at pixel (x, y)
            for (int z = 0; z < depth; ++z) {
                curve.push_back(focusVolume[z].at<float>(y, x));
            }

            // Median of the curve
            std::vector<float> sortedCurve = curve;
            std::nth_element(sortedCurve.begin(), sortedCurve.begin() + depth / 2, sortedCurve.end());
            float median = sortedCurve[depth / 2];

            // Compute MAD
            std::vector<float> deviations;
            deviations.reserve(depth);
            for (float v : curve) {
                deviations.push_back(std::abs(v - median));
            }
            std::nth_element(deviations.begin(), deviations.begin() + depth / 2, deviations.end());
            float mad = deviations[depth / 2];

            float maxVal = *std::max_element(curve.begin(), curve.end());
            float c_mad = mad / (maxVal + epsilon);

            if (c_mad <= threshold) {
                mask.at<uchar>(y, x) = 255;
            }
        }
    }

    // Optional debug output
    if (debugMode && !debugFolder.isEmpty()) {
        QDir().mkpath(debugFolder);
        QString path = debugFolder + "/mad_mask.png";
        cv::imwrite(path.toStdString(), mask);
    }

    return mask;
}

void saveHeightMapDebug(const cv::Mat& heightMap, const QString& debugFolder, const QString& name) {
    if (heightMap.empty()) return;

    QDir().mkpath(debugFolder);

    QString basePath = debugFolder + "/" + name;

    // Save grayscale visualization (8-bit normalized)
    cv::Mat normalized;
    cv::normalize(heightMap, normalized, 0, 255, cv::NORM_MINMAX);
    normalized.convertTo(normalized, CV_8U);
    cv::imwrite((basePath + ".png").toStdString(), normalized);

    // Save colormap
    cv::Mat colorMap;
    cv::applyColorMap(normalized, colorMap, cv::COLORMAP_JET);
    cv::imwrite((basePath + "_colormap.png").toStdString(), colorMap);

    // Save full-resolution 16-bit TIFF
    double minVal, maxVal;
    cv::minMaxLoc(heightMap, &minVal, &maxVal);

    cv::Mat scaled, heightMap16U;
    heightMap.convertTo(scaled, CV_64F);
    if (maxVal > minVal) {
        scaled = (scaled - minVal) / (maxVal - minVal) * 65535.0;
    }
    else {
        scaled = cv::Mat::zeros(heightMap.size(), CV_64F);
    }
    scaled.convertTo(heightMap16U, CV_16U);
    cv::imwrite((basePath + ".tiff").toStdString(), heightMap16U);
}
void save16BitHeightMapDebug(const cv::Mat& heightMap, const QString& debugFolder, const QString& name)
{
    if (heightMap.empty()) return;

    QDir().mkpath(debugFolder);
    QString basePath = debugFolder + "/" + name;

    // Save grayscale visualization (8-bit normalized)
    cv::Mat normalized;
    cv::normalize(heightMap, normalized, 0, 255, cv::NORM_MINMAX);
    normalized.convertTo(normalized, CV_8U);
    cv::imwrite((basePath + ".png").toStdString(), normalized);

    // Save colormap
    cv::Mat colorMap;
    cv::applyColorMap(normalized, colorMap, cv::COLORMAP_JET);
    cv::imwrite((basePath + "_colormap.png").toStdString(), colorMap);

    // Save original 16-bit TIFF
    cv::imwrite((basePath + ".tiff").toStdString(), heightMap);

    // ===== Save full PLY with normals, color, and u/v =====
    QString plyPath = basePath + ".ply";
    std::ofstream ofs(plyPath.toStdString());
    if (!ofs.is_open()) {
        qWarning("Failed to open PLY file for writing.");
        return;
    }

    float scale = 0.005f; // mm to meters
    int validPoints = 0;
    for (int y = 0; y < heightMap.rows; ++y)
        for (int x = 0; x < heightMap.cols; ++x)
            if (heightMap.at<ushort>(y, x) > 0)
                ++validPoints;

    // Write header
    ofs << "ply\nformat ascii 1.0\n";
    ofs << "comment MSI pointcloud\n";
    ofs << "element vertex " << validPoints << "\n";
    ofs << "property float x\nproperty float y\nproperty float z\n";
    ofs << "property float nx\nproperty float ny\nproperty float nz\n";
    ofs << "property uchar red\nproperty uchar green\nproperty uchar blue\n";
    ofs << "property int u\nproperty int v\n";
    ofs << "end_header\n";

    for (int y = 0; y < heightMap.rows; ++y) {
        for (int x = 0; x < heightMap.cols; ++x) {
            ushort d = heightMap.at<ushort>(y, x);
            if (d == 0) continue;

            // Coordinate
            float z = d * scale;
            float xx = static_cast<float>(x);
            float yy = static_cast<float>(y);

            // Dummy normal
            float nx = 0.0f, ny = 0.0f, nz = 0.0f;

            // Dummy color — grayscale based on 8-bit normalized map
            uchar colorVal = normalized.at<uchar>(y, x);
            uchar r = colorVal, g = colorVal, b = colorVal;

            // u, v = pixel index
            int u = x, v = y;

            ofs << xx << " " << yy << " " << z << " "
                << nx << " " << ny << " " << nz << " "
                << (int)r << " " << (int)g << " " << (int)b << " "
                << u << " " << v << "\n";
        }
    }

    ofs.close();
    qDebug("PLY saved with full format: %s", qUtf8Printable(plyPath));
}


cv::Mat computePeakCountMask(const std::vector<cv::Mat>& focusVolume,
    const QString& debugFolder,
    bool debugMode,
    int kernelSize,
    int peakThreshold,
    float peakProminence)
{
    if (focusVolume.empty()) return {};

    int depth = static_cast<int>(focusVolume.size());
    int rows = focusVolume[0].rows;
    int cols = focusVolume[0].cols;
    int radius = kernelSize / 2;

    cv::Mat mask(rows, cols, CV_8U, cv::Scalar(0));  // 255 = reliable
    cv::Mat peakCountMap(rows, cols, CV_32F, cv::Scalar(0)); // store raw peak count

    auto countLocalPeaks = [&](const std::vector<float>& curve, float minProminence) -> int {
        int peakCount = 0;
        int size = static_cast<int>(curve.size());
        for (int i = 1; i < size - 1; ++i) {
            float prev = curve[i - 1];
            float curr = curve[i];
            float next = curve[i + 1];

            if ((curr - prev) > minProminence && (next - curr) > minProminence) {
                peakCount++;
            }
        }
        return peakCount;
        };

    for (int y = radius; y < rows - radius; ++y) {
        for (int x = radius; x < cols - radius; ++x) {
            std::vector<float> curve;
            curve.reserve(depth);

            for (int z = 0; z < depth; ++z) {
                const cv::Mat& slice = focusVolume[z];
                cv::Rect roi(x - radius, y - radius, kernelSize, kernelSize);
                cv::Mat patch = slice(roi);
                cv::Scalar mean, stddev;
                cv::meanStdDev(patch, mean, stddev);
                curve.push_back(static_cast<float>(mean[0]));
            }

            int peakCount = countLocalPeaks(curve, peakProminence);
            peakCountMap.at<float>(y, x) = static_cast<float>(peakCount);

            if (peakCount > peakThreshold) {
                mask.at<uchar>(y, x) = 255;  // unreliable
            }
        }
    }

    // Optional debug output
    if (debugMode && !debugFolder.isEmpty()) {
        QDir().mkpath(debugFolder);

        // Save binary mask
        cv::imwrite((debugFolder + "/peak_mask.png").toStdString(), mask);

        // Normalize and save peak count map
        cv::Mat peakCountNormalized;
        cv::normalize(peakCountMap, peakCountNormalized, 0, 255, cv::NORM_MINMAX);
        peakCountNormalized.convertTo(peakCountNormalized, CV_8U);
        cv::imwrite((debugFolder + "/peak_count.png").toStdString(), peakCountNormalized);

        // Optional: Apply color map
        cv::Mat colorMap;
        cv::applyColorMap(peakCountNormalized, colorMap, cv::COLORMAP_JET);
        cv::imwrite((debugFolder + "/peak_count_colormap.png").toStdString(), colorMap);
    }

    return mask;
}


void applyGuidedFilter(const cv::Mat& guide, const cv::Mat& src, cv::Mat& dst,
    int radius, double eps) {
    CV_Assert(!guide.empty() && !src.empty());
    cv::Mat guideGray, srcFloat;

    // Convert to required format
    if (guide.channels() == 3)
        cv::cvtColor(guide, guideGray, cv::COLOR_BGR2GRAY);
    else
        guideGray = guide;

    guideGray.convertTo(guideGray, CV_32F, 1.0 / 255.0);
    src.convertTo(srcFloat, CV_32F, 1.0 / 255.0);

    // Apply guided filter
    cv::ximgproc::guidedFilter(guideGray, srcFloat, dst, radius, eps);

    // Convert result back to 8U
    dst.convertTo(dst, CV_8U, 255.0);
}

int findBestFocusedSlice(const std::vector<cv::Mat>& images) {
    int bestIndex = 0;
    double bestScore = -1.0;

    for (int i = 0; i < images.size(); ++i) {
        cv::Mat gray, floatImg, gradX, gradY, magnitude;

        // Convert to float
        images[i].convertTo(floatImg, CV_32F, 1.0 / 255.0);

        // Option 1: Tenengrad (Sobel)
        cv::Sobel(floatImg, gradX, CV_32F, 1, 0, 3);
        cv::Sobel(floatImg, gradY, CV_32F, 0, 1, 3);
        cv::magnitude(gradX, gradY, magnitude);

        double meanFocus = cv::mean(magnitude)[0];

        if (meanFocus > bestScore) {
            bestScore = meanFocus;
            bestIndex = i;
        }
    }

    return bestIndex;
}

cv::Mat erodeHeightMap8U(const cv::Mat& heightMap8U, int iterations, int kernelSize) {
    CV_Assert(heightMap8U.type() == CV_8U);  // Ensure it's 16-bit unsigned

    cv::Mat eroded = heightMap8U.clone();
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernelSize, kernelSize));

    for (int i = 0; i < iterations; ++i) {
        cv::erode(eroded, eroded, kernel);
    }

    return eroded;
}

void preprocessFocusSlices(std::vector<cv::Mat>& focusVolume,
    const QString& debugFolder,
    bool debugMode,
    int dilateIter,
    int erodeExtra) {
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    QDir().mkpath(debugFolder);

    for (size_t i = 0; i < focusVolume.size(); ++i) {
        cv::Mat& focusSlice = focusVolume[i];

        if (debugMode) {
            // Save original shape for reference
            cv::Mat vis;
            cv::normalize(focusSlice, vis, 0, 255, cv::NORM_MINMAX);
            vis.convertTo(vis, CV_8U);
            QString beforePath = debugFolder + QString("/focus_%1_before.png").arg(i, 3, 10, QChar('0'));
            //cv::imwrite(beforePath.toStdString(), vis);
        }

        for (int j = 0; j < dilateIter; ++j) {
            cv::dilate(focusSlice, focusSlice, kernel);
        }

        for (int j = 0; j < dilateIter + erodeExtra; ++j) {
            cv::erode(focusSlice, focusSlice, kernel);
        }

        if (debugMode) {
            // Save processed shape
            cv::Mat vis;
            cv::normalize(focusSlice, vis, 0, 255, cv::NORM_MINMAX);
            vis.convertTo(vis, CV_8U);
            QString afterPath = debugFolder + QString("/focus_%1_after.png").arg(i, 3, 10, QChar('0'));
            cv::imwrite(afterPath.toStdString(), vis);
        }
    }
}

void plotFocusCurveAtPatch(const std::vector<cv::Mat>& focusVolume,
    int x, int y,
    int kernelSize,
    const QString& debugFolder,
    const QString& title) {
    if (focusVolume.empty() || kernelSize % 2 == 0) return;

    int depth = static_cast<int>(focusVolume.size());
    int radius = kernelSize / 2;
    int rows = focusVolume[0].rows;
    int cols = focusVolume[0].cols;

    // Boundary check
    if (x < radius || y < radius || x >= cols - radius || y >= rows - radius) {
        qDebug() << "Point out of valid bounds for patch size!";
        return;
    }

    QDir().mkpath(debugFolder);
    std::vector<double> focusCurve;

    for (int z = 0; z < depth; ++z) {
        const cv::Mat& slice = focusVolume[z];
        cv::Rect roi(x - radius, y - radius, kernelSize, kernelSize);
        cv::Mat patch = slice(roi);
        cv::Scalar mean, stddev;
        cv::meanStdDev(patch, mean, stddev);
        focusCurve.push_back(static_cast<double>(mean[0]*100));  // or mean[0] for intensity average
    }

    // --- Plotting ---
    int width = 800, height = 400;
    cv::Mat plot(height, width, CV_8UC3, cv::Scalar(255, 255, 255));
    int margin = 50;
    int graphW = width - 2 * margin;
    int graphH = height - 2 * margin;

    double minVal = *std::min_element(focusCurve.begin(), focusCurve.end());
    double maxVal = *std::max_element(focusCurve.begin(), focusCurve.end());
    double range = maxVal - minVal + 1e-6;

    int n = static_cast<int>(focusCurve.size());
    for (int i = 1; i < n; ++i) {
        int x0 = margin + (i - 1) * graphW / (n - 1);
        int x1 = margin + i * graphW / (n - 1);
        int y0 = margin + static_cast<int>(graphH * (1.0 - (focusCurve[i - 1] - minVal) / range));
        int y1 = margin + static_cast<int>(graphH * (1.0 - (focusCurve[i] - minVal) / range));
        cv::line(plot, { x0, y0 }, { x1, y1 }, cv::Scalar(0, 0, 255), 2);
    }

    for (int i = 0; i < n; ++i) {
        int px = margin + i * graphW / (n - 1);
        int py = margin + static_cast<int>(graphH * (1.0 - (focusCurve[i] - minVal) / range));
        cv::line(plot, { px - 4, py - 4 }, { px + 4, py + 4 }, cv::Scalar(0, 0, 0), 1);
        cv::line(plot, { px - 4, py + 4 }, { px + 4, py - 4 }, cv::Scalar(0, 0, 0), 1);
        std::string labelText = std::to_string(i) + " (" + cv::format("%.2f", focusCurve[i]) + ")";
        cv::putText(plot, labelText, { px - 10, py - 8 }, cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(0, 0, 0), 1);
    }

    cv::line(plot, { margin, margin }, { margin, height - margin }, cv::Scalar(0, 0, 0), 1);
    cv::line(plot, { margin, height - margin }, { width - margin, height - margin }, cv::Scalar(0, 0, 0), 1);
    cv::putText(plot, "Z-Slice Index", { width / 2 - 50, height - 10 }, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
    cv::putText(plot, "Focus Value", { 5, margin - 10 }, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
    cv::putText(plot, title.toStdString(), { margin, margin - 25 }, cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 1);

    QString filename = debugFolder + QString("/patch_curve_%1_%2.png").arg(x).arg(y);
    cv::imwrite(filename.toStdString(), plot);
}

void plotAllPatchCurves(const std::vector<cv::Mat>& focusVolume, int kernelSize, const QString& debugFolder) {
    if (focusVolume.empty()) return;

    int rows = focusVolume[0].rows;
    int cols = focusVolume[0].cols;
    int step = kernelSize;

    for (int y = kernelSize; y < rows - kernelSize; y += step) {
        for (int x = kernelSize; x < cols - kernelSize; x += step) {
            plotFocusCurveAtPatch(focusVolume, x, y, kernelSize, debugFolder);
        }
    }
}

void computeLaplacianPyramidStack(const std::vector<cv::Mat>& inputImages,
    std::vector<cv::Mat>& focusMeasures,
    const QString& debugFolder,
    bool debugMode,
    int levels) {

    QDir().mkpath(debugFolder); // Auto-create folder if it doesn't exist

    for (size_t i = 0; i < inputImages.size(); ++i) {
        const cv::Mat& img = inputImages[i];
        cv::Mat gray, floatImg;

        if (img.channels() == 3)
            cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        else
            gray = img;

        gray.convertTo(floatImg, CV_32F, 1.0 / 255.0);

        std::vector<cv::Mat> gaussianPyr;
        gaussianPyr.push_back(floatImg.clone());

        for (int l = 0; l < levels; ++l) {
            cv::Mat down;
            cv::pyrDown(gaussianPyr.back(), down);
            gaussianPyr.push_back(down);
        }

        std::vector<cv::Mat> laplacianPyr;
        for (int l = 0; l < levels; ++l) {
            cv::Mat up;
            cv::pyrUp(gaussianPyr[l + 1], up, gaussianPyr[l].size());
            cv::Mat lap;
            cv::subtract(gaussianPyr[l], up, lap, cv::noArray(), CV_32F);
            laplacianPyr.push_back(cv::abs(lap));
        }

        // Aggregate laplacian layers up to original resolution
        cv::Mat focusMeasure = cv::Mat::zeros(floatImg.size(), CV_32F);
        for (int l = 0; l < levels; ++l) {
            cv::Mat up = laplacianPyr[l];
            for (int j = l; j < levels - 1; ++j) {
                cv::pyrUp(up, up);
            }
            if (up.size() != focusMeasure.size()) {
                cv::resize(up, up, focusMeasure.size());
            }
            focusMeasure += up;
        }

        focusMeasures.push_back(focusMeasure);

        if (debugMode && !debugFolder.isEmpty()) {
            cv::Mat debugOut;
            cv::normalize(focusMeasure, debugOut, 0, 255, cv::NORM_MINMAX);
            debugOut.convertTo(debugOut, CV_8U);

            QString fileName = QString("laplacian_pyr_%1.png").arg(i, 3, 10, QChar('0'));
            QString fullPath = debugFolder + "/" + fileName;
            cv::imwrite(fullPath.toStdString(), debugOut);
        }
    }
}


cv::Mat reconstructAndSaveFocusedImage(const std::vector<cv::Mat>& inputImages,
    const std::vector<cv::Mat>& focusMeasures,
    const QString& debugFolder)
{
    CV_Assert(!inputImages.empty() && inputImages.size() == focusMeasures.size());

    int rows = inputImages[0].rows;
    int cols = inputImages[0].cols;
    cv::Mat focused(rows, cols, CV_8U);  // output focused image

    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            float maxVal = -1.0f;
            uchar bestPixel = 0;

            for (size_t z = 0; z < focusMeasures.size(); ++z) {
                float val = focusMeasures[z].at<float>(y, x);
                if (val > maxVal) {
                    maxVal = val;
                    bestPixel = inputImages[z].at<uchar>(y, x);
                }
            }

            focused.at<uchar>(y, x) = bestPixel;
        }
    }

    // Save the image

    QDir().mkpath(QFileInfo(debugFolder).absolutePath());
    cv::imwrite((debugFolder + "/focusedImage.png").toStdString(), focused);
    return focused;
}

cv::Mat autoThresholdWireMask(const cv::Mat& dogMask, const QString& debugFolder) {
    // Normalize to 8-bit if needed (dogMask is in float)
    cv::Mat normDog;
    cv::normalize(dogMask, normDog, 0, 255, cv::NORM_MINMAX);
    normDog.convertTo(normDog, CV_8U);

    // Apply Otsu's thresholding
    cv::Mat wireMask;
    double threshVal = cv::threshold(normDog, wireMask, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    if (!debugFolder.isEmpty()) {
        QDir().mkpath(debugFolder);
        cv::imwrite((debugFolder + "/dog_normalized.png").toStdString(), normDog);
        cv::imwrite((debugFolder + "/wire_mask.png").toStdString(), wireMask);
        qDebug() << "Otsu threshold used:" << threshVal;
    }

    return wireMask;
}

cv::Mat smoothHeightMapDirectionally(const cv::Mat& heightMap,
    int kernelSize,
    int angleBins,
    const QString& debugFolder,
    bool debugMode)
{
    CV_Assert(heightMap.type() == CV_8U);

    if (debugMode) {
        QDir().mkpath(debugFolder);
        cv::imwrite((debugFolder + "/input_heightMap.png").toStdString(), heightMap);
    }

    cv::Mat floatMap;
    heightMap.convertTo(floatMap, CV_32F);

    // 1. Compute gradients
    cv::Mat gradX, gradY;
    cv::Sobel(floatMap, gradX, CV_32F, 1, 0, 3);
    cv::Sobel(floatMap, gradY, CV_32F, 0, 1, 3);

    if (debugMode) {
        cv::Mat gradMag;
        cv::magnitude(gradX, gradY, gradMag);
        cv::Mat gradMagVis;
        cv::normalize(gradMag, gradMagVis, 0, 255, cv::NORM_MINMAX);
        gradMagVis.convertTo(gradMagVis, CV_8U);
        cv::imwrite((debugFolder + "/gradient_magnitude.png").toStdString(), gradMagVis);
    }

    // 2. Get angle map (0 to 360°) and quantize
    cv::Mat angle;
    cv::phase(gradX, gradY, angle, true); // degrees
    angle /= (360.0 / angleBins);         // e.g., bin size 45° for 8 bins
    angle.convertTo(angle, CV_8U);        // quantized bins

    // 3. Create directional kernels
    std::vector<cv::Mat> kernels(angleBins);
    for (int i = 0; i < angleBins; ++i) {
        double theta = i * CV_PI / angleBins;
        int k = kernelSize / 2;
        cv::Mat kernel = cv::Mat::zeros(kernelSize, kernelSize, CV_32F);
        for (int y = -k; y <= k; ++y) {
            for (int x = -k; x <= k; ++x) {
                double proj = x * cos(theta) + y * sin(theta);
                if (std::abs(proj) < 1.0) {
                    kernel.at<float>(y + k, x + k) = 1.0f;
                }
            }
        }
        cv::normalize(kernel, kernel, 0, 1, cv::NORM_MINMAX);
        kernels[i] = kernel;

        if (debugMode) {
            cv::Mat vis;
            kernel.convertTo(vis, CV_8U, 255.0);
            QString fileName = QString("kernel_%1.png").arg(i);
            cv::imwrite((debugFolder + "/" + fileName).toStdString(), vis);
        }
    }

    // 4. Apply directional smoothing
    cv::Mat smoothed = cv::Mat::zeros(heightMap.size(), CV_32F);
    for (int i = 0; i < angleBins; ++i) {
        cv::Mat mask = (angle == i);
        cv::Mat blurred;
        cv::filter2D(floatMap, blurred, -1, kernels[i]);
        blurred.copyTo(smoothed, mask);
    }

    if (debugMode) {
        cv::Mat debugOut;
        cv::normalize(smoothed, debugOut, 0, 255, cv::NORM_MINMAX);
        debugOut.convertTo(debugOut, CV_8U);
        cv::imwrite((debugFolder + "/smoothed_result.png").toStdString(), debugOut);
    }

    cv::Mat result;
    smoothed.convertTo(result, CV_8U);
    return result;
}

// --- FFT Sharpness ---
double computeFFTSharpness(const cv::Mat& patch, double freqThreshold)
{
    cv::Mat floatPatch;
    patch.convertTo(floatPatch, CV_64F);

    // Hanning window
    cv::Mat win;
    cv::createHanningWindow(win, patch.size(), CV_64F);
    floatPatch = floatPatch.mul(win);

    // FFT
    cv::Mat dftComplex;
    cv::dft(floatPatch, dftComplex, cv::DFT_COMPLEX_OUTPUT);
    cv::Mat planes[2];
    cv::split(dftComplex, planes);
    cv::magnitude(planes[0], planes[1], planes[0]);
    cv::Mat magnitude = planes[0];

    int cx = magnitude.cols / 2, cy = magnitude.rows / 2;
    double total = 0.0, highFreq = 0.0;

    for (int y = 0; y < magnitude.rows; ++y) {
        for (int x = 0; x < magnitude.cols; ++x) {
            double dx = x - cx, dy = y - cy;
            double r = std::sqrt(dx * dx + dy * dy);
            double val = magnitude.at<double>(y, x);
            total += val;
            if (r > freqThreshold)
                highFreq += val;
        }
    }
    return (total > 0.0) ? (highFreq / total) : 0.0;
}

// --- Fit Plane from 3D Points ---
Eigen::Vector3d fitPlaneFromPoints(const std::vector<cv::Point3d>& points)
{
    Eigen::MatrixXd A(points.size(), 3);
    Eigen::VectorXd b(points.size());

    for (size_t i = 0; i < points.size(); ++i) {
        A(i, 0) = points[i].x;
        A(i, 1) = points[i].y;
        A(i, 2) = 1.0;
        b(i) = points[i].z;
    }

    return A.colPivHouseholderQr().solve(b); // [a, b, c]
}

void renderPlaneToMat16U(
    const Eigen::Vector3d& plane,
    cv::Size fullSize,
    cv::Mat& out16U,
    bool normalize,
    int numZSlices,
    const cv::Rect& roi)  // default = full image
{
    double a = plane[0], b = plane[1], c = plane[2];
    out16U = cv::Mat(fullSize, CV_16UC1, cv::Scalar(0));
    cv::Mat planeZ(fullSize, CV_64F, cv::Scalar(0));

    cv::Rect renderROI = roi.area() > 0 ? (roi & cv::Rect(0, 0, fullSize.width, fullSize.height))
        : cv::Rect(0, 0, fullSize.width, fullSize.height);

    double zMin = std::numeric_limits<double>::max();
    double zMax = std::numeric_limits<double>::lowest();

    for (int y = renderROI.y; y < renderROI.y + renderROI.height; ++y)
        for (int x = renderROI.x; x < renderROI.x + renderROI.width; ++x) {
            double z = a * x + b * y + c;
            planeZ.at<double>(y, x) = z;

            if (!normalize) {
                zMin = std::min(zMin, z);
                zMax = std::max(zMax, z);
            }
        }

    if (normalize && numZSlices > 0) {
        zMin = 0.0;
        zMax = (numZSlices - 1);  // custom normalization range
    }

    //qDebug() << "zMax:" << zMax << " zMin:" << zMin;

    for (int y = renderROI.y; y < renderROI.y + renderROI.height; ++y)
        for (int x = renderROI.x; x < renderROI.x + renderROI.width; ++x) {
            double val = planeZ.at<double>(y, x);
            ushort finalVal = 0;

          
            if (normalize && zMax > zMin)
            {
                double normVal = 65535.0 * (val - zMin) / (zMax - zMin + 1e-6);
                finalVal = static_cast<ushort>(std::clamp(normVal, 0.0, 65535.0));
            }
            else
                finalVal = static_cast<ushort>(std::clamp(val, 0.0, 65535.0));

            //qDebug() << "val:" << val << " finalVal:" << finalVal;
            out16U.at<ushort>(y, x) = finalVal;
        }
}

void normalizeZSlice8UTo16U(
    const cv::Mat& heightMap8U,  // input CV_8UC1 Z-slice index map
    double zMin,                 // real-world min Z (e.g., 0.0)
    double zMax,                 // real-world max Z (e.g., 9.0)
    cv::Mat& out16U              // output CV_16UC1 normalized map
)
{
    CV_Assert(heightMap8U.type() == CV_8UC1);

    out16U = cv::Mat::zeros(heightMap8U.size(), CV_16UC1);

    for (int y = 0; y < heightMap8U.rows; ++y) {
        for (int x = 0; x < heightMap8U.cols; ++x) {
            uchar zIndex = heightMap8U.at<uchar>(y, x);  // 0–255

            double zReal = static_cast<double>(zIndex) + 8;  // use as-is or map to real Z
            double normVal = 65535.0 * (zReal - zMin) / (zMax - zMin + 1e-6);
            ushort finalVal = static_cast<ushort>(std::clamp(normVal, 0.0, 65535.0));

            out16U.at<ushort>(y, x) = finalVal;
        }
    }
}


double computeStddevFocus(const cv::Mat& roi) {
    cv::Scalar mean, stddev;
    cv::meanStdDev(roi, mean, stddev);
    return stddev[0]; // focus score
}

// --- MAIN PACKAGED FUNCTION ---
cv::Mat generateFittedPlaneFromZStack(
    const std::vector<cv::Mat>& zStack,
    const SurfaceROI surfaceRoi,
    double freqThreshold,
    bool normalizeOutput)
{
    auto rects = surfaceRoi.planeRois;
    std::vector<int> bestIndices(rects.size(), -1);
    std::vector<double> bestScores(rects.size(), -1.0);

    // FFT-based best focus per region
    for (int z = 0; z < static_cast<int>(zStack.size()); ++z) {
        const cv::Mat& img = zStack[z];
        for (size_t i = 0; i < rects.size(); ++i) {
            cv::Rect roi = rects[i] & cv::Rect(0, 0, img.cols, img.rows);
            if (roi.area() == 0) continue;

            double score = computeStddevFocus(img(roi));
            if (score > bestScores[i]) {
                bestScores[i] = score;
                bestIndices[i] = z;  // using index directly as Z
            }
        }
    }

    // Build 3D points using Z = index
    std::vector<cv::Point3d> pts3D;
    for (size_t i = 0; i < rects.size(); ++i) {
        auto center = (rects[i].tl() + rects[i].br()) * 0.5;
        double z = static_cast<double>(bestIndices[i]);  // index as height
        pts3D.emplace_back(center.x, center.y, z);
        qDebug() << "centerX:" << center.x << " centerY:" << center.y << " z:" << z;
     }

    // Fit plane: z = ax + by + c
    Eigen::Vector3d plane = fitPlaneFromPoints(pts3D);

    // Render plane to 16-bit image
    cv::Mat outImage;
    renderPlaneToMat16U(plane, zStack[0].size(), outImage, normalizeOutput, zStack.size(), surfaceRoi.surfaceRoi);

    return outImage;
}

void analyzeFocusStackToGraphsPerROI(
    const std::vector<cv::Mat>& stack,
    const std::vector<cv::Rect>& rois,
    const QString& debugFolderPath)
{
    if (stack.empty() || rois.empty()) {
        qDebug() << "Image stack or ROIs are empty!";
        return;
    }

    QDir().mkpath(debugFolderPath);

    auto drawGraph = [](const std::vector<double>& values, const std::string& label,
        int width = 800, int height = 400) -> cv::Mat {
            cv::Mat plot(height, width, CV_8UC3, cv::Scalar(255, 255, 255));
            double minVal = *std::min_element(values.begin(), values.end());
            double maxVal = *std::max_element(values.begin(), values.end());
            double range = maxVal - minVal + 1e-6;
            int margin = 50;
            int graphW = width - 2 * margin;
            int graphH = height - 2 * margin;
            int n = static_cast<int>(values.size());

            for (int i = 1; i < n; ++i) {
                int x0 = margin + (i - 1) * graphW / (n - 1);
                int x1 = margin + i * graphW / (n - 1);
                int y0 = margin + static_cast<int>(graphH * (1.0 - (values[i - 1] - minVal) / range));
                int y1 = margin + static_cast<int>(graphH * (1.0 - (values[i] - minVal) / range));
                cv::line(plot, { x0, y0 }, { x1, y1 }, cv::Scalar(0, 0, 255), 2);
            }

            for (int i = 0; i < n; ++i) {
                int x = margin + i * graphW / (n - 1);
                int y = margin + static_cast<int>(graphH * (1.0 - (values[i] - minVal) / range));
                int crossSize = 4;
                cv::line(plot, { x - crossSize, y - crossSize }, { x + crossSize, y + crossSize }, cv::Scalar(0, 0, 0), 1);
                cv::line(plot, { x - crossSize, y + crossSize }, { x + crossSize, y - crossSize }, cv::Scalar(0, 0, 0), 1);
                std::string labelText = std::to_string(i) + " (" + cv::format("%.1f", values[i] * 100) + ")";
                cv::putText(plot, labelText, { x - 10, y - 8 }, cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(0, 0, 0), 1);
            }

            cv::line(plot, { margin, margin }, { margin, height - margin }, cv::Scalar(0, 0, 0), 1);
            cv::line(plot, { margin, height - margin }, { width - margin, height - margin }, cv::Scalar(0, 0, 0), 1);
            cv::putText(plot, "Z-Slice Index", { width / 2 - 50, height - 10 },
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
            cv::putText(plot, "Focus Value", { 5, margin - 10 },
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
            cv::putText(plot, label, { margin, margin - 25 },
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 1);

            return plot;
        };

    for (size_t roiIdx = 0; roiIdx < rois.size(); ++roiIdx) {
        std::vector<double> means, stddevs;

        for (const auto& img : stack) {
            cv::Rect roi = rois[roiIdx] & cv::Rect(0, 0, img.cols, img.rows);
            if (roi.area() == 0) {
                means.push_back(0);
                stddevs.push_back(0);
                continue;
            }

            cv::Mat roiPatch = img(roi);
            cv::Scalar mean, stddev;
            cv::meanStdDev(roiPatch, mean, stddev);
            means.push_back(mean[0]);
            stddevs.push_back(stddev[0]);
        }

        QString baseName = QString("roi%1").arg(roiIdx);
        cv::Mat meanPlot = drawGraph(means, "Mean Intensity - ROI " + std::to_string(roiIdx));
        cv::Mat stdPlot = drawGraph(stddevs, "StdDev Intensity - ROI " + std::to_string(roiIdx));

        cv::imwrite((debugFolderPath + "/" + baseName + "_mean_plot.png").toStdString(), meanPlot);
        cv::imwrite((debugFolderPath + "/" + baseName + "_std_plot.png").toStdString(), stdPlot);
    }

    qDebug() << "Per-ROI focus graphs saved to:" << debugFolderPath;
}

cv::Mat stackHeightMapsByMax(const std::vector<cv::Mat>& heightMaps)
{
    if (heightMaps.empty())
        return cv::Mat();

    cv::Size size = heightMaps[0].size();
    cv::Mat finalMap = cv::Mat::zeros(size, CV_16UC1);

    for (int y = 0; y < size.height; ++y) {
        for (int x = 0; x < size.width; ++x) {
            ushort maxVal = 0;
            for (const auto& map : heightMaps) {
                if (!map.empty() && map.size() == size && map.type() == CV_16UC1) {
                    maxVal = std::max(maxVal, map.at<ushort>(y, x));
                }
            }
            finalMap.at<ushort>(y, x) = maxVal;
        }
    }

    return finalMap;
}
