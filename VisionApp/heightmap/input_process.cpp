#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudaarithm.hpp>
#include "input_process.h"
#include "param.h"
#include "utils.h"

bool loadSingleInputImage(const InputParam& input, const size_t index, cv::cuda::GpuMat& dst) 
{
    // Ensure the file extension starts with a dot
    std::string extension = input.fileExtension;
    if (extension.front() != '.') {
        extension = "." + extension;
    }

    // Construct full file path
    std::string filePath = input.folderPath + '/' + std::to_string(index) + extension;

    // Check if file exists
    if (!fileExists(filePath)) {
        std::cerr << filePath << " does not exist" << std::endl;
        return false;
    }

    // Load image (optimized: load grayscale directly if needed)
    
    cv::Mat img8U;
    switch (input.channel) {
        case Channel::GRAY: {
            img8U = cv::imread(filePath, cv::IMREAD_GRAYSCALE);
            break;
        }
        case Channel::BLUE: {
            cv::Mat img8UC3 = cv::imread(filePath, cv::IMREAD_COLOR);
            cv::extractChannel(img8UC3, img8U, 0);
            break;
        }
        case Channel::GREEN: {
            cv::Mat img8UC3 = cv::imread(filePath, cv::IMREAD_COLOR);
            cv::extractChannel(img8UC3, img8U, 1);
            break;
        }
        case Channel::RED: {
            cv::Mat img8UC3 = cv::imread(filePath, cv::IMREAD_COLOR);
            cv::extractChannel(img8UC3, img8U, 2);
            break;
        }
        default: {
            std::cerr << "Unsupported channel type" << std::endl;
            return false;
        }
    }

    if (img8U.empty()) {
        std::cerr << "Error loading image: " << filePath << std::endl;
        return false;
    }

    cv::Mat img32F;
    img8U.convertTo(img32F, CV_32F);

    dst.upload(img32F);

    return true;
}

bool loadSingleInputImageFromMemory(const InputParam& input,
    const size_t index,
    cv::cuda::GpuMat& dst)
{
    // 0) Basic checks
    if (!input.inputImages) { std::cerr << "input.inputImages is null\n"; return false; }
    const auto& imgs = *input.inputImages;
    if (index >= imgs.size()) { std::cerr << "index " << index << " out of range (size=" << imgs.size() << ")\n"; return false; }

    const cv::Mat& src = imgs[index];
    if (src.empty()) { std::cerr << "inputImages[" << index << "] is empty\n"; return false; }

    if (input.size.area() > 0 && src.size() != input.size) {
        std::cerr << "Size mismatch: src=" << src.cols << "x" << src.rows
            << " expected=" << input.size.width << "x" << input.size.height << "\n";
        // return false; // or allow and proceed
    }

    const int sc = src.channels();
    const int sd = src.depth();

    // 1) Fast path: already 1ch float -> upload directly (no extra copies)
    if (sc == 1 && sd == CV_32F) {
        dst.upload(src);            // dst: 1ch 32F
        return true;
    }

    // 2) We’ll upload once, then convert on GPU.
    //    For best support, ensure the uploaded color image is either 8U or 32F:
    //    - cuda::cvtColor supports 8U and 32F well.
    cv::Mat cpuColor32f;            // only used if we need 32F color
    cv::cuda::GpuMat gColor;        // uploaded color (8U or 32F)
    cv::cuda::GpuMat gGray;         // gray (8U or 32F)
    cv::cuda::GpuMat gTmp;          // scratch

    auto upload_as_8u = [&]() {
        gColor.upload(src);         // src is already 8U
        };
    auto upload_as_32f = [&]() {
        if (sd == CV_32F) {
            gColor.upload(src);
        }
        else {
            // Convert on CPU to 32F once (for 16U/64F etc.) then upload
            src.convertTo(cpuColor32f, CV_32F);
            gColor.upload(cpuColor32f);
        }
        };

    // 3) Branch by requested channel
    if (input.channel == Channel::GRAY) {
        if (sc == 1) {
            // Upload scalar image (any depth), then convert to 32F on GPU
            cv::cuda::GpuMat g1;
            g1.upload(src);
            g1.convertTo(dst, CV_32F);   // all-GPU conversion
            return true;
        }
        else if (sc == 3 || sc == 4) {
            if (sd == CV_8U) upload_as_8u();
            else             upload_as_32f();  // cvtColor supports 32F too

            const int code = (sc == 3) ? cv::COLOR_BGR2GRAY : cv::COLOR_BGRA2GRAY;
            cv::cuda::cvtColor(gColor, gGray, code);

            // If gGray is 8U, convert to 32F; if already 32F, this is a no-op copy
            gGray.convertTo(dst, CV_32F);
            return true;
        }
        else {
            std::cerr << "Unsupported channel count for GRAY: " << sc << "\n";
            return false;
        }
    }
    else {
        // BLUE / GREEN / RED channel selection
        const int cidx = (input.channel == Channel::BLUE) ? 0
            : (input.channel == Channel::GREEN) ? 1 : 2;

        if (sc >= 3) {
            if (sd == CV_8U || sd == CV_32F) {
                // Upload in native 8U or 32F
                gColor.upload(src);
            }
            else {
                // e.g., 16U/64F -> convert to 32F on CPU once, then upload
                src.convertTo(cpuColor32f, CV_32F);
                gColor.upload(cpuColor32f);
            }

            // Extract requested channel on GPU
            // (either via split or mixChannels; split is simplest)
            std::vector<cv::cuda::GpuMat> chans;
            chans.resize(sc);
            cv::cuda::split(gColor, chans);    // allocs once per call
            cv::cuda::GpuMat gCh = chans[cidx];

            // Ensure 32F output
            gCh.convertTo(dst, CV_32F);
            return true;
        }
        else if (sc == 1) {
            // If caller asked for B/G/R but source is 1ch, just pass it through
            cv::cuda::GpuMat g1; g1.upload(src);
            g1.convertTo(dst, CV_32F);
            return true;
        }
        else {
            std::cerr << "Unsupported channel count: " << sc << "\n";
            return false;
        }
    }
}
