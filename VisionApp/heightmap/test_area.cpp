#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <limits> // For numeric_limits

// --- Helper Function: Calculate Focus Measure for a single window ---
// Example using Sum of Modified Laplacian (SML)
double calculateSML(const cv::Mat& image_roi) {
    cv::Mat laplacian_x, laplacian_y;
    // SML Kernels
    cv::Mat kernel_x = (cv::Mat_<double>(1, 3) << -1, 2, -1);
    cv::Mat kernel_y = (cv::Mat_<double>(3, 1) << -1, 2, -1);

    // Apply kernels
    cv::filter2D(image_roi, laplacian_x, CV_64F, kernel_x, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
    cv::filter2D(image_roi, laplacian_y, CV_64F, kernel_y, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);

    // Calculate sum of absolute values
    cv::Mat abs_laplacian_x, abs_laplacian_y;
    cv::absdiff(laplacian_x, cv::Scalar(0), abs_laplacian_x);
    cv::absdiff(laplacian_y, cv::Scalar(0), abs_laplacian_y);

    return cv::sum(abs_laplacian_x)[0] + cv::sum(abs_laplacian_y)[0];
}

// Example using Variance
double calculateVariance(const cv::Mat& image_roi) {
    cv::Scalar mean, stddev;
    cv::meanStdDev(image_roi, mean, stddev);
    return stddev[0] * stddev[0]; // Variance is stddev squared
}


int main3() {
    // 1. --- Input ---
    //std::vector<std::string> image_paths = { "slice_0.png", "slice_1.png", /* ... */, "slice_N.png" };

    int N = 40; // Example value, you can change this
    double z_scale = 0.1; // Example z_scale, you can change this
    std::string input_folder = "E:/Work/Data/test20/";

    std::vector<std::string> image_paths;
    std::vector<double> z_positions;

    for (int i = 0; i <= N; ++i) {
        std::stringstream ss;
        ss << i << ".png";
        image_paths.push_back(ss.str());
        z_positions.push_back(i * z_scale);
    }

    std::cout << "Image paths:" << std::endl;
    for (const std::string& path : image_paths) {
        std::cout << path << std::endl;
    }

    std::cout << "\nZ positions:" << std::endl;
    for (double z : z_positions) {
        std::cout << z << std::endl;
    }

    //std::exit(1);

    if (image_paths.empty()) return -1;

    // Load first image to get dimensions
    cv::Mat first_image = cv::imread(input_folder + image_paths[0], cv::IMREAD_GRAYSCALE); // Work in grayscale
    if (first_image.empty()) {
        std::cerr << "Error loading first image." << std::endl;
        return -1;
    }
    int height = first_image.rows;
    int width = first_image.cols;
    int num_slices = image_paths.size();

    // --- Initialization ---
    cv::Mat height_map = cv::Mat::zeros(height, width, CV_64F); // Store Z values (double precision)
    cv::Mat max_focus_map = cv::Mat::zeros(height, width, CV_64F); // Store max focus values found so far
    max_focus_map.setTo(std::numeric_limits<double>::lowest()); // Initialize with very low values

    // All-in-focus image (optional but often useful)
    cv::Mat all_in_focus_image = cv::Mat::zeros(height, width, first_image.type());


    // --- Parameters ---
    int window_size = 9; // Example: Use a 9x9 window (must be odd)
    int half_window = window_size / 2;


    // 2. --- Process Slice by Slice ---
    for (int k = 0; k < num_slices; ++k) {
        std::cout << "Processing slice " << k << "/" << num_slices << std::endl;

        // Load current slice
        cv::Mat current_slice = cv::imread(input_folder + image_paths[k], cv::IMREAD_GRAYSCALE);
        if (current_slice.empty()) {
            std::cerr << "Warning: Could not load slice " << k << std::endl;
            continue;
        }
        if (current_slice.rows != height || current_slice.cols != width) {
            std::cerr << "Warning: Slice " << k << " has different dimensions." << std::endl;
            continue;
        }

        // Pad the image to handle borders when extracting windows
        cv::Mat padded_slice;
        cv::copyMakeBorder(current_slice, padded_slice, half_window, half_window, half_window, half_window, cv::BORDER_REPLICATE);


        // 3. --- Calculate Focus Measure for each pixel ---
        // This is the most compute-intensive part - consider parallelization (e.g., OpenMP)
#pragma omp parallel for // Optional: Parallelize the outer loop if using OpenMP
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {

                // Extract the window from the padded image
                cv::Rect roi_rect(x, y, window_size, window_size); // ROI top-left corner is (x,y) in the *padded* image
                cv::Mat window = padded_slice(roi_rect);

                // Calculate focus measure for this window
                 double current_focus_measure = calculateSML(window);
                //double current_focus_measure = calculateVariance(window); // Or use another FMO


                // 4. --- Update Max Focus and Height Map ---
                // This comparison needs to be thread-safe if parallelizing without proper reduction
                // Using direct write like this with #pragma omp parallel for on the *outer* loop
                // is generally safe because different threads handle different (y) rows.
                // If parallelizing inner loop or using more complex schemes, use critical sections or atomics.
                if (current_focus_measure > max_focus_map.at<double>(y, x)) {
                    max_focus_map.at<double>(y, x) = current_focus_measure;
                    height_map.at<double>(y, x) = z_positions[k];
                    // Optional: Update all-in-focus image
                    all_in_focus_image.at<uchar>(y, x) = current_slice.at<uchar>(y, x);
                }
            }
        }
    }

    // 5. --- Post-processing (Optional) ---
    // Convert height map to CV_32F because medianBlur doesn't support CV_64F
    cv::Mat height_map_32f;
    height_map.convertTo(height_map_32f, CV_32F);

    // Apply Median filtering to the 32-bit float map
    cv::Mat filtered_height_map_32f; // Output should also be CV_32F
    cv::medianBlur(height_map_32f, filtered_height_map_32f, 3); // Use a small kernel size (e.g., 3x3)

    // 6. --- Output ---
    // Normalize the *filtered* 32F height map for visualization
    cv::Mat display_height_map;
    cv::normalize(filtered_height_map_32f, display_height_map, 0, 255, cv::NORM_MINMAX, CV_8U);

    // Save the filtered 32F map as raw data (tiff supports float)
    // Or save the original unfiltered CV_64F map if you prefer
    // cv::imwrite("height_map_raw_unfiltered.tiff", height_map); // Save original double
    cv::imwrite("height_map_filtered_raw.tiff", filtered_height_map_32f); // Save filtered float

    // Save the visualized map (8-bit)
    cv::imwrite("height_map_filtered_visual.png", display_height_map);
    cv::imwrite("all_in_focus.png", all_in_focus_image);

    cv::imshow("Height Map", display_height_map);
    cv::imshow("All-in-focus", all_in_focus_image);
    cv::waitKey(0);

    return 0;
}