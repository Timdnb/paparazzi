#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <iostream>
#include <filesystem>

namespace fs = std::filesystem;

int length(cv::Mat &image, cv::Vec4i line) {
    int count = 0;
    int x1 = line[0], y1 = line[1], x2 = line[2], y2 = line[3];
    float dy = (y2 - y1) / (x2 - x1);
    // Iterate over the line segment and count white pixels
    for (int x = x1; x < x2; x++) {
        int y = y1 + dy * (x - x1);
        
        // Ensure coordinates are within image bounds
        if (x >= 0 && x < image.cols && y >= 0 && y < image.rows) {
            // Check if the pixel is white (assuming white is 255 in grayscale)
            if (image.at<uchar>(y, x) == 255) {
                count++;
            }
        }
    }
    
    return count;
}

void remove_mess(cv::Mat &image, int window_size = 15, double thresh = 0.35) {
    int pixels_threshold = window_size * window_size * thresh;

    for (int y_start = 0; y_start < image.rows - window_size + 1; y_start += window_size) {
        for (int x_start = 0; x_start < image.cols - window_size + 1; x_start += window_size) {
            int white_pixels = 0;

            for (int y = y_start; y < y_start + window_size; y++) {
                for (int x = x_start; x < x_start + window_size; x++) {
                    // Check if the pixel is white (assuming 255 represents white)
                    if (image.at<uchar>(y, x) == 255) {
                        white_pixels += 3;
                    }
                }
            }

            // If there are pixels_threshold or more white pixels, replace the entire ROI with black pixels
            if (white_pixels >= pixels_threshold) {
                for (int y = y_start; y < y_start + window_size; y++) {
                    for (int x = x_start; x < x_start + window_size; x++) {
                        image.at<uchar>(y, x) = 0;
                    }
                }
            }
        }
    }
}

int main(int argc, char** argv) {
    // Read the image file

    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <directory_path>" << std::endl;
        return -1;
    }

    std::string directory = argv[1];

    // Read filenames in the directory
    std::vector<std::string> image_files;
    cv::glob(directory, image_files);
    std::sort(image_files.begin(), image_files.end()); // Sort filenames alphabetically

    if (image_files.empty()) {
        std::cerr << "No image files found in the specified directory." << std::endl;
        return -1;
    }

    int current_index = 150;
    char key = 0;

    while (key != 'q') {
        cv::Mat frame = cv::imread(image_files[current_index]);if (frame.empty()) {
            std::cerr << "Error: Unable to load input image." << std::endl;
            return -1;
        }

        // Rotate the image
        cv::transpose(frame, frame);
        cv::flip(frame, frame, 0);

        // Convert to grayscale
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // Apply Gaussian blur
        cv::GaussianBlur(gray, gray, cv::Size(15, 15), 0.9);

        // Detect edges using Canny edge detector
        cv::Mat edges;
        cv::Canny(gray, edges, 15, 20);

        // First guess
        int y_min = 120;
        // Consider only edges from the region where y > 150
        cv::Mat edges_roi = edges(cv::Rect(0, y_min, edges.cols, edges.rows - y_min));

        // Remove unwanted elements
        remove_mess(edges_roi);

        // Detect lines using Hough transform
        std::vector<cv::Vec4i> lines;
        cv::HoughLinesP(edges_roi, lines, 1, 5 * CV_PI / 180, 10, 100, 30);

        // Convert edges to colored image (for displaying)
        cv::cvtColor(edges, edges, cv::COLOR_GRAY2BGR);

        // Draw detected lines on the colored image
        for (size_t i = 0; i < lines.size(); i++) {
            cv::Vec4i line = lines[i];
            float angle = atan2(line[3] - line[1], line[2] - line[0]) * 180 / CV_PI;
            if (std::abs(angle) < 45) {
                cv::line(edges, cv::Point(line[0], line[1] + y_min), cv::Point(line[2], line[3] + y_min), cv::Scalar(0, 0, 255), 2);
            }
        }

        // Display the resulting image
        cv::imshow("Result", edges);
        key = cv::waitKey(0);

        if (key == 81 || key == 'a') { // Left arrow or 'a' key
            current_index = (current_index - 1 + image_files.size()) % image_files.size();
        } else if (key == 83 || key == 'd') { // Right arrow or 'd' key
            current_index = (current_index + 1) % image_files.size();
        }
    }

    cv::destroyAllWindows();
        return 0;
}
