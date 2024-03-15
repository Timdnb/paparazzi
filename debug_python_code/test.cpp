#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <iostream>
#include <filesystem>

/*
COMPILE:
g++ test.cpp -o test -I/usr/include/opencv4 -lopencv_stitching -lopencv_alphamat
 -lopencv_aruco -lopencv_barcode -lopencv_bgsegm -lopencv_bioinspired -lopencv_c
calib -lopencv_dnn_objdetect -lopencv_dnn_superres -lopencv_dpm -lopencv_face -l
opencv_freetype -lopencv_fuzzy -lopencv_hdf -lopencv_hfs -lopencv_img_hash -lope
ncv_intensity_transform -lopencv_line_descriptor -lopencv_mcc -lopencv_quality -
lopencv_rapid -lopencv_reg -lopencv_rgbd -lopencv_saliency -lopencv_shape -lopen
cv_stereo -lopencv_structured_light -lopencv_phase_unwrapping -lopencv_superres 
-lopencv_optflow -lopencv_surface_matching -lopencv_tracking -lopencv_highgui -l
opencv_datasets -lopencv_text -lopencv_plot -lopencv_ml -lopencv_videostab -lope
ncv_videoio -lopencv_viz -lopencv_wechat_qrcode -lopencv_ximgproc -lopencv_video
 -lopencv_xobjdetect -lopencv_objdetect -lopencv_calib3d -lopencv_imgcodecs -lop
encv_features2d -lopencv_dnn -lopencv_flann -lopencv_xphoto -lopencv_photo -lope
ncv_imgproc -lopencv_core

RUN:
./test ./cyberzoo_poles_panels_mats/20190121-142935/
*/

namespace fs = std::filesystem;

int length(cv::Mat &image, cv::Vec4i line) {
    //printf("Image shape: (%d, %d)\n", image.rows, image.cols);
    int MARGIN = 2;
    int count = 0;
    int x1 = line[0], y1 = line[1], x2 = line[2], y2 = line[3];
    float dydx = static_cast<float>(y2 - y1) / static_cast<float>(x2 - x1);
    //printf("dy: %i, dx: %i -> dy/dx: %f\n", y2 - y1, x2 - x1, dydx);
    // Iterate over the line segment and count white pixels
    for (int x = x1; x <= x2; x++) {
        int y = y1 + dydx * (x - x1);
        
        // Ensure coordinates are within image bounds
        for (int ty = y - MARGIN; ty <= y + MARGIN; ty++) {
            if (x >= 0 && x < image.cols && ty >= 0 && ty < image.rows) {
                //printf("x: %u, y: %u -> pixel: %hhu\n", x, ty, image.at<uchar>(ty, x) == 255);
                // Check if the pixel is white (assuming white is 255 in grayscale)
                if (image.at<uchar>(ty, x) == 255) {
                    count++;
                    break;
                }
            }
        }
    }
    
    return count;
}

std::array<int, 2> findIntersection(cv::Vec4i line1, cv::Vec4i line2) {
    float x1 = line1[0], y1 = line1[1], x2 = line1[2], y2 = line1[3];
    float x3 = line2[0], y3 = line2[1], x4 = line2[2], y4 = line2[3];

    float denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    float x = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denom;
    float y = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denom;

    return {static_cast<int>(x), static_cast<int>(y)};
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

cv::Vec4f performLinearRegressionWithGuess(cv::Mat binaryImage, cv::Vec4i model, int xRange[2]) {
    const int ACCEPT_REGION = 10;

    std::vector<std::vector<int>> points;

    int x0 = model[0];
    int y0 = model[1];
    int x1 = model[2];
    int y1 = model[3];

    float m = static_cast<float>(y1 - y0) / static_cast<float>(x1 - x0);
    float b = y0 - m * x0;

    printf("x: %u - %u\n", xRange[0], xRange[1]);
    for (int x = xRange[0]; x <= xRange[1]; x++) {
        int distance = ACCEPT_REGION;
        int bestY = -1;
        int prediction = static_cast<int>(m * x + b);

        for (int y = std::max(0, prediction - ACCEPT_REGION); y <= std::min(binaryImage.rows - 1, prediction + ACCEPT_REGION); y++) {
            if(binaryImage.at<uchar>(y, x) == 255) {
                int newDistance = abs(y - prediction);
                if (newDistance < distance) {
                    bestY = y;
                    distance = newDistance;
                }
            }
        }

        if (bestY != -1) {
            //for (int j = -2; j <= 2; j++) {
                //binaryImage.at<uchar>(bestY + j, x) = 255;
            //}
            //printf("new_point: %u, %u\n", x, bestY);
            points.push_back({x, bestY});
        }
    }

    if (points.size() < 2) {
        std::cout << "No white pixels found in the image." << std::endl;
        return model;
    }

    // Convert vector of vectors to 2D array for fitting
    cv::Mat pointsMat(points.size(), 2, CV_32S);
    for (size_t i = 0; i < points.size(); i++) {
        pointsMat.at<int>(i, 0) = points[i][0]; // x coordinate
        pointsMat.at<int>(i, 1) = points[i][1]; // y coordinate
        //for (int j = -2; j <= 2; j++) {
        //    binaryImage.at<uchar>(points[i][1] + j, points[i][0]) = 255;
        //}
    }

    // Perform linear regression
    cv::Vec4f newModel; // Updated model
    cv::fitLine(pointsMat, newModel, cv::DIST_L2, 0, 0.01, 0.01);

    // Convert the model from (vx, vy, x0, y0) to (x0, y0, x1, y1)
    float vx = newModel[0];
    float vy = newModel[1];
    float x_ref = newModel[2];
    float y_ref = newModel[3];

    x0 = xRange[0];
    x1 = xRange[1];
    y0 = static_cast<int> (y_ref + (x0 - x_ref) * (vy / vx));
    y1 = static_cast<int> (y_ref + (x1 - x_ref) * (vy / vx));
    
    newModel[0] = x0;
    newModel[1] = y0;
    newModel[2] = x1;
    newModel[3] = y1;

    return newModel;
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

    int current_index = 239;
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
        cv::HoughLinesP(edges_roi, lines, 3, 5 * CV_PI / 180, 10, 100, 30);

        // Convert edges to colored image (for displaying)
        cv::Mat colored(edges.size(), CV_8UC3);
        cv::cvtColor(edges, colored, cv::COLOR_GRAY2BGR);

        // Draw detected lines on the colored image
        printf("\n--- %u ---\n", current_index);
        cv::Vec4i final_lines[2] = {cv::Vec4i(0, 0, 0, 0), cv::Vec4i(0, 0, 0, 0)};
        int lengths[2] = {0, 0};
        for (size_t i = 0; i < lines.size(); i++) {
            cv::Vec4i line = lines[i];
            line[1] += y_min;
            line[3] += y_min;
            float angle = atan2(line[3] - line[1], line[2] - line[0]) * 180 / CV_PI;
            if (std::abs(angle) < 45) {
                int len = length(edges, line);
                int idx = static_cast<int> (angle > 0);
                if (len > lengths[idx]) {
                    lengths[idx] = len;
                    final_lines[idx] = line;
                }
            }
        }
        
        for (size_t i = 0; i < 2; i++) {
            if (lengths[i] > 0) {
                //printf("i: %lu -> %u \n", i, lengths[i]);
                //cv::line(colored, cv::Point(final_lines[i][0], final_lines[i][1]), cv::Point(final_lines[i][2], final_lines[i][3]), cv::Scalar(0, 0, 255), 1);
            }
        }

        std::array<int, 2> center = {0, 0};

        // skip if both are 0
        if (lengths[0] || lengths[1]) {
            if (lengths[0] == 0) {
                center[0] = final_lines[1][2];
                center[1] = final_lines[1][3];
            }
            else if (lengths[1] == 0) {
                center[0] = final_lines[0][2];
                center[1] = final_lines[0][3];
            }
            else {
                center = findIntersection(final_lines[0], final_lines[1]);
                // In case of error just take the middle point at random
                if (center[0] >= edges.cols or center[1] >= edges.rows) {
                    center[0] = static_cast<float> (final_lines[0][2] + final_lines[1][0]) / 2;
                    center[1] = static_cast<float> (final_lines[0][3] + final_lines[1][1]) / 2;
                }
            }

            for(int i = 0; i <= 1; i++) {
                if(lengths[i] > 0) {
                    int range_x[2] = {center[0] * i, center[0] * static_cast<int> (! static_cast<bool>(i)) + edges.cols * i};
                    final_lines[i] = performLinearRegressionWithGuess(edges, final_lines[i], range_x);
                    cv::cvtColor(edges, colored, cv::COLOR_GRAY2BGR);
                    //cv::line(colored, cv::Point(final_lines[i][0], final_lines[i][1]), cv::Point(final_lines[i][2], final_lines[i][3]), cv::Scalar(0, 255, 0), 1);
                    int x1 = final_lines[i][0], y1 = final_lines[i][1], x2 = final_lines[i][2], y2 = final_lines[i][3];
                    float dydx = static_cast<float>(y2 - y1) / static_cast<float>(x2 - x1);
                    for (int x = x1; x <= x2; x += 5) {
                        int y = y1 + dydx * (x - x1);
                        bool valid = false;
                        for (int j = -2; j <= 2; j++) {
                            int newY = y + j;
                            colored.at<cv::Vec3b>(newY, x) == cv::Vec3b(255, 255, 255);
                            if (newY >= 0 && newY < edges.rows && edges.at<uchar>(newY, x) == 255) {
                                valid = true;
                            }
                        }
                        if(!valid && y >= 0 && y < edges.rows) {
                            //printf("%u, %u \n", x, y);
                            //colored.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 255);
                            cv::Point center(x, y);
                            cv::circle(colored, center, 5, cv::Scalar(0, 0, 255), -1);
                            //std::cout << x << y << colored.at<cv::Vec3b>(y - 7, x) << std::endl;
                        }
                    }
                }
            }
        }

        // Display the resulting image
        cv::imshow("Result", colored);
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
