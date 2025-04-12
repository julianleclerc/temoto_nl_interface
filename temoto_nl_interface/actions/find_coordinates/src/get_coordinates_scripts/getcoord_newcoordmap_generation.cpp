#include <opencv2/opencv.hpp>
#include <vector>

cv::Mat get2(const cv::Mat& cost_map, double resolution, int grid_scale) {
    // Calculate pixels per grid cell
    int pixels_per_grid_cell = grid_scale;

    // Map dimensions
    int height = cost_map.rows;
    int width = cost_map.cols;

    // Create a copy of the map to draw the grid on
    cv::Mat grid_map = cost_map.clone();

    // Ensure cost_map is in grayscale
    cv::Mat cost_map_gray;
    if (cost_map.channels() == 3) {
        cv::cvtColor(cost_map, cost_map_gray, cv::COLOR_BGR2GRAY);
    } else {
        cost_map_gray = cost_map.clone();
    }

    // Threshold to binary
    cv::Mat binary_map;
    cv::threshold(cost_map_gray, binary_map, 254, 255, cv::THRESH_BINARY);

    // Define grid line color (Red in BGR)
    cv::Vec3b grid_color(0, 0, 255);

    // Generate x positions for vertical grid lines
    std::vector<int> x_positions;
    for (int x = 0; x < width; x += pixels_per_grid_cell) {
        x_positions.push_back(x);
    }

    // Draw vertical grid lines
    for (int x : x_positions) {
        for (int y = 0; y < height; y++) {
            grid_map.at<cv::Vec3b>(y, x) = grid_color;
        }
    }

    // Generate y positions for horizontal grid lines
    std::vector<int> y_positions;
    for (int y = 0; y < height; y += pixels_per_grid_cell) {
        y_positions.push_back(y);
    }

    // Draw horizontal grid lines
    for (int y : y_positions) {
        for (int x = 0; x < width; x++) {
            grid_map.at<cv::Vec3b>(y, x) = grid_color;
        }
    }

    return grid_map;
}

cv::Mat get(const cv::Mat& cost_map, double resolution, int grid_scale) {
    // Calculate pixels per grid cell
    int pixels_per_grid_cell = grid_scale;

    // Map dimensions
    int height = cost_map.rows;
    int width = cost_map.cols;

    // Create a copy of the map to draw the grid on
    cv::Mat grid_map = cost_map.clone();

    // Ensure cost_map is in grayscale
    cv::Mat cost_map_gray;
    if (cost_map.channels() == 3) {
        cv::cvtColor(cost_map, cost_map_gray, cv::COLOR_BGR2GRAY);
    } else {
        cost_map_gray = cost_map.clone();
    }

    // Threshold to binary
    cv::Mat binary_map;
    cv::threshold(cost_map_gray, binary_map, 254, 255, cv::THRESH_BINARY);

    // Define grid line color (Red in BGR)
    cv::Vec3b grid_color(0, 0, 255);

    // Generate x positions for vertical grid lines
    std::vector<int> x_positions;
    for (int x = 0; x < width; x += pixels_per_grid_cell) {
        x_positions.push_back(x);
    }

    // Draw vertical grid lines
    for (int x : x_positions) {
        for (int y = 0; y < height; y++) {
            if (binary_map.at<uchar>(y, x) == 255) {
                grid_map.at<cv::Vec3b>(y, x) = grid_color;
            }
        }
    }

    // Generate y positions for horizontal grid lines
    std::vector<int> y_positions;
    for (int y = 0; y < height; y += pixels_per_grid_cell) {
        y_positions.push_back(y);
    }

    // Draw horizontal grid lines
    for (int y : y_positions) {
        for (int x = 0; x < width; x++) {
            if (binary_map.at<uchar>(y, x) == 255) {
                grid_map.at<cv::Vec3b>(y, x) = grid_color;
            }
        }
    }

    return grid_map;
}