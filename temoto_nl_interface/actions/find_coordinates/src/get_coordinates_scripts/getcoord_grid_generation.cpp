#include <opencv2/opencv.hpp>
#include <vector>

cv::Mat get2(const cv::Mat& cost_map, int resolution, int grid_scale) {
    int pixels_per_grid_cell = grid_scale;
    int height = cost_map.rows;
    int width = cost_map.cols;

    cv::Mat grid_map = cost_map.clone();

    cv::Mat cost_map_gray;
    if (cost_map.channels() == 3) {
        cv::cvtColor(cost_map, cost_map_gray, cv::COLOR_BGR2GRAY);
    } else {
        cost_map_gray = cost_map.clone();
    }

    cv::Mat binary_map;
    cv::threshold(cost_map_gray, binary_map, 254, 255, cv::THRESH_BINARY);

    cv::Scalar grid_color(0, 0, 255); 

    for (int x = 0; x < width; x += pixels_per_grid_cell) {
        for (int y = 0; y < height; ++y) {
            grid_map.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 255);
        }
    }

    for (int y = 0; y < height; y += pixels_per_grid_cell) {
        for (int x = 0; x < width; ++x) {
            grid_map.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 255);
        }
    }

    return grid_map;
}

cv::Mat get(const cv::Mat& cost_map, int resolution, int grid_scale) {
    int pixels_per_grid_cell = grid_scale;
    int height = cost_map.rows;
    int width = cost_map.cols;

    cv::Mat grid_map = cost_map.clone();

    cv::Mat cost_map_gray;
    if (cost_map.channels() == 3) {
        cv::cvtColor(cost_map, cost_map_gray, cv::COLOR_BGR2GRAY);
    } else {
        cost_map_gray = cost_map.clone();
    }

    cv::Mat binary_map;
    cv::threshold(cost_map_gray, binary_map, 254, 255, cv::THRESH_BINARY);

    for (int x = 0; x < width; x += pixels_per_grid_cell) {
        for (int y = 0; y < height; ++y) {
            if (binary_map.at<uchar>(y, x) == 255) {
                grid_map.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 255);
            }
        }
    }

    for (int y = 0; y < height; y += pixels_per_grid_cell) {
        for (int x = 0; x < width; ++x) {
            if (binary_map.at<uchar>(y, x) == 255) {
                grid_map.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 255);
            }
        }
    }

    return grid_map;
}
