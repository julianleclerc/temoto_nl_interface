#include <opencv2/opencv.hpp>
#include <cmath>

cv::Mat get(const cv::Mat& map_img, double resolution, 
            const std::pair<double, double>& origin, 
            const struct RobotPosition& robot_position) {
    // Create a mutable copy of the map image
    cv::Mat modified_map = map_img.clone();

    // Extract robot's position in world coordinates
    double robot_x = robot_position.transform.translation.x;
    double robot_y = robot_position.transform.translation.y;

    // Map size in pixels
    int height = map_img.rows;
    int width = map_img.cols;

    // Convert robot's position to pixel coordinates
    double origin_x = origin.first;
    double origin_y = origin.second;
    
    int pixel_x = static_cast<int>((robot_x - origin_x) / resolution);
    int pixel_y = height - static_cast<int>((robot_y - origin_y) / resolution) - 1;

    // Ensure pixel_x and pixel_y are within image bounds
    pixel_x = std::max(0, std::min(pixel_x, width - 1));
    pixel_y = std::max(0, std::min(pixel_y, height - 1));

    // Create free space mask
    cv::Mat free_space_mask = cv::Mat::zeros(height, width, CV_8UC1);

    // Identify free space pixels (254 or 255 in all channels)
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            cv::Vec3b pixel = map_img.at<cv::Vec3b>(y, x);
            if ((pixel[0] == 254 && pixel[1] == 254 && pixel[2] == 254) ||
                (pixel[0] == 255 && pixel[1] == 255 && pixel[2] == 255)) {
                free_space_mask.at<uchar>(y, x) = 1;
            }
        }
    }

    // Prepare image for flood fill
    cv::Mat flood_fill_img = free_space_mask.clone();

    // Perform flood fill starting from the robot's pixel position
    cv::floodFill(flood_fill_img, cv::Mat(), cv::Point(pixel_x, pixel_y), 2);

    // Create a mask of unreachable free spaces
    cv::Mat unreachable_mask = cv::Mat::zeros(height, width, CV_8UC1);
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (free_space_mask.at<uchar>(y, x) == 1 && 
                flood_fill_img.at<uchar>(y, x) != 2) {
                unreachable_mask.at<uchar>(y, x) = 1;
            }
        }
    }

    // Set unreachable free space areas to black
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (unreachable_mask.at<uchar>(y, x) == 1) {
                modified_map.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0);
            }
        }
    }

    return modified_map;
}

// Struct to represent robot position (you may need to adjust based on your actual implementation)
struct RobotPosition {
    struct {
        struct {
            double x;
            double y;
        } translation;
    } transform;
};