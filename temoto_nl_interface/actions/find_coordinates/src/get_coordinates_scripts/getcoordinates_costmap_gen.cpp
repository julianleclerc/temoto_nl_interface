#include <opencv2/opencv.hpp>
#include <cmath>

cv::Mat get(const cv::Mat& map_img, double resolution, double inflation_radius_m) {
    // Convert inflation radius from meters to pixels
    int inflation_radius_px = static_cast<int>(std::ceil(inflation_radius_m / resolution));

    // Create masks for obstacles and free space
    cv::Mat obstacles, free_space;
    cv::compare(map_img, 0, obstacles, cv::CMP_EQ);
    cv::compare(map_img, 254, free_space, cv::CMP_EQ);

    // Create a mask for distance transform: obstacles as 0, free space as 1
    cv::Mat distance_mask;
    map_img.convertTo(distance_mask, CV_8U);
    distance_mask = (distance_mask != 0);

    // Compute the distance transform
    cv::Mat dist_transform;
    cv::distanceTransform(distance_mask, dist_transform, cv::DIST_L2, 5);

    // Convert distances from pixels to meters
    cv::Mat dist_transform_m;
    dist_transform.convertTo(dist_transform_m, CV_64F, resolution);

    // Initialize the cost map to 255 (white represents free space)
    cv::Mat cost_map = cv::Mat::ones(map_img.size(), CV_8U) * 255;

    // Create mask for inflation zone
    cv::Mat inflation_zone;
    cv::inRange(dist_transform_m, 0, inflation_radius_m, inflation_zone);
    inflation_zone = inflation_zone & ~obstacles;

    // Assign gray (70) to the inflation zone
    cost_map.setTo(70, inflation_zone);

    // Assign black (0) to obstacle cells
    cost_map.setTo(0, obstacles);

    return cost_map;
}