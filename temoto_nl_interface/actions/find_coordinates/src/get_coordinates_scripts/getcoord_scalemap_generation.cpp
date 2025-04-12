#include <opencv2/opencv.hpp>

std::pair<cv::Mat, double> get(const cv::Mat& map_img, double resolution, double scale_factor) {
    int original_width = map_img.cols;
    int original_height = map_img.rows;

    int new_width = std::max(1, static_cast<int>(original_width * scale_factor));
    int new_height = std::max(1, static_cast<int>(original_height * scale_factor));

    int interpolation;
    if (scale_factor > 1) {
        interpolation = cv::INTER_NEAREST;
    } else if (scale_factor < 1) {
        interpolation = cv::INTER_AREA;
    } else {
        return {map_img.clone(), resolution};
    }

    cv::Mat scaled_img;
    cv::resize(map_img, scaled_img, cv::Size(new_width, new_height), 0, 0, interpolation);

    double new_resolution = resolution / scale_factor;

    int max_dim = std::max(new_width, new_height);

    cv::Mat square_img;
    if (scaled_img.channels() == 3) {
        square_img = cv::Mat::zeros(max_dim, max_dim, CV_8UC3);
    } else {
        square_img = cv::Mat::zeros(max_dim, max_dim, CV_8UC1);
    }

    scaled_img.copyTo(square_img(cv::Rect(0, 0, new_width, new_height)));

    return {square_img, new_resolution};
}
