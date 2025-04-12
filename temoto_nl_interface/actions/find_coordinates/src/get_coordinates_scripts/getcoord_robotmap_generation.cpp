#include <opencv2/opencv.hpp>
#include <cmath>

struct Quaternion {
    double x, y, z, w;
};

struct Transform {
    struct {
        double x, y, z;
    } translation;
    Quaternion rotation;
};

class RobotMapGenerator {
public:
    static cv::Mat get(const cv::Mat& cost_map, 
                       double resolution, 
                       const std::pair<double, double>& origin, 
                       const Transform& trans) {
        // Ensure color image
        cv::Mat robot_map_color;
        if (cost_map.channels() == 1) {
            cv::cvtColor(cost_map, robot_map_color, cv::COLOR_GRAY2BGR);
        } else if (cost_map.channels() == 3) {
            robot_map_color = cost_map.clone();
        } else {
            throw std::runtime_error("Unexpected image shape for cost_map");
        }

        int map_height = robot_map_color.rows;
        int map_width = robot_map_color.cols;

        // Extract position
        double robot_x = trans.translation.x;
        double robot_y = trans.translation.y;

        // Compute yaw angle from quaternion
        Quaternion q = trans.rotation;
        double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
        double robot_yaw = std::atan2(siny_cosp, cosy_cosp);

        // Convert robot's position to map pixel coordinates
        double origin_x = origin.first;
        double origin_y = origin.second;
        
        int map_x = static_cast<int>((robot_x - origin_x) / resolution);
        int map_y = map_height - static_cast<int>((robot_y - origin_y) / resolution);

        // Drawing parameters
        int circle_radius = 5;
        cv::Scalar circle_color(0, 0, 255);        // Red in BGR
        cv::Scalar border_color(255, 0, 0);        // Blue in BGR
        cv::Scalar line_color(0, 255, 0);          // Green in BGR
        int circle_thickness = -1;                 // Filled circle
        int line_length = 20;
        int line_thickness = 2;
        int border_thickness = 1;

        // Draw red circle with blue border
        cv::circle(robot_map_color, 
                   cv::Point(map_x, map_y), 
                   circle_radius + border_thickness, 
                   border_color, 
                   circle_thickness);
        cv::circle(robot_map_color, 
                   cv::Point(map_x, map_y), 
                   circle_radius, 
                   circle_color, 
                   circle_thickness);

        // Compute orientation line endpoint
        int end_x = static_cast<int>(map_x + line_length * std::cos(robot_yaw));
        int end_y = static_cast<int>(map_y - line_length * std::sin(robot_yaw));

        // Draw orientation line with border
        cv::line(robot_map_color, 
                 cv::Point(map_x, map_y), 
                 cv::Point(end_x, end_y), 
                 border_color, 
                 line_thickness + border_thickness);
        cv::line(robot_map_color, 
                 cv::Point(map_x, map_y), 
                 cv::Point(end_x, end_y), 
                 line_color, 
                 line_thickness);

        return robot_map_color;
    }
};