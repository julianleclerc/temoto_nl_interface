#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>

using json = nlohmann::json;

class CoordinateConverter {
public:
    static std::string get(const std::string& assistant_reply, 
                            double resolution, 
                            const std::pair<double, double>& origin, 
                            const cv::Size& map_shape, 
                            double angle) {
        try {
            // Parse the assistant reply
            json assistant_reply_dict = json::parse(assistant_reply);

            // Extract coordinates
            json& coords = assistant_reply_dict["coordinates"];
            
            // Check if coordinates exist
            if (!coords.contains("x") || !coords.contains("y")) {
                throw std::runtime_error("Invalid coordinates in assistant reply");
            }

            // Extract image coordinates
            double x_img = coords["x"];
            double y_img = coords["y"];

            // Get origin coordinates
            double origin_x = origin.first;
            double origin_y = origin.second;

            // Map dimensions
            int map_height = map_shape.height;

            // Convert image coordinates to world coordinates
            double x_world = x_img * resolution + origin_x;
            double y_world = (map_height - y_img) * resolution + origin_y;

            // Update coordinates in the dictionary
            coords["x"] = x_world;
            coords["y"] = y_world;

            // Add angle to the response
            assistant_reply_dict["angle"] = angle;

            // Return the updated JSON string
            return assistant_reply_dict.dump();

        } catch (const std::exception& e) {
            // Create error response
            json error_response = {
                {"success", "false"},
                {"error", "ConversionError"},
                {"message", std::string("Error converting coordinates: ") + e.what()}
            };

            return error_response.dump();
        }
    }
};