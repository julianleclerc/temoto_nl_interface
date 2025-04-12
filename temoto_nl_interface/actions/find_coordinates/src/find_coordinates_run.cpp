#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <unordered_map>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>

// Include custom script headers
#include "getcoord_scalemap_generation.h"
#include "getcoord_costmap_generation.h"
#include "getcoord_nonTraversable_generation.h"
#include "getcoord_grid_generation.h"
#include "getcoord_objectmap_generation.h"
#include "getcoord_pixelcoord_return.h"
#include "getcoord_newcoordmap_generation.h"
#include "getcoord_origincoord_return.h"

using json = nlohmann::json;

class CoordinateFinder {
private:
    // Configuration parameters
    float inflation_radius_m = 0.2;
    int scale_factor = 2;
    int grid_scale = 20; // in px
    float resolution = 0.05;
    std::vector<float> origin = {0.0, 0.0, 0.0};

    // Data storage
    json items_data;
    json map_data;

    // Internal helper methods
    json loadJsonFile(const std::string& filepath) {
        std::ifstream file(filepath);
        if (!file.is_open()) {
            throw std::runtime_error("Could not open file: " + filepath);
        }
        return json::parse(file);
    }

    std::string encodeImageToBase64(const cv::Mat& image) {
        std::vector<uchar> buffer;
        cv::imencode(".png", image, buffer);
        
        std::string base64_string;
        cv::Mat base64_mat(buffer);
        base64_mat.convertTo(base64_mat, CV_8UC1);
        
        // Base64 encoding (simplified, replace with actual base64 encoding)
        return std::string(base64_mat.begin<char>(), base64_mat.end<char>());
    }

public:
    CoordinateFinder(const std::string& items_json_path) {
        // Load items data from JSON
        items_data = loadJsonFile(items_json_path);
    }

    json findCoordinates(const std::string& map_path, const std::string& target_object, const std::string& object_description) {
        try {
            // Load map
            cv::Mat map_img = cv::imread(map_path, cv::IMREAD_GRAYSCALE);
            if (map_img.empty()) {
                throw std::runtime_error("Failed to load map image");
            }

            // Scale map
            cv::Mat scaled_img;
            float scaled_resolution;
            std::tie(scaled_img, scaled_resolution) = GetCoordScaleMapGeneration::process(map_img, resolution, scale_factor);

            // Generate cost map
            cv::Mat cost_map = GetCoordCostmapGeneration::process(scaled_img, scaled_resolution, inflation_radius_m);

            // Darken non-traversable areas (simplified without robot transform)
            cv::Mat non_traversable_map = GetCoordNonTraversableGeneration::process(cost_map, scaled_resolution, origin);

            // Create grid
            cv::Mat grid_map = GetCoordGridGeneration::process(non_traversable_map, scaled_resolution, grid_scale);

            // Populate map with objects
            cv::Mat object_map = GetCoordObjectMapGeneration::process(grid_map, scaled_resolution, origin, items_data);

            // Encode map to base64 (for AI processing simulation)
            std::string encoded_map = encodeImageToBase64(object_map);

            // Simulate AI coordinate finding (you'd replace this with actual AI integration)
            json result = simulateAICoordinateSearch(target_object, object_description, encoded_map);

            // Generate new coordinates map
            cv::Mat new_coords_map;
            float angle_deg;
            std::tie(new_coords_map, angle_deg) = GetCoordNewCoordmapGeneration::process(
                object_map, scaled_resolution, origin, result, items_data
            );

            // Get origin coordinates
            json origin_coords = GetCoordOriginCoordReturn::process(
                result, scaled_resolution, origin, 
                {object_map.rows, object_map.cols}, angle_deg
            );

            return origin_coords;

        } catch (const std::exception& e) {
            return {
                {"success", "false"},
                {"error", e.what()},
                {"message", "Failed to process coordinates"}
            };
        }
    }

private:
    // Simulate AI coordinate search (replace with actual AI integration)
    json simulateAICoordinateSearch(const std::string& target_object, 
                                    const std::string& object_description, 
                                    const std::string& encoded_map) {
        // This is a placeholder for actual AI coordinate search
        // In a real implementation, you'd use an AI service or custom logic
        return {
            {"success", "true"},
            {"target_id", target_object},
            {"coordinates", {{"x", 200}, {"y", 200}}},
            {"message", "Simulated coordinate search result"}
        };
    }
};

// Example usage
int main() {
    try {
        // Initialize coordinate finder with items JSON path
        CoordinateFinder finder("/path/to/items.json");

        // Find coordinates for a specific object
        json result = finder.findCoordinates(
            "/path/to/map.pgm",  // Map image path
            "table",             // Target object class
            "wooden table"       // Object description
        );

        // Print result
        std::cout << "Coordinate Search Result:\n" 
                  << result.dump(4) << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}