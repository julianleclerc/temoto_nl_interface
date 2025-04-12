#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include <queue>
#include <unordered_set>
#include <cmath>
#include <limits>

using json = nlohmann::json;

class PathfindingConverter {
private:
    // Custom hash function for pair<int, int>
    struct PairHash {
        template <class T1, class T2>
        std::size_t operator () (const std::pair<T1, T2>& p) const {
            auto h1 = std::hash<T1>{}(p.first);
            auto h2 = std::hash<T2>{}(p.second);
            return h1 ^ h2;
        }
    };

    // Node class for A* pathfinding
    class Node {
    public:
        std::pair<int, int> position;
        Node* parent;
        double g;  // Cost from start to current node
        double h;  // Estimated cost from current to goal
        double f;  // Total cost

        Node(std::pair<int, int> pos, Node* par = nullptr) 
            : position(pos), parent(par), g(0), h(0), f(0) {}

        // Custom comparator for priority queue
        struct Compare {
            bool operator()(const Node* a, const Node* b) const {
                return a->f > b->f;
            }
        };
    };

    // Heuristic function (Euclidean distance)
    static double heuristic(const std::pair<int, int>& a, const std::pair<int, int>& b) {
        int dx = a.first - b.first;
        int dy = a.second - b.second;
        return std::sqrt(dx*dx + dy*dy);
    }

    // Check if a position is walkable
    static bool isWalkable(const cv::Mat& object_map, const std::pair<int, int>& position) {
        int row = position.first;
        int col = position.second;

        // Check bounds
        if (row < 0 || row >= object_map.rows || 
            col < 0 || col >= object_map.cols) {
            return false;
        }

        // Check pixel value
        if (object_map.channels() == 3) {
            // Color image: check if all channels are 255
            cv::Vec3b pixel = object_map.at<cv::Vec3b>(row, col);
            return pixel[0] == 255 && pixel[1] == 255 && pixel[2] == 255;
        } else {
            // Grayscale image: check if pixel is 255
            return object_map.at<uchar>(row, col) == 255;
        }
    }

public:
    static std::string get(const cv::Mat& object_map, 
                           double resolution, 
                           const std::pair<double, double>& origin, 
                           const json& items_data, 
                           const json& robot_coords, 
                           const std::string& assistant_reply) {
        try {
            // Parse assistant reply
            json assistant_data = json::parse(assistant_reply);
            std::string target_id = assistant_data.value("target_id", "");

            // Find target item coordinates
            json item_coords;
            for (const auto& [item_class, items_list] : items_data["items"].items()) {
                for (const auto& item : items_list) {
                    if (item["id"] == target_id) {
                        item_coords = item["coordinates"];
                        break;
                    }
                }
                if (!item_coords.empty()) break;
            }

            if (item_coords.empty()) {
                assistant_data["success"] = "false";
                assistant_data["error"] = "Target ID not found in items_data";
                assistant_data["message"] = "The specified target could not be found.";
                return assistant_data.dump();
            }

            // Map dimensions
            int map_height = object_map.rows;
            int map_width = object_map.cols;

            // Extract origin coordinates
            double origin_x = origin.first;
            double origin_y = origin.second;

            // Convert world to map coordinates
            auto world_to_map = [&](double x, double y) {
                int col = static_cast<int>((x - origin_x) / resolution);
                int row = map_height - static_cast<int>((y - origin_y) / resolution) - 1;
                return std::make_pair(row, col);
            };

            // Convert robot and item positions
            double robot_x = robot_coords["x"];
            double robot_y = robot_coords["y"];
            auto robot_position = world_to_map(robot_x, robot_y);

            double item_x = item_coords["x"];
            double item_y = item_coords["y"];
            auto item_position = world_to_map(item_x, item_y);

            // Check map bounds
            auto is_within_bounds = [&](const std::pair<int, int>& pos) {
                return pos.first >= 0 && pos.first < map_height && 
                       pos.second >= 0 && pos.second < map_width;
            };

            if (!is_within_bounds(robot_position)) {
                assistant_data["success"] = "false";
                assistant_data["error"] = "Robot coordinates out of map bounds";
                assistant_data["message"] = "The robot's position is outside the map boundaries.";
                return assistant_data.dump();
            }

            if (!is_within_bounds(item_position)) {
                assistant_data["success"] = "false";
                assistant_data["error"] = "Item coordinates out of map bounds";
                assistant_data["message"] = "The item's position is outside the map boundaries.";
                return assistant_data.dump();
            }

            // A* Pathfinding
            std::priority_queue<Node*, std::vector<Node*>, Node::Compare> open_list;
            std::unordered_set<std::pair<int, int>, PairHash> closed_set;

            // Movements: up, down, left, right
            std::vector<std::pair<int, int>> movements = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

            // Starting node
            Node* start_node = new Node(robot_position);
            start_node->h = heuristic(robot_position, item_position);
            start_node->f = start_node->h;

            double min_distance = start_node->h;
            Node* best_node = start_node;

            open_list.push(start_node);

            while (!open_list.empty()) {
                Node* current_node = open_list.top();
                open_list.pop();

                auto current_position = current_node->position;

                // Skip if already processed
                if (closed_set.count(current_position)) continue;

                closed_set.insert(current_position);

                // Update best node if closer to the item
                double current_distance = heuristic(current_position, item_position);
                if (current_distance < min_distance) {
                    min_distance = current_distance;
                    best_node = current_node;
                }

                // If reached the item position, break
                if (current_position == item_position) {
                    best_node = current_node;
                    break;
                }

                // Explore neighbors
                for (const auto& move : movements) {
                    std::pair<int, int> neighbor_position = {
                        current_position.first + move.first, 
                        current_position.second + move.second
                    };

                    if (!isWalkable(object_map, neighbor_position) || 
                        closed_set.count(neighbor_position)) continue;

                    Node* neighbor_node = new Node(neighbor_position, current_node);
                    neighbor_node->g = current_node->g + 1;
                    neighbor_node->h = heuristic(neighbor_position, item_position);
                    neighbor_node->f = neighbor_node->g + neighbor_node->h;

                    open_list.push(neighbor_node);
                }
            }

            // Return map coordinates
            assistant_data["coordinates"] = {
                {"x", best_node->position.second},  // col index as x-coordinate
                {"y", best_node->position.first}    // row index as y-coordinate
            };

            assistant_data["success"] = "true";
            assistant_data["error"] = "none";

            // Clean up dynamically allocated nodes
            std::unordered_set<Node*> to_delete;
            Node* current = best_node;
            while (current) {
                to_delete.insert(current);
                current = current->parent;
            }
            for (auto node : to_delete) {
                delete node;
            }

            return assistant_data.dump();

        } catch (const std::exception& e) {
            json error_response = {
                {"success", "false"},
                {"error", "PathfindingError"},
                {"message", std::string("Error in pathfinding: ") + e.what()}
            };
            return error_response.dump();
        }
    }
};