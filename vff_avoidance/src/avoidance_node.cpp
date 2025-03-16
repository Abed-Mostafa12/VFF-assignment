#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <vector>
#include <cmath>

class AvoidanceNode : public rclcpp::Node {
public:
    AvoidanceNode() : Node("avoidance_node") {

        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&AvoidanceNode::scan_callback, this, std::placeholders::_1));


        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);


        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/vff_debug", 10);


        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), std::bind(&AvoidanceNode::control_loop, this));
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        latest_scan_ = msg;
    }

    void control_loop() {
    if (!latest_scan_) return; 

    double attractive_x = 1.0;
    double attractive_y = 0.0;

    double repulsive_x = 0.0;
    double repulsive_y = 0.0;
    
    double min_distance = std::numeric_limits<double>::max();
    double min_angle = 0.0;


    for (size_t i = 0; i < latest_scan_->ranges.size(); ++i) {
        double distance = latest_scan_->ranges[i];

        if (distance < min_distance && distance > latest_scan_->range_min) {
            min_distance = distance;
            min_angle = latest_scan_->angle_min + i * latest_scan_->angle_increment;
        }
    }


    const double repulsion_gain = 0.8;
    if (min_distance < 1.5) {
        double strength = repulsion_gain / min_distance;
        repulsive_x = -strength * cos(min_angle);
        repulsive_y = -strength * sin(min_angle);
    }


    double result_x = attractive_x + repulsive_x;
    double result_y = attractive_y + repulsive_y;


    double result_angle = atan2(result_y, result_x);
    double speed = std::min(0.5, sqrt(result_x * result_x + result_y * result_y));


    const double max_angular_speed = 1.5;
    double angular_z = std::max(-max_angular_speed, std::min(max_angular_speed, result_angle));


    auto cmd_vel = geometry_msgs::msg::Twist();
    cmd_vel.linear.x = speed;
    cmd_vel.angular.z = angular_z;
    cmd_vel_pub_->publish(cmd_vel);


    auto marker_array = visualization_msgs::msg::MarkerArray();
    auto create_marker = [this](int id, double x, double y, float r, float g, float b) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "vff_vectors";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;

        geometry_msgs::msg::Point start, end;
        start.x = 0.0;
        start.y = 0.0;
        start.z = 0.0;
        end.x = x;
        end.y = y;
        end.z = 0.0;

        marker.points.push_back(start);
        marker.points.push_back(end);
        marker.scale.x = 0.02;
        marker.scale.y = 0.04;
        marker.scale.z = 0.04;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1.0;
        return marker;
    };

    marker_array.markers.push_back(create_marker(0, attractive_x, attractive_y, 0.0, 0.0, 1.0));
    marker_array.markers.push_back(create_marker(1, repulsive_x, repulsive_y, 1.0, 0.0, 0.0)); 
    marker_array.markers.push_back(create_marker(2, result_x, result_y, 0.0, 1.0, 0.0));     

    if (marker_pub_->get_subscription_count() > 0) {
        marker_pub_->publish(marker_array);
    }
}


    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AvoidanceNode>());
    rclcpp::shutdown();
    return 0;
}
