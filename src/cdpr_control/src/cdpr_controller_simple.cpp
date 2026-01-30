#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <cmath>
#include <vector>
#include <array>

using namespace std::chrono_literals;

class CDPRController : public rclcpp::Node
{
public:
    CDPRController() : Node("cdpr_controller")
    {
        // Platform parameters
        platform_mass_ = 10.0;
        
        // Initialize base attachment points (8 cables)
        a_ = {{
            {-5.0, -5.0, 10.0}, {-5.0, 5.0, 10.0},
            {5.0, -5.0, 10.0}, {5.0, 5.0, 10.0},
            {-5.0, -5.0, 10.0}, {-5.0, 5.0, 10.0},
            {5.0, -5.0, 10.0}, {5.0, 5.0, 10.0}
        }};
        
        // Platform attachment points
        b_ = {{
            {-0.5, -0.5, 0.0}, {-0.5, 0.5, 0.0},
            {0.5, -0.5, 0.0}, {0.5, 0.5, 0.0},
            {-0.3, -0.3, 0.0}, {-0.3, 0.3, 0.0},
            {0.3, -0.3, 0.0}, {0.3, 0.3, 0.0}
        }};
        
        // Publishers
        cable_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/cable_markers", 10);
        platform_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "/platform/pose", 10);
        joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states", 10);
        
        // Timer for control loop (50Hz)
        timer_ = create_wall_timer(20ms, std::bind(&CDPRController::controlLoop, this));
        
        RCLCPP_INFO(get_logger(), "CDPR Controller initialized - Simple Version");
    }

private:
    struct Vec3 {
        double x, y, z;
        
        Vec3 operator+(const Vec3& other) const {
            return {x + other.x, y + other.y, z + other.z};
        }
        
        Vec3 operator-(const Vec3& other) const {
            return {x - other.x, y - other.y, z - other.z};
        }
        
        Vec3 operator*(double scalar) const {
            return {x * scalar, y * scalar, z * scalar};
        }
        
        double length() const {
            return std::sqrt(x*x + y*y + z*z);
        }
        
        Vec3 normalized() const {
            double len = length();
            if (len > 1e-6) {
                return {x/len, y/len, z/len};
            }
            return {0, 0, 0};
        }
        
        double dot(const Vec3& other) const {
            return x*other.x + y*other.y + z*other.z;
        }
        
        Vec3 cross(const Vec3& other) const {
            return {
                y*other.z - z*other.y,
                z*other.x - x*other.z,
                x*other.y - y*other.x
            };
        }
    };
    
    void controlLoop()
    {
        // 1. Generate circular trajectory
        double t = now().seconds();
        double radius = 2.0;
        double omega = 0.5;
        
        Vec3 desired_position = {
            radius * std::cos(omega * t),
            radius * std::sin(omega * t),
            5.0 + 0.5 * std::sin(omega * t * 0.5)
        };
        
        // 2. Compute cable lengths and directions
        std::vector<double> cable_lengths(8);
        std::vector<Vec3> cable_directions(8);
        
        for (int i = 0; i < 8; i++) {
            Vec3 platform_point = desired_position + b_[i];
            Vec3 cable_vec = platform_point - a_[i];
            cable_lengths[i] = cable_vec.length();
            cable_directions[i] = cable_vec.normalized();
        }
        
        // 3. Publish visualization data
        publishCableMarkers(cable_lengths, cable_directions);
        publishPlatformPose(desired_position);
        publishJointStates(cable_lengths);
        
        // 4. Log for debugging
        static int counter = 0;
        if (counter++ % 50 == 0) {
            RCLCPP_INFO(get_logger(), "Platform: [%.2f, %.2f, %.2f]", 
                       desired_position.x, desired_position.y, desired_position.z);
        }
    }
    
    void publishCableMarkers(const std::vector<double>& lengths,
                            const std::vector<Vec3>& directions)
    {
        auto marker_array = visualization_msgs::msg::MarkerArray();
        
        for (int i = 0; i < 8; i++) {
            auto marker = visualization_msgs::msg::Marker();
            marker.header.frame_id = "world";
            marker.header.stamp = now();
            marker.ns = "cables";
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            marker.action = visualization_msgs::msg::Marker::ADD;
            
            // Position at midpoint
            Vec3 midpoint = a_[i] + directions[i] * (lengths[i] / 2.0);
            marker.pose.position.x = midpoint.x;
            marker.pose.position.y = midpoint.y;
            marker.pose.position.z = midpoint.z;
            
            // Orientation (align with cable direction)
            Vec3 z_axis = {0, 0, 1};
            Vec3 axis = z_axis.cross(directions[i]);
            double angle = std::acos(z_axis.dot(directions[i]));
            
            if (axis.length() > 1e-6) {
                axis = axis.normalized();
                marker.pose.orientation.x = axis.x * std::sin(angle/2);
                marker.pose.orientation.y = axis.y * std::sin(angle/2);
                marker.pose.orientation.z = axis.z * std::sin(angle/2);
                marker.pose.orientation.w = std::cos(angle/2);
            } else {
                marker.pose.orientation.w = 1.0;
            }
            
            marker.scale.x = 0.05;  // diameter
            marker.scale.y = 0.05;  // diameter
            marker.scale.z = lengths[i];  // length
            
            // Color based on cable number
            marker.color.r = 0.8f;
            marker.color.g = 0.2f + 0.1f * i;
            marker.color.b = 0.2f;
            marker.color.a = 0.8f;
            
            marker_array.markers.push_back(marker);
        }
        
        cable_markers_pub_->publish(marker_array);
    }
    
    void publishPlatformPose(const Vec3& position)
    {
        auto pose_msg = geometry_msgs::msg::PoseStamped();
        pose_msg.header.frame_id = "world";
        pose_msg.header.stamp = now();
        pose_msg.pose.position.x = position.x;
        pose_msg.pose.position.y = position.y;
        pose_msg.pose.position.z = position.z;
        pose_msg.pose.orientation.w = 1.0;
        
        platform_pose_pub_->publish(pose_msg);
    }
    
    void publishJointStates(const std::vector<double>& lengths)
    {
        auto joint_msg = sensor_msgs::msg::JointState();
        joint_msg.header.stamp = now();
        
        for (int i = 0; i < 8; i++) {
            joint_msg.name.push_back("cable_" + std::to_string(i+1));
            joint_msg.position.push_back(lengths[i]);
            joint_msg.velocity.push_back(0.1);
            joint_msg.effort.push_back(50.0 + 20.0 * std::sin(now().seconds() + i));
        }
        
        joint_state_pub_->publish(joint_msg);
    }
    
    // Member variables
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cable_markers_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr platform_pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    
    std::array<Vec3, 8> a_;  // Base points
    std::array<Vec3, 8> b_;  // Platform points
    double platform_mass_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CDPRController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
