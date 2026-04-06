#include "rclcpp/rclcpp.hpp"
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/buffer.hpp"
#include <tb3_nav_pick_and_place/srv/get_marker_pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_trajectory.hpp>

using namespace ros2_aruco_interfaces::msg;
using namespace tb3_nav_pick_and_place::srv;
using moveit::planning_interface::MoveGroupInterface;

class ArucoMarkerPoseReader : public rclcpp::Node {
public:
    ArucoMarkerPoseReader() : Node("ArUco_marker_pose_reader"), 
                            tf2_buffer_(this->get_clock()), 
                            tf2_listener_(tf2_buffer_) {
        aruco_markers_subscription_ = this->create_subscription<ArucoMarkers>(
            "/aruco_markers", 
            10, 
            std::bind(&ArucoMarkerPoseReader::markers_callback, this, std::placeholders::_1));
        
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/marker_pose",
            10
        );

        // TODO: move to lambda
        pose_server_ = this->create_service<GetMarkerPose>(
            "GetMarkerPose", 
            std::bind(&ArucoMarkerPoseReader::send_pose, this, std::placeholders::_1, std::placeholders::_2)
        );
    }

    void init() {
        arm_ = std::make_unique<MoveGroupInterface>(shared_from_this(), "arm");
    }

private:
    void send_pose(const std::shared_ptr<GetMarkerPose::Request> request, std::shared_ptr<GetMarkerPose::Response> response) {
        RCLCPP_INFO(this->get_logger(), "Marker position request received!");
        response->marker_pose = marker_pose_;
        
        // sent the pose for the pick, now this node has to stop publishing the planning scene
        aruco_markers_subscription_ = nullptr;
        rclcpp::shutdown();
    }

    void markers_callback(const ArucoMarkers::SharedPtr msg) {
        // TODO: extend read and srv to multiple marker ids
        int target_id = 1;
        int index = -1;

        for (int i = 0; i < msg->marker_ids.size(); i++) {
            if ((msg->marker_ids)[i] == target_id) {
                index = i;
                break;
            }
        }

        if (index == -1) {
            RCLCPP_INFO(this->get_logger(), "No marker found with marker id %i", target_id);
            return;
        }

        geometry_msgs::msg::Pose marker_pose = msg->poses[index];
        // follow ROS 2 coordinate system
        // TODO: correct aruco ros configuration file
        geometry_msgs::msg::PoseStamped marker_pose_stamped;
        marker_pose_stamped.header.stamp = this->now();
        marker_pose_stamped.header.frame_id = "camera_link"; // topic /aruco_markers has incorrect headers
        marker_pose_stamped.pose.position.x = marker_pose.position.z;
        marker_pose_stamped.pose.position.y = marker_pose.position.x * -1;
        marker_pose_stamped.pose.position.z = marker_pose.position.y * -1;
        // pose_publisher_->publish(marker_pose_stamped);

        if(tf2_buffer_.canTransform("base_footprint", "camera_link", tf2::TimePointZero)) {
            geometry_msgs::msg::PoseStamped marker_pose_footprint = tf2_buffer_.transform<geometry_msgs::msg::PoseStamped>(marker_pose_stamped, "base_footprint");
            marker_pose_ = marker_pose_footprint;
            pose_publisher_->publish(marker_pose_footprint);
            
            // Spawn object based on marker position
            std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
            collision_objects.resize(1);

            collision_objects[0].id = "pickable_object";
            collision_objects[0].header.frame_id = "base_footprint";
            collision_objects[0].primitives.resize(2);
            collision_objects[0].primitive_poses.resize(2);
            collision_objects[0].primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
            collision_objects[0].primitives[0].dimensions = {0.3, 0.005};
            collision_objects[0].primitive_poses[0].position.x = marker_pose_.pose.position.x;
            collision_objects[0].primitive_poses[0].position.y = marker_pose_.pose.position.y;
            collision_objects[0].primitive_poses[0].position.z = marker_pose_.pose.position.z - 0.25;
            collision_objects[0].primitives[1].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
            collision_objects[0].primitives[1].dimensions = {0.01, 0.05};
            collision_objects[0].primitive_poses[1].position.x = marker_pose_.pose.position.x;
            collision_objects[0].primitive_poses[1].position.y = marker_pose_.pose.position.y;
            collision_objects[0].primitive_poses[1].position.z = marker_pose_.pose.position.z - 0.40;
            collision_objects[0].operation = moveit_msgs::msg::CollisionObject::ADD;

            planning_scene_interface_.applyCollisionObjects(collision_objects);
            // RCLCPP_INFO(this->get_logger(), "Collision objects added to the planning scene.");
        }
    }

    rclcpp::Subscription<ArucoMarkers>::SharedPtr aruco_markers_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::Service<GetMarkerPose>::SharedPtr pose_server_;
    geometry_msgs::msg::PoseStamped marker_pose_;
    tf2_ros::Buffer tf2_buffer_;
    tf2_ros::TransformListener tf2_listener_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    std::unique_ptr<MoveGroupInterface> arm_;

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArucoMarkerPoseReader>();
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
}