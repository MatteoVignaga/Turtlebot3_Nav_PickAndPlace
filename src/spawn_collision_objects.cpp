#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>

const double tau = 2 * M_PI;

int main(int argc, char **argv)
{
    // ROS2 Initialization
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("pap_node");

    // Logger
    auto logger = rclcpp::get_logger("pap_node");

    // Spinner with more thread for avoiding blocks
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner_thread([&executor]() { executor.spin(); });

    // Wait initialization
    rclcpp::sleep_for(std::chrono::seconds(2));

    // MoveIt2 interface
    using moveit::planning_interface::MoveGroupInterface;
    MoveGroupInterface arm(node, "arm");
    arm.setPoseReferenceFrame("base_link");
    
    // Collision objects
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.resize(3);

    collision_objects[0].id = "table1";
    collision_objects[0].header.frame_id = "base_link";
    collision_objects[0].primitives.resize(3);
    collision_objects[0].primitive_poses.resize(3);
    collision_objects[0].primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    collision_objects[0].primitives[0].dimensions = {0.1, 0.3, 0.02};
    collision_objects[0].primitive_poses[0].position.x = 0.05;
    collision_objects[0].primitive_poses[0].position.y = 0.0;
    collision_objects[0].primitive_poses[0].position.z = 0.15;
    collision_objects[0].primitives[1].type = shape_msgs::msg::SolidPrimitive::BOX;
    collision_objects[0].primitives[1].dimensions = {0.1, 0.02, 0.16};
    collision_objects[0].primitive_poses[1].position.x = 0.05;
    collision_objects[0].primitive_poses[1].position.y = -0.15;
    collision_objects[0].primitive_poses[1].position.z = 0.08;
    collision_objects[0].primitives[2].type = shape_msgs::msg::SolidPrimitive::BOX;
    collision_objects[0].primitives[2].dimensions = {0.1, 0.02, 0.16};
    collision_objects[0].primitive_poses[2].position.x = 0.05;
    collision_objects[0].primitive_poses[2].position.y = 0.15;
    collision_objects[0].primitive_poses[2].position.z = 0.08;
    collision_objects[0].pose.position.y = 0.205;
    collision_objects[0].pose.orientation.z = 0.6;
    collision_objects[0].operation = moveit_msgs::msg::CollisionObject::ADD;

    collision_objects[1].id = "table2";
    collision_objects[1].header.frame_id = "base_link";
    collision_objects[1].primitives.resize(3);
    collision_objects[1].primitive_poses.resize(3);
    collision_objects[1].primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    collision_objects[1].primitives[0].dimensions = {0.1, 0.3, 0.02};
    collision_objects[1].primitive_poses[0].position.x = 0.05;
    collision_objects[1].primitive_poses[0].position.y = 0.0;
    collision_objects[1].primitive_poses[0].position.z = 0.15;
    collision_objects[1].primitives[1].type = shape_msgs::msg::SolidPrimitive::BOX;
    collision_objects[1].primitives[1].dimensions = {0.1, 0.02, 0.16};
    collision_objects[1].primitive_poses[1].position.x = 0.05;
    collision_objects[1].primitive_poses[1].position.y = -0.15;
    collision_objects[1].primitive_poses[1].position.z = 0.08;
    collision_objects[1].primitives[2].type = shape_msgs::msg::SolidPrimitive::BOX;
    collision_objects[1].primitives[2].dimensions = {0.1, 0.02, 0.16};
    collision_objects[1].primitive_poses[2].position.x = 0.05;
    collision_objects[1].primitive_poses[2].position.y = 0.15;
    collision_objects[1].primitive_poses[2].position.z = 0.08;
    collision_objects[1].pose.position.y = -0.205;
    collision_objects[1].pose.orientation.z = -0.6;
    collision_objects[1].operation = moveit_msgs::msg::CollisionObject::ADD;

    collision_objects[2].id = "pickable_object";
    collision_objects[2].header.frame_id = "base_link";
    collision_objects[2].primitives.resize(1);
    collision_objects[2].primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    collision_objects[2].primitives[0].dimensions = {0.08, 0.005};
    collision_objects[2].pose.position.x = 0.1;
    collision_objects[2].pose.position.y = 0.2;
    collision_objects[2].pose.position.z = 0.04 + 0.16;
    collision_objects[2].operation = moveit_msgs::msg::CollisionObject::ADD;

    // Add objects to the scene
    planning_scene_interface.applyCollisionObjects(collision_objects);
    RCLCPP_INFO(logger, "Collision objects added to the planning scene.");

    // stop the spinner
    rclcpp::shutdown();
    spinner_thread.join();
    return 0;
}