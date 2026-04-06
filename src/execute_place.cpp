#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <linkattacher_msgs/srv/detach_link.hpp>

/**
 * Performs a moveit motion planning operation
 * synchronously, so the scenario may be executed
 * through multiple sequential operations.
 * The returned value is a boolean that defines
 * the success of the operation.
 */
void perform_sync_movement(rclcpp::Logger logger, std::string operation_name, moveit::planning_interface::MoveGroupInterface& move_group) {
    
    // Planning
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto plan_success = move_group.plan(plan);
    // RCLCPP_INFO(logger, "Error code for %s: %s", operation_name.c_str(), plan_success);
    while (plan_success != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(logger, "Visualizing plan for %s: FAILED: %s. Retrying...", operation_name.c_str(), error_code_to_string(plan_success).c_str());
        plan_success = move_group.plan(plan);
    }
    RCLCPP_INFO(logger, "Visualizing plan for %s: SUCCESS.", operation_name.c_str());

    // Execution
    auto move_success = move_group.move();
    while (move_success != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(logger, "Execution of %s motion failed! Retrying...", operation_name.c_str());
        move_success = move_group.move();
    }
    RCLCPP_INFO(logger, "Execution of %s motion completed.", operation_name.c_str());
    rclcpp::sleep_for(std::chrono::seconds(1));
}

int main(int argc, char **argv) {
    // ROS2 Initialization
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("place_node");

    // Logger
    auto logger = rclcpp::get_logger("place_node");

    // Spinner with more thread
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner_thread([&executor]() { executor.spin(); });

    // Wait initialization
    rclcpp::sleep_for(std::chrono::seconds(2));

    // ROS 2 server clients for grasp simulation
    rclcpp::Client<linkattacher_msgs::srv::DetachLink>::SharedPtr detach_link_client = node->create_client<linkattacher_msgs::srv::DetachLink>("/DETACHLINK");

    // MoveIt2 interface setup
    using moveit::planning_interface::MoveGroupInterface;
    MoveGroupInterface arm(node, "arm");
    MoveGroupInterface gripper(node, "gripper");
    arm.setNumPlanningAttempts(5);
    arm.setGoalTolerance(0.01);
    arm.setPlanningTime(30.0);
    RCLCPP_INFO(logger, "Planning frame: %s", arm.getPlanningFrame().c_str());
    // RCLCPP_INFO(logger, "End effector link: %s", arm.getEndEffectorLink().c_str());

    // extend arm forward ---------------------------------------------------------------
    geometry_msgs::msg::PoseStamped extended_pose;
    extended_pose = arm.getCurrentPose();
    extended_pose.header.stamp = node->now();
    extended_pose.pose.position.x += 0.05;
    extended_pose.pose.position.z -= 0.04;
    arm.setPoseTarget(extended_pose);

    perform_sync_movement(logger, "[Extend Arm]", arm);

    // Open gripper --------------------------------------------------------------------
    gripper.setNamedTarget("open");
    perform_sync_movement(logger, "[Open Gripper]", gripper);

    // Detach pickable_object to end effector in RVIZ -----------------------------------
    arm.detachObject("pickable_object");   

    // Detach pickable_object to end effector in Gazebo ---------------------------------
    auto detach_request = std::make_shared<linkattacher_msgs::srv::DetachLink::Request>();
    detach_request->model1_name = "custom_turtlebot3_test";         // name of robot in Gazebo
    detach_request->link1_name = "gripper_left_link";               // name of gripper link
    detach_request->model2_name = "pickable_object_0";              // name of object to pick
    detach_request->link2_name = "object";                          // name of object link

    while (!detach_link_client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(logger, "Waiting for the DetachLink service...");
    }

    auto detach_future = detach_link_client->async_send_request(detach_request);
    if (detach_future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
        auto response = detach_future.get();
        if (response->success) {
            RCLCPP_INFO(logger, "Object detached successfully.");
        } else {
            RCLCPP_ERROR(logger, "Failed to detach object.");
            rclcpp::shutdown();
            spinner_thread.join();
            return 0;
        }
    } else {
        RCLCPP_ERROR(logger, "Failed to receive detach object service.");
        rclcpp::shutdown();
        spinner_thread.join();
        return 0;
    }

    // stop the spinner -----------------------------------------------------------------
    rclcpp::shutdown();
    spinner_thread.join();
    return 0;
}