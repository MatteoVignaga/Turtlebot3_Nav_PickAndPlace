#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <linkattacher_msgs/srv/attach_link.hpp>
#include <tb3_nav_pick_and_place/srv/get_marker_pose.hpp>

using namespace tb3_nav_pick_and_place::srv;

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
    auto node = std::make_shared<rclcpp::Node>("pick_node");

    // Logger
    auto logger = rclcpp::get_logger("pick_node");

    // Spinner with more thread
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner_thread([&executor]() { executor.spin(); });

    // Wait initialization
    rclcpp::sleep_for(std::chrono::seconds(2));

    // ROS 2 server clients for grasp simulation
    rclcpp::Client<linkattacher_msgs::srv::AttachLink>::SharedPtr attach_link_client = node->create_client<linkattacher_msgs::srv::AttachLink>("/ATTACHLINK");
    rclcpp::Client<GetMarkerPose>::SharedPtr get_pose_client = node->create_client<GetMarkerPose>("/GetMarkerPose");

    // MoveIt2 interface setup
    using moveit::planning_interface::MoveGroupInterface;
    MoveGroupInterface arm(node, "arm");
    MoveGroupInterface gripper(node, "gripper");
    arm.setPoseReferenceFrame("base_link");
    arm.setNumPlanningAttempts(5);
    arm.setGoalTolerance(0.01);
    arm.setPlanningTime(30.0);
    RCLCPP_INFO(logger, "Planning frame: %s", arm.getPlanningFrame().c_str());
    // RCLCPP_INFO(logger, "End effector link: %s", arm.getEndEffectorLink().c_str());

    // Retrieve object position ---------------------------------------------------------
    auto pose_request = std::make_shared<GetMarkerPose::Request>();
    while(!get_pose_client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(logger, "Waiting for marker pose detection...");
    }

    auto get_pose_future = get_pose_client->async_send_request(pose_request);
    geometry_msgs::msg::PoseStamped approach_object_target_pose;
    if (get_pose_future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
        RCLCPP_INFO(logger, "Object position retrieved successfully.");
        approach_object_target_pose = get_pose_future.get()->marker_pose;
        approach_object_target_pose.pose.position.z -= 0.15;   // 15cm lower to account for marker position relative to obj
        approach_object_target_pose.pose.position.x -= 0.05;   // 4cm closer to avoid reaching too far "into" the object
        RCLCPP_INFO(logger, "Approaching object at: {x:%f, y:%f, z:%f}", approach_object_target_pose.pose.position.x,
                                                                        approach_object_target_pose.pose.position.y,
                                                                        approach_object_target_pose.pose.position.z);
    } else {
        RCLCPP_ERROR(logger, "Failed to retrieve object position.");
        rclcpp::shutdown();
        spinner_thread.join();
        return 0;
    }

    arm.setPoseTarget(approach_object_target_pose);

    perform_sync_movement(logger, "[Approach Object]", arm);

    // Close gripper --------------------------------------------------------------------
    gripper.setNamedTarget("close");
    perform_sync_movement(logger, "[Close Gripper]", gripper);

    // Attach pickable_object to end effector in RVIZ -----------------------------------
    arm.attachObject("pickable_object", 
                    "end_effector_link", 
                    {"gripper_left_link", "gripper_right_link"}
                );   

    // Attach pickable_object to end effector in Gazebo ---------------------------------
    auto attach_request = std::make_shared<linkattacher_msgs::srv::AttachLink::Request>();
    attach_request->model1_name = "custom_turtlebot3_test";         // name of robot in Gazebo
    attach_request->link1_name = "gripper_left_link";               // name of gripper link
    attach_request->model2_name = "pickable_object_0";              // name of object to pick
    attach_request->link2_name = "object";                          // name of object link

    while (!attach_link_client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(logger, "Waiting for the AttachLink service...");
    }

    auto attach_future = attach_link_client->async_send_request(attach_request);
    if (attach_future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
        auto response = attach_future.get();
        if (response->success) {
            RCLCPP_INFO(logger, "Object attached successfully.");
        } else {
            RCLCPP_ERROR(logger, "Failed to attach object.");
            rclcpp::shutdown();
            spinner_thread.join();
            return 0;
        }
    } else {
        RCLCPP_ERROR(logger, "Failed to receive attach object service.");
        rclcpp::shutdown();
        spinner_thread.join();
        return 0;
    }

    // Lift object ---------------------------------------------------------------------
    geometry_msgs::msg::Pose lift_pose;
    lift_pose = approach_object_target_pose.pose;
    lift_pose.position.x -= 0.05; // bring closer
    lift_pose.position.z += 0.05; // lift up
    arm.setPoseTarget(lift_pose);
    perform_sync_movement(logger, "[Lift Object]", arm);

    // stop the spinner -----------------------------------------------------------------
    rclcpp::shutdown();
    spinner_thread.join();
    return 0;
}