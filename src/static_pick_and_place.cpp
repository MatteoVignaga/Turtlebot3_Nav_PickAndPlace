#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <linkattacher_msgs/srv/attach_link.hpp>
#include <linkattacher_msgs/srv/detach_link.hpp>

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

    // ros2 servers for grasp simulation
    rclcpp::Client<linkattacher_msgs::srv::AttachLink>::SharedPtr attach_link_client = node->create_client<linkattacher_msgs::srv::AttachLink>("/ATTACHLINK");
    rclcpp::Client<linkattacher_msgs::srv::DetachLink>::SharedPtr detach_link_client = node->create_client<linkattacher_msgs::srv::DetachLink>("/DETACHLINK");

    // MoveIt2 interface
    using moveit::planning_interface::MoveGroupInterface;
    MoveGroupInterface arm(node, "arm");
    MoveGroupInterface gripper(node, "gripper");
    arm.setPoseReferenceFrame("base_link");
    arm.setNumPlanningAttempts(5);
    arm.setGoalTolerance(0.01);
    arm.setPlanningTime(30.0);
    
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

    // Approach pickable_object
    /* At time 172.699000000
    - Translation: [0.109, 0.205, 0.200]
    - Rotation: in Quaternion (xyzw) [0.008, -0.018, 0.401, 0.916]
    - Rotation: in RPY (radian) [0.000, -0.038, 0.825]
    - Rotation: in RPY (degree) [0.000, -2.189, 47.261]
    - Matrix:
    0.678 -0.734 -0.026  0.109
    0.734  0.679 -0.028  0.205
    0.038  0.000  0.999  0.224
    0.000  0.000  0.000  1.000 */
    geometry_msgs::msg::Pose approach_object_target_pose;
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, -0.038, 0.825);
    approach_object_target_pose.orientation = tf2::toMsg(orientation);
    approach_object_target_pose.position.x = 0.109;
    approach_object_target_pose.position.y = 0.205;
    approach_object_target_pose.position.z = 0.224;
    arm.setPoseTarget(approach_object_target_pose, "");

    RCLCPP_INFO(logger, "Planning frame: %s", arm.getPlanningFrame().c_str());
    RCLCPP_INFO(logger, "End effector link: %s", arm.getEndEffectorLink().c_str());

    // Planning
    moveit::planning_interface::MoveGroupInterface::Plan approach_plan;
    bool approach_plan_success = (arm.plan(approach_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(logger, "Visualizing approach plan: %s", approach_plan_success ? "SUCCESS" : "FAILED");

    // Execution
    if (approach_plan_success) {
        auto approach_move_success = arm.move();
        if (approach_move_success == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(logger, "Approach motion execution completed.");
            rclcpp::sleep_for(std::chrono::seconds(1));
        } else {
            RCLCPP_ERROR(logger, "Approach motion failed!");
            rclcpp::shutdown();
            spinner_thread.join();
            return 0;
        }
        
    } else {
        RCLCPP_ERROR(logger, "Motion planning for approach failed!");
        rclcpp::shutdown();
        spinner_thread.join();
        return 0;
    }

    // Close gripper
    gripper.setNamedTarget("close");
    // Planning
    moveit::planning_interface::MoveGroupInterface::Plan close_gripper_plan;
    bool close_gripper_plan_success = (gripper.plan(close_gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(logger, "Visualizing pick plan: %s", close_gripper_plan_success ? "SUCCESS" : "FAILED");

    // Execution
    if (close_gripper_plan_success) {
        auto close_gripper_move_success = gripper.move();
        if (close_gripper_move_success == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(logger, "Close gripper motion execution completed.");
            rclcpp::sleep_for(std::chrono::seconds(1));
        } else {
            RCLCPP_ERROR(logger, "Close gripper motion failed!");
            rclcpp::shutdown();
            spinner_thread.join();
            return 0;
        }
    } else {
        RCLCPP_ERROR(logger, "Motion planning for close gripper failed!");
        rclcpp::shutdown();
        spinner_thread.join();
        return 0;
    }

    // Attach pickable_object to end effector in RVIZ
    arm.attachObject("pickable_object", "end_effector_link", {"gripper_left_link", "gripper_right_link"});   

    // Attach pickable_object to end effector in Gazebo
    auto attach_request = std::make_shared<linkattacher_msgs::srv::AttachLink::Request>();
    attach_request->model1_name = "turtlebot3_manipulation_system";  // Nome del robot in Gazebo
    attach_request->link1_name = "gripper_left_link";                // Nome del link del gripper
    attach_request->model2_name = "pickable_object";                 // Nome dell'oggetto da afferrare
    attach_request->link2_name = "link";                             // Nome del link dell'oggetto

    while (!attach_link_client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(logger, "Waiting for the AttachLink service...");
    }

    auto attach_future = attach_link_client->async_send_request(attach_request);
    if (attach_future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
        auto response = attach_future.get();
        if (response->success == true) {
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

    // Move to other table
    /* At time 226.776000000
    - Translation: [0.073, -0.233, 0.224]
    - Rotation: in Quaternion (xyzw) [-0.009, -0.017, -0.475, 0.880]
    - Rotation: in RPY (radian) [0.000, -0.038, -0.991]
    - Rotation: in RPY (degree) [0.000, -2.187, -56.773]
    - Matrix:
    0.548  0.837 -0.021  0.073
    -0.836  0.548  0.032 -0.233
    0.038  0.000  0.999  0.224
    0.000  0.000  0.000  1.000 */
    geometry_msgs::msg::Pose move_object_target_pose;
    orientation.setRPY(0.0, -0.038, -0.991);
    move_object_target_pose.orientation = tf2::toMsg(orientation);
    move_object_target_pose.position.x = 0.073;
    move_object_target_pose.position.y = -0.233;
    move_object_target_pose.position.z = 0.224; // per non sollevare troppo l'object
    arm.setPoseTarget(move_object_target_pose, "");

    RCLCPP_INFO(logger, "Planning frame: %s", arm.getPlanningFrame().c_str());
    RCLCPP_INFO(logger, "End effector link: %s", arm.getEndEffectorLink().c_str());

    // Planning
    moveit::planning_interface::MoveGroupInterface::Plan move_object_plan;
    bool move_object_plan_success = (arm.plan(move_object_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(logger, "Visualizing place plan: %s", move_object_plan_success ? "SUCCESS" : "FAILED");

    // Execution
    if (move_object_plan_success) {
        auto move_object_move_success = arm.move();
        if (move_object_move_success == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(logger, "Move object motion execution completed.");
            rclcpp::sleep_for(std::chrono::seconds(1));
        } else {
            RCLCPP_ERROR(logger, "Move object motion failed!");
            rclcpp::shutdown();
            spinner_thread.join();
            return 0;
        }
    } else {
        RCLCPP_ERROR(logger, "Motion planning for move object failed!");
        rclcpp::shutdown();
        spinner_thread.join();
        return 0;
    }

    // Open gripper
    gripper.setNamedTarget("open");
    // Planning
    moveit::planning_interface::MoveGroupInterface::Plan open_gripper_plan;
    bool open_gripper_plan_success = (gripper.plan(open_gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(logger, "Visualizing pick plan: %s", open_gripper_plan_success ? "SUCCESS" : "FAILED");

    // Execution
    if (open_gripper_plan_success) {
        auto open_gripper_move_success = gripper.move();
        if (open_gripper_move_success == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(logger, "Open gripper motion execution completed.");
            rclcpp::sleep_for(std::chrono::seconds(1));
        } else {
            RCLCPP_ERROR(logger, "Open gripper motion failed!");
            rclcpp::shutdown();
            spinner_thread.join();
            return 0;
        }
    } else {
        RCLCPP_ERROR(logger, "Motion planning for open gripper failed!");
        rclcpp::shutdown();
        spinner_thread.join();
        return 0;
    }

    // Detach pickable_object from end effector in RVIZ
    arm.detachObject("pickable_object");

    // Detach pickable_object from end effector in Gazebo
    auto detach_request = std::make_shared<linkattacher_msgs::srv::DetachLink::Request>();
    detach_request->model1_name = "turtlebot3_manipulation_system";  // Nome del robot in Gazebo
    detach_request->link1_name = "gripper_left_link";                // Nome del link del gripper
    detach_request->model2_name = "pickable_object";                 // Nome dell'oggetto da staccare
    detach_request->link2_name = "link";                             // Nome del link dell'oggetto

    while (!detach_link_client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(logger, "Waiting for the DetachLink service...");
    }

    auto detach_future = detach_link_client->async_send_request(detach_request);
    if (detach_future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
        auto response = detach_future.get();
        if (response->success == true) {
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

    // Move to home
    arm.setNamedTarget("home");
    moveit::planning_interface::MoveGroupInterface::Plan home_plan;
    bool home_success = (gripper.plan(home_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(logger, "Visualizing pick plan: %s", home_success ? "SUCCESS" : "FAILED");

    // Execution
    if (home_success) {
        arm.move();
        RCLCPP_INFO(logger, "Home motion execution completed.");
        rclcpp::sleep_for(std::chrono::seconds(1));
    } else {
        RCLCPP_ERROR(logger, "Motion planning for home failed!");
        rclcpp::shutdown();
        spinner_thread.join();
        return 0;
    }

    // stop the spinner
    rclcpp::shutdown();
    spinner_thread.join();
    return 0;
}