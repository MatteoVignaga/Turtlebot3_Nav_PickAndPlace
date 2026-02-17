#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <linkattacher_msgs/srv/attach_link.hpp>
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
    while (!plan_success != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(logger, "Visualizing plan for %s: FAILED. Retrying...", operation_name.c_str());
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
    auto node = std::make_shared<rclcpp::Node>("pap_node");

    // Logger
    auto logger = rclcpp::get_logger("pap_node");

    // Spinner with more thread
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner_thread([&executor]() { executor.spin(); });

    // Wait initialization
    rclcpp::sleep_for(std::chrono::seconds(2));

    // ROS 2 servers for grasp simulation
    rclcpp::Client<linkattacher_msgs::srv::AttachLink>::SharedPtr attach_link_client = node->create_client<linkattacher_msgs::srv::AttachLink>("/ATTACHLINK");
    rclcpp::Client<linkattacher_msgs::srv::DetachLink>::SharedPtr detach_link_client = node->create_client<linkattacher_msgs::srv::DetachLink>("/DETACHLINK");

    // MoveIt2 interface setup
    using moveit::planning_interface::MoveGroupInterface;
    MoveGroupInterface arm(node, "arm");
    MoveGroupInterface gripper(node, "gripper");
    arm.setPoseReferenceFrame("base_link");
    arm.setNumPlanningAttempts(5);
    arm.setGoalTolerance(0.01);
    arm.setPlanningTime(30.0);
    // RCLCPP_INFO(logger, "Planning frame: %s", arm.getPlanningFrame().c_str());
    // RCLCPP_INFO(logger, "End effector link: %s", arm.getEndEffectorLink().c_str());
    
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

    // Add objects to the scene ---------------------------------------------------------
    planning_scene_interface.applyCollisionObjects(collision_objects);
    RCLCPP_INFO(logger, "Collision objects added to the planning scene.");

    // Approach pickable_object ---------------------------------------------------------
    /* From tf2 echo:
            At time 172.699000000
            - Translation: [0.109, 0.205, 0.200]
            - Rotation: in Quaternion (xyzw) [0.008, -0.018, 0.401, 0.916]
            - Rotation: in RPY (radian) [0.000, -0.038, 0.825]
    */
    geometry_msgs::msg::Pose approach_object_target_pose;
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, -0.038, 0.825);
    approach_object_target_pose.orientation = tf2::toMsg(orientation);
    approach_object_target_pose.position.x = 0.109;
    approach_object_target_pose.position.y = 0.205;
    approach_object_target_pose.position.z = 0.224;
    arm.setPoseTarget(approach_object_target_pose, "");

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
    attach_request->model1_name = "turtlebot3_manipulation_system";  // name of robot in Gazebo
    attach_request->link1_name = "gripper_left_link";                // name of gripper link
    attach_request->model2_name = "pickable_object";                 // name of object to pick
    attach_request->link2_name = "link";                             // name of object link

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

    // Move to other table --------------------------------------------------------------
    /* From tf2 echo: 
            At time 226.776000000
            - Translation: [0.073, -0.233, 0.224]
            - Rotation: in Quaternion (xyzw) [-0.009, -0.017, -0.475, 0.880]
            - Rotation: in RPY (radian) [0.000, -0.038, -0.991]
    */
    geometry_msgs::msg::Pose move_object_target_pose;
    orientation.setRPY(0.0, -0.038, -0.991);
    move_object_target_pose.orientation = tf2::toMsg(orientation);
    move_object_target_pose.position.x = 0.073;
    move_object_target_pose.position.y = -0.233;
    move_object_target_pose.position.z = 0.224;
    arm.setPoseTarget(move_object_target_pose, "");

    perform_sync_movement(logger, "[Move Object]", arm);

    // Open gripper ---------------------------------------------------------------------
    gripper.setNamedTarget("open");
    perform_sync_movement(logger, "[Open Gripper]", gripper);

    // Detach pickable_object from end effector in RVIZ ---------------------------------
    arm.detachObject("pickable_object");

    // Detach pickable_object from end effector in Gazebo -------------------------------
    auto detach_request = std::make_shared<linkattacher_msgs::srv::DetachLink::Request>();
    detach_request->model1_name = "turtlebot3_manipulation_system";  // name of robot in Gazebo
    detach_request->link1_name = "gripper_left_link";                // name of gripper link 
    detach_request->model2_name = "pickable_object";                 // name of object
    detach_request->link2_name = "link";                             // name of object link

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

    // Move to home ---------------------------------------------------------------------
    arm.setNamedTarget("home");
    perform_sync_movement(logger, "[Home]", arm);

    // stop the spinner -----------------------------------------------------------------
    rclcpp::shutdown();
    spinner_thread.join();
    return 0;
}