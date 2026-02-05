#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
//#include <nav2_msgs/action/navigate_through_poses.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

/* void goal_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::WrappedResult &result, 
                    rclcpp::Node::SharedPtr node) {
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(node->get_logger(), "Goal succeeded!");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(node->get_logger(), "Goal was aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(node->get_logger(), "Goal was canceled");
            break;
        default:
            RCLCPP_ERROR(node->get_logger(), "Unknown result code");
            break;
    }
    rclcpp::shutdown();
} */

void goal_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result, 
                    rclcpp::Node::SharedPtr node) {
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(node->get_logger(), "Goal succeeded!");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(node->get_logger(), "Goal was aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(node->get_logger(), "Goal was canceled");
            break;
        default:
            RCLCPP_ERROR(node->get_logger(), "Unknown result code");
            break;
    }
    rclcpp::shutdown();
}

int main(int argc, char const *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("navigate_through_poses");

    /* auto nav2_client = rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>(node, "navigate_through_poses");
    
    auto goal_msg = nav2_msgs::action::NavigateThroughPoses::Goal();

    geometry_msgs::msg::PoseStamped pose1;
    pose1.header.frame_id = "map";
    pose1.pose.position.x = 3.8;
    pose1.pose.position.y = 0.5;
    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, 0);
    pose1.pose.orientation = tf2::toMsg(orientation);
    goal_msg.poses.push_back(pose1);

    geometry_msgs::msg::PoseStamped pose2;
    pose2.header.frame_id = "map";
    pose2.pose.position.x = 4.0;
    pose2.pose.position.y = 0.5;
    pose2.pose.orientation = tf2::toMsg(orientation);
    goal_msg.poses.push_back(pose2);

    geometry_msgs::msg::PoseStamped pose3;
    pose3.header.frame_id = "map";
    pose3.pose.position.x = 4.05;
    pose3.pose.position.y = 0.5;
    pose3.pose.orientation = tf2::toMsg(orientation);
    goal_msg.poses.push_back(pose3);

    nav2_client->wait_for_action_server();
    auto goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SendGoalOptions();
    goal_options.result_callback = std::bind(&goal_callback, std::placeholders::_1, node);

    nav2_client->async_send_goal(goal_msg, goal_options); */

    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(node, "navigate_to_pose");

    auto first_goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    first_goal_msg.pose.header.frame_id = "map";
    first_goal_msg.pose.header.stamp = node->now();
    first_goal_msg.pose.pose.position.x = 3.8;
    first_goal_msg.pose.pose.position.y = 0.5;
    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, 0);
    first_goal_msg.pose.pose.orientation = tf2::toMsg(orientation);
    
    nav_client->wait_for_action_server();
    
    // Richiesta del goal
    auto first_goal_future = nav_client->async_send_goal(first_goal_msg);
    if (rclcpp::spin_until_future_complete(node, first_goal_future) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Failed to send first goal.");
        rclcpp::shutdown();
        return 1;
    }

    // Ottenimento handle ed esecuzione
    auto first_goal_handle = first_goal_future.get();
    auto first_result_future = nav_client->async_get_result(first_goal_handle);
    if (rclcpp::spin_until_future_complete(node, first_result_future) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Failed to navigate to execute goal.");
        rclcpp::shutdown();
        return 1;
    }

    // Ottenimento risultato
    auto first_result = first_result_future.get();
    if (first_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_ERROR(node->get_logger(), "Failed to navigate to first pose.");
        rclcpp::shutdown();
        return 1;
    }

    RCLCPP_INFO(node->get_logger(), "Reached first pose!");

    auto second_goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    second_goal_msg.pose.header.frame_id = "map";
    second_goal_msg.pose.header.stamp = node->now();
    second_goal_msg.pose.pose.position.x = 4.05;
    second_goal_msg.pose.pose.position.y = 0.5;
    second_goal_msg.pose.pose.orientation = tf2::toMsg(orientation);
    
    nav_client->wait_for_action_server();
    
    // Richiesta del goal
    auto second_goal_future = nav_client->async_send_goal(second_goal_msg);
    if (rclcpp::spin_until_future_complete(node, second_goal_future) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Failed to send second goal.");
        rclcpp::shutdown();
        return 1;
    }

    // Ottenimento handle ed esecuzione
    auto second_goal_handle = second_goal_future.get();
    auto second_result_future = nav_client->async_get_result(second_goal_handle);
    if (rclcpp::spin_until_future_complete(node, second_result_future) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Failed to navigate to execute goal.");
        rclcpp::shutdown();
        return 1;
    }

    // Ottenimento risultato
    auto second_result = second_result_future.get();
    if (second_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_ERROR(node->get_logger(), "Failed to navigate to second pose.");
        rclcpp::shutdown();
        return 1;
    }

    RCLCPP_INFO(node->get_logger(), "Reached second pose!");

    // rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
