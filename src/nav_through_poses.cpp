#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

int main(int argc, char const *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("navigate_through_poses");

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
