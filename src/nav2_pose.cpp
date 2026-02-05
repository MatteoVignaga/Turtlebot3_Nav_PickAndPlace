#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using NavigateToPoseAction = nav2_msgs::action::NavigateToPose;
using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPoseAction>;

void goal_result_callback(const GoalHandle::WrappedResult &result, rclcpp::Node::SharedPtr node) {
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

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("navigate_to_pose");
    rclcpp_action::Client<NavigateToPoseAction>::SharedPtr nav_client = rclcpp_action::create_client<NavigateToPoseAction>(node, "navigate_to_pose");
    node->declare_parameter<double>("x", 0.0);
    node->declare_parameter<double>("y", 0.0);
    node->declare_parameter<double>("yaw", 0.0);

    auto goal_msg = NavigateToPoseAction::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = node->now();
    goal_msg.pose.pose.position.x = node->get_parameter("x").as_double();
    goal_msg.pose.pose.position.y = node->get_parameter("y").as_double();
    double yaw = node->get_parameter("yaw").as_double();
    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, yaw);
    goal_msg.pose.pose.orientation = tf2::toMsg(orientation);
    
    nav_client->wait_for_action_server();
    
    auto send_goal_options = rclcpp_action::Client<NavigateToPoseAction>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(&goal_result_callback, 
                    std::placeholders::_1, node);
    
    nav_client->async_send_goal(goal_msg, send_goal_options);
    rclcpp::spin(node);
}