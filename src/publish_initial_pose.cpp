#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

class InitialPoseSetterNode : public rclcpp::Node {
public:
    InitialPoseSetterNode() : Node("Initial_pose_setter") {
        rclcpp::QoS qos(10);
        qos.transient_local();

        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", qos);
        pub_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), [this]() {
            geometry_msgs::msg::PoseWithCovarianceStamped msg;
            msg.header.stamp = this->now();
            msg.header.frame_id = "map";
            msg.pose.pose.position.x = 0.0;
            msg.pose.pose.position.y = 0.0;
            msg.pose.pose.position.z = 0.0;
            msg.pose.pose.orientation.x = 0.0;
            msg.pose.pose.orientation.y = 0.0;
            msg.pose.pose.orientation.z = 0.0;
            msg.pose.pose.orientation.w = 1.0;

            // covariance obtained looking at what RViz publishes
            for (int i = 0; i < 36; i++) {
                msg.pose.covariance[i] = 0.0;
            }
            msg.pose.covariance[0] = 0.25;  
            msg.pose.covariance[7] = 0.25;  
            msg.pose.covariance[35] = 0.068;

            pose_publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Initial pose for navigation published!");
            if (pose_publisher_->get_subscription_count() > 0) {
                rclcpp::shutdown();
            }
        });
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_publisher_;
    rclcpp::TimerBase::SharedPtr pub_timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<InitialPoseSetterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}