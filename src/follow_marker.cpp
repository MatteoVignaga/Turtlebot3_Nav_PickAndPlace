#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cmath>

class FollowMarkerNode : public rclcpp::Node {
public: 
    FollowMarkerNode() : Node("follow_marker") {
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&FollowMarkerNode::publish_velocity, this));
        marker_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/marker_pose", 
            10, 
            std::bind(&FollowMarkerNode::read_marker_position, this, std::placeholders::_1));
    }

    /**
     * TODO: modifica per pubblicare cmd_vel in relatione a pose.
     * Nota: pose è relativa a base_footprint.
     * Nota: angular.z positiva gira a sinistra.
     * 
     * Problema: come faccio se il marker non è stato rilevato?
     * 
     * Possibile soluzione: pubblicare un messaggio se non è in vista,
     * ma non sono sicuro si possa con ros2_aruco perchè quando non 
     * vengono rilevati marker non viene pubblicato nulla. Strano 
     * perchè leggendo il codice la pubblicazione avviene in una
     * callback sul topic delle immagini, quindi dovrebbe pubblicare 
     * ad ogni immagine pubblicata (aka ad ogni frame video).
     * 
     * Possibile soluzione: confronto il timestamp con now(): se è
     * più vecchio di mezzo secondo mi fermo perchè significa che non
     * è più stato rilevato nessun marcatore. Se il marcatore sta a
     * meno di 15cm di distanza mi fermo. Altrimenti, vai avanti e gira
     * in base alle coordinate x ed y del marcatore.
     */
    void publish_velocity() {
        geometry_msgs::msg::Twist msg;
        rclcpp::Duration interval(std::chrono::nanoseconds(200 * 1000000)); // 200ms
        rclcpp::Duration delay = this->now() - target_position_.header.stamp;

        if (delay > interval) {
            // Nessuna posa ottenuta da >200ms: ferma il robot.
            RCLCPP_INFO(this->get_logger(), "No target pose detected in more than 200ms.");
            msg.linear.x = 0.0;
            msg.angular.z = 0.0;
            cmd_vel_publisher_->publish(msg);
        } else {
            // Posa valida: se non è stato raggiunto, vai al target.
            // Obiettivo: arrivare almeno a 30cm dall'oggetto, con al
            // massimo 2° di tolleranza in rotazione.
            msg.linear.x = 0.0;
            msg.angular.z = 0.0;
            double theta = atan2(target_position_.pose.position.y, target_position_.pose.position.x); // in radians
            if (target_position_.pose.position.x > 0.3) msg.linear.x = 0.15;
            if (theta > 0.05) msg.angular.z = 0.3;
            if (theta < -0.05) msg.angular.z = -0.3;
            RCLCPP_INFO(this->get_logger(), "Publishing message: { linear.x = %f, angular.z = %f }.", msg.linear.x, msg.angular.z);
            cmd_vel_publisher_->publish(msg);

            if (msg.linear.x == 0.0 && msg.angular.z == 0) {
                RCLCPP_INFO(this->get_logger(), "Target pose reached!");
                rclcpp::shutdown();
            }
        } 
    }

    void read_marker_position(const geometry_msgs::msg::PoseStamped pose) {
        target_position_ = pose;
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr marker_pose_subscriber_;
    geometry_msgs::msg::PoseStamped target_position_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FollowMarkerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}