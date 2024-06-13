#include <chrono>
#include <cmath>
#include <memory>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

using namespace std::chrono_literals;

class NoisyOdometryNode : public rclcpp::Node
{
public:
  NoisyOdometryNode()
  : Node("noisy_odometry_node"), distribution_(0, 0.05)    // (Mean, Std Deviation)
  {
    // Subscribe to the original odometry topic
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&NoisyOdometryNode::odomCallback, this, std::placeholders::_1));
    
    // Publisher for noisy odometry
    noisy_odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("noisy_odom", 10);
    
    // Broadcaster for the noisy odometry transform
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    RCLCPP_INFO(this->get_logger(), "NoisyOdometryNode started");
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    auto noisy_msg = *msg;

    // Introduce noise
    noisy_msg.pose.pose.position.x += distribution_(generator_);
    noisy_msg.pose.pose.position.y += distribution_(generator_);
    // noisy_msg.pose.pose.position.z += distribution_(generator_);
    // noisy_msg.pose.pose.orientation.x += distribution_(generator_);
    // noisy_msg.pose.pose.orientation.y += distribution_(generator_);
    // noisy_msg.pose.pose.orientation.z += distribution_(generator_);
    // noisy_msg.pose.pose.orientation.w += distribution_(generator_);

    // Publish the noisy odometry message
    noisy_odom_publisher_->publish(noisy_msg);

    // Broadcast the transform based on the noisy odometry
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = msg->header.stamp;
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_footprint";
    transform.transform.translation.x = noisy_msg.pose.pose.position.x;
    transform.transform.translation.y = noisy_msg.pose.pose.position.y;
    transform.transform.translation.z = noisy_msg.pose.pose.position.z;
    transform.transform.rotation = noisy_msg.pose.pose.orientation;

    tf_broadcaster_->sendTransform(transform);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr noisy_odom_publisher_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::default_random_engine generator_;
  std::normal_distribution<double> distribution_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NoisyOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
