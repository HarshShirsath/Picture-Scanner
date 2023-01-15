#pragma once

#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <string>

class OdomUpdater : public rclcpp::Node
{
public:
    OdomUpdater(std::string node_name) : Node(node_name)
    {
        // Initialize the transform broadcaster
        m_tf_broadcaster =
            std::make_unique<tf2_ros::TransformBroadcaster>(this);

        // odom subscriber callback
        m_subscriber_odom = this->create_subscription<nav_msgs::msg::Odometry>(
            "/robot1/odom", 10,
            std::bind(&OdomUpdater::update_odom_callback, this, std::placeholders::_1));
    }

private:
    // attributes
    std::unique_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster{nullptr};
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_subscriber_odom;
   

    // methods
    void update_odom_callback(const std::shared_ptr<nav_msgs::msg::Odometry> msg);
};