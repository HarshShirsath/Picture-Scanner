#include <string>
#include "odom_updater.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/pose.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include "rclcpp/rclcpp.hpp"

void OdomUpdater::update_odom_callback(const std::shared_ptr<nav_msgs::msg::Odometry> msg)
{
    std::string odom = "/robot1/odom";
    std::string base_footprint = "/robot1/base_footprint";

    geometry_msgs::msg::TransformStamped t;

    /*******************************************
     * broadcaster: "/robot1/odom" -> "/robot1/base_footprint"
     *******************************************/
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = odom;
    t.child_frame_id = base_footprint;

    t.transform.translation.x = msg->pose.pose.position.x;
    t.transform.translation.y = msg->pose.pose.position.y;
    t.transform.translation.z = msg->pose.pose.position.z;

    t.transform.rotation.x = msg->pose.pose.orientation.x;
    t.transform.rotation.y = msg->pose.pose.orientation.y;
    t.transform.rotation.z = msg->pose.pose.orientation.z;
    t.transform.rotation.w = msg->pose.pose.orientation.w;

    // Send the transformation
    m_tf_broadcaster->sendTransform(t);
}
