#include <string>
#include <rclcpp/rclcpp.hpp>
#include "target_reacher/target_reacher.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/pose.hpp"

void TargetReacher::goal_reached_callback(const std::shared_ptr<std_msgs::msg::Bool> goal_reached)
{
    if (goal_reached->data == true && !m_is_aruco_marker_detected)
    {
        geometry_msgs::msg::Twist vel;
        vel.angular.z = 0.2;
        m_rotation_publisher->publish(vel);
    }
}


void TargetReacher::aruco_markers_callback(const std::shared_ptr<ros2_aruco_interfaces::msg::ArucoMarkers> msg)
{
    if (msg)
    {
        m_is_aruco_marker_detected = true;
        double retrived_x{0.0};
        double retrived_y{0.0};

        int marker_id = msg->marker_ids.at(0);

        get_final_destination_from_parameters(marker_id, &retrived_x, &retrived_y);

        std::string origin = this->get_parameter("final_destination.frame_id").as_string();
        broadcast_frame_final_destination(origin, retrived_x, retrived_y);

        double final_goal_x{0.0};
        double final_goal_y{0.0};
        compute_goal_in_odom_frame(&final_goal_x,&final_goal_y);

        // Set goal
        m_bot_controller->set_goal(final_goal_x, final_goal_y);
    }
}


void TargetReacher::get_final_destination_from_parameters(const int marker_id, double *x, double *y)
{
    if (!x || !y)
    {
        RCLCPP_WARN(this->get_logger(), "Missing required arguments");
        return;
    }

    switch (marker_id)
    {
    case 0:
        *x = this->get_parameter("final_destination.aruco_0.x").as_double();
        *y = this->get_parameter("final_destination.aruco_0.y").as_double();
        break;

    case 1:
        *x = this->get_parameter("final_destination.aruco_1.x").as_double();
        *y = this->get_parameter("final_destination.aruco_1.y").as_double();
        break;

    case 2:
        *x= this->get_parameter("final_destination.aruco_2.x").as_double();
        *y= this->get_parameter("final_destination.aruco_2.y").as_double();
        break;

    case 3:
        *x = this->get_parameter("final_destination.aruco_3.x").as_double();
        *y= this->get_parameter("final_destination.aruco_3.y").as_double();
        break;

    default:
        break;
    }
}


void TargetReacher::broadcast_frame_final_destination(const std::string &given_frame, const double retrived_x, const double retrived_y)
{
    geometry_msgs::msg::TransformStamped t;

    /*******************************************
     * static broadcaster: "given_frame" -> "final_destination"
     *******************************************/
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = given_frame;
    t.child_frame_id = "final_destination";

    t.transform.translation.x = retrived_x;
    t.transform.translation.y = retrived_y;
    t.transform.translation.z = 0;

    t.transform.rotation.x = 0;
    t.transform.rotation.y = 0;
    t.transform.rotation.z = 0;
    t.transform.rotation.w = 1;

    // Send the transformation
    m_tf_broadcaster->sendTransform(t);
}


void TargetReacher::compute_goal_in_odom_frame(double *final_destination_x, double *final_destination_y)
{
    if (!final_destination_x || !final_destination_y)
    {
        RCLCPP_WARN(this->get_logger(), "Missing required arguments");
        return;
    }

    geometry_msgs::msg::TransformStamped f;

    // Look up for the transformation between odom and final_destination frames
    try
    {
        f = m_tf_buffer->lookupTransform("robot1/odom", "final_destination", tf2::TimePointZero);
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            "robot1/odom", "final_destination", ex.what());
        return;
    }

    *final_destination_x = f.transform.translation.x;
    *final_destination_y = f.transform.translation.y;
}
