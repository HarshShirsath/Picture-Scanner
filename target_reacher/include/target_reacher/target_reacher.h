#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <string>
#include "bot_controller/bot_controller.h"
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

// timer
class TargetReacher : public rclcpp::Node
{
public:
    TargetReacher(std::shared_ptr<BotController> const &bot_controller) : Node("target_reacher")
    {

        m_bot_controller = bot_controller;

        // Declare parameters
        auto aruco_target_x = this->declare_parameter<double>("aruco_target.x");
        auto aruco_target_y = this->declare_parameter<double>("aruco_target.y");

        this->declare_parameter<std::string>("final_destination.frame_id");
        this->declare_parameter<double>("final_destination.aruco_0.x");
        this->declare_parameter<double>("final_destination.aruco_0.y");
        this->declare_parameter<double>("final_destination.aruco_1.x");
        this->declare_parameter<double>("final_destination.aruco_1.y");
        this->declare_parameter<double>("final_destination.aruco_2.x");
        this->declare_parameter<double>("final_destination.aruco_2.y");
        this->declare_parameter<double>("final_destination.aruco_3.x");
        this->declare_parameter<double>("final_destination.aruco_3.y");

        // Set Goal1 for aruco target
        m_bot_controller->set_goal(aruco_target_x, aruco_target_y);


        m_tf_buffer =
            std::make_unique<tf2_ros::Buffer>(this->get_clock());

        
        // goal_reached subscriber callback
        m_subscriber_goal_reached = this->create_subscription<std_msgs::msg::Bool>(
            "/goal_reached", 10,
            std::bind(&TargetReacher::goal_reached_callback, this, std::placeholders::_1));

        m_rotation_publisher = this->create_publisher<geometry_msgs::msg::Twist>(
            "/robot1/cmd_vel",10);

        // aruco_markers subscriber callback
        m_subscriber_aruco_markers = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
            "/aruco_markers", 10,
            std::bind(&TargetReacher::aruco_markers_callback, this,std::placeholders::_1));

        
        // Initialize the static transform broadcaster
        m_tf_broadcaster =
            std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);

        // Initialize the transform listener
        m_tf_listener =
            std::make_unique<tf2_ros::TransformListener>(*m_tf_buffer);

    }

private:
    // attributes
    std::shared_ptr<BotController> m_bot_controller;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_subscriber_goal_reached;
    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr m_subscriber_aruco_markers;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_rotation_publisher;

    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> m_tf_broadcaster{nullptr};
    std::unique_ptr<tf2_ros::TransformListener> m_tf_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;

    bool m_is_aruco_marker_detected{false};

    // methods
    void goal_reached_callback(const std::shared_ptr<std_msgs::msg::Bool> goal_reached);
    void aruco_markers_callback(const std::shared_ptr<ros2_aruco_interfaces::msg::ArucoMarkers> msg);

    void get_final_destination_from_parameters(const int marker_id, double* x,double*y);
    void broadcast_frame_final_destination(const std::string &given_frame,const double retrived_x,const double retrived_y);
    void compute_goal_in_odom_frame(double* final_destination_x,double*final_destination_y);
};