#ifndef PICO_SIMULATOR_ROBOT_H_
#define PICO_SIMULATOR_ROBOT_H_

#include "virtualbase.h"
#include "jsonconfig.h"
#include "world.h"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <iostream>
#include <string>

struct MapConfig{
    double mapResolution, mapOffsetX, mapOffsetY, mapOrientation;
    bool mapInitialised;
};
class Robot : public rclcpp::Node
{

public:
    Robot(const std::string &name, Id id, bool disable_speedcap = false, bool uncertain_odom = false);

    ~Robot();

    void pubTransform(const geo::Pose3D &pose);

    void internalTransform();

    void importMetadata(const nav_msgs::msg::MapMetaData& metadata);

    geo::Pose3D laser_pose;
    std::string robot_name;
    Id robot_id;
    Virtualbase base;

    std::shared_ptr<const geometry_msgs::msg::Twist> base_ref_;   
    bool request_open_door_;

    // Publisher
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_bumperF;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_bumperR;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_laser;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose;
    tf2_ros::TransformBroadcaster pub_tf2;
    tf2_ros::StaticTransformBroadcaster pub_tf2static;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joints_ground_truth;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joints_internal;

private:
    // Node Handle is not needed in ROS 2, as the class itself is a Node

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_base_ref;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_open_door;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_speak;
    void baseReferenceCallback(const std::shared_ptr<const geometry_msgs::msg::Twist>& msg);
    void openDoorCallback(const std::shared_ptr<const std_msgs::msg::Empty>& msg);
    void speakCallback(const std::shared_ptr<const std_msgs::msg::String>& msg);
};


typedef std::shared_ptr<Robot> RobotPtr;


#endif
