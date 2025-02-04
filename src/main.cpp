#include "world.h"
#include "visualization.h"
#include "heightmap.h"
#include "lrf.h"
#include "bumper.h"
#include "door.h"
#include "robot.h"

#include <unistd.h>
#include <geolib/ros/msg_conversions.h>
#include <geolib/Box.h>
#include "jsonconfig.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>
#include <string>

#include <tf2/LinearMath/Quaternion.h>
#include <geolib/shapes.h>
#include "virtualbase.h"
#include "moving_object.h"
#include "random"

#include <vector>


int main(int argc, char **argv){

    std::string config_filename;
    std::string heightmap_filename;

    for(int i = 1; i < argc; i++){
        std::string config_supplied("--config");
        std::string map_supplied("--map");
        if(config_supplied.compare(argv[i])==0 && i + 1 < argc){
            config_filename = std::string(argv[i+1]);
        }
        if(map_supplied.compare(argv[i])==0 && i + 1 < argc){
            heightmap_filename = std::string(argv[i+1]);
        }
    }

    // REMEMBER TO CHANGE THE DEFAULT PATHS
    if (config_filename.empty()) {
        config_filename = "/home/wiktor/Desktop/mrc/emc_simulator/data/defaultconfig.json";
    }
    if (heightmap_filename.empty()) {
        heightmap_filename = "/home/wiktor/Desktop/mrc/emc_simulator/data/heightmap.pgm";
    }

    // Debug prints to check file paths
    std::cout << "Config file path: " << config_filename << std::endl;
    std::cout << "Heightmap file path: " << heightmap_filename << std::endl;

    // Check if the config file exists
    std::ifstream config_file(config_filename);
    if (!config_file.good()) {
        std::cerr << "Config file not found or not accessible: " << config_filename << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("pyro_simulator");

    // Create jointstate publisher (for use with state_publisher node)
    auto marker_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>("geometry", 10);

    Config config(config_filename);
    config.print();

    World world;
    LRF lrf;
    lrf.setAngleLimits(-2, 2);
    lrf.setNumBeams(1000);
    lrf.setRangeLimits(0.01, 10);

    // The bumper class implementation uses an artificial lrf sensor
    Bumper bumper;
    double robot_radius = 0.21; // HERO radius [m]
    if (config.use_pyro.value()) {
        robot_radius = 0.13; // PYRO radius [m]
    }
    bumper.setRobotRadius(robot_radius);
    double bumperSize = 0.01; // [m]
    bumper.setBumperRadius(bumperSize);

    double cycle_freq = 30;

    bool visualize = false;

    // Load heightmap
    std::vector<Door> doors;
    MapLoader loader;
    cv::Mat mapImage;
    nav_msgs::msg::MapMetaData metadata;
    loader.getMapImage(mapImage);
    loader.getMapMetadata(metadata);
    if (!loader.isInitialized())
    {
        std::cout << "[pyro SIMULATOR] Heightmap could not be loaded from server" << std::endl;
        return 1;
    }
    geo::ShapePtr heightmap = createHeightMapShape(mapImage, metadata.resolution, doors);
    if (!heightmap)
    {
        std::cout << "[pyro SIMULATOR] Heightmap could not be converted to shape" << std::endl;
        return 1;
    }
    world.addObject(geo::Pose3D::identity(), heightmap, geo::Vector3(0, 0, 1), walltype);

    // Get centerbox for height map (the visualization is constraining canvas within this box)
    visualization::Bbox bbox; bbox.xmin = -1; bbox.xmax = 1; bbox.ymin=-1; bbox.ymax = 1;

    std::vector<geo::Vector3> meshpoints = heightmap->getMesh().getPoints();

    bbox.xmin = std::max_element(meshpoints.begin(), meshpoints.end(),[](const geo::Vector3& a, const geo::Vector3& b){return a.getX() > b.getX() ;} )->getX();
    bbox.ymin = std::max_element(meshpoints.begin(), meshpoints.end(),[](const geo::Vector3& a, const geo::Vector3& b){return a.getY() > b.getY() ;} )->getY();
    bbox.xmax = std::max_element(meshpoints.begin(), meshpoints.end(),[](const geo::Vector3& a, const geo::Vector3& b){return a.getX() < b.getX() ;} )->getX();
    bbox.ymax = std::max_element(meshpoints.begin(), meshpoints.end(),[](const geo::Vector3& a, const geo::Vector3& b){return a.getY() < b.getY() ;} )->getY();

    // Add moving objects
    for(std::vector<MovingObject>::iterator it = config.moving_objects.value().begin(); it != config.moving_objects.value().end(); ++it) {
        it->id = world.addObject(it->init_pose,makeWorldSimObject(*it),geo::Vector3(0,1,1), movingObjecttype);
        world.setVelocity(it->id,geo::Vector3(0.0,0.0,0.0),0.0);
    }

    // Add robot
    geo::ShapePtr robot_shape = std::make_shared<geo::Shape>();
    geo::createCylinder(*robot_shape, robot_radius, 1, 32); // height = 1, nvertices = 32;
    geo::Vector3 robot_color(0, 0, 1);
    
    geo::Pose3D spawnLocation;
    if (config.spawn_provided.value())
        spawnLocation = config.spawn.value();
    else // no init position specified. spawn robot in the middle of the map.
    {
        double map_width = mapImage.cols * metadata.resolution;
        double map_height = mapImage.rows * metadata.resolution;
        spawnLocation = geo::Pose3D(map_width / 2, map_height / 2, 0,0,0, 1.57);
    }
    Id pyro_id = world.addObject(spawnLocation, robot_shape, robot_color, robottype);
    Robot robot("pyro", pyro_id, config.disable_speedcap.value(), config.uncertain_odom.value());

    // Add transform from ground truth to internal robot
    if (config.provide_internal_pose.value()) {robot.internalTransform();}

    // Add door
    for(std::vector<Door>::iterator it = doors.begin(); it != doors.end(); ++it)
    {
        Door& door = *it;
        door.id = world.addObject(door.init_pose, door.shape, geo::Vector3(0, 1, 0), doortype);
    }

    RCLCPP_INFO(node->get_logger(), "start cycle");
    rclcpp::Rate r(cycle_freq);
    double time_ = 0;
    double dt;
    while(rclcpp::ok())
    {
        rclcpp::spin_some(node);
        rclcpp::Time time = node->now();

        if(time_==0){
            dt = 0;
            time_ = time.seconds();
        }
        else{
            dt = time.seconds() - time_;
            time_ = time.seconds();
        }

        geo::Pose3D robot_pose = world.object(robot.robot_id).pose;
        if (robot.base_ref_) // If there is a twist message in the queue
        {
            // Set robot velocity
            geometry_msgs::msg::Twist cmd = *robot.base_ref_;
            robot.base.applyTwistAndUpdate(cmd, dt);
            robot.base_ref_.reset();
        }
        else{ // apply previous one again
            robot.base.update(dt);
        }
        geometry_msgs::msg::Twist actual_twist = robot.base.getActualTwist();
        world.setVelocity(robot.robot_id, geo::Vector3(actual_twist.linear.x, actual_twist.linear.y, 0), actual_twist.angular.z);

        //check if object should start moving
        for(std::vector<MovingObject>::iterator ito = config.moving_objects.value().begin(); ito != config.moving_objects.value().end(); ++ito){

            MovingObject& obj = *ito;
            // check if it should start
            geo::Vector3 dist_obj_robot = world.object(robot.robot_id).pose.getOrigin() -  world.object(obj.id).pose.getOrigin();
            double safety_radius = sqrt( std::pow(obj.width/2,2) + std::pow(obj.length/2,2)) + 0.3;
            if(dist_obj_robot.length() < obj.trigger_radius && obj.is_moving == false && obj.finished_moving == false){
                obj.is_moving = true;
                geo::Vector3 unit_vel = (obj.final_pose.getOrigin() - obj.init_pose.getOrigin());
                unit_vel =world.object(obj.id).pose.R.transpose()*unit_vel / unit_vel.length();
                world.setVelocity(obj.id, unit_vel*obj.velocity,0.0);
            }

            // check if it should stop
            geo::Vector3 dist_obj_dest = world.object(obj.id).pose.getOrigin() -  obj.final_pose.getOrigin();
            if(dist_obj_dest.length() < 0.1 && obj.is_moving == true && obj.finished_moving == false && obj.repeat == false){
                obj.is_moving = false;
                obj.finished_moving = true;
                world.setVelocity(obj.id, geo::Vector3(0.0,0.0,0.0),0.0);
            }

            // reverse and repeat ... :o)
            if(dist_obj_dest.length() < 0.1 && obj.is_moving == true && obj.finished_moving == false && obj.repeat == true){
                obj.is_moving = true;
                obj.finished_moving = false;
                geo::Pose3D placeholder = obj.init_pose;
                obj.init_pose = obj.final_pose;
                obj.final_pose = placeholder;
                geo::Vector3 unit_vel = (obj.final_pose.getOrigin() - obj.init_pose.getOrigin());
                unit_vel =world.object(obj.id).pose.R.transpose()*unit_vel / unit_vel.length();
                world.setVelocity(obj.id, unit_vel*obj.velocity,0.0);
            }

            // Check if it should pause
            if(dist_obj_robot.length() < safety_radius && obj.is_moving == true && obj.is_paused == false){
                obj.is_paused = true;
                world.setVelocity(obj.id, geo::Vector3(0.0,0.0,0.0),0.0);
            }

            // Check if it should continue after being paused
            if(dist_obj_robot.length() > safety_radius && obj.is_paused == true){
                obj.is_paused = false;
                geo::Vector3 unit_vel = (obj.final_pose.getOrigin() - obj.init_pose.getOrigin());
                unit_vel =world.object(obj.id).pose.R.transpose()*unit_vel / unit_vel.length();
                world.setVelocity(obj.id, unit_vel*obj.velocity,0.0);
            }
        }

        // handle door requests
        if (robot.request_open_door_)
        {
            for(std::vector<Door>::iterator it = doors.begin(); it != doors.end(); ++it)
            {
                Door& door = *it;
                if (!door.closed)
                    continue;

                // Test if robot is in front of door. If not, don't open it
                geo::Pose3D rel_robot_pose = door.init_pose.inverse() * robot_pose;

                if (std::abs(rel_robot_pose.t.y) > 1 || std::abs(rel_robot_pose.t.x) > door.size / 2)
                    continue;

                world.setVelocity(door.id, door.open_vel, 0);
                door.closed = false;
            }
            robot.request_open_door_ = false;
        }

        if(config.uncertain_odom.value() && static_cast<int>(time.seconds()) % 6 == 0 ){
            robot.base.updateWheelUncertaintyFactors();
        }

        // Stop doors that have moved far enough
        for(std::vector<Door>::iterator it = doors.begin(); it != doors.end(); ++it)
        {
            Door& door = *it;
            if (!door.closed && (door.init_pose.t - world.object(door.id).pose.t).length2() > door.open_distance * door.open_distance)
                world.setVelocity(door.id, geo::Vector3(0, 0, 0), 0);
        }

        world.update(time.seconds());

        bool collision = false;
        // create output
        // Create laser data
        sensor_msgs::msg::LaserScan scan_msg;
        scan_msg.header.frame_id = "base_link";
        lrf.generateLaserData(world, robot, scan_msg);
        robot.pub_laser->publish(scan_msg);

        // Create bumper data 
        std_msgs::msg::Bool bump_msg_f; // front bumper message
        std_msgs::msg::Bool bump_msg_r; // rear bumper message
        bumper.generateBumperData(world,robot,bump_msg_f,bump_msg_r);
        robot.pub_bumperF->publish(bump_msg_f);
        robot.pub_bumperR->publish(bump_msg_r);

        // Detect collission based on bumper data 
        collision = collision || bump_msg_f.data || bump_msg_r.data;

        // Create odom data
        nav_msgs::msg::Odometry odom_msg = robot.base.getOdom();
        if(!config.uncertain_odom.value()){
            geo::convert(world.object(robot.robot_id).pose, odom_msg.pose.pose);
        }
        odom_msg.header.stamp = time;
        odom_msg.header.frame_id = "odomframe";

        robot.pub_odom->publish(odom_msg);
        // Write tf2 data
        geo::Pose3D pose = world.object(robot.robot_id).pose;
        robot.pubTransform(pose);
        
        // Visualize             
        if (visualize)
            visualization::visualize(world, robot, collision, config.show_full_map.value(), bbox, robot_radius);

        auto objects = visualization::create_rviz_objectmsg(world);
        marker_pub->publish(objects);

        if (collision)
            RCLCPP_WARN(node->get_logger(), "COLLISION!");

        r.sleep();
    }

    return 0;
}
