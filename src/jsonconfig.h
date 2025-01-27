//
// Created by bob on 30-3-20.
//

#ifndef PROJECT_JSONCONFIG_H
#define PROJECT_JSONCONFIG_H

#include "moving_object.h"
#include "json.hpp"
#include <fstream>
#include <boost/optional.hpp>

#include <rclcpp/rclcpp.hpp> // Replace ros/console.h with rclcpp/rclcpp.hpp

#include <vector>

class Config{

public:
    // Read config options from supplied json file
    Config(std::string filename){

        // Read the json object
        std::ifstream i(filename);
        nlohmann::json doc;
        try {
            doc = nlohmann::json::parse(i);
        }
        catch (const std::exception &e) {
            throw std::invalid_argument("[Error] json file not found or not a json file");
        }

//        // read mapfile
//        assert(doc.find("map_file") != doc.end());
//        std::string s = doc.at("map_file");
//        mapfile = s;

        // Read odometry settings
        if(doc.find("uncertain_odom") != doc.end()){
            uncertain_odom = doc.at("uncertain_odom");
        }
        else{
            uncertain_odom = false;
        }

        // Read full map settings
        if(doc.find("show_full_map") != doc.end()){
            show_full_map = doc.at("show_full_map");
        }
        else{
            show_full_map = false;
        }

        // Read internal pose settings
        if(doc.find("provide_internal_pose") != doc.end()){
            provide_internal_pose = doc.at("provide_internal_pose");
        }
        else{
            provide_internal_pose = true;
        }

        // read moving objects
        std::vector<MovingObject> ms;
        if(doc.find("objects") != doc.end())
        {
            for (const auto& object : doc.at("objects") )
            {
                assert(object.is_object());
                assert(object.find("length") != object.end());  //
                assert(object.find("width") != object.end());  //
                assert(object.find("trigger_radius") != object.end());  //
                assert(object.find("init_pose") != object.end());  //
                assert(object.find("final_pose") != object.end());  //
                assert(object.find("velocity") != object.end());  //
                assert(object.find("repeat") != object.end());  //

                MovingObject m;

                m.length = object.at("length");
                m.width = object.at("width");
                m.trigger_radius = object.at("trigger_radius");
                m.velocity = object.at("velocity");
                m.init_pose.setOrigin(geo::Vector3(object.at("init_pose")[0], object.at("init_pose")[1],0));
                m.init_pose.setRPY(0,0,object.at("init_pose")[2]);
                m.final_pose.setOrigin(geo::Vector3(object.at("final_pose")[0], object.at("final_pose")[1],0));
                m.final_pose.setRPY(0,0,object.at("final_pose")[2]);
                m.repeat = object.at("repeat");

                m.is_moving = false;
                m.is_paused = false;
                m.finished_moving = false;

                ms.push_back(m);

            }
        }
        moving_objects = ms;

        //Check if speedcap disabled is in json
        if(doc.find("disable_speedcap") != doc.end()){
            disable_speedcap = doc.at("disable_speedcap");
        }
        else{
            disable_speedcap = false;
        }

        //Check if pyro is used in json
        if(doc.find("use_pyro") != doc.end()){
            use_pyro = doc.at("use_pyro");
        }
        else{
            use_pyro = true;
        }
        
        if(doc.find("initial_pose") != doc.end()){
            std::vector<double> spawn_location = doc.at("initial_pose");
            assert(spawn_location.size()==3);
            spawn = geo::Pose3D(spawn_location[0],spawn_location[1],0,0,0,spawn_location[2]); // config value is [x,y,theta] = [x,y,0,0,0,yaw]
            spawn_provided = true;
        }
        else{
            spawn_provided = false;
        }

    }

    void print(){
        RCLCPP_INFO(rclcpp::get_logger("logger"), "Simulator configuration settings:");
        RCLCPP_INFO(rclcpp::get_logger("logger"), "Uncertain odom: %s", uncertain_odom.value() ? "true" : "false");
        RCLCPP_INFO(rclcpp::get_logger("logger"), "Show full map: %s", show_full_map.value() ? "true" : "false");
        RCLCPP_INFO(rclcpp::get_logger("logger"), "Provide internal pose: %s", provide_internal_pose.value() ? "true" : "false");
        RCLCPP_INFO(rclcpp::get_logger("logger"), "Disable speedcap: %s", disable_speedcap.value() ? "true" : "false");
        RCLCPP_INFO(rclcpp::get_logger("logger"), "Use pyro: %s", use_pyro.value() ? "true" : "false");
        if (spawn_provided.value())
        {
            RCLCPP_INFO(rclcpp::get_logger("logger"), "Spawn Location: %s", spawn.value().getOrigin().toString().c_str());
            RCLCPP_INFO(rclcpp::get_logger("logger"), "Spawn Rotation: %s", spawn.value().getBasis().toString().c_str());
        }
        else
            RCLCPP_INFO(rclcpp::get_logger("logger"), "No spawn Location provided, will spawn robot in the middle of the map.");
        RCLCPP_INFO(rclcpp::get_logger("logger"), "Imported %zu moving objects", moving_objects.value().size());
    }

public:
    boost::optional<bool> show_full_map;
    boost::optional<bool> uncertain_odom;
    boost::optional<bool> provide_internal_pose;
    boost::optional<std::vector<MovingObject>> moving_objects;
    boost::optional<bool> disable_speedcap;
    boost::optional<bool> use_pyro;
    boost::optional<geo::Pose3D> spawn;
    boost::optional<bool> spawn_provided;
};

#endif //PROJECT_JSONCONFIG_H
