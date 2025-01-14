#include "lrf.h"
#include "world.h"

#include <geolib/Shape.h>
#include <rclcpp/rclcpp.hpp>

// ----------------------------------------------------------------------------------------------------

float randomUniform(float min, float max)
{
    return (rand() / static_cast<float>(RAND_MAX)) * (max - min) + min;
}

// ----------------------------------------------------------------------------------------------------

LRF::LRF()
{
    noise_level_ = 0.01;
}

// ----------------------------------------------------------------------------------------------------

LRF::~LRF()
{
}

// ----------------------------------------------------------------------------------------------------

void LRF::setAngleLimits(double a_min, double a_max)
{
    lrf_.setAngleLimits(a_min, a_max);

}

// ----------------------------------------------------------------------------------------------------

void LRF::setNumBeams(unsigned int n)
{
    lrf_.setNumBeams(n);
}

// ----------------------------------------------------------------------------------------------------

void LRF::setNoiseLevel(double noise_level)
{
    noise_level_ = noise_level;
}

// ----------------------------------------------------------------------------------------------------

void LRF::setRangeLimits(double r_min, double r_max)
{
    lrf_.setRangeLimits(r_min, r_max);
}

// ----------------------------------------------------------------------------------------------------

void LRF::generateLaserData(const World& world, const Robot& robot, sensor_msgs::msg::LaserScan& scan_msg) const
{
    const Object& robot_obj = world.object(robot.robot_id);
    geo::Pose3D laser_pose = robot_obj.pose * robot.laser_pose;
    geo::Pose3D laser_pose_inv = laser_pose.inverse();

    std::vector<double> ranges(lrf_.getNumBeams(), 10.0); // max range of lrf = 10.0 m
    for(std::vector<Object>::const_iterator it = world.objects().begin(); it != world.objects().end(); ++it)
    {
        const Object& obj = *it;
        if (&obj == &robot_obj) // ignore robot itself
            continue;

        if (!obj.shape)
            continue;

        // Set render options
        geo::LaserRangeFinder::RenderOptions opt;
        opt.setMesh(obj.shape->getMesh(), laser_pose_inv * obj.pose);

        geo::LaserRangeFinder::RenderResult res(ranges);
        lrf_.render(opt, res);
    }

    // Create message
    scan_msg.angle_min = lrf_.getAngleMin();
    scan_msg.angle_max = lrf_.getAngleMax();
    scan_msg.range_min = lrf_.getRangeMin();
    scan_msg.range_max = lrf_.getRangeMax();

    // Make sure ranges in scan message is correct size
    scan_msg.ranges.resize(lrf_.getNumBeams());

    scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / (scan_msg.ranges.size() - 1);

    // Copy ranges to scan message and add noise
    for(unsigned int i = 0; i < ranges.size(); ++i)
    {
        double r = ranges[i];
        scan_msg.ranges[i] = r + randomUniform(-noise_level_, noise_level_);
    }

    // Stamp with current ROS 2 time
    scan_msg.header.stamp = rclcpp::Clock().now();
}
