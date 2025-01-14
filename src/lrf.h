#ifndef PICO_SIMULATOR_LRF_H_
#define PICO_SIMULATOR_LRF_H_

#include "robot.h"

#include <geolib/sensors/LaserRangeFinder.h>
#include <sensor_msgs/msg/laser_scan.hpp>

class World;

class LRF
{

public:

    LRF();

    ~LRF();

    void generateLaserData(const World& world, const Robot& robot, sensor_msgs::msg::LaserScan& scan_msg) const;

    void setAngleLimits(double a_min, double a_max);

    void setNumBeams(unsigned int n);

    void setRangeLimits(double r_min, double r_max);

    void setNoiseLevel(double noise_level);

private:

    geo::LaserRangeFinder lrf_;

    double noise_level_;

    sensor_msgs::msg::LaserScan msg_prototype_;

};

#endif
