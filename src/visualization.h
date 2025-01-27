#ifndef PICO_SIMULATOR_VISUALIZATION_H_
#define PICO_SIMULATOR_VISUALIZATION_H_

#include <visualization_msgs/msg/marker_array.h>

#include "world.h"
#include "robot.h"

namespace visualization
{
    struct Bbox {
        double xmin, ymin, xmax, ymax;
    };

void visualize(const World& world, const Robot& robot, bool collision, bool show_full_map,Bbox bbox, double robot_radius);

visualization_msgs::msg::MarkerArray create_rviz_objectmsg(const World &world);
}
#endif
