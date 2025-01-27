#ifndef PICO_SIMULATOR_HEIGHTMAP_H_
#define PICO_SIMULATOR_HEIGHTMAP_H_

#include <geolib/datatypes.h>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/imgproc.hpp>

struct Door;

class MapLoader : public rclcpp::Node
{
public:
    MapLoader();

    void getMap(nav_msgs::msg::OccupancyGrid& mapRef);
    
    void getMapMetadata(nav_msgs::msg::MapMetaData& metadataRef);
    void getMapImage(cv::Mat& imageRef);
    bool isInitialized(){ return initialized; };

private:
    void load();
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map;
    bool initialized = false;
    nav_msgs::msg::OccupancyGrid map;
    cv::Mat mapImage;
};

geo::ShapePtr createHeightMapShape(const std::string& filename, std::vector<Door>& doors);
geo::ShapePtr createHeightMapShape(const cv::Mat& image_tmp, double resolution, std::vector<Door>& doors);

#endif
