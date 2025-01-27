#include "visualization.h"
#include "world.h"

#include <geolib/Shape.h>

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

#include <vector>

double resolution = 0.01;
cv::Point2d canvas_center;


namespace visualization
{


cv::Point2d worldToCanvas(const geo::Vector3& p)
{
    return cv::Point2d(-p.y / resolution, -p.x / resolution) + canvas_center;
}

// ----------------------------------------------------------------------------------------------------

void visualize(const World& world, const Robot& robot, bool collision = false, bool show_full_map = false, Bbox center_frame = {-1e5, -1e5, 1e5, 1e5}, double robot_radius = 0.17)
{
    int dim = 500;
    if(show_full_map){
        dim = 1000;
    }

    cv::Mat canvas(dim, dim, CV_8UC3, cv::Scalar(100, 100, 100));
    canvas_center = cv::Point2d(canvas.rows / 2, canvas.cols / 2);

    // Determine camera pose
    geo::Pose3D frame_center_pose = geo::Pose3D::identity();
    if (!show_full_map){
        frame_center_pose = world.object(robot.robot_id).pose;
    }

    Bbox midpoint_frame; //maximum range for the midpoint of the view.
    midpoint_frame.xmin = center_frame.xmin + dim*resolution/2 -0.5;
    midpoint_frame.xmax = center_frame.xmax - dim*resolution/2 +0.5;
    midpoint_frame.ymin = center_frame.ymin + dim*resolution/2 -0.5;
    midpoint_frame.ymax = center_frame.ymax - dim*resolution/2 +0.5;


    Object robot_object = world.object(robot.robot_id);
    // check translation of world within bbox for sliding camera
    double xview, yview;
    xview = robot_object.pose.getOrigin().getX();
    if(robot_object.pose.getOrigin().getX() > midpoint_frame.xmax )
        xview = midpoint_frame.xmax;
    if(robot_object.pose.getOrigin().getX() < midpoint_frame.xmin )
        xview = midpoint_frame.xmin;
    yview = robot_object.pose.getOrigin().getY();
    if(robot_object.pose.getOrigin().getY() > midpoint_frame.ymax )
        yview = midpoint_frame.ymax;
    if(robot_object.pose.getOrigin().getY() < midpoint_frame.ymin)
        yview = midpoint_frame.ymin;

    // If there is not enough mapsize for a midpointbox, fix views to center
    if(midpoint_frame.xmax < midpoint_frame.xmin){
        xview = midpoint_frame.xmax + midpoint_frame.xmin /2;
    }
    if(midpoint_frame.ymax < midpoint_frame.ymin){
        yview = midpoint_frame.ymax + midpoint_frame.ymin /2;
    }

    // Draw robot
    cv::Scalar robot_color(0, 0, 255);

    // scaling of the head and LRF (which should be drawn inside the circumference of the robot)
    const double scaling = robot_radius/0.21;

    geo::Vector3 robot_center_point = geo::Vector3( 0.0, 0.0, 0);
    geo::Vector3 lrf_point = scaling*geo::Vector3( 0.15, 0.0, 0);
    std::vector<geo::Vector3> robot_head_points;
    std::vector<geo::Vector3> eye_points;
    std::vector<geo::Vector3> eye_pupil_points;

    robot_head_points.push_back(scaling*geo::Vector3( 0.08, -0.16, 0));
    robot_head_points.push_back(scaling*geo::Vector3( 0.08,  0.16, 0));
    robot_head_points.push_back(scaling*geo::Vector3(-0.08,  0.16, 0));
    robot_head_points.push_back(scaling*geo::Vector3(-0.08, -0.16, 0));
    eye_points.push_back(scaling*geo::Vector3( 0.0,  0.07, 0));
    eye_points.push_back(scaling*geo::Vector3( 0.0, -0.07, 0));
    eye_pupil_points.push_back(scaling*geo::Vector3( 0.025,  0.07, 0));
    eye_pupil_points.push_back(scaling*geo::Vector3( 0.025, -0.07, 0));

    if(show_full_map == true) {
        robot_center_point = (robot_object.pose * robot_center_point) + geo::Vector3(-xview, -yview, 0);
        lrf_point = (robot_object.pose * lrf_point) + geo::Vector3(-xview, -yview, 0);
        for (unsigned int i = 0; i < robot_head_points.size(); ++i) {
            robot_head_points[i] = (robot_object.pose * robot_head_points[i]) + geo::Vector3(-xview, -yview, 0);
        }
        for (unsigned int i = 0; i < eye_points.size(); ++i) {
            eye_points[i] = (robot_object.pose * eye_points[i]) + geo::Vector3(-xview, -yview, 0);
        }
        for (unsigned int i = 0; i < eye_pupil_points.size(); ++i) {
            eye_pupil_points[i] = (robot_object.pose * eye_pupil_points[i]) + geo::Vector3(-xview, -yview, 0);
        }
    }
    else
    {
        robot_center_point = (frame_center_pose.inverse() * robot_object.pose * robot_center_point);
        lrf_point = (frame_center_pose.inverse() * robot_object.pose * lrf_point);
        for (unsigned int i = 0; i < robot_head_points.size(); ++i) {
            robot_head_points[i] = (frame_center_pose.inverse() * robot_object.pose * robot_head_points[i]);
        }
        for (unsigned int i = 0; i < eye_points.size(); ++i) {
            eye_points[i] = (frame_center_pose.inverse() * robot_object.pose * eye_points[i]);
        }
        for (unsigned int i = 0; i < eye_pupil_points.size(); ++i) {
            eye_pupil_points[i] = (frame_center_pose.inverse() * robot_object.pose * eye_pupil_points[i]);
        }
    }

    cv::Point2d p_lrf = worldToCanvas(lrf_point);
    cv::circle(canvas, p_lrf, scaling*2, robot_color, 2);
    cv::Point2d p_robot_center = worldToCanvas(robot_center_point);
    cv::circle(canvas, p_robot_center, robot_radius/resolution, robot_color, 2);

    for(unsigned int i = 0; i < robot_head_points.size(); ++i)
    {
        unsigned int j = (i + 1) % robot_head_points.size();
        cv::Point2d p1 = worldToCanvas(robot_head_points[i]);
        cv::Point2d p2 = worldToCanvas(robot_head_points[j]);
        cv::line(canvas, p1, p2, robot_color, 2);
    }
    for(unsigned int i = 0; i < eye_points.size(); ++i)
    {
        cv::Point2d p_eye = worldToCanvas(eye_points[i]);
        cv::circle(canvas, p_eye, scaling*4, robot_color, 2);
    }
    for(unsigned int i = 0; i < eye_pupil_points.size(); ++i)
    {
        cv::Point2d p_eye_pupil = worldToCanvas(eye_pupil_points[i]);
        cv::circle(canvas, p_eye_pupil, scaling*1.5, robot_color, 2);
    }
    

    for(std::vector<Object>::const_iterator it = world.objects().begin(); it != world.objects().end(); ++it)
    {
        const Object& obj = *it;
        if (!obj.shape)
            continue;
        if (obj.type == robottype)
            continue;

        const std::vector<geo::Vector3>& vertices = obj.shape->getMesh().getPoints();
        const std::vector<geo::TriangleI>& triangles = obj.shape->getMesh().getTriangleIs();

        cv::Scalar line_color(obj.color.x * 255, obj.color.y * 255, obj.color.z * 255);

        Object robot_object = world.object(robot.robot_id);
        geo::Transform t;
        if(show_full_map== false){
            t = robot_object.pose.inverse() * obj.pose;
        } else{
            geo::Transform viewbox(-xview, -yview,0,0,0,0);
            t = viewbox*obj.pose;
        }

        for(std::vector<geo::TriangleI>::const_iterator it2 = triangles.begin(); it2 != triangles.end(); ++it2)
        {
            const geo::TriangleI& triangle = *it2;

            cv::Point2d p1_2d = worldToCanvas(t * vertices[triangle.i1_]);
            cv::Point2d p2_2d = worldToCanvas(t * vertices[triangle.i2_]);
            cv::Point2d p3_2d = worldToCanvas(t * vertices[triangle.i3_]);

            cv::line(canvas, p1_2d, p2_2d, line_color, 2);
            cv::line(canvas, p2_2d, p3_2d, line_color, 2);
            cv::line(canvas, p1_2d, p3_2d, line_color, 2);
        }
    }

    if(collision){
        cv::putText(canvas,"COLLISION!",cv::Point(20,20),cv::FONT_HERSHEY_COMPLEX,1,cv::Scalar(0,0,255));
    }

    cv::imshow("simulator", canvas);
    cv::waitKey(3);
}

visualization_msgs::msg::MarkerArray create_rviz_objectmsg(const World &world)
{
    visualization_msgs::MarkerArray objects;
    visualization_msgs::Marker object;

    object.header.frame_id = "map";
    object.header.stamp = ros::Time::now();
    object.action = visualization_msgs::Marker::MODIFY;
    object.pose.position.x = 0.0;

    object.pose.position.y = 0.0;
                             
    object.pose.position.z = 0.01;
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    object.pose.orientation.x = q.x();
    object.pose.orientation.y = q.y();
    object.pose.orientation.z = q.z();
    object.pose.orientation.w = q.w();
    object.scale.y = 1.0;
    object.scale.z = 1.0;
    object.color.a = 1.0;
    object.id = 0;

    for(std::vector<Object>::const_iterator it = world.objects().begin(); it != world.objects().end(); ++it)
    {
        const Object& obj = *it;
        if (!obj.shape)
            continue;
        if (obj.type == robottype)
            continue;
        if (obj.type == walltype)
            continue;

        if (obj.type == doortype)
        {
            object.type = visualization_msgs::Marker::TRIANGLE_LIST;
            object.scale.x = 1.0;
            object.color.r = 0.0;
            object.color.b = 0.0;
            object.color.g = 1.0;
            object.ns = "doortype";
        }
        else if (obj.type == movingObjecttype)
        {
            object.type = visualization_msgs::Marker::LINE_STRIP;
            object.scale.x = 0.05;
            object.color.r = 1.00;
            object.color.b = 0.00;
            object.color.g = 0.00;
            object.ns = "movingObjecttype";
        }
        else
        {
            continue;
        }

        object.points.clear();

        const std::vector<geo::Vector3>& vertices = obj.shape->getMesh().getPoints();
        const std::vector<geo::TriangleI>& triangles = obj.shape->getMesh().getTriangleIs();

        for(std::vector<geo::TriangleI>::const_iterator it2 = triangles.begin(); it2 != triangles.end(); ++it2)
        {
            const geo::TriangleI& triangle = *it2;
            geo::Vec2d p1vec = (obj.pose * vertices[triangle.i1_]).projectTo2d();
            geo::Vec2d p2vec = (obj.pose * vertices[triangle.i2_]).projectTo2d();
            geo::Vec2d p3vec = (obj.pose * vertices[triangle.i3_]).projectTo2d();

            geometry_msgs::msg::Point p1;
            p1.x = p1vec.x; p1.y = p1vec.y; p1.z = 0;
            geometry_msgs::msg::Point p2;
            p2.x = p2vec.x; p2.y = p2vec.y; p2.z = 0;
            geometry_msgs::msg::Point p3;
            p3.x = p3vec.x; p3.y = p3vec.y; p3.z = 0;

            object.points.push_back(p1);
            object.points.push_back(p2);
            object.points.push_back(p3);
        }

        objects.markers.push_back(object);
        object.id++;
    }
    return objects;
    }
}


