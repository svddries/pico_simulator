#include "virtualbase.h"

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>

Virtualbase::Virtualbase(bool disable_speedcap, bool uncertain_odom) : disable_speedcap_(disable_speedcap), uncertain_odom_(uncertain_odom)
{
    std::random_device rd;
    gen = std::mt19937(rd());
    //std::normal_distribution<double> dis(0.0,0.003);
    dis = std::uniform_real_distribution<double>(-0.002,0.002);

    if (uncertain_odom_) {
        std::uniform_real_distribution<double> dis_pos_init = std::uniform_real_distribution<double>(-2.0,2.0);
        odometry_state.pose.pose.position.x = dis_pos_init(gen);
        odometry_state.pose.pose.position.y = dis_pos_init(gen);
        odometry_state.pose.pose.position.z = 0.0;
        std::uniform_real_distribution<double> dis_angle_init = std::uniform_real_distribution<double>(0.0, 2 * M_PI);
        tf2::Quaternion q;
        q.setRPY(0, 0, dis_angle_init(gen));
        odometry_state.pose.pose.orientation.x = q.x();
        odometry_state.pose.pose.orientation.y = q.y();
        odometry_state.pose.pose.orientation.z = q.z();
        odometry_state.pose.pose.orientation.w = q.w();
    }
    else
    {
        odometry_state.pose.pose.position.x = 0.0;
        odometry_state.pose.pose.position.y = 0.0;
        odometry_state.pose.pose.position.z = 0.0;
        odometry_state.pose.pose.orientation.x = 0.0;
        odometry_state.pose.pose.orientation.y = 0.0;
        odometry_state.pose.pose.orientation.z = 0.0;
        odometry_state.pose.pose.orientation.w = 1.0;
    }
    holonomic_operation_ = false;
    updateWheelUncertaintyFactors();
}

void Virtualbase::applyTwistAndUpdate(const geometry_msgs::msg::Twist& cmd, double dt)
{
    // apply speedcap
    geometry_msgs::msg::Twist twist;
    if(disable_speedcap_ == false){
        twist.linear.x  = sgn<double>(cmd.linear.x)  * std::min(std::abs(cmd.linear.x),0.5);
        twist.linear.y  = sgn<double>(cmd.linear.y)  * std::min(std::abs(cmd.linear.y),0.5);
        twist.angular.z = sgn<double>(cmd.angular.z) * std::min(std::abs(cmd.angular.z),1.2);

    }
    else{
        twist = cmd;
    }

    // Disable holonomic operation if necessary
    if (!holonomic_operation_)
    {
        twist.linear.y = 0;
    }

    // save twist to keep odometry updated
    reference_twist = twist;

    // apply twist to odometry state (what it thinks it drove)
    odometry_state = update_odometry(odometry_state,twist,dt);

    // Calculate wheel speeds (topview clockwise start 9oclock) that would have resulted in the twist
    const double lw = 0.1; // distance to wheel
    double v1 = twist.linear.x - lw*twist.angular.z;
    double v2 = -0.5*twist.linear.x - 0.5*sqrt(3.0)*twist.linear.y - lw*twist.angular.z;
    double v3 = -0.5*twist.linear.x + 0.5*sqrt(3.0)*twist.linear.y - lw*twist.angular.z;

    v1 = a1*v1; v2=a2*v2; v3=a3*v3;

    // Calculate actual twist from wrong wheelspeeds
    actual_twist.linear.x = (2.0/3.0)* v1 -(1/3.0)*v2 -(1/3.0)*v3;
    actual_twist.linear.y = -sqrt(3.0)/3 * v2 + sqrt(3.0)/3.0 * v3;
    actual_twist.angular.z = -v1 / (3.0*lw) -v2 / (3.0*lw) -v3 / (3.0*lw);
}

nav_msgs::msg::Odometry Virtualbase::update_odometry(const nav_msgs::msg::Odometry odom, const geometry_msgs::msg::Twist twist, double dt) const
{
    tf2::Transform delta;
    delta.setOrigin( tf2::Vector3(twist.linear.x*dt, twist.linear.y*dt, 0));

    tf2::Quaternion q;
    q.setRPY(0, 0, twist.angular.z*dt); 
    delta.setRotation(q);

    tf2::Transform Todom;
    Todom.setRotation(tf2::Quaternion(odom.pose.pose.orientation.x,
                                        odom.pose.pose.orientation.y,
                                        odom.pose.pose.orientation.z,
                                        odom.pose.pose.orientation.w));
    Todom.setOrigin(tf2::Vector3(odom.pose.pose.position.x,
                                odom.pose.pose.position.y,
                                0));

    tf2::Transform new_odom = Todom*delta;

    nav_msgs::msg::Odometry new_odom_msg;
    new_odom_msg.pose.pose.position.x = new_odom.getOrigin()[0];
    new_odom_msg.pose.pose.position.y = new_odom.getOrigin()[1];
    new_odom_msg.pose.pose.orientation.x = new_odom.getRotation().getX();
    new_odom_msg.pose.pose.orientation.y = new_odom.getRotation().getY();
    new_odom_msg.pose.pose.orientation.z = new_odom.getRotation().getZ();
    new_odom_msg.pose.pose.orientation.w = new_odom.getRotation().getW();

    return new_odom_msg;
}

