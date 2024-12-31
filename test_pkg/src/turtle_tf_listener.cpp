#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>

double vel_linear, vel_angular;

int main(int argc, char** argv) {
    ros::init(argc, argv, "turtle_tf_listener");
    ros::NodeHandle nh;

    // Retrieve parameters from the ROS parameter server
    
    ros::param::param<double>("vel_linear", vel_linear, 0.1);
    ros::param::param<double>("vel_angular", vel_angular, 0.1);
    // nh.param<double>("vel_linear", vel_linear, 0.1);
    // nh.param<double>("vel_angular", vel_angular, 0.1);
    ROS_INFO("vel_linear = %f", vel_linear);
    ROS_INFO("vel_angular = %f", vel_angular);

    tf::TransformListener listener;
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);

    ros::Rate rate(10.0);
    while (nh.ok()) {
        tf::StampedTransform transform;
        try {
            listener.lookupTransform("/turtle2", "/turtle1", ros::Time(0), transform);
        } catch (tf::TransformException& ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(0.1).sleep();
            continue;
        }

        geometry_msgs::Twist vel_msg;
        vel_msg.linear.x = vel_linear * sqrt(pow(transform.getOrigin().x(), 2) + pow(transform.getOrigin().y(), 2));
        vel_msg.angular.z = vel_angular * atan2(transform.getOrigin().y(), transform.getOrigin().x());
        cmd_vel_pub.publish(vel_msg);

        rate.sleep();
    }
    return 0;
}
