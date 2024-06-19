/**
 * Converts odometry to tf
 * Subscribes to /ugv/odom to get the odometry data as nav_msgs/Odometry objects
 * Converts nav_msgs/Odometry to tf
 * Publishes tf on /tf topic
 * 
 * Code recycled from the first project
 * 
 * @author anto, abdo, jie
*/

#include <ros/ros.h>
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>

class tf_sub_pub {
    private:
        ros::NodeHandle n;
        tf::TransformBroadcaster br;
        ros::Subscriber sub;

    public:
        tf_sub_pub() {
            sub = n.subscribe("/ugv/odom", 1000, &tf_sub_pub::callback, this);
        }

        void callback(const nav_msgs::Odometry::ConstPtr &data) {
            tf::Transform transform;
            transform.setOrigin(tf::Vector3(data->pose.pose.position.x, data->pose.pose.position.y, 0));
            tf::Quaternion q;

            tf::Pose pose; // Get the yaw from the odometry message: https://answers.ros.org/question/41233/how-to-understand-robot-orientation-from-quaternion-yaw-angle/
            tf::poseMsgToTF(data->pose.pose, pose);
            double yaw_angle = tf::getYaw(pose.getRotation());
            // ROS_INFO("Yaw: %lf", yaw_angle);

            q.setRPY(0, 0, yaw_angle);
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, data->header.stamp, "odom", "base_link"));
        }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "ugv_odom_to_tf");
    tf_sub_pub my_tf_sub_pub; 
    ros::spin();
    return 0;
}