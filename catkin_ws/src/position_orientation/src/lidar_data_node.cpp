#include <lidar_data/LidarPose.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/tf.h>

class LidarDataNode 
{
  public:
    ros::NodeHandle nh;
    ros::Publisher data_pub;
    ros::Subscriber odom_sub;

    LidarDataNode(){
        data_pub = nh.advertise<lidar_data::LidarPose>("lidar_data", 10);
        odom_sub = nh.subscribe("/Odometry", 10, &LidarDataNode::odomCallback, this);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
        // 从 Odometry 话题中获取位姿
        const auto &pose = msg->pose.pose;

        // 提取并转换四元数为欧拉角
        tf::Quaternion q(pose.orientation.x, pose.orientation.y,
                         pose.orientation.z, pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        if (roll < 0)
            roll += 2 * M_PI;
        if (pitch < 0)
            pitch += 2 * M_PI;
        if (yaw < 0)
            yaw += 2 * M_PI;

        lidar_data::LidarPose output;
        output.x = pose.position.x;
        output.y = pose.position.y;
        output.z = pose.position.z;
        output.roll = roll;
        output.pitch = pitch;
        output.yaw = yaw;

        data_pub.publish(output);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_data_node");
    LidarDataNode node;
    ros::spin();
    return 0;
}
