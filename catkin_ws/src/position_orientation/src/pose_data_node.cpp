#include <geometry_msgs/PoseStamped.h>
#include <lidar_data/LidarPose.h>
#include <ros/ros.h>
#include <tf/tf.h>

class PoseDataNode {
  public:
    ros::NodeHandle nh;
    ros::Publisher pose_pub;
    ros::Subscriber pose_sub;

    PoseDataNode() {
        pose_pub = nh.advertise<lidar_data::LidarPose>("lidar_data", 10);
        pose_sub = nh.subscribe("mavros/local_position/pose", 10, &PoseDataNode::poseCallback, this);
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
        // 从 PoseStamped 消息中获取位姿
        const auto &pose = msg->pose;

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

        pose_pub.publish(output);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "pose_stamped_node");
    PoseDataNode node;
    ros::spin();
    return 0;
}
