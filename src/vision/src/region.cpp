#include <ros/ros.h>
#include <ros_tools/LidarPose.h>
#include <std_msgs/Int32.h>

// 定义六个区域的左下角和右上角坐标
struct Region {
    float x1, y1, x2, y2;
};

Region regions[] = {{3.35, 2.25, 4.15, 3.15}, {3.35, 0.15, 4.15, 1.65},
                    {1.95, 2.25, 2.75, 3.15}, {1.65, 0.75, 2.75, 1.65},
                    {0.25, 2.25, 1.35, 3.15}, {0.25, 0.75, 1.05, 1.65}};

int getRegionNumber(float x, float y) {
    for (int i = 0; i < 6; ++i)
        if (regions[i].x1 <= x && x <= regions[i].x2 && regions[i].y1 <= y &&
            y <= regions[i].y2)
            return i + 1;
    return 0;
}

void lidarCallback(const ros_tools::LidarPose::ConstPtr &msg,
                   ros::Publisher &pub) {
    float x = msg->x;
    float y = msg->y;
    int region_number = getRegionNumber(x, y);

    std_msgs::Int32 region_msg;
    region_msg.data = region_number;
    pub.publish(region_msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "region");
    ros::NodeHandle nh;

    ros::Publisher region_pub =
        nh.advertise<std_msgs::Int32>("/region_data", 10);
    ros::Subscriber lidar_sub = nh.subscribe<ros_tools::LidarPose>(
        "/lidar_data", 10,
        boost::bind(lidarCallback, _1, boost::ref(region_pub)));

    ros::spin();

    return 0;
}