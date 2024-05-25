#include <cmath>
#include <cv_detect/BarMsg.h>
#include <cv_detect/BoxMsg.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros_tools/LidarPose.h>
#include <ros_tools/target_class.hpp>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <vector>

int current_region = -1;

class region
{
    public:
        float center_x, center_y, length, width;
        bool flag = false; // 是否已经过
        int scan_mode=1;

        region(float center_x, float center_y, float length, float width):center_x(center_x),center_y(center_y),length(length),width(width) {}

        void fly_to_scan(ros::Publisher &local_pos_pub, ros_tools::LidarPose &lidar_pose_data, geometry_msgs::TwistStamped vel_msg, int &mode, cv_detect::BarMsg barcode_data, ros::Rate &rate)
        {
            switch(scan_mode)
            {
                case 1: // 飞到scan点
                    static target scanPoint(center_x+0.5, center_y, 1.8, 0); // 起飞朝南，d435扫码朝北
                    if(!scanPoint.pos_check(lidar_pose_data))
                    {
                        scanPoint.fly_to_target(local_pos_pub);
                    }
                    else
                    {
                        scan_mode = 2;
                    }
                case 2: // 向下扫码, 判断奇偶并返回scan点
                    if(barcode_data.n != -1 && barcode_data.n%2 == 1)
                    {
                        scan_mode = 3; //奇数投掷
                    }
                    else if(barcode_data.n != -1 && barcode_data.n%2 == 0)
                    {
                        while(!scanPoint.pos_check(lidar_pose_data))
                        {
                            scanPoint.fly_to_target(local_pos_pub);
                            ros::spinOnce();
                            rate.sleep();
                        }
                        mode = 3; //偶数返航
                    }
                    else if(lidar_pose_data.z > 0.1)
                    {
                        vel_msg.twist.linear.z = -0.1;
                    }
                    else if(lidar_pose_data.z < 0.1)
                    {
                        while(!scanPoint.pos_check(lidar_pose_data))
                        {
                            scanPoint.fly_to_target(local_pos_pub);
                            ros::spinOnce();
                            rate.sleep();
                        }
                        mode = 3; //找不到码返航
                    }
                case 3: // 前往投掷点
                    static target top(center_x, center_y, 1.8, 0);
                    if(!top.pos_check(lidar_pose_data))
                    {
                        top.fly_to_target(local_pos_pub);
                    }
                    else
                    {
                        scan_mode = 5;
                    }
                case 4: // 投掷并返航
                    /*
                    ** TODO: 投掷
                    */
                   mode = 3;
            }
        }

};

bool check_region(ros_tools::LidarPose &lidar_pose_data, std::vector<region> &regions)
{
    float box_distance = 100; // 无穷远
    int box_num = -1;
    for(size_t i=0; i<regions.size(); i++)
    {
        if(box_distance > sqrt(pow(lidar_pose_data.x - regions[i].center_x, 2) +
                               pow(lidar_pose_data.y - regions[i].center_y, 2)))
        { box_num=i; current_region = i;}
    }
    if(regions[box_num].flag)
    {
        return false;
    }
    else
    {
        regions[box_num].flag = true;
        return true;
    }
}

float vector2theta(float x, float y) {
    float angle = atan2(y, x);
    return angle < 0 ? angle += 2 * M_PI : angle;
}

mavros_msgs::State current_state;
ros_tools::LidarPose lidar_pose_data;
cv_detect::BoxMsg box_data; // TODO: cv消息修改为box_data
cv_detect::BarMsg barcode_data;
std_msgs::Int32 supersonic_data;
geometry_msgs::TwistStamped vel_msg;

std::vector<target> targets;
std::vector<region> regions;

void state_cb(const mavros_msgs::State::ConstPtr &msg) { current_state = *msg; }
void lidar_cb(const ros_tools::LidarPose::ConstPtr &msg) { lidar_pose_data = *msg; }
void led_cb(const cv_detect::BoxMsg::ConstPtr &msg) { box_data = *msg; }
void barcode_cb(const cv_detect::BarMsg::ConstPtr &msg) { barcode_data = *msg; }
void supersonic_cb(const std_msgs::Int32::ConstPtr &msg) { supersonic_data = *msg; }

int main(int argc, char **argv) {
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber lidar_data_sub = nh.subscribe<ros_tools::LidarPose>("lidar_data", 10, lidar_cb);
    ros::Subscriber led_sub = nh.subscribe<cv_detect::BoxMsg>("box_msg", 10, led_cb);
    ros::Subscriber barcode_sub = nh.subscribe<cv_detect::BarMsg>("barcode_msg", 10, barcode_cb);
    ros::Subscriber supersonic_sub = nh.subscribe<std_msgs::Int32>("supersonic_data", 10, supersonic_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::Publisher velocity_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::Rate rate(20.0);

    std::vector<target> targets = {
        target(0.00, -0.05, 1.8, 0),    target(3.25, -0.05, 1.8, 0),
        target(3.25, -4.05, 1.8, 0),    target(0.05, -4.05, 1.8, 0),
        target(0.05, -3.25, 1.8, 0),    target(2.45, -3.25, 1.8, 0),
        target(2.45, -2.45, 1.8, 0),    target(0.05, -2.45, 1.8, 0),
        target(0.05, -1.65, 1.8, 0),    target(2.45, -1.65, 1.8, 0),
        target(2.45, -0.85, 1.8, 0),    target(0.05, -0.85, 1.8, 0),
        target(0.00, 0.00, 1.8, 0),     target(0.00, 0.00, 0.5, 0)};

    std::vector<region> regions = {
        region(2.70, -0.80, 0.90, 1.10),
        region(2.70, -2.35, 0.80, 0.80),
        region(2.70, -3.65, 0.80, 0.80),
        region(1.20, -0.65, 0.90, 0.80),
        region(1.20, -2.20, 0.90, 0.80),
        region(0.90, -3.75, 1.50, 0.80)};

    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    size_t target_index = 0;
    int mode = 1;
    target debox_point(0, 0, 0, 0);

    // 起飞前检查
    while (!current_state.armed || current_state.mode != "OFFBOARD")
    {
        if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if (arming_client.call(arm_cmd) && arm_cmd.response.success)
            {
                ROS_INFO("Vehicle armed");
            }
            last_request = ros::Time::now();
        }
        else
        {
            if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                {
                    ROS_INFO("Offboard enabled");
                    ROS_INFO("Mode: %s", current_state.mode.c_str());
                }
                last_request = ros::Time::now();
            }
        }
        ros::spinOnce();
        rate.sleep();
    }

    // 主任务循环
    while (ros::ok())
    {
        switch(mode)
        {
            case 1: // 定点巡防
                {
                    ROS_INFO("Mode 1");
                    if(!targets[target_index].pos_check(lidar_pose_data))
                    {
                        targets[target_index].fly_to_target(local_pos_pub);
                    }
                    else if (box_data.value && check_region(lidar_pose_data, regions))
                    {
                        mode = 2;
                    }
                    else if(targets[target_index].pos_check(lidar_pose_data))
                    {
                        ROS_INFO("Reached target %zu", target_index);
                        target_index++;
                    }
                    else if (target_index >= targets.size())
                    {
                        ROS_INFO("All targets reached");
                        arm_cmd.request.value = false;
                        arming_client.call(arm_cmd);
                        ROS_INFO("Vehicle disarmed");
                        break;
                    }
                }
            case 2: // 找码扫码投掷
                {
                    ROS_INFO("Mode 2");
                    ROS_INFO("Box detected!");
                    ROS_INFO("Box: (%f, %f)", 
                            regions[current_region].center_x, regions[current_region].center_y);
                    target debox_point(lidar_pose_data.x, lidar_pose_data.y, lidar_pose_data.z, 0);

                    regions[current_region].fly_to_scan(local_pos_pub, lidar_pose_data, vel_msg, mode, barcode_data, rate);
                }
            case 3: // 返回路线继续巡防
                {
                    ROS_INFO("Mode 3");
                    if (!debox_point.pos_check(lidar_pose_data))
                    {
                        debox_point.fly_to_target(local_pos_pub);
                    }
                    else{ mode = 1; }
                }
        }

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}