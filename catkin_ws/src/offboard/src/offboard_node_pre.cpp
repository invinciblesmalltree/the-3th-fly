#include <cmath>
#include <cv_detect/BarMsg.h>
#include <cv_detect/LedMsg.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <lidar_data/LidarPose.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <vector>

class target
{
  public:
    float x, y, z, yaw;

    target(float x, float y, float z, float yaw):x(x),y(y),z(z),yaw(yaw) {}

    void fly_to_target(ros::Publisher &local_pos_pub)
    {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = z;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = sin(yaw / 2);
        pose.pose.orientation.w = cos(yaw / 2);

        local_pos_pub.publish(pose);
    }

    bool pos_check(lidar_data::LidarPose &lidar_pose_data)
    {
        float distance = sqrt(pow(lidar_pose_data.x - x, 2) +
                            pow(lidar_pose_data.y - y, 2) +
                            pow(lidar_pose_data.z - z, 2));
        return distance < 0.1;
    }
};

class region
{
    public:
        float center_x, center_y, length, width;
        bool flag = false; // 是否已经经过

        region(float center_x, float center_y, float length, float width):center_x(center_x),center_y(center_y),length(length),width(width) {}

        void fly_to_top(ros::Publisher &local_pos_pub)
        {
            target top(center_x, center_y, 1.8, 0.0);
            top.fly_to_target(local_pos_pub);
            if(!top.pos_check(lidar_pose_data))
            {
                top.fly_to_target(local_pos_pub);
            }
        }

        void fly_to_scan(ros::Publisher &local_pos_pub)
        {
            target scanPoint(center_x+0.5, center_y, 1.2, 0); // 起飞朝南，d435扫码朝北
            scanPoint.fly_to_target(local_pos_pub);
            if(!scanPoint.pos_check(lidar_pose_data))
            {
                scanPoint.fly_to_target(local_pos_pub);
            }
            else
            {
                if (barcode_data%2==1)
                {
                        has_scaned_firstBox = true;
                        fly_to_top(local_pos_pub);
                        /*
                        ** TODO: 投掷
                        */
                }
            }
        }

        void check_region()
        {
            float box_distance = 0;
            for(size_t i=0; i<regions.size(); i++)
            {
                if(box_distance <= sqrt(pow(lidar_pose_data.x - regions[i].x, 2) +
                                         pow(lidar_pose_data.y - regions[i].y, 2) +
                                         pow(lidar_pose_data.z - regions[i].z, 2)))
                { regions[i].flag=true; }
            }
        }
}

float vector2theta(float x, float y) {
    float angle = atan2(y, x);
    return angle < 0 ? angle += 2 * M_PI : angle;
}

mavros_msgs::State current_state;
lidar_data::LidarPose lidar_pose_data;
cv_detect::LedMsg box_data; // TODO: cv消息修改为box_data
cv_detect::BarMsg barcode_data;
std_msgs::Int32 supersonic_data;
geometry_msgs::TwistStamped vel_msg;

void state_cb(const mavros_msgs::State::ConstPtr &msg) { current_state = *msg; }
void lidar_cb(const lidar_data::LidarPose::ConstPtr &msg) { lidar_pose_data = *msg; }
void led_cb(const cv_detect::LedMsg::ConstPtr &msg) { blue_data = *msg; }
void barcode_cb(const cv_detect::BarMsg::ConstPtr &msg) { barcode_data = *msg; }
void supersonic_cb(const std_msgs::Int32::ConstPtr &msg) { supersonic_data = *msg; }

int main(int argc, char **argv) {
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber lidar_data_sub = nh.subscribe<lidar_data::LidarPose>("lidar_data", 10, lidar_cb);
    ros::Subscriber led_sub = nh.subscribe<cv_detect::LedMsg>("blue_msg", 10, led_cb);
    ros::Subscriber barcode_sub = nh.subscribe<cv_detect::BarMsg>("barcode_msg", 10, barcode_cb);
    ros::Subscriber supersonic_sub = nh.subscribe<std_msgs::Int32>("supersonic_data", 10, supersonic_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::Publisher velocity_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::Rate rate(20.0);

    std::vector<target> targets = {
        target(0, 0, 1.8, 0),      target(0, 3.4, 1.8, 0),
        target(3.75, 3.4, 1.8, 0), target(3.75, 0, 1.8, 0),
        target(3.05, 0, 1.8, 0),   target(3.05, 2.7, 1.8, 0),
        target(2.25, 2.7, 1.8, 0), target(2.25, 0, 1.8, 0),
        target(1.5, 0, 1.8, 0),    target(1.5, 2.7, 1.8, 0),
        target(0.65, 2.7, 1.8, 0), target(0.65, 0, 1.8, 0),
        target(0, 0, 1.8, 0),      target(0, 0, 0.5, 0)};

    std:vector<region>regions = {
        region(3.05, -1.15, 0.90, 1.10),
        region(3.05, -2.70, 0.80, 0.80),
        region(3.05, -4.00, 0.80, 0.80),
        region(1.55, -1.00, 0.90, 0.80),
        region(1.55, -2.55, 0.90, 0.80),
        region(1.25, -4.10, 1.50, 0.80)};

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
    bool has_scaned_firstBox = false;
    bool has_scaned_secondBox = false;

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
                    else if (box_data.value)
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
                        arming_client.call(arm_cmd); // 并不能disarm
                        ROS_INFO("Vehicle disarmed");
                        break;
                    }
                }
                case 2: // 找码代码
                {
                    ROS_INFO("Mode 2");
                     
                    if (!scan_point.reached) 
                    {
                        scan_point.fly_to_target(local_pos_pub);
                        float distance = sqrt(pow(lidar_pose_data.x - scan_point.x, 2) +
                                            pow(lidar_pose_data.y - scan_point.y, 2) +
                                            pow(lidar_pose_data.z - scan_point.z, 2));
                        if (distance < 0.1)
                            scan_point.reached = true;
                    } 
                    else
                    {
                        vel_msg.twist.linear.x = 0;
                    }
                    if (barcode_data.delta_x > 500)
                        vel_msg.twist.linear.y = 0;
                    else
                        vel_msg.twist.linear.y =
                                barcode_data.delta_x > 0 ? 0.1 : -0.1;
                        vel_msg.twist.linear.z = 0;
                        velocity_pub.publish(vel_msg);
                        ROS_INFO("Supersonic data: %dcm", supersonic_data.data);

                    if (abs(barcode_data.delta_x) < 50) 
                    {
                        if (supersonic_data.data > 100)
                            supersonic_data.data = 50;
                            pole_point.x = lidar_pose_data.x -
                                        supersonic_data.data / 100.0 - 0.225;
                            pole_point.y = lidar_pose_data.y;
                            pole_point.z = 1.25;
                            pole_point.yaw = lidar_pose_data.yaw;
                            scan_point.x = pole_point.x + 0.3;
                            scan_point.y = pole_point.y;
                            scan_point.z = 1.25;
                            scan_point.reached = false;
                            round_point.x = pole_point.x + 0.5;
                            round_point.y = pole_point.y;
                            round_point.z = 1.25;
                            round_point.yaw = -M_PI;
                            mode = 3;
                            ROS_INFO("Pole: (%f, %f)", pole_point.x, pole_point.y);
                            ROS_INFO("Mode 3");
                    }

                    static target debox_point(0, 0, 0, 0);
                    debox_point.x=lidar.pose_data.x;
                    debox_point.y=lidar.pose_data.y;
                    debox_point.z=lidar.pose_data.z;
                    ROS_INFO("Box detected!");

                    //TODO：判断箱子属于哪个region
                    ROS_INFO("Box: (%f, %f)", regions[n].center_x, regions[n].center_y);
                    box_point
                    if (!regions[n].fly_to_scan(local_pos_pub))
                    {
                        regions[n].fly_to_scan(local_pos_pub);
                    }
                    else{ mode = 4; }
                }
            case 3: // 扫码投掷
                {
                    ROS_INFO("Mode 3");
                    static target debox_point(0, 0, 0, 0);
                    debox_point.x=lidar.pose_data.x;
                    ROS_INFO("Box detected!");
                    //TODO：判断箱子属于哪个region
                    ROS_INFO("Box: (%f, %f)", regions[n].center_x, regions[n].center_y);
                    box_point
                    if (!regions[n].fly_to_scan(local_pos_pub))
                    {
                        regions[n].fly_to_scan(local_pos_pub);
                    }
                    else{ mode = 4; }
                }
            case 4: // 返回路线继续巡防
                {
                    ROS_INFO("Mode 4");
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