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
        region(float center_x, float center_y, float length, float width):center_x(center_x),center_y(center_y),length(length),width(width) {}

        void fly_to_top(ros::Publisher &local_pos_pub)
        {
            target top(center_x, center_y, 1.8, 0.0);
            top.fly_to_target(local_pos_pub); // TODO：此时无人机方向需要校准朝北
            while(!top.pos_check(lidar_pose_data))
            {
                top.fly_to_target(local_pos_pub);
            }
        }

        void fly_to_scan(ros::Publisher &local_pos_pub)
        {
            target scanPoint(center_x, center_y, 1.5, -M_PI); // 起飞朝南，扫码朝北
            scanPoint.fly_to_target(local_pos_pub);
            while(!scanPoint.pos_check(lidar_pose_data))
            {
                scanPoint.fly_to_target(local_pos_pub);
            }

            /*
            ** TODO：扫码
            */

           if (barcode_data%2==1)
           {
                fly_to_top(local_pos_pub)
                /*
                ** TODO: 投掷
                */
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
        target(0.00, 0.00, 1.80, 0.00),
        target(0.00, 0.00, 1.80, 0.00),
        target(0.00, 0.00, 1.80, 0.00),
        target(0.00, 0.00, 1.80, 0.00),
        target(0.00, 0.00, 1.80, 0.00),
        target(0.00, 0.00, 1.80, 0.00),
        target(0.00, 0.00, 1.80, 0.00),};

    std:vector<region>regions = {
        region(0.00, 0.00, 0.00, 0.00),
        region(0.00, 0.00, 0.00, 0.00),
        region(0.00, 0.00, 0.00, 0.00),
        region(0.00, 0.00, 0.00, 0.00),
        region(0.00, 0.00, 0.00, 0.00),
        region(0.00, 0.00, 0.00, 0.00),};

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
    int n = 0;
    bool has_passed_firstBox = false;
    bool has_passed_secondBox = false;

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
            case 1: // 正常巡线
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
            case 2: // 扫码投掷
                {
                    ROS_INFO("Mode 2");

                    ROS_INFO("Box detected!");
                    ROS_INFO("Box: (%f, %f)", regions[n].center_x, regions[n].center_y);
                    while (!regions[n].fly_to_scan(local_pos_pub))
                    {
                        regions[n].fly_to_scan(local_pos_pub);
                    }
                    blue_point.fly_to_target(local_pos_pub);
                    if (!has_passed_blue && (ros::Time::now() - last_request > ros::Duration(6.0))) { has_passed_blue = true; }
                    if (has_passed_blue)
                        if (!first_scan_point.pos_check(lidar_pose_data))
                        {
                            first_scan_point.fly_to_target(local_pos_pub);
                        }
                        else{ mode = 3; }
                }
            case 3: // 找杆代码
                {
                    ROS_INFO("Mode 3");
                    if (!scan_point.pos_check(lidar_pose_data))
                    {
                        scan_point.fly_to_target(local_pos_pub);
                    }
                    else //发布线速度寻杆
                    {
                        vel_msg.twist.linear.x = 0;
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

                            pole_point.x = lidar_pose_data.x - supersonic_data.data / 100.0 - 0.225;
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

                            mode = 4;
                            ROS_INFO("Pole: (%f, %f)", pole_point.x, pole_point.y);
                        }
                    }
                }
            case 4: // 扫码代码
                {
                    ROS_INFO("Mode 4");
                    if (!scan_point.pos_check(lidar_pose_data)) 
                    {
                        scan_point.fly_to_target(local_pos_pub);
                    }
                    else
                    {
                        if (barcode_data.n != -1)
                        {
                            n = barcode_data.n;
                            ROS_INFO("Barcode: %d", n);
                            targets.pop_back(); // 移除vector最后元素
                            targets.pop_back();
                            targets.pop_back();
                            targets.pop_back();
                            targets.push_back(target(n * 0.1, -2.0, 1.25, -M_PI)); // 向容器添加新元素
                            targets.push_back(target(n * 0.1, -2.0, 0.7, -M_PI));
                            targets.push_back(target(n * 0.1, -2.0, 0.1, -M_PI));

                            mode = 5;
                        }
                    }
                }
            case 5: // 绕杆代码
                {
                    ROS_INFO("Mode 5");
                    if (!round_point.pos_check(lidar_pose_data))
                    {
                        round_point.fly_to_target(local_pos_pub);
                    }
                    else
                    {
                        if (lidar_pose_data.yaw > 11 * M_PI / 6)
                        {
                            has_passed_half = true;
                            ROS_INFO("Half passed");
                        }
                        if (has_passed_half && lidar_pose_data.yaw < M_PI)
                        {
                            mode = 6;
                            ROS_INFO("Full passed");
                        }
                        float angle_to_target = vector2theta(pole_point.x - lidar_pose_data.x, pole_point.y - lidar_pose_data.y);
                        float angular_angle = angle_to_target - lidar_pose_data.yaw;
                        if (angular_angle > M_PI)
                            angular_angle -= 2 * M_PI;
                        if (angular_angle < -M_PI)
                            angular_angle += 2 * M_PI;
                        vel_msg.twist.angular.z = angular_angle * 2.5;
                        float distance = sqrt(pow(lidar_pose_data.x - pole_point.x, 2) +
                                            pow(lidar_pose_data.y - pole_point.y, 2));
                        float v_x = 0.2 * cos(lidar_pose_data.yaw + M_PI / 2),
                            v_y = 0.2 * sin(lidar_pose_data.yaw + M_PI / 2),
                            delta_dis = distance - 0.5;
                        v_x += delta_dis * 2 * cos(angle_to_target);
                        v_y += delta_dis * 2 * sin(angle_to_target);
                        vel_msg.twist.linear.x = v_x;
                        vel_msg.twist.linear.y = v_y;
                        vel_msg.twist.linear.z = 1.25 - lidar_pose_data.z;
                        velocity_pub.publish(vel_msg);
                    }
                }
            case 6: // 返回蓝色物体继续巡线
                {
                    ROS_INFO("Mode 6");
                    if (!blue_point.pos_check(lidar_pose_data))
                    {
                        blue_point.fly_to_target(local_pos_pub);
                    }
                    else{ mode = 1; }
                }
        }

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}