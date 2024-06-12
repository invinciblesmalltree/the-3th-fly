#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <ros_tools/LidarPose.h>
#include <ros_tools/target_class.hpp>
#include <std_msgs/Int32.h>
#include <vector>
#include <vision/box_data.h>

float vector2theta(float x, float y) {
    float angle = atan2(y, x);
    return angle < 0 ? angle += 2 * M_PI : angle;
}

mavros_msgs::State current_state;
ros_tools::LidarPose lidar_pose_data;
int region_data, barcode_data, offboard_order = 0;
vision::box_data box_data;

void state_cb(const mavros_msgs::State::ConstPtr &msg) { current_state = *msg; }

void lidar_cb(const ros_tools::LidarPose::ConstPtr &msg) {
    lidar_pose_data = *msg;
}

void region_cb(const std_msgs::Int32::ConstPtr &msg) {
    region_data = msg->data;
}

void box_cb(const vision::box_data::ConstPtr &msg) { box_data = *msg; }

void barcode_cb(const std_msgs::Int32::ConstPtr &msg) {
    barcode_data = msg->data;
}

void offboard_order_cb(const std_msgs::Int32::ConstPtr &msg) {
    offboard_order = msg->data;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub =
        nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
    ros::Subscriber lidar_data_sub =
        nh.subscribe<ros_tools::LidarPose>("/lidar_data", 10, lidar_cb);
    ros::Subscriber region_data_sub =
        nh.subscribe<std_msgs::Int32>("/region_data", 10, region_cb);
    ros::Subscriber box_data_sub =
        nh.subscribe<vision::box_data>("/yolov5/box_detect", 10, box_cb);
    ros::Subscriber barcode_data_sub =
        nh.subscribe<std_msgs::Int32>("/barcode_data", 10, barcode_cb);
    ros::Subscriber offboard_order_sub =
        nh.subscribe<std_msgs::Int32>("/offboard_order", 10, offboard_order_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(
        "/mavros/setpoint_position/local", 10);
    ros::Publisher velocity_pub = nh.advertise<geometry_msgs::TwistStamped>(
        "/mavros/setpoint_velocity/cmd_vel", 10);
    ros::Publisher led_pub = nh.advertise<std_msgs::Int32>("/led", 10);
    ros::Publisher servo_pub =
        nh.advertise<std_msgs::Int32>("/servo", 10);
    ros::Publisher screen_data_pub =
        nh.advertise<std_msgs::Int32>("/screen_data", 10);
    ros::ServiceClient arming_client =
        nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    ros::ServiceClient command_client =
        nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
    ros::ServiceClient set_mode_client =
        nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    ros::Rate rate(20.0);

    std::vector<target> targets = {
        target(0, 0, 1.8, 0),      target(0, 3.4, 1.8, 0),
        target(3.75, 3.4, 1.8, 0), target(3.75, 2.7, 1.8, 0), // 区域1
        target(3.75, 1.2, 1.8, 0),                            // 区域2
        target(3.75, 0.6, 1.8, 0),                            // 区域2
        target(3.75, 0, 1.8, 0),   target(3.05, 0, 1.8, 0),
        target(3.05, 2.7, 1.8, 0), target(2.35, 2.7, 1.8, 0), // 区域3
        target(2.2, 1.2, 1.8, 0),                             // 区域4
        target(2.2, 0, 1.8, 0),    target(1.5, 0, 1.8, 0),
        target(1.5, 2.7, 1.8, 0),  target(0.8, 2.7, 1.8, 0), // 区域5
        target(0.65, 1.2, 1.8, 0),                           // 区域6
        target(0.65, 0, 1.8, 0),   target(0, 0, 1.8, 0),
        target(0, 0, 0.5, 0)};

    bool is_region_center[] = {0, 0, 0, 1, 1, 1, 0, 0, 0, 1,
                               1, 0, 0, 0, 1, 1, 0, 0, 0};

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
    int mode = 0;
    bool region_vis[7]{}, first_box = true, has_thrown = false,
                          has_lighted = false;
    geometry_msgs::TwistStamped box_forward_vel;
    target box_center(0, 0, 1.8, 0), passed_point(0, 0, 1.8, -M_PI / 2),
        scan_point(0, 0, 1.8, -M_PI / 2);
    int box_id = -1, current_barcode;

    while (ros::ok() && !offboard_order) {
        ros::spinOnce();
        rate.sleep();
    }

    while (ros::ok()) {
        if (!current_state.armed &&
            (ros::Time::now() - last_request > ros::Duration(1.0))) {
            if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                ROS_INFO("Vehicle armed");
            }
            last_request = ros::Time::now();
        } else if (current_state.mode != "OFFBOARD" &&
                   (ros::Time::now() - last_request > ros::Duration(1.0))) {
            if (set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
                geometry_msgs::PoseStamped pose;
                pose.pose.position.x = 0;
                pose.pose.position.y = 0;
                pose.pose.position.z = 0.5;
                local_pos_pub.publish(pose);
            }
            last_request = ros::Time::now();
        }
        if (current_state.armed && current_state.mode == "OFFBOARD") {
            break;
        }
        ros::spinOnce();
        rate.sleep();
    }

    while (ros::ok()) {
        if (mode == 0) { // 正常巡线
            if (target_index >= targets.size()) {
                ROS_INFO("All targets reached");
                mavros_msgs::CommandLong command_srv;
                command_srv.request.broadcast = false;
                command_srv.request.command = 21;
                command_srv.request.confirmation = 0;
                command_srv.request.param4 = 0;
                if (command_client.call(command_srv) &&
                    command_srv.response.success) {
                    ROS_INFO("Land command sent successfully");
                }
                break;
            } else if (!targets[target_index].pos_check(lidar_pose_data)) {
                targets[target_index].fly_to_target(local_pos_pub);
            } else {
                ROS_INFO("Reached target %zu", target_index);
                if (is_region_center[target_index]) {
                    ROS_INFO("Region center %d reached", region_data);
                    last_request = ros::Time::now();
                    mode = 4;
                    ROS_INFO("Mode 4");
                } else
                    target_index++;
            }
        } else if (mode == 1) { // 飞往箱子中心
            box_forward_vel.twist.linear.z = 1.8 - lidar_pose_data.z;
            if (~box_data.class_id) {
                box_forward_vel.twist.linear.x = -box_data.y / 1000.0;
                box_forward_vel.twist.linear.y = -box_data.x / 1000.0;
                if (box_data.x * box_data.x + box_data.y * box_data.y < 500) {
                    ROS_INFO("Box %d arrived", box_data.class_id);
                    region_vis[region_data] = true;
                    box_center.x = lidar_pose_data.x;
                    box_center.y = lidar_pose_data.y;
                    box_id = box_data.class_id;
                    scan_point.x = passed_point.x = box_center.x;
                    scan_point.y = passed_point.y = box_center.y + 0.8;
                    scan_point.reached = passed_point.reached =
                        box_center.reached = false;
                    scan_point.z = box_id == 0 ? 0.75 : 0.48;
                    current_barcode = -1;
                    std_msgs::Int32 led_msg;
                    led_msg.data = has_lighted;
                    led_pub.publish(led_msg);
                    ROS_INFO(has_lighted ? "Green led lighted"
                                         : "Red led lighted");
                    if (!has_lighted)
                        has_lighted = true;
                    std_msgs::Int32 screen_data;
                    screen_data.data = region_data;
                    screen_data_pub.publish(screen_data);
                    mode = 2;
                    ROS_INFO("Mode 2");
                }
            }
            velocity_pub.publish(box_forward_vel);
        } else if (mode == 2) { // 扫码
            if (!passed_point.pos_check(lidar_pose_data)) {
                passed_point.fly_to_target(local_pos_pub);
                last_request = ros::Time::now();
            } else {
                scan_point.fly_to_target(local_pos_pub);
                if (~barcode_data) {
                    current_barcode = barcode_data;
                    ROS_INFO("Barcode: %d", current_barcode);
                    passed_point.reached = false;
                    mode = 3;
                    ROS_INFO("Mode 3");
                }
                if (ros::Time::now() - last_request > ros::Duration(8.0)) {
                    current_barcode = 0;
                    ROS_INFO("Cannot find barcode");
                    passed_point.reached = false;
                    mode = 3;
                    ROS_INFO("Mode 3");
                }
            }
        } else if (mode == 3) { // 返航到箱子中央
            if (!passed_point.pos_check(lidar_pose_data)) {
                passed_point.fly_to_target(local_pos_pub);
            } else {
                if (current_barcode & 1) {
                    if (!box_center.pos_check(lidar_pose_data)) {
                        box_center.fly_to_target(local_pos_pub);
                    } else {
                        if (!has_thrown) {
                            std_msgs::Int32 servo_msg;
                            servo_msg.data = 1;
                            servo_pub.publish(servo_msg);
                            ROS_INFO("Throwing");
                            last_request = ros::Time::now();
                            has_thrown = true;
                        } else if (ros::Time::now() - last_request >
                                   ros::Duration(1.0)) {
                            mode = 0;
                            ROS_INFO("Mode 0");
                        } else
                            box_center.fly_to_target(local_pos_pub);
                    }
                } else {
                    mode = 0;
                    ROS_INFO("Mode 0");
                }
            }
        } else if (mode == 4) { // 延迟并检测箱子
            targets[target_index].fly_to_target(local_pos_pub);
            if (!region_vis[region_data] && ~box_data.class_id) {
                ROS_INFO("Box detected, current region: %d", region_data);
                target_index++;
                mode = 1;
                ROS_INFO("Mode 1");
            }
            if (ros::Time::now() - last_request > ros::Duration(1.0)) {
                target_index++;
                mode = 0;
                ROS_INFO("Mode 0");
            }
        }

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}