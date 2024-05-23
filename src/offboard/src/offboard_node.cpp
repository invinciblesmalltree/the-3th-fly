#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <ros_tools/LidarPose.h>
#include <ros_tools/target_class.hpp>
#include <vector>

float vector2theta(float x, float y) {
    float angle = atan2(y, x);
    return angle < 0 ? angle += 2 * M_PI : angle;
}

mavros_msgs::State current_state;
ros_tools::LidarPose lidar_pose_data;

void state_cb(const mavros_msgs::State::ConstPtr &msg) { current_state = *msg; }

void lidar_cb(const ros_tools::LidarPose::ConstPtr &msg) {
    lidar_pose_data = *msg;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub =
        nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber lidar_data_sub =
        nh.subscribe<ros_tools::LidarPose>("lidar_data", 10, lidar_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(
        "mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client =
        nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient command_client =
        nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
    ros::ServiceClient set_mode_client =
        nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::Rate rate(20.0);

    std::vector<target> targets = {
        target(0, 0, 1.8, 0),      target(0, 3.4, 1.8, 0),
        target(3.75, 3.4, 1.8, 0), target(3.75, 0, 1.8, 0),
        target(3.05, 0, 1.8, 0),   target(3.05, 2.7, 1.8, 0),
        target(2.25, 2.7, 1.8, 0), target(2.25, 0, 1.8, 0),
        target(1.5, 0, 1.8, 0),    target(1.5, 2.7, 1.8, 0),
        target(0.65, 2.7, 1.8, 0), target(0.65, 0, 1.8, 0),
        target(0, 0, 1.8, 0),      target(0, 0, 0.5, 0)};

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
    int n = 0;

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
        if (mode == 0) {
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
                target_index++;
            }
        }

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}