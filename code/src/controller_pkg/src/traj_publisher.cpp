#pragma region INCLUDES

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/String.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include "fla_utils/param_utils.h"

#include <move_base_msgs/MoveBaseActionGoal.h>
#include <move_base_msgs/MoveBaseAction.h>
//#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalStatus.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <state_machine/State.h>
#include <nav_msgs/Path.h>
#include <sstream>
#include <iostream>
#include <math.h>
#include <string>

#define STATIC_POSE 0
#define PI M_PI
#define TFOUTPUT 1

#pragma endregion

#pragma region VARIABLES

actionlib_msgs::GoalStatusArray move_base_status;
geometry_msgs::Vector3 start;
geometry_msgs::Vector3 next_pos;
trajectory_msgs::MultiDOFJointTrajectoryPoint static_msg;
trajectory_msgs::MultiDOFJointTrajectoryPoint explore_msg;

// parameters
std::string name;
float flight_height, start_x, start_y, start_z, start_turn;
uint state;

#pragma endregion

#pragma region CALLBACKS

void cmd_vel_callback(geometry_msgs::Twist const& cmd_vel)
{
    explore_msg.velocities[0] = cmd_vel;
}

void lp_goal_callback(nav_msgs::Path const& path_msg)
{
        int idx = trunc(path_msg.poses.size() * 0.8);

        next_pos.x = path_msg.poses[idx].pose.position.x;
        next_pos.y = path_msg.poses[idx].pose.position.y;
        next_pos.z = flight_height;

        explore_msg.transforms[0].translation = next_pos;
        explore_msg.transforms[0].rotation.x = path_msg.poses[idx].pose.orientation.x;
        explore_msg.transforms[0].rotation.y = path_msg.poses[idx].pose.orientation.y;
        explore_msg.transforms[0].rotation.z = path_msg.poses[idx].pose.orientation.z;
        explore_msg.transforms[0].rotation.w = path_msg.poses[idx].pose.orientation.w;
}

void move_base_status_callback(actionlib_msgs::GoalStatusArray const& status_msg)
{
    move_base_status = status_msg;
}

void state_callback(state_machine::State const& state_msg)
{
    state = state_msg.state;
}

void init_messages()
{
    start.x = start_x;
    start.y = start_y;
    start.z = start_z;
    
    static_msg.transforms.resize(1);
    static_msg.transforms[0].translation = geometry_msgs::Vector3();
    tf2::Quaternion rot;
    rot.setRPY(0, 0, start_turn);
    static_msg.transforms[0].rotation = tf2::toMsg(rot);
    static_msg.velocities.resize(1);
    static_msg.velocities[0] = geometry_msgs::Twist();
    static_msg.accelerations.resize(1);
    static_msg.accelerations[0] = geometry_msgs::Twist();

    explore_msg.transforms.resize(1);
    explore_msg.velocities.resize(1);
    explore_msg.accelerations.resize(1);
}

#pragma endregion

int main(int argc, char **argv)
{
    ros::init(argc, argv, "traj_publisher");
    state = 0;

    ros::NodeHandle n;
    fla_utils::SafeGetParam(n, "traj_publisher/av_desired", name);
    fla_utils::SafeGetParam(n, "traj_publisher/flight_height", flight_height);
    fla_utils::SafeGetParam(n, "traj_publisher/start_x", start_x);
    fla_utils::SafeGetParam(n, "traj_publisher/start_y", start_y);
    fla_utils::SafeGetParam(n, "traj_publisher/start_z", start_z);
    fla_utils::SafeGetParam(n, "traj_publisher/start_turn", start_turn);

    ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel", 1, cmd_vel_callback);
    ros::Subscriber lp_goal_sub = n.subscribe("move_base/TrajectoryPlannerROS/local_plan/", 1, lp_goal_callback);
    ros::Subscriber move_base_status_sub = n.subscribe("move_base/status", 1, move_base_status_callback);
    ros::Subscriber state_machine_sub = n.subscribe("/state", 1, state_callback);
    
    ros::Publisher desired_state_pub = n.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("desired_state", 1);

    ros::Rate loop_rate(500);
    tf::TransformBroadcaster br;

    init_messages();

    while (ros::ok())
    {
        switch (state)
        {
            case 0: // init -> wait at current position
                static_msg.transforms[0].translation = start;

                desired_state_pub.publish(static_msg); 
                break;

            case 1: // takeoff -> fly to start position
                static_msg.transforms[0].translation = start;

                desired_state_pub.publish(static_msg); // start position
                break;

            case 2: // explore -> fly to next goal
                desired_state_pub.publish(explore_msg);
                break;

            case 3: // landing -> lower z position
                explore_msg.transforms[0].translation.z = 0;

                desired_state_pub.publish(explore_msg); // current position on ground
                break;

            default: // exit
                return 0;
        }

        tf::Transform desired_pose(tf::Transform::getIdentity());

        br.sendTransform(tf::StampedTransform(desired_pose, ros::Time::now(),
                                              "world", name));

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
