#ifndef HITBOT_ROS_CONTROL
#define HITBOT_ROS_CONTROL

#include "hitbot_interface.h"
#include <thread> // For std::this_thread
#include <chrono> // For std::chrono
#include <iostream>
#include <modbus.h>
#include <cmath>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/GripperCommandAction.h>
/**
 * @brief This class wraps the functions of unitree-arm-sdk
 * for controlling HITBOT robots into the ros_control framework.
 */
class HITBOT_HW : public hardware_interface::RobotHW
{
public:
    /**
     * @brief Default constructor
     *
     * @note Be sure to call the init() method before operation.
     */
    HITBOT_HW(ros::NodeHandle &nh);

    /**
     * @brief Initializes the model informations.
     *
     */

    void init();
    void dinit();
    void gripper_port_init();
    void init_hardware_interface();
    void write_float_to_slave(modbus_t *ctx, int, int , float );
    void read(const ros::Time &time, const ros::Duration &period) override;
    void write(const ros::Time &time, const ros::Duration &period) override;
    double degreesToRadiansNormalized(double degrees);
    void claw_pos_callback(const std_msgs::Float64::ConstPtr &msg);
    void joint_command_callback(const sensor_msgs::JointState::ConstPtr& msg);
    void convert_cmd_high_to_low(const double cmd[4], double cmd_low_level[4]);
    void convert_cmd_low_to_high(const double cmd_low_level[4], double cmd[4]);
    void interpolate_and_send(double last_command[4], double command[4], int n);
    bool all_close(const double cmd[4], const double pos[4], double threshold);
    bool hardware_limit(const double cmd[4], const double cmd_last[4]);

    void read_thread();
    ros::Publisher command_debug_pub, joints_state_pub;
    ros::Subscriber gripper_position_sub, joints_command_sub;
    modbus_t *ctx;
    double claw_position;
    sensor_msgs::JointState joint_state;
    sensor_msgs::JointState joint_command;
    ros::Time last_command_time;
    float control_rate = 500;

private:
    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::PositionJointInterface arm_pos_interface;

    // control command for hitbot
    double cmd[4];
    double cmd_low_level[4];
    double cmd_low_level_last[4];
    bool cmd_state_close, pos_inited, cmd_inited;

    double pos[4];
    double vel[4];
    double eff[4];

    // control class for hitbot
    ControlBeanEx *rbt;
    ros::NodeHandle *_nh;
    float goal_angle1;
    float goal_angle2;
    float goal_z;
    float goal_r;
    float goal_speed;
    float rough;
    double DEG_TO_RAD = M_PI / 180.0;
    double EE_OFFSET = 28.0; // deg

};

#endif // Hitbot_ROS_control