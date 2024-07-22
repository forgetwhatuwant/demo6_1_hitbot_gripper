#include "hitbot_hw.hpp"
#include <thread>


HITBOT_HW::HITBOT_HW(ros::NodeHandle &nh)
{
    /* Setting parameters */
    _nh = &nh;
    command_debug_pub = _nh->advertise<std_msgs::Float32MultiArray>("/hitbot_scara/joint_command_debug", 10);
    joints_state_pub = _nh->advertise<sensor_msgs::JointState>("/hitbot_scara/joints_state", 10);

    this->init();
    joints_command_sub = nh.subscribe<sensor_msgs::JointState>("/hitbot_scara/joints_command", 10,  &HITBOT_HW::joint_command_callback, this);
    cmd_state_close = false;
    cmd_inited = false;
}

void HITBOT_HW::read_thread() {
    ros::Rate rate_read(50.0);

    while (ros::ok())
    {
        const ros::Time time = ros::Time::now();
        const ros::Duration period;
        // update the encoder reading
        this->read(time, period);
        rate_read.sleep();
    }
}


void HITBOT_HW::joint_command_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
    for (int i = 0; i < msg->position.size() && i < 4; ++i)
    {
        cmd[i] = msg->position[i];
    }
    cmd_inited = true;
    last_command_time = ros::Time::now();

}

void HITBOT_HW::init()
{
    ROS_INFO("Hitbot Arm Initializing...");

    int ret = net_port_initial();
    // TODO: pass robot id with ROS parameters
    rbt = get_robot(190);
    if (rbt->is_connected())
    {
        std::cout << "Robot connected" << std::endl;
    }
    else
    {
        std::cout << "Robot not connected" << std::endl;
    }
    //
    ret = rbt->initial(1, 410);
    std::cout << "robot initial return = " << ret << std::endl;
    rbt->set_robot_buf_size(100);
    rbt->set_Interpolation_position_buf_catch_count(200);
    // initialize the robot arm to zero position
    // rbt->new_movej_angle(0, 0, 0, -EE_OFFSET, 100, 1);
    // rbt->wait_stop();

    rbt->get_robot_real_coor();
    // get initial pos
    pos[0] = double(rbt->real_z / 1000.0);
    pos[1] = double(rbt->real_angle1 * DEG_TO_RAD);
    pos[2] = double(rbt->real_angle2 * DEG_TO_RAD);
    pos[3] = degreesToRadiansNormalized(rbt->real_rotation);
}

void HITBOT_HW::dinit()
{
    // stop the robot arm here
}

double HITBOT_HW::degreesToRadiansNormalized(double degrees)
{
    double radians = degrees * (M_PI / 180.0);

    // Normalize the angle to the range -pi to pi using fmod
    radians = std::fmod(radians, 2 * M_PI);

    // Adjust values to be within -pi to pi
    if (radians > M_PI)
    {
        radians -= 2 * M_PI;
    }
    else if (radians < -M_PI)
    {
        radians += 2 * M_PI;
    }

    return radians;
}

void HITBOT_HW::convert_cmd_high_to_low(const double cmd[4], double cmd_low_level[4]) {
    cmd_low_level[2] = double(cmd[0] * 1000);
    cmd_low_level[0] = double(cmd[1] / DEG_TO_RAD);
    cmd_low_level[1] = double(cmd[2] / DEG_TO_RAD);
    cmd_low_level[3] = (double(cmd[3] / DEG_TO_RAD) + cmd_low_level[0] + cmd_low_level[1] - EE_OFFSET);
}

void HITBOT_HW::convert_cmd_low_to_high(const double cmd_low_level[4], double cmd[4]) {
    cmd[0] = cmd_low_level[0] / 1000;
    cmd[1] = cmd_low_level[1] * DEG_TO_RAD;
    cmd[2] = cmd_low_level[2] * DEG_TO_RAD;
    cmd[3] = (cmd_low_level[3] - cmd_low_level[1] - cmd_low_level[2] + EE_OFFSET) * DEG_TO_RAD;
}

void HITBOT_HW::read(const ros::Time &time, const ros::Duration &period)
{
    // update the encoder reading
    rbt->get_robot_real_coor();
    double pos_low_level[4];
    pos_low_level[0] = rbt->real_z;
    pos_low_level[1] = rbt->real_angle1;
    pos_low_level[2] = rbt->real_angle2;
    pos_low_level[3] = rbt->real_rotation;
    convert_cmd_low_to_high(pos_low_level, pos); // Convert low level pos to high level

    // publish current joint states
    joint_state.position.resize(4);
    joint_state.header.stamp = ros::Time::now(); // Current time
    joint_state.name = {
        "joint1",
        "joint2",
        "joint3",
        "joint4",
    };
    for (int i = 0; i < 4; ++i) {
        joint_state.position[i] = pos[i];
    }
    joints_state_pub.publish(joint_state);
}

void HITBOT_HW::interpolate_and_send(double last_command[4], double command[4], int n) {
    if (n <= 0) {
        std::cout << "Number of steps must be positive." << std::endl;
        return;
    }
    double send_cmd[4] = {0.0, 0.0, 0.0, 0.0};
    
    for (int i = 1; i <= n; ++i) {
        for (int j = 0; j < 4; ++j) {
            // Linearly interpolate between last_command and command
            send_cmd[j] = last_command[j] + (command[j] - last_command[j]) * (static_cast<double>(i) / n);
        }
        // Execute the send function with the interpolated command
        ROS_INFO_STREAM("hi_position_send: " << send_cmd[0] << ", " << send_cmd[1] << ", " << send_cmd[2] << ", " << send_cmd[3]);
        rbt->hi_position_send(send_cmd[0], send_cmd[1], send_cmd[2], send_cmd[3]);
   }
}

bool HITBOT_HW::all_close(const double cmd[4], const double pos[4], double threshold) {
  for (int i = 0; i < 4; ++i) {
    if (std::abs(cmd[i] - pos[i]) > threshold) {
      return false;
    }
  }
  return true;
}

bool HITBOT_HW::hardware_limit(const double cmd[4], const double cmd_last[4]) {
    // cmd: [joint 1 in deg, joint2 in deg, joint3 in mm, joint4 in deg]
    // limit: joint 3 diff should not exceeds 4mm/ms and all joint 2,3,4 should not exceeds 2 degs/ms
    double control_period = (1/control_rate)*1000; // in ms

    // Limit for joint 3: 4 mm/ms
    const double joint3_limit = 1;
    // Limit for joints 2, 3, 4: 2 deg/ms
    const double joint124_limit = 1;
    bool valid = true;
    
    // std::cout << "cmd: " << cmd[0] << ", " << cmd[1] << ", " << cmd[2] << ", " << cmd[3] << std::endl;
    // std::cout << "cmd diff/ms: " << std::abs(cmd[0] - cmd_last[0])/ control_period << ", " 
    // << std::abs(cmd[1] - cmd_last[1])/ control_period << ", " 
    // << std::abs(cmd[2] - cmd_last[2])/ control_period << ", " 
    // << std::abs(cmd[3] - cmd_last[3])/ control_period << std::endl;

    // Check each joint individually using if statements
    if (std::abs(cmd[0] - cmd_last[0]) / control_period > joint124_limit) {
        std::cout << "joint 1 change exceed limit: " << std::abs(cmd[0] - cmd_last[0])/ control_period  << std::endl;
        valid = false;
    }

    if (std::abs(cmd[1] - cmd_last[1]) / control_period > joint124_limit) {
        std::cout << "joint 2 change exceed limit: " << std::abs(cmd[1] - cmd_last[1])/ control_period  << std::endl;
        valid = false;
    }

    if (std::abs(cmd[2] - cmd_last[2]) / control_period > joint3_limit) {
        std::cout << "joint 3 change exceed limit: " << std::abs(cmd[2] - cmd_last[2])/ control_period  << std::endl;
        valid = false;
    }

    if (std::abs(cmd[3] - cmd_last[3]) / control_period > joint124_limit) {
        std::cout << "joint 4 change exceed limit: " << std::abs(cmd[3] - cmd_last[3])/ control_period  << std::endl;
        valid = false;
    }

    return valid;
    }

void HITBOT_HW::write(const ros::Time &time, const ros::Duration &period)
{

    if(cmd_inited)
    {
        convert_cmd_high_to_low(cmd, cmd_low_level);
        double pos_low_level[4];
        convert_cmd_high_to_low(pos, pos_low_level);

        // std::cout << "cmd_low_level: " << cmd_low_level[0] << ", " << cmd_low_level[1] << ", " << cmd_low_level[2] << ", " << cmd_low_level[3] << std::endl;
        // std::cout << "pos_low_level: " << pos_low_level[0] << ", " << pos_low_level[1] << ", " << pos_low_level[2] << ", " << pos_low_level[3] << std::endl;
        // std::cout << "cmd_low_level_last: " << cmd_low_level_last[0] << ", " << cmd_low_level_last[1] << ", " << cmd_low_level_last[2] << ", " << cmd_low_level_last[3] << std::endl;

        // cmd_state_close = all_close(cmd_low_level, cmd_low_level_last, 1);
        cmd_state_close = hardware_limit(cmd_low_level, cmd_low_level_last);

        // cmd_state_close = false;

        // const ros::Time cur_time = ros::Time::now();
        // const ros::Duration command_gap = cur_time - last_command_time;
        // std::cout << "command_gap: " << command_gap.sec << std::endl;

        // if (command_gap.sec > 1.0)
        if (!cmd_state_close)
        {
            std::cout << "cmd_low_level: " << cmd_low_level[0] << ", " << cmd_low_level[1] << ", " << cmd_low_level[2] << ", " << cmd_low_level[3] << std::endl;
            // std::cout << "pos_low_level: " << pos_low_level[0] << ", " << pos_low_level[1] << ", " << pos_low_level[2] << ", " << pos_low_level[3] << std::endl;
            std::cout << "cmd_low_level_last: " << cmd_low_level_last[0] << ", " << cmd_low_level_last[1] << ", " << cmd_low_level_last[2] << ", " << cmd_low_level_last[3] << std::endl;

            // command and states are not close, use new_movej_angle()
            std::cout << "command and last cmd are not close, use new_movej_angle()" << std::endl;
            // std::cout << "command is old use new_movej_angle()" << std::endl;
            rbt->new_movej_angle(cmd_low_level[0], cmd_low_level[1], cmd_low_level[2], cmd_low_level[3], 100, 1);
            rbt->wait_stop();
            cmd_state_close = true;

        }
        else
        {   
            // command and states are close, use hi_position_send()
            // std::cout << "command and states are close, use hi_position_send()" << std::endl;
            int n = 1;
            // interpolate_and_send(pos_low_level, cmd_low_level, n);
            // interpolate_and_send(cmd_low_level_last, cmd_low_level, n);
            rbt->hi_position_send(cmd_low_level[0], cmd_low_level[1], cmd_low_level[2], cmd_low_level[3]);


        }

        for (int i = 0; i < 4; ++i) {
                cmd_low_level_last[i] = cmd_low_level[i];
            }

        // for debugging
        // ROS_INFO_STREAM("Joint Commands: " << cmroslaunch hitbot_hw hitbot_twin_and_upsample.launchd[0] << ", " << cmd[1] << ", " << cmd[2] << ", " << cmd[3]);
        std_msgs::Float32MultiArray commands;
        commands.data.push_back(cmd[0]);
        commands.data.push_back(cmd[1]);
        commands.data.push_back(cmd[2]);
        commands.data.push_back(cmd[3]);
        command_debug_pub.publish(commands);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hitbot_ros_msg_controller");
    ros::NodeHandle nh("~");

    HITBOT_HW robot(nh);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Time prev_time = ros::Time::now();
    ros::Time last_command_time = ros::Time::now();

    ros::Rate rate_write(robot.control_rate);
    std::thread t1(&HITBOT_HW::read_thread, robot);

    while (ros::ok())
    {
        const ros::Time time = ros::Time::now();
        const ros::Duration period = time - prev_time;
        prev_time = time;
        robot.write(time, period);
        // robot.read(time, period);

        rate_write.sleep();
    }
    t1.join();

    robot.dinit();

    return 0;
}