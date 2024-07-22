#include "hitbot_hw.hpp"
#include <std_msgs/Float64MultiArray.h>


HITBOT_HW::HITBOT_HW(ros::NodeHandle &nh)
{
    /* Setting parameters */
    _nh = &nh;
    command_debug_pub = _nh->advertise<std_msgs::Float32MultiArray>("/hitbot_scara/joint_command_debug", 10);
    joints_state_pub = _nh->advertise<sensor_msgs::JointState>("/hitbot_scara/joints_state", 10);

    this->init();
    this->init_hardware_interface();
    gripper_position_sub = nh.subscribe<std_msgs::Float64>("/hitbot_claw_position", 10, &HITBOT_HW::claw_pos_callback, this);
}

void HITBOT_HW::claw_pos_callback(const std_msgs::Float64::ConstPtr &msg)
{
    // ROS_INFO("Received double data: %.4f", msg->data);
    claw_position = msg->data;
}

void HITBOT_HW::init_hardware_interface()
{
    std::cout << "Register hardware" << std::endl;
    /* Define hardware interface */
    hardware_interface::JointStateHandle state_handle_1("joint1", &pos[0], &vel[0], &eff[0]);
    jnt_state_interface.registerHandle(state_handle_1);
    hardware_interface::JointStateHandle state_handle_2("joint2", &pos[1], &vel[1], &eff[1]);
    jnt_state_interface.registerHandle(state_handle_2);
    hardware_interface::JointStateHandle state_handle_3("joint3", &pos[2], &vel[2], &eff[2]);
    jnt_state_interface.registerHandle(state_handle_3);
    hardware_interface::JointStateHandle state_handle_4("joint4", &pos[3], &vel[3], &eff[3]);
    jnt_state_interface.registerHandle(state_handle_4);

    registerInterface(&jnt_state_interface);

    hardware_interface::JointHandle pos_handle_1(jnt_state_interface.getHandle("joint1"), &cmd[0]);
    arm_pos_interface.registerHandle(pos_handle_1);
    hardware_interface::JointHandle pos_handle_2(jnt_state_interface.getHandle("joint2"), &cmd[1]);
    arm_pos_interface.registerHandle(pos_handle_2);
    hardware_interface::JointHandle pos_handle_3(jnt_state_interface.getHandle("joint3"), &cmd[2]);
    arm_pos_interface.registerHandle(pos_handle_3);
    hardware_interface::JointHandle pos_handle_4(jnt_state_interface.getHandle("joint4"), &cmd[3]);
    arm_pos_interface.registerHandle(pos_handle_4);

    registerInterface(&arm_pos_interface);
    gripper_port_init();
}
void HITBOT_HW::gripper_port_init()
{
    // init gripper write
    ctx = modbus_new_rtu("/dev/ttyUSB_girpper", 115200, 'N', 8, 1);
    if (ctx == nullptr)
    {
        ;
    }
    if (modbus_connect(ctx) == -1)
    {
        modbus_free(ctx);
    }
    uint32_t to_sec = 1;    // s
    uint32_t to_usec = 100; // us
    modbus_set_response_timeout(ctx, to_sec, to_usec);
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
    rbt->set_robot_buf_size(32);
    rbt->set_Interpolation_position_buf_catch_count(32);
    // initialize the robot arm to zero position
    // rbt->new_movej_angle(0, 0, 0, -EE_OFFSET, 100, 1);
    cmd_low_level[0] = 57;
    cmd_low_level[1] = -124;
    cmd_low_level[2] = -141;
    cmd_low_level[3] = -29;

    // cmd_low_level[0] = 0;
    // cmd_low_level[1] = 0;
    // cmd_low_level[2] = 0;
    // cmd_low_level[3] = -EE_OFFSET;
    rbt->new_movej_angle(cmd_low_level[0], cmd_low_level[1], cmd_low_level[2], cmd_low_level[3], 100, 1);
    for (int i = 0; i < 4; ++i) {
        cmd_low_level_last[i] = cmd_low_level[i];
    }

    // rbt->new_movej_angle(0, 0, 0, -EE_OFFSET, 100, 1);

    rbt->wait_stop();

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
void HITBOT_HW::write_float_to_slave(modbus_t *ctx, int slave_id, int addr, float value)
{
    uint16_t registers[2];
    modbus_set_float_badc(value, registers); // 转换浮点数为 Modbus 寄存器格式

    modbus_set_slave(ctx, slave_id); // 设置从机地址
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    int rc = modbus_write_registers(ctx, addr, 2, registers);
    if (rc == -1)
    {
        std::cerr << "Write failed to slave " << slave_id << ": " << modbus_strerror(errno) << "\n";
    }
    else
    {
        // std::cout << "Float value written to slave " << slave_id << "\n";
        ;
    }
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
void HITBOT_HW::read(const ros::Time &time, const ros::Duration &period)
{
    // update the encoder reading
    rbt->get_robot_real_coor();

    // get current pos
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

// void HITBOT_HW::write(const ros::Time &time, const ros::Duration &period)
// {

//     convert_cmd_high_to_low(cmd, cmd_low_level);
//     double pos_low_level[4];
//     // convert_cmd_high_to_low(pos, pos_low_level);

//     // std::cout << "cmd_low_level: " << cmd_low_level[0] << ", " << cmd_low_level[1] << ", " << cmd_low_level[2] << ", " << cmd_low_level[3] << std::endl;
//     // std::cout << "pos_low_level: " << pos_low_level[0] << ", " << pos_low_level[1] << ", " << pos_low_level[2] << ", " << pos_low_level[3] << std::endl;
//     // std::cout << "cmd_low_level_last: " << cmd_low_level_last[0] << ", " << cmd_low_level_last[1] << ", " << cmd_low_level_last[2] << ", " << cmd_low_level_last[3] << std::endl;

//     // cmd_state_close = all_close(cmd_low_level, cmd_low_level_last, 1);
//     // cmd_state_close = hardware_limit(cmd_low_level, cmd_low_level_last);

//     cmd_state_close = true;

//     // const ros::Time cur_time = ros::Time::now();
//     // const ros::Duration command_gap = cur_time - last_command_time;
//     // std::cout << "command_gap: " << command_gap.sec << std::endl;

//     // if (command_gap.sec > 1.0)
//     if (!cmd_state_close)
//     {
//         // std::cout << "cmd_low_level: " << cmd_low_level[0] << ", " << cmd_low_level[1] << ", " << cmd_low_level[2] << ", " << cmd_low_level[3] << std::endl;
//         // std::cout << "pos_low_level: " << pos_low_level[0] << ", " << pos_low_level[1] << ", " << pos_low_level[2] << ", " << pos_low_level[3] << std::endl;
//         // std::cout << "cmd_low_level_last: " << cmd_low_level_last[0] << ", " << cmd_low_level_last[1] << ", " << cmd_low_level_last[2] << ", " << cmd_low_level_last[3] << std::endl;

//         // command and states are not close, use new_movej_angle()
//         // std::cout << "command and last cmd are not close, use new_movej_angle()" << std::endl;
//         // std::cout << "command is old use new_movej_angle()" << std::endl;
//         rbt->new_movej_angle(cmd_low_level[0], cmd_low_level[1], cmd_low_level[2], cmd_low_level[3], 100, 1);
//         // rbt->wait_stop();
//         // cmd_state_close = true;

//     }
//     else
//     {   
//         // command and states are close, use hi_position_send()
//         // std::cout << "command and states are close, use hi_position_send()" << std::endl;
//         // std::cout << "cmd_low_level: " << cmd_low_level[0] << ", " << cmd_low_level[1] << ", " << cmd_low_level[2] << ", " << cmd_low_level[3] << std::endl;

//         int n = 1;
//         // interpolate_and_send(pos_low_level, cmd_low_level, n);
//         // interpolate_and_send(cmd_low_level_last, cmd_low_level, n);
//         rbt->hi_position_send(cmd_low_level[0], cmd_low_level[1], cmd_low_level[2], cmd_low_level[3]);


//     }

//     for (int i = 0; i < 4; ++i) {
//             cmd_low_level_last[i] = cmd_low_level[i];
//         }

//     // for debugging
//     // ROS_INFO_STREAM("Joint Commands: " << cmroslaunch hitbot_hw hitbot_twin_and_upsample.launchd[0] << ", " << cmd[1] << ", " << cmd[2] << ", " << cmd[3]);
//     std_msgs::Float32MultiArray commands;
//     commands.data.push_back(cmd[0]);
//     commands.data.push_back(cmd[1]);
//     commands.data.push_back(cmd[2]);
//     commands.data.push_back(cmd[3]);
//     command_debug_pub.publish(commands);
// }


void HITBOT_HW::write(const ros::Time &time, const ros::Duration &period)
{

    convert_cmd_high_to_low(cmd, cmd_low_level);
    // rbt->new_movej_angle(cmd_low_level[0], cmd_low_level[1], cmd_low_level[2], cmd_low_level[3], 100, 1);
    rbt->hi_position_send(cmd_low_level[0], cmd_low_level[1], cmd_low_level[2], cmd_low_level[3]);

    // for debugging
    std_msgs::Float32MultiArray commands;
    commands.data.push_back(cmd[0]);
    commands.data.push_back(cmd[1]);
    commands.data.push_back(cmd[2]);
    commands.data.push_back(cmd[3]);
    command_debug_pub.publish(commands);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hitbot_ros_controller");
    ros::NodeHandle nh("~");

    HITBOT_HW robot(nh);

    // ros::Subscriber sub = nh.subscribe("number_topic", 10, numberclaw_pos_callback);
    controller_manager::ControllerManager cm(&robot);
    
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Time prev_time = ros::Time::now();
    ros::Rate rate(300.0);

    int init_loop = 0;

    for (int i = 0; i < 2000; ++i)
    {
        // this loop is for initializing the controller manager
        const ros::Time time = ros::Time::now();
        const ros::Duration period = time - prev_time;
        robot.read(time, period);
        cm.update(time, period);
        rate.sleep();
    }

    // ros log 
    ROS_INFO("Hitbot Arm Ready to Move");
    
    while (ros::ok())
    {
        const ros::Time time = ros::Time::now();
        const ros::Duration period = time - prev_time;
        prev_time = time;
        // auto start_time = std::chrono::high_resolution_clock::now();
        robot.read(time, period);
        cm.update(time, period);
        robot.write(time, period);
        // robot.write_float_to_slave(robot.ctx, 1, 0x02, robot.claw_position * 1000);
        // auto end_time = std::chrono::high_resolution_clock::now();

        // // 计算运行时间
        // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        // std::cout << "程序运行时间: " << duration.count() << " 微秒" << std::endl;
        rate.sleep();

    }

    robot.dinit();

    return 0;
}