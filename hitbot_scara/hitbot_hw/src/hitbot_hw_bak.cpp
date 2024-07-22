#include "hitbot_hw.hpp"
#include <std_msgs/Float64MultiArray.h>


HITBOT_HW::HITBOT_HW(ros::NodeHandle &nh)
{
    /* Setting parameters */
    _nh = &nh;
    command_debug_pub = _nh->advertise<std_msgs::Float32MultiArray>("joint_command", 10);

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
    rbt->new_movej_angle(0, 0, 0, -EE_OFFSET, 100, 1);
    // cmd[0] = 57;
    // cmd[1] = -124;
    // cmd[2] = -141;
    // cmd[3] = -29;
    // rbt->new_movej_angle(cmd[0], cmd[1], cmd[2], cmd[3], 100, 1);
 
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
    float converted_angle;
    // update the encoder reading
    rbt->get_robot_real_coor();

    // get current pos
    pos[0] = rbt->real_z / 1000.0;
    pos[1] = rbt->real_angle1 * DEG_TO_RAD;
    pos[2] = rbt->real_angle2 * DEG_TO_RAD;
    converted_angle = rbt->real_rotation - rbt->real_angle1 - rbt->real_angle2 + EE_OFFSET;
    pos[3] = converted_angle*DEG_TO_RAD;
}

void HITBOT_HW::write(const ros::Time &time, const ros::Duration &period)
{
    std_msgs::Float32MultiArray commands;
    double goal_r_deg;

    goal_z = double(cmd[0] * 1000);
    goal_angle1 = double(cmd[1] / DEG_TO_RAD);
    goal_angle2 = double(cmd[2] / DEG_TO_RAD);
    goal_r_deg = double(cmd[3] / DEG_TO_RAD);
    goal_r = (goal_r_deg + goal_angle1 + goal_angle2 - EE_OFFSET);
    // std::cout << "Commands: " << goal_angle1 << " , " << goal_angle2 << " , " << goal_z << " , " << goal_r << std::endl;
    // publish the command with a message
    // rbt->new_movej_angle(goal_angle1, goal_angle2, goal_z, goal_r, 200, 1);
    rbt->hi_position_send(goal_angle1, goal_angle2, goal_z, goal_r);

    // for debugging
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