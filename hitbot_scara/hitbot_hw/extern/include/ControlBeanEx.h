#ifndef CONTROLBEANEX_H
#define CONTROLBEANEX_H

class ControlBeanEx {
private:
    void* bean;
    float private_V[8];

public:
    float x, y, z, goal_angle1, goal_angle2, rotation;
    bool communicate_success, initial_finish, servo_off_flag, move_flag;
    float angle1_after_judge, angle2_after_judge;
    bool isReach_after_judge;
    float real_x, real_y, real_z, real_angle1, real_angle2, real_rotation;

    ControlBeanEx(void* b);
    ~ControlBeanEx();
    bool    is_connected();
    int     initial(int generation, float z_travel);
    int     set_angle_move(float angle1, float angle2, float z, float rotation,float speed);
    bool    judge_in_range(float x, float y, float z, float ratation);
    void    get_scara_param();
    bool    is_collision();
    int     get_card_num();
    void    set_arm_length(float l1, float l2);
    int     change_attitude(float speed);
    int     single_axis_move(int axis, float distance);
    int     trail_move(int point_number, float* x, float* y, float* z, float* r, float speed);
    int     set_position_move(float goal_x, float goal_y, float goal_z, float rotation, float speed, float acceleration,int interpolation, int move_mode);
    int     xyz_move(int direction, float distance, float speed);
    void    stop_move();
    bool    set_digital_out(int io_number, bool value);
    int     unlock_position();
    int     get_digital_in(int io_in_number);
    int     set_efg_state(int type, float distance);
    int     get_efg_state(int type, float* distance);
    int     get_efg_state(int* type, float* distance);
    //新版电动夹爪状态控制接口  -1：该机型不支持；1：设置指令成功；2：输入参数错误 3：未初始化；101：参数异常，被篡改；
    int     set_efg_state(int channel, int type, float distance);
    //新版电动夹爪状态控制接口 -1：该机型不支持；1：设置指令成功； 3：未初始化；
    int     get_efg_state(int channel, int type, float* distance);
    //Micah 123
    int     get_efg_state_dji(int* type, float* distance);
    int     set_efg_state_dji(int type, float distance);
    
    int     get_digital_out(int io_out_num);
    bool    set_cooperation_fun_state(bool state);
    bool    get_cooperation_fun_state();
    bool    set_drag_teach(bool state);
    bool    get_drag_teach();
    bool    judge_position_gesture(float x, float y);
    void    get_robot_real_coor();
    bool    is_robot_goto_target();
    int     single_joint_move(int axis, float distance, float speed);
    void    set_allow_distance_at_target_position(float x_distance, float y_distance, float z_distance, float r_distance);
    int     start_joint_monitor();
    int     get_joint_state(int joint_num);
    int     joint_home(int joint_num);
    void    set_catch_or_release_accuracy(float accuracy);
    int     movel_xyz(float goal_x, float goal_y, float goal_z, float rotation, float speed);
    int     movej_xyz(float goal_x, float goal_y, float goal_z, float goal_r, float speed, float rough);
    int     movej_angle(float angle1, float angle2, float goal_z, float goal_r, float speed, float rough);
    bool    wait_stop();
    void    clear_move_list_buf();
    float   get_arm1_length();
    float   get_arm2_length();
    int     pause_move();
    int     resume_move();
    int     movel_xyz_with_joint5(float goal_x, float goal_y, float goal_z, float goal_r, float joint5, float speed);
    int     movej_xyz_lr(float goal_x, float goal_y, float goal_z, float goal_r, float speed, float roughly, int lr);
    int     movej_j5(float j5_pos, float speed);
    float   get_j5_positon(int type);
    int     read_efrom(int addr);
    int     write_eform(int addr, int value);
    int     get_robot_id();
    int     limited_speed(bool state);
    float   get_current_comm_error(int joint_num);

    int     get_error_code();
    int     set_tool_fun1(float Tool_L,float Tool_R);
    int     set_tool_fun2(float p_x, float p_y);
    int     set_tool_fun3(float p1_x, float p1_y, float p1_r, float p2_x, float p2_y, float p2_r);
    int     new_movej_xyz_lr(float goal_x, float goal_y, float goal_z, float goal_r, float speed, float roughly, int lr);
    int     new_movej_angle(float angle1, float angle2, float goal_z, float goal_r, float speed, float rough);
    //Z轴自动零位校准验证测试
    int     auto_set_z_zero();
    void    set_j1_limit(float j1_deg);
    bool    check_joint(int joint_num,bool state);
    //PID参数设置
    int     set_pid_by_id(int num);//add by yhr 2022-06-27 氦豚两套pid参数订制需求
    int     com485_initial(int  baudRate);
    int     com485_send(unsigned char * data,unsigned char len);
    int     com485_recv(unsigned char* data);
    int     get_hard_emergency_stop_state();
    int     clear_hard_emergency_stop();
    int     set_Interpolation_position_buf_catch_count(int count);
    int     set_robot_buf_size(int count);
    int     hi_position_send(double j1_deg, double j2_deg, double z_mm, double j4_deg);
};


#endif  // CONTROLBEANEX_H
