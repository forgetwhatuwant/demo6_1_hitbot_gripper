#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <modbus/modbus.h>

#include <iostream>
#include <chrono>
#include <thread>

// Declare global Modbus context
modbus_t *ctx;

void gripperPortInit()
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

void writeFloatToSlave(modbus_t *ctx, int slave_id, int addr, float value) {
    uint16_t registers[2];
    modbus_set_float_badc(value, registers);

    modbus_set_slave(ctx, slave_id);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    int rc = modbus_write_registers(ctx, addr, 2, registers);
    if (rc == -1) {
        std::cerr << "Write failed to slave " << slave_id << ": " << modbus_strerror(errno) << std::endl;
    }
    else
    {
        // std::cout << "Float value " << value << " written to slave " << slave_id << "\n";
    }
}

// Callback function to handle received messages
void clawPositionCallback(const std_msgs::Float64::ConstPtr& msg) {
    float claw_position = msg->data;
    // Assuming slave ID and address are constants, change as needed
    const int slave_id = 1;
    const int addr = 0x02; // Modbus address to write to
    // std::cout << "Robot claw_position: " << claw_position * 1000 << std::endl;

    writeFloatToSlave(ctx, slave_id, addr, claw_position * 1000);
}

int main(int argc, char **argv) {
    // Initialize ROS node
    ros::init(argc, argv, "gripper_controller");
    ros::NodeHandle nh;

    // Initialize Modbus connection
    gripperPortInit();
    ROS_INFO_STREAM("Gripper Controller Initiated");

    // Subscribe to the claw position topic
    ros::Subscriber gripper_position_sub = nh.subscribe<std_msgs::Float64>("/hitbot_claw_position", 10, clawPositionCallback);

    while (ros::ok())
    {
        // ROS event loop
        ros::spin();
    }
    // Clean up Modbus connection
    modbus_close(ctx);
    modbus_free(ctx);

    return 0;
}