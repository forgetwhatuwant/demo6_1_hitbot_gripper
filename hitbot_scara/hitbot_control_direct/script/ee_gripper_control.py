#!/usr/bin/env python
from pymodbus.client import ModbusSerialClient as ModbusClient
from pymodbus.exceptions import ModbusException, ConnectionException
import logging
import struct
import copy
import math
import rospy
import time


class GripperControl(object):
    SLAVE_ADDRESS = 0x01
    POSITION_SET_ADDRESS = 0x02
    VELOCITY_SET_ADDRESS = 0x04
    CURRENT_SET_ADDRESS = 0x06
    READ_LENGTH = 2
    POSITION_READ_ADDRESS = 0x42
    VELOCITY_READ_ADDRESS = 0x44
    CURRENT_READ_ADDRESS = 0x46

    def __init__(self) -> None:
        serial_port = "/dev/ttyUSB_girpper"
        baud_rate = 115200
        parity = "N"
        data_bits = 8
        stop_bits = 1
        self.client = ModbusClient(
            method="rtu",
            port=serial_port,
            baudrate=baud_rate,
            stopbits=stop_bits,
            bytesize=data_bits,
            parity=parity,
        )
        if self.client.connect():
            print("Modbus RTU Client Connected")
        else:
            print("Failed to connect to Modbus RTU Client")

    def set_gripper_position(self, position):
        gripper_position_values = self.float_to_32bit_hex_values(position)
        write_response = self.client.write_registers(
            address=self.POSITION_SET_ADDRESS,
            values=gripper_position_values,
            slave=self.SLAVE_ADDRESS,
        )
        if not write_response.isError():
            # print("position written successfully")
            pass
        else:
            print("Failed to write position")

    def set_gripper_velocity(self, velocity):
        gripper_velocity_values = self.float_to_32bit_hex_values(velocity)
        velocity_write_response = self.client.write_registers(
            address=self.VELOCITY_SET_ADDRESS,
            values=gripper_velocity_values,
            slave=self.SLAVE_ADDRESS,
        )
        if not velocity_write_response.isError():
            # print("velocity written successfully")
            pass
        else:
            print("Failed to write velocity")

    def set_gripper_current(self, current):
        gripper_current_values = self.float_to_32bit_hex_values(current)
        current_write_response = self.client.write_registers(
            address=self.CURRENT_SET_ADDRESS,
            values=gripper_current_values,
            slave=self.SLAVE_ADDRESS,
        )
        if not current_write_response.isError():
            # print("current written successfully")
            pass
        else:
            print("Failed to write current")

    def read_gripper_position(self):
        position_read_response = self.client.read_holding_registers(
            address=self.POSITION_READ_ADDRESS,
            count=self.READ_LENGTH,
            slave=self.SLAVE_ADDRESS,
        )
        if not position_read_response.isError():
            # print("Register Values: ", position_read_response.registers)
            hex_values = position_read_response.registers
            result_position = self.hex_values_to_float(hex_values)
            # print(f"合成的浮点数: {result_position}")
        else:
            print("Failed to read registers")
            result_position = -1
        return result_position

    def read_gripper_velocity(self):
        velocity_read_response = self.client.read_holding_registers(
            address=self.VELOCITY_READ_ADDRESS,
            count=self.READ_LENGTH,
            slave=self.SLAVE_ADDRESS,
        )
        if not velocity_read_response.isError():
            # print("Register Values: ", velocity_read_response.registers)
            hex_values = velocity_read_response.registers
            result_velocity = self.hex_values_to_float(hex_values)
            # print(f"合成的浮点数: {result_velocity}")
        else:
            print("Failed to read registers")
            result_velocity = -1
        return result_velocity

    def read_gripper_current(self):
        current_read_response = self.client.read_holding_registers(
            address=self.CURRENT_READ_ADDRESS,
            count=self.READ_LENGTH,
            slave=self.SLAVE_ADDRESS,
        )
        if not current_read_response.isError():
            # print("Register Values: ", current_read_response.registers)
            hex_values = current_read_response.registers
            result_current = self.hex_values_to_float(hex_values)
        else:
            print("Failed to read registers")
            result_current = -1
        return result_current

    def float_to_32bit_hex_values(self, float_value):
        # 使用struct.pack()将浮点数转换为4字节的字节对象
        byte_data = struct.pack("!f", float_value)
        return_value = 0
        # 使用struct.unpack()将4字节的字节对象转换为一个32位的整数
        int_value = struct.unpack("!I", byte_data)[0]

        # 取高16位和低16位
        high_16_bits = int_value >> 16
        low_16_bits = int_value & 0xFFFF
        # print("high_16_bits:", high_16_bits)
        # print("low_16_bits:", low_16_bits)
        # # 将两个16位整数分别转换为16进制字符串，前面加上'0x'，然后用逗号隔开
        # hex_values = f'0x{format(high_16_bits, "04X")}, 0x{format(low_16_bits, "04X")}'

        return [high_16_bits, low_16_bits]

    def hex_values_to_float(self, hex_values):
        # 将两个16进制数合并为一个32位的整数
        combined_value = (hex_values[0] << 16) | hex_values[1]

        # 使用struct.unpack()将整数解包为浮点数
        float_value = struct.unpack("!f", struct.pack("!I", combined_value))[0]

        return float_value


# if __name__ == "__main__":
#     gripper = GripperControl()
#     while not rospy.is_shutdown():
#         gripper.set_gripper_velocity(100)
#         gripper.set_gripper_position(10)
#         time.sleep(0.01)
#         gripper.set_gripper_position(20)
#         print(gripper.read_gripper_current())
