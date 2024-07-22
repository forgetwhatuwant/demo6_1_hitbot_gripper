#!/usr/bin/env python
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_commander import conversions  # 添加这一行
import sys


def moveit_example():
    # 初始化 ROS 节点
    rospy.init_node("moveit_example", anonymous=True)

    # 初始化 MoveIt
    moveit_commander.roscpp_initialize(sys.argv)

    # 创建机械臂控制对象
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "arm_group"  # 替换为你的机械臂组的名称
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # 设置初始位置
    start_joint_values = move_group.get_current_joint_values()

    # 设置目标位置列表
    target_poses = []

    # 第一个目标位置
    pose1 = geometry_msgs.msg.PoseStamped()
    pose1.header.stamp = rospy.Time.now()
    pose1.header.frame_id = "world"
    pose1.pose.position.x = 0.311305
    pose1.pose.position.y = -0.182233
    pose1.pose.position.z = 0.182810505041290129
    pose1.pose.orientation.x = 0.035
    pose1.pose.orientation.y = 1
    pose1.pose.orientation.z = 0.0
    pose1.pose.orientation.w = 0.00
    target_poses.append(pose1)

    # 第二个目标位置（插入的新目标点位）
    pose2 = geometry_msgs.msg.PoseStamped()
    pose2.header.stamp = rospy.Time.now()
    pose2.header.frame_id = "world"
    pose2.pose.position.x = 0.311305
    pose2.pose.position.y = 0.132233
    pose2.pose.position.z = 0.162810505041290129
    pose2.pose.orientation.x = 0.035
    pose2.pose.orientation.y = 1
    pose2.pose.orientation.z = 0.0
    pose2.pose.orientation.w = 0.00
    target_poses.append(pose2)

    # 移动到第一个目标位置
    move_group.set_pose_target(target_poses[0])
    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    # 在機器人移動的過程中插入新的目標點
    waypoints = []
    waypoints.append(move_group.get_current_pose().pose)  # 当前位置作为起点

    # 添加插入的新目标点
    new_pose = geometry_msgs.msg.PoseStamped()
    new_pose.header.stamp = rospy.Time.now()
    new_pose.header.frame_id = "world"
    new_pose.pose.position.x = 0.311305
    new_pose.pose.position.y = 0.132233
    new_pose.pose.position.z = 0.162810505041290129
    new_pose.pose.orientation.x = 0.035
    new_pose.pose.orientation.y = 1
    new_pose.pose.orientation.z = 0.0
    new_pose.pose.orientation.w = 0.00
    waypoints.append(new_pose)

    # 计算插补路径
    fraction = 0.0
    plan, fraction = move_group.compute_cartesian_path(
        [conversions.pose_to_list(p.pose) for p in waypoints],
        0.01,  # eef_step 步进值
        0.0,  # jump_threshold 跳跃阈值
        True,  # avoid_collisions 避免碰撞
    )

    # 执行插补路径
    move_group.execute(plan, wait=True)
    move_group.stop()

    # 结束 MoveIt 进程
    moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    try:
        moveit_example()
    except rospy.ROSInterruptException:
        pass
