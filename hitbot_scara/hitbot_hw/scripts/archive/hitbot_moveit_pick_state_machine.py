#!/usr/bin/env python
"""
This script define the state machine of the robot picking
using the python transitions library to define a FSM (finite state machine)
The detail of the trasition library is in https://github.com/pytransitions/transitions

# Zhenghao Fei zfei@zju.edu.cn
"""

from transitions import Machine
import math
import rospy
import enum


class PickingStatesConsts(enum.Enum):
    IDLE = 0
    ERROR = 1
    PAUSE = 2
    EMERGENCY_STOP = 3
    READY = 4
    APPROACH = 5
    GRASP = 6
    POST_GRASP = 7
    PLACE = 8
    HOMING = 9


class PickingStateMachine(object):
    def __init__(
        self,
        name="PickingStateMachine",
        graph_machine=False,
    ):
        self.name = name
        self.states = PickingStatesConsts
        if graph_machine:
            from transitions.extensions import GraphMachine

            self.machine = GraphMachine(
                model=self,
                states=self.states,
                initial=self.states.IDLE,
            )
        else:
            self.machine = Machine(
                model=self,
                states=self.states,
                initial=self.states.IDLE,
            )
        self.define_transitions()
        self.init_variables()

    def init_variables(self):
        self.state = PickingStatesConsts.IDLE
        self.last_state = self.state

    def save_last_state(self):
        self.last_state = self.state

    def hello_state(self):
        rospy.loginfo(
            rospy.get_name()
            + " [PickFSM] leave %s enter %s" % (self.last_state.name, self.state.name)
        )

    def on_enter_ready(self):
        rospy.loginfo(rospy.get_name() + " [PickFSM] on_enter_ready")

    def on_enter_idle(self):
        rospy.loginfo(rospy.get_name() + " [PickFSM] on_enter_idle")

    def on_enter_approach(self):
        rospy.loginfo(rospy.get_name() + " [PickFSM] on_enter_approach")

    def on_enter_grasp(self):
        rospy.loginfo(rospy.get_name() + " [PickFSM] on_enter_grasp")

    def on_enter_post_grasp(self):
        rospy.loginfo(rospy.get_name() + " [PickFSM] on_enter_post_grasp")

    def on_enter_homing(self):
        rospy.loginfo(rospy.get_name() + " [PickFSM] on_enter_homing")

    def on_emergency_stop(self):
        rospy.loginfo(rospy.get_name() + " [PickFSM] on_emergency_stop")

    def define_transitions(self):
        self.machine.add_transition(
            trigger="enter_ready",
            source=[self.states.IDLE, self.states.HOMING],
            dest=self.states.READY,
            before=["save_last_state", "on_enter_ready"],
            after=["hello_state"],
        )

        self.machine.add_transition(
            trigger="enter_idle",
            source=self.states.READY,
            dest=self.states.IDLE,
            before=["save_last_state", "on_enter_idle"],
            after=["hello_state"],
        )

        self.machine.add_transition(
            trigger="enter_approach",
            source=self.states.READY,
            dest=self.states.APPROACH,
            before=["save_last_state", "on_enter_approach"],
            after=["hello_state"],
        )

        self.machine.add_transition(
            trigger="enter_grasp",
            source=self.states.APPROACH,
            dest=self.states.GRASP,
            before=["save_last_state", "on_enter_grasp"],
            after=["hello_state"],
        )

        self.machine.add_transition(
            trigger="enter_post_grasp",
            source=self.states.GRASP,
            dest=self.states.POST_GRASP,
            before=["save_last_state", "on_enter_post_grasp"],
            after=["hello_state"],
        )

        self.machine.add_transition(
            trigger="enter_homing",
            source="*",
            dest=self.states.HOMING,
            before=["save_last_state", "on_enter_homing"],
            after=["hello_state"],
        )

        self.machine.add_transition(
            trigger="emergency_stop",
            source="*",
            dest=self.states.EMERGENCY_STOP,
            before=["save_last_state", "on_emergency_stop"],
            after=["hello_state"],
        )


if __name__ == "__main__":
    picking_state = PickingStateMachine(graph_machine=True)
    picking_state.get_graph().draw("my_state_diagram.png", prog="dot")
