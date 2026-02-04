#!/usr/bin/env python3

## Header #########################################################################################
import rclpy
import signal
import numpy as np
from structure import RobotDesign
from solver import ActuationModel
from markers import *

## Streaming field result and estimated orientation ###############################################
def stream_State():
    theta = 0.0                                                               # initial orientation

    rclpy.init()
    node = rclpy.create_node('stream_')

    shutting_down = {"value": False}

    def signal_handler(signal, frame):
        if shutting_down["value"]:
            return
        shutting_down["value"] = True
        node.get_logger().info('You pressed Ctrl+C! Exiting...')
        if rclpy.ok():
            rclpy.shutdown()

    signal.signal(signal.SIGINT, signal_handler)

    # b_array = ArrowArray(node, 'b_array')
    r1 = MarkerPoint(node, 'r1')
    r3 = MarkerPoint(node, 'r3')
    m1 = MarkerArrow(node, 'm1')
    m3 = MarkerArrow(node, 'm3')

    while rclpy.ok():

        Robot = RobotDesign(
            magnet_distance=2.5,
            moment_value=0.001875,
            number_magnets=3,
            pattern="e5",
            )

        Act = ActuationModel(
            robot_design=Robot,
            robot_position=np.array([[0.0], [0.0], [0.0]]),
            robot_orientation=theta,
            desired_torque=np.array([[0.0], [0.0], [0.007]]),
            desired_force=np.array([[0.0], [0.0], [0.0]]),
            )

        # b_array.publish_arrow_array(spacing=2.5/4,
        #                           num_grid=15,
        #                           shaft_thick=0.05,
        #                           head_size=0.2,
        #                           color=MarkerColors().yellow,
        #                           vector_function=Act.compute_Field)

        # r1.publish_SinglePoint(radius=0.5,
        #                  color=MarkerColors().red, 
        #                  position=Act.rw['1'])
        
        # r3.publish_SinglePoint(radius=0.5,
        #                  color=MarkerColors().blue,
        #                  position=Act.rw['3'])

        m1.publish_SingleArrow(shaft_thick=1.0,
                               head_size=0.5,
                               color=MarkerColors().red,
                               position=Act.rw['1'],
                               orientation=Act.mw['1'])

        m3.publish_SingleArrow(shaft_thick=1.0,
                               head_size=0.5,
                               color=MarkerColors().blue,
                               position=Act.rw['3'],
                               orientation=Act.mw['3'])

        # theta += np.pi/180

        # print(Act.rw['1'].flatten(), Act.mw['1'].flatten())
        print(Act.rw['3'].flatten(), Act.mw['3'].flatten())

        rclpy.spin_once(node, timeout_sec=0.0)

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

## Main function ##################################################################################
def main():
    stream_State()

## Execution ######################################################################################
if __name__ == "__main__":
    main()
