#!/usr/bin/env python3

## Header ##########################################################
import rclpy
import time
import signal
import PSU_serial as PSU
from structure import *
from computation import *
from visualization import *

## ROS signal shutdown #############################################
## Main function ###################################################
test_mode = False
theta = 0.0                                    # initial orientation
Res = np.array([8.80, 8.00, 8.40, 8.60,    # resistance of each coil
                8.40, 8.70, 8.50, 8.60])

def experiment():

    rclpy.init()
    node = rclpy.create_node('experiment')

    def signal_handler(signal, frame):
        node.get_logger().info('You pressed Ctrl+C! Exiting...')
        rclpy.shutdown()

    signal.signal(signal.SIGINT, signal_handler)

    field = ArrowArray(node, 'field')
    r1 = MarkerPoint(node, 'r1')
    r3 = MarkerPoint(node, 'r3')

    if not test_mode:
        PSU.Turn_on()

    global theta  

    while rclpy.ok():
    # while theta <= 2*np.pi:

        Structure1 = Config1(
            distance=2.5, 
            moment_value=0.001875,
            number_magnets=3,
            )

        Eval = Evaluation(
            structure=Structure1,
            position=np.array([[60.0], [60.0], [64.0]]),
            orientation=theta,
            desired_torque=np.array([[0.0], [0.0], [0.007]]),
            desired_force=np.array([[0.0], [0.0], [0.0]]),
            )
        
        Com = Computation(
            structure=Structure1,
            position=np.array([[60.0], [60.0], [64.0]]),
            orientation=theta,
            desired_torque=np.array([[0.0], [0.0], [0.007]]),
            desired_force=np.array([[0.0], [0.0], [0.0]]),
            current_vector=Eval.Iopt
            )

        V = Eval.Iopt.flatten() * Res

        print(Eval.Iopt)

        field.publish_arrow_array(spacing=2.5/4,
                                  num_grid=15,
                                  shaft_thick=0.05,
                                  head_size=0.2,
                                  color=yellow,
                                  vector_function=Com.comField)

        r1.publish_point(radius=0.5,
                         color=red, 
                         position=Eval.rW['1'].flatten())
        
        r3.publish_point(radius=0.5,
                         color=blue,
                         position=Eval.rW['3'].flatten())

        if not test_mode:
            PSU.Transmit_DC(V)

        theta += np.pi/180

        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(0.1)

    if not test_mode:
        PSU.Turn_off()

    node.destroy_node()
    rclpy.shutdown()

## Execution #######################################################
if __name__ == '__main__':
    experiment()
