#!/usr/bin/env python3

## Header ##########################################################
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

## Marker color ####################################################
red = np.array([1.0, 0.0, 0.0])
green = np.array([0.0, 1.0, 0.0])
blue = np.array([0.0, 0.0, 1.0])
yellow = np.array([1.0, 1.0, 0.0])
white = np.array([1.0, 1.0, 1.0])
black = np.array([0.0, 0.0, 0.0])

## Point Marker ####################################################
class MarkerPoint:

    def __init__(self, node: Node, topic_name: str):
        self.node = node

        self.publisher = node.create_publisher(Marker, topic_name, 10)

    def publish_point(self, 
                      radius,
                      color, 
                      position):

        marker = Marker()
        marker.header = Header(frame_id="map", stamp=self.node.get_clock().now().to_msg())
        marker.ns = "single_point"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = radius
        marker.scale.y = radius
        marker.scale.z = radius
        marker.color.a = 1.0
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]

        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]

        self.publisher.publish(marker)

## Arrow marker ####################################################
class MarkerArrow:

    def __init__(self, node: Node):
        self.node = node

    def create_arrow_marker(self,
                            namespace,
                            marker_id,                            
                            shaft_thick,
                            head_size,
                            color,
                            bound1,
                            bound2):
        
        marker = Marker()
        marker.header = Header(frame_id="map", stamp=self.node.get_clock().now().to_msg())
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x = shaft_thick         # arrow shaft thickness
        marker.scale.y = head_size                  # arrowhead size
        marker.color.a = 1.0                          # transparency
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]

        initial = Marker().pose.position
        initial.x, initial.y, initial.z = bound1

        terminal = Marker().pose.position
        terminal.x = bound1[0] + bound2[0] * 1
        terminal.y = bound1[1] + bound2[1] * 1
        terminal.z = bound1[2] + bound2[2] * 1
        
        marker.points.append(initial)                # initial point
        marker.points.append(terminal)              # terminal point

        return marker

class SingleArrow(MarkerArrow):
    
    def __init__(self, node: Node, topic_name: str):
        super().__init__(node)

        self.publisher = node.create_publisher(Marker, topic_name, 10)
        
    def publish_single_arrow(self,
                             shaft_thick,
                             head_size,
                             color,
                             bound1,
                             bound2):
        
        marker = self.create_arrow_marker(shaft_thick=shaft_thick,
                                          head_size=head_size,
                                          color=color,
                                          bound1=bound1,
                                          bound2=bound2,
                                          namespace="single_arrow",
                                          marker_id=0)
        
        self.publisher.publish(marker)
        
class ArrowArray(MarkerArrow):

    def __init__(self, node: Node, topic_name: str):
        super().__init__(node)

        self.publisher = node.create_publisher(MarkerArray, topic_name, 10)
            
    def publish_arrow_array(self, 
                            spacing, 
                            num_grid,
                            shaft_thick,
                            head_size,
                            color,
                            vector_function):
        
        marker_array = MarkerArray()
        id_count = 0

        offset = spacing * (num_grid // 2)

        for i in range(num_grid):
            for j in range(num_grid):
                x = spacing * i - offset
                y = spacing * j - offset
                z = 60.0

                vector_field = vector_function(np.array([[x], [y], [z]]))/100
                vector_field = np.asarray(vector_field).flatten()

                marker = self.create_arrow_marker(shaft_thick=shaft_thick,
                                                  head_size=head_size,
                                                  color=color,
                                                  bound1=np.array([x, y, z]),
                                                  bound2=vector_field,
                                                  namespace="vector_field",
                                                  marker_id=id_count)
                
                marker_array.markers.append(marker)
                id_count += 1

        self.publisher.publish(marker_array)
