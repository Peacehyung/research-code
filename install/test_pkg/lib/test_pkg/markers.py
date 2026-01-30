#!/usr/bin/env python3

## Header #########################################################################################
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

## Marker colors ##################################################################################
class MarkerColors:

    def __init__(self):
        
        self.red = np.array([1.0, 0.0, 0.0])
        self.green = np.array([0.0, 1.0, 0.0])
        self.blue = np.array([0.0, 0.0, 1.0])
        self.yellow = np.array([1.0, 1.0, 0.0])
        self.white = np.array([1.0, 1.0, 1.0])
        self.black = np.array([0.0, 0.0, 0.0])

## Point marker ###################################################################################
class MarkerPoint:

    def __init__(self, node: Node, topic_name: str):
        
        self.node = node
        self.publisher = node.create_publisher(Marker, topic_name, 10)

    def _make_Header(self) -> Header:
        return Header(frame_id="map", stamp=self.node.get_clock().now().to_msg())

    def publish_Point(self, radius: float, color: np.ndarray, position: np.ndarray):

        marker = Marker()
        marker.header = self._make_Header()
        marker.ns = "single_point"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = radius
        marker.scale.y = radius
        marker.scale.z = radius
        marker.color.a = 1.0                                           # transparency, 1.0 = opaque
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]

        marker.pose.position.x = float(position[0])
        marker.pose.position.y = float(position[1])
        marker.pose.position.z = float(position[2])

        self.publisher.publish(marker)

## Arrow marker family ############################################################################
class MarkerArrow:

    def __init__(self, node: Node):
        self.node = node

    def _make_Header(self) -> Header:
        return Header(frame_id="map", stamp=self.node.get_clock().now().to_msg())

    def create_ArrowMarker(
            self,
            namespace: str,
            marker_id: int,                            
            shaft_thick: float,                                             # arrow shaft thickness
            head_size: float,                                                      # arrowhead size
            color: np.ndarray,
            bound1: np.ndarray,
            bound2: np.ndarray,
            ) -> Marker:
        
        marker = Marker()
        marker.header = self._make_Header()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x = shaft_thick         
        marker.scale.y = head_size                  
        marker.color.a = 1.0                          
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]

        initial = Marker().pose.position
        initial.x, initial.y, initial.z = (float(bound1[0]), float(bound1[1]), float(bound1[2]))

        terminal = Marker().pose.position
        terminal.x = float(bound1[0] + bound2[0])
        terminal.y = float(bound1[1] + bound2[1])
        terminal.z = float(bound1[2] + bound2[2])
        
        marker.points.append(initial)                # initial point
        marker.points.append(terminal)              # terminal point

        return marker

class SingleArrow(MarkerArrow):
    
    def __init__(self, node: Node, topic_name: str):
        super().__init__(node)

        self.publisher = node.create_publisher(Marker, topic_name, 10)
        
    def publish_single_arrow(
            self,
            shaft_thick: float,
            head_size: float,
            color: np.ndarray,
            bound1: np.ndarray,
            bound2: np.ndarray,
            ):
        
        marker = self.create_ArrowMarker(
            shaft_thick=shaft_thick,
            head_size=head_size,
            color=color,
            bound1=bound1,
            bound2=bound2,
            namespace="single_arrow",
            marker_id=0
            )
        
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
        if spacing <= 0:
            raise ValueError("spacing must be > 0")
        if num_grid <= 0:
            raise ValueError("num_grid must be > 0")
        self._validate_vec3(color, "color")
        if vector_function is None:
            raise ValueError("vector_function must be provided")
        
        marker_array = MarkerArray()
        id_count = 0

        offset = spacing * (num_grid // 2)

        for i in range(num_grid):
            for j in range(num_grid):
                x = spacing * i - offset
                y = spacing * j - offset
                z = 60.0

                vector_field = vector_function(np.array([[x], [y], [z]])) / 100
                vector_field = np.asarray(vector_field).flatten()
                self._validate_vec3(vector_field, "vector_field")

                marker = self.create_ArrowMarker(shaft_thick=shaft_thick,
                                                  head_size=head_size,
                                                  color=color,
                                                  bound1=np.array([x, y, z]),
                                                  bound2=vector_field,
                                                  namespace="vector_field",
                                                  marker_id=id_count)
                
                marker_array.markers.append(marker)
                id_count += 1

        self.publisher.publish(marker_array)
