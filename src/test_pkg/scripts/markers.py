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
class MarkerProperties:
    
    def __init__(
            self,
            node: Node,
            topic_name: str,
            ):

        self.node = node
        self.publisher = node.create_publisher(Marker, topic_name, 10)
        self.topic_name = topic_name
    
    def _make_Header(self) -> Header:
        return Header(frame_id="map", stamp=self.node.get_clock().now().to_msg())

    def _create_Marker(self, marker_type: int, marker_action: int):
        self.marker = Marker()
        self.marker.header = self._make_Header()
        self.marker.type = marker_type
        self.marker.action = marker_action

    def _set_MarkerStyle(
            self,
            marker_namespace: str,
            marker_id: int,            
            marker_scaleX: float, 
            marker_scaleY: float,
            marker_scaleZ: float,
            marker_transparency: float,
            marker_color: np.ndarray,
            ):
        
        self.marker.ns = marker_namespace
        self.marker.id = marker_id
        self.marker.scale.x = marker_scaleX
        self.marker.scale.y = marker_scaleY
        self.marker.scale.z = 0.0 if marker_scaleZ is None else marker_scaleZ
        self.marker.color.a = marker_transparency
        self.marker.color.r = float(marker_color[0])
        self.marker.color.g = float(marker_color[1])
        self.marker.color.b = float(marker_color[2])

## Point marker ###################################################################################
class MarkerPoint(MarkerProperties):

    def __init__(self, node: Node, topic_name: str):
        
        super().__init__(node, topic_name)

        self._create_Marker(marker_type=Marker.SPHERE, marker_action=Marker.ADD)            
        
    def publish_SinglePoint(
            self,
            radius: float, 
            color: np.ndarray, 
            position: np.ndarray,
            transparency: float = 1.0,                                 # transparency, 1.0 = opaque
            ):

        self._set_MarkerStyle(
            marker_namespace="single_point",
            marker_id=0,
            marker_scaleX=radius,
            marker_scaleY=radius,
            marker_scaleZ=radius,
            marker_transparency=transparency,
            marker_color=color,
            )

        pos = position.flatten()
        self.marker.pose.position.x = float(pos[0])
        self.marker.pose.position.y = float(pos[1])
        self.marker.pose.position.z = float(pos[2])

        self.publisher.publish(self.marker)

## Arrow marker family ############################################################################
class MarkerArrow(MarkerProperties):

    def __init__(self, node: Node, topic_name: str):
        
        super().__init__(node, topic_name)

        self._create_Marker(marker_type=Marker.ARROW, marker_action=Marker.ADD)

    def publish_SingleArrow(
            self,
            shaft_thick: float, 
            head_size: float, 
            color: np.ndarray,
            position: np.ndarray,
            orientation: np.ndarray,
            transparency: float = 1.0,                                 # transparency, 1.0 = opaque
            ):

        self._set_MarkerStyle(
            marker_namespace="single_arrow",
            marker_id=0,
            marker_scaleX=shaft_thick,
            marker_scaleY=head_size,
            marker_scaleZ=None,
            marker_transparency=transparency,
            marker_color=color,
            )

        pos = self.marker.pose.position
        pos.x, pos.y, pos.z = (float(position[0]), float(position[1]), float(position[2]))

        ori = self.marker.pose.position
        ori.x = float(position[0] + orientation[0])
        ori.y = float(position[1] + orientation[1])
        ori.z = float(position[2] + orientation[2])

        self.marker.points.append(pos)                # initial point
        self.marker.points.append(ori)                # terminal point

        self.publisher.publish(self.marker)

        self.marker.pose.position.x = float(position[0])
        self.marker.pose.position.y = float(position[1])
        self.marker.pose.position.z = float(position[2])

        self.publisher.publish(self.marker)

#     def create_ArrowMarker(
#             self,
#             namespace: str,
#             marker_id: int,                            
#             shaft_thick: float,                                             # arrow shaft thickness
#             head_size: float,                                                      # arrowhead size
#             color: np.ndarray,
#             bound1: np.ndarray,
#             bound2: np.ndarray,
#             ) -> Marker:
        
#         marker = Marker()
#         marker.header = self._make_Header()
#         marker.ns = namespace
#         marker.id = marker_id
#         marker.type = Marker.ARROW
#         marker.action = Marker.ADD
#         marker.scale.x = shaft_thick         
#         marker.scale.y = head_size                  
#         marker.color.a = 1.0                          
#         marker.color.r = color[0]
#         marker.color.g = color[1]
#         marker.color.b = color[2]

#         initial = Marker().pose.position
#         initial.x, initial.y, initial.z = (float(bound1[0]), float(bound1[1]), float(bound1[2]))

#         terminal = Marker().pose.position
#         terminal.x = float(bound1[0] + bound2[0])
#         terminal.y = float(bound1[1] + bound2[1])
#         terminal.z = float(bound1[2] + bound2[2])
        
#         marker.points.append(initial)                # initial point
#         marker.points.append(terminal)              # terminal point

#         return marker

# class SingleArrow(MarkerArrow):
    
#     def __init__(self, node: Node, topic_name: str):
#         super().__init__(node)

#         self.publisher = node.create_publisher(Marker, topic_name, 10)
        
#     def publish_single_arrow(
#             self,
#             shaft_thick: float,
#             head_size: float,
#             color: np.ndarray,
#             bound1: np.ndarray,
#             bound2: np.ndarray,
#             ):
        
#         marker = self.create_ArrowMarker(
#             shaft_thick=shaft_thick,
#             head_size=head_size,
#             color=color,
#             bound1=bound1,
#             bound2=bound2,
#             namespace="single_arrow",
#             marker_id=0
#             )
        
#         self.publisher.publish(marker)
        
# class ArrowArray(MarkerArrow):

#     def __init__(self, node: Node, topic_name: str):
#         super().__init__(node)

#         self.publisher = node.create_publisher(MarkerArray, topic_name, 10)
            
#     def publish_arrow_array(self, 
#                             spacing, 
#                             num_grid,
#                             shaft_thick,
#                             head_size,
#                             color,
#                             vector_function):
#         # if spacing <= 0:
#         #     raise ValueError("spacing must be > 0")
#         # if num_grid <= 0:
#         #     raise ValueError("num_grid must be > 0")
#         # self._validate_vec3(color, "color")
#         # if vector_function is None:
#         #     raise ValueError("vector_function must be provided")
        
#         marker_array = MarkerArray()
#         id_count = 0

#         offset = spacing * (num_grid // 2)

#         for i in range(num_grid):
#             for j in range(num_grid):
#                 x = spacing * i - offset
#                 y = spacing * j - offset
#                 z = 0.0

#                 vector_field = vector_function(np.array([[x], [y], [z]]))
#                 vector_field = np.asarray(vector_field).flatten()
#                 # self._validate_vec3(vector_field, "vector_field")

#                 marker = self.create_ArrowMarker(shaft_thick=shaft_thick,
#                                                   head_size=head_size,
#                                                   color=color,
#                                                   bound1=np.array([x, y, z]),
#                                                   bound2=vector_field,
#                                                   namespace="vector_field",
#                                                   marker_id=id_count)
                
#                 marker_array.markers.append(marker)
#                 id_count += 1

#         self.publisher.publish(marker_array)
