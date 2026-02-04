#!/usr/bin/env python3

## Header #########################################################################################
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Header
from geometry_msgs.msg import Point
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
    
    def __init__(self, node: Node, topic_name: str):
        self.node = node
        self.topic_name = topic_name
        self.publisher = self.node.create_publisher(Marker, self.topic_name, 10)
    
    def _make_Header(self) -> Header:
        return Header(frame_id="map", stamp=self.node.get_clock().now().to_msg())

    def _create_Marker(self, marker_type: int, marker_action: int):
        self.marker = Marker()
        self.marker.header = self._make_Header()
        self.marker.type = marker_type
        self.marker.action = marker_action

    def _set_MarkerStyle(
            self,     
            marker_scaleX: float, 
            marker_scaleY: float,
            marker_scaleZ: float,
            marker_transparency: float,
            marker_color: np.ndarray,
            ):

        self.marker.scale.x = marker_scaleX
        self.marker.scale.y = marker_scaleY
        self.marker.scale.z = marker_scaleZ
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
        
        # self.publisher = self.node.create_publisher(Marker, self.topic_name, 10)
        self.marker.ns = "single_point"
        self.marker.id = 0

        self._set_MarkerStyle(
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
            shaft_diameter: float, 
            head_diameter: float,
            head_length: float, 
            color: np.ndarray,
            position: np.ndarray,
            vector: np.ndarray,
            transparency: float = 1.0,                                 # transparency, 1.0 = opaque
            ):
        
        # self.publisher = self.node.create_publisher(Marker, self.topic_name, 10)
        self.marker.ns = "single_arrow"
        self.marker.id = 0

        self._set_MarkerStyle(
            marker_scaleX=shaft_diameter,
            marker_scaleY=head_diameter,
            marker_scaleZ=head_length,
            marker_transparency=transparency,
            marker_color=color,
            )

        pos = position.flatten()
        vec = vector.flatten()
        norm = np.linalg.norm(vec)

        if norm > 1e-9:
            vec = vec / norm
        else:
            vec = vec

        scale = 1000.0 * norm                               # scale factor for better visualization

        start = Point()
        start.x = float(pos[0])
        start.y = float(pos[1])
        start.z = float(pos[2])

        end = Point()
        end.x = float(pos[0] + scale * vec[0])
        end.y = float(pos[1] + scale * vec[1])
        end.z = float(pos[2] + scale * vec[2])

        self.marker.points = [start, end]
        self.publisher.publish(self.marker)

    # def publish_ArrowArray(
    #         self,
    #         spacing: float,
    #         number_grid: int,
    #         shaft_diameter: float,
    #         head_diameter: float,
    #         head_length: float,
    #         color: np.ndarray,
    #         vector_function,
    #         ):       
    
        
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
