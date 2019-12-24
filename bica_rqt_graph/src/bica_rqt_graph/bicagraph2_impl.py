#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2019, Intelligent Robotics Core S.L.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Francisco Martin Rico - fmrico at gmail.com


import rclpy

import tf2_py
import tf2_ros

from rclpy.node import Node
from rclpy.qos import InvalidQoSProfileException
from rclpy.qos import qos_profile_system_default
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSLivelinessPolicy
from rclpy.qos import QoSPresetProfiles
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy

from bica_msgs.msg import GraphUpdate

class GraphNode:
    def __init__(self, node_str):
        self.name = node_str.split('::')[1]
        self.type = node_str.split('::')[2]
    
    def __eq__(self, o):
        return self.name == o.name  # and self.type == o.type
    
    def __repr__(self):
        return 'node::' + self.name + "::" + self.type

class GraphEdge:
    def __init__(self, node_str):
        connection = node_str.split('::')[1]
        self.source = connection.split('->')[0]
        self.target = connection.split('->')[1]
        self.content = node_str.split('::')[2]
        self.type = node_str.split('::')[3]

    def __eq__(self, o):
        return self.source == o.source and self.target == o.target and self.type == o.type and self.content == o.content 

    def __repr__(self):
        return 'edge::' + self.source + '->' + self.target + '::' + self.content + '::' + self.type

class BicaGraphImpl(Node):

    def __init__(self):
        super().__init__('rqt_bica_graph')
        
        self.initialized = self.count_publishers('/graph_updates') == 0
        
        self.update_sub = self.create_subscription(
            GraphUpdate,
            '/graph_updates',
            self.graph_update_callback,
            qos_profile=QoSProfile(
                history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                depth=100,
                reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE)
            )
        
        self.sync_update_sub = self.create_subscription(
            GraphUpdate,
            '/graph_updates_sync',
            self.graph_sync_callback,
            qos_profile=QoSProfile(
                history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                depth=100,
                reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE)
            )
       
        self.graph_sync_pub = self.create_publisher(GraphUpdate,
            '/graph_updates_sync', 
            qos_profile=QoSProfile(
                history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                depth=100,
                reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE)
            )

        if not self.initialized:
            msg = GraphUpdate()
            msg.operation_type = 3  # msg.REQSYNC
            self.graph_sync_pub.publish(msg)

        self.nodes = []
        self.edges = []
        self.seq = 0

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, node=self)

    def __repr__(self):
        ret = "Nodes: " + str(len(self.nodes)) + '\n'
        for i in self.nodes:
            ret = ret + str(i) + '\n'
        ret = ret + "Edges: " + str(len(self.edges)) + '\n'
        for i in self.edges:
            ret = ret + str(i) + '\n'
        return ret  
    
    def init_graph(self, msg):
        self.nodes = []
        self.edges = []
        self.seq = msg.seq
        self.initialized = True
        for element in msg.object.splitlines():
            if element[0:4] == "node":
                self.nodes.append(GraphNode(element))
            if element[0:4] == "edge":
                self.edges.append(GraphEdge(element))

    def add_node(self, msg):
        self.nodes.append(GraphNode(msg.object))
        self.seq = msg.seq

    def remove_node(self, msg):
        node_to_remove = GraphNode(msg.object)
        self.nodes.remove(node_to_remove)
        self.seq = msg.seq

        new_edges = [x for x in self.edges if x.source != node_to_remove.name and x.target != node_to_remove.name] 
        self.edges = new_edges

    def add_edge(self, msg):
        self.edges.append(GraphEdge(msg.object))
        self.seq = msg.seq

    def remove_edge(self, msg):
        edge_to_remove = GraphEdge(msg.object)
        self.edges.remove(edge_to_remove)
        self.seq = msg.seq

    def graph_sync_callback(self, msg):
        # self.get_logger().info('I heard: a new graph or update ' + str(msg.operation_type) + ' ' + str(self.initialized))
        
        # REQSYNC
        if msg.operation_type == 3 and self.initialized: # msg.REQSYNC
            print("REQSYNC")
            msg = GraphUpdate()
            msg.operation_type = 2  # msg.SYNC
            msg.element_type = 2
            msg.node_id = self.get_name()
            msg.seq = self.seq
            msg.object = str(self)
            self.graph_sync_pub.publish(msg)

        # SYNC
        if msg.operation_type == 2 and not self.initialized:
            print("SYNC")
            self.init_graph(msg)
        
    def graph_update_callback(self, msg):
        self.get_logger().info('I heard: a new graph or update')

        # ADD NODE
        if msg.operation_type == 0 and msg.element_type == 0 and self.initialized:
            self.add_node(msg)
        
        # ADD EDGE
        if msg.operation_type == 0 and msg.element_type == 1 and self.initialized:
            self.add_edge(msg)

        # REMOVE NODE
        if msg.operation_type == 1 and msg.element_type == 0 and self.initialized:
            self.remove_node(msg)

        # REMOVE EDGE
        if msg.operation_type == 1 and msg.element_type == 1 and self.initialized:
            self.remove_edge(msg)
