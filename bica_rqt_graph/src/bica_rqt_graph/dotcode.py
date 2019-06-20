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

import re
import copy

import roslib
import math

import rospy
import pydot
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from bica_msgs.msg import Graph, Edge

import tf

try:
    unicode
    # we're on python2, or the "unicode" function has already been defined elsewhere
except NameError:
    unicode = str
    # we're on python3



class BicaGraphDotcodeGenerator:

    def __init__(self):
        self.listener_ = tf.TransformListener()
        pass

    def generate_dotgraph(
        self,
        bicagraphinst,
        dotcode_factory,
        orientation='LR',
        rank='same',  # None, same, min, max, source, sink
        ranksep=0.2,  # vertical distance between layers
        rankdir='TB',  # direction of layout (TB top > bottom, LR left > right)
        simplify=True,  # remove double edges
        quiet=False,
        unreachable=False):
        """
        See generate_dotcode
        """

        print (bicagraphinst.get_graph())


        # create the graph
        # result = "digraph G {\n
        # rankdir=%(orientation)s;\n%(nodes_str)s\n%(edges_str)s}\n" % vars()
        dotgraph = dotcode_factory.get_graph(
            rank=rank,
            ranksep=ranksep,
            simplify=simplify,
            rankdir=orientation)

        for node in bicagraphinst.get_graph().nodes:
            dotcode_factory.add_node_to_graph(
                dotgraph,
                nodename=node.id,
                nodelabel=node.id,
                shape='ellipse',
                url=node.id)

        for edge in bicagraphinst.get_graph().edges:
            if edge.type == Edge.EDGE_TYPE_STRING:
                dotcode_factory.add_edge_to_graph(
                    dotgraph,
                    edge.source,
                    edge.target,
                    label=edge.string_data,
                    url='%s %s %s '.format(edge.source, edge.target, edge.string_data),
                    penwidth=1,
                    color=[0, 0, 0])
            elif edge.type == Edge.EDGE_TYPE_DOUBLE:
                dotcode_factory.add_edge_to_graph(
                    dotgraph,
                    edge.source,
                    edge.target,
                    label=edge.double_data,
                    url='%s %s %ld '.format(edge.source, edge.target, edge.double_data),
                    penwidth=1,
                    color=[0, 0, 0])
            elif edge.type == Edge.EDGE_TYPE_TF:
                try:
                    (pose,orientation_q) = self.listener_.lookupTransform(edge.source, edge.target, rospy.Time(0))
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue

                # (roll, pitch, yaw) = euler_from_quaternion (orientation_q)
                yaw = math.atan2(pose[1], pose[0])
                pitch = math.atan2(pose[0], pose[2])
                tf_label = '{0:.2f} {1:.2f} {2:.2f} ({3:.2f} {4:.2f})'.format(pose[0], pose[1], pose[2], yaw, pitch)
                dotcode_factory.add_edge_to_graph(
                    dotgraph,
                    edge.source,
                    edge.target,
                    label=tf_label,
                    url='%s %s %s'.format(edge.source, edge.target, "tf"),
                    penwidth=1,
                    color=[0, 0, 255])
            else:
                print("Error: Edge type unknown")

        return dotgraph

    def generate_dotcode(
        self,
        bicagraphinst,
        dotcode_factory,
        orientation='LR',
        rank='same',  # None, same, min, max, source, sink
        ranksep=0.2,  # vertical distance between layers
        rankdir='TB',  # direction of layout (TB top > bottom, LR left > right)
        simplify=False,  # remove double edges
        quiet=False,
        unreachable=False):
        """
        @param bicagraphinst: BicaGraph instance
        @param ns_filter: nodename filter
        @type  ns_filter: string
        @param topic_filter: topicname filter
        @type  ns_filter: string
        @param graph_mode str: NODE_NODE_GRAPH | NODE_TOPIC_GRAPH | NODE_TOPIC_ALL_GRAPH
        @type  graph_mode: str
        @param orientation: rankdir value (see ORIENTATIONS dict)
        @type  dotcode_factory: object
        @param dotcode_factory: abstract factory manipulating dot language objects
        @param hide_single_connection_topics: if true remove topics with just one connection
        @param hide_dead_end_topics: if true remove topics with publishers only
        @param cluster_namespaces_level: if > 0 places box around members of same namespace
               (TODO: multiple namespace layers)
        @param accumulate_actions: if true each 5 action topic graph nodes are shown as single graph node
        @return: dotcode generated from graph singleton
        @rtype: str
        """
        dotgraph = self.generate_dotgraph(
            bicagraphinst=bicagraphinst,
            dotcode_factory=dotcode_factory,
            orientation=orientation,
            rank=rank,
            ranksep=ranksep,
            rankdir=rankdir,
            simplify=simplify,
            quiet=quiet,
            unreachable=unreachable)
        dotcode = dotcode_factory.create_dot(dotgraph)
        return dotcode
