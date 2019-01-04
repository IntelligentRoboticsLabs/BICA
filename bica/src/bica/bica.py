#!/usr/bin/env python

from std_srvs.srv import *
from sets import Set
import rospy
import rosnode

class Component:

    def __init__(self, rate):
        self.active_ = False
        self.activations_ = Set([])

        self.act_sr_ = rospy.Service(rospy.get_name()+"/activate", Empty, self.activateCallback)
        self.act_sr_ = rospy.Service(rospy.get_name()+"/deactivate", Empty, self.deActivateCallback)
        #self.activators_sub_ = {}

        self.rate_ = rospy.Rate(rate)

    def activateCallback(self, req):

        self.activations_.add(req._connection_header['callerid'])

        return []
        #self.activators_sub_[id] = rospy.Subscriber("/"+id+"/active", Empty, self.activeCB)})

    #def activeCB(self, msg):

    def activateCode(self):
        pass

    def deActivateCode(self):
        pass

    def deActivateCallback(self, req):

        self.activations_.remove(req._connection_header['callerid'])

        return []
    #def is_component_active(self, node):
    #
    #    try:
    #        act_srv = rospy.ServiceProxy("/"+node+"/activate", Empty)
    #        resp = act_srv()
    #    except rospy.ServiceException, e:
    #        print "Service call failed: %s"%e

    def updateCallerList(self):

        nodes_actives = Set(rosnode.get_node_names())

        for a in Set(self.activations_):
            if a not in nodes_actives:
                self.activations_.remove(a)

        #for a in self.activations_:
        #    if not is_component_active(a):
        #        self.activations_.remove(a)

        if not self.active_ and len(self.activations_) > 0:
            self.active_ = True
            self.activateCode()

        if  self.active_ and len(self.activations_) == 0:
            self.deActivateCode()
            self.active_ = False


    def ok(self):

        self.updateCallerList()
        if self.active_:
            self.step()

        self.rate_.sleep()
        return not rospy.is_shutdown()
