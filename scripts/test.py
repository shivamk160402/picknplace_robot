#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState


def callback_arm(state_info):
    # rospy.loginfo(rospy.get_caller_id() + 'I heard %s', state_info.process_value)
    global state , a
    state = round(state_info.process_value,1)
    if state > a:
        print('you entered value less than process_value',state)
        rospy.loginfo('publsihing data...')
        pub.publish(a)

    elif state < a:
        print('you entered value greater than process_value',state)
        rospy.loginfo('publsihing data...')
        pub.publish(a)

    elif state == a:
        a = eval(input('your goal is achived please enter next goal to achive'))
        a = float(a)
        pub.publish(a)

def callback_gripper(state_info):
    # rospy.loginfo(rospy.get_caller_id() + 'I heard %s', state_info.process_value)
    global state , a, j
    state = round(state_info.process_value,1)
    if state > a:
        print('you entered value less than process_value',state)
        rospy.loginfo('publsihing data...')
        pub.publish(a)

    elif state < a:
        print('you entered value greater than process_value',state)
        rospy.loginfo('publsihing data...')
        pub.publish(a)

    elif state == a:
        a = input('your goal is achived please enter next goal to achive')
        a = float(a)
        pub.publish(a)



try:
    j = input("enter the name of joint:")
    rospy.init_node('cmd_state_and_ctrl_rev27_rev29', anonymous=True)
    if j == "arm":
        a = eval(input('enter the valuue to publish to rev 27'))
        a = float(a)
        state = 0
        sub = rospy.Subscriber('/pnp_bot/Rev27_position_controller/state', JointControllerState, callback_arm)
        pub = rospy.Publisher('/pnp_bot/Rev27_position_controller/command', Float64 , queue_size=10)
        rate = rospy.Rate(2) # 10hz

        rospy.spin()

    elif j == "gripper":
        a = eval(input('enter the valuue to publish to rev 29'))
        a = float(a)
        state = 0
        sub = rospy.Subscriber('/pnp_bot/Rev29_position_controller/state', JointControllerState, callback_gripper)
        pub = rospy.Publisher('/pnp_bot/Rev29_position_controller/command', Float64 , queue_size=10)
        rate = rospy.Rate(2) # 10hz
        
        rospy.spin()
    
    elif j == "exit":
        print("exiting the joint")
        rospy.signal_shutdown()


except rospy.ROSInterruptException:
        pass
