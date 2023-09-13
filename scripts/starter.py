#!/usr/bin/env python

import roslib; roslib.load_manifest('project1')
import rospy
import random

from geometry_msgs.msg import Twist
from kobuki_msgs.msg import SensorState

# control at 25hz
CONTROL_PERIOD = rospy.Duration(0.04)

# react to bumpers for 1 second
BUMPER_DURATION = rospy.Duration(1.0)

# random actions should last 2.0 second
RANDOM_ACTION_DURATION = rospy.Duration(2.0)

# our controller class
class Controller:

    # called when an object of type Controller is created
    def __init__(self):

        # initialize rospy
        rospy.init_node('starter')

        # set up publisher for commanded velocity
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity',
                                           Twist, queue_size=1)

        # start out in wandering state
        self.state = 'wander'

        # when did we transition to this state
        self.state_start_time = rospy.Time.now()

        # pick out a random action to do when we start driving
        self.pick_random_action()

        # we will stop driving if picked up or about to drive off edge
        # of something
        self.cliff_alert = 0

        # we will change states when bumper is hit but initially no bumper
        self.bumper = 0

        # set up subscriber for sensor state for bumpers/cliffs
        rospy.Subscriber('/mobile_base/sensors/core',
                         SensorState, self.sensor_callback)

        # TODO: set up other publishers/subscribers here if necessary

        # set up control timer at 100 Hz
        rospy.Timer(CONTROL_PERIOD, self.control_callback)

    # called whenever sensor messages are received
    def sensor_callback(self, msg):

        # set cliff alert
        self.cliff_alert = msg.cliff

        # set bumper bitflags
        self.bumper = msg.bumper
                            
    # called when it's time to choose a new random wander direction
    def pick_random_action(self, timer_event=None):

        self.wander_action = Twist()
        self.wander_action.linear.x = random.uniform(0.2, 0.3)
        self.wander_action.angular.z = random.uniform(-1.0, 1.0)

    # called many times per second
    def control_callback(self, timer_event=None):

        # initialize commanded vel to 0, 0
        cmd_vel = Twist()

        # only set commanded velocity to non-zero if not picked up:
        if not self.cliff_alert:

            orig_state = self.state

            time_in_state = rospy.Time.now() - self.state_start_time

            ################################################################################

            # handle state transition
            if self.state == 'wander':

                if self.bumper & SensorState.BUMPER_CENTRE:
                    self.state = 'backward'
                elif self.bumper & SensorState.BUMPER_LEFT:
                    self.state = 'turn_right'
                elif self.bumper & SensorState.BUMPER_RIGHT:
                    self.state = 'turn_left'
                elif time_in_state > RANDOM_ACTION_DURATION:
                    # fake a transition to reset the wander action
                    orig_state = None

            elif self.state == 'backward' and time_in_state > BUMPER_DURATION:

                self.state = 'turn_left'

            elif self.state in ['turn_left', 'turn_right'] and time_in_state > BUMPER_DURATION:

                self.state = 'wander'

            # handle updating state timer and random action
            if self.state != orig_state: # was state changed?
                # if so, record when we entered into it
                self.state_start_time = rospy.Time.now()
                # special case for entering wander state - pick a new random action
                if self.state == 'wander':
                    self.pick_random_action()

            ################################################################################

            # handle state output (control)
            # each state maps straightforwardly to command
            if self.state == 'backward':
                cmd_vel.linear.x = -0.3
            elif self.state == 'turn_left':
                cmd_vel.angular.z = 1.5
            elif self.state == 'turn_right':
                cmd_vel.angular.z = -1.5
            else: # forward
                cmd_vel = self.wander_action

        self.cmd_vel_pub.publish(cmd_vel)
        
    # called by main function below (after init)
    def run(self):
        
        # timers and callbacks are already set up, so just spin.
        # if spin returns we were interrupted by Ctrl+C or shutdown
        rospy.spin()


# main function
if __name__ == '__main__':
    try:
        ctrl = Controller()
        ctrl.run()
    except rospy.ROSInterruptException:
        pass
    
