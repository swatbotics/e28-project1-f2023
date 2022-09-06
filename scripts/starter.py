#!/usr/bin/env python

import roslib; roslib.load_manifest('project1')
import rospy
import random

from geometry_msgs.msg import Twist
from kobuki_msgs.msg import SensorState

# control at 100Hz
CONTROL_PERIOD = rospy.Duration(0.01)

# react to bumpers for 1 second
BUMPER_DURATION = rospy.Duration(1.0)

# random actions should last 0.5 second
RANDOM_ACTION_DURATION = rospy.Duration(0.5)

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

        # pick out a random action to do when we start driving
        self.pick_random_action()

        # we will stop driving if picked up or about to drive off edge
        # of something
        self.cliff_alert = 0

        # set up subscriber for sensor state for bumpers/cliffs
        rospy.Subscriber('/mobile_base/sensors/core',
                         SensorState, self.sensor_callback)

        # TODO: set up other publishers/subscribers here if necessary

        # set up control timer at 100 Hz
        rospy.Timer(CONTROL_PERIOD, self.control_callback)

        # set up timer for random actions at 2 hz
        rospy.Timer(RANDOM_ACTION_DURATION, self.pick_random_action)

    # called whenever sensor messages are received
    def sensor_callback(self, msg):

        # set cliff alert
        self.cliff_alert = msg.cliff

        # ignore bumper if we are already reacting to it
        if self.state in ['backward', 'turn_left', 'turn_right']:
            return

        # see what we should do next
        next_state = None

        if msg.bumper & SensorState.BUMPER_CENTRE:
            next_state = 'backward'
        elif msg.bumper & SensorState.BUMPER_LEFT:
            next_state = 'turn_right'
        elif msg.bumper & SensorState.BUMPER_RIGHT:
            next_state = 'turn_left'

        # if bumped, go to next state
        if next_state is not None:
            
            self.state = next_state

            # in 1 second, finish this state
            rospy.Timer(BUMPER_DURATION, self.bumper_done, oneshot=True)
                            
    # called when we are done with a bumper reaction
    def bumper_done(self, timer_event=None):

        # if we just backed up, time to turn
        if self.state == 'backward':
            
            # go to turning state
            self.state = 'turn_left'
            # reset again in a second
            rospy.Timer(BUMPER_DURATION, self.bumper_done, oneshot=True)
                        
        else: # we just turned, so go return to wandering
            
            self.state = 'wander'

    # called when it's time to choose a new random wander direction
    def pick_random_action(self, timer_event=None):

        self.wander_action = Twist()
        self.wander_action.linear.x = random.uniform(0.2, 0.3)
        self.wander_action.angular.z = random.uniform(-1.0, 1.0)
    
    # called 100 times per second
    def control_callback(self, timer_event=None):

        # initialize commanded vel to 0, 0
        cmd_vel = Twist()

        # only set commanded velocity to non-zero if not picked up:
        if not self.cliff_alert:

            # state maps straightforwardly to command
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
    
