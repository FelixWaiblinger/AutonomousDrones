#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from state_machine.msg import State

class StateMachine():
    def __init__(self):
        rospy.init_node('state_machine', anonymous=True)

        rospy.Subscriber("drone_1/true_pose", PoseStamped, self.first_drone_callback)
        rospy.Subscriber("drone_2/true_pose", PoseStamped, self.second_drone_callback)
        rospy.Subscriber("drone_1/move_base/current_goal", PoseStamped, self.first_drone_current_goal_callback)
        rospy.Subscriber("drone_2/move_base/current_goal", PoseStamped, self.second_drone_current_goal_callback)

        self.statePub = rospy.Publisher("state", State, queue_size=10)

        self.internal_state = 0
        self.start_time = rospy.get_rostime().secs
        self.drone_1_start = {'x': 2, 'y': -5, 'z': 1.5}
        self.drone_2_start = {'x': 2, 'y': 5, 'z': 2.0}
        self.drone_1_position = PoseStamped()
        self.drone_2_position = PoseStamped()

        self.drone_1_current_goal = PoseStamped()
        self.drone_2_current_goal = PoseStamped()

        self.drone_1_previous_goal = {'x': 5000, 'y': 5000, 'z': 5000}
        self.drone_2_previous_goal = {'x': 5000, 'y': 5000, 'z': 5000}
        self.drone_1_previous_position = {'x': 5000, 'y': 5000, 'z': 5000}
        self.drone_2_previous_position = {'x': 5000, 'y': 5000, 'z': 5000}
        self.previous_goal_update_time = rospy.get_rostime().secs


    def first_drone_callback(self, data):
        #rospy.loginfo(f"first drone: {data}")
        self.drone_1_position = data


    def second_drone_callback(self, data):
        #rospy.loginfo(f"second drone: {data}")
        self.drone_2_position = data


    def first_drone_current_goal_callback(self, data):
        #rospy.loginfo(f"second drone: {data}")
        self.drone_1_current_goal = data


    def second_drone_current_goal_callback(self, data):
        #rospy.loginfo(f"second drone: {data}")
        self.drone_2_current_goal = data


    def initialized(self):
        # false if simulation time is less than 10 seconds
        return rospy.get_rostime().secs > self.start_time + 10

    
    def drones_in_position(self):
        # false if both drones are more then 1m from their target position
        if abs(self.drone_1_position.pose.position.x - self.drone_1_start['x']) > 1\
        or abs(self.drone_1_position.pose.position.y - self.drone_1_start['y']) > 1\
        or abs(self.drone_1_position.pose.position.z - self.drone_1_start['z']) > 1\
        or abs(self.drone_2_position.pose.position.x - self.drone_2_start['x']) > 1\
        or abs(self.drone_2_position.pose.position.y - self.drone_2_start['y']) > 1\
        or abs(self.drone_2_position.pose.position.z - self.drone_2_start['z']) > 1:
            return False
        self.previous_goal_update_time = rospy.get_rostime().secs
        return True


    def map_explored(self):

        # returns true if after 15 sec previous goals for drone 1 and 2 are the same as current goal for drone 1 and 2 and the position of the drones didnt change in the last 15 secs
        
        if rospy.get_rostime().secs > self.previous_goal_update_time + 10: 
            if abs(self.drone_1_current_goal.pose.position.x - self.drone_1_previous_goal['x']) > 0.01\
            or abs(self.drone_1_current_goal.pose.position.y - self.drone_1_previous_goal['y']) > 0.01\
            or abs(self.drone_1_current_goal.pose.position.z - self.drone_1_previous_goal['z']) > 0.01\
            or abs(self.drone_2_current_goal.pose.position.x - self.drone_2_previous_goal['x']) > 0.01\
            or abs(self.drone_2_current_goal.pose.position.y - self.drone_2_previous_goal['y']) > 0.01\
            or abs(self.drone_2_current_goal.pose.position.z - self.drone_2_previous_goal['z']) > 0.01\
            or abs(self.drone_1_position.pose.position.x - self.drone_1_previous_position['x']) > 0.5\
            or abs(self.drone_1_position.pose.position.y - self.drone_1_previous_position['y']) > 0.5\
            or abs(self.drone_1_position.pose.position.z - self.drone_1_previous_position['z']) > 0.5\
            or abs(self.drone_2_position.pose.position.x - self.drone_2_previous_position['x']) > 0.5\
            or abs(self.drone_2_position.pose.position.y - self.drone_2_previous_position['y']) > 0.5\
            or abs(self.drone_2_position.pose.position.z - self.drone_2_previous_position['z']) > 0.5:

                self.drone_1_previous_goal['x'] = self.drone_1_current_goal.pose.position.x
                self.drone_1_previous_goal['y'] = self.drone_1_current_goal.pose.position.y
                self.drone_1_previous_goal['z'] = self.drone_1_current_goal.pose.position.z
                self.drone_2_previous_goal['x'] = self.drone_2_current_goal.pose.position.x
                self.drone_2_previous_goal['y'] = self.drone_2_current_goal.pose.position.y
                self.drone_2_previous_goal['z'] = self.drone_2_current_goal.pose.position.z
                
                self.drone_1_previous_position['x'] = self.drone_1_position.pose.position.x
                self.drone_1_previous_position['y'] = self.drone_1_position.pose.position.y
                self.drone_1_previous_position['z'] = self.drone_1_position.pose.position.z
                self.drone_2_previous_position['x'] = self.drone_2_position.pose.position.x
                self.drone_2_previous_position['y'] = self.drone_2_position.pose.position.y
                self.drone_2_previous_position['z'] = self.drone_2_position.pose.position.z

                self.previous_goal_update_time = rospy.get_rostime().secs
                
                return False
            rospy.loginfo(f"current_position: {self.drone_1_position.pose.position.x}  drone_1_previous_position : {self.drone_1_previous_position['x']}")
            rospy.loginfo(f"Test true")
            return True
        rospy.loginfo(f"current_position: {self.drone_1_position.pose.position.x}  drone_1_previous_position : {self.drone_1_previous_position['x']}")
        return False

    def drones_landed(self):
        # false if both drones are higher than 10cm
        if self.drone_1_position.pose.position.z - self.drone_1_start['z'] > .1\
        or self.drone_2_position.pose.position.z - self.drone_2_start['z'] > .1:
            return False
        
        return True


    def send_state(self, state):
        rospy.loginfo(f"old: {self.internal_state} new: {state}")
        self.internal_state = state
        self.statePub.publish(State(Header(), state))


    def start(self):
        rate = rospy.Rate(10) # 10ms sleep time

        self.statePub.publish(State(Header(), 0))

        while not rospy.is_shutdown():
            # initialize
            if self.internal_state == 0:
                if self.initialized():
                    self.send_state(1)

            # take off to start position
            elif self.internal_state == 1:
                if self.drones_in_position():
                    self.send_state(2)

            # explore
            elif self.internal_state == 2:
                if self.map_explored():
                    self.send_state(3)

            # land at current position
            elif self.internal_state == 3:
                if self.drones_landed():
                    self.send_state(4)

            else:
                break
            rate.sleep()


if __name__=='__main__':
    sm = StateMachine()
    sm.start()
