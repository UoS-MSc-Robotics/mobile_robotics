#! /usr/bin/env python3
"""Program to make the turtlebot3 robot explore the environment."""

import math
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class ExploringController():
    """Exploring Controller Class."""

    def __init__(self):
        """Initialize the Exploring Controller Class."""
        rospy.init_node('exploring_controller_node', disable_signals=True, anonymous=True)

        update_rate = 50
        time_period = 1. / update_rate

        self.linear_speed = 0.26
        self.angular_speed = 1.82
        self.clipping_distance = 3.5
        self.scaling_factor = 402.0

        self.magnitude = 0
        self.target_angle = 0
        self.laser_data = []
        self.full_laser_data = []

        # Subscribers
        rospy.Subscriber("/scan", LaserScan, self.laser_callback)

        # Publishers
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Timers
        rospy.Timer(rospy.Duration(time_period), self.controller_update)

        # Shutdown Function
        rospy.on_shutdown(self.terminate)

    def move_forward(self):
        """Move the robot forward."""
        twist = Twist()
        twist.linear.x = \
            self.linear_speed * self.magnitude / self.scaling_factor
        twist.angular.z = 0
        self.pub.publish(twist)

    def move_backward(self):
        """Move the robot backward."""
        twist = Twist()
        twist.linear.x = \
            -self.linear_speed * self.magnitude / self.scaling_factor
        twist.angular.z = 0
        self.pub.publish(twist)
    
    def turn_left(self):
        """Turn the robot left."""
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = \
            self.angular_speed * self.magnitude / self.scaling_factor
        self.pub.publish(twist)

    def turn_right(self):
        """Turn the robot right."""
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = \
            -self.angular_speed * self.magnitude / self.scaling_factor
        self.pub.publish(twist)

    def stop_moving(self):
        """Stop the robot."""
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        self.pub.publish(twist)

    def laser_callback(self, msg):
        """Callback function for the laser scan."""
        # clip the laser data
        self.laser_data = msg.ranges[0:90] + msg.ranges[270:360]
        self.full_laser_data = msg.ranges
        
        horizontal_component = 0
        vertical_component = 0

        for i in range(len(self.laser_data)):
            if self.laser_data[i] > self.clipping_distance:
                self.laser_data = list(self.laser_data)
                self.laser_data[i] = self.clipping_distance
                self.laser_data = tuple(self.laser_data)

            horizontal_component += self.laser_data[i] * math.cos(math.radians(i))
            vertical_component += self.laser_data[i] * math.sin(math.radians(i))
        
        self.magnitude = math.sqrt(horizontal_component**2 + vertical_component**2)
        self.target_angle = math.degrees(math.atan2(vertical_component, horizontal_component))

        print("Magnitude: ", self.magnitude)
        print("Target Angle: ", self.target_angle)

    def obstacle_avoider(self):
        """Avoid obstacles."""
        # if 85 < self.target_angle < 95:
        #     self.move_forward()
        # if self.target_angle < 85:
        #     self.turn_left()
        # if self.target_angle > 95:
        #     self.turn_right()

        # if object is headon, move backward
        if any((distance < 0.2 for distance in self.laser_data[0:20]) or any(distance < 0.2 for distance in self.laser_data[70:90])):
            self.move_backward()
        # if there is significant obstacles in the indices from 0 to 65, turn right
        elif any(distance < 0.5 for distance in self.laser_data[0:65]):
            self.turn_right()
        # if there is significant obstacles in the indices from 295 to 360, turn left
        elif any(distance < 0.5 for distance in self.laser_data[295:360]):
            self.turn_left()
        else:
            self.move_forward()

    def controller_update(self, _):
        """Update the controller."""
        # Check if the laser data is available
        if self.laser_data:
            self.obstacle_avoider()
        else:
            self.stop_moving()

    def terminate(self):
        """Terminate the node."""
        rospy.logwarn("Shutting down exploring_controller_node.")


def main():
    """Mimic Main Function."""
    try:
        ExploringController()
        rospy.spin()
    except (rospy.exceptions.ROSInitException,
            rospy.exceptions.ROSException, KeyboardInterrupt):
        rospy.signal_shutdown("Done")


if __name__ == '__main__':
    main()
