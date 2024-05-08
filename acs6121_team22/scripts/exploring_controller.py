#! /usr/bin/env python3
"""Program to make the turtlebot3 robot explore the environment."""

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
        self.full_laser_data = []

        self.clipping_distance = 3.5
        self.backward_threshold = 0.25
        self.turning_threshold = 0.5

        # Subscribers
        rospy.Subscriber("/scan", LaserScan, self.laser_callback)

        # Publishers
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10, latch=True)

        # Timers
        rospy.Timer(rospy.Duration(time_period), self.controller_update)

        # Shutdown Function
        rospy.on_shutdown(self.terminate)

    def move_forward(self):
        """Move the robot forward."""
        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = 0
        self.pub.publish(twist)

    def move_backward(self):
        """Move the robot backward."""
        twist = Twist()
        twist.linear.x = -self.linear_speed
        twist.angular.z = 0
        self.pub.publish(twist)

    def turn_smooth_left(self):
        """Turn the robot smooth left."""
        twist = Twist()
        twist.linear.x = self.linear_speed / 2
        twist.angular.z = self.angular_speed
        self.pub.publish(twist)

    def turn_smooth_right(self):
        """Turn the robot smooth right."""
        twist = Twist()
        twist.linear.x = self.linear_speed / 2
        twist.angular.z = -self.angular_speed
        self.pub.publish(twist)

    def turn_hard_left(self):
        """Turn the robot hard left."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = self.angular_speed
        self.pub.publish(twist)

    def turn_hard_right(self):
        """Turn the robot hard right."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = -self.angular_speed
        self.pub.publish(twist)

    def stop_moving(self):
        """Stop the robot."""
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        self.pub.publish(twist)

    def laser_callback(self, msg):
        """Callback function for the laser scan."""
        self.full_laser_data = msg.ranges

        for i, _ in enumerate(self.full_laser_data):
            if self.full_laser_data[i] == 0:
                self.full_laser_data = list(self.full_laser_data)
                self.full_laser_data[i] = self.clipping_distance
                self.full_laser_data = tuple(self.full_laser_data)

    def obstacle_avoider(self):
        """Avoid obstacles."""

        # if object is headon, move backward
        if any((distance < self.backward_threshold for distance in self.full_laser_data[0:20]) or \
               any(distance < self.backward_threshold for distance in self.full_laser_data[70:90])):
            self.move_backward()
        # if there is significant obstacles in the indices from 0 to 40, turn right
        elif (any(distance < self.turning_threshold for distance in self.full_laser_data[0:40])) and \
            not (any(distance < self.turning_threshold for distance in self.full_laser_data[320:360])):
            if any(distance < self.turning_threshold/2 for distance in self.full_laser_data[0:40]):
                self.turn_hard_right()
            else:
                self.turn_smooth_right()
        # if there is significant obstacles in the indices from 320 to 360, turn left
        elif (any(distance < self.turning_threshold for distance in self.full_laser_data[320:360])) and \
            not (any(distance < self.turning_threshold for distance in self.full_laser_data[0:40])):
            if any(distance < self.turning_threshold/2 for distance in self.full_laser_data[320:360]):
                self.turn_hard_left()
            else:
                self.turn_smooth_left()
        elif any(distance < self.turning_threshold for distance in self.full_laser_data[0:40]):
            if any(distance < self.turning_threshold/2 for distance in self.full_laser_data[0:40]):
                self.turn_hard_right()
            else:
                self.turn_smooth_right()
        # if there is significant obstacles in the indices from 320 to 360, turn left
        elif any(distance < self.turning_threshold for distance in self.full_laser_data[320:360]):
            if any(distance < self.turning_threshold/2 for distance in self.full_laser_data[320:360]):
                self.turn_hard_left()
            else:
                self.turn_smooth_left()
        else:
            self.move_forward()

    def controller_update(self, _):
        """Update the controller."""
        # Check if the laser data is available
        if self.full_laser_data:
            self.obstacle_avoider()
            pass
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
