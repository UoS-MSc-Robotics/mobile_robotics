#! /usr/bin/env python3
"""Program to make the turtlebot3 robot explore the environment."""

import rospy
from std_msgs.msg import String


class ExploringController():
    """Exploring Controller Class."""

    def __init__(self):
        """Initialize the Exploring Controller Class."""
        rospy.init_node('exploring_controller_node')

        update_rate = 50
        time_period = 1. / update_rate

        # Subscribers
        rospy.Subscriber("/chatter", String, self.chatter_callback)

        # Publishers
        self.pub = rospy.Publisher('/chatter', String, queue_size=10)

        # Timers
        rospy.Timer(rospy.Duration(time_period), self.chatter_update)

        # Shutdown Function
        rospy.on_shutdown(self.terminate)

    def chatter_callback(self, msg):
        """Listen for the chatter topic."""
        rospy.loginfo(f"Subscribing: {msg.data}")

    def chatter_update(self, _):
        """Update the chatter topic."""
        chatter_msg = String()
        chatter_msg.data = f"Hello World: {int(rospy.get_time())}"
        self.pub.publish(chatter_msg)

        # Display the message on the console
        rospy.loginfo(f"Publishing: {chatter_msg.data}")

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
