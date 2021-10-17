#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Pose, Quaternion
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt
from tf.transformations import quaternion_from_euler

class nexus:

    def __init__(self):
        # Creates a node with name 'turtlebot_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('nexus_control', anonymous=True)
        # Define Params
        self.x_goal = rospy.get_param("~x")
        self.y_goal = rospy.get_param("~y")
        self.f_goal = rospy.get_param("~f")
        self.q_goal = quaternion_from_euler(0,0,self.f_goal)
        
        # Publisher which will publish to the topic '/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # A subscriber to the topic '/odom'. self.update_pose is called
        # when a message of type Odometry is received.
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.update_odom)

        self.odom = Odometry()
        self.rate = rospy.Rate(10)

    def update_odom(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.odom = data
        self.x_pose = round(self.odom.pose.position.x, 2)
        self.y_pose = round(self.odom.pose.position.y, 2)
        self.z_pose = round(self.odom.pose.position.z, 2)
        self.qx_orientation = round(self.odom.pose.orientation.x, 2)
        self.qy_orientation = round(self.odom.pose.orientation.y, 2)
        self.qz_orientation = round(self.odom.pose.orientation.z, 2)
        self.qw_orientation = round(self.odom.pose.orientation.w, 2)
        print(self.odom)
        print(self.x_pose)
        print(self.qw_orientation)

    def euclidean_distance(self, goal_odom):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((self.x_goal - self.x_pose), 2) +
                    pow((self.y_goal - self.y_pose), 2))

    def linear_vel(self, goal_pose, constant=1):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant=6):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * (self.steering_angle(goal_pose) - self.pose.theta)

    def move2pose(self):
        """Moves the turtle to the goal."""
        goal_odom = Odometry()

        # Get the input from the user.
        goal_odom.odom.pose.position.x = self.x_goal
        goal_odom.odom.pose.position.y = self.y_goal
        goal_odom.odom.pose.orientation.x = self.q_goal[1]
        goal_odom.odom.pose.orientation.y = self.q_goal[2]
        goal_odom.odom.pose.orientation.z= self.q_goal[3]
        goal_odom.odom.pose.orientation.w= self.q_goal[4]

        print(goal_odom)
        # Please, insert a number slightly greater than 0 (e.g. 0.01).
        distance_tolerance = 0.01

        vel_msg = Twist()

        """ while self.euclidean_distance(goal_pose) >= distance_tolerance:

            # Porportional controller.
            # https://en.wikipedia.org/wiki/Proportional_control

            # Linear velocity in the x-axis.
            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goal_pose)

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

            # Publish at the desired rate.
            self.rate.sleep() """

        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        # If we press control + C, the node will stop.
        rospy.spin()

if __name__ == '__main__':
    try:
        go = nexus()
        go.move2pose()
    except rospy.ROSInterruptException:
        pass