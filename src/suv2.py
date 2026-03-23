#!/usr/bin/env python3
import rospy
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Twist
import math

class MovingBoxController:
    def __init__(self):
        rospy.init_node('move_box_cmd_vel')


        self.model_name = rospy.get_param("~model_name", "suv")


        self.start_x = rospy.get_param("~start_x", 28.24)
        self.start_y = rospy.get_param("~start_y", -0.72)
        self.start_z = rospy.get_param("~start_z", 0.1)


        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)


        self.state = ModelState()
        self.state.model_name = self.model_name
        self.state.pose.position.x = self.start_x
        self.state.pose.position.y = self.start_y
        self.state.pose.position.z = self.start_z
        yaw = 2*math.pi
        self.state.pose.orientation.x = 0.0
        self.state.pose.orientation.y = 0.0
        self.state.pose.orientation.z = math.sin(0.785)
        self.state.pose.orientation.w = math.cos(0.785)
        self.state.reference_frame = 'world'
        self.state.twist.linear.x = 0.0

        self.cmd_vel_x = 0.0  


        rospy.Subscriber("/engel_cmd_vel", Twist, self.cmd_vel_callback)

        self.rate = rospy.Rate(50)  # 10 Hz

    def cmd_vel_callback(self, msg):

        self.cmd_vel_x = msg.linear.x

    def update_start_position_if_changed(self):

        new_x = rospy.get_param("~start_x", self.start_x)
        new_y = rospy.get_param("~start_y", self.start_y)
        new_z = rospy.get_param("~start_z", self.start_z)

        if (new_x, new_y, new_z) != (self.start_x, self.start_y, self.start_z):
            rospy.loginfo(f"Başlangıç konumu güncellendi: ({new_x}, {new_y}, {new_z})")
            self.start_x, self.start_y, self.start_z = new_x, new_y, new_z
            self.state.pose.position.x = new_x
            self.state.pose.position.y = new_y
            self.state.pose.position.z = new_z

    def run(self):
        while not rospy.is_shutdown():

            self.update_start_position_if_changed()


            self.state.twist.linear.x = self.cmd_vel_x
            self.state.pose.position.y += self.cmd_vel_x * 0.1  

            try:
                self.set_state(self.state)
            except rospy.ServiceException as e:
                rospy.logerr(f"SetModelState çağrısı başarısız: {e}")

            self.rate.sleep()

if __name__ == "__main__":
    controller = MovingBoxController()
    controller.run()
