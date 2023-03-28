# Author: Alexander Schperberg, aschperb@gmail.com
# Date: 2023-03-27
import numpy as np
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Vector3

#################################################################################################################
# Joystick parameters for x,y, and z task space
LINEAR_VELOCITY = 0.1
LINEAR_VELOCITY_Z = 0.1
LINEAR_VELOCITY_LIMIT = LINEAR_VELOCITY
# Joystick parameters for fingers
LINEAR_VELOCITY_GRIPPER = 0.1
# Joystick parameter for body motor
LINEAR_VELOCITY_BODY_MOTOR = 0.5
# Joystick parameter for gripper orientation
ORIENTATION_VELOCITY_GRIPPER = 0.05

class JOYSTICK:
    def __init__(self):
        self.joystick = Joy()
        self.sub_joy = rospy.Subscriber('/joy', Joy, self.receive_joystick)
        self.pub_foot_selection = rospy.Publisher('/cmd_foot_selection', Joy, queue_size=2)
        self.pub_leg_gripper = rospy.Publisher('/cmd_scaler', Joy, queue_size=2)
        self.pub_body_vel = rospy.Publisher('/cmd_scaler_body', Vector3, queue_size=2)
        self.leg_gripper = Joy()
        self.foot_selection = Joy()
        self.body_vel = Vector3()
        self.axes = np.zeros((4,1))
        self.buttons = np.zeros((17,1))
        self.MAX_VELOCITY_X_Y = LINEAR_VELOCITY
        self.MIN_VELOCITY_X_Y = -LINEAR_VELOCITY
        self.MAX_VELOCITY_Z = LINEAR_VELOCITY_Z
        self.LINEAR_VELOCITY_GRIPPER = LINEAR_VELOCITY_GRIPPER
        self.LINEAR_VELOCITY_BODY_MOTOR = LINEAR_VELOCITY_BODY_MOTOR
        self.ORIENTATION_VELOCITY_GRIPPER = ORIENTATION_VELOCITY_GRIPPER
        self.ORIENTATION_CHANGE = True
        self.RATE_GET_JOYSTICK = 500

    def receive_joystick(self, msg):
        self.joystick = msg

    def get_buttons(self):
        try:
            buttons = self.joystick.buttons
            self.A = buttons[0]
            self.B = buttons[1]
            self.X = buttons[2]
            self.Y = buttons[3]
            self.LB = buttons[4]
            self.RB = buttons[5]
            self.XBOX = buttons[8]
            self.LSTICK = buttons[9]
            self.RSTICK = buttons[10]
            self.BACK = buttons[6]
            self.START = buttons[7]

        except:
            self.A = 0
            self.B = 0
            self.X = 0
            self.Y = 0
            self.LB = 0
            self.RB = 0
            self.XBOX = 0
            self.LSTICK = 0
            self.RSTICK = 0
            self.BACK = 0
            self.START = 0

    def get_axes(self):
        try:
            axes = self.joystick.axes
            self.L_HORIZONTAL = axes[0] # Left: 0 to 1, Right: -1 to 0
            self.L_VERTICAL = axes[1] # Up: 0 to 1, Down: -1 to 0
            self.R_HORIZONTAL = axes[3] # Left: 0 to 1, Right: -1 to 0
            self.R_VERTICAL = axes[4] # Up: 0 to 1, Down: -1 to 0
            self.LT = axes[2]
            self.RT = axes[5]
            self.UP_DOWN_ARROW = axes[-1]
            self.RIGHT_LEFT_ARROW = axes[-2]

        except:
            self.L_HORIZONTAL = 0 # Left: 0 to 1, Right: -1 to 0
            self.L_VERTICAL = 0  # Up: 0 to 1, Down: -1 to 0
            self.R_HORIZONTAL = 0  # Left: 0 to 1, Right: -1 to 0
            self.R_VERTICAL = 0  # Up: 0 to 1, Down: -1 to 0
            self.LT = 0
            self.RT = 0
            self.UP_DOWN_ARROW = 0
            self.RIGHT_LEFT_ARROW = 0


    def send_velocity_commands(self):
        self.get_axes()
        self.get_buttons()

        # linear velocity for arm
        vel_y = (self.L_HORIZONTAL - (-1))/(1 - (-1))*(self.MAX_VELOCITY_X_Y-self.MIN_VELOCITY_X_Y)+self.MIN_VELOCITY_X_Y
        vel_x = (self.L_VERTICAL - (-1))/(1 - (-1))*(self.MAX_VELOCITY_X_Y-self.MIN_VELOCITY_X_Y)+self.MIN_VELOCITY_X_Y

        if self.RT<1.0:
            vel_z = ((1.0 - self.RT)/2.0)*self.MAX_VELOCITY_Z
        elif self.LT<1.0:
            vel_z = -1*((1.0 - self.LT)/2.0)*self.MAX_VELOCITY_Z
        else:
            vel_z = 0.0

        axes_list = [0.0]*8

        axes_list[0] = vel_y*-1
        axes_list[1] = vel_x
        axes_list[2] = vel_z

        # linear velocity for body
        # linear velocity
        vel_y_body = (self.R_HORIZONTAL - (-1))/(1 - (-1))*(self.MAX_VELOCITY_X_Y-self.MIN_VELOCITY_X_Y)+self.MIN_VELOCITY_X_Y
        vel_x_body = (self.R_VERTICAL - (-1))/(1 - (-1))*(self.MAX_VELOCITY_X_Y-self.MIN_VELOCITY_X_Y)+self.MIN_VELOCITY_X_Y

        # get open/close command for gripper
        if self.LSTICK == 1:
            axes_list[3] = self.LINEAR_VELOCITY_GRIPPER
        if self.RSTICK == 1:
            axes_list[3] = -self.LINEAR_VELOCITY_GRIPPER

        # get body motor
        body_motor = 0
        if self.START:
            body_motor = self.LINEAR_VELOCITY_BODY_MOTOR
        if self.BACK:
            body_motor = self.LINEAR_VELOCITY_BODY_MOTOR * -1
        axes_list[4] = body_motor

        # decide which orientation change occurs based on user
        if self.LB == 1:
            self.ORIENTATION_CHANGE = True
        if self.RB == 1:
            self.ORIENTATION_CHANGE = False

        if self.ORIENTATION_CHANGE:
            rot = 1
        else:
            rot = -1

        # get orientation for gripper
        if self.B == 1:
            axes_list[5] = self.ORIENTATION_VELOCITY_GRIPPER * rot
        else:
            axes_list[5]= 0.0

        if self.Y == 1:
            axes_list[6] = self.ORIENTATION_VELOCITY_GRIPPER * rot
        else:
            axes_list[6] = 0.0

        if self.X == 1:
            axes_list[7] = self.ORIENTATION_VELOCITY_GRIPPER * rot
        else:
            axes_list[7] = 0.0

        # get keys for foot selection (0 axes = foot 0, 1 axes = foot 1, 2 axes = foot 2, 3 axes = foot 3, 4 axes = grasp or release obstacle)
        axes_list_foot = [0.0]*8
        if self.UP_DOWN_ARROW == 1.0:
            axes_list_foot[0] = 1
        if self.RIGHT_LEFT_ARROW == -1.0:
            axes_list_foot[1] = 1
        if self.UP_DOWN_ARROW == -1.0:
            axes_list_foot[2] = 1
        if self.RIGHT_LEFT_ARROW == 1.0:
            axes_list_foot[3] = 1
        if self.A == 1:
            axes_list_foot[4] = 1

        self.leg_gripper.axes = axes_list
        self.foot_selection.axes = axes_list_foot
        self.body_vel.x = vel_y_body*-1
        self.body_vel.y = vel_x_body
        self.pub_foot_selection.publish(self.foot_selection)
        self.pub_leg_gripper.publish(self.leg_gripper)
        self.pub_body_vel.publish(self.body_vel)

if __name__ == '__main__':
    rospy.init_node('joystick_commands', anonymous=False)
    joy_stick = JOYSTICK()
    rate = rospy.Rate(joy_stick.RATE_GET_JOYSTICK)
    while not rospy.is_shutdown():
        joy_stick.send_velocity_commands()
        rate.sleep()