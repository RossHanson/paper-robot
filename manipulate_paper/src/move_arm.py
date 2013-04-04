#!/usr/bin/env python
import roslib; roslib.load_manifest('manipulate_paper')
import rospy
import ee_cart_imped_action
import ee_cart_imped_control.control_switcher
import arm_navigation_msgs.msg
import arm_navigation_msgs.srv
import actionlib
import geometry_msgs.msg
from std_msgs.msg import String

class ArmControl:
    def __init__(self, arm_name):
        self.arm_name = arm_name
        self.switchToForceControl()
        self.force_control = ee_cart_imped_action.EECartImpedClient\
            (self.arm_name)
        self.switchToArmNavigationControl()
        self.move_arm_control =\
            actionlib.SimpleActionClient\
            ('/move_'+ self.arm_name,\
                 arm_navigation_msgs.msg.MoveArmAction)
        def switchToArmNavigationControl(self):
            rospy.loginfo('Switching to arm navigation control on arm %s', self.arm_name)
            ee_cart_imped_control.control_switcher.PR2CMClient.load_cartesian\
                (self.arm_name)
        def switchToForceControl(self):
            rospy.loginfo('Switching to force control on arm %s', self.arm_name)
            ee_cart_imped_control.control_switcher.PR2CMClient.load_ee_cart_imped\
                (self.arm_name)

        def moveToPoseForceControl(self,pose_stamped,time):
            self.switchToForceControl()
            self.force_control.moveToPoseStamped(pose_stamped, time)

def reset_arms():
    control_right = ee_cart_imped_action.EECartImpedClient('right_arm')
    control_left  = ee_cart_imped_action.EECartImpedClient('left_arm')
    control_right.addTrajectoryPoint(0.1, -0.75, 0, 0, 0, 0, 1,
                               700, 700, 700, 30, 30, 30,
                               False, False, False, False, False,
                               False, 4, '/torso_lift_link')
    control_left.addTrajectoryPoint(0.1, 0.75, 0, 0, 0, 0, 1,
                               700, 700, 700, 30, 30, 30,
                               False, False, False, False, False,
                               False, 4, '/torso_lift_link')
    control_right.sendGoal()
    control_left.sendGoal()

def move_arm(arm_name,x,y,z,xOR,yOR,zOR,ref_frame):
    control = ee_cart_imped_action.EECartImpedClient(arm_name)
    print x,y,z,xOR,yOR,zOR
    control.addTrajectoryPoint(float(x),float(y),float(z),float(xOR),float(yOR),float(zOR),1,
                               700,700,700,30,30,30,
                               False, False, False, False, False,
                               False, 4, ref_frame)
    control.sendGoal()

def move_to_table_edge(table_x, table_z, arm_name="right_arm"):
    offset_x = 0.06
    offset_z = 0.01
    control = ee_cart_imped_action.EECartImpedClient(arm_name)
    rospy.loginfo("Table x and y: %s %s" % (table_x,table_z))
    control.addTrajectoryPoint(float(table_x)-offset_x,0.0,float(table_z)-offset_z,0.1,0.0,1.0,1,
                               700,700,700,30,30,30,
                               False,False,False,False,False,False,
                               4,"/torso_lift_link")
    control.sendGoal()

def main():
    rospy.init_node('force_control_test', anonymous=True)
    rospy.Subscriber('force_control_commands',String, callback)
    rospy.spin()
    
def callback(data):
    rospy.loginfo(rospy.get_name() + ": I heard %s" % data.data)
    args = data.data.split(" ")
    print locals()
    func_to_cal = globals()[args[0]]
    apply(func_to_cal,args[1:])
    rospy.loginfo("Finished callback")
    
if __name__=='__main__':
    main()    
