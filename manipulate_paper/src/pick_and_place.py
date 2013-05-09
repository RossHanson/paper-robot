import roslib
roslib.load_manifest('pr2_pick_and_place_demos')
import rospy
from pr2_pick_and_place_demos.pick_and_place manager import *
from object_manipulator.convert_functions import *

class SimplePickAndPlaceExample():

    def __init__(self):
        rospy.loginfo("initializing pick and place manager")
        self.papm = PickAndPlaceManager()
        rospy.loginfo("finished initializing pick and place manager")

    def pick_up_object_near_point(self,target_point, whicharm):
        self.papm.call_tabletop_detection(update_table=1,update_place_rectangle =1,
                                          clear_attached_objects =1)

        success = self.papm.pick_up_object_near_point(target_point,whicharm)
        if success:
            rospy.loginfo("pick up was successful")
        else:
            rospy.loginfo("pick up failed")

    def place_object(self, whicharm, place_rect_dims, place_rect_center):
        rospy.info("Putting down the object in the %s gripper"\
                       % self.papm.arm_dict[whicharm])
        success = self.papm.put_down_object(whicharm,
                                            max_place_tries=25,
                                            use_place_override=1)
        if success:
            rospy.loginfo("place returned success")
        else:
            rospy.loginfo("place returned failure")
        return success

def move_object(table_edge_x, table_z, obj_offset=.02):
    rospy.init_node('simple_pick_and_place_example')
    sppe = SimplePickAndPlaceExample()
    target_point_xyz = [table_edge_x+.2,0,table_z]
    target_point = create_point_stamped(target_point_xyz,'torso_lift_link')
    success = sppe.pick_up_object_near_point(target_point,0)

    if success:
        place_rect_dims = [.3, .3]
        center_xyz = [table_edge_x+obj_offset,0,table_z]
        center_quat = [0,0,01]
        place_rect_center = create_point_stamped(center_xyz+center_quat,'torso_lift_link')
        rospy.loginfo("Started placement!")
        sppe.place_object(0,place_rect_dims,place_rect_center)
        rospy.loginfo("Finished placement!")
