#!/usr/bin/env python3

# Copyright 2024 - Andrew Kwok Fai LUI, 
# Robotics and Autonomous Systems Group, REF, RI
# and the Queensland University of Technology

__author__ = 'Andrew Lui'
__copyright__ = 'Copyright 2024'
__license__ = 'GPL'
__version__ = '1.0'
__email__ = 'ak.lui@qut.edu.au'
__status__ = 'Development'

import rospy
from rviz_marker.rviz_tools import *

if __name__ == '__main__':
    rospy.init_node('test_rv_node', anonymous=False)   
    # create the RVizVisualizer 
    rv = RvizVisualizer()
    # add a custom transform called 'workspace'
    transform_pose = Pose()
    transform_pose.position = Point(1, 1, 1)
    transform_pose.orientation = Quaternion(0, 0, 0, 1)
    rv.add_custom_tf('workspace', 'map', transform_pose)
    # add a sphere marker as a persistent marker to the RVizVisualizer
    sphere_marker = create_sphere_marker(name='sphere', id=1, xyz=[0.5, 0, 0], reference_frame='workspace', dimensions=0.20, rgba=[1.0, 0.5, 0.5, 1.0])
    rv.add_persistent_marker(sphere_marker) 
    rospy.spin()