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
    # add a group of markers for 'work_area'
    sphere_marker = create_sphere_marker(name='work_area', id=1, xyz=[1, 1, 1], reference_frame='map', dimensions=0.20, rgba=[1.0, 0.5, 0.5, 1.0])
    rv.add_persistent_marker(sphere_marker) 
    axis_marker = create_axisplane_marker(name='work_area', id=2, bbox2d=[-1, -1, 1, 1], offset=0, reference_frame='map', axes='xy', rgba=[1.0, 0.5, 0.5])
    rv.add_persistent_marker(axis_marker) 
    arrow_marker = create_arrow_marker(name='work_area', id=3, xyzrpy=[1, 1, 1, 0, 3.14, 0], reference_frame='map', dimensions=0.50, rgba=[1.0, 0.5, 0.5, 1.0])
    rv.add_persistent_marker(arrow_marker)     

    rospy.spin()