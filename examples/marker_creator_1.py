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
    # add a axis plane marker on xy plane as a persistent marker to the RVizVisualizer
    axis_plane_marker_xy = create_axisplane_marker(name='axisplane', id=1, bbox2d=[-1, -1, 1, 1], offset=2, reference_frame='map', axes='xy', rgba=[1, 0, 0])
    rv.add_persistent_marker(axis_plane_marker_xy)
    # add a axis plane marker on xy plane as a persistent marker to the RVizVisualizer
    axis_plane_marker_xz = create_axisplane_marker(name='axisplane', id=2, bbox2d=[-1, -1, 1, 1], offset=2, reference_frame='map', axes='xz', rgba=[0, 1, 0])
    rv.add_persistent_marker(axis_plane_marker_xz)
    # add a axis plane marker on yz plane as a persistent marker to the RVizVisualizer
    axis_plane_marker_xz = create_axisplane_marker(name='axisplane', id=3, bbox2d=[-1, -1, 1, 1], offset=2, reference_frame='map', axes='yz', rgba=[0, 0, 1])
    rv.add_persistent_marker(axis_plane_marker_xz)     
    rospy.spin()