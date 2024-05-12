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
    # add a sphere marker as a persistent marker to the RVizVisualizer
    sphere_marker = create_sphere_marker(name='sphere', id=1, xyz=[1, 1, 1], reference_frame='map', dimensions=0.20, rgba=[1.0, 0.5, 0.5, 0.5])
    rv.add_persistent_marker(sphere_marker, pub_tf=True)  # the tf is named 'sphere.1'.
    # add a cube marker of which the pose is defined in the reference frame of 'sphere.1'
    cube_marker = create_cube_marker_from_bbox(name='cube', id=1, bbox3d=[-0.5, 0.5, -0.5, 0.5, -0.5, 0.5], reference_frame='sphere.1', rgba=[0.5, 1.0, 0.5, 0.5])    
    rv.add_persistent_marker(cube_marker)
    rospy.spin()