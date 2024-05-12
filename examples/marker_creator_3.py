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
    # add a small cube marker
    cuboid_marker_1 = create_cube_marker_from_xyzrpy(name='cube', id=1, xyzrpy=[0, 0, 0, 0, 0, 0], reference_frame='map',
                                                dimensions=0.2, rgba=[1.0, 0.5, 0.5, 0.5])
    rv.add_persistent_marker(cuboid_marker_1)
    # add a larger cube marker
    cuboid_marker_2 = create_cube_marker_from_xyzrpy(name='cube', id=2, xyzrpy=[0.5, 0.5, 0.5, 1.2, 0.0, 1.2], reference_frame='map',
                                                dimensions=0.5, rgba=[0.0, 0.5, 1.0, 0.5])
    rv.add_persistent_marker(cuboid_marker_2)

    rospy.spin()