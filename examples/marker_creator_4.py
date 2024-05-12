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
    # add a path marker
    path_marker = create_path_marker(name='path', id=1, xyzlist=[(0, 0, 0), (0, 0, 1), (0, 1, 1), (1, 1, 1), (1, 0, 0)], reference_frame='map',
                                                line_width=0.05, rgba=[1.0, 0.5, 0.5, 0.5])
    rv.add_persistent_marker(path_marker)

    rospy.spin()