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
    # add line markers as a temporary marker to the RVizVisualizer with a different lifetime
    for i in range(10):
        rv.pub_temporary_marker(create_line_marker(name='line', id=i, xyz1=[i * 0.5, 0, 0], xyz2=[i * 0.5, 1, 0], reference_frame='map',
                                                    line_width=0.02, rgba=[1.0, 1.0, 0.0, 1.0], lifetime=rospy.Duration(i))) 
 
    rospy.spin()