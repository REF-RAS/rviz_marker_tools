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
    # computing the full path of the stl file
    # teapot_mesh = 'file://' + os.path.join(os.path.dirname(__file__), '../docs/assets/UtahTeapot.stl')
    # teapot_mesh = os.path.join(os.path.dirname(__file__), '../docs/assets/UtahTeapot.stl')
    # teapot_mesh = 'package://rviz_marker_tools/docs/assets/UtahTeapot.stl'
    teapot_mesh = 'rviz_marker_tools/docs/assets/UtahTeapot.stl' 
    logger.info(f'mesh_file location: {teapot_mesh}')
    mesh_marker = create_mesh_marker(name='teapot', id=1, file_uri=teapot_mesh, xyzrpy=[-1.0, -1.0, 0.0, 0, 0, 0], 
                                     reference_frame='map', dimensions=[0.05, 0.05, 0.05], rgba=[0.5, 1.0, 1.0, 1.0])
    rv.add_persistent_marker(mesh_marker) 
    rospy.spin()