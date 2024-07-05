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

def create_marker_array(grid_dim:tuple, grid_cell_size:tuple, tile_size:tuple) -> MarkerArray:
    marker_array = MarkerArray()
    for x in range(grid_dim[0]):
        for y in range(grid_dim[1]):
            xyzrpy=[x * grid_cell_size[0], y * grid_cell_size[1], 0.0, 0, 0, 0]
            tile = create_cube_marker_from_xyzrpy('tile', x + y * grid_dim[0], xyzrpy, reference_frame='map', 
                                    dimensions=[tile_size[0], tile_size[1], tile_size[2]], rgba=[0.0, 0.2, 1.0, 0.5])
            marker_array.markers.append(tile)
    return marker_array

if __name__ == '__main__':
    rospy.init_node('test_rv_node', anonymous=False)    
    # create the RVizVisualizer 
    rv = RvizVisualizer()
    # add a marker array
    marker_array = create_marker_array((9, 3), (0.5, 0.5), (0.46, 0.46, 0.01))
    rv.add_persistent_marker_array(marker_array)

    rospy.spin()