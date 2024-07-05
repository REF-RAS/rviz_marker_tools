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

# -- the test program
if __name__ == '__main__':
    rospy.loginfo(f'running test_rv_node')
    rospy.init_node('test_rv_node', anonymous=False)    
    # test the mark visualization
    rv = RvizVisualizer()
    the_pose:Pose = Pose()
    the_pose.position = Point(0, 0, 0)
    the_pose.orientation = Quaternion(0, 0, 0, 1)
    # create world frame
    rv.add_custom_tf('world', 'map', the_pose)

    text_marker_1 = rv.add_persistent_marker(create_text_marker(name='text', id=1, text='Hello', xyzrpy=[0, 0, 0, 0.2, 0, 0], reference_frame='world', dimensions=0.3), pub_tf=True)
    text_marker_2 = rv.add_persistent_marker(create_text_marker(name='text', id=2, text='World', xyzrpy=[0, 1, 0, 0.2, 0, 0], reference_frame='world', dimensions=0.3), pub_tf=True)
    rv.add_persistent_marker(create_line_marker('line', 1, [1, 0, 0], [0, 0, 1], 'world', 0.01, rgba=[0.0, 1.0, 1.0, 1.0]), pub_period=0.1)
    rv.add_persistent_marker(create_sphere_marker('sphere', 1, [1, 1, 1], 'world', 0.05, rgba=[0.5, 1.0, 1.0, 1.0]))    
    rv.pub_temporary_marker(create_arrow_marker('arrow', 1, [1.0, 0.0, 0.0, 1.0, 0.0, 0.0], 'world', lifetime=rospy.Duration(3.0)))
    # delete the line marker
    rospy.sleep(rospy.Duration(2.0))
    rv.delete_persistent_marker('line', 1)
    # delete all markers
    rospy.sleep(rospy.Duration(2.0))
    rv.delete_all_persistent_markers()
    # display stl mesh file
    teapot_mesh = os.path.join(os.path.dirname(__file__), '../docs/assets/UtahTeapot.stl')
    teapot_mesh = 'file://' + teapot_mesh
    rv.add_persistent_marker(create_mesh_marker('teapot', 1, teapot_mesh, [-1.0, -1.0, 0.0, 0, 0, 0], 'world', [0.05, 0.05, 0.05], rgba=[0.5, 1.0, 1.0, 1.0]))  
    # display image as pointcloud
    image_bgr = cv2.imread(os.path.join(os.path.dirname(__file__), '../docs/assets/CoralFish.png'))
    pc2_message = create_pointcloud_from_image(image_bgr, (0, 0.5, 0), pixel_physical_size=[0.002, 0.002, -1], reference_frame='world')
    rv.add_pointcloud('the_image', pc2_message)
    # add the text marker
    rospy.sleep(rospy.Duration(2.0))
    text_marker_1 = rv.add_persistent_marker(create_text_marker('text', 1, 'Hello', [0, 0, 0, 0.2, 0, 0], 'world', 0.3), pub_period=0.1, pub_tf=True)
    for i in range(100):
        pose = text_marker_1.pose
        pose.position.x += random.uniform(-0.5, 0.5)
        rospy.sleep(rospy.Duration(0.2))
    # delete the text marker again
    rv.delete_persistent_marker('text', 1)
    # create marker array
    marker_array = MarkerArray()
    for x in range(4):
        for y in range(4):
            xyzrpy=[x * 0.4, y * 0.4, 1.0, 0, 0, 0]
            tile = create_cube_marker_from_xyzrpy('tile', x + y * 4, xyzrpy, reference_frame='map', 
                                    dimensions=[0.3, 0.3, 0.05], rgba=[0.0, 0.2, 1.0, 0.5])
            marker_array.markers.append(tile)    
    rv.add_persistent_marker_array(marker_array)
    rospy.sleep(rospy.Duration(5.0))
    rv.delete_all_persistent_marker_arrays()
    logger.info(f'The demo is completed')
    rospy.spin()