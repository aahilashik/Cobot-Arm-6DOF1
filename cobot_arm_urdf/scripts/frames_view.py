# # from mpl_toolkits import mplot3d
# import numpy as np
# import matplotlib.pyplot as plt
 
# # fig = plt.figure()
 
# # # syntax for 3-D projection
# # ax = plt.axes(projection ='3d')

# # def plot3DFrame(x, y, z):
# #     ax.plot3D([x, x+0.1], [y, y], [z, z], 'red')
# #     ax.plot3D([x, x], [y, y+0.1], [z, z], 'green')
# #     ax.plot3D([x, x], [y, y], [z, z+0.1], 'blue')
    

# # plot3DFrame(0, 0, 0)

# # plot3DFrame(-0.2, 0, 0.02)

# # plot3DFrame(-0.2, 0, 0.18)
# # plot3DFrame(-0.2, 0, 0.516)
# # plot3DFrame(-0.2, 0, 1.016)
# # plot3DFrame(-0.345, 0, 1.21349)
# # plot3DFrame(-0.345, 0, 1.44349)

# # plot3DFrame(-0.345, 0, 1.63788)

# # ax.set_title('3D Frame Plot')
# # plt.show()



from gcodeparser import GcodeParser
import matplotlib.pyplot as plt
import numpy as np
import math

ax = plt.axes(projection='3d')

def rotRPY(theta):
    R_x = np.array([[1,         0,                  0                   ],
                    [0,         math.cos(theta[0]), -math.sin(theta[0]) ],
                    [0,         math.sin(theta[0]), math.cos(theta[0])  ]   ])

    R_y = np.array([[math.cos(theta[1]),    0,      math.sin(theta[1])  ],
                    [0,                     1,      0                   ],
                    [-math.sin(theta[1]),   0,      math.cos(theta[1])  ]   ])

    R_z = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),    0   ],
                    [math.sin(theta[2]),    math.cos(theta[2]),     0   ],
                    [0,                     0,                      1   ]  ])

    return np.dot(R_z, np.dot( R_y, R_x ))

point0 = np.array([0, 0, 2])

x, y, z = [], [], []
X, Y = 0.0, 0.0
with open('/media/ubuntussd/OS/Users/NKKS-HOME/Desktop/example.gcode', 'r') as f:
    gcode = f.read()
    parsed_gcode = GcodeParser(gcode)
    append = False
    for line in parsed_gcode.lines:
        if line.params == {}:
            continue
        
        if line.params.get("S") == 30:
            append = True
        elif line.params.get("S") == 50:
            append = False
            # plt.plot(x, y,linewidth=2.0)

            ax.plot3D(x, y, z,linewidth=2.0)

            if len(x)>>0: X, Y = x[-1], y[-1]
            x, y, z = [], [], []
        
        if not append:
            print(line)
            if line.params.get("X"):
                X, Y = line.params.get("X"), line.params.get("Y")
            #     # plt.scatter(line.params.get("X"), line.params.get("Y"))
            #     plt.plot([X, line.params.get("X")], [Y, line.params.get("Y")], linewidth=5.0)
            continue
        if line.command_str == "G1" and line.params.get("X"):
            if X!=None:
                # plt.plot([X, line.params.get("X")], [Y, line.params.get("Y")], linewidth=5.0)
                point1 = np.array([X, Y, 0])
                point1 = point0 + np.dot(rotRPY([np.pi/4, 0, 0]), point1)
                point2 = np.array([line.params.get("X"), line.params.get("Y"), 0])
                point2 = point0 + np.dot(rotRPY([np.pi/4, 0, 0]), point2)

                ax.plot3D([point1[0], point2[0]], [point1[1], point2[1]], [point1[2], point2[2]], linewidth=5.0)
                X, Y = None, None

            point1 = np.array([line.params.get("X"), line.params.get("Y"), 0])
            point1 = point0 + np.dot(rotRPY([np.pi/4, 0, 0]), point1)

            x.append(point1[0])
            y.append(point1[1])
            z.append(point1[2])

plt.show()













# #!/usr/bin/env python

# PACKAGE_NAME = 'rviz'
# import roslib; roslib.load_manifest(PACKAGE_NAME)
# #import sys, os.path, time
# #import numpy as np
# #from scipy import linalg
# #from matplotlib import pyplot as plt
# import rospy
# from visualization_msgs.msg import Marker, MarkerArray
# from geometry_msgs.msg import Pose, Point, Vector3, Quaternion
# from std_msgs.msg import ColorRGBA



# rospy.init_node('marker_test')
# marker_pub = rospy.Publisher('marker_test', Marker)

# def make_marker(marker_type, scale, r, g, b, a):
#     # make a visualization marker array for the occupancy grid
#     m = Marker()
#     m.action = Marker.ADD
#     m.header.frame_id = '/base_link'
#     m.header.stamp = rospy.Time.now()
#     m.ns = 'marker_test_%d' % marker_type
#     m.id = 0
#     m.type = marker_type

#     m.pose.position.x = 0
#     m.pose.position.y = 2
#     m.pose.position.z = 0

#     m.pose.orientation.x = 0
#     m.pose.orientation.y = 0.5
#     m.pose.orientation.z = 0
#     m.pose.orientation.w = 0.5
#     m.scale = scale
#     m.color.r = 1.0;
#     m.color.g = 1.0;
#     m.color.b = 1.0;
#     m.color.a = 1.0;
#     return m

# def make_arrow_points_marker(scale, tail, tip, idnum):
#     # make a visualization marker array for the occupancy grid
#     m = Marker()
#     m.action = Marker.ADD
#     m.header.frame_id = '/base_link'
#     m.header.stamp = rospy.Time.now()
#     m.ns = 'points_arrows'
#     m.id = idnum
#     m.type = Marker.ARROW
#     m.pose.orientation.y = 0
#     m.pose.orientation.w = 1
#     m.scale = scale
#     m.color.r = 0.2
#     m.color.g = 0.5
#     m.color.b = 1.0
#     m.color.a = 0.3

#     m.points = [ tail, tip ]
#     return m
 
# while not rospy.is_shutdown():
#     rospy.loginfo('Publishing arrow marker')

#     # scale = Vector3(2,4,0.69)
#     # marker_pub.publish(make_arrow_points_marker(scale,Point(0,0,0), Point(3,0,0), 3))

#     scale = Vector3(0.297, 0.420, 0.02)
#     # marker_pub.publish(make_marker(Marker.SPHERE,   scale, 1, .5, .2, .3))
#     # marker_pub.publish(make_marker(Marker.CYLINDER, scale, .5, .2, 1, .3))
#     marker_pub.publish(make_marker(Marker.CUBE, scale, .2, 1, .5, .3))
#     # marker_pub.publish(make_marker(Marker.ARROW,    scale, 1, 1, 1, .5))
#     rospy.sleep(1.0)

