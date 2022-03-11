#!/usr/bin/env python
import rospy
from tf2_msgs.msg import TFMessage
from tf.transformations import euler_from_quaternion, quaternion_from_euler


tf = {}
positions = []
joints = ["Link_1", "Link_2", "Link_3", "Link_4", "Link_5", "base_link", ]
def TFcallback(data):
    global positions
    print(len(data.transforms))
    # positions = []
    for transform in data.transforms:
        _tf = {}

        orientation_q = transform.transform.rotation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

        pose = transform.transform.translation
        # positions.append(position)


        print("chain.addSegment(Segment(Joint(Joint::Rot), Frame(Rotation::RPY({}, {}, {}), Vector({}, {}, {}))));\t// {}".format(roll, pitch, yaw, pose.x, pose.y, pose.z, transform.child_frame_id))

        # print("\t", transform.child_frame_id , " - ", (roll, pitch, yaw))
    # tf[]
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

        # if transform.child_frame_id in joints:
            # position = transform.transform.translation
            # positions.append(position)
        # plotFrame(position.x, position.y, position.z)



def spinOnce():
    r = rospy.Rate(10)
    r.sleep()
    
def TFlistener():
    global positions
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/tf_static", TFMessage, TFcallback)

    rospy.spin()
    # while True:
        # spinOnce()
        # if positions!= []: break

if __name__ == '__main__':
    TFlistener()

    # if positions!= []:
    #     import matplotlib
    #     import numpy as np
    #     from mpl_toolkits import mplot3d
    #     import matplotlib.pyplot as plt

    #     fig = plt.figure()
    #     ax = plt.axes(projection='3d')
    #     ax.set_title('3D plot')

    #     def plotFrame(x, y, z):
    #         ax.plot3D([x, x+0.05], [y, y], [z, z], 'red')
    #         ax.plot3D([x, x], [y, y+0.05], [z, z], 'green')
    #         ax.plot3D([x, x], [y, y], [z, z+0.05], 'blue')
            
    #     if len(positions) >= 5:
    #         for position in positions:
    #             plotFrame(position.x, position.y, position.z)
            
    #     plt.show()