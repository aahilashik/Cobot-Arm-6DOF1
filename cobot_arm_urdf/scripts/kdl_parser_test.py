import kdl_parser_py.urdf

def runTest():
    filename = "/home/ubuntussd/test_ws/src/Cobot_Arm_URDF/urdf/Cobot_Arm_URDF.urdf"
    (ok, tree) = kdl_parser_py.urdf.treeFromFile(filename)
    # KDL doesn't count fixed joints (since they aren't kinematic)
    print(tree.getNrOfJoints(), 8)
    # KDL doesn't count base link (since it's attached by fixed links
    print(tree.getNrOfSegments(), 10)
    chain = tree.getChain("base_link", "tool_tip")
    print(chain.getNrOfSegments(), 2)
    print(chain.getNrOfJoints(), 2)
    print(chain.getSegment(0).getName(), "gripper_pole")
    print(chain.getSegment(0).getJoint().getName(), "gripper_extension")
    print(chain.getSegment(1).getName(), "right_gripper")
    print(chain.getSegment(1).getJoint().getName(), "right_gripper_joint")

    print(chain.getSegment(4))

    inertia = chain.getSegment(1).getInertia()
    print(inertia.getCOG().z(), 3.0)

if __name__ == '__main__':
    runTest()
