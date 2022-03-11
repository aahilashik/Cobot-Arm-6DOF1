/*

            |ROS Parameter Server|                                                     |Forward Kinematics|
                                \                                                        /
                      |          \                                                      /
                    (or)        |URDF::Model|  -->  |KDL::Tree|  -->  |KDL::Chain|  ---(
                      |          /          \                        /                  \
                                /            |Min & Max Joint Limits|                    \
                      |Filesystem|                             '-----------------------|Inverse Kinematics|

*/

#include <iostream>

// URDF and KDL Variables
#include <urdf/model.h>
#include <kdl/chain.hpp>
#include <kdl/frames_io.hpp>

// KDL Parser for URDF File
#include <kdl_parser/kdl_parser.hpp>

// KDL Solvers
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolverpos_lma.hpp>

// Time Processing
#include <chrono>

// TF Processing
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>


using namespace std;

// Solver Result Status
enum {
         E_DEGRADED                 = +1,
         E_NOERROR                  =  0,
         E_NO_CONVERGE              = -1,
         E_UNDEFINED                = -2,
         E_NOT_UP_TO_DATE           = -3,
         E_SIZE_MISMATCH            = -4,
         E_MAX_ITERATIONS_EXCEEDED  = -5,
         E_OUT_OF_RANGE             = -6,
         E_NOT_IMPLEMENTED          = -7,
         E_SVD_FAILED               = -8
     };
 
const char* strError(int error) {
    if (E_NOERROR == error) return "No error";
    else if (E_NO_CONVERGE == error) return "Failed to converge";
    else if (E_UNDEFINED == error) return "Undefined value";
    else if (E_DEGRADED == error) return "Converged but degraded solution";
    else if (E_NOT_UP_TO_DATE == error) return "Internal data structures not up to date with Chain";
    else if (E_SIZE_MISMATCH == error) return "The size of the input does not match the internal state";
    else if (E_MAX_ITERATIONS_EXCEEDED == error) return "The maximum number of iterations is exceeded";
    else if (E_OUT_OF_RANGE == error) return "The requested index is out of range";
    else if (E_NOT_IMPLEMENTED == error) return "The requested function is not yet implemented";
    else  if (E_SVD_FAILED == error) return "SVD failed";
    else return "UNKNOWN ERROR";
}

// To retrieve Robot Model from URDF file
char urdf_filename[] = "/home/ubuntussd/test_ws/src/cobot_arm_urdf/urdf/cobot_arm_urdf.urdf";

// To retrieve Robot Model from ROS Parameter Server
bool use_robot_description  = false;
string robot_desc_param     = "robot_description";

ros::Publisher jointStatePublisher;
ros::Subscriber cmdPoseSubscriber;

sensor_msgs::JointState jointState;

urdf::Model model;
KDL::Tree tree;
KDL::Chain chain;

KDL::JntArray minLimit, maxLimit;

boost::shared_ptr<KDL::ChainFkSolverPos> fksolver_p;
boost::shared_ptr<KDL::ChainIkSolverVel> iksolver_v;
boost::shared_ptr<KDL::ChainIkSolverPos> iksolver_p;

int num_of_trials = 50;
// Joint Position Array that will contain the results
KDL::JntArray resJntPosArray(chain.getNrOfJoints());     
void inverseKinematicsCallback( const geometry_msgs::Pose& pose ) {

    double pX, pY, pZ;            // Position
    double qX, qY, qZ, qW;        // Quaternion Angles
    
    pX = pose.position.x;
    pY = pose.position.y;
    pZ = pose.position.z;  // metre to mm conversion
    qX = pose.orientation.x;
    qY = pose.orientation.y;
    qZ = pose.orientation.z;
    qW = pose.orientation.w;


    // Record start time
    auto _begin = std::chrono::high_resolution_clock::now();
    
    // Creation of Joint Array
    KDL::JntArray curJntPosArray(chain.getNrOfJoints());     // Current/Initial Joint Position Array
    KDL::JntArray outJntPosArray(chain.getNrOfJoints());     // Joint Position Array that will contain the results

    for (int i=0; i<chain.getNrOfJoints(); i++)
        curJntPosArray(i)=(double) jointState.position[i];
    // curJntPosArray(0)=(double) KDL::deg2rad * -0;
    // curJntPosArray(1)=(double) KDL::deg2rad * +0;
    // curJntPosArray(2)=(double) KDL::deg2rad * -0;
    // curJntPosArray(3)=(double) KDL::deg2rad * +0;
    // curJntPosArray(4)=(double) KDL::deg2rad * -0;
    // curJntPosArray(5)=(double) KDL::deg2rad * +0;

    // Set Destination Frame
    KDL::Frame F_dest = KDL::Frame( KDL::Rotation::Quaternion(qX, qY, qZ, qW), 
                                            KDL::Vector(pX, pY, pZ));

    // int kinematics_status = iksolver_p->CartToJnt(curJntPosArray, F_dest, outJntPosArray);
    // cout << "INFO : IKSolver Stat - " << strError(kinematics_status) << endl;
    // if (kinematics_status>=0) {
    //     resJntPosArray = outJntPosArray;
    //     cout << "  DATA : IK Joint Angles : " << endl;
    //     cout << setw(6) << "No" << setw(15) << "Angle(rad)" << setw(5) << "  -  " << setw(15) << "Angle(deg)" << endl;
    //     for (int i=0; i<chain.getNrOfJoints(); i++) {
    //         cout << setw(6) << i << setw(15) << resJntPosArray(i) << setw(5) << "  -  " << setw(15) << resJntPosArray(i) * KDL::rad2deg << endl;
    //     }
    // } else {
    //     cerr << "ERROR : Could not calculate Inverse Kinematics :(" << endl;
    // }

    int trial, kinematics_status;
    for (trial=0; trial<num_of_trials; trial++) {
        // Loop to assign some values to the joint positions
        for (int i=0; i<chain.getNrOfJoints(); i++) {
            // Random value within limits
            double randWL = (minLimit(i)*1000.0) + ( std::rand() % int( maxLimit(i)*1000.0 - minLimit(i)*1000.0 + 1 ) );
            if (trial==0)
                // Set Current Joint Positions
                curJntPosArray(i) = (double) jointState.position[i];
            else if (trial<10)
                // Set Current Joint Angles with 30% of Random Values
                curJntPosArray(i) += (double) 0.3*randWL/1000.0;
            else
                // Set Random Joint Angles within limits
                curJntPosArray(i) = (double) randWL/1000.0;
        }
        
        kinematics_status = iksolver_p->CartToJnt(curJntPosArray, F_dest, outJntPosArray);
        if (kinematics_status>=0) {
            resJntPosArray = outJntPosArray;
            cout << "  DATA : IK Joint Angles : " << endl;
            cout << setw(6) << "No" << setw(15) << "Angle(rad)" << setw(5) << "  -  " << setw(15) << "Angle(deg)" << endl;
            for (int i=0; i<chain.getNrOfJoints(); i++)
                cout << setw(6) << i << setw(15) << resJntPosArray(i) << setw(5) << "  -  " << setw(15) << resJntPosArray(i) * KDL::rad2deg << endl;
            
            break;
        } 
    }
    cout << "INFO : IKSolver Iter " << trial << " | Stat - " << strError(kinematics_status) << endl;

    // Record end time
    auto _end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> _elapsed = _end - _begin;
    cout << "INFO : IK - Elapsed Time : " << _elapsed.count() << " seconds!" << endl;
    cout << "INFO : IK - Frequency : " << 0.001/_elapsed.count() << " kHz!" << endl;

}

int main(int argc, char **argv) {
    // Initialize the ROS node and nodehandle
    ros::init(argc, argv, "cobot_arm_plugin");
    ros::NodeHandle nh;

    // Load URDF model/KDL tree from the parameter server(if true) or from urdf file(if false)
    if (use_robot_description) {
        // Load URDF Model from ROS parameter server
        if (!model.initParamWithNodeHandle(robot_desc_param, nh)){
            cerr << "ERROR : Failed to retrieve robot model from ROS parameter server";
            return -1;
        }
    } else {
        // Load XML file to load URDF 
        tinyxml2::XMLDocument xmlDocument;
        xmlDocument.LoadFile(urdf_filename);
        if (xmlDocument.Error()){
            cerr <<  xmlDocument.ErrorStr() << endl;
            return -1;
        }

        // Load URDF Model from Xml Document
        if (model.initXml(&xmlDocument))
            cout << "INFO : URDF Model has been loaded successfully!!" << endl;
        else {
            cerr << "ERROR : Failed to parse urdf robot model from xml!";
            return -1;
        }
    }

    // Load KDL Tree from urdf model
    if (kdl_parser::treeFromUrdfModel(model, tree))
        cout << "INFO : KDL Tree has been loaded successfully!!" << endl;
    else {
        cerr << "ERROR : Failed to load kdl tree from urdf model!";
        return -1;
    }

    // Load KDL Chain from KDL Tree
    if (tree.getChain("robot_footprint", "tool_tip", chain))
        cout << "INFO : KDL Chain has been loaded successfully!!" << endl;
    else {
        cerr << "ERROR : Failed to load KDL Chain from KDL Tree!" << endl;
        return -1;
    }

    // Get Minimum and Maximum Joints Limits
    cout << "INFO : \rGetting Joint Limits values....";
    minLimit = KDL::JntArray(chain.getNrOfJoints());
    maxLimit = KDL::JntArray(chain.getNrOfJoints());
    shared_ptr<const urdf::Joint> jointTmp;
    int index = 0;
    for(unsigned int i=0;i<chain.getNrOfSegments();i++){
        if (chain.getSegment(i).getJoint().getType() != KDL::Joint::None) { 
            jointTmp = model.getJoint(chain.getSegment(i).getJoint().getName());
            if ( jointTmp->type != urdf::Joint::CONTINUOUS )  {
                minLimit(index)     = (double)jointTmp->limits->lower;
                maxLimit(index++)   = (double)jointTmp->limits->upper;
            }
        }
    }
    // Print Joint Min & Max limits value
    cout << "\r  DATA : Joint Limits :                  " << endl;
    cout << setw(6) << "No" << setw(10) << "Min" << setw(5) << "  -  " << setw(8) << "Max" << endl;
    for (int i=0; i<chain.getNrOfJoints(); i++) {
        cout << setw(6) << i << setw(10) << minLimit(i) << setw(5) << "  -  " << setw(8) << (maxLimit(i)) << endl;
    }

    //Creation of the Kinematics solvers
            //Forward Kinematics Position solver
        // KDL::ChainFkSolverPos_recursive fksolver_p(chain);
        fksolver_p.reset(new KDL::ChainFkSolverPos_recursive(chain));
            //Inverse Kinematics Velocity solver
        // KDL::ChainIkSolverVel_pinv iksolver_v(chain);		
        iksolver_v.reset(new KDL::ChainIkSolverVel_pinv(chain));
            // Inverse Kinematics Position solver with joint limits
            // Maximum 1000 iterations, stop at accuracy 1e-8
        // KDL::ChainIkSolverPos_NR_JL iksolver_p(chain, minLimit, maxLimit, fksolver_p, iksolver_v, 10000, 1e-8);    
        iksolver_p.reset(new KDL::ChainIkSolverPos_NR_JL(chain, minLimit, maxLimit, *fksolver_p, *iksolver_v, 1000, 1e-8));
               
    jointStatePublisher = nh.advertise<sensor_msgs::JointState>("/joint_states", 1000);

    cmdPoseSubscriber = nh.subscribe("/cobot_arm/cmd_pose", 1000, inverseKinematicsCallback);

    // sensor_msgs::JointState jointState;
    
    resJntPosArray.resize(chain.getNrOfJoints());
    SetToZero(resJntPosArray);

    // resJntPosArray(0)=(double) KDL::deg2rad * -0;
    // resJntPosArray(1)=(double) KDL::deg2rad * +45;
    // resJntPosArray(2)=(double) KDL::deg2rad * -110;
    // resJntPosArray(3)=(double) KDL::deg2rad * +0;
    // resJntPosArray(4)=(double) KDL::deg2rad * 65;
    // resJntPosArray(5)=(double) KDL::deg2rad * +0;

    while (ros::ok()) {
        jointState.header.stamp = ros::Time::now();
        jointState.header.seq+=1;
        jointState.name.resize(chain.getNrOfJoints());
        jointState.position.resize(chain.getNrOfJoints());
        jointState.velocity.resize(0);
        jointState.effort.resize(0);
        jointState.name = {"Joint_1", "Joint_2", "Joint_3", "Joint_4", "Joint_5", "Joint_6"};

        jointState.position = { resJntPosArray(0), resJntPosArray(1), 
                                  resJntPosArray(2), resJntPosArray(3), 
                                    resJntPosArray(4), resJntPosArray(5)};

        jointStatePublisher.publish(jointState);
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
    ros::spin();
}