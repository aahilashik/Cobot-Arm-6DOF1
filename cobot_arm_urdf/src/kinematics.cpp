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

int main(int argc, char **argv) {
    // Load URDF model/KDL tree from the parameter server(if true) or from urdf file(if false)
    urdf::Model model;
    // KDL::Tree tree;
    if (use_robot_description) {
        // Initialize the ROS node and nodehandle
        ros::init(argc, argv, "robot_desc_retriever");
        ros::NodeHandle nh;
        
        // Load URDF Model from ROS parameter server
        if (!model.initParamWithNodeHandle(robot_desc_param, nh)){
            cerr << "ERROR : Failed to retrieve robot model from ROS parameter server";
            return false;
        }

        // string robot_desc_string;
        // nh.param(robot_desc_param, robot_desc_string, string());
        // if (!kdl_parser::treeFromString(robot_desc_string, tree)){
        //     cerr << "ERROR : Failed to construct kdl tree";
        //     return -1;
        // }

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

        // if (!kdl_parser::treeFromFile(urdf_filename, tree)){
        //     cerr <<"Failed to load kdl tree from file!" ;
        //     return -1;
        // }
    }

    // Load KDL Tree from urdf model
    KDL::Tree tree;
    if (kdl_parser::treeFromUrdfModel(model, tree))
        cout << "INFO : KDL Tree has been loaded successfully!!" << endl;
    else {
        cerr << "ERROR : Failed to load kdl tree from urdf model!";
        return -1;
    }

    // Load KDL Chain from KDL Tree
    KDL::Chain chain;
    if (tree.getChain("robot_footprint", "tool_tip", chain))
        cout << "INFO : KDL Chain has been loaded successfully!!" << endl;
    else {
        cerr << "ERROR : Failed to load KDL Chain from KDL Tree!" << endl;
        return -1;
    }

    // Get Minimum and Maximum Joints Limits
    cout << "INFO : \rGetting Joint Limits values....";
    KDL::JntArray minLimit = KDL::JntArray(chain.getNrOfJoints());
    KDL::JntArray maxLimit = KDL::JntArray(chain.getNrOfJoints());
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
    } // delete jointTmp;
    // Print Joint Min & Max limits value
    cout << "\r  DATA : Joint Limits :                  " << endl;
    cout << setw(6) << "No" << setw(10) << "Min" << setw(5) << "  -  " << setw(8) << "Max" << endl;
    for (int i=0; i<chain.getNrOfJoints(); i++) {
        cout << setw(6) << i << setw(10) << minLimit(i) << setw(5) << "  -  " << setw(8) << (maxLimit(i)) << endl;
    }

    //Creation of the Kinematics solvers
            //Forward Kinematics Position solver
        KDL::ChainFkSolverPos_recursive fksolver(chain);		
            //Inverse Kinematics Velocity solver
        KDL::ChainIkSolverVel_pinv iksolver_v(chain);		
            // Inverse Kinematics Position solver with joint limits
            // Maximum 1000 iterations, stop at accuracy 1e-8
        // KDL::ChainIkSolverPos_NR_JL iksolver_p(chain, minLimit, maxLimit, fksolver, iksolver_v, 10000, 1e-8);    


        Eigen::Matrix<double,6,1> L;
            L(0)=1;     L(1)=1;     L(2)=1;
            L(3)=0.1;  L(4)=0.1;  L(5)=0.1;
        KDL::ChainIkSolverPos_LMA iksolver_p(chain, L);    
    

    //////////////////////////////////////
    // Test - Forward Kinematics Solver //
    //////////////////////////////////////
    cout << "\nTest FK Solver" << endl;

        if (true) {
            // Record start time
            auto _begin = chrono::high_resolution_clock::now();

            // Create joint array
            KDL::JntArray jntPosArray = KDL::JntArray(chain.getNrOfJoints());

            // Loop to assign some values to the joint positions
            // for(int i=0;i<nj;i++){
            //     float myinput = (22.0/7.0)/180.0 * 0.0;
            //     jointpositions(i) = (double)myinput;
            // }

            // Assign some values to the joint positions
            jntPosArray(0) = 0.51;
            jntPosArray(1) = -0.97;
            jntPosArray(2) = -1.44;
            jntPosArray(3) = -0.64;
            jntPosArray(4) = 0.84;
            jntPosArray(5) = 0.91;
        
            // Frame that will contain the results
            KDL::Frame cartpos;    
        
            // Calculate FK position
            double roll, pitch, yaw;    // To store Euler(Roll, Pitch and Yaw)
            double x, y, z, w;          // To store Quaternion(X, Y, Z, W)
            int kinematics_status = fksolver.JntToCart(jntPosArray, cartpos);
            cout << "INFO : FKSolver Stat - " << strError(kinematics_status) << endl;
            if(kinematics_status>=0){
                cout << "  DATA : FK Frame : " << endl;

                cartpos.M.GetRPY(roll, pitch, yaw);
                cartpos.M.GetQuaternion(x, y, z, w);
                // cout << cartpos << endl;

                cout << "      Position   : " << cartpos.p << endl;
                cout << "      Quaternion : [" << x << ", " << y << ", " << z << ", " << w << "]" << endl;
            } else {
                cerr << "ERROR : Could not calculate Forward Kinematics :(" << endl;
            }

            // Record end time
            auto _end = chrono::high_resolution_clock::now();
            chrono::duration<double> _elapsed = _end - _begin;
            cout << "INFO : FK - Elapsed Time : " << _elapsed.count() << " seconds!" << endl;
            cout << "INFO : FK - Frequency : " << 0.001/_elapsed.count() << " kHz!" << endl;
        }


    //////////////////////////////////////
    // Test - Inverse Kinematics Solver //
    //////////////////////////////////////
    cout << "\nTest IK Solver" << endl;
    
        if (true) {
            // Record start time
            auto _begin = std::chrono::high_resolution_clock::now();
            
            // Creation of Joint Array
            KDL::JntArray resJntPosArray(chain.getNrOfJoints());     // Joint Position Array that will contain the results
            KDL::JntArray curJntPosArray(chain.getNrOfJoints());     // Current/Initial Joint Position Array

            curJntPosArray(0)=(double) 22.0/7.0/180.0* -0;
            curJntPosArray(1)=(double) 22.0/7.0/180.0* +0;
            curJntPosArray(2)=(double) 22.0/7.0/180.0* -0;
            curJntPosArray(3)=(double) 22.0/7.0/180.0* +0;
            curJntPosArray(4)=(double) 22.0/7.0/180.0* -0;
            curJntPosArray(5)=(double) 22.0/7.0/180.0* +0;

            // // Create joint array
            // KDL::JntArray jntPosArray = KDL::JntArray(chain.getNrOfJoints());

            // // Assign some values to the joint positions
            // jntPosArray(0) = 0.51;
            // jntPosArray(1) = -0.97;
            // jntPosArray(2) = -1.44;
            // jntPosArray(3) = -0.64;
            // jntPosArray(4) = 0.84;
            // jntPosArray(5) = 0.91;

            // curJntPosArray(0)=(double) jntPosArray(0) + 0.51;
            // curJntPosArray(1)=(double) jntPosArray(1) + 0.51;
            // curJntPosArray(2)=(double) jntPosArray(2) + 0.51;
            // curJntPosArray(3)=(double) jntPosArray(3) + 0.51;
            // curJntPosArray(4)=(double) jntPosArray(4) + 0.51;
            // curJntPosArray(5)=(double) jntPosArray(5) + 0.51;

            // Set Destination Frame
            KDL::Frame F_dest = KDL::Frame( KDL::Rotation::Quaternion(0.322489, -0.0831008, 0.786326, 0.520372), 
                                                        KDL::Vector(-0.250856, 0.11735, 1.43523));
        
            int kinematics_status_ = iksolver_p.CartToJnt(curJntPosArray, F_dest, resJntPosArray);
            cout << "INFO : IKSolver Stat - " << strError(kinematics_status_) << endl;
            if (kinematics_status_>=0) {
                cout << "  DATA : IK Joint Angles : " << endl;
                cout << setw(6) << "No" << setw(15) << "Angle(rad)" << setw(5) << "  -  " << setw(15) << "Angle(deg)" << endl;
                for (int i=0; i<chain.getNrOfJoints(); i++) {
                    cout << setw(6) << i << setw(15) << resJntPosArray(i) << setw(5) << "  -  " << setw(15) << resJntPosArray(i) * 180.0 / (22.0/7.0) << endl;

                    // cout << resJntPosArray(i) * 180.0 / (22.0/7.0) << " \t- " << (int)(resJntPosArray(i) * 180.0 / (22.0/7.0)) ;
                    // cout << endl;
                }
            } else {
                cerr << "ERROR : Could not calculate Inverse Kinematics :(" << endl;
            }

            // Record end time
            auto _end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> _elapsed = _end - _begin;
            cout << "INFO : IK - Elapsed Time : " << _elapsed.count() << " seconds!" << endl;
            cout << "INFO : IK - Frequency : " << 0.001/_elapsed.count() << " kHz!" << endl;

            // F_dest = KDL::Frame(KDL::Rotation::Quaternion(-0.5, -0.5, -0.5, 0.5), KDL::Vector(-0.345, 0.0, 1.63785));

            // KDL::JntArray q(chain.getNrOfJoints());
            // KDL::JntArray q_init(chain.getNrOfJoints());

            // q_init(5)=(double) 22.0/7.0/180.0* 170.0;
            // kinematics_status_ = iksolver_p.CartToJnt(q_init,F_dest,q);
            // if(kinematics_status_>=0){
            // for (int i=0; i<chain.getNrOfJoints(); i++) {
            //         cout << q(i) << " "; // * 180.0 / (22.0/7.0) << " ";
            //     cout << endl;
            // }
            //     printf("%s \n","Succes, thanks KDL!");
            // }else{
            //     printf("%s \n","Error: could not calculate forward kinematics :(");
            // }
        }


    //////////////////////////////////////
    // Test - TF Listener Tool Tip Pose //
    //////////////////////////////////////
    cout << "\nTest TF Listener" << endl;

        if (true) {
            // Initialize the ROS node and nodehandle
            ros::init(argc, argv, "tf_listener");
            ros::NodeHandle nh;

            // Record start time
            auto _begin = std::chrono::high_resolution_clock::now();

            tf::TransformListener listener;

            tf::StampedTransform transform;
            try {
                listener.waitForTransform("/base_link", "/tool_tip", ros::Time(), ros::Duration(0.5));
                listener.lookupTransform("/base_link", "/tool_tip", ros::Time(0), transform);

                cout << "DATA : Position    [" << transform.getOrigin().x() << ", " << transform.getOrigin().y() << ", " << transform.getOrigin().z() << "]"<< endl;
                cout << "DATA : Orientation [" << transform.getRotation().x() << ", " << transform.getRotation().y() << ", " << transform.getRotation().z() << ", " << transform.getRotation().w() << "]"<< endl;
            }
            catch (tf::TransformException &ex) {
                cerr << "ERROR : " << ex.what() << endl;
                ros::Duration(1.0).sleep();
            }

            // Record end time
            auto _end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> _elapsed = _end - _begin;
            cout << "INFO : TF - Elapsed Time : " << _elapsed.count() << " seconds!" << endl;
            cout << "INFO : TF - Frequency : " << 0.001/_elapsed.count() << " kHz!" << endl;

        }


    //////////////////////////////////////
    // Test - Joint States Topic Listen //
    //////////////////////////////////////
    cout << "\nTest Joint States Listener" << endl;

        if (true) {
            // Initialize the ROS node and nodehandle
            ros::init(argc, argv, "joint_states_listener");
            ros::NodeHandle nh;

            // Record start time
            auto _begin = std::chrono::high_resolution_clock::now();
    
    
    
            boost::shared_ptr<const sensor_msgs::JointState> msg = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", nh);
            if (msg) cout<<"Msg received!"<<msg->name.front()<<std::endl;
            else cout<<"No message!"<<std::endl;

            for (int i=0; i<msg->name.size(); i++) {
                cout << "\tJName : " << msg->name[i] << endl;
                cout << "\tPosition : " << msg->position[i] << endl;
            }

            // boost::shared_ptr<sensor_msgs::JointState> joint_states = ros::topic::waitForMessage("/joint_states", nh);

            // cout << "DATA : " << joint_states.position[0] << endl;
            // cout << "DATA : " << joint_states.velocity << endl;



            // Record end time
            auto _end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> _elapsed = _end - _begin;
            cout << "INFO : TF - Elapsed Time : " << _elapsed.count() << " seconds!" << endl;
            cout << "INFO : TF - Frequency : " << 0.001/_elapsed.count() << " kHz!" << endl;

        }


    //////////////////////////////////////
    // Test - Joint States - Publisher  //
    //////////////////////////////////////
    cout << "\nTest Joint States Publisher" << endl;

        if (true) {
            // Initialize the ROS node and nodehandle
            ros::init(argc, argv, "joint_states_listener");
            ros::NodeHandle nh;

            ros::Publisher jointStatePublisher;
            sensor_msgs::JointState jointState;

            // Record start time
            auto _begin = std::chrono::high_resolution_clock::now();
    
            // jointStatePublisher = nh.advertise<sensor_msgs::JointState>("joint_state", 1);

            // jointState.name.resize(chain.getNrOfJoints());
            // jointState.position.resize(chain.getNrOfJoints());
            // jointState.velocity.resize(0);
            // jointState.effort.resize(0);

            // jointState.name = {"Joint_1", "Joint_2", "Joint_3", "Joint_4", "Joint_5", "Joint_6"};
            
            // jointState.position = {-2.29277, -1.74075, -1.45238,  1.28662, -1.65377, 0.741108};

            // jointStatePublisher.publish(jointState);

            // Record end time
            auto _end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> _elapsed = _end - _begin;
            cout << "INFO : TF - Elapsed Time : " << _elapsed.count() << " seconds!" << endl;
            cout << "INFO : TF - Frequency : " << 0.001/_elapsed.count() << " kHz!" << endl;

        }

 




    return 0;
}