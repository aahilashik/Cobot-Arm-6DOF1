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

// To retrieve Robot Model from ROS Parameter Server
bool use_robot_description  = true;
string robot_desc_param     = "robot_description";

int main(int argc, char **argv) {

    /************************** Model **************************/
    // Load URDF model/KDL tree from the parameter server(if true) or from urdf file(if false)
    urdf::Model model;

    // Initialize the ROS node and nodehandle
    ros::init(argc, argv, "robot_desc_retriever");
    ros::NodeHandle nh;
    
    // Load URDF Model from ROS parameter server
    if (!model.initParamWithNodeHandle(robot_desc_param, nh)){
        cerr << "ERROR : Failed to retrieve robot model from ROS parameter server";
        return false;
    }

    /************************** Tree **************************/
    // Load KDL Tree from urdf model
    KDL::Tree tree;
    if (kdl_parser::treeFromUrdfModel(model, tree))
        cout << "INFO : KDL Tree has been loaded successfully!!" << endl;
    else {
        cerr << "ERROR : Failed to load kdl tree from urdf model!";
        return -1;
    }

    /************************** Chain **************************/
    // Load KDL Chain from KDL Tree
    KDL::Chain chain;
    if (tree.getChain("robot_footprint", "tool_tip", chain))
        cout << "INFO : KDL Chain has been loaded successfully!!" << endl;
    else {
        cerr << "ERROR : Failed to load KDL Chain from KDL Tree!" << endl;
        return -1;
    }

    /************************** Joint Limits **************************/
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
    }        
    // minLimit(5) = 0.0;
    // maxLimit(5) = 0.0;

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
        KDL::ChainIkSolverPos_NR_JL iksolver_p(chain, minLimit, maxLimit, fksolver, iksolver_v, 100, 1e-8 );    

        // Eigen::Matrix<double,6,1> L;
        //     L(0)=1;     L(1)=1;     L(2)=1;
        //     L(3)=0.1;  L(4)=0.1;  L(5)=0.1;
        // KDL::ChainIkSolverPos_LMA iksolver_p(chain, L);


    //////////////////////////////////////
    // Test - Inverse Kinematics Solver //
    //////////////////////////////////////
    cout << "\nTest IK Solver" << endl;
    
        if (true) {
            // Record start time
            auto _begin = std::chrono::high_resolution_clock::now();
            




                double PI = 3.141592653589;
                int num_of_trials = 1000; 
                int n = chain.getNrOfJoints();
                KDL::JntArray q(n);
                KDL::JntArray q_init(n);
                KDL::JntArray q_sol(n);
                int count = 0;
                // KDL::Frame F_dest = KDL::Frame( KDL::Rotation::RPY(90, 0, -45), KDL::Vector(0.4, 0.4, 0.8));
                KDL::Frame F_dest = KDL::Frame( KDL::Rotation::Quaternion(0.406661, 0.713477, 0.0505514, 0.56835), KDL::Vector(-0.156785, 0.292287, 0.630705));
                int trial;
                for (trial=0;trial<num_of_trials;++trial) {
                    // q.data.setRandom();
                    // q.data *= PI;
                    // q_init.data.setRandom();
                    // q_init.data *= PI/2.0;
                    // q_init(5) = 0.0;
                    // for(int i=0; i<n; i++)
                    //     cout << "\tInit J" << q_init(i) << endl;
                    // cout << "Is qinti " << (minLimit.data) << endl;

                    // cout << "Randd : " << rand() << endl;
                    // Loop to assign some values to the joint positions
                    for(int i=0; i<n; i++){
                        double myinput = (minLimit(i)*1000.0) + ( std::rand() % int( maxLimit(i)*1000.0 - minLimit(i)*1000.0 + 1 ) );
                        q_init(i) = (double)myinput/1000.0;
                        
                    }


                    // q_init(0) += -0.797338;
                    // q_init(1) += 1.0416;
                    // q_init(2) += -1.62159;
                    // q_init(3) += 0.248595;
                    // q_init(4) += -0.311472;
                    // q_init(5) += -0.598789;
                    


                    KDL::Frame pos_goal,pos_reached;
                    //solver.compute_fwdpos(q.data);
                    //pos_goal = solver.T_base_head;
                    int retval = iksolver_p.CartToJnt(q_init, F_dest, q_sol);
                    if (retval==0) count++;
                    double roll, pitch, yaw;    // To store Euler(Roll, Pitch and Yaw)
                    double x, y, z, w;          // To store Quaternion(X, Y, Z, W)
                    if(retval==0){
                        cout << "  DATA : IK Joint Angles : " << endl;
                        cout << setw(6) << "No" << setw(15) << "Angle(rad)" << setw(5) << "  -  " << setw(15) << "Angle(deg)" << endl;
                        for (int i=0; i<chain.getNrOfJoints(); i++) {
                            cout << setw(6) << i << setw(15) << q_sol(i) << setw(5) << "  -  " << setw(15) << q_sol(i) * 180.0 / (22.0/7.0) << endl;
                        }
                        break;
                    }
                    // cout << "INFO : It - " << trial+1 << " | IKSolver Stat - " << strError(retval) << endl;
                    
                }
                cout << trial << "No. of Correct : " << count << endl;
                cout << q_sol.data << endl;

            // Record end time
            auto _end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> _elapsed = _end - _begin;
            cout << "INFO : IK - Elapsed Time : " << _elapsed.count() << " seconds!" << endl;
            cout << "INFO : IK - Frequency : " << 0.001/_elapsed.count() << " kHz!" << endl;
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