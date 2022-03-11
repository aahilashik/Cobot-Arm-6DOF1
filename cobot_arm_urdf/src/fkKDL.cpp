#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <stdio.h>
#include <iostream>
#include <chrono>

#include <kdl_parser/kdl_parser.hpp>
// #include <kdl/treefksolverpos_recursive.hpp>

#include <kdl/chainiksolver.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>


using namespace KDL;
using namespace std;
 
 
int main( int argc, char** argv )
{
    char filename[] = "/home/ubuntussd/test_ws/src/Cobot_Arm_URDF/urdf/Cobot_Arm_URDF.urdf";
    KDL::Tree tree;
    if (!kdl_parser::treeFromFile(filename, tree)){
        cout <<"Failed to construct kdl tree" ;
        return false;
    }

    Chain chain;
    tree.getChain("robot_footprint", "tool_tip", chain);

    // TreeFkSolverPos_recursive fk_solver(tree);
    // ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);


    // // Definition of a kinematic chain & add segments to the chain
    // Chain chain;
    // chain.addSegment(Segment(Joint(Joint::RotX), Frame(Rotation::RPY(0, 0, 0), Vector(0.0, 0.0, 0.0))));        // Global Frame 
    // chain.addSegment(Segment(Joint(Joint::RotX), Frame(Rotation::RPY(, 0, 0), Vector(-0.2, 0.0, 0.02))));      // Base Link (Link 0)
    // chain.addSegment(Segment(Joint(Joint::RotY), Frame(Rotation::RPY(0, 0, 0), Vector(0.0, 0.16, 0.0))));       // Link 1
    // chain.addSegment(Segment(Joint(Joint::RotY), Frame(Rotation::RPY(0, 0, 0), Vector(0.0, 0.336, 0.0))));      // Link 2
    // chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Rotation::RPY(0, 0, 0), Vector(0.0, 0.0, -0.5))));       // Link 3
    // chain.addSegment(Segment(Joint(Joint::RotX), Frame(Rotation::RPY(0, 0, 0), Vector(0.19746, 0.145, 0.0))));  // Link 4
    // chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Rotation::RPY(0, 0, 0), Vector(0.23, 0.0, 0.0))));       // Link 5
    // chain.addSegment(Segment(Joint(Joint::RotX), Frame(Rotation::RPY(0, 0, 0), Vector(0.12, 0.0, 0.0))));       // End Tool
    // chain.addSegment(Segment(Joint(Joint::RotX), Frame(Rotation::RPY(0, 0, 0), Vector(0.07439, 0.0, 0.0))));    // Tool Tip



    // //Definition of a kinematic chain & add segments to the chain
    // Chain chain;
    // chain.addSegment(Segment(Joint(Joint::None), Frame(Rotation::RPY(0.0, 0.0, 0.0), Vector(0.0, 0.0, 0.0))));	                                // base_link
    // chain.addSegment(Segment(Joint(Joint::None), Frame(Rotation::RPY(1.5708, 0.0, 0.0), Vector(-0.2, 0.0, 0.02))));	                            // Link_0
    
    // chain.addSegment(Segment(Joint(Joint::RotY), Frame(Rotation::RPY(0.0, 0.0, 0.0), Vector(0.0, 0.16, 0.0))));	                                // Link_1
    // chain.addSegment(Segment(Joint(Joint::RotY), Frame(Rotation::RPY(-1.5708, 0.0, 3.1416), Vector(0.0, 0.336, 0.0))));	                // Link_2
    // chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Rotation::RPY(1.5708, 1.5708, 0), Vector(0.0, 0.0, -0.5))));	                            // Link_3
    // chain.addSegment(Segment(Joint(Joint::RotX), Frame(Rotation::RPY(0.0, 0.0, 0.0), Vector(0.19746, 0.145, 0.0))));	                        // Link_4
    // chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Rotation::RPY(0.0, 0.0, 0.0), Vector(0.23, 0.0, 0.0))));	                                // Link_5
    // chain.addSegment(Segment(Joint(Joint::RotX), Frame(Rotation::RPY(0.0, 0.0, 0.0), Vector(0.12, 0.0, 0.0))));	                                // End_Tool
 
    // chain.addSegment(Segment(Joint(Joint::None), Frame(Rotation::RPY(0.0, 0.0, 0.0), Vector(0.07439, 0.0, 0.0))));	                            // tool_tip



    // //Definition of a kinematic chain & add segments to the chain
    // Chain chain;

    // chain.addSegment(Segment(Joint(Joint::None), Frame(Rotation::RPY(0.0, -0.0, 0.0), Vector(0.0, 0.0, 0.0))));	// base_link
    // chain.addSegment(Segment(Joint(Joint::None), Frame(Rotation::RPY(1.5708, -0.0, 0.0), Vector(-0.2, 0.0, 0.02))));	// Link_0

    // chain.addSegment(Segment(Joint(Joint::RotY), Frame(Rotation::RPY(0.0, -0.0, 0.0), Vector(0.0, 0.16, 0.0))));	// Link_1
    // chain.addSegment(Segment(Joint(Joint::RotY), Frame(Rotation::RPY(-1.5708, 8.47032947254e-22, -3.14158530718), Vector(0.0, 0.336, 0.0))));	// Link_2
    // chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Rotation::RPY(-1.57079625922, 1.57059665359, 3.14158904796), Vector(0.0, 0.0, -0.5))));	// Link_3
    // chain.addSegment(Segment(Joint(Joint::RotX), Frame(Rotation::RPY(0.0, -0.0, 0.0), Vector(0.19746, 0.145, 0.0))));	// Link_4
    // chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Rotation::RPY(0.0, -0.0, 0.0), Vector(0.23, 0.0, 0.0))));	// Link_5
    // chain.addSegment(Segment(Joint(Joint::RotX), Frame(Rotation::RPY(0.0, -0.0, 0.0), Vector(0.12, 0.0, 0.0))));	// End_Tool

    // chain.addSegment(Segment(Joint(Joint::None), Frame(Rotation::RPY(0.0, -0.0, 0.0), Vector(0.07439, 0.0, 0.0))));	// tool_tip


    // //Definition of a kinematic chain & add segments to the chain
    // Chain chain;
    // chain.addSegment(Segment(Joint(Joint::None), Frame(Rotation::RPY(0.0, 0.0, 0.0), Vector(0.0, 0.0, 0.0))));	// base_link
    // chain.addSegment(Segment(Joint(Joint::None), Frame(Rotation::RPY(0.0, 0.0, 0.0), Vector(-0.2, 0.0, 0.02))));	// Link_0

    // chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Rotation::RPY(0.0, 0.0, 0.0), Vector(0.0, 0.0, 0.16))));	// Link_1
    // chain.addSegment(Segment(Joint(Joint::RotY), Frame(Rotation::RPY(0.0, 0.0, 0.0), Vector(0.0, 0.0, 0.336))));	// Link_2
    // chain.addSegment(Segment(Joint(Joint::RotY), Frame(Rotation::RPY(0.0, 0.0, 0.0), Vector(0.0, 0.0, 0.5))));	// Link_3
    // chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Rotation::RPY(0.0, 0.0, 0.0), Vector(-0.145, 0.0, 0.19749))));	// Link_4
    // chain.addSegment(Segment(Joint(Joint::RotY), Frame(Rotation::RPY(0.0, 0.0, 0.0), Vector(0.0, 0.0, 0.23))));	// Link_5
    // chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Rotation::RPY(0.0, 0.0, 0.0), Vector(0.0, 0.0, 0.12))));	// End_Tool

    // chain.addSegment(Segment(Joint(Joint::None), Frame(Rotation::RPY(0.0, 0.0, 0.0), Vector(0.0, 0.0, 0.07439))));	// tool_tip

                                                                        // plot3DFrame(0, 0, 0)
                                                                        // plot3DFrame(-0.2, 0, 0.02)

                                                                        // plot3DFrame(-0.2, 0, 0.18)
                                                                        // plot3DFrame(-0.2, 0, 0.516)
                                                                        // plot3DFrame(-0.2, 0, 1.016)
                                                                        // plot3DFrame(-0.345, 0, 1.21349)
                                                                        // plot3DFrame(-0.345, 0, 1.44349)
                                                                        // plot3DFrame(-0.345, 0, 1.56349)

                                                                        // plot3DFrame(-0.345, 0, 1.63788)

    JntArray minLimit = JntArray(chain.getNrOfJoints());
    JntArray maxLimit = JntArray(chain.getNrOfJoints());

    // Assign some values to the joint positions
    for(unsigned int i=0;i<chain.getNrOfJoints();i++){
        minLimit(i)=(double) 22.0/7.0/180.0* -190;
        maxLimit(i)=(double) 22.0/7.0/180.0* 190;
    }

    // Create solver based on kinematic chain
    ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);
    ChainIkSolverVel_pinv iksolver_v(chain);		//Inverse velocity solver
    ChainIkSolverPos_NR_JL iksolver_p(chain, minLimit, maxLimit, fksolver, iksolver_v, 10000, 1e-8);//Maximum 100 iterations, stop at accuracy 1e-6

    auto _begin = chrono::high_resolution_clock::now();

    // Create joint array
    unsigned int nj = chain.getNrOfJoints();
    JntArray jointpositions = JntArray(nj);

    // // Assign some values to the joint positions
    // for(unsigned int i=0;i<nj;i++){
    //     float myinput;
    //     //printf ("Enter the position of joint %i: ",i);
    //     // scanf ("%e",&myinput);
    // 	myinput = 0.0; // (22.0/7.0)/180.0 * 0.0;
    //     jointpositions(i) = (double)myinput;
    // }

    cout << nj << endl;
    jointpositions(0) = 0.0;
    jointpositions(1) = 0.0;
    jointpositions(2) = 0.0;
    jointpositions(3) = 0.0;
    jointpositions(4) = 0.0;
    jointpositions(5) = 0.0;

	// jointpositions(0)=(double)22.0/7.0 / 180.0 * 45;
	// jointpositions(1)=(double)22.0/7.0 / 180.0 * 0.0;
	// jointpositions(2)=(double)22.0/7.0 / 180.0 * 0.0;
 
    // Create the frame that will contain the results
    Frame cartpos;    
 
    // Calculate forward position kinematics
    bool kinematics_status;
    double roll,pitch,yaw;
    double x, y, z, w;
    kinematics_status = fksolver.JntToCart(jointpositions,cartpos);
    if(kinematics_status>=0){

        cartpos.M.GetRPY(roll,pitch,yaw);
        cartpos.M.GetQuaternion(x, y, z, w);
        // cout << cartpos << endl;
        cout << "Quaternion \t: [" << x << ", " << y << ", " << z << ", " << w << "]" << endl;

        cout << "Position \t: " << cartpos.p << endl;

        printf("%s \n","Succes, thanks KDL!");
    } else {
        printf("%s \n","Error: could not calculate forward kinematics :(");
    }

    auto _end = chrono::high_resolution_clock::now();
    chrono::duration<double> _elapsed = _end - _begin;
    cout << "INFO : Elapsed Time : " << _elapsed.count() << " seconds!" << endl;
    cout << "INFO : Frequency : " << 0.001/_elapsed.count() << " kHz!" << endl;


    //Set destination frame
    Frame F_dest = Frame(Rotation::Quaternion(-0.5, -0.5, -0.5, 0.5), Vector(-0.345, 0.0, 1.63785));;

    JntArray q(chain.getNrOfJoints());
    JntArray q_init(chain.getNrOfJoints());

    q_init(5)=(double) 22.0/7.0/180.0* 170.0;
    kinematics_status = iksolver_p.CartToJnt(q_init,F_dest,q);
    if(kinematics_status>=0){
	for (int i=0; i<chain.getNrOfJoints(); i++) {
            cout << q(i) << " "; // * 180.0 / (22.0/7.0) << " ";
	    cout << endl;
	}
        printf("%s \n","Succes, thanks KDL!");
    }else{
        printf("%s \n","Error: could not calculate forward kinematics :(");
    }



}

// g++ -L/opt/ros/melodic/lib -I/opt/ros/melodic/include -I/usr/include/eigen3 fkKDL.cpp -lorocos-kdl -lkdl_parser -o main && ./main


