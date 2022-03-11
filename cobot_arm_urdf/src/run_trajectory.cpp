/*
-------------------------------------
  Control the 6DOF Arm/Manipulator!
-------------------------------------
  Key Controls :
  ------------          
    q     w     e                  Y-     Z+    X-
      \   |   /                      \    |    /
        \ | /                          \  |  /
 a/A --- s/S --- d/D       roll(+/-) --- pitch(+/-) --- yaw(+/-)
        / | \                          /  |  \
      /   |   \                      /    |    \
    z     x     c                  X+     Z-    Y+

  Steps Selections :
  ----------------
    Options (For Each Axis)
        Linear  : ( 1.0mm / 5.0mm / 10.0mm / 50.0mm / 100.0mm )
        Angular : ( 1 deg / 5 deg / 10 deg / 50 deg / 100 deg )

    Default (For Each Axis)
        Linear  :  10.0 mm | Angular :  10.0 deg

    E/Z : Decrease/Increase the X-axis Linear Steps
    Q/C : Decrease/Increase the Y-axis Linear Steps
    X/W : Decrease/Increase the Z-axis Linear Steps 
     S  : Set new Home location with current pose
    R/r : Decrease/Increase the X-axis Angular Steps
    T/t : Decrease/Increase the Y-axis Angular Steps
    Y/y : Decrease/Increase the Z-axis Angular Steps

  Ctrl + C to Quit
*/

// C C++ Headers
#include <iostream>
#include <stdio.h>

// Terminal Mode Handler
#include <termios.h>

// Keyboard Interrupt Handler
#include <signal.h>

// KDL Variables
#include <kdl/frames_io.hpp>

// TF Processing
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>

// Filesystem Handling
#include <fstream>
#include <sys/stat.h>
#include <ros/package.h>

// Trajectory
#include <kdl/trajectory.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/trajectory_stationary.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/utilities/error.h>


using namespace std;  

// Workspace Limits
int minX =   90, maxX = 680;
int minY = -600, maxY = 600;
int minZ =    0, maxZ = 1160;


ros::Publisher armPosePublisher;
geometry_msgs::Pose currPose;

string baseFrame  = "/base_link";
string endFrame   = "/tool_tip";

string moduleName = "cobot_arm_urdf";

void quit(int sig) {
    cout << "WARN : Keyboard Interrupt Detected..!" << endl;
    exit(0);
}

void clearNTerminalLines(int NUM_LINES) {
    char terminal_clearline [4];
    char terminal_moveup [4];

    sprintf(terminal_clearline, "%c[2K", 0x1B);
    sprintf(terminal_moveup, "%c[1A", 0x1B);

    for (int i=0; i<NUM_LINES; i++)
        cout << terminal_moveup << terminal_clearline << endl;
}

void publishPose(KDL::Frame frame) {
    currPose.position.x = frame.p.x();
    currPose.position.y = frame.p.y();
    currPose.position.z = frame.p.z();

    double qX, qY, qZ, qW;
    frame.M.GetQuaternion(qX, qY, qZ, qW);
    currPose.orientation.x = qX;
    currPose.orientation.y = qY;
    currPose.orientation.z = qZ;
    currPose.orientation.w = qW;

    armPosePublisher.publish(currPose);
}

tf::TransformListener* listener;

void getCurrentPose() {
  tf::StampedTransform transform;
  try {
    listener->waitForTransform(baseFrame, endFrame, ros::Time(), ros::Duration(0.5));
    listener->lookupTransform(baseFrame, endFrame, ros::Time(0), transform);

    currPose.position.x = transform.getOrigin().x();
    currPose.position.y = transform.getOrigin().y();
    currPose.position.z = transform.getOrigin().z();
    currPose.orientation.x = transform.getRotation().x();
    currPose.orientation.y = transform.getRotation().y();
    currPose.orientation.z = transform.getRotation().z();
    currPose.orientation.w = transform.getRotation().w();
  } catch (tf::TransformException &ex) {
    cerr << "ERROR : " << ex.what() << endl;
  }
  return;
}

void generateTrajectory() {
    try {
		KDL::Path_RoundedComposite* path = new Path_RoundedComposite(0.2, 0.01, new KDL::RotationalInterpolation_SingleAxis());

		path->Add(Frame(Rotation::RPY(M_PI,0,0), Vector(-1,0,0)));
		path->Add(Frame(Rotation::RPY(M_PI/2,0,0), Vector(-0.5,0,0)));
		path->Add(Frame(Rotation::RPY(0,0,0), Vector(0,0,0)));
		path->Add(Frame(Rotation::RPY(0.7,0.7,0.7), Vector(1,1,1)));
		path->Add(Frame(Rotation::RPY(0,0.7,0), Vector(1.5,0.3,0)));
		path->Add(Frame(Rotation::RPY(0.7,0.7,0), Vector(1,1,0)));

		path->Finish(); 
        		
        KDL::VelocityProfile* velpref = new KDL::VelocityProfile_Trap(0.5,0.1);
		velpref->SetProfile(0,path->PathLength());  
		KDL::Trajectory* traject = new KDL::Trajectory_Segment(path, velpref);

		for (double t=0.0; t <= traject->Duration(); t+= dt) {
			Frame nextPose;
			nextPose = traject->Pos(t);
		}
        
        delete traject;
	} catch(Error& error) {
		cout << "Error : Trajectory - " << error.Description() << endl;
		cout << "\twith the following type " << error.GetType() << endl;
	}

}


void keyLoop() {


    cout << "-------------------------------------" << endl;
    cout << "  Control the 6DOF Arm/Manipulator!" << endl;
    cout << "-------------------------------------" << endl;
    cout << "  Key Controls :" << endl;
    cout << "  ------------" << endl;
    cout << "    q     w     e                  Y-     Z+    X-" << endl;
    cout << "      \\   |   /                      \\    |    /" << endl;
    cout << "        \\ | /                          \\  |  /" << endl;
    cout << " a/A --- s/S --- d/D    roll(+/-) --- pitch(+/-) --- yaw(+/-)" << endl;
    cout << "        / | \\                          /  |  \\" << endl;
    cout << "      /   |   \\                      /    |    \\" << endl;
    cout << "    z     x     c                  X+     Z-    Y+" << endl;
    cout << endl;
    cout << "  Steps Selections :" << endl;
    cout << "  ----------------" << endl;
    cout << "  -- Options (For Each Axis)" << endl;
    cout << "        Linear  : ( 1.0mm / 5.0mm / 10.0mm / 50.0mm / 100.0mm )" << endl;
    cout << "        Angular : ( 1 deg / 5 deg / 10 deg / 50 deg / 100 deg )" << endl;
    cout << endl;
    cout << "  -- Default (For Each Axis)" << endl;
    cout << "        Linear  :  10.0 mm | Angular :  10.0 deg" << endl;
    cout << endl;
    cout << "    E/Z : Decrease/Increase the X-axis Linear Steps" << endl;
    cout << "    Q/C : Decrease/Increase the Y-axis Linear Steps" << endl;
    cout << "    X/W : Decrease/Increase the Z-axis Linear Steps " << endl;
    cout << "    R/r : Decrease/Increase the X-axis Angular Steps " << endl;
    cout << "    T/t : Decrease/Increase the Y-axis Angular Steps " << endl;
    cout << "    Y/y : Decrease/Increase the Z-axis Angular Steps " << endl;
    cout << "     s  : Go to Home location" << endl;
    cout << "     S  : Set new Home location with current pose" << endl;
    cout << endl;
    cout << "  Ctrl + C to Quit" << endl;
    cout << endl;
    cout << "------------------------------------------------------" << endl;
    cout << endl;

  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  char key;
  double pX, pY, pZ;            // Position
  double eR, eP, eY;            // Euler Angles
  double qX, qY, qZ, qW;        // Quaternion Angles

  pX = currPose.position.x * 1000.0;
  pY = currPose.position.y * 1000.0;
  pZ = currPose.position.z * 1000.0;  // metre to mm conversion
  qX = currPose.orientation.x;
  qY = currPose.orientation.y;
  qZ = currPose.orientation.z;
  qW = currPose.orientation.w;

  KDL::Frame currFrame = KDL::Frame( KDL::Rotation::Quaternion(qX, qY, qZ, qW), 
                                        KDL::Vector(pX, pY, pZ));

  currFrame.M.GetRPY(eR, eP, eY);
  eR = KDL::rad2deg * eR;   // rad to deg conversion
  eP = KDL::rad2deg * eP;
  eY = KDL::rad2deg * eY;

  while (ros::ok()) {
    // cout << "  Current : Linear Steps = (" << dx << ", " << dy << ", " << dz << ") mm | Angular Step =  (" << dR << ", " << dP << ", " << dY << ") deg" << endl;
    cout << "  Position     X: " << pX << " | Y: " << pY << " | Z: " << pZ << endl;
    cout << "  Orientation  R: " << eR << " | P: " << eP << " | Y: " << eY << endl;
    cout << endl;

    clearNTerminalLines(4);
  

    pX = min(max(int(pX), minX), maxX);
    pY = min(max(int(pY), minY), maxY);
    pZ = min(max(int(pZ), minZ), maxZ);

    eR = ( ((int)eR/180)%2 ? ((int)eR%180)+(eR<0?180:-180) : ((int)eR%180) );
    eP = ( ((int)eP/180)%2 ? ((int)eP%180)+(eP<0?180:-180) : ((int)eP%180) );
    eY = ( ((int)eY/180)%2 ? ((int)eY%180)+(eY<0?180:-180) : ((int)eY%180) );

    currFrame = KDL::Frame( KDL::Rotation::RPY(KDL::deg2rad*eR, KDL::deg2rad*eP, KDL::deg2rad*eY),
                                        KDL::Vector(pX/1000.0, pY/1000.0, pZ/1000.0));

    publishPose(currFrame);
    
  }

  return;
}

int main(int argc, char** argv) {

  // Initialize the ROS node and nodehandle
  ros::init(argc, argv, "arm_teleop_keyboard");
  ros::NodeHandle nh;

  // cmd_pose Publisher
  armPosePublisher = nh.advertise<geometry_msgs::Pose>("/cobot_arm/cmd_pose", 10);

  // Keyboard interrupt Handler
  signal(SIGINT, quit);

  currPose.position.x = 0.0;
  currPose.position.y = 0.0;
  currPose.position.z = 0.0;
  currPose.orientation.x = 0.0;
  currPose.orientation.y = 0.0;
  currPose.orientation.z = 0.0;
  currPose.orientation.w = 1.0;

      
  return(0);
}


