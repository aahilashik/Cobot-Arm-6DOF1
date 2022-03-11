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

// Threading
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"

// KDL Variables
#include <kdl/frames_io.hpp>

// TF Processing
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>

// Filesystem Handling
#include <fstream>
#include <sys/stat.h>
#include <ros/package.h>

using namespace std;  

// Keyboard Button/Character Values
#define KEYCODE_Q 0x51 
#define KEYCODE_q 0x71
#define KEYCODE_W 0x57
#define KEYCODE_w 0x77
#define KEYCODE_E 0x45
#define KEYCODE_e 0x65
#define KEYCODE_A 0x41
#define KEYCODE_a 0x61
#define KEYCODE_S 0x53
#define KEYCODE_s 0x73
#define KEYCODE_D 0x44
#define KEYCODE_d 0x64
#define KEYCODE_Z 0x5A
#define KEYCODE_z 0x7A
#define KEYCODE_X 0x58
#define KEYCODE_x 0x78
#define KEYCODE_C 0x43
#define KEYCODE_c 0x63
#define KEYCODE_H 0x48
#define KEYCODE_h 0x68
#define KEYCODE_P 0x50
#define KEYCODE_p 0x70
#define KEYCODE_R 0x52
#define KEYCODE_r 0x72
#define KEYCODE_T 0x54
#define KEYCODE_t 0x74
#define KEYCODE_Y 0x59
#define KEYCODE_y 0x79

// Workspace Limits
int minX =   90, maxX = 680;
int minY = -600, maxY = 600;
int minZ =    0, maxZ = 1160;


ros::Publisher armPosePublisher;
geometry_msgs::Pose currPose;
geometry_msgs::Pose homePose;

ros::Publisher surfacePublisher;
visualization_msgs::Marker surface;

string baseFrame  = "/base_link";
string endFrame   = "/tool_tip";

string moduleName = "cobot_arm_urdf";

string folderPath     = ros::package::getPath(moduleName);
string homePoseFile   = folderPath + "/cache/homePose.txt";
string surfaceFile    = folderPath + "/cache/surface.txt";

int kfd = 0;
struct termios cooked, raw;
void quit(int sig) {
  tcsetattr(kfd, TCSANOW, &cooked);
  exit(0);
}

int arr[3] = {0, 0, 0};
bool checkArrowKeys(char c) {
    for (int i=0; i<2; i++)
        swap(arr[i], arr[i+1]);

    if ((arr[0]==0x1B) & (arr[1]==0x5B))
      return true;

    arr[2] = c;
    return false;
}

// void swap_front(char c) {
//     for (int i=0; i<2; i++)
//         swap(arr[i], arr[i+1]);
//     arr[2] = c;

//     for(int i = 0; i < 3; i++)
//         cout << arr[i] << " ";
//     cout << endl;
// }

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

///////////////////////// Log /////////////////////////
// bool isPathExists(string path)
// {
//     struct stat buffer;
//     return (stat (path.c_str(), &buffer) == 0);
// }
auto isPathExists = [](string path) {
  struct stat buffer;
  return (stat (path.c_str(), &buffer) == 0);
};

bool createCacheDir(string dirPath) {
    if (isPathExists(dirPath))
        cout << "INFO : Cache Directory already exists!!" << endl;
    else {
        if (mkdir(dirPath.c_str(), 0777) == -1)
            cerr << "ERROR : Cache Directory not created!!" << endl;
        else
            cout << "INFO : Cache Directory created!!" << endl;
    }
}

void saveHomePoseToFile(KDL::Frame pose) {
  double pX, pY, pZ;
  pX = pose.p.x();
  pY = pose.p.y();
  pZ = pose.p.z();

  double qX, qY, qZ, qW;
  pose.M.GetQuaternion(qX, qY, qZ, qW);

  ostringstream dataStream;
  dataStream << pX << " " << pY << " " << pZ << " " << qX << " " << qY << " " << qZ << " " << qW;
  string data = dataStream.str();

  ofstream outFile(homePoseFile);
  outFile << data;
  outFile.close();
}

KDL::Frame loadHomePoseFromFile(){
  if (!isPathExists(homePoseFile)) {
    return KDL::Frame( KDL::Rotation::RPY(0.0, 0.0, 0.0), 
                              KDL::Vector(0.0, 0.0, 0.0));
  }
  ifstream inFile; 
  stringstream data;
  inFile.open(homePoseFile);
  data << inFile.rdbuf();
  istringstream dataStream(data.str());

  double pX, pY, pZ;
  double qX, qY, qZ, qW;

  dataStream >> pX;
  dataStream >> pY;
  dataStream >> pZ;
  dataStream >> qX;
  dataStream >> qY;
  dataStream >> qZ;
  dataStream >> qW;

  return KDL::Frame( KDL::Rotation::Quaternion(qX, qY, qZ, qW), 
                                      KDL::Vector(pX, pY, pZ));
}

void saveSurfacePoseToFile(KDL::Frame pose) {
  double pX, pY, pZ;
  pX = pose.p.x();
  pY = pose.p.y();
  pZ = pose.p.z();

  double qX, qY, qZ, qW;
  pose.M.GetQuaternion(qX, qY, qZ, qW);

  ostringstream dataStream;
  dataStream << pX << " " << pY << " " << pZ << " " << qX << " " << qY << " " << qZ << " " << qW;
  string data = dataStream.str();

  ofstream outFile(surfaceFile);
  outFile << data;
  outFile.close();
}

KDL::Frame loadSurfacePoseFromFile(){
  if (!isPathExists(surfaceFile)) {
    return KDL::Frame( KDL::Rotation::RPY(M_PI/2.0, 0.0, 0.0), 
                              KDL::Vector(0.68, 0.0, 0.55));
  }
  ifstream inFile; 
  stringstream data;
  inFile.open(surfaceFile);
  data << inFile.rdbuf();
  istringstream dataStream(data.str());

  double pX, pY, pZ;
  double qX, qY, qZ, qW;

  dataStream >> pX;
  dataStream >> pY;
  dataStream >> pZ;
  dataStream >> qX;
  dataStream >> qY;
  dataStream >> qZ;
  dataStream >> qW;

  return KDL::Frame( KDL::Rotation::Quaternion(qX, qY, qZ, qW), 
                                      KDL::Vector(pX, pY, pZ));
}
///////////////////////// End /////////////////////////

void publishWorkspace(KDL::Frame frame) {
  surface.header.frame_id = "robot_footprint";
  surface.header.stamp = ros::Time::now();

  surface.ns = "A3 WorkSpace";
  surface.id = 0;

  surface.type = visualization_msgs::Marker::CUBE;
  surface.action = visualization_msgs::Marker::ADD;

  surface.pose.position.x = frame.p.x();
  surface.pose.position.y = frame.p.y();
  surface.pose.position.z = frame.p.z();

  double qX, qY, qZ, qW;
  frame.M.GetQuaternion(qX, qY, qZ, qW);
  surface.pose.orientation.x = qX;
  surface.pose.orientation.y = qY;
  surface.pose.orientation.z = qZ;
  surface.pose.orientation.w = qW;

  // A3 Paper Size - 420x297x10(mm)
  surface.scale.x = 0.010;
  surface.scale.y = 0.297;
  surface.scale.z = 0.420;

  surface.color.r = 1.0f;
  surface.color.g = 1.0f;
  surface.color.b = 1.0f;
  surface.color.a = 0.5;

  surface.lifetime = ros::Duration();

  surfacePublisher.publish(surface);
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
    cout << "     p  : Go to Work Space location" << endl;
    cout << "     P  : Set new Work Space location with Current Pose" << endl;
    cout << endl;
    cout << "  Ctrl + C to Quit" << endl;
    cout << endl;
    cout << "------------------------------------------------------" << endl;
    cout << endl;

  // Get the console in Raw Mode                                                              
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
  double dx, dy, dz;            // Change in Position
  double dR, dP, dY;            // Change in Angles
  int xIdx=2, yIdx=2, zIdx=2;   // Linear Step Index
  int RIdx=2, PIdx=2, YIdx=2;   // Angular Step Index
  double dSteps[5] = {1.0, 5.0, 10.0, 50.0, 100.0};

  pX = currPose.position.x * 1000.0;
  pY = currPose.position.y * 1000.0;
  pZ = currPose.position.z * 1000.0;  // metre to mm conversion
  qX = currPose.orientation.x;
  qY = currPose.orientation.y;
  qZ = currPose.orientation.z;
  qW = currPose.orientation.w;

  KDL::Frame currFrame = KDL::Frame( KDL::Rotation::Quaternion(qX, qY, qZ, qW), 
                                        KDL::Vector(pX, pY, pZ));
  KDL::Frame homeFrame = loadHomePoseFromFile();
  KDL::Frame surfFrame = loadSurfacePoseFromFile();
  // KDL::Frame homeFrame = KDL::Frame( KDL::Rotation::Quaternion(qX, qY, qZ, qW),
                                        // KDL::Vector(pX, pY, pZ));

  currFrame.M.GetRPY(eR, eP, eY);
  eR = KDL::rad2deg * eR;   // rad to deg conversion
  eP = KDL::rad2deg * eP;
  eY = KDL::rad2deg * eY;

  while (true) {
    dx = dSteps[xIdx];
    dy = dSteps[yIdx];
    dz = dSteps[zIdx];
    dR = dSteps[RIdx];
    dP = dSteps[PIdx];
    dY = dSteps[YIdx];

    cout << "  Current : Linear Steps = (" << dx << ", " << dy << ", " << dz << ") mm | Angular Step =  (" << dR << ", " << dP << ", " << dY << ") deg" << endl;
    cout << "  Position     X: " << pX << " | Y: " << pY << " | Z: " << pZ << endl;
    cout << "  Orientation  R: " << eR << " | P: " << eP << " | Y: " << eY << endl;
    cout << endl;

    // get the next event from the keyboard  
    if (read(kfd, &key, 1) < 0) {
      perror("read():");
      exit(-1);
    }

    // swap_front(key);
    clearNTerminalLines(4);
    // printf("value: 0x%02X | %c\n\n\n\n", key, key);
  
    if (checkArrowKeys(key)) continue;
    switch(key) {
      // Increase/Decrease the Position/Orientation
      case KEYCODE_e: pX-=dx; break;
      case KEYCODE_q: pY-=dy; break;
      case KEYCODE_x: pZ-=dz; break;
      case KEYCODE_z: pX+=dx; break;
      case KEYCODE_c: pY+=dy; break;
      case KEYCODE_w: pZ+=dz; break;
      case KEYCODE_A: eR-=dR; break;
      case KEYCODE_S: eP-=dP; break;
      case KEYCODE_D: eY-=dY; break;
      case KEYCODE_a: eR+=dR; break;
      case KEYCODE_s: eP+=dP; break;
      case KEYCODE_d: eY+=dY; break;

      // Increase/Decrease the Movement Steps
      case KEYCODE_E: xIdx=max(0, xIdx-1); break;
      case KEYCODE_Q: yIdx=max(0, yIdx-1); break;
      case KEYCODE_X: zIdx=max(0, zIdx-1); break;
      case KEYCODE_Z: xIdx=min(4, xIdx+1); break;
      case KEYCODE_C: yIdx=min(4, yIdx+1); break;
      case KEYCODE_W: zIdx=min(4, zIdx+1); break;
      case KEYCODE_R: RIdx=max(0, RIdx-1); break;
      case KEYCODE_T: PIdx=max(0, PIdx-1); break;
      case KEYCODE_Y: YIdx=max(0, YIdx-1); break;
      case KEYCODE_r: RIdx=min(4, RIdx+1); break;
      case KEYCODE_t: PIdx=min(4, PIdx+1); break;
      case KEYCODE_y: YIdx=min(4, YIdx+1); break;

      // Goto/Change the Home Position
      case KEYCODE_h:
        // currFrame = homeFrame;
        pX = homeFrame.p.x() * 1000.0;  // metre to mm conversion
        pY = homeFrame.p.y() * 1000.0;
        pZ = homeFrame.p.z() * 1000.0;

        homeFrame.M.GetRPY(eR, eP, eY);
        eR = KDL::rad2deg * eR;   // rad to deg conversion
        eP = KDL::rad2deg * eP;
        eY = KDL::rad2deg * eY;
        break;
      case KEYCODE_H: 
        cout << "    Are you sure?" << endl;
        cout << "  Are you sure to set the current position as home position?" << endl;
        cout << "  Please confirm(y/n)? " << endl;
        cout << endl;
        cin >> key;
        if ((key == 'y') | (key == 'Y')) {
          homeFrame = KDL::Frame( KDL::Rotation::RPY(KDL::deg2rad*eR, KDL::deg2rad*eP, KDL::deg2rad*eY),
                                        KDL::Vector(pX/1000.0, pY/1000.0, pZ/1000.0));
          saveHomePoseToFile(homeFrame);
          clearNTerminalLines(4);
        }
        break;

      // Goto/Change the Work Surface Position
      case KEYCODE_p:
        pX = surfFrame.p.x() * 1000.0;  // metre to mm conversion
        pY = surfFrame.p.y() * 1000.0;
        pZ = surfFrame.p.z() * 1000.0;

        surfFrame.M.GetRPY(eR, eP, eY);
        eR = KDL::rad2deg * eR;   // rad to deg conversion
        eP = KDL::rad2deg * eP;
        eY = KDL::rad2deg * eY;
        break;
      case KEYCODE_P: 
        cout << "    Are you sure?" << endl;
        cout << "  Are you sure to set the current position as Work Surface?" << endl;
        cout << "  Please confirm(y/n)? " << endl;
        cout << endl;
        cin >> key;
        if ((key == 'y') | (key == 'Y')) {
          surfFrame = KDL::Frame( KDL::Rotation::RPY(KDL::deg2rad*eR, KDL::deg2rad*eP, KDL::deg2rad*eY),
                                        KDL::Vector(pX/1000.0, pY/1000.0, pZ/1000.0));
          saveSurfacePoseToFile(surfFrame);
          clearNTerminalLines(4);
        }
        break;

      default: continue;
    }

    pX = min(max(int(pX), minX), maxX);
    pY = min(max(int(pY), minY), maxY);
    pZ = min(max(int(pZ), minZ), maxZ);

    eR = ( ((int)eR/180)%2 ? ((int)eR%180)+(eR<0?180:-180) : ((int)eR%180) );
    eP = ( ((int)eP/180)%2 ? ((int)eP%180)+(eP<0?180:-180) : ((int)eP%180) );
    eY = ( ((int)eY/180)%2 ? ((int)eY%180)+(eY<0?180:-180) : ((int)eY%180) );

    currFrame = KDL::Frame( KDL::Rotation::RPY(KDL::deg2rad*eR, KDL::deg2rad*eP, KDL::deg2rad*eY),
                                        KDL::Vector(pX/1000.0, pY/1000.0, pZ/1000.0));

    publishPose(currFrame);
    publishWorkspace(surfFrame);
  }

  return;
}

int main(int argc, char** argv) {

  // Initialize the ROS node and nodehandle
  ros::init(argc, argv, "arm_teleop_keyboard");
  ros::NodeHandle nh;

  // cmd_pose Publisher
  armPosePublisher = nh.advertise<geometry_msgs::Pose>("/cobot_arm/cmd_pose", 10);
  // Surface Pose Publisher
  surfacePublisher = nh.advertise<visualization_msgs::Marker>("/cobot_arm/surface", 10);
  
  //////////////////////////////////////
  // Test - TF Listener Tool Tip Pose //
  //////////////////////////////////////
    tf::TransformListener listener;

    cout << "INFO : Start to read the current arm position!" << endl;
    tf::StampedTransform transform;
    try {
      listener.waitForTransform("/base_link", "/tool_tip", ros::Time(), ros::Duration(2.0));
      listener.lookupTransform("/base_link", "/tool_tip", ros::Time(0), transform);

      currPose.position.x = transform.getOrigin().x();
      currPose.position.y = transform.getOrigin().y();
      currPose.position.z = transform.getOrigin().z();
      currPose.orientation.x = transform.getRotation().x();
      currPose.orientation.y = transform.getRotation().y();
      currPose.orientation.z = transform.getRotation().z();
      currPose.orientation.w = transform.getRotation().w();
      cout << "DATA : Pos(X Y Z) : " << transform.getOrigin().x() 
                                      << " " << transform.getOrigin().y()
                                      << " " << transform.getOrigin().z() << endl;

      cout << "INFO : Got the current arm position from Rviz!" << endl;
    } catch (tf::TransformException &ex) {
      cerr << "ERROR : " << ex.what() << endl;
      // ros::Duration(0.5).sleep();

      currPose.position.x = 0.0;
      currPose.position.y = 0.0;
      currPose.position.z = 0.0;
      currPose.orientation.x = 0.0;
      currPose.orientation.y = 0.0;
      currPose.orientation.z = 0.0;
      currPose.orientation.w = 1.0;

      cout << "WARN : Failed to read and set the current arm pose as origin!" << endl;
    }

  // Check/Create cache directory to store latest home position
  createCacheDir(folderPath + "/cache");

  // Keyboard interrupt Handler
  signal(SIGINT, quit);

  boost::thread my_thread(&keyLoop);
  
  my_thread.interrupt() ;
  my_thread.join() ;
      
  return(0);
}


