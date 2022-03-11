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
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <math.h>

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
#include <kdl/path_composite.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/utilities/error.h>


using namespace std;  
using namespace KDL;  

// // Keyboard Button/Character Values
// #define KEYCODE_W 0x57
// #define KEYCODE_w 0x77
// #define KEYCODE_E 0x45
// #define KEYCODE_e 0x65
// #define KEYCODE_A 0x41
// #define KEYCODE_a 0x61
// #define KEYCODE_S 0x53
// #define KEYCODE_s 0x73
// #define KEYCODE_Z 0x5A
// #define KEYCODE_z 0x7A
// #define KEYCODE_X 0x58
// #define KEYCODE_x 0x78
// #define KEYCODE_H 0x48
// #define KEYCODE_h 0x68
// #define KEYCODE_T 0x54
// #define KEYCODE_t 0x74
// #define KEYCODE_Y 0x59
// #define KEYCODE_y 0x79

// Keyboard Button/Character Values
// Record
#define KEYCODE_R 0x52
#define KEYCODE_r 0x72
// Delete
#define KEYCODE_D 0x44
#define KEYCODE_d 0x64
// Clear
#define KEYCODE_C 0x43
#define KEYCODE_c 0x63
// Play
#define KEYCODE_P 0x50 
#define KEYCODE_p 0x70
// Save
#define KEYCODE_S 0x53
#define KEYCODE_s 0x73
// Quit
#define KEYCODE_Q 0x51 
#define KEYCODE_q 0x71
// GCode
#define KEYCODE_G 0x47
#define KEYCODE_g 0x67

// Workspace Limits
int minX =   90, maxX = 680;
int minY = -600, maxY = 600;
int minZ =    0, maxZ = 1160;


ros::Publisher waypointsVisPub;
ros::Publisher armPosePublisher;
ros::Publisher surfacePublisher;

geometry_msgs::Pose currPose;
geometry_msgs::PoseArray poseArray;
vector<geometry_msgs::Pose> waypoints;

string baseFrame  = "/base_link";
string endFrame   = "/tool_tip";

string moduleName = "cobot_arm_urdf";

string folderPath   = ros::package::getPath(moduleName);
string filePath     = folderPath + "/cache/waypoints/waypoints.txt";
string surfaceFile  = folderPath + "/cache/surface.txt";

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

void clearNTerminalLines(int NUM_LINES) {
  char terminal_clearline [4];
  char terminal_moveup [4];

  sprintf(terminal_clearline, "%c[2K", 0x1B);
  sprintf(terminal_moveup, "%c[1A", 0x1B);

  for (int i=0; i<NUM_LINES; i++)
    cout << terminal_moveup << terminal_clearline << endl;
}

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

void saveWaypointsToFile(vector<geometry_msgs::Pose> Waypoints){
  double pX, pY, pZ;
  double qX, qY, qZ, qW;
  ostringstream dataStream;
  for (auto pose: Waypoints) {
    pX = pose.position.x;
    pY = pose.position.y;
    pZ = pose.position.z;
    qX = pose.orientation.x;
    qY = pose.orientation.y;
    qZ = pose.orientation.z;
    qW = pose.orientation.w;

    dataStream << pX << " " << pY << " " << pZ << " " << qX << " " << qY << " " << qZ << " " << qW << "\n";
  }

  string data = dataStream.str();

  ofstream outFile(filePath);
  outFile << data;
  outFile.close();
}

vector<geometry_msgs::Pose> loadWaypointsFromFile() {
  vector<geometry_msgs::Pose> Waypoints;
  if (isPathExists(filePath)) 
    cout << "INFO : Saved Waypoints file exists!" << endl;
  else {
    cout << "INFO : Saved Waypoints file does not exists!" << endl;
    // Return empty poses vector
    return Waypoints;
  }

  ifstream inFile;
  string line;
  inFile.open(filePath);

  double pX, pY, pZ;
  double qX, qY, qZ, qW;
  while (std::getline(inFile, line)) {
    istringstream dataStream(line);

    dataStream >> pX;
    dataStream >> pY;
    dataStream >> pZ;
    dataStream >> qX;
    dataStream >> qY;
    dataStream >> qZ;
    dataStream >> qW;

    currPose.position.x = pX;
    currPose.position.y = pY;
    currPose.position.z = pZ;
    currPose.orientation.x = qX;
    currPose.orientation.y = qY;
    currPose.orientation.z = qZ;
    currPose.orientation.w = qW;

    Waypoints.push_back(currPose);
  }
  return Waypoints;
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

geometry_msgs::Pose pubPose;
void publishPose(KDL::Frame frame) {
  pubPose.position.x = frame.p.x();
  pubPose.position.y = frame.p.y();
  pubPose.position.z = frame.p.z();

  double qX, qY, qZ, qW;
  frame.M.GetQuaternion(qX, qY, qZ, qW);
  pubPose.orientation.x = qX;
  pubPose.orientation.y = qY;
  pubPose.orientation.z = qZ;
  pubPose.orientation.w = qW;

  armPosePublisher.publish(pubPose);
}

void getEulerFromQuat(double x, double y, double z, double w, double &rollX, double &pitchY, double &yawZ) {
  double t0, t1, t2, t3, t4;
  t0 = +2.0 * ((w * x) + (y * z));
  t1 = +1.0 - 2.0 * ((x * x) + (y * y));

  t2 = +2.0 * ((w * y) - (z * x));
  t2 = ((t2 > +1.0) ? +1.0 : t2);
  t2 = ((t2 < -1.0) ? -1.0 : t2);

  t3 = +2.0 * ((w * z) + (x * y));
  t4 = +1.0 - (2.0 * ((y * y) + (z * z)));
  
  rollX   = atan2(t0, t1);
  pitchY  = asin(t2);
  yawZ    = atan2(t3, t4);
}


auto getKeyPress = [](){
  char key;
  // get the next event from the keyboard  
  if (read(kfd, &key, 1) < 0) {
    perror("read():");
    exit(-1);
  }
  return key;
};

auto Distance = [](geometry_msgs::Pose p1, geometry_msgs::Pose p2) {
  return sqrtf(pow((p2.position.x - p1.position.x), 2) + pow((p2.position.y - p1.position.y), 2) + pow((p2.position.z - p1.position.z), 2));
};

double STEP_TIME = 0.1;
double FEED_RATE = 0.01; // m/s
bool goToPose(geometry_msgs::Pose nextPose, int NUM_STEPS=20) {
  getCurrentPose();

  // NUM_STEPS = Distance(currPose, nextPose) / STEP_TIME;

  double Rr, Pp, Yy;
  getEulerFromQuat( currPose.orientation.x, currPose.orientation.y, 
                      currPose.orientation.z, currPose.orientation.w,
                          Rr, Pp, Yy);

  double r1, p1, y1;
  getEulerFromQuat( nextPose.orientation.x, nextPose.orientation.y,
                      nextPose.orientation.z, nextPose.orientation.w,
                          r1, p1, y1);

  double xX, yY, zZ;
  xX = currPose.position.x;
  yY = currPose.position.y;
  zZ = currPose.position.z;

  double dX, dY, dZ;
  dX = nextPose.position.x - xX;
  dY = nextPose.position.y - yY;
  dZ = nextPose.position.z - zZ;

  double dRoll, dPitch, dYaw;
  dRoll   = r1 - Rr;
  dPitch  = p1 - Pp;
  dYaw    = y1 - Yy;
  dRoll   = ( abs(dRoll)>=180.0   ? -(360.0 - dRoll)  : dRoll ); 
  dPitch  = ( abs(dPitch)>=180.0  ? -(360.0 - dPitch) : dPitch );
  dYaw    = ( abs(dYaw)>=180.0    ? -(360.0 - dYaw)   : dYaw );

  for (int step=1; step<=NUM_STEPS; step++) {
    if ((getKeyPress() == KEYCODE_Q) | (getKeyPress() == KEYCODE_q))
      return false;
    
    cout << "----------------------" << endl;
    cout << "  Playing Trajectory" << endl;
    cout << "----------------------" << endl;
    cout << "Orientation (R P Y) :" <<  Rr + (dRoll * step / NUM_STEPS) << " " << Pp + (dPitch * step / NUM_STEPS) << " " << Yy + (dYaw * step / NUM_STEPS) << endl;
    cout << "Position    (X Y Z) : " << xX + (dX * step / NUM_STEPS) << " " << yY + (dY * step / NUM_STEPS) << " " << zZ + (dZ * step / NUM_STEPS) << endl;
    cout << "  Press Q to Cancel/Stop" << endl;
    cout << endl;
    publishPose(  KDL::Frame( Rotation::RPY(  Rr + (dRoll * step / NUM_STEPS),
                                                Pp + (dPitch * step / NUM_STEPS),
                                                  Yy + (dYaw * step / NUM_STEPS)),
                                Vector( xX + (dX * step / NUM_STEPS),
                                          yY + (dY * step / NUM_STEPS),
                                            zZ + (dZ * step / NUM_STEPS)))
                );
    ros::Duration(STEP_TIME).sleep();
    clearNTerminalLines(7);
  }
  return true;
}

void playTrajectory(vector<geometry_msgs::Pose> Waypoints) {
  for (auto WP:Waypoints) {
    if (!goToPose(WP))
      break;
  }
}

void playTrajectory1(vector<geometry_msgs::Pose> Waypoints) {
  int NSTEPS = 20;
  for (int i=0; i < Waypoints.size() - 1; i++) {
    double Rr, Pp, Yy;
    getEulerFromQuat( Waypoints.at(i).orientation.x, Waypoints.at(i).orientation.y, 
                        Waypoints.at(i).orientation.z, Waypoints.at(i).orientation.w,
                            Rr, Pp, Yy);

    double r1, p1, y1;
    getEulerFromQuat( Waypoints.at(i+1).orientation.x, Waypoints.at(i+1).orientation.y,
                        Waypoints.at(i+1).orientation.z, Waypoints.at(i+1).orientation.w,
                            r1, p1, y1);

    double xX, yY, zZ;
    xX = Waypoints.at(i).position.x;
    yY = Waypoints.at(i).position.y;
    zZ = Waypoints.at(i).position.z;

    double dX, dY, dZ;
    dX = Waypoints.at(i+1).position.x - xX;
    dY = Waypoints.at(i+1).position.y - yY;
    dZ = Waypoints.at(i+1).position.z - zZ;
  
    double dRoll, dPitch, dYaw;
    dRoll   = r1 - Rr;
    dPitch  = p1 - Pp;
    dYaw    = y1 - Yy;
    dRoll   = ( abs(dRoll)>=180.0   ? -(360.0 - dRoll)  : dRoll ); 
    dPitch  = ( abs(dPitch)>=180.0  ? -(360.0 - dPitch) : dPitch );
    dYaw    = ( abs(dYaw)>=180.0    ? -(360.0 - dYaw)   : dYaw );

    for (int step=1; step<=NSTEPS; step++) {
      if ((getKeyPress() == KEYCODE_Q) | (getKeyPress() == KEYCODE_q))
        break;

      cout << "----------------------" << endl;
      cout << "  Playing Trajectory" << endl;
      cout << "----------------------" << endl;  
      cout << "Orientation(R P Y) " <<  Rr + (dRoll * step / NSTEPS) << " " << Pp + (dPitch * step / NSTEPS) << " " << Yy + (dYaw * step / NSTEPS) << endl;
      cout << "Position(X Y Z) : " << xX + (dX * step / NSTEPS) << " " << yY + (dY * step / NSTEPS) << " " << zZ + (dZ * step / NSTEPS) << endl;
      cout << endl;
      publishPose(  KDL::Frame( Rotation::RPY(  Rr + (dRoll * step / NSTEPS), 
                                                  Pp + (dPitch * step / NSTEPS), 
                                                    Yy + (dYaw * step / NSTEPS)), 
                                  Vector( xX + (dX * step / NSTEPS), 
                                            yY + (dY * step / NSTEPS),
                                              zZ + (dZ * step / NSTEPS)))
                  );
      ros::Duration(0.2).sleep();
      clearNTerminalLines(6);
    }
  }

  // // try {
	// 	KDL::Path_RoundedComposite* path = new KDL::Path_RoundedComposite(0.1, 0.01, new KDL::RotationalInterpolation_SingleAxis());
  //   double pX, pY, pZ;
  //   double qX, qY, qZ, qW;
  //   for (auto WP:Waypoints) {
  //     pX = WP.position.x;
  //     pY = WP.position.y;
  //     pZ = WP.position.z;
  //     qX = WP.orientation.x;
  //     qY = WP.orientation.y;
  //     qZ = WP.orientation.z;
  //     qW = WP.orientation.w;

  //     // path->Add(Frame(Rotation::Quaternion(qX, qY, qZ, qW), Vector(pX, pY, pZ)));
  //   }
  //   cout << "\ncompare "<< (Vector(0, 0, 0)==Vector(0, 0.01, 0)) << endl;
	// 	path->Add(Frame(Rotation::RPY(22.0/7.0,0,0), Vector(0, 0, 0)));
	// 	path->Add(Frame(Rotation::RPY(22.0/7.0,0,0), Vector(0, 0, 0.1)));
	// 	path->Add(Frame(Rotation::RPY(22.0/7.0,44.0/7.0,0), Vector(0, 0, 0.1)));
	// 	path->Add(Frame(Rotation::RPY(0,22.0/7.0,0), Vector(-1, 1, 1)));
	// 	path->Add(Frame(Rotation::RPY(22.0/7.0,0,0), Vector(-1, 1, -1)));
	// 	path->Add(Frame(Rotation::RPY(0,22.0/7.0,0), Vector(-1, -1, -1)));
	// 	path->Add(Frame(Rotation::RPY(22.0/7.0,0,0), Vector(-1, -1, 1)));

  //   // path->Add(Frame(Rotation::Quaternion(0.707107, 0, 0, 0.707107), Vector(0.66, 0.0, 0.543)));
  //   // // path->Add(Frame(Rotation::Quaternion(-0.5, -0.5, 0.5, -0.5), Vector(0.66, 0.0, 0.543)));
  //   // path->Add(Frame(Rotation::Quaternion(0, 0, 0, 1), Vector(0.66, -0.15, 0.543)));
  //   // // path->Add(Frame(Rotation::Quaternion(3.36886e-10, 0.707107, -3.62042e-09, 0.707107), Vector(0.66, -0.15, 0.543)));
  //   // path->Add(Frame(Rotation::Quaternion(0.707107, 0, 0, 0.707107), Vector(0.66, 0.0, 0.543)));

	// 	path->Finish();
        		
  //   // KDL::VelocityProfile* velpref = new KDL::VelocityProfile_Trap(0.5,0.1);
	// 	// velpref->SetProfile(0, path->PathLength());
	// 	// KDL::Trajectory* traject = new KDL::Trajectory_Segment(path, velpref);

	// 	// KDL::Frame nextPose;
  //   // double dt = 0.1;
	// 	// for (double t=0.0; t <= traject->Duration(); t+= dt) {
  //   //   if ((getKeyPress() == KEYCODE_Q) | (getKeyPress() == KEYCODE_q))
  //   //     break;
	// 	// 	publishPose(traject->Pos(t));
	// 	// }
        
  //   // delete traject;
	// // } catch(KDL::Error& error) {
	// // 	cout << "Error : Trajectory - " << error.Description() << endl;
	// // 	cout << "\twith the following type " << error.GetType() << endl;
	// // }
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

void publishWorkspace(KDL::Frame frame) {
  visualization_msgs::Marker surface;
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
  surface.color.a = 1.0;

  surface.lifetime = ros::Duration();

  surfacePublisher.publish(surface);
}

void playGCode(string FILENAME, KDL::Frame surface) {
  ifstream gcode(FILENAME);
  if (gcode.is_open()) {
      double Rr, Pp, Yy;
      double xX, yY, zZ;
      string line, word;
      while (std::getline(gcode, line)) {
        cout << line << endl;
        if ((getKeyPress() == KEYCODE_Q) | (getKeyPress() == KEYCODE_q))
          return ;
        
        cout << "----------------------" << endl;
        cout << "  Playing GCode" << endl;
        cout << "----------------------" << endl;
        cout << "Position(X Y Z)    : " << xX << " " << yY << " " << zZ << endl;
        cout << "  Press Q to Cancel/Stop" << endl;
        cout << endl;

        if (strcmp(line.substr(0, 2).c_str(), "G1")==0) {
          stringstream ss(line.substr(3));

          ss >> word;
          xX = stod(word.substr(1)) * 3.0;
          ss >> word;
          yY = stod(word.substr(1)) * 3.0;
          ss >> word;
          // zZ = stod(word.substr(1));
          // cout << "F : " << stod(word.substr(1)) << endl;
        } else if (strcmp(line.substr(0, 4).c_str(), "M300")==0) {
          stringstream ss(line.substr(4));
          ss >> word;
          zZ = (stod(word.substr(1))>40 ? -50.0 : -10.0 );
        } else {
          clearNTerminalLines(6);
          continue;
        }

        KDL::Frame fr(KDL::Rotation::RPY(0, 0, 0),
                          KDL::Vector(xX/1000.0, yY/1000.0, zZ/1000.0));
        fr = surface * fr;

          geometry_msgs::Pose gPose;
          gPose.position.x = fr.p.x();
          gPose.position.y = fr.p.y();
          gPose.position.z = fr.p.z();

          double qX, qY, qZ, qW;
          fr.M.GetQuaternion(qX, qY, qZ, qW);
          gPose.orientation.x = qX;
          gPose.orientation.y = qY;
          gPose.orientation.z = qZ;
          gPose.orientation.w = qW;

        clearNTerminalLines(6);
        if (!goToPose(gPose, 5))
          break;
        // publishPose(fr);
      }
      gcode.close();
  }
}

void keyLoop() {

    cout << "-------------------------------------" << endl;
    cout << "  Control the 6DOF Arm/Manipulator!" << endl;
    cout << "-------------------------------------" << endl;
    cout << "  Key Controls :" << endl;
    cout << "  ------------" << endl;
    cout << "    R/r : Record the current pose" << endl;
    cout << "    D/d : Delete the Last recorded pose" << endl;
    cout << "    C/c : Clear all the recorded pose" << endl;
    cout << "    P/p : Play the recorded pose" << endl;
    cout << endl;
    cout << "  Ctrl + C to Quit" << endl;
    cout << endl;
    cout << "------------------------------------------------------" << endl;
    cout << endl;

  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Timeout
  cooked.c_cc[VMIN] = 0;
  cooked.c_cc[VTIME] = 1;
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  // Setup the TF Listener
  listener = new(tf::TransformListener);

  // Load Saved Waypoints
  waypoints = loadWaypointsFromFile();
  KDL::Frame surfFrame = loadSurfacePoseFromFile();
  
  char information[30];   ////////////////////////////////////////////////////////////////////

  char key;
  double pX, pY, pZ;            // Position
  double eR, eP, eY;            // Euler Angles
  double qX, qY, qZ, qW;        // Quaternion Angles
  while (true) {
    cout << "INFO : " << information << endl;
    cout << "  DATA : Waypoints" << endl;
    cout << "  " << setw(3) << "IDX" << setw(36) << "Position{X Y Z}(mm)" << setw(5) << "  -  " << setw(46) << "Orientation{x y z w}(deg)" << endl;
    for (auto& wp: waypoints) {
      cout  << "  " << setw(3) << &wp-&waypoints[0] // Calculate Index in Vector 
              << "  {"  << setw(10) << wp.position.x*1000 << " " 
                          << setw(10) << wp.position.y*1000 << " " 
                            << setw(10) << wp.position.z*1000 << "}"
              << setw(5) << "  -  "
              << "{"  << setw(10) << wp.orientation.x << " " 
                        << setw(10) << wp.orientation.y << " " 
                          << setw(10)  << wp.orientation.z << " " 
                            << setw(10) << wp.orientation.w << "}" << endl;
      // cout << "path->Add(Frame(Rotation::Quaternion(" << wp.orientation.x << ", " << wp.orientation.y << ", "  << wp.orientation.z << ", "  << wp.orientation.w << "), Vector(" << wp.position.x << ", "  << wp.position.y << ", "  << wp.position.z << ")));" << endl;
    }
    cout << endl;

    // Get the Current Position and Orientation
    getCurrentPose();
    pX = currPose.position.x * 1000.0;
    pY = currPose.position.y * 1000.0;
    pZ = currPose.position.z * 1000.0;  // metre to mm conversion
    qX = currPose.orientation.x;
    qY = currPose.orientation.y;
    qZ = currPose.orientation.z;
    qW = currPose.orientation.w;

    // Display the Current Position and Orientation
    cout << "  Position     (mm) X: " << pX << " | Y: " << pY << " | Z: " << pZ << endl;
    cout << "  Orientation (deg) x: " << qX << " | y: " << qY << " | z: " << qZ << " | w: " << qW << endl;
    cout << endl;

    // Get the next event from the keyboard  
    if (read(kfd, &key, 1) < 0) {
      perror("read():");
      exit(-1);
    }

    clearNTerminalLines(4 + (1*waypoints.size()) + 3);
    // printf("value: 0x%02X | %c\n\n\n\n", key, key);
  
    if (checkArrowKeys(key)) continue;
    switch(key) {
      // Record
      case KEYCODE_R:
      case KEYCODE_r: strcpy(information, "New waypoint recorded!");
                      waypoints.push_back(currPose); break;

      // Delete
      case KEYCODE_D: 
      case KEYCODE_d: strcpy(information, "Last waypoint deleted!");
                      if (waypoints.size()>0) waypoints.pop_back(); break;

      // Clear
      case KEYCODE_C: 
      case KEYCODE_c: strcpy(information, "All waypoints cleared!");
                      waypoints.clear() ; break;

      // Play
      case KEYCODE_P:
      case KEYCODE_p: if (waypoints.size()>0) playTrajectory(waypoints); break;

      // Save
      case KEYCODE_S: 
      case KEYCODE_s: strcpy(information, "Waypoints saved successfully!");
                      saveWaypointsToFile(waypoints); break;
      
      // GCode
      case KEYCODE_G: 
      case KEYCODE_g: playGCode("/media/ubuntussd/OS/Users/NKKS-HOME/Desktop/example.gcode", loadSurfacePoseFromFile());
      // Skip
      default:  continue;
    }
    key = 0;  // Reset the keypress

    // cout << "INFO : " << information << endl;

    poseArray.header.stamp = ros::Time::now();
    poseArray.header.seq+=1;
    poseArray.header.frame_id = "robot_footprint";
    poseArray.poses = waypoints;
    waypointsVisPub.publish(poseArray);
    
    publishWorkspace(surfFrame);
  }

  return;
}

int main(int argc, char** argv) {

  // Initialize the ROS node and nodehandle
  ros::init(argc, argv, "arm_record_and_play");
  ros::NodeHandle nh;

  // Publisher - Visualize the Waypoints 
  waypointsVisPub = nh.advertise<geometry_msgs::PoseArray>("/cobot_arm/waypoints", 10);  
  // Publisher - cmd_pose (plugin)
  armPosePublisher = nh.advertise<geometry_msgs::Pose>("/cobot_arm/cmd_pose", 10);
  // Surface Pose Publisher
  surfacePublisher = nh.advertise<visualization_msgs::Marker>("/cobot_arm/surface", 10);

  
  // Check/Create cache directory to store latest home position
  createCacheDir(folderPath + "/cache/waypoints");

  // Keyboard interrupt Handler
  signal(SIGINT, quit);

  currPose.position.x = 0.0;
  currPose.position.y = 0.0;
  currPose.position.z = 0.0;
  currPose.orientation.x = 0.0;
  currPose.orientation.y = 0.0;
  currPose.orientation.z = 0.0;
  currPose.orientation.w = 1.0;

  boost::thread my_thread(&keyLoop);
  
  my_thread.interrupt() ;
  my_thread.join() ;
      
  return(0);
}


