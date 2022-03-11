#include <iostream>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <urdf/model.h>

#include "tinyxml2.h"

using namespace KDL;
using namespace std;

int main() {
    const char* filename = "/home/ubuntussd/test_ws/src/Cobot_Arm_URDF/urdf/Cobot_Arm_URDF.urdf";
    KDL::Tree my_tree;
    if (!kdl_parser::treeFromFile(filename, my_tree)){
        cout <<"Failed to construct kdl tree" ;
        return false;
    }

    Chain chain;

    KDL::TreeFkSolverPos_recursive fk_solver(my_tree);
    my_tree.getChain("robot_footprint", "tool_tip", chain);


    ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);
    
    tinyxml2::XMLDocument xmlDocument;
    /*load the XML-FILE*/
    xmlDocument.LoadFile("/home/ubuntussd/test_ws/src/Cobot_Arm_URDF/urdf/Cobot_Arm_URDF.urdf");
    /*test if file has been loaded successfully*/
    if (xmlDocument.Error()){
        /*error occurred*/
        std::cout << "Error: " <<  xmlDocument.ErrorStr() << std::endl;
    } else {
        /*give a success message to user*/
        std::cout << "File \"sample.xml\" has been loaded successfully!!" << std::endl;
    }

//    KDL::Tree my_tree;
   urdf::Model my_model;
   if (!my_model.initXml(&xmlDocument)){
      cout << ("Failed to parse urdf robot model");
      return false;
   }
   if (!kdl_parser::treeFromUrdfModel(my_model, my_tree)){
      cout << ("Failed to construct kdl tree");
      return false;
   }


    // KDL::Tree my_tree;
    // ros::NodeHandle node;
    // std::string robot_desc_string;
    // node.param("robot_description", robot_desc_string, std::string());
    // if (!kdl_parser::treeFromString(robot_desc_string, my_tree)){
    //     cout <<"Failed to construct kdl tree");
    //     return false;
    // }

    // KDL::Tree my_tree;
    // TiXmlDocument xml_doc;
    // xml_doc.Parse(xml_string.c_str());
    // xml_root = xml_doc.FirstChildElement("robot");
    // if (!xml_root){
    //     cout <<"Failed to get robot from xml document");
    //     return false;
    // }
    // if (!kdl_parser::treeFromXml(xml_root, my_tree)){
    //     cout <<"Failed to construct kdl tree");
    //     return false;
    // }

    // KDL::Tree my_tree;
    // urdf::ModelInterface my_model;
    // // if (!my_model.initTree("/home/ubuntussd/test_ws/src/Cobot_Arm_URDF/urdf/Cobot_Arm_URDF.urdf")){
    // //     cout << "Failed to parse urdf robot model" ;
    // //     return false;
    // // }
    // // KDL::Tree my_tree;
    // if (!kdl_parser::treeFromFile("/home/ubuntussd/test_ws/src/Cobot_Arm_URDF/urdf/Cobot_Arm_URDF.urdf", my_tree)){
    //     cout << "Failed to construct kdl tree";
    //     return false;
    // }
    // if (!kdl_parser::treeFromUrdfModel(my_model, my_tree)){
    //     cout << "Failed to construct kdl tree" << endl;
    //     return false;
    // }




    char filename[] = "/home/ubuntussd/test_ws/src/Cobot_Arm_URDF/urdf/Cobot_Arm_URDF.urdf";

    // Load URDF file in XML Document 
    tinyxml2::XMLDocument xmlDocument;
    xmlDocument.LoadFile(filename);
    if (xmlDocument.Error()){
        cout << "Error: " <<  xmlDocument.ErrorStr() << std::endl;
    } else {
        cout << "URDF File has been loaded successfully!!" << std::endl;
    }

    KDL::Tree   tree;
    urdf::Model model;
    // Load URDF Model from Xml Document
    if (!model.initXml(&xmlDocument)){
        ROS_ERROR("Failed to parse urdf robot model");
        return false;
    }
    if (!kdl_parser::treeFromUrdfModel(model, tree)){
        ROS_ERROR("Failed to construct kdl tree");
        return false;
    }



}