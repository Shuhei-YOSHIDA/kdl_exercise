/**
 * @file sample1.cpp
 * @breaf Sample of KDL::Tree, KDL::Chain, FK, IK
 */

#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
//#include <kdl/chainiksolver.hpp>
//#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_lma.hpp> // LM-method
#include <kdl/chainfksolverpos_recursive.hpp>

#include <chrono>


void usageexit()
{
  std::cerr << "rosrun kdl_exercise sample1 <chain_root> <chain_tip> <ik_trg_diff_x> <y> <z>" << std::endl
    << "ex: ./sample1 base_link wrist_3_link -0.5 0 +0.5" << std::endl;
  exit(-1);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sample1");
  ros::NodeHandle nh;

  if (argc < 6) usageexit();
  //std::string chain_root = "base_link";
  //std::string chain_tip = "wrist_3_link";
  std::string chain_root = argv[1];
  std::string chain_tip = argv[2];
  double diff_x = atof(argv[3]);
  double diff_y = atof(argv[4]);
  double diff_z = atof(argv[5]);
  std::cout << "input are ..." << std::endl
            << "chain_root: " << chain_root << std::endl
            << "chain_tip: " << chain_tip << std::endl
            << "ik_target_diff_x: " << diff_x << std::endl
            << "ik_target_diff_y: " << diff_y << std::endl
            << "ik_target_diff_z: " << diff_z << std::endl;


  // Get URDF from parameter
  std::string robot_description_text;
  if (!nh.getParam("/robot_description", robot_description_text))
  {
    ROS_FATAL("Could not get param /robot_description");
    return -1;
  }

  // Parse URDF by using kdl_parser
  KDL::Tree robot_tree;
  kdl_parser::treeFromString(robot_description_text, robot_tree);

  // Show link names
  std::map<std::string, KDL::TreeElement> robot_segment = robot_tree.getSegments();
  for (auto itr = robot_segment.begin(); itr != robot_segment.end(); ++itr)
  {
    std::cout << "---------------" << std::endl;
    std::cout << itr->first << std::endl;
    std::cout << itr->second.segment.getJoint().getName() << std::endl;
  }
  std::cout << "=====================" << std::endl << "root_name of tree" << std::endl;
  std::cout << robot_tree.getRootSegment()->first << std::endl;
  std::cout << std::endl;

  // Get serial kinematic chain from tree
  KDL::Chain chain;
  if (!robot_tree.getChain(chain_root, chain_tip, chain))
  {
    std::cerr << "" << std::endl;
    return -1;
  }

  std::cout << "chain from " << chain_root << " to " << chain_tip << std::endl;
  for (int i = 0; i < chain.getNrOfSegments(); i++)
  {
    auto seg_n = chain.getSegment(i).getName();
    auto jnt_n = chain.getSegment(i).getJoint().getName();
    auto jntype_n = chain.getSegment(i).getJoint().getTypeName();
    std::cout << "segment (" << i << "):" << seg_n << " : joint " << jnt_n << ": jnt_type " << jntype_n  << std::endl;
  }
  std::cout << std::endl;

  // Forward kinematics
  std::cout << "FK for joints which are all zero" << std::endl;
  KDL::ChainFkSolverPos_recursive fksolver(chain);
  KDL::JntArray jointpositions = KDL::JntArray(chain.getNrOfJoints());
  for (int i = 0; i < chain.getNrOfJoints(); i++)
  {
    jointpositions(i) = 0.0;
  }

  KDL::Frame cartpos;
  auto fk_start = std::chrono::system_clock::now();
  int fk_res = fksolver.JntToCart(jointpositions, cartpos);
  auto fk_end = std::chrono::system_clock::now();
  if (fk_res < 0)
  {
    std::cerr << "failed fk" << std::endl;
    return -1;
  }
  auto fk_time = std::chrono::duration_cast<std::chrono::nanoseconds>(fk_end - fk_start).count();
  std::cout << "fk elappsed time: " << fk_time << "[nano_sec]" << std::endl;
  double qx, qy, qz, qw;
  cartpos.M.GetQuaternion(qx, qy, qz, qw);
  std::cout << "x: " << cartpos.p.x() << std::endl
            << "y: " << cartpos.p.y() << std::endl
            << "z: " << cartpos.p.z() << std::endl
            << "qx: " << qx << std::endl
            << "qy: " << qy << std::endl
            << "qz: " << qz << std::endl
            << "qw: " << qw << std::endl;
  std::cout << std::endl;

  // Inverse kinemematics
  KDL::ChainIkSolverPos_LMA iksolver(chain);
  double new_x = cartpos.p.x() + diff_x;
  double new_y = cartpos.p.z() + diff_y;
  double new_z = cartpos.p.z() + diff_z;
  cartpos.p.x(new_x);
  cartpos.p.y(new_y);
  cartpos.p.z(new_z);

  KDL::JntArray q(chain.getNrOfJoints());
  auto ik_start = std::chrono::system_clock::now();
  int ik_res = iksolver.CartToJnt(jointpositions, cartpos, q);
  auto ik_end = std::chrono::system_clock::now();
  std:: cout << "result: " << iksolver.strError(ik_res) << std::endl;
  if (ik_res != KDL::ChainIkSolverPos_LMA::E_NOERROR)
  {
    std::cerr << "failed ik " << std::endl;
    return -1;
  }
  auto ik_time = std::chrono::duration_cast<std::chrono::nanoseconds>(ik_end - ik_start).count();
  std::cout << "ik elappsed time: " << ik_time << "[nano_sec]" << std::endl;

  for (int i = 0; i < chain.getNrOfJoints(); i++)
  {
    std::cout << "joint " << i << " :" << q(i) << std::endl;
  }

  return 0;
}
