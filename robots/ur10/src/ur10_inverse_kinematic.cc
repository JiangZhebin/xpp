#include <cmath>
#include <map>

#include <ur10/ur10_inverse_kinematic.h>

#include <kdl/chain.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>

#include <xpp_states/cartesian_declarations.h>
#include <cstdlib>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <boost/date_time.hpp>
#include <trac_ik/trac_ik.hpp>
using namespace xpp;
using namespace KDL;

UR10InverseKinematics::VectorXd
UR10InverseKinematics::GetJointAngles(const Vector3d& ee_pos_B,const Eigen::Quaterniond& base_ori_B, const std::string& joint_desired_topic) const
{

const std::string joint_desired_topic_ = joint_desired_topic;

//load robot model
robot_model_loader::RobotModelLoader robot_model_loader("ur10_rviz_urdf_robot_description");
robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
ROS_DEBUG("Model frame: %s", kinematic_model->getModelFrame().c_str());
robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
kinematic_state->setToDefaultValues();
const robot_state::JointModelGroup* joint_model_group =  kinematic_model->getJointModelGroup("manipulator");
ROS_DEBUG("after getjointkinematics");
const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
std::vector<double> joint_values;

//create joint group
kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
for (std::size_t i = 0; i < joint_names.size(); ++i)
{
  ROS_DEBUG("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
}
kinematic_state->enforceBounds();
double time_out =0.01;

//set EE pose !!!!!!TODO!!!!!!!!
//end_effector_state.translation()=Eigen::Vector3d(0,0,0.5);
//end_effector_state.matrix()<<0,0,-1,ee_pos_B.x(),
//                            0,1,0,ee_pos_B.y(),
//                            1,0,0,ee_pos_B.z(),
//                            0,0,0,1;

//affin transformation matrxi from base frame to end effector's frame
Eigen::Isometry3d end_effector_state;
if(joint_desired_topic_ == "/xpp/joint_ur10_des_1"){

//  Eigen::Matrix3d initial_rotation;
//                  initial_rotation<<1,0,0,
//                                    0,-1,0,
//                                    0,0,1;
//  Eigen::Quaterniond initial_r_q(initial_rotation);
//  end_effector_state.rotate(initial_r_q);

//  end_effector_state.pretranslate(ee_pos_B);


  Eigen::Matrix4d base_ori_matrix = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d initial_matrix;
  initial_matrix<<1,0,0,ee_pos_B.x(),
                  0,1,0,ee_pos_B.y(),
                  0,0,1,ee_pos_B.z(),
                  0,0,0,1;

  // ensure that the rotation angle is between -90 and 90 degree.
  Vector3d base_ori_euler = base_ori_B.toRotationMatrix().eulerAngles(2,1,0);
  for(int i=0;i<3;i++){
    while(base_ori_euler[i]>1.5707 || base_ori_euler[i]<-1.5707){
      if(base_ori_euler[i]>1.5707)
        base_ori_euler[i]-=1.5707;
      if(base_ori_euler[i]<-1.5707)
        base_ori_euler[i]+=1.5707;
    }
  }
  //convert euler angles to rotation matrix
  Eigen::Matrix3d m ;
  m=Eigen::AngleAxisd(base_ori_euler.x(),Eigen::Vector3d::UnitZ())
    *Eigen::AngleAxisd(base_ori_euler.y(), Eigen::Vector3d::UnitY())
    *Eigen::AngleAxisd(base_ori_euler.z(), Eigen::Vector3d::UnitX());
  base_ori_matrix.block<3,3>(0,0) = m;

  //base_ori_matrix.block<3,3>(0,0)= base_ori_B.toRotationMatrix();


  //initial rotation + base rotation
  end_effector_state.matrix()= initial_matrix*base_ori_matrix;
  //form: cubic, end effector rotation range (-90, 90)



  //std::cout<<"Transformation matrix is : "<<end_effector_state.rotation()<<std::endl;


}
if(joint_desired_topic_ == "/xpp/joint_ur10_des_2"){

//  Eigen::Matrix3d initial_rotation;
//                  initial_rotation<<-1,0,0,
//                                    0,-1,0,
//                                    0,0,1;
//  Eigen::Quaterniond initial_r_q(initial_rotation);

// // end_effector_state.rotate(Eigen::Quaterniond())= initial_r_q;
//  end_effector_state.pretranslate(ee_pos_B);

  Eigen::Matrix4d base_ori_matrix = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d initial_matrix;
  initial_matrix<<-1,0,0,ee_pos_B.x(),
                  0,-1,0,ee_pos_B.y(),
                  0,0,1,ee_pos_B.z(),
                  0,0,0,1;

  Vector3d base_ori_euler = base_ori_B.toRotationMatrix().eulerAngles(2,1,0);
  for(int i=0;i<3;i++){
    while(base_ori_euler[i]>1.5707 || base_ori_euler[i]<-1.5707){
      if(base_ori_euler[i]>1.5707)
        base_ori_euler[i]-=1.5707;
      if(base_ori_euler[i]<-1.5707)
        base_ori_euler[i]+=1.5707;
    }
  }
  //convert euler angles to rotation matrix
  Eigen::Matrix3d m ;
  m=Eigen::AngleAxisd(base_ori_euler.x(),Eigen::Vector3d::UnitZ())
    *Eigen::AngleAxisd(base_ori_euler.y(), Eigen::Vector3d::UnitY())
    *Eigen::AngleAxisd(base_ori_euler.z(), Eigen::Vector3d::UnitX());
  base_ori_matrix.block<3,3>(0,0) = m;
  end_effector_state.matrix()= initial_matrix*base_ori_matrix;


//  end_effector_state.matrix()<<-1,0,0,ee_pos_B.x(),
//                                0,-1,0,ee_pos_B.y(),
//                                0,0,1,ee_pos_B.z(),
//                                0,0,0,1;
}
//std::cout<<end_effector_state.affine()<<std::endl;
//std::cout<<end_effector_state.matrix()<<std::endl;

//try 10 different initializations to solve the IK
//for(int i=0; i<10;i++){
  bool found_ik = kinematic_state->setFromIK(joint_model_group,end_effector_state,time_out);

  if (found_ik)
  {
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for(std::size_t i=0; i < joint_names.size(); ++i)
    {
      ROS_DEBUG("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
//    break;
  }
  else
  {
    ROS_INFO("Did not find IK solution");
  }
//}
//KDL::Chain chain;
//KDL::JntArray ll, ul;
//chain.addSegment(Segment("shoulder_pan_joint",Joint(Joint::RotZ),Frame(Rotation::RPY(0.0,0.0,0.0),Vector(0.0,0.0,0.1273))));
//chain.addSegment(Segment("shoulder_lift_joint",Joint(Joint::RotY),Frame(Rotation::RPY(0.0,1.570796325,0.0),Vector(0.0,0.220941,0.0))));
//chain.addSegment(Segment("elbow_joint",Joint(Joint::RotY),Frame(Rotation::RPY(0.0,0.0,0.0),Vector(0.0,-0.1719,0.612))));
//chain.addSegment(Segment("wrist_1_joint",Joint(Joint::RotY),Frame(Rotation::RPY(0.0,1.570796325,0.0),Vector(0.0,0.0,0.5723))));
//chain.addSegment(Segment("wrist_2_joint",Joint(Joint::RotZ),Frame(Rotation::RPY(0.0,0.0,0.0),Vector(0.0,0.1149,0.0))));
//chain.addSegment(Segment("wrist_3_joint",Joint(Joint::RotY),Frame(Rotation::RPY(0.0,0.0,0.0),Vector(0.0,0.0,0.1157))));
//chain.addSegment(Segment("ee_fixed_joint",Joint(Joint::None),Frame(Rotation::RPY(0.0,0.0,1.570796325),Vector(0.0,0.0922,0.0))));

////Creation of the solvers:
//ChainFkSolverPos_recursive fksolver1(chain);//Forward position solver
//ChainIkSolverVel_pinv iksolver1v(chain);//Inverse velocity solver
//ChainIkSolverPos_NR_JL iksolver1(chain,fksolver1,iksolver1v,100,1e-6);//Maximum 100 iterations, stop at accuracy 1e-6

////Creation of jntarrays:
//JntArray q(chain.getNrOfJoints());
//JntArray q_init(chain.getNrOfJoints());
////RRT
////for(int i=0;i<10;i++){
//  double j1= GetRandomNumber(-3.1415,3.1415);
//  double j2= GetRandomNumber(-3.1415,3.1415);
//  double j3= GetRandomNumber(-3.1415,3.1415);
//  double j4= GetRandomNumber(-3.1415,3.1415);
//  double j5= GetRandomNumber(-3.1415,3.1415);
//  double j6= GetRandomNumber(-3.1415,3.1415);

//  q_init.data<<0,j2,j3,0,j5,j6;
////  std::cout<<"q_init data: "<<q_init.data<<std::endl;
////  //Set destination frame
//  Frame F_dest;
//  F_dest.p = Vector(ee_pos_B.x(),ee_pos_B.y(),ee_pos_B.z());

//   F_dest.M = Rotation::RPY(0,-1.570796325,0);
//  int ret = iksolver1.CartToJnt(q_init,F_dest,q);
////  if(ret==0)
////    break;
////  else
////    std::cout<<"did not find the answer"<<std::endl;
//  std::cout<< "KDL result: "<< ret<<std::endl;

//}
//std::cout<< "KDL result number:"<<q.data<<std::endl;
Eigen::VectorXd b = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(joint_values.data(), joint_values.size());
return b;//q.data;

}

double
UR10InverseKinematics::GetRandomNumber(double fmin, double fmax) const{
  double f=(double)rand() / RAND_MAX;
  return fmin+f*(fmax-fmin);
}
