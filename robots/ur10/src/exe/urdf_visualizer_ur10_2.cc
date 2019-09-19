/******************************************************************************
Copyright (c) 2017, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <ros/ros.h>

#include <xpp_msgs/topic_names.h>

#include <xpp_vis/urdf_visualizer.h>
#include <xpp_vis/cartesian_joint_converter.h>

#include <kdl/chain.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>

#include <ur10/ur10_inverse_kinematic.h>
#include <ur10/inverse_kinematics_ur10_2.h>
using namespace xpp;
using namespace KDL;
int main(int argc, char *argv[])
{
  ::ros::init(argc, argv, "ur10_urdf_visualizer");

  const std::string joint_desired_1 = "xpp/joint_ur10_des_1";
  const std::string joint_desired_2 = "xpp/joint_ur10_des_2";
  const std::string joint_desired_3 = "a";
  const std::string joint_desired_4 = "a";


//  auto ik = std::make_shared<InverseKinematicsUR10_2>();
//  CartesianJointConverter inv_kin_converter(ik,
//					    xpp_msgs::robot_state_desired,
//              joint_desired_1,joint_desired_2,joint_desired_3,joint_desired_4);

  std::vector<UrdfVisualizer::URDFName> joint_names(JointCount);
  joint_names.at(SPJ) = "shoulder_pan_joint";
  joint_names.at(SLJ) = "shoulder_lift_joint";
  joint_names.at(EJ) = "elbow_joint";
  joint_names.at(W1J) = "wrist_1_joint";
  joint_names.at(W2J)= "wrist_2_joint";
  joint_names.at(W3J)="wrist_3_joint";
  joint_names.at(EEFJ)="ee_fixed_joint";
  std::string urdf = "ur10_rviz_urdf_robot_description";
  UrdfVisualizer node_des(urdf, joint_names, "base_link", "world",
        joint_desired_1, "ur10_1",Vector3d(-0.4,0,0));


  UrdfVisualizer node_des_2(urdf, joint_names, "base_link", "world",
        joint_desired_2, "ur10_2",Vector3d(0.4,0,0));

  ::ros::spin();

  return 1;
}

