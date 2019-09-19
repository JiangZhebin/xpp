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

#include <xpp_vis/cartesian_joint_converter.h>

#include <ros/node_handle.h>

#include <xpp_msgs/RobotStateJoint.h>
#include <xpp_states/convert.h>

namespace xpp {

CartesianJointConverter::CartesianJointConverter (const InverseKinematics::Ptr& ik,
                                                  const std::string& cart_topic,
                                                  const std::string& joint_topic1,
                                                  const std::string& joint_topic2,
                                                  const std::string& joint_topic3,
                                                  const std::string& joint_topic4)
{
  inverse_kinematics_ = ik;
  flag=1;

  ::ros::NodeHandle n;
  cart_state_sub_ = n.subscribe(cart_topic, 1, &CartesianJointConverter::StateCallback, this);
  ROS_DEBUG("Subscribed to: %s", cart_state_sub_.getTopic().c_str());

  joint_state_pub_1  = n.advertise<xpp_msgs::RobotStateJoint>(joint_topic1, 1);
  ROS_DEBUG("Publishing to: %s", joint_state_pub_1.getTopic().c_str());
  if(joint_topic2!="a"){
    joint_state_pub_2  = n.advertise<xpp_msgs::RobotStateJoint>(joint_topic2, 1);
    ROS_DEBUG("Publishing to: %s", joint_state_pub_2.getTopic().c_str());
  }
  if(joint_topic3!="a"){
    joint_state_pub_3  = n.advertise<xpp_msgs::RobotStateJoint>(joint_topic3, 1);
    ROS_DEBUG("Publishing to: %s", joint_state_pub_3.getTopic().c_str());
  }
  if(joint_topic4!="a"){
    joint_state_pub_4  = n.advertise<xpp_msgs::RobotStateJoint>(joint_topic4, 1);
    ROS_DEBUG("Publishing to: %s", joint_state_pub_4.getTopic().c_str());
  }
  }

void
CartesianJointConverter::StateCallback (const xpp_msgs::RobotStateCartesian& cart_msg)
{
  auto cart = Convert::ToXpp(cart_msg);
  ee_initial.SetCount(cart.ee_motion_.GetEECount());
  ee_initial.SetAll(Vector3d(0,0,0.9));

  // transform feet from world -> base frame
//  Eigen::Matrix3d B_R_W = cart.base_.ang.q.normalized().toRotationMatrix().inverse();
  EndeffectorsPos ee_W(cart.ee_motion_.GetEECount());
  StateAng3d base_ang = cart.base_.ang;

  for (auto ee : ee_W.GetEEsOrdered()){
    ee_W.at(ee) = cart.ee_motion_.at(ee).p_;
//    std::cout<<"ee_pos passed to kdl"<<ee_W.at(ee)<<std::endl;
  }
    //ee_B.at(ee) = B_R_W * (cart.ee_motion_.at(ee).p_ - cart.base_.lin.p_);
//    std::cout<<"ee_pos previous"<<ee_previous<<std::endl;
  while(flag){
      q =  inverse_kinematics_->GetAllJointAngles(ee_W, base_ang.q).ToVec();
      flag=false;
    }
  if(ee_W!=ee_initial)
    q =  inverse_kinematics_->GetAllJointAngles(ee_W,base_ang.q).ToVec();



  if(q.size()<=6){
  //  std::cout<<"processing q"<<q<<std::endl;
    xpp_msgs::RobotStateJoint joint_msg;
//    joint_msg.base            = cart_msg.base;
//    joint_msg.ee_contact      = cart_msg.ee_contact;
//    joint_msg.time_from_start = cart_msg.time_from_start;
    joint_msg.joint_state.position = std::vector<double>(q.data(), q.data()+q.size());
    // Attention: Not filling joint velocities or torques

    joint_state_pub_1.publish(joint_msg);
  }
  if(q.size()>6 && q.size()<=12){

   // std::cout<<" q:"<<q<<std::endl;
    xpp_msgs::RobotStateJoint joint_msg1;
    joint_msg1.base            = cart_msg.base;
    joint_msg1.ee_contact      = cart_msg.ee_contact;
    joint_msg1.time_from_start = cart_msg.time_from_start;
    joint_msg1.joint_state.position = std::vector<double>(q.data(), q.data()+6);
    joint_state_pub_1.publish(joint_msg1);
  //  ROS_ERROR("PUblishing");
//    for(auto i:joint_msg1.joint_state.position)
//      std::cout<<"joint state 1: "<< i <<std::endl;
    xpp_msgs::RobotStateJoint joint_msg2;
    joint_msg2.base            = cart_msg.base;
    joint_msg2.ee_contact      = cart_msg.ee_contact;
    joint_msg2.time_from_start = cart_msg.time_from_start;
    joint_msg2.joint_state.position = std::vector<double>(q.data()+6, q.data()+q.size());
    joint_state_pub_2.publish(joint_msg2);
//    for(auto i:joint_msg2.joint_state.position)
//      std::cout<<"joint state 2: "<< i <<std::endl;
  }

  if(q.size()>12){
    xpp_msgs::RobotStateJoint joint_msg1;
    joint_msg1.base            = cart_msg.base;
    joint_msg1.ee_contact      = cart_msg.ee_contact;
    joint_msg1.time_from_start = cart_msg.time_from_start;
    joint_msg1.joint_state.position = std::vector<double>(q.data(), q.data()+6);
    joint_state_pub_1.publish(joint_msg1);

    xpp_msgs::RobotStateJoint joint_msg2;
    joint_msg2.base            = cart_msg.base;
    joint_msg2.ee_contact      = cart_msg.ee_contact;
    joint_msg2.time_from_start = cart_msg.time_from_start;
    joint_msg2.joint_state.position = std::vector<double>(q.data()+6, q.data()+12);
    joint_state_pub_2.publish(joint_msg2);

    xpp_msgs::RobotStateJoint joint_msg3;
    joint_msg3.base            = cart_msg.base;
    joint_msg3.ee_contact      = cart_msg.ee_contact;
    joint_msg3.time_from_start = cart_msg.time_from_start;
    joint_msg3.joint_state.position = std::vector<double>(q.data()+12, q.data()+18);
    joint_state_pub_3.publish(joint_msg3);

    xpp_msgs::RobotStateJoint joint_msg4;
    joint_msg4.base            = cart_msg.base;
    joint_msg4.ee_contact      = cart_msg.ee_contact;
    joint_msg4.time_from_start = cart_msg.time_from_start;
    joint_msg4.joint_state.position = std::vector<double>(q.data()+18, q.data()+q.size());
    joint_state_pub_4.publish(joint_msg4);
  }

}

} /* namespace xpp */
