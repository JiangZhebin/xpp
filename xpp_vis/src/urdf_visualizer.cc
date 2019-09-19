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

#include <xpp_vis/urdf_visualizer.h>

namespace xpp {

UrdfVisualizer::UrdfVisualizer(const std::string& urdf_name,
                               const std::vector<URDFName>& joint_names_in_urdf,
                               const URDFName& base_joint_in_urdf,
                               const std::string& fixed_frame,
                               const std::string& state_topic,
                               const std::string& tf_prefix,
                               const Vector3d& base_pos)
{
  joint_names_in_urdf_ = joint_names_in_urdf;
  base_joint_in_urdf_  = base_joint_in_urdf;
  rviz_fixed_frame_   = fixed_frame;
  tf_prefix_ = tf_prefix;
  base_pos_ = base_pos;

  ::ros::NodeHandle nh;
  state_sub_des_ = nh.subscribe(state_topic, 1, &UrdfVisualizer::StateCallback, this);
  ROS_DEBUG("Subscribed to: %s", state_sub_des_.getTopic().c_str());

  // Load model from file
  KDL::Tree my_kdl_tree;
  urdf::Model my_urdf_model;
  bool model_ok  = my_urdf_model.initParam(urdf_name);
  if(!model_ok)
  {
    ROS_ERROR("Invalid URDF File");
    exit(EXIT_FAILURE);
  }
  ROS_DEBUG("URDF successfully parsed");
  kdl_parser::treeFromUrdfModel(my_urdf_model, my_kdl_tree);
  ROS_DEBUG("Robot tree is ready");

  robot_publisher = std::make_shared<robot_state_publisher::RobotStatePublisher>(my_kdl_tree);
}

void
UrdfVisualizer::StateCallback(const xpp_msgs::RobotStateJoint& msg)
{
  auto joint_positions = AssignAngleToURDFJointName(msg.joint_state);
 // std::cout<<"msg state: "<<msg.base.pose.position<<std::endl;
  xpp_msgs::RobotStateJoint n_msg = msg;

  auto W_X_B_message   = GetBaseFromRos(::ros::Time::now(), msg.base.pose);

  //set base to be the origin of the world frame
  tf::StampedTransform transform;
  transform.setOrigin( tf::Vector3(base_pos_.x(), base_pos_.y(), base_pos_.z()) );
  tf::Quaternion q;
  q.setRPY(0, 0, 0.0);
  transform.setRotation(q);
  transform.stamp_    = ::ros::Time::now();//+::ros::Duration(-1.0);
  transform.frame_id_ = rviz_fixed_frame_;
  transform.child_frame_id_  = tf_prefix_ + "/" + base_joint_in_urdf_;
  tf_broadcaster_.sendTransform(transform);

  robot_publisher->publishTransforms(joint_positions, ::ros::Time::now(), tf_prefix_);
  robot_publisher->publishFixedTransforms(tf_prefix_);
}

UrdfVisualizer::UrdfnameToJointAngle
UrdfVisualizer::AssignAngleToURDFJointName(const sensor_msgs::JointState &msg) const
{
  UrdfnameToJointAngle q;

  for (int i=0; i<msg.position.size(); ++i)
    q[joint_names_in_urdf_.at(i)] = msg.position.at(i);

  return q;
}

geometry_msgs::TransformStamped
UrdfVisualizer::GetBaseFromRos(const ::ros::Time& stamp,
                               const geometry_msgs::Pose &msg) const
{
  // Converting from joint messages to robot state
  geometry_msgs::TransformStamped W_X_B_message;
  W_X_B_message.header.stamp    = stamp;
  W_X_B_message.header.frame_id = rviz_fixed_frame_;
  W_X_B_message.child_frame_id  = tf_prefix_ + "/" + base_joint_in_urdf_;
  W_X_B_message.transform.translation.x = 0;//msg.position.x;
  W_X_B_message.transform.translation.y = 0;//msg.position.y;
  W_X_B_message.transform.translation.z = 0;//msg.position.z;

//  W_X_B_message.transform.rotation.w =0;//msg.orientation.w;
//  W_X_B_message.transform.rotation.x =0;//msg.orientation.x;
//  W_X_B_message.transform.rotation.y =0;//msg.orientation.y;
//  W_X_B_message.transform.rotation.z =0;//msg.orientation.z;

  return W_X_B_message;
}

} // namespace xpp
