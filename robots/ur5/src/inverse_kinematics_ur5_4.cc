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

#include <ur5/inverse_kinematics_ur5_4.h>

#include <cmath>
#include <iostream>
#include <xpp_states/cartesian_declarations.h>
#include <xpp_states/endeffector_mappings.h>

namespace xpp {

Joints
InverseKinematicsUR5_4::GetAllJointAngles(const EndeffectorsPos& x_B) const
{

  std::vector<Eigen::VectorXd> q_vec;

  // make sure always exactly 4 elements

  auto x_four_B = x_B.ToImpl();
  x_four_B.resize(4, x_four_B.front());
//std::cout<<x_biped_B.at(0)<<"ee pos "<< x_biped_B.at(1)<<std::endl;
  q_vec.push_back(leg.GetJointAngles(x_four_B.at(0)+Vector3d(0.5,0,0),"/xpp/joint_ur5_des_4_1" ));
  q_vec.push_back(leg.GetJointAngles(x_four_B.at(1)+Vector3d(-0.5,0,0),"/xpp/joint_ur5_des_4_2" ));
  q_vec.push_back(leg.GetJointAngles(x_four_B.at(2)+Vector3d(0,0.5,0),"/xpp/joint_ur5_des_4_3" ));
  q_vec.push_back(leg.GetJointAngles(x_four_B.at(3)+Vector3d(0,-0.5,0),"/xpp/joint_ur5_des_4_4" ));

  return Joints(q_vec);
}


} /* namespace xpp */


