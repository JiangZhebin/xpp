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

#ifndef XPP_VIS_INVERSEKINEMATICS_HYQ1_H_
#define XPP_VIS_INVERSEKINEMATICS_HYQ1_H_

#include <xpp_vis/inverse_kinematics.h>
#include <ur10/ur10_inverse_kinematic.h>

namespace xpp {

/**
 * @brief Inverse Kinematics for one HyQ leg attached to a brick (base).
 */
class InverseKinematicsUR10 : public InverseKinematics {
public:
  InverseKinematicsUR10() = default;
  virtual ~InverseKinematicsUR10() = default;

  /**
   * @brief Returns joint angles to reach for a specific foot position.
   * @param pos_B  3D-position of the foot expressed in the base frame (B).
   */
  Joints GetAllJointAngles(const EndeffectorsPos& pos_B,const Eigen::Quaterniond& base_ori) const override;

  /**
   * @brief Number of endeffectors (feet, hands) this implementation expects.
   */
  int GetEECount() const override { return 1; };

private:
  UR10InverseKinematics leg;
};

} /* namespace xpp  */

#endif /* XPP_VIS_INVERSEKINEMATICS_HYQ1_H_ */

