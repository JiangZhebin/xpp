

#ifndef XPP_VIS_UR10_INVERSE_KINEMATIC_H_
#define XPP_VIS_UR10_INVERSE_KINEMATIC_H_

#include <Eigen/Dense>
#include <Eigen/Core>

namespace xpp {

enum UR10JointID {SPJ=0, SLJ, EJ, W1J,W2J,W3J,EEFJ,JointCount};
enum KneeBend{Forward,Backward};
/**
 * @brief Converts a hyq foot position to joint angles.
 */
class UR10InverseKinematics {
public:
  using VectorXd=Eigen::VectorXd;
  using Vector3d=Eigen::Vector3d;

  /**
   * @brief Default c'tor initializing leg lengths with standard values.
   */
  UR10InverseKinematics () = default;
  virtual ~UR10InverseKinematics () = default;

  /**
   * @brief Returns the joint angles to reach a Cartesian foot position.
   * @param ee_pos_H  Foot position xyz expressed in the frame attached
   * at the hip-aa (H).
   */
  VectorXd GetJointAngles(const Vector3d& ee_pos_H, const Eigen::Quaterniond& ee_ori_H,const std::string& joint_desired_topic) const;

  /**
   * @brief Restricts the joint angles to lie inside the feasible range
   * @param q[in/out]  Current joint angle that is adapted if it exceeds
   * the specified range.
   * @param joint  Which joint (HAA, HFE, KFE) this value represents.
   */
  void EnforceLimits(double& q, UR10JointID joint) const;

  double GetRandomNumber(double fmin, double fmax) const;

};

} /* namespace xpp */

#endif /* XPP_VIS_HYQLEG_INVERSE_KINEMATICS_H_ */
