#include "moveit/ompl_interface/detail/ompl_constraint.h"

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/Constraints.h>

namespace ompl_interface
{
/****************************
 * Base class for constraints
 * **************************/

BaseConstraint::BaseConstraint(robot_model::RobotModelConstPtr robot_model, const std::string& group,
                               const unsigned int num_dofs, const unsigned int num_cons_)
  : robot_model_(robot_model), ob::Constraint(num_dofs, num_cons_)
{
  // Setup Moveit's robot model for kinematic calculations
  robot_state_.reset(new robot_state::RobotState(robot_model_));
  robot_state_->setToDefaultValues();
  joint_model_group_ = robot_state_->getJointModelGroup(group);

  // use end-effector link by default
  // \todo use the link specified in the constraint message
  link_name_ = joint_model_group_->getLinkModelNames().back();
}

void BaseConstraint::init(moveit_msgs::Constraints constraints)
{
  parseConstraintMsg(constraints);
}

Eigen::Isometry3d BaseConstraint::forwardKinematics(const Eigen::Ref<const Eigen::VectorXd>& joint_values) const
{
  robot_state_->setJointGroupPositions(joint_model_group_, joint_values);
  return robot_state_->getGlobalLinkTransform(link_name_);
}

Eigen::MatrixXd BaseConstraint::geometricJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_values) const
{
  robot_state_->setJointGroupPositions(joint_model_group_, joint_values);
  return robot_state_->getJacobian(joint_model_group_);
}

/************************************
 * Equality constraints on X position
 * **********************************/

XPositionConstraint::XPositionConstraint(robot_model::RobotModelConstPtr robot_model, const std::string& group,
                                         const unsigned int num_dofs)
  : BaseConstraint(robot_model, group, num_dofs, 1)
{
}

void XPositionConstraint::parseConstraintMsg(moveit_msgs::Constraints constraints)
{
  ROS_INFO_STREAM("Creating equality constraints on x-position of end-effector.");
  geometry_msgs::Point position =
      constraints.position_constraints.at(0).constraint_region.primitive_poses.at(0).position;
  target_position_ << position.x, position.y, position.z;
}

void XPositionConstraint::function(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::VectorXd> out) const
{
  out[0] = forwardKinematics(x).translation().x() - target_position_.x();
}

void XPositionConstraint::jacobian(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::MatrixXd> out) const
{
  out.row(0) = geometricJacobian(x).row(0);
}

}  // namespace ompl_interface