#include "moveit/ompl_interface/detail/ompl_constraint.h"

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/Constraints.h>
#include <eigen_conversions/eigen_msg.h>

namespace ompl_interface
{
double Bounds::distance(double value) const
{
  if (value < lower)
    return lower - value;
  else if (value > upper)
    return value - upper;
  else
    return 0.0;
}

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

void BaseConstraint::function(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::VectorXd> out) const
{
  auto current_values = calcError(x);
  for (std::size_t i{ 0 }; i < bounds_.size(); ++i)
  {
    out[i] = bounds_[i].distance(current_values[i]);
  }
}

void BaseConstraint::jacobian(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::MatrixXd> out) const
{
  auto current_values = calcError(x);
  auto current_jacobian = calcErrorJacobian(x);

  for (std::size_t i{ 0 }; i < bounds_.size(); ++i)
  {
    if (current_values[i] > bounds_[i].upper + getTolerance())
    {
      out.row(i) = current_jacobian.row(i);
    }
    else if (current_values[i] < bounds_[i].lower - getTolerance())
    {
      out.row(i) = -current_jacobian.row(i);
    }
    else
    {
      out.row(i) = Eigen::VectorXd::Zero(n_);
    }
  }
}

/******************************************
 * Position constraints
 * ****************************************/

void PositionConstraint::parseConstraintMsg(moveit_msgs::Constraints constraints)
{
  bounds_.clear();
  bounds_ = positionConstraintMsgToBoundVector(constraints.position_constraints.at(0));
  // ROS_INFO_STREAM("Parsed x constraints" << bounds_[0]);
  // ROS_INFO_STREAM("Parsed y constraints" << bounds_[1]);
  // ROS_INFO_STREAM("Parsed z constraints" << bounds_[2]);

  // extract target / nominal value
  geometry_msgs::Point position =
      constraints.position_constraints.at(0).constraint_region.primitive_poses.at(0).position;
  target_position_ << position.x, position.y, position.z;

  tf::quaternionMsgToEigen(constraints.position_constraints[0].constraint_region.primitive_poses[0].orientation,
                           target_orientation_);

  link_name_ = constraints.position_constraints.at(0).link_name;
}

Eigen::VectorXd PositionConstraint::calcError(const Eigen::Ref<const Eigen::VectorXd>& x) const
{
  return target_orientation_.matrix().transpose() * (forwardKinematics(x).translation() - target_position_);
}

Eigen::MatrixXd PositionConstraint::calcErrorJacobian(const Eigen::Ref<const Eigen::VectorXd>& x) const
{
  return target_orientation_.matrix().transpose() * geometricJacobian(x).topRows(3);
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
  // std::cout << "target: " << target_position_.transpose() << std::endl;
  // std::cout << "fk pose error: " << forwardKinematics(x).translation().transpose() << std::endl;
}

void XPositionConstraint::jacobian(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::MatrixXd> out) const
{
  out.row(0) = geometricJacobian(x).row(0);
}

/************************************
 * Message conversion functions
 * **********************************/
std::vector<Bounds> positionConstraintMsgToBoundVector(moveit_msgs::PositionConstraint pos_con)
{
  auto dims = pos_con.constraint_region.primitives.at(0).dimensions;

  // dimension of -1 signifies unconstrained parameter, so set to infinity
  for (auto& dim : dims)
  {
    if (dim == -1)
      dim = std::numeric_limits<double>::infinity();
  }

  return { { -dims[0] / 2, dims[0] / 2 }, { -dims[1] / 2, dims[1] / 2 }, { -dims[2] / 2, dims[2] / 2 } };
}

/******************************************
 * Constraint Factory
 * ****************************************/

std::shared_ptr<BaseConstraint> createConstraint(robot_model::RobotModelConstPtr robot_model, const std::string& group,
                                                 moveit_msgs::Constraints constraints)
{
  std::size_t num_dofs = robot_model->getJointModelGroup(group)->getVariableCount();
  std::size_t num_pos_con = constraints.position_constraints.size();
  std::size_t num_ori_con = constraints.orientation_constraints.size();

  if (num_pos_con > 0 && num_ori_con > 0)
  {
    ROS_ERROR_STREAM("Combining position and orientation constraints not implemented yet.");
    return nullptr;
  }
  else if (num_pos_con > 0)
  {
    if (num_pos_con > 1)
    {
      ROS_ERROR_STREAM("Only a single position constraints supported. Using the first one.");
    }
    auto pos_con = std::make_shared<PositionConstraint>(robot_model, group, num_dofs);
    // auto pos_con = std::make_shared<XPositionConstraint>(robot_model, group, num_dofs);
    pos_con->init(constraints);
    return pos_con;
  }
  else if (num_ori_con > 0)
  {
    if (num_ori_con > 1)
    {
      ROS_ERROR_STREAM("Only a single orientation constraints supported. Using the first one.");
    }

    ROS_ERROR_STREAM("Orientation constraints not implemented yet.");
    return nullptr;
  }
  else
  {
    ROS_ERROR_STREAM("No path constraints found in planning request.");
    return nullptr;
  }
}

}  // namespace ompl_interface