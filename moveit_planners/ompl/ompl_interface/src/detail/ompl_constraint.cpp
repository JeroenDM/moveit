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
  Eigen::MatrixXd jacobian;
  bool success = robot_state_->getJacobian(joint_model_group_, joint_model_group_->getLinkModel(link_name_),
                                           Eigen::Vector3d(0.0, 0.0, 0.0), jacobian);
  assert(success);
  return jacobian;
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

/******************************************
 * Angle-axis error constraints
 * ****************************************/
void AngleAxisConstraint::parseConstraintMsg(moveit_msgs::Constraints constraints)
{
  bounds_.clear();
  bounds_ = orientationConstraintMsgToBoundVector(constraints.orientation_constraints.at(0));
  // ROS_INFO_STREAM("Parsing angle-axis constraints");
  // ROS_INFO_STREAM("Parsed rx / roll constraints" << bounds_[0]);
  // ROS_INFO_STREAM("Parsed ry / pitch constraints" << bounds_[1]);
  // ROS_INFO_STREAM("Parsed rz / yaw constraints" << bounds_[2]);

  tf::quaternionMsgToEigen(constraints.orientation_constraints.at(0).orientation, target_orientation_);

  link_name_ = constraints.orientation_constraints.at(0).link_name;
}

Eigen::VectorXd AngleAxisConstraint::calcError(const Eigen::Ref<const Eigen::VectorXd>& x) const
{
  // TODO(jeroendm) I'm not sure yet whether I want the error expressed in the current ee_frame, or target_frame,
  // or world frame. This implementation expressed the error in the end-effector frame.
  Eigen::Matrix3d Rerror = forwardKinematics(x).rotation().transpose() * target_orientation_;
  Eigen::AngleAxisd aa(Rerror);
  double angle = aa.angle();
  assert(std::abs(angle) < M_PI);
  return aa.axis() * angle;
}

// mysterious minus sign in Jacobian, found through trial and error...
Eigen::MatrixXd AngleAxisConstraint::calcErrorJacobian(const Eigen::Ref<const Eigen::VectorXd>& x) const
{
  // Eigen::AngleAxisd aa {forwardKinematics(x).rotation().transpose() * target_as_quat_};

  Eigen::AngleAxisd aa{ forwardKinematics(x).rotation() };
  // TODO(jeroendm) Find out where the mysterious minus sign comes from
  return -angularVelocityToAngleAxis(aa.angle(), aa.axis()) * geometricJacobian(x).bottomRows(3);
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

  link_name_ = constraints.position_constraints.at(0).link_name;
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

std::vector<Bounds> orientationConstraintMsgToBoundVector(moveit_msgs::OrientationConstraint ori_con)
{
  std::vector<double> dims{ ori_con.absolute_x_axis_tolerance, ori_con.absolute_y_axis_tolerance,
                            ori_con.absolute_z_axis_tolerance };

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
      ROS_WARN_STREAM("Only a single position constraints supported. Using the first one.");
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
      ROS_WARN_STREAM("Only a single orientation constraints supported. Using the first one.");
    }

    auto ori_con = std::make_shared<AngleAxisConstraint>(robot_model, group, num_dofs);
    ori_con->init(constraints);
    return ori_con;
  }
  else
  {
    ROS_ERROR_STREAM("No path constraints found in planning request.");
    return nullptr;
  }
}

/******************************************
 * Angular velocity conversion
 * ****************************************/
Eigen::Matrix3d angularVelocityToAngleAxis(double angle, Eigen::Vector3d axis)
{
  Eigen::Matrix3d E;

  double t{ std::abs(angle) };
  Eigen::Vector3d r{ axis * angle };
  Eigen::Matrix3d r_skew;
  r_skew << 0, -r[2], r[1], r[2], 0, -r[0], -r[1], r[0], 0;

  double C;
  C = (1 - 0.5 * t * std::sin(t) / (1 - std::cos(t)));

  E = Eigen::Matrix3d::Identity() - 0.5 * r_skew + r_skew * r_skew / (t * t) * C;
  return E;
}

}  // namespace ompl_interface