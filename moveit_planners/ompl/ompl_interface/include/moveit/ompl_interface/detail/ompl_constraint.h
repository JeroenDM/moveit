/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Jeroen De Maeyer */

#pragma once

#include <ompl/base/Constraint.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/Constraints.h>

namespace ompl_interface
{
namespace ob = ompl::base;

/** \brief Abstract base class for differen types of constraint implementations of ompl::base::Constraint
 *
 * @@todo clean-up what is public / protected / private,
 * I was being lax for debugging.
 * */
class BaseConstraint : public ob::Constraint
{
public:
  BaseConstraint(robot_model::RobotModelConstPtr robot_model, const std::string& group, const unsigned int num_dofs,
                 const unsigned int num_cons_ = 3);

  /** \brief initialize constraint based on message content.
   *
   * This is needed, because we cannot call the pure virtual
   * parseConstraintsMsg method from the constructor of this class.
   * */
  void init(moveit_msgs::Constraints constraints);

  /** OMPL's main constraint evaluation function.
   *
   *  OMPL requires you to override at least "function" which represents the constraint F(q) = 0
   * */
  virtual void function(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::VectorXd> out) const = 0;

  /** Optionally you can also provide dF(q)/dq, the Jacobian of  the constriants.
   *
   * */
  virtual void jacobian(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::MatrixXd> out) const = 0;

  // generic helper functions for robot kinematics.
  // @todo Are these actually const, as the robot state is modified? How come it works?
  Eigen::Isometry3d forwardKinematics(const Eigen::Ref<const Eigen::VectorXd>& joint_values) const;
  Eigen::MatrixXd geometricJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_values) const;

  /** \brief Parse bounds on position and orientation parameters from MoveIt's constraint message.
   *
   * This can be non-trivial given the often complex structure of these messages.
   * For the current example with equality constraints it could be to simple
   * to have this separate function instead of using the init function directly.
   * */
  virtual void parseConstraintMsg(moveit_msgs::Constraints constraints) = 0;

protected:
  // MoveIt's robot representation for kinematic calculations
  robot_model::RobotModelConstPtr robot_model_;
  robot_state::RobotStatePtr robot_state_;
  const robot_state::JointModelGroup* joint_model_group_;

  /** \brief Robot link the constraints are applied to. */
  std::string link_name_;

  /** \brief target for equality constraints, nominal value for inequality constraints. */
  Eigen::Vector3d target_position_;

  /** \brief target for equality constraints, nominal value for inequality constraints. */
  Eigen::Quaterniond target_orientation_;
};

/** \brief hardcoded equality constraints on x position.
 *
 * OMPL's constraint planning methods "Atlas" and "TangentBundle"
 * only work for equality constraints, and maybe a very specific version
 * of inequality constraints.
 *
 * This simple hard-coded constraints serves as a simple example
 * that can be solved by all constrained planning methods.
 * */
class XPositionConstraint : public BaseConstraint
{
public:
  XPositionConstraint(robot_model::RobotModelConstPtr robot_model, const std::string& group,
                      const unsigned int num_dofs);
  virtual void parseConstraintMsg(moveit_msgs::Constraints constraints) override;
  virtual void function(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::VectorXd> out) const override;
  virtual void jacobian(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::MatrixXd> out) const override;
};

}  // namespace ompl_interface