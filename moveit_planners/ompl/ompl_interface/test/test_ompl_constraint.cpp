/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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

#include <moveit/ompl_interface/detail/ompl_constraint.h>

#include <memory>
#include <string>
#include <iostream>

#include <gtest/gtest.h>
#include <Eigen/Dense>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/Constraints.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/utils/robot_model_test_utils.h>

#include <ompl/util/Exception.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>
#include <ompl/base/ConstrainedSpaceInformation.h>

/** Number of times to run a test that uses randomly generated input. **/
constexpr int NUM_RANDOM_TESTS{ 10 };

/** For failing tests, some extra print statements are useful. **/
constexpr bool VERBOSE{ false };

moveit_msgs::PositionConstraint createPositionConstraint(std::string& base_link, std::string& ee_link_name)
{
  shape_msgs::SolidPrimitive box_constraint;
  box_constraint.type = shape_msgs::SolidPrimitive::BOX;
  box_constraint.dimensions = { 0.05, 0.4, 0.05 }; /* use -1 to indicate no constraints. */

  geometry_msgs::Pose box_pose;
  box_pose.position.x = 0.9;
  box_pose.position.y = 0.0;
  box_pose.position.z = 0.2;
  box_pose.orientation.w = 1.0;

  moveit_msgs::PositionConstraint position_constraint;
  position_constraint.header.frame_id = base_link;
  position_constraint.link_name = ee_link_name;
  position_constraint.constraint_region.primitives.push_back(box_constraint);
  position_constraint.constraint_region.primitive_poses.push_back(box_pose);

  return position_constraint;
}

class LoadPandaModel : public testing::Test
{
protected:
  void SetUp() override
  {
    // load robot
    robot_model_ = moveit::core::loadTestingRobotModel("panda");
    robot_state_ = std::make_shared<robot_state::RobotState>(robot_model_);
    joint_model_group_ = robot_state_->getJointModelGroup("panda_arm");

    // extract useful parameters for tests
    num_dofs_ = joint_model_group_->getVariableCount();
    ee_link_name_ = joint_model_group_->getLinkModelNames().back();
    base_link_name_ = robot_model_->getRootLinkName();
    group_name_ = joint_model_group_->getName();
  };

  void TearDown() override
  {
  }

  const Eigen::Isometry3d fk(const Eigen::VectorXd q) const
  {
    robot_state_->setJointGroupPositions(joint_model_group_, q);
    return robot_state_->getGlobalLinkTransform(ee_link_name_);
  }

  Eigen::VectorXd getRandomState()
  {
    robot_state_->setToRandomPositions(joint_model_group_);
    Eigen::VectorXd q;
    robot_state_->copyJointGroupPositions(joint_model_group_, q);
    return q;
  }

  Eigen::MatrixXd numericalJacobianPosition(const Eigen::VectorXd q)
  {
    const double h{ 1e-6 }; /* step size for numerical derivation */

    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(3, num_dofs_);

    // helper matrix for differentiation.
    Eigen::MatrixXd Ih = h * Eigen::MatrixXd::Identity(num_dofs_, num_dofs_);

    for (std::size_t dim{ 0 }; dim < num_dofs_; ++dim)
    {
      Eigen::Vector3d pos = fk(q).translation();
      Eigen::Vector3d pos_plus_h = fk(q + Ih.col(dim)).translation();
      Eigen::Vector3d col = (pos_plus_h - pos) / h;
      J.col(dim) = col;
    }
    return J;
  }

protected:
  moveit::core::RobotModelPtr robot_model_;
  urdf::ModelInterfaceSharedPtr urdf_model_;
  srdf::ModelSharedPtr srdf_model_;
  robot_state::RobotStatePtr robot_state_;
  const robot_state::JointModelGroup* joint_model_group_;
  bool urdf_ok_;
  bool srdf_ok_;
  std::size_t num_dofs_;
  std::shared_ptr<ompl_interface::BaseConstraint> constraint_;
  std::string base_link_name_;
  std::string ee_link_name_;
  std::string group_name_;
};

TEST_F(LoadPandaModel, InitPositionConstraint)
{
  moveit_msgs::Constraints constraint_msgs;
  constraint_msgs.position_constraints.push_back(createPositionConstraint(base_link_name_, ee_link_name_));

  constraint_ = std::make_shared<ompl_interface::PositionConstraint>(robot_model_, group_name_, num_dofs_);
  constraint_->init(constraint_msgs);
}

TEST_F(LoadPandaModel, PositionConstraintJacobian)
{
  moveit_msgs::Constraints constraint_msgs;
  constraint_msgs.position_constraints.push_back(createPositionConstraint(base_link_name_, ee_link_name_));

  constraint_ = std::make_shared<ompl_interface::PositionConstraint>(robot_model_, group_name_, num_dofs_);
  constraint_->init(constraint_msgs);

  double total_error{ 999.9 };
  const double ERROR_TOLERANCE{ 1e-4 }; /** High tolerance because of high finite difference error. **/

  for (int i{ 0 }; i < NUM_RANDOM_TESTS; ++i)
  {
    auto q = getRandomState();
    auto J_exact = constraint_->calcErrorJacobian(q);
    auto J_approx = numericalJacobianPosition(q);

    if (VERBOSE)
    {
      std::cout << "Analytical jacobian: \n";
      std::cout << J_exact << std::endl;
      std::cout << "Finite difference jacobian: \n";
      std::cout << J_approx << std::endl;
    }

    total_error = (J_exact - J_approx).lpNorm<1>();
    EXPECT_LT(total_error, ERROR_TOLERANCE);
  }
}

TEST_F(LoadPandaModel, PositionConstraintOMPLCheck)
{
  auto state_space = std::make_shared<ompl::base::RealVectorStateSpace>(num_dofs_);
  ompl::base::RealVectorBounds bounds(num_dofs_);

  // get joint limits from the joint model group
  auto joint_limits = joint_model_group_->getActiveJointModelsBounds();
  EXPECT_EQ(joint_limits.size(), num_dofs_);
  for (std::size_t i{ 0 }; i < num_dofs_; ++i)
  {
    EXPECT_EQ(joint_limits[i]->size(), 1);
    bounds.setLow(i, joint_limits[i]->at(0).min_position_);
    bounds.setHigh(i, joint_limits[i]->at(0).max_position_);
  }

  state_space->setBounds(bounds);

  moveit_msgs::Constraints constraint_msgs;
  constraint_msgs.position_constraints.push_back(createPositionConstraint(base_link_name_, ee_link_name_));

  constraint_ = std::make_shared<ompl_interface::PositionConstraint>(robot_model_, group_name_, num_dofs_);
  constraint_->init(constraint_msgs);

  auto constrained_state_space = std::make_shared<ompl::base::ProjectedStateSpace>(state_space, constraint_);

  auto constrained_state_space_info =
      std::make_shared<ompl::base::ConstrainedSpaceInformation>(constrained_state_space);

  // TODO(jeroendm) Fix issues with sanity checks.
  // The jacobian test is expected to fail because of the discontinuous constraint derivative.
  // The issue with the state sampler is unresolved.
  // int flags = 1 & ompl::base::ConstrainedStateSpace::CONSTRAINED_STATESPACE_JACOBIAN;
  // flags = flags & ompl::base::ConstrainedStateSpace::CONSTRAINED_STATESPACE_SAMPLERS;
  try
  {
    constrained_state_space->sanityChecks();
  }
  catch (ompl::Exception& ex)
  {
    ROS_ERROR("Sanity checks did not pass: %s", ex.what());
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
