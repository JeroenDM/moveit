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

/* Author: Ioan Sucan */

#include <moveit/ompl_interface/planning_context_manager.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/profiler/profiler.h>
#include <utility>

#include <ompl/geometric/planners/AnytimePathShortening.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/pRRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/sbl/pSBL.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/fmt/BFMT.h>
#include <ompl/geometric/planners/pdst/PDST.h>
#include <ompl/geometric/planners/stride/STRIDE.h>
#include <ompl/geometric/planners/rrt/BiTRRT.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include <ompl/geometric/planners/est/BiEST.h>
#include <ompl/geometric/planners/est/ProjEST.h>
#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/prm/SPARS.h>
#include <ompl/geometric/planners/prm/SPARStwo.h>

#include <moveit/ompl_interface/parameterization/joint_space/joint_model_state_space.h>
#include <moveit/ompl_interface/parameterization/work_space/pose_model_state_space.h>

using namespace std::placeholders;

namespace ompl_interface
{
constexpr char LOGNAME[] = "planning_context_manager";

struct PlanningContextManager::CachedContexts
{
  std::map<std::pair<std::string, std::string>, std::vector<ModelBasedPlanningContextPtr> > contexts_;
  std::mutex lock_;
};

}  // namespace ompl_interface

ompl_interface::MultiQueryPlannerAllocator::~MultiQueryPlannerAllocator()
{
  // Store all planner data
  for (const auto& entry : planner_data_storage_paths_)
  {
    ROS_INFO("Storing planner data");
    ob::PlannerData data(planners_[entry.first]->getSpaceInformation());
    planners_[entry.first]->getPlannerData(data);
    storage_.store(data, entry.second.c_str());
  }
}

template <typename T>
ompl::base::PlannerPtr ompl_interface::MultiQueryPlannerAllocator::allocatePlanner(
    const ob::SpaceInformationPtr& si, const std::string& new_name, const ModelBasedPlanningContextSpecification& spec)
{
  // Store planner instance if multi-query planning is enabled
  auto cfg = spec.config_;
  auto it = cfg.find("multi_query_planning_enabled");
  bool multi_query_planning_enabled = false;
  if (it != cfg.end())
  {
    multi_query_planning_enabled = boost::lexical_cast<bool>(it->second);
    cfg.erase(it);
  }
  if (multi_query_planning_enabled)
  {
    // If we already have an instance, use that one
    auto planner_map_it = planners_.find(new_name);
    if (planner_map_it != planners_.end())
      return planner_map_it->second;

    // Certain multi-query planners allow loading and storing the generated planner data. This feature can be
    // selectively enabled for loading and storing using the bool parameters 'load_planner_data' and
    // 'store_planner_data'. The storage file path is set using the parameter 'planner_data_path'.
    // File read and write access are handled by the PlannerDataStorage class. If the file path is invalid
    // an error message is printed and the planner is constructed/destructed with default values.
    it = cfg.find("load_planner_data");
    bool load_planner_data = false;
    if (it != cfg.end())
    {
      load_planner_data = boost::lexical_cast<bool>(it->second);
      cfg.erase(it);
    }
    it = cfg.find("store_planner_data");
    bool store_planner_data = false;
    if (it != cfg.end())
    {
      store_planner_data = boost::lexical_cast<bool>(it->second);
      cfg.erase(it);
    }
    it = cfg.find("planner_data_path");
    std::string planner_data_path;
    if (it != cfg.end())
    {
      planner_data_path = it->second;
      cfg.erase(it);
    }
    // Store planner instance for multi-query use
    planners_[new_name] =
        allocatePlannerImpl<T>(si, new_name, spec, load_planner_data, store_planner_data, planner_data_path);
    return planners_[new_name];
  }
  else
  {
    // Return single-shot planner instance
    return allocatePlannerImpl<T>(si, new_name, spec);
  }
}

template <typename T>
ompl::base::PlannerPtr ompl_interface::MultiQueryPlannerAllocator::allocatePlannerImpl(
    const ob::SpaceInformationPtr& si, const std::string& new_name, const ModelBasedPlanningContextSpecification& spec,
    bool load_planner_data, bool store_planner_data, const std::string& file_path)
{
  ob::PlannerPtr planner;
  // Try to initialize planner with loaded planner data
  if (load_planner_data)
  {
    ROS_INFO("Loading planner data");
    ob::PlannerData data(si);
    storage_.load(file_path.c_str(), data);
    planner.reset(allocatePersistentPlanner<T>(data));
    if (!planner)
      ROS_ERROR_NAMED(LOGNAME,
                      "Creating a '%s' planner from persistent data is not supported. Going to create a new instance.",
                      new_name.c_str());
  }
  if (!planner)
    planner.reset(new T(si));
  if (!new_name.empty())
    planner->setName(new_name);
  planner->params().setParams(spec.config_, true);
  //  Remember which planner instances to store when the destructor is called
  if (store_planner_data)
    planner_data_storage_paths_[new_name] = file_path;
  return planner;
}

// default implementation
template <typename T>
inline ompl::base::Planner*
ompl_interface::MultiQueryPlannerAllocator::allocatePersistentPlanner(const ob::PlannerData& /*data*/)
{
  return nullptr;
};
// TODO: remove when ROS Melodic and older are no longer supported
// namespace is scoped instead of global because of GCC bug 56480
#if OMPL_VERSION_VALUE >= 1005000
namespace ompl_interface
{
template <>
inline ompl::base::Planner*
MultiQueryPlannerAllocator::allocatePersistentPlanner<ompl::geometric::PRM>(const ob::PlannerData& data)
{
  return new og::PRM(data);
};
template <>
inline ompl::base::Planner*
MultiQueryPlannerAllocator::allocatePersistentPlanner<ompl::geometric::PRMstar>(const ob::PlannerData& data)
{
  return new og::PRMstar(data);
};
template <>
inline ompl::base::Planner*
MultiQueryPlannerAllocator::allocatePersistentPlanner<ompl::geometric::LazyPRM>(const ob::PlannerData& data)
{
  return new og::LazyPRM(data);
};
template <>
inline ompl::base::Planner*
MultiQueryPlannerAllocator::allocatePersistentPlanner<ompl::geometric::LazyPRMstar>(const ob::PlannerData& data)
{
  return new og::LazyPRMstar(data);
};
}  // namespace ompl_interface
#endif

ompl_interface::PlanningContextManager::PlanningContextManager(moveit::core::RobotModelConstPtr robot_model,
                                                               constraint_samplers::ConstraintSamplerManagerPtr csm)
  : robot_model_(std::move(robot_model))
  , constraint_sampler_manager_(std::move(csm))
  , max_goal_samples_(10)
  , max_state_sampling_attempts_(4)
  , max_goal_sampling_attempts_(1000)
  , max_planning_threads_(4)
  , max_solution_segment_length_(0.0)
  , minimum_waypoint_count_(2)
{
  cached_contexts_.reset(new CachedContexts());
  registerDefaultPlanners();
}

ompl_interface::PlanningContextManager::~PlanningContextManager() = default;

ompl_interface::ConfiguredPlannerAllocator
ompl_interface::PlanningContextManager::plannerSelector(const std::string& planner) const
{
  auto it = known_planners_.find(planner);
  if (it != known_planners_.end())
    return it->second;
  else
  {
    ROS_ERROR_NAMED(LOGNAME, "Unknown planner: '%s'", planner.c_str());
    return ConfiguredPlannerAllocator();
  }
}

template <typename T>
void ompl_interface::PlanningContextManager::registerPlannerAllocatorHelper(const std::string& planner_id)
{
  registerPlannerAllocator(planner_id, [&](const ob::SpaceInformationPtr& si, const std::string& new_name,
                                           const ModelBasedPlanningContextSpecification& spec) {
    return planner_allocator_.allocatePlanner<T>(si, new_name, spec);
  });
}

void ompl_interface::PlanningContextManager::registerDefaultPlanners()
{
  registerPlannerAllocatorHelper<og::AnytimePathShortening>("geometric::AnytimePathShortening");
  registerPlannerAllocatorHelper<og::BFMT>("geometric::BFMT");
  registerPlannerAllocatorHelper<og::BiEST>("geometric::BiEST");
  registerPlannerAllocatorHelper<og::BiTRRT>("geometric::BiTRRT");
  registerPlannerAllocatorHelper<og::BKPIECE1>("geometric::BKPIECE");
  registerPlannerAllocatorHelper<og::EST>("geometric::EST");
  registerPlannerAllocatorHelper<og::FMT>("geometric::FMT");
  registerPlannerAllocatorHelper<og::KPIECE1>("geometric::KPIECE");
  registerPlannerAllocatorHelper<og::LazyPRM>("geometric::LazyPRM");
  registerPlannerAllocatorHelper<og::LazyPRMstar>("geometric::LazyPRMstar");
  registerPlannerAllocatorHelper<og::LazyRRT>("geometric::LazyRRT");
  registerPlannerAllocatorHelper<og::LBKPIECE1>("geometric::LBKPIECE");
  registerPlannerAllocatorHelper<og::LBTRRT>("geometric::LBTRRT");
  registerPlannerAllocatorHelper<og::PDST>("geometric::PDST");
  registerPlannerAllocatorHelper<og::PRM>("geometric::PRM");
  registerPlannerAllocatorHelper<og::PRMstar>("geometric::PRMstar");
  registerPlannerAllocatorHelper<og::ProjEST>("geometric::ProjEST");
  registerPlannerAllocatorHelper<og::RRT>("geometric::RRT");
  registerPlannerAllocatorHelper<og::RRTConnect>("geometric::RRTConnect");
  registerPlannerAllocatorHelper<og::RRTstar>("geometric::RRTstar");
  registerPlannerAllocatorHelper<og::SBL>("geometric::SBL");
  registerPlannerAllocatorHelper<og::SPARS>("geometric::SPARS");
  registerPlannerAllocatorHelper<og::SPARStwo>("geometric::SPARStwo");
  registerPlannerAllocatorHelper<og::STRIDE>("geometric::STRIDE");
  registerPlannerAllocatorHelper<og::TRRT>("geometric::TRRT");
}

ompl_interface::ConfiguredPlannerSelector ompl_interface::PlanningContextManager::getPlannerSelector() const
{
  return std::bind(&PlanningContextManager::plannerSelector, this, std::placeholders::_1);
}

void ompl_interface::PlanningContextManager::setPlannerConfigurations(
    const planning_interface::PlannerConfigurationMap& pconfig)
{
  planner_configs_ = pconfig;
}

ompl_interface::ModelBasedPlanningContextPtr ompl_interface::PlanningContextManager::getPlanningContext(
    const planning_interface::PlannerConfigurationSettings& config,
    const std::string& state_space_parameterization_type, const moveit_msgs::MotionPlanRequest& req) const
{
  // Check for a cached planning context
  ModelBasedPlanningContextPtr context;

  {
    std::unique_lock<std::mutex> slock(cached_contexts_->lock_);
    auto cached_contexts =
        cached_contexts_->contexts_.find(std::make_pair(config.name, state_space_parameterization_type));
    if (cached_contexts != cached_contexts_->contexts_.end())
    {
      for (const ModelBasedPlanningContextPtr& cached_context : cached_contexts->second)
        if (cached_context.unique())
        {
          ROS_DEBUG_NAMED(LOGNAME, "Reusing cached planning context");
          context = cached_context;
          break;
        }
    }
  }

  // Create a new planning context
  if (!context)
  {
    ModelBasedStateSpaceSpecification space_spec(robot_model_, config.group);
    ModelBasedPlanningContextSpecification context_spec;
    context_spec.config_ = config.config;
    context_spec.planner_selector_ = getPlannerSelector();
    context_spec.constraint_sampler_manager_ = constraint_sampler_manager_;
    context_spec.state_space_ = createStateSpace(state_space_parameterization_type, space_spec);
    context_spec.ompl_simple_setup_.reset(new ompl::geometric::SimpleSetup(context_spec.state_space_));

    ROS_DEBUG_NAMED(LOGNAME, "Creating new planning context");
    context.reset(new ModelBasedPlanningContext(config.name, context_spec));
    {
      std::unique_lock<std::mutex> slock(cached_contexts_->lock_);
      cached_contexts_->contexts_[std::make_pair(config.name, state_space_parameterization_type)].push_back(context);
    }
  }

  context->setMaximumPlanningThreads(max_planning_threads_);
  context->setMaximumGoalSamples(max_goal_samples_);
  context->setMaximumStateSamplingAttempts(max_state_sampling_attempts_);
  context->setMaximumGoalSamplingAttempts(max_goal_sampling_attempts_);
  if (max_solution_segment_length_ > std::numeric_limits<double>::epsilon())
    context->setMaximumSolutionSegmentLength(max_solution_segment_length_);
  context->setMinimumWaypointCount(minimum_waypoint_count_);

  context->setSpecificationConfig(config.config);

  return context;
}

bool ompl_interface::PlanningContextManager::doesGroupHaveIKSolver(const std::string& group_name) const
{
  const moveit::core::JointModelGroup* jmg = robot_model_->getJointModelGroup(group_name);
  if (jmg)
  {
    const std::pair<moveit::core::JointModelGroup::KinematicsSolver,
                    moveit::core::JointModelGroup::KinematicsSolverMap>& ik_solver_pair = jmg->getGroupKinematics();
    bool ik = false;
    // check that we have a direct means to compute IK
    if (ik_solver_pair.first)
    {
      ik = jmg->getVariableCount() == ik_solver_pair.first.bijection_.size();
    }
    // or an IK solver for each of the subgroups
    else if (!ik_solver_pair.second.empty())
    {
      unsigned int variable_count = 0;
      unsigned int bijection_count = 0;
      for (const auto& jt : ik_solver_pair.second)
      {
        variable_count += jt.first->getVariableCount();
        bijection_count += jt.second.bijection_.size();
      }
      if (variable_count == jmg->getVariableCount() && variable_count == bijection_count)
        ik = true;
    }
    return ik;
  }
  else
  {
    ROS_ERROR_NAMED("planning_context_manager", "planning group '%s' not found when testing for IK Solver.",
                    group_name.c_str());
    return false;
  }
}

const std::string& ompl_interface::PlanningContextManager::selectStateSpaceType(
    const moveit_msgs::MotionPlanRequest& req, bool enforce_joint_model_state_space) const
{
  // the user forced us to plan in joint space
  if (enforce_joint_model_state_space)
  {
    return ompl_interface::JointModelStateSpace::PARAMETERIZATION_TYPE;
  }

  // if there are joint or visibility constraints, we need to plan in joint space
  else if (!req.path_constraints.joint_constraints.empty() && !req.path_constraints.visibility_constraints.empty())
  {
    return ompl_interface::JointModelStateSpace::PARAMETERIZATION_TYPE;
  }

  // if we have path constraints and an inverse kinematics solver, we prefer interpolating in pose space
  else if ((!req.path_constraints.position_constraints.empty() ||
            !req.path_constraints.orientation_constraints.empty()) &&
           doesGroupHaveIKSolver(req.group_name))
  {
    return ompl_interface::PoseModelStateSpace::PARAMETERIZATION_TYPE;
  }
  // in all other cases, plan in joint space
  else
  {
    return ompl_interface::JointModelStateSpace::PARAMETERIZATION_TYPE;
  }
}

ompl_interface::ModelBasedStateSpacePtr ompl_interface::PlanningContextManager::createStateSpace(
    const std::string& parameterization_type, const ModelBasedStateSpaceSpecification& space_spec) const
{
  ompl_interface::ModelBasedStateSpacePtr state_space;
  if (parameterization_type == ompl_interface::JointModelStateSpace::PARAMETERIZATION_TYPE)
  {
    state_space = std::make_shared<ompl_interface::JointModelStateSpace>(space_spec);
    state_space->computeLocations();
    return state_space;
  }
  else if (parameterization_type == ompl_interface::PoseModelStateSpace::PARAMETERIZATION_TYPE)
  {
    state_space = std::make_shared<ompl_interface::PoseModelStateSpace>(space_spec);
    state_space->computeLocations();
    return state_space;
  }
  else
  {
    ROS_ERROR_NAMED(LOGNAME, "Unkown state space parameterization type '%s'. No state space created.",
                    parameterization_type.c_str());
    return ompl_interface::ModelBasedStateSpacePtr();
  }
}

ompl_interface::ModelBasedPlanningContextPtr ompl_interface::PlanningContextManager::getPlanningContext(
    const planning_scene::PlanningSceneConstPtr& planning_scene, const moveit_msgs::MotionPlanRequest& req,
    moveit_msgs::MoveItErrorCodes& error_code, const ros::NodeHandle& nh, bool use_constraints_approximation) const
{
  if (req.group_name.empty())
  {
    ROS_ERROR_NAMED(LOGNAME, "No group specified to plan for");
    error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
    return ModelBasedPlanningContextPtr();
  }

  error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;

  if (!planning_scene)
  {
    ROS_ERROR_NAMED(LOGNAME, "No planning scene supplied as input");
    return ModelBasedPlanningContextPtr();
  }

  // identify the correct planning configuration
  auto pc = planner_configs_.end();
  if (!req.planner_id.empty())
  {
    pc = planner_configs_.find(req.planner_id.find(req.group_name) == std::string::npos ?
                                   req.group_name + "[" + req.planner_id + "]" :
                                   req.planner_id);
    if (pc == planner_configs_.end())
      ROS_WARN_NAMED(LOGNAME,
                     "Cannot find planning configuration for group '%s' using planner '%s'. Will use defaults instead.",
                     req.group_name.c_str(), req.planner_id.c_str());
  }

  if (pc == planner_configs_.end())
  {
    pc = planner_configs_.find(req.group_name);
    if (pc == planner_configs_.end())
    {
      ROS_ERROR_NAMED(LOGNAME, "Cannot find planning configuration for group '%s'", req.group_name.c_str());
      return ModelBasedPlanningContextPtr();
    }
  }

  // Check if sampling in JointModelStateSpace is enforced for this group by user.
  // This is done by setting 'enforce_joint_model_state_space' to 'true' for the desired group in ompl_planning.yaml.
  //
  // Some planning problems like orientation path constraints are represented in PoseModelStateSpace and sampled via
  // IK. However consecutive IK solutions are not checked for proximity at the moment and sometimes happen to be
  // flipped, leading to invalid trajectories. This workaround lets the user prevent this problem by forcing rejection
  // sampling in JointModelStateSpace.
  bool enforce_joint_model_state_space = false;
  auto it = pc->second.config.find("enforce_joint_model_state_space");
  if (it != pc->second.config.end() && boost::lexical_cast<bool>(it->second))
  {
    enforce_joint_model_state_space = true;
  }

  const std::string state_space_type = selectStateSpaceType(req, enforce_joint_model_state_space);
  ModelBasedPlanningContextPtr context = getPlanningContext(pc->second, state_space_type, req);

  if (context)
  {
    context->clear();

    moveit::core::RobotStatePtr start_state = planning_scene->getCurrentStateUpdated(req.start_state);

    // Setup the context
    context->setPlanningScene(planning_scene);
    context->setMotionPlanRequest(req);
    context->setCompleteInitialState(*start_state);

    context->setPlanningVolume(req.workspace_parameters);
    if (!context->setPathConstraints(req.path_constraints, &error_code))
      return ModelBasedPlanningContextPtr();

    if (!context->setGoalConstraints(req.goal_constraints, req.path_constraints, &error_code))
      return ModelBasedPlanningContextPtr();

    try
    {
      context->configure(nh, use_constraints_approximation);
      ROS_DEBUG_NAMED(LOGNAME, "%s: New planning context is set.", context->getName().c_str());
      error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    }
    catch (ompl::Exception& ex)
    {
      ROS_ERROR_NAMED(LOGNAME, "OMPL encountered an error: %s", ex.what());
      context.reset();
    }
  }

  return context;
}
