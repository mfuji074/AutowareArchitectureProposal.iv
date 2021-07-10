#include <lane_change_planner/data_manager.h>
#include <lane_change_planner/route_handler.h>
#include <lane_change_planner/state/common_functions.h>
#include <lane_change_planner/state/multipolicy_decision_making.h>
#include <lane_change_planner/utilities.h>

#include <lanelet2_extension/utility/utilities.h>
#include <tf2/utils.h>

namespace lane_change_planner
{
MultipolicyDecisionMaking::MultipolicyDecisionMaking(
  const Status & status, const std::shared_ptr<DataManager> & data_manager_ptr,
  const std::shared_ptr<RouteHandler> & route_handler_ptr)
: StateBase(status, data_manager_ptr, route_handler_ptr)
{
}

State MultipolicyDecisionMaking::getCurrentState() const { return current_state_; }

void MultipolicyDecisionMaking::entry()
{
  current_state_ = State::FOLLOWING_LANE;
  next_state_ = State::FOLLOWING_LANE;

  ros_parameters_ = data_manager_ptr_->getLaneChangerParameters();
  lane_change_approved_ = false;
  force_lane_change_ = false;
  status_.lane_change_available = false;
  status_.lane_change_ready = false;
}

State MultipolicyDecisionMaking::getNextState() const
{
  return next_state_;
}

autoware_planning_msgs::PathWithLaneId MultipolicyDecisionMaking::getPath() const
{
  return status_.lane_change_path.path;
}

void MultipolicyDecisionMaking::update()
{
  // update input data
  current_twist_ = data_manager_ptr_->getCurrentSelfVelocity();
  current_pose_ = data_manager_ptr_->getCurrentSelfPose();
  dynamic_objects_ = data_manager_ptr_->getDynamicObjects();

  next_state_ = current_state_;
  lanelet::ConstLanelet current_lane;
  const double backward_path_length = ros_parameters_.backward_path_length;
  const double forward_path_length = ros_parameters_.forward_path_length;

  bool found_valid_path = false;
  // update lanes
  {
    if (!route_handler_ptr_->getClosestLaneletWithinRoute(current_pose_.pose, &current_lane)) {
      ROS_ERROR("failed to find closest lanelet within route!!!");
      return;
    }
    current_lanes_ = route_handler_ptr_->getLaneletSequence(
      current_lane, current_pose_.pose, backward_path_length, forward_path_length);
    const double lane_change_prepare_length =
      current_twist_->twist.linear.x * ros_parameters_.lane_change_prepare_duration;
    lanelet::ConstLanelets current_check_lanes = route_handler_ptr_->getLaneletSequence(
      current_lane, current_pose_.pose, 0.0, lane_change_prepare_length);
    lanelet::ConstLanelet lane_change_lane;
    if (route_handler_ptr_->getLaneChangeTarget(current_check_lanes, &lane_change_lane)) {
      constexpr double lane_change_lane_length = 200.0;
      lane_change_lanes_ = route_handler_ptr_->getLaneletSequence(
        lane_change_lane, current_pose_.pose, lane_change_lane_length, lane_change_lane_length);
    } else {
      lane_change_lanes_.clear();
    }
  }
  // update lane_follow_path
  {
    constexpr double check_distance = 100.0;
    const double lane_change_prepare_duration = ros_parameters_.lane_change_prepare_duration;
    const double lane_changing_duration = ros_parameters_.lane_changing_duration;
    const double minimum_lane_change_length = ros_parameters_.minimum_lane_change_length;
    status_.lane_follow_path = route_handler_ptr_->getReferencePath(
      current_lanes_, current_pose_.pose, backward_path_length, forward_path_length,
      ros_parameters_);

    if (lane_change_lanes_.empty()) {
      next_state_ = State::FOLLOWING_LANE;
    } else {
      // find candidate paths
      const auto lane_change_paths = route_handler_ptr_->getLaneChangePaths(
        current_lanes_, lane_change_lanes_, current_pose_.pose, current_twist_->twist,
        ros_parameters_);

      // get lanes used for detection
      lanelet::ConstLanelets check_lanes;
      if (!lane_change_paths.empty()) {
        const auto & longest_path = lane_change_paths.front();
        // we want to see check_distance [m] behind vehicle so add lane changing length
        const double check_distance_with_path =
          check_distance + longest_path.preparation_length + longest_path.lane_change_length;
        check_lanes = route_handler_ptr_->getCheckTargetLanesFromPath(
          longest_path.path, lane_change_lanes_, check_distance_with_path);
      }

      // select valid path
      const auto valid_paths = state_machine::common_functions::selectValidPaths(
        lane_change_paths, current_lanes_, check_lanes, route_handler_ptr_->getOverallGraph(),
        current_pose_.pose, route_handler_ptr_->isInGoalRouteSection(current_lanes_.back()),
        route_handler_ptr_->getGoalPose());
      debug_data_.lane_change_candidate_paths = valid_paths;
      found_valid_path = !valid_paths.empty();

      // get explored_paths (valid_paths + lane_following_path)
      change_lane_size_ = valid_paths.size();
      auto lane_follow_path = route_handler_ptr_->getLaneChangePaths(
        current_lanes_, current_lanes_, current_pose_.pose, current_twist_->twist,
        ros_parameters_);
      std::vector<LaneChangePath> explored_paths = valid_paths;
      explored_paths.insert(explored_paths.end(), lane_follow_path.begin(), lane_follow_path.begin()+1);

      // add decelerated paths to explored_paths
      change_lane_size_ *= 2;
      auto explored_paths_decelerated = explored_paths;
      geometry_msgs::Twist velocity_max;
      for (size_t i = 0; i < explored_paths_decelerated.size(); i++) {
        for (size_t j = 0; j < explored_paths_decelerated[i].path.points.size(); j++) {
          velocity_max.linear.x = explored_paths_decelerated[i].path.points[j].point.twist.linear.x*0.5;
          explored_paths_decelerated[i].path.points[j].point.twist = velocity_max;
        }
        if (i < explored_paths_decelerated.size()-1) {
          explored_paths.insert(explored_paths.end()-1, explored_paths_decelerated[i]);
        } else {
          explored_paths.insert(explored_paths.end(), explored_paths_decelerated[i]);
        }
      }

      //std::cout << "current_pose : " << current_pose_.pose << std::endl;
      //std::cout << "goal_pose : " << route_handler_ptr_->getGoalPose() << std::endl;

      // explore best next state
      LaneChangePath selected_path;
      next_state_ = exploreBestState(explored_paths, current_lanes_, check_lanes, dynamic_objects_, current_pose_.pose,
                                    current_twist_->twist, ros_parameters_, &selected_path);
      debug_data_.selected_path = selected_path.path;
      status_.lane_change_path = selected_path;
    }
    status_.lane_follow_lane_ids = util::getIds(current_lanes_);
    status_.lane_change_lane_ids = util::getIds(lane_change_lanes_);
  }
}

void MultipolicyDecisionMaking::initBeliefState()
{
  // Estimate other vehicles' state from perception
}

State MultipolicyDecisionMaking::exploreBestState(
  const std::vector<LaneChangePath> & paths, const lanelet::ConstLanelets & current_lanes,
  const lanelet::ConstLanelets & target_lanes,
  const autoware_perception_msgs::DynamicObjectArray::ConstPtr & dynamic_objects,
  const geometry_msgs::Pose & current_pose, const geometry_msgs::Twist & current_twist,
  const LaneChangerParameters & ros_parameters, LaneChangePath * selected_path)
{
  // parameter
  std::vector<double> costs;
  double cost_min = 1.0e10;
  const double coeff_efficiency = 1.0e-3;

  for (const auto & path : paths) {
    // safety
    double cost_safety = runForwardSimulation(
        path.path, current_lanes, target_lanes, dynamic_objects, current_pose, current_twist,
        ros_parameters, true, path.acceleration);

    // efficiency
    std::vector<uint64_t> target_lane_ids = util::getIds(target_lanes);
    int64_t goal_lane_id = target_lane_ids[target_lane_ids.size()-1];
    autoware_planning_msgs::PathPointWithLaneId pathpointwithlaneid_terminal = path.path.points[path.path.points.size()-1];
    int64_t path_terminal_lane_id = pathpointwithlaneid_terminal.lane_ids[pathpointwithlaneid_terminal.lane_ids.size()-1];
    double cost_efficiency = coeff_efficiency * (double)std::abs(path_terminal_lane_id - goal_lane_id);

    // cost
    double cost = cost_safety + cost_efficiency;
    costs.push_back(cost);
    if (cost < cost_min) {
      *selected_path = path;
      cost_min = cost;
    }

    debug_data_.mpdm_total_costs.push_back(cost);
    debug_data_.mpdm_safety_costs.push_back(cost_safety);
    debug_data_.mpdm_efficiency_costs.push_back(cost_efficiency);
  }
  /*
  std::cout << "costs : ";
  for (auto & item : costs) {
    std::cout << item << ",  ";
  }
  std::cout << std::endl;
  */
  if (cost_min < 0.0) {
    return State::FOLLOWING_LANE;
  }

  std::vector<double>::iterator iter = std::min_element(costs.begin(), costs.end());
  size_t index = std::distance(costs.begin(), iter);
  if (index > (change_lane_size_-1)) {
    return State::FOLLOWING_LANE;
  } else {
    return State::EXECUTING_LANE_CHANGE;
  }
}

double MultipolicyDecisionMaking::runForwardSimulation(
  const autoware_planning_msgs::PathWithLaneId & path, const lanelet::ConstLanelets & current_lanes,
  const lanelet::ConstLanelets & target_lanes,
  const autoware_perception_msgs::DynamicObjectArray::ConstPtr & dynamic_objects,
  const geometry_msgs::Pose & current_pose, const geometry_msgs::Twist & current_twist,
  const LaneChangerParameters & ros_parameters, const bool use_buffer, const double acceleration)
{
  if (path.points.empty()) {
    return -1.0;
  }
  if (target_lanes.empty() || current_lanes.empty()) {
    return -1.0;
  }

  double cost = 0.0;
  if (dynamic_objects == nullptr) {
    return cost;
  }
  const auto arc = lanelet::utils::getArcCoordinates(current_lanes, current_pose);
  constexpr double check_distance = 100.0;

  // parameters
  const double time_resolution = ros_parameters.prediction_time_resolution;
  const double vehicle_width = ros_parameters.vehicle_width;
  double buffer;
  double lateral_buffer;
  if (use_buffer) {
    buffer = ros_parameters.hysteresis_buffer_distance;
    lateral_buffer = 0.5;
  } else {
    buffer = 0.0;
    lateral_buffer = 0.0;
  }
  const double target_lane_check_end_time =
    ros_parameters.lane_change_prepare_duration + ros_parameters.lane_changing_duration;

  // find obstacle in lane change target lanes
  // retrieve lanes that are merging target lanes as well
  const auto target_lane_object_indices =
    util::filterObjectsByLanelets(*dynamic_objects, target_lanes);

  // find objects in current lane
  const auto current_lane_object_indices_lanelet = util::filterObjectsByLanelets(
    *dynamic_objects, current_lanes, arc.length, arc.length + check_distance);
  const auto current_lane_object_indices = util::filterObjectsByPath(
    *dynamic_objects, current_lane_object_indices_lanelet, path,
    vehicle_width / 2 + lateral_buffer);

  const auto vehicle_predicted_path = util::convertToPredictedPath(
    path, current_twist, current_pose, target_lane_check_end_time, time_resolution, acceleration);

  cost = computeCost(vehicle_predicted_path, target_lanes, current_lane_object_indices, target_lane_object_indices,
                      dynamic_objects, current_pose, current_twist, ros_parameters, use_buffer);

  return cost;
}

double MultipolicyDecisionMaking::computeCost(
  const autoware_perception_msgs::PredictedPath & vehicle_predicted_path, const lanelet::ConstLanelets & target_lanes,
  const std::vector<size_t> & current_lane_object_indices, const std::vector<size_t> & target_lane_object_indices,
  const autoware_perception_msgs::DynamicObjectArray::ConstPtr & dynamic_objects,
  const geometry_msgs::Pose & current_pose, const geometry_msgs::Twist & current_twist,
  const LaneChangerParameters & ros_parameters, const bool use_buffer)
{
  // parameters
  const double coeff_safety = 1.0;
  const double time_resolution = ros_parameters.prediction_time_resolution;
  const double prediction_duration = ros_parameters.prediction_duration;
  const double min_thresh = ros_parameters.min_stop_distance;
  const double stop_time = ros_parameters.stop_time;
  const double vehicle_width = ros_parameters.vehicle_width;
  double buffer;
  double lateral_buffer;
  if (use_buffer) {
    buffer = ros_parameters.hysteresis_buffer_distance;
    lateral_buffer = 0.5;
  } else {
    buffer = 0.0;
    lateral_buffer = 0.0;
  }
  double current_lane_check_start_time = 0.0;
  const double current_lane_check_end_time =
    ros_parameters.lane_change_prepare_duration + ros_parameters.lane_changing_duration;
  double target_lane_check_start_time = 0.0;
  const double target_lane_check_end_time =
    ros_parameters.lane_change_prepare_duration + ros_parameters.lane_changing_duration;
  if (!ros_parameters.enable_collision_check_at_prepare_phase) {
    current_lane_check_start_time = ros_parameters.lane_change_prepare_duration;
    target_lane_check_start_time = ros_parameters.lane_change_prepare_duration;
  }
  // Cost
  double cost_safety = 0.0;

  // Efficiency
  {

  }

  // Safety
  {
    // Collision check for objects in current lane
    for (const auto & i : current_lane_object_indices) {
      const auto & obj = dynamic_objects->objects.at(i);
      std::vector<autoware_perception_msgs::PredictedPath> predicted_paths;
      predicted_paths = obj.state.predicted_paths;
      for (const auto & obj_path : predicted_paths) {
        double distance = util::getDistanceBetweenPredictedPaths(
          obj_path, vehicle_predicted_path, current_lane_check_start_time,
          current_lane_check_end_time, time_resolution);
        double thresh;
        if (state_machine::common_functions::isObjectFront(current_pose, obj.state.pose_covariance.pose)) {
          thresh = util::l2Norm(current_twist.linear) * stop_time;
        } else {
          thresh = util::l2Norm(obj.state.twist_covariance.twist.linear) * stop_time;
        }
        thresh = std::max(thresh, min_thresh);
        thresh += buffer;
        cost_safety += coeff_safety/((distance - thresh)*(distance - thresh));
      }
    }

    // Collision check for objects in lane change target lane
    for (const auto & i : target_lane_object_indices) {
      const auto & obj = dynamic_objects->objects.at(i);
      std::vector<autoware_perception_msgs::PredictedPath> predicted_paths;
      predicted_paths = obj.state.predicted_paths;

      bool is_object_in_target = false;
      if (ros_parameters.use_predicted_path_outside_lanelet) {
        is_object_in_target = true;
      } else {
        for (const auto & llt : target_lanes) {
          if (lanelet::utils::isInLanelet(obj.state.pose_covariance.pose, llt))
            is_object_in_target = true;
        }
      }

      if (is_object_in_target) {
        for (const auto & obj_path : predicted_paths) {
          const double distance = util::getDistanceBetweenPredictedPaths(
            obj_path, vehicle_predicted_path, target_lane_check_start_time,
            target_lane_check_end_time, time_resolution);
          double thresh;
          if (state_machine::common_functions::isObjectFront(current_pose, obj.state.pose_covariance.pose)) {
            thresh = util::l2Norm(current_twist.linear) * stop_time;
          } else {
            thresh = util::l2Norm(obj.state.twist_covariance.twist.linear) * stop_time;
          }
          thresh = std::max(thresh, min_thresh);
          thresh += buffer;
          cost_safety += coeff_safety/((distance - thresh)*(distance - thresh));
        }
      }
      else {
        const double distance = util::getDistanceBetweenPredictedPathAndObject(
          obj, vehicle_predicted_path, target_lane_check_start_time, target_lane_check_end_time,
          time_resolution);
        double thresh = min_thresh;
        if (state_machine::common_functions::isObjectFront(current_pose, obj.state.pose_covariance.pose)) {
          thresh = std::max(thresh, util::l2Norm(current_twist.linear) * stop_time);
        }
        thresh += buffer;
        cost_safety += coeff_safety/((distance - thresh)*(distance - thresh));
      }
    }
  }

  return cost_safety;
}

}  // namespace lane_change_planner
