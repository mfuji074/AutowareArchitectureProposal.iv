#ifndef LANE_CHANGE_PLANNER_MULTIPOLICY_DECISION_MAKING_H
#define LANE_CHANGE_PLANNER_MULTIPOLICY_DECISION_MAKING_H

#include <autoware_perception_msgs/DynamicObjectArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <lane_change_planner/state/state_base_class.h>
#include <lanelet2_core/primitives/Primitive.h>

#include <memory>

namespace lane_change_planner
{
class MultipolicyDecisionMaking : public StateBase
{
private:
  geometry_msgs::PoseStamped current_pose_;
  geometry_msgs::TwistStamped::ConstPtr current_twist_;
  autoware_perception_msgs::DynamicObjectArray::ConstPtr dynamic_objects_;
  bool lane_change_approved_;
  bool force_lane_change_;
  lanelet::ConstLanelets current_lanes_;
  lanelet::ConstLanelets lane_change_lanes_;

  void initBeliefState();
  State exploreBestState(
    const std::vector<LaneChangePath> & paths, const lanelet::ConstLanelets & current_lanes,
    const lanelet::ConstLanelets & target_lanes,
    const autoware_perception_msgs::DynamicObjectArray::ConstPtr & dynamic_objects,
    const geometry_msgs::Pose & current_pose, const geometry_msgs::Twist & current_twist,
    const LaneChangerParameters & ros_parameters, LaneChangePath * selected_path);
  double runForwardSimulation(
    const autoware_planning_msgs::PathWithLaneId & path, const lanelet::ConstLanelets & current_lanes,
    const lanelet::ConstLanelets & target_lanes,
    const autoware_perception_msgs::DynamicObjectArray::ConstPtr & dynamic_objects,
    const geometry_msgs::Pose & current_pose, const geometry_msgs::Twist & current_twist,
    const LaneChangerParameters & ros_parameters, const bool use_buffer, const double acceleration);
  double computeCost(
    const autoware_perception_msgs::PredictedPath & vehicle_predicted_path, const lanelet::ConstLanelets & target_lanes,
    const std::vector<size_t> & current_lane_object_indices, const std::vector<size_t> & target_lane_object_indices,
    const autoware_perception_msgs::DynamicObjectArray::ConstPtr & dynamic_objects,
    const geometry_msgs::Pose & current_pose, const geometry_msgs::Twist & current_twist,
    const LaneChangerParameters & ros_parameters, const bool use_buffer);

  size_t change_lane_size_;
  State current_state_;
  State next_state_;

public:
  MultipolicyDecisionMaking(
  const Status & status, const std::shared_ptr<DataManager> & data_manager_ptr,
  const std::shared_ptr<RouteHandler> & route_handler_ptr);

  // override virtual functions
  void entry() override;
  void update() override;
  State getNextState() const override;
  State getCurrentState() const override;
  autoware_planning_msgs::PathWithLaneId getPath() const override;
};
}

#endif  // MULTIPOLICY_DECISION_MAKING_H
