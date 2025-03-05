#ifndef _EXPLORATION_FSM_H_
#define _EXPLORATION_FSM_H_

#include <algorithm>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <glog/logging.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <visualization_msgs/Marker.h>


//&&&&&&&&&&&&&&&&&&&&
#include <exploration_manager/DroneState.h>
#include <exploration_manager/PairOpt.h>
#include <exploration_manager/PairOptResponse.h>







using Eigen::Vector3d;
using std::shared_ptr;
using std::string;
using std::unique_ptr;
using std::vector;

namespace fast_planner {
class FastPlannerManager;
class ExplorationManager;
class PlanningVisualization;
struct FSMParam;
struct FSMData;

enum EXPL_STATE { INIT, WAIT_TRIGGER, PLAN_TRAJ, PUB_TRAJ, EXEC_TRAJ, FINISH, RTB};

class ExplorationFSM {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
  ExplorationFSM(/* args */) {}
  ~ExplorationFSM() {}

  void init(ros::NodeHandle &nh);

private:
  /* planning utils */
  shared_ptr<FastPlannerManager> planner_manager_;
  shared_ptr<ExplorationManager> expl_manager_;
  shared_ptr<PlanningVisualization> visualization_;

  shared_ptr<FSMParam> fp_;
  shared_ptr<FSMData> fd_;
  EXPL_STATE state_;

  /* ROS utils */
  ros::NodeHandle node_;
  ros::Timer exec_timer_, safety_timer_, vis_timer_, frontier_timer_;
  ros::Subscriber trigger_sub_, odom_sub_;
  ros::Publisher replan_pub_, bspline_pub_, grid_tour_pub_, uncertainty_pub_;
  ///&&&&&&&&&&&&&&&&&&&&&
  ros::Publisher drone_state_pub_,opt_pub_,opt_res_pub_;
  ros::Subscriber drone_state_sub_,opt_sub_,opt_res_sub_;
  void droneStateTimerCallback(const ros::TimerEvent& e);
  void droneStateMsgCallback(const exploration_manager::DroneStateConstPtr& msg);
  void optTimerCallback(const ros::TimerEvent& e); 
  void optMsgCallback(const exploration_manager::PairOptConstPtr& msg);
  void optResMsgCallback(const exploration_manager::PairOptResponseConstPtr& msg);
  int getId();
  ros::Timer drone_state_timer_, opt_timer_, swarm_traj_timer_;
  void findUnallocated(const vector<int>& actives, vector<int>& missed);
 
  vector<double> pre_cost;
  bool frontier_ready_;

  ros::Time trajectory_start_time_;

  /* helper functions */
  int callExplorationPlanner();
  void transitState(EXPL_STATE new_state, string pos_call);

  /* ROS functions */
  void FSMCallback(const ros::TimerEvent &e);
  void safetyCallback(const ros::TimerEvent &e);
  void frontierCallback(const ros::TimerEvent &e);
  void triggerCallback(const geometry_msgs::PoseStampedPtr &msg);
  void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
  void visualize();
  void clearVisMarker();
  
};

} // namespace fast_planner

#endif