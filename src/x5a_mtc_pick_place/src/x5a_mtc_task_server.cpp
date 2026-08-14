#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <fstream>
#include <functional>
#include <future>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/planner_interface.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/predicate_filter.h>
#include <moveit/task_constructor/task.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_task_constructor_msgs/action/execute_task_solution.hpp>
#include <moveit_task_constructor_msgs/msg/solution.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <x5a_task_interfaces/action/pick_place.hpp>

namespace mtc = moveit::task_constructor;
using namespace std::chrono_literals;

namespace
{
using SteadyClock = std::chrono::steady_clock;

struct TimedPose
{
  geometry_msgs::msg::PoseStamped pose;
  SteadyClock::time_point received{};
  bool valid{ false };
};

struct TimedStable
{
  bool value{ false };
  SteadyClock::time_point received{};
  bool valid{ false };
};

struct FrozenInput
{
  geometry_msgs::msg::PoseStamped cube;
  geometry_msgs::msg::PoseStamped box;
};

struct CandidateCounters
{
  std::atomic_size_t current_states{ 0 };
  std::atomic_size_t grasp_ik{ 0 };
  std::atomic_size_t place_ik{ 0 };
  std::atomic_size_t pick_complete{ 0 };
};

// Per-target metrics for the joint-limit A/B benchmark (plan-only).
// Plain fields are safe: task->plan() runs synchronously in one thread.
struct TargetMetrics
{
  std::size_t id{ 0 };
  double cube_x{ 0.0 };
  double cube_y{ 0.0 };
  double cube_z{ 0.0 };
  double box_x{ 0.0 };
  double box_y{ 0.0 };
  std::size_t grasp_ik{ 0 };
  std::size_t place_ik{ 0 };
  std::size_t connect1{ 0 };
  std::size_t connect2{ 0 };
  std::size_t approach_success{ 0 };
  std::size_t lift_success{ 0 };
  double approach_fraction{ 0.0 };
  double lift_fraction{ 0.0 };
  std::size_t pick_complete{ 0 };
  std::size_t complete{ 0 };
  double planning_ms{ 0.0 };
  bool j4_valid{ false };
  double j4_min{ 0.0 };
  double j4_max{ 0.0 };
  std::size_t solutions_using_j4_below_old_min{ 0 };
};

geometry_msgs::msg::Quaternion quaternionFromRpy(const std::vector<double>& rpy)
{
  tf2::Quaternion q;
  q.setRPY(rpy.at(0), rpy.at(1), rpy.at(2));
  return tf2::toMsg(q);
}

geometry_msgs::msg::PoseStamped makePose(
  const std::string& frame, double x, double y, double z,
  const geometry_msgs::msg::Quaternion& orientation)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = z;
  pose.pose.orientation = orientation;
  return pose;
}

moveit_msgs::msg::CollisionObject makeBox(
  const std::string& id, const std::string& frame,
  double x, double y, double z, double sx, double sy, double sz)
{
  moveit_msgs::msg::CollisionObject object;
  object.id = id;
  object.header.frame_id = frame;
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions = { sx, sy, sz };
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.w = 1.0;
  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(pose);
  return object;
}

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

double wrapPi(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

bool nearlySameRpy(const std::vector<double>& a, const std::vector<double>& b)
{
  if (a.size() < 3 || b.size() < 3) {
    return false;
  }
  return std::abs(wrapPi(a[0] - b[0])) < 0.05 &&
         std::abs(wrapPi(a[1] - b[1])) < 0.05 &&
         std::abs(wrapPi(a[2] - b[2])) < 0.05;
}

void appendUniqueRpy(
  std::vector<std::vector<double>>& out, const std::vector<double>& rpy, std::size_t cap)
{
  if (rpy.size() < 3 || out.size() >= cap) {
    return;
  }
  const std::vector<double> wrapped = { wrapPi(rpy[0]), wrapPi(rpy[1]), wrapPi(rpy[2]) };
  for (const auto& existing : out) {
    if (nearlySameRpy(existing, wrapped)) {
      return;
    }
  }
  out.push_back(wrapped);
}

Eigen::Isometry3d eigenFromXyzRpy(double x, double y, double z, const std::vector<double>& rpy)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(x, y, z);
  pose.linear() =
    (Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
     Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
     Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX()))
      .toRotationMatrix();
  return pose;
}

double milliseconds(SteadyClock::time_point from, SteadyClock::time_point to)
{
  return std::chrono::duration<double, std::milli>(to - from).count();
}

int64_t steadyNanoseconds(SteadyClock::time_point value)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(value.time_since_epoch()).count();
}

double seconds(const builtin_interfaces::msg::Duration& value)
{
  return static_cast<double>(value.sec) + 1e-9 * static_cast<double>(value.nanosec);
}

std::string vectorString(const std::vector<double>& values)
{
  std::ostringstream stream;
  stream << "[";
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i > 0) stream << ",";
    stream << std::fixed << std::setprecision(4) << values[i];
  }
  stream << "]";
  return stream.str();
}

std::string vectorString(const std::vector<std::string>& values)
{
  std::ostringstream stream;
  stream << "[";
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i > 0) stream << ",";
    stream << values[i];
  }
  stream << "]";
  return stream.str();
}

// The X5A has one gripper DOF mirrored into joint7/joint8. Any trajectory
// that carries a finger joint runs on the gripper controller; everything
// else runs on the arm controller.
std::string controllerForJoints(const std::vector<std::string>& joints)
{
  for (const auto& name : joints) {
    if (name == "joint7" || name == "joint8") {
      return "x5a_gripper_controller";
    }
  }
  return "x5a_arm_controller";
}

std::string moveItErrorName(int32_t code)
{
  using Code = moveit_msgs::msg::MoveItErrorCodes;
  switch (code) {
    case Code::SUCCESS: return "SUCCESS";
    case Code::FAILURE: return "FAILURE";
    case Code::PLANNING_FAILED: return "PLANNING_FAILED";
    case Code::INVALID_MOTION_PLAN: return "INVALID_MOTION_PLAN";
    case Code::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
      return "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE";
    case Code::CONTROL_FAILED: return "CONTROL_FAILED";
    case Code::UNABLE_TO_AQUIRE_SENSOR_DATA: return "UNABLE_TO_AQUIRE_SENSOR_DATA";
    case Code::TIMED_OUT: return "TIMED_OUT";
    case Code::PREEMPTED: return "PREEMPTED";
    case Code::START_STATE_IN_COLLISION: return "START_STATE_IN_COLLISION";
    case Code::START_STATE_VIOLATES_PATH_CONSTRAINTS: return "START_STATE_VIOLATES_PATH_CONSTRAINTS";
    case Code::GOAL_IN_COLLISION: return "GOAL_IN_COLLISION";
    case Code::GOAL_VIOLATES_PATH_CONSTRAINTS: return "GOAL_VIOLATES_PATH_CONSTRAINTS";
    case Code::GOAL_CONSTRAINTS_VIOLATED: return "GOAL_CONSTRAINTS_VIOLATED";
    case Code::INVALID_GROUP_NAME: return "INVALID_GROUP_NAME";
    case Code::INVALID_GOAL_CONSTRAINTS: return "INVALID_GOAL_CONSTRAINTS";
    case Code::INVALID_ROBOT_STATE: return "INVALID_ROBOT_STATE";
    case Code::INVALID_LINK_NAME: return "INVALID_LINK_NAME";
    case Code::INVALID_OBJECT_NAME: return "INVALID_OBJECT_NAME";
    case Code::FRAME_TRANSFORM_FAILURE: return "FRAME_TRANSFORM_FAILURE";
    case Code::COLLISION_CHECKING_UNAVAILABLE: return "COLLISION_CHECKING_UNAVAILABLE";
    case Code::ROBOT_STATE_STALE: return "ROBOT_STATE_STALE";
    case Code::SENSOR_INFO_STALE: return "SENSOR_INFO_STALE";
    case Code::COMMUNICATION_FAILURE: return "COMMUNICATION_FAILURE";
    case Code::CRASH: return "CRASH";
    case Code::ABORT: return "ABORT";
    case Code::NO_IK_SOLUTION: return "NO_IK_SOLUTION";
    default: return "UNKNOWN";
  }
}
}  // namespace

class X5aMtcTaskServer : public rclcpp::Node
{
public:
  using PickPlace = x5a_task_interfaces::action::PickPlace;
  using GoalHandle = rclcpp_action::ServerGoalHandle<PickPlace>;
  using ExecuteTaskSolution = moveit_task_constructor_msgs::action::ExecuteTaskSolution;

  explicit X5aMtcTaskServer(const rclcpp::NodeOptions& options)
  : Node("x5a_mtc_task_server", options)
  {
    loadParameters();
    createSubscriptions();
    arm_client_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
      this, arm_action_name_);
    gripper_client_ = rclcpp_action::create_client<control_msgs::action::GripperCommand>(
      this, gripper_action_name_);
    execute_client_ =
      rclcpp_action::create_client<moveit_task_constructor_msgs::action::ExecuteTaskSolution>(
      this, execute_task_action_name_);
  }

  bool initialize()
  {
    robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(
      shared_from_this(), "robot_description");
    robot_model_ = robot_model_loader_->getModel();
    if (!robot_model_) {
      RCLCPP_ERROR(get_logger(), "failed to load robot_description");
      return false;
    }
    if (!robot_model_->hasJointModelGroup(arm_group_) ||
        !robot_model_->hasJointModelGroup(gripper_group_)) {
      RCLCPP_ERROR(get_logger(), "required MoveIt groups arm/gripper are unavailable");
      return false;
    }

    // Effective joint bounds come from joint_limits.yaml (via
    // robot_description_planning) and override the URDF values.
    if (const auto* joint4_model = robot_model_->getJointModel("joint4")) {
      const auto& bounds = joint4_model->getVariableBounds();
      RCLCPP_INFO(
        get_logger(), "[JOINT_LIMIT] joint4 min=%.3f max=%.3f",
        bounds[0].min_position_, bounds[0].max_position_);
    } else {
      RCLCPP_ERROR(get_logger(), "[JOINT_LIMIT] joint4 model not found");
    }

    transit_planner_ = std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this());
    transit_planner_->setPlannerId("RRTConnect");
    transit_planner_->setTimeout(connect_timeout_);
    transit_planner_->setMaxVelocityScalingFactor(transit_velocity_);
    transit_planner_->setMaxAccelerationScalingFactor(transit_acceleration_);

    precision_cartesian_ = std::make_shared<mtc::solvers::CartesianPath>();
    precision_cartesian_->setStepSize(cartesian_step_);
    precision_cartesian_->setJumpThreshold(jump_threshold_);
    precision_cartesian_->setMinFraction(min_cartesian_fraction_);
    precision_cartesian_->setMaxVelocityScalingFactor(precision_velocity_);
    precision_cartesian_->setMaxAccelerationScalingFactor(precision_acceleration_);

    lift_cartesian_ = std::make_shared<mtc::solvers::CartesianPath>();
    lift_cartesian_->setStepSize(cartesian_step_);
    lift_cartesian_->setJumpThreshold(jump_threshold_);
    lift_cartesian_->setMinFraction(min_cartesian_fraction_);
    lift_cartesian_->setMaxVelocityScalingFactor(lift_velocity_);
    lift_cartesian_->setMaxAccelerationScalingFactor(lift_acceleration_);

    // Approach/lift/retreat use MoveRelative min_distance as the success
    // gate, so the Cartesian solver must return partial paths.
    flexible_cartesian_ = std::make_shared<mtc::solvers::CartesianPath>();
    flexible_cartesian_->setStepSize(cartesian_step_);
    flexible_cartesian_->setJumpThreshold(jump_threshold_);
    flexible_cartesian_->setMinFraction(0.0);
    flexible_cartesian_->setMaxVelocityScalingFactor(lift_velocity_);
    flexible_cartesian_->setMaxAccelerationScalingFactor(lift_acceleration_);

    gripper_planner_ = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

    action_server_ = rclcpp_action::create_server<PickPlace>(
      this, action_name_,
      std::bind(&X5aMtcTaskServer::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&X5aMtcTaskServer::handleCancel, this, std::placeholders::_1),
      std::bind(&X5aMtcTaskServer::handleAccepted, this, std::placeholders::_1));

    ready_timer_ = create_wall_timer(100ms, [this]() {
      if (!ready_announced_ && interfacesReady() && joint_states_valid_) {
        ready_announced_ = true;
        RCLCPP_INFO(get_logger(), "X5A TASK SERVER: READY");
      }
    });
    RCLCPP_INFO(
      get_logger(),
      "persistent MTC server started; max_ik=%d max_complete=%d task_mode=%s "
      "seed_grasp_oris=%zu seed_place_oris=%zu max_grasp_oris=%d max_place_oris=%d",
      max_ik_solutions_, max_complete_solutions_, task_mode_.c_str(),
      grasp_orientations_.size(), place_orientations_.size(),
      max_grasp_orientations_, max_place_orientations_);
    return true;
  }

private:
  template <typename T>
  T parameter(const std::string& name, const T& default_value)
  {
    if (!has_parameter(name)) {
      declare_parameter<T>(name, default_value);
    }
    T value = default_value;
    get_parameter(name, value);
    return value;
  }

  void loadParameters()
  {
    arm_group_ = parameter<std::string>("planning.group", "arm");
    gripper_group_ = parameter<std::string>("planning.gripper_group", "gripper");
    eef_name_ = parameter<std::string>("planning.eef", "x5a_gripper");
    base_frame_ = parameter<std::string>("planning.base_frame", "base_link");
    tcp_frame_ = parameter<std::string>("planning.tcp_frame", "tool0");
    transit_velocity_ = parameter<double>("planning.transit_velocity_scaling", 0.45);
    transit_acceleration_ = parameter<double>("planning.transit_acceleration_scaling", 0.18);
    precision_velocity_ = parameter<double>("planning.precision_velocity_scaling", 0.15);
    precision_acceleration_ = parameter<double>("planning.precision_acceleration_scaling", 0.08);
    lift_velocity_ = parameter<double>("planning.lift_retreat_velocity_scaling", 0.20);
    lift_acceleration_ = parameter<double>("planning.lift_retreat_acceleration_scaling", 0.10);
    grasp_rpy_ = parameter<std::vector<double>>(
      "grasp_orientation_rpy", { 0.0, 1.45, 0.0 });
    transit_rpy_ = parameter<std::vector<double>>(
      "transit_orientation_rpy", { 0.0, 1.15, 0.0 });
    // Flat RPY triplets; e.g. [0,1.45,0, 0,1.45,0.35, ...]. Each triplet is
    // one grasp-orientation candidate. Empty -> the single grasp_rpy above.
    const std::vector<double> flat_candidates = parameter<std::vector<double>>(
      "grasp_orientation_candidates", {});
    grasp_orientations_.clear();
    for (std::size_t i = 0; i + 2 < flat_candidates.size(); i += 3) {
      grasp_orientations_.push_back(
        { flat_candidates[i], flat_candidates[i + 1], flat_candidates[i + 2] });
    }
    if (grasp_orientations_.empty()) {
      grasp_orientations_.push_back(grasp_rpy_);
    }
    // Flat RPY triplets for the place (PRE_PLACE) orientation candidates.
    const std::vector<double> flat_place = parameter<std::vector<double>>(
      "place_orientation_candidates", {});
    place_orientations_.clear();
    for (std::size_t i = 0; i + 2 < flat_place.size(); i += 3) {
      place_orientations_.push_back(
        { flat_place[i], flat_place[i + 1], flat_place[i + 2] });
    }
    if (place_orientations_.empty()) {
      place_orientations_.push_back(transit_rpy_);
    }

    max_pose_age_ = parameter<double>("vision.max_pose_age", 0.5);
    object_size_x_ = parameter<double>("object.size_x", 0.03);
    object_size_y_ = parameter<double>("object.size_y", 0.03);
    object_size_z_ = parameter<double>("object.size_z", 0.03);
    place_z_ = parameter<double>("place.z", 0.144);
    table_x_ = parameter<double>("table.x", 0.22);
    table_y_ = parameter<double>("table.y", 0.0);
    table_z_ = parameter<double>("table.z", -0.026);
    table_size_x_ = parameter<double>("table.size_x", 0.70);
    table_size_y_ = parameter<double>("table.size_y", 0.80);
    table_size_z_ = parameter<double>("table.size_z", 0.04);
    pre_grasp_height_ = parameter<double>("motion.pre_grasp_height", 0.08);
    lift_height_ = parameter<double>("motion.lift_height", 0.08);
    pre_place_height_ = parameter<double>("motion.pre_place_height", 0.08);
    retreat_height_ = parameter<double>("motion.retreat_height", 0.08);
    cartesian_step_ = parameter<double>("motion.cartesian_step", 0.01);
    jump_threshold_ = parameter<double>("motion.jump_threshold", 0.0);
    min_cartesian_fraction_ = parameter<double>("motion.min_cartesian_fraction", 0.95);
    grasp_offset_x_ = parameter<double>("grasp.x_offset", 0.0);
    grasp_offset_y_ = parameter<double>("grasp.y_offset", 0.0);
    grasp_offset_z_ = parameter<double>("grasp.z_offset", 0.040);
    ready_joints_ = parameter<std::vector<double>>(
      "ready_joints", { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });

    max_ik_solutions_ = parameter<int>("mtc.max_ik_solutions", 8);
    ik_timeout_ = parameter<double>("mtc.ik_timeout", 0.5);
    max_complete_solutions_ = parameter<int>("mtc.max_complete_solutions", 3);
    connect_timeout_ = parameter<double>("mtc.connect_timeout", 5.0);
    plan_timeout_ = parameter<double>("mtc.plan_timeout", 60.0);
    place_candidate_offset_ = parameter<double>("mtc.place_candidate_offset", 0.01);
    place_candidate_yaw_ = parameter<double>("mtc.place_candidate_yaw", 0.20);
    approach_candidate_count_ = parameter<int>("mtc.approach_candidate_count", 4);
    approach_pitch_step_ = parameter<double>("mtc.approach_pitch_step", 0.10);
    max_grasp_orientations_ = parameter<int>("mtc.max_grasp_orientations", 12);
    max_place_orientations_ = parameter<int>("mtc.max_place_orientations", 6);
    cartesian_min_height_scale_ = parameter<double>("mtc.cartesian_min_height_scale", 0.50);
    reachability_step_ = parameter<double>("mtc.reachability_step", 0.05);
    reachability_z_ = parameter<double>("mtc.reachability_z", 0.020);
    joint_state_max_age_ = parameter<double>("mtc.joint_state_max_age", 0.5);
    current_state_timeout_ = parameter<double>("mtc.current_state_timeout", 2.0);
    plan_only_ = parameter<bool>("mtc.plan_only", false);
    task_mode_ = parameter<std::string>("mtc.task_mode", "full");
    execute_timeout_ = parameter<double>("mtc.execute_timeout", 300.0);
    benchmark_targets_file_ = parameter<std::string>("mtc.benchmark_targets_file", "");
    benchmark_out_dir_ = parameter<std::string>("mtc.benchmark_out_dir", "/home/quella/arx/arm/logs");
    if (task_mode_ != "full" && task_mode_ != "pre_grasp" && task_mode_ != "benchmark" &&
        task_mode_ != "pick" && task_mode_ != "reachability") {
      RCLCPP_WARN(get_logger(), "unknown mtc.task_mode '%s'; using 'full'", task_mode_.c_str());
      task_mode_ = "full";
    }
    workspace_x_min_ = parameter<double>("workspace.x_min", 0.08);
    workspace_x_max_ = parameter<double>("workspace.x_max", 0.41);
    workspace_y_min_ = parameter<double>("workspace.y_min", 0.08);
    workspace_y_max_ = parameter<double>("workspace.y_max", 0.48);
    workspace_r_max_ = parameter<double>("workspace.r_max", 0.54);

    action_name_ = parameter<std::string>("action_name", "/x5a_mtc_task_server");
    arm_action_name_ = parameter<std::string>(
      "arm_action_name", "/x5a_arm_controller/follow_joint_trajectory");
    gripper_action_name_ = parameter<std::string>(
      "gripper.action_name", "/x5a_gripper_controller/gripper_cmd");
    execute_task_action_name_ = parameter<std::string>(
      "execute_task_action_name", "/execute_task_solution");
    joint_state_topic_ = parameter<std::string>("state.joint_state_topic", "/joint_states");
    box_pose_topic_ = parameter<std::string>("box.pose_topic", "/x5a_vision/box_pose");
    box_stable_topic_ = parameter<std::string>("box.stable_topic", "/x5a_vision/box_stable");
    for (const std::string color : { "red", "white", "orange" }) {
      cube_pose_topics_[color] = parameter<std::string>(
        "cube_topics." + color + "_pose", "/x5a_vision/" + color + "_cube_pose");
      cube_stable_topics_[color] = parameter<std::string>(
        "cube_topics." + color + "_stable", "/x5a_vision/" + color + "_cube_stable");
    }
  }

  void createSubscriptions()
  {
    for (const std::string color : { "red", "white", "orange" }) {
      cube_pose_subscriptions_.push_back(create_subscription<geometry_msgs::msg::PoseStamped>(
        cube_pose_topics_.at(color), 10,
        [this, color](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(data_mutex_);
          cube_poses_[color] = { *msg, SteadyClock::now(), msg->header.frame_id == base_frame_ };
        }));
      cube_stable_subscriptions_.push_back(create_subscription<std_msgs::msg::Bool>(
        cube_stable_topics_.at(color), 10,
        [this, color](std_msgs::msg::Bool::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(data_mutex_);
          cube_stable_[color] = { msg->data, SteadyClock::now(), true };
        }));
    }
    box_pose_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      box_pose_topic_, 10, [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        box_pose_ = { *msg, SteadyClock::now(), msg->header.frame_id == base_frame_ };
      });
    box_stable_subscription_ = create_subscription<std_msgs::msg::Bool>(
      box_stable_topic_, 10, [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        box_stable_ = { msg->data, SteadyClock::now(), true };
      });
    joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_, 20, [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
        if (msg->name.size() < 6 || msg->position.size() < 6) {
          return;
        }
        std::lock_guard<std::mutex> lock(data_mutex_);
        joint_states_valid_ = true;
        joint_state_received_ = SteadyClock::now();
        last_joint_positions_.clear();
        for (const std::string name : { "joint1", "joint2", "joint3", "joint4", "joint5", "joint6" }) {
          const auto it = std::find(msg->name.begin(), msg->name.end(), name);
          if (it != msg->name.end()) {
            const std::size_t index = static_cast<std::size_t>(it - msg->name.begin());
            if (index < msg->position.size()) {
              last_joint_positions_.push_back(msg->position[index]);
            }
          }
        }
      });
  }

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID&, std::shared_ptr<const PickPlace::Goal> goal)
  {
    if (cube_pose_topics_.count(goal->color) == 0) {
      RCLCPP_WARN(get_logger(), "rejecting unsupported color '%s'", goal->color.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    bool expected = false;
    if (!busy_.compare_exchange_strong(expected, true)) {
      RCLCPP_WARN(get_logger(), "rejecting '%s': another task is active", goal->color.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    command_received_ = SteadyClock::now();
    RCLCPP_INFO(
      get_logger(), "COMMAND ACCEPTED: %s T_command_received=%ld steady_ns",
      goal->color.c_str(), steadyNanoseconds(command_received_));
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle>)
  {
    std::lock_guard<std::mutex> lock(active_task_mutex_);
    if (active_task_ != nullptr) {
      active_task_->preempt();
    }
    if (active_exec_goal_) {
      RCLCPP_WARN(get_logger(), "canceling active ExecuteTaskSolution goal");
      execute_client_->async_cancel_goal(active_exec_goal_);
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    std::thread([this, goal_handle]() { executeGoal(goal_handle); }).detach();
  }

  bool interfacesReady() const
  {
    return arm_client_->action_server_is_ready() &&
           gripper_client_->action_server_is_ready() &&
           execute_client_->action_server_is_ready();
  }

  bool freezeInput(const std::string& color, FrozenInput& frozen, std::string& error,
                   bool require_box = true)
  {
    const auto now = SteadyClock::now();
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!joint_states_valid_ || milliseconds(joint_state_received_, now) > joint_state_max_age_ * 1000.0) {
      error = "ROBOT_NOT_READY: joint_states stale";
      return false;
    }
    if (!interfacesReady()) {
      error = "ROBOT_NOT_READY: arm/gripper/MTC execution interface offline";
      return false;
    }
    const auto& stable = cube_stable_.at(color);
    const auto& pose = cube_poses_.at(color);
    if (!stable.valid || !stable.value) {
      error = "ROBOT_NOT_READY: selected cube is not stable";
      return false;
    }
    if (!pose.valid || milliseconds(pose.received, now) > max_pose_age_ * 1000.0) {
      error = "ROBOT_NOT_READY: selected cube pose stale or wrong frame";
      return false;
    }
    if (require_box) {
      if (!box_stable_.valid || !box_stable_.value) {
        error = "ROBOT_NOT_READY: box is not stable";
        return false;
      }
      if (!box_pose_.valid || milliseconds(box_pose_.received, now) > max_pose_age_ * 1000.0) {
        error = "ROBOT_NOT_READY: box pose stale or wrong frame";
        return false;
      }
    }
    frozen.cube = pose.pose;
    frozen.box = box_pose_.pose;
    if (!insideWorkspace(frozen.cube.pose.position.x, frozen.cube.pose.position.y)) {
      error = "TARGET_UNREACHABLE: cube is outside the grasp envelope";
      return false;
    }
    if (require_box && !insideWorkspace(frozen.box.pose.position.x, frozen.box.pose.position.y)) {
      error = "TARGET_UNREACHABLE: box is outside the grasp envelope";
      return false;
    }
    return true;
  }

  bool insideWorkspace(double x, double y) const
  {
    if (x < workspace_x_min_ || x > workspace_x_max_ ||
        y < workspace_y_min_ || y > workspace_y_max_) {
      return false;
    }
    return std::hypot(x, y) <= workspace_r_max_;
  }

  void publishStage(const std::shared_ptr<GoalHandle>& goal_handle, const std::string& stage)
  {
    auto feedback = std::make_shared<PickPlace::Feedback>();
    feedback->stage = stage;
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(get_logger(), "TASK_STAGE: %s", stage.c_str());
  }

  void finishGoal(
    const std::shared_ptr<GoalHandle>& goal_handle, bool success, const std::string& message)
  {
    auto result = std::make_shared<PickPlace::Result>();
    result->success = success;
    result->message = message;
    if (success) {
      goal_handle->succeed(result);
    } else if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
    } else {
      goal_handle->abort(result);
    }
    busy_ = false;
  }

  void logSolutionTrajectories(const mtc::SolutionBase& solution)
  {
    moveit_task_constructor_msgs::msg::Solution message;
    solution.toMsg(message);
    for (std::size_t index = 0; index < message.sub_trajectory.size(); ++index) {
      const auto& trajectory = message.sub_trajectory[index].trajectory.joint_trajectory;
      if (trajectory.points.empty()) {
        continue;
      }
      double max_speed = 0.0;
      for (std::size_t point = 1; point < trajectory.points.size(); ++point) {
        const double t0 = seconds(trajectory.points[point - 1].time_from_start);
        const double t1 = seconds(trajectory.points[point].time_from_start);
        if (t1 <= t0) {
          continue;
        }
        const auto& q0 = trajectory.points[point - 1].positions;
        const auto& q1 = trajectory.points[point].positions;
        for (std::size_t joint = 0; joint < q0.size() && joint < q1.size(); ++joint) {
          max_speed = std::max(max_speed, std::abs(q1[joint] - q0[joint]) / (t1 - t0));
        }
      }
      RCLCPP_INFO(
        get_logger(),
        "solution trajectory %zu joints=%s points=%zu duration=%.3f s "
        "max_dq_dt=%.4f rad/s q_start=%s q_goal=%s",
        index, vectorString(trajectory.joint_names).c_str(), trajectory.points.size(),
        seconds(trajectory.points.back().time_from_start), max_speed,
        vectorString(trajectory.points.front().positions).c_str(),
        vectorString(trajectory.points.back().positions).c_str());
    }
  }

  std::vector<std::pair<double, double>> placeCandidates(double x, double y) const
  {
    const std::vector<std::pair<double, double>> offsets = {
      { 0.0, 0.0 }, { place_candidate_offset_, 0.0 },
      { -place_candidate_offset_, 0.0 }, { 0.0, place_candidate_offset_ },
      { 0.0, -place_candidate_offset_ }
    };
    std::vector<std::pair<double, double>> candidates;
    for (const auto& offset : offsets) {
      const double px = x + offset.first;
      const double py = y + offset.second;
      if (insideWorkspace(px, py)) {
        candidates.emplace_back(px, py);
      }
    }
    return candidates;
  }

  std::vector<std::vector<double>> sampleGraspOrientations(double x, double y) const
  {
    const std::size_t cap = static_cast<std::size_t>(std::max(1, max_grasp_orientations_));
    std::vector<std::vector<double>> out;
    for (const auto& seed : grasp_orientations_) {
      appendUniqueRpy(out, seed, cap);
    }
    const double yaw0 = std::atan2(y, x);
    const double yaw_step = place_candidate_yaw_ > 1e-3 ? place_candidate_yaw_ : 0.20;
    const double base_pitch = grasp_rpy_.size() > 1 ? grasp_rpy_[1] : 1.45;
    std::vector<double> pitches;
    pitches.push_back(base_pitch);
    for (int i = 1; i <= std::max(0, approach_candidate_count_); ++i) {
      pitches.push_back(base_pitch - static_cast<double>(i) * approach_pitch_step_);
    }
    for (std::size_t i = 0; i < pitches.size(); ++i) {
      if (pitches[i] < 0.50) {
        continue;
      }
      appendUniqueRpy(out, { 0.0, pitches[i], yaw0 }, cap);
      if (i <= 2) {
        appendUniqueRpy(out, { 0.0, pitches[i], yaw0 + yaw_step }, cap);
        appendUniqueRpy(out, { 0.0, pitches[i], yaw0 - yaw_step }, cap);
      }
    }
    return out;
  }

  std::vector<std::vector<double>> samplePlaceOrientations(double x, double y) const
  {
    const std::size_t cap = static_cast<std::size_t>(std::max(1, max_place_orientations_));
    std::vector<std::vector<double>> out;
    const double yaw0 = std::atan2(y, x);
    const double yaw_step = place_candidate_yaw_ > 1e-3 ? place_candidate_yaw_ : 0.20;
    const double r = std::hypot(x, y);
    // Far / high-Y boxes need a steeper TCP. The old shallow pitches
    // (0.85-1.15) stretch the wrist into the table at PRE_PLACE z=0.224.
    const std::vector<double> pitches = r >= 0.42 ?
      std::vector<double>{ 1.45, 1.30, 1.15, 1.00 } :
      std::vector<double>{ 1.15, 1.30, 1.00, 0.85 };
    for (double pitch : pitches) {
      appendUniqueRpy(out, { 0.0, pitch, yaw0 }, cap);
    }
    for (const auto& seed : place_orientations_) {
      appendUniqueRpy(out, seed, cap);
    }
    if (!pitches.empty()) {
      appendUniqueRpy(out, { 0.0, pitches[0], yaw0 + yaw_step }, cap);
      appendUniqueRpy(out, { 0.0, pitches[0], yaw0 - yaw_step }, cap);
    }
    return out;
  }

  std::unique_ptr<mtc::stages::MoveRelative> makeVerticalRelative(
    const std::string& name, const mtc::solvers::PlannerInterfacePtr& planner,
    double min_height, double max_height, double world_z)
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>(name, planner);
    stage->setGroup(arm_group_);
    stage->setIKFrame(tcp_frame_);
    stage->setMinMaxDistance(min_height, max_height);
    geometry_msgs::msg::Vector3Stamped direction;
    direction.header.frame_id = base_frame_;
    direction.vector.z = world_z;
    stage->setDirection(direction);
    return stage;
  }

  std::unique_ptr<mtc::Task> createTask(
    const FrozenInput& frozen, const std::shared_ptr<CandidateCounters>& counters,
    TargetMetrics* metrics = nullptr, bool pick_only = false)
  {
    auto task = std::make_unique<mtc::Task>();
    task->setName("x5a MTC pick place");
    task->setRobotModel(robot_model_);
    task->setProperty("group", arm_group_);
    task->setProperty("eef", eef_name_);
    task->setProperty("hand", gripper_group_);
    task->setProperty("ik_frame", tcp_frame_);
    // Overall planning budget. Must cover CurrentState + both Connect stages
    // + the IK propagation across all orientation/XY candidates.
    task->setTimeout(plan_timeout_);

    const double top_x = frozen.cube.pose.position.x;
    const double top_y = frozen.cube.pose.position.y;
    const double top_z = frozen.cube.pose.position.z;
    const auto grasp_oris = sampleGraspOrientations(top_x, top_y);
    const auto place_oris = pick_only ?
      std::vector<std::vector<double>>{} :
      samplePlaceOrientations(frozen.box.pose.position.x, frozen.box.pose.position.y);
    const double approach_min = std::max(0.02, cartesian_min_height_scale_ * pre_grasp_height_);
    const double lift_min = std::max(0.02, cartesian_min_height_scale_ * lift_height_);
    const double retreat_min = std::max(0.02, cartesian_min_height_scale_ * retreat_height_);
    RCLCPP_INFO(
      get_logger(),
      "[MTC] cube=(%.3f,%.3f,%.3f) box=(%.3f,%.3f) grasp_oris=%zu place_oris=%zu pick_only=%s",
      top_x, top_y, top_z, frozen.box.pose.position.x, frozen.box.pose.position.y,
      grasp_oris.size(), place_oris.size(), pick_only ? "true" : "false");

    auto current = std::make_unique<mtc::stages::CurrentState>("CurrentState");
    current->addSolutionCallback([counters](const mtc::SolutionBase& solution) {
      if (!solution.isFailure()) {
        ++counters->current_states;
      }
    });
    auto current_filter = std::make_unique<mtc::stages::PredicateFilter>(
      "object not already attached", std::move(current));
    current_filter->setPredicate([](const mtc::SolutionBase& solution, std::string& comment) {
      if (solution.start()->scene()->getCurrentState().hasAttachedBody("object")) {
        comment = "object is already attached";
        return false;
      }
      return true;
    });
    task->add(std::move(current_filter));

    auto add_scene = std::make_unique<mtc::stages::ModifyPlanningScene>("add table and cube");
    add_scene->addObject(makeBox(
      "table", base_frame_, table_x_, table_y_, table_z_,
      table_size_x_, table_size_y_, table_size_z_));
    add_scene->addObject(makeBox(
      "object", base_frame_, top_x, top_y, top_z - object_size_z_ * 0.5,
      object_size_x_, object_size_y_, object_size_z_));
    task->add(std::move(add_scene));

    {
      auto allow_expected = std::make_unique<mtc::stages::ModifyPlanningScene>(
        "allow expected grasp support collisions");
      allow_expected->allowCollisions(
        mtc::stages::ModifyPlanningScene::Names{ "object" },
        mtc::stages::ModifyPlanningScene::Names{ "link6", "link7", "link8", "tool0" }, true);
      allow_expected->allowCollisions("object", "table", true);
      task->add(std::move(allow_expected));
    }

    mtc::Stage* open_hand_state = nullptr;
    {
      auto open = std::make_unique<mtc::stages::MoveTo>("open gripper", gripper_planner_);
      open->setGroup(gripper_group_);
      open->setGoal("open");
      open_hand_state = open.get();
      task->add(std::move(open));
    }

    {
      auto connect = std::make_unique<mtc::stages::Connect>(
        "Current to PRE_GRASP",
        mtc::stages::Connect::GroupPlannerVector{ { arm_group_, transit_planner_ } });
      connect->setTimeout(connect_timeout_);
      if (metrics != nullptr) {
        connect->addSolutionCallback([metrics](const mtc::SolutionBase& solution) {
          if (!solution.isFailure()) {
            ++metrics->connect1;
          }
        });
      }
      task->add(std::move(connect));
    }

    mtc::Stage* pick_stage = nullptr;
    {
      auto pick = std::make_unique<mtc::SerialContainer>("Pick");

      auto grasp_candidates = std::make_unique<mtc::Alternatives>(
        "MTC grasp orientation candidates");
      for (std::size_t index = 0; index < grasp_oris.size(); ++index) {
        const std::vector<double> orientation = grasp_oris[index];
        auto candidate = std::make_unique<mtc::SerialContainer>(
          "grasp candidate " + std::to_string(index + 1));

        const auto grasp_pose = makePose(
          base_frame_, top_x + grasp_offset_x_, top_y + grasp_offset_y_,
          top_z + grasp_offset_z_, quaternionFromRpy(orientation));

        // Placed before ComputeIK, so this stage propagates backward from
        // the grasp. Direction is the forward approach (-Z); MTC inverts
        // it when planning backward, leaving pre-grasp above the cube.
        auto approach = makeVerticalRelative(
          "APPROACH candidate " + std::to_string(index + 1),
          flexible_cartesian_, approach_min, pre_grasp_height_, -1.0);
        if (metrics != nullptr) {
          const double requested = pre_grasp_height_;
          approach->addSolutionCallback(
            [this, metrics, requested](const mtc::SolutionBase& solution) {
              const double fraction = cartesianTravelFraction(solution, requested);
              if (!solution.isFailure()) {
                ++metrics->approach_success;
              }
              metrics->approach_fraction = std::max(metrics->approach_fraction, fraction);
            });
        }
        candidate->insert(std::move(approach));

        auto grasp_generator = std::make_unique<mtc::stages::GeneratePose>(
          "generate grasp pose " + std::to_string(index + 1));
        grasp_generator->setPose(grasp_pose);
        grasp_generator->setMonitoredStage(open_hand_state);
        auto grasp_ik = std::make_unique<mtc::stages::ComputeIK>(
          "grasp ComputeIK " + std::to_string(index + 1), std::move(grasp_generator));
        grasp_ik->setGroup(arm_group_);
        grasp_ik->setEndEffector(eef_name_);
        grasp_ik->setIKFrame(tcp_frame_);
        grasp_ik->setMaxIKSolutions(max_ik_solutions_);
        grasp_ik->setMinSolutionDistance(0.10);
        grasp_ik->setProperty("timeout", ik_timeout_);
        grasp_ik->properties().configureInitFrom(
          mtc::Stage::INTERFACE, { "target_pose" });
        grasp_ik->addSolutionCallback([counters](const mtc::SolutionBase& solution) {
          if (!solution.isFailure()) {
            ++counters->grasp_ik;
          }
        });
        candidate->insert(std::move(grasp_ik));

        auto allow = std::make_unique<mtc::stages::ModifyPlanningScene>(
          "allow gripper-object collision");
        allow->allowCollisions(
          mtc::stages::ModifyPlanningScene::Names{ "object" },
          mtc::stages::ModifyPlanningScene::Names{ "link6", "link7", "link8", "tool0" }, true);
        candidate->insert(std::move(allow));

        auto allow_support = std::make_unique<mtc::stages::ModifyPlanningScene>(
          "allow object-table support collision");
        allow_support->allowCollisions("object", "table", true);
        candidate->insert(std::move(allow_support));

        auto close = std::make_unique<mtc::stages::MoveTo>("close gripper", gripper_planner_);
        close->setGroup(gripper_group_);
        close->setGoal("closed");
        candidate->insert(std::move(close));

        auto attach = std::make_unique<mtc::stages::ModifyPlanningScene>("ATTACH object");
        attach->attachObject("object", tcp_frame_);
        candidate->insert(std::move(attach));

        auto lift = makeVerticalRelative(
          "LIFT candidate " + std::to_string(index + 1),
          flexible_cartesian_, lift_min, lift_height_, 1.0);
        if (metrics != nullptr) {
          const double requested = lift_height_;
          lift->addSolutionCallback(
            [this, metrics, requested](const mtc::SolutionBase& solution) {
              const double fraction = cartesianTravelFraction(solution, requested);
              if (!solution.isFailure()) {
                ++metrics->lift_success;
              }
              metrics->lift_fraction = std::max(metrics->lift_fraction, fraction);
            });
        }
        candidate->insert(std::move(lift));

        grasp_candidates->add(std::move(candidate));
      }
      pick->insert(std::move(grasp_candidates));
      pick->addSolutionCallback([counters, metrics](const mtc::SolutionBase& solution) {
        if (!solution.isFailure()) {
          ++counters->pick_complete;
          if (metrics != nullptr) {
            ++metrics->pick_complete;
          }
        }
      });

      pick_stage = pick.get();
      task->add(std::move(pick));
    }

    if (pick_only) {
      return task;
    }

    {
      auto connect = std::make_unique<mtc::stages::Connect>(
        "LIFT to PRE_PLACE",
        mtc::stages::Connect::GroupPlannerVector{ { arm_group_, transit_planner_ } });
      connect->setTimeout(connect_timeout_);
      if (metrics != nullptr) {
        connect->addSolutionCallback([metrics](const mtc::SolutionBase& solution) {
          if (!solution.isFailure()) {
            ++metrics->connect2;
          }
        });
      }
      task->add(std::move(connect));
    }

    auto alternatives = std::make_unique<mtc::Alternatives>("dynamic place candidates");
    const auto candidates = placeCandidates(
      frozen.box.pose.position.x, frozen.box.pose.position.y);
    std::size_t place_index = 0;
    for (const auto& candidate_xy : candidates) {
      // Place orientation is a candidate set too (same logic as grasp):
      // the XY offsets are combined with several pitch/yaw variants and MTC
      // keeps only chains that stay feasible end to end.
      for (std::size_t ori_index = 0; ori_index < place_oris.size(); ++ori_index) {
        ++place_index;
        auto place = std::make_unique<mtc::SerialContainer>(
          "Place candidate " + std::to_string(place_index));
        const auto candidate_orientation =
          quaternionFromRpy(place_oris[ori_index]);

        auto place_generator = std::make_unique<mtc::stages::GeneratePose>(
          "generate PRE_PLACE pose " + std::to_string(place_index));
        place_generator->setPose(makePose(
          base_frame_, candidate_xy.first, candidate_xy.second,
          place_z_ + pre_place_height_, candidate_orientation));
        place_generator->setMonitoredStage(pick_stage);
        auto place_ik = std::make_unique<mtc::stages::ComputeIK>(
          "place ComputeIK " + std::to_string(place_index), std::move(place_generator));
        place_ik->setGroup(arm_group_);
        place_ik->setEndEffector(eef_name_);
        place_ik->setIKFrame(tcp_frame_);
        place_ik->setMaxIKSolutions(max_ik_solutions_);
        place_ik->setMinSolutionDistance(0.10);
        place_ik->setProperty("timeout", ik_timeout_);
        place_ik->properties().configureInitFrom(
          mtc::Stage::INTERFACE, { "target_pose" });
        place_ik->addSolutionCallback([counters](const mtc::SolutionBase& solution) {
          if (!solution.isFailure()) {
            ++counters->place_ik;
          }
        });
        place->insert(std::move(place_ik));

        auto allow_support = std::make_unique<mtc::stages::ModifyPlanningScene>(
          "allow object-table support collision for place");
        allow_support->allowCollisions("object", "table", true);
        place->insert(std::move(allow_support));

        auto descend = std::make_unique<mtc::stages::MoveRelative>(
          "DESCEND", precision_cartesian_);
        descend->setGroup(arm_group_);
        descend->setIKFrame(tcp_frame_);
        descend->setMinMaxDistance(pre_place_height_, pre_place_height_);
        geometry_msgs::msg::Vector3Stamped down;
        down.header.frame_id = base_frame_;
        down.vector.z = -1.0;
        descend->setDirection(down);
        place->insert(std::move(descend));

        auto open = std::make_unique<mtc::stages::MoveTo>("open gripper", gripper_planner_);
        open->setGroup(gripper_group_);
        open->setGoal("open");
        place->insert(std::move(open));

        auto forbid = std::make_unique<mtc::stages::ModifyPlanningScene>(
          "forbid gripper-object collision");
        forbid->allowCollisions(
          mtc::stages::ModifyPlanningScene::Names{ "object" },
          mtc::stages::ModifyPlanningScene::Names{ "link6", "link7", "link8", "tool0" }, false);
        place->insert(std::move(forbid));

        auto detach = std::make_unique<mtc::stages::ModifyPlanningScene>("DETACH object");
        detach->detachObject("object", tcp_frame_);
        place->insert(std::move(detach));

        auto retreat = makeVerticalRelative(
          "RETREAT", flexible_cartesian_, retreat_min, retreat_height_, 1.0);
        place->insert(std::move(retreat));

        alternatives->insert(std::move(place));
      }
    }
    task->add(std::move(alternatives));

    auto home = std::make_unique<mtc::stages::MoveTo>("HOME", transit_planner_);
    home->setGroup(arm_group_);
    std::map<std::string, double> home_joints;
    for (std::size_t i = 0; i < ready_joints_.size() && i < 6; ++i) {
      home_joints["joint" + std::to_string(i + 1)] = ready_joints_[i];
    }
    home->setGoal(home_joints);
    home->restrictDirection(mtc::stages::MoveTo::FORWARD);
    task->add(std::move(home));
    return task;
  }

  // Diagnostic task for the staged protocol (step C): plan/execute ONLY the
  // first arm motion, Current -> PRE_GRASP, with the same transit planner and
  // scaling as the full task. The PRE_GRASP target is identical to the first
  // approach candidate of the full task (pitch step 0).
  std::unique_ptr<mtc::Task> createPreGraspTask(
    const FrozenInput& frozen, const std::shared_ptr<CandidateCounters>& counters)
  {
    auto task = std::make_unique<mtc::Task>();
    task->setName("x5a MTC PRE_GRASP only");
    task->setRobotModel(robot_model_);
    task->setProperty("group", arm_group_);
    task->setProperty("eef", eef_name_);
    task->setProperty("hand", gripper_group_);
    task->setProperty("ik_frame", tcp_frame_);
    task->setTimeout(plan_timeout_);

    const double top_x = frozen.cube.pose.position.x;
    const double top_y = frozen.cube.pose.position.y;
    const double top_z = frozen.cube.pose.position.z;
    const auto pre_grasp_pose = makePose(
      base_frame_, top_x + grasp_offset_x_, top_y + grasp_offset_y_,
      top_z + grasp_offset_z_ + pre_grasp_height_, quaternionFromRpy(grasp_rpy_));
    RCLCPP_INFO(
      get_logger(), "[MTC] PRE_GRASP_TARGET x=%.4f y=%.4f z=%.4f rpy=%s",
      pre_grasp_pose.pose.position.x, pre_grasp_pose.pose.position.y,
      pre_grasp_pose.pose.position.z, vectorString(grasp_rpy_).c_str());

    auto current = std::make_unique<mtc::stages::CurrentState>("CurrentState");
    current->addSolutionCallback([counters](const mtc::SolutionBase& solution) {
      if (!solution.isFailure()) {
        ++counters->current_states;
      }
    });
    task->add(std::move(current));

    auto add_scene = std::make_unique<mtc::stages::ModifyPlanningScene>("add table and cube");
    add_scene->addObject(makeBox(
      "table", base_frame_, table_x_, table_y_, table_z_,
      table_size_x_, table_size_y_, table_size_z_));
    add_scene->addObject(makeBox(
      "object", base_frame_, top_x, top_y, top_z - object_size_z_ * 0.5,
      object_size_x_, object_size_y_, object_size_z_));
    task->add(std::move(add_scene));

    auto pre_grasp = std::make_unique<mtc::stages::MoveTo>("PRE_GRASP", transit_planner_);
    pre_grasp->setGroup(arm_group_);
    pre_grasp->setIKFrame(tcp_frame_);
    pre_grasp->setGoal(pre_grasp_pose);
    task->add(std::move(pre_grasp));
    return task;
  }

  void executeGoal(const std::shared_ptr<GoalHandle>& goal_handle)
  {
    const std::string color = goal_handle->get_goal()->color;
    if (task_mode_ == "benchmark") {
      runBenchmark(goal_handle);
      return;
    }
    if (task_mode_ == "reachability") {
      runReachability(goal_handle);
      return;
    }
    const bool pre_grasp_mode = (task_mode_ == "pre_grasp");
    const bool pick_mode = (task_mode_ == "pick");
    RCLCPP_INFO(
      get_logger(), "[MTC] TASK_MODE=%s plan_only=%s", task_mode_.c_str(),
      plan_only_ ? "true" : "false");
    publishStage(goal_handle, "FAST_READINESS_CHECK");
    FrozenInput frozen;
    std::string error;
    if (!freezeInput(color, frozen, error, !pre_grasp_mode && !pick_mode)) {
      RCLCPP_ERROR(get_logger(), "%s", error.c_str());
      finishGoal(goal_handle, false, error);
      return;
    }

    const auto frozen_time = SteadyClock::now();
    RCLCPP_INFO(
      get_logger(),
      "T_pose_frozen=%ld steady_ns cube=(%.4f,%.4f,%.4f) box=(%.4f,%.4f)",
      steadyNanoseconds(frozen_time), frozen.cube.pose.position.x,
      frozen.cube.pose.position.y, frozen.cube.pose.position.z,
      frozen.box.pose.position.x, frozen.box.pose.position.y);
    publishStage(goal_handle, "POSE_FROZEN");

    auto counters = std::make_shared<CandidateCounters>();
    std::unique_ptr<mtc::Task> task;
    try {
      task = pre_grasp_mode ?
        createPreGraspTask(frozen, counters) :
        createTask(frozen, counters, nullptr, pick_mode);
    } catch (const std::exception& exception) {
      finishGoal(goal_handle, false, std::string("TASK INITIALIZATION FAILED: ") + exception.what());
      return;
    }

    {
      std::lock_guard<std::mutex> lock(active_task_mutex_);
      active_task_ = task.get();
    }
    const auto plan_start = SteadyClock::now();
    const double command_to_plan_ms = milliseconds(command_received_, plan_start);
    RCLCPP_INFO(
      get_logger(), "T_mtc_plan_start=%ld steady_ns warm_command_latency_ms=%.3f",
      steadyNanoseconds(plan_start), command_to_plan_ms);
    publishStage(goal_handle, "MTC_PLAN_START");

    moveit::core::MoveItErrorCode plan_result;
    try {
      plan_result = task->plan(static_cast<std::size_t>(max_complete_solutions_));
    } catch (const mtc::InitStageException& exception) {
      std::ostringstream details;
      details << exception;
      RCLCPP_ERROR(get_logger(), "TASK INITIALIZATION FAILED: %s", details.str().c_str());
      {
        std::lock_guard<std::mutex> lock(active_task_mutex_);
        active_task_ = nullptr;
      }
      finishGoal(goal_handle, false, "TASK INITIALIZATION FAILED");
      return;
    } catch (const std::exception& exception) {
      RCLCPP_ERROR(get_logger(), "TASK PLANNING FAILED: %s", exception.what());
      {
        std::lock_guard<std::mutex> lock(active_task_mutex_);
        active_task_ = nullptr;
      }
      finishGoal(goal_handle, false, std::string("TASK PLANNING FAILED: ") + exception.what());
      return;
    }

    const auto plan_done = SteadyClock::now();
    const std::size_t complete = task->numSolutions();
    RCLCPP_INFO(get_logger(), "T_mtc_plan_done=%ld steady_ns", steadyNanoseconds(plan_done));
    RCLCPP_INFO(get_logger(), "MTC CurrentState: %s", counters->current_states > 0 ? "PASS" : "FAIL");
    RCLCPP_INFO(get_logger(), "grasp IK candidates: %zu", counters->grasp_ik.load());
    RCLCPP_INFO(get_logger(), "pick solutions: %zu", counters->pick_complete.load());
    RCLCPP_INFO(get_logger(), "place IK candidates: %zu", counters->place_ik.load());
    RCLCPP_INFO(get_logger(), "complete task solutions: %zu", complete);

    if (!plan_result || complete == 0) {
      std::ostringstream details;
      task->explainFailure(details);
      RCLCPP_ERROR(get_logger(), "TASK PLANNING FAILED\n%s", details.str().c_str());
      {
        std::lock_guard<std::mutex> lock(active_task_mutex_);
        active_task_ = nullptr;
      }
      finishGoal(goal_handle, false, "TASK PLANNING FAILED; robot remained at CurrentState");
      return;
    }
    if (goal_handle->is_canceling()) {
      {
        std::lock_guard<std::mutex> lock(active_task_mutex_);
        active_task_ = nullptr;
      }
      finishGoal(goal_handle, false, "task canceled after planning; robot did not move");
      return;
    }

    RCLCPP_INFO(get_logger(), "FULL TASK PLANNED BEFORE MOTION: YES");
    logSolutionTrajectories(*task->solutions().front());

    // Build the execution message and log the subtrajectory -> controller
    // table once, so plan-only runs already verify the mapping that real
    // execution will use.
    moveit_task_constructor_msgs::msg::Solution solution_message;
    task->solutions().front()->toMsg(solution_message);
    for (std::size_t index = 0; index < solution_message.sub_trajectory.size(); ++index) {
      const auto& sub = solution_message.sub_trajectory[index];
      const auto& trajectory = sub.trajectory.joint_trajectory;
      const std::string controller = sub.execution_info.controller_names.empty() ?
        controllerForJoints(trajectory.joint_names) : sub.execution_info.controller_names.front();
      RCLCPP_INFO(
        get_logger(),
        "[MTC] SUBTRAJECTORY %zu/%zu stage_id=%u joints=%s points=%zu duration=%.3f "
        "controller=%s",
        index + 1, solution_message.sub_trajectory.size(), sub.info.stage_id,
        vectorString(trajectory.joint_names).c_str(), trajectory.points.size(),
        trajectory.points.empty() ? 0.0 : seconds(trajectory.points.back().time_from_start),
        controller.c_str());
    }
    publishStage(goal_handle, "FULL_TASK_PLANNED");
    if (plan_only_) {
      {
        std::lock_guard<std::mutex> lock(active_task_mutex_);
        active_task_ = nullptr;
      }
      finishGoal(goal_handle, true, "MTC TASK PLAN: PASS; execution skipped");
      return;
    }
    const auto execution_start = SteadyClock::now();
    RCLCPP_INFO(get_logger(), "T_execution_start=%ld steady_ns", steadyNanoseconds(execution_start));
    publishStage(goal_handle, "EXECUTION_START");

    // Drive the ExecuteTaskSolution action directly instead of
    // Task::execute() so every completed subtrajectory and the final error
    // code are observable. move_group's ExecuteTaskSolutionCapability
    // publishes feedback (sub_id/sub_no) after each successful
    // subtrajectory; the failing subtrajectory is therefore the first one
    // after the last received feedback.
    struct ExecutionReport
    {
      std::atomic_int last_sub_id{ -1 };
      std::atomic_int32_t error_code{ moveit_msgs::msg::MoveItErrorCodes::FAILURE };
      std::atomic_bool goal_rejected{ false };
      std::atomic_bool canceled{ false };
      std::atomic_bool done{ false };
    };
    auto report = std::make_shared<ExecutionReport>();

    auto goal_message = ExecuteTaskSolution::Goal();
    goal_message.solution = solution_message;
    rclcpp_action::Client<ExecuteTaskSolution>::SendGoalOptions options;
    options.goal_response_callback =
      [this, report](
        rclcpp_action::ClientGoalHandle<ExecuteTaskSolution>::SharedPtr handle) {
        if (!handle) {
          report->goal_rejected = true;
          report->done = true;
          RCLCPP_ERROR(get_logger(), "[MTC] EXECUTION_GOAL_REJECTED by /execute_task_solution");
          return;
        }
        RCLCPP_INFO(
          get_logger(), "[MTC] EXECUTION_GOAL_ACCEPTED t=%ld steady_ns",
          steadyNanoseconds(SteadyClock::now()));
      };
    options.feedback_callback =
      [this, report](
        rclcpp_action::ClientGoalHandle<ExecuteTaskSolution>::SharedPtr,
        const std::shared_ptr<const ExecuteTaskSolution::Feedback> feedback) {
        report->last_sub_id = static_cast<int>(feedback->sub_id);
        RCLCPP_INFO(
          get_logger(), "[MTC] EXECUTION_FEEDBACK sub_id=%u/%u t=%ld steady_ns",
          feedback->sub_id, feedback->sub_no, steadyNanoseconds(SteadyClock::now()));
      };
    options.result_callback = [this, report](
                                const rclcpp_action::ClientGoalHandle<
                                  ExecuteTaskSolution>::WrappedResult& result) {
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          report->error_code =
            result.result ? result.result->error_code.val : moveit_msgs::msg::MoveItErrorCodes::FAILURE;
          break;
        case rclcpp_action::ResultCode::CANCELED:
          report->canceled = true;
          break;
        case rclcpp_action::ResultCode::ABORTED:
          report->error_code =
            result.result ? result.result->error_code.val : moveit_msgs::msg::MoveItErrorCodes::FAILURE;
          break;
        default:
          break;
      }
      RCLCPP_INFO(
        get_logger(), "[MTC] EXECUTION_RESULT t=%ld steady_ns error_code=%d (%s) canceled=%s",
        steadyNanoseconds(SteadyClock::now()), report->error_code.load(),
        moveItErrorName(report->error_code.load()).c_str(),
        report->canceled.load() ? "true" : "false");
      report->done = true;
    };

    auto goal_handle_future = execute_client_->async_send_goal(goal_message, options);
    if (goal_handle_future.wait_for(10s) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "[MTC] EXECUTION_GOAL_NO_RESPONSE from /execute_task_solution");
      {
        std::lock_guard<std::mutex> lock(active_task_mutex_);
        active_task_ = nullptr;
      }
      finishGoal(goal_handle, false, "EXECUTION_ABORTED: no response from /execute_task_solution");
      return;
    }
    {
      auto exec_goal_handle = goal_handle_future.get();
      std::lock_guard<std::mutex> lock(active_task_mutex_);
      active_exec_goal_ = exec_goal_handle;
    }

    const auto execution_deadline =
      SteadyClock::now() + std::chrono::duration<double>(execute_timeout_);
    bool cancel_sent = false;
    while (!report->done.load() && SteadyClock::now() < execution_deadline) {
      if (goal_handle->is_canceling() && !cancel_sent) {
        cancel_sent = true;
        RCLCPP_WARN(get_logger(), "[MTC] cancel requested during execution; canceling task goal");
        execute_client_->async_cancel_goal(active_exec_goal_);
      }
      std::this_thread::sleep_for(50ms);
    }
    const auto execution_done = SteadyClock::now();
    RCLCPP_INFO(get_logger(), "T_execution_done=%ld steady_ns", steadyNanoseconds(execution_done));
    {
      std::lock_guard<std::mutex> lock(active_task_mutex_);
      active_exec_goal_.reset();
      active_task_ = nullptr;
    }
    if (!report->done.load()) {
      RCLCPP_ERROR(get_logger(), "[MTC] EXECUTION_TIMEOUT after %.1f s", execute_timeout_);
      finishGoal(goal_handle, false, "EXECUTION_ABORTED: ExecuteTaskSolution timeout");
      return;
    }
    if (report->goal_rejected.load()) {
      finishGoal(goal_handle, false, "EXECUTION_ABORTED: goal rejected by /execute_task_solution");
      return;
    }
    if (report->canceled.load() || goal_handle->is_canceling()) {
      finishGoal(goal_handle, false, "task canceled during execution");
      return;
    }
    if (report->error_code.load() != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      const int fail_index = report->last_sub_id.load() + 1;
      std::string fail_controller = "UNKNOWN";
      std::string fail_joints = "[]";
      uint32_t fail_stage_id = 0;
      if (fail_index >= 0 &&
          static_cast<std::size_t>(fail_index) < solution_message.sub_trajectory.size()) {
        const auto& sub = solution_message.sub_trajectory[fail_index];
        fail_stage_id = sub.info.stage_id;
        fail_joints = vectorString(sub.trajectory.joint_trajectory.joint_names);
        fail_controller = sub.execution_info.controller_names.empty() ?
          controllerForJoints(sub.trajectory.joint_trajectory.joint_names) :
          sub.execution_info.controller_names.front();
      }
      RCLCPP_ERROR(
        get_logger(),
        "[MTC] EXECUTION_FAILED FAIL_STAGE=subtrajectory_%d/%zu FAIL_CONTROLLER=%s "
        "FAIL_REASON=%s (%d) stage_id=%u joints=%s",
        fail_index + 1, solution_message.sub_trajectory.size(), fail_controller.c_str(),
        moveItErrorName(report->error_code.load()).c_str(), report->error_code.load(),
        fail_stage_id, fail_joints.c_str());
      finishGoal(
        goal_handle, false,
        "EXECUTION_ABORTED: subtrajectory " + std::to_string(fail_index + 1) +
          " controller " + fail_controller + " reason " +
          moveItErrorName(report->error_code.load()) + " (" +
          std::to_string(report->error_code.load()) + ")");
      return;
    }
    publishStage(goal_handle, "COMPLETE");
    finishGoal(goal_handle, true, "MTC PICK AND PLACE: PASS");
  }

  // ------------------------------------------------------------- benchmark
  // Translation-progress estimate of a Cartesian stage: |end - start| /
  // |target - start| of the TCP. Used on success AND failure solutions, so
  // a partial Cartesian path is visible even when the stage fails.
  double cartesianFraction(const mtc::SolutionBase& solution,
                           const geometry_msgs::msg::Pose& target) const
  {
    const auto* start = solution.start();
    const auto* end = solution.end();
    if (start == nullptr || end == nullptr) {
      return 0.0;
    }
    const auto& state0 = start->scene()->getCurrentState();
    const auto& state1 = end->scene()->getCurrentState();
    const Eigen::Isometry3d p0 = state0.getGlobalLinkTransform(tcp_frame_);
    const Eigen::Isometry3d p1 = state1.getGlobalLinkTransform(tcp_frame_);
    const Eigen::Vector3d t(target.position.x, target.position.y, target.position.z);
    const double total = (t - p0.translation()).norm();
    if (total < 1e-6) {
      return 1.0;
    }
    return std::clamp((p1.translation() - p0.translation()).norm() / total, 0.0, 1.0);
  }

  double cartesianTravelFraction(const mtc::SolutionBase& solution, double requested) const
  {
    const auto* start = solution.start();
    const auto* end = solution.end();
    if (start == nullptr || end == nullptr || requested < 1e-6) {
      return 0.0;
    }
    const Eigen::Vector3d p0 =
      start->scene()->getCurrentState().getGlobalLinkTransform(tcp_frame_).translation();
    const Eigen::Vector3d p1 =
      end->scene()->getCurrentState().getGlobalLinkTransform(tcp_frame_).translation();
    return std::clamp((p1 - p0).norm() / requested, 0.0, 1.0);
  }

  void collectJoint4(const mtc::Task& task, TargetMetrics& metrics)
  {
    moveit_task_constructor_msgs::msg::Solution message;
    for (const auto& solution : task.solutions()) {
      message = moveit_task_constructor_msgs::msg::Solution{};
      solution->toMsg(message);
      double solution_min = std::numeric_limits<double>::infinity();
      double solution_max = -std::numeric_limits<double>::infinity();
      bool found = false;
      for (const auto& sub : message.sub_trajectory) {
        const auto& names = sub.trajectory.joint_trajectory.joint_names;
        const auto joint_it = std::find(names.begin(), names.end(), "joint4");
        if (joint_it == names.end()) {
          continue;
        }
        const std::size_t index = static_cast<std::size_t>(joint_it - names.begin());
        for (const auto& point : sub.trajectory.joint_trajectory.points) {
          if (index < point.positions.size()) {
            found = true;
            solution_min = std::min(solution_min, point.positions[index]);
            solution_max = std::max(solution_max, point.positions[index]);
          }
        }
      }
      if (!found) {
        continue;
      }
      metrics.j4_valid = true;
      metrics.j4_min = std::min(metrics.j4_min, solution_min);
      metrics.j4_max = std::max(metrics.j4_max, solution_max);
      if (solution_min < -1.28 - 1e-6) {
        ++metrics.solutions_using_j4_below_old_min;
      }
    }
  }

  std::vector<TargetMetrics> loadBenchmarkTargets()
  {
    std::vector<TargetMetrics> targets;
    std::ifstream stream(benchmark_targets_file_);
    if (!stream.is_open()) {
      RCLCPP_ERROR(
        get_logger(), "[BENCH] cannot open targets file '%s'",
        benchmark_targets_file_.c_str());
      return targets;
    }
    std::string line;
    while (std::getline(stream, line)) {
      if (line.empty() || line[0] == '#') {
        continue;
      }
      std::istringstream tokens(line);
      TargetMetrics target;
      if (!(tokens >> target.id >> target.cube_x >> target.cube_y >> target.cube_z >>
            target.box_x >> target.box_y)) {
        RCLCPP_WARN(get_logger(), "[BENCH] skipping malformed targets line '%s'", line.c_str());
        continue;
      }
      target.j4_min = std::numeric_limits<double>::infinity();
      targets.push_back(target);
    }
    return targets;
  }

  FrozenInput targetToFrozen(const TargetMetrics& target) const
  {
    FrozenInput frozen;
    frozen.cube = makePose(
      base_frame_, target.cube_x, target.cube_y, target.cube_z,
      []() { geometry_msgs::msg::Quaternion identity; identity.w = 1.0; return identity; }());
    frozen.box = makePose(
      base_frame_, target.box_x, target.box_y, 0.0,
      []() { geometry_msgs::msg::Quaternion identity; identity.w = 1.0; return identity; }());
    return frozen;
  }

  static std::string benchmarkTimestamp()
  {
    const auto now = std::chrono::system_clock::now();
    const std::time_t time = std::chrono::system_clock::to_time_t(now);
    std::tm local{};
    localtime_r(&time, &local);
    std::ostringstream stream;
    stream << std::put_time(&local, "%Y%m%d_%H%M%S");
    return stream.str();
  }

  void runBenchmark(const std::shared_ptr<GoalHandle>& goal_handle)
  {
    RCLCPP_INFO(get_logger(), "[BENCH] mode: plan-only; execution disabled");
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      RCLCPP_INFO(
        get_logger(), "[BENCH] start_joints=%s joint_states_age_ms=%.0f",
        vectorString(last_joint_positions_).c_str(),
        milliseconds(joint_state_received_, SteadyClock::now()));
    }
    publishStage(goal_handle, "BENCHMARK_START");
    std::vector<TargetMetrics> targets = loadBenchmarkTargets();
    if (targets.empty()) {
      finishGoal(goal_handle, false, "BENCHMARK_FAILED: no targets loaded");
      return;
    }
    RCLCPP_INFO(get_logger(), "[BENCH] loaded %zu targets", targets.size());

    std::ostringstream report;
    report << "id cube_x cube_y cube_z box_x box_y current_states grasp_ik place_ik "
              "approach_fraction approach_success lift_fraction lift_success "
              "connect1 connect2 pick_complete complete planning_ms j4_min j4_max j4_below_old\n";
    std::size_t planning_success = 0;
    std::size_t pick_success = 0;
    std::size_t total_complete = 0;
    std::size_t ik_failed = 0;
    std::size_t cartesian_partial = 0;
    double global_j4_min = std::numeric_limits<double>::infinity();

    for (auto& target : targets) {
      publishStage(goal_handle, "BENCHMARK_TARGET_" + std::to_string(target.id));
      auto counters = std::make_shared<CandidateCounters>();
      const FrozenInput frozen = targetToFrozen(target);
      std::unique_ptr<mtc::Task> task;
      try {
        task = createTask(frozen, counters, &target);
      } catch (const std::exception& exception) {
        RCLCPP_ERROR(get_logger(), "[BENCH] target %zu task init failed: %s",
                     target.id, exception.what());
        continue;
      }
      {
        std::lock_guard<std::mutex> lock(active_task_mutex_);
        active_task_ = task.get();
      }
      const auto start = SteadyClock::now();
      moveit::core::MoveItErrorCode plan_result;
      try {
        plan_result = task->plan(static_cast<std::size_t>(max_complete_solutions_));
      } catch (const std::exception& exception) {
        RCLCPP_ERROR(get_logger(), "[BENCH] target %zu plan threw: %s",
                     target.id, exception.what());
      }
      target.planning_ms = milliseconds(start, SteadyClock::now());
      target.grasp_ik = counters->grasp_ik.load();
      target.place_ik = counters->place_ik.load();
      target.complete = task->numSolutions();
      collectJoint4(*task, target);
      {
        std::lock_guard<std::mutex> lock(active_task_mutex_);
        active_task_ = nullptr;
      }
      if (target.pick_complete > 0) {
        ++pick_success;
      }
      if (plan_result && target.complete > 0) {
        ++planning_success;
        total_complete += target.complete;
      }
      if (target.complete == 0 && (target.grasp_ik == 0 || target.place_ik == 0)) {
        ++ik_failed;
      }
      if (target.approach_fraction < 0.999 || target.lift_fraction < 0.999) {
        ++cartesian_partial;
      }
      if (target.j4_valid) {
        global_j4_min = std::min(global_j4_min, target.j4_min);
      }
      RCLCPP_INFO(
        get_logger(),
        "[BENCH] target=%zu cube=(%.4f,%.4f,%.4f) box=(%.4f,%.4f) current_states=%zu "
        "grasp_ik=%zu place_ik=%zu approach=%.3f/%zu lift=%.3f/%zu connect=%zu/%zu "
        "pick=%zu complete=%zu plan_ms=%.1f j4=[%s] j4_below_old=%zu",
        target.id, target.cube_x, target.cube_y, target.cube_z, target.box_x,
        target.box_y, counters->current_states.load(), target.grasp_ik,
        target.place_ik, target.approach_fraction, target.approach_success,
        target.lift_fraction, target.lift_success, target.connect1, target.connect2,
        target.pick_complete, target.complete, target.planning_ms,
        target.j4_valid ?
          ("[" + std::to_string(target.j4_min) + ", " + std::to_string(target.j4_max) + "]").c_str() :
          "[none]",
        target.solutions_using_j4_below_old_min);
      report << target.id << " " << target.cube_x << " " << target.cube_y << " "
             << target.cube_z << " " << target.box_x << " " << target.box_y << " "
             << counters->current_states.load() << " " << target.grasp_ik << " "
             << target.place_ik << " " << target.approach_fraction << " "
             << target.approach_success << " " << target.lift_fraction << " "
             << target.lift_success << " " << target.connect1 << " "
             << target.connect2 << " " << target.pick_complete << " "
             << target.complete << " "
             << target.planning_ms << " "
             << (target.j4_valid ? target.j4_min : 0.0) << " "
             << (target.j4_valid ? target.j4_max : 0.0) << " "
             << target.solutions_using_j4_below_old_min << "\n";
    }

    std::ostringstream summary;
    summary << "[BENCH] SUMMARY planning_success=" << planning_success << "/"
            << targets.size() << " pick_success=" << pick_success << "/"
            << targets.size() << " total_complete_solutions=" << total_complete
            << " IK_failed=" << ik_failed << " cartesian_partial=" << cartesian_partial
            << " minimum_joint4_used="
            << (global_j4_min < 1e9 ? std::to_string(global_j4_min) : std::string("n/a"));
    RCLCPP_INFO(get_logger(), "%s", summary.str().c_str());
    publishStage(goal_handle, "BENCHMARK_DONE");

    const std::string out_path =
      benchmark_out_dir_ + "/benchmark_" + benchmarkTimestamp() + ".txt";
    std::ofstream out(out_path);
    if (out.is_open()) {
      out << "# X5A MTC plan-only benchmark (task_mode=benchmark)\n";
      out << "# " << summary.str() << "\n";
      out << report.str();
      out.close();
      RCLCPP_INFO(get_logger(), "[BENCH] results written to %s", out_path.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "[BENCH] cannot write results to %s", out_path.c_str());
    }
    finishGoal(goal_handle, true, summary.str());
  }

  void runReachability(const std::shared_ptr<GoalHandle>& goal_handle)
  {
    publishStage(goal_handle, "REACHABILITY_START");
    const auto* jmg = robot_model_->getJointModelGroup(arm_group_);
    if (jmg == nullptr) {
      finishGoal(goal_handle, false, "REACHABILITY_FAILED: arm group missing");
      return;
    }

    planning_scene::PlanningScene scene(robot_model_);
    scene.processCollisionObjectMsg(makeBox(
      "table", base_frame_, table_x_, table_y_, table_z_,
      table_size_x_, table_size_y_, table_size_z_));
    auto& acm = scene.getAllowedCollisionMatrixNonConst();
    for (const char* link : { "link6", "link7", "link8", "tool0", "table" }) {
      acm.setEntry("object", link, true);
    }

    const double step = std::max(0.02, reachability_step_);
    const double cube_z = reachability_z_;
    const double grasp_z = cube_z + grasp_offset_z_;
    std::vector<double> xs;
    std::vector<double> ys;
    for (double x = workspace_x_min_; x <= workspace_x_max_ + 1e-9; x += step) {
      xs.push_back(x);
    }
    for (double y = workspace_y_min_; y <= workspace_y_max_ + 1e-9; y += step) {
      ys.push_back(y);
    }

    const std::vector<std::vector<double>> seeds = {
      { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },
      { 0.0, 0.8, 0.8, -0.5, 0.0, 0.0 },
    };

    std::size_t reachable = 0;
    std::size_t ik_only = 0;
    std::size_t none = 0;
    std::ostringstream grid;
    std::ostringstream details;
    grid << "y\\x";
    for (double x : xs) {
      grid << " " << std::fixed << std::setprecision(2) << x;
    }
    grid << "\n";
    details << "x y r ik free best_pitch\n";

    moveit::core::RobotState state(robot_model_);
    state.setToDefaultValues();
    if (state.getJointModel("joint7") != nullptr) {
      const double open = 0.044;
      state.setJointPositions("joint7", &open);
      state.setJointPositions("joint8", &open);
    }

    for (auto y_it = ys.rbegin(); y_it != ys.rend(); ++y_it) {
      const double y = *y_it;
      grid << std::fixed << std::setprecision(2) << y;
      for (double x : xs) {
        scene.processCollisionObjectMsg(makeBox(
          "object", base_frame_, x, y, cube_z - object_size_z_ * 0.5,
          object_size_x_, object_size_y_, object_size_z_));
        const auto oris = sampleGraspOrientations(x, y);
        std::size_t ik = 0;
        std::size_t free = 0;
        double best_pitch = 0.0;
        std::vector<std::vector<double>> cell_seeds = seeds;
        cell_seeds.push_back({ std::atan2(y, x), 0.6, 1.0, -0.8, 0.0, 0.0 });
        for (const auto& rpy : oris) {
          const Eigen::Isometry3d pose = eigenFromXyzRpy(
            x + grasp_offset_x_, y + grasp_offset_y_, grasp_z, rpy);
          bool found = false;
          for (const auto& seed : cell_seeds) {
            state.setJointGroupPositions(jmg, seed);
            state.update();
            if (state.setFromIK(jmg, pose, tcp_frame_, 0.08)) {
              found = true;
              break;
            }
          }
          if (!found) {
            continue;
          }
          ++ik;
          collision_detection::CollisionRequest request;
          collision_detection::CollisionResult result;
          scene.checkCollision(request, result, state);
          if (!result.collision) {
            ++free;
            best_pitch = rpy[1];
          }
        }
        char mark = '.';
        if (free > 0) {
          mark = '#';
          ++reachable;
        } else if (ik > 0) {
          mark = 'o';
          ++ik_only;
        } else {
          ++none;
        }
        grid << "   " << mark;
        details << std::fixed << std::setprecision(3) << x << " " << y << " "
                << std::hypot(x, y) << " " << ik << " " << free << " "
                << best_pitch << "\n";
      }
      grid << "\n";
    }

    std::ostringstream summary;
    summary << "[REACH] cells=" << (xs.size() * ys.size())
            << " reachable=" << reachable << " ik_only=" << ik_only
            << " none=" << none << " step=" << step
            << " grasp_z=" << grasp_z;
    RCLCPP_INFO(get_logger(), "%s", summary.str().c_str());
    RCLCPP_INFO(get_logger(), "[REACH] legend: # collision-free IK  o IK colliding  . no IK\n%s",
                grid.str().c_str());

    const std::string out_path =
      benchmark_out_dir_ + "/reachability_" + benchmarkTimestamp() + ".txt";
    std::ofstream out(out_path);
    if (out.is_open()) {
      out << "# X5A grasp reachability (collision-aware KDL)\n";
      out << "# " << summary.str() << "\n";
      out << "# workspace x=[" << workspace_x_min_ << "," << workspace_x_max_
          << "] y=[" << workspace_y_min_ << "," << workspace_y_max_ << "]\n";
      out << "# legend: # collision-free IK  o IK colliding  . no IK\n";
      out << grid.str() << "\n" << details.str();
      out.close();
      RCLCPP_INFO(get_logger(), "[REACH] results written to %s", out_path.c_str());
    }
    publishStage(goal_handle, "REACHABILITY_DONE");
    finishGoal(goal_handle, true, summary.str());
  }

  std::mutex data_mutex_;
  std::map<std::string, TimedPose> cube_poses_;
  std::map<std::string, TimedStable> cube_stable_;
  TimedPose box_pose_;
  TimedStable box_stable_;
  bool joint_states_valid_{ false };
  SteadyClock::time_point joint_state_received_{};
  std::vector<double> last_joint_positions_;

  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr>
    cube_pose_subscriptions_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr>
    cube_stable_subscriptions_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr box_pose_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr box_stable_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;

  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr arm_client_;
  rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr gripper_client_;
  rclcpp_action::Client<moveit_task_constructor_msgs::action::ExecuteTaskSolution>::SharedPtr
    execute_client_;
  rclcpp_action::Server<PickPlace>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr ready_timer_;

  std::atomic_bool busy_{ false };
  bool ready_announced_{ false };
  SteadyClock::time_point command_received_{};
  std::mutex active_task_mutex_;
  mtc::Task* active_task_{ nullptr };
  rclcpp_action::ClientGoalHandle<
    moveit_task_constructor_msgs::action::ExecuteTaskSolution>::SharedPtr active_exec_goal_;

  std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
  moveit::core::RobotModelConstPtr robot_model_;
  mtc::solvers::PipelinePlannerPtr transit_planner_;
  mtc::solvers::CartesianPathPtr precision_cartesian_;
  mtc::solvers::CartesianPathPtr lift_cartesian_;
  mtc::solvers::CartesianPathPtr flexible_cartesian_;
  mtc::solvers::JointInterpolationPlannerPtr gripper_planner_;

  std::string arm_group_;
  std::string gripper_group_;
  std::string eef_name_;
  std::string base_frame_;
  std::string tcp_frame_;
  std::string action_name_;
  std::string arm_action_name_;
  std::string gripper_action_name_;
  std::string execute_task_action_name_;
  std::string joint_state_topic_;
  std::string box_pose_topic_;
  std::string box_stable_topic_;
  std::map<std::string, std::string> cube_pose_topics_;
  std::map<std::string, std::string> cube_stable_topics_;

  double transit_velocity_;
  double transit_acceleration_;
  double precision_velocity_;
  double precision_acceleration_;
  double lift_velocity_;
  double lift_acceleration_;
  std::vector<double> grasp_rpy_;
  std::vector<double> transit_rpy_;
  std::vector<std::vector<double>> grasp_orientations_;
  std::vector<std::vector<double>> place_orientations_;
  double max_pose_age_;
  double object_size_x_;
  double object_size_y_;
  double object_size_z_;
  double place_z_;
  double table_x_;
  double table_y_;
  double table_z_;
  double table_size_x_;
  double table_size_y_;
  double table_size_z_;
  double pre_grasp_height_;
  double lift_height_;
  double pre_place_height_;
  double retreat_height_;
  double cartesian_step_;
  double jump_threshold_;
  double min_cartesian_fraction_;
  double grasp_offset_x_;
  double grasp_offset_y_;
  double grasp_offset_z_;
  std::vector<double> ready_joints_;
  int max_ik_solutions_;
  double ik_timeout_;
  int max_complete_solutions_;
  double connect_timeout_;
  double plan_timeout_;
  double place_candidate_offset_;
  double place_candidate_yaw_;
  int approach_candidate_count_;
  double approach_pitch_step_;
  int max_grasp_orientations_;
  int max_place_orientations_;
  double cartesian_min_height_scale_;
  double reachability_step_;
  double reachability_z_;
  double joint_state_max_age_;
  double current_state_timeout_;
  bool plan_only_;
  std::string task_mode_;
  double execute_timeout_;
  std::string benchmark_targets_file_;
  std::string benchmark_out_dir_;
  double workspace_x_min_;
  double workspace_x_max_;
  double workspace_y_min_;
  double workspace_y_max_;
  double workspace_r_max_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<X5aMtcTaskServer>(options);
  if (!node->initialize()) {
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
