// Luca Grigolin at PROFACTOR GmbH

// Helper Functions for Planner and Executor

#pragma once

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <vector>

#include <Eigen/Geometry>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <rosbag2_cpp/writer.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <nlohmann/json.hpp>
#include <filesystem>
#include <fstream>
#include <rclcpp/duration.hpp>

#include <rosbag2_cpp/reader.hpp>

#include <moveit_msgs/msg/move_it_error_codes.hpp>

#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp> // to check collisions

#include <geometric_shapes/shape_operations.h>   // constructMsgFromShape
#include <geometric_shapes/mesh_operations.h>    // createMeshFromResource

#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>      // TOTG

#include <tf2_ros/buffer.h> // Tf
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <random> // for rng

#include <std_msgs/msg/color_rgba.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>

#include <stdexcept>
#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_share_directory.hpp>


using moveit::planning_interface::PlanningSceneInterface;
using json = nlohmann::json;
using Pose6 = std::array<double, 6>;

std::string pkg_share =
    ament_index_cpp::get_package_share_directory("biostruct_amura");
std::filesystem::path ws_folder = std::filesystem::path(pkg_share)
                                    .parent_path()  // biostruct_amura
                                    .parent_path()  // share
                                    .parent_path()  // install
                                    .parent_path(); // abb_ws
std::string ws_path = ws_folder.string();

// MQTT

// -------------------- plan --------------------
struct plan {
    std::string ply_id;
    std::vector<std::string> path;
    double travel_height;
    double approach;
    double speed;
};

inline void from_json(const json& j, plan& p) {
    j.at("ply_id").get_to(p.ply_id);
    j.at("path").get_to(p.path);
    j.at("travel_height").get_to(p.travel_height);
    j.at("approach").get_to(p.approach);
    j.at("speed").get_to(p.speed);
}

// -------------------- PickingPose --------------------
struct PickingPose {
    double x, y, z;
    double rx, ry, rz;
    double dx, dy, theta;
};

inline void from_json(const json& j, PickingPose& p) {
    j.at("x").get_to(p.x);
    j.at("y").get_to(p.y);
    j.at("z").get_to(p.z);
    j.at("rx").get_to(p.rx);
    j.at("ry").get_to(p.ry);
    j.at("rz").get_to(p.rz);
    j.at("dx").get_to(p.dx);
    j.at("dy").get_to(p.dy);
    j.at("theta").get_to(p.theta);
}

// -------------------- PickingPose --------------------

// CONVERSIONS

// - math
std::vector<double> deg2rad(const std::vector<double>& degrees) {
  constexpr double DEG2RAD = M_PI / 180.0;
  std::vector<double> radians(degrees.size());
  std::transform(degrees.begin(), degrees.end(), radians.begin(),
                 [=](double d) { return d * DEG2RAD; });
  return radians;
}

std::vector<double> rad2deg(const std::vector<double>& radians) {
  constexpr double RAD2DEG = 180.0 / M_PI;
  std::vector<double> degrees(radians.size());
  std::transform(radians.begin(), radians.end(), degrees.begin(),
                 [=](double r) { return r * RAD2DEG; });
  return degrees;
}


// - data types
geometry_msgs::msg::Pose isometry3dToPose(const Eigen::Isometry3d& transform)
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = transform.translation().x();
    pose.position.y = transform.translation().y();
    pose.position.z = transform.translation().z();
    Eigen::Quaterniond q(transform.rotation());
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    return pose;
}

// LOCAL DATA

// Joint angle definitions in degrees (Gripper Adapter 10 cm)
std::unordered_map<std::string, std::vector<double>> joint_values_deg = {
  {"DefaultPose",         {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
  {"RefPickingPose",      {-51.0, 20.0, 15.0, 2.0, 54.0, -68.62}}, // to seed for faster IK on PickingPose
  {"PickingPose",         {-51.0, 20.0, 15.0, 2.0, 54.0, -68.62}}, 
  // {"bottomPlacingPose",   {12.0, 16.0, 42.0, -1.0, 30.0, 13.0}}, 
  {"refPlacingPose",      {12.0, 16.0, 42.0, -1.0, 30.0, 13.0}}, 
  // {"sternPlacingPose",    {10.0, 32.0, 27.0, 10.0, 111.0, 6.0}},
  // {"sidewallPlacingPose", {3.0, 22.0, 39.0, -78.0, 68.0, -28.0}},
  {"visionPose",          {-52.0, 15.0, -12.0, 1.0, 86.0, -68.62}} 
};
//3.20 m roof

// Lookup function with conversion to radians
std::vector<double> getJointValues(const std::string& pose_name) {
  auto it = joint_values_deg.find(pose_name);
  if (it == joint_values_deg.end()) {
      throw std::invalid_argument("Unknown pose name: " + pose_name);
  }
  return deg2rad(it->second);
}

// add or overwrite joint values for a given pose name
void setJointValues(const std::string& name,
                    const std::vector<double>& values)
{
    joint_values_deg[name] = values;   // inserts or overwrites
}

// - kinematics
geometry_msgs::msg::PoseStamped getPoseFromJointValues(
  const std::vector<double>& joint_values,
  const moveit::core::RobotModelConstPtr& robot_model,
  const std::string& planning_group,
  const std::string& link_name)
{
  moveit::core::RobotState robot_state(robot_model);
  robot_state.setToDefaultValues();
  robot_state.setJointGroupPositions(planning_group, joint_values);
  robot_state.update();

  const Eigen::Isometry3d& link_pose = robot_state.getGlobalLinkTransform(link_name);

  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.frame_id = robot_model->getModelFrame();
  pose_msg.pose = isometry3dToPose(link_pose);

  return pose_msg;
}


// Get the Pose from its name
geometry_msgs::msg::PoseStamped getPose(
  const std::string& pose_name,
  const moveit::core::RobotModelConstPtr& robot_model,
  const std::string& planning_group,
  const std::string& link_name
){
  return getPoseFromJointValues(getJointValues(pose_name), robot_model, planning_group, link_name);
}


std::vector<double> getJointValuesFromState(
  const moveit::core::RobotStateConstPtr& state,
  const std::string& planning_group)
{
  std::vector<double> joint_values;
  state->copyJointGroupPositions(planning_group, joint_values);
  return joint_values;
}


// - Inverse Kinematics
void setJointValuesFromPose(const std::string& seed_name,
             const geometry_msgs::msg::Pose& target,
             const std::string& out_key,
             const moveit::core::RobotModelConstPtr& robot_model,
             const std::string& planning_group,
             const std::string& link_name)
{
    const auto* jmg = robot_model->getJointModelGroup(planning_group);
    if (!jmg)
        throw std::runtime_error("Invalid planning group: " + planning_group);

    moveit::core::RobotState state(robot_model);
    state.setToDefaultValues();
    state.setJointGroupPositions(planning_group,
                                 deg2rad(joint_values_deg.at(seed_name)));
    state.update();

    if (!state.setFromIK(jmg, target, link_name))
        throw std::runtime_error("IK failed for " + out_key);

    joint_values_deg[out_key] =
        rad2deg(getJointValuesFromState(
            std::make_shared<moveit::core::RobotState>(state),
            planning_group));
}



// ROBOT PLY-POSE YAML MEMORY

// retrieve from yaml file the joint values
inline Pose6 yaml_seq_to_pose6(const YAML::Node& n)
{
    if (!n || !n.IsSequence() || n.size() != 6)
        throw std::runtime_error("Expected a sequence of 6 doubles");
    Pose6 p{};
    for (std::size_t i = 0; i < 6; ++i) p[i] = n[i].as<double>();
    return p;
}

// Read from the memory by Ply ID
inline std::optional<Pose6> read_from_memory(const std::string& yaml_path,
                                             const std::string& id)
{
    YAML::Node root = YAML::LoadFile(yaml_path);
    YAML::Node poses = root["poses"];
    if (!poses || !poses.IsMap()) return std::nullopt;

    YAML::Node entry = poses[id];
    if (!entry) return std::nullopt;

    return yaml_seq_to_pose6(entry);
}

// to not corrupt the file if the process crashes midwrite
inline void atomic_write_text(const std::string& path, const std::string& content)
{
    namespace fs = std::filesystem;
    fs::path p(path);
    fs::create_directories(p.parent_path());

    fs::path tmp = p;
    tmp += ".tmp";

    {
        std::ofstream out(tmp.string(), std::ios::out | std::ios::trunc);
        if (!out) throw std::runtime_error("Failed to open temp file for writing: " + tmp.string());
        out << content;
        out.flush();
        if (!out) throw std::runtime_error("Failed while writing temp file: " + tmp.string());
    }

    // Atomic on POSIX if same filesystem
    fs::rename(tmp, p);
}

// write or overwritejoint values given a ply ID
inline void write_to_memory(const std::string& yaml_path,
                               const std::string& id,
                               const Pose6& values)
{
    YAML::Node root;
    try {
        root = YAML::LoadFile(yaml_path);
    } catch (...) {
        root = YAML::Node(YAML::NodeType::Map);
    }

    if (!root["poses"] || !root["poses"].IsMap())
        root["poses"] = YAML::Node(YAML::NodeType::Map);

    YAML::Node seq(YAML::NodeType::Sequence);
    seq.SetStyle(YAML::EmitterStyle::Flow);   // <-- force in line [j1, ... , j6]
    for (double v : values) seq.push_back(v);

    root["poses"][id] = seq;

    // Serialize then atomic write
    std::stringstream ss;
    ss << root;
    atomic_write_text(yaml_path, ss.str());
}

// load joint values for placing plies from the memory.yaml
void loadMemory(const std::string& memory_yaml_path)
{
    YAML::Node root;
    try
    {
        root = YAML::LoadFile(memory_yaml_path);
    }
    catch (const std::exception& e)
    {
        throw std::runtime_error("Failed to load memory: File not Found");
        return;
    }

    if (!root["poses"] || !root["poses"].IsMap())
    {
        throw std::runtime_error("Failed to load memory: Memory Empty");
        return;
    }

    const YAML::Node& poses = root["poses"];

    for (auto it = poses.begin(); it != poses.end(); ++it)
    {
        const std::string pose_name = it->first.as<std::string>();
        const YAML::Node& values_node = it->second;

        if (!values_node.IsSequence() || values_node.size() != 6)
        {
            RCLCPP_WARN(rclcpp::get_logger("[Memory Loader]"),
                        "Pose '%s' ignored: expected 6 joint values",
                        pose_name.c_str());
            continue;
        }

        std::vector<double> values(6);
        for (std::size_t i = 0; i < 6; ++i)
            values[i] = values_node[i].as<double>();

        // Insert or overwrite
        if(pose_name == "vision"){
          setJointValues(pose_name+"Pose", values);
        }
        else{
          setJointValues(pose_name+"PlacingPose", values);
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("[Memory Loader]"), "Loaded %zu ply poses from memory.yaml", poses.size());
}


// PATH PLANNING FUNCTIONS 


// Projection of the pick and place poses for human collaborative paths 
void projectWaypoints(const std::string& ply_id,
                      const double& travel_height,
                      const double& approach,
                      const moveit::core::RobotModelConstPtr& robot_model,
                      const std::string& planning_group,
                      const std::string& link_name)
{
    auto pick  = getPose("PickingPose",  robot_model, planning_group, link_name);
    auto mould = getPose("refPlacingPose", robot_model, planning_group, link_name);
    auto place = getPose(ply_id+"PlacingPose", robot_model, planning_group, link_name);

    // --- lambda helpers -------------------------------------------------

    // apply an offset along x-axis of the given pose
    auto applyOffsetTool = [](geometry_msgs::msg::Pose& pose,
                              const Eigen::Vector3d& offset_tool)
    {
      Eigen::Quaterniond q(pose.orientation.w,
                           pose.orientation.x,
                           pose.orientation.y,
                           pose.orientation.z);

      Eigen::Vector3d p(pose.position.x,
                        pose.position.y,
                        pose.position.z);

      p += q * offset_tool;

      pose.position.x = p.x();
      pose.position.y = p.y();
      pose.position.z = p.z();
    };

    // returns the x-axis unit vector of the given pose
    auto getXAxis = [](const geometry_msgs::msg::Pose& pose) {
      Eigen::Quaterniond q(pose.orientation.w,
                           pose.orientation.x,
                           pose.orientation.y,
                           pose.orientation.z);
      return q * Eigen::Vector3d::UnitX();
    };

    // compute offset for pick 
    double offset_pick_x = travel_height-pick.pose.position.z;
    if (offset_pick_x < 0.0)  throw std::runtime_error("Travel height lower than picking pose.");

    // --- Compute bottom placing offset -------------------------------------
    
    Eigen::Vector3d x_axis = getXAxis(mould.pose);   // world direction of tool X
    double denom = x_axis.z();
    if (std::fabs(denom) < 1e-6)
      throw std::runtime_error("Bottom place pose X-axis is parallel to world XY plane; cannot project to a Z plane.");

    double offset_mould_x = (travel_height - mould.pose.position.z) / denom;
    // --- Apply projection tool-frame offsets ----------------------------------

    applyOffsetTool(pick.pose,  Eigen::Vector3d(-offset_pick_x, 0.0, 0.0));
    applyOffsetTool(mould.pose, Eigen::Vector3d(offset_mould_x, 0.0, 0.0));
    applyOffsetTool(place.pose, Eigen::Vector3d(-approach, 0.0, 0.0));

    // Force Mould Top orientation equal to projected pick orientation
    mould.pose.orientation = pick.pose.orientation;
    // Force Mould Top x-position equal to projected pick x-position
    mould.pose.position.x = pick.pose.position.x;

    // --- Solve IK -------------------------------------------------------------

    setJointValuesFromPose("PickingPose",        pick.pose,  "TableTopPose",robot_model, planning_group, link_name);
    setJointValuesFromPose("refPlacingPose",  mould.pose, "MouldTopPose",robot_model, planning_group, link_name);
    setJointValuesFromPose(ply_id+"PlacingPose",  place.pose, ply_id+"TopPose",robot_model, planning_group, link_name);
}


// sample a point from the upper sphere for calibration
Eigen::Vector3d sample_cap_point_surface(
    const Eigen::Vector3d& c, double r,
    double z_min_offset, std::mt19937& rng)
{
  // constraint: (p.z - c.z) >= z_min_offset
  // On sphere surface: p = c + r * dir, dir.z = cos(theta)
  const double cos_theta_min = z_min_offset / r;
  const double cos_theta_max = 1.0;

  std::uniform_real_distribution<double> U01(0.0, 1.0);
  std::uniform_real_distribution<double> Uphi(0.0, 2.0 * M_PI);

  const double u = U01(rng);
  const double cos_theta = cos_theta_min + (cos_theta_max - cos_theta_min) * u;
  const double sin_theta = std::sqrt(std::max(0.0, 1.0 - cos_theta * cos_theta));
  const double phi = Uphi(rng);

  Eigen::Vector3d dir(sin_theta * std::cos(phi),
                      sin_theta * std::sin(phi),
                      cos_theta);

  return c + r * dir;
}


// Build rotation so that:
// - z axis points from p -> c (toward sphere center)
// - x/y preserve the board orientation AFTER a 180° rotation about the board's local z-axis
//   (i.e., we use R0 * RotZ(pi) as the reference "roll" frame)
Eigen::Matrix3d make_look_at_board(
    const Eigen::Matrix3d& R0,
    const Eigen::Vector3d& p,     // new position
    const Eigen::Vector3d& c)     // center to look at
{
  Eigen::Vector3d z_new = (c - p);
  const double zn = z_new.norm();
  if (zn < 1e-9) throw std::runtime_error("p is at center; undefined look direction");
  z_new /= zn;

  // Reference axes: board rotated 180° about its *local* z
  // R_ref = R0 * RotZ(pi)  =>  x_ref = -x0, y_ref = -y0, z_ref = z0
  const Eigen::Vector3d x_ref = -R0.col(0);
  const Eigen::Vector3d y_ref = -R0.col(1);

  // Project x_ref onto plane orthogonal to z_new
  Eigen::Vector3d x_new = x_ref - (x_ref.dot(z_new)) * z_new;
  if (x_new.norm() < 1e-8) {
    // x_ref was nearly parallel to z_new, fall back to y_ref
    x_new = y_ref - (y_ref.dot(z_new)) * z_new;
  }
  x_new.normalize();

  // Right-handed frame (x cross y = z)  =>  y = z cross x
  Eigen::Vector3d y_new = z_new.cross(x_new);
  if (y_new.norm() < 1e-12) throw std::runtime_error("Degenerate frame construction");
  y_new.normalize();

  // Re-orthonormalize x (guards numeric drift)
  x_new = y_new.cross(z_new);
  x_new.normalize();

  Eigen::Matrix3d R;
  R.col(0) = x_new;
  R.col(1) = y_new;
  R.col(2) = z_new;
  return R;
}



// Sample the new pose for extrinsic calibration
geometry_msgs::msg::Pose sample_pose_on_cap_look_center(
    moveit_visual_tools::MoveItVisualTools& visual_tools,
    const geometry_msgs::msg::Pose& pose_in,
    const double& object_radius,
    std::mt19937& rng)
{
  // Sample radius r in [2.0R, 2.5R]
  std::uniform_real_distribution<double> r_dist(
      1.7 * object_radius,
      2.5 * object_radius);
  const double r = r_dist(rng);

  // Center of sphere = input pose position
  Eigen::Vector3d c(pose_in.position.x, pose_in.position.y, pose_in.position.z);

  // Initial orientation (board pose)
  Eigen::Quaterniond q0(pose_in.orientation.w,
                        pose_in.orientation.x,
                        pose_in.orientation.y,
                        pose_in.orientation.z);
  Eigen::Matrix3d R0 = q0.normalized().toRotationMatrix();

  // Sample point on spherical cap: z_offset > 60% of r
  const double z_min_offset = 0.65 * r;
  Eigen::Vector3d p = sample_cap_point_surface(c, r, z_min_offset, rng);

  // Orientation: x points to center, y/z preserve roll from board pose
  Eigen::Matrix3d R = make_look_at_board(R0, p, c);
  Eigen::Quaterniond q(R);
  q.normalize();

  geometry_msgs::msg::Pose out;
  out.position.x = p.x();
  out.position.y = p.y();
  out.position.z = p.z();
  out.orientation.x = q.x();
  out.orientation.y = q.y();
  out.orientation.z = q.z();
  out.orientation.w = q.w();

  // --- RViz visualization: sphere centered exactly at the input pose ---

  std_msgs::msg::ColorRGBA color;
  color.r = 0.2f; color.g = 0.6f; color.b = 0.9f; color.a = 0.5f;

  geometry_msgs::msg::Vector3 scale;
  scale.x = 2.0 * r ;
  scale.y = 2.0 * r ;
  scale.z = 2.0 * r ;

  // NOTE: pass Pose (not PoseStamped)
  visual_tools.publishSphere(pose_in, color, scale);
  visual_tools.trigger();

  return out;
}




double smartBlend(const geometry_msgs::msg::Pose& prev,
                  const geometry_msgs::msg::Pose& curr,
                  const geometry_msgs::msg::Pose& next,
                  bool is_last)
{
    if (is_last)
        return 0.0;

    Eigen::Vector3d p(prev.position.x, prev.position.y, prev.position.z);
    Eigen::Vector3d c(curr.position.x, curr.position.y, curr.position.z);
    Eigen::Vector3d n(next.position.x, next.position.y, next.position.z);

    const double d_prev = (c - p).norm();
    const double d_next = (n - c).norm();

    // bottleneck distance
    const double d = std::min(d_prev, d_next);

    // ---- Pilz-safe limits ----
    constexpr double MAX_BLEND = 0.5;
    constexpr double SAFETY_FRACTION = 0.45;  // < 0.5 two consequent blends cannot overlap

    // hard upper bound to avoid overlap
    const double max_allowed = SAFETY_FRACTION * d; // 

    // scale inside safe region
    double blend = 0.75 * d;   // your original intent
    blend = std::min(blend, max_allowed);
    blend = std::min(blend, MAX_BLEND);

    // numerical safety
    if (blend < 1e-6)
        return 0.0;

    return blend;
}


// CONSTRAINTS

// Construct orientation constraint for the gripper orientation 
// (applied to every point of the trajectory)
moveit_msgs::msg::OrientationConstraint getOrientationConstraint()
{
  moveit_msgs::msg::OrientationConstraint ocm;
  ocm.link_name = "link_6"; // Gripper
  ocm.header.frame_id = "base";
  ocm.orientation.x = 0.0;
  ocm.orientation.y = 0.707;
  ocm.orientation.z = 0.0;
  ocm.orientation.w = 0.707;
  ocm.absolute_x_axis_tolerance = 3;
  ocm.absolute_y_axis_tolerance = 0.786*2; // 90 (so start state works for sidewall and stern)
  // if deploy is close to placing pose allow more tolerance 
  ocm.absolute_z_axis_tolerance = 3; // 45
  ocm.weight = 1.0;
  // ocm.parameterization;
  return ocm;
}


// Construct a planning-area constraint to reduce the search-space
moveit_msgs::msg::PositionConstraint getAllowedAreaConstraint()
{
  moveit_msgs::msg::PositionConstraint box_constraint;
  box_constraint.link_name = "link_6";
  box_constraint.header.frame_id = "base";

  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = { 2.5, 5.0, 2.5 };
  box_constraint.constraint_region.primitives.emplace_back(box);

  geometry_msgs::msg::Pose box_pose;
  box_pose.position.x = 1.0;
  box_pose.position.y = 0.0;
  box_pose.position.z = 1.0;
  box_constraint.constraint_region.primitive_poses.emplace_back(box_pose);
  box_constraint.weight = 1.0;

  return box_constraint;
}


// apply collision objects to the scene (Unideal robotic cell)
void addCollisionEnvironment(
    const std::string& base_link,
    moveit::planning_interface::PlanningSceneInterface& psi
)
{
  shape_msgs::msg::Mesh mesh_msg;

  // ---------- AIRBUS PROJECT BOX ----------
  {
    moveit_msgs::msg::CollisionObject obj;
    obj.header.frame_id = base_link;
    obj.id = "airbus_project_box";

    // Create the box primitive
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 3.600;
    primitive.dimensions[primitive.BOX_Y] = 0.680;
    primitive.dimensions[primitive.BOX_Z] = 1.450;

    // Compute pose (center of the box)
    geometry_msgs::msg::Pose P;
    P.orientation.w = 1.0;
    P.position.x = 1.250;
    P.position.y = 1.490;
    P.position.z = 0.325;

    obj.primitives.push_back(primitive);
    obj.primitive_poses.push_back(P);
    obj.operation = obj.ADD;

    if (psi.applyCollisionObject(obj))
      RCLCPP_INFO(rclcpp::get_logger("Collision Environment"), "Airbus Project box added correctly!");
  }

  // ---------- DESK BOX ----------
  {
    moveit_msgs::msg::CollisionObject obj;
    obj.header.frame_id = base_link;
    obj.id = "desk_box";

    // Create the box primitive
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 2.000;
    primitive.dimensions[primitive.BOX_Y] = 0.800;
    primitive.dimensions[primitive.BOX_Z] = 0.750;

    // Compute pose (center of the box)
    geometry_msgs::msg::Pose P;
    P.orientation.w = 1.0;
    P.position.x = 2.100;
    P.position.y = -0.600;
    P.position.z = -0.025;

    obj.primitives.push_back(primitive);
    obj.primitive_poses.push_back(P);
    obj.operation = obj.ADD;

    if (psi.applyCollisionObject(obj))
      RCLCPP_INFO(rclcpp::get_logger("Collision Environment"), "Desk Box added correctly!");
  }

  // ---------- TABLE BOX ----------
  {
    moveit_msgs::msg::CollisionObject obj;
    obj.header.frame_id = base_link;
    obj.id = "table_box";

    // Create the box primitive
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 3.000;
    primitive.dimensions[primitive.BOX_Y] = 1.100; //0.800;
    primitive.dimensions[primitive.BOX_Z] = 0.45; // 1.150; // 45 cm

    // Compute pose (center of the box)
    geometry_msgs::msg::Pose P;
    P.orientation.w = 1.0;
    P.position.x = 1.600;
    P.position.y = -1.600; // -1.450;
    P.position.z = 0.225; //0.175;

    obj.primitives.push_back(primitive);
    obj.primitive_poses.push_back(P);
    obj.operation = obj.ADD;

    if (psi.applyCollisionObject(obj))
      RCLCPP_INFO(rclcpp::get_logger("Collision Environment"), "Table Box added correctly!");
  }

  // ---------- WOOD BOX ----------
  {
    moveit_msgs::msg::CollisionObject obj;
    obj.header.frame_id = base_link;
    obj.id = "wood_box";

    // Create the box primitive
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 1.000;
    primitive.dimensions[primitive.BOX_Y] = 2.000;
    primitive.dimensions[primitive.BOX_Z] = 1.200;

    // Compute pose (center of the box)
    geometry_msgs::msg::Pose P;
    P.orientation.w = 1.0;
    P.position.x = -1.050;
    P.position.y = -1.800;
    P.position.z = 0.200;

    obj.primitives.push_back(primitive);
    obj.primitive_poses.push_back(P);
    obj.operation = obj.ADD;

    if (psi.applyCollisionObject(obj))
      RCLCPP_INFO(rclcpp::get_logger("Collision Environment"), "Wood Box added correctly!");
  }

  // ---------- ROOF BOX ----------
  {
    moveit_msgs::msg::CollisionObject obj;
    obj.header.frame_id = base_link;
    obj.id = "roof_box";

    // Create the box primitive
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 5.000;
    primitive.dimensions[primitive.BOX_Y] = 5.000;
    primitive.dimensions[primitive.BOX_Z] = 0.200;

    // Compute pose (center of the box)
    geometry_msgs::msg::Pose P;
    P.orientation.w = 1.0;
    P.position.x = 1.500;
    P.position.y = 0.000;
    P.position.z = 3.100;
    obj.primitives.push_back(primitive);
    obj.primitive_poses.push_back(P);
    obj.operation = obj.ADD;

    if (psi.applyCollisionObject(obj))
      RCLCPP_INFO(rclcpp::get_logger("Collision Environment"), "Roof Box added correctly!");
  }

  // ---------- WALL BOX ----------
  {
    moveit_msgs::msg::CollisionObject obj;
    obj.header.frame_id = base_link;
    obj.id = "wall_box";

    // Create the box primitive
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 5.000;
    primitive.dimensions[primitive.BOX_Y] = 0.200;
    primitive.dimensions[primitive.BOX_Z] = 5.000;

    // Compute pose (center of the box)
    geometry_msgs::msg::Pose P;
    P.orientation.w = 1.0;
    P.position.x = 1.500;
    P.position.y = -2.320;
    P.position.z = 2.000;
    obj.primitives.push_back(primitive);
    obj.primitive_poses.push_back(P);
    obj.operation = obj.ADD;

    if (psi.applyCollisionObject(obj))
      RCLCPP_INFO(rclcpp::get_logger("Collision Environment"), "Wall Box added correctly!");

    // ---------- COLORS (RViz) ----------
    moveit_msgs::msg::PlanningScene ps;
    ps.is_diff = true;

    auto make_color = [](const std::string& id,
                        float r, float g, float b, float a) {
      moveit_msgs::msg::ObjectColor c;
      c.id = id;
      c.color.r = r;
      c.color.g = g;
      c.color.b = b;
      c.color.a = a;
      return c;
    };

    // Solid, distinct colors for good visual separation
    ps.object_colors.push_back(make_color("airbus_project_box", 1.0f, 0.0f, 0.0f, 0.95f)); // orange
    ps.object_colors.push_back(make_color("desk_box",          0.0f, 0.6f, 1.0f, 0.95f));  // greenish
    ps.object_colors.push_back(make_color("table_box",         0.0f, 0.0f, 0.9f, 0.95f));  // blue
    ps.object_colors.push_back(make_color("wood_box",          0.6f, 0.3f, 0.1f, 0.95f));  // brown

    // Semi-transparent "structural" obstacles
    ps.object_colors.push_back(make_color("roof_box",          0.8f, 0.8f, 0.8f, 0.25f));  // grey, transparent
    ps.object_colors.push_back(make_color("wall_box",          0.8f, 0.8f, 0.8f, 0.25f));  // grey, transparent

    psi.applyPlanningScene(ps);
  }
}


// apply a collision object for the board for extrinsics calibration
void collisionBoard(
    const geometry_msgs::msg::Pose& board_pose,
    const std::string& base_link,
    moveit::planning_interface::PlanningSceneInterface& psi,
    double object_radius)
{
  // --- Collision board: centered at pose, but shifted down by 5 cm in Z, with Z dimension 10 cm ---
  moveit_msgs::msg::CollisionObject obj;
  obj.header.frame_id = base_link;
  obj.id = "calibration_board";

  shape_msgs::msg::SolidPrimitive prim;
  prim.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  prim.dimensions.resize(2);
  prim.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] = 0.10; // 10 cm height
  prim.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] = object_radius;

  geometry_msgs::msg::Pose box_pose = board_pose; 
  box_pose.position.z -= 0.05;                      // board pose is on the surface

  obj.primitives.push_back(prim);
  obj.primitive_poses.push_back(box_pose);
  obj.operation = moveit_msgs::msg::CollisionObject::ADD;

  // Apply to planning scene
  psi.applyCollisionObject(obj);
}


// create planning scene monitor for collision checks
planning_scene_monitor::PlanningSceneMonitorPtr
    createPlanningSceneMonitor(const rclcpp::Node::SharedPtr& node)
{
  auto psm = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node, "robot_description");
  psm->startSceneMonitor();
  psm->startStateMonitor();

  if (!psm->getStateMonitor()->waitForCurrentState(node->now(), 2.0))
      throw std::runtime_error("No current state");

  psm->requestPlanningSceneState();
  rclcpp::sleep_for(std::chrono::seconds(1));

  return psm;
}


// Check that the joint positions in 'state' satisfy bounds for the given JointModelGroup.
bool checkJointLimits(
    const moveit::core::RobotState& state,
    const moveit::core::JointModelGroup* jmg)
{
  auto logger = rclcpp::get_logger("PoseChecker");

  if (!jmg)
  {
    RCLCPP_ERROR(logger, "JointModelGroup pointer is null!");
    return false;
  }

  // Fast check 
  if (state.satisfiesBounds(jmg))
    return true;

  // Detailed violation report
  RCLCPP_ERROR(logger,
               "Joint values for group '%s' violate joint limits!",
               jmg->getName().c_str());

  const auto& joints = jmg->getActiveJointModels();
  const double tol = 1e-9;

  for (const auto* joint : joints)
  {
    const auto& bounds_vec = joint->getVariableBounds();
    if (bounds_vec.empty())
      continue;

    const auto& bounds = bounds_vec[0];  

    double v = state.getJointPositions(joint)[0];

    bool below = v < bounds.min_position_ - tol;
    bool above = v > bounds.max_position_ + tol;

    if (below || above)
    {
      RCLCPP_ERROR(logger,
                   "  Joint '%s': %f not in [%f, %f]",
                   joint->getName().c_str(),
                   v,
                   bounds.min_position_,
                   bounds.max_position_);
    }
  }

  return false;
}


// Check that a given pose is valid an then if it
// is collision-free in the current planning scene
void poseCheck(
    const planning_scene_monitor::PlanningSceneMonitorPtr& psm,
    const moveit::core::JointModelGroup* joint_model_group,
    const std::string& pose_name)
{
  planning_scene_monitor::LockedPlanningSceneRO scene(psm);
  const auto& model = scene->getRobotModel();

  moveit::core::RobotState test_state(model);
  test_state.setToDefaultValues();
  test_state.update();

  test_state.setJointGroupPositions(joint_model_group, getJointValues(pose_name));

  const bool valid = checkJointLimits(test_state, joint_model_group);
  if (!valid)
    throw std::runtime_error(pose_name + " has invalid joint values.");

  const bool collision =
      scene->isStateColliding(test_state, joint_model_group->getName());
  if (collision)
    throw std::runtime_error(pose_name + " is colliding.");
  else
    RCLCPP_INFO(rclcpp::get_logger("PoseChecker"), "%s is valid and collision-free.", pose_name.c_str());
}


// use TOTG to change the velocity of trajectory
moveit_msgs::msg::RobotTrajectory
respeedTrajectory(const moveit::core::RobotModelConstPtr& robot_model,
                     const std::string& planning_group,
                     const moveit_msgs::msg::RobotTrajectory& input_traj,
                     double vel_scale,
                     double acc_scale)
{
    // Prepare a RobotTrajectory object
    robot_trajectory::RobotTrajectory rt(robot_model, planning_group);

    // We need a RobotState to seed the trajectory
    moveit::core::RobotStatePtr ref_state =
        std::make_shared<moveit::core::RobotState>(robot_model);
    ref_state->setToDefaultValues();

    // Load the input message
    rt.setRobotTrajectoryMsg(*ref_state, input_traj.joint_trajectory);

    // TOTG time parametrization
    trajectory_processing::TimeOptimalTrajectoryGeneration totg(
        0.1,   // path_tolerance
        0.1,   // resample_dt
        0.001  // min_angle_change
    );

    if (!totg.computeTimeStamps(rt, vel_scale, acc_scale))
        throw std::runtime_error("TOTG time parametrization failed");

    // Output
    moveit_msgs::msg::RobotTrajectory out;
    rt.getRobotTrajectoryMsg(out);
    return out;
}


// Encode RobotTrajectory in a json
nlohmann::json encodeTrajectory(
    const moveit_msgs::msg::RobotTrajectory& traj,
    const std::string& trajectory_id)
{
  const auto& jt = traj.joint_trajectory;
  if (jt.joint_names.empty() || jt.points.empty())
    throw std::runtime_error("trajectoryToJson: trajectory is empty");

  nlohmann::json j;

  j["trajectory_ID"] = trajectory_id;
  j["joint_names"] = jt.joint_names;
  j["points"] = nlohmann::json::array();

  for (const auto& p : jt.points) {
    nlohmann::json jp;
    jp["t"] = rclcpp::Duration(p.time_from_start).seconds();
    jp["q"] = p.positions;
    if (!p.velocities.empty())    jp["qd"]  = p.velocities;
    if (!p.accelerations.empty()) jp["qdd"] = p.accelerations;
    j["points"].push_back(std::move(jp));
  }

  return j;
}


// Decode a json to a RobotTrajectory object
static moveit_msgs::msg::RobotTrajectory decodeTrajectory(const std::string& s)
{
  using nlohmann::json;

  const json j = json::parse(s);

  moveit_msgs::msg::RobotTrajectory traj;
  auto& jt = traj.joint_trajectory;

  jt.joint_names = j.at("joint_names").get<std::vector<std::string>>();

  for (const auto& jp : j.at("points"))
  {
    trajectory_msgs::msg::JointTrajectoryPoint p;

    p.positions = jp.at("q").get<std::vector<double>>();
    if (jp.contains("qd"))  p.velocities    = jp.at("qd").get<std::vector<double>>();
    if (jp.contains("qdd")) p.accelerations = jp.at("qdd").get<std::vector<double>>();

    const double t = jp.at("t").get<double>();
    p.time_from_start = rclcpp::Duration::from_seconds(t);

    jt.points.push_back(std::move(p));
  }

  return traj;
}


// Encode R (3x3) and t (3x1) 
inline nlohmann::json encode_tf(const geometry_msgs::msg::TransformStamped& tf)
{
  const auto& tr = tf.transform.translation;
  const auto& q  = tf.transform.rotation;

  // Quaternion -> rotation matrix
  Eigen::Quaterniond qe(q.w, q.x, q.y, q.z);
  Eigen::Matrix3d R = qe.normalized().toRotationMatrix();

  nlohmann::json j;
  j["header"]["frame_id"] = tf.header.frame_id;     // "base_link"
  j["child_frame_id"]     = tf.child_frame_id;      // "link_6"
  j["stamp"]["sec"]       = tf.header.stamp.sec;
  j["stamp"]["nanosec"]   = tf.header.stamp.nanosec;

  j["t"] = { tr.x, tr.y, tr.z };                    // translation (meters)

  j["R"] = nlohmann::json::array();                 // 3x3 rotation matrix (row-major)
  for (int r = 0; r < 3; ++r)
  {
    nlohmann::json row = nlohmann::json::array();
    for (int c = 0; c < 3; ++c) row.push_back(R(r, c));
    j["R"].push_back(row);
  }

  return j;
}


// Importing the AMURA draping cell scene for visualization
void publishRoboticsLabScene(const rclcpp::Node::SharedPtr& node,
                              const std::string& base_link,
                              const moveit::core::RobotModelConstPtr& robot_model)
{
  // Create a new MoveItVisualTools for the permanent scene visualization
  moveit_visual_tools::MoveItVisualTools scene_tools(
      node, base_link, "Scene_Perm_Topic", robot_model);

  rclcpp::Rate rate(2);  // 2 Hz -> 500 ms
  rate.sleep();


  // Mesh resources
  const std::string cell_uri  = "file://" + ws_path +
      "/src/profactor_ros2/biostruct_robotics_lab/meshes/Drapecell.stl";

  // Common pose objects
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  geometry_msgs::msg::Pose p;

  // Draping cell
  p.orientation.w = 1.0;
  p.position.x = -2.0;
  p.position.y = -2.450;
  p.position.z = -0.727;
  T.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
  scene_tools.publishMesh(T, cell_uri, rviz_visual_tools::GREY, 0.001);
  scene_tools.trigger();
  
  rate.sleep();
}


// Blocks until current joint positions are within tol of the trajectory goal.
inline void wait_until_at_goal(
    moveit::planning_interface::MoveGroupInterface& move_group_interface,
    const std::string& planning_group,
    const moveit_msgs::msg::RobotTrajectory& traj,
    double goal_tol = 0.03,                 // rad
    std::chrono::milliseconds poll_period = std::chrono::milliseconds(20),
    double get_state_timeout_s = 0.2)       // seconds
{
  const auto* jmg =
      move_group_interface.getRobotModel()->getJointModelGroup(planning_group);
  if (!jmg) {
    throw std::runtime_error("wait_until_at_goal: JointModelGroup not found: " + planning_group);
  }

  const auto& jt = traj.joint_trajectory;
  if (jt.points.empty()) {
    throw std::runtime_error("wait_until_at_goal: trajectory has no points");
  }
  const auto& q_goal = jt.points.back().positions;

  while (rclcpp::ok())
  {
    auto st = move_group_interface.getCurrentState(get_state_timeout_s);
    if (!st) {
      rclcpp::sleep_for(poll_period);
      continue;
    }

    std::vector<double> q;
    st->copyJointGroupPositions(jmg, q);

    if (q.size() != q_goal.size()) {
      throw std::runtime_error("wait_until_at_goal: size mismatch current vs goal");
    }

    double maxerr = 0.0;
    for (size_t k = 0; k < q.size(); ++k)
      maxerr = std::max(maxerr, std::abs(q[k] - q_goal[k]));

    if (maxerr <= goal_tol) return;

    rclcpp::sleep_for(poll_period);
  }

  throw std::runtime_error("wait_until_at_goal: interrupted (rclcpp::ok() == false)");
}


// Blocks until joint positions stop changing (i.e. robot is no longer moving).
inline void wait_until_joints_stopped(
    moveit::planning_interface::MoveGroupInterface& move_group_interface,
    const std::string& planning_group,
    double pos_tol = 1e-4,                  // rad
    std::chrono::milliseconds poll_period = std::chrono::milliseconds(20),
    double get_state_timeout_s = 0.2)       // seconds
{
  const auto* jmg =
      move_group_interface.getRobotModel()->getJointModelGroup(planning_group);
  if (!jmg) {
    throw std::runtime_error(
        "wait_until_joints_stopped: JointModelGroup not found: " + planning_group);
  }

  // Get initial joint positions
  auto st_prev = move_group_interface.getCurrentState(get_state_timeout_s);
  if (!st_prev)
    throw std::runtime_error("wait_until_joints_stopped: failed to get initial state");

  std::vector<double> q_prev;
  st_prev->copyJointGroupPositions(jmg, q_prev);

  while (rclcpp::ok())
  {
    rclcpp::sleep_for(poll_period);

    auto st = move_group_interface.getCurrentState(get_state_timeout_s);
    if (!st)
      continue;

    std::vector<double> q;
    st->copyJointGroupPositions(jmg, q);

    if (q.size() != q_prev.size()) {
      throw std::runtime_error(
          "wait_until_joints_stopped: size mismatch current vs previous");
    }

    double maxdiff = 0.0;
    for (size_t k = 0; k < q.size(); ++k)
      maxdiff = std::max(maxdiff, std::abs(q[k] - q_prev[k]));

    if (maxdiff <= pos_tol)
      return;  // joints no longer changing → stopped

    q_prev = q;  // still moving → update reference
  }

  throw std::runtime_error(
      "wait_until_joints_stopped: interrupted (rclcpp::ok() == false)");
}
