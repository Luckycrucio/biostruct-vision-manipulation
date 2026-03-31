// Luca Grigolin at PROFACTOR GmbH

// PLANNER node

#include <memory>
#include <thread>
#include <mutex>
#include <atomic>
#include <optional>
#include <condition_variable>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// Std messages
#include <std_msgs/msg/string.hpp>

// MoveIt – planning, scenes, kinematics
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>

// MoveIt – visual tools
#include <moveit_visual_tools/moveit_visual_tools.h>

// MoveIt – trajectory sequence service
#include <moveit_msgs/srv/get_motion_sequence.hpp>
#include <moveit_msgs/action/move_group_sequence.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

// Collision shapes
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>

// Other external helpers
#include "biostruct_amura/helpers.h"

// JSON
#include <nlohmann/json.hpp>

// TF2 for RPY → Quaternion
#include <tf2/LinearMath/Quaternion.h>

// File writing
#include <fstream>

// Tf transforms 
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/persistence.hpp>



// Aliases and Constants
namespace rvt = rviz_visual_tools;
static const std::string NODE_NAME = "planning_node";
static const std::string PLANNING_GROUP = "arm"; // must match MoveIt SRDF group
static const std::string LINK_BASE_NAME = "base";
static const std::string LINK_TOOL_NAME = "link_6"; // end effector link
using moveit::planning_interface::PlanningSceneInterface;
std::filesystem::path source_path = __FILE__;
auto project_dir = source_path.parent_path();

// Create a ROS LOGGER for the planning node
static auto const LOGGER = rclcpp::get_logger(NODE_NAME);
// Create a ROS LOGGER for MQTT communication
auto mqtt_logger = rclcpp::get_logger("MQTT_Planner");

int main(int argc, char * argv[])
{
    //                          [Node initialization]

    // Initialize ROS 2 
    rclcpp::init(argc, argv);
    {
        // This let u pass params without declaring them
        // (best practice would be to declare parameters in the corresponding classes
        // and provide descriptions about expected use)
        rclcpp::NodeOptions node_options;
        node_options.automatically_declare_parameters_from_overrides(true);
        // Create the Node
        auto const node = std::make_shared<rclcpp::Node>(NODE_NAME, node_options);
        // Spin up a separate executor for the node so MoveItVisualTools, prompts and 
        // service clients can work while the main thread does the planning/executing
        rclcpp::executors::SingleThreadedExecutor executor; // MultiThreadedExecutor for more complex
        executor.add_node(node);
        auto spinner = std::thread([&executor]() { executor.spin(); });

        //                          [MoveIt initialization]

        // Create the MoveIt MoveGroup Interface for one group
        using moveit::planning_interface::MoveGroupInterface;
        auto move_group_interface = MoveGroupInterface(node, PLANNING_GROUP);
        // Create the MoveIt PlanningScene Interface to add/remove collision objects
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        // Kinematic Model Pointers (group and tool link) for the visualization and conversions
        auto robot_model = move_group_interface.getRobotModel();
        auto joint_model_group = robot_model->getJointModelGroup(move_group_interface.getName());
        auto link_model = robot_model->getLinkModel(LINK_TOOL_NAME); 
        auto tf_buffer   = std::make_shared<tf2_ros::Buffer>(node->get_clock());
        auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

        // ----------------- Shared state for MQTT messages -----------------

        std::mutex mtx; // execution block (to lock)
        std::condition_variable cv;  // condition variable for mqtt synchronization
        // storing messages structures
        std::optional<std_msgs::msg::String> picking_pose_msg; 
        std::optional<std_msgs::msg::String> plan_msg; 
        std::optional<std_msgs::msg::String> term_msg; 
        std::optional<std_msgs::msg::String> calib_msg; 
        std::optional<std_msgs::msg::String> calib_done_vision_msg;
        std::optional<std_msgs::msg::String> to_vision_msg;
        std::optional<std_msgs::msg::String> save_pose_msg;

        // ----------------- Subscriptions for MQTT topics -----------------

        auto vision_sub = node->create_subscription<std_msgs::msg::String>(
            "/picking_pose", 10,
            [&](const std_msgs::msg::String::SharedPtr msg)
            {
                {
                    std::lock_guard<std::mutex> lock(mtx);
                    picking_pose_msg = *msg;   // store first (or latest) message
                }
                cv.notify_one();
            });

        auto plan_sub = node->create_subscription<std_msgs::msg::String>(
            "/plan", 10,
            [&](const std_msgs::msg::String::SharedPtr msg)
            {
                {
                    std::lock_guard<std::mutex> lock(mtx);
                    plan_msg = *msg;
                }
                cv.notify_one();
            });
        
        auto termination_sub = node->create_subscription<std_msgs::msg::String>(
        "/termination", 10,
        [&](const std_msgs::msg::String::SharedPtr msg)
        {
            {
                std::lock_guard<std::mutex> lock(mtx);
                term_msg = *msg;
            }
            cv.notify_one();
        });

        auto calibration_sub = node->create_subscription<std_msgs::msg::String>(
        "/calibration_trigger", 10,
        [&](const std_msgs::msg::String::SharedPtr msg)
        {
            {
                std::lock_guard<std::mutex> lock(mtx);
                calib_msg = *msg;
            }
            cv.notify_one();
        });

        auto done_vision_sub = node->create_subscription<std_msgs::msg::String>(
        "/calib_done_vision", 10,
        [&](const std_msgs::msg::String::SharedPtr msg)
        {
            {
                std::lock_guard<std::mutex> lock(mtx);
                calib_done_vision_msg = *msg;
            }
            cv.notify_one();
        });

        auto to_vision_sub = node->create_subscription<std_msgs::msg::String>(
        "/to_vision", 10,
        [&](const std_msgs::msg::String::SharedPtr msg)
        {
            {
                std::lock_guard<std::mutex> lock(mtx);
                to_vision_msg = *msg;
            }
            cv.notify_one();
        });

        auto save_pose_sub = node->create_subscription<std_msgs::msg::String>(
        "/save_pose", 10,
        [&](const std_msgs::msg::String::SharedPtr msg)
        {
            {
                std::lock_guard<std::mutex> lock(mtx);
                save_pose_msg = *msg;
            }
            cv.notify_one();
        });



        // ----------------- Publishers for MQTT topic -----------------

        auto ready_pub =
            node->create_publisher<std_msgs::msg::String>("/ready", 10);
        if (!ready_pub) {
            RCLCPP_ERROR(mqtt_logger, "Planner Ready Publisher is null.");
        }

        auto traj_pub =
            node->create_publisher<std_msgs::msg::String>("/trajectories", 10);
        if (!traj_pub) {
            RCLCPP_ERROR(mqtt_logger, "Trajectory Publisher is null.");
        }

        auto tcp_transf_pub =
            node->create_publisher<std_msgs::msg::String>("/tcp_transform", 10);
        if (!tcp_transf_pub) {
            RCLCPP_ERROR(mqtt_logger, "TCP Transformation Publisher is null.");
        }

        auto est_ext_pub =
            node->create_publisher<std_msgs::msg::String>("/estimate_ext", 10);
        if (!est_ext_pub) {
            RCLCPP_ERROR(mqtt_logger, "Estimate Extrinsics Publisher is null.");
        }


        //                    [RViz-MoveItVisualTools initialization]

        // MoveItVisualTools setup
        const std::string RVT_TOPIC = "rvt_Dyn_Topic"; // name of the topic
        // Construct and initialize MoveItVisualTools
        moveit_visual_tools::MoveItVisualTools moveit_visual_tools(node, 
                                                                    LINK_BASE_NAME, 
                                                                    RVT_TOPIC,
                                                                    move_group_interface.getRobotModel()                                                                                        
        );
        // Create a new MoveItVisualTools for the semi permanent scene visualization
        moveit_visual_tools::MoveItVisualTools semi_perm_scene_tools(
            node, LINK_BASE_NAME, "Semi_Dyn_Topic", robot_model);
        moveit_visual_tools.deleteAllMarkers(); // clear RViz previous markers
        moveit_visual_tools.loadRemoteControl(); // enable RViz "NEXT" button panel

        //                                  [Local Helpers]
    
        // HELPERS - VISUALIZATION

        // renders a big title at z=3
        auto const draw_title = [&moveit_visual_tools](const std::string& text) {
            Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
            text_pose.translation().z() = 3.7;

            // publish with a manually set scale (meters)
            geometry_msgs::msg::Vector3 scale;
            scale.x = 0.05;  
            scale.y = 0.05;  
            scale.z = 0.2;  
            moveit_visual_tools.publishText(
                text_pose, text, rvt::WHITE, scale, /*static_id=*/false);
        };

        // Blocks until u click Next and show instructions on prompt (step by step demos)
        auto const prompt = [&moveit_visual_tools](auto text) {
            moveit_visual_tools.prompt(text);
        };

        // Draw the planned motion path traced by the gripper
        auto const draw_trajectory_tool_path =
            [ &moveit_visual_tools,
                jmg = joint_model_group,
                lm = link_model
            ]
            (auto const trajectory, auto const color = rvt::LIME_GREEN) {
                moveit_visual_tools.publishTrajectoryLine(trajectory, lm, jmg, color);
        };

        // shows labelled axes and name for visualized poses
        auto const draw_poses = [&](const std::vector<std::pair<std::string, geometry_msgs::msg::Pose>>& poses)
        {
            for(const auto& [name, p] : poses)
            {
            moveit_visual_tools.publishAxisLabeled(p, name);
            }
            moveit_visual_tools.trigger(); // flush the marker batch
        };

        // shows labelled axes and name for visualized poses
        auto const semi_perm_draw_poses = [&](const std::vector<std::pair<std::string, geometry_msgs::msg::Pose>>& poses)
        {
            for(const auto& [name, p] : poses)
            {
            semi_perm_scene_tools.publishAxisLabeled(p, name);
            }
            semi_perm_scene_tools.trigger(); // flush the marker batch
        };

        // - Planning Wrapper (success/fail) for move_group_interface
        auto request_plan = [&move_group_interface]() {
            moveit::planning_interface::MoveGroupInterface::Plan msg;
            auto const ok = static_cast<bool>(move_group_interface.plan(msg));
            if(!ok){
            RCLCPP_ERROR(LOGGER, "Planning failed!");
            }
            return std::make_pair(ok, msg);
        };


        // ---------------------- INITIALIZE SCENE -------------------------------

        //prompt("Press 'Next' in the RVizVisualToolsGui window to load the Amura-Automatic-Draping scene");

        const std::string title = "AMURA_CELL";
        draw_title(title);
        moveit_visual_tools.trigger();

        // SHOW the Visuals for the scene
        publishAmuraDrapingScene(node,
                                    LINK_BASE_NAME,
                                    robot_model);

        // ADD Collision Objects
        moveit::planning_interface::PlanningSceneInterface psi;
        addCollisionEnvironment(LINK_BASE_NAME);
        auto psm = createPlanningSceneMonitor(node);

        // Initialize Planner for move_group_interface
        move_group_interface.setPlanningPipelineId("pilz_industrial_motion_planner");
        move_group_interface.setPlannerId("PTP");
        move_group_interface.setWorkspace(-5, -5, -5, 5, 5, 5);
        move_group_interface.setPlanningTime(10.0);
        move_group_interface.setNumPlanningAttempts(12);

        // define once the picking pose
        geometry_msgs::msg::PoseStamped picking_pose_tool;
        geometry_msgs::msg::PoseStamped pickingPose;   // expressed in base_link
        tf2::Quaternion q;
        // fixed frame setup
        picking_pose_tool.header.frame_id = LINK_TOOL_NAME;
        picking_pose_tool.header.stamp = rclcpp::Time(0);
        // define once the camera pose 
        geometry_msgs::msg::PoseStamped camera_pose;
        
        // ply id string for saving robot pose function
        std::string ply_id_2save;

        // pkg directory
        std::string pkg_share =
        ament_index_cpp::get_package_share_directory("biostruct_amura");
        std::filesystem::path ws_folder = std::filesystem::path(pkg_share)
                                            .parent_path()  // biostruct_amura
                                            .parent_path()  // share
                                            .parent_path()  // install
                                            .parent_path(); // abb_ws
        std::string memory_yaml_path = ws_folder.string() +
                            "/src/profactor_ros2/biostruct_amura/memory/memory.yaml";;
        // Load memory 
        loadMemory(memory_yaml_path);
            
        // robot might be moving (unfinished motion from previous activity)
        wait_until_joints_stopped(move_group_interface, PLANNING_GROUP);

        //                                    [Planner Ready]

        // publish the ready message to the coordinator
        try
        {
            std_msgs::msg::String out;
            ready_pub->publish(out);
            RCLCPP_INFO(LOGGER, "Planner fully initialized.");
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_ERROR(mqtt_logger, "Failed to publish planner ready message: %s", ex.what());
        }

        // =====================================================================
        //         >>>>>>>>>>>>>>>>  MAIN PROCESSING LOOP  <<<<<<<<<<<<<<
        // =====================================================================


        while (true)
        {

            // ----------------- Wait until plan message or termination is published -----------------

            {
                std::unique_lock<std::mutex> lock(mtx);
                RCLCPP_INFO(mqtt_logger, "Waiting for a command ...");
                cv.wait(lock, [&]{
                    return plan_msg.has_value() || term_msg.has_value() ||
                             calib_msg.has_value() || to_vision_msg.has_value() || save_pose_msg.has_value();

                });
            }

            if (term_msg.has_value()){

                // ---------------- Termination ----------------
                
                RCLCPP_INFO(mqtt_logger, "Termination command received, shutting down Planner.");
                break;
            }

            if(to_vision_msg.has_value()){

                // ---------------- Plan To Vision Pose ----------------

                // retrieve joint values from the current state
                const moveit::core::RobotStateConstPtr start_state = move_group_interface.getCurrentState();
                const std::vector<double> start_joint_values = getJointValuesFromState(start_state, PLANNING_GROUP);

                // compute joint values for the goal state
                const std::vector<double> target_joint_values = getJointValues("visionPose");

                // configure start and goal for the PTP planner
                move_group_interface.setStartStateToCurrentState(); // Use the freshest monitored state as start
                move_group_interface.setJointValueTarget(target_joint_values);

                // VISUALIZE - pre plan
                draw_poses({{"StartPose", getPoseFromJointValues(start_joint_values, robot_model, PLANNING_GROUP, LINK_TOOL_NAME).pose},
                            {"visionPose", getPoseFromJointValues(target_joint_values, robot_model, PLANNING_GROUP, LINK_TOOL_NAME).pose}});
                moveit_visual_tools.trigger();

                // PLAN
                auto [success, plan] = request_plan();

                // VISUALIZE - post plan
                if (success)
                {
                    // draws the planned trajectory
                    draw_trajectory_tool_path(plan.trajectory, rvt::ORANGE);
                    moveit_visual_tools.trigger();
                }

                // reset for next plan 
                to_vision_msg.reset();

                // Build JSON
                auto j = encodeTrajectory(plan.trajectory, "To Vision Pose");

                // Publish

                std_msgs::msg::String msg;
                msg.data = j.dump();           // compact, fastest

                try {
                    traj_pub->publish(msg);
                    RCLCPP_INFO(mqtt_logger,
                                "Trajectory JSON published (%zu bytes)", msg.data.size());
                }
                catch (const std::exception& e) {
                    RCLCPP_ERROR(mqtt_logger,
                                "Failed to publish trajectory JSON: %s", e.what());
                }

                RCLCPP_INFO(LOGGER, "Plan to Vision Pose completed. Planner ready for next command.");

            }

            if(save_pose_msg.has_value()){

                ply_id_2save = save_pose_msg->data;
                auto st = move_group_interface.getCurrentState(0.2);
                if (!st)
                {
                    RCLCPP_WARN(LOGGER, "save_pose: getCurrentState() returned null");
                }
                else
                {
                    std::vector<double> joints;
                    st->copyJointGroupPositions(joint_model_group, joints);
                    // save angles in degrees
                    joints = rad2deg(joints);
                    // 3) write first 6 joint values into YAML under this id
                    Pose6 v6{joints[0], joints[1], joints[2], joints[3], joints[4], joints[5]};
                    write_to_memory(memory_yaml_path, ply_id_2save, v6);
                    RCLCPP_INFO(LOGGER, "save_pose: stored '%s' -> [%.6f %.6f %.6f %.6f %.6f %.6f]",
                                ply_id_2save.c_str(),
                                v6[0], v6[1], v6[2], v6[3], v6[4], v6[5]);
                }

                // reload memory to update poses 
                loadMemory(memory_yaml_path);

                // publish the ready message to the coordinator
                try
                {
                    std_msgs::msg::String out;
                    ready_pub->publish(out);
                }
                catch (const tf2::TransformException& ex)
                {
                    RCLCPP_ERROR(mqtt_logger, "Failed to publish ready message: %s", ex.what());
                }

                save_pose_msg.reset();
            }

            if (calib_msg.has_value()){
                
                // ---------------- Extrinsic Calibration  ----------------

                move_group_interface.setEndEffectorLink("tool_camera");
                std::random_device rd; // random seed
                std::mt19937 rng(rd());
                geometry_msgs::msg::Pose board_pose;
                // Position (meters, in base_link)
                board_pose.position.x = 1.783; // meters
                board_pose.position.y = 0.0;
                board_pose.position.z = 0.71; // meters
                // Orientation: identity (no rotation w.r.t. base_link)
                board_pose.orientation.x = 0.0;
                board_pose.orientation.y = 0.0;
                board_pose.orientation.z = 0.0;
                board_pose.orientation.w = 1.0;
                draw_poses({
                        {"BoardPose",   board_pose}
                });
                moveit_visual_tools.trigger();
                
                // Calibration Sphere 
                // (measure board from center to corner to enclose it in a sphere)
                double board_radius = 0.22; // meters


                // add collision sphere and visualize
                collisionBoard(
                    board_pose, 
                    LINK_BASE_NAME,
                    psi,
                    board_radius
                );

                // Number of samples
                constexpr int N_SAMPLES = 100;

                for (int i = 0; i < N_SAMPLES; ++i)
                {
                    geometry_msgs::msg::Pose sampled_pose =
                        sample_pose_on_cap_look_center(moveit_visual_tools,
                                    board_pose, 
                                    board_radius, 
                                    rng
                        );

                    // visualize poses
                    draw_poses({
                        {"BoardPose",   board_pose},
                        {"SampledPose_" + std::to_string(i), sampled_pose}
                    });
                    moveit_visual_tools.trigger();

                    // Move Robot to that pose
                    
                    // configure start and goal for the PTP planner
                    move_group_interface.setStartStateToCurrentState(); // Use the freshest monitored state as start
                    move_group_interface.setPoseTarget(sampled_pose); // Target in cartesian space

                    // PLAN
                    auto [success, plan] = request_plan();  
                    if(!success){
                        RCLCPP_WARN(LOGGER, "Pilz PTP planning failed.");
                        moveit_visual_tools.deleteAllMarkers();
                        i = i-1; // sample another pose
                        continue; // <-- go to next iteration
                    }         
                    // VISUALIZE - post plan
                    else
                    {
                        // if trajectory is too complicated, get rid of it 
                        const auto& jt = plan.trajectory.joint_trajectory;
                        const double duration_s = jt.points.empty() ? 0.0
                                            : rclcpp::Duration(jt.points.back().time_from_start).seconds();

                        if (duration_s > 6.0)
                        {
                            moveit_visual_tools.deleteAllMarkers();
                            i = i - 1;
                            continue;
                        }
                        // draws the planned trajectory 
                        draw_trajectory_tool_path(plan.trajectory, rvt::ORANGE);
                        moveit_visual_tools.trigger();
                    }

                    // EXECUTE 
                    // The planner here executes only because we are in calibration mode
                    // and it has been commanded by the coordinator
                    const auto exec_result = move_group_interface.execute(plan);

                    // Wait until robot is at goal
                    wait_until_at_goal(move_group_interface, PLANNING_GROUP, plan.trajectory, 0.001);

                    if (exec_result != moveit::core::MoveItErrorCode::SUCCESS)
                    {
                        RCLCPP_WARN(LOGGER, "Pilz PTP execution failed with code: %d", exec_result.val);
                        moveit_visual_tools.deleteAllMarkers();
                        i = i-1; // sample another pose
                        continue; // <-- go to next iteration
                    }
                    else
                    {
                        // publish current transformation from TCP to Base 
                        try
                        {
                            rclcpp::sleep_for(std::chrono::seconds(1));  // small sleep
                            // lookupTransform(target_frame, source_frame, time)
                            const auto tf =
                                tf_buffer->lookupTransform(LINK_BASE_NAME,
                                                        LINK_TOOL_NAME,
                                                        tf2::TimePointZero);

                            std_msgs::msg::String out;
                            out.data = encode_tf(tf).dump();
                            tcp_transf_pub->publish(out);
                        }
                        catch (const tf2::TransformException& ex)
                        {
                            RCLCPP_ERROR(mqtt_logger,
                                        "TF lookup failed (%s <- %s): %s",
                                        LINK_BASE_NAME.c_str(),
                                        LINK_TOOL_NAME.c_str(),
                                        ex.what());
                        }
                    }
    
                    // waits for vision to be done 
                    // ----------------- Wait until plan message or termination is published -----------------
                    {
                        std::unique_lock<std::mutex> lock(mtx);
                        RCLCPP_INFO(mqtt_logger, "Waiting for vision to be done ...");
                        cv.wait(lock, [&]{
                            return calib_done_vision_msg;

                        });
                    }
                    calib_done_vision_msg.reset();

                    moveit_visual_tools.deleteAllMarkers(); // clear RViz previous markers
                }


                // publish the Final Extrinsics Estimation Trigger
                std_msgs::msg::String msg;
                try {
                    est_ext_pub->publish(msg);
                    RCLCPP_INFO(mqtt_logger,
                                "Extrinsics Estimation Trigger published succesfully.");
                }
                catch (const std::exception& e) {
                    RCLCPP_ERROR(mqtt_logger,
                                "Failed to publish Extrinsics Estimation Trigger: %s", e.what());
                }

                // reset for next calibration 
                calib_msg.reset();
                // remove the board from the planning scene
                psi.removeCollisionObjects(std::vector<std::string>{"calibration_board"});
                // reset tool link 
                move_group_interface.setEndEffectorLink(LINK_TOOL_NAME);

                RCLCPP_INFO(LOGGER, "Extrinsic Calibration completed. Planner ready for next command.");
            }

            if (plan_msg.has_value()){

                // ---------------- Plan Trajectory ----------------

                // Decode plan 
                plan pl;  
                try {
                    json j = json::parse(plan_msg->data);   // plan_msg contains your JSON string
                    pl = j.get<plan>();                     // decode into struct
                    // Print plan
                    RCLCPP_INFO(mqtt_logger, "[New Plan]:");
                    RCLCPP_INFO(mqtt_logger, "Ply ID: %s", pl.ply_id.c_str());
                    //RCLCPP_INFO(mqtt_logger, "Path ID: %s", pl.path_id.c_str());
                    for (const auto& s : pl.path) {
                        RCLCPP_INFO(mqtt_logger, "Path entry: %s", s.c_str());
                    }
                    RCLCPP_INFO(mqtt_logger, "Travel Height: %f", pl.travel_height);
                    RCLCPP_INFO(mqtt_logger, "Approach: %f", pl.approach);
                    RCLCPP_INFO(mqtt_logger, "Speed: %f", pl.speed);
                }
                catch (const std::exception& e) {
                    RCLCPP_ERROR(mqtt_logger, "Error decoding plan: %s", e.what());
                }

                // local variables from plan
                std::string ply_id = pl.ply_id;
                //std::string path_id = pl.path_id;
                std::vector<std::string> pose_names = pl.path;
                double travel_height = pl.travel_height; // height of travel waypoints 
                double approach = pl.approach; // approach distance for placing operation
                double speed = pl.speed; // trajectory speed m/s

                if (picking_pose_msg.has_value())
                {
                    // Load the camera pose (estimated from estrinsics)

                    // Load rotation + translation from YAML
                    const std::string yaml_path = ws_folder.parent_path().string() + 
                                "/vision_ws/vision_app/calib/extrinsics/T_cam2tcp_10_00_29__03022026.yaml";

                    cv::FileStorage fs(yaml_path, cv::FileStorage::READ);
                    if (!fs.isOpened())
                    {
                        RCLCPP_ERROR(node->get_logger(), "Failed to open YAML file: %s", yaml_path.c_str());
                        return 1;
                    }

                    cv::Mat R_cv, t_cv;
                    fs["R"] >> R_cv;
                    fs["t"] >> t_cv;
                    fs.release();

                    // OpenCV -> Eigen
                    Eigen::Matrix3d R;
                    Eigen::Vector3d t;

                    for (int i = 0; i < 3; ++i)
                    {
                        for (int j = 0; j < 3; ++j)
                            R(i, j) = R_cv.at<double>(i, j);

                        t(i) = t_cv.at<double>(i, 0);
                    }

                    // Rotation matrix -> quaternion
                    Eigen::Quaterniond qc(R);
                    qc.normalize();

                    // PoseStamped instead of TF
                    camera_pose.header.stamp = node->now();
                    camera_pose.header.frame_id = "link_6";   // pose is expressed in link_6

                    camera_pose.pose.position.x = t.x();
                    camera_pose.pose.position.y = t.y();
                    camera_pose.pose.position.z = t.z();

                    camera_pose.pose.orientation.w = qc.w();
                    camera_pose.pose.orientation.x = qc.x();
                    camera_pose.pose.orientation.y = qc.y();
                    camera_pose.pose.orientation.z = qc.z();


                    // ---------------- Decode picking pose ----------------

                    try
                    {
                        json j = json::parse(picking_pose_msg->data);
                        PickingPose pose = j.get<PickingPose>();

                        // Fill Pose (wrt TCP)
                        picking_pose_tool.pose.position.x = pose.x;
                        picking_pose_tool.pose.position.y = pose.y;
                        picking_pose_tool.pose.position.z = pose.z;

                        // Convert Euler (rx, ry, rz) to quaternion
                        q.setRPY(pose.rx, pose.ry, pose.rz);
                        q.normalize();

                        picking_pose_tool.pose.orientation.x = q.x();
                        picking_pose_tool.pose.orientation.y = q.y();
                        picking_pose_tool.pose.orientation.z = q.z();
                        picking_pose_tool.pose.orientation.w = q.w();

                        RCLCPP_INFO(
                            mqtt_logger,
                            "Decoded picking_pose wrt TCP: x=%.3f y=%.3f z=%.3f  rx=%.3f ry=%.3f rz=%.3f",
                            pose.x, pose.y, pose.z, pose.rx, pose.ry, pose.rz
                        );
                    }
                    catch (const std::exception& e)
                    {
                        RCLCPP_ERROR(mqtt_logger, "Error decoding picking_pose wrt TCP: %s", e.what());
                    }

                    // transform from TCP to base
                    try
                    {
                        tf_buffer->transform(
                            picking_pose_tool,
                            pickingPose,              // now expressed in base_link
                            LINK_BASE_NAME,            // "base_link"
                            tf2::durationFromSec(0.05)
                        );
                        tf_buffer->transform(
                            camera_pose,
                            camera_pose,              // now expressed in base_link
                            LINK_BASE_NAME,            // "base_link"
                            tf2::durationFromSec(0.05)
                        );
                    }
                    catch (const tf2::TransformException& ex)
                    {
                        RCLCPP_ERROR(LOGGER, "TF transform failed: %s", ex.what());
                    }

                    // draw pose for debug
                    semi_perm_draw_poses({
                        {"DetectedPose", pickingPose.pose},
                        {"CameraPose", camera_pose.pose}
                    });
                    semi_perm_scene_tools.trigger();

                    // update the memory with the estimated picking pose [UNCOMMENT FOR REAL HARDWARE]

                    //setJointValuesFromPose("RefPickingPose", pickingPose,  "PickingPose",robot_model, PLANNING_GROUP, LINK_TOOL_NAME);

                    // project the picking and placing poses
                    projectWaypoints(ply_id, travel_height, approach, robot_model, PLANNING_GROUP, LINK_TOOL_NAME);

                    RCLCPP_INFO(LOGGER, "Picking Pose projected succesfully!");

                    // Check all the poses for the ply routine for collisions and joint limits
                    poseCheck(psm, joint_model_group, "PickingPose");
                    poseCheck(psm, joint_model_group, "refPlacingPose");
                    poseCheck(psm, joint_model_group, ply_id+"PlacingPose");
                    poseCheck(psm, joint_model_group, "TableTopPose");
                    poseCheck(psm, joint_model_group, "MouldTopPose");
                    poseCheck(psm, joint_model_group, ply_id+"TopPose");

                    // reset for next picking pose 
                    picking_pose_msg.reset();
                }

                // create a joint poses environment for the plan
                std::vector<std::pair<std::string, geometry_msgs::msg::Pose>> waypoint_poses;

                // create a list of joint poses for the plan
                waypoint_poses.reserve(pose_names.size());
                for (const auto& s : pose_names) {
                    waypoint_poses.push_back(
                        std::make_pair(s, getPoseFromJointValues(getJointValues(s), robot_model, PLANNING_GROUP, LINK_TOOL_NAME).pose));
                }

                // Draw the poses
                semi_perm_draw_poses(waypoint_poses);


                //                                [PLAN SEQUENCE PATH]

                // Service client (plan_sequence_path)
                using GetMotionSequence = moveit_msgs::srv::GetMotionSequence;
                auto service_client = node->create_client<GetMotionSequence>("/plan_sequence_path");
                while (!service_client->wait_for_service(std::chrono::seconds(10)))
                {
                    RCLCPP_WARN(LOGGER, "Waiting for service /plan_sequence_path to be available...");
                }

                // -------------------- PRE-VISUALIZATION --------------------

                std::string description = ply_id;
                draw_title(description);
                // draw_poses(waypoint_poses);
                // moveit_visual_tools.trigger();

                // -------------------- BUILD MotionSequenceRequest --------------------
                moveit_msgs::msg::MotionSequenceRequest seq_req;
                seq_req.items.reserve(pose_names.size());

                for (size_t i = 1; i < pose_names.size(); ++i)
                {
                    const auto curr = getPose(pose_names[i], robot_model, PLANNING_GROUP, LINK_TOOL_NAME);
                    moveit_msgs::msg::MotionSequenceItem item;

                    auto prev = getPose(pose_names[i - 1], robot_model, PLANNING_GROUP, LINK_TOOL_NAME);
                    bool last = (i + 1 == pose_names.size());
                    geometry_msgs::msg::PoseStamped next;
                    if (!last)
                    next = getPose(pose_names[i + 1], robot_model, PLANNING_GROUP, LINK_TOOL_NAME);

                    item.blend_radius = smartBlend(prev.pose, curr.pose, next.pose, last);

                    auto& r = item.req;
                    // for the first segment, set the start state (the others are implicitly chained)
                    if (i == 1) {
                        // This forces the planner to stick with the robot current configuration
                        moveit::core::RobotState start_state = *move_group_interface.getCurrentState();
                        start_state.update();
                        moveit_msgs::msg::RobotState start_state_msg;
                        moveit::core::robotStateToRobotStateMsg(start_state, start_state_msg);
                        r.start_state = start_state_msg;
                    }
                    r.group_name = PLANNING_GROUP;
                    r.pipeline_id = "pilz_industrial_motion_planner";
                    r.planner_id  = "LIN";
                    r.allowed_planning_time = 10.0;
                    r.max_velocity_scaling_factor = speed;
                    r.max_acceleration_scaling_factor = speed;
                    r.num_planning_attempts = 3;
                    r.goal_constraints.clear();
                    r.goal_constraints.push_back(
                        kinematic_constraints::constructGoalConstraints(LINK_TOOL_NAME, curr));

                    seq_req.items.push_back(std::move(item));
                }

                // -------------------- PLAN VIA SHARED SERVICE --------------------
                auto service_request = std::make_shared<GetMotionSequence::Request>();
                service_request->request = seq_req;

                auto service_future = service_client->async_send_request(service_request);

                std::future_status service_status;
                do
                {
                    switch (service_status = service_future.wait_for(std::chrono::seconds(1)); service_status)
                    {
                    case std::future_status::deferred:
                        RCLCPP_ERROR(LOGGER, "Deferred");
                        break;
                    case std::future_status::timeout:
                        RCLCPP_INFO(LOGGER, "Waiting for trajectory plan...");
                        break;
                    case std::future_status::ready:
                        RCLCPP_INFO(LOGGER, "Service ready!");
                        break;
                    }
                } while (service_status != std::future_status::ready);

                auto service_response = service_future.get();
                if (service_response->response.error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
                {
                    RCLCPP_ERROR(LOGGER, "Planning failed for %s with error code: %d",
                                ply_id.c_str(),
                                service_response->response.error_code.val);
                }

                RCLCPP_INFO(LOGGER, "%s planning successful", ply_id.c_str());

                // collect the sequence in a single trajectory
                moveit_msgs::msg::RobotTrajectory traj;
                // keep joint names 
                traj.joint_trajectory.joint_names = service_response->response.planned_trajectories[0].joint_trajectory.joint_names;
                for (const auto& t : service_response->response.planned_trajectories){
                    // append points 
                    traj.joint_trajectory.points.insert(
                        traj.joint_trajectory.points.end(),
                        t.joint_trajectory.points.begin(),
                        t.joint_trajectory.points.end());
                }
                draw_trajectory_tool_path(traj, rvt::ORANGE);
                moveit_visual_tools.trigger();

                // -------------------- PUBLISH TRAJECTORY --------------------

                //prompt("Press 'Next' in the RVizVisualToolsGui window to execute the trajectory.");

                // delete the picking pose
                // semi_perm_scene_tools.deleteAllMarkers();
                // semi_perm_scene_tools.trigger(); 

                // reset for next plan 
                plan_msg.reset();

                // Build JSON
                auto j = encodeTrajectory(traj, ply_id);

                // Publish

                std_msgs::msg::String msg;
                msg.data = j.dump();           // compact, fastest

                try {
                    traj_pub->publish(msg);
                    RCLCPP_INFO(mqtt_logger,
                                "Trajectory JSON published (%zu bytes)", msg.data.size());
                }
                catch (const std::exception& e) {
                    RCLCPP_ERROR(mqtt_logger,
                                "Failed to publish trajectory JSON: %s", e.what());
                }

                RCLCPP_INFO(LOGGER, "Routine Path Plan completed. Planner ready for next command.");

            }


        }

        // Begin shutdown: stop executor, then join thread
        executor.cancel();
        if (spinner.joinable())
        spinner.join();

        executor.remove_node(node);
        // At end of this scope:
        //    - visual_tools
        //    - planning_scene_interface
        //    - move_group
        //    - node
        //    are all destroyed while the rclcpp context is still valid. 
    }

    // Shutdown ROS2
    rclcpp::shutdown();
    return 0;
}




