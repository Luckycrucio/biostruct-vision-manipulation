// Luca Grigolin at PROFACTOR GmbH

// EXECUTOR node

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
#include "biostruct_robotics_lab/helpers.h"

// JSON
#include <nlohmann/json.hpp>

// TF2 for RPY → Quaternion
#include <tf2/LinearMath/Quaternion.h>

// File writing
#include <fstream>


// Aliases and Constants
namespace rvt = rviz_visual_tools;
static const std::string NODE_NAME = "execution_node";
static const std::string PLANNING_GROUP = "arm"; // must match MoveIt SRDF group
static const std::string LINK_BASE_NAME = "base";
static const std::string LINK_TOOL_NAME = "link_6"; // end effector link
using moveit::planning_interface::PlanningSceneInterface;

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
        moveit_visual_tools.loadRemoteControl(); // enable RViz "NEXT" button panel

        //                                  [Local Helpers]
    
        // HELPERS - VISUALIZATION

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

        // build plan wrapper from an arbitrary trajectory (using current robot state)
        auto const trajectory_plan_wrapper =
            [&](moveit_msgs::msg::RobotTrajectory& traj)
        {
            MoveGroupInterface::Plan plan_wrapper;
            // zero out header timestamp
            traj.joint_trajectory.header.stamp.sec = 0;
            traj.joint_trajectory.header.stamp.nanosec = 0; 

            // trajectory: use provided traj
            plan_wrapper.trajectory = traj;

            // start_state_: use current robot state
            moveit_msgs::msg::RobotState start_msg;
            {
                moveit::core::RobotState rs(*move_group_interface.getCurrentState());
                rs.update();
                moveit::core::robotStateToRobotStateMsg(rs, start_msg);
            }
            plan_wrapper.start_state = start_msg;

            // planning_time_: any positive value is acceptable
            plan_wrapper.planning_time = 0.05;

            return plan_wrapper;
        };

        
        //                                  [MQTT PLANNING LOOP]

        // ----------------- Shared state for MQTT messages -----------------

        std::mutex mtx; // execution block (to lock)
        std::condition_variable cv;  // condition variable for mqtt synchronization
        // storing messages structures
        std::optional<std_msgs::msg::String> execution_msg; 
        std::optional<std_msgs::msg::String> term_msg; 
        std::optional<std_msgs::msg::String> trajectory_msg;

        // ----------------- Subscriptions for MQTT topics -----------------

        auto execution_sub = node->create_subscription<std_msgs::msg::String>(
            "/execution", 10,
            [&](const std_msgs::msg::String::SharedPtr msg)
        {
            {
                std::lock_guard<std::mutex> lock(mtx);
                execution_msg = *msg;
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

        auto trajectory_sub = node->create_subscription<std_msgs::msg::String>(
            "/trajectories_to_execute", 10,
            [&](const std_msgs::msg::String::SharedPtr msg)
        {
            {
                std::lock_guard<std::mutex> lock(mtx);
                trajectory_msg = *msg;   // store first (or latest) message
            }
            cv.notify_one();
        });

        // ----------------- Publisher for MQTT topic -----------------

        auto feedback_pub =
            node->create_publisher<std_msgs::msg::String>("/feedback", 10);
        if (!feedback_pub) {
            RCLCPP_ERROR(mqtt_logger, "Feedback Publisher is null.");
        }

        // =====================================================================
        //         >>>>>>>>>>>>>>>>  MAIN PROCESSING LOOP  <<<<<<<<<<<<<<
        // =====================================================================



        while (true)
        {

            // ----------------- Wait until BOTH messages are received -----------------

            {
                std::unique_lock<std::mutex> lock(mtx);
                RCLCPP_INFO(mqtt_logger, "Waiting for an execution ...");
                cv.wait(lock, [&]{
                    return (execution_msg.has_value() && trajectory_msg.has_value()) || term_msg.has_value();

                });
            }

            if (term_msg.has_value()){

                // ---------------- Termination ----------------
                
                RCLCPP_INFO(mqtt_logger, "Termination command received, shutting down Executor.");
                break;
            }

            if (execution_msg.has_value() && trajectory_msg.has_value()){

                
                RCLCPP_INFO(mqtt_logger, "Execution trigger received.");

                // ---------------- Decode trajectory ----------------
                moveit_msgs::msg::RobotTrajectory traj = decodeTrajectory(trajectory_msg->data);

                // safety check on the received trajectory 

                auto& jt = traj.joint_trajectory;
                if (jt.points.empty()) {
                    RCLCPP_ERROR(LOGGER, "Trajectory has no points");
                } 
                else{
                    // Overwrite the first trajectory point with current joint positions
                    // (near joint limits there could be false mismatches)
                    // (EGM noise or robot has moved a bit while time has passed)
                    // Overwrite the first trajectory point with current joint positions
                    // only if at least one joint differs by more than 0.1 rad
                    auto current_state = move_group_interface.getCurrentState(1.0);
                    if (!current_state) {
                        throw std::runtime_error("Failed to get current state");
                    }
                    const auto* jmg = current_state->getJointModelGroup(PLANNING_GROUP);
                    std::vector<double> current;
                    current_state->copyJointGroupPositions(jmg, current);

                    const auto& q0 = jt.points.front().positions;
                    if (q0.size() != current.size()) {
                        throw std::runtime_error("Size mismatch between trajectory start and current state");
                    }
                    bool overwrite = false;
                    for (size_t i = 0; i < q0.size(); ++i)
                    {
                        if (std::abs(current[i] - q0[i]) > 0.1) {
                        overwrite = true;
                        break;
                        }
                    }
                    if (overwrite)
                    {
                        jt.points.front().positions = current;
                        jt.points.front().time_from_start.sec = 0;
                        jt.points.front().time_from_start.nanosec = 0;
                    }

                    // make the plan disappear and draw the trajectory after encode-decode operations
                    moveit_visual_tools.deleteAllMarkers();
                    draw_trajectory_tool_path(traj, rvt::LIME_GREEN);
                    moveit_visual_tools.trigger();

                    // Execute
                    auto plan_wrapper = trajectory_plan_wrapper(traj);
                    moveit::core::MoveItErrorCode rc = move_group_interface.execute(plan_wrapper);

                    wait_until_at_goal(move_group_interface, PLANNING_GROUP, traj, 0.001);

                    // get feedback
                    if (rc == moveit::core::MoveItErrorCode::SUCCESS) {
                        RCLCPP_INFO(LOGGER, "Execution succeeded");
                        // publish feedback to coordinator
                        std_msgs::msg::String msg;
                        try {
                            feedback_pub->publish(msg);
                            RCLCPP_INFO(mqtt_logger,
                                        "Feedback published succesfully.");
                        }
                        catch (const std::exception& e) {
                            RCLCPP_ERROR(mqtt_logger,
                                        "Failed to publish feedback: %s", e.what());
                        }
                    } else {
                        RCLCPP_ERROR(LOGGER, "Execution failed, no feedback published, code=%d", rc.val);
                    }
                    moveit_visual_tools.deleteAllMarkers();
                    moveit_visual_tools.trigger();
                    semi_perm_scene_tools.deleteAllMarkers();
                    semi_perm_scene_tools.trigger(); 
                }

            }

            // reset for next execution 
            execution_msg.reset();
            trajectory_msg.reset();
            RCLCPP_INFO(LOGGER, "Executor cycle completed. Ready for next plan.");
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
