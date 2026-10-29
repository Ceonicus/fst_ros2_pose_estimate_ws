#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif
#include <std_msgs/msg/bool.hpp>
#include <iostream>
#include <string>
#include <map>
#include <unordered_map>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
public:
    MTCTaskNode(const rclcpp::NodeOptions &options);

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

    void doTask(const std::string &object_name = "object1", const bool reverse = false);

    void initializeSuction();
    void initializeFingerGripper();

    void setupPlanningScene();

private:
    mtc::Task createTask(const std::string &object_name, const bool reverse = false);

    mtc::Task task_;
    rclcpp::Node::SharedPtr node_;
    moveit::planning_interface::PlanningSceneInterface psi_;
    std::unordered_map<std::string, geometry_msgs::msg::Pose> object_poses_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr suction_init_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr finger_gripper_init_pub_;
};

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions &options)
    : node_{std::make_shared<rclcpp::Node>("mtc_node", options)}, psi_{}
{
    suction_init_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
        "/initialize_suction", rclcpp::QoS(1).reliable());

    finger_gripper_init_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
        "/initialize_finger_gripper", rclcpp::QoS(1).reliable());
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
    return node_->get_node_base_interface();
}

// ------------------------------------------------------------------
// Publish initialize_suction = true, wait for hardware to finish
// ------------------------------------------------------------------
void MTCTaskNode::initializeSuction()
{
    RCLCPP_INFO(LOGGER, "Publishing initialize_suction = true");
    std_msgs::msg::Bool msg;
    msg.data = true;
    suction_init_pub_->publish(msg);

    // Give hardware interface time to complete suction init
    // (StopMotion + CloseRPC + on_activate + suction init sequence ~10s)
    RCLCPP_INFO(LOGGER, "Waiting for suction initialization to complete...");
    rclcpp::sleep_for(std::chrono::seconds(15));
    RCLCPP_INFO(LOGGER, "Suction initialization wait done.");
}

// ------------------------------------------------------------------
// Publish initialize_finger_gripper = true, wait for hardware to finish
// ------------------------------------------------------------------
void MTCTaskNode::initializeFingerGripper()
{
    RCLCPP_INFO(LOGGER, "Publishing initialize_finger_gripper = true");
    std_msgs::msg::Bool msg;
    msg.data = true;
    finger_gripper_init_pub_->publish(msg);

    // Give hardware interface time to complete finger gripper init
    RCLCPP_INFO(LOGGER, "Waiting for finger gripper initialization to complete...");
    rclcpp::sleep_for(std::chrono::seconds(15));
    RCLCPP_INFO(LOGGER, "Finger gripper initialization wait done.");
}

void MTCTaskNode::setupPlanningScene()
{
    moveit_msgs::msg::CollisionObject object1;
    object1.id = "object1";
    object1.header.frame_id = "world";
    object1.primitives.resize(1);
    object1.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    object1.primitives[0].dimensions = {0.1, 0.02};

    geometry_msgs::msg::Pose pose1;
    pose1.position.x = 0.5;
    pose1.position.y = -0.55;
    pose1.position.z = 0.05;
    pose1.orientation.w = 1.0;
    object1.pose = pose1;
    psi_.applyCollisionObject(object1);
    object_poses_["object1"] = pose1;

    moveit_msgs::msg::CollisionObject object2;
    object2.id = "object2";
    object2.header.frame_id = "world";
    object2.primitives.resize(1);
    object2.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    object2.primitives[0].dimensions = {0.1, 0.02};

    geometry_msgs::msg::Pose pose2;
    pose2.position.x = 0.1;
    pose2.position.y = -0.55;
    pose2.position.z = 0.05;
    pose2.orientation.w = 1.0;
    object2.pose = pose2;
    psi_.applyCollisionObject(object2);
    object_poses_["object2"] = pose2;
}

void MTCTaskNode::doTask(const std::string &object_name, const bool reverse)
{
    task_ = createTask(object_name, reverse);

    try
    {
        task_.init();
    }
    catch (mtc::InitStageException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER, e);
        return;
    }

    while (!task_.plan())
    {
        RCLCPP_WARN_STREAM(LOGGER, "No solution found, retrying...");
    }

    task_.introspection().publishSolution(*task_.solutions().front());

    auto result = task_.execute(*task_.solutions().front());
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
        RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
        return;
    }
}

mtc::Task MTCTaskNode::createTask(const std::string &object_name, const bool reverse)
{
    mtc::Task task;
    task.stages()->setName("pick and place task");
    task.loadRobotModel(node_);

    const auto &arm_group_name        = "arm";
    const auto &finger_gripper_name   = "finger_gripper";
    const auto &hand_frame            = "wrist3_link";
    const auto &eef_name              = "eef";
    const auto &suction_gripper_name  = "suction_gripper";

    task.setProperty("group", arm_group_name);
    task.setProperty("eef", eef_name);
    task.setProperty("ik_frame", hand_frame);

    mtc::Stage *current_state_ptr = nullptr;
    auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
    current_state_ptr = stage_state_current.get();
    task.add(std::move(stage_state_current));

    auto sampling_planner      = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
    auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
    auto cartesian_planner     = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(1.0);
    cartesian_planner->setMaxAccelerationScalingFactor(1.0);
    cartesian_planner->setStepSize(.01);
    /////
    auto stage_approach6 = std::make_unique<mtc::stages::MoveRelative>(
        "approach object", cartesian_planner);
    stage_approach6->setGroup(arm_group_name);
    stage_approach6->setIKFrame(hand_frame);
    stage_approach6->setMinMaxDistance(0.2, 0.2);  // exactly 10cm

    geometry_msgs::msg::Vector3Stamped approach_vec6;
    approach_vec6.header.frame_id = "world";
    approach_vec6.vector.z = -1.0;   // unit direction = -Z (normalize direction)
    stage_approach6->setDirection(approach_vec6);
    task.add(std::move(stage_approach6));

    auto stage_approach4 = std::make_unique<mtc::stages::MoveRelative>(
        "approach asd", cartesian_planner);
    stage_approach4->setGroup(arm_group_name);
    stage_approach4->setIKFrame(hand_frame);
    stage_approach4->setMinMaxDistance(0.08, 0.08);  // exactly 10cm

    geometry_msgs::msg::Vector3Stamped approach_vec4;
    approach_vec4.header.frame_id = "world";
    approach_vec4.vector.x = 0.707;  // -1/√2
    approach_vec4.vector.y = 0.707;  
    stage_approach4->setDirection(approach_vec4);
    task.add(std::move(stage_approach4));

    auto stage_approach2 = std::make_unique<mtc::stages::MoveRelative>(
        "approach object", cartesian_planner);
    stage_approach2->setGroup(arm_group_name);
    stage_approach2->setIKFrame(hand_frame);
    stage_approach2->setMinMaxDistance(0.055, 0.055);  // exactly 10cm

    geometry_msgs::msg::Vector3Stamped approach_vec2;
    approach_vec2.header.frame_id = "world";
    approach_vec2.vector.z = 1.0;   // unit direction = -Z (normalize direction)
    stage_approach2->setDirection(approach_vec2);
    task.add(std::move(stage_approach2));

    // Move home
    // auto stage_move_home = std::make_unique<mtc::stages::MoveTo>("move home", interpolation_planner);
    // stage_move_home->setGroup(arm_group_name);
    // stage_move_home->setGoal("home");
    // task.add(std::move(stage_move_home));

    std::map<std::string, double> suction_tool_pick_joints = {
        {"j1", -2.271130728130233},
        {"j2", -2.1910067304160545},
        {"j3", -1.7586443916544534},  // note: j3 is 4th in topic but use name, not index
        {"j4", -0.7659728000903013},
        {"j5",  1.5725353709867185},
        {"j6", -1.4872969055570973}
    };

    auto stage_pick_suction_tool = std::make_unique<mtc::stages::MoveTo>(
        "pick up suction tool pose", interpolation_planner);
    stage_pick_suction_tool->setGroup(arm_group_name);
    stage_pick_suction_tool->setGoal(suction_tool_pick_joints);
    task.add(std::move(stage_pick_suction_tool));
    /////////////
    auto stage_approach = std::make_unique<mtc::stages::MoveRelative>(
        "approach object", cartesian_planner);
    stage_approach->setGroup(arm_group_name);
    stage_approach->setIKFrame(hand_frame);
    stage_approach->setMinMaxDistance(0.055, 0.055);  // exactly 10cm

    geometry_msgs::msg::Vector3Stamped approach_vec;
    approach_vec.header.frame_id = "world";
    approach_vec.vector.z = -1.0;   // unit direction = -Z (normalize direction)
    stage_approach->setDirection(approach_vec);
    task.add(std::move(stage_approach));
//////////
    auto stage_approach3 = std::make_unique<mtc::stages::MoveRelative>(
        "approach asd", cartesian_planner);
    stage_approach3->setGroup(arm_group_name);
    stage_approach3->setIKFrame(hand_frame);
    stage_approach3->setMinMaxDistance(0.08, 0.08);  // exactly 10cm

    geometry_msgs::msg::Vector3Stamped approach_vec3;
    approach_vec3.header.frame_id = "world";
    approach_vec3.vector.x = -0.707;  // -1/√2
    approach_vec3.vector.y = -0.707;  
    stage_approach3->setDirection(approach_vec3);
    task.add(std::move(stage_approach3));

    ////
    auto stage_approach5 = std::make_unique<mtc::stages::MoveRelative>(
        "approach object", cartesian_planner);
    stage_approach5->setGroup(arm_group_name);
    stage_approach5->setIKFrame(hand_frame);
    stage_approach5->setMinMaxDistance(0.2, 0.2);  // exactly 10cm

    geometry_msgs::msg::Vector3Stamped approach_vec5;
    approach_vec5.header.frame_id = "world";
    approach_vec5.vector.z = 1.0;   // unit direction = -Z (normalize direction)
    stage_approach5->setDirection(approach_vec5);
    task.add(std::move(stage_approach5));

    return task;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);

    auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
    rclcpp::executors::MultiThreadedExecutor executor;

    auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]()
    {
        executor.add_node(mtc_task_node->getNodeBaseInterface());
        executor.spin();
        executor.remove_node(mtc_task_node->getNodeBaseInterface());
    });

    mtc_task_node->setupPlanningScene();

    // Initialize finger gripper first, then run task
    // mtc_task_node->initializeFingerGripper();
    // mtc_task_node->initializeSuction();
    // mtc_task_node->doTask("object2", false);

    // rclcpp::sleep_for(std::chrono::seconds(10));
    mtc_task_node->doTask("object2", false);
    // mtc_task_node->doTask("object2", false);

    spin_thread->join();
    rclcpp::shutdown();
    return 0;
}