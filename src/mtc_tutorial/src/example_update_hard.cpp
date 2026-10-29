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

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
public:
    MTCTaskNode(const rclcpp::NodeOptions &options);

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

    void doTask();

    void setupPlanningScene();

private:
    // Compose an MTC task from a series of stages.
    mtc::Task createTask();
    mtc::Task task_;
    rclcpp::Node::SharedPtr node_;
};

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions &options)
    : node_{std::make_shared<rclcpp::Node>("mtc_node", options)}
{
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
    return node_->get_node_base_interface();
}

void MTCTaskNode::setupPlanningScene()
{
    moveit_msgs::msg::CollisionObject object;
    object.id = "object";
    object.header.frame_id = "world";
    object.primitives.resize(1);
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    object.primitives[0].dimensions = {0.1, 0.02};

    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.5;
    pose.position.y = -0.55;
    pose.orientation.w = 1.0;
    object.pose = pose;

    moveit::planning_interface::PlanningSceneInterface psi;
    psi.applyCollisionObject(object);
}

void MTCTaskNode::doTask()
{
    task_ = createTask();

    try
    {
        task_.init();
    }
    catch (mtc::InitStageException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER, e);
        return;
    }

    if (!task_.plan(10))
    {
        RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
        return;
    }
    task_.introspection().publishSolution(*task_.solutions().front());

    auto result = task_.execute(*task_.solutions().front());
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
        RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
        return;
    }

    return;
}

mtc::Task MTCTaskNode::createTask()
{
    mtc::Task task;
    task.stages()->setName("pick and place task");
    task.loadRobotModel(node_);

    const auto &arm_group_name = "arm";
    const auto &hand_group_name = "gripper";
    const auto &hand_frame = "wrist3_link";
    const auto &eef_name = "eef";

    task.setProperty("group", arm_group_name);
    task.setProperty("eef", eef_name);
    task.setProperty("ik_frame", hand_frame);

    mtc::Stage *current_state_ptr = nullptr;
    auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
    current_state_ptr = stage_state_current.get();
    task.add(std::move(stage_state_current));

    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
    auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(1.0);
    cartesian_planner->setMaxAccelerationScalingFactor(1.0);
    cartesian_planner->setStepSize(.01);

    // === 1. Open hand ===
    auto stage_open_hand =
        std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
    stage_open_hand->setGroup(hand_group_name);
    stage_open_hand->setGoal("open");
    task.add(std::move(stage_open_hand));

    // === 2. Move above object ===
    geometry_msgs::msg::PoseStamped pre_grasp_pose;
    pre_grasp_pose.header.frame_id = "world";
    pre_grasp_pose.pose.position.x = 0.5;
    pre_grasp_pose.pose.position.y = -0.55;
    pre_grasp_pose.pose.position.z = 0.25; // hover above object
    pre_grasp_pose.pose.orientation.x = 1.0;
    pre_grasp_pose.pose.orientation.y = 0.0;
    pre_grasp_pose.pose.orientation.z = 0.0;
    pre_grasp_pose.pose.orientation.w = 0.0;

    auto stage_move_pregrasp =
        std::make_unique<mtc::stages::MoveTo>("move above object", interpolation_planner);
    stage_move_pregrasp->setGroup(arm_group_name);
    stage_move_pregrasp->setGoal(pre_grasp_pose);
    task.add(std::move(stage_move_pregrasp));

    // // === 3. Approach object (downwards) ===
    auto stage_approach =
        std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
    stage_approach->setGroup(arm_group_name);
    stage_approach->setIKFrame(hand_frame);
    stage_approach->setMinMaxDistance(0.01, 0.05);
    geometry_msgs::msg::Vector3Stamped approach_vec;
    approach_vec.header.frame_id = "world";
    approach_vec.vector.z = -0.1; // move down 10cm
    stage_approach->setDirection(approach_vec);
    task.add(std::move(stage_approach));

    auto stage_collision = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
    stage_collision->allowCollisions("object",
                                     task.getRobotModel()
                                         ->getJointModelGroup(hand_group_name)
                                         ->getLinkModelNamesWithCollisionGeometry(),
                                     true);
    task.add(std::move(stage_collision));

    // // === 4. Close hand (grasp) ===
    auto stage_close_hand =
        std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
    stage_close_hand->setGroup(hand_group_name);
    stage_close_hand->setGoal("close");
    task.add(std::move(stage_close_hand));

    // // === 5. Lift object ===

    auto stage_attach = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
    stage_attach->attachObject("object", hand_frame);
    //   attach_object_stage = stage_attach.get();
    task.add(std::move(stage_attach));

    auto stage_lift =
        std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
    stage_lift->setGroup(arm_group_name);
    stage_lift->setIKFrame(hand_frame);
    stage_lift->setMinMaxDistance(0.01, 0.05);
    geometry_msgs::msg::Vector3Stamped lift_vec;
    lift_vec.header.frame_id = "world";
    lift_vec.vector.z = 0.15; // lift up 15 cm
    stage_lift->setDirection(lift_vec);
    task.add(std::move(stage_lift));

    // // === 6. Move to place position ===
    geometry_msgs::msg::PoseStamped place_pose;
    place_pose.header.frame_id = "world";
    place_pose.pose.position.x = 0.0; // new location
    place_pose.pose.position.y = 0.5;
    place_pose.pose.position.z = 0.25;
    place_pose.pose.orientation.x = 1.0;
    place_pose.pose.orientation.y = 0.0;
    place_pose.pose.orientation.z = 0.0;
    place_pose.pose.orientation.w = 0.0;

    auto stage_move_place =
        std::make_unique<mtc::stages::MoveTo>("move to place", interpolation_planner);
    stage_move_place->setGroup(arm_group_name);
    stage_move_place->setGoal(place_pose);
    task.add(std::move(stage_move_place));

    // // === 7. Lower to place ===
    auto stage_lower = std::make_unique<mtc::stages::MoveRelative>("lower to place", cartesian_planner);
    stage_lower->setGroup(arm_group_name);
    stage_lower->setIKFrame(hand_frame);
    stage_lower->setMinMaxDistance(0.01, 0.05);
    geometry_msgs::msg::Vector3Stamped lower_vec;
    lower_vec.header.frame_id = "world";
    lower_vec.vector.z = -0.12; // move down 12cm
    stage_lower->setDirection(lower_vec);
    task.add(std::move(stage_lower));

    // // === 8. Open hand (release) ===
    auto stage_open_release =
        std::make_unique<mtc::stages::MoveTo>("open hand release", interpolation_planner);
    stage_open_release->setGroup(hand_group_name);
    stage_open_release->setGoal("open");
    task.add(std::move(stage_open_release));


    auto stage_forbid = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
    stage_forbid->allowCollisions("object",
                             task.getRobotModel()
                                 ->getJointModelGroup(hand_group_name)
                                 ->getLinkModelNamesWithCollisionGeometry(),
                             false);
    task.add(std::move(stage_forbid));


    auto stage_detach = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
    stage_detach->detachObject("object", hand_frame);
    task.add(std::move(stage_detach));

    

    // // === 9. Retreat back ===
    auto stage_retreat =
        std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
    stage_retreat->setGroup(arm_group_name);
    stage_retreat->setIKFrame(hand_frame);
    stage_retreat->setMinMaxDistance(0.01, 0.05);
    geometry_msgs::msg::Vector3Stamped retreat_vec;
    retreat_vec.header.frame_id = "world";
    retreat_vec.vector.z = 0.15; // move up after placing
    stage_retreat->setDirection(retreat_vec);
    task.add(std::move(stage_retreat));

    // // === 10. Return home ===
    auto stage_move_home =
        std::make_unique<mtc::stages::MoveTo>("move home", interpolation_planner);
    stage_move_home->setGroup(arm_group_name);
    stage_move_home->setGoal("home");
    task.add(std::move(stage_move_home));

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
    executor.remove_node(mtc_task_node->getNodeBaseInterface()); });

    mtc_task_node->setupPlanningScene();
    mtc_task_node->doTask();

    spin_thread->join();
    rclcpp::shutdown();
    return 0;
}