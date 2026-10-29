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

// Convenience aliases — avoids the broken ::Ptr typedef on solver subclasses
using InterpPlanner    = std::shared_ptr<mtc::solvers::JointInterpolationPlanner>;
using CartesianPlanner = std::shared_ptr<mtc::solvers::CartesianPath>;

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

    /**
     * @brief Appends all stages required to pick up the suction tool into @p task.
     *
     * Sequence:
     *   1. MoveTo                  – pre-grasp joint pose
     *   2. MoveRelative -Z  55 mm  – insert flange into tool holder
     *   3. MoveRelative (-X,-Y) 80 mm  – latch / rotate into lock
     *   4. MoveRelative +Z 200 mm  – lift tool clear of rack
     */
    void pickUpSuctionTool(mtc::Task           &task,
                           const std::string   &arm_group_name,
                           const std::string   &hand_frame,
                           const InterpPlanner    &interp_planner,
                           const CartesianPlanner &cartesian_planner);

    /**
     * @brief Appends all stages required to place the suction tool back into @p task.
     *
     * Sequence (exact reverse of pick-up):
     *   1. MoveTo                  – pre-place joint pose (above rack slot)
     *   2. MoveRelative -Z 200 mm  – lower tool into rack slot
     *   3. MoveRelative (+X,+Y) 80 mm  – reverse-rotate to release latch
     *   4. MoveRelative +Z  55 mm  – withdraw flange from tool holder
     */
    void placeSuctionTool(mtc::Task           &task,
                          const std::string   &arm_group_name,
                          const std::string   &hand_frame,
                          const InterpPlanner    &interp_planner,
                          const CartesianPlanner &cartesian_planner);

    /**
     * @brief Appends all stages required to pick up the finger gripper into @p task.
     *
     * Sequence:
     *   1. MoveTo                   – pre-grasp joint pose
     *   2. MoveRelative -Z  74 mm   – lower onto finger tool coupler
     *   3. MoveRelative (-X,-Y) 90 mm  – latch / rotate into lock
     *   4. MoveRelative +Z 270 mm   – lift tool clear of rack
     */
    void pickUpFingerTool(mtc::Task           &task,
                          const std::string   &arm_group_name,
                          const std::string   &hand_frame,
                          const InterpPlanner    &interp_planner,
                          const CartesianPlanner &cartesian_planner);

    /**
     * @brief Appends all stages required to place the finger gripper back into @p task.
     *
     * Sequence (exact reverse of pick-up):
     *   1. MoveTo                   – pre-place joint pose (above rack slot)
     *   2. MoveRelative -Z 270 mm   – lower tool into rack slot
     *   3. MoveRelative (+X,+Y) 90 mm  – reverse-rotate to release latch
     *   4. MoveRelative +Z  (arm retreats via next MoveTo or further stage)
     */
    void placeFingerTool(mtc::Task           &task,
                         const std::string   &arm_group_name,
                         const std::string   &hand_frame,
                         const InterpPlanner    &interp_planner,
                         const CartesianPlanner &cartesian_planner);

    mtc::Task task_;
    rclcpp::Node::SharedPtr node_;
    moveit::planning_interface::PlanningSceneInterface psi_;
    std::unordered_map<std::string, geometry_msgs::msg::Pose> object_poses_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr suction_init_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr finger_gripper_init_pub_;
};

// ─────────────────────────────────────────────────────────────────────────────
//  Constructor
// ─────────────────────────────────────────────────────────────────────────────
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

// ─────────────────────────────────────────────────────────────────────────────
//  Hardware initialisation helpers
// ─────────────────────────────────────────────────────────────────────────────
void MTCTaskNode::initializeSuction()
{
    RCLCPP_INFO(LOGGER, "Publishing initialize_suction = true");
    std_msgs::msg::Bool msg;
    msg.data = true;
    suction_init_pub_->publish(msg);

    RCLCPP_INFO(LOGGER, "Waiting for suction initialization to complete...");
    rclcpp::sleep_for(std::chrono::seconds(15));
    RCLCPP_INFO(LOGGER, "Suction initialization wait done.");
}

void MTCTaskNode::initializeFingerGripper()
{
    RCLCPP_INFO(LOGGER, "Publishing initialize_finger_gripper = true");
    std_msgs::msg::Bool msg;
    msg.data = true;
    finger_gripper_init_pub_->publish(msg);

    RCLCPP_INFO(LOGGER, "Waiting for finger gripper initialization to complete...");
    rclcpp::sleep_for(std::chrono::seconds(15));
    RCLCPP_INFO(LOGGER, "Finger gripper initialization wait done.");
}

// ─────────────────────────────────────────────────────────────────────────────
//  Planning scene
// ─────────────────────────────────────────────────────────────────────────────
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

// ─────────────────────────────────────────────────────────────────────────────
//  pickUpSuctionTool
// ─────────────────────────────────────────────────────────────────────────────
void MTCTaskNode::pickUpSuctionTool(
    mtc::Task           &task,
    const std::string   &arm_group_name,
    const std::string   &hand_frame,
    const InterpPlanner    &interp_planner,
    const CartesianPlanner &cartesian_planner)
{
    // ── 1. Pre-grasp joint pose ───────────────────────────────────────────────
    const std::map<std::string, double> pick_joints = {
        {"j1", -2.276985687786772},
        {"j2", -2.183257197960033},
        {"j3", -1.7513238944500433},
        {"j4", -0.7879155829968729},
        {"j5",  1.5622683523522447},
        {"j6", -1.500316700652331}
    };

    auto stage_pre_grasp = std::make_unique<mtc::stages::MoveTo>(
        "pick_suction: pre-grasp pose", interp_planner);
    stage_pre_grasp->setGroup(arm_group_name);
    stage_pre_grasp->setGoal(pick_joints);
    task.add(std::move(stage_pre_grasp));

    // ── 2. Insert flange into holder: -Z 55 mm ───────────────────────────────
    auto stage_insert = std::make_unique<mtc::stages::MoveRelative>(
        "pick_suction: insert into holder", cartesian_planner);
    stage_insert->setGroup(arm_group_name);
    stage_insert->setIKFrame(hand_frame);
    stage_insert->setMinMaxDistance(0.06, 0.06);
    {
        geometry_msgs::msg::Vector3Stamped v;
        v.header.frame_id = "world";
        v.vector.z = -1.0;
        stage_insert->setDirection(v);
    }
    task.add(std::move(stage_insert));

    // ── 3. Latch / rotate into lock: (-X, -Y) 80 mm ──────────────────────────
    auto stage_latch = std::make_unique<mtc::stages::MoveRelative>(
        "pick_suction: latch into lock", cartesian_planner);
    stage_latch->setGroup(arm_group_name);
    stage_latch->setIKFrame(hand_frame);
    stage_latch->setMinMaxDistance(0.08, 0.08);
    {
        geometry_msgs::msg::Vector3Stamped v;
        v.header.frame_id = "world";
        v.vector.x = -0.707;  // -1/√2
        v.vector.y = -0.707;
        stage_latch->setDirection(v);
    }
    task.add(std::move(stage_latch));

    // ── 4. Lift tool clear of rack: +Z 200 mm ────────────────────────────────
    auto stage_lift = std::make_unique<mtc::stages::MoveRelative>(
        "pick_suction: lift tool", cartesian_planner);
    stage_lift->setGroup(arm_group_name);
    stage_lift->setIKFrame(hand_frame);
    stage_lift->setMinMaxDistance(0.2, 0.2);
    {
        geometry_msgs::msg::Vector3Stamped v;
        v.header.frame_id = "world";
        v.vector.z = 1.0;
        stage_lift->setDirection(v);
    }
    task.add(std::move(stage_lift));
}

// ─────────────────────────────────────────────────────────────────────────────
//  placeSuctionTool
// ─────────────────────────────────────────────────────────────────────────────
void MTCTaskNode::placeSuctionTool(
    mtc::Task           &task,
    const std::string   &arm_group_name,
    const std::string   &hand_frame,
    const InterpPlanner    &interp_planner,
    const CartesianPlanner &cartesian_planner)
{
    // ── 1. Pre-place joint pose (above rack slot) ─────────────────────────────
    const std::map<std::string, double> place_joints = {
        {"j1", -2.2859767859429887},
        {"j2", -2.1586792837338553},
        {"j3", -1.3651479595253595},
        {"j4", -1.198604783912273},
        {"j5",  1.562169549075803},
        {"j6", -1.509307931966602}
    };

    auto stage_pre_place = std::make_unique<mtc::stages::MoveTo>(
        "place_suction: pre-place pose", interp_planner);
    stage_pre_place->setGroup(arm_group_name);
    stage_pre_place->setGoal(place_joints);
    task.add(std::move(stage_pre_place));

    // ── 2. Lower into slot: -Z 200 mm ────────────────────────────────────────
    auto stage_lower = std::make_unique<mtc::stages::MoveRelative>(
        "place_suction: lower into slot", cartesian_planner);
    stage_lower->setGroup(arm_group_name);
    stage_lower->setIKFrame(hand_frame);
    stage_lower->setMinMaxDistance(0.2, 0.2);
    {
        geometry_msgs::msg::Vector3Stamped v;
        v.header.frame_id = "world";
        v.vector.z = -1.0;
        stage_lower->setDirection(v);
    }
    task.add(std::move(stage_lower));

    // ── 3. Release latch: reverse-rotate (+X, +Y) 80 mm ─────────────────────
    auto stage_release = std::make_unique<mtc::stages::MoveRelative>(
        "place_suction: release latch", cartesian_planner);
    stage_release->setGroup(arm_group_name);
    stage_release->setIKFrame(hand_frame);
    stage_release->setMinMaxDistance(0.08, 0.08);
    {
        geometry_msgs::msg::Vector3Stamped v;
        v.header.frame_id = "world";
        v.vector.x = 0.707;   // +1/√2  (exact reverse of latch)
        v.vector.y = 0.707;
        stage_release->setDirection(v);
    }
    task.add(std::move(stage_release));

    // ── 4. Withdraw flange: +Z 55 mm ─────────────────────────────────────────
    auto stage_withdraw = std::make_unique<mtc::stages::MoveRelative>(
        "place_suction: withdraw flange", cartesian_planner);
    stage_withdraw->setGroup(arm_group_name);
    stage_withdraw->setIKFrame(hand_frame);
    stage_withdraw->setMinMaxDistance(0.06, 0.06);
    {
        geometry_msgs::msg::Vector3Stamped v;
        v.header.frame_id = "world";
        v.vector.z = 1.0;
        stage_withdraw->setDirection(v);
    }
    task.add(std::move(stage_withdraw));
}

// ─────────────────────────────────────────────────────────────────────────────
//  pickUpFingerTool
// ─────────────────────────────────────────────────────────────────────────────
void MTCTaskNode::pickUpFingerTool(
    mtc::Task           &task,
    const std::string   &arm_group_name,
    const std::string   &hand_frame,
    const InterpPlanner    &interp_planner,
    const CartesianPlanner &cartesian_planner)
{
    // ── 1. Pre-grasp joint pose ───────────────────────────────────────────────
    const std::map<std::string, double> pick_joints = {
        {"j1", -2.0383339635503734},
        {"j2", -2.194165905259161},
        {"j3", -1.7303036971241026},
        {"j4", -0.7978446464890154},
        {"j5",  1.5721291057624265},
        {"j6", -1.2719889866977334}
    };

    auto stage_pre_grasp = std::make_unique<mtc::stages::MoveTo>(
        "pick_finger: pre-grasp pose", interp_planner);
    stage_pre_grasp->setGroup(arm_group_name);
    stage_pre_grasp->setGoal(pick_joints);
    task.add(std::move(stage_pre_grasp));

    // ── 2. Lower onto coupler: -Z 74 mm ──────────────────────────────────────
    auto stage_lower = std::make_unique<mtc::stages::MoveRelative>(
        "pick_finger: lower onto coupler", cartesian_planner);
    stage_lower->setGroup(arm_group_name);
    stage_lower->setIKFrame(hand_frame);
    stage_lower->setMinMaxDistance(0.061, 0.061);
    {
        geometry_msgs::msg::Vector3Stamped v;
        v.header.frame_id = "world";
        v.vector.z = -1.0;
        stage_lower->setDirection(v);
    }
    task.add(std::move(stage_lower));

    // ── 3. Latch / rotate into lock: (-X, -Y) 90 mm ──────────────────────────
    auto stage_latch = std::make_unique<mtc::stages::MoveRelative>(
        "pick_finger: latch into lock", cartesian_planner);
    stage_latch->setGroup(arm_group_name);
    stage_latch->setIKFrame(hand_frame);
    stage_latch->setMinMaxDistance(0.09, 0.09);
    {
        geometry_msgs::msg::Vector3Stamped v;
        v.header.frame_id = "world";
        v.vector.x = -0.707;  // -1/√2
        v.vector.y = -0.707;
        stage_latch->setDirection(v);
    }
    task.add(std::move(stage_latch));

    // ── 4. Lift tool clear of rack: +Z 270 mm ────────────────────────────────
    auto stage_lift = std::make_unique<mtc::stages::MoveRelative>(
        "pick_finger: lift tool", cartesian_planner);
    stage_lift->setGroup(arm_group_name);
    stage_lift->setIKFrame(hand_frame);
    stage_lift->setMinMaxDistance(0.27, 0.27);
    {
        geometry_msgs::msg::Vector3Stamped v;
        v.header.frame_id = "world";
        v.vector.z = 1.0;
        stage_lift->setDirection(v);
    }
    task.add(std::move(stage_lift));
}

// ─────────────────────────────────────────────────────────────────────────────
//  placeFingerTool
// ─────────────────────────────────────────────────────────────────────────────
void MTCTaskNode::placeFingerTool(
    mtc::Task           &task,
    const std::string   &arm_group_name,
    const std::string   &hand_frame,
    const InterpPlanner    &interp_planner,
    const CartesianPlanner &cartesian_planner)
{
    // ── 1. Pre-place joint pose (above rack slot) ─────────────────────────────
    const std::map<std::string, double> place_joints = {
        {"j1", -2.0775073321307302},
        {"j2", -2.146931414375243},
        {"j3", -1.2010120152215957},
        {"j4", -1.374412564325527},
        {"j5",  1.5717456105654553},
        {"j6", -1.3111586268525643}
    };

    auto stage_pre_place = std::make_unique<mtc::stages::MoveTo>(
        "place_finger: pre-place pose", interp_planner);
    stage_pre_place->setGroup(arm_group_name);
    stage_pre_place->setGoal(place_joints);
    task.add(std::move(stage_pre_place));

    // ── 2. Lower into slot: -Z 270 mm ────────────────────────────────────────
    auto stage_lower = std::make_unique<mtc::stages::MoveRelative>(
        "place_finger: lower into slot", cartesian_planner);
    stage_lower->setGroup(arm_group_name);
    stage_lower->setIKFrame(hand_frame);
    stage_lower->setMinMaxDistance(0.27, 0.27);
    {
        geometry_msgs::msg::Vector3Stamped v;
        v.header.frame_id = "world";
        v.vector.z = -1.0;
        stage_lower->setDirection(v);
    }
    task.add(std::move(stage_lower));

    // ── 3. Release latch: reverse-rotate (+X, +Y) 90 mm ─────────────────────
    auto stage_release = std::make_unique<mtc::stages::MoveRelative>(
        "place_finger: release latch", cartesian_planner);
    stage_release->setGroup(arm_group_name);
    stage_release->setIKFrame(hand_frame);
    stage_release->setMinMaxDistance(0.09, 0.09);
    {
        geometry_msgs::msg::Vector3Stamped v;
        v.header.frame_id = "world";
        v.vector.x = 0.707;   // +1/√2  (exact reverse of latch)
        v.vector.y = 0.707;
        stage_release->setDirection(v);
    }
    task.add(std::move(stage_release));

    auto stage_withdraw = std::make_unique<mtc::stages::MoveRelative>(
        "place_finger: withdraw flange", cartesian_planner);
    stage_withdraw->setGroup(arm_group_name);
    stage_withdraw->setIKFrame(hand_frame);
    stage_withdraw->setMinMaxDistance(0.061, 0.061);
    {
        geometry_msgs::msg::Vector3Stamped v;
        v.header.frame_id = "world";
        v.vector.z = 1.0;
        stage_withdraw->setDirection(v);
    }
    task.add(std::move(stage_withdraw));
}

// ─────────────────────────────────────────────────────────────────────────────
//  doTask
// ─────────────────────────────────────────────────────────────────────────────
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

// ─────────────────────────────────────────────────────────────────────────────
//  createTask
// ─────────────────────────────────────────────────────────────────────────────
mtc::Task MTCTaskNode::createTask(const std::string &object_name, const bool reverse)
{
    mtc::Task task;
    task.stages()->setName("pick and place task");
    task.loadRobotModel(node_);

    const auto &arm_group_name       = "arm";
    const auto &finger_gripper_name  = "finger_gripper";   // reserved for future use
    const auto &hand_frame           = "wrist3_link";
    const auto &eef_name             = "eef";
    const auto &suction_gripper_name = "suction_gripper";  // reserved for future use

    task.setProperty("group", arm_group_name);
    task.setProperty("eef", eef_name);
    task.setProperty("ik_frame", hand_frame);

    // ── Current state ─────────────────────────────────────────────────────────
    mtc::Stage *current_state_ptr = nullptr;
    auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
    current_state_ptr = stage_state_current.get();
    task.add(std::move(stage_state_current));

    // ── Solvers ───────────────────────────────────────────────────────────────
    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);

    InterpPlanner    interp = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

    CartesianPlanner cart   = std::make_shared<mtc::solvers::CartesianPath>();
    cart->setMaxVelocityScalingFactor(1.0);
    cart->setMaxAccelerationScalingFactor(1.0);
    cart->setStepSize(0.01);
    ///home positon
    auto stage_move_home = std::make_unique<mtc::stages::MoveTo>("move home", interp);
    stage_move_home->setGroup(arm_group_name);
    stage_move_home->setGoal("home");
    task.add(std::move(stage_move_home));

    // ── Tool-change sequence ──────────────────────────────────────────────────
    pickUpFingerTool (task, arm_group_name, hand_frame, interp, cart);
    placeFingerTool  (task, arm_group_name, hand_frame, interp, cart);

    // task.add(std::move(stage_move_home));
    
    pickUpSuctionTool(task, arm_group_name, hand_frame, interp, cart);
    placeSuctionTool (task, arm_group_name, hand_frame, interp, cart);

    return task;
}

// ─────────────────────────────────────────────────────────────────────────────
//  main
// ─────────────────────────────────────────────────────────────────────────────
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

    mtc_task_node->doTask("object2", false);

    spin_thread->join();
    rclcpp::shutdown();
    return 0;
}