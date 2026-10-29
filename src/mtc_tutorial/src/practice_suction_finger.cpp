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
#include <iostream>
#include <string>
#include <unordered_map>



static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
public:
    MTCTaskNode(const rclcpp::NodeOptions &options);

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

<<<<<<< HEAD
    void doTask(const std::string& object_name = "object1", const bool move=false, const bool reverse=false);
=======
    void doTask(const std::string& object_name = "object1", const bool initialize_suction=false, const bool initialize_finger_gripper=false, const bool reverse=false);
>>>>>>> caf6357 (moveit changes)

    void setupPlanningScene();
    

private:
    // Compose an MTC task from a series of stages.
<<<<<<< HEAD
    mtc::Task createTask(const std::string &object_name, const bool move=false, const bool reverse=false);
=======
    mtc::Task createTask(const std::string &object_name, const bool initialize_suction=false, const bool initialize_finger_gripper=false, const bool reverse=false);
>>>>>>> caf6357 (moveit changes)
    mtc::Task task_;
    rclcpp::Node::SharedPtr node_;
    moveit::planning_interface::PlanningSceneInterface psi_;
    std::unordered_map<std::string, geometry_msgs::msg::Pose> object_poses_;
};

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions &options)
    : node_{std::make_shared<rclcpp::Node>("mtc_node", options)}, psi_{}
{
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
    return node_->get_node_base_interface();
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

    


    // moveit_msgs::msg::CollisionObject obstacle;
    // obstacle.id = "obstacle";
    // obstacle.header.frame_id = "world";
    // obstacle.primitives.resize(1);
    // obstacle.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    // obstacle.primitives[0].dimensions = {1.0, 0.05, 0.25};  

    // geometry_msgs::msg::Pose pose3;
    // pose3.position.x = 0.1;
    // pose3.position.y = 0.3;
    // pose3.position.z = 0.125;
    // pose3.orientation.w = 1.0;
    // obstacle.pose = pose3;
    // psi_.applyCollisionObject(obstacle);


    // moveit_msgs::msg::CollisionObject obstacle2;
    // obstacle2.id = "obstacle2";
    // obstacle2.header.frame_id = "world";
    // obstacle2.primitives.resize(1);
    // obstacle2.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    // obstacle2.primitives[0].dimensions = {2.0, 2.0, 0.05};  

    // geometry_msgs::msg::Pose pose4;
    // pose4.position.x = 0.0;
    // pose4.position.y = 0.0;
    // pose4.position.z = -0.027;
    // pose4.orientation.w = 1.0;
    // obstacle2.pose = pose4;
    // psi_.applyCollisionObject(obstacle2);

    // moveit_msgs::msg::CollisionObject obstacle3;
    // obstacle3.id = "obstacle3";
    // obstacle3.header.frame_id = "world";
    // obstacle3.primitives.resize(1);
    // obstacle3.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    // obstacle3.primitives[0].dimensions = {1.0, 0.05, 0.25};  

    // geometry_msgs::msg::Pose pose5;
    // pose5.position.x = 0.1;
    // pose5.position.y = -0.3;
    // pose5.position.z = 0.125;
    // pose5.orientation.w = 1.0;
    // obstacle3.pose = pose5;
    // psi_.applyCollisionObject(obstacle3);

    
}

<<<<<<< HEAD
void MTCTaskNode::doTask(const std::string& object_name,const bool move, const bool reverse)
{
    task_ = createTask(object_name,move, reverse);
=======
void MTCTaskNode::doTask(const std::string& object_name,const bool initialize_suction, const bool initialize_finger_gripper, const bool reverse)
{
    task_ = createTask(object_name,initialize_suction, initialize_finger_gripper, reverse);
>>>>>>> caf6357 (moveit changes)

    try
    {
        task_.init();
    }
    catch (mtc::InitStageException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER, e);
        return;
    }
    while(!task_.plan())
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

    return;
}

<<<<<<< HEAD
mtc::Task MTCTaskNode::createTask(const std::string &object_name, const bool move, const bool reverse)
=======
mtc::Task MTCTaskNode::createTask(const std::string &object_name, const bool initialize_suction, const bool initialize_finger_gripper, const bool reverse)
>>>>>>> caf6357 (moveit changes)
{
    mtc::Task task;
    task.stages()->setName("pick and place task");
    task.loadRobotModel(node_);

    const auto &arm_group_name = "arm";
    const auto &finger_gripper_name = "finger_gripper";
    const auto &suction_gripper_name = "suction_gripper";
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

    auto stage_move_home =
        std::make_unique<mtc::stages::MoveTo>("move home", interpolation_planner);
    stage_move_home->setGroup(arm_group_name);
    stage_move_home->setGoal("home");
    task.add(std::move(stage_move_home));

    // auto stage_pick =
    //     std::make_unique<mtc::stages::MoveTo>("Pick finger Tool", interpolation_planner);
    // stage_pick->setGroup(arm_group_name);
    // stage_pick->setGoal("pick_finger_tool");
    // task.add(std::move(stage_pick));

<<<<<<< HEAD

    // auto stage_initialize =
    //     std::make_unique<mtc::stages::MoveTo>("initialize finger Tool", interpolation_planner);
    // stage_initialize->setGroup(suction_gripper_name);
    // stage_initialize->setGoal("initialize_finger_tool");
    // task.add(std::move(stage_initialize));
=======
   if (initialize_finger_gripper) {
        auto stage_initialize =
            std::make_unique<mtc::stages::MoveTo>("initialize finger Tool", interpolation_planner);
        stage_initialize->setGroup(suction_gripper_name);
        stage_initialize->setGoal("initialize_finger_tool");
        task.add(std::move(stage_initialize));


    }
    
>>>>>>> caf6357 (moveit changes)

    // auto stage_pick_suction =
    //     std::make_unique<mtc::stages::MoveTo>("Pick suction Tool", interpolation_planner);
    // stage_pick_suction->setGroup(arm_group_name);
    // stage_pick_suction->setGoal("pick_suction_tool");
<<<<<<< HEAD
    // task.add(std::move(stage_pick_suction));

    if (move==false) {
=======
    // task.add(std::move(stage_pick_suction));>

    if (initialize_suction) {
>>>>>>> caf6357 (moveit changes)
        auto stage_initialize2 =
            std::make_unique<mtc::stages::MoveTo>("initialize suction Tool", interpolation_planner);
        stage_initialize2->setGroup(suction_gripper_name);
        stage_initialize2->setGoal("initialize_suction_tool");
        task.add(std::move(stage_initialize2));
    }

    // auto stage_suction_on =
    //     std::make_unique<mtc::stages::MoveTo>("Turning Suction On", interpolation_planner);
    // stage_suction_on->setGroup(suction_gripper_name);
    // stage_suction_on->setGoal("suction_on");
    // task.add(std::move(stage_suction_on));

    // auto stage_suction_off =
    //     std::make_unique<mtc::stages::MoveTo>("Turning Suction Off", interpolation_planner);
    // stage_suction_off->setGroup(suction_gripper_name);
    // stage_suction_off->setGoal("suction_off");
    // task.add(std::move(stage_suction_off));

    // === 1. Open hand ===
    // auto stage_open_hand = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
    // stage_open_hand->setGroup(finger_gripper_name);
    // stage_open_hand->setGoal("open");
    // task.add(std::move(stage_open_hand));

       // // // === 4. Close hand (grasp) ===
    

    // // === 2. Move above object ===
    geometry_msgs::msg::PoseStamped pre_grasp_pose;
       
    if (reverse==false){
        RCLCPP_INFO_STREAM(LOGGER, "Picking " << object_name);
         pre_grasp_pose.header.frame_id = "world";
        auto obj_pose = object_poses_.at(object_name);
        pre_grasp_pose.pose.position.x = obj_pose.position.x;
        pre_grasp_pose.pose.position.y = obj_pose.position.y;
        pre_grasp_pose.pose.position.z = obj_pose.position.z + 0.05 + 0.20 +0.3; // object center z + half height + hover offset
        pre_grasp_pose.pose.orientation.x = 1.0;
        pre_grasp_pose.pose.orientation.y = 0.0;
        pre_grasp_pose.pose.orientation.z = 0.0;
        pre_grasp_pose.pose.orientation.w = 0.0;
     
    }
    else if (reverse==true && object_name=="object1")
    {
        pre_grasp_pose.header.frame_id = "world";
        pre_grasp_pose.pose.position.x = 0.0; // new location
        pre_grasp_pose.pose.position.y = 0.5;
        pre_grasp_pose.pose.position.z = 0.30 ;
        pre_grasp_pose.pose.orientation.x = 1.0;
        pre_grasp_pose.pose.orientation.y = 0.0;
        pre_grasp_pose.pose.orientation.z = 0.0;
        pre_grasp_pose.pose.orientation.w = 0.0;
        RCLCPP_INFO_STREAM(LOGGER, "Placing back " << object_name);
    }
    else if (reverse==true && object_name=="object2")
    {
        pre_grasp_pose.header.frame_id = "world";
        pre_grasp_pose.pose.position.x = 0.2; // new location
        pre_grasp_pose.pose.position.y = 0.7;
        pre_grasp_pose.pose.position.z = 0.30 ;
        pre_grasp_pose.pose.orientation.x = 1.0;
        pre_grasp_pose.pose.orientation.y = 0.0;
        pre_grasp_pose.pose.orientation.z = 0.0;
        pre_grasp_pose.pose.orientation.w = 0.0;
        RCLCPP_INFO_STREAM(LOGGER, "Placing back " << object_name);
    }
<<<<<<< HEAD
    if (move==true){
    auto stage_move_pregrasp =
        std::make_unique<mtc::stages::MoveTo>("move above object", interpolation_planner);
    stage_move_pregrasp->setGroup(arm_group_name);
    stage_move_pregrasp->setIKFrame(hand_frame);
    stage_move_pregrasp->setGoal(pre_grasp_pose);
    task.add(std::move(stage_move_pregrasp));
    }

    // auto stage_close_hand =
    //     std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
    // stage_close_hand->setGroup(finger_gripper_name);
    // stage_close_hand->setGoal("close");
    // task.add(std::move(stage_close_hand));
=======

    if (!initialize_finger_gripper && !initialize_suction) {

         auto stage_open_hand = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
        stage_open_hand->setGroup(finger_gripper_name);
        stage_open_hand->setGoal("open");
        task.add(std::move(stage_open_hand));
        auto stage_close_hand =
            std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
        stage_close_hand->setGroup(finger_gripper_name);
        stage_close_hand->setGoal("close");
        task.add(std::move(stage_close_hand));
        
        auto stage_move_pregrasp =
            std::make_unique<mtc::stages::MoveTo>("move above object", interpolation_planner);
        stage_move_pregrasp->setGroup(arm_group_name);
        stage_move_pregrasp->setIKFrame(hand_frame);
        stage_move_pregrasp->setGoal(pre_grasp_pose);
        task.add(std::move(stage_move_pregrasp));
    }

    
>>>>>>> caf6357 (moveit changes)

 

    // // === 3. Approach object (downwards) ===
    // auto stage_approach =
    //     std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
    // stage_approach->setGroup(arm_group_name);
    // stage_approach->setIKFrame(hand_frame);
    // stage_approach->setMinMaxDistance(0.01, 0.05);
    // geometry_msgs::msg::Vector3Stamped approach_vec;
    // approach_vec.header.frame_id = "world";
    // approach_vec.vector.z = -0.1; // move down 10cm
    // stage_approach->setDirection(approach_vec);
    // task.add(std::move(stage_approach));

    // auto stage_collision = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
    // stage_collision->allowCollisions(object_name,
    //                                  task.getRobotModel()
    //                                      ->getJointModelGroup(finger_gripper_name)
    //                                      ->getLinkModelNamesWithCollisionGeometry(),
    //                                  true);
    // task.add(std::move(stage_collision));

    // // // === 4. Close hand (grasp) ===
    // auto stage_close_hand =
    //     std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
    // stage_close_hand->setGroup(finger_gripper_name);
    // stage_close_hand->setGoal("close");
    // task.add(std::move(stage_close_hand));

    // // // === 5. Lift object ===

    // auto stage_attach = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
    // stage_attach->attachObject(object_name, hand_frame);
    // //   attach_object_stage = stage_attach.get();
    // task.add(std::move(stage_attach));

    // auto stage_lift =
    //     std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
    // stage_lift->setGroup(arm_group_name);
    // stage_lift->setIKFrame(hand_frame);
    // stage_lift->setMinMaxDistance(0.01, 0.05);
    // geometry_msgs::msg::Vector3Stamped lift_vec;
    // lift_vec.header.frame_id = "world";
    // lift_vec.vector.z = 0.15; // lift up 15 cm
    // stage_lift->setDirection(lift_vec);
    // task.add(std::move(stage_lift));

    // // // === 6. Move to place position ===
    // geometry_msgs::msg::PoseStamped place_pose;
    // if (object_name == "object1" && reverse==false) {
    //     // geometry_msgs::msg::PoseStamped place_pose;
    //     place_pose.header.frame_id = "world";
    //     place_pose.pose.position.x = 0.0; // new location
    //     place_pose.pose.position.y = 0.5;
    //     place_pose.pose.position.z = 0.30;
    //     place_pose.pose.orientation.x = 1.0;
    //     place_pose.pose.orientation.y = 0.0;
    //     place_pose.pose.orientation.z = 0.0;
    //     place_pose.pose.orientation.w = 0.0;
    //     // RCLCPP_INFO_STREAM(LOGGER, "Placing at location of object2");
    // }
    // else if (object_name == "object2" && reverse==false) {
    //     place_pose.header.frame_id = "world";
    //     place_pose.pose.position.x = 0.2; // new location
    //     place_pose.pose.position.y = 0.7;
    //     place_pose.pose.position.z = 0.30;
    //     place_pose.pose.orientation.x = 1.0;
    //     place_pose.pose.orientation.y = 0.0;
    //     place_pose.pose.orientation.z = 0.0;
    //     place_pose.pose.orientation.w = 0.0;
    //     // RCLCPP_INFO_STREAM(LOGGER, "Placing at location of object1");
    // }
    // else if (reverse==true) {
    //     place_pose.header.frame_id = "world";
    //     place_pose.pose.position.x = object_poses_.at(object_name).position.x;
    //     place_pose.pose.position.y = object_poses_.at(object_name).position.y;
    //     place_pose.pose.position.z = 0.30;
    //     place_pose.pose.orientation.x = 1.0;
    //     place_pose.pose.orientation.y = 0.0;
    //     place_pose.pose.orientation.z = 0.0;
    //     place_pose.pose.orientation.w = 0.0;
    //     RCLCPP_INFO_STREAM(LOGGER, "Placing back at original location of " << object_name);
    // }

    // auto stage_move_place =
    //     std::make_unique<mtc::stages::MoveTo>("move to place", interpolation_planner);
    // stage_move_place->setGroup(arm_group_name);
    // stage_move_place->setIKFrame(hand_frame);
    // stage_move_place->setGoal(place_pose);
    // task.add(std::move(stage_move_place));

    // // // === 7. Lower to place ===
    // auto stage_lower = std::make_unique<mtc::stages::MoveRelative>("lower to place", cartesian_planner);
    // stage_lower->setGroup(arm_group_name);
    // stage_lower->setIKFrame(hand_frame);
    // stage_lower->setMinMaxDistance(0.01, 0.05);
    // geometry_msgs::msg::Vector3Stamped lower_vec;
    // lower_vec.header.frame_id = "world";
    // lower_vec.vector.z = -0.12; // move down 12cm
    // stage_lower->setDirection(lower_vec);
    // task.add(std::move(stage_lower));

    // // // === 8. Open hand (release) ===
    // auto stage_open_release =
    //     std::make_unique<mtc::stages::MoveTo>("open hand release", interpolation_planner);
    // stage_open_release->setGroup(finger_gripper_name);
    // stage_open_release->setGoal("open");
    // task.add(std::move(stage_open_release));


    // auto stage_forbid = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
    // stage_forbid->allowCollisions(object_name,
    //                          task.getRobotModel()
    //                              ->getJointModelGroup(finger_gripper_name)
    //                              ->getLinkModelNamesWithCollisionGeometry(),
    //                          false);
    // task.add(std::move(stage_forbid));


    // auto stage_detach = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
    // stage_detach->detachObject(object_name, hand_frame);
    // task.add(std::move(stage_detach));

    

    // // // === 9. Retreat back ===
    // auto stage_retreat =
    //     std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
    // stage_retreat->setGroup(arm_group_name);
    // stage_retreat->setIKFrame(hand_frame);
    // stage_retreat->setMinMaxDistance(0.01, 0.05);
    // geometry_msgs::msg::Vector3Stamped retreat_vec;
    // retreat_vec.header.frame_id = "world";
    // retreat_vec.vector.z = 0.15; // move up after placing
    // stage_retreat->setDirection(retreat_vec);
    // task.add(std::move(stage_retreat));

    // // // === 10. Return home ===
    // auto stage_move_home =
    //     std::make_unique<mtc::stages::MoveTo>("move home", interpolation_planner);
    // stage_move_home->setGroup(arm_group_name);
    // stage_move_home->setGoal("home");
    // task.add(std::move(stage_move_home));

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
    // Change to "object2" to select it; the hover pose will automatically adjust based on the object's position
<<<<<<< HEAD
    mtc_task_node->doTask("object2", false,  false);
    rclcpp::sleep_for(std::chrono::seconds(30));
    mtc_task_node->doTask("object2", true,  false);
    // mtc_task_node->doTask("object1", false);
    // mtc_task_node->doTask("object1", true);
    // mtc_task_node->doTask("object2", true);
=======
    mtc_task_node->doTask("object2", false, true,  false);
    rclcpp::sleep_for(std::chrono::seconds(10));
    mtc_task_node->doTask("object2", false, false,  false);
  
>>>>>>> caf6357 (moveit changes)
                                                        

    spin_thread->join();
    rclcpp::shutdown();
    return 0;
}