#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <map>
#include <string>
#include <utility>
#include <vector>
#include <chrono>
#include <thread>
#include <ros/ros.h>
#include <iostream>



static const std::string PLANNING_GROUP = "panda_arm";


void setTarget(geometry_msgs::Pose &target_pose, double Ort_w, double Ort_x, double Ort_y,
               double Ort_z, double Pos_x, double Pos_y, double Pos_z) {
  target_pose.orientation.w = Ort_w;
  target_pose.orientation.x = Ort_x;
  target_pose.orientation.y = Ort_y;
  target_pose.orientation.z = Ort_z;
  target_pose.position.x = Pos_x;
  target_pose.position.y = Pos_y;
  target_pose.position.z = Pos_z;
}

class Demo
{
private:
  ros::NodeHandle node_handle;
  moveit::planning_interface::MoveGroupInterface *move_group;
  moveit_visual_tools::MoveItVisualTools *visual_tools;
  moveit::planning_interface::PlanningSceneInterface *planning_scene_interface;
  const moveit::core::JointModelGroup *joint_model_group;
  std::vector <moveit_msgs::CollisionObject> collision_objects;
  std::vector<moveit_msgs::ObjectColor> object_colors;

public:
  Demo() : node_handle("~")
  {
    move_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
    planning_scene_interface = new moveit::planning_interface::PlanningSceneInterface;
    visual_tools = new moveit_visual_tools::MoveItVisualTools("iiwa_link_0");
    joint_model_group = move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // Used my favorite planner.
    std::string PlannerId = "BKPIECE";

    // Setting parameters values to speed up the planning process.
    std::map <std::string, std::string> MapParam { {"range", "0.5"}, {"border_fraction", "0.14"},
                                                {"failed_expansion_score_factor", "0.92"},
                                                {"projection_evaluator", "joints(panda_joint1,panda_joint2)"},
                                                {"type", "geometric::BKPIECE"}, };
    move_group->setPlannerId(PlannerId);
    move_group->setPlanningTime(5);
    move_group->setPlannerParams(PlannerId, PLANNING_GROUP, MapParam);
  }

  bool planAndExecuteOnWayPoints(std::vector<geometry_msgs::Pose> waypoints, bool enable_contraint)
  {
    /*
     * This function solves the problem of moving through waypoints while avoiding obstacles.
     * It works by planning between each two followed waypoints while it concatenate different trajectories to execute them as one trajectory.
     * Here you can pass to this function not just two point, but a vector of "valid" points.
     */


    /*
     * This condition is to add constraint on the end-effector orientation (with big tolerances), just in case we do not get smooth planning */
    if (enable_contraint)
    {
      moveit_msgs::OrientationConstraint ocm;
      ocm.link_name = "panda_link7";
      ocm.header.frame_id = "panda_link0";
      ocm.orientation.x = 1.0;
      ocm.absolute_x_axis_tolerance = 0.5;
      ocm.absolute_y_axis_tolerance = 0.5;
      ocm.absolute_z_axis_tolerance = 0.5;
      ocm.weight = 1.0;

      moveit_msgs::Constraints test_constraints;
      test_constraints.orientation_constraints.push_back(ocm);
      move_group->setPathConstraints(test_constraints);
    }


    /*
     * The following is for creating the waypoints collision free planning
     * ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ */

    robot_trajectory::RobotTrajectory rt1(move_group->getCurrentState()->getRobotModel(), PLANNING_GROUP);
    robot_trajectory::RobotTrajectory rt2(move_group->getCurrentState()->getRobotModel(), PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = false;

    // this loop will move over the passed waypoints (you can provide multiple valid waypoints).
    for (int i=0;i<waypoints.size();i++)
    {
      move_group->setPoseTarget(waypoints.at(i));
      success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      rt2.setRobotTrajectoryMsg(*move_group->getCurrentState(), my_plan.trajectory_);

      // If it is first plan, so we save trajectory in rt1. Starting from second plan, we concatenate trajectories to rt1.
      if (i < 1)
        rt1.setRobotTrajectoryMsg(*move_group->getCurrentState(), my_plan.trajectory_);
      else
        rt1.append(rt2, 0.01);

      // We change the first state of the next plan to be the last state of the previous plan (in joints for sure, cartesian coordinates will cause a problem).
      robot_state::RobotState start_state(*move_group->getCurrentState());
      std::vector<double> joint_values;
      move_group->getCurrentState()->copyJointGroupPositions(joint_model_group, joint_values);
      int n = my_plan.trajectory_.joint_trajectory.points.size() - 1;
      joint_values = my_plan.trajectory_.joint_trajectory.points[n].positions;
      start_state.setJointGroupPositions(joint_model_group, joint_values);
      move_group->setStartState(start_state);
    }
    moveit_msgs::RobotTrajectory trajectory_msg;
    rt1.getRobotTrajectoryMsg(trajectory_msg);
    my_plan.trajectory_ = trajectory_msg;
    bool exec_success = false;
    if (success)
      exec_success = (move_group->execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    return exec_success;

  }

  void addObject(std::string ObjFrame, double r, double g, double b, const std::string& ObjId, double x, double y, double X,
                  double Y, double Z, double Ort_x, double Ort_y, double Ort_z, double Ort_w, bool attach){

    /*
     * Adding object to the environment.
     */
    moveit_msgs::CollisionObject collision_object;
    moveit_msgs::ObjectColor object_color;

    collision_object.header.frame_id = std::move(ObjFrame);

    collision_object.id = ObjId;

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(2);
    primitive.dimensions[0] = x;
    primitive.dimensions[1] = y;

    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = Ort_w;
    box_pose.orientation.x = Ort_x;
    box_pose.orientation.y = Ort_y;
    box_pose.orientation.z = Ort_z;
    box_pose.position.x = X;
    box_pose.position.y = Y;
    box_pose.position.z = Z;


    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    object_color.id = ObjId;
    object_color.color.a = 1;
    object_color.color.r = r;
    object_color.color.g = g;
    object_color.color.b = b;

    collision_objects.push_back(collision_object);
    object_colors.push_back(object_color);
    planning_scene_interface->addCollisionObjects(collision_objects, object_colors);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (attach)
      move_group->attachObject(collision_object.id);
  }

  void addEnv()
  {
    /*
     * In general, I use a function to build all the environment objects using "addObject" function.
     * The process would be efficient to build the environment through a loop reading objects details from a config file.
     * Here I did not add a config file as we have just one object. However, I kept the loop structure just to show.
     */

    std::vector<std::string> obj_id{"obs_1"};
    for (auto & i : obj_id)
      addObject("panda_link0", 0.1, 0.1, 0.5, i, 1.0, 0.1, 0.25, 0.0,
                 0.5, 0.0, 0.0, 0.0, 1.0, false);

  }

  void start()
  {
    /* Putting all together */

    addEnv();

    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose pos1, pos2;
    setTarget(pos1, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.65);
    setTarget(pos2, 0.0, 1.0, 0.0, 0.0, 0.25, -0.25, 0.65);
    waypoints.push_back(pos1);
    waypoints.push_back(pos2);
    planAndExecuteOnWayPoints(waypoints, true);


  }

};

int main(int argc, char **argv) {

  ros::init(argc, argv, "move_group_interface");
  ros::AsyncSpinner spinner(8);
  spinner.start();

  Demo go;
  go.start();


  ros::waitForShutdown();
  ros::shutdown();
  return 0;

}
