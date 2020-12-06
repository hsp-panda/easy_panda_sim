/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <actionlib/server/simple_action_server.h>

#include <control_msgs/JointTrajectoryControllerState.h>

#include <franka_gripper/franka_gripper.h>

#include <ros/node_handle.h>

#include <mutex>
#include <string>


class FrankaGripperSim
{
public:

    FrankaGripperSim();

    ~FrankaGripperSim();

    void start();

    void update();

    void clean();

private:

    double wrap_width(const double& width);

    ros::Duration evaluate_duration(const double& finger_position_desired, const double& finger_position_feedback, const double& speed);

    void feedback_callback(const control_msgs::JointTrajectoryControllerStatePtr&);

    const std::string command_topic_name_ = "/franka_gripper_sim_controller/command";

    const std::string state_topic_name_ = "/franka_gripper_sim_controller/state";

    double finger_position_des_ = 0.0;

    double finger_position_fb_ = 0.0;

    double max_width_ = 0.08;

    std::mutex mutex_;

    ros::NodeHandle handle_;

    ros::Publisher publisher_;

    ros::Subscriber subscriber_;

    ros::Duration duration_;

    actionlib::SimpleActionServer<franka_gripper::GraspAction> action_grasp_;

    actionlib::SimpleActionServer<franka_gripper::HomingAction> action_homing_;

    actionlib::SimpleActionServer<franka_gripper::MoveAction> action_move_;

    actionlib::SimpleActionServer<franka_gripper::StopAction> action_stop_;

    bool grasp(const franka_gripper::GraspGoalConstPtr&);

    bool homing(const franka_gripper::HomingGoalConstPtr&);

    bool move(const franka_gripper::MoveGoalConstPtr&);

    bool stop(const franka_gripper::StopGoalConstPtr&);

    template <typename T_action, typename T_goal, typename T_result>
    void action_callback(actionlib::SimpleActionServer<T_action>* server, std::function<bool(const T_goal&)> handler, const T_goal& goal)
    {
        T_result result;

        result.success = handler(goal);
        server->setSucceeded(result);
    }
};
