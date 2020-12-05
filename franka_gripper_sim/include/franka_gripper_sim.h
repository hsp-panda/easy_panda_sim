/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <actionlib/server/simple_action_server.h>

#include <franka_gripper/franka_gripper.h>

#include <ros/node_handle.h>


class FrankaGripperSim
{
public:

    FrankaGripperSim();

    ~FrankaGripperSim();

    void start();

private:
    ros::NodeHandle handle_;

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
