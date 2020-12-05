/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */


#include <franka_gripper_sim.h>

using franka_gripper::GraspAction;
using franka_gripper::GraspGoalConstPtr;
using franka_gripper::GraspResult;
using franka_gripper::HomingAction;
using franka_gripper::HomingGoalConstPtr;
using franka_gripper::HomingResult;
using franka_gripper::MoveAction;
using franka_gripper::MoveGoalConstPtr;
using franka_gripper::MoveResult;
using franka_gripper::StopAction;
using franka_gripper::StopGoalConstPtr;
using franka_gripper::StopResult;
using namespace std::placeholders;


FrankaGripperSim::FrankaGripperSim() :
    handle_("~"),
    action_grasp_
    (
        handle_,
        "grasp",
        [this](const auto& goal)
        {
            return action_callback<GraspAction, GraspGoalConstPtr, GraspResult>(&action_grasp_, std::bind(&FrankaGripperSim::grasp, this, std::placeholders::_1), goal);
        },
        false
    ),
    action_homing_
    (
        handle_,
        "homing",
        [this](const auto& goal)
        {
            return action_callback<HomingAction, HomingGoalConstPtr, HomingResult>(&action_homing_, std::bind(&FrankaGripperSim::homing, this, std::placeholders::_1), goal);
        },
        false
    ),
    action_move_
    (
        handle_,
        "move",
        [this](const auto& goal)
        {
            return action_callback<MoveAction, MoveGoalConstPtr, MoveResult>(&action_move_, std::bind(&FrankaGripperSim::move, this, std::placeholders::_1), goal);
        },
        false
    ),
    action_stop_
    (
        handle_,
        "stop",
        [this](const auto& goal)
        {
            return action_callback<StopAction, StopGoalConstPtr, StopResult>(&action_stop_, std::bind(&FrankaGripperSim::stop, this, std::placeholders::_1), goal);
        },
        false
    )
{}


FrankaGripperSim::~FrankaGripperSim()
{}


void FrankaGripperSim::start()
{
    action_grasp_.start();

    action_homing_.start();

    action_move_.start();

    action_stop_.start();
}


bool FrankaGripperSim::grasp(const franka_gripper::GraspGoalConstPtr&)
{
    return true;
}


bool FrankaGripperSim::homing(const HomingGoalConstPtr&)
{
    return true;
}


bool FrankaGripperSim::move(const MoveGoalConstPtr&)
{
    return true;
}


bool FrankaGripperSim::stop(const StopGoalConstPtr&)
{
    return true;
}
