/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */


#include <franka_gripper_sim.h>

#include <trajectory_msgs/JointTrajectory.h>

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
    publisher_(handle_.advertise<trajectory_msgs::JointTrajectory>(command_topic_name_, 1000)),
    subscriber_(handle_.subscribe(state_topic_name_, 1000, &FrankaGripperSim::feedback_callback, this)),
    duration_(ros::Duration(1.0, 0.0)),
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


void FrankaGripperSim::update()
{
    const std::lock_guard<std::mutex> lock(mutex_);

    trajectory_msgs::JointTrajectory msg;

    msg.joint_names.push_back("panda_finger_joint1");
    msg.joint_names.push_back("panda_finger_joint2");
    msg.points.resize(1);
    msg.points.at(0).positions.push_back(finger_position_des_);
    msg.points.at(0).positions.push_back(finger_position_des_);
    msg.points.at(0).time_from_start = duration_;

    publisher_.publish(msg);
}


void FrankaGripperSim::clean()
{
    publisher_.shutdown();
    subscriber_.shutdown();
}


double FrankaGripperSim::wrap_width(const double& width)
{
    if (width < 0)
        return 0.0;
    else if (width > max_width_)
        return max_width_;

    return width;
}


ros::Duration FrankaGripperSim::evaluate_duration(const double& finger_position_desired, const double& finger_position_feedback, const double& speed)
{
    if (speed <= 0)
        return ros::Duration(1.0, 0.0);

    double duration = std::fabs(finger_position_desired - finger_position_feedback) / speed;

    if (duration >= 1.0)
        return ros::Duration(duration, 0.0);
    else
        return ros::Duration(0.0, duration * 1e9);
}


void FrankaGripperSim::feedback_callback(const control_msgs::JointTrajectoryControllerStatePtr& msg)
{
    const std::lock_guard<std::mutex> lock(mutex_);

    finger_position_fb_ = (msg->actual.positions[0] + msg->actual.positions[1]) / 2.0;
}


bool FrankaGripperSim::grasp(const franka_gripper::GraspGoalConstPtr& goal)
{
    /**
     * Basic implementation, ignoring goal->{force, epsilon.{inner, outer}}
     * (see https://frankaemika.github.io/libfranka/classfranka_1_1Gripper.html#a047bc39267d66d6fb26c4c70669d68c2).
     */

    const std::lock_guard<std::mutex> lock(mutex_);

    finger_position_des_ = wrap_width(goal->width) / 2.0;
    duration_ = evaluate_duration(finger_position_des_, finger_position_fb_, goal->speed);

    return true;
}


bool FrankaGripperSim::homing(const HomingGoalConstPtr&)
{
    const std::lock_guard<std::mutex> lock(mutex_);

    finger_position_des_ = max_width_ / 2.0;

    duration_ = ros::Duration(1.0, 0.0);

    return true;
}


bool FrankaGripperSim::move(const MoveGoalConstPtr& goal)
{
    const std::lock_guard<std::mutex> lock(mutex_);

    finger_position_des_ = wrap_width(goal->width) / 2.0;
    duration_ = evaluate_duration(finger_position_des_, finger_position_fb_, goal->speed);

    return true;
}


bool FrankaGripperSim::stop(const StopGoalConstPtr&)
{
    const std::lock_guard<std::mutex> lock(mutex_);

    duration_ = ros::Duration(1.0, 0.0);

    return true;
}
