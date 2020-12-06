/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <cstdlib>

#include <ros/init.h>

#include <franka_gripper_sim.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "franka_gripper_sim_node");

    FrankaGripperSim franka_gripper_sim;

    franka_gripper_sim.start();

    /* 5fps are enough for gripper control. */
    ros::Rate loop_rate(5);

    while (ros::ok())
    {
        franka_gripper_sim.update();

        ros::spinOnce();

        loop_rate.sleep();
    }

    franka_gripper_sim.clean();

    return EXIT_SUCCESS;
}
