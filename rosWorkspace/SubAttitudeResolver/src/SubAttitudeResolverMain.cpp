/**
 * @file SubAttitudeResolverMain.cpp
 *
 * @brief Driver program for the AttitudeResolver
 *
 * Revision History
 * May 29, 2012     Bryan Hansen    Initial implementation
 */

#include <stdio.h>

#include "SubAttitudeResolver.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "AttitudeResolver");

    SubAttitudeResolver attitudeResolver;

    attitudeResolver.run();

    return 0;
}
