//
// Created by boss on 2021-01-04.
//

#ifndef ROSSERIAL_MBED_ROS_H
#define ROSSERIAL_MBED_ROS_H

#include "ros/node_handle.h"
#include "HardwareImpl.h"

namespace ros
{
    typedef ros::NodeHandle_<HardwareImpl> NodeHandle;
}

#endif //ROSSERIAL_MBED_ROS_H
