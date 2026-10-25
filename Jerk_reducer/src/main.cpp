// Copyright (c) 2024
// Licensed under the Apache License, Version 2.0

#include <ros/ros.h>
#include "jerk_reducer/jerk_reducer.hpp"

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "jerk_reducer");
  jerk_reducer::JerkReducer node;
  node.spin();
  return 0;
}