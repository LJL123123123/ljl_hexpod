/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, Qiayuan Liao
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

//
// Created by qiayuan on 12/27/20.
//

#include "legged_hexapod_hw/HexapodHW.h"

#include <legged_hw/LeggedHWLoop.h>

int main(int argc, char** argv) {

  
  ros::init(argc, argv, "legged_hexapod_hw");
  ros::NodeHandle nh;
  ros::NodeHandle robotHwNh("~");

  IrisTac_Project Project;
  DM_USB2CAN test = DM_USB2CAN("DM_USB2CAN1",100,0.001,921600,"can1",128,1024,8);
  DM_USB2CAN test1 = DM_USB2CAN("DM_USB2CAN2",100,0.001,921600,"can2",128,1024,8);
  DM_USB2CAN test2 = DM_USB2CAN("DM_USB2CAN3",100,0.001,921600,"can3",128,1024,8);
  DM_USB2CAN test3 = DM_USB2CAN("DM_USB2CAN4",100,0.001,921600,"can4",128,1024,8);
  DM_USB2CAN test4 = DM_USB2CAN("DM_USB2CAN5",100,0.001,921600,"can5",128,1024,8);
  DM_USB2CAN test5 = DM_USB2CAN("DM_USB2CAN6",100,0.001,921600,"can6",128,1024,8);

  Gloabl_Timer::Instance();
  PeriodicTaskManager::Instance();

  test.start();
  test1.start();
  test2.start();
  test3.start();
  test4.start();
  test5.start();
  
  
  Project.start();

  // Run the hardware interface node
  // -------------------------------

  // We run the ROS loop in a separate thread as external calls, such
  // as service callbacks loading controllers, can block the (main) control loop

  ros::AsyncSpinner spinner(3);
  spinner.start();

  try {
    // Create the hardware interface specific to your robot
    std::shared_ptr<legged::HexapodHW> hexapodHw = std::make_shared<legged::HexapodHW>();
    // Initialize the hardware interface:
    // 1. retrieve configuration from rosparam
    // 2. initialize the hardware and interface it with ros_control
    hexapodHw->initWithProject(nh, robotHwNh, Project);

    // Start the control loop
    legged::LeggedHWLoop controlLoop(nh, hexapodHw);

    // Wait until shutdown signal received
    ros::waitForShutdown();
  } catch (const ros::Exception& e) {
    ROS_FATAL_STREAM("Error in the hardware interface:\n"
                     << "\t" << e.what());
    return 1;
  }

  return 0;
}
