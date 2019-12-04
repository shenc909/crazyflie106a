/*
 * Copyright (c) 2017, The Regents of the University of California (Regents).
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *
 *    3. Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Please contact the author(s) of this library if you have any questions.
 * Authors: David Fridovich-Keil   ( dfk@eecs.berkeley.edu )
 */

///////////////////////////////////////////////////////////////////////////////
//
// Defines the StateConverter class, which listens for new crazyflie state
// messages and immediately republishes them as fastrack state messages.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef FASTRACK_CRAZYFLIE_DEMOS_STATE_CONVERTER_H
#define FASTRACK_CRAZYFLIE_DEMOS_STATE_CONVERTER_H

#include <fastrack/utils/types.h>
#include <fastrack/utils/uncopyable.h>

#include <fastrack_msgs/State.h>
#include <crazyflie_msgs/PositionVelocityStateStamped.h>

#include <ros/ros.h>

namespace fastrack {
namespace crazyflie {

class StateConverter : private Uncopyable {
public:
  ~StateConverter() {}
  explicit StateConverter()
    : initialized_(false) {}

  // Initialize this class with all parameters and callbacks.
  bool Initialize(const ros::NodeHandle& n);

private:
  bool LoadParameters(const ros::NodeHandle& n);
  bool RegisterCallbacks(const ros::NodeHandle& n);

  // Callback for processing new state signals.
  void StateCallback(const crazyflie_msgs::PositionVelocityStateStamped::ConstPtr& msg);

  // Publishers/subscribers and related topics.
  ros::Subscriber raw_state_sub_;
  ros::Publisher fastrack_state_pub_;

  std::string raw_state_topic_;
  std::string fastrack_state_topic_;

  // Naming and initialization.
  std::string name_;
  bool initialized_;
};

} //\namespace crazyflie
} //\namespace fastrack

#endif
