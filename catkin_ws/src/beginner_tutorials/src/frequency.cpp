/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// %Tag(FULLTEXT)%
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <ctime>
#include <iostream>
#include <chrono>
#include <string>
using namespace std;
using namespace std::chrono;

//std_msgs::String msg;
ros::Publisher chatter_pub;
// std::string ss = "unknown";
bool sensor_ok = 0;
// Add a new global variable "last_freq_update_" for each chatter node
double last_freq_update0_ = 0;
double last_freq_update1_ = 0;
double last_freq_update2_ = 0;

// Copy/paste a new chatterCallback for each chatter node 
// Change "actual" value as needed and change ref number for "last_freq_update_" for each new chatterCallback
void chatterCallback0(const std_msgs::String::ConstPtr& msg)
{

  static double prev_time = 0;
  double curr_time =ros::Time::now().toSec();
  double freq = 1/(curr_time - prev_time);

  double actual = 10; // EDIT HERE FOR EXPECTED FREQUENCY FROM CHATTER TOPIC
  if ((actual - 0.5 < freq) && (freq < actual + 0.5)) // checks if signal frequency is within 1 Hz of expected frequency
    {
      sensor_ok = 1;
    }
  else
    {
      sensor_ok = 0;
    }

  prev_time = curr_time;
  last_freq_update0_ = ros::Time::now().toSec(); // record the last time the callback was called

}

void chatterCallback1(const std_msgs::String::ConstPtr& msg)
{

  static double prev_time = 0;
  double curr_time =ros::Time::now().toSec();
  double freq = 1/(curr_time - prev_time);

  double actual = 20; // EDIT HERE FOR EXPECTED FREQUENCY FROM CHATTER TOPIC
  if ((actual - 0.5 < freq) && (freq < actual + 0.5)) // checks if signal frequency is within 1 Hz of expected frequency
    {
      sensor_ok = 1;
    }
  else
    {
      sensor_ok = 0;
    }

  prev_time = curr_time;
  last_freq_update1_ = ros::Time::now().toSec(); // record the last time the callback was called

}

void chatterCallback2(const std_msgs::String::ConstPtr& msg)
{

  static double prev_time = 0;
  double curr_time =ros::Time::now().toSec();
  double freq = 1/(curr_time - prev_time);

  double actual = 30; // EDIT HERE FOR EXPECTED FREQUENCY FROM CHATTER TOPIC
  if ((actual - 0.5 < freq) && (freq < actual + 0.5)) // checks if signal frequency is within 1 Hz of expected frequency
    {
      sensor_ok = 1;
    }
  else
    {
      sensor_ok = 0;
    }

  prev_time = curr_time;
  last_freq_update2_ = ros::Time::now().toSec(); // record the last time the callback was called

}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  chatter_pub = n.advertise<std_msgs::Bool>("sensor_ok", 10);
  // Copy/paste following for each chatter node (assign a new chatterCallback to each node)
  ros::Subscriber sub0 = n.subscribe("/node0/chatter", 10, chatterCallback0);
  ros::Subscriber sub1 = n.subscribe("/node1/chatter", 10, chatterCallback1);
  ros::Subscriber sub2 = n.subscribe("/node2/chatter", 10, chatterCallback2);


  ros::Rate loop_rate(1); // Rate frequency.cpp is publishing

  ros::AsyncSpinner spinner(5); // Specify # of threads
  spinner.start();

  std_msgs::Bool msgOut;

  while (ros::ok())
  {

  double curr_freq_update = ros::Time::now().toSec();
  // Edit conditional statement below to match # of chatter nodes
  if (((curr_freq_update - last_freq_update0_) > 1) || ((curr_freq_update - last_freq_update1_) > 1) || ((curr_freq_update - last_freq_update2_) > 1))
  {
    sensor_ok = 0; // sensor is not ok if talker is not publishing
  }

  // ROS_INFO("last: %.2f,   new: %.2f,   diff: %.2f",last_freq_update0_,curr_freq_update,curr_freq_update-last_freq_update0_);
  msgOut.data = sensor_ok;
  chatter_pub.publish(msgOut); 


  // Loop timing
  // ros::spinOnce();

  
  loop_rate.sleep();
  // ros::waitForShutdown();

  }



  return 0;
}

