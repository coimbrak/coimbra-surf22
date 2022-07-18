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
#include <ctime>
#include <iostream>
#include <chrono>
#include <string>
using namespace std;
using namespace std::chrono;

// double prev_time = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
// auto prev_time = std::chrono::system_clock::now();

//std_msgs::String msg;
ros::Publisher chatter_pub;
std::string ss = "unknown";


/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
// %Tag(CALLBACK)%
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  // double curr_time = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
  // double freq = 1000/(curr_time - prev_time);

  static double prev_time = 0;

  double curr_time =ros::Time::now().toSec();
  // auto curr_time = std::chrono::system_clock::now();
  double freq = curr_time - prev_time;

  ROS_INFO("old: %.2f new: %.2f, diff: %.2f",prev_time,curr_time,curr_time-prev_time);
  freq = 1/freq;
  ss = "frequency: " + std::to_string(freq);
  prev_time = curr_time;


  //ss << "frequency: " << freq;
  //msg1.data = ss.str();
  //ROS_INFO("Frequency: [%s]", ss.str()); // msg.data ?
  // prev_time = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();

}
// %EndTag(CALLBACK)%

int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");

  ros::NodeHandle n;
  
  chatter_pub = n.advertise<std_msgs::String>("chatterfreq", 10); //ros::Publisher
  ros::Subscriber sub = n.subscribe("chatter", 10, chatterCallback);


  ros::Rate loop_rate(1); // EDITED BY KAILA FOR TESTING

  // prev_time = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();

  ros::AsyncSpinner spinner(5); // Specify # of threads
  spinner.start();

  while (ros::ok())

  {

  std_msgs::String msgOut; 
  msgOut.data = ss.c_str();
  chatter_pub.publish(msgOut); 


  // Loop timing
  // ros::spinOnce();

  
  loop_rate.sleep();
  // ros::waitForShutdown();

  }



  return 0;
}

