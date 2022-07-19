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
#include <std_msgs/Int32.h>
#include <ctime>
#include <iostream>
#include <chrono>
#include <string>
using namespace std;
using namespace std::chrono;

//std_msgs::String msg;
ros::Publisher chatter_pub;
ros::Publisher count_pub;
// std::string ss = "unknown";
bool sensor_ok = 0;
int countPub = 0; // count how often the talker is publishing


/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
// %Tag(CALLBACK)%
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{

  static double prev_time = 0;
  double curr_time =ros::Time::now().toSec();
  double freq = 1/(curr_time - prev_time);

  ROS_INFO("old: %.2f new: %.2f, diff: %.2f",prev_time,curr_time,curr_time-prev_time);
  double actual = 12; // EDIT HERE FOR EXPECTED FREQUENCY
  if ((actual - 0.5 < freq) && (freq < actual + 0.5)) // checks if signal frequency is within 1 Hz of expected frequency
    {
      sensor_ok = 1;
    }
  else
    {
      sensor_ok = 0;
    }
  
  // ss = "frequency: " + std::to_string(sensor_ok);
  prev_time = curr_time;


  //ss << "frequency: " << freq;
  //msg1.data = ss.str();
  //ROS_INFO("Frequency: [%s]", ss.str()); 

}
// %EndTag(CALLBACK)%

void countCallback(const std_msgs::String::ConstPtr& msg)
{
  countPub += 1;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");

  ros::NodeHandle n;
  
  // chatter_pub = n.advertise<std_msgs::String>("chatterfreq", 10); //ros::Publisher
  chatter_pub = n.advertise<std_msgs::Bool>("sensor_ok", 10);
  count_pub = n.advertise<std_msgs::Int32>("count_pub", 10);
  ros::Subscriber sub = n.subscribe("chatter", 10, chatterCallback);
  ros::Subscriber subCount = n.subscribe("chatter", 10, countCallback);


  ros::Rate loop_rate(1); // EDITED BY KAILA FOR TESTING

  ros::AsyncSpinner spinner(5); // Specify # of threads
  spinner.start();

  std_msgs::Bool msgOut;
  std_msgs::Int32 countOut;

  while (ros::ok())

  {

  // std_msgs::String msgOut; 
  // msgOut.data = ss.c_str();
  
  msgOut.data = sensor_ok;
  chatter_pub.publish(msgOut); 

  countOut.data = countPub;
  count_pub.publish(countOut);


  // Loop timing
  // ros::spinOnce();

  
  loop_rate.sleep();
  // ros::waitForShutdown();

  }



  return 0;
}

