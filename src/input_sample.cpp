/*
 * Copyright 2023 Australian Centre For Robotics
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * 
 * You may obtain a copy of the License at
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 * Author: Darren Tsai
 */

// Press 'i' to take a sample, Press 'o' to start optimization, Press 'e' to terminate node
#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include <termios.h>

int getch()
{
  static struct termios oldt, newt;
  tcgetattr(STDIN_FILENO, &oldt);  // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                // disable buffering
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);  // apply new settings
  int c = getchar();                        // read character (non-blocking)
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "input_sample");
  ros::NodeHandle sample("~");
  ros::Publisher sample_publisher;
  std_msgs::Int8 flag;
  sample_publisher = sample.advertise<std_msgs::Int8>("/flag", 20);

  while (sample_publisher.getNumSubscribers() == 0)
  {
    ROS_ERROR("Waiting for a subscriber ...");
    sleep(2);
  }
  ROS_INFO("Found a subscriber!");
  std::cout << " Now, press an appropriate key ... " << std::endl;
  std::cout << " 'i' to take an input sample" << std::endl;
  std::cout << " 'o' to start the optimization process" << std::endl;
  std::cout << " 'e' to end the calibration process" << std::endl;
  ros::spinOnce();

  while (ros::ok())
  {
    int c = getch();
    if (c == 'i')  // flag to take an input sample
    {
      flag.data = 1;
      sample_publisher.publish(flag);
    }
    if (c == 'o')  // flag to start optimization
    {
      flag.data = 2;
      sample_publisher.publish(flag);
      ROS_INFO("starting optimization ...");
    }
    if (c == '\n')  // flag to use the input sample
    {
      flag.data = 4;
      sample_publisher.publish(flag);
    }
    if (c == 'e')  // flag to terminate the node; if you do Ctrl+C, press enter after that
    {
      ros::shutdown();
    }
  }
  return 0;
}
