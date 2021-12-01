/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2017-2020, Waterplus http://www.6-robot.com
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the WaterPlus nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  FOOTPRINTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
/* @author Zhang Wanjie                                             */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include "dynamic_reconfigure/server.h"
#include "wpm2_tutorials/JointDegreeConfig.h"
#include <math.h>

static ros::Publisher joint_ctrl_pub;
static sensor_msgs::JointState ctrl_msg;

void cbJointDegree(wpm2_tutorials::JointDegreeConfig &config, uint32_t level) 
{
    ROS_WARN("关节 Joint_1= %d", config.Joint_1);
    ROS_WARN("关节 Joint_2= %d", config.Joint_2);
    ROS_WARN("关节 Joint_3= %d", config.Joint_3);
    ROS_WARN("关节 Joint_4= %d", config.Joint_4);
    ROS_WARN("关节 Joint_5= %d", config.Joint_5);
    ROS_WARN("关节 Joint_6= %d", config.Joint_6);
    ROS_WARN("手爪 Gripper= %d", config.Gripper);
    ctrl_msg.position[0] = config.Joint_1;
    ctrl_msg.position[1] = config.Joint_2;
    ctrl_msg.position[2] = config.Joint_3;
    ctrl_msg.position[3] = config.Joint_4;
    ctrl_msg.position[4] = config.Joint_5;
    ctrl_msg.position[5] = config.Joint_6;
    ctrl_msg.position[6] = config.Gripper;
    joint_ctrl_pub.publish(ctrl_msg);
}

int main(int argc, char** argv)
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "wpm2_joint_rcfg");

    ctrl_msg.name.resize(7);
    ctrl_msg.position.resize(7);
    ctrl_msg.velocity.resize(7);
    //关节角度
    ctrl_msg.position[0] = 0;
    ctrl_msg.position[1] = 0;
    ctrl_msg.position[2] = 0;
    ctrl_msg.position[3] = 0;
    ctrl_msg.position[4] = 0;
    ctrl_msg.position[5] = 0;
    ctrl_msg.position[6] = 35000; //手爪闭合
    //运动速度
    ctrl_msg.velocity[0] = 1000;
    ctrl_msg.velocity[1] = 1000;
    ctrl_msg.velocity[2] = 1000;
    ctrl_msg.velocity[3] = 1000;
    ctrl_msg.velocity[4] = 1000;
    ctrl_msg.velocity[5] = 1000;
    ctrl_msg.velocity[6] = 1000;

    ros::NodeHandle nh;
    joint_ctrl_pub = nh.advertise<sensor_msgs::JointState>("wpm2/joint_ctrl_degree", 10);

    dynamic_reconfigure::Server<wpm2_tutorials::JointDegreeConfig> server;
    dynamic_reconfigure::Server<wpm2_tutorials::JointDegreeConfig>::CallbackType f;

    f = boost::bind(&cbJointDegree, _1, _2);
    server.setCallback(f);

    ros::Time time = ros::Time(0);
    ros::Rate r(10);
    while(nh.ok())
    {
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
