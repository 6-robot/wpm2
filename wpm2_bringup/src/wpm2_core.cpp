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
/*!******************************************************************
 @author     ZhangWanjie
 ********************************************************************/

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int32MultiArray.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/GripperCommandAction.h>
#include "driver/WPM2_Driver.h"
#include <math.h>
#include <vector>

using namespace std;

typedef struct
{
    float position[7];
    int velocity[7];
}st_wpm_pose;

typedef struct
{
    float fGapSize;
    int nGripperPos;
    double fPosePerMM;
}stGripperPos;
static vector<stGripperPos> arGripperPos;

static CWPM2_Driver wpm2;
static double fDegToAng = 3.1415926/180;
static double fAngToDeg = 180/3.1415926;
static double pos_send[7];
static int vel_send[7];
static st_wpm_pose tmpPose;
static vector<st_wpm_pose> arPose;
static int nExecIndex = 0;
static bool bExecPath = false;
static bool bExecToGoal = true;

typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> TrajectoryServer;
typedef actionlib::SimpleActionServer<control_msgs::GripperCommandAction> GripperServer;

bool poseArrived()
{
    bool bArrived = true;
    for(int i=0;i<6;i++)
    {
        double fDegDiff = fabs(arPose[nExecIndex].position[i] - wpm2.nRecvJointPos[i]*0.01);
        if(fDegDiff > 1)
        {
            bArrived = false;
        }
    }
    return bArrived;
}

// 响应 Move_Group 的轨迹执行
void executeTrajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal, TrajectoryServer* as)
{
    arPose.clear();
    int nrOfPoints = goal->trajectory.points.size();
    ROS_INFO("Trajectory with %d positions received \n", nrOfPoints);
    if(bExecToGoal == false)
    {
        nExecIndex = 0;
    }
    else
    {
        nExecIndex = nrOfPoints - 1;
    }
    for(int i=0; i<nrOfPoints; i++)
    {
        int nPos =  goal->trajectory.points[i].positions.size();
        //ROS_WARN("point[%d] hase %d positions\n",i, nPos);
        if(nPos > 7)
        nPos = 7;
        for(int j=0;j<nPos;j++)
        {
            tmpPose.position[j] = goal->trajectory.points[i].positions[j] * fAngToDeg;
            tmpPose.velocity[j] = 2000;
        }
        arPose.push_back(tmpPose);

        /////////////////////////////////////////////////
        // ROS_WARN("point[%d] pos %.1f %.1f %.1f %.1f %.1f %.1f",i,
        // goal->trajectory.points[i].positions[0],
        // goal->trajectory.points[i].positions[1],
        // goal->trajectory.points[i].positions[2],
        // goal->trajectory.points[i].positions[3],
        // goal->trajectory.points[i].positions[4],
        // goal->trajectory.points[i].positions[5]);
        //  ROS_WARN("point[%d] vel %.1f %.1f %.1f %.1f %.1f %.1f",i,
        // goal->trajectory.points[i].velocities[0],
        // goal->trajectory.points[i].velocities[1],
        // goal->trajectory.points[i].velocities[2],
        // goal->trajectory.points[i].velocities[3],
        // goal->trajectory.points[i].velocities[4],
        // goal->trajectory.points[i].velocities[5]);
        // ROS_WARN("point[%d] acc %.1f %.1f %.1f %.1f %.1f %.1f",i,
        // goal->trajectory.points[i].accelerations[0],
        // goal->trajectory.points[i].accelerations[1],
        // goal->trajectory.points[i].accelerations[2],
        // goal->trajectory.points[i].accelerations[3],
        // goal->trajectory.points[i].accelerations[4],
        // goal->trajectory.points[i].accelerations[5]);
        // ROS_WARN("time_from_start = %f",goal->trajectory.points[i].time_from_start.toSec());
        // ROS_WARN("--------------------------------------------------");
        /////////////////////////////////////////////////
    }
    
    bExecPath = true;
    
    ros::Rate r(30);
    while( bExecPath == true)
    {
        if(poseArrived() == true)
        {
            nExecIndex ++;
        }
        if(nExecIndex >= arPose.size())
        {
            // 执行完毕
            bExecPath = false;
            ROS_INFO("ExecPath done!");
        }
        else
        {
            //未执行完毕
            for(int i=0;i<6;i++)
            {
                pos_send[i] = arPose[nExecIndex].position[i];
                vel_send[i] = arPose[nExecIndex].velocity[i];
            }
            
            // 速度优化
            double fTmpPosDiff[6];
            for(int i=0;i<6;i++)
            {
                fTmpPosDiff[i] = fabs(pos_send[i] - wpm2.nRecvJointPos[i]*0.01);
            }
            // 找出位置差最大的值,以它为最大速度
            double fDiffMax = 0;
            int nDiffMaxIndex = 0;
            for(int i=0;i<6;i++)
            {
                if(fTmpPosDiff[i] > fDiffMax)
                {
                    fDiffMax = fTmpPosDiff[i];
                    nDiffMaxIndex = i;
                }
            }
            // 计算运动速度
            int nMaxVelocity = 8000;
            if(fDiffMax > 0)
            {
                for(int i=0;i<6;i++)
                {
                    double tmpVel = fTmpPosDiff[i];
                    tmpVel /= fDiffMax;
                    tmpVel *= nMaxVelocity;
                    vel_send[i] = tmpVel;
                }
            }
            
            wpm2.SetJoints(pos_send,vel_send);
        }

        r.sleep();
    }

    as->setSucceeded();
}

void InitGripperPosVal()
{
    stGripperPos tmpGP;
    tmpGP.fGapSize = 0;
    tmpGP.nGripperPos = 38000;
    tmpGP.fPosePerMM = 0;
    arGripperPos.push_back(tmpGP);

    tmpGP.fGapSize = 0.006;
    tmpGP.nGripperPos = 35000;
    arGripperPos.push_back(tmpGP);

    tmpGP.fGapSize = 0.020;
    tmpGP.nGripperPos = 30000;
    arGripperPos.push_back(tmpGP);

    tmpGP.fGapSize = 0.035;
    tmpGP.nGripperPos = 25000;
    arGripperPos.push_back(tmpGP);

    tmpGP.fGapSize = 0.049;
    tmpGP.nGripperPos = 20000;
    arGripperPos.push_back(tmpGP);

    tmpGP.fGapSize = 0.063;
    tmpGP.nGripperPos = 15000;
    arGripperPos.push_back(tmpGP);

    tmpGP.fGapSize = 0.076;
    tmpGP.nGripperPos = 10000;
    arGripperPos.push_back(tmpGP);

    tmpGP.fGapSize = 0.087;
    tmpGP.nGripperPos = 5000;
    arGripperPos.push_back(tmpGP);

    tmpGP.fGapSize = 0.099;
    tmpGP.nGripperPos = 0;
    arGripperPos.push_back(tmpGP);

    int nNumGP = arGripperPos.size();
    for(int i=1;i<nNumGP;i++)
    {
        double fDiffSize = fabs(arGripperPos[i].fGapSize - arGripperPos[i-1].fGapSize);
        int nDiffPos = fabs(arGripperPos[i].nGripperPos - arGripperPos[i-1].nGripperPos);
        arGripperPos[i].fPosePerMM = fDiffSize/nDiffPos;
    }
}

// 手爪位置计算
int CalGripperPos(float inGapSize)
{
    int nNumGP = arGripperPos.size();
    int nRetGripperPos = 0;
    if(nNumGP > 0)
    {
        int nIndexGP = nNumGP - 1;
        if(inGapSize >= arGripperPos[nIndexGP].fGapSize)
        {
            nRetGripperPos = arGripperPos[nIndexGP].nGripperPos;
            return nRetGripperPos;
        }
        for(int i=1;i<nNumGP;i++)
        {
            if(inGapSize < arGripperPos[i].fGapSize)
            {
                nIndexGP = i;
                break;
            }
        }
        if(nIndexGP < nNumGP)
        {
            double fDiffGapSize = fabs(inGapSize - arGripperPos[nIndexGP].fGapSize);
            int nDiffGripperPos = (fDiffGapSize/arGripperPos[nIndexGP].fPosePerMM);
            nRetGripperPos = arGripperPos[nIndexGP].nGripperPos + nDiffGripperPos;
        }
    }
    return nRetGripperPos;
}

// 响应 GripperCommand 回调函数
void executeGripper(const control_msgs::GripperCommandGoalConstPtr & goal, GripperServer* as)
{
	float gapSize = goal->command.position;
    float maxEffort = goal->command.max_effort;

    int nGripperPos = CalGripperPos(gapSize);
    ROS_INFO("[executeGripper]gapSize = %f, gripprPos = %d", gapSize, nGripperPos);

    // 执行指令
    pos_send[6] = nGripperPos;
	wpm2.SetJoints(pos_send,vel_send);

    // 监测执行目标是否完成
    while(ros::ok())
    {
        int nDiff = abs(nGripperPos - wpm2.nRecvJointPos[6]);
        if(nDiff < 100)
        {
            break;
        }
    }

	as->setSucceeded();
}

//角度控制机械臂
void JointCtrlDegreeCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    bExecPath = false;
    int nNumJoint = msg->position.size();
    if(nNumJoint > 7)
    {
        nNumJoint = 7;
    }
    ROS_INFO("----------------------------");
    for(int i=0;i<nNumJoint;i++)
    {
        pos_send[i] = msg->position[i];
        vel_send[i] = msg->velocity[i];
        ROS_INFO("[JointCtrlDegreeCallback] %d - %s = %.2f", i, msg->name[i].c_str(),msg->position[i]);
    }
    wpm2.SetJoints(pos_send,vel_send);
}

//弧度控制机械臂
void JointCtrlRadianCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    bExecPath = false;
    int nNumJoint = msg->position.size();
    if(nNumJoint > 7)
    {
        nNumJoint = 7;
    }
    ROS_INFO("----------------------------");
    for(int i=0;i<nNumJoint;i++)
    {
        if(i != 6)
        pos_send[i] = msg->position[i] * fAngToDeg;
        else
        pos_send[i] = msg->position[i]; //手爪
        vel_send[i] = msg->velocity[i];
        ROS_INFO("[JointCtrlRadianCallback] %d - %s = %.2f (%.0f Deg)", i, msg->name[i].c_str(),msg->position[i],pos_send[i]);
    }
    wpm2.SetJoints(pos_send,vel_send);
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"wpm2_core");
    ros::NodeHandle n;

    ros::Subscriber joint_ctrl_degree_sub = n.subscribe("/wpm2/joint_ctrl_degree",30,&JointCtrlDegreeCallback);
    ros::Subscriber joint_ctrl_radian_sub = n.subscribe("/wpm2/joint_ctrl_radian",30,&JointCtrlRadianCallback);
    
	TrajectoryServer tserver(n, "wpm2_controller/follow_joint_trajectory", boost::bind(&executeTrajectory, _1, &tserver), false);
  	ROS_INFO("TrajectoryActionServer: Starting");
    tserver.start();
    InitGripperPosVal();
    GripperServer gserver(n, "wpm2_gripper_controller/gripper_command", boost::bind(&executeGripper, _1, &gserver), false);
 	ROS_INFO("GripperActionServer: Starting");
    gserver.start();

    ros::NodeHandle n_param("~");
    std::string strSerialPort;
    n_param.param<std::string>("serial_port", strSerialPort, "/dev/ftdi");
    wpm2.Open(strSerialPort.c_str(),115200);
    ROS_WARN("WPM2_Port = %s",strSerialPort.c_str());
    n_param.param<bool>("exec_to_goal", bExecToGoal, true);

    ros::Publisher joint_state_pub = n.advertise<sensor_msgs::JointState>("/joint_states",100);
    sensor_msgs::JointState msg;
    std::vector<std::string> joint_name(7);
    std::vector<double> joint_pos(7);

    joint_name[0] = "joint1_joint2";
    joint_name[1] = "joint2_joint3";
    joint_name[2] = "joint3_joint4";
    joint_name[3] = "joint4_joint5";
    joint_name[4] = "joint5_joint6";
    joint_name[5] = "joint6_palm";
    joint_name[6] = "wpm2_palm_finger";
    joint_pos[0] = 0.0f;
    joint_pos[1] = 0.0f;
    joint_pos[2] = 0.0f;
    joint_pos[3] = 0.0f;
    joint_pos[4] = 0.0f;
    joint_pos[5] = 0.0f;
    joint_pos[6] = 0.0f;  
    
    ros::Rate r(30);

    r.sleep();
    for(int i=0;i<7;i++)
    {
        pos_send[i] = 0;
        vel_send[i] = 2000;
    }
    pos_send[6] = 25000;//手爪
    wpm2.SetJoints(pos_send,vel_send);
    int nCount = 0;

    while(n.ok())
    {
        wpm2.ReadNewData();
        //ROS_INFO("[wpm2.nParseCount]= %d",wpm2.nParseCount);
        
        msg.header.stamp = ros::Time::now();
        msg.header.seq ++;

        double fTmp = 0;
        for(int i=0;i<6;i++)
        {
            fTmp = wpm2.nRecvJointPos[i];
            fTmp *= 0.01;
            joint_pos[i] = fTmp*fDegToAng;
        }
        fTmp = 38000 - wpm2.nRecvJointPos[6];
        if(fTmp < 0) fTmp = 0;
        fTmp *= 0.5/38000;
        joint_pos[6] = fTmp;

        msg.name = joint_name;
        msg.position = joint_pos;
        joint_state_pub.publish(msg);

        ros::spinOnce();
        r.sleep();
    }
}