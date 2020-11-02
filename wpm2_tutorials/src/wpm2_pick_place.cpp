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
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/GripperCommandAction.h>

static tf::StampedTransform transform;

static float fPick_x = -0.3;
static float fPick_y = 0.0;
static float fPick_z = 0.6;

static float fPick_gap = 0.05;

static float fPlace_x = 0;
static float fPlace_y = 0.2;
static float fPlace_z = 0.6;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wpm2_pick_place");

    actionlib::SimpleActionClient<control_msgs::GripperCommandAction> acGripper("wpm2_gripper_controller/gripper_command", true);

    ros::NodeHandle node_handle; 
    ros::AsyncSpinner spinner(1);
    spinner.start();

    printf("[wpm2_pick_place] 延迟5秒,等待Rviz启动... \n");
    //sleep(5.0);

    printf("[wpm2_pick_place] 等待手爪服务启动... \n");
    acGripper.waitForServer();
    printf("[wpm2_pick_place] 准备进行轨迹规划... \n");
    moveit::planning_interface::MoveGroupInterface group("arm");

    // 设置机械臂末端的目标位置
    geometry_msgs::Pose target_pose;
    tf::Quaternion quat;
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    control_msgs::GripperCommandGoal gripperCmd;
    bool gripper_success = true;
    moveit::planning_interface::MoveItErrorCode plan_success;
    bool exec_success = true;

    /********* 抓取阶段 *********/
    printf("[1]调整到准备抓取姿态 \n");
    quat.setRPY(0.0, -1.57, 0.0);
    transform.setRotation(quat);
    target_pose.orientation.x= transform.getRotation().getX();
    target_pose.orientation.y = transform.getRotation().getY();
    target_pose.orientation.z = transform.getRotation().getZ();
    target_pose.orientation.w = transform.getRotation().getW();
    target_pose.position.x = fPick_x + 0.2;
    target_pose.position.y = fPick_y;
    target_pose.position.z = fPick_z;
    group.setPoseTarget(target_pose);

    plan_success = group.plan(plan);
    if(plan_success == moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
        printf("[wpm2_pick_place] 轨迹规划成功! 开始执行! \n");
        // 机械臂按照规划的轨迹运动
        group.execute(plan);
    }
    else
    {
        printf("[wpm2_pick_place] 轨迹规划失败,请检查规划目标的参数 \n");
        return 0;
    }

    printf("[2]张开手爪 \n");
    gripperCmd.command.position = 0.09;
    acGripper.sendGoal(gripperCmd);
    gripper_success = acGripper.waitForResult(ros::Duration(10.0));
    if (gripper_success == false)
    {
        // 任务执行失败
        printf("[wpm2_pick_place] 手爪张开失败,程序退出 \n");
        return 0;
    }

    printf("[3]向前伸出 \n");
    quat.setRPY(0.0, -1.57, 0.0);
    transform.setRotation(quat);
    target_pose.orientation.x= transform.getRotation().getX();
    target_pose.orientation.y = transform.getRotation().getY();
    target_pose.orientation.z = transform.getRotation().getZ();
    target_pose.orientation.w = transform.getRotation().getW();
    target_pose.position.x = fPick_x;
    target_pose.position.y = fPick_y;
    target_pose.position.z = fPick_z;
    group.setPoseTarget(target_pose);
    plan_success = group.plan(plan);
    if(plan_success == true)
    {
        printf("[wpm2_pick_place] 轨迹规划成功! 开始执行! \n");
        // 机械臂按照规划的轨迹运动
        group.execute(plan);
    }
    else
    {
        printf("[wpm2_pick_place] 轨迹规划失败,请检查规划目标的参数 \n");
        return 0;
    }

    printf("[4]闭合手爪 抓取物体\n");
    gripperCmd.command.position = fPick_gap;
    acGripper.sendGoal(gripperCmd);
    gripper_success = acGripper.waitForResult(ros::Duration(10.0));
    if (gripper_success == false)
    {
        // 任务执行失败
        printf("[wpm2_pick_place] 手爪闭合失败,程序退出 \n");
        return 0;
    }

    /********* 放置阶段 *********/
    printf("[5]拿着物品抬起 \n");
    quat.setRPY(0.0, -1.57, 0.0);
    transform.setRotation(quat);
    target_pose.orientation.x= transform.getRotation().getX();
    target_pose.orientation.y = transform.getRotation().getY();
    target_pose.orientation.z = transform.getRotation().getZ();
    target_pose.orientation.w = transform.getRotation().getW();
    target_pose.position.x = fPick_x + 0.2;
    target_pose.position.y = fPick_y;
    target_pose.position.z = fPick_z + 0.1;
    group.setPoseTarget(target_pose);

    plan_success = group.plan(plan);
    if(plan_success == true)
    {
        printf("[wpm2_pick_place] 轨迹规划成功! 开始执行! \n");
        // 机械臂按照规划的轨迹运动
        group.execute(plan);
    }
    else
    {
        printf("[wpm2_pick_place] 轨迹规划失败,请检查规划目标的参数 \n");
        return 0;
    }

    // 延时1秒,让关节反馈数据更新,否则下一次规划会出现初始位置错误
    sleep(1.0);

    printf("[6] 将物品转移到放置地点 \n");
    quat.setRPY(0.0, -1.57, -1.57);
    transform.setRotation(quat);
    target_pose.orientation.x= transform.getRotation().getX();
    target_pose.orientation.y = transform.getRotation().getY();
    target_pose.orientation.z = transform.getRotation().getZ();
    target_pose.orientation.w = transform.getRotation().getW();
    target_pose.position.x = fPlace_x;
    target_pose.position.y = fPlace_y;
    target_pose.position.z = fPlace_z;
    group.setPoseTarget(target_pose);

    plan_success = group.plan(plan);
    if(plan_success == true)
    {
        printf("[wpm2_pick_place] 轨迹规划成功! 开始执行! \n");
        // 机械臂按照规划的轨迹运动
        group.execute(plan);
    }
    else
    {
        printf("[wpm2_pick_place] 轨迹规划失败,请检查规划目标的参数 \n");
        return 0;
    }

    printf("[7]张开手爪 \n");
    gripperCmd.command.position = 0.09;
    acGripper.sendGoal(gripperCmd);
    gripper_success = acGripper.waitForResult(ros::Duration(10.0));
    if (gripper_success == false)
    {
        // 任务执行失败
        printf("[wpm2_pick_place] 手爪张开失败,程序退出 \n");
        return 0;
    }

    printf("[8] 回到默认姿态 \n");
    quat.setRPY(0.0, 0.0, 0.0);
    transform.setRotation(quat);
    target_pose.orientation.x= transform.getRotation().getX();
    target_pose.orientation.y = transform.getRotation().getY();
    target_pose.orientation.z = transform.getRotation().getZ();
    target_pose.orientation.w = transform.getRotation().getW();
    target_pose.position.x = 0;
    target_pose.position.y = 0;
    target_pose.position.z = 0.78;
    group.setPoseTarget(target_pose);

    plan_success = group.plan(plan);
    if(plan_success == true)
    {
        printf("[wpm2_pick_place] 轨迹规划成功! 开始执行! \n");
        // 机械臂按照规划的轨迹运动
        group.execute(plan);
    }
    else
    {
        printf("[wpm2_pick_place] 轨迹规划失败,请检查规划目标的参数 \n");
        return 0;
    }
  
    ros::shutdown(); 
    return 0;
}