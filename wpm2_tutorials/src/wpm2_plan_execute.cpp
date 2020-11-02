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
 
#include <tf/transform_broadcaster.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

static tf::StampedTransform transform;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wpm2_plan_execute");

    ros::NodeHandle node_handle; 
    ros::AsyncSpinner spinner(1);
    spinner.start();

    printf("[wpm2_planning] 延迟5秒,等待Rviz启动... \n");
    sleep(5.0);

    printf("[wpm2_planning] 准备进行轨迹规划... \n");
    moveit::planning_interface::MoveGroupInterface group("arm");

    // 设置机械臂末端的目标位置
    geometry_msgs::Pose target_pose;

   
    // 机械臂末端朝向(这里设置为竖直向上,以免运动时误碰周围物体)
    tf::Quaternion quat;
    // 手爪初始姿态是竖直向上,函数三个参数分别为滚转,俯仰和偏转角,单位为弧度
    quat.setRPY(0.0, 0.0, 0.0);
    // 将欧拉角旋转量转换成四元数表达
    transform.setRotation(quat);
    target_pose.orientation.x= transform.getRotation().getX();
    target_pose.orientation.y = transform.getRotation().getY();
    target_pose.orientation.z = transform.getRotation().getZ();
    target_pose.orientation.w = transform.getRotation().getW();

    // 机械臂末端的空间坐标,单位为米
    target_pose.position.x = -0.1;
    target_pose.position.y = 0.0;
    target_pose.position.z = 0.75;
    group.setPoseTarget(target_pose);

    // 进行运动规划，只是计算出轨迹，还不会控制机械臂实际运动
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);
    if(success == moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
        printf("[wpm2_planning] 轨迹规划成功! 开始执行! \n");

        // 机械臂按照规划的轨迹运动
        group.execute(my_plan);
    }
    else
    {
        printf("[wpm2_planning] 轨迹规划失败,请检查规划目标的参数 \n");
    }
  
    ros::shutdown(); 
    return 0;
}