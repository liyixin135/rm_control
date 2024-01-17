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
// Created by bruce on 2021/7/28.
//

#pragma once

#include <ros/ros.h>
#include <rm_msgs/ChassisCmd.h>
#include <rm_msgs/GameStatus.h>
#include <rm_msgs/GameRobotStatus.h>
#include <rm_msgs/PowerHeatData.h>
#include <rm_msgs/PowerManagementSampleAndStatusData.h>

namespace rm_common
{
class PowerLimit
{
public:
  PowerLimit(ros::NodeHandle& nh)

  {
    // 拿数据，数据全在config下的manual里面
    if (!nh.getParam("safety_power", safety_power_))
      ROS_ERROR("Safety power no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("capacitor_threshold", capacitor_threshold_))
      ROS_ERROR("Capacitor threshold no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("charge_power", charge_power_))
      ROS_ERROR("Charge power no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("extra_power", extra_power_))
      ROS_ERROR("Extra power no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("burst_power", burst_power_))
      ROS_ERROR("Burst power no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("power_gain", power_gain_))
      ROS_ERROR("power gain no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("buffer_threshold", buffer_threshold_))
      ROS_ERROR("buffer threshold no defined (namespace: %s)", nh.getNamespace().c_str());
  }
  // 定义power_limit的mode
  typedef enum
  {
    CHARGE = 0,
    BURST = 1,
    NORMAL = 2,
    ALLOFF = 3,
    TEST = 4,
  } Mode;

  // 传入safety_power，更新safety power_
  void updateSafetyPower(int safety_power)
  {
    if (safety_power > 0)
      safety_power_ = safety_power;
    ROS_INFO("update safety power: %d", safety_power);
  }
  // 更新state
  void updateState(uint8_t state)
  {
    expect_state_ = state;
  }
  // 从裁判系统扒id和底盘功率上限
  void setGameRobotData(const rm_msgs::GameRobotStatus data)
  {
    robot_id_ = data.robot_id;
    chassis_power_limit_ = data.chassis_power_limit;
  }
  // 从裁判系统扒底盘功率缓冲区
  void setChassisPowerBuffer(const rm_msgs::PowerHeatData data)
  {
    chassis_power_buffer_ = data.chassis_power_buffer;
  }
  void setCapacityData(const rm_msgs::PowerManagementSampleAndStatusData data)
  {
    // 超电是否在线取决于能不能读到裁判系统的时间
    capacity_is_online_ = ros::Time::now() - data.stamp < ros::Duration(0.3);
    // 全在裁判系统扒
    cap_energy_ = data.capacity_remain_charge;
    cap_state_ = data.state_machine_running_state;
  }
  // 更新referee是否online
  void setRefereeStatus(bool status)
  {
    referee_is_online_ = status;
  }

  // 不传参数，直接return state
  uint8_t getState()
  {
    return expect_state_;
  }
  void setLimitPower(rm_msgs::ChassisCmd& chassis_cmd, bool is_gyro)
  {
    // 排除掉工程
    if (robot_id_ == rm_msgs::GameRobotStatus::BLUE_ENGINEER || robot_id_ == rm_msgs::GameRobotStatus::RED_ENGINEER)
      chassis_cmd.power_limit = 400;
    else
    {
      // standard and hero，我要看的领域
      if (referee_is_online_)  // 裁判系统在线
      {
        if (capacity_is_online_)  // 超电在线，其实也是用裁判系统读出来的超电时间戳作为标志位
        {
          // 如果裁判系统功率上限大于burst模式下提供的功率，那么底盘命令的功率限制就等于burst（chassis_power_limit_这个是裁判系统读出来的
          if (chassis_power_limit_ > burst_power_)
            chassis_cmd.power_limit = burst_power_;
          else
          {
            // 如果裁判系统和超电都在，但是裁判系统读出来的功率限制不足以让底盘进入burst，选择模式进入
            switch (cap_state_)  // cap_state_是从裁判系统扒出来的
            {
              case NORMAL:
                normal(chassis_cmd);
                break;
              case BURST:
                burst(chassis_cmd, is_gyro);
                break;
              case CHARGE:
                charge(chassis_cmd);
                break;
              default:
                zero(chassis_cmd);
                break;
            }
          }
        }
        else  // 裁判系统在线，但是超电不在，那用normal
          normal(chassis_cmd);
      }
      // 如果裁判系统不在，直接命令定死，给底盘功率限制是60,这个是步兵最低等级的功率，绝对安全功率
      else
        chassis_cmd.power_limit = safety_power_;
    }
  }

private:
  // 以下都是裁判系统读出来的功率限制小于burst后选择进入的模式
  // charge
  void charge(rm_msgs::ChassisCmd& chassis_cmd)
  {
    chassis_cmd.power_limit = chassis_power_limit_ * 0.70;
  }
  //
  void normal(rm_msgs::ChassisCmd& chassis_cmd)
  {
    double buffer_energy_error = chassis_power_buffer_ - buffer_threshold_;
    double plus_power = buffer_energy_error * power_gain_;
    chassis_cmd.power_limit = chassis_power_limit_ + plus_power;
    // TODO:Add protection when buffer<5
  }
  // zero模式下底盘功率限制直接给0,一点都没有
  void zero(rm_msgs::ChassisCmd& chassis_cmd)
  {
    chassis_cmd.power_limit = 0.0;
  }
  void burst(rm_msgs::ChassisCmd& chassis_cmd, bool is_gyro)
  {
    // 如果cap_energy_大于capacitor_threshold_（设置了一个最低限制，cap_energy_是从裁判系统读出来的超电能量
    if (cap_energy_ > capacitor_threshold_)
    {
      if (is_gyro)  // 小陀螺时直接加上extra_power_50,直接
        chassis_cmd.power_limit = chassis_power_limit_ + extra_power_;
      else
        chassis_cmd.power_limit = burst_power_;
    }
    else
      expect_state_ = NORMAL;
  }

  int chassis_power_buffer_;
  int robot_id_, chassis_power_limit_;
  float cap_energy_;
  double safety_power_{};
  double capacitor_threshold_{};
  double charge_power_{}, extra_power_{}, burst_power_{};
  double buffer_threshold_{};
  double power_gain_{};
  uint8_t expect_state_{}, cap_state_{};

  bool referee_is_online_;
  bool capacity_is_online_;
};
}  // namespace rm_common
