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
// Created by qiayuan on 5/27/21.
//

#pragma once

#include "rm_common/decision/service_caller.h"
#include "rm_common/decision/controller_manager.h"

namespace rm_common
{
class CalibrationService
{
public:
  CalibrationService(XmlRpc::XmlRpcValue& rpc_value, ros::NodeHandle& nh)
  {
    // 利用rpc_value判断参数文件是否有start_controllers，stop_controllers，query_services这三个容器
    ROS_ASSERT(rpc_value.hasMember("start_controllers"));
    ROS_ASSERT(rpc_value.hasMember("stop_controllers"));
    ROS_ASSERT(rpc_value.hasMember("services_name"));
    // 利用rpc_value判断start_controllers，stop_controllers，query_services类型的type是否正确
    ROS_ASSERT(rpc_value["start_controllers"].getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(rpc_value["stop_controllers"].getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(rpc_value["services_name"].getType() == XmlRpc::XmlRpcValue::TypeArray);
    // 用定义在该文件的start_controllers，stop_controllers容器获取写入参数文件中的start_controllers，stop_controllers
    start_controllers = getControllersName(rpc_value["start_controllers"]);
    stop_controllers = getControllersName(rpc_value["stop_controllers"]);
    // 前面都是获取参数
    for (int i = 0; i < rpc_value["services_name"].size(); ++i)
    {
      // 用配置文件中获取的services_name构造QueryCalibrationServiceCaller，并放置到query_services中
      query_services.push_back(new QueryCalibrationServiceCaller(nh, rpc_value["services_name"][i]));
    }
  }
  // 设置is_calibrated为false
  void setCalibratedFalse()
  {
    for (auto& service : query_services)
      service->getService().response.is_calibrated = false;
  }
  // 用于获取是否校准成功
  bool isCalibrated()
  {
    bool is_calibrated = true;
    for (auto& service : query_services)
      // service->isCalibrated()这个接口会得到校准是否成功，如果query_services中所有的服务都返回true，那么说明所有校准都成功，此时才返回true
      is_calibrated &= service->isCalibrated();
    return is_calibrated;
  }
  // 呼叫服务
  void callService()
  {
    for (auto& service : query_services)
      service->callService();
  }
  // 创建三个容器，start_controllers，stop_controllers，query_services,<>内是数据类型
  std::vector<std::string> start_controllers, stop_controllers;
  std::vector<QueryCalibrationServiceCaller*> query_services;

private:
  // 用容器controllers获取rpc_value内的控制器，并返回controllers
  static std::vector<std::string> getControllersName(XmlRpc::XmlRpcValue& rpc_value)
  {
    std::vector<std::string> controllers;
    for (int i = 0; i < rpc_value.size(); ++i)
    {
      controllers.push_back(rpc_value[i]);
    }
    return controllers;
  }
};

class CalibrationQueue
{
public:
  explicit CalibrationQueue(XmlRpc::XmlRpcValue& rpc_value, ros::NodeHandle& nh, ControllerManager& controller_manager)
    : controller_manager_(controller_manager), switched_(false)
  {
    // Don't calibration if using simulation，如果是仿真就不校准
    ros::NodeHandle nh_global;
    bool use_sim_time;
    nh_global.param("use_sim_time", use_sim_time, false);
    if (use_sim_time || rpc_value.getType() != XmlRpc::XmlRpcValue::TypeArray)
      return;
    // 将每个校准控制器写入calibration_services_中（和push_back函数相似）
    for (int i = 0; i < rpc_value.size(); ++i)
      calibration_services_.emplace_back(rpc_value[i], nh);
    // 最后一个询问是ros实时时间
    last_query_ = ros::Time::now();
    calibration_itr_ = calibration_services_.end();
    // Start with calibrated, you should use reset() to start calibration.
  }
  // 每次调用reset，calibration_itr_会指向calibration_services_中的第一个
  void reset()
  {
    if (calibration_services_.empty())
      return;
    calibration_itr_ = calibration_services_.begin();
    switched_ = false;
    for (auto service : calibration_services_)
      service.setCalibratedFalse();
  }
  void update(const ros::Time& time, bool flip_controllers)
  {
    // 如果校准服务为控，则结束
    if (calibration_services_.empty())
      return;
    // 如果已完成校准，则结束
    if (isCalibrated())
      return;
    // 如果开关开了
    if (switched_)
    {
      // 如果校准迭代器已经校准
      if (calibration_itr_->isCalibrated())
      {
        if (flip_controllers)
          controller_manager_.startControllers(calibration_itr_->stop_controllers);
        controller_manager_.stopControllers(calibration_itr_->start_controllers);
        calibration_itr_++;
        switched_ = false;
      }
      else if ((time - last_query_).toSec() > .2)
      {
        last_query_ = time;
        calibration_itr_->callService();
      }
    }
    else
    {
      // Switch controllers
      switched_ = true;
      if (calibration_itr_ != calibration_services_.end())
      {
        controller_manager_.startControllers(calibration_itr_->start_controllers);
        controller_manager_.stopControllers(calibration_itr_->stop_controllers);
      }
    }
  }
  void update(const ros::Time& time)
  {
    update(time, true);
  }
  bool isCalibrated()
  {
    return calibration_itr_ == calibration_services_.end();
  }
  void stopController()
  {
    // 如果校准服务为空，则结束
    if (calibration_services_.empty())
      return;
    // 如果校准迭代不等于校准服务的最后一个 并且 开关是开着的
    if (calibration_itr_ != calibration_services_.end() && switched_)
      // 关闭校准迭代指向的控制控制器
      controller_manager_.stopControllers(calibration_itr_->stop_controllers);
  }
  void stop()
  {
    // 如歌开关开着
    if (switched_)
    {
      // 校准迭代等于校准服务最后一个校准控制器服务
      calibration_itr_ = calibration_services_.end();
      // 然后将开关关闭
      switched_ = false;
    }
  }

private:
  ros::Time last_query_;
  // 校准服务
  std::vector<CalibrationService> calibration_services_;
  // 校准迭代器
  std::vector<CalibrationService>::iterator calibration_itr_;
  // 控制器管理
  ControllerManager& controller_manager_;
  bool switched_;
};
}  // namespace rm_common
