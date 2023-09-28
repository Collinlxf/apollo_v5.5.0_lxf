/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/planning/planning_base.h"

#include "cyber/time/clock.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"   //规划的配置标志（planning_gflags）
#include "modules/planning/proto/planning_internal.pb.h" // 规划的内部数据结构（planning_internal.pb.h）
#include "modules/planning/tasks/task_factory.h" // 任务工厂（task_factory）。

namespace apollo {
namespace planning {

using apollo::common::Status;

/*
1.构造函数PlanningBase初始化成员变量 injector_，这个成员变量是一个智能指针，用于管理依赖注入。
*/
PlanningBase::PlanningBase(const std::shared_ptr<DependencyInjector>& injector)
    : injector_(injector) {}

PlanningBase::~PlanningBase() {}

Status PlanningBase::Init(const PlanningConfig& config) {
  injector_->planning_context()->Init(); //这表示初始化规划上下文（planning_context）。
  TaskFactory::Init(config, injector_);  //初始化任务工厂。
  return Status::OK(); //返回 Status::OK()，表示初始化成功。
}


/*
1. 函数主要目的是填充trajectory_pb 对象的头部信息，包括时间戳和路由（routing）的头部信息。
2. ADCTrajectory在planning.proto中被定义
3. FillPlanningPb 函数中将不同模块的时间戳赋值到 trajectory_pb 的作用是协调不同模块的时间戳，并将这些时间戳记录在生成的规划轨迹数据中，以便在后续处理中可以根据需要使用这些时间戳信息。
4. 在自动驾驶系统中，这些传感器通常以不同的频率生成数据，并且数据的时间戳也可能不完全一致。
5. 将这些时间戳赋值给 trajectory_pb 的目的是为了将规划轨迹的时间信息与传感器数据的时间信息关联起来。
6. 这有助于在后续的数据处理中，例如感知模块和决策模块，将规划轨迹与感知数据进行时间上的匹配和协调，以确保系统操作的一致性和准确性。
*/
void PlanningBase::FillPlanningPb(const double timestamp,
                                  ADCTrajectory* const trajectory_pb) {
  trajectory_pb->mutable_header()->set_timestamp_sec(timestamp);
  if (local_view_.prediction_obstacles->has_header()) {
    trajectory_pb->mutable_header()->set_lidar_timestamp(
        local_view_.prediction_obstacles->header().lidar_timestamp());
    trajectory_pb->mutable_header()->set_camera_timestamp(
        local_view_.prediction_obstacles->header().camera_timestamp());
    trajectory_pb->mutable_header()->set_radar_timestamp(
        local_view_.prediction_obstacles->header().radar_timestamp());
  }
  trajectory_pb->mutable_routing_header()->CopyFrom(
      local_view_.routing->header());
}
}  // namespace planning
}  // namespace apollo
