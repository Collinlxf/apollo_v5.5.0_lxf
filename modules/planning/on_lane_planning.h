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

#pragma once

#include <memory>
#include <string>
#include <vector>

//该头文件包含了平滑器（Smoother）的定义和实现。平滑器用于规划模块中的轨迹平滑，它可以使车辆的轨迹更加平滑，以提高驾驶舒适性和安全性。
#include "modules/planning/common/smoothers/smoother.h"
//该头文件包含了 OnLanePlannerDispatcher 的定义和实现。OnLanePlannerDispatcher 是规划模块中的一个调度器，负责调度不同的车道规划器（Lane Planner）。它根据不同的交通场景选择合适的车道规划器来生成车辆的路径。
#include "modules/planning/planner/on_lane_planner_dispatcher.h"
#include "modules/planning/planning_base.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

/**
 * @class planning
 *
 * @brief Planning module main class. It processes GPS and IMU as input,
 * to generate planning info.
 */
class OnLanePlanning : public PlanningBase {
 public:
 /*OnLanePlanning 类继承自 PlanningBase，这是规划模块的基类。
 它通过调用 PlanningBase 的构造函数来初始化，并在构造函数中创建了 OnLanePlannerDispatcher 的实例。
 1. std::unique_ptr是C++11中引入的
 2. std::make_unique是C++14中引入的
 3. std::unique_ptr和std::make_unique作用相似，都可用于管理动态分配的对象，以便自动释放内存资源，从而帮助防止内存泄漏。
 4. 区别：std::unique_ptr 是一种智能指针类型，用于管理动态分配的对象的所有权，而 std::make_unique 是一个辅助函数，用于创建 std::unique_ptr 指向动态分配的对象的实例。
 5. planner_dispatcher_没有在类OnLanePlanning中被显示的申明为成员变量，但是在构造函数中进行初始化，这是一种常见的做法，可以使初始化过程更加清晰和可读。
 6. planner_dispatcher_ 被初始化为 std::make_unique<OnLanePlannerDispatcher>()，这表示创建了一个新的 OnLanePlannerDispatcher 对象，并将其存储在 planner_dispatcher_ 中。
 7. planner_dispatcher_也是类OnLanePlanning的成员变量
 */
  explicit OnLanePlanning(const std::shared_ptr<DependencyInjector>& injector)
      : PlanningBase(injector) {
    planner_dispatcher_ = std::make_unique<OnLanePlannerDispatcher>();
  }
  virtual ~OnLanePlanning();

  /**
   * @brief Planning name.
   */
  std::string Name() const override;

  /**
   * @brief module initialization function
   * @return initialization status
   * 在OnLanePlanning的父类PlanningBase中virtual apollo::common::Status Init(const PlanningConfig& config);被定义为了一个虚函数
   */
  common::Status Init(const PlanningConfig& config) override;

  /**
   * @brief main logic of the planning module, runs periodically triggered by
   * timer.
   */
  void RunOnce(const LocalView& local_view,
               ADCTrajectory* const ptr_trajectory_pb) override;

  common::Status Plan(
      const double current_time_stamp,
      const std::vector<common::TrajectoryPoint>& stitching_trajectory,
      ADCTrajectory* const trajectory) override;

 private:
  common::Status InitFrame(const uint32_t sequence_num,
                           const common::TrajectoryPoint& planning_start_point,
                           const common::VehicleState& vehicle_state);

  common::VehicleState AlignTimeStamp(const common::VehicleState& vehicle_state,
                                      const double curr_timestamp) const;

  void ExportReferenceLineDebug(planning_internal::Debug* debug);
  bool CheckPlanningConfig(const PlanningConfig& config);
  void GenerateStopTrajectory(ADCTrajectory* ptr_trajectory_pb);
  void ExportFailedLaneChangeSTChart(const planning_internal::Debug& debug_info,
                                     planning_internal::Debug* debug_chart);
  void ExportOnLaneChart(const planning_internal::Debug& debug_info,
                         planning_internal::Debug* debug_chart);
  void ExportOpenSpaceChart(const planning_internal::Debug& debug_info,
                            const ADCTrajectory& trajectory_pb,
                            planning_internal::Debug* debug_chart);
  void AddOpenSpaceOptimizerResult(const planning_internal::Debug& debug_info,
                                   planning_internal::Debug* debug_chart);
  void AddPartitionedTrajectory(const planning_internal::Debug& debug_info,
                                planning_internal::Debug* debug_chart);

  void AddStitchSpeedProfile(planning_internal::Debug* debug_chart);

  void AddPublishedSpeed(const ADCTrajectory& trajectory_pb,
                         planning_internal::Debug* debug_chart);

  void AddPublishedAcceleration(const ADCTrajectory& trajectory_pb,
                                planning_internal::Debug* debug);

  void AddFallbackTrajectory(const planning_internal::Debug& debug_info,
                             planning_internal::Debug* debug_chart);

 private:
  routing::RoutingResponse last_routing_;
  std::unique_ptr<ReferenceLineProvider> reference_line_provider_;
  Smoother planning_smoother_;
};

}  // namespace planning
}  // namespace apollo
