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

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/common/status/status.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/dreamview/proto/chart.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/map/hdmap/hdmap.h"
#include "modules/perception/proto/traffic_light_detection.pb.h"
#include "modules/planning/common/dependency_injector.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/local_view.h"
#include "modules/planning/common/trajectory/publishable_trajectory.h"
#include "modules/planning/planner/planner.h"
#include "modules/planning/planner/planner_dispatcher.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/proto/traffic_rule_config.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"
#include "modules/routing/proto/routing.pb.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 * 这段代码定义了规划模块（Planning）的核心类 PlanningBase，它是规划模块的基类，负责规划自动驾驶车辆的轨迹。
 */
namespace apollo {
namespace planning {
/**
 * @class planning
 *
 * @brief PlanningBase module main class.
 */
class PlanningBase {
 public:
 /*
 1. 构造函数被删除（= delete）（= delete，这是 C++11 引入的语法，告诉编译器不要生成默认构造函数。），意味着不能创建 PlanningBase 的实例，只能通过其派生类来创建。析构函数用于释放资源。
 2. 禁止直接实例化 PlanningBase的原因是 PlanningBase 是一个抽象基类（Abstract Base Class），其中包含了纯虚函数（virtual 函数且没有实现），并且它的主要目的是为其派生类提供一个规范，定义规划模块的接口和基本行为。
 3. explicit 关键字：这表示构造函数是显式的，即不能通过隐式类型转换来调用。前面虽然禁止了默认构造函数，但是但仍然可以在 PlanningBase 的派生类中定义构造函数
 */
  PlanningBase() = delete;

  explicit PlanningBase(const std::shared_ptr<DependencyInjector>& injector);
  /*
  1. 在C++中，将析构函数声明为虚函数的主要好处是支持多态和资源管理。
  2. 在C++中，通常情况下，如果一个类有虚函数，那么它的析构函数也应该是虚函数，以确保在使用多态时正确地销毁对象。这有助于避免内存泄漏和资源泄漏等问题。
  */
  virtual ~PlanningBase();

  virtual apollo::common::Status Init(const PlanningConfig& config);

  /*
  1. 在C++中，当一个成员函数在类的声明中被声明为纯虚函数（通过= 0标记），就意味着这个函数必须在派生类中进行实现。这种情况通常出现在基类（如 PlanningBase）希望强制其派生类（如 NaviPlanning 或 OnLanePlanning）来实现特定的行为，但基类本身不能提供合适的默认实现。
  2. 因此Name()和RunOnce()，在planning_base.cc中并没有被实现
  */
  virtual std::string Name() const = 0;
  /*
  1.  virtual void RunOnce 函数，它在 PlanningBase 中被声明为纯虚函数，因此没有提供默认实现。这是因为不同的规划策略（导航规划、车道规划等）可能需要不同的实现逻辑，而这个逻辑是由派生类来提供的。
  */
  virtual void RunOnce(const LocalView& local_view,
                       ADCTrajectory* const adc_trajectory) = 0;

  /**
   * @brief Plan the trajectory given current vehicle state
   * Plan 函数的作用，它是规划模块中的一个核心函数，用于根据当前车辆状态和前方轨迹点来进行路径规划和轨迹生成。具体的功能包括但不限于：
   * 根据当前时间戳 current_time_stamp 来确定规划的时间点。
   * 基于当前车辆状态、前方轨迹点以及规划模块的配置，生成车辆的行驶轨迹。
   * 将生成的轨迹存储在传入的 trajectory 指针所指向的对象中。
   */
  virtual apollo::common::Status Plan(
      const double current_time_stamp,
      const std::vector<common::TrajectoryPoint>& stitching_trajectory,
      ADCTrajectory* const trajectory) = 0;

 protected:
  /*
  1. FillPlanningPb 函数在 Apollo 中的作用是用于填充规划生成的轨迹信息到 ADCTrajectory 对象中。具体来说，这个函数的目的是将规划模块生成的车辆轨迹数据填充到指定的数据结构中，以便后续的处理和发布。
  */
  virtual void FillPlanningPb(const double timestamp,
                              ADCTrajectory* const trajectory_pb);

  LocalView local_view_;
  /*
  1.这个指针的作用是在规划模块中引用高精度地图的实例。通过将其初始化为 nullptr，规划模块可以在初始化时不需要立即关联到特定的高精度地图。相反，规划模块可以在需要时动态地设置该指针，以指向正确的高精度地图实例。
  2.延迟加载: 高精度地图可能很大，加载需要时间。因此，规划模块可以选择在需要时才加载地图数据，而不是在初始化时加载。
  3.动态切换地图: 在运行时，可能需要切换到不同的地图区域，这个指针可以用于在切换时更新到新的地图实例。
  4.避免硬编码依赖: Apollo 可能支持多种高精度地图供应商或不同的地图版本。通过使用指针并根据需要进行初始化，可以使规划模块更具灵活性，以便在不同的配置或场景下使用不同的地图。
  */
  const hdmap::HDMap* hdmap_ = nullptr;

  double start_time_ = 0.0;
  size_t seq_num_ = 0;

  PlanningConfig config_;
  TrafficRuleConfigs traffic_rule_configs_;
  std::unique_ptr<Frame> frame_;
  std::unique_ptr<Planner> planner_;
  std::unique_ptr<PublishableTrajectory> last_publishable_trajectory_;
  std::unique_ptr<PlannerDispatcher> planner_dispatcher_;
  std::shared_ptr<DependencyInjector> injector_;
};

}  // namespace planning
}  // namespace apollo
