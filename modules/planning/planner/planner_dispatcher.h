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

#include "modules/common/status/status.h"
#include "modules/common/util/factory.h"
#include "modules/planning/planner/planner.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

/**
 * @class planning
 *
 * @brief PlannerDispatcher module main class.
 */
class PlannerDispatcher {
 public:
  PlannerDispatcher() = default;
  virtual ~PlannerDispatcher() = default;

  virtual common::Status Init() {
    RegisterPlanners();
    return common::Status::OK();
  }

  virtual std::unique_ptr<Planner> DispatchPlanner(
      const PlanningConfig& planning_config,
      const std::shared_ptr<DependencyInjector>& injector) = 0;

 protected:
  void RegisterPlanners();

  /*
  1. common::util::Factory 是一个工厂类模板，它用于创建不同类型对象的工厂。
  2. <PlannerType, Planner, Planner* (*)(const std::shared_ptr<DependencyInjector>& injector)> 是对模板 common::util::Factory 进行实例化时的模板参数。
    2.1 PlannerType：表示工厂将要创建的对象的类型。在这里，它表示规划器的类型，比如 RTK、PublicRoad、Lattice、Navi 等
    2.2 Planner：表示工厂将要创建的对象的基类类型，通常是一个抽象类，规划器基类。
    2.3 Planner* (*)(const std::shared_ptr<DependencyInjector>& injector)：表示工厂将要创建对象的函数指针类型，它是一个函数指针，指向一个函数，该函数接受一个 std::shared_ptr<DependencyInjector> 参数并返回一个 Planner* 指针，即创建的规划器对象的指针。
    2.4 planner_factory_ 是工厂的实例，它将根据不同的 PlannerType 调用不同的函数来创建相应的规划器对象。工厂类内部会维护一个映射，将 PlannerType 与对应的创建函数指针关联起来。
    */
  common::util::Factory<
      PlannerType, Planner,
      Planner* (*)(const std::shared_ptr<DependencyInjector>& injector)>
      planner_factory_;
};

}  // namespace planning
}  // namespace apollo
