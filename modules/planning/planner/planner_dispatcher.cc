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

#include "modules/planning/planner/planner_dispatcher.h"

#include <memory>

#include "modules/planning/planner/lattice/lattice_planner.h"
#include "modules/planning/planner/navi/navi_planner.h"
#include "modules/planning/planner/public_road/public_road_planner.h"
#include "modules/planning/planner/rtk/rtk_replay_planner.h"
#include "modules/planning/proto/planning_config.pb.h"

namespace apollo {
namespace planning {

/*
1. PlannerType 在planning.proto中被声明
2. void PlannerDispatcher::RegisterPlanners() 是一个注册函数，它在调度器初始化时被调用。它注册了不同类型的规划器，并将每种类型的规划器与一个 lambda 函数关联起来，这个 lambda 函数用于创建对应类型的规划器对象。
3. PlannerType::RTK 对应的是 RTKReplayPlanner。
4. PlannerType::PUBLIC_ROAD 对应的是 PublicRoadPlanner。
5. PlannerType::LATTICE 和 PlannerType::NAVI 就是同样的。
6. Lambda函数的一般语法如下：
  [ captures ] ( parameters ) -> return_type {
    // 函数体
}
  captures：捕获列表，用于指定在函数体内使用的外部变量。可以是空的，也可以包含一个或多个外部变量，用逗号分隔。捕获可以按值捕获（复制外部变量的值）或按引用捕获（引用外部变量）。
  parameters：函数参数列表，类似于普通函数的参数列表。
  return_type：返回值的类型，可以省略，编译器通常能够自动推断出来。
  函数体：Lambda函数的实际执行代码。
7. 
*/
void PlannerDispatcher::RegisterPlanners() {
  planner_factory_.Register(
      PlannerType::RTK,
      [](const std::shared_ptr<DependencyInjector>& injector) -> Planner* {
        return new RTKReplayPlanner(injector);
      });
  planner_factory_.Register(
      PlannerType::PUBLIC_ROAD,
      [](const std::shared_ptr<DependencyInjector>& injector) -> Planner* {
        return new PublicRoadPlanner(injector);
      });
  planner_factory_.Register(
      PlannerType::LATTICE,
      [](const std::shared_ptr<DependencyInjector>& injector) -> Planner* {
        return new LatticePlanner(injector);
      });
  planner_factory_.Register(
      PlannerType::NAVI,
      [](const std::shared_ptr<DependencyInjector>& injector) -> Planner* {
        return new NaviPlanner(injector);
      });
}

}  // namespace planning
}  // namespace apollo
