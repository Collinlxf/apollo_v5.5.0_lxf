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

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "cyber/message/raw_message.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/perception/proto/traffic_light_detection.pb.h"
#include "modules/planning/common/message_process.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/planning_base.h"
#include "modules/planning/proto/learning_data.pb.h"
#include "modules/planning/proto/pad_msg.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"
#include "modules/routing/proto/routing.pb.h"
#include "modules/storytelling/proto/story.pb.h"

namespace apollo {
namespace planning {

/*
1. PlanningComponent是通用的模板类（cyber::Component）的实例，其中final表示PlanningComponet类不能在被继承
2. prediction::PredictionObstacles、canbus::Chassis 和 localization::LocalizationEstimate 是称为泛型参数，因为它们用于定义一个通用的模板类（cyber::Component）的实例。
3. Init()：用于组件初始化的函数，覆盖了基类的虚函数。
4. Proc(...)：用于处理输入数据并执行规划的函数，同样覆盖了基类的虚函数。
5. CheckRerouting() 和 CheckInput()：用于辅助功能的私有函数。
*/
class PlanningComponent final
    : public cyber::Component<prediction::PredictionObstacles, canbus::Chassis,
                              localization::LocalizationEstimate> {
 public:
  PlanningComponent() = default;

  ~PlanningComponent() = default;

 public:
  /*override对基类中虚函数的重写*/
  bool Init() override;

  bool Proc(const std::shared_ptr<prediction::PredictionObstacles>&
                prediction_obstacles,
            const std::shared_ptr<canbus::Chassis>& chassis,
            const std::shared_ptr<localization::LocalizationEstimate>&
                localization_estimate) override;

 private:
  void CheckRerouting();
  bool CheckInput();

 private:
 /*
 1. std::shared_ptr 用于共享通信通道和全局配置/依赖项。
 2. std::shared_ptr 表示多个指针可以共享对同一资源的所有权，资源会在最后一个 std::shared_ptr 被销毁时释放。
 3. std::shared_ptr 主要用于共享读取器（cyber::Reader）和写入器（cyber::Writer），这些用于与其他模块或组件进行通信，例如接收来自感知模块的交通灯信息或向其他模块发送规划轨迹。
 4. 使用 std::shared_ptr 以确保多个模块可以安全地共享这些通信通道。 
 */
  std::shared_ptr<cyber::Reader<perception::TrafficLightDetection>>
      traffic_light_reader_;
  std::shared_ptr<cyber::Reader<routing::RoutingResponse>> routing_reader_;
  std::shared_ptr<cyber::Reader<planning::PadMessage>> pad_msg_reader_;
  std::shared_ptr<cyber::Reader<relative_map::MapMsg>> relative_map_reader_;
  std::shared_ptr<cyber::Reader<storytelling::Stories>> story_telling_reader_;

  std::shared_ptr<cyber::Writer<ADCTrajectory>> planning_writer_;
  std::shared_ptr<cyber::Writer<routing::RoutingRequest>> rerouting_writer_;
  std::shared_ptr<cyber::Writer<PlanningLearningData>>
      planning_learning_data_writer_;
  
  /*
  std::unique_ptr 用于独占规划模块的核心组件。
  */
  std::mutex mutex_;
  perception::TrafficLightDetection traffic_light_;
  routing::RoutingResponse routing_;
  planning::PadMessage pad_msg_;
  relative_map::MapMsg relative_map_;
  storytelling::Stories stories_;

  LocalView local_view_;
  
  /*
  1. std::unique_ptr 用于独占规划模块的核心组件。
  2. std::unique_ptr 的指针被销毁时，它会自动释放资源。
  3. std::unique_ptr 主要用于 planning_base_，这是规划模块的一个核心组件，它负责实际的规划工作。因为规划模块是独立的，它需要拥有唯一的 planning_base_ 对象，因此使用了 std::unique_ptr。
  */
  std::unique_ptr<PlanningBase> planning_base_;
  std::shared_ptr<DependencyInjector> injector_;

  PlanningConfig config_;
  MessageProcess message_process_;
};

CYBER_REGISTER_COMPONENT(PlanningComponent)

}  // namespace planning
}  // namespace apollo
