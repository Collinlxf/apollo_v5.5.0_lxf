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
#include "modules/planning/planning_component.h"

#include "cyber/common/file.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/util/message_util.h"
#include "modules/common/util/util.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/map/pnc_map/pnc_map.h"
#include "modules/planning/common/history.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/navi_planning.h"
#include "modules/planning/on_lane_planning.h"

namespace apollo {
namespace planning {

using apollo::cyber::ComponentBase;
using apollo::hdmap::HDMapUtil;
using apollo::perception::TrafficLightDetection;
using apollo::relative_map::MapMsg;
using apollo::routing::RoutingRequest;
using apollo::routing::RoutingResponse;
using apollo::storytelling::Stories;

/*
1. bool PlanningComponent::Init()该函数在规划模块启动时被调用，用于初始化模块。
2. FLAGS_use_navigation_mode 是一个命令行参数，通常是通过 Google 的开源库 GFlags（或者其他类似的库）来定义和解析的。
3. 具体而言，FLAGS_use_navigation_mode 可能是在 Apollo 的模块启动脚本或启动命令中定义的，然后在模块的主函数中使用。
*/
bool PlanningComponent::Init() {
  injector_ = std::make_shared<DependencyInjector>();

  if (FLAGS_use_navigation_mode) {
    planning_base_ = std::make_unique<NaviPlanning>(injector_);
  } else {
    planning_base_ = std::make_unique<OnLanePlanning>(injector_);
  }
  /*
  1.ACHECK 是 Apollo 中的一个宏，用于进行断言检查。如果括号中的条件为 false，它将触发一个错误并终止程序执行。
  2.ComponentBase::GetProtoConfig(&config_):这个部分是一个调用，用于从配置文件中加载规划模块的配置参数，并将这些参数存储在名为 config_ 的配置对象中。这是通过 &config_ 来传递配置对象的指针，以便在函数内部填充配置数据。
  */
  ACHECK(ComponentBase::GetProtoConfig(&config_))
      << "failed to load planning config file "
      << ComponentBase::ConfigFilePath();

  if (FLAGS_planning_offline_learning ||
      config_.learning_mode() != PlanningConfig::NO_LEARNING) {
    if (!message_process_.Init(config_, injector_)) {
      AERROR << "failed to init MessageProcess";
      return false;
    }
  }

  /*
  1.planning_base_ 是规划模块中的一个成员变量，它是一个指向规划基类的智能指针（std::unique_ptr）。根据代码中的初始化过程，planning_base_ 在初始化时被赋予了具体的规划基类的实例。这个基类是根据配置参数 FLAGS_use_navigation_mode 的值，可以是 NaviPlanning 或 OnLanePlanning 的一个实例。
  2.planning_base_在被初始化的时候已经被赋予了具体的实例。例如lattice算法就被赋予了OnLanePlanning
  3.planning_base_是一个指针，所以会用->。
  4.当您有一个指向类对象的指针时，您应该使用箭头操作符来访问该对象的成员。planning_base_ 是一个指向类对象的智能指针（std::unique_ptr），因此需要使用箭头操作符 -> 来调用该对象的成员函数 Init(config_)。
  */
  planning_base_->Init(config_);

  /*
  1. 这段代码创建了一个名为 routing_reader_ 的消息订阅器（Reader），用于订阅路由信息 (RoutingResponse) 的数据流。
  2. routing_reader_ 是一个类成员变量，用于存储创建的消息订阅器。
  3. node_->CreateReader<RoutingResponse>(...) 是 Apollo 框架提供的函数，用于创建一个消息订阅器，用于接收指定类型的消息，这里是 RoutingResponse 类型的消息。
  4. [this]：这是Lambda的捕获列表，它告诉编译器在Lambda函数体内可以访问哪些外部变量。
  5. std::lock_guard<std::mutex> lock(mutex_);：这一行创建了一个 std::lock_guard 对象，用于锁定名为 mutex_ 的互斥锁。互斥锁的目的是确保在多线程环境中对共享资源（在这里是 routing_ 对象）的访问是安全的。通过锁定互斥锁，Lambda函数确保在处理路由消息时没有其他线程能够同时修改 routing_ 对象。
  6. 锁mutex_用于保证规划模块 (Planning) 内部的多线程对共享资源的访问是安全的。具体来说，它用于确保在规划模块内部的不同函数中对 routing_ 对象的访问是线程安全的。
  7. 在规划模块 (Planning) 内部，有多个函数涉及到对 routing_ 对象的读取或写入操作。由于这些函数可以在不同的线程中执行，如果没有互斥锁的保护，可能会导致多个线程同时访问 routing_ 对象，从而引发竞争条件和数据不一致的问题。
  */
  routing_reader_ = node_->CreateReader<RoutingResponse>(
      config_.topic_config().routing_response_topic(),
      [this](const std::shared_ptr<RoutingResponse>& routing) {
        AINFO << "Received routing data: run routing callback."
              << routing->header().DebugString();
        std::lock_guard<std::mutex> lock(mutex_);
        routing_.CopyFrom(*routing);
      });

  traffic_light_reader_ = node_->CreateReader<TrafficLightDetection>(
      config_.topic_config().traffic_light_detection_topic(),
      [this](const std::shared_ptr<TrafficLightDetection>& traffic_light) {
        ADEBUG << "Received traffic light data: run traffic light callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        traffic_light_.CopyFrom(*traffic_light);
      });

  pad_msg_reader_ = node_->CreateReader<PadMessage>(
      config_.topic_config().planning_pad_topic(),
      [this](const std::shared_ptr<PadMessage>& pad_msg) {
        ADEBUG << "Received pad data: run pad callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        pad_msg_.CopyFrom(*pad_msg);
      });

  story_telling_reader_ = node_->CreateReader<Stories>(
      config_.topic_config().story_telling_topic(),
      [this](const std::shared_ptr<Stories>& stories) {
        ADEBUG << "Received story_telling data: run story_telling callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        stories_.CopyFrom(*stories);
      });

  if (FLAGS_use_navigation_mode) {
    relative_map_reader_ = node_->CreateReader<MapMsg>(
        config_.topic_config().relative_map_topic(),
        [this](const std::shared_ptr<MapMsg>& map_message) {
          ADEBUG << "Received relative map data: run relative map callback.";
          std::lock_guard<std::mutex> lock(mutex_);
          relative_map_.CopyFrom(*map_message);
        });
  }

  /*
  1. 这一行代码创建了一个消息写入器 planning_writer_，用于发布自动驾驶车辆的轨迹数据。轨迹数据的类型是 ADCTrajectory，这是 Apollo 中定义的消息类型。
  2. node_ 是一个 Cyber RT 的节点对象，用于与其他节点进行通信。
  3. config_.topic_config().planning_trajectory_topic() 返回了配置文件中指定的规划轨迹数据的通信主题名称。这个通信主题用于发布规划模块生成的车辆轨迹数据。
  */
  planning_writer_ = node_->CreateWriter<ADCTrajectory>(
      config_.topic_config().planning_trajectory_topic());

  /*
  1. 这一行代码创建了一个消息写入器 rerouting_writer_，用于发布重新路由请求数据。重新路由请求的数据类型是 RoutingRequest，这也是 Apollo 中定义的消息类型。重新路由请求通常是由规划模块发起的，用于请求新的路由信息，以应对某些情况下的路径变更或重规划需求。
  */
  rerouting_writer_ = node_->CreateWriter<RoutingRequest>(
      config_.topic_config().routing_request_topic());

  planning_learning_data_writer_ = node_->CreateWriter<PlanningLearningData>(
      config_.topic_config().planning_learning_data_topic());

  return true;
}

bool PlanningComponent::Proc(
    const std::shared_ptr<prediction::PredictionObstacles>&
        prediction_obstacles,
    const std::shared_ptr<canbus::Chassis>& chassis,
    const std::shared_ptr<localization::LocalizationEstimate>&
        localization_estimate) {
  /*
  1. ACHECK 是 Apollo 项目中自定义的一个宏，用于进行断言检查（Assertion Check）。
  2. ACHECK用法如下ACHECK(condition) << "Error message if condition is false";如果 condition 表达式为假（即条件不成立），ACHECK 将输出错误信息，然后触发 CHECK 失败。输出的错误信息会包含在程序的错误日志中，有助于开发人员识别问题所在。
  */
  ACHECK(prediction_obstacles != nullptr);

  // check and process possible rerouting request
  /*调用 CheckRerouting() 函数，用于检查是否需要重新规划路径，如果需要，则会发送路径请求。*/
  CheckRerouting();

  // process fused input data
  //TODO 后续看local_view_的用法
  local_view_.prediction_obstacles = prediction_obstacles;
  local_view_.chassis = chassis;
  local_view_.localization_estimate = localization_estimate;
  /*
  1. 下面的代码块是一个局部作用域的方式，常常用于限定临时变量的作用范围，确保在离开该作用域时自动释放资源（在这种情况下是 std::lock_guard 对象）。虽然它看起来像一个单独的代码块，但实际上是 PlanningComponent::Proc() 函数的一部分，它在函数的执行过程中起到了特定的作用。
  2. 这个代码块的主要作用是更新路由数据（local_view_.routing）。
  */
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!local_view_.routing ||
        hdmap::PncMap::IsNewRouting(*local_view_.routing, routing_)) {
      local_view_.routing =
          std::make_shared<routing::RoutingResponse>(routing_);
    }
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    local_view_.traffic_light =
        std::make_shared<TrafficLightDetection>(traffic_light_);
    local_view_.relative_map = std::make_shared<MapMsg>(relative_map_);
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    local_view_.pad_msg = std::make_shared<PadMessage>(pad_msg_);
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    local_view_.stories = std::make_shared<Stories>(stories_);
  }

  if (!CheckInput()) {
    AERROR << "Input check failed";
    return false;
  }

  if (config_.learning_mode() != PlanningConfig::NO_LEARNING) {
    // data process for online training
    message_process_.OnChassis(*local_view_.chassis);
    message_process_.OnPrediction(*local_view_.prediction_obstacles);
    message_process_.OnRoutingResponse(*local_view_.routing);
    message_process_.OnStoryTelling(*local_view_.stories);
    message_process_.OnTrafficLightDetection(*local_view_.traffic_light);
    message_process_.OnLocalization(*local_view_.localization_estimate);
  }

  // publish learning data frame for RL test
  if (config_.learning_mode() == PlanningConfig::RL_TEST) {
    PlanningLearningData planning_learning_data;
    LearningDataFrame* learning_data_frame =
        injector_->learning_based_data()->GetLatestLearningDataFrame();
    if (learning_data_frame) {
      planning_learning_data.mutable_learning_data_frame()
                            ->CopyFrom(*learning_data_frame);
      common::util::FillHeader(node_->Name(), &planning_learning_data);
      planning_learning_data_writer_->Write(planning_learning_data);
    } else {
      AERROR << "fail to generate learning data frame";
      return false;
    }
    return true;
  }

  ADCTrajectory adc_trajectory_pb;
  planning_base_->RunOnce(local_view_, &adc_trajectory_pb);
  common::util::FillHeader(node_->Name(), &adc_trajectory_pb);

  // modify trajectory relative time due to the timestamp change in header
  auto start_time = adc_trajectory_pb.header().timestamp_sec();
  const double dt = start_time - adc_trajectory_pb.header().timestamp_sec();
  for (auto& p : *adc_trajectory_pb.mutable_trajectory_point()) {
    p.set_relative_time(p.relative_time() + dt);
  }
  planning_writer_->Write(adc_trajectory_pb);

  // record in history
  auto* history = injector_->history();
  history->Add(adc_trajectory_pb);

  return true;
}


/*
1.mutable_rerouting() 函数的声明通常不会直接在代码中出现，它通常是通过 Protocol Buffers（ProtoBuf）的消息类生成的。ProtoBuf 允许您通过消息类的 mutable_ 方法来获取可变的子消息，而无需显式声明这些方法。这样做的好处是可以动态地访问和修改消息的字段。
2.在 Apollo 中，mutable_rerouting() 通常是 PlanningStatus 或类似的消息类型中的一个成员方法，而不需要显式的声明。通过调用 mutable_rerouting()，您可以获得一个可变的 rerouting 子消息，以便修改其中的字段，如上述代码中的 rerouting->set_need_rerouting(false)。
*/
void PlanningComponent::CheckRerouting() {
  auto* rerouting = injector_->planning_context()
                        ->mutable_planning_status()
                        ->mutable_rerouting();                   
  if (!rerouting->need_rerouting()) {
    return; //如果!rerouting->need_rerouting()条件成立，那么提前退出CheckRerouting函数，不在执行后续代码
  }
  common::util::FillHeader(node_->Name(), rerouting->mutable_routing_request());
  rerouting->set_need_rerouting(false);
  rerouting_writer_->Write(rerouting->routing_request());
}


/*
1. PlanningComponent::CheckInput() 函数的主要目的是确保规划模块的输入数据是有效的，如果有任何关键数据不可用，将设置错误原因，并且防止规划继续执行，以避免因缺少重要信息而导致错误的规划结果。这有助于确保规划模块的安全性和可靠性。*
*/
bool PlanningComponent::CheckInput() {
  ADCTrajectory trajectory_pb;
  auto* not_ready = trajectory_pb.mutable_decision()
                        ->mutable_main_decision()
                        ->mutable_not_ready();

  if (local_view_.localization_estimate == nullptr) {
    not_ready->set_reason("localization not ready");
  } else if (local_view_.chassis == nullptr) {
    not_ready->set_reason("chassis not ready");
  } else if (HDMapUtil::BaseMapPtr() == nullptr) {
    not_ready->set_reason("map not ready");
  } else {
    // nothing
  }

  if (FLAGS_use_navigation_mode) {
    if (!local_view_.relative_map->has_header()) {
      not_ready->set_reason("relative map not ready");
    }
  } else {
    if (!local_view_.routing->has_header()) {
      not_ready->set_reason("routing not ready");
    }
  }

  if (not_ready->has_reason()) {
    AERROR << not_ready->reason() << "; skip the planning cycle.";
    common::util::FillHeader(node_->Name(), &trajectory_pb);
    planning_writer_->Write(trajectory_pb);
    return false;
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
