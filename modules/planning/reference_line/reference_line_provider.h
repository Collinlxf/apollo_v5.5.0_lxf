/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

/**
 * @file reference_line_provider.h
 *
 * @brief Declaration of the class ReferenceLineProvider.
 */

#pragma once

#include <list>
#include <memory>
#include <queue>
#include <string>
#include <unordered_set>
#include <vector>

#include "cyber/cyber.h"
#include "modules/common/util/factory.h"
#include "modules/common/util/util.h"
#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/pnc_map/pnc_map.h"
#include "modules/map/relative_map/proto/navigation.pb.h"
#include "modules/planning/common/indexed_queue.h"
#include "modules/planning/math/smoothing_spline/spline_2d_solver.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/reference_line/discrete_points_reference_line_smoother.h"
#include "modules/planning/reference_line/qp_spline_reference_line_smoother.h"
#include "modules/planning/reference_line/reference_line.h"
#include "modules/planning/reference_line/spiral_reference_line_smoother.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

/**
 * @class ReferenceLineProvider
 * @brief The class of ReferenceLineProvider.
 *        It provides smoothed reference line to planning.
 */
/**
 * 类概述
   1. 功能：提供平滑的参考线给规划模块使用。
2. 成员变量：
    2.1 smoother_：平滑器，用于对参考线进行平滑处理。
    2.2 pnc_map_：规划地图，用于创建基于路由和当前位置的参考线和相关片段。
    2.3 relative_map_：相对地图，提供相对于车辆的地图信息。
    2.4 vehicle_state_：车辆状态，包括车辆位置、速度等信息。
    2.5 routing_：路由信息，提供规划参考线的路径信息。
    2.6 reference_lines_：储存计算的参考线。
    2.7 route_segments_：参考线对应的路段信息。
    2.8 last_calculation_time_：上次计算参考线的时间。
3. 主要成员函数
    3.1 ReferenceLineProvider(): 默认构造函数。
    3.2 ReferenceLineProvider(...): 带参数的构造函数，用于初始化各种依赖项。
    3.3 ~ReferenceLineProvider(): 默认析构函数。
    3.4 UpdateRoutingResponse(const routing::RoutingResponse& routing): 更新路由信息。
    3.5 UpdateVehicleState(const common::VehicleState& vehicle_state): 更新车辆状态信息。
    3.6 Start() / Stop(): 启动/停止参考线提供器。
    3.7 GetReferenceLines(...): 获取平滑的参考线和相关路段。
    3.8 LastTimeDelay(): 获取上次计算参考线的时间延迟。
    3.9 FutureRouteWaypoints(): 获取未来路径上的车道信息。
    3.10 UpdatedReferenceLine(): 判断参考线是否已经更新。
    3.11 IsValidReferenceLine(): 判断计算的参考线是否有效。
    3.12 Shrink(...): 根据给定的SL点缩短参考线。
 * **/
class ReferenceLineProvider {
 public:
  ReferenceLineProvider() = default;
  /*
  1. 构造函数初始化：构造函数的目标是在创建 ReferenceLineProvider 对象时初始化其成员变量和依赖项，以确保后续的参考线生成工作可以正常进行。
  2. const common::VehicleStateProvider* vehicle_state_provider：这是一个指向车辆状态提供者对象的指针，用于获取车辆的当前状态信息，例如位置、速度等。
  3. const hdmap::HDMap* base_map：这是一个指向 HD 地图对象的指针，用于获取高精度地图的信息。HDMap 是 Apollo 中用于处理高精度地图的类。
  4. const std::shared_ptr<relative_map::MapMsg>& relative_map = nullptr：这是一个可选参数，是指向相对地图消息对象的共享指针。相对地图提供了车辆周围环境的相对信息，包括其他车辆的位置等。这是一个可选参数，因此可以传递 nullptr，表示相对地图信息在构造时可能不可用。
  */ 
  ReferenceLineProvider(
      const common::VehicleStateProvider* vehicle_state_provider,
      const hdmap::HDMap* base_map,
      const std::shared_ptr<relative_map::MapMsg>& relative_map = nullptr);

  /**
   * @brief Default destructor.
   */
  ~ReferenceLineProvider();

  bool UpdateRoutingResponse(const routing::RoutingResponse& routing);

  void UpdateVehicleState(const common::VehicleState& vehicle_state);

  bool Start();

  void Stop();
  
  /**
 * @brief 获取平滑的参考线和相关路段。
 * @param reference_lines 用于存储平滑的参考线的列表。
 * @param segments 用于存储参考线对应的路段的列表。
 * @return 如果成功生成参考线和路段，则返回 true；否则返回 false。
 *
 * 该函数通过调用内部的 CreateReferenceLine 函数，基于车辆当前状态、路由信息和地图，生成平滑的参考线和相应的路段。
 * 如果生成成功，将更新成员变量 reference_lines_ 和 route_segments_。
 * ```
 * 示例用途：
 * ReferenceLineProvider reference_line_provider;
 * std::list<ReferenceLine> smoothed_reference_lines;
 * std::list<hdmap::RouteSegments> route_segments;
 * if (reference_line_provider.GetReferenceLines(&smoothed_reference_lines, &route_segments)) {
 *   // 成功获取平滑的参考线和路段，可以用于规划。
 *   // ...
 * } else {
 *   // 获取失败，处理错误情况。
 *   // ...
 * }
 * ```
 */
  bool GetReferenceLines(std::list<ReferenceLine>* reference_lines,
                         std::list<hdmap::RouteSegments>* segments);
  
  /**
 * @brief 获取上次计算参考线的时间延迟。
 * @return 上次计算参考线的时间延迟，单位为秒。
 *
 * 该函数用于测量上次计算参考线的时间延迟，即从计算开始到当前时刻的时间间隔。
 * 这个时间延迟信息可以在规划模块中用于监测参考线生成的效率和性能。
  */
  double LastTimeDelay();
  

  /**
 * @brief 获取未来路径上的车道信息
 * @return 未来路径上的车道信息，以 LaneWaypoint 对象的 vector 形式返回。
 *
 * 该函数用于获取未来路径上的车道信息，返回的是车道上一系列的路标点（waypoints），每个点包含车道ID、道路类型、相对位置等信息。
 * 车道信息在规划中常被用于辅助决策，例如判断车辆是否需要变道，预测车道的行驶情况等。
 *
 * 示例用途：
 * ReferenceLineProvider reference_line_provider;
 * std::vector<routing::LaneWaypoint> future_waypoints = reference_line_provider.FutureRouteWaypoints();
 * // 使用未来路径上的车道信息进行规划决策
 * ```
 */
  std::vector<routing::LaneWaypoint> FutureRouteWaypoints();
  
  /**
 * @brief 判断当前参考线是否已经更新
 * @param 无
 * @return 如果参考线已经更新，返回 true；否则返回 false
 *
 * 该函数通过读取原子变量 is_reference_line_updated_ 的值，判断当前参考线是否已经被更新。
 * 参考线的更新通常发生在规划模块计算了新的参考线并准备供其他模块使用时。
 * 这个函数的主要目的是提供一个机制，让其他模块知道当前的参考线状态，以便它们在需要时可以及时获取最新的参考线信息。
 * 使用场景可能包括但不限于：在新的参考线生成后，执行与参考线相关的其他任务。
 */
  bool UpdatedReferenceLine() { return is_reference_line_updated_.load(); }

 private:
  /**
   * @brief Use PncMap to create reference line and the corresponding segments
   * based on routing and current position. This is a thread safe function.
   * @return true if !reference_lines.empty() && reference_lines.size() ==
   *                 segments.size();
   **/

  /**
 * @brief 生成参考线并划分路段
 * @param reference_lines 用于存储生成的平滑参考线的列表
 * @param segments 用于存储参考线对应的路段的列表
 * @return 如果成功生成参考线并划分路段，返回 true；否则，返回 false
 *
 * 该函数的主要功能是使用 PncMap 根据当前的路由和车辆位置生成参考线，并根据路由和车辆位置划分路段。
 * 参考线和路段的生成是基于当前路由和车辆位置，通过 PncMap 进行线程安全的处理。
 * 如果成功生成参考线和划分路段，则返回 true，否则返回 false。
 */
  bool CreateReferenceLine(std::list<ReferenceLine>* reference_lines,
                           std::list<hdmap::RouteSegments>* segments);

  /**
   * @brief store the computed reference line. This function can avoid
   * unnecessary copy if the reference lines are the same.
   */
  /**
 * @brief 更新参考线并设置参考线优先级
 * @param reference_lines 新生成的参考线
 * @param route_segments 对应的路段信息
 *
 * 该函数用于更新当前参考线提供器的参考线，并为这些参考线设置优先级。在规划场景中，可能存在多条可选的参考线，通过设置优先级来确定规划算法应该选择哪一条参考线。这个函数接受新生成的参考线和对应的路段信息，确保参考线和路段信息一一对应。
 * 示例用途：
 * ReferenceLineProvider reference_line_provider;
 * std::list<ReferenceLine> new_reference_lines;
 * std::list<hdmap::RouteSegments> new_route_segments;
 * // ...（生成新的参考线和路段信息）
 * reference_line_provider.UpdateReferenceLine(new_reference_lines, new_route_segments);
 */

/*
与 UpdatedReferenceLine 函数相比，UpdatedReferenceLine 函数用于判断参考线是否已经更新过。
而 UpdateReferenceLine 函数则是用于执行更新操作。在规划模块中，通常会先调用 UpdatedReferenceLine 函数检查是否需要更新，如果需要，则再调用 UpdateReferenceLine 函数进行更新。
这样的设计可以提高效率，避免不必要的更新操作。
*/
  void UpdateReferenceLine(
      const std::list<ReferenceLine>& reference_lines,
      const std::list<hdmap::RouteSegments>& route_segments);
  
  /**
 * @brief 生成参考线的工作线程函数
 * @param 无参数
 * @return 无返回值
 *
 * 该函数是 `ReferenceLineProvider` 类中的一个工作线程函数，用于在后台生成参考线。
 * 在多线程环境中，通过这个函数实现异步生成参考线，提高系统的响应性和规划效率。
 * 
 * 实际用途举例：
 * 当规划模块启动后，可以通过调用 `Start()` 函数启动参考线生成工作线程，
 * 工作线程会在后台根据车辆状态、路由信息等异步生成参考线，以确保规划模块随时可以获取最新的参考线进行路径规划。
 * ```
 * 示例用法：
 * ReferenceLineProvider reference_line_provider;
 * reference_line_provider.Start();  // 启动参考线生成工作线程
 * ```
 */
  void GenerateThread();

  /**
 * @brief 判断当前的参考线是否有效。
 *
 * 参考线的有效性检查涉及多个方面，包括是否成功生成了参考线、是否满足平滑度等要求。
 * 有效的参考线可以被用于规划，而无效的参考线可能导致规划失败或者生成不符合要求的轨迹。
 *
 * @return 参考线是否有效，返回 true 表示有效，false 表示无效。
 *
 * 示例用途：
 * planning::ReferenceLineProvider reference_line_provider;
 * if (reference_line_provider.IsValidReferenceLine()) {
 *   // 如果当前参考线有效，执行规划操作
 *   planner.Plan(reference_line_provider.GetReferenceLines());
 * } else {
 *   // 如果当前参考线无效，采取其他策略或者等待下一次参考线更新
 *   planner.Wait();
 * }
 */
  void IsValidReferenceLine();

  /**
 * @brief 为变道路段设置优先级
 * @param route_segments 要设置优先级的路段列表
 * @return 无
 *
 * 该函数用于为变道路段设置优先级，以影响规划时的参考线选择。优先级的使用场景包括在存在多条可能的参考线时，通过设置不同路段的优先级来决定哪一条参考线更适合当前的规划情境。
 * 优先级值越高，表示规划系统更倾向于选择该参考线。
 * ```
 * 示例用途：
 * std::list<hdmap::RouteSegments> route_segments;
 * reference_line_provider.PrioritzeChangeLane(&route_segments);  // 为变道路段设置优先级
 * ```
 * 
 * 举例：
假设在某个交叉口附近，有两条可选的规划路径，一条包含变道路段 A，另一条包含变道路段 B。
通过调用 PrioritzeChangeLane 函数，系统根据当前交叉口的车流情况和路况信息，为这两个变道路段设置了不同的优先级。
如果路段 A 被设置了更高的优先级，规划系统就更有可能选择包含路段 A 的路径，以确保平稳且合理的变道行为。
 */
  void PrioritzeChangeLane(std::list<hdmap::RouteSegments>* route_segments);

  bool CreateRouteSegments(const common::VehicleState& vehicle_state,
                           std::list<hdmap::RouteSegments>* segments);

  bool IsReferenceLineSmoothValid(const ReferenceLine& raw,
                                  const ReferenceLine& smoothed) const;

  bool SmoothReferenceLine(const ReferenceLine& raw_reference_line,
                           ReferenceLine* reference_line);

  bool SmoothPrefixedReferenceLine(const ReferenceLine& prefix_ref,
                                   const ReferenceLine& raw_ref,
                                   ReferenceLine* reference_line);

  void GetAnchorPoints(const ReferenceLine& reference_line,
                       std::vector<AnchorPoint>* anchor_points) const;

  bool SmoothRouteSegment(const hdmap::RouteSegments& segments,
                          ReferenceLine* reference_line);

  /**
   * @brief This function creates a smoothed forward reference line
   * based on the given segments.
   */
  bool ExtendReferenceLine(const common::VehicleState& state,
                           hdmap::RouteSegments* segments,
                           ReferenceLine* reference_line);

  AnchorPoint GetAnchorPoint(const ReferenceLine& reference_line,
                             double s) const;

  bool GetReferenceLinesFromRelativeMap(
      std::list<ReferenceLine>* reference_lines,
      std::list<hdmap::RouteSegments>* segments);

  /**
   * @brief This function get adc lane info from navigation path and map
   * by vehicle state.
   */
  bool GetNearestWayPointFromNavigationPath(
      const common::VehicleState& state,
      const std::unordered_set<std::string>& navigation_lane_ids,
      hdmap::LaneWaypoint* waypoint);

  bool Shrink(const common::SLPoint& sl, ReferenceLine* ref,
              hdmap::RouteSegments* segments);

 private:
  bool is_initialized_ = false;
  std::atomic<bool> is_stop_{false};

  std::unique_ptr<ReferenceLineSmoother> smoother_;
  ReferenceLineSmootherConfig smoother_config_;

  std::mutex pnc_map_mutex_;
  std::unique_ptr<hdmap::PncMap> pnc_map_;

  // Used in Navigation mode
  std::shared_ptr<relative_map::MapMsg> relative_map_;

  std::mutex vehicle_state_mutex_;
  common::VehicleState vehicle_state_;

  std::mutex routing_mutex_;
  routing::RoutingResponse routing_;
  bool has_routing_ = false;

  std::mutex reference_lines_mutex_;
  std::list<ReferenceLine> reference_lines_;
  std::list<hdmap::RouteSegments> route_segments_;
  double last_calculation_time_ = 0.0;

  std::queue<std::list<ReferenceLine>> reference_line_history_;
  std::queue<std::list<hdmap::RouteSegments>> route_segments_history_;

  std::future<void> task_future_;

  std::atomic<bool> is_reference_line_updated_{true};

  const common::VehicleStateProvider* vehicle_state_provider_ = nullptr;
};

}  // namespace planning
}  // namespace apollo
