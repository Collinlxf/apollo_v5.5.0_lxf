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
 * @file reference_line.h
 **/

#pragma once

#include <string>
#include <utility>
#include <vector>

#include "modules/common/math/vec2d.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/map/pnc_map/path.h"
#include "modules/map/proto/map.pb.h"
#include "modules/map/proto/map_geometry.pb.h"
#include "modules/planning/proto/sl_boundary.pb.h"
#include "modules/planning/reference_line/reference_point.h"
#include "modules/routing/proto/routing.pb.h"

namespace apollo {
namespace planning {

class ReferenceLine {
 public:
 /*
 1. 默认构造函数
 */
  ReferenceLine() = default;
  /*
  1. 复制构造函数
  */
  explicit ReferenceLine(const ReferenceLine& reference_line) = default;
  template <typename Iterator>
  /*
  1. 参数化构造函数
  2. 作用：从给定的迭代器范围构造 ReferenceLine 对象，同时构造 map_path_。
  */
  ReferenceLine(const Iterator begin, const Iterator end)
      : reference_points_(begin, end),
        map_path_(std::move(std::vector<hdmap::MapPathPoint>(begin, end))) {}
  /*
  1. 参数化构造函数
  2. 作用：从给定的参考点构造 ReferenceLine 对象。
  */
  explicit ReferenceLine(const std::vector<ReferencePoint>& reference_points);
  /*
  1. 参数化构造函数
  2. 参数：hdmap_path 是 hdmap::Path 类型的对象，表示高精度地图上的路径。
  3. 作用：从给定的高精度地图路径构造 ReferenceLine 对象。
  */  
  explicit ReferenceLine(const hdmap::Path& hdmap_path);

  /** Stitch current reference line with the other reference line
   * The stitching strategy is to use current reference points as much as
   * possible. The following two examples show two successful stitch cases.
   *
   * Example 1
   * this:   |--------A-----x-----B------|
   * other:                 |-----C------x--------D-------|
   * Result: |------A-----x-----B------x--------D-------|
   * In the above example, A-B is current reference line, and C-D is the other
   * reference line. If part B and part C matches, we update current reference
   * line to A-B-D.
   *
   * Example 2
   * this:                  |-----A------x--------B-------|
   * other:  |--------C-----x-----D------|
   * Result: |--------C-----x-----A------x--------B-------|
   * In the above example, A-B is current reference line, and C-D is the other
   * reference line. If part A and part D matches, we update current reference
   * line to C-A-B.
   * 作用：尝试将当前参考线与另一条参考线拼接在一起，使得两者之间没有重叠。
   * 实例路况举例：
   * 1. 拓展驶入区域：当车辆需要进入某个区域，但是当前参考线的终点处于这个区域外，而该区域又存在一条新的参考线时，可以使用 Stitch 函数将当前参考线和新的参考线拼接在一起，以确保车辆可以顺利进入该区域。
   * 2. 道路分支：当道路出现分支时，不同分支可能有不同的参考线。在车辆接近分叉点时，可以使用 Stitch 函数将当前参考线与新的分支参考线拼接，以确保规划的平滑性和连贯性。
   * 3. 绕过障碍物：当参考线上的某一段由于障碍物不可通行，规划需要绕过障碍物时，可以使用 Stitch 函数将规划绕过障碍物的参考线与原始参考线拼接在一起，以确保规划的流畅性。
   * 4. 车道变更：在需要进行车道变更的情况下，可能存在一条新的车道参考线。使用 Stitch 函数可以将当前车道参考线与新的车道参考线拼接，以平滑地实现车道变更。
   * 5. 区域路径规划：当需要规划穿越某个区域的路径时，该区域可能有独立的参考线。使用 Stitch 函数将当前路径参考线与区域内的参考线拼接，确保规划路径的连续性。
   * @return false if these two reference line cannot be stitched
   */

  bool Stitch(const ReferenceLine& other);

  /*
  1. Segment 函数的作用是对参考线进行分割，这在规划模块中可能与路径规划和行驶策略有关。它允许在参考线上的特定位置进行分割，以获得满足某些条件的子参考线。
  2. 两个Segment是函数重载的用法，Segment被调用时根据参数来区别调用的是哪个函数
  */
  bool Segment(const common::math::Vec2d& point, const double distance_backward,
               const double distance_forward);

  bool Segment(const double s, const double distance_backward,
               const double distance_forward);
  
  /*
  1. const hdmap::Path& - 返回参考线对应的地图路径。
  2. 高精度地图通常包含车道信息、交叉口拓扑、障碍物位置等详细信息。ReferenceLine 是规划模块中用于规划车辆轨迹的参考线，它在高精度地图上进行定义。
  */
  const hdmap::Path& map_path() const;
  /*
  1. const std::vector<ReferencePoint>& - 返回参考线上的参考点。
  2. 这些参考点是沿着车辆规划路径均匀分布的。假设你在规划模块中需要查询某个位置的参考信息，你可以使用这个函数，传入空间中的横向位置（例如xy坐标系下的一个点），然后这个函数会返回在该位置最近的参考点。
  */
  const std::vector<ReferencePoint>& reference_points() const;
  
  /*
  1. GetReferencePoint函数根据参考线上的 s 值获取参考点。
  2. 参数s：参考线上的纵向位置。
  3. 使用案例：
    3.1 在规划过程中，车辆需要按照参考线行驶。为了控制车辆，规划算法需要不断查询参考线上的参考点，以获取车辆当前所在位置的状态信息。
    3.2 当规划算法需要获取车辆某一时刻的参考点时，可以调用 GetReferencePoint 函数，传入车辆纵向位置 s，以获取该位置的参考点信息。
    3.3 获取到的参考点包含了车辆在该位置的坐标、速度、曲率等信息，这些信息对于路径规划和车辆控制至关重要。
    
    double current_s = 10.0;  // 假设车辆当前纵向位置为 10 米
    ReferencePoint current_point = reference_line.GetReferencePoint(current_s);

  */
  ReferencePoint GetReferencePoint(const double s) const;

  /*
  message FrenetFramePoint {
  optional double s = 1;
  optional double l = 2;
  optional double dl = 3;
  optional double ddl = 4;
}


message PathPoint {
  // coordinates
  //x, y, z 分别表示路径点的三维空间坐标，通常表示车辆在世界坐标系下的位置。
  optional double x = 1;
  optional double y = 2;
  optional double z = 3;

  // direction on the x-y plane
  //theta 表示路径点在平面上的方向角度。
  optional double theta = 4;
  // curvature on the x-y planning
  //kappa 表示路径点处的曲率，用于描述路径的弯曲程度。
  optional double kappa = 5;
  // accumulated distance from beginning of the path
  //s 表示从路径的起点开始累积的距离，通常用于表示路径上的位置。
  optional double s = 6;

  // derivative of kappa w.r.t s.
  //dkappa 和 ddkappa 分别表示曲率 kappa 关于累积距离 s 的一阶和二阶导数。
  optional double dkappa = 7;
  // derivative of derivative of kappa w.r.t s.
  optional double ddkappa = 8;
  // The lane ID where the path point is on
  //lane_id 表示路径点所在的车道的唯一标识。
  optional string lane_id = 9;

  // derivative of x and y w.r.t parametric parameter t in CosThetareferenceline
  // x_derivative 和 y_derivative 表示路径点坐标 x 和 y 分别关于参数 t 的一阶导数。
  optional double x_derivative = 10;
  optional double y_derivative = 11;

  作用：这个函数在规划中用于将路径上的点转换到 Frenet 坐标系中。
}

  */
  common::FrenetFramePoint GetFrenetPoint(
      const common::PathPoint& path_point) const;

  /*
  1. ToFrenetFrame 函数用于将给定的轨迹点映射到 Frenet（弗雷内）坐标系中。Frenet坐标系通常用于描述车辆在车道中的位置，包括横向偏移（横向坐标 l）和纵向位置（纵向坐标 s）。
  2. 函数返回一个 std::pair，其中包含两个数组。第一个数组 [s, l, dl] 表示在 Frenet 坐标系中的纵向位置、横向位置和纵向速度，第二个数组 [ddl, ddl_2] 表示纵向加速度和纵向加速度的导数。
  */
  std::pair<std::array<double, 3>, std::array<double, 3>> ToFrenetFrame(
      const common::TrajectoryPoint& traj_point) const;
  

  /*
  1. 参数：start_s：参考线上的起始 s 值、end_s：参考线上的结束 s 值。
  2. 作用：该函数的主要作用是从参考线中提取在指定 s 范围内的参考点。这对于规划算法来说是关键的，因为规划通常需要在车辆当前位置附近获取一些参考点，以便进行路径规划和轨迹生成。
  3. 实际使用：在规划模块中，当需要在某个特定的 s 范围内获取参考点时，可以调用这个函数。例如，在规划算法的某个步骤中，可能需要获取车辆前方的一些参考点，用于生成平滑的轨迹。通过调用这个函数，可以获取指定 s 范围内的参考点，供后续规划步骤使用。
  */
  std::vector<ReferencePoint> GetReferencePoints(double start_s,
                                                 double end_s) const;
  
  /*
  1. 函数作用：GetNearestReferenceIndex 函数的作用是在参考线上找到最接近给定 s 值的参考点，并返回该点在 reference_points_ 中的索引。
  */
  size_t GetNearestReferenceIndex(const double s) const;
  
  /*
  1. GetNearestReferencePoint 函数通过给定的二维点坐标，找到最接近这个点的参考线上的参考点。
  */
  ReferencePoint GetNearestReferencePoint(const common::math::Vec2d& xy) const;
  
  /*
  1. GetLaneSegments函数用于从参考线上的 start_s 到 end_s 范围内，获取覆盖的车道段信息。
  2. hdmap::LaneSegment 包含车道的一部分信息，例如车道 ID、起始和结束 s 值、宽度等。
  */
  std::vector<hdmap::LaneSegment> GetLaneSegments(const double start_s,
                                                  const double end_s) const;
  /*
  1. GetNearestReferencePoint 函数是用来获取参考线上距离某个纵向位置 s 最近的参考点的。
  2. 实际使用：这个函数在规划中的常见用途是根据车辆当前的纵向位置，获取距离最近的参考点，以便进行规划决策。
  */
  ReferencePoint GetNearestReferencePoint(const double s) const;
  
  /*
  1. 通过提供的坐标 (x, y)，在参考线上查找距离最近的参考点。
  2. 此函数对于在规划过程中需要将二维坐标映射到参考线上的情况非常有用。例如，当规划器收到来自感知模块的目标位置时，可以使用这个函数找到最接近目标位置的参考点，以便在规划路径时参考该点的信息。
  */
  ReferencePoint GetReferencePoint(const double x, const double y) const;
  
  /*
  1. 作用：GetApproximateSLBoundary 函数的作用是获取一个近似的 SL 边界，这个 SL 边界是在指定 s 范围内对参考线上的一个给定车辆包围框（common::math::Box2d）的投影。具体来说，它计算车辆包围框在 s 范围内的投影，以近似表示车辆包围框在参考线上的位置。
  2. 参数：
    2.1 box：表示车辆包围框的 common::math::Box2d 对象。
    2.2 start_s 和 end_s：指定参考线上的 s 范围，用于确定投影的位置。
    2.3 sl_boundary：用于存储计算得到的近似 SL 边界的 SLBoundary 对象。
  3. 返回值：bool 类型，表示函数是否成功执行。如果在指定的 s 范围内找到了适当的投影，函数返回 true；否则返回 false。
  4. 函数实现解释：
    4.1 首先，通过调用 GetNearestReferencePoint 函数，找到 s 范围内距离车辆包围框最近的参考点。
    4.2 如果找到了最近的参考点，计算该参考点在车辆包围框局部坐标系下的坐标，以确定车辆包围框在参考线上的相对位置。
    4.3 根据车辆包围框在局部坐标系下的坐标，计算车辆包围框在参考线上的投影。
    4.4 将计算得到的投影信息填充到 SLBoundary 对象中，并返回 true 表示成功。
  */
  bool GetApproximateSLBoundary(const common::math::Box2d& box,
                                const double start_s, const double end_s,
                                SLBoundary* const sl_boundary) const;
  /*
  message SLBoundary {
  optional double start_s = 1;
  optional double end_s = 2;
  optional double start_l = 3;
  optional double end_l = 4;
  repeated apollo.common.SLPoint boundary_point = 5;
}
  */
  
  /*
  1. common::math::Box2d 用于表示一个矩形框
  2. hdmap::Polygon 用于表示一个多边形。
  3. 这两个函数的目的是这些函数的目的是将这些形状在参考线上的投影表示为 SL 边界。SL 边界提供了在参考线上的 s 和 l 坐标。
  */
  bool GetSLBoundary(const common::math::Box2d& box,
                     SLBoundary* const sl_boundary) const;
  bool GetSLBoundary(const hdmap::Polygon& polygon,
                     SLBoundary* const sl_boundary) const;
  /*
  1. 参数：sl_point：common::SLPoint 类型的参数，表示参考线上的SL坐标。xy_point：指向 common::math::Vec2d 类型对象的指针，用于存储转换后的XY坐标。
  2. 作用：将参考线上的SL坐标点转换为XY坐标系中的点。SL坐标系中的sl_point 包括s（纵向位置）和l（横向位置），而XY坐标系中的 xy_point 包括x和y。
  3. 具体实现：
     3.1 通过参考线上的 SLToXY 方法，使用 s 和 l 两个坐标参数，将SL坐标转换为XY坐标。
     3.2 如果转换成功，则将结果存储在传递给函数的 xy_point 指针指向的对象中，并返回 true；否则，返回 false。
  4. 主要目的：此函数的主要目的是提供一个方便的接口，使得在参考线上进行规划时可以轻松地在SL坐标系和XY坐标系之间进行转换。
  */
  bool SLToXY(const common::SLPoint& sl_point,
              common::math::Vec2d* const xy_point) const;
  bool XYToSL(const common::math::Vec2d& xy_point,
              common::SLPoint* const sl_point) const;
  /*
  这个模板版本可以接受不同类型的平面直角坐标点 XYPoint，并将其转换为 Vec2d 类型，然后再调用原始的 XYToSL 函数进行处理。
  */
  template <class XYPoint>
  bool XYToSL(const XYPoint& xy, common::SLPoint* const sl_point) const {
    return XYToSL(common::math::Vec2d(xy.x(), xy.y()), sl_point);
  }
  
  /**
 * @brief 获取参考线上指定 s 处的车道宽度。
 * @param s 参考线上的纵向位置。
 * @param lane_left_width 指向左车道宽度的指针，用于存储左侧车道的宽度。
 * @param lane_right_width 指向右车道宽度的指针，用于存储右侧车道的宽度。
 * @return 如果成功获取车道宽度，返回 true；否则返回 false。
 */
  bool GetLaneWidth(const double s, double* const lane_left_width,
                    double* const lane_right_width) const;

  /**
 * @brief 获取指定纵向位置 s 处的车道中心线到当前车道的横向偏移。
 * @param s 参考线上的纵向位置。
 * @param l_offset 输出参数，表示横向偏移。
 * @return 如果获取偏移成功，则返回 true，否则返回 false。
 */
  bool GetOffsetToMap(const double s, double* l_offset) const;
  
  /**
 * @brief 获取参考线上指定 s 处的道路宽度。
 * @param s 在参考线上的纵向位置。
 * @param road_left_width 指向道路左侧宽度的指针，用于存储道路左侧的宽度。
 * @param road_right_width 指向道路右侧宽度的指针，用于存储道路右侧的宽度。
 * @return 如果成功获取车道宽度信息，则返回 true；否则返回 false。
 * @details 该函数用于获取给定 s 处的车道宽度信息。在规划场景中，车道宽度可能不是均匀的，
 *          因此需要根据具体位置获取车道的左右宽度。如果成功获取车道宽度信息，将更新
 *          road_left_width 和 road_right_width 的值。
 */
  bool GetRoadWidth(const double s, double* const road_left_width,
                    double* const road_right_width) const;
  
  /**
 * @brief 获取给定 s 处的道路类型
 * @param s 参考线上的纵向位置
 * @return hdmap::Road::Type - 道路类型
 * @details 该函数根据参考线上的 s 值，查询地图路径，获取对应位置的道路类型。
 *          道路类型是 `hdmap::Road::Type` 枚举类型，表示道路的类别，如城市道路、高速公路等。
 */
  hdmap::Road::Type GetRoadType(const double s) const;
  
  /**
 * @brief 从给定的纵向位置 s 获取车道信息。
 * @param s 在参考线上的纵向位置。
 * @param lanes 用于存储车道信息的指针向量。
 * @return void
 * @details
 *   该函数根据参考线上的 s 值获取车道信息，并将结果存储在 lanes 指针向量中。
 *   车道信息以 hdmap::LaneInfoConstPtr 类型存储在 lanes 中。
 *   注意：s 处可能涉及多条车道，因此通过 lanes 向量返回所有相关的车道信息。
 */
  void GetLaneFromS(const double s,
                    std::vector<hdmap::LaneInfoConstPtr>* lanes) const;
  
  
/**
 * @brief 获取给定 SL 边界下的车道宽度。
 * @param sl_boundary SL 边界
 * @return 车道宽度
 * @details 该函数计算给定 SL 边界下车道的宽度。SL 边界是在参考线上定义的，其中包含了车道的起始和结束 s 值以及车道的左右边界。车道宽度是通过计算左右边界之间的距离来确定的。
 *          如果 SL 边界无效或车道宽度小于等于零，则返回零。
 */
 // 例如，如果车辆在某一段车道上有一侧是墙壁，另一侧是车道边缘，通过调用 GetDrivingWidth 函数，可以计算车辆在这段车道上的行驶宽度，确保车辆在沿着参考线行驶时不会与墙壁相撞。
  double GetDrivingWidth(const SLBoundary& sl_boundary) const;

  /**
   * @brief: check if a box/point is on lane along reference line
   * 实际用途举例：假设有一个自动驾驶车辆正在行驶，系统需要检查车辆当前的位置是否在规划的参考线上，以确保车辆保持在规划好的路径上。这个检查在不同的场景下都很有用。
   */

  /**
 * @brief 判断给定的 SL 点是否在参考线上。
 * @param sl_point SL 点的位置。
 * @return 如果 SL 点在参考线上，则返回 true；否则返回 false。
 */
  bool IsOnLane(const common::SLPoint& sl_point) const;

  /**
 * @brief 判断给定的二维坐标点是否在参考线上。
 * @param vec2d_point 二维坐标点。
 * @return 如果二维坐标点在参考线上，则返回 true；否则返回 false。
 */
  bool IsOnLane(const common::math::Vec2d& vec2d_point) const;

  /**
 * @brief 通过模板函数判断给定的点是否在参考线上。
 * @param xy 任意类型的点，支持包含 x() 和 y() 方法的类型。
 * @return 如果点在参考线上，则返回 true；否则返回 false。
 */
  template <class XYPoint>
  bool IsOnLane(const XYPoint& xy) const {
    return IsOnLane(common::math::Vec2d(xy.x(), xy.y()));
  }

  /**
 * @brief 判断给定的 SL 边界是否与参考线有交集。
 * @param sl_boundary SL 边界。
 * @return 如果 SL 边界与参考线有交集，则返回 true；否则返回 false。
 */
  bool IsOnLane(const SLBoundary& sl_boundary) const;

  /**
   * @brief: check if a box/point is on road
   *         (not on sideways/medians) along reference line
   * 实际用途举例：假设有一辆车正在行驶，而规划模块需要检查车辆当前位置是否在车道上。这就是这些函数可能被使用的场景。
   */

  /**
 * @brief 检查给定的SL坐标点是否在车道上。
 * @param sl_point 参考线上的SL坐标点。
 * @return 如果SL坐标点在车道上，则返回true；否则，返回false。
 */
  bool IsOnRoad(const common::SLPoint& sl_point) const;
  /**
 * @brief 检查给定的二维坐标点是否在车道上。
 * @param vec2d_point 二维坐标点。
 * @return 如果二维坐标点在车道上，则返回true；否则，返回false。
 */
  bool IsOnRoad(const common::math::Vec2d& vec2d_point) const;

  /**
 * @brief 检查给定的SL边界是否完全在车道上。
 * @param sl_boundary SL边界。
 * @return 如果SL边界完全在车道上，则返回true；否则，返回false。
 */
  bool IsOnRoad(const SLBoundary& sl_boundary) const;

  /**
   * @brief Check if a box is blocking the road surface. The criteria is to
   * check whether the remaining space on the road surface is larger than the
   * provided gap space.
   * @param boxed the provided box
   * @param gap check the gap of the space
   * @return true if the box blocks the road.
   */

  /**
 * @brief 检查一个框是否阻挡了道路表面。
 * @param box2d 要检查的框。
 * @param gap 检查空间的间隙大小。
 * @return 如果框阻挡了道路，则返回 true。
 */
//  该函数用于检查给定的矩形框（通常表示车辆或物体）是否阻挡了道路表面。
// 在规划中，这个函数可以用于判断当前车辆是否占用了规划路径上的道路空间，以便进行决策。
// 举例来说，当车辆需要变道时，系统可能会使用这个函数来检查变道过程中是否有其他车辆或障碍物占用了目标车道，以避免碰撞。
  bool IsBlockRoad(const common::math::Box2d& box2d, double gap) const;

  /**
   * @brief check if any part of the box has overlap with the road.
   */

  /**
 * @brief 判断给定的矩形框是否与参考线有重叠。
 * @param box 给定的矩形框。
 * @return 如果有重叠，返回 true；否则返回 false。
 */
  // 实际用途举例：
  // 在规划中，可能会使用此函数来检查车辆所在的区域是否与参考线有重叠，
  // 以避免规划路径与障碍物相交或其他不安全的情况。
  // 例如，当车辆位于参考线上，可能需要确保车辆周围的区域不会与参考线重叠，
  // 以确保规划的路径是安全的。
  // 如果存在重叠，可能需要调整规划或采取其他措施来确保安全性。
  // 返回值 true 表示存在重叠，false 表示不存在重叠。
  // 注意：此处的 "重叠" 定义取决于具体的业务场景和使用上下文。
  // 具体而言，可以是碰撞检测、安全区域的判断等。
  // 在不同的应用场景中，可能需要根据具体需要调整重叠的定义。
  // 例如，可以根据车辆的大小、形状等来调整判断条件。
  // 在实际使用中，开发人员可能需要根据具体的规划需求和系统特性来决定如何使用此函数。
  // 对于不同的车辆和规划场景，可能需要调整判断条件以满足具体要求。
  // 总体而言，此函数用于规划系统中对参考线与特定区域之间关系的检查。
  bool HasOverlap(const common::math::Box2d& box) const;
  
  /**
 * @brief 获取参考线的长度
 * @return 参考线的长度，即对应地图路径的长度
 */
  // 假设有一个规划任务，需要在参考线上按照某种规则生成路径点，而生成的路径点的密度或间隔需要与参考线的长度相关。在这种情况下，可以使用 Length 函数获取参考线的长度，然后根据长度信息进行路径点的生成。
  // 例如，可以根据参考线的长度确定路径点的生成密度，以确保生成的路径点在整个参考线上均匀分布。
  double Length() const { return map_path_.length(); }
  
  /**
 * @brief 获取 `ReferenceLine` 对象的调试信息。
 * @return 包含对象信息的字符串。
 * @details 此函数返回包含 `ReferenceLine` 对象详细信息的字符串，用于调试和日志记录。
 *          该字符串包括参考线的长度、优先级等信息，以及每个参考点的详细信息。
 *          通常用于调试和排查问题时，打印对象的详细信息以便进行分析。
 *          例如，当规划模块产生异常结果时，通过调用此函数，可以输出 `ReferenceLine`
 *          对象的详细信息，有助于定位问题出现的原因。
 */
  std::string DebugString() const;
  
  /**
 * @brief 根据纵向位置 s 获取参考线上的速度限制。
 * @param s 纵向位置。
 * @return 参考线上 s 处的速度限制。如果没有速度限制，则返回默认值。
 */
//  在进行车道变更决策时，规划算法可能会查询当前车辆所在车道的参考线，然后使用 GetSpeedLimitFromS 函数获取当前位置的速度限制。
// 这个速度限制可以用于调整车辆的速度，确保在变更车道时不会超速。
// 因此，该函数在规划中用于获取车辆在参考线上特定位置的速度限制信息，以指导规划算法的决策。
  double GetSpeedLimitFromS(const double s) const;
  
  /**
 * @brief 添加指定范围的速度限制信息。
 * @param start_s 起始 s 值，参考线上的纵向位置。
 * @param end_s 结束 s 值，参考线上的纵向位置。
 * @param speed_limit 速度限制值，单位：米每秒（m/s）。
 * @return void
 */
// 假设在一段参考线上的某个区间（从 start_s 到 end_s）存在临时的交通限速区域，可以通过调用 AddSpeedLimit 函数，将这个速度限制信息添加到参考线中。
  void AddSpeedLimit(double start_s, double end_s, double speed_limit);
  
  /**
 * @brief 获取参考线的优先级。
 * @return 一个整数，表示参考线的优先级。数值越大，优先级越高。
 */
  uint32_t GetPriority() const { return priority_; }
  

  /**
 * @brief 设置参考线的优先级。
 * @param priority 优先级值，一个正整数。
 * @return void
 *
 * 该函数用于为当前的参考线对象设置一个优先级值。优先级值越高，表示规划时更倾向于选择该参考线。
 * 优先级的使用场景可以是在多条可能的参考线中，通过设置优先级来决定哪一条参考线更适合当前的规划情境。
 * ```
 * 示例用途：
 * ReferenceLine reference_line;
 * reference_line.SetPriority(2);  // 设置参考线的优先级为 2
 * ```
 */
  void SetPriority(uint32_t priority) { priority_ = priority; }
  
  /**
 * @brief 获取当前参考线对象对应的高精度地图路径。
 * @return 高精度地图路径对象的引用。
 *
 * 该函数返回当前参考线对象所代表的高精度地图路径，以供规划模块使用。通过该路径对象，可以获取参考线的详细地图信息，
 * 包括车道、道路类型、坐标等。这提供了规划算法在参考线上进行更精细化的路径规划和决策制定的可能性。
 * ```
 * 示例用途：
 * const hdmap::Path& map_path = reference_line.GetMapPath();
 * // 在 map_path 上执行相关地图查询和分析操作
 * ```
 */
  const hdmap::Path& GetMapPath() const { return map_path_; }

 private:
  /**
   * @brief Linearly interpolate p0 and p1 by s0 and s1.
   * The input has to satisfy condition: s0 <= s <= s1
   * p0 and p1 must have lane_waypoint.
   * Note: it requires p0 and p1 are on the same lane, adjacent lanes, or
   * parallel neighboring lanes. Otherwise the interpolated result may not
   * valid.
   * @param p0 the first anchor point for interpolation.
   * @param s0 the longitutial distance (s) of p0 on current reference line.
   * s0 <= s && s0 <= s1
   * @param p1 the second anchor point for interpolation
   * @param s1 the longitutial distance (s) of p1 on current reference line.
   * s1
   * @param s identifies the middle point that is going to be
   * interpolated.
   * s >= s0 && s <= s1
   * @return The interpolated ReferencePoint.
   */
  static ReferencePoint Interpolate(const ReferencePoint& p0, const double s0,
                                    const ReferencePoint& p1, const double s1,
                                    const double s);
  ReferencePoint InterpolateWithMatchedIndex(
      const ReferencePoint& p0, const double s0, const ReferencePoint& p1,
      const double s1, const hdmap::InterpolatedIndex& index) const;

  static double FindMinDistancePoint(const ReferencePoint& p0, const double s0,
                                     const ReferencePoint& p1, const double s1,
                                     const double x, const double y);

 private:
  struct SpeedLimit {
    double start_s = 0.0;
    double end_s = 0.0;
    double speed_limit = 0.0;  // unit m/s
    SpeedLimit() = default;
    SpeedLimit(double _start_s, double _end_s, double _speed_limit)
        : start_s(_start_s), end_s(_end_s), speed_limit(_speed_limit) {}
  };
  /**
   * This speed limit overrides the lane speed limit
   **/
  std::vector<SpeedLimit> speed_limit_;
  std::vector<ReferencePoint> reference_points_;
  hdmap::Path map_path_;
  uint32_t priority_ = 0;
};

}  // namespace planning
}  // namespace apollo
