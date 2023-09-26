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

#ifndef CYBER_COMPONENT_COMPONENT_BASE_H_
#define CYBER_COMPONENT_COMPONENT_BASE_H_

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "gflags/gflags.h"

#include "cyber/proto/component_conf.pb.h"

#include "cyber/class_loader/class_loader.h"
#include "cyber/common/environment.h"
#include "cyber/common/file.h"
#include "cyber/node/node.h"
#include "cyber/scheduler/scheduler.h"

namespace apollo {
namespace cyber {

using apollo::cyber::proto::ComponentConfig;
using apollo::cyber::proto::TimerComponentConfig;

class ComponentBase : public std::enable_shared_from_this<ComponentBase> {
 public:
  template <typename M>
  using Reader = cyber::Reader<M>;

  virtual ~ComponentBase() {}

  virtual bool Initialize(const ComponentConfig& config) { return false; }
  virtual bool Initialize(const TimerComponentConfig& config) { return false; }
  virtual void Shutdown() {
    if (is_shutdown_.exchange(true)) {
      return;
    }

    Clear();
    for (auto& reader : readers_) {
      reader->Shutdown();
    }
    scheduler::Instance()->RemoveTask(node_->Name());
  }

  /*lxf
  1. 这段代码是一个 C++ 模板函数，用于从指定的配置文件中加载 Protobuf 格式的配置信息，并将其填充到给定的 config 对象中。
  2. template <typename T>：这是一个函数模板声明，表示接下来定义的函数是一个模板函数，可以用于不同类型的参数 T。在这里，T 是一个模板类型参数，表示配置对象的类型。
  3. bool GetProtoConfig(T* config)：这是模板函数的函数签名。它接受一个指向类型为 T 的配置对象的指针 config 作为参数，并返回一个布尔值，指示是否成功加载配置信息。
  4. bool GetProtoConfig(T* config) const {}函数返回一个布尔值，指示是否成功加载配置信息。如果加载成功，返回 true；否则，返回 false。
  **5. const表示不会修改类的成员变量：const 成员函数不能修改类的非静态成员变量。在这里，它表示该成员函数不会修改 config_file_path_，即不会修改类的成员变量 config_file_path_ 的值。
  6. common::GetProtoFromFile(config_file_path_, config)：这是实际的配置加载操作，它调用了一个名为 GetProtoFromFile 的函数，将配置文件 config_file_path_ 中的内容加载到给定的 config 对象中。
  7. config 是一个已经定义和分配内存空间的 Protobuf 消息对象。在 C++ 中，Protobuf 消息对象是通过 new 或栈上的分配来创建的，因此已经有了足够的内存来存储消息的数据。
  8. common::GetProtoFromFile(config_file_path_, config) 函数将配置文件的内容解析为 Protobuf 消息，并将数据填充到 config 对象中。这个填充操作不会分配新的内存，而是使用 config 对象已有的内存来存储配置数据。
  */
  template <typename T>
  bool GetProtoConfig(T* config) const {
    return common::GetProtoFromFile(config_file_path_, config);
  }

 protected:
  virtual bool Init() = 0;
  virtual void Clear() { return; }
  const std::string& ConfigFilePath() const { return config_file_path_; }

  void LoadConfigFiles(const ComponentConfig& config) {
    if (!config.config_file_path().empty()) {
      if (config.config_file_path()[0] != '/') {
        config_file_path_ = common::GetAbsolutePath(common::WorkRoot(),
                                                    config.config_file_path());
      } else {
        config_file_path_ = config.config_file_path();
      }
    }

    if (!config.flag_file_path().empty()) {
      std::string flag_file_path = config.flag_file_path();
      if (flag_file_path[0] != '/') {
        flag_file_path =
            common::GetAbsolutePath(common::WorkRoot(), flag_file_path);
      }
      google::SetCommandLineOption("flagfile", flag_file_path.c_str());
    }
  }

  void LoadConfigFiles(const TimerComponentConfig& config) {
    if (!config.config_file_path().empty()) {
      if (config.config_file_path()[0] != '/') {
        config_file_path_ = common::GetAbsolutePath(common::WorkRoot(),
                                                    config.config_file_path());
      } else {
        config_file_path_ = config.config_file_path();
      }
    }

    if (!config.flag_file_path().empty()) {
      std::string flag_file_path = config.flag_file_path();
      if (flag_file_path[0] != '/') {
        flag_file_path =
            common::GetAbsolutePath(common::WorkRoot(), flag_file_path);
      }
      google::SetCommandLineOption("flagfile", flag_file_path.c_str());
    }
  }

  std::atomic<bool> is_shutdown_ = {false};
  std::shared_ptr<Node> node_ = nullptr;
  std::string config_file_path_ = "";
  std::vector<std::shared_ptr<ReaderBase>> readers_;
};

}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_COMPONENT_COMPONENT_BASE_H_
