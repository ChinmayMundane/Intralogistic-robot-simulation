/*
 * Copyright (C) 2025 Your Name
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef GZ_SIM_SYSTEMS_DUMMY_SENSOR_HH_
#define GZ_SIM_SYSTEMS_DUMMY_SENSOR_HH_

#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  // Forward declaration
  class DummySensorPrivate;

  /// \brief A simple sensor plugin that publishes "Hello World" to a gz-transport topic
  /// on load, which can be bridged to a ROS 2 topic.
  class DummySensor
      : public System,
        public ISystemConfigure,
        public ISystemPostUpdate
  {
    public: DummySensor();
    public: ~DummySensor() override = default;

    public: void Configure(const Entity &_entity,
                          const std::shared_ptr<const sdf::Element> &_sdf,
                          EntityComponentManager &_ecm,
                          EventManager &_eventMgr) override;

    public: void PostUpdate(const UpdateInfo &_info,
                           const EntityComponentManager &_ecm) override;

    private: std::unique_ptr<DummySensorPrivate> dataPtr;
  };
}
}
}
}

#endif
