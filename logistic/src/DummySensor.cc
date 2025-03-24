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

#include "logistic/DummySensor.hh"

#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/Model.hh>
#include <gz/msgs/stringmsg.pb.h>

using namespace gz;
using namespace sim;
using namespace systems;

class gz::sim::systems::DummySensorPrivate
{
  public: Model model{kNullEntity};
  public: std::string modelName;
  public: gz::transport::Node node;
  public: gz::transport::Node::Publisher pub;
  public: bool configured{false};
  public: bool messageSent{false};
};

//////////////////////////////////////////////////
DummySensor::DummySensor()
  : System(), dataPtr(std::make_unique<DummySensorPrivate>())
{
}

//////////////////////////////////////////////////
void DummySensor::Configure(const Entity &_entity,
                            const std::shared_ptr<const sdf::Element> &_sdf,
                            EntityComponentManager &_ecm,
                            EventManager &/*_eventMgr*/)
{
  this->dataPtr->model = Model(_entity);
  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "DummySensor plugin should be attached to a model entity. "
          << "Failed to initialize." << std::endl;
    return;
  }

  this->dataPtr->modelName = this->dataPtr->model.Name(_ecm);

  // Initialize the publisher with a default topic name or from SDF
  std::string topic = "/dummy_sensor/output";
  if (_sdf->HasElement("topic"))
  {
    topic = _sdf->Get<std::string>("topic");
  }

  this->dataPtr->pub = this->dataPtr->node.Advertise<gz::msgs::StringMsg>(topic);
  if (!this->dataPtr->pub)
  {
    gzerr << "Failed to advertise topic [" << topic << "] for DummySensor." << std::endl;
    return;
  }

  this->dataPtr->configured = true;
  gzmsg << "DummySensor configured for model [" << this->dataPtr->modelName << "]" << std::endl;
}

//////////////////////////////////////////////////
void DummySensor::PostUpdate(const UpdateInfo &_info,
                             const EntityComponentManager &_ecm)
{
  GZ_PROFILE("DummySensor::PostUpdate");

  if (_info.paused || !this->dataPtr->configured)
    return;

  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzwarn << "Model [" << this->dataPtr->modelName << "] no longer valid. "
           << "Disabling DummySensor plugin." << std::endl;
    this->dataPtr->configured = false;
    return;
  }

  // Publish "Hello World" only once on the first update
  if (!this->dataPtr->messageSent)
  {
    gz::msgs::StringMsg msg;
    msg.set_data("Hello World");
    this->dataPtr->pub.Publish(msg);
    gzmsg << "DummySensor on [" << this->dataPtr->modelName << "] published 'Hello World' at "
          << std::chrono::duration_cast<std::chrono::duration<double>>(_info.simTime).count()
          << "s." << std::endl;
    this->dataPtr->messageSent = true;
  }
}

GZ_ADD_PLUGIN(
  DummySensor,
  System,
  DummySensor::ISystemConfigure,
  DummySensor::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(DummySensor, "gz::sim::systems::DummySensor")

