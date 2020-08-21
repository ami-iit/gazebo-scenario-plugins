#ifndef GAZEBO_SCENARIO_PLUGINS_ACTUATIONDELAY_H
#define GAZEBO_SCENARIO_PLUGINS_ACTUATIONDELAY_H

#include <ignition/gazebo/Entity.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/EventManager.hh>
#include <ignition/gazebo/System.hh>

#include <memory>

namespace gsp {
    class ActuationDelay;
} // namespace gsp

class gsp::ActuationDelay final
    : public ignition::gazebo::System
    , public ignition::gazebo::ISystemConfigure
    , public ignition::gazebo::ISystemPreUpdate
    , public ignition::gazebo::ISystemPostUpdate
{
public:
    ActuationDelay();
    ~ActuationDelay() override;

    void Configure(const ignition::gazebo::Entity& entity,
                   const std::shared_ptr<const sdf::Element>& sdf,
                   ignition::gazebo::EntityComponentManager& ecm,
                   ignition::gazebo::EventManager& eventMgr) override;

    void PreUpdate(const ignition::gazebo::UpdateInfo& info,
                   ignition::gazebo::EntityComponentManager& ecm) override;

    void
    PostUpdate(const ignition::gazebo::UpdateInfo& info,
               const ignition::gazebo::EntityComponentManager& ecm) override;

private:
    class Impl;
    std::unique_ptr<Impl> pImpl = nullptr;
};

#endif // GAZEBO_SCENARIO_PLUGINS_ACTUATIONDELAY_H
