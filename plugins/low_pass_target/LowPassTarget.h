#ifndef GAZEBO_SCENARIO_PLUGINS_LOW_PASS_TARGET_H
#define GAZEBO_SCENARIO_PLUGINS_LOW_PASS_TARGET_H

#include <ignition/gazebo/Entity.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/EventManager.hh>
#include <ignition/gazebo/System.hh>

#include <memory>

namespace gsp {
    class LowPassTarget;
} // namespace gsp

class gsp::LowPassTarget final
    : public ignition::gazebo::System
    , public ignition::gazebo::ISystemConfigure
    , public ignition::gazebo::ISystemPreUpdate
    , public ignition::gazebo::ISystemPostUpdate
{
public:
    LowPassTarget();
    ~LowPassTarget() override;

    void Configure(const ignition::gazebo::Entity& entity,
                   const std::shared_ptr<const sdf::Element>& sdf,
                   ignition::gazebo::EntityComponentManager& ecm,
                   ignition::gazebo::EventManager& eventMgr) override;

    void PreUpdate(const ignition::gazebo::UpdateInfo& info,
                   ignition::gazebo::EntityComponentManager& ecm) override;

    void PostUpdate( //
        const ignition::gazebo::UpdateInfo& info,
        const ignition::gazebo::EntityComponentManager& ecm) override;

private:
    class Impl;
    std::unique_ptr<Impl> pImpl = nullptr;
};

#endif // GAZEBO_SCENARIO_PLUGINS_LOW_PASS_TARGET_H
