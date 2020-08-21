#include "ActuationDelay.h"

#include <ignition/gazebo/components/Joint.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/plugin/Register.hh>
#include <scenario/gazebo/Joint.h>
#include <scenario/gazebo/Log.h>
#include <scenario/gazebo/Model.h>
#include <scenario/gazebo/components/JointPositionTarget.h>
#include <scenario/gazebo/helpers.h>
#include <sdf/Element.hh>

#include <deque>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

using namespace gsp;

class ActuationDelay::Impl
{
public:
    size_t nrOfDelaySteps;
    ignition::gazebo::Entity modelEntity;
    std::shared_ptr<scenario::gazebo::Model> model;

    using JointName = std::string;
    using PositionData = std::vector<double>;
    std::unordered_map<JointName, PositionData> q0;
    std::unordered_map<JointName, PositionData> currentReferences;
    std::unordered_map<JointName, std::deque<PositionData>> queues;
};

ActuationDelay::ActuationDelay()
    : System()
    , pImpl{std::make_unique<Impl>()}
{}

ActuationDelay::~ActuationDelay() = default;

void ActuationDelay::Configure(const ignition::gazebo::Entity& entity,
                               const std::shared_ptr<const sdf::Element>& sdf,
                               ignition::gazebo::EntityComponentManager& ecm,
                               ignition::gazebo::EventManager& eventMgr)
{
    // Store the model entity
    pImpl->modelEntity = entity;

    // Create a model object that abstracts the ECS
    auto model = std::make_shared<scenario::gazebo::Model>();

    // Initialize model
    if (!model->initialize(entity, &ecm, &eventMgr)) {
        sError << "Failed to initialize model for controller" << std::endl;
        return;
    }

    // Check its validity
    if (model->valid()) {
        pImpl->model = model;
    }
    else {
        sError << "Failed to create a model from Entity [" << entity << "]"
               << std::endl;
        return;
    }

    if (scenario::gazebo::utils::verboseFromEnvironment()) {
        sDebug << "Received SDF context:" << std::endl;
        std::cout << sdf->ToString("") << std::endl;
    }

    // This is the <plugin> element (with extra options stored in its children)
    sdf::ElementPtr pluginElement = sdf->Clone();

    // Check if it contains extra options stored in <plugin> children
    if (!pluginElement->HasElement("delay")) {
        sError << "Failed to find element 'delay' in the plugin's sdf context";
        return;
    }

    if (!pluginElement->Get<size_t>("delay", pImpl->nrOfDelaySteps, 0)) {
        sError << "Failed to get the 'delay' element";
        return;
    }

    sDebug << "Enabled the delay of joint position targets for "
           << pImpl->nrOfDelaySteps << " physics steps" << std::endl;

    // Create the queues for all joints. Then, only the joints that use
    // JointForceCmd are actually processed.
    for (const auto& joint : pImpl->model->joints()) {
        pImpl->queues[joint->name()] = {};
    }

    // Get the initial positions of the joints, they will be the applied targets
    // while waiting the queue to get filled
    for (const auto& joint : pImpl->model->joints()) {
        pImpl->q0[joint->name()] = joint->jointPosition();
    }
}

void ActuationDelay::PreUpdate(const ignition::gazebo::UpdateInfo& info,
                               ignition::gazebo::EntityComponentManager& ecm)
{
    if (pImpl->nrOfDelaySteps == 0) {
        return;
    }

    if (info.paused) {
        return;
    }

    if (!(pImpl->model && pImpl->model->valid())) {
        return;
    }

    // This plugin keeps being called also after the model was removed
    try {
        pImpl->model->controllerPeriod();
    }
    catch (const scenario::gazebo::exceptions::ComponentNotFound&) {
        return;
    }

    ecm.Each<ignition::gazebo::components::Joint,
             ignition::gazebo::components::Name,
             ignition::gazebo::components::JointPositionTarget,
             ignition::gazebo::components::ParentEntity>(
        [&](const ignition::gazebo::Entity& /*entity*/,
            ignition::gazebo::components::Joint* /*jointComponent*/,
            ignition::gazebo::components::Name* nameComponent,
            ignition::gazebo::components::JointPositionTarget*
                jointPosTargetComponent,
            ignition::gazebo::components::ParentEntity* parentEntityComponent)
            -> bool {
            if (parentEntityComponent->Data() != pImpl->modelEntity) {
                return true;
            }

            // Get joint name and position target from the ECS
            const std::string& name = nameComponent->Data();
            const std::vector<double>& positionTarget =
                jointPosTargetComponent->Data();

            auto joint = pImpl->model->getJoint(name);
            if (joint->dofs() != positionTarget.size()) {
                // Skip joints with 0 or wrong position targets
                return true;
            }

            // Backup the current target, it is restored in the PostUpdate
            pImpl->currentReferences[name] = positionTarget;

            // Add a new element in back of the queue
            auto& queue = pImpl->queues[name];
            queue.push_back(positionTarget);

            // Wait to fill the queue
            if (queue.size() < pImpl->nrOfDelaySteps) {

                // Apply the initial position
                if (!joint->setJointPositionTarget(pImpl->q0.at(name))) {
                    sError << "Failed to set the initial target of joint "
                           << name << std::endl;
                }

                return true;
            }

            // Set the delayed target that will be processed by the
            // PID controller
            if (!joint->setJointPositionTarget(queue.front())) {
                sError << "Failed to set the delayed position target of joint "
                       << name << std::endl;
            }

            // Remove the front element
            queue.pop_front();
            return true;
        });
}

void ActuationDelay::PostUpdate(
    const ignition::gazebo::UpdateInfo& /*info*/,
    const ignition::gazebo::EntityComponentManager& ecm)
{
    // We have to work around the APIs here
    auto& _ecm = const_cast<ignition::gazebo::EntityComponentManager&>(ecm);

    _ecm.Each<ignition::gazebo::components::Joint,
              ignition::gazebo::components::Name,
              ignition::gazebo::components::JointPositionTarget,
              ignition::gazebo::components::ParentEntity>(
        [&](const ignition::gazebo::Entity& entity,
            ignition::gazebo::components::Joint* /*jointComponent*/,
            ignition::gazebo::components::Name* nameComponent,
            ignition::gazebo::components::JointPositionTarget*
            /*jointPosTargetComponent*/,
            ignition::gazebo::components::ParentEntity* parentEntityComponent)
            -> bool {
            if (parentEntityComponent->Data() != pImpl->modelEntity) {
                return true;
            }

            // Get the name of the joint
            const std::string& name = nameComponent->Data();

            // Restore the original reference in the ECM
            scenario::gazebo::utils::setExistingComponentData<
                ignition::gazebo::components::JointPositionTarget>(
                &_ecm, entity, pImpl->currentReferences[name]);

            // TODO: clean something if the joint changes control mode?
            return true;
        });
}

IGNITION_ADD_PLUGIN(gsp::ActuationDelay,
                    gsp::ActuationDelay::System,
                    gsp::ActuationDelay::ISystemConfigure,
                    gsp::ActuationDelay::ISystemPreUpdate,
                    gsp::ActuationDelay::ISystemPostUpdate)
