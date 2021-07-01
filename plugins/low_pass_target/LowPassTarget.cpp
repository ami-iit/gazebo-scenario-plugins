#include "LowPassTarget.h"

#include <Iir.h>
#include <ignition/gazebo/components/JointForceCmd.hh>
#include <ignition/gazebo/components/JointPositionReset.hh>
#include <ignition/gazebo/components/JointVelocityReset.hh>
#include <ignition/plugin/Register.hh>
#include <scenario/core/Joint.h>
#include <scenario/gazebo/Joint.h>
#include <scenario/gazebo/Log.h>
#include <scenario/gazebo/components/JointController.h>
#include <scenario/gazebo/helpers.h>
#include <sdf/Element.hh>

#include <stdexcept>
#include <string>

constexpr size_t MaxFilterOrder = 100;

using namespace gsp;

class LowPassTarget::Impl
{
public:
    std::string prefix;
    std::shared_ptr<scenario::gazebo::Joint> joint;

    double target;
    bool ok = false;

    size_t order;
    double sampleRate;
    double cutoffFrequency;
    std::unique_ptr<Iir::Butterworth::LowPass<MaxFilterOrder>> butterworth;

    enum class TargetType
    {
        Invalid,
        Auto,
        Force,
        Position,
        Velocity,
    } targetType;
};

LowPassTarget::LowPassTarget()
    : System()
    , pImpl{std::make_unique<Impl>()}
{}

LowPassTarget::~LowPassTarget() = default;

void LowPassTarget::Configure(const ignition::gazebo::Entity& entity,
                              const std::shared_ptr<const sdf::Element>& sdf,
                              ignition::gazebo::EntityComponentManager& ecm,
                              ignition::gazebo::EventManager& eventMgr)
{
    // Create a joint object that abstracts the ECS
    auto joint = std::make_shared<scenario::gazebo::Joint>();

    // Initialize model
    if (!joint->initialize(entity, &ecm, &eventMgr)) {
        sError << "Failed to initialize the joint object" << std::endl;
        return;
    }

    // Check its validity
    if (joint->valid()) {
        pImpl->joint = joint;
    }
    else {
        sError << "Failed to create a joint from Entity [" << entity << "]"
               << std::endl;
        return;
    }

    // Compute and store the log prefix
    pImpl->prefix = "[" + pImpl->joint->name() + "|"
                    + std::to_string(pImpl->joint->entity()) + "] ";

    if (scenario::gazebo::utils::verboseFromEnvironment()) {
        sDebug << pImpl->prefix << "Received SDF context:" << std::endl;
        std::cout << sdf->ToString("") << std::endl;
    }

    // This is the <plugin> element (with extra options stored in its children)
    sdf::ElementPtr pluginElement = sdf->Clone();

    // Check if it contains all mandatory options stored in <plugin> children
    if (!pluginElement->HasElement("cutoff_frequency")) {
        sError << pImpl->prefix << "Failed to find element 'cutoff_frequency' "
               << "in the plugin's sdf context" << std::endl;
        return;
    }

    // Read the 'cutoff_frequency' option
    if (!pluginElement->Get<double>(
            "cutoff_frequency", pImpl->cutoffFrequency, 0.0)) {
        sError << pImpl->prefix
               << "Failed to get the 'cutoff_frequency' element" << std::endl;
        return;
    }

    // Read the optional 'order' option
    pluginElement->Get<size_t>("order", pImpl->order, 2);

    // Read the optional 'sampling_rate' option
    pluginElement->Get<double>("sample_rate", pImpl->sampleRate, 0.0);

    // TODO: support using a lower sample rate wrt physics
    if (pImpl->sampleRate != 0.0) {
        sWarning << "Specifying a sample rate different is not yet supported"
                 << std::endl;
        pImpl->sampleRate = 0.0;
    }

    // Read the optional 'target_type' option
    std::string targetType;
    pluginElement->Get<std::string>("target_type", targetType, "AUTO");

    if (targetType == "FORCE") {
        pImpl->targetType = Impl::TargetType::Force;
    }
    else if (targetType == "POSITION") {
        pImpl->targetType = Impl::TargetType::Position;
    }
    else if (targetType == "VELOCITY") {
        pImpl->targetType = Impl::TargetType::Velocity;
    }
    else if (targetType == "AUTO") {
        pImpl->targetType = Impl::TargetType::Auto;
    }
    else {
        sWarning << pImpl->prefix << "Target type '" << targetType
                 << "' not recognized. Using AUTO." << std::endl;
        pImpl->targetType = Impl::TargetType::Auto;
    }

    // If a controller (either PIDs or custom) is inserted before this plugin,
    // the plugin will not work due to the order of which plugins are processed.
    if (ecm.EntityHasComponentType(
            ecm.ParentEntity(pImpl->joint->entity()),
            ignition::gazebo::components::JointController::typeId)) {
        sError << pImpl->prefix
               << "This plugin must be inserted before enabling any controller"
               << std::endl;
        return;
    }
}

void LowPassTarget::PreUpdate(const ignition::gazebo::UpdateInfo& info,
                              ignition::gazebo::EntityComponentManager& ecm)
{
    if (info.paused) {
        return;
    }

    // This plugin keeps being called also after the model
    // is removed from the simulation
    try {
        pImpl->joint->type();
    }
    catch (const scenario::gazebo::exceptions::ComponentNotFound&) {
        return;
    }

    if (!(pImpl->joint && pImpl->joint->valid()) || pImpl->joint->dofs() != 1) {
        sError << pImpl->prefix << "The joint is not valid" << std::endl;
        return;
    }

    // Handle the Auto target type by mapping the current joint control mode
    if (pImpl->targetType == Impl::TargetType::Auto) {

        switch (pImpl->joint->controlMode()) {

            case scenario::core::JointControlMode::Force:
                pImpl->targetType = Impl::TargetType::Force;
                break;

            case scenario::core::JointControlMode::Position:
            case scenario::core::JointControlMode::PositionInterpolated:
                pImpl->targetType = Impl::TargetType::Position;
                break;

            case scenario::core::JointControlMode::Velocity:
            case scenario::core::JointControlMode::VelocityFollowerDart:
                pImpl->targetType = Impl::TargetType::Velocity;
                break;

            case scenario::core::JointControlMode::Idle:
            case scenario::core::JointControlMode::Invalid:
                sError << pImpl->prefix << "Failed to automatically detect the "
                       << " joint control type. Disabling the plugin."
                       << std::endl;
                pImpl->targetType = Impl::TargetType::Invalid;
                break;
        }
    }

    if (pImpl->targetType == Impl::TargetType::Invalid) {
        return;
    }

    // Lambda used to allocate the filter the first run
    auto create = [&]() -> bool {
        // Automatically compute the sampling rate from the physics
        if (pImpl->sampleRate == 0.0) {
            const auto dt =
                scenario::gazebo::utils::steadyClockDurationToDouble(info.dt);
            pImpl->sampleRate = 1.0 / dt;
        }

        sDebug << pImpl->prefix << "Creating the filter ("
               << "order=" << pImpl->order << ","
               << "sampleRate=" << pImpl->sampleRate << ","
               << "cutoffFrequency=" << pImpl->cutoffFrequency << ")"
               << std::endl;

        // Create the filter
        auto f = std::make_unique<decltype(pImpl->butterworth)::element_type>();

        // Initialize the parameters
        try {
            f->setup(pImpl->order, pImpl->sampleRate, pImpl->cutoffFrequency);
        }
        catch (const std::exception& e) {
            sError << pImpl->prefix << "Filter parameters are not valid"
                   << std::endl;
            return false;
        }

        // Store the filter
        pImpl->butterworth = std::move(f);
        return true;
    };

    // Lambda used to initialize the filter the first run
    auto initialize = [&](const double target,
                          const double threshold = 0.01,
                          const size_t convergedSteps = 50) {
        sDebug << pImpl->prefix << " Initializing the filter" << std::endl;

        for (size_t i = 0; i < convergedSteps; ++i) {
            if (std::abs(pImpl->butterworth->filter(target) - target)
                > threshold * std::abs(target)) {
                i = 0;
            }
        }
    };

    // If commands / reset components are found after the filter has been
    // created, reset the filter
    if (pImpl->butterworth) {
        switch (pImpl->targetType) {

            case Impl::TargetType::Force:
                if (ecm.EntityHasComponentType(
                        pImpl->joint->entity(),
                        ignition::gazebo::components::JointForceCmd::typeId)) {
                    pImpl->butterworth->reset();
                    const double initialState =
                        scenario::gazebo::utils::getExistingComponentData<
                            ignition::gazebo::components::JointForceCmd>(
                            &ecm, pImpl->joint->entity())[0];
                    initialize(initialState);
                }
                break;

            case Impl::TargetType::Position:
                if (ecm.EntityHasComponentType(
                        pImpl->joint->entity(),
                        ignition::gazebo::components::JointPositionReset::
                            typeId)) {
                    pImpl->butterworth->reset();
                    const double initialState =
                        scenario::gazebo::utils::getExistingComponentData<
                            ignition::gazebo::components::JointPositionReset>(
                            &ecm, pImpl->joint->entity())[0];
                    initialize(initialState);
                }
                break;
            case Impl::TargetType::Velocity:
                if (ecm.EntityHasComponentType(
                        pImpl->joint->entity(),
                        ignition::gazebo::components::JointVelocityReset::
                            typeId)) {
                    pImpl->butterworth->reset();
                    const double initialState =
                        scenario::gazebo::utils::getExistingComponentData<
                            ignition::gazebo::components::JointVelocityReset>(
                            &ecm, pImpl->joint->entity())[0];
                    initialize(initialState);
                }
                break;
            case Impl::TargetType::Auto:
            case Impl::TargetType::Invalid:
                break;
        }
    }

    pImpl->ok = false;

    // Filter the data contained in the component of the active target
    switch (pImpl->targetType) {

        case Impl::TargetType::Force: {
            pImpl->target = pImpl->joint->generalizedForceTarget();

            if (!pImpl->butterworth) {
                if (!create()) {
                    return;
                }
                initialize(pImpl->joint->generalizedForce());
            }

            const auto filteredTarget =
                pImpl->butterworth->filter(pImpl->target);

            pImpl->ok =
                !std::isnan(filteredTarget)
                && pImpl->joint->setGeneralizedForceTarget(filteredTarget);
            break;
        }

        case Impl::TargetType::Position: {
            pImpl->target = pImpl->joint->positionTarget();

            if (!pImpl->butterworth) {
                if (!create()) {
                    return;
                }
                initialize(pImpl->joint->position());
            }

            const auto filteredTarget =
                pImpl->butterworth->filter(pImpl->target);

            pImpl->ok = !std::isnan(filteredTarget)
                        && pImpl->joint->setPositionTarget(filteredTarget);
            break;
        }

        case Impl::TargetType::Velocity: {
            pImpl->target = pImpl->joint->velocityTarget();

            if (!pImpl->butterworth) {
                if (!create()) {
                    return;
                }
                initialize(pImpl->joint->velocity());
            }

            const auto filteredTarget =
                pImpl->butterworth->filter(pImpl->target);

            pImpl->ok = !std::isnan(filteredTarget)
                        && pImpl->joint->setVelocityTarget(filteredTarget);
            break;
        }

        case Impl::TargetType::Auto:
        case Impl::TargetType::Invalid: {
            sError << pImpl->prefix
                   << "Failed to automatically detect target type" << std::endl;
            return;
        }
    }

    if (!pImpl->ok) {
        sError << pImpl->prefix << "Failed to set the filtered target"
               << std::endl;
        return;
    }
}

void LowPassTarget::PostUpdate(
    const ignition::gazebo::UpdateInfo& info,
    const ignition::gazebo::EntityComponentManager& /*ecm*/)
{
    if (info.paused) {
        return;
    }

    // This plugin keeps being called also after the model
    // is removed from the simulation
    try {
        pImpl->joint->type();
    }
    catch (const scenario::gazebo::exceptions::ComponentNotFound&) {
        return;
    }

    if (pImpl->ok) {
        // Restore the target
        switch (pImpl->targetType) {

            case Impl::TargetType::Force:
                pImpl->joint->setGeneralizedForceTarget(pImpl->target);
                break;

            case Impl::TargetType::Position:
                pImpl->joint->setPositionTarget(pImpl->target);
                break;

            case Impl::TargetType::Velocity:
                pImpl->joint->setVelocityTarget(pImpl->target);
                break;

            case Impl::TargetType::Auto:
            case Impl::TargetType::Invalid: {
                return;
            }
        }
    }
}

IGNITION_ADD_PLUGIN(gsp::LowPassTarget,
                    gsp::LowPassTarget::System,
                    gsp::LowPassTarget::ISystemConfigure,
                    gsp::LowPassTarget::ISystemPreUpdate,
                    gsp::LowPassTarget::ISystemPostUpdate)
