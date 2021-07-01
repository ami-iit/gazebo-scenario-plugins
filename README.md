
:warning: This repository is still experimental, expect bugs and breaking changes :warning:

# gazebo-scenario-plugins

[![Version](https://img.shields.io/pypi/v/gazebo-scenario-plugins.svg)](https://pypi.org/project/gazebo-scenario-plugins/)
[![Python versions](https://img.shields.io/pypi/pyversions/gazebo-scenario-plugins.svg)](https://pypi.org/project/gazebo-scenario-plugins/)
[![Status](https://img.shields.io/pypi/status/gazebo-scenario-plugins.svg)](https://pypi.org/project/gazebo-scenario-plugins/)
[![Format](https://img.shields.io/pypi/format/gazebo-scenario-plugins.svg)](https://pypi.org/project/gazebo-scenario-plugins/)
[![License](https://img.shields.io/pypi/l/gazebo-scenario-plugins.svg)](https://pypi.org/project/gazebo-scenario-plugins/)
[![CI/CD](https://github.com/dic-iit/gazebo-scenario-plugins/actions/workflows/ci_cd.yml/badge.svg)](https://github.com/dic-iit/gazebo-scenario-plugins/actions/workflows/ci_cd.yml)

Ignition Gazebo Plugins implemented with [ScenarIO](https://robotology.github.io/gym-ignition/master/motivations/why_gym_ignition.html).

## Plugins

| Name                                        | Description                                                  |
| ------------------------------------------- | ------------------------------------------------------------ |
| [`ActuationDelay`](plugins/actuation_delay) | Simulate actuation delay inserting the targets (joint references) into a FIFO buffer. |
| [`LowPassTarget`](plugins/low_pass_target)  | Filter the joint targets (references) with a configurable Butterworth low-pass filter. |

## Dependencies

`gazebo-scenario-plugins` expects to find installed and configured the following dependencies:

- [`ignitionrobotics/ign-gazebo`](https://github.com/ignitionrobotics/ign-gazebo)
- [`robotology/gym-ignition`](https://github.com/robotology/gym-ignition)

Visit the `gym-ignition` repository to check what Ignition distribution is currently supported.

## Installation

Install only the C++ resources with:

```bash
git clone https://github.com/dic-iit/gazebo-scenario-plugins
cd gazebo-scenario-plugins
cmake -S . -B build
cmake --build build/ --target install
```

or the complete C++ and Python resources with:

```bash
pip3 install gazebo-scenario-plugins
```

## System configuration

If you have installed only the C++ plugins, make sure to add the install prefix to the `IGN_GAZEBO_SYSTEM_PLUGIN_PATH` environment variable. This way, Ignition Gazebo will automatically find the plugins without the need to specify the absolute path to their library.

If, instead, you rely to the `gazebo-scenario-plugins` Python package, it will automatically configure the environment right after it is `import`ed into your project.

## Usage

The plugins stored in this repository can be inserted in a simulated world either by including them in the SDF file, or by loading them programmatically from either C++ or Python using the ScenarIO APIs.

The three pieces of information you need to know for loading plugins are the following:

1. **`lib_name`**: the OS-agnostic name of the plugin library. E.g., if the plugin file is `libActuationDelay.so`, the library name is `ActuationDelay`.
1. **`class_name`**: the name of the class implementing the plugin. E.g. `gsp::ActuationDelay`.
1. **Plugin context**: the configuration options of the plugin.

### From the sdf

Select the plugin you want to add and use the following template to add it to the SDF element they correspond (world, model, joint, ...).


```xml
<plugin filename="<lib_name>" name="class_name">
    <!-- Optional XML plugin context -->
    <optional_config1>bar</optional_config1>
    <optional_config2>42</optional_config2>
    <optional_config3>l1 l2 l3 l4</optional_config3>
</plugin>
```

### From C++

You can use from your C++ code any of the following methods:

- `scenario::gazebo::World::insertWorldPlugin`
- `scenario::gazebo::Model::insertModelPlugin`
- `scenario::gazebo::utils::insertPluginToGazeboEntity`

Refer to the [ScenarIO  C++ APIs documentation](https://robotology.github.io/gym-ignition/master/breathe/gazebo.html) for more details.

### From Python

The low-level APIs to load plugins from your Python code match almost 1:1 the C++ usage:

- `scenario.bindings.gazebo.World.insert_world_plugin`
- `scenario.bindings.gazebo.Model.insert_model_plugin`
- `scenario.bindings.gazebo.insert_plugin_to_gazebo_entity`

Refer to the [ScenarIO Python APIs documentation](https://robotology.github.io/gym-ignition/master/apidoc/scenario/scenario.bindings.html) for more details.

In addition to the low-level APIs, this repository provides a `gazebo_scenario_plugins` Python package that includes helper classes that simplify building the plugin context. Refer to the [`gazebo_scenario_plugins.plugins`](python/gazebo_scenario_plugins/plugins) module for more details.

## Contributing

Pull requests are welcome.

For major changes, please open an issue first to discuss what you would like to change.

## License

[LGPL v2.1](https://choosealicense.com/licenses/lgpl-2.1/) or any later version.

