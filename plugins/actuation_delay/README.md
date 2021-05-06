# ActuationDelay

Delay the position targets (references) of all joints belonging to a model for a configurable number of physics steps.

:warning: This plugin must be inserted in the model before enabling the controller (either the default PID or a custom one). 
In the case of PIDs, they are enabled by ScenarIO when either `Joint::setControlMode` or `Model::setJointControlMode` are called.

## Metadata

| `lib_name`       | `class_name`          |
| ---------------- | --------------------- |
| `ActuationDelay` | `gsp::ActuationDelay` |

## Context

| Name    | Optional | Type | Default | Description                                              |
| ------- | -------- | ---- | ------- | -------------------------------------------------------- |
| `delay` | No       | int  | -       | The number of physics step to delay all joint references |

## SDF

```xml
<plugin filename="ActuationDelay" name="gsp::ActuationDelay">
    <!-- Context -->
    <delay>10</delay>
</plugin>
```

## Python

```python
from gazebo_scenario_plugins import plugins
from scenario import gazebo as scenario_gazebo

# [...] Create the simulation and insert the model 'my_model'

# Create the plugin context
actuation_delay = plugins.ActuationDelay(delay=10)

# Insert the plugin to the model
ok = scenario_gazebo.insert_plugin_to_gazebo_entity(
    my_model.to_gazebo(), *actuation_delay.args())
assert ok, "Failed to insert the plugin"
```
