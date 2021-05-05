# LowPassTarget

Filter the joint targets (references) with a configurable Butterworth low-pass filter.

:warning: This plugin must be inserted in the model before enabling the controller (either the default PID or a custom one). 
In the case of PIDs, they are enabled by ScenarIO when either `Joint::setControlMode` or `Model::setJointControlMode` are called.

## Metadata

| `lib_name`      | `class_name`         |
| --------------- | -------------------- |
| `LowPassTarget` | `gsp::LowPassTarget` |

## Context

| Name               | Optional | Type                                                | Default | Description                                                  |
| ------------------ | -------- | --------------------------------------------------- | ------- | ------------------------------------------------------------ |
| `cutoff_frequency` | No       | float                                               | -       | The cutoff frequency of the Butterworth filter. It cannot be bigger than half of `sample_rate`. |
| `order`            | Yes      | integer                                             | 2       | The order of the Butterworth filter                          |
| `sample_rate`      | Yes      | float                                               | 0       | The sample rate of the filter. If 0, it defaults to the physics rate of the server. |
| `target_type`      | Yes      | string<br />`AUTO`, `FORCE`, `POSITION`, `VELOCITY` | `AUTO`  | The type of target to filter. If `AUTO`, the type is inferred from the joint control mode. |

## SDF

```xml
<plugin filename="LowPassTarget" name="gsp::LowPassTarget">
    <!-- Context -->
    <cutoff_frequency>5.0</cutoff_frequency>
    <!-- Optional context -->
    <order>2</order>
    <sample_rate>100.0</sample_rate>
    <target_type>AUTO</target_type>
</plugin>
```

## Python

```python
from gazebo_scenario_plugins import plugins
from scenario import gazebo as scenario_gazebo

# [...] Create the simulation and insert the model 'my_model'

# Create the plugin context
low_pass_target = plugins.LowPassTarget(
    cutoff_frequency=5.0, 
    order=2,
    sample_rate=100.0,
    target_type=plugins.LowPassTargetType.AUTO,
)

# Get the joint 'my_joint_name' from the model
my_joint = my_model.get_link("my_joint_name")

# Insert the plugin to the joint
ok = scenario_gazebo.insert_plugin_to_gazebo_entity(
    my_joint.to_gazebo(), *low_pass_target.args())
assert ok, "Failed to insert the plugin"
```
