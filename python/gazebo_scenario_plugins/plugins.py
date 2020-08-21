from dataclasses import dataclass, field
from gazebo_scenario_plugins import plugin


@dataclass
class ActuationDelay(plugin.Plugin):
    """
    Delay applied targets for a given number of physics steps.

    Note:
        Only Position targets are currently supported.

    Warning:
        The plugin must be added to the model before any joint switches to Position
        control mode.
    """

    delay: int

    # Private attributes
    _plugin_name: str = field(init=False, repr=False, default="ActuationDelay")
    _plugin_class: str = field(init=False, repr=False, default="gsp::ActuationDelay")

    def to_xml(self) -> str:

        return f"""
        <sdf version='{plugin.SDF_VERSION}'>
            <delay>{self.delay}</delay>
        </sdf>
        """
