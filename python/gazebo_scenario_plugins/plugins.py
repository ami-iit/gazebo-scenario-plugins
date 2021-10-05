import enum
from dataclasses import dataclass, field

from . import plugin


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


class LowPassTargetType(enum.Enum):

    AUTO = enum.auto()
    FORCE = enum.auto()
    POSITION = enum.auto()
    VELOCITY = enum.auto()


@dataclass
class LowPassTarget(plugin.Plugin):
    """
    Filter the joint targets (references) with a configurable Butterworth low-pass filter.

    Warning:
        The plugin must be added to the model before the joint switches to Position
        control mode.
    """

    cutoff_frequency: float

    order: int = 2
    sample_rate: float = 0.0
    target_type: LowPassTargetType = LowPassTargetType.AUTO

    # Private attributes
    _plugin_name: str = field(init=False, repr=False, default="LowPassTarget")
    _plugin_class: str = field(init=False, repr=False, default="gsp::LowPassTarget")

    def to_xml(self) -> str:

        return f"""
        <sdf version='{plugin.SDF_VERSION}'>
            <order>{self.order}</order>
            <sample_rate>{self.sample_rate}</sample_rate>
            <target_type>{self.target_type.name}</target_type>
            <cutoff_frequency>{self.cutoff_frequency}</cutoff_frequency>
        </sdf>
        """
