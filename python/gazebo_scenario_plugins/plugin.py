import abc
from dataclasses import dataclass, field
from typing import Tuple

# Default SDF version used in the serialized XML context
SDF_VERSION = 1.7

# Read the following for more information about dataclasses internals:
# https://florimond.dev/blog/articles/2018/10/reconciling-dataclasses-and-properties-in-python/


@dataclass
class Plugin(abc.ABC):
    """
    Base class of all World and Model plugins.

    The Plugin abstract class provides boilerplate code that simplifies and streamlines
    the definition of helper classes that insert Ignition Gazebo plugins to either World
    or Model objects.

    Classes that inherit from Plugin have to provide only the following information:

    1) All the properties of the plugin in the form or dataclass fields
    2) The specification of the plugin (plugin name and plugin class)
    3) The serialized XML context

    Example:

    .. code-block:: python

        plugin = MyPlugin(my_property=42)
        model = MyModel(world=my_world)

        model.insert_model_plugin(*plugin.args())
    """

    _plugin_name: str = field(init=False, repr=False)
    _plugin_class: str = field(init=False, repr=False)

    @abc.abstractmethod
    def to_xml(self) -> str:
        """
        Get the XML plugin content.

        Returns:
            The XML plugin content.
        """

    def args(self) -> Tuple[str, str, str]:
        """
        Get the arguments passed to the ScenarI/O methods used to insert plugins.

        Returns:
            A tuple with the args required to insert the plugin.
        """
        return str(self._plugin_name), str(self._plugin_class), self.to_xml()
