import os
from pathlib import Path


def setup_environment() -> None:
    """
    Configures the environment extending the search path of Ignition Gazebo with the
    folder containing the plugins of this repository.
    """

    plugin_dir = Path(os.path.dirname(__file__)) / "lib"

    if "IGN_GAZEBO_SYSTEM_PLUGIN_PATH" in os.environ:
        os.environ["IGN_GAZEBO_SYSTEM_PLUGIN_PATH"] += f":{plugin_dir}"
    else:
        os.environ["IGN_GAZEBO_SYSTEM_PLUGIN_PATH"] = f"{plugin_dir}"


setup_environment()
