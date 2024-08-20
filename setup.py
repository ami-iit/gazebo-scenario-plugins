# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

from pathlib import Path

import cmake_build_extension
import setuptools

setuptools.setup(
    ext_modules=[
        cmake_build_extension.CMakeExtension(
            name="actuation_delay",
            source_dir=str(Path(".") / "plugins" / "actuation_delay"),
            install_prefix="gazebo_scenario_plugins",
            cmake_depends_on=["scenario"],
            disable_editable=True,
        ),
        cmake_build_extension.CMakeExtension(
            name="low_pass_target",
            source_dir=str(Path(".") / "plugins" / "low_pass_target"),
            install_prefix="gazebo_scenario_plugins",
            cmake_depends_on=["scenario"],
            disable_editable=True,
        ),
    ],
    cmdclass=dict(build_ext=cmake_build_extension.BuildExtension),
)
