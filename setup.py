from pathlib import Path
from setuptools import setup, find_packages
from cmake_build_extension import BuildExtension, CMakeExtension

# Read the contents of the README file
with open(Path(__file__).parent.absolute() / "README.md", encoding="utf-8") as f:
    long_description = f.read()

setup(
    name="gazebo-scenario-plugins",
    author="Diego Ferigo",
    author_email="diego.ferigo@iit.it",
    description="Plugins for the Ignition Gazebo simulator implemented with ScenarI/O",
    long_description=long_description,
    long_description_content_type='text/markdown',
    url="https://github.com/dic-iit/gazebo-scenario-plugins",
    keywords="gazebo ignition simulation robotics plugin",
    license="LGPL",
    platforms=["linux"],
    classifiers=[
        "Development Status :: 4 - Beta",
        "Operating System :: POSIX :: Linux",
        "Topic :: Games/Entertainment :: Simulation",
        "Framework :: Robot Framework",
        "Intended Audience :: Developers",
        "Intended Audience :: Science/Research",
        "Programming Language :: C++",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3 :: Only",
        "License :: OSI Approved :: GNU Lesser General Public License v2 or later (LGPLv2+)",
    ],
    use_scm_version=dict(local_scheme="dirty-tag"),
    python_requires='>=3.8',
    install_requires=[
        "gym-ignition",
    ],
    setup_requires=[
        "cmake>=3.18.2",
        "setuptools_scm",
        "ninja",
        "gym-ignition",
        "cmake-build-extension",
    ],
    packages=find_packages("python"),
    package_dir={'': "python"},
    ext_modules=[CMakeExtension(name="actuation_delay",
                                source_dir=str(Path(".")/"plugins"/"actuation_delay"),
                                install_prefix="gazebo_scenario_plugins",
                                cmake_depends_on=["scenario"]),
                 ],
    cmdclass=dict(build_ext=BuildExtension),
    zip_safe=False,
)
