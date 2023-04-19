# redesigned-journey

[![pre-commit.ci status](https://results.pre-commit.ci/badge/github/prachandabhanu/redesigned-journey/main.svg)](https://results.pre-commit.ci/latest/github/prachandabhanu/redesigned-journey/main)

## Code Quality
### ament_clang_format
Reference: [here](https://www.youtube.com/watch?v=2gIyu09UEC8)
1. Install `sudo apt install ros-foxy-ament-clang-formate`
2. Add in package.xml: `<test_depend>ament_cmake_clang_formate</test_depend>`
3. `colcon build`
4. `colcon test`
5. `colcon test-result --verbose`
6. `cd src/<package>`
7. `ament_clang_format`
8. `ament_clang_format --reformat`
9. `ament_clang_formate --config <path> --reformat`

### (Advanced) [Precommit hook](https://pre-commit.com/)
1. Install: `pip install pre-commit`
2. Check version: `pre-commit --version`
3. Setup workspace: `pre-commit install`
4. Run against all files: `pre-commit run --all-files` or `pre-commit run --all-files --hook-stage manual`

### Alternatively
Reference [here](https://github.com/PointCloudLibrary/pcl/blob/master/.dev/format.sh)
1. `./format.sh /usr/bin/clang-format` or `./format.sh $(which clang-format)`

## ROS2 Debug
Read the post [here](https://answers.ros.org/question/267261/how-can-i-run-ros2-nodes-in-a-debugger-eg-gdb/)
1. Compile your C++ package with the symbols,

`colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo`

2. Run the GDB Server in a localhost:port

`ros2 run --prefix 'gdbserver localhost:3000' azure_kinect_bm_pkg azure_listener`

3. Open VSCode on your workspace, open the debug section (side bar) and create new launch.json configuration file for debugging. Configure as follow:

```
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "C++ Debugger",
            "request": "launch",
            "type": "cppdbg",
            "miDebuggerServerAddress": "localhost:3000",
            "cwd": "/",
            "program": "/home/vishnu/ros_ws/colcon_bm_ws/install/azure_kinect_bm_pkg/lib/azure_kinect_bm_pkg/azure_listener"
        }
    ]
}
```

4. Run the debugger and use VSCode to debug your code.
5. For more detailed explanation read [here](https://gist.github.com/prachandabhanu/03f0f91c37d3e815f5d1a27983a99a78)

## Code Quality
### ament_clang_format
Reference: [here](https://www.youtube.com/watch?v=2gIyu09UEC8)
1. Install `sudo apt install ros-foxy-ament-clang-formate`
2. Add in package.xml: `<test_depend>ament_cmake_clang_formate</test_depend>`
3. `colcon build`
4. `colcon test`
5. `colcon test-result --verbose`
6. `cd src/<package>`
7. `ament_clang_format`
8. `ament_clang_format --reformat`
9. `ament_clang_formate --config <path> --reformat`

### (Advanced) [Precommit hook](https://pre-commit.com/)
1. Install: `pip install pre-commit`
2. Check version: `pre-commit --version`
3. Setup workspace: `pre-commit install`
4. Run against all files: `pre-commit run --all-files` or `pre-commit run --all-files --hook-stage manual`

### Alternatively
Reference [here](https://github.com/PointCloudLibrary/pcl/blob/master/.dev/format.sh)
1. `./format.sh /usr/bin/clang-format` or `./format.sh $(which clang-format)`

## Documentation
1. `pip install sphinx`
2. `cd /path/to/project`
3. `mkdir docs`
4. `cd docs && sphinx-quickstart`
5. `make html`
6. `sudo apt-get install texlive-full texmaker`
7. `make latexpdf`

## Libraries Links
1. PCL link library reference [here](https://github.com/ros-perception/perception_pcl/tree/ros2)
