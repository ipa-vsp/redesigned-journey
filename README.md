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
