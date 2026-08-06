# Robot URDF Kinematics

Standalone C++17 URDF reader and forward-kinematics tool for serial industrial robot arms.

## Dependencies

- CMake 3.16 or newer
- C++17 compiler
- Eigen3
- tinyxml2

On Ubuntu:

```bash
sudo apt install cmake g++ libeigen3-dev libtinyxml2-dev pkg-config
```

## Build

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

## Examples

FANUC M-710iC50/M-710iC70:

```bash
./build/robot_kinematics --urdf "/tmp/read_robot_zips/m710/M-710iC50 M-710iC70/urdf/M-710iC50 M-710iC70.STEP.SLDASM.urdf" --joints 0 0 0 0 0 0
```

KUKA KR 90 R3100 extra:

```bash
./build/robot_kinematics --urdf "/tmp/read_robot_zips/kr90/KR 90 R3100 extra/urdf/KR 90 R3100 extra.urdf" --base base_link --tip tool0 --joints 0 0 0 0 0 0
```

Joint values are radians. If `--joints` is omitted, the tool uses zero for every revolute joint in the resolved chain.

## KR90 UI Angle Mapping

The provided KR90 UI screenshots display controller-style joint angles in degrees. To compare those positions against the URDF chain used by this project, convert them before calling the CLI:

```text
urdf_J1 = ui_J1
urdf_J2 = ui_J2 + 90 deg
urdf_J3 = ui_J3 - 90 deg
urdf_J4 = ui_J4
urdf_J5 = ui_J5
urdf_J6 = ui_J6
```

Then convert degrees to radians for `--joints`.

Four of the supplied screenshot samples match the URDF forward-kinematics output to millimeter precision after this mapping. One screenshot sample differs by exactly about `600 mm` on X while Y/Z still match, so it is not used as a passing regression sample.

## Notes

The tool reads mesh URI strings but does not load or render mesh geometry. It accepts URDF names that contain spaces because the supplied robot packages use them, even though ROS/catkin package names normally should not.
