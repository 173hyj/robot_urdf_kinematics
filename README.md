# Robot URDF Kinematics

轻量级工业机器人 URDF 正运动学工具，包含 C++ 命令行程序、自动化测试和 MATLAB 交互式实体网格可视化。项目不依赖 ROS。

## 功能

- 解析 URDF 的 link、joint、origin、axis 和 parent/child 拓扑；
- 计算串联机械臂正运动学并输出 4×4 齐次变换；
- 支持 FANUC M-710iC50/M-710iC70 与 KUKA KR 90 的控制器角度映射；
- MATLAB 滑块交互、实体网格显示、末端 XYZ/WPR 输出。

## 目录

```text
include/robot_kinematics/    C++ 公共头文件
src/                         C++ 实现和命令行程序
tests/                       C++ 自动化测试
matlab/m710_visualizer.m     M-710iC50 MATLAB 可视化
urdf_fk_single.cpp           无第三方库的单文件示例
```

## 依赖

### C++

- CMake ≥ 3.16
- C++17 编译器
- Eigen3
- tinyxml2

Ubuntu/Debian：

```bash
sudo apt update
sudo apt install cmake g++ libeigen3-dev libtinyxml2-dev pkg-config
```

Windows 推荐 Visual Studio 2022 C++ Build Tools、CMake 和 vcpkg：

```powershell
vcpkg install eigen3 tinyxml2 --triplet x64-windows
```

### MATLAB

- MATLAB R2023b 或更高版本
- Robotics System Toolbox

## 编译和测试

Linux/macOS：

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

Windows + vcpkg：

```powershell
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE="path/to/vcpkg/scripts/buildsystems/vcpkg.cmake"
cmake --build build --config Release
ctest --test-dir build --output-on-failure -C Release
```

## 命令行用法

关节值单位为弧度；省略 `--joints` 时所有旋转关节使用 0：

```bash
./build/robot_kinematics --urdf "/path/to/robot.urdf" \
  --base base_link --tip tool0 --joints 0 0 0 0 0 0
```

程序会输出模型信息、运动链和末端 4×4 变换矩阵。

## FANUC M-710iC50 角度规则

RoboGuide/FANUC 界面角度不能直接作为 URDF 的独立关节变量。当前项目验证出的联动是：

```text
J1_URDF = J1
J2_URDF = J2
J3_URDF = J3 + J2
J4_URDF = J4
J5_URDF = J5
J6_URDF = J6
```

对比界面末端位置时，目标 Link 使用 `Link_6`，而不是 `tool0`。4 组 M-710iC50 截图数据均已验证，位置误差小于 0.001 mm。

## KUKA KR 90 角度规则

```text
J1_URDF = J1_UI
J2_URDF = J2_UI + 90°
J3_URDF = J3_UI - 90°
J4_URDF = J4_UI
J5_URDF = J5_UI
J6_URDF = J6_UI
```

转换为弧度后传给 C++ 程序。已提供的 4 组 KR 90 数据均通过验证。

## MATLAB 可视化

请将 M-710iC50 URDF 和对应 `meshes` 目录放在项目的 `robot_test_data/m710/...` 路径下，然后运行：

```matlab
cd('path/to/robot_urdf_kinematics/matlab')
m710_visualizer
```

功能包括：

- 6 个关节滑块和数值输入；
- 实时实体网格显示；
- 自动应用 FANUC `J3_URDF = J3 + J2`；
- `Link_6` 末端 XYZ（mm）和 W/P/R 输出；
- 重置角度和复制末端位姿。

脚本会自动把 URDF 中的 `package://` 网格 URI 转换为本地 `meshes` 路径。

## 单文件示例

`urdf_fk_single.cpp` 只使用 C++14 标准库，不依赖 Eigen、KDL 或 ROS：

```powershell
cl /EHsc /utf-8 urdf_fk_single.cpp
```

运行前请修改文件末尾的 URDF 路径、目标 Link 和 `jointValues`。

## 验证状态

- C++ 自动化测试：1/1 通过；
- KUKA KR 90：4 组位置验证通过；
- FANUC M-710iC50：4 组位置验证通过，误差 < 0.001 mm；
- MATLAB R2023b 实体网格可视化：已运行验证。

## 数据和许可说明

本仓库不包含专有机器人模型、截图或个人本地路径。请自行提供有权使用的 URDF、STL/OBJ/DAE 网格和测试数据，并遵守对应厂商许可。
