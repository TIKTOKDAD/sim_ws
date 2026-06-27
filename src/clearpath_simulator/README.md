# clearpath_simulator

## 中文架构导读

这个仓库提供 Clearpath 机器人在 Gazebo Harmonic / ROS 2 Jazzy 下运行仿真的资源和生成器。
它不是单一功能包，而是由三个 ROS 2 package 组成：

- `clearpath_simulator`：元包，主要用于把 simulator 相关 package 作为一个整体发布。
- `clearpath_gz`：Gazebo 入口包，包含 launch 文件、world 文件、GUI 配置、mesh 模型和地理参考图。
- `clearpath_generator_gz`：仿真专用生成器，根据 `robot.yaml` 生成 Gazebo bridge 参数和 ROS 2 launch 文件。

典型启动链路如下：

1. `ros2 launch clearpath_gz simulation.launch.py` 是总入口。
2. `simulation.launch.py` 先包含 `gz_sim.launch.py` 启动 Gazebo world，再包含 `robot_spawn.launch.py` 准备机器人。
3. `robot_spawn.launch.py` 读取 `~/clearpath/robot.yaml`，按需运行描述、语义描述、launch 和参数生成器。
4. `clearpath_generator_gz` 会把平台自带传感器、用户配置传感器和命名空间转换成 `ros_gz_bridge` 所需的配置。
5. 生成完成后，`ros_gz_sim create` 从 `robot_description` 把机器人实体 spawn 到当前 world。

维护时最需要注意的几个约定：

- `world` 参数只传世界名，不传 `.sdf` 后缀；`gz_sim.launch.py` 会自动拼接。
- 仿真默认开启 `use_sim_time`，这样 ROS 节点统一使用 Gazebo `/clock`。
- `GZ_SIM_RESOURCE_PATH` 会追加 `clearpath_gz/worlds`、`clearpath_gz/meshes` 和已 source 的 ROS package
  share 路径，否则 Gazebo 可能找不到 world、mesh 或依赖模型。
- `robot.yaml` 中的 namespace 会影响机器人模型名、bridge topic 和 TF topic；空 namespace 时模型名为 `robot`。
- PTZ/PTU 传感器在仿真中有额外的 relay/controller 节点，用来把真实驱动接口映射到 Gazebo joint controller。

## Setup

Prerequisites:
  - Install [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html)

### Gazebo Harmonic

See [Gazebo Installation](https://gazebosim.org/docs/latest/ros_installation/) for more information
on installing Gazebo.

```
sudo apt-get install ros-${ROS_DISTRO}-ros-gz
```

### Workspace

```
mkdir ~/clearpath_ws/src -p
cd ~/clearpath_ws/src
git clone https://github.com/clearpathrobotics/clearpath_simulator.git
cd ~/clearpath_ws
rosdep install -r --from-paths src -i -y
colcon build --symlink-install
```

### Setup path

```
mkdir ~/clearpath
```

Copy your `robot.yaml` into `~/clearpath`

## Launch

```
ros2 launch clearpath_gz simulation.launch.py
```

## Worlds

The `clearpath_gz` package includes several simulation worlds. To select a specific world, use the
`world` launch parameter, e.g.
```
ros2 launch clearpath_gz simulation.launch.py world:=pipeline
```

Available worlds are:

| World                 | Description                                                                                                            | Screenshots                  | Geographic Location      |
|-----------------------|------------------------------------------------------------------------------------------------------------------------|------------------------------|--------------------------|
| `construction`        | The same floorplan as the `office` world, but under construction. Features non-solid walls and debris piles.           | [link](docs/construction.md) | Waterloo ON, Canada      |
| `office`              | The same floorplan as the `construction` world. Features narrow hallways, doorways, meeting rooms, and loading docks.  | [link](docs/office.md)       | Waterloo ON, Canada      |
| `orchard`             | An outdoor, agricultural environment featuring rows of trees. The terrain has small slopes, but is mostly flat.        | [link](docs/orchard.md)      | Nikea, Greece            |
| `pipeline`            | A rugged, outdoor environment featuring steeper hills, a river and bridge, a small cave, solar panels, and a pipeline. | [link](docs/pipeline.md)     | Northern Alberta, Canada |
| `solar_farm`          | An outdoor, agricultural environmentf featuring gentle hills, a barn, rows of solar panels, and fences.                | [link](docs/solar_farm.md)   | Stonewall MB, Canada     |
| `warehouse` (default) | A flat, indoor warehouse environment. Features shelves and people.                                                     | [link](docs/warehouse.md)    | Rio de Janeiro, Brazil   |

## Generator Tests

Changes to the generators in this repository (`clearpath_generator_gz`) may affect the
generated output for launch files and parameter files. The
[clearpath_generator_tests](https://github.com/clearpathrobotics/clearpath_generator_tests)
repository versions the expected output and validates it through CI.

Before merging, ensure a corresponding branch with the **same name** exists in
`clearpath_generator_tests` with regenerated samples. See the
[Development Workflow](https://github.com/clearpathrobotics/clearpath_generator_tests#development-workflow)
section of `clearpath_generator_tests` for the full process.


## Creating Map Tiles

The `orchard`, `pipeline`, and `solar_farm` worlds include geotagged TIF images in the `geotif`
directory. These images can be used to generate map tiles of the simulation environment, if
desired.

To generate the tiles, first install the `gdal-bin` package:
```bash
sudo apt install gdal-bin
```

Then run the following command to generate the tiles:
```bash
gdal2tiles.py $(ros2 pkg prefix clearpath_gz)/share/clearpath_gz/geotif/WORLD_geo.tif
```
substituting `WORLD` with `orchard`, `pipeline`, or `solar_farm`.

The generated files will be located in the current working directory in a new directory called
`WORLD_geo` (e.g. `pipeline_geo`).

Note that while the simulation worlds' locations have been chosen to be geographically similar to
the envrionments depicted, the simulations are wholly fictional locations; the generated tiles
will not mesh seamlessly into any satellite map of the region depicted.
