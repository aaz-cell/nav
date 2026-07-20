# iSweep + TEB 导航实验

本仓库保留任意形状地面机器人 iSweep 导航实验所需的 ROS Noetic 代码，重点包含 **iSweep + TEB** 实验组、**GlobalPlanner + TEB** 对照组，以及两组共用的定位、地图和速度参数。

## 实验组设置

| 组别 | 全局规划 | 局部规划/控制 | 启动文件 |
| --- | --- | --- | --- |
| iSweep + TEB | iSweep 扫掠体积轨迹 | TEB | `navigation_isweep_switch.launch`，设置 `run_isweep_teb_local_planner:=true` |
| 对照组 | ROS `global_planner/GlobalPlanner` | TEB | `navigation_global_teb_control.launch` |
| iSweep 消融组 | iSweep | iSweep 风险感知局部规划器 | `navigation_isweep_switch.launch`，设置 `run_isweep_teb_local_planner:=false` |

实验组和对照组共用定位入口、代价地图配置、TEB 参数、速度上限和 `/cmd_vel` 输出话题。对照实验时请使用相同地图、起终点和启动参数。

## 仓库结构

- `isweep_planner/`：iSweep 全局搜索、SVSDF 轨迹优化及风险感知局部规划。
- `navigation_adapter/`：把定位、地图、目标和里程计转换为 iSweep 输入。
- `dynamic_obstacle_adapter/`：合并静态栅格地图与实时激光障碍物。
- `planner_bridge/`：处理 iSweep 速度输出、重规划状态及 Ackermann 约束。
- `sentry_nav/`：iSweep + TEB、对照组启动文件，公共参数和 RViz 配置。
- `sentry_slam/FAST_LIO*`：两组共用的 MID360 建图与定位链。
- `ros_numpy/`：FAST-LIO 定位脚本使用的 Git 子模块。

原 NEXTE 哨兵项目中的串口通讯、决策、Point-LIO、PCD 转换工具、旧导航入口和其他传感器示例已移除。

## 环境与编译

目标环境为 Ubuntu 20.04、ROS Noetic 和 Livox MID360。先准备 `livox_ros_driver2`，再在 catkin 工作空间中克隆并编译：

```bash
cd ~/nav_ws/src
git clone --recurse-submodules git@github.com:aaz-cell/nav.git NEXTE_Sentry_Nav
cd ..
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

若首次克隆时没有拉取子模块：

```bash
git submodule update --init --recursive
```

## 地图数据

PCD、PGM、bag 和运行日志不提交到 Git。启动时通过 `map` 和 `map_2d` 参数分别传入本机 PCD 先验地图与二维地图 YAML。例如：

```bash
MAP_PCD=/absolute/path/to/scans.pcd
MAP_2D=/absolute/path/to/map.yaml
```

## 运行

启动 iSweep + TEB 实验组：

```bash
roslaunch sentry_nav navigation_isweep_switch.launch \
  run_isweep_teb_local_planner:=true \
  map:="$MAP_PCD" \
  map_2d:="$MAP_2D" \
  rviz:=true
```

启动 GlobalPlanner + TEB 对照组：

```bash
roslaunch sentry_nav navigation_global_teb_control.launch \
  map:="$MAP_PCD" \
  map_2d:="$MAP_2D" \
  rviz:=true
```

如果定位和地图节点已由其他进程提供，可对两组都设置：

```bash
run_localization:=false rviz:=false
```

通过 RViz 的 `2D Nav Goal` 向 `/move_base_simple/goal` 发布目标。两组默认都向 `/cmd_vel` 输出控制指令，连接真机前请确认急停、底盘方向和速度限制。

## 主要接口

- 定位：`/localization`
- 里程计：`/Odometry`
- 静态地图：`/prior_map`
- 实时激光：`/scan`
- 导航目标：`/move_base_simple/goal`
- 速度指令：`/cmd_vel`
- iSweep 轨迹：`/isweep_planner/trajectory`

公共 TEB 与代价地图参数位于 `sentry_nav/param/`；iSweep 参数位于 `isweep_planner/config/planner.yaml`。

## 实验结果记录

`experiment_recorder.py` 对 iSweep + TEB 与 GlobalPlanner + TEB 使用同一套指标定义。每次向 `/move_base_simple/goal` 发布目标后，它会记录端到端规划延迟、首条全局路径长度、实际行驶距离、导航时间、终点误差、最小激光量程、速度/加速度/jerk、停顿、换向和成功状态。

在线记录 iSweep + TEB：

```bash
mkdir -p ~/robot3/experiment_data
rosrun sentry_nav experiment_recorder.py \
  --mode isweep_teb \
  --output ~/robot3/experiment_data/isweep_teb_runs.csv
```

在线记录对照组：

```bash
rosrun sentry_nav experiment_recorder.py \
  --mode global_teb \
  --output ~/robot3/experiment_data/global_teb_runs.csv
```

记录器应在发送 RViz 导航目标前启动。它支持在同一进程中连续记录多个目标，并在目标成功、失败、超时、被新目标替换或节点退出时写入一行。默认统一成功容差为位置 `0.10 m`、航向 `0.20 rad`、持续 `1.0 s`，默认超时为 `180 s`。

离线汇总一个或多个 rosbag 时使用系统 ROS Python；该流程不需要 conda 中的 Open3D：

```bash
source /opt/ros/noetic/setup.bash
source ~/robot3/nav_ws/devel/setup.bash

/usr/bin/python3 \
  ~/robot3/nav_ws/src/NEXTE_Sentry_Nav/sentry_nav/scripts/experiment_recorder.py \
  --bag ~/robot3/experiment_data/*.bag \
  --mode auto \
  --output ~/robot3/experiment_data/experiment_results.csv
```

用于离线分析的 bag 至少应包含 `/move_base_simple/goal`、`/localization`、`/cmd_vel`，以及对应组的 `/isweep_planner/trajectory` 或 `/move_base/GlobalPlanner/plan`。若还要统计最小激光量程和状态，需同时记录 `/scan`、`/isweep_teb_local_planner/status` 或 `/move_base/status`；iSweep 内部耗时还需 `/isweep_planner/planning_time` 与 `/isweep_planner/planning_stats`。

离线模式生成两个文件：

- `experiment_results.csv`：每个 bag 中每个目标一行的详细指标。
- `experiment_results_summary.csv`：按模式汇总运行次数、成功率，以及规划时间、路径长度、导航时间、实际距离和控制平滑性的均值/中位数/标准差。

自动识别依赖两组各自的全局路径话题。若一个 bag 中混有两个导航栈的话题，应显式使用 `--mode isweep_teb` 或 `--mode global_teb`。公平对照采用 `planning_latency_s`，即目标消息到第一条有效全局路径的端到端时间；`planner_reported_time_s` 和 `isweep_*` 字段仅作为 iSweep 内部阶段分析，不应直接当成两组共同指标。

## Python 环境

ROS Noetic 节点应使用系统 Python。若终端自动激活了 conda，可使用：

```bash
bash sentry_nav/scripts/launch_isweep_clean.sh \
  run_isweep_teb_local_planner:=true \
  map:="$MAP_PCD" \
  map_2d:="$MAP_2D"
```

脚本会退出 conda 环境，并自动定位当前 catkin 工作空间。

## 许可

仓库顶层代码沿用 [MIT License](LICENSE)。FAST-LIO、FAST-LIO-Localization、ros_numpy 及内嵌第三方组件保留各自的许可证。
