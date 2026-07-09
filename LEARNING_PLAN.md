# 大坝斜坡履带机器人 3D 仿真器学习计划

本计划根据 mentor 新需求更新：目标不再只是“差速轮小车在斜坡上跑通”，而是逐步做成一个面向大坝斜坡场景的轻量级移动机器人仿真器。最终需要支持运行时三维位姿反馈、轮胎/履带速度与打滑指标、摩擦设置、传感器、障碍物避让和自动巡航。

当前仓库已经跑通了 PyBullet 基础流程：平地/斜坡场景、简化差速底盘、DIRECT 仿真、GUI 手动控制、CSV 日志和轨迹图。接下来要从“运动学演示”逐步升级到“物理接触 + 传感器 + 自动巡航”。

当前实现路线更新：

- `diff_drive + physics` 保留为平地教学、基础控制和 GUI 手动演示底座；当前是后驱布局，驱动轮在车体后方、单支撑轮在前方，形成更大的三角支撑。
- `tracked_proxy` 保留为坡面姿态、履带打滑和大坝斜坡实验底座；它不是连续履带，但更适合观察坡面 pitch/roll。
- 当前主线同时推进两套模型：二轮模型保证入门稳定性，履带代理保证坡面数据更可靠；阶段 3 斜坡反馈 build 使用 `configs/step3_feedback.yaml` 和 `twr_slope_5deg` 参考坡面。

## 新需求拆解

mentor 明确的需求可以拆成 4 条主线：

- 大坝斜坡建模：先做单一大斜坡，再做坝顶、坝脚、边界和障碍物，最后再考虑 heightfield 或分段坡面。
- 运行数据反馈：记录小车三维坐标、姿态角、速度、左右驱动侧目标速度、实际速度、接触法向力、摩擦系数、打滑估计和轨迹误差。
- 传感器与自动巡航：先接 LiDAR/超声波射线、碰撞/bumper、IMU 和轮速里程计，再做避障和路径跟踪。
- 履带车建模：真实车是两条履带，不是严格意义的“两轮车”，但控制上仍可以先按左右两侧差速驱动来建模。

## 参考仓库使用方式

重点参考 `akinami3/PybulletRobotics`，本地路径：

```text
references/repos/PybulletRobotics/
```

建议优先阅读：

```text
MobileRobot/mobile_robot_basic_en.ipynb
MobileRobot/mobile_robot_sensor_en.ipynb
MobileRobot/mobile_robot_wheel_odometry_en.ipynb
MobileRobot/mobile_robot_local_path_planning_dwa_en.ipynb
urdf/simple_two_wheel_car.urdf
```

它适合作为学习和迁移参考，尤其是：

- `setJointMotorControl2`：左右驱动速度控制。
- `getJointState`：读取轮子角度、速度、关节反馈。
- `changeDynamics`：设置地面和轮子的摩擦。
- `rayTestBatch`：实现 LiDAR/超声波等距离传感器。
- `getCameraImage`：实现相机。
- DWA：用局部轨迹预测做避障和自动巡航。

注意：这个参考仓库本身定位是“学习机器人算法”，不直接等于工程级履带车仿真。我们应该借鉴它的模块和思路，而不是整仓库照搬。

履带物理 V2 阶段优先参考：

```text
references/repos/bullet3/examples/pybullet/examples/snake.py
references/repos/bullet3/examples/pybullet/examples/racecar_differential.py
references/repos/pybullet_sim/scripts/diff_drive_robot.py
references/repos/pybullet_sim/scripts/06_navigate_environment.py
```

- Bullet `snake.py`：参考 `anisotropicFriction`，用各向异性摩擦模拟“前后方向牵引强、横向允许滑移”的履带接触。
- Bullet `racecar_differential.py`：参考多个轮/关节联动的思路，避免只驱动一个轮导致履带转向被被动滚轮拖住。
- `pybullet_sim`：参考传感器、目标导航、障碍环境和 Gym 环境封装，不直接迁移其平地轮式动力学。

## 小车可能用到的参数清单

后续日志不要只记录二维轨迹，而应记录一组更接近真实机器人调试的状态量。第一版可以先记录“优先字段”，后续再补充细项。

### 1. 位姿和姿态

优先字段：

```text
t
x
y
z
roll
pitch
yaw
qx
qy
qz
qw
```

含义：

- `x, y, z`：小车在仿真世界坐标系中的三维位置。
- `roll, pitch, yaw`：横滚、俯仰、航向角；斜坡上尤其要关注 `pitch` 和 `roll`。
- `qx, qy, qz, qw`：四元数姿态，适合后续更严谨地做姿态计算。

PyBullet 数据来源：

```text
getBasePositionAndOrientation()
getEulerFromQuaternion()
```

### 2. 速度和加速度

优先字段：

```text
vx
vy
vz
body_forward_speed
body_lateral_speed
body_vertical_speed
angular_velocity_x
angular_velocity_y
angular_velocity_z
yaw_rate
```

后续可补：

```text
linear_acceleration_x
linear_acceleration_y
linear_acceleration_z
angular_acceleration_z
```

含义：

- `vx, vy, vz`：世界坐标系下的线速度。
- `body_forward_speed`：车体前进方向速度，用来和轮/履带表面速度比较。
- `body_lateral_speed`：车体侧滑速度，履带车转向时很有价值。
- `yaw_rate`：航向角速度，也就是自动巡航常用的 `w`。

PyBullet 数据来源：

```text
getBaseVelocity()
```

### 3. 轮子或履带驱动状态

优先字段：

```text
left_target_drive_speed
right_target_drive_speed
left_actual_drive_speed
right_actual_drive_speed
left_track_surface_speed
right_track_surface_speed
left_body_track_speed
right_body_track_speed
left_drive_position
right_drive_position
left_motor_torque
right_motor_torque
```

履带模型后建议命名：

```text
left_target_track_speed
right_target_track_speed
left_actual_track_speed
right_actual_track_speed
```

含义：

- 目标速度：控制器希望左/右侧达到的速度。
- 实际速度：PyBullet 中关节真实反馈速度。
- 履带表面速度：同侧驱动轮和滚轮按有效半径换算后的平均线速度。
- 履带局部车速：车体在左右履带位置上的局部前向速度，转向时不能直接用车体中心速度代替。
- 驱动位置：轮子或履带驱动轮累计转角，可用于里程计。
- 电机力矩：后续估算负载或能耗时会用到。

PyBullet 数据来源：

```text
setJointMotorControl2()
getJointState()
```

### 4. 接触、摩擦和打滑

优先字段：

```text
ground_lateral_friction
drive_lateral_friction
left_contact_normal_force
right_contact_normal_force
contact_count
left_slip_ratio
right_slip_ratio
body_lateral_slip_speed
```

后续可补：

```text
contact_position_x
contact_position_y
contact_position_z
contact_link_name
rolling_friction
spinning_friction
```

含义：

- 摩擦系数：记录当前实验设置，方便复现实验。
- 接触法向力：能反映轮/履带压在坡面上的状态。
- `slip_ratio`：轮/履带表面速度和车体前进速度的差，用来估计打滑。
- `body_lateral_slip_speed`：车体侧向速度，适合观察履带车坡面横滑。

PyBullet 数据来源：

```text
changeDynamics()
getDynamicsInfo()
getContactPoints()
```

第一版打滑估计：

```text
drive_surface_speed = drive_radius * drive_angular_speed
reference_speed = max(abs(drive_surface_speed), abs(body_forward_speed))
slip_ratio = clamp((drive_surface_speed - body_forward_speed) / reference_speed, -1, 1)
```

履带物理 V2 的打滑估计改为左右履带局部速度：

```text
left_body_track_speed = body_forward_speed - yaw_rate * wheel_base / 2
right_body_track_speed = body_forward_speed + yaw_rate * wheel_base / 2
left_reference_speed = max(abs(left_track_surface_speed), abs(left_body_track_speed))
right_reference_speed = max(abs(right_track_surface_speed), abs(right_body_track_speed))
left_slip_ratio = clamp((left_track_surface_speed - left_body_track_speed) / left_reference_speed, -1, 1)
right_slip_ratio = clamp((right_track_surface_speed - right_body_track_speed) / right_reference_speed, -1, 1)
```

当轮/履带表面速度和车体局部速度都低于约 `0.03 m/s` 时，打滑率记为 0，并在 Dashboard 中标记为“低速”。注意：这里的打滑是带符号估计指标，不等同于真实轮胎实验台测得的精确滑移率；负数表示驱动表面速度低于车体局部速度。

### 5. 地形和环境状态

优先字段：

```text
terrain_type
slope_deg
local_ground_height
local_terrain_normal_x
local_terrain_normal_y
local_terrain_normal_z
nearest_obstacle_distance
```

含义：

- `slope_deg`：当前坡度设置。
- `local_terrain_normal_*`：小车附近地面法向量，后续判断局部坡向会用。
- `nearest_obstacle_distance`：避障控制和安全评估都需要。

数据来源：

```text
场景配置
rayTestBatch()
getContactPoints()
```

### 6. 传感器状态

优先字段：

```text
encoder_left
encoder_right
imu_roll
imu_pitch
imu_yaw
gyro_x
gyro_y
gyro_z
lidar_min_distance
lidar_front_distance
lidar_left_distance
lidar_right_distance
bumper_contact
```

后续可补：

```text
camera_frame_id
camera_detected_marker_id
camera_detected_marker_distance
```

含义：

- 编码器：轮速/履带速度与里程计来源。
- IMU：姿态、角速度和加速度估计。
- LiDAR/超声波：自动避障的主要输入。
- Bumper：碰撞或接触式安全保护。

### 7. 控制和任务状态

优先字段：

```text
command_linear_velocity
command_angular_velocity
target_x
target_y
target_z
path_tracking_error
heading_error
controller_name
controller_state
waypoint_index
goal_reached
```

含义：

- 控制命令：记录算法输出，方便和实际运动对比。
- 跟踪误差：评价自动巡航效果。
- 控制器状态：例如 `cruise`、`avoid_obstacle`、`stop`。
- 任务状态：当前巡检点、是否到达目标。

### 8. 安全和实验指标

优先字段：

```text
collision_count
rollover_risk
max_abs_roll
max_abs_pitch
mission_success
energy_proxy
```

含义：

- `collision_count`：碰撞次数。
- `rollover_risk`：可以先用 `abs(roll)` 或 `abs(pitch)` 是否超过阈值近似。
- `energy_proxy`：可以先用电机力矩和关节速度的乘积积分近似，不追求真实电池模型。

第一版建议先实现这些最小字段：

```text
t, x, y, z, roll, pitch, yaw,
vx, vy, vz, body_forward_speed, yaw_rate,
left_target_drive_speed, right_target_drive_speed,
left_actual_drive_speed, right_actual_drive_speed,
ground_lateral_friction, drive_lateral_friction,
left_contact_normal_force, right_contact_normal_force,
left_slip_ratio, right_slip_ratio,
nearest_obstacle_distance,
command_linear_velocity, command_angular_velocity
```

## 关于履带车是否属于差速二轮车

结论：**履带车不是严格的二轮差速车，但可以先按差速驱动模型近似。**

两轮差速车是左右两个轮子分别驱动，通过左右速度差转向。两条履带的机器人也是左右两侧分别驱动，通过左右履带速度差转向，这类车通常叫 `tracked vehicle` 或 `skid-steer vehicle`。从控制输入看，可以仍然使用：

```text
左侧速度 v_left
右侧速度 v_right
线速度 v = (v_left + v_right) / 2
角速度 w = (v_right - v_left) / track_width
```

区别在于物理接触更复杂：履带和地面是长接触面，转向时会侧滑，摩擦、地形和接触模型比两个圆轮更重要。因此本项目的可行路线是：

1. 先用左右两轮差速模型把控制、日志、传感器和自动巡航跑通。
2. 再把左右轮升级为“左右履带近似模型”，例如每侧多个小轮/滚轮或长条接触块。
3. 最后再根据需要提高履带接触真实性，不一开始追求完整连续履带物理。

## 阶段 0：确认环境能运行

目标：确认 Python、Conda、PyBullet 和 DIRECT 模式可用。

当前可运行：

```bash
conda activate slope-sim
python scripts/check_env.py
python -m pytest -q
```

成功标志：

```text
DIRECT connected: 0
pytest 全部通过
```

如果终端显示 `(base)`，说明还在 Conda 基础环境，不是项目环境。需要执行 `conda activate slope-sim`。

## 阶段 1：保留当前平地差速车演示

目标：保留一个最小可运行版本，作为后续所有复杂功能的回归基线。

当前可运行：

```bash
python main.py --config configs/flat_demo.yaml --duration-sec 1 --mode direct
```

如果在 Ubuntu 本机 X11 桌面，可以运行手动控制：

```bash
python main.py --config configs/flat_demo.yaml --gui --manual
```

成功标志：

```text
log: results/logs/slope_0_*.csv
figure: results/figures/slope_0_trajectory.png
```

说明：这一步仍然可以使用当前的简化模型。它的价值是快速确认入口、日志、画图、GUI 和配置系统没有坏。

## 阶段 2：从运动学演示升级到物理轮地接触

目标：让机器人真正依赖 PyBullet 的轮子关节和地面接触运动，为后续摩擦、打滑和履带近似打基础。

当前进度：

- 已支持 `drive_model: physics`，通过 `setJointMotorControl2()` 控制左右驱动轮。
- `diff_drive` 已调整为左右轮中心距 `0.5m`，与配置中的 `wheel_base` 对齐。
- 默认 GUI demo 使用后驱二轮差速 + 单前低摩擦支撑轮，作为平地教学、基础控制和手动演示底座。
- `diff_drive` 的驱动轮位于 `x=-0.12`，支撑轮位于 `x=+0.30`，前后支撑距离约 `0.42m`；坡面姿态实验仍优先使用 `tracked_proxy`。

后续开发任务：

- 继续保留 `kinematic` 模式，方便和物理模式对比。
- 观察不同坡度和摩擦下的轮速、接触力、打滑率变化。
- 后续如果模型抖动，优先检查轮距、出生高度、支撑轮摩擦、驱动轴参考点和接触点数量。

建议开发后运行：

```bash
python main.py --config configs/gui_step2_demo.yaml --mode direct --duration-sec 2 --drive-model physics --robot-model diff_drive
```

成功标志：

- CSV 同时记录目标轮速和实际轮速。
- CSV 能持续记录 `x, y, z, roll, pitch, yaw`。
- 车辆能在平地前进和转向。
- 物理模式下不再依赖 `resetBasePositionAndOrientation` 每步硬改位置。
- GUI 手动模式下直行不明显左右摇晃，转向能连续响应。

## 阶段 3：加入摩擦、接触和打滑数据反馈

目标：满足“运行时有数据反馈”的核心需求。

建议新增日志字段：

```text
t
x
y
z
roll
pitch
yaw
vx
vy
vz
body_forward_speed
yaw_rate
velocity_sensor_body_forward_speed
velocity_sensor_yaw_rate
linear_acceleration_x
linear_acceleration_y
linear_acceleration_z
angular_acceleration_z
left_target_wheel_speed
right_target_wheel_speed
left_actual_wheel_speed
right_actual_wheel_speed
left_contact_normal_force
right_contact_normal_force
ground_lateral_friction
ground_rolling_friction
ground_spinning_friction
wheel_lateral_friction
support_lateral_friction
left_slip_ratio
right_slip_ratio
left_slip_speed
right_slip_speed
left_slip_valid
right_slip_valid
terrain_type
local_ground_height
local_terrain_normal_x
local_terrain_normal_y
local_terrain_normal_z
nearest_obstacle_distance
command_linear_velocity
command_angular_velocity
```

可行实现：

- `getJointState()` 读取轮/履带驱动侧实际角速度。
- `getBaseVelocity()` 读取车体真实线速度和角速度。
- `changeDynamics()` 设置地面、轮子或履带接触件的摩擦系数。
- `getContactPoints()` 读取接触点、法向力和可用的摩擦相关字段。
- 用“轮边线速度”和“车体前向速度”的差计算打滑估计：

```text
wheel_surface_speed = wheel_radius * wheel_angular_speed
reference_speed = max(abs(wheel_surface_speed), abs(body_forward_speed))
slip_ratio = clamp((wheel_surface_speed - body_forward_speed) / reference_speed, -1, 1)
```

建议开发后运行：

```bash
python main.py \
  --config configs/step3_feedback.yaml \
  --duration-sec 3 \
  --mode direct \
  --drive-model physics \
  --terrain-model twr_slope_5deg \
  --robot-model tracked_proxy \
  --wheel-radius 0.08 \
  --wheel-friction 0.8 \
  --ground-friction 0.8
```

成功标志：

- CSV 中能看到实际轮速、车体速度、接触力和打滑估计。
- CSV 中能看到速度传感、加速度、地形高度、地形法向和摩擦配置。
- CSV 中能看到三维位置和姿态角，尤其是坡面上的 `z`、`roll`、`pitch`。
- 改变摩擦系数后，打滑指标和轨迹表现会变化。
- 能画出 `slip_ratio`、`slip_speed`、接触力和摩擦力随时间变化的图。

注意：PyBullet 不一定直接给出“真实工程意义上的轮胎摩擦系数使用量”。更稳妥的做法是记录“设置的摩擦系数 + 接触法向力 + 接触摩擦力 + 打滑估计 + 速度误差”，作为实验指标。这里的签名打滑率是带正负号的趋势指标；有效接触点是法向力大于阈值的 PyBullet 接触求解点，不等于真实履带接触面积。

## 阶段 4：建立大坝斜坡场景

目标：把“单一斜坡”扩展为“大坝斜坡的大体建模”。

建议从简单到复杂：

1. 单一长斜坡：验证坡度、姿态、轮地接触和打滑。
2. 坝脚 + 坝坡 + 坝顶平台：更接近真实大坝横截面。
3. 加边界、护栏、巡检障碍物：给自动避障准备场景。
4. 分段坡面或 heightfield：模拟起伏和不平整。

建议配置字段：

```yaml
terrain:
  type: dam_slope
  slope_deg: 25
  slope_length: 8.0
  crest_length: 3.0
  toe_length: 2.0
  width: 4.0
```

建议开发后运行：

```bash
python main.py --config configs/dam_slope.yaml --duration-sec 3 --mode direct
```

成功标志：

- 场景能看到坝脚、坡面和坝顶平台。
- 机器人在坡面上运行时 `z`、`pitch`、接触力和打滑指标有变化。
- 日志和图表仍能生成。

## 阶段 5：从二轮近似升级到履带近似

目标：让模型更接近真实“两条履带”的车。

当前进度：

- 已有 `tracked_proxy`，不是完整连续履带，而是“中间驱动轮 + 前后滚轮 + 履带外观条”的工程近似。
- 同侧所有滚轮已按同一条履带线速度联动，并用各向异性摩擦改善 skid-steer 转向。
- 当前不引入连续履带链条作为硬依赖；`tracked_proxy` 作为坡面姿态和履带打滑实验底座保留。

推荐路线：

- 第一版：仍使用左右差速输入，但把变量名从 `wheel_base/wheel_radius` 逐步抽象成 `track_width/drive_radius`。
- 第二版：每侧用多个小轮或滚轮近似履带接触，左右两侧统一控制。
- V2 校准版：同侧所有滚轮联动、按半径换算角速度、使用各向异性摩擦解决转向卡顿。
- 第三版：每侧加入长条接触块或更细的履带外观，但控制仍是左右履带速度。

可行性判断：

- 控制层面：完全可行，履带车可以按左右侧差速控制。
- 物理层面：PyBullet 做“连续柔性履带”难度高，不建议作为第一目标。
- 实验层面：用多轮/接触块近似履带，已经足够做坡面巡航、避障、打滑趋势和算法验证。

建议开发后运行：

```bash
python main.py --config configs/gui_step2_demo.yaml --robot-model tracked_proxy --drive-model physics --duration-sec 3 --mode direct
```

成功标志：

- 配置中可以选择 `diff_drive` 或 `tracked_proxy`。
- 履带近似模型只作为实验对比，不影响默认二轮差速演示稳定运行。
- 日志中使用 `left_track_speed/right_track_speed` 或兼容字段。

注意：当前命令行使用的是 `--robot-model tracked_proxy`，不是旧写法 `--robot tracked`。

## 阶段 5.5：履带物理 V2 与转向校准

目标：修复 `tracked_proxy` 前进转向和原地转向卡顿，让实际 `yaw_rate` 能明显响应角速度命令。

实施内容：

- 同侧驱动轮、前滚轮、后滚轮全部参与速度控制。
- 控制器输出先转成履带线速度，再按每个滚轮有效半径换算角速度。
- 对履带接触件使用各向异性摩擦，保留前后牵引，允许横向滑移。
- 记录左右履带表面速度、左右履带局部车速，并用它们重新计算左右打滑率。

成功标志：

```text
v=0, w=0.8       原地转向尾段 yaw_rate 接近 0.8 rad/s
v=0.35, w=0.8    前进转向不再卡死，yaw 持续变化
v=0.35, w=0      直行时左右打滑率接近 0
```

当前判断：V2 已能作为坡面姿态和履带打滑实验底座使用；`diff_drive` 仍保留为平地教学和基础控制底座。两套模型需要分别用回归测试保护，避免转向卡顿、倒车翘头和打滑率尖峰再次出现。

可运行命令：

```bash
python main.py --config configs/gui_step2_demo.yaml --mode direct --duration-sec 1.5 --drive-model physics --robot-model tracked_proxy --target-angular-velocity 0.8
```

## 阶段 6：接入基础传感器

目标：让机器人不只“知道真值”，还通过传感器感知环境，为自动巡航做准备。

建议先做这些传感器：

- 轮速编码器：来自 `getJointState()`。
- IMU：来自车体姿态、角速度和线加速度估计。
- LiDAR/超声波：参考 `PybulletRobotics` 的 `rayTestBatch`。
- Bumper：参考其 force/bumper 思路，用接触力或碰撞检测判断。
- 相机：参考 `getCameraImage`，后续用于巡线或识别标记。

建议开发后运行：

```bash
python main.py --config configs/dam_slope.yaml --sensors lidar imu encoder --duration-sec 3 --mode direct
```

成功标志：

- CSV 中有传感器字段。
- GUI 模式下可以显示 LiDAR 射线或障碍物命中点。
- 障碍物距离小于阈值时能被检测到。

注意：`--sensors` 当前还没有，需要这一阶段开发。

## 阶段 7：实现最小自动避障

目标：先实现一个能跑、能避开简单障碍物的自动巡航版本。

建议先做反应式避障：

```text
前方无障碍：保持巡航速度
前方距离过近：减速
左前方更空：向左绕
右前方更空：向右绕
危险距离内：停车或后退
```

建议开发后运行：

```bash
python main.py \
  --config configs/dam_slope.yaml \
  --controller reactive_avoidance \
  --duration-sec 10 \
  --mode direct
```

成功标志：

- 场景里放置障碍物后，机器人不会直接撞上。
- 日志记录最近障碍距离、避障状态和控制输出。
- 图中能看到绕行轨迹。

注意：`--controller reactive_avoidance` 当前还没有，需要这一阶段开发。

## 阶段 8：迁移 DWA 局部规划

目标：从简单避障升级到更像真实机器人自动巡航的局部路径规划。

参考：

```text
references/repos/PybulletRobotics/MobileRobot/mobile_robot_local_path_planning_dwa_en.ipynb
```

DWA 的输入：

```text
当前位姿 x, y, yaw
当前速度 v, w
目标点 goal
LiDAR 障碍物命中点
机器人半径或外形近似
速度/加速度限制
```

DWA 的输出：

```text
下一步线速度 v
下一步角速度 w
```

建议开发后运行：

```bash
python main.py \
  --config configs/dam_slope.yaml \
  --controller dwa \
  --goal 6 1 \
  --duration-sec 15 \
  --mode direct
```

成功标志：

- 机器人能朝目标点移动。
- 遇到障碍物时能改变轨迹。
- 能记录 DWA 选择的速度、角速度和最近障碍距离。

注意：DWA 第一版可以先在平地跑通，再搬到大坝斜坡。这样排错会轻很多。

## 阶段 9：加入全局路径和巡检任务

目标：让机器人从“避障”升级到“按任务巡航”。

推荐路线：

- 第一版：给定几个巡检 waypoint，依次到达。
- 第二版：在简化地图上用 A* 生成全局路径。
- 第三版：DWA 负责局部避障，A* 或 waypoint 负责全局目标。

参考：

```text
references/repos/PybulletRobotics/MobileRobot/mobile_robot_global_path_planning_a_star_en.ipynb
references/repos/PythonRobotics/PathTracking/
```

建议开发后运行：

```bash
python main.py \
  --config configs/dam_slope.yaml \
  --controller dwa \
  --mission configs/missions/dam_patrol.yaml \
  --duration-sec 30 \
  --mode direct
```

成功标志：

- 机器人能依次接近多个巡检点。
- 轨迹图显示目标点、参考路径和实际路径。
- 输出任务完成率、最小障碍距离、平均打滑率和耗时。

## 阶段 10：建立实验指标和汇总分析

目标：把仿真变成可以向 mentor 展示和对比的实验系统。

建议指标：

```text
任务完成率
终点误差
平均轨迹误差
最小障碍距离
碰撞次数
平均打滑率
最大打滑率
平均接触法向力
坡面姿态稳定性
单位距离能耗近似
```

建议开发后运行：

```bash
python experiments/run_dam_sweep.py \
  --slopes 10 20 30 \
  --frictions 0.4 0.8 1.2 \
  --controllers reactive_avoidance dwa \
  --trials 3
```

成功标志：

- 每次实验都有独立 CSV。
- 汇总表能比较坡度、摩擦和控制器的影响。
- 可以生成轨迹图、打滑曲线图和指标表。

注意：`run_dam_sweep.py` 当前还没有，需要这一阶段开发。

## 阶段 11：最终展示版本

目标：形成一个可演示、可解释、可复现实验的项目版本。

最终应支持：

- 大坝斜坡场景。
- 履带近似机器人。
- 三维坐标、姿态角、速度和坡面状态日志。
- 轮速/履带速度、摩擦、接触力、打滑估计日志。
- LiDAR/IMU/编码器/bumper 等基础传感器。
- 简单障碍物避让。
- DWA 或 waypoint 巡航。
- 自动生成日志、图表和汇总报告。

推荐演示命令：

```bash
python main.py \
  --config configs/dam_slope.yaml \
  --robot tracked \
  --controller dwa \
  --mission configs/missions/dam_patrol.yaml \
  --duration-sec 30 \
  --mode gui
```

最终成功标志：

- GUI 中能看到履带机器人在大坝斜坡场景中巡航。
- 遇到障碍物可以绕行或停车。
- 运行结束后输出 CSV、轨迹图、打滑曲线和指标汇总。
- README 中能用一组命令复现实验。

## 实际可行方向

最可行的路线是：**先用稳定二轮差速底盘跑通数据反馈、传感器和巡航算法，再把履带作为单独物理建模专题推进。**

短期可行：

- 保留现有稳定二轮差速基础。
- 使用 `diff_drive + physics` 作为默认 GUI 和手动控制模型。
- 加轮速、接触力、摩擦和打滑估计日志。
- 用 `rayTestBatch` 做 LiDAR，先实现简单避障。

中期可行：

- 做大坝斜坡几何场景。
- 在二轮差速功能稳定后，再单独评估多轮或长接触块近似两条履带。
- 迁移 DWA 做局部避障和目标点巡航。

长期再考虑：

- 更真实的履带接触。
- 非均匀地形和湿滑材料。
- 相机识别、SLAM 或更复杂的路径规划。

不建议一开始做：

- 完整连续履带物理仿真。
- 复杂视觉识别。
- 高精度土壤/橡胶/履带接触模型。

这些会很快把项目带到工程仿真或机器人研究级别，不适合作为当前学习型 PyBullet 项目的第一阶段。
