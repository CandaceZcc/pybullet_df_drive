# 阶段一架构说明

本文件描述当前可运行代码。完整三阶段目标和企业参数边界以 `3d仿真平台需求规格.md` 为准。

## 主流程

```text
main.py
  ├─ load_config()
  ├─ run_experiment()        DIRECT / GUI 自动实验
  └─ run_manual_demo()       GUI 人工驾驶
         |
         +-- ActiveManualWorld     当前场景、车辆和场地参数
         +-- apply_manual_switch_request()
         +-- TelemetryDashboard    驾驶、显式应用、复位和内部诊断
```

阶段一原本只要求启动时选择；用户在 GUI 预验收中要求补充运行期车型和场地切换。当前实现仍未进入障碍物管理，不提前引入完整阶段二 `SimulationCoordinator`。

## 车型注册表

`slope_sim/model_registry.py` 是车型的唯一注册入口。每个 `RobotModelSpec` 保存：

- URDF 路径。
- 控制器类型：`differential` 或 `active_steering`。
- 内部轮径、轮距、轴距和安全出生高度。
- 驱动轮、转向轮和支撑轮的语义名称。
- 主动转向机械角度限位。

代码通过 `p.getJointInfo()` 建立“关节名 → PyBullet 索引”映射，不能把某个 URDF 当前的数字索引写死在控制器中。这样修改 URDF 固定挂点时，不会意外把电机命令发给错误关节。

## 四种 URDF

| 车型 | 驱动布局 | 支撑/转向结构 |
|---|---|---|
| `df_front` | 左右轮位于 `x=+0.22 m` | 后部一个球形支撑轮 |
| `df_mid` | 左右轮位于 `x=0 m` | 前后各一个球形支撑轮 |
| `df_back` | 左右轮位于 `x=-0.22 m` | 前部一个球形支撑轮 |
| `active_steering_4wd` | 四轮独立驱动 | 左右前轮为“转向父关节 + 车轮旋转子关节” |

四个模型都预留了顶部安装板、前雷达挂点和后雷达挂点，阶段一只做几何预留，不生成企业传感器数据。

四种车型的车体碰撞尺寸统一为 `0.72 × 0.44 × 0.14 m`，总质量保持在约 `5.95–5.99 kg`，轮径和轮距使用同一基线，避免比较驱动布局时混入明显的尺寸或质量差异。

## 控制器

### 差速控制

`DifferentialDriveRobot.command_twist(v, w)` 使用轮距和轮径把车体线速度、角速度换成左右轮角速度。左右轮同速时直行，速度不同时转弯，方向相反时可原地转向。

### 主动转向控制

`ActiveSteeringRobot` 的四个车轮都有速度电机，两个前轮还有位置电机：

1. `command_wheel_speeds()` 接收四个驱动轮速度和两个前轮转向速度。
2. 转向速度乘以物理步长，积分为新的目标角。
3. 目标角限制在 `±0.55 rad`，避免超过 URDF 机械范围。
4. `read_drive_wheel_speeds()` 和 `read_steering_wheel_angles()` 从 PyBullet 关节状态读取实际值，不直接回显命令。

主动转向车的四个实际轮速和两个实际前轮转角进入统一遥测，因此 CSV 与 Dashboard 都能直接检查四驱和转向反馈。

这一层级借鉴 Bullet 官方 racecar 示例，但本项目用关节名解析，未复制官方示例的硬编码索引。

## 三类场地

### `flat`

使用质量为 0 的静态长方体作为连续水平碰撞面。

### `slope`

`slope` 由高位平地、下坡、低位平地三个静态 box 组成。`SceneInfo.body_ids` 按 `(upper_id, ramp_id, lower_id)` 保存三者所有权，`body_id` 指向中间坡段，便于保持旧的单主地形调用。创建中任一后续段失败时，场景工厂按创建逆序删除已创建的 body，不能留下半套地形。

正 `slope_deg` 时，车辆从高位平地沿世界 `+X` 行驶，依次经过下坡和低位平地。出生点位于高位平地中部，出生姿态水平；高位平地表面比低位平地高 `8 * sin(slope_deg)`。接缝有很小的水平重叠，避免 raycast 或车轮落入数值缝隙。

### `golf_heightfield`

`generate_golf_heightfield()` 使用固定随机种子叠加低频丘陵和横坡、椭圆高斯丘、负高斯浅洼与小尺度连续波，再交给 PyBullet `GEOM_HEIGHTFIELD` 创建单个碰撞地形。平滑驾驶廊道会适度减弱局部丘洼和细节，但保留大部分低频丘洼。

关键点：

- 相同 `golf_seed` 和 `golf_relief` 生成完全相同的高度数组。
- 只用低频连续函数，不逐格生成独立随机高度，因此不会形成尖锐噪声或台阶。
- 出生位置和初始姿态来自真实 heightfield 射线高度与局部法向。
- 公共高度数组语义是 `rows=y`、`columns=x`，以 y-major 顺序写入。PyBullet 的 heightfield 参数把快轴视为 `numHeightfieldRows`，所以创建时传入 `numHeightfieldRows=columns`、`numHeightfieldColumns=rows`；这样非方形网格的世界 X/Y 方向不会交换。

## GUI 相机状态流

Dashboard 的“启用跟随”和视角是持续状态。每帧 `TelemetryDashboard.current_command()` 读取控件并写入 `DashboardCommand`；`merge_manual_commands()` 合并 PyBullet 键盘驾驶输入时保留这两个字段；`limit_manual_command_step()` 处理加速度限制和场景动作时继续透传它们。随后手动主循环在物理步进和状态读取后，若跟随开启，调用 `update_follow_camera()`，并从当前 `ActiveManualWorld.active_robot.robot.robot_id` 读取活动车辆。车型或场地切换完成后，`ActiveManualWorld` 已替换，因此不会继续引用已删除的车体。

- `front`：Dashboard 显示为“车后”，target 跟随车体位置，yaw 为车辆实际 yaw 减 90 度。
- `side`：target 跟随车体位置，yaw 等于车辆实际 yaw。
- `custom`：Dashboard 显示为“固定”，target 仍跟随车体位置，yaw 使用配置的 `camera_yaw`，不随车头旋转。

## 地形探测

PyBullet 的单次 `rayTest()` 只返回最近命中。从机器人正上方发射会先打到机器人自身，因此 `_probe_terrain_for_robot()` 在车体侧面偏移一个安全距离后向下探测。平面和斜面高度不受横向偏移影响；高尔夫场地的结果是车体附近局部地形近似，企业级传感器语义会在阶段三重新收敛。

## Dashboard 运行期切换事务

Dashboard 不在 Qt 按钮回调中调用 PyBullet。下拉框只保存待应用值，点击按钮后生成一次性请求，由 `run_manual_demo()` 所在物理主线程处理：

1. 车型切换先在当前出生点成功加载新车，再删除旧车，因此加载失败时旧车仍可继续使用。
2. 场地切换先清零命令，再通过 `create_slope_scene()` 的 `resetSimulation()` 重建场地和当前车型。
3. 目标场地失败时，用保存的上一个 `TerrainSelection` 重建旧世界；回滚也失败才终止仿真，避免无地面继续步进。
4. 成功或回滚后更新活动 `body_id`、相机引用和 Dashboard 控件，清空旧平滑值与曲线。
5. CSV 时间保持连续，并用每行的 `robot_model`、`terrain_type` 标识切换前后的活动对象。

## 配置边界

启动配置和 Dashboard 运行期可选择的业务参数是：

- `robot_model`：四种车型。
- `terrain_model`：`flat` / `slope` / `golf_heightfield`。
- `slope_deg`：仅斜面使用。
- `golf_seed`、`golf_relief`：高尔夫场地复现与起伏预设。

轮距、轮径、质量、惯量、电机力、摩擦、heightfield 网格等仍是内部仿真/诊断参数，不属于企业 eCAL 字段。

## 验证

`scripts/verify_stage1_matrix.py` 对四车型 × 三场地逐项执行：

- 加载和静置落地。
- 短距离前进。
- 逐帧检查有限位姿、地形边界、地面接触和最大穿透深度。
- 逐帧检查 roll/pitch，避免只看最终帧漏掉中途翻转。

此外，pytest 覆盖注册表、URDF 关节、支撑轮、三种差速车的前进/后退/左右转/差速转向、主动转向速度积分和限位、4+2 实际反馈、三段下坡、高尔夫 heightfield 轴映射和廊道、配置/CLI、日志、Dashboard 一次性请求、持续相机状态、车型事务替换、场地事务重建和失败回滚。
