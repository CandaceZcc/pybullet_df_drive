# 运行架构图

本文描述正式 `runSim` v2 会话。源码本地 GUI 验收使用 `--no-interface`，此时没有 C++ Command 和 eCAL 进程边界。

```text
                                  runSim launcher
                                        |
                                        v
                      +----------------------------------+
                      | Python main.py                    |
                      | GUI + manual + v2-realtime + eCAL |
                      +--------+------------------+-------+
                               |                  |
               same process    |                  | child process
                               v                  v
        +---------------------------+   +---------------------------+
        | PyBullet GUI              |   | C++ Command               |
        | physics world             |   | CommandInstanceLock       |
        | keyboard events           |   | authenticated Unix socket |
        +------------+--------------+   +-------------+-------------+
                     |                                |
                     |                                | /sim/wheel/command
                     |                                | 100 Hz, one publisher
                     v                                v
        +---------------------------+   +---------------------------+
        | V2ManualWorldRuntime      |<--| eCAL discovery             |
        | command subscriber        |   +---------------------------+
        | state / sensor publishers |
        +------------+--------------+
                     |
                     | 100 Hz: /sim/wheel/state
                     |  10 Hz: /sim/lidar/points
                     |  10 Hz: /sim/rtk/state
                     |  10 Hz: /sim/imu/attitude
                     v
        +---------------------------+
        | Qt Dashboard              |
        | telemetry, scene, capture |
        +------------+--------------+
                     |
                     | authenticated target lease
                     +-------------------------> C++ Command socket
```

## 命令与安全状态

```text
Dashboard / keyboard
        |
        v
Unix socket target lease -----> C++ Command -----> eCAL WheelCommand
                                       |
                                       +---- timeout / socket close / child exit
                                                     |
                                                     v
                                                   zero command
```

Command 在 socket 对 Python runtime 可见后等待 eCAL subscriber 发现完成，再开始 10 ms 控制节拍。等待上限为 5 秒；没有订阅者时退出并报告错误。该顺序消除首帧发布早于 eCAL discovery 的竞态。

## 记录与导出

```text
Dashboard capture request
        |
        v
Python capture coordinator -----> C++ Recorder -----> session.mcap
                                       |
                                       | complete five-topic boundary
                                       v
                                 C++ Export
                                       |
                         +-------------+-------------+
                         v                           v
                   PCD / PLY                   lidar.lvx2
```

Recorder 在完整五 topic 边界开始和停止。Export 使用同一 release 的 descriptor 与可执行文件生成点云文件；Dashboard 保存最近一次成功 LVX2 的绝对路径，供 Livox Viewer 2 导入。

## 进程边界

| 组件 | 进程 | 资源所有权 |
| --- | --- | --- |
| PyBullet、Dashboard、v2 runtime | Python 主进程 | 物理世界、GUI、eCAL 状态/传感器 participant |
| Command | C++ 子进程 | wheel command publisher、认证 socket、实例锁 |
| Recorder | C++ 子进程 | MCAP 会话与控制 socket |
| Export | 短生命周期 C++ 进程 | PCD、PLY、LVX2、导出回执 |
| ROS Bridge/RViz2 | 可选子进程 | 仅实时点云显示 |

关闭 `runSim` 时，Python 撤销本机命令会话并回收 Command；Command 发送安全零命令。关闭 Recorder、ROS Bridge 或 Viewer 不会改变正在运行的物理世界。
