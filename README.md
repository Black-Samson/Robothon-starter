# MuJoCo Robotics Simulation Hackathon

本次 Hackathon 要求参赛者基于 [Google DeepMind MuJoCo](https://github.com/google-deepmind/mujoco) 构建机器人仿真模拟器、任务场景或数据采集系统。

我们会提供一个官方 GitHub Repo，参赛者需要通过提交 **Pull Request** 的方式提交作品。最终评审将基于 PR 中的代码、模型、运行说明和 demo 视频进行。

## 参与方式

1. Fork 官方 Hackathon Repo
2. 在自己的分支中完成项目开发
3. 提交可运行代码、模型文件、运行说明和 demo 视频
4. 向官方 Repo 发起 Pull Request
5. 在 PR 描述中说明项目目标、技术方案、运行方式和最终效果

## 核心要求

- 使用 MuJoCo 作为主要物理仿真引擎
- 可以使用任意机器人本体，包括机械臂、移动机器人、四足机器人、人形机器人、夹爪、无人车、灵巧手 / 多指机械手等
- 可以使用开源机器人模型，也可以自定义 MJCF 模型
- 需要构建一个可运行的模拟任务、交互系统或数据采集环境
- 最终必须通过 Pull Request 提交
- PR 中需要包含 demo 视频或视频链接

## 推荐方向

- **复杂遥操**：键盘、手柄、VR、Web UI、动作捕捉等输入方式
- **长程任务**：导航、抓取、搬运、装配、开门、整理、清理等多阶段任务
- **数据采集**：自动生成轨迹、状态、动作、图像、深度、传感器或任务标签数据
- **灵巧手操作**：多指抓取、手内旋转、工具使用、按钮操作、拧瓶盖、拼插等
- **实际场景**：K12 教育、校园安防、家庭服务、仓储物流、工业巡检等
- **自由探索**：任何有创意的 MuJoCo 机器人仿真项目

## 特别鼓励

欢迎使用真实开源机器人本体，例如：

- Unitree Go1 / Go2 / G1
- Boston Dynamics Spot
- Franka Emika Panda
- Shadow Hand
- LEAP Hand
- Robotiq Gripper
- 其他 MuJoCo / MJCF 开源模型

可参考模型库：

- [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie)
- [MuJoCo Model Gallery](https://mujoco.readthedocs.io/en/latest/models.html)

## PR 提交内容

每个 PR 需要包含：

- 项目代码
- MuJoCo 场景文件 / 机器人模型 / 相关资源
- 运行说明，包括依赖、安装方式、启动命令、操作方式
- Demo 视频或视频链接
- 简短项目说明，包括：
  - 项目名称
  - 使用的机器人本体
  - 任务目标
  - 技术方案
  - 核心功能
  - 项目亮点
  - 当前限制
  - 未来改进方向

## Demo 视频要求

视频需要由提交的代码运行产生，并展示：

- 模拟环境启动
- 机器人本体和任务场景
- 任务执行过程
- 遥操、自动控制或数据采集逻辑
- 最终结果或任务状态

建议视频长度：1 到 3 分钟。

## 评审标准

- **可运行性**：代码是否能顺利运行，是否容易复现
- **MuJoCo 使用深度**：是否充分利用 MJCF、物理仿真、碰撞、关节、传感器、执行器等能力
- **任务设计**：任务是否清晰、有挑战性、有现实意义
- **控制能力**：是否体现遥操、自动控制、策略控制、任务规划或数据采集能力
- **灵巧操作能力**：如果使用灵巧手，是否体现多指协调、精细操作或高自由度控制
- **工程质量**：代码结构、文档、配置和资源管理是否清晰
- **展示效果**：demo 视频是否直观、有说服力
- **创新性**：场景、本体、任务或应用方向是否有新意

## 示例题目

- 用 Boston Dynamics Spot 完成校园安防巡逻模拟
- 用 Unitree Go1 / Go2 完成复杂地形巡检任务
- 用 Franka Panda 完成 K12 实验器材整理任务
- 用 Shadow Hand / LEAP Hand 完成精细抓取和手内旋转任务
- 构建一个 Web / 手柄 / VR 遥操机器人系统
- 自动生成机器人抓取轨迹数据集
- 模拟家庭服务机器人完成开门、拾取、放置、清理桌面等长程任务

## 最终目标

提交一个可以运行、可以展示、可以复现的 MuJoCo 机器人仿真项目。参赛者通过 Pull Request 提交作品，评审会根据 PR 内容和 demo 视频进行评估。

---

## 本仓库说明

本 Repo 为 Hackathon 官方起始仓库，已包含部分机器人模型与示例脚本，供参赛者 Fork 后在此基础上开发。

### 快速开始

```bash
python3 -m pip install -r requirements.txt
```

运行示例：

```bash
python examples/run_x2_demo.py
python examples/run_zsl1_demo.py
```

使用 MuJoCo Viewer 查看模型：

```bash
python -m mujoco.viewer
```

### 已包含资源

| 路径 | 说明 |
|------|------|
| `assets/x2/` | X2 人形机器人（ultra / hand / fist 等变体） |
| `assets/zsl-1/` | ZSL-1 机器人 URDF / MuJoCo 模型 |
| `examples/` | 示例运行脚本 |
| `model_catalog.json` | 推荐开源机器人模型参考列表 |
