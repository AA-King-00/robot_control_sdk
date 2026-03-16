# robot_control_sdk

一个用于学习、实现和验证机器人控制算法的个人机器人运动控制 SDK，目标平台为 UR5。

## 项目目标

- 构建工业风格的 C++ 机器人 SDK
- 从零实现核心机器人学模块
- 在仿真环境和真实硬件上验证算法
- 为机器人 / 具身智能方向打造扎实的工程作品集

## 当前阶段

第一个月：工程基础 + 正向运动学

当前任务：
- 搭建仓库结构与编码规范
- 用 C++ 实现 UR5 正向运动学
- 使用 Google Test 添加单元测试
- 使用 GitHub Actions 添加 CI 构建检查
- 在仿真和真实机器人上验证正向运动学结果

## 项目结构
```
robot_control_sdk/
├─ include/        # 公共头文件
├─ src/            # 源文件
├─ tests/          # 单元测试
├─ examples/       # 可运行示例
├─ config/         # 机器人参数与 yaml 配置文件
├─ docs/           # 笔记与设计文档
├─ scripts/        # 辅助脚本
└─ .github/        # CI 工作流
```

## 构建方法
bash
mkdir -p build
cd build
cmake ..
make -j