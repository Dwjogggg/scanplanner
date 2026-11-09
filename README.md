# SCAN-Planner 本地开发与四足仿真工作区

基于 [wuyi2121/SCAN-Planner](https://github.com/wuyi2121/SCAN-Planner) 的本地部署与修改，包含 ROS 规划核心、Go2/MuJoCo 仿真接口及场景配置。

## 目录

- `src/SCAN-Planner/`：目前部署使用的规划源码。
- `catkin_ws/*.py`、`*.launch`、`*.json`：物理仿真、ROS 桥接、场景与测试。
- `catkin_ws/src/scan_planner`：指向源码包的相对符号链接。
- `docs/`：架构解读和服务器部署说明。

本仓库首次提交保存当前服务器快照。现有源码在初始化前已被修改，无法由这次初始提交单独还原此前每次修改；也不将此快照标记为上游原始版本。上游链接与作者信息保留在源码 README 中。

代码源自上游 Apache-2.0 项目；第三方依赖、Go2 资源和步态权重保留各自许可，本仓库不打包这些外部资源。

## 日常版本管理

```bash
git status
git diff
# 选择本次需要保存的文件，避免无关改动混在一起
git add path/to/file
git commit -m "说明本次修改及目的"
# 配置 GitHub origin 并完成认证后
git push
```

VS Code 打开本目录后，可在 Source Control 中查看差异、暂存和提交。编译产物、录像、日志及模型权重已设为忽略，不会随正常源码提交上传。

## 构建与运行

参见 [服务器部署](docs/服务器部署.md) 和 [架构与论文对照](docs/SCAN-Planner架构与论文对照.md)。部署文档中的 `/home/hexiao/3d-navigation`、Conda 环境和外部资源路径属于当前服务器，迁移到其他机器需调整。
