# Low-poly 3D 初版

本版从 master `1c97008` 开始制作。打开工程后默认进入银石 3D 场景，默认只有一辆车，加载完成后自动开始连续刷圈，空格可暂停／继续。车辆由 master 已有的规则司机驾驶，未接入其他分支的银石学习专家。

## 如何打开

使用 Godot **4.6.3 .NET**，导入当前 worktree 的 `project.godot`，构建后运行。命令行方式：

```sh
dotnet build StintegyEVO.csproj -p:Optimize=true
/Applications/Godot_mono.app/Contents/MacOS/Godot --path .
```

首次准备银石时会计算原有的最小曲率参考线，本机约需半分钟，期间会显示加载文字。原来的二维调试场景保留在 `Levels/root.tscn`。

## 镜头和操作

| 操作 | 效果 |
|---|---|
| 1 | 俯斜跟车：看赛车形体、并排距离和当前弯道 |
| 2 | 高空全景：看完整赛道布局 |
| 3 | 侧面高空全局：从侧上方观察整条赛道和场边建筑 |
| 滚轮 | 缩放当前镜头 |
| 鼠标右键拖动 | 绕观察点旋转 |
| 左右方向键／点击排名行 | 切换观察车辆 |
| F | 切换跟车和全景 |
| 空格／RUN、PAUSE | 开始或暂停仿真 |
| Q、E | 降低、提高轮胎使用档 |
| A、D | 降低、提高动力输出档 |
| H | 隐藏／显示 HUD |

镜头覆盖全长 5.891 公里的赛道时，真实尺寸的车必然很小。右下角小地图用于定位当前车辆，近距离细节用跟车镜头查看。

## 设计依据

这些链接分别对应参考作品和具体制作方法；本版没有下载或复用游戏美术资产，也没有生成图片资产。

1. [Art of Rally 官方画面](https://www.artofrally.com/)：参考有辨识度的车身轮廓、简洁环境和俯斜构图。落到本项目，是降低环境颜色的存在感，把车、路肩和比赛线路留给视线。
2. [Blender 的平滑／平面着色教程](https://docs.blender.org/UATEST/manual/en/dev/modeling/meshes/editing/face/shading.html)：低多边形的可读性来自形体和面的法线。本版用独立面法线表达车鼻、侧箱、前后翼和轮胎，不靠密集纹理伪装细节。
3. [Godot 程序化网格教程](https://docs.godotengine.org/en/stable/tutorials/3d/procedural_geometry/surfacetool.html)：用于顶点、法线和颜色的组织。道路按纵向分块，横向细分以表现路拱与横坡。
4. [Godot 环境与后处理教程](https://docs.godotengine.org/en/stable/tutorials/3d/environment_and_post_processing.html)：本版采用一盏暖色太阳、较冷的环境补光和 Filmic 色调映射。顶点颜色明确按 sRGB 转换，避免整体发白。
5. [Godot MultiMesh 文档](https://docs.godotengine.org/en/stable/classes/class_multimeshinstance3d.html)：树木按空间簇实例化，避免为每棵树增加独立更新逻辑。

采用灰绿色环境、深灰路面、红白路肩；车队用有限的红、黄、蓝绿和白色配色。建筑有明确用途：看台、车库、计时桥。HUD 用米白平面和深色文字，红色只承担选中状态，不使用玻璃卡片、装饰性渐变或发光边框。

## 真实性与初版边界

- 赛车尺寸来自碰撞尺寸的现行默认量级；车位、朝向和驾驶行为来自 Core。
- 赛道中心线、高差和横坡读取 Core。高差从坡度积分恢复并处理闭环误差；Core 自带银石高程就是近似模型，不是实测地形。
- 场边建筑是示意性模型，不是银石建筑的精确复刻。
- 场景不新增 Godot 刚体或碰撞体，未修改车辆动力学、规则司机或训练代码。
- master 的规则司机在本机 20 车密集发车场景中仍很重。仿真在单个后台任务中按固定 1/60 秒推进，画面读取完成后的快照，保持镜头和界面可操作。
- HUD 中 FPS 是画面帧率，CORE 是一次仿真步耗时，SIM 是由该耗时估算的最高实时倍率。画面流畅不代表仿真达到实时。暂停时允许正在计算的一步收尾，不会积累追赶任务。

## 验证

基线核心测试 259 项通过；新增 3 项测试检查坡度与高度一致、横坡方向和赛道首尾接缝。图形冒烟测试会操作三个镜头、缩放、选车、窗口缩放和暂停／继续，并保存真实 Metal 视口截图：

```sh
dotnet test Core/Tests/StintegyEVO.Core.Tests.csproj -c Release --filter FullyQualifiedName~TrackSurfaceGeometryTests
/Applications/Godot_mono.app/Contents/MacOS/Godot --path . --script res://Tools/lowpoly_smoke.gd
```

截图位于 `.tmp/lowpoly/`。运行机器负载会影响结果，应将渲染和仿真耗时分开比较。
