# Stereo Planar Localization (Minimal, Non-SLAM)

中文与英文双语说明，面向最小可运行实现。

- Core script: main.py
- Scope: three images + camera intrinsics K + planar geometric solve
- Not included: SLAM, VO pipelines, ROS, map optimization

---

## 1. Chinese Guide

### 1.1 项目简介
本项目实现一个简化的双目视觉平面定位流程，输入为：

- 左目图像 left_img
- 右目图像 right_img
- 查询图像 query_img
- 相机内参矩阵 K

核心流程：

1. ORB 特征提取与匹配（left-query, right-query）
2. 通过 findEssentialMat + recoverPose 得到平移方向 tL, tR
3. 将方向投影到 XOY 平面，仅使用 (tx, ty)
4. 构建 4 个平面方程求解 (xL, yL, xR, yR)
5. 输出双目中心 (x, y)

补充约束采用：yL - yR = 0（假设双目基线平行世界 X 轴）。

### 1.2 用 uv 管理环境（从零开始）

#### Step A: 安装 uv（Linux）
可选其一：

方式 1（官方安装脚本）：

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

方式 2（如果系统已有 pipx）：

```bash
pipx install uv
```

安装后确认：

```bash
uv --version
```

#### Step B: 在项目目录创建虚拟环境

```bash
cd /home/liuyu/project/MELoc
uv venv .venv
```

#### Step C: 安装依赖

```bash
uv pip install --python .venv/bin/python numpy scipy opencv-python
```

#### Step D: 可选激活

```bash
source .venv/bin/activate
```

> 说明：即使不激活，也可以直接使用 .venv/bin/python 运行。

### 1.3 运行方式

#### 方式 1：Demo（自动生成仿真数据并测试）

```bash
.venv/bin/python main.py --demo --demo_out demo_data --seed 7
```

运行后会在 demo_data 下生成：

- left_demo.png
- right_demo.png
- query_demo.png

并输出：

- tL, tR（方向向量）
- xL, yL, xR, yR
- Stereo center (x, y)
- Residual norm
- Ground truth / Abs error / Oracle 对照结果（demo 模式）

#### 方式 2：真实输入模式

```bash
.venv/bin/python main.py \
  --left /path/to/left.png \
  --right /path/to/right.png \
  --query /path/to/query.png \
  --K 700,0,480,0,700,320,0,0,1 \
  --xD 1.4 \
  --yD 1.0 \
  --B 0.6 \
  --init 0,0,1,0
```

### 1.4 输入参数解析

- --left
  - 左目图像路径（灰度读入）
- --right
  - 右目图像路径（灰度读入）
- --query
  - 查询图像路径（灰度读入）
- --K
  - 内参矩阵，9 个逗号分隔浮点数，按行主序
  - 例如：fx,0,cx,0,fy,cy,0,0,1
- --xD, --yD
  - 查询相机在平面坐标系中的已知位置 CD=(xD,yD)
- --B
  - 双目基线长度
- --init
  - 求解初值 xL,yL,xR,yR，默认 0,0,1,0
- --demo
  - 启用自包含仿真数据与演示流程
- --demo_out
  - demo 图像输出目录，默认 demo_data
- --seed
  - demo 随机种子，默认 42

### 1.5 输出解析

标准输出关键字段：

- tL (direction), tR (direction)
  - recoverPose 返回并归一化后的平移方向（无绝对尺度）
- xL, yL, xR, yR
  - 求解出的左右相机平面坐标
- Stereo center (x, y)
  - 双目中心坐标，x=(xL+xR)/2, y=(yL+yR)/2
- Residual norm
  - 4 个约束方程残差范数

demo 模式额外输出：

- Ground truth
- Abs error
- Oracle solve / Oracle abs error

### 1.6 数学模型（实现一致）

未知量：xL, yL, xR, yR

约束：

1) tLy(xD - xL) - tLx(yD - yL) = 0

2) tRy(xD - xR) - tRx(yD - yR) = 0

3) (xL - xR)^2 + (yL - yR)^2 = B^2

4) yL - yR = 0

### 1.7 工程优化点

- ORB + BFMatcher KNN 比值检验
- 双向一致性匹配（mutual check）
- F 矩阵 RANSAC 几何内点筛选（可回退）
- least_squares + Huber loss + multi-start
- 解分支筛选：xR > xL

### 1.8 已知局限与建议

- recoverPose 的 t 仅为方向，尺度不可观
- 若纹理少、动态物体多、视差小，方向估计会不稳
- 该简化模型依赖平面和安装假设，存在多解/镜像分支风险

最小改进建议：

1. 在真实系统中加入已知基线朝向角约束（替代 yL=yR）
2. 多组初值并比较残差与先验一致性
3. 对输入图像做畸变校正与曝光归一化

---

## 2. English Guide

### 2.1 Overview
This repository provides a minimal stereo planar localization pipeline.

Inputs:

- Left image (left_img)
- Right image (right_img)
- Query image (query_img)
- Camera intrinsics matrix K

Pipeline:

1. ORB feature extraction and matching (left-query, right-query)
2. Essential matrix estimation and pose recovery via findEssentialMat + recoverPose
3. Keep only planar direction components (tx, ty)
4. Solve 4 planar equations for (xL, yL, xR, yR)
5. Output stereo center (x, y)

The extra equation used here is yL - yR = 0, meaning the stereo baseline is parallel to the world X-axis.

### 2.2 Environment Setup with uv

#### Step A: Install uv (Linux)
Option 1:

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

Option 2 (if pipx is available):

```bash
pipx install uv
```

Verify:

```bash
uv --version
```

#### Step B: Create virtual environment

```bash
cd /home/liuyu/project/MELoc
uv venv .venv
```

#### Step C: Install dependencies

```bash
uv pip install --python .venv/bin/python numpy scipy opencv-python
```

#### Step D: Optional activation

```bash
source .venv/bin/activate
```

### 2.3 How to Run

#### Mode 1: Demo mode (synthetic data + full test)

```bash
.venv/bin/python main.py --demo --demo_out demo_data --seed 7
```

Outputs generated demo images:

- left_demo.png
- right_demo.png
- query_demo.png

And prints:

- direction vectors tL, tR
- solved xL, yL, xR, yR
- stereo center (x, y)
- residual norm
- ground truth / abs error / oracle comparison (demo mode)

#### Mode 2: Real input mode

```bash
.venv/bin/python main.py \
  --left /path/to/left.png \
  --right /path/to/right.png \
  --query /path/to/query.png \
  --K 700,0,480,0,700,320,0,0,1 \
  --xD 1.4 \
  --yD 1.0 \
  --B 0.6 \
  --init 0,0,1,0
```

### 2.4 Argument Reference

- --left: path to left image
- --right: path to right image
- --query: path to query image
- --K: 9 comma-separated values (row-major), e.g. fx,0,cx,0,fy,cy,0,0,1
- --xD, --yD: known planar position of query camera CD=(xD,yD)
- --B: stereo baseline length
- --init: initial guess xL,yL,xR,yR, default 0,0,1,0
- --demo: run synthetic self-contained demo
- --demo_out: directory for generated demo images
- --seed: random seed for demo generation

### 2.5 Output Reference

Main outputs:

- tL (direction), tR (direction): unit translation directions from recoverPose
- xL, yL, xR, yR: solved planar camera coordinates
- Stereo center (x, y): midpoint of left/right camera coordinates
- Residual norm: norm of equation residuals

Additional outputs in demo mode:

- Ground truth
- Abs error
- Oracle solve / Oracle abs error

### 2.6 Equations Used

Unknowns: xL, yL, xR, yR

1) tLy(xD - xL) - tLx(yD - yL) = 0

2) tRy(xD - xR) - tRx(yD - yR) = 0

3) (xL - xR)^2 + (yL - yR)^2 = B^2

4) yL - yR = 0

### 2.7 Practical Notes

- recoverPose translation is direction-only (scale ambiguous)
- Results can degrade under low texture, repeated patterns, or weak baseline geometry
- The minimal model is assumption-driven and may admit mirror/degenerate solutions

Recommended minimal upgrades:

1. Replace yL=yR with a known baseline heading constraint
2. Keep multi-start and residual-based branch validation
3. Use undistorted images and consistent exposure

---

## 3. Quick Command Summary

```bash
# 1) Create env
uv venv .venv

# 2) Install deps
uv pip install --python .venv/bin/python numpy scipy opencv-python

# 3) Run demo
.venv/bin/python main.py --demo --demo_out demo_data --seed 7
```

---

## 4. Open-Source Readiness Checklist

This repository now includes standard open-source project files:

- License: LICENSE (MIT)
- Build and package metadata: pyproject.toml
- Contribution guide: CONTRIBUTING.md
- Code of conduct: CODE_OF_CONDUCT.md
- Security policy: SECURITY.md
- Changelog: CHANGELOG.md
- Citation metadata: CITATION.cff
- Issue templates: .github/ISSUE_TEMPLATE/
- PR template: .github/pull_request_template.md
- Editor consistency: .editorconfig
- Ignore rules: .gitignore

### Maintainer Notes

- Replace placeholder repository links in pyproject.toml and CITATION.cff.
- Keep CHANGELOG.md updated for every user-visible change.
- Tag releases with semantic versions, for example v0.1.0.

---

## 5. Suggested Release Flow

1. Update CHANGELOG.md under Unreleased.
2. Run demo validation:
  .venv/bin/python main.py --demo --demo_out demo_data --seed 7
3. Bump version in pyproject.toml and CITATION.cff.
4. Commit with a release message.
5. Create a git tag and publish a release note.

This keeps documentation, code, and metadata consistent for external users.
