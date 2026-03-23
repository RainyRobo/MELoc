# MELoc — Stereo Planar Localization

<div align="center">

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![Python 3.10+](https://img.shields.io/badge/Python-3.10%2B-brightgreen.svg)](https://www.python.org/)
[![OpenCV](https://img.shields.io/badge/OpenCV-4.8%2B-orange.svg)](https://opencv.org/)

**Language** · [English](#overview) | [中文](#中文指南) 

</div>

Minimal stereo camera localization using planar geometric constraints.

---

## Quick Navigation

- [🇬🇧 English Guide](#overview)
- [🇨🇳 中文指南](#中文指南)

---

<a name="overview"></a>

## 📖 English Guide

### Overview

MELoc solves the camera localization problem on a plane using three input images (left, right, query) and known camera intrinsics. The method combines ORB feature matching, essential matrix decomposition, and planar geometric constraints to estimate the position of a stereo camera pair.

**Design philosophy:** Minimal implementation — no SLAM, visual odometry pipelines, or map optimization. Intended for resource-constrained embedded environments.

---

### 📋 Contents (English)

- [Installation](#installation-en)
- [Usage](#usage-en)
- [Parameters](#parameters-en)
- [Mathematical Formulation](#math-en)
- [Implementation Details](#implementation-en)
- [Limitations](#limitations-en)
- [License](#license-en)

---

<a name="installation-en"></a>

### 📦 Installation

#### Python Setup (Python 3.10+)

```bash
python -m venv .venv
source .venv/bin/activate
pip install numpy scipy opencv-python
```

#### Alternative: Using uv

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
uv venv .venv
uv pip install --python .venv/bin/python numpy scipy opencv-python
```

---

<a name="usage-en"></a>

### 🚀 Usage

#### Demo Mode

```bash
python main.py --demo --seed 7
```

Generates synthetic images (`left_demo.png`, `right_demo.png`, `query_demo.png`) and outputs:
- Translation directions $t_L, t_R$
- Camera coordinates $(x_L, y_L, x_R, y_R)$
- Stereo center $(x, y)$
- Residual norm
- Ground truth comparison

#### Real Data Mode

```bash
python main.py \
  --left left.png \
  --right right.png \
  --query query.png \
  --K 700,0,480,0,700,320,0,0,1 \
  --xD 1.4 \
  --yD 1.0 \
  --B 0.6
```

---

<a name="parameters-en"></a>

### 🔧 Parameters

| Parameter | Type | Description | Required |
|-----------|------|-------------|----------|
| `--left` | str | Left image path (grayscale) | Yes* |
| `--right` | str | Right image path (grayscale) | Yes* |
| `--query` | str | Query image path (grayscale) | Yes* |
| `--K` | str | Camera intrinsics (9 floats, row-major) | Yes |
| `--xD` | float | Query camera X coordinate | Yes |
| `--yD` | float | Query camera Y coordinate | Yes |
| `--B` | float | Stereo baseline length (m) | Yes |
| `--init` | str | Initial values `xL,yL,xR,yR` (default: 0,0,1,0) | No |
| `--demo` | flag | Enable demo mode | No |
| `--demo_out` | str | Output directory for demo (default: demo_data) | No |
| `--seed` | int | Random seed (default: 42) | No |

*In demo mode, image parameters are not required

---

<a name="math-en"></a>

### 📐 Mathematical Formulation

#### Problem Statement

Unknowns: $(x_L, y_L, x_R, y_R)$ — left and right camera coordinates on the plane

System of equations:

$$\begin{align}
(1) & \quad t_{Ly}(x_D - x_L) - t_{Lx}(y_D - y_L) = 0 \\
(2) & \quad t_{Ry}(x_D - x_R) - t_{Rx}(y_D - y_R) = 0 \\
(3) & \quad (x_L - x_R)^2 + (y_L - y_R)^2 = B^2 \\
(4) & \quad y_L - y_R = 0
\end{align}$$

Where:
- $(x_D, y_D)$ — known query camera position
- $t_L, t_R$ — translation directions from essential matrix decomposition
- $B$ — stereo baseline length
- Equation (4) assumes baseline is parallel to world X-axis

#### Camera Intrinsics Format

Camera matrix K is provided as 9 comma-separated values in row-major order:

$$K = \begin{pmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{pmatrix}$$

Example: `--K 700,0,480,0,700,320,0,0,1` represents $f_x = 700, c_x = 480, f_y = 700, c_y = 320$.

---

<a name="implementation-en"></a>

### ⚙️ Implementation Details

#### Algorithmic Pipeline

1. **Feature Extraction**: ORB (3500 keypoints)
2. **Feature Matching**: BFMatcher with KNN ratio test (threshold 0.75)
3. **Consistency Check**: Bidirectional matching validation
4. **Geometric Filtering**: F-matrix RANSAC (inlier threshold 1.0)
5. **Pose Estimation**: `cv2.findEssentialMat` + `cv2.recoverPose`
6. **Non-linear Optimization**: `scipy.optimize.least_squares` with Huber loss
7. **Solution Selection**: Multi-start initialization, validated by $x_R > x_L$

#### Robustness Mechanisms

- KNN ratio test removes ambiguous matches
- Bidirectional consistency reduces false positives
- RANSAC eliminates geometric outliers
- Huber loss downweights residual outliers
- Multi-start prevents local minima entrapment

---

<a name="limitations-en"></a>

### ⚠️ Limitations

1. **No Absolute Scale**: `recoverPose` returns direction only, not magnitude
2. **Texture Dependency**: Sparse features cause poor matching
3. **Parallax Sensitivity**: Small baseline leads to unstable direction estimation
4. **Planar Assumption**: Requires scene to satisfy planarity; multiple solutions possible
5. **Fixed Baseline Orientation**: Equation (4) assumes known baseline direction

#### Recommended Improvements

1. Add hard baseline direction constraint (replace equation 4)
2. Use multi-initialization with residual-based solution selection
3. Apply lens distortion correction to input images
4. Normalize image exposure before feature extraction
5. Fuse with inertial or odometry prior constraints

---

<a name="license-en"></a>

### 📄 License

MIT License — see [LICENSE](LICENSE)

---

---

<a name="中文指南"></a>

## 📕 中文指南

### 项目简介

MELoc 是一个轻量级的立体视觉平面定位系统。该方案基于平面几何约束，使用三张输入图像（左目、右目、查询）和已知的相机内参，来估计立体相机对的位置。

采用 ORB 特征匹配、本质矩阵分解和平面几何约束相结合的方法实现。

**设计理念：** 最小化实现 — 无 SLAM、视觉里程计管道或地图优化。专为资源受限的嵌入式环境设计。

---

### 📋 目录（中文）

- [环境配置](#环境配置)
- [使用方法](#使用方法)
- [参数说明](#参数说明)
- [数学模型](#数学模型)
- [实现细节](#实现细节)
- [已知限制](#已知限制)
- [许可证](#许可证-中文)

---

### 📦 环境配置

#### Python 环境（Python 3.10+）

```bash
python -m venv .venv
source .venv/bin/activate
pip install numpy scipy opencv-python
```

#### 替代方案：使用 uv

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
uv venv .venv
uv pip install --python .venv/bin/python numpy scipy opencv-python
```

---

### 🚀 使用方法

#### 演示模式

```bash
python main.py --demo --seed 7
```

自动生成合成图像（`left_demo.png`、`right_demo.png`、`query_demo.png`）并输出：
- 平移方向 $t_L, t_R$
- 相机坐标 $(x_L, y_L, x_R, y_R)$
- 双目中心 $(x, y)$
- 残差范数
- 真值对比

#### 实际数据模式

```bash
python main.py \
  --left left.png \
  --right right.png \
  --query query.png \
  --K 700,0,480,0,700,320,0,0,1 \
  --xD 1.4 \
  --yD 1.0 \
  --B 0.6
```

---

### 🔧 参数说明

| 参数 | 类型 | 说明 | 必需 |
|------|------|------|------|
| `--left` | str | 左目图像路径（灰度） | 是* |
| `--right` | str | 右目图像路径（灰度） | 是* |
| `--query` | str | 查询图像路径（灰度） | 是* |
| `--K` | str | 相机内参（9个浮点数，行主序） | 是 |
| `--xD` | float | 查询相机 X 坐标 | 是 |
| `--yD` | float | 查询相机 Y 坐标 | 是 |
| `--B` | float | 立体基线长度（米） | 是 |
| `--init` | str | 初值 `xL,yL,xR,yR`（默认：0,0,1,0） | 否 |
| `--demo` | flag | 启用演示模式 | 否 |
| `--demo_out` | str | 演示输出目录（默认：demo_data） | 否 |
| `--seed` | int | 随机种子（默认：42） | 否 |

*演示模式下不需要提供实际图像参数

---

### 📐 数学模型

#### 问题陈述

未知量：$(x_L, y_L, x_R, y_R)$ — 左右相机在平面上的坐标

方程组：

$$\begin{align}
(1) & \quad t_{Ly}(x_D - x_L) - t_{Lx}(y_D - y_L) = 0 \\
(2) & \quad t_{Ry}(x_D - x_R) - t_{Rx}(y_D - y_R) = 0 \\
(3) & \quad (x_L - x_R)^2 + (y_L - y_R)^2 = B^2 \\
(4) & \quad y_L - y_R = 0
\end{align}$$

其中：
- $(x_D, y_D)$ — 查询相机的已知位置
- $t_L, t_R$ — 从本质矩阵分解得到的平移方向
- $B$ — 立体基线长度
- 方程(4)假设基线平行于世界 X 轴

#### 相机内参格式

相机矩阵 K 以行主序的 9 个逗号分隔值提供：

$$K = \begin{pmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{pmatrix}$$

示例：`--K 700,0,480,0,700,320,0,0,1` 表示 $f_x = 700, c_x = 480, f_y = 700, c_y = 320$。

---

### ⚙️ 实现细节

#### 算法流程

1. **特征提取**：ORB（3500 个关键点）
2. **特征匹配**：BFMatcher + KNN 比值检验（阈值 0.75）
3. **一致性检查**：双向匹配验证
4. **几何筛选**：F 矩阵 RANSAC（内点阈值 1.0）
5. **位姿估计**：`cv2.findEssentialMat` + `cv2.recoverPose`
6. **非线性优化**：`scipy.optimize.least_squares` + Huber 损失函数
7. **解选择**：多初值启动，通过 $x_R > x_L$ 验证

#### 鲁棒性机制

- KNN 比值检验去除歧义匹配
- 双向一致性检查减少假正样本
- RANSAC 消除几何外点
- Huber 损失函数抑制残差外点
- 多初值避免局部最小值

---

### ⚠️ 已知限制

1. **无绝对尺度**：`recoverPose` 仅返回方向，不包含大小
2. **纹理依赖性**：稀疏特征导致匹配不佳
3. **视差敏感性**：小基线导致方向估计不稳定
4. **平面假设**：场景必须满足平面约束，可能存在多个解
5. **固定基线方向**：方程(4)假设基线方向已知

#### 改进建议

1. 添加基线方向硬约束（替换方程4）
2. 使用多初值启动并通过残差选择最优解
3. 对输入图像进行透镜畸变校正
4. 在特征提取前进行图像曝光归一化
5. 融合惯性或里程计先验约束

---

<a name="许可证-中文"></a>

### 📄 许可证

MIT 许可证 — 详见 [LICENSE](LICENSE)

---

<div align="center">

**Copyright © 2025 MELoc Contributors**

</div>
