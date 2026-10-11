# Task: 修复双轴机动耦合导致的 Yaw 漂移

## 项目背景

- 平台：英飞凌 **CYT4BB7**（Seekfree CYT4BB 开源库），双核 Cortex-M7
- IMU：**IMU660RB**，6 轴（加速度计 + 陀螺仪），**无磁力计、无 GPS**
- 姿态解算：Mahony 互补滤波 + 四元数，在 [project/code/quaternion.c](project/code/quaternion.c)
- 关键函数：`my_quaternion()`（[quaternion.c:47](project/code/quaternion.c#L47)）
- **调用频率：500 Hz**（在 [project/user/cm7_0_isr.c:52](project/user/cm7_0_isr.c#L52) 的 ISR 里调用）
- **dt = 0.002s**（[project/user/main_cm7_0.c:83](project/user/main_cm7_0.c#L83)：`quaternion_Init(..., 2.5, 0.001, 0.002f)`，kp=2.5, ki=0.001）
- 量程：陀螺 `GYRO_SCALE = 14.3` LSB/(°/s)（推断为 ±2000°/s），加速度 `ACC_SCALE = 4098` LSB/g

代码根目录：`E:\CYT4bb7Libraryhere1\CYT4BB7_Library-master\CYT4BB7_Library-master\Seekfree_CYT4BB_Opensource_Library\`

---

## 问题

**症状**：
- 单独动 roll、单独动 pitch → yaw **稳定，不飘**
- **roll 和 pitch 同时晃动** → yaw 出现明显漂移
- 偶尔抖动一两次 → 没事
- **频繁抖动 → 概率性出现 yaw 飘走且静置后不回来**

**物理本质**（务必先理解再动手）：

1. **加速度计 PI 反馈通过 dq3 污染 q3（yaw）**
   - Mahony 算出的误差 `ex = ay*ckz - az*cky`、`ey = az*ckx - ax*ckz` 通过 PI 加到 gx、gy 上
   - 四元数微分方程里 [quaternion.c:178](project/code/quaternion.c#L178)：
     ```
     dq3 = (q0*gz + q1*gy - q2*gx) * halfT
     ```
   - **gx 和 gy 直接出现在 dq3 里**。机动期间加速度计被线性加速度污染，PI 把错误加到 gx/gy，然后污染 q3 → yaw 偏移
   - **单轴动**：另一轴 g 接近 0，污染弱
   - **双轴同时动**：gx 和 gy 都有量，污染叠加，q3 偏移明显

2. **Coning Error（圆锥误差）**
   - 三轴旋转不可交换。roll/pitch 同时周期性摆动时，一阶 Euler 积分四元数会在 yaw 方向产生**净直流分量**
   - 当前 [quaternion.c:175-178](project/code/quaternion.c#L175-L178) 是一阶 Euler 积分（注释里写"龙格库塔法"但实际不是），频率虽然 500 Hz 但仍会累积
   - 这就是为什么"偶发抖动没事，频繁抖动概率出问题"——单次 coning error 量小，多次累积才显著

3. **6 轴系统 yaw 无绝对参考**
   - 重力只能矫正 roll/pitch，**yaw 一旦被污染没办法自动回到原始航向**
   - **物理上能做到的最好结果**：机动期间允许漂移，但机动结束后通过静止检测**锁定 yaw**（ZUPT, Zero-Update Technique），让它不再继续累积

## 用户的预期

> **"机动期间 yaw 可以飘，但机动结束后能靠静止检测修正一部分。"**

也就是说，**目标不是"yaw 永不飘"**（物理上做不到），而是：
- 单轴/低强度运动 → yaw 不应被污染（这点目前已经基本做到）
- 双轴同时机动 → yaw 漂移幅度应**显著减小**
- 机动结束后回到水平静置 → yaw 应该**停止漂移并锁定**，不能继续累积；如果能通过静止检测识别出"机动期间的虚假漂移"（vs. 真实的 bias 偏移）并部分修正，更好

---

## 你的任务

**在 [project/code/quaternion.c](project/code/quaternion.c) 现有的 Mahony 框架基础上修改**，解决双轴耦合 yaw 漂移问题。

工作流程：

1. **完整通读** [project/code/quaternion.c](project/code/quaternion.c)（特别是 `my_quaternion()`）和 [project/code/quaternion.h](project/code/quaternion.h)
2. **核对**下面"当前代码状态"清单，确认你看到的代码确实是这个版本（用户最近改过，回退了上一轮的几个机制）
3. **设计方案**：写一段简短的方案说明（思路、改动点、预期效果），通过 chat 输出给我
4. **改代码**：直接修改 [project/code/quaternion.c](project/code/quaternion.c)，必要时改 [project/code/quaternion.h](project/code/quaternion.h)
5. **改完简述**：每处改动写一行中文注释说明"为什么改"，并在 chat 中总结

---

## 当前代码状态（用户最近回退/简化过，请确认）

| 位置 | 当前状态 | 注意 |
|---|---|---|
| [L61-63](project/code/quaternion.c#L61-L63) | `ff_gx = -imu660rb_gyro_x` 等，**没有减 gyro_offset** | **开机 imu_calibration 算出来的 offset 完全没用上**！bias 全靠死区压 |
| [L66-68](project/code/quaternion.c#L66-L68) | 三轴死区 5.0 raw（≈0.35°/s） | 静态 bias 抑制依赖这个死区 |
| [L70-71](project/code/quaternion.c#L70-L71) | 低通 kk=0.15, kk1=0.15 | 滤波相对强 |
| [L107](project/code/quaternion.c#L107) | `if(fabsf(gz) < 0.1f) gz = 0.0f;`（**弧度**，≈5.7°/s） | yaw 弧度死区，比较激进 |
| [L120](project/code/quaternion.c#L120) | `if(fabsf(norm - 1.0f) < 0.2f)` 才启用加速度计 | **窗口偏宽松**，双轴机动时仍易满足 |
| [L160-162](project/code/quaternion.c#L160-L162) | motion_hold 机制**已注释掉** | 高角速度时不再降 kp |
| [L168-169](project/code/quaternion.c#L168-L169) | `gz = gz`（yaw 无 PI 反馈） | 6 轴下无磁力计，这是正确的 |
| 整体 | **没有静止检测、没有在线 bias 估计、没有 ZUPT、没有 coning 补偿** | 这些都是这次要加的方向 |

---

## 推荐的解决方向（按优先级）

按"投入 vs. 收益"排序，建议你**优先做前 3 个**：

### 1. ★★★ 收紧加速度计有效性窗口 + 双轴运动专门屏蔽

当前 [L120](project/code/quaternion.c#L120) `|norm-1| < 0.2` 太宽松。双轴机动时线性加速度叠加重力，模长经常仍在 0.8~1.2 之间，触发 PI 反馈污染 yaw。

建议改为：
- 模长窗口收紧到 `|norm-1| < 0.05`（约 ±0.5 m/s²）
- **额外加双轴运动检测**：当 `|gx| 和 |gy|` **同时**超过某阈值（如各 0.3 rad/s ≈ 17°/s）时，强制 `ex=ey=0` 并冻结 `exInt/eyInt`（不要继续累积，否则机动结束后积分项还有错误残留）

### 2. ★★★ 加静止检测 + ZUPT 锁定 yaw

500 Hz 下持续 200ms（100 tick）满足以下条件认为"静止"：
- `|ff_gx| < 5`、`|ff_gy| < 5`、`|ff_gz| < 5`（raw）
- `|norm - 1| < 0.03`

静止时：
- **冻结 q3 的更新**（dq3 不加到 siyaun->q3，或者用极小的衰减让 q3 趋于稳定值）
- 同时在线更新 `gyro_offset_x/y/z`（用 IIR：`offset += (raw - offset) * alpha`，启动期 alpha=0.05、稳态 alpha=0.001）
- 让 [L61-63](project/code/quaternion.c#L61-L63) 改成 `ff_gx = -(imu660rb_gyro_x - gyro_offset_x)`，**把开机校准和在线 bias 估计都启用起来**

### 3. ★★ Coning 补偿（一阶）

在四元数积分前加一项 coning 补偿。最简单的一阶形式：保存上一拍角速度 `(gx_prev, gy_prev, gz_prev)`，叠加一项叉积修正：

```
gx_eff = gx + (1/12) * (gy_prev*gz - gz_prev*gy)
gy_eff = gy + (1/12) * (gz_prev*gx - gx_prev*gz)
gz_eff = gz + (1/12) * (gx_prev*gy - gy_prev*gx)   // 这一项是 coning 在 yaw 上的反向补偿
```

用 `gx_eff/gy_eff/gz_eff` 代替 `gx/gy/gz` 进入四元数积分。

### 4. ★ 升级为二阶积分（中点法）

当前一阶 Euler 在 500Hz 下其实精度够，但中点法会进一步减小积分误差。如果做了 #3，这个可以不做。

### 5. ★ 重新加回 motion_hold

机动检测后保持低 kp 一段时间（如 200ms），避免机动尾部残余振动被高 kp 放大。注意：如果做了 #1 的"双轴运动屏蔽"，这条收益变小，可以不做。

---

## 约束

- **不要重构无关代码**，最小改动为先
- **不要引入新硬件依赖**（没有磁力计 / GPS）
- 嵌入式 MCU 环境：**避免 double**、避免动态内存、注意浮点开销
- 中文注释风格保留即可，新加的注释用中文
- 不要动 [project/code/ztjx.c](project/code/ztjx.c) 的 EKF 方案（那是另一套方案）
- 改完后用 chat 简述每处改动的**理由**和**预期效果**——不要写新的文档文件

## 验收场景

我会在硬件上测试以下场景：

1. **静态**：水平静置 60 秒，yaw 漂移 < 1°
2. **单轴机动**：单独 roll 或单独 pitch 摆动 ±45° × 10 次，yaw 残余偏差 < 2°
3. **双轴同步机动**：roll 和 pitch **同时** ±45° 圆锥运动 10 次后静置 5 秒，yaw 残余偏差 < 5°（当前可能是 20°+）
4. **频繁双轴机动**：双轴圆锥运动连续 30 秒，期间 yaw 可以飘，但**停止后 5 秒内 yaw 必须停止累积**（这是 ZUPT 验收点）

## 相关上下文

- [project/code/quaternion.c](project/code/quaternion.c) — **主目标文件**
- [project/code/quaternion.h](project/code/quaternion.h) — 结构体定义
- [project/user/cm7_0_isr.c:52](project/user/cm7_0_isr.c#L52) — `my_quaternion` 调用点（ISR，500Hz）
- [project/user/main_cm7_0.c:83](project/user/main_cm7_0.c#L83) — kp/ki/dt 初始化参数
- [project/code/ztjx.c](project/code/ztjx.c) — 另一套 EKF（**本任务不动**），里面有一些 IMU 数据处理可参考
- [project/code/attitude_control.c](project/code/attitude_control.c) — 姿态控制环（参考用，**本任务不动**）

---

## 注意事项 / 易踩坑

1. `ff_gx/ff_gy/ff_gz` 是 **raw LSB 单位**（除以 `GYRO_SCALE=14.3` 才是 °/s）。死区数值和单位一定要对齐
2. `gx/gy/gz` 在 [L101-103](project/code/quaternion.c#L101-L103) 之后是**弧度/秒**
3. `imu_calibration()`（[L203](project/code/quaternion.c#L203)）写了 `gyro_offset_x/y/z` 但**主路径没用它**——要么删，要么在 `ff_g*` 计算里把它用起来
4. 注意 [L120](project/code/quaternion.c#L120) 的 `norm` 是**归一化前**的模长（即使 if 分支内除以了 norm，变量本身不变），后续静止检测复用这个变量是 OK 的
5. `imu660rb_gyro_x/y/z` 是 `int16` raw 值，参与浮点运算时注意先转 `(float)`
6. 别把 `kp` 设成 0 — 那样会完全失去加速度计修正，长期 roll/pitch 也会漂
