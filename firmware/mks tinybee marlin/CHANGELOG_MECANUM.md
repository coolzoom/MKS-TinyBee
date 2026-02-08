# 麦克纳姆轮底盘固件修改日志 (Mecanum Robot Base)

基于 MKS TinyBee Marlin 固件，将 XYZA 四轴映射为麦克纳姆轮小车四个轮子，并实现 `firmware/robotbase/README.md` 描述的串口控制协议。底层仍使用 Marlin，仅做最小化修改。

---

## 修改日期

2026-02-08

---

## 1. Configuration.h

### 1.1 线性轴数

- **LINEAR_AXES**：由注释改为 `#define LINEAR_AXES 4`，使 X、Y、Z、A 四个轴对应四个轮子。

### 1.2 行程限制取消（MECANUM_ROBOTBASE）

- 新增 **MECANUM_ROBOTBASE** 宏，用于启用麦克纳姆底盘模式。
- 在 `MECANUM_ROBOTBASE` 下：
  - **X/Y/Z/I (A) 行程**：设为 `-10000` ~ `10000` mm，等效取消行进距离限制。
- 未启用时保持原有 `X_MIN_POS`/`X_MAX_POS` 等（0、X_BED_SIZE 等）。

### 1.3 I 轴 (A 轴) 配置

- **I_ENABLE_ON**：在 `LINEAR_AXES >= 4` 时启用，设为 `0`。
- **DISABLE_I**：在 `LINEAR_AXES >= 4` 时启用，设为 `false`。
- **INVERT_I_DIR**：在 `LINEAR_AXES >= 4` 时启用，设为 `false`。
- **I_HOME_DIR**：在 `LINEAR_AXES >= 4` 时启用，设为 `-1`。
- **I_MIN_POS / I_MAX_POS**：在 MECANUM 模式下设为 `-10000` / `10000`。

### 1.4 步进与速度参数（四轴 + E）

- **DEFAULT_AXIS_STEPS_PER_UNIT**：由 `{ 80, 80, 1600, 400 }` 改为 `{ 80, 80, 1600, 80, 400 }`（增加 I 轴 80，与 X/Y 轮一致）。
- **DEFAULT_MAX_FEEDRATE**：增加第 4 轴，改为 `{ 300, 300, 5, 300, 25 }`。
- **DEFAULT_MAX_ACCELERATION**：增加第 4 轴，改为 `{ 200, 200, 50, 200, 500 }`。
- **HOMING_FEEDRATE_MM_M**：由 3 个改为 4 个，`{ (50*60), (50*60), (4*60), (50*60) }`，对应 XYZA。

---

## 2. 引脚 (pins_MKS_TINYBEE.h)

- 在 **ENABLED(MECANUM_ROBOTBASE)** 时，将第四轴 (I/A) 映射到 E1 驱动：
  - **I_STEP_PIN** = 141  
  - **I_DIR_PIN** = 142  
  - **I_ENABLE_PIN** = 140  

即使用 E1 步进驱动作为第四个轮子。

---

## 3. 新增 feature：mecanum_robotbase

### 3.1 头文件 `src/feature/mecanum_robotbase.h`

- 声明 `bool process_robotbase_command(char *command);`
- 仅在 `MECANUM_ROBOTBASE` 启用时参与编译。

### 3.2 实现 `src/feature/mecanum_robotbase.cpp`

- **协议**：实现 README 中的机器人底盘协议（115200，换行结束，先回 ACK）。
- **支持命令**：
  - **基础运动**：`F`、`B[速度]`、`L[速度]`、`R[速度]`、`SL[速度]`、`SR[速度]`
  - **距离控制**：`FD[距离]`、`BD[距离]:[速度]`、`SDL[距离]`、`SDR[距离]:[速度]`、`LD[角度]`、`RD[角度]:[速度]`
  - **特殊**：`S` 立即停止（最高优先级）
  - **状态**：`STATUS` 返回 `stepflage, isSerialControlled, isDistanceControlled, currentSpeed, isRaytracingEnabled, isCentered`
  - **光线追踪**：`FINDRAY`、`DISABLERAY`（状态保存，逻辑可扩展）
  - **识别**：`ID`、`?` 回复 `robotbase`（不先回 ACK）
- **运动实现**：将上述命令转换为 Marlin 相对运动：
  - 使用 `G91` → `G1 X Y Z A F...` → `G90`，通过 `queue.enqueue_one()` 注入。
  - 连续运动（F/B/L/R/SL/SR）：注入 10000 mm 长位移，由 `S` 或 quickstop 停止。
  - 定长/定角（FD/BD/SDL/SDR/LD/RD）：注入对应位移后 `planner.synchronize()` 等待完成再回 `ok`。
- **麦克纳姆映射**（与 robotbase.ino 逻辑一致）：
  - 前进: (-d, +d, -d, +d)；后退: (+d, -d, +d, -d)
  - 左平移: (-d, -d, +d, +d)；右平移: (+d, +d, -d, -d)
  - 左转: (+d, +d, +d, +d)；右转: (-d, -d, -d, -d)
- **参数**：默认速度 50 mm/s，速度 0–200，距离 0–10000 mm，角度 0–360°；横向使用系数 0.7071；旋转半径 150 mm（可配置）。

---

## 4. 串口命令拦截 (queue.cpp)

- 在 **get_serial_commands()** 中，在跳过前导空格后、G 码行号/校验和处理前：
  - 若 `ENABLED(MECANUM_ROBOTBASE)` 且 **process_robotbase_command(command)** 返回 `true`，则本行视为已处理，**不再入队**，直接 `continue`。
- 这样与 README 一致的单行命令（F、B、S、STATUS 等）走底盘协议，其余仍走 Marlin G 码。

---

## 5. 与 robotbase.ino 的关系

- **逻辑参考**：麦克纳姆方向与运动类型参考 `firmware/robotbase/robotbase.ino`（前后/左右平移/旋转的轮子符号与系数）。
- **实现方式**：不沿用该 .ino 的定时器/中断步进，而是保留 Marlin 为底层，通过 G91/G1/G90 与 planner 实现相同运动效果。

---

## 6. 使用说明

- 串口：**250000** 波特率，换行符 `\n` 结束。（与 Configuration.h 中 BAUDRATE 一致；若需 115200 可在 Configuration.h 中改回。）
- 每条命令先回 `ACK`（除 `ID`/`?` 只回 `robotbase`）。
- 停止用 `S`，可随时中断运动。
- 距离/角度命令在执行完成后回 `ok`。

---

## 7. PlatformIO 编译修复（2026-02-08）

为通过 `pio run -e mks_tinybee` 编译，额外修改：

### Configuration.h
- **USE_IMIN_PLUG**：在 `LINEAR_AXES >= 4` 时启用，满足 SanityCheck 对 I 轴回零到 MIN 的要求。

### Configuration_adv.h
- **HOMING_BUMP_MM**：由 3 元组改为 4 元组 `{ 5, 5, 2, 5 }`（对应 XYZA）。
- **HOMING_BUMP_DIVISOR**：由 3 元组改为 4 元组 `{ 2, 2, 4, 2 }`。
- **AXIS_RELATIVE_MODES**：由 4 元组改为 5 元组 `{ false, false, false, false, false }`（X Y Z I E）。

### pins_MKS_TINYBEE.h
- 在 MECANUM_ROBOTBASE 块中增加 **I_MIN_PIN -1**、**I_MAX_PIN -1**，表示无物理限位，I 轴不参与回零。

### mecanum_robotbase.cpp
- 删除未使用的局部变量 `dx, dy, dz, da`，消除编译警告。

### 波特率
- **BAUDRATE**：由 115200 改为 **250000**（Configuration.h）。
- **BAUDRATE_2**：由 115200 改为 **250000**（第二串口）。

### 麦克纳姆模式下不执行回零、不设限位
- **G28（回零）**：在 `MECANUM_ROBOTBASE` 下，G28 不执行任何物理回零动作；仅将当前逻辑位置视为原点（`set_axis_is_at_home`），同步规划器后直接返回。
- **软限位**：在 `apply_motion_limits()` 中，若 `MECANUM_ROBOTBASE` 则直接 return，不施加任何轴限位。
- **上电状态**：在 `setup()` 中，若 `MECANUM_ROBOTBASE`，将所有线性轴标记为“已回零”，上电后无需执行 G28 即可运动。

---

## 8. 文件清单

| 文件 | 修改类型 |
|------|----------|
| `Marlin/Configuration.h` | 修改：LINEAR_AXES=4、行程、I 轴、steps/feedrate/accel/homing、USE_IMIN_PLUG、BAUDRATE 250000、**I_DRIVER_TYPE A4988**（四轴时启用） |
| `Marlin/Configuration_adv.h` | 修改：HOMING_BUMP_MM、HOMING_BUMP_DIVISOR、AXIS_RELATIVE_MODES |
| `Marlin/src/pins/esp32/pins_MKS_TINYBEE.h` | 修改：I 轴引脚及 I_MIN_PIN/I_MAX_PIN=-1 |
| `Marlin/src/feature/mecanum_robotbase.h` | 新增 |
| `Marlin/src/feature/mecanum_robotbase.cpp` | 新增 |
| `Marlin/src/gcode/queue.cpp` | 修改：包含头文件并调用 process_robotbase_command |
| `Marlin/src/gcode/calibrate/G28.cpp` | 修改：MECANUM 下 G28 不执行回零，仅设当前为原点 |
| `Marlin/src/module/motion.cpp` | 修改：MECANUM 下 apply_motion_limits 直接 return |
| `Marlin/src/MarlinCore.cpp` | 修改：MECANUM 下 setup() 中标记各轴已回零 |
| `CHANGELOG_MECANUM.md` | 新增（本文件） |
