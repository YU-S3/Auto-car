# Soberup 2026 N30pro 代码总结

> 最近核对日期: 2026-08-12
> 项目: 第21届智能车竞赛 — 疯狂电路组
> 平台: Infineon TC264D (TriCore) + MT9V03X 灰度摄像头
> 开发环境: ADS v1.10.2 (Tasking 编译器)
> 开源归档说明: 本文保留原始完整技术说明；其中测试、调试产物和路径生成脚本属于历史研发材料，未随本公开仓库发布。

---

## 一、项目概述

本项目是参加**第21届全国大学生智能汽车竞赛——疯狂电路组**的智能车控制程序。赛道由白色胶带在蓝底喷绘布上铺设，模拟"电路图"引导线，元器件符号（MOS管、晶闸管、三极管）构成三叉路口节点。

**核心任务**: 车模沿白色引导线自主行驶，从普通元器件干扰图案中识别真正的三叉节点，并按预定路线正确通过，避免遗漏目标元器件。

**成绩公式**: 用时 + 遗漏元器件罚分（每个30s）

---

## 二、硬件平台

| 组件 | 型号/说明 | 关键引脚 |
|------|-----------|----------|
| MCU | Infineon TC264D (双核 TriCore) | CPU0: 控制调度, CPU1: 图像处理 |
| 摄像头 | MT9V03X 总钻风 (灰度, 188×120) | PCLK=P02.1, VSY=P02.0, DATA=P00.0~P00.7 |
| IMU | IMU660RC (SPI0，陀螺仪默认±4000°/s) | CLK=P20.11, MOSI=P20.14, MISO=P20.12, CS=P20.13 |
| 显示屏 | IPS114 (SPI2, 135×86) | SCL=P15.3, SDA=P15.5, RST=P15.1, DC=P15.0, CS=P15.2 |
| 无线串口 | UART2 (115200) | TX=P10.5, RX=P10.6, RTS=P33.8 |
| 电机驱动 | DRV8245-Q1 (HW VQFN) | 左: DIR=P33.11, PWM=ATOM3_CH4_P33.12 |
|  |  | 右: DIR=P33.9, PWM=ATOM3_CH0_P33.10 |
| 编码器 | 增量式 ×2 | 左: TIM4 (P02.8/P33.5), 右: TIM6 (P20.3/P20.0) |
| 负压风扇 | 无刷电调 50Hz PWM | ATOM0_CH0_P22.1 |
| NSLEEP | P33.6 (输出) / P33.7 (按键输入) | 电机驱动休眠控制 |
| 蜂鸣器 | P33.10 |  |
| 电池检测 | ADC0_CH11_A11 | 分压检测 |
| LED指示 | LED1=P00.9, LED2=P00.12, LED3=P21.5, LED4=P21.4 | 状态指示 |
| 按键 | UP=P02.6, DOWN=P02.5, LEFT=P02.7, RIGHT=P02.4, PRESS=P00.8 | 菜单操作 |

**禁止使用的引脚**: P14.2~P14.6, P10.5, P10.6 (Boot引脚), P20.2 (仅输入), P21.6 (TC264DA不可用)

**当前引脚冲突**: `buzzer.c` 把蜂鸣器定义为 P33.10，而右电机 PWM 也使用 P33.10。蜂鸣器模块当前没有调用者；不能在现有电机配置下再初始化或驱动该蜂鸣器引脚。

---

## 三、软件架构

### 3.1 双核分工

```
CPU0 (core0_main):
├─ 系统初始化 (时钟/调试/无线模块)
├─ 菜单系统 & Flash参数管理
├─ 定时器中断:
│   ├─ PIT CH0 (2ms): 轨迹记录 + combined_control(速度环/惯导每次执行，转向每2次执行)
│   └─ PIT CH1 (2ms): Mahony姿态解算 + 独立连续yaw梯形积分 + 本趟GZ峰值记录
├─ 外部中断: 摄像头VSYNC/ToF/按键
├─ 串口中断: 无线模块接收/调试输出
└─ 主循环: 计时停车 + 仅Record停车后的保存提示 + 本趟PWM/里程重定位诊断屏显 + 无延时周期无线发送

CPU1 (core1_main):
├─ 等待 start_flag
└─ 主循环 (摄像头帧同步):
    ├─ route_progress冻结本帧里程、选择事件阶段和配置索引
    ├─ OTSU二值化
    ├─ 虚线补线 (Dashedline_Makeup)
    ├─ 8邻域双爬虫寻线 (search_line_main)
    ├─ 状态更新 (state_update: 岔路/元素检测)
    └─ 路径偏差计算 (caculate_error)
```

### 3.2 中断优先级分配

| 中断 | 优先级 | 说明 |
|------|--------|------|
| DMA (摄像头) | 60 | 摄像头采集完成 |
| ERU CH3/CH7 (VSYNC) | 43 | 摄像头场中断 |
| ERU CH0/CH4 | 40 | 外部中断 |
| CCU6_0 CH1 (陀螺仪) | 31 | Mahony姿态更新、连续yaw与GZ峰值 |
| CCU6_0 CH0 (控制) | 30 | 2ms主控制循环 |
| UART2 (无线) | 16~18 | 无线串口收发 |

---

## 四、目录结构

```
Soberup_2026_N30pro/
├── CLAUDE.md                  # AI 助手指令
├── CODE_SUMMARY.md            # 本文件 - 代码总结
├── CHANGELOG.md               # 版本更新日志
├── thresholds.md              # 阈值修改追踪记录
├── path.py                    # 四种固定路线展开、覆盖校验、数组与路线图导出脚本
├── path_routes_basic.py       # 四种跑法基础节点顺序/长度/图片输出脚本
├── path_solver.c              # C 版精确闭环路径求解器（不画图）
├── code/                      # ★ 项目核心代码
│   ├── ADRC.c/h               # LADRC 自抗扰速度控制器
│   ├── battery_detect.c/h     # 电池电压检测
│   ├── buzzer.c               # 蜂鸣器驱动
│   ├── dashedline_makeup.c/h  # 虚线补线算法
│   ├── element_line.c/h       # 赛道元素检测 & 岔路出口校验
│   ├── encoder.c/h            # 编码器初始化、采样与引脚定义
│   ├── image.c/h              # 图像二值化 (OTSU)
│   ├── inertial_navigation.c/h # 里程计、区间聚合打点、轨迹Flash校验、速度表与路标重定位
│   ├── initialize.c/h         # 系统与控制中断初始化
│   ├── LED.c/h                # LED 指示灯
│   ├── line.c/h               # 路径偏差计算 & 岔路序列
│   ├── maze.c/h               # 8邻域双爬虫寻线算法
│   ├── menu.h                 # 菜单兼容转发头，实际接口位于 menu/menu.h
│   ├── menu/                  # 表驱动层级菜单（静态分配，无动态内存）
│   │   ├── menu.c/h           # 对外入口、按键扫描、保存提示与冻结Flash常量
│   │   ├── menu_nav.c/h       # 页面栈、光标、滚动和数值编辑状态机
│   │   ├── menu_tree.c/h      # Set/PRM/IMG/SUB/Path/Cal/MOT描述表与动作适配
│   │   ├── menu_render.c/h    # IPS114通用列表、状态与数值高亮绘制
│   │   ├── menu_pages.c/h     # Set、IMG、陀螺校准、编码器特殊页面
│   │   └── menu_params.c/h    # 110槽参数页版本升级、兼容加载与分组读改写
│   ├── motor.c/h              # 电机控制 & 综合控制
│   ├── state_control.c/h      # 状态机 (normal/fork_turn/fork_straight)
│   ├── map_event.c/h           # 地图事件: Path_Memory→事件序列解析
│   ├── route_progress.c/h      # 逻辑三叉合并、里程窗口和状态裁决命令
│   ├── run_features.c/h        # Record/Speed/Fork/DRS四个RAM运行开关及互斥/依赖关系
│   ├── route_config.c/h        # 预赛/决赛正反四套路表、100项共享容量与安全查表接口
│   ├── trajectory_policy.c/h   # 单字节元素编解码与历史/实时error速度上限策略
│   ├── tuning_presets.c/h      # 5组转向/速度/前视/出口回退调参预设及Flash序列化
│   ├── tool.c/h               # PID/LTD/工具函数
│   └── 本文件夹作用.txt
├── user/                      # 用户入口文件
│   ├── cpu0_main.c/h          # CPU0 主程序
│   ├── cpu1_main.c            # CPU1 主程序
│   ├── isr.c/h                # 中断服务函数
│   └── isr_config.h           # 中断优先级配置
├── tests/                     # Python静态契约与算法模型测试
│   ├── test_route_progress_contract.py # 三叉窗口、帧顺序、功能互斥与参数布局契约
│   ├── test_route_selection_contract.py # SUB四项、方向持久化与路线消费者契约
│   └── test_fork_yaw_protection_contract.py # 角度观测器表、实际叉数与偏航保护契约
├── libraries/                 # 第三方库
│   ├── infineon_libraries/    # Infineon iLLD (TC26B)
│   ├── zf_common/             # Seekfree 公共层 (时钟/调试/FIFO等)
│   ├── zf_driver/             # Seekfree 外设驱动 (ADC/PWM/GPIO/UART等)
│   ├── zf_device/             # Seekfree 设备驱动 (摄像头/IMU/屏幕等)
│   └── zf_components/         # Seekfree 应用组件 (上位机通信)
├── record/                    # 调试记录 & 分析文档
│   ├── 寻线算法代码说明.md
│   ├── 赛道元素认知.md
│   ├── 速度环工况分析.md
│   ├── 转向微分项说明.md
│   └── 图片视觉分析报告.md
└── Debug/                     # 编译产物
```

---

## 五、核心算法详解

### 5.1 图像处理流水线

```
MT9V03X 灰度图 (188×120)
    │
    ▼
局部自适应阈值二值化 (image.c)
    │  - 6列×4行网格分块, 稀疏采样 (x步长4, y步长2) 建 32-bin 直方图
    │  - 中间4列 (x≈31~156) 汇总为全局直方图 → OTSU 求全局基准阈值
    │  - 可信度: 类间差 <30 则沿用上一帧阈值 (帧间 EMA 平滑 α=0.25)
    │  - 每块: 25% 分位/最暗峰 与全局参考作差 → 局部补偿 raise (夹到 [0,30])
    │  - 异常亮块(>30)标记无效, 用曼哈顿最近有效块的中位数修复; 全无效回填上帧
    │  - 二值化阈值 = 全局基准 + 纵向插值后的 raise, lt.bu 一次比较 4 像素
    │
    ▼
虚线补线 (dashedline_makeup.c)
    │  - Fork矫正开启时按route_progress配置索引补线，窗口外为FILL_NONE
    │  - Fork矫正关闭时继续按原fork_num查表
    │  - 横向补线: 扫描每行, 白色段间黑间隙 ≤ 预期线宽*1.5 → 填 Make_up(128)
    │  - 纵向补线: 列扫描, 黑间隙 ≤ 5cm等效像素 → 填 Make_up(128)
    │
    ▼
8邻域双爬虫寻线 (maze.c)
```

### 5.2 8邻域双爬虫寻线 (maze.c)

核心函数: `findline_twoside(Point L, Point R, *Lnum, *Rnum, MAX_step)`

**左爬虫** (沿白色左边缘):
- 前方=黑 → 右转 (dir+1)
- 左前=黑 → 直行
- 左前=白 → 左前移 + 左转 (dir+3)

**右爬虫** (沿白色右边缘):
- 前方=黑 → 左转 (dir+3)
- 右前=黑 → 直行
- 右前=白 → 右前移 + 右转 (dir+1)

**停止条件**:
- Y坐标越界 (y<10 或 y≥120)
- 打圈超限 (turn≥4)
- 步数超限 (step≥MAX_step-2)
- 触边界 (x≤5 或 x≥W-5 且 step>80)
- 几何回爬检测: 相对原始起爬行先向上推进≥20px，随后连续4个移动点满足 `y >= start_y - 5`（已删除像素回爬计数）
- 左右爬虫相遇: 欧氏距离 ≤ `TOUCH_DISTANCE_PX=2` (touch=1)

**边界出口记录**: 三层边界 (进入检测 x≤10、记录层 x≤13/11 左, x≥W-10/W-13/W-11 右, y≤15 上), 记录首次触达的坐标和步数

**烂线检测与恢复**:
- 首尾欧氏距离 <20px 或 <总步数*0.3 → 烂线
- “先向上推进、再回到底部起爬高度”的几何回爬标志 → 烂线（像素回爬与 `visited` 数组已删除，统一以几何回爬判定）
- 十字边线重建继续使用无几何守卫的公共爬线接口
- 单侧烂线时从好侧原始起点找回坏侧起点重爬；恢复时重置几何状态，并用同一判定统一复检左右两侧
- 直行与斜向移动统一采用“先写当前点、再增加点数”，避免斜向分支留下数组空洞或旧点

**距离趋势分析**: 取最后30对左右边缘点做线性回归 → 斜率(slope)用于岔路判断

### 5.3 岔路检测与状态机 (state_control.c)

**状态枚举**:
- `normal`: 正常巡线
- `fork_straight`: 直行通过三叉
- `fork_turn`: 转弯通过三叉

`line_state` 是独立的直角元素状态机（`none/angle`）；只有 `car_state==normal` 时才允许进入直角，三叉状态会中止当前直角且不增加 `angle_num`。

**每帧执行顺序**:
1. `route_progress_begin_frame()` 在CPU1帧开头读取一次里程，确定逻辑三叉、窗口阶段和配置索引。
2. OTSU二值化后，`Dashedline_Makeup()` 使用本帧配置索引补线；Fork开启且当前事件未到窗口时使用 `FILL_NONE`。
3. 双爬虫和 `count_valid_exits()` 完成通用视觉识别，并只接受“左+上、上+右、左+右”三种有序穿出边界对。
4. `state_update()` 将视觉条件与里程命令合并，最终决定拒绝、视觉进入、强制进入、保持、正常退出或强制退出。
5. `caculate_error()` 使用本帧已经裁决完成的三叉状态和同一配置索引计算偏差。

入口视觉重定位发生在状态裁决阶段；它会立即更新全局里程和轨迹索引，但本帧的三叉阶段使用帧首快照，不会在同一帧再次跳阶段或退出。

**三叉进入条件** (normal → fork_straight/fork_turn):
```
count_valid_exits() == 1   // 三层候选边界中任一层通过L-only/R-only、合法边界对及几何检查
route_config中的预期穿出边界对 == 本帧实际边界对
R_point >= 70 && L_point >= 70  // 左右线足够长
trend.slope > 80  // 赛道明显变宽 (岔路特征)
```

- `FORK_ENTRY_DEBOUNCE=0`，当前等价于单帧视觉条件成立即尝试进入。
- Pre Fwd `fork_num=40` 盲区（Fork矫正关闭时）：非直角状态下持续循右线（`caculate_error` 强制用右线偏差），仅接受左右三叉（`fork_dbg_src_L==2 && fork_dbg_src_R==2`）作为视觉入口，检测到时同帧 `fork_num=41` 并进入叉41；其他类型三叉不进三叉状态。Fork矫正开启时走原地图窗口逻辑。
- 穿出边界对匹配（`route_fork_exit_pair_matches`）现已接入视觉入口：仅当该叉在 `fork_exit_pair_*[]` 表内配置了穿出对（≠INVALID）时才强制匹配，表空(INVALID)时不强制，避免空表阻塞全部入口；Pre Fwd 叉40盲区入口豁免穿出对匹配（只认左右三叉）。Fork红区强制入口仍绕过该视觉条件。
- Fork关闭时保持原视觉入口；Fork开启时只有入口蓝窗允许视觉入口，红区起点仍未进入则按地图类型强制进入。
- 视觉入口在Speed或Fork任一开启时调用现有 `INS_Relocalize_At_Flag()`；强制入口不重定位。

**三叉退出条件**:
- fork_straight: 保留 `slope<100` 退出门槛；视觉进入时保存入口帧实际的左上/上右/左右穿出类型，强制进入时保存路线配置类型。仅当坡度低于100，且本帧无有效穿出类型或类型与入口不同，才累计退出计数，连续3帧后退出。Record模式另记录入叉时左右轮累计编码器值，按左右轮独立实测系数换算平均距离，达到400mm时也退出；非Record模式不启用该距离条件。
- fork_turn: yaw硬阈值退出，以及准备角度后连续3帧 `element_count<1` 的视觉丢失提前退出；yaw角度使用 `get_yaw()` + `yaw_angle_difference()`，不再使用独立连续yaw梯形积分。
- 通用“叉中叉”新出口候选、确认和穿出点tracker均已删除。此前 Final Rev 叉8视觉跳号、叉18→19转弯态交接、Final Fwd 叉19上右入口限制与叉20→21直行态交接等叉号特例已于2026-08-12全部移除，仅保留 Pre Fwd `fork_num=40` 盲区循右线特例。
- Fork开启时，入口蓝窗或红区一旦提交事件，在 `hard_end` 前禁止退出并逐帧清空直行/转弯视觉退出计数；出口蓝窗才重新开放原退出条件，`window_end` 仍未退出则强制回到 `normal`。
- Fork开启时不允许新出口跨窗口直接衔接下一事件；当前事件完成后由 `route_progress_on_exit()` 设置 `fork_num=fork_index+1`，下一帧再由下一事件窗口决定能否进入。
- 正常和强制退出都不进行里程重定位。

**地图进度权威**: `route_progress` 不使用可能已经错误的 `fork_num` 反查事件，而是按事件游标和本帧里程选择逻辑三叉。进入窗口时同步地图的 `fork_index`、此前唯一angle id数量及补线/方向索引，可以同时修正漏计、多计和配置错位。

预赛/决赛正反四套 `fill_mode` 与 `fork_exit_pair` 均由 `route_config.c` 集中维护为100项；补线和视觉入口分别通过带赛段、方向参数的安全接口读取。

**科目差异**:
- `route_config.c` 内四套 `fork_type_*[]`: 决定每个岔路是直行(0)还是转弯(1)
- `route_config.c` 内四套 `fork_seq_*[]`: 决定驶出方向 0=左, 1=右, 2=十字
- `route_config.c` 内四套 `fork_exit_pair_*[]`: 标定左、右爬虫分别应从左/上、上/右或左/右边界穿出
- 类型、方向、补线和穿出边界对表统一使用 `FORK_CONFIG_CAPACITY=40`；业务模块只能通过带边界保护的接口读取。

**角度观测器（入叉偏航保护，2026-08-12 移植自 `n30pro/try` 分支）**:
- `state_enter_current_fork()` 快照入叉 `fork_entry_yaw=get_yaw()`，末尾调用 `state_check_fork_entry_yaw_protection(config_index)`。
- 通过 `route_fork_entry_angle_valid()`（`fork_index < route_get_actual_fork_count()`）校验后取配置参考航向 `route_get_fork_entry_angle_deg()`，与实测 yaw 做 ±180° 安全差；偏差 >`FORK_YAW_PROTECT_ERROR_DEG(90°)` 累计连续叉数，连续 `FORK_YAW_PROTECT_CONFIRM_FORKS(2)` 叉超偏即 `car_stop(STOP_REASON_YAW_PROTECTION)`，否则清零。
- 不受 `run_features_map_enabled()` 约束（Record/Speed/Fork 全关也生效）；只对 `ROUTE_ACTUAL_FORK_*` 范围内的三叉校验，编码器标定科目直接跳过。
- `route_config.c` 新增四套 `fork_entry_angle_*[]`（100项）：预赛正向原填入 52 叉参考航向（90 的倍数，网格赛道）；2026-08-13 因叉20/叉27 参考航向与实际入叉不符导致误停车，`ROUTE_ACTUAL_FORK_PRELIM_FORWARD` 暂置 0 关闭观测器，待修正参考表后恢复 52。其余三条路线全 0 预留。
- 新增停车原因 `STOP_REASON_YAW_PROTECTION`，屏显 `YAW_PROTECT`。

**直角元素**:
- `check_sharp_turn_state()` 复用双爬虫 `exit_record` 出口记录：两条爬虫同侧离开、未从顶部离开、且出边点 y >= `angle_detect_lookahead`（默认20，IMG→FWD→Detect可调，数值越大识别越晚）才判直角，避免拐角还很远时过早识别；不再扫描 `visited` 数组。`==1` 时进入 `angle`，Speed或Fork任一开启时按 `trajectory_encode_angle(angle_num)` 编码重定位。
- 独立连续yaw变化 >60° 时退出并执行 `angle_num++`；与三叉共用同一连续角度源，不再对Euler yaw做整数截断或±180°环绕处理。

### 5.4 路径偏差计算 (line.c)

**十字直行专用边线重建（2026-07-11）**:
- 仅在 `car_state==fork_straight && get_current_fork_seq(fork_num)==2` 时执行；不改变通用三叉进入条件。
- `caculate_error()` 先调用 `cross_rebuild_edges()`：仅接受三叉检测最终确认、且左右爬虫均来自侧边界的 `out_L/out_R`；从两者较小 y 的上方 5 行开始逐行向上搜索。横扫只检查图像中央 100 列（约 x=44～143），排除屏幕边缘杂乱跳变；候选双边还必须连续 5 行保持合理线宽和中心连续，才作为新起点。
- 两侧从新起点以索引 0 重新执行双边爬线，瞄点和误差完全使用新边线；不再保留或使用原拐点后的边线。
- `cross_rebuild_result` 细分为：1=成功，-1=侧边穿出条件不成立，-2=中央区域无连续双边，-3=新爬线未向上推进。重爬验收遍历整条新边线，分别寻找左右线最靠上的点，并将边线截断到该点，去掉墙跟随过程中可能绕回起点的尾段。已有成功瞄点时，后续连续最多 3 帧失败会保持最近一次有效瞄点和误差；离开十字立即清空保持。

```
caculate_error():
  1. 确定前视距离 front_point:
     - fork_turn: 125 (fork_front_point)
     - angle元素: min(L_point, R_point)
     - 十字直行: 70
     - 正常: normal_front_point（代码初值85；Flash槽无效时回退105，可在IMG菜单调整）
  
  2. 取前视点附近5点平均 → L, R
  
  3. 计算 M = (L+R)/2
  
  4. 计算各点相对于图像中线的夹角:
     - M_result (中线偏差角)
     - L_result (左线偏差角)
     - R_result (右线偏差角)
  
  5. 根据状态和 fork_seq 选择使用哪个偏差:
     - normal: 单侧寻线失败时使用另一侧；否则默认 M_result
     - fork_turn: 在相对入叉转角不超过40°时，fork_seq==0/1分别直接瞄对应侧爬线最远端之前 `fork_exit_lookback_points` 个点，IMG同步显示该实际控制瞄点；不依赖tracker或穿出点。回退索引无效时退回125点三叉前瞻的L/R边界循迹。
     - fork_straight: 保持原逻辑，fork_seq==0/1 优先使用有效 out_L/out_R，穿出点无效时退回 L_result/R_result；fork_seq==2 使用 M_result
```

**当前边界**:
- `fork_seq==5` 是遗留分支，现有数组只使用0/1/2。
- `get_current_fork_seq()` 已转发到 `route_get_fork_seq()`；科目2配置同样扩展为100项，超出决策三叉数的索引为确定的零填充值，不再越界。
- 直角状态把 `front_point` 设为两侧点数最小值；当该值小于5但另一侧仍较长时，5点平均的起始索引可能为负，需要后续增加边界保护。
- 两侧都没有有效点时，全局 `L/R` 可能保留上一帧坐标，当前函数没有独立的“双侧失败”安全输出。

### 5.5 转向控制 (tool.c)

**转向公式**:
```
output = Kp·e + Kp2·e·|e| + Kd·(-dy) + GKD·(-GZ)
         ─────   ─────────   ─────────   ──────────
         线性P    平方P      TD微分D     陀螺仪D
```

**线性跟踪微分器 (Linear TD)**:
- 二阶低通滤波器形式: `ÿ = ω²(u-y) - 2ζωẏ`
- 输出: y=平滑信号, dy=平滑微分
- 参数: td_freq=15Hz (跟踪速度), td_damp=1.0 (临界阻尼)
- 死区: dead_zone=5px (平滑偏差在此内仅输出陀螺仪阻尼)
- 更新周期: h=0.004s (4ms)

**当前 IMU 配置（2026-07-16 高速弯适配）**:
- IMU660RC 默认陀螺仪量程为 ±4000°/s；`GZ` 仍以 °/s 提供给转向控制，`GKD`、TD参数和差速系数未调整。`GYRO_LPF_ALPHA=0.2`，重新启用软件低通为 Mahony Euler yaw 提供更平滑的角度信号。
- 三叉与直角退出统一使用 `get_yaw()`（Mahony Euler yaw）经 `yaw_angle_difference()` 做 ±180° 环绕安全比较；`get_continuous_yaw()`（去零偏Z轴角速度梯形积分）已不再用于状态机角度判断。
- 每次发车前仍清零并记录本趟绝对角速度峰值，供内部诊断；运行与停车页面不再显示 `GZMax`。超过2000°/s说明旧量程会削顶，接近约4580°/s时高度疑似新档位16位满码。

**与传统差分的对比**:
- 优点: 无噪声放大、参数独立可调
- 缺点: 需额外调 freq 和 damp

### 5.6 速度控制 (motor.c)

**综合控制** (`combined_control`):
```
1. 偏差低通滤波（当前 α=1.0，等价于不滤波）
2. 选择基础目标速度:
   - Speed关闭: 对传入的 basic_speed 做软启动，步进5
   - Speed开启的发车阶段: 从100起步、每2ms增加5，追向 INS_Get_Target_Speed() 合成的“离线速度表 + 实时error上限”目标
   - 缓启动速度与规划目标相遇后: 结束发车缓启动，后续直接使用 INS_Get_Target_Speed()
3. 转向PD控制 (每2次 combined_control 执行1次):
   - PID原始输出直接作为速度差，并按固定 `steer_output_max` 限幅
4. 差速分配:
   - 右转(steer>0): 右内轮减速0.8*steer, 左外轮加速0.2*steer
   - 左转(steer<0): 左内轮减速0.8*|steer|, 右外轮加速0.2*|steer|
   - 左右目标最终限幅为 `[-encoder_max_val, encoder_max_val]`；负目标进入速度环后，最终电机PWM仍由 `PWM_MIN=0` 禁止实际反转
5. 左右轮独立 PID + 速度前馈:
   - PWM = PID(error) + K_forward * target_speed
   - 积分分离: |error|<500 → 积分累积, 抗饱和上限2000
6. 每次调用末尾执行 INS_Update()，更新里程、yaw、低通偏差，并在Record中聚合区间元素与峰值
```

**当前速度决策关系**:
- `speed_decision_enable=0` 时使用 `basic_speed` 和 `motor.c` 内原有 `SOFT_START_MIN=100`、`ACCEL_STEP=5` 软启动；这适用于Record、Fork或DRS单开和四项全关。
- Speed开启时复用同一缓启动：每2ms读取当前规划目标，当前速度低于目标时按5递增，达到或超过目标时立即对齐并关闭缓启动；若实时error或路线速度表先降低到当前速度以下，也会立即在该交点切换。
- `INS_Get_Target_Speed()` 先按里程取当前轨迹点的 `target_speed`，再与当前低通偏差对应的实时速度上限取较小值；表内速度已在保存时综合元素和历史 `peak_error`。
- error 速度上限在绝对值不超过15时为 `MaxV`，15～45线性降至 `MinV`，达到45时为 `MinV`；偏差只限速，不会产生元素或改变三叉计数。
- 地图无效、路径为空或已到末点时 `INS_Get_Target_Speed()` 回退 `min_target_speed`；Speed或Fork任一开启时发车入口只校验当前 RAM 地图，不自动Load或Replan，失败显示 `NO RAM MAP`。
- Speed和Fork可独立或同时开启；两者同时开启时共用 `car_pos.mileage_ticks` 和 `Current_Path_Index`，入口重定位会同时对齐速度表与三叉事件。
- `stop_flag` 会在左右轮速度控制器内强制 PWM 为0；`car_stop()` 在置位后还会立即调用 `NSLEEP_Sleep()` 拉低电机驱动硬件使能，防止在途控制 ISR 的旧 PWM 再次带动车轮，原有 PIT 关闭和双轮 PWM 清零继续作为软件冗余保护。传入目标速度为0只会把基础速度清零，直接转向差速仍可能生成非0左右轮目标，因此不能把它当作等价的硬停车。
- Speed开启时传入的 `basic_speed` 不直接作为实际速度给定；初始化阶段转向输出上限仍设为 `basic_speed+200`，编码器目标上限仍设为 `2*basic_speed`。
- 电机 PWM 软件上限当前为10000（100%占空比）。堵转保护已启用（`STALL_PROTECTION_ENABLE=1`）：PWM≥5000、编码器绝对速度≤30、连续250个2ms周期（500ms）后停车。
- 速度表生成或加载后会预扫描连续 `MaxV` 平台；首尾里程达到25000 ticks的平台用位图标记为DRS区，但末端20000 ticks保留为负压恢复区。进入标记区且实时速度目标仍为 `MaxV` 时把负压降到55%，进入恢复区、离开平台或实时error触发限速时恢复菜单基础负压。
- PIT 控制周期为2ms：速度 PID、前馈、负压更新和 `INS_Update()` 每2ms调用；转向与差速目标每4ms更新一次，中间周期复用上一组左右轮目标。

**负压风扇控制**:
- 无刷电调 50Hz PWM (1~2ms 脉宽)
- 菜单可调百分比 (0~100%)
- 启动: 步进式平滑加速 (每步2%, 1ms步进)
- Speed开启时，`negative_pressure_update()` 会在预扫描长直道DRS区内把高于55%的基础负压降到55%；实时目标低于 `MaxV`、驶出DRS区或关闭Speed时立即恢复基础负压。基础负压不高于55%时不会反向升高。

### 5.7 LADRC 自抗扰控制 (ADRC.c)

> 当前状态：模块保留但未接入 `motor.c` 或实际控制调用链。车辆当前速度环使用的是左右轮 PosPID + `K_forward` 前馈，不能把 LADRC 与现用速度环理解为同时工作。

**结构**: LESO (线性扩张状态观测器) + LSEF (线性状态误差反馈)

```
LESO (实时估计转速 z1 和扰动 z2):
  e  = z1 - actual_speed
  z1 += h * (z2 + b0*u - beta1*e)    // beta1 = 2*wo
  z2 -= h * beta2 * e                 // beta2 = wo²

LSEF (状态误差反馈):
  u0 = wc * (target_speed - z1)       // wc=控制器带宽

扰动补偿:
  u = (u0 - z2) / b0                  // 扰动前馈补偿
```

**参数**: b0=系统增益, wc=控制器带宽, wo=观测器带宽 (通常 wo = 3~5*wc)

**限制**: z2 被 clamp 在 ±(max_pwm * b0 * 1.5) 防积分爆炸

### 5.8 惯导与轨迹速度表 (inertial_navigation.c/h)

当前模块不是二维位置/姿态融合导航，也不按记录轨迹控制方向。方向始终由摄像头巡线决定；该模块主要承担“编码器累计里程 + 固定触发打点 + 历史风险速度表 + Speed实时限速 + Flash完整性校验 + 路标重定位”。

**四个RAM功能开关**:
- `record_enable`: 独占功能。开启后关闭Speed/Fork，保留原始视觉巡线和三叉判断，按距离记录轨迹，速度使用 `basic_speed`。
- `speed_decision_enable`: 使用轨迹速度表与实时error上限，不限制原始三叉视觉入口。
- `fork_correction_enable`: 使用三叉里程窗口，速度仍可单独选择 `basic_speed` 或速度表。
- `drs_enable`: 默认关闭；仅与Speed同时开启时允许长直道降低负压，单独开启不改变速度、负压或地图校验。
- 四项全关时允许无地图普通视觉运行；Speed与Fork可以单独开启或同时开启，任一地图功能开启都要求有效RAM地图。

**实时状态与里程**:
- `INS_Update()` 每2ms读取左右轮实际速度和 IMU yaw；里程增量为 `fabs((enc_L+enc_R)/2)`，因此里程只增不减。
- 视觉偏差在惯导模块内以 `filtered=0.7*raw+0.3*previous` 独立低通；同一低通值同时供Record区间峰值聚合和Speed实时限速使用。
- 元素统一由 `trajectory_policy.c/h` 编码：`0x00`普通路段，`0x01～0x7F`为三叉（编码=`fork_num+1`），高位为1的 `0x80～0xFF`为直角；不再使用 `200+angle_num` 魔数。
- `INS_Sync_Tracking_Index()` 只按当前里程向前推进 `Current_Path_Index`。

**轨迹记录与存储**:
- `INS_Pos_t` 固定为8字节：`uint32_t mileage_ticks`、`uint16_t target_speed`、`uint8_t element_code`和`int8_t peak_error`；编译期检查锁定结构体大小，不再持久化 `delta_yaw`。
- `INS_Update()` 在每个记录区间内保留绝对值最大且带符号的低通 `peak_error`（写入前限幅到-90～90），并锁存该区间第一个非零元素。
- `INS_Record_Trajectory()` 仅在 Record 模式生效，使用 `next_record_mileage=1000、2000、3000……` 固定触发线；达线时保存真实当前里程，然后把下一触发线推到当前里程之后，超出1000 tick的余量不再被重置成累积相位漂移。
- 保存前会把不足1000 tick的最后区间作为真实终点补入，保留停车前最后一段的 `peak_error` 和元素；重复保存不会追加同里程节点。
- 容量为3072点（约24KB RAM，1000 tick间隔时约覆盖52.9m）；达上限后停止新增、置 `path_record_full`，不覆盖已有数据。
- `Save Map` 会先用当前 MinV/MaxV/Decel/Accel 自动生成速度表，再把首个轨迹点目标设为 MaxV，随后写入 Flash 并回读校验；不再要求用户在保存前手动执行 `Re-Plan Vel`。加载旧地图时先完成原始载荷CRC校验，再在RAM中把首点规范为当前地图的 MaxV，因此旧地图也能使用新的发车缓启动。

**速度表生成** (`INS_Generate_Velocity_Table`):
1. 校验 `0<=MinV<=MaxV<=2000` 且 Decel/Accel 均在1～200；非法参数拒绝规划和保存。
2. 每点先计算元素上限和历史error上限，取较小值；直角、转弯三叉和超出路线配置范围的未知三叉使用 `MinV`，普通路段和直行三叉不因元素身份降速。
3. 历史 `|peak_error|<=15` 时上限为 `MaxV`，15～45整数线性降至 `MinV`，`|peak_error|>=45` 时为 `MinV`。
4. 直角和转弯三叉的连续区间以中点为基准，在驶入前30个索引、驶出后10个索引形成非对称低速平台；元素自身仍保持 `MinV`。
5. 从后向前使用可调 `DECEL_STEP`建立减速包络，从前向后使用可调 `ACCEL_STEP`建立加速包络；默认仍为15/50。
6. 空间包络完成后首点强制设为 MaxV；Speed开启时的发车加速斜坡由 `motor.c` 负责。

**Flash轨迹格式与验证**:
- sector 0第0页保存 `TRJ2`元数据（magic、版本2、单点大小、点数、总里程、规划参数和CRC32）；第5～10页保存最多24KB轨迹，第11页保留。
- 使用标准 CRC-32/ISO-HDLC（多项式 `0xEDB88320`）覆盖实际 `point_count*8` 字节轨迹载荷；保存时先擦元数据使旧图失效，写完载荷后最后提交元数据，随后立即回读。
- 保存回读使用独立页缓冲与 RAM 原图逐字节比较并复算CRC；校验失败会让Flash地图继续保持无效，但不会覆盖唯一的Record轨迹，允许直接重试保存。
- 加载依次验证magic/版本/8字节记录、点数和规划参数、CRC32、里程严格递增、末点总里程、速度不超过2000及 `peak_error`在-90～90；仅全部通过后才置 `trajectory_map_valid`。
- 旧16字节地图没有新magic/版本，会作为旧格式明确拒绝，不自动迁移；任一验证失败均不发布RAM地图且清空地图事件。

**路标重定位** (`INS_Relocalize_At_Flag`):
- 仅在Speed或Fork任一开启且RAM地图有效时生效；只由真实视觉进入三叉或直角时调用，强制进入和所有退出都不调用。
- 在当前轨迹索引前后各60点内，从前向后查找完全相同的 `element_code`；仅当里程差绝对值小于5000 ticks时覆盖当前里程和索引。
- 当前实现找到第一个同编码点后就退出，即使该点里程差超限，也不会继续寻找窗口内后续同编码点。
- 每次有效调用累计 `MC` 尝试次数，真正覆盖里程和索引时累计成功次数并保存最近一次带符号修正量 `D`；统计在每次 `INS_Init()` 发车初始化时清零，停车后显示 `MC:成功/尝试` 与 `D`。

### 5.9 三叉里程状态矫正 (map_event / route_progress)

**map_event**:
- 扫描 `Path_Memory`，把连续相同且非0的 `element_code` 聚合为事件；事件类型和编号统一通过 `trajectory_policy` 解码，不再判断 `>=200` 魔数。
- 同一编码被普通路段分隔时会形成多个事件片段；`route_progress` 按相同fork id合并为一个逻辑三叉。
- `MAX_MAP_EVENTS` 由80扩展到128；满后停止新增、置 `map_event_overflow`，不写越界。事件数量可以为0，轨迹是否有效由Flash元数据、CRC32和轨迹语义校验决定。
- `peak_error` 不会生成三叉或直角事件；偏差只用于速度限制。

**逻辑事件构建**:
- `route_progress` 按fork id合并断裂片段，起止里程取最早起点和最晚终点；fork id转换为0基 `route_config` 索引并做100项容量校验。
- `angle_count_before` 按此前出现过的不同angle id统计，断裂片段不会重复增加；进入三叉窗口时同步 `fork_num`、`angle_num` 和配置索引。
- 地图事件溢出、fork id越界、事件跨度为空、原始事件交叠或最终边界非法时，逻辑事件构建失败；Fork开启时显示 `FORK MAP ERR` 并禁止发车。Speed单开仍可使用语义有效的速度表。

**窗口生成**:

```text
hard_start   = S + round((E-S) × 40%)
hard_end     = E - round((E-S) × 20%)
window_start = max(0, S - 5000)
window_end   = E + 3000
```

- 红区默认至少1000 ticks；不足时以事件中点恢复，原事件本身不足1000时使用完整 `[S,E]`。
- 相邻逻辑三叉按两个原始事件之间的中点裁剪外扩窗口；裁剪只缩小，窗口为左闭右开，绝不重叠。
- 五个参数均可在Path/Assist调节并持久化：缩减比例0%～45%/步进5%，外扩和红区最小值0～10000/步进500 ticks。

**状态命令**:
- 窗口外拒绝三叉视觉入口；入口蓝窗 `[window_start,hard_start)` 允许现有视觉条件进入。
- 未提交事件到达 `hard_start` 后请求强制进入；即使单帧跨过红区也会先强制进入，下一帧才可能退出，避免同帧进入又退出。
- 事件提交后，在 `hard_end` 前保持三叉状态并清除可跨帧的退出证据；出口蓝窗 `[hard_end,window_end)` 恢复原直行/偏航/出口退出。
- 到达 `window_end` 仍在三叉时强制退出。正常退出和强制退出都只推进事件和计数，不修改里程或轨迹索引。
- `fork_assist.c/h`、`progress_anchor.c/h`、Corr/Gate/FFix/Lock和旧lockout已删除。

### 5.10 路径规划脚本 (path.py)

`path.py` 用 Python 建模预赛/决赛赛道图，将四条已选定的关键节点路线按最短段展开，校验目标边与目标节点覆盖，再输出 `fork_type`、`fork_seq`、`fill_mode` 和 `fork_exit_pair` 数组。

**关键规则**:
- 起点位于 0-4 左侧边界线上，由 `START_EDGE_NODES` + `START_POSITION_RATIO` 解析为虚拟起点 `999`
- 起点落在目标边中间时，将该边拆成两个独立目标半边，避免“走半条算整条”
- 固定输出初赛正向、初赛反向、决赛正向、决赛反向四种跑法；关键节点之间用 Dijkstra 最短路展开为完整路径
- 当前校验长度为初赛正/反向均 17.2m，决赛正/反向均 26.5m；决赛正向包含37个决策三叉，反向包含36个
- 当前工程按 `subject_id + route_direction` 读取预赛正向、预赛反向、决赛正向、决赛反向四套 `fork_type/fork_seq/fill_mode/fork_exit_pair`；决赛反向第9项保留实车标定的额外纵向补线
- `path.py` 从 `code/route_config.h` 读取 `FORK_CONFIG_CAPACITY`，每次运行会校验四条路线的起终点、目标边/节点覆盖和固定长度，并输出 40 位 `fork_type_*`、`fork_seq_*`、`fill_mode_*`、`fork_exit_pair_*` 数组；左+直映射左+上，直+右映射上+右，左+右及十字均映射左右穿出点
- 路线三叉数超过共享容量时脚本会明确报错，不再通过切片静默截断后续配置
- 四张路线图保存为 `.codex_tmp/track_prelim_forward.png`、`.codex_tmp/track_prelim_reverse.png`、`.codex_tmp/track_final_forward.png`、`.codex_tmp/track_final_reverse.png`；同时导出 `.codex_tmp/path_arrays.json`
- `path_routes_basic.py` 是可单独分享执行的自包含脚本，内置最小赛道模型和四条固定路线，只输出简化后的节点顺序、三叉数量、总长度，并为四种跑法生成基础路线图 `route_images/basic_prelim_forward.png`、`route_images/basic_prelim_reverse.png`、`route_images/basic_final_forward.png`、`route_images/basic_final_reverse.png`；脚本默认会调用系统图片查看器显示生成图片，可用 `ROUTE_SHOW_IMAGES=0` 关闭自动显示，重复经过的物理边会按固定物理边法线分到两侧并行显示，标签只保留步骤号；直线中间分段点仅用于底层长度/覆盖校验，不出现在分享输出中

`path_solver.c` 是不依赖 Python/Matplotlib 的 C 版精确求解器，目标是在同一赛道规则下证明最短闭环路线。

**关键策略**:
- 状态为 `(当前位置, 上一节点, 已覆盖目标边mask)`，保留禁止原地掉头约束
- 使用已证明最短的 28.8m 合法闭环作为初始上界
- 剪枝下界 = 未覆盖目标边总长 + 未覆盖目标组件的 0 成本收缩 MST 连接下界
- 状态记忆表记录同一状态的最短到达距离，劣解直接剪枝
- 默认不画图，只输出简化后的关键路线节点、总长度、40 位 `fork_type_final`、`fork_seq_final`、`fill_mode_final`；直线中间分段点仅用于内部最短路和长度校验，不出现在 `route nodes` 打印中
- 当前已记录 MOS：节点 31（栅极朝 48 方向，建模用相邻节点 32 表示栅极侧）和节点 37（栅极朝 42 方向）；`fill_mode` 会在经过这些 MOS 节点时按栅极侧/非栅极侧补线
- 默认起跑方向为反向发车 `999 -> 2`，可设置 `PATH_START_HEADING=1` 临时切回原方向 `999 -> 1`
- 若未设置 `EXACT_STATE_LIMIT` 并完整跑完，可证明当前约束下最短；若设置状态上限提前停止，则只输出当前最好路线并明确标注未证明

---

## 六、菜单系统 (`code/menu/`)

**按键操作**: 5键 (上下左右+确认)

菜单使用静态描述表和固定深度页面栈，不使用动态内存。普通列表统一由 `menu_nav` 处理上下选择、页面进入/返回、滚动和按位编辑；Set、IMG、Gyro Calib 与 Encoder Check 通过自定义页面回调保留原业务流程。根 `code/menu.h` 只转发到 `code/menu/menu.h`，因此菜单外现有 `#include "menu.h"` 无需修改。

**菜单结构**:
```
ROOT
├── Set  — 按功能组合校验RAM地图和逻辑三叉，通过后等待2秒发车
├── PRM
│   ├── Preset  — 5组调参预设，支持直接应用或进入编辑后保存并应用
│   ├── Steering — Kp/Kd/Kp2/GKD/TD/死区
│   └── Speed    — 基础速度、左右PI、Kff、负压
├── IMG  — 实时图像、曝光、显示模式、fork_num、普通/三叉前视点、出口回退量与三叉调试
├── SUB  — 选择并持久化 Pre Fwd / Pre Rev / Final Fwd / Final Rev
├── Path
│   ├── Map      — Record、Speed、Fork、DRS开关，轨迹加载/保存、速度表重规划
│   ├── Velocity — MaxV、MinV、Decel、Accel
│   ├── Assist   — 入口/出口缩减、入口/出口外扩、红区最小长度
│   └── Status   — 地图/逻辑事件、阶段、四边界、来源与同步前后值
├── Cal
│   ├── IMU Status
│   ├── Gyro Calib
│   └── Encoder Check
└── MOT — 直接调用现有NSLEEP唤醒/休眠封装
```

**导航规则**:
- 浏览态：UP/DOWN循环选择，PRESS进入/执行，LEFT返回父页面；父页面保留原光标。
- 编辑态：UP/DOWN修改当前数位，LEFT/RIGHT换位，PRESS结束编辑。
- SUB进入时按当前赛段与方向组合对齐光标；选择四项之一后立即保存，后续三叉类型、驶出方向、补线表、穿出边界和停车编号均按 `subject_id + route_direction` 读取对应配置。叉号特例仅保留 Pre Fwd 的 `fork_num=40` 盲区循右线。
- IMG主页面提供 EXP/VIEW/FRK/FWD/DBG 五项，EXP、FRK和FWD支持按位编辑并以红色高亮当前位；FWD分别编辑普通前视、三叉前视、直角检测前瞻 Detect（0～100）和0～40的出口回退量 Back。图像在寻线计算完成后与加粗边线、实际控制瞄点一起输出，最底部 `y=112` 保留寻线状态。状态面板每4帧刷新一次以降低LCD占用。
- PRM/Preset提供5个Flash持久化预设，每组保存 Kp、Kd、Kp2、GKD、TD频率/阻尼、死区、基础速度、普通前视、三叉前视和出口回退量；短按直接应用，进入编辑页后可 Save+Apply。
- Path/Velocity 中 Decel/Accel 可在1～200内编辑，默认15/50；Path/Assist五个窗口参数使用固定步进编辑，离开Path页时一起写入参数页。
- `Save Map` 会自动重规划、写入并回读验证，显示 `SAVED` 或 `INVALID`；`Load Map` 仅在完整校验通过时显示 `LOADED`，否则显示 `NO MAP`。Set不读Flash、不重新规划；Speed或Fork开启但RAM地图无效时显示 `NO RAM MAP`，Fork开启但逻辑事件非法时显示 `FORK MAP ERR`。
- Cal列表页只在进入或按键变化时重绘；IMU Status和Encoder Check子页继续实时刷新，避免Cal列表整页闪烁。IMU Status显示零偏、Pitch/Roll、Mahony Yaw和实时GZ；运行界面不再显示GZMax。

**Flash 存储兼容**:
- 参数仍固定在 sector 0、`PARAM_PAGE=1`；33号为布局版本3，40号为普通前视、41号为当前科目，42号起存放5组×12 word预设，其后为Decel/Accel、五个三叉窗口参数和发车方向，当前总长110 word。旧参数页方向槽无效时自动使用反向并回写，保持原有发车行为。
- 两个 `PosPID` 与 `SteeringPID` 继续按结构体原始字节 `memcpy`，并用编译期尺寸检查锁定32/28字节。
- 从布局版本1升级时保留普通参数，将当前出口回退量设为30，并明确清空按11 word保存的旧预设；不把旧预设按12 word步长误读。用户已确认旧预设可以重新设置。
- PRM、IMG、SUB、Path与Preset保存均先读完整参数页、只覆盖本组槽位、再写完整页，保证34～37陀螺槽和其他参数不被破坏。
- 从布局版本2升级到3时保留所有旧参数和五组预设，只给新增窗口槽写入40%、20%、5000、3000、1000默认值。
- Steering/Speed/IMG/Path仍在离开相应页面时保存；SUB把赛段写入41号槽位、方向写入新增末尾槽位；Record、Speed、Fork、DRS四个功能开关只存在RAM中，其中DRS上电默认关闭。
- 轨迹与参数页分开：sector 0第0页为轨迹元数据，第5～10页为8字节轨迹点载荷，第11页保留。旧16字节轨迹不做自动迁移，需重新Record。

**ADS构建提示**: 当前新增 `code/run_features.c` 和 `code/route_progress.c`，并删除 `fork_assist.c`、`progress_anchor.c`。首次构建前需在ADS中Refresh工程，让managed build重新发现源文件并生成Debug下的makefile；不要手工维护自动生成文件。

---

## 七、关键阈值与参数

| 参数 | 值 | 位置 | 说明 |
|------|----|------|------|
| FORK_ENTRY_DEBOUNCE | 0 | state_control.c | 当前无消抖，首个满足条件帧即尝试进入 |
| FORK_EXIT_ANGLE | 60° | state_control.c | 转弯三叉退出角度，决赛适配 |
| fork_straight退出确认 | 3帧 | state_control.c | 连续3帧无出口且赛道不再变宽后退出 |
| Record直行三叉里程退出 | 400mm | state_control.c | 左右轮按0.017526346/0.017562399 mm/tick分别换算后取平均，与原视觉退出条件为OR |
| EXIT_STEP_RATIO | 0.7 | element_line.h:4 | 出口点步数比例 |
| FORK_SIDE_EXIT_MIN_Y | 0 | element_line.c:13 | 判断条件为 y>0，实际接受 y≥1 |
| trend.slope阈值 | 80 | state_control.c:142 | 三叉进入坡度门槛 |
| 四路线停车编号 | Pre Fwd 54 / Pre Rev 22 / Final Fwd 39 / Final Rev 38 | route_config.h | 分别按四条路线的实际三叉数量加2设置，驶过路线结束后的第二个三叉再停车 |
| FORK_REENTRY_DELAY_FRAMES | 0 | state_control.c:65 | 决赛密集三叉暂时取消固定帧重入阻塞 |
| FORK_CONFIG_CAPACITY | 100 | route_config.h | 类型、方向、补线、状态计数和菜单共用的配置容量；预赛正向新路线含52个决策三叉，超出原40项容量 |
| FORK_YAW_PROTECT_ERROR_DEG | 90° | state_control.c | 角度观测器：入叉实测yaw与配置参考航向偏差超过该值视为走错方向（移植自 n30pro/try） |
| FORK_YAW_PROTECT_CONFIRM_FORKS | 2 | state_control.c | 角度观测器：连续多少个三叉入叉超偏才停车，避免单叉误判 |
| ROUTE_ACTUAL_FORK_* | 0/0/0/0（暂置0） | route_config.h | 角度观测器实际参与校验的三叉数量；Pre Fwd 曾为52，2026-08-13 因参考航向不符误停车暂置0，其余三条路线未提供数据保持0 |
| fork_exit_lookback_points | 默认30点，范围0～40 | line.c / IMG / Preset | 转弯三叉在40°内按该回退量直接选择对应侧爬线最远端之前的爬线点，不依赖tracker或穿出点；每组预设独立保存 |
| angle_detect_lookahead | 默认20行，范围0～100 | element_line.c / IMG FWD / Flash | 直角检测前瞻行：出边点 y 必须 >= 此值才判直角，数值越大识别越晚；可在IMG→FWD→Detect调整并持久化 |
| GEOMETRIC_BACKTRACK_ADVANCE_MIN | 20px | maze.h | 几何回爬检测开始生效前所需的最小向上推进量 |
| GEOMETRIC_BACKTRACK_RETURN_MARGIN | 5px | maze.h | 判定回到原始起爬高度附近的纵向容差 |
| GEOMETRIC_BACKTRACK_CONFIRM_POINTS | 4点 | maze.h | 几何回爬需要连续满足的实际移动点数 |
| MAX_MAP_EVENTS | 128 | map_event.h | 地图事件容量；满后置 `map_event_overflow` 并停止新增 |
| 三叉入口/出口缩减 | 40% / 20% | route_progress.c / Path Assist | 原事件两端按比例缩减红色强制核心，范围各0%～45%、步进5% |
| 三叉入口/出口外扩 | 5000 / 3000 ticks | route_progress.c / Path Assist | 蓝窗外扩，范围各0～10000、步进500，并受相邻中点裁剪 |
| 三叉红区最小长度 | 1000 ticks | route_progress.c / Path Assist | 防止强制核心被比例缩减压没，范围0～10000、步进500 |
| STRAIGHT_ANGLE_DEG | 35° | path.py:62 | 路线数组生成时区分直行与转弯的夹角 |
| 控制/转向周期 | 2ms/4ms | initialize.c:104, motor.c:390-395 | 速度环与INS每2ms，转向与差速每4ms |
| SOFT_START_MIN / motor ACCEL_STEP | 100 / 5 | motor.c | 基础速度及Speed发车缓启动下限与每2ms加速步进；Speed仅在首次与速度表相遇前使用 |
| 速度表发车首点 | MaxV | inertial_navigation.c | 生成后及旧地图加载校验后均把RAM首点规范为MaxV，时间缓启动由motor.c完成 |
| RECORD_INTERVAL_TICKS | 1000 ticks | inertial_navigation.h | Record固定里程触发线，约对应17.21mm |
| MAX_PATH_POINTS / INS_Pos_t | 3072 / 8字节 | inertial_navigation.h | 轨迹载荷共24KB，约覆盖52.9m |
| TRAJECTORY_ERROR_FILTER_ALPHA | 0.7 | inertial_navigation.c | `filtered=0.7*raw+0.3*previous`，供峰值记录和Speed限速共用 |
| error降速阈值 | 15 / 45 | trajectory_policy.h | 不降速上界 / 完全降到MinV下界，中间线性插值 |
| DRS长直道/恢复区/负压 | MaxV平台≥25000 ticks / 尾部20000 ticks / 55% | inertial_navigation.h / motor.c | 速度表生成或加载时用位图标记长MaxV平台，末端恢复区不标记；进入DRS区且实时目标仍为MaxV时降负压，进入恢复区或实时限速时恢复基础负压 |
| peak_error存储范围 | -90～90 | inertial_navigation.h | 区间内绝对值最大的带符号低通偏差 |
| 速度表低速平台 | 元素中点前30 / 后10索引 | inertial_navigation.c | 入弯前更早降到MinV，驶出后更早恢复加速；元素自身始终保持MinV |
| 离线 DECEL_STEP / ACCEL_STEP | 默认15 / 50，范围1～200 | inertial_navigation.h / Path菜单 | 逆向减速与正向加速空间包络步进，可持久化调节 |
| 轨迹Flash页 | 0 / 5～10 / 11 | inertial_navigation.c | 元数据 / 24KB轨迹载荷 / 保留页，载荷使用CRC32校验 |
| 重定位搜索/漂移门限 | ±60点 / 5000 ticks | inertial_navigation.c | 路标匹配范围与允许的里程误差（用户实测约10cm） |
| td_freq | 15Hz | tool.c:160 | TD自然频率 |
| td_damp | 1.0 | tool.c:161 | TD阻尼比 |
| dead_zone | 5px | tool.c:162 | 转向死区 |
| K_forward | 菜单可调 | motor.c:291 | 速度前馈系数 |
| negative_pressure_percent | 菜单可调 | motor.c:29 | 负压百分比 |
| PWM_MAX | 10000 | motor.c:19 | 左右电机最终PWM软件限幅（100%占空比）；本趟峰值发车时清零并在运行/停车页面显示 |
| steer_output_max | basic_speed+200 | initialize.c:64 | 转向PID直接速度差的固定输出限幅 |
| 堵转保护 | 开启（5000/30/250周期=500ms） | motor.c:23-26 | 大PWM且编码器几乎不转持续500ms后停车 |
| 脱线姿态保护 | pitch绝对值>40° | maze.c:679 | 双侧爬线点数均小于10时立即停车的附加条件 |
| IMU量程/加速度输出率 | ±4000°/s / 60Hz | zf_device_imu660rc.h/.c | 高速弯避免Z轴角速度在±2000°/s削顶；GKD保持不变 |
| GYRO_LPF_ALPHA | 0.2 | MahonyAHRS.c | 陀螺仪软件低通系数，为 Mahony Euler yaw 提供平滑角度信号 |
| PID_I_OUTPUT_MAX | 1000 | motor.h:15 | 积分输出上限（PWM单位），配合同向积分保持逻辑确保弯中负载补偿 |
| 计时停车 | 8s | cpu0_main.c:66 | 发车后计时停车上限 |

> 详细修改记录见 `thresholds.md`

---

## 八、调试与通信

**无线串口发送函数** (`send_speed_to_pc`):
```
"三叉编号,目标速度左,目标速度右,实际速度左,实际速度右,error\r\n"
```

当前 `cpu0_main.c` 主循环会在未停车时无延时调用该函数，因此持续自动发送；首字段在 `car_state == normal` 时发送0，在 `fork_straight`、`fork_turn` 或 `fork_exit_pending` 三叉状态下发送当前 `fork_num`；其余五字段为左右目标/实际速度和 `return_error()` 返回的实时路径偏差。旧版11字段三叉出口诊断格式已不在当前发送函数中。

**显示屏**: IPS114 (135×86) 运行时只持续显示本趟最大 `PWM`；停车后增加 `MC:成功/尝试` 与最近一次里程修正量 `D`。Record保存提示会同时显示这些诊断，退出提示后主循环会重新绘制停车诊断。旧 `B/GZMax/correct fork/correct num` 运行与停车屏显均已删除。

---

## 九、修改记录中的关键改动

1. **无线串口**: 关闭自动波特率 (WIRELESS_UART_AUTO_BAUD_RATE=0), 底层UART直达发送
2. **RTS引脚**: 从 P10_2 改为 P33_8
3. **NSLEEP**: 适配 DRV8245-Q1 HW版本上电握手时序 (30μs低电平脉冲)
4. **负压风扇**: 从 #define 常量改为 float 变量, 菜单可调+Flash可存
5. **三叉坡度门槛**: 曾从80调至50适配MOS渐进三叉，后因误触发回退为80；当前值80
6. **出口点比例**: 曾从0.7调至0.4适配渐宽叉，后因误触发回退为0.7；当前值0.7
7. **直行退出帧数**: 从 10 调至 5 (快速响应)
8. **叉中叉逻辑**: 新出口候选、确认、直接交接及后续穿出点tracker均已删除
9. **触边停止步数**: 从 60 调至 80 (防误判)
10. **Flash页合并**: 5页→1页 (释放空间给轨迹存储)
11. **路线数组脚本**: `path.py` 改为展开并校验四条固定关键路线，导出 C 数组、JSON 与四张路线图；当前长度为初赛17.2m、决赛26.5m
12. **左右边界三叉出口 y 下限**: `FORK_SIDE_EXIT_MIN_Y` 当前为 0，保留 4.2m 实测版本配置
13. **决赛反向基线**: 更新 `fork_type/fork_seq/fill_mode`，转弯退出角45°→60°（决赛适配），停车序号33→38，密集三叉暂时取消固定重入帧阻塞；FILL_FORK_MAX 20→40 与 FORK_MAX 一致；fork_num=8 两侧穿出按 fork_num=9 处理
14. **三叉配置集中化**: 新增 `route_config.c/h` 统一40项类型、方向、补线数据和安全接口；速度规划移除 `fnum<30`，`path.py` 改为读取共享容量并拒绝超容量静默截断；后续停车编号已扩展为四路线独立配置，当前硬编码编号特例为 Final Rev 的第8叉跳号和第18叉转弯态连续交接，以及 Final Fwd 的第20叉直行态连续交接
15. **紧凑轨迹点**: `INS_Pos_t` 从16字节改为8字节，3072点保存真实里程、目标速度、单字节元素和区间 `peak_error`；固定1000 tick触发线消除超额余量的相位累积。
16. **多源速度决策**: 新增 `trajectory_policy.c/h` 统一元素编解码和15～45 error速度上限；离线速度表同时考虑元素与历史峰值，Speed开启时由 `INS_Get_Target_Speed()` 叠加实时低通error限速，关闭时 `motor.c` 使用基础速度。
17. **轨迹Flash事务**: 第0页保存TRJ2元数据，第5～10页保存24KB载荷；CRC32、回读和语义校验全部通过后才发布RAM地图，旧16字节图明确拒绝。Flash读取由用户在菜单中手动触发；Speed或Fork只使用当前RAM地图，无有效RAM地图时显示 `NO RAM MAP` 并阻止发车。
18. **地图事件与菜单适配**: 事件使用单字节元素编码并扩容到128；Path/Velocity新增1～200的Decel/Accel持久化参数，删除无效Spd开关，保存时自动重规划并验证。
19. **高速弯连续yaw**: 陀螺仪恢复±4000°/s，去零偏Z轴角速度每2ms做梯形积分；三叉/直角用连续浮点角差退出，Mahony继续负责姿态显示。GZMax仍在发车时清零并由内部记录，但不再占用运行/停车屏幕。
20. **Speed发车缓启动**: 速度表首点由固定100改为MaxV；Speed发车复用Record的100起步、每2ms增加5的缓启动，与实时规划目标相遇后切换为速度表直接控制，旧地图加载后同样在RAM中规范首点。
21. **运行功能与三叉里程状态矫正**: `run_features.c/h` 将Record、Speed、Fork拆为RAM开关，Record独占而Speed/Fork可组合；`route_progress.c/h` 把同编号地图片段合并为逻辑三叉，建立红色强制核心和蓝色识别窗口，并统一管理入口、保持、退出、编号同步与诊断来源。旧 `fork_assist`、`progress_anchor` 已删除。
22. **长直道DRS负压**: Path/Map新增上电默认关闭且不写Flash的DRS独立开关；仅Speed与DRS同时开启时生效。速度表生成或加载后预扫描连续MaxV平台，以25000 ticks为最小长度用位图标记DRS区，并将平台末端20000 ticks留作负压恢复区；车辆进入标记区且实时速度决策仍为MaxV时把负压降到55%，进入恢复区、实时偏差限速、关闭Speed或关闭DRS时恢复菜单基础负压。
23. **非对称低速平台**: 速度规划把转弯元素中心前后的固定MinV扩展从对称±20点拆为驶入前30点、驶出后10点，在更早入弯减速的同时缩短出弯低速拖延；元素自身仍保持MinV。
24. **yaw 角度源切换与三叉出口重构**: `state_control.c` 全面改用 `get_yaw()`（Mahony Euler yaw）+ `yaw_angle_difference()` 替代 `get_continuous_yaw()`；状态进入/退出统一为 `state_enter_current_fork()` / `state_finish_current_fork()`，入口区分视觉/强制来源，退出恢复 route_progress 里程窗口控制。三叉转弯使用视觉丢失提前退出通道，并保留陀螺仪硬兜底；叉中叉新出口识别、直接交接和穿出点tracker均已删除。`GYRO_LPF_ALPHA` 从 1.0 降为 0.2 重新启用软件低通。`PID_I_OUTPUT_MAX` 从 600 提高到 2000，PosPID 大误差时同向积分保持不再泄放。计时停车从 13s 缩短为 8s。
25. **转向差速归一化**: `motor.c` 将转向PID原始输出按固定 `steer_output_max` 归一化为 `[-1,1]` 比例，再乘当前 `current_ramp_speed` 形成差速；继续使用0.2/0.8内外轮分配，并把左右目标下限统一限制为0，使降速前后轮速比例基本一致且所有状态禁止反转。
26. **恢复原始转向差速**: 2026-07-20按用户要求撤销归一化，恢复“转向PID输出直接作为速度差 + `steer_output_max` 限幅”的控制和原调参方式；左右目标范围恢复为 `[-encoder_max_val, encoder_max_val]`，实际PWM仍由速度环与电机输出的0下限禁止反转。
27. **无线调试字段替换**: `send_speed_to_pc()` 的首尾字段由 yaw、陀螺 Z 角速度替换为三叉状态编号、实时 `error`，中间四个左右目标/实际速度字段及六字段顺序保持不变；编号字段在 `normal` 状态发送0，在三个三叉状态发送当前 `fork_num`。
28. **三叉穿出边界白名单**: `route_config` 为初赛/决赛逐叉标定左上、上右或左右有序穿出边界对；通用检测拒绝左爬虫穿右、右爬虫穿左等非法组合，所有视觉入口必须匹配当前配置，Fork红区强制入口例外。`path.py` 为四条路线同步生成40项C数组和JSON字段，十字按左右穿出点处理。
29. **预赛/决赛正反向发车选择**: SUB扩展为 `Pre Fwd / Pre Rev / Final Fwd / Final Rev` 四项并持久化方向；`route_config` 接入四套类型、驶出方向、补线和穿出边界表，所有消费者按赛段与方向联合查询，旧参数默认映射到反向配置。
30. **四路线停车与连续叉范围修正**: 停车编号拆为 Pre Fwd 22、Pre Rev 22、Final Fwd 39、Final Rev 38，均对应驶过各路线结束后的第二个三叉；第18叉向第19叉的同帧连续交接增加决赛条件，仅允许 Final Rev 触发。
31. **决赛正向第20叉直行态连续交接**: 删除进入第20叉前的视觉跳号。Final Fwd 已处于 `fork_straight`、`fork_num=20` 时，若叉内检测到左右三叉，则同帧结束第20叉并按第21叉配置重新进入；不使用转角门槛，不影响其他路线。
32. **直行三叉退出增加穿出模式**: 保留原 `trend.slope<100` 和连续3帧门槛；低坡度时，当前无有效穿出类型或类型与入口不同时才累计退出。十字仍处于高坡度时不会仅因第一次爬线穿出模式波动而退出。
33. **决赛正向第19叉上右入口限制**: Fork矫正关闭时，Final Fwd `fork_num=19` 只有视觉识别为上右三叉才允许进入；其他模式会继续保持normal，其他路线、编号及Fork矫正流程不变。
34. **Record直行三叉400mm退出**: 中等速度5次2m标定得到左轮0.017526346 mm/tick、右轮0.017562399 mm/tick；Record模式进入直行三叉时记录左右累计编码器值，分别换算后取平均，达到400mm即走现有统一退出，与原连续3帧视觉条件为OR。Speed、Fork和基础模式不启用该距离条件。
35. **预赛正向新路线与容量扩容**: 预赛正向路线更新为52个决策三叉、总长47.7m（新赛道关键节点0~67）；`FORK_CONFIG_CAPACITY` 从40扩到100，四条路线的 `fork_type/fork_seq/fill_mode/fork_exit_pair` 全部补齐为100项。`fork_exit_pair_prelim_forward` 暂不使用，全0(INVALID)预留，视觉入口匹配保持 `#if 0` 禁用；其余路线保持原数据仅补零；`ROUTE_STOP_FORK_PRELIM_FORWARD` 从22改为54（52叉+2）。
36. **移植角度观测器（三叉入口偏航保护）**: 从 `n30pro/try` 分支移植入叉偏航保护：`state_enter_current_fork()` 快照入叉 yaw，末尾 `state_check_fork_entry_yaw_protection()` 对比 `route_get_fork_entry_angle_deg()` 参考航向，偏差 >90° 且连续 2 叉即 `car_stop(STOP_REASON_YAW_PROTECTION)`，不受功能开关约束。`route_config` 新增四套 `fork_entry_angle_*[]`（100项）与 `ROUTE_ACTUAL_FORK_*`：预赛正向填入 52 叉参考航向，其余三条路线全 0 预留、实际叉数置 0 暂不生效。`fork_exit_pair_prelim_forward` 不使用穿出边界对功能，数组全 0(INVALID) 预留。
37. **移除全部叉号特例并新增预赛正向叉40盲区**: 删除 state_control.c 中 Final Rev 叉8视觉跳号（`#if 0`）、叉18→19转弯态交接、Final Fwd 叉19上右入口限制与叉20→21直行态交接全部特例及对应契约测试（`test_fork18/19/20_handoff_contract.py`）；新增 Pre Fwd `fork_num=40` 盲区特例：Fork矫正关闭时非直角状态持续循右线（`caculate_error` 强制 `error=R_result`），仅接受左右三叉（`fork_dbg_src_L==2 && fork_dbg_src_R==2`）作为视觉入口，检测到时同帧 `fork_num=41` 并进入叉41，其他类型三叉不进三叉状态。同时删除 `state_control.c` 中已禁用的 `#if 0` 边界对匹配死代码，并更新 `test_route_config_contract.py` 对应契约断言。
38. **删除进叉时的LCD写入**: 移除 `state_enter_current_fork()` 中的 `ips114_show_int(0,70,config_index,2)`。该写操作在每帧视觉管线中间插入阻塞 SPI 事务，把最重的入叉帧拖过 100fps 帧预算导致画面丢帧卡顿（菜单里会被整幅图像覆盖、运行中又与 CPU0 屏显 SPI 竞争）。IMG 菜单 FRK 行仍显示 fork_num，功能不受影响。
39. **穿出表生效**: `state_update()` 视觉入口接入 `route_fork_exit_pair_matches()`。仅当该叉在 `fork_exit_pair_*[]` 配置了穿出对（≠INVALID）时才强制匹配，表空不强制（避免空表阻塞入口）；Pre Fwd 叉40盲区入口豁免穿出对匹配。此前 `#if 0` 里的死代码改为生效代码，`test_route_config_contract.py` 新增入口闸门契约测试。预赛正向 52 叉穿出对由用户提供并填入 `fork_exit_pair_prelim_forward`（1=左上 2=上右 3=左右，叉40盲区跳过、叉41=左右）。
40. **直角转向前瞻已撤销**: 曾新增 `angle_front_point` 把直角状态 `front_point` 改为可调，用户确认不需要，已回退为原 `min(L_point,R_point)`，删除 Angle 菜单行与 `O_ANGLE_FRONT` 槽。直角仅保留检测前瞻 `angle_detect_lookahead`（见 item 41）。
41. **直角检测前瞻生效并可调**: 修复 `check_sharp_turn_state` 的 `lookahead_y`（原传入20但被 `(void)` 忽略）——现在要求两侧爬虫同侧出边且出边点 y >= 该值才判直角，解决"识别太早"（拐角还很远就在图像顶端触发）。新增 `angle_detect_lookahead`（默认20，范围0～100），IMG→FWD 新增 Detect 行可调，Flash 参数页追加 `O_ANGLE_DETECT` 槽并持久化。
42. **偏航保护暂时关闭**: 预赛正向叉20、叉27处入叉实测航向与配置参考航向差 >90°，车跑对方向仍被误停车（陀螺仪正确，是参考表值与实际不符）。`ROUTE_ACTUAL_FORK_PRELIM_FORWARD` 从52置0，观测器对该路线全部叉直接跳过；代码与参考表保留，待修正后恢复。

---

## 十、文件依赖关系图

```
cpu0_main.c ──→ initialize.c/h (init_mian) ──→ motor.c/h, code/menu/menu.c, ...

CPU1图像帧:
mt9v03x_image
    ↓ route_progress_begin_frame()（冻结本帧里程与逻辑事件阶段）
    ↓ otsuThreshold_fast() → binarization()
    ↓ Dashedline_Makeup()（逻辑事件配置 → route_config 补线配置）
    ↓ search_line_main() [maze.c]
      ├── findline_twoside()（8邻域双爬虫）
      └── calc_side_distance_trend()
    ↓ state_update() [state_control.c]
      ├── count_valid_exits() [element_line.c] → 三种合法有序穿出边界对
      ├── get_continuous_yaw() [MahonyAHRS.c] → 三叉/直角相对转角退出
      ├── route_progress入口许可/红区强制进入/红区内保持/蓝窗末端强制退出
      ├── route_config预期边界对匹配 → 视觉三叉入口 → trajectory_policy编码 → INS_Relocalize_At_Flag()
      ├── 强制三叉入口只改状态，不执行里程重定位
      └── 统一进入/退出提交 → fork_num、angle_num与逻辑事件同步
    ↓ caculate_error() [line.c] → route_progress_get_config_fork_index() → route_config

CPU0陀螺中断(2ms):
IMU660RC原始陀螺 → 物理量转换/静态零偏扣除 [MahonyAHRS.c]
    ├── GZ(°/s) → steering_fuzzy_pd_control() 的GKD阻尼
    ├── 梯形积分 → continuous_yaw → CPU1 state_update()
    ├── 绝对值峰值 → GZMax（发车清零并内部保留，不再屏显）
    └── rad/s + 加速度计 → Mahony四元数 → Pitch/Roll/Euler Yaw显示

CPU0控制中断(2ms):
return_error() → combined_control() [motor.c]
    ├── Speed开启 → INS_Get_Target_Speed()
    │     ├── Path_Memory[Current_Path_Index].target_speed
    │     └── filtered_error → trajectory_error_speed_cap() [trajectory_policy.c]
    ├── DRS开启 + 长MaxV平台位图（尾部20000 ticks恢复区）+ 实时MaxV → 负压降至55%，否则恢复基础负压
    ├── Speed关闭 → basic_speed
    ├── 每4ms steering_fuzzy_pd_control() [tool.c] → 固定输出限幅 → 0.2/0.8直接差速
    ├── speed_control_L/R()（PosPID + 前馈）
    └── INS_Update()（所有模式累计里程/低通error；Record额外记录区间峰值与元素）

轨迹保存/加载（菜单上下文）:
Path操作 / Record停车提示
    └── inertial_navigation.c
          ├── 离线规划 → trajectory_policy + route_config
          ├── sector0 page5~10 轨迹载荷
          ├── sector0 page0 TRJ2元数据 + CRC32
          └── 验证成功 → route_progress_rebuild_map() → map_event + 逻辑三叉；失败 → NO MAP/INVALID

Set发车:
    ├── Record开启 → 清空旧图并按原始视觉逻辑录图
    ├── Speed或Fork开启 → 校验当前RAM地图；无效则NO RAM MAP
    ├── Fork开启 → 额外校验逻辑三叉窗口；非法则FORK MAP ERR
    └── 所有开关关闭 → 基础速度 + 原始视觉三叉逻辑
```

---

## 十一、编码注意事项

1. **所有源文件必须 UTF-8 编码**, 禁止 GBK/GB2312
2. 代码中存在 GBK→UTF-8 双重编码导致的乱码, 编辑时如 Edit 工具无法匹配乱码文本, 使用 Write 工具重写整个文件
3. **阈值修改必须记录到 `thresholds.md`**, 包含: 改了什么、为什么、日期
4. 上传代码前必须在提交信息中记录功能改动, 并询问"这个版本跑得怎么样?"
5. 中断优先级不能相同 (TC264 有 255 级, 数值越大优先级越高)
6. **用户长期偏好**: 对范围明确的小型改动，在无更高优先级流程强制时，确认关键参数后直接实现，不再单独创建设计文档
