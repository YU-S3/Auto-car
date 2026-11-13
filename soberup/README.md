# Soberup main：锁定的源码基线

状态：**已下载、完整性已校验的上游源码基线；尚未完成本车移植、ADS编译或实车验证。** 后续以本目录中的实际源码为阅读依据，不依赖在线分支继续变化。

初次阅读可先看[项目结构与算法调用讲解](../docs/soberup-project-walkthrough.md)。

整理后的[本车软件架构](../docs/architecture/system-architecture.md)区分当前源码与待实现接口；[2026文杰杯完赛评估](../docs/assessment/2026-11-14-finish-readiness.md)列出硬件、固定超时、路线/返回判定与实车验收缺口。用户已确认宣讲规则无调整，当前参考基线仍不等于本车完赛固件。

## 1. 版本和下载范围

| 项目 | 锁定值 |
| --- | --- |
| 上游 | [Mantou-WYH/soberup-2026-public](https://github.com/Mantou-WYH/soberup-2026-public) |
| 分支 | main，N30pro视觉版；未混入inav |
| 提交 | `7abfdac0d0156576ba5186716a7c1e42992359ca` |
| 完整Git树 | `0a17400d274d55768758f326566368ad1c4b23c6` |
| 下载日期 | 2026-11-13 |
| 源文件数量 | 707个；包含工程和依赖，不是707个自有算法文件 |
| 解压后文件总字节数 | 18,460,349B |
| 原始ZIP | 当前不保留压缩包；原下载包3,022,993B，其SHA-256保留在清单中 |
| 原样源码目录 | [upstream/](upstream/) |
| 完整清单和校验值 | [upstream-manifest.json](upstream-manifest.json) |
| 复用模块清单 | [baseline-selection.json](baseline-selection.json) |

保留完整main快照是为防止只拷几个C文件后缺少头文件、库和工程定义。算法优先使用范围由下表和selection清单限定；源码下载完整不代表所有模块都会接入本车。

校验方法：ZIP成员CRC检查；逐文件Git blob SHA-1和SHA-256；从所有路径、模式和blob重新计算Git树，与已锁定提交的Git树完全相同。未运行上游代码或构建脚本。

## 2. 本地阅读与复用顺序

| 顺序 | 文件 | 用途 | 本车处理 |
| --- | --- | --- | --- |
| 0 | [CODE_SUMMARY.md](upstream/CODE_SUMMARY.md)、[README.md](upstream/README.md) | 上游架构和源码导览 | 历史工具可能未随归档提供，按本地文件核对 |
| 1 | [code/image.c](upstream/code/image.c)、[image.h](upstream/code/image.h) | OTSU、分块亮度补偿、二值图 | 优先迁移算法；把输入、尺寸和配置从SDK全局变量中分离 |
| 2 | [code/maze.c](upstream/code/maze.c)、[maze.h](upstream/code/maze.h)、[tool.c](upstream/code/tool.c) | 边界跟踪、线段与几何辅助 | 加入图像边界、迭代上限和退化图像回归 |
| 3 | [code/element_line.c](upstream/code/element_line.c) | 出口、路口、元素判断 | 用电路器件和三端节点的本车图像验证 |
| 4 | [code/dashedline_makeup.c](upstream/code/dashedline_makeup.c) | 有界横纵补线 | 重做120°镜头和安装高度尺度表，避免误连相邻支路 |
| 5 | [code/line.c](upstream/code/line.c) | 目标线与循迹偏差 | 明确坐标、单位、置信度；显示函数留在适配层 |
| 6 | [code/state_control.c](upstream/code/state_control.c) | 三叉/直角状态与进入退出 | 接入新事件接口、超时和冷却；不保留RC硬件依赖 |
| 7 | [code/route_config.c](upstream/code/route_config.c)、[route_progress.c](upstream/code/route_progress.c)、[map_event.c](upstream/code/map_event.c)、[trajectory_policy.c](upstream/code/trajectory_policy.c) | 已知地图动作、事件及进度 | 参考表达方式；替换本校地图，不视为通用在线规划器 |
| 8 | [code/motor.c](upstream/code/motor.c)、[encoder.c](upstream/code/encoder.c) | 双轮控制与速度反馈 | 参考结构，130电机、有效计数及引脚重标定 |

以上模块均是**可阅读、可选取并移植的基础源码**，当前仍依赖上游类型、全局变量和其他模块；不是已经可独立编译的纯C算法库。下一阶段在本项目应用层中逐个解耦，保留来源说明。

配套的`run_features`、`tuning_presets`及相应头文件已保留。IMU、初始化、屏幕菜单、Flash、入口和ISR也保留用于理解依赖，但不能原样接管本车硬件。

## 3. 必须适配的差异

| 差异 | 上游情况 | 本车基线 |
| --- | --- | --- |
| 电机与GPIO | N30pro；motor.h使用P33组及NSLEEP | 两个130有刷电机＋DRV8701E双驱V1.3；P11.9—P11.12映射待确认 |
| IMU | `imu660rc_*`，部分逻辑固定2ms积分 | IMU660RB / LSM6DSRTR；RB驱动＋本车时间戳、零偏和轴向 |
| 编码器 | A＋DIR，TIM4/TIM6，固定尖峰阈值和计数/周期假设 | 按接线选A＋DIR或AB；有效count/rev、周期、符号及溢出实测 |
| 相机 | 188×120，补线标定引用16cm高度 | MT9V034/120°；同分辨率但安装和尺度重新标定 |
| CPU | 上游已有双核分工 | 本项目保守从一个CPU执行算法开始，后续根据时序决定分核 |
| 地图 | 固定赛道动作和参数表 | 本校路线、元器件唯一ID、漏件30秒收益和合法返回策略另做 |

## 4. 两份逐飞库版本分别管理

- 本快照随附`libraries/doc/version.txt`首行为**V3.4.1**。这是原工程依赖，不覆盖它。
- 前期另行核验的官方TC264库是**V3.4.6**，提交`9adccd9caa536ce6bfb5deea4107e186c65c77b9`，目前只保留资料核验记录，没有在本目录假装已升级。
- 新应用工程接入官方库时，比较API和配置后逐模块迁移Soberup算法；不同时加入两份`libraries`到同一链接工程。
- `upstream/`作为原样对照，不直接修改；后续适配在项目应用层独立进行，构建产物也放在上游快照目录外。

## 5. 许可与文件编码

上游自有代码声明GPL-3.0-or-later：[LICENSE](upstream/LICENSE)。随附逐飞/iLLD等分别适用原许可，保留[THIRD_PARTY_NOTICES.md](upstream/THIRD_PARTY_NOTICES.md)及文件头。复制模块时同时记录上游提交、文件路径和修改说明。

源码按原始字节保存；部分文件采用GBK/GB18030而非UTF-8。阅读工具应按文件识别编码，不批量转码`upstream/`，避免破坏可校验快照。

## 6. 本地验证与下一步

在项目根目录运行：

```powershell
python tools/vendor_soberup.py
```

已有源码及清单时脚本离线校验所有源码和完整Git树，不要求ZIP存在，也不会因ZIP缺失自动重新下载。发现源码修改、缺失或额外文件会报错。只有没有本地清单、需要首次获取时才下载锁定版本；压缩包如需保存，位于`soberup/archive.zip`，不创建third_party目录。

接下来按[TC264适配计划](../docs/superpowers/plans/2026-11-13-tc264-v03-porting-plan.md)完成H0—H3；算法内存与计算预算以[硬件型号文档](../docs/hardware-components.md)为准。不能把下载完成表述为已经可在本车直接刷机参赛。
