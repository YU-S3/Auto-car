# TC264_BTB＋MT9V034硬件v0.3：第二轮仓库筛选

更新日期：2026-11-13。本文是[初筛报告](2026-11-13-open-source-repository-screening.md)的当前结论，取代其中“硬件型号未明确”条件下的排序；初筛源码和许可证记录继续保留。用户已认可方案，Soberup main已[下载为本地锁版基线](../../soberup/README.md)，完整源码树哈希校验通过；器件总SRAM240KB及保守资源统一见[硬件型号文档](../hardware-components.md)。

## 1. 推荐组合

**底层以逐飞官方TC264库为基础，算法以Soberup main为首选参考；仅迁移经过PC回放验证的算法模块。** 这比整份烧录比赛归档更符合本车TC264_BTB、130电机、DRV8701E和IMU660RB的组合。

| 层级 | 仓库/分支 | 与本车匹配 | 当前选择 |
| --- | --- | --- | --- |
| P0底层基线 | [seekfree/TC264_Library，master](https://gitee.com/seekfree/TC264_Library) | 同MCU、ADS；实际含MT9V03X、RB、IPS114、编码器/PWM驱动 | **采用为新工程基线候选**；锁定提交并核对本板走线，尚未编译 |
| P0相机资料 | [seekfree/MT9V03X_Product](https://gitee.com/seekfree/MT9V03X_Product) | 官方仓库说明MT9V034，全局快门 | 采用为相机手册/配置资料入口；不是整车算法库 |
| P1核心算法 | [Soberup main](https://github.com/Mantou-WYH/soberup-2026-public/tree/main) | 同TC264D、灰度摄像头、差速、同赛题；已有补线和真实三叉处理 | **首选算法参考**；RB接口、GPIO、编码器、标定和路线必须适配 |
| P2延后定位 | [Soberup inav](https://gitee.com/sandcarving-netizens/soberup-2026-public/tree/inav) | 同MCU家族，含视觉辅助定位 | 视觉基线稳定且传感器/地图规则确认后再评估，不与main混抄 |
| P2同题对照 | [l-f-yang/fei](https://gitee.com/l-f-yang/fei) | 同题、灰度图、差速，但TC387/N30与本车不同 | 从整车候选降为图像/状态机/控制方法对照，不移植其链接和多核工程 |
| P3基础原理 | [SJTU-AuTop/RT1064-Code](https://github.com/SJTU-AuTop/RT1064-Code) | C视觉方法可研究，MCU和赛题不同 | 只阅读局部阈值/爬线/点集/十字方法，先厘清许可 |
| P3控制学习 | [Enterprise_E](https://github.com/ittuann/Enterprise_E) | 有控制资料；无公开完整摄像头部分，底盘不同 | 控制理论对照，不作底座 |
| 暂不采用 | 树莓派、ROS、Loongson/OpenCV整车、OpenART模型、TC264 FreeRTOS重构 | 平台、执行依赖或任务不匹配 | 本车首版不引入这些运行环境；PC仍可用OpenCV辅助回放 |

选型已经获认可；Soberup main已导入第三方原始快照目录，但尚未集成到本车应用、编译或刷写固件。随附SDK为V3.4.1，官方V3.4.6是另行核验的后续底层适配目标，二者不混淆。

## 2. 官方库核验结果

逐飞[TC264官网文档](https://www.seekfree.com.cn/产品资料/核心板/英飞凌系列/tc264核心板/)明确指向TC264_Library及ADS/DAS资料。通过公开API读取官方完整目录树，返回13,666项且未截断；只把相关文件索引和已读文本保存到[本轮证据](hardware-v03-repo-evidence.json)，不保存无关SDK大文件。

已锁定核验快照：`9adccd9caa536ce6bfb5deea4107e186c65c77b9`，版本文件V3.4.6。其基础ADS工程在`Seekfree_TC264_Opensource_Library/Seekfree_TC264_Opensource_Library/`，不是仓库顶层。

| 需求 | 官方文件/例程 | 判断 |
| --- | --- | --- |
| 相机并口采集 | `libraries/zf_device/zf_device_mt9v03x.[ch]` | 已读源码：DMA、帧完成标记、188×120默认尺寸和配置接口；可用作底层 |
| RB传感器 | `libraries/zf_device/zf_device_imu660rb.[ch]` | 已读源码：RB专用读写与比例转换；不需要从RC库改寄存器来模拟RB |
| 编码器 | `libraries/zf_driver/zf_driver_encoder.[ch]`；`Example/Coreboard_Demo/E07_encoder_demo` | 已读接口及初始化：dir/quad区分；读取int16；例程目录已核实 |
| 有刷PWM与定时 | `Example/Coreboard_Demo/E04_pwm_demo`、`E05_pit_demo` | 已核实目录，不等于完整DRV8701E双驱V1.3示例；需板级组合PWM＋GPIO |
| IPS114 | `libraries/zf_device/zf_device_ips114.[ch]` | 已读源码：小缓冲按行写屏；官网资料列ST7789V手册 |
| 双核 | `E13_dual_core_demo` | 仅在单核时序达标后借鉴同步方法，不默认分核 |
| 段布局 | `E14_specifies_variable_or_code_location_demo`、`.lsl` | 大数组定点分配与.map核对的参考，不把240KB当普通堆 |

基础库不是本题的器件识别、覆盖规划或最优路径算法库。这些应用层仍须按本校PPT和最后规则实现。逐飞[演示车模目录](https://www.seekfree.com.cn/微信推文/演示车模/)也列有疯狂电路浅析，但本轮微信正文直连不可读，因此只记录官方入口，没有把二手摘要当作已核验算法或另一个完整开源仓库。

### 引脚和标定的边界

相机头文件默认DMA5、P02.1/P02.0、P00.0—P00.7；RB默认SPI0的P20组；IPS114默认SPI2的P15组。**这些均为源码默认，不是本车V1.1的已确认接线。** 本轮未找到能确认完全匹配用户V1.1缩微主板的公开原理图，不能将V1.0 MINI板或别人的自制板等同替代。

相机初始化可能走SCCB或UART；标注TXD/RXD不意味着像素通过串口发送。50FPS是官方该版本默认配置，不代表本车实测50FPS；驱动注释中的3.8ms采集时间也是历史注释，不能当验收数据。

## 3. Soberup的匹配项与必须修改项

以下是重新读取实际源码后确认的差异，不仅依据README。

| 部位 | 源码证据 | 对本车的处理 |
| --- | --- | --- |
| IMU | [MahonyAHRS.c:2,18,58-63](https://github.com/Mantou-WYH/soberup-2026-public/blob/main/code/MahonyAHRS.c#L2)使用`zf_device_imu660rc.h`、`imu660rc_*`及固定2ms积分周期；[initialize.c:27](https://github.com/Mantou-WYH/soberup-2026-public/blob/main/code/initialize.c#L27)初始化RC | 用官方RB驱动，经适配层转rad/s；重新定ODR/零偏/坐标/时间戳。不能仅全局替换RC字母为RB |
| 电机 | [motor.h:33-41](https://github.com/Mantou-WYH/soberup-2026-public/blob/main/code/motor.h#L33)使用P33.9—P33.12及独立NSLEEP控制 | 本车给定候选是P11.9—P11.12；查接线后重新选可用PWM通道和DIR，确认是否有使能/睡眠脚 |
| 编码器模式 | [encoder.c:9-11](https://github.com/Mantou-WYH/soberup-2026-public/blob/main/code/encoder.c#L9)实际是`encoder_dir_init`；[encoder.h:4-10](https://github.com/Mantou-WYH/soberup-2026-public/blob/main/code/encoder.h#L4)选TIM4和TIM6 | 它使用A＋DIR，不是TIM6的AB完全正交例子；本车如果接AB，应重新分配支持的定时器和引脚 |
| 编码器算法 | [encoder.c:18-44](https://github.com/Mantou-WYH/soberup-2026-public/blob/main/code/encoder.c#L18)固定尖峰阈值300、5ms文字假设、读取后清零及滤波 | 不复制阈值和轮距/计数比例；负轮速用`+0.5f`再截断有非对称风险，双向运动必须单测；读清计数也需核对边沿丢失问题 |
| 相机与补线 | [dashedline_makeup.c:7,22,38,54](https://github.com/Mantou-WYH/soberup-2026-public/blob/main/code/dashedline_makeup.c#L7)为188×120、16cm高度的行尺度表；路线控制补线模式 | 尺寸兼容但120°镜头和安装几何未知，所有尺度表重标定；不要对平行线路误补线 |
| 地图 | [route_config.c:5](https://github.com/Mantou-WYH/soberup-2026-public/blob/main/code/route_config.c#L5)固定路线和转向表 | 替换为本校地图；明确预录许可；元器件唯一ID、覆盖收益和回程预算另做 |
| 计算与内存 | [CODE_SUMMARY](https://github.com/Mantou-WYH/soberup-2026-public/blob/main/CODE_SUMMARY.md)说明CPU0/CPU1分工及其他模块 | 本车先单核运行、共享C核心；不能照搬整套数组、双核调度和Flash布局 |

[本轮Soberup证据](hardware-v03-soberup-evidence.json)保留相关行。主要收益仍是视觉、补线、真实路口和事件执行的经验，不能因此省略硬件适配。

## 4. 新发现候选：qinnnnnnnnnnn/the-21st-smart-vehicles

[仓库](https://gitee.com/qinnnnnnnnnnn/the-21st-smart-vehicles)通过公开API可读取，描述为第21届疯狂电路代码。目录树有1900项，实际包含`ZhuMeng_10/Seekfree_TC264_Opensource_Library/`工程、应用C源码、库和大量Debug构建产物；本轮读取了`code/H_motor.h`，接口为`Motor_Init/car_go/fuya`，并非只发现一个空仓库。

然而API未声明根许可证，本轮未核实完整软件授权；项目关联说明使用IMU660RC、130°镜头，和本车RB/120°仍有差异。其应用层尚未深审，不因同为TC264就提高到Soberup之前。**结论：保留为后续阅读候选，不纳入当前复制/移植清单。** [核验索引](tc264-additional-candidate-evidence.json)

新增检索的红外阵列/AS5040疯狂电路硬件项目虽然有TC264D和RB，但感知路线不是本车摄像头方案，且未确认可用视觉软件，不作为替代主线。

## 5. 许可、完整性与使用方式

- 官方TC264库的许可文本是GPL-3.0-or-later，厂商库和英飞凌组件分别保留原声明；不根据早年介绍的许可描述覆盖当前文件。
- Soberup自有代码GPL-3.0-or-later，与官方库一起使用仍应保留第三方通知。不是所有程序都已公开：历史路径生成/求解和部分测试材料不在归档。
- fei仍保留“立创MIT、仓库WTFPL、内含厂商许可”的区别；这次不扩大它在TC264工程中的用途。
- AuTop等缺少明确根许可的项目继续只作原理阅读，不能因为硬件现在明确了就默认获准复制。

本轮全部为读取、静态核验和文档更新；没有运行外部工程，没有改写生产固件，没有宣称通过TC264编译或实车验收。

## 6. 对实现计划的修改

1. 固定TC264D/ADS/官方库版本，取消STC/RT1064和舵机分支作为当前实现路线。
2. 固定188×120输入，以C核心在PC与板上共用；Python/OpenCV只用于采图、标注、参考和报告，不成为板端依赖。
3. 改为CPU0应用优先；视觉截止期不达标后才评估CPU1分担。分核前明确帧所有权和内存屏障，`volatile`不等于完整跨核同步。
4. 保留130电机双轮速度环；按实际轮端count、采样周期和有效轮距建立单位明确的差速控制。
5. 显示采用IPS114行刷新与降频，禁止默认全帧RGB565缓存及全图float映射表。
6. 内存按DSPR0/1和CSA/栈分区验收；不只检查“总数小于240KB”。

执行入口：[硬件基线](../hardware-profile.md)、[TC264适配补充计划](../superpowers/plans/2026-11-13-tc264-v03-porting-plan.md)、[原赛题算法计划](../superpowers/plans/2026-11-13-wenjie-vision-algorithms.md)。
