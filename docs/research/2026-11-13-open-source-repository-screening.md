# 文杰杯疯狂电路组：开源仓库筛选与复用建议

核查日期：2026-11-13。**本文保留第一轮筛选记录，当前推荐以[硬件v0.3第二轮筛选](2026-11-13-tc264-v03-repository-selection.md)为准。** 用户现已明确TC264_BTB＋MT9V034＋130差速＋IMU660RB，硬件基线见[hardware-profile](../hardware-profile.md)。下文“型号未提供”的条件和A1/A2排序属于初筛历史，不再作为当前开工条件。

本轮读取了仓库目录、关键源码、许可证及工程说明；打开并检查了一个24,678,146字节的比赛源码压缩包。**没有执行第三方代码，没有编译或进行实车验证。** 搜索引擎描述仅用于发现候选，推荐依据以项目自身文件为主。

## 1. 结论与推荐顺序

确实有高度接近的项目。建议优先研究两个同赛题工程：**Soberup 的 main 视觉分支**和**三江学院 l-f-yang/fei**。前者适合理解电路图白线、补线和三叉执行，后者有比较完整的比赛工程及分模块说明。AuTop适合作为基础视觉算法参考，Enterprise_E适合作控制算法参考。

| 优先级 | 项目 | 已核实内容 | 推荐用途 | 主要限制 |
| --- | --- | --- | --- | --- |
| A1 | [Soberup：Gitee](https://gitee.com/sandcarving-netizens/soberup-2026-public/tree/main) / [GitHub同名归档](https://github.com/Mantou-WYH/soberup-2026-public) | main/inav两分支、TC264D工程、图像/补线/三叉/路线源码 | 同赛题主参考，优先main | 固定路线、标定和姿态/里程依赖；不是完整在线探索系统 |
| A2 | [l-f-yang/fei](https://gitee.com/l-f-yang/fei) | TC387、N30Pro、比赛源码ZIP、图像/状态机/PID | 第二套同赛题实现，理解循迹到双电机控制链路 | 平台特定、应用文件较大；许可证说明存在差异 |
| B1 | [SJTU-AuTop/RT1064-Code](https://github.com/SJTU-AuTop/RT1064-Code) | 自适应爬线、点集处理、十字和Y路口、PID | 视觉基础算法与路口处理原理 | 不同赛题；没有查到根项目明确许可证 |
| B2 | [ittuann/Enterprise_E](https://github.com/ittuann/Enterprise_E) | PID、控制、编码器等；README明确排除摄像头部分 | 控制环、滤波和调参学习 | 不能提供本任务的核心视觉实现；全向底盘控制需重做映射 |
| C1 | [HaliFax-Desk/SmartCar21-Loongson](https://github.com/HaliFax-Desk/SmartCar21-Loongson) | Python视觉脚本、C++目录、视频/图像素材目录 | PC端快速比较算法和回放形式 | 走马观碑赛题；无明确根许可证；原型包含标为不使用的文件 |
| C2 | [SJTU-AuTop/LiveViewer](https://github.com/SJTU-AuTop/LiveViewer) | 配套上位机仓库，组织页说明其基于Qt5 | 借鉴数据展示与调试协议组织 | 协议需适配；未深读源码；无明确根许可证 |
| C3 | [SJTU-AuTop/Openart-Code](https://github.com/SJTU-AuTop/Openart-Code) | 三叉数字、AprilTag、动物水果模型及代码的项目说明 | 仅在后续有OpenART且需要分类时参考 | 目标类别与电路元器件不同；不是本赛题首选 |

等级含义：A为同赛题优先研究；B为有明确价值的模块参考；C为条件合适再用。未以星数或奖项代替源码核验。

## 2. A1：Soberup，最接近现有实现计划

**定位与架构。** TC264D + MT9V03X，总体为双核采集/视觉、状态机、控制和路线配置。仓库README明确其是停止维护的固化归档，测试脚本、实车记录及部分开发工具没有随仓库发布。[README](https://github.com/Mantou-WYH/soberup-2026-public/blob/main/README.md#L1)

Gitee API核实有main和inav分支；**Gitee默认分支是inav，GitHub默认分支是main**，打开后不能仅凭默认页判断是哪套方案。本轮还比对了两站main分支`code/image.c`，blob SHA均为`81cc173243603be118d296aa68a2af33ea03b8cc`，只说明该文件一致，不保证两个仓库所有分支和文件完全同步。

### 建议阅读的文件

| 文件与源码证据 | 已看到的实现 | 对我们的价值 |
| --- | --- | --- |
| [CODE_SUMMARY.md，第5—26行及后续代码导览](https://github.com/Mantou-WYH/soberup-2026-public/blob/main/CODE_SUMMARY.md#L5) | 平台、白线/真实三叉任务、188×120灰度图、模块职责 | 建立完整阅读地图；说明中出现的文件仍需核查是否公开 |
| [code/image.c，第20—42行、第468行、第538行起](https://github.com/Mantou-WYH/soberup-2026-public/blob/main/code/image.c#L20) | 6×4分块、全局OTSU、局部亮度补偿和阈值可信度处理 | 对照本车曝光/照明变化；先测简单阈值基线，再决定是否采用完整补偿 |
| [code/dashedline_makeup.c，第1—58行、第64行起](https://github.com/Mantou-WYH/soberup-2026-public/blob/main/code/dashedline_makeup.c#L1) | 按行线宽/地面尺度表限制横纵补线；是否补线由路线进度配置决定 | 很贴近器件断线；它不是对任意黑区都补白 |
| [code/state_control.c，第3—54行](https://github.com/Mantou-WYH/soberup-2026-public/blob/main/code/state_control.c#L3) | 三叉状态、里程窗口、陀螺仪退出、连续帧及防错路参数 | 可提取“入口确认—执行—退出”的结构，但依赖要重新设计 |
| [code/route_config.c，第5—160行](https://github.com/Mantou-WYH/soberup-2026-public/blob/main/code/route_config.c#L5) | 预赛/决赛、正向/反向的转弯类型、方向、穿出对和补线配置数组 | 参考固定地图的动作序列设计；必须换成本次校赛地图 |
| [code/maze.c](https://github.com/Mantou-WYH/soberup-2026-public/blob/main/code/maze.c)、[line.c](https://github.com/Mantou-WYH/soberup-2026-public/blob/main/code/line.c)、[motor.c](https://github.com/Mantou-WYH/soberup-2026-public/blob/main/code/motor.c) | 已核实目录中实际存在源码 | 下一轮深入入口，分别对应爬线、方向误差和控制；本轮未逐函数完成质量审查 |

### 不能直接照搬的部分

- main称为视觉版，是指主要方向控制来自视觉；源码仍有IMU、编码器和路线进度依赖，不能理解成“完全只用摄像头”。若校赛限制辅助传感器，需要逐处替换，不能只删除驱动文件。
- 补线代码文件头写明适配188×120图像及16cm相机高度，并含行尺度表。镜头、安装角度、高度改变后必须重标定。
- `route_config.c`是预定路线表。它不是自动识别整张新电路图并求最优路线的算法。
- `CODE_SUMMARY.md`目录说明中出现`path.py`、`path_solver.c`等历史开发材料，但本轮完整Gitee main目录树未发现这两个文件，文档开头也说明部分工具未发布。不能声称下载后已有完整规划器和测试集。
- `state_control.c`第46—47行的入叉防抖和重入延迟默认值均为0；这不证明必然有错，但移植时不能认为它已经满足我们计划中的多帧防抖要求。

**许可证。** README明确自有代码GPL-3.0-or-later；[THIRD_PARTY_NOTICES.md](https://github.com/Mantou-WYH/soberup-2026-public/blob/main/THIRD_PARTY_NOTICES.md)列出逐飞和英飞凌组件的各自许可。复制代码进入最终工程前按实际使用文件保留声明并核对分发义务。

**建议。** 先研究main，提取视觉/补线/事件和路线配置的设计方法。inav包含二维定位、地图匹配等更多模块，等辅助传感器许可及工期明确后再评估，不应一开始混合两个分支。

## 3. A2：l-f-yang/fei，第二套同赛题完整工程

**实际检查范围。** 通过Gitee API读到README、LICENSE、完整文件树和OTSU示例；下载源码ZIP到内存后只做解包阅读，未执行。压缩包24,678,146字节，共2,486项，应用层C/H有10个文件，此外大量条目为SDK/工程/构建资料，不能把总条目数当算法规模。

压缩包位置：[软件/比赛源码.zip](https://gitee.com/l-f-yang/fei/tree/master/软件)，内容根目录为`6_25_car/`。本轮核实源码压缩包blob SHA为`c083bd8d68d4d88782099edc6ef375818b22cd7c`。

README第28—38行说明TC387、N30Pro有刷差速和无刷负压、总钻风、编码器及陀螺仪；第49—106行描述视觉、多层状态机和双环控制。[README入口](https://gitee.com/l-f-yang/fei/blob/master/README.md)

| 压缩包内路径与位置 | 已读取到的内容 | 可利用部分 |
| --- | --- | --- |
| `6_25_car/code/image.c:325`、`:416`、`:448` | OTSU、分块和阈值计算；文件共1479行 | 二值化基线和图像输入格式 |
| `6_25_car/code/image.c:801`、`:889` | 基点选择、图像处理入口 | 跟线和失效回退的具体阅读入口 |
| `6_25_car/code/image.c:1299`、`:1398` | 补线中线、`Err_Sum`加权误差 | 从视觉结果接方向控制的链路 |
| `6_25_car/code/find.c:78`、`:159`、`:406` | 保护停车、速度斜坡、整车状态分发；文件共660行 | 行驶/转弯/冷却的状态组织 |
| `6_25_car/code/pid.c:27`、`:161`、`:231`、`:342` | 状态清理、左右轮速度控制、方向控制；文件共381行 | 差速底盘的控制起点 |

### 核查出的限制

1. 图像和状态机有直接耦合，数组和函数使用120×188等固定尺寸。若主控不同，适配工作包括头文件、变量类型、内存位置、多核同步和驱动，不能只改引脚。
2. 目录中独立算法说明不等于完整比赛逻辑。以`软件/算法详细设计/大津法/OSTU.c`为例，第14行阈值未初始化，第56行分母`PK[i]*(1-PK[i])`未排除0，第66行直接返回阈值。全黑/全白等退化输入有风险，不能不加测试直接使用。这是对该示例的静态发现，不扩大为整份工程已经全面审计。
3. 本轮确认有固定逻辑和地图相关组织，但没有确认存在通用自动拓扑探索、唯一元器件计数和按本次30秒漏件罚时优化的完整实现；这些仍需单独建设。
4. **许可证冲突需在复用前厘清：** 立创项目页标MIT，实际Gitee根`LICENSE`为WTFPL v2，压缩包还包含`libraries/doc/GPL3_permission_statement.txt`。不能把整包当MIT。根文件的宽松声明也不能覆盖第三方库。

**建议。** 用作第二套视觉—状态机—控制的对照案例，特别适合TC387差速车。先按模块学习和验证，避免与Soberup整个应用层拼接；两套状态机、尺度表和参数并不兼容。

## 4. B1：SJTU-AuTop/RT1064-Code，基础视觉算法参考

**实际结构。** `Libraries/`和`Project/`；`Project/CODE/`有图像处理、十字、Y路、圆环、车库、控制、调试等模块，工程目录提供IAR和MDK入口。README明确自制主板引脚和逐飞主板不同。[README](https://github.com/SJTU-AuTop/RT1064-Code#readme)

值得读的关键位置：

- [imgproc.c:50](https://github.com/SJTU-AuTop/RT1064-Code/blob/master/Project/CODE/imgproc.c#L50)：自适应阈值。
- [imgproc.c:247](https://github.com/SJTU-AuTop/RT1064-Code/blob/master/Project/CODE/imgproc.c#L247)、[imgproc.c:290](https://github.com/SJTU-AuTop/RT1064-Code/blob/master/Project/CODE/imgproc.c#L290)：左右自适应边界跟踪。
- [imgproc.c:505](https://github.com/SJTU-AuTop/RT1064-Code/blob/master/Project/CODE/imgproc.c#L505)、[imgproc.c:576](https://github.com/SJTU-AuTop/RT1064-Code/blob/master/Project/CODE/imgproc.c#L576)、[imgproc.c:597](https://github.com/SJTU-AuTop/RT1064-Code/blob/master/Project/CODE/imgproc.c#L597)：等距重采样、局部角度、非极大值抑制；适合学习拐点和出口特征。
- [cross.c:51](https://github.com/SJTU-AuTop/RT1064-Code/blob/master/Project/CODE/cross.c#L51)、[cross.c:128](https://github.com/SJTU-AuTop/RT1064-Code/blob/master/Project/CODE/cross.c#L128)：十字处理与远端续线。
- [yroad.c:29](https://github.com/SJTU-AuTop/RT1064-Code/blob/master/Project/CODE/yroad.c#L29)：Y路检测/运行状态。
- [pid.c:4](https://github.com/SJTU-AuTop/RT1064-Code/blob/master/Project/CODE/pid.c#L4)：简洁PID实现，适合理解接口；最终要另测限幅、积分及停机行为。

**适配差异。** 这是第16届智能视觉组工程，三叉数字和AprilTag等任务不同于电路器件图案。传统赛道的双边界假设不一定适用于窄白线和器件符号，不能只反转二值化颜色就宣称适配成功。

**许可证与结论。** GitHub元数据未识别许可证，根目录列表也未见LICENSE。本轮没有查到对整个项目的明确授权文件，因此列为优先阅读的算法参考；代码复制和再分发前需厘清作者及第三方许可。该结论不等于认定所有文件都没有许可。

## 5. B2与C类的使用边界

### Enterprise_E

[README第3行](https://github.com/ittuann/Enterprise_E/blob/main/README.md#L3)明确只开放“除摄像头部分外”的工程。实际`CODE/`中有`pid.c`、`control.c`、`encoder.c`等，不能因为有`camera.c`就认为完整图像算法已公开。根LICENSE为Apache-2.0，第三方仍单独核对。

建议读其控制博客、PID和滤波思路；不把全向轮电机混控移到普通四轮差速车。对当前视觉、器件和拓扑覆盖任务的缺口帮助有限，所以不选为主框架。

### SmartCar21-Loongson

已核实`LineDetect/Python_Script/scripts/`有`2ROI_Scan.py`、`HALF_8Area.py`、`HALF_Frame_8Area.py`、补线、拐点及二值/Canny中心线脚本，还存在名为`NOT_USE(FULL8Area_Test).py`的文件。[目录](https://github.com/HaliFax-Desk/SmartCar21-Loongson/tree/main/LineDetect/Python_Script/scripts)

适合PC端比较，不代表已经具备MCU适配。目录也有录制素材和C++试验，未逐个验证其效果。未查到明确根许可证，先作为阅读参考；不能依据文件名里的“CAN USE”当作实车可靠性证明。

### AuTop配套仓库

[LiveViewer](https://github.com/SJTU-AuTop/LiveViewer)可作为调试工具参考，使用前先核对图传协议和许可；本轮只核实定位和元数据，没有深度代码审计。[Openart-Code](https://github.com/SJTU-AuTop/Openart-Code)说明其模型用于数字、AprilTag、水果动物，与本题元件类别不同，本阶段不投入训练部署。

## 6. 未选为软件主线的候选

| 项目 | 本轮结论 | 原因/依据 |
| --- | --- | --- |
| [Rh-i/TC264_FreeRTOS_Demo](https://github.com/Rh-i/TC264_FreeRTOS_Demo) | 暂不采用为整车底座 | README已知问题含遥控断联失控、串口发送重入卡死及双核待完善项；任务也不同。可读分层结构，不为本次比赛额外引入RTOS重构 |
| [南京信息工程大学疯狂电路车模](https://oshwhub.com/xunmujun/project_qupgzwff) | 主要转给硬件队友 | 项目作者说明自己负责硬件；网页抓取403，仅检索到项目描述，未确认完整视觉源码，不能列为已核实软件仓库 |
| [成都大学疯狂电路车模](https://oshwhub.com/shiyimeng/project_chpihuwh) | 待核实附件，不进入软件短名单 | 页面抓取403，未确认应用算法源码的可访问性与完整性 |
| [RaspberryCar](https://github.com/Mingrui-Yu/RaspberryCar)、[AutoRun-Car](https://github.com/VendettaMask/AutoRun-Car) | 低优先级候选 | 公开项目描述主要为树莓派巡线/避障，运行环境和电路组要素差异大；未深读源码，不把它们当可移植完整固件 |
| ROS导航、真实道路YOLO车道线、虚拟推箱子视觉组 | 不纳入当前主线 | 与本次电路图窄白线、元器件和支路覆盖的目标不同，不能因名称带“智能视觉”就认为同赛题 |

上述“未采用”是当前项目匹配判断，不是对项目整体质量的否定。

## 7. 与当前项目的对应和复用顺序

本地已检查全目录，目前只有PPT、资料和实现计划，没有固件和视觉实现。`docs/superpowers/plans/2026-11-13-wenjie-vision-algorithms.md:5`定义的是计划架构，文档开头明确尚未实现，第45行开始列出建议文件结构。因此下面全部为“计划吸收/参考”，不是现有代码已经具备功能。

| 本地计划模块 | 优先外部参考 | 选择 | 仍须完成 |
| --- | --- | --- | --- |
| `vision/preprocess` | Soberup `image.c:20`；fei示例OTSU | 先做简单基线，再按失败场景吸收复杂分块 | 本车曝光、内存和全黑/全白等退化输入测试 |
| `vision/track` | AuTop `imgproc.c:247/290`；Soberup maze目录 | 参考爬线与点集表达 | 窄白线、符号双边缘、越界终止和循环上限 |
| `vision/elements` | Soberup `dashedline_makeup.c:54` | 优先研究有界补线和出口约束 | 本车尺度表；禁止误连平行支路；器件通过检测对照 |
| `navigation/fsm` | Soberup `state_control.c:22`；fei `find.c:406` | 选择一套结构改造，不混合状态变量 | 多帧确认、超时、重入冷却、丢帧和辅助传感器限制 |
| `navigation/planner` | Soberup `route_config.c:5` | 有地图且允许录入时参考动作序列 | 本校地图、唯一器件ID、30秒罚时、合法返回和在线模式 |
| `control` | fei `pid.c:161/231/342`；Enterprise_E PID | 选与车体匹配的闭环结构 | 周期、单位、极性、限幅、停机和电机参数 |
| `telemetry` | 逐飞助手；AuTop LiveViewer定位 | 优先使用现有兼容工具 | 帧协议、时间戳、非阻塞和竞赛通信许可 |

推荐的实际工作顺序：

1. 取得本车主控/相机型号和已有工程，匹配厂商SDK。先验证采图与电机，不换掉已经稳定的底层。
2. 读Soberup README、CODE_SUMMARY，再沿`image → maze → element_line → state_control → line → motor`跟踪一帧到电机的流程；本轮已深读其中部分文件，其余需继续逐函数阅读。
3. 对照fei的`image.c → find.c → pid.c`，选更适合本车的一套应用层组织；另一套只用于对照，不把两套全局变量拼接。
4. 用本车原始图像建立20—50个关键片段，先测试断线、器件伪分支、普通T和三端器件，再移植。
5. 在确认允许预录地图之前，不能把Soberup路线表当唯一解法。确认需要在线探索时，独立开发节点身份、拓扑、覆盖与返回逻辑。

工期判断：若实际是TC264D+MT9V03X，Soberup优先级进一步提高；若是TC387差速车，fei值得更早深入；若是STC或其他小RAM MCU，优先吸收算法原理并保留原厂底层，重新预算浮点、图像缓存及多核依赖。此处是工程推断，不能替代实际编译和实车测量。

## 8. 核验记录和结论边界

- GitHub使用已连接工具读取元数据、目录及关键源码；Gitee网页读取受限后，通过公开API读取目录、base64源码和ZIP。并未因网页失败而直接放弃核验。
- [Gitee核验记录](repo-audit-evidence.json)保存README/许可文本、目录、blob SHA和压缩包内关键行位置；[GitHub核验记录](github-repo-audit-evidence.json)保存候选元数据与文件SHA，便于后续锁定版本。
- 本报告源码行号对应本轮读取的分支快照，分支后续变化可能改变行号。Gitee API返回的tree字段`sha`有时只是分支名称，不能当不可变commit使用；锁版时应另记录实际commit。
- “仓库公开、能阅读”与“许可证允许直接复制、能在本车运行”分别核实。未明确许可的项目仅推荐阅读，未经移植和回归的工程不称为可直接参赛版本。
- 本轮不修改原实现计划的算法结论：这两个同赛题工程能明显减少循迹、补线和状态机的探索成本，但仍不能代替本校计分、覆盖规划及现场鲁棒性验证。
