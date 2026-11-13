# 文杰杯疯狂电路视觉车：资料入口

2026-11-14新增：

- [算法/硬件接入分工设计](superpowers/specs/2026-11-14-algorithm-hardware-worksplit-design.md)：按规则行为划分主责、现有模块复用范围、统一目录、跨侧数据契约和交接责任。
- [改进准备工作包](superpowers/plans/2026-11-14-algorithm-hardware-improvement-plan.md)：共同先行C0/C1、算法A0—A5、接入B0—B5，以及SW01—SW10负责人、依赖与M0—M4联合验收。第一批先做接口/PC回放与引脚/ADS/停机。

- [软件专项达标检查](assessment/2026-11-14-software-compliance-audit.md)：规则行为与源码逐项对应，列出10项软件问题；明确不以缺元件分类器、在线规划器或车端计分模块直接判违规。软件当前不能通过本车就绪验收。

- [第21届疯狂电路组规则补充基线](research/2026-11-14-national21-rule-baseline.md)：依用户补充核对全国总规则、赛道细则、问答和后期计分；区分校赛三科、全国单科、国赛75%，补齐IMU、重量、禁入边界与自制电路资格。

- [系统架构](architecture/system-architecture.md)：当前双核调用链、目标共享C架构、模块边界、接口单位、帧所有权、启动/停车状态、调度及240KB资源预算。
- [2026文杰杯完赛评估](assessment/2026-11-14-finish-readiness.md)：用户确认规则无调整后的要求矩阵、8项关键源码发现、从构建到三科有效返回的验收门槛。当前不能认定已达到完赛要求。
- [ADR-0001：共享C与保守基线](adr/0001-tc264-shared-c-baseline.md)：已认可方案及约束，区分接受设计和实现完成。
- [规则与公开资料核对记录](research/2026-11-14-competition-rule-check.json)：规则来源、确认状态和检索范围限制。

下列2026-11-13资料继续作为硬件、来源和实施计划依据；当前架构与就绪状态以本次新增评估为准。

更新日期：2026-11-13。当前硬件为TC264_BTB＋MT9V034/120°＋130有刷双电机差速＋DRV8701E＋双1024线编码器＋IMU660RB＋IPS114。SRAM总规格按用户确认的240KB记录。当前阶段：硬件型号与保守资源预算已落档，Soberup main源码快照已下载并校验；尚未完成本车固件集成、编译或实车验证。

建议按顺序阅读：

1. [硬件型号与保守计算资源](hardware-components.md)：元件型号统一入口；240KB总SRAM，算法数据≤96KB，单CPU执行算法，视觉30Hz初始目标。
2. [已下载Soberup源码基线](../soberup/README.md)：锁定main提交、38个C/H文件的分组选择、完整依赖、校验清单和移植边界。
3. [硬件v0.3开发核验与待实测项](hardware-profile.md)：用户选型、官方驱动默认值和本车接线明确区分，实际链接布局另行核对。
4. [当前仓库筛选：TC264硬件v0.3](research/2026-11-13-tc264-v03-repository-selection.md)：逐飞官方库打底，Soberup main作算法参考，列出RB/RC、GPIO、编码器和相机标定差异。
5. [TC264适配补充计划](superpowers/plans/2026-11-13-tc264-v03-porting-plan.md)：先做工程/引脚、独立驱动、帧所有权和内存验收，再迁移算法。
6. [赛题算法实现计划](superpowers/plans/2026-11-13-wenjie-vision-algorithms.md)：循迹、元器件、路口、覆盖规划和三科实车验收，已按v0.3更新。
7. [赛题与资料调研](research/2026-11-13-视觉组资料调研.md)：PPT页码、计分公式和资料来源。
8. [Soberup项目结构与阅读指南](soberup-project-walkthrough.md)：逐目录职责、双核运行链、算法模块、调用入口与本车重点。

[原始宣讲PPT](<“文杰杯”智能车挑战赛宣讲会5.24(1).pptx>)保留不变。[第一轮开源筛选](research/2026-11-13-open-source-repository-screening.md)保留作为历史调研，当前工程选型以第二轮为准。

本轮主要变化：SRAM规格统一为240KB，采用算法96KB＋系统预留96KB＋保护余量48KB的保守预算；Soberup main固定在提交7abfdac0d015，707个上游文件保存在根目录`soberup/upstream/`，保留许可证和校验清单，不保留ZIP；校验脚本支持无ZIP离线校验。原样随附SDK为V3.4.1，后续官方V3.4.6适配单独进行。
