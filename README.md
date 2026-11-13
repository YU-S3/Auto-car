# 文杰杯疯狂电路视觉智能车

硬件：TC264_BTB/TC264D、MT9V034/120°、130有刷双电机差速、DRV8701E双驱V1.3、双1024线编码器、IMU660RB、IPS114。SRAM规格240KB，保守算法数据预算≤96KB；首版单CPU应用，C算法供PC与MCU共用。

- [硬件型号与保守计算资源](docs/hardware-components.md)
- [软件架构：源码、模块、接口与调度预算](docs/architecture/system-architecture.md)
- [2026文杰杯完赛要求与当前差距评估](docs/assessment/2026-11-14-finish-readiness.md)
- [第21届疯狂电路组规则补充与适用范围](docs/research/2026-11-14-national21-rule-baseline.md)
- [已下载的Soberup源码基线及模块选择](soberup/README.md)
- [全部资料与实施计划](docs/README.md)

Soberup main锁定提交`7abfdac0d0156576ba5186716a7c1e42992359ca`，源码、依赖、许可证及校验清单保存在根目录`soberup/`；当前不保留ZIP。707个文件的完整Git树已校验；本车适配、ADS编译和实车运行尚未验证。

在根目录执行`python tools/vendor_soberup.py`可离线重新校验锁定源码，无需ZIP，发现修改不会覆盖原文件。上游快照保持原样，后续算法移植在项目应用层进行。

2026-11-14评估：用户确认宣讲PPT规则无调整；当前仍是参考源码基线，尚不能认定具备三科完赛能力。优先补齐本车硬件适配、约13秒固定停车逻辑、路线与真实返回判定、帧有效期及实车验收。架构中的新目录和接口为待实现设计，不代表固件已经完成。
