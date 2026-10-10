# V0.5：模型、参数与证据入口

普通直行→左−30°漂移→固定10 s维持→连续换向→右+30°漂移→固定10 s维持→同步退漂→自然直行。五组使用Step37c同一控制器与直线道路参考坐标，均完成60 s并通过状态任务。外部ASB全零，不跟踪指定几何八字路径。

|操作|入口|是否启动车辆|
|---|---|---|
|阅读结果与限制|[综合报告](../../docs/v0.5/report.md)、[总表](../../docs/v0.5/summary.csv)|否|
|核对发布内容|根目录MANIFEST.sha256、source/publication_sources.json|否|
|原机准备冻结参数|initialize_v05.m|否|
|查看核心模型|model/stabilizer_bar_step37a.slx|否|
|复算完整时序|取得evidence/raw_data_index.json中的本地原始记录，再按研究脚本说明恢复依赖|否|

## 原机入口

需要Windows、MATLAB/Simulink R2023a和合法CarSim2020.0环境。MATLAB切换到本目录：

```matlab
v05_case='kf384_kr384'; % kf192_kr384 / kf576_kr384 / kf384_kr192 / kf384_kr576
initialize_v05;        % 校验原机sim/par哈希，加载参数，不仿真
open_system('stabilizer_bar_step37a');
% 仅在准备独立输出配置并核验后运行：
% out=sim('stabilizer_bar_step37a','StopTime','60');
```

初始化会写基础工作区，核心仍是model/a001_step37a_initialize.m；不求配平、不调增益。模型标识及回调保留原名，避免改名造成行为变化。入口引用原机已核查的研究sim/par；config里的.snapshot仅供追溯，不自动安装或改写GUI。

**不要直接用原输出路径重新运行。** 新实验必须独立输出，保留物理参数并核对新配置身份。临时目录可能被系统清理；缺文件时入口应失败，不能跳过哈希检查。换机器需合法CarSim安装、数据库及路径迁移，未验证即插即用。

## 文件职责和复核范围

- model、source/profiles、internal两项核心源码是Step37c逐字节发布副本。support内仍有兼容路径数据，但该支路不参与控制。
- source/research_scripts保存原研究脚本，含本机跨Step依赖及历史路径；它们是审计来源，不是任意下载环境中的批量运行入口。不要对全部历史目录genpath。
- evidence含配置身份、独立审计、实际Echo/log和小型冻结参数；raw MAT、VS/VSB留本地，仅提供SHA索引。未取得原始数据时只能审核摘要，不能声称独立复算了全部时序。
- docs/v0.5含完整统计窗口、悬架表域、小表与关键图。图为显示取点，验收仍使用0.0005 s原生记录。
- 本次发布不新增车辆仿真，只核对复制字节、原机五配置初始化和模型编译。原始研究及V0.4内容保持冻结。
