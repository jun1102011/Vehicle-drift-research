# V0.4：从这里开始

本版为Step35f/35g的发布副本：2.5 s连续β调度，V12、β−30°、车体AVz=0.5989797412715093，普通直行→起漂→固定10 s维持→退漂→自然方向直行。五种内部辅助刚度均PASS。**无路径反馈，外部ASB四路恒零。**

|要做什么|入口|是否启动车辆|
|---|---|---|
|阅读研究判断与结果|[主报告](../../docs/v0.4/report.md)、[五行总表](../../docs/v0.4/summary.csv)|否|
|检查发布文件是否完整|根目录MANIFEST.sha256及[来源映射](source/publication_sources.json)|否|
|在原机准备模型/冻结参数|本目录`initialize_v04.m`，默认384/384|否|
|启动一次新运行|显式调用`sim`，按下方说明|是；会写CarSim输出|
|完整重算历史时序|依据[evidence/raw_data_index.json](evidence/raw_data_index.json)找到本地raw，再用原研究分析入口|否；Git不含raw|

## 原机初始化与运行

MATLAB切换到本目录后：

```matlab
v04_case = 'kf384_kr384'; % 还可选kf192_kr384/kf576_kr384/kf384_kr192/kf384_kr576
initialize_v04;          % 检查文件哈希，加载冻结参数；不启动仿真
open_system('stabilizer_bar_step35g');
% 只有准备好新的独立结果目录/输出路径后才显式运行：
% out = sim('stabilizer_bar_step35g','StopTime','60');
```

发布初始化只读原机已核对的研究sim/par，不自动生成/修改输入文件。示例中sim已注释，因为沿用研究sim会再次写其LastRun输出；正式复现实验应先派生独立输出路径并核对非目标配置，保存新身份/哈希，不能覆盖验收记录。本次发布仅做初始化和编译核对，不执行新的整车仿真。

## 文件职责

- `model/`：逐字节保留Step35g模型及唯一核心初始化、兼容support。可见旧路径诊断接口仍在模型，但已旁路，不能当执行路径反馈。
- `source/profiles/`：R3工作点、全部节点及状态任务协议；K严格3×5，初始化不求配平、不调增益。
- `config/`：五组研究par/sim原样快照；路径用于来源追溯，不自动安装为GUI数据集。
- `internal/state_cycle_core.m`、`native_state_capture.m`：对应模型内的阶段/执行与原生事件算法；核心在SLX内实际运行。
- `internal/`其他原样脚本：研究运行/离线分析的审计来源，包含跨Step历史门槛与路径依赖；不是下载后可任意直接运行的批处理入口。尤其run_four_cases拒绝覆盖历史记录，不要为重跑删除其门槛。
- `evidence/`：每组小型配置/运行/独立验收/echo/log，唯一raw只记录位置与SHA；VS/VSB保留本地。`controller_numeric.mat`仅约5KB冻结参数，不是重复时序。
- `docs/v0.4/figures/`：β、状态、输入/约束、轮胎轮荷、轨迹/维持/退出和实际结构。plot_structure.m保留可编辑结构来源。

图为原始记录取点显示，没有滤波；原生验收不使用绘图抽样。报告说明采样边界、悬架表外推和性能取舍。新机器必须重设合法安装/数据库路径并重新做配置核验，未验证跨机器即插即用。
