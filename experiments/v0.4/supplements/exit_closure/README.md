# V0.4退漂补充证据：Step35m/n

本目录补充当前同步连续节点退漂的文档与审计，不替换原V0.4模型或运行入口。

- [综合报告](../../../../docs/v0.4/report.md)区分原Step35f/g起漂与本批Step35m/n退出。
- [退漂报告](../../../../docs/v0.4/exit_closure/report.md)及同目录总表/图给出完整结论。
- source/state_cycle_core.m与profile.json保留Step35m实际调度核心和冻结参数；K为3×5、所有调度量同一进度，不采用C独立K时钟。
- source/analysis中的代码是离线复算来源快照，原入口位于本机Step35n/internal。依赖相邻Step35m及此前历史读取模块，不是仓库内即插即用程序；本次没有宣称独立可运行包。
- evidence/raw_data_index.json提供五份原始记录及物理配置身份，raw与大体积VS/VSB未上传。完整复算需要这些原始证据与原目录依赖。
- evidence/recorded_schedule_protocol.json核对实际K/偏置并复算冻结验收，validation.json核对原生事件、执行公式与来源；source/publication_sources.json列出逐字节发布映射。

本次新增车辆运行0次，未改模型、参数或阈值。原V0.4的initialize_v04仍运行原退出方案，不能用它冒充本批Step35m退出。原报告与原模型状态可从提交63e158a64d033251da65bbe30c44cefd9e1fb190追溯。
