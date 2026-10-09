# V0.4主动起漂补充证据：Step36a

本目录归档已经完成的五组单轴主动侧倾力矩试验及符号核查。内部辅助刚度统一384/384；前/后单轴±250 N·m只在起漂后1.5—2.0 s作用，原车辆反馈全程保留。

- [综合报告第7节](../../../../docs/v0.4/report.md)解释主动力矩与固定刚度的方向关系，避免将N·m与N·m/deg直接对应。
- [专项报告](../../../../docs/v0.4/active_entry/report.md)、五行summary.csv、方向表及关键图均在docs/v0.4/active_entry。
- evidence/raw_data_index.json给出五份57通道无损记录、配置及CarSim必要证据的身份。完整MAT与VS/VSB保留本机，没有上传。
- source为实际脉冲、轮端映射、五状态调度核心、初始化、冻结参数和离线分析的审计快照。实际模型SLX的哈希保留于direction_check.json；没有用这些快照覆盖原V0.4模型。
- legacy_stiffness_mapping.m仅用于追溯M=-ΔKφ的符号，来自历史Step27，不是本批执行控制器。
- 分析代码依赖本机相邻历史读取模块和原始记录；不是仓库内即插即用的独立复现包。本批唯一初始化a001_step36a_initialize.m属于研究副本；原initialize_v04仍运行原V0.4任务。
- 负向组脉冲前±0位差的原始记录及审计保留；实际车辆运行总数5，没有为技术后处理问题追加仿真。

本次Git整理新增车辆运行0次，不调增益、不改预算、不把起漂方向外推到主动退漂或全过程闭环收益。source/publication_sources.json记录逐字节来源映射。旧版b11e6d4和最初63e158a均保留在Git历史。
