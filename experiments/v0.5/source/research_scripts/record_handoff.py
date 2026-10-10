"""完成核验后向工作区AGENTS追加事实入口；不覆盖历史内容。"""
from prepare_research import *
def main():
 a=read(W/'audit/final_integrity.json');v=read(W/'audit/final_road_validation.json');rows=read(W/'results/analysis/summary.json')
 assert a['passed'] and v['passed'] and len(rows)==5 and all(r['actual_end_s']==60 for r in rows)
 target=Path('E:/00硕士毕业论文/AGENTS.md');title='## 最新完成任务（2026-10-10，Step37c 道路坐标修复与五刚度补齐）'
 assert title not in target.read_text(encoding='utf-8-sig'),'拒绝重复追加'
 recover='/'.join(f"{r['recovery_s']:.4f}" for r in rows)
 txt=f'''

{title}

用户授权优先核查道路坐标并补齐比较条件。最新入口：`{W.as_posix()}/先看这里.md`，唯一主报告`results/analysis/report.md`，总表`summary.csv`。本批新增五次60s全部PASS并停止；不自动重跑、调控制器或发布Git。

- 原道路实际为起点(0,-16)、半径15m闭合圆，圆心(0,-1)。192/384和384/576旧错误查询点距圆心约.18504/.15136m，圆坐标Jacobian=1-L/15接近零。支持圆心邻域XY→SL迭代条件恶化；没有内部迭代/查询ID证据，不冒称具体代码缺陷、质心或特定车轮错误。
- 独立Step37c研究sim/par仅OPT_PATH_LOOP 1→0、SEGMENT_TYPE 3→0、半径/圆心角替换SEGMENT_LENGTH2000；保留起点和朝向，S=X、L=Y+16。物理Z=0、μ=.85、RR=1、车辆/初态/控制器全部保持，GUI/正式配置未写回。边界参数±2000m保持，坐标变换后的远端全局边界形状不同；本任务未接近边界。实际四轮地面高度/坡度/附着输出审核通过。
- 五组384/384、192/384、576/384、384/192、384/576统一新坐标。旧3组完整60s、新旧全部56数值通道逐位一致；旧2短记录的全部共同前缀也56通道逐位一致，新运行补齐右10s维持与退出。旧Step37b原始FAIL仍保留为道路求解中断，不能解释成车辆失控或该刚度无法右漂。
- 全部左右600点进入、两段固定20000点维持、2000点恢复通过；最终恢复按上述顺序{recover}s。左起漂/换向时序与Step37b相同。不同阶段排序和速度/轮矩/轮荷取舍仍在，不将通过等同整体最优、机械安全或鲁棒性。
- 沿用Step37a LeftRight_R3、3×5、.01/.0005、12/76、Fx/惯量/执行预算；外部ASB四路零、无路径跟踪，后轴正向κ.4上限不恢复。自然左右双环不称指定几何八字跟踪；当前研究为固定内部辅助刚度，非主动ASB控制。
- {len(read(W/'source/runner_freeze.json')['files'])}执行文件与{len(read(W/'source/runner_freeze.json')['external_files'])}历史/正式保护文件哈希保持。每组唯一无损raw与必要CarSim证据；raw合计{a['raw_bytes']}字节，旧raw只引用，派生缓存经重建与指标复算后清理。原悬架/轮胎外推继续保留，修复道路坐标不等于组件实物范围验证。
- 只读复算internal/review_case.py、compare_cases.py；绘图render_comparison、render_road_diagnosis；先重建cache后绘图，不调用已完成run_state_only_case/run_remaining。初始化保留model/a001_step37a_initialize.m原名与原字节。Git main/V0.4仍6de15ed，未发布新版本。
'''
 with target.open('ab') as f:f.write(txt.encode('utf-8'))
 print('HANDOFF_APPENDED')
if __name__=='__main__':main()
