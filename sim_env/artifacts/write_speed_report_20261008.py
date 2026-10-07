"""依据完整原始报告生成中文验收记录，保留失败，不改通过门槛。"""
import argparse
import json
from pathlib import Path


SCENARIOS=('open','long_wall','consecutive','corner','blocked','square_loop','slalom_loop','wall_loop')


def read(path):
    """兼容Windows与WSL写出的UTF-8 JSON。"""
    return json.loads(path.read_text(encoding='utf-8-sig'))


def number(value, digits=2):
    """未知指标显示缺失，不能用零值掩盖未测数据。"""
    return '—' if value is None else f'{value:.{digits}f}'


def main():
    """以本次八场结果与独立执行基准写文档，明确仿真参数和未完成部分。"""
    parser=argparse.ArgumentParser()
    parser.add_argument('--prefix',required=True)
    args=parser.parse_args()
    root=Path(__file__).resolve().parent
    reports={name:read(root/f'{args.prefix}-{name}.json') for name in SCENARIOS}
    passed=sum(report['passed_navigation'] for report in reports.values())
    lines=[
        '# Go2 速度跟随修复与验收 · 2026-10-08',
        '',
        f'本次完整复测 **{passed}/8场通过**。人的速度和机器狗前进请求上限均为0.8 m/s，角速度请求上限1.0 rad/s，机身矩形0.70×0.32 m。未修改验收门槛，也没有以零碰撞替代任务或流畅性通过。',
        '',
        f'最终证据前缀：`{args.prefix}`。八场使用相同源码与原生插件，身份为`rate / trail / camera / steady / process`，相机显式向下25°。**25°是仿真安装试验，不是实机标定；默认启动仍为水平0°。**',
        '',
        '## 1. 实际结果',
        '',
        '| 场景 | 完整验收 | 任务 | 流畅性 | 人移动期间平均vx m/s | 低速占比 | 最长低速 s | 移动时最大间距 m | 狗圈数 |',
        '|---|---|---|---|---:|---:|---:|---:|---:|'
    ]
    for name,report in reports.items():
        verdict=lambda key:'通过' if report[key] else '未通过'
        title=report['scenario_specification']['name']
        laps=report.get('task_completion',{}).get('robot',{}).get('completed_laps')
        # 封闭通道的目标未行走，移动指标不适用；原报告的占位零不能解释为零间距。
        applicable=name!='blocked'
        mean=number(report.get('target_moving_mean_actual_vx'),3) if applicable else '—'
        slow=number(100*report['target_moving_robot_slow_ratio'],1)+'%' if applicable else '—'
        longest=number(report.get('target_moving_longest_slow_seconds')) if applicable else '—'
        gap=number(report.get('target_moving_max_distance')) if applicable else '—'
        lines.append(f"| {title} | {verdict('passed_navigation')} | {verdict('scenario_objective_met')} | {verdict('passed_fluency')} | {mean} | {slow} | {longest} | {gap} | {number(laps,0)} |")
    failed_names=[name for name,report in reports.items() if not report['passed_navigation']]
    lines += ['',
        f"开阔稳态实测vx为 **{number(reports['open'].get('steady_actual_vx'),3)} m/s**。封闭通道的目标是稳定停车，其零速度不能用于评价行走跟随速度。",
        '',
        '失败原因直接取原报告：' if failed_names else '本次八场全部达到现有门槛；单次运行不能证明重复成功率。', '']
    for name,report in reports.items():
        if not report['passed_navigation']:
            lines.append('- '+report['scenario_specification']['name']+'：'+'；'.join(report.get('debug_reasons',[])))
    lines += ['',
        '原始JSON分别记录安全、刺激身份、任务完成和流畅性。最大间距3.5 m、绕障平均速度≥0.56 m/s、低于0.05 m/s的占比≤15%、单次≤2 s及循环两圈均沿用schema 7。某一项未通过时，不能把整场称为成功。',
        '',
        f'[八场真实离线回放]({args.prefix}-验收回放.html) · [轨迹图]({args.prefix}-trajectories.png) · [速度图]({args.prefix}-speeds.png) · [源码/插件/退出审计]({args.prefix}-final-audit.json) · [本轮源码与二进制快照](speed-fix-history32-source-20261008/manifest.json)',
        '',
        '## 2. 根因与实现',
        '',
        '1. **实测速度与模型输入不是同一个量。** 对锁定ONNX步态做0.4～0.8 m/s的仿真执行校准，保留请求上限0.8；模型内部输入及实际速度独立记录。≤0.3 m/s不做死区或起步补偿，0.3～0.4 m/s仅平滑衔接。没有直接写入机器狗位置或速度。',
        '2. **短参考让优化器提前刹车。** 跟随意图覆盖MPPI预测窗口及一次重规划周期；FollowSpeedCritic将人速前馈和间距需求纳入候选评分，近期权重更高，仍允许未来转弯减速。',
        '3. **相机投影存在系统误差。** 主点改为像素中心(width−1)/2、(height−1)/2，关闭深度缓冲MSAA，并让3×3地面邻域每个像素分别比较自己的射线深度。独立物理检查的99%投影误差从旧约0.327 m降到约0.00045 m，已知直行距离由1.7 m到4.1 m；这个检查使用理想渲染，不能代替实机标定。',
        '4. **同一时钟源不意味着ROS消息同时到达。** 长墙旧记录163次ODOM_STALE全部是里程计领先控制时钟约10～50 ms。现在选不晚于控制时刻、且在0.2 s有效期内的真实里程计及该帧速度；不修改时间戳，不放宽真正过期的门槛。',
        '5. **平滑器可能继续向错误方向加速。** 当新目标已下降或反向时先取消旧的反向加速度，再做普通加速度/加加速度限制。该投影不承诺所有切换都满足原加加速度界。',
        '6. **空间间距小，不代表轨迹进度落后少。** 交错回环旧记录中，机器狗尚未通过第四组障碍，12 m人体历史已删掉下方未完成分支，目标从障碍下方跳到返程段。运行历史扩大到有界32 m、512点（人的速度0.8 m/s时约40 s），点数容量覆盖0.08 m采样；地图尺寸与自由证据保持原语义。有效UWB历史形成目标区域，终点仍限制在当前进度前方2.4 m窗口，人体顶点不作为强制停车点。裁剪丢锚后投影到仍真实记录的近段；人体历史没有写入自由地图。',
        '7. **优化与末级停车必须一致。** BrakingCritic与Python复用同一C++矩形几何，检查请求、实测及中间转速组合；末级重新检查经过加权与平滑的输出。静态历史已按采集时刻TF投影，不能再把图像年龄重复计为执行延迟；名义执行保持0.25 s，深度健康门槛仍为0.9 s。',
        '8. **停得下、能接入两格，都不等于能续行。** 失败的相机地图只有2个可达格，短圆弧对照也未打开大通道，因此证据不支持直接改用Hybrid A*。候选评分与实际正向跟随输出都检查名义停车后0.6 m已知直行出口；它是现有直行/转身搜索的充分条件，不增加机身尺寸，也不声明所有曲线均不可能。零命令仍允许实际制动；观察动作沿用原区域、旋转与相机支持校验。前进受限且实测前速低于0.12 m/s时主动请求观察，避免等15秒无进展超时。',
        '9. **执行停车不等于撤销优化。** 输入和跟随路径仍有效的临时制动保留MPPI动作并送零速度参考，底盘依然归零；暂停、输入失效和路径失效照常撤销。深度融合、搜索与界面渲染分开调度，原生矩形校验降低Python回调耗时。',
        '',
        '[时钟到达顺序反例](odometry-clock-order-root-cause-20261008.json) · [圆弧接入对照](curve-connector-evidence-20261008.json) · [286项逻辑检查日志](speed-fix-unit-final-20261008.log)',
        '',
        '## 3. 可靠性与追赶余量',
        '',
        '四组交错回环在旧12 m历史试验中曾完成两圈、平均约0.608 m/s、低速占比约1.4%、最长低速0.91 s，但最大间距8.80 m仍未通过。随后同版八场复测只有7/8通过，交错回环再次遗漏未完成分支；该轮报告保留为`navigation-speed-viability-20261008`。最终32 m历史完整复测见上表，不能选择较好的一次冒充稳定成功率。',
        '',
        ('只有前视相机时，必须保留能够观察下一段的站位和运动空间。本次达到八场门槛后，仍需重复交错回环、长墙回环等关键场景，才能评估稳定性；没有证明任意通道都可连续跟随。' if not failed_names else '只有前视相机时，必须保留能够观察下一段的站位和运动空间。本次未通过的场景已在上表及原报告中列出，观察与绕障衔接、进度与间距仍需分别处理，不能声称已可靠完成连续跟随。'),
        '',
        '另一个独立问题是追赶余量：人持续0.8 m/s、狗上限0.8 m/s时，绕障损失的距离不能靠同速直行补回，只能依赖安全捷径、人的停顿或允许更大间距。短时提高狗的追赶上限属于新的行为选择，必须另做执行与闭环验证；本轮没有擅自修改0.8上限。',
        '',
        '下一步应区分两件事：按当前相机安装参数重复关键场景，评估观察/绕障与进度稳定性；根据用户选择处理追赶速度与允许间距。不能靠降低验收门槛或把未知改成自由空间掩盖它们。',
        '',
        '## 4. 相机与验证边界',
        '',
        'D435i仿真使用约50 mm红外双目基线、58°垂直视场、424×240理想内参，机身安装中点[0.23,0.025,0.135] m仍是占位值。俯角同时作用于MuJoCo相机轴、ROS光学TF及原始IMU安装轴，报告逐样本核对身份。',
        '',
        '仅按机身高度约0.32 m估算，水平相机最近可见地面在机身中心前约1.05 m；向下25°约0.56 m，实际还受姿态及机器人遮挡影响。这解释了近处盲区为什么会影响转弯，但不能把25°视为已经确认的最优实机安装。',
        '',
        '已独立复测0.4/0.6/0.8 m/s直行及0.8 m/s、±0.3 rad/s转弯，姿态、速度误差和停稳检查通过。补充0.4/0.8 m/s、±0.8 rad/s紧弯也通过所记录的速度误差门槛。请求上限与步态瞬时实测速度不同。',
        '',
        '[执行基准](speed-fix-execution-final-20261008.json) · [紧弯能力诊断](speed-fix-turn-capacity-20261008.json) · [深度像素检查](speed-fix-depth-final-20261008.json)',
        '',
        '仿真为平地静态障碍、真值里程计和理想深度；没有完成CAPO/VIO误差接入、真实D435i噪声/曝光/硬同步验证、UWB NLOS/人体遮挡、动态障碍、Go2 Sport执行辨识或RK3588端到端性能验收。0.45 m/s²制动参数和0.25 s保持时间是仿名义参数，并非实机安全认证。',
        '',
        '## 5. 自己启动与验收',
        '',
        'Windows PowerShell，从项目目录运行。首次或更新代码后先安装实验台：',
        '', '```powershell',
        "wsl.exe -d Ubuntu-22.04 -u chy --exec bash '/mnt/c/Users/chy/Documents/ChatGPT/go2_slam 2/sim_env/scripts/install_follow_demo.sh'",
        '# 使用本轮相同的显式仿真安装角度启动网页实验台',
        '.\\sim_env\\Start-FollowDemo.ps1 -CameraPitchDeg 25',
        '```', '',
        '在网页选择场景、载入并重置，等姿态就绪，确认起始周围1.2 m净空，再开始跟随并让目标沿预置路线行走。人的速度保持0.8；看实际速度、间距、停车原因和完整圈数。机器狗仍需要你亲自验收。',
        '', '```powershell',
        '# 仅关网页不会停止后台仿真',
        '.\\sim_env\\Stop-FollowDemo.ps1',
        '# 重跑全部场景；先结束人工实例，报告使用新前缀',
        '.\\sim_env\\Check-NavigationDemo.ps1 -Scenarios open,long_wall,consecutive,corner,blocked,square_loop,slalom_loop,wall_loop -Speed 0.8 -Laps 2 -CameraPitchDeg 25 -ReportPrefix navigation-self-08',
        '```', '',
        '自动检查无论通过或失败都会保存结果并关闭自己启动的实例。失败退出不意味着未关闭；本次实际退出与源码一致性见上方审计JSON。所有清理仅针对本任务启动的进程，不结束其他任务、整台WSL或Docker。',
        ''
    ]
    destination=root/'速度跟随修复与验收_20261008.md'
    destination.write_text('\n'.join(lines),encoding='utf-8')
    print(destination)


if __name__=='__main__':
    main()
