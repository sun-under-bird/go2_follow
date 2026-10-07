"""把物理验收样本画成离线轨迹图；场景真值只用于报告，不参与导航。"""
import argparse
import hashlib
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Circle, Rectangle,Polygon
from follow_demo.scenarios import SCENARIOS

# 本机导出图采用中文字体；Linux 复现时允许回退到已安装的中文字体。
# WSL 未装中文字体包时使用同机 Windows 字体，不复制字体文件或改仿真依赖。
windows_font = Path('/mnt/c/Windows/Fonts/msyh.ttc')
if windows_font.is_file():
    font_manager.fontManager.addfont(str(windows_font))
matplotlib.rcParams['font.sans-serif'] = ['Microsoft YaHei','Noto Sans CJK SC','SimHei','DejaVu Sans']
matplotlib.rcParams['axes.unicode_minus'] = False

def main():
    """读取已有证据，不启动仿真；展示真实行走路线与最后停车位置。"""
    parser = argparse.ArgumentParser()
    parser.add_argument('--directory', type=Path, default=Path.home() / 'go2_sim/artifacts')
    parser.add_argument('--prefix', default='navigation-v2')
    parser.add_argument('--scenarios', nargs='+', choices=list(SCENARIOS), default=list(SCENARIOS),
                        help='本图包含的场景；完整未运行状态另见离线回放')
    args = parser.parse_args()
    count = len(args.scenarios)
    fig, axes = plt.subplots(math.ceil((count + 1) / 3), 3, figsize=(13, 4 * math.ceil((count + 1) / 3)), constrained_layout=True)
    titles = {name: scene['name'] for name, scene in SCENARIOS.items()}
    for ax, (name, scene) in zip(axes.flat, ((name,SCENARIOS[name]) for name in args.scenarios)):
        report_path = args.directory / f'{args.prefix}-{name}.json'
        samples_path = args.directory / f'{args.prefix}-{name}-samples.json'
        if not report_path.exists() or not samples_path.exists():
            ax.set_title(titles[name] + ' / 无记录')
            continue
        report, samples = json.loads(report_path.read_text(encoding='utf-8-sig')), json.loads(samples_path.read_text(encoding='utf-8-sig'))
        # 灰色箱体是验收时已知的仿真真值，仅解释走到了哪里，不代表控制器曾看见整张图。
        geometry_matches = report.get('source_sha256',{}).get('scenarios.py') == hashlib.sha256(
            (Path(__file__).resolve().parents[1]/'scenarios.py').read_bytes()).hexdigest()
        embedded_scene=report.get('scenario_specification')
        embedded_matches=(isinstance(embedded_scene,dict) and hashlib.sha256(
            json.dumps(embedded_scene,ensure_ascii=False,sort_keys=True).encode()).hexdigest()==report.get('scenario_spec_sha256'))
        boxes=embedded_scene['boxes'] if embedded_matches else scene['boxes'] if geometry_matches else []
        for x, y, sx, sy, _ in boxes:
            ax.add_patch(Rectangle((x - sx, y - sy), 2 * sx, 2 * sy, color='#687783', alpha=.7))
        if samples:
            ax.plot([r['target'][0] for r in samples], [r['target'][1] for r in samples],
                    '--', color='#d1873e', label='目标实际路线')
            ax.plot([r['x'] for r in samples], [r['y'] for r in samples], color='#168b8b', lw=2, label='机器人实际路线')
            final = samples[-1]
            ax.plot(final['x'], final['y'], 'x', color='#b74444', ms=9, label='最终位置')
            if report.get('acceptance_schema_version',0)>=6:
                # 包络取本场报告的尺寸，按最后实测朝向旋转，不能把新版矩形画成旧圆形。
                shape = report['final_navigation']['navigation']['map']['footprint']
                length,width = shape['length'],shape['width']
                c,s = math.cos(final['yaw']),math.sin(final['yaw'])
                corners = [(final['x']+sx*length/2*c-sy*width/2*s,
                            final['y']+sx*length/2*s+sy*width/2*c) for sx,sy in ((1,1),(-1,1),(-1,-1),(1,-1))]
                ax.add_patch(Polygon(corners,fill=False,color='#b74444',alpha=.6))
            else:
                ax.add_patch(Circle((final['x'], final['y']), .48, fill=False, color='#b74444', alpha=.6))
        passed = report.get('passed_navigation', False)
        ax.set_title(titles[name] + (' / 通过' if passed else ' / 未通过'), fontsize=11)
        ax.text(.02, .02, report.get('final_code', report.get('error', '无记录')), transform=ax.transAxes, fontsize=8)
        # 所有场景都包含完整实际轨迹和目标；较大绕行不能因旧固定坐标范围被截掉。
        far_x = max((max(r['x'],r['target'][0]) for r in samples),default=8)+1
        near_x = min((min(r['x'],r['target'][0]) for r in samples),default=0)-1
        far_y = max((max(r['y'],r['target'][1]) for r in samples),default=2.5)+1
        near_y = min((min(r['y'],r['target'][1]) for r in samples),default=-1.5)-1
        if boxes:
            # 方形外墙可能位于路线之外，图表范围也必须包含完整已核对的场景真值。
            far_x=max(far_x,max(x+sx for x,_,sx,_,_ in boxes)+.5)
            near_x=min(near_x,min(x-sx for x,_,sx,_,_ in boxes)-.5)
            far_y=max(far_y,max(y+sy for _,y,_,sy,_ in boxes)+.5)
            near_y=min(near_y,min(y-sy for _,y,_,sy,_ in boxes)-.5)
        ax.set(xlim=(min(-1,near_x),max(9,far_x)), ylim=(min(-2.5,near_y),max(3.5,far_y)),
               xlabel='x (m)', ylabel='y (m)')
        ax.set_aspect('equal')
        ax.grid(alpha=.2)
    # 动态分配布局，新增场景不能被固定的五场图表静默遗漏。
    handles,labels=[],[]
    for ax in list(axes.flat)[:count]:
        handles,labels=ax.get_legend_handles_labels()
        if handles:
            break
    for ax in list(axes.flat)[count:]:
        ax.axis('off')
    legend=axes.flat[count]
    legend.legend(handles, labels, loc='upper left')
    legend.text(0, .45, '灰色障碍真值仅用于解释报告\n没有碰撞不等于完成绕障\n各场单次运行，不代表成功率\n未运行场景明确显示无记录\n控制器：Nav2 MPPI + 统一执行保护',
                      transform=legend.transAxes, fontsize=10, linespacing=1.8)
    fig.suptitle('前视相机局部导航：真实物理轨迹验收', fontsize=15)
    fig.savefig(args.directory / f'{args.prefix}-trajectories.png', dpi=160)
    fig.savefig(args.directory / f'{args.prefix}-trajectories.svg')
    plt.close(fig)
    print(args.directory / f'{args.prefix}-trajectories.png')

    # 同时展示整段速度，不截去转弯停车或人停止之后的保持时段。
    fig,axes = plt.subplots(count,1,figsize=(12,2.4*count),constrained_layout=True,squeeze=False)
    axes=axes.ravel()
    for ax,name in zip(axes,args.scenarios):
        path = args.directory/f'{args.prefix}-{name}-samples.json'
        if not path.exists():
            ax.set_title(titles[name]+' / 无记录')
            continue
        rows = json.loads(path.read_text(encoding='utf-8-sig'))
        if not rows:
            continue
        times = [r['t']-rows[0]['t'] for r in rows]
        for label,key,color in [('实际机身前进速度','vx','#168b8b'),('目标速度','target_speed','#d1873e')]:
            ax.plot(times,[r.get(key,0) for r in rows],label=label,color=color,lw=1.5)
        ax.plot(times,[r['command'][0] for r in rows],label='最终前进指令',color='#657fa1',alpha=.65)
        # 实测步态存在瞬时波动，不能用固定0.85上界裁掉真实超调或反向残余。
        values=[float(r.get(key,0.)) for r in rows for key in ('vx','target_speed')]
        ax.set(title=titles[name],ylabel='m/s',xlabel='开始跟随后仿真时间（s）',
               ylim=(min(-.05,min(values)-.03),max(.85,max(values)+.03)))
        ax.grid(alpha=.2)
    for ax in axes:
        if ax.get_legend_handles_labels()[0]:
            ax.legend(loc='upper right',ncol=3)
            break
    fig.savefig(args.directory/f'{args.prefix}-speeds.png',dpi=150)
    plt.close(fig)


if __name__ == '__main__':
    main()
