"""绘制新增场景的测试路线总览；不是机器人已行走轨迹或已观测地图。"""
import argparse
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Rectangle
from follow_demo.scenarios import SCENARIOS
from follow_demo.scenario_route import RouteWalker


def main():
    """导出三种回环的米制布局，标出方向、箱体、人与狗的初始位置。"""
    parser=argparse.ArgumentParser()
    parser.add_argument('--directory',type=Path,default=Path.home()/'go2_sim/artifacts')
    args=parser.parse_args()
    font=Path('/mnt/c/Windows/Fonts/msyh.ttc')
    if font.exists():
        font_manager.fontManager.addfont(str(font))
    plt.rcParams['font.sans-serif']=['Microsoft YaHei','Noto Sans CJK SC','DejaVu Sans']
    plt.rcParams['axes.unicode_minus']=False
    fig,axes=plt.subplots(1,3,figsize=(16,6),constrained_layout=True)
    for ax,name in zip(axes,('square_loop','slalom_loop','wall_loop')):
        scene=SCENARIOS[name]
        walker=RouteWalker(scene)
        for x,y,sx,sy,_ in scene['boxes']:
            ax.add_patch(Rectangle((x-sx,y-sy),2*sx,2*sy,color='#7e8e9c'))
        points=walker.points
        ax.plot([p[0] for p in points],[p[1] for p in points],'--',color='#d28b27',lw=2,label='目标预置循环路线')
        for a,b in zip(points,points[1:]):
            center=[(a[i]+b[i])/2 for i in (0,1)]
            delta=[(b[i]-a[i])*.12 for i in (0,1)]
            ax.annotate('',xy=[center[i]+delta[i] for i in (0,1)],xytext=center,
                        arrowprops=dict(arrowstyle='->',color='#d28b27',lw=1.7))
        ax.plot(*points[0],'o',color='#d28b27',label='目标初始位置')
        ax.add_patch(Rectangle((-.35,-.16),.7,.32,color='#168d83',label='机器狗初始位置'))
        ax.set_title(f"{scene['name']}\n每圈 {walker.length:.1f} m / 0.5 m/s 时约 {walker.length/.5:.1f} s",fontsize=12)
        ax.set(xlabel='x / m',ylabel='y / m',aspect='equal')
        # 范围包含完整墙体与路线，不用局部视图隐去后半段返程。
        xs=[p[0] for p in points]+[x+sign*sx for x,_,sx,_,_ in scene['boxes'] for sign in (-1,1)]
        ys=[p[1] for p in points]+[y+sign*sy for _,y,_,sy,_ in scene['boxes'] for sign in (-1,1)]
        ax.set_xlim(min(-1,min(xs))-1,max(xs)+1)
        ax.set_ylim(min(-1,min(ys))-1,max(ys)+1)
        ax.grid(alpha=.2)
    axes[1].legend(loc='upper left',fontsize=9)
    fig.suptitle('循环场景布局：灰色障碍仅用于仿真；虚线仅用于目标运动，不输入导航器',fontsize=13)
    args.directory.mkdir(parents=True,exist_ok=True)
    for extension in ('png','svg'):
        fig.savefig(args.directory/f'scenario-loops-layout-20261005.{extension}',dpi=160)
    plt.close(fig)


if __name__ == '__main__':
    main()
