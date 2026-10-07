"""离线绘制 API 快照中的实际观测地图，不向控制器提供场景真值。"""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.patches import Circle


def main():
    """查看卡住时的未知边界、机身包络、路线与实际朝向，输出可保存的 PNG。"""
    parser = argparse.ArgumentParser()
    parser.add_argument('snapshot',type=Path)
    args = parser.parse_args()
    state = json.loads(args.snapshot.read_text(encoding='utf-8-sig'))
    nav = state['control']['navigation']
    grid = nav['map']
    cells = np.fromiter(map(int,grid['cells']),dtype=np.uint8).reshape(grid['size'],grid['size'])
    x,y = grid['origin']
    extent = [x,x+grid['size']*grid['resolution'],y,y+grid['size']*grid['resolution']]
    fig,ax = plt.subplots(figsize=(9,8),constrained_layout=True)
    ax.imshow(cells,origin='lower',extent=extent,vmin=0,vmax=4,
              cmap=ListedColormap(['#152031','#637688','#bd6860','#328f8a','#a08c61']))
    ax.add_patch(Circle((state['x'],state['y']),grid['radius'],fill=False,color='white'))
    ax.arrow(state['x'],state['y'],.7*np.cos(state['yaw']),.7*np.sin(state['yaw']),color='white',width=.015)
    route = nav['path']
    if route:
        ax.plot(*np.asarray(route).T,'o-',color='#ffe78a',label='Search path')
    ax.plot(*state['target'],'x',color='#ffa351',label='Target (diagnostic only)')
    ax.set(xlim=(state['x']-2.5,state['x']+3.),ylim=(state['y']-3.,state['y']+2.),aspect='equal',
           title=f"Observed map / {nav['code']} / yaw={state['yaw']:.2f}, look={nav['look_yaw']:.2f}",
           xlabel='odom x (m)',ylabel='odom y (m)')
    ax.grid(alpha=.2)
    ax.legend()
    output = args.snapshot.with_suffix('.png')
    fig.savefig(output,dpi=140)
    plt.close(fig)
    print(output)


if __name__ == '__main__':
    main()
