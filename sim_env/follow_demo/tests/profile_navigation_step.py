"""离线重建报告中的控制状态，定位计算开销；不运行物理或发布底盘命令。"""
import cProfile, json, pstats
from pathlib import Path
import numpy as np
from follow_demo.navigation import NavigationController
from follow_demo.local_planner import Plan
from follow_demo.controller import Pose


def main():
    """报告已保存的地图仅用于重放性能，不作为新仿真场景的自由证据。"""
    root=Path.home()/'go2_sim/artifacts'
    row=json.loads((root/'speed-fix-live-state-20261006.json').read_text(encoding='utf-8-sig'))
    nav=row['control']['navigation']; saved=nav['map']; now=row['control']['control_t']
    core=NavigationController()
    try:
        grid=core.grid; grid.origin=np.asarray(saved['origin']); grid.static_history=True
        codes=np.asarray(list(saved['cells']),dtype=int).reshape(saved['size'],saved['size'])
        grid.seen[codes!=0]=now; grid.occupied[:]=codes==2
        grid.confirmed=True; grid.last_depth=now-.2
        pose=Pose(now,row['x'],row['y'],row['yaw']); core.history.add(pose)
        core.target=np.asarray(row['target']); core.target_velocity=row['control']['estimated_target_velocity']; core.target_stamp=now
        core.plan=Plan('FOLLOWING','PROFILE',nav['path']); core.last_plan=now
        core.motion=[row[k] for k in ('vx','vy','wz')]
        core.external_active=True; core.external_command=nav['raw_command']; core.external_stamp=now
        profile=cProfile.Profile(); profile.enable()
        for _ in range(10): core.step(now)
        profile.disable()
        pstats.Stats(profile).sort_stats('cumtime').print_stats(25)
    finally: core.close()


if __name__=='__main__': main()
