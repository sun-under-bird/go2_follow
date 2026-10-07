"""用真实渲染深度核对像素中心约定；场景真值仅用于检验，绝不进入导航地图。"""
import fcntl
import json
import math
from pathlib import Path

import mujoco
import numpy as np
from follow_demo.simulation import Simulation, ROOT
from follow_demo.camera_profile import FOVY
from follow_demo.local_map import RollingMap


def measure(offsamples):
    """站稳后冻结一帧，比较两种主点约定的地面深度误差和实测自由通道。"""
    sim = Simulation(render=False)
    renderer = None
    try:
        for _ in range(round(4. / sim.model.opt.timestep)):
            sim.policy.update(sim.data, np.zeros(3))
            sim.policy.torque(sim.data)
            mujoco.mj_step(sim.model, sim.data)
        mujoco.mj_forward(sim.model, sim.data)
        sim.model.vis.quality.offsamples = offsamples
        renderer = mujoco.Renderer(sim.model, height=120, width=212)
        renderer.enable_depth_rendering()
        option = mujoco.MjvOption()
        option.geomgroup[4] = 0
        renderer.update_scene(sim.data, camera='front_left', scene_option=option)
        depth = renderer.render().copy()
        camera = sim.model.camera('front_left').id
        position = sim.data.cam_xpos[camera].copy()
        # MuJoCo相机轴是右、上、后；ROS optical为右、下、前。
        rotation = sim.data.cam_xmat[camera].reshape(3, 3) @ np.diag([1., -1., -1.])
        focal = 120 / (2 * math.tan(math.radians(FOVY / 2)))
        rows, cols = np.indices(depth.shape)
        result = {}
        for convention, cx, cy in [('half_size', 106., 60.), ('pixel_center', 105.5, 59.5)]:
            rays = np.stack(((cols-cx)/focal, (rows-cy)/focal, np.ones(depth.shape)), axis=-1)
            world_ray = rays @ rotation.T
            expected = np.divide(-position[2], world_ray[..., 2], out=np.full(depth.shape, np.inf),
                                 where=world_ray[..., 2] < -1e-6)
            selected = (expected > 1.) & (expected < 4.5) & (depth < 5.)
            error = abs(depth[selected]-expected[selected])
            grid = RollingMap(static_history=True)
            grid.confirm_start(0., 0., 4.)
            grid.integrate(depth, [focal, focal, cx, cy], rotation, position, 4.1)
            free, _, _ = grid.layers(4.1)
            distance = 0.
            for x in np.arange(.1, 5., .1):
                if not grid.route_clear(free, [[0., 0.], [float(x), 0.]], 0.):
                    break
                distance = float(x)
            result[convention] = dict(pixel_count=int(selected.sum()),
                                      error_quantiles_m=np.quantile(error, [.5, .9, .99]).tolist(),
                                      observed_free_route_m=distance,
                                      obstacle_cells=int(grid.occupied.sum()))
        return result
    finally:
        if renderer is not None:
            renderer.close()
        sim.close()


def main():
    """独占物理资源，保存检验结果后退出，不启动ROS或HTTP服务。"""
    with (ROOT / 'follow_demo/runtime.lock').open('a+') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        result = {str(samples): measure(samples) for samples in (4, 0)}
        # 关闭MSAA后，逐像素主点约定必须与真实渲染达到毫米内的一致性。
        result['passed'] = result['0']['pixel_center']['error_quantiles_m'][2] < .001
        path = ROOT / 'artifacts/depth-pixel-center-20261006.json'
        path.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
        print(json.dumps(result, ensure_ascii=False), flush=True)
        if not result['passed']:
            raise SystemExit(1)


if __name__ == '__main__':
    main()
