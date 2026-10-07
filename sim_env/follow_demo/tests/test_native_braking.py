"""原生停车优化必须与已有Python矩形保护等价，不能用提速掩盖擦角或缓存漏障。"""
import math
import json
from pathlib import Path
import unittest
from unittest.mock import patch
import numpy as np
from follow_demo.controller import Pose
from follow_demo.navigation import NavigationController
from follow_demo.local_map import RollingMap
from follow_demo.local_planner import LocalPlanner


class NativeBrakingTests(unittest.TestCase):
    """在安装原生运行库的WSL检查实际二进制，其他环境明确跳过。"""
    def setUp(self):
        """每例拥有一个控制器及原生地图，结束后释放两种资源。"""
        self.core = NavigationController()
        self.addCleanup(self.core.close)
        if self.core.native_braking.library is None:
            self.skipTest('未安装本实验台原生库，不能冒充二进制校验通过')

    def compare(self, grid, free, pose, forward, turn, lateral):
        """同一输入分别进入C++共享几何与原Python参考，比较拒收和完整轨迹。"""
        core = self.core
        core.grid = grid
        actual = core.braking_trajectory(pose,forward,turn,lateral,free)
        actual_diagnostic = core.trajectory_diagnostic.copy()
        with patch.object(core.native_braking,'brake',return_value=None):
            expected = core.braking_trajectory(pose,forward,turn,lateral,free)
        self.assertEqual(actual[0],expected[0])
        np.testing.assert_allclose(actual[1],expected[1],atol=1e-12,rtol=1e-12)
        self.assertEqual(actual_diagnostic.get('elapsed_s'),core.trajectory_diagnostic.get('elapsed_s'))

    def test_random_braking_matches_python_including_rotation_and_side_drift(self):
        """不同地图密度、朝向、转弯、侧移和陈旧度的600条停车轨迹必须一致。"""
        grid = RollingMap(size=81,resolution=.05,static_history=True)
        grid.origin[:] = [-2.,-2.]
        rng = np.random.default_rng(20261008)
        for index in range(600):
            free = rng.random((81,81)) > (.002 if index%2 else .02)
            grid.last_depth = 1.-rng.uniform(0.,.5)
            pose = Pose(1.,*rng.uniform(-1.8,1.8,2),rng.uniform(-math.pi,math.pi))
            forward,turn,lateral = rng.uniform(-.2,.8),rng.uniform(-1.,1.),rng.uniform(-.1,.1)
            with self.subTest(index=index):
                self.compare(grid,free,pose,forward,turn,lateral)

    def test_grid_line_contacts_and_zero_turn_match_reference(self):
        """精确接触格线、旋转为零和转速极小值也不能因浮点或凸包处理漏障。"""
        grid = RollingMap(size=81,resolution=.1,static_history=True)
        grid.origin[:] = [-4.,-4.]
        grid.last_depth = 1.
        for forward in (0.,.4,.8):
            for turn in (0.,1e-12,1.,-1.):
                free = np.ones((81,81),dtype=bool)
                free[40,45] = False
                self.compare(grid,free,Pose(1.,.05,.04,0.),forward,turn,0.)

    def test_writable_map_update_invalidates_native_cache(self):
        """同一个外部可写地图新增机身内障碍时，缓存必须马上拒收。"""
        grid = RollingMap(size=81,resolution=.1,static_history=True)
        grid.origin[:] = [-4.,-4.]
        grid.last_depth = 1.
        free = np.ones((81,81),dtype=bool)
        self.core.grid = grid
        pose = Pose(1.,0.,0.,0.)
        self.assertTrue(self.core.braking_trajectory(pose,.1,0.,0.,free)[0])
        free[grid.cell(0.,0.)] = False
        self.assertFalse(self.core.braking_trajectory(pose,.1,0.,0.,free)[0])

    def test_native_rejoin_matches_complete_python_route(self):
        """400条折线、共线点、重复点及终点转向，与原完整路线扫掠逐条一致。"""
        grid = RollingMap(size=81,resolution=.05,static_history=True)
        grid.origin[:] = [-2.,-2.]
        rng = np.random.default_rng(8008)
        for index in range(400):
            free = rng.random((81,81)) > (.002 if index%2 else .01)
            route = [rng.uniform(-1.5,1.5,2).tolist()]
            for _ in range(4):
                route.append((np.asarray(route[-1])+rng.uniform(-.3,.3,2)).tolist())
            if index%3 == 0:
                route.insert(1,route[0])
            if index%5 == 0:
                route.insert(2,((np.asarray(route[1])+route[2])/2).tolist())
            yaw = rng.uniform(-math.pi,math.pi)
            final = None if index%2 else rng.uniform(-math.pi,math.pi)
            self.assertEqual(self.core.native_braking.route(grid,free,route,yaw,final),
                             grid.route_clear(free,route,yaw,final),msg=f'路线反例{index}')

    def test_safe_stand_does_not_imply_restartable_search(self):
        """长墙失败快照中机身安全却无法接入搜索，优化器须区分这两种状态。"""
        fixture=json.loads((Path(__file__).parent/'fixtures/long-wall-connection-20261008.json').read_text())
        grid=RollingMap(size=fixture['size'],resolution=fixture['resolution'],static_history=True)
        grid.origin=np.array(fixture['origin'])
        pose=Pose(*fixture['pose'])
        image=np.fromiter(map(int,fixture['cells']),dtype=np.uint8).reshape(grid.size,grid.size)
        grid.seen[np.isin(image,[1,3,4])]=pose.t
        grid.occupied[:]=image==2
        free,_,_=grid.layers(pose.t)
        self.assertTrue(grid.pose_clear(free,pose.x,pose.y,pose.yaw))
        self.core.native_braking.prepare(grid,free)
        native=self.core.native_braking
        self.assertFalse(native.library.go2_geometry_seedable(native.handle,pose.x,pose.y,pose.yaw))
        reachable,_=LocalPlanner().reachable(grid,free,pose,np.ones_like(grid.seen))
        self.assertFalse(np.isfinite(reachable).any())

    def test_seedable_pocket_has_no_confirmed_continuation(self):
        """物理失败的相机地图证明：接入两格不等于有出口，评分不能将二者混为一谈。"""
        fixture=json.loads((Path(__file__).parent/'fixtures/slalom-pocket-20261008.json').read_text(encoding='utf-8-sig'))
        grid=RollingMap(size=fixture['size'],resolution=fixture['resolution'],static_history=True)
        grid.origin=np.array(fixture['origin'])
        pose=Pose(*fixture['pose'])
        image=np.fromiter(map(int,fixture['cells']),dtype=np.uint8).reshape(grid.size,grid.size)
        grid.seen[np.isin(image,[1,3,4])]=pose.t
        grid.occupied[:]=image==2
        free,_,_=grid.layers(pose.t)
        native=self.core.native_braking
        native.prepare(grid,free)
        self.assertTrue(native.library.go2_geometry_seedable(native.handle,pose.x,pose.y,pose.yaw))
        self.assertFalse(native.library.go2_geometry_continuable(native.handle,pose.x,pose.y,pose.yaw,.6))
        reachable,_=LocalPlanner().reachable(grid,free,pose,np.ones_like(grid.seen))
        self.assertEqual(int(np.isfinite(reachable).sum()),2)
        self.core.grid=grid
        self.core.plan.kind='FOLLOWING'
        self.core.motion=(0.,0.,0.)
        # 极小前移的碰撞停车本身可行；拒绝原因必须来自续行条件，不能冒充碰撞。
        self.assertTrue(self.core.braking_trajectory(pose,.001,0.,0.,free)[0])
        self.assertFalse(self.core.braking_safe(pose,(.001,0.),free)[0])
        self.assertEqual(self.core.trajectory_diagnostic.get('rejection'),'CONTINUATION_UNCONFIRMED')
        self.assertTrue(self.core.braking_safe(pose,(0.,0.),free)[0])

    def test_continuation_does_not_require_extra_body_margin(self):
        """长的已知窄直廊保留原矩形，续行偏好不能变成额外硬膨胀。"""
        grid=RollingMap(size=61,resolution=.1,static_history=True)
        grid.origin[:]=[-3.,-3.]
        _,gy=grid.centers()
        free=abs(gy)<.31
        native=self.core.native_braking
        native.prepare(grid,free)
        self.assertTrue(native.library.go2_geometry_continuable(native.handle,0.,0.,0.,.6))

    def test_restart_domain_matches_existing_continuous_seed_search(self):
        """稀疏障碍、窄通道和窗口边界下，可续行判定须与当前搜索的起点接入一致。"""
        grid=RollingMap(size=61,resolution=.1,static_history=True)
        grid.origin[:]=[-3.,-3.]
        gx,gy=grid.centers()
        rng=np.random.default_rng(8810)
        for index in range(12):
            free=(abs(gy)<.31) if index==0 else rng.random((61,61))>.003
            pose=Pose(1.,*rng.uniform(-2.9,2.9,2),rng.uniform(-math.pi,math.pi))
            if index==0:
                pose=Pose(1.,0.,0.,0.)
            self.core.native_braking.prepare(grid,free)
            native=self.core.native_braking
            actual=bool(native.library.go2_geometry_seedable(native.handle,pose.x,pose.y,pose.yaw))
            reachable,_=LocalPlanner().reachable(grid,free,pose,np.ones_like(grid.seen))
            self.assertEqual(actual,bool(np.isfinite(reachable).any()),msg=f'接入反例{index}')

    def test_continuation_fallback_matches_native_corridors_and_window_edges(self):
        """缺库时仍按同一出口几何拒收；窄廊和地图边缘不能获得默认放行。"""
        grid=RollingMap(size=61,resolution=.1,static_history=True)
        grid.origin[:]=[-3.,-3.]
        _,gy=grid.centers()
        free=abs(gy)<.31
        native=self.core.native_braking
        for x,y,yaw in ((0.,0.,0.),(0.,0.,math.pi/4),(2.7,0.,0.),(-2.7,0.,math.pi)):
            pose=Pose(1.,x,y,yaw)
            expected=native.continuable(grid,free,pose)
            with patch.object(native,'library',None):
                actual=native.continuable(grid,free,pose)
            self.assertEqual(actual,expected)


if __name__ == '__main__':
    unittest.main()
