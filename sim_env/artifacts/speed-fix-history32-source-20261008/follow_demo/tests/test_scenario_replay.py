"""检查回放的场景快照来源，避免新场景覆盖旧证据的几何解释。"""
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from follow_demo.tests.render_follow_replay import load_scene


class ScenarioReplayTests(unittest.TestCase):
    """只检查证据文件读取，不启动浏览器和服务。"""

    def scene(self, changed=False):
        """写入最小实际采样和快照，模拟全局场景文件已经变化的情况。"""
        snapshot=dict(name='旧场景',boxes=[[3.,0.,.2,1.,1.]],route=[[7.,0.]])
        digest=hashlib.sha256(json.dumps(snapshot,ensure_ascii=False,sort_keys=True).encode()).hexdigest()
        if changed:
            snapshot['boxes'][0][0]=9.
        report=dict(scenario_specification=snapshot,scenario_spec_sha256=digest,
                    source_sha256={'scenarios.py':'old'},passed_navigation=False)
        with tempfile.TemporaryDirectory() as temporary:
            directory=Path(temporary)
            (directory/'test-open.json').write_text(json.dumps(report),encoding='utf-8')
            (directory/'test-open-samples.json').write_text(json.dumps([dict(t=0.,x=0.,y=0.)]),encoding='utf-8')
            return load_scene(directory,'test','open',dict(boxes=[[100.,0.,1.,1.,1.]]),'new')

    def test_verified_embedded_geometry_survives_later_scenario_changes(self):
        """回放使用本场快照，而不是当前文件中另一套障碍。"""
        scene=self.scene()
        self.assertTrue(scene['geometry_verified'])
        self.assertEqual(scene['boxes'][0][0],3.)
        self.assertEqual(scene['status'],'未通过')

    def test_modified_embedded_geometry_is_hidden(self):
        """摘要不匹配时隐藏几何，不凭存在快照字段就当作已核对。"""
        scene=self.scene(changed=True)
        self.assertFalse(scene['geometry_verified'])
        self.assertEqual(scene['boxes'],[])


if __name__ == '__main__':
    unittest.main()
