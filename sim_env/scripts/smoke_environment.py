"""验证安装、模型、推理和跨进程 ROS 通信；不代表跟随算法验收。"""
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import traceback

ROOT = Path.home() / 'go2_sim'
REPORT = {'checks': {}, 'timestamp': time.strftime('%Y-%m-%dT%H:%M:%S%z')}


def check(name, operation):
    """独立记录每项检查，单项失败后仍收集其他诊断结果。"""
    try:
        REPORT['checks'][name] = {'ok': True, 'details': operation()}
        print('PASS', name, flush=True)
    except Exception:
        REPORT['checks'][name] = {'ok': False, 'error': traceback.format_exc()}
        print('FAIL', name, flush=True)


def check_ros_packages():
    """确认后续开发需要的 ROS 包和自定义消息均可加载。"""
    from ament_index_python.packages import get_package_prefix
    from unitree_go.msg import LowState
    from unitree_api.msg import Request
    packages = ['rviz2', 'nav2_mppi_controller', 'nav2_planner', 'stereo_image_proc',
                'fusion_estimator', 'go2_description', 'go2_driver', 'go2_twist_bridge']
    assert LowState().imu_state is not None and Request() is not None
    return {package: get_package_prefix(package) for package in packages}


def check_image_bridge():
    """检验 Python NumPy、OpenCV 与 ROS cv_bridge 的实际 ABI 兼容性。"""
    import cv2
    import numpy as np
    from cv_bridge import CvBridge
    frame = np.zeros((24, 32, 3), dtype=np.uint8)
    bridge = CvBridge()
    recovered = bridge.imgmsg_to_cv2(bridge.cv2_to_imgmsg(frame, encoding='bgr8'))
    assert np.array_equal(frame, recovered)
    return {'opencv': cv2.__version__, 'numpy': np.__version__}


def check_policy():
    """对锁定的 ONNX 模型执行一次推理，确认依赖和张量接口可用。"""
    import numpy as np
    import onnxruntime as ort
    model = ROOT / 'third_party/capo-sim-assets/sim_patch/models/go2_policy.onnx'
    options = ort.SessionOptions()
    options.intra_op_num_threads = 1
    session = ort.InferenceSession(str(model), options, providers=['CPUExecutionProvider'])
    inputs = {item.name: np.zeros(item.shape, dtype=np.float32) for item in session.get_inputs()}
    outputs = session.run(None, inputs)
    assert all(np.isfinite(output).all() for output in outputs)
    return {'onnxruntime': ort.__version__, 'inputs': {key: list(value.shape) for key, value in inputs.items()},
            'outputs': [list(output.shape) for output in outputs]}


def check_physics_and_render():
    """加载 Go2 场景、执行物理步并离屏渲染，保存可检查的模型截图。"""
    import mujoco
    import numpy as np
    from PIL import Image
    model = mujoco.MjModel.from_xml_path(str(ROOT / 'third_party/unitree_mujoco/unitree_robots/go2/scene.xml'))
    data = mujoco.MjData(model)
    if model.nkey:
        mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    model.vis.global_.offwidth = 960
    model.vis.global_.offheight = 720
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [0, 0, 0.25]
    camera.distance, camera.azimuth, camera.elevation = 1.8, 135, -20
    with mujoco.Renderer(model, height=720, width=960) as renderer:
        renderer.update_scene(data, camera=camera)
        frame = renderer.render()
        assert frame.std() > 5, '渲染图像几乎全空白'
        Image.fromarray(frame).save(ROOT / 'artifacts/go2_model.png')
    # 此处只检验求解器正常推进，不运行步态策略，也不评估行走稳定性。
    for _ in range(100):
        mujoco.mj_step(model, data)
    assert np.isfinite(data.qpos).all()
    return {'mujoco': mujoco.__version__, 'nq': model.nq, 'nu': model.nu,
            'physics_time': data.time, 'renderer': os.environ.get('MUJOCO_GL'),
            'image': str(ROOT / 'artifacts/go2_model.png')}


def check_ros_roundtrip():
    """启动独立发布进程，确认 localhost DDS 发现和消息传输可用。"""
    import rclpy
    from std_msgs.msg import String
    rclpy.init()
    node = rclpy.create_node('go2_environment_check')
    received = []
    subscription = node.create_subscription(String, '/chatter', lambda msg: received.append(msg.data), 10)
    log = (ROOT / 'logs/ros_talker.log').open('w')
    process = subprocess.Popen(['ros2', 'run', 'demo_nodes_cpp', 'talker'], stdout=log,
                               stderr=subprocess.STDOUT, start_new_session=True)
    try:
        deadline = time.monotonic() + 18
        while time.monotonic() < deadline and len(received) < 3:
            rclpy.spin_once(node, timeout_sec=0.2)
        assert len(received) >= 3, '18 秒内未收到独立进程发布的 ROS 消息'
        return {'received': len(received), 'domain': os.environ.get('ROS_DOMAIN_ID'),
                'rmw': os.environ.get('RMW_IMPLEMENTATION')}
    finally:
        # 只停止本次创建的进程组，不使用 pkill，以免干扰用户其他 ROS 任务。
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
        log.close()
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


def main():
    """执行环境冒烟检查，写入机器可读报告并通过退出码表达结果。"""
    ROOT.joinpath('artifacts').mkdir(exist_ok=True)
    for name, operation in [('ros_packages', check_ros_packages), ('image_bridge', check_image_bridge),
                            ('onnx_policy', check_policy), ('mujoco_render', check_physics_and_render),
                            ('ros_roundtrip', check_ros_roundtrip)]:
        check(name, operation)
    REPORT['passed'] = all(result['ok'] for result in REPORT['checks'].values())
    path = ROOT / 'artifacts/environment-check.json'
    path.write_text(json.dumps(REPORT, indent=2, ensure_ascii=False))
    print(path)
    raise SystemExit(0 if REPORT['passed'] else 1)


if __name__ == '__main__':
    main()
