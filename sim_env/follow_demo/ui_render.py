"""把软件OpenGL界面渲染放入独立进程，避免占住导航所在Python进程的GIL。"""
import os
import time
import traceback
import cv2
import mujoco
from .camera_profile import WIDTH, HEIGHT


def request_state(connection, quit_event):
    """只申请最新冻结快照；单次请求单次回复，不积压物理或图像任务。"""
    connection.send(('state',))
    while not quit_event.is_set():
        if connection.poll(.1):
            return connection.recv()
    return None


def encode(frame):
    """在渲染子进程编码JPEG，主进程仅缓存已编码结果。"""
    pixels = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR) if frame.ndim == 3 else frame
    ok, encoded = cv2.imencode('.jpg', pixels, [cv2.IMWRITE_JPEG_QUALITY, 85])
    return encoded.tobytes() if ok else None


def render_worker(scene_path, connection, quit_event, owner_pid):
    """加载只读物理模型和私有mjData；父进程退出或通信断开后自行释放OpenGL。"""
    try:
        cv2.setNumThreads(1)
        model = mujoco.MjModel.from_xml_path(scene_path)
        drawing = mujoco.MjData(model)
        periods = dict(stereo=.5, overview=1.)
        deadlines = dict.fromkeys(periods, time.monotonic())
        with mujoco.Renderer(model, height=HEIGHT, width=WIDTH) as camera, \
                mujoco.Renderer(model, height=270, width=480) as overview:
            while not quit_event.is_set() and os.getppid() == owner_pid:
                for name in ('stereo', 'overview'):
                    now = time.monotonic()
                    if quit_event.is_set() or now < deadlines[name]:
                        continue
                    missed = max(0, int((now-deadlines[name])/periods[name]))
                    deadlines[name] = now+periods[name]
                    state = request_state(connection, quit_event)
                    if state is None:
                        break
                    if not state:
                        continue
                    started = time.monotonic()
                    for key in ('qpos', 'qvel', 'mocap_pos', 'mocap_quat'):
                        getattr(drawing, key)[:] = state[key]
                    mujoco.mj_forward(model, drawing)
                    frames = None
                    if name == 'stereo':
                        # 左右图共用同一冻结数据，不混入第二个物理采集时刻。
                        camera.update_scene(drawing, camera='front_left')
                        left = cv2.cvtColor(camera.render(), cv2.COLOR_RGB2GRAY)
                        camera.update_scene(drawing, camera='front_right')
                        right = cv2.cvtColor(camera.render(), cv2.COLOR_RGB2GRAY)
                        frames = (state['t'], left, right)
                        images = dict(left=encode(left), right=encode(right))
                    else:
                        view = mujoco.MjvCamera()
                        view.lookat[:] = [state['x']+.5, state['y'], .4]
                        view.distance, view.azimuth, view.elevation = 5., 135, -25
                        overview.update_scene(drawing, camera=view)
                        images = dict(scene=encode(overview.render().copy()))
                    connection.send(('frame', name, state, started, frames, images, missed))
                quit_event.wait(max(.001, min(deadlines.values())-time.monotonic()))
    except (EOFError, BrokenPipeError):
        # 父进程已关闭，既不重连，也不继续无主渲染。
        pass
    except Exception:
        try:
            connection.send(('error', traceback.format_exc()))
        except (EOFError, BrokenPipeError):
            pass
    finally:
        connection.close()
