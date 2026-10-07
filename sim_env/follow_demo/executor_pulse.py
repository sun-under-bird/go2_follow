"""通过独立单调定时唤醒 DDS 等待，不执行回调或伪造传感器更新。"""
import threading
import time


class ExecutorPulse:
    """限制执行器等待空档，避开底层墙钟条件变量受系统校时影响的问题。"""

    def __init__(self, executors, period=.02):
        """保存已有执行器；只有它们自己的线程能够读取消息和执行回调。"""
        if period <= 0:
            raise ValueError('唤醒间隔必须为正')
        self.executors = tuple(executors)
        self.period = period
        self.quit = threading.Event()
        self.state = dict(ticks=0, last_wall=None, maximum_lateness_s=0., error='')
        self.thread = threading.Thread(target=self.run, name='go2-executor-pulse', daemon=True)

    def start(self):
        """在节点加入执行器后启动单一唤醒线程。"""
        self.thread.start()

    def run(self):
        """用单调期限唤醒 guard condition；错过期限时跳过，避免积压补发。"""
        deadline = time.monotonic() + self.period
        try:
            while not self.quit.wait(max(0., deadline - time.monotonic())):
                now = time.monotonic()
                for executor in self.executors:
                    # wake 只打断 wait_set.wait；禁止在这里 spin、刷新输入时间或发送命令。
                    executor.wake()
                previous = self.state
                self.state = dict(ticks=previous['ticks'] + 1, last_wall=now,
                                  maximum_lateness_s=max(previous['maximum_lateness_s'], max(0., now - deadline)),
                                  error='')
                # 不使用 time.time：WSL 日期回拨不能把本线程的下一次唤醒推迟数秒。
                deadline += self.period
                if deadline <= now:
                    deadline = now + self.period
        except Exception as error:
            self.state = dict(self.state, error=str(error))

    def diagnostics(self):
        """报告真实唤醒进度与异常；此状态不参与里程计、深度或控制健康判定。"""
        return dict(self.state, period_s=self.period, worker_alive=self.thread.is_alive())

    def close(self):
        """先停并回收唤醒线程，随后才可销毁执行器的 guard condition。"""
        self.quit.set()
        if self.thread.ident is not None:
            self.thread.join(timeout=5)
        if self.thread.is_alive():
            raise RuntimeError('执行器单调唤醒线程未退出')
