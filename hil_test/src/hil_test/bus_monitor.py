"""被动监听 CAN 帧 + 日志落盘 + 周期/ID 统计."""

import csv
import os
import threading
import time

from hil_test.can_socket import CanSocket


class BusMonitor:
    """只读监听 CAN 总线；记录帧、统计周期，供测试断言."""

    def __init__(self, interface, log_dir=None):
        self.interface = interface
        self._sock = CanSocket(interface, recv_timeout=0.02)
        self._lock = threading.Lock()
        self._frames = []       # [(t_monotonic, can_id, data_bytes)]
        self._last_ts = {}      # can_id → 最近时间戳
        self._running = False
        self._thread = None
        self._csv = None
        self._log_file = None
        if log_dir:
            os.makedirs(log_dir, exist_ok=True)
            self._log_file = open(
                os.path.join(log_dir, 'can_log.csv'), 'a', encoding='utf-8', newline='')
            self._csv = csv.writer(self._log_file)

    def start(self):
        """启动监听线程；接口不可用返回 False."""
        if not self._sock.open():
            return False
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        return True

    def _loop(self):
        while self._running:
            frame = self._sock.recv()
            if frame is None:
                continue
            can_id, data = frame
            t = time.monotonic()
            with self._lock:
                self._frames.append((t, can_id, data))
                self._last_ts[can_id] = t
            if self._csv is not None:
                self._csv.writerow([time.time(), hex(can_id), data.hex()])

    def stop(self):
        """停止监听并关闭."""

        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        self._sock.close()
        if self._log_file is not None:
            self._log_file.close()
            self._log_file = None

    # ---- 查询 ----
    def count(self, can_id):
        """统计某 ID 帧数."""
        with self._lock:
            return sum(1 for _, cid, _ in self._frames if cid == can_id)

    def latest(self, can_id):
        """最近一帧 (t_monotonic, data)；无则 None."""
        with self._lock:
            for t, cid, data in reversed(self._frames):
                if cid == can_id:
                    return t, data
        return None

    def periods(self, can_id):
        """某 ID 相邻帧间隔列表（秒）."""
        with self._lock:
            ts = [t for t, cid, _ in self._frames if cid == can_id]
        return [ts[i + 1] - ts[i] for i in range(len(ts) - 1)]

    def period_stats(self, can_id):
        """周期统计 {min, avg, max}（秒）；帧数不足返回 None."""
        periods = self.periods(can_id)
        if not periods:
            return None
        return {'min': min(periods), 'avg': sum(periods) / len(periods), 'max': max(periods)}

    def wait_for(self, can_id, timeout=5.0):
        """等待某 ID 帧出现；出现返回 True，超时 False."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.count(can_id) > 0:
                return True
            time.sleep(0.01)
        return False

    def clear(self):
        """清空已记录帧（用于分段测量）."""
        with self._lock:
            self._frames.clear()
            self._last_ts.clear()
