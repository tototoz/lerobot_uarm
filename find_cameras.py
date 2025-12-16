#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
浏览器实时相机查看器（OpenCV & RealSense, 不保存帧）
- 枚举并连接所有相机（可按类型过滤）
- 通过 Flask 提供 MJPEG 流，网页里实时刷新
- 适合无桌面的服务器/SSH 环境
用法：
    python find_cameras.py
    python find_cameras.py opencv --host 0.0.0.0 --port 8000
    python find_cameras.py realsense --max-width 960
"""

import argparse
import logging
import threading
import time
from typing import Any, Dict, List

import numpy as np
from lerobot.cameras.configs import ColorMode
from lerobot.cameras.opencv.camera_opencv import OpenCVCamera
from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
from lerobot.cameras.realsense.camera_realsense import RealSenseCamera
from lerobot.cameras.realsense.configuration_realsense import RealSenseCameraConfig

import cv2
from flask import Flask, Response, render_template_string

logger = logging.getLogger(__name__)

# ---------------- 相机发现 ----------------
def find_all_opencv_cameras() -> List[Dict[str, Any]]:
    infos: List[Dict[str, Any]] = []
    logger.info("正在搜索 OpenCV 摄像头...")
    try:
        for cam_info in OpenCVCamera.find_cameras():
            infos.append(cam_info)
        logger.info(f"找到 {len(infos)} 个 OpenCV 摄像头。")
    except Exception as e:
        logger.error(f"搜索 OpenCV 摄像头出错: {e}")
    return infos


def find_all_realsense_cameras() -> List[Dict[str, Any]]:
    infos: List[Dict[str, Any]] = []
    logger.info("正在搜索 RealSense 摄像头...")
    try:
        for cam_info in RealSenseCamera.find_cameras():
            infos.append(cam_info)
        logger.info(f"找到 {len(infos)} 个 RealSense 摄像头。")
    except ImportError:
        logger.warning("跳过 RealSense：未安装 pyrealsense2。")
    except Exception as e:
        logger.error(f"搜索 RealSense 摄像头出错: {e}")
    return infos


def find_and_print_cameras(camera_type_filter: str | None = None) -> List[Dict[str, Any]]:
    all_infos: List[Dict[str, Any]] = []
    if camera_type_filter:
        camera_type_filter = camera_type_filter.lower()
    if camera_type_filter is None or camera_type_filter == "opencv":
        all_infos.extend(find_all_opencv_cameras())
    if camera_type_filter is None or camera_type_filter == "realsense":
        all_infos.extend(find_all_realsense_cameras())

    if not all_infos:
        logger.warning("未检测到任何摄像头。")
    else:
        print("\n--- Detected Cameras ---")
        for i, cam_info in enumerate(all_infos):
            print(f"Camera #{i}:")
            for k, v in cam_info.items():
                if k == "default_stream_profile" and isinstance(v, dict):
                    print(f"  {k}:")
                    for kk, vv in v.items():
                        print(f"    {kk}: {vv}")
                else:
                    print(f"  {k}: {v}")
            print("-" * 20)
    return all_infos

# ---------------- 相机创建与清理 ----------------
def create_camera_instance(cam_meta: Dict[str, Any]) -> Dict[str, Any] | None:
    cam_type = cam_meta.get("type")
    cam_id = cam_meta.get("id")
    instance = None
    logger.info(f"准备启动 {cam_type} 相机：{cam_id}")
    try:
        if cam_type == "OpenCV":
            cfg = OpenCVCameraConfig(index_or_path=cam_id, color_mode=ColorMode.RGB)
            instance = OpenCVCamera(cfg)
        elif cam_type == "RealSense":
            cfg = RealSenseCameraConfig(serial_number_or_name=cam_id, color_mode=ColorMode.RGB)
            instance = RealSenseCamera(cfg)
        else:
            logger.warning(f"未知相机类型: {cam_type} (id={cam_id})，跳过。")
            return None
        instance.connect(warmup=False)
        logger.info(f"已连接：{cam_type} {cam_id}")
        return {"instance": instance, "meta": cam_meta}
    except Exception as e:
        logger.error(f"连接/配置 {cam_type} 相机失败（{cam_id}）：{e}")
        try:
            if instance and getattr(instance, "is_connected", False):
                instance.disconnect()
        except Exception:
            pass
        return None


def cleanup_cameras(cameras: List[Dict[str, Any]]):
    logger.info(f"正在断开 {len(cameras)} 个相机...")
    for camd in cameras:
        try:
            if camd["instance"] and camd["instance"].is_connected:
                camd["instance"].disconnect()
        except Exception as e:
            logger.error(f"断开相机 {camd['meta'].get('id')} 出错: {e}")

# ---------------- 采集线程 & MJPEG ----------------
class MultiCamStreamer:
    def __init__(self, cameras: List[Dict[str, Any]], max_width: int | None = 960, fps_limit: float | None = 30.0):
        self.cameras = cameras
        self.max_width = max_width
        self.fps_limit = fps_limit
        self.latest_bgr: Dict[int, np.ndarray | None] = {i: None for i in range(len(cameras))}
        self.fail_count: Dict[int, int] = {i: 0 for i in range(len(cameras))}
        self.running = threading.Event()
        self.running.set()
        self.threads: List[threading.Thread] = []

    def _resize(self, img: np.ndarray) -> np.ndarray:
        if self.max_width is None or img.shape[1] <= self.max_width:
            return img
        h, w = img.shape[:2]
        scale = self.max_width / float(w)
        new_size = (int(w * scale), int(h * scale))
        return cv2.resize(img, new_size, interpolation=cv2.INTER_LINEAR)

    def grab_loop(self, idx: int):
        cam = self.cameras[idx]["instance"]
        while self.running.is_set():
            t0 = time.perf_counter()
            try:
                rgb = cam.read()  # 约定返回 RGB numpy 数组
                if rgb is None:
                    raise RuntimeError("read() 返回 None")
                bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                bgr = self._resize(bgr)
                self.latest_bgr[idx] = bgr
                self.fail_count[idx] = 0
            except Exception as e:
                self.fail_count[idx] += 1
                if self.fail_count[idx] == 1 or self.fail_count[idx] % 30 == 0:
                    logger.error(f"[Camera #{idx}] 连续读取失败 {self.fail_count[idx]} 次：{e}")
                # 连续太多失败：置空帧，但继续尝试，防止偶发断流
                self.latest_bgr[idx] = None
                time.sleep(0.05)

            if self.fps_limit:
                dt = time.perf_counter() - t0
                sleep_t = max(0.0, (1.0 / self.fps_limit) - dt)
                if sleep_t > 0:
                    time.sleep(sleep_t)

    def start(self):
        for i in range(len(self.cameras)):
            th = threading.Thread(target=self.grab_loop, args=(i,), daemon=True)
            th.start()
            self.threads.append(th)

    def stop(self):
        self.running.clear()
        for th in self.threads:
            th.join(timeout=1.0)

    def mjpeg_generator(self, idx: int):
        boundary = b"--frame"
        while self.running.is_set():
            frame = self.latest_bgr.get(idx)
            if frame is None:
                time.sleep(0.01)
                continue
            ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            if not ok:
                continue
            jpg = buf.tobytes()
            yield boundary + b"\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n"

# ---------------- Flask 应用 ----------------
HTML = """
<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <title>Multi-Cam Viewer</title>
  <style>
    body{font-family:system-ui,Arial;margin:16px;background:#0b0b0c;color:#e5e7eb}
    .grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(320px,1fr));gap:12px}
    .card{background:#111827;border-radius:12px;padding:8px;box-shadow:0 2px 8px rgba(0,0,0,.3)}
    .meta{font-size:12px;color:#9ca3af;margin:4px 0 8px}
    img{width:100%;height:auto;border-radius:8px;background:#000}
    .hdr{display:flex;justify-content:space-between;align-items:center;margin-bottom:8px}
    a{color:#60a5fa;text-decoration:none}
  </style>
</head>
<body>
  <div class="hdr">
    <div><strong>Multi-Cam Viewer</strong></div>
    <div class="meta">{{ cams|length }} camera(s)</div>
  </div>
  <div class="grid">
    {% for i,meta in cams %}
    <div class="card">
      <div class="meta">{{ meta.get('name','') }} — {{ meta.get('id','') }} ({{ meta.get('type','') }})</div>
      <img src="/stream/{{ i }}">
    </div>
    {% endfor %}
  </div>
  <div class="meta" style="margin-top:12px;">按 Ctrl+C 结束。刷新页面可重连流。</div>
</body>
</html>
"""

def run_web_server(cameras: List[Dict[str, Any]], host: str, port: int, max_width: int | None, fps_limit: float | None):
    app = Flask(__name__)
    streamer = MultiCamStreamer(cameras, max_width=max_width, fps_limit=fps_limit)
    streamer.start()

    @app.route("/")
    def index():
        meta_list = [(i, c["meta"]) for i, c in enumerate(cameras)]
        return render_template_string(HTML, cams=meta_list)

    @app.route("/stream/<int:idx>")
    def stream(idx: int):
        if idx < 0 or idx >= len(cameras):
            return "invalid index", 404
        return Response(streamer.mjpeg_generator(idx),
                        mimetype="multipart/x-mixed-replace; boundary=frame")

    try:
        app.run(host=host, port=port, threaded=True)
    finally:
        logger.info("正在停止采集线程并断开相机...")
        streamer.stop()
        cleanup_cameras(cameras)

# ---------------- CLI 入口 ----------------
def main():
    parser = argparse.ArgumentParser(description="浏览器实时相机查看器（不保存帧）")
    parser.add_argument("camera_type", nargs="?", default=None, choices=["realsense", "opencv"],
                        help="指定只使用某类相机（realsense/opencv），默认全部。")
    parser.add_argument("--host", default="127.0.0.1", help="监听地址（默认 127.0.0.1，仅本机访问；改为 0.0.0.0 允许局域网访问）")
    parser.add_argument("--port", type=int, default=8001, help="HTTP 端口（默认 8000）")
    parser.add_argument("--max-width", type=int, default=960, help="画面最大宽度（等比缩放；设 0 不缩放）")
    parser.add_argument("--fps", type=float, default=30.0, help="采集帧率上限（默认 30；设 0 不限）")
    parser.add_argument("--log-level", default="INFO", choices=["DEBUG","INFO","WARNING","ERROR"], help="日志级别")
    args = parser.parse_args()

    logging.basicConfig(level=getattr(logging, args.log_level), format="%(levelname)s:%(name)s:%(message)s")

    # 发现相机
    all_meta = find_and_print_cameras(args.camera_type)
    if not all_meta:
        return

    # 连接相机
    cameras: List[Dict[str, Any]] = []
    for meta in all_meta:
        inst = create_camera_instance(meta)
        if inst:
            cameras.append(inst)
    if not cameras:
        logger.error("无可用相机。")
        return

    max_w = None if args.max_width == 0 else args.max_width
    fps_lim = None if args.fps == 0 else args.fps

    # 启动 Web 预览
    try:
        run_web_server(cameras, host=args.host, port=args.port, max_width=max_w, fps_limit=fps_lim)
    except KeyboardInterrupt:
        logger.info("用户中断。")
    finally:
        cleanup_cameras(cameras)

if __name__ == "__main__":
    main()

