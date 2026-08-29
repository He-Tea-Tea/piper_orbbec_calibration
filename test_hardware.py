# -*- coding: utf-8 -*-
"""
硬件自检脚本：相机和机械臂分开验证。

用法：
  export DISPLAY=:0
  python3 test_hardware.py camera            # 验证奥比 335Le：弹出实时画面，Esc 退出
  python3 test_hardware.py arm               # 验证松灵 PiPER（默认 arm0）：打印当前 TCP 位姿
  python3 test_hardware.py arm arm1          # 指定臂：arm0 / arm1 / ...
"""
import platform
import sys

import cv2
import numpy as np


def arm_channels():
    """每臂 CAN 通道与接口（与 collect_data.py 的 ARMS 保持一致）。"""
    if platform.system() == "Windows":
        return {"arm0": ("agx_cando", "0"), "arm1": ("agx_cando", "1")}
    return {"arm0": ("socketcan", "can0"), "arm1": ("socketcan", "can2")}


def test_camera():
    from pyorbbecsdk import OBFormat, Pipeline

    print("启动 335Le 相机 ...")
    pipeline = Pipeline()
    pipeline.start()
    print("相机已启动，等待画面（按 Esc 退出）...")
    try:
        while True:
            frames = pipeline.wait_for_frames(1000)
            if frames is None:
                continue
            cf = frames.get_color_frame()
            if cf is None:
                continue
            h, w = cf.get_height(), cf.get_width()
            data = np.asanyarray(cf.get_data())
            fmt = cf.get_format()
            if fmt == OBFormat.RGB:
                bgr = cv2.cvtColor(np.resize(data, (h, w, 3)), cv2.COLOR_RGB2BGR)
            elif fmt == OBFormat.MJPG:
                bgr = cv2.imdecode(data, cv2.IMREAD_COLOR)
            else:
                continue
            cv2.imshow("camera_test", bgr)
            if cv2.waitKey(1) == 27:
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
    print("相机验证完成")


def test_arm(arm="arm0"):
    from libs.arm_connection import connect_arm, diagnose_arm

    channels = arm_channels()
    if arm not in channels:
        raise SystemExit(f"未知机械臂: {arm}，可用: {list(channels)}")
    interface, channel = channels[arm]

    print(f"连接松灵 PiPER [{arm}] (interface={interface}, channel={channel}) ...")
    arm_cfg = {
        "can_channel": channel,
        "can_bitrate": 1000000,
        "fw_profile": "default",   # 连接时自动读取固件版本并切换正确 profile
    }
    robot, used_profile, _firmware = connect_arm(arm_cfg, fw_profile="default")
    print(f"连接成功，实际固件 profile = {used_profile}")
    try:
        diagnose_arm(robot, arm_id=arm)
    finally:
        if hasattr(robot, "disconnect"):
            robot.disconnect()
    print("机械臂验证完成")


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) >= 2 else None
    if mode not in ("camera", "arm"):
        print(__doc__)
        sys.exit(1)
    if mode == "camera":
        test_camera()
    else:
        # 可选第二参数指定臂：python test_hardware.py arm arm1
        arm = sys.argv[2] if len(sys.argv) >= 3 else "arm0"
        if not arm.isdigit() and not arm.startswith("arm"):
            print(__doc__)
            sys.exit(1)
        if arm.isdigit():
            arm = f"arm{arm}"
        test_arm(arm)
