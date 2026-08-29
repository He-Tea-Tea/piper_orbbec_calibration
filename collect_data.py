# -*- coding: utf-8 -*-
"""
奥比中光 Gemini 335Le + 松灵 PiPER（6轴）—— 眼在手外 数据采集

功能：
  1. 实时显示相机画面
  2. 按 s 时读取当前 TCP 末端位姿
  3. 同步保存 N.jpg + poses.txt + tcp_poses.csv
  4. poses.txt 保持每行 6 个数，兼容原 compute_to_hand.py

单位：
  x, y, z       -> m
  roll,pitch,yaw -> rad

重要：
  TCP_OFFSET 表示 TCP 相对于末端法兰坐标系的偏移。
  如果 TCP_OFFSET 全 0，则 TCP == flange（法兰中心）。
"""

import argparse
import csv
import logging
import os
import platform
import sys
import time

import cv2
import numpy as np

from pyorbbecsdk import OBFormat, Pipeline

from libs.auxiliary import create_folder_with_date, popup_message
from libs.log_setting import CommonLog


# ============================================================
# 用户配置
# ============================================================
# 多臂配置：每条臂一个 CAN 通道 + TCP 偏移
# - Windows 官方 CANDO：interface=agx_cando，channel "0"/"1"
#   （一个 USB-CAN 设备一个通道；多张卡时在设备管理器里看编号）
# - Ubuntu/Linux SocketCAN：interface=socketcan，channel "can0"/"can1"
if platform.system() == "Windows":
    _DEFAULT_CHANNELS = {"arm0": "0", "arm1": "1"}
else:
    _DEFAULT_CHANNELS = {"arm0": "can0", "arm1": "can1"}

# TCP 相对末端法兰的位姿偏移：[x, y, z, roll, pitch, yaw]
# 位置单位 m，角度单位 rad。全 0 表示 TCP == flange（法兰中心）。
# 换夹爪后应重新标定工具坐标（见 README 第 9 节），并把数值填到这里。
# fw_profile：机械臂固件驱动 profile（default/v183/v188/v189）。
#   固件版本不匹配会导致位姿无法获取/数值乱，见 README 排查。
#   保持 "default" 即可：连接时会读固件版本并自动切换到正确 profile。
ARMS = {
    "arm0": {
        "can_channel": _DEFAULT_CHANNELS["arm0"],
        "tcp_offset": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "fw_profile": "default",
    },
    "arm1": {
        "can_channel": _DEFAULT_CHANNELS["arm1"],
        "tcp_offset": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "fw_profile": "default",
    },
}
DEFAULT_ARM = "arm0"

# 连接后等待第一帧有效机械臂反馈的最长时间
POSE_WAIT_TIMEOUT = 3.0
POSE_POLL_INTERVAL = 0.02


logger_ = CommonLog(logging.getLogger(__name__))

# 项目根目录（脚本所在目录）
BASE_DIR = os.path.dirname(os.path.abspath(__file__))

# 棋盘格实时检测：每 N 帧检测一次（结果缓存用于其余帧的叠加显示），
# 参数与 compute_to_hand.py 标定时一致，保证预览所见即标定所用。
CHESS_DETECT_EVERY_N_FRAMES = 3


def load_checkerboard_size(config_path):
    """读棋盘格角点数 (XX, YY)；读取失败时回退 (9, 6) 并告警。"""
    import yaml

    try:
        with open(config_path, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f)
        a = cfg["checkerboard_args"]
        xx, yy = int(a["XX"]), int(a["YY"])
        if xx < 2 or yy < 2:
            raise ValueError("角点数至少 2x2")
        return xx, yy
    except Exception as exc:
        logger_.warning(f"读取棋盘格参数失败({exc})，回退默认 9x6，"
                        f"请检查 {config_path} 与 config.yaml 里 XX/YY 一致")
        return 9, 6


def _read_terminal_keys():
    """
    非阻塞读取终端键盘输入（Linux: select；Windows: msvcrt）。

    解决 cv2.waitKey 只能收 OpenCV 窗口按键、窗口无焦点/被转发到远程时
    按 s 无效的问题：终端里按 s 回车（或 q 回车）即可保存/退出。
    返回字符列表。
    """
    keys = []
    if platform.system() == "Windows":
        import msvcrt
        while msvcrt.kbhit():
            try:
                keys.append(msvcrt.getch().decode())
            except Exception:
                pass
    else:
        import select
        while select.select([sys.stdin], [], [], 0.0)[0]:
            ch = sys.stdin.read(1)
            if not ch:
                break
            keys.append(ch)
    return keys


def normalize_arm(arm):
    """把用户输入规范成 ARMS 里的键："0"/"arm0" -> "arm0"。"""
    arm = str(arm).strip().lower()
    if arm.isdigit():
        arm = f"arm{arm}"
    if arm not in ARMS:
        raise ValueError(f"未知机械臂: {arm}，可用: {list(ARMS)}")
    return arm


def parse_args():
    p = argparse.ArgumentParser(description="眼在手外数据采集（支持多臂，同一相机轮流标定）")
    p.add_argument("--arm", default=DEFAULT_ARM,
                   help=f"机械臂 ID（默认 {DEFAULT_ARM}）：{', '.join(ARMS)}")
    return p.parse_args()


# ============================================================
# 机械臂
# ============================================================
def connect_piper(arm_cfg):
    """连接 PiPER（固件版本自动适配），并配置 TCP offset。

    返回 (robot, used_fw_profile)。连接失败抛出 RuntimeError。
    """
    from libs.arm_connection import connect_arm

    robot, used_profile, _firmware = connect_arm(
        arm_cfg, fw_profile=arm_cfg.get("fw_profile", "default")
    )

    # get_tcp_pose() 是根据 flange pose + TCP offset 计算出的 TCP 位姿。
    # tcp_offset 全 0 时，TCP 与 flange 重合。
    tcp_offset = arm_cfg["tcp_offset"]
    if hasattr(robot, "set_tcp_offset"):
        robot.set_tcp_offset(tcp_offset)
    elif any(abs(v) > 1e-12 for v in tcp_offset):
        raise RuntimeError(
            "当前 pyAgxArm 没有 set_tcp_offset()，但该臂 TCP_OFFSET 非零。"
            "请升级 pyAgxArm 后再采集真实 TCP 位姿。"
        )

    return robot, used_profile


def _extract_pose_msg(ret):
    """从 pyAgxArm MessageAbstract 中提取 6 维 pose。"""
    if ret is None:
        return None, None, None

    msg = getattr(ret, "msg", None)
    if msg is None:
        return None, None, None

    try:
        values = [float(v) for v in msg]
    except Exception:
        return None, None, None

    if len(values) != 6 or not np.all(np.isfinite(values)):
        return None, None, None

    sdk_timestamp = getattr(ret, "timestamp", None)
    hz = getattr(ret, "hz", None)
    return values, sdk_timestamp, hz


def get_current_tcp_pose(robot):
    """
    读取当前 TCP 位姿。

    返回：
      (True, pose, sdk_timestamp, hz)
      (False, error_string, None, None)
    """
    if robot is None:
        return False, "机械臂未连接", None, None

    if not hasattr(robot, "get_tcp_pose"):
        return (
            False,
            "当前 pyAgxArm 版本没有 get_tcp_pose()，请升级官方 pyAgxArm",
            None,
            None,
        )

    try:
        ret = robot.get_tcp_pose()
    except Exception as exc:
        return False, f"调用 get_tcp_pose() 异常: {exc}", None, None

    pose, sdk_timestamp, hz = _extract_pose_msg(ret)
    if pose is None:
        return False, f"TCP 位姿反馈无效: {ret}", None, None

    return True, pose, sdk_timestamp, hz


def wait_for_valid_tcp_pose(robot, timeout=POSE_WAIT_TIMEOUT):
    """机械臂 connect() 后反馈线程需要一点时间，循环等待第一帧有效 TCP 位姿。"""
    deadline = time.monotonic() + timeout
    last_error = "尚未收到 TCP 位姿反馈"

    while time.monotonic() < deadline:
        state, data, sdk_timestamp, hz = get_current_tcp_pose(robot)
        if state:
            return True, data, sdk_timestamp, hz
        last_error = data
        time.sleep(POSE_POLL_INTERVAL)

    return False, last_error, None, None


# ============================================================
# 相机
# ============================================================
def color_frame_to_bgr(color_frame):
    h = color_frame.get_height()
    w = color_frame.get_width()
    data = np.asanyarray(color_frame.get_data())
    fmt = color_frame.get_format()

    if fmt == OBFormat.RGB:
        return cv2.cvtColor(np.resize(data, (h, w, 3)), cv2.COLOR_RGB2BGR)
    if fmt == OBFormat.MJPG:
        return cv2.imdecode(data, cv2.IMREAD_COLOR)
    return None


def start_camera():
    pipeline = Pipeline()
    pipeline.start()
    return pipeline


# ============================================================
# 保存
# ============================================================
def init_csv_file(folder):
    csv_path = os.path.join(folder, "tcp_poses.csv")
    if not os.path.exists(csv_path):
        with open(csv_path, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow([
                "index",
                "sdk_timestamp",
                "x_m",
                "y_m",
                "z_m",
                "roll_rad",
                "pitch_rad",
                "yaw_rad",
            ])
    return csv_path


def save_sample(folder, count, bgr, pose, sdk_timestamp):
    """
    保存一组同步数据。

    poses.txt：保持原标定程序兼容格式
    tcp_poses.csv：增加编号和时间戳，方便检查
    N.jpg：对应同一编号
    """
    image_path = os.path.join(folder, f"{count}.jpg")
    poses_path = os.path.join(folder, "poses.txt")
    csv_path = os.path.join(folder, "tcp_poses.csv")

    # 先保存图片；若后续位姿写入失败，则删除本张图片，避免图像/位姿错位。
    if not cv2.imwrite(image_path, bgr):
        raise IOError(f"图像保存失败: {image_path}")

    try:
        pose_line = ",".join(f"{v:.9f}" for v in pose)

        with open(poses_path, "a", encoding="utf-8") as f:
            f.write(pose_line + "\n")
            f.flush()
            os.fsync(f.fileno())

        with open(csv_path, "a", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow([
                count,
                "" if sdk_timestamp is None else f"{float(sdk_timestamp):.9f}",
                *[f"{v:.9f}" for v in pose],
            ])
            f.flush()
            os.fsync(f.fileno())

    except Exception:
        # 只要位姿保存失败，就删除对应图片，防止后续 compute_to_hand.py 错位。
        try:
            if os.path.exists(image_path):
                os.remove(image_path)
        except OSError:
            pass
        raise


# ============================================================
# 主流程
# ============================================================
def main():
    count = 1
    robot = None
    pipeline = None

    args = parse_args()
    arm = normalize_arm(args.arm)
    arm_cfg = ARMS[arm]
    cam0_origin_path = create_folder_with_date(arm=arm)

    # 1. 机械臂
    try:
        robot, fw_profile = connect_piper(arm_cfg)
        state, pose, sdk_timestamp, hz = wait_for_valid_tcp_pose(robot)
        if not state:
            raise RuntimeError(pose)

        logger_.info(
            f"机械臂 [{arm}] 连接成功（固件 profile={fw_profile}），"
            f"当前 TCP 位姿 [x,y,z,roll,pitch,yaw] (m/rad): {pose}"
        )
        logger_.info(
            f"[{arm}] CAN channel={arm_cfg['can_channel']}, "
            f"TCP_OFFSET (flange -> TCP): {arm_cfg['tcp_offset']}"
        )
        if hz is not None:
            logger_.info(f"当前 TCP 反馈频率: {hz} Hz")

    except Exception as e:
        logger_.error(f"机械臂连接/读取 TCP 失败: {e}")
        popup_message(
            "提醒",
            "机械臂连接或 TCP 位姿读取失败。\n"
            "请检查 USB-CAN、CAN 波特率、固件版本以及 pyAgxArm 版本。",
        )
        if robot is not None and hasattr(robot, "disconnect"):
            robot.disconnect()
        sys.exit(1)

    # 2. 相机
    try:
        pipeline = start_camera()
    except Exception as e:
        logger_.error(f"相机连接异常: {e}")
        popup_message("提醒", "相机连接异常，请检查 USB 连接")
        if robot is not None and hasattr(robot, "disconnect"):
            robot.disconnect()
        sys.exit(1)

    init_csv_file(cam0_origin_path)

    # 棋盘格尺寸（与 config.yaml / compute_to_hand.py 一致）
    xx, yy = load_checkerboard_size(os.path.join(BASE_DIR, "config.yaml"))
    logger_.info(f"[{arm}] 棋盘格角点数: {xx}x{yy}（实时检测叠加显示，"
                 f"每 {CHESS_DETECT_EVERY_N_FRAMES} 帧检测一次）")

    logger_.info(f"[{arm}] 开始采集：机械臂停稳后按 s 保存（终端按 s 回车即可，无需点击视频窗口）；q 回车或 Esc 退出")
    logger_.info(f"[{arm}] 数据保存目录: {os.path.abspath(cam0_origin_path)}")
    logger_.info("poses.txt 每行对应同编号 JPG；tcp_poses.csv 额外保存编号和时间戳")

    # 棋盘格检测状态（缓存，供画面叠加显示）
    frame_idx = 0
    cb_detected = False
    cb_corners = None
    cb_criteria = (cv2.TERM_CRITERIA_MAX_ITER | cv2.TERM_CRITERIA_EPS, 30, 0.001)

    try:
        while True:
            frames = pipeline.wait_for_frames(1000)
            if frames is None:
                continue

            color_frame = frames.get_color_frame()
            if color_frame is None:
                continue

            bgr = color_frame_to_bgr(color_frame)
            if bgr is None:
                continue

            # 棋盘格实时检测（降频执行，结果缓存）
            frame_idx += 1
            if frame_idx % CHESS_DETECT_EVERY_N_FRAMES == 0:
                gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
                ret_cb, corners = cv2.findChessboardCorners(gray, (xx, yy), None)
                if ret_cb:
                    corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1),
                                               cb_criteria)
                cb_detected, cb_corners = ret_cb, corners

            # 画面上显示操作提示 + 棋盘格检测状态
            preview = bgr.copy()
            cv2.putText(
                preview,
                "S: save image + TCP pose    ESC: quit",
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            if cb_detected and cb_corners is not None:
                # 框出棋盘格角点（绿色连线 + 角点）
                cv2.drawChessboardCorners(preview, (xx, yy), cb_corners, True)
                cv2.putText(
                    preview,
                    f"CHESSBOARD OK ({xx}x{yy})",
                    (20, 75),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )
            else:
                cv2.putText(
                    preview,
                    "NO CHESSBOARD",
                    (20, 75),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 0, 255),
                    2,
                    cv2.LINE_AA,
                )
            cv2.imshow("Capture_Video", preview)

            # 按键双通道：OpenCV 窗口按键 + 终端键盘输入（select/msvcrt）。
            # 终端里按 s 回车即可保存、q 回车退出——无需点击视频窗口，
            # 也不受 DISPLAY 转发/窗口焦点影响。
            keys = []
            k = cv2.waitKey(30) & 0xFF
            if k not in (-1, 255):
                keys.append(chr(k))
            keys.extend(_read_terminal_keys())

            quit_flag = False
            for key in keys:
                key = key.lower()
                if key == "s":
                    # 按键瞬间读取一次 TCP 位姿，尽量与当前保存图像对应。
                    state, pose, sdk_timestamp, hz = get_current_tcp_pose(robot)

                    if not state:
                        logger_.error(f"第 {count} 组保存失败，TCP 位姿无效: {pose}")
                        continue

                    if not cb_detected:
                        logger_.warning(
                            f"第 {count} 组：当前画面未检测到棋盘格，"
                            "保存的图片可能无法用于标定，请确认棋盘格完整清晰"
                        )
                    else:
                        logger_.info(
                            f"第 {count} 组：棋盘格检测正常（{xx}x{yy}）"
                        )

                    try:
                        save_sample(
                            cam0_origin_path,
                            count,
                            bgr,
                            pose,
                            sdk_timestamp,
                        )
                    except Exception as exc:
                        logger_.error(f"第 {count} 组数据写入失败: {exc}")
                        continue

                    logger_.info(
                        f"=== 已保存第 {count} 组，TCP(m/rad): "
                        f"[{pose[0]:.9f}, {pose[1]:.9f}, {pose[2]:.9f}, "
                        f"{pose[3]:.9f}, {pose[4]:.9f}, {pose[5]:.9f}]"
                    )
                    count += 1

                elif key in ("q", chr(27)):
                    quit_flag = True

            if quit_flag:
                break

    finally:
        if pipeline is not None:
            pipeline.stop()
        cv2.destroyAllWindows()
        if robot is not None and hasattr(robot, "disconnect"):
            robot.disconnect()


if __name__ == "__main__":
    main()
