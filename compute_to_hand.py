# -*- coding: utf-8 -*-
"""
眼在手外 —— 手眼标定求解

输入：eye_hand_data 下最新数据目录中的 棋盘格图像（N.jpg）+ 机械臂位姿（poses.txt）
流程：
  1. 逐张检测棋盘格角点（findChessboardCorners + cornerSubPix 亚像素精化）
  2. calibrateCamera（张正友法）：求相机内参 + 每张图标定板在相机系下的位姿
  3. poses2_main：把机械臂位姿转逆齐次矩阵，写 RobotToolPose.csv
  4. calibrateHandEye（Tsai 两步法）：解出 相机在基座系下的位姿
输出：旋转矩阵 R、平移向量 t、四元数；并保存到 calib_<臂ID>.json（如 calib_arm0.json）

用法：
  python compute_to_hand.py                 # 标定最近数据目录（目录名带臂名则自动识别）
  python compute_to_hand.py --arm arm0      # 标定 arm0 的最新数据目录
  python compute_to_hand.py <数据目录> --arm arm0   # 显式指定目录和臂
"""
import argparse
import json
import logging
import os
import re
import sys

import cv2
import numpy as np
import yaml

from libs.auxiliary import find_latest_data_folder
from libs.log_setting import CommonLog
from save_poses2 import poses2_main

logger_ = logging.getLogger(__name__)
logger_ = CommonLog(logger_)

np.set_printoptions(precision=8, suppress=True)

# 项目根目录（脚本所在目录）
BASE_DIR = os.path.dirname(os.path.abspath(__file__))


def load_checkerboard_args(config_path):
    """读棋盘格参数。返回 (XX, YY, L)。"""
    with open(config_path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    args = cfg["checkerboard_args"]
    return int(args["XX"]), int(args["YY"]), float(args["L"])


def normalize_arm(arm):
    """把用户输入规范成臂 ID："0"/"arm0" -> "arm0"。"""
    arm = str(arm).strip().lower()
    if arm.isdigit():
        arm = f"arm{arm}"
    if not re.fullmatch(r"arm\d+", arm):
        raise ValueError(f"机械臂 ID 格式应为 arm0/arm1/...，收到: {arm}")
    return arm


def infer_arm_from_dir(data_dir):
    """从数据目录名推断臂 ID，如 data20240101_arm0 -> "arm0"；无臂名返回 None。"""
    m = re.search(r"_arm(\d+)", os.path.basename(str(data_dir)))
    return f"arm{m.group(1)}" if m else None


def parse_args():
    p = argparse.ArgumentParser(description="手眼标定求解（眼在手外，支持多臂）")
    p.add_argument("data_dir", nargs="?", default=None,
                   help="数据目录路径（默认取该臂最新目录）")
    p.add_argument("--arm", default=None,
                   help="机械臂 ID：arm0/arm1/...（默认从目录名推断，推断不出用 arm0）")
    return p.parse_args()


def rotation_matrix_to_quaternion(R):
    """旋转矩阵 -> 四元数 [x, y, z, w]（Shepperd 法，数值稳定）。"""
    R = np.asarray(R, dtype=float)
    trace = np.trace(R)
    if trace > 0:
        s = np.sqrt(trace + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return np.array([x, y, z, w])


def func(data_dir):
    """执行完整标定，返回 (R, t)。data_dir 是包含 N.jpg 与 poses.txt 的目录。"""
    XX, YY, L = load_checkerboard_args(os.path.join(BASE_DIR, "config.yaml"))
    images_dir = data_dir
    poses_file = os.path.join(data_dir, "poses.txt")

    # 棋盘格 3D 点（世界坐标系建在标定板上，Z=0）
    objp = np.zeros((XX * YY, 3), np.float32)
    objp[:, :2] = np.mgrid[0:XX, 0:YY].T.reshape(-1, 2)
    objp = L * objp

    criteria = (cv2.TERM_CRITERIA_MAX_ITER | cv2.TERM_CRITERIA_EPS, 30, 0.001)

    obj_points, img_points = [], []
    used_index = []  # 成功检测角点的图片序号（与 poses.txt 行号对应）

    # 图片从 1.jpg 开始，与 collect_data.py 的命名一致
    images_num = [f for f in os.listdir(images_dir) if f.endswith(".jpg")]
    for i in range(1, len(images_num) + 1):
        image_file = os.path.join(images_dir, f"{i}.jpg")
        if not os.path.exists(image_file):
            continue
        img = cv2.imread(image_file)
        if img is None:
            logger_.warning(f"读图失败(跳过): {image_file}")
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        size = gray.shape[::-1]

        ret, corners = cv2.findChessboardCorners(gray, (XX, YY), None)
        if not ret:
            logger_.warning(f"角点检测失败(跳过): {image_file}")
            continue

        corners2 = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)
        obj_points.append(objp)
        img_points.append(corners2)
        used_index.append(i)

    N = len(img_points)
    if N < 3:
        raise RuntimeError(f"有效棋盘格图像仅 {N} 张，至少需要 3 张，请重新采集")

    # 相机标定（张正友法）：内参 + 每张图标定板在相机系下的位姿
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, size, None, None)
    logger_.info(f"相机内参标定完成，重投影误差 RMS = {ret:.4f} 像素")
    logger_.info(f"内参矩阵:\n{mtx}")

    # 机械臂位姿 -> 逆齐次矩阵 CSV（注意：只保留角点检测成功的帧对应的位姿行）
    # poses.txt 每行对应一张图；used_index 里的 i 就是 poses.txt 的第 i 行
    pose_lines = []
    with open(poses_file, "r", encoding="utf-8") as f:
        all_lines = f.read().strip().splitlines()
    if len(all_lines) < max(used_index):
        raise RuntimeError(
            f"poses.txt 行数({len(all_lines)}) < 需要的最小编号({max(used_index)})，"
            "请确认采集时图片与位姿一一对应"
        )
    for i in used_index:
        pose_lines.append(all_lines[i - 1])

    filtered_poses = os.path.join(BASE_DIR, "poses_filtered.txt")
    with open(filtered_poses, "w", encoding="utf-8") as f:
        f.write("\n".join(pose_lines) + "\n")

    csv_file = os.path.join(BASE_DIR, "RobotToolPose.csv")
    poses2_main(filtered_poses, csv_file)
    tool_pose = np.loadtxt(csv_file, delimiter=",")

    R_tool, t_tool = [], []
    for i in range(N):
        R_tool.append(tool_pose[0:3, 4 * i:4 * i + 3])
        t_tool.append(tool_pose[0:3, 4 * i + 3])

    # 手眼标定（Tsai 两步法）
    R, t = cv2.calibrateHandEye(
        R_tool, t_tool, rvecs, tvecs, cv2.CALIB_HAND_EYE_TSAI
    )
    return R, t, mtx, dist, ret, used_index


def main():
    args = parse_args()

    # 数据目录：显式传入，或按臂（--arm）找最新目录
    if args.data_dir:
        data_dir = args.data_dir
    else:
        latest = find_latest_data_folder(
            os.path.join(BASE_DIR, "eye_hand_data"), arm=args.arm
        )
        if latest is None:
            raise RuntimeError("未找到 eye_hand_data 下的数据目录，请先运行 collect_data.py 采集")
        data_dir = latest

    # 臂 ID：优先 --arm，其次从目录名推断（data..._arm0），最后默认 arm0
    arm = args.arm or infer_arm_from_dir(data_dir) or "arm0"
    arm = normalize_arm(arm)

    logger_.info(f"使用数据目录: {data_dir}")
    logger_.info(f"机械臂 ID: {arm}")
    R, t, mtx, dist, rms, used_index = func(data_dir)

    # 输出结果
    print("=" * 60)
    print(f"相机标定重投影 RMS: {rms:.4f} 像素")
    print(f"有效图像数: {len(used_index)}")
    print("\n旋转矩阵 R（相机->基座）:")
    print(R)
    print("\n平移向量 t（米）:")
    print(t)

    quat = rotation_matrix_to_quaternion(R)
    print("\n四元数 [x, y, z, w]:")
    print(quat)

    # 保存结果 JSON，供 verify_calibration.py / 后续识别抓取直接使用
    result = {
        "mode": "eye_to_hand",
        "arm": arm,
        "R": R.tolist(),
        "t": t.flatten().tolist(),
        "quaternion_xyzw": quat.tolist(),
        "intrinsics": {
            "mtx": mtx.tolist(),
            "dist": dist.ravel().tolist(),
            "rms_px": float(rms),
        },
        "data_dir": data_dir,
    }
    out_json = os.path.join(BASE_DIR, f"calib_{arm}.json")
    with open(out_json, "w", encoding="utf-8") as f:
        json.dump(result, f, indent=2, ensure_ascii=False)
    logger_.info(f"标定结果已保存: {out_json}")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        logger_.error(f"标定失败: {e}")
        sys.exit(1)
