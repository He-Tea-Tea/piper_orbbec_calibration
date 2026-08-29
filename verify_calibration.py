# -*- coding: utf-8 -*-
"""
手眼标定精度验证（一致性误差评估）

两种用法：
  1) 用已有标定结果验证：
     python verify_calibration.py --mode eye_to_hand --calib-file calib_arm0.json
  2) 用训练集重算外参，再用验证集评估（推荐）：
     python verify_calibration.py --mode eye_to_hand \
         --train-dir eye_hand_data/data2026051801_arm0 \
         --eval-dir  eye_hand_data/data2026051802_arm0
  多臂：加 --arm arm0/arm1 可让默认目录查找只在该臂内进行。

评估原理：标定板固定在机械臂末端，用标定结果把每帧的标定板重建到
固定坐标系下（理论上恒定），以首帧为基准统计各帧平移/旋转偏差。
"""
import argparse
import json
import os
from typing import Dict, List, Tuple

import cv2
import numpy as np
import yaml

from libs.auxiliary import find_latest_data_folder

BASE_DIR = os.path.dirname(os.path.abspath(__file__))


def parse_args():
    p = argparse.ArgumentParser(description="验证手眼标定精度")
    p.add_argument("--mode", choices=["eye_in_hand", "eye_to_hand"], required=True)
    p.add_argument("--train-dir", default=None, help="训练集目录（可选）")
    p.add_argument("--eval-dir", default=None, help="验证集目录（默认该臂最新）")
    p.add_argument("--calib-file", default=None, help="已有标定结果 JSON")
    p.add_argument("--save-calib", default=None, help="把本次计算结果保存到 JSON")
    p.add_argument("--arm", default=None, help="机械臂 ID（arm0/arm1/...），用于过滤默认目录")
    p.add_argument("--config", default=os.path.join(BASE_DIR, "config.yaml"))
    return p.parse_args()


def load_checkerboard_args(config_path: str) -> Tuple[int, int, float]:
    with open(config_path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    a = cfg["checkerboard_args"]
    return int(a["XX"]), int(a["YY"]), float(a["L"])


def sorted_image_files(folder: str) -> List[str]:
    files = [f for f in os.listdir(folder) if f.lower().endswith(".jpg")]

    def key(name):
        stem = os.path.splitext(name)[0]
        return (0, int(stem)) if stem.isdigit() else (1, stem)

    files.sort(key=key)
    return [os.path.join(folder, f) for f in files]


def load_poses_txt(folder: str) -> List[List[float]]:
    path = os.path.join(folder, "poses.txt")
    if not os.path.exists(path):
        raise FileNotFoundError(f"未找到 poses.txt: {path}")
    poses = []
    with open(path, "r", encoding="utf-8") as f:
        for line_no, line in enumerate(f, start=1):
            line = line.strip()
            if not line:
                continue
            vals = [float(x) for x in line.split(",")]
            if len(vals) != 6:
                raise ValueError(f"poses.txt 第 {line_no} 行不是6个数")
            poses.append(vals)
    return poses


def euler_xyz_to_rot(rx, ry, rz) -> np.ndarray:
    """固定坐标系 XYZ 欧拉角（弧度）-> 旋转矩阵，R = Rz*Ry*Rx。"""
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(rx), -np.sin(rx)],
                   [0, np.sin(rx), np.cos(rx)]])
    Ry = np.array([[np.cos(ry), 0, np.sin(ry)],
                   [0, 1, 0],
                   [-np.sin(ry), 0, np.cos(ry)]])
    Rz = np.array([[np.cos(rz), -np.sin(rz), 0],
                   [np.sin(rz), np.cos(rz), 0],
                   [0, 0, 1]])
    return Rz @ Ry @ Rx


def pose_to_h(pose) -> np.ndarray:
    x, y, z, rx, ry, rz = pose
    h = np.eye(4)
    h[:3, :3] = euler_xyz_to_rot(rx, ry, rz)
    h[:3, 3] = [x, y, z]
    return h


def inv_h(h: np.ndarray) -> np.ndarray:
    h_inv = np.eye(4)
    h_inv[:3, :3] = h[:3, :3].T
    h_inv[:3, 3] = -h[:3, :3].T @ h[:3, 3]
    return h_inv


def build_dataset(folder: str, xx: int, yy: int, cell_m: float) -> Dict:
    image_paths = sorted_image_files(folder)
    if not image_paths:
        raise ValueError(f"目录内没有 .jpg 图片: {folder}")
    poses_all = load_poses_txt(folder)

    objp = np.zeros((xx * yy, 3), np.float32)
    objp[:, :2] = np.mgrid[0:xx, 0:yy].T.reshape(-1, 2)
    objp = cell_m * objp

    criteria = (cv2.TERM_CRITERIA_MAX_ITER | cv2.TERM_CRITERIA_EPS, 30, 0.001)
    obj_points, img_points, used_idx, skipped = [], [], [], []
    image_size = None

    for idx, path in enumerate(image_paths):
        img = cv2.imread(path)
        if img is None:
            skipped.append((idx, os.path.basename(path), "读图失败"))
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        image_size = gray.shape[::-1]
        ret, corners = cv2.findChessboardCorners(gray, (xx, yy), None)
        if not ret:
            skipped.append((idx, os.path.basename(path), "角点检测失败"))
            continue
        corners2 = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)
        obj_points.append(objp)
        img_points.append(corners2 if corners2 is not None else corners)
        used_idx.append(idx)

    if image_size is None:
        raise ValueError("没有可用图像")
    if len(img_points) < 3:
        raise ValueError(f"有效图像过少: {len(img_points)}，至少3张")

    rms, _, _, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, image_size, None, None)

    robot_h_list = []
    for i in used_idx:
        if i >= len(poses_all):
            raise ValueError(
                f"图片索引 {i} 超过 poses.txt 行数 {len(poses_all)}："
                "图像与位姿数量不一致"
            )
        robot_h_list.append(pose_to_h(poses_all[i]))

    return {
        "folder": folder,
        "used_indices": used_idx,
        "skipped": skipped,
        "robot_h_list": robot_h_list,   # H_base_ee
        "rvecs": rvecs,
        "tvecs": tvecs,
        "rms": float(rms),
    }


def handeye_from_dataset(mode: str, ds: Dict) -> np.ndarray:
    r_list, t_list = [], []
    for h_base_ee in ds["robot_h_list"]:
        h_use = h_base_ee if mode == "eye_in_hand" else inv_h(h_base_ee)
        r_list.append(h_use[:3, :3])
        t_list.append(h_use[:3, 3])
    R, t = cv2.calibrateHandEye(r_list, t_list, ds["rvecs"], ds["tvecs"],
                                cv2.CALIB_HAND_EYE_TSAI)
    h = np.eye(4)
    h[:3, :3] = R
    h[:3, 3] = np.asarray(t).reshape(3)
    return h


def rvec_tvec_to_h_obj_cam(rvec, tvec) -> np.ndarray:
    R, _ = cv2.Rodrigues(rvec)
    h = np.eye(4)
    h[:3, :3] = R
    h[:3, 3] = np.asarray(tvec).reshape(3)
    return h


def rot_angle_deg(r: np.ndarray) -> float:
    val = np.clip((np.trace(r) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.degrees(np.arccos(val)))


def evaluate_consistency(mode: str, ds: Dict, h_calib: np.ndarray) -> Dict:
    """把标定板重建到固定坐标系，与首帧比较偏差。"""
    invariants = []
    for h_base_ee, rvec, tvec in zip(ds["robot_h_list"], ds["rvecs"], ds["tvecs"]):
        h_cam_obj = rvec_tvec_to_h_obj_cam(rvec, tvec)
        if mode == "eye_in_hand":
            h_inv = h_base_ee @ h_calib @ h_cam_obj      # 重建到基座系
        else:
            h_inv = inv_h(h_base_ee) @ h_calib @ h_cam_obj  # 重建到末端系
        invariants.append(h_inv)

    h_ref = invariants[0]
    t_err, r_err = [], []
    for h in invariants:
        d = inv_h(h_ref) @ h
        t_err.append(float(np.linalg.norm(d[:3, 3]) * 1000.0))  # m -> mm
        r_err.append(rot_angle_deg(d[:3, :3]))

    t_arr, r_arr = np.array(t_err), np.array(r_err)

    def stats(a):
        return {"mean": float(a.mean()), "rms": float(np.sqrt(np.mean(a ** 2))),
                "max": float(a.max())}

    return {"count": len(invariants),
            "translation_mm": stats(t_arr),
            "rotation_deg": stats(r_arr)}


def save_calib_json(path: str, h_calib: np.ndarray) -> None:
    with open(path, "w", encoding="utf-8") as f:
        json.dump({"mode": "eye_to_hand",
                   "R": h_calib[:3, :3].tolist(),
                   "t": h_calib[:3, 3].tolist()}, f, indent=2, ensure_ascii=False)


def load_calib_json(path: str) -> np.ndarray:
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    r = np.asarray(data["R"], dtype=float)
    t = np.asarray(data["t"], dtype=float).reshape(3)
    if r.shape != (3, 3):
        raise ValueError("JSON 里的 R 必须是 3x3")
    h = np.eye(4)
    h[:3, :3] = r
    h[:3, 3] = t
    return h


def resolve_eval_dir(arg: str, arm=None) -> str:
    if arg:
        return arg
    latest = find_latest_data_folder(os.path.join(BASE_DIR, "eye_hand_data"), arm=arm)
    if latest is None:
        raise ValueError("未找到数据目录，请传 --eval-dir")
    return latest


def main():
    args = parse_args()
    xx, yy, cell = load_checkerboard_args(args.config)
    eval_dir = resolve_eval_dir(args.eval_dir, args.arm)

    if args.calib_file:
        h_calib = load_calib_json(args.calib_file)
        calib_src = f"读取文件 {args.calib_file}"
    else:
        train_dir = args.train_dir if args.train_dir else eval_dir
        train_ds = build_dataset(train_dir, xx, yy, cell)
        h_calib = handeye_from_dataset(args.mode, train_ds)
        calib_src = f"训练集 {train_dir} 重算"
        if args.save_calib:
            save_calib_json(args.save_calib, h_calib)

    eval_ds = build_dataset(eval_dir, xx, yy, cell)
    metrics = evaluate_consistency(args.mode, eval_ds, h_calib)

    print("=" * 60)
    print(f"模式: {args.mode}")
    print(f"标定来源: {calib_src}")
    print(f"验证目录: {eval_dir}")
    print(f"总图像: {len(sorted_image_files(eval_dir))}，有效: {len(eval_ds['used_indices'])}，"
          f"失败: {len(eval_ds['skipped'])}")
    print(f"相机标定重投影 RMS: {eval_ds['rms']:.4f} 像素")
    print("\n外参矩阵 H（相机->基座）:")
    np.set_printoptions(precision=8, suppress=True)
    print(h_calib)
    m = metrics
    print("\n一致性误差（相对首帧）:")
    print(f"平移误差(mm): mean={m['translation_mm']['mean']:.3f} "
          f"rms={m['translation_mm']['rms']:.3f} max={m['translation_mm']['max']:.3f}")
    print(f"旋转误差(deg): mean={m['rotation_deg']['mean']:.3f} "
          f"rms={m['rotation_deg']['rms']:.3f} max={m['rotation_deg']['max']:.3f}")
    print("\n参考标准: 平移 RMS<=5mm、旋转<=1°；超出请检查姿态多样性/标定板平整度")
    if eval_ds["skipped"]:
        print("\n失败帧(前10条):")
        for idx, name, reason in eval_ds["skipped"][:10]:
            print(f"  index={idx}, file={name}, reason={reason}")


if __name__ == "__main__":
    main()
