# -*- coding: utf-8 -*-
"""
眼在手外 —— 把采集的机械臂 TCP 位姿（poses.txt）转换为齐次变换矩阵，取逆后存 CSV。

原理：
  机械臂控制器给出的是"末端在基座系下的位姿" H_base_ee。
  眼在手外模式需要的是"基座在末端系下的位姿"（即 H_base_ee 的逆），
  与相机观测共同构成手眼方程，解出 相机在基座系下的位姿。

欧拉角约定（与 PiPER 一致）：固定坐标系 XYZ，旋转矩阵 R = Rz * Ry * Rx。
"""
import csv
import os
import sys

import numpy as np


def euler_angles_to_rotation_matrix(rx, ry, rz):
    """固定坐标系 XYZ 欧拉角 -> 旋转矩阵（弧度）。"""
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


def pose_to_homogeneous_matrix(pose):
    """(x,y,z,rx,ry,rz) 米+弧度 -> 4x4 齐次矩阵。"""
    x, y, z, rx, ry, rz = pose
    H = np.eye(4)
    H[:3, :3] = euler_angles_to_rotation_matrix(rx, ry, rz)
    H[:3, 3] = [x, y, z]
    return H


def inverse_transformation_matrix(T):
    """4x4 齐次矩阵求逆（利用旋转矩阵正交性 R^-1 = R^T）。"""
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv = np.eye(4)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv


def save_matrices_to_csv(matrices, file_name):
    """把若干 4x4 矩阵按列拼接后写入 CSV（每行 4*N 个数）。"""
    num = len(matrices)
    combined = np.zeros((4, 4 * num))
    for i, m in enumerate(matrices):
        combined[:, 4 * i: 4 * i + 4] = m
    with open(file_name, "w", newline="") as f:
        writer = csv.writer(f)
        for row in combined:
            writer.writerow(row)


def poses2_main(filepath, out_csv="RobotToolPose.csv"):
    """
    读 poses.txt（每行 x,y,z,rx,ry,rz），转逆齐次矩阵存 CSV。
    返回转换后的矩阵列表。
    """
    with open(filepath, "r", encoding="utf-8") as f:
        lines = f.readlines()

    values = [float(v) for line in lines for v in line.split(",")]
    if len(values) % 6 != 0:
        raise ValueError(f"poses.txt 数据不是6的倍数行: 共 {len(values)} 个数")

    matrices = []
    for i in range(0, len(values), 6):
        H = pose_to_homogeneous_matrix(values[i:i + 6])
        matrices.append(inverse_transformation_matrix(H))

    save_matrices_to_csv(matrices, out_csv)
    print(f"已把 {len(matrices)} 组位姿（取逆）写入 {out_csv}")
    return matrices


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("用法: python save_poses2.py <poses.txt路径> [输出csv路径]")
        sys.exit(1)
    poses2_main(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else "RobotToolPose.csv")
