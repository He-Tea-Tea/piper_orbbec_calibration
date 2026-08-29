# -*- coding: utf-8 -*-
"""通用工具函数。"""
import os
import re
import tkinter as tk
from datetime import datetime
from tkinter import messagebox


def create_folder_with_date(prefix="eye_hand_data", arm=None):
    """创建按日期命名的数据目录：eye_hand_data/dataYYYYMMDD[NN]，避免重名。
    arm 不为 None 时目录带臂名：eye_hand_data/dataYYYYMMDD_arm0[NN]。
    """
    today = datetime.now().strftime("%Y%m%d")
    if arm is not None:
        base_folder = os.path.join(prefix, f"data{today}_{arm}")
    else:
        base_folder = os.path.join(prefix, f"data{today}")
    index = 0
    folder = base_folder
    while os.path.exists(folder):
        index += 1
        folder = f"{base_folder}{str(index).zfill(2)}"
    os.makedirs(folder)
    return folder


def find_latest_data_folder(prefix="eye_hand_data", arm=None):
    """返回 eye_hand_data 下最新数据目录的完整路径；没有则返回 None。

    arm 为 None 时匹配所有目录（含旧的无臂名格式）；
    指定 arm（如 "arm0"）时只匹配该臂的目录（dataYYYYMMDD_arm0[NN]）。
    """
    if not os.path.isdir(prefix):
        return None
    if arm is not None:
        pattern = re.compile(rf"^data(\d{{8}})_{arm}(\d*)$")
    else:
        pattern = re.compile(r"^data(\d{8})(?:_arm\d+)?(\d*)$")
    folders = [
        f for f in os.listdir(prefix)
        if os.path.isdir(os.path.join(prefix, f)) and pattern.match(f)
    ]
    if not folders:
        return None
    folders.sort(
        key=lambda x: (pattern.match(x).group(1),
                       int(pattern.match(x).group(2) or 0),
                       x),
        reverse=True,
    )
    return os.path.join(prefix, folders[0])


def popup_message(title, message):
    """弹出提示框。"""
    root = tk.Tk()
    root.withdraw()
    root.attributes("-topmost", True)
    messagebox.showinfo(title, message)
    root.destroy()
