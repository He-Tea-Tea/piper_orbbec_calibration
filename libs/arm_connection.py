# -*- coding: utf-8 -*-
"""
pyAgxArm 连接与固件版本自动适配（PiPER 系列）。

背景（见 pyAgxArm 文档 docs/piper/piper_api.md 与 firmware_reference.md）：
  - PiPER 固件版本决定 SDK 驱动 profile：PiperFW.DEFAULT / V183 / V188 / V189；
  - profile 与固件不匹配时，反馈帧解析会出错（如 get_arm_status 用错协议、MIT
    参数不对），表现为位姿获取异常、数值乱等；
  - 官方推荐流程：先按任意 profile 连接 -> get_firmware() 读软件版本 ->
    resolve_firmware_profile() 得到正确 profile -> 必要时重连。

本模块提供：
  - connect_arm(): 完整流程（探测固件 + 自动切换正确 profile + 重连）
  - diagnose_arm(): 打印通信/固件/位姿逐项诊断，供 test_hardware.py 使用
"""
import logging
import time

logger = logging.getLogger(__name__)

# 等待机械臂反馈就绪的最长时间
OK_WAIT_TIMEOUT = 5.0
OK_POLL_INTERVAL = 0.1
# get_firmware() 请求超时
FIRMWARE_TIMEOUT = 2.0


def build_config(arm_cfg, fw_profile="default"):
    """生成 create_agx_arm_config 配置（socketcan / agx_cando 自动按平台）。"""
    import platform

    from pyAgxArm import (
        ArmModel,
        create_agx_arm_config,
    )

    if platform.system() == "Windows":
        interface = "agx_cando"
    else:
        interface = "socketcan"

    return create_agx_arm_config(
        robot=ArmModel.PIPER,
        firmeware_version=fw_profile,
        interface=interface,
        channel=arm_cfg["can_channel"],
        bitrate=arm_cfg.get("can_bitrate", 1000000),
    )


def create_and_connect(arm_cfg, fw_profile="default"):
    """用指定 profile 创建驱动并连接，返回 robot 实例。"""
    from pyAgxArm import AgxArmFactory

    cfg = build_config(arm_cfg, fw_profile)
    robot = AgxArmFactory.create_arm(cfg)
    robot.connect()
    return robot


def wait_ok(robot, timeout=OK_WAIT_TIMEOUT, poll_interval=OK_POLL_INTERVAL):
    """等待机械臂数据接收恢复正常（is_ok）。返回 (True, 用时) 或 (False, 错误说明)。"""
    deadline = time.monotonic() + timeout
    last_fps = 0.0
    while time.monotonic() < deadline:
        try:
            if robot.is_ok():
                return True, 0.0
            last_fps = robot.get_fps() if hasattr(robot, "get_fps") else 0.0
        except Exception:
            pass
        time.sleep(poll_interval)
    return False, f"等待数据接收超时（is_ok=False，当前 fps={last_fps:.1f} Hz）"


def detect_firmware_profile(robot, configured_profile="default"):
    """
    读取机械臂固件版本并解析出推荐 profile。

    返回 (recommended_profile, firmware_info)：
      - recommended_profile: 推荐 profile（字符串）；读不到/解析失败时为
        configured_profile（沿用当前配置）；
      - firmware_info: get_firmware() 返回的 dict 或 None。
    """
    recommended = configured_profile
    firmware = None
    try:
        firmware = robot.get_firmware(timeout=FIRMWARE_TIMEOUT)
    except Exception as exc:
        logger.warning(f"读取固件版本失败: {exc}")
        return recommended, None

    if firmware is None:
        logger.warning("get_firmware() 返回 None，无法确认固件版本，沿用当前 profile")
        return recommended, None

    software_version = firmware.get("software_version")
    logger.info(f"机械臂固件: hardware={firmware.get('hardware_version')}, "
                f"software={software_version}")
    if not software_version:
        return recommended, firmware

    try:
        from pyAgxArm import resolve_firmware_profile
        recommended = resolve_firmware_profile("piper", software_version)
    except Exception as exc:
        logger.warning(f"resolve_firmware_profile({software_version!r}) 失败: {exc}")
        recommended = configured_profile

    return recommended, firmware


def connect_arm(arm_cfg, fw_profile="default"):
    """
    完整连接流程（固件自动适配）：
      1. 用配置的 profile 连接；
      2. 读 get_firmware()，resolve_firmware_profile() 得到推荐 profile；
      3. 推荐 profile 与配置不同时，断开并用推荐 profile 重连；
      4. 等待数据接收正常。

    返回 (robot, used_profile, firmware_info)。
    连接失败抛出 RuntimeError。
    """
    robot = create_and_connect(arm_cfg, fw_profile)
    ok, msg = wait_ok(robot, timeout=3.0)
    if not ok:
        logger.warning(f"连接后数据未就绪（{msg}），继续尝试读取固件…")

    recommended, firmware = detect_firmware_profile(robot, fw_profile)
    if recommended != fw_profile:
        logger.info(
            f"固件版本需要 profile {recommended}（当前配置 {fw_profile}），重连…"
        )
        try:
            robot.disconnect()
        except Exception:
            pass
        robot = create_and_connect(arm_cfg, recommended)
        ok, msg = wait_ok(robot)
        if not ok:
            raise RuntimeError(msg)
    elif not ok:
        raise RuntimeError(msg)

    return robot, recommended, firmware


def diagnose_arm(robot, arm_id="arm0"):
    """打印通信 / 固件 / 位姿逐项诊断。返回 True 表示位姿可正常获取。"""
    print("=" * 60)
    print(f"机械臂 [{arm_id}] 诊断")
    print("=" * 60)

    # 1. 通信层
    try:
        print(f"is_connected = {robot.is_connected()}")
        print(f"is_ok        = {robot.is_ok()}  (数据接收是否正常)")
        print(f"get_fps     = {robot.get_fps():.1f} Hz  (反馈帧接收频率)")
        err = robot.get_comm_error() if hasattr(robot, "get_comm_error") else None
        if err is not None:
            print(f"comm_error   = {err}")
        else:
            print("comm_error   = None")
    except Exception as exc:
        print(f"通信层查询异常: {exc}")

    # 2. 固件版本
    firmware = None
    try:
        firmware = robot.get_firmware(timeout=FIRMWARE_TIMEOUT)
    except Exception as exc:
        print(f"读取固件版本异常: {exc}")
    if firmware is not None:
        software_version = firmware.get("software_version", "?")
        print(f"固件 software_version = {software_version}")
        try:
            from pyAgxArm import resolve_firmware_profile
            print(f"推荐 profile = {resolve_firmware_profile('piper', software_version)}")
        except Exception as exc:
            print(f"推荐 profile 解析失败: {exc}")
    else:
        print("固件版本读取失败（返回 None）——通信未建立或固件 profile 不匹配")

    # 3. 逐项读取反馈
    results = {}
    try:
        arm_status = robot.get_arm_status()
        if arm_status is not None:
            results["arm_status"] = arm_status.msg
            print(f"arm_status = {arm_status.msg}")
        else:
            print("arm_status = None")
    except Exception as exc:
        print(f"arm_status 读取异常: {exc}")

    for name, fn in (("joint_angles", robot.get_joint_angles),
                     ("flange_pose", robot.get_flange_pose),
                     ("tcp_pose", robot.get_tcp_pose)):
        try:
            ret = fn()
            if ret is not None:
                results[name] = ret.msg
                print(f"{name:12s} = {ret.msg}")
            else:
                results[name] = None
                print(f"{name:12s} = None")
        except Exception as exc:
            print(f"{name:12s} 读取异常: {exc}")

    # 4. 结论
    ok = results.get("tcp_pose") is not None and results.get("flange_pose") is not None
    if ok:
        print("\n结论：位姿获取正常。")
    else:
        print("\n结论：位姿无法获取。按顺序排查：")
        print("  1) CAN 通信：sudo ip link set can0 up type can bitrate 1000000")
        print("     再用 candump can0 确认有帧；fps>0 才说明通信正常")
        print("  2) 固件 profile：上表'推荐 profile'与当前配置不一致时，")
        print("     在 collect_data.py 的 ARMS 里把 fw_profile 改成推荐值（或保持 'default' 让代码自动切换）")
        print("  3) 机械臂状态：确认已上电、未急停、示教器无报错（arm_status 非 None 且无 error）")
    return ok
