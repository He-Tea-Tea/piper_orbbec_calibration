# 松灵 PiPER + 奥比中光 335Le 手眼标定（眼在手外）

针对 **松灵 PiPER 6 轴机械臂 + 奥比中光 Gemini 335Le 深度相机** 重写的完整标定项目。

- 标定模式：**眼在手外**（相机固定不动，标定板固定在机械臂末端）
- 算法：OpenCV —— 棋盘格角点检测 + 张正友相机标定（`calibrateCamera`）+ Tsai 手眼标定（`calibrateHandEye`）
- 输出：相机在机械臂基座坐标系下的位姿（旋转矩阵 R + 平移向量 t + 四元数）

## 目录结构

```
├── collect_data.py        # 数据采集（相机画面 + 机械臂位姿同步保存）
├── save_poses2.py         # 位姿 -> 逆齐次矩阵 CSV
├── compute_to_hand.py     # 手眼标定求解（输出并保存 calib_arm0.json / calib_arm1.json）
├── verify_calibration.py  # 精度验证（一致性误差）
├── test_hardware.py       # 硬件自检（相机/机械臂分开测）
├── config.yaml            # 棋盘格参数
├── requirements.txt
└── libs/                  # 工具库（日志、日期目录等）
```

## 使用步骤

### 1. 安装环境

```powershell
cd piper_orbbec_calibration
python3 -m pip install -r requirements.txt
```

```bash
cd ~/桌面
git clone https://github.com/agilexrobotics/pyAgxArm.git
cd pyAgxArm
python3 -m pip install .
```
注意：
- 奥比 335Le 必须用 v2 SDK：PyPI 包名是 `pyorbbecsdk2`，但代码里 `import` 名是 `pyorbbecsdk`；
- v1（`pyorbbecsdk`）与 v2（`pyorbbecsdk2`）**不能共存**，安装前先卸载 v1。

### 1.5 Linux（工控机）环境准备

代码会自动识别平台：**Linux 下走 `socketcan`，臂通道固定为 `can0`/`can1`**（Windows 走 `agx_cando` 的 `"0"`/`"1"`），无需改代码。以 Ubuntu 为例：

**① 安装依赖、加权限**

```bash
sudo apt install -y can-utils python3-pip
# 当前用户加入 dialout 组（USB-CAN 设备节点权限），注销重登后生效
sudo usermod -aG dialout $USER
```

**② 启用 CAN 接口（每次开机 / 拔插 USB-CAN 后执行）**

```bash
ip link show | grep -i can     # 确认接口名（多路 CAN 卡可能是 can0/can1）
sudo ip link set can0 up type can bitrate 1000000   # 与机械臂波特率一致
sudo ip link set can1 up type can bitrate 1000000
# 验证：另开终端跑 candump can0，手动动一下机械臂，应能看到 CAN 帧
```

- 位姿乱码 → 试 `sudo ip link set can0 down` 后改用 `bitrate 500000` 重新 up；
- 想开机自动启用，把 `ip link` 命令写进 systemd service 或 `/etc/rc.local`；
- 接口名不是 `can0`/`can1` 时，改 `collect_data.py` 顶部 `_DEFAULT_CHANNELS` 和 `test_hardware.py` 的 `arm_channels()`。

**③ 相机**：奥比 335Le 在 Linux 用 v2 SDK（安装见上），USB3 设备权限问题按官方文档处理。

### 2. 硬件自检（分开测，先确认两个设备都能通）

```powershell
python3 test_hardware.py camera   # 弹窗出画面 = 相机 OK
python3 test_hardware.py arm      # 完整诊断 arm0：通信/固件/位姿逐项输出
python3 test_hardware.py arm arm1 # 有第二条臂时按臂诊断（arm0 / arm1 / ...）
```

### 3. 修改棋盘格参数（config.yaml）

- XX/YY 是**角点数**（黑白格交叉点数 = 格子数 - 1）
- L 是**一格实际边长**（米），建议量 10 格总长 ÷ 10

### 4. 物理布置（眼在手外）

- 相机固定（俯视或稍倾斜），**全程不能动**
- 棋盘格贴死在机械臂末端
- 示教器把工件坐标系切到 **Base**
- 两条臂共享同一相机时：**同一时间画面里只留一条臂的标定板**，另一条臂让到视野外（详见第 10 节）

### 5. 采集数据

```powershell
python3 collect_data.py            # 默认 arm0
python3 collect_data.py --arm arm1 # 采集第二条臂
```

- 动机械臂，让棋盘格完整、清晰、尽量大地出现在画面里
- **画面会实时叠加棋盘格检测结果**：检测到 → 绿色框出角点网格并显示 `CHESSBOARD OK (9x6)`；未检测到 → 红色提示 `NO CHESSBOARD`。保存时终端也会提示该帧棋盘格是否有效
- **姿态要多变**：翻来翻去（绕不同轴转、靠近远离、左右倾斜）
- 机械臂**停稳后**按 `s` 保存（**终端里直接按 `s` 回车即可**，无需点击视频窗口；`q` 回车退出），建议 15~20 张
- 数据自动存到 `eye_hand_data\dataYYYYMMDD_arm0\`、`dataYYYYMMDD_arm1\`（按臂分目录，图像 + poses.txt 一一对应）

### 6. 求解标定

```powershell
python3 compute_to_hand.py            # 自动找最新数据目录（目录名带臂名则自动识别）
python3 compute_to_hand.py --arm arm1 # 显式指定臂，找该臂最新目录
python3 compute_to_hand.py eye_hand_data\data20240101_arm0 --arm arm0 # 指定目录+臂
```

自动找到对应数据目录并求解，终端打印 R / t / 四元数，结果按臂保存到 `calib_arm0.json` / `calib_arm1.json`（JSON 内含 `"arm"` 字段，供后续使用区分）。

### 7. 验证精度（建议再做一组独立验证数据）

```powershell
python3 verify_calibration.py --mode eye_to_hand --calib-file calib_arm0.json
# 或：用训练集重算、验证集评估（多臂时加 --arm 过滤目录）
python3 verify_calibration.py --mode eye_to_hand --arm arm0 `
    --train-dir eye_hand_data\dataXXX_arm0 --eval-dir eye_hand_data\dataYYY_arm0
```

参考标准：平移误差 RMS ≤ 5 mm、旋转误差 ≤ 1°。

### 8. 标定板到机械臂末端 / 夹爪的距离需要算吗？（原理）

**标定阶段不需要，也不应该输入任何"标定板中心到末端/夹爪"的偏移。**

眼在手外模式下，每帧满足：

```
H_base_tcp · H_tcp_board = X · H_cam_board
```

- `H_base_tcp`：机械臂上报的 TCP 位姿（poses.txt，每帧不同）
- `H_tcp_board`：标定板相对 TCP 的固定位姿 —— 你关心的"板中心到末端/夹爪的距离和朝向"全在这个常数矩阵里
- `X = H_base_cam`：要解的相机在基座系下的位姿
- `H_cam_board`：相机测到的标定板位姿

取两次不同位姿做相对运动后，`H_tcp_board` 会在方程两侧**完全抵消**：

```
(H_base_tcp2 · H_base_tcp1⁻¹) · X = X · (H_cam_board2 · H_cam_board1⁻¹)
```

所以只要标定板与末端**刚性固定**（`H_tcp_board` 恒定），板子装在末端上多远、什么朝向，都不影响标定结果，无需测量、无需输入。`compute_to_hand.py` 与 `verify_calibration.py` 正是按此实现，没有任何标定板偏移参数——这是正确的。

⚠️ 前提：采集全程板子不能松动；若板子固定在夹爪上，采集期间**夹爪必须锁住**（开合会改变 `H_tcp_board`）；换夹爪或重贴板子后需重新标定。

### 9. 标定完成后的实际使用（识别 → 抓取）

手眼标定的产出是 `calib_arm0.json` / `calib_arm1.json`（每条臂一份）：相机在**对应那条臂**的基座系下的外参 `R`、`t` 与相机内参。它的作用是**把相机看到的任何点/物体位姿变换到机械臂基座系**，机械臂才能照着运动。使用前还需要两个**应用层偏移**（不属于标定，需单独设置）：

| 偏移 | 是什么 | 怎么设置 |
|---|---|---|
| 夹爪工具坐标（TCP） | 夹爪抓取中心相对末端法兰/原 TCP 的位姿 | 示教器做工具坐标标定（四点法），或在 `collect_data.py` 的 `TCP_OFFSET` 里配置；换夹爪要重新标定 |
| 物体抓取点偏移 | 抓取点在**物体坐标系**下的位置（如球心、重心） | 在识别代码里定义，如 `grasp_local = [0, 0, r]` |

#### 使用流程（眼在手外）

```
相机识别物体 → 物体在相机系下的位姿 T_cam_obj
      ↓  X = [R | t]（calib_arm0.json / calib_arm1.json）
物体在基座系下的位姿 T_base_obj = X · T_cam_obj
      ↓  叠加上物体系下的抓取点偏移 + 预抓取高度
抓取目标位姿（基座系）→ 转为机械臂位姿格式下发 → 夹爪闭合
```

点变换公式（相机系点 `p_cam` → 基座系点 `p_base`）：

```
p_base = R · p_cam + t
```

#### 示例代码（OpenCV + pyAgxArm）

```python
import json
import cv2
import numpy as np

# 1) 载入标定结果（用哪条臂就加载哪个文件）
with open("calib_arm0.json", encoding="utf-8") as f:  # 或 calib_arm1.json
    calib = json.load(f)
R_cam2base = np.array(calib["R"])                # 3x3，相机 -> 基座
t_cam2base = np.array(calib["t"]).reshape(3)     # 平移，米
mtx = np.array(calib["intrinsics"]["mtx"])
dist = np.array(calib["intrinsics"]["dist"])

def cam2base(p_cam):
    return R_cam2base @ p_cam + t_cam2base

# 2) 识别物体：求物体在相机系下的位姿（示例用 solvePnP）
#    object_points: 物体上已知点的 3D 坐标（物体坐标系）
#    image_points : 这些点在图像上的像素坐标
ret, rvec, tvec = cv2.solvePnP(object_points, image_points, mtx, dist)
R_cam2obj, _ = cv2.Rodrigues(rvec)
t_cam2obj = tvec.reshape(3)

# 3) 物体位姿转到基座系
R_base2obj = R_cam2base @ R_cam2obj
t_base2obj = R_cam2base @ t_cam2obj + t_cam2base

# 4) 抓取点 = 物体位姿 + 物体系下的抓取点偏移（如球心）
grasp_local = np.array([0.0, 0.0, 0.0])          # 按实际物体修改
grasp_base = R_base2obj @ grasp_local + t_base2obj

# 5) 姿态转机械臂格式（固定坐标系 XYZ，R = Rz*Ry*Rx，与 save_poses2.py 一致）
def rot_to_euler_xyz(R):
    sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
    if sy > 1e-6:
        rx = np.arctan2(R[2, 1], R[2, 2])
        ry = np.arctan2(-R[2, 0], sy)
        rz = np.arctan2(R[1, 0], R[0, 0])
    else:
        rx = np.arctan2(-R[1, 2], R[1, 1])
        ry = np.arctan2(-R[2, 0], sy)
        rz = 0.0
    return np.array([rx, ry, rz])

rx, ry, rz = rot_to_euler_xyz(R_base2obj)

# 6) 下发：先到预抓取点（如沿当前朝向后退 0.15 m），再直线进给抓取
pregrasp_base = grasp_base - R_base2obj[:, 2] * 0.15  # 按夹爪接近方向调整
robot.move_to([*pregrasp_base, rx, ry, rz])           # 预抓取
robot.move_to([*grasp_base, rx, ry, rz])              # 抓取点
robot.gripper_close()
```

#### 使用要点

- **TCP 定义全程一致**：采集、验证、应用三段用同一个 TCP 定义（都默认 flange 或都设相同的 `TCP_OFFSET`），否则会整体偏一个固定量。
- **先预抓取、再直线进给**：避免直接斜插撞到物体；接近方向沿 `R_base2obj` 的某一轴（取决于夹爪安装朝向）。
- **抓取不准且误差恒定**：先怀疑夹爪工具坐标没标好（四点法），再怀疑物体抓取点偏移设错，与手眼标定无关。
- 深度相机可直接在深度图上取物体表面点 `p_cam = (u, v, depth)`，再用内参逆投影成相机系 3D 点，省去 `solvePnP`。

### 10. 两条机械臂怎么区分（同一相机，两臂轮流标）

标定结果是"相机在**某一条臂**的基座系下的位姿"，**一对一**。两条臂 = 两个基座系 = 两份标定记录，全程靠**臂 ID**（`arm0` / `arm1`）区分，不能混。

**① 连接层（区分硬件）**：每条臂固定一个 CAN 接口+通道，`collect_data.py` 顶部的 `ARMS` 表里配置：

| 臂 ID | Windows（agx_cando） | Linux（socketcan） |
|---|---|---|
| `arm0` | channel `"0"` | channel `"can0"` |
| `arm1` | channel `"1"` | channel `"can1"` |

- Linux 下先把 `can0`/`can1` 用 `ip link` 启用（见第 1.5 节），接口名不同时改 `collect_data.py` 的 `_DEFAULT_CHANNELS`；
- Windows 下多张卡时在设备管理器里确认编号，把 `ARMS` 改成实际值。

**② 采集层（区分数据）**：每条臂各采一组，目录自动按臂命名：

```powershell
python3 collect_data.py --arm arm0   # 只动臂0，画面里只有它的标定板
python3 collect_data.py --arm arm1   # 只动臂1，臂0让出画面
```

采集时**同一时间画面里只留一条臂的标定板**，另一条臂收到视野外或转到背对相机——混入两个标定板会导致该帧角点检测/位姿错乱。

**③ 标定层（区分结果）**：每臂各跑一次，输出按臂命名：

```powershell
python3 compute_to_hand.py --arm arm0   # -> calib_arm0.json
python3 compute_to_hand.py --arm arm1   # -> calib_arm1.json
```

两个结果的 `R` 应基本相同，`t` 的差值等于两条臂基座的相对位置——可用这个做自检。

**④ 使用层（区分动作）**：抓取前指定臂 ID，加载对应 json，把相机坐标转到**那条臂**的基座系：

```python
def load_calib(arm):
    with open(f"calib_{arm}.json", encoding="utf-8") as f:
        c = json.load(f)
    assert c["arm"] == arm            # 防止拿错文件
    return np.array(c["R"]), np.array(c["t"]).reshape(3)

# 例：臂1去抓
R, t = load_calib("arm1")
p_base = R @ p_cam + t                # 相机点 -> 臂1基座系
# 然后连臂1（collect_data.py 里的 ARMS["arm1"]["can_channel"]）下发运动
```

**⑤ 换臂重标**：换夹爪、重贴标定板后，只重标受影响的那条臂即可；相机动了则两条臂都要重标（因为 `R/t` 都以相机为参照）。

### 11. 位姿无法获取 / 数值乱排查

先跑诊断，再对症处理：

```bash
python3 test_hardware.py arm arm0   # 或 arm1
```

诊断输出会逐项显示 `is_ok` / `get_fps` / `comm_error` / 固件版本与推荐 profile / `arm_status` / `joint_angles` / `flange_pose` / `tcp_pose`。**Linux 下连接前还会自动检查 CAN 接口状态**（未启用/不存在/波特率不一致会直接报错并给出修复命令）。按下面的表定位：

| 诊断现象 | 根因 | 处理 |
|---|---|---|
| 连接时报"接口 canX 不存在" | 第二块 USB-CAN 没识别/没插好 | `dmesg \| grep -i can`；跑 `pyAgxArm/scripts/ubuntu/find_all_can_port.sh` 确认两块卡都在；换 USB 口重插 |
| 连接时报"接口 canX 未启用（DOWN）" | 该 CAN 接口没 up | `sudo ip link set canX up type can bitrate 1000000`（X=0/1），重启后需重配（可写进 systemd/rc.local） |
| 连接时报"波特率不一致" | 接口波特率不是 1 Mbit/s | `sudo ip link set canX down` 后重新 up 并指定 `bitrate 1000000` |
| `fps = 0 Hz`、`is_ok = False`、`comm_error = None`、接口检查通过 | **该路 CAN 上没有机械臂数据**：臂没上电 / 线没接好 / 接线端子松 | 用 `candump canX` 看是否有帧；确认臂1 已上电、CAN 线接的是正确模块；换臂0 的线交叉验证 |
| `fps = 0 Hz`、`is_ok = False`、`comm_error` 有值 | 通信层报错 | 查看 `comm_error` 具体内容，多数是接口/权限问题（dialout 组） |
| `固件版本读取失败（返回 None）` 但 fps > 0 | 固件 profile 与机械臂固件不匹配，反馈帧解析不出来 | 代码已自动适配：连接时读 `get_firmware()` 并用 `resolve_firmware_profile()` 切到正确 profile（`default`/`v183`/`v188`/`v189`），日志会打印 `固件 profile=...`；若仍失败，在 `collect_data.py` 的 `ARMS` 里把该臂 `fw_profile` 改为诊断输出的"推荐 profile" |
| `arm_status = None` 且一切无数据 | 机械臂未上电 / 急停 / 未使能 | 示教器确认上电、无急停、无报错 |
| `tcp_pose` 能读但 `z` 恒为 0 或数值固定不变 | 端位姿反馈帧（CAN 0x2A2/0x2A3/0x2A4）部分缺失 | 确认机械臂在动、固件与 profile 匹配；重启机械臂电源后重试 |
| `flange_pose` 正常但 `tcp_pose = None` | SDK 太旧，基类没有 `get_tcp_pose` | 用仓库内 `pyAgxArm-master` 安装新版（`pip install .`），或改用 `get_flange_pose()` + `set_tcp_offset` 的组合 |

说明（SDK 源码依据）：`get_tcp_pose()` 是在 `get_flange_pose()` 基础上套用 TCP 偏移算出来的；`get_flange_pose()` 由 CAN 反馈帧 `0x2A2`（X/Y）、`0x2A3`（Z/RX）、`0x2A4`（RY/RZ）拼装，三帧都收不到才返回 None。所以"位姿无法获取"最底层的原因基本都在**通信未建立**或**固件 profile 不匹配**这两类，按上表即可定位。

## 常见问题

| 现象 | 原因 | 处理 |
|---|---|---|
| 机械臂连不上 | USB-CAN 驱动/接线/波特率/CAN 未启用 | Windows 查设备管理器；Linux 检查 `ip link` 是否 up、dialout 权限（见第 1.5 节）；位姿乱码时试 bitrate=500000 |
| 位姿数值乱 / 位姿获取不到 | 固件 profile 不匹配 / CAN 未通 / 机械臂未就绪 | 先跑 `test_hardware.py arm armX` 看诊断，按第 11 节表处理；固件不匹配时连接会自动切换 profile |
| 相机连不上 | v1/v2 SDK 冲突、USB 松动 | 卸载 pyorbbecsdk，只留 pyorbbecsdk2 |
| 角点检测失败多 | 板子太小/太远/反光 | 相机放近（0.5~0.7m）、换大板、避反光 |
| 标定误差大 | 姿态单一 / 保存时机械臂在动 / L 填错 | 姿态要多变、停稳再按 s、重量 L |
| 标定结果准但抓取偏一个固定量 | 夹爪工具坐标 / 物体抓取点偏移没设对 | 示教器做 TCP 四点法标定；核对应用代码里的 grasp_local（见第 9 节） |
| 连的臂不对 / 结果像另一条臂 | 自检、采集、标定没指定同一臂 ID | 全程用同一 `--arm`/`armN` 参数；核对 `ARMS` 里的 CAN channel（见第 10 节） |
| 双臂标定互相干扰、角点检测乱 | 画面里同时出现两块标定板 | 同一时间只留一条臂的标定板在画面里（见第 10 节） |
