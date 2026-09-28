# M400 控制测试｜妙算 3

## 项目简介

基于 DJI Payload SDK 3.16.0 的 Matrice 400＋妙算 3 实验项目：在 DJI Pilot 中记录飞行遥测、查看状态、测试位置与航向控制，并通过独立网线与树莓派 4B 交换文本及 Pilot 控件事件。本仓库是基于 [DJI Payload-SDK](https://github.com/dji-sdk/Payload-SDK) 的开发 fork，不是官方示例原仓库。

当前应用版本为 **1.4**（DPK `01.04.00.00`）；改代码时默认不自动升版本。开发命令、部署路径、网络初次配置和踩坑见 [AGENTS.md](AGENTS.md)。

## 配置

- **硬件**：M400＋妙算 3（Ubuntu 20.04／ARM64／GCC 9.4）＋树莓派 4B。
- **应用**：`m400-control-test`，Payload SDK-妙算 3，PSDK 3.16.0。
- **源码**：`D:\M400-Manifold3-Control`；飞控与遥测见 [m400_control_service.c](samples/sample_c/module_sample/m400_control/m400_control_service.c)，Pilot 界面见 [中文 Widget JSON](samples/sample_c/platform/linux/manifold3/app_json/widget/cn_big_screen/widget_config.json)。
- **SSH**：Pi4 `ssh pi4@192.168.124.14`；妙算 E3 `ssh dji@192.168.42.140`；经 Pi4 跳板 `ssh -J pi4@192.168.124.14 dji@10.88.77.54`。
- **机载网线**：Pi4 `eth0` 为 `10.88.77.1`，妙算 `eth0` 为 `10.88.77.54`；应用文本／控件通道为 TCP `14560`。

`192.168.124.14`、`10.88.77.54` 为此前 DHCP 地址，变更时需更新命令。妙算默认用户名和密码均为 `dji`；本机也已配置 SSH 密钥。Pi4 以 Wi-Fi 上网、以太网直连妙算扩展坞，使用 NetworkManager `shared` 模式为妙算提供地址、DNS 和网络出口；通信不依赖 M400 机载局域网。

App ID、App Key 和高级 License 仅放在本机 `dji_sdk_app_info.h`、`app.json`，**不提交**。新克隆时从 [应用头文件模板](samples/sample_c++/platform/linux/manifold3/application/dji_sdk_app_info.example.h) 和 [DPK 模板](samples/sample_c/platform/linux/manifold3/app_json/app.example.json) 创建本地配置。本仓库保留上游 [LICENSE.txt](LICENSE.txt) 和 [EULA.txt](EULA.txt)。

### 在电脑上看妙算桌面

电脑连接 M400 E3 调参网络时，直接打开 `http://192.168.42.140:6080/vnc.html`。经 Pi4 跳板时，在 Windows PowerShell 执行以下命令，保持终端开着，再用浏览器打开 `http://127.0.0.1:6081/vnc.html`；结束时按 `Ctrl+C`。

```powershell
ssh -N -L 6081:127.0.0.1:6080 -J pi4@192.168.124.14 dji@10.88.77.54
```

`-J` 让 SSH 经 Pi4 到妙算，`-L` 将电脑的 `6081` 转发到妙算的 VNC `6080`；Pi4 只转发，不运行桌面或 RViz2。VNC 走 Wi-Fi／以太网，**不走 DJI 遥控器图传**，飞远后 Wi-Fi 断开就无法观看。浏览器 VNC 对高帧率 3D 可能卡顿；妙算负责建图、电脑本地 RViz2 仅负责渲染会更合适。[DJI 妙算 VNC 说明](https://developer.dji.com/doc/payload-sdk-tutorial/cn/manifold-quick-start/manifold-development.html)

## 主要功能

- 50 Hz 飞行状态 CSV：记录融合位置、RTK、速度、姿态等；Pilot 可启停，并在 Payload 设置页浏览最近文件日期和时间。已安装应用的数据位于 `/open_app/m400-control-test/data/logs/`。
- Pilot 控件：主界面有录制启停、红色紧急悬停和浮窗入口；Payload 设置页有 RTK、经纬度、命令状态、六项航线参数、相对两点航线执行按钮及两个固定目标航线按钮。浮窗上行显示最近 Pi4 文本，下行显示最近 CMD 响应。
- 相对两点航线：已起飞时输入前后／左右／上下距离（各 −10～10 m，正数分别为前、左、上）、航点速度（1～10 m/s）、安全起飞高度和终点绝对航向（−180～180°）。按“执行”后从当前坐标与机头方向计算终点，在内存中生成 WPML KMZ，调用 Waypoint 3 上传／启动。航段使用“直线到点停”、航向沿航线；终点执行 `rotateYaw`，结束 `noAction`（不返航）。`waypointSpeed` 与 `autoFlightSpeed` 使用输入的航点速度；到首航点的 `globalTransitionalSpeed` 也取相同值，**不是垂直起飞速度**。本按钮只允许已起飞时运行，安全起飞高度此时不生效。红色按钮会停止本应用启动的 Waypoint 3 任务，停止后的实际悬停需在 M400 上验证。
- 固定目标航线：两个按钮都飞向 `22.602549522388877, 113.99172104792952`、起飞点以上 20 m，分别以 2 / 10 m/s 为航段目标速度；终点绝对航向取按键时指向目标的方位角。沿用原来的 100 m 距离限制和 10 秒内二次按键确认，但飞行实现改为 Waypoint 3，而非旧 `speedFactor`。飞机已在空中时，安全起飞高度不会自动强制它爬升。
- 旧 Joystick 相对运动与分步定点 API 仍保留供代码调用，但不再显示为 Pilot 运动按钮；遥控器暂停会收回其控制权。Waypoint 3 的 M400 导入、终点旋转和任务完成状态还没有实飞验证。
- Pi4 双向通信：Pi4 文本显示在 Pilot 浮窗；Pilot 按钮、录制开关和 CSV 选择事件推送给 Pi4。文本不会被解释为飞控指令。Pi4 断电/断线不使飞控等待；断线期间的事件不补发，浮窗可能保留断线前最后一条 Pi4 文本。
- MATLAB 多高度扫描航线：[普通扫描入口](tools/main_scan.m) 和 [定距停转入口](tools/main_scan_rotate.m)。后者设置 `rotationSpacingM`，在每层沿实际航程定距插入停转航点；2D／3D 预览以紫色星标标出，KMZ 在各点顺序执行四次 90° `rotateYaw`，无负载动作。`rotateYaw` 在 WPML 中是绝对航向，文档未给该动作的角速度／角加速度设置，支持机型表也未明确列 M400；导入、完整旋转及平顺性仍待实测。

## API 介绍

### 飞行控制与遥测（妙算 C）

公开声明见 [m400_control_service.h](samples/sample_c/module_sample/m400_control/m400_control_service.h)。

| API | 作用 |
| --- | --- |
| `M400Control_StartRecording()` / `M400Control_StopRecording()` | 启停 CSV 录制 |
| `M400Control_MoveRelative(direction, distanceM)` / `M400Control_MoveVertical(deltaM)` | 相对水平 / 垂直位移 |
| `M400Control_YawRelative(deltaDegree)` / `M400Control_EmergencyHover()` | 限速相对旋转 / 停止 Waypoint 3 任务，或中断旧 Joystick 运动并短时发送悬停指令 |
| `M400Control_GotoCoordinate(target)` / `M400Control_GotoCoordinateWithLimit(target, limitM)` | 先航向、再高度、最后水平的分步定点；后者限制每轴水平位置偏移指令 |
| `M400Control_RunWaypointRelative(move)` | 从当前位姿生成并启动两点 Waypoint 3 任务；返回启动结果，任务完成需看回调／Pilot |
| `M400Control_RunWaypointCoordinate(target)` | 从当前位姿生成并启动目标经纬度／相对起飞点高度的两点 Waypoint 3 任务；限当前点 100 m 内 |
| `DjiFcSubscription_*` / `DjiFlightController_*` / `DjiWidget_*` | 底层 DJI 遥测订阅、控制权与 joystick 指令、Pilot 控件注册 |

`T_M400GotoTarget` 包含 WGS-84 经纬度、起飞点以上高度、航向、容差和超时。旧 Joystick API 是同步调用；两个 `M400Control_RunWaypoint*` API 同步上传／启动，但 Waypoint 3 飞行本身异步进行。旧 `speedFactor`/`limitM` 是**每轴水平位置偏移指令的米数上限**，不是 m/s 或飞控内部加速度上限；固定目标按钮的新 2/10 数字才是航点间目标速度（m/s）。当前两点航线按已起飞场景设计。

水平位置模式旧 API 只追踪**终点误差**，不是直线航迹跟踪；侧风下偏离后再横向修正属预期现象。新两点航线选择 WPML 的“直线飞行、到点停”类型，但不承诺实际轨迹零横向误差。若起终点高度不同，WPML 只规定航点高度，没有“两点间必先垂直/必先水平”的顺序；需要先升后平飞时，应额外插入同经纬度、目标高度的中间航点。[DJI 飞控说明](https://developer.dji.com/doc/payload-sdk-tutorial/cn/function-overview/advanced-function/flight-control.html) · [DJI WPML 航点参数](https://developer.dji.com/doc/cloud-api-tutorial/cn/api-reference/dji-wpml/waylines-wpml.html)

### MATLAB 航线

- `generate_m400_scan_kmz(cfg)`：普通多高度扫描，入口 [main_scan.m](tools/main_scan.m)。
- `generate_m400_scan_rotate_kmz(cfg)`：在扫描路径上按 `cfg.rotationSpacingM` 米插入停转点，入口 [main_scan_rotate.m](tools/main_scan_rotate.m)。返回值含 `rotationIndices`（MATLAB 从 1 开始的航点序号）、`rotationLatLon`、`rotationHeightM` 和预览图路径。

### M3 ↔ Pi4 通信

妙算端已有独立模块 [m400_pi4_bridge.h](samples/sample_c/module_sample/m400_control/m400_pi4_bridge.h) / [实现](samples/sample_c/module_sample/m400_control/m400_pi4_bridge.c)：

| API | 作用 |
| --- | --- |
| `M400Pi4Bridge_Start(interfaceName, allowedPeerIp, port, callback)` | 启动独立 TCP 监听线程；Pi4 发来的文本交给 `callback`，返回 `0` 表示启动线程成功 |
| `M400Pi4Bridge_PublishEvent(type, index, name, state)` | 向当前 Pi4 订阅者非阻塞推送一条 Pilot 控件事件；无人订阅时直接跳过 |

树莓派端可复用 [tools/m400_pi4_client.py](tools/m400_pi4_client.py)，交互终端 [tools/pi4_console.py](tools/pi4_console.py) 也使用它：

| API | 作用 |
| --- | --- |
| `send_text_once(host, message, port=14560)` | 短连接发送一条文本，收到 `OK` 后返回；适合其他 Pi4 程序直接调用 |
| `M400Pi4Client(host, port=14560)` + `connect()` / `lines()` | 建立全双工订阅并持续读取 `ACK`、Pilot 事件和时间帧；断线后由调用者重连 |
| `M400Pi4Client.send_text(message)` / `send_time_request()` / `close()` | 在订阅连接上发文本、请求时间样本、断开；文本 `ACK` 从 `lines()` 异步读取 |

```python
from m400_pi4_client import send_text_once

send_text_once("10.88.77.54", "Pi4 采集程序已启动")
```

服务绑定妙算 `eth0:14560`，仅接受 Pi4 `10.88.77.1`；文本限一行、UTF-8 最多 240 字节。短连接直接发送 `<文本>\n`，返回 `OK` 或 `ERR`。订阅连接先发 `SUBSCRIBE_BUTTONS\n`，随后以 `TEXT\t<文本>\n` 发送，返回 `ACK\tOK` 或 `ACK\tERR ...`；Pilot 事件为 `BUTTON/SWITCH/LIST\twidget_index\t名称\t状态\n`。当前仅允许一个事件订阅者，事件表示控件操作，**不代表飞行动作完成**。Pi4 终端还支持约 10 秒一次的时差测量；实际校时默认关闭。
