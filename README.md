# M400 控制测试｜妙算 3

## 项目简介

基于 DJI Payload SDK 3.16.0 的 Matrice 400＋妙算 3 实验项目：在 DJI Pilot 中记录飞行遥测、查看状态、测试位置与航向控制，并通过独立网线与树莓派 4B 交换文本及 Pilot 控件事件。本仓库是基于 [DJI Payload-SDK](https://github.com/dji-sdk/Payload-SDK) 的开发 fork，不是官方示例原仓库。

当前应用版本为 **1.3**（DPK `01.03.00.00`）；改代码时默认不自动升版本。妙算原生编译、安装和启动已完成；Pilot 新版界面与飞行动作仍需实际复核。开发命令、部署路径、网络初次配置和踩坑见 [AGENTS.md](AGENTS.md)。

## 配置

| 项目 | 当前配置 |
| --- | --- |
| 硬件与系统 | M400＋妙算 3（Ubuntu 20.04 / ARM64 / GCC 9.4）＋树莓派 4B |
| 应用 | `m400-control-test`；Payload SDK-妙算 3；PSDK 3.16.0 |
| 源码 | `D:\M400-Manifold3-Control`；飞控与遥测在 [m400_control_service.c](samples/sample_c/module_sample/m400_control/m400_control_service.c) |
| Pilot 界面 | `samples/sample_c/platform/linux/manifold3/app_json/widget/{cn_big_screen,en_big_screen}/widget_config.json` |
| 树莓派 SSH | `ssh pi4@192.168.124.14` |
| 妙算 SSH：E3 / Pi4 跳板 | `ssh dji@192.168.42.140` / `ssh -J pi4@192.168.124.14 dji@10.88.77.54` |
| Pi4 ↔ 妙算 | Pi4 `eth0`：`10.88.77.1`；妙算 `eth0`：`10.88.77.54`；TCP 端口 `14560` |

`192.168.124.14`、`10.88.77.54` 为此前 DHCP 地址，变更时需更新命令。妙算默认用户名和密码均为 `dji`；本机也已配置 SSH 密钥。Pi4 以 Wi-Fi 上网、以太网直连妙算扩展坞，使用 NetworkManager `shared` 模式为妙算提供地址、DNS 和网络出口；通信不依赖 M400 机载局域网。

App ID、App Key 和高级 License 仅放在本机 `dji_sdk_app_info.h`、`app.json`，**不提交**。新克隆时从 [应用头文件模板](samples/sample_c++/platform/linux/manifold3/application/dji_sdk_app_info.example.h) 和 [DPK 模板](samples/sample_c/platform/linux/manifold3/app_json/app.example.json) 创建本地配置。本仓库保留上游 [LICENSE.txt](LICENSE.txt) 和 [EULA.txt](EULA.txt)。

## 主要功能

- 50 Hz 飞行状态 CSV：记录融合位置、RTK、速度、姿态等；Pilot 可启停，并在 Payload 设置页浏览最近文件日期和时间。已安装应用的数据位于 `/open_app/m400-control-test/data/logs/`。
- Pilot 控件：主界面有录制启停、红色紧急悬停和浮窗入口；Payload 设置页有 RTK、经纬度、命令状态及运动测试按钮。浮窗上行显示最近 Pi4 文本，下行显示最近 CMD 响应。
- 飞行试验：相对前后左右各 1 m、上下各 1 m、前进 5 m、左右旋转各 90°；分步定点 F2/F10 和官方例程风格定点 F2。固定测试点为 `22.602549522388877, 113.99172104792952`，目标高度为**起飞点以上 20 m**。固定点按钮须在 10 秒内连续按同一个两次，初始距离限 100 m。这里使用 PSDK joystick 闭环控制，不是 Waypoint/FlyTo，也不承诺自动避障。
- Pi4 双向通信：Pi4 文本显示在 Pilot 浮窗；Pilot 按钮、录制开关和 CSV 选择事件推送给 Pi4。文本不会被解释为飞控指令。Pi4 断电/断线不使飞控等待；断线期间的事件不补发，浮窗可能保留断线前最后一条 Pi4 文本。
- MATLAB 多高度扫描航线：修改 [tools/main.m](tools/main.m) 的区域、高度、间距、方向等参数，生成 WPML KMZ、地图叠加 2D 图和带方向箭头的 3D 图。KMZ 设置 `followWayline`，预期机头跟随当前航段；尚未通过 M400 Pilot 实机导入验证。

## API 介绍

### 飞行控制与遥测（妙算 C）

公开声明见 [m400_control_service.h](samples/sample_c/module_sample/m400_control/m400_control_service.h)。

| API | 作用 |
| --- | --- |
| `M400Control_StartRecording()` / `M400Control_StopRecording()` | 启停 CSV 录制 |
| `M400Control_MoveRelative(direction, distanceM)` / `M400Control_MoveVertical(deltaM)` | 相对水平 / 垂直位移 |
| `M400Control_YawRelative(deltaDegree)` / `M400Control_EmergencyHover()` | 限速相对旋转 / 中断运动并短时发送悬停指令 |
| `M400Control_GotoCoordinate(target)` / `M400Control_GotoCoordinateWithLimit(target, limitM)` | 先航向、再高度、最后水平的分步定点；后者限制每轴水平位置偏移指令 |
| `M400Control_GotoCoordinateOfficialDemo(target)` | 水平、高度、航向角同时给指令的例程风格对照 |
| `DjiFcSubscription_*` / `DjiFlightController_*` / `DjiWidget_*` | 底层 DJI 遥测订阅、控制权与 joystick 指令、Pilot 控件注册 |

`T_M400GotoTarget` 包含 WGS-84 经纬度、起飞点以上高度、航向、容差和超时。控制 API 是同步调用；耗时动作放在工作线程。`speedFactor`/`limitM` 是**每轴水平位置偏移指令的米数上限**，不是 m/s 或飞控内部加速度上限。飞行控制要求飞机已在空中并取得控制权。

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
