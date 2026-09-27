# M400 控制测试｜妙算 3

## 项目简介

基于 DJI Payload SDK 3.16.0 的 Matrice 400＋妙算 3 实验项目：通过 DJI Pilot 采集飞行数据、查看状态并测试位置/航向控制。本仓库是本地开发副本，不是 DJI 官方示例原仓库。

> 当前版本 **1.3**（DPK 格式 `01.03.00.00`），已在妙算 3 原生编译、安装并启动。此前版 Pilot 控件顺序已现场确认；1.3 的 Pilot 界面和新版飞行动作尚未现场复核。

## 项目基础配置

| 项目 | 当前配置 |
| --- | --- |
| 平台 | M400＋妙算 3；E3 地址 `192.168.42.140`；妙算 Ubuntu 20.04 / ARM64 / GCC 9.4 |
| 应用 | `m400-control-test`；Payload SDK-妙算 3；PSDK 3.16.0；App ID 仅保存在本机配置 |
| 树莓派 SSH（电脑同 Wi-Fi） | `ssh pi4@192.168.124.14` |
| 妙算 SSH（E3 调参链路） | `ssh dji@192.168.42.140` |
| 妙算 SSH（经树莓派跳板） | `ssh -J pi4@192.168.124.14 dji@10.88.77.54` |
| 妙算登录账号 | 默认用户名和密码均为 `dji`；本机也已配置密钥 |
| 版本 | 默认保持 1.3，**改代码不自动升版本**；明确改版本时同步 `app.json` 和程序的 `firmwareVersion` |

App ID、App Key 和高级 License 只保存在本机的 `dji_sdk_app_info.h`、`app.json` 中；这两个文件不提交。新克隆时分别复制 [头文件模板](samples/sample_c++/platform/linux/manifold3/application/dji_sdk_app_info.example.h) 和 [DPK 模板](samples/sample_c/platform/linux/manifold3/app_json/app.example.json)，填写自己的应用信息。Widget JSON 的 `version: 1.0` 是界面配置版本，不随 DPK 改。SDK 文档提示应用更新时应更新版本；若同版本安装被拒绝，先查错，不擅自递增。本仓库基于 [DJI 官方 Payload-SDK](https://github.com/dji-sdk/Payload-SDK) 的 fork，保留原始 [LICENSE.txt](LICENSE.txt) 和 [EULA.txt](EULA.txt)；公开或再分发时应自行核对许可义务。

## 路径介绍

| 用途 | 路径 |
| --- | --- |
| Windows 源码 | `D:\M400-Manifold3-Control` |
| 飞控、遥测、CSV、Widget 回调 | [m400_control_service.c](samples/sample_c/module_sample/m400_control/m400_control_service.c) |
| 可复用 API 声明 | [m400_control_service.h](samples/sample_c/module_sample/m400_control/m400_control_service.h) |
| Pilot 中英文界面 | `samples/sample_c/platform/linux/manifold3/app_json/widget/{cn_big_screen,en_big_screen}/widget_config.json` |
| DPK 元数据 / 程序版本 | 本机 `samples/sample_c/platform/linux/manifold3/app_json/app.json`（仓库仅保留 [模板](samples/sample_c/platform/linux/manifold3/app_json/app.example.json)）/ [application.cpp](samples/sample_c++/platform/linux/manifold3/application/application.cpp) |
| 妙算源码 | `/home/dji/m400-src-v07`（历史目录名，内容随上传更新） |
| 妙算编译产物 / 打包暂存程序 | `/home/dji/m400-src-v07/build-manifold3-ui/bin/dji_sdk_demo_on_manifold3_cxx` / `/home/dji/m400-src-v07/build-manifold3/bin/dji_sdk_demo_on_manifold3_cxx` |
| 当前 DPK / Windows 副本 | `/home/dji/m400-dpk-current/m400-control-test_v01.03.00.00.dpk` / `D:\M400-Manifold3-Control\build-manifold3\dpk\m400-control-test_v01.03.00.00.dpk` |
| 已安装应用 / 可执行文件 | `/open_app/m400-control-test` / `/open_app/m400-control-test/bin/dji_sdk_demo_on_manifold3_cxx` |
| 已安装 Widget / CSV / 日志 | `/open_app/m400-control-test/widget/` / `/open_app/m400-control-test/data/logs/` / `/open_app/m400-control-test/logs/latest.log` |

旧 DPK 留在 `/home/dji/m400-dpk-v*/`；不能仅凭目录名判断安装版本。`dji_app_ctl list m400-control-test` 看安装版本，`dji_app_ctl status m400-control-test` 看运行状态。`dji` 账号通常不能直接读取安装后的 `data` 目录；CSV 可从 Pilot 导出。

## 实现的功能

- 以 50 Hz 记录飞行状态、融合位置、RTK、速度、姿态等 CSV；默认关闭，Pilot 可开始/停止。Payload 设置页可浏览最近 CSV 的日期和时间；同秒文件名自动加 `_01` 等后缀防覆盖。
- 主界面保留开始录制、停止录制、红色紧急悬停三个快捷按钮及浮窗入口。浮窗每秒刷新录制、RTK、经纬度和最近控制结果；Payload 设置页另有录制开关、状态与运动按钮。
- 运动测试：前后左右各 1 m、上下各 1 m、前进 5 m、左右旋转各 90°，以及分步定点 F2/F10、官方例程风格定点 F2。固定点按钮须在 10 秒内连续按**同一个**两次，初始目标距离限 100 m。
- 固定测试点为 `22.602549522388877, 113.99172104792952`，目标高度是**起飞点以上 20 m**，不是海拔或实时离地高度。本项目是 PSDK joystick 闭环试验，**不是** Waypoint/FlyTo 航线任务，也不承诺自动避障。

| 控制 | 水平 / 垂直 / 航向 | 关键区别 |
| --- | --- | --- |
| 前后左右 1 m | 位置 / 位置 / 角速率 | 相对按下时位置和机头方向；水平偏移不截断 |
| 前进 5 m | 位置 / 位置 / 角速率 | 水平每轴位置偏移指令截断到 ±2 m |
| 左右旋转 90° | 保持位置 / 高度 / 角速率 | 指令最多 30°/s，斜坡 10°/s²；不是实测飞行硬上限 |
| 分步定点 F2 / F10 | 位置 / 位置 / 角速率 | 先对航向、再升降、最后水平移动；水平每轴偏移上限 2/10 m |
| 官方例程风格 F2 | 位置 / 位置 / 角度 | 水平、高度、目标航向同时给指令；水平每轴偏移上限 2 m |

这里的 `speedFactor` 是**水平位置偏移指令的每轴米数上限**，不是 m/s，也不限制飞控内部加速度。F10 可能更激进；若两轴剩余误差都不超过 2 m，F2/F10 的水平指令可能相同。

## 关键 API

| API | 作用 |
| --- | --- |
| `M400Control_StartRecording()` / `M400Control_StopRecording()` | 启停 CSV 录制 |
| `M400Control_MoveRelative(direction, distanceM)` / `M400Control_MoveVertical(deltaM)` | 相对水平 / 垂直运动 |
| `M400Control_YawRelative(deltaDegree)` / `M400Control_EmergencyHover()` | 限速相对旋转 / 中断运动并短时发送零速度悬停指令 |
| `M400Control_GotoCoordinate(target)` / `M400Control_GotoCoordinateWithLimit(target, limitM)` | 分步定点；后者可设置水平每轴位置偏移上限 |
| `M400Control_GotoCoordinateOfficialDemo(target)` | 例程风格的水平、高度、航向角同时控制 |
| `DjiFcSubscription_SubscribeTopic()` / `DjiFcSubscription_GetLatestValueOfTopic()` | 订阅并读取飞控遥测 |
| `DjiFlightController_ObtainJoystickCtrlAuthority()` / `SetJoystickMode()` / `ExecuteJoystickAction()` / `ReleaseJoystickCtrlAuthority()` | 获取控制权、设置模式、循环发指令、释放控制权 |
| `DjiWidget_RegUiConfigByDirPath()` / `DjiWidget_RegHandlerList()` | 注册 Pilot 界面及控件回调 |

`T_M400GotoTarget` 在 [API 头文件](samples/sample_c/module_sample/m400_control/m400_control_service.h) 中定义，含 WGS-84 经纬度、起飞点以上高度、目标航向、容差和超时。控制 API 是同步调用；定点控制应放在工作线程，紧急悬停可中断运动。代码要求飞机在空中；控制权获取失败时会提示检查遥控器模式。

## 开发和更新流程

纯航线规划无需更新 DPK：在 MATLAB 中打开 [tools/main.m](tools/main.m)，修改文件开头的高度层、扫描间距/方向和扫描速度，点击“运行”即可；底层生成函数是 [generate_m400_scan_kmz.m](tools/generate_m400_scan_kmz.m)。首次没有保存区域时，`main.m` 会询问是否在地图上顺时针点选；之后默认复用，只有将 `updateRegion=true` 才重新点选，也可直接填写 `regionLatLon`。输出到 `tools/waypoint_output/`：带时间戳的 KMZ、`*_map2d.png`（卫星地图叠加）和 `*_route3d.png`（分层三维航线）。任务默认使用 M400、相对起飞点高度、安全飞往首航点、结束返航、失控退出并返航、安全起飞高度 40 m、航线过渡速度 10 m/s、返航高度 50 m，且不写负载动作；过渡速度与扫描飞行速度分别配置。先在 Pilot 2 导入、预览并检查高度、机型及动作；当前尚未通过 M400 Pilot 实机导入验证。

这是**更新 DPK**，不是刷妙算固件。日常只上传修改的文件；下面没改的 `scp` 行直接跳过，只有明确改版本时才上传最后两项。

```powershell
Set-Location 'D:\M400-Manifold3-Control'
$key = 'C:\Users\Ducky\.ssh\codex_manifold3_ed25519'
scp -i $key 'samples/sample_c/module_sample/m400_control/m400_control_service.c' dji@192.168.42.140:/home/dji/m400-src-v07/samples/sample_c/module_sample/m400_control/
scp -i $key 'samples/sample_c/platform/linux/manifold3/app_json/widget/cn_big_screen/widget_config.json' dji@192.168.42.140:/home/dji/m400-src-v07/samples/sample_c/platform/linux/manifold3/app_json/widget/cn_big_screen/
scp -i $key 'samples/sample_c/platform/linux/manifold3/app_json/widget/en_big_screen/widget_config.json' dji@192.168.42.140:/home/dji/m400-src-v07/samples/sample_c/platform/linux/manifold3/app_json/widget/en_big_screen/
scp -i $key 'samples/sample_c/platform/linux/manifold3/app_json/app.json' dji@192.168.42.140:/home/dji/m400-src-v07/samples/sample_c/platform/linux/manifold3/app_json/
scp -i $key 'samples/sample_c++/platform/linux/manifold3/application/application.cpp' dji@192.168.42.140:/home/dji/m400-src-v07/samples/sample_c++/platform/linux/manifold3/application/
ssh -i $key dji@192.168.42.140
```

在妙算 SSH 中原生编译、打包、安装、启动（无需 `sudo`）：

```bash
cd /home/dji/m400-src-v07
cmake -S samples/sample_c++/platform/linux/manifold3 -B build-manifold3-ui -DCMAKE_BUILD_TYPE=Release
cmake --build build-manifold3-ui --target dji_sdk_demo_on_manifold3_cxx -- -j4
mkdir -p build-manifold3/bin /home/dji/m400-dpk-current
cp build-manifold3-ui/bin/dji_sdk_demo_on_manifold3_cxx build-manifold3/bin/
ver=$(python3 -c 'import json; print(json.load(open("samples/sample_c/platform/linux/manifold3/app_json/app.json"))["firmware_version"])')
bash tools/build_dpk/build_dpk.sh -i samples/sample_c/platform/linux/manifold3/app_json/app.json -o /home/dji/m400-dpk-current
dji_app_ctl install -i "/home/dji/m400-dpk-current/m400-control-test_v$ver.dpk"
dji_app_ctl start m400-control-test
sleep 10
dji_app_ctl status m400-control-test
dji_app_ctl list m400-control-test
```

同版本重打包会覆盖 `m400-dpk-current` 中的同名包，不动历史包。需要 Windows 留一份时，退出 SSH 后执行：

```powershell
scp -i $key dji@192.168.42.140:/home/dji/m400-dpk-current/m400-control-test_v01.03.00.00.dpk 'D:\M400-Manifold3-Control\build-manifold3\dpk\'
```

## 联合开发网络（树莓派 Wi-Fi → 妙算 3）

树莓派 4B 通过 Wi-Fi 上网，并用网口直连妙算 3 扩展坞网口；树莓派的 [NetworkManager `shared` 模式](https://networkmanager.dev/docs/api/latest/nm-settings-nmcli.html)负责给妙算分配地址、DNS 和 NAT。**不依赖 M400 内部网络转发，也不需要额外 AP。**这是两个独立网段，妙算并未直接加入 Wi-Fi：

```text
电脑 ── Wi-Fi 192.168.124.0/24 ── 树莓派 wlan0 192.168.124.14
                                  树莓派 eth0 10.88.77.1 ── 网线 ── 妙算 eth0 10.88.77.54
                                                       妙算 E3 调参地址 192.168.42.140
```

以上是 2026-09-27 的实测地址；两机重启一次后地址未变，但 `192.168.124.14` 和 `10.88.77.54` 都是 DHCP 地址，后续仍可能变化。`192.168.42.140` 只用于电脑连接飞机调参链路时访问妙算。

电脑 PowerShell 登录树莓派、经树莓派跳板登录妙算（跳板连接已由用户验证）：

```powershell
ssh pi4@192.168.124.14
ssh -J pi4@192.168.124.14 dji@10.88.77.54
```

首次配置树莓派（`m3-share` **只创建一次**；已存在时只需 `sudo nmcli connection up m3-share`）：

```bash
ip route get 1.1.1.1  # 须显示 dev wlan0、src 192.168.124.14；否则先确认 Wi-Fi 出口
sudo nmcli connection add type ethernet ifname eth0 con-name m3-share \
  ipv4.method shared ipv4.addresses 10.88.77.1/24 ipv6.method disabled
sudo nmcli connection up m3-share
ip -4 -br addr show eth0  # 预期 10.88.77.1/24
```

妙算扩展坞网口当前为 `eth0`。初次配置或未拿到地址时使用 [DJI `netctl.sh`](https://developer.dji.com/doc/payload-sdk-tutorial/cn/manifold-quick-start/manifold-platform-capabilities/system-tools.html)：

```bash
sudo /system/bin/netctl.sh setup_dhcp_ip --type eth --iface eth0
sudo /system/bin/netctl.sh setup_route --route "default>eth" --iface eth:eth0
ip -4 -br addr show eth0  # 当前为 10.88.77.54/24
ping -c 3 10.88.77.1  # 树莓派网口
ping -c 3 1.1.1.1     # 外网；失败时先在树莓派检查 Wi-Fi 出口
getent hosts baidu.com # DNS
```

从树莓派本机访问妙算：`ssh dji@10.88.77.54`。网线重插后会自动重连，但原 SSH/数据流会中断，妙算地址也可能改变。拔掉网线时，树莓派 Wi-Fi 和本地采集不受影响；妙算失去通过树莓派的网络出口，E3/PSDK 链路不依赖此线。当前妙算的以太网默认路由优先于机身网络；以太网断开后可能回退到机身默认路由，**不保证仍能上网**。配置保留、重启后自动恢复已实测一次；若树莓派 Wi-Fi 断开但网线仍连着，妙算也无法经树莓派上网。

## 踩坑总结

- **Pilot 设置页空白：**应用启动后先点右侧「飞行设置」，再切「Payload 设置」触发刷新。`Running` 或主界面浮窗出现，不代表右侧已刷新。
- **Widget 排序：**Pilot 按 `widget_index`，仅调 JSON `widget_list` 数组顺序无效。须同步核对中英文 JSON 和 C 回调索引，逐个确认按钮动作。Pilot 自动生成的「显示实时数据」在自定义列表之前，不能单独重排；CSV 的 `0=无后缀` 控件已移除，但文件名防重逻辑仍在。
- **交叉编译 ABI：**WSL Ubuntu 22.04 可用于编辑和传文件；其默认运行库编出的 ARM64 程序曾因 `GLIBC/GLIBCXX` 版本不匹配而安装失败。当前可靠路径是在妙算 Ubuntu 20.04 / GCC 9.4 上原生编译；DPK 打包成功不等于能运行。
- **版本和状态：**`status` 曾把版本字段反序显示，安装版本以 `dji_app_ctl list`、`app.json` 和 DPK 文件名为准。同版本安装若被拒绝，不自动升版本。更新 DPK 后既有 CSV 目录此前未归零，但重要数据仍应自行导出。
- **按需排错：**脚本报 `$'\r'` 时运行 `sed -i 's/\r$//' tools/build_dpk/build_dpk.sh`；异常退出看 `journalctl -u m400-control-test.service -n 100 --no-pager`；安装失败看 `/blackbox/system/app_temp_files/m400-control-test_*.log`。

参考：[DJI 妙算运行例程](https://developer.dji.com/doc/payload-sdk-tutorial/cn/manifold-quick-start/run-sample-code.html) · [DJI 飞控接口说明](https://developer.dji.com/doc/payload-sdk-tutorial/cn/function-overview/advanced-function/flight-control.html)
