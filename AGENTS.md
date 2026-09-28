# 开发与维护指南

本文件既是仓库维护约定，也是 M400＋妙算 3＋Pi4 的操作教程。项目概览、功能与可调用 API 见 [README.md](README.md)。

## 仓库约定与路径

- 唯一编辑目录：`D:\M400-Manifold3-Control`；不要回到旧的 `D:\Payload-SDK-Rndis-Simplify-Manifold3` 或临时 public/upstream 副本改项目。
- 改代码默认保持应用版本 **1.4**（DPK `01.04.00.00`），只有用户明确要求才改版本；若改版本，同步本机 `app.json` 和 `application.cpp` 的 `firmwareVersion`。Widget JSON 的 `version: 1.0` 是独立配置版本。
- App ID、App Key、高级 License 只保存在未跟踪的 `samples/sample_c++/platform/linux/manifold3/application/dji_sdk_app_info.h` 与 `samples/sample_c/platform/linux/manifold3/app_json/app.json`。公开前检查 `git status`、暂存差异，不能提交凭据、DPK 或飞行 CSV。
- 修改和部署分开：除非用户明确要求，不自动传输、编译、安装、启动、提交或推送。用户在 `tools/main_scan.m`、`tools/main_scan_rotate.m` 等文件中的现有修改应保留。

| 内容 | 位置 |
| --- | --- |
| 本地项目 | `D:\M400-Manifold3-Control` |
| M3 飞控、CSV、Widget 回调 | `samples/sample_c/module_sample/m400_control/m400_control_service.c/.h` |
| M3 ↔ Pi4 TCP 模块 | `samples/sample_c/module_sample/m400_control/m400_pi4_bridge.c/.h` |
| Pi4 通信 API / 终端 | `tools/m400_pi4_client.py` / `tools/pi4_console.py` |
| MATLAB 航线入口 / 生成器 | `tools/main_scan.m`、`tools/main_scan_rotate.m` / `tools/generate_m400_scan_kmz.m`、`tools/generate_m400_scan_rotate_kmz.m` |
| Pilot 中英文配置 | `samples/sample_c/platform/linux/manifold3/app_json/widget/{cn_big_screen,en_big_screen}/widget_config.json` |
| M3 上源码 | `/home/dji/m400-src-v07`（历史目录名，源码可持续覆盖更新） |
| M3 原生编译输出 / 打包暂存程序 | `/home/dji/m400-src-v07/build-manifold3-ui/bin/dji_sdk_demo_on_manifold3_cxx` / `/home/dji/m400-src-v07/build-manifold3/bin/dji_sdk_demo_on_manifold3_cxx` |
| 已安装应用 / 可执行文件 | `/open_app/m400-control-test` / `/open_app/m400-control-test/bin/dji_sdk_demo_on_manifold3_cxx` |
| 已安装 CSV / 运行日志 | `/open_app/m400-control-test/data/logs/` / `/open_app/m400-control-test/logs/latest.log` |
| Pi4 客户端目录 | `/home/pi4/Desktop/M400-Manifold3-Clients` |

DPK 可以放在 `/home/dji/m400-dpk-current/`，历史包也可能散在 `/home/dji/m400-dpk-v*/` 等目录；**目录名不是已安装版本的证据**。用 `dji_app_ctl list m400-control-test` 看安装版本、`dji_app_ctl status m400-control-test` 看运行状态。`dji` 账号可能不能直接读安装后的 `data` 目录；CSV 可从 Pilot 导出。更新 DPK 后历史 CSV 曾保留，但这不是永久保证。

## 本地编码 → M3 编译/更新 DPK

在 Windows 修改源码后，只上传实际改动的文件。电脑连 E3 调参链路时可直接 `ssh dji@192.168.42.140`；通过 Pi4 网络时用下面的跳板命令。妙算默认用户名、密码均为 `dji`；也可用本机已配置密钥。

```powershell
Set-Location 'D:\M400-Manifold3-Control'
$pi = 'pi4@192.168.124.14'
$m3 = 'dji@10.88.77.54'
scp -J $pi 'samples/sample_c/module_sample/m400_control/m400_control_service.c' "${m3}:/home/dji/m400-src-v07/samples/sample_c/module_sample/m400_control/"
# 如果修改了通信模块，也分别上传 m400_pi4_bridge.c 和 m400_pi4_bridge.h 到同一目录。
# 两点航线与遥控器告警功能还须上传 m400_waypoint_kmz.c/.h、m400_control_alert.c/.h 和 m400_control_service.h；新增 .c 后重新运行 cmake 配置以更新 GLOB 源文件列表。
# 如果修改了控件，上传中英文 widget_config.json 到 M3 源码中的对应目录。
# 仅明确改版本时，才上传本机 app.json 与 application.cpp。
ssh -J $pi $m3
```

IP 是此前 DHCP 记录值，变更时在 Pi4/M3 上分别核对 `ip -4 -br addr`。首次连接/不同 SSH 密钥环境下，可改用本机 SSH 配置或显式 `-i`；不要把私钥或应用凭据复制进仓库。进入 M3 SSH 后，在目标 Ubuntu 20.04 / ARM64 / GCC 9.4 上原生编译，**不要用 WSL 22.04 默认 ARM64 运行库构建后直接安装**：

```bash
cd /home/dji/m400-src-v07
cmake -S samples/sample_c++/platform/linux/manifold3 -B build-manifold3-ui -DCMAKE_BUILD_TYPE=Release
cmake --build build-manifold3-ui --target dji_sdk_demo_on_manifold3_cxx -- -j4
mkdir -p build-manifold3/bin /home/dji/m400-dpk-current
cp build-manifold3-ui/bin/dji_sdk_demo_on_manifold3_cxx build-manifold3/bin/
ver=$(python3 -c 'import json; print(json.load(open("samples/sample_c/platform/linux/manifold3/app_json/app.json"))["firmware_version"])')
bash tools/build_dpk/build_dpk.sh -i samples/sample_c/platform/linux/manifold3/app_json/app.json -o /home/dji/m400-dpk-current
dji_app_ctl stop m400-control-test
dji_app_ctl install -i "/home/dji/m400-dpk-current/m400-control-test_v$ver.dpk"
dji_app_ctl start m400-control-test
dji_app_ctl status m400-control-test
dji_app_ctl list m400-control-test
```

这是更新 DPK，**不是刷固件**。同版本重打包会覆盖 `m400-dpk-current` 内同名包，不修改其它历史目录。若用户需要 Windows 副本，可从 M3 `scp` 回 `build-manifold3/dpk/`；不要误把未更新的本地副本当成本次部署包。安装失败时先查原因，不擅自递增版本。

## Pi4 通信客户端更新与使用

M3 端 `M400Pi4Bridge_Start()` 在服务启动时绑定 `eth0:14560`，只接收源 IP `10.88.77.1`。Pi4 端 `pi4_console.py` **依赖** `m400_pi4_client.py`，更新时两个文件需放在同一目录：

```powershell
scp tools/pi4_console.py tools/m400_pi4_client.py pi4@192.168.124.14:/home/pi4/Desktop/M400-Manifold3-Clients/
```

Pi4 上运行交互终端：

```bash
cd ~/Desktop/M400-Manifold3-Clients
python3 pi4_console.py 10.88.77.54
```

终端标题原位刷新两机时差、RTT、M3 UTC；消息区显示 Pilot 控件事件，底部输入行向 M3 发文本。自动校时默认关闭，仅测时差；确认 M3 时钟可靠后，可用 `sudo python3 pi4_console.py 10.88.77.54 --sync-time` 启用，运行中 `Ctrl+T` 切换。实际校时需要 root 或 `CAP_SYS_TIME`；该机制是 TCP 上的四时间戳估计，并非标准 NTP。独立程序只想发一条 Pilot 文本可调用 `send_text_once()`；需要持续收事件则用 `M400Pi4Client.connect()/lines()`，并自行处理断线重连。Pi4 关机不阻塞 M3 控制线程；浮窗最后一条 Pi4 文本可能滞留，不能当在线心跳。

## 联合开发网络初次配置

电脑和 Pi4 在同一 Wi-Fi，Pi4 `wlan0` 曾为 `192.168.124.14`；Pi4 `eth0` 直连 M3 扩展坞 `eth0`，使用 `10.88.77.0/24`。M3 E3 调参地址另为 `192.168.42.140`。两机重启后地址曾保持，但 Pi4 Wi-Fi/M3 eth0 均可能随 DHCP 变化。

Pi4 上只创建一次 NetworkManager 共享连接 `m3-share`；已存在时只需 `sudo nmcli connection up m3-share`：

```bash
ip route get 1.1.1.1  # 确认上网出口是 wlan0
sudo nmcli connection add type ethernet ifname eth0 con-name m3-share \
  ipv4.method shared ipv4.addresses 10.88.77.1/24 ipv6.method disabled
sudo nmcli connection up m3-share
ip -4 -br addr show eth0
```

M3 初次配置扩展坞 `eth0` 或未拿到 IP 时，使用 DJI `netctl.sh`：

```bash
sudo /system/bin/netctl.sh setup_dhcp_ip --type eth --iface eth0
sudo /system/bin/netctl.sh setup_route --route "default>eth" --iface eth:eth0
ip -4 -br addr show eth0
ping -c 3 10.88.77.1
```

网线断开时，Pi4 Wi-Fi 与本地采集不受影响；M3 失去通过 Pi4 的网络出口，已有 SSH/TCP 流中断，但 E3/PSDK 不依赖此线。M3 以太网断开后是否还能经机身网络上网不保证。以上配置重启后曾自动恢复一次；不等于地址永远固定。

## MATLAB 航线生成

航线规划不需重打 DPK。普通扫描运行 `tools/main_scan.m`；定距停转 360° 运行 `tools/main_scan_rotate.m`，并在文件开头设置 `rotationSpacingM`。两者均可设置高度层、扫描间距、方向和速度。首次无已保存区域时可在地图上顺时针点选，以后默认复用；仅 `updateRegion=true` 才重新点选，也可手填 `regionLatLon`。输出在 `tools/waypoint_output/`：KMZ、地图叠加 `*_map2d.png` 和分层 `*_route3d.png`；紫色星标为停转点。停转间距在每个高度层重新累计，层间爬升不计入。`rotateYaw` 为绝对航向角，生成器以四个连续 90° 目标角表示一圈；WPML 未给出该动作的角速度／加速度字段，M400 Pilot 对动作的导入和实飞平顺性尚未验证。

任务默认：M400、相对起飞点高度、安全飞往首航点、结束返航、失控退出并返航、安全起飞 40 m、全局航线过渡速度 10 m/s、返航高度 50 m、无负载动作。扫描速度与全局过渡速度分开配置。导入前在 Pilot 2 中预览航线、高度、机型和动作。

应用内的“执行两点直线航线”与 MATLAB 扫描航线不同：仅已起飞时执行，从实时融合位置生成两点 WPML KMZ，结束动作为 `noAction`；安全起飞高度在此场景不生效。Waypoint 3 上传／启动只表示任务已接受，不等于飞行及终点偏航动作已完成；需核对 Pilot 与实际航迹。普通 scan 预期应改为协调转弯，而当前生成器仍把所有航点写为“直线到点停”；不要把现有输出误认为已是协调转弯。

## 踩坑与排错

- Pilot「Payload 设置」空白：先打开右侧「飞行设置」，再切回「Payload 设置」触发刷新。应用显示 Running 或主界面浮窗出现，不代表右侧已刷新。
- Widget 排序按 `widget_index`，只改 JSON 数组顺序无效。改控件要同步核对中英文 JSON 与 C 回调索引。Pilot 自带的「显示实时数据」入口在自定义列表前，不能单独挪动。
- WSL 22.04 适合编辑、传文件；其默认 ARM64 交叉编译产物曾因 `GLIBC_2.33/2.34`、`GLIBCXX_3.4.29` 不兼容而无法在 M3 安装。用 M3 原生 Ubuntu 20.04/GCC 9.4 构建；DPK 打包成功不等于程序能运行。
- `dji_app_ctl status` 曾把版本字段反序显示；版本以 `dji_app_ctl list`、本机 `app.json` 和 DPK 文件名交叉核对。同版本安装拒绝先看错误日志，不自动升版。
- 打包脚本若报 `$'\r'`，在 M3 源码中运行 `sed -i 's/\r$//' tools/build_dpk/build_dpk.sh`。异常退出看 `journalctl -u m400-control-test.service -n 100 --no-pager`；安装失败看 `/blackbox/system/app_temp_files/m400-control-test_*.log`。
- Pi4 文本/控件事件是展示与通知链路，不是远程飞控 API：`BUTTON` 表示按下，不保证动作完成；当前单订阅者，断线事件不补发。若 Pi4 没开机，M3 监听线程独立重试，不会阻塞飞控工作线程。
- 遥控器暂停会收回 PSDK Joystick 控制权。控制程序订阅授权切换事件并退出当前循环；再次点按钮需重新申请控制权，飞控若拒绝不能靠软件强制抢回。历史版本未订阅此事件，暂停后可能长时间显示 `controller busy`。官方例程风格定点 F2 因实飞过于突兀已从本项目应用逻辑移除；上游 PSDK 示例保留。
- 2026-09-28 用户实飞反馈：自定义水平与旋转动作平稳；F10 比 F2 快，但侧风下终点纠偏会导致航迹不直。现有 Joystick 位置控制只追终点，严格直线需横向误差反馈与实飞整定；WPML 航线的 `useStraightLine=1` 也只是尽量贴线。旧低高度航线已能启动并飞向首航点，但未完整执行，不能据此确认新停转动作的 M400 兼容性。

参考：[DJI 妙算运行例程](https://developer.dji.com/doc/payload-sdk-tutorial/cn/manifold-quick-start/run-sample-code.html) · [DJI 妙算网络工具](https://developer.dji.com/doc/payload-sdk-tutorial/cn/manifold-quick-start/manifold-platform-capabilities/system-tools.html) · [DJI 飞控接口](https://developer.dji.com/doc/payload-sdk-tutorial/cn/function-overview/advanced-function/flight-control.html)
