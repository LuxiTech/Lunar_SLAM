# Luxi Web Control 启动说明

本包同时提供两套可相互切换的网页，共用同一个 ROS 2/HTTP 服务：

- `map_portal`（默认）：面向普通用户的地图中心，提供三维地图/代价地图查看、图层下载、
  相机开关与 RGB 画面、机器人控制与双电池信息、IMU 水平校准、自动定位、目标点下发和
  导航任务控制；
- `developer`：原有开发者控制台，负责机器人遥控、相机选择、IMU 校准、建图和定位。

默认入口 `/` 打开地图中心；页面右上角可直接切换用户模式与开发者模式，开发者页面
固定为 `/developer`，用户页面固定为 `/user`。`web_ui_mode` 参数仍可兼容旧脚本，用于
指定 `/` 的默认页面。地图 API 在两种模式下保持一致，外部网页或程序可以直接调用。


## 2. 关闭所有旧进程

首次改用网页托管相机或系统状态不明确时，在工程根目录执行：

```bash
bash scripts/stop_luxi_system.sh
```

看到下面的提示后才能继续：

```text
Luxi/D1 processes stopped, stale PID files removed, and port 8080 is available.
```

不要使用 `killall python3`，也不要通过改成 8081 来绕过旧网页进程。

## 3. 启动网页服务

只需在一个终端中进入工程根目录并执行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch luxi_web_control lekiwi_web_control.launch.py \
  bind_address:=0.0.0.0 http_port:=8080
```

`0.0.0.0` 表示网页监听本机所有网卡，不要求本机必须拥有特定的 D1 网段地址。D1 的
ROS 2 通信仍由该启动文件中的 `ROS_DOMAIN_ID=42` 和 Fast DDS 设置管理；网页监听范围
与 D1 通信网段彼此独立。

`0.0.0.0` 是监听方式，不是浏览器访问地址。启动日志会自动列出当前网卡可访问的 URL。
例如机器人连接其他 Wi-Fi 后地址是 `10.121.31.213`，手机或电脑应打开：

```text
http://10.121.31.213:8080
```

访问设备必须与机器人 Wi-Fi 网络互通；如果 Wi-Fi 开启了 AP/客户端隔离，即使地址正确
也不能相互访问，需要在路由器中关闭客户端隔离或换用允许局域网互访的热点。

浏览器打开：

```text
http://机器人电脑IP:8080
```

查看本机 IP：

```bash
hostname -I
```

默认打开地图中心。需要建图时，点击页面右上角“开发者模式”；完成后点击“用户模式”
即可返回，无需停止或重新启动 ROS 2 服务。旧部署如需让 `/` 默认打开开发者页面，仍可
追加兼容参数 `web_ui_mode:=developer`。

## 地图中心 HTTP API

地图中心以 `mapNNN` 为资源单位。地图选择列表固定显示多行，选择一个 map 后，下方会
直接显示该地图的 DB、PLY、BT、过滤版 PLY/BT、语义标注和 HLoc 元数据，每个文件都
可以独立下载。页面打开后会自动选中编号最新的可用过滤版地图。从开发者模式停止一次
新建图后，后端会自动导出点云、过滤离群点、重建 OctoMap，完成后两个页面都会自动
选中并显示该过滤地图。历史地图仍由用户确认后点击“加载地图预览”，不会在初始
连接时自动启动地图转换。存在过滤版时，网页和未
显式指定 `filtered` 的预览 API 都默认使用过滤版。地图沿用开发者页面的三维轨道视图：
拖动改变 yaw/pitch、滚轮缩放；进入“选择目标”模式后点击可通行面选择导航目标。

导航区点击“选择并发送目标点”后，采用与 RViz 相同的手势：在可通行面按下确定
目标位置，保持按住并拖动箭头确定朝向，松开后同时发送位置和方向。角度输入框仍可
用于精确调整（0° 指向地图 +X，90° 指向 +Y）。
到达位置后，机器人会原地对准该方向，方向误差小于配置阈值后才报告导航结束。
“停止任务”只停止跟随并清除当前和缓存路径，定位进程会继续运行；路径清空后可以用
WASD 或页面轮盘临时手动控制，松开即停车，再次选点即可开始下一次任务。

地图中心不会返回服务器绝对路径。地图下载只接受已发现的 `mapNNN` 和固定图层名称，
大文件使用流式响应并支持 `Range` 断点续传。网页内所有地图、建图、定位、导航、相机
和机器人控制均通过 HTTP API 实现；`GET /api/capabilities` 可以查询完整操作清单。
面向外部网页和应用的完整请求/响应、局域网、轮询、错误码与安全说明见
[API接口使用与交付说明.md](API接口使用与交付说明.md)。

```text
GET  /api/capabilities
GET  /api/maps
GET  /api/maps/map037
POST /api/maps/map037/preview       {}  # 默认过滤版（存在时）
POST /api/maps/map037/preview       {"filtered": false}
GET  /api/maps/map037/preview/cloud
GET  /api/maps/map037/preview/voxels
GET  /api/maps/map037/preview/terrain
GET  /api/maps/map037/preview/costmap
GET  /api/maps/map037/preview/obstacles
GET  /api/maps/map037/preview/path
GET  /api/maps/map037/download/database
GET  /api/maps/map037/download/cloud
GET  /api/maps/map037/download/octomap
GET  /api/maps/map037/download/filtered_cloud
GET  /api/maps/map037/download/filtered_octomap
GET  /api/maps/map037/download/annotations
GET  /api/maps/map037/download/hloc_metadata
```

导航任务继续使用统一接口：

```text
POST /api/navigation/localize       {"map_id": "map037"}
POST /api/navigation/goal           {"x": 1.2, "y": 0.8, "z": 0.0, "yaw": 1.57}
POST /api/navigation/home/set       {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0}
POST /api/navigation/home/return    {}
POST /api/navigation/start          {}
POST /api/navigation/halt           {}
POST /api/navigation/stop           {}
POST /api/system/restart            {"confirm": "restart_all_services"}
GET  /api/status
GET  /api/navigation/path
```

返航点与当前地图绑定，由网页服务端统一保存，因此不同手机或电脑看到的是同一个返航
点。“一键返回起点”会取消手动控制和已有导航，清除旧路径，规划成功且跟随器确认收到
新路径后才自动出发。普通任务到达、停止或异常中断后，路径接口返回空路径。

页面顶部“一键重启”会先停车、停止导航/建图/相机、让机器人趴下并关闭全部 Luxi/D1
服务，确认清理完成后按当前 HTTP 地址、端口和页面模式重新拉起网页服务。页面会自动
等待服务恢复并刷新。为避免无人值守运动，重启后不会自动站立或启动相机；请重新开启
相机、机器人控制并完成 IMU 校准。

浏览器跨页面调用示例：

```javascript
const robot = "http://机器人IP:8080";
const catalog = await fetch(`${robot}/api/maps`).then(response => response.json());
await fetch(`${robot}/api/navigation/goal`, {
  method: "POST",
  headers: {"Content-Type": "application/json"},
  body: JSON.stringify({x: 1.2, y: 0.8, z: 0.0, yaw: 1.57}),
});
```

部署到非可信局域网前，应在反向代理处配置 HTTPS、访问令牌和允许的网页来源；导航和
机器人控制接口不应直接暴露到公网。

## 4. 在网页中启动或切换相机

1. 打开网页中的“相机选择与开关”；
2. 选择 `D455 / D455F`、`D435i` 或 `HIK 双目 + H30 IMU`；
3. 点击“启动 / 切换”，等待状态显示“运行中”且 RGB 画面出现；
4. D435i/D455 首次启动或更换支架后，将机器人放在水平面并点击“一键水平校准”；
5. 再点击“开始建图”。建图结束必须点击“停止建图”并等待数据库保存完成。

切换相机会自动停止行驶、建图和定位，再关闭旧相机并启动新 profile。同一时间只允许
一套传感器发布者。不要在其他终端手工启动 `sensor_bringup.launch.py`；网页发现外部
相机时会拒绝接管，以免误杀未知进程。

D455F 在当前室内照明下启用自动白平衡时，实测会在正常与明显暖色之间反复漂移。
驱动现已固定为 `3450 K`：启动后色彩立即稳定，适合建图连续帧。如果机器人换到色温
差异很大的室外或其他场地，应重新实测并修改
`device/D455/ros2_ws/src/lunar_d455_bringup/config/d455.yaml`，不要重新打开自动白平衡。

只测试本机网页、不连接 D1 底盘时，可改用：

```bash
ros2 launch luxi_web_control web_control.launch.py \
  bind_address:=127.0.0.1 http_port:=8080
```

## 5. 关闭系统

正常关闭顺序：

1. 网页点击“停止行驶”；
2. 网页点击“停止建图”并等待保存完成；
3. 网页点击“停止定位”（如果已经启动定位）；
4. 网页点击“关闭相机”；
5. 终端按 `Ctrl-C` 关闭网页；网页退出也会关闭由它启动的相机；
6. 回到工程根目录执行一次最终清理：

   ```bash
   bash scripts/stop_luxi_system.sh
   ```
