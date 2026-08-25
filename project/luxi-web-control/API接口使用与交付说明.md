# Luxi 用户端 HTTP API 接口使用与交付说明

文档版本：2.0  
适用代码：`lunar_slam` 工作区 2026-08-19 当前版本  
服务包：`luxi_web_control`  
默认端口：`8080`  
默认用户页面：`map_portal`

## 1. 接口结论

当前用户网页中的地图清单、地图预览、地图文件下载、静态障碍物、代价地图、相机与 RGB
画面、机器人控制与电量、IMU 校准、定位、目标点、返航、导航启停、WASD/轮盘遥控、急停和
一键重启均由 HTTP API 实现。外部网页、手机应用或其他程序不需要复制网页逻辑，可以直接
调用同一组接口。

保存地图可以读取加入障碍物和代价值后的结果：

- `GET /api/maps/{map_id}/preview/costmap`：独立读取可通行表面和 `0～1` 代价值；
- `GET /api/maps/{map_id}/preview/obstacles`：独立读取由保存 PLY 分割出的静态障碍物；
- `GET /api/maps/{map_id}/preview/terrain`：一次读取上述两层的聚合结果；
- `GET /api/maps/{map_id}/preview/cloud`：读取彩色 PLY 点云；
- `GET /api/maps/{map_id}/preview/voxels`：读取 BT OctoMap 占据体素。

这里要区分两类障碍：

1. `terrain/costmap/obstacles` 是保存地图的静态图层，可以通过 HTTP 获取、显示和重复使用；
2. D435i 导航时生成的滚动动态障碍层是实时安全数据，会随时间衰减，目前由导航进程内部通过
   ROS 2 话题 `/navigation/local_obstacles/points` 使用，不会写回 `.db/.ply/.bt`，当前也不通过
   HTTP 地图接口下载。不能把一次实时动态障碍误认为永久地图。

## 2. 网络连接要求

### 2.1 是否必须连接同一个局域网

最简单、推荐的方式是机器人电脑和调用端连接同一个局域网或同一个 Wi-Fi。严格来说不要求
二者处于完全相同的子网，只要调用端能够路由到机器人 IP，并且 TCP `8080` 端口可达即可。

例如机器人地址为 `192.168.137.20`，API 根地址为：

```text
http://192.168.137.20:8080
```

`0.0.0.0` 只是服务端监听所有网卡的配置，不能作为客户端访问地址。机器人上可执行：

```bash
hostname -I
```

也可以从 `GET /api/status` 返回值的 `access_urls` 查看可访问地址。

如果无法访问，请检查：

- 两台设备能否互相 `ping`；
- 路由器是否开启了 AP 隔离/客户端隔离；
- 防火墙是否允许 TCP 8080；
- 浏览器中使用的是否是机器人的真实 IP，而不是 `0.0.0.0`；
- 服务是否使用 `bind_address:=0.0.0.0` 启动。

### 2.2 启动服务

在机器人电脑的工程根目录执行：

```bash
cd /home/lunar/project/lunar_slam
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch luxi_web_control lekiwi_web_control.launch.py \
  bind_address:=0.0.0.0 http_port:=8080 web_ui_mode:=map_portal
```

开发者页面和用户页面二选一，但 API 完全相同。切换开发者页面只需改为：

```text
web_ui_mode:=developer
```

### 2.3 HTTP、跨域和安全

- API 使用 HTTP/1.1 风格的 `GET`、`POST`、`OPTIONS`；
- JSON 接口使用 UTF-8，POST 请求应发送 `Content-Type: application/json`；
- POST JSON 必须是对象，最大请求体为 2 MiB；
- 服务已返回 `Access-Control-Allow-Origin: *`，局域网中的其他网页可以跨域调用；
- 当前接口没有账号、令牌或权限分级，因此只能直接部署在可信局域网；
- 不得把 8080 直接映射到公网。跨公网使用时必须在反向代理上增加 HTTPS、身份认证、访问
  白名单和限流；
- 如果调用网页自身是 HTTPS，而机器人仍是 HTTP，浏览器可能以“混合内容”为由阻止请求。
  此时应让反向代理同时提供 HTTPS API，或在可信局域网中让调用网页也使用 HTTP。

当前服务不是 WebSocket/SSE 推送服务。“订阅网页数据”应理解为按合理频率轮询 GET 接口：

- 状态：`GET /api/status`，建议 1 Hz；
- RGB：`GET /api/preview/rgb?t=时间戳`，建议 2～5 Hz；
- 路径：建议 1～2 Hz；
- 地图几何图层：地图加载后读取一次，不要高频重复拉取；
- 地图清单：建议 10～30 秒一次；
- 手动速度：按住控制时必须约 10 Hz 连续 POST，松开立即 POST `/api/stop`。

## 3. 通用调用规则

以下示例统一使用：

```bash
ROBOT=http://192.168.137.20:8080
```

GET 示例：

```bash
curl --fail --silent --show-error "$ROBOT/api/status" | python3 -m json.tool
```

POST 示例：

```bash
curl --fail --silent --show-error \
  -X POST -H 'Content-Type: application/json' \
  -d '{"map_id":"map037"}' \
  "$ROBOT/api/navigation/localize"
```

成功 JSON 一定包含：

```json
{"ok": true}
```

失败 JSON 的统一结构为：

```json
{"ok": false, "error": "失败原因"}
```

常见 HTTP 状态码：

| 状态码 | 含义 |
|---|---|
| `200` | 请求完成 |
| `202` | 异步任务已接受，需要继续轮询 `/api/status` |
| `206` | 地图文件 Range 分段下载成功 |
| `400` | JSON、参数、地图编号或前置条件格式错误 |
| `404` | 资源不存在、图层不存在或 RGB 尚不可用 |
| `409` | 当前状态冲突，例如未定位、地图未加载、硬件服务不可用 |
| `413` | JSON 请求体超过 2 MiB |
| `416` | 下载 Range 超出文件范围 |
| `423` | 运动被急停、导航、机器人状态或另一个控制客户端锁定 |
| `500` | 服务内部或子进程停止失败 |

客户端必须同时检查 HTTP 状态码和 JSON 中的 `ok`，不能只判断网络请求是否返回。

## 4. 推荐的完整业务流程

### 4.1 只查看和下载地图

1. `GET /api/maps` 获取 map 列表；
2. 选择一个 `mapNNN`，优先使用记录中的 `default_variant`；
3. `POST /api/maps/mapNNN/preview` 加载显示图层；
4. 分别 GET `cloud`、`voxels`、`costmap`、`obstacles`；
5. 客户端在 Canvas/WebGL/Three.js 中组合显示；拖动、旋转和缩放属于客户端视图操作，不需要
   再向机器人发送 API；
6. 使用 `files[].download_url` 独立下载 DB、PLY、BT 等文件。

### 4.2 启动定位并发送导航目标

1. `POST /api/camera/start`；
2. 轮询 `/api/status`，等待 `camera.ready == true`；
3. `POST /api/robot/control` 开启机器人，等待
   `robot_control.control_ready == true`；
4. 机器人放在水平面并保持静止，`POST /api/imu/calibrate`；
5. 等待 `imu_calibration.state == "calibrated"`；
6. 加载地图预览；
7. `POST /api/navigation/localize`；
8. 等待 `navigation.planning_localization_ready == true` 且
   `navigation.planner_map_ready == true`；
9. `POST /api/navigation/goal` 设置地图坐标和最终朝向；
10. 等待 `navigation.path_ready == true`；
11. `POST /api/navigation/start`；
12. 持续轮询状态和路径。到达后 `task_state/follower_state` 会更新，旧路径会清空。

不要跳过相机、机器人反馈和 IMU 校准状态检查。所有运动命令都应由有人监护的上层程序发出。

## 5. API 总表

### 5.1 发现、状态和地图

| 方法 | 路径 | 作用 |
|---|---|---|
| GET | `/api` 或 `/api/capabilities` | 返回 API 版本、操作清单和可下载图层名 |
| GET | `/api/status` | 返回机器人、相机、IMU、建图、定位、导航和控制状态 |
| GET | `/api/maps` | 返回按 map 分组的地图清单、文件和导航摘要 |
| GET | `/api/maps/{map_id}` | 返回一个地图的公开信息 |
| POST | `/api/maps/{map_id}/preview` | 加载该地图的网页显示图层 |
| GET | `/api/maps/{map_id}/preview/cloud` | 彩色点云 |
| GET | `/api/maps/{map_id}/preview/voxels` | OctoMap 体素 |
| GET | `/api/maps/{map_id}/preview/terrain` | 可通行面、代价、静态障碍聚合层 |
| GET | `/api/maps/{map_id}/preview/costmap` | 独立代价地图层 |
| GET | `/api/maps/{map_id}/preview/obstacles` | 独立静态障碍层 |
| GET | `/api/maps/{map_id}/preview/path` | 当前有效规划路径 |
| GET | `/api/maps/{map_id}/download/{layer}` | 独立下载地图文件，支持 Range |

### 5.2 相机、机器人和系统

| 方法 | 路径 | 作用 |
|---|---|---|
| GET | `/api/preview/rgb` | 返回 JPEG/PNG 二进制 RGB 帧 |
| GET | `/api/preview/cloud` | 开发者建图模式的实时稀疏点云 |
| POST | `/api/camera/start` | 启动或切换相机 |
| POST | `/api/camera/stop` | 关闭网页托管的相机 |
| POST | `/api/robot/control` | 开启/关闭机器人网页控制 |
| POST | `/api/robot/height` | 设置已验证双足 LQR 模式的腿高等级 |
| POST | `/api/imu/calibrate` | 开始静止水平校准 |
| POST | `/api/estop` | 设置/解除 HTTP 急停锁存 |
| POST | `/api/system/restart` | 安全关闭全部服务并重新拉起网页 |

### 5.3 建图、定位和导航

| 方法 | 路径 | 作用 |
|---|---|---|
| POST | `/api/mapping/start` | 启动新地图建图 |
| POST | `/api/mapping/stop` | 停止建图并等待保存 |
| POST | `/api/navigation/localize` | 使用已预览地图启动 HLoc + ICP 定位 |
| POST | `/api/navigation/stop` | 关闭整个定位/导航进程，保留网页已加载地图 |
| POST | `/api/navigation/goal` | 设置带最终 yaw 的导航目标 |
| POST | `/api/navigation/start` | 执行已生成的安全路径 |
| POST | `/api/navigation/halt` | 停止任务、清空路径，但保持定位 |
| POST | `/api/navigation/home/set` | 设置当前地图的返航点 |
| POST | `/api/navigation/home/return` | 取消当前任务并自动规划返航 |

### 5.4 手动遥控和语义标注

| 方法 | 路径 | 作用 |
|---|---|---|
| POST | `/api/cmd_vel` | WASD/轮盘连续速度心跳 |
| POST | `/api/stop` | 释放本客户端速度租约并停车 |
| GET | `/api/semantic/annotations?map_id={map_id}` | 读取岩石、墙、坑标注 |
| POST | `/api/semantic/save` | 校验并保存语义标注 |

## 6. 地图接口详细说明

### 6.1 查询地图清单

```text
GET /api/maps
```

简化响应示例：

```json
{
  "ok": true,
  "maps": [{
    "id": "map037",
    "default_variant": "filtered",
    "convertible": true,
    "loadable": true,
    "localizable": true,
    "filtered_loadable": true,
    "filtered_localizable": true,
    "api": {
      "detail": "/api/maps/map037",
      "preview": "/api/maps/map037/preview",
      "cloud": "/api/maps/map037/preview/cloud",
      "voxels": "/api/maps/map037/preview/voxels",
      "terrain": "/api/maps/map037/preview/terrain",
      "costmap": "/api/maps/map037/preview/costmap",
      "obstacles": "/api/maps/map037/preview/obstacles",
      "path": "/api/maps/map037/preview/path"
    },
    "files": [{
      "layer": "database",
      "filename": "map037.db",
      "size_bytes": 123456,
      "modified_at": "2026-08-19T08:00:00Z",
      "download_url": "/api/maps/map037/download/database"
    }]
  }],
  "navigation": {}
}
```

字段含义：

- `convertible`：存在 `.db`，可尝试生成预览；
- `loadable`：原始 PLY/BT 图层齐全；
- `localizable`：原始图层和 HLoc 索引齐全；
- `filtered_loadable`：过滤版 PLY/BT 齐全；
- `filtered_localizable`：过滤版图层和 HLoc 索引齐全；
- `default_variant`：外部客户端默认应选的版本；存在完整过滤版时为 `filtered`；
- `files`：只列出实际存在并允许下载的文件，不返回服务器绝对路径。

地图 ID 只接受 `map` 加数字，例如 `map037`。不能传任意文件路径。

### 6.2 加载地图预览

```text
POST /api/maps/map037/preview
Content-Type: application/json

{}
```

空对象表示按 `default_variant` 加载。显式选择原始版：

```json
{"filtered": false}
```

显式选择过滤版：

```json
{"filtered": true}
```

成功响应包含 `map_id` 和 `map_variant`。调用可能需要读取大文件或从 `.db` 转换 PLY/BT，
最长可能持续数分钟，因此调用端应显示“地图加载中”，并把该请求超时设为至少 360 秒。
同一时刻只能进行一个地图转换/加载操作。

必须先成功 POST 预览，再 GET 对应 map 的各个预览图层。如果读取的 map 不是当前已加载 map，
接口返回 `409`，而不是错误地返回另一张地图的数据。

### 6.3 彩色点云

```text
GET /api/maps/map037/preview/cloud
```

响应主体：

```json
{
  "ok": true,
  "cloud": {
    "map_id": "map037",
    "variant": "filtered",
    "point_count": 2,
    "error": null,
    "points": [[1.0, 2.0, 0.1, 255, 120, 30]]
  }
}
```

`points` 每项为 `[x, y, z, r, g, b]`，坐标单位为米，颜色为 `0～255`。

### 6.4 OctoMap 体素

```text
GET /api/maps/map037/preview/voxels
```

`points` 每项为 `[x, y, z, size]`，`size` 是体素边长，单位为米。响应同时包含
`frame_id`、`resolution`、`point_count`、`age_seconds` 和可能的 `error`。

### 6.5 静态代价地图

```text
GET /api/maps/map037/preview/costmap
```

响应示例：

```json
{
  "ok": true,
  "costmap": {
    "map_id": "map037",
    "variant": "filtered",
    "resolution": 0.05,
    "robot_radius": 0.25,
    "costmap_margin": 0.35,
    "point_count": 1,
    "error": null,
    "points": [[1.0, 2.0, 0.1, 0.75]]
  }
}
```

`points` 每项为 `[x, y, z, cost]`。`cost` 已限制在 `0～1`：越接近 0 表示边缘代价越低，
越接近 1 表示越靠近不可通行边界、规划代价越高。它不是传统二维栅格图片，而是带地表高度
的三维点集；调用端可按 cost 做绿→黄→橙/红色映射。

### 6.6 静态障碍物

```text
GET /api/maps/map037/preview/obstacles
```

响应示例：

```json
{
  "ok": true,
  "obstacles": {
    "map_id": "map037",
    "variant": "filtered",
    "resolution": 0.05,
    "obstacle_min_height": 0.15,
    "point_count": 1,
    "error": null,
    "points": [[3.0, 4.0, 0.2]]
  }
}
```

`points` 每项为 `[x, y, z]`。这些点是保存 PLY 根据地面支撑、高度和法向分割得到的静态
障碍，不是把 BT 中所有占据体素都直接当成障碍。

### 6.7 聚合地形层

```text
GET /api/maps/map037/preview/terrain
```

主要字段：

```json
{
  "traversable_count": 100,
  "obstacle_count": 20,
  "traversable_points": [[1.0, 2.0, 0.1, 0.25]],
  "obstacle_points": [[3.0, 4.0, 0.2]]
}
```

其中 `traversable_points` 就是代价地图点。希望一次请求获得两层时使用 `terrain`；希望前端
模块独立订阅图层时使用 `costmap` 和 `obstacles`。

### 6.8 路径

```text
GET /api/maps/map037/preview/path
```

`points` 每项为 `[x, y, z]`。重要字段包括：

- `valid`：当前是否有可显示路径；
- `point_count`：当前返回点数；
- `planning_state`：`pending`、`ready`、`failed`、`map_ready` 等；
- `error`：规划失败原因；
- `stale`：当前实现固定为 `false`，任务结束后直接清空旧路径。

停止任务、到达目标、定位恢复失败或导航异常后，接口返回空路径，不应继续显示旧路线。

### 6.9 文件下载

允许的 `{layer}`：

| layer | 文件含义 |
|---|---|
| `database` | RTAB-Map `.db` |
| `cloud` | 原始彩色 `.ply` |
| `octomap` | 原始 `.bt` |
| `filtered_cloud` | 过滤版 `.ply` |
| `filtered_octomap` | 过滤版 `.bt` |
| `annotations` | 语义 `annotations.json` |
| `hloc_metadata` | HLoc `metadata.yaml` |

必须以 `/api/maps` 中实际返回的 `files[].download_url` 为准；缺失的层返回 404。

普通下载：

```bash
curl --fail -OJ "$ROBOT/api/maps/map037/download/cloud"
```

断点续传：

```bash
curl --fail -C - -OJ "$ROBOT/api/maps/map037/download/database"
```

服务返回 `Accept-Ranges: bytes`、`Content-Disposition` 和正确的 `Content-Range`。只支持单个
字节范围，不支持一个请求携带多个 ranges。

## 7. 状态接口

```text
GET /api/status
```

顶层重要字段：

- `state`：HTTP 手动控制状态，常见为 `idle/moving/timeout/estop/disabled`；
- `access_urls`：当前可访问 URL；
- `command_timeout`、`command`、`limits`：速度看门狗、当前命令和上限；
- `robot_control`：机器人开关、姿态、控制反馈、双电池和腿高；
- `imu_calibration`：校准状态、服务可用性、是否忙、能否开始；
- `camera`：相机 profile、进程状态、发布者数量、是否 ready；
- `mapping`：建图进程状态；
- `navigation`：定位、地图、路径、任务、返航和机器人位姿；
- `preview`：RGB/点云预览是否有数据。

导航最常用的判断字段：

| 字段 | 用途 |
|---|---|
| `navigation.state` | 定位/导航进程是否 `running` |
| `navigation.map_id` / `map_variant` | 当前地图和版本 |
| `navigation.localization_stage` | `stopped/searching/refining/localized` |
| `navigation.planning_localization_ready` | 是否有最新且经 ICP 验证的定位 |
| `navigation.planner_map_ready` | 规划器地形地图是否就绪 |
| `navigation.pose` | 当前地图坐标 `{x,y,z,yaw,source}` |
| `navigation.path_ready` | 是否存在可执行路径 |
| `navigation.active` | 路径跟随是否活动 |
| `navigation.follower_state` | 跟随器状态 |
| `navigation.task_state` | 高层任务状态 |
| `navigation.home` | 当前返航点或 `null` |
| `navigation.return_home_pending` | 是否正在等待返航路径/启动 |
| `navigation.planning_error` | 规划错误或 `null` |
| `navigation.localization_health` | 定位健康状态 |

相机启动、机器人站立、IMU 校准、定位和重启均为异步流程。收到 202 只代表任务已接受，
必须以 `/api/status` 中的反馈为准。

## 8. 相机、RGB、机器人和 IMU

### 8.1 相机

```text
POST /api/camera/start
{"profile":"zedx"}
```

可用 profile 从 `status.camera.profiles` 获取，当前配置为：`d455`、`d435i`、`hik`、
`zedx`。默认值仍为 `d455`；`zedx` 显示为 `ZED X + ZED Link Duo`。

关闭：

```text
POST /api/camera/stop
{}
```

切换相机会先停止运动、建图和定位。网页只管理由自己启动的相机；检测到外部相机进程时会
拒绝接管，以免杀死未知进程。

### 8.2 RGB 图像

```text
GET /api/preview/rgb
```

该接口返回 JPEG 或 PNG 二进制，不返回 JSON。相机尚无图像时返回 404。

HTML 可直接显示：

```html
<img id="robot-rgb" alt="机器人 RGB">
<script>
  const base = "http://192.168.137.20:8080";
  setInterval(() => {
    document.querySelector("#robot-rgb").src =
      `${base}/api/preview/rgb?t=${Date.now()}`;
  }, 300);
</script>
```

### 8.3 机器人控制和电量

开启：

```text
POST /api/robot/control
{"active":true}
```

安全关闭并趴下/释放控制：

```text
POST /api/robot/control
{"active":false}
```

接口返回 202 后轮询：

```text
robot_control.transitioning == false
robot_control.control_ready == true
```

电量读取：

```text
status.robot_control.battery.online
status.robot_control.battery.percentage
status.robot_control.battery.packs[]
```

双电池总百分比取当前在线电池包的较低值。离线时百分比为 `null`，调用端不能把它显示为 0%。

腿高（只允许反馈确认的双足 LQR 模式）：

```text
POST /api/robot/height
{"height":4.5}
```

当前配置范围为 0～9，但调用端应读取 `status.robot_control.body_height.minimum/maximum`，不要
硬编码。其他形态会返回 409。

### 8.4 IMU 校准

```text
POST /api/imu/calibrate
{}
```

前置条件：机器人站立并完全静止、带 IMU 的相机链已启动、没有建图或导航进程在运行。
轮询 `status.imu_calibration`：

- `service_available`：校准服务是否在线；
- `can_start`：当前是否允许发起；
- `busy`：正在等待静止或采样；
- `state`：等待、采样、`calibrated` 或 `failed`；
- `message`：用户提示；
- `sample_count`：已采样数量。

校准期间不得触碰机器人。

## 9. 定位、目标、返航和任务控制

所有坐标均在所选地图的 `map` 坐标系中，单位为米；`yaw` 使用弧度，0 指向 +X，`π/2`
指向 +Y。后端会把 yaw 归一化到 `[-π, π]`。

### 9.1 启动定位

必须先加载同一张地图的预览：

```text
POST /api/navigation/localize
{"map_id":"map037"}
```

要求地图具有 DB、所选版本 PLY/BT 和 HLoc 索引。收到 202 后等待：

```text
navigation.state == "running"
navigation.planning_localization_ready == true
navigation.planner_map_ready == true
```

### 9.2 设置导航目标

```text
POST /api/navigation/goal
{"x":1.2,"y":0.8,"z":0.0,"yaw":1.57}
```

`x/y/z/yaw` 必须是有限数字。省略的字段当前会取 0，但交付应用应全部显式发送，避免把目标
意外设置到原点。网页从 `costmap.points` 选取可通行位置，并将其中的 z 作为目标高度；外部
程序也应使用相同方法，不应只在俯视图中任意猜测 z。

设置目标只触发规划，不立即运动。等待 `navigation.path_ready == true` 后再执行：

```text
POST /api/navigation/start
{}
```

### 9.3 停止任务但保持定位

```text
POST /api/navigation/halt
{}
```

这是用户页面“停止任务”的接口：停止路径跟随，清空当前路径和旧路径缓存，但保持定位与地图
进程。之后可以进行手动遥控，再选新目标继续导航。

完全关闭定位/导航进程使用：

```text
POST /api/navigation/stop
{}
```

两者含义不同，不要混用。

### 9.4 返航点

设置返航姿态：

```text
POST /api/navigation/home/set
{"x":0.0,"y":0.0,"z":0.0,"yaw":0.0}
```

返航点由当前导航任务管理器保存，同一台机器人上的多个网页会看到同一个 `navigation.home`。
启动新的定位进程后应重新确认返航点。

自动返航：

```text
POST /api/navigation/home/return
{}
```

该操作会停车、取消现有任务，规划返航路线，并由高层任务管理器在安全路径就绪后自动出发。
调用端应轮询 `task_state`、`return_home_pending`、`active` 和 `follower_state`，不要自行绕过
状态机重复发送低层启动命令。

## 10. 手动遥控、租约和急停

### 10.1 连续速度心跳

```text
POST /api/cmd_vel
{
  "linear_x":0.10,
  "linear_y":0.0,
  "angular_z":0.30,
  "client_id":"tablet-control-0001"
}
```

`client_id` 建议每个页面/应用启动时生成一次，允许字符为字母、数字、点、下划线、冒号和
短横线，长度 8～128。它用于防止两台手机同时抢占速度控制。

服务器会按 `/api/status.limits` 自动截断速度。当前配置中 `linear_y` 上限为 0，因此不能横移。

按住 WASD 或轮盘时必须每 100 ms 左右重复发送。默认 `command_timeout` 为 0.8 秒；超过该
时间没有新心跳会自动发布零速度。因此只发一次命令会表现为“动一下就停”，这是安全看门狗
正常工作，不是底盘故障。

松开按键、页面失焦、页面关闭前应立即调用：

```text
POST /api/stop
{"client_id":"tablet-control-0001"}
```

只有租约持有者可以正常释放自己的运动。`{"force":true}` 仅供系统安全清理脚本使用，普通
客户端不应使用。

导航正在执行、急停已锁存、机器人未反馈可控或另一个客户端持有租约时，速度接口返回 423。

### 10.2 急停

锁存急停并立即发送零速度：

```text
POST /api/estop
{"active":true}
```

解除 HTTP 急停：

```text
POST /api/estop
{"active":false}
```

解除急停不等于自动恢复运动；必须重新确认机器人、定位、路径和现场安全后再发送命令。

## 11. 建图和语义标注

### 11.1 建图

```text
POST /api/mapping/start
{}
```

```text
POST /api/mapping/stop
{}
```

开始前相机链必须健康，且不能存在网页之外启动的冲突建图节点。响应中的
`mapping.require_robot_standing` 表示是否启用机器人站立门禁。当前测试配置为 `false`，
相机就绪后无需启动 D1 即可建图；这不会跳过 RGB-D/IMU 唯一发布者、IMU 校准或冲突节点
检查。机器人移动建图前应把 `mapping_require_robot_standing` 恢复为 `true`。停止接口会
等待托管进程退出和数据库保存，不能收到请求后立刻断电。

开发者模式实时点云：

```text
GET /api/preview/cloud
```

此接口返回有上限的实时点集。手动速度租约活动期间，为保证控制响应，大点云接口可能返回
409 `preview paused while manual control is active`。

### 11.2 读取语义标注

```text
GET /api/semantic/annotations?map_id=map037
```

没有保存文件时会按 OctoMap 建议地面高度返回一个空模板，`saved` 为 false。

### 11.3 保存语义标注

```text
POST /api/semantic/save
```

请求示例：

```json
{
  "schema_version":1,
  "map_id":"map037",
  "frame_id":"map",
  "ground":{"z":0.0,"minimum_height":0.15},
  "occupied_labels":[
    {"type":"rock","x":1.0,"y":2.0,"z":0.3,"size":0.05},
    {"type":"wall","x":1.1,"y":2.0,"z":0.4,"size":0.05}
  ],
  "pits":[{
    "id":"pit_001",
    "type":"pit",
    "depth":0.4,
    "polygon":[[0.0,0.0],[1.0,0.0],[0.0,1.0]]
  }]
}
```

服务会校验类型、有限数值、标注高度、体素范围、坑深度、坑 ID 唯一性以及多边形面积，再原子
保存规范化 JSON。`occupied_labels.type` 只允许 `rock` 或 `wall`；坑多边形至少 3 个点。

当前全局规划器明确把 `pit` 多边形作为禁行区域。`rock/wall` 标签用于保存和显示；几何障碍
规划仍以 PLY 地形分割出的静态障碍为准，因此不要只画标签而忽略实际 PLY/BT 地图质量。

## 12. 一键重启

```text
POST /api/system/restart
{"confirm":"restart_all_services"}
```

确认字符串必须完全一致。接口返回 202 后，当前 HTTP 连接会在服务关闭时中断。上层客户端应：

1. 立即停止发送所有运动请求；
2. 每 1～2 秒重试 `GET /api/status`；
3. 最多等待响应中的 `reconnect_timeout_seconds`（当前 120 秒）；
4. 服务恢复后重新读取全部状态，不沿用重启前的“相机已开/机器人已开/已校准”假设。

重启会先停车、停止导航/建图/相机并安全关闭机器人控制，再清理服务并重新启动网页。重启后
不会无人值守地让机器人自动站立或继续旧任务。

## 13. 外部程序完整示例

### 13.1 JavaScript：加载地图图层

```javascript
const base = "http://192.168.137.20:8080";

async function request(path, options = {}) {
  const response = await fetch(base + path, options);
  const result = await response.json();
  if (!response.ok || !result.ok) {
    throw new Error(result.error || `HTTP ${response.status}`);
  }
  return result;
}

const catalog = await request("/api/maps");
const map = [...catalog.maps].reverse()
  .find(item => item.filtered_loadable) ?? catalog.maps.at(-1);

await request(`/api/maps/${map.id}/preview`, {
  method: "POST",
  headers: {"Content-Type": "application/json"},
  body: JSON.stringify({filtered: map.default_variant === "filtered"}),
});

const [cloud, costmap, obstacles] = await Promise.all([
  request(`/api/maps/${map.id}/preview/cloud`),
  request(`/api/maps/${map.id}/preview/costmap`),
  request(`/api/maps/${map.id}/preview/obstacles`),
]);

console.log(cloud.cloud.points);
console.log(costmap.costmap.points);
console.log(obstacles.obstacles.points);
```

地图三维拖动不需要专门的后端接口。客户端保存自己的 `yaw/pitch/zoom/pan`，将 API 返回的
三维点投影到 Canvas/WebGL 即可。项目现有 `web/map_projection.js` 可作为投影参考。

### 13.2 JavaScript：发送目标并开始

```javascript
await request("/api/navigation/goal", {
  method: "POST",
  headers: {"Content-Type": "application/json"},
  body: JSON.stringify({x: 1.2, y: 0.8, z: 0.05, yaw: Math.PI / 2}),
});

for (;;) {
  const status = await request("/api/status");
  if (status.navigation.planning_error) {
    throw new Error(status.navigation.planning_error);
  }
  if (status.navigation.path_ready) break;
  await new Promise(resolve => setTimeout(resolve, 500));
}

await request("/api/navigation/start", {
  method: "POST",
  headers: {"Content-Type": "application/json"},
  body: "{}",
});
```

### 13.3 Python：通用客户端

```python
import requests

BASE = "http://192.168.137.20:8080"

def get(path):
    response = requests.get(BASE + path, timeout=10)
    response.raise_for_status()
    return response.json()

def post(path, body=None, timeout=20):
    response = requests.post(BASE + path, json=body or {}, timeout=timeout)
    result = response.json()
    if not response.ok or not result.get("ok"):
        raise RuntimeError(result.get("error", response.text))
    return result

maps = get("/api/maps")["maps"]
selected = maps[-1]
post(
    f"/api/maps/{selected['id']}/preview",
    {"filtered": selected["default_variant"] == "filtered"},
    timeout=360,
)
costmap = get(f"/api/maps/{selected['id']}/preview/costmap")
print(costmap["costmap"]["point_count"])
```

生产客户端还应为 GET 增加 JSON `ok` 检查、重试退避、取消机制和运动安全状态机。

## 14. 兼容接口

开发者旧页面仍使用以下接口，新项目优先使用 map-scoped 接口：

```text
GET  /api/navigation/maps
GET  /api/navigation/voxels
GET  /api/navigation/cloud
GET  /api/navigation/path
GET  /api/navigation/terrain
POST /api/navigation/load_map  {"map_id":"map037","filtered":true}
```

这些兼容接口返回“当前已加载地图”的图层，没有 URL 中的 map ID 校验。外部交付程序推荐使用
`/api/maps/{map_id}/preview/...`，这样不会把另一张已加载地图误当成请求目标。

## 15. 交付验证说明

本接口文档依据当前后端路由、用户网页和开发者网页逐项核对。自动化测试实际启动了本机临时
HTTP 服务和 ROS 2 节点，验证了：

- API 能力清单和网页资源；
- map 清单不泄露服务器绝对路径；
- DB/PLY/BT 文件白名单和 Range 断点下载；
- map-scoped 彩色点云、静态代价地图、静态障碍物接口；
- 速度请求到 ROS `Twist`、速度限幅、0.8 秒看门狗、控制租约、停车和急停；
- RGB 未就绪、相机 profile、机器人控制、腿高和 IMU 前置条件错误能够被正确拒绝；
- 电池 ROS 消息能够反映到 HTTP 状态；
- 地图/地形解析和导航控制状态的单元测试。

相机成像、D1 实际站立、真机 IMU 采样、真实路线运动和动态绕障属于硬件/现场验收，不能用
无人值守自动化测试代替。接口在这些操作上已验证路由、参数、状态机和失败响应，但交付时仍应
按“有人监护、低速、短距离、先急停验证”的顺序完成现场验收。

运行时可通过下面的机器可读接口确认部署版本包含本文操作：

```bash
curl --fail --silent "$ROBOT/api/capabilities" | python3 -m json.tool
```

当前 `version` 应为 `2`，并应包含：

```text
/api/maps/{map_id}/preview/costmap
/api/maps/{map_id}/preview/obstacles
```
