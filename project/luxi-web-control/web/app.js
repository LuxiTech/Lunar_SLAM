"use strict";

const $ = (selector) => document.querySelector(selector);
const mapProjection = window.LuxiMapProjection;
const connection = $("#connection");
const connectionText = $("#connectionText");
const topic = $("#topic");
const subscribers = $("#subscribers");
const state = $("#state");
const commandValue = $("#commandValue");
const linearInput = $("#linearSpeed");
const angularInput = $("#angularSpeed");
const linearValue = $("#linearValue");
const angularValue = $("#angularValue");
const estopButton = $("#estopButton");
const releaseButton = $("#releaseButton");
const toast = $("#toast");
const joystickPad = $("#joystickPad");
const joystickKnob = $("#joystickKnob");
const joystickLinear = $("#joystickLinear");
const joystickAngular = $("#joystickAngular");
const mappingState = $("#mappingState");
const mappingDetail = $("#mappingDetail");
const mappingStartButton = $("#mappingStartButton");
const mappingStopButton = $("#mappingStopButton");
const rgbPreview = $("#rgbPreview");
const rgbPreviewState = $("#rgbPreviewState");
const rgbPreviewHint = $("#rgbPreviewHint");
const cloudPreview = $("#cloudPreview");
const cloudPreviewState = $("#cloudPreviewState");
const cloudPreviewHint = $("#cloudPreviewHint");
const navigationState = $("#navigationState");
const navigationDetail = $("#navigationDetail");
const navigationMapSelect = $("#navigationMapSelect");
const navigationLoadButton = $("#navigationLoadButton");
const navigationLocateButton = $("#navigationLocateButton");
const navigationStopButton = $("#navigationStopButton");
const navigationGoalButton = $("#navigationGoalButton");
const voxelMapCanvas = $("#voxelMapCanvas");
const voxelMapHint = $("#voxelMapHint");
const navigationShowCloud = $("#navigationShowCloud");
const navigationShowVoxels = $("#navigationShowVoxels");
const navigationShowMappingOrigin = $("#navigationShowMappingOrigin");
const navigationShowRobot = $("#navigationShowRobot");

const held = new Set();
let estopActive = false;
let online = false;
let toastTimer = null;
let commandRequestPending = false;
let joystickPointerId = null;
let joystickX = 0;
let joystickY = 0;
let rgbRefreshPending = false;
let rgbObjectUrl = null;
let cloudRefreshPending = false;
let navigationMapsRefreshPending = false;
let navigationLoadPending = false;
let navigationLocatePending = false;
let voxelRefreshPending = false;
let navigationCloudRefreshPending = false;
let voxelViewport = null;
let selectedGoal = null;
let navigationCloud = {};
let navigationVoxels = {};
let navigationPath = {};
let navigationView = null;
let navigationDrag = null;
let navigationPinch = null;
const navigationPointers = new Map();
let navigationGoalMode = false;
let navigationMapRecords = new Map();
let navigationPose = null;

const keyActions = {
  KeyW: "forward",
  ArrowUp: "forward",
  KeyS: "backward",
  ArrowDown: "backward",
  KeyA: "left",
  ArrowLeft: "left",
  KeyD: "right",
  ArrowRight: "right",
};

function showToast(message) {
  toast.textContent = message;
  toast.classList.add("show");
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => toast.classList.remove("show"), 2200);
}

async function api(path, body = {}, options = {}) {
  const response = await fetch(path, {
    method: "POST",
    headers: {"Content-Type": "application/json"},
    body: JSON.stringify(body),
    cache: "no-store",
    keepalive: options.keepalive || false,
  });
  const result = await response.json().catch(() => ({}));
  if (!response.ok) {
    throw new Error(result.error || `HTTP ${response.status}`);
  }
  return result;
}

function currentCommand() {
  const linear = Number(linearInput.value);
  const angular = Number(angularInput.value);
  if (joystickPointerId !== null) {
    return {
      linear_x: -joystickY * linear,
      linear_y: 0,
      angular_z: -joystickX * angular,
    };
  }
  let linearX = 0;
  let angularZ = 0;
  if (held.has("forward")) linearX += linear;
  if (held.has("backward")) linearX -= linear;
  if (held.has("left")) angularZ += angular;
  if (held.has("right")) angularZ -= angular;
  return {linear_x: linearX, linear_y: 0, angular_z: angularZ};
}

function updateReadout(command = currentCommand()) {
  commandValue.textContent = `x ${command.linear_x.toFixed(2)} · yaw ${command.angular_z.toFixed(2)}`;
  joystickLinear.value = command.linear_x.toFixed(2);
  joystickAngular.value = command.angular_z.toFixed(2);
}

function controlActive() {
  return held.size > 0 || joystickPointerId !== null;
}

async function sendCommand() {
  if (!controlActive() || estopActive || commandRequestPending) return;
  commandRequestPending = true;
  const command = currentCommand();
  updateReadout(command);
  try {
    await api("/api/cmd_vel", command);
  } catch (error) {
    if (!String(error.message).includes("emergency stop")) showToast(`控制失败：${error.message}`);
  } finally {
    commandRequestPending = false;
  }
}

function stop(options = {}) {
  held.clear();
  joystickPointerId = null;
  joystickX = 0;
  joystickY = 0;
  updateJoystickKnob();
  updateReadout({linear_x: 0, angular_z: 0});
  api("/api/stop", {}, {keepalive: options.keepalive}).catch(() => {});
}

function beginAction(action) {
  if (estopActive) {
    showToast("请先解除急停");
    return;
  }
  joystickPointerId = null;
  joystickX = 0;
  joystickY = 0;
  updateJoystickKnob();
  held.add(action);
  updateReadout();
  sendCommand();
}

function endAction(action) {
  held.delete(action);
  updateReadout();
  if (held.size > 0) sendCommand();
  else stop();
}

function updateJoystickKnob() {
  const padRadius = joystickPad.clientWidth * 0.5;
  const knobRadius = joystickKnob.clientWidth * 0.5;
  const travel = Math.max(1, padRadius - knobRadius - 5);
  joystickKnob.style.transform =
    `translate(calc(-50% + ${joystickX * travel}px), ` +
    `calc(-50% + ${joystickY * travel}px))`;
  const active = joystickPointerId !== null;
  joystickKnob.classList.toggle("active", active);
  joystickPad.classList.toggle("active", active);
}

function setJoystickFromPointer(event) {
  const rect = joystickPad.getBoundingClientRect();
  const radius = Math.max(1, Math.min(rect.width, rect.height) * 0.5);
  const centerX = rect.left + rect.width * 0.5;
  const centerY = rect.top + rect.height * 0.5;
  let x = (event.clientX - centerX) / radius;
  let y = (event.clientY - centerY) / radius;
  const length = Math.hypot(x, y);
  if (length > 1) {
    x /= length;
    y /= length;
  }

  const deadzone = 0.07;
  if (length < deadzone) {
    joystickX = 0;
    joystickY = 0;
  } else {
    const limitedLength = Math.min(1, length);
    const scaledLength = (limitedLength - deadzone) / (1 - deadzone);
    const scale = scaledLength / limitedLength;
    joystickX = x * scale;
    joystickY = y * scale;
  }
  updateJoystickKnob();
  updateReadout();
  sendCommand();
}

joystickPad.addEventListener("pointerdown", (event) => {
  event.preventDefault();
  if (estopActive) {
    showToast("请先解除急停");
    return;
  }
  held.clear();
  joystickPointerId = event.pointerId;
  joystickPad.setPointerCapture?.(event.pointerId);
  setJoystickFromPointer(event);
});

joystickPad.addEventListener("pointermove", (event) => {
  if (joystickPointerId !== event.pointerId) return;
  event.preventDefault();
  setJoystickFromPointer(event);
});

function endJoystick(event) {
  if (joystickPointerId !== event.pointerId) return;
  if (joystickPad.hasPointerCapture?.(event.pointerId)) {
    joystickPad.releasePointerCapture(event.pointerId);
  }
  stop();
}

joystickPad.addEventListener("pointerup", endJoystick);
joystickPad.addEventListener("pointercancel", endJoystick);
joystickPad.addEventListener("lostpointercapture", (event) => {
  if (joystickPointerId === event.pointerId) stop();
});

document.addEventListener("keydown", (event) => {
  if (event.code === "Space") {
    event.preventDefault();
    if (!event.repeat) activateEstop();
    return;
  }
  const action = keyActions[event.code];
  if (!action || event.repeat || ["INPUT", "TEXTAREA"].includes(event.target.tagName)) return;
  event.preventDefault();
  beginAction(action);
});

document.addEventListener("keyup", (event) => {
  const action = keyActions[event.code];
  if (!action) return;
  event.preventDefault();
  endAction(action);
});

$("#stopButton").addEventListener("pointerdown", (event) => {
  event.preventDefault();
  stop();
});

function updateSpeeds() {
  linearValue.value = `${Number(linearInput.value).toFixed(2)} m/s`;
  angularValue.value = `${Number(angularInput.value).toFixed(2)} rad/s`;
  updateReadout();
  if (controlActive()) sendCommand();
}
linearInput.addEventListener("input", updateSpeeds);
angularInput.addEventListener("input", updateSpeeds);

async function activateEstop() {
  stop();
  try {
    await api("/api/estop", {active: true});
    setEstopUi(true);
    showToast("急停已锁定");
  } catch (error) {
    showToast(`急停请求失败：${error.message}`);
  }
}

async function releaseEstop() {
  try {
    await api("/api/estop", {active: false});
    setEstopUi(false);
    showToast("急停已解除，小车仍保持停止");
  } catch (error) {
    showToast(`解除失败：${error.message}`);
  }
}

function setEstopUi(active) {
  estopActive = active;
  estopButton.hidden = active;
  releaseButton.hidden = !active;
  if (active) stop();
}
estopButton.addEventListener("click", activateEstop);
releaseButton.addEventListener("click", releaseEstop);

function setConnection(isOnline) {
  online = isOnline;
  connection.classList.toggle("online", isOnline);
  connection.classList.toggle("offline", !isOnline);
  connectionText.textContent = isOnline ? "控制服务在线" : "连接已断开";
}

const stateNames = {
  disabled: "输出已禁用",
  idle: "已停止",
  moving: "运动中",
  timeout: "超时停车",
  estop: "急停锁定",
};

const mappingStateNames = {
  disabled: "不可用",
  stopped: "未启动",
  running: "建图中",
  failed: "启动失败",
};

const navigationStateNames = {
  disabled: "不可用",
  stopped: "未加载",
  running: "定位/规划中",
  failed: "启动失败",
};

function updateNavigation(navigation) {
  if (!navigation) return;
  const name = navigationStateNames[navigation.state] || navigation.state;
  navigationState.textContent = name;
  navigationState.className = `preview-state ${navigation.state === "running" ? "live" : ""}`;
  const selectedMap = navigationMapRecords.get(navigationMapSelect.value);
  navigationLoadButton.disabled = navigationLoadPending || !selectedMap?.convertible;
  navigationMapSelect.disabled = navigationLoadPending;
  navigationLocateButton.disabled =
    navigationLoadPending || navigationLocatePending ||
    navigation.state === "running" ||
    !selectedMap?.localizable ||
    navigationCloud.map_id !== selectedMap?.id;
  navigationStopButton.disabled = !navigation.enabled || navigation.state !== "running";
  navigationGoalButton.disabled = !navigation.localization_ready;
  navigationPose = navigation.pose || null;
  if (navigationLoadPending) {
    drawNavigationMap(navigationVoxels, navigationPath, navigationCloud);
    return;
  } else if (navigation.last_error) {
    navigationDetail.textContent = navigation.last_error;
  } else if (navigation.state === "running") {
    const mapName = navigation.map_id || "所选地图";
    const pose = navigation.pose;
    const poseText = pose
      ? ` x=${pose.x.toFixed(2)}m，y=${pose.y.toFixed(2)}m，yaw=${pose.yaw_degrees.toFixed(1)}°`
      : "";
    if (navigation.localization_stage === "localized") {
      const fitness = navigation.localization_fitness == null
        ? "" : `，fitness=${Number(navigation.localization_fitness).toFixed(3)}`;
      navigationDetail.textContent =
        `${mapName} 已完成 HLoc 粗定位和 ICP 精定位：${poseText}${fitness}。`;
    } else if (navigation.localization_stage === "refining") {
      navigationDetail.textContent =
        `${mapName} 已找到 HLoc 全局候选，正在进行 ICP 精配准：${poseText}。`;
    } else {
      navigationDetail.textContent =
        `正在 ${mapName} 中进行全局粗定位；请缓慢移动或转动机器人。`;
    }
    if (navigation.map_id && navigationCloud.map_id !== navigation.map_id) {
      refreshNavigationCloud();
    }
  } else if (!navigation.enabled) {
    navigationDetail.textContent = "定位与规划未启用；仍可加载并查看保存的两种地图图层。";
  } else {
    navigationDetail.textContent =
      "地图图层加载完成后，点击“自动定位”并缓慢移动或转动机器人。";
  }
  drawNavigationMap(navigationVoxels, navigationPath, navigationCloud);
}

function updateNavigationMaps(maps) {
  const previous = navigationMapSelect.value;
  navigationMapRecords = new Map(maps.map((item) => [item.id, item]));
  navigationMapSelect.replaceChildren();
  for (const item of maps) {
    const option = document.createElement("option");
    option.value = item.id;
    option.disabled = !item.convertible;
    option.textContent = item.loadable
      ? `${item.id}${item.cloud_path ? "（完整彩色点云 + 体素地图" : "（体素地图；未导出彩色点云"}${item.localizable ? " + HLoc）" : "；未构建 HLoc）"}`
      : item.convertible
        ? `${item.id}（选择后自动转换）`
        : `${item.id}（缺少 .db，无法转换）`;
    navigationMapSelect.append(option);
  }
  if (!maps.length) {
    const option = document.createElement("option");
    option.textContent = "未发现可用地图";
    option.value = "";
    navigationMapSelect.append(option);
  }
  if (Array.from(navigationMapSelect.options).some((option) => option.value === previous)) {
    navigationMapSelect.value = previous;
  } else {
    const firstConvertible = maps.find((item) => item.convertible);
    navigationMapSelect.value = firstConvertible ? firstConvertible.id : "";
  }
}

async function refreshNavigationMaps() {
  if (navigationMapsRefreshPending) return;
  navigationMapsRefreshPending = true;
  try {
    const response = await fetch("/api/navigation/maps", {cache: "no-store"});
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    const result = await response.json();
    updateNavigationMaps(Array.isArray(result.maps) ? result.maps : []);
    updateNavigation(result.navigation);
  } catch (_error) {
    navigationDetail.textContent = "无法读取地图目录。";
  } finally {
    navigationMapsRefreshPending = false;
  }
}

function canvasMetrics(canvas) {
  const rect = canvas.getBoundingClientRect();
  const width = Math.max(1, Math.round(rect.width));
  const height = Math.max(1, Math.round(rect.height));
  const ratio = Math.max(1, window.devicePixelRatio || 1);
  if (canvas.width !== width * ratio || canvas.height !== height * ratio) {
    canvas.width = width * ratio;
    canvas.height = height * ratio;
  }
  const context = canvas.getContext("2d");
  context.setTransform(ratio, 0, 0, ratio, 0, 0);
  return {context, width, height, rect};
}

function drawNavigationMap(voxels, path, cloud) {
  const {context, width, height} = canvasMetrics(voxelMapCanvas);
  context.fillStyle = "#080d13";
  context.fillRect(0, 0, width, height);
  const points = Array.isArray(voxels.points) ? voxels.points : [];
  const pathPoints = Array.isArray(path.points) ? path.points : [];
  const cloudPoints = Array.isArray(cloud.points) ? cloud.points : [];
  const mapHasGeometry = Boolean(cloudPoints.length || points.length || pathPoints.length);
  const mappingOrigin = [0, 0, 0];
  const all = cloudPoints.concat(
    points,
    pathPoints,
    selectedGoal ? [[selectedGoal.x, selectedGoal.y, 0]] : [],
    mapHasGeometry && navigationShowMappingOrigin.checked ? [mappingOrigin] : [],
    navigationPose && navigationShowRobot.checked
      ? [[navigationPose.x, navigationPose.y, navigationPose.z || 0]]
      : [],
  );
  if (!all.length) {
    voxelViewport = null;
    return;
  }
  let minX = Infinity;
  let maxX = -Infinity;
  let minY = Infinity;
  let maxY = -Infinity;
  let minZ = Infinity;
  let maxZ = -Infinity;
  for (const point of all) {
    const x = Number(point[0]);
    const y = Number(point[1]);
    const z = Number(point[2]) || 0;
    minX = Math.min(minX, x);
    maxX = Math.max(maxX, x);
    minY = Math.min(minY, y);
    maxY = Math.max(maxY, y);
    minZ = Math.min(minZ, z);
    maxZ = Math.max(maxZ, z);
  }
  const spanX = Math.max(0.5, maxX - minX);
  const spanY = Math.max(0.5, maxY - minY);
  const spanZ = Math.max(0.5, maxZ - minZ);
  const span = Math.max(spanX, spanY, spanZ);
  if (!navigationView) {
    navigationView = {...mapProjection.defaultView};
  }
  const scale = Math.min((width - 44) / span, (height - 44) / span) * navigationView.zoom;
  const centerX = (minX + maxX) * 0.5;
  const centerY = (minY + maxY) * 0.5;
  const centerZ = (minZ + maxZ) * 0.5;
  const center = [centerX, centerY, centerZ];
  voxelViewport = {
    minX, maxX, minY, maxY, minZ, maxZ, centerX, centerY, centerZ,
    scale, width, height, view: navigationView,
  };
  const toCanvas = (point) => {
    const {horizontal, vertical, depth} = mapProjection.projectMapPoint(
      point,
      center,
      navigationView,
    );
    return [width * 0.5 + horizontal * scale, height * 0.5 - vertical * scale, depth];
  };
  const drawCompassAxis = (point, color, label) => {
    const projected = mapProjection.projectMapPoint(
      point,
      [0, 0, 0],
      navigationView,
    );
    const dx = projected.horizontal;
    const dy = -projected.vertical;
    const length = Math.max(1e-6, Math.hypot(dx, dy));
    const originX = 48;
    const originY = height - 42;
    const tipX = originX + 28 * dx / length;
    const tipY = originY + 28 * dy / length;
    context.strokeStyle = color;
    context.fillStyle = color;
    context.lineWidth = 2;
    context.beginPath();
    context.moveTo(originX, originY);
    context.lineTo(tipX, tipY);
    context.stroke();
    context.font = "600 11px system-ui, sans-serif";
    context.fillText(label, tipX + 3, tipY - 3);
  };
  drawCompassAxis([1, 0, 0], "#ff7580", "+X 前");
  drawCompassAxis([0, 1, 0], "#65e3b5", "+Y 左");
  if (navigationShowCloud.checked) {
    for (const point of cloudPoints) {
      const [x, y] = toCanvas(point);
      const red = Math.max(0, Math.min(255, Number(point[3]) || 0));
      const green = Math.max(0, Math.min(255, Number(point[4]) || 0));
      const blue = Math.max(0, Math.min(255, Number(point[5]) || 0));
      context.fillStyle = `rgba(${red}, ${green}, ${blue}, .72)`;
      context.fillRect(x - 1, y - 1, 2, 2);
    }
  }
  if (navigationShowVoxels.checked) {
    context.fillStyle = "rgba(101, 227, 181, .54)";
    for (const point of points) {
      const [x, y] = toCanvas(point);
      const voxelSize = Math.max(1, Math.min(14, (Number(point[3]) || voxels.resolution || 0.1) * scale));
      context.fillRect(x - voxelSize * 0.5, y - voxelSize * 0.5, voxelSize, voxelSize);
    }
  }
  if (pathPoints.length) {
    context.strokeStyle = "#ffd166";
    context.lineWidth = 2.5;
    context.beginPath();
    pathPoints.forEach((point, index) => {
      const [x, y] = toCanvas(point);
      if (index) context.lineTo(x, y);
      else context.moveTo(x, y);
    });
    context.stroke();
  }
  if (selectedGoal) {
    const [x, y] = toCanvas([selectedGoal.x, selectedGoal.y]);
    context.strokeStyle = "#ff7580";
    context.lineWidth = 2;
    context.beginPath();
    context.arc(x, y, 6, 0, Math.PI * 2);
    context.stroke();
  }
  if (mapHasGeometry && navigationShowMappingOrigin.checked) {
    const [x, y] = toCanvas(mappingOrigin);
    context.save();
    context.strokeStyle = "#ff9f43";
    context.fillStyle = "#ff9f43";
    context.lineWidth = 2.5;
    context.beginPath();
    context.moveTo(x - 8, y);
    context.lineTo(x + 8, y);
    context.moveTo(x, y - 8);
    context.lineTo(x, y + 8);
    context.stroke();
    context.beginPath();
    context.arc(x, y, 4, 0, Math.PI * 2);
    context.fill();
    const label = "建图起点 (0.00, 0.00, 0.00)";
    context.font = "600 12px system-ui, sans-serif";
    const labelWidth = context.measureText(label).width;
    context.fillStyle = "rgba(8, 13, 19, .84)";
    context.fillRect(x + 11, y - 23, labelWidth + 10, 20);
    context.fillStyle = "#ffe0b2";
    context.fillText(label, x + 16, y - 9);
    context.restore();
  }
  if (navigationPose && navigationShowRobot.checked) {
    const robot = [
      navigationPose.x,
      navigationPose.y,
      navigationPose.z || 0,
    ];
    const heading = [
      robot[0] + 0.38 * Math.cos(navigationPose.yaw),
      robot[1] + 0.38 * Math.sin(navigationPose.yaw),
      robot[2],
    ];
    const [x, y] = toCanvas(robot);
    const [tipX, tipY] = toCanvas(heading);
    const dx = tipX - x;
    const dy = tipY - y;
    const length = Math.max(1, Math.hypot(dx, dy));
    const sideX = -dy / length;
    const sideY = dx / length;
    context.save();
    context.strokeStyle = "#c792ff";
    context.fillStyle = "#c792ff";
    context.lineWidth = 3;
    context.beginPath();
    context.arc(x, y, 6, 0, Math.PI * 2);
    context.fill();
    context.beginPath();
    context.moveTo(x, y);
    context.lineTo(tipX, tipY);
    context.stroke();
    context.beginPath();
    context.moveTo(tipX, tipY);
    context.lineTo(tipX - dx * 0.28 + sideX * 5, tipY - dy * 0.28 + sideY * 5);
    context.lineTo(tipX - dx * 0.28 - sideX * 5, tipY - dy * 0.28 - sideY * 5);
    context.closePath();
    context.fill();
    context.font = "600 12px system-ui, sans-serif";
    context.fillText("机器人", x + 10, y - 10);
    context.restore();
  }
}

async function refreshVoxelMap() {
  if (voxelRefreshPending) return;
  voxelRefreshPending = true;
  try {
    const [voxelResponse, pathResponse] = await Promise.all([
      fetch("/api/navigation/voxels", {cache: "no-store"}),
      fetch("/api/navigation/path", {cache: "no-store"}),
    ]);
    if (!voxelResponse.ok || !pathResponse.ok) throw new Error("preview unavailable");
    navigationVoxels = (await voxelResponse.json()).voxels || {};
    navigationPath = (await pathResponse.json()).path || {};
    drawNavigationMap(navigationVoxels, navigationPath, navigationCloud);
    voxelMapHint.classList.toggle(
      "hidden",
      Boolean((navigationVoxels.points || []).length || (navigationCloud.points || []).length),
    );
  } catch (_error) {
    voxelMapHint.classList.remove("hidden");
  } finally {
    voxelRefreshPending = false;
  }
}

async function refreshNavigationCloud() {
  if (navigationCloudRefreshPending) return;
  navigationCloudRefreshPending = true;
  try {
    const response = await fetch("/api/navigation/cloud", {cache: "no-store"});
    if (!response.ok) throw new Error("cloud unavailable");
    navigationCloud = (await response.json()).cloud || {};
  } catch (_error) {
    navigationCloud = {};
  } finally {
    navigationCloudRefreshPending = false;
  }
  drawNavigationMap(navigationVoxels, navigationPath, navigationCloud);
  voxelMapHint.classList.toggle(
    "hidden",
    Boolean((navigationVoxels.points || []).length || (navigationCloud.points || []).length),
  );
}

async function loadNavigationMap(automatic = false) {
  const mapId = navigationMapSelect.value;
  const map = navigationMapRecords.get(mapId);
  if (!map?.convertible || navigationLoadPending) return;
  let navigationResult = {enabled: true, state: "stopped"};
  navigationLoadPending = true;
  updateNavigation(navigationResult);
  selectedGoal = null;
  navigationCloud = {};
  navigationVoxels = {};
  navigationPath = {};
  navigationView = null;
  navigationPose = null;
  setNavigationGoalMode(false);
  drawNavigationMap(navigationVoxels, navigationPath, navigationCloud);
  if (!map.loadable) {
    navigationDetail.textContent = `${mapId} 正在转换为网页显示格式，请稍候…`;
  } else if (!map.localizable) {
    navigationDetail.textContent = `${mapId} 正在使用 GPU 构建 HLoc 索引，请稍候…`;
  }
  try {
    const result = await api("/api/navigation/load_map", {map_id: mapId});
    updateNavigationMaps(Array.isArray(result.maps) ? result.maps : []);
    navigationResult = result.navigation || navigationResult;
    updateNavigation(navigationResult);
    await refreshNavigationCloud();
    const loadedMap = navigationMapRecords.get(mapId);
    showToast(
      `${mapId}${map.loadable ? " 已加载" : " 已转换并加载"}；` +
      (loadedMap?.localizable ? "可点击“自动定位”" : "尚未构建 HLoc 索引")
    );
  } catch (error) {
    showToast(`${automatic ? "地图转换或加载" : "地图加载"}失败：${error.message}`);
  } finally {
    navigationLoadPending = false;
    updateNavigation(navigationResult);
  }
}

async function startNavigationLocalization() {
  const mapId = navigationMapSelect.value;
  if (!mapId || navigationLocatePending) return;
  navigationLocatePending = true;
  navigationLocateButton.disabled = true;
  navigationPose = null;
  try {
    const result = await api("/api/navigation/localize", {map_id: mapId});
    updateNavigation(result.navigation);
    showToast("自动定位已启动，请缓慢移动或转动机器人");
  } catch (error) {
    showToast(`自动定位启动失败：${error.message}`);
  } finally {
    navigationLocatePending = false;
  }
}

async function stopNavigation() {
  try {
    const result = await api("/api/navigation/stop");
    setNavigationGoalMode(false);
    updateNavigation(result.navigation);
    showToast("地图定位与规划已停止；当前地图仍保留显示");
  } catch (error) {
    showToast(`停止失败：${error.message}`);
  }
}

navigationLoadButton.addEventListener("click", loadNavigationMap);
navigationLocateButton.addEventListener("click", startNavigationLocalization);
navigationStopButton.addEventListener("click", stopNavigation);
navigationMapSelect.addEventListener("change", () => {
  selectedGoal = null;
  navigationCloud = {};
  navigationVoxels = {};
  navigationPath = {};
  navigationView = null;
  navigationPose = null;
  setNavigationGoalMode(false);
  drawNavigationMap(navigationVoxels, navigationPath, navigationCloud);
  updateNavigation({enabled: true, state: "stopped"});
  loadNavigationMap(true);
});
navigationShowCloud.addEventListener("change", () => drawNavigationMap(navigationVoxels, navigationPath, navigationCloud));
navigationShowVoxels.addEventListener("change", () => drawNavigationMap(navigationVoxels, navigationPath, navigationCloud));
navigationShowMappingOrigin.addEventListener("change", () => drawNavigationMap(navigationVoxels, navigationPath, navigationCloud));
navigationShowRobot.addEventListener("change", () => drawNavigationMap(navigationVoxels, navigationPath, navigationCloud));
function setNavigationGoalMode(enabled) {
  navigationGoalMode = enabled;
  navigationGoalButton.classList.toggle("active", enabled);
  navigationGoalButton.textContent = enabled ? "请点击地图目标" : "选择目标点";
  voxelMapCanvas.classList.toggle("selecting-goal", enabled);
}

navigationGoalButton.addEventListener("click", () => {
  if (navigationGoalButton.disabled) return;
  setNavigationGoalMode(!navigationGoalMode);
  if (navigationGoalMode) showToast("请在地图中点击目标点；可先退出选点模式调整视角");
});

async function selectNavigationGoal(event) {
  if (!voxelViewport) return;
  const rect = voxelMapCanvas.getBoundingClientRect();
  const horizontal = (event.clientX - rect.left - voxelViewport.width * 0.5) / voxelViewport.scale;
  const vertical = (voxelViewport.height * 0.5 - (event.clientY - rect.top)) / voxelViewport.scale;
  const {x, y} = mapProjection.unprojectGround(
    horizontal,
    vertical,
    [voxelViewport.centerX, voxelViewport.centerY, voxelViewport.centerZ],
    voxelViewport.view,
  );
  selectedGoal = {x, y};
  try {
    await api("/api/navigation/goal", {x, y, z: 0});
    setNavigationGoalMode(false);
    showToast(`目标点已发送：${x.toFixed(2)}, ${y.toFixed(2)}`);
    refreshVoxelMap();
  } catch (error) {
    showToast(`规划请求失败：${error.message}`);
  }
}

voxelMapCanvas.addEventListener("pointerdown", (event) => {
  if (navigationGoalMode) {
    selectNavigationGoal(event);
    return;
  }
  if (!voxelViewport) return;
  navigationPointers.set(event.pointerId, {x: event.clientX, y: event.clientY});
  voxelMapCanvas.setPointerCapture?.(event.pointerId);
  if (navigationPointers.size === 2) {
    const [first, second] = Array.from(navigationPointers.values());
    navigationDrag = null;
    navigationPinch = {
      distance: Math.hypot(first.x - second.x, first.y - second.y),
      zoom: navigationView.zoom,
    };
    voxelMapCanvas.classList.add("dragging");
    return;
  }
  if (navigationPointers.size !== 1) return;
  navigationDrag = {
    pointerId: event.pointerId,
    x: event.clientX,
    y: event.clientY,
    yaw: navigationView.yaw,
    pitch: navigationView.pitch,
  };
  voxelMapCanvas.classList.add("dragging");
});
voxelMapCanvas.addEventListener("pointermove", (event) => {
  const pointer = navigationPointers.get(event.pointerId);
  if (!pointer) return;
  pointer.x = event.clientX;
  pointer.y = event.clientY;
  if (navigationPointers.size === 2 && navigationPinch) {
    const [first, second] = Array.from(navigationPointers.values());
    const distance = Math.hypot(first.x - second.x, first.y - second.y);
    if (navigationPinch.distance > 0) {
      navigationView.zoom = Math.max(
        0.3, Math.min(5.0, navigationPinch.zoom * distance / navigationPinch.distance),
      );
      drawNavigationMap(navigationVoxels, navigationPath, navigationCloud);
    }
    return;
  }
  if (!navigationDrag || navigationDrag.pointerId !== event.pointerId) return;
  navigationView.yaw = navigationDrag.yaw + (event.clientX - navigationDrag.x) * 0.012;
  navigationView.pitch = Math.max(
    0.16, Math.min(1.4, navigationDrag.pitch + (event.clientY - navigationDrag.y) * 0.012),
  );
  drawNavigationMap(navigationVoxels, navigationPath, navigationCloud);
});
function endNavigationDrag(event) {
  if (!navigationPointers.has(event.pointerId)) return;
  if (voxelMapCanvas.hasPointerCapture?.(event.pointerId)) {
    voxelMapCanvas.releasePointerCapture(event.pointerId);
  }
  navigationPointers.delete(event.pointerId);
  navigationPinch = null;
  if (navigationPointers.size === 1) {
    const [pointerId, pointer] = navigationPointers.entries().next().value;
    navigationDrag = {
      pointerId,
      x: pointer.x,
      y: pointer.y,
      yaw: navigationView.yaw,
      pitch: navigationView.pitch,
    };
  } else {
    navigationDrag = null;
    voxelMapCanvas.classList.remove("dragging");
  }
}
voxelMapCanvas.addEventListener("pointerup", endNavigationDrag);
voxelMapCanvas.addEventListener("pointercancel", endNavigationDrag);
voxelMapCanvas.addEventListener("wheel", (event) => {
  if (!navigationView) return;
  event.preventDefault();
  navigationView.zoom = Math.max(0.3, Math.min(5.0, navigationView.zoom * Math.exp(-event.deltaY * 0.001)));
  drawNavigationMap(navigationVoxels, navigationPath, navigationCloud);
}, {passive: false});

function updateMapping(mapping) {
  if (!mapping) return;
  const mappingStateName = mappingStateNames[mapping.state] || mapping.state;
  mappingState.textContent = mappingStateName;
  mappingState.className = `mapping-state ${mapping.state}`;
  mappingStartButton.disabled = !mapping.enabled || mapping.state === "running";
  mappingStopButton.disabled = !mapping.enabled || mapping.state !== "running";
  if (mapping.last_error) {
    mappingDetail.textContent = mapping.last_error;
  } else if (mapping.state === "running") {
    const uptime = mapping.uptime_seconds == null ? "" : ` · ${mapping.uptime_seconds}s`;
    mappingDetail.textContent = `正在写入 RTAB-Map 数据库${uptime}`;
  } else if (!mapping.enabled) {
    mappingDetail.textContent = "当前节点未启用建图控制。";
  } else {
    mappingDetail.textContent = "开始前请确认 D435i 相机驱动已经运行。";
  }
}

async function startMapping() {
  try {
    const result = await api("/api/mapping/start");
    updateMapping(result.mapping);
    showToast("RTAB-Map 正在启动，请等待相机输入检查完成");
  } catch (error) {
    showToast(`建图启动失败：${error.message}`);
  }
}

async function stopMapping() {
  mappingStopButton.disabled = true;
  try {
    const result = await api("/api/mapping/stop");
    updateMapping(result.mapping);
    showToast("RTAB-Map 已停止，地图数据库已保存");
  } catch (error) {
    showToast(`建图停止失败：${error.message}`);
  }
}

mappingStartButton.addEventListener("click", startMapping);
mappingStopButton.addEventListener("click", stopMapping);

function setPreviewState(element, active, text) {
  element.textContent = text;
  element.classList.toggle("live", active);
}

function updatePreviewStatus(preview) {
  if (!preview || !preview.enabled) {
    setPreviewState(rgbPreviewState, false, "预览未启用");
    setPreviewState(cloudPreviewState, false, "预览未启用");
    return;
  }
  const rgbLive = preview.rgb_age_seconds != null && preview.rgb_age_seconds < 3;
  setPreviewState(rgbPreviewState, rgbLive, rgbLive ? "实时" : "等待相机");
  const cloud = preview.cloud || {};
  const cloudLive = cloud.point_count > 0 && cloud.age_seconds != null && cloud.age_seconds < 5;
  const cloudText = cloudLive ? `${cloud.point_count} 点` : "等待建图";
  setPreviewState(cloudPreviewState, cloudLive, cloudText);
}

async function refreshRgbPreview() {
  if (rgbRefreshPending) return;
  rgbRefreshPending = true;
  try {
    const response = await fetch(`/api/preview/rgb?t=${Date.now()}`, {cache: "no-store"});
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    const image = await response.blob();
    if (!image.type.startsWith("image/")) throw new Error("invalid image response");
    const nextUrl = URL.createObjectURL(image);
    const previousUrl = rgbObjectUrl;
    rgbObjectUrl = nextUrl;
    rgbPreview.src = nextUrl;
    rgbPreviewHint.classList.add("hidden");
    if (previousUrl) URL.revokeObjectURL(previousUrl);
  } catch (_error) {
    if (!rgbPreview.src) rgbPreviewHint.classList.remove("hidden");
  } finally {
    rgbRefreshPending = false;
  }
}

function drawCloud(points) {
  const rect = cloudPreview.getBoundingClientRect();
  const width = Math.max(1, Math.round(rect.width));
  const height = Math.max(1, Math.round(rect.height));
  const ratio = Math.max(1, window.devicePixelRatio || 1);
  if (cloudPreview.width !== width * ratio || cloudPreview.height !== height * ratio) {
    cloudPreview.width = width * ratio;
    cloudPreview.height = height * ratio;
  }
  const context = cloudPreview.getContext("2d");
  context.setTransform(ratio, 0, 0, ratio, 0, 0);
  context.fillStyle = "#080d13";
  context.fillRect(0, 0, width, height);
  if (!points.length) return;

  const previewView = {yaw: Math.PI / 2, pitch: 0.42, zoom: 1};
  const projected = points.map(([x, y, z, red, green, blue]) => {
    const result = mapProjection.projectMapPoint(
      [x, y, z],
      [0, 0, 0],
      previewView,
    );
    return {x: result.horizontal, y: result.vertical, red, green, blue};
  });
  const xs = projected.map((point) => point.x);
  const ys = projected.map((point) => point.y);
  const spanX = Math.max(0.1, Math.max(...xs) - Math.min(...xs));
  const spanY = Math.max(0.1, Math.max(...ys) - Math.min(...ys));
  const scale = Math.min((width - 24) / spanX, (height - 24) / spanY);
  const centerX = (Math.min(...xs) + Math.max(...xs)) * 0.5;
  const centerY = (Math.min(...ys) + Math.max(...ys)) * 0.5;
  for (const point of projected) {
    const screenX = width * 0.5 + (point.x - centerX) * scale;
    const screenY = height * 0.5 - (point.y - centerY) * scale;
    context.fillStyle = `rgb(${point.red}, ${point.green}, ${point.blue})`;
    context.fillRect(screenX, screenY, 2, 2);
  }
}

async function refreshCloudPreview() {
  if (cloudRefreshPending) return;
  cloudRefreshPending = true;
  try {
    const response = await fetch("/api/preview/cloud", {cache: "no-store"});
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    const result = await response.json();
    const cloud = result.cloud || {};
    const points = Array.isArray(cloud.points) ? cloud.points : [];
    drawCloud(points);
    cloudPreviewHint.classList.toggle("hidden", points.length > 0);
  } catch (_error) {
    cloudPreviewHint.classList.remove("hidden");
  } finally {
    cloudRefreshPending = false;
  }
}

async function refreshStatus() {
  try {
    const response = await fetch("/api/status", {cache: "no-store"});
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    const data = await response.json();
    setConnection(true);
    topic.textContent = data.cmd_vel_topic;
    subscribers.textContent = String(data.subscriber_count);
    state.textContent = stateNames[data.state] || data.state;
    setEstopUi(Boolean(data.estop_active));
    updateMapping(data.mapping);
    updateNavigation(data.navigation);
    updatePreviewStatus(data.preview);

    const linearLimit = Number(data.limits.linear_x);
    const angularLimit = Number(data.limits.angular_z);
    if (linearLimit > 0) {
      linearInput.max = String(linearLimit);
      if (Number(linearInput.value) > linearLimit) linearInput.value = String(linearLimit);
    }
    if (angularLimit > 0) {
      angularInput.max = String(angularLimit);
      if (Number(angularInput.value) > angularLimit) angularInput.value = String(angularLimit);
    }
    updateSpeeds();
  } catch (_error) {
    if (online) stop({keepalive: true});
    setConnection(false);
    subscribers.textContent = "--";
    state.textContent = "不可用";
  }
}

setInterval(sendCommand, 100);
setInterval(refreshStatus, 1000);
setInterval(refreshRgbPreview, 500);
setInterval(refreshCloudPreview, 1200);
setInterval(refreshNavigationMaps, 2500);
setInterval(refreshVoxelMap, 1000);
window.addEventListener("resize", () => {
  refreshCloudPreview();
  refreshVoxelMap();
});
window.addEventListener("blur", () => stop({keepalive: true}));
document.addEventListener("visibilitychange", () => {
  if (document.hidden) stop({keepalive: true});
});
window.addEventListener("pagehide", () => {
  held.clear();
  navigator.sendBeacon("/api/stop", new Blob(["{}"], {type: "application/json"}));
});

updateSpeeds();
refreshStatus();
refreshRgbPreview();
refreshCloudPreview();
refreshNavigationMaps();
refreshVoxelMap();
