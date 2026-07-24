"use strict";

const $ = (selector) => document.querySelector(selector);
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
const navigationStopButton = $("#navigationStopButton");
const voxelMapCanvas = $("#voxelMapCanvas");
const voxelMapHint = $("#voxelMapHint");

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
let voxelRefreshPending = false;
let voxelViewport = null;
let selectedGoal = null;

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
  navigationLoadButton.disabled = !navigation.enabled || !navigationMapSelect.value;
  navigationStopButton.disabled = !navigation.enabled || navigation.state !== "running";
  if (navigation.last_error) {
    navigationDetail.textContent = navigation.last_error;
  } else if (navigation.state === "running") {
    navigationDetail.textContent = `正在使用 ${navigation.map_id || "所选地图"} 进行定位；点击地图发送目标点。`;
  } else if (!navigation.enabled) {
    navigationDetail.textContent = "当前节点未启用地图定位与规划控制。";
  } else {
    navigationDetail.textContent = "选择同时包含 .db 和 .bt 的地图后加载。";
  }
}

function updateNavigationMaps(maps) {
  const previous = navigationMapSelect.value;
  navigationMapSelect.replaceChildren();
  for (const item of maps) {
    const option = document.createElement("option");
    option.value = item.id;
    option.disabled = !item.loadable;
    option.textContent = item.loadable ? `${item.id}（数据库 + 体素地图）` : `${item.id}（缺少 ${item.database_path ? ".bt" : ".db"}）`;
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
    const firstLoadable = maps.find((item) => item.loadable);
    navigationMapSelect.value = firstLoadable ? firstLoadable.id : "";
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

function drawVoxelMap(voxels, path) {
  const {context, width, height} = canvasMetrics(voxelMapCanvas);
  context.fillStyle = "#080d13";
  context.fillRect(0, 0, width, height);
  const points = Array.isArray(voxels.points) ? voxels.points : [];
  const pathPoints = Array.isArray(path.points) ? path.points : [];
  const all = points.concat(pathPoints, selectedGoal ? [[selectedGoal.x, selectedGoal.y, 0]] : []);
  if (!all.length) {
    voxelViewport = null;
    return;
  }
  const xs = all.map((point) => point[0]);
  const ys = all.map((point) => point[1]);
  const minX = Math.min(...xs);
  const maxX = Math.max(...xs);
  const minY = Math.min(...ys);
  const maxY = Math.max(...ys);
  const spanX = Math.max(0.5, maxX - minX);
  const spanY = Math.max(0.5, maxY - minY);
  const scale = Math.min((width - 36) / spanX, (height - 36) / spanY);
  voxelViewport = {minX, maxX, minY, maxY, scale, width, height};
  const toCanvas = (point) => [
    width * 0.5 + (point[0] - (minX + maxX) * 0.5) * scale,
    height * 0.5 - (point[1] - (minY + maxY) * 0.5) * scale,
  ];
  context.fillStyle = "rgba(101, 227, 181, .62)";
  for (const point of points) {
    const [x, y] = toCanvas(point);
    context.fillRect(x - 1, y - 1, 2, 2);
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
    const voxels = (await voxelResponse.json()).voxels || {};
    const path = (await pathResponse.json()).path || {};
    drawVoxelMap(voxels, path);
    voxelMapHint.classList.toggle("hidden", !(voxels.points || []).length);
  } catch (_error) {
    voxelMapHint.classList.remove("hidden");
  } finally {
    voxelRefreshPending = false;
  }
}

async function loadNavigationMap() {
  const mapId = navigationMapSelect.value;
  if (!mapId) return;
  navigationLoadButton.disabled = true;
  try {
    const result = await api("/api/navigation/load_map", {map_id: mapId});
    selectedGoal = null;
    updateNavigation(result.navigation);
    showToast(`${mapId} 正在加载，请等待定位和体素地图就绪`);
  } catch (error) {
    showToast(`地图加载失败：${error.message}`);
  }
}

async function stopNavigation() {
  try {
    const result = await api("/api/navigation/stop");
    updateNavigation(result.navigation);
    showToast("地图定位与规划已停止");
  } catch (error) {
    showToast(`停止失败：${error.message}`);
  }
}

navigationLoadButton.addEventListener("click", loadNavigationMap);
navigationStopButton.addEventListener("click", stopNavigation);
navigationMapSelect.addEventListener("change", () => updateNavigation({enabled: true, state: "stopped"}));
voxelMapCanvas.addEventListener("click", async (event) => {
  if (!voxelViewport) return;
  const rect = voxelMapCanvas.getBoundingClientRect();
  const x = voxelViewport.minX + (event.clientX - rect.left - (voxelViewport.width - (voxelViewport.maxX - voxelViewport.minX) * voxelViewport.scale) * 0.5) / voxelViewport.scale;
  const y = voxelViewport.maxY - (event.clientY - rect.top - (voxelViewport.height - (voxelViewport.maxY - voxelViewport.minY) * voxelViewport.scale) * 0.5) / voxelViewport.scale;
  selectedGoal = {x, y};
  try {
    await api("/api/navigation/goal", {x, y, z: 0});
    showToast(`目标点已发送：${x.toFixed(2)}, ${y.toFixed(2)}`);
    refreshVoxelMap();
  } catch (error) {
    showToast(`规划请求失败：${error.message}`);
  }
});

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

  const yaw = 0.78;
  const cosine = Math.cos(yaw);
  const sine = Math.sin(yaw);
  const projected = points.map(([x, y, z, red, green, blue]) => ({
    x: x * cosine - y * sine,
    y: (x * sine + y * cosine) * 0.42 - z,
    red,
    green,
    blue,
  }));
  const xs = projected.map((point) => point.x);
  const ys = projected.map((point) => point.y);
  const spanX = Math.max(0.1, Math.max(...xs) - Math.min(...xs));
  const spanY = Math.max(0.1, Math.max(...ys) - Math.min(...ys));
  const scale = Math.min((width - 24) / spanX, (height - 24) / spanY);
  const centerX = (Math.min(...xs) + Math.max(...xs)) * 0.5;
  const centerY = (Math.min(...ys) + Math.max(...ys)) * 0.5;
  for (const point of projected) {
    const screenX = width * 0.5 + (point.x - centerX) * scale;
    const screenY = height * 0.5 + (point.y - centerY) * scale;
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
