"use strict";

const $ = (selector) => document.querySelector(selector);
const mapProjection = window.LuxiMapProjection;
const navigationUi = window.LuxiNavigationUi;
const elements = {
  connection: $("#connectionBadge"),
  connectionText: $("#connectionText"),
  mapSelect: $("#mapSelect"),
  mapFiles: $("#mapFiles"),
  filtered: $("#filteredToggle"),
  refreshMaps: $("#refreshMapsButton"),
  preview: $("#previewButton"),
  mapMessage: $("#mapMessage"),
  mapDetails: $("#mapDetails"),
  canvas: $("#mapCanvas"),
  canvasEmpty: $("#canvasEmpty"),
  mapLoading: $("#mapLoading"),
  mapLoadingText: $("#mapLoadingText"),
  showCloud: $("#showCloud"),
  showVoxels: $("#showVoxels"),
  showTerrain: $("#showTerrain"),
  showCostmap: $("#showCostmap"),
  showObstacles: $("#showObstacles"),
  showPath: $("#showPath"),
  zoomOut: $("#zoomOutButton"),
  zoomIn: $("#zoomInButton"),
  resetView: $("#resetViewButton"),
  navigationState: $("#navigationState"),
  navigationMessage: $("#navigationMessage"),
  liveDetails: $("#liveDetails"),
  localize: $("#localizeButton"),
  stopLocalization: $("#stopLocalizationButton"),
  goalX: $("#goalX"),
  goalY: $("#goalY"),
  goalZ: $("#goalZ"),
  goalYaw: $("#goalYaw"),
  sendGoal: $("#sendGoalButton"),
  startTask: $("#startTaskButton"),
  haltTask: $("#haltTaskButton"),
  setHome: $("#setHomeButton"),
  returnHome: $("#returnHomeButton"),
  cameraState: $("#cameraState"),
  cameraProfile: $("#cameraProfileSelect"),
  cameraMessage: $("#cameraMessage"),
  cameraStart: $("#cameraStartButton"),
  cameraStop: $("#cameraStopButton"),
  rgbState: $("#rgbState"),
  rgbPreview: $("#rgbPreview"),
  rgbHint: $("#rgbHint"),
  robotControlToggle: $("#robotControlToggle"),
  robotControlState: $("#robotControlState"),
  robotControlMessage: $("#robotControlMessage"),
  robotBatteryState: $("#robotBatteryState"),
  robotBatteryDetail: $("#robotBatteryDetail"),
  imuState: $("#imuCalibrationState"),
  imuMessage: $("#imuCalibrationMessage"),
  imuButton: $("#imuCalibrationButton"),
  restartSystem: $("#restartSystemButton"),
  restartOverlay: $("#restartOverlay"),
  restartStatus: $("#restartStatus"),
  manualPanel: $("#manualControlPanel"),
  manualState: $("#manualControlState"),
  manualMessage: $("#manualControlMessage"),
  joystickPad: $("#joystickPad"),
  joystickKnob: $("#joystickKnob"),
  joystickLinear: $("#joystickLinear"),
  joystickAngular: $("#joystickAngular"),
  manualStop: $("#manualStopButton"),
  toast: $("#toast"),
};

const manualHeld = new Set();
const manualKeyActions = {
  KeyW: "forward", ArrowUp: "forward",
  KeyS: "backward", ArrowDown: "backward",
  KeyA: "left", ArrowLeft: "left",
  KeyD: "right", ArrowRight: "right",
};
const controlClientId = window.crypto?.randomUUID
  ? window.crypto.randomUUID()
  : `portal-${Date.now()}-${Math.random().toString(16).slice(2)}`;
const MANUAL_LINEAR_SPEED = 0.12;
const MANUAL_ANGULAR_SPEED = 0.45;

const layerNames = {
  database: "RTAB-Map 数据库",
  cloud: "彩色点云",
  octomap: "OctoMap 体素",
  filtered_cloud: "过滤版彩色点云",
  filtered_octomap: "过滤版 OctoMap",
  annotations: "语义标注",
  hloc_metadata: "HLoc 元数据",
};

const state = {
  maps: [],
  mapById: new Map(),
  navigation: {},
  cloud: {},
  voxels: {},
  terrain: {},
  path: {},
  selectedMapId: null,
  loadedMapId: null,
  goal: null,
  goalMode: false,
  selectionPurpose: "goal",
  goalDrag: null,
  view: {...mapProjection.defaultView},
  viewport: null,
  pointer: null,
  toastTimer: null,
  drawPending: false,
  geometryRequestPending: false,
  statusRequestPending: false,
  camera: {},
  robotControl: {},
  cameraRequestPending: false,
  robotControlRequestPending: false,
  imuRequestPending: false,
  restarting: false,
  manualPointerId: null,
  joystickX: 0,
  joystickY: 0,
  manualCommandPending: false,
  manualErrorAt: 0,
  estopActive: false,
  rgbRequestPending: false,
  rgbObjectUrl: null,
  automaticFilteredMapHandled: null,
  automaticFilteredMapLoading: false,
};

class ApiError extends Error {
  constructor(message, status = 0) {
    super(message);
    this.status = status;
  }
}

async function api(path, options = {}) {
  const request = {
    method: options.method || "GET",
    cache: "no-store",
    headers: {},
  };
  if (options.body !== undefined) {
    request.headers["Content-Type"] = "application/json";
    request.body = JSON.stringify(options.body);
  }
  const response = await fetch(path, request);
  const result = await response.json().catch(() => ({}));
  if (!response.ok) {
    throw new ApiError(result.error || `HTTP ${response.status}`, response.status);
  }
  return result;
}

function post(path, body = {}) {
  return api(path, {method: "POST", body});
}

function showToast(message) {
  elements.toast.textContent = message;
  elements.toast.classList.add("show");
  clearTimeout(state.toastTimer);
  state.toastTimer = setTimeout(() => elements.toast.classList.remove("show"), 2600);
}

function setMessage(element, message, kind = "") {
  element.textContent = message;
  element.className = `message ${kind}`.trim();
}

function setOnline(online) {
  elements.connection.classList.toggle("online", online);
  elements.connection.classList.toggle("offline", !online);
  elements.connectionText.textContent = online ? "机器人在线" : "机器人连接中断";
}

function formatBytes(bytes) {
  const value = Number(bytes);
  if (!Number.isFinite(value)) return "--";
  if (value < 1024) return `${value} B`;
  const units = ["KB", "MB", "GB", "TB"];
  let scaled = value;
  let unit = -1;
  do {
    scaled /= 1024;
    unit += 1;
  } while (scaled >= 1024 && unit < units.length - 1);
  return `${scaled.toFixed(scaled >= 100 ? 0 : scaled >= 10 ? 1 : 2)} ${units[unit]}`;
}

function selectedMap() {
  return state.mapById.get(state.selectedMapId) || null;
}

function renderMapDetails(record) {
  if (!record) {
    elements.mapDetails.innerHTML = "<div><dt>地图</dt><dd>--</dd></div>";
    return;
  }
  const totalSize = record.files.reduce((sum, file) => sum + file.size_bytes, 0);
  const latest = [...record.files]
    .sort((a, b) => String(b.modified_at).localeCompare(String(a.modified_at)))[0];
  const capability = record.localizable
    ? "可定位、可导航"
    : record.loadable ? "可查看，尚无 HLoc" : "可转换";
  elements.mapDetails.innerHTML = [
    ["地图编号", record.id],
    ["状态", capability],
    ["可用文件", `${record.files.length} 个`],
    ["文件总量", formatBytes(totalSize)],
    ["最近更新", latest ? new Date(latest.modified_at).toLocaleString() : "--"],
  ].map(([name, value]) => `<div><dt>${name}</dt><dd>${value}</dd></div>`).join("");
}

function createDownloadLink(file) {
  const link = document.createElement("a");
  link.className = "download-link";
  link.href = file.download_url;
  link.download = file.filename;
  const extension = file.filename.split(".").pop().slice(0, 4).toUpperCase();
  const icon = document.createElement("span");
  icon.className = "file-icon";
  icon.textContent = extension;
  const description = document.createElement("span");
  const title = document.createElement("strong");
  title.textContent = layerNames[file.layer] || file.layer;
  const size = document.createElement("small");
  size.textContent = `${file.filename} · ${formatBytes(file.size_bytes)}`;
  description.append(title, size);
  const arrow = document.createElement("span");
  arrow.className = "download-arrow";
  arrow.textContent = "↓";
  link.append(icon, description, arrow);
  return link;
}

function renderMapFiles(record) {
  elements.mapFiles.replaceChildren();
  if (!record?.files.length) {
    const empty = document.createElement("p");
    empty.className = "muted";
    empty.textContent = "该地图还没有可下载的 DB、PLY 或 BT 文件。";
    elements.mapFiles.append(empty);
    return;
  }
  elements.mapFiles.append(...record.files.map(createDownloadLink));
}

function renderMapOptions() {
  elements.mapSelect.replaceChildren();
  for (const record of [...state.maps].reverse()) {
    const option = document.createElement("option");
    option.value = record.id;
    option.textContent = `${record.id} · ${record.files.length} 文件${record.filtered_loadable ? " · 已过滤" : ""}`;
    option.selected = record.id === state.selectedMapId;
    elements.mapSelect.append(option);
  }
  if (!state.maps.length) {
    const option = document.createElement("option");
    option.value = "";
    option.textContent = "暂无地图";
    elements.mapSelect.append(option);
  }
}

function updateVariantAvailability(record) {
  elements.filtered.disabled = !record?.filtered_loadable;
  if (elements.filtered.disabled) elements.filtered.checked = false;
}

function selectMap(mapId, preferDefaultVariant = false) {
  if (mapId && state.mapById.has(mapId)) state.selectedMapId = mapId;
  const record = selectedMap();
  if (record && preferDefaultVariant) {
    elements.filtered.checked = record.default_variant === "filtered";
  }
  renderMapDetails(record);
  renderMapFiles(record);
  updateVariantAvailability(record);
  elements.preview.disabled = !record?.convertible;
  elements.localize.disabled = !record || (
    elements.filtered.checked
      ? !record.filtered_localizable
      : !record.localizable
  );
  elements.mapSelect.value = state.selectedMapId || "";
}

async function refreshMaps() {
  const previous = state.selectedMapId;
  try {
    const result = await api("/api/maps");
    state.maps = Array.isArray(result.maps) ? result.maps : [];
    state.mapById = new Map(state.maps.map((record) => [record.id, record]));
    state.navigation = result.navigation || state.navigation;
    const latestFiltered = [...state.maps].reverse().find(
      (record) => record.filtered_loadable,
    );
    const next = state.mapById.has(previous)
      ? previous
      : latestFiltered?.id || (state.maps.length ? state.maps[state.maps.length - 1].id : "");
    const isNewSelection = next !== previous;
    state.selectedMapId = next;
    renderMapOptions();
    selectMap(next, isNewSelection);
    setOnline(true);
    if (!state.maps.length) setMessage(elements.mapMessage, "没有发现已保存的地图。", "error");
    else setMessage(
      elements.mapMessage,
      `已自动选择 ${next}；请确认过滤版本后点击“加载地图预览”。`,
    );
  } catch (error) {
    setOnline(false);
    setMessage(elements.mapMessage, `地图目录读取失败：${error.message}`, "error");
  }
}

function hasGeometry() {
  return Boolean(
    state.cloud.points?.length || state.voxels.points?.length ||
    state.terrain.traversable_points?.length || state.terrain.obstacle_points?.length
  );
}

function setMapLoading(active, message = "正在加载地图…") {
  elements.mapLoading.hidden = !active;
  elements.mapLoadingText.textContent = message;
  elements.preview.textContent = active ? "地图加载中…" : "加载地图预览";
  elements.canvas.setAttribute("aria-busy", String(active));
}

async function loadPreview() {
  const record = selectedMap();
  if (!record) return;
  elements.preview.disabled = true;
  setMapLoading(true, `正在加载 ${record.id} ${elements.filtered.checked ? "过滤版" : "原始版"}地图…`);
  setMessage(elements.mapMessage, "正在读取并处理地图图层，首次转换可能需要几分钟…");
  try {
    const result = await post(`/api/maps/${record.id}/preview`, {
      filtered: elements.filtered.checked,
    });
    state.loadedMapId = record.id;
    state.view = {...mapProjection.defaultView};
    setMapLoading(true, "地图数据已准备，正在加载三维图层…");
    await refreshGeometry(true);
    await refreshMaps();
    setMessage(elements.mapMessage, result.message, "success");
    showToast(`${record.id} 地图预览已加载`);
    return true;
  } catch (error) {
    setMessage(elements.mapMessage, `地图加载失败：${error.message}`, "error");
    showToast(`地图加载失败：${error.message}`);
    return false;
  } finally {
    elements.preview.disabled = false;
    setMapLoading(false);
  }
}

async function showAutomaticallyFilteredMap(mapId) {
  if (state.automaticFilteredMapLoading ||
      state.automaticFilteredMapHandled === mapId) return;
  state.automaticFilteredMapLoading = true;
  state.automaticFilteredMapHandled = mapId;
  try {
    await refreshMaps();
    const record = state.mapById.get(mapId);
    if (!record?.filtered_loadable) {
      throw new Error("过滤地图文件尚未就绪");
    }
    selectMap(mapId, true);
    elements.filtered.checked = true;
    if (!await loadPreview()) throw new Error("过滤地图加载失败");
    setMessage(elements.mapMessage, `${mapId} 自动过滤完成，已显示过滤结果。`, "success");
  } catch (error) {
    setMessage(elements.mapMessage, `${mapId} 过滤结果显示失败：${error.message}`, "error");
  } finally {
    state.automaticFilteredMapLoading = false;
  }
}

async function refreshGeometry(includeCloud = false) {
  if (state.geometryRequestPending || document.hidden || !state.loadedMapId) return;
  state.geometryRequestPending = true;
  try {
    const previewRoot = `/api/maps/${state.loadedMapId}/preview`;
    const requests = [
      api(`${previewRoot}/voxels`),
      api(`${previewRoot}/terrain`),
      api(`${previewRoot}/path`),
    ];
    if (includeCloud || !state.cloud.points?.length) {
      requests.push(api(`${previewRoot}/cloud`));
    }
    const results = await Promise.all(requests);
    state.voxels = results[0].voxels || {};
    state.terrain = results[1].terrain || {};
    state.path = results[2].path || {};
    if (results[3]) state.cloud = results[3].cloud || {};
    state.loadedMapId = state.cloud.map_id || state.voxels.map_id || state.loadedMapId;
    elements.canvasEmpty.classList.toggle("hidden", hasGeometry());
    scheduleDraw();
  } catch (_error) {
    scheduleDraw();
  } finally {
    state.geometryRequestPending = false;
  }
}

function canvasMetrics() {
  const rect = elements.canvas.getBoundingClientRect();
  const ratio = Math.max(1, window.devicePixelRatio || 1);
  const width = Math.max(1, Math.round(rect.width));
  const height = Math.max(1, Math.round(rect.height));
  if (elements.canvas.width !== Math.round(width * ratio) || elements.canvas.height !== Math.round(height * ratio)) {
    elements.canvas.width = Math.round(width * ratio);
    elements.canvas.height = Math.round(height * ratio);
  }
  const context = elements.canvas.getContext("2d");
  context.setTransform(ratio, 0, 0, ratio, 0, 0);
  return {context, width, height, rect};
}

function visibleGeometry() {
  const result = [];
  if (elements.showCloud.checked) result.push(...(state.cloud.points || []));
  if (elements.showVoxels.checked) result.push(...(state.voxels.points || []));
  if (elements.showTerrain.checked || elements.showCostmap.checked) {
    result.push(...(state.terrain.traversable_points || []));
  }
  if (elements.showObstacles.checked) result.push(...(state.terrain.obstacle_points || []));
  if (elements.showPath.checked) result.push(...(state.path.points || []));
  if (state.goal) result.push([state.goal.x, state.goal.y, state.goal.z]);
  const home = state.navigation.home;
  if (home) result.push([home.x, home.y, home.z]);
  const pose = state.navigation.pose;
  if (pose) result.push([pose.x, pose.y, pose.z || 0]);
  return result;
}

function computeViewport(width, height) {
  const points = visibleGeometry();
  if (!points.length) return null;
  let minX = Infinity;
  let maxX = -Infinity;
  let minY = Infinity;
  let maxY = -Infinity;
  let minZ = Infinity;
  let maxZ = -Infinity;
  for (const point of points) {
    const x = Number(point[0]);
    const y = Number(point[1]);
    const z = Number(point[2]) || 0;
    if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) continue;
    minX = Math.min(minX, x);
    maxX = Math.max(maxX, x);
    minY = Math.min(minY, y);
    maxY = Math.max(maxY, y);
    minZ = Math.min(minZ, z);
    maxZ = Math.max(maxZ, z);
  }
  if (!Number.isFinite(minX)) return null;
  const spanX = Math.max(.5, maxX - minX);
  const spanY = Math.max(.5, maxY - minY);
  const spanZ = Math.max(.5, maxZ - minZ);
  const span = Math.max(spanX, spanY, spanZ);
  const baseScale = Math.min((width - 56) / span, (height - 56) / span);
  return {
    centerX: (minX + maxX) / 2,
    centerY: (minY + maxY) / 2,
    centerZ: (minZ + maxZ) / 2,
    scale: baseScale * state.view.zoom,
    width,
    height,
    view: state.view,
  };
}

function mapToScreen(point, viewport = state.viewport) {
  const projected = mapProjection.projectMapPoint(
    point,
    [viewport.centerX, viewport.centerY, viewport.centerZ],
    viewport.view,
  );
  return [
    viewport.width / 2 + projected.horizontal * viewport.scale,
    viewport.height / 2 - projected.vertical * viewport.scale,
    projected.depth,
  ];
}

function canvasGroundPoint(event, groundZ) {
  if (!state.viewport) return null;
  const rect = elements.canvas.getBoundingClientRect();
  const horizontal = (
    event.clientX - rect.left - state.viewport.width / 2
  ) / state.viewport.scale;
  const vertical = (
    state.viewport.height / 2 - (event.clientY - rect.top)
  ) / state.viewport.scale;
  return mapProjection.unprojectGround(
    horizontal,
    vertical,
    [state.viewport.centerX, state.viewport.centerY, state.viewport.centerZ],
    state.viewport.view,
    groundZ,
  );
}

function drawPointLayer(context, points, color, size = 2) {
  context.fillStyle = color;
  for (const point of points) {
    const [x, y] = mapToScreen(point);
    context.fillRect(x - size / 2, y - size / 2, size, size);
  }
}

function drawMap() {
  state.drawPending = false;
  const {context, width, height} = canvasMetrics();
  context.clearRect(0, 0, width, height);
  state.viewport = computeViewport(width, height);
  if (!state.viewport) return;

  if (elements.showCloud.checked) {
    for (const point of state.cloud.points || []) {
      const [x, y] = mapToScreen(point);
      const red = Math.max(0, Math.min(255, Number(point[3]) || 120));
      const green = Math.max(0, Math.min(255, Number(point[4]) || 150));
      const blue = Math.max(0, Math.min(255, Number(point[5]) || 140));
      context.fillStyle = `rgba(${red},${green},${blue},.66)`;
      context.fillRect(x - 1, y - 1, 2, 2);
    }
  }
  if (elements.showTerrain.checked) {
    drawPointLayer(context, state.terrain.traversable_points || [], "rgba(76, 215, 160, .34)", 3);
  }
  if (elements.showCostmap.checked) {
    for (const point of state.terrain.traversable_points || []) {
      const cost = Math.max(0, Math.min(1, Number(point[3]) || 0));
      if (cost <= 0) continue;
      const [x, y] = mapToScreen(point);
      const green = Math.round(210 - 145 * cost);
      const blue = Math.round(95 - 50 * cost);
      context.fillStyle = `rgba(255,${green},${blue},${.3 + .65 * cost})`;
      const size = Math.max(2, Math.min(8, 2.5 + cost * 3));
      context.fillRect(x - size / 2, y - size / 2, size, size);
    }
  }
  if (elements.showObstacles.checked) {
    drawPointLayer(context, state.terrain.obstacle_points || [], "rgba(255, 91, 91, .78)", 3.4);
  }
  if (elements.showVoxels.checked) {
    drawPointLayer(context, state.voxels.points || [], "rgba(210, 226, 221, .42)", 2.4);
  }
  if (elements.showPath.checked && state.path.points?.length) {
    context.strokeStyle = state.path.stale ? "rgba(255, 208, 120, .5)" : "#ffd078";
    context.lineWidth = 2.5;
    context.setLineDash(state.path.stale ? [7, 5] : []);
    context.beginPath();
    state.path.points.forEach((point, index) => {
      const [x, y] = mapToScreen(point);
      if (index) context.lineTo(x, y);
      else context.moveTo(x, y);
    });
    context.stroke();
    context.setLineDash([]);
  }

  const origin = mapToScreen([0, 0, 0]);
  context.strokeStyle = "rgba(255, 208, 120, .9)";
  context.lineWidth = 2;
  context.beginPath();
  context.moveTo(origin[0] - 6, origin[1]);
  context.lineTo(origin[0] + 6, origin[1]);
  context.moveTo(origin[0], origin[1] - 6);
  context.lineTo(origin[0], origin[1] + 6);
  context.stroke();

  const pose = state.navigation.pose;
  if (pose) {
    const [x, y] = mapToScreen([pose.x, pose.y, pose.z || 0]);
    const yaw = Number(pose.yaw) || 0;
    const [tipX, tipY] = mapToScreen([
      pose.x + .35 * Math.cos(yaw),
      pose.y + .35 * Math.sin(yaw),
      pose.z || 0,
    ]);
    context.fillStyle = "#a98aff";
    context.beginPath();
    context.arc(x, y, 6, 0, Math.PI * 2);
    context.fill();
    context.strokeStyle = "#c0aaff";
    context.lineWidth = 3;
    context.beginPath();
    context.moveTo(x, y);
    context.lineTo(tipX, tipY);
    context.stroke();
  }

  if (state.goal) {
    const [x, y] = mapToScreen([state.goal.x, state.goal.y, state.goal.z]);
    context.strokeStyle = "#ff7979";
    context.fillStyle = "rgba(255, 121, 121, .2)";
    context.lineWidth = 2.5;
    context.beginPath();
    context.arc(x, y, 8, 0, Math.PI * 2);
    context.fill();
    context.stroke();
    context.beginPath();
    context.moveTo(x - 11, y);
    context.lineTo(x + 11, y);
    context.moveTo(x, y - 11);
    context.lineTo(x, y + 11);
    context.stroke();
    const yaw = Number(state.goal.yaw) || 0;
    const [tipX, tipY] = mapToScreen([
      state.goal.x + .30 * Math.cos(yaw),
      state.goal.y + .30 * Math.sin(yaw),
      state.goal.z,
    ]);
    context.beginPath();
    context.moveTo(x, y);
    context.lineTo(tipX, tipY);
    context.stroke();
  }
  const home = state.navigation.home;
  if (home) {
    const [x, y] = mapToScreen([home.x, home.y, home.z]);
    context.strokeStyle = "#4db6ff";
    context.fillStyle = "rgba(77, 182, 255, .22)";
    context.lineWidth = 2.5;
    context.beginPath();
    context.arc(x, y, 8, 0, Math.PI * 2);
    context.fill();
    context.stroke();
    const yaw = Number(home.yaw) || 0;
    const [tipX, tipY] = mapToScreen([
      home.x + .30 * Math.cos(yaw),
      home.y + .30 * Math.sin(yaw),
      home.z,
    ]);
    context.beginPath();
    context.moveTo(x, y);
    context.lineTo(tipX, tipY);
    context.stroke();
    context.fillStyle = "#8fd2ff";
    context.font = "600 12px system-ui, sans-serif";
    context.fillText("返航点", x + 10, y - 10);
  }
}

function scheduleDraw() {
  if (state.drawPending) return;
  state.drawPending = true;
  requestAnimationFrame(drawMap);
}

function setZoom(factor) {
  state.view.zoom = Math.max(.3, Math.min(8, state.view.zoom * factor));
  scheduleDraw();
}

function nearestTerrainGoal(event) {
  if (!state.viewport) return null;
  const rect = elements.canvas.getBoundingClientRect();
  const cursorX = event.clientX - rect.left;
  const cursorY = event.clientY - rect.top;
  let best = null;
  let bestDistance = Infinity;
  for (const point of state.terrain.traversable_points || []) {
    const [x, y] = mapToScreen(point);
    const distance = (x - cursorX) ** 2 + (y - cursorY) ** 2;
    if (distance < bestDistance) {
      bestDistance = distance;
      best = point;
    }
  }
  if (!best || bestDistance > 32 ** 2) return null;
  const degrees = Number(elements.goalYaw.value);
  return {
    x: Number(best[0]), y: Number(best[1]), z: Number(best[2]) || 0,
    yaw: Number.isFinite(degrees) ? degrees * Math.PI / 180 : 0,
  };
}

function setGoalMode(active, purpose = "goal") {
  state.goalMode = active;
  if (active) state.selectionPurpose = purpose;
  elements.sendGoal.classList.toggle(
    "active", active && state.selectionPurpose === "goal");
  elements.setHome.classList.toggle(
    "active", active && state.selectionPurpose === "home");
  elements.sendGoal.textContent = active && state.selectionPurpose === "goal"
    ? "按住目标点并拖动朝向" : "选择并发送目标点";
  elements.setHome.textContent = active && state.selectionPurpose === "home"
    ? "按住返航点并拖动朝向" : "在地图设置返航点";
  elements.canvas.classList.toggle("goal-selecting", active);
}

function setGoal(goal) {
  const yaw = Number.isFinite(Number(goal.yaw)) ? Number(goal.yaw) : 0;
  state.goal = {...goal, yaw: Math.atan2(Math.sin(yaw), Math.cos(yaw))};
  elements.goalX.value = state.goal.x.toFixed(2);
  elements.goalY.value = state.goal.y.toFixed(2);
  elements.goalZ.value = state.goal.z.toFixed(2);
  elements.goalYaw.value = (state.goal.yaw * 180 / Math.PI).toFixed(0);
  scheduleDraw();
  return state.goal;
}

function goalFromInputs() {
  const goal = {
    x: Number(elements.goalX.value),
    y: Number(elements.goalY.value),
    z: Number(elements.goalZ.value),
    yaw: Number(elements.goalYaw.value) * Math.PI / 180,
  };
  if (!Object.values(goal).every(Number.isFinite)) {
    throw new Error("请先在地图选择目标，或输入完整的 X/Y/Z 坐标");
  }
  return goal;
}

async function navigationAction(action, successMessage) {
  try {
    const result = await action();
    showToast(successMessage || result.message || "操作成功");
    await refreshStatus();
  } catch (error) {
    showToast(`操作失败：${error.message}`);
    setMessage(elements.navigationMessage, error.message, "error");
  }
}

async function startLocalization() {
  const record = selectedMap();
  if (!record) return;
  if (state.loadedMapId !== record.id) {
    showToast("请先加载所选地图预览");
    return;
  }
  await navigationAction(
    () => post("/api/navigation/localize", {map_id: record.id}),
    "自动定位正在启动",
  );
}

async function sendNavigationGoal(goal = null) {
  try {
    const target = setGoal(goal || goalFromInputs());
    await navigationAction(
      () => post("/api/navigation/goal", target),
      "目标点已发送，正在规划路径",
    );
  } catch (error) {
    showToast(error.message);
  }
}

function chooseGoalOnMap() {
  if (state.goalMode) {
    setGoalMode(false);
    return;
  }
  if (!state.loadedMapId || !hasGeometry()) {
    showToast("请先加载地图预览");
    return;
  }
  if (!state.navigation.planning_localization_ready) {
    showToast("请先启动定位并等待精定位完成");
    return;
  }
  setGoalMode(true, "goal");
  showToast("在绿色区域按下确定位置，保持按住并拖动箭头选择方向，松开后提交");
}

function chooseHomeOnMap() {
  if (state.goalMode) {
    setGoalMode(false);
    return;
  }
  if (!state.loadedMapId || !hasGeometry()) {
    showToast("请先加载地图预览");
    return;
  }
  setGoalMode(true, "home");
  showToast("在绿色区域按下设置返航位置，拖动箭头选择返航到达方向");
}

async function setNavigationHome(home) {
  try {
    const result = await post("/api/navigation/home/set", home);
    updateNavigation(result.navigation || state.navigation);
    state.goal = null;
    showToast("返航点已保存；蓝色标记为一键返航目标");
    scheduleDraw();
  } catch (error) {
    showToast(`设置返航点失败：${error.message}`);
  }
}

async function returnNavigationHome() {
  stopManualControl();
  state.path = {};
  scheduleDraw();
  await navigationAction(
    () => post("/api/navigation/home/return"),
    "已停止其他导航，正在规划返航路径；规划成功后自动出发",
  );
  await refreshGeometry(false);
}

function updateGoalDirection(event) {
  if (!state.goalDrag || state.goalDrag.id !== event.pointerId || !state.goal) {
    return false;
  }
  const point = canvasGroundPoint(event, state.goal.z);
  if (!point) return true;
  const dx = point.x - state.goal.x;
  const dy = point.y - state.goal.y;
  if (Math.hypot(dx, dy) >= 0.02) {
    state.goal.yaw = Math.atan2(dy, dx);
    elements.goalYaw.value = (state.goal.yaw * 180 / Math.PI).toFixed(0);
    scheduleDraw();
  }
  event.preventDefault();
  return true;
}

function finishGoalGesture(event, submit) {
  if (!state.goalDrag || state.goalDrag.id !== event.pointerId) return false;
  if (elements.canvas.hasPointerCapture?.(event.pointerId)) {
    elements.canvas.releasePointerCapture(event.pointerId);
  }
  state.goalDrag = null;
  const purpose = state.selectionPurpose;
  setGoalMode(false);
  if (submit && state.goal) {
    if (purpose === "home") setNavigationHome(state.goal);
    else sendNavigationGoal(state.goal);
  }
  return true;
}

function navigationStageLabel(navigation) {
  const labels = {
    stopped: "未启动",
    searching: "搜索定位",
    refining: "精定位中",
    localized: "定位完成",
  };
  return labels[navigation.localization_stage] || navigation.localization_stage || "未启动";
}

function updateNavigation(navigation) {
  const wasActive = Boolean(state.navigation.active);
  state.navigation = navigation || {};
  if (wasActive && !state.navigation.active) {
    state.path = {};
  }
  const recoveryState = navigationUi.isRecoveryState(state.navigation.follower_state);
  const stage = recoveryState
    ? navigationUi.followerLabel(state.navigation.follower_state)
    : navigationStageLabel(state.navigation);
  const active = state.navigation.state === "running" || state.navigation.localization_stage === "localized";
  elements.navigationState.textContent = stage;
  elements.navigationState.classList.toggle("active", active);
  const pose = state.navigation.pose;
  const poseText = pose
    ? `${Number(pose.x).toFixed(2)}, ${Number(pose.y).toFixed(2)}, ${Number(pose.z || 0).toFixed(2)}`
    : "--";
  elements.liveDetails.innerHTML = [
    ["定位阶段", stage],
    ["机器人位置", poseText],
    ["规划状态", state.navigation.planning_state || "--"],
    ["任务状态", state.navigation.task_state || "--"],
    ["跟随状态", navigationUi.followerLabel(state.navigation.follower_state)],
  ].map(([name, value]) => `<div><dt>${name}</dt><dd>${value}</dd></div>`).join("");
  const recoveryMessage = navigationUi.recoveryMessage(
    state.navigation, state.navigation.map_id || "所选地图");
  if (recoveryMessage) {
    setMessage(elements.navigationMessage, recoveryMessage);
  } else if (state.navigation.planning_error) {
    setMessage(elements.navigationMessage, state.navigation.planning_error, "error");
  } else if (state.navigation.localization_stage === "localized") {
    setMessage(elements.navigationMessage, "定位有效，可以选择并发送导航目标。", "success");
  } else if (state.navigation.state === "running") {
    setMessage(elements.navigationMessage, "请缓慢移动或转动机器人，等待定位收敛。");
  }
  elements.stopLocalization.disabled = state.navigation.state !== "running";
  elements.haltTask.disabled = !state.navigation.active && !state.navigation.path_ready;
  elements.startTask.disabled = Boolean(state.navigation.active) ||
    !state.navigation.path_ready || !state.navigation.planning_localization_ready ||
    state.estopActive;
  elements.sendGoal.disabled = state.navigation.state !== "running" ||
    !state.navigation.planner_map_ready ||
    !state.navigation.planning_localization_ready || !state.loadedMapId || !hasGeometry();
  elements.setHome.disabled = state.navigation.state !== "running"
    || !state.loadedMapId || !hasGeometry();
  elements.returnHome.disabled = state.navigation.state !== "running"
    || !state.navigation.home || !state.navigation.planning_localization_ready
    || state.estopActive;
  updateManualAvailability();
  scheduleDraw();
}

async function startNavigationTask() {
  stopManualControl();
  await navigationAction(
    () => post("/api/navigation/start"),
    "导航任务已开始",
  );
}

async function haltNavigationTask() {
  stopManualControl();
  state.path = {};
  state.goal = null;
  elements.goalX.value = "";
  elements.goalY.value = "";
  elements.goalZ.value = "";
  elements.goalYaw.value = "0";
  setGoalMode(false);
  scheduleDraw();
  await navigationAction(
    () => post("/api/navigation/halt"),
    "导航任务已停止，旧路径已清空，定位继续保持",
  );
  await refreshGeometry(false);
}

function updateCamera(camera) {
  state.camera = camera || {};
  const profiles = Array.isArray(state.camera.profiles) ? state.camera.profiles : [];
  const profileSignature = profiles.map((profile) => profile.id).join(",");
  if (elements.cameraProfile.dataset.profiles !== profileSignature) {
    const previous = elements.cameraProfile.value;
    elements.cameraProfile.replaceChildren(...profiles.map((profile) => {
      const option = document.createElement("option");
      option.value = profile.id;
      option.textContent = profile.label;
      return option;
    }));
    elements.cameraProfile.dataset.profiles = profileSignature;
    elements.cameraProfile.value = profiles.some((item) => item.id === previous)
      ? previous
      : (state.camera.profile || state.camera.default_profile || profiles[0]?.id || "");
  }
  const labels = {
    running: "运行中", starting: "启动中", waiting: "等待数据",
    stopped: "未启动", failed: "失败", conflict: "冲突", disabled: "未启用",
  };
  elements.cameraState.textContent = labels[state.camera.state] || state.camera.state || "未知";
  elements.cameraState.classList.toggle("active", state.camera.state === "running");
  const sameProfile = state.camera.managed
    && state.camera.profile === elements.cameraProfile.value;
  elements.cameraProfile.disabled = !state.camera.enabled || state.cameraRequestPending;
  elements.cameraStart.disabled = !state.camera.enabled || state.cameraRequestPending
    || !elements.cameraProfile.value || sameProfile;
  elements.cameraStop.disabled = !state.camera.enabled || state.cameraRequestPending
    || !state.camera.managed;
  if (state.camera.last_error) {
    setMessage(elements.cameraMessage, state.camera.last_error, "error");
  } else if (state.camera.external) {
    setMessage(elements.cameraMessage, "检测到外部相机进程，需先停止后才能由网页接管。", "error");
  } else if (state.camera.state === "running") {
    setMessage(elements.cameraMessage, `${state.camera.profile} 图像、内参与 RGB-D 已就绪。`, "success");
  } else if (["starting", "waiting"].includes(state.camera.state)) {
    const counts = state.camera.publisher_counts || {};
    setMessage(elements.cameraMessage, `等待数据：RGB ${counts.color || 0} · 内参 ${counts.camera_info || 0} · RGB-D ${counts.rgbd || 0}`);
  } else {
    setMessage(elements.cameraMessage, "选择相机后启动；切换会安全停止定位任务。");
  }
}

async function startCamera() {
  const profile = elements.cameraProfile.value;
  if (!profile) return;
  if (state.camera.managed && state.camera.profile !== profile
    && !window.confirm("切换相机会停止当前定位和导航，确认继续？")) return;
  state.cameraRequestPending = true;
  updateCamera(state.camera);
  try {
    const result = await post("/api/camera/start", {profile});
    updateCamera(result.camera);
    showToast(`${profile} 正在启动`);
  } catch (error) {
    showToast(`相机启动失败：${error.message}`);
  } finally {
    state.cameraRequestPending = false;
    updateCamera(state.camera);
  }
}

async function stopCamera() {
  if (!window.confirm("关闭相机会停止当前定位和导航，确认继续？")) return;
  state.cameraRequestPending = true;
  updateCamera(state.camera);
  try {
    const result = await post("/api/camera/stop");
    updateCamera(result.camera);
    showToast("相机已关闭");
  } catch (error) {
    showToast(`相机关闭失败：${error.message}`);
  } finally {
    state.cameraRequestPending = false;
    updateCamera(state.camera);
  }
}

function manualControlAvailable() {
  return Boolean(state.robotControl.active)
    && !Boolean(state.navigation.active)
    && !Boolean(state.navigation.path_ready)
    && !state.estopActive;
}

function manualControlActive() {
  return manualHeld.size > 0 || state.manualPointerId !== null;
}

function currentManualCommand() {
  if (state.manualPointerId !== null) {
    return {
      linear_x: -state.joystickY * MANUAL_LINEAR_SPEED,
      linear_y: 0,
      angular_z: -state.joystickX * MANUAL_ANGULAR_SPEED,
    };
  }
  let linearX = 0;
  let angularZ = 0;
  if (manualHeld.has("forward")) linearX += MANUAL_LINEAR_SPEED;
  if (manualHeld.has("backward")) linearX -= MANUAL_LINEAR_SPEED;
  if (manualHeld.has("left")) angularZ += MANUAL_ANGULAR_SPEED;
  if (manualHeld.has("right")) angularZ -= MANUAL_ANGULAR_SPEED;
  return {linear_x: linearX, linear_y: 0, angular_z: angularZ};
}

function updateJoystickKnob() {
  const padRadius = elements.joystickPad.clientWidth * .5;
  const knobRadius = elements.joystickKnob.clientWidth * .5;
  const travel = Math.max(1, padRadius - knobRadius - 5);
  elements.joystickKnob.style.transform =
    `translate(calc(-50% + ${state.joystickX * travel}px), `
    + `calc(-50% + ${state.joystickY * travel}px))`;
  const active = state.manualPointerId !== null;
  elements.joystickPad.classList.toggle("active", active);
  elements.joystickKnob.classList.toggle("active", active);
}

function updateManualReadout(command = currentManualCommand()) {
  elements.joystickLinear.textContent = Number(command.linear_x).toFixed(2);
  elements.joystickAngular.textContent = Number(command.angular_z).toFixed(2);
}

function updateManualAvailability() {
  const available = manualControlAvailable();
  elements.manualPanel.classList.toggle("disabled", !available);
  elements.joystickPad.setAttribute("aria-disabled", String(!available));
  if (state.estopActive) {
    elements.manualState.textContent = "急停中";
    elements.manualMessage.textContent = "请先在开发者控制页解除急停。";
  } else if (state.navigation.active || state.navigation.path_ready) {
    elements.manualState.textContent = state.navigation.active ? "导航任务中" : "路径待执行";
    elements.manualMessage.textContent = "请先点击“停止任务”；定位会继续保持，之后即可手动控制。";
  } else if (!state.robotControl.active) {
    elements.manualState.textContent = "等待机器人";
    elements.manualMessage.textContent = "请先开启机器人控制。";
  } else {
    elements.manualState.textContent = "手动控制可用";
    elements.manualMessage.textContent = "WASD 或拖动轮盘；松开后自动停车，定位继续运行。";
  }
  elements.manualState.classList.toggle("active", available);
  if (!available && manualControlActive()) stopManualControl();
}

async function sendManualCommand() {
  if (!manualControlActive() || !manualControlAvailable() || state.manualCommandPending) return;
  state.manualCommandPending = true;
  const command = currentManualCommand();
  updateManualReadout(command);
  try {
    await post("/api/cmd_vel", {...command, client_id: controlClientId});
  } catch (error) {
    const now = Date.now();
    if (now - state.manualErrorAt > 1500) {
      state.manualErrorAt = now;
      showToast(`网页控制失败：${error.message}`);
    }
  } finally {
    state.manualCommandPending = false;
  }
}

function stopManualControl(options = {}) {
  manualHeld.clear();
  state.manualPointerId = null;
  state.joystickX = 0;
  state.joystickY = 0;
  updateJoystickKnob();
  updateManualReadout({linear_x: 0, angular_z: 0});
  fetch("/api/stop", {
    method: "POST",
    headers: {"Content-Type": "application/json"},
    body: JSON.stringify({client_id: controlClientId}),
    cache: "no-store",
    keepalive: Boolean(options.keepalive),
  }).catch(() => {});
}

function beginManualAction(action) {
  if (!manualControlAvailable()) {
    showToast(state.navigation.active || state.navigation.path_ready
      ? "请先停止导航任务，再使用网页控制"
      : "请先开启机器人控制");
    return;
  }
  state.manualPointerId = null;
  state.joystickX = 0;
  state.joystickY = 0;
  updateJoystickKnob();
  manualHeld.add(action);
  updateManualReadout();
  sendManualCommand();
}

function endManualAction(action) {
  manualHeld.delete(action);
  if (manualHeld.size) sendManualCommand();
  else stopManualControl();
}

function setJoystickFromPointer(event) {
  const rect = elements.joystickPad.getBoundingClientRect();
  const radius = Math.max(1, Math.min(rect.width, rect.height) * .5);
  let x = (event.clientX - (rect.left + rect.width * .5)) / radius;
  let y = (event.clientY - (rect.top + rect.height * .5)) / radius;
  const length = Math.hypot(x, y);
  if (length > 1) {
    x /= length;
    y /= length;
  }
  const deadzone = .07;
  if (length < deadzone) {
    state.joystickX = 0;
    state.joystickY = 0;
  } else {
    const limitedLength = Math.min(1, length);
    const scaledLength = (limitedLength - deadzone) / (1 - deadzone);
    const scale = scaledLength / limitedLength;
    state.joystickX = x * scale;
    state.joystickY = y * scale;
  }
  updateJoystickKnob();
  updateManualReadout();
  sendManualCommand();
}

function updateRobotControl(control) {
  const status = control || {};
  state.robotControl = status;
  const active = Boolean(status.active);
  const transitioning = Boolean(status.transitioning);
  elements.robotControlToggle.checked = Boolean(status.enabled) && active;
  elements.robotControlToggle.disabled = !status.enabled || transitioning
    || state.robotControlRequestPending || (!status.feedback_online && !active);
  const controlLabels = {
    active: "控制开启", inactive: "控制关闭", enabling: "开启中",
    disabling: "关闭中", disabled: "未启用", failed: "失败",
  };
  elements.robotControlState.textContent = controlLabels[status.state] || status.state || "离线";
  elements.robotControlState.classList.toggle("active", active);
  const postureLabels = {
    standing: "站立", standing_up: "起身中", prone: "趴下", unknown: "未知",
    offline: "离线",
  };
  const posture = postureLabels[status.posture] || status.posture || "未知";
  const sdk = status.sdk_active === true ? "SDK 已开启"
    : status.sdk_active === false ? "SDK 已关闭" : "SDK 未知";
  const bridge = status.bridge_active ? "控制桥在线" : "控制桥离线";
  setMessage(
    elements.robotControlMessage,
    status.last_error || status.feedback_error
      || `姿态 ${posture} · FSM ${status.fsm_state || "无反馈"} · ${sdk} · ${bridge}`,
    status.last_error || status.feedback_error ? "error" : "",
  );

  const battery = status.battery || {};
  const packs = Array.isArray(battery.packs)
    ? battery.packs.filter((pack) => pack.online) : [];
  if (battery.online && battery.percentage !== null && battery.percentage !== undefined) {
    elements.robotBatteryState.textContent = `${Number(battery.percentage).toFixed(0)}%`;
    elements.robotBatteryDetail.textContent = packs.map((pack) => {
      const percentage = pack.percentage == null ? "--" : `${Number(pack.percentage).toFixed(0)}%`;
      const voltage = pack.voltage == null ? "" : ` · ${Number(pack.voltage).toFixed(1)} V`;
      return `电池 ${pack.pack}: ${percentage}${voltage}`;
    }).join("  |  ") || "电池反馈在线";
  } else {
    elements.robotBatteryState.textContent = "电量离线";
    elements.robotBatteryDetail.textContent = active
      ? "等待控制桥转发双电池反馈" : "开启机器人控制后显示双电池信息";
  }
  updateManualAvailability();
}

function updateImuCalibration(calibration) {
  const status = calibration || {};
  const labels = {
    offline: "服务离线", idle: "等待校准", waiting_stationary: "等待静止",
    collecting: "采集中", calibrated: "校准完成", stale: "姿态已变化",
    failed: "校准失败",
  };
  elements.imuState.textContent = labels[status.state] || status.state || "未校准";
  elements.imuState.classList.toggle("active", status.state === "calibrated");
  const robotReady = Boolean(state.robotControl.active);
  elements.imuButton.disabled = !status.can_start || Boolean(status.busy)
    || state.imuRequestPending || !robotReady;
  if (!robotReady) {
    setMessage(elements.imuMessage, "请先开启机器人控制，将机器人放在水平面并保持静止。", "error");
  } else if (status.state === "collecting") {
    setMessage(
      elements.imuMessage,
      `请勿移动机器人：${status.sample_count || 0}/${status.required_samples || 0} 帧`,
    );
  } else if (status.state === "calibrated") {
    const roll = Number(status.roll_degrees || 0).toFixed(2);
    const pitch = Number(status.pitch_degrees || 0).toFixed(2);
    setMessage(elements.imuMessage, `校准完成：roll=${roll}°，pitch=${pitch}°`, "success");
  } else {
    setMessage(
      elements.imuMessage,
      status.message || "机器人已开启；确认水平且完全静止后开始校准。",
    );
  }
}

async function startImuCalibration() {
  if (!state.robotControl.active) {
    showToast("请先开启机器人控制并保持机器人静止");
    return;
  }
  if (!window.confirm("确认机器人已开启、放在水平面并完全静止？校准期间请勿移动。")) return;
  state.imuRequestPending = true;
  elements.imuButton.disabled = true;
  try {
    const result = await post("/api/imu/calibrate");
    updateImuCalibration(result.imu_calibration);
    showToast("IMU 校准已开始，请保持机器人静止");
  } catch (error) {
    showToast(`IMU 校准失败：${error.message}`);
  } finally {
    state.imuRequestPending = false;
    refreshStatus();
  }
}

async function toggleRobotControl() {
  const requested = elements.robotControlToggle.checked;
  elements.robotControlToggle.checked = !requested;
  const message = requested
    ? "机器人将站立并开放控制。确认场地安全且有人持急停？"
    : "机器人将停车、趴下并释放控制，确认继续？";
  if (!window.confirm(message)) return;
  state.robotControlRequestPending = true;
  elements.robotControlToggle.disabled = true;
  try {
    const result = await post("/api/robot/control", {active: requested});
    updateRobotControl(result.robot_control);
    showToast(requested ? "正在开启机器人控制" : "正在关闭机器人控制");
  } catch (error) {
    showToast(`机器人控制切换失败：${error.message}`);
  } finally {
    state.robotControlRequestPending = false;
    refreshStatus();
  }
}

function delay(milliseconds) {
  return new Promise((resolve) => setTimeout(resolve, milliseconds));
}

async function waitForSystemRestart() {
  const deadline = Date.now() + 120000;
  let observedOffline = false;
  while (Date.now() < deadline) {
    await delay(1000);
    const controller = new AbortController();
    const timeout = setTimeout(() => controller.abort(), 1500);
    try {
      const response = await fetch("/api/status", {
        cache: "no-store",
        signal: controller.signal,
      });
      if (!response.ok) throw new Error(`HTTP ${response.status}`);
      if (observedOffline) {
        elements.restartStatus.textContent = "服务已重新启动，正在刷新页面…";
        await delay(700);
        window.location.reload();
        return;
      }
      elements.restartStatus.textContent = "重启请求已接受，等待旧服务安全关闭…";
    } catch (_error) {
      observedOffline = true;
      elements.restartStatus.textContent = "服务已关闭，正在重新拉起，请稍候…";
    } finally {
      clearTimeout(timeout);
    }
  }
  elements.restartStatus.textContent = "重启超过 120 秒，请查看服务器 log/luxi_system_restart.log 后手动刷新。";
  state.restarting = false;
  elements.restartSystem.disabled = false;
}

async function restartSystem() {
  if (state.restarting) return;
  const confirmed = window.confirm(
    "一键重启会立即停车、让机器人趴下，并关闭导航、建图、相机、机器人桥和网页服务，随后重新拉起网页。确认继续？",
  );
  if (!confirmed) return;
  stopManualControl();
  state.restarting = true;
  elements.restartSystem.disabled = true;
  try {
    await post("/api/system/restart", {confirm: "restart_all_services"});
    elements.restartOverlay.hidden = false;
    waitForSystemRestart();
  } catch (error) {
    state.restarting = false;
    elements.restartSystem.disabled = false;
    showToast(`系统重启请求失败：${error.message}`);
  }
}

function updateRgbStatus(preview) {
  const live = preview?.enabled && preview.rgb_age_seconds != null
    && preview.rgb_age_seconds < 3;
  elements.rgbState.textContent = live ? "实时" : "等待相机";
  elements.rgbState.classList.toggle("active", live);
}

async function refreshRgbPreview() {
  if (state.rgbRequestPending || document.hidden) return;
  state.rgbRequestPending = true;
  try {
    const response = await fetch(`/api/preview/rgb?t=${Date.now()}`, {cache: "no-store"});
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    const blob = await response.blob();
    if (!blob.type.startsWith("image/")) throw new Error("invalid RGB image");
    const nextUrl = URL.createObjectURL(blob);
    const previousUrl = state.rgbObjectUrl;
    state.rgbObjectUrl = nextUrl;
    elements.rgbPreview.src = nextUrl;
    elements.rgbHint.classList.add("hidden");
    if (previousUrl) URL.revokeObjectURL(previousUrl);
  } catch (_error) {
    if (!elements.rgbPreview.src) elements.rgbHint.classList.remove("hidden");
  } finally {
    state.rgbRequestPending = false;
  }
}

async function refreshStatus() {
  if (state.statusRequestPending) return;
  state.statusRequestPending = true;
  try {
    const result = await api("/api/status");
    setOnline(true);
    state.estopActive = Boolean(result.estop_active);
    updateNavigation(result.navigation || {});
    updateCamera(result.camera || {});
    updateRobotControl(result.robot_control || {});
    updateImuCalibration(result.imu_calibration || {});
    updateRgbStatus(result.preview || {});
    const postprocess = result.mapping?.postprocess || {};
    if (postprocess.state === "completed" && postprocess.map_id &&
        state.automaticFilteredMapHandled !== postprocess.map_id) {
      void showAutomaticallyFilteredMap(postprocess.map_id);
    } else if (["waiting", "filtering"].includes(postprocess.state)) {
      setMessage(elements.mapMessage, postprocess.message || "新地图正在自动过滤…");
    } else if (postprocess.state === "failed") {
      setMessage(elements.mapMessage, postprocess.message || "新地图自动过滤失败。", "error");
    }
  } catch (_error) {
    setOnline(false);
  } finally {
    state.statusRequestPending = false;
  }
}

elements.mapSelect.addEventListener("change", () => {
  selectMap(elements.mapSelect.value, true);
});
elements.filtered.addEventListener("change", () => selectMap(state.selectedMapId));
elements.refreshMaps.addEventListener("click", refreshMaps);
elements.preview.addEventListener("click", loadPreview);
elements.cameraProfile.addEventListener("change", () => updateCamera(state.camera));
elements.cameraStart.addEventListener("click", startCamera);
elements.cameraStop.addEventListener("click", stopCamera);
elements.robotControlToggle.addEventListener("change", toggleRobotControl);
elements.imuButton.addEventListener("click", startImuCalibration);
elements.restartSystem.addEventListener("click", restartSystem);
elements.localize.addEventListener("click", startLocalization);
elements.stopLocalization.addEventListener("click", () => navigationAction(
  () => post("/api/navigation/stop"), "定位已停止",
));
elements.sendGoal.addEventListener("click", chooseGoalOnMap);
elements.setHome.addEventListener("click", chooseHomeOnMap);
elements.returnHome.addEventListener("click", returnNavigationHome);
elements.startTask.addEventListener("click", startNavigationTask);
elements.haltTask.addEventListener("click", haltNavigationTask);
for (const input of [elements.goalX, elements.goalY, elements.goalZ, elements.goalYaw]) {
  input.addEventListener("keydown", (event) => {
    if (event.key !== "Enter") return;
    event.preventDefault();
    sendNavigationGoal();
  });
}

for (const element of [
  elements.showCloud, elements.showVoxels, elements.showTerrain,
  elements.showCostmap, elements.showObstacles, elements.showPath,
]) {
  element.addEventListener("change", scheduleDraw);
}
elements.zoomIn.addEventListener("click", () => setZoom(1.25));
elements.zoomOut.addEventListener("click", () => setZoom(.8));
elements.resetView.addEventListener("click", () => {
  state.view = {...mapProjection.defaultView};
  scheduleDraw();
});

elements.canvas.addEventListener("wheel", (event) => {
  event.preventDefault();
  setZoom(event.deltaY < 0 ? 1.15 : .87);
}, {passive: false});

elements.canvas.addEventListener("pointerdown", (event) => {
  if (!state.viewport) return;
  if (state.goalMode) {
    if (state.goalDrag) return;
    const goal = nearestTerrainGoal(event);
    if (!goal) {
      showToast("该位置附近没有可通行地面，请点击绿色区域");
      return;
    }
    setGoal(goal);
    state.goalDrag = {id: event.pointerId};
    elements.canvas.setPointerCapture(event.pointerId);
    event.preventDefault();
    return;
  }
  elements.canvas.setPointerCapture(event.pointerId);
  state.pointer = {
    id: event.pointerId,
    startX: event.clientX,
    startY: event.clientY,
    yaw: state.view.yaw,
    pitch: state.view.pitch,
  };
  elements.canvas.classList.add("dragging");
});

elements.canvas.addEventListener("pointermove", (event) => {
  if (updateGoalDirection(event)) return;
  if (!state.pointer || state.pointer.id !== event.pointerId) return;
  state.view.yaw = state.pointer.yaw + (event.clientX - state.pointer.startX) * .012;
  state.view.pitch = Math.max(
    .16,
    Math.min(1.4, state.pointer.pitch + (event.clientY - state.pointer.startY) * .012),
  );
  scheduleDraw();
});

elements.canvas.addEventListener("pointerup", (event) => {
  if (finishGoalGesture(event, true)) return;
  if (!state.pointer || state.pointer.id !== event.pointerId) return;
  state.pointer = null;
  elements.canvas.classList.remove("dragging");
});

elements.canvas.addEventListener("pointercancel", (event) => {
  if (finishGoalGesture(event, false)) return;
  state.pointer = null;
  elements.canvas.classList.remove("dragging");
});

elements.joystickPad.addEventListener("pointerdown", (event) => {
  event.preventDefault();
  if (!manualControlAvailable()) {
    showToast(state.navigation.active || state.navigation.path_ready
      ? "请先停止导航任务，再使用轮盘控制"
      : "请先开启机器人控制");
    return;
  }
  manualHeld.clear();
  state.manualPointerId = event.pointerId;
  elements.joystickPad.setPointerCapture?.(event.pointerId);
  setJoystickFromPointer(event);
});

elements.joystickPad.addEventListener("pointermove", (event) => {
  if (state.manualPointerId !== event.pointerId) return;
  event.preventDefault();
  setJoystickFromPointer(event);
});

function endJoystick(event) {
  if (state.manualPointerId !== event.pointerId) return;
  if (elements.joystickPad.hasPointerCapture?.(event.pointerId)) {
    elements.joystickPad.releasePointerCapture(event.pointerId);
  }
  stopManualControl();
}

elements.joystickPad.addEventListener("pointerup", endJoystick);
elements.joystickPad.addEventListener("pointercancel", endJoystick);
elements.joystickPad.addEventListener("lostpointercapture", (event) => {
  if (state.manualPointerId === event.pointerId) stopManualControl();
});
elements.manualStop.addEventListener("pointerdown", (event) => {
  event.preventDefault();
  stopManualControl();
});

document.addEventListener("keydown", (event) => {
  const action = manualKeyActions[event.code];
  if (!action || event.repeat
    || event.target.closest?.("input, select, textarea, button")) return;
  event.preventDefault();
  beginManualAction(action);
});

document.addEventListener("keyup", (event) => {
  const action = manualKeyActions[event.code];
  if (!action) return;
  event.preventDefault();
  endManualAction(action);
});

window.addEventListener("resize", scheduleDraw);
window.addEventListener("pagehide", () => stopManualControl({keepalive: true}));
document.addEventListener("visibilitychange", () => {
  if (document.hidden) stopManualControl({keepalive: true});
  if (!document.hidden) {
    refreshMaps();
    refreshStatus();
    refreshGeometry(true);
  }
});

refreshMaps();
refreshStatus();
refreshGeometry(true);
updateJoystickKnob();
setInterval(refreshStatus, 1000);
setInterval(sendManualCommand, 100);
setInterval(refreshRgbPreview, 200);
setInterval(() => refreshGeometry(false), 1600);
setInterval(refreshMaps, 10000);
setInterval(() => refreshGeometry(true), 12000);
