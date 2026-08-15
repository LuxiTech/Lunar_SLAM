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
const robotControlToggle = $("#robotControlToggle");
const robotControlState = $("#robotControlState");
const robotControlDetail = $("#robotControlDetail");
const robotBatteryState = $("#robotBatteryState");
const robotBatteryDetail = $("#robotBatteryDetail");
const bodyHeightInput = $("#bodyHeight");
const bodyHeightValue = $("#bodyHeightValue");
const bodyHeightDetail = $("#bodyHeightDetail");
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
const mappingMode = $("#mappingMode");
const imuCalibrationState = $("#imuCalibrationState");
const imuCalibrationDetail = $("#imuCalibrationDetail");
const imuCalibrationButton = $("#imuCalibrationButton");
let mappingModeInitialized = false;
const rgbPreview = $("#rgbPreview");
const rgbPreviewState = $("#rgbPreviewState");
const rgbPreviewHint = $("#rgbPreviewHint");
const navigationState = $("#navigationState");
const navigationDetail = $("#navigationDetail");
const navigationMapSelect = $("#navigationMapSelect");
const navigationLoadButton = $("#navigationLoadButton");
const navigationFilterButton = $("#navigationFilterButton");
const navigationLocateButton = $("#navigationLocateButton");
const navigationStopButton = $("#navigationStopButton");
const navigationGoalButton = $("#navigationGoalButton");
const navigationStartButton = $("#navigationStartButton");
const navigationHaltButton = $("#navigationHaltButton");
const voxelMapCanvas = $("#voxelMapCanvas");
const voxelMapHint = $("#voxelMapHint");
const navigationShowCloud = $("#navigationShowCloud");
const navigationShowTraversable = $("#navigationShowTraversable");
const navigationShowCostmap = $("#navigationShowCostmap");
const navigationShowObstacles = $("#navigationShowObstacles");
const navigationShowVoxels = $("#navigationShowVoxels");
const navigationShowSemantics = $("#navigationShowSemantics");
const navigationShowMappingOrigin = $("#navigationShowMappingOrigin");
const navigationShowRobot = $("#navigationShowRobot");
const semanticTool = $("#semanticTool");
const semanticGroundZ = $("#semanticGroundZ");
const semanticMinimumHeight = $("#semanticMinimumHeight");
const semanticBrushRadius = $("#semanticBrushRadius");
const semanticPitDepth = $("#semanticPitDepth");
const semanticFinishPitButton = $("#semanticFinishPitButton");
const semanticUndoButton = $("#semanticUndoButton");
const semanticReloadButton = $("#semanticReloadButton");
const semanticSaveButton = $("#semanticSaveButton");
const semanticStatus = $("#semanticStatus");
const RGB_PREVIEW_INTERVAL_MS = 100;

const held = new Set();
const controlClientId = (() => {
  if (window.crypto?.randomUUID) return window.crypto.randomUUID();
  return `browser-${Date.now()}-${Math.random().toString(16).slice(2)}`;
})();
let estopActive = false;
let online = false;
let toastTimer = null;
let commandRequestPending = false;
let commandFailureToastAt = 0;
let consecutiveCommandTimeouts = 0;
let controlSessionActive = false;
// Do not permit motion during the one-second window before the first actual
// D1 feedback snapshot arrives.
let robotControlReady = false;
let robotControlRequestPending = false;
let bodyHeightRequestPending = false;
let bodyHeightDragging = false;
let bodyHeightTimer = null;
let imuCalibrationRequestPending = false;
let currentImuCalibration = {};
let joystickPointerId = null;
let joystickX = 0;
let joystickY = 0;
let rgbRefreshPending = false;
let rgbObjectUrl = null;
const heavyPreviewControllers = new Set();
let navigationMapsRefreshPending = false;
let navigationLoadPending = false;
let navigationLocatePending = false;
let voxelRefreshPending = false;
let navigationCloudRefreshPending = false;
let voxelViewport = null;
let selectedGoal = null;
let navigationCloud = {};
let navigationVoxels = {};
let navigationTerrain = {};
let navigationPath = {};
let navigationView = null;
let navigationDrag = null;
let navigationPinch = null;
const navigationPointers = new Map();
let navigationGoalMode = false;
let selectedGoalPending = false;
let navigationStatus = {};
let navigationMapRecords = new Map();
let navigationUseFiltered = false;
let navigationPose = null;
let semanticAnnotation = null;
let semanticDraftPit = [];
let semanticHistory = [];
let semanticBrushActive = false;
let semanticBrushPointerId = null;
let semanticDirty = false;

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

class ApiError extends Error {
  constructor(message, status = 0, kind = "http") {
    super(message);
    this.name = "ApiError";
    this.status = status;
    this.kind = kind;
  }
}

async function api(path, body = {}, options = {}) {
  const controller = options.timeoutMs ? new AbortController() : null;
  const timer = controller
    ? setTimeout(() => controller.abort(), options.timeoutMs)
    : null;
  try {
    const response = await fetch(path, {
      method: "POST",
      headers: {"Content-Type": "application/json"},
      body: JSON.stringify(body),
      cache: "no-store",
      keepalive: options.keepalive || false,
      signal: controller?.signal,
    });
    const result = await response.json().catch(() => ({}));
    if (!response.ok) {
      throw new ApiError(result.error || `HTTP ${response.status}`, response.status);
    }
    return result;
  } catch (error) {
    if (error.name === "AbortError") {
      throw new ApiError("控制请求超时", 0, "timeout");
    }
    throw error;
  } finally {
    if (timer !== null) clearTimeout(timer);
  }
}

function controlFailureMessage(error) {
  const message = String(error?.message || error);
  if (message.includes("not standing with SDK control and bridge ready")) {
    return "机器人未站立，或 SDK/控制桥尚未就绪；运动指令已拒绝";
  }
  if (message.includes("control is in use by another browser")) {
    return "另一台手机或浏览器正在控制机器人";
  }
  if (message.includes("emergency stop is active")) {
    return "急停已开启，运动指令已拒绝";
  }
  return message;
}

function heavyPreviewAllowed() {
  return !controlActive() && !controlSessionActive && !document.hidden;
}

function cancelHeavyPreviewRequests() {
  for (const controller of heavyPreviewControllers) controller.abort();
  heavyPreviewControllers.clear();
}

async function fetchHeavyPreview(path) {
  if (!heavyPreviewAllowed()) throw new Error("preview paused");
  const controller = new AbortController();
  heavyPreviewControllers.add(controller);
  try {
    return await fetch(path, {cache: "no-store", signal: controller.signal});
  } finally {
    heavyPreviewControllers.delete(controller);
  }
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
  if (!controlActive() || !robotControlReady || estopActive) return;
  cancelHeavyPreviewRequests();
  if (commandRequestPending) return;
  commandRequestPending = true;
  const command = currentCommand();
  updateReadout(command);
  try {
    await api(
      "/api/cmd_vel",
      {...command, client_id: controlClientId},
      {timeoutMs: 500},
    );
    consecutiveCommandTimeouts = 0;
  } catch (error) {
    if (error.kind === "timeout") consecutiveCommandTimeouts += 1;
    else consecutiveCommandTimeouts = 0;
    const now = Date.now();
    if (
      (error.kind !== "timeout" || consecutiveCommandTimeouts >= 3)
      && now - commandFailureToastAt > 1500
    ) {
      commandFailureToastAt = now;
      const detail = error.kind === "timeout"
        ? "控制链路延迟，指令正在续发；持续中断时机器人会由看门狗自动停车"
        : controlFailureMessage(error);
      showToast(detail);
    }
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
  api(
    "/api/stop",
    {client_id: controlClientId},
    {keepalive: options.keepalive},
  ).catch(() => {});
}

function beginAction(action) {
  if (estopActive) {
    showToast("请先解除急停");
    return;
  }
  if (!robotControlReady) {
    showToast(robotControlState.textContent === "正在站立"
      ? "机器人正在站立，请等待控制就绪"
      : "机器人控制尚未就绪，请先开启或重试控制");
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
  if (!robotControlReady) {
    showToast(robotControlState.textContent === "正在站立"
      ? "机器人正在站立，请等待控制就绪"
      : "机器人控制尚未就绪，请先开启或重试控制");
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

const robotControlStateNames = {
  disabled: "不可用",
  inactive: "已趴下",
  enabling: "正在站立",
  active: "站立且可控制",
  disabling: "正在趴下",
  standing_up: "正在站立",
  standing_uncontrolled: "站立但控制未就绪",
  recovery_required: "需重新开启",
  offline: "状态离线",
  unknown: "未知姿态",
  failed: "切换失败",
};

const robotPostureNames = {
  prone: "趴下",
  standing_up: "站立转换中",
  standing: "非趴下",
  offline: "离线",
  unknown: "未知",
};

const hlocReasonNames = {
  RETRIEVAL_SCORE_LOW: "当前画面与地图参考图差异过大",
  MATCHES_LOW: "局部特征匹配不足",
  LANDMARKS_LOW: "带深度的地图特征不足",
  PNP_FAILED: "特征几何关系无法求出位姿",
  PNP_INLIERS_LOW: "几何一致的特征数量不足",
  PNP_INLIER_RATIO_LOW: "特征几何一致率过低",
  REPROJECTION_ERROR_HIGH: "视觉重投影误差过大",
  DEPTH_MISSING: "当前深度图不可用",
  DEPTH_VALID_POINTS_LOW: "有效深度匹配点不足",
  DEPTH_RESIDUAL_HIGH: "当前深度与地图深度不一致",
  SENSOR_TIME_MISMATCH: "彩色图与深度图时间不同步",
  CAMERA_INFO_SIZE_MISMATCH: "相机内参与图像尺寸不一致",
  CAMERA_TF_UNAVAILABLE: "相机到机器人坐标变换不可用",
  WAITING_FOR_SENSOR_DATA: "正在等待 RGB-D 数据",
  PROCESSING: "正在计算粗定位",
};

function hlocFailureDetail(navigation) {
  const diagnostics = navigation.hloc_diagnostics || {};
  const reason = diagnostics.reason || navigation.hloc_status;
  if (!reason || ["LOCALIZED", "ACCEPTED", "PROCESSING"].includes(reason)) return "";
  const description = hlocReasonNames[reason] || reason;
  const counts = Number.isFinite(Number(diagnostics.matches))
    ? `，匹配=${Number(diagnostics.matches)}` : "";
  const reference = diagnostics.reference
    ? `，候选=${String(diagnostics.reference).split("/").pop()}` : "";
  return `${description}${counts}${reference}`;
}

function updateRobotControl(control) {
  if (!control) return;
  const managed = Boolean(control.enabled);
  const active = Boolean(control.active);
  const bridgeActive = Boolean(control.bridge_active);
  const ready = Boolean(control.control_ready);
  const transitioning = Boolean(control.transitioning);
  robotControlReady = !managed || ready;
  // Standing/bridge-active is not the same as accepting motion commands.
  // Reflect only the backend's complete safety gate in the switch.
  robotControlToggle.checked = managed && ready;
  robotControlToggle.setAttribute(
    "aria-label",
    ready ? "结束机器人控制" : "开启机器人控制",
  );
  robotControlToggle.title = ready ? "机器人控制已开启" : "机器人控制未开启";
  robotControlToggle.disabled = !managed || transitioning || robotControlRequestPending
    || (!control.feedback_online && !bridgeActive && !active);
  robotControlState.textContent = robotControlStateNames[control.state] || control.state;
  robotControlState.className = `mapping-state ${control.state}`;
  const posture = robotPostureNames[control.posture] || control.posture || "未知";
  const fsm = control.fsm_state || "无反馈";
  const sdk = control.sdk_active === true ? "SDK 已开启"
    : control.sdk_active === false ? "SDK 已关闭" : "SDK 未知";
  const bridge = control.bridge_active ? "桥已连接" : "桥未连接";
  const morphology = control.controller_mode ? ` · 形态 ${control.controller_mode}` : "";
  const actualState = `姿态 ${posture} · FSM ${fsm} · ${sdk} · ${bridge}${morphology}`;
  if (!managed) {
    robotControlDetail.textContent = "当前启动配置未启用 D1 控制";
  } else if (control.last_error || control.feedback_error) {
    robotControlDetail.textContent = `${actualState} · ${controlFailureMessage(
      control.last_error || control.feedback_error
    )}`;
  } else if (transitioning) {
    const action = control.state === "enabling"
      ? "正在启用 SDK 并让机器人站立"
      : "正在清零、趴下并释放 SDK";
    robotControlDetail.textContent = `${action} · ${actualState}`;
  } else {
    robotControlDetail.textContent = actualState;
  }

  const battery = control.battery || {};
  const packs = Array.isArray(battery.packs) ? battery.packs.filter((pack) => pack.online) : [];
  if (battery.online && battery.percentage !== null && battery.percentage !== undefined) {
    robotBatteryState.textContent = `${Number(battery.percentage).toFixed(0)}%`;
    robotBatteryState.className = "mapping-state active";
    robotBatteryDetail.textContent = packs.map((pack) => {
      const percentage = pack.percentage === null || pack.percentage === undefined
        ? "--" : `${Number(pack.percentage).toFixed(0)}%`;
      const voltage = pack.voltage === null || pack.voltage === undefined
        ? "" : ` · ${Number(pack.voltage).toFixed(1)} V`;
      return `电池 ${pack.pack}: ${percentage}${voltage}`;
    }).join("  |  ") || "电池反馈在线";
  } else {
    robotBatteryState.textContent = "电量离线";
    robotBatteryState.className = "mapping-state offline";
    robotBatteryDetail.textContent = active
      ? "等待 slam_d1_bridge 转发电池反馈"
      : "开启机器人控制后显示双电池信息";
  }

  const height = control.body_height || {};
  const heightSupported = height.supported === true;
  if (Number.isFinite(Number(height.minimum))) bodyHeightInput.min = height.minimum;
  if (Number.isFinite(Number(height.maximum))) bodyHeightInput.max = height.maximum;
  const reportedHeight = height.current ?? height.target;
  if (!bodyHeightDragging && !bodyHeightRequestPending && Number.isFinite(Number(reportedHeight))) {
    bodyHeightInput.value = Number(reportedHeight).toFixed(0);
  }
  const heightLevel = Number(bodyHeightInput.value);
  const heightMinimum = Number(height.minimum ?? bodyHeightInput.min);
  const heightMaximum = Number(height.maximum ?? bodyHeightInput.max);
  const heightSpan = heightMaximum - heightMinimum;
  const heightPercentage = heightSpan > 0
    ? Math.max(0, Math.min(100, 100 * (heightLevel - heightMinimum) / heightSpan))
    : 0;
  bodyHeightValue.value = `${heightPercentage.toFixed(0)}%`;
  bodyHeightInput.disabled = !heightSupported || !managed || !robotControlReady
    || transitioning || bodyHeightRequestPending;
  if (!heightSupported) {
    bodyHeightDetail.textContent = height.reason
      || "当前形态不支持连续腿高调节";
  } else if (height.online) {
    bodyHeightDetail.textContent =
      `单体双足模式 · 当前 ${heightPercentage.toFixed(0)}% · 控制档位 ${heightLevel.toFixed(1)}/9`;
  } else {
    bodyHeightDetail.textContent = "单体双足模式 · 0～9 档对应 0～100%（原 30% 位置为新上限）；需先开启 SDK 控制";
  }
}

async function toggleRobotControl() {
  const requested = robotControlToggle.checked;
  robotControlToggle.checked = !requested;
  const prompt = requested
    ? "机器人将站立并开放网页运动控制。确认场地已清空且有人持急停？"
    : "机器人将立即停车、趴下并释放 SDK 控制。确认继续？";
  if (!window.confirm(prompt)) return;
  stop();
  robotControlRequestPending = true;
  robotControlToggle.disabled = true;
  try {
    const result = await api("/api/robot/control", {active: requested});
    updateRobotControl(result.robot_control);
    showToast(requested ? "正在开启机器人控制" : "正在结束机器人控制");
  } catch (error) {
    showToast(`机器人控制切换失败：${error.message}`);
  } finally {
    robotControlRequestPending = false;
    refreshStatus();
  }
}

robotControlToggle.addEventListener("change", toggleRobotControl);

async function sendBodyHeight() {
  clearTimeout(bodyHeightTimer);
  if (!robotControlReady || bodyHeightInput.disabled || bodyHeightRequestPending) return;
  bodyHeightRequestPending = true;
  bodyHeightInput.disabled = true;
  stop();
  try {
    const result = await api("/api/robot/height", {height: Number(bodyHeightInput.value)});
    updateRobotControl(result.robot_control);
  } catch (error) {
    showToast(`高度调整失败：${error.message}`);
  } finally {
    bodyHeightRequestPending = false;
    refreshStatus();
  }
}

bodyHeightInput.addEventListener("pointerdown", () => { bodyHeightDragging = true; });
bodyHeightInput.addEventListener("pointerup", () => { bodyHeightDragging = false; });
bodyHeightInput.addEventListener("pointercancel", () => { bodyHeightDragging = false; });
bodyHeightInput.addEventListener("input", () => {
  const level = Number(bodyHeightInput.value);
  const minimum = Number(bodyHeightInput.min);
  const span = Number(bodyHeightInput.max) - minimum;
  const percentage = span > 0
    ? Math.max(0, Math.min(100, 100 * (level - minimum) / span))
    : 0;
  bodyHeightValue.value = `${percentage.toFixed(0)}%`;
  clearTimeout(bodyHeightTimer);
  bodyHeightTimer = setTimeout(sendBodyHeight, 180);
});
bodyHeightInput.addEventListener("change", sendBodyHeight);

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

const imuCalibrationStateNames = {
  offline: "服务离线",
  idle: "等待校准",
  starting: "启动 H30",
  waiting_stationary: "等待静止",
  collecting: "采集中",
  calibrated: "校准完成",
  stale: "姿态已变化",
  failed: "校准失败",
};

function updateImuCalibration(calibration) {
  if (!calibration) return;
  currentImuCalibration = calibration;
  const stateName = imuCalibrationStateNames[calibration.state] || calibration.state;
  imuCalibrationState.textContent = stateName;
  imuCalibrationState.className = `mapping-state ${calibration.state}`;
  const busy = Boolean(calibration.busy);
  imuCalibrationButton.disabled = !calibration.can_start || busy
    || imuCalibrationRequestPending;
  if (calibration.state === "starting") {
    imuCalibrationDetail.textContent = "正在独立启动 H30 校准链路，请保持机器人静止。";
  } else if (calibration.state === "collecting") {
    imuCalibrationDetail.textContent =
      `请勿移动机器人：${calibration.sample_count || 0}/${calibration.required_samples || 0} 帧`;
  } else if (calibration.state === "calibrated") {
    const rollValue = calibration.installation_roll_degrees
      ?? (Number(calibration.roll_degrees) + 90.0);
    const pitchValue = calibration.installation_pitch_degrees
      ?? calibration.pitch_degrees;
    const roll = Number(rollValue).toFixed(2);
    const pitch = Number(pitchValue).toFixed(2);
    const saved = calibration.persisted ? "，已保存并应用到建图/定位" : "";
    imuCalibrationDetail.textContent = `实际安装偏角：roll=${roll}°，pitch=${pitch}°${saved}`;
  } else {
    imuCalibrationDetail.textContent = calibration.message
      || "将机器人放在水平面并保持静止，然后点击一键校准。";
  }
}

async function startImuCalibration() {
  if (!window.confirm("确认机器人已放在水平面并完全静止？校准期间不能建图或导航。")) return;
  stop();
  imuCalibrationRequestPending = true;
  imuCalibrationButton.disabled = true;
  try {
    const result = await api("/api/imu/calibrate");
    updateImuCalibration(result.imu_calibration);
    showToast("IMU 校准已开始，请保持机器人静止");
  } catch (error) {
    showToast(`IMU 校准失败：${error.message}`);
  } finally {
    imuCalibrationRequestPending = false;
    refreshStatus();
  }
}

imuCalibrationButton.addEventListener("click", startImuCalibration);

const navigationStateNames = {
  disabled: "不可用",
  stopped: "未加载",
  running: "定位/规划中",
  failed: "启动失败",
};

function updateNavigation(navigation) {
  if (!navigation) return;
  navigationStatus = navigation;
  if (!navigationLoadPending && ["original", "filtered"].includes(navigation.map_variant)) {
    navigationUseFiltered = navigation.map_variant === "filtered";
  }
  let name = navigationStateNames[navigation.state] || navigation.state;
  if (navigation.follower_state === "localization_degraded") {
    name = "定位恢复中（已停车）";
  } else if (navigation.active) {
    name = "行驶中";
  } else if (navigation.follower_state === "goal_reached") {
    name = "已到达";
  } else if (navigation.path_ready) {
    name = "规划完成";
  } else if (navigation.planning_state === "pending") {
    name = "正在规划";
  } else if (navigation.planning_state === "failed") {
    name = "规划失败";
  }
  navigationState.textContent = name;
  navigationState.className = `preview-state ${navigation.state === "running" ? "live" : ""}`;
  const selectedMap = navigationMapRecords.get(navigationMapSelect.value);
  const selectedVariant = navigationUseFiltered ? "filtered" : "original";
  const selectedLocalizable = navigationUseFiltered
    ? selectedMap?.filtered_localizable
    : selectedMap?.localizable;
  const selectedLoadable = navigationUseFiltered
    ? selectedMap?.filtered_loadable
    : selectedMap?.loadable;
  navigationLoadButton.disabled = navigationLoadPending || !selectedMap?.convertible;
  navigationFilterButton.disabled = navigationLoadPending || !selectedMap?.convertible;
  navigationFilterButton.textContent = navigationUseFiltered
    ? "查看原始地图"
    : selectedMap?.filtered_loadable ? "查看过滤地图" : "生成过滤地图";
  navigationFilterButton.classList.toggle("active", navigationUseFiltered);
  navigationMapSelect.disabled = navigationLoadPending;
  navigationLocateButton.disabled =
    navigationLoadPending || navigationLocatePending ||
    navigation.state === "running" ||
    !selectedLoadable ||
    navigationCloud.map_id !== selectedMap?.id ||
    navigationCloud.variant !== selectedVariant ||
    navigationVoxels.variant !== selectedVariant;
  navigationStopButton.disabled = !navigation.enabled || navigation.state !== "running";
  const hasTraversableTerrain =
    Array.isArray(navigationTerrain.traversable_points) &&
    navigationTerrain.traversable_points.length > 0;
  navigationGoalButton.disabled =
    navigation.state !== "running" || !hasTraversableTerrain;
  if (!navigationGoalMode) {
    navigationGoalButton.textContent =
      selectedGoalPending
        ? navigation.planning_localization_ready
          ? "提交已选目标"
          : "重新选择目标"
        : "选择目标点";
  }
  navigationStartButton.disabled =
    !navigation.planning_localization_ready || !navigation.path_ready ||
    navigation.active || estopActive || !robotControlReady;
  navigationHaltButton.disabled = !navigation.active;
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
      ? ` x=${pose.x.toFixed(2)}m，y=${pose.y.toFixed(2)}m，` +
        `地表z=${pose.z.toFixed(2)}m，yaw=${pose.yaw_degrees.toFixed(1)}°`
      : "";
    if (navigation.follower_state === "localization_degraded") {
      navigationDetail.textContent =
        `${mapName} 定位暂时失效，车辆保持零速度；可信定位恢复后将自动继续。`;
    } else if (navigation.localization_stage === "localized") {
      const fitness = navigation.localization_fitness == null
        ? "" : `，fitness=${Number(navigation.localization_fitness).toFixed(3)}`;
      const motionText = navigation.active
        ? "正在沿规划路径行驶。"
        : !navigation.planning_localization_ready
          ? Number(navigation.localization_fitness) < 0.05
            ? "当前扫描与地图没有有效重叠（常见于地图边缘或视野被遮挡），正在重新启动 HLoc；可先选择目标，但恢复前不会提交规划。"
            : "近期 ICP 未通过地图匹配验证，已暂停新的规划请求；可先选择目标，并调整相机视野等待定位恢复。"
        : navigation.follower_state === "goal_reached"
          ? "已到达目标点。"
          : navigation.planning_state === "pending"
            ? "正在计算新路径，请稍候。"
          : navigation.planning_state === "failed"
            ? `规划失败：${navigation.planning_error || "未生成可执行路径"}；` +
              "灰色虚线仅为上一条有效路径参考，不能用于出发。"
          : navigation.path_ready
            ? robotControlReady
              ? `规划完成，共 ${navigation.path_point_count} 个路径点；可点击“出发”。`
              : `规划完成，共 ${navigation.path_point_count} 个路径点；请先开启机器人控制。`
            : "请选择目标点并等待路径规划完成。";
      navigationDetail.textContent =
        `${mapName} 已完成 HLoc 粗定位和 ICP 精定位：${poseText}${fitness}。${motionText}`;
    } else if (navigation.localization_stage === "refining") {
      navigationDetail.textContent =
        `${mapName} 已找到 HLoc 全局候选，正在进行 ICP 精配准：${poseText}。`;
    } else {
      const failure = hlocFailureDetail(navigation);
      navigationDetail.textContent = failure
        ? `${mapName} 粗定位尚未通过：${failure}。` +
          "请让相机看到墙角、门框、箱体等有区分度的物体，并缓慢转动；机器人保持零速度。"
        : `正在 ${mapName} 中进行全局粗定位；请缓慢移动或转动机器人。`;
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
    if (item.filtered_loadable) option.textContent += "；可切换过滤版";
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
    const latestConvertible = [...maps].reverse().find((item) => item.convertible);
    navigationMapSelect.value = latestConvertible ? latestConvertible.id : "";
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

function semanticVoxelKey(point) {
  return [point.x ?? point[0], point.y ?? point[1], point.z ?? point[2]]
    .map((value) => Number(value).toFixed(3))
    .join(":");
}

function semanticGround() {
  return {
    z: Number(semanticGroundZ.value),
    minimum_height: Math.max(0, Number(semanticMinimumHeight.value)),
  };
}

function semanticSnapshot() {
  return {
    annotation: semanticAnnotation
      ? JSON.parse(JSON.stringify(semanticAnnotation))
      : null,
    draftPit: JSON.parse(JSON.stringify(semanticDraftPit)),
  };
}

function updateSemanticControls() {
  const loaded = Boolean(semanticAnnotation);
  const labels = semanticAnnotation?.occupied_labels || [];
  const pits = semanticAnnotation?.pits || [];
  const rockCount = labels.filter((item) => item.type === "rock").length;
  const wallCount = labels.filter((item) => item.type === "wall").length;
  semanticFinishPitButton.disabled = !loaded || semanticDraftPit.length < 3;
  semanticUndoButton.disabled = !semanticHistory.length;
  semanticReloadButton.disabled = !loaded;
  semanticSaveButton.disabled = !loaded || !semanticDirty || semanticDraftPit.length > 0;
  semanticStatus.textContent = loaded
    ? "岩石 " + rockCount + " 体素 · 墙 " + wallCount +
      " 体素 · 坑 " + pits.length + " 区域" +
      (semanticDraftPit.length ? ` · 坑草稿 ${semanticDraftPit.length} 点` : "") +
      (semanticDirty ? " · 尚未保存" : " · 已保存")
    : "加载地图后可以标注";
  const marking = loaded && semanticTool.value !== "orbit";
  voxelMapCanvas.classList.toggle("semantic-marking", marking);
}

function pushSemanticHistory() {
  if (!semanticAnnotation) return;
  semanticHistory.push(semanticSnapshot());
  if (semanticHistory.length > 50) semanticHistory.shift();
  semanticDirty = true;
  updateSemanticControls();
}

function drawNavigationMap(voxels, path, cloud) {
  const {context, width, height} = canvasMetrics(voxelMapCanvas);
  context.fillStyle = "#080d13";
  context.fillRect(0, 0, width, height);
  const points = Array.isArray(voxels.points) ? voxels.points : [];
  const pathPoints = Array.isArray(path.points) ? path.points : [];
  const cloudPoints = Array.isArray(cloud.points) ? cloud.points : [];
  const traversablePoints = Array.isArray(navigationTerrain.traversable_points)
    ? navigationTerrain.traversable_points : [];
  const obstaclePoints = Array.isArray(navigationTerrain.obstacle_points)
    ? navigationTerrain.obstacle_points : [];
  const mapHasGeometry = Boolean(
    cloudPoints.length || points.length || traversablePoints.length ||
    obstaclePoints.length || pathPoints.length
  );
  const mappingOrigin = [0, 0, 0];
  const all = cloudPoints.concat(
    points,
    traversablePoints,
    obstaclePoints,
    pathPoints,
    selectedGoal ? [[selectedGoal.x, selectedGoal.y, selectedGoal.z]] : [],
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
  const terrainResolution = Number(navigationTerrain.resolution) || 0.05;
  if (navigationShowObstacles.checked) {
    context.fillStyle = "rgba(216, 59, 72, .78)";
    for (const point of obstaclePoints) {
      const [x, y] = toCanvas(point);
      const size = Math.max(2, Math.min(15, terrainResolution * scale));
      context.fillRect(x - size * 0.5, y - size * 0.5, size, size);
    }
  }
  if (navigationShowTraversable.checked) {
    context.fillStyle = "rgba(101, 227, 181, .38)";
    for (const point of traversablePoints) {
      const [x, y] = toCanvas(point);
      const size = Math.max(1.5, Math.min(14, terrainResolution * scale));
      context.fillRect(x - size * 0.5, y - size * 0.5, size, size);
    }
  }
  if (navigationShowCostmap.checked) {
    for (const point of traversablePoints) {
      const cost = Math.max(0, Math.min(1, Number(point[3]) || 0));
      if (cost <= 0) continue;
      const red = 255;
      const green = Math.round(209 - 71 * cost);
      const blue = Math.round(102 - 41 * cost);
      context.fillStyle = `rgba(${red}, ${green}, ${blue}, ${0.35 + 0.55 * cost})`;
      const size = Math.max(2, Math.min(15, terrainResolution * scale));
      const [x, y] = toCanvas(point);
      context.fillRect(x - size * 0.5, y - size * 0.5, size, size);
    }
  }
  if (navigationShowVoxels.checked) {
    context.fillStyle = "rgba(190, 203, 218, .42)";
    const ground = semanticGround();
    const filterHighVoxels = (
      semanticAnnotation && ["rock", "wall"].includes(semanticTool.value)
    );
    for (const point of points) {
      if (filterHighVoxels && Number(point[2]) < ground.z + ground.minimum_height) {
        continue;
      }
      const [x, y] = toCanvas(point);
      const voxelSize = Math.max(1, Math.min(14, (Number(point[3]) || voxels.resolution || 0.05) * scale));
      context.fillRect(x - voxelSize * 0.5, y - voxelSize * 0.5, voxelSize, voxelSize);
    }
  }
  if (semanticAnnotation && navigationShowSemantics.checked) {
    const groundZ = semanticGround().z;
    for (const pit of semanticAnnotation.pits || []) {
      const polygon = pit.polygon || [];
      if (polygon.length < 3) continue;
      context.fillStyle = "rgba(61, 157, 255, .25)";
      context.strokeStyle = "#58a6ff";
      context.lineWidth = 2;
      context.beginPath();
      polygon.forEach((point, index) => {
        const [x, y] = toCanvas([point[0], point[1], groundZ]);
        if (index) context.lineTo(x, y);
        else context.moveTo(x, y);
      });
      context.closePath();
      context.fill();
      context.stroke();
    }
    for (const label of semanticAnnotation.occupied_labels || []) {
      const [x, y] = toCanvas([label.x, label.y, label.z]);
      context.fillStyle = label.type === "rock" ? "#ff9f43" : "#ff5e66";
      context.beginPath();
      context.arc(x, y, 4.5, 0, Math.PI * 2);
      context.fill();
    }
    if (semanticDraftPit.length) {
      context.strokeStyle = "#8dc6ff";
      context.fillStyle = "#8dc6ff";
      context.lineWidth = 2;
      context.beginPath();
      semanticDraftPit.forEach((point, index) => {
        const [x, y] = toCanvas([point[0], point[1], groundZ]);
        if (index) context.lineTo(x, y);
        else context.moveTo(x, y);
        context.fillRect(x - 2, y - 2, 4, 4);
      });
      context.stroke();
    }
  }
  if (pathPoints.length) {
    context.strokeStyle = path.stale ? "rgba(190, 203, 218, .68)" : "#ffd166";
    context.lineWidth = 2.5;
    context.setLineDash(path.stale ? [8, 6] : []);
    context.beginPath();
    pathPoints.forEach((point, index) => {
      const [x, y] = toCanvas(point);
      if (index) context.lineTo(x, y);
      else context.moveTo(x, y);
    });
    context.stroke();
    context.setLineDash([]);
  }
  if (selectedGoal) {
    const [x, y] = toCanvas([selectedGoal.x, selectedGoal.y, selectedGoal.z]);
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
  if (voxelRefreshPending || !heavyPreviewAllowed()) return;
  voxelRefreshPending = true;
  try {
    const [voxelResponse, pathResponse, terrainResponse] = await Promise.all([
      fetchHeavyPreview("/api/navigation/voxels"),
      fetchHeavyPreview("/api/navigation/path"),
      fetchHeavyPreview("/api/navigation/terrain"),
    ]);
    if (!voxelResponse.ok || !pathResponse.ok || !terrainResponse.ok) {
      throw new Error("preview unavailable");
    }
    navigationVoxels = (await voxelResponse.json()).voxels || {};
    navigationPath = (await pathResponse.json()).path || {};
    navigationTerrain = (await terrainResponse.json()).terrain || {};
    drawNavigationMap(navigationVoxels, navigationPath, navigationCloud);
    voxelMapHint.classList.toggle(
      "hidden",
      Boolean(
        (navigationVoxels.points || []).length ||
        (navigationCloud.points || []).length ||
        (navigationTerrain.traversable_points || []).length ||
        (navigationTerrain.obstacle_points || []).length
      ),
    );
  } catch (_error) {
    voxelMapHint.classList.remove("hidden");
  } finally {
    voxelRefreshPending = false;
  }
}

async function refreshNavigationCloud() {
  if (navigationCloudRefreshPending || !heavyPreviewAllowed()) return;
  navigationCloudRefreshPending = true;
  try {
    const response = await fetchHeavyPreview("/api/navigation/cloud");
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
    Boolean(
      (navigationVoxels.points || []).length ||
      (navigationCloud.points || []).length ||
      (navigationTerrain.traversable_points || []).length ||
      (navigationTerrain.obstacle_points || []).length
    ),
  );
}

function clearSemanticAnnotations() {
  semanticAnnotation = null;
  semanticDraftPit = [];
  semanticHistory = [];
  semanticDirty = false;
  semanticTool.value = "orbit";
  updateSemanticControls();
}

async function refreshSemanticAnnotations(mapId) {
  if (!mapId) {
    clearSemanticAnnotations();
    return;
  }
  const response = await fetch(
    "/api/semantic/annotations?map_id=" + encodeURIComponent(mapId),
    {cache: "no-store"},
  );
  const result = await response.json();
  if (!response.ok) throw new Error(result.error || "标注加载失败");
  semanticAnnotation = result.annotation;
  semanticDraftPit = [];
  semanticHistory = [];
  semanticDirty = false;
  semanticGroundZ.value = Number(semanticAnnotation.ground.z).toFixed(2);
  semanticMinimumHeight.value =
    Number(semanticAnnotation.ground.minimum_height).toFixed(2);
  updateSemanticControls();
  drawNavigationMap(navigationVoxels, navigationPath, navigationCloud);
}

async function loadNavigationMap(automatic = false) {
  const mapId = navigationMapSelect.value;
  const map = navigationMapRecords.get(mapId);
  if (!map?.convertible || navigationLoadPending) return false;
  const filtered = navigationUseFiltered;
  const variantLabel = filtered ? "过滤地图" : "原始地图";
  const variantLoadable = filtered ? map.filtered_loadable : map.loadable;
  const variantLocalizable = filtered ? map.filtered_localizable : map.localizable;
  let navigationResult = {enabled: true, state: "stopped"};
  navigationLoadPending = true;
  updateNavigation(navigationResult);
  selectedGoal = null;
  selectedGoalPending = false;
  navigationCloud = {};
  navigationVoxels = {};
  navigationTerrain = {};
  navigationPath = {};
  navigationView = null;
  navigationPose = null;
  clearSemanticAnnotations();
  setNavigationGoalMode(false);
  drawNavigationMap(navigationVoxels, navigationPath, navigationCloud);
  if (!variantLoadable) {
    navigationDetail.textContent = `${mapId} 正在生成${variantLabel}，请稍候…`;
  } else if (!variantLocalizable) {
    navigationDetail.textContent =
      `${mapId} 地图图层正在加载；点击“自动定位”时会按需构建 HLoc 索引。`;
  }
  try {
    const result = await api("/api/navigation/load_map", {
      map_id: mapId,
      filtered: navigationUseFiltered,
    });
    navigationUseFiltered = result.map_variant === "filtered";
    updateNavigationMaps(Array.isArray(result.maps) ? result.maps : []);
    navigationResult = result.navigation || navigationResult;
    updateNavigation(navigationResult);
    await Promise.all([
      refreshNavigationCloud(),
      refreshVoxelMap(),
      refreshSemanticAnnotations(mapId),
    ]);
    const loadedMap = navigationMapRecords.get(mapId);
    showToast(
      `${mapId} ${variantLoadable ? "已加载" : "已生成并加载"}${variantLabel}；` +
      ((navigationUseFiltered ? loadedMap?.filtered_localizable : loadedMap?.localizable)
        ? "可点击“自动定位”" : "点击“自动定位”将构建 HLoc 索引")
    );
    return true;
  } catch (error) {
    showToast(`${automatic ? "地图转换或加载" : "地图加载"}失败：${error.message}`);
    return false;
  } finally {
    navigationLoadPending = false;
    updateNavigation(navigationResult);
  }
}

async function toggleNavigationFilter() {
  if (navigationLoadPending) return;
  const previous = navigationUseFiltered;
  navigationUseFiltered = !previous;
  updateNavigation({enabled: true, state: "stopped"});
  if (!await loadNavigationMap(false)) {
    navigationUseFiltered = previous;
    updateNavigation({enabled: true, state: "stopped"});
  }
}

async function startNavigationLocalization() {
  const mapId = navigationMapSelect.value;
  const map = navigationMapRecords.get(mapId);
  const mapLocalizable = navigationUseFiltered
    ? map?.filtered_localizable
    : map?.localizable;
  if (!mapId || navigationLocatePending) return;
  navigationLocatePending = true;
  navigationLocateButton.disabled = true;
  navigationPose = null;
  if (!mapLocalizable) {
    navigationDetail.textContent =
      `${mapId} 首次定位正在构建 GPU HLoc 索引，完成后会自动启动相机和定位…`;
  }
  try {
    const result = await api("/api/navigation/localize", {map_id: mapId});
    if (Array.isArray(result.maps)) updateNavigationMaps(result.maps);
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

async function startNavigationMotion() {
  navigationStartButton.disabled = true;
  try {
    const result = await api("/api/navigation/start");
    updateNavigation(result.navigation);
    showToast("导航已出发；手动操作或停止按钮会立即取消导航");
  } catch (error) {
    showToast(`出发失败：${error.message}`);
  }
}

async function haltNavigationMotion() {
  navigationHaltButton.disabled = true;
  try {
    const result = await api("/api/navigation/halt");
    updateNavigation(result.navigation);
    showToast("导航行驶已停止，定位和规划路径仍保留");
  } catch (error) {
    showToast(`停止行驶失败：${error.message}`);
  }
}

navigationLoadButton.addEventListener("click", loadNavigationMap);
navigationFilterButton.addEventListener("click", toggleNavigationFilter);
navigationLocateButton.addEventListener("click", startNavigationLocalization);
navigationStopButton.addEventListener("click", stopNavigation);
navigationStartButton.addEventListener("click", startNavigationMotion);
navigationHaltButton.addEventListener("click", haltNavigationMotion);
navigationMapSelect.addEventListener("change", () => {
  navigationUseFiltered = false;
  selectedGoal = null;
  selectedGoalPending = false;
  navigationCloud = {};
  navigationVoxels = {};
  navigationTerrain = {};
  navigationPath = {};
  navigationView = null;
  navigationPose = null;
  clearSemanticAnnotations();
  setNavigationGoalMode(false);
  drawNavigationMap(navigationVoxels, navigationPath, navigationCloud);
  updateNavigation({enabled: true, state: "stopped"});
  loadNavigationMap(true);
});
navigationShowCloud.addEventListener("change", () => drawNavigationMap(navigationVoxels, navigationPath, navigationCloud));
navigationShowTraversable.addEventListener("change", () => drawNavigationMap(navigationVoxels, navigationPath, navigationCloud));
navigationShowCostmap.addEventListener("change", () => drawNavigationMap(navigationVoxels, navigationPath, navigationCloud));
navigationShowObstacles.addEventListener("change", () => drawNavigationMap(navigationVoxels, navigationPath, navigationCloud));
navigationShowVoxels.addEventListener("change", () => drawNavigationMap(navigationVoxels, navigationPath, navigationCloud));
navigationShowSemantics.addEventListener("change", () => drawNavigationMap(navigationVoxels, navigationPath, navigationCloud));
navigationShowMappingOrigin.addEventListener("change", () => drawNavigationMap(navigationVoxels, navigationPath, navigationCloud));
navigationShowRobot.addEventListener("change", () => drawNavigationMap(navigationVoxels, navigationPath, navigationCloud));
function setNavigationGoalMode(enabled) {
  navigationGoalMode = enabled;
  if (enabled) semanticTool.value = "orbit";
  navigationGoalButton.classList.toggle("active", enabled);
  navigationGoalButton.textContent = enabled ? "请点击地图目标" : "选择目标点";
  voxelMapCanvas.classList.toggle("selecting-goal", enabled);
  updateSemanticControls();
}

navigationGoalButton.addEventListener("click", () => {
  if (navigationGoalButton.disabled) return;
  if (selectedGoalPending && !navigationGoalMode &&
      navigationStatus.planning_localization_ready) {
    sendNavigationGoal(selectedGoal);
    return;
  }
  setNavigationGoalMode(!navigationGoalMode);
  if (navigationGoalMode) showToast("请在地图中点击目标点；可先退出选点模式调整视角");
});

async function sendNavigationGoal(goal) {
  if (!goal) return;
  try {
    await api("/api/navigation/goal", goal);
    selectedGoalPending = false;
    setNavigationGoalMode(false);
    showToast(
      `地面目标已发送：${goal.x.toFixed(2)}, ${goal.y.toFixed(2)}, ${goal.z.toFixed(2)}`
    );
    refreshVoxelMap();
  } catch (error) {
    showToast(`规划请求失败：${error.message}`);
  }
}

async function selectNavigationGoal(event) {
  if (!voxelViewport) return;
  const point = nearestTraversableGoal(event);
  if (!point) return;
  const {x, y, z} = point;
  selectedGoal = {x, y, z};
  if (!navigationStatus.planning_localization_ready) {
    selectedGoalPending = true;
    setNavigationGoalMode(false);
    showToast(
      `目标已选：${x.toFixed(2)}, ${y.toFixed(2)}, ${z.toFixed(2)}；定位恢复后点击“提交已选目标”`
    );
    drawNavigationMap(navigationVoxels, navigationPath, navigationCloud);
    return;
  }
  await sendNavigationGoal(selectedGoal);
}

function nearestTraversableGoal(event) {
  const points = Array.isArray(navigationTerrain.traversable_points)
    ? navigationTerrain.traversable_points : [];
  if (!voxelViewport || !points.length) return null;
  const rect = voxelMapCanvas.getBoundingClientRect();
  const clickX = event.clientX - rect.left;
  const clickY = event.clientY - rect.top;
  let best = null;
  let bestDistanceSquared = Infinity;
  for (const point of points) {
    const screen = navigationPointToCanvas(point);
    if (!screen) continue;
    const distanceSquared =
      (screen.x - clickX) ** 2 + (screen.y - clickY) ** 2;
    if (distanceSquared < bestDistanceSquared) {
      best = point;
      bestDistanceSquared = distanceSquared;
    }
  }
  const terrainPixels =
    (Number(navigationTerrain.resolution) || 0.05) * voxelViewport.scale;
  const maximumDistance = Math.max(14, Math.min(30, terrainPixels * 2));
  if (!best || bestDistanceSquared > maximumDistance ** 2) {
    showToast("该位置附近没有可通行地面，请点击绿色可通行区域");
    return null;
  }
  return {x: Number(best[0]), y: Number(best[1]), z: Number(best[2])};
}

function canvasGroundPoint(event, groundZ) {
  if (!voxelViewport) return null;
  const rect = voxelMapCanvas.getBoundingClientRect();
  const horizontal = (event.clientX - rect.left - voxelViewport.width * 0.5) / voxelViewport.scale;
  const vertical = (voxelViewport.height * 0.5 - (event.clientY - rect.top)) / voxelViewport.scale;
  return mapProjection.unprojectGround(
    horizontal,
    vertical,
    [voxelViewport.centerX, voxelViewport.centerY, voxelViewport.centerZ],
    voxelViewport.view,
    groundZ,
  );
}

function navigationPointToCanvas(point) {
  if (!voxelViewport) return null;
  const projected = mapProjection.projectMapPoint(
    point,
    [voxelViewport.centerX, voxelViewport.centerY, voxelViewport.centerZ],
    voxelViewport.view,
  );
  return {
    x: voxelViewport.width * 0.5 + projected.horizontal * voxelViewport.scale,
    y: voxelViewport.height * 0.5 - projected.vertical * voxelViewport.scale,
  };
}

function pointInsidePolygon(x, y, polygon) {
  let inside = false;
  for (let index = 0, previous = polygon.length - 1;
    index < polygon.length;
    previous = index, index += 1) {
    const currentPoint = polygon[index];
    const previousPoint = polygon[previous];
    const intersects = ((currentPoint.y > y) !== (previousPoint.y > y)) &&
      (x < (previousPoint.x - currentPoint.x) * (y - currentPoint.y) /
        (previousPoint.y - currentPoint.y) + currentPoint.x);
    if (intersects) inside = !inside;
  }
  return inside;
}

function applySemanticBrush(event) {
  if (!semanticAnnotation || !voxelViewport) return;
  const rect = voxelMapCanvas.getBoundingClientRect();
  const cursorX = event.clientX - rect.left;
  const cursorY = event.clientY - rect.top;
  const radius = Math.max(3, Math.min(80, Number(semanticBrushRadius.value)));
  const type = semanticTool.value;
  const labels = new Map(
    (semanticAnnotation.occupied_labels || []).map(
      (label) => [semanticVoxelKey(label), label],
    ),
  );
  if (type === "erase") {
    for (const [key, label] of labels) {
      const screen = navigationPointToCanvas([label.x, label.y, label.z]);
      if (screen && Math.hypot(screen.x - cursorX, screen.y - cursorY) <= radius) {
        labels.delete(key);
      }
    }
    semanticAnnotation.pits = (semanticAnnotation.pits || []).filter((pit) => {
      const projected = (pit.polygon || []).map((point) =>
        navigationPointToCanvas([point[0], point[1], semanticGround().z])
      ).filter(Boolean);
      return projected.length < 3 || !pointInsidePolygon(cursorX, cursorY, projected);
    });
  } else {
    const ground = semanticGround();
    for (const point of navigationVoxels.points || []) {
      if (Number(point[2]) < ground.z + ground.minimum_height) continue;
      const screen = navigationPointToCanvas(point);
      if (!screen || Math.hypot(screen.x - cursorX, screen.y - cursorY) > radius) {
        continue;
      }
      labels.set(semanticVoxelKey(point), {
        type,
        x: Number(point[0]),
        y: Number(point[1]),
        z: Number(point[2]),
        size: Number(point[3]) || Number(navigationVoxels.resolution) || 0.05,
      });
    }
  }
  semanticAnnotation.ground = semanticGround();
  semanticAnnotation.occupied_labels = Array.from(labels.values());
  semanticDirty = true;
  updateSemanticControls();
  drawNavigationMap(navigationVoxels, navigationPath, navigationCloud);
}

function addSemanticPitVertex(event) {
  const point = canvasGroundPoint(event, semanticGround().z);
  if (!point || !semanticAnnotation) return;
  pushSemanticHistory();
  semanticDraftPit.push([
    Number(point.x.toFixed(3)),
    Number(point.y.toFixed(3)),
  ]);
  updateSemanticControls();
  drawNavigationMap(navigationVoxels, navigationPath, navigationCloud);
}

voxelMapCanvas.addEventListener("pointerdown", (event) => {
  if (navigationGoalMode) {
    selectNavigationGoal(event);
    return;
  }
  if (semanticAnnotation && semanticTool.value !== "orbit") {
    if (semanticTool.value === "pit") {
      addSemanticPitVertex(event);
      return;
    }
    pushSemanticHistory();
    semanticBrushActive = true;
    semanticBrushPointerId = event.pointerId;
    voxelMapCanvas.setPointerCapture?.(event.pointerId);
    applySemanticBrush(event);
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
  if (semanticBrushActive && semanticBrushPointerId === event.pointerId) {
    applySemanticBrush(event);
    return;
  }
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
  if (semanticBrushPointerId === event.pointerId) {
    if (voxelMapCanvas.hasPointerCapture?.(event.pointerId)) {
      voxelMapCanvas.releasePointerCapture(event.pointerId);
    }
    semanticBrushActive = false;
    semanticBrushPointerId = null;
    return;
  }
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

semanticTool.addEventListener("change", () => {
  if (semanticTool.value !== "orbit") {
    setNavigationGoalMode(false);
    navigationShowSemantics.checked = true;
  }
  updateSemanticControls();
  drawNavigationMap(navigationVoxels, navigationPath, navigationCloud);
});

function applySemanticGroundInputs() {
  if (!semanticAnnotation) return;
  semanticAnnotation.ground = semanticGround();
  semanticDirty = true;
  updateSemanticControls();
  drawNavigationMap(navigationVoxels, navigationPath, navigationCloud);
}

semanticGroundZ.addEventListener("change", applySemanticGroundInputs);
semanticMinimumHeight.addEventListener("change", applySemanticGroundInputs);

semanticFinishPitButton.addEventListener("click", () => {
  if (!semanticAnnotation || semanticDraftPit.length < 3) return;
  const depth = Number(semanticPitDepth.value);
  if (!Number.isFinite(depth) || depth <= 0) {
    showToast("坑深必须大于 0");
    return;
  }
  pushSemanticHistory();
  const usedIds = new Set((semanticAnnotation.pits || []).map((pit) => pit.id));
  let sequence = 1;
  while (usedIds.has(`pit_${String(sequence).padStart(3, "0")}`)) sequence += 1;
  semanticAnnotation.pits.push({
    id: `pit_${String(sequence).padStart(3, "0")}`,
    type: "pit",
    depth,
    polygon: semanticDraftPit,
  });
  semanticDraftPit = [];
  semanticDirty = true;
  updateSemanticControls();
  drawNavigationMap(navigationVoxels, navigationPath, navigationCloud);
});

semanticUndoButton.addEventListener("click", () => {
  const snapshot = semanticHistory.pop();
  if (!snapshot) return;
  semanticAnnotation = snapshot.annotation;
  semanticDraftPit = snapshot.draftPit;
  semanticDirty = true;
  if (semanticAnnotation) {
    semanticGroundZ.value = Number(semanticAnnotation.ground.z).toFixed(2);
    semanticMinimumHeight.value =
      Number(semanticAnnotation.ground.minimum_height).toFixed(2);
  }
  updateSemanticControls();
  drawNavigationMap(navigationVoxels, navigationPath, navigationCloud);
});

semanticReloadButton.addEventListener("click", async () => {
  const mapId = semanticAnnotation?.map_id;
  if (!mapId || (semanticDirty &&
    !window.confirm("放弃尚未保存的修改并重新载入标注？"))) return;
  try {
    await refreshSemanticAnnotations(mapId);
    showToast("标注已重新载入");
  } catch (error) {
    showToast(`标注重载失败：${error.message}`);
  }
});

semanticSaveButton.addEventListener("click", async () => {
  if (!semanticAnnotation) return;
  semanticAnnotation.ground = semanticGround();
  semanticSaveButton.disabled = true;
  try {
    const result = await api("/api/semantic/save", semanticAnnotation);
    semanticAnnotation = result.annotation;
    semanticDraftPit = [];
    semanticHistory = [];
    semanticDirty = false;
    updateSemanticControls();
    drawNavigationMap(navigationVoxels, navigationPath, navigationCloud);
    showToast("语义标注已校验并保存");
  } catch (error) {
    semanticDirty = true;
    updateSemanticControls();
    showToast(`标注保存失败：${error.message}`);
  }
});

function updateMapping(mapping) {
  if (!mapping) return;
  const mappingStateName = mappingStateNames[mapping.state] || mapping.state;
  mappingState.textContent = mappingStateName;
  mappingState.className = `mapping-state ${mapping.state}`;
  const calibrationRequired = (currentImuCalibration.service_available
    || currentImuCalibration.standalone_available)
    && currentImuCalibration.state !== "calibrated";
  mappingStartButton.disabled = !mapping.enabled || mapping.state === "running"
    || calibrationRequired;
  mappingStopButton.disabled = !mapping.enabled || mapping.state !== "running";
  mappingMode.disabled = !mapping.enabled || mapping.state === "running";
  if (mapping.state === "running" && mapping.mode) {
    mappingMode.value = mapping.mode;
    mappingModeInitialized = true;
  } else if (!mappingModeInitialized && mapping.default_mode) {
    mappingMode.value = mapping.default_mode;
    mappingModeInitialized = true;
  }
  const modeNames = {
    crestereo: "CREStereo 高质量 640×360",
    crestereo_max: "CREStereo 高帧率 10 Hz",
    vpi: "VPI",
  };
  const modeName = modeNames[mapping.mode] || mapping.mode;
  if (mapping.last_error) {
    mappingDetail.textContent = mapping.last_error;
  } else if (mapping.state === "running") {
    const uptime = mapping.uptime_seconds == null ? "" : ` · ${mapping.uptime_seconds}s`;
    mappingDetail.textContent = `${modeName} + Luxi 正在写入 RTAB-Map 数据库${uptime}`;
  } else if (!mapping.enabled) {
    mappingDetail.textContent = "当前节点未启用建图控制。";
  } else {
    mappingDetail.textContent = calibrationRequired
      ? "请先将机器人放在水平面并完成 IMU 一键校准。"
      : "CREStereo 为稳定主链路；极致模式输出 960×540 深度并冲刺 10 Hz；VPI 为备用链路。";
  }
}

async function startMapping() {
  mappingStartButton.disabled = true;
  try {
    const result = await api("/api/mapping/start", {mode: mappingMode.value});
    updateMapping(result.mapping);
    showToast("RTAB-Map 正在启动，请等待相机输入检查完成");
  } catch (error) {
    mappingStartButton.disabled = false;
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
    return;
  }
  const rgbLive = preview.rgb_age_seconds != null && preview.rgb_age_seconds < 3;
  setPreviewState(rgbPreviewState, rgbLive, rgbLive ? "实时" : "等待相机");
}

async function refreshRgbPreview() {
  if (rgbRefreshPending || document.hidden) return;
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

async function refreshStatus() {
  try {
    const response = await fetch("/api/status", {cache: "no-store"});
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    const data = await response.json();
    controlSessionActive = Boolean(data.control_session_active);
    if (controlSessionActive) cancelHeavyPreviewRequests();
    setConnection(true);
    topic.textContent = data.cmd_vel_topic;
    subscribers.textContent = String(data.subscriber_count);
    state.textContent = stateNames[data.state] || data.state;
    setEstopUi(Boolean(data.estop_active));
    updateRobotControl(data.robot_control);
    updateImuCalibration(data.imu_calibration);
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
setInterval(refreshRgbPreview, RGB_PREVIEW_INTERVAL_MS);
setInterval(refreshNavigationMaps, 2500);
setInterval(refreshVoxelMap, 1000);
window.addEventListener("resize", () => {
  refreshVoxelMap();
});
document.addEventListener("visibilitychange", () => {
  if (document.hidden) stop({keepalive: true});
});
window.addEventListener("pagehide", () => {
  held.clear();
  navigator.sendBeacon(
    "/api/stop",
    new Blob(
      [JSON.stringify({client_id: controlClientId})],
      {type: "application/json"},
    ),
  );
});

updateSpeeds();
refreshStatus();
refreshRgbPreview();
refreshNavigationMaps();
refreshVoxelMap();
