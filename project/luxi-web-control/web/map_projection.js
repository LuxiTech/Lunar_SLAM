(function initializeMapProjection(root, factory) {
  const projection = factory();
  if (typeof module === "object" && module.exports) module.exports = projection;
  root.LuxiMapProjection = projection;
}(typeof globalThis !== "undefined" ? globalThis : this, () => {
  const defaultView = Object.freeze({
    yaw: Math.PI / 2,
    // Prefer a map-like overview so vertical walls read as contours instead
    // of overlapping radial streaks. Dragging still exposes the full 3-D view.
    pitch: 1.15,
    zoom: 1.0,
  });

  function projectMapPoint(point, center, view) {
    const dx = Number(point[0] || 0) - center[0];
    const dy = Number(point[1] || 0) - center[1];
    const dz = Number(point[2] || 0) - center[2];
    const cosineYaw = Math.cos(view.yaw);
    const sineYaw = Math.sin(view.yaw);
    const cosinePitch = Math.cos(view.pitch);
    const sinePitch = Math.sin(view.pitch);
    const horizontal = cosineYaw * dx - sineYaw * dy;
    const depth = sineYaw * dx + cosineYaw * dy;
    const vertical = cosinePitch * dz + sinePitch * depth;
    return {horizontal, vertical, depth};
  }

  function unprojectGround(horizontal, vertical, center, view, groundZ = 0) {
    const cosineYaw = Math.cos(view.yaw);
    const sineYaw = Math.sin(view.yaw);
    const cosinePitch = Math.cos(view.pitch);
    const sinePitch = Math.sin(view.pitch);
    if (Math.abs(sinePitch) < 1e-6) {
      throw new Error("map view pitch is too small for ground selection");
    }
    const depth = (
      vertical - cosinePitch * (groundZ - center[2])
    ) / sinePitch;
    return {
      x: center[0] + cosineYaw * horizontal + sineYaw * depth,
      y: center[1] - sineYaw * horizontal + cosineYaw * depth,
    };
  }

  return {defaultView, projectMapPoint, unprojectGround};
}));

(function installNavigationUi(global) {
  const followerLabels = {
    localization_dead_reckoning: "定位短时推算中（前方净空，限距 8 cm）",
    localization_recovery_waiting: "定位丢失，已停止平移并等待安全旋转",
    localization_recovery_spin: "正在安全原地旋转，持续恢复定位",
    localization_recovery_verifying: "发现定位候选，停车验证中",
    localization_recovery_confirming: "定位恢复确认中",
    replanning_after_relocalization: "定位恢复，正在重新规划",
    replanning_waiting_for_clear: "定位恢复，等待安全绕行路线",
    replan_after_relocalization_timeout: "恢复后重规划超时，已停车",
    traction_boost: "检测到无位移，正在短时增力",
    aligning_goal_heading: "已到目标位置，正在对准到达方向",
    obstacle_recovery_spin: "前方受阻，正在安全原地转向",
    obstacle_recovery_timeout: "转向仍无法脱困，已安全停车",
    stuck_no_progress: "增力后仍无位移，已安全停车",
    goal_reached: "导航结束：已到达目标点",
  };

  function followerLabel(state) {
    if (followerLabels[state]) return followerLabels[state];
    if (state?.startsWith("localization_recovery_")) return "定位恢复中";
    return state || "--";
  }

  function recoveryMessage(navigation, mapName = "所选地图") {
    const state = navigation?.follower_state;
    if (state === "localization_dead_reckoning") {
      return `${mapName} 暂时失去地图匹配；仅因丢失前最后一次前方净空且当前深度仍净空，` +
        "导航核心允许纯前向续行，速度不超过 0.10 m/s、距离不超过 8 cm。";
    }
    if (state === "localization_recovery_waiting") {
      return `${mapName} 定位已丢失且不满足安全续行条件，机器人保持零平移；` +
        "周身旋转空间确认安全后开始原地搜索。";
    }
    if (state === "localization_recovery_spin") {
      return `${mapName} 已停止平移，正在导航安全门监控下沿原路径方向原地旋转；` +
        "旋转没有时间上限，定位稳定恢复或用户停止任务后才结束。";
    }
    if (state === "localization_recovery_verifying" ||
        state === "localization_recovery_confirming") {
      return `${mapName} 已发现定位候选，车辆保持零速度进行连续匹配验证；确认后自动重规划。`;
    }
    if (state === "replanning_after_relocalization") {
      return `${mapName} 定位已经重新确认，正在从修正后的位置到原目标重新规划；新路径生成前保持停车。`;
    }
    if (state === "replanning_waiting_for_clear") {
      return `${mapName} 正在使用重建后的深度障碍层寻找安全路径；原目标仍保留，路径可用后自动继续。`;
    }
    return "";
  }

  function isRecoveryState(state) {
    return state === "localization_dead_reckoning" ||
      state?.startsWith("localization_recovery_") ||
      state?.startsWith("replanning_");
  }

  global.LuxiNavigationUi = Object.freeze({
    followerLabel,
    recoveryMessage,
    isRecoveryState,
  });
})(typeof globalThis !== "undefined" ? globalThis : this);
