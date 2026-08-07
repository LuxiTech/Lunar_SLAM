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
