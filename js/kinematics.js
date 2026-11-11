/* kinematics.js — forward kinematics evaluator.
 *
 * Per-node local frame chain:
 *   M_world(node) = M_world(parent) · M_mount(node) · M_joint(node)
 * M_mount holds the installation offset/euler (geometry placement).
 * M_joint holds ONLY the current joint motion. Because the child's joint axis
 * is defined in its own local frame and transformed by parent rotation, a
 * parent rotation rotates the child axis in the world — it is never pinned to
 * a world direction. */
(function (global) {
  'use strict';

  const MR = global.MR;
  const { M4, V3 } = MR;

  function deg2rad(d) { return (d * Math.PI) / 180; }

  function mountMatrix(node) {
    const t = M4.fromTranslation(node.mount.offset);
    const [rx, ry, rz] = node.mount.eulerDeg.map(deg2rad);
    const r = M4.fromEulerXYZ(rx, ry, rz);
    return M4.multiply(t, r);
  }

  function rootMatrix(rig) {
    const t = M4.fromTranslation(rig.rootTransform.position);
    const [rx, ry, rz] = rig.rootTransform.eulerDeg.map(deg2rad);
    const r = M4.fromEulerXYZ(rx, ry, rz);
    return M4.multiply(t, r);
  }

  /* Apply the joint's declared limit policy to a raw curve value.
   * Returns the applied display value plus a visible status.
   * Geometry dimensions are never consulted or modified. */
  function applyJointLimit(node, raw) {
    const j = node.joint;
    if (j.type === 'fixed') {
      return { raw, applied: 0, status: 'fixed', clamped: false };
    }
    if (j.limitType === 'continuous' || j.type === 'continuous') {
      // continuous rotation: wrap to (-180, 180] deg or (-pi, pi] rad
      const span = j.unit === 'rad' ? 2 * Math.PI : 360;
      let v = raw % span;
      if (v > span / 2) v -= span;
      if (v <= -span / 2) v += span;
      return { raw, applied: v, status: 'continuous', clamped: false };
    }
    if (raw < j.min) {
      return { raw, applied: j.min, status: 'clamped-low', clamped: true };
    }
    if (raw > j.max) {
      return { raw, applied: j.max, status: 'clamped-high', clamped: true };
    }
    return { raw, applied: raw, status: 'in-range', clamped: false };
  }

  function jointMotionMatrix(node, appliedDisplay) {
    const j = node.joint;
    const axis = V3.norm(j.axis);
    if (j.type === 'prismatic') {
      const d = MR.Rig.jointValueToBase(node, appliedDisplay);
      return M4.fromAxisTranslation(axis, d);
    }
    if (j.type === 'revolute' || j.type === 'continuous') {
      const angle = MR.Rig.jointValueToBase(node, appliedDisplay);
      return M4.fromAxisAngle(axis, angle);
    }
    return M4.identity();
  }

  /* Evaluate the whole rig at time t (seconds).
   * Returns a pose snapshot: per-node frames, values, limit status and the
   * ordered node list (deterministic pre-order DFS). */
  function evaluate(rig, t) {
    const { order, errors } = MR.Rig.validateHierarchy(rig.nodes);
    const byId = new Map(rig.nodes.map((n) => [n.id, n]));
    const frames = new Map();
    const Mroot = rootMatrix(rig);

    for (const node of order) {
      const raw = MR.Rig.evalCurve(node.curve, t);
      const limit = applyJointLimit(node, raw);
      const Mmount = mountMatrix(node);
      const Mjoint = jointMotionMatrix(node, limit.applied);
      const Mlocal = M4.multiply(Mmount, Mjoint);

      let Mworld;
      if (node.parentId === null || !byId.has(node.parentId)) {
        Mworld = M4.multiply(Mroot, Mlocal);
      } else {
        const parentFrame = frames.get(node.parentId);
        Mworld = M4.multiply(parentFrame.world, Mlocal);
      }

      const axisLocal = V3.norm(node.joint.axis);
      const axisWorld = V3.norm(M4.transformDir(Mworld, axisLocal));
      frames.set(node.id, {
        node,
        raw,
        applied: limit.applied,
        status: limit.status,
        clamped: limit.clamped,
        mount: Mmount,
        joint: Mjoint,
        local: Mlocal,
        world: Mworld,
        position: M4.getTranslation(Mworld),
        axisLocal,
        axisWorld
      });
    }

    let tipPosition = null;
    const tipId = rig.tipNodeId;
    if (tipId && frames.has(tipId)) tipPosition = frames.get(tipId).position;

    return {
      t,
      order,
      errors,
      frames,
      tipPosition,
      anyClamped: [...frames.values()].some((f) => f.clamped)
    };
  }

  /* Closed-form reference for the two-link sample (XY plane, axes +Z).
   * theta1/theta2 in radians; theta2 measured relative to link 1. */
  function twoLinkAnalytic(origin, L1, L2, theta1, theta2) {
    const phi = theta1 + theta2;
    return [
      origin[0] + L1 * Math.sin(theta1) + L2 * Math.sin(phi),
      origin[1] - L1 * Math.cos(theta1) - L2 * Math.cos(phi),
      origin[2] || 0
    ];
  }

  /* Closed-form reference for the slide sample:
   * carriage moves along +X by d, then an arm of length L at angle theta. */
  function slideAnalytic(d, L, theta) {
    return [
      d + L * Math.sin(theta),
      0.04 - L * Math.cos(theta),
      0
    ];
  }

  MR.Kinematics = {
    evaluate, applyJointLimit, mountMatrix, rootMatrix, jointMotionMatrix,
    twoLinkAnalytic, slideAnalytic
  };
})(typeof window !== 'undefined' ? window : globalThis);
