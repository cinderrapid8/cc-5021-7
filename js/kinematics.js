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
   * ordered node list (deterministic pre-order DFS).
   *
   * overrides: optional Map<nodeId, displayValue> replacing the curve output
   * for that node — used by the closure solver, which owns the free joints.
   * Driven values are never rewritten by this path. */
  function evaluate(rig, t, overrides) {
    const { order, errors } = MR.Rig.validateHierarchy(rig.nodes);
    const byId = new Map(rig.nodes.map((n) => [n.id, n]));
    const frames = new Map();
    const Mroot = rootMatrix(rig);

    for (const node of order) {
      let raw;
      let limit;
      const ov = overrides?.get(node.id);
      if (ov !== undefined) {
        // Solver-provided value: still run through the limit policy so the UI
        // can show when the constraint solution wants to leave the joint range.
        raw = ov;
        limit = applyJointLimit(node, ov);
      } else {
        raw = MR.Rig.evalCurve(node.curve, t);
        limit = applyJointLimit(node, raw);
      }
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
        solverOwned: ov !== undefined,
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

  /* World position of an anchor offset attached to a node, for a given pose. */
  function anchorWorld(frame, offset) {
    return M4.transformPoint(frame.world, offset);
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

  /* Planar four-bar reference (links along +X, joint axes +Z).
   * Ground pivots O=(0,0), B=(d,0); crank angle theta2 is the sole input.
   * Returns absolute orientation (rad) of the coupler and rocker plus the
   * coupler point P = B + c·(cos q4, sin q4). branch = +1 / -1 selects the
   * assembly mode. NaN fields mean the input is outside the reachable range.
   *
   * Derived from the closure a·u2 + b·u3 = d·ex + c·u4 and the two
   * Freudenstein equations, solved with the tan(theta/2) substitution. */
  function fourBarAnalytic(a, b, c, d, theta2, branch) {
    const s = branch >= 0 ? 1 : -1;
    const k1 = d / a;
    const k2 = d / c;
    const k3 = (a * a - b * b + c * c + d * d) / (2 * a * c);
    const k4 = d / b;
    const k5 = (c * c - d * d - a * a - b * b) / (2 * a * b);
    const ct = Math.cos(theta2), st = Math.sin(theta2);

    // A t^2 + B t + C = 0, t = tan(q/2); stable for A ~ 0.
    const halfAngle = (A, B, C, sign) => {
      const disc = B * B - 4 * A * C;
      if (disc < -1e-10) return null;
      const root = Math.sqrt(Math.max(0, disc));
      if (Math.abs(A) < 1e-12) {
        if (Math.abs(B) < 1e-12) return Math.PI; // equation degenerates: q = pi
        return 2 * Math.atan(-C / B);
      }
      const t = sign > 0 ? (-B + root) / (2 * A) : (-B - root) / (2 * A);
      return 2 * Math.atan(t);
    };

    const q4 = halfAngle(
      k1 - k3 + ct * (k2 - 1), 2 * st,
      ct * (1 + k2) - k1 - k3, s
    );
    const q3 = halfAngle(
      k1 - k5 - ct * (1 + k4), 2 * st,
      ct * (1 - k4) - k1 - k5, s
    );
    if (q4 === null || q3 === null) {
      return { reachable: false, q3: NaN, q4: NaN, P: [NaN, NaN, NaN] };
    }
    const P = [d + c * Math.cos(q4), c * Math.sin(q4), 0];
    return { reachable: true, q3, q4, P };
  }

  /* Planar 2R inverse kinematics reference. Base O, link lengths L1, L2,
   * target point T in the XY plane. Returns absolute joint orientations
   * (rad) for the requested branch, or null when T is out of reach. */
  function twoRikAnalytic(O, L1, L2, T, branch) {
    const dx = T[0] - O[0];
    const dy = T[1] - O[1];
    const r = Math.hypot(dx, dy);
    if (r > L1 + L2 + 1e-12 || r < Math.abs(L1 - L2) - 1e-12) return null;
    const phi = Math.atan2(dy, dx);
    const cosElbow = (r * r - L1 * L1 - L2 * L2) / (2 * L1 * L2);
    const elbow = Math.acos(Math.max(-1, Math.min(1, cosElbow)));
    const s = branch >= 0 ? 1 : -1;
    const q2rel = s * elbow; // relative angle at the elbow
    const q1 = phi - Math.atan2(L2 * Math.sin(q2rel), L1 + L2 * Math.cos(q2rel));
    return { q1, q2: q1 + q2rel, elbow: q2rel };
  }

  MR.Kinematics = {
    evaluate, applyJointLimit, mountMatrix, rootMatrix, jointMotionMatrix,
    anchorWorld, twoLinkAnalytic, slideAnalytic,
    fourBarAnalytic, twoRikAnalytic
  };
})(typeof window !== 'undefined' ? window : globalThis);
