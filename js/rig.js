/* rig.js — mechanical rig model: nodes, joints, curves, units, validation.
 * Design rule: mount offsets (geometry placement) and joint motion are stored
 * separately. Joint limits never resize geometry. */
(function (global) {
  'use strict';

  const MR = global.MR;
  let _seq = 1;
  const nid = (prefix) => `${prefix}_${(_seq++).toString(36)}${Math.random().toString(36).slice(2, 6)}`;

  /* ---------- units ---------- */
  const ANGLE_UNITS = ['deg', 'rad'];
  const LENGTH_UNITS = ['mm', 'cm', 'm'];
  const ANGLE_TO_RAD = { deg: Math.PI / 180, rad: 1 };
  const LENGTH_TO_M = { mm: 0.001, cm: 0.01, m: 1 };

  function createNode(partial) {
    const node = {
      id: partial.id || nid('node'),
      name: partial.name || 'Node',
      enabled: partial.enabled !== false,
      parentId: partial.parentId === undefined ? null : partial.parentId,

      // Mounting transform: where the joint frame is attached on the parent.
      // Pure placement, never touched by joint motion or limit enforcement.
      mount: {
        offset: Array.isArray(partial.mount?.offset) ? [...partial.mount.offset] : [0, 0, 0],
        eulerDeg: Array.isArray(partial.mount?.eulerDeg) ? [...partial.mount.eulerDeg] : [0, 0, 0]
      },

      // Joint. type: 'revolute' | 'prismatic' | 'fixed' | 'continuous'.
      joint: {
        type: partial.joint?.type || 'fixed',
        axis: Array.isArray(partial.joint?.axis) ? [...partial.joint.axis] : [0, 0, 1],
        unit: partial.joint?.unit || 'deg',
        limitType: partial.joint?.limitType || 'range', // 'range' | 'continuous'
        min: partial.joint?.min ?? -90,
        max: partial.joint?.max ?? 90
      },

      // Time curve driving the joint value (in the joint's display unit).
      curve: {
        mode: partial.curve?.mode || 'hold', // hold | sine | linear | keyframes
        base: partial.curve?.base ?? 0,
        amplitude: partial.curve?.amplitude ?? 30,
        period: partial.curve?.period ?? 4,
        phase: partial.curve?.phase ?? 0,
        speed: partial.curve?.speed ?? 1,
        keyframes: Array.isArray(partial.curve?.keyframes)
          ? partial.curve.keyframes.map((k) => ({ t: k.t, v: k.v }))
          : [{ t: 0, v: 0 }, { t: 2, v: 30 }, { t: 4, v: 0 }]
      },

      // Purely visual part. Dimensions are independent of joint values.
      part: {
        shape: partial.part?.shape || 'box', // box | cylinder | sphere | none
        size: Array.isArray(partial.part?.size) ? [...partial.part.size] : [0.1, 0.1, 0.1],
        color: partial.part?.color || '#6ea8ff',
        // where the part mesh is drawn relative to this node's joint frame
        partOffset: Array.isArray(partial.part?.partOffset) ? [...partial.part.partOffset] : [0, 0, 0]
      },

      showAxes: !!partial.showAxes
    };
    return node;
  }

  function createRig(name) {
    return {
      version: 1,
      name: name || 'Rig',
      rootTransform: {
        position: [0, 0, 0],
        eulerDeg: [0, 0, 0]
      },
      nodes: []
    };
  }

  /* ---------- curves ---------- */
  function evalCurve(curve, t) {
    switch (curve.mode) {
      case 'hold':
        return curve.base;
      case 'sine':
        return curve.base + curve.amplitude * Math.sin(
          (2 * Math.PI * t) / Math.max(curve.period, 1e-6) + curve.phase
        );
      case 'linear':
        return curve.base + curve.speed * t;
      case 'keyframes': {
        const ks = [...curve.keyframes].sort((a, b) => a.t - b.t);
        if (ks.length === 0) return 0;
        if (t <= ks[0].t) return ks[0].v;
        for (let i = 1; i < ks.length; i++) {
          if (t <= ks[i].t) {
            const a = ks[i - 1], b = ks[i];
            const f = (t - a.t) / Math.max(b.t - a.t, 1e-9);
            return a.v + (b.v - a.v) * f;
          }
        }
        return ks[ks.length - 1].v;
      }
      default:
        return 0;
    }
  }

  /* Convert a joint value from its display unit into the solver base unit
   * (radians for rotation, metres for translation). */
  function jointValueToBase(node, displayValue) {
    if (node.joint.type === 'prismatic') {
      return displayValue * (LENGTH_TO_M[node.joint.unit] || 1);
    }
    return displayValue * (ANGLE_TO_RAD[node.joint.unit] || 1);
  }

  /* ---------- hierarchy validation ---------- */
  /* Returns { order, errors }. order is the guaranteed evaluation order
   * (pre-order DFS from root-orphans). Errors list cycles / dangling refs. */
  function validateHierarchy(nodes) {
    const byId = new Map();
    const errors = [];
    for (const n of nodes) {
      if (byId.has(n.id)) errors.push(`重复节点 id: ${n.id}`);
      byId.set(n.id, n);
    }
    const childCount = new Map();
    for (const n of nodes) {
      if (n.parentId !== null) {
        if (!byId.has(n.parentId)) {
          errors.push(`节点「${n.name}」(${n.id}) 的父引用 ${n.parentId} 不存在（失效引用），已按根节点处理`);
        } else {
          childCount.set(n.parentId, (childCount.get(n.parentId) || 0) + 1);
        }
      }
    }
    // cycle detection via DFS coloring: 0=white 1=gray 2=black
    const color = new Map();
    const order = [];
    const visit = (id, stack) => {
      const c = color.get(id) || 0;
      if (c === 1) {
        const chain = stack.concat(id).map((x) => byId.get(x)?.name || x).join(' → ');
        errors.push(`检测到层级循环: ${chain}；循环链已断开，链尾节点按根节点处理`);
        return false;
      }
      if (c === 2) return true;
      color.set(id, 1);
      order.push(id);
      for (const n of nodes) {
        if (n.parentId === id) visit(n.id, stack.concat(id));
      }
      color.set(id, 2);
      return true;
    };
    // roots first: null parent, dangling parent, or nodes that would close a cycle
    for (const n of nodes) {
      const dangling = n.parentId !== null && !byId.has(n.parentId);
      if (n.parentId === null || dangling) visit(n.id, []);
    }
    // anything not visited is part of a cycle whose root's parent is inside the cycle
    for (const n of nodes) {
      if (!color.has(n.id)) {
        // break the cycle by treating this node as a root
        errors.push(`节点「${n.name}」形成无入口循环，已从其父节点断开`);
        visit(n.id, []);
      }
    }
    return { order: order.map((id) => byId.get(id)).filter(Boolean), errors };
  }

  /* Parent change / delete checks used by the UI. */
  function canReparent(nodes, nodeId, newParentId) {
    if (newParentId === null) return { ok: true };
    if (newParentId === nodeId) return { ok: false, reason: '不能将节点挂到自身' };
    const byId = new Map(nodes.map((n) => [n.id, n]));
    if (!byId.has(newParentId)) return { ok: false, reason: '目标父节点不存在' };
    // walk up from candidate parent; if nodeId appears, it would create a cycle
    let cur = newParentId;
    const seen = new Set();
    while (cur !== null) {
      if (cur === nodeId) return { ok: false, reason: '不能挂到自己的子孙节点（会产生循环）' };
      if (seen.has(cur)) return { ok: false, reason: '现有层级已含循环，请先修复' };
      seen.add(cur);
      cur = byId.get(cur)?.parentId ?? null;
      if (cur !== null && !byId.has(cur)) break;
    }
    return { ok: true };
  }

  function safeDelete(nodes, nodeId) {
    const byId = new Map(nodes.map((n) => [n.id, n]));
    if (!byId.has(nodeId)) return { ok: false, reason: '节点不存在' };
    const children = nodes.filter((n) => n.parentId === nodeId);
    if (children.length > 0) {
      return { ok: false, reason: `该节点仍有 ${children.length} 个子节点，请先移走或删除子节点` };
    }
    return { ok: true };
  }

  /* ---------- sample rigs ---------- */
  function buildMechCharacter() {
    const rig = createRig('机械角色 · 双臂演示');
    const base = createNode({
      name: '底座', parentId: null,
      mount: { offset: [0, 0, 0] },
      joint: { type: 'continuous', axis: [0, 1, 0], unit: 'deg', limitType: 'continuous', min: 0, max: 0 },
      curve: { mode: 'sine', base: 0, amplitude: 45, period: 12, phase: 0 },
      part: { shape: 'cylinder', size: [0.28, 0.08, 0.28], color: '#8a93a6', partOffset: [0, 0.04, 0] },
      showAxes: false
    });
    const waist = createNode({
      name: '腰转台', parentId: base.id,
      mount: { offset: [0, 0.08, 0] },
      joint: { type: 'revolute', axis: [0, 1, 0], unit: 'deg', limitType: 'range', min: -70, max: 70 },
      curve: { mode: 'sine', base: 0, amplitude: 55, period: 6, phase: 0 },
      part: { shape: 'cylinder', size: [0.18, 0.12, 0.18], color: '#ffb347', partOffset: [0, 0.06, 0] }
    });
    const torso = createNode({
      name: '躯干', parentId: waist.id,
      mount: { offset: [0, 0.12, 0] },
      joint: { type: 'fixed', axis: [0, 1, 0], unit: 'deg', limitType: 'range', min: 0, max: 0 },
      curve: { mode: 'hold', base: 0, amplitude: 0 },
      part: { shape: 'box', size: [0.3, 0.34, 0.18], color: '#6ea8ff', partOffset: [0, 0.17, 0] }
    });
    const eye = createNode({
      name: '视觉眼', parentId: torso.id,
      mount: { offset: [0, 0.28, 0.095] },
      joint: { type: 'fixed' },
      part: { shape: 'sphere', size: [0.05, 0.05, 0.05], color: '#7dffc8' }
    });
    const shoulder = createNode({
      name: '肩关节', parentId: torso.id,
      mount: { offset: [0.2, 0.26, 0] },
      joint: { type: 'revolute', axis: [0, 0, 1], unit: 'deg', limitType: 'range', min: -120, max: 90 },
      curve: { mode: 'sine', base: 20, amplitude: 70, period: 4, phase: 0 },
      part: { shape: 'sphere', size: [0.07, 0.07, 0.07], color: '#ff6b8a' }
    });
    const upperArm = createNode({
      name: '上臂', parentId: shoulder.id,
      mount: { offset: [0, 0, 0] },
      joint: { type: 'fixed' },
      part: { shape: 'cylinder', size: [0.045, 0.3, 0.045], color: '#c9d4ea', partOffset: [0, -0.15, 0] }
    });
    const elbow = createNode({
      name: '肘关节', parentId: upperArm.id,
      mount: { offset: [0, -0.3, 0] },
      joint: { type: 'revolute', axis: [0, 0, 1], unit: 'deg', limitType: 'range', min: 0, max: 140 },
      curve: { mode: 'keyframes', base: 0, keyframes: [
        { t: 0, v: 20 }, { t: 1.5, v: 120 }, { t: 3, v: 40 }, { t: 4.5, v: 130 }, { t: 6, v: 20 }
      ] },
      part: { shape: 'sphere', size: [0.06, 0.06, 0.06], color: '#ff6b8a' }
    });
    const slide = createNode({
      name: '伸缩滑轨', parentId: elbow.id,
      mount: { offset: [0, 0, 0] },
      joint: { type: 'prismatic', axis: [0, -1, 0], unit: 'cm', limitType: 'range', min: 0, max: 25 },
      curve: { mode: 'sine', base: 12, amplitude: 12, period: 3, phase: Math.PI / 2 },
      part: { shape: 'box', size: [0.05, 0.05, 0.05], color: '#9d8cff' }
    });
    const forearm = createNode({
      name: '前臂杆', parentId: slide.id,
      mount: { offset: [0, 0, 0] },
      joint: { type: 'fixed' },
      part: { shape: 'cylinder', size: [0.035, 0.24, 0.035], color: '#c9d4ea', partOffset: [0, -0.12, 0] }
    });
    const tip = createNode({
      name: '末端标记', parentId: forearm.id,
      mount: { offset: [0, -0.24, 0] },
      joint: { type: 'fixed' },
      part: { shape: 'sphere', size: [0.055, 0.055, 0.055], color: '#7dffc8' },
      showAxes: true
    });
    rig.nodes.push(base, waist, torso, eye, shoulder, upperArm, elbow, slide, forearm, tip);
    rig.tipNodeId = tip.id;
    return rig;
  }

  /* Analytical two-link arm in the XY plane, joint axes = +Z. */
  function buildTwoLink() {
    const rig = createRig('两节臂 · 解析核对');
    const j1 = createNode({
      name: '根旋转关节 A', parentId: null,
      mount: { offset: [0, 0, 0] },
      joint: { type: 'revolute', axis: [0, 0, 1], unit: 'deg', limitType: 'range', min: -360, max: 360 },
      curve: { mode: 'hold', base: 30, amplitude: 0 },
      part: { shape: 'cylinder', size: [0.06, 0.05, 0.06], color: '#ff6b8a' }
    });
    const link1 = createNode({
      name: '连杆 1 (L1=1m)', parentId: j1.id,
      mount: { offset: [0, 0, 0] },
      joint: { type: 'fixed' },
      part: { shape: 'cylinder', size: [0.03, 1, 0.03], color: '#c9d4ea', partOffset: [0, -0.5, 0] }
    });
    const j2 = createNode({
      name: '肘关节 B', parentId: link1.id,
      mount: { offset: [0, -1, 0] },
      joint: { type: 'revolute', axis: [0, 0, 1], unit: 'deg', limitType: 'range', min: -360, max: 360 },
      curve: { mode: 'hold', base: 60, amplitude: 0 },
      part: { shape: 'sphere', size: [0.06, 0.06, 0.06], color: '#ffb347' }
    });
    const link2 = createNode({
      name: '连杆 2 (L2=0.8m)', parentId: j2.id,
      mount: { offset: [0, 0, 0] },
      joint: { type: 'fixed' },
      part: { shape: 'cylinder', size: [0.025, 0.8, 0.025], color: '#9d8cff', partOffset: [0, -0.4, 0] }
    });
    const tip = createNode({
      name: '末端', parentId: link2.id,
      mount: { offset: [0, -0.8, 0] },
      joint: { type: 'fixed' },
      part: { shape: 'sphere', size: [0.05, 0.05, 0.05], color: '#7dffc8' },
      showAxes: true
    });
    rig.nodes.push(j1, link1, j2, link2, tip);
    rig.tipNodeId = tip.id;
    return rig;
  }

  /* Linear slide + one revolute arm. */
  function buildSlide() {
    const rig = createRig('滑轨 + 单臂 · 解析核对');
    const carriage = createNode({
      name: '水平滑台 (X轴)', parentId: null,
      mount: { offset: [0, 0, 0] },
      joint: { type: 'prismatic', axis: [1, 0, 0], unit: 'm', limitType: 'range', min: -1, max: 1 },
      curve: { mode: 'sine', base: 0.4, amplitude: 0.4, period: 5, phase: 0 },
      part: { shape: 'box', size: [0.12, 0.08, 0.12], color: '#9d8cff' }
    });
    const j1 = createNode({
      name: '立柱旋转关节', parentId: carriage.id,
      mount: { offset: [0, 0.04, 0] },
      joint: { type: 'revolute', axis: [0, 0, 1], unit: 'deg', limitType: 'range', min: -180, max: 180 },
      curve: { mode: 'hold', base: 45, amplitude: 0 },
      part: { shape: 'cylinder', size: [0.05, 0.05, 0.05], color: '#ff6b8a' }
    });
    const link = createNode({
      name: '连杆 (L=0.7m)', parentId: j1.id,
      mount: { offset: [0, 0, 0] },
      joint: { type: 'fixed' },
      part: { shape: 'cylinder', size: [0.025, 0.7, 0.025], color: '#c9d4ea', partOffset: [0, -0.35, 0] }
    });
    const tip = createNode({
      name: '末端', parentId: link.id,
      mount: { offset: [0, -0.7, 0] },
      joint: { type: 'fixed' },
      part: { shape: 'sphere', size: [0.05, 0.05, 0.05], color: '#7dffc8' },
      showAxes: true
    });
    rig.nodes.push(carriage, j1, link, tip);
    rig.tipNodeId = tip.id;
    return rig;
  }

  /* Rig whose curves deliberately overshoot joint ranges. */
  function buildLimitDemo() {
    const rig = createRig('越界限制策略演示');
    const j1 = createNode({
      name: '受限转动 (±45°)', parentId: null,
      joint: { type: 'revolute', axis: [0, 0, 1], unit: 'deg', limitType: 'range', min: -45, max: 45 },
      curve: { mode: 'sine', base: 0, amplitude: 90, period: 6 },
      part: { shape: 'cylinder', size: [0.06, 0.05, 0.06], color: '#ff6b8a' }
    });
    const link = createNode({
      name: '连杆', parentId: j1.id,
      joint: { type: 'fixed' },
      part: { shape: 'cylinder', size: [0.03, 0.6, 0.03], color: '#c9d4ea', partOffset: [0, -0.3, 0] }
    });
    const slide = createNode({
      name: '受限滑动 (0~10cm)', parentId: link.id,
      mount: { offset: [0, -0.6, 0] },
      joint: { type: 'prismatic', axis: [0, -1, 0], unit: 'cm', limitType: 'range', min: 0, max: 10 },
      curve: { mode: 'sine', base: 5, amplitude: 15, period: 5 },
      part: { shape: 'box', size: [0.05, 0.05, 0.05], color: '#9d8cff' }
    });
    const tip = createNode({
      name: '末端', parentId: slide.id,
      joint: { type: 'fixed' },
      part: { shape: 'sphere', size: [0.05, 0.05, 0.05], color: '#7dffc8' },
      showAxes: true
    });
    rig.nodes.push(j1, link, slide, tip);
    rig.tipNodeId = tip.id;
    return rig;
  }

  MR.Rig = {
    nid, createNode, createRig, evalCurve, jointValueToBase,
    validateHierarchy, canReparent, safeDelete,
    buildMechCharacter, buildTwoLink, buildSlide, buildLimitDemo,
    ANGLE_UNITS, LENGTH_UNITS, ANGLE_TO_RAD, LENGTH_TO_M
  };
})(typeof window !== 'undefined' ? window : globalThis);
