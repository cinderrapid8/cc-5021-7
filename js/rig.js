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

      showAxes: !!partial.showAxes,

      // Closure-loop role. A driven joint takes its value from its time curve
      // (the actuator); a free joint (driven=false) is an unknown solved from
      // the closure constraints. Fixed joints can never be free.
      driven: partial.driven !== undefined ? !!partial.driven : true
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
      nodes: [],
      // Closure constraints coupling nodes of the (still tree-shaped) hierarchy.
      // type: 'coincident' | 'distance'. Empty array => pure tree, direct FK.
      constraints: []
    };
  }

  /* ---------- closure constraints ---------- */
  let _cseq = 1;
  const cid = () => `cstr_${(_cseq++).toString(36)}${Math.random().toString(36).slice(2, 6)}`;
  const CONSTRAINT_TYPES = ['coincident', 'distance'];
  const CONSTRAINT_AXES = ['x', 'y', 'z'];

  function createConstraint(partial) {
    const c = {
      id: partial.id || cid(),
      name: partial.name || '闭合约束',
      type: CONSTRAINT_TYPES.includes(partial.type) ? partial.type : 'coincident',
      nodeA: partial.nodeA || null,
      nodeB: partial.nodeB || null,
      // Anchor points measured in each node's own joint frame (metres).
      offsetA: Array.isArray(partial.offsetA) ? [...partial.offsetA] : [0, 0, 0],
      offsetB: Array.isArray(partial.offsetB) ? [...partial.offsetB] : [0, 0, 0],
      // distance constraints only: required separation (metres).
      distance: partial.distance ?? 0,
      // Which world axes participate. Planar four-bars use ['x','y']; a full
      // 3D pin uses all three. Fewer equations = fewer required unknowns.
      axes: Array.isArray(partial.axes) && partial.axes.length
        ? partial.axes.filter((a) => CONSTRAINT_AXES.includes(a))
        : ['x', 'y', 'z'],
      enabled: partial.enabled !== false
    };
    return c;
  }

  /* Structural checks independent of any pose. */
  function validateConstraints(rig) {
    const byId = new Map(rig.nodes.map((n) => [n.id, n]));
    const errors = [];
    const list = rig.constraints || [];
    const seen = new Set();
    for (const c of list) {
      if (seen.has(c.id)) errors.push(`约束「${c.name}」(${c.id}) id 重复`);
      seen.add(c.id);
      if (!CONSTRAINT_TYPES.includes(c.type)) {
        errors.push(`约束「${c.name}」类型未知: ${c.type}`);
        continue;
      }
      if (!byId.has(c.nodeA) || !byId.has(c.nodeB)) {
        errors.push(`约束「${c.name}」引用了不存在的节点，已跳过`);
        continue;
      }
      if (c.nodeA === c.nodeB) {
        errors.push(`约束「${c.name}」的两个端点不能是同一节点`);
      }
      if (!c.axes.length) errors.push(`约束「${c.name}」至少需要一个参与轴`);
      if (c.type === 'distance' && (!(c.distance >= 0) || !isFinite(c.distance))) {
        errors.push(`约束「${c.name}」的距离必须是非负有限值`);
      }
    }
    return errors;
  }

  /* ---------- curves ---------- */
  function evalCurve(curve, t) {
    switch (curve.mode) {
      case 'hold':
        return curve.base;
      case 'sine':
        return curve.base + curve.amplitude * Math.sin(
          (2 * Math.PI * t) / Math.max(curve.period, 1e-6) + (curve.phase || 0)
        );
      case 'linear':
        return curve.base + (curve.speed || 0) * t;
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

  /* =====================================================================
   * Closed-loop samples. The hierarchy stays a tree; the loop is expressed
   * only through rig.constraints. Joints marked driven:false are unknowns.
   * All planar samples live in the XY plane with revolute axes +Z.
   * ===================================================================== */

  /* Planar crank-rocker four-bar (Grashof): ground O=A0, crank a=0.35,
   * coupler b=1.0, rocker c=0.9, ground pivots d=1.1 apart.
   * Tree: O -> crank -> coupler -> P ; ground pivot B is a second root node
   * carrying the rocker -> Q. One coincident constraint closes P == Q. */
  function buildFourBar() {
    const rig = createRig('四杆机构 · 曲柄摇杆闭环');
    const O = createNode({
      name: '机架铰点 O', parentId: null,
      joint: { type: 'fixed' },
      part: { shape: 'cylinder', size: [0.07, 0.05, 0.07], color: '#8a93a6' }
    });
    const crank = createNode({
      name: '曲柄 A (驱动, a=0.35)', parentId: O.id,
      joint: { type: 'continuous', axis: [0, 0, 1], unit: 'deg', limitType: 'continuous', min: 0, max: 0 },
      curve: { mode: 'linear', base: 0, speed: 45 },
      part: { shape: 'cylinder', size: [0.025, 0.35, 0.025], color: '#ffb347', partOffset: [0.175, 0, 0] }
    });
    const coupler = createNode({
      name: '连杆 B (自由铰, b=1.0)', parentId: crank.id,
      mount: { offset: [0.35, 0, 0] },
      joint: { type: 'revolute', axis: [0, 0, 1], unit: 'deg', limitType: 'continuous', min: 0, max: 0 },
      driven: false,
      curve: { mode: 'hold', base: -30 },
      part: { shape: 'cylinder', size: [0.022, 1, 0.022], color: '#6ea8ff', partOffset: [0.5, 0, 0] }
    });
    const P = createNode({
      name: '耦合点 P (连杆端)', parentId: coupler.id,
      mount: { offset: [1, 0, 0] },
      joint: { type: 'fixed' },
      part: { shape: 'sphere', size: [0.06, 0.06, 0.06], color: '#7dffc8' },
      showAxes: true
    });
    const B = createNode({
      name: '机架铰点 B (d=1.1)', parentId: null,
      mount: { offset: [1.1, 0, 0] },
      joint: { type: 'fixed' },
      part: { shape: 'cylinder', size: [0.07, 0.05, 0.07], color: '#8a93a6' }
    });
    const rocker = createNode({
      name: '摇杆 C (自由铰, c=0.9)', parentId: B.id,
      joint: { type: 'revolute', axis: [0, 0, 1], unit: 'deg', limitType: 'continuous', min: 0, max: 0 },
      driven: false,
      curve: { mode: 'hold', base: 75 },
      part: { shape: 'cylinder', size: [0.025, 0.9, 0.025], color: '#9d8cff', partOffset: [0.45, 0, 0] }
    });
    const Q = createNode({
      name: '耦合点 Q (摇杆端)', parentId: rocker.id,
      mount: { offset: [0.9, 0, 0] },
      joint: { type: 'fixed' },
      part: { shape: 'sphere', size: [0.055, 0.055, 0.055], color: '#ff6b8a' }
    });
    rig.nodes.push(O, crank, coupler, P, B, rocker, Q);
    rig.tipNodeId = P.id;
    rig.constraints.push(createConstraint({
      name: 'P=Q 销轴闭合 (XY)',
      type: 'coincident', nodeA: P.id, nodeB: Q.id, axes: ['x', 'y']
    }));
    return rig;
  }

  /* Two independent planar arms whose tips share one end effector.
   * Each arm has 2 revolute joints; only arm-1 joints are driven, arm-2 is
   * fully solved. Two assembly branches (elbow up / elbow down) exist.
   * Base separation 1.2 m stays inside arm-2's reach [0.1, 1.1] margin across
   * the whole driven sweep, so the closure never leaves its envelope. */
  function buildDualArm() {
    const rig = createRig('双支链共末端 · 多解分支');
    const O1 = createNode({
      name: '支链1 基座 O1', parentId: null,
      joint: { type: 'fixed' },
      part: { shape: 'cylinder', size: [0.06, 0.04, 0.06], color: '#8a93a6' }
    });
    const a1 = createNode({
      name: '支链1 肩 (驱动)', parentId: O1.id,
      joint: { type: 'revolute', axis: [0, 0, 1], unit: 'deg', min: -360, max: 360 },
      curve: { mode: 'sine', base: 55, amplitude: 15, period: 8 },
      part: { shape: 'cylinder', size: [0.024, 0.7, 0.024], color: '#ffb347', partOffset: [0.35, 0, 0] }
    });
    const a2 = createNode({
      name: '支链1 肘 (驱动)', parentId: a1.id,
      mount: { offset: [0.7, 0, 0] },
      joint: { type: 'revolute', axis: [0, 0, 1], unit: 'deg', min: -360, max: 360 },
      curve: { mode: 'sine', base: -70, amplitude: 10, period: 8, phase: 1.2 },
      part: { shape: 'cylinder', size: [0.02, 0.7, 0.02], color: '#ffd98a', partOffset: [0.35, 0, 0] }
    });
    const E1 = createNode({
      name: '共同末端 E', parentId: a2.id,
      mount: { offset: [0.7, 0, 0] },
      joint: { type: 'fixed' },
      part: { shape: 'sphere', size: [0.07, 0.07, 0.07], color: '#7dffc8' },
      showAxes: true
    });
    const O2 = createNode({
      name: '支链2 基座 O2', parentId: null,
      mount: { offset: [1.2, 0, 0] },
      joint: { type: 'fixed' },
      part: { shape: 'cylinder', size: [0.06, 0.04, 0.06], color: '#8a93a6' }
    });
    const b1 = createNode({
      name: '支链2 肩 (自由)', parentId: O2.id,
      joint: { type: 'revolute', axis: [0, 0, 1], unit: 'deg', limitType: 'continuous', min: 0, max: 0 },
      driven: false,
      curve: { mode: 'hold', base: 140 },
      part: { shape: 'cylinder', size: [0.024, 0.6, 0.024], color: '#9d8cff', partOffset: [0.3, 0, 0] }
    });
    const b2 = createNode({
      name: '支链2 肘 (自由)', parentId: b1.id,
      mount: { offset: [0.6, 0, 0] },
      joint: { type: 'revolute', axis: [0, 0, 1], unit: 'deg', limitType: 'continuous', min: 0, max: 0 },
      driven: false,
      curve: { mode: 'hold', base: -70 },
      part: { shape: 'cylinder', size: [0.02, 0.5, 0.02], color: '#c0b3ff', partOffset: [0.25, 0, 0] }
    });
    const E2 = createNode({
      name: '支链2 末端 (闭合到 E)', parentId: b2.id,
      mount: { offset: [0.5, 0, 0] },
      joint: { type: 'fixed' },
      part: { shape: 'sphere', size: [0.05, 0.05, 0.05], color: '#ff6b8a' }
    });
    rig.nodes.push(O1, a1, a2, E1, O2, b1, b2, E2);
    rig.tipNodeId = E1.id;
    rig.constraints.push(createConstraint({
      name: '两支链末端重合 (XY)',
      type: 'coincident', nodeA: E1.id, nodeB: E2.id, axes: ['x', 'y']
    }));
    return rig;
  }

  /* Same four-bar geometry but with a second, conflicting distance constraint
   * added so the constraint set is over-determined and generally inconsistent:
   * the solver must report the residual instead of hiding it by moving inputs. */
  function buildOverConstrained() {
    const rig = buildFourBar();
    rig.name = '过约束冲突演示 · 残差不可消除';
    const crank = rig.nodes.find((n) => n.name.startsWith('曲柄'));
    const rocker = rig.nodes.find((n) => n.name.startsWith('摇杆'));
    rig.constraints.push(createConstraint({
      name: '额外距离约束 (与销轴冲突)',
      type: 'distance',
      nodeA: crank.id, nodeB: rocker.id,
      offsetA: [0.175, 0, 0], offsetB: [0.45, 0, 0],
      distance: 0.3,
      axes: ['x', 'y']
    }));
    return rig;
  }

  /* Rocker-driven four-bar. The actuator is placed on the rocker and sweeps a
   * range wider than the geometric swing limits, so near the dead points the
   * assembly ceases to exist and the solver reports unreachable / singular. */
  function buildDeadPoint() {
    const rig = buildFourBar();
    rig.name = '死点 / 可达范围演示 · 摇杆驱动';
    const crank = rig.nodes.find((n) => n.name.startsWith('曲柄'));
    const rocker = rig.nodes.find((n) => n.name.startsWith('摇杆'));
    crank.driven = false;
    crank.joint.limitType = 'range';
    crank.joint.min = -360;
    crank.joint.max = 360;
    crank.curve = { mode: 'hold', base: 0 };
    rocker.driven = true;
    rocker.joint.limitType = 'range';
    // geometric swing for this geometry is about 95.7°..143.8°; the sine
    // sweeps through it and overshoots both dead points
    rocker.joint.min = -360;
    rocker.joint.max = 360;
    rocker.curve = { mode: 'sine', base: 120, amplitude: 30, period: 10, phase: 0 };
    return rig;
  }

  MR.Rig = {
    nid, createNode, createRig, evalCurve, jointValueToBase,
    validateHierarchy, canReparent, safeDelete,
    createConstraint, validateConstraints, cid,
    CONSTRAINT_TYPES, CONSTRAINT_AXES,
    buildMechCharacter, buildTwoLink, buildSlide, buildLimitDemo,
    buildFourBar, buildDualArm, buildOverConstrained, buildDeadPoint,
    ANGLE_UNITS, LENGTH_UNITS, ANGLE_TO_RAD, LENGTH_TO_M
  };
})(typeof window !== 'undefined' ? window : globalThis);
