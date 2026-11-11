/* Node-only integration smoke test with a minimal DOM stub.
 * Exercises every browser script including app.js wiring. */
'use strict';
const fs = require('fs');
const path = require('path');
const vm = require('vm');

function makeEl(tag) {
  const el = {
    tag, children: [], style: {}, dataset: {}, classList: {
      _set: new Set(),
      add(c) { this._set.add(c); },
      remove(c) { this._set.delete(c); },
      contains(c) { return this._set.has(c); },
      get toString() { return () => [...this._set].join(' '); }
    },
    _attrs: {},
    _listeners: {},
    width: 800, height: 600,
    parentElement: { clientWidth: 800, clientHeight: 600 },
    appendChild(k) { this.children.push(k); return k; },
    addEventListener(ev, fn) { (this._listeners[ev] ||= []).push(fn); },
    setAttribute(k, v) { this._attrs[k] = v; },
    getContext() {
      return new Proxy({}, { get: (t, p) => {
        if (p === 'canvas') return el;
        return typeof t[p] !== 'undefined' ? t[p] : (...a) => {
          if (p === 'measureText') return { width: 10 };
        };
      }, set: (t, p, v) => { t[p] = v; return true; } });
    },
    click() { (this._listeners.click || []).forEach((f) => f({})); },
    querySelector(sel) {
      return makeEl(sel);
    },
    get innerHTML() { return this._html || ''; },
    set innerHTML(v) { this._html = v; this.children = []; },
    get textContent() { return this._text || ''; },
    set textContent(v) { this._text = String(v); },
    set className(v) { v.split(' ').forEach((c) => c && this.classList.add(c)); }
  };
  return el;
}

const ids = {};
const documentStub = {
  createElement: (t) => makeEl(t),
  createTextNode: (s) => ({ text: s }),
  querySelector(sel) {
    if (!ids[sel]) ids[sel] = makeEl(sel);
    return ids[sel];
  },
  getElementById(id) {
    const sel = '#' + id;
    if (!ids[sel]) ids[sel] = makeEl(id);
    return ids[sel];
  },
  querySelectorAll() { return []; }
};

const rafQueue = [];
const winListeners = {};
const sandbox = {
  console,
  document: documentStub,
  window: {},
  addEventListener(ev, fn) { (winListeners[ev] ||= []).push(fn); },
  localStorage: (() => {
    const store = new Map();
    return {
      setItem: (k, v) => store.set(k, v),
      getItem: (k) => (store.has(k) ? store.get(k) : null),
      removeItem: (k) => store.delete(k)
    };
  })(),
  requestAnimationFrame: (fn) => { rafQueue.push(fn); return rafQueue.length; },
  performance: { now: () => Date.now() },
  setTimeout, clearTimeout,
  Blob: function () {},
  URL: { createObjectURL: () => 'blob:x', revokeObjectURL: () => {} },
  devicePixelRatio: 1,
  Date, Math, JSON, parseFloat, parseInt, isNaN
};
sandbox.window = sandbox;
sandbox.globalThis = sandbox;
vm.createContext(sandbox);

for (const f of ['math3d.js', 'rig.js', 'kinematics.js', 'renderer.js', 'state.js', 'inspector.js', 'tests.js', 'app.js']) {
  const code = fs.readFileSync(path.join(__dirname, '..', 'js', f), 'utf8');
  vm.runInContext(code, sandbox, { filename: f });
}

// emulate DOMContentLoaded
const app = sandbox.window.MRApp;
let failures = 0;
const check = (name, cond, detail) => {
  console.log(`${cond ? 'PASS' : 'FAIL'}  ${name}${detail ? ' — ' + detail : ''}`);
  if (!cond) failures++;
};

app.init();
// first frame is scheduled via rAF, as in a real browser
rafQueue.shift()(16);
check('首帧生成姿态', !!app.currentPose);
check('测试面板渲染了 8 项', (ids['#tests-panel']?.children.length || 0) === 8,
  'children=' + ids['#tests-panel']?.children.length);
check('层级树渲染了角色节点', (ids['#tree']?.children.length || 0) === 10);
check('矩阵面板生成矩阵块', (ids['#matrix-panel']?.children.length || 0) >= 3);
check('末端位置写入状态栏', /\[-?\d/.test(ids['#status-tip']?.textContent || ''), ids['#status-tip']?.textContent);

// drive the animation clock 60 frames
const firstTip = [...app.currentPose.tipPosition];
for (let i = 0; i < 60; i++) {
  const fn = rafQueue.shift();
  fn(16 * (i + 1));
}
const moved = sandbox.MR.V3.len(sandbox.MR.V3.sub(app.currentPose.tipPosition, firstTip));
check('动画时钟驱动末端运动', moved > 1e-4, '位移=' + moved.toFixed(4));
check('轨迹已记录', app.renderer.trail.length > 10, '点数=' + app.renderer.trail.length);

// switching rigs
app.loadRig(sandbox.MR.Rig.buildTwoLink());
rafQueue.shift()(100);
check('切换两节臂预设', app.currentPose.frames.size === 5);
const twolinkTip = app.currentPose.tipPosition;
const expect = sandbox.MR.Kinematics.twoLinkAnalytic([0,0,0], 1, 0.8, Math.PI/6, Math.PI/3);
check('预设末端与解析公式一致', sandbox.MR.Tests.vapprox(twolinkTip, expect, 1e-9));

// persistence round trip through the app's localStorage
sandbox.MR.State.Storage.save();
const raw = sandbox.localStorage.getItem(sandbox.MR.State.STORAGE_KEY);
check('自动保存写入 localStorage', !!raw && JSON.parse(raw).format === 'mr-rig');
const before = sandbox.MR.Kinematics.evaluate(sandbox.MR.State.state.rig, 2.34).tipPosition;
sandbox.MR.State.Storage.load();
const after = sandbox.MR.Kinematics.evaluate(sandbox.MR.State.state.rig, 2.34).tipPosition;
check('重新加载后姿态一致', sandbox.MR.Tests.vapprox(before, after, 1e-12));

// limit demo clamps visible (drive to a time where sine overshoots)
app.loadRig(sandbox.MR.Rig.buildLimitDemo());
sandbox.MR.State.state.time = 1.5; // sine near its extremes for the ±45 joint
rafQueue.shift()(200);
const clampedFrames = [...app.currentPose.frames.values()].filter((f) => f.clamped);
check('极限演示产生越界钳制帧', clampedFrames.length >= 1, '数量=' + clampedFrames.length);
check('状态栏显示限制告警', ids['#status-limit']?.classList.contains('limit-warn'));

// hierarchy guard through the app delete path
sandbox.MR.State.state.selectedId = sandbox.MR.State.state.rig.nodes[0].id;
app.deleteSelected();
check('带子节点删除被拒绝', sandbox.MR.State.state.rig.nodes.length === 4);

// add node through the toolbar path
const beforeCount = sandbox.MR.State.state.rig.nodes.length;
app.addNode();
check('添加节点进入层级', sandbox.MR.State.state.rig.nodes.length === beforeCount + 1);
check('检查器已渲染选中节点', (ids['#inspector-host']?.children[0]?.children.length || 0) > 0);

console.log(`\n${failures === 0 ? 'ALL SMOKE TESTS PASSED' : failures + ' SMOKE FAILURE(S)'}`);
process.exitCode = failures ? 1 : 0;
