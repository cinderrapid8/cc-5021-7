/* app.js — application wiring: tree, timeline, matrix panel, tests, camera. */
(function (global) {
  'use strict';

  const MR = global.MR;
  const { state, Storage } = MR.State;
  const { M4, V3 } = MR;

  const $ = (sel) => document.querySelector(sel);

  const app = {
    camera: new MR.OrbitCamera(),
    renderer: null,
    inspector: null,
    currentPose: null,
    saveTimer: null,

    init() {
      Storage.load();
      this.renderer = new MR.Renderer($('#view'));
      this.inspector = MR.Inspector.buildInspector(this);
      $('#inspector-host').appendChild(this.inspector.element);
      this.bindCanvas();
      this.bindToolbar();
      this.bindTimeline();
      this.bindRootEditor();
      this.applyViewFlags();
      if (!state.selectedId && state.rig.nodes.length) state.selectedId = state.rig.nodes[0].id;
      this.refreshAll();
      this.runTests();
      requestAnimationFrame((ts) => this.tick(ts));
      window.addEventListener('resize', () => this.resize());
      this.resize();
    },

    /* ---------- persistence ---------- */
    scheduleSave() {
      if (!state.autoSave) return;
      clearTimeout(this.saveTimer);
      this.saveTimer = setTimeout(() => Storage.save(), 400);
    },

    flash(msg, isError = false) {
      const bar = $('#flash');
      bar.textContent = msg;
      bar.className = isError ? 'flash error show' : 'flash show';
      clearTimeout(this._flashT);
      this._flashT = setTimeout(() => bar.classList.remove('show'), 2600);
    },

    /* ---------- canvas / camera ---------- */
    resize() {
      const c = $('#view');
      const host = c.parentElement;
      const dpr = window.devicePixelRatio || 1;
      c.width = host.clientWidth * dpr;
      c.height = host.clientHeight * dpr;
      c.style.width = host.clientWidth + 'px';
      c.style.height = host.clientHeight + 'px';
    },

    bindCanvas() {
      const c = $('#view');
      let drag = null;
      c.addEventListener('pointerdown', (e) => {
        drag = { x: e.clientX, y: e.clientY, btn: e.button };
        c.setPointerCapture(e.pointerId);
      });
      c.addEventListener('pointermove', (e) => {
        if (!drag) return;
        const dx = e.clientX - drag.x;
        const dy = e.clientY - drag.y;
        if (drag.btn === 2 || e.shiftKey) {
          this.camera.distance = Math.max(1, Math.min(15, this.camera.distance * (1 + dy * 0.005)));
        } else {
          this.camera.yaw += dx * 0.008;
          this.camera.pitch = Math.max(-1.4, Math.min(1.4, this.camera.pitch + dy * 0.008));
        }
        drag.x = e.clientX;
        drag.y = e.clientY;
      });
      c.addEventListener('pointerup', () => { drag = null; });
      c.addEventListener('contextmenu', (e) => e.preventDefault());
      c.addEventListener('wheel', (e) => {
        e.preventDefault();
        this.camera.distance = Math.max(1, Math.min(15, this.camera.distance * (1 + e.deltaY * 0.001)));
      }, { passive: false });
    },

    /* ---------- toolbar ---------- */
    bindToolbar() {
      $('#btn-add').addEventListener('click', () => this.addNode());
      $('#btn-save').addEventListener('click', () => {
        Storage.save();
        this.flash('已保存到浏览器本地存储');
      });
      $('#btn-export').addEventListener('click', () => Storage.exportFile());
      $('#btn-import').addEventListener('click', () => $('#file-import').click());
      $('#file-import').addEventListener('change', async (e) => {
        const file = e.target.files[0];
        if (!file) return;
        try {
          await Storage.importFile(file);
          this.renderer.clearTrail();
          this.refreshAll();
          this.flash('导入成功');
        } catch (err) {
          this.flash(err.message, true);
        }
        e.target.value = '';
      });
      const presets = [
        ['#preset-char', () => MR.Rig.buildMechCharacter()],
        ['#preset-twolink', () => MR.Rig.buildTwoLink()],
        ['#preset-slide', () => MR.Rig.buildSlide()],
        ['#preset-limit', () => MR.Rig.buildLimitDemo()]
      ];
      for (const [sel, builder] of presets) {
        $(sel).addEventListener('click', () => this.loadRig(builder()));
      }
      $('#btn-clear-trail').addEventListener('click', () => this.renderer.clearTrail());
      const flags = [
        ['#chk-grid', 'showGrid'],
        ['#chk-axes', 'showAllAxes'],
        ['#chk-labels', 'showLabels'],
        ['#chk-trail', 'showTrail']
      ];
      for (const [sel, key] of flags) {
        $(sel).checked = state[key];
        $(sel).addEventListener('change', () => {
          state[key] = $(sel).checked;
          this.applyViewFlags();
          this.scheduleSave();
        });
      }
    },

    applyViewFlags() {
      this.renderer.showGrid = state.showGrid;
      this.renderer.showAllAxes = state.showAllAxes;
      this.renderer.showLabels = state.showLabels;
      this.renderer.showTrail = state.showTrail;
    },

    loadRig(rig) {
      state.rig = rig;
      state.selectedId = rig.nodes[0]?.id ?? null;
      state.time = 0;
      this.renderer.clearTrail();
      this.refreshAll();
      this.scheduleSave();
    },

    addNode() {
      const node = MR.Rig.createNode({
        name: '新节点',
        parentId: state.selectedId,
        mount: { offset: [0, 0.15, 0] },
        joint: { type: 'revolute', axis: [0, 0, 1], min: -90, max: 90 },
        curve: { mode: 'sine', base: 0, amplitude: 30, period: 4 },
        part: { shape: 'box', size: [0.08, 0.08, 0.08], color: '#6ea8ff' }
      });
      state.rig.nodes.push(node);
      state.selectedId = node.id;
      this.refreshAll();
      this.scheduleSave();
    },

    deleteSelected() {
      const id = state.selectedId;
      if (!id) return;
      const check = MR.Rig.safeDelete(state.rig.nodes, id);
      if (!check.ok) {
        this.flash(check.reason, true);
        return;
      }
      state.rig.nodes = state.rig.nodes.filter((n) => n.id !== id);
      state.selectedId = state.rig.nodes[0]?.id ?? null;
      this.refreshAll();
      this.scheduleSave();
    },

    /* ---------- tree ---------- */
    renderTree() {
      const host = $('#tree');
      host.innerHTML = '';
      const byId = new Map(state.rig.nodes.map((n) => [n.id, n]));
      const childrenOf = (pid) => state.rig.nodes.filter((n) => n.parentId === pid);
      const dangling = new Set(
        state.rig.nodes.filter((n) => n.parentId && !byId.has(n.parentId)).map((n) => n.id)
      );

      const renderNode = (node, depth) => {
        const row = document.createElement('div');
        row.className = 'tree-row' + (node.id === state.selectedId ? ' selected' : '');
        row.style.paddingLeft = 8 + depth * 14 + 'px';
        const badge = node.joint.type === 'prismatic' ? '滑'
          : node.joint.type === 'continuous' ? '连'
          : node.joint.type === 'revolute' ? '转' : '固';
        const badgeClass = { 滑: 'b-slide', 连: 'b-cont', 转: 'b-rev', 固: 'b-fix' }[badge];
        row.innerHTML = `<span class="badge ${badgeClass}">${badge}</span><span class="tree-name"></span>`;
        row.querySelector('.tree-name').textContent = node.name;
        if (dangling.has(node.id)) {
          const warn = document.createElement('span');
          warn.className = 'tree-warn';
          warn.textContent = '失效父引用';
          row.appendChild(warn);
        }
        row.addEventListener('click', () => {
          state.selectedId = node.id;
          this.refreshAll();
        });
        host.appendChild(row);
        childrenOf(node.id).forEach((ch) => renderNode(ch, depth + 1));
      };
      state.rig.nodes.filter((n) => n.parentId === null || dangling.has(n.id))
        .forEach((n) => renderNode(n, 0));
    },

    /* ---------- root transform ---------- */
    bindRootEditor() {
      const rerender = () => {
        const rt = state.rig.rootTransform;
        const host = $('#root-editor');
        host.innerHTML = '';
        const mk = (label, vec, step) => {
          const wrap = document.createElement('div');
          wrap.className = 'root-row';
          wrap.appendChild(Object.assign(document.createElement('span'), { textContent: label, className: 'field-label' }));
          ['x', 'y', 'z'].forEach((axis, i) => {
            const inp = document.createElement('input');
            inp.type = 'number';
            inp.className = 'num';
            inp.step = String(step);
            inp.value = String(vec[i]);
            inp.title = axis;
            inp.addEventListener('change', () => {
              vec[i] = parseFloat(inp.value) || 0;
              this.scheduleSave();
            });
            wrap.appendChild(inp);
          });
          host.appendChild(wrap);
        };
        mk('根平移 (米)', rt.position, 0.1);
        mk('根欧拉角 (度)', rt.eulerDeg, 5);
      };
      this._rerenderRoot = rerender;
      rerender();
    },

    /* ---------- timeline ---------- */
    bindTimeline() {
      $('#btn-play').addEventListener('click', () => {
        state.playing = !state.playing;
        $('#btn-play').textContent = state.playing ? '暂停' : '播放';
      });
      $('#btn-reset').addEventListener('click', () => {
        state.time = 0;
        this.renderer.clearTrail();
      });
      const slider = $('#time-slider');
      slider.addEventListener('input', (e) => {
        state.time = parseFloat(e.target.value);
      });
      slider.addEventListener('pointerdown', () => {
        this._wasPlaying = state.playing;
        state.playing = false;
      });
      slider.addEventListener('pointerup', () => {
        state.playing = this._wasPlaying !== false;
        $('#btn-play').textContent = state.playing ? '暂停' : '播放';
      });
      $('#duration').addEventListener('change', (e) => {
        state.duration = Math.max(0.5, parseFloat(e.target.value) || 8);
      });
      $('#speed').addEventListener('change', (e) => {
        state.speed = Math.max(0.1, parseFloat(e.target.value) || 1);
      });
      $('#loop').addEventListener('change', (e) => { state.loop = e.target.checked; });
    },

    /* ---------- matrix panel ---------- */
    renderMatrix(pose) {
      const host = $('#matrix-panel');
      const node = state.rig.nodes.find((n) => n.id === state.selectedId);
      if (!node) {
        host.innerHTML = '<div class="empty-hint">选择节点后显示变换矩阵</div>';
        return;
      }
      const f = pose.frames.get(node.id);
      const table = (title, m) => {
        const wrap = document.createElement('div');
        wrap.className = 'matrix-block';
        wrap.appendChild(Object.assign(document.createElement('div'), { textContent: title, className: 'matrix-title' }));
        const grid = document.createElement('div');
        grid.className = 'matrix-grid';
        for (const row of M4.toRows(m)) {
          for (const v of row) {
            const cell = document.createElement('div');
            cell.className = 'matrix-cell';
            cell.textContent = Math.abs(v) < 1e-10 ? '0' : v.toFixed(3);
            grid.appendChild(cell);
          }
        }
        wrap.appendChild(grid);
        return wrap;
      };
      host.innerHTML = '';
      host.appendChild(table('M_mount 安装矩阵', f.mount));
      host.appendChild(table('M_joint 关节运动矩阵', f.joint));
      host.appendChild(table('M_world 世界矩阵 = 父世界 × Mount × Joint', f.world));
      const info = document.createElement('div');
      info.className = 'matrix-meta';
      const axisTxt = (a) => `[${a.map((x) => x.toFixed(3)).join(', ')}]`;
      info.innerHTML =
        `关节轴（局部）${axisTxt(f.axisLocal)}<br>` +
        `关节轴（世界）${axisTxt(f.axisWorld)}<br>` +
        `曲线原始值 ${fmt(f.raw)} ${node.joint.unit} → 应用值 ${fmt(f.applied)} ${node.joint.unit}` +
        (f.clamped ? ` <span class="clamp-tag">${f.status === 'clamped-low' ? '已触下限' : '已触上限'}</span>` : '');
      host.appendChild(info);
    },

    /* ---------- tests panel ---------- */
    runTests() {
      const host = $('#tests-panel');
      host.innerHTML = '';
      const results = MR.Tests.runAll();
      for (const r of results) {
        const item = document.createElement('details');
        item.className = 'test-item ' + (r.pass ? 'pass' : 'fail');
        const summary = document.createElement('summary');
        summary.textContent = (r.pass ? '✓ ' : '✗ ') + r.name;
        item.appendChild(summary);
        if (r.detail) {
          const pre = document.createElement('pre');
          pre.className = 'test-detail';
          pre.textContent = r.detail;
          item.appendChild(pre);
        }
        host.appendChild(item);
      }
      const failed = results.filter((r) => !r.pass).length;
      $('#tests-summary').textContent = failed
        ? `${results.length - failed}/${results.length} 项通过，${failed} 项失败`
        : `全部 ${results.length} 项解析核对通过`;
      $('#tests-summary').className = failed ? 'summary fail' : 'summary pass';
    },

    /* ---------- status bar ---------- */
    renderStatus(pose) {
      const tip = pose.tipPosition;
      const tipTxt = tip ? `[${tip.map((x) => x.toFixed(3)).join(', ')}] m` : '未指定';
      const clamped = [...pose.frames.values()].filter((f) => f.clamped);
      $('#status-tip').textContent = tipTxt;
      $('#status-limit').textContent = clamped.length
        ? clamped.map((f) => `${f.node.name}:${f.status === 'clamped-low' ? '下限' : '上限'}`).join('；')
        : '所有关节在限制范围内';
      $('#status-limit').className = clamped.length ? 'limit-warn' : 'limit-ok';
      if (pose.errors.length) {
        $('#status-errors').textContent = pose.errors.join(' | ');
        $('#status-errors').classList.add('show');
      } else {
        $('#status-errors').classList.remove('show');
      }
    },

    /* ---------- frame loop ---------- */
    tick(ts) {
      if (state.lastWall == null) state.lastWall = ts;
      const dt = Math.min(0.05, (ts - state.lastWall) / 1000);
      state.lastWall = ts;
      if (state.playing) {
        state.time += dt * state.speed;
        if (state.loop && state.time > state.duration) {
          state.time = state.time % state.duration;
          this.renderer.clearTrail();
        }
        state.time = Math.min(state.time, state.duration);
      }
      const pose = MR.Kinematics.evaluate(state.rig, state.time);
      this.currentPose = pose;
      if (state.showTrail && pose.tipPosition) this.renderer.addTrailPoint(pose.tipPosition);
      this.renderer.render(pose, this.camera, state.rig, state.selectedId);
      this.renderMatrix(pose);
      this.renderStatus(pose);
      $('#time-label').textContent = `t = ${state.time.toFixed(2)} s`;
      const slider = $('#time-slider');
      slider.max = String(state.duration);
      slider.value = String(state.time);
      requestAnimationFrame((t) => this.tick(t));
    },

    refreshAll() {
      this.renderTree();
      this.inspector.rerender();
      this._rerenderRoot?.();
      this.scheduleSave();
    }
  };

  function fmt(v) {
    return (Math.round(v * 1000) / 1000).toString();
  }

  global.MRApp = app;
  window.addEventListener('DOMContentLoaded', () => app.init());
})(typeof window !== 'undefined' ? window : globalThis);
