/* inspector.js — node property editor.
 * Mount transform and joint motion are edited in separate sections so the
 * separation is visible; joint limits never edit part dimensions. */
(function (global) {
  'use strict';

  const MR = global.MR;

  const el = (tag, attrs = {}, ...kids) => {
    const node = document.createElement(tag);
    for (const [k, v] of Object.entries(attrs)) {
      if (k === 'class') node.className = v;
      else if (k === 'text') node.textContent = v;
      else if (k.startsWith('on') && typeof v === 'function') node.addEventListener(k.slice(2), v);
      else if (v !== null && v !== undefined) node.setAttribute(k, v);
    }
    for (const kid of kids.flat()) {
      if (kid == null) continue;
      node.appendChild(typeof kid === 'string' ? document.createTextNode(kid) : kid);
    }
    return node;
  };

  function numInput(value, step, oninput, extra = {}) {
    const input = el('input', {
      type: 'number', step: String(step), value: String(value),
      class: 'num', ...extra
    });
    input.addEventListener('change', () => oninput(parseFloat(input.value) || 0));
    return input;
  }

  function vec3Inputs(vec, step, onchange) {
    return ['x', 'y', 'z'].map((axis, i) =>
      el('label', { class: 'vec-cell' },
        axis.toUpperCase(),
        numInput(vec[i], step, (v) => {
          vec[i] = v;
          onchange();
        }))
    );
  }

  function section(title, ...kids) {
    return el('div', { class: 'section' },
      el('div', { class: 'section-title', text: title }),
      ...kids
    );
  }

  function selectInput(value, options, onchange) {
    const s = el('select', { class: 'full' });
    for (const o of options) {
      const opt = el('option', { value: o, text: o });
      if (o === value) opt.selected = true;
      s.appendChild(opt);
    }
    s.addEventListener('change', () => onchange(s.value));
    return s;
  }

  function field(label, ...kids) {
    return el('div', { class: 'field' },
      el('span', { class: 'field-label', text: label }),
      ...kids
    );
  }

  function buildInspector(app) {
    const { state } = MR.State;
    const root = el('div', { id: 'inspector' });

    function rerender() {
      root.innerHTML = '';
      const node = state.rig.nodes.find((n) => n.id === state.selectedId);
      if (!node) {
        root.appendChild(el('div', { class: 'empty-hint', text: '在左侧层级中选择一个节点查看属性' }));
        return;
      }
      const refresh = () => { rerender(); app.refreshAll(); };

      /* ---- identity ---- */
      const nameInput = el('input', { class: 'full', value: node.name });
      nameInput.addEventListener('change', () => { node.name = nameInput.value; app.refreshAll(); });
      const parentOptions = [el('option', { value: '', text: '— 根节点 —' })];
      for (const n of state.rig.nodes) {
        if (n.id === node.id) continue;
        const opt = el('option', { value: n.id, text: n.name });
        if (n.id === node.parentId) opt.selected = true;
        parentOptions.push(opt);
      }
      const parentSel = el('select', { class: 'full' }, ...parentOptions);
      parentSel.addEventListener('change', () => {
        const target = parentSel.value || null;
        const check = MR.Rig.canReparent(state.rig.nodes, node.id, target);
        if (!check.ok) {
          app.flash(check.reason, true);
          rerender();
          return;
        }
        node.parentId = target;
        app.refreshAll();
      });
      const enabledChk = el('input', { type: 'checkbox' });
      enabledChk.checked = node.enabled;
      enabledChk.addEventListener('change', () => { node.enabled = enabledChk.checked; app.refreshAll(); });

      root.appendChild(section('节点',
        field('名称', nameInput),
        field('父节点', parentSel),
        el('label', { class: 'check-line' }, enabledChk, '启用该节点')
      ));

      /* ---- mount transform (geometry placement only) ---- */
      root.appendChild(section('安装偏移 Mount（几何定位，与关节运动分离）',
        el('div', { class: 'vec-row' }, ...vec3Inputs(node.mount.offset, 0.01, refresh)),
        field('安装欧拉角 (度)',
          el('div', { class: 'vec-row' }, ...vec3Inputs(node.mount.eulerDeg, 5, refresh)))
      ));

      /* ---- joint ---- */
      const j = node.joint;
      const typeSel = selectInput(j.type, ['fixed', 'revolute', 'prismatic', 'continuous'], (v) => {
        j.type = v;
        if (v === 'prismatic') { j.unit = 'cm'; j.min = 0; j.max = 20; }
        else if (v === 'fixed') { j.unit = 'deg'; j.min = 0; j.max = 0; }
        else { j.unit = 'deg'; j.limitType = v === 'continuous' ? 'continuous' : 'range'; j.min = -90; j.max = 90; }
        refresh();
      });
      const unitOptions = j.type === 'prismatic' ? MR.Rig.LENGTH_UNITS : MR.Rig.ANGLE_UNITS;
      const unitSel = selectInput(j.unit, unitOptions, (v) => { j.unit = v; refresh(); });
      const limitSel = selectInput(j.limitType, ['range', 'continuous'], (v) => {
        j.limitType = v;
        refresh();
      });
      const jointKids = [
        field('类型', typeSel),
        field('单位', unitSel),
        field('关节轴 (局部坐标)', el('div', { class: 'vec-row' }, ...vec3Inputs(j.axis, 0.1, refresh)))
      ];
      if (j.type !== 'fixed') {
        const drivenChk = el('input', { type: 'checkbox' });
        drivenChk.checked = node.driven !== false;
        drivenChk.addEventListener('change', () => {
          node.driven = drivenChk.checked;
          MR.State.state.solverSeed = null;
          refresh();
        });
        jointKids.push(el('label', { class: 'check-line' }, drivenChk,
          '驱动关节（取时间曲线值；取消勾选则成为闭环求解的自由未知量）'));
        jointKids.push(field('限制类型', limitSel));
        if (j.limitType === 'range' && j.type !== 'continuous') {
          jointKids.push(field('下限 ' + j.unit, numInput(j.min, 1, (v) => { j.min = v; refresh(); })));
          jointKids.push(field('上限 ' + j.unit, numInput(j.max, 1, (v) => { j.max = v; refresh(); })));
        } else {
          jointKids.push(el('div', { class: 'hint', text: '连续转动：无范围限制，角度自动卷绕到 (-180°, 180°]' }));
        }
      }
      root.appendChild(section('关节 Joint', ...jointKids));

      /* ---- curve ---- */
      const c = node.curve;
      const modeSel = selectInput(c.mode, ['hold', 'sine', 'linear', 'keyframes'], (v) => {
        c.mode = v; refresh();
      });
      const curveKids = [field('曲线模式', modeSel)];
      if (c.mode !== 'keyframes') {
        curveKids.push(field('基准值 ' + j.unit, numInput(c.base, 1, (v) => { c.base = v; refresh(); })));
      }
      if (c.mode === 'sine') {
        curveKids.push(field('幅值', numInput(c.amplitude, 1, (v) => { c.amplitude = v; refresh(); })));
        curveKids.push(field('周期 (秒)', numInput(c.period, 0.5, (v) => { c.period = Math.max(0.1, v); refresh(); })));
        curveKids.push(field('相位 (弧度)', numInput(c.phase, 0.1, (v) => { c.phase = v; refresh(); })));
      }
      if (c.mode === 'linear') {
        curveKids.push(field('速率 (' + j.unit + '/秒)', numInput(c.speed, 1, (v) => { c.speed = v; refresh(); })));
      }
      if (c.mode === 'keyframes') {
        const table = el('div', { class: 'kf-table' });
        const rows = [...c.keyframes].sort((a, b) => a.t - b.t);
        rows.forEach((kf, idx) => {
          const row = el('div', { class: 'kf-row' });
          row.appendChild(numInput(kf.t, 0.1, (v) => { kf.t = Math.max(0, v); refresh(); }));
          row.appendChild(numInput(kf.v, 1, (v) => { kf.v = v; refresh(); }));
          const del = el('button', { class: 'btn small danger', text: '删' });
          del.addEventListener('click', () => {
            c.keyframes = c.keyframes.filter((k) => k !== kf);
            refresh();
          });
          row.appendChild(del);
          table.appendChild(row);
        });
        const add = el('button', { class: 'btn small', text: '+ 添加关键帧' });
        add.addEventListener('click', () => {
          const lastT = c.keyframes.length ? Math.max(...c.keyframes.map((k) => k.t)) + 1 : 0;
          c.keyframes.push({ t: lastT, v: 0 });
          refresh();
        });
        curveKids.push(el('div', { class: 'kf-head' }, '时间 t(秒)', '值 (' + j.unit + ')', ''));
        curveKids.push(table, add);
      }
      root.appendChild(section('时间曲线驱动', ...curveKids));

      /* ---- part ---- */
      const p = node.part;
      root.appendChild(section('零件（仅视觉，尺寸不参与关节约束）',
        field('形状', selectInput(p.shape, ['box', 'cylinder', 'sphere', 'none'], (v) => { p.shape = v; refresh(); })),
        field('颜色', (() => {
          const color = el('input', { type: 'color', value: p.color });
          color.addEventListener('change', () => { p.color = color.value; app.refreshAll(); });
          return color;
        })()),
        field('尺寸 (米)', el('div', { class: 'vec-row' }, ...vec3Inputs(p.size, 0.01, refresh))),
        field('零件偏移 (米)', el('div', { class: 'vec-row' }, ...vec3Inputs(p.partOffset, 0.01, refresh)))
      ));

      /* ---- axes + danger zone ---- */
      const axisChk = el('input', { type: 'checkbox' });
      axisChk.checked = node.showAxes;
      axisChk.addEventListener('change', () => { node.showAxes = axisChk.checked; app.refreshAll(); });
      const delBtn = el('button', { class: 'btn danger full', text: '删除节点' });
      delBtn.addEventListener('click', () => app.deleteSelected());
      root.appendChild(section('显示 / 操作',
        el('label', { class: 'check-line' }, axisChk, '始终显示该节点局部坐标轴'),
        delBtn
      ));
    }

    return { element: root, rerender };
  }

  MR.Inspector = { buildInspector };
})(typeof window !== 'undefined' ? window : globalThis);
