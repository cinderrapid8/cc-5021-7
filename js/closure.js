/* closure.js — closure-constraint panel: constraint editor, per-iteration
 * residual table, solver diagnostics and failed-frame export/replay. */
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

  const STATUS_TEXT = {
    converged: '已收敛',
    'no-constraints': '无闭环（直接求值）',
    inconsistent: '过约束不相容',
    unreachable: '不可达 / 死点',
    'no-progress': '迭代停滞',
    'max-iterations': '达到迭代上限',
    'structural-error': '约束结构错误'
  };

  function buildClosurePanel(app) {
    const { state } = MR.State;
    const root = el('div', { id: 'closure-panel' });
    const MAX_FAILED = 20;

    function nodeName(id) {
      return state.rig.nodes.find((n) => n.id === id)?.name || id;
    }

    function download(name, text) {
      const blob = new Blob([text], { type: 'application/json' });
      const url = URL.createObjectURL(blob);
      const a = document.createElement('a');
      a.href = url;
      a.download = name;
      a.click();
      URL.revokeObjectURL(url);
    }

    function addConstraint() {
      if (!Array.isArray(state.rig.constraints)) state.rig.constraints = [];
      const first = state.rig.nodes[0];
      const last = state.rig.nodes[state.rig.nodes.length - 1];
      state.rig.constraints.push(MR.Rig.createConstraint({
        name: '新闭合约束',
        type: 'coincident',
        nodeA: first?.id,
        nodeB: last?.id,
        axes: ['x', 'y']
      }));
      app.refreshAll();
    }

    function nodeSelect(currentId, onchange) {
      const s = el('select', { class: 'full' });
      for (const n of state.rig.nodes) {
        const opt = el('option', { value: n.id, text: n.name });
        if (n.id === currentId) opt.selected = true;
        s.appendChild(opt);
      }
      s.addEventListener('change', () => onchange(s.value));
      return s;
    }

    function vecEditor(vec, onchange) {
      return el('div', { class: 'vec-row' },
        [0, 1, 2].map((i) => {
          const inp = el('input', {
            type: 'number', step: '0.01', class: 'num', value: String(vec[i])
          });
          inp.addEventListener('change', () => { vec[i] = parseFloat(inp.value) || 0; onchange(); });
          return el('label', { class: 'vec-cell' }, ['X', 'Y', 'Z'][i], inp);
        }));
    }

    function renderConstraintEditor() {
      const host = el('div', { class: 'constraint-list' });
      const list = state.rig.constraints || [];
      if (!list.length) {
        host.appendChild(el('div', { class: 'empty-hint', text: '当前机构无闭合约束，按树状机构直接求值' }));
      }
      list.forEach((c, idx) => {
        const card = el('div', { class: 'constraint-card' + (c.enabled === false ? ' disabled' : '') });
        card.appendChild(el('div', { class: 'constraint-head' },
          el('input', {
            type: 'checkbox',
            ...(c.enabled !== false ? { checked: 'checked' } : {}),
            onchange: (e) => { c.enabled = e.target.checked; app.refreshAll(); }
          }),
          el('input', {
            class: 'constraint-name', value: c.name,
            onchange: (e) => { c.name = e.target.value; app.refreshAll(); }
          }),
          el('button', {
            class: 'btn small danger', text: '删',
            onclick: () => {
              state.rig.constraints.splice(idx, 1);
              app.refreshAll();
            }
          })
        ));
        const typeSel = el('select', { class: 'full' },
          ...MR.Rig.CONSTRAINT_TYPES.map((ty) => {
            const o = el('option', { value: ty, text: ty === 'coincident' ? '销轴重合 (coincident)' : '定距 (distance)' });
            if (ty === c.type) o.selected = true;
            return o;
          }));
        typeSel.addEventListener('change', () => { c.type = typeSel.value; app.refreshAll(); });
        card.appendChild(field('类型', typeSel));
        card.appendChild(field('端点 A', nodeSelect(c.nodeA, (v) => { c.nodeA = v; app.refreshAll(); })));
        card.appendChild(field('端点 B', nodeSelect(c.nodeB, (v) => { c.nodeB = v; app.refreshAll(); })));
        card.appendChild(field('A 锚点偏移 (米)', vecEditor(c.offsetA, () => app.scheduleSave())));
        card.appendChild(field('B 锚点偏移 (米)', vecEditor(c.offsetB, () => app.scheduleSave())));
        if (c.type === 'distance') {
          const d = el('input', { type: 'number', step: '0.01', class: 'num', value: String(c.distance) });
          d.addEventListener('change', () => { c.distance = parseFloat(d.value) || 0; app.refreshAll(); });
          card.appendChild(field('目标距离 (米)', d));
        }
        const axisRow = el('div', { class: 'axis-row' });
        for (const ax of MR.Rig.CONSTRAINT_AXES) {
          const chk = el('input', { type: 'checkbox' });
          chk.checked = c.axes.includes(ax);
          chk.addEventListener('change', () => {
            const set = new Set(c.axes);
            if (chk.checked) set.add(ax); else set.delete(ax);
            c.axes = MR.Rig.CONSTRAINT_AXES.filter((a) => set.has(a));
            app.refreshAll();
          });
          axisRow.appendChild(el('label', { class: 'chk' }, chk, ax.toUpperCase()));
        }
        card.appendChild(field('参与轴', axisRow));
        host.appendChild(card);
      });
      return host;
    }

    function field(label, kid) {
      return el('div', { class: 'field' },
        el('span', { class: 'field-label', text: label }), kid);
    }

    function renderReport(rep) {
      const box = el('div', { class: 'solver-report' });
      if (!rep) {
        box.appendChild(el('div', { class: 'empty-hint', text: '尚未求解' }));
        return box;
      }
      const ok = rep.converged;
      const head = el('div', { class: 'solver-status ' + (ok ? 'ok' : 'bad') },
        `${STATUS_TEXT[rep.status] || rep.status} · 残差 ${rep.finalResidual ? sci(rep.finalResidual.maxAbs) : '—'} m`);
      box.appendChild(head);

      const meta = el('div', { class: 'solver-meta' },
        `驱动关节 ${rep.drivenCount} · 自由未知量 ${rep.unknownCount} · 约束方程 ${rep.equationCount}`,
        el('br'),
        `迭代 ${rep.iterations} 次 · 尝试初值 ${rep.attempts || 1} 组 · 雅可比秩 ${rep.rank}/${rep.unknownCount} · 零空间维数 ${rep.nullDim}`
      );
      box.appendChild(meta);

      if (rep.warnings.length) {
        const ul = el('div', { class: 'solver-warnings' });
        for (const w of rep.warnings) ul.appendChild(el('div', { text: '⚠ ' + w }));
        box.appendChild(ul);
      }
      if (rep.structuralErrors.length) {
        const ul = el('div', { class: 'solver-warnings' });
        for (const w of rep.structuralErrors) ul.appendChild(el('div', { text: '✗ ' + w }));
        box.appendChild(ul);
      }

      if (rep.finalResidual?.perConstraint?.length) {
        const table = el('div', { class: 'res-table' });
        table.appendChild(el('div', { class: 'res-row res-head' }, '约束', '闭合误差 (m)'));
        for (const pc of rep.finalResidual.perConstraint) {
          const bad = pc.norm > MR.Solver.DEFAULTS.tolerance;
          table.appendChild(el('div', { class: 'res-row' + (bad ? ' bad' : ' ok') },
            pc.name, sci(pc.norm)));
        }
        box.appendChild(table);
      }

      if (rep.iterationsLog.length) {
        const det = el('details', { class: 'iter-details' });
        det.appendChild(el('summary', { text: `每次迭代残差（${rep.iterationsLog.length} 行）` }));
        const grid = el('div', { class: 'iter-grid' });
        grid.appendChild(el('div', { class: 'iter-head' }, '尝试', '迭代', '残差范数', '最大分量', '秩', '零空间', '步长', 'λ', '接受'));
        for (const l of rep.iterationsLog) {
          grid.appendChild(el('div', { class: 'iter-row' },
            String(l.attempt ?? 1), String(l.iter), sci(l.residualNorm), sci(l.maxAbs),
            String(l.rank), String(l.nullDim), sci(l.dxNorm), sci(l.lambda),
            l.accepted ? '✓' : '✗'));
        }
        det.appendChild(grid);
        box.appendChild(det);
      }
      return box;
    }

    function sci(v) {
      if (v === null || v === undefined) return '—';
      if (!Number.isFinite(v)) return '∞';
      if (v === 0) return '0';
      return v.toExponential(2);
    }

    function recordFailure(rep) {
      if (!state.failedFrames) state.failedFrames = [];
      const rig = JSON.parse(JSON.stringify(state.rig));
      state.failedFrames.push({
        recordedAt: new Date().toISOString(),
        t: state.time,
        status: rep.status,
        maxAbs: rep.finalResidual?.maxAbs ?? null,
        warnings: rep.warnings,
        iterationsLog: rep.iterationsLog,
        seed: rep.seed ? Object.fromEntries(rep.seed) : {},
        rig
      });
      if (state.failedFrames.length > MAX_FAILED) state.failedFrames.shift();
    }

    function renderFailedFrames() {
      const box = el('div', { class: 'failed-frames' });
      const frames = state.failedFrames || [];
      box.appendChild(el('div', { class: 'section-title' },
        `失败帧 (${frames.length}/${MAX_FAILED})`));
      if (!frames.length) {
        box.appendChild(el('div', { class: 'hint', text: '暂无失败帧。求解失败（不可达/过约束/迭代停滞）时会自动记录。' }));
      }
      frames.forEach((f, i) => {
        const row = el('div', { class: 'failed-row' },
          el('div', { class: 'failed-title', text: `#${i + 1} t=${f.t.toFixed(2)}s · ${STATUS_TEXT[f.status] || f.status} · ${sci(f.maxAbs)}m` }),
          el('div', { class: 'btn-group' },
            el('button', {
              class: 'btn small', text: '复现',
              onclick: () => {
                state.rig = f.rig;
                state.time = f.t;
                state.playing = false;
                document.querySelector('#btn-play').textContent = '播放';
                app.clearSolverSeed();
                app.refreshAll();
                app.flash(`已载入失败帧 #${i + 1}，可在面板查看迭代残差并调整参数`);
              }
            }),
            el('button', {
              class: 'btn small', text: '导出',
              onclick: () => download(
                `failed-frame-${i + 1}-t${f.t.toFixed(2)}.json`,
                JSON.stringify({ format: 'mr-failed-frame', version: 1, ...f }, null, 2))
            }),
            el('button', {
              class: 'btn small danger', text: '删',
              onclick: () => { state.failedFrames.splice(i, 1); rerender(); }
            })
          )
        );
        box.appendChild(row);
      });
      if (frames.length) {
        box.appendChild(el('div', { class: 'btn-group' },
          el('button', {
            class: 'btn small', text: '导出全部失败帧',
            onclick: () => download('failed-frames.json',
              JSON.stringify({ format: 'mr-failed-frames', version: 1, frames }, null, 2))
          }),
          el('button', {
            class: 'btn small danger', text: '清空',
            onclick: () => { state.failedFrames = []; rerender(); }
          })
        ));
      }
      return box;
    }

    function rerender() {
      root.innerHTML = '';
      const has = MR.Solver.hasConstraints(state.rig);
      const summary = el('div', { class: 'hint' },
        has
          ? '本机构含闭合约束：驱动关节取曲线值（只读），自由关节由约束迭代求解。'
          : '本机构为无闭环树状机构，沿层级直接正向求值。');
      root.appendChild(summary);

      const editorSection = el('div', { class: 'section' },
        el('div', { class: 'section-title' }, '闭合约束编辑'),
        renderConstraintEditor(),
        el('button', { class: 'btn full', text: '+ 添加约束', onclick: addConstraint })
      );
      root.appendChild(editorSection);

      root.appendChild(el('div', { class: 'section' },
        el('div', { class: 'section-title' }, '求解结果'),
        renderReport(app.lastSolverReport)));

      root.appendChild(el('div', { class: 'section' }, renderFailedFrames()));
    }

    return { element: root, rerender, recordFailure };
  }

  MR.Closure = { buildClosurePanel, STATUS_TEXT };
})(typeof window !== 'undefined' ? window : globalThis);
