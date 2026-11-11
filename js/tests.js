/* tests.js — analytic verification suite.
 * Runs in both the browser (UI panel) and Node (headless self-check). */
(function (global) {
  'use strict';

  const MR = global.MR;
  const { V3 } = MR;
  const EPS = 1e-6;

  function approx(a, b, eps = EPS) {
    return Math.abs(a - b) <= eps;
  }
  function vapprox(a, b, eps = EPS) {
    return V3.len(V3.sub(a, b)) <= eps;
  }

  function runAll() {
    const results = [];
    const push = (name, pass, detail) => results.push({ name, pass, detail });

    /* 1. Two-link arm, known angles, compared against the closed form. */
    {
      const rig = MR.Rig.buildTwoLink();
      const j1 = rig.nodes.find((n) => n.name.startsWith('根旋转'));
      const j2 = rig.nodes.find((n) => n.name.startsWith('肘关节'));
      const L1 = 1, L2 = 0.8;
      const cases = [[30, 60], [-45, 30], [120, -50]];
      let ok = true;
      const details = [];
      for (const [a, b] of cases) {
        j1.curve.base = a;
        j2.curve.base = b;
        const pose = MR.Kinematics.evaluate(rig, 0);
        const expect = MR.Kinematics.twoLinkAnalytic(
          [0, 0, 0], L1, L2, a * Math.PI / 180, b * Math.PI / 180
        );
        const got = pose.tipPosition;
        const good = vapprox(got, expect, 1e-9);
        ok = ok && good;
        details.push(`θ1=${a}° θ2=${b}° → 解析[${expect.map((x) => x.toFixed(4))}] 求解[${got.map((x) => x.toFixed(4))}]`);
      }
      push('两节臂末端位置 vs 解析公式 (3 组角度)', ok, details.join('\n'));
    }

    /* 2. Slide + revolute arm against the closed form. */
    {
      const rig = MR.Rig.buildSlide();
      const carriage = rig.nodes[0];
      const j1 = rig.nodes.find((n) => n.name.includes('旋转'));
      const L = 0.7;
      let ok = true;
      const details = [];
      for (const [d, thetaDeg] of [[0.4, 45], [-0.6, -30], [0.9, 120]]) {
        carriage.curve.base = d;
        j1.curve.base = thetaDeg;
        const pose = MR.Kinematics.evaluate(rig, 0);
        const expect = MR.Kinematics.slideAnalytic(d, L, thetaDeg * Math.PI / 180);
        const good = vapprox(pose.tipPosition, expect, 1e-9);
        ok = ok && good;
        details.push(`d=${d}m θ=${thetaDeg}° → 解析[${expect.map((x) => x.toFixed(4))}] 求解[${pose.tipPosition.map((x) => x.toFixed(4))}]`);
      }
      push('滑轨+单臂末端位置 vs 解析公式 (3 组)', ok, details.join('\n'));
    }

    /* 3. Angle wrap across the ±180 boundary (continuous joint). */
    {
      const rig = MR.Rig.buildMechCharacter();
      const base = rig.nodes[0]; // continuous turntable
      base.curve = { mode: 'hold', base: 270 };
      const f = MR.Kinematics.applyJointLimit(base, 270);
      const pose = MR.Kinematics.evaluate(rig, 0);
      const frame = pose.frames.get(base.id);
      // continuous value wrapped to -90; geometry must be unchanged size
      const wrapOk = approx(frame.applied, -90, 1e-9);
      const geoOk = frame.node.part.size[0] === 0.28;
      push('连续转动跨界 270° → -90°（且几何尺寸未被修改）',
        wrapOk && geoOk,
        `应用值=${frame.applied}°, 底座半径仍=${frame.node.part.size[0]}m`);
    }

    /* 4. Root transform shifts the whole rigid pose. */
    {
      const rig = MR.Rig.buildTwoLink();
      const p0 = MR.Kinematics.evaluate(rig, 0).tipPosition;
      rig.rootTransform.position = [1.5, -0.4, 0.7];
      rig.rootTransform.eulerDeg = [0, 90, 0];
      const pose = MR.Kinematics.evaluate(rig, 0);
      const shifted = V3.sub(pose.tipPosition, p0);
      // expected transform: RotY(90)*p + t
      const p = p0;
      const expect = [
        1.5 + p[2],
        -0.4 + p[1],
        0.7 - p[0]
      ];
      push('根变换（平移+Y轴90°）整体作用于末端',
        vapprox(pose.tipPosition, expect, 1e-9),
        `期望[${expect.map((x) => x.toFixed(4))}] 实际[${pose.tipPosition.map((x) => x.toFixed(4))}]`);
    }

    /* 5. Limit positions: overshoot is clamped, visibly flagged, geometry intact. */
    {
      const rig = MR.Rig.buildLimitDemo();
      const j1 = rig.nodes[0];
      const slide = rig.nodes.find((n) => n.joint.type === 'prismatic');
      j1.curve = { mode: 'hold', base: 200 };   // range ±45
      slide.curve = { mode: 'hold', base: -50 }; // range 0..10 cm
      const pose = MR.Kinematics.evaluate(rig, 0);
      const fj = pose.frames.get(j1.id);
      const fs = pose.frames.get(slide.id);
      const ok =
        approx(fj.applied, 45, 1e-12) && fj.status === 'clamped-high' &&
        approx(fs.applied, 0, 1e-12) && fs.status === 'clamped-low' &&
        j1.part.size[0] === 0.06 && slide.part.size[1] === 0.05;
      push('极限位置：越界钳制 + 越界标记，几何不被篡改',
        ok,
        `转动 raw=200→applied=${fj.applied}(${fj.status}); 滑动 raw=-50→applied=${fs.applied}(${fs.status})`);
    }

    /* 6. Parent rotation must carry the child joint axis into the world. */
    {
      const rig = MR.Rig.createRig('axis test');
      const parent = MR.Rig.createNode({
        name: '父', parentId: null,
        joint: { type: 'revolute', axis: [0, 1, 0], unit: 'deg', min: -180, max: 180 },
        curve: { mode: 'hold', base: 90 },
        part: { shape: 'none' }
      });
      const child = MR.Rig.createNode({
        name: '子', parentId: parent.id,
        mount: { offset: [0.3, 0, 0] },
        joint: { type: 'revolute', axis: [1, 0, 0], unit: 'deg', min: -90, max: 90 },
        curve: { mode: 'hold', base: 0 },
        part: { shape: 'none' }
      });
      rig.nodes.push(parent, child);
      const pose = MR.Kinematics.evaluate(rig, 0);
      // parent RotY(90): child local +X axis maps to world -Z
      const worldAxis = pose.frames.get(child.id).axisWorld;
      const ok = vapprox(worldAxis, [0, 0, -1], 1e-9);
      // and the child mount point [0.3,0,0] rotates to [0,0,-0.3]
      const posOk = vapprox(pose.frames.get(child.id).position, [0, 0, -0.3], 1e-9);
      push('父层旋转后子关节轴随之旋转（不固定在世界方向）',
        ok && posOk,
        `子轴世界方向=[${worldAxis.map((x) => x.toFixed(4))}] 期望[0,0,-1]; 子原点=[${pose.frames.get(child.id).position.map((x) => x.toFixed(4))}]`);
    }

    /* 7. Save / reload round trip preserves the pose exactly. */
    {
      const rig = MR.Rig.buildMechCharacter();
      rig.rootTransform = { position: [0.2, 0, -0.1], eulerDeg: [10, 20, 30] };
      const t = 1.37;
      const before = MR.Kinematics.evaluate(rig, t);
      const json = JSON.stringify(rig);
      const restored = JSON.parse(json);
      const after = MR.Kinematics.evaluate(restored, t);
      let ok = vapprox(before.tipPosition, after.tipPosition, 1e-12);
      const details = [`保存前末端[${before.tipPosition.map((x) => x.toFixed(6))}]`,
        `重载后末端[${after.tipPosition.map((x) => x.toFixed(6))}]`];
      for (const id of before.frames.keys()) {
        if (!vapprox(before.frames.get(id).position, after.frames.get(id).position, 1e-12)) ok = false;
      }
      push('保存→重新加载保持相同姿态', ok, details.join('\n'));
    }

    /* 8. Hierarchy checks: dangling reference and cycle. */
    {
      const rig = MR.Rig.createRig('bad refs');
      const a = MR.Rig.createNode({ name: 'A', parentId: null, part: { shape: 'none' } });
      const b = MR.Rig.createNode({ name: 'B', parentId: a.id, part: { shape: 'none' } });
      const c = MR.Rig.createNode({ name: 'C', parentId: 'ghost-id', part: { shape: 'none' } });
      rig.nodes.push(a, b, c);
      const v1 = MR.Rig.validateHierarchy(rig.nodes);
      const danglingCaught = v1.errors.some((e) => e.includes('失效引用'));
      // force a cycle A->B->A
      a.parentId = b.id;
      const v2 = MR.Rig.validateHierarchy(rig.nodes);
      const cycleCaught = v2.errors.some((e) => e.includes('循环'));
      const orderComplete = v2.order.length === 3;
      // UI guard must reject the reparent that would create a cycle
      const guard = MR.Rig.canReparent(rig.nodes, a.id, b.id);
      push('层级校验：失效引用与循环均被检出，求值顺序仍确定',
        danglingCaught && cycleCaught && orderComplete && guard.ok === false,
        `失效引用=${danglingCaught}; 循环=${cycleCaught}; 顺序长度=${v2.order.length}; 防重挂拒绝=${!guard.ok}`);
    }

    return results;
  }

  MR.Tests = { runAll, approx, vapprox };

  /* Node headless entry point. */
  if (typeof require === 'function' && typeof module !== 'undefined' && global.__MR_NODE) {
    const results = runAll();
    let failed = 0;
    for (const r of results) {
      console.log(`${r.pass ? 'PASS' : 'FAIL'}  ${r.name}`);
      if (r.detail) console.log(r.detail.split('\n').map((l) => '      ' + l).join('\n'));
      if (!r.pass) failed++;
    }
    console.log(`\n${results.length - failed}/${results.length} passed`);
    process.exitCode = failed ? 1 : 0;
  }
})(typeof window !== 'undefined' ? window : globalThis);
