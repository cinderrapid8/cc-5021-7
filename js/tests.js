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

    /* 9. Four-bar closure vs the Freudenstein analytic reference. The solver
     * must recover both coupler and rocker angles over a full crank turn. */
    {
      const rig = MR.Rig.buildFourBar();
      const idC = rig.nodes.find((n) => n.name.startsWith('连杆')).id;
      const idR = rig.nodes.find((n) => n.name.startsWith('摇杆')).id;
      let seed;
      let maxErr = 0;
      let fails = 0;
      const detail = [];
      for (let deg = 0; deg <= 360; deg += 30) {
        const rep = MR.Solver.solve(rig, deg / 45, seed ? { guess: seed } : {});
        if (!rep.converged) { fails++; continue; }
        seed = rep.seed;
        const ref = MR.Kinematics.fourBarAnalytic(0.35, 1, 0.9, 1.1, deg * Math.PI / 180, 1);
        const fC = rep.pose.frames.get(idC);
        const fR = rep.pose.frames.get(idR);
        const q3 = Math.atan2(fC.world[1], fC.world[0]);
        const q4 = Math.atan2(fR.world[1], fR.world[0]);
        const e = Math.max(
          Math.abs(MR.Tests.angleWrap(q3 - ref.q3)),
          Math.abs(MR.Tests.angleWrap(q4 - ref.q4)),
          V3.len(V3.sub(rep.pose.tipPosition, ref.P))
        );
        maxErr = Math.max(maxErr, e);
        if (deg % 90 === 0) detail.push(`曲柄${deg}° → 摇杆${(q4 * 180 / Math.PI).toFixed(2)}° 解析${(ref.q4 * 180 / Math.PI).toFixed(2)}° 残差${rep.finalResidual.maxAbs.toExponential(1)}m`);
      }
      push('四杆闭环：全周曲柄转动 vs Freudenstein 解析解',
        fails === 0 && maxErr < 1e-6,
        detail.join('\n') + `\n最大姿态误差=${maxErr.toExponential(2)} rad/m`);
    }

    /* 10. Dual-branch shared end effector vs planar 2R inverse kinematics,
     * with branch continuity across the whole driven sweep. */
    {
      const rig = MR.Rig.buildDualArm();
      const b1 = rig.nodes.find((n) => n.name === '支链2 肩 (自由)');
      const b2 = rig.nodes.find((n) => n.name === '支链2 肘 (自由)');
      let seed;
      let maxErr = 0;
      let fails = 0;
      let flips = 0;
      let prev = null;
      for (let t = 0; t <= 8; t += 0.25) {
        const rep = MR.Solver.solve(rig, t, seed ? { guess: seed } : {});
        if (!rep.converged) { fails++; continue; }
        seed = rep.seed;
        const ik = MR.Kinematics.twoRikAnalytic([1.2, 0, 0], 0.6, 0.5, rep.pose.tipPosition, -1);
        const q1 = Math.atan2(rep.pose.frames.get(b1.id).world[1], rep.pose.frames.get(b1.id).world[0]);
        const q2 = Math.atan2(rep.pose.frames.get(b2.id).world[1], rep.pose.frames.get(b2.id).world[0]);
        maxErr = Math.max(maxErr,
          Math.abs(MR.Tests.angleWrap(q1 - ik.q1)),
          Math.abs(MR.Tests.angleWrap(q2 - ik.q2)));
        if (prev !== null && Math.abs(MR.Tests.angleWrap(q1 - prev)) > 1.2) flips++;
        prev = q1;
      }
      push('双支链共末端：自由支链 vs 2R 逆解析，分支连续不跳变',
        fails === 0 && maxErr < 1e-5 && flips === 0,
        `最大角度误差=${maxErr.toExponential(2)} rad，分支跳变=${flips}，失败帧=${fails}`);
    }

    /* 11. Over-constrained inconsistent rig: residual must stay visible and
     * the driven joint values must be exactly the curve values (never tuned
     * to fake a zero residual). */
    {
      const rig = MR.Rig.buildOverConstrained();
      const crank = rig.nodes.find((n) => n.name.startsWith('曲柄'));
      const expectedInput = MR.Kinematics.applyJointLimit(crank, MR.Rig.evalCurve(crank.curve, 1)).applied;
      const rep = MR.Solver.solve(rig, 1, {});
      const frame = rep.pose.frames.get(crank.id);
      const inputUntouched = approx(frame.applied, expectedInput, 1e-12);
      const residualShown = rep.finalResidual.maxAbs > 0.01 && rep.converged === false;
      const flagged = rep.warnings.some((w) => w.includes('过约束'));
      push('过约束不相容：残差如实保留，驱动值不被篡改',
        inputUntouched && residualShown && flagged,
        `驱动应用值=${frame.applied}（期望 ${expectedInput}）；最大残差=${rep.finalResidual.maxAbs.toFixed(4)}m；状态=${rep.status}`);
    }

    /* 12. Under-determined rig: more free joints than equations, solver must
     * report the nullspace dimension and still return a minimum-norm pose. */
    {
      const rig = MR.Rig.createRig('欠约束');
      const o = MR.Rig.createNode({ name: 'O', parentId: null, joint: { type: 'fixed' }, part: { shape: 'none' } });
      const j1 = MR.Rig.createNode({
        name: 'J1', parentId: o.id, joint: { type: 'revolute', axis: [0, 0, 1], limitType: 'continuous', min: 0, max: 0 },
        driven: false, curve: { mode: 'hold', base: 30 }, part: { shape: 'none' }
      });
      const j2 = MR.Rig.createNode({
        name: 'J2', parentId: j1.id, mount: { offset: [0.5, 0, 0] },
        joint: { type: 'revolute', axis: [0, 0, 1], limitType: 'continuous', min: 0, max: 0 },
        driven: false, curve: { mode: 'hold', base: 20 }, part: { shape: 'none' }
      });
      const j3 = MR.Rig.createNode({
        name: 'J3', parentId: j2.id, mount: { offset: [0.5, 0, 0] },
        joint: { type: 'revolute', axis: [0, 0, 1], limitType: 'continuous', min: 0, max: 0 },
        driven: false, curve: { mode: 'hold', base: 10 }, part: { shape: 'none' }
      });
      const tip = MR.Rig.createNode({
        name: 'tip', parentId: j3.id, mount: { offset: [0.4, 0, 0] },
        joint: { type: 'fixed' }, part: { shape: 'none' }
      });
      rig.nodes.push(o, j1, j2, j3, tip);
      rig.tipNodeId = tip.id;
      rig.constraints.push(MR.Rig.createConstraint({
        name: '末端高度 y=0.6', type: 'coincident',
        nodeA: tip.id, nodeB: o.id, offsetB: [0, 0.6, 0], axes: ['y']
      }));
      const rep = MR.Solver.solve(rig, 0, {});
      const yErr = Math.abs(rep.pose.tipPosition[1] - 0.6);
      const nullReported = rep.nullDim === 2 && rep.warnings.some((w) => w.includes('自由度不足'));
      push('自由度不足：零空间维数明确反馈，最小范数解仍满足约束',
        rep.converged && yErr < 1e-6 && nullReported,
        `零空间=${rep.nullDim}（期望 2），y 残差=${yErr.toExponential(2)}m，未知量=${rep.unknownCount} 方程=${rep.equationCount}`);
    }

    /* 13. Dead point / reachability: rocker angles outside the geometric
     * swing range must fail explicitly; inside the range must converge. */
    {
      const rig = MR.Rig.buildDeadPoint();
      const rocker = rig.nodes.find((n) => n.name.startsWith('摇杆'));
      let insideOk = true;
      let outsideFail = true;
      const detail = [];
      for (const deg of [110, 120, 130, 140]) {
        rocker.curve = { mode: 'hold', base: deg, phase: 0 };
        const rep = MR.Solver.solve(rig, 0, {});
        if (!rep.converged) insideOk = false;
        detail.push(`摇杆${deg}°(区内) → ${rep.converged ? '收敛' : rep.status}`);
      }
      for (const deg of [60, 170]) {
        rocker.curve = { mode: 'hold', base: deg, phase: 0 };
        const rep = MR.Solver.solve(rig, 0, {});
        if (rep.converged) outsideFail = false;
        detail.push(`摇杆${deg}°(区外) → ${rep.converged ? '收敛' : rep.status + ' 残差' + rep.finalResidual.maxAbs.toExponential(1)}`);
      }
      push('死点/可达范围：区内可装配，越界明确失败并解释',
        insideOk && outsideFail, detail.join('\n'));
    }

    /* 14. Time reversal: every pose reached while sweeping forward is
     * recovered (within solver tolerance) when sweeping backward. */
    {
      const rig = MR.Rig.buildFourBar();
      const fwd = [];
      let seed;
      for (let t = 0; t <= 4; t += 0.5) {
        const r = MR.Solver.solve(rig, t, seed ? { guess: seed } : {});
        seed = r.seed;
        fwd.push([t, new Map(r.displayValues)]);
      }
      let maxDiff = 0;
      for (let i = fwd.length - 1; i >= 0; i--) {
        const [t, vals] = fwd[i];
        const r = MR.Solver.solve(rig, t, seed ? { guess: seed } : {});
        seed = r.seed;
        for (const [id, v] of vals) {
          maxDiff = Math.max(maxDiff, Math.abs(MR.Tests.angleWrap(
            (r.displayValues.get(id) - v) * Math.PI / 180)));
        }
      }
      push('时间倒放：逆向播放恢复相同姿态（分支连续）',
        maxDiff < 1e-5, `倒放姿态最大偏差=${maxDiff.toExponential(2)} rad`);
    }

    /* 15. Parameter jump without a warm start still assembles via the
     * multi-start fallback instead of returning a fake converged frame. */
    {
      const rig = MR.Rig.buildFourBar();
      const rep = MR.Solver.solve(rig, 4, {});
      const residualHonest = rep.finalResidual.maxAbs <= 1e-7;
      push('参数突变（无热启动）：多起点回退找到真实装配姿态',
        rep.converged && residualHonest,
        `尝试初值=${rep.attempts} 组，残差=${rep.finalResidual.maxAbs.toExponential(2)}m，迭代=${rep.iterations}`);
    }

    /* 16. Structural validation catches dangling constraint references. */
    {
      const rig = MR.Rig.buildFourBar();
      rig.constraints.push(MR.Rig.createConstraint({
        name: '坏引用', type: 'distance', nodeA: 'ghost', nodeB: rig.nodes[0].id, distance: 0.2
      }));
      const errors = MR.Rig.validateConstraints(rig);
      const rep = MR.Solver.solve(rig, 0, {});
      push('约束结构校验：失效节点引用被检出并阻断求解',
        errors.some((e) => e.includes('不存在')) && rep.status === 'structural-error',
        errors.join('；'));
    }

    /* 17. Prismatic free joint: a crank pin drives a Y-axis slider through a
     * single coincident-Y constraint; slider position must match r·sin(theta). */
    {
      const rig = MR.Rig.createRig('滑块闭环');
      const O = MR.Rig.createNode({ name: 'O', parentId: null, joint: { type: 'fixed' }, part: { shape: 'none' } });
      const crank = MR.Rig.createNode({
        name: '曲柄', parentId: O.id,
        joint: { type: 'continuous', axis: [0, 0, 1], limitType: 'continuous', min: 0, max: 0 },
        curve: { mode: 'linear', base: 0, speed: 30 }, part: { shape: 'none' }
      });
      const pin = MR.Rig.createNode({
        name: '销', parentId: crank.id, mount: { offset: [0.3, 0, 0] },
        joint: { type: 'fixed' }, part: { shape: 'none' }
      });
      const slider = MR.Rig.createNode({
        name: '滑块(自由)', parentId: null,
        joint: { type: 'prismatic', axis: [0, 1, 0], unit: 'm', min: -2, max: 2 },
        driven: false, curve: { mode: 'hold', base: 0 }, part: { shape: 'none' }
      });
      rig.nodes.push(O, crank, pin, slider);
      rig.tipNodeId = pin.id;
      rig.constraints.push(MR.Rig.createConstraint({
        name: '销Y=滑块Y', type: 'coincident',
        nodeA: pin.id, nodeB: slider.id, axes: ['y']
      }));
      let seed;
      let maxErr = 0;
      let fails = 0;
      for (let deg = 0; deg <= 360; deg += 30) {
        const rep = MR.Solver.solve(rig, deg / 30, seed ? { guess: seed } : {});
        if (!rep.converged) { fails++; continue; }
        seed = rep.seed;
        const got = rep.pose.frames.get(slider.id).applied;
        const expect = 0.3 * Math.sin(deg * Math.PI / 180);
        maxErr = Math.max(maxErr, Math.abs(got - expect));
      }
      push('移动副自由关节：滑块位移 vs r·sinθ 解析关系',
        fails === 0 && maxErr < 1e-6,
        `最大位移误差=${maxErr.toExponential(2)} m，失败帧=${fails}`);
    }

    return results;
  }

  function angleWrap(a) {
    while (a > Math.PI) a -= 2 * Math.PI;
    while (a < -Math.PI) a += 2 * Math.PI;
    return a;
  }

  MR.Tests = { runAll, approx, vapprox, angleWrap };

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
