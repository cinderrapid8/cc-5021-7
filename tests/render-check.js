global.__MR_NODE = true;
require('/workspace/js/math3d.js');
require('/workspace/js/lineq.js');
require('/workspace/js/rig.js');
require('/workspace/js/kinematics.js');
require('/workspace/js/solver.js');
require('/workspace/js/renderer.js');

const fakeCanvas = {
  width: 800, height: 600,
  getContext: () => new Proxy({}, {
    get: (t, p) => (p in t ? t[p] : () => (p === 'measureText' ? { width: 5 } : undefined)),
    set: (t, p, v) => { t[p] = v; return true; }
  })
};
const r = new MR.Renderer(fakeCanvas);
const cam = new MR.OrbitCamera();

for (const [name, build] of [
  ['角色', MR.Rig.buildMechCharacter],
  ['两节臂', MR.Rig.buildTwoLink],
  ['滑轨', MR.Rig.buildSlide],
  ['极限', MR.Rig.buildLimitDemo],
  ['四杆', MR.Rig.buildFourBar],
  ['双支链', MR.Rig.buildDualArm],
  ['过约束', MR.Rig.buildOverConstrained],
  ['死点', MR.Rig.buildDeadPoint]
]) {
  const rig = build();
  let seed = null;
  for (const t of [0, 0.7, 1.5, 3.1]) {
    let pose;
    let report = null;
    if (MR.Solver.hasConstraints(rig)) {
      const rep = MR.Solver.solve(rig, t, seed ? { guess: seed } : {});
      if (rep.converged) seed = rep.seed;
      pose = rep.pose;
      report = rep;
    } else {
      pose = MR.Kinematics.evaluate(rig, t);
    }
    if (pose.tipPosition) r.addTrailPoint(pose.tipPosition);
    r.render(pose, cam, rig, rig.nodes[0].id, report);
  }
  console.log(name, 'OK, trail points =', r.trail.length);
  r.clearTrail();
}

cam.pitch = 1.3; cam.yaw = 2.2; cam.distance = 1.2;
r.render(MR.Kinematics.evaluate(MR.Rig.buildMechCharacter(), 1), cam, MR.Rig.buildMechCharacter(), null);
console.log('极端相机角度 OK');
