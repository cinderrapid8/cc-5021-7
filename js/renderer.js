/* renderer.js — dependency-free 3D renderer on a 2D canvas.
 * Painter's algorithm with flat shading; draws grid, parts, local axes,
 * end-effector trail and visible limit warnings. */
(function (global) {
  'use strict';

  const MR = global.MR;
  const { V3, M4 } = MR;

  const AXIS_COLORS = { x: '#ff5d5d', y: '#5dd97a', z: '#5d9bff' };
  const LIGHT = V3.norm([0.45, 0.8, 0.6]);

  function shade(hex, factor) {
    const n = parseInt(hex.slice(1), 16);
    const r = Math.min(255, Math.max(0, Math.round(((n >> 16) & 255) * factor)));
    const g = Math.min(255, Math.max(0, Math.round(((n >> 8) & 255) * factor)));
    const b = Math.min(255, Math.max(0, Math.round((n & 255) * factor)));
    return `rgb(${r},${g},${b})`;
  }

  /* ---------- mesh generation (local space, before node frame) ---------- */
  function boxMesh(size, offset) {
    const [hx, hy, hz] = size.map((s) => s / 2);
    const [ox, oy, oz] = offset;
    const c = [
      [-hx, -hy, -hz], [hx, -hy, -hz], [hx, hy, -hz], [-hx, hy, -hz],
      [-hx, -hy, hz], [hx, -hy, hz], [hx, hy, hz], [-hx, hy, hz]
    ].map((p) => [p[0] + ox, p[1] + oy, p[2] + oz]);
    const faces = [
      [0, 3, 2, 1], [4, 5, 6, 7], [0, 1, 5, 4],
      [2, 3, 7, 6], [1, 2, 6, 5], [0, 4, 7, 3]
    ];
    return faces.map((f) => ({ quad: f.map((i) => c[i]) }));
  }

  function cylinderMesh(size, offset, seg = 20) {
    const [r, h, r2] = size;
    const [ox, oy, oz] = offset;
    const ring = (y, rad) => {
      const pts = [];
      for (let i = 0; i < seg; i++) {
        const a = (2 * Math.PI * i) / seg;
        pts.push([ox + rad * Math.cos(a), oy + y, oz + rad * Math.sin(a)]);
      }
      return pts;
    };
    const bottom = ring(-h / 2, r);
    const top = ring(h / 2, r2 ?? r);
    const tb = [ox, oy - h / 2, oz];
    const tt = [ox, oy + h / 2, oz];
    const polys = [];
    for (let i = 0; i < seg; i++) {
      const k = (i + 1) % seg;
      polys.push({ quad: [bottom[i], bottom[k], top[k], top[i]] });
    }
    // end caps as triangle fans
    for (let i = 0; i < seg; i++) {
      const k = (i + 1) % seg;
      polys.push({ tri: [tb, bottom[i], bottom[k]] });
      polys.push({ tri: [tt, top[k], top[i]] });
    }
    return polys;
  }

  function sphereMesh(size, offset, lat = 10, lon = 14) {
    const [rx, ry, rz] = size.map((s) => s / 2);
    const [ox, oy, oz] = offset;
    const pt = (phi, theta) => [
      ox + rx * Math.sin(phi) * Math.cos(theta),
      oy + ry * Math.cos(phi),
      oz + rz * Math.sin(phi) * Math.sin(theta)
    ];
    const polys = [];
    for (let i = 0; i < lat; i++) {
      const p0 = (Math.PI * i) / lat;
      const p1 = (Math.PI * (i + 1)) / lat;
      for (let j = 0; j < lon; j++) {
        const t0 = (2 * Math.PI * j) / lon;
        const t1 = (2 * Math.PI * (j + 1)) / lon;
        polys.push({ quad: [pt(p0, t0), pt(p0, t1), pt(p1, t1), pt(p1, t0)] });
      }
    }
    return polys;
  }

  function meshFor(node) {
    const p = node.part;
    if (p.shape === 'none') return [];
    if (p.shape === 'box') return boxMesh(p.size, p.partOffset);
    if (p.shape === 'cylinder') return cylinderMesh(p.size, p.partOffset);
    if (p.shape === 'sphere') return sphereMesh(p.size, p.partOffset);
    return [];
  }

  function faceNormal(poly) {
    const pts = poly.quad || poly.tri;
    const a = V3.sub(pts[1], pts[0]);
    const b = V3.sub(pts[2], pts[0]);
    return V3.norm(V3.cross(a, b));
  }

  class Renderer {
    constructor(canvas) {
      this.canvas = canvas;
      this.ctx = canvas.getContext('2d');
      this.showGrid = true;
      this.showAllAxes = false;
      this.showLabels = true;
      this.showTrail = true;
      this.trail = [];
      this.maxTrail = 900;
    }

    clearTrail() { this.trail = []; }

    addTrailPoint(p) {
      const last = this.trail[this.trail.length - 1];
      if (!last || V3.len(V3.sub(last, p)) > 1e-5) {
        this.trail.push([...p]);
        if (this.trail.length > this.maxTrail) this.trail.shift();
      }
    }

    render(pose, camera, rig, selectedId) {
      const ctx = this.ctx;
      const w = this.canvas.width;
      const h = this.canvas.height;
      ctx.clearRect(0, 0, w, h);
      ctx.fillStyle = '#0e1320';
      ctx.fillRect(0, 0, w, h);

      if (this.showGrid) this.drawGrid(camera, w, h);

      const items = []; // {depth, draw}
      for (const frame of pose.frames.values()) {
        const node = frame.node;
        if (!node.enabled) continue;
        this.collectPart(items, frame, camera, w, h, node.id === selectedId);
      }
      items.sort((a, b) => b.depth - a.depth);
      for (const it of items) it.draw();

      // overlays drawn on top of solids
      if (this.showTrail) this.drawTrail(camera, w, h);
      for (const frame of pose.frames.values()) {
        const node = frame.node;
        if (!node.enabled) continue;
        if (this.showAllAxes || node.showAxes) this.drawAxes(frame, camera, w, h);
        if (this.showLabels) this.drawLabel(frame, camera, w, h);
        if (frame.clamped) this.drawClampWarning(frame, camera, w, h);
      }
    }

    collectPart(items, frame, camera, w, h, selected) {
      const polys = meshFor(frame.node);
      const worldPolys = [];
      for (const poly of polys) {
        const pts = (poly.quad || poly.tri).map((p) => M4.transformPoint(frame.world, p));
        worldPolys.push({ pts, normal: V3.norm(M4.transformDir(frame.world, faceNormal(poly))) });
      }
      const cam = camera.basis();
      for (const poly of worldPolys) {
        const viewPts = poly.pts.map((p) => camera.toView(p));
        if (viewPts.some((v) => v[2] <= 0.05)) continue;
        const depth = viewPts.reduce((s, v) => s + v[2], 0) / viewPts.length;
        const toCam = V3.norm(V3.sub(cam.eye, poly.pts[0]));
        const facing = V3.dot(poly.normal, toCam);
        if (facing <= 0) continue; // back-face cull
        const baseColor = selected ? '#ffd166' : frame.node.part.color;
        const lit = 0.35 + 0.65 * Math.abs(V3.dot(poly.normal, LIGHT));
        const fill = shade(baseColor, lit);
        items.push({
          depth,
          draw: () => {
            const proj = viewPts.map((v) => {
              const f = (h / 2) / Math.tan(camera.fovY / 2);
              return { x: w / 2 + (v[0] * f) / v[2], y: h / 2 - (v[1] * f) / v[2] };
            });
            const ctx = this.ctx;
            ctx.beginPath();
            ctx.moveTo(proj[0].x, proj[0].y);
            for (let i = 1; i < proj.length; i++) ctx.lineTo(proj[i].x, proj[i].y);
            ctx.closePath();
            ctx.fillStyle = fill;
            ctx.fill();
            ctx.strokeStyle = selected ? '#ffe6a3' : 'rgba(0,0,0,0.45)';
            ctx.lineWidth = selected ? 1.6 : 0.8;
            ctx.stroke();
          }
        });
      }
    }

    drawGrid(camera, w, h) {
      const ctx = this.ctx;
      const ext = 3;
      const step = 0.5;
      ctx.lineWidth = 1;
      for (let i = -ext * 2; i <= ext * 2; i++) {
        const v = i * step;
        const major = i % 2 === 0;
        ctx.strokeStyle = major ? 'rgba(140,165,220,0.28)' : 'rgba(140,165,220,0.12)';
        this.strokeWorld([-ext, 0, v], [ext, 0, v], camera, w, h);
        this.strokeWorld([v, 0, -ext], [v, 0, ext], camera, w, h);
      }
      // world origin axes, slightly above the floor
      this.drawArrow([0, 0.001, 0], [0.35, 0.001, 0], AXIS_COLORS.x, camera, w, h);
      this.drawArrow([0, 0.001, 0], [0, 0.35, 0.001], AXIS_COLORS.y, camera, w, h);
      this.drawArrow([0, 0.001, 0], [0, 0.001, 0.35], AXIS_COLORS.z, camera, w, h);
    }

    strokeWorld(a, b, camera, w, h) {
      const pa = camera.project(a, w, h);
      const pb = camera.project(b, w, h);
      if (!pa || !pb) return;
      const ctx = this.ctx;
      ctx.beginPath();
      ctx.moveTo(pa.x, pa.y);
      ctx.lineTo(pb.x, pb.y);
      ctx.stroke();
    }

    drawArrow(a, b, color, camera, w, h) {
      const ctx = this.ctx;
      const pa = camera.project(a, w, h);
      const pb = camera.project(b, w, h);
      if (!pa || !pb) return;
      ctx.strokeStyle = color;
      ctx.fillStyle = color;
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(pa.x, pa.y);
      ctx.lineTo(pb.x, pb.y);
      ctx.stroke();
      ctx.beginPath();
      ctx.arc(pb.x, pb.y, 2.5, 0, Math.PI * 2);
      ctx.fill();
    }

    drawAxes(frame, camera, w, h) {
      const o = frame.position;
      const len = 0.14;
      // full local basis = rotation part of world matrix
      const xAxis = V3.norm([frame.world[0], frame.world[1], frame.world[2]]);
      const yAxis = V3.norm([frame.world[4], frame.world[5], frame.world[6]]);
      const zAxis = V3.norm([frame.world[8], frame.world[9], frame.world[10]]);
      this.drawArrow(o, V3.add(o, V3.scale(xAxis, len)), AXIS_COLORS.x, camera, w, h);
      this.drawArrow(o, V3.add(o, V3.scale(yAxis, len)), AXIS_COLORS.y, camera, w, h);
      this.drawArrow(o, V3.add(o, V3.scale(zAxis, len)), AXIS_COLORS.z, camera, w, h);
      // joint axis emphasized: white ring at end
      const end = V3.add(o, V3.scale(frame.axisWorld, len * 1.35));
      const p = camera.project(end, w, h);
      if (p) {
        const ctx = this.ctx;
        ctx.strokeStyle = '#ffffff';
        ctx.lineWidth = 1.6;
        ctx.beginPath();
        ctx.arc(p.x, p.y, 4.5, 0, Math.PI * 2);
        ctx.stroke();
      }
    }

    drawLabel(frame, camera, w, h) {
      const p = camera.project(V3.add(frame.position, [0, 0.06, 0]), w, h);
      if (!p) return;
      const ctx = this.ctx;
      ctx.font = '11px system-ui, sans-serif';
      ctx.fillStyle = frame.clamped ? '#ff8f8f' : 'rgba(220,230,255,0.85)';
      ctx.textAlign = 'center';
      ctx.fillText(frame.node.name, p.x, p.y);
    }

    drawClampWarning(frame, camera, w, h) {
      const p = camera.project(V3.add(frame.position, [0, 0.11, 0]), w, h);
      if (!p) return;
      const ctx = this.ctx;
      const pulse = 5 + Math.sin(performance.now() / 180) * 1.6;
      ctx.strokeStyle = '#ff4d4d';
      ctx.lineWidth = 2.4;
      ctx.beginPath();
      ctx.arc(p.x, p.y, pulse + 5, 0, Math.PI * 2);
      ctx.stroke();
      ctx.font = 'bold 10px system-ui, sans-serif';
      ctx.fillStyle = '#ff9d9d';
      ctx.textAlign = 'center';
      ctx.fillText(
        frame.status === 'clamped-low' ? '下限' : '上限',
        p.x, p.y - 12
      );
    }

    drawTrail(camera, w, h) {
      if (this.trail.length < 2) return;
      const ctx = this.ctx;
      ctx.lineWidth = 1.6;
      let prev = null;
      for (let i = 0; i < this.trail.length; i++) {
        const p = camera.project(this.trail[i], w, h);
        if (!p || !prev) { prev = p; continue; }
        const f = i / this.trail.length;
        ctx.strokeStyle = `rgba(125,255,200,${0.15 + 0.75 * f})`;
        ctx.beginPath();
        ctx.moveTo(prev.x, prev.y);
        ctx.lineTo(p.x, p.y);
        ctx.stroke();
        prev = p;
      }
    }
  }

  MR.Renderer = Renderer;
})(typeof window !== 'undefined' ? window : globalThis);
