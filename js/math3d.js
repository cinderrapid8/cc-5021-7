/* math3d.js — column-major 4x4 matrix math, no dependencies.
 * Matrices are plain Float64Array(16), index = column*4 + row. */
(function (global) {
  'use strict';

  const MR = global.MR || (global.MR = {});

  const V3 = {
    create: (x = 0, y = 0, z = 0) => [x, y, z],
    clone: (v) => [v[0], v[1], v[2]],
    add: (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]],
    sub: (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]],
    scale: (v, s) => [v[0] * s, v[1] * s, v[2] * s],
    dot: (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2],
    cross: (a, b) => [
      a[1] * b[2] - a[2] * b[1],
      a[2] * b[0] - a[0] * b[2],
      a[0] * b[1] - a[1] * b[0]
    ],
    len: (v) => Math.hypot(v[0], v[1], v[2]),
    norm(v) {
      const l = Math.hypot(v[0], v[1], v[2]) || 1;
      return [v[0] / l, v[1] / l, v[2] / l];
    },
    lerp: (a, b, t) => [
      a[0] + (b[0] - a[0]) * t,
      a[1] + (b[1] - a[1]) * t,
      a[2] + (b[2] - a[2]) * t
    ],
    equal: (a, b, eps = 1e-10) =>
      Math.abs(a[0] - b[0]) <= eps &&
      Math.abs(a[1] - b[1]) <= eps &&
      Math.abs(a[2] - b[2]) <= eps
  };

  const M4 = {
    identity: () => {
      const m = new Float64Array(16);
      m[0] = m[5] = m[10] = m[15] = 1;
      return m;
    },

    /* c * r index helper */
    idx: (r, c) => c * 4 + r,

    multiply(a, b) {
      const out = new Float64Array(16);
      for (let c = 0; c < 4; c++) {
        for (let r = 0; r < 4; r++) {
          let s = 0;
          for (let k = 0; k < 4; k++) {
            s += a[k * 4 + r] * b[c * 4 + k];
          }
          out[c * 4 + r] = s;
        }
      }
      return out;
    },

    fromTranslation(v) {
      const m = M4.identity();
      m[12] = v[0];
      m[13] = v[1];
      m[14] = v[2];
      return m;
    },

    /* Rodrigues axis-angle. axis must be a unit vector. angle in radians. */
    fromAxisAngle(axis, angle) {
      const [x, y, z] = axis;
      const c = Math.cos(angle);
      const s = Math.sin(angle);
      const t = 1 - c;
      const m = M4.identity();
      m[0] = c + x * x * t;
      m[1] = y * x * t + z * s;
      m[2] = z * x * t - y * s;
      m[4] = x * y * t - z * s;
      m[5] = c + y * y * t;
      m[6] = z * y * t + x * s;
      m[8] = x * z * t + y * s;
      m[9] = y * z * t - x * s;
      m[10] = c + z * z * t;
      return m;
    },

    /* Axis must be unit. d in metres for prismatic joints. */
    fromAxisTranslation(axis, d) {
      const m = M4.identity();
      m[12] = axis[0] * d;
      m[13] = axis[1] * d;
      m[14] = axis[2] * d;
      return m;
    },

    fromEulerXYZ(rx, ry, rz) {
      const cx = Math.cos(rx), sx = Math.sin(rx);
      const cy = Math.cos(ry), sy = Math.sin(ry);
      const cz = Math.cos(rz), sz = Math.sin(rz);
      // R = Rz * Ry * Rx, row-major entries M[row][col],
      // stored column-major at m[col*4 + row].
      const m = M4.identity();
      m[0] = cy * cz;                   // M00
      m[1] = cy * sz;                   // M10
      m[2] = -sy;                       // M20
      m[4] = cz * sy * sx - sz * cx;    // M01
      m[5] = sz * sy * sx + cz * cx;    // M11
      m[6] = cy * sx;                   // M21
      m[8] = cz * sy * cx + sz * sx;    // M02
      m[9] = sz * sy * cx - cz * sx;    // M12
      m[10] = cy * cx;                  // M22
      return m;
    },

    transformPoint(m, p) {
      const x = p[0], y = p[1], z = p[2];
      return [
        m[0] * x + m[4] * y + m[8] * z + m[12],
        m[1] * x + m[5] * y + m[9] * z + m[13],
        m[2] * x + m[6] * y + m[10] * z + m[14]
      ];
    },

    /* Extract a direction vector (w=0), e.g. local axis -> world axis. */
    transformDir(m, d) {
      const x = d[0], y = d[1], z = d[2];
      return [
        m[0] * x + m[4] * y + m[8] * z,
        m[1] * x + m[5] * y + m[9] * z,
        m[2] * x + m[6] * y + m[10] * z
      ];
    },

    /* Inverse of an affine matrix with orthonormal rotation part. */
    invertAffine(m) {
      const out = new Float64Array(16);
      // transpose 3x3
      for (let r = 0; r < 3; r++) {
        for (let c = 0; c < 3; c++) {
          out[c * 4 + r] = m[r * 4 + c];
        }
      }
      const tx = m[12], ty = m[13], tz = m[14];
      out[12] = -(out[0] * tx + out[4] * ty + out[8] * tz);
      out[13] = -(out[1] * tx + out[5] * ty + out[9] * tz);
      out[14] = -(out[2] * tx + out[6] * ty + out[10] * tz);
      out[15] = 1;
      return out;
    },

    getTranslation(m) {
      return [m[12], m[13], m[14]];
    },

    /* Row-major nested array for table rendering. */
    toRows(m) {
      const rows = [];
      for (let r = 0; r < 4; r++) {
        const row = [];
        for (let c = 0; c < 4; c++) row.push(m[c * 4 + r]);
        rows.push(row);
      }
      return rows;
    }
  };

  /* Orbit camera producing view-space points for perspective projection. */
  class OrbitCamera {
    constructor() {
      this.target = [0, 0.5, 0];
      this.distance = 4.2;
      this.yaw = -0.6;
      this.pitch = 0.32;
      this.fovY = 42 * Math.PI / 180;
    }
    basis() {
      const cp = Math.cos(this.pitch);
      const eye = [
        this.target[0] + this.distance * cp * Math.sin(this.yaw),
        this.target[1] + this.distance * Math.sin(this.pitch),
        this.target[2] + this.distance * cp * Math.cos(this.yaw)
      ];
      const forward = V3.norm(V3.sub(this.target, eye));
      const right = V3.norm(V3.cross(forward, [0, 1, 0]));
      const up = V3.cross(right, forward);
      return { eye, forward, right, up };
    }
    /* World point -> view coords (x right, y up, z depth forward). */
    toView(p) {
      const b = this.basis();
      const rel = V3.sub(p, b.eye);
      return [
        V3.dot(rel, b.right),
        V3.dot(rel, b.up),
        V3.dot(rel, b.forward)
      ];
    }
    project(p, w, h) {
      const v = this.toView(p);
      if (v[2] <= 0.05) return null;
      const f = (h / 2) / Math.tan(this.fovY / 2);
      return {
        x: w / 2 + (v[0] * f) / v[2],
        y: h / 2 - (v[1] * f) / v[2],
        depth: v[2]
      };
    }
  }

  MR.V3 = V3;
  MR.M4 = M4;
  MR.OrbitCamera = OrbitCamera;
  global.MR = MR;
})(typeof window !== 'undefined' ? window : globalThis);
