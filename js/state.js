/* state.js — application state, persistence and animation clock. */
(function (global) {
  'use strict';

  const MR = global.MR;
  const STORAGE_KEY = 'mr_rig_demo_v1';

  const state = {
    rig: MR.Rig.buildMechCharacter(),
    selectedId: null,
    time: 0,
    duration: 8,          // timeline window in seconds
    playing: true,
    speed: 1,
    loop: true,
    lastWall: null,
    showGrid: true,
    showAllAxes: false,
    showLabels: true,
    showTrail: true,
    autoSave: true,
    // closure solver: warm-start seed from the previous accepted frame and
    // frames that could not be assembled (exported for reproduction)
    solverSeed: null,
    failedFrames: [],
    subSteps: 4
  };

  function serialize() {
    return JSON.stringify({
      format: 'mr-rig',
      version: 1,
      savedAt: new Date().toISOString(),
      rig: state.rig,
      time: state.time,
      duration: state.duration,
      speed: state.speed,
      loop: state.loop,
      subSteps: state.subSteps,
      failedFrames: state.failedFrames,
      view: {
        showGrid: state.showGrid,
        showAllAxes: state.showAllAxes,
        showLabels: state.showLabels,
        showTrail: state.showTrail
      }
    }, null, 2);
  }

  function restoreFromObject(data) {
    if (!data || data.format !== 'mr-rig' || !data.rig || !Array.isArray(data.rig.nodes)) {
      throw new Error('文件格式无效：缺少 mr-rig 标识或节点数组');
    }
    state.rig = data.rig;
    state.time = typeof data.time === 'number' ? data.time : 0;
    state.duration = data.duration || 8;
    state.speed = data.speed || 1;
    state.loop = data.loop !== false;
    state.subSteps = data.subSteps || 4;
    state.failedFrames = Array.isArray(data.failedFrames) ? data.failedFrames : [];
    state.solverSeed = null;
    if (data.view) Object.assign(state, data.view);
    const exists = state.rig.nodes.some((n) => n.id === state.selectedId);
    if (!exists) state.selectedId = state.rig.nodes[0]?.id ?? null;
  }

  const Storage = {
    save() {
      try {
        localStorage.setItem(STORAGE_KEY, serialize());
        return true;
      } catch (e) {
        console.warn('自动保存失败', e);
        return false;
      }
    },
    load() {
      const raw = localStorage.getItem(STORAGE_KEY);
      if (!raw) return false;
      try {
        restoreFromObject(JSON.parse(raw));
        return true;
      } catch (e) {
        console.warn('存档损坏，已忽略', e);
        return false;
      }
    },
    clear() {
      localStorage.removeItem(STORAGE_KEY);
    },
    exportFile() {
      const blob = new Blob([serialize()], { type: 'application/json' });
      const url = URL.createObjectURL(blob);
      const a = document.createElement('a');
      a.href = url;
      a.download = `${state.rig.name || 'rig'}-${Date.now()}.json`;
      a.click();
      URL.revokeObjectURL(url);
    },
    importFile(file) {
      return new Promise((resolve, reject) => {
        const reader = new FileReader();
        reader.onload = () => {
          try {
            restoreFromObject(JSON.parse(reader.result));
            resolve();
          } catch (e) {
            reject(e);
          }
        };
        reader.onerror = () => reject(reader.error);
        reader.readAsText(file);
      });
    },
    restoreFromObject
  };

  MR.State = { state, Storage, serialize, STORAGE_KEY };
})(typeof window !== 'undefined' ? window : globalThis);
