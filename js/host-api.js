// ═══════════════════════════════════════════════════════════════════
// RobSimul Simple – Host-Schnittstelle
// Steuerung aus einer Host-Software (iframe gleiche/fremde Domain, WebView2, CefSharp)
//   direkt:      RobSim.cmd('view','oben')        (gleiche Domain / ExecuteScriptAsync)
//   postMessage: {robsim:1, id, cmd:'view', args:['oben']}  → Antwort {robsim:1, id, ok, result|error}
//   Ereignisse:  {robsim:1, event:'ready'|'state'|'parsed', data}
// Beschreibung: INTEGRATION.md, Abschnitt 4a
// ═══════════════════════════════════════════════════════════════════
(function () {
  'use strict';
  var API_VERSION = 1;
  var listeners = {};

  var VIEW_ALIAS = { front: 'vorne', back: 'hinten', left: 'links', right: 'rechts', top: 'oben', bottom: 'unten',
    isometric: 'iso', isometrisch: 'iso' };
  var PATH_ALIAS = { on: 'all', alle: 'all', full: 'all', bis: 'upto', until: 'upto', off: 'none', aus: 'none' };
  var PROJ_ALIAS = { perspective: 'persp', perspektive: 'persp', orthographic: 'ortho', orthografisch: 'ortho', ortho: 'ortho', persp: 'persp' };
  var REND_ALIAS = { shaded: 'shaded', schattiert: 'shaded', wireframe: 'wireframe', drahtmodell: 'wireframe', draht: 'wireframe', wire: 'wireframe' };

  function lc(v) { return String(v == null ? '' : v).trim().toLowerCase(); }
  function bool(v, cur) {
    var s = lc(v);
    if (s === 'toggle' || s === '') return !cur;
    return !(s === 'off' || s === 'aus' || s === 'false' || s === '0' || s === 'hide');
  }
  function aplotVisible() { var p = document.getElementById('aplot-panel'); return !!p && getComputedStyle(p).display !== 'none'; }
  function uiVisible(part) {
    switch (part) {
      case 'header': return !document.body.classList.contains('rs-hide-header');
      case 'toolbar': return !document.body.classList.contains('rs-hide-toolbar');
      case 'editor': return !document.body.classList.contains('rs-hide-editor');
      case 'panel': return !document.body.classList.contains('rs-hide-panel');
      case 'sim': case 'base': case 'tcp': { var b = document.getElementById('sec-' + part); return !!b && b.style.display !== 'none'; }
    }
    return false;
  }
  function setUI(part, on) {
    switch (part) {
      case 'header': document.body.classList.toggle('rs-hide-header', !on); break;
      case 'toolbar': document.body.classList.toggle('rs-hide-toolbar', !on); break;
      case 'editor':   // ganz ausblenden inkl. Umschalter und Ziehgriff
        document.body.classList.toggle('rs-hide-editor', !on);
        if (on) { var ep = document.querySelector('.ep'); if (ep && ep._panelClosed) toggleLeftPanel(); }
        break;
      case 'panel':
        document.body.classList.toggle('rs-hide-panel', !on);
        if (on) toggleRightPanel(false);
        break;
      case 'sim': case 'base': case 'tcp': {   // Abschnitt im rechten Bereich ganz ausblenden
        var b = document.getElementById('sec-' + part); if (!b) break;
        b.style.display = on ? '' : 'none'; if (b.previousElementSibling) b.previousElementSibling.style.display = on ? '' : 'none';
        break;
      }
      default: throw new Error('unbekannter Bereich: ' + part);
    }
    window.dispatchEvent(new Event('resize'));
  }


  // ── Hilfen Editor / Simulation / BASE / TCP ─────────────────────
  var FR = ['x', 'y', 'z', 'a', 'b', 'c'];
  function frame(src, def) {
    var o = {}; src = src || {};
    FR.forEach(function (k) { var v = src[k] !== undefined ? src[k] : src[k.toUpperCase()]; o[k] = v !== undefined ? +v || 0 : (def ? def[k] || 0 : 0); });
    o.name = src.name != null ? String(src.name) : (def && def.name) || '';
    return o;
  }
  function copyList(l) { return l.map(function (e) { return frame(e); }); }
  function idx(n, len, allowNew) {   // Nummer 1…len (BASE_DATA[n], TOOL_DATA[n]) → Index
    var i = Math.round(+n) - 1;
    if (!(i >= 0 && i < len + (allowNew ? 1 : 0))) throw new Error('ungültige Nummer: ' + n + ' (1…' + (len + (allowNew ? 1 : 0)) + ')');
    return i;
  }
  function needSteps() { if (!(parsedData.steps && parsedData.steps.length)) throw new Error('kein Programm geladen'); }
  function click(id) { var b = document.getElementById(id); if (b) b.click(); }
  function curLine() { return typeof _gutActive === 'number' && _gutActive >= 0 ? _gutActive + 1 : null; }
  function lineToT(line) {
    var L = parsedData.posOfLine; if (!L || !L.length) throw new Error('kein Programm geladen');
    var i = L[Math.max(0, Math.min(L.length - 1, Math.round(+line) - 1))];
    if (!(i >= 0)) throw new Error('Zeile ' + line + ' gehört zu keiner Position'); return i;
  }
  function plcRun() {
    return new Promise(function (res) {
      if (_progDirty()) throw new Error('Programm geändert – erst parsen');
      _plc.res = null; plcStart();
      var box = document.getElementById('plc-box');
      if (!box || !box.querySelector('#plc-bar')) return res({ found: false, message: box ? box.textContent.trim() : '' });
      var t0 = Date.now();
      (function poll() {
        if (_plc.res) return res({ found: true, result: JSON.parse(JSON.stringify(_plc.res)) });
        if (Date.now() - t0 > 300000 || !box.querySelector('#plc-bar')) return res({ found: false, message: box.textContent.trim() });
        setTimeout(poll, 100);
      })();
    });
  }

  function togglePlayDir(dir) {
    if (_progDirty()) throw new Error('Programm geändert – erst parsen');
    if (sim.playing && sim.dir === dir) return;
    if (_ikPending) { _playAfterParse = dir; return; }
    startPlay(dir);
  }
  function waitIK(timeoutMs) {
    return new Promise(function (res, rej) {
      var t0 = Date.now();
      (function poll() {
        if (!_ikPending) return res(getState());
        if (Date.now() - t0 > (timeoutMs || 120000)) return rej(new Error('IK-Zeitüberschreitung'));
        setTimeout(poll, 50);
      })();
    });
  }

  function getState() {
    return {
      api: API_VERSION,
      view: curView, projection: projMode, render: renderMode, path: pathMode, aplot: aplotVisible(),
      ui: { header: uiVisible('header'), toolbar: uiVisible('toolbar'), editor: uiVisible('editor'), panel: uiVisible('panel'),
            sim: uiVisible('sim'), base: uiVisible('base'), tcp: uiVisible('tcp') },
      format: progFmt, points: parsedData.positions.length, steps: (parsedData.steps || []).length,
      t: Math.round(sim.t * 1000) / 1000, step: sim.stepIdx, line: curLine(),
      playing: sim.playing, dir: sim.dir, speed: +document.getElementById('spd-s').value, realtime: !!_realtimeMode,
      status: (document.getElementById('sim-status') || {}).textContent || '',
      dirty: _progDirty(), ikPending: !!_ikPending,
      base: { active: baseNavIdx + 1, count: baseList.length }, tcp: { active: tcpNavIdx + 1, count: tcpList.length }
    };
  }

  // Befehlstabelle: Name → Funktion(args…). Rückgabe = Ergebnis (oder Promise)
  var commands = {
    view: function (v) { var n = VIEW_ALIAS[lc(v)] || lc(v); if (!VIEWS[n]) throw new Error('unbekannte Ansicht: ' + v); setView(n); },
    projection: function (v) {
      var n = lc(v) === 'toggle' ? (projMode === 'persp' ? 'ortho' : 'persp') : PROJ_ALIAS[lc(v)];
      if (!n) throw new Error('unbekannte Projektion: ' + v); setProjection(n);
    },
    render: function (v) {
      var n = lc(v) === 'toggle' ? (renderMode === 'shaded' ? 'wireframe' : 'shaded') : REND_ALIAS[lc(v)];
      if (!n) throw new Error('unbekannte Darstellung: ' + v); setRenderMode(n);
    },
    zoom: function (v) {
      var s = lc(v), f = s === 'in' || s === '+' ? 0.8 : s === 'out' || s === '-' ? 1.25 : parseFloat(v);
      if (!(f > 0)) throw new Error('ungültiger Zoom: ' + v); zoomBy(f);
    },
    fit: function () { fitView(); },
    path: function (v) { var n = PATH_ALIAS[lc(v)] || lc(v); if (!/^(all|upto|none)$/.test(n)) throw new Error('unbekannte G-Code-Anzeige: ' + v); setPathMode(n); },
    aplot: function (v) { setAPlot(bool(v, aplotVisible())); },
    ui: function (part, v) { part = lc(part); setUI(part, bool(v, uiVisible(part))); },
    loadProgram: function (text, name) {
      if (typeof text === 'object' && text) { loadProgramFiles(text, name); }
      else {
        if (name) { var f = detectProgramFormat(String(text), String(name)); if (f && f !== progFmt) setProgramFormat(f); }
        _codeEl.value = String(text == null ? '' : text); _codeEl.dispatchEvent(new Event('input')); parseAndLoad();
      }
      return waitIK().then(function (st) { emit('parsed', st); return st; });
    },
    // Editor
    getProgram: function () { return _codeEl.value; },
    setProgram: function (text, name) {     // Text setzen ohne Parsen (Simulation gesperrt bis parse)
      if (name) { var f = detectProgramFormat(String(text), String(name)); if (f && f !== progFmt) setProgramFormat(f); }
      _codeEl.value = String(text == null ? '' : text); _codeEl.dispatchEvent(new Event('input'));
    },
    parse: function () { parseAndLoad(); return waitIK().then(function (st) { emit('parsed', st); return st; }); },
    setFormat: function (f) { f = lc(f); if (!/^(kuka|abb|fanuc|apt)$/.test(f)) throw new Error('unbekanntes Format: ' + f); setProgramFormat(f); },
    gotoLine: function (line) {             // Programmzeile (1-basiert) → Roboter an diese Position
      if (_progDirty() || _ikPending) throw new Error('Programm nicht geparst');
      var i = lineToT(line); pauseSim(); applySimT(i);
      var si = parsedData.stepOfPos ? parsedData.stepOfPos[i] : -1; if (si >= 0) applyStep(si);
    },
    getLine: function () { return curLine(); },
    // Simulation
    play: function (dir) { togglePlayDir(lc(dir) === 'back' || dir === -1 ? -1 : 1); },
    pause: function () { pauseSim(); setStatus('paused', 'PAUSED'); },
    stop: function () { pauseSim(); setStatus('paused', 'PAUSED'); },
    start: function () { click('b-start'); },
    end: function () { click('b-end'); },
    step: function (dir) { needSteps(); click(lc(dir) === 'back' || dir === -1 ? 'b-steprev' : 'b-stepfwd'); },
    seek: function (t) { pauseSim(); applySimT(Math.max(0, Math.min(Math.max(0, parsedData.positions.length - 1), +t || 0))); },
    seekPercent: function (p) { var N = parsedData.positions.length; pauseSim(); applySimT(Math.max(0, Math.min(1, (+p || 0) / 100)) * Math.max(0, N - 1)); },
    speed: function (v) {
      var n = Math.max(1, Math.min(100, Math.round(+v))); if (!(n > 0)) throw new Error('ungültiges Tempo: ' + v);
      var el = document.getElementById('spd-s'); el.value = n; el.dispatchEvent(new Event('input'));
    },
    realtime: function (v) { var on = bool(v, !!_realtimeMode); if (on !== !!_realtimeMode) toggleRealtime(); },
    // BASE (Nummer n = BASE_DATA[n], 1-basiert)
    getBases: function () { return { active: baseNavIdx + 1, list: copyList(baseList) }; },
    setBase: function (n, data) {
      var i = idx(n, baseList.length, true);
      if (i === baseList.length) baseList.push(frame(data, { name: 'BASE ' + (i + 1) }));
      else baseList[i] = frame(data, baseList[i]);
      baseNavTo(i); applyBase(); return { active: i + 1, list: copyList(baseList) };
    },
    setBases: function (list) {
      if (!Array.isArray(list) || !list.length) throw new Error('Liste erwartet');
      baseList.length = 0; list.forEach(function (e, i) { baseList.push(frame(e, { name: 'BASE ' + (i + 1) })); });
      baseNavTo(Math.min(baseNavIdx, baseList.length - 1)); applyBase(); return { active: baseNavIdx + 1, list: copyList(baseList) };
    },
    selectBase: function (n) { baseNavTo(idx(n, baseList.length)); },
    deleteBase: function (n) { baseNavTo(idx(n, baseList.length)); baseDelCurrent(); return { active: baseNavIdx + 1, list: copyList(baseList) }; },
    optimizeBase: function () { return plcRun(); },
    applyOptimizedBase: function () { if (!_plc.res) throw new Error('kein Vorschlag vorhanden'); plcApply(); return waitIK(); },
    undoOptimizedBase: function () { if (!_plc.undo) throw new Error('nichts zurückzunehmen'); plcUndo(); return waitIK(); },
    // TCP (Nummer n = TOOL_DATA[n], bezogen auf den Flansch)
    getTools: function () {
      var pt = parsedData.tools || {}, prog = {};
      Object.keys(pt).forEach(function (k) { prog[k] = frame(pt[k]); });
      return { active: tcpNavIdx + 1, list: copyList(tcpList), program: prog };
    },
    setTool: function (n, data) {
      var i = idx(n, tcpList.length, true);
      if (i === tcpList.length) tcpList.push(frame(data, { name: 'TCP ' + (i + 1) }));
      else tcpList[i] = frame(data, tcpList[i]);
      tcpNavTo(i); applyTCP(); return { active: i + 1, list: copyList(tcpList) };
    },
    setTools: function (list) {
      if (!Array.isArray(list) || !list.length) throw new Error('Liste erwartet');
      tcpList.length = 0; list.forEach(function (e, i) { tcpList.push(frame(e, { name: 'TCP ' + (i + 1) })); });
      tcpNavTo(Math.min(tcpNavIdx, tcpList.length - 1)); applyTCP(); return { active: tcpNavIdx + 1, list: copyList(tcpList) };
    },
    selectTool: function (n) { tcpNavTo(idx(n, tcpList.length)); },
    deleteTool: function (n) {
      if (tcpList.length <= 1) throw new Error('mindestens ein TCP erforderlich');
      tcpNavTo(idx(n, tcpList.length)); tcpDelCurrent(); return { active: tcpNavIdx + 1, list: copyList(tcpList) };
    },
    getState: function () { return getState(); }
  };

  function cmd(name, a, b) {
    var fn = commands[name];
    if (!fn) throw new Error('unbekannter Befehl: ' + name);
    var r = fn(a, b);
    return r === undefined ? getState() : r;
  }

  // ── Ereignisse ──────────────────────────────────────────────────
  function post(msg) {
    try { if (window.parent && window.parent !== window) window.parent.postMessage(msg, '*'); } catch (e) {}
    try { if (window.chrome && window.chrome.webview) window.chrome.webview.postMessage(msg); } catch (e) {}
  }
  function emit(event, data) {
    (listeners[event] || []).forEach(function (cb) { try { cb(data); } catch (e) { console.error(e); } });
    post({ robsim: API_VERSION, event: event, data: data });
  }

  // ── Eingehende Nachrichten (iframe / WebView2) ─────────────────
  function handle(msg, reply) {
    if (!msg || typeof msg !== 'object' || !msg.robsim || !msg.cmd) return;
    var args = Array.isArray(msg.args) ? msg.args : (msg.value !== undefined ? [msg.value] : []);
    Promise.resolve().then(function () { return cmd(msg.cmd, args[0], args[1]); })
      .then(function (r) { reply({ robsim: API_VERSION, id: msg.id, ok: true, result: r }); },
            function (e) { reply({ robsim: API_VERSION, id: msg.id, ok: false, error: String(e && e.message || e) }); });
  }
  window.addEventListener('message', function (e) {
    if (e.source !== window.parent || window.parent === window) return;  // nur vom einbettenden Fenster
    var msg = e.data; if (typeof msg === 'string') { try { msg = JSON.parse(msg); } catch (x) { return; } }
    handle(msg, function (r) { e.source.postMessage(r, e.origin === 'null' ? '*' : e.origin); });
  });
  if (window.chrome && window.chrome.webview) {
    window.chrome.webview.addEventListener('message', function (e) {
      var msg = e.data; if (typeof msg === 'string') { try { msg = JSON.parse(msg); } catch (x) { return; } }
      handle(msg, function (r) { window.chrome.webview.postMessage(r); });
    });
  }

  // ── Einbettungsmodus per URL: ?embed=1 (Kopfzeile+Toolbar aus), ?hide=header,toolbar,editor,panel
  //    ?embed=1 = nur 3D-Fenster (Kopfzeile, Toolbar, Editor, rechter Bereich aus); ?show=editor,panel blendet wieder ein
  //    Startwerte: ?view=oben&projection=ortho&render=wireframe&path=upto&aplot=1
  function applyUrl() {
    var q = new URLSearchParams(location.search), hide = [];
    if (q.get('embed') === '1') hide.push('header', 'toolbar', 'editor', 'panel');   // nur 3D-Fenster
    (q.get('hide') || '').split(',').forEach(function (h) { h = lc(h); if (h) hide.push(h); });
    var show = (q.get('show') || '').split(',').map(lc);
    hide.filter(function (h) { return show.indexOf(h) < 0; }).forEach(function (h) { try { setUI(h, false); } catch (e) {} });
    ['projection', 'render', 'path', 'view'].forEach(function (k) { if (q.has(k)) { try { cmd(k, q.get(k)); } catch (e) { console.warn(e.message); } } });
    if (q.has('aplot')) { try { cmd('aplot', q.get('aplot')); } catch (e) {} }
  }

  var lastState = '';
  window.RobSim = {
    version: API_VERSION,
    cmd: cmd,
    commands: Object.keys(commands),
    getState: getState,
    on: function (ev, cb) { (listeners[ev] = listeners[ev] || []).push(cb); },
    off: function (ev, cb) { listeners[ev] = (listeners[ev] || []).filter(function (f) { return f !== cb; }); },
    _emitState: function () { var st = getState(), s = JSON.stringify(st); if (s === lastState) return; lastState = s; emit('state', st); }
  };

  // Änderungen, die nicht über Befehle laufen (Maus, Modul-Bedienung, Simulation, Map „Übernehmen“), melden
  var lastProg = null, lastLine = null;
  setInterval(function () {
    if (!applyUrl.done) return;
    var code = _codeEl.value;
    if (code !== lastProg) { if (lastProg !== null) emit('program', { text: code, dirty: _progDirty() }); lastProg = code; }
    var ln = curLine(); if (ln !== lastLine) { lastLine = ln; emit('line', { line: ln, t: sim.t, step: sim.stepIdx }); }
    window.RobSim._emitState();
  }, 200);

  window.addEventListener('load', function () {
    applyUrl();
    // nach setView('iso') im Lade-Handler des Simulators
    setTimeout(function () { applyUrl.done = true; emit('ready', getState()); }, 0);
  });
})();
