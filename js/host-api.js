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
      case 'editor': { var ep = document.querySelector('.ep'); return !!ep && !ep._panelClosed; }
      case 'panel': { var sp = document.querySelector('.sim-panel'); return !!sp && !sp.classList.contains('closed'); }
    }
    return false;
  }
  function setUI(part, on) {
    switch (part) {
      case 'header': document.body.classList.toggle('rs-hide-header', !on); break;
      case 'toolbar': document.body.classList.toggle('rs-hide-toolbar', !on); break;
      case 'editor': if (uiVisible('editor') !== on) toggleLeftPanel(); break;
      case 'panel': toggleRightPanel(!on); break;
      default: throw new Error('unbekannter Bereich: ' + part);
    }
    window.dispatchEvent(new Event('resize'));
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
      ui: { header: uiVisible('header'), toolbar: uiVisible('toolbar'), editor: uiVisible('editor'), panel: uiVisible('panel') },
      format: progFmt, points: parsedData.positions.length, t: sim.t, playing: sim.playing, ikPending: !!_ikPending
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
    getProgram: function () { return _codeEl.value; },
    play: function (dir) { startPlay(lc(dir) === 'back' || dir === -1 ? -1 : 1); },
    pause: function () { pauseSim(); },
    seek: function (t) { pauseSim(); applySimT(Math.max(0, Math.min(Math.max(0, parsedData.positions.length - 1), +t || 0))); },
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
  //    Startwerte: ?view=oben&projection=ortho&render=wireframe&path=upto&aplot=1
  function applyUrl() {
    var q = new URLSearchParams(location.search), hide = [];
    if (q.get('embed') === '1') hide.push('header', 'toolbar');
    (q.get('hide') || '').split(',').forEach(function (h) { h = lc(h); if (h) hide.push(h); });
    hide.forEach(function (h) { try { setUI(h, false); } catch (e) {} });
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

  window.addEventListener('load', function () {
    applyUrl();
    // nach setView('iso') im Lade-Handler des Simulators
    setTimeout(function () { applyUrl.done = true; emit('ready', getState()); }, 0);
  });
})();
