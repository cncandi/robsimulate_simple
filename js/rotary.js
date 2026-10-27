// ═══════════════════════════════════════════════════════════════════
// RobSimul Simple – Drehachse (externe Achse E1, mathematisch gekoppelt)
// KRL: $BASE=EK(MACHINE_DEF[2].ROOT,MACHINE_DEF[2].MECH_TYPE,BASE_DATA[n]) und LIN {X..,E1 ..}
// Die Positionen stehen im mitdrehenden Werkstück-Koordinatensystem (BASE n, Drehachse = X, Y oder Z
// dieses BASE). Welt = BASE · Rot(E1) · Position. Bahn und Rohling drehen mit E1 mit.
// Konfiguration: Befehl rotaryAxis (host-api.js) oder Kopfkommentar
//   ;P3D-ROTARY AXIS=X BASE=2 D0=60.3 L=500
// ═══════════════════════════════════════════════════════════════════
var rotAxis = { on: false, explicit: null, header: null, ek: false, axis: 'X', base: 0, d0: 60, d1: 60, L: 500, chuck: 'start', e1: null, placed: '' };
(function () {
  'use strict';
  var I4 = [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1];
  var workGrp = new THREE.Group(); workGrp.matrixAutoUpdate = false; workGrp.visible = false; scene.add(workGrp);
  pathGrp.matrixAutoUpdate = false; posGrp.matrixAutoUpdate = false;

  function axisRot(deg, axis) {
    var r = deg * Math.PI / 180, c = Math.cos(r), s = Math.sin(r);
    if (axis === 'Y') return [c,0,s,0, 0,1,0,0, -s,0,c,0, 0,0,0,1];
    if (axis === 'Z') return [c,-s,0,0, s,c,0,0, 0,0,1,0, 0,0,0,1];
    return [1,0,0,0, 0,c,-s,0, 0,s,c,0, 0,0,0,1];
  }
  function inv(M) { // starre Transformation
    var R = [M[0],M[4],M[8], M[1],M[5],M[9], M[2],M[6],M[10]], t = [M[3],M[7],M[11]];
    return [R[0],R[1],R[2],-(R[0]*t[0]+R[1]*t[1]+R[2]*t[2]),
            R[3],R[4],R[5],-(R[3]*t[0]+R[4]*t[1]+R[5]*t[2]),
            R[6],R[7],R[8],-(R[6]*t[0]+R[7]*t[1]+R[8]*t[2]), 0,0,0,1];
  }
  function baseM(n) { var b = _baseByN(n); return b ? _baseMat(b) : I4.slice(); }
  function setM(obj, M) { obj.matrix.set.apply(obj.matrix, M); obj.matrixWorldNeedsUpdate = true; }
  function num(v, d) { v = parseFloat(v); return isFinite(v) ? v : d; }
  function axisOf(v, d) { v = String(v || '').trim().toUpperCase(); return /^[XYZ]$/.test(v) ? v : d; }

  // Kopfkommentar ;P3D-ROTARY KEY=VALUE …
  window._rotHeader = function (code) {
    var m = String(code || '').match(/^\s*;\s*P3D-ROTARY\b(.*)$/im);
    if (!m) return null;
    var o = {}; m[1].replace(/([A-Z0-9_]+)\s*=\s*([^\s;]+)/gi, function (_, k, v) { o[k.toUpperCase()] = v; return ''; });
    return { chuck: /^(END|ENDE|RIGHT|RECHTS)$/i.test(o.CHUCK || '') ? 'end' : (o.CHUCK ? 'start' : undefined), axis: axisOf(o.AXIS, 'X'), base: o.BASE ? Math.round(num(o.BASE, 0)) : null, d0: num(o.D0, null), d1: num(o.D1, null), L: num(o.L, null) };
  }

  // Vor dem Einbrennen der BASE: Drehachse aktiv? Konfiguration zusammenführen, BASE-Lage vorbelegen
  window._rotPrepare = function (code) {
    var P = parsedData.positions, hasE1 = P.some(function (p) { return typeof p.E1 === 'number'; });
    rotAxis.header = window._rotHeader(code);
    rotAxis.ek = !!parsedData.ekBase;
    var ex = rotAxis.explicit;
    var want = ex && ex.on === false ? false : !!((ex && ex.on) || rotAxis.header || rotAxis.ek);
    rotAxis.on = want && hasE1;
    var src = Object.assign({}, rotAxis.header || {}, ex || {});
    rotAxis.axis = axisOf(src.axis, 'X');
    rotAxis.chuck = src.chuck === 'end' ? 'end' : 'start'; rotAxis.d0 = num(src.d0, 60); rotAxis.d1 = num(src.d1, rotAxis.d0); rotAxis.L = num(src.L, 500); // d1: Ø am Ende (konischer Rohling)
    var bn = 0; P.some(function (p) { if (typeof p.E1 === 'number' && p.baseN > 0) { bn = p.baseN; return true; } return false; });
    rotAxis.base = src.base || parsedData.ekBase || bn || 0;
    // E1 je Position (fehlende Werte = letzter bekannter Wert)
    var last = 0; rotAxis.e1 = new Float64Array(P.length);
    P.forEach(function (p, i) { if (typeof p.E1 === 'number') last = p.E1; rotAxis.e1[i] = last; });
    if (!rotAxis.on) { var e1el = document.getElementById('rot-e1'); if (e1el) e1el.hidden = true; workGrp.visible = false; setM(pathGrp, I4); setM(posGrp, I4); return; }
    // BASE noch nicht eingemessen (alles 0): Standardlage vor dem Roboter, damit Bahn erreichbar ist
    var n = rotAxis.base;
    if (n > 0) {
      while (baseList.length < n) baseList.push({ x:0, y:0, z:0, a:0, b:0, c:0, name: 'BASE ' + (baseList.length + 1) });
      var b = baseList[n - 1];
      if (['x','y','z','a','b','c'].every(function (k) { return !b[k]; })) {
        var L = rotAxis.L;
        var d = rotAxis.axis === 'Z' ? { x: 1100, y: 0, z: 200, a: 0, b: 0, c: 0 }
              : rotAxis.axis === 'Y' ? { x: 1000, y: -L / 2, z: 600, a: 0, b: 0, c: 0 }
              : { x: 1000, y: -L / 2, z: 600, a: 90, b: 0, c: 0 };
        Object.assign(b, d); if (!b.name || /^BASE \d+$/.test(b.name)) b.name = 'Drehachse';
        rotAxis.placed = 'BASE ' + n + ' war nicht eingemessen und wurde auf eine Standardlage vor dem Roboter gesetzt.';
        if (typeof renderBaseNav === 'function') renderBaseNav();
      } else rotAxis.placed = '';
    }
    buildWork();
  };

  // Einbrennen: Welt = BASE · Rot(E1) · Position; p.part = Lage bei E1 = 0 (für Bahnanzeige/Tempo)
  window._rotBake = function (p, w, bn) {
    if (!rotAxis.on || typeof p.E1 !== 'number') { delete p.part; return w; }
    p.part = { X: w.X, Y: w.Y, Z: w.Z, A: w.A, B: w.B, C: w.C };
    return _fromT(_mul4(_mul4(baseM(bn), axisRot(p.E1, rotAxis.axis)), _poseT(p.rel)));
  };

  function buildWork() {
    while (workGrp.children.length) { var c = workGrp.children.pop(); c.traverse(function (o) { if (o.geometry) o.geometry.dispose(); if (o.material) o.material.dispose(); }); }
    var r = Math.max(1, rotAxis.d0 / 2), r1 = Math.max(1, rotAxis.d1 / 2), L = Math.max(1, rotAxis.L), inner = new THREE.Group();
    var body = new THREE.Mesh(new THREE.CylinderGeometry(r1, r, L, 48, 1, false), new THREE.MeshStandardMaterial({ color: 0x9ea5ae, metalness: 0.8, roughness: 0.35 }));
    body.position.y = L / 2; inner.add(body);
    // Spannfutter am Rohlinganfang (u = 0) oder -ende (u = L)
    var cr = rotAxis.chuck === 'end' ? r1 : r, ch = Math.max(20, cr * 0.5);
    var chuck = new THREE.Mesh(new THREE.CylinderGeometry(cr * 1.9, cr * 1.9, ch, 48), new THREE.MeshStandardMaterial({ color: 0x4a5160, metalness: 0.6, roughness: 0.45 }));
    chuck.position.y = rotAxis.chuck === 'end' ? L + ch / 2 : -ch / 2; inner.add(chuck);
    for (var k = 0; k < 3; k++) {   // drei Spannbacken
      var a = k * Math.PI * 2 / 3, jaw = new THREE.Mesh(new THREE.BoxGeometry(cr * 0.45, ch * 0.6, cr * 0.7), new THREE.MeshStandardMaterial({ color: 0x343a44, metalness: 0.6, roughness: 0.5 }));
      jaw.position.set(Math.cos(a) * cr * 1.25, rotAxis.chuck === 'end' ? L - ch * 0.3 : ch * 0.3, Math.sin(a) * cr * 1.25); jaw.rotation.y = -a; inner.add(jaw);
    }
    // Zylinder liegt entlang Y → auf die Drehachse des BASE drehen
    if (rotAxis.axis === 'X') inner.rotation.z = -Math.PI / 2;
    else if (rotAxis.axis === 'Z') inner.rotation.x = Math.PI / 2;
    workGrp.add(inner); workGrp.visible = true;
  }

  function e1At(t) {
    var e = rotAxis.e1; if (!e || !e.length) return 0;
    var i = Math.max(0, Math.min(e.length - 1, Math.floor(t))), j = Math.min(e.length - 1, i + 1), f = Math.max(0, Math.min(1, t - i));
    return e[i] + (e[j] - e[i]) * f;
  }
  window._rotE1At = e1At;
  // Pro Bild: Bahn, Zielpunkte und Rohling auf den aktuellen E1-Wert drehen
  window._rotApply = function (t) {
    if (!rotAxis.on) return;
    var Bm = baseM(rotAxis.base), W = _mul4(Bm, axisRot(e1At(t), rotAxis.axis));
    setM(workGrp, W); var G = _mul4(W, inv(Bm)); setM(pathGrp, G); setM(posGrp, G);
    var el = document.getElementById('rot-e1'); if (el) { el.hidden = false; el.textContent = 'Drehachse E1 ' + e1At(t).toFixed(1) + '°'; }
  };
  window._rotState = function () {
    return { on: rotAxis.on, axis: rotAxis.axis, base: rotAxis.base, d0: rotAxis.d0, d1: rotAxis.d1, L: rotAxis.L, chuck: rotAxis.chuck, e1: rotAxis.on ? Math.round(e1At(sim.t) * 10) / 10 : null, note: rotAxis.placed };
  };
  window._rotPart = function (p) { return p.part || p; };
  // Kollision (Robot Map / Prüfung): gedrucktes Teil bei Satz i als Zylinder um die Drehachse,
  // Radius = Düsenradius dieses Satzes (mindestens Rohling), Länge = Rohling bzw. bisheriger Blattbereich
  var _hp = null, _hpPos = null;
  window._rotHitFn = function (pos, i) {
    if (!rotAxis.on) return null;
    var ax = rotAxis.axis, k = ax === 'X' ? 0 : ax === 'Y' ? 1 : 2;
    if (_hpPos !== pos || !_hp) {
      var N = pos.length, H = new Float64Array(N * 3), lo = 0, hi = rotAxis.L;
      for (var j = 0; j < N; j++) {
        var q = pos[j].rel;
        var rj = Math.min(rotAxis.d0, rotAxis.d1) / 2;
        if (q && typeof pos[j].E1 === 'number') { var v = [q.X, q.Y, q.Z]; rj = Math.max(rj, Math.hypot(v[(k + 1) % 3], v[(k + 2) % 3]) - 0.5); lo = Math.min(lo, v[k]); hi = Math.max(hi, v[k]); }
        H[j * 3] = rj;   // Teil reicht höchstens bis an die Düse (Lagen wachsen radial nach außen) H[j * 3 + 1] = lo; H[j * 3 + 2] = hi;
      }
      _hp = H; _hpPos = pos;
    }
    var R = _hp[i * 3], lo2 = _hp[i * 3 + 1], hi2 = _hp[i * 3 + 2], Bi = inv(baseM(rotAxis.base));
    return function (P, r) {
      var l = [Bi[0]*P[0]+Bi[1]*P[1]+Bi[2]*P[2]+Bi[3], Bi[4]*P[0]+Bi[5]*P[1]+Bi[6]*P[2]+Bi[7], Bi[8]*P[0]+Bi[9]*P[1]+Bi[10]*P[2]+Bi[11]];
      var da = Math.max(lo2 - l[k], 0, l[k] - hi2), dr = Math.max(0, Math.hypot(l[(k + 1) % 3], l[(k + 2) % 3]) - R);
      return da * da + dr * dr < r * r;
    };
  };
})();
