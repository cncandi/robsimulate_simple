// ═══════════════════════════════════════════════════════════════════════
// Teilposition optimieren
// Bewertet die Bahn bei verschobener/gedrehter BASE (X, Y, Drehung um Z = BASE-A, Z bleibt):
// je Stützpunkt die beste Stellung über alle IK-Konfigurationen und A-Werte (wie die Robot Map).
// Strafpunkte: unerreichbar 8, Achsgrenze 5, Kollision 3, Singularität 1, nahe Achsgrenze 0,3.
// Ergebnis: Ursachen-Diagnose, beste Lage als Vorschlag, Übernehmen / Rückgängig.
// ═══════════════════════════════════════════════════════════════════════
var _plc = { run: 0, res: null, undo: null };
var PLC_PEN = { reach: 8, limit: 5, coll: 3, elbow: 1, shoulder: 1, wrist: 1 };

function _plcTargetBase() {
  var cnt = {}, P = parsedData.positions || [];
  P.forEach(function(p) { if (p.rel && isFinite(p.rel.X) && p.baseN >= 1) cnt[p.baseN] = (cnt[p.baseN] || 0) + 1; });
  var best = 0, n = 0; for (var k in cnt) if (cnt[k] > n) { n = cnt[k]; best = +k; }
  return best;
}
function _plcSamples(idx, k) {
  if (idx.length <= k) return idx.slice();
  var out = []; for (var s = 0; s < k; s++) out.push(idx[Math.round(s * (idx.length - 1) / (k - 1))]);
  return out;
}
// Kandidat: Teil um (dx,dy) verschieben und um dth (°) um seinen Mittelpunkt (Welt-Z) drehen
function _plcBaseMat(B0, cx, cy, dx, dy, dth) {
  return _mul4(_poseT({ X: cx + dx, Y: cy + dy, Z: 0, A: dth, B: 0, C: 0 }), _mul4(_poseT({ X: -cx, Y: -cy, Z: 0, A: 0, B: 0, C: 0 }), B0));
}
function _plcApplyM(M, r) { return [M[0]*r.X + M[1]*r.Y + M[2]*r.Z + M[3], M[4]*r.X + M[5]*r.Y + M[6]*r.Z + M[7], M[8]*r.X + M[9]*r.Y + M[10]*r.Z + M[11]]; }

// Kollision Glieder/Werkzeug gegen Box b (6 Werte), wie _aCollides
function _plcColl(pts, b, R) {
  function hit(P, r) {
    var dx = Math.max(b[0] - P[0], 0, P[0] - b[3]), dy = Math.max(b[1] - P[1], 0, P[1] - b[4]), dz = Math.max(b[2] - P[2], 0, P[2] - b[5]);
    return dx*dx + dy*dy + dz*dz < r*r;
  }
  for (var k = 0; k < pts.length - 1; k++) {
    var A = pts[k], E = pts[k+1], tool = k === pts.length - 2;
    var L = Math.hypot(E[0]-A[0], E[1]-A[1], E[2]-A[2]); if (L < 1e-6) continue;
    var r = tool ? 20 : R, end = tool ? Math.max(0, L - 40) : L, n = Math.max(1, Math.ceil(end / 20));
    for (var s = 0; s <= n; s++) {
      var f = (end * s / n) / L;
      if (hit([A[0] + (E[0]-A[0])*f, A[1] + (E[1]-A[1])*f, A[2] + (E[2]-A[2])*f], r)) return true;
    }
  }
  return false;
}

// Bewertung einer Stellung q (in Grenzen): {pen, cat}
function _plcRateQ(q, box, ctx) {
  var fk = fkAll(q), pts = fk.pts, pen = 0, cat = '';
  if (_plcColl(pts, box, ctx.R)) { pen += PLC_PEN.coll; cat = 'coll'; }
  if (Math.abs(Math.sin(q[4] * Math.PI / 180)) < 0.14) { pen += 1; cat = cat || 'wrist'; }
  if (Math.hypot(pts[4][0], pts[4][1]) < 60) { pen += 1; cat = cat || 'shoulder'; }
  var dS = Math.hypot(pts[4][0]-pts[1][0], pts[4][1]-pts[1][1], pts[4][2]-pts[1][2]);
  if (ctx.armMax - dS < ctx.warn) { pen += 1; cat = cat || 'elbow'; }
  else if (ctx.armMax - dS < 3 * ctx.warn) pen += 0.1;
  for (var j = 0; j < 6; j++) { var lim = JOINTS_DEF[j]; if (Math.min(q[j] - lim.min, lim.max - q[j]) < 5) { pen += 0.3; break; } }
  return { pen: pen, cat: cat, q: q };
}
// Beste Stellung einer Pose über alle Konfigurationen
function _plcRatePose(x, y, z, a, b, c, box, ctx) {
  var best = { pen: PLC_PEN.reach, cat: 'reach', axis: -1 }, limAxis = -1, limCnt = 0;
  var sols;
  if (ctx.analytic) sols = ikAnalyticAll(x, y, z, a, b, c);
  else { var r = solveIKPrecise(x, y, z, a, b, c, ctx.prevQ); sols = (r && r.score < 20) ? [r.angles] : []; }
  for (var s = 0; s < sols.length; s++) {
    var sol = sols[s], q = [], ok = true, badAx = -1;
    for (var i = 0; i < 6; i++) {
      var lo = JOINTS_DEF[i].min, hi = JOINTS_DEF[i].max, v = sol[i], alt = null;
      for (var k = -2; k <= 2; k++) { var w = sol[i] + 360 * k; if (w >= lo && w <= hi && (alt === null || Math.abs(w) < Math.abs(alt))) alt = w; }
      if (alt === null) { ok = false; badAx = i; break; }
      q.push(alt);
    }
    if (!ok) { limCnt++; if (limAxis < 0) limAxis = badAx; continue; }
    var rt = _plcRateQ(q, box, ctx);
    if (rt.pen < best.pen) best = rt;
    if (best.pen === 0) break;
  }
  if (best.cat === 'reach' && limCnt) best = { pen: PLC_PEN.limit, cat: 'limit', axis: limAxis };
  return best;
}

// Konfiguration (vorne/über Kopf, Ellbogen oben/unten, Hand normal/Flip): ein Wechsel = Achssprung
function _plcConfSig(q, ctx) {
  var pts = fkAll(q).pts, c1 = Math.cos(q[0] * Math.PI / 180), s1 = Math.sin(q[0] * Math.PI / 180);
  var front = (pts[4][0] * c1 + pts[4][1] * s1) >= 0 ? 'v' : 'k';
  var el = (q[2] - ctx.th3s) * FK_SIGNS[2] >= 0 ? 'o' : 'u';
  var w = Math.sin(q[4] * Math.PI / 180) >= 0 ? 'n' : 'f';
  return front + el + w;
}
function _plcTh3s() {
  var J = JOINTS_DEF, o2 = J[1].off, o3 = J[2].off, ux = o3[0] + J[3].off[0], uz = o3[2];
  return Math.atan2(o2[0]*uz - o2[2]*ux, o2[0]*ux + o2[2]*uz) * 180 / Math.PI * FK_SIGNS[2];
}
// Kontext: Positionen, Zielbase, Stützpunkte
function _plcContext() {
  var P = parsedData.positions || [], bn = _plcTargetBase();
  if (!bn || !baseList[bn - 1]) return null;
  var idx = [];
  P.forEach(function(p, i) { if (p.type !== 'CIRC_AUX' && p.rel && isFinite(p.rel.X) && p.baseN === bn) idx.push(i); });
  if (!idx.length) return null;
  var B0 = _baseMat(baseList[bn - 1]), cx = 0, cy = 0;
  idx.forEach(function(i) { var w = _plcApplyM(B0, P[i].rel); cx += w[0]; cy += w[1]; });
  cx /= idx.length; cy /= idx.length;
  var relT = P.map(function(p) { return (p.rel && isFinite(p.rel.X)) ? _poseT(p.rel) : null; });
  return { P: P, bn: bn, idx: idx, B0: B0, cx: cx, cy: cy, relT: relT, R: _aCollRadius(), armMax: _armMaxReach(), warn: _elbowWarnMm(),
           analytic: ikAnalyticSupported(), th3s: _plcTh3s(), prevQ: jointAngles.slice(), tool: (P[idx[0]].toolN || 1) };
}

// Kandidat bewerten. samples: Satzindizes; nA: Anzahl A-Stufen (zusätzlich zum Programm-A)
function _plcEval(ctx, M, samples, nA, detail) {
  var P = ctx.P, N = P.length, bx = [Infinity, Infinity, Infinity, -Infinity, -Infinity, -Infinity];
  var boxAt = {}, need = {}; samples.forEach(function(i) { need[i] = 1; });
  for (var i = 0; i < N; i++) {
    var p = P[i], w;
    if (p.baseN === ctx.bn && ctx.relT[i]) w = _plcApplyM(M, p.rel); else if (isFinite(p.X)) w = [p.X, p.Y, p.Z]; else w = null;
    if (w) { bx[0] = Math.min(bx[0], w[0]); bx[1] = Math.min(bx[1], w[1]); bx[2] = Math.min(bx[2], w[2]);
             bx[3] = Math.max(bx[3], w[0]); bx[4] = Math.max(bx[4], w[1]); bx[5] = Math.max(bx[5], w[2]); }
    if (need[i]) boxAt[i] = bx.slice();
  }
  var tot = 0, totP = 0, bad = 0, badProg = 0, jumps = 0, cats = {}, axes = {}, prevQ = ctx.prevQ, prevSig = '';
  for (var s = 0; s < samples.length; s++) {
    var k = samples[s], W = _fromT(_mul4(M, ctx.relT[k])), best = null;
    // Programm wie programmiert (A unverändert), Konfiguration stetig wie in der Simulation
    if (ctx.analytic) {
      var an = solveIKAnalytic(W.X, W.Y, W.Z, W.A, W.B, W.C, prevQ), pp;
      if (!an) pp = PLC_PEN.reach;
      else if (!an.ok) pp = PLC_PEN.limit;
      else {
        var sg = _plcConfSig(an.angles, ctx);
        pp = _plcRateQ(_aNormQ(an.angles), boxAt[k], ctx).pen;
        if (s > 0 && sg !== prevSig) { pp += PLC_PEN.coll; jumps++; }
        prevSig = sg;
      }
      if (an) prevQ = an.angles;
      totP += pp; if (pp >= 1) badProg++;
    }
    var pr = _plcRatePose(W.X, W.Y, W.Z, W.A, W.B, W.C, boxAt[k], ctx);
    best = pr; if (!ctx.analytic && pr.pen >= 1) badProg++;
    for (var a = 0; a < nA && best.pen > 0; a++) {
      var r = _plcRatePose(W.X, W.Y, W.Z, W.A + (a + 1) * 360 / (nA + 1), W.B, W.C, boxAt[k], ctx);
      if (r.pen < best.pen) best = r;
    }
    tot += best.pen;
    if (best.pen >= 1) { bad++; cats[best.cat] = (cats[best.cat] || 0) + 1; if (best.cat === 'limit') axes[best.axis] = (axes[best.axis] || 0) + 1; }
  }
  var r0 = { cost: (tot + 0.5 * totP) / samples.length, bad: bad, badProg: badProg, jumps: jumps, n: samples.length };
  if (detail) { r0.cats = cats; r0.axes = axes; r0.box = bx; }
  return r0;
}

// Ursachen in Worten
function _plcDiagText(ev, ctx) {
  if (!ev.bad && !ev.badProg) return 'Teillage ist nicht die Ursache: alle geprüften Sätze sind fahrbar.';
  if (!ev.bad) return 'Mit angepasstem A (Robot Map) sind alle geprüften Sätze fahrbar. Wie programmiert haben ' + ev.badProg + ' Sätze Probleme' + (ev.jumps ? ' (' + ev.jumps + ' Achssprünge)' : '') + ' – eine andere Teillage kann das beheben.';
  var c = ev.cats, n = ev.bad, sh = function(k) { return (c[k] || 0) / n; };
  var dC = Math.hypot(ctx.cx, ctx.cy), out = [];
  var far = sh('reach') + sh('elbow'), near = sh('shoulder') + sh('coll');
  if (far >= 0.5) out.push('Teil liegt zu weit vom Roboter entfernt (Arm gestreckt oder außer Reichweite)');
  else if (near >= 0.5) out.push(sh('coll') >= sh('shoulder') ? 'Roboter kollidiert mit dem Teil – Teil liegt zu nah oder ungünstig zum Roboter' : 'Teil liegt zu nah an der A1-Achse');
  if (sh('limit') >= 0.3) {
    var ax = Object.keys(ev.axes).sort(function(a, b) { return ev.axes[b] - ev.axes[a]; })[0];
    var where = dC < 0.5 * ctx.armMax ? 'Teil liegt zu nah am Roboter' : dC > 0.9 * ctx.armMax ? 'Teil liegt zu weit vom Roboter entfernt' : '';
    if (!out.length && where) out.push(where);
    out.push('Achsgrenze A' + (+ax + 1) + (+ax === 0 ? ' (Teil liegt seitlich außerhalb des Schwenkbereichs)' : ''));
  }
  if (sh('wrist') >= 0.5) out.push('Handgelenk-Singularität (A5 ≈ 0) – Orientierung/Lage ungünstig');
  if (!out.length) out.push('gemischte Ursachen');
  var lst = [['reach', 'unerreichbar'], ['limit', 'Achsgrenze'], ['coll', 'Kollision'], ['elbow', 'Arm gestreckt'], ['shoulder', 'Schulter'], ['wrist', 'Handgelenk']]
    .filter(function(e) { return c[e[0]]; }).map(function(e) { return e[1] + ' ' + c[e[0]]; }).join(', ');
  return 'Ursache: ' + out.join('; ') + '. Teilmitte ' + Math.round(dC) + ' mm von der A1-Achse, Armlänge Schulter–Hand ' + Math.round(ctx.armMax) + ' mm. (' + lst + ' von ' + ev.n + ' geprüften Sätzen)';
}

function _plcBox(html) { var el = document.getElementById('plc-box'); if (el) { el.style.display = ''; el.innerHTML = html; } }
function _plcEsc(s) { return String(s).replace(/[&<>]/g, function(c) { return { '&': '&amp;', '<': '&lt;', '>': '&gt;' }[c]; }); }

function plcStart() {
  if (typeof _progDirty === 'function' && _progDirty()) { _showParseNeeded(); return; }
  var sec = document.getElementById('sec-base'); if (sec && sec.classList.contains('collapsed')) toggleSection('base', false);
  var sp = document.querySelector('.sim-panel'); if (sp && sp.classList.contains('closed') && typeof toggleRightPanel === 'function') toggleRightPanel(false);
  var ctx = _plcContext();
  if (!ctx) { _plcBox('<div class="plc-msg">Keine BASE im Programm ($BASE=BASE_DATA[n]) – die Teillage ist über die BASE festgelegt und kann nur dann optimiert werden.</div>'); return; }
  var run = ++_plc.run, tcpSave = Object.assign({}, TCP_DEF);
  if (typeof _applyToolToTCPDEF === 'function') _applyToolToTCPDEF(ctx.tool);
  var S1 = _plcSamples(ctx.idx, 36), S2 = _plcSamples(ctx.idx, 90), SF = _plcSamples(ctx.idx, 400);
  var before = _plcEval(ctx, ctx.B0, SF, 11, true);
  var diag = _plcDiagText(before, ctx);
  // Grob: Teilmitte auf Raster um den Roboter, 4 Drehlagen
  var span = ctx.armMax, cands = [{ dx: 0, dy: 0, dth: 0 }], st = span / 5;
  for (var gx = -1.2 * span; gx <= 1.2 * span + 1; gx += st)
    for (var gy = -1.2 * span; gy <= 1.2 * span + 1; gy += st) {
      var rr = Math.hypot(gx, gy); if (rr < 0.2 * span || rr > 1.25 * span) continue;
      for (var t = 0; t < 4; t++) cands.push({ dx: gx - ctx.cx, dy: gy - ctx.cy, dth: t * 90 });
    }
  var scored = [], ci = 0, phase = 1, top = [], cur = null, step = 0, astep = 0, refineIdx = 0, iter = 0;
  var cost = function(c, S, nA) { var e = _plcEval(ctx, _plcBaseMat(ctx.B0, ctx.cx, ctx.cy, c.dx, c.dy, c.dth), S, nA); return e.cost + 0.02 * Math.hypot(c.dx, c.dy) / span + 0.005 * Math.abs(_plcWrap(c.dth)) / 180; };
  _plcBox('<div class="plc-msg">' + _plcEsc(diag) + '</div><div class="plc-prog"><div id="plc-bar"></div></div><div class="plc-msg" id="plc-st">Suche Teillage …</div>');
  if (!before.bad && !before.badProg) { Object.assign(TCP_DEF, tcpSave); _plcBox('<div class="plc-msg">' + _plcEsc(diag) + '</div>'); return; }
  function prog(f, m) { var b = document.getElementById('plc-bar'); if (b) b.style.width = Math.round(f * 100) + '%'; var s = document.getElementById('plc-st'); if (s && m) s.textContent = m; }
  function chunk() {
    if (run !== _plc.run) return;
    var t0 = performance.now();
    while (performance.now() - t0 < 40) {
      if (phase === 1) {
        if (ci >= cands.length) { scored.sort(function(a, b) { return a.f - b.f; }); top = scored.slice(0, 4).map(function(s) { return s.c; }); phase = 2; refineIdx = 0; cur = null; continue; }
        var c = cands[ci++]; scored.push({ c: c, f: cost(c, S1, 5) });
        prog(0.6 * ci / cands.length, 'Grobsuche ' + ci + ' / ' + cands.length);
      } else if (phase === 2) {
        if (!cur) {
          if (refineIdx >= top.length) { phase = 3; continue; }
          cur = { c: top[refineIdx], f: cost(top[refineIdx], S2, 11) }; step = span / 10; astep = 22.5;
        }
        var improved = false, moves = [[step, 0, 0], [-step, 0, 0], [0, step, 0], [0, -step, 0], [0, 0, astep], [0, 0, -astep]];
        for (var m = 0; m < moves.length; m++) {
          var nc = { dx: cur.c.dx + moves[m][0], dy: cur.c.dy + moves[m][1], dth: cur.c.dth + moves[m][2] };
          var nf = cost(nc, S2, 11); if (nf < cur.f - 1e-9) { cur = { c: nc, f: nf }; improved = true; }
        }
        if (!improved) { step /= 2; astep /= 2; }
        if (step < 5 && astep < 1) { (_plc.best = (!_plc.best || cur.f < _plc.best.f) ? cur : _plc.best); refineIdx++; cur = null; }
        prog(0.6 + 0.38 * refineIdx / top.length, 'Feinsuche ' + Math.min(top.length, refineIdx + 1) + ' / ' + top.length);
      } else {
        var b = _plc.best.c, M = _plcBaseMat(ctx.B0, ctx.cx, ctx.cy, b.dx, b.dy, b.dth);
        var after = _plcEval(ctx, M, SF, 11, true), nb = _fromT(M);
        Object.assign(TCP_DEF, tcpSave);
        _plc.res = { bn: ctx.bn, base: { x: nb.X, y: nb.Y, z: nb.Z, a: nb.A, b: nb.B, c: nb.C }, before: before, after: after, diag: diag, mv: b };
        _plcShow(); return;
      }
    }
    setTimeout(chunk, 0);
  }
  _plc.best = null;
  setTimeout(chunk, 0);
}
function _plcWrap(a) { return ((a % 360) + 540) % 360 - 180; }
function _plcR(v, d) { var f = Math.pow(10, d || 0); return Math.round(v * f) / f; }

function _plcShow() {
  var r = _plc.res, o = baseList[r.bn - 1], b = r.base, B = r.before, A = r.after;
  var row = function(k, l, u) { return '<tr><td>' + l + '</td><td>' + _plcR(o[k] || 0, 1) + '</td><td><b>' + _plcR(b[k], 1) + '</b></td><td>' + u + '</td></tr>'; };
  var gain = A.bad < B.bad || (A.bad === B.bad && A.badProg < B.badProg);
  var h = '<div class="plc-msg">' + _plcEsc(r.diag) + '</div>'
    + '<div class="plc-res"><b>Problemsätze ' + B.bad + ' → ' + A.bad + '</b> von ' + A.n + ' (mit optimiertem A)'
    + '<br>Programm unverändert (A wie programmiert): ' + B.badProg + ' → ' + A.badProg + (A.jumps || B.jumps ? ', davon Achssprünge ' + B.jumps + ' → ' + A.jumps : '')
    + '<br>Verschiebung ' + _plcR(Math.hypot(r.mv.dx, r.mv.dy)) + ' mm, Drehung um Z ' + _plcR(_plcWrap(r.mv.dth), 1) + '°</div>'
    + '<table class="plc-tab"><tr><th></th><th>alt</th><th>neu</th><th></th></tr>' + row('x', 'X', 'mm') + row('y', 'Y', 'mm') + row('a', 'A(Z)', '°') + '</table>';
  if (A.bad) h += '<div class="plc-msg">Verbleibende Probleme auch in der besten Lage – Ursache liegt nicht nur in der Teillage (Bahn, Werkzeugorientierung oder Roboter).</div>';
  if (A.badProg > A.bad) h += '<div class="plc-msg">Nach Übernehmen Robot Map erzeugen und Linie übernehmen: einige Sätze sind nur mit geändertem A fahrbar.</div>';
  h += '<div class="row" style="margin-top:6px">' + (gain ? '<button class="btn primary" onclick="plcApply()">Übernehmen</button>' : '<span class="plc-msg">Keine bessere Lage gefunden.</span>')
    + '<button class="btn" onclick="plcClose()">Verwerfen</button></div>';
  _plcBox(h);
}
function plcApply() {
  var r = _plc.res; if (!r) return;
  var e = baseList[r.bn - 1]; _plc.undo = { bn: r.bn, base: { x: e.x, y: e.y, z: e.z, a: e.a, b: e.b, c: e.c } };
  ['x', 'y', 'z', 'a', 'b', 'c'].forEach(function(k) { e[k] = _plcR(r.base[k], 3); });
  baseNavTo(r.bn - 1); syncBaseFrameGroups(); applyBase();
  _plcBox('<div class="plc-msg">BASE ' + r.bn + ' gesetzt: X ' + _plcR(e.x, 1) + ', Y ' + _plcR(e.y, 1) + ', A ' + _plcR(e.a, 1) + '°. Roboterstellung wird neu berechnet.</div>'
    + '<div class="row" style="margin-top:6px"><button class="btn" onclick="plcUndo()">Rückgängig</button><button class="btn" onclick="plcClose()">Schließen</button></div>');
}
function plcUndo() {
  var u = _plc.undo; if (!u) return;
  Object.assign(baseList[u.bn - 1], u.base); _plc.undo = null;
  baseNavTo(u.bn - 1); syncBaseFrameGroups(); applyBase(); plcClose();
}
function plcClose() { _plc.run++; var el = document.getElementById('plc-box'); if (el) { el.style.display = 'none'; el.innerHTML = ''; } }
