// ══════════════════════════════════════════════════════════════════════════
// Programmformate: KUKA KRL · ABB RAPID · FANUC TP (LS) · APT/CL-Data
// Jeder Leser liefert dieselbe Struktur wie parseKRL (positions/steps/finalState, mit lineNum,
// toolN, baseN, velCP, rel), damit Simulation, Editor-Kopplung und Robot Map unverändert arbeiten.
// Jeder Erzeuger schreibt ein Programm im jeweiligen Format aus geparsten Daten (für den Formatwechsel).
// Winkel A,B,C = KUKA-Konvention: R = Rz(A)·Ry(B)·Rx(C), Grad.
// ══════════════════════════════════════════════════════════════════════════
const PROG_FORMATS = {
  kuka:  { label: 'KUKA KRL',    ext: 'src', accept: ['.src', '.krl', '.dat'] },
  abb:   { label: 'ABB RAPID',   ext: 'mod', accept: ['.mod', '.prg', '.modx'] },
  fanuc: { label: 'FANUC TP',    ext: 'ls',  accept: ['.ls'] },
  apt:   { label: 'APT CL-Data', ext: 'apt', accept: ['.apt', '.cl', '.cls', '.aptsource', '.clt'] },
};

// ── Mathe: ABC ↔ Matrix ↔ Quaternion, Werkzeugachse ─────────────────────
const _FD = Math.PI / 180;
function _fmtR(A, B, C) {
  const ca = Math.cos(A*_FD), sa = Math.sin(A*_FD), cb = Math.cos(B*_FD), sb = Math.sin(B*_FD), cc = Math.cos(C*_FD), sc = Math.sin(C*_FD);
  return [[ca*cb, ca*sb*sc - sa*cc, ca*sb*cc + sa*sc],
          [sa*cb, sa*sb*sc + ca*cc, sa*sb*cc - ca*sc],
          [-sb,   cb*sc,            cb*cc]];
}
function _fmtABC(R) {
  const B = Math.asin(Math.max(-1, Math.min(1, -R[2][0])));
  let A, C;
  if (Math.abs(Math.cos(B)) > 1e-9) { A = Math.atan2(R[1][0], R[0][0]); C = Math.atan2(R[2][1], R[2][2]); }
  else { A = 0; C = Math.atan2(-R[1][2], R[1][1]); }
  return { A: A/_FD, B: B/_FD, C: C/_FD };
}
function _fmtQuatToABC(w, x, y, z) {
  const n = Math.hypot(w, x, y, z) || 1; w /= n; x /= n; y /= n; z /= n;
  return _fmtABC([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                  [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                  [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]]);
}
function _fmtABCToQuat(A, B, C) {
  const R = _fmtR(A, B, C), t = R[0][0] + R[1][1] + R[2][2];
  let w, x, y, z;
  if (t > 0) { const s = Math.sqrt(t + 1) * 2; w = s/4; x = (R[2][1]-R[1][2])/s; y = (R[0][2]-R[2][0])/s; z = (R[1][0]-R[0][1])/s; }
  else if (R[0][0] > R[1][1] && R[0][0] > R[2][2]) { const s = Math.sqrt(1 + R[0][0] - R[1][1] - R[2][2]) * 2; w = (R[2][1]-R[1][2])/s; x = s/4; y = (R[0][1]+R[1][0])/s; z = (R[0][2]+R[2][0])/s; }
  else if (R[1][1] > R[2][2]) { const s = Math.sqrt(1 + R[1][1] - R[0][0] - R[2][2]) * 2; w = (R[0][2]-R[2][0])/s; x = (R[0][1]+R[1][0])/s; y = s/4; z = (R[1][2]+R[2][1])/s; }
  else { const s = Math.sqrt(1 + R[2][2] - R[0][0] - R[1][1]) * 2; w = (R[1][0]-R[0][1])/s; x = (R[0][2]+R[2][0])/s; y = (R[1][2]+R[2][1])/s; z = s/4; }
  if (w < 0) { w = -w; x = -x; y = -y; z = -z; }
  return [w, x, y, z];
}
// APT-Werkzeugachse (i,j,k) zeigt von der Spitze zur Spindel; Werkzeug-Z des Roboters zeigt in das Teil: z = -(i,j,k).
// A ist frei (Drehung um die Werkzeugachse) und wird 0 gesetzt; B so gewählt, dass |B| ≤ 90° (senkrecht nach unten = B 0, C 180).
function _fmtAxisToABC(i, j, k) {
  const n = Math.hypot(i, j, k) || 1, zx = -i/n, zy = -j/n, zz = -k/n;
  let C = -Math.asin(Math.max(-1, Math.min(1, zy))) / _FD, B = Math.atan2(zx, zz) / _FD;
  if (Math.abs(B) > 90) { B -= 180 * Math.sign(B); C = 180 - C; }
  if (C > 180) C -= 360; if (C <= -180) C += 360;
  return { A: 0, B: B, C: C };
}
function _fmtABCToAxis(A, B, C) { const R = _fmtR(A, B, C); return [-R[0][2], -R[1][2], -R[2][2]]; }
const _f3 = v => (Math.abs(v) < 5e-7 ? 0 : v).toFixed(3);

// ── gemeinsamer Programmaufbau (Struktur wie parseKRL) ───────────────────
function _progBuilder(base0) {
  const steps = [], positions = [], din = {}, dout = {}, anout = {};
  let vel = 0.167, tool = 1, base = base0 || 0;
  const snap = () => ({ variables: {}, digitalIn: {...din}, digitalOut: {...dout}, analogOut: {...anout}, analogIn: {} });
  const W = o => { o.rel = {X:o.X, Y:o.Y, Z:o.Z, A:o.A, B:o.B, C:o.C}; o.baseN = base; o.toolN = tool; return o; };
  const step = (ln, type, extra) => steps.push({ lineNum: ln, type, ...extra, snapshot: snap() });
  return {
    get vel() { return vel; },
    move(type, p, ln) { const pi = positions.length; positions.push(W({ type, X:p.X, Y:p.Y, Z:p.Z, A:p.A, B:p.B, C:p.C, lineNum: ln, velCP: vel, snapshot: snap() })); step(ln, 'move', { posIdx: pi, label: type }); },
    circ(via, end, ln) { const s = snap(), pi = positions.length;
      positions.push(W({ type:'CIRC_AUX', X:via.X, Y:via.Y, Z:via.Z, A:via.A, B:via.B, C:via.C, lineNum: ln, snapshot: s }));
      positions.push(W({ type:'CIRC', X:end.X, Y:end.Y, Z:end.Z, A:end.A, B:end.B, C:end.C, lineNum: ln, velCP: vel, snapshot: s }));
      step(ln, 'move', { posIdx: pi + 1, label: 'CIRC' }); },
    ptpAxis(q, ln) { step(ln, 'ptpAxis', { angles: q.slice() }); },
    setVel(v, ln) { if (v > 0 && Math.abs(v - vel) > 1e-9) { vel = v; step(ln, 'velcp', { v }); } },
    setTool(n, ln) { if (n !== tool) { tool = n; step(ln, 'tool', { n }); } },
    setBase(n, ln) { if (n !== base) { base = n; step(ln, 'base', { n }); } },
    wait(t, ln) { step(ln, 'wait', { t }); },
    dout(n, on, ln) { dout[n] = on; step(ln, 'dout', { n, v: on ? 'TRUE' : 'FALSE' }); },
    din(n, ln) { din[n] = false; step(ln, 'din', { n }); },
    halt(ln) { step(ln, 'halt', {}); },
    other(raw, ln) { step(ln, 'other', { raw }); },
    tools: {},
    result() { return { steps, positions, tools: this.tools, finalState: { variables: {}, digitalIn: {...din}, digitalOut: {...dout}, analogOut: {...anout}, analogIn: {} } }; },
  };
}
const _num = s => parseFloat(String(s).trim());

// ── ABB RAPID ─────────────────────────────────────────────────────────────
// robtarget benannt (CONST/VAR/PERS … := [[x,y,z],[q1..q4],…]) oder direkt in der Bewegung;
// MoveL/MoveJ/MoveC, Offs(p,dx,dy,dz), MoveAbsJ [[j1..j6],…]; Geschwindigkeit vNNN oder [v_tcp,…];
// tooldata/wobjdata aus dem Modul werden für die Simulation übernommen (tool0 = Flansch, wobj0 = Welt);
// nicht deklarierte toolN/wobjN → TOOL N / BASE N des Roboters. SetDO, WaitDI, WaitTime, Stop.
// Werkzeugrahmen: Z aus dem Flansch, X bei Grundstellung nach unten (ABB tool0 = KUKA FLANGE = TCP-Nullrahmen).
const ABB_TOOL0 = 100;              // Werkzeugnummer für tool0 (Flansch)
var _fmtWobjSrc = {};               // zuletzt gelesene wobjdata je Name (Änderungen in der BASE-Eingabe bleiben erhalten)
function _fmtPoseMat(x, y, z, q) { const o = _fmtQuatToABC(q[0], q[1], q[2], q[3]), R = _fmtR(o.A, o.B, o.C); return { R, t: [x, y, z] }; }
function _fmtMatMul(a, b) {
  const R = [0, 1, 2].map(i => [0, 1, 2].map(j => a.R[i][0]*b.R[0][j] + a.R[i][1]*b.R[1][j] + a.R[i][2]*b.R[2][j]));
  return { R, t: [0, 1, 2].map(i => a.R[i][0]*b.t[0] + a.R[i][1]*b.t[1] + a.R[i][2]*b.t[2] + a.t[i]) };
}
// zwei [[x,y,z],[q1..q4]]-Rahmen ab Position k im Literal
function _fmtFrames(lit) {
  const re = /\[\s*\[([^\[\]]+)\]\s*,\s*\[([^\[\]]+)\]\s*\]/g, out = []; let m;
  while ((m = re.exec(lit))) { const p = m[1].split(',').map(_num), q = m[2].split(',').map(_num); if (p.length === 3 && q.length === 4) out.push(_fmtPoseMat(p[0], p[1], p[2], q)); }
  return out;
}
function _fmtFrameObj(F) { const o = _fmtABC(F.R); return { x: F.t[0], y: F.t[1], z: F.t[2], a: o.A, b: o.B, c: o.C }; }
function parseABB(text) {
  const P = _progBuilder(), lines = text.split(/\r?\n/), targets = {}, speeds = {};
  // Programmzeilen mit Fortsetzung bis ';' zusammenführen (Zeilennummer = erste Zeile)
  const stmts = []; let buf = '', start = 0;
  lines.forEach((raw, ln) => {
    const l = raw.replace(/!.*$/, '').trim(); if (!l) return;
    if (!buf) start = ln; buf += (buf ? ' ' : '') + l;
    if (/;\s*$/.test(l) || /^(MODULE|ENDMODULE|PROC|ENDPROC|ENDFUNC|ENDTRAP)\b/i.test(l)) { stmts.push({ ln: start, s: buf.replace(/;\s*$/, '') }); buf = ''; }
  });
  if (buf) stmts.push({ ln: start, s: buf });
  // Werkzeuge und Werkobjekte des Moduls
  const toolId = {}, wobjId = {};
  let nextTool = ABB_TOOL0 + 1;
  stmts.forEach(({ s }) => {
    let m;
    if ((m = s.match(/^(?:LOCAL\s+)?(?:TASK\s+)?(?:CONST|VAR|PERS)\s+tooldata\s+(\w+)\s*:=\s*\[(.*)\]\s*$/i))) {
      const F = _fmtFrames(m[2])[0]; if (!F) return;
      const id = /^tool0$/i.test(m[1]) ? ABB_TOOL0 : nextTool++;
      toolId[m[1].toLowerCase()] = id; P.tools[id] = Object.assign(_fmtFrameObj(F), { name: m[1], robhold: !/^\s*FALSE/i.test(m[2]) });
    } else if ((m = s.match(/^(?:LOCAL\s+)?(?:TASK\s+)?(?:CONST|VAR|PERS)\s+wobjdata\s+(\w+)\s*:=\s*\[(.*)\]\s*$/i))) {
      if (/^wobj0$/i.test(m[1])) return;
      const fr = _fmtFrames(m[2]); if (!fr.length) return;
      const W = _fmtFrameObj(fr.length > 1 ? _fmtMatMul(fr[0], fr[1]) : fr[0]);
      if (typeof baseList === 'undefined') return;
      let i = baseList.findIndex(b => b && b.abbName === m[1]);
      const dn = m[1].match(/^wobj(\d+)$/i);   // wobjN ohne Zuordnung → BASE N
      if (i < 0 && dn && +dn[1] >= 1 && +dn[1] <= baseList.length && !baseList[+dn[1] - 1].abbName) i = +dn[1] - 1;
      if (i < 0) { baseList.push({ x: 0, y: 0, z: 0, a: 0, b: 0, c: 0 }); i = baseList.length - 1; _fmtWobjSrc[m[1]] = null; }
      const e = baseList[i]; e.abbName = m[1]; e.name = m[1];
      if (_fmtWobjSrc[m[1]] !== m[2]) { Object.assign(e, W); _fmtWobjSrc[m[1]] = m[2]; }
      wobjId[m[1].toLowerCase()] = i + 1;
    }
  });
  const toolNum = nm => { const k = nm.toLowerCase();
    if (toolId[k]) return toolId[k];
    if (k === 'tool0') { P.tools[ABB_TOOL0] = { x: 0, y: 0, z: 0, a: 0, b: 0, c: 0, name: 'tool0' }; return ABB_TOOL0; }
    const n = parseInt(k.replace(/\D/g, ''), 10); return n > 0 ? n : 1; };
  const wobjNum = nm => { const k = nm.toLowerCase(); if (wobjId[k]) return wobjId[k]; if (k === 'wobj0') return 0;
    const n = parseInt(k.replace(/\D/g, ''), 10); return isNaN(n) ? 0 : n; };
  const poseOf = lit => {
    const m = lit.match(/\[\s*\[([^\]]+)\]\s*,\s*\[([^\]]+)\]/); if (!m) return null;
    const xyz = m[1].split(',').map(_num), q = m[2].split(',').map(_num), o = _fmtQuatToABC(q[0], q[1], q[2], q[3]);
    return { X: xyz[0], Y: xyz[1], Z: xyz[2], A: o.A, B: o.B, C: o.C };
  };
  const target = t => {
    t = t.trim();
    if (t[0] === '[') return poseOf(t);
    const off = t.match(/^Offs\s*\(\s*(\w+)\s*,([^,]+),([^,]+),([^)]+)\)/i);
    if (off && targets[off[1]]) { const b = targets[off[1]]; return { ...b, X: b.X + _num(off[2]), Y: b.Y + _num(off[3]), Z: b.Z + _num(off[4]) }; }
    return targets[t] ? { ...targets[t] } : null;
  };
  // Argumente einer Bewegung trennen (Kommas in [] und () nicht trennen)
  const args = s => { const out = []; let d = 0, cur = '';
    for (const ch of s) { if (ch === '[' || ch === '(') d++; if (ch === ']' || ch === ')') d--; if (ch === ',' && d === 0) { out.push(cur.trim()); cur = ''; } else cur += ch; }
    if (cur.trim()) out.push(cur.trim()); return out; };
  const speedOf = v => { if (!v) return null; v = v.trim();
    if (v[0] === '[') return _num(v.slice(1)) / 1000;
    if (speeds[v] !== undefined) return speeds[v];
    const m = v.match(/^v(\d+)$/i); if (m) return +m[1] / 1000;
    if (/^vmax$/i.test(v)) return 5; return null; };
  const toolWobj = (a, ln) => {
    const rest = a.join(','), w = rest.match(/\\WObj\s*:=\s*(\w+)/i);
    // Werkzeug = Argument nach Geschwindigkeit und Zone (Name beliebig), sonst erstes tool…-Argument
    const plain = a.slice(1).map(x => x.replace(/\\.*$/, '').trim());
    const tl = plain.find(x => toolId[x.toLowerCase()]) || plain.find(x => /^tool\w*$/i.test(x));
    if (tl) P.setTool(toolNum(tl), ln);
    P.setBase(w ? wobjNum(w[1]) : 0, ln);
  };
  stmts.forEach(({ ln, s }) => {
    let m;
    if (/^(?:LOCAL\s+)?(?:TASK\s+)?(?:CONST|VAR|PERS)\s+(tooldata|wobjdata|loaddata)\b/i.test(s)) return;
    if ((m = s.match(/^(?:LOCAL\s+)?(?:TASK\s+)?(?:CONST|VAR|PERS)\s+robtarget\s+(\w+)\s*:=\s*(\[.*\])/i))) { const p = poseOf(m[2]); if (p) targets[m[1]] = p; return; }
    if ((m = s.match(/^(?:LOCAL\s+)?(?:CONST|VAR|PERS)\s+speeddata\s+(\w+)\s*:=\s*\[([^,\]]+)/i))) { speeds[m[1]] = _num(m[2]) / 1000; return; }
    if ((m = s.match(/^(MoveL|MoveJ)\s+(.*)$/i))) {
      const a = args(m[2]), p = target(a[0]); if (!p) { P.other(s, ln); return; }
      const v = speedOf(a[1]); if (v) P.setVel(v, ln); toolWobj(a, ln);
      P.move(/MoveJ/i.test(m[1]) ? 'PTP' : 'LIN', p, ln); return; }
    if ((m = s.match(/^MoveC\s+(.*)$/i))) {
      const a = args(m[1]), via = target(a[0]), end = target(a[1]); if (!via || !end) { P.other(s, ln); return; }
      const v = speedOf(a[2]); if (v) P.setVel(v, ln); toolWobj(a.slice(1), ln); P.circ(via, end, ln); return; }
    if ((m = s.match(/^MoveAbsJ\s+\[\s*\[([^\]]+)\]/i))) { P.ptpAxis(m[1].split(',').map(_num).slice(0, 6), ln); return; }
    if ((m = s.match(/^SetDO\s+(\w+)\s*,\s*(\d)/i))) { P.dout(parseInt(m[1].replace(/\D/g, ''), 10) || 1, m[2] === '1', ln); return; }
    if ((m = s.match(/^WaitDI\s+(\w+)/i))) { P.din(parseInt(m[1].replace(/\D/g, ''), 10) || 1, ln); return; }
    if ((m = s.match(/^WaitTime\s+([\d.]+)/i))) { P.wait(_num(m[1]), ln); return; }
    if (/^(Stop|Break)\b/i.test(s)) { P.halt(ln); return; }
    if (/^(MODULE|ENDMODULE|PROC|ENDPROC)\b/i.test(s)) return;
    P.other(s, ln);
  });
  return P.result();
}

// ── FANUC TP (LS) ─────────────────────────────────────────────────────────
// /MN: J/L P[n] … , C P[via] + Folgezeile P[end]; UTOOL_NUM/UFRAME_NUM; DO[n]=ON/OFF, WAIT DI[n]=ON, WAIT t(sec).
// /POS: P[n]{ GP1: UF : u, UT : t, X = … mm, Y, Z, W = … deg, P, R }  (W um X, P um Y, R um Z → A=R, B=P, C=W)
// oder Achswerte J1…J6 → PTP mit Achswinkeln.
function parseFANUC(text) {
  const P = _progBuilder(), lines = text.split(/\r?\n/), pts = {};
  const posSec = text.match(/\/POS([\s\S]*?)(\/END|$)/i);
  if (posSec) {
    const re = /P\[(\d+)(?::[^\]]*)?\]\s*\{([\s\S]*?)\}\s*;/gi; let m;
    while ((m = re.exec(posSec[1]))) {
      const b = m[2], g = k => { const x = b.match(new RegExp('\\b' + k + '\\s*=\\s*([-+\\d.]+)', 'i')); return x ? _num(x[1]) : null; };
      const uf = b.match(/UF\s*:\s*(\d+)/i), ut = b.match(/UT\s*:\s*(\d+)/i);
      const e = { uf: uf ? +uf[1] : 0, ut: ut ? +ut[1] : 1 };
      if (g('X') !== null) Object.assign(e, { X: g('X'), Y: g('Y'), Z: g('Z'), A: g('R') || 0, B: g('P') || 0, C: g('W') || 0 });
      else if (g('J1') !== null) e.q = [g('J1'), g('J2'), g('J3'), g('J4'), g('J5'), g('J6')].map(v => v || 0);
      pts[+m[1]] = e;
    }
  }
  let inMN = false, prevC = null;
  const velOf = (s, joint) => { let m;
    if ((m = s.match(/([\d.]+)\s*mm\/sec/i))) return _num(m[1]) / 1000;
    if ((m = s.match(/([\d.]+)\s*cm\/min/i))) return _num(m[1]) / 6000;
    if ((m = s.match(/([\d.]+)\s*inch\/min/i))) return _num(m[1]) * 0.0254 / 60;
    if (joint && (m = s.match(/([\d.]+)\s*%/))) return _num(m[1]) / 100 * 2.0;
    return null; };
  const go = (type, n, s, ln) => { const e = pts[n]; if (!e) { P.other(s, ln); return; }
    P.setTool(e.ut || 1, ln); P.setBase(e.uf || 0, ln);
    const v = velOf(s, type === 'PTP'); if (v) P.setVel(v, ln);
    if (e.q) P.ptpAxis(e.q, ln); else P.move(type, e, ln); };
  lines.forEach((raw, ln) => {
    const t = raw.trim();
    if (/^\/MN\b/i.test(t)) { inMN = true; return; }
    if (/^\/(POS|END)\b/i.test(t)) { inMN = false; return; }
    if (!inMN) return;
    const l = t.replace(/^\d+\s*:\s*/, '').replace(/;\s*$/, '').replace(/!.*$/, '').trim(); if (!l) return;
    let m;
    if (prevC !== null && (m = l.match(/^:?\s*P\[(\d+)/i))) {
      const a = pts[prevC], e = pts[+m[1]]; prevC = null;
      if (a && e && !a.q && !e.q) { P.setTool(e.ut || 1, ln); P.setBase(e.uf || 0, ln); const v = velOf(l, false); if (v) P.setVel(v, ln); P.circ(a, e, ln); }
      else P.other(l, ln); return; }
    prevC = null;
    if ((m = l.match(/^J\s+P\[(\d+)/i))) { go('PTP', +m[1], l, ln); return; }
    if ((m = l.match(/^L\s+P\[(\d+)/i))) { go(/CNT/i.test(l) ? 'SLIN' : 'LIN', +m[1], l, ln); return; }
    if ((m = l.match(/^C\s+P\[(\d+)/i))) { prevC = +m[1]; return; }
    if ((m = l.match(/^UTOOL_NUM\s*=\s*(\d+)/i))) { P.setTool(+m[1] || 1, ln); return; }
    if ((m = l.match(/^UFRAME_NUM\s*=\s*(\d+)/i))) { P.setBase(+m[1], ln); return; }
    if ((m = l.match(/^DO\[(\d+)[^\]]*\]\s*=\s*(ON|OFF)/i))) { P.dout(+m[1], /ON/i.test(m[2]), ln); return; }
    if ((m = l.match(/^WAIT\s+DI\[(\d+)/i))) { P.din(+m[1], ln); return; }
    if ((m = l.match(/^WAIT\s+([\d.]+)\s*\(sec\)/i))) { P.wait(_num(m[1]), ln); return; }
    if (/^(PAUSE|ABORT)\b/i.test(l)) { P.halt(ln); return; }
    P.other(l, ln);
  });
  return P.result();
}

// ── APT / CL-Data ────────────────────────────────────────────────────────
// GOTO/x,y,z[,i,j,k], FROM/…, RAPID, FEDRAT/f[,MMPM|IPM], CIRCLE/xc,yc,zc,i,j,k,r + GOTO (Kreisende),
// UNITS/INCHES, LOADTL/n, TLAXIS/i,j,k, PPRINT, FINI. Fortsetzungszeilen mit '$'.
// Orientierung aus der Werkzeugachse; A (Drehung um die Werkzeugachse) = 0 → mit der Robot Map optimieren.
// Koordinaten beziehen sich auf den Werkstücknullpunkt = BASE 1 (wie $BASE=BASE_DATA[1]).
function parseAPT(text) {
  const P = _progBuilder(1), lines = text.split(/\r?\n/);
  let scale = 1, axis = [0, 0, 1], rapid = false, circle = null, cur = null, feedMmMin = null;
  const recs = []; let buf = '', start = 0;
  lines.forEach((raw, ln) => { let l = raw.trim(); if (!l) return; if (!buf) start = ln;
    if (/\$\s*$/.test(l)) { buf += l.replace(/\$\s*$/, ''); return; } recs.push({ ln: start, s: (buf + l).trim() }); buf = ''; });
  if (buf) recs.push({ ln: start, s: buf });
  const pose = (x, y, z, ax) => { const o = _fmtAxisToABC(ax[0], ax[1], ax[2]); return { X: x*scale, Y: y*scale, Z: z*scale, ...o }; };
  recs.forEach(({ ln, s }) => {
    const m = s.match(/^([A-Z0-9_]+)\s*\/\s*(.*)$/i), word = (m ? m[1] : s).toUpperCase(), a = m ? m[2].split(',').map(x => x.trim()) : [];
    const n = a.map(_num);
    switch (word) {
      case 'UNITS': scale = /INCH/i.test(a[0] || '') ? 25.4 : 1; return;
      case 'LOADTL': P.setTool(n[0] > 0 ? n[0] : 1, ln); return;
      case 'TLAXIS': if (n.length >= 3 && n.slice(0, 3).every(isFinite)) axis = n.slice(0, 3); return;
      case 'FEDRAT': { let f = n.find(isFinite); if (!isFinite(f)) return;
        const ipm = a.some(x => /IPM/i.test(x)); f = f * (ipm ? 25.4 : 1); feedMmMin = f; rapid = false; P.setVel(f / 60000, ln); return; }
      case 'RAPID': rapid = true; return;
      case 'CIRCLE': if (n.length >= 7) circle = { c: [n[0]*scale, n[1]*scale, n[2]*scale], ax: [n[3], n[4], n[5]] }; return;
      case 'FROM': case 'GOTO': {
        if (n.length < 3 || !n.slice(0, 3).every(isFinite)) { P.other(s, ln); return; }
        if (n.length >= 6 && n.slice(3, 6).every(isFinite)) axis = n.slice(3, 6);
        const p = pose(n[0], n[1], n[2], axis);
        if (circle && cur) { _aptArc(P, cur, p, circle, ln); circle = null; }
        else if (word === 'FROM' || rapid) { const v = P.vel; P.setVel(rapid ? 0.5 : v, ln); P.move('PTP', p, ln); if (rapid && feedMmMin) P.setVel(feedMmMin / 60000, ln); rapid = false; }
        else P.move('LIN', p, ln);
        cur = p; return; }
      case 'PPRINT': case 'PARTNO': case 'MULTAX': case 'FINI': case 'END': case 'CUTTER': case 'SPINDL': case 'COOLNT': case 'MACHIN': case 'CLDATA': case 'SET': return;
      default: P.other(s, ln);
    }
  });
  return P.result();
}
// Kreisbogen von a nach b um Mittelpunkt c, Drehsinn rechtshändig um die Kreisachse → CIRC mit Hilfspunkt in der Bogenmitte
// (Vollkreis: zwei Halbkreise)
function _aptArc(P, a, b, circ, ln) {
  const c = circ.c, nrm = Math.hypot(...circ.ax) || 1, n = circ.ax.map(v => v / nrm);
  const va = [a.X-c[0], a.Y-c[1], a.Z-c[2]], vb = [b.X-c[0], b.Y-c[1], b.Z-c[2]];
  const cross = (u, v) => [u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0]], dot = (u, v) => u[0]*v[0]+u[1]*v[1]+u[2]*v[2];
  let sweep = Math.atan2(dot(cross(va, vb), n), dot(va, vb)); if (sweep <= 1e-6) sweep += 2 * Math.PI;
  const rot = (v, t) => { const k = cross(n, v), d = dot(n, v), ct = Math.cos(t), st = Math.sin(t);
    return [0, 1, 2].map(i => v[i]*ct + k[i]*st + n[i]*d*(1-ct)); };
  const mid = (t, from) => { const r = rot(va, t); return { ...from, X: c[0]+r[0], Y: c[1]+r[1], Z: c[2]+r[2] }; };
  if (sweep > 2 * Math.PI - 1e-3) { const h = mid(Math.PI, b); P.circ(mid(Math.PI/2, b), h, ln); P.circ(mid(1.5*Math.PI, b), b, ln); }
  else P.circ(mid(sweep / 2, b), b, ln);
}

// ── Erzeuger (Formatwechsel) ─────────────────────────────────────────────
// Grundlage: geparste Daten, Koordinaten wie im Programm (rel, ohne BASE-Verrechnung).
function _progWalk(pd, cb) {
  const pos = pd.positions;
  pd.steps.forEach(s => {
    if (s.type === 'move') { const p = pos[s.posIdx]; if (!p) return;
      if (p.type === 'CIRC') cb('circ', { via: pos[s.posIdx - 1].rel || pos[s.posIdx - 1], end: p.rel || p, src: p }); else cb('move', { p: p.rel || p, src: p }); }
    else cb(s.type, s);
  });
}
function generateProgram(fmt, pd, name) {
  name = (name || 'ROBSIM').replace(/\W/g, '_').toUpperCase();
  const L = [];
  if (fmt === 'kuka') {
    L.push('DEF ' + name + '()');
    Object.keys(pd.tools || {}).forEach(n => { const t = pd.tools[n];
      L.push('TOOL_DATA[' + n + ']={X ' + _f3(t.x) + ', Y ' + _f3(t.y) + ', Z ' + _f3(t.z) + ', A ' + _f3(t.a) + ', B ' + _f3(t.b) + ', C ' + _f3(t.c) + '} ; ' + (t.name || '')); });
    L.push('$BASE=BASE_DATA[1]', '$TOOL=TOOL_DATA[1]');
    const kp = p => '{X ' + _f3(p.X) + ', Y ' + _f3(p.Y) + ', Z ' + _f3(p.Z) + ', A ' + _f3(p.A) + ', B ' + _f3(p.B) + ', C ' + _f3(p.C) + '}';
    _progWalk(pd, (t, s) => {
      if (t === 'move') L.push((s.src.type === 'SLIN' ? 'SLIN ' : s.src.type === 'PTP' ? 'PTP ' : 'LIN ') + kp(s.p) + (s.src.type === 'PTP' ? '' : ' C_DIS'));
      else if (t === 'circ') L.push('CIRC ' + kp(s.via) + ', ' + kp(s.end) + ' C_DIS');
      else if (t === 'ptpAxis') L.push('PTP {' + s.angles.map((x, j) => 'A' + (j + 1) + ' ' + _f3(x)).join(', ') + '}');
      else if (t === 'velcp') L.push('$VEL.CP=' + (+s.v).toFixed(4));
      else if (t === 'tool') L.push('$TOOL=TOOL_DATA[' + s.n + ']');
      else if (t === 'base') L.push('$BASE=BASE_DATA[' + (s.n || 1) + ']');
      else if (t === 'wait') L.push('WAIT SEC ' + s.t);
      else if (t === 'dout') L.push('$OUT[' + s.n + ']=' + (/TRUE|ON|1/i.test(s.v) ? 'TRUE' : 'FALSE'));
      else if (t === 'din') L.push('WAIT FOR $IN[' + s.n + ']');
      else if (t === 'halt') L.push('HALT');
    });
    L.push('END');
  } else if (fmt === 'abb') {
    let tool = 1, wobj = 0, v = 0.167;
    const rt = p => { const q = _fmtABCToQuat(p.A, p.B, p.C);
      return '[[' + _f3(p.X) + ',' + _f3(p.Y) + ',' + _f3(p.Z) + '],[' + q.map(x => x.toFixed(6)).join(',') + '],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]]'; };
    const tName = n => (pd.tools && pd.tools[n] && pd.tools[n].name) || (n === ABB_TOOL0 ? 'tool0' : 'tool' + n);
    const wName = n => (typeof baseList !== 'undefined' && baseList[n - 1] && baseList[n - 1].abbName) || 'wobj' + n;
    const sd = () => '[' + Math.round(v * 1000) + ',500,5000,1000]', tw = () => tName(tool) + (wobj ? '\\WObj:=' + wName(wobj) : '');
    const fr = (x, y, z, A, B, C) => '[[' + _f3(x) + ',' + _f3(y) + ',' + _f3(z) + '],[' + _fmtABCToQuat(A, B, C).map(q => q.toFixed(6)).join(',') + ']]';
    L.push('MODULE ' + name);
    // Werkzeuge und Werkobjekte mit den Werten der Simulation deklarieren
    const usedT = new Set(), usedW = new Set(); let ct = 1, cw = 0;
    pd.steps.forEach(s => { if (s.type === 'tool') ct = s.n; if (s.type === 'base') cw = s.n || 0; if (s.type === 'move') { usedT.add(ct); if (cw) usedW.add(cw); } });
    usedT.forEach(n => { if (n === ABB_TOOL0) return;
      const t = (pd.tools && pd.tools[n]) || (typeof _tcpEntryForTool === 'function' ? _tcpEntryForTool(n) : null); if (!t) return;
      L.push('  PERS tooldata ' + tName(n) + ' := [TRUE,' + fr(t.x || 0, t.y || 0, t.z || 0, t.a || 0, t.b || 0, t.c || 0) + ',[1,[0,0,1],[1,0,0,0],0,0,0]];'); });
    usedW.forEach(n => { const b = typeof baseList !== 'undefined' && baseList[n - 1]; if (!b) return;
      L.push('  PERS wobjdata ' + wName(n) + ' := [FALSE,TRUE,"",' + fr(b.x || 0, b.y || 0, b.z || 0, b.a || 0, b.b || 0, b.c || 0) + ',[[0,0,0],[1,0,0,0]]];'); });
    L.push('  PROC main()');
    _progWalk(pd, (t, s) => {
      if (t === 'move') L.push('    ' + (s.src.type === 'PTP' ? 'MoveJ ' : 'MoveL ') + rt(s.p) + ', ' + sd() + ', z1, ' + tw() + ';');
      else if (t === 'circ') L.push('    MoveC ' + rt(s.via) + ', ' + rt(s.end) + ', ' + sd() + ', z1, ' + tw() + ';');
      else if (t === 'ptpAxis') L.push('    MoveAbsJ [[' + s.angles.map(_f3).join(',') + '],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]], v500, fine, ' + tName(tool) + ';');
      else if (t === 'velcp') v = +s.v || v;
      else if (t === 'tool') tool = s.n;
      else if (t === 'base') wobj = s.n || 0;
      else if (t === 'wait') L.push('    WaitTime ' + s.t + ';');
      else if (t === 'dout') L.push('    SetDO do' + s.n + ', ' + (/TRUE|ON|1/i.test(s.v) ? 1 : 0) + ';');
      else if (t === 'din') L.push('    WaitDI di' + s.n + ', 1;');
      else if (t === 'halt') L.push('    Stop;');
    });
    L.push('  ENDPROC', 'ENDMODULE');
  } else if (fmt === 'fanuc') {
    let tool = 1, uf = 0, v = 0.167, k = 0; const mn = [], ps = [];
    const P = p => { k++; ps.push('P[' + k + ']{\n   GP1:\n\tUF : ' + uf + ', UT : ' + tool + ',\t\tCONFIG : \'N U T, 0, 0, 0\',\n\tX = ' + _f3(p.X) + '  mm,\tY = ' + _f3(p.Y) +
      '  mm,\tZ = ' + _f3(p.Z) + '  mm,\n\tW = ' + _f3(p.C) + ' deg,\tP = ' + _f3(p.B) + ' deg,\tR = ' + _f3(p.A) + ' deg\n};'); return k; };
    const J = q => { k++; ps.push('P[' + k + ']{\n   GP1:\n\tUF : ' + uf + ', UT : ' + tool + ',\n\t' + q.map((x, i) => 'J' + (i + 1) + ' = ' + _f3(x) + ' deg').join(',\t') + '\n};'); return k; };
    const sp = () => Math.max(1, Math.round(v * 1000)) + 'mm/sec';
    _progWalk(pd, (t, s) => {
      if (t === 'move') { const n = P(s.p); mn.push(s.src.type === 'PTP' ? 'J P[' + n + '] 100% FINE' : 'L P[' + n + '] ' + sp() + (s.src.type === 'SLIN' ? ' CNT100' : ' CNT50')); }
      else if (t === 'circ') { const a = P(s.via), b = P(s.end); mn.push('C P[' + a + ']'); mn.push('    :  P[' + b + '] ' + sp() + ' CNT50'); }
      else if (t === 'ptpAxis') mn.push('J P[' + J(s.angles) + '] 100% FINE');
      else if (t === 'velcp') v = +s.v || v;
      else if (t === 'tool') { tool = s.n; mn.push('UTOOL_NUM=' + s.n); }
      else if (t === 'base') { uf = s.n || 0; mn.push('UFRAME_NUM=' + uf); }
      else if (t === 'wait') mn.push('WAIT ' + (+s.t).toFixed(2) + '(sec)');
      else if (t === 'dout') mn.push('DO[' + s.n + ']=' + (/TRUE|ON|1/i.test(s.v) ? 'ON' : 'OFF'));
      else if (t === 'din') mn.push('WAIT DI[' + s.n + ']=ON');
      else if (t === 'halt') mn.push('PAUSE');
    });
    L.push('/PROG  ' + name, '/ATTR', 'DEFAULT_GROUP\t= 1,*,*,*,*;', '/MN');
    mn.forEach((x, i) => L.push(String(i + 1).padStart(4) + ':  ' + x + ' ;'));
    L.push('/POS'); ps.forEach(x => L.push(x)); L.push('/END');
  } else if (fmt === 'apt') {
    let rapid = false;
    L.push('PARTNO/' + name, 'UNITS/MM', 'MULTAX/ON');
    const g = p => { const ax = _fmtABCToAxis(p.A, p.B, p.C); return 'GOTO/' + _f3(p.X) + ',' + _f3(p.Y) + ',' + _f3(p.Z) + ',' + ax.map(x => x.toFixed(6)).join(','); };
    _progWalk(pd, (t, s) => {
      if (t === 'move') { if (s.src.type === 'PTP') L.push('RAPID'); L.push(g(s.p)); }
      else if (t === 'circ') { L.push(g(s.via)); L.push(g(s.end)); }   // Bogen als Sehnenzug Hilfspunkt/Endpunkt
      else if (t === 'velcp') L.push('FEDRAT/' + (+s.v * 60000).toFixed(1) + ',MMPM');
      else if (t === 'tool') L.push('LOADTL/' + s.n);
      else if (t === 'wait') L.push('PPRINT WAIT ' + s.t + ' SEC');
    });
    L.push('FINI');
  }
  return L.join('\n') + '\n';
}

// Format am Inhalt erkennen (null = unklar)
function detectProgramFormat(text, fileName) {
  const ext = (fileName || '').toLowerCase().replace(/^.*\./, '.');
  for (const [k, f] of Object.entries(PROG_FORMATS)) if (fileName && f.accept.includes(ext)) return k;
  const t = text.slice(0, 20000);
  if (/^\s*\/PROG\b/m.test(t) && /^\s*\/MN\b/m.test(t)) return 'fanuc';
  if (/^\s*(MODULE\s+\w+|PROC\s+\w+\s*\()/mi.test(t) || /\bMove[LJC]\s+[\[\w]/.test(t)) return 'abb';
  if (/^\s*(GOTO|FEDRAT|CIRCLE|LOADTL|FROM)\s*\//mi.test(t)) return 'apt';
  if (/^\s*(DEF\s+\w+\s*\(|(LIN|PTP|SLIN|CIRC)\s*\{|LIN_EXTRUDE\s*\()/mi.test(t)) return 'kuka';
  return null;
}
function parseProgram(fmt, text) {
  if (fmt === 'abb') return parseABB(text);
  if (fmt === 'fanuc') return parseFANUC(text);
  if (fmt === 'apt') return parseAPT(text);
  return parseKRL(text);
}
