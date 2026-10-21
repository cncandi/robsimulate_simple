// RobSimul Simple — reduzierte Fassung von RobSimul V0.84
// Umfang: Roboterwahl (RobLib), KUKA-KRL-Simulation, A-Plot
// On-Demand-Rendering: nur rendern wenn nötig (spart GPU/CPU/Akku im Leerlauf)
let _renderFrames = 4;
function requestRender(n) { _renderFrames = Math.max(_renderFrames, n || 2); }
// console.log nur mit ?debug in der URL (Produktiv still; warn/error bleiben)
if (!/[?&]debug\b/.test(location.search)) { const _noop = function(){}; console.log = _noop; console.debug = _noop; }


// ═══════════════════════════════════════════════════
// KUKA KR8 R1420 HW — KINEMATIK
// Robot CS: X=vorwärts, Y=links, Z=oben
// Three.js scene: Z-up (camera up=(0,0,1)) → gleich wie Robot CS!
// ═══════════════════════════════════════════════════
const JOINTS_DEF = [
  {name:'A1',off:[150,0,450],  min:-170,max:170, axis:'Rz'},
  {name:'A2',off:[610,0,0],   min:-185,max:65,  axis:'Ry'},
  {name:'A3',off:[0,0,200],   min:-120,max:180, axis:'Ry'},
  {name:'A4',off:[630,0,0],   min:-165,max:165, axis:'Rx'},
  {name:'A5',off:[80,0,0],    min:-115,max:140, axis:'Ry'},
  {name:'A6',off:[0,0,0],     min:-180,max:180, axis:'Rx'}
];

const TCP_DEF = {x:364.5, y:0, z:46.5, a:0, b:90, c:0};
const FK_SIGNS = [-1,1,1,-1,1,-1];

// Current joint angles (degrees)
let jointAngles = [0,-90,90,0,0,0];

// ─── 3×3 matrix math ───
function mRz(a){const c=Math.cos(a),s=Math.sin(a);return[[c,-s,0],[s,c,0],[0,0,1]];}
function mRy(a){const c=Math.cos(a),s=Math.sin(a);return[[c,0,s],[0,1,0],[-s,0,c]];}
function mRx(a){const c=Math.cos(a),s=Math.sin(a);return[[1,0,0],[0,c,-s],[0,s,c]];}
function mMul(A,B){const R=[[0,0,0],[0,0,0],[0,0,0]];for(let i=0;i<3;i++)for(let j=0;j<3;j++)for(let k=0;k<3;k++)R[i][j]+=A[i][k]*B[k][j];return R;}
function mVec(M,v){return[M[0][0]*v[0]+M[0][1]*v[1]+M[0][2]*v[2],M[1][0]*v[0]+M[1][1]*v[1]+M[1][2]*v[2],M[2][0]*v[0]+M[2][1]*v[1]+M[2][2]*v[2]];}
function mT(M){return[[M[0][0],M[1][0],M[2][0]],[M[0][1],M[1][1],M[2][1]],[M[0][2],M[1][2],M[2][2]]];}
function rotZYX(a,b,c){const r=d=>d*Math.PI/180;return mMul(mRz(r(a)),mMul(mRy(r(b)),mRx(r(c))));}

// ─── FK ───────────────────────────────────────────
// Returns: { pts: [[x,y,z]×7, tcp], rot_final, rot_all }
function fkAll(angles_deg) {
  const r = a => a * Math.PI / 180;
  const axFns = [mRz,mRy,mRy,mRx,mRy,mRx];
  let pos = [0,0,0], rot = [[1,0,0],[0,1,0],[0,0,1]];
  const pts = [[0,0,0]];   // base origin
  const rots = [rot];

  for (let i = 0; i < 6; i++) {
    rot = mMul(rot, axFns[i](r(angles_deg[i] * FK_SIGNS[i])));
    const off = mVec(rot, JOINTS_DEF[i].off);
    pos = [pos[0]+off[0], pos[1]+off[1], pos[2]+off[2]];
    pts.push([...pos]);
    rots.push(rot);
  }

  // TCP: R_fc=Ry(90°) = KUKA Flansch-KS (Z vorwärts, X unten)
  const R_fc  = mRy(Math.PI/2);
  const R_usr = rotZYX(TCP_DEF.a, TCP_DEF.b, TCP_DEF.c);
  const R_tcp = mMul(R_fc, R_usr);
  const p_tcp_local = mVec(R_fc, [TCP_DEF.x, TCP_DEF.y, TCP_DEF.z]);
  const p_tcp_world = mVec(rot, p_tcp_local);
  const tcp = [pos[0]+p_tcp_world[0], pos[1]+p_tcp_world[1], pos[2]+p_tcp_world[2]];
  pts.push(tcp);

  const R_tcp_world = mMul(rot, R_tcp);
  return { pts, rot_final: rot, rots, tcp_rot: R_tcp_world };
}

// ─── IK (numerisch, DLS) ─────────────────────────
const ORI_SCALE = 100.0;

function logOriErr(Rt, R_cur) {
  const Re = mMul(Rt, mT(R_cur));
  const trace = Re[0][0]+Re[1][1]+Re[2][2];
  const cosT  = Math.max(-1, Math.min(1, (trace-1)*0.5));
  const theta = Math.acos(cosT);
  const sinT  = Math.sin(theta);
  if (sinT < 1e-7) return [0,0,0];
  const k = theta * ORI_SCALE / (2*sinT);
  return [(Re[2][1]-Re[1][2])*k, (Re[0][2]-Re[2][0])*k, (Re[1][0]-Re[0][1])*k];
}
function fkTCP_full(angles_deg) {
  const r = fkAll(angles_deg);
  return { pos: r.pts[7], rot: r.tcp_rot };
}

function err6(angs, tp, Rt) {
  const fk = fkTCP_full(angs);
  const ep = [tp[0]-fk.pos[0], tp[1]-fk.pos[1], tp[2]-fk.pos[2]];
  const eo = logOriErr(Rt, fk.rot);
  return [...ep, ...eo];
}

function solve6x6(A, b) {
  const M = A.map((row,i) => [...row, b[i]]);
  for (let c=0; c<6; c++) {
    let mx=c;
    for (let r=c+1;r<6;r++) if (Math.abs(M[r][c])>Math.abs(M[mx][c])) mx=r;
    [M[c],M[mx]]=[M[mx],M[c]];
    if (Math.abs(M[c][c])<1e-14) continue;
    for (let r=c+1;r<6;r++) {
      const f=M[r][c]/M[c][c];
      for (let j=c;j<=6;j++) M[r][j]-=f*M[c][j];
    }
  }
  const x=Array(6).fill(0);
  for (let i=5;i>=0;i--) {
    if (Math.abs(M[i][i])<1e-14) continue;
    x[i]=M[i][6];
    for (let j=i+1;j<6;j++) x[i]-=M[i][j]*x[j];
    x[i]/=M[i][i];
  }
  return x;
}

const R2DEG = 180 / Math.PI;

// ── Levenberg-Marquardt Kern ─────────────────────────────────────────────────
// cfg:
//   tp            [x,y,z]       Zielposition
//   Rt            3×3 Matrix    Zielorientierung (null wenn posOnly)
//   starts        Array         Start-Gelenkwinkel-Arrays
//   dt            number        Jacobian-Schrittweite
//   lam           number        LM-Dämpfung
//   tolP/tolO     number        Positions-/Orientierungstoleranz
//   maxIter       number        Max. Iterationen
//   stepMax       number        Max. Schrittweite
//   stepScale     number        step = min(stepMax, stepScale/max(1,score))
//   earlyStop     number        Bricht ab wenn score < (tolP+tolO)*earlyStop (0=aus)
//   okThresh      number        ok=true wenn score < okThresh
//   posOnly       bool          Nur Positionsfehler, 3D-Jacobian (default false)
//   a6penalty     {target,weight} Soft-Constraint auf A6 (default null)
//   a6fixed       number|null   A6 hart fixieren, nur A1–A5 optimieren
//   okExtra       fn(q)=>bool   Zusätzliche ok-Bedingung
function solveLM(cfg) {
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  const {
    tp, Rt, starts, dt, lam, tolP, tolO = 0, maxIter,
    stepMax, stepScale, earlyStop = 0, okThresh,
    posOnly = false, a6penalty = null, a6fixed = null, okExtra = null,
  } = cfg;

  let bestScore = Infinity;
  let bestQ = starts[0].slice();

  for (const startQ of starts) {
    const q = startQ.slice();
    if (a6fixed !== null) q[5] = a6fixed;

    for (let iter = 0; iter < maxIter; iter++) {
      let score, eP, eO, converged;

      if (posOnly) {
        // ── 3D Positions-Jacobian ──────────────────────────────────────────
        const fk = fkTCP_full(q);
        const ex = tp[0]-fk.pos[0], ey = tp[1]-fk.pos[1], ez = tp[2]-fk.pos[2];
        eP = Math.sqrt(ex*ex + ey*ey + ez*ez);
        const a6e = a6penalty ? (q[5]-a6penalty.target)*a6penalty.weight*Math.PI/180 : 0;
        score = eP + (a6penalty ? Math.abs(a6e)*R2DEG/a6penalty.weight : 0);
        if (score < bestScore) { bestScore = score; bestQ = q.slice(); }
        converged = eP < tolP && (!a6penalty || Math.abs(q[5]-a6penalty.target) < 1);
        if (converged) break;

        // Jacobian: J[i] = [∂ex/∂qi, ∂ey/∂qi, ∂ez/∂qi]
        const J3 = [];
        for (let i = 0; i < 6; i++) {
          const q1 = q.slice(); q1[i] += dt;
          const f1 = fkTCP_full(q1);
          J3.push([(f1.pos[0]-fk.pos[0])/dt, (f1.pos[1]-fk.pos[1])/dt, (f1.pos[2]-fk.pos[2])/dt]);
        }
        const JtJ = Array.from({length:6}, () => Array(6).fill(0));
        const Jte = Array(6).fill(0);
        const e3 = [ex, ey, ez];
        for (let i = 0; i < 6; i++) {
          for (let r = 0; r < 3; r++) {
            Jte[i] += J3[i][r] * e3[r];
            for (let j = 0; j < 6; j++) JtJ[i][j] += J3[i][r] * J3[j][r];
          }
          JtJ[i][i] += lam;
        }
        if (a6penalty) { JtJ[5][5] += a6penalty.weight**2; Jte[5] += a6e*a6penalty.weight; }
        const dq = solve6x6(JtJ, Jte);
        const step = Math.min(stepMax, stepScale/Math.max(1, bestScore));
        for (let i = 0; i < 6; i++) {
          if (!isFinite(dq[i])) continue;
          q[i] = clamp(q[i]-step*dq[i], JOINTS_DEF[i].min, JOINTS_DEF[i].max);
        }

      } else {
        // ── 6D Positions+Orientierungs-Jacobian ───────────────────────────
        const e = err6(q, tp, Rt);
        eP = Math.sqrt(e[0]**2+e[1]**2+e[2]**2);
        const a6e = a6penalty ? (q[5]-a6penalty.target)*a6penalty.weight*Math.PI/180 : 0;
        eO = Math.sqrt(e[3]**2+e[4]**2+e[5]**2+(a6penalty ? a6e**2 : 0));
        score = eP + eO;
        if (score < bestScore) { bestScore = score; bestQ = q.slice(); }
        if (eP < tolP && eO < tolO) break;

        const J = [];
        for (let i = 0; i < 6; i++) {
          if (a6fixed !== null && i === 5) { J.push([0,0,0,0,0,0]); continue; }
          const q1 = q.slice(); q1[i] += dt;
          const e1 = err6(q1, tp, Rt);
          J.push([(e1[0]-e[0])/dt,(e1[1]-e[1])/dt,(e1[2]-e[2])/dt,
                  (e1[3]-e[3])/dt,(e1[4]-e[4])/dt,(e1[5]-e[5])/dt]);
        }
        const JtJ = Array.from({length:6}, () => Array(6).fill(0));
        const Jte = Array(6).fill(0);
        for (let i = 0; i < 6; i++) {
          for (let r = 0; r < 6; r++) {
            Jte[i] += J[i][r]*e[r];
            for (let j = 0; j < 6; j++) JtJ[i][j] += J[i][r]*J[j][r];
          }
          JtJ[i][i] += lam;
        }
        if (a6penalty) { JtJ[5][5] += a6penalty.weight**2; Jte[5] += a6e*a6penalty.weight; }
        const dq = solve6x6(JtJ, Jte);
        const step = Math.min(stepMax, stepScale/Math.max(1, bestScore));
        const updateN = (a6fixed !== null) ? 5 : 6;
        for (let i = 0; i < updateN; i++) {
          if (!isFinite(dq[i])) continue;
          q[i] = clamp(q[i]-step*dq[i], JOINTS_DEF[i].min, JOINTS_DEF[i].max);
        }
        if (a6fixed !== null) q[5] = a6fixed;
      }
    }
    if (earlyStop > 0 && bestScore < (tolP+tolO)*earlyStop) break;
  }

  const ok = bestScore < okThresh && (okExtra ? okExtra(bestQ) : true);
  return {angles: bestQ, score: bestScore, ok};
}

// ── Analytische IK (Zentralhand) ─────────────────────────────────────────────
// Kette: Rz,Ry,Ry,Rx,Ry,Rx (wie fkAll). Bei A4/A5-Versatz nur entlang X und A6 ohne Versatz
// schneiden sich die Handachsen in p4 (Zentralhand) -> geschlossene Lösung, bis zu 8 Konfigurationen.
// Jede Lösung wird per FK verifiziert; ohne gültige Lösung liefert die Funktion null (-> numerische IK).
const R2D = 180/Math.PI;
function ikAnalyticSupported() {
  const J = JOINTS_DEF;
  return J.length === 6 &&
    Math.abs(J[3].off[1]) < 1e-9 && Math.abs(J[3].off[2]) < 1e-9 &&
    Math.abs(J[4].off[1]) < 1e-9 && Math.abs(J[4].off[2]) < 1e-9 &&
    Math.abs(J[5].off[1]) < 1e-9 && Math.abs(J[5].off[2]) < 1e-9;
}

// Alle Gelenklösungen (Grad) für eine TCP-Pose; ohne Achsgrenzen-/360°-Auswahl
function ikAnalyticAll(tx, ty, tz, ta, tb, tc) {
  const J = JOINTS_DEF, S = FK_SIGNS;
  const Rt = rotZYX(ta, tb, tc);
  // Flansch aus TCP (Umkehr von fkAll): R6 = Rt * R_tcp^T, p6 = P - R6 * R_fc * tcp
  const R_fc = mRy(Math.PI/2);
  const R_tcp = mMul(R_fc, rotZYX(TCP_DEF.a, TCP_DEF.b, TCP_DEF.c));
  const R6 = mMul(Rt, mT(R_tcp));
  const pt = mVec(R6, mVec(R_fc, [TCP_DEF.x, TCP_DEF.y, TCP_DEF.z]));   // wie fkAll: rot * R_fc * tcp
  const p6 = [tx - pt[0], ty - pt[1], tz - pt[2]];
  // Handwurzel p4 = p6 - (L5+L6) * x-Achse(R6)
  const L56 = J[4].off[0] + J[5].off[0];
  const px = p6[0] - L56*R6[0][0], py = p6[1] - L56*R6[1][0], pz = p6[2] - L56*R6[2][0];
  const o1 = J[0].off, o2 = J[1].off, o3 = J[2].off;
  const ys = o1[1] + o2[1] + o3[1];                 // konstanter Y-Versatz im A1-System
  const wx = o3[0] + J[3].off[0], wz = o3[2];       // Unterarm bis Handwurzel (x,z)
  const r2 = px*px + py*py - ys*ys;
  if (r2 < 0) return [];
  const out = [];
  for (const sx of [1, -1]) {                        // vorne / über Kopf
    const Xp = sx*Math.sqrt(r2);
    const th1 = Math.atan2(py, px) - Math.atan2(ys, Xp);
    const Qx = Xp - o1[0], Qz = pz - o1[2];
    const A = o2[0]*wx + o2[2]*wz, B = o2[0]*wz - o2[2]*wx;
    const K = (Qx*Qx + Qz*Qz - (o2[0]*o2[0] + o2[2]*o2[2]) - (wx*wx + wz*wz)) / 2;
    const AB = Math.hypot(A, B);
    if (AB < 1e-9 || Math.abs(K) > AB*(1+1e-12)) continue;
    const base = Math.atan2(B, A), dlt = Math.acos(Math.max(-1, Math.min(1, K/AB)));
    for (const th3 of [base + dlt, base - dlt]) {    // Ellbogen oben / unten
      const c3 = Math.cos(th3), s3 = Math.sin(th3);
      const ux = o2[0] + wx*c3 + wz*s3, uz = o2[2] - wx*s3 + wz*c3;
      const th2 = Math.atan2(uz, ux) - Math.atan2(Qz, Qx);
      const R3 = mMul(mRz(th1), mMul(mRy(th2), mRy(th3)));
      const M = mMul(mT(R3), R6);                    // = Rx(th4) Ry(th5) Rx(th6)
      const c5 = Math.max(-1, Math.min(1, M[0][0]));
      for (const sw of [1, -1]) {                    // Handgelenk normal / Flip
        const th5 = sw*Math.acos(c5), s5 = Math.sin(th5);
        let th4, th6;
        if (Math.abs(s5) > 1e-9) {
          th4 = Math.atan2(M[1][0]/s5, -M[2][0]/s5);
          th6 = Math.atan2(M[0][1]/s5,  M[0][2]/s5);
        } else {                                     // Singularität: A4+A6 gekoppelt, A4 = 0
          th4 = 0;
          th6 = Math.atan2(-M[1][2], M[1][1]) * (c5 > 0 ? 1 : -1);
        }
        out.push([th1, th2, th3, th4, th5, th6].map((v, i) => v*R2D*S[i]));
      }
    }
  }
  return out;
}

// Nächstgelegene gültige Lösung zu prevQ (inkl. ±360°-Varianten); null wenn nicht analytisch lösbar
// strict: keine Ausweichkopie ±360° in die Achsgrenzen (Achse müsste dafür zurückdrehen) → ok=false
function solveIKAnalytic(tx, ty, tz, ta, tb, tc, prevQ, strict) {
  if (!ikAnalyticSupported()) return null;
  prevQ = prevQ || jointAngles;
  let best = null, bestCost = Infinity;
  for (const sol of ikAnalyticAll(tx, ty, tz, ta, tb, tc)) {
    const q = [];
    let inLim = true;
    for (let i = 0; i < 6; i++) {
      const lo = JOINTS_DEF[i].min, hi = JOINTS_DEF[i].max;
      let v = sol[i] + 360*Math.round((prevQ[i] - sol[i]) / 360);   // nächstgelegene 360°-Kopie
      if ((v < lo || v > hi) && strict) inLim = false;
      else if (v < lo || v > hi) {                                   // sonst nächste Kopie in den Grenzen
        let alt = null;
        for (let k = -2; k <= 2; k++) { const w = sol[i] + 360*k; if (w >= lo && w <= hi && (alt === null || Math.abs(w - prevQ[i]) < Math.abs(alt - prevQ[i]))) alt = w; }
        if (alt === null) inLim = false; else v = alt;
      }
      q.push(v);
    }
    // Abstand über den kürzesten Winkelweg: prevQ kann stetig fortgeführt außerhalb ±360° liegen
    // (Simulationstabelle); sonst wirkt die nahe Lösung 360° entfernt und die Über-Kopf-Stellung gewinnt
    let cost = 0;
    for (let i = 0; i < 6; i++) cost += shortestAngleDiff(prevQ[i], q[i])**2;
    if (!inLim) cost += 1e9;
    if (cost < bestCost) { bestCost = cost; best = {q, inLim}; }
  }
  if (!best) return null;
  // Verifikation per FK
  const fk = fkTCP_full(best.q);
  const eP = Math.hypot(fk.pos[0]-tx, fk.pos[1]-ty, fk.pos[2]-tz);
  const eO = Math.hypot(...logOriErr(rotZYX(ta, tb, tc), fk.rot));
  if (!(eP < 0.01 && eO < 0.01)) return null;
  return { angles: best.q, score: eP + eO, ok: best.inLim };
}

// Analytisch, sonst numerisch (gleiche Rückgabe {angles, score, ok})
function ikFastA(x,y,z,a,b,c,init)    { return solveIKAnalytic(x,y,z,a,b,c,init) || solveIKFast(x,y,z,a,b,c,init); }
function ikRobustA(x,y,z,a,b,c,init)  { return solveIKAnalytic(x,y,z,a,b,c,init) || solveIK(x,y,z,a,b,c,init); }
function ikPreciseA(x,y,z,a,b,c,init) { return solveIKAnalytic(x,y,z,a,b,c,init) || solveIKPrecise(x,y,z,a,b,c,init); }

// ── IK-Wrapper (öffentliche API bleibt identisch) ───────────────────────────

function solveIK(tx, ty, tz, ta, tb, tc, initAngles) {
  return solveLM({
    tp:[tx,ty,tz], Rt:rotZYX(ta,tb,tc),
    starts:[
      initAngles || jointAngles.slice(),
      [0,-90,90,0,0,0],[0,-90,90,0,-45,0],[0,-90,90,0,-90,0],
      [0,-90,90,-90,-45,0],[0,-90,90,90,-45,0],
      [0,-120,110,0,-45,0],[0,-60,60,0,-45,0],
    ],
    dt:0.3, lam:0.5, tolP:1.0, tolO:1.0,
    maxIter:300, stepMax:2.0, stepScale:10.0,
    earlyStop:1.5, okThresh:30,
  });
}
// ═══════════════════════════════════════════════════
// KRL PARSER (original vom KUKA Simulator)
// ═══════════════════════════════════════════════════
const KW=new Set(['LIN','PTP','SLIN','CIRC','DECL','IF','ELSE','ENDIF','FOR','ENDFOR',
  'WHILE','ENDWHILE','LOOP','ENDLOOP','DEF','END','DEFFCT','ENDFCT','RETURN','WAIT','HALT']);
let parsedData={positions:[],finalState:{variables:{},digitalIn:{},digitalOut:{},analogOut:{},analogIn:{}}};

function parsePos(str){
  const p={X:0,Y:0,Z:0,A:0,B:0,C:0,S:null,T:null};
  let m;const rx=/([XYZABCST])\s+([-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?)/g;
  while((m=rx.exec(str))!==null)if(m[1]in p)p[m[1]]=parseFloat(m[2]);
  return p;
}
function parseVal(s){
  s=s.trim().replace(/;.*$/,'').trim();
  if(/^TRUE$/i.test(s))return true;if(/^FALSE$/i.test(s))return false;
  const n=Number(s);return(!isNaN(n)&&s!=='')?n:s.replace(/^"|"$/g,'');
}
// ── BASE-Transformation (KUKA): Welt <-> base-relativ ───────────────
// Punkte werden in Weltkoordinaten gespeichert; KRL-{X..}-Werte sind base-relativ.
// BASE_DATA[n]: n>=1 -> baseList[n-1]; n<1 oder unbekannt -> Welt-Nullframe (Identitaet).
function _baseByN(n){
  if (!n || n < 1) return null;
  return baseList[n-1] || null;
}
function _rotZYX(A,B,C){ // KUKA: R = Rz(A)Ry(B)Rx(C), flat 3x3 (row-major)
  var d=Math.PI/180, a=A*d,b=B*d,c=C*d;
  var ca=Math.cos(a),sa=Math.sin(a),cb=Math.cos(b),sb=Math.sin(b),cc=Math.cos(c),sc=Math.sin(c);
  return [ ca*cb, ca*sb*sc-sa*cc, ca*sb*cc+sa*sc,
           sa*cb, sa*sb*sc+ca*cc, sa*sb*cc-ca*sc,
           -sb,   cb*sc,          cb*cc ];
}
function _eulZYX(R){ // flat 3x3 -> {A,B,C} deg
  var r2d=180/Math.PI, A,B,C, sy=Math.sqrt(R[0]*R[0]+R[3]*R[3]);
  if (sy>1e-9){ A=Math.atan2(R[3],R[0]); B=Math.atan2(-R[6],sy); C=Math.atan2(R[7],R[8]); }
  else        { A=Math.atan2(-R[1],R[4]); B=Math.atan2(-R[6],sy); C=0; }
  return { A:A*r2d, B:B*r2d, C:C*r2d };
}
function _poseT(p){ var R=_rotZYX(p.A||0,p.B||0,p.C||0);
  return [R[0],R[1],R[2],p.X||0, R[3],R[4],R[5],p.Y||0, R[6],R[7],R[8],p.Z||0, 0,0,0,1]; }
function _mul4(M,N){ var o=new Array(16).fill(0);
  for(var r=0;r<4;r++)for(var c=0;c<4;c++)for(var k=0;k<4;k++)o[r*4+c]+=M[r*4+k]*N[k*4+c]; return o; }
function _fromT(M){ var R=[M[0],M[1],M[2],M[4],M[5],M[6],M[8],M[9],M[10]], e=_eulZYX(R);
  return { X:M[3], Y:M[7], Z:M[11], A:e.A, B:e.B, C:e.C }; }
// Welt -> base-relativ (fuer Anzeige/Ausgabe)
function _baseMat(b){ return _poseT({X:b.x||0,Y:b.y||0,Z:b.z||0,A:b.a||0,B:b.b||0,C:b.c||0}); } // BASE-Felder sind klein geschrieben
// base-relativ -> Welt (fuer Speicherung/Eingabe)
function baseToWorld(p, baseN){ var b=_baseByN(baseN);
  if(!b) return { X:p.X, Y:p.Y, Z:p.Z, A:p.A, B:p.B, C:p.C };
  return _fromT(_mul4(_baseMat(b), _poseT(p))); }
function parseKRL(code){
  const lines=code.split(/\r?\n/);const steps=[],positions=[],vars={},din={},dout={},anout={},anin={};
  let _velCP = 0.167; // m/s aktuelle Geschwindigkeit
  function snap(){return{variables:{...vars},digitalIn:{...din},digitalOut:{...dout},analogOut:{...anout},analogIn:{...anin}};}
  function pushStep(ln,type,extra){steps.push({lineNum:ln,type,...extra,snapshot:snap()});}
  let curBase = 0; // aktiver BASE (BASE_DATA[n]); 0 = Welt-Nullframe
  let curTool = 1; // aktives TOOL (TOOL_DATA[n]); 1 = Standard
  // KRL-{X..} ist die Code-Koordinate -> als rel merken (Welt = rel + aktueller BASE-Offset, separat angewandt)
  function W(o){ o.rel={X:o.X,Y:o.Y,Z:o.Z,A:o.A,B:o.B,C:o.C}; o.baseN=curBase; o.toolN=curTool; return o; }
  for(let ln=0;ln<lines.length;ln++){
    let raw=lines[ln];const ci=raw.indexOf(';');if(ci===0)continue;
    let line=(ci>0?raw.slice(0,ci):raw).trim();if(!line)continue;
    const circM=line.match(/^CIRC\s+(\{[^}]+\})\s*,\s*(\{[^}]+\})/i);
    if(circM){const s=snap();const pi=positions.length;
      positions.push(W({type:'CIRC_AUX',...parsePos(circM[1].slice(1,-1)),lineNum:ln,snapshot:s}));
      positions.push(W({type:'CIRC',...parsePos(circM[2].slice(1,-1)),lineNum:ln,snapshot:s}));
      pushStep(ln,'move',{posIdx:pi+1,label:'CIRC'});continue;}
    // CIRC_EXTRUDE ({Hilfspunkt},{Zielpunkt},N) — Extrusionsparameter wird ignoriert
    const circExtM=line.match(/^CIRC_EXTRUDE\s*\(\s*(\{[^}]+\})\s*,\s*(\{[^}]+\})\s*(?:,[^)]*)?\)/i);
    if(circExtM){const s=snap();const pi=positions.length;
      positions.push(W({type:'CIRC_AUX',...parsePos(circExtM[1].slice(1,-1)),lineNum:ln,snapshot:s}));
      positions.push(W({type:'CIRC',...parsePos(circExtM[2].slice(1,-1)),lineNum:ln,snapshot:s}));
      pushStep(ln,'move',{posIdx:pi+1,label:'CIRC'});continue;}
    const circP=line.match(/^CIRC\s+(\{[^}]+\})\s*,?\s*$/i);
    if(circP){let next='';
      for(let j=ln+1;j<Math.min(ln+3,lines.length);j++){next=lines[j].trim().replace(/^;.*/,'');if(next)break;}
      const endM=next.match(/^\{([^}]+)\}/);
      if(endM){const s=snap(),pi=positions.length;
        positions.push(W({type:'CIRC_AUX',...parsePos(circP[1].slice(1,-1)),lineNum:ln,snapshot:s}));
        positions.push(W({type:'CIRC',...parsePos(endM[1]),lineNum:ln,snapshot:s}));
        pushStep(ln,'move',{posIdx:pi+1,label:'CIRC'});}continue;}
    const moveM=line.match(/^(LIN|PTP|SLIN)\s+\{([^}]+)\}/i);
    if(moveM){const pi=positions.length;
      const moveType=moveM[1].toUpperCase();
      const parsedMove=parsePos(moveM[2]);
      // PTP mit Achswinkeln (A1..A6) → als ptpAngles speichern
      const axisM=moveM[2].match(/A1\s*([-\d.]+)/i);
      if(moveType==='PTP' && axisM) {
        const aMatch=moveM[2].match(/A([1-6])\s*([-\d.]+)/gi)||[];
        const ptpQ=[0,-90,90,0,0,0];
        aMatch.forEach(function(s){const m2=s.match(/A([1-6])\s*([-\d.]+)/i);if(m2)ptpQ[+m2[1]-1]=parseFloat(m2[2]);});
        pushStep(ln,'ptpAxis',{angles:ptpQ.slice()});continue;
      }
      positions.push(W({type:moveType,...parsedMove,lineNum:ln,velCP:_velCP,snapshot:snap()}));
      pushStep(ln,'move',{posIdx:pi,label:moveType});continue;}
    // LIN_EXTRUDE({X ...},N) — Extrusionsparameter wird ignoriert
    const linExtM=line.match(/^LIN_EXTRUDE\s*\(\s*\{([^}]+)\}\s*(?:,[^)]*)?\)/i);
    if(linExtM){const pi=positions.length;
      positions.push(W({type:'LIN',...parsePos(linExtM[1]),lineNum:ln,velCP:_velCP,snapshot:snap()}));
      pushStep(ln,'move',{posIdx:pi,label:'LIN'});continue;}
    let m;
    // ── Systemvariablen mit vollständigen Daten ──────────────────────────
    if((m=line.match(/^\$TOOL\s*=\s*TOOL_DATA\[(\d+)\]/i))){curTool=+m[1];pushStep(ln,'tool',{n:+m[1]});continue;}
    if((m=line.match(/^\$BASE\s*=\s*BASE_DATA\[(\d+)\]/i))){curBase=+m[1];pushStep(ln,'base',{n:+m[1]});continue;}
    if((m=line.match(/^\$VEL\.CP\s*=\s*([\d.]+)/i))){_velCP=parseFloat(m[1])||0.167;pushStep(ln,'velcp',{v:_velCP});continue;}
    if((m=line.match(/^\$VEL\.PTP\s*=\s*([\d.]+)/i))){pushStep(ln,'velptp',{v:parseFloat(m[1])});continue;}
    if((m=line.match(/^\$ACC\.CP\s*=\s*([\d.]+)/i))){pushStep(ln,'acccp',{v:parseFloat(m[1])});continue;}
    if((m=line.match(/^\$advance\s*=\s*([\d.]+)/i))){pushStep(ln,'advance',{v:parseFloat(m[1])});continue;}
    if(/^HALT$/i.test(line)){pushStep(ln,'halt',{});continue;}
    if(/^BRAKE$/i.test(line)){pushStep(ln,'brake',{});continue;}
    if((m=line.match(/^WAIT\s+SEC\s+([\d.]+)/i))){pushStep(ln,'wait',{t:parseFloat(m[1])});continue;}
    if((m=line.match(/^WAIT\s+FOR\s+(.+)/i))){
      const dinM=m[1].match(/^\$IN\[(\d+)\]/i);
      if(dinM){din[+dinM[1]]=false;pushStep(ln,'din',{n:+dinM[1]});}
      else{pushStep(ln,'waitFor',{cond:m[1].trim()});}
      continue;}
    // ── I/O ──────────────────────────────────────────────────────────────
    if((m=line.match(/^\$OUT\s*\[(\d+)\]\s*=\s*(TRUE|FALSE|ON|OFF|1|0)/i))){
      const v=m[2].toUpperCase();dout[+m[1]]=v==='TRUE'||v==='ON'||v==='1';
      pushStep(ln,'dout',{n:+m[1],v:v});continue;}
    if((m=line.match(/^\$IN\s*\[(\d+)\]/i))){din[+m[1]]=false;pushStep(ln,'din',{n:+m[1]});continue;}
    if((m=line.match(/^\$ANOUT\s*\[(\d+)\]\s*=\s*([\d.\-+]+)/i))){
      const v=parseFloat(m[2]);anout[+m[1]]=isNaN(v)?0:Math.max(-10,Math.min(10,v));
      pushStep(ln,'aout',{n:+m[1],v:v});continue;}
    if((m=line.match(/^\$ANIN\s*\[(\d+)\]/i))){anin[+m[1]]=0;pushStep(ln,'ain',{n:+m[1]});continue;}
    // ── Variablen ─────────────────────────────────────────────────────────
    if((m=line.match(/^DECL\s+(INT|REAL|BOOL|CHAR)\s+(\w+)(?:\s*=\s*(.+))?/i))||
       (m=line.match(/^(INT|REAL|BOOL|CHAR)\s+(\w+)(?:\s*=\s*(.+))?/i))){
      const vt=m[1].toUpperCase(),vn=m[2],vv=m[3]?m[3].trim():'';
      vars[vn]=parseVal(vv||'0');pushStep(ln,'var',{varType:vt,name:vn,val:vv||''});continue;}
    if((m=line.match(/^([A-Za-z_]\w*)\s*=\s*(.+)/))){
      if(!KW.has(m[1].toUpperCase())){
        const _target=m[1],_expr=m[2].trim();
        try{const _fn=new Function(...Object.keys(vars),'return ('+_expr+')');vars[_target]=_fn(...Object.values(vars));}
        catch(e){vars[_target]=parseVal(_expr)||0;}
        pushStep(ln,'calc',{target:_target,expr:_expr});continue;}}
    pushStep(ln,'other',{raw:line});
  }
  return{steps,positions,finalState:{variables:{...vars},digitalIn:{...din},digitalOut:{...dout},analogOut:{...anout},analogIn:{...anin}}};
}
// ═══════════════════════════════════════════════════
// THREE.JS SETUP (Z-up scene = KUKA Robot CS)
// ═══════════════════════════════════════════════════
const canvas=document.getElementById('c3d');
const _codeEl=document.getElementById('code-input'), _gutEl=document.getElementById('gutter');
const renderer=new THREE.WebGLRenderer({canvas,antialias:true});
renderer.setPixelRatio(window.devicePixelRatio);
const scene=new THREE.Scene();
scene.background=new THREE.Color(0x1b1b1d);

const perspCam=new THREE.PerspectiveCamera(50,1,1,80000);
perspCam.up.set(0,0,1);
const activeCam=perspCam;

// Lights
scene.add(new THREE.AmbientLight(0xffffff, 0.85));
const sun=new THREE.DirectionalLight(0xffffff,0.8);
sun.position.set(1000,1000,2000);scene.add(sun);
const fill=new THREE.DirectionalLight(0xffffff,0.4);fill.position.set(-1000,-500,1000);scene.add(fill);

// Grid (horizontal, Z=0)
const grid=new THREE.GridHelper(8000,40,0x77819b,0x494e5c);
grid.rotation.x=Math.PI/2;
scene.add(grid);

// World origin frame
function wArrow(d,col,len,org){scene.add(new THREE.ArrowHelper(new THREE.Vector3(...d).normalize(),new THREE.Vector3(...org),len,col,len*.18,len*.09));}
wArrow([1,0,0],0xff3333,300,[0,0,0]);
wArrow([0,1,0],0x33ff33,300,[0,0,0]);
wArrow([0,0,1],0x3377ff,300,[0,0,0]);

// Groups
const posGrp=new THREE.Group();scene.add(posGrp);
const pathGrp=new THREE.Group();scene.add(pathGrp);
const robotGrp=new THREE.Group();scene.add(robotGrp);
const markerGrp=new THREE.Group();scene.add(markerGrp);
const markerVisuals=new THREE.Group();markerGrp.add(markerVisuals);
{
  const sp=new THREE.Mesh(new THREE.SphereGeometry(28,12,12),new THREE.MeshBasicMaterial({color:0x55aef4,transparent:true,opacity:.9}));
  markerVisuals.add(sp);
  const ring=new THREE.Mesh(new THREE.TorusGeometry(36,5,6,24),new THREE.MeshBasicMaterial({color:0x32c6da}));
  ring.rotation.x=Math.PI/2;markerVisuals.add(ring);
  for(const[d,c]of[[[1,0,0],0xff6666],[[0,1,0],0x66ff66],[[0,0,1],0x6699ff]])
    markerVisuals.add(new THREE.ArrowHelper(new THREE.Vector3(...d),new THREE.Vector3(),190,c,38,19));
  markerGrp.visible=false;
}
const TYPE_COL={LIN:0xf05500,PTP:0xffaa00,SLIN:0x00aaff,CIRC:0xaa44ff,CIRC_AUX:0x445566};
const frameSize=40, sphereSize=7, pathLineWidth=2;
function kukaEuler(A,B,C){const r=THREE.MathUtils.degToRad;return new THREE.Euler(r(C),r(B),r(A),'ZYX');}

function makeFrame(pos){
  const grp=new THREE.Group();
  for(const[d,c]of[[[1,0,0],0xff4444],[[0,1,0],0x44ff44],[[0,0,1],0x4488ff]])
    grp.add(new THREE.ArrowHelper(new THREE.Vector3(...d),new THREE.Vector3(),frameSize,c,frameSize*.2,frameSize*.1));
  grp.add(new THREE.Mesh(new THREE.SphereGeometry(sphereSize,8,8),new THREE.MeshBasicMaterial({color:TYPE_COL[pos.type]!==undefined?TYPE_COL[pos.type]:0xffffff})));
  grp.position.set(pos.X,pos.Y,pos.Z);
  grp.setRotationFromEuler(kukaEuler(pos.A,pos.B,pos.C));
  grp.userData.posIdx=-1;return grp;
}

// ═══════════════════════════════════════════════════
// ROBOT 3D MODEL
// ═══════════════════════════════════════════════════
const robotColor=0xcc4400, jointColor=0xe8a020;

function buildCylinder(from,to,radius,color){
  const v1=new THREE.Vector3(...from),v2=new THREE.Vector3(...to);
  const dir=new THREE.Vector3().subVectors(v2,v1);
  const len=dir.length();if(len<1)return null;
  const geo=new THREE.CylinderGeometry(radius,radius,len,8);
  const mat=new THREE.MeshPhongMaterial({color, shininess:80, specular:0x444444});
  const mesh=new THREE.Mesh(geo,mat);
  const mid=new THREE.Vector3().addVectors(v1,v2).multiplyScalar(.5);
  mesh.position.copy(mid);
  const up=new THREE.Vector3(0,1,0);
  const dn=dir.clone().normalize();
  const ax=new THREE.Vector3().crossVectors(up,dn).normalize();
  const ang=Math.acos(Math.max(-1,Math.min(1,up.dot(dn))));
  if(ax.length()>0.001)mesh.quaternion.setFromAxisAngle(ax,ang);
  else if(dn.y<0)mesh.rotation.z=Math.PI;
  return mesh;
}

// ── STL-Achsmodelle ───────────────────────────────────────────
const axisSTLMeshes  = [null,null,null,null,null,null];
const axisPivots     = [];   // THREE.Group pro Achse (Kette)
let pedestalMesh     = null;
let toolMesh         = null;
let stlRefAngles     = [0,-90,90,0,0,0];  // Pose, in der die STL-Dateien modelliert sind
// Build pivot chain once; re-build when JOINTS_DEF changes
function buildPivotChain() {
  axisPivots.forEach(function(g){ if(g.parent) g.parent.remove(g); });
  axisPivots.length = 0;
  let parent = robotGrp;
  for (let i = 0; i < 6; i++) {
    const g = new THREE.Group();
    // Each pivot is offset by its joint's link offset IN the parent's local frame
    if (i === 0) g.position.set(0, 0, 0);
    else { const o = JOINTS_DEF[i-1].off; g.position.set(o[0],o[1],o[2]); }
    parent.add(g);
    axisPivots.push(g);
    parent = g;
  }
}
// Update pivot rotations from current joint angles
function updatePivotRotations(angles) {
  const r = Math.PI / 180;
  for (let i = 0; i < 6; i++) {
    if (!axisPivots[i]) continue;
    const a = angles[i] * FK_SIGNS[i] * r;
    axisPivots[i].rotation.set(0,0,0);
    switch(JOINTS_DEF[i].axis) {
      case 'Rz': axisPivots[i].rotation.z = a; break;
      case 'Ry': axisPivots[i].rotation.y = a; break;
      case 'Rx': axisPivots[i].rotation.x = a; break;
    }
  }
}

// 3×3 Matrix → THREE.Quaternion
function _mat3ToQuat(R) {
  const m4 = new THREE.Matrix4();
  m4.set(R[0][0],R[0][1],R[0][2],0,
         R[1][0],R[1][1],R[1][2],0,
         R[2][0],R[2][1],R[2][2],0,
         0,0,0,1);
  return new THREE.Quaternion().setFromRotationMatrix(m4);
}
// Achsen-STL Posen aus FK aktualisieren
function _applySTLMeshPoses(angles) {
  const ref  = fkAll(stlRefAngles);
  const curr = fkAll(angles);
  for (let i = 0; i < 6; i++) {
    const applyToMesh = function(mesh) {
      if (!mesh) return;
      const R  = mMul(curr.rots[i+1], mT(ref.rots[i+1]));
      const rp = mVec(R, ref.pts[i]);
      mesh.quaternion.copy(_mat3ToQuat(R));
      mesh.position.set(curr.pts[i][0]-rp[0], curr.pts[i][1]-rp[1], curr.pts[i][2]-rp[2]);
    };
    applyToMesh(axisSTLMeshes[i]);
    ((window._axisExtraMeshes||{})[i]||[]).forEach(applyToMesh);
  }
}

// Werkzeug-STL Pose aus FK (Flansch A6 + Rz180°-Offset)
function _applyToolMeshPose(angles) {
  if (!toolMesh) return;
  const fkC  = fkAll(angles);
  const Rt   = mMul(fkC.rots[6], [[-1,0,0],[0,-1,0],[0,0,1]]); // × Rz(180°)
  toolMesh.quaternion.copy(_mat3ToQuat(Rt));
  toolMesh.position.set(fkC.pts[6][0], fkC.pts[6][1], fkC.pts[6][2]);
}

// Skelett-Visualisierung aufbauen
function _buildSkeleton(angles) {
  const fk   = fkAll(angles);
  const pts  = fk.pts;
  const linkR = [28,20,16,12,8,6];
  const jntR  = [40,38,30,24,20,16];
  // Roboter aus der Bibliothek mit Achsmodellen (STL): Skelett ausblenden, nur TCP-Achsen zeigen
  const hasSTL = axisSTLMeshes.some(Boolean);
  for (let i = 0; i < 7 && !hasSTL; i++) {
    if (!pts[i] || !pts[i+1]) continue;
    const cyl = buildCylinder(pts[i], pts[i+1], linkR[i] || 5, robotColor);
    if (cyl) robotGrp.add(cyl);
  }
  for (let i = 1; i <= 6 && !hasSTL; i++) {
    const s = new THREE.Mesh(
      new THREE.SphereGeometry(jntR[i-1] || 10, 12, 8),
      new THREE.MeshPhongMaterial({color:jointColor, shininess:120, specular:0x666666})
    );
    s.position.set(pts[i][0], pts[i][1], pts[i][2]);
    robotGrp.add(s);
  }
  if (pts[7]) {
    const tcp = pts[7], R = fk.tcp_rot, aLen = 150;
    for (const [[cx,cy,cz], col] of [[[1,0,0],0xff4444],[[0,1,0],0x44ff44],[[0,0,1],0x4488ff]]) {
      robotGrp.add(new THREE.ArrowHelper(
        new THREE.Vector3(...mVec(R,[cx,cy,cz])).normalize(),
        new THREE.Vector3(...tcp), aLen, col, aLen*.2, aLen*.1
      ));
    }
  }
}

function buildRobotModel(angles) {
  if (!axisPivots.length) buildPivotChain();
  robotGrp.children.filter(ch => !axisPivots.includes(ch)).forEach(ch => {
    robotGrp.remove(ch);
    // Skelett wird pro Bild neu aufgebaut: eigene Geometrien/Materialien freigeben
    // (ArrowHelper teilt seine Geometrie in three.js — dort nur Materialien)
    if (ch.isMesh) { ch.geometry.dispose(); ch.material.dispose(); }
    else if (ch.line) { ch.line.material.dispose(); ch.cone.material.dispose(); }
  });
  updatePivotRotations(angles);
  _applySTLMeshPoses(angles);
  _applyToolMeshPose(angles);
  _buildSkeleton(angles);
}

function downloadFile(content, filename) {
  const a = document.createElement('a');
  a.href = 'data:text/plain;charset=utf-8,' + encodeURIComponent(content);
  a.download = filename;
  document.body.appendChild(a);
  a.click();
  document.body.removeChild(a);
}

// ── Roboter-Konfiguration (Kinematik, STLs, TCP, BASE) aus RobLib-ZIP ──
// Werkzeuge: $TOOL=TOOL_DATA[n] -> tcpList[n-1]; Basen: $BASE=BASE_DATA[n] -> baseList[n-1]
var tcpList  = [{ x:364.5, y:0, z:46.5, a:0, b:90, c:0, name:'TCP 1' }];
var baseList = [{ x:0, y:0, z:0, a:0, b:0, c:0, name:'BASE 1' }];

function _stlMesh(buf, color, extra) {
  var geo = stlLoader.parse(buf); geo.computeVertexNormals();
  return new THREE.Mesh(geo, new THREE.MeshPhongMaterial(Object.assign({color:color, shininess:80}, extra||{})));
}
function _removeMesh(m) { if (m) { scene.remove(m); m.geometry.dispose(); } }

function applyKinematicData(data, stlBuffers) {
  stlBuffers = stlBuffers || {};
  if (data.joints) {
    while (JOINTS_DEF.length < data.joints.length)
      JOINTS_DEF.push({name:'A'+(JOINTS_DEF.length+1), off:[0,0,0], min:-180, max:180, axis:'Ry'});
    data.joints.forEach((j, i) => {
      if (!JOINTS_DEF[i]) return;
      if (j.offset) { JOINTS_DEF[i].off[0]=j.offset.x||0; JOINTS_DEF[i].off[1]=j.offset.y||0; JOINTS_DEF[i].off[2]=j.offset.z||0; }
      if (j.min !== undefined) JOINTS_DEF[i].min = j.min;
      if (j.max !== undefined) JOINTS_DEF[i].max = j.max;
      if (j.axis) JOINTS_DEF[i].axis = j.axis;
      if (j.name) JOINTS_DEF[i].name = j.name;
    });
  }
  if (data.stlRefAngles) {
    stlRefAngles = data.stlRefAngles;
    jointAngles = stlRefAngles.slice();   // Kanonenstellung
  }
  if (data.stlFiles) {
    Object.entries(data.stlFiles).forEach(function(entry) {
      var key = entry[0], val = entry[1];
      if (!val) return;
      var axIdx = parseInt(key.replace('A','')) - 1;
      if (axIdx < 0 || axIdx > 5) return;
      var parts = Array.isArray(val) ? val : [val];   // ältere ZIPs: ein Objekt statt Liste
      _removeMesh(axisSTLMeshes[axIdx]); axisSTLMeshes[axIdx] = null;
      if (!window._axisExtraMeshes) window._axisExtraMeshes = {};
      (window._axisExtraMeshes[axIdx]||[]).forEach(_removeMesh);
      window._axisExtraMeshes[axIdx] = [];
      var keyLower = key.toLowerCase();
      parts.forEach(function(p, pi) {
        var fname = (p.name||'').replace(/\.(stl|osd)$/i,'').toLowerCase();
        var buf = stlBuffers[fname] || (pi===0 ? stlBuffers[keyLower]||stlBuffers['axis'+keyLower] : null);
        if (!buf) return;
        var mesh = _stlMesh(buf, p.color||0xe8a020);
        scene.add(mesh);
        if (pi===0) axisSTLMeshes[axIdx] = mesh;
        else window._axisExtraMeshes[axIdx].push(mesh);
      });
    });
  }
  if (data.sceneModels) {
    var sm = data.sceneModels;
    if (sm.pedestal) {
      var pname = (sm.pedestal.name || 'podest').toLowerCase();
      // Fallback: Schlüssel mit 'base' oder 'podest'
      var pbuf = stlBuffers[pname] ||
        (Object.entries(stlBuffers).find(function(e){ return /base|podest/i.test(e[0]); })||[])[1];
      if (pbuf) {
        _removeMesh(pedestalMesh);
        pedestalMesh = _stlMesh(pbuf, 0x334455, {shininess:40});
        scene.add(pedestalMesh);
      }
    }
    if (sm.tool) {
      var tbuf = stlBuffers[(sm.tool.name || 'tool1_tcp').toLowerCase()];
      if (tbuf) {
        _removeMesh(toolMesh);
        toolMesh = _stlMesh(tbuf, 0xdd9944, {side:THREE.DoubleSide, specular:0x666666});
        scene.add(toolMesh);
      }
    }
  }
  if (data.tcpList && data.tcpList.length) tcpList = data.tcpList;
  if (data.baseList && data.baseList.length) baseList = data.baseList;
  baseNavTo(0);
  if (data.tcp) {
    var t = data.tcp;
    tcpList = [{ x:t.x||0, y:t.y||0, z:t.z||0, a:t.a||0, b:t.b||0, c:t.c||0, name:'TCP 1' }];
  }
  _applyToolToTCPDEF(1);
  buildPivotChain();
  buildRobotModel(jointAngles);
  setStatus('paused', 'Kinematik geladen');
}
function toggleLeftPanel() {
  const ep = document.querySelector('.ep');
  const btn = document.getElementById('ep-toggle-btn');
  if (!ep) return;
  const closed = ep._panelClosed;
  if (closed) {
    ep.style.width = ep._savedW || '380px';
    ep.style.minWidth = '';
    ep.style.overflow = '';
    ep.style.padding = '';
    ep._panelClosed = false;
    if (btn) btn.textContent = '◀';
  } else {
    ep._savedW = ep.offsetWidth + 'px';
    ep.style.width = '0';
    ep.style.minWidth = '0';
    ep.style.overflow = 'hidden';
    ep.style.padding = '0';
    ep._panelClosed = true;
    if (btn) btn.textContent = '▶';
  }
  window.dispatchEvent(new Event('resize'));
}

// Stellt sicher, dass baseList einen Eintrag fuer jede im Programm referenzierte BASE_DATA[n] hat
// (fehlende Bases = Identitaet).
function ensureBasesForProgram() {
  var maxN = 0;
  parsedData.positions.forEach(function(p){ if (p.baseN && p.baseN > maxN) maxN = p.baseN; });
  parsedData.steps.forEach(function(s){ if (s.type==='base' && s.n > maxN) maxN = s.n; });
  var changed = false;
  while (baseList.length < maxN) {
    baseList.push({ x:0, y:0, z:0, a:0, b:0, c:0, name: 'BASE ' + (baseList.length+1) });
    changed = true;
  }
  if (changed) renderBaseNav();
}

// TCP-Eintrag fuer eine Tool-Nummer (Fallback: erster Eintrag)
function _tcpEntryForTool(toolN) {
  return tcpList[(toolN || 1) - 1] || tcpList[0] || null;
}
// TCP_DEF (per Property-Mutation, da const) auf das TCP der Tool-Nummer setzen
function _applyToolToTCPDEF(toolN) {
  var t = _tcpEntryForTool(toolN);
  if (!t) return;
  TCP_DEF.x=t.x||0; TCP_DEF.y=t.y||0; TCP_DEF.z=t.z||0; TCP_DEF.a=t.a||0; TCP_DEF.b=t.b||0; TCP_DEF.c=t.c||0;
}

// true wenn alle Punkte dasselbe Tool nutzen
function _toolsAreUniform(positions) {
  if (!positions || !positions.length) return true;
  var t0 = positions[0].toolN || 1;
  for (var i=1;i<positions.length;i++){ if ((positions[i].toolN||1) !== t0) return false; }
  return true;
}

// Aktives Tool fuer die Anzeige setzen: TCP-Offset aus tcpList umschalten
function setActiveTool(toolN) {
  if (!toolN) toolN = 1;
  _applyToolToTCPDEF(toolN);
  // STL-Umschaltung pro Tool folgt via robmodel-XML
}
// Welt = Code-Koordinate (rel) + aktueller BASE-Offset. Nur Transformation, kein Neuzeichnen.
function bakeBaseOffset() {
  parsedData.positions.forEach(function(p){
    try {
      if (!p.rel) {
        if (typeof p.X !== 'number' || !isFinite(p.X)) return; // Achs-/Nicht-Kartesisch: kein BASE-Offset
        p.rel = { X:p.X, Y:p.Y, Z:p.Z, A:p.A, B:p.B, C:p.C };
      }
      if (typeof p.rel.X !== 'number' || !isFinite(p.rel.X)) return;
      var bn = (p.baseN != null) ? p.baseN : 0; // jeder Punkt nutzt seinen eigenen aktiven BASE
      var w = baseToWorld(p.rel, bn);
      p.X=w.X; p.Y=w.Y; p.Z=w.Z; p.A=w.A; p.B=w.B; p.C=w.C;
    } catch(e) {}
  });
}

// ── BASE bearbeiten (wie RobSimul) ───────────────────────────────
// Eingabe verschiebt Bahn und Zielpunkte sofort; die IK wird beim naechsten Parsen (F9) neu berechnet.
var baseNavIdx = 0;
const baseFrameGroups = [];   // ein Koordinatensystem pro BASE-Eintrag

function makeBaseFrameGroup(name) {
  var grp = new THREE.Group();
  var sz = 150;
  [[1,0,0,0xff3333],[0,1,0,0x33ff33],[0,0,1,0x3388ff]].forEach(function(d) {
    grp.add(new THREE.ArrowHelper(new THREE.Vector3(d[0],d[1],d[2]), new THREE.Vector3(0,0,0), sz, d[3], sz*.2, sz*.1));
  });
  var cv = document.createElement('canvas');
  cv.width = 256; cv.height = 64;
  var sprite = new THREE.Sprite(new THREE.SpriteMaterial({map:new THREE.CanvasTexture(cv),depthTest:false,transparent:true,alphaTest:0.01,depthWrite:false}));
  sprite.scale.set(180,44,1);
  sprite.position.set(20,0,-10);
  grp.add(sprite);
  grp.userData.labelSprite = sprite;
  grp.userData.labelCanvas = cv;
  updateBaseFrameLabel(grp, name, false);
  return grp;
}

function updateBaseFrameLabel(grp, name, isActive) {
  var cv = grp.userData.labelCanvas;
  var ctx = cv.getContext('2d');
  ctx.clearRect(0,0,256,64);
  ctx.font = 'bold ' + (isActive ? '30' : '26') + 'px monospace';
  ctx.strokeStyle = 'rgba(0,0,0,0.85)';
  ctx.lineWidth = 5;
  ctx.lineJoin = 'round';
  ctx.strokeText(name || 'BASE', 6, 44);
  ctx.fillStyle = isActive ? '#ffffff' : '#93c5fd';
  ctx.fillText(name || 'BASE', 6, 44);
  grp.userData.labelSprite.material.map.needsUpdate = true;
}

function syncBaseFrameGroups() {
  while (baseFrameGroups.length < baseList.length) {
    var grp = makeBaseFrameGroup(baseList[baseFrameGroups.length].name);
    scene.add(grp);
    baseFrameGroups.push(grp);
  }
  while (baseFrameGroups.length > baseList.length) scene.remove(baseFrameGroups.pop());
  baseList.forEach(function(b, i) {
    var grp = baseFrameGroups[i], r = Math.PI/180, isActive = (i === baseNavIdx);
    grp.position.set(b.x||0, b.y||0, b.z||0);
    grp.rotation.set((b.c||0)*r, (b.b||0)*r, (b.a||0)*r, 'ZYX');
    if (!b.name) b.name = 'BASE '+(i+1);
    updateBaseFrameLabel(grp, b.name, isActive);
    grp.traverse(function(obj) {
      if (obj.material) { obj.material.opacity = isActive ? 1.0 : 0.35; obj.material.transparent = !isActive; }
    });
  });
  requestRender();
}

function baseGetInputs() {
  var v = function(k){ return parseFloat(document.getElementById('base-'+k).value) || 0; };
  return { x:v('x'), y:v('y'), z:v('z'), a:v('a'), b:v('b'), c:v('c') };
}
function baseSetInputs(b) {
  ['x','y','z','a','b','c'].forEach(function(k) { document.getElementById('base-'+k).value = b && b[k] !== undefined ? b[k] : 0; });
}

function renderBaseNav() {
  syncBaseFrameGroups();
  var n = baseList.length, i = baseNavIdx;
  var name = (n && baseList[i]) ? baseList[i].name || '' : '';
  document.getElementById('base-nav-label').textContent = n ? (i+1)+' / '+n+(name?' · '+name:'') : '—';
  var dis = function(id, d) { document.getElementById(id).disabled = d; };
  dis('base-nav-first', i<=0||!n);
  dis('base-nav-prev',  i<=0||!n);
  dis('base-nav-next',  i>=n-1);
  dis('base-nav-last',  i>=n-1);
  dis('base-nav-del',   !n);
}

function baseNavTo(idx) {
  if (!baseList.length) return;
  baseNavIdx = Math.max(0, Math.min(idx, baseList.length - 1));
  var entry = baseList[baseNavIdx];
  if (!entry.name) entry.name = 'BASE ' + (baseNavIdx+1);
  baseSetInputs(entry);
  document.getElementById('base-name').value = entry.name;
  renderBaseNav();
}

function baseAddCurrent() {
  var b = baseGetInputs();
  b.name = 'BASE ' + (baseList.length + 1);
  baseList.push(b);
  baseNavTo(baseList.length - 1);
}

function baseDelCurrent() {
  if (!baseList.length) return;
  baseList.splice(baseNavIdx, 1);
  if (baseList.length) baseNavTo(Math.min(baseNavIdx, baseList.length - 1));
  else { baseNavIdx = 0; renderBaseNav(); }
  applyBase();
}

function baseNameChanged(val) {
  if (!baseList[baseNavIdx]) return;
  baseList[baseNavIdx].name = val;
  renderBaseNav();
}

function updateBaseDef() {
  var b = baseGetInputs(), e = baseList[baseNavIdx];
  if (e) { e.x=b.x; e.y=b.y; e.z=b.z; e.a=b.a; e.b=b.b; e.c=b.c; }
  syncBaseFrameGroups();
  applyBase();
}

// BASE-Offset neu anwenden und Bahn/Zielpunkte neu zeichnen (Kamera bleibt, IK erst beim Parsen)
function applyBase() {
  if (!parsedData.positions.length) return;
  bakeBaseOffset();
  var savedT = orbitTarget.clone(), savedR = orbitState.radius;
  buildScene(parsedData.positions);
  orbitTarget.copy(savedT); orbitState.radius = savedR; updateCamera();
  aPlotInvalidate();
  // Achsstellungen passen nicht mehr zur verschobenen Bahn: Hinweis ausblenden und
  // nach kurzer Eingabepause automatisch neu rechnen (vorher erst mit F9)
  var w = document.getElementById('sim-warn'); if (w) w.style.display = 'none';
  setStatus('paused', 'BASE geändert…');
  clearTimeout(applyBase._t);
  applyBase._t = setTimeout(function() { if (!_ikPending) parseAndLoad(); }, 700);
}

// Mausrad auf Zahlenfeldern (wie RobSimul): Wert um Schrittweite aendern
(function() {
  var hovered = null;
  document.addEventListener('mouseover', function(e) { if (e.target.type === 'number') hovered = e.target; });
  document.addEventListener('mouseout',  function(e) { if (e.target.type === 'number') hovered = null; });
  document.addEventListener('wheel', function(e) {
    if (!hovered) return;
    e.preventDefault();
    var step = parseFloat(hovered.step) || 1;
    hovered.value = (parseFloat(hovered.value) || 0) + (e.deltaY < 0 ? step : -step);
    hovered.dispatchEvent(new Event('input', {bubbles:true}));
  }, {passive:false});
})();

// ── Roboterauswahl (RobLib) ───────────────────────────────────────
var _libRobots = [];

function openLibPickerModal() {
  document.getElementById('libPickerSearch').value = '';
  document.getElementById('libPickerList').innerHTML = '<div class="lib-msg">Lade…</div>';
  document.getElementById('libPickerModal').style.display = 'flex';
  if (_libRobots.length) { renderLibPickerList(''); return; }
  fetch('https://cnc-technik.de/robsimul/roblib/api.php?action=list')
    .then(function(r){ return r.json(); })
    .then(function(data){
      _libRobots = (data.robots||[]).filter(function(r){ return (r.type||'robot') === 'robot'; });
      renderLibPickerList('');
    }).catch(function(e){
      document.getElementById('libPickerList').innerHTML = '<div class="lib-msg err">Fehler: '+e.message+'</div>';
    });
}

function closeLibPickerModal() {
  document.getElementById('libPickerModal').style.display = 'none';
}

function renderLibPickerList(q) {
  var el = document.getElementById('libPickerList');
  var items = q ? _libRobots.filter(function(r){ return r.name.toLowerCase().includes(q.toLowerCase()); }) : _libRobots;
  if (!items.length) { el.innerHTML='<div class="lib-msg">Keine Einträge.</div>'; return; }
  el.innerHTML = items.map(function(r,i){return '<div class="lib-item">'
    +'<div style="flex:1;min-width:0"><div class="name">'+(r.name||'')+'</div>'
    +'<div class="meta">'+(r.marke||'')+' '+(r.modell||'')+'</div></div>'
    +'<button class="btn primary" data-lib-pick="'+i+'">Laden</button>'
    +'</div>';
  }).join('');
  el.querySelectorAll('[data-lib-pick]').forEach(function(btn){
    btn.onclick = function(){ loadLibRobot(items[+btn.dataset.libPick]); closeLibPickerModal(); };
  });
}

document.getElementById('libPickerSearch').addEventListener('input', function(){ renderLibPickerList(this.value); });
document.getElementById('libPickerModal').addEventListener('click', function(e){
  if (e.target === this) closeLibPickerModal();
});

async function loadLibRobot(robot) {
  var lbl = document.getElementById('simple-robot-name');
  lbl.textContent = 'lädt ' + robot.name + '…';
  try {
    var resp = await fetch(robot.zip_url);
    var zip = await JSZip.loadAsync(await resp.blob());
    var jsonFile = null;
    zip.forEach(function(path,entry){ if(!entry.dir&&path.endsWith('.json')) jsonFile=entry; });
    if (!jsonFile) throw new Error('Kein JSON in ZIP');
    var data = JSON.parse(await jsonFile.async('string'));
    var stlBufs = {};
    var proms = [];
    zip.forEach(function(path,entry){
      var lp=path.toLowerCase();
      if(!entry.dir&&(lp.endsWith('.stl')||lp.endsWith('.osd'))){
        var fname=path.replace(/.*\//,'').toLowerCase().replace(/\.(stl|osd)$/i,'');
        proms.push(entry.async('arraybuffer').then(function(buf){
          var stlData=lp.endsWith('.osd')?osdToBinaryStl(buf):buf;
          stlBufs[fname]=stlData;
          // Alias: "base" auch unter 'podest' ablegen (sceneModels.pedestal)
          if(/base/i.test(fname)&&!stlBufs['podest']) stlBufs['podest']=stlData;
        }));
      }
    });
    await Promise.all(proms);
    applyKinematicData(data, stlBufs);
    lbl.textContent = robot.name;
    setStatus('paused','Geladen: '+robot.name);
    if (parsedData.positions.length) parseAndLoad();   // IK mit neuer Kinematik
  } catch(e) {
    lbl.textContent = 'Fehler beim Laden';
    setStatus('paused', 'Roboter nicht geladen: ' + e.message);
  }
}
function saveProgram() {
  var code = document.getElementById('code-input').value;
  downloadFile(code, 'programm.src');
}

function loadProgram() {
  const inp = document.getElementById('prog-file-in');
  inp.onchange = e => {
    const file = e.target.files[0]; if (!file) return;
    const reader = new FileReader();
    reader.onload = ev => {
      document.getElementById('code-input').value = ev.target.result;
      rebuildGutter();
      setStatus('paused', 'Programm geladen: ' + file.name);
    };
    reader.readAsText(file);
    inp.value = '';
  };
  inp.click();
}
// ═══════════════════════════════════════════════════
// IK TABLE — pre-computed for each program position
// ═══════════════════════════════════════════════════
let ikTable = []; // [{angles, ok, score}] per position


// Single-Start, schnelle Konvergenz (für Echtzeit-Nutzung)
function solveIKFast(tx, ty, tz, ta, tb, tc, initAngles) {
  return solveLM({
    tp:[tx,ty,tz], Rt:rotZYX(ta,tb,tc),
    starts:[initAngles ? initAngles.slice() : jointAngles.slice()],
    dt:0.4, lam:0.8, tolP:1.0, tolO:1.0,
    maxIter:80, stepMax:2.0, stepScale:8.0,
    earlyStop:0, okThresh:25,
  });
}

// Hochpräzisions-IK für Map: viele Iterationen, enge Toleranz
function solveIKPrecise(tx, ty, tz, ta, tb, tc, initAngles) {
  return solveLM({
    tp:[tx,ty,tz], Rt:rotZYX(ta,tb,tc),
    starts:[initAngles ? initAngles.slice() : jointAngles.slice()],
    dt:0.2, lam:0.3, tolP:0.01, tolO:0.01,
    maxIter:500, stepMax:1.0, stepScale:5.0,
    earlyStop:0, okThresh:0.1,
  });
}

// ── Jacobi-Singularitätserkennung (PDF Punkt 2) ───────────────
// computeManipulability — wird noch für DP-Solver genutzt, bleibt erhalten
function computeManipulability(angles_deg) {
  var dt = 0.5;
  var e0 = fkTCP_full(angles_deg);
  var J = [];
  for (let i = 0; i < 6; i++) {
    var q1 = angles_deg.slice(); q1[i] += dt;
    var e1 = fkTCP_full(q1);
    J.push([
      (e1.pos[0]-e0.pos[0])/dt, (e1.pos[1]-e0.pos[1])/dt, (e1.pos[2]-e0.pos[2])/dt,
      (e1.rot[0][0]-e0.rot[0][0])/dt, (e1.rot[1][0]-e0.rot[1][0])/dt, (e1.rot[2][0]-e0.rot[2][0])/dt
    ]);
  }
  var JtJ_diag = Array(6).fill(0);
  for (let i = 0; i < 6; i++)
    for (let r = 0; r < 6; r++) JtJ_diag[i] += J[i][r]*J[i][r];
  var minEig = Math.min.apply(null, JtJ_diag);
  var maxEig = Math.max.apply(null, JtJ_diag);
  return { manipulability: minEig, condition: maxEig / Math.max(minEig, 1e-9) };
}
// ── Singularitätstyp-Klassifikation ──────────────────────────
// Wrist:    |sin(A5)| < sin(8°)  → Achsen 4+6 kollinear
// Shoulder: Handgelenk-XY < 50mm → WZ auf A1-Achse
// Elbow:    isSingular() && weder Wrist noch Shoulder → Jacobi-Rangabfall
function classifySingTypes(angles_deg) {
  var types = [];

  // ── 1. Handgelenk: sin(A5) ≈ 0  →  A4 und A6 kollinear ──────
  var a5rad = angles_deg[4] * Math.PI / 180;
  if (Math.abs(Math.sin(a5rad)) < 0.14) types.push('wrist');

  // ── 2. Schulter: Handgelenk-XY auf A1-Achse ──────────────────
  var fk = fkAll(angles_deg), pts = fk.pts;
  var wx = pts[4][0], wy = pts[4][1];
  if (Math.sqrt(wx*wx + wy*wy) < 60) types.push('shoulder');

  // ── 3. Ellbogen (Streckstellung) ──────────────────────────────
  // Gestreckt ist der Arm, wenn beide Ellbogen-Lösungen der IK zusammenfallen: A3 = Streckwinkel.
  // Der Streckwinkel folgt aus den Achsmaßen (Unterarm-Versatz) und ist je Roboter verschieden
  // (KR8: ca. 18°, nicht 90°). Singulär innerhalb ±10° um den Streckwinkel.
  if (ikAnalyticSupported()) {
    var J = JOINTS_DEF, o2 = J[1].off, o3 = J[2].off, ux = o3[0] + J[3].off[0], uz = o3[2];
    var th3s = Math.atan2(o2[0]*uz - o2[2]*ux, o2[0]*ux + o2[2]*uz);
    var dd = angles_deg[2] * FK_SIGNS[2] * Math.PI / 180 - th3s;
    dd = Math.atan2(Math.sin(dd), Math.cos(dd));
    if (Math.abs(dd) < 10 * Math.PI / 180) types.push('elbow');
  } else {
    var a3rad = angles_deg[2] * Math.PI / 180;
    if (Math.abs(Math.cos(a3rad)) < 0.17) types.push('elbow');
  }

  return types;
}

// ── Globaler DP-Planer (nach Redundanzoptimierung-PDF) ────────
// Kostenfunktion: A4/A5 teuer, A6 kontinuierlich, Limits/Singularitäten bestrafen
function ikCost(from, to) {
  if (!from || !to) return 1e9;
  var W = [1, 1, 1, 8, 8, 1];  // A4/A5 hoch gewichten
  var cost = 0;
  for (let i = 0; i < 6; i++) {
    var d = shortestAngleDiff(from[i], to[i]);
    cost += W[i] * Math.abs(d);
  }
  // Limit-Penalty: Strafe bei Nähe zu Achsgrenzen
  for (let j = 0; j < 6; j++) {
    var lim = JOINTS_DEF[j];
    var margin = Math.min(to[j] - lim.min, lim.max - to[j]);
    if (margin < 10) cost += (10 - margin) * 5;
  }
  // Singularitäts-Penalty: A5 nahe 0
  var a5 = Math.abs(to[4]);
  if (a5 < 15) cost += (15 - a5) * 10;
  return cost;
}

// Generiert mehrere IK-Kandidaten pro Position (A6 periodisch gekachelt)
function ikCandidates(pos, prevAngles) {
  var cands = [];
  // A6 aus vorherigen Winkeln als Startwert
  var a6Plan = prevAngles[5];
  // Nur 2 A6-Kacheln: aktuell + benachbarte 360°-Kopie
  var planStart = prevAngles.slice(); planStart[5] = a6Plan;
  var planStart360 = prevAngles.slice(); planStart360[5] = a6Plan + 360;
  var planStartN360 = prevAngles.slice(); planStartN360[5] = a6Plan - 360;
  var a6Offsets = [0];  // Kachelung nur via Starts, nicht als Loop
  var baseStarts = [
    planStart,      // Plan-A6: wichtigster Kandidat
    prevAngles,     // Kontinuität vom Vorgänger
    planStart360,   // A6 + 360
    planStartN360,  // A6 - 360
  ];
  var seen = [];
  for (let oi = 0; oi < a6Offsets.length; oi++) {
    for (let si = 0; si < baseStarts.length; si++) {
      var start = baseStarts[si].slice();
      start[5] += a6Offsets[oi];  // A6 kacheln
      var res = solveIKFast(pos.X, pos.Y, pos.Z, pos.A, pos.B, pos.C, start);
      if (!res.ok) continue;
      // Normalisiere auf kürzesten Weg vom Vorgänger
      var ang = res.angles.map(function(v, i) {
        return prevAngles[i] + shortestAngleDiff(prevAngles[i], v);
      });
      // Duplikat-Check (0.5° Toleranz pro Achse)
      var isDup = seen.some(function(s) {
        return s.every(function(v, i) { return Math.abs(v - ang[i]) < 0.5; });
      });
      if (isDup) continue;
      // Harte Grenzen prüfen (mit 5° Sicherheitsabstand)
      var inLimits = ang.every(function(v, i) {
        return v >= JOINTS_DEF[i].min + 5 && v <= JOINTS_DEF[i].max - 5;
      });
      if (!inLimits) continue;
      seen.push(ang);
      cands.push({ angles: ang, score: res.score });
    }
  }
  return cands;
}

// Startkonfiguration aus parsedData ermitteln (letzter PTP vor ersten LIN)
function _ikGetStartQ() {
  var q = jointAngles.slice();
  if (!parsedData.steps) return q;
  for (let i = 0; i < parsedData.steps.length; i++) {
    var s = parsedData.steps[i];
    if (s.type === 'ptpAxis' && s.angles) q = s.angles.slice();
    if (s.type === 'move') break;
  }
  return q;
}
// Pfad 2: DPSolver (kompakter globaler Optimierer)
function _ikTableDPSolver(positions, N) {
  var arcS = [0];
  for (let i = 1; i < N; i++) {
    var p0 = positions[i-1], p1 = positions[i];
    var dx = p1.X-p0.X, dy = p1.Y-p0.Y, dz = p1.Z-p0.Z;
    arcS.push(arcS[i-1] + Math.sqrt(dx*dx+dy*dy+dz*dz));
  }
  var targetPts = positions.map(function(p, i) {
    return {s:arcS[i], X:p.X, Y:p.Y, Z:p.Z, A:p.A, B:p.B, C:p.C};
  });
  var result = DPSolver.plan(targetPts, _ikGetStartQ());
  for (let i = 0; i < N; i++) {
    var rp = result.rawPath[i] || result.rawPath[result.rawPath.length-1];
    ikTable.push({ angles: rp.q.slice(), ok: true, score: 0 });
  }
}

// Pfad 3: Fallback — manueller DP-Planer mit Kandidaten
function _ikTableFallbackDP(positions, N) {
  // Kandidaten pro Position erzeugen
  var allCands = [];
  var prevQ = jointAngles.slice();
  for (let i = 0; i < N; i++) {
    var p = positions[i];
    var cands = ikCandidates(p, prevQ);
    if (!cands.length) {
      var res = solveIK(p.X, p.Y, p.Z, p.A, p.B, p.C, prevQ);
      cands = [{ angles: res.angles, score: res.score }];
    }
    allCands.push(cands);
    prevQ = cands[0].angles.slice();
  }

  // DP forward pass
  var INF = 1e12;
  var dp = allCands.map(function(c) {
    return c.map(function() { return { cost: INF, prevJ: -1 }; });
  });
  for (let j = 0; j < allCands[0].length; j++) {
    dp[0][j].cost = ikCost(jointAngles, allCands[0][j].angles);
  }
  for (let i = 1; i < N; i++) {
    for (let j = 0; j < allCands[i].length; j++) {
      for (let k = 0; k < allCands[i-1].length; k++) {
        var c = dp[i-1][k].cost + ikCost(allCands[i-1][k].angles, allCands[i][j].angles);
        if (c < dp[i][j].cost) { dp[i][j].cost = c; dp[i][j].prevJ = k; }
      }
    }
  }

  // Besten Endknoten wählen + Pfad zurückverfolgen
  var bestJ = 0, bestCost = INF;
  for (let j = 0; j < allCands[N-1].length; j++) {
    if (dp[N-1][j].cost < bestCost) { bestCost = dp[N-1][j].cost; bestJ = j; }
  }
  var path = new Array(N);
  var cur = bestJ;
  for (let i = N-1; i >= 0; i--) {
    path[i] = cur;
    cur = dp[i][cur].prevJ;
    if (cur < 0) cur = 0;
  }

  for (let i = 0; i < N; i++) {
    var cand = allCands[i][path[i]];
    ikTable.push({ angles: cand.angles, score: cand.score, ok: cand.score < 20 });
  }
}
// ── Fortschrittsleiste für Parse & Load ──────────────────────
function parseProgress(pct, msg) {
  var w = document.getElementById('parse-prog'); if (w) w.style.display = 'block';
  var b = document.getElementById('parse-prog-bar'); if (b) b.style.width = Math.max(0, Math.min(100, pct)) + '%';
  var m = document.getElementById('parse-prog-msg'); if (m) m.textContent = msg || '';
}
function parseProgressHide() {
  var b = document.getElementById('parse-prog-bar'); if (b) b.style.width = '100%';
  setTimeout(function(){ var w = document.getElementById('parse-prog'); if (w) w.style.display = 'none'; if (b) b.style.width = '0%'; }, 250);
}

// IK-Tabelle häppchenweise berechnen, damit die Fortschrittsleiste echt mitläuft (UI bleibt responsiv).
// Achssprünge zwischen aufeinanderfolgenden Sätzen erkennen (z. B. Handachse klappt um, weil A5 an der
// Achsgrenze ist). Ein realer Roboter bleibt dort mit Endschalter stehen; die Simulation weicht nur aus.
let _ikJumps = [];
function _ikJumpCheck() {
  const el = document.getElementById('sim-warn'), P = parsedData.positions;
  _ikJumps = [];
  // Prüfung auf den Bahnstützpunkten (Abstand max. 25 mm): normale Bewegung bleibt dort bei wenigen Grad
  let lastSeg = -1;
  for (let k = 1; k < trajectory.length; k++) {
    const s = trajectory[k], a = trajectory[k-1].angles, b = s.angles, i = s.segIdx;
    if (!a || !b || !P[i] || P[i].type === 'PTP' || i === lastSeg) continue;
    let jmax = 0, jx = 0;
    for (let j = 0; j < 6; j++) { const d = Math.abs(shortestAngleDiff(a[j], b[j])); if (d > jmax) { jmax = d; jx = j; } }
    if (jmax > 15) { _ikJumps.push({ i: i, axis: jx + 1, deg: jmax }); lastSeg = i; }
  }
  // Kollision der simulierten Stellung mit dem gedruckten Teil
  const coll = [];
  for (let i = 0; i < ikTable.length; i++) { const a = ikTable[i] && ikTable[i].angles; if (a && _aCollides(_aNormQ(a), i)) coll.push(i); }
  if (!el) return;
  if (!_ikJumps.length && !coll.length) { el.style.display = 'none'; return; }
  const msgs = []; let f;
  if (_ikJumps.length) {
    f = _ikJumps[0]; const ln = P[f.i].lineNum !== undefined ? P[f.i].lineNum + 1 : '?';
    msgs.push('Achssprung an ' + _ikJumps.length + ' Sätzen, erster bei Satz ' + (f.i + 1) + ' (Zeile ' + ln + '), A' + f.axis + ' '
      + Math.round(f.deg) + '° – Achsgrenze erreicht, auf dem Roboter nicht fahrbar.');
  }
  if (coll.length) {
    const ci = coll[0], ln = P[ci].lineNum !== undefined ? P[ci].lineNum + 1 : '?';
    msgs.push('Kollision mit dem Bauteil an ' + coll.length + ' Sätzen, erster bei Satz ' + (ci + 1) + ' (Zeile ' + ln + ').');
    if (!f || ci < f.i) f = { i: ci };
  }
  el.textContent = 'Achtung: ' + msgs.join(' ') + ' Robot Map prüfen.';
  el.style.display = '';
  el.onclick = function() { pauseSim(); applySimT(Math.max(0, f.i - 1)); const si = parsedData.stepOfPos ? parsedData.stepOfPos[f.i - 1] : -1; if (si >= 0) applyStep(si); };
}

function computeIKTableProgressive(positions, done) {
  ikTable = [];
  var N = positions.length;
  if (!N) { buildTrajectory(positions, ikTable); done && done(); return; }
  // Kleine Programme MIT einheitlichem Tool: DPSolver in einem Rutsch (schnell genug)
  if (N <= 150 && _toolsAreUniform(positions)) {
    _applyToolToTCPDEF(positions[0].toolN || 1);
    parseProgress(40, N + ' Punkte — IK wird berechnet…');
    setTimeout(function(){
      try { _ikTableDPSolver(positions, N); } catch(e) { _ikTableFallbackDP(positions, N); }
      buildTrajectory(positions, ikTable);
      parseProgress(100, 'fertig'); done && done();
    }, 20);
    return;
  }
  // Große Programme oder mehrere Tools: Schnell-IK in Häppchen, TCP pro Punkt gesetzt
  var prevQ = _ikGetStartQ(), i = 0, CH = Math.max(25, Math.floor(N/50));
  (function chunk(){
    var end = Math.min(i + CH, N);
    for (; i < end; i++) {
      var p = positions[i];
      _applyToolToTCPDEF(p.toolN); // aktives Tool dieses Punkts
      var res = ikFastA(p.X, p.Y, p.Z, p.A, p.B, p.C, prevQ);
      var angles = (res.ok ? res.angles : prevQ).map(function(v, j){ return prevQ[j] + shortestAngleDiff(prevQ[j], v); });
      ikTable.push({ angles: angles, score: res.score, ok: res.ok });
      prevQ = angles.slice();
    }
    parseProgress(Math.round(i/N*100), 'IK: ' + i + ' / ' + N);
    if (i < N) setTimeout(chunk, 0);
    else { buildTrajectory(positions, ikTable); parseProgress(100, 'fertig'); done && done(); }
  })();
}
const orbitTarget=new THREE.Vector3(500,0,600);
const orbitState={theta:-0.7,phi:1.05,radius:3500};
const VIEW_K=Math.tan(25*Math.PI/180);    // halber Bildwinkel (FOV 50°) für Verschieben in der Bildebene

// Standardansichten (Kamera-Richtung vom Ziel aus)
const VIEWS={
  vorne:{theta:-Math.PI/2,phi:Math.PI/2}, hinten:{theta:Math.PI/2,phi:Math.PI/2},
  links:{theta:Math.PI,phi:Math.PI/2},     rechts:{theta:0,phi:Math.PI/2},
  oben:{theta:-Math.PI/2,phi:1e-4},        unten:{theta:-Math.PI/2,phi:Math.PI-1e-4},
  iso:{theta:-0.7,phi:1.05}
};

function updateCamera(){
  const{theta,phi,radius}=orbitState;
  const dir=new THREE.Vector3(Math.sin(phi)*Math.cos(theta),Math.sin(phi)*Math.sin(theta),Math.cos(phi));
  perspCam.position.copy(orbitTarget).addScaledVector(dir,radius);
  perspCam.lookAt(orbitTarget);
  requestRender();
}

function _markView(name){
  document.querySelectorAll('[data-view]').forEach(b=>b.classList.toggle('on',b.dataset.view===name));
}
function setView(name){
  const v=VIEWS[name]; if(!v)return;
  orbitState.theta=v.theta; orbitState.phi=v.phi;
  _markView(name); fitView();
}
function zoomBy(f){
  orbitState.radius=Math.max(80,Math.min(40000,orbitState.radius*f));
  updateCamera();
}
// Einpassen: Roboter + Programmbahn ins Bild, Blickrichtung bleibt
function fitView(){
  const box=new THREE.Box3();
  fkAll(jointAngles).pts.forEach(p=>box.expandByPoint(new THREE.Vector3(p[0],p[1],p[2])));
  parsedData.positions.forEach(p=>box.expandByPoint(new THREE.Vector3(p.X,p.Y,p.Z)));
  axisSTLMeshes.concat(pedestalMesh,toolMesh).forEach(m=>{ if(m&&m.visible) box.expandByObject(m); });
  const ctr=new THREE.Vector3();box.getCenter(ctr);
  orbitTarget.copy(ctr);
  orbitState.radius=Math.max(800,box.getSize(new THREE.Vector3()).length()*1.35);
  updateCamera();
}

function resize(){
  const vp=canvas.parentElement;const w=vp.clientWidth,h=vp.clientHeight;
  renderer.setSize(w,h);perspCam.aspect=w/h;perspCam.updateProjectionMatrix();
  updateCamera();
}
window.addEventListener('resize', function(){ resize(); _aPlotFitWidth(); });

// ═══════════════════════════════════════════════════
// MAUS: Orbit (links/rechts), Pan (Mitte), Zoom (Rad)
// ═══════════════════════════════════════════════════
let drag=null;
canvas.addEventListener('mousedown',e=>{
  if(e.button>2)return;
  e.preventDefault();
  drag={pan:e.button!==0,lx:e.clientX,ly:e.clientY};
  canvas.classList.add('dragging');
});
window.addEventListener('mousemove',e=>{
  if(!drag)return;
  const dx=e.clientX-drag.lx,dy=e.clientY-drag.ly;
  drag.lx=e.clientX;drag.ly=e.clientY;
  if(!drag.pan){orbitState.theta-=dx*.007;orbitState.phi=Math.max(1e-4,Math.min(Math.PI-1e-4,orbitState.phi+dy*.007));_markView(null);}
  else{
    // Verschieben in der Bildebene
    const sc=orbitState.radius*2*VIEW_K/Math.max(1,canvas.clientHeight);
    const fwd=activeCam.getWorldDirection(new THREE.Vector3());
    const right=new THREE.Vector3().crossVectors(fwd,activeCam.up).normalize();
    const up=new THREE.Vector3().crossVectors(right,fwd).normalize();
    orbitTarget.addScaledVector(right,-dx*sc).addScaledVector(up,dy*sc);
  }
  updateCamera();
});
window.addEventListener('mouseup',()=>{if(drag){drag=null;canvas.classList.remove('dragging');}});
canvas.addEventListener('contextmenu',e=>e.preventDefault());
canvas.addEventListener('wheel',e=>zoomBy(1+e.deltaY*.001),{passive:true});

_codeEl.addEventListener('input',()=>rebuildGutter());

// ═══════════════════════════════════════════════════
// GELENKWINKEL ANWENDEN — Robotermodell
// ═══════════════════════════════════════════════════
function applyAngles(angles) { requestRender();
  for (let i=0;i<6;i++) jointAngles[i] = angles[i];
  buildRobotModel(angles);
}
// STLLoader inline (r128 compatible)
class STLLoader {
  parse(buffer) {
    // Reliable binary check: 80 header + 4 count + n*50 = filesize
    const view = new DataView(buffer);
    const n = view.getUint32(80, true);
    if (n > 0 && 84 + n * 50 === buffer.byteLength) {
      return this._parseBinary(buffer);
    }
    const text = new TextDecoder().decode(new Uint8Array(buffer));
    if (text.startsWith('solid') && text.includes('facet')) return this._parseASCII(text);
    return this._parseBinary(buffer);
  }
  _parseASCII(text) {
    const geo = new THREE.BufferGeometry();
    const verts = [], norms = [];
    const lines = text.split("\n");
    let nx=0,ny=0,nz=0;
    for (const line of lines) {
      const l = line.trim();
      if (l.startsWith("facet normal")) { const p=l.split(/\s+/); nx=+p[2];ny=+p[3];nz=+p[4]; }
      else if (l.startsWith("vertex")) { const p=l.split(/\s+/); verts.push(+p[1],+p[2],+p[3]); norms.push(nx,ny,nz); }
    }
    geo.setAttribute("position",new THREE.Float32BufferAttribute(verts,3));
    geo.setAttribute("normal",new THREE.Float32BufferAttribute(norms,3));
    return geo;
  }
  _parseBinary(buffer) {
    const view = new DataView(buffer);
    const geo = new THREE.BufferGeometry();
    const n = view.getUint32(80, true);
    const verts = new Float32Array(n*9), norms = new Float32Array(n*9);
    let off = 84;
    for (let i=0;i<n;i++) {
      const nx=view.getFloat32(off,true),ny=view.getFloat32(off+4,true),nz=view.getFloat32(off+8,true); off+=12;
      for (let v=0;v<3;v++) { const j=i*9+v*3; verts[j]=view.getFloat32(off,true);verts[j+1]=view.getFloat32(off+4,true);verts[j+2]=view.getFloat32(off+8,true); norms[j]=nx;norms[j+1]=ny;norms[j+2]=nz; off+=12; } off+=2;
    }
    geo.setAttribute("position",new THREE.Float32BufferAttribute(verts,3));
    geo.setAttribute("normal",new THREE.Float32BufferAttribute(norms,3));
    return geo;
  }
}
const stlLoader = new STLLoader();

// ── OSD → Binary STL ─────────────────────────────────────────────
function osdToBinaryStl(arrayBuffer) {
  var data=new Uint8Array(arrayBuffer),view=new DataView(arrayBuffer);
  if(!data.length)return new ArrayBuffer(84);
  // Prüfen ob bereits binäres STL (80-Byte-Header + Triangle-Count)
  if(data.length>=84){var tc=new DataView(arrayBuffer).getUint32(80,true);if(tc>0&&84+tc*50===data.length)return arrayBuffer;}
  var hdr=new TextDecoder('utf-8',{fatal:false}).decode(data.slice(0,100));
  var isV2=hdr.includes('+SprutCAM: OpenGL stream file (version 2.00)');
  var headerSize=isV2?105:165;
  var CMD=[4,0,12,12,12,isV2?1:0,64,128,48,72,104];
  var LINE={1:1,2:1,3:1};
  var prims=[],cur=null,snx=0,sny=0,snz=0;
  for(var i=headerSize;i<data.length;){
    var code=data[i],ls=i+1,cs=CMD[code]||0; i+=1+cs;
    if(code===0){var t=view.getInt32(ls,true);if(t<10){cur={t:t,vx:[],vy:[],vz:[],nx:[],ny:[],nz:[]};prims.push(cur);}}
    else if(code===1){if(cur){for(var j=0;j<cur.vx.length;j++){var x=cur.vx[j],y=cur.vy[j],z=cur.vz[j];cur.vx[j]=-z;cur.vy[j]=x;cur.vz[j]=y;}for(var j=0;j<cur.nx.length;j++){var x=cur.nx[j],y=cur.ny[j],z=cur.nz[j];cur.nx[j]=-z;cur.ny[j]=x;cur.nz[j]=y;}}cur=null;}
    else if(code===2){snx=view.getFloat32(ls,true);sny=view.getFloat32(ls+4,true);snz=view.getFloat32(ls+8,true);if(cur){cur.nx.push(snx);cur.ny.push(sny);cur.nz.push(snz);}}
    else if(code===3){if(cur){cur.vx.push(view.getFloat32(ls,true));cur.vy.push(view.getFloat32(ls+4,true));cur.vz.push(view.getFloat32(ls+8,true));if(cur.nx.length<cur.vx.length){cur.nx.push(snx);cur.ny.push(sny);cur.nz.push(snz);}}}
  }
  var tris=[];
  function cn(ax,ay,az,bx,by,bz,cx,cy,cz){var ux=bx-ax,uy=by-ay,uz=bz-az,vx=cx-ax,vy=cy-ay,vz=cz-az,nx=uy*vz-uz*vy,ny=uz*vx-ux*vz,nz=ux*vy-uy*vx,l=Math.sqrt(nx*nx+ny*ny+nz*nz);return l>1e-10?[nx/l,ny/l,nz/l]:[0,0,1];}
  for(var pi=0;pi<prims.length;pi++){
    var p=prims[pi]; if(LINE[p.t])continue;
    var n=p.vx.length; if(n<3)continue;
    var idx;
    if(p.t===5){idx=[];for(var i=0;i<n-2;i++){if(i%2===0){idx.push(i,i+1,i+2);}else{idx.push(i+1,i,i+2);}}}
    else if(p.t===6){idx=[];for(var i=1;i<n-1;i++)idx.push(0,i,i+1);}
    else{idx=[];for(var i=0;i<n;i++)idx.push(i);}
    for(var i=0;i+2<idx.length;i+=3){
      var ia=idx[i],ib=idx[i+1],ic=idx[i+2];
      var ax=p.vx[ia],ay=p.vy[ia],az=p.vz[ia],bx=p.vx[ib],by=p.vy[ib],bz=p.vz[ib],cx=p.vx[ic],cy=p.vy[ic],cz=p.vz[ic];
      var fn; if(p.nx.length===n){fn=[p.nx[ia],p.ny[ia],p.nz[ia]];var l=Math.sqrt(fn[0]*fn[0]+fn[1]*fn[1]+fn[2]*fn[2]);if(l>1e-10){fn=[fn[0]/l,fn[1]/l,fn[2]/l];}else fn=cn(ax,ay,az,bx,by,bz,cx,cy,cz);}else fn=cn(ax,ay,az,bx,by,bz,cx,cy,cz);
      tris.push(fn[0],fn[1],fn[2],ax,ay,az,bx,by,bz,cx,cy,cz);
      tris.push(-fn[0],-fn[1],-fn[2],ax,ay,az,cx,cy,cz,bx,by,bz);
    }
  }
  var tc=tris.length/12,out=new ArrayBuffer(84+tc*50),dv=new DataView(out);
  dv.setUint32(80,tc,true);
  var off=84;
  for(var i=0;i<tris.length;i+=12){for(var j=0;j<12;j++){dv.setFloat32(off,tris[i+j],true);off+=4;}dv.setUint16(off,0,true);off+=2;}
  return out;
}
// ═══════════════════════════════════════════════════
// SIMULATION ENGINE
// ═══════════════════════════════════════════════════
const sim={t:0,playing:false,dir:1,stepTarget:null,stepIdx:0};

function simSpeed(){return(parseInt(document.getElementById('spd-s').value)/100)*3.0;}
// Orientierung über den kürzesten Winkelweg (A 179,8° → -179,6° dreht 0,6°, nicht 359,4°)
function lerpPos(a,b,f){return{X:a.X+(b.X-a.X)*f,Y:a.Y+(b.Y-a.Y)*f,Z:a.Z+(b.Z-a.Z)*f,
  A:a.A+shortestAngleDiff(a.A,b.A)*f,B:a.B+shortestAngleDiff(a.B,b.B)*f,C:a.C+shortestAngleDiff(a.C,b.C)*f};}
function shortestAngleDiff(from, to) {
  // Returns smallest signed difference (always <= 180°)
  var d = ((to - from) % 360 + 540) % 360 - 180;
  return d;
}
function lerpAngles(a, b, f) {
  return a.map(function(v, i) {
    return v + shortestAngleDiff(v, b[i]) * f;
  });
}

// ── Trajectory: pre-computed fine-grained path ────────────
// Each entry: {pos:{X,Y,Z,A,B,C}, angles:[6], segIdx:int}
// segIdx = index in parsedData.positions of destination point
let trajectory = [];   // built once on parseAndLoad
const PLANNED_COL = 0x60687b, PATH_COL = 0x32c6da; // geplante / abgefahrene Bahn
var _realtimeMode = false;  // Standard: aus

// ── DPSolver initialisieren ───────────────────────────────────
(function() {
  DPSolver.fkFn = function(q) { return fkAll(q); };
  DPSolver.solveIKFn = function(x,y,z,a,b,c,init) { return solveIK(x,y,z,a,b,c,init); };
  DPSolver.singFn = function(q) {
    var m = computeManipulability(q);
    return Math.max(0, 1.0 - m.condition / 1000.0);
  };
  DPSolver.limits = JOINTS_DEF.map(function(j){ return {min:j.min, max:j.max}; });
  DPSolver.settings.wAxes = [1,1,1,4,5,0.25];
  DPSolver.settings.a6Copies = [-1,0,1];
  DPSolver.settings.smoothSamples = 150;
})();
let trajMax = 0;       // trajectory.length - 1
let trajSegEnd = null; // Int32Array: Index des letzten Stützpunkts je Satz

// ── Geometric helpers ─────────────────────────────────────
function vec3(p){return[p.X,p.Y,p.Z];}
function v3len(a){return Math.sqrt(a[0]**2+a[1]**2+a[2]**2);}
function v3sub(a,b){return[a[0]-b[0],a[1]-b[1],a[2]-b[2]];}
function v3add(a,b){return[a[0]+b[0],a[1]+b[1],a[2]+b[2]];}
function v3scale(a,s){return[a[0]*s,a[1]*s,a[2]*s];}
function v3dot(a,b){return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}
function v3cross(a,b){return[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];}
function v3norm(a){const l=v3len(a)||1;return[a[0]/l,a[1]/l,a[2]/l];}

// Slerp orientation (simple linear for Euler — close enough for simulation)
function slerpOri(a,b,f){
  return{A:a.A+(b.A-a.A)*f,B:a.B+(b.B-a.B)*f,C:a.C+(b.C-a.C)*f};
}

// Circle through 3 points: returns {center, radius, normal, startAngle, totalAngle, arcLen}
function circleFrom3(p0,pAux,p1){
  const a=vec3(p0),m=vec3(pAux),b=vec3(p1);
  const ab=v3sub(b,a),am=v3sub(m,a);
  const n=v3norm(v3cross(ab,am));
  // Circumcenter in the plane
  const ac=v3sub(b,a),ad=v3sub(m,a);
  const d=2*(ac[0]*(ad[1]*n[2]-ad[2]*n[1])-ac[1]*(ad[0]*n[2]-ad[2]*n[0])+ac[2]*(ad[0]*n[1]-ad[1]*n[0]));
  if(Math.abs(d)<1e-9) return null;
  // xax = component of (p0-pAux) projected onto circle plane, normalized
  const pa=v3sub(a,m);
  const pa_proj=v3sub(pa,v3scale(n,v3dot(pa,n)));
  if(v3len(pa_proj)<1e-9) return null;
  const xax=v3norm(pa_proj);
  const yax=v3cross(n,xax);
  function proj2(p){const rel=v3sub(p,a);return[v3dot(rel,xax),v3dot(rel,yax)];}
  const A2=proj2(a),M2=proj2(m),B2=proj2(b);
  // circumcenter in 2D
  const ax=A2[0],ay=A2[1],bx=B2[0],by=B2[1],mx=M2[0],my=M2[1];
  const D2=2*(ax*(by-my)+bx*(my-ay)+mx*(ay-by));
  if(Math.abs(D2)<1e-9)return null;
  const ux=((ax**2+ay**2)*(by-my)+(bx**2+by**2)*(my-ay)+(mx**2+my**2)*(ay-by))/D2;
  const uy=((ax**2+ay**2)*(mx-bx)+(bx**2+by**2)*(ax-mx)+(mx**2+my**2)*(bx-ax))/D2;
  const center=v3add(a,v3add(v3scale(xax,ux),v3scale(yax,uy)));
  const radius=v3len(v3sub(a,center));
  const toA=v3norm(v3sub(a,center));
  const toB=v3norm(v3sub(b,center));
  const toM=v3norm(v3sub(m,center));
  const angA=Math.atan2(v3dot(toA,yax),v3dot(toA,xax));
  const angB=Math.atan2(v3dot(toB,yax),v3dot(toB,xax));
  const angM=Math.atan2(v3dot(toM,yax),v3dot(toM,xax));
  // Determine direction (CCW or CW) via auxiliary point
  let totalAngle=angB-angA;
  const mAngle=angM-angA;
  // normalize
  const norm2=(v)=>{let x=v;while(x>Math.PI)x-=2*Math.PI;while(x<-Math.PI)x+=2*Math.PI;return x;}
  totalAngle=norm2(totalAngle);
  const mN=norm2(mAngle);
  // if aux is not between a and b, flip direction
  if(totalAngle>0&&mN<0){totalAngle-=2*Math.PI;}
  else if(totalAngle<0&&mN>0){totalAngle+=2*Math.PI;}
  const arcLen=Math.abs(radius*totalAngle);
  return{center,radius,n,xax,yax,startAngle:angA,totalAngle,arcLen};
}

function sampleArc(arc,f){
  const angle=arc.startAngle+arc.totalAngle*f;
  return[
    arc.center[0]+arc.radius*(Math.cos(angle)*arc.xax[0]+Math.sin(angle)*arc.yax[0]),
    arc.center[1]+arc.radius*(Math.cos(angle)*arc.xax[1]+Math.sin(angle)*arc.yax[1]),
    arc.center[2]+arc.radius*(Math.cos(angle)*arc.xax[2]+Math.sin(angle)*arc.yax[2]),
  ];
}

// Catmull-Rom spline for SLIN
function catmullRomSegment(p0,p1,p2,p3,t){
  const s=0.5;
  return[
    s*((-p0[0]+3*p1[0]-3*p2[0]+p3[0])*t**3+(2*p0[0]-5*p1[0]+4*p2[0]-p3[0])*t**2+(-p0[0]+p2[0])*t+2*p1[0])/1,
    s*((-p0[1]+3*p1[1]-3*p2[1]+p3[1])*t**3+(2*p0[1]-5*p1[1]+4*p2[1]-p3[1])*t**2+(-p0[1]+p2[1])*t+2*p1[1])/1,
    s*((-p0[2]+3*p1[2]-3*p2[2]+p3[2])*t**3+(2*p0[2]-5*p1[2]+4*p2[2]-p3[2])*t**2+(-p0[2]+p2[2])*t+2*p1[2])/1,
  ];
}

// ── Build trajectory from parsed positions ────────────────
function buildTrajectory(positions, ikTab) {
  trajectory = [];
  const N = positions.length;
  if (!N) { trajMax = 0; return; }

  // Adaptives Sampling: mehr Punkte = größerer Schritt
  const STEP_MM = N > 500 ? 25 : N > 200 ? 15 : N > 50 ? 10 : 8;

  // IK with warm-start from previous angles
  // Used for LIN/SLIN/CIRC to guarantee straight Cartesian path
  function ikWarm(pos, prevAngles) {
    const res = ikRobustA(pos.X, pos.Y, pos.Z, pos.A, pos.B, pos.C, prevAngles);
    if (!res.ok) return prevAngles;
    // Sprungsperre wie bei LIN: keine Achse springt zwischen zwei Bahnpunkten um mehr als 30°
    const maxDelta = res.angles.reduce((m, v, j) => Math.max(m, Math.abs(v - prevAngles[j])), 0);
    return maxDelta < 30 ? res.angles : prevAngles;
  }

  function pushSample(pos, angles, segIdx) {
    trajectory.push({pos, angles, segIdx});
  }

  // Startwinkel: letzter PTP-Achsbefehl vor den LIN-Positionen
  var _buildStart = (ikTab[0]&&ikTab[0].angles) || [0,-90,90,0,0,0];
  if (parsedData && parsedData.steps) {
    for (let _si=0; _si<parsedData.steps.length; _si++) {
      var _st=parsedData.steps[_si];
      if (_st.type==='ptpAxis'&&_st.angles) _buildStart=_st.angles.slice();
      if (_st.type==='move') break;
    }
  }
  // Erster Punkt: direkt ikTab[0] verwenden — exakt gleich wie angPrev im Loop
  var _ang0 = (ikTab[0]&&ikTab[0].angles) ? ikTab[0].angles : _buildStart;
  pushSample(positions[0], _ang0, 0);

  for (let i = 1; i < N; i++) {
    const prev = positions[i-1];
    const curr = positions[i];
    const angPrev = (ikTab[i-1]&&ikTab[i-1].angles) || [0,-90,90,0,0,0];
    const angCurr = (ikTab[i]&&ikTab[i].angles)   || [0,-90,90,0,0,0];
    const type = curr.type;

    if (type === 'CIRC_AUX') {
      pushSample(curr, angCurr, i);
      continue;
    }

    if (type === 'CIRC' && i > 0 && (positions[i-1]&&positions[i-1].type) === 'CIRC_AUX') {
      const pStart = i >= 2 ? positions[i-2] : prev;
      const pAux   = prev;
      const pEnd   = curr;
      const arc    = circleFrom3(pStart, pAux, pEnd);
      if (!arc || arc.arcLen < 1) { pushSample(curr, angCurr, i); continue; }
      const steps = Math.max(4, Math.ceil(arc.arcLen / STEP_MM));
      const angStart = (ikTab[Math.max(0,i-2)]&&ikTab[Math.max(0,i-2)].angles) || angPrev;
      let warmAng = [...angStart];
      for (let s = 1; s <= steps; s++) {
        const f = s / steps;
        const pt = sampleArc(arc, f);
        const ori = slerpOri(pStart, pEnd, f);
        const pos = {X:pt[0], Y:pt[1], Z:pt[2], ...ori};
        // IK at each arc point → true circular path
        warmAng = ikWarm(pos, warmAng);
        pushSample(pos, warmAng, i);
      }
      continue;
    }

    if (type === 'SLIN') {
      const p0 = i >= 2  ? vec3(positions[i-2]) : vec3(prev);
      const p1 = vec3(prev);
      const p2 = vec3(curr);
      const p3 = i+1 < N ? vec3(positions[i+1]) : vec3(curr);
      const dist = v3len(v3sub(p2, p1));
      const steps = Math.max(4, Math.ceil(dist / STEP_MM));
      let warmAng = [...angPrev];
      for (let s = 1; s <= steps; s++) {
        const f = s / steps;
        const pt = catmullRomSegment(p0,p1,p2,p3,f);
        const ori = slerpOri(prev, curr, f);
        const pos = {X:pt[0], Y:pt[1], Z:pt[2], ...ori};
        // IK at each spline point → smooth curve in Cartesian space
        warmAng = ikWarm(pos, warmAng);
        pushSample(pos, warmAng, i);
      }
      continue;
    }

    if (type === 'LIN') {
      const dist = v3len(v3sub(vec3(curr), vec3(prev)));
      const steps = Math.max(4, Math.ceil(dist / STEP_MM));
      // Normalize angCurr to closest path from angPrev (keine 360°-Sprünge)
      const angCurrLin = angCurr.map(function(v, j) {
        return angPrev[j] + shortestAngleDiff(angPrev[j], v);
      });
      // Alle Stützpunkte per IK auf der Geraden (auch der erste); Achsinterpolation nur als Rückfall bei Sprung
      let warmAng = angPrev.slice();
      for (let s = 1; s <= steps; s++) {
        const f = s / steps;
        const pos = lerpPos(prev, curr, f);
        if (dist < 20) {
          warmAng = lerpAngles(angPrev, angCurrLin, f);
        } else {
          const res = ikRobustA(pos.X, pos.Y, pos.Z, pos.A, pos.B, pos.C, warmAng);
          if (res.ok) {
            const resN = res.angles.map(function(v, j) {
              return warmAng[j] + shortestAngleDiff(warmAng[j], v);
            });
            const totalDelta = resN.reduce(function(s, v, j) { return s + Math.abs(v - warmAng[j]); }, 0);
            const maxDelta = resN.reduce(function(s, v, j) { return Math.max(s, Math.abs(v - warmAng[j])); }, 0);
            warmAng = (totalDelta < 45 && maxDelta < 30) ? resN : lerpAngles(angPrev, angCurrLin, f);
          } else {
            warmAng = lerpAngles(angPrev, angCurrLin, f);
          }
        }
        pushSample(pos, warmAng, i);
      }
      continue;
    }

    // PTP: joint-space interpolation mit kürzestem Winkelweg
    const dist = v3len(v3sub(vec3(curr), vec3(prev)));
    const steps = Math.max(2, Math.ceil(dist / (STEP_MM*3)));
    // Normalize angCurr to be closest to angPrev (no 360° flips)
    const angCurrN = angCurr.map(function(v, j) {
      var diff = shortestAngleDiff(angPrev[j], v);
      return angPrev[j] + diff;
    });
    for (let s = 1; s <= steps; s++) {
      const f = s / steps;
      const pos = lerpPos(prev, curr, f);
      const angles = lerpAngles(angPrev, angCurrN, f);
      pushSample(pos, angles, i);
    }
  }

  trajMax = Math.max(0, trajectory.length - 1);
  // Letzter Stützpunkt je Satz (liegt genau auf dem Programmpunkt) für die Umrechnung Satz → Stützpunkt
  trajSegEnd = new Int32Array(N).fill(-1);
  for (let k = 0; k < trajectory.length; k++) { const si = trajectory[k].segIdx; if (si >= 0 && si < N) trajSegEnd[si] = k; }
  for (let i = 1; i < N; i++) if (trajSegEnd[i] < 0) trajSegEnd[i] = trajSegEnd[i-1];

  // Abgefahrene Bahn: einmal bauen, beim Abspielen nur DrawRange verlängern.
  // Gerade Abschnitte nur mit Endpunkt (Kreis/Spline mit allen Stützpunkten) -> weniger Eckpunkte;
  // das Stück bis zur aktuellen Position zeichnet eine kurze Kopflinie.
  [visitedLine, visitedHead].forEach(o => { if (o) { pathGrp.remove(o); o.geometry.dispose(); o.material.dispose(); } });
  visitedLine = visitedHead = null;
  if (trajectory.length > 1) {
    const keep = [];
    _visitedVtx = new Int32Array(trajectory.length);
    for (let i = 0; i < trajectory.length; i++) {
      const s = trajectory[i], nx = trajectory[i+1], ty = (positions[s.segIdx] || {}).type;
      if (i === 0 || !nx || nx.segIdx !== s.segIdx || ty === 'CIRC' || ty === 'SLIN') keep.push(i);
      _visitedVtx[i] = keep.length;
    }
    _visitedKeep = keep;
    const arr = new Float32Array(keep.length * 3);
    keep.forEach((k, i) => { const p = trajectory[k].pos; arr[i*3] = p.X; arr[i*3+1] = p.Y; arr[i*3+2] = p.Z; });
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(arr, 3));
    geo.setDrawRange(0, 0);
    const mat = new THREE.LineBasicMaterial({color:PATH_COL, linewidth:pathLineWidth});
    visitedLine = new THREE.Line(geo, mat);
    visitedLine.frustumCulled = false;
    const hg = new THREE.BufferGeometry();
    hg.setAttribute('position', new THREE.BufferAttribute(new Float32Array(6), 3));
    visitedHead = new THREE.Line(hg, mat.clone());
    visitedHead.frustumCulled = false; visitedHead.visible = false;
    pathGrp.add(visitedLine, visitedHead);
  }
}

function getTrajSample(t) {
  if (!trajectory.length) return null;
  const f = Math.max(0, Math.min(1, t));
  const idx = Math.floor(f * trajMax);
  const next = Math.min(idx+1, trajMax);
  const frac = f * trajMax - idx;
  const a = trajectory[idx], b = trajectory[next];
  if (!a) return null;
  const pos = lerpPos(a.pos, b.pos, frac);
  const angles = lerpAngles(a.angles, b.angles, frac);
  return {pos, angles, segIdx: a.segIdx};
}

// Satzposition t (0..N-1) → Anteil 0..1 der Stützpunktliste. Satzweise: innerhalb von Satz k+1 linear
// zwischen dem Endstützpunkt von Satz k und dem von Satz k+1 (Stützpunkte liegen gleichmäßig auf der Strecke).
// Vorher gleichmäßig über alle Stützpunkte → Sätze mit vielen/wenigen Stützpunkten liefen zu langsam/schnell.
function simTToTrajT(t) {
  const N = parsedData.positions.length;
  if (N < 2 || !trajMax) return 0;
  if (!trajSegEnd || trajSegEnd.length !== N) return t / (N-1);
  const k = Math.max(0, Math.min(N - 2, Math.floor(t))), f = Math.max(0, Math.min(1, t - k));
  const a = Math.max(0, trajSegEnd[k]), b = Math.max(a, trajSegEnd[k+1]);
  return (a + f * (b - a)) / trajMax;
}

function getIPos(t){
  const s = getTrajSample(simTToTrajT(t));
  return (s&&s.pos) || null;
}

function getIKAngles(t) {
  const s = getTrajSample(simTToTrajT(t));
  return (s&&s.angles) || null;
}

function updateMarkerPose(ipos){markerGrp.position.set(ipos.X,ipos.Y,ipos.Z);markerGrp.setRotationFromEuler(kukaEuler(ipos.A,ipos.B,ipos.C));markerGrp.visible=true;}

// Visited path
let visitedLine=null;
let visitedHead=null, _visitedVtx=null, _visitedKeep=null;
// G-Code-Darstellung: 'all' = ganze Bahn, 'upto' = bis zur aktuellen Simulationsposition, 'none' = ausgeblendet
let plannedLine=null, pathMode='all';
function setPathMode(mode){
  pathMode=mode;
  document.querySelectorAll('[data-path]').forEach(b=>b.classList.toggle('on',b.dataset.path===mode));
  updateVisitedPath(sim.t); requestRender();
}
function _applyPathMode(t){
  const show=pathMode!=='none';
  pathGrp.visible=show; posGrp.visible=show;
  if(!show)return;
  const N=parsedData.positions.length;
  const cnt=pathMode==='upto' ? (t>0||sim.t>0 ? Math.min(N,Math.floor(t)+1) : 1) : N;
  if(plannedLine) plannedLine.geometry.setDrawRange(0,cnt);
  // Zielpunkte: Punktwolke per DrawRange, Koordinatenkreuze einzeln
  posGrp.children.forEach((c,i)=>{ if(c.isPoints) c.geometry.setDrawRange(0,cnt); else c.visible=i<cnt; });
}
function updateVisitedPath(t){
  _applyPathMode(t);
  if(!visitedLine)return;
  const cutIdx=(trajectory.length&&t>0)?Math.floor(simTToTrajT(t)*trajMax):0;
  if(cutIdx<1){visitedLine.geometry.setDrawRange(0,0);visitedHead.visible=false;return;}
  const n=_visitedVtx[cutIdx];
  visitedLine.geometry.setDrawRange(0, n>=2?n:0);
  // Kopflinie vom letzten gespeicherten Eckpunkt bis zur aktuellen Bahnposition
  const a=trajectory[_visitedKeep[n-1]].pos, b=trajectory[cutIdx].pos, h=visitedHead.geometry.attributes.position;
  h.setXYZ(0,a.X,a.Y,a.Z); h.setXYZ(1,b.X,b.Y,b.Z); h.needsUpdate=true;
  visitedHead.visible=true;
}
function setStatus(cls,txt){const el=document.getElementById('sim-status');el.className='sstatus '+cls;el.textContent=txt;}
function applySimT(t){
  const pos=parsedData.positions,N=pos.length;if(!N)return;
  sim.t=Math.max(0,Math.min(N-1,t));
  document.getElementById('pos-s').value=sim.t;
  {const pe=document.getElementById('pos-pct'); if(pe) pe.textContent=(N>1?Math.round(sim.t/(N-1)*100):0)+' %';}
  const idx=Math.min(Math.floor(sim.t),N-1);
  if(typeof setActiveTool==='function')setActiveTool(pos[idx]&&pos[idx].toolN); // aktives TCP/Tool dieses Satzes
  // Show trajectory sample count
  const trajSamples = trajectory.length;
  document.getElementById('pos-v').textContent=`${idx+1} / ${N}` + (trajSamples>N?` (${trajSamples} Schritte)`:'');
  updateVisitedPath(sim.t);updateGutterActive(pos[idx].lineNum);
  {const si=parsedData.stepOfPos?parsedData.stepOfPos[idx]:-1;if(si>=0)sim.stepIdx=si;}
  // Bei exaktem Positionsindex: präzise ikTable-Winkel verwenden
  let angles;
  if(Number.isInteger(sim.t) && ikTable[idx] && ikTable[idx].ok) {
    angles = ikTable[idx].angles;
  } else {
    angles = getIKAngles(sim.t);
  }
  if(angles){
    applyAngles(angles);
    // Place simulation marker at actual FK TCP (not stored pos) → always consistent
    const fkRes=fkAll(angles);
    const fkTCP=fkRes.pts[7];
    // Marker always at actual FK TCP so sphere and coordinate axes coincide
    {
      const R=fkRes.tcp_rot;
      const Beu=-Math.asin(Math.max(-1,Math.min(1,R[2][0])));
      const cb=Math.cos(Beu);let Aeu,Ceu;
      if(Math.abs(cb)>1e-6){Aeu=Math.atan2(R[1][0]/cb,R[0][0]/cb);Ceu=Math.atan2(R[2][1]/cb,R[2][2]/cb);}
      else{Aeu=0;Ceu=Math.atan2(-R[1][2],R[1][1]);}
      const nd=v=>{let d=v*180/Math.PI;while(d>180)d-=360;while(d<=-180)d+=360;return d;}
      updateMarkerPose({X:fkTCP[0],Y:fkTCP[1],Z:fkTCP[2],A:nd(Aeu),B:nd(Beu),C:nd(Ceu)});
    }
    // Status bar: show FK error vs stored trajectory target
    const ip=getIPos(sim.t);
    if(ip){
      const err=Math.sqrt((fkTCP[0]-ip.X)**2+(fkTCP[1]-ip.Y)**2+(fkTCP[2]-ip.Z)**2);
      const mi=document.getElementById('marker-info');
      mi.style.display='block';
      mi.textContent=`#${idx+1} ${pos[idx].type}  X${fkTCP[0].toFixed(1)} Y${fkTCP[1].toFixed(1)} Z${fkTCP[2].toFixed(1)}  FK-Δ:${err.toFixed(1)}mm`;
    }
  } else {
    const ip=getIPos(sim.t);if(ip)updateMarkerPose(ip);
    const mi=document.getElementById('marker-info');
    if(ip){mi.style.display='block';mi.textContent=`#${idx+1} ${pos[idx].type}  X${ip.X.toFixed(1)} Y${ip.Y.toFixed(1)} Z${ip.Z.toFixed(1)}`;}
  }
  aPlotDraw();
}
function applyStep(idx){
  const steps=parsedData.steps;if(!(steps&&steps.length))return;
  idx=Math.max(0,Math.min(steps.length-1,idx));sim.stepIdx=idx;const step=steps[idx];
  updateGutterActive(step.lineNum);
  let posIdx=-1;for(let i=idx;i>=0;i--){if(steps[i].type==='move'){posIdx=steps[i].posIdx;break;}}
  const N=parsedData.positions.length;
  if(posIdx>=0){
    const pos=parsedData.positions[posIdx];if(typeof setActiveTool==='function')setActiveTool(pos.toolN);updateMarkerPose(pos);markerGrp.visible=true;updateVisitedPath(posIdx);
    document.getElementById('pos-s').value=posIdx;document.getElementById('pos-v').textContent=`${posIdx+1} / ${N}`;
    document.getElementById('marker-info').style.display='block';document.getElementById('marker-info').textContent=`Z.${step.lineNum+1}  #${posIdx+1} ${pos.type}  X${pos.X.toFixed(1)} Y${pos.Y.toFixed(1)} Z${pos.Z.toFixed(1)}`;
    if((ikTable[posIdx]&&ikTable[posIdx].ok))applyAngles(ikTable[posIdx].angles);
    var _vEl=document.getElementById('tcp-vel-v');
    if(_vEl){var _v=(pos.velCP?pos.velCP:0.167);_vEl.textContent=Math.round(_v*60000)+' mm/min';}
  }else{document.getElementById('marker-info').style.display='none';if(N>0)document.getElementById('pos-v').textContent=`— / ${N}`;}
  const typeLabel={move:'MOVE',signal:'SIGNAL',var:'VARIABLE',other:'STATEMENT'};
  setStatus('paused',`L.${step.lineNum+1}  ${typeLabel[step.type]||''}`);
}

function toggleRealtime() {
  _realtimeMode = !_realtimeMode;
  var btn = document.getElementById('btn-realtime');
  if (btn) btn.classList.toggle('on', _realtimeMode);
}

function pauseSim(){sim.playing=false;sim.stepTarget=null;document.getElementById('b-playfwd').classList.remove('on');document.getElementById('b-playrev').classList.remove('on');}
function stopSim(goTo){pauseSim();if(goTo!==undefined)applySimT(goTo);}
let lastTs=null;
// Simulationsgeschwindigkeit für aktuellen Satz berechnen
// Mittlere Satzlänge (mm) des Programms, einmal je Positionsliste berechnet
let _meanSegCache = { pos: null, v: 1 };
function _meanSegLen(pos) {
  if (_meanSegCache.pos === pos) return _meanSegCache.v;
  let sum = 0;
  for (let i = 1; i < pos.length; i++) sum += Math.sqrt((pos[i].X-pos[i-1].X)**2 + (pos[i].Y-pos[i-1].Y)**2 + (pos[i].Z-pos[i-1].Z)**2);
  _meanSegCache = { pos: pos, v: pos.length > 1 ? Math.max(0.1, sum / (pos.length - 1)) : 1 };
  return _meanSegCache.v;
}
function _simCalcSpeed(pos, curPosIdx, nextPosIdx) {
  // VEL.CP gilt für den Zielpunkt (nextPosIdx), nicht den Ausgangspunkt
  const curVelCP = (pos[nextPosIdx] && pos[nextPosIdx].velCP) ? pos[nextPosIdx].velCP : 0.167;
  const p0 = pos[curPosIdx], p1 = pos[nextPosIdx];
  const segDist = (p0 && p1 && nextPosIdx !== curPosIdx)
    ? Math.sqrt((p1.X-p0.X)**2+(p1.Y-p0.Y)**2+(p1.Z-p0.Z)**2) : 0;
  const velMmS = curVelCP * 1000;
  // Anzeige IMMER die programmierte TCP-Geschwindigkeit des aktuellen Satzes (passt zum Programm),
  // unabhaengig vom Abspieltempo (Slider).
  const dispMmMin = Math.round(velMmS * 60);
  // Zeit je Satz proportional zur Streckenlänge (kurze Sätze, z. B. 1 mm Lagenwechsel, ohne Stillstand).
  // Mindestlänge 0,1 mm, damit reine Umorientierungen nicht in null Zeit ablaufen.
  const eff = Math.max(segDist, 0.1);
  if (!_realtimeMode) {
    // Slider: Tempo in Sätzen/s bezogen auf die mittlere Satzlänge → gleiche Gesamtdauer wie zuvor
    return { speed: simSpeed() * _meanSegLen(pos) / eff, velCP: curVelCP, displayMmMin: dispMmMin };
  }
  return { speed: velMmS / eff, velCP: curVelCP, displayMmMin: dispMmMin };
}

// Fortschritt über dt satzweise integrieren: jeder Satz hat seine eigene Rate (Sätze/s),
// die Bildzeit wird über Satzgrenzen hinweg korrekt aufgeteilt (kein Überspringen nach kurzen Sätzen)
function _simIntegrate(pos, t, dir, dt) {
  const N = pos.length;
  let rem = dt, guard = 0;
  while (rem > 1e-9 && guard++ < 100000) {
    const i0 = dir > 0 ? Math.floor(t + 1e-9) : Math.ceil(t - 1e-9) - 1;
    if (i0 < 0 || i0 >= N - 1) break;
    const rate = _simCalcSpeed(pos, i0, i0 + 1).speed;
    const bound = dir > 0 ? i0 + 1 : i0;
    const need = Math.abs(bound - t) / rate;
    if (need > rem) { t += dir * rate * rem; rem = 0; }
    else { t = bound; rem -= need; }
  }
  return t;
}

// Simulationszeit um dt vorrücken; rendert selbst und gibt false zurück wenn frame() nicht weiterrendern soll
function _simAdvance(N, dt, speed, velCP, displayMmMin) {
  let newT;
  const pos = parsedData.positions;
  if (sim.stepTarget !== null) {
    const sd = sim.stepTarget > sim.t ? 1 : -1;
    newT = _simIntegrate(pos, sim.t, sd, dt);
    if ((sd > 0 && newT >= sim.stepTarget) || (sd < 0 && newT <= sim.stepTarget)) {
      newT = sim.stepTarget; sim.stepTarget = null;
      applySimT(newT); setStatus('paused','PAUSED'); renderer.render(scene, activeCam);
      return false;
    }
  } else {
    newT = _simIntegrate(pos, sim.t, sim.dir, dt);
    if      (newT >= N-1) { newT = N-1; pauseSim(); setStatus('end','END ✓'); }
    else if (newT <= 0)   { newT = 0;   pauseSim(); setStatus('paused','START'); }
    else {
      // Ist-Geschwindigkeit (Bahn-mm pro echter Sekunde, geglättet) neben dem Sollwert
      const P = parsedData.positions, D = t => { const i = Math.min(Math.floor(t), P.length - 2), f = t - i;
        return i < 0 ? 0 : Math.hypot(P[i+1].X-P[i].X, P[i+1].Y-P[i].Y, P[i+1].Z-P[i].Z) * f; };
      let dist = 0; const a = Math.min(sim.t, newT), b = Math.max(sim.t, newT);
      if (Math.floor(a) === Math.floor(b)) dist = Math.abs(D(b) - D(a));
      else { dist = (Math.hypot(P[Math.floor(a)+1].X-P[Math.floor(a)].X, P[Math.floor(a)+1].Y-P[Math.floor(a)].Y, P[Math.floor(a)+1].Z-P[Math.floor(a)].Z) - D(a)) + D(b);
        for (let i = Math.floor(a) + 1; i < Math.floor(b); i++) dist += Math.hypot(P[i+1].X-P[i].X, P[i+1].Y-P[i].Y, P[i+1].Z-P[i].Z); }
      if (dt > 0) sim.vIst = sim.vIst === undefined ? dist / dt : sim.vIst * 0.9 + (dist / dt) * 0.1;
      setStatus('playing', (sim.dir>0?'▶':'◀') + '  ' + (displayMmMin??Math.round(velCP*60000)) + ' mm/min'
        + (sim.vIst !== undefined ? ' · Ist ' + Math.round(sim.vIst * 60) : ''));
    }
  }
  applySimT(newT);
  return true;
}

function frame(ts) {
  requestAnimationFrame(frame);
  // Bildzeit bis 0,5 s übernehmen: auch bei wenigen Bildern/s läuft Echtzeit mit der echten Uhr
  const dt = lastTs !== null ? Math.min((ts-lastTs)/1000, .5) : 0;
  lastTs = ts;
  const N = parsedData.positions.length;
  if (N > 0 && (sim.playing || sim.stepTarget !== null)) {
    const pos = parsedData.positions;
    const i0  = Math.min(Math.floor(sim.t), pos.length-1);
    const i1  = Math.min(i0+1, pos.length-1);
    const { speed, velCP, displayMmMin } = _simCalcSpeed(pos, i0, i1);
    const velEl = document.getElementById('tcp-vel-v');
    if (velEl) velEl.textContent = displayMmMin + ' mm/min';
    _simAdvance(N, dt, speed, velCP, displayMmMin);
    _renderFrames = Math.max(_renderFrames, 1); // während Sim rendern
  }
  if (_renderFrames > 0) { renderer.render(scene, activeCam); _renderFrames--; }
}
requestAnimationFrame(frame);

// Buttons
document.getElementById('b-start').onclick=()=>{pauseSim();applySimT(0);if((parsedData.steps&&parsedData.steps.length))applyStep(0);else setStatus('stopped','STOPPED');};
document.getElementById('b-end').onclick=()=>{pauseSim();applySimT(Math.max(0,parsedData.positions.length-1));const last=((parsedData.steps&&parsedData.steps.length)||1)-1;if((parsedData.steps&&parsedData.steps.length))applyStep(last);else setStatus('stopped','STOPPED');};
document.getElementById('b-stop').onclick=()=>{pauseSim();setStatus('paused','PAUSED');};
// Abspielen; bei geändertem Programm erst parsen (IK), danach automatisch starten
let _playAfterParse=0;
function startPlay(dir){
  const N=parsedData.positions.length; if(!N)return;
  sim.vIst=undefined;
  if(dir>0&&sim.t>=N-1)applySimT(0);
  if(dir<0&&sim.t<=0)applySimT(N-1);
  sim.playing=true;sim.dir=dir;sim.stepTarget=null;
  document.getElementById('b-playfwd').classList.toggle('on',dir>0);
  document.getElementById('b-playrev').classList.toggle('on',dir<0);
  setStatus('playing',dir>0?'▶ FORWARD':'◀ BACKWARD');
}
function togglePlay(dir){
  if(sim.playing&&sim.dir===dir){pauseSim();setStatus('paused','PAUSED');return;}
  if(_codeEl.value!==_parsedCode||_ikPending){_playAfterParse=dir;if(!_ikPending)parseAndLoad();return;}
  startPlay(dir);
}
document.getElementById('b-playfwd').onclick=()=>togglePlay(1);
document.getElementById('b-playrev').onclick=()=>togglePlay(-1);
document.getElementById('b-stepfwd').onclick=()=>{if(!(parsedData.steps&&parsedData.steps.length))return;pauseSim();applyStep(sim.stepIdx+1);};
document.getElementById('b-steprev').onclick=()=>{if(!(parsedData.steps&&parsedData.steps.length))return;pauseSim();applyStep(sim.stepIdx-1);};
document.getElementById('spd-s').addEventListener('input',function(){document.getElementById('spd-v').textContent=this.value+'%';});
document.getElementById('pos-s').addEventListener('mousedown',()=>{if(sim.playing){pauseSim();setStatus('paused','PAUSED');}});
document.getElementById('pos-s').addEventListener('input',function(){const posIdx=Math.round(parseFloat(this.value));applySimT(posIdx);{const si=parsedData.stepOfPos?parsedData.stepOfPos[posIdx]:-1;if(si>=0)applyStep(si);}});

window.addEventListener('keydown',e=>{
  if(e.key==='F5'){e.preventDefault();document.getElementById('b-playfwd').click();return;}
  if(e.key==='F6'){e.preventDefault();document.getElementById('b-stop').click();return;}
  if(e.key==='F7'){e.preventDefault();document.getElementById('b-stepfwd').click();return;}
  if(e.key==='F8'){e.preventDefault();document.getElementById('b-steprev').click();return;}
  if(e.key==='F9'){e.preventDefault();document.getElementById('parse-btn').click();return;}
  if(e.target===document.getElementById('code-input'))return;
  if(e.key===' '){e.preventDefault();document.getElementById('b-stop').click();}
  if(e.key==='Home'){e.preventDefault();document.getElementById('b-start').click();}
  if(e.key==='End'){e.preventDefault();document.getElementById('b-end').click();}
});
// ═══════════════════════════════════════════════════
// PATH BUILDING
// ═══════════════════════════════════════════════════
const FRAME_LIMIT = 300;   // bis zu dieser Punktzahl: Koordinatenkreuz je Zielpunkt, darüber Punktwolke

// Gruppe leeren und eigene Geometrien/Materialien freigeben (ArrowHelper-Geometrie ist geteilt)
function _disposeGroup(grp){
  grp.traverse(o=>{
    if(o.isArrowHelper){ o.line.material.dispose(); o.cone.material.dispose(); return; }
    if(o.parent&&o.parent.isArrowHelper) return;
    if(o.geometry) o.geometry.dispose();
    if(o.material) o.material.dispose();
  });
  grp.clear();
}

function buildScene(positions){ requestRender();
  _disposeGroup(posGrp);_disposeGroup(pathGrp);
  visitedLine=visitedHead=plannedLine=null;markerGrp.visible=false;
  if(!positions.length)return;
  const N=positions.length, arr=new Float32Array(N*3);
  positions.forEach((p,i)=>{arr[i*3]=p.X;arr[i*3+1]=p.Y;arr[i*3+2]=p.Z;});
  if(N>1){const g=new THREE.BufferGeometry();g.setAttribute('position',new THREE.BufferAttribute(arr,3));
    plannedLine=new THREE.Line(g,new THREE.LineBasicMaterial({color:PLANNED_COL,linewidth:pathLineWidth}));
    plannedLine.frustumCulled=false;pathGrp.add(plannedLine);}
  if(N<=FRAME_LIMIT){
    positions.forEach((pos,i)=>{const g=makeFrame(pos);g.userData.posIdx=i;posGrp.add(g);});
  }else{
    const col=new Float32Array(N*3),c=new THREE.Color();
    positions.forEach((p,i)=>{c.setHex(TYPE_COL[p.type]!==undefined?TYPE_COL[p.type]:0xffffff);col[i*3]=c.r;col[i*3+1]=c.g;col[i*3+2]=c.b;});
    const g=new THREE.BufferGeometry();
    g.setAttribute('position',new THREE.BufferAttribute(arr.slice(),3));
    g.setAttribute('color',new THREE.BufferAttribute(col,3));
    posGrp.add(new THREE.Points(g,new THREE.PointsMaterial({size:4,sizeAttenuation:false,vertexColors:true})));
  }
  const box=new THREE.Box3();box.setFromArray(arr);
  const ctr=new THREE.Vector3();box.getCenter(ctr);const span=box.getSize(new THREE.Vector3()).length();
  orbitTarget.copy(ctr);orbitState.radius=Math.max(800,span*1.9);updateCamera();
  _applyPathMode(sim.t);
}

// ═══════════════════════════════════════════════════
// GUTTER (Zeilennummern, aktive Zeile)
// ═══════════════════════════════════════════════════
// Es werden nur die gerade sichtbaren Zeilennummern erzeugt (auch bei 50.000 Zeilen nur ~50 Elemente)
let _gutLines=0, _gutActive=-1, _gutKey='';
function _lineHeight(){ return parseFloat(getComputedStyle(_codeEl).lineHeight)||20; }
function renderGutter(){
  const lh=_lineHeight(), st=_codeEl.scrollTop;
  const first=Math.max(0,Math.floor(st/lh)), cnt=Math.ceil(_gutEl.clientHeight/lh)+2;
  const last=Math.min(_gutLines,first+cnt);
  const key=first+'|'+last+'|'+_gutActive+'|'+st;
  if(key===_gutKey)return; _gutKey=key;
  let html='<div style="transform:translateY('+(first*lh-st)+'px)">';
  for(let i=first;i<last;i++) html+='<div class="gl'+(i===_gutActive?' active':'')+'">'+(i+1)+'</div>';
  _gutEl.innerHTML=html+'</div>';
  // Balken über der aktiven Zeile im Textfeld (8 px = Innenabstand oben)
  const hl=document.getElementById('code-hl');
  if(hl){ if(_gutActive<0){hl.style.top='-100px';} else { const y=8+_gutActive*lh-st; hl.style.top=y+'px'; hl.style.height=lh+'px';
    hl.style.display=(y<-lh||y>_codeEl.clientHeight)?'none':''; } }
}
function buildGutter(count){ _gutLines=count; _gutKey=''; renderGutter(); }
function rebuildGutter(){
  const v=_codeEl.value; let n=1;
  for(let i=v.indexOf('\n');i>=0;i=v.indexOf('\n',i+1)) n++;
  buildGutter(n);
}
_codeEl.addEventListener('scroll',renderGutter);
// Cursorzeile im Editor → Roboter an diese Programmposition (Klick, Pfeiltasten, Bild auf/ab …)
let _edLine=-1;
function _editorCursorToSim(){
  if(!parsedData||!parsedData.posOfLine||_ikPending)return;
  const v=_codeEl.value, c=_codeEl.selectionStart;
  let line=0; for(let k=v.indexOf('\n');k>=0&&k<c;k=v.indexOf('\n',k+1)) line++;
  if(line===_edLine)return; _edLine=line;
  const L=parsedData.posOfLine; const idx=L[Math.min(line,L.length-1)];
  if(idx<0)return;
  if(sim.playing){pauseSim();setStatus('paused','PAUSED');}
  _edDriving=true;
  try{ applySimT(idx); const si=parsedData.stepOfPos?parsedData.stepOfPos[idx]:-1; if(si>=0)applyStep(si); }
  finally{ _edDriving=false; }
  _gutActive=line; _gutKey=''; renderGutter();
}
_codeEl.addEventListener('click',_editorCursorToSim);
_codeEl.addEventListener('keyup',e=>{ if(/^(Arrow|Page|Home|End|Enter|Backspace|Delete)/.test(e.key)) _editorCursorToSim(); });
_codeEl.addEventListener('keydown',e=>{ if(e.repeat&&/^(ArrowUp|ArrowDown|Page)/.test(e.key)) setTimeout(_editorCursorToSim,0); });
let _edDriving=false; // Positionswechsel kommt aus dem Editor: Editor nicht scrollen, Cursorzeile bleibt markiert
function updateGutterActive(lineNum){
  if(_edDriving)return;
  if(lineNum===_gutActive)return;
  _gutActive=lineNum;
  const lh=_lineHeight(), top=lineNum*lh;
  if(top<_codeEl.scrollTop||top>_codeEl.scrollTop+_codeEl.clientHeight-lh)_codeEl.scrollTop=Math.max(0,top-_codeEl.clientHeight/2);
  renderGutter();
}

// ═══════════════════════════════════════════════════
// PARSE & LOAD
// ═══════════════════════════════════════════════════
let _parsedCode=null, _ikPending=false;
function parseAndLoad(){
  const code=_codeEl.value;
  _parsedCode=code; _ikPending=true;
  parsedData=parseKRL(code);const N=parsedData.positions.length;
  // Schnellzugriff: Positionsindex -> Programmschritt
  parsedData.stepOfPos=new Int32Array(N).fill(-1);
  parsedData.steps.forEach((s,i)=>{if(s.type==='move'&&parsedData.stepOfPos[s.posIdx]<0)parsedData.stepOfPos[s.posIdx]=i;});
  _gutActive=-1;
  ensureBasesForProgram(); // Eintraege fuer alle referenzierten BASE_DATA[n] sicherstellen
  bakeBaseOffset(); // Code-Koordinaten + jeweils aktiver BASE-Offset pro Punkt -> Welt
  const _nLines=code.split(/\r?\n/).length;
  buildGutter(_nLines);
  // Programmzeile → Positionsindex (Zeilen ohne Bewegung: letzte Bewegung davor)
  parsedData.posOfLine=new Int32Array(_nLines).fill(-1);
  parsedData.positions.forEach((p,i)=>{if(p.lineNum!==undefined&&p.lineNum<_nLines&&parsedData.posOfLine[p.lineNum]<0)parsedData.posOfLine[p.lineNum]=i;});
  for(let l=1;l<_nLines;l++) if(parsedData.posOfLine[l]<0) parsedData.posOfLine[l]=parsedData.posOfLine[l-1];
  for(let l=0;l<_nLines&&parsedData.posOfLine[l]<0;l++) if(N) parsedData.posOfLine[l]=0; // Programmkopf → Anfang
  const ps=document.getElementById('pos-s');ps.min=0;ps.max=Math.max(0,N-1);ps.step=0.001;ps.value=0;
  document.getElementById('pos-v').textContent=N>0?`1 / ${N}`:'— / —';
  stopSim(0);setStatus('stopped','STOPPED');document.getElementById('marker-info').style.display='none';
  buildScene(parsedData.positions);
  // IK fuer alle Positionen vorab berechnen (haeppchenweise mit Fortschrittsleiste)
  setStatus('paused','IK lädt…');
  parseProgress(15, N + ' Punkte');
  setTimeout(()=>{
    computeIKTableProgressive(parsedData.positions, ()=>{
      _ikPending=false;
      if(N>0){applySimT(0);sim.stepIdx=0;if(parsedData.steps.length)applyStep(0);else setStatus('paused','BEREIT');}
      else setStatus('stopped','STOPPED');
      parseProgressHide();
      _ikJumpCheck();
      aPlotInvalidate();
      if(_playAfterParse){const d=_playAfterParse;_playAfterParse=0;startPlay(d);}
    });
  },10);
}
document.getElementById('parse-btn').addEventListener('click',function(){
  parseAndLoad();
});

// ═══════════════════════════════════════════════════
// INIT
// ═══════════════════════════════════════════════════
buildRobotModel([0,-90,90,0,0,0]);
baseNavTo(0);
document.getElementById('code-input').value=`$BASE = BASE_DATA[1]
$TOOL=TOOL_DATA[24]
$advance=5
$VEL.CP=0.167
PTP {A1 -19.000, A2 -56.735, A3 134.892, A4 -92.636, A5 89.448, A6 168.170}
$VEL.CP=0.003
LIN {X 730.000, Y 385.000, Z 10.000, A -73.693, B 0.000, C 180.000} C_DIS
LIN {X 730.000, Y 385.000, Z 0.000, A -73.693, B 0.000, C 180.000} C_DIS
LIN {X 1150.000, Y 385.000, Z 0.000, A -82.990, B 0.000, C 180.000} C_DIS
LIN {X 1150.000, Y -175.000, Z 0.000, A -110.153, B 0.000, C 180.000} C_DIS
LIN {X 730.000, Y -175.000, Z 0.000, A -114.981, B 0.000, C 180.000} C_DIS
LIN {X 730.000, Y 385.000, Z 0.000, A -73.693, B 0.000, C 180.000} C_DIS
$VEL.CP=0.167
LIN {X 730.000, Y 385.000, Z 10.000, A -73.693, B 0.000, C 180.000} C_DIS
PTP {A1 0.000, A2 -90.000, A3 90.000, A4 0.000, A5 0.000, A6 0.000}
END`;
// ── Editor-Breite per Ziehgriff ────────────────────────────
(function(){
  function makeResizer(handleId, getPanel, getDir) {
    var handle = document.getElementById(handleId);
    if (!handle) return;
    var dragging = false, startX = 0, startW = 0;
    handle.addEventListener('mousedown', function(e){
      var panel = getPanel();
      if (!panel) return;
      dragging = true; startX = e.clientX; startW = panel.offsetWidth;
      document.body.style.cursor = 'ew-resize';
      document.body.style.userSelect = 'none';
      e.preventDefault();
    });
    window.addEventListener('mousemove', function(e){
      if (!dragging) return;
      var panel = getPanel();
      if (!panel) return;
      var delta = (e.clientX - startX) * getDir();
      var w = Math.max(120, Math.min(800, startW + delta));
      panel.style.width = w + 'px';
      resize();
    });
    window.addEventListener('mouseup', function(){
      if (!dragging) return;
      dragging = false;
      document.body.style.cursor = '';
      document.body.style.userSelect = '';
      resize();
    });
  }
  // Left handle: editor
  makeResizer('ep-resize-handle', function(){ return document.querySelector('.ep'); }, function(){ return 1; });
})();
// ══════════════════════════════════════════════════════
// ROBOT MAP – Redundanzachse A (Drehung um Werkzeug-Z)
// x = Bahnlänge (links Beginn, rechts Ende), y = A-Wert -360…360°
// Felder: unerreichbar, Achsgrenze, Singularität. Grüne Linie = gewählter A-Verlauf,
// automatisch angelegt, Knoten verschiebbar. Die Linie legt A für ALLE Programmpunkte fest.
// ══════════════════════════════════════════════════════
const SING_BIT = { wrist:1, shoulder:2, elbow:4 };
const CELL_LIMIT = 8, CELL_REACH = 16, CELL_SING = 7, CELL_COLL = 32;
const AMAP_MIN = -360, AMAP_MAX = 360;
const AMAP_COL = {
  reach: [110, 18, 32],   // dunkelrot: keine IK-Lösung
  limit: [222, 40, 82],   // rot: nur außerhalb der Achsgrenzen lösbar
  sing:  [235, 196, 50],  // gelb: singulär
  coll:  [255, 140, 26],  // orange: Kollision Roboter/Werkzeug mit gedrucktem Teil
};
const AMAP_KIND_TXT = { 1:'unerreichbar', 2:'Achsgrenze', 3:'Singularität', 4:'Achssprung', 5:'Kollision' };
const AMAP_KIND_COL = { 1:'#ff4d6a', 2:'#ff4d6a', 3:'#ebc432', 4:'#ff9b3d', 5:'#ff8c1a' };

var _aPlotDists = [];
var _aPlotEdits = {};
var _aMap = null;          // { sig, L, astep, nA, samples:[{d, code:Uint8Array}] }
var _aPath = null;         // grüne Linie: [{d, a}] nach d sortiert, erster d=0, letzter d=L
var _aVerify = null;       // { bad:Int32Array, kind:Uint8Array }
var _aLayers = { reach:true, limit:true, sing:true, coll:true, prog:true, path:true };
var _aScanId = 0, _aVerifyId = 0;
var _aPlotVersion = 0, _aPlotCache = null, _aPlotCacheVer = -1, _aMapImg = null, _aMapImgKey = '';
var _aG = { ML:54, MT:10, MR:12, MB:46, W:0, H:0, CW:0, CH:0, dpr:1 };
var _aDrag = null;         // { idx } Knoten, der gezogen wird
var _aHover = null;        // { x, y }

function _aPos() { return (parsedData && parsedData.positions) ? parsedData.positions : []; }
function _aTotal() { return _aPlotDists.length ? (_aPlotDists[_aPlotDists.length-1] || 1) : 1; }
function aPlotMsg(txt) { var el = document.getElementById('aplot-msg'); if (el) el.textContent = txt; }

function _aPlotCalcDists(pos) {
  _aPlotDists = new Array(pos.length);
  if (!pos.length) return;
  _aPlotDists[0] = 0;
  for (let i = 1; i < pos.length; i++) {
    var dx = pos[i].X - pos[i-1].X, dy = pos[i].Y - pos[i-1].Y, dz = pos[i].Z - pos[i-1].Z;
    _aPlotDists[i] = _aPlotDists[i-1] + Math.sqrt(dx*dx + dy*dy + dz*dz);
  }
}

// Kennung der Bahngeometrie: Karte bleibt gültig, solange sich X/Y/Z/B/C, Werkzeug und Roboter nicht ändern.
// Nur geänderte A-Werte (z. B. nach "Übernehmen") verwerfen die Karte nicht.
function _aSig(pos) {
  var h = pos.length | 0;
  function mix(v) { h = (Math.imul(h, 31) + Math.round(v * 100)) | 0; }
  for (let i = 0; i < pos.length; i++) { var p = pos[i]; mix(p.X); mix(p.Y); mix(p.Z); mix(p.B); mix(p.C); }
  mix(TCP_DEF.x); mix(TCP_DEF.y); mix(TCP_DEF.z); mix(TCP_DEF.a); mix(TCP_DEF.b); mix(TCP_DEF.c);
  JOINTS_DEF.forEach(function(j) { mix(j.min); mix(j.max); });
  return h;
}
function _aCheckMap() {
  if (!_aMap) return;
  if (_aMap.sig !== _aSig(_aPos())) {
    _aMap = null; _aPath = null; _aVerify = null; _aMapImg = null;
    aPlotMsg('Bahn geändert — MAP neu erzeugen');
  }
}

// Index des Bahnsegments, in dem die Bogenlänge d liegt
function _aSegAt(d) {
  var lo = 0, hi = _aPlotDists.length - 1;
  while (hi - lo > 1) { var m = (lo + hi) >> 1; if (_aPlotDists[m] <= d) lo = m; else hi = m; }
  return lo;
}
function _aNearestIdx(d) {
  var i = _aSegAt(d), N = _aPlotDists.length;
  if (i + 1 < N && Math.abs(_aPlotDists[i+1] - d) < Math.abs(d - _aPlotDists[i])) i++;
  return i;
}
// Pose auf der Bahn bei Bogenlänge d (linear zwischen den Programmpunkten)
function _aPoseAt(d, pos) {
  var i = _aSegAt(d), N = pos.length;
  if (i >= N - 1) return pos[N-1];
  var L0 = _aPlotDists[i], L1 = _aPlotDists[i+1], f = L1 > L0 ? (d - L0) / (L1 - L0) : 0;
  var p0 = pos[i], p1 = pos[i+1];
  function ang(a, b) { return a + shortestAngleDiff(a, b) * f; }
  return { X:p0.X+(p1.X-p0.X)*f, Y:p0.Y+(p1.Y-p0.Y)*f, Z:p0.Z+(p1.Z-p0.Z)*f, B:ang(p0.B,p1.B), C:ang(p0.C,p1.C) };
}
// A-Wert der grünen Linie bei Bogenlänge d
function _aPathA(d) {
  var P = _aPath; if (!P || !P.length) return 0;
  if (d <= P[0].d) return P[0].a;
  for (let k = 1; k < P.length; k++) {
    if (d <= P[k].d) { var f = P[k].d > P[k-1].d ? (d - P[k-1].d) / (P[k].d - P[k-1].d) : 1; return P[k-1].a + (P[k].a - P[k-1].a) * f; }
  }
  return P[P.length-1].a;
}

// IK-Klassifikation einer Pose bei vorgegebenem A: {code, angles}
// Achswerte der Simulation sind stetig fortgeführt (können außerhalb ±360° liegen):
// auf die 360°-Kopie innerhalb der Achsgrenzen zurückführen
function _aNormQ(q) {
  return q.map(function(v, j) {
    var w = v - 360 * Math.round(v / 360), lo = JOINTS_DEF[j].min, hi = JOINTS_DEF[j].max;
    if (w < lo && w + 360 <= hi) w += 360; else if (w > hi && w - 360 >= lo) w -= 360;
    return w;
  });
}
function _aStartQ(i) { return _aNormQ((ikTable && ikTable[i] && ikTable[i].angles) ? ikTable[i].angles : jointAngles); }

// ── Kollision: Roboterglieder und Werkzeug gegen das bis Punkt i gedruckte Teil ──
// Teil = Hülle (achsparallele Box) aller Bahnpunkte 0…i. Glieder Sockel→Flansch als Zylinder mit Radius R,
// Werkzeug Flansch→TCP mit Radius 20 mm ohne die untersten 40 mm (Düse berührt das Teil zwangsläufig).
var _aPP = null, _aPPpos = null;
function _aPartPrefix(pos) {
  if (_aPPpos === pos && _aPP) return _aPP;
  var N = pos.length, B = new Float64Array(N * 6), b = [Infinity, Infinity, Infinity, -Infinity, -Infinity, -Infinity];
  for (let i = 0; i < N; i++) {
    var p = pos[i];
    b[0] = Math.min(b[0], p.X); b[1] = Math.min(b[1], p.Y); b[2] = Math.min(b[2], p.Z);
    b[3] = Math.max(b[3], p.X); b[4] = Math.max(b[4], p.Y); b[5] = Math.max(b[5], p.Z);
    B.set(b, i * 6);
  }
  _aPP = B; _aPPpos = pos; return B;
}
function _aCollRadius() { var el = document.getElementById('aplot-coll'); return el ? Math.max(0, parseFloat(el.value) || 0) : 60; }
function _aCollides(angles, i) {
  var pos = _aPos(); if (i < 0 || i >= pos.length) return false;
  var B = _aPartPrefix(pos), o = i * 6, pts = fkAll(angles).pts, R = _aCollRadius();
  function hit(P, r) {
    var dx = Math.max(B[o] - P[0], 0, P[0] - B[o+3]), dy = Math.max(B[o+1] - P[1], 0, P[1] - B[o+4]), dz = Math.max(B[o+2] - P[2], 0, P[2] - B[o+5]);
    return dx*dx + dy*dy + dz*dz < r*r;
  }
  for (let k = 0; k < pts.length - 1; k++) {
    var A = pts[k], E = pts[k+1], tool = k === pts.length - 2;
    var L = Math.hypot(E[0]-A[0], E[1]-A[1], E[2]-A[2]); if (L < 1e-6) continue;
    var r = tool ? 20 : R, end = tool ? Math.max(0, L - 40) : L, n = Math.max(1, Math.ceil(end / 20));
    for (let s = 0; s <= n; s++) {
      var f = (end * s / n) / L;
      if (hit([A[0] + (E[0]-A[0])*f, A[1] + (E[1]-A[1])*f, A[2] + (E[2]-A[2])*f], r)) return true;
    }
  }
  return false;
}

// strict: Achsen laufen stetig von initQ weiter (keine ±360°-Ausweichkopie)
// iPart: Index des Bahnpunkts für die Kollisionsprüfung (-1 = keine)
function _aClassify(p, a, initQ, strict, iPart) {
  var analytic = ikAnalyticSupported();
  var an = analytic ? solveIKAnalytic(p.X, p.Y, p.Z, a, p.B, p.C, initQ, strict) : null;
  if (an && !an.ok) return { code: CELL_LIMIT, angles: null };
  if (analytic && !an) return { code: CELL_REACH, angles: null };   // geometrisch keine Lösung
  var res = (an && an.ok) ? an : solveIKPrecise(p.X, p.Y, p.Z, a, p.B, p.C, initQ);
  if (!res || !res.ok) return { code: CELL_REACH, angles: null };
  var m = classifySingTypes(res.angles).reduce(function(acc, k) { return acc | SING_BIT[k]; }, 0);
  if (iPart !== undefined && iPart >= 0 && _aCollides(res.angles, iPart)) m |= CELL_COLL;
  return { code: m, angles: res.angles };
}

// ── Layout / Panel ───────────────────────────────────────
function _aPlotFitWidth() {
  var vp = document.querySelector('.vp'), panel = document.getElementById('aplot-panel');
  if (!vp || !panel) return;
  var r = vp.getBoundingClientRect();
  panel.style.left = r.left + 'px';
  panel.style.width = r.width + 'px';
  if (panel.classList.contains('max')) { panel.style.top = r.top + 'px'; panel.style.height = r.height + 'px'; }
  else { panel.style.top = ''; panel.style.height = ''; }
  _aLayoutCanvas();
}
function _aLayoutCanvas() {
  var canvas = document.getElementById('aplot-canvas'), body = document.querySelector('#aplot-panel .aplot-body');
  if (!canvas || !body) return;
  var w = Math.max(300, body.clientWidth), h = Math.max(160, body.clientHeight);
  var dpr = window.devicePixelRatio || 1;
  if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
    canvas.width = Math.round(w * dpr); canvas.height = Math.round(h * dpr);
    canvas.style.width = w + 'px'; canvas.style.height = h + 'px';
  }
  _aG.W = w; _aG.H = h; _aG.dpr = dpr;
  _aG.CW = w - _aG.ML - _aG.MR; _aG.CH = h - _aG.MT - _aG.MB;
}
function toggleAPlot() {
  var p = document.getElementById('aplot-panel'); if (!p) return;
  var show = getComputedStyle(p).display === 'none';
  p.style.display = show ? 'flex' : 'none';
  document.getElementById('rib-aplot').classList.toggle('on', show);
  if (show) { _aPlotFitWidth(); aPlotInvalidate(); }
}
function aPlotToggleMax() {
  var p = document.getElementById('aplot-panel');
  p.classList.toggle('max');
  _aPlotFitWidth(); aPlotInvalidate();
}
function aPlotLayer(key, on) { _aLayers[key] = on; aPlotInvalidate(); }

// ── Zeichnen ─────────────────────────────────────────────
function _ax(d) { return _aG.ML + d / _aTotal() * _aG.CW; }
function _ay(a) { return _aG.MT + (AMAP_MAX - a) / (AMAP_MAX - AMAP_MIN) * _aG.CH; }
function _aXtoD(x) { return Math.max(0, Math.min(_aTotal(), (x - _aG.ML) / _aG.CW * _aTotal())); }
function _aYtoA(y) { return Math.max(AMAP_MIN, Math.min(AMAP_MAX, AMAP_MAX - (y - _aG.MT) / _aG.CH * (AMAP_MAX - AMAP_MIN))); }

// Karte als Bild: eine Spalte je Bildschirmpixel, Zeilen = A-Schritte. Mehrere Scanpunkte in einer
// Spalte werden zusammengefasst (schlechtester Zustand gewinnt), leere Spalten nehmen den nächsten Scanpunkt.
function _aColumnCodes(nCol) {
  var M = _aMap, S = M.samples, L = M.L, nA = M.nA;
  var out = new Uint8Array(nCol * nA), j = 0;
  for (let c = 0; c < nCol; c++) {
    var d0 = c / nCol * L, d1 = (c + 1) / nCol * L, any = false;
    while (j < S.length && S[j].d < d0) j++;
    for (let k = j; k < S.length && S[k].d < d1; k++) {
      any = true; var code = S[k].code;
      for (let s = 0; s < nA; s++) out[c*nA + s] |= code[s];
    }
    if (!any) {
      var dm = (d0 + d1) / 2, best = Math.min(j, S.length - 1);
      if (best > 0 && Math.abs(S[best-1].d - dm) < Math.abs(S[best].d - dm)) best--;
      out.set(S[best].code, c * nA);
    }
  }
  return out;
}
function _aMapImage(nCol) {
  var key = nCol + '|' + _aLayers.reach + _aLayers.limit + _aLayers.sing + _aLayers.coll;
  if (_aMapImg && _aMapImgKey === key) return _aMapImg;
  var nA = _aMap.nA, codes = _aColumnCodes(nCol);
  var cv = document.createElement('canvas'); cv.width = nCol; cv.height = nA;
  var cx = cv.getContext('2d'), img = cx.createImageData(nCol, nA), D = img.data;
  for (let c = 0; c < nCol; c++) for (let s = 0; s < nA; s++) {
    var code = codes[c*nA + s], col = null;
    if ((code & CELL_REACH) && _aLayers.reach) col = AMAP_COL.reach;
    else if ((code & CELL_LIMIT) && _aLayers.limit) col = AMAP_COL.limit;
    else if ((code & CELL_COLL) && _aLayers.coll) col = AMAP_COL.coll;
    else if ((code & CELL_SING) && _aLayers.sing) col = AMAP_COL.sing;
    if (!col) continue;
    var o = ((nA - 1 - s) * nCol + c) * 4;
    D[o] = col[0]; D[o+1] = col[1]; D[o+2] = col[2]; D[o+3] = 200;
  }
  cx.putImageData(img, 0, 0);
  _aMapImg = cv; _aMapImgKey = key;
  return cv;
}

function _aDrawStatic(c, pos) {
  var G = _aG, L = _aTotal();
  c.fillStyle = '#242426'; c.fillRect(0, 0, G.W, G.H);
  c.fillStyle = '#18181a'; c.fillRect(G.ML, G.MT, G.CW, G.CH);
  if (_aMap) {
    var img = _aMapImage(Math.max(1, Math.round(G.CW)));
    c.imageSmoothingEnabled = false;
    // Zeile s liegt bei A = MIN + s·astep: Bildzeilen um eine halbe Zeile versetzt zentrieren
    var rowH = G.CH / (AMAP_MAX - AMAP_MIN) * _aMap.astep;
    c.drawImage(img, G.ML, G.MT - rowH / 2, G.CW, G.CH + rowH);
    c.fillStyle = '#242426'; c.fillRect(G.ML, G.MT - rowH, G.CW, rowH); c.fillRect(G.ML, G.MT + G.CH, G.CW, rowH);
  }
  // Gitter A
  c.font = '10px ' + getComputedStyle(document.body).getPropertyValue('--mono'); c.textAlign = 'right';
  for (let deg = -360; deg <= 360; deg += 45) {
    var y = _ay(deg), major = deg % 90 === 0;
    c.strokeStyle = deg === 0 ? 'rgba(200,200,210,.35)' : major ? 'rgba(255,255,255,.08)' : 'rgba(255,255,255,.035)';
    c.lineWidth = 1;
    c.beginPath(); c.moveTo(G.ML, y); c.lineTo(G.ML + G.CW, y); c.stroke();
    if (major) { c.fillStyle = deg === 0 ? '#e2e2e4' : '#9a9a9f'; c.fillText(deg + '°', G.ML - 5, y + 3.5); }
  }
  // Gitter Bahnlänge
  var stepM = [0.01,0.02,0.05,0.1,0.2,0.5,1,2,5,10,20,50,100,200,500].find(function(s) { return L/1000/s <= 10; }) || 1000;
  c.textAlign = 'center';
  for (let m = 0; m <= L/1000 + 1e-9; m += stepM) {
    var x = _ax(m * 1000);
    c.strokeStyle = 'rgba(255,255,255,.06)'; c.beginPath(); c.moveTo(x, G.MT); c.lineTo(x, G.MT + G.CH); c.stroke();
    c.fillStyle = '#9a9a9f'; c.fillText((stepM < 1 ? m.toFixed(2) : Math.round(m)) + ' m', x, G.MT + G.CH + 34);
  }
  c.strokeStyle = '#505055'; c.lineWidth = 1; c.strokeRect(G.ML + .5, G.MT + .5, G.CW - 1, G.CH - 1);
  // Problemstreifen unter der Karte
  c.fillStyle = '#1b1b1d'; c.fillRect(G.ML, G.MT + G.CH + 6, G.CW, 10);
  c.save(); c.translate(12, G.MT + G.CH / 2); c.rotate(-Math.PI/2);
  c.fillStyle = '#9a9a9f'; c.textAlign = 'center'; c.fillText('A [°]', 0, 0); c.restore();
  if (!pos.length) {
    c.fillStyle = '#9a9a9f'; c.font = '13px sans-serif'; c.fillText('Kein Programm geladen', G.ML + G.CW/2, G.MT + G.CH/2);
    return;
  }
  // Programm-A (grau). Sprünge über 180° (Umbruch ±180) unterbrechen die Linie.
  if (_aLayers.prog) {
    c.strokeStyle = 'rgba(200,204,214,.75)'; c.lineWidth = 1; c.setLineDash([3, 3]);
    c.beginPath();
    for (let i = 0; i < pos.length; i++) {
      var px = _ax(_aPlotDists[i]), py = _ay(pos[i].A);
      if (i === 0 || Math.abs(pos[i].A - pos[i-1].A) > 180) c.moveTo(px, py); else c.lineTo(px, py);
    }
    c.stroke(); c.setLineDash([]);
  }
  // Verifikation: kritische Punkte im Streifen
  if (_aVerify && _aVerify.bad.length) {
    var B = _aVerify.bad, K = _aVerify.kind;
    for (let k = 0; k < B.length; k++) {
      c.fillStyle = AMAP_KIND_COL[K[k]] || '#ff4d6a';
      c.fillRect(Math.floor(_ax(_aPlotDists[B[k]])), G.MT + G.CH + 6, 2, 10);
    }
  }
}

function _aDrawPath(c) {
  if (!_aPath || !_aLayers.path) return;
  c.strokeStyle = '#3ddc84'; c.lineWidth = 2;
  c.beginPath();
  _aPath.forEach(function(n, k) { var x = _ax(n.d), y = _ay(n.a); if (k) c.lineTo(x, y); else c.moveTo(x, y); });
  c.stroke();
  _aPath.forEach(function(n, k) {
    var x = _ax(n.d), y = _ay(n.a), act = _aDrag && _aDrag.idx === k;
    c.fillStyle = act ? '#ffffff' : '#d8f03c';
    c.beginPath(); c.arc(x, y, act ? 5 : 3.5, 0, 2*Math.PI); c.fill();
  });
}

function _aDrawCursor(c, pos) {
  if (typeof sim === 'undefined' || pos.length < 2) return;
  var N = pos.length, i0 = Math.max(0, Math.min(N-1, Math.floor(sim.t))), i1 = Math.min(i0+1, N-1), fr = sim.t - i0;
  var d = _aPlotDists[i0] + (_aPlotDists[i1] - _aPlotDists[i0]) * fr, x = _ax(d);
  c.strokeStyle = 'rgba(85,174,244,.7)'; c.lineWidth = 1; c.setLineDash([4, 3]);
  c.beginPath(); c.moveTo(x, _aG.MT); c.lineTo(x, _aG.MT + _aG.CH); c.stroke(); c.setLineDash([]);
  var a = pos[i0].A;
  c.fillStyle = '#55aef4'; c.beginPath(); c.arc(x, _ay(a), 4, 0, 2*Math.PI); c.fill();
  if (!_aHover) {
    var info = document.getElementById('aplot-info');
    if (info) info.textContent = 'Pos ' + (i0+1) + '/' + N + ' · ' + (d/1000).toFixed(3) + ' m · A ' + a.toFixed(2) + '°';
  }
}

function aPlotDraw() {
  var canvas = document.getElementById('aplot-canvas'), panel = document.getElementById('aplot-panel');
  if (!canvas || !panel || getComputedStyle(panel).display === 'none') return;
  if (!_aG.W) _aLayoutCanvas();
  var pos = _aPos(), G = _aG, dpr = G.dpr;
  if (_aPlotDists.length !== pos.length) _aPlotCalcDists(pos);
  var pw = Math.round(G.W * dpr), ph = Math.round(G.H * dpr);
  if (!_aPlotCache || _aPlotCache.width !== pw || _aPlotCache.height !== ph || _aPlotCacheVer !== _aPlotVersion) {
    if (!_aPlotCache) _aPlotCache = document.createElement('canvas');
    _aPlotCache.width = pw; _aPlotCache.height = ph; _aPlotCacheVer = _aPlotVersion;
    var c2 = _aPlotCache.getContext('2d');
    c2.setTransform(dpr, 0, 0, dpr, 0, 0);
    _aDrawStatic(c2, pos);
  }
  var ctx = canvas.getContext('2d');
  ctx.setTransform(1, 0, 0, 1, 0, 0);
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.drawImage(_aPlotCache, 0, 0);
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  if (!pos.length) return;
  _aDrawPath(ctx);
  _aDrawCursor(ctx, pos);
}
function aPlotInvalidate() {
  _aPlotCalcDists(_aPos());
  _aCheckMap();
  _aMapImg = null;
  _aPlotVersion++;
  aPlotDraw();
}

// ── MAP erzeugen ─────────────────────────────────────────
// Scanpunkte gleichmäßig über die Bahnlänge, innerhalb jedes Abschnitts versetzt (Goldener Schnitt),
// damit periodische Bahnen (z. B. Ecken jeder Lage) nicht immer an derselben Stelle getroffen werden.
function aPlotStartScan() {
  var pos = _aPos();
  if (!pos.length) { aPlotMsg('Kein Programm geladen'); return; }
  _aPlotCalcDists(pos);
  var L = _aTotal(), N = pos.length;
  var resSel = document.getElementById('aplot-res').value;
  var astep = parseFloat(document.getElementById('aplot-astep').value) || 3;
  var nA = Math.round((AMAP_MAX - AMAP_MIN) / astep) + 1;
  var S = resSel === 'all' ? N : resSel === 'auto' ? Math.min(N, 1000) : Math.min(N, parseInt(resSel, 10));
  var targets = [];
  if (S >= N) { for (let i = 0; i < N; i++) targets.push({ d:_aPlotDists[i], p:pos[i] }); }
  else {
    targets.push({ d:0, p:pos[0] });
    for (let j = 0; j < S; j++) { var d = (j + ((j * 0.6180339887) % 1)) / S * L; targets.push({ d:d, p:_aPoseAt(d, pos) }); }
    targets.push({ d:L, p:pos[N-1] });
    targets.sort(function(a, b) { return a.d - b.d; });
  }
  // Programm-A stetig fortgesetzt (Umbruch ±180° entfernt), auf den Kartenbereich begrenzt
  var anchorA = new Float64Array(N); anchorA[0] = pos[0].A;
  for (let i = 1; i < N; i++) { var dA = pos[i].A - pos[i-1].A; anchorA[i] = anchorA[i-1] + dA - 360 * Math.round(dA / 360); }
  for (let i = 0; i < N; i++) { while (anchorA[i] > AMAP_MAX) anchorA[i] -= 360; while (anchorA[i] < AMAP_MIN) anchorA[i] += 360; }
  var scanId = ++_aScanId, samples = [], si = 0, prevQs = null;
  var overlay = document.getElementById('aplot-scan-overlay'), bar = document.getElementById('aplot-scan-bar');
  var pctEl = document.getElementById('aplot-scan-pct'), msgEl = document.getElementById('aplot-scan-msg');
  if (overlay) overlay.style.display = 'flex';
  aPlotMsg('');
  var t0 = performance.now();
  function step() {
    if (scanId !== _aScanId) return;
    var tEnd = performance.now() + 15;
    do {
      // Achsstellungen laufen stetig entlang der Bahn: jede Zeile startet mit der Achsstellung derselben Zeile
      // der vorigen Spalte. Eine waagerechte Linie in der Karte entspricht so genau der Roboterbewegung.
      // Zeilen ohne Vorgänger werden stetig von der Nachbarzeile aus gefüllt; fehlt alles (erste Spalte oder
      // Spalte nach vollständig gesperrter Spalte), beginnt die Spalte bei der Achsstellung des Programms.
      var t = targets[si], p = t.p, code = new Uint8Array(nA), done = new Uint8Array(nA), qs = new Array(nA).fill(null), r, ic = _aNearestIdx(t.d);
      var havePrev = prevQs && prevQs.some(function(q) { return q; });
      if (havePrev) {
        for (let s = 0; s < nA; s++) if (prevQs[s]) {
          r = _aClassify(p, AMAP_MIN + s * astep, prevQs[s], true, ic);
          code[s] = r.code; done[s] = 1; qs[s] = r.angles;
        }
      } else {
        // Anfangsspalte: jede Zeile darf mit beliebiger A6-Umdrehung beginnen (legt die Anfahrbewegung fest)
        var i0 = _aNearestIdx(t.d), s0 = Math.max(0, Math.min(nA - 1, Math.round((anchorA[i0] - AMAP_MIN) / astep)));
        for (let dir = 1; dir >= -1; dir -= 2) {
          var qa = _aStartQ(i0);
          for (let s = dir > 0 ? s0 : s0 - 1; s >= 0 && s < nA; s += dir) {
            r = _aClassify(p, AMAP_MIN + s * astep, qa, false, ic);
            code[s] = r.code; done[s] = 1; qs[s] = r.angles; if (r.angles) qa = r.angles;
          }
        }
      }
      for (let s = 1; s < nA; s++) if (!done[s] && qs[s-1]) {
        r = _aClassify(p, AMAP_MIN + s * astep, qs[s-1], true, ic); code[s] = r.code; done[s] = 1; qs[s] = r.angles;
      }
      for (let s = nA - 2; s >= 0; s--) if (!done[s] && qs[s+1]) {
        r = _aClassify(p, AMAP_MIN + s * astep, qs[s+1], true, ic); code[s] = r.code; done[s] = 1; qs[s] = r.angles;
      }
      // Zeilen ohne erreichbaren Nachbarn: von irgendeiner gültigen Stellung der Spalte aus prüfen
      var anyQ = qs.find(function(q) { return q; }) || _aStartQ(_aNearestIdx(t.d));
      for (let s = 0; s < nA; s++) if (!done[s]) {
        r = _aClassify(p, AMAP_MIN + s * astep, anyQ, true, ic); code[s] = r.code; qs[s] = r.angles;
      }
      prevQs = qs;
      samples.push({ d:t.d, code:code });
      si++;
    } while (si < targets.length && performance.now() < tEnd);
    var pct = Math.round(si / targets.length * 100);
    if (bar) bar.style.width = pct + '%';
    if (pctEl) pctEl.textContent = pct + '%';
    if (msgEl) msgEl.textContent = 'MAP: ' + si + ' / ' + targets.length + ' Bahnpunkte';
    if (si < targets.length) { setTimeout(step, 0); return; }
    if (overlay) overlay.style.display = 'none';
    _aMap = { sig:_aSig(pos), L:L, astep:astep, nA:nA, samples:samples };
    _aMapImg = null; _aVerify = null;
    aPlotMsg('MAP: ' + samples.length + ' Bahnpunkte × ' + nA + ' A-Werte in ' + ((performance.now() - t0)/1000).toFixed(1) + ' s');
    aPlotAutoPath();
  }
  setTimeout(step, 0);
}

// ── Automatische grüne Linie ─────────────────────────────
// 1) Karte in Spalten: gesperrt = unerreichbar, Achsgrenze oder singulär
// 2) DP über Spalten: kurze A-Änderungen, Abstand zu Störbereichen, keine Querung gesperrter Felder
// 3) Ergebnis zu wenigen Knoten vereinfachen (gerade Stücke, solange sie frei bleiben)
function aPlotAutoPath() {
  if (!_aMap) { aPlotMsg('Erst MAP erzeugen'); return; }
  var pos = _aPos(), L = _aMap.L, nA = _aMap.nA, astep = _aMap.astep;
  var C = Math.max(2, Math.min(_aMap.samples.length, 600));
  var codes = _aColumnCodes(C);
  var blocked = new Uint8Array(C * nA), pre = new Int32Array(C * (nA + 1));
  for (let c = 0; c < C; c++) for (let s = 0; s < nA; s++) {
    var b = codes[c*nA + s] ? 1 : 0;
    blocked[c*nA + s] = b;
    pre[c*(nA+1) + s + 1] = pre[c*(nA+1) + s] + b;
  }
  function blockedRange(c, s0, s1) { if (s0 > s1) { var t = s0; s0 = s1; s1 = t; } return pre[c*(nA+1) + s1 + 1] - pre[c*(nA+1) + s0]; }
  // Abstandsstrafe: Sicherheitsabstand in Grad zu gesperrten Feldern (in A-Richtung, eigene und Nachbarspalten)
  var safe = Math.max(0, parseFloat(document.getElementById('aplot-safe').value) || 0);
  var R = Math.round(safe / astep), pen = new Float32Array(C * nA);
  if (R > 0) for (let c = 0; c < C; c++) {
    for (let s = 0; s < nA; s++) {
      var dmin = R + 1;
      for (let cc = Math.max(0, c-1); cc <= Math.min(C-1, c+1); cc++)
        for (let k = 0; k <= R && k < dmin; k++)
          if ((s-k >= 0 && blocked[cc*nA + s-k]) || (s+k < nA && blocked[cc*nA + s+k])) { dmin = k; break; }
      if (dmin <= R) pen[c*nA + s] = (R + 1 - dmin) * 4;
    }
  }
  var BIG = 1e5, WDEV = 0.02, progA = new Float64Array(C);
  for (let c = 0; c < C; c++) progA[c] = pos[_aNearestIdx((c + 0.5) / C * L)].A;
  function nodeCost(c, s) {
    if (blocked[c*nA + s]) return BIG;
    var dev = Math.abs(AMAP_MIN + s*astep - progA[c]) / astep;
    return pen[c*nA + s] + WDEV * dev;
  }
  var cost = new Float64Array(nA), prev = new Int32Array(C * nA), nc = new Float64Array(nA);
  for (let s = 0; s < nA; s++) cost[s] = nodeCost(0, s);
  for (let c = 1; c < C; c++) {
    for (let s = 0; s < nA; s++) {
      var own = nodeCost(c, s), best = Infinity, bp = s;
      for (let s0 = 0; s0 < nA; s0++) {
        if (cost[s0] >= best) continue;
        var jump = Math.abs(s - s0), cst = cost[s0] + jump;
        if (jump && (blockedRange(c-1, s0, s) || blockedRange(c, s0, s))) cst += BIG;
        if (cst < best) { best = cst; bp = s0; }
      }
      nc[s] = best + own; prev[c*nA + s] = bp;
    }
    cost.set(nc);
  }
  var sEnd = 0;
  for (let s = 1; s < nA; s++) if (cost[s] < cost[sEnd]) sEnd = s;
  var rows = new Int32Array(C); rows[C-1] = sEnd;
  for (let c = C-1; c > 0; c--) rows[c-1] = prev[c*nA + rows[c]];
  var crosses = cost[sEnd] >= BIG;

  // Vereinfachen: vom aktuellen Knoten so weit wie möglich gerade, solange jede Spalte frei bleibt
  // und der Abstand zu Störbereichen nicht schlechter wird als auf der DP-Lösung
  function colD(c) { return c === 0 ? 0 : c === C-1 ? L : (c + 0.5) / C * L; }
  function rowA(s) { return AMAP_MIN + s * astep; }
  // Eine Gerade ist zulässig, wenn sie in keiner Spalte schlechter liegt als die DP-Lösung
  // (nicht gesperrt, wo die DP frei ist; nicht näher an Störbereichen)
  function lineOk(c0, c1) {
    var a0 = rowA(rows[c0]), a1 = rowA(rows[c1]), last = rows[c0];
    for (let c = c0 + 1; c <= c1; c++) {
      var s = Math.round((a0 + (a1 - a0) * (c - c0) / (c1 - c0) - AMAP_MIN) / astep);
      var dpBlocked = blocked[c*nA + rows[c]];
      if (blocked[c*nA + s] > dpBlocked || pen[c*nA + s] > pen[c*nA + rows[c]] + 1e-6) return false;
      if (!dpBlocked && s !== last && (blockedRange(c-1, last, s) || blockedRange(c, last, s))) return false;
      last = s;
    }
    return true;
  }
  var nodes = [{ d:0, a:rowA(rows[0]) }], i = 0;
  while (i < C - 1) {
    var j = i + 1;
    while (j + 1 < C && lineOk(i, j + 1)) j++;
    nodes.push({ d:colD(j), a:rowA(rows[j]) });
    i = j;
  }
  _aPath = nodes;
  aPlotMsg(crosses ? 'Kein Weg ohne Störbereiche gefunden — bestmögliche Linie, kritische Stellen siehe Streifen'
                   : 'Weg automatisch angelegt (' + nodes.length + ' Knoten)');
  _aPathChanged();
}

// Grüne Linie aus dem Programm-A übernehmen (zum Nacharbeiten): Programm-A stetig fortgesetzt,
// dann auf wenige Knoten reduziert (Abweichung ≤ 0,5°)
function aPlotPathFromProgram() {
  var pos = _aPos(); if (!pos.length) return;
  _aPlotCalcDists(pos);
  var U = [pos[0].A];
  for (let i = 1; i < pos.length; i++) { var d = pos[i].A - pos[i-1].A; U.push(U[i-1] + d - 360 * Math.round(d / 360)); }
  var keep = new Uint8Array(pos.length); keep[0] = keep[pos.length-1] = 1;
  var stack = [[0, pos.length-1]];
  while (stack.length) {
    var seg = stack.pop(), a = seg[0], b = seg[1], worst = -1, wd = 0.5;
    for (let i = a + 1; i < b; i++) {
      var f = (_aPlotDists[i] - _aPlotDists[a]) / ((_aPlotDists[b] - _aPlotDists[a]) || 1);
      var e = Math.abs(U[i] - (U[a] + (U[b] - U[a]) * f));
      if (e > wd) { wd = e; worst = i; }
    }
    if (worst >= 0) { keep[worst] = 1; stack.push([a, worst], [worst, b]); }
  }
  var nodes = [];
  for (let i = 0; i < pos.length; i++) if (keep[i]) nodes.push({ d:_aPlotDists[i], a:Math.max(AMAP_MIN, Math.min(AMAP_MAX, U[i])) });
  if (nodes.length > 1 && nodes[nodes.length-1].d === nodes[nodes.length-2].d) nodes.pop();
  _aPath = nodes;
  aPlotMsg('Linie aus Programm (' + nodes.length + ' Knoten)');
  _aPathChanged();
}

// ── Verifikation: jeden Programmpunkt mit dem A der grünen Linie prüfen ──
function _aPathChanged() {
  _aVerify = null;
  var hint = document.getElementById('aplot-edit-hint');
  if (hint) hint.style.display = _aPath ? '' : 'none';
  aPlotInvalidate();
  _aVerifyStart();
}
function _aVerifyStart() {
  var el = document.getElementById('aplot-verify');
  var pos = _aPos();
  if (!_aPath || !pos.length) { if (el) { el.textContent = '—'; el.className = ''; } return; }
  var id = ++_aVerifyId, N = pos.length, i = 0;
  var q = _aStartQ(0);
  var bad = [], kind = [];
  if (el) { el.textContent = 'prüfe…'; el.className = 'run'; }
  function step() {
    if (id !== _aVerifyId) return;
    var tEnd = performance.now() + 15;
    for (; i < N && performance.now() < tEnd; i++) {
      var p = pos[i], r = _aClassify(p, _aPathA(_aPlotDists[i]), q, i > 0, i), k = 0;
      if (r.code & CELL_REACH) k = 1; else if (r.code & CELL_LIMIT) k = 2; else if (r.code & CELL_COLL) k = 5; else if (r.code & CELL_SING) k = 3;
      if (r.angles) {
        if (!k && i > 0) { for (let j = 0; j < 6; j++) if (Math.abs(r.angles[j] - q[j]) > 45) { k = 4; break; } }
        q = r.angles;
      }
      if (k) { bad.push(i); kind.push(k); }
    }
    if (i < N) { if (el) el.textContent = 'prüfe… ' + Math.round(i / N * 100) + '%'; setTimeout(step, 0); return; }
    _aVerify = { bad:Int32Array.from(bad), kind:Uint8Array.from(kind) };
    if (el) {
      if (!bad.length) { el.textContent = 'OK — alle ' + N + ' Punkte'; el.className = 'ok'; }
      else {
        var cnt = {}; kind.forEach(function(k) { cnt[k] = (cnt[k] || 0) + 1; });
        el.textContent = bad.length + ' kritisch: ' + Object.keys(cnt).map(function(k) { return cnt[k] + ' ' + AMAP_KIND_TXT[k]; }).join(', ')
          + ' (Punkt ' + (bad[0]+1) + '–' + (bad[bad.length-1]+1) + ')';
        el.className = 'bad';
      }
    }
    _aPlotVersion++; aPlotDraw();
  }
  setTimeout(step, 0);
}

// ── Übernehmen / Zurücksetzen ────────────────────────────
function aPlotReset() {
  if (!_aMap) { _aPath = null; _aPathChanged(); return; }
  aPlotAutoPath();
}
function aPlotApply() {
  var pos = _aPos();
  if (!pos.length || !_aPath) { aPlotMsg('Keine Linie vorhanden'); return; }
  _aPlotCalcDists(pos);
  _aPlotEdits = {};
  for (let i = 0; i < pos.length; i++) {
    var a = Math.round(_aPathA(_aPlotDists[i]) * 1000) / 1000;
    if (Math.abs(a - pos[i].A) > 0.0005) _aPlotEdits[i] = a;
  }
  var nEd = Object.keys(_aPlotEdits).length;
  if (!nEd) { aPlotMsg('Programm entspricht bereits der Linie'); return; }
  var code = document.getElementById('code-input');
  var lines = code.value.split('\n');
  // A-Wert in der n-ten Koordinatengruppe {…} einer Zeile ersetzen; false wenn die Gruppe fehlt
  function setA(li, groupIdx, newA) {
    var n = -1, done = false;
    lines[li] = lines[li].replace(/\{[^}]*\}/g, function(g) {
      if (++n !== groupIdx) return g;
      done = true;
      return g.replace(/\bA\s*[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?/, 'A ' + newA.toFixed(3));
    });
    return done;
  }
  // LIN, LIN_EXTRUDE, SLIN, PTP: 1. Gruppe; CIRC/CIRC_EXTRUDE: Zielpunkt = 2. Gruppe bzw. Folgezeile "{…}"
  Object.keys(_aPlotEdits).forEach(function(idxStr) {
    var p = pos[+idxStr], newA = _aPlotEdits[idxStr];
    if (!p || p.lineNum === undefined) return;
    if (p.type !== 'CIRC') { setA(p.lineNum, 0, newA); return; }
    if (setA(p.lineNum, 1, newA)) return;
    for (var li = p.lineNum + 1; li < Math.min(p.lineNum + 3, lines.length); li++) {
      if (/^\s*\{/.test(lines[li])) { setA(li, 0, newA); return; }
    }
  });
  code.value = lines.join('\n');
  _aPlotEdits = {};
  document.getElementById('aplot-edit-hint').style.display = 'none';
  parseAndLoad();
  aPlotMsg('Übernommen: ' + nEd + ' A-Werte geändert');
}

// Roboter in der Szene auf A an Bahnpunkt idx stellen (Vorschau beim Ziehen)
function aPlotLivePreview(idx, newA) {
  var p = _aPos()[idx]; if (!p) return;
  var warmQ = (ikTable && ikTable[idx] && ikTable[idx].angles) ? ikTable[idx].angles : jointAngles.slice();
  var res = ikPreciseA(p.X, p.Y, p.Z, newA, p.B, p.C, warmQ);
  if (res && res.ok) applyAngles(res.angles);
}

// ── Maus: Knoten ziehen, Knoten auf Linie einfügen, Rechtsklick löscht, Klick setzt Simulationsposition ──
(function() {
  function local(e) { var r = document.getElementById('aplot-canvas').getBoundingClientRect(); return { x:e.clientX - r.left, y:e.clientY - r.top }; }
  function hitNode(m) {
    if (!_aPath || !_aLayers.path) return -1;
    var best = -1, bd = 9;
    _aPath.forEach(function(n, k) { var dd = Math.hypot(_ax(n.d) - m.x, _ay(n.a) - m.y); if (dd < bd) { bd = dd; best = k; } });
    return best;
  }
  function nearLine(m) { return _aPath && _aLayers.path && Math.abs(_ay(_aPathA(_aXtoD(m.x))) - m.y) < 6; }
  function cellInfo(d, a) {
    if (!_aMap) return '';
    var S = _aMap.samples, lo = 0, hi = S.length - 1;
    while (hi - lo > 1) { var mm = (lo + hi) >> 1; if (S[mm].d <= d) lo = mm; else hi = mm; }
    var k = Math.abs(S[hi].d - d) < Math.abs(d - S[lo].d) ? hi : lo;
    var code = S[k].code[Math.round((a - AMAP_MIN) / _aMap.astep)];
    if (code & CELL_REACH) return 'unerreichbar';
    if (code & CELL_LIMIT) return 'Achsgrenze';
    if (code & CELL_COLL) return 'Kollision mit Bauteil';
    if (code & CELL_SING) return 'singulär (' + ['wrist','shoulder','elbow'].filter(function(t) { return code & SING_BIT[t]; })
      .map(function(t) { return { wrist:'Handgelenk', shoulder:'Schulter', elbow:'Ellbogen' }[t]; }).join(', ') + ')';
    return 'frei';
  }
  function showHover(m) {
    var pos = _aPos(), info = document.getElementById('aplot-info');
    if (!pos.length || !info) return;
    if (m.x < _aG.ML || m.x > _aG.ML + _aG.CW || m.y < _aG.MT || m.y > _aG.MT + _aG.CH) { _aHover = null; return; }
    _aHover = m;
    var d = _aXtoD(m.x), a = _aYtoA(m.y), i = _aNearestIdx(d), p = pos[i];
    var txt = 'Pos ' + (i+1) + ' · Zeile ' + (p.lineNum !== undefined ? p.lineNum + 1 : '–') + ' · ' + (d/1000).toFixed(3) + ' m · A ' + a.toFixed(1) + '°';
    var ci = cellInfo(d, a); if (ci) txt += ' · ' + ci;
    if (_aPath) txt += ' · Linie ' + _aPathA(d).toFixed(1) + '°';
    info.textContent = txt;
  }
  document.addEventListener('DOMContentLoaded', function() {
    var cv = document.getElementById('aplot-canvas'); if (!cv) return;
    cv.addEventListener('mousedown', function(e) {
      if (e.button !== 0) return;
      var m = local(e), k = hitNode(m);
      if (k < 0 && nearLine(m)) {
        var d = _aXtoD(m.x);
        k = _aPath.findIndex(function(n) { return n.d > d; });
        if (k <= 0) return;
        _aPath.splice(k, 0, { d:d, a:_aPathA(d) });
      }
      if (k >= 0) { _aDrag = { idx:k }; cv.style.cursor = 'grabbing'; e.preventDefault(); aPlotDraw(); return; }
      // Klick in die Karte: Simulation an diese Bahnstelle setzen
      var pos = _aPos();
      if (pos.length && m.x >= _aG.ML && m.x <= _aG.ML + _aG.CW) {
        var i = _aNearestIdx(_aXtoD(m.x));
        if (sim.playing) { pauseSim(); setStatus('paused', 'PAUSED'); }
        applySimT(i);
        var si = parsedData.stepOfPos ? parsedData.stepOfPos[i] : -1;
        if (si >= 0) applyStep(si);
      }
    });
    cv.addEventListener('contextmenu', function(e) {
      var k = hitNode(local(e));
      if (k > 0 && k < _aPath.length - 1) { e.preventDefault(); _aPath.splice(k, 1); _aPathChanged(); }
    });
    cv.addEventListener('mousemove', function(e) {
      var m = local(e);
      if (!_aDrag) { cv.style.cursor = hitNode(m) >= 0 ? 'grab' : nearLine(m) ? 'copy' : 'crosshair'; }
      showHover(m); if (!_aDrag) aPlotDraw();
    });
    cv.addEventListener('mouseleave', function() { _aHover = null; aPlotDraw(); });
    document.addEventListener('mousemove', function(e) {
      if (!_aDrag) return;
      var m = local(e), P = _aPath, k = _aDrag.idx, n = P[k];
      n.a = Math.round(_aYtoA(m.y) * 2) / 2;
      if (k > 0 && k < P.length - 1) {
        var gap = _aTotal() * 0.001;
        n.d = Math.max(P[k-1].d + gap, Math.min(P[k+1].d - gap, _aXtoD(m.x)));
      }
      var info = document.getElementById('aplot-info');
      if (info) info.textContent = 'Knoten ' + (k+1) + ' · ' + (n.d/1000).toFixed(3) + ' m · A ' + n.a.toFixed(1) + '°';
      aPlotLivePreview(_aNearestIdx(n.d), n.a);
      aPlotDraw();
    });
    document.addEventListener('mouseup', function() {
      if (!_aDrag) return;
      _aDrag = null; document.getElementById('aplot-canvas').style.cursor = 'crosshair';
      _aPathChanged();
    });
    // Höhe des Panels an der Oberkante ziehen
    var grip = document.getElementById('aplot-grip'), panel = document.getElementById('aplot-panel');
    if (grip) grip.addEventListener('mousedown', function(e) {
      e.preventDefault();
      var y0 = e.clientY, h0 = panel.getBoundingClientRect().height;
      function mv(ev) { panel.classList.remove('max'); panel.style.setProperty('--aplot-h', Math.max(220, Math.min(window.innerHeight - 120, h0 + y0 - ev.clientY)) + 'px'); _aPlotFitWidth(); aPlotInvalidate(); }
      function up() { document.removeEventListener('mousemove', mv); document.removeEventListener('mouseup', up); }
      document.addEventListener('mousemove', mv); document.addEventListener('mouseup', up);
    });
    if (window.ResizeObserver) new ResizeObserver(function() { _aLayoutCanvas(); _aPlotVersion++; aPlotDraw(); })
      .observe(document.querySelector('#aplot-panel .aplot-body'));
  });
})();

parseAndLoad();

window.addEventListener('load', function() {
  resize();
  setView('iso');
});
