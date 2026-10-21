const APP_VERSION = 'V0.84';
// On-Demand-Rendering: nur rendern wenn nötig (spart GPU/CPU/Akku im Leerlauf)
let _renderFrames = 4;
function requestRender(n) { _renderFrames = Math.max(_renderFrames, n || 2); }
// console.log nur mit ?debug in der URL (Produktiv still; warn/error bleiben)
if (!/[?&]debug\b/.test(location.search)) { const _noop = function(){}; console.log = _noop; console.debug = _noop; }


// ── Splash Screen ─────────────────────────────────────────────
function splashProgress(pct, msg) {
  var bar = document.getElementById('splash-bar');
  var txt = document.getElementById('splash-msg');
  if (bar) bar.style.width = pct + '%';
  if (txt && msg) txt.textContent = msg;
}

function splashHide() {
  var el = document.getElementById('splash-screen');
  if (!el) return;
  el.style.transition = 'opacity .5s ease';
  el.style.opacity = '0';
  setTimeout(function(){ el.style.display = 'none'; }, 520);
}

// Schrittweise Progress während Initialisierung
splashProgress(10, '3D Szene wird aufgebaut…');
'use strict';
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

function fkTCP_pos(angles_deg) {
  return fkAll(angles_deg).pts[7];
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

// IK nur mit Position + A6-Ziel — Orientierung frei
function solveIKPosA6(tx, ty, tz, a6target, initAngles) {
  const starts = [initAngles ? initAngles.slice() : jointAngles.slice()];
  starts.forEach(s => { s[5] = a6target; });
  return solveLM({
    tp:[tx,ty,tz], Rt:null, posOnly:true,
    starts,
    dt:0.3, lam:0.5, tolP:0.5, tolO:0,
    maxIter:300, stepMax:2.0, stepScale:10.0,
    earlyStop:0, okThresh:5,
    a6penalty:{target:a6target, weight:60},
    okExtra: q => Math.abs(q[5]-a6target) < 10,
  });
}

// IK mit A6-Zielwert als starke Soft-Constraint
function solveIKTargetA6(tx, ty, tz, ta, tb, tc, a6target, initAngles) {
  const starts = [
    initAngles ? initAngles.slice() : jointAngles.slice(),
    [0,-90,90,0,0,a6target],[0,-90,90,-90,0,a6target],[0,-90,90,90,0,a6target],
  ];
  starts.forEach(s => { s[5] = a6target; });
  return solveLM({
    tp:[tx,ty,tz], Rt:rotZYX(ta,tb,tc),
    starts,
    dt:0.3, lam:0.5, tolP:0.5, tolO:0.5,
    maxIter:200, stepMax:2.0, stepScale:10.0,
    earlyStop:1.5, okThresh:15,
    a6penalty:{target:a6target, weight:50},
  });
}

// IK mit fixiertem A6 — nur A1–A5 werden optimiert
function solveIKFixedA6(tx, ty, tz, ta, tb, tc, a6fixed, initAngles) {
  const starts = [
    initAngles ? initAngles.slice() : jointAngles.slice(),
    [0,-90,90,0,0,a6fixed],[0,-90,90,0,-45,a6fixed],
    [0,-90,90,-90,0,a6fixed],[0,-90,90,90,0,a6fixed],
  ];
  starts.forEach(s => { s[5] = a6fixed; });
  return solveLM({
    tp:[tx,ty,tz], Rt:rotZYX(ta,tb,tc),
    starts,
    dt:0.3, lam:0.5, tolP:1.0, tolO:1.0,
    maxIter:150, stepMax:2.0, stepScale:10.0,
    earlyStop:1.5, okThresh:20,
    a6fixed,
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
  if (typeof baseList === 'undefined' || !baseList) return null;
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
function _invRigid(M){ var R=[M[0],M[1],M[2],M[4],M[5],M[6],M[8],M[9],M[10]], t=[M[3],M[7],M[11]];
  var Rt=[R[0],R[3],R[6],R[1],R[4],R[7],R[2],R[5],R[8]];
  var ti=[-(Rt[0]*t[0]+Rt[1]*t[1]+Rt[2]*t[2]), -(Rt[3]*t[0]+Rt[4]*t[1]+Rt[5]*t[2]), -(Rt[6]*t[0]+Rt[7]*t[1]+Rt[8]*t[2])];
  return [Rt[0],Rt[1],Rt[2],ti[0], Rt[3],Rt[4],Rt[5],ti[1], Rt[6],Rt[7],Rt[8],ti[2], 0,0,0,1]; }
function _fromT(M){ var R=[M[0],M[1],M[2],M[4],M[5],M[6],M[8],M[9],M[10]], e=_eulZYX(R);
  return { X:M[3], Y:M[7], Z:M[11], A:e.A, B:e.B, C:e.C }; }
// Welt -> base-relativ (fuer Anzeige/Ausgabe)
function _baseMat(b){ return _poseT({X:b.x||0,Y:b.y||0,Z:b.z||0,A:b.a||0,B:b.b||0,C:b.c||0}); } // BASE-Felder sind klein geschrieben
function worldToBase(p, baseN){ var b=_baseByN(baseN);
  if(!b) return { X:p.X, Y:p.Y, Z:p.Z, A:p.A, B:p.B, C:p.C };
  return _fromT(_mul4(_invRigid(_baseMat(b)), _poseT(p))); }
// base-relativ -> Welt (fuer Speicherung/Eingabe)
function baseToWorld(p, baseN){ var b=_baseByN(baseN);
  if(!b) return { X:p.X, Y:p.Y, Z:p.Z, A:p.A, B:p.B, C:p.C };
  return _fromT(_mul4(_baseMat(b), _poseT(p))); }
// aktiver BASE-Index (BASE_DATA[n]) an einer Textzeile, durch Vorwaerts-Scan
function activeBaseAtLine(lineIdx){
  try { var lns=document.getElementById('code-input').value.split(/\r?\n/), cur=0;
    for(var i=0;i<lns.length && i<=lineIdx;i++){ var m=lns[i].match(/^\s*\$BASE\s*=\s*BASE_DATA\[(\d+)\]/i); if(m) cur=+m[1]; }
    return cur; } catch(e){ return 0; }
}

// Generiert KRL-Code aus parsedData (Umkehrung von parseKRL)
function generateKRL(pd) {
  var lines = [];
  if (!pd || (!pd.steps && !pd.positions)) return '';
  var positions = pd.positions || [];

  // Fallback: nur Positionen vorhanden
  if (!pd.steps || !pd.steps.length) {
    lines.push('$BASE = BASE_DATA[1]');
    lines.push('$TOOL = TOOL_DATA[1]');
    lines.push('$VEL.CP=0.167');
    positions.forEach(function(pos) {
      var posR = pos.rel || pos;
      var S = (pos.S != null ? ' S '+pos.S : ''), T = (pos.T != null ? ' T '+pos.T : '');
      lines.push('LIN {X '+posR.X.toFixed(3)+',Y '+posR.Y.toFixed(3)+',Z '+posR.Z.toFixed(3)+',A '+posR.A.toFixed(3)+',B '+posR.B.toFixed(3)+',C '+posR.C.toFixed(3)+S+T+'}');
    });
    return lines.join('\n');
  }

  var curBase = 0; // aktiver BASE (BASE_DATA[n]); 0 = Welt
  pd.steps.forEach(function(s) {
    switch (s.type) {
      case 'comment':  lines.push('; ' + (s.text || '')); break;
      case 'tool':     lines.push('$TOOL=TOOL_DATA[' + s.n + ']'); break;
      case 'base':     curBase = s.n; lines.push('$BASE=BASE_DATA[' + s.n + ']'); break;
      case 'velcp':    lines.push('$VEL.CP=' + parseFloat(s.v || 0.167).toFixed(3)); break;
      case 'velptp':   lines.push('$VEL.PTP=' + Math.round(s.v || 100)); break;
      case 'acccp':    lines.push('$ACC.CP=' + parseFloat(s.v || 1).toFixed(1)); break;
      case 'advance':  lines.push('$advance=' + Math.round(s.v != null ? s.v : 5)); break;
      case 'wait':     lines.push('WAIT SEC ' + parseFloat(s.t || 0).toFixed(1)); break;
      case 'halt':     lines.push('HALT'); break;
      case 'brake':    lines.push('BRAKE'); break;
      case 'dout':     lines.push('$OUT[' + s.n + ']=' + (s.v || 'FALSE')); break;
      case 'din':      lines.push('$IN[' + s.n + ']'); break;
      case 'aout':     lines.push('$ANOUT[' + s.n + ']=' + parseFloat(s.v || 0).toFixed(2)); break;
      case 'ain':      lines.push('$ANIN[' + s.n + ']'); break;
      case 'waitFor':  lines.push('WAIT FOR ' + (s.cond || '$IN[1]')); break;
      case 'calc': lines.push((s.target||'v') + ' = ' + (s.expr||'0')); break;
      case 'var': {
        var vt = s.varType || 'REAL';
        var vv = s.val != null ? s.val : (vt === 'BOOL' ? 'FALSE' : vt === 'INT' ? '0' : '0.0');
        lines.push('DECL ' + vt + ' ' + s.name + '=' + vv);
        break;
      }
      case 'ptpAxis':
        if (s.angles) lines.push('PTP {A1 '+s.angles[0].toFixed(3)+',A2 '+s.angles[1].toFixed(3)+',A3 '+s.angles[2].toFixed(3)+',A4 '+s.angles[3].toFixed(3)+',A5 '+s.angles[4].toFixed(3)+',A6 '+s.angles[5].toFixed(3)+'}');
        break;
      case 'move': {
        var pos = positions[s.posIdx];
        if (!pos) break;
        var posR = pos.rel || pos;
        var Sv = (pos.S != null ? ' S '+pos.S : ''), Tv = (pos.T != null ? ' T '+pos.T : '');
        var coord = '{X '+posR.X.toFixed(3)+',Y '+posR.Y.toFixed(3)+',Z '+posR.Z.toFixed(3)+',A '+posR.A.toFixed(3)+',B '+posR.B.toFixed(3)+',C '+posR.C.toFixed(3)+Sv+Tv+'}';
        lines.push((s.moveType || 'LIN') + ' ' + coord);
        break;
      }
      case 'circ': {
        var pVia = positions[s.viaIdx], pTo = positions[s.posIdx];
        if (pVia && pTo) {
          var pViaR = pVia.rel || pVia, pToR = pTo.rel || pTo;
          var cVia = '{X '+pViaR.X.toFixed(3)+',Y '+pViaR.Y.toFixed(3)+',Z '+pViaR.Z.toFixed(3)+',A '+pViaR.A.toFixed(3)+',B '+pViaR.B.toFixed(3)+',C '+pViaR.C.toFixed(3)+'}';
          var cTo  = '{X '+pToR.X.toFixed(3)+',Y '+pToR.Y.toFixed(3)+',Z '+pToR.Z.toFixed(3)+',A '+pToR.A.toFixed(3)+',B '+pToR.B.toFixed(3)+',C '+pToR.C.toFixed(3)+'}';
          lines.push('CIRC ' + cVia + ', ' + cTo);
        }
        break;
      }
    }
  });
  return lines.join('\n');
}
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
    const circExtM=line.match(/^CIRC_EXTRUDE\s*\(\s*(\{[^}]+\})\s*,\s*(\{[^}]+\})\s*,[^)]*\)/i);
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
    const linExtM=line.match(/^LIN_EXTRUDE\s*\(\s*\{([^}]+)\}\s*,[^)]*\)/i);
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
const renderer=new THREE.WebGLRenderer({canvas,antialias:true});
renderer.setPixelRatio(window.devicePixelRatio);
const scene=new THREE.Scene();
// Kabel-System initialisieren (Y-Achse = oben in RobSimul)
if(window.cableSystem) window.cableSystem.init(THREE, scene, { upAxis: 'y', viewerOnly: true });
scene.background=new THREE.Color(0x1e1e1e); // bg-pro default

const perspCam=new THREE.PerspectiveCamera(50,1,1,80000);
perspCam.up.set(0,0,1);
const orthoCam=new THREE.OrthographicCamera(-2000,2000,2000,-2000,-50000,50000);
orthoCam.up.set(0,0,1);
let activeCam=perspCam, orthoHalfSize=2000, currentView='iso';

// Lights
scene.add(new THREE.AmbientLight(0xffffff, 0.85));
const sun=new THREE.DirectionalLight(0xffffff,0.8);
sun.position.set(1000,1000,2000);scene.add(sun);
const fill=new THREE.DirectionalLight(0xffffff,0.4);fill.position.set(-1000,-500,1000);scene.add(fill);

// Grid (horizontal, Z=0)
const grid=new THREE.GridHelper(8000,40,0x0e1e30,0x0a1828);
grid.rotation.x=Math.PI/2;
// GridHelper uses LineSegments with array material
function setGridColor(col){
  if(Array.isArray(grid.material)){grid.material.forEach(m=>m.color.set(col));}
  else{grid.material.color.set(col);}
}
scene.add(grid);

// World origin frame
function wArrow(d,col,len,org){scene.add(new THREE.ArrowHelper(new THREE.Vector3(...d).normalize(),new THREE.Vector3(...org),len,col,len*.18,len*.09));}
wArrow([1,0,0],0xff3333,300,[0,0,0]);
wArrow([0,1,0],0x33ff33,300,[0,0,0]);
wArrow([0,0,1],0x3377ff,300,[0,0,0]);

// Groups
const posGrp=new THREE.Group();scene.add(posGrp);
const pathGrp=new THREE.Group();scene.add(pathGrp);
const stlGrp=new THREE.Group();scene.add(stlGrp);
const robotGrp=new THREE.Group();scene.add(robotGrp);
const tcpTraceGrp=new THREE.Group();scene.add(tcpTraceGrp);

// Simulation marker (white sphere + CS arrows, like KUKA sim)

// BASE Koordinatensystem (Weltkoordinaten)
// ── TCP Frame Groups — eines pro TCP-Eintrag ────────────────────
const tcpFrameGroups = [];

function syncTcpFrameGroups() {
  while (tcpFrameGroups.length < tcpList.length) {
    var idx = tcpFrameGroups.length;
    var grp = makeBaseFrameGroup(tcpList[idx] ? tcpList[idx].name : 'TCP '+(idx+1));
    grp.visible = showBaseFrame;
    scene.add(grp);
    tcpFrameGroups.push(grp);
  }
  while (tcpFrameGroups.length > tcpList.length) {
    scene.remove(tcpFrameGroups.pop());
  }
  // Flansch-Weltpose aus FK — identisch zu fkAll()
  var fkC = fkAll(jointAngles);
  var rot6 = fkC.rots[6];   // A6 Rohrotation
  var pos6 = fkC.pts[6];    // A6 Rohposition
  // Gleiche Flansch-Korrektur wie in fkAll: R_fc = Ry(90°) → Z vorwärts, X unten
  var R_fc = mRy(Math.PI/2);

  tcpList.forEach(function(t, i) {
    var grp = tcpFrameGroups[i];
    if (!grp) return;
    if (!t.name) t.name = 'TCP '+(i+1);
    var isActive = (i === tcpNavIdx);

    // TCP-Position: exakt wie fkAll für pts[7]
    var R_usr = rotZYX(t.a||0, t.b||0, t.c||0);
    var p_local = mVec(R_fc, [t.x||0, t.y||0, t.z||0]);
    var p_world = mVec(rot6, p_local);
    grp.position.set(pos6[0]+p_world[0], pos6[1]+p_world[1], pos6[2]+p_world[2]);

    // TCP-Rotation: exakt wie fkAll für tcp_rot
    var R_tcp_world = mMul(rot6, mMul(R_fc, R_usr));
    grp.quaternion.copy(_mat3ToQuat(R_tcp_world));

    updateBaseFrameLabel(grp, t.name, isActive);
    grp.traverse(function(obj) {
      if (obj.material) {
        obj.material.opacity = isActive ? 1.0 : 0.35;
        obj.material.transparent = !isActive;
      }
    });
    grp.visible = showBaseFrame;
    if (grp.userData.labelSprite) grp.userData.labelSprite.visible = showAxisLabels;
  });
}

// BASE Koordinatensystem — eines pro BASE-Eintrag
const baseFrameGrp = new THREE.Group(); // alias für Kompatibilität (zeigt auf aktiven)
const baseFrameGroups = []; // eine Gruppe pro BASE-Eintrag

function makeBaseFrameGroup(name) {
  var grp = new THREE.Group();
  var sz = 150;
  [[1,0,0,0xff3333],[0,1,0,0x33ff33],[0,0,1,0x3388ff]].forEach(function(d) {
    grp.add(new THREE.ArrowHelper(
      new THREE.Vector3(d[0],d[1],d[2]),
      new THREE.Vector3(0,0,0),
      sz, d[3], sz*.2, sz*.1
    ));
  });
  // Sprite-Label
  var canvas = document.createElement('canvas');
  canvas.width = 256; canvas.height = 64;
  var ctx = canvas.getContext('2d');
  ctx.fillStyle = 'rgba(0,0,0,0)';
  ctx.clearRect(0,0,256,64);
  ctx.font = 'bold 28px monospace';
  ctx.strokeStyle = 'rgba(0,0,0,0.85)';
  ctx.lineWidth = 5;
  ctx.lineJoin = 'round';
  ctx.strokeText(name || 'BASE', 4, 44);
  ctx.fillStyle = '#60a5fa';
  ctx.fillText(name || 'BASE', 4, 44);
  var tex = new THREE.CanvasTexture(canvas);
  tex.premultipliedAlpha = false;
  var sprite = new THREE.Sprite(new THREE.SpriteMaterial({map:tex,depthTest:false,transparent:true,alphaTest:0.01,depthWrite:false}));
  sprite.scale.set(180,44,1);
  sprite.position.set(20,0,-10);
  sprite.userData.isLabel = true;
  grp.add(sprite);
  grp.userData.labelSprite = sprite;
  grp.userData.labelCanvas = canvas;
  return grp;
}

function updateAxisLabelsVisibility() {
  [].concat(baseFrameGroups, tcpFrameGroups).forEach(function(grp) {
    if (grp && grp.userData.labelSprite) {
      grp.userData.labelSprite.visible = showAxisLabels;
    }
  });
}

function updateBaseFrameLabel(grp, name, isActive) {
  var canvas = grp.userData.labelCanvas;
  if (!canvas) return;
  var ctx = canvas.getContext('2d');
  ctx.clearRect(0,0,256,64);
  // transparenter Hintergrund für alle, aktives = heller
  ctx.font = 'bold ' + (isActive ? '30' : '26') + 'px monospace';
  ctx.strokeStyle = 'rgba(0,0,0,0.85)';
  ctx.lineWidth = 5;
  ctx.lineJoin = 'round';
  ctx.strokeText(name || 'BASE', 6, 44);
  ctx.fillStyle = isActive ? '#ffffff' : '#93c5fd';
  ctx.fillText(name || 'BASE', 6, 44);
  if (grp.userData.labelSprite) grp.userData.labelSprite.material.map.needsUpdate = true;
}

function syncBaseFrameGroups() {
  // Sicherstellen dass für jeden BASE-Eintrag eine Gruppe existiert
  while (baseFrameGroups.length < baseList.length) {
    var idx = baseFrameGroups.length;
    var grp = makeBaseFrameGroup(baseList[idx] ? baseList[idx].name : 'BASE '+(idx+1));
    grp.visible = showBaseFrame;
    scene.add(grp);
    baseFrameGroups.push(grp);
  }
  // Überschüssige entfernen
  while (baseFrameGroups.length > baseList.length) {
    var old = baseFrameGroups.pop();
    scene.remove(old);
  }
  // Labels + Position aktualisieren
  baseList.forEach(function(b, i) {
    var grp = baseFrameGroups[i];
    if (!grp) return;
    var r = Math.PI/180;
    grp.position.set(b.x||0, b.y||0, b.z||0);
    grp.rotation.set((b.c||0)*r, (b.b||0)*r, (b.a||0)*r, 'ZYX');
    if (!b.name) b.name = 'BASE '+(i+1);
    var isActive = (i === baseNavIdx);
    updateBaseFrameLabel(grp, b.name, isActive);
    // Aktiv: volle Helligkeit; inaktiv: gedimmt
    grp.traverse(function(obj) {
      if (obj.material) {
        obj.material.opacity = isActive ? 1.0 : 0.35;
        obj.material.transparent = !isActive;
      }
    });
    grp.visible = showBaseFrame;
    if (grp.userData.labelSprite) grp.userData.labelSprite.visible = showAxisLabels;
  });
}
scene.add(baseFrameGrp); // legacy (wird nicht mehr direkt genutzt)
baseFrameGrp.visible = false; // hide legacy group
const markerGrp=new THREE.Group();scene.add(markerGrp);
const markerVisuals=new THREE.Group();markerGrp.add(markerVisuals);
{
  const sp=new THREE.Mesh(new THREE.SphereGeometry(28,12,12),new THREE.MeshBasicMaterial({color:0xf05500,transparent:true,opacity:.9}));
  markerVisuals.add(sp);
  const ring=new THREE.Mesh(new THREE.TorusGeometry(36,5,6,24),new THREE.MeshBasicMaterial({color:0xff6600}));
  ring.rotation.x=Math.PI/2;markerVisuals.add(ring);
  for(const[d,c]of[[[1,0,0],0xff6666],[[0,1,0],0x66ff66],[[0,0,1],0x6699ff]])
    markerVisuals.add(new THREE.ArrowHelper(new THREE.Vector3(...d),new THREE.Vector3(),190,c,38,19));
  markerGrp.visible=false;
}
const selSphere=new THREE.Mesh(new THREE.SphereGeometry(34,12,12),new THREE.MeshBasicMaterial({color:0x00ccff,transparent:true,opacity:.35,depthTest:false}));
selSphere.visible=false;scene.add(selSphere);

const TYPE_COL={LIN:0xf05500,PTP:0xffaa00,SLIN:0x00aaff,CIRC:0xaa44ff,CIRC_AUX:0x445566};
let frameSize=40,sphereSize=7;
let traceLineWidth=2, pathLineWidth=2;  // Linienstärken
// SVG Icons für Visibility-Zustände
var svgIconSolid  = '<svg viewBox="0 0 24 24" width="16" height="16"><circle cx="12" cy="12" r="10" fill="#f58220"/></svg>';
var svgIconTransp = '<svg viewBox="0 0 24 24" width="16" height="16"><circle cx="12" cy="12" r="9" fill="none" stroke="#f58220" stroke-width="3"/></svg>';
var svgIconHidden = '<svg viewBox="0 0 24 24" width="16" height="16"><circle cx="12" cy="12" r="9" fill="none" stroke="#f58220" stroke-width="3"/><line x1="4" y1="20" x2="20" y2="4" stroke="#f58220" stroke-width="3" stroke-linecap="round"/></svg>';
var sceneSTLMode = { pedestal:'solid', tool:'solid' };
function getSTLVisIcon(m){return m==='solid'?svgIconSolid:m==='transparent'?svgIconTransp:svgIconHidden;}
function applyMeshMode(mesh,mode){if(!mesh)return;if(mode==='hidden'){mesh.visible=false;}else if(mode==='transparent'){mesh.visible=true;mesh.material.transparent=true;mesh.material.opacity=0.3;mesh.material.depthWrite=false;mesh.material.needsUpdate=true;}else{mesh.visible=true;mesh.material.transparent=false;mesh.material.opacity=1;mesh.material.depthWrite=true;mesh.material.needsUpdate=true;}}
function cycleSTLMode(type,idx){var c={solid:'transparent',transparent:'hidden',hidden:'solid'};var btn,mode;if(type==='axis'){axisSTLMode[idx]=c[axisSTLMode[idx]]||'solid';mode=axisSTLMode[idx];btn=document.getElementById('asl-vis'+idx);if(btn)btn.innerHTML=getSTLVisIcon(mode);applyMeshMode(axisSTLMeshes[idx],mode);}else if(type==='pedestal'){sceneSTLMode.pedestal=c[sceneSTLMode.pedestal]||'solid';mode=sceneSTLMode.pedestal;btn=document.getElementById('vis-pedestal');if(btn)btn.innerHTML=getSTLVisIcon(mode);applyMeshMode(pedestalMesh,mode);}else if(type==='tool'){sceneSTLMode.tool=c[sceneSTLMode.tool]||'solid';mode=sceneSTLMode.tool;btn=document.getElementById('vis-tool');if(btn)btn.innerHTML=getSTLVisIcon(mode);applyMeshMode(toolMesh,mode);}}
function setAxisSTLMode(idx,mode){axisSTLMode[idx]=mode;var btn=document.getElementById('asl-vis'+idx);if(btn)btn.innerHTML=getSTLVisIcon(mode);applyMeshMode(axisSTLMeshes[idx],mode);}
var axisSTLMode = ['solid','solid','solid','solid','solid','solid']; // solid|transparent|hidden

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

function rebuildFrames(){
  posGrp.clear(); posGrp.visible = showPosFrames;
  parsedData.positions.forEach((pos,i)=>{const g=makeFrame(pos);g.userData.posIdx=i;posGrp.add(g);});
  if(selectedPosIdx!==null){const p=parsedData.positions[selectedPosIdx];if(p)selSphere.position.set(p.X,p.Y,p.Z);}
  markerVisuals.scale.setScalar(frameSize/120);
}

document.getElementById('frame-size-sl').addEventListener('input',function(){
  frameSize=parseInt(this.value);document.getElementById('frame-size-v').textContent=frameSize;rebuildFrames();
});
document.getElementById('sphere-size-sl').addEventListener('input',function(){
  sphereSize=parseInt(this.value);document.getElementById('sphere-size-v').textContent=sphereSize;rebuildFrames();
});

// ═══════════════════════════════════════════════════
// ROBOT 3D MODEL
// ═══════════════════════════════════════════════════
let robotColor=0xcc4400, jointColor=0xe8a020, tcpColor=0xffee00;
let showRobot3D=true, showTCPTrace=true;
const tcpTracePoints=[];

function hexToInt(hex){return parseInt(hex.slice(1),16);}

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

// ── STL axis model state ──────────────────────────────────────
const axisSTLMeshes  = [null,null,null,null,null,null];
const axisSTLBase64  = [null,null,null,null,null,null];
const axisPivots     = [];   // THREE.Group per axis (hierarchical chain)
let pedestalMesh     = null;
let showSkeleton     = true;
let showPosFrames    = true;   // Zielkoordinatensysteme (posGrp)
let showBaseFrame    = true;   // BASE Koordinatensystem
let showAxisLabels   = true;   // Achslabel der BASE KS
let showTCPMarker    = true;   // TCP Frame (markerVisuals)
let showSTLRobot     = true;
let showToolMesh     = true;
let showPedestalMesh = true;
let stlRefAngles     = [0,-90,90,0,0,0];  // pose at which STL files are modelled

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
  // Reattach any loaded STL meshes with corrected local offsets
  attachSTLMeshesToPivots();
}

// STL axis meshes are in scene directly (like pedestal)
// Their world transform is updated in buildRobotModel via updateAxisSTLTransforms
function attachSTLMeshesToPivots() {
  // no-op: meshes are in scene, not in pivot hierarchy
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
      mesh.visible = showSTLRobot && showRobot3D;
      if (!mesh.visible) return;
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
  toolMesh.visible = showRobot3D && showToolMesh;
  if (!toolMesh.visible) return;
  const fkC  = fkAll(angles);
  const Rt   = mMul(fkC.rots[6], [[-1,0,0],[0,-1,0],[0,0,1]]); // × Rz(180°)
  toolMesh.quaternion.copy(_mat3ToQuat(Rt));
  toolMesh.position.set(fkC.pts[6][0], fkC.pts[6][1], fkC.pts[6][2]);
}

// Skelett-Visualisierung aufbauen
function _buildSkeleton(angles) {
  if (!showSkeleton || !showRobot3D) return;
  const fk   = fkAll(angles);
  const pts  = fk.pts;
  const linkR = [28,20,16,12,8,6];
  const jntR  = [40,38,30,24,20,16];
  for (let i = 0; i < 7; i++) {
    if (!pts[i] || !pts[i+1]) continue;
    const cyl = buildCylinder(pts[i], pts[i+1], linkR[i] || 5, robotColor);
    if (cyl) robotGrp.add(cyl);
  }
  for (let i = 1; i <= 6; i++) {
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
  robotGrp.children.filter(ch => !axisPivots.includes(ch)).forEach(ch => robotGrp.remove(ch));
  updatePivotRotations(angles);
  _applySTLMeshPoses(angles);
  _applyToolMeshPose(angles);
  if (pedestalMesh) pedestalMesh.visible = showPedestalMesh;
  _buildSkeleton(angles);
  if (tcpFrameGroups.length) syncTcpFrameGroups();
}

// ── STL loading for axes ──────────────────────────────────────
function buildAxisSTLUI() {
  const el = document.getElementById('axis-stl-ui');
  if (!el) return;
  el.innerHTML = JOINTS_DEF.map((j,i) =>
    `<div class="axis-stl-row">
      <span class="axis-stl-lbl">A${i+1}</span>
      <span class="axis-stl-name" id="asl-name${i}">—</span>
      <button class="stl-vis-btn" id="asl-vis${i}" onclick="cycleSTLMode('axis',${i})" title="Klicken zum Wechseln">${svgIconSolid}</button>
      <button class="axis-stl-btn" onclick="pickAxisSTL(${i})">+ STL</button>
      <button class="axis-stl-btn" id="asl-del${i}" style="display:none;color:var(--err)" onclick="removeAxisSTL(${i})">✕</button>
    </div>`
  ).join('') +
  `<div style="display:flex;align-items:center;gap:5px;margin-top:6px;font-size:.75em;border-top:1px solid var(--bdr);padding-top:5px">
    <span style="color:var(--txt3);flex:1">Referenz-Pose A1..A6:</span>
    <input id="stl-ref-angles" type="text" value="0,-90,90,0,0,0"
      style="background:var(--bg1);color:#9ecfea;border:1px solid var(--bdr);border-radius:3px;padding:1px 4px;font-size:.9em;width:130px;font-family:inherit"
      onchange="applySTLRefAngles(this.value)" title="Winkel in Grad (kommagetrennt) bei denen die STL-Dateien erstellt wurden">
  </div>`;
}

function applySTLRefAngles(val) {
  const parts = val.split(',').map(v=>parseFloat(v.trim()));
  if (parts.length===6 && parts.every(v=>!isNaN(v))) {
    stlRefAngles = parts;
    updateSTLRefFK();
    buildRobotModel(jointAngles);
  }
}

function pickAxisSTL(idx) {
  const inp = document.getElementById('axis-stl-file');
  inp.onchange = e => {
    const file = e.target.files[0]; if (!file) return;
    loadAxisSTL(idx, file); inp.value='';
  };
  inp.click();
}

function loadAxisSTL(idx, file) {
  const reader = new FileReader();
  reader.onload = e => {
    const buf = e.target.result;
    parseGeometry(buf, file.name).then(geo => {
      if (axisSTLMeshes[idx]) { scene.remove(axisSTLMeshes[idx]); axisSTLMeshes[idx].geometry.dispose(); }
      axisSTLMeshes[idx] = new THREE.Mesh(geo, new THREE.MeshPhongMaterial({color:0xe8a020,shininess:80}));
      scene.add(axisSTLMeshes[idx]);
      axisSTLBase64[idx] = btoa(String.fromCharCode.apply(null, new Uint8Array(buf)));
      window['_axisSTLBuffer'+idx] = buf;
      if (!window._axisSTLBuffers) window._axisSTLBuffers = {};
      window._axisSTLBuffers[idx] = buf;
      document.getElementById('asl-name'+idx).textContent = file.name.replace(/\.[^.]+$/i,'');
      document.getElementById('asl-del'+idx).style.display = '';
      buildRobotModel(jointAngles);
    }).catch(err => { alert('Fehler: '+err.message); });
  };
  reader.readAsArrayBuffer(file);
}

function removeAxisSTL(idx) {
  if (axisSTLMeshes[idx]) {
    scene.remove(axisSTLMeshes[idx]);
    axisSTLMeshes[idx].geometry.dispose();
    axisSTLMeshes[idx] = null;
  }
  document.getElementById('asl-name'+idx).textContent = '—';
  document.getElementById('asl-del'+idx).style.display = 'none';
  buildRobotModel(jointAngles);
}

function loadAxisSTLFromBase64(idx, b64, filename, rawBuffer, color) {
  const buffer = rawBuffer || (() => {
    const bin = atob(b64);
    const buf = new ArrayBuffer(bin.length);
    const view = new Uint8Array(buf);
    for (let i = 0; i < bin.length; i++) view[i] = bin.charCodeAt(i);
    return buf;
  })();
  try {
    const geo  = stlLoader.parse(buffer);
    geo.computeVertexNormals();
    if (axisSTLMeshes[idx]) { scene.remove(axisSTLMeshes[idx]); axisSTLMeshes[idx].geometry.dispose(); }
    axisSTLMeshes[idx] = new THREE.Mesh(geo, new THREE.MeshPhongMaterial({color:color||0xe8a020,shininess:80}));
    scene.add(axisSTLMeshes[idx]);
    if (b64) axisSTLBase64[idx] = b64;
    const nameEl = document.getElementById('asl-name'+idx);
    if (nameEl) nameEl.textContent = filename.replace(/\.[^.]+$/i,'');
    const delEl = document.getElementById('asl-del'+idx);
    if (delEl) delEl.style.display = '';
    buildRobotModel(jointAngles);
  } catch(e) { console.error('STL load error A'+(idx+1), e); }
}

function loadPedestalSTL() {
  const inp = document.getElementById('pedestal-stl-file');
  inp.onchange = e => {
    const file = e.target.files[0]; if (!file) return;
    const reader = new FileReader();
    reader.onload = ev => {
      const geo = stlLoader.parse(ev.target.result);
      geo.computeVertexNormals();
      if (pedestalMesh) { scene.remove(pedestalMesh); pedestalMesh.geometry.dispose(); }
      pedestalMesh = new THREE.Mesh(geo, new THREE.MeshPhongMaterial({color:0x334455,shininess:40}));
      scene.add(pedestalMesh);
      document.getElementById('pedestal-name').textContent = file.name.replace(/\.stl$/i,'');
    };
    reader.readAsArrayBuffer(file);
    inp.value = '';
  };
  inp.click();
}

function toggleSkeleton() {
  showSkeleton = !showSkeleton;
  document.getElementById('btn-show-skeleton')?.classList.toggle('on', showSkeleton);
  var cb = document.getElementById('cfg-show-skeleton'); if (cb) cb.checked = showSkeleton;
  buildRobotModel(jointAngles);
}

function toggleSTLRobot() {
  showSTLRobot = !showSTLRobot;
  document.getElementById('btn-show-stl-robot').classList.toggle('on', showSTLRobot);
  buildRobotModel(jointAngles);
}



















// TCP Trace
function addTCPTracePoint(tcp) {
  if (!showTCPTrace) return;
  tcpTracePoints.push(new THREE.Vector3(...tcp));
  if (tcpTracePoints.length > 2000) tcpTracePoints.shift();
  rebuildTCPTrace();
}

function rebuildTCPTrace() {
  tcpTraceGrp.clear();
  if (tcpTracePoints.length < 2) return;
  const geo = new THREE.BufferGeometry().setFromPoints(tcpTracePoints);
  var _traceColEl = document.getElementById('cfg-trace-col') || document.getElementById('cfg-path-col');
  const col = hexToInt(_traceColEl.value);
  const mat = new THREE.LineBasicMaterial({color: col, opacity:.5, transparent:true, linewidth: traceLineWidth});
  tcpTraceGrp.add(new THREE.Line(geo, mat));
}

// Color update


document.getElementById('cfg-tcp-col').addEventListener('input', function(){
  tcpColor=hexToInt(this.value); buildRobotModel(jointAngles);
});

document.getElementById('btn-robot3d').addEventListener('click', function(){
  showRobot3D=!showRobot3D;this.classList.toggle('on',showRobot3D);
  if(!showRobot3D){ buildRobotModel(jointAngles); } else buildRobotModel(jointAngles);
});
function toggleProjection(){
  var isPersp = activeCam === perspCam;
  var asp = canvas.clientWidth / canvas.clientHeight;
  if(isPersp){
    // Perspektive → Ortho: aktuelle Kameraposition/Richtung übernehmen
    var fovRad = perspCam.fov * Math.PI / 360;
    orthoHalfSize = Math.tan(fovRad) * orbitState.radius;
    orthoCam.position.copy(perspCam.position);
    orthoCam.quaternion.copy(perspCam.quaternion);
    orthoCam.up.copy(perspCam.up);
    orthoCam.left  = -orthoHalfSize * asp;
    orthoCam.right =  orthoHalfSize * asp;
    orthoCam.top    =  orthoHalfSize;
    orthoCam.bottom = -orthoHalfSize;
    orthoCam.zoom = 1;
    orthoCam.updateProjectionMatrix();
    activeCam = orthoCam;
    var btn=document.getElementById('btn-persp');
    if(btn){btn.setAttribute('data-i18n','ortho');btn.textContent=t('ortho');btn.classList.remove('on');}
  } else {
    // Ortho → Perspektive: Position zurück + orbitState ableiten
    perspCam.position.copy(orthoCam.position);
    perspCam.quaternion.copy(orthoCam.quaternion);
    perspCam.up.copy(orthoCam.up);
    var dx=perspCam.position.x-orbitTarget.x,dy=perspCam.position.y-orbitTarget.y,dz=perspCam.position.z-orbitTarget.z;
    orbitState.radius=Math.sqrt(dx*dx+dy*dy+dz*dz)||orbitState.radius;
    orbitState.phi=Math.acos(Math.max(-1,Math.min(1,dz/orbitState.radius)));
    orbitState.theta=Math.atan2(dy,dx);
    perspCam.aspect=asp;
    perspCam.updateProjectionMatrix();
    activeCam = perspCam;
    var btn=document.getElementById('btn-persp');
    if(btn){btn.setAttribute('data-i18n','persp');btn.textContent=t('persp');btn.classList.add('on');}
  }
  resize();
}

// ── Steuerungspanel ────────────────────────────────────────
// ══════════════════════════════════════════════
// SAVE / LOAD — Kinematik, TCP, Programm
// ══════════════════════════════════════════════

function downloadFile(content, filename) {
  const a = document.createElement('a');
  a.href = 'data:text/plain;charset=utf-8,' + encodeURIComponent(content);
  a.download = filename;
  document.body.appendChild(a);
  a.click();
  document.body.removeChild(a);
}

// ── Kinematik ──────────────────────────────────
// STL offset/rotation per axis (set via UI, default 0)
var axisSTLOffsets = [
  {px:0,py:0,pz:0,rx:0,ry:0,rz:0},
  {px:0,py:0,pz:0,rx:0,ry:0,rz:0},
  {px:0,py:0,pz:0,rx:0,ry:0,rz:0},
  {px:0,py:0,pz:0,rx:0,ry:0,rz:0},
  {px:0,py:0,pz:0,rx:0,ry:0,rz:0},
  {px:0,py:0,pz:0,rx:0,ry:0,rz:0}
];
// Scene STL offsets (pedestal, tool)
var sceneSTLOffsets = {
  pedestal: {px:0,py:0,pz:0,rx:0,ry:0,rz:0, name:'podest'},
  tool:     {px:0,py:0,pz:0,rx:0,ry:0,rz:0, name:'tool1_tcp'}
};

function getKinematicData() {
  var data = {
    name: (document.getElementById('kin-name')&&document.getElementById('kin-name').value) || 'Kinematik',
    joints: JOINTS_DEF.map(function(j) {
      return {name:j.name, axis:j.axis, offset:{x:j.off[0],y:j.off[1],z:j.off[2]}, min:j.min, max:j.max};
    }),
    stlRefAngles: stlRefAngles,
    stlFiles: {}
  };
  for (let i = 0; i < 6; i++) {
    var name = (document.getElementById('asl-name'+i)&&document.getElementById('asl-name'+i).textContent) || '';
    var off = axisSTLOffsets[i];
    data.stlFiles['A'+(i+1)] = {
      name: name || ('a'+(i+1)),
      posx: off.px, posy: off.py, posz: off.pz,
      posrx: off.rx, posry: off.ry, posrz: off.rz,
      color: (axisSTLMeshes[i] && axisSTLMeshes[i].material)
        ? '#' + axisSTLMeshes[i].material.color.getHexString() : '#e8a020'
    };
  }
  // Scene models
  data.sceneModels = {
    pedestal: Object.assign({}, sceneSTLOffsets.pedestal),
    tool:     Object.assign({}, sceneSTLOffsets.tool)
  };
  return data;
}

function getKinematicJSON() {
  return JSON.stringify(getKinematicData(), null, 2);
}

function saveKinematic() {
  var kinName = ((document.getElementById('kin-name')&&document.getElementById('kin-name').value) || 'kinematik').replace(/\s+/g,'_');
  var data = getKinematicData();

  if (typeof JSZip === 'undefined') {
    // Fallback: JSON only
    downloadFile(JSON.stringify(data, null, 2), kinName + '.json');
    return;
  }

  var zip = new JSZip();
  // JSON config (no STL data embedded)
  zip.file(kinName + '.json', JSON.stringify(data, null, 2));

  // Axis STLs — direkt aus ArrayBuffer (kein Base64-Umweg)
  var axisAdded = 0;
  for (let i = 0; i < 6; i++) {
    var buf = window['_axisSTLBuffer'+i];
    if (!buf) buf = window._axisSTLBuffers && window._axisSTLBuffers[i];
    if (buf) {
      var stlName = (data.stlFiles['A'+(i+1)] && data.stlFiles['A'+(i+1)].name) || ('a'+(i+1));
      if (stlName.toLowerCase().indexOf('.stl') === -1) stlName += '.stl';
      zip.file(stlName, new Uint8Array(buf));
      axisAdded++;
    } else if (axisSTLBase64[i]) {
      // Fallback: base64 → binary
      var stlName2 = (data.stlFiles['A'+(i+1)] && data.stlFiles['A'+(i+1)].name) || ('a'+(i+1));
      if (stlName2.toLowerCase().indexOf('.stl') === -1) stlName2 += '.stl';
      try {
        var bin = atob(axisSTLBase64[i]);
        var arr = new Uint8Array(bin.length);
        for (let k = 0; k < bin.length; k++) arr[k] = bin.charCodeAt(k);
        zip.file(stlName2, arr);
        axisAdded++;
      } catch(ex) { console.warn('ZIP: STL base64 error A'+(i+1), ex); }
    }
  }


  // Pedestal STL
  if (window._pedestalSTLBuffer) {
    var pName = (sceneSTLOffsets.pedestal.name || 'podest') + '.stl';
    zip.file(pName, new Uint8Array(window._pedestalSTLBuffer));
  }

  // Tool STL
  if (window._toolSTLBuffer) {
    var tName = (sceneSTLOffsets.tool.name || 'tool1_tcp') + '.stl';
    zip.file(tName, new Uint8Array(window._toolSTLBuffer));
  }

  // Szenen-STLs (manuell geladen)
  if (typeof stlObjects !== 'undefined') {
    var sceneAdded = 0;
    for (let si = 0; si < stlObjects.length; si++) {
      var so = stlObjects[si];
      if (!so || !so.buf) continue;
      var sName = so.name || ('scene_'+si+'.stl');
      if (sName.toLowerCase().indexOf('.stl') === -1) sName += '.stl';
      zip.file('scene/' + sName, new Uint8Array(so.buf));
      sceneAdded++;
    }

  }

  zip.generateAsync({type:'blob', compression:'DEFLATE', compressionOptions:{level:6}})
    .then(function(blob) {
      var url = URL.createObjectURL(blob);
      var a = document.createElement('a');
      a.href = url; a.download = kinName + '.zip';
      document.body.appendChild(a); a.click();
      document.body.removeChild(a);
      setTimeout(function(){ URL.revokeObjectURL(url); }, 2000);
    });
}

function loadKinematic() {
  var inp = document.getElementById('kin-file-in');
  inp.onchange = function(e) {
    var file = e.target.files[0]; if (!file) return;
    if (file.name.endsWith('.zip')) {
      loadKinematicZIP(file);
    } else {
      var reader = new FileReader();
      reader.onload = function(ev) {
        try {
          var data = file.name.endsWith('.xml')
            ? parseKinematicXML(ev.target.result)
            : JSON.parse(ev.target.result);
          applyKinematicData(data, {});
        } catch(err) { alert('Fehler beim Laden: ' + err.message); }
        inp.value = '';
      };
      reader.readAsText(file);
    }
    inp.value = '';
  };
  inp.click();
}

function loadKinematicZIP(file) {
  if (typeof JSZip === 'undefined') {
    alert('JSZip nicht geladen');
    return;
  }
  JSZip.loadAsync(file).then(function(zip) {
    // Find JSON config
    var jsonFile = null;
    zip.forEach(function(path, entry) {
      if (!entry.dir && path.endsWith('.json')) jsonFile = entry;
    });
    if (!jsonFile) { alert('Keine JSON-Konfiguration in ZIP gefunden'); return; }

    jsonFile.async('string').then(function(jsonStr) {
      var data;
      try { data = JSON.parse(jsonStr); } catch(e) { alert('JSON Fehler: ' + e.message); return; }

      // Collect STL buffers from ZIP
      var stlBuffers = {};
      var sceneBuffers = {}; // scene/name → buffer (mit Pfad)
      var stlPromises = [];
      zip.forEach(function(path, entry) {
        if (!entry.dir && path.toLowerCase().endsWith('.stl')) {
          var fname = path.replace(/.*\//, '').replace(/\.stl$/i, '').toLowerCase();
          var fullPath = path;
          stlPromises.push(
            entry.async('arraybuffer').then(function(buf) {
              stlBuffers[fname] = buf;
              if (fullPath.toLowerCase().indexOf('scene/') >= 0) {
                sceneBuffers[path.replace(/.*scene\//i,'')] = buf;
                sceneBuffers[fname] = buf;
              }
            })
          );
        }
      });

      Promise.all(stlPromises).then(function() {
        applyKinematicData(data, stlBuffers, sceneBuffers);
      });
    });
  }).catch(function(e) { alert('ZIP Fehler: ' + e.message); });
}

function parseKinematicXML(xml) {
  const parser = new DOMParser();
  const doc = parser.parseFromString(xml, 'text/xml');
  const name = (function(){var _e=doc.querySelector('kinematik');return _e&&_e.getAttribute('name');})() || 'Kinematik';
  const joints = [...doc.querySelectorAll('joint')].map(j => ({
    name: j.getAttribute('name'),
    axis: j.getAttribute('axis'),
    offset: {
      x: parseFloat((function(){var _e=j.querySelector('offset');return _e&&_e.getAttribute('x');})() || 0),
      y: parseFloat((function(){var _e=j.querySelector('offset');return _e&&_e.getAttribute('y');})() || 0),
      z: parseFloat((function(){var _e=j.querySelector('offset');return _e&&_e.getAttribute('z');})() || 0)
    },
    min: parseFloat(j.getAttribute('min') || -180),
    max: parseFloat(j.getAttribute('max') || 180)
  }));
  return {name, joints};
}

function applyKinematicData(data, stlBuffers, sceneBuffers) {
  stlBuffers = stlBuffers || {};
  sceneBuffers = sceneBuffers || {};
  if (data.name) {
    const el = document.getElementById('kin-name');
    if (el) el.value = data.name;
    // Update section title
    const secT = document.querySelector('.sec-t span:last-of-type');
  }
  if (data.joints) {
    // Extend or replace JOINTS_DEF
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
  buildKinConfig();
  buildSteuerAxes();
  // Restore STL files from embedded data
  if (data.stlRefAngles) {
    stlRefAngles = data.stlRefAngles;
    const inp = document.getElementById('stl-ref-angles');
    if (inp) inp.value = stlRefAngles.join(',');
    // Kanonenstellung auf aktuelle Gelenkwinkel anwenden
    jointAngles = stlRefAngles.slice();
  }
  if (data.stlFiles) {
    Object.entries(data.stlFiles).forEach(function(entry) {
      var key = entry[0]; var val = entry[1];
      if (!val) return;
      var axIdx = parseInt(key.replace('A','')) - 1;
      if (axIdx < 0 || axIdx > 5) return;
      // Backward compat: single object → array
      var parts = Array.isArray(val) ? val : [val];
      // Store offset/rotation from first part (backward compat)
      var first = parts[0];
      if (axisSTLOffsets[axIdx] && first) {
        axisSTLOffsets[axIdx] = {
          px: first.posx||0, py: first.posy||0, pz: first.posz||0,
          rx: first.posrx||0, ry: first.posry||0, rz: first.posrz||0
        };
      }
      // Load each part — remove old meshes first
      if (axisSTLMeshes[axIdx]) {
        scene.remove(axisSTLMeshes[axIdx]);
        axisSTLMeshes[axIdx].geometry.dispose();
        axisSTLMeshes[axIdx] = null;
      }
      // Remove extra parts from previous load
      if (!window._axisExtraMeshes) window._axisExtraMeshes = {};
      (window._axisExtraMeshes[axIdx]||[]).forEach(function(m){ scene.remove(m); m.geometry.dispose(); });
      window._axisExtraMeshes[axIdx] = [];
      var keyLower = key.toLowerCase();
      parts.forEach(function(p, pi) {
        var fname = (p.name||'').replace(/\.(stl|osd)$/i,'').toLowerCase();
        var buf = stlBuffers[fname] || (pi===0 ? stlBuffers[keyLower]||stlBuffers['axis'+keyLower] : null);
        if (!buf && p.data) { /* base64 fallback */ }
        if (!buf) return;
        if (!window._zipSTLCache) window._zipSTLCache = {};
        if (pi===0) window._zipSTLCache[keyLower] = buf;
        var dispName = (fname&&fname!=='—'&&fname!=='-') ? fname+'.stl' : keyLower+'.stl';
        parseGeometry(buf, dispName).then(function(geo) {
          var mat = new THREE.MeshPhongMaterial({color: p.color||0xe8a020, shininess:80});
          var mesh = new THREE.Mesh(geo, mat);
          scene.add(mesh);
          if (pi===0) {
            axisSTLMeshes[axIdx] = mesh;
            window['_axisSTLBuffer'+axIdx] = buf;
            if (!window._axisSTLBuffers) window._axisSTLBuffers = {};
            window._axisSTLBuffers[axIdx] = buf;
            var nameEl = document.getElementById('asl-name'+axIdx);
            if (nameEl) nameEl.textContent = dispName.replace(/\.stl$/i,'');
            var delEl = document.getElementById('asl-del'+axIdx);
            if (delEl) delEl.style.display='';
          } else {
            window._axisExtraMeshes[axIdx].push(mesh);
          }
          buildRobotModel(jointAngles);
        });
      });
    });
  }
  // Scene models from ZIP
  if (data.sceneModels) {
    var sm = data.sceneModels;
    if (sm.pedestal) {
      sceneSTLOffsets.pedestal = Object.assign(sceneSTLOffsets.pedestal, sm.pedestal);
      var pname = (sm.pedestal.name || 'podest').toLowerCase();
      // Fallback: wenn kein exakter Treffer, suche nach Schlüssel mit 'base' oder 'podest'
      var pbuf = stlBuffers[pname] ||
        (Object.entries(stlBuffers).find(function(e){ return /base|podest/i.test(e[0]); })||[])[1];
      if (pbuf) {
        var buf = pbuf;
        window._pedestalSTLBuffer = buf;
        if (!window._zipSTLCache) window._zipSTLCache = {};
        window._zipSTLCache['podest'] = buf; window._zipSTLCache['pedestal'] = buf;
        var geo = stlLoader.parse(buf); geo.computeVertexNormals();
        if (pedestalMesh) { scene.remove(pedestalMesh); pedestalMesh.geometry.dispose(); }
        pedestalMesh = new THREE.Mesh(geo, new THREE.MeshPhongMaterial({color:0x334455,shininess:40}));
        scene.add(pedestalMesh);
        var el = document.getElementById('pedestal-name');
        if (el) el.textContent = pname;
      }
    }
    if (sm.tool) {
      sceneSTLOffsets.tool = Object.assign(sceneSTLOffsets.tool, sm.tool);
      var tname = (sm.tool.name || 'tool1_tcp').toLowerCase();
      if (stlBuffers[tname]) {
        var tbuf = stlBuffers[tname];
        window._toolSTLBuffer = tbuf;
        var tgeo = stlLoader.parse(tbuf); tgeo.computeVertexNormals();
        if (toolMesh) { scene.remove(toolMesh); toolMesh.geometry.dispose(); toolMesh.material.dispose(); }
        var mat = new THREE.MeshPhongMaterial({color:0xdd9944,transparent:false,opacity:1.0,side:THREE.DoubleSide,specular:0x666666});
        toolMesh = new THREE.Mesh(tgeo, mat);
        scene.add(toolMesh);
        document.getElementById('tool-filename').textContent = tname;
        document.getElementById('tool-controls').style.display = 'block';
      }
    }
  }
  // TCP-Liste aus JSON
  if (data.tcpList && data.tcpList.length) {
    tcpList = data.tcpList;
    tcpNavIdx = 0;
    tcpSetInputs(tcpList[0]);
    var tni = document.getElementById('tcp-name'); if(tni) tni.value = tcpList[0].name||'TCP 1';
    renderTcpNav();
  }
  // BASE-Liste aus JSON
  if (data.baseList && data.baseList.length) {
    baseList = data.baseList;
    baseNavIdx = 0;
    baseSetInputs(baseList[0]);
    var bni2 = document.getElementById('base-name'); if(bni2) bni2.value = baseList[0].name||'BASE 1';
    renderBaseNav();
  }
  // TCP aus JSON in Liste
  if (data.tcp) {
    var t = data.tcp;
    tcpList = [{ x:t.x||0, y:t.y||0, z:t.z||0, a:t.a||0, b:t.b||0, c:t.c||0, name:'TCP 1' }];
    var tni2 = document.getElementById('tcp-name'); if(tni2) tni2.value = 'TCP 1';
    tcpNavIdx = 0;
    tcpSetInputs(tcpList[0]);
    renderTcpNav();
  } else if (!tcpList.length) {
    tcpList = [{ x:0,y:0,z:0,a:0,b:0,c:0, name:'TCP 1' }];
    tcpNavIdx = 0; renderTcpNav();
  }
  // BASE immer mindestens 1 Eintrag
  if (!baseList.length) {
    baseList = [{ x:0,y:0,z:0,a:0,b:0,c:0, name:'BASE 1' }];
    baseNavIdx = 0; renderBaseNav();
  }
  buildRobotModel(jointAngles);

  // Code-Editor wiederherstellen + parsen
  if (data.code) {
    var codeEl3 = document.getElementById('code-input');
    if (codeEl3) {
      codeEl3.value = data.code;
      codeEl3.style.display = '';
      // kuka-form Formularansicht schliessen
      var fw = document.getElementById('fv-form-wrap'); if(fw) fw.style.display='none';
      var fv = document.getElementById('krl-form-view'); if(fv) fv.style.display='none';
      // Code parsen und Szene aufbauen
      if (typeof parseAndLoad === 'function') parseAndLoad();
    }
  }

  // Szenenmodelle aus stlScene + scene/ STLs
  if (Array.isArray(data.stlScene) && data.stlScene.length) {
    // bestehende leeren
    stlObjects.forEach(function(o){ if(o&&o.mesh) stlGrp.remove(o.mesh); });
    stlObjects.length = 0;
    var sceneLoaded = 0;
    data.stlScene.forEach(function(entry) {
      var fname = (entry.name||'').replace(/^scene\//i,'');
      var key = fname.replace(/\.stl$/i,'').toLowerCase();
      var buf = sceneBuffers[fname] || sceneBuffers[key] || stlBuffers[key];
      var offset = entry.offset || {};
      if (buf) {
        parseGeometry(buf, fname).then(function(geo) {
          var mat = new THREE.MeshPhongMaterial({color:0x4499cc,shininess:60,side:THREE.DoubleSide});
          var mesh = new THREE.Mesh(geo, mat);
          stlGrp.add(mesh);
          var idx = stlObjects.length;
          stlObjects.push({mesh:mesh, name:fname, buf:buf, offset:Object.assign({x:0,y:0,z:0,a:0,b:0,c:0},offset), displayName:entry.displayName||''});
          updateSTL(idx);
          sceneLoaded++;
          stlNavIdx = 0; renderStlNav();
        });
      } else {
        // Eintrag ohne STL (nur Parameter)
        stlObjects.push({mesh:null, name:fname, buf:null, offset:Object.assign({x:0,y:0,z:0,a:0,b:0,c:0},offset), displayName:entry.displayName||''});
      }
    });
    stlNavIdx = 0; renderStlNav();
  }

  if (Array.isArray(data.cables)) window.cableSystem?.loadCables(data.cables);
  setStatus('paused', 'Kinematik geladen');
}

function toggleLeftPanel() {
  const ep = document.querySelector('.ep');
  const btn = document.getElementById('ep-toggle-btn');
  if (!ep) return;
  const closed = ep._panelClosed;
  if (closed) {
    ep.style.width = ep._savedW || '300px';
    ep.style.minWidth = '';
    ep.style.overflow = '';
    ep._panelClosed = false;
    if (btn) btn.textContent = '◀';
  } else {
    ep._savedW = ep.offsetWidth + 'px';
    ep.style.width = '0';
    ep.style.minWidth = '0';
    ep.style.overflow = 'hidden';
    ep._panelClosed = true;
    if (btn) btn.textContent = '▶';
  }
  window.dispatchEvent(new Event('resize'));
}

function toggleRightPanel() {
  const ip = document.getElementById('info-panel');
  const btn = document.getElementById('ip-toggle-btn');
  if (!ip) return;
  const closed = ip._panelClosed;
  if (closed) {
    ip.style.width = ip._savedW || '270px';
    ip.style.minWidth = '';
    ip.style.overflow = '';
    ip._panelClosed = false;
    if (btn) btn.textContent = '▶';
  } else {
    ip._savedW = ip.offsetWidth + 'px';
    ip.style.width = '0';
    ip.style.minWidth = '0';
    ip.style.overflow = 'hidden';
    ip._panelClosed = true;
    if (btn) btn.textContent = '◀';
  }
  window.dispatchEvent(new Event('resize'));
}


// ── TCP Navigation ────────────────────────────────────────────────
var tcpList = [];   // [{x,y,z,a,b,c,name}]
var tcpNavIdx = -1; // -1 = kein gespeicherter, direkte Eingabe

function tcpGetInputs() {
  return {
    x: parseFloat(document.getElementById('tcp-x')?.value)||0,
    y: parseFloat(document.getElementById('tcp-y')?.value)||0,
    z: parseFloat(document.getElementById('tcp-z')?.value)||0,
    a: parseFloat(document.getElementById('tcp-a')?.value)||0,
    b: parseFloat(document.getElementById('tcp-b')?.value)||0,
    c: parseFloat(document.getElementById('tcp-c')?.value)||0,
  };
}

function tcpSetInputs(t) {
  ['x','y','z','a','b','c'].forEach(k => {
    var el = document.getElementById('tcp-' + k);
    if (el) el.value = t[k] !== undefined ? t[k] : 0;
  });
}

function tcpNavTo(idx) {
  if (!tcpList.length) return;
  tcpNavIdx = Math.max(0, Math.min(idx, tcpList.length - 1));
  tcpSetInputs(tcpList[tcpNavIdx]);
  var entry = tcpList[tcpNavIdx];
  if (entry && !entry.name) entry.name = 'TCP ' + (tcpNavIdx+1);
  var n = document.getElementById('tcp-name');
  if (n) n.value = entry ? entry.name : '';
  renderTcpNav();
}

function tcpAddCurrent() {
  var name = 'TCP ' + (tcpList.length + 1);
  var t = { x:0, y:0, z:0, a:0, b:90, c:0, name: name };
  tcpList.push(t);
  tcpNavIdx = tcpList.length - 1;
  tcpSetInputs(t);
  var ni = document.getElementById('tcp-name');
  if (ni) ni.value = name;
  renderTcpNav();
}

function tcpDelCurrent() {
  if (tcpNavIdx < 0 || !tcpList.length) return;
  tcpList.splice(tcpNavIdx, 1);
  tcpNavIdx = Math.min(tcpNavIdx, tcpList.length - 1);
  if (tcpNavIdx >= 0) tcpSetInputs(tcpList[tcpNavIdx]);
  renderTcpNav();
}

function tcpNameChanged(val) {
  if (tcpList[tcpNavIdx]) {
    tcpList[tcpNavIdx].name = val;
    syncTcpFrameGroups();
  }
}

function renderTcpNav() {
  syncTcpFrameGroups();
  var n = tcpList.length, i = tcpNavIdx;
  var lbl = document.getElementById('tcp-nav-label');
  var name = (n && tcpList[i]) ? tcpList[i].name || '' : '';
  if (lbl) lbl.textContent = n ? (i+1)+'/'+n+(name?' · '+name:'') : '—';
  var dis = function(id, d) { var b=document.getElementById(id); if(b) b.disabled=d; };
  dis('tcp-nav-first', i<=0||!n);
  dis('tcp-nav-prev',  i<=0||!n);
  dis('tcp-nav-next',  i>=n-1);
  dis('tcp-nav-last',  i>=n-1);
  dis('tcp-nav-del',   !n);
}

// TCP nav init happens via DOMContentLoaded
document.addEventListener('DOMContentLoaded', function() {
  // Jeder Roboter hat mindestens 1 TCP und 1 BASE
  if (!tcpList.length) {
    tcpList.push({ x: parseFloat(document.getElementById('tcp-x')?.value)||364.5,
                   y: parseFloat(document.getElementById('tcp-y')?.value)||0,
                   z: parseFloat(document.getElementById('tcp-z')?.value)||46.5,
                   a: parseFloat(document.getElementById('tcp-a')?.value)||0,
                   b: parseFloat(document.getElementById('tcp-b')?.value)||90,
                   c: parseFloat(document.getElementById('tcp-c')?.value)||0,
                   name: 'TCP 1' });
    tcpNavIdx = 0;
    var tni = document.getElementById('tcp-name'); if(tni) tni.value = tcpList[0].name || 'TCP 1';
  }
  renderTcpNav();
});


// ── BASE Navigation ───────────────────────────────────────────────
var baseList   = [];
var baseNavIdx = -1;

function baseGetInputs() {
  return {
    x: parseFloat(document.getElementById('base-x')?.value)||0,
    y: parseFloat(document.getElementById('base-y')?.value)||0,
    z: parseFloat(document.getElementById('base-z')?.value)||0,
    a: parseFloat(document.getElementById('base-a')?.value)||0,
    b: parseFloat(document.getElementById('base-b')?.value)||0,
    c: parseFloat(document.getElementById('base-c')?.value)||0,
  };
}

function baseSetInputs(t) {
  ['x','y','z','a','b','c'].forEach(function(k) {
    var el = document.getElementById('base-' + k);
    if (el) el.value = t[k] !== undefined ? t[k] : 0;
  });
}

function baseNavTo(idx) {
  if (!baseList.length) return;
  baseNavIdx = Math.max(0, Math.min(idx, baseList.length - 1));
  baseSetInputs(baseList[baseNavIdx]);
  var entry = baseList[baseNavIdx];
  if (entry && !entry.name) entry.name = 'BASE ' + (baseNavIdx+1);
  var n = document.getElementById('base-name');
  if (n) n.value = entry ? entry.name : '';
  renderBaseNav();
  syncBaseFrameGroups(); // nur Frame/Anzeige; keine Trace-Verschiebung beim Navigieren
}

function baseAddCurrent() {
  var t = baseGetInputs();
  t.name = 'BASE ' + (baseList.length + 1);
  baseList.push(t);
  baseNavIdx = baseList.length - 1;
  var ni = document.getElementById('base-name');
  if (ni) ni.value = t.name;
  renderBaseNav();
}

function baseDelCurrent() {
  if (baseNavIdx < 0 || !baseList.length) return;
  baseList.splice(baseNavIdx, 1);
  baseNavIdx = Math.min(baseNavIdx, baseList.length - 1);
  if (baseNavIdx >= 0) baseSetInputs(baseList[baseNavIdx]);
  renderBaseNav();
}

function baseNameChanged(val) {
  if (baseList[baseNavIdx]) {
    baseList[baseNavIdx].name = val;
    syncBaseFrameGroups();
  }
}

function renderBaseNav() {
  syncBaseFrameGroups();
  var n = baseList.length, i = baseNavIdx;
  var lbl = document.getElementById('base-nav-label');
  var name = (n && baseList[i]) ? baseList[i].name || '' : '';
  if (lbl) lbl.textContent = n ? (i+1)+'/'+n+(name?' · '+name:'') : '—';
  var dis = function(id, d) { var b=document.getElementById(id); if(b) b.disabled=d; };
  dis('base-nav-first', i<=0||!n);
  dis('base-nav-prev',  i<=0||!n);
  dis('base-nav-next',  i>=n-1);
  dis('base-nav-last',  i>=n-1);
  dis('base-nav-del',   !n);
}

// Stellt sicher, dass baseList einen Eintrag fuer jede im Programm referenzierte BASE_DATA[n] hat.
// Fehlende Bases werden als Identitaet (Offset 0) angelegt, damit sie navigier-/einstellbar sind.
function ensureBasesForProgram() {
  if (typeof parsedData === 'undefined' || !parsedData || !parsedData.positions) return;
  var maxN = 0;
  parsedData.positions.forEach(function(p){ if (p.baseN && p.baseN > maxN) maxN = p.baseN; });
  if (parsedData.steps) parsedData.steps.forEach(function(s){ if (s && s.type==='base' && s.n && s.n > maxN) maxN = s.n; });
  var changed = false;
  while (baseList.length < maxN) {
    baseList.push({ x:0, y:0, z:0, a:0, b:0, c:0, name: 'BASE ' + (baseList.length+1) });
    changed = true;
  }
  if (baseNavIdx < 0 && baseList.length) baseNavIdx = 0;
  if (changed) renderBaseNav();
}

// ── TOOL/TCP-Umschaltung ──────────────────────────────────────
// $TOOL=TOOL_DATA[n] wird auf die bestehende tcpList abgebildet: TOOL_DATA[n] -> tcpList[n-1].
// (STL-Umschaltung pro Tool folgt via robmodel-XML.)

// TCP-Eintrag fuer eine Tool-Nummer aus der tcpList holen (mit Fallback)
function _tcpEntryForTool(toolN) {
  if (!toolN) toolN = 1;
  if (typeof tcpList !== 'undefined' && tcpList) {
    if (tcpList[toolN-1]) return tcpList[toolN-1];
    if (tcpList.length) return tcpList[(typeof tcpNavIdx!=='undefined' && tcpNavIdx>=0)?tcpNavIdx:0] || tcpList[0];
  }
  return null;
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

function updateBaseDef() {
  var b = baseGetInputs();
  if (baseList[baseNavIdx]) {
    baseList[baseNavIdx].x = b.x; baseList[baseNavIdx].y = b.y; baseList[baseNavIdx].z = b.z;
    baseList[baseNavIdx].a = b.a; baseList[baseNavIdx].b = b.b; baseList[baseNavIdx].c = b.c;
  }
  syncBaseFrameGroups();
  applyBase(); // Trace/Endpunkte aus rel + aktuellem BASE neu berechnen
}

// Aktiver BASE (der im Panel gewaehlte). KUKA: BASE_DATA[n] -> baseList[n-1]
function _activeBaseN() { return ((typeof baseNavIdx !== 'undefined' && baseNavIdx >= 0) ? baseNavIdx : 0) + 1; }

// Welt = Code-Koordinate (rel) + aktueller BASE-Offset. Nur Transformation, kein Neuzeichnen.
function bakeBaseOffset() {
  if (typeof parsedData === 'undefined' || !parsedData || !parsedData.positions) return;
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

// Bei Uebernehmen: Offset neu rechnen und Szene/IK/Karten neu zeichnen (Kamera bleibt).
function applyBase() {
  if (typeof parsedData === 'undefined' || !parsedData || !parsedData.positions || !parsedData.positions.length) return;
  bakeBaseOffset();
  var savedT = (typeof orbitTarget !== 'undefined' && orbitTarget && orbitTarget.clone) ? orbitTarget.clone() : null;
  var savedR = (typeof orbitState !== 'undefined' && orbitState) ? orbitState.radius : null;
  var savedH = (typeof orthoHalfSize !== 'undefined') ? orthoHalfSize : null;
  if (typeof buildScene === 'function') buildScene(parsedData.positions);
  if (savedT) { orbitTarget.copy(savedT); if (savedR!=null) orbitState.radius=savedR; if (savedH!=null) orthoHalfSize=savedH; if (typeof updateCamera==='function') updateCamera(); }
  // IK NICHT hier neu rechnen (blockiert ~5s) – Pfad/Frames sind via buildScene aktualisiert;
  // die IK fuer die verschobenen Endpunkte wird beim naechsten Parsen/Start berechnet.
  if (typeof renderPositions === 'function') renderPositions(parsedData.positions);
  if (typeof selectedPosIdx !== 'undefined' && selectedPosIdx !== null) {
    var c = document.getElementById('pcard-'+selectedPosIdx); if (c) c.classList.add('selected');
    var sp = parsedData.positions[selectedPosIdx];
    if (sp && typeof selSphere !== 'undefined' && selSphere) { selSphere.position.set(sp.X,sp.Y,sp.Z); selSphere.visible=true; }
  }
}

document.addEventListener('DOMContentLoaded', function() {
  renderStlNav();
});
document.addEventListener('DOMContentLoaded', function() {
  if (!baseList.length) {
    baseList.push({ x:0, y:0, z:0, a:0, b:0, c:0, name: 'BASE 1' });
    var bni = document.getElementById('base-name'); if(bni) bni.value = 'BASE 1';
    baseNavIdx = 0;
  }
  renderBaseNav();
  syncBaseFrameGroups();
});


// ── NEU / LADEN / SPEICHERN (komplette Szene) ─────────────────────


// ── STEP / OCCT Import ────────────────────────────────────────────
var _occtModule = null;
var _occtLoading = null;

function getOCCT() {
  if (_occtModule) return Promise.resolve(_occtModule);
  if (_occtLoading) return _occtLoading;
  if (typeof occtimportjs === 'undefined') return Promise.reject(new Error('occt-import-js nicht geladen'));
  _occtLoading = occtimportjs({
    locateFile: function(path) {
      return 'https://cdn.jsdelivr.net/npm/occt-import-js@0.0.23/dist/' + path;
    }
  }).then(function(m) { _occtModule = m; return m; });
  return _occtLoading;
}

function stepToGeometry(arrayBuffer) {
  return getOCCT().then(function(occt) {
    var buf = new Uint8Array(arrayBuffer);
    occt.FS.writeFile('/input.stp', buf);
    var result = occt.ReadStepFile('/input.stp', null);
    if (!result.success || !result.meshes || !result.meshes.length)
      throw new Error('STEP lesen fehlgeschlagen');
    var positions = [], normals = [], indices = [], offset = 0;
    result.meshes.forEach(function(mesh) {
      var pos = mesh.attributes.position.array;
      var nor = mesh.attributes.normal && mesh.attributes.normal.array;
      var idx = mesh.index && mesh.index.array;
      for (var j=0;j<pos.length;j++) positions.push(pos[j]);
      if (nor) for (var j=0;j<nor.length;j++) normals.push(nor[j]);
      if (idx) for (var j=0;j<idx.length;j++) indices.push(idx[j]+offset);
      offset += pos.length/3;
    });
    var geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.Float32BufferAttribute(positions,3));
    if (normals.length) geo.setAttribute('normal', new THREE.Float32BufferAttribute(normals,3));
    if (indices.length) geo.setIndex(indices);
    geo.computeVertexNormals();
    return geo;
  });
}

// Universeller Geometry-Loader: STL oder STEP
function parseGeometry(arrayBuffer, fileName) {
  var ext = (fileName||'').split('.').pop().toLowerCase();
  if (ext==='stp'||ext==='step') {
    setStatus('paused', 'STEP wird geladen (OCCT)...');
    return stepToGeometry(arrayBuffer);
  }
  // STL (synchron → in Promise verpackt)
  return Promise.resolve(stlLoader.parse(arrayBuffer));
}

function sceneNeu() {
  if (!confirm('Alles zurücksetzen? Alle Daten gehen verloren.')) return;

  // Simulation stoppen
  pauseSim();
  setStatus('stopped', 'STOPPED');

  // Simulationsdaten zuerst leeren (generateKRL braucht leere parsedData)
  parsedData = {positions:[], steps:[], finalState:{variables:{},digitalIn:{},digitalOut:{},analogOut:{},analogIn:{}}};
  ikTable = []; trajectory = []; trajMax = 0;
  buildScene([]);
  clearTCPTrace();

  // Editor direkt leeren — kuka-form deactivieren falls aktiv, dann textarea leer setzen
  if (typeof FormatRegistry !== 'undefined' && FormatRegistry.getActiveId() !== 'kuka') {
    FormatRegistry.setActive('kuka');
  }
  // Immer direkt leeren (generateKRL schreibt sonst Fallback-Code)
  var codeEl = document.getElementById('code-input');
  if (codeEl) { codeEl.value = ''; codeEl.style.display = ''; }
  // kuka-form Formularansicht verstecken falls aktiv
  var fvWrap = document.getElementById('fv-form-wrap');
  if (fvWrap) fvWrap.style.display = 'none';
  var fvView = document.getElementById('krl-form-view');
  if (fvView) fvView.style.display = 'none';
  var gutEl = document.getElementById('gutter'); if(gutEl) gutEl.innerHTML = '';

  // Roboter: Kinematik + STLs
  for (var i=0;i<6;i++) {
    if (axisSTLMeshes[i]) { scene.remove(axisSTLMeshes[i]); axisSTLMeshes[i].geometry.dispose(); axisSTLMeshes[i] = null; }
    axisSTLBase64[i] = null;
    if (window._axisSTLBuffers) window._axisSTLBuffers[i] = null;
    // UI zurücksetzen
    var nameEl = document.getElementById('asl-name'+i); if(nameEl) nameEl.textContent = '—';
    var delEl  = document.getElementById('asl-del'+i);  if(delEl)  delEl.style.display = 'none';
  }
  axisSTLMode = ['solid','solid','solid','solid','solid','solid'];
  JOINTS_DEF.forEach(function(j){ j.off=[0,0,0]; j.min=-180; j.max=180; });
  var knEl = document.getElementById('kin-name'); if(knEl) knEl.value = 'Neue Szene';
  jointAngles = [0,-90,90,0,0,0];
  buildKinConfig();
  buildRobotModel(jointAngles);
  // Skeleton + non-pivot children aus robotGrp entfernen
  var toRemove = robotGrp.children.filter(function(ch){ return axisSTLMeshes.indexOf(ch)===-1 && axisPivots.indexOf(ch)===-1; });
  toRemove.forEach(function(ch){ robotGrp.remove(ch); });

  // Podest + Tool STL
  if (pedestalMesh) { scene.remove(pedestalMesh); pedestalMesh.geometry.dispose(); pedestalMesh=null; }
  if (toolMesh) { if(toolMesh.parent) toolMesh.parent.remove(toolMesh); toolMesh.geometry.dispose(); toolMesh.material.dispose(); toolMesh=null; }
  var tcEl=document.getElementById('tool-controls'); if(tcEl) tcEl.style.display='none';
  var tfEl=document.getElementById('tool-filename'); if(tfEl) tfEl.textContent='';
  window._pedestalSTLBuffer = null; window._toolSTLBuffer = null;

  // Szenenmodelle
  stlObjects.forEach(function(o){ if(o&&o.mesh) stlGrp.remove(o.mesh); });
  stlObjects.length = 0; stlNavIdx = 0;
  renderStlNav();

  // TCP
  tcpList = [{ x:0, y:0, z:0, a:0, b:90, c:0, name:'TCP 1' }];
  tcpNavIdx = 0; tcpSetInputs(tcpList[0]);
  var tni = document.getElementById('tcp-name'); if(tni) tni.value = 'TCP 1';
  renderTcpNav();

  // BASE
  baseList = [{ x:0, y:0, z:0, a:0, b:0, c:0, name:'BASE 1' }];
  baseNavIdx = 0; baseSetInputs(baseList[0]);
  var bni = document.getElementById('base-name'); if(bni) bni.value = 'BASE 1';
  renderBaseNav();

  // Marker + Selektion
  markerGrp.visible = false;
  selSphere.visible = false;
  selectedPosIdx = null;

  setStatus('stopped', 'Neue Szene');
}

function sceneLaden() {
  var inp = document.getElementById('scene-file-in');
  inp.onchange = function(e) {
    var file = e.target.files[0]; if (!file) return;
    inp.value = '';
    var ext = file.name.split('.').pop().toLowerCase();
    if (ext==='stp'||ext==='step') {
      // STEP als Szenenmodell laden
      var reader = new FileReader();
      reader.onload = function(ev) {
        parseGeometry(ev.target.result, file.name).then(function(geo) {
          var mat = new THREE.MeshPhongMaterial({color:0x4499cc,shininess:60,side:THREE.DoubleSide});
          var mesh = new THREE.Mesh(geo, mat);
          stlGrp.add(mesh);
          stlObjects.push({mesh:mesh,name:file.name,buf:ev.target.result,offset:{x:0,y:0,z:0,a:0,b:0,c:0},displayName:''});
          stlNavIdx = stlObjects.length-1;
          renderStlNav();
          setStatus('paused','STEP geladen: '+file.name);
        }).catch(function(err){ alert('STEP Fehler: '+err.message); });
      };
      reader.readAsArrayBuffer(file);
    } else {
      loadKinematicZIP(file);
    }
  };
  inp.click();
}

function sceneSpeichern() {
  var name = ((document.getElementById('kin-name')&&document.getElementById('kin-name').value) || 'szene').replace(/\s+/g,'_');
  var zip = new JSZip();

  // 1. Kinematik-JSON erweitert um TCP-/BASE-/Szenenmodell-Listen
  var data = getKinematicData();
  // Code-Editor Inhalt — bevorzuge code-input.value, Fallback: generateKRL(parsedData)
  var codeEl = document.getElementById('code-input');
  var codeVal = codeEl ? codeEl.value.trim() : '';
  if (!codeVal && typeof generateKRL === 'function') codeVal = generateKRL(parsedData) || '';
  data.code = codeVal;
  data.tcpList  = tcpList.map(function(t){ return Object.assign({},t); });
  data.baseList = baseList.map(function(b){ return Object.assign({},b); });
  data.stlScene = stlObjects.filter(Boolean).map(function(o,i){
    return { name: o.name||('scene_'+(i+1)+'.stl'), displayName: o.displayName||'', offset: Object.assign({},o.offset||{}) };
  });
  data.tcp = { x:TCP_DEF.x, y:TCP_DEF.y, z:TCP_DEF.z, a:TCP_DEF.a, b:TCP_DEF.b, c:TCP_DEF.c };
  zip.file(name + '.json', JSON.stringify(data, null, 2));

  // 2. Achsen-STLs
  for (var i = 0; i < 6; i++) {
    var buf = (window._axisSTLBuffers && window._axisSTLBuffers[i]) || window['_axisSTLBuffer'+i];
    if (buf) zip.file((data.stlFiles['A'+(i+1)].name||('a'+(i+1)))+'.stl', new Uint8Array(buf));
  }

  // 3. Podest + Tool STL
  if (window._pedestalSTLBuffer) zip.file('podest.stl', new Uint8Array(window._pedestalSTLBuffer));
  if (window._toolSTLBuffer) zip.file('tool.stl', new Uint8Array(window._toolSTLBuffer));

  // 4. Szenenmodell-STLs
  stlObjects.forEach(function(o, i) {
    if (!o||!o.buf) return;
    var n = (o.name||('scene_'+(i+1)+'.stl')).replace(/[^a-zA-Z0-9._-]/g,'_');
    if (!n.match(/\.stl$/i)) n += '.stl';
    zip.file('scene/' + n, new Uint8Array(o.buf));
  });

  zip.generateAsync({type:'blob', compression:'DEFLATE', compressionOptions:{level:6}})
    .then(function(blob) {
      var url = URL.createObjectURL(blob);
      var a = document.createElement('a');
      a.href = url; a.download = name + '.zip';
      document.body.appendChild(a); a.click();
      document.body.removeChild(a);
      setTimeout(function(){ URL.revokeObjectURL(url); }, 2000);
    });
}


// ── Library Picker Modal ──────────────────────────────────────────
var _libPickerType = null;
var _libPickerItems = [];

var _allLibItems = [];
var _libPickerTabType = 'all';

function openLibPickerModal(type) {
  _libPickerType = type;
  _libPickerTabType = type === 'all' ? 'all' : type;
  var titles = {all:'📚 Library', robot:'🦾 Roboter', positioner:'🔄 Positionierer', rail:'🛤️ Schienen', object:'📦 Objekte', station:'🏗️ Stationen', endeffektor:'🔧 Endeffektoren', umfeld:'🏭 Umfeld'};
  var el = document.getElementById('libPickerTitle');
  if (el) el.textContent = titles[type] || 'Library';
  document.getElementById('libPickerSearch').value = '';
  document.getElementById('libPickerList').innerHTML = '<div style="color:var(--txt3);padding:8px;font-family:monospace;font-size:12px">Lade…</div>';
  // Set active tab
  document.querySelectorAll('.lp-tab').forEach(function(b){
    var active = b.dataset.lpType === _libPickerTabType;
    b.style.background = active ? 'rgba(37,99,235,.2)' : 'rgba(255,255,255,.05)';
    b.style.borderColor = active ? 'rgba(37,99,235,.4)' : 'rgba(255,255,255,.15)';
    b.style.color = active ? '#60a5fa' : '#6a8fa8';
    b.classList.toggle('on', active);
  });
  var m = document.getElementById('libPickerModal');
  m.style.cssText = 'display:flex;position:fixed;inset:0;z-index:2000;background:rgba(0,0,0,.7);align-items:center;justify-content:center';
  if (_allLibItems.length) { applyLibPickerFilter(); return; }
  fetch('https://cnc-technik.de/robsimul/roblib/api.php?action=list')
    .then(function(r){return r.json();})
    .then(function(data){
      _allLibItems = (data.robots||[]).filter(function(r){ var t=r.type||'robot'; return t==='robot'||t==='station'; });
      applyLibPickerFilter();
    }).catch(function(e){
      document.getElementById('libPickerList').innerHTML = '<div style="color:#f87171;padding:8px;font-family:monospace;font-size:11px">Fehler: '+e.message+'</div>';
    });
}

function applyLibPickerFilter() {
  _libPickerItems = _libPickerTabType === 'all' ? _allLibItems :
    _allLibItems.filter(function(r){ return (r.type||'robot') === _libPickerTabType; });
  renderLibPickerList(document.getElementById('libPickerSearch').value||'');
}

function closeLibPickerModal() {
  document.getElementById('libPickerModal').style.display = 'none';
  _libPickerType = null;
}

function renderLibPickerList(q) {
  var el = document.getElementById('libPickerList'); if(!el) return;
  var items = q ? _libPickerItems.filter(function(r){return r.name.toLowerCase().includes(q.toLowerCase());}) : _libPickerItems;
  if (!items.length) { el.innerHTML='<div style="color:var(--txt3);padding:8px;font-family:monospace;font-size:12px">Keine Einträge.</div>'; return; }
  el.innerHTML = items.map(function(r,i){return '<div style="display:flex;align-items:center;gap:8px;padding:7px 0;border-bottom:1px solid rgba(255,255,255,.06)">'
    +'<div style="flex:1;min-width:0"><div style="font-family:monospace;font-size:12px;color:var(--txt)">'+(r.name||'')+'</div>'
    +'<div style="font-size:10px;color:var(--txt3)">'+(r.marke||'')+' '+(r.modell||'')+'</div></div>'
    +'<button data-lib-pick="'+i+'" style="padding:2px 10px;background:rgba(37,99,235,.2);border:1px solid rgba(37,99,235,.4);color:#60a5fa;border-radius:3px;cursor:pointer;font-family:monospace;font-size:11px">Laden</button>'
    +'</div>';
  }).join('');
  el.querySelectorAll('[data-lib-pick]').forEach(function(btn){
    btn.onclick = function(){ var item=_libPickerItems[+btn.dataset.libPick]; loadLibItem(item.type||_libPickerType, item); closeLibPickerModal(); };
  });
}

document.addEventListener('DOMContentLoaded', function(){
  var si = document.getElementById('libPickerSearch');
  if (si) si.addEventListener('input', function(){ renderLibPickerList(this.value); });
  document.addEventListener('click', function(e){
    var tab = e.target.closest('.lp-tab');
    if (!tab) return;
    _libPickerTabType = tab.dataset.lpType || 'all';
    document.querySelectorAll('.lp-tab').forEach(function(b){
      var active = b.dataset.lpType === _libPickerTabType;
      b.style.background = active ? 'rgba(37,99,235,.2)' : 'rgba(255,255,255,.05)';
      b.style.borderColor = active ? 'rgba(37,99,235,.4)' : 'rgba(255,255,255,.15)';
      b.style.color = active ? '#60a5fa' : '#6a8fa8';
    });
    applyLibPickerFilter();
  });
  document.getElementById('libPickerModal')?.addEventListener('click', function(e){
    if (e.target===document.getElementById('libPickerModal')) closeLibPickerModal();
  });
  // Objekt STL direkt laden
  var oi = document.getElementById('obj-stl-input');
  if (oi) oi.addEventListener('change', function(e){
    Array.from(e.target.files).forEach(function(file){ loadObjectSTL(file); });
    e.target.value='';
  });
});

// ── Library Item Loading ──────────────────────────────────────────
var rails = [];  // {id, name, stlMeshes, grp, offset, position, length, axis}
var positioners = [];  // {id, name, joints, stlMeshes, angles, grp, offset}
var objects = [];      // {id, name, mesh, offset}

async function loadLibItem(type, robot) {
  var resp = await fetch(robot.zip_url);
  var blob = await resp.blob();
  var zip = await JSZip.loadAsync(blob);
  var jsonFile = null;
  zip.forEach(function(path,entry){ if(!entry.dir&&path.endsWith('.json')) jsonFile=entry; });
  if (!jsonFile) { setStatus('paused','Kein JSON in ZIP'); return; }
  var jsonStr = await jsonFile.async('string');
  var data = JSON.parse(jsonStr);
  var stlBufs = {};
  var proms = [];
  zip.forEach(function(path,entry){
    var lp=path.toLowerCase();
    if(!entry.dir&&(lp.endsWith('.stl')||lp.endsWith('.osd'))){
      var fname=path.replace(/.*\//,'').toLowerCase().replace(/\.(stl|osd)$/i,'');
      proms.push(entry.async('arraybuffer').then(function(buf){
        var stlData=lp.endsWith('.osd')?osdToBinaryStl(buf):buf;
        stlBufs[fname]=stlData;
        // Alias: "base" → auch unter 'podest' speichern damit sceneModels.pedestal-Lookup klappt
        if(/base/i.test(fname)&&!stlBufs['podest']) stlBufs['podest']=stlData;
      }));
    }
  });
  await Promise.all(proms);
  var t = type || (data.type) || 'robot';
  if (t==='station') { await loadStation(robot, data, stlBufs); }
  else { applyKinematicData(data, stlBufs); setStatus('paused','Geladen: '+robot.name); }
}

// ── Positionierer ─────────────────────────────────────────────────
async function loadPositioner(robot, data, stlBufs) {
  var pos = { id: robot.id, name: robot.name, grp: new THREE.Group(), stlMeshes: [], angles: [0,0,0,0,0,0], offset:{x:0,y:0,z:0,a:0,b:0,c:0} };
  scene.add(pos.grp);
  // Load STL meshes
  var stlFiles = data.stlFiles || {};
  for (var ax of ['A1','A2','A3','A4','A5','A6']) {
    var info = stlFiles[ax]; if (!info) continue;
    var parts = Array.isArray(info)?info:[info];
    for (var p of parts) {
      var fname=(p.name||'').replace(/\.stl$/i,'').toLowerCase();
      var buf=stlBufs[fname]; if(!buf) continue;
      var geo=stlLoader.parse(buf); geo.computeVertexNormals();
      var mat=new THREE.MeshPhongMaterial({color:p.color||0xe8a020,shininess:80});
      var mesh=new THREE.Mesh(geo,mat); pos.grp.add(mesh); pos.stlMeshes.push(mesh);
    }
  }
  positioners.push(pos);
  renderPositionerList();
  setStatus('paused', 'Positionierer geladen: '+robot.name);
}

function renderPositionerList() {
  var el = document.getElementById('positioner-list'); if(!el) return;
  if (!positioners.length) { el.innerHTML='<div class="empty">Keine Positionierer geladen</div>'; return; }
  el.innerHTML = positioners.map(function(pos,i){
    var o=pos.offset||{};
    return '<div style="border:1px solid rgba(255,255,255,.1);border-radius:4px;padding:8px;margin-bottom:6px">'
      +'<div style="display:flex;align-items:center;gap:6px;margin-bottom:6px">'
      +'<span style="flex:1;font-family:monospace;font-size:12px;color:var(--txt)">'+pos.name+'</span>'
      +'<button onclick="removePositioner('+i+')" style="background:rgba(204,51,51,.15);border:1px solid rgba(204,51,51,.3);color:#f87171;border-radius:3px;padding:1px 6px;cursor:pointer;font-size:12px">✕</button></div>'
      +'<div class="stl-grid">'
      +'<span class="stl-lbl">X</span><input class="stl-inp" type="number" value="'+(o.x||0)+'" oninput="updatePositionerOffset('+i+',&apos;x&apos;,this.value)" step="1"><span class="stl-unit">mm</span>'
      +'<span class="stl-lbl">Y</span><input class="stl-inp" type="number" value="'+(o.y||0)+'" oninput="updatePositionerOffset('+i+',&apos;y&apos;,this.value)" step="1"><span class="stl-unit">mm</span>'
      +'<span class="stl-lbl">Z</span><input class="stl-inp" type="number" value="'+(o.z||0)+'" oninput="updatePositionerOffset('+i+',&apos;z&apos;,this.value)" step="1"><span class="stl-unit">mm</span>'
      +'<span class="stl-lbl">A</span><input class="stl-inp" type="number" value="'+(o.a||0)+'" oninput="updatePositionerOffset('+i+',&apos;a&apos;,this.value)" step="1"><span class="stl-unit">°</span>'
      +'<span class="stl-lbl">B</span><input class="stl-inp" type="number" value="'+(o.b||0)+'" oninput="updatePositionerOffset('+i+',&apos;b&apos;,this.value)" step="1"><span class="stl-unit">°</span>'
      +'<span class="stl-lbl">C</span><input class="stl-inp" type="number" value="'+(o.c||0)+'" oninput="updatePositionerOffset('+i+',&apos;c&apos;,this.value)" step="1"><span class="stl-unit">°</span>'
      +'</div></div>';
  }).join('');
}

function updatePositionerOffset(idx, k, v) {
  if (!positioners[idx]) return;
  positioners[idx].offset[k] = parseFloat(v)||0;
  applyPositionerOffset(idx);
}

function applyPositionerOffset(idx) {
  var pos = positioners[idx]; if(!pos) return;
  var o=pos.offset, r=Math.PI/180;
  pos.grp.position.set(o.x||0,o.y||0,o.z||0);
  pos.grp.rotation.set((o.c||0)*r,(o.b||0)*r,(o.a||0)*r,'ZYX');
}

function removePositioner(idx) {
  var pos=positioners[idx]; if(!pos) return;
  scene.remove(pos.grp);
  positioners.splice(idx,1);
  renderPositionerList();
}

// ── Schienen / Rails ─────────────────────────────────────────────
async function loadRail(robot, data, stlBufs) {
  if (parametricRail.mesh) { parametricRail.grp.remove(parametricRail.mesh); parametricRail.mesh.geometry.dispose(); parametricRail.mesh=null; }
  parametricRail.length   = data.length_mm || 2000;
  parametricRail.height   = data.height_mm || 200;
  parametricRail.width    = data.width_mm  || 400;
  parametricRail.axis     = data.axis      || 'X+';
  parametricRail.name     = robot.name;
  parametricRail.position = 0;
  parametricRail.active   = true;
  parametricRail.transform = {x:0, y:0, z:0, rx:0, ry:0, rz:0};
  _buildParametricRailMesh();
  renderRailPanel();
  _autoPositionRailUnderRobot();
  _applyRailTransform();
  _applyRailRobotPosition();
  setStatus('paused', 'Schiene geladen: ' + robot.name);
}

function renderRailPanel() {
  var el = document.getElementById('rail-panel'); if(!el) return;
  if (!parametricRail.active || !parametricRail.name) {
    el.innerHTML = '<div class="empty">Keine Schiene geladen</div>';
    return;
  }
  var r = parametricRail;
  el.innerHTML = '<div style="border:1px solid rgba(255,255,255,.1);border-radius:4px;padding:8px">'
    +'<div style="display:flex;align-items:center;gap:6px;margin-bottom:6px">'
    +'<span style="flex:1;font-family:monospace;font-size:12px;color:var(--txt)">🛤️ '+r.name+'</span>'
    +'<button onclick="removeParametricRail()" style="background:rgba(204,51,51,.15);border:1px solid rgba(204,51,51,.3);color:#f87171;border-radius:3px;padding:1px 6px;cursor:pointer;font-size:12px">✕</button>'
    +'</div>'
    +'<div style="font-size:10px;color:var(--txt3);font-family:monospace;margin-bottom:8px">'+r.length+' × '+r.width+' × '+r.height+' mm | '+r.axis+'</div>'
    +'<div style="font-size:10px;color:var(--txt3);font-family:monospace;letter-spacing:.06em;margin-bottom:4px">ROBOTER-POSITION</div>'
    +'<div style="display:flex;align-items:center;gap:6px;margin-bottom:8px">'
    +'<input type="range" id="rail-pos-slider" min="0" max="'+r.length+'" value="'+r.position+'" step="1" oninput="setRailPosition(+this.value)" style="flex:1;accent-color:var(--acc)">'
    +'<input class="stl-inp" id="rail-pos-val" type="number" value="'+r.position+'" min="0" max="'+r.length+'" step="1" oninput="setRailPosition(+this.value)" style="width:72px">'
    +'<span class="stl-unit">mm</span></div>'
    +'<div style="font-size:10px;color:var(--txt3);font-family:monospace;letter-spacing:.06em;margin-bottom:4px">POSITION IM RAUM</div>'
    +'<div class="stl-grid">'
    +'<span class="stl-lbl">X</span><input class="stl-inp" id="rail-x" type="number" value="'+Math.round(r.transform.x)+'" step="1" oninput="updateRailTransform()"><span class="stl-unit">mm</span>'
    +'<span class="stl-lbl">Y</span><input class="stl-inp" id="rail-y" type="number" value="'+Math.round(r.transform.y)+'" step="1" oninput="updateRailTransform()"><span class="stl-unit">mm</span>'
    +'<span class="stl-lbl">Z</span><input class="stl-inp" id="rail-z" type="number" value="'+Math.round(r.transform.z)+'" step="1" oninput="updateRailTransform()"><span class="stl-unit">mm</span>'
    +'<span class="stl-lbl">Rx</span><input class="stl-inp" id="rail-rx" type="number" value="'+r.transform.rx+'" step="1" oninput="updateRailTransform()"><span class="stl-unit">°</span>'
    +'<span class="stl-lbl">Ry</span><input class="stl-inp" id="rail-ry" type="number" value="'+r.transform.ry+'" step="1" oninput="updateRailTransform()"><span class="stl-unit">°</span>'
    +'<span class="stl-lbl">Rz</span><input class="stl-inp" id="rail-rz" type="number" value="'+r.transform.rz+'" step="1" oninput="updateRailTransform()"><span class="stl-unit">°</span>'
    +'</div></div>';
}

function removeParametricRail() {
  if (parametricRail.mesh) { parametricRail.grp.remove(parametricRail.mesh); parametricRail.mesh.geometry.dispose(); parametricRail.mesh=null; }
  parametricRail.active=false; parametricRail.name=null;
  robotGrp.position.set(0,0,0);
  robotGrp.position.set(0,0,0);
  renderRailPanel();
}

function renderRailList() { renderRailPanel(); }


// ── Parametrische Schiene ─────────────────────────────────────────
var parametricRail = {
  active: false, mesh: null, grp: null,
  axis: 'Y+', position: 0,
  length: 2000, height: 200, width: 400,
  transform: {x:0, y:0, z:0, rx:0, ry:0, rz:0}
};
(function(){ parametricRail.grp = new THREE.Group(); scene.add(parametricRail.grp); })();

function toggleRail(active) {
  parametricRail.active = active;
  if (!active) {
    if (parametricRail.mesh) { parametricRail.grp.remove(parametricRail.mesh); parametricRail.mesh.geometry.dispose(); parametricRail.mesh=null; }
    return;
  }
  parametricRail.length = parseFloat(document.getElementById('rail-length').value)||2000;
  parametricRail.height = parseFloat(document.getElementById('rail-height').value)||200;
  parametricRail.width  = parseFloat(document.getElementById('rail-width').value)||400;
  var slider=document.getElementById('rail-pos-slider'); if(slider) slider.max=parametricRail.length;
  _buildParametricRailMesh();
  _autoPositionRailUnderRobot();
  _applyRailTransform();
  _applyRailRobotPosition();
}

function _buildParametricRailMesh() {
  if (parametricRail.mesh) { parametricRail.grp.remove(parametricRail.mesh); parametricRail.mesh.geometry.dispose(); }
  var L=parametricRail.length, H=parametricRail.height, W=parametricRail.width, ax=parametricRail.axis;
  var geo = (ax==='X+'||ax==='X-') ? new THREE.BoxGeometry(L,H,W) :
            (ax==='Y+'||ax==='Y-') ? new THREE.BoxGeometry(W,L,H) :
                                     new THREE.BoxGeometry(W,H,L);
  var mat=new THREE.MeshPhongMaterial({color:0x2563eb,transparent:true,opacity:0.3,side:THREE.DoubleSide});
  parametricRail.mesh=new THREE.Mesh(geo,mat);
  parametricRail.grp.add(parametricRail.mesh);
}

function _autoPositionRailUnderRobot() {
  var t=parametricRail.transform;
  if (t.rx||t.ry||t.rz) return;
  // Robot stays at origin; set base offset to 0 (rail will slide via _applyRailRobotPosition)
  t.x=0; t.y=0; t.z=0;
  var el;
  if((el=document.getElementById('rail-x'))) el.value=0;
  if((el=document.getElementById('rail-y'))) el.value=0;
  if((el=document.getElementById('rail-z'))) el.value=0;
}

function updateRailGeometry() {
  parametricRail.length = parseFloat(document.getElementById('rail-length').value)||2000;
  parametricRail.height = parseFloat(document.getElementById('rail-height').value)||200;
  parametricRail.width  = parseFloat(document.getElementById('rail-width').value)||400;
  var slider=document.getElementById('rail-pos-slider'); if(slider) slider.max=parametricRail.length;
  if (!parametricRail.active) return;
  _buildParametricRailMesh();
  _applyRailRobotPosition();
}

function setRailAxis(ax) {
  parametricRail.axis = ax;
  document.querySelectorAll('.rail-axis-btn').forEach(function(btn){
    var on=btn.dataset.railAxis===ax;
    btn.style.background=on?'rgba(37,99,235,.2)':'rgba(255,255,255,.05)';
    btn.style.border=on?'1px solid rgba(37,99,235,.4)':'1px solid rgba(255,255,255,.15)';
    btn.style.color=on?'#60a5fa':'#6a8fa8';
  });
  if (!parametricRail.active) return;
  _buildParametricRailMesh();
  _applyRailRobotPosition();
}

function setRailPosition(val) {
  parametricRail.position = parseFloat(val)||0;
  var slider=document.getElementById('rail-pos-slider'), inp=document.getElementById('rail-pos-val');
  if(slider) slider.value=parametricRail.position;
  if(inp)    inp.value=parametricRail.position;
  if (!parametricRail.active) return;
  _applyRailRobotPosition();
}

function updateRailTransform() {
  var t=parametricRail.transform;
  t.x  = parseFloat(document.getElementById('rail-x').value)||0;
  t.y  = parseFloat(document.getElementById('rail-y').value)||0;
  t.z  = parseFloat(document.getElementById('rail-z').value)||0;
  t.rx = parseFloat(document.getElementById('rail-rx').value)||0;
  t.ry = parseFloat(document.getElementById('rail-ry').value)||0;
  t.rz = parseFloat(document.getElementById('rail-rz').value)||0;
  _applyRailTransform();
  if (parametricRail.active) _applyRailRobotPosition();
}

function _applyRailTransform() {
  var t=parametricRail.transform, r=Math.PI/180;
  parametricRail.grp.position.set(t.x, t.y, t.z);
  parametricRail.grp.rotation.set(t.rx*r, t.ry*r, t.rz*r, 'XYZ');
}

function _applyRailRobotPosition() {
  var p=parametricRail.position, ax=parametricRail.axis;
  var t=parametricRail.transform, r=Math.PI/180;
  var dir=new THREE.Vector3();
  if      (ax==='X+') dir.set( 1,0,0);
  else if (ax==='X-') dir.set(-1,0,0);
  else if (ax==='Y+') dir.set(0, 1,0);
  else if (ax==='Y-') dir.set(0,-1,0);
  else if (ax==='Z+') dir.set(0,0, 1);
  else                dir.set(0,0,-1);
  dir.applyEuler(new THREE.Euler(t.rx*r, t.ry*r, t.rz*r, 'XYZ'));
  robotGrp.position.set(t.x + dir.x*p, t.y + dir.y*p, t.z + dir.z*p);
  _applyRailTransform();
}

// ── Bewegliche Objekte ────────────────────────────────────────────
async function loadObjectFromLib(robot, data, stlBufs) {
  var stlFiles = data.stlFiles || {};
  var firstPart = null;
  for (var ax of Object.keys(stlFiles)) {
    var info=stlFiles[ax]; var parts=Array.isArray(info)?info:[info];
    if (parts[0]) { firstPart=parts[0]; break; }
  }
  if (!firstPart) { setStatus('paused','Kein STL im Objekt'); return; }
  var fname=(firstPart.name||'').replace(/\.stl$/i,'').toLowerCase();
  var buf=stlBufs[fname]; if(!buf) { setStatus('paused','STL nicht gefunden'); return; }
  var geo=stlLoader.parse(buf); geo.computeVertexNormals();
  var mat=new THREE.MeshPhongMaterial({color:firstPart.color||0x4499cc,shininess:60,side:THREE.DoubleSide});
  var mesh=new THREE.Mesh(geo,mat);
  stlGrp.add(mesh);
  stlObjects.push({mesh:mesh,name:robot.name+'.stl',buf:buf,offset:{x:0,y:0,z:0,a:0,b:0,c:0},displayName:robot.name});
  stlNavIdx=stlObjects.length-1;
  renderStlNav();
  setStatus('paused','Objekt geladen: '+robot.name);
}

async function loadObjectSTL(file) {
  var buf=await file.arrayBuffer();
  var geo=await parseGeometry(buf,file.name); geo.computeVertexNormals();
  var mat=new THREE.MeshPhongMaterial({color:0x4499cc,shininess:60,side:THREE.DoubleSide});
  var mesh=new THREE.Mesh(geo,mat);
  stlGrp.add(mesh);
  stlObjects.push({mesh:mesh,name:file.name,buf:new Uint8Array(buf),offset:{x:0,y:0,z:0,a:0,b:0,c:0},displayName:file.name.replace(/\.stl$/i,'')});
  stlNavIdx=stlObjects.length-1;
  renderStlNav();
}

// ── Station ───────────────────────────────────────────────────────
async function loadStation(robot, data, stlBufs) {
  var c = data.components || {};
  // Robot
  if (c.robot && c.robot.joints) { applyKinematicData(c.robot, stlBufs); }
  else if (data.joints) { applyKinematicData(data, stlBufs); }
  // Rail
  if (c.rail) { await loadRail(robot, c.rail, stlBufs); }
  // Positioners
  if (Array.isArray(c.positioners)) {
    for (var i=0;i<c.positioners.length;i++) await loadPositioner(robot, c.positioners[i], stlBufs);
  }
  // Fixtures (feste Objekte)
  if (Array.isArray(c.fixtures)) { c.fixtures.forEach(loadFixture); }
  // Labels (bewegliche Objekte) – als statische STL/Objekte laden
  if (Array.isArray(c.labels)) {
    for (var i=0;i<c.labels.length;i++) await loadObjectFromLib(robot, c.labels[i], stlBufs);
  }
  // Endeffektoren
  if (Array.isArray(c.effectors)) {
    for (var i=0;i<c.effectors.length;i++) await loadEndeffektor(robot, c.effectors[i], stlBufs);
  }
  // Umfeld
  if (Array.isArray(c.environment)) {
    for (var i=0;i<c.environment.length;i++) await loadUmfeld(robot, c.environment[i], stlBufs);
  }
  setStatus('paused','Station geladen: '+robot.name);
}

// ── Fixture (festes Objekt, Primitiv) ────────────────────────────
function loadFixture(data) {
  var geo;
  if ((data.objectType||'box') === 'cylinder') {
    geo = new THREE.CylinderGeometry(
      data.radius||200, data.radius||200,
      data.height||500, 32
    );
  } else {
    geo = new THREE.BoxGeometry(
      data.length||500, data.height||500, data.width||500
    );
  }
  var col = parseInt((data.color||'#607080').replace('#',''), 16);
  var mat = new THREE.MeshPhongMaterial({color:col, shininess:40, transparent:true, opacity:0.75, side:THREE.DoubleSide});
  var mesh = new THREE.Mesh(geo, mat);
  mesh.position.set(data.x||0, data.y||0, data.z||0);
  mesh.setRotationFromEuler(kukaEuler(data.rz||0, data.ry||0, data.rx||0));
  stlGrp.add(mesh);
  stlObjects.push({mesh:mesh, name:(data.name||'fixture'), buf:null, offset:{x:data.x||0,y:data.y||0,z:data.z||0,a:data.rz||0,b:data.ry||0,c:data.rx||0}, displayName:data.name||'Fixture'});
  renderStlNav();
}

// ── Endeffektor ───────────────────────────────────────────────────
async function loadEndeffektor(robot, data, stlBufs) {
  var stlKey = (data.stlFile||'').replace(/.*\//,'').replace(/\.stl$/i,'').toLowerCase();
  var buf = stlBufs[stlKey];
  if (!buf) { setStatus('paused','Endeffektor STL nicht gefunden: '+stlKey); return; }
  var geo = stlLoader.parse(buf); geo.computeVertexNormals();
  var mat = new THREE.MeshPhongMaterial({color:0xdd9944, shininess:60, side:THREE.DoubleSide});
  if (toolMesh) { scene.remove(toolMesh); toolMesh.geometry.dispose(); toolMesh.material.dispose(); }
  toolMesh = new THREE.Mesh(geo, mat);
  scene.add(toolMesh);
  window._toolSTLBuffer = buf;
  var o = data.offset||{};
  sceneSTLOffsets.tool = {x:o.x||0, y:o.y||0, z:o.z||0, a:o.rz||0, b:o.ry||0, c:o.rx||0, name:stlKey};
  var el = document.getElementById('tool-filename'); if(el) el.textContent = stlKey;
  var ec = document.getElementById('tool-controls'); if(ec) ec.style.display='block';
  setStatus('paused','Endeffektor geladen: '+(data.name||stlKey));
}

// ── Umfeld (Umgebungsobjekt, STL) ─────────────────────────────────
async function loadUmfeld(robot, data, stlBufs) {
  var stlKey = (data.stlFile||'').replace(/.*\//,'').replace(/\.stl$/i,'').toLowerCase();
  var buf = stlBufs[stlKey];
  if (!buf) { setStatus('paused','Umfeld STL nicht gefunden: '+stlKey); return; }
  var geo = stlLoader.parse(buf); geo.computeVertexNormals();
  var mat = new THREE.MeshPhongMaterial({color:0x607080, shininess:40, transparent:true, opacity:0.6, side:THREE.DoubleSide});
  var mesh = new THREE.Mesh(geo, mat);
  var o = data.offset||{};
  mesh.position.set(o.x||0, o.y||0, o.z||0);
  mesh.setRotationFromEuler(kukaEuler(o.rz||0, o.ry||0, o.rx||0));
  stlGrp.add(mesh);
  stlObjects.push({mesh:mesh, name:stlKey, buf:buf, offset:{x:o.x||0,y:o.y||0,z:o.z||0,a:o.rz||0,b:o.ry||0,c:o.rx||0}, displayName:data.name||stlKey});
  renderStlNav();
  setStatus('paused','Umfeld geladen: '+(data.name||stlKey));
}

window.toggleRail = toggleRail;
window.setRailAxis = setRailAxis;
window.setRailPosition = setRailPosition;
window.updateRailGeometry = updateRailGeometry;
window.updateRailTransform = updateRailTransform;
window.removeParametricRail = removeParametricRail;

function newKinematic() {
  if (!confirm(t('confirm_new_kin'))) return;
  JOINTS_DEF.forEach(j => { j.off=[0,0,0]; j.min=-180; j.max=180; });
  const el = document.getElementById('kin-name');
  if (el) el.value = 'Neue Kinematik';
  buildKinConfig();
  buildRobotModel(jointAngles);
}

// ── TCP ──────────────────────────────────────
function getTCPJSON() {
  return JSON.stringify({
    name: 'TCP',
    tcp: {x: TCP_DEF.x, y: TCP_DEF.y, z: TCP_DEF.z, a: TCP_DEF.a, b: TCP_DEF.b, c: TCP_DEF.c}
  }, null, 2);
}

function saveTCP() {
  downloadFile(getTCPJSON(), 'tcp.json');
}

function loadTCP() {
  const inp = document.getElementById('tcp-file-in');
  inp.onchange = e => {
    const file = e.target.files[0]; if (!file) return;
    const reader = new FileReader();
    reader.onload = ev => {
      try {
        let data;
        if (file.name.endsWith('.xml')) {
          const doc = new DOMParser().parseFromString(ev.target.result, 'text/xml');
          const t = doc.querySelector('tcp');
          data = {tcp:{x:+(t&&t.getAttribute)('x'),y:+(t&&t.getAttribute)('y'),z:+(t&&t.getAttribute)('z'),a:+(t&&t.getAttribute)('a'),b:+(t&&t.getAttribute)('b'),c:+(t&&t.getAttribute)('c')}};
        } else {
          data = JSON.parse(ev.target.result);
        }
        if (data.tcp) {
          const t = data.tcp;
          ['x','y','z','a','b','c'].forEach(k => {
            const el = document.getElementById('tcp-'+k);
            if (el && t[k] !== undefined) el.value = t[k];
          });
          updateTCPDef();
          setStatus('paused', 'TCP geladen');
        }
      } catch(err) { alert('Fehler: ' + err.message); }
      inp.value = '';
    };
    reader.readAsText(file);
  };
  inp.click();
}

// ── Programm ────────────────────────────────
// ── ABB 3DP Binary Format ─────────────────────────────────────────────────────
var _3DP_BLOCK = 232;
var _3DP_MAGIC = [0x64,0x61,0x74,0x61,0x33,0x64,0x70,0x00];
var _3DP_CONST_ID = [0x6c,0xea,0x3a,0x16];
var _3DP_CONST_PRE = [0xe8,0xc4,0x00];
var _3DP_GLOBAL = [36.5612,22263.7031,24466.2812,0.0,17309.2344];

function _3dp_readF32(buf, off) {
  var dv = new DataView(buf, off, 4); return dv.getFloat32(0, true);
}
function _3dp_writeF32(dv, off, v) { dv.setFloat32(off, v, true); }

function _3dp_quatToABC(q1,q2,q3,q4) {
  var A = Math.atan2(2*(q1*q4+q2*q3), 1-2*(q3*q3+q4*q4)) * 180/Math.PI;
  var sinB = 2*(q1*q3-q4*q2);
  sinB = Math.max(-1, Math.min(1, sinB));
  var B = Math.asin(sinB) * 180/Math.PI;
  var C = Math.atan2(2*(q1*q2+q3*q4), 1-2*(q2*q2+q3*q3)) * 180/Math.PI;
  return {A:A, B:B, C:C};
}

function _3dp_abcToQuat(Adeg,Bdeg,Cdeg) {
  var a=Adeg*Math.PI/180, b=Bdeg*Math.PI/180, c=Cdeg*Math.PI/180;
  var ca=Math.cos(a/2), sa=Math.sin(a/2);
  var cb=Math.cos(b/2), sb=Math.sin(b/2);
  var cc=Math.cos(c/2), sc=Math.sin(c/2);
  return {
    q1: ca*cb*cc + sa*sb*sc,
    q2: ca*cb*sc - sa*sb*cc,
    q3: ca*sb*cc + sa*cb*sc,
    q4: sa*cb*cc - ca*sb*sc
  };
}

function _3dp_parseBlock(buf, bOff) {
  var u8 = new Uint8Array(buf, bOff, 8);
  for (var i=0;i<8;i++) if(u8[i]!==_3DP_MAGIC[i]) return null;
  var x=_3dp_readF32(buf,bOff+35), y=_3dp_readF32(buf,bOff+39), z=_3dp_readF32(buf,bOff+43);
  var q1=_3dp_readF32(buf,bOff+47), q2=_3dp_readF32(buf,bOff+51);
  var q3=_3dp_readF32(buf,bOff+55), q4=_3dp_readF32(buf,bOff+59);
  var spd=_3dp_readF32(buf,bOff+107), zone=_3dp_readF32(buf,bOff+111);
  if (!isFinite(x)||!isFinite(y)||!isFinite(z)||spd<=0||spd>100000) return null;
  var abc=_3dp_quatToABC(q1,q2,q3,q4);
  return {x:x, y:y, z:z, A:abc.A, B:abc.B, C:abc.C, spd:spd, zone:zone};
}

function _3dp_encodeBlock(pt, gp) {
  var buf = new ArrayBuffer(_3DP_BLOCK);
  var u8 = new Uint8Array(buf);
  var dv = new DataView(buf);
  _3DP_MAGIC.forEach(function(b,i){u8[i]=b;});
  _3DP_CONST_ID.forEach(function(b,i){u8[8+i]=b;});
  (gp||_3DP_GLOBAL).forEach(function(v,i){dv.setFloat32(12+i*4,v,true);});
  _3DP_CONST_PRE.forEach(function(b,i){u8[32+i]=b;});
  _3dp_writeF32(dv,35,pt.x); _3dp_writeF32(dv,39,pt.y); _3dp_writeF32(dv,43,pt.z);
  var q = _3dp_abcToQuat(pt.A||0, pt.B||0, pt.C||0);
  _3dp_writeF32(dv,47,q.q1); _3dp_writeF32(dv,51,q.q2);
  _3dp_writeF32(dv,55,q.q3); _3dp_writeF32(dv,59,q.q4);
  _3dp_writeF32(dv,107,pt.spd); _3dp_writeF32(dv,111,pt.zone||0); _3dp_writeF32(dv,115,1.0);
  return buf;
}

function _3dp_spdName(mmps) {
  var std={5:'v5',10:'v10',20:'v20',25:'v25',30:'v30',40:'v40',50:'v50',
           60:'v60',80:'v80',100:'v100',150:'v150',200:'v200',300:'v300',400:'v400',500:'v500'};
  var r=Math.round(mmps); if(std[r]) return std[r];
  var c=Object.keys(std).reduce(function(a,b){return Math.abs(b-mmps)<Math.abs(a-mmps)?b:a;});
  return std[c];
}

async function _3dp_loadZip(file) {
  var zip = await JSZip.loadAsync(file);
  var printFiles = Object.keys(zip.files)
    .filter(function(n){return /Print_\d+$/.test(n);})
    .sort(function(a,b){
      return parseInt(a.match(/\d+$/)[0]) - parseInt(b.match(/\d+$/)[0]);
    });
  if (!printFiles.length) { alert('Keine Print_N Dateien in ZIP gefunden.'); return; }

  var lines = [];
  var totalPts = 0;
  for (var fi=0; fi<printFiles.length; fi++) {
    var ab = await zip.files[printFiles[fi]].async('arraybuffer');
    var nb = Math.floor(ab.byteLength/_3DP_BLOCK);
    var layerPts = [];
    for (var b=0; b<nb; b++) {
      var pt = _3dp_parseBlock(ab, b*_3DP_BLOCK);
      if (pt) layerPts.push(pt);
    }
    if (!layerPts.length) continue;
    var ln = parseInt(printFiles[fi].match(/\d+$/)[0]);
    lines.push('; Layer ' + ln);
    layerPts.forEach(function(pt, idx) {
      if (idx===0 || Math.abs(pt.spd - layerPts[idx-1].spd)>0.5)
        lines.push('$VEL.CP=' + (pt.spd/1000).toFixed(4));
      var moveType = (idx<3) ? 'PTP' : 'LIN';
      lines.push(moveType + ' {X ' + pt.x.toFixed(3) + ',Y ' + pt.y.toFixed(3) +
        ',Z ' + pt.z.toFixed(3) + ',A ' + pt.A.toFixed(3) +
        ',B ' + pt.B.toFixed(3) + ',C ' + pt.C.toFixed(3) + '}');
      totalPts++;
    });
  }
  document.getElementById('code-input').value = lines.join('\n');
  rebuildGutter();
  parseKRL(lines.join('\n'));
  setStatus('paused', 'ABB 3DP geladen: ' + printFiles.length + ' Layer, ' + totalPts + ' Punkte');
}

async function _3dp_saveZip() {
  if (typeof JSZip === 'undefined') { alert('JSZip nicht geladen.'); return; }
  var pd = parsedData;
  if (!pd||!pd.positions||!pd.positions.length) { alert('Kein Programm geparst.'); return; }
  var ptsPerLayer = 997;
  var zip = new JSZip();
  var chunks = [];
  for (var i=0; i<pd.positions.length; i+=ptsPerLayer)
    chunks.push(pd.positions.slice(i, i+ptsPerLayer));

  for (var ci=0; ci<chunks.length; ci++) {
    var chunk = chunks[ci];
    var blocks = [];
    var p0 = chunk[0];
    [[p0.X,p0.Y,100,133.33],[p0.X,p0.Y,30,150],[p0.X,p0.Y,2,83.33]].forEach(function(r){
      blocks.push(_3dp_encodeBlock({x:r[0],y:r[1],z:r[2],A:p0.A||0,B:p0.B||0,C:p0.C||0,spd:r[3],zone:0}));
    });
    chunk.forEach(function(pos){
      blocks.push(_3dp_encodeBlock({x:pos.X,y:pos.Y,z:pos.Z,A:pos.A||0,B:pos.B||0,C:pos.C||0,spd:(pos.velCP||0.1)*1000,zone:0}));
    });
    while(blocks.length<1000) blocks.push(new ArrayBuffer(_3DP_BLOCK));
    var total = new Uint8Array(_3DP_BLOCK*1000);
    blocks.slice(0,1000).forEach(function(b,i){total.set(new Uint8Array(b),i*_3DP_BLOCK);});
    zip.file('Print_'+(ci+1), total);
  }
  var blob = await zip.generateAsync({type:'blob',compression:'DEFLATE'});
  var a = document.createElement('a'); a.href=URL.createObjectURL(blob);
  a.download='programm_3dp.zip'; a.click();
}
// ── Ende ABB 3DP ──────────────────────────────────────────────────────────────

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


function toggleSteuerPanel(){
  const p=document.getElementById('steuer-panel');
  p.classList.toggle('visible');
  document.getElementById('btn-steuer').classList.toggle('on', p.classList.contains('visible'));
  if(p.classList.contains('visible')) updateSteuerPanel();
}

function buildSteuerAxes(){
  const el=document.getElementById('sp-axes');
  if(!el) return;
  el.innerHTML=JOINTS_DEF.map((j,i)=>`
    <div class="sp-axis-row">
      <span class="sp-axis-lbl">A${i+1}:</span>
      <span class="sp-axis-min">${j.min}</span>
      <div class="sp-slider-wrap" id="sp-wrap${i}" style="cursor:ew-resize">
        <div class="sp-slider-fill" id="sp-fill${i}"></div>
        <div class="sp-slider-thumb" id="sp-thumb${i}"></div>
      </div>
      <span class="sp-axis-max">${j.max}</span>
      <input class="sp-axis-inp" id="sp-av${i}" type="number"
        value="0" step="0.1" min="${j.min}" max="${j.max}"
        onchange="spAxisChanged(${i},this.value)"
        onkeydown="if(event.key==='Enter')spAxisChanged(${i},this.value)">
    </div>`).join('');

  // Make sliders draggable
  JOINTS_DEF.forEach((j,i)=>{
    const wrap=document.getElementById('sp-wrap'+i);
    if(!wrap) return;
    let dragging=false;
    wrap.addEventListener('mousedown',e=>{
      e.preventDefault(); dragging=true;
      document.addEventListener('mousemove',onDrag);
      document.addEventListener('mouseup',()=>{dragging=false;document.removeEventListener('mousemove',onDrag);},{once:true});
      function onDrag(ev){
        if(!dragging) return;
        const r=wrap.getBoundingClientRect();
        const pct=Math.max(0,Math.min(1,(ev.clientX-r.left)/r.width));
        const val=j.min+(j.max-j.min)*pct;
        spAxisChanged(i, val.toFixed(2));
      }
      onDrag(e);
    });
    // Also make TCP values editable on click
  });

  // Make TCP display values editable (click to edit) - no arguments.callee
  function makeTCPEditable(id){
    const el=document.getElementById(id);
    if(!el) return;
    el.style.cursor='text';
    el.title='Klicken zum Bearbeiten';
    el.onclick=function(){
      const cur=this.textContent;
      const parent=this.parentNode;
      const inp=document.createElement('input');
      inp.type='number'; inp.value=parseFloat(cur)||0; inp.step='0.1';
      inp.style.cssText='width:100%;background:#070d1a;color:#9ecfea;border:1px solid var(--acc);border-radius:2px;font-family:inherit;font-size:inherit;padding:0 2px;outline:none;text-align:right';
      this.replaceWith(inp); inp.focus(); inp.select();
      function commit(){
        const span=document.createElement('span');
        span.id=id; span.className='sp-tcp-val';
        span.textContent=parseFloat(inp.value||0).toFixed(3);
        inp.replaceWith(span);
        makeTCPEditable(id);
        spTCPChanged();
      }
      inp.addEventListener('blur',commit);
      inp.addEventListener('keydown',e=>{
        if(e.key==='Enter'){e.preventDefault();inp.blur();}
        if(e.key==='Escape'){inp.value=parseFloat(cur)||0;inp.blur();}
      });
    };
  }
  ['sp-x','sp-y','sp-z','sp-a','sp-b','sp-c'].forEach(makeTCPEditable);
}

function spAxisChanged(i, val){
  const v=Math.max(JOINTS_DEF[i].min, Math.min(JOINTS_DEF[i].max, parseFloat(val)||0));
  const newAngles=jointAngles.slice();
  newAngles[i]=v;
  applyAngles(newAngles);
  document.getElementById('jog-status').textContent='A'+(i+1)+': '+v.toFixed(2)+'°';
}

function spTCPChanged(){
  const g=id=>{const el=document.getElementById(id);return parseFloat((el&&el.textContent)||(el&&el.value))||0;};
  const x=g('sp-x'),y=g('sp-y'),z=g('sp-z'),a=g('sp-a'),b=g('sp-b'),cc=g('sp-c');
  const res=solveIK(x,y,z,a,b,cc);
  if(res.ok){
    applyAngles(res.angles);
    document.getElementById('jog-status').textContent='IK OK  Δ'+res.score.toFixed(2)+'mm';
  } else {
    document.getElementById('jog-status').textContent='IK fehlgeschlagen  Δ'+res.score.toFixed(1)+'mm';
  }
}

function updateSteuerPanel(){
  const p=document.getElementById('steuer-panel');
  if(!(p&&p.classList).contains('visible')) return;
  // Axes display
  JOINTS_DEF.forEach((j,i)=>{
    const ang=jointAngles[i];
    const pct=Math.max(0,Math.min(100,(ang-j.min)/(j.max-j.min)*100));
    const fill=document.getElementById('sp-fill'+i);
    const thumb=document.getElementById('sp-thumb'+i);
    const inp=document.getElementById('sp-av'+i);
    if(fill) fill.style.width=pct+'%';
    if(thumb) thumb.style.left=pct+'%';
    if(inp&&document.activeElement!==inp) inp.value=ang.toFixed(2);
  });
  // TCP display + jog inputs
  const fk=fkAll(jointAngles);
  const tcp=fk.pts[7];
  if(tcp){
    const f=v=>v.toFixed(3);
    const setV=(id,v)=>{const el=document.getElementById(id);if(el)el.textContent=v;};
    setV('sp-x',f(tcp[0]));setV('sp-y',f(tcp[1]));setV('sp-z',f(tcp[2]));
    const R=fk.tcp_rot;
    const B=-Math.asin(Math.max(-1,Math.min(1,R[2][0])));
    const cb=Math.cos(B);
    let A,C;
    if(Math.abs(cb)>1e-6){A=Math.atan2(R[1][0]/cb,R[0][0]/cb);C=Math.atan2(R[2][1]/cb,R[2][2]/cb);}
    else{A=0;C=Math.atan2(-R[1][2],R[1][1]);}
    const nd=v=>{let d=v*180/Math.PI;while(d>180)d-=360;while(d<=-180)d+=360;return d;}
    const ndS=v=>nd(v).toFixed(3);
    setV('sp-a',ndS(A));setV('sp-b',ndS(B));setV('sp-c',ndS(C));

  }
}

// ── Insert PTP (axis angles) into program ──
function insertPTP(){
  const a=jointAngles;
  const velMmMin=parseFloat((document.getElementById('jog-feed')&&document.getElementById('jog-feed').value))||250;
  const krl=`PTP {A1 ${a[0].toFixed(3)}, A2 ${a[1].toFixed(3)}, A3 ${a[2].toFixed(3)}, A4 ${a[3].toFixed(3)}, A5 ${a[4].toFixed(3)}, A6 ${a[5].toFixed(3)}}`;
  appendToProgram(krl, velMmMin);
  document.getElementById('jog-status').textContent='✓ PTP eingefügt';
}

function insertLIN(){
  const fk=fkAll(jointAngles); const tcp=fk.pts[7];
  const R=fk.tcp_rot;
  const B=-Math.asin(Math.max(-1,Math.min(1,R[2][0])));
  const cb=Math.cos(B);
  let A=0,C=0;
  if(Math.abs(cb)>1e-6){A=Math.atan2(R[1][0]/cb,R[0][0]/cb);C=Math.atan2(R[2][1]/cb,R[2][2]/cb);}
  else{C=Math.atan2(-R[1][2],R[1][1]);}
  const nd=v=>{let d=v*180/Math.PI;while(d>180)d-=360;while(d<=-180)d+=360;return d;}
  const vel=parseFloat((document.getElementById('jog-feed')&&document.getElementById('jog-feed').value))||250; // mm/min
  const krl=`LIN {X ${tcp[0].toFixed(3)}, Y ${tcp[1].toFixed(3)}, Z ${tcp[2].toFixed(3)}, A ${nd(A).toFixed(3)}, B ${nd(B).toFixed(3)}, C ${nd(C).toFixed(3)}}`;
  appendToProgram(krl, vel);
  document.getElementById('jog-status').textContent='✓ LIN eingefügt';
}

// ── Append line to KRL editor (before END) ──
// vel: mm/s for LIN ($VEL.CP), null for PTP
// velPct: % for PTP (BAS #VEL_PTP), undefined for LIN
function appendToProgram(krlLine, vel){
  // vel in mm/min → m/s = vel / 60000
  const ta=document.getElementById('code-input');
  let code=ta.value;
  const lines=code.split(/\r?\n/);
  let endIdx=lines.length-1;
  for(let i=lines.length-1;i>=0;i--){
    if(/^\s*END\s*$/i.test(lines[i])){endIdx=i;break;}
  }
  const insLines=[];
  if(vel!==null && vel!==undefined){
    const mPerSec = vel / 60000;
    insLines.push(`$VEL.CP=${mPerSec.toFixed(4)}`);
  }
  insLines.push(krlLine);
  lines.splice(endIdx,0,...insLines);
  ta.value=lines.join('\n');
  rebuildGutter();
  // Wenn Formular aktiv: fvBuild aufrufen + eingefügte Zeile öffnen
  if (typeof FormatRegistry !== 'undefined' && FormatRegistry.getActiveId() === 'kuka-form') {
    if (typeof fvBuild === 'function') {
      // Zeige die neu eingefügte Zeile (erste insLines-Zeile)
      var newLineIdx = endIdx + insLines.length - 1;
      if (typeof fvExpandedLine !== 'undefined') fvExpandedLine = newLineIdx;
      fvBuild(newLineIdx);
    }
  }
}

// Drag steuerungspanel
(function(){
  let dragging=false,ox=0,oy=0;
  const hdr=document.getElementById('steuer-hdr');
  const panel=document.getElementById('steuer-panel');
  if(!hdr||!panel) return;
  hdr.addEventListener('mousedown',e=>{if(e.button!==0)return;dragging=true;ox=e.clientX-panel.offsetLeft;oy=e.clientY-panel.offsetTop;e.preventDefault();});
  window.addEventListener('mousemove',e=>{if(!dragging)return;panel.style.left=(e.clientX-ox)+'px';panel.style.top=(e.clientY-oy)+'px';});
  window.addEventListener('mouseup',()=>dragging=false);
})();

function toggleGrid(){
  grid.visible = !grid.visible;
  document.getElementById('btn-grid').classList.toggle('on', grid.visible);
}

var _themeIndex = 1; // Default: bg-pro
var _themes = ['dark','bg-pro','bg-white','bg-minimal','bg-win11','bg-deep','bg-vivid','bg-matrix'];
var _themeBg  = [0x070d1a, 0x1e1e1e, 0xf0f0eb, 0xf4f4f4, 0xf3f6fc, 0x000408, 0x1a0a2e, 0x000800];
var _themeGrid= [0x0e1e30, 0x2d2d30, 0xbbbbaa, 0xcccccc, 0xc8d8e8, 0x0a1020, 0x2a1040, 0x001400];
document.getElementById('btn-bg').addEventListener('click', function(){
  // Alle Theme-Klassen entfernen
  _themes.forEach(function(t){ document.body.classList.remove(t); });
  _themeIndex = (_themeIndex + 1) % _themes.length;
  var t = _themes[_themeIndex];
  if (t !== 'dark') document.body.classList.add(t);
  scene.background = new THREE.Color(_themeBg[_themeIndex]);
  setGridColor(_themeGrid[_themeIndex]);
});
document.getElementById('btn-tcp-trace').addEventListener('click', function(){
  showTCPTrace=!showTCPTrace;this.classList.toggle('on',showTCPTrace);
  document.getElementById('cfg-show-trace').checked=showTCPTrace;
  if(!showTCPTrace)tcpTraceGrp.clear();
});

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

var SING_MANIP_THRESH = 0.001;
var SING_COND_THRESH  = 500;

function isSingular(angles_deg) {
  if (Math.abs(angles_deg[4]) < 8) return true;
  var m = computeManipulability(angles_deg);
  return (m.manipulability < SING_MANIP_THRESH || m.condition > SING_COND_THRESH);
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

  // ── 3. Ellbogen (Streckstellung): cos(A3) ≈ 0 ────────────────
  // Analytisch: d(Armreichweite)/d(A3) = 630·200·cos(A3)/dist
  // Nullstelle bei A3 = ±90°  →  arm an max. oder min. Extension
  // FK_SIGNS[2] = 1, also effektiver Winkel = A3_deg
  var a3rad = angles_deg[2] * Math.PI / 180;
  if (Math.abs(Math.cos(a3rad)) < 0.17) types.push('elbow'); // ±80°..±100°

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

// Pfad 1: Warm-Start IK für große Punktmengen (> 150 Punkte)
function _ikTableWarmStart(positions, N) {
  splashProgress && splashProgress(50, N + ' Punkte — Schnell-IK wird berechnet…');
  var prevQ = _ikGetStartQ();
  for (let i = 0; i < N; i++) {
    var p = positions[i];
    _applyToolToTCPDEF(p.toolN); // aktives Tool dieses Punkts
    var res = solveIKFast(p.X, p.Y, p.Z, p.A, p.B, p.C, prevQ);
    var angles = (res.ok ? res.angles : prevQ)
      .map(function(v, j) { return prevQ[j] + shortestAngleDiff(prevQ[j], v); });
    ikTable.push({ angles: angles, score: res.score, ok: res.ok });
    prevQ = angles.slice();
  }
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

function computeIKTable(positions) {
  ikTable = [];
  var N = positions.length;
  if (!N) { buildTrajectory(positions, ikTable); return; }

  if (N > 150 || !_toolsAreUniform(positions)) {
    _ikTableWarmStart(positions, N);
    buildTrajectory(positions, ikTable);
    return;
  }

  _applyToolToTCPDEF(positions[0].toolN || 1);
  try {
    _ikTableDPSolver(positions, N);
  } catch(e) {
    console.warn('DPSolver Fehler, Fallback:', e.message);
    _ikTableFallbackDP(positions, N);
  }

  buildTrajectory(positions, ikTable);
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
      var res = solveIKFast(p.X, p.Y, p.Z, p.A, p.B, p.C, prevQ);
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

function updateCamera(){
  if(activeCam===perspCam){
    const{theta,phi,radius}=orbitState;
    perspCam.position.set(orbitTarget.x+radius*Math.sin(phi)*Math.cos(theta),orbitTarget.y+radius*Math.sin(phi)*Math.sin(theta),orbitTarget.z+radius*Math.cos(phi));
    perspCam.lookAt(orbitTarget);
  }else{
    const vp=canvas.parentElement;const asp=vp.clientWidth/vp.clientHeight;
    orthoCam.left=-orthoHalfSize*asp;orthoCam.right=orthoHalfSize*asp;
    orthoCam.top=orthoHalfSize;orthoCam.bottom=-orthoHalfSize;orthoCam.updateProjectionMatrix();
    const D=40000,t=orbitTarget;
    switch(currentView){
      case'top':orthoCam.position.set(t.x,t.y,t.z+D);orthoCam.up.set(0,1,0);break;
      case'bottom':orthoCam.position.set(t.x,t.y,t.z-D);orthoCam.up.set(0,-1,0);break;
      case'front':orthoCam.position.set(t.x,t.y-D,t.z);orthoCam.up.set(0,0,1);break;
      case'back':orthoCam.position.set(t.x,t.y+D,t.z);orthoCam.up.set(0,0,1);break;
      case'left':orthoCam.position.set(t.x-D,t.y,t.z);orthoCam.up.set(0,0,1);break;
      case'right':orthoCam.position.set(t.x+D,t.y,t.z);orthoCam.up.set(0,0,1);break;
    }
    orthoCam.lookAt(t);
  }
  requestRender();
}
updateCamera();

function setView(view){
  currentView=view;
  document.querySelectorAll('.vbtn').forEach(b=>b.classList.remove('on'));
  (function(){var _e=document.getElementById('vb-'+view);if(_e)_e.classList.add('on');})();
  activeCam=(view==='iso')?perspCam:orthoCam;
  // sync projection button label
  const btn=document.getElementById('btn-persp');
  if(btn){
    if(activeCam===perspCam){btn.textContent='⊡ Perspektiv';btn.classList.add('on');}
    else{btn.textContent='⊞ Ortho';btn.classList.remove('on');}
  }
  updateCamera();resize();
}

// Nächstgelegene Standard-Ansicht zur aktuellen Kamera
function _nearestView(){
  const cam = (typeof activeCam!=='undefined'&&activeCam)?activeCam:perspCam;
  const v = cam.position.clone().sub(orbitTarget);
  if (v.lengthSq() < 1e-6) return 'top';
  v.normalize();
  const cand=[['top',[0,0,1]],['bottom',[0,0,-1]],['front',[0,-1,0]],['back',[0,1,0]],['left',[-1,0,0]],['right',[1,0,0]]];
  let best='top',bd=-Infinity;
  for(const [name,d] of cand){const dot=v.x*d[0]+v.y*d[1]+v.z*d[2]; if(dot>bd){bd=dot;best=name;}}
  return best;
}

function resize(){
  const vp=canvas.parentElement;const w=vp.clientWidth,h=vp.clientHeight;
  renderer.setSize(w,h);perspCam.aspect=w/h;perspCam.updateProjectionMatrix();
  const asp=w/h;orthoCam.left=-orthoHalfSize*asp;orthoCam.right=orthoHalfSize*asp;
  orthoCam.top=orthoHalfSize;orthoCam.bottom=-orthoHalfSize;orthoCam.updateProjectionMatrix();
  requestRender();
}
window.addEventListener('resize', function(){ resize(); _aPlotFitWidth(); });
resize();

// ═══════════════════════════════════════════════════
// MOUSE / ORBIT / DRAG
// ═══════════════════════════════════════════════════
const raycaster=new THREE.Raycaster();
let drag=null,dragPos=null,dragMoved=false;

function getNDC(e){const r=canvas.getBoundingClientRect();return new THREE.Vector2((e.clientX-r.left)/r.width*2-1,-((e.clientY-r.top)/r.height)*2+1);}
function getWorldOnZPlane(e,z){const ndc=getNDC(e);raycaster.setFromCamera(ndc,activeCam);const plane=new THREE.Plane(new THREE.Vector3(0,0,1),-z);const hit=new THREE.Vector3();raycaster.ray.intersectPlane(plane,hit);return hit;}

function raycastPositions(e){
  const ndc=getNDC(e);raycaster.setFromCamera(ndc,activeCam);
  const hits=raycaster.intersectObjects(posGrp.children,true);
  if(!hits.length)return null;
  let grp=hits[0].object;
  while(grp.parent&&grp.parent!==posGrp)grp=grp.parent;
  return grp.userData.posIdx!==undefined?grp.userData.posIdx:null;
}

canvas.addEventListener('mousedown',e=>{
  if(e.button!==0&&e.button!==1&&e.button!==2)return;
  dragMoved=false;e.preventDefault();
  if(e.button===0){
    const hit=raycastPositions(e);
    if(hit!==null&&!sim.playing){
      selectPosition(hit);
      const pos=parsedData.positions[hit];
      const sw=getWorldOnZPlane(e,pos.Z);
      dragPos={idx:hit,origPos:{...pos},startWorld:sw,mode:currentDragMode};
      canvas.classList.add('dragging');return;
    }
    drag={btn:activeCam===perspCam?2:1,lx:e.clientX,ly:e.clientY};
  }else if(e.button===1){drag={btn:1,lx:e.clientX,ly:e.clientY};}
  else{drag={btn:2,lx:e.clientX,ly:e.clientY};}
  canvas.classList.add('dragging');
});

window.addEventListener('mousemove',e=>{
  if(dragPos){
    dragMoved=true;
    const pos=dragPos.origPos;
    if(dragPos.mode==='tz'){
      const ndc=getNDC(e);raycaster.setFromCamera(ndc,activeCam);
      const cRight=new THREE.Vector3().crossVectors(activeCam.getWorldDirection(new THREE.Vector3()),activeCam.up).normalize();
      const plane=new THREE.Plane().setFromNormalAndCoplanarPoint(cRight,dragPos.startWorld);
      const hit=new THREE.Vector3();raycaster.ray.intersectPlane(plane,hit);
      if(hit){const dz=hit.z-dragPos.startWorld.z;applyDraggedPos(dragPos.idx,{...parsedData.positions[dragPos.idx],X:pos.X,Y:pos.Y,Z:pos.Z+dz},false);}
    }else{
      const hit=getWorldOnZPlane(e,pos.Z);
      if(hit){const dx=hit.x-dragPos.startWorld.x,dy=hit.y-dragPos.startWorld.y;applyDraggedPos(dragPos.idx,{...parsedData.positions[dragPos.idx],X:pos.X+dx,Y:pos.Y+dy,Z:pos.Z},false);}
    }
    return;
  }
  if(!drag)return;
  const dx=e.clientX-drag.lx,dy=e.clientY-drag.ly;
  drag.lx=e.clientX;drag.ly=e.clientY;
  if(Math.abs(dx)+Math.abs(dy)>2)dragMoved=true;
  if(drag.btn===2&&activeCam===perspCam){orbitState.theta-=dx*.007;orbitState.phi=Math.max(.04,Math.min(Math.PI-.04,orbitState.phi+dy*.007));updateCamera();}
  else if(drag.btn===1||drag.btn===2){
    if(activeCam===perspCam){const sc=orbitState.radius*.0008;const right=new THREE.Vector3().crossVectors(perspCam.getWorldDirection(new THREE.Vector3()),perspCam.up).normalize();orbitTarget.addScaledVector(right,-dx*sc);orbitTarget.z+=dy*sc;}
    else{const vp=canvas.parentElement;const sc=orthoHalfSize*2/vp.clientHeight;const right=new THREE.Vector3().crossVectors(orthoCam.getWorldDirection(new THREE.Vector3()),orthoCam.up).normalize();orbitTarget.addScaledVector(right,-dx*sc);orbitTarget.addScaledVector(orthoCam.up,dy*sc);}
    updateCamera();
  }
});

window.addEventListener('mouseup',e=>{
  if(dragPos){if(dragMoved)syncPositionToCode(dragPos.idx);dragPos=null;canvas.classList.remove('dragging');return;}
  if(drag){drag=null;canvas.classList.remove('dragging');if(!dragMoved&&e.button===0){const hit=raycastPositions(e);if(hit===null)deselectPosition();}else if(!dragMoved&&e.button===1){setView(_nearestView());}}
});
canvas.addEventListener('contextmenu',e=>e.preventDefault());
canvas.addEventListener('wheel',e=>{
  if(activeCam===perspCam)orbitState.radius=Math.max(80,Math.min(40000,orbitState.radius*(1+e.deltaY*.001)));
  else orthoHalfSize=Math.max(50,Math.min(20000,orthoHalfSize*(1+e.deltaY*.001)));
  updateCamera();
},{passive:true});

// ═══════════════════════════════════════════════════
// POSITION SELECTION & EDITING
// ═══════════════════════════════════════════════════
let selectedPosIdx=null,currentDragMode='translate';

function setDragMode(mode){currentDragMode=mode;document.getElementById('ep-translate').classList.toggle('active',mode==='translate');document.getElementById('ep-tz').classList.toggle('active',mode==='tz');}

function selectPosition(idx){ requestRender();
  selectedPosIdx=idx;_epNavUpdateInfo();const pos=parsedData.positions[idx];
  if(typeof setActiveTool==='function')setActiveTool(pos.toolN); // TCP/Tool dieses Punkts aktiv
  // Wenn Formular aktiv: zugehörige Karte öffnen + scrollen
  if (typeof FormatRegistry !== 'undefined' && FormatRegistry.getActiveId() === 'kuka-form') {
    if (typeof fvBuild === 'function' && pos.lineNum !== undefined) {
      fvExpandedLine = pos.lineNum;
      fvBuild(pos.lineNum);
    }
  }
  selSphere.position.set(pos.X,pos.Y,pos.Z);selSphere.visible=true;
  document.getElementById('ep-title').textContent=`#${idx+1}  ${pos.type}  (Z.${pos.lineNum+1})`;
  const dDisp=pos.rel || pos;
  ['x','y','z','a','b','c'].forEach(k=>document.getElementById('ep-'+k).value=dDisp[k.toUpperCase()].toFixed(3));
  document.getElementById('edit-panel').style.display='block';
  document.querySelectorAll('.pc').forEach((el,i)=>el.classList.toggle('selected',i===idx));
  // Hochpräzisions-IK für exakte Zielposition
  if(ikTable[idx]&&ikTable[idx].ok){
    var initAngles = ikTable[idx].angles.slice();
    var precise = solveIKPrecise(pos.X, pos.Y, pos.Z, pos.A, pos.B, pos.C, initAngles);
    if (precise.ok) {
      ikTable[idx].angles = precise.angles;
    }
    tweenToAngles(ikTable[idx].angles, 500);
  }
  // Show all IK variants for this position
  showEpIKSolutions(pos.X, pos.Y, pos.Z, pos.A, pos.B, pos.C);

  // FK-Δ berechnen und anzeigen (Abweichung IK→Ziel)
  if (ikTable[idx] && ikTable[idx].ok) {
    var fkChk = fkAll(ikTable[idx].angles);
    var tcpChk = fkChk.pts[7];
    if (tcpChk) {
      var dx = tcpChk[0]-pos.X, dy = tcpChk[1]-pos.Y, dz = tcpChk[2]-pos.Z;
      var err = Math.sqrt(dx*dx+dy*dy+dz*dz);
      document.getElementById('rb-fk').textContent = err.toFixed(2);
    }
  }
}

function epNavFirst(){ if(parsedData.positions.length) selectPosition(0); }
function epNavLast() { const N=parsedData.positions.length; if(N) selectPosition(N-1); }
function epNavPrev() { if(selectedPosIdx===null||!parsedData.positions.length) return; selectPosition(Math.max(0,selectedPosIdx-1)); }
function epNavNext() { if(selectedPosIdx===null||!parsedData.positions.length) return; selectPosition(Math.min(parsedData.positions.length-1,selectedPosIdx+1)); }
function _epNavUpdateInfo() {
  const el=document.getElementById('ep-nav-info'); if(!el) return;
  const N=parsedData.positions.length;
  if(selectedPosIdx===null||!N){ el.textContent='—'; return; }
  el.textContent=(selectedPosIdx+1)+' / '+N;
}

function deselectPosition(){ requestRender();
  selectedPosIdx=null;_epNavUpdateInfo();selSphere.visible=false;
  document.getElementById('edit-panel').style.display='none';
  document.querySelectorAll('.pc').forEach(el=>el.classList.remove('selected'));
}

document.getElementById('ep-close').onclick=deselectPosition;

function applyEditPanel(){
  if(selectedPosIdx===null)return;
  const pos=parsedData.positions[selectedPosIdx];
  const disp={X:parseFloat(document.getElementById('ep-x').value)||0,Y:parseFloat(document.getElementById('ep-y').value)||0,Z:parseFloat(document.getElementById('ep-z').value)||0,A:parseFloat(document.getElementById('ep-a').value)||0,B:parseFloat(document.getElementById('ep-b').value)||0,C:parseFloat(document.getElementById('ep-c').value)||0};
  const w=baseToWorld(disp, (pos.baseN!=null?pos.baseN:_activeBaseN())); // Eingabe ist Code-Koordinate -> Welt (+ BASE-Offset des Punkts)
  const newPos={...pos,X:w.X,Y:w.Y,Z:w.Z,A:w.A,B:w.B,C:w.C,rel:{X:disp.X,Y:disp.Y,Z:disp.Z,A:disp.A,B:disp.B,C:disp.C}};
  applyDraggedPos(selectedPosIdx,newPos,true);
}

['ep-x','ep-y','ep-z','ep-a','ep-b','ep-c'].forEach(id=>document.getElementById(id).addEventListener('keydown',e=>{if(e.key==='Enter')applyEditPanel();}));

function applyDraggedPos(idx,newPos,syncCode){ requestRender();
  var _bn = (newPos.baseN!=null) ? newPos.baseN : ((parsedData.positions[idx] && parsedData.positions[idx].baseN!=null) ? parsedData.positions[idx].baseN : _activeBaseN());
  newPos.baseN = _bn;
  const dD = newPos.rel || worldToBase(newPos, _bn); // Code-Koordinate (rel)
  newPos.rel={X:dD.X,Y:dD.Y,Z:dD.Z,A:dD.A,B:dD.B,C:dD.C};
  parsedData.positions[idx]=newPos;
  const grp=posGrp.children[idx];
  if(grp){grp.position.set(newPos.X,newPos.Y,newPos.Z);grp.setRotationFromEuler(kukaEuler(newPos.A,newPos.B,newPos.C));}
  if(idx===selectedPosIdx){selSphere.position.set(newPos.X,newPos.Y,newPos.Z);['x','y','z','a','b','c'].forEach(k=>document.getElementById('ep-'+k).value=dD[k.toUpperCase()].toFixed(3));}
  const card=document.getElementById('pcard-'+idx);
  if(card)card.querySelectorAll('.pf').forEach((el,i)=>{const vals=[dD.X,dD.Y,dD.Z,dD.A,dD.B,dD.C];const labels=['X','Y','Z','A(Z)','B(Y)','C(X)'];const units=['mm','mm','mm','°','°','°'];if(i<6)el.innerHTML=`<span>${labels[i]}</span> ${vals[i].toFixed(2)} ${units[i]}`;});
  updateVisitedPath(sim.t);
  if(syncCode){
    syncPositionToCode(idx);
    // Trajektorie nach Apply neu aufbauen
    buildScene(parsedData.positions);
    computeIKTable(parsedData.positions);
    return;
  }
  // Beim Dragging: nur IK für diesen Punkt
  const res=solveIK(newPos.X,newPos.Y,newPos.Z,newPos.A,newPos.B,newPos.C);
  ikTable[idx]=res;
  updateIKBadge(idx,res);
}

function syncPositionToCode(idx){
  const pos=parsedData.positions[idx];if(pos.lineNum===undefined)return;
  const pr=pos.rel || pos; // Code-Koordinate (unveraendert) in den Text schreiben
  const ta=document.getElementById('code-input');const lines=ta.value.split(/\r?\n/);const oldLine=lines[pos.lineNum];
  const str=`{X ${pr.X.toFixed(3)},Y ${pr.Y.toFixed(3)},Z ${pr.Z.toFixed(3)},A ${pr.A.toFixed(3)},B ${pr.B.toFixed(3)},C ${pr.C.toFixed(3)}${pos.S!==null&&pos.S!==undefined?',S '+pos.S:''}${pos.T!==null&&pos.T!==undefined?',T '+pos.T:''}}`;
  lines[pos.lineNum]=oldLine.replace(/\{[^}]+\}/,str);ta.value=lines.join('\n');
  rebuildGutter();
  // Formular-Ansicht aktualisieren
  if(FormatRegistry.getActiveId()==='kuka-form'&&typeof fvBuild==='function') fvBuild(-1);
}

document.getElementById('code-input').addEventListener('input',()=>rebuildGutter());

// ═══════════════════════════════════════════════════
// APPLY ANGLES — update robot model + status bar
// ═══════════════════════════════════════════════════
function applyAngles(angles) { requestRender();
  for (let i=0;i<6;i++) jointAngles[i] = angles[i];
  buildRobotModel(angles);
  // status bar
  for (let i=0;i<6;i++) {
    const el = document.getElementById('rb-a'+(i+1));
    if (el) el.textContent = angles[i].toFixed(1);
  }
  // FK
  const fk = fkAll(angles);
  const tcp = fk.pts[7];
  document.getElementById('rb-fk').textContent = '—';
  addTCPTracePoint(tcp);
  // Update Steuerungspanel live
  updateSteuerPanel();
}

// ═══════════════════════════════════════════════════
// TOOL STL
// ═══════════════════════════════════════════════════
let toolMesh=null;
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

function loadToolFile(){document.getElementById('tool-file').click();}
document.getElementById('tool-file').addEventListener('change',function(){if(this.files[0])loadToolSTL(this.files[0]);this.value='';});

function loadToolSTL(file){
  const reader=new FileReader();
  reader.onload=e=>{
    if(toolMesh){markerGrp.remove(toolMesh);toolMesh.geometry.dispose();toolMesh.material.dispose();toolMesh=null;}
    const geo=stlLoader.parse(e.target.result);geo.computeVertexNormals();
    const mat=new THREE.MeshPhongMaterial({color:0xdd9944,transparent:false,opacity:1.0,side:THREE.DoubleSide,specular:0x666666});
    toolMesh=new THREE.Mesh(geo,mat);scene.add(toolMesh);
    buildRobotModel(jointAngles);
    document.getElementById('tool-controls').style.display='block';
    document.getElementById('tool-filename').textContent=file.name.replace(/\.stl$/i,'');
  };
  reader.readAsArrayBuffer(file);
}

function removeToolSTL(){
  if(toolMesh){markerGrp.remove(toolMesh);toolMesh.geometry.dispose();toolMesh.material.dispose();toolMesh=null;}
  document.getElementById('tool-controls').style.display='none';
}

// ═══════════════════════════════════════════════════
// STL SCENE MODELS
// ═══════════════════════════════════════════════════
const stlObjects=[];
document.getElementById('stl-file').addEventListener('change',function(){for(const file of this.files)loadSTL(file);this.value='';});

function loadSTL(file){
  const reader=new FileReader();
  reader.onload=e=>{
    const geo=stlLoader.parse(e.target.result);geo.computeVertexNormals();
    geo.computeBoundingBox();const ctr=new THREE.Vector3();geo.boundingBox.getCenter(ctr);geo.translate(-ctr.x,-ctr.y,-ctr.z);
    const mat=new THREE.MeshPhongMaterial({color:0x7799bb,transparent:true,opacity:.55,side:THREE.DoubleSide,specular:0x444444});
    const mesh=new THREE.Mesh(geo,mat);stlGrp.add(mesh);
    const idx=stlObjects.length;stlObjects.push({mesh,name:file.name,buf:e.target.result});renderSTLEntry(idx);
  };
  reader.readAsArrayBuffer(file);
}

let stlNavIdx = 0;

function stlLoadForCurrent() {
  var inp = document.getElementById('stl-replace-inp');
  if (!inp) {
    inp = document.createElement('input');
    inp.type = 'file'; inp.accept = '.stl'; inp.id = 'stl-replace-inp';
    inp.style.display = 'none';
    document.body.appendChild(inp);
  }
  inp.onchange = function(e) {
    var file = e.target.files[0]; if (!file) return;
    var reader = new FileReader();
    reader.onload = function(ev) {
      var buf = ev.target.result;
      var idx = stlNavIdx;
      parseGeometry(buf, file.name).then(function(geo) {
      var obj = stlObjects[idx];
      if (obj && obj.mesh) { stlGrp.remove(obj.mesh); obj.mesh.geometry.dispose(); obj.mesh.material.dispose(); }
      var mat = new THREE.MeshPhongMaterial({color:0x4499cc,shininess:60,side:THREE.DoubleSide});
      var mesh = new THREE.Mesh(geo, mat);
      stlGrp.add(mesh);
      if (obj) {
        obj.mesh = mesh; obj.name = file.name; obj.buf = buf;
      } else {
        stlObjects[idx] = { mesh: mesh, name: file.name, buf: buf, offset: {x:0,y:0,z:0,a:0,b:0,c:0}, displayName: '' };
      }
      updateSTL(idx);
      renderStlNav();
      }).catch(function(err){ alert('Fehler: '+err.message); });
    };
    reader.readAsArrayBuffer(file);
    inp.value = '';
  };
  inp.click();
}

function stlAddEntry() {
  stlObjects.push({ mesh: null, name: '', buf: null, offset: {x:0,y:0,z:0,a:0,b:0,c:0}, displayName: '' });
  stlNavIdx = stlObjects.length - 1;
  renderStlNav();
}

function renderSTLEntry(idx) {
  stlNavIdx = idx;
  renderStlNav();
}

function renderStlNav() {
  var navEl = document.getElementById('stl-nav-ui');
  var fldEl = document.getElementById('stl-fields');
  if (!navEl || !fldEl) return;
  var objs = stlObjects.filter(Boolean);
  var realIdx = stlNavIdx;
  // find valid idx
  var validObjs = stlObjects.map((o,i) => o ? i : -1).filter(i => i >= 0);
  if (!validObjs.length) {
    navEl.innerHTML = '';
    fldEl.innerHTML = '<div class="empty">Keine STL Szenenmodelle geladen</div>';
    return;
  }
  if (!stlObjects[realIdx]) realIdx = validObjs[0];
  stlNavIdx = realIdx;
  var pos = validObjs.indexOf(realIdx);
  var n = validObjs.length;
  var obj = stlObjects[realIdx];
  var o = obj.offset || {x:0,y:0,z:0,a:0,b:0,c:0};

  var btnS = 'min-width:32px;height:28px;font-size:14px;cursor:pointer;background:rgba(255,255,255,.06);border:1px solid rgba(255,255,255,.15);color:#9ab;border-radius:4px;padding:0 4px';
  var btnD = btnS + ';opacity:.3;cursor:default';
  var nav = function(lbl, cb, dis) {
    var b = document.createElement('button');
    b.textContent = lbl; b.style.cssText = dis ? btnD : btnS;
    if (!dis) b.onclick = cb; else b.disabled = true; return b;
  };

  navEl.innerHTML = '';
  var row = document.createElement('div');
  row.style.cssText = 'display:flex;align-items:center;gap:4px;margin-bottom:8px';
  row.appendChild(nav('⏮', function(){ stlNavIdx=validObjs[0]; renderStlNav(); }, pos===0));
  row.appendChild(nav('◀', function(){ stlNavIdx=validObjs[Math.max(0,pos-1)]; renderStlNav(); }, pos===0));
  var lbl = document.createElement('span');
  lbl.style.cssText = 'flex:1;text-align:center;font-family:monospace;font-size:12px;color:var(--txt2)';
  lbl.textContent = (pos+1)+' / '+n+(obj.displayName?' · '+obj.displayName:'');
  row.appendChild(lbl);
  row.appendChild(nav('▶', function(){ stlNavIdx=validObjs[Math.min(validObjs.length-1,pos+1)]; renderStlNav(); }, pos===n-1));
  row.appendChild(nav('⏭', function(){ stlNavIdx=validObjs[validObjs.length-1]; renderStlNav(); }, pos===n-1));
  var addBtn = document.createElement('button');
  addBtn.textContent='+'; addBtn.style.cssText='min-width:32px;height:28px;font-size:14px;cursor:pointer;background:rgba(37,99,235,.2);border:1px solid rgba(37,99,235,.5);color:#60a5fa;border-radius:4px;padding:0 4px';
  addBtn.onclick = function(){ stlAddEntry(); };
  row.appendChild(addBtn);
  var delBtn = document.createElement('button');
  delBtn.textContent='✕'; delBtn.style.cssText='min-width:32px;height:28px;font-size:14px;cursor:pointer;background:rgba(204,51,51,.15);border:1px solid rgba(204,51,51,.3);color:#f87171;border-radius:4px;padding:0 4px';
  delBtn.onclick = function(){ removeSTL(realIdx); };
  row.appendChild(delBtn);
  navEl.appendChild(row);

  fldEl.innerHTML = `
    <input id="stl-nav-name" type="text" placeholder="Name" value="${obj.displayName||''}" style="width:100%;background:var(--bg1);border:1px solid var(--bdr);border-radius:3px;padding:3px 6px;font-family:monospace;font-size:.82em;color:var(--txt);margin-bottom:6px;box-sizing:border-box">
    <div style="display:flex;align-items:center;gap:6px;margin-bottom:6px">
      <span style="flex:1;font-size:.75em;color:var(--txt3);font-family:monospace;overflow:hidden;text-overflow:ellipsis;white-space:nowrap">${obj.name||'— kein STL —'}</span>
      <button onclick="stlLoadForCurrent()" style="height:28px;padding:0 10px;font-size:11px;cursor:pointer;background:rgba(255,255,255,.06);border:1px solid rgba(255,255,255,.15);color:#9ab;border-radius:4px;white-space:nowrap">STL laden</button>
    </div>
    <div class="stl-grid" style="margin-bottom:4px">
      <span class="stl-lbl">X</span><input class="stl-inp" id="stl-f-x" type="number" value="${o.x||0}" step="1"><span class="stl-unit">mm</span>
      <span class="stl-lbl">Y</span><input class="stl-inp" id="stl-f-y" type="number" value="${o.y||0}" step="1"><span class="stl-unit">mm</span>
      <span class="stl-lbl">Z</span><input class="stl-inp" id="stl-f-z" type="number" value="${o.z||0}" step="1"><span class="stl-unit">mm</span>
      <span class="stl-lbl">A(Z)</span><input class="stl-inp" id="stl-f-a" type="number" value="${o.a||0}" step="1"><span class="stl-unit">°</span>
      <span class="stl-lbl">B(Y)</span><input class="stl-inp" id="stl-f-b" type="number" value="${o.b||0}" step="1"><span class="stl-unit">°</span>
      <span class="stl-lbl">C(X)</span><input class="stl-inp" id="stl-f-c" type="number" value="${o.c||0}" step="1"><span class="stl-unit">°</span>
    </div>`;

  document.getElementById('stl-nav-name').oninput = function() {
    stlObjects[realIdx].displayName = this.value;
    document.querySelector('#stl-nav-ui span') && (document.querySelector('#stl-nav-ui span').textContent = (pos+1)+'/'+n+(this.value?' · '+this.value:''));
  };
  for (var ax of ['x','y','z','a','b','c']) {
    (function(a) {
      document.getElementById('stl-f-'+a).addEventListener('input', function() {
        if (!stlObjects[realIdx].offset) stlObjects[realIdx].offset = {};
        stlObjects[realIdx].offset[a] = parseFloat(this.value)||0;
        updateSTL(realIdx);
      });
    })(ax);
  }
}

function updateSTL(idx){var obj=stlObjects[idx];if(!obj||!obj.mesh)return;var o=obj.offset||{};obj.mesh.position.set(o.x||0,o.y||0,o.z||0);obj.mesh.setRotationFromEuler(kukaEuler(o.a||0,o.b||0,o.c||0));}
function removeSTL(idx){var obj=stlObjects[idx];if(!obj)return;stlGrp.remove(obj.mesh);obj.mesh.geometry.dispose();obj.mesh.material.dispose();stlObjects[idx]=null;renderStlNav();}

// Auto-Load STL Dateien vom Server (stl/ Ordner)
function autoLoadSTLFiles() {
  var axisFiles = ['a1.stl','a2.stl','a3.stl','a4.stl','a5.stl','a6.stl'];
  var base = 'stl/';

  var loadedCount = 0;
  var totalAxis = axisFiles.length;

  axisFiles.forEach(function(name, idx) {
    fetch(base + name)
      .then(function(r) { if(!r.ok) throw new Error('not found'); return r.arrayBuffer(); })
      .then(function(buf) {
        var geo = stlLoader.parse(buf);
        geo.computeVertexNormals();
        if (axisSTLMeshes[idx]) { scene.remove(axisSTLMeshes[idx]); axisSTLMeshes[idx].geometry.dispose(); }
        axisSTLMeshes[idx] = new THREE.Mesh(geo, new THREE.MeshPhongMaterial({color:0xe8a020,shininess:80}));
        scene.add(axisSTLMeshes[idx]);
        axisSTLBase64[idx] = btoa(String.fromCharCode.apply(null, new Uint8Array(buf)));
        if (!window._axisSTLBuffers) window._axisSTLBuffers = {};
        window._axisSTLBuffers[idx] = buf;
        var nameEl = document.getElementById('asl-name'+idx);
        var delEl  = document.getElementById('asl-del'+idx);
        if (nameEl) nameEl.textContent = name.replace(/\.stl$/i,'');
        if (delEl)  delEl.style.display = '';
        // Nach letzter Achse: buildRobotModel aufrufen
        loadedCount++;
        if (loadedCount === totalAxis) {
          buildRobotModel(jointAngles);
        }
      })
      .catch(function() {
        loadedCount++;
        if (loadedCount === totalAxis) {
          buildRobotModel(jointAngles);
        }
      });
  });

  // Podest
  fetch(base + 'podest.stl')
    .then(function(r) { if(!r.ok) throw new Error(); return r.arrayBuffer(); })
    .then(function(buf) {
      window._pedestalSTLBuffer = buf;
      sceneSTLOffsets.pedestal.name = 'podest';
      var geo = stlLoader.parse(buf); geo.computeVertexNormals();
      if (pedestalMesh) { scene.remove(pedestalMesh); pedestalMesh.geometry.dispose(); }
      pedestalMesh = new THREE.Mesh(geo, new THREE.MeshPhongMaterial({color:0x446688,shininess:60}));
      scene.add(pedestalMesh);
      var nb = document.getElementById('pedestal-name');
      if (nb) nb.textContent = 'podest';
    })
    .catch(function() {});

  // Werkzeug
  fetch(base + 'tool1_tcp.stl')
    .then(function(r) { if(!r.ok) throw new Error(); return r.arrayBuffer(); })
    .then(function(buf) {
      window._toolSTLBuffer = buf;
      sceneSTLOffsets.tool.name = 'tool1_tcp';
      var geo = stlLoader.parse(buf); geo.computeVertexNormals();
      if (toolMesh) { scene.remove(toolMesh); toolMesh.geometry.dispose(); }
      toolMesh = new THREE.Mesh(geo, new THREE.MeshPhongMaterial({color:0x334455,shininess:40,transparent:true,opacity:0.8}));
      scene.add(toolMesh);
      var nt = document.getElementById('tool-name');
      if (nt) nt.textContent = 'tool1_tcp';
    })
    .catch(function() {});
}

// ═══════════════════════════════════════════════════
// SIMULATION ENGINE
// ═══════════════════════════════════════════════════
const sim={t:0,playing:false,dir:1,stepTarget:null,stepIdx:0};
const breakpoints=new Set();

function simSpeed(){return(parseInt(document.getElementById('spd-s').value)/100)*3.0;}
function lerpPos(a,b,f){return{X:a.X+(b.X-a.X)*f,Y:a.Y+(b.Y-a.Y)*f,Z:a.Z+(b.Z-a.Z)*f,A:a.A+(b.A-a.A)*f,B:a.B+(b.B-a.B)*f,C:a.C+(b.C-a.C)*f};}
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
let trajectoryRef = [];  // Referenz-Trajektorie für Map (unveränderlich)
let trajMax = 0;       // trajectory.length - 1

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
  const ab=v3sub(b,a),am=v3sub(m,a),mb=v3sub(b,m);
  const n=v3norm(v3cross(ab,am));
  // Circumcenter in the plane
  const ac=v3sub(b,a),ad=v3sub(m,a);
  const d=2*(ac[0]*(ad[1]*n[2]-ad[2]*n[1])-ac[1]*(ad[0]*n[2]-ad[2]*n[0])+ac[2]*(ad[0]*n[1]-ad[1]*n[0]));
  if(Math.abs(d)<1e-9) return null;
  const s=((v3dot(ac,ac)*(ad[1]*n[2]-ad[2]*n[1])-v3dot(ad,ad)*(ac[1]*n[2]-ac[2]*n[1]))/d);
  // Center via parametric circumcenter formula
  const cx=a[0]+0.5*(b[0]-a[0]),cy=a[1]+0.5*(b[1]-a[1]),cz=a[2]+0.5*(b[2]-a[2]);
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
  const v=([a,b,c])=>[a,b,c];
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
    const res = solveIK(pos.X, pos.Y, pos.Z, pos.A, pos.B, pos.C, prevAngles);
    return res.ok ? res.angles : prevAngles;
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
      // Für ersten Schritt: linear interpolieren als sicherer Warm-Start
      let warmAng = lerpAngles(angPrev, angCurrLin, 1/steps);
      // Erster Schritt immer linear (kein IK-Sprung am Segment-Anfang)
      pushSample(lerpPos(prev, curr, 1/steps), warmAng, i);
      for (let s = 2; s <= steps; s++) {
        const f = s / steps;
        const pos = lerpPos(prev, curr, f);
        if (dist < 20) {
          warmAng = lerpAngles(angPrev, angCurrLin, f);
        } else {
          const res = solveIK(pos.X, pos.Y, pos.Z, pos.A, pos.B, pos.C, warmAng);
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
  // Referenz-Trajektorie für Map (tief kopieren)
  trajectoryRef = trajectory.map(function(t) {
    return { pos: t.pos, angles: t.angles ? t.angles.slice() : [] };
  });

  // Rebuild path line from actual Cartesian trajectory
  pathGrp.clear();
  // Full programmed path from parsedData.positions (always correct)
  if (parsedData.positions.length > 1) {
    const progPts = parsedData.positions.map(p => new THREE.Vector3(p.X, p.Y, p.Z));
    pathGrp.add(new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(progPts),
      new THREE.LineBasicMaterial({color:hexToInt((document.getElementById('cfg-planned-col')||{value:'#1a3050'}).value)})
    ));
  }
  // Scan wird nur durch 'MAP erzeugen' gestartet
  // Scan wird nur durch 'MAP erzeugen' Button gestartet
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

// Legacy: map t (0..N-1) to trajectory fraction
function simTToTrajT(t) {
  const N = parsedData.positions.length;
  return N > 1 ? t / (N-1) : 0;
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
function updateVisitedPath(t){
  if(visitedLine){pathGrp.remove(visitedLine);visitedLine=null;}
  if(!trajectory.length||t<=0)return;
  const pathCol=hexToInt(document.getElementById('cfg-path-col').value);
  const tf=simTToTrajT(t);
  const cutIdx=Math.floor(tf*trajMax);
  if(cutIdx<1)return;
  const pts=trajectory.slice(0,cutIdx+1).map(s=>new THREE.Vector3(s.pos.X,s.pos.Y,s.pos.Z));
  if(pts.length>1){visitedLine=new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts),new THREE.LineBasicMaterial({color:pathCol,linewidth:pathLineWidth}));pathGrp.add(visitedLine);}
}

function setStatus(cls,txt){const el=document.getElementById('sim-status');el.className='sstatus '+cls;el.textContent=txt;}

function checkBP(prevT,newT,dir){
  const pos=parsedData.positions;
  if(dir>0){const fr=Math.floor(prevT)+(Number.isInteger(prevT)?1:0);for(let i=fr;i<=Math.floor(newT);i++)if(i<pos.length&&breakpoints.has(pos[i].lineNum))return i;}
  else{const fr=Math.ceil(prevT)-(Number.isInteger(prevT)?1:0);for(let i=fr;i>=Math.ceil(newT);i--)if(i>=0&&breakpoints.has(pos[i].lineNum))return i;}
  return null;
}

function applySimT(t){ _tween = null;
  const pos=parsedData.positions,N=pos.length;if(!N)return;
  sim.t=Math.max(0,Math.min(N-1,t));
  document.getElementById('pos-s').value=sim.t;
  const idx=Math.min(Math.floor(sim.t),N-1);
  if(typeof setActiveTool==='function')setActiveTool(pos[idx]&&pos[idx].toolN); // aktives TCP/Tool dieses Satzes
  // Show trajectory sample count
  const trajSamples = trajectory.length;
  document.getElementById('pos-v').textContent=`${idx+1} / ${N}` + (trajSamples>N?` (${trajSamples} Schritte)`:'');
  updateVisitedPath(sim.t);updateGutterActive(pos[idx].lineNum);updatePosCards(idx);updateSignalsForStep(pos[idx]);
  if(parsedData.steps){const si=parsedData.steps.findIndex(s=>s.type==='move'&&s.posIdx===idx);if(si>=0)sim.stepIdx=si;}
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
      document.getElementById('rb-fk').textContent=err.toFixed(2);
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

function updateSignalsForStep(pos){
  if(!(pos&&pos.snapshot))return;const s=pos.snapshot;
  renderVariables(s.variables);renderDigital(s.digitalIn,'$IN','din-list');renderDigital(s.digitalOut,'$OUT','dout-list');renderAnalog(s.analogOut);
}

function applyStep(idx){
  const steps=parsedData.steps;if(!(steps&&steps.length))return;
  idx=Math.max(0,Math.min(steps.length-1,idx));sim.stepIdx=idx;const step=steps[idx];
  updateGutterActive(step.lineNum);
  if(step.snapshot){renderVariables(step.snapshot.variables);renderDigital(step.snapshot.digitalIn,'$IN','din-list');renderDigital(step.snapshot.digitalOut,'$OUT','dout-list');renderAnalog(step.snapshot.analogOut);}
  let posIdx=-1;for(let i=idx;i>=0;i--){if(steps[i].type==='move'){posIdx=steps[i].posIdx;break;}}
  const N=parsedData.positions.length;
  if(posIdx>=0){
    const pos=parsedData.positions[posIdx];if(typeof setActiveTool==='function')setActiveTool(pos.toolN);updateMarkerPose(pos);markerGrp.visible=true;updatePosCards(posIdx);updateVisitedPath(posIdx);
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


// ── Achsen-Interpolation (Tween) ──────────────────────────────
var _tween = null;  // { from, to, t, duration }

function tweenToAngles(targetAngles, durationMs) {
  durationMs = durationMs || 600;
  _tween = {
    from: jointAngles.slice(),
    to:   targetAngles.slice(),
    t:    0,
    duration: durationMs / 1000
  };
}

function updateTween(dt) {
  if (!_tween) return;
  _tween.t += dt;
  var f = Math.min(_tween.t / _tween.duration, 1);
  // Ease in-out
  f = f < 0.5 ? 2*f*f : -1 + (4 - 2*f)*f;
  var angles = _tween.from.map(function(v, i) {
    return v + (_tween.to[i] - v) * f;
  });
  applyAngles(angles);
  if (_tween.t >= _tween.duration) _tween = null;
}


let lastTs=null;
// Simulationsgeschwindigkeit für aktuellen Satz berechnen
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
  if (!_realtimeMode) {
    // Slider steuert nur, wie schnell abgespielt wird – nicht den angezeigten Wert
    return { speed: simSpeed(), velCP: curVelCP, displayMmMin: dispMmMin };
  }
  const N = pos.length;
  const speed = Math.max(0.01, segDist > 1 ? velMmS/segDist : (N > 1 ? velMmS/100 : simSpeed()));
  return { speed, velCP: curVelCP, displayMmMin: dispMmMin };
}

// Simulationszeit um dt vorrücken; rendert selbst und gibt false zurück wenn frame() nicht weiterrendern soll
function _simAdvance(N, dt, speed, velCP, displayMmMin) {
  const prevT = sim.t;
  let newT;
  if (sim.stepTarget !== null) {
    const sd = sim.stepTarget > sim.t ? 1 : -1;
    newT = sim.t + sd * speed * dt;
    if ((sd > 0 && newT >= sim.stepTarget) || (sd < 0 && newT <= sim.stepTarget)) {
      newT = sim.stepTarget; sim.stepTarget = null;
      applySimT(newT); setStatus('paused','PAUSED'); renderer.render(scene, activeCam);
      return false;
    }
  } else {
    newT = sim.t + sim.dir * speed * dt;
    const bp = checkBP(prevT, newT, sim.dir);
    if (bp !== null) { pauseSim(); applySimT(bp); setStatus('bp','● BREAKPOINT'); renderer.render(scene, activeCam); return false; }
    if      (newT >= N-1) { newT = N-1; pauseSim(); setStatus('end','END ✓'); }
    else if (newT <= 0)   { newT = 0;   pauseSim(); setStatus('paused','START'); }
    else { setStatus('playing', (sim.dir>0?'▶':'◀') + '  ' + (displayMmMin??Math.round(velCP*60000)) + ' mm/min'); }
  }
  applySimT(newT);
  return true;
}

function frame(ts) {
  requestAnimationFrame(frame);
  const dt = lastTs !== null ? Math.min((ts-lastTs)/1000, .1) : 0;
  lastTs = ts;
  updateTween(dt);
  const N = parsedData.positions.length;
  if (N > 0 && (sim.playing || sim.stepTarget !== null) && !_tween) {
    const pos = parsedData.positions;
    const i0  = Math.min(Math.floor(sim.t), pos.length-1);
    const i1  = Math.min(i0+1, pos.length-1);
    const { speed, velCP, displayMmMin } = _simCalcSpeed(pos, i0, i1);
    const velEl = document.getElementById('tcp-vel-v');
    if (velEl) velEl.textContent = displayMmMin + ' mm/min';
    _simAdvance(N, dt, speed, velCP, displayMmMin);
    _renderFrames = Math.max(_renderFrames, 1); // während Sim rendern
  }
  if (_tween) _renderFrames = Math.max(_renderFrames, 1);
  if (_renderFrames > 0) { renderer.render(scene, activeCam); _renderFrames--; }
}
requestAnimationFrame(frame);

// Buttons
document.getElementById('b-start').onclick=()=>{pauseSim();applySimT(0);if((parsedData.steps&&parsedData.steps.length))applyStep(0);else setStatus('stopped','STOPPED');};
document.getElementById('b-end').onclick=()=>{pauseSim();applySimT(Math.max(0,parsedData.positions.length-1));const last=((parsedData.steps&&parsedData.steps.length)||1)-1;if((parsedData.steps&&parsedData.steps.length))applyStep(last);else setStatus('stopped','STOPPED');};
document.getElementById('b-stop').onclick=()=>{pauseSim();setStatus('paused','PAUSED');};
document.getElementById('b-playfwd').onclick=()=>{if(sim.playing&&sim.dir===1){pauseSim();setStatus('paused','PAUSED');}else{parseAndLoad();if(!parsedData.positions.length)return;if(sim.t>=parsedData.positions.length-1)applySimT(0);sim.playing=true;sim.dir=1;sim.stepTarget=null;document.getElementById('b-playfwd').classList.add('on');document.getElementById('b-playrev').classList.remove('on');setStatus('playing','▶ FORWARD');}};
document.getElementById('b-playrev').onclick=()=>{if(sim.playing&&sim.dir===-1){pauseSim();setStatus('paused','PAUSED');}else{parseAndLoad();if(!parsedData.positions.length)return;if(sim.t<=0)applySimT(parsedData.positions.length-1);sim.playing=true;sim.dir=-1;sim.stepTarget=null;document.getElementById('b-playrev').classList.add('on');document.getElementById('b-playfwd').classList.remove('on');setStatus('playing','◀ BACKWARD');}};
document.getElementById('b-stepfwd').onclick=()=>{if(!(parsedData.steps&&parsedData.steps.length))return;pauseSim();applyStep(sim.stepIdx+1);};
document.getElementById('b-steprev').onclick=()=>{if(!(parsedData.steps&&parsedData.steps.length))return;pauseSim();applyStep(sim.stepIdx-1);};
document.getElementById('spd-s').addEventListener('input',function(){document.getElementById('spd-v').textContent=this.value+'%';});
document.getElementById('pos-s').addEventListener('mousedown',()=>{if(sim.playing){pauseSim();setStatus('paused','PAUSED');}});
document.getElementById('pos-s').addEventListener('input',function(){const posIdx=Math.round(parseFloat(this.value));applySimT(posIdx);if(parsedData.steps){const si=parsedData.steps.findIndex(s=>s.type==='move'&&s.posIdx===posIdx);if(si>=0)applyStep(si);}});

window.addEventListener('keydown',e=>{
  const inEditor=e.target===document.getElementById('code-input')||e.target.tagName==='INPUT';
  // F-keys: always active
  if(e.key==='F5'){e.preventDefault();document.getElementById('b-playfwd').click();return;}
  if(e.key==='F6'){e.preventDefault();document.getElementById('b-stop').click();return;}
  if(e.key==='F7'){e.preventDefault();document.getElementById('b-stepfwd').click();return;}
  if(e.key==='F8'){e.preventDefault();document.getElementById('b-steprev').click();return;}
  if(e.key==='F9'){e.preventDefault();document.getElementById('parse-btn').click();return;}
  if(e.key==='F11'){e.preventDefault();toggleSteuerPanel();return;}
  if(e.key==='F11'){e.preventDefault();toggleGrid();return;}
  if(inEditor)return;
  if(e.key===' '){e.preventDefault();document.getElementById('b-stop').click();}
  if(e.key==='Home'){e.preventDefault();document.getElementById('b-start').click();}
  if(e.key==='End'){e.preventDefault();document.getElementById('b-end').click();}
  if(e.key==='Escape'){deselectPosition();}
  // Touchpad-freundliche Steuerung
  if(e.key==='s'||e.key==='S'){e.preventDefault();fitAllToView();return;}
  // Pan mit Cursortasten, Rotation mit Shift+Cursor
  if(e.key.indexOf('Arrow')===0){
    e.preventDefault();
    var step = e.shiftKey ? 8 : 50;  // Shift = Rotation in Grad, normal = Pan in mm
    if(e.shiftKey){
      // Rotation: Theta/Phi anpassen
      if(e.key==='ArrowLeft')  orbitState.theta -= step*Math.PI/180;
      if(e.key==='ArrowRight') orbitState.theta += step*Math.PI/180;
      if(e.key==='ArrowUp')    orbitState.phi   = Math.max(0.1, orbitState.phi - step*Math.PI/180);
      if(e.key==='ArrowDown')  orbitState.phi   = Math.min(Math.PI-0.1, orbitState.phi + step*Math.PI/180);
    } else {
      // Pan: orbitTarget verschieben in Bildschirm-Ebene
      var camDir = new THREE.Vector3();
      activeCam.getWorldDirection(camDir);
      var right = new THREE.Vector3().crossVectors(camDir, activeCam.up).normalize();
      var up    = new THREE.Vector3().crossVectors(right, camDir).normalize();
      if(e.key==='ArrowLeft')  orbitTarget.addScaledVector(right, -step);
      if(e.key==='ArrowRight') orbitTarget.addScaledVector(right,  step);
      if(e.key==='ArrowUp')    orbitTarget.addScaledVector(up,     step);
      if(e.key==='ArrowDown')  orbitTarget.addScaledVector(up,    -step);
    }
    updateCamera();
    return;
  }
  // + / - Zoom
  if(e.key==='+'||e.key==='='){e.preventDefault();orbitState.radius=Math.max(200,orbitState.radius*0.85);orthoHalfSize=Math.max(200,orthoHalfSize*0.85);updateCamera();return;}
  if(e.key==='-'){e.preventDefault();orbitState.radius=Math.min(50000,orbitState.radius*1.18);orthoHalfSize=Math.min(50000,orthoHalfSize*1.18);updateCamera();return;}
});

// Alles ins Bild bringen (Roboter + Pfad + Trace)
function fitAllToView() {
  var box = new THREE.Box3();
  // Roboter-Pivots
  if (typeof pivots !== 'undefined' && pivots.length) {
    pivots.forEach(function(p) {
      var pos = new THREE.Vector3();
      p.getWorldPosition(pos);
      box.expandByPoint(pos);
    });
  }
  // Programm-Pfad
  if (parsedData && parsedData.positions) {
    parsedData.positions.forEach(function(p) {
      box.expandByPoint(new THREE.Vector3(p.X, p.Y, p.Z));
    });
  }
  // Trace-Punkte
  if (typeof tcpTracePoints !== 'undefined' && tcpTracePoints.length) {
    tcpTracePoints.forEach(function(p) { box.expandByPoint(p); });
  }
  if (box.isEmpty()) return;
  var ctr = new THREE.Vector3();
  box.getCenter(ctr);
  var span = box.getSize(new THREE.Vector3()).length();
  orbitTarget.copy(ctr);
  orbitState.radius = Math.max(600, span * 1.4);
  orthoHalfSize = Math.max(500, span * 0.6);
  updateCamera();
}

// ═══════════════════════════════════════════════════
// PATH BUILDING
// ═══════════════════════════════════════════════════
function buildScene(positions){ requestRender();
  posGrp.clear(); posGrp.visible = showPosFrames;pathGrp.clear();tcpTraceGrp.clear();tcpTracePoints.length=0;
  visitedLine=null;markerGrp.visible=false;selSphere.visible=false;
  if(!positions.length)return;
  const pts=positions.map(p=>new THREE.Vector3(p.X,p.Y,p.Z));
  if(pts.length>1)pathGrp.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts),new THREE.LineBasicMaterial({color:hexToInt((document.getElementById('cfg-planned-col')||{value:'#1a3050'}).value),linewidth:pathLineWidth})));
  positions.forEach((pos,i)=>{const g=makeFrame(pos);g.userData.posIdx=i;posGrp.add(g);});
  const box=new THREE.Box3();pts.forEach(p=>box.expandByPoint(p));
  const ctr=new THREE.Vector3();box.getCenter(ctr);const span=box.getSize(new THREE.Vector3()).length();
  orbitTarget.copy(ctr);orbitState.radius=Math.max(800,span*1.9);orthoHalfSize=Math.max(800,span*.7);updateCamera();
}

// ═══════════════════════════════════════════════════
// GUTTER
// ═══════════════════════════════════════════════════
let posLineNums=new Set();
function buildGutter(count){
  const gut=document.getElementById('gutter');gut.innerHTML='';
  for(let i=0;i<count;i++){
    const row=document.createElement('div');row.className='gl';row.dataset.line=i;
    if(posLineNums.has(i))row.classList.add('can-bp');if(breakpoints.has(i))row.classList.add('has-bp');
    const ln=document.createElement('span');ln.className='ln';ln.textContent=i+1;
    const dot=document.createElement('span');dot.className='bpdot';
    row.appendChild(ln);row.appendChild(dot);row.addEventListener('click',()=>toggleBP(i,row));gut.appendChild(row);
  }
}
function toggleBP(ln,row){if(!posLineNums.has(ln))return;if(breakpoints.has(ln)){breakpoints.delete(ln);row.classList.remove('has-bp');}else{breakpoints.add(ln);row.classList.add('has-bp');}}
function rebuildGutter(){const lines=document.getElementById('code-input').value.split(/\r?\n/);buildGutter(lines.length);}
document.getElementById('code-input').addEventListener('scroll',function(){document.getElementById('gutter').scrollTop=this.scrollTop;});
function updateGutterActive(lineNum){
  // Formular-Modus: Karte hervorheben
  var fv = document.getElementById('krl-form-view');
  if (fv && fv.style.display !== 'none') {
    fv.querySelectorAll('.fv-card,.fv-sv-card').forEach(function(el){ el.classList.remove('fv-sim-active'); });
    var card = fv.querySelector('[data-line="'+lineNum+'"]');
    if (card) { card.classList.add('fv-sim-active'); card.scrollIntoView({block:'nearest',behavior:'smooth'}); }
    return;
  }
  // KUKA KRL Modus: Gutter
  document.querySelectorAll('#gutter .gl').forEach(r=>r.classList.remove('active'));
  const t=document.querySelector(`#gutter .gl[data-line="${lineNum}"]`);if(!t)return;t.classList.add('active');
  const ta=document.getElementById('code-input');const lh=20,pv=10;const top=lineNum*lh+pv;
  if(top<ta.scrollTop||top>ta.scrollTop+ta.clientHeight-lh)ta.scrollTop=Math.max(0,top-ta.clientHeight/2);
}

// ═══════════════════════════════════════════════════
// UI RENDER HELPERS
// ═══════════════════════════════════════════════════
const TYPE_LBL={LIN:'LIN — Linear',PTP:'PTP — Point-to-Point',SLIN:'SLIN — Soft-Linear',CIRC:'CIRC — Endpunkt',CIRC_AUX:'CIRC — Hilfspunkt'};
function ff(v,d=2){return typeof v==='number'?v.toFixed(d):String(v);}

function updateIKBadge(idx, res) {
  const card = document.getElementById('pcard-'+idx);
  if (!card) return;
  const badge = card.querySelector('.ik-reach');
  if (!badge) return;
  badge.className = 'ik-reach' + ((res&&res.ok) ? '' : ' err');
  badge.textContent = (res&&res.ok) ? `IK ✓  Δ${res.score.toFixed(1)}` : 'IK ✗ nicht erreichbar';
}

function renderPositions(positions){ requestRender();
  const el=document.getElementById('pos-list');
  if(!positions.length){el.innerHTML='<div class="empty">' + t('no_pos2') + '</div>';return;}
  el.innerHTML=positions.map((p,i)=>{
    const tc=(p.type||'').toLowerCase();
    const ik=ikTable[i];
    const d=p.rel || p;
    const ikHtml=ik?`<div class="psep"></div><div class="ik-reach${ik.ok?'':' err'}">${ik.ok?`IK ✓  Δ${ik.score.toFixed(1)}`:'IK ✗ nicht erreichbar'}</div>`:'';
    return`<div class="pc ${tc}" id="pcard-${i}">
      <div class="pc-type ${tc}">#${i+1} &nbsp;${TYPE_LBL[p.type]||p.type}${p.lineNum!==undefined?`<span style="color:var(--txt3);font-weight:normal;font-size:.85em"> L.${p.lineNum+1}</span>`:''}
      </div>
      <div class="pc-grid">
        <div class="pf"><span>X</span> ${ff(d.X)} mm</div><div class="pf"><span>Y</span> ${ff(d.Y)} mm</div><div class="pf"><span>Z</span> ${ff(d.Z)} mm</div>
        <div class="psep"></div>
        <div class="pf"><span>A(Z)</span> ${ff(d.A)}°</div><div class="pf"><span>B(Y)</span> ${ff(d.B)}°</div><div class="pf"><span>C(X)</span> ${ff(d.C)}°</div>
        ${ikHtml}
      </div></div>`;
  }).join('');
  positions.forEach((_,i)=>{(function(){var _e=document.getElementById('pcard-'+i);if(_e)_e.addEventListener('click',function(){pauseSim();setStatus('paused','PAUSED');selectPosition(i);});})();;});
}

function updatePosCards(activeIdx){document.querySelectorAll('.pc').forEach((el,i)=>el.classList.toggle('sim-cur',i===activeIdx));(function(){var _e=document.getElementById('pcard-'+activeIdx);if(_e)_e.scrollIntoView({block:'nearest',behavior:'smooth'});})();;}
function renderVariables(vars){const el=document.getElementById('var-list');const entries=Object.entries(vars);if(!entries.length){el.innerHTML='<div class="empty">' + t('no_vars') + '</div>';return;}el.innerHTML=entries.map(([n,v])=>{const d=typeof v==='boolean'?(v?'TRUE':'FALSE'):String(v);return`<div class="vr"><span class="vn">${n}</span><span class="vv">${d}</span></div>`;}).join('');}
function _spToggleDig(prefix,idx){
  const key=prefix==='$IN'?'digitalIn':'digitalOut';
  const cur=parsedData.finalState[key][idx];
  const on=cur===true||cur==='TRUE'||cur===1;
  parsedData.finalState[key][idx]=!on;
  if(prefix==='$IN') renderDigital(parsedData.finalState.digitalIn,'$IN','din-list');
  else renderDigital(parsedData.finalState.digitalOut,'$OUT','dout-list');
}
function renderDigital(sigs,prefix,elId){const el=document.getElementById(elId);const entries=Object.entries(sigs).sort((a,b)=>+a[0]-+b[0]);if(!entries.length){el.innerHTML=`<div class="empty">Kein ${prefix}</div>`;return;}el.innerHTML=entries.map(([idx,val])=>{const on=val===true||val==='TRUE'||val===1;return`<div class="sr"><div class="led ${on?'on':'off'}"></div><span class="sn">${prefix}[${idx}]</span><button onclick="_spToggleDig('${prefix}',${idx})" class="sp-tog ${on?'on':'off'}">${on?'AN':'AUS'}</button></div>`;}).join('');}
function _spSetAnOut(idx,v){parsedData.finalState.analogOut[idx]=parseFloat(v);const lbl=document.getElementById('sp-an-v-'+idx);if(lbl)lbl.textContent=(v>=0?'+':'')+parseFloat(v).toFixed(2)+' V';}
function renderAnalog(sigs){const el=document.getElementById('anout-list');if(!el)return;const entries=Object.entries(sigs||{}).sort((a,b)=>+a[0]-+b[0]);if(!entries.length){el.innerHTML='<div class="empty">Kein $ANOUT</div>';return;}el.innerHTML=entries.map(([idx,v])=>{return`<div class="ar"><div class="ah"><span class="an">$ANOUT[${idx}]</span><span class="av" id="sp-an-v-${idx}">${v>=0?'+':''}${v.toFixed(2)} V</span></div><input type="range" class="sp-aslider" min="-10" max="10" step="0.1" value="${v}" oninput="_spSetAnOut(${idx},this.value)"></div>`;}).join('');}
function _spSetAnIn(idx,v){parsedData.finalState.analogIn[idx]=parseFloat(v);const lbl=document.getElementById('sp-ain-v-'+idx);if(lbl)lbl.textContent=(v>=0?'+':'')+parseFloat(v).toFixed(2)+' V';}
function renderAnalogIn(sigs){const el=document.getElementById('ain-list');if(!el)return;const entries=Object.entries(sigs||{}).sort((a,b)=>+a[0]-+b[0]);if(!entries.length){el.innerHTML='<div class="empty">Kein $ANIN</div>';return;}el.innerHTML=entries.map(([idx,v])=>{const vn=typeof v==='number'?v:0;return`<div class="ar"><div class="ah"><span class="an">$ANIN[${idx}]</span><span class="av" id="sp-ain-v-${idx}">${vn>=0?'+':''}${vn.toFixed(2)} V</span></div><input type="range" class="sp-aslider" min="-10" max="10" step="0.1" value="${vn}" oninput="_spSetAnIn(${idx},this.value)"></div>`;}).join('');}

// ── Live update when editing position coordinates ──────────
function liveEditUpdate() {
  const x=parseFloat(document.getElementById('ep-x').value)||0;
  const y=parseFloat(document.getElementById('ep-y').value)||0;
  const z=parseFloat(document.getElementById('ep-z').value)||0;
  const a=parseFloat(document.getElementById('ep-a').value)||0;
  const b=parseFloat(document.getElementById('ep-b').value)||0;
  const cv=parseFloat(document.getElementById('ep-c').value)||0;

  // Update 3D marker + coordinate frame live
  if (selectedPosIdx!==null) {
    const grp=posGrp.children[selectedPosIdx];
    if(grp){grp.position.set(x,y,z);grp.setRotationFromEuler(kukaEuler(a,b,cv));}
    selSphere.position.set(x,y,z);
    markerGrp.position.set(x,y,z);markerGrp.setRotationFromEuler(kukaEuler(a,b,cv));markerGrp.visible=true;
    // Update stored position so FK error is computed correctly
    parsedData.positions[selectedPosIdx] = {
      ...parsedData.positions[selectedPosIdx], X:x,Y:y,Z:z,A:a,B:b,C:cv
    };
  }

  // Compute IK + update robot
  const res=solveIK(x,y,z,a,b,cv);
  if(res.ok) applyAngles(res.angles);
  document.getElementById('rb-fk').textContent=res.score.toFixed(2);

  // Compute and show all unique solutions
  showEpIKSolutions(x,y,z,a,b,cv);
}

function showEpIKSolutions(x,y,z,a,b,cv) {
  // 12 kanonische Starts — decken alle 8 Roboter-Konfigurationen ab:
  // Schulter (A1 Vorne/Hinten/Links/Rechts) × Elbow (↑/↓) × Handgelenk (normal/flip)
  const CANON = [
    // Elbow↑, Handgelenk normal
    [   0,  -90,  90,   0,  -90,  0],  // Vorne
    [ 170,  -90,  90,   0,  -90,  0],  // Hinten
    [  90,  -90,  90,   0,  -90,  0],  // Rechts
    [ -90,  -90,  90,   0,  -90,  0],  // Links
    // Elbow↑, Handgelenk flip (A5 > 0)
    [   0,  -90,  90,   0,   90,  0],  // Vorne, Flip
    [ 170,  -90,  90,   0,   90,  0],  // Hinten, Flip
    [  90,  -90,  90,   0,   90,  0],  // Rechts, Flip
    // Elbow↓ Variante A (A3 tief)
    [   0,  -30, -90,   0,  -90,  0],  // Vorne, Elbow↓
    [ 170,  -30, -90,   0,  -90,  0],  // Hinten, Elbow↓
    // Elbow↓ Variante B (A2 tief, A3 hoch → anderer Arm-Bogen)
    [   0, -150, 120,   0,  -90,  0],  // Vorne, Elbow↓ alt
    [ 170, -150, 120,   0,  -90,  0],  // Hinten, Elbow↓ alt
    // Overhead-Bereich
    [   0,  -20, 150,   0,  -90,  0],
  ];

  const cur = jointAngles.slice();
  const raw = [];

  for (const start of CANON) {
    const res = solveLM({
      tp:[x,y,z], Rt:rotZYX(a,b,cv),
      starts:[start],
      dt:0.3, lam:0.5, tolP:0.8, tolO:0.8,
      maxIter:250, stepMax:2.0, stepScale:10.0,
      earlyStop:0, okThresh:8,
    });
    if (res.ok) raw.push(res.angles);
  }

  // Deduplizierung: Lösungen mit max. Gelenkwinkelabstand < 12° gelten als identisch
  const unique = [];
  for (const ang of raw) {
    const isDup = unique.some(u =>
      u.reduce((mx,v,i) => Math.max(mx, Math.abs(v-ang[i])), 0) < 12
    );
    if (!isDup) unique.push(ang);
  }

  // Klassifikation jeder Lösung anhand der tatsächlichen Gelenkwinkel
  function classify(ang) {
    const a1 = ang[0], a3 = ang[2], a5 = ang[4];
    // Schulter
    const shoulder = Math.abs(a1) < 80 ? 'Vorne'
                   : a1 > 80            ? 'Rechts'
                   :                      'Links';
    // Elbow
    const elbow = a3 > 50 ? 'Elbow↑' : 'Elbow↓';
    // Handgelenk
    const wrist = a5 > 10 ? '· Flip' : '';
    return shoulder + ' · ' + elbow + (wrist ? ' ' + wrist : '');
  }

  const solutions = unique.map(angles => {
    const inLimit = angles.every((v,i) => v >= JOINTS_DEF[i].min && v <= JOINTS_DEF[i].max);
    const cost = Math.sqrt(angles.reduce((s,v,i) => s+(v-cur[i])**2, 0));
    return { angles, inLimit, cost, label:classify(angles), score:0, ok:true };
  });

  // Sort: innerhalb Limit zuerst, dann nach Bewegungskosten (nächste Konfiguration zuerst)
  solutions.sort((a,b) => (!a.inLimit&&b.inLimit)?1:(!b.inLimit&&a.inLimit)?-1:a.cost-b.cost);
  if (solutions.length > 0) solutions[0].isBest = true;

  const listEl=document.getElementById('ep-ik-list');
  const secEl=document.getElementById('ep-ik-solutions');
  if(!listEl){return;}
  secEl.style.display=solutions.length?'block':'none';

  // Store solutions globally for safe index-based access
  window._epSolutions = solutions;

  listEl.innerHTML = solutions.map(function(sol, i) {
    var badge = sol.inLimit
      ? '<span class="ep-sol-ok ok">' + (sol.isBest ? '★ ' : '') + 'OK</span>'
      : '<span class="ep-sol-ok lim">Limit</span>';
    var angleStr = sol.angles.map(function(v, j) {
      return 'A'+(j+1)+':'+v.toFixed(0)+'°';
    }).join(' ');
    return '<div class="ep-sol' + (sol.isBest ? ' best' : '') + '" data-sol-idx="' + i + '">'
      + '<span class="ep-sol-lbl">' + sol.label + '</span>'
      + '<span class="ep-sol-cost">' + sol.cost.toFixed(0) + '°</span>'
      + badge
      + '<div style="font-size:.7em;color:var(--txt3);width:100%;margin-top:2px">' + angleStr + '</div>'
      + '</div>';
  }).join('');

  // Attach click handlers directly (no inline JSON)
  listEl.querySelectorAll('.ep-sol').forEach(function(el) {
    el.addEventListener('click', function() {
      var idx = parseInt(this.getAttribute('data-sol-idx'));
      var sols = window._epSolutions;
      if (!sols || idx < 0 || idx >= sols.length) return;
      // Highlight selected
      listEl.querySelectorAll('.ep-sol').forEach(function(e) { e.style.outline = 'none'; });
      this.style.outline = '2px solid var(--acc)';
      // FK → XYZABC → Editor schreiben
      applyEpSolution(sols[idx].angles);
    });
  });
}

function applyEpSolution(angles) {
  tweenToAngles(angles, 500);

  // FK berechnen und Felder aktualisieren (nur Anzeige, kein Code-Writeback)
  var fk  = fkAll(angles);
  var tcp = fk.pts[7];
  var R   = fk.tcp_rot;
  var B2  = -Math.asin(Math.max(-1, Math.min(1, R[2][0])));
  var cb2 = Math.cos(B2);
  var A2  = Math.abs(cb2) > 1e-6 ? Math.atan2(R[1][0]/cb2, R[0][0]/cb2) : 0;
  var C2  = Math.abs(cb2) > 1e-6 ? Math.atan2(R[2][1]/cb2, R[2][2]/cb2) : Math.atan2(-R[1][2], R[1][1]);
  function toDeg(v){ var d=v*180/Math.PI; while(d>180)d-=360; while(d<=-180)d+=360; return d; }
  document.getElementById('ep-x').value = tcp[0].toFixed(3);
  document.getElementById('ep-y').value = tcp[1].toFixed(3);
  document.getElementById('ep-z').value = tcp[2].toFixed(3);
  document.getElementById('ep-a').value = toDeg(A2).toFixed(3);
  document.getElementById('ep-b').value = toDeg(B2).toFixed(3);
  document.getElementById('ep-c').value = toDeg(C2).toFixed(3);

  if (selectedPosIdx === null) return;
  ikTable[selectedPosIdx] = { angles: angles, score: 0, ok: true };
}

function writeBackPosition(idx, x, y, z, a, b, c) {
  if (idx === null || idx === undefined) return;
  var pos = parsedData.positions[idx];
  if (!pos) return;
  // Update KRL editor
  var ta = document.getElementById('code-input');
  if (!ta) return;
  var lines = ta.value.split('\n');
  var lineNum = pos.lineNum;  // lineNum ist 0-basiert
  if (lineNum < 0 || lineNum >= lines.length) return;
  var line = lines[lineNum];
  // Replace coordinate values in the line
  var newCoords = 'X ' + x.toFixed(3) + ', Y ' + y.toFixed(3) +
    ', Z ' + z.toFixed(3) + ', A ' + a.toFixed(3) +
    ', B ' + b.toFixed(3) + ', C ' + c.toFixed(3);
  lines[lineNum] = line.replace(
    /X\s*[-\d.]+\s*,\s*Y\s*[-\d.]+\s*,\s*Z\s*[-\d.]+\s*,\s*A\s*[-\d.]+\s*,\s*B\s*[-\d.]+\s*,\s*C\s*[-\d.]+/,
    newCoords
  );
  ta.value = lines.join('\n');
  rebuildGutter();
}

function toggleSec(titleEl){titleEl.closest('.sec').classList.toggle('collapsed');}

// ═══════════════════════════════════════════════════
// KINEMATIK CONFIG UI
// ═══════════════════════════════════════════════════
function buildKinConfig(){
  const limEl=document.getElementById('joint-limits-ui');
  limEl.innerHTML=`
    <div class="joint-cfg" style="font-size:.68em;color:var(--txt3);font-weight:bold;border-bottom:1px solid var(--bdr)">
      <span></span><span style="text-align:center">Min [°]</span><span style="text-align:center">Max [°]</span><span></span><span></span>
    </div>`+
  JOINTS_DEF.map((j,i)=>`
    <div class="joint-cfg">
      <span class="jl">A${i+1}</span>
      <input id="jmin${i}" type="number" value="${j.min}" step="1" style="width:100%">
      <input id="jmax${i}" type="number" value="${j.max}" step="1" style="width:100%">
      <span style="color:var(--txt3);font-size:.75em">${j.axis}</span><span></span>
    </div>`).join('');

  const offEl=document.getElementById('joint-offsets-ui');
  offEl.innerHTML=`
    <div class="joint-cfg" style="font-size:.68em;color:var(--txt3);font-weight:bold;border-bottom:1px solid var(--bdr);margin-top:6px">
      <span></span><span style="text-align:center">X [mm]</span><span style="text-align:center">Y [mm]</span><span style="text-align:center">Z [mm]</span><span></span>
    </div>`+
  JOINTS_DEF.map((j,i)=>`
    <div class="joint-cfg">
      <span class="jl">A${i+1}</span>
      <input id="jox${i}" type="number" value="${j.off[2]}" step="1" style="width:100%">
      <input id="joy${i}" type="number" value="${j.off[1]}" step="1" style="width:100%">
      <input id="joz${i}" type="number" value="${j.off[0]}" step="1" style="width:100%">
      <span style="color:var(--txt3);font-size:.75em">${j.axis}</span>
    </div>`).join('');
}

function applyKinematicConfig(){
  JOINTS_DEF.forEach((j,i)=>{
    j.min=parseFloat(document.getElementById('jmin'+i).value)||j.min;
    j.max=parseFloat(document.getElementById('jmax'+i).value)||j.max;
    const ox=parseFloat((document.getElementById('jox'+i)&&document.getElementById('jox'+i).value));
    const oy=parseFloat((document.getElementById('joy'+i)&&document.getElementById('joy'+i).value));
    const oz=parseFloat((document.getElementById('joz'+i)&&document.getElementById('joz'+i).value));
    if(!isNaN(ox))j.off[2]=ox;  // X-Feld → off[2]
    if(!isNaN(oy))j.off[1]=oy;
    if(!isNaN(oz))j.off[0]=oz;  // Z-Feld → off[0]
  });
  buildPivotChain();
  buildRobotModel(jointAngles);
  // Recompute IK + trajectory with new kinematics
  if(parsedData.positions.length>0){
    setStatus('paused','Kinematik geändert — IK wird neu berechnet…');
    setTimeout(()=>{
      computeIKTable(parsedData.positions);
      renderPositions(parsedData.positions);
      if(trajectory.length>0) buildTrajectory(parsedData.positions, ikTable);
      const angles=getIKAngles(simTToTrajT(sim.t));
      if(angles) applyAngles(angles);
      setStatus('paused','Kinematik übernommen · IK neu berechnet');
    },10);
  } else {
    setStatus('paused','Kinematik übernommen');
  }
}

function updateTCPDef(){
  TCP_DEF.x=parseFloat(document.getElementById('tcp-x').value)||0;
  TCP_DEF.y=parseFloat(document.getElementById('tcp-y').value)||0;
  TCP_DEF.z=parseFloat(document.getElementById('tcp-z').value)||0;
  TCP_DEF.a=parseFloat(document.getElementById('tcp-a').value)||0;
  TCP_DEF.b=parseFloat(document.getElementById('tcp-b').value)||0;
  TCP_DEF.c=parseFloat(document.getElementById('tcp-c').value)||0;
  buildRobotModel(jointAngles);
  // KEINE IK-Neuberechnung hier — IK wird nur beim Parsen berechnet (dann pro Punkt mit seinem TCP).
  // Editieren des TCP setzt nur TCP_DEF bzw. den tcpList-Eintrag; zum Anwenden neu parsen (F9).
}

// ═══════════════════════════════════════════════════
// PARSE & LOAD
// ═══════════════════════════════════════════════════
var _krlSnapshot = '';
var _krlSnapshotParsed = false;

function parseAndLoad(){
  var _activeId = (typeof FormatRegistry !== 'undefined' && FormatRegistry.getActiveId) ? FormatRegistry.getActiveId() : null;
  var _isKRL = !_activeId || _activeId === 'kuka' || _activeId === 'kuka-form';
  var code;
  if (_isKRL) {
    code = document.getElementById('code-input').value;
    if (code !== _krlSnapshot) { _krlSnapshot = code; _krlSnapshotParsed = false; }
  } else {
    if (!_krlSnapshot) return;
    if (_krlSnapshotParsed && parsedData && parsedData.positions.length > 0) {
      // Bereits geparst — nur Simulation neu starten, keine IK-Neuberechnung
      stopSim(0); setStatus('stopped','STOPPED');
      applySimT(0); sim.stepIdx = 0;
      if (parsedData.steps && parsedData.steps.length) applyStep(0);
      else setStatus('paused','BEREIT');
      return;
    }
    code = _krlSnapshot;
    _krlSnapshotParsed = true;
  }
  parsedData=parseKRL(code);const N=parsedData.positions.length;
  ensureBasesForProgram(); // Eintraege fuer alle referenzierten BASE_DATA[n] sicherstellen
  bakeBaseOffset(); // Code-Koordinaten + jeweils aktiver BASE-Offset pro Punkt -> Welt
  posLineNums=new Set(parsedData.positions.map(p=>p.lineNum).filter(n=>n!==undefined));
  for(const bp of[...breakpoints])if(!posLineNums.has(bp))breakpoints.delete(bp);
  buildGutter(code.split(/\r?\n/).length);
  const ps=document.getElementById('pos-s');ps.min=0;ps.max=Math.max(0,N-1);ps.step=0.001;ps.value=0;
  document.getElementById('pos-v').textContent=N>0?`1 / ${N}`:'— / —';
  stopSim(0);setStatus('stopped','STOPPED');document.getElementById('marker-info').style.display='none';deselectPosition();
  buildScene(parsedData.positions);
  // Pre-compute IK for all positions (häppchenweise mit Fortschrittsleiste)
  setStatus('paused','IK lädt…');
  parseProgress(15, N + ' Punkte');
  setTimeout(()=>{
    computeIKTableProgressive(parsedData.positions, ()=>{
      renderPositions(parsedData.positions);
      renderVariables(parsedData.finalState.variables);renderDigital(parsedData.finalState.digitalIn,'$IN','din-list');renderDigital(parsedData.finalState.digitalOut,'$OUT','dout-list');renderAnalog(parsedData.finalState.analogOut);
      if(N>0){applySimT(0);sim.stepIdx=0;if((parsedData.steps&&parsedData.steps.length))applyStep(0);else setStatus('paused','BEREIT');}
      else setStatus('stopped','STOPPED');
      parseProgressHide();
    });
  },10);
}

document.getElementById('parse-btn').addEventListener('click',function(){
  // Scan-Daten invalidieren — MAP erzeugen muss danach neu geklickt werden
  _aPlotReach = null; _aPlotReachMid = null; _aPlotReachAngles = null; _aPlotReachSing = null;
  parseAndLoad();
  var info = document.getElementById('aplot-info');
  if (info) info.textContent = '— MAP erzeugen klicken für Reachability-Scan';
});

// ═══════════════════════════════════════════════════
// INIT
// ═══════════════════════════════════════════════════
splashProgress(30, 'Kinematik wird geladen…');
buildKinConfig();
buildSteuerAxes();
buildAxisSTLUI();
splashProgress(60, 'Robotermodell wird erstellt…');
buildRobotModel([0,-90,90,0,0,0]);

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


// STL-Dateien werden per fetch aus ./stl/ geladen
function xhrSTL(url, onDone, onErr) {
  // Absolute URL aufbauen um Pfad-Probleme zu vermeiden
  var base = window.location.href.replace(/\/[^\/]*$/, '/');
  var absUrl = url.indexOf('http') === 0 ? url : base + url.replace(/^\.\//,'');
  var xhr = new XMLHttpRequest();
  xhr.open('GET', absUrl, true);
  xhr.responseType = 'arraybuffer';
  xhr.timeout = 60000;
  xhr.onload = function() {
    if ((xhr.status >= 200 && xhr.status < 300) || xhr.status === 0) {
      if (xhr.response && xhr.response.byteLength > 0) {
        onDone(xhr.response);
      } else {
        console.warn('XHR leer:', absUrl);
        if(onErr) onErr('empty response');
      }
    } else {
      console.warn('XHR Status ' + xhr.status + ':', absUrl);
      if(onErr) onErr('HTTP ' + xhr.status);
    }
  };
  xhr.onerror   = function(e) { console.error('XHR Netzwerkfehler:', absUrl, e); if(onErr) onErr('network error'); };
  xhr.ontimeout = function()  { console.error('XHR Timeout:', absUrl); if(onErr) onErr('timeout'); };
  xhr.send();
}

function loadDefaultSTLs() {
  var axes = ['A1','A2','A3','A4','A5','A6'];
  var loaded = 0;
  var mi = document.getElementById('marker-info');
  if (mi) { mi.style.display='block'; mi.textContent='Lade STL Modelle...'; }
  var i = 0;
  function next() {
    if (i >= axes.length) {
      if (mi) { mi.style.display = loaded > 0 ? 'none' : 'block'; if(loaded===0) mi.textContent='STL nicht geladen — ↺ STL klicken'; }
      return;
    }
    var ax = axes[i++];
    var idx = parseInt(ax.replace('A','')) - 1;
    if (mi) mi.textContent = 'Lade ' + ax + '...';
    // ZIP-Cache zuerst prüfen
    var _cachedBuf = window._zipSTLCache && window._zipSTLCache[ax.toLowerCase()];
    if (_cachedBuf) {
      (function(buf, axName, axIdx) {
        try {
          var geo = stlLoader.parse(buf); geo.computeVertexNormals();
          if (axisSTLMeshes[axIdx]) { scene.remove(axisSTLMeshes[axIdx]); axisSTLMeshes[axIdx].geometry.dispose(); }
          axisSTLMeshes[axIdx] = new THREE.Mesh(geo, new THREE.MeshPhongMaterial({color:0xe8a020,shininess:80}));
          scene.add(axisSTLMeshes[axIdx]);
          if (!axisSTLMode[axIdx]) axisSTLMode[axIdx] = 'solid';
          setAxisSTLMode(axIdx, axisSTLMode[axIdx]);
          var nameEl = document.getElementById('asl-name'+axIdx);
          if (nameEl) nameEl.textContent = axName.toLowerCase();
          var delEl = document.getElementById('asl-del'+axIdx);
          if (delEl) delEl.style.display = '';
          loaded++; buildRobotModel(jointAngles);
        } catch(e) { console.error('STL parse (cache):', axName, e); }
        next();
      })(_cachedBuf, ax, idx);
      return;
    }
    xhrSTL('./stl/' + ax.toLowerCase() + '.stl',
      function(buf) {
        try {
          window['_axisSTLBuffer'+idx] = buf;
          // Also keep a copy in the array for safe access
          if (!window._axisSTLBuffers) window._axisSTLBuffers = {};
          window._axisSTLBuffers[idx] = buf;
          var geo = stlLoader.parse(buf);
          geo.computeVertexNormals();
          if (axisSTLMeshes[idx]) { scene.remove(axisSTLMeshes[idx]); axisSTLMeshes[idx].geometry.dispose(); }
          axisSTLMeshes[idx] = new THREE.Mesh(geo, new THREE.MeshPhongMaterial({color:0xe8a020,shininess:80}));
          scene.add(axisSTLMeshes[idx]);
          // Mode anwenden falls schon gesetzt
          if (!axisSTLMode[idx]) axisSTLMode[idx] = 'solid';
          setAxisSTLMode(idx, axisSTLMode[idx]);
          var nameEl = document.getElementById('asl-name'+idx);
          if (nameEl) nameEl.textContent = ax.toLowerCase();
          var delEl = document.getElementById('asl-del'+idx);
          if (delEl) delEl.style.display = '';
          loaded++;
          buildRobotModel(jointAngles);
        } catch(e) { console.error('STL parse:', ax, e); }
        next();
      },
      function(e) { console.warn('STL load error:', ax, e); next(); }
    );
  }
  next();
}


// Podest + Tool per fetch laden
function loadDefaultSceneSTLs() {
  xhrSTL('./stl/podest.stl', function(buf) {
    try {
      window._pedestalSTLBuffer = buf;
      var geo = stlLoader.parse(buf); geo.computeVertexNormals();
      if (pedestalMesh) { scene.remove(pedestalMesh); pedestalMesh.geometry.dispose(); }
      pedestalMesh = new THREE.Mesh(geo, new THREE.MeshPhongMaterial({color:0x334455,shininess:40}));
      scene.add(pedestalMesh);
      var el = document.getElementById('pedestal-name');
      if (el) el.textContent = 'podest';
      var vb = document.getElementById('vis-pedestal');
      if (vb) vb.innerHTML = svgIconSolid;
    } catch(e) { console.error('Podest parse:', e); }
  }, function(e) { console.warn('Podest load:', e); });

  xhrSTL('./stl/tool1_tcp.stl', function(buf) {
    try {
      window._toolSTLBuffer = buf;
      var geo = stlLoader.parse(buf); geo.computeVertexNormals();
      if (toolMesh) { scene.remove(toolMesh); toolMesh.geometry.dispose(); toolMesh.material.dispose(); }
      var mat = new THREE.MeshPhongMaterial({color:0xdd9944,transparent:false,opacity:1.0,side:THREE.DoubleSide,specular:0x666666});
      toolMesh = new THREE.Mesh(geo, mat);
      scene.add(toolMesh);
      document.getElementById('tool-filename').textContent = 'Tool1_TCP';
      var tvb = document.getElementById('vis-tool');
      if (tvb) tvb.innerHTML = svgIconSolid;
      document.getElementById('tool-controls').style.display = 'block';
      buildRobotModel(jointAngles);
    } catch(e) { console.error('Tool parse:', e); }
  }, function(e) { console.warn('Tool load:', e); });
}


function toggleToolMesh() {
  showToolMesh = !showToolMesh;
  document.getElementById('btn-show-tool').classList.toggle('on', showToolMesh);
  if (toolMesh) toolMesh.visible = showRobot3D && showToolMesh;
}
function togglePedestalMesh() {
  showPedestalMesh = !showPedestalMesh;
  document.getElementById('btn-show-pedestal').classList.toggle('on', showPedestalMesh);
  if (pedestalMesh) pedestalMesh.visible = showPedestalMesh;
}

// ── 3-Panel Resize (Editor | Viewport | Parameter) ────────
(function(){
  function makeResizer(handleId, getPanel, getDir, saveKey) {
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
      // Breite in Settings speichern
      var panel = getPanel();
      if (panel && saveKey) {
        try {
          var s = JSON.parse(localStorage.getItem(SETTINGS_KEY) || '{}');
          s[saveKey] = panel.offsetWidth;
          localStorage.setItem(SETTINGS_KEY, JSON.stringify(s));
        } catch(e) {}
      }
    });
  }
  // Left handle: editor
  makeResizer('ep-resize-handle', function(){ return document.querySelector('.ep'); }, function(){ return 1; }, 'epWidth');
  // Right handle: info-panel
  makeResizer('ip-resize-handle', function(){ return document.getElementById('info-panel'); }, function(){ return -1; }, 'ipWidth');

  // Gespeicherte Breiten wiederherstellen
  window.addEventListener('load', function(){
    try {
      var s = JSON.parse(localStorage.getItem(SETTINGS_KEY) || '{}');
      if (s.epWidth) {
        var ep = document.querySelector('.ep');
        if (ep) { ep.style.width = s.epWidth + 'px'; resize(); }
      }
      if (s.ipWidth) {
        var ip = document.getElementById('info-panel');
        if (ip) { ip.style.width = s.ipWidth + 'px'; resize(); }
      }
    } catch(e) {}
  });
})();

// ══════════════════════════════════════════════════════════
// EINSTELLUNGSFENSTER — localStorage-Persistenz
// ══════════════════════════════════════════════════════════
var SETTINGS_KEY = 'robsim_settings';

var defaultSettings = {
  fzEditor: 17,
  fzUi:     17,
  fzPanel:  17,
  fzStatus: 17
};

function loadSettings() {
  try {
    var s = localStorage.getItem(SETTINGS_KEY);
    return s ? Object.assign({}, defaultSettings, JSON.parse(s)) : Object.assign({}, defaultSettings);
  } catch(e) { return Object.assign({}, defaultSettings); }
}

function saveSettings() {
  var s = {
    fzEditor: parseInt(document.getElementById('fz-editor').value) || 17,
    fzUi:     parseInt(document.getElementById('fz-ui').value)     || 17,
    fzPanel:  parseInt(document.getElementById('fz-panel').value)  || 17,
    fzStatus: parseInt(document.getElementById('fz-status').value) || 17
  };
  try { localStorage.setItem(SETTINGS_KEY, JSON.stringify(s)); } catch(e) {}
  applyFZ();
}

function resetSettings() {
  try { localStorage.removeItem(SETTINGS_KEY); } catch(e) {}
  applySettingsToUI(defaultSettings);
  applyFZ();
}

function applySettingsToUI(s) {
  var fields = {
    'fz-editor': s.fzEditor, 'fz-ui': s.fzUi,
    'fz-panel': s.fzPanel, 'fz-status': s.fzStatus
  };
  Object.entries(fields).forEach(function(kv) {
    var el = document.getElementById(kv[0]);
    if (el) el.value = kv[1];
  });
}

function fzWheel(e, inp) {
  // Legacy — global wheel handler takes care of this now
}

function applyFZ() {
  var fzE = parseInt(document.getElementById('fz-editor').value) || 17;
  var fzU = parseInt(document.getElementById('fz-ui').value)     || 17;
  var fzP = parseInt(document.getElementById('fz-panel').value)  || 17;
  var fzS = parseInt(document.getElementById('fz-status').value) || 17;

  // Editor + Gutter: CSS-Variablen setzen → alles synchron
  var lh = Math.round(fzE * 1.6);
  document.documentElement.style.setProperty('--fsz', fzE + 'px');
  document.documentElement.style.setProperty('--lh',  lh  + 'px');
  var codeEl = document.getElementById('code-input');
  if (codeEl) { codeEl.style.fontSize = fzE + 'px'; codeEl.style.lineHeight = lh + 'px'; }
  var gut = document.getElementById('gutter');
  if (gut) { gut.style.fontSize = fzE + 'px'; gut.style.lineHeight = lh + 'px'; }

  // Alles mit fontSize setzen via universellen Ansatz:
  // Toolbar, Buttons, Header, alle Labels
  document.querySelectorAll(
    '.tb, .sbtn, .slbl, .sv, .sstatus, header, .htag, ' +
    '.vbtn, #view-bar, #parse-btn, #parse-area'
  ).forEach(function(el) { el.style.fontSize = fzU + 'px'; });

  // Alle floating Panels (Steuerung, Einstellungen, Hilfe, Achsenkarte)
  document.querySelectorAll(
    '#steuer-panel *, #settings-panel *, #help-panel *,' +
    '#steuer-panel, #settings-panel, #help-panel'
  ).forEach(function(el) { el.style.fontSize = fzU + 'px'; });

  // Rechtes Info-Panel: alle Kindelemente
  var ip = document.getElementById('info-panel');
  if (ip) {
    ip.style.fontSize = fzP + 'px';
    ip.querySelectorAll('*').forEach(function(el) { el.style.fontSize = ''; });
    // Reset dann gezielt setzen
    ip.querySelectorAll(
      '.sec-t,.sec-b,.pt,label,.stl-lbl,.stl-inp,.stl-unit,.cfg-lbl,.cfg-row,' +
      '.pc,.pf,.vr,.sr,.ar,.ep-grid,.ep-lbl,.ep-inp,.ep-unit,input,select,button,span,div'
    ).forEach(function(el) { el.style.fontSize = fzP + 'px'; });
  }

  // Statusleiste + Roboterbar
  document.querySelectorAll('#robot-bar, #robot-bar *, .rb-val, .rb-dim, .scene-hint, #marker-info')
    .forEach(function(el) { el.style.fontSize = fzS + 'px'; });
}

function initSettings() {
  var s = loadSettings();
  applySettingsToUI(s);
  applyFZ();
}

function toggleSettings() {
  var p = document.getElementById('settings-panel');
  if (!p) return;
  p.style.display = (p.style.display === 'none' || p.style.display === '') ? 'block' : 'none';
}


// Drag for settings panel
(function() {
  var panel = document.getElementById('settings-panel');
  var handle = document.getElementById('settings-drag-handle');
  if (!panel || !handle) return;
  var dragging = false, ox = 0, oy = 0;
  handle.addEventListener('mousedown', function(e) {
    dragging = true;
    ox = e.clientX - panel.offsetLeft;
    oy = e.clientY - panel.offsetTop;
    panel.style.transform = 'none';
    e.preventDefault();
  });
  window.addEventListener('mousemove', function(e) {
    if (!dragging) return;
    panel.style.left = (e.clientX - ox) + 'px';
    panel.style.top  = (e.clientY - oy) + 'px';
  });
  window.addEventListener('mouseup', function() { dragging = false; });
})();


function clearTCPTrace() {
  tcpTracePoints.length = 0;
  tcpTraceGrp.clear();
}


function resetAll() {
  if (!confirm(t('confirm_reset'))) return;
  // Roboter Heimstellung
  jointAngles = [0,-90,90,0,0,0];
  applyAngles(jointAngles);
  // TCP Trace löschen
  clearTCPTrace();
  // Simulation stoppen
  pauseSim();
  setStatus('stopped', 'STOPPED');
  // Positionen/Variablen leeren
  parsedData = {positions:[], steps:[], finalState:{variables:{},digitalIn:{},digitalOut:{},analogOut:{}}};
  ikTable = [];
  trajectory = [];
  trajMax = 0;
  buildScene([]);
  renderPositions([]);
  renderVariables({});
  renderDigital({}, '$OUT', 'dout-list');
  renderDigital({}, '$IN',  'din-list');
  renderAnalog({});
  // Deselect
  deselectPosition();
  markerGrp.visible = false;
  // TCP zurücksetzen
  TCP_DEF = {x:364.5, y:0, z:46.5, a:0, b:90, c:0};
  document.getElementById('tcp-x').value = 364.5;
  document.getElementById('tcp-y').value = 0;
  document.getElementById('tcp-z').value = 46.5;
  document.getElementById('tcp-a').value = 0;
  document.getElementById('tcp-b').value = 90;
  document.getElementById('tcp-c').value = 0;
  // Kamera auf ISO
  currentView = 'iso';
  activeCam = perspCam;
  updateCamera();
}


function togglePosFrames() {
  showPosFrames = !showPosFrames;
  document.getElementById('btn-show-posframes').classList.toggle('on', showPosFrames);
  posGrp.visible = showPosFrames;
}
function toggleBaseFrame() {
  showBaseFrame = !showBaseFrame;
  document.getElementById('btn-show-baseframe').classList.toggle('on', showBaseFrame);
  baseFrameGroups.forEach(function(g){g.visible=showBaseFrame;});
  tcpFrameGroups.forEach(function(g){g.visible=showBaseFrame;});
  updateAxisLabelsVisibility();
}
function toggleTCPMarker() {
  showTCPMarker = !showTCPMarker;
  document.getElementById('btn-show-tcpmarker').classList.toggle('on', showTCPMarker);
  markerVisuals.visible = showTCPMarker;
}


// Mausrad auch ohne Fokus: beim Hover-Element
(function() {
  var hoveredNumber = null;
  document.addEventListener('mouseover', function(e) {
    if (e.target && e.target.type === 'number') hoveredNumber = e.target;
  });
  document.addEventListener('mouseout', function(e) {
    if (e.target && e.target.type === 'number') hoveredNumber = null;
  });
  document.addEventListener('wheel', function(e) {
    var el = hoveredNumber;
    if (!el) return;
    e.preventDefault();
    var step = parseFloat(el.step) || 1;
    var val  = parseFloat(el.value) || 0;
    val += e.deltaY < 0 ? step : -step;
    var mn = el.min !== '' ? parseFloat(el.min) : -Infinity;
    var mx = el.max !== '' ? parseFloat(el.max) :  Infinity;
    el.value = Math.max(mn, Math.min(mx, val));
    el.dispatchEvent(new Event('input', {bubbles:true}));
    el.dispatchEvent(new Event('change', {bubbles:true}));
  }, {passive: false});
})();


// ── DP-Solver Parameter aus UI anwenden ────────────────────────
function dpSolverApplySettings() {
  function gn(id, def) {
    var el = document.getElementById(id);
    return el ? (parseFloat(el.value) || def) : def;
  }
  var wA4    = gn('dp-wA4', 4);
  var wA5    = gn('dp-wA5', 5);
  var wA6    = gn('dp-wA6', 0.25);
  var wSwitch= gn('dp-wSwitch', 5000);
  var copies = Math.round(gn('dp-a6copies', 1));
  var safety = gn('dp-safety', 3);
  var tcptol = gn('dp-tcptol', 2);
  var sing   = gn('dp-singthresh', 0.01);

  DPSolver.settings.wAxes        = [1, 1, 1, wA4, wA5, wA6];
  DPSolver.settings.wConfigSwitch = wSwitch;
  DPSolver.settings.safetyDeg    = safety;
  DPSolver.settings.maxTcpPosError= tcptol;
  DPSolver.settings.minSingularity= sing;

  // A6-Kopien: copies=1 → [-1,0,1], copies=2 → [-2,-1,0,1,2] etc.
  var a6 = [0];
  for (let k = 1; k <= copies; k++) { a6.push(k); a6.push(-k); }
  DPSolver.settings.a6Copies = a6;

  // Sofort neu berechnen
  if (parsedData && parsedData.positions.length > 0) {
    computeIKTable(parsedData.positions);
  }
}


// ── Hochpräzisions IK (mehr Iterationen, kleiner Schritt) ────


// ── Settings-Listener (nach DOM-Ready gebunden) ──────────────
function bindSettingsEvents() {
  function bind(id, evt, fn) {
    var el = document.getElementById(id);
    if (el) el.addEventListener(evt, fn);
  }
  // Farben
  bind('cfg-link-col',  'input',  function(){ robotColor=hexToInt(this.value); buildRobotModel(jointAngles); });
  bind('cfg-link-col',  'change', function(){ robotColor=hexToInt(this.value); buildRobotModel(jointAngles); });
  bind('cfg-joint-col', 'input',  function(){ jointColor=hexToInt(this.value); buildRobotModel(jointAngles); });
  bind('cfg-joint-col', 'change', function(){ jointColor=hexToInt(this.value); buildRobotModel(jointAngles); });
  bind('cfg-tcp-col',   'input',  function(){ tcpColor=hexToInt(this.value);   buildRobotModel(jointAngles); });
  bind('cfg-tcp-col',   'change', function(){ tcpColor=hexToInt(this.value);   buildRobotModel(jointAngles); });
  bind('cfg-path-col',  'input',  function(){ pathCol=hexToInt(this.value); buildScene(parsedData.positions); });
  bind('cfg-path-col',  'change', function(){ pathCol=hexToInt(this.value); buildScene(parsedData.positions); });
  bind('cfg-trace-col',   'input',  function(){ tcpTraceGrp.clear(); tcpTracePoints.length=0; });
  bind('cfg-planned-col', 'input',  function(){ buildScene(parsedData.positions); });
  bind('cfg-planned-col', 'change', function(){ buildScene(parsedData.positions); });
  bind('cfg-show-tool',     'change', function(){ if(this.checked!==showToolMesh)    toggleToolMesh(); });
  bind('cfg-show-pedestal', 'change', function(){ if(this.checked!==showPedestalMesh) togglePedestalMesh(); });
  bind('cfg-trace-col', 'change', function(){ tcpTraceGrp.clear(); tcpTracePoints.length=0; });
  // Sichtbarkeit
  bind('cfg-show-trace',      'change', function(){ showTCPTrace=this.checked; if(!showTCPTrace)tcpTraceGrp.clear(); var b=document.getElementById('btn-tcp-trace'); if(b)b.classList.toggle('on',this.checked); });
  bind('cfg-show-skeleton',   'change', function(){ if(this.checked!==showSkeleton) toggleSkeleton(); });
  bind('cfg-show-stlrobot',   'change', function(){ if(this.checked!==showSTLRobot) toggleSTLRobot(); });
  bind('cfg-show-posframes',  'change', function(){ if(this.checked!==showPosFrames) togglePosFrames(); });
  bind('cfg-show-baseframe',  'change', function(){ if(this.checked!==showBaseFrame) toggleBaseFrame(); });
  bind('cfg-show-axislabels', 'change', function(){ showAxisLabels=this.checked; updateAxisLabelsVisibility(); });
  bind('cfg-show-tcpmarker',  'change', function(){ if(this.checked!==showTCPMarker) toggleTCPMarker(); });
}


// setEditorLang → FormatRegistry (Rückwärtskompatibilität)
function setEditorLang(lang) { FormatRegistry.setActive(lang); }

// ══════════════════════════════════════════════════════
// A-WINKEL DIAGRAMM
// ══════════════════════════════════════════════════════
var _aPlotDists = [];
var _aPlotEdits = {};   // {idx: newA} — geänderte A-Werte
var _aPlotDragging = false;
var _aPlotReach       = null;  // [{ok:[bool,…]}] pro Wegpunkt
var _aPlotReachMid    = null;  // [{ok:[bool,…]}] pro Mittelpunkt
var _aPlotReachAngles = null;  // [[angles|null,…]] IK-Lösung pro Wegpunkt × A-Step
var _aPlotReachSing   = null;  // [[types[],…]] Singularitätstypen pro Wegpunkt × A-Step
var _aPlotScanPending    = false; // Scan nur starten wenn explizit angefordert
var _aPlotSuppressDraw  = false; // aPlotDraw in buildScene unterdrücken
var _aPlotReachScanId = 0;
var _aPlotAutoInserts = []; // [{afterIdx, X,Y,Z,A,B,C}] eingefügte Hilfspunkte
var _aPlotDragIdx  = -1;
var _aPlotML = 56, _aPlotMT = 14, _aPlotMR = 10, _aPlotMB = 30;
var _aPlotAMIN = -360, _aPlotAMAX = 360;

// aPlot-Panel auf Breite des Editor-Panels (.ep) ausrichten
function _aPlotFitWidth() {
  var vp    = document.querySelector('.vp');
  var panel = document.getElementById('aplot-panel');
  if (!vp || !panel) return;
  var r = vp.getBoundingClientRect();
  panel.style.left  = r.left + 'px';
  panel.style.width = r.width + 'px';
  var canvas = document.getElementById('aplot-canvas');
  if (canvas) canvas.width = Math.max(300, Math.round(r.width) - 14);
}

function toggleAPlot() {
  var p = document.getElementById('aplot-panel');
  if (!p) return;
  var show = p.style.display === 'none';
  p.style.display = show ? 'block' : 'none';
  var btn = document.getElementById('btn-aplot');
  if (btn) btn.classList.toggle('on', show);
  if (show) {
    _aPlotFitWidth();
    // Sofort _aPlotPos setzen damit erster Drag funktioniert
    window._aPlotPos = (parsedData && parsedData.positions) ? parsedData.positions : [];
    _aPlotCalcDists(window._aPlotPos);
    aPlotDraw();
  }
}

function _aPlotCalcDists(pos) {
  _aPlotDists = [0];
  for (let i = 1; i < pos.length; i++) {
    var dx = pos[i].X - pos[i-1].X, dy = pos[i].Y - pos[i-1].Y, dz = pos[i].Z - pos[i-1].Z;
    _aPlotDists.push(_aPlotDists[i-1] + Math.sqrt(dx*dx + dy*dy + dz*dz));
  }
}

// ── aPlot Zeichenhilfen ──────────────────────────────────────────────────────

function _aPlotDrawBackground(ctx, ML, MT, CW, CH, W, H) {
  const AMIN = -360, ARNG = 720;
  ctx.fillStyle = '#07111a'; ctx.fillRect(0, 0, W, H);
  ctx.fillStyle = '#0b1925'; ctx.fillRect(ML, MT, CW, CH);
  ctx.font = '10px monospace';
  [-360,-270,-180,-90,0,90,180,270,360].forEach(function(deg) {
    var yp = MT + (1 - (deg - AMIN) / ARNG) * CH;
    var isZero = deg === 0, is180 = Math.abs(deg) === 180;
    ctx.strokeStyle = isZero ? '#1d4060' : '#0d2030';
    ctx.lineWidth = (isZero || is180) ? 1 : 0.5;
    ctx.beginPath(); ctx.moveTo(ML, yp); ctx.lineTo(ML + CW, yp); ctx.stroke();
    ctx.fillStyle = isZero ? '#70b0d0' : is180 ? '#4a8090' : '#2a5060';
    ctx.textAlign = 'right';
    ctx.fillText(deg + '°', ML - 4, yp + 3.5);
  });
  ctx.save();
  ctx.translate(10, MT + CH / 2);
  ctx.rotate(-Math.PI / 2);
  ctx.fillStyle = '#4a8090'; ctx.font = '10px monospace'; ctx.textAlign = 'center';
  ctx.fillText('A (Rz) [°]', 0, 0);
  ctx.restore();
}

function _aPlotDrawVerticalGrid(ctx, ML, MT, CW, CH, pos, total) {
  var xSteps = Math.max(1, Math.min(8, pos.length - 1));
  for (let xs = 0; xs <= xSteps; xs++) {
    var xf  = xs / xSteps;
    var xpx = ML + xf * CW;
    ctx.strokeStyle = '#0d2030'; ctx.lineWidth = 0.5;
    ctx.beginPath(); ctx.moveTo(xpx, MT); ctx.lineTo(xpx, MT + CH); ctx.stroke();
    ctx.fillStyle = '#3a6080'; ctx.textAlign = 'center'; ctx.font = '10px monospace';
    ctx.fillText((xf * total / 1000).toFixed(2) + 'm', xpx, MT + CH + 18);
  }
  ctx.strokeStyle = '#2a5070'; ctx.lineWidth = 1;
  ctx.strokeRect(ML, MT, CW, CH);
}

function _aPlotDrawReachBands(ctx, ML, MT, CW, CH, pos, total) {
  if (!_aPlotReach || !_aPlotReach.length) return;
  const AMIN = -360, ARNG = 720, ASTEP = 6, NSTEPS = Math.round(720 / ASTEP);
  var reachN = Math.min(_aPlotReach.length, pos.length);
  for (let ri = 0; ri < reachN; ri++) {
    var reach = _aPlotReach[ri]; if (!reach) continue;
    var xMid = ML + (_aPlotDists[ri] / total) * CW;
    var xL = ri === 0 ? ML : (ML + (_aPlotDists[ri-1] / total) * CW + xMid) / 2;
    var xR = ri === pos.length-1 ? ML+CW : (xMid + ML + (_aPlotDists[ri+1] / total) * CW) / 2;
    ctx.save();
    ctx.beginPath(); ctx.rect(xL, MT, xR - xL, CH); ctx.clip();
    for (let si = 0; si < NSTEPS; si++) {
      if (reach[si]) continue;
      var yTop = MT + (1 - (-360 + (si+1)*ASTEP - AMIN) / ARNG) * CH;
      var yBot = MT + (1 - (-360 + si*ASTEP   - AMIN) / ARNG) * CH;
      ctx.fillStyle = 'rgba(200,160,0,0.18)';
      ctx.fillRect(xL, yTop, xR - xL, yBot - yTop);
    }
    ctx.restore();
  }
  ctx.fillStyle = 'rgba(200,160,0,0.5)'; ctx.fillRect(ML+4, MT+4, 10, 8);
  ctx.fillStyle = '#aaa080'; ctx.font = '9px monospace'; ctx.textAlign = 'left';
  ctx.fillText('keine IK-Lösung', ML+17, MT+11);
}

function _aPlotDrawCurves(ctx, ML, MT, CW, CH, pos, total) {
  const AMIN = -360, ARNG = 720;
  var xp = function(i) { return ML + (_aPlotDists[i] / total) * CW; };
  var yp = function(a) { return MT + (1 - (a - AMIN) / ARNG) * CH; };
  // Originallinie (orange)
  ctx.beginPath(); ctx.strokeStyle = '#f05500'; ctx.lineWidth = 2;
  pos.forEach(function(p, i) { i===0 ? ctx.moveTo(xp(i),yp(p.A)) : ctx.lineTo(xp(i),yp(p.A)); });
  ctx.stroke();
  // Wegpunkte
  pos.forEach(function(p, i) {
    var a = (_aPlotEdits[i] !== undefined) ? _aPlotEdits[i] : p.A;
    ctx.fillStyle = (_aPlotEdits[i] !== undefined) ? '#ffee00' : '#ff8800';
    ctx.beginPath(); ctx.arc(xp(i), yp(a), 4, 0, 2*Math.PI); ctx.fill();
  });
  // Editierte Linie (gelb gestrichelt)
  if (Object.keys(_aPlotEdits).length > 0) {
    ctx.beginPath(); ctx.strokeStyle = '#ffee00'; ctx.lineWidth = 1.5; ctx.setLineDash([4,3]);
    pos.forEach(function(p, i) {
      var a = (_aPlotEdits[i] !== undefined) ? _aPlotEdits[i] : p.A;
      i===0 ? ctx.moveTo(xp(i),yp(a)) : ctx.lineTo(xp(i),yp(a));
    });
    ctx.stroke(); ctx.setLineDash([]);
  }
}

function _aPlotDrawSingMarkers(ctx, ML, MT, CW, CH, pos, total) {
  if (typeof classifySingTypes !== 'function' || !pos.length) return;
  const AMIN = -360, ARNG = 720;
  const SING_LCOL = { wrist:'#ffb400', shoulder:'#dc3232', elbow:'#32b4dc' };
  const SING_LBL  = { wrist:'Handgelenk', shoulder:'Schulter', elbow:'Ellbogen' };
  ctx.save();
  var legendTypes = {};

  // Reachability-Scan: singuläre erreichbare A-Werte
  if (_aPlotReach && _aPlotReach.length === pos.length &&
      typeof _aPlotReachAngles !== 'undefined' && _aPlotReachAngles) {
    var ASTEP = 6, NSTEPS = Math.round(720 / ASTEP);
    for (let ri = 0; ri < pos.length; ri++) {
      var xMid = ML + (_aPlotDists[ri] / total) * CW;
      var xL = ri===0 ? ML : (ML+(_aPlotDists[ri-1]/total)*CW + xMid)/2;
      var xR = ri===pos.length-1 ? ML+CW : (xMid + ML+(_aPlotDists[ri+1]/total)*CW)/2;
      for (let si = 0; si < NSTEPS; si++) {
        if (!_aPlotReach[ri][si]) continue;
        var ang = _aPlotReachAngles[ri] && _aPlotReachAngles[ri][si];
        if (!ang) continue;
        var stypes = classifySingTypes(ang);
        if (!stypes.length) continue;
        var yp = MT + (1 - (-360 + si*ASTEP - AMIN) / ARNG) * CH;
        stypes.forEach(function(t) {
          legendTypes[t] = true;
          ctx.fillStyle = SING_LCOL[t]; ctx.globalAlpha = 0.7;
          ctx.beginPath(); ctx.arc((xL+xR)/2, yp, 2.5, 0, 2*Math.PI); ctx.fill();
        });
      }
    }
    ctx.globalAlpha = 1;
  }

  // Aktueller Pfad (ikTable): Raute-Symbole
  if (typeof ikTable !== 'undefined' && ikTable.length > 0) {
    var singN = Math.min(ikTable.length, pos.length);
    for (let i = 0; i < singN; i++) {
      var ik = ikTable[i]; if (!ik || !ik.angles) continue;
      var stypes = classifySingTypes(ik.angles);
      if (!stypes.length) continue;
      var xpS = ML + (_aPlotDists[i] / total) * CW;
      var ypS = MT + (1 - (pos[i].A - AMIN) / ARNG) * CH;
      stypes.forEach(function(t) {
        legendTypes[t] = true;
        var r = 6;
        ctx.fillStyle = SING_LCOL[t]; ctx.strokeStyle = '#000'; ctx.lineWidth = 0.5;
        ctx.beginPath();
        ctx.moveTo(xpS, ypS-r); ctx.lineTo(xpS+r, ypS);
        ctx.lineTo(xpS, ypS+r); ctx.lineTo(xpS-r, ypS);
        ctx.closePath(); ctx.fill(); ctx.stroke();
      });
    }
  }

  // Legende
  ctx.font = '9px monospace';
  var lx = ML+CW-4, ly = MT+CH-4;
  ['elbow','shoulder','wrist'].filter(function(t){ return legendTypes[t]; })
    .forEach(function(t, i) {
      var yy = ly - i*13;
      ctx.fillStyle = SING_LCOL[t]; ctx.globalAlpha = 0.85;
      ctx.beginPath();
      ctx.moveTo(lx-78,yy-6); ctx.lineTo(lx-74,yy-2);
      ctx.lineTo(lx-78,yy+2); ctx.lineTo(lx-82,yy-2);
      ctx.closePath(); ctx.fill(); ctx.globalAlpha = 1;
      ctx.fillStyle = SING_LCOL[t]; ctx.textAlign = 'left';
      ctx.fillText(SING_LBL[t], lx-70, yy);
    });
  ctx.restore();
}

function _aPlotDrawCursor(ctx, ML, MT, CW, CH, pos, total) {
  if (typeof sim === 'undefined' || pos.length < 2) return;
  const AMIN = -360, ARNG = 720;
  var N  = pos.length;
  var i0 = Math.max(0, Math.min(N-1, Math.floor(sim.t)));
  var i1 = Math.min(i0+1, N-1);
  var fr = sim.t - i0;
  var cd = _aPlotDists[i0] + (_aPlotDists[i1] - _aPlotDists[i0]) * fr;
  var ca = pos[i0].A + (pos[i1].A - pos[i0].A) * fr;
  var cx = ML + (cd / total) * CW;
  var cy = MT + (1 - (ca - AMIN) / ARNG) * CH;
  ctx.strokeStyle = 'rgba(0,200,255,0.55)'; ctx.lineWidth = 1; ctx.setLineDash([4,3]);
  ctx.beginPath(); ctx.moveTo(cx, MT); ctx.lineTo(cx, MT+CH); ctx.stroke();
  ctx.setLineDash([]);
  ctx.fillStyle = '#00ccff';
  ctx.beginPath(); ctx.arc(cx, cy, 4, 0, 2*Math.PI); ctx.fill();
  var info = document.getElementById('aplot-info');
  if (info) info.textContent = 'A = ' + ca.toFixed(2) + '°  ·  Pos ' + (i0+1) + '/' + N + '  ·  ' + (cd/1000).toFixed(3) + ' m';
}

// ── aPlotDraw: Koordinator ───────────────────────────────────────────────────
function aPlotDraw() {
  var canvas = document.getElementById('aplot-canvas');
  var panel  = document.getElementById('aplot-panel');
  if (!canvas || !panel || panel.style.display === 'none') return;

  var W = canvas.width, H = canvas.height;
  var ML=56, MT=14, MR=10, MB=30;
  var CW = W-ML-MR, CH = H-MT-MB;
  var ctx = canvas.getContext('2d');
  ctx.clearRect(0, 0, W, H);

  _aPlotDrawBackground(ctx, ML, MT, CW, CH, W, H);

  var pos = (parsedData && parsedData.positions) ? parsedData.positions : [];
  if (!pos.length) {
    ctx.fillStyle = '#3a6080'; ctx.font = '13px monospace'; ctx.textAlign = 'center';
    ctx.fillText('Kein Programm geladen', W/2, H/2);
    return;
  }

  _aPlotCalcDists(pos);
  var total = _aPlotDists[_aPlotDists.length-1] || 1;
  _aPlotML=ML; _aPlotMT=MT; window._aPlotCW=CW; window._aPlotCH=CH;
  window._aPlotTotal=total; window._aPlotPos=pos;

  _aPlotDrawVerticalGrid(ctx, ML, MT, CW, CH, pos, total);
  _aPlotDrawReachBands(ctx, ML, MT, CW, CH, pos, total);
  _aPlotDrawCurves(ctx, ML, MT, CW, CH, pos, total);
  _aPlotDrawSingMarkers(ctx, ML, MT, CW, CH, pos, total);
  _aPlotDrawCursor(ctx, ML, MT, CW, CH, pos, total);
}

// Map Canvas Drag: A-Wert eines Punktes ändern
(function() {
  function getCanvas() { return document.getElementById('aplot-canvas'); }
  function nearestPoint(mx) {
    var pos = window._aPlotPos; if (!pos || !pos.length) return -1;
    var CW = window._aPlotCW||600, total = window._aPlotTotal||1;
    var best = -1, bestD = 20;
    pos.forEach(function(p,i) {
      var xp = _aPlotML + (_aPlotDists[i] / total) * CW;
      var d = Math.abs(mx - xp);
      if (d < bestD) { bestD = d; best = i; }
    });
    return best;
  }
  function yToA(my) {
    var CH = window._aPlotCH||180;
    var a = _aPlotAMIN + (1 - (my - _aPlotMT) / CH) * (_aPlotAMAX - _aPlotAMIN);
    return Math.max(_aPlotAMIN, Math.min(_aPlotAMAX, a));
  }
  document.addEventListener('DOMContentLoaded', function() {
    var c = getCanvas(); if (!c) return;
    c.style.cursor = 'ns-resize';
    c.addEventListener('mousedown', function(e) {
      var r = c.getBoundingClientRect();
      var mx = e.clientX - r.left;
      // Sicherstellen dass _aPlotPos und Distanzen aktuell sind
      window._aPlotPos = (parsedData && parsedData.positions) ? parsedData.positions : [];
      if (_aPlotDists.length !== window._aPlotPos.length) _aPlotCalcDists(window._aPlotPos);
      var idx = nearestPoint(mx);
      if (idx < 0) return;
      _aPlotDragging = true; _aPlotDragIdx = idx;
      e.preventDefault();
    });
    document.addEventListener('mousemove', function(e) {
      if (!_aPlotDragging || _aPlotDragIdx < 0) return;
      var c2 = getCanvas(); if (!c2) return;
      var r = c2.getBoundingClientRect();
      var my = e.clientY - r.top;
      var newA = Math.round(yToA(my) * 10) / 10;
      _aPlotEdits[_aPlotDragIdx] = newA;
      var hint = document.getElementById('aplot-edit-hint');
      if (hint) hint.style.display = '';
      aPlotDraw();
      aPlotLivePreview(_aPlotDragIdx, newA);
      // Status aktualisieren
      var info = document.getElementById('aplot-info');
      if (info) info.textContent = 'P' + (_aPlotDragIdx+1) + '  A = ' + newA.toFixed(1) + '°  ✎ ziehen';
    });
    document.addEventListener('mouseup', function() {
      _aPlotDragging = false; _aPlotDragIdx = -1;
    });
  });
})();

function aPlotLivePreview(idx, newA) {
  var pos = (parsedData && parsedData.positions) ? parsedData.positions : [];
  window._aPlotPos = pos;
  if (!pos[idx]) return;
  // Wenn Formular aktiv: Karte öffnen
  if (typeof FormatRegistry !== 'undefined' && FormatRegistry.getActiveId() === 'kuka-form'
      && typeof fvBuild === 'function' && pos[idx].lineNum !== undefined) {
    if (fvExpandedLine !== pos[idx].lineNum) {
      fvExpandedLine = pos[idx].lineNum;
      fvBuild(pos[idx].lineNum);
    }
  }
  var p = pos[idx];
  // Warm-Start per segIdx
  var warmQ = jointAngles.slice();
  if (trajectory && trajectory.length) {
    for (let ti = trajectory.length-1; ti >= 0; ti--) {
      if (trajectory[ti].segIdx === idx) { warmQ = trajectory[ti].angles; break; }
    }
  }
  // Hochpräzisions-IK: nur A ändert sich, XYZ BC exakt aus Programm
  var res = solveIKPrecise(p.X, p.Y, p.Z, newA, p.B, p.C, warmQ);
  if (res && res.ok) applyAngles(res.angles);
}

// Reachability-Scan: Wegpunkte + Mittelpunkte, 6°-Schritte
function aPlotStartScan() {
  // Scan-Daten zurücksetzen und neu starten
  _aPlotReach = null; _aPlotReachMid = null;
  _aPlotReachAngles = null; _aPlotReachSing = null;
  aPlotScanReachability();
}

function aPlotScanReachability() {
  var pos = (parsedData && parsedData.positions) ? parsedData.positions : [];
  if (!pos.length) { _aPlotReach = null; _aPlotReachMid = null; return; }
  _aPlotReach = null; _aPlotReachMid = null; _aPlotReachAngles = null;
  var ASTEP = 6, NSTEPS = Math.round(720 / ASTEP);
  var scanId = ++_aPlotReachScanId;
  var wpResult  = [], midResult = [], wpAngles = [], wpSing = [];

  // Fortschrittsanzeige einblenden
  var overlay = document.getElementById('aplot-scan-overlay');
  var bar     = document.getElementById('aplot-scan-bar');
  var pctEl   = document.getElementById('aplot-scan-pct');
  var msgEl   = document.getElementById('aplot-scan-msg');
  if (overlay) overlay.style.display = 'flex';

  // Scan-Positionen: wp0, mid01, wp1, mid12, …, wp[N-1]
  var scanList = [];
  for (let i = 0; i < pos.length; i++) {
    scanList.push({ type:'wp', idx:i, p: pos[i] });
    if (i < pos.length - 1) {
      var p0 = pos[i], p1 = pos[i+1];
      scanList.push({ type:'mid', idx:i, subIdx:0, p:{
        X:(p0.X+p1.X)/2, Y:(p0.Y+p1.Y)/2, Z:(p0.Z+p1.Z)/2,
        B:(p0.B+p1.B)/2, C:(p0.C+p1.C)/2
      }});
    }
  }
  var totalSteps = scanList.length;

  var si = 0;
  function scanNext() {
    if (scanId !== _aPlotReachScanId) {
      if (overlay) overlay.style.display = 'none';
      return;
    }
    if (si >= scanList.length) {
      _aPlotReach       = wpResult;
      _aPlotReachMid    = midResult;
      _aPlotReachAngles = wpAngles;
      _aPlotReachSing   = wpSing;
      if (overlay) overlay.style.display = 'none';
      aPlotDraw();
      return;
    }
    // Fortschritt aktualisieren
    var pct = Math.round(si / totalSteps * 100);
    if (bar)   bar.style.width   = pct + '%';
    if (pctEl) pctEl.textContent = pct + '%';
    if (msgEl) msgEl.textContent = 'Scan ' + (si+1) + ' / ' + totalSteps + ' Positionen…';

    var item = scanList[si];
    var p = item.p;
    var initQ  = jointAngles.slice();
    var reach  = new Array(NSTEPS);
    var angles = (item.type === 'wp') ? new Array(NSTEPS) : null;
    var sing   = (item.type === 'wp') ? new Array(NSTEPS) : null;
    for (let s = 0; s < NSTEPS; s++) {
      var aTest = -360 + s * ASTEP;
      var res = solveIKPrecise(p.X, p.Y, p.Z, aTest, p.B, p.C, initQ);
      reach[s] = !!(res && res.ok);
      if (reach[s]) {
        initQ = res.angles;
        if (angles) angles[s] = res.angles.slice();
        if (sing)   sing[s]   = classifySingTypes(res.angles);
      } else {
        if (angles) angles[s] = null;
        if (sing)   sing[s]   = [];
      }
    }
    if (item.type === 'wp') { wpResult.push(reach); wpAngles.push(angles); wpSing.push(sing); }
    else                      midResult.push(reach);
    si++;
    setTimeout(scanNext, 0);
  }
  setTimeout(scanNext, 0);
}

// ── Auto-Lösung: erweitertes DP mit optionalen Mittelpunkt-Hilfspunkten ──
// Knotenfolge: wp0, mid01, wp1, mid12, …, wp[N-1]
// Mittelpunkte sind OPTIONAL: Bypass kostet 0 wenn direkte Interpolation erreichbar,
// sonst wird ein Hilfspunkt mit C_DIS eingefügt.
function aPlotAutoSolve() {
  var infoEl = document.getElementById('aplot-info');
  if (!_aPlotReach || !_aPlotReachMid) {
    if (infoEl) infoEl.textContent = 'Scan läuft noch — bitte warten…'; return;
  }
  var pos = (parsedData && parsedData.positions) ? parsedData.positions : [];
  if (!pos.length) { if (infoEl) infoEl.textContent = 'Kein Programm geladen.'; return; }
  if (_aPlotReach.length !== pos.length) {
    if (infoEl) infoEl.textContent = 'Scan veraltet — bitte neu parsen.'; return;
  }

  var ASTEP = 6, NSTEPS = Math.round(720 / ASTEP), INF = 1e9;
  var N = pos.length;

  // ── Knotenfolge aufbauen ──────────────────────────────
  // nodes[k] = { type:'wp'|'mid', idx, subIdx, reach[NSTEPS] }
  // NSUB Zwischenpunkte pro Segment (t=1/(NSUB+1) … NSUB/(NSUB+1))
  var nodes = [];
  for (let i = 0; i < N; i++) {
    nodes.push({ type:'wp', idx:i, reach:_aPlotReach[i] });
    if (i < N-1)
      nodes.push({ type:'mid', idx:i, subIdx:0, reach:(_aPlotReachMid[i] || _aPlotReach[i]) });
  }
  var M = nodes.length; // 2N-1

  // ── DP: dp[k][s] = {cost, prev_k, prev_s} ────────────
  // Für Mittelpunkte: Zustand NSTEPS = "Bypass" (kein Hilfspunkt)
  var BYPASS = NSTEPS; // extra Zustand
  var dp = [];
  for (let k = 0; k < M; k++) {
    var states = (nodes[k].type === 'mid') ? NSTEPS + 1 : NSTEPS;
    dp.push(new Array(states));
    for (let s = 0; s < states; s++) dp[k][s] = { cost: INF, pk: -1, ps: -1 };
  }

  // Hilfsfunktionen für DP-Filterung
  var BUFFER_PENALTY = 120;
  var BUFFER_WIDTH   = 3;  // 18° Abstand zu gelben Bereichen

  // Ist ein WP-Schritt singulär? → komplett sperren
  function isSingStep(wpIdx, stepIdx) {
    if (!_aPlotReachSing || !_aPlotReachSing[wpIdx]) return false;
    var s = _aPlotReachSing[wpIdx][stepIdx];
    return s && s.length > 0;
  }

  // Pufferkosten: Abstand zu nicht-erreichbaren Nachbarn
  function bufferCost(reach, stepIdx) {
    if (!reach) return 0;
    for (let b = 1; b <= BUFFER_WIDTH; b++) {
      var lo = stepIdx-b, hi = stepIdx+b;
      if ((lo>=0 && !reach[lo]) || (hi<reach.length && !reach[hi]))
        return BUFFER_PENALTY / b;
    }
    return 0;
  }

  function singCost(nodeIdx, stepIdx) {
    // Nur Pufferkosten — Singularität wird durch isSingStep gefiltert
    return bufferCost(nodes[nodeIdx].reach, stepIdx);
  }

  // Startpunkt wp[0]
  var startA = pos[0].A;
  for (let s = 0; s < NSTEPS; s++) {
    if (!nodes[0].reach[s]) continue;
    if (isSingStep(0, s)) continue;
    dp[0][s].cost = Math.abs((-360 + s*ASTEP) - startA) + bufferCost(nodes[0].reach, s);
  }

  // DP vorwärts
  for (let k = 1; k < M; k++) {
    var nd  = nodes[k];
    var pkv = k - 1;

    if (nd.type === 'wp') {
      // Von Mittelpunkt (k-1) zu Wegpunkt (k)
      var midNd = nodes[pkv];
      for (let s = 0; s < NSTEPS; s++) {          // Ziel wp
        if (!nd.reach[s]) continue;
        if (isSingStep(nd.idx, s)) continue;       // singuläre Schritte sperren
        var aTo = -360 + s*ASTEP;
        var sCost = bufferCost(nd.reach, s);
        // Aus realen Mittelpunkt-Zuständen
        for (let ps = 0; ps < NSTEPS; ps++) {
          if (dp[pkv][ps].cost >= INF) continue;
          if (!midNd.reach[ps]) continue;
          var cost = dp[pkv][ps].cost + Math.abs(aTo - (-360+ps*ASTEP)) + sCost;
          if (cost < dp[k][s].cost) { dp[k][s] = {cost, pk:pkv, ps}; }
        }
        // Aus Bypass-Zustand
        if (dp[pkv][BYPASS].cost < INF) {
          var cost = dp[pkv][BYPASS].cost + Math.abs(aTo - dp[pkv][BYPASS]._bypA) + sCost;
          if (cost < dp[k][s].cost) { dp[k][s] = {cost, pk:pkv, ps:BYPASS}; }
        }
      }
    } else {
      // Von Wegpunkt (k-1) zu Mittelpunkt (k)
      var wpNd = nodes[pkv];
      for (let ps = 0; ps < NSTEPS; ps++) {       // Quell wp
        if (dp[pkv][ps].cost >= INF) continue;
        if (!wpNd.reach[ps]) continue;
        var aFrom = -360 + ps*ASTEP;
        // → reale Mittelpunkt-Zustände
        for (let s = 0; s < NSTEPS; s++) {
          if (!nd.reach[s]) continue;
          var aTo   = -360 + s*ASTEP;
          var cost  = dp[pkv][ps].cost + Math.abs(aTo - aFrom);
          if (cost < dp[k][s].cost) { dp[k][s] = {cost, pk:pkv, ps}; }
        }
        // → Bypass: kostenfrei, aber nur wenn aFrom erreichbar am Mittelpunkt
        // Bypass-A = aFrom (wird linear weitergegeben)
        var midStep = Math.round((aFrom + 360) / ASTEP);
        midStep = Math.max(0, Math.min(NSTEPS-1, midStep));
        var bypOk = nd.reach[midStep]; // erreichbar am Mittelpunkt?
        // Puffer-Check für Bypass
        var bypBuf = 0;
        for (let _b = 1; _b <= BUFFER_WIDTH; _b++) {
          var _lo = midStep-_b, _hi = midStep+_b;
          if ((_lo>=0 && !nd.reach[_lo])||(_hi<nd.reach.length&&!nd.reach[_hi])) { bypBuf=BUFFER_PENALTY/_b; break; }
        }
        var bypCost = dp[pkv][ps].cost + (bypOk ? bypBuf : 300); // hohe Strafe wenn gelb
        if (bypCost < dp[k][BYPASS].cost) {
          dp[k][BYPASS] = {cost:bypCost, pk:pkv, ps, _bypA:aFrom};
        }
      }
    }
  }

  // Bestes Ende (letzter Wegpunkt)
  var lastK = M - 1;
  var best = -1, bestCost = INF;
  for (let s = 0; s < NSTEPS; s++) {
    if (dp[lastK][s].cost < bestCost) { bestCost = dp[lastK][s].cost; best = s; }
  }
  if (best < 0) {
    // Fallback: ohne Singularitätssperre nochmal versuchen
    for (let s = 0; s < NSTEPS; s++) {
      if (!nodes[0].reach[s]) continue;
      dp[0][s].cost = Math.abs((-360+s*ASTEP)-startA);
    }
    for (let k = 1; k < M; k++) { /* reset */ for (let s2=0;s2<dp[k].length;s2++) dp[k][s2]={cost:INF,pk:-1,ps:-1}; }
    if (infoEl) infoEl.textContent = '⚠ Keine singularitätsfreie Lösung — zeige beste verfügbare Bahn';
    return;
  }

  // ── Pfad zurückverfolgen ──────────────────────────────
  var path = new Array(M);
  path[lastK] = best;
  for (let k = M-2; k >= 0; k--) {
    var nxt = path[k+1];
    path[k] = dp[k+1][nxt].ps;
  }

  // ── Ergebnis extrahieren ──────────────────────────────
  _aPlotEdits = {};
  _aPlotAutoInserts = [];

  for (let k = 0; k < M; k++) {
    var nd = nodes[k];
    if (nd.type === 'wp') {
      var aNew = -360 + path[k] * ASTEP;
      if (Math.abs(aNew - pos[nd.idx].A) > 0.5) _aPlotEdits[nd.idx] = aNew;
    }
    // Keine Hilfspunkte einfügen — vermeidet Bahnabweichungen durch C_DIS-Rundung
  }

  var hint = document.getElementById('aplot-edit-hint');
  if (hint) hint.style.display = '';
  var insStr = _aPlotAutoInserts.length ? '  +' + _aPlotAutoInserts.length + ' Hilfspunkt(e)' : '';
  if (infoEl) infoEl.textContent =
    '⚡ Auto-Lösung: Δ' + bestCost.toFixed(0) + '°' + insStr + ' — Übernehmen zum Anwenden';
  aPlotDraw();
}

function aPlotReset() {
  _aPlotEdits = {};
  _aPlotAutoInserts = [];
  var hint = document.getElementById('aplot-edit-hint');
  if (hint) hint.style.display = 'none';
  aPlotDraw();
}

function aPlotApply() {
  var pos = (parsedData && parsedData.positions) ? parsedData.positions : [];
  var hasEdits   = Object.keys(_aPlotEdits).length > 0;
  var hasInserts = _aPlotAutoInserts && _aPlotAutoInserts.length > 0;
  if (!pos.length || (!hasEdits && !hasInserts)) return;
  var code = document.getElementById('code-input');
  if (!code) return;
  var nl = '\n';
  var lines = code.value.split(nl);

  // ── 1. A-Werte an bestehenden Wegpunkten ändern ───────
  // Baue Index: Wegpunkt[i] → Zeilennummer (nur LIN/CIRC/SLIN/PTP)
  var wpLineMap = {};
  var linCount = 0;
  for (let li = 0; li < lines.length; li++) {
    if (lines[li].match(/^\s*(LIN|SLIN|CIRC)\s*[{]/i)) {
      wpLineMap[linCount] = li;
      linCount++;
    }
  }
  Object.keys(_aPlotEdits).forEach(function(idxStr) {
    var idx = parseInt(idxStr);
    var newA = _aPlotEdits[idx];
    var li = wpLineMap[idx];
    if (li === undefined) return;
    lines[li] = lines[li].replace(/,\s*A\s*([\-\d\.]+)/, ', A ' + newA.toFixed(3));
  });

  // ── 2. Hilfspunkte einfügen (rückwärts, damit Indizes stabil bleiben) ──
  if (hasInserts) {
    // Sortieren: höchster afterWpIdx zuerst
    var inserts = _aPlotAutoInserts.slice().sort(function(a,b){ return b.afterWpIdx !== a.afterWpIdx ? b.afterWpIdx-a.afterWpIdx : b.subIdx-a.subIdx; });
    inserts.forEach(function(ins) {
      var li = wpLineMap[ins.afterWpIdx];
      if (li === undefined) return;
      // Verschleifung vom Originalpunkt übernehmen
      var origLine = lines[li] || '';
      var verlM = origLine.match(/(C_DIS|C_PTP|C_VEL|C_ORI)\s*$/);
      var verl = verlM ? ' ' + verlM[1] : ' C_DIS';
      var hlp = 'LIN {X ' + ins.X.toFixed(3) + ', Y ' + ins.Y.toFixed(3) +
                ', Z ' + ins.Z.toFixed(3) + ', A ' + ins.A.toFixed(3) +
                ', B ' + ins.B.toFixed(3) + ', C ' + ins.C.toFixed(3) + '}' + verl +
                '  ; AUTO-HILFSPUNKT';
      lines.splice(li + 1, 0, hlp);
    });
  }

  code.value = lines.join(nl);
  _aPlotEdits = {};
  _aPlotAutoInserts = [];
  var hint = document.getElementById('aplot-edit-hint');
  if (hint) hint.style.display = 'none';
  var info = document.getElementById('aplot-info');
  parseAndLoad();
  if (info) info.textContent = '✓ Übernommen';
}

// Drag-Handler fuer aplot-panel
(function() {
  var hdr = null, panel = null, ox = 0, oy = 0, px = 0, py = 0;
  document.addEventListener('DOMContentLoaded', function() {
    hdr   = document.getElementById('aplot-hdr');
    panel = document.getElementById('aplot-panel');
    if (!hdr || !panel) return;
    hdr.addEventListener('mousedown', function(e) {
      var r = panel.getBoundingClientRect();
      ox = e.clientX - r.left; oy = e.clientY - r.top;
      document.addEventListener('mousemove', onMove);
      document.addEventListener('mouseup', onUp);
    });
    function onMove(e) {
      panel.style.left   = (e.clientX - ox) + 'px';
      panel.style.top    = (e.clientY - oy) + 'px';
      panel.style.bottom = 'auto';
    }
    function onUp() {
      document.removeEventListener('mousemove', onMove);
      document.removeEventListener('mouseup', onUp);
    }
  });
})();

splashProgress(80, 'Programm wird geparst…');
parseAndLoad();


// STL nach vollständigem Laden der Seite (inkl. Three.js CDN)
// STL wird manuell per '↺ STL' Button geladen
// Settings nach vollständigem DOM-Load anwenden
window.addEventListener('load', function() {
  splashProgress(95, 'Einstellungen werden geladen…');
  initSettings();
  bindSettingsEvents();
  // Theme beim Start anwenden
  document.body.classList.add('bg-pro');
  setGridColor(0x2d2d30);
  splashProgress(100, 'Bereit.');
  setTimeout(function() {
    splashHide();
    // STL-Dateien werden nur auf Klick geladen (Anlage laden)
    // autoLoadSTLFiles();
  }, 400);

  // Map-Button nutzt toggleAPlot() (enthält _aPlotFitWidth)
  var _abtn = document.getElementById('btn-aplot');
  if (_abtn) _abtn.onclick = toggleAPlot;

  // Versionsnummer setzen
  ['ver-header','ver-help','ver-splash'].forEach(function(id) {
    var el = document.getElementById(id);
    if (el) el.textContent = APP_VERSION;
  });
  document.title = 'RobSimul Simple ' + APP_VERSION + ' · cnc-technik.de';

  // Format-Button initialisieren + Formular als Default
  FormatRegistry.initButton();
  FormatRegistry.setActive('kuka');
  if (typeof fvBuild === 'function') fvBuild(-1);
});




// ═══════════════════════════════════════════════════
// FORMAT SAVE / LOAD
// ═══════════════════════════════════════════════════

var _FMT_EXT = {
  'kuka-form': '.pfg.xml', 'kuka': '.src', 'abb': '.mod', 'fanuc': '.ls',
  'yaskawa': '.jbi', 'kawasaki': '.pg', 'staubli': '.pgx', 'ur': '.script',
  'adept': '.v2', 'omron': '.v2', 'epson': '.prg', 'comau': '.pdl',
  'aubo': '.aubo', 'dobot': '.dobot', 'denso': '.pcs', 'nachi': '.fdr',
  'hanwha': '.txt', 'igus': '.txt', 'estun': '.txt', 'neura': '.cpp', 'mabi': '.txt'
};

// ── Formular → XML ─────────────────────────────────────────────────
function parsedDataToXml(pd) {
  if (!pd) return '';
  var lines = ['<?xml version="1.0" encoding="UTF-8"?>'];
  lines.push('<PFGProgram version="1.0">');

  // Positions
  lines.push('  <Positions>');
  (pd.positions || []).forEach(function(p, i) {
    var pr = p.rel || p; // Code-Koordinaten exportieren
    lines.push('    <Pos idx="' + i + '" type="' + (p.type||'LIN') + '"' +
      ' X="' + pr.X.toFixed(4) + '" Y="' + pr.Y.toFixed(4) + '" Z="' + pr.Z.toFixed(4) +
      '" A="' + pr.A.toFixed(4) + '" B="' + pr.B.toFixed(4) + '" C="' + pr.C.toFixed(4) +
      '" velCP="' + (p.velCP || 0.167).toFixed(4) + '"' +
      (p.S != null ? ' S="' + p.S + '"' : '') +
      (p.T != null ? ' T="' + p.T + '"' : '') + '/>');
  });
  lines.push('  </Positions>');

  // Steps
  lines.push('  <Steps>');
  (pd.steps || []).forEach(function(s) {
    var esc = function(v) { return String(v||'').replace(/&/g,'&amp;').replace(/"/g,'&quot;'); };
    switch (s.type) {
      case 'tool':    lines.push('    <Tool n="' + s.n + '"/>'); break;
      case 'base':    lines.push('    <Base n="' + s.n + '"/>'); break;
      case 'velcp':   lines.push('    <VelCP v="' + (s.v||0.167).toFixed(4) + '"/>'); break;
      case 'velptp':  lines.push('    <VelPTP v="' + (s.v||1).toFixed(2) + '"/>'); break;
      case 'var':     lines.push('    <Var type="' + esc(s.varType||'REAL') + '" name="' + esc(s.name) + '" value="' + esc(s.val||'') + '"/>'); break;
      case 'move':    lines.push('    <Move posIdx="' + s.posIdx + '" moveType="' + (s.moveType||'LIN') + '" label="' + (s.label||'') + '"/>'); break;
      case 'circ':    lines.push('    <Circ viaIdx="' + s.viaIdx + '" posIdx="' + s.posIdx + '"/>'); break;
      case 'halt':    lines.push('    <Halt/>'); break;
      case 'brake':   lines.push('    <Brake/>'); break;
      case 'wait':    lines.push('    <Wait t="' + (s.t||0) + '"/>'); break;
      case 'waitFor': lines.push('    <WaitFor cond="' + esc(s.cond||'') + '"/>'); break;
      case 'din':     lines.push('    <DIn n="' + s.n + '"/>'); break;
      case 'dout':    lines.push('    <DOut n="' + s.n + '" v="' + esc(s.v||'FALSE') + '"/>'); break;
      case 'ain':     lines.push('    <AIn n="' + s.n + '"/>'); break;
      case 'aout':    lines.push('    <AOut n="' + s.n + '" v="' + (s.v||0) + '"/>'); break;
      case 'calc':    lines.push('    <Calc target="' + esc(s.target||'') + '" expr="' + esc(s.expr||'') + '"/>'); break;
      case 'comment': lines.push('    <Comment text="' + esc(s.text||'') + '"/>'); break;
      case 'ptpAxis': lines.push('    <PTPAxis angles="' + (s.angles||[]).join(',') + '"/>'); break;
      default:        if (s.raw) lines.push('    <Other raw="' + esc(s.raw) + '"/>'); break;
    }
  });
  lines.push('  </Steps>');
  lines.push('</PFGProgram>');
  return lines.join('\n');
}

// ── XML → parsedData ────────────────────────────────────────────────
function xmlToParsedData(xmlText) {
  var parser = new DOMParser();
  var doc = parser.parseFromString(xmlText, 'application/xml');
  var positions = [], steps = [];

  var posEls = doc.querySelectorAll('Positions > Pos');
  posEls.forEach(function(el) {
    positions.push({
      type:  el.getAttribute('type') || 'LIN',
      X:     parseFloat(el.getAttribute('X')) || 0,
      Y:     parseFloat(el.getAttribute('Y')) || 0,
      Z:     parseFloat(el.getAttribute('Z')) || 0,
      A:     parseFloat(el.getAttribute('A')) || 0,
      B:     parseFloat(el.getAttribute('B')) || 0,
      C:     parseFloat(el.getAttribute('C')) || 0,
      velCP: parseFloat(el.getAttribute('velCP')) || 0.167,
      S:     el.hasAttribute('S') ? parseInt(el.getAttribute('S')) : null,
      T:     el.hasAttribute('T') ? parseInt(el.getAttribute('T')) : null,
    });
  });

  var stepEls = doc.querySelectorAll('Steps > *');
  stepEls.forEach(function(el) {
    var tag = el.tagName;
    switch (tag) {
      case 'Tool':    steps.push({type:'tool',   n: parseInt(el.getAttribute('n'))||1}); break;
      case 'Base':    steps.push({type:'base',   n: parseInt(el.getAttribute('n'))||1}); break;
      case 'VelCP':   steps.push({type:'velcp',  v: parseFloat(el.getAttribute('v'))||0.167}); break;
      case 'VelPTP':  steps.push({type:'velptp', v: parseFloat(el.getAttribute('v'))||1}); break;
      case 'Var':     steps.push({type:'var',    varType: el.getAttribute('type')||'REAL', name: el.getAttribute('name')||'v', val: el.getAttribute('value')||''}); break;
      case 'Move':    steps.push({type:'move',   posIdx: parseInt(el.getAttribute('posIdx'))||0, moveType: el.getAttribute('moveType')||'LIN', label: el.getAttribute('label')||''}); break;
      case 'Circ':    steps.push({type:'circ',   viaIdx: parseInt(el.getAttribute('viaIdx'))||0, posIdx: parseInt(el.getAttribute('posIdx'))||0}); break;
      case 'Halt':    steps.push({type:'halt'}); break;
      case 'Brake':   steps.push({type:'brake'}); break;
      case 'Wait':    steps.push({type:'wait',   t: parseFloat(el.getAttribute('t'))||0}); break;
      case 'WaitFor': steps.push({type:'waitFor',cond: el.getAttribute('cond')||''}); break;
      case 'DIn':     steps.push({type:'din',    n: parseInt(el.getAttribute('n'))||1}); break;
      case 'DOut':    steps.push({type:'dout',   n: parseInt(el.getAttribute('n'))||1, v: el.getAttribute('v')||'FALSE'}); break;
      case 'AIn':     steps.push({type:'ain',    n: parseInt(el.getAttribute('n'))||1}); break;
      case 'AOut':    steps.push({type:'aout',   n: parseInt(el.getAttribute('n'))||1, v: parseFloat(el.getAttribute('v'))||0}); break;
      case 'Calc':    steps.push({type:'calc',   target: el.getAttribute('target')||'v', expr: el.getAttribute('expr')||'0'}); break;
      case 'Comment': steps.push({type:'comment',text: el.getAttribute('text')||''}); break;
      case 'PTPAxis': steps.push({type:'ptpAxis',angles: (el.getAttribute('angles')||'').split(',').map(Number)}); break;
      case 'Other':   steps.push({type:'other',  raw: el.getAttribute('raw')||''}); break;
    }
  });

  // BASE je Position aus Step-Reihenfolge zuordnen (Koordinaten bleiben as-is = base-relativ)
  var _cb = 0;
  steps.forEach(function(s){
    if (s.type === 'base') _cb = s.n;
    else if (s.type === 'move') { var p = positions[s.posIdx]; if (p && p.baseN == null) p.baseN = _cb; }
    else if (s.type === 'circ') { var pv = positions[s.viaIdx], pt = positions[s.posIdx]; if (pv && pv.baseN == null) pv.baseN = _cb; if (pt && pt.baseN == null) pt.baseN = _cb; }
  });
  positions.forEach(function(p){
    if (p.baseN == null) p.baseN = 0;
    p.rel = { X:p.X, Y:p.Y, Z:p.Z, A:p.A, B:p.B, C:p.C }; // <Pos> sind die Code-Koordinaten
  });

  return {positions: positions, steps: steps, finalState: {variables:{}, digitalIn:{}, digitalOut:{}, analogOut:{}, analogIn:{}}};
}

// ── Save ────────────────────────────────────────────────────────────
function fmtSaveFile() {
  var id = (typeof FormatRegistry !== 'undefined' && FormatRegistry.getActiveId) ? FormatRegistry.getActiveId() : 'kuka';
  var ext = _FMT_EXT[id] || '.txt';
  var baseName = 'robsimul_' + id;
  var content, mime;

  if (id === 'abb-binary') {
    if (typeof _3dp_saveZip === 'function') _3dp_saveZip();
    return;
  }
  if (id === 'kuka-form') {
    // Formular → XML
    content = parsedDataToXml(parsedData);
    mime = 'application/xml';
    baseName = 'robsimul_formular';
  } else {
    // Alle anderen → Editor-Inhalt
    var ci = document.getElementById('code-input');
    content = ci ? ci.value : '';
    mime = 'text/plain';
  }

  var blob = new Blob([content], {type: mime + ';charset=utf-8'});
  var a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = baseName + ext;
  document.body.appendChild(a); a.click();
  setTimeout(function() { URL.revokeObjectURL(a.href); a.remove(); }, 500);
}

// ── Load ────────────────────────────────────────────────────────────
function fmtLoadFile() {
  var inp = document.getElementById('fmt-load-input');
  if (!inp) return;
  var id = (typeof FormatRegistry !== 'undefined' && FormatRegistry.getActiveId) ? FormatRegistry.getActiveId() : 'kuka';
  if (id === 'abb-binary') {
    inp.accept = '.zip';
    inp.onchange = function() {
      var file = inp.files[0]; if (!file) return;
      if (typeof _3dp_loadZip === 'function') _3dp_loadZip(file);
      inp.value = '';
    };
    inp.click();
    return;
  }
  inp.accept = (id === 'kuka-form') ? '.xml,.pfg.xml' : '.src,.mod,.ls,.jbi,.pg,.pgx,.script,.v2,.prg,.pdl,.aubo,.dobot,.pcs,.fdr,.txt,.cpp,.as,.erd,.erp,.krl';
  inp.onchange = function() {
    var file = inp.files[0]; if (!file) return;
    var reader = new FileReader();
    reader.onload = function(e) {
      var text = e.target.result;
      var activeId = (typeof FormatRegistry !== 'undefined' && FormatRegistry.getActiveId) ? FormatRegistry.getActiveId() : 'kuka';
      if (activeId === 'kuka-form') {
        try {
          var pd = xmlToParsedData(text);
          if (!pd.positions.length && !pd.steps.length) { alert('XML konnte nicht gelesen werden.'); return; }
          parsedData = pd;
          if (typeof bakeBaseOffset === 'function') bakeBaseOffset();
          _krlSnapshot = generateKRL ? (generateKRL(pd)||'') : '';
          _krlSnapshotParsed = true;
          if (typeof FormatRegistry !== 'undefined') FormatRegistry.setActive('kuka');
        } catch(err) { alert('Fehler beim Laden: ' + err.message); }
      } else {
        var ci = document.getElementById('code-input');
        if (ci) {
          ci.value = text;
          if (typeof rebuildGutter === 'function') rebuildGutter();
          if (activeId === 'kuka') parseAndLoad();
        }
      }
      inp.value = '';
    };
    reader.readAsText(file);
  };
  inp.click();
}

// ═══════════════════════════════════════════════════
// CUSTOM FORMATS + GLOBALE MAPPING-TABELLE
// ═══════════════════════════════════════════════════

var _MAPPING_KEYS = [
  {key:'moveJ',  label:'MoveJ / PTP',        hint:'{N} {X}{Y}{Z}{A}{B}{C} {VEL_MMS} {VEL_PCT} {TOOL} {BASE}'},
  {key:'moveL',  label:'MoveL / LIN',        hint:'{N} {X}{Y}{Z}{A}{B}{C} {VEL_MMS} {TOOL} {BASE}'},
  {key:'moveC',  label:'MoveC / CIRC',       hint:'{N} {VN} {X}{Y}{Z} {VX}{VY}{VZ} {VEL_MMS}'},
  {key:'halt',   label:'Stop / HALT',        hint:''},
  {key:'brake',  label:'Pause / BRAKE',      hint:''},
  {key:'dout',   label:'SetDO / $OUT',       hint:'{CH} {VAL}'},
  {key:'din',    label:'WaitDI / $IN',       hint:'{CH}'},
  {key:'aout',   label:'SetAO / $ANOUT',     hint:'{CH} {VAL_F}'},
  {key:'ain',    label:'GetAI / $ANIN',      hint:'{CH}'},
  {key:'wait',   label:'WaitTime / WAIT SEC',hint:'{T} {T_MS}'},
  {key:'tool',   label:'SetTool / $TOOL',    hint:'{N}'},
  {key:'base',   label:'SetBase / $BASE',    hint:'{N}'},
  {key:'varInt', label:'VarInt / DECL INT',  hint:'{NAME} {INITVAL}'},
  {key:'varReal',label:'VarReal / DECL REAL',hint:'{NAME} {INITVAL}'},
  {key:'varBool',label:'VarBool / DECL BOOL',hint:'{NAME} {INITVAL}'},
  {key:'call',   label:'Call / CALL',        hint:'{PROG} {ARGS}'},
  {key:'calc',   label:'Calc / =',           hint:'{TARGET} {EXPR}'},
  {key:'velcp',  label:'Geschwindigkeit / VEL',hint:'{VEL_MMS} {VEL_PCT} {VEL_MS}'},
  {key:'comment',label:'Kommentar',            hint:'{TEXT}'},
  {key:'moveS',  label:'MoveS / SLIN / Spline',hint:'{N} {X}{Y}{Z} {VEL_MMS}'},
];

// Eingebaute Formate
var _MAPPING_FMTS_BUILTIN = [
  {id:'pfg',label:'Formular PFG'},
  {id:'kuka',label:'KUKA KRL'},{id:'abb',label:'ABB RAPID'},{id:'fanuc',label:'FANUC TP'},
  {id:'yaskawa',label:'Yaskawa INFORM'},{id:'kawasaki',label:'Kawasaki AS'},
  {id:'staubli',label:'Stäubli VAL3'},{id:'ur',label:'Universal Robots'},
  {id:'adept',label:'Adept V+'},{id:'omron',label:'Omron V+'},{id:'epson',label:'Epson SPEL+'},
  {id:'comau',label:'Comau PDL2'},{id:'aubo',label:'AUBO Script'},{id:'dobot',label:'Dobot Script'},
  {id:'denso',label:'Denso PACScript'},{id:'nachi',label:'Nachi FD'},{id:'hanwha',label:'Hanwha HCR'},
  {id:'igus',label:'igus iRC'},{id:'estun',label:'Estun ER'},{id:'neura',label:'NEURA SDK'},{id:'mabi',label:'MABI'},
];
// Custom Formate (aus localStorage)
var _MAPPING_FMTS_CUSTOM = [];

function _allFmts() { return _MAPPING_FMTS_BUILTIN.concat(_MAPPING_FMTS_CUSTOM); }

// Custom Befehlszeilen
var _MAPPING_KEYS_CUSTOM = [];
try { var _ck=localStorage.getItem('robsimul_custom_keys'); if(_ck) _MAPPING_KEYS_CUSTOM=JSON.parse(_ck); } catch(e){}
function _saveCustomKeys(){try{localStorage.setItem('robsimul_custom_keys',JSON.stringify(_MAPPING_KEYS_CUSTOM));}catch(e){}}
function _allKeys(){ return _MAPPING_KEYS.concat(_MAPPING_KEYS_CUSTOM); }

// Filter-Zustand: Set von sichtbaren Format-IDs (null = alle)
var _mappingVisible = null;

// ── Custom Format Generator ─────────────────────────────────────────
function _makeCustomGenerate(id, commentSyntax) {
  var cmt = commentSyntax || '; ';
  return function(pd) {
    if (!pd) return '';
    pd = (typeof _pfgNormalize === 'function') ? _pfgNormalize(pd) : pd;
    var positions = pd.positions||[], steps = pd.steps||[];
    var lines = [], curVel=0.167, toolN=1, baseN=1;
    var hf = (typeof fmtHfLoad==='function') ? fmtHfLoad(id) : null;
    if (hf && hf.header) hf.header.split('\n').forEach(function(l){lines.push(l);});

    function tpl(key, vars) {
      if (typeof applyTpl==='function') {
        var r = applyTpl(id, key, vars);
        if (r !== null) return r===''?null:r;
      }
      return null;
    }
    function pushOrCmt(val, key, vars) {
      if (val != null) lines.push(val);
      else lines.push(cmt + '[NICHT UNTERSTÜTZT] ' + key + (vars?' '+JSON.stringify(vars):''));
    }
    steps.forEach(function(s) {
      var velMms=Math.max(1,Math.round((curVel||0.167)*1000));
      var velPct=Math.max(1,Math.min(100,Math.round((curVel||0.167)/2.0*100)));
      var pos, pv, pt, mv, cv;
      switch(s.type) {
        case 'tool':  curVel=curVel; toolN=s.n; pushOrCmt(tpl('tool',{N:s.n,TOOL:s.n}),'tool',{N:s.n}); break;
        case 'base':  baseN=s.n; pushOrCmt(tpl('base',{N:s.n,BASE:s.n}),'base',{N:s.n}); break;
        case 'velcp': curVel=s.v||0.167; velMms=Math.max(1,Math.round(curVel*1000)); break;
        case 'halt':  pushOrCmt(tpl('halt',{}),'halt'); break;
        case 'brake': pushOrCmt(tpl('brake',{})||tpl('halt',{}),'brake'); break;
        case 'wait':  pushOrCmt(tpl('wait',{T:parseFloat(s.t||0).toFixed(1),T_MS:Math.round(parseFloat(s.t||0)*1000)}),'wait',{T:s.t}); break;
        case 'din':   pushOrCmt(tpl('din',{CH:s.n}),'din',{CH:s.n}); break;
        case 'dout':{ var dv=s.v==='TRUE'||s.v==='1'; pushOrCmt(tpl('dout',{CH:s.n,VAL:dv?'TRUE':'FALSE'}),'dout',{CH:s.n}); break; }
        case 'aout':  pushOrCmt(tpl('aout',{CH:s.n,VAL_F:parseFloat(s.v||0).toFixed(2)}),'aout',{CH:s.n}); break;
        case 'ain':   pushOrCmt(tpl('ain',{CH:s.n}),'ain',{CH:s.n}); break;
        case 'var': { var vt=s.varType||'REAL',vi=s.val||(vt==='BOOL'?'FALSE':vt==='INT'?'0':'0.0'); var vk=vt==='INT'?'varInt':vt==='BOOL'?'varBool':'varReal'; pushOrCmt(tpl(vk,{TYPE:vt,NAME:s.name||'v',INITVAL:vi}),vk); break; }
        case 'calc':  pushOrCmt(tpl('calc',{TARGET:s.target||'v',EXPR:s.expr||'0'}),'calc'); break;
        case 'move': {
          pos=positions[s.posIdx]; if(!pos) break;
          var isLin=s.moveType==='LIN'||s.moveType==='SLIN';
          mv={N:(s.posIdx||0)+1,X:pos.X.toFixed(3),Y:pos.Y.toFixed(3),Z:pos.Z.toFixed(3),
            A:pos.A.toFixed(3),B:pos.B.toFixed(3),C:pos.C.toFixed(3),
            VEL_MMS:velMms,VEL_PCT:velPct,VEL_MS:(curVel||0.167).toFixed(4),TOOL:toolN,BASE:baseN};
          pushOrCmt(tpl(isLin?'moveL':'moveJ',mv),isLin?'moveL':'moveJ',mv); break;
        }
        case 'circ': {
          pv=positions[s.viaIdx]; pt=positions[s.posIdx]; if(!pv||!pt) break;
          cv={N:(s.posIdx||0)+1,VN:(s.viaIdx||0)+1,
            X:pt.X.toFixed(3),Y:pt.Y.toFixed(3),Z:pt.Z.toFixed(3),
            A:pt.A.toFixed(3),B:pt.B.toFixed(3),C:pt.C.toFixed(3),
            VX:pv.X.toFixed(3),VY:pv.Y.toFixed(3),VZ:pv.Z.toFixed(3),
            VEL_MMS:velMms,TOOL:toolN,BASE:baseN};
          pushOrCmt(tpl('moveC',cv),'moveC',cv); break;
        }
      }
    });
    if (hf && hf.footer) hf.footer.split('\n').forEach(function(l){lines.push(l);});
    return lines.join('\n');
  };
}

// ── Custom Format registrieren ──────────────────────────────────────
function _registerCustomFmt(entry) {
  if (typeof FormatRegistry === 'undefined') return;
  var existing = FormatRegistry._allFormats ? FormatRegistry._allFormats().find(function(f){return f.id===entry.id;}) : null;
  if (existing) return;
  var logoHtml = entry.logo
    ? '<img src="' + entry.logo + '" height="14" style="vertical-align:middle;margin-right:6px;max-width:56px;object-fit:contain" onerror="this.style.display=\'none\'">'
    : '';
  var genFn = _makeCustomGenerate(entry.id, entry.comment||'; ');
  FormatRegistry.register({
    id:         entry.id,
    label:      entry.label,
    icon:       logoHtml,
    activate:   (function(gfn){ return function(){
      var ci=document.getElementById('code-input'), gt=document.getElementById('gutter');
      var fv=document.getElementById('krl-form-view'), fw=document.getElementById('fv-form-wrap');
      if(ci) ci.style.display=''; if(gt) gt.style.display='';
      if(fv) fv.style.display='none'; if(fw) fw.style.display='none';
      var pd=(typeof parsedData!=='undefined')?parsedData:{positions:[],steps:[],finalState:{}};
      if(ci){ ci.value=gfn(pd); if(typeof rebuildGutter==='function') rebuildGutter(); }
    };})(genFn),
    deactivate: function(){},
    _generate:  genFn,
  });
}

// ── Custom Formate laden ────────────────────────────────────────────
function loadCustomFormats() {
  try {
    var stored = localStorage.getItem('robsimul_custom_fmts');
    if (!stored) return;
    var list = JSON.parse(stored);
    _MAPPING_FMTS_CUSTOM = list;
    list.forEach(function(entry) { _registerCustomFmt(entry); });
  } catch(e) {}
}

function saveCustomFormats() {
  try { localStorage.setItem('robsimul_custom_fmts', JSON.stringify(_MAPPING_FMTS_CUSTOM)); } catch(e) {}
}

// Custom Format hinzufügen
function addCustomFormat(label, logo, commentSyntax) {
  var id = 'custom_' + label.toLowerCase().replace(/[^a-z0-9]/g,'_').replace(/__+/g,'_');
  if (_MAPPING_FMTS_CUSTOM.find(function(e){return e.id===id;})) { alert('ID bereits vorhanden: ' + id); return; }
  var entry = {id:id, label:label, logo:logo||'', comment:commentSyntax||'; '};
  _MAPPING_FMTS_CUSTOM.push(entry);
  saveCustomFormats();
  _registerCustomFmt(entry);
  return id;
}

function addCustomKey(key, label) {
  if(!key||!label){alert('Key und Label erforderlich.');return;}
  key=key.replace(/[^a-z0-9_]/gi,'_');
  if(_allKeys().find(function(k){return k.key===key;})){alert('Key bereits vorhanden: '+key);return;}
  _MAPPING_KEYS_CUSTOM.push({key:key,label:label,hint:''});
  _saveCustomKeys();
  _buildMappingTable();
}
function deleteCustomKey(key) {
  if(!confirm('Befehlszeile "'+key+'" löschen?')) return;
  _MAPPING_KEYS_CUSTOM=_MAPPING_KEYS_CUSTOM.filter(function(k){return k.key!==key;});
  _saveCustomKeys();
  _buildMappingTable();
}
function openAddKeyDialog(){
  var d=document.getElementById('add-key-dialog'); if(d){d.style.display='flex';document.getElementById('add-key-id').focus();}
}
function closeAddKeyDialog(){ var d=document.getElementById('add-key-dialog'); if(d) d.style.display='none'; }
function submitAddKey(){
  var key=(document.getElementById('add-key-id').value||'').trim();
  var label=(document.getElementById('add-key-label').value||'').trim();
  addCustomKey(key,label);
  if(document.getElementById('add-key-dialog')) {
    document.getElementById('add-key-id').value='';
    document.getElementById('add-key-label').value='';
    closeAddKeyDialog();
  }
}

function deleteCustomFormat(id) {
  _MAPPING_FMTS_CUSTOM = _MAPPING_FMTS_CUSTOM.filter(function(e){return e.id!==id;});
  saveCustomFormats();
  try { localStorage.removeItem('robsimul_cmd_' + id); } catch(e){}
  try { localStorage.removeItem('robsimul_hf_' + id); } catch(e){}
}

// Beim Start laden
if (document.readyState === 'loading') {
  document.addEventListener('DOMContentLoaded', loadCustomFormats);
} else {
  loadCustomFormats();
}

// ── Mapping-Tabelle ─────────────────────────────────────────────────
function openMappingTable() {
  var ov = document.getElementById('mapping-overlay');
  if (!ov) return;
  if (_mappingVisible === null) _mappingVisible = new Set(_allFmts().map(function(f){return f.id;}));
  ov.style.display = 'flex';
  _buildFilterBar();
  _buildMappingTable();
}

function closeMappingTable() {
  var ov = document.getElementById('mapping-overlay');
  if (ov) ov.style.display = 'none';
}

// Filter-Bar aufbauen
function _buildFilterBar() {
  var bar = document.getElementById('mapping-filter-bar');
  if (!bar) return;
  var fmts = _allFmts();
  var allVis = _mappingVisible === null || _mappingVisible.size === fmts.length;
  var html = '<button onclick="mappingFilterAll()" style="margin-right:4px;padding:2px 8px;border-radius:3px;cursor:pointer;font-size:11px;'+(allVis?'background:var(--acc);color:#000;border:none':'background:var(--bg3);color:var(--txt2);border:1px solid var(--bdr)')+'">ALLE</button>';
  fmts.forEach(function(f) {
    var vis = _mappingVisible===null || _mappingVisible.has(f.id);
    var isCustom = _MAPPING_FMTS_CUSTOM.find(function(e){return e.id===f.id;});
    var label = f.label.split(' ')[0];  // Kurzname
    html += '<button onclick="mappingFilterToggle(\'' + f.id + '\')" title="' + f.label + '" style="margin:2px;padding:2px 7px;border-radius:3px;cursor:pointer;font-size:10px;'
      +(vis?'background:var(--acc);color:#000;border:none;':'background:var(--bg3);color:var(--txt3);border:1px solid var(--bdr);')
      +(isCustom?'font-style:italic;':'')
      +'">'+ label +'</button>';
  });
  bar.innerHTML = html;
}

function mappingFilterAll() {
  var fmts = _allFmts();
  var allVis = _mappingVisible===null || _mappingVisible.size===fmts.length;
  if (allVis) {
    _mappingVisible = new Set([fmts[0].id]);  // nur ersten behalten
  } else {
    _mappingVisible = null;
  }
  _buildFilterBar(); _buildMappingTable();
}

function mappingFilterToggle(id) {
  if (_mappingVisible === null) _mappingVisible = new Set(_allFmts().map(function(f){return f.id;}));
  if (_mappingVisible.has(id)) { _mappingVisible.delete(id); }
  else { _mappingVisible.add(id); }
  if (_mappingVisible.size === 0) _mappingVisible.add(id);
  _buildFilterBar(); _buildMappingTable();
}

// Tabelle aufbauen
function _buildMappingTable() {
  var tbl = document.getElementById('mapping-table');
  if (!tbl) return;
  var fmts = _allFmts().filter(function(f){ return _mappingVisible===null||_mappingVisible.has(f.id); });

  var head = '<thead><tr><th style="position:sticky;left:0;z-index:2;background:var(--bg2);padding:4px 8px;border:1px solid var(--bdr);min-width:160px">Befehl</th>';
  fmts.forEach(function(f) {
    var isCustom = !!_MAPPING_FMTS_CUSTOM.find(function(e){return e.id===f.id;});
    head += '<th style="padding:3px 6px;border:1px solid var(--bdr);min-width:180px;background:var(--bg2);'+(isCustom?'font-style:italic;color:var(--acc);':'')+'">'
      + f.label
      + ' <button onclick="mappingResetFormat(\'' + f.id + '\')" title="Zurücksetzen" style="background:none;border:none;color:var(--txt3);cursor:pointer;font-size:10px;padding:0 2px">↺</button>'
      + (isCustom ? ' <button onclick="mappingDeleteCustom(\'' + f.id + '\')" title="Hersteller löschen" style="background:none;border:none;color:#f87171;cursor:pointer;font-size:10px;padding:0 2px">✕</button>' : '')
      + '</th>';
  });
  head += '</tr></thead>';

  var body = '<tbody>';
  _allKeys().forEach(function(mk) {
    var isCustomKey = !!_MAPPING_KEYS_CUSTOM.find(function(k){return k.key===mk.key;});
    body += '<tr><td style="position:sticky;left:0;z-index:1;background:var(--bg2);padding:4px 8px;border:1px solid var(--bdr);font-weight:600;color:var(--acc)">'
      + mk.label
      + (isCustomKey?' <button onclick="deleteCustomKey(\''+mk.key+'\')" title="Zeile löschen" style="background:none;border:none;color:#f87171;cursor:pointer;font-size:10px">✕</button>':'')
      + '<br><span style="color:var(--txt3);font-weight:400;font-size:10px">' + mk.hint + '</span></td>';
    fmts.forEach(function(f) {
      var defVal = (typeof FMT_CMD_DEFAULTS!=='undefined'&&FMT_CMD_DEFAULTS[f.id]) ? (FMT_CMD_DEFAULTS[f.id][mk.key]||'') : '';
      var storedVal = '';
      try { var s=localStorage.getItem('robsimul_cmd_'+f.id); if(s){var o=JSON.parse(s);if(o[mk.key]!==undefined)storedVal=o[mk.key];} } catch(e){}
      var val = storedVal!==''?storedVal:defVal;
      var isCustom = storedVal!=='';
      var isEmpty = val==='';
      var bg = isEmpty?'rgba(220,60,60,.12)':isCustom?'rgba(60,180,100,.08)':'';
      body += '<td style="border:1px solid var(--bdr);padding:2px;vertical-align:top;background:'+bg+'">'
        +'<textarea id="mc_'+f.id+'_'+mk.key+'" data-default="'+defVal.replace(/"/g,'&quot;')+'" spellcheck="false"'
        +' oninput="mappingCellChanged(this)"'
        +' style="width:176px;height:44px;resize:vertical;background:transparent;border:none;color:var(--txt1);font-family:monospace;font-size:10px;padding:2px;outline:none">'
        +val.replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;')+'</textarea></td>';
    });
    body += '</tr>';
  });
  body += '</tbody>';
  tbl.innerHTML = head + body;
}

function mappingCellChanged(ta) {
  var isDefault = ta.value === ta.dataset.default;
  ta.parentElement.style.background = ta.value.trim()===''?'rgba(220,60,60,.12)':!isDefault?'rgba(60,180,100,.08)':'';
}

function mappingApplyAll() {
  _allFmts().forEach(function(f) {
    var cmds={}, hasChange=false;
    _allKeys().forEach(function(mk) {
      var cell=document.getElementById('mc_'+f.id+'_'+mk.key);
      if(!cell) return;
      cmds[mk.key]=cell.value;
      if(cell.value!==cell.dataset.default) hasChange=true;
    });
    try {
      if(hasChange) localStorage.setItem('robsimul_cmd_'+f.id,JSON.stringify(cmds));
      else localStorage.removeItem('robsimul_cmd_'+f.id);
    } catch(e){}
  });
  // Aktives Format neu generieren
  if(typeof FormatRegistry!=='undefined'&&FormatRegistry.getActiveId) {
    var id=FormatRegistry.getActiveId();
    var fmts=FormatRegistry._allFormats?FormatRegistry._allFormats():[];
    var fmt=fmts.find(function(f){return f.id===id;});
    if(fmt&&fmt._generate&&typeof parsedData!=='undefined') {
      var ci=document.getElementById('code-input');
      if(ci){ci.value=fmt._generate(parsedData);if(typeof rebuildGutter==='function')rebuildGutter();}
    }
  }
  closeMappingTable();
}

function mappingResetFormat(fmtId) {
  try{localStorage.removeItem('robsimul_cmd_'+fmtId);}catch(e){}
  var defaults=(typeof FMT_CMD_DEFAULTS!=='undefined')?(FMT_CMD_DEFAULTS[fmtId]||{}):{};
  _allKeys().forEach(function(mk) {
    var cell=document.getElementById('mc_'+fmtId+'_'+mk.key);
    if(!cell) return;
    cell.value=defaults[mk.key]||'';
    cell.parentElement.style.background='';
  });
}

function mappingDeleteCustom(id) {
  if(!confirm('Hersteller "'+id+'" wirklich löschen?')) return;
  deleteCustomFormat(id);
  _mappingVisible = null;
  _buildFilterBar(); _buildMappingTable();
}

function mappingResetAll() {
  if(!confirm('Alle Mapping-Einstellungen auf Standard zurücksetzen?')) return;
  _allFmts().forEach(function(f){try{localStorage.removeItem('robsimul_cmd_'+f.id);}catch(e){}});
  _buildMappingTable();
}

function mappingExport() {
  var data={_custom:_MAPPING_FMTS_CUSTOM};
  _allFmts().forEach(function(f) {
    try{var s=localStorage.getItem('robsimul_cmd_'+f.id);if(s)data[f.id]=JSON.parse(s);}catch(e){}
  });
  var blob=new Blob([JSON.stringify(data,null,2)],{type:'application/json'});
  var a=document.createElement('a');a.href=URL.createObjectURL(blob);
  a.download='robsimul_mapping.json';document.body.appendChild(a);a.click();
  setTimeout(function(){URL.revokeObjectURL(a.href);a.remove();},500);
}

function mappingImport() {
  var inp=document.getElementById('mapping-import-input');
  if(!inp) return;
  inp.onchange=function() {
    var file=inp.files[0];if(!file) return;
    var reader=new FileReader();
    reader.onload=function(e) {
      try {
        var data=JSON.parse(e.target.result);
        if(data._custom && Array.isArray(data._custom)) {
          data._custom.forEach(function(entry){
            if(!_MAPPING_FMTS_CUSTOM.find(function(e){return e.id===entry.id;})) {
              _MAPPING_FMTS_CUSTOM.push(entry);
              _registerCustomFmt(entry);
            }
          });
          saveCustomFormats();
          delete data._custom;
        }
        Object.keys(data).forEach(function(fmtId){
          try{localStorage.setItem('robsimul_cmd_'+fmtId,JSON.stringify(data[fmtId]));}catch(e){}
        });
        _mappingVisible=null; _buildFilterBar(); _buildMappingTable();
        alert('Mapping importiert.');
      } catch(err){alert('Fehler: '+err.message);}
      inp.value='';
    };
    reader.readAsText(file);
  };
  inp.click();
}

// ── Hersteller hinzufügen Dialog ────────────────────────────────────
function openAddFmtDialog() {
  var dlg=document.getElementById('add-fmt-dialog');
  if(dlg){ dlg.style.display='flex'; document.getElementById('add-fmt-name').focus(); }
}
function closeAddFmtDialog() {
  var dlg=document.getElementById('add-fmt-dialog');
  if(dlg) dlg.style.display='none';
}
function submitAddFmt() {
  var name=(document.getElementById('add-fmt-name').value||'').trim();
  var logo=(document.getElementById('add-fmt-logo').value||'').trim();
  var cmt=(document.getElementById('add-fmt-cmt').value||'; ').trim()||'; ';
  if(!name){alert('Name erforderlich.');return;}
  var id=addCustomFormat(name, logo, cmt+' ');
  if(!id) return;
  // Logo-Upload-Vorschau
  var uploadInp=document.getElementById('add-fmt-logo-file');
  if(uploadInp&&uploadInp.files&&uploadInp.files[0]) {
    var reader=new FileReader();
    reader.onload=function(e){
      var entry=_MAPPING_FMTS_CUSTOM.find(function(en){return en.id===id;});
      if(entry){entry.logo=e.target.result;saveCustomFormats();}
    };
    reader.readAsDataURL(uploadInp.files[0]);
  }
  _mappingVisible=null; _buildFilterBar(); _buildMappingTable();
  closeAddFmtDialog();
  document.getElementById('add-fmt-name').value='';
  document.getElementById('add-fmt-logo').value='';
  document.getElementById('add-fmt-cmt').value='';
}

// ── postMessage API für iframe-Einbettung ──────────────────────────────────
window.addEventListener('message', function(e) {
  if (e.origin !== 'https://cnc-technik.de') return;
  var msg = e.data;
  if (!msg || typeof msg !== 'object') return;

  if (msg.type === 'krl:load') {
    var ta = document.getElementById('code-input');
    if (!ta) return;
    ta.value = msg.code || '';
    rebuildGutter();
    parseAndLoad();
    e.source.postMessage({ type: 'krl:loaded', ok: true }, e.origin);
  }

  if (msg.type === 'krl:get') {
    var ta2 = document.getElementById('code-input');
    var code = ta2 ? ta2.value : '';
    e.source.postMessage({ type: 'krl:code', code: code }, e.origin);
  }

  if (msg.type === 'krl:control') {
    if (msg.action === 'play')  { document.getElementById('b-playfwd') && document.getElementById('b-playfwd').click(); }
    if (msg.action === 'pause') { typeof pauseSim === 'function' && pauseSim(); }
    if (msg.action === 'stop')  { typeof stopSim  === 'function' && stopSim(0); }
  }
});

// ── embed-Modus (?embed=1) ─────────────────────────────────────────────────
(function() {
  if (new URLSearchParams(location.search).get('embed') !== '1') return;
  function applyEmbed() {
    var sel = [
      'header',
      '.sidebar', '#sidebar', '#left-panel',
      '#menu-bar', '.menu-bar',
      '#top-bar', '.top-bar',
      '#file-panel', '.file-panel',
      '#upload-btn', '.upload-btn'
    ];
    sel.forEach(function(s) {
      var el = document.querySelector(s);
      if (el) el.style.setProperty('display', 'none', 'important');
    });
  }
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', applyEmbed);
  } else {
    applyEmbed();
  }
})();
