'use strict';
// ═══════════════════════════════════════════════════════════════
// RobotDPSolver — portiert für Vanilla JS (kein ES-Module)
// Ersetzt computeIKTable + buildTrajectory
// ═══════════════════════════════════════════════════════════════

// ── Hilfsfunktionen ───────────────────────────────────────────
function _angleDeltaDeg(a, b) {
  var d = b - a;
  while (d > 180) d -= 360;
  while (d < -180) d += 360;
  return d;
}

function _unwrapAngle(prev, cur) {
  var c = cur;
  while (c - prev > 180)  c -= 360;
  while (c - prev < -180) c += 360;
  return c;
}

function _clamp(v, lo, hi) { return Math.max(lo, Math.min(hi, v)); }

function _jlPenalty(q, limits, safety) {
  safety = safety || 3;
  var p = 0;
  for (var i = 0; i < 6; i++) {
    var d = Math.min(q[i] - limits[i].min, limits[i].max - q[i]);
    if (d <= 0) return 1e15;
    if (d < safety) p += 1e6 * (safety - d + 1);
    p += 1 / Math.max(d, 0.001);
  }
  return p;
}

function _jlOk(q, limits, safety) {
  safety = safety || 0;
  for (var i = 0; i < 6; i++) {
    if (q[i] < limits[i].min + safety) return false;
    if (q[i] > limits[i].max - safety) return false;
  }
  return true;
}

function _configKey(q) {
  return (q[0] >= 0 ? 'S+' : 'S-') + '/' + (q[2] >= 0 ? 'E+' : 'E-') + '/' + (q[4] >= 0 ? 'W+' : 'W-');
}

// ── Natürlicher kubischer Spline ──────────────────────────────
function NaturalCubicSpline(xs, ys) {
  this.xs = xs.slice();
  this.ys = ys.slice();
  this.n  = xs.length;
  this.y2 = this._secondDerivatives(xs, ys);
}
NaturalCubicSpline.prototype._secondDerivatives = function(x, y) {
  var n = x.length, y2 = new Array(n).fill(0), u = new Array(n-1).fill(0);
  for (var i = 1; i < n-1; i++) {
    var sig = (x[i]-x[i-1])/(x[i+1]-x[i-1]);
    var p   = sig*y2[i-1]+2;
    y2[i]   = (sig-1)/p;
    u[i]    = (6*((y[i+1]-y[i])/(x[i+1]-x[i])-(y[i]-y[i-1])/(x[i]-x[i-1]))/(x[i+1]-x[i-1])-sig*u[i-1])/p;
  }
  y2[n-1] = 0;
  for (var k = n-2; k >= 0; k--) y2[k] = y2[k]*y2[k+1]+u[k];
  return y2;
};
NaturalCubicSpline.prototype.sample = function(x) {
  var xs=this.xs, ys=this.ys, y2=this.y2, n=this.n;
  if (x <= xs[0])   return ys[0];
  if (x >= xs[n-1]) return ys[n-1];
  var lo=0, hi=n-1;
  while (hi-lo > 1) { var m=(hi+lo)>>1; if(xs[m]>x) hi=m; else lo=m; }
  var h=xs[hi]-xs[lo], a=(xs[hi]-x)/h, b=(x-xs[lo])/h;
  return a*ys[lo]+b*ys[hi]+((a*a*a-a)*y2[lo]+(b*b*b-b)*y2[hi])*h*h/6;
};

function _smoothPath(path, sampleCount) {
  sampleCount = sampleCount || 200;
  if (path.length < 2) return path;
  var xs = path.map(function(p){ return p.s; });
  var splines = [];
  for (var ax = 0; ax < 6; ax++) {
    splines.push(new NaturalCubicSpline(xs, path.map(function(p){ return p.q[ax]; })));
  }
  var s0=xs[0], s1=xs[xs.length-1], result=[];
  for (var i = 0; i < sampleCount; i++) {
    var t = sampleCount===1 ? 0 : i/(sampleCount-1);
    var s = s0+(s1-s0)*t;
    result.push({ s:s, q:splines.map(function(sp){ return sp.sample(s); }) });
  }
  return result;
}

function _unwrapPath(path) {
  if (path.length < 2) return path;
  var out = path.map(function(p){ return { s:p.s, q:p.q.slice(), config:p.config, singularity:p.singularity }; });
  for (var i = 1; i < out.length; i++) {
    for (var ax = 0; ax < 6; ax++) {
      out[i].q[ax] = _unwrapAngle(out[i-1].q[ax], out[i].q[ax]);
    }
  }
  return out;
}

// ── Haupt-Solver ──────────────────────────────────────────────
// Globale Einstellungen (werden von außen gesetzt)
var DPSolver = {
  settings: {
    safetyDeg:          3,
    maxTcpPosError:     2.0,    // mm
    maxTcpRotErrorDeg:  2.0,
    minSingularity:     0.01,
    a6Copies:           [-2,-1,0,1,2],
    smoothSamples:      200,
    wMove:              1,
    wAxes:              [1,1,1,4,5,0.25],
    wLimit:             50,
    wSingularity:       200,
    wConfigSwitch:      5000,
    wTcpError:          10000,
  },

  // ── Externe Hooks (müssen gesetzt werden) ──────────────────
  solveIKFn:  null,   // (x,y,z,a,b,c,initQ) => {ok,angles,score}
  fkFn:       null,   // (angles) => {pts:[...], tcp_rot:[[...]]}
  singFn:     null,   // (angles) => number (höher=besser)
  limits:     null,   // [{min,max}] x6

  // ── Plan berechnen ─────────────────────────────────────────
  plan: function(targetPoints, qStart) {
    // targetPoints: [{s, X,Y,Z,A,B,C}]
    var S = this.settings;
    var layers = this._buildLayers(targetPoints);
    var rawPath = this._dp(layers, qStart);
    var unwrapped = _unwrapPath(rawPath);
    var smoothed  = _smoothPath(unwrapped, S.smoothSamples);
    return { rawPath:rawPath, smoothedPath:smoothed, ok:true };
  },

  _buildLayers: function(targetPoints) {
    var S = this.settings;
    var self = this;
    var layers = [];

    for (var ti = 0; ti < targetPoints.length; ti++) {
      var tp = targetPoints[ti];
      var nodes = [];
      var seen  = [];

      // Mehrere Startwerte für IK
      var inits = [null, [0,-90,90,0,0,0], [0,-90,90,0,-45,0],
                   [180,-90,90,0,0,0], [0,-120,110,0,0,0]];

      for (var ii = 0; ii < inits.length; ii++) {
        var res = self.solveIKFn(tp.X, tp.Y, tp.Z, tp.A, tp.B, tp.C, inits[ii]);
        if (!res.ok) continue;
        var baseQ = res.angles;

        for (var ki = 0; ki < S.a6Copies.length; ki++) {
          var q = baseQ.slice();
          q[5] = q[5] + 360 * S.a6Copies[ki];

          if (!_jlOk(q, self.limits, S.safetyDeg)) continue;

          // FK-Fehler prüfen
          var fkr = self.fkFn(q);
          var tcp = fkr.pts[7];
          if (!tcp) continue;
          var dpos = Math.sqrt(
            (tcp[0]-tp.X)*(tcp[0]-tp.X)+
            (tcp[1]-tp.Y)*(tcp[1]-tp.Y)+
            (tcp[2]-tp.Z)*(tcp[2]-tp.Z)
          );
          if (dpos > S.maxTcpPosError) continue;

          // Singularität
          var sing = self.singFn ? self.singFn(q) : self._defaultSing(q);
          if (sing < S.minSingularity) continue;

          // Duplikat (0.5° Toleranz)
          var dup = seen.some(function(s){ return s.every(function(v,i){ return Math.abs(v-q[i])<0.5; }); });
          if (dup) continue;
          seen.push(q.slice());

          nodes.push({ s:tp.s, q:q.slice(), config:_configKey(q), singularity:sing, posErr:dpos, tp:tp });
        }
      }

      // Fallback: mindestens ein Kandidat
      if (nodes.length === 0) {
        var fb = self.solveIKFn(tp.X, tp.Y, tp.Z, tp.A, tp.B, tp.C, null);
        if (fb.ok) nodes.push({ s:tp.s, q:fb.angles.slice(), config:_configKey(fb.angles), singularity:0.1, posErr:99, tp:tp });
      }

      layers.push(nodes);
    }
    return layers;
  },

  _dp: function(layers, qStart) {
    var S = this.settings;
    var self = this;
    var N = layers.length;
    var costs   = layers.map(function(l){ return new Array(l.length).fill(1e15); });
    var parents = layers.map(function(l){ return new Array(l.length).fill(-1); });

    // Init erste Schicht
    for (var j = 0; j < layers[0].length; j++) {
      costs[0][j] = self._nodeCost(layers[0][j]);
      if (qStart) costs[0][j] += self._transCost({q:qStart,config:_configKey(qStart)}, layers[0][j]);
    }

    // Vorwärts-DP
    for (var li = 1; li < N; li++) {
      for (var j = 0; j < layers[li].length; j++) {
        var cur = layers[li][j];
        for (var k = 0; k < layers[li-1].length; k++) {
          var prev = layers[li-1][k];
          var c = costs[li-1][k] + self._transCost(prev,cur) + self._nodeCost(cur);
          if (c < costs[li][j]) { costs[li][j]=c; parents[li][j]=k; }
        }
      }
    }

    // Besten Endpunkt
    var last=N-1, bestJ=0, bestC=costs[last][0];
    for (var j = 1; j < layers[last].length; j++) {
      if (costs[last][j] < bestC) { bestC=costs[last][j]; bestJ=j; }
    }

    // Rückwärts-Trace
    var path=[], idx=bestJ;
    for (var li = last; li >= 0; li--) {
      var nd = layers[li][idx];
      path.push({ s:nd.s, q:nd.q.slice(), config:nd.config, singularity:nd.singularity });
      idx = parents[li][idx];
      if (idx < 0 && li > 0) break;
    }
    return path.reverse();
  },

  _nodeCost: function(nd) {
    var S = this.settings;
    var lim  = _jlPenalty(nd.q, this.limits, S.safetyDeg);
    var sing  = 1.0 / Math.max(nd.singularity, 1e-9);
    var tcpE  = nd.posErr || 0;
    return S.wLimit*lim + S.wSingularity*sing + S.wTcpError*tcpE;
  },

  _transCost: function(prev, cur) {
    var S = this.settings;
    var c = 0;
    for (var ax = 0; ax < 6; ax++) {
      var d = cur.q[ax] - prev.q[ax];
      c += S.wMove * S.wAxes[ax] * d * d;
    }
    if (prev.config && cur.config && prev.config !== cur.config) c += S.wConfigSwitch;
    return c;
  },

  _defaultSing: function(q) {
    var a5 = Math.abs(((q[4]+180)%360)-180);
    return Math.min(a5, 180-a5) / 180.0;
  }
};
