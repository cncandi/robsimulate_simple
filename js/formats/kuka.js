// ══════════════════════════════════════════════════════
// FORMAT: KUKA KRL  —  Ausgabe aus parsedData + Templates
// ══════════════════════════════════════════════════════

function _kukaGenerate(pd) {
  if (!pd) return '';
  var positions = pd.positions || [];
  var steps     = pd.steps     || [];
  var lines     = [];

  // Header aus Settings oder Default
  var hf = (typeof fmtHfLoad === 'function') ? fmtHfLoad('kuka') : null;
  var hdr = (hf && hf.header) ? hf.header : 'DEF PP_MAIN()\n BAS(#INITMOV,0)';
  hdr.split('\n').forEach(function(l) { lines.push(l); });

  function tpl(key, vars, fallback) {
    if (typeof applyTpl === 'function') {
      var r = applyTpl('kuka', key, vars);
      if (r !== null) return r;
    }
    return fallback;
  }

  var curVel = 0.167;

  if (steps.length) {
    steps.forEach(function(s) {
      var pos, coord, v;
      switch (s.type) {
        case 'comment':  lines.push('; ' + (s.text || '')); break;
        case 'tool':     lines.push(tpl('tool',   {N:s.n}, '$TOOL=TOOL_DATA[' + s.n + ']')); break;
        case 'base':     lines.push(tpl('base',   {N:s.n}, '$BASE=BASE_DATA[' + s.n + ']')); break;
        case 'velcp':    curVel = s.v || 0.167;
                         lines.push('$VEL.CP=' + parseFloat(s.v || 0.167).toFixed(3)); break;
        case 'velptp':   lines.push('$VEL.PTP=' + Math.round(s.v || 100)); break;
        case 'acccp':    lines.push('$ACC.CP=' + parseFloat(s.v || 1).toFixed(1)); break;
        case 'advance':  lines.push('$advance=' + Math.round(s.v != null ? s.v : 5)); break;
        case 'halt':     lines.push(tpl('halt',   {}, 'HALT')); break;
        case 'brake':    lines.push('BRAKE'); break;
        case 'wait':     lines.push(tpl('wait',   {T:parseFloat(s.t||0).toFixed(1), T_MS:Math.round(parseFloat(s.t||0)*1000)}, 'WAIT SEC ' + parseFloat(s.t||0).toFixed(1))); break;
        case 'waitFor':  lines.push('WAIT FOR ' + (s.cond || '$IN[1]')); break;
        case 'dout':     lines.push(tpl('dout',   {CH:s.n, VAL:s.v||'FALSE'}, '$OUT[' + s.n + ']=' + (s.v||'FALSE'))); break;
        case 'din':      lines.push(tpl('din',    {CH:s.n}, 'WAIT FOR $IN[' + s.n + ']')); break;
        case 'aout':     lines.push(tpl('aout',   {CH:s.n, VAL_F:parseFloat(s.v||0).toFixed(2)}, '$ANOUT[' + s.n + ']=' + parseFloat(s.v||0).toFixed(2))); break;
        case 'ain':      lines.push('$ANIN[' + s.n + ']'); break;
        case 'var': {
          var vt = s.varType || 'REAL';
          var vv = s.val != null ? s.val : (vt==='BOOL'?'FALSE':vt==='INT'?'0':'0.0');
          var vk = vt==='INT'?'varInt':vt==='BOOL'?'varBool':'varReal';
          lines.push(tpl(vk, {TYPE:vt,NAME:s.name||'v',INITVAL:vv}, 'DECL '+vt+' '+(s.name||'v')+'='+vv));
          break;
        }
        case 'calc':     lines.push(tpl('calc',{TARGET:s.target||'v',EXPR:s.expr||'0'}, (s.target||'v')+' = '+(s.expr||'0'))); break;
        case 'ptpAxis':
          if (s.angles) lines.push('PTP {A1 '+s.angles[0].toFixed(3)+',A2 '+s.angles[1].toFixed(3)+',A3 '+s.angles[2].toFixed(3)+',A4 '+s.angles[3].toFixed(3)+',A5 '+s.angles[4].toFixed(3)+',A6 '+s.angles[5].toFixed(3)+'}');
          break;
        case 'move': {
          pos = positions[s.posIdx]; if (!pos) break;
          var mt = s.moveType || 'LIN';
          var mv = {
            N: s.posIdx+1,
            X: pos.X.toFixed(3), Y: pos.Y.toFixed(3), Z: pos.Z.toFixed(3),
            A: pos.A.toFixed(3), B: pos.B.toFixed(3), C: pos.C.toFixed(3),
            VEL_MMS: Math.max(1, Math.round((curVel||0.167)*1000)),
            VEL_PCT: Math.max(1, Math.min(100, Math.round((curVel||0.167)/2.0*100))),
            VEL_MS: (curVel||0.167).toFixed(4)
          };
          var Sv = (pos.S != null ? ' S '+pos.S : ''), Tv = (pos.T != null ? ' T '+pos.T : '');
          var def = mt + ' {X '+pos.X.toFixed(3)+',Y '+pos.Y.toFixed(3)+',Z '+pos.Z.toFixed(3)+',A '+pos.A.toFixed(3)+',B '+pos.B.toFixed(3)+',C '+pos.C.toFixed(3)+Sv+Tv+'}';
          var key = (mt==='LIN'||mt==='SLIN') ? 'moveL' : 'moveJ';
          lines.push(tpl(key, mv, def));
          break;
        }
        case 'circ': {
          var pVia = positions[s.viaIdx], pTo = positions[s.posIdx];
          if (!pVia || !pTo) break;
          var cv = {
            N: s.posIdx+1, VN: s.viaIdx+1,
            X:pTo.X.toFixed(3),  Y:pTo.Y.toFixed(3),  Z:pTo.Z.toFixed(3),
            A:pTo.A.toFixed(3),  B:pTo.B.toFixed(3),  C:pTo.C.toFixed(3),
            VX:pVia.X.toFixed(3),VY:pVia.Y.toFixed(3),VZ:pVia.Z.toFixed(3),
            VA:pVia.A.toFixed(3),VB:pVia.B.toFixed(3),VC:pVia.C.toFixed(3),
          };
          var cVia = '{X '+pVia.X.toFixed(3)+',Y '+pVia.Y.toFixed(3)+',Z '+pVia.Z.toFixed(3)+',A '+pVia.A.toFixed(3)+',B '+pVia.B.toFixed(3)+',C '+pVia.C.toFixed(3)+'}';
          var cTo  = '{X '+pTo.X.toFixed(3)+',Y '+pTo.Y.toFixed(3)+',Z '+pTo.Z.toFixed(3)+',A '+pTo.A.toFixed(3)+',B '+pTo.B.toFixed(3)+',C '+pTo.C.toFixed(3)+'}';
          lines.push(tpl('moveC', cv, 'CIRC ' + cVia + ', ' + cTo));
          break;
        }
      }
    });
  } else {
    // Fallback: nur Positionen
    lines.push('$BASE=BASE_DATA[1]');
    lines.push('$TOOL=TOOL_DATA[1]');
    lines.push('$VEL.CP=0.167');
    positions.forEach(function(pos) {
      var Sv = (pos.S != null ? ' S '+pos.S : ''), Tv = (pos.T != null ? ' T '+pos.T : '');
      var mv = {X:pos.X.toFixed(3),Y:pos.Y.toFixed(3),Z:pos.Z.toFixed(3),A:pos.A.toFixed(3),B:pos.B.toFixed(3),C:pos.C.toFixed(3)};
      lines.push(tpl('moveL', mv, 'LIN {X '+pos.X.toFixed(3)+',Y '+pos.Y.toFixed(3)+',Z '+pos.Z.toFixed(3)+',A '+pos.A.toFixed(3)+',B '+pos.B.toFixed(3)+',C '+pos.C.toFixed(3)+Sv+Tv+'}'));
    });
  }

  // Footer aus Settings oder Default
  var ftr = (hf && hf.footer) ? hf.footer : 'END';
  ftr.split('\n').forEach(function(l) { lines.push(l); });

  return lines.join('\n');
}

FormatRegistry.register({
  id:    'kuka',
  label: 'KUKA KRL',
  icon:  '<img src="logos/kuka.png" height="14" style="vertical-align:middle;margin-right:6px;max-width:56px;object-fit:contain">',

  activate: function () {
    var ci = document.getElementById('code-input');
    var gt = document.getElementById('gutter');
    var fv = document.getElementById('krl-form-view');
    var fw = document.getElementById('fv-form-wrap');
    if (ci) ci.style.display = '';
    if (gt) gt.style.display = '';
    if (fv) fv.style.display = 'none';
    if (fw) fw.style.display = 'none';
    // Simple: einziges Format — Editorinhalt bleibt unverändert (kein Neugenerieren)
    if (typeof rebuildGutter === 'function') rebuildGutter();
  },

  deactivate: function () {
    var ci = document.getElementById('code-input');
    if (!ci || !ci.value) return;
    _krlSnapshot = ci.value;
    // parsedData synchronisieren damit Format-Wechsel aktuellen Stand nutzt
    if (typeof parseKRL === 'function') {
      var pd = parseKRL(_krlSnapshot);
      if (pd && pd.positions) {
        parsedData = pd;
        _krlSnapshotParsed = true;
      }
    }
  },

  _generate: _kukaGenerate,
});
