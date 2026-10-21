"""Erzeugt robsimulate_simple/index.html aus robsimulate/index.html.
Behalten: Roboterauswahl (Library, nur Roboter), KRL-Editor, Simulation, A-Plot.
"""
import re, sys

SRC = '/home/claude/robsimulate/index.html'
DST = '/home/claude/robsimulate_simple/index.html'
s = open(SRC, encoding='utf-8').read()

def between(a, b, incl_b=False):
    i = s.index(a)
    j = s.index(b, i)
    return s[i:j + (len(b) if incl_b else 0)]

def balanced(frag):
    return frag.count('<div') == frag.count('</div>')

# ── 1. Kopf + Sprite (bis Ende des SVG-Sprites) ─────────────────────
head = s[:s.index('<header>')]
head = head.replace('<title>RobSimul · cnc-technik.de</title>',
                    '<title>RobSimul Simple · cnc-technik.de</title>')

# ── 2. Neue Kopfzeile + Toolbar ─────────────────────────────────────
header = '''<header>
  <div class="appname"><svg class="ico"><use href="#s-robot"/></svg> RobSimul&nbsp;<span style="color:var(--txt2);font-weight:400">Simple</span>&nbsp;<span id="ver-header" class="ver-tag"></span></div>
  <button class="simple-hbtn" onclick="openLibPickerModal('robot')" title="Robotermodell aus der Bibliothek wählen">
    <svg class="ico"><use href="#s-robot"/></svg>Roboter wählen
  </button>
  <span class="simple-robot-lbl">Aktiv: <b id="simple-robot-name">Standardmodell</b></span>
  <div class="mnu-right">
    <span style="color:var(--txt3);font-size:.72em">© CAD/CAM Systeme Datentechnik Reitz</span>
  </div>
</header>

<!-- ── TOOLBAR ──────────────────────────────────────────── -->
<div class="tb">
  <button class="sbtn" id="rib-editor" onclick="toggleLeftPanel()" title="Editor ein/ausblenden" style="min-width:60px"><svg class="ico"><use href="#s-open"/></svg>Editor</button>
  <div class="sep"></div>
  <button class="sbtn icon-only" id="b-start"   title="Zum Anfang [Home]"><svg class="ico"><use href="#s-skip-s"/></svg></button>
  <button class="sbtn icon-only" id="b-playrev"  title="Rückwärts"><svg class="ico"><use href="#s-play-b"/></svg></button>
  <button class="sbtn icon-only" id="b-steprev"  title="Schritt zurück [←]"><svg class="ico"><use href="#s-step-b"/></svg></button>
  <button class="sbtn icon-only r" id="b-stop"   title="Stop [Space]"><svg class="ico"><use href="#s-stop"/></svg></button>
  <button class="sbtn icon-only g" id="b-stepfwd" title="Schritt vor [→]"><svg class="ico"><use href="#s-step-f"/></svg></button>
  <button class="sbtn icon-only g" id="b-playfwd" title="Vorwärts [F5]"><svg class="ico"><use href="#s-play-f"/></svg></button>
  <button class="sbtn icon-only" id="b-end"      title="Zum Ende [End]"><svg class="ico"><use href="#s-skip-e"/></svg></button>
  <div class="sep"></div>
  <span class="slbl">Speed</span>
  <input type="range" class="ss" id="spd-s" min="1" max="100" value="40" style="width:70px">
  <span class="sv" id="spd-v">40%</span>
  <button id="btn-realtime" class="sbtn" title="Echtzeit-Simulation" onclick="toggleRealtime()" style="padding:2px 5px"><svg class="ico"><use href="#s-clock"/></svg></button>
  <div class="sep"></div>
  <span class="slbl">Step</span>
  <input type="range" class="ss" id="pos-s" min="0" max="1" value="0" step="0.001" style="width:120px">
  <span class="sv" id="pos-v">— / —</span>
  <div class="sep"></div>
  <span class="slbl" id="tcp-vel-lbl">TCP</span>
  <span class="sv" id="tcp-vel-v" style="min-width:72px">0 mm/s</span>
  <div class="sep"></div>
  <span class="sstatus stopped" id="sim-status">STOPPED</span>
  <span style="flex:1"></span>
  <button class="sbtn" id="rib-aplot" onclick="toggleAPlot()" title="A-Winkel Map ein/ausblenden"><svg class="ico"><use href="#s-chart"/></svg>A-Plot</button>
  <button class="sbtn" onclick="setView('iso')" title="Ansicht zurücksetzen (ISO)"><svg class="ico"><use href="#s-iso"/></svg>ISO</button>
</div>
'''

# Versteckte Elemente, die simulator.js beim Start anspricht
stash = '''
<!-- Versteckte Bedienelemente: werden von simulator.js referenziert, in Simple nicht angezeigt -->
<div id="legacy-stash" style="display:none" aria-hidden="true">
  <button id="btn-robot3d" class="on"></button>
  <button id="btn-show-skeleton" class="on"></button>
  <button id="btn-tcp-trace"></button>
  <button id="btn-grid"></button>
  <button id="btn-bg"></button>
  <button id="btn-persp" class="on"></button>
  <button id="btn-steuer"></button>
  <button id="btn-aplot"></button>
  <input type="range" id="frame-size-sl" min="1" max="600" value="120"><span id="frame-size-v">120</span>
  <input type="range" id="sphere-size-sl" min="1" max="200" value="18"><span id="sphere-size-v">18</span>
  <input type="file" id="scene-file-in" accept=".zip,.json,.stp,.step">
</div>
'''

# ── 3. Hauptbereich (Editor, Viewport, Info-Panel) ──────────────────
main = between('<div class="main">', '\n<script src=')
# Original enthält ein überzähliges schließendes </div> am Ende
if main.rstrip().endswith('</div>\n\n</div>') or main.endswith('\n\n</div>'):
    main = main[:main.rindex('</div>')].rstrip() + '\n'
assert balanced(main), (main.count('<div'), main.count('</div>'))
# Format-/Postprozessor-Bedienelemente aus dem Editor-Kopf entfernen
a = main.index('      <div style="display:flex;align-items:center;gap:4px;flex-shrink:0">')
b = main.index('    </div>\n    <div class="ew">')
assert balanced(main[a:b]), 'fmt-Block nicht balanciert'
main = main[:a] + main[b:]
main = main.replace('<span id="editor-lang-label">KRL Programm</span>',
                    '<span id="editor-lang-label">KUKA KRL</span>')
# Formularansicht entfällt
fv = main.index('      <div id="fv-form-wrap"')
fve = main.index('</div>\n      </div>\n', fv) + len('</div>\n      </div>\n')
assert balanced(main[fv:fve])
main = main[:fv] + main[fve:]
# Editor-Buttons mit Text statt Emoji
main = main.replace('<button class="sbtn" onclick="saveProgram()" style="flex:1">💾</button>',
                    '<button class="sbtn" onclick="saveProgram()" style="flex:1" title="Programm als .src speichern"><svg class="ico"><use href="#s-save"/></svg>Speichern</button>')
main = main.replace('<button class="sbtn" onclick="loadProgram()" style="flex:1">📂</button>',
                    '<button class="sbtn" onclick="loadProgram()" style="flex:1" title="KRL-Datei (.src) öffnen"><svg class="ico"><use href="#s-open"/></svg>Öffnen</button>')
# Info-Panel (Kinematik, TCP, BASE, I/O …) bleibt intern, wird aber nicht angezeigt
main = main.replace('<div id="ip-resize-handle" class="ip-resize-handle"></div>',
                    '<div id="ip-resize-handle" class="ip-resize-handle" style="display:none"></div>')
i0 = main.index('<button id="ip-toggle-btn"'); i1 = main.index('>', i0)
tag = main[i0:i1]
tag = tag.replace('style="width:18px;', 'style="display:none!important;width:18px;').replace('display:flex;', '')
main = main[:i0] + tag + main[i1:]
main = main.replace('<div class="info-panel" id="info-panel">',
                    '<div class="info-panel" id="info-panel" style="display:none">')

# ── 4. Skripte (nur benötigte Module) ───────────────────────────────
scripts = '''<script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
<script src="https://cdnjs.cloudflare.com/ajax/libs/jszip/3.10.1/jszip.min.js"></script>
<script src="https://cdn.jsdelivr.net/npm/occt-import-js@0.0.23/dist/occt-import-js.js"></script>
<script src="./js/i18n.js?v=20261021a"></script>
<script src="./js/dp-solver.js?v=20261021a"></script>
<script src="./js/formats/registry.js?v=20261021a"></script>
<script src="./js/simulator.js?v=20261021a"></script>
<script src="./js/formats/kuka.js?v=20261021a"></script>
'''

# ── 5. A-Plot, Einstellungen (versteckt), Splash, Fortschritt ───────
aplot = between('<!-- A-Winkel Diagramm -->', '<!-- Einstellungsfenster -->')
settings = between('<!-- Einstellungsfenster -->', '<!-- Hilfefenster -->')
splash = between('<!-- Splash Screen -->', '<!-- ── ROBLIB Modal')
splash = splash.replace('>RobSimul</div>', '>RobSimul Simple</div>', 1)

# ── 6. Library-Auswahl: nur Roboter ─────────────────────────────────
lib = between('<!-- Library Picker Modal -->', '<!-- ═══')
t0 = lib.index('    <div id="libPickerTabs"')
t1 = lib.index('    </div>\n', t0) + len('    </div>\n')
lib = lib[:t0] + lib[t1:]
lib = lib.replace('>Library</h3>', '>Roboter wählen</h3>')

extra_css = '''<style>
.simple-hbtn{display:flex;align-items:center;gap:6px;height:26px;margin:0 10px;padding:0 12px;
  background:rgba(255,96,0,.12);border:1px solid var(--acc);border-radius:3px;color:var(--acc);
  font-family:monospace;font-size:12px;cursor:pointer}
.simple-hbtn:hover{background:rgba(255,96,0,.25)}
.simple-robot-lbl{font-family:monospace;font-size:12px;color:var(--txt2)}
.simple-robot-lbl b{color:var(--txt)}
#rib-aplot.on{border-color:var(--acc);color:var(--acc)}
</style>
'''

glue = '''<script>
// ── RobSimul Simple: Anpassungen ─────────────────────────────────
(function(){
  // Aktiven Roboternamen anzeigen
  var _origLoadLibItem = window.loadLibItem;
  window.loadLibItem = async function(type, robot){
    var lbl = document.getElementById('simple-robot-name');
    if (lbl) lbl.textContent = 'lädt ' + (robot && robot.name || '') + '…';
    try {
      await _origLoadLibItem(type, robot);
      if (lbl) lbl.textContent = robot && robot.name || '—';
    } catch(e) {
      if (lbl) lbl.textContent = 'Fehler beim Laden';
      throw e;
    }
  };
  // A-Plot-Button-Zustand spiegeln
  var _origToggleAPlot = window.toggleAPlot;
  window.toggleAPlot = function(){
    _origToggleAPlot();
    var p = document.getElementById('aplot-panel');
    var b = document.getElementById('rib-aplot');
    if (b && p) b.classList.toggle('on', p.style.display !== 'none');
  };
  var _abtn = document.getElementById('btn-aplot');
  if (_abtn) _abtn.onclick = window.toggleAPlot;
})();
</script>
'''

out = (head + header + '\n' + main + '\n' + stash + '\n' + scripts + '\n' + aplot + settings
       + splash + lib + extra_css + glue + '\n</body>\n</html>\n')
# Zeilenende der Splash-Sektion enthält ggf. bereits Teile – prüfen
assert out.count('<div') == out.count('</div>'), (out.count('<div'), out.count('</div>'))
open(DST, 'w', encoding='utf-8').write(out)
print('ok', len(out), 'bytes,', out.count('\n'), 'Zeilen')
