# RobSimul Simple – Integration in andere Software

Stand: 27.10.2026 · Repository `cncandi/robsimulate_simple` · Live: https://cnc-technik.de/robsimul/simple/

## 1. Was die Software ist

Reine Browser-Anwendung ohne Build-Schritt und ohne Server-Logik. Vier Dateien, rund 200 KB:

| Datei | Inhalt |
|---|---|
| `index.html` | Oberfläche, SVG-Symbole, Dialoge (18 KB) |
| `css/simulator.css` | Gestaltung (16 KB) |
| `js/simulator.js` | Kinematik, IK, KRL-Parser, Simulation, Robot Map, Bedienung (159 KB) |
| `js/formats.js` | Parser/Generatoren ABB RAPID, FANUC TP, APT CL-Data |
| `js/placement.js` | Teilposition optimieren (BASE X/Y/Drehung um Z) |
| `js/dp-solver.js` | globaler IK-Pfadplaner für Programme bis 150 Punkte (9 KB) |
| `js/host-api.js` | Host-Schnittstelle `RobSim` (Befehle, postMessage, WebView2, Einbettungsmodus) |
| `js/graphics.js` | Grafikqualität: PBR-Material, Studio-Umgebung, Schatten, ACES-Tonemapping, SSAO, FXAA |
| `js/vendor/three-r128/` | three.js-r128-Erweiterungen lokal (RoomEnvironment, EffectComposer, SSAOPass, FXAA u. a.) |

Funktionen: Roboter aus der RobLib-Bibliothek, Simulation von KUKA KRL, ABB RAPID, FANUC TP und APT, BASE-Verwaltung, Robot Map
(A-Winkel-Karte mit Achsgrenzen, Singularitäten, Kollision, grüner Weglinie, Verifikation),
Editor mit Zeilenkopplung, G-Code-Anzeige Alle/Bis Position/Aus.

## 2. Abhängigkeiten

| Abhängigkeit | Quelle | Zweck | offline nötig? |
|---|---|---|---|
| three.js r128 | `https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js` | 3D | ja, lokal ablegen |
| JSZip 3.10.1 | `https://cdnjs.cloudflare.com/ajax/libs/jszip/3.10.1/jszip.min.js` | Roboter-ZIP entpacken | ja, lokal ablegen |
| Schriften Inter, JetBrains Mono | Google Fonts | Optik | nein (Fallback-Schrift) |
| RobLib-Bibliothek | `https://cnc-technik.de/robsimul/roblib/api.php?action=list` | Roboterliste | nur für "Roboter wählen" |

Bibliothek und Roboter-ZIPs liefern `Access-Control-Allow-Origin: *`, sind also von jeder Domain aus abrufbar.

Browser: aktueller Chrome/Edge/Firefox/Safari mit WebGL. Für den Betrieb genügt statisches Hosting (oder lokale Dateien in einer WebView).

## 3. Einbau-Varianten

### A. iframe auf die Live-Version (am wenigsten Aufwand)
```html
<iframe src="https://cnc-technik.de/robsimul/simple/" style="width:100%;height:800px;border:0"></iframe>
```
Andere Domain → kein direkter Funktionszugriff (Browser-Sicherheit). Steuerung über die
`postMessage`-Schnittstelle (Abschnitt 4a).

### B. Kopie auf derselben Domain im iframe (empfohlen für Web-Software)
Die vier Dateien (plus three.js/JSZip lokal) in die eigene Anwendung kopieren, z. B. unter `/robsim/`,
und per iframe einbinden. Gleiche Domain → direkter Zugriff:
```js
const sim = document.getElementById('robsim').contentWindow;
sim.document.getElementById('code-input').value = krlText;
sim.parseAndLoad();
```
Vorteil: Simulator bleibt gekapselt (eigene IDs, CSS, Tastenkürzel), volle Steuerung per JavaScript.

### C. Desktop-Software (C#/.NET, z. B. ENCY-Extension, P3D Slicer)
WebView2 (Windows) oder CefSharp verwenden, Dateien lokal mitliefern (three.js/JSZip lokal, siehe Abschnitt 2):
```csharp
await webView.EnsureCoreWebView2Async();
webView.CoreWebView2.SetVirtualHostNameToFolderMapping("robsim.local", @"C:\...\robsim",
    CoreWebView2HostResourceAccessKind.Allow);
webView.Source = new Uri("https://robsim.local/index.html");
// Programm übergeben:
await webView.ExecuteScriptAsync(
  $"document.getElementById('code-input').value = {JsonSerializer.Serialize(krlText)}; parseAndLoad();");
// Ergebnis lesen (z. B. geänderter Code nach 'Übernehmen' in der Map):
string json = await webView.ExecuteScriptAsync("document.getElementById('code-input').value");
```
Befehle und Rückmeldungen laufen über `RobSim.cmd(...)` bzw. `PostWebMessageAsJson` /
`WebMessageReceived` (Abschnitt 4a).

### D. Direkt in die Seite der Host-Software einbauen (nicht empfohlen)
Der Code arbeitet mit globalen Funktionen/Variablen (mehrere hundert Namen), festen Element-IDs,
Seiten-Layout im `body`, globalen Tastenkürzeln (F5–F9, Pos1/Ende, Leertaste) und dokumentweiten
Ereignis-Handlern. Konflikte mit der Host-Seite sind wahrscheinlich. Wenn doch: Variante B verwenden.

## 4. Programmierschnittstelle (globale Funktionen im Simulator-Fenster)

Alle Winkel in Grad, Längen in mm. `t` = Satzposition 0 … N−1 (Kommazahl = zwischen zwei Sätzen).

### Programm
| Aufruf | Wirkung |
|---|---|
| `document.getElementById('code-input').value = krl` | KRL-Text setzen |
| `parseAndLoad()` | parsen und IK rechnen (asynchron, in Häppchen) |
| `loadProgramFiles({datei: text, …}, hauptdatei?)` | Hauptprogramm + Module laden und parsen (ABB: `Load`, `%"MODUL:PROC"%`, Prozeduraufrufe werden aufgelöst). Ohne `hauptdatei` wird die Datei mit Load/`%"…"%` gewählt |
| `robsimModuleResolver = dateinamen => ({datei: text})` | Suchfunktion der Host-Software für fehlende Module (auch als Promise). Wird nach dem Parsen mit den gesuchten Dateinamen aufgerufen (aus `Load … File:="…"` und `MODUL` + .prg/.mod/.modx/.sys); gefundene Module werden ergänzt und neu geparst |
| `progFiles` | zusätzliche Module `{datei: text}` (Hauptprogramm steht im Editor) |
| `parsedData.missing` | nicht gefundene Unterprogramme; `parsedData.modules` = verwendete Module |
| `parsedData.positions[i].src` | `{file, line}` bei Sätzen aus einem Modul (`lineNum` = Aufrufzeile im Hauptprogramm) |
| `_ikPending` | `true` solange die IK rechnet; fertig wenn `false` |
| `parsedData.positions[i]` | `{type, X,Y,Z,A,B,C, rel, lineNum, velCP, toolN, baseN}` – X…C in Weltkoordinaten (BASE eingerechnet), `rel` = Werte wie im Programm |
| `parsedData.steps` | Programmschritte inkl. `$VEL.CP`, `$OUT`, `WAIT` … |
| `ikTable[i].angles` | Achswerte A1–A6 je Satz (stetig fortgeführt; mit `_aNormQ(q)` auf die Achsgrenzen zurückführen) |
| `ikTable[i].ok` | IK-Lösung innerhalb der Achsgrenzen gefunden |
| `_ikJumps` | Liste der Achssprünge `{i, axis, deg}` (Achsgrenze erreicht) |
| `saveProgram()` | Programm herunterladen (Endung je Format: .src/.mod/.ls/.apt) |
| `setProgramFormat('kuka'|'abb'|'fanuc'|'apt')` | Format wählen; geparstes, unverändertes Programm wird umgerechnet |
| `detectProgramFormat(text, dateiname)` | Format erkennen |
| `generateProgram(fmt, parsedData, name)` | Programmtext in einem Format erzeugen |
| `progFmt` | aktuelles Format |

### Roboter
| Aufruf | Wirkung |
|---|---|
| `loadLibRobot({name, zip_url})` | Roboter aus RobLib laden (async) |
| `resetDefaultRobot()` | Standardmodell (Skelett) |
| `applyKinematicData(json, stlBuffers)` | eigenes Roboter-JSON + STL-Puffer `{dateiname: ArrayBuffer}` laden |
| `JOINTS_DEF`, `TCP_DEF`, `tcpList` | aktuelle Kinematik / Werkzeug |

### BASE
| Aufruf | Wirkung |
|---|---|
| `baseList[n] = {x,y,z,a,b,c,name}` | BASE_DATA[n+1] |
| `baseNavTo(n)` | BASE n in der Eingabe anzeigen |
| `applyBase()` | Bahn sofort verschieben, IK 0,7 s später automatisch neu |

### TCP (Werkzeuge des Roboters)
| Aufruf | Wirkung |
|---|---|
| `tcpList[n] = {x,y,z,a,b,c,name}` | TOOL_DATA[n+1], bezogen auf den Flansch |
| `tcpNavTo(n)` | TCP n in der Eingabe anzeigen |
| `applyTCP()` | Roboter sofort mit neuem TCP, IK 0,7 s später automatisch neu |

Im Programm definierte Werkzeuge (`parsedData.tools`) haben Vorrang vor `tcpList`.

### Teilposition
| Aufruf | Wirkung |
|---|---|
| `plcStart()` | Ursache ermitteln und beste Lage suchen (BASE X, Y, A; Z bleibt), Ergebnis im Base-Bereich |
| `_plc.res` | `{bn, base:{x,y,z,a,b,c}, before, after, diag}` – `before/after`: `{bad, badProg, jumps, n, cats}` |
| `plcApply()` / `plcUndo()` | Vorschlag übernehmen / zurücknehmen |

### Simulation und Ansicht
| Aufruf | Wirkung |
|---|---|
| `startPlay(1)` / `startPlay(-1)` / `pauseSim()` | abspielen vor/zurück, anhalten |
| `applySimT(t)` | auf Satzposition springen |
| `toggleRealtime()` | Echtzeit ($VEL.CP) ein/aus |
| `setPathMode('all'|'upto'|'none')` | G-Code-Anzeige |
| `setView('vorne'|'hinten'|'links'|'rechts'|'oben'|'unten'|'iso')`, `zoomBy(f)`, `fitView()` | Ansicht |
| `setProjection('persp'|'ortho')`, `setRenderMode('shaded'|'wireframe')` | Projektion, Darstellung |
| `setAPlot(true|false)` | A-Plot ein/aus |
| `toggleRightPanel()`, `toggleLeftPanel()` | Seitenbereiche ein/aus |

### Robot Map
| Aufruf | Wirkung |
|---|---|
| `toggleAPlot()` | Map ein/aus |
| `aPlotStartScan()` | Map erzeugen (wartet ggf. auf die IK), legt danach automatisch die Linie an |
| `aPlotAutoPath()` / `aPlotPathFromProgram()` | grüne Linie automatisch / aus dem Programm |
| `_aPath` | Linie als Knoten `[{d: Bahnlänge mm, a: A-Wert}]` |
| `_aVerify` | Verifikation `{bad: Satzindizes, kind: 1=unerreichbar 2=Achsgrenze 3=Singularität 4=Achssprung 5=Kollision}` |
| `aPlotApply()` | A-Werte der Linie ins Programm schreiben (ändert `code-input`, parst neu) |

## 4a. Host-Steuerung `RobSim` (Ansicht, Darstellung, Anzeige)

Ziel: Die Host-Software hat **eigene** Schaltflächen (oder Menü/Tastenkürzel) und schickt Befehle an das Modul.
Die Toolbar des Moduls wird dazu ausgeblendet, damit keine Bedienelemente doppelt vorhanden sind.

### Bereiche einzeln ein-/ausblenden
Jeder Bereich lässt sich unabhängig ein- und ausblenden, per URL beim Start oder per Befehl `ui` zur Laufzeit.

| Bereich | Inhalt |
|---|---|
| `header` | Kopfzeile (Logo, Roboter wählen, Öffnen, Speichern) |
| `toolbar` | ganze Toolbar |
| `tb-view` `tb-proj` `tb-render` `tb-zoom` `tb-path` `tb-aplot` | einzelne Toolbar-Gruppen (Ansichten, Perspektive/Ortho, Schattiert/Draht, Zoom/Einpassen, G-Code-Anzeige, A-Plot-Schalter) |
| `editor` | Programm-Editor links |
| `sim` | Abschnitt Simulation (rechts) |
| `base` | Abschnitt BASE (rechts) |
| `tcp` | Abschnitt TCP (rechts) |
| `statusbar` | Statuszeile unten |
| `legend` | Achsenlegende X/Y/Z im 3D-Fenster |
| `hint` | Maushinweis im 3D-Fenster |
| `aplot` | A-Plot / Robot Map |

Gruppen: `panel` = `sim`,`base`,`tcp`; `tb-all` = alle Toolbar-Gruppen; `all` = alle Bereiche (außer `aplot`).
Der rechte Bereich verschwindet automatisch, wenn `sim`, `base` und `tcp` aus sind, und erscheint, sobald einer davon an ist.

| URL-Parameter | Wirkung |
|---|---|
| `embed=1` | nur 3D-Fenster mit Statuszeile: `header`, `toolbar`, `editor`, `sim`, `base`, `tcp` aus |
| `hide=a,b,…` | diese Bereiche aus |
| `show=a,b,…` | diese Bereiche an (wird nach `embed` und `hide` angewendet) |
| `theme=dark\|light\|industrial` | Theme (wie P3D-Slicer-Online: Dunkel, Hell, Industrie) |
| `quality=standard\|high\|max` | Grafikqualität (siehe unten) |
| `view=…`, `projection=…`, `render=…`, `path=…`, `aplot=1` | Startzustand |

Beispiele:
- nur 3D: `index.html?embed=1`
- 3D + TCP-Abschnitt: `index.html?embed=1&show=tcp`
- 3D + Simulation + BASE, ohne Statuszeile: `index.html?embed=1&show=sim,base&hide=statusbar`
- volle Oberfläche ohne Ansichts-Gruppe: `index.html?hide=tb-view`

Zur Laufzeit: `RobSim.cmd('ui', 'tcp', 'on')`, `RobSim.cmd('ui', 'sim,base', 'off')`, `RobSim.cmd('ui', 'editor', 'toggle')`.

### Mausbelegung im 3D-Fenster (fest)
| Taste | Funktion |
|---|---|
| rechte Taste ziehen | Drehen |
| mittlere Taste ziehen | Verschieben |
| Mausrad | Zoom |
| linke Taste | keine Kamerafunktion (frei für die Host-Software) |

### Befehle
Aufruf: `RobSim.cmd(befehl, wert[, wert2])`. Rückgabe: aktueller Zustand (siehe unten), bei `loadProgram` ein Promise.
Ungültige Werte lösen einen Fehler aus (`Error('unbekannte Ansicht: …')`).

| Befehl | Werte | Toolbar-Entsprechung |
|---|---|---|
| `view` | `oben` `unten` `links` `rechts` `vorne` `hinten` `iso` (auch `top` `bottom` `left` `right` `front` `back`) | Ansicht oben/unten/links/rechts/vorne/hinten/isometrisch |
| `projection` | `persp` \| `ortho` \| `toggle` | Perspektive / Orthografisch |
| `render` | `shaded` \| `wireframe` \| `toggle` | Schattiert / Drahtmodell |
| `zoom` | `in` \| `out` \| Faktor (z. B. `0.8` = näher, `1.25` = weiter) | Zoom + / Zoom − |
| `fit` | – | Alles in Bild (Einpassen) |
| `theme` | `dark` \| `light` \| `industrial` \| `toggle` (auch `dunkel` `hell` `industrie`) | Darstellung wechseln (Kopfzeile) |
| `quality` | `standard` \| `high` \| `max` \| `toggle` | Grafikqualität (Toolbar „Grafik“) |
| `path` | `all` \| `upto` \| `none` | Anzeige G-Code / G-Code bis Position / Anzeige aus |
| `aplot` | `on` \| `off` \| `toggle` | Anzeige A-Plot (Robot Map) |
| `ui` | Bereich oder Liste (`"sim,base"`), Wert `on` \| `off` \| `toggle` | Bereiche ein-/ausblenden (Liste siehe „Bereiche einzeln ein-/ausblenden“) |
| `getState` | – | Zustand abfragen |

#### Editor (Programm übergeben)
Die Host-Software kann den Editor des Moduls ausblenden (`ui editor off` bzw. `hide=editor`) und ihren eigenen Editor verwenden.
Zeilennummern sind 1-basiert.

| Befehl | Werte | Wirkung |
|---|---|---|
| `loadProgram` | Text, Dateiname (optional) – oder `{datei: text}` für Hauptprogramm + Module | setzen + parsen; Promise erfüllt sich nach der IK (Zustand) |
| `setProgram` | Text, Dateiname (optional) | nur setzen, nicht parsen; Simulation ist gesperrt (`dirty: true`), bis `parse` |
| `parse` | – | parsen + IK (wie F9); Promise erfüllt sich nach der IK |
| `getProgram` | – | Programmtext (z. B. nach „Übernehmen“ in der Map oder „Teilposition übernehmen“) |
| `setFormat` | `kuka` \| `abb` \| `fanuc` \| `apt` | Format; ein geparstes Programm wird umgerechnet |
| `gotoLine` | Zeile | Roboter an die Position dieser Programmzeile (Cursor im Host-Editor → Simulation) |
| `getLine` | – | aktuelle Programmzeile der Simulation |

Ereignisse für den Host-Editor: `line` `{line, t, step}` bei jedem Zeilenwechsel der Simulation (Zeile im Host-Editor
markieren) und `program` `{text, dirty}`, wenn das Modul den Programmtext selbst ändert.

#### Simulation
| Befehl | Werte | Wirkung |
|---|---|---|
| `play` | `fwd` \| `back` | abspielen (Fehler, wenn das Programm nach einer Änderung nicht geparst ist) |
| `pause` / `stop` | – | anhalten |
| `start` / `end` | – | zum Anfang / Ende |
| `step` | `fwd` \| `back` | einen Satz vor / zurück |
| `seek` | Satzposition `t` (0 … N−1, Kommazahl erlaubt) | Position setzen |
| `seekPercent` | 0 … 100 | Position in Prozent |
| `speed` | 1 … 100 | Tempo (Schieberegler) |
| `realtime` | `on` \| `off` \| `toggle` | Echtzeit mit programmierter Geschwindigkeit ($VEL.CP) |

#### BASE (Nummer n = BASE_DATA[n], 1-basiert)
Werte: `{x, y, z, a, b, c, name}` in mm/Grad, bezogen auf den Roboterfuß. Fehlende Felder bleiben beim Ändern
erhalten, bei neuen Einträgen 0. Nach jeder Änderung rechnet das Modul die IK automatisch neu (Ereignis `state`, `ikPending`).

| Befehl | Werte | Wirkung |
|---|---|---|
| `getBases` | – | `{active, list:[…]}` |
| `setBase` | n, Werte | BASE n ändern; n = Anzahl+1 legt eine neue an |
| `setBases` | Liste | alle BASE ersetzen |
| `selectBase` | n | BASE n in der Eingabe anzeigen |
| `deleteBase` | n | BASE n löschen |
| `optimizeBase` | – | Teilposition optimieren (Promise): `{found:true, result:{bn, base, before, after, diag}}` oder `{found:false, message}` |
| `applyOptimizedBase` / `undoOptimizedBase` | – | Vorschlag übernehmen / zurücknehmen (Promise nach IK) |

#### TCP (Nummer n = TOOL_DATA[n], bezogen auf den Flansch)
| Befehl | Werte | Wirkung |
|---|---|---|
| `getTools` | – | `{active, list:[…], program:{nr: {…}}}` – `program` = im Programm definierte Werkzeuge (haben Vorrang) |
| `setTool` | n, Werte | TCP n ändern; n = Anzahl+1 legt einen neuen an |
| `setTools` | Liste | alle TCP ersetzen |
| `selectTool` | n | TCP n in der Eingabe anzeigen |
| `deleteTool` | n | TCP n löschen (mindestens einer bleibt) |

#### Roboter (RobLib)
| Befehl | Werte | Wirkung |
|---|---|---|
| `robotLibrary` | – \| `close` | Auswahldialog der Roboterbibliothek im Modul öffnen / schließen (Auswahl wird im Browser gespeichert) |
| `loadRobot` | `{name, zip_url}` \| `default` | Roboter aus RobLib laden (nur HTTPS) bzw. Standardmodell; Promise nach IK |
| `getRobot` | – | `{name, zip_url}` des aktuellen Roboters (`zip_url` = `null` beim Standardmodell) |

Der Zustand enthält `robot` (Name des aktuellen Roboters). Die Host-Software kann damit z. B. je Maschine
`getRobot` speichern und später mit `loadRobot` wiederherstellen.

Zustand (`RobSim.getState()` und Ergebnis jedes Befehls):
```json
{"api":1,"theme":"industrial","quality":"high","view":"iso","projection":"persp","render":"shaded","path":"all","aplot":false,
 "ui":{"header":false,"toolbar":false,"editor":false,"sim":false,"base":false,"tcp":true,"statusbar":true,"legend":true,"hint":true,
       "tb-view":true,"tb-proj":true,"tb-render":true,"tb-zoom":true,"tb-path":true,"tb-aplot":true,"panel":true,"aplot":false},
 "format":"kuka","points":120,"steps":130,"t":0,"step":0,"line":1,
 "playing":false,"dir":1,"speed":40,"realtime":false,"status":"STOPPED","dirty":false,"ikPending":false,
 "robot":"Standardmodell","base":{"active":1,"count":1},"tcp":{"active":1,"count":1}}
```
`view` ist `null`, sobald der Nutzer die Ansicht mit der Maus gedreht hat. Damit kann die Host-Software ihre
Schaltflächen (aktiv/inaktiv) synchron halten.

### Drehachse E1 (mathematisch gekoppelt, z. B. Schnecken auf Rohling drucken)
KRL mit gekoppelter Zusatzachse wird erkannt: `$BASE=EK(MACHINE_DEF[2].ROOT,MACHINE_DEF[2].MECH_TYPE,BASE_DATA[n])`
und Sätze `LIN {X …,Y …,Z …,A …,B …,C …,E1 …}`. Die Positionen stehen im mitdrehenden Werkstück-KS (BASE n);
Welt = BASE · Rot(E1) · Position. Bahn, Zielpunkte und Rohling drehen in der Simulation mit E1, der Roboter folgt.
Konfiguration per Kopfkommentar `;P3D-ROTARY AXIS=X BASE=2 D0=60.3 D1=60.3 L=500` (D1 = Ø am Rohlingende, konisch) oder Befehl:

| Befehl | Wirkung |
|---|---|
| `rotaryAxis({on, axis:'X'\|'Y'\|'Z', base, d0, d1, L, path:'upto'}, 'noparse'?)` | Drehachse = X/Y/Z von BASE n, Rohling Ø d0 (Anfang) bis d1 (Ende) × L; ohne `'noparse'` wird das geladene Programm neu geparst |
| `rotaryAxis(null)` | nur Programm (EK/Kopfkommentar) entscheidet |
| `rotaryAxis()` | Zustand `{on, axis, base, d0, L, e1, note}` (auch in `getState().rotary`) |

Ist BASE n noch nicht eingemessen (alle Werte 0), setzt der Simulator eine Standardlage vor dem Roboter.
Kollisionsprüfung: Teil = Zylinder um die Achse bis zum Düsenradius des Satzes.

### Ereignisse
| Ereignis | Wann | Daten |
|---|---|---|
| `ready` | Modul geladen, Befehle möglich | Zustand |
| `state` | Zustand geändert (auch durch Maus, Modul-Bedienung oder laufende Simulation; höchstens 5× je Sekunde) | Zustand |
| `parsed` | `loadProgram`/`parse` fertig (IK berechnet) | Zustand |
| `line` | Programmzeile der Simulation gewechselt | `{line, t, step}` |
| `program` | Programmtext im Modul geändert | `{text, dirty}` |

Gleiche Domain: `RobSim.on('state', st => …)`. Fremde Domain / WebView2: als Nachricht (siehe unten).

### Variante 1: gleiche Domain (iframe) oder WebView2 per `ExecuteScriptAsync`
```js
const sim = document.getElementById('robsim').contentWindow;
sim.RobSim.cmd('view', 'oben');
sim.RobSim.cmd('projection', 'ortho');
sim.RobSim.on('state', st => updateMyButtons(st));
await sim.RobSim.cmd('loadProgram', krlText, 'teil.src');
```
```csharp
await webView.ExecuteScriptAsync("RobSim.cmd('render','wireframe')");
```

### Variante 2: Nachrichten (iframe fremde Domain, WebView2, CefSharp)
Anfrage an das Modul:
```json
{"robsim": 1, "id": "42", "cmd": "view", "args": ["oben"]}
```
Antwort des Moduls (gleiche `id`):
```json
{"robsim": 1, "id": "42", "ok": true, "result": { …Zustand… }}
{"robsim": 1, "id": "43", "ok": false, "error": "unbekannte Ansicht: xyz"}
```
Ereignis vom Modul (ohne `id`):
```json
{"robsim": 1, "event": "state", "data": { …Zustand… }}
```
Nachrichten werden nur vom einbettenden Fenster (`window.parent`) bzw. von WebView2 angenommen.
Statt `args` ist auch `"value": "oben"` erlaubt. Strings mit JSON werden ebenfalls angenommen.

Web-Host (iframe):
```js
const frame = document.getElementById('robsim');
let n = 0; const pending = {};
function robsim(cmd, ...args) {
  return new Promise((ok, fail) => {
    const id = String(++n); pending[id] = r => r.ok ? ok(r.result) : fail(new Error(r.error));
    frame.contentWindow.postMessage({ robsim: 1, id, cmd, args }, '*');
  });
}
window.addEventListener('message', e => {
  if (e.source !== frame.contentWindow || !e.data || !e.data.robsim) return;
  const m = e.data;
  if (m.id && pending[m.id]) { pending[m.id](m); delete pending[m.id]; }
  else if (m.event === 'ready' || m.event === 'state') updateMyButtons(m.data);
});
// Host-Schaltflächen:
btnTop.onclick   = () => robsim('view', 'oben');
btnOrtho.onclick = () => robsim('projection', 'ortho');
btnWire.onclick  = () => robsim('render', 'wireframe');
btnFit.onclick   = () => robsim('fit');
btnPathUpto.onclick = () => robsim('path', 'upto');
btnAPlot.onclick = () => robsim('aplot', 'toggle');
```
Vor dem ersten Befehl auf das Ereignis `ready` warten.

WebView2 (C#):
```csharp
webView.Source = new Uri("https://robsim.local/index.html?embed=1");
webView.CoreWebView2.WebMessageReceived += (s, e) => {
    var msg = JsonDocument.Parse(e.WebMessageAsJson).RootElement;
    if (msg.TryGetProperty("event", out var ev) && ev.GetString() == "state")
        UpdateButtons(msg.GetProperty("data"));          // view, projection, render, path, aplot …
};
void RobSim(string cmd, params object[] args) =>
    webView.CoreWebView2.PostWebMessageAsJson(
        JsonSerializer.Serialize(new { robsim = 1, id = Guid.NewGuid().ToString(), cmd, args }));
// Host-Schaltflächen:
RobSim("view", "iso"); RobSim("projection", "persp"); RobSim("render", "shaded");
RobSim("zoom", "in");  RobSim("path", "none");       RobSim("aplot", "on");
```

### Zuordnung Host-Schaltfläche → Befehl
| Host-Schaltfläche | Befehl |
|---|---|
| Ansicht oben / unten / links / rechts / vorne / hinten / iso | `view` `oben` / `unten` / `links` / `rechts` / `vorne` / `hinten` / `iso` |
| Perspektive / Orthografisch | `projection` `persp` / `ortho` |
| Schattiert / Drahtmodell | `render` `shaded` / `wireframe` |
| Zoom + / Zoom − | `zoom` `in` / `out` |
| Alles in Bild | `fit` |
| Theme der Host-Software übernehmen | `theme` `dark` / `light` / `industrial` |
| Anzeige A-Plot | `aplot` `toggle` (oder `on`/`off`) |
| Anzeige G-Code / G-Code bis / Anzeige aus | `path` `all` / `upto` / `none` |
| Play / Rückwärts / Stop | `play` `fwd` / `back`, `pause` |
| Anfang / Ende / Satz vor / Satz zurück | `start` / `end` / `step` `fwd` / `step` `back` |
| Positionsregler / Tempo / Echtzeit | `seekPercent` / `speed` / `realtime` |
| Cursor im Host-Editor | `gotoLine` |
| Parse & Load | `parse` (oder `loadProgram` mit Text) |
| BASE / TCP-Eingabe | `setBase` / `setTool` |
| Teilposition optimieren | `optimizeBase`, danach `applyOptimizedBase` |

## 4b. Grafikqualität

| Stufe | Inhalt | Einsatz |
|---|---|---|
| `standard` | bisherige Darstellung (Phong-Material, ohne Schatten) | schwache Rechner, sehr große Zellen |
| `high` (Standard) | PBR-Material (`MeshPhysicalMaterial`, Lack mit Klarlack, graue Teile metallisch), Studio-Umgebung (`RoomEnvironment`, keine HDR-Datei nötig), weiche Schatten auf einem Schattenboden, ACES-Tonemapping, sRGB-Ausgabe | normaler Betrieb |
| `max` | wie `high` + Ambient Occlusion (SSAO, Kontaktschatten in Ritzen) + FXAA | Präsentation, Screenshots |

In allen Stufen werden STL-Normalen mit Knickwinkel 32° geglättet: Rundungen wirken glatt, scharfe Kanten bleiben scharf.
Linien, Zielpunkte, Koordinatensysteme und Beschriftungen behalten in allen Stufen ihre exakten Farben.
Kein Bloom: Er würde dünne Bahnlinien und Markierungen überstrahlen.



Bewegungen: `PTP {…}` (kartesisch oder `A1…A6`), `LIN {…}`, `SLIN {…}`, `CIRC {Hilfspunkt},{Zielpunkt}`,
`LIN_EXTRUDE({…}[,n])`, `CIRC_EXTRUDE({…},{…}[,n])` (Extrusionsparameter wird ignoriert).
Daten: `$TOOL=TOOL_DATA[n]`, `$BASE=BASE_DATA[n]`, `$VEL.CP`, `$VEL.PTP`, `$ACC.CP`, `$advance`,
`$OUT[n]=…`, `$IN[n]`, `$ANOUT[n]=…`, `$ANIN[n]`, `WAIT SEC`, `WAIT FOR`, `HALT`, `BRAKE`, `DECL`/Zuweisungen.
Nicht ausgewertet: Steuerstrukturen (IF/LOOP/FOR …), Unterprogramme, Interrupts, eigene Befehle wie `SET_TCP_SPEED`.

### Weitere Formate (js/formats.js)
- **ABB RAPID**: `MoveL/MoveJ/MoveC/MoveAbsJ`, benannte und eingebettete robtargets, `Offs()`, speeddata (`vNNN`, eigene), `toolN`, `\WObj:=wobjN`, `SetDO`, `WaitDI`, `WaitTime`, `Stop`. Quaternion → A,B,C.
- **FANUC TP (.ls)**: `/POS`-Abschnitt (UF, UT, X…R oder J1…J6), `J/L/C P[n]` mit mm/sec, cm/min, inch/min, %; `UTOOL_NUM`, `UFRAME_NUM`, `DO[]`, `WAIT DI[]`, `WAIT sec`, `PAUSE`. W,P,R → C,B,A.
- **APT CL-Data**: `GOTO` (mit Werkzeugachse i,j,k), `FROM`, `RAPID`, `FEDRAT` (MMPM/IPM), `CIRCLE`, `LOADTL`, `TLAXIS`, `UNITS`. Koordinaten = BASE 1. APT hat keine Drehung um die Werkzeugachse → A = 0, mit der Robot Map optimieren; „Übernehmen“ schreibt dann KUKA.
- **ABB-Programmablauf**: PROC main des Hauptprogramms, Prozeduraufrufe, `%"MODUL:PROC"%`/`%stringvar%`, `CallByVar`; `IF`/`TEST` → erster Zweig, `FOR` mit Zahlengrenzen (max. 1000 Durchläufe), `WHILE` einmal; `Load`/`Unload` werden übersprungen (Module müssen mit übergeben werden); ERROR-Handler werden nicht simuliert. Fehlende Module werden gesucht: zuerst über `robsimModuleResolver`, in Chrome/Edge sonst im Programmordner (einmalige Ordnerfreigabe beim Öffnen, wird im Browser gespeichert; durchsucht 2 Unterebenen). In Firefox/Safari beim Öffnen alle Dateien gemeinsam wählen.
- **Werkzeuge/Werkobjekte im Programm**: ABB `tooldata` (tframe) und `wobjdata` (uframe∘oframe) sowie KRL `TOOL_DATA[n]={X…}` werden für die Simulation übernommen und haben Vorrang vor den Werkzeugen des Roboters. `tool0` = Flansch, `wobj0` = Welt. Werkzeugrahmen: Z aus dem Flansch (ABB tool0 = KUKA FLANGE). wobjdata erscheinen als BASE-Einträge mit ihrem Namen; „Teilposition optimieren“ schreibt die neue Lage in die wobjdata-Deklaration. `parsedData.tools` = `{nr: {x,y,z,a,b,c,name}}` (ABB-Werkzeuge ab Nr. 101, tool0 = 100). Die Fußzeile zeigt das aktive Werkzeug.
- Umschalten des Formats erzeugt das Programm neu: Kommentare und nicht ausgewertete Zeilen gehen dabei verloren.

## 6. Roboterformat (RobLib-ZIP)

ZIP mit einer JSON-Datei und STL-Dateien (`a1.stl` … `a6.stl`, `podest.stl`, `tool1_tcp.stl`):
```json
{
  "name": "KR8",
  "joints": [{"name":"A1","axis":"Rz","offset":{"x":150,"y":0,"z":450},"min":-170,"max":170}, …],
  "stlRefAngles": [0,-90,90,0,0,0],
  "stlFiles": {"A1": {"name":"a1","color":"#ffffff"}, …},
  "sceneModels": {"pedestal": {"name":"podest"}, "tool": {"name":"tool1_tcp"}},
  "tcp": {"x":364.5,"y":0,"z":46.5,"a":0,"b":90,"c":0},
  "tcpList": [ … optional, Werkzeug n = Eintrag n-1 … ],
  "baseList": [ … optional … ]
}
```
Bibliotheksliste: `GET …/roblib/api.php?action=list` → `{ok, count, robots:[{id,type,name,marke,modell,zip_url,thumb_url,…}]}`
(nur Einträge mit `type` = `robot` sind Roboter).

## 7. Gespeicherte Zustände (Browser)

| Schlüssel (localStorage) | Inhalt |
|---|---|
| `robsimSimple.robot` | zuletzt gewählter Roboter `{name, zip_url}` oder `{name:'default'}` |
| `robsimSimple.ui` | ausgeblendete Bereiche |
| `robsimSimple.fmt` | gewähltes Programmformat |
| `robsimSimple.theme` | gewähltes Theme (Standard `industrial`) |
| `robsimSimple.quality` | Grafikqualität (Standard `high`) |

Kein Server-Speicher, keine Cookies, keine Anmeldung.

## 8. Noch nicht vorhanden – bei Bedarf ergänzen

- Robot-Map-Befehle (Scan, Auto-Weg, Übernehmen) und Ereignis `verify`
  (Ansicht, Editor, Simulation, BASE, TCP, Roboter sind vorhanden, Abschnitt 4a).
- **Offline-Paket**: three.js und JSZip lokal statt CDN.

## 9. Grenzen

- Ein Roboter mit Zentralhand (analytische IK), sonst numerische IK (langsamer).
- Redundanz nur über A (Drehung um Werkzeug-Z).
- Kollision nur Roboterglieder/Werkzeug gegen das bereits gedruckte Teil (vereinfacht als Quader).
- Keine externen Achsen, keine Steuerstrukturen im KRL.
