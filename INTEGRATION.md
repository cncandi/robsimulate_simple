# RobSimul Simple – Integration in andere Software

Stand: 22.10.2026 · Repository `cncandi/robsimulate_simple` · Live: https://cnc-technik.de/robsimul/simple/

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
Nachteil: andere Domain → die Host-Software kann **nicht** auf Funktionen im iframe zugreifen
(Browser-Sicherheit). Für Datenaustausch (Programm hineingeben, Ergebnis zurückholen) wird eine
`postMessage`-Schnittstelle benötigt – **noch nicht eingebaut** (siehe Abschnitt 8).

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
Rückmeldungen aus dem Simulator an die Host-Software: `window.chrome.webview.postMessage(...)` – dafür
ist eine kleine Ergänzung im Simulator nötig (Abschnitt 8).

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

## 5. Unterstützter KRL-Umfang

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

Kein Server-Speicher, keine Cookies, keine Anmeldung.

## 8. Noch nicht vorhanden – bei Bedarf ergänzen

- **Nachrichten-Schnittstelle** (`postMessage` für iframe auf fremder Domain, `chrome.webview.postMessage`
  für WebView2): Befehle `loadProgram`, `loadRobot`, `setBase`, `play/pause/seek`, `getProgram` und
  Ereignisse `parsed`, `applied` (Map übernommen), `verify`.
- **Einbettungsmodus**: Kopfzeile/Toolbar per URL-Parameter ausblenden, z. B. `?embed=1`.
- **Offline-Paket**: three.js und JSZip lokal statt CDN.

## 9. Grenzen

- Ein Roboter mit Zentralhand (analytische IK), sonst numerische IK (langsamer).
- Redundanz nur über A (Drehung um Werkzeug-Z).
- Kollision nur Roboterglieder/Werkzeug gegen das bereits gedruckte Teil (vereinfacht als Quader).
- Keine externen Achsen, keine Steuerstrukturen im KRL.
