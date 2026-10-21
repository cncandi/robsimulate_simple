# RobSimul Simple

Reduzierte Fassung von [RobSimul](https://github.com/cncandi/robsimulate) (Basis V0.84).
Live: https://cnc-technik.de/robsimul/simple/

## Funktionsumfang
- Robotermodell aus der RobLib-Bibliothek wählen (Kinematik, Achs-STLs, Podest, Werkzeug, TCP/BASE-Daten)
- KUKA-KRL-Programm bearbeiten, als .src öffnen/speichern, parsen (F9)
- Simulation: Play vor/zurück, Einzelschritt, Anfang/Ende, Geschwindigkeit, Echtzeit, Positionsregler
- Robot Map (A): Karte der A-Werte entlang der Bahn (links Beginn, rechts Ende) mit Feldern für unerreichbar,
  Achsgrenze und Singularität. Achsstellungen laufen stetig entlang der Bahn, Achsgrenzen (z. B. A6 ±180°) erscheinen dort,
  wo die Achse zurückdrehen müsste. Grüne Linie wird automatisch um die Störfelder gelegt (Sicherheitsabstand einstellbar),
  Knoten verschiebbar / einfügbar / löschbar. Verifikation prüft jeden Programmpunkt mit dem A der Linie.
  "Übernehmen" schreibt A für alle Punkte ins KRL.
- Ellbogen-Singularität: Streckwinkel aus den Achsmaßen berechnet (KR8 ca. 18°), nicht pauschal A3 = ±90°.
- 3D-Ansicht: Standardansichten Vorne/Hinten/Links/Rechts/Oben/Unten/Isometrisch, Perspektive/Orthogonal,
  Zoom +/−, Einpassen; Maus: Drehen (links), Verschieben (rechts/Mitte), Zoom (Rad)
- Oberfläche im Stil von P3D-SO (Farben, Toolbar mit Icon über Label, Panels, Statusleiste)

## Dateien
- `index.html` – Oberfläche
- `css/simulator.css` – Styles
- `js/simulator.js` – Kinematik, IK, KRL-Parser, Simulation, A-Plot
- `js/dp-solver.js` – globaler IK-Pfadplaner

Die Rechenkerne (FK/IK, Trajektorie, DP-Planer, Reachability-Scan) sind unverändert aus RobSimul übernommen;
die Ergebnisse wurden gegen RobSimul V0.84 abgeglichen.

## Deploy
Push auf `main` → GitHub Actions → FTP nach `/simple/` (cnc-technik.de/robsimul/simple/).
