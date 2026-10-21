# RobSimul Simple

Reduzierte Version von [RobSimul](https://github.com/cncandi/robsimulate) (Stand V0.84).

Enthalten:
- Robotermodell aus der RobLib-Bibliothek wählen
- KUKA-KRL-Programm laden, bearbeiten und simulieren
- A-Plot (A-Winkel-Verlauf, Reachability-Map, Auto-Lösung)

Entfernt gegenüber RobSimul: weitere Roboterformate/Postprozessoren, KRL-Formularansicht,
Stationen/Positionierer/Schienen-Auswahl, Kabelsystem, Info-Panel, Menüband.
Die Roboterkonfiguration (Kinematik, TCP, BASE) läuft intern unverändert weiter.

`index.html` wurde mit einem Build-Skript aus der RobSimul-`index.html` erzeugt;
`js/simulator.js` ist bis auf das Standardformat (`kuka` statt `kuka-form`) unverändert.
