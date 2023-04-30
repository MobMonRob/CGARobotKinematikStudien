# CGARobotKinematikStudien
In dieser Studienarbeit wurde eine inverse Kinematik für den UR5e umgesetzt.
Die Datei InverseKinematikUR5e.clu beinhaltet ein GAALOPScript zur Visualisierung der geometrischen Objekte.
Die anschließende Berechnung der Winkel ist derzeit auskommentiert, da hier noch einige Fehler vorhanden sind.

Aus dem GAALOPScript wurde ein Python-Code generiert, welcher in der Datei ik_ur5e.py inkludiert ist.
Das Python-Script liest die Posen aus der Datei poses.txt ein und berechnet anschließend die geometrischen Objekte der inversen Kinematik anhand des generierten Codes.
Die geometrischen Objekte der letzten Pose werden dabei in die Datei multivectors.csv geschrieben.
