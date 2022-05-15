# Miniprojet: Robocop
Repo contenant le code source pour notre miniprojet de robotique (MICRO-315), réalisé lors du semestre de printemps 2022.

Le but était de créer un radar de vitesse/une voiture de police: notre robot détecte un objet bruyant ayant une vitesse supérieure à une vitesse seuil, et se met à sa poursuite après avoir activé LEDs et sirènes.

Ce code a été réalisé pour un epuck-2 avec [sa librairie](https://www.gctronic.com/doc/index.php?title=e-puck2_robot_side_development).

### Mode d'emploi
Reset le robot devant une référence, idealement de couleur blanche, et ne rien faire pendant 2 secondes pour laisser le temps au robot de s'initialiser. C'est tout! Dès qu'un objet ayant une vitesse supérieure à 7 cm/s passe devant le robot, il se mettra à sa poursuite. Il faut s'assurer que l'objet produise un bruit, idéalement sinusoïdal.

### Defines et options de compilation
- `OBJECT_LENGTH` dans _radar.c_: le programme détecte un objet ayant une largeur définie dans ce define. Par défaut, il détecte un autre epuck-2 de largeur 7.3 cm.
- `MAX_SPEED` dans _radar.c_: vitesse seuil. Sans ce define, le robot ne détecte plus une vitesse en cm/s mais plutôt en nbr de mesures/s. Par défaut, `MAX_SPEED` est define à 7 cm/s.
- `COMPUTE_SIGNED_ANGLE` dans _audio_processing.c_: avec ce define, le robot peut mesurer un angle entre lui et une source de son peu importe son orientation. Sans ce define, le robot ne peut que mesurer un angle compris entre 0 et 180 degrés. Par défaut, `COMPUTE_SIGNED_ANGLE` n'est pas defined.
