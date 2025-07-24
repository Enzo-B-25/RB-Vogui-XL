# RB-Vogui-XL

Ce dépôt regroupe les travaux réalisés sur le robot **RB-XL-Vogui** à Polytech Dijon (site du Creusot).

## Structure du projet

Les scripts liés à la détection d’objets via YOLO (You Only Look Once) sont disponibles dans le package ROS suivant :

```bash
cd ~/catkin_ws/src/robot_packages/pkg_transm_cohoma

Utilisation de YOLO
YOLO 2D

Lancement :

python3 yolo2d.py

Visualisation dans RViz :

    Ouvrir RViz :

rviz

Ajouter un affichage de type "Camera Image"

Sélectionner le topic :

    /yolo/image_annotated

[Exemple YOLO 2D](images/yolo2d_result.png)
YOLO 3D

Lancement :

python3 yolo3d.py

Préparation de l'IHM Robotnik :

    Accéder à l’IHM du robot :
    http://192.168.0.200/robotnik_hmi/index.php

    Aller dans l’onglet Control Panel → Localisation Panel

    S'assurer que le Module est désactivé (cliquer sur le bouton STOP si nécessaire)

Visualisation dans RViz :

    Ouvrir RViz :

rviz

Ajouter un affichage de type "Marker"

Sélectionner le topic :

    /yolo/marker
