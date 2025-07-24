#RB-Vogui-XL

Ce dépôt regroupe les travaux réalisés sur le robot RB-XL-Vogui à Polytech Dijon (site du Creusot).
Structure du projet

Les scripts liés à la détection d’objets via YOLO (You Only Look Once) sont disponibles dans le package ROS suivant :

catkin_ws/src/robot_packages/pkg_transm_cohoma

Utilisation de YOLO
YOLO 2D

Lancement :

cd ~/catkin_ws/src/robot_packages/pkg_transm_cohoma
python3 yolo2d.py

Visualisation dans RViz :

    Ouvrir RViz :

rviz

Ajouter un affichage de type "Camera Image".

Sélectionner le topic :

    /yolo/image_annotated

    (Une capture d'écran de RViz avec l'image annotée peut être insérée ici :)
    ![Exemple YOLO 2D](images/yolo2d_result.png)

YOLO 3D

Lancement :

cd ~/catkin_ws/src/robot_packages/pkg_transm_cohoma
python3 yolo3d.py

Préparation de l'IHM Robotnik :

    Ouvrir l’IHM du robot à l'adresse :
    http://192.168.0.200/robotnik_hmi/index.php

    Aller dans l’onglet Control Panel → Localisation Panel.

    Vérifier que le Module n’est pas actif. Si nécessaire, appuyer sur le bouton STOP (rouge).

Visualisation dans RViz :

    Ouvrir RViz :

rviz

Ajouter un affichage de type "Marker".

Sélectionner le topic :

    /yolo/marker

    (Une capture d'écran montrant les bounding boxes 3D dans RViz peut être insérée ici :)
    ![Exemple YOLO 3D](images/yolo3d_result.png)
