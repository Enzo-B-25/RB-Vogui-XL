#!/bin/bash

TARGET_ID="046d:0825"
USBRESET="./usbreset"

# Lire lsusb ligne par ligne
while read -r line; do
    # On teste si la ligne contient l’ID cible
    if [[ "$line" == *"$TARGET_ID"* ]]; then
        echo "Trouvé : $line"

        # Extraire le Bus et le Device à la main
        BUS=$(echo "$line" | cut -d ' ' -f 2)
        DEV_RAW=$(echo "$line" | cut -d ' ' -f 4)
        DEV=${DEV_RAW%:}  # Enlève le ":" à la fin

        DEVICE_PATH="/dev/bus/usb/$BUS/$DEV"
        echo "Chemin détecté : $DEVICE_PATH"

        # Vérifie que le fichier existe avant de reset
        if [ ! -e "$DEVICE_PATH" ]; then
            echo "Erreur : $DEVICE_PATH n'existe pas"
            echo "Contenu de /dev/bus/usb/$BUS :"
            ls /dev/bus/usb/$BUS/
            exit 1
        fi

        echo "Réinitialisation de $DEVICE_PATH"
        sudo "$USBRESET" "$DEVICE_PATH"
        exit 0
    fi
done < <(lsusb)

echo "Périphérique $TARGET_ID non trouvé"
exit 1

