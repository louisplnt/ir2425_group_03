#!/bin/bash

PACKAGE_NAME="assignment_1"
SCRIPT_DIR="$HOME/catkin_ws/src/ir2425_group_03/$PACKAGE_NAME/src"

# Vérifier si le dossier existe
if [ ! -d "$SCRIPT_DIR" ]; then
    echo "Le dossier $SCRIPT_DIR n'existe pas. Vérifiez votre package."
    exit 1
fi

# Trouver les fichiers exécutables
cd "$SCRIPT_DIR" || exit
SCRIPTS=$(find . -type f -executable -print)

# Vérifier s'il y a des fichiers exécutables
if [ -z "$SCRIPTS" ]; then
    echo "Aucun fichier exécutable trouvé dans $SCRIPT_DIR"
    exit 1
fi

# Ouvrir chaque script dans un onglet distinct du terminal
for script in $SCRIPTS; do
    script_name=$(basename "$script")
    gnome-terminal --tab -- bash -c "source ~/.bashrc; echo 'Lancement de : rosrun $PACKAGE_NAME $script_name'; rosrun $PACKAGE_NAME $script_name; exec bash"
    sleep 0.5  # Petite pause pour éviter les ouvertures simultanées
done
