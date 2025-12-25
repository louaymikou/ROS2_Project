# 🗺️ GUIDE COMPLET POUR CRÉER UNE BONNE CARTE SLAM

## ⚠️ PROBLÈMES COURANTS ET SOLUTIONS

### Problème : Plusieurs cartes superposées
**Causes :**
- Robot déplace trop vite
- Odométrie qui dérive
- Loop closure mal configuré
- Anciennes données de pose dans le cache

**Solutions appliquées :**
✅ Configuration SLAM optimisée (paramètres plus stricts)
✅ Script de nettoyage créé
✅ Instructions détaillées ci-dessous

---

## 📝 PROCÉDURE ÉTAPE PAR ÉTAPE

### PRÉPARATION

#### 1. Nettoyer l'environnement
```bash
cd ~/ROS2_Project
chmod +x clean_and_map.sh
./clean_and_map.sh
```

Ce script va :
- Tuer tous les processus ROS
- Supprimer les anciennes cartes
- Nettoyer le cache SLAM
- Proposer de reconstruire le projet

---

### CRÉATION DE LA CARTE

#### 2. Lancer la simulation SLAM (Terminal 1)
```bash
cd ~/ROS2_Project
source install/setup.bash
ros2 launch my_robot_controller slam_mapping.launch.py
```

**Attendre que vous voyez :**
- `[slam_toolbox]: Message filter subscribing to topics...`
- Le robot apparaît dans Gazebo

---

#### 3. Lancer RViz pour visualiser (Terminal 2)
```bash
cd ~/ROS2_Project
source install/setup.bash
rviz2
```

**Configuration RViz :**
1. **Fixed Frame** : Changer en `map`
2. **Ajouter Map** :
   - Click "Add" (en bas à gauche)
   - Choisir "By topic" → `/map` → OK
   - Reliability: Best Effort
   
3. **Ajouter LaserScan** :
   - Click "Add"
   - Choisir "By topic" → `/scan` → OK
   - Size: 0.05
   - Color: Rouge
   
4. **Ajouter RobotModel** :
   - Click "Add"
   - Choisir "By display type" → RobotModel → OK

**Vous devez voir :**
- Le robot au centre
- Les scans laser en rouge
- La carte qui se construit en gris/noir/blanc

---

#### 4. Contrôler le robot (Terminal 3)
```bash
cd ~/ROS2_Project
source install/setup.bash
python3 src/my_robot_controller/keyboard_controller.py
```

---

### 🎯 TECHNIQUE DE MAPPING (TRÈS IMPORTANT!)

#### Règles d'or :

1. **VITESSE LENTE** 🐌
   - Appuyer sur **"1"** pour mode lent (0.2 m/s)
   - NE JAMAIS utiliser vitesse rapide pendant le mapping!

2. **MOUVEMENTS PROGRESSIFS**
   - Avancer 1-2 mètres → **S'ARRÊTER 2-3 secondes**
   - Tourner légèrement → **S'ARRÊTER 2-3 secondes**
   - Répéter

3. **PARCOURS RECOMMANDÉ**
   ```
   Départ (0,0)
   ↓
   ↓ Avancer doucement vers le Nord
   ↓ STOP 3 sec
   ↓
   → Tourner 90° droite (touche D)
   → STOP 3 sec
   → Avancer le long du mur Est
   → STOP tous les 2m
   ↓
   ↓ Tourner 90° droite
   ↓ Longer le mur Sud
   ↓ STOP régulièrement
   ←
   ← Tourner 90° droite
   ← Longer le mur Ouest
   ← STOP régulièrement
   ↑
   ↑ Revenir au point de départ
   ```

4. **BIEN SCANNER LES COINS**
   - À chaque coin, faire une **rotation sur place** (360°)
   - Tourner très lentement
   - S'arrêter tous les 90°

5. **VÉRIFIER DANS RViz**
   - La carte doit être **cohérente**
   - Pas de murs doubles
   - Les lignes doivent être nettes
   - Si vous voyez des superpositions → **STOP et recommencer**

---

#### 5. Sauvegarder la carte (Terminal 4)

**ATTENDRE d'avoir fait un tour complet !**

```bash
cd ~/ROS2_Project
source install/setup.bash
ros2 run nav2_map_server map_saver_cli -f ~/my_robot_map
```

**Vous devez voir :**
```
[INFO] [map_saver]: Receiving map from topic...
[INFO] [map_saver]: Map received
[INFO] [map_saver]: Writing map to ~/my_robot_map.yaml
[INFO] [map_saver]: Map saved
```

---

#### 6. Vérifier la carte sauvegardée

```bash
ls -lh ~/my_robot_map.*
```

**Vous devez avoir :**
- `my_robot_map.yaml` (fichier de configuration)
- `my_robot_map.pgm` (image de la carte)

**Visualiser la carte :**
```bash
eog ~/my_robot_map.pgm
# ou
xdg-open ~/my_robot_map.pgm
```

---

## ✅ CHECKLIST DE QUALITÉ

Votre carte est bonne si :

- [ ] Les murs sont des **lignes droites** continues
- [ ] **Pas de murs doublés** ou fantômes
- [ ] Les **coins sont nets** (angles de 90°)
- [ ] L'étagère est visible à sa position
- [ ] L'**espace libre** est bien blanc
- [ ] Les **obstacles** sont noirs
- [ ] Les **zones inconnues** sont grises
- [ ] Le fichier .pgm fait plus de 20 KB

---

## 🔧 SI LA CARTE EST MAUVAISE

### Recommencer proprement :

```bash
# 1. Tuer tous les processus
killall -9 gzserver gzclient rviz2

# 2. Nettoyer
./clean_and_map.sh

# 3. Recommencer depuis l'étape 2
```

### Conseils supplémentaires :

1. **Bouger ENCORE PLUS LENTEMENT**
2. Faire des pauses plus longues
3. Éviter les mouvements brusques
4. Bien observer RViz en temps réel
5. Si vous voyez une anomalie, arrêter immédiatement

---

## 📊 PARAMÈTRES SLAM OPTIMISÉS

Les paramètres ont été configurés pour :
- ✅ Meilleure précision du scan matching
- ✅ Loop closure plus strict (évite fausses associations)
- ✅ Mise à jour de carte plus fréquente
- ✅ Traitement de tous les scans (pas de skip)
- ✅ Distances de recherche réduites (environnement fermé)

---

## 🎓 ASTUCES D'EXPERT

1. **Faire le tour en 5-10 minutes** (pas de précipitation!)
2. **Regarder le terminal SLAM** pour les warnings
3. **Si un message d'erreur apparaît** → recommencer
4. **Sauvegarder plusieurs versions** de la carte
5. **Tester la carte** avec la navigation avant de l'utiliser en production

---

## 📞 COMMANDES UTILES

### Vérifier que SLAM fonctionne :
```bash
ros2 topic echo /map --once
ros2 topic hz /scan
ros2 node info /slam_toolbox
```

### Voir les transformations :
```bash
ros2 run tf2_tools view_frames
```

### Logs SLAM détaillés :
```bash
ros2 run slam_toolbox async_slam_toolbox_node --ros-args --log-level debug
```

---

## 🎯 RÉSUMÉ EN 3 POINTS

1. **NETTOYER** : `./clean_and_map.sh`
2. **MAPPER LENTEMENT** : Vitesse 1, pauses fréquentes, tour complet
3. **SAUVEGARDER** : `ros2 run nav2_map_server map_saver_cli -f ~/my_robot_map`

---

**Bonne chance pour votre cartographie ! 🗺️✨**
