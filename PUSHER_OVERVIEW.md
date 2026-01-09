# 🎯 Mécanisme de Poussée - Vue d'ensemble

## 📐 Configuration du système

```
Vue de profil (de côté) :
                            
                            ┌─────────────┐
                            │  PRISME     │  ← Peut glisser sur le rail
                            │  POUSSEUR   │
                            └──────┬──────┘
    ═══════════════════════════════════════  ← Rail horizontal
                            
    ┌─────────────────────────────────────┐
    │                                     │
    │         ROBOT (base_link)           │
    │                                     │
    └─────────────────────────────────────┘
         ↑                         ↑
       ARRIÈRE                   AVANT
```

## 🔄 Mouvement du prisme

```
Position INITIALE (0.0 m) - Arrière :
    
    [PRISME]
    ═══════════════════════════════
    └─────────────────────┘
         ROBOT


Position MILIEU (0.3 m) :
    
              [PRISME]
    ═══════════════════════════════
    └─────────────────────┘
         ROBOT


Position AVANT (0.6 m) :
    
                        [PRISME]
    ═══════════════════════════════
    └─────────────────────┘
         ROBOT
```

## 🎮 Utilisation typique

### 1. Chargement du cube
```
[CUBE] placé sous le prisme à l'arrière
    ↓
[PRISME] en position 0.0 (arrière)
    ↓
Robot transporte le cube vers la destination
```

### 2. Déchargement du cube
```
Robot arrive à destination
    ↓
[PRISME] pousse vers l'avant (0.6m)
    ↓
[CUBE] tombe/glisse en avant du robot
    ↓
[PRISME] retourne à l'arrière (0.0m)
```

## ⚙️ Caractéristiques techniques

| Paramètre | Valeur |
|-----------|--------|
| **Type de joint** | Prismatique (glissant) |
| **Axe de mouvement** | X (longitudinal) |
| **Position minimale** | 0.0 m (arrière) |
| **Position maximale** | 0.6 m (avant) |
| **Vitesse max** | 0.5 m/s |
| **Effort max** | 50 N |
| **Masse du prisme** | 0.2 kg |
| **Dimensions** | 15cm × 30cm × 12cm |

## 🎯 Cas d'usage

### Usage 1 : Transport simple
1. Placer cube à l'arrière
2. Naviguer vers destination
3. Pousser complètement (0.6m)
4. Cube tombe devant le robot

### Usage 2 : Poussée partielle
1. Placer cube à l'arrière
2. Pousser à mi-chemin (0.3m)
3. Cube reste sur le robot mais avancé
4. Utile pour repositionnement

### Usage 3 : Poussée progressive
1. Plusieurs petites poussées
2. Contrôle fin de la position du cube
3. Évacuation progressive

## 🔧 Commandes rapides

```bash
# Lancer la simulation
ros2 launch blue_line_follower simulation.launch.py

# Contrôler le pousseur (nouveau terminal)
ros2 run blue_line_follower pusher_controller

# Tester en temps réel (nouveau terminal)
ros2 run blue_line_follower pusher_tester

# Vérifier la position actuelle
ros2 topic echo /joint_states | grep -A 10 "pusher_joint"
```

## 📊 Topics ROS2

```bash
# Action pour contrôler le pousseur
/pusher_controller/follow_joint_trajectory

# État des joints (position actuelle)
/joint_states

# Contrôleurs actifs
ros2 control list_controllers
```

## ✅ Vérifications avant utilisation

- [ ] La simulation Gazebo est lancée
- [ ] Le contrôleur `pusher_controller` est actif
- [ ] Le prisme est visible au-dessus du robot
- [ ] La position initiale est à l'arrière (0.0m)
- [ ] Le cube est correctement positionné

## 🚀 Commande directe simple

Pour une poussée rapide sans menu :

```bash
# Pousser complètement vers l'avant
ros2 action send_goal /pusher_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: ['pusher_joint'], 
  points: [{positions: [0.6], velocities: [0.0], 
  time_from_start: {sec: 3}}]}}" --feedback

# Retour à l'arrière
ros2 action send_goal /pusher_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: ['pusher_joint'], 
  points: [{positions: [0.0], velocities: [0.0], 
  time_from_start: {sec: 3}}]}}" --feedback
```

## 🎨 Couleurs des composants

- 🔵 **Robot** : Bleu
- ⚪ **Rail** : Blanc/Argenté
- 🟠 **Prisme pousseur** : Orange

Ces couleurs facilitent l'identification dans Gazebo et RViz.
