# 📦 Guide d'utilisation des cubes de test

## 🎯 Cubes ajoutés à la simulation

J'ai ajouté **3 cubes rouges** dans la simulation Gazebo pour tester le mécanisme de poussée :

### Positions des cubes :

1. **test_cube_1** : Position (0, 0, 0.45) - **Au-dessus du robot au démarrage**
2. **test_cube_2** : Position (5, 0, 0.3) - Sur la ligne bleue, 5m en avant
3. **test_cube_3** : Position (10, 0, 0.3) - Sur la ligne bleue, 10m en avant

## 🎮 Comment déplacer les cubes

### Méthode 1 : Avec la souris dans Gazebo

1. **Lancer la simulation** :
   ```bash
   ros2 launch blue_line_follower simulation.launch.py
   ```

2. **Dans la fenêtre Gazebo** :
   - Cliquez sur l'icône de **translation** (flèches croisées) dans la barre d'outils
   - Cliquez sur un cube pour le sélectionner
   - Déplacez-le avec les flèches colorées (X=rouge, Y=vert, Z=bleu)
   - Ou utilisez l'icône de **rotation** pour le faire pivoter

### Méthode 2 : Avec des commandes ROS2

Pour déplacer un cube par commande :

```bash
# Exemple: déplacer test_cube_1 à une nouvelle position
ros2 service call /gazebo/set_entity_state gazebo_msgs/srv/SetEntityState \
  "{state: {name: 'test_cube_1', pose: {position: {x: 0.0, y: 0.0, z: 0.45}}}}"
```

## 🧪 Scénarios de test

### Test 1 : Cube au-dessus du robot
Le `test_cube_1` est déjà positionné au-dessus du robot :

```bash
# Terminal 1: Lancer la simulation
ros2 launch blue_line_follower simulation.launch.py

# Terminal 2: Contrôler le pousseur
ros2 run blue_line_follower pusher_controller
# Choisir: 4 (Séquence complète AVANT)
```

**Résultat attendu** : Le prisme pousse le cube vers l'avant du robot

### Test 2 : Positionner manuellement un cube

1. Dans Gazebo, utilisez la souris pour déplacer `test_cube_2` ou `test_cube_3`
2. Positionnez-le au-dessus du robot (environ z=0.45)
3. Lancez le contrôleur de poussée
4. Testez différentes positions de poussée

### Test 3 : Navigation avec cube

```bash
# Terminal 1: Simulation déjà lancée

# Terminal 2: Naviguer vers un marqueur
ros2 run blue_line_follower aruco_navigation_client
# Choisir un marqueur (ex: 3)

# Terminal 3: Une fois arrivé, pousser le cube
ros2 run blue_line_follower pusher_controller
# Choisir: 4 (Séquence complète AVANT)
```

## 📏 Caractéristiques des cubes

| Propriété | Valeur |
|-----------|--------|
| **Dimensions** | 20cm × 20cm × 20cm |
| **Masse** | 0.5 kg |
| **Couleur** | Rouge |
| **Type** | Dynamique (bouge avec la physique) |
| **Friction** | 0.8 (glisse modérément) |

## 🎨 Personnalisation des cubes

Pour modifier les cubes, éditez :
```
src/blue_line_follower/models/test_cube/model.sdf
```

### Changer la taille :
```xml
<size>0.2 0.2 0.2</size>  <!-- Augmentez pour un cube plus grand -->
```

### Changer la couleur :
```xml
<ambient>0.8 0.2 0.2 1</ambient>  <!-- Rouge actuel -->
<ambient>0.2 0.8 0.2 1</ambient>  <!-- Vert -->
<ambient>0.2 0.2 0.8 1</ambient>  <!-- Bleu -->
```

### Changer la masse :
```xml
<mass>0.5</mass>  <!-- En kg -->
```

## 🔄 Réinitialiser un cube

Si un cube tombe ou sort de la zone :

```bash
# Remettre test_cube_1 au-dessus du robot
ros2 service call /gazebo/set_entity_state gazebo_msgs/srv/SetEntityState \
  "{state: {name: 'test_cube_1', pose: {position: {x: 0.0, y: 0.0, z: 0.45}, \
  orientation: {x: 0, y: 0, z: 0, w: 1}}}}"
```

## 🚀 Workflow complet de test

### Cycle complet avec cube :

```bash
# 1. Lancer la simulation
ros2 launch blue_line_follower simulation.launch.py

# 2. Le cube test_cube_1 est déjà au-dessus du robot

# 3. (Optionnel) Naviguer vers un point
ros2 run blue_line_follower aruco_navigation_client
# Exemple: Aller au marqueur 5

# 4. Pousser le cube vers l'avant
ros2 run blue_line_follower pusher_controller
# Choisir: 1 (Pousser vers l'avant)

# 5. Attendre que le cube glisse/tombe

# 6. Retourner le prisme à l'arrière
# Choisir: 3 (Retour à l'arrière)

# 7. Répéter avec un autre cube
```

## 📊 Visualisation dans RViz

Pour voir le cube dans RViz :
1. RViz se lance automatiquement avec la simulation
2. Le cube apparaît comme un objet dans l'environnement
3. Vous pouvez observer la collision avec le prisme pousseur

## ⚠️ Conseils

- **Position initiale** : Le cube au-dessus du robot (z=0.45) est à la bonne hauteur pour le prisme
- **Physique** : Les cubes obéissent à la gravité et aux collisions
- **Stabilité** : Si le cube tremble, augmentez la friction dans le SDF
- **Performance** : 3 cubes sont suffisants, trop d'objets ralentissent la simulation

## 🎯 Points de test recommandés

| Test | Description | Résultat attendu |
|------|-------------|------------------|
| **Poussée simple** | Cube au-dessus, poussée à 0.6m | Cube glisse vers l'avant |
| **Poussée partielle** | Cube au-dessus, poussée à 0.3m | Cube reste sur le robot |
| **Poussée + navigation** | Naviguer puis pousser | Cube livré à destination |
| **Retour arrière** | Après poussée, retour à 0.0m | Prisme revient seul |
| **Cube mal positionné** | Cube à côté du prisme | Prisme passe à côté |

## 🐛 Dépannage

### Le cube traverse le prisme
- Vérifiez les collisions dans le URDF du prisme
- Augmentez `kp` dans les propriétés de contact

### Le cube tombe immédiatement
- La position z est trop basse
- Rehaussez-le : `z: 0.5` au lieu de `z: 0.45`

### Le cube ne bouge pas
- Vérifiez qu'il n'est pas en mode `static`
- Dans `model.sdf` : `<static>false</static>`
