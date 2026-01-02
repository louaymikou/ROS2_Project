# Guide d'Ajustement des Paramètres EKF

Ce guide vous aide à ajuster les paramètres de l'EKF selon vos besoins spécifiques.

## 📊 Problèmes Courants et Solutions

### 1. Le robot oscille ou ne suit pas bien le chemin

**Symptômes:**
- Le robot zigzague
- Trajectoires instables
- Vibrations dans le mouvement

**Solution:** Augmenter la souplesse du filtre

Éditez [config/ekf_params.yaml](config/ekf_params.yaml):

```yaml
# AVANT (strict)
process_noise_covariance: [0.05, 0, 0, ...

# APRÈS (plus souple)
process_noise_covariance: [0.1, 0, 0, ...
```

Multipliez toutes les valeurs diagonales par 2-3.

---

### 2. L'orientation dérive avec le temps

**Symptômes:**
- Le robot pense qu'il tourne alors qu'il va droit
- Erreur d'orientation qui s'accumule
- Le robot "perd le nord"

**Solution:** Augmenter le poids de l'IMU pour l'orientation

Éditez [config/ekf_params.yaml](config/ekf_params.yaml):

```yaml
# S'assurer que l'IMU fournit l'orientation
imu0_config: [false, false, false,     # position (non utilisé)
              true,  true,  true,      # ✅ orientation (roll, pitch, yaw)
              false, false, false,     # vitesse linéaire
              true,  true,  true,      # ✅ vitesse angulaire
              true,  true,  true]      # ✅ accélération

# Vérifier aussi:
imu0_remove_gravitational_acceleration: true
```

---

### 3. Le robot détecte mal les glissements

**Symptômes:**
- Erreurs quand le robot glisse
- Mauvaise estimation sur sols glissants
- Position incorrecte après glissement

**Solution:** Donner plus de poids aux accélérations de l'IMU

Éditez [config/ekf_params.yaml](config/ekf_params.yaml):

```yaml
# S'assurer que les accélérations sont activées
imu0_config: [false, false, false,
              true,  true,  true,
              false, false, false,
              true,  true,  true,
              true,  true,  true]      # ✅ Accélérations activées

# Réduire le bruit des accélérations dans process_noise_covariance
# Lignes 13-15 (accélérations x, y, z)
```

---

### 4. Le robot réagit trop lentement

**Symptômes:**
- Retard dans les mouvements
- Latence visible
- Réponse molle

**Solution:** Augmenter la fréquence de l'EKF

Éditez [config/ekf_params.yaml](config/ekf_params.yaml):

```yaml
# AVANT
frequency: 50.0

# APRÈS
frequency: 100.0
```

⚠️ **Note:** Cela augmente la charge CPU. Vérifiez que votre système peut suivre.

---

### 5. Le robot est trop réactif / nerveux

**Symptômes:**
- Mouvements saccadés
- Sur-correction
- Instabilité

**Solution:** Réduire la fréquence et augmenter le timeout

Éditez [config/ekf_params.yaml](config/ekf_params.yaml):

```yaml
frequency: 30.0  # Réduit de 50 à 30 Hz
sensor_timeout: 0.2  # Augmenté de 0.1 à 0.2 secondes
```

---

### 6. Les données IMU semblent bruitées

**Symptômes:**
- Orientation qui saute
- Valeurs erratiques
- Instabilité à l'arrêt

**Solution:** Augmenter le bruit dans la config IMU

Modifiez [description/imu.xacro](description/imu.xacro):

```xml
<!-- AVANT -->
<stddev>2e-4</stddev>  <!-- Angular velocity -->
<stddev>1.7e-2</stddev>  <!-- Linear acceleration -->

<!-- APRÈS (plus de filtrage) -->
<stddev>5e-4</stddev>  <!-- Angular velocity -->
<stddev>3e-2</stddev>  <!-- Linear acceleration -->
```

Puis rebuild:
```bash
colcon build --packages-select my_robot_controller
```

---

## 🎯 Configurations Prédéfinies

### Configuration Conservative (Stable mais moins précise)

Pour un robot qui préfère la stabilité à la précision:

```yaml
frequency: 30.0
sensor_timeout: 0.2

# Covariances plus élevées (plus souple)
process_noise_covariance: [0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                           0, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                           ...]

# Moins de poids sur l'IMU
imu0_config: [false, false, false,
              true,  true,  true,   # Orientation
              false, false, false,
              false, false, true,   # Seulement yaw_dot
              false, false, false]  # Pas d'accélérations
```

### Configuration Aggressive (Précise mais potentiellement instable)

Pour un robot qui a besoin de précision maximale:

```yaml
frequency: 100.0
sensor_timeout: 0.05

# Covariances plus basses (plus strict)
process_noise_covariance: [0.02, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                           0, 0.02, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                           ...]

# Poids maximal sur l'IMU
imu0_config: [false, false, false,
              true,  true,  true,   # Orientation
              false, false, false,
              true,  true,  true,   # Toutes les vitesses angulaires
              true,  true,  true]   # Toutes les accélérations
```

### Configuration Équilibrée (Recommandée pour débuter)

Configuration par défaut - bon équilibre:

```yaml
frequency: 50.0
sensor_timeout: 0.1

# Voir config/ekf_params.yaml pour les valeurs complètes
```

---

## 🧪 Méthodologie de Test

### Étape 1: Test Statique

Robot à l'arrêt, vérifier que l'odométrie ne dérive pas:

```bash
# Lancer la simulation
ros2 launch my_robot_controller launch_sim.launch.py

# Dans un autre terminal, surveiller l'odométrie
ros2 topic echo /odometry/local | grep -A 3 "position:"

# Le robot ne doit pas bouger virtuellement
```

✅ **Bon résultat:** Position stable (variations < 1cm)
❌ **Mauvais résultat:** Position dérive continuellement

---

### Étape 2: Test de Rotation Pure

Faire tourner le robot sur lui-même:

```bash
# Envoyer une commande de rotation pure
ros2 topic pub /diff_cont/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}"

# Observer l'orientation
ros2 topic echo /odometry/local | grep -A 4 "orientation:"
```

✅ **Bon résultat:** Orientation change de manière fluide et précise
❌ **Mauvais résultat:** Sauts, instabilité, ou dérive

---

### Étape 3: Test de Translation Pure

Faire avancer le robot en ligne droite:

```bash
# Commande de mouvement avant
ros2 topic pub /diff_cont/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}"

# Observer la position ET l'orientation
ros2 topic echo /odometry/local
```

✅ **Bon résultat:** Le robot avance droit, orientation stable
❌ **Mauvais résultat:** Le robot dérive latéralement ou l'orientation change

---

### Étape 4: Test de Mouvement Combiné

Trajectoire en courbe:

```bash
ros2 topic pub /diff_cont/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.3}}"
```

✅ **Bon résultat:** Trajectoire fluide et cohérente
❌ **Mauvais résultat:** Saccades, sur-corrections

---

### Étape 5: Test de Navigation Complète

Utiliser Nav2 pour naviguer vers un point:

```bash
# Lancer la navigation (avec carte pré-existante)
ros2 launch my_robot_controller launch_navigation.launch.py

# Envoyer un goal dans RViz2
```

✅ **Bon résultat:** Le robot suit le chemin de manière fluide
❌ **Mauvais résultat:** Oscillations, échecs de suivi de trajectoire

---

## 📈 Métriques à Surveiller

### 1. Covariance de l'odométrie

```bash
ros2 topic echo /odometry/local --field pose.covariance
```

- Valeurs trop **élevées** → Le filtre n'est pas confiant (problème)
- Valeurs trop **basses** → Le filtre est trop confiant (peut masquer des erreurs)
- Valeurs **stables** → Bon signe

### 2. Fréquence de publication

```bash
ros2 topic hz /odometry/local
```

- Devrait être proche de la `frequency` configurée (50 Hz par défaut)
- Si beaucoup plus bas → Problème de performance CPU

### 3. Latence

```bash
ros2 topic delay /odometry/local
```

- Devrait être < 50ms
- Si > 100ms → Problème de performance

### 4. Comparaison IMU vs Wheel Odom

```bash
# Terminal 1: Wheel odometry
ros2 topic echo /diff_cont/odom | grep -A 4 "orientation:"

# Terminal 2: Fused odometry
ros2 topic echo /odometry/local | grep -A 4 "orientation:"
```

Les orientations doivent être similaires mais l'odométrie fusionnée doit être plus stable.

---

## 🔧 Commandes de Diagnostic

### Vérifier les paramètres actuels

```bash
ros2 param get /ekf_filter_node frequency
ros2 param get /ekf_filter_node odom0_config
ros2 param get /ekf_filter_node imu0_config
```

### Recharger les paramètres sans redémarrer

```bash
ros2 param set /ekf_filter_node frequency 60.0
```

⚠️ **Note:** Certains paramètres nécessitent un redémarrage du node.

### Enregistrer des données pour analyse offline

```bash
# Enregistrer 30 secondes de données
ros2 bag record -d 30 /imu/data /diff_cont/odom /odometry/local /cmd_vel

# Rejouer les données
ros2 bag play <nom_du_bag>
```

---

## 📚 Ressources Supplémentaires

- [Documentation robot_localization](http://docs.ros.org/en/humble/p/robot_localization/)
- [Tuning Guide officiel](http://docs.ros.org/en/melodic/api/robot_localization/html/configuring_robot_localization.html)
- [IMU Architecture](./IMU_ARCHITECTURE.md)
- [Guide d'intégration IMU](./IMU_INTEGRATION_GUIDE.md)

---

## 💡 Conseils Généraux

1. **Commencez conservateur:** Utilisez la config par défaut, puis ajustez progressivement
2. **Testez isolément:** Changez un paramètre à la fois
3. **Documentez vos changements:** Notez ce qui marche et ce qui ne marche pas
4. **Utilisez des métriques objectives:** Ne vous fiez pas qu'à l'observation visuelle
5. **Soyez patient:** Le tuning peut prendre du temps

---

## ⚠️ Avertissements

- Des covariances **trop basses** peuvent rendre le filtre trop rigide et insensible aux changements réels
- Des covariances **trop élevées** peuvent rendre le filtre trop mou et lent à réagir
- Une fréquence **trop élevée** peut surcharger le CPU
- Désactiver l'IMU complètement retire les bénéfices de la fusion de capteurs

---

Besoin d'aide ? Consultez le [IMU Integration Guide](./IMU_INTEGRATION_GUIDE.md) ou lancez le script de test :

```bash
./test_imu_integration.sh
```
