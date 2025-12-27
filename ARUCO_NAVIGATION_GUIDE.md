# Navigation ArUco - Guide d'utilisation

## 📖 Description

Ce système permet au robot de naviguer automatiquement vers un marqueur ArUco spécifique en suivant une ligne bleue.

### Fonctionnalités

- **Navigation intelligente** : Le robot détermine automatiquement s'il doit aller en avant ou en arrière selon la position du marqueur cible
- **Détection stable** : Requiert plusieurs détections consécutives pour éviter les faux positifs
- **Feedback en temps réel** : Affiche la progression de la navigation
- **Annulation possible** : Permet d'arrêter la navigation à tout moment

### Logique de direction

- **Marche avant** : Si le numéro du marqueur cible > numéro du marqueur actuel
- **Marche arrière** : Si le numéro du marqueur cible < numéro du marqueur actuel
- **Déjà arrivé** : Si le numéro du marqueur cible = numéro du marqueur actuel

## 🚀 Démarrage rapide

### Étape 1 : Lancer la simulation

```bash
cd ~/line_follower_ws
source install/setup.bash
ros2 launch blue_line_follower simulation.launch.py
```

### Étape 2 : Lancer le nœud de suivi de ligne

Dans un nouveau terminal :

```bash
source ~/line_follower_ws/install/setup.bash
ros2 run blue_line_follower line_follower_node
```

### Étape 3 : Lancer le serveur d'action ArUco

Dans un nouveau terminal :

```bash
source ~/line_follower_ws/install/setup.bash
ros2 run blue_line_follower aruco_navigation_server
```

### Étape 4 : Utiliser le client pour naviguer

#### Option A : Utiliser le client Python

```bash
source ~/line_follower_ws/install/setup.bash
# Exemple : naviguer vers ArUco numéro 5
ros2 run blue_line_follower aruco_navigation_client 5
```

#### Option B : Utiliser la ligne de commande

```bash
source ~/line_follower_ws/install/setup.bash
ros2 action send_goal /navigate_to_aruco custom_interfaces/action/NavigateToAruco "{target_aruco_id: 5}" --feedback
```

## 🎯 Exemples d'utilisation

### Naviguer vers ArUco 10

```bash
ros2 run blue_line_follower aruco_navigation_client 10
```

**Sortie attendue :**
```
[INFO] [aruco_navigation_client]: 🎯 Demande de navigation vers ArUco 10...
[INFO] [aruco_navigation_client]: ⏳ Attente du serveur d'action...
[INFO] [aruco_navigation_client]: ✅ Demande acceptée!
[INFO] [aruco_navigation_client]: 🚀 Navigation en cours...

[INFO] [aruco_navigation_client]: ➡️ ArUco: 3 | ⏱️  2.1s | ArUco 3 → 10 (distance: 7)
[INFO] [aruco_navigation_client]: ➡️ ArUco: 5 | ⏱️  4.3s | ArUco 5 → 10 (distance: 5)
[INFO] [aruco_navigation_client]: ➡️ ArUco: 8 | ⏱️  7.2s | ArUco 8 → 10 (distance: 2)
[INFO] [aruco_navigation_client]: ➡️ ArUco: 10 | ⏱️  9.5s | ArUco 10 → 10 (distance: 0)

=================================
[INFO] [aruco_navigation_client]: 🎉 NAVIGATION RÉUSSIE!
[INFO] [aruco_navigation_client]:    ArUco final: 10
[INFO] [aruco_navigation_client]:    Direction: AVANT
[INFO] [aruco_navigation_client]:    Temps: 9.5s
=================================
```

### Naviguer en arrière vers ArUco 2

Si le robot est actuellement à ArUco 8 :

```bash
ros2 run blue_line_follower aruco_navigation_client 2
```

**Sortie attendue :**
```
[INFO] [aruco_navigation_server]: ⬅️ ArUco 2 est avant, marche arrière
[INFO] [aruco_navigation_client]: ⬅️ ArUco: 7 | ⏱️  1.8s | ArUco 7 → 2 (distance: 5)
[INFO] [aruco_navigation_client]: ⬅️ ArUco: 5 | ⏱️  3.5s | ArUco 5 → 2 (distance: 3)
[INFO] [aruco_navigation_client]: ⬅️ ArUco: 2 | ⏱️  5.2s | ArUco 2 → 2 (distance: 0)

=================================
[INFO] [aruco_navigation_client]: 🎉 NAVIGATION RÉUSSIE!
[INFO] [aruco_navigation_client]:    ArUco final: 2
[INFO] [aruco_navigation_client]:    Direction: ARRIÈRE
[INFO] [aruco_navigation_client]:    Temps: 5.2s
=================================
```

## 🔍 Vérification du système

### Vérifier que l'action est disponible

```bash
ros2 action list
```

Vous devriez voir :
```
/navigate_to_aruco
```

### Voir les détails de l'action

```bash
ros2 action info /navigate_to_aruco
```

### Voir l'interface de l'action

```bash
ros2 interface show custom_interfaces/action/NavigateToAruco
```

## 🛑 Annuler une navigation

Pour annuler une navigation en cours, appuyez sur `Ctrl+C` dans le terminal du client.

Le robot s'arrêtera immédiatement et retournera l'état actuel.

## ⚙️ Services disponibles

Le système utilise les services suivants du nœud line_follower :

### Activer/Désactiver le mouvement

```bash
# Activer
ros2 service call /enable_movement std_srvs/srv/SetBool "{data: true}"

# Désactiver
ros2 service call /enable_movement std_srvs/srv/SetBool "{data: false}"
```

### Changer la direction manuellement

```bash
# Marche avant
ros2 service call /set_forward_direction std_srvs/srv/SetBool "{data: true}"

# Marche arrière
ros2 service call /set_forward_direction std_srvs/srv/SetBool "{data: false}"
```

## 📊 Structure de l'action

### Goal (Objectif)
```
int32 target_aruco_id    # Numéro du marqueur ArUco cible (0-49)
```

### Feedback (Retour d'information)
```
int32 current_aruco_id   # ArUco actuellement détecté
string current_direction # Direction actuelle (AVANT/ARRIÈRE)
float32 elapsed_time     # Temps écoulé
string status_message    # Message de statut
```

### Result (Résultat)
```
bool success             # Navigation réussie ?
int32 final_aruco_id     # ArUco final atteint
bool went_forward        # Direction utilisée
float32 navigation_time  # Temps total
float32 distance_traveled # Distance parcourue
```

## 🐛 Dépannage

### Erreur : "Serveur d'action non disponible"

**Vérifier que le serveur est lancé :**
```bash
ros2 node list | grep aruco
```

**Solution :** Lancer le serveur :
```bash
ros2 run blue_line_follower aruco_navigation_server
```

### Erreur : "Numéro ArUco invalide"

Les numéros ArUco doivent être entre 0 et 49 (DICT_4X4_50).

### Erreur : "Aucun ArUco détecté au départ"

Le robot doit pouvoir détecter un marqueur ArUco avant de commencer la navigation.

**Solutions :**
1. Assurez-vous que des marqueurs ArUco sont visibles dans la simulation
2. Vérifiez que les caméras fonctionnent correctement
3. Attendez quelques secondes que le robot détecte un marqueur

### Le robot ne bouge pas

**Vérifier que le mouvement est activé :**
```bash
ros2 service call /enable_movement std_srvs/srv/SetBool "{data: true}"
```

## 📝 Architecture du système

```
┌─────────────────────────────────────┐
│  aruco_navigation_client            │
│  (Envoie l'objectif)                │
└──────────────┬──────────────────────┘
               │
               ▼
┌─────────────────────────────────────┐
│  aruco_navigation_server            │
│  (Gère la navigation)               │
│  - Détecte position actuelle        │
│  - Détermine direction              │
│  - Envoie feedback                  │
│  - Contrôle via services            │
└──────────────┬──────────────────────┘
               │
               ▼
┌─────────────────────────────────────┐
│  line_follower_node                 │
│  (Suit la ligne)                    │
│  - Détecte ligne bleue              │
│  - Détecte marqueurs ArUco          │
│  - Contrôle moteurs                 │
└─────────────────────────────────────┘
```

## 📚 En savoir plus

Pour comprendre le fonctionnement des actions ROS2, consultez le guide fourni :
- [ROS2 Guided Lab 06 - Actions & Long-Running Tasks](/home/wayay/Downloads/ROS2 Guided Lab 06 - Actions & Long-Running T.md)

## 🎓 Exercices pratiques

1. **Navigation séquentielle** : Créer un script qui navigue vers plusieurs marqueurs dans l'ordre
2. **Navigation conditionnelle** : Naviguer vers un marqueur seulement s'il est à moins de 5 positions
3. **Calcul de distance** : Implémenter le calcul de distance réelle parcourue
4. **Optimisation** : Améliorer la vitesse de détection des marqueurs ArUco

Bon développement ! 🤖
