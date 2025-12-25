#!/usr/bin/env python3
"""
Visualiser la carte PGM et créer une version HTML
"""
import numpy as np
from PIL import Image
import os

# Charger l'image PGM
map_path = os.path.expanduser('~/my_robot_map.pgm')
img = Image.open(map_path)
img_array = np.array(img)

print(f"📊 Informations sur la carte :")
print(f"   Taille: {img_array.shape[1]} x {img_array.shape[0]} pixels")
print(f"   Valeurs min/max: {img_array.min()} / {img_array.max()}")
print(f"   Taille fichier: {os.path.getsize(map_path) / 1024:.1f} KB")

# Sauvegarder en PNG
png_path = os.path.expanduser('~/ROS2_Project/my_robot_map.png')
img.save(png_path, 'PNG')
print(f"\n✅ Carte sauvegardée en PNG: {png_path}")

# Créer une page HTML pour visualiser
html_path = os.path.expanduser('~/ROS2_Project/view_map.html')
html_content = f"""<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>Carte du Robot - Visualisation</title>
    <style>
        body {{
            font-family: Arial, sans-serif;
            margin: 20px;
            background: #1e1e1e;
            color: #fff;
        }}
        .container {{
            max-width: 1200px;
            margin: 0 auto;
            background: #2d2d2d;
            padding: 20px;
            border-radius: 10px;
        }}
        h1 {{
            color: #4CAF50;
            text-align: center;
        }}
        .info {{
            background: #383838;
            padding: 15px;
            border-radius: 5px;
            margin: 20px 0;
        }}
        .info h3 {{
            color: #4CAF50;
            margin-top: 0;
        }}
        .map-container {{
            text-align: center;
            background: #000;
            padding: 20px;
            border-radius: 5px;
        }}
        .map-image {{
            max-width: 100%;
            border: 3px solid #4CAF50;
            border-radius: 5px;
            image-rendering: pixelated;
        }}
        .legend {{
            display: flex;
            justify-content: center;
            gap: 30px;
            margin: 20px 0;
            flex-wrap: wrap;
        }}
        .legend-item {{
            display: flex;
            align-items: center;
            gap: 10px;
        }}
        .color-box {{
            width: 40px;
            height: 40px;
            border: 2px solid #fff;
        }}
    </style>
</head>
<body>
    <div class="container">
        <h1>🗺️ Carte SLAM du Robot Mobile Manipulateur</h1>
        
        <div class="info">
            <h3>📊 Informations de la Carte</h3>
            <p><strong>Taille:</strong> {img_array.shape[1]} × {img_array.shape[0]} pixels</p>
            <p><strong>Résolution:</strong> 0.05 m/pixel (5 cm)</p>
            <p><strong>Origine:</strong> (-10.7, -16, 0)</p>
            <p><strong>Dimension réelle:</strong> ~{img_array.shape[1]*0.05:.1f}m × {img_array.shape[0]*0.05:.1f}m</p>
        </div>

        <div class="legend">
            <div class="legend-item">
                <div class="color-box" style="background: white;"></div>
                <span><strong>Blanc</strong> = Espace libre (navigable)</span>
            </div>
            <div class="legend-item">
                <div class="color-box" style="background: black;"></div>
                <span><strong>Noir</strong> = Obstacles (murs, étagère)</span>
            </div>
            <div class="legend-item">
                <div class="color-box" style="background: gray;"></div>
                <span><strong>Gris</strong> = Zone inconnue</span>
            </div>
        </div>

        <div class="map-container">
            <img src="my_robot_map.png" alt="Carte du robot" class="map-image">
        </div>

        <div class="info">
            <h3>🎯 Éléments de l'Environnement</h3>
            <ul>
                <li>🔵 <strong>Murs bleus</strong> - Périmètre de la zone (16m × 16m)</li>
                <li>📚 <strong>Étagère de stockage</strong> - 3 niveaux × 3 colonnes (9 casiers)</li>
                <li>📦 <strong>Petits packages</strong> - Bleus (0.3m cubes)</li>
                <li>📦 <strong>Grands packages</strong> - Orange (0.5×0.4×0.6m)</li>
            </ul>
        </div>

        <div class="info">
            <h3>✅ Validation de la Carte</h3>
            <p><strong>Qualité:</strong> {'✅ Bonne' if os.path.getsize(map_path) > 100000 else '⚠️ À vérifier'}</p>
            <p><strong>Taille fichier:</strong> {os.path.getsize(map_path) / 1024:.1f} KB</p>
            <p><strong>Fichiers:</strong></p>
            <ul>
                <li>my_robot_map.pgm (carte originale)</li>
                <li>my_robot_map.yaml (configuration)</li>
                <li>my_robot_map.png (visualisation)</li>
            </ul>
        </div>
    </div>
</body>
</html>
"""

with open(html_path, 'w', encoding='utf-8') as f:
    f.write(html_content)

print(f"✅ Page HTML créée: {html_path}")
print(f"\n🌐 Pour voir la carte, ouvrez:")
print(f"   firefox {html_path}")
print(f"\nOu double-cliquez sur le fichier dans VS Code")
