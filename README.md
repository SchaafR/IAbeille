# IABEILLES — Système de surveillance de ruches intelligent

[![University](https://img.shields.io/badge/Sorbonne-Université-blue.svg)](https://www.sorbonne-universite.fr/)
[![Department](https://img.shields.io/badge/Polytech-Electronique%20%26%20Informatique-orange.svg)](https://www.polytech.sorbonne-universite.fr/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

**IABEILLES** est une solution IoT (Internet des Objets) développée à Polytech Sorbonne pour protéger les colonies d'abeilles contre la prédation des frelons asiatiques. Le système utilise l'intelligence artificielle pour la détection visuelle et sonore, tout en transmettant des données environnementales via le réseau LoRaWAN.

---

## Table des Matières
* [Présentation](#-présentation)
* [Spécifications Matérielles](#-spécifications-matérielles)
* [Installation Logicielle](#-installation-logicielle)
* [Configuration Réseau (LoRaWAN/TTN)](#-configuration-réseau)
* [Mise en Service](#-mise-en-service)
* [Équipe](#-équipe)

---

## Présentation

### Mission
Transformer des mesures physiques en **informations exploitables** pour les apiculteurs et chercheurs. Le système surveille en temps réel :
* **Détection visuelle** (IA) : Identification des abeilles et frelons.
* **Détection sonore** : Analyse de l'activité acoustique.
* **Environnement** : Température, humidité et luminosité.

### Architecture de communication
`Capteurs (I2C/Analog)` ➔ `ESP32S3 / Nano BLE` ➔ `Module LoRa E5` ➔ `Passerelle LoRaWAN` ➔ `TTN` ➔ `Bleep (Dashboard)`

---

## Spécifications Matérielles

| Composant | Rôle Principal |
| :--- | :--- |
| **XIAO ESP32S3 Sense** | Cœur de l'IA (Vision) & Caméra |
| **Arduino Nano BLE Sense** | Analyse sonore & Gestion des capteurs |
| **Module LoRa E5** | Communication longue portée |
| **DHT22 & BH1750** | Monitoring Température/Humidité/Lumière |
| **Panneau Solaire & LiPo** | Autonomie énergétique (3.7V 1000mAh) |
| **PCB Custom** | Interconnexion et robustesse |

---

## Installation Logicielle

### 1. Prérequis
* [Arduino IDE](https://www.arduino.cc/en/software)
* Support de carte **ESP32 by Espressif Systems** (via le gestionnaire de cartes).
* Support de carte **Arduino Mbed OS Nano Boards**.

### 2. Récupération du projet
```bash
git clone https://github.com/SchaafR/IAbeille.git
```

### 3. Installation des Librairies Custom (IA)
Les modèles d'inférence sont fournis sous forme d'archives `.zip` dans le dossier `/data`.
1. Localisez votre dossier Arduino (ex: `Documents/Arduino/libraries`).
2. Décompressez-y les fichiers suivants :
   - `BeeGuardAI_Hornet_Bee_Bees_G1V2_inferencing.zip`
   - `Hornets_lib_3.zip`
3. Redémarrez l'IDE Arduino.

### 4. Dépendances Standard
Installez via le **Library Manager** d'Arduino :
* `DHT sensor library`
* `BH1750`

---

## Configuration Réseau

### The Things Network (TTN)
Le module LoRa E5 utilise l'activation **OTAA**.
1. **Créer une application** sur [TTN Console](https://console.cloud.thethings.network/).
2. **Enregistrer l'appareil** avec les identifiants présents dans le code source :
   - `DevEUI`
   - `AppEUI` / `JoinEUI`
   - `AppKey`
3. **Payload Formatter** : Utilisez le script JavaScript situé dans `/src/payload_formatter` pour décoder les trames hexadécimales en données lisibles.

---

## Mise en Service

### Flashage des cartes
1. Ouvrez `COM_N_AI.ino` dans Arduino IDE.
2. Connectez le **XIAO ESP32S3**.
3. Sélectionnez le port correspondant et le type de carte **XIAO_ESP32S3**.
4. Cliquez sur **Upload** (→).
5. Répétez l'opération pour l'Arduino Nano avec le fichier `hornet_lib_3.ino`.

### Déploiement physique
> [!IMPORTANT]
> Vérifiez l'étanchéité du boîtier et le serrage des presse-étoupes avant l'installation en extérieur.

1. Actionnez le bouton **On/Off**.
2. Un **bip sonore (0.5s)** confirme l'initialisation du système.
3. Vérifiez la LED de charge (Panneau Solaire/Batterie).
4. Surveillez l'onglet **Live Data** sur TTN pour confirmer la réception des premières trames.

---

## Équipe (Rédacteurs)

* **Léo Dupuy**
* **Amine Filahi**
* **Benoit Lavieville**
* **Marguerite Massinga**
* **Rémy Schaaf**

---

## Remerciements
Projet réalisé dans le cadre de la 4ème année à **Polytech Sorbonne**. Merci à l'équipe pédagogique et aux encadrants pour leur soutien technique et méthodologique.

## Contacte
En cas de problème ou de questions,vous pouvez nous contacter à l'adresse : schaarem@gmail.com
