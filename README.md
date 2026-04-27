IABEILLES

Sorbonne Université
4 place Jussieu, 75005 Paris, France

Guide utilisateur

Rédacteurs

* Léo Dupuy
* Amine Filahi
* Benoit Lavieville
* Marguerite Massinga
* Rémy Schaaf

Table des matières

1. Objectif du document￼
2. Présentation du projet￼
    * 2.1 Mission et objectif￼
3. Détail du matériel￼
4. Montage matériel￼
5. Logiciels requis￼
6. Installation logicielle￼
    * 6.1 Télécharger le code source￼
    * 6.2 Installer Arduino IDE￼
    * 6.3 Installer une librairie Arduino depuis un dossier local￼
    * 6.4 Installer les librairies Arduino nécessaires￼
    * 6.5 Configurer TTN pour recevoir les données du module LoRaWAN E5￼
    * 6.6 Configurer Bleep pour visualiser les données￼
7. Implémentation logicielle￼
    * 7.1 Téléverser le code sur la carte avec Arduino IDE￼
8. Intégration et mise en service￼
9. Remerciements￼

1. Objectif du document

Ce document a pour objectif de fournir un guide utilisateur complet permettant de mettre en œuvre la solution développée dans le cadre du projet IABEILLES.

Ce projet a été réalisé au cours du second semestre de la quatrième année de la formation Électronique et Informatique de Polytech Sorbonne.

Le guide présente les différentes étapes nécessaires pour installer, configurer et utiliser le système, depuis la récupération du code source jusqu’à la mise en service du dispositif sur une ruche.

2. Présentation du projet

Le projet IABEILLES a été développé pour répondre à la menace croissante représentée par les frelons asiatiques autour des ruches.

Il s’agit d’un système connecté conçu pour surveiller automatiquement une ruche. Le dispositif collecte des informations à l’aide de différents capteurs, puis transmet ces données afin qu’elles puissent être consultées et exploitées par l’utilisateur.

L’objectif n’est pas seulement de mesurer des paramètres physiques, mais de transformer ces mesures en informations utiles et facilement interprétables. Les données peuvent ensuite être consultées via TTN et Bleep, afin de suivre l’activité de la ruche, de repérer plus rapidement certaines anomalies et de faciliter la prise de décision.

Concrètement, ce système permet à un apiculteur, un chercheur ou un utilisateur technique d’installer une solution de suivi sur une ruche et d’obtenir une vision plus claire de son fonctionnement, sans intervention permanente.

Le projet IABEILLES associe ainsi l’Internet des objets, l’intelligence artificielle et l’apiculture afin de proposer un outil de surveillance moderne, simple à déployer et utile au quotidien.

2.1 Mission et objectif

La mission d’IABEILLES est de proposer une solution fiable et intelligente pour surveiller en temps réel l’activité autour des ruches, notamment la présence de frelons asiatiques.

Face à la pression prédatrice qui menace les colonies d’abeilles, ce système constitue un outil d’aide pour les apiculteurs et les chercheurs. En combinant l’intelligence artificielle, des capteurs adaptés aux conditions extérieures et une communication longue portée, il permet de suivre l’activité devant la ruche de manière plus précise.

L’objectif n’est pas uniquement de détecter la présence de frelons, mais aussi d’apporter une vision plus claire de la pression exercée sur les ruches afin de mieux protéger les abeilles.

3. Détail du matériel

Le tableau suivant présente les principaux composants utilisés dans le projet.

Matériel	Quantité	Coût unitaire	Coût total	Fonction
XIAO ESP32S3 Sense	1	16,90 €	16,90 €	Intègre le module d’intelligence artificielle pour la détection visuelle des abeilles et des frelons.
Module LoRa E5	1	18,90 €	18,90 €	Envoie les informations vers TTN.
DHT22	1	17,10 €	17,10 €	Mesure la température et l’humidité autour de la ruche.
Capteur de lumière BH1750 ADA4681	1	5,90 €	5,90 €	Mesure la luminosité ambiante.
Batterie LiPo 3,7 Vcc 1000 mAh MB-P3090003	1	23,90 €	23,90 €	Alimentation principale du système.
Arduino Nano BLE Sense	1	42,65 €	42,65 €	Intègre le modèle de détection sonore et récupère les informations des capteurs.
Buzzer	1	2,81 €	2,81 €	Émet une alerte sonore lors de la mise en activité du système.
Module chargeur LiPo DFR0264	1	5,20 €	5,20 €	Permet la recharge de la batterie, notamment via le panneau solaire.
Antenne LoRa	1	8,30 €	8,30 €	Permet l’émission et la réception des données du module LoRa.
Panneau photovoltaïque	1	12,90 €	12,90 €	Recharge la batterie.
Régulateur Pololu 3,3 V	1	11,00 €	11,00 €	Abaisse la tension de la batterie pour alimenter certains composants en 3,3 V.
Régulateur Pololu 5 V	1	11,00 €	11,00 €	Fournit une tension de sortie de 5 V.
Résistances 1/4 W 5 %	3	0,20 €	0,60 €	Permettent de réaliser un diviseur de tension pour contrôler le niveau de batterie.
Bouton d’alimentation	1	2,80 €	2,80 €	Permet d’allumer et d’éteindre le système.
Tendeurs	2	2,00 €	4,00 €	Permettent de fixer le boîtier de caméra à la ruche.
Impression 3D / découpe laser	-	-	0,00 €	Permet de fabriquer certaines pièces mécaniques du système.
PCB	2	50,00 €	100,00 €	Assure les connexions électriques entre les composants du circuit.
Câbles	-	0,20 €	0,00 €	Assurent la liaison entre les différents composants.
Boîtier étanche	1	5,90 €	5,90 €	Protège le circuit contre les agressions extérieures comme la pluie ou la poussière.
Joint d’étanchéité	-	6,00 €	0,00 €	Assure l’étanchéité du système.
Presse-étoupes	3	1,00 €	3,00 €	Permettent le passage des câbles à travers les parois du boîtier.
Breadboard	1	5,90 €	5,90 €	Permet de réaliser les tests du système avant la conception du PCB.

4. Montage matériel

Les schémas de câblage des deux cartes doivent être respectés afin de garantir le bon fonctionnement du système.

Il est important de connecter chaque composant à l’emplacement prévu. Le code fourni avec le projet a été développé pour cette architecture matérielle. Si une architecture différente est utilisée, il sera nécessaire d’adapter le code ainsi que le PCB.

Avant toute mise sous tension, vérifiez les points suivants :

* les cartes sont correctement connectées ;
* les capteurs sont branchés aux bons emplacements ;
* les câbles ne sont pas inversés ;
* l’alimentation est correctement reliée ;
* aucun fil dénudé ne risque de provoquer un court-circuit.

5. Logiciels requis

Arduino IDE

Arduino IDE est le logiciel utilisé pour ouvrir, lire, modifier et téléverser le code sur les cartes électroniques.

Il permet également de vérifier que le programme ne contient pas d’erreurs avant son envoi sur la carte.

The Things Network, TTN

The Things Network, souvent appelé TTN, est une plateforme permettant de recevoir les données transmises par le système via le réseau LoRaWAN.

Dans ce projet, TTN permet de récupérer les données envoyées par le module LoRaWAN E5.

Bleep

Bleep est une plateforme permettant de visualiser les données récupérées depuis TTN.

Elle permet à l’utilisateur de consulter les informations du dispositif de manière plus lisible.

6. Installation logicielle

6.1 Télécharger le code source

Pour installer l’outil, vous devez d’abord récupérer le code source depuis GitHub.

Rendez-vous sur le dépôt GitHub du projet à l’adresse suivante :

https://github.com/SchaafR/IAbeille

Une fois sur la page du dépôt, cliquez sur le bouton vert <> Code.

Deux méthodes de téléchargement sont possibles.

Option 1 : cloner le dépôt avec Git

Si Git est installé sur votre ordinateur, copiez l’URL du dépôt, puis exécutez la commande suivante dans un terminal :

git clone https://github.com/SchaafR/IAbeille.git

Cette commande télécharge automatiquement le projet sur votre ordinateur.

Option 2 : télécharger le projet au format ZIP

Si vous ne souhaitez pas utiliser Git, vous pouvez télécharger directement le projet au format ZIP.

Pour cela :

1. Cliquez sur le bouton <> Code.
2. Cliquez sur Download ZIP.
3. Une fois le téléchargement terminé, ouvrez le fichier ZIP.
4. Extrayez son contenu dans le dossier de votre choix.

Le code source du projet est maintenant disponible sur votre ordinateur.

6.2 Installer Arduino IDE

Télécharger Arduino IDE

Pour utiliser l’outil, vous devez installer Arduino IDE sur votre ordinateur.

Arduino IDE est le logiciel qui permet d’écrire, de vérifier et d’envoyer du code vers une carte Arduino ou compatible.

Rendez-vous sur le site officiel d’Arduino à l’adresse suivante :

https://www.arduino.cc/en/software

Choisissez ensuite la version adaptée à votre système d’exploitation :

* Windows ;
* macOS ;
* Linux.

Cliquez ensuite sur le bouton de téléchargement correspondant.

Installer Arduino IDE sous Windows

Une fois le fichier téléchargé :

1. Ouvrez le fichier d’installation.
2. Acceptez les conditions d’utilisation.
3. Laissez les options d’installation par défaut.
4. Cliquez sur Install.
5. Une fois l’installation terminée, cliquez sur Finish.

Arduino IDE est maintenant installé sur votre ordinateur.

Installer Arduino IDE sous macOS

Une fois le fichier téléchargé :

1. Ouvrez le fichier .dmg.
2. Glissez l’application Arduino IDE dans le dossier Applications.
3. Ouvrez ensuite Arduino IDE depuis le dossier Applications.

Lors du premier lancement, macOS peut demander une confirmation de sécurité. Dans ce cas, autorisez l’ouverture de l’application.

Installer Arduino IDE sous Linux

Une fois le fichier téléchargé :

1. Extrayez l’archive téléchargée.
2. Ouvrez le dossier extrait.
3. Suivez les instructions d’installation fournies sur le site officiel d’Arduino.

Vérifier l’installation

Après l’installation, ouvrez Arduino IDE.

Si le logiciel se lance correctement, l’installation est terminée. Vous pourrez ensuite connecter votre carte à l’ordinateur et configurer le type de carte ainsi que le port utilisé.

6.3 Installer une librairie Arduino depuis un dossier local

Le modèle d’intelligence artificielle est fourni sous forme de librairie Arduino. Il doit donc être ajouté aux librairies Arduino pour que le projet fonctionne correctement.

Récupérer la librairie

La librairie est déjà fournie avec le projet. Elle se trouve dans le dossier suivant :

IAbeille/data/BeeGuardAI_Hornet_Bee_Bees_G1V2_inferencing.zip

Une autre librairie doit également être installée de la même manière :

IAbeille/data/Hornets_lib_3.zip

Copier la librairie

Copiez le fichier .zip de la librairie à installer.

Il est important de conserver l’ensemble du contenu de la librairie. Ne copiez pas uniquement certains fichiers individuellement.

Coller la librairie dans le dossier Arduino

Vous devez maintenant placer la librairie dans le dossier des librairies Arduino.

Le chemin dépend de votre système d’exploitation.

Sous Windows

Chemin le plus courant :

C:\Users\VotreNomUtilisateur\Documents\Arduino\libraries

Sous macOS

Chemin le plus courant :

/Users/VotreNomUtilisateur/Documents/Arduino/libraries

Sous Linux

Chemin le plus courant :

/home/VotreNomUtilisateur/Arduino/libraries

Décompresser la librairie

Une fois le fichier .zip copié dans le dossier libraries, décompressez-le directement dans ce dossier.

Après décompression, vous pouvez supprimer le fichier .zip afin de ne conserver que le dossier de la librairie.

Vérifier l’installation

Une fois la librairie copiée et décompressée, redémarrez Arduino IDE.

Pour vérifier que la librairie est bien reconnue :

1. Ouvrez Arduino IDE.
2. Cliquez sur le menu Sketch.
3. Allez dans Include Library.
4. Vérifiez que le nom de la librairie apparaît dans la liste.

Si la librairie apparaît dans cette liste, elle est correctement installée et prête à être utilisée dans le projet.

Répétez la même procédure pour le fichier suivant :

Hornets_lib_3.zip

6.4 Installer les librairies Arduino nécessaires

Présentation

Avant de téléverser le programme sur la carte, certaines librairies doivent être installées dans Arduino IDE.

Les librairies permettent d’ajouter des fonctionnalités au programme, par exemple gérer une caméra, lire un capteur de température ou communiquer avec un capteur de luminosité.

Liste des librairies utilisées

Le programme utilise les librairies suivantes :

#include <Arduino.h>
#include "esp_camera.h"
#include <ctype.h>
#include <string.h>
#include <DHT.h>
#include <BH1750.h>
#include <Wire.h>

Voici le détail des librairies nécessaires :

Librairie	Utilité	Installation nécessaire
Arduino.h	Librairie de base d’Arduino.	Non, incluse avec Arduino IDE.
esp_camera.h	Gestion de la caméra ESP32.	Non, incluse avec le support de carte ESP32.
ctype.h	Fonctions de traitement de caractères.	Non, incluse par défaut.
string.h	Fonctions de manipulation de chaînes de caractères.	Non, incluse par défaut.
DHT.h	Gestion du capteur DHT.	Oui.
BH1750.h	Gestion du capteur de luminosité BH1750.	Oui.
Wire.h	Communication I2C entre la carte et certains capteurs.	Non, incluse avec Arduino IDE.

Installer les librairies depuis Arduino IDE

Les librairies DHT.h et BH1750.h doivent être installées manuellement depuis le gestionnaire de librairies Arduino.

Pour cela :

1. Ouvrez Arduino IDE.
2. Cliquez sur le menu Sketch.
3. Allez dans Include Library.
4. Cliquez sur Manage Libraries….
5. Dans la barre de recherche, recherchez la librairie à installer.
6. Sélectionnez la librairie correspondante.
7. Cliquez sur Install.

Librairie DHT

Dans le gestionnaire de librairies, recherchez :

DHT sensor library

Installez la librairie DHT sensor library.

Arduino IDE peut proposer d’installer des dépendances supplémentaires. Dans ce cas, acceptez leur installation.

Librairie BH1750

Dans le gestionnaire de librairies, recherchez :

BH1750

Installez la librairie BH1750.

Support de carte ESP32

La librairie esp_camera.h est disponible uniquement si le support des cartes ESP32 est installé dans Arduino IDE.

Si ce n’est pas encore fait, il faut installer le support ESP32 depuis le gestionnaire de cartes.

Pour cela :

1. Ouvrez Arduino IDE.
2. Cliquez sur File, puis Preferences.
3. Dans le champ Additional Boards Manager URLs, ajoutez l’URL du gestionnaire de cartes ESP32 si elle n’est pas déjà présente.
4. Cliquez sur OK.
5. Allez ensuite dans Tools.
6. Cliquez sur Board, puis Boards Manager.
7. Recherchez ESP32.
8. Installez le package esp32 by Espressif Systems.

Une fois le support ESP32 installé, la librairie esp_camera.h sera disponible automatiquement.

Vérifier l’installation

Après l’installation des librairies, redémarrez Arduino IDE.

Pour vérifier qu’une librairie est bien installée :

1. Cliquez sur Sketch.
2. Allez dans Include Library.
3. Vérifiez que les librairies installées apparaissent dans la liste.

Vous pouvez ensuite ouvrir le programme et lancer une vérification en cliquant sur le bouton Verify.

Si aucune erreur de librairie manquante n’apparaît, l’installation est correcte.

6.5 Configurer TTN pour recevoir les données du module LoRaWAN E5

Présentation

Pour recevoir les données envoyées par le module LoRaWAN E5, il est nécessaire d’utiliser une plateforme réseau LoRaWAN.

Dans ce projet, la plateforme utilisée est TTN, aussi appelée The Things Network. TTN permet de connecter un appareil LoRaWAN à Internet afin de visualiser les données envoyées par le module.

Le fonctionnement général est le suivant :

Module LoRaWAN E5 → Passerelle LoRaWAN → TTN → Visualisation des données

Le module LoRaWAN E5 envoie les données par radio. Une passerelle LoRaWAN à proximité reçoit ces données, puis les transmet à TTN via Internet.

Prérequis

Avant de commencer, assurez-vous d’avoir :

* un compte TTN ;
* un module LoRaWAN E5 configuré ;
* une passerelle LoRaWAN disponible à proximité ;
* les identifiants LoRaWAN du module :
    * DevEUI ;
    * JoinEUI, aussi appelé AppEUI ;
    * AppKey ;
* la bonne région radio, par exemple Europe 863-870 MHz pour la France.

Le module et la passerelle doivent utiliser le même plan de fréquence pour pouvoir communiquer correctement.

Créer un compte TTN

Rendez-vous sur le site de TTN :

https://www.thethingsnetwork.org/

Cliquez sur Sign up ou Get started afin de créer un compte.

Une fois le compte créé, connectez-vous à la console TTN.

Pour la version communautaire de The Things Stack Sandbox, la console est disponible à l’adresse suivante :

https://console.cloud.thethings.network/

Créer une application

Une fois connecté à la console TTN :

1. Cliquez sur Applications.
2. Cliquez sur Create application.
3. Renseignez un nom pour l’application.

Exemple :

iabeille-application

4. Ajoutez une description si nécessaire.
5. Cliquez sur Create application.

L’application sert à regrouper les appareils LoRaWAN du projet. C’est dans cette application que le module LoRaWAN E5 sera ajouté.

Ajouter le module LoRaWAN E5 dans TTN

Dans l’application que vous venez de créer :

1. Allez dans le menu End devices.
2. Cliquez sur Register end device.

TTN propose plusieurs méthodes d’ajout. Dans le cas d’un module LoRaWAN E5 configuré manuellement, choisissez généralement :

Enter end device specifics manually

Renseigner les informations du module

Vous devez maintenant compléter les informations du module.

Plan de fréquence

Pour une utilisation en France ou en Europe, sélectionnez généralement :

Europe 863-870 MHz

Le plan de fréquence doit correspondre à celui utilisé par le module LoRaWAN E5 et par la passerelle LoRaWAN.

Version LoRaWAN

Sélectionnez la version LoRaWAN utilisée par votre module.

Exemples courants :

LoRaWAN Specification 1.0.2

ou :

LoRaWAN Specification 1.0.3

La version exacte dépend de la configuration du module LoRaWAN E5.

Mode d’activation

Choisissez le mode d’activation suivant :

OTAA

OTAA signifie Over-The-Air Activation. C’est la méthode recommandée, car elle permet au module de rejoindre le réseau de manière sécurisée.

Renseigner les identifiants LoRaWAN

Pour que TTN reconnaisse le module, vous devez renseigner les identifiants suivants.

DevEUI

Le DevEUI est l’identifiant unique du module LoRaWAN.

Exemple :

70B3D57ED0000001

JoinEUI / AppEUI

Le JoinEUI, parfois appelé AppEUI, identifie l’application ou le serveur de jointure.

Exemple :

0000000000000000

AppKey

L’AppKey est une clé de sécurité utilisée lors de l’activation OTAA.

Exemple :

00112233445566778899AABBCCDDEEFF

Ces valeurs doivent être identiques à celles configurées dans le programme du module LoRaWAN E5. Si une seule valeur est différente, le module ne pourra pas rejoindre le réseau TTN.

Finaliser l’ajout de l’appareil

Une fois les informations renseignées :

1. Donnez un nom à l’appareil dans le champ End device ID.

Exemple :

iabeille-lorawan-e5

2. Vérifiez toutes les informations.
3. Cliquez sur Register end device.

Le module LoRaWAN E5 est maintenant enregistré dans TTN.

Vérifier la réception des données

Après l’enregistrement du module :

1. Ouvrez votre application TTN.
2. Cliquez sur End devices.
3. Sélectionnez votre module LoRaWAN E5.
4. Ouvrez l’onglet Live data.

Lorsque le module envoie des données, elles doivent apparaître dans cette page.

Vous pouvez y voir notamment :

* les tentatives de connexion au réseau ;
* les messages de type Join request ;
* les messages de type Join accept ;
* les données reçues, appelées uplinks ;
* le contenu brut envoyé par le module.

Comprendre les données reçues

Les données envoyées par le module apparaissent généralement sous forme de valeur brute, souvent encodée en hexadécimal ou en Base64.

Exemple :

{
  "frm_payload": "AQIDBA=="
}

Cette valeur correspond aux données envoyées par le module. Pour les rendre lisibles, il peut être nécessaire d’ajouter un Payload formatter dans TTN.

Ajouter un décodeur de données

Pour transformer les données brutes en valeurs compréhensibles, vous pouvez utiliser un Payload formatter.

Dans TTN :

1. Ouvrez votre application.
2. Cliquez sur Payload formatters.
3. Sélectionnez Uplink.
4. Choisissez le format Javascript formatter.
5. Copiez-collez le contenu du fichier suivant :

IAbeille/src/payload formateur

6. Cliquez sur Save changes.

Problèmes fréquents

Aucune donnée n’apparaît dans TTN

Vérifiez que :

* le module LoRaWAN E5 est bien alimenté ;
* une passerelle LoRaWAN est disponible à proximité ;
* le module utilise le bon plan de fréquence ;
* le DevEUI, le JoinEUI et l’AppKey sont corrects ;
* le mode d’activation est bien OTAA ;
* le programme du module envoie bien des données.

Le module n’arrive pas à rejoindre le réseau

Vérifiez les messages dans l’onglet Live data.

Si vous voyez des erreurs de type Join failed, le problème vient souvent d’une clé incorrecte, d’une mauvaise région radio ou d’une version LoRaWAN mal configurée.

Les données sont illisibles

Si les données apparaissent sous forme brute, cela signifie qu’il manque probablement un décodeur de payload.

Dans ce cas, ajoutez ou adaptez le Payload formatter afin de convertir les données reçues en valeurs lisibles.

Résultat attendu

Une fois la configuration terminée, TTN doit afficher les messages envoyés par le module LoRaWAN E5 dans l’onglet Live data.

L’utilisateur peut alors vérifier que les données sont bien reçues et suivre les valeurs transmises par le projet.

6.6 Configurer Bleep pour visualiser les données

Bleep permet de visualiser les données transmises par le dispositif de manière plus lisible.

Une fois les données correctement reçues sur TTN, connectez-vous à la plateforme Bleep utilisée pour le projet. Vérifiez ensuite que les données associées à votre dispositif apparaissent bien dans l’interface.

Si aucune donnée n’apparaît dans Bleep, vérifiez d’abord que les données sont bien visibles dans TTN. Si TTN ne reçoit aucune donnée, le problème vient probablement de la configuration LoRaWAN, de la passerelle ou du module.

7. Implémentation logicielle

7.1 Téléverser le code sur la carte avec Arduino IDE

Présentation

Une fois Arduino IDE installé et les bibliothèques nécessaires ajoutées, il faut envoyer le programme sur la carte.

Cette opération s’appelle le téléversement ou le flashage. Elle permet de transférer le code depuis l’ordinateur vers la carte Arduino ou compatible.

Ouvrir le projet dans Arduino IDE

Commencez par ouvrir le fichier principal du programme.

1. Lancez Arduino IDE.
2. Cliquez sur File.
3. Cliquez sur Open.
4. Sélectionnez le fichier du projet.

Le fichier à ouvrir possède généralement l’extension suivante :

.ino

Dans le cadre de ce projet, ouvrez le fichier suivant :

COM_N_AI.ino

Une fois le fichier ouvert, le code du programme apparaît dans Arduino IDE.

Brancher la carte à l’ordinateur

Branchez la carte XIAO à l’ordinateur à l’aide d’un câble USB.

Il est recommandé d’utiliser un câble USB permettant le transfert de données. Certains câbles USB servent uniquement à la recharge et ne permettent pas de communiquer avec la carte.

Lorsque la carte est correctement branchée, une LED s’allume généralement sur celle-ci.

Sélectionner le type de carte

Arduino IDE doit savoir quel type de carte est utilisé.

Pour sélectionner la carte :

1. Cliquez sur le menu Tools.
2. Allez dans Board.
3. Sélectionnez le modèle correspondant à votre carte.

Pour ce projet, sélectionnez :

XIAO ESP32S3

Le choix de la carte est important. Si le mauvais modèle est sélectionné, le code peut ne pas se téléverser correctement ou ne pas fonctionner comme prévu.

Si la carte n’est pas déjà installée dans Arduino IDE :

1. Cliquez sur Board Manager.
2. Recherchez ESP32 by Espressif Systems.
3. Installez le package correspondant.
4. Sélectionnez ensuite la carte XIAO ESP32S3 dans la liste des cartes disponibles.

Sélectionner le port USB

Il faut ensuite sélectionner le port sur lequel la carte est connectée.

1. Cliquez sur le menu Tools.
2. Allez dans Port.
3. Sélectionnez le port correspondant à la carte.

Exemples de ports possibles sous Windows :

COM3
COM4
COM5

Exemples de ports possibles sous macOS ou Linux :

/dev/ttyUSB0
/dev/ttyACM0
/dev/cu.usbserial

Si plusieurs ports sont affichés, débranchez la carte puis rebranchez-la. Le port qui apparaît après le branchement est généralement celui de la carte.

Vérifier le code

Avant d’envoyer le programme sur la carte, il est conseillé de vérifier que le code ne contient pas d’erreur.

Pour cela, cliquez sur le bouton Verify. Ce bouton est représenté par une icône en forme de coche :

✓

Arduino IDE va alors compiler le programme.

Si tout est correct, un message similaire apparaît en bas de la fenêtre :

Compilation done.

ou :

Done compiling.

Si une erreur apparaît, le téléversement ne pourra pas être effectué. Il faut alors corriger l’erreur indiquée avant de continuer.

Téléverser le code sur la carte

Une fois le code vérifié, vous pouvez l’envoyer sur la carte.

Cliquez sur le bouton Upload. Ce bouton est représenté par une flèche vers la droite :

→

Arduino IDE compile à nouveau le programme, puis le transfère vers la carte.

Pendant cette étape, il ne faut pas débrancher la carte.

Lorsque le téléversement est terminé, un message apparaît en bas de la fenêtre :

Upload done.

ou :

Done uploading.

Le programme est maintenant installé sur la carte.

Vérifier le fonctionnement du programme

Le moniteur série permet d’afficher les messages envoyés par la carte vers l’ordinateur. Il est très utile pour vérifier que le programme fonctionne correctement.

Pour l’ouvrir :

1. Cliquez sur Tools.
2. Cliquez sur Serial Monitor.

Vous pouvez aussi utiliser l’icône du moniteur série en haut à droite d’Arduino IDE.

Une fois le moniteur série ouvert, vérifiez que la vitesse de communication est correcte.

La vitesse la plus courante est :

9600 baud

Cependant, certains programmes utilisent une autre vitesse, par exemple :

115200 baud

La vitesse sélectionnée dans le moniteur série doit correspondre à celle indiquée dans le code avec la ligne :

Serial.begin(9600);

ou :

Serial.begin(115200);

Résultat attendu

À la fin de cette étape, le programme est correctement envoyé sur la carte.

Dans le cadre de ce projet, il est possible qu’une détection soit réalisée sans affichage immédiat de bounding boxes. Si vous ne voyez pas de rectangles de détection et que les scores restent à 0, cela peut être normal selon les conditions de test.

La carte peut maintenant fonctionner de manière autonome avec le code installé. Elle exécutera automatiquement ce programme à chaque démarrage ou à chaque remise sous tension.

Problèmes fréquents

La carte n’apparaît pas dans les ports

Vérifiez que :

* le câble USB permet bien le transfert de données ;
* la carte est correctement branchée ;
* le pilote USB de la carte est installé ;
* Arduino IDE a été redémarré après le branchement.

Le téléversement échoue

Vérifiez que :

* le bon type de carte est sélectionné ;
* le bon port USB est sélectionné ;
* aucune autre application n’utilise le port série ;
* le câble USB fonctionne correctement ;
* les bibliothèques nécessaires sont bien installées.

8. Intégration et mise en service

8.1 Connecter les composants

Avant de mettre le dispositif sous tension, vérifiez que toutes les cartes électroniques et tous les composants sont correctement connectés aux emplacements prévus.

Assurez-vous notamment que :

* les cartes sont bien insérées ;
* les connecteurs sont correctement branchés ;
* aucun câble n’est débranché ou mal positionné ;
* la batterie est connectée ;
* les capteurs sont reliés aux bons emplacements.

Cette vérification permet d’éviter un mauvais fonctionnement du système lors du premier démarrage.

8.2 Allumer le dispositif

Une fois les branchements vérifiés, allumez le dispositif à l’aide du bouton On/Off.

Au démarrage, le système doit émettre un son avec le buzzer pendant environ :

0,5 seconde

Ce signal sonore indique que le dispositif est bien alimenté et que le programme démarre correctement.

8.3 Vérifier la recharge de la batterie

Après l’allumage du dispositif, vérifiez que la batterie est correctement prise en charge par le module de recharge.

Pour cela, observez la LED présente sur la carte LiPo DFR0264.

Si la LED de la carte s’allume, cela indique que le module détecte la batterie et que le circuit de recharge fonctionne correctement.

8.4 Vérifier la réception des données

Une fois le dispositif allumé, les données doivent être transmises automatiquement.

Vous devez vérifier que les informations sont bien reçues sur les plateformes prévues :

* TTN ;
* Bleep.

Vérification sur TTN

Connectez-vous à votre compte The Things Network.

Dans l’application correspondant au projet, ouvrez l’onglet Live data de votre appareil.

Vous devez voir apparaître les messages envoyés par le dispositif.

Vérification sur Bleep

Connectez-vous à la plateforme Bleep utilisée pour le projet.

Vérifiez que les données du dispositif sont bien visibles et qu’elles se mettent à jour correctement.

8.5 Valider le fonctionnement complet du système

Avant d’installer le dispositif sur la ruche, assurez-vous que l’ensemble du système fonctionne correctement.

Les points suivants doivent être validés :

* le dispositif s’allume correctement ;
* le buzzer émet bien un son au démarrage ;
* la batterie est détectée et peut se recharger ;
* les données sont reçues sur TTN ;
* les données sont reçues sur Bleep ;
* aucun câble ou composant ne semble mal connecté.

Cette étape est importante afin de s’assurer que le dispositif est opérationnel avant son installation définitive.

8.6 Fixer le système à la ruche

Une fois toutes les vérifications terminées et le fonctionnement validé, vous pouvez fixer le dispositif à la ruche.

Veillez à installer le système de manière stable et sécurisée, afin qu’il reste bien en place pendant son utilisation.

Le dispositif est maintenant prêt à fonctionner sur la ruche.

9. Remerciements

Nous tenons à remercier l’ensemble des personnes ayant contribué, directement ou indirectement, à la réalisation du projet IABEILLES.

Nous remercions particulièrement l’équipe pédagogique de Polytech Sorbonne pour son accompagnement, ses conseils et le suivi apporté tout au long du projet.

Nous remercions également les encadrants, enseignants et intervenants qui nous ont aidés à faire évoluer la solution, à surmonter les difficultés techniques et à structurer notre démarche de conception.

Enfin, nous remercions toutes les personnes ayant participé aux échanges, aux tests et aux retours d’expérience, qui ont permis d’améliorer le système et de rendre ce guide plus clair pour les futurs utilisateurs.
