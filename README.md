# IAbeille
 Système autonome de détection audio et visuelle des frelons asiatiques sur une ruche basé sur l'intelligence artificielle.<br>
 
La version actuelle implémente : <br>
- Détection audio et visuelle de frelons à l'aide de modèles d'IA entrainés via Edge-Impulse, <br>
- Uplink et Downlink sur TTN via un module Lorawan,<br>
- Mesure de la température, humidité et luminosité, <br>
- Gestion de la charge, envois et mesures par une machine à état, <br>
- Charge par panneau solaire d'une batterie de 1000mAh <br>

 ## Structure des dossiers : <br>
 /doc -> Documents techniques et de suivi du projet<br>
  /article -> Articles utilisés pour le développement de l'IA son
 /src -> Codes du projet.<br>
  /ComNANO_BLE (communication) et /NanoBLE_FS (complet) -> Gestion de l'Arduino Nano Sense 33 <br> 
  /XiaO_FS (complet) -> Gestion de la Xiao Sense <br>
 /tests -> Tests et brouillons éventuels.<br>
 hardware.zip -> PCB, boitier, schémas<br>
 /data -> Modèles d'IA (edge impulse ou autre).<br>

 ## Informations
 - Pour la compilation du projet sous Arduino IDE les librairies de /data peuvent être importées en allant dans Sketch->Include librairy->Add .ZIP librairy...<br>
 - Deux modèles edge-impulse avec les mêmes paramètres peuvent être combinés en un modèle : https://forum.edgeimpulse.com/t/multiple-models-in-same-device/6086/8
 

 
