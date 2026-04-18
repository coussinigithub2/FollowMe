Malheureusement, je ne peux pas générer directement un fichier PDF téléchargeable. Cependant, je peux vous fournir la traduction complète et structurée du document afin que vous puissiez la copier dans un traitement de texte (comme Word ou Google Docs) et l'enregistrer au format PDF.

Voici la traduction en français de la documentation du script **Follow Me (Version 1.3)** :

---

# Script Follow Me
[cite_start]**Par Velfire, Juillet 2019** [cite: 2]

## [cite_start]Ce qu'il FAIT [cite: 3]
* [cite_start]Guide le pilote de la porte de départ jusqu'au seuil de la piste (si le trajet ne nécessite pas de rouler sur la piste elle-même)[cite: 5].
* [cite_start]Guide le pilote de la piste jusqu'à la porte d'arrivée[cite: 6].
* [cite_start]Demande automatique de la voiture Follow Me (FM) au départ si les informations de piste sont lues dans le FMC et que les conditions sont remplies[cite: 7, 8].
* [cite_start]Demande automatique de la voiture FM à l'arrivée si les conditions préalables sont remplies[cite: 9].

## [cite_start]Fonctionnalités [cite: 10]
* [cite_start]La voiture FM apparaît automatiquement et guide l'avion vers la piste ou la porte sans intervention de l'utilisateur[cite: 13, 14].
* [cite_start]Signale au pilote de ralentir pour un virage en indiquant la direction de celui-ci[cite: 15].
* [cite_start]Emprunte un itinéraire plus court avec moins de virages (qui n'est pas forcément l'itinéraire le plus court en distance absolue)[cite: 17].

## [cite_start]Ce qu'il NE FAIT PAS [cite: 18]
* [cite_start]Permettre à l'utilisateur de spécifier un itinéraire souhaité[cite: 19].
* [cite_start]Afficher une carte de l'aéroport ou votre position sur une carte[cite: 20].
* [cite_start]Circuler sur la piste si l'avion doit l'emprunter pour faire un demi-tour (le véhicule s'arrêtera à l'entrée de la piste, appelée "zone chaude")[cite: 21, 22].
* [cite_start]Garantir que la porte assignée n'est pas déjà occupée[cite: 23].
* [cite_start]Cacher l'onglet "Follow Me" sur le côté droit de l'écran[cite: 25].

---

## [cite_start]INSTALLATION [cite: 27]
**Prérequis :**
* [cite_start]**X-Plane 11**[cite: 29].
* [cite_start]Plugin **FlyWithLua (FWL)** installé (version testée : 2.7.17)[cite: 30].
* [cite_start]Aéroport possédant des routes de taxi ATC définies[cite: 31].

[cite_start]**Instructions :** [cite: 32]
1. [cite_start]Dézippez le fichier FollowMe[cite: 33].
2. [cite_start]Placez le script `.lua` dans : `X-Plane 11 > Resources > plugins > FlyWithLua > Scripts`[cite: 34, 35].
3. [cite_start]Placez le dossier `follow_me` dans le même répertoire[cite: 36, 37].

---

## [cite_start]OPÉRATIONS [cite: 38]
### Fenêtre de l'interface utilisateur
1. [cite_start]Cliquez sur l'onglet **FM** à droite de l'écran pour ouvrir l'interface[cite: 40].
2. [cite_start]Vous pouvez faire glisser l'onglet verticalement pour le repositionner[cite: 41].

### [cite_start]Préférences [cite: 77]
* [cite_start]Vous pouvez sauvegarder la position de l'onglet FM (indépendamment de l'avion utilisé)[cite: 78].
* [cite_start]Les autres réglages sont liés au modèle d'avion spécifique[cite: 79].
* [cite_start]Cliquez sur **"Save Pref."** pour enregistrer vos réglages dans le fichier `FollowMe.prf`[cite: 80, 81, 82].

### [cite_start]Spécifier le type d'avion [cite: 83]
[cite_start]Cette information permet au plugin de déterminer les portes et les voies de circulation (taxiways) adaptées à la taille de votre appareil (ex: un A380 ne peut pas passer partout)[cite: 84, 85, 86].

---

## PARAMÈTRES DE DÉPART ET D'ARRIVÉE
### [cite_start]Départ [cite: 89]
* [cite_start]Sélectionnez la piste dans la liste déroulante[cite: 93, 94].
* [cite_start]Cochez **"Get from FMS"** pour que le plugin récupère automatiquement la piste depuis votre ordinateur de bord[cite: 96].

### [cite_start]Arrivée [cite: 99]
* [cite_start]Cochez **"Auto Assign"** pour qu'une porte vous soit attribuée automatiquement lors du toucher des roues[cite: 101, 107].
* Pour vous aider à vous garer précisément, cliquez sur **"Show Ramp Start"** : un diamant apparaîtra à la porte. [cite_start]Garez-vous face à la face **ROUGE** du diamant[cite: 114, 115, 116].

### [cite_start]Limiter la vitesse à 20 kts [cite: 130]
* [cite_start]Si coché, la voiture ne dépassera pas 20 nœuds[cite: 131].
* Si décoché, la voiture (une Ferrari 458 dans le simulateur) peut aller très vite. [cite_start]Attention à ne pas percuter le véhicule ou à ne pas manquer votre virage ![cite: 133, 137, 138, 139].

---

## CONDITIONS POUR LA DEMANDE AUTOMATIQUE
[cite_start]**Au départ :** [cite: 140]
1. [cite_start]Case "Get from FMS" cochée[cite: 141].
2. [cite_start]Piste de départ spécifiée dans le FMS[cite: 141].
3. [cite_start]Feux Stroboscopiques (Strobe) **OU** feux de Taxi allumés[cite: 142].

[cite_start]**À l'arrivée :** [cite: 144]
1. [cite_start]Case "Auto Assign" cochée[cite: 145].
2. [cite_start]Vitesse sol inférieure à 30 kts[cite: 146].
3. [cite_start]Fenêtre de l'interface FM fermée[cite: 147].
4. [cite_start]Feux de Taxi allumés[cite: 148].

---

## [cite_start]NOTES POUR LES DÉVELOPPEURS [cite: 190]
[cite_start]L'auteur a utilisé le langage **Lua** via FlyWithLua car la syntaxe est plus accessible que le C++[cite: 203, 204]. [cite_start]Le script utilise l'algorithme de recherche **A*** pour trouver les itinéraires le plus rapidement et efficacement possible[cite: 231].