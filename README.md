# Mycromesh

Systeme de reseau mesh LoRa pour noeuds IoT distribues, base sur ESP32-C3.

**Version firmware :** 1.4.0

---

## Table des matieres

1. [Presentation](#presentation)
2. [Materiel requis](#materiel-requis)
3. [Brochage](#brochage)
4. [Installation](#installation)
5. [Configuration](#configuration)
6. [Commandes](#commandes)
7. [Protocole reseau](#protocole-reseau)
8. [Routage Mesh (Dijkstra)](#routage-mesh-dijkstra)
9. [Transfert de fichiers](#transfert-de-fichiers)
10. [Correction d'erreurs Reed-Solomon](#correction-derreurs-reed-solomon)
11. [Planificateur Cron](#planificateur-cron)
12. [Gestion d'energie](#gestion-denergie)
13. [Capteurs](#capteurs)
14. [Systeme de passerelle](#systeme-de-passerelle)
15. [Procedure de demarrage](#procedure-de-demarrage)
16. [Mise a jour firmware (OTA)](#mise-a-jour-firmware-ota)
17. [Diffusion reseau (broadcast)](#diffusion-reseau-broadcast)
18. [Tunnel reseau E/S (netio)](#tunnel-reseau-es-netio)
19. [Mode Bluetooth (BLE)](#mode-bluetooth-ble)
20. [Fichiers carte SD](#fichiers-carte-sd)
21. [Parametres configurables](#parametres-configurables)
22. [Limites du systeme](#limites-du-systeme)
23. [Reference des fonctions](#reference-des-fonctions)

---

## Presentation

Mycromesh est un firmware pour microcontroleur ESP32-C3 qui cree un reseau mesh (maille) sans fil utilisant la technologie radio LoRa a 433 MHz. Chaque noeud du reseau peut :

- **Communiquer** avec les autres noeuds via un routage intelligent (Dijkstra)
- **Collecter** des donnees environnementales (temperature, humidite, pression, gaz)
- **Transmettre** des fichiers avec correction d'erreurs Reed-Solomon
- **Planifier** des taches automatiques via un systeme cron
- **Economiser** l'energie grace au deep sleep avec reveil programmable
- **Se mettre a jour** a distance via OTA (Over-The-Air)

Le reseau est auto-organisable : chaque noeud decouvre ses voisins, partage la topologie et calcule les chemins optimaux automatiquement.

---

## Materiel requis

| Composant | Description |
|-----------|-------------|
| ESP32-C3 | Microcontroleur principal |
| Module LoRa | Radio 433 MHz (SX1276/SX1278 compatible) |
| Carte SD | Stockage de configuration et donnees |
| BME680 | Capteur environnemental temperature/humidite/pression/gaz (optionnel) |
| AHT20 | Capteur temperature/humidite I2C (optionnel) |
| BMP280 | Capteur pression/temperature I2C (optionnel) |
| DS18B20 | Capteur temperature OneWire (optionnel) |

### Dependances logicielles (bibliotheques Arduino)

- `LiteLora` - Driver LoRa leger (bibliotheque embarquee)
- `SPI` - Bus SPI pour LoRa et SD
- `SD` - Gestion de la carte SD
- `ESP32Time` - Horloge temps reel logicielle
- `Adafruit_BME680` / `Adafruit_Sensor` - Capteur BME680
- `Adafruit_BMP280` - Capteur BMP280
- `Adafruit_AHT10` - Capteur AHT20
- `NimBLEDevice` - Bluetooth Low Energy (Nordic UART Service)
- `OneWire` / `DallasTemperature` - Capteur DS18B20
- `Wire` - Bus I2C
- `Preferences` - Stockage persistant NVS
- `Update` - Mise a jour firmware OTA
- `esp_sleep` / `driver/gpio` - Gestion du deep sleep

---

## Brochage

```
ESP32-C3
--------
Pin 21  ->  CS LoRa (SPI Chip Select)
Pin  8  ->  IRQ LoRa (Interruption de reception)
Pin  7  ->  CS Carte SD (SPI Chip Select)
Pin 20  ->  Controle alimentation module LoRa
Pin  1  ->  Entree de reveil deep sleep (GPIO wakeup)
Pin  3  ->  Bus OneWire (capteur DS18B20)
I2C     ->  Capteurs BME680, AHT20, BMP280
```

Le bus SPI est partage entre le module LoRa et la carte SD. Le firmware commute les lignes GPIO (pins 20 et 21) pour multiplexer l'acces aux deux peripheriques.

---

## Installation

1. Installer l'environnement Arduino IDE avec le support ESP32
2. Installer les bibliotheques requises via le gestionnaire de bibliotheques
3. Connecter le materiel selon le brochage ci-dessus
4. Preparer la carte SD avec les fichiers de configuration (voir [Fichiers carte SD](#fichiers-carte-sd))
5. Compiler et telecharger `main/main.ino` sur l'ESP32-C3
6. Ouvrir le moniteur serie a 9600 bauds

---

## Configuration

La configuration se fait via des fichiers sur la carte SD. Au premier demarrage, les valeurs par defaut sont utilisees si `localAddress == 0` :

| Parametre | Valeur par defaut | Description |
|-----------|-------------------|-------------|
| `localAddress` | 61 | Adresse unique de la station |
| `MAX_PING_AGE` | 20000 ms | Timeout des pings |
| `starttimeout` | 15000 ms | Timeout de la procedure de demarrage |
| `DELAY` | 3600000 ms | Duree de retention des donnees temporaires |
| `MAX_ENTRY_AGE` | 20000 ms | Timeout des messages en attente d'acquittement |
| `actiontimerdel` | 30 s | Delai d'inactivite avant deep sleep |
| `maintmode` | true | Mode maintenance (empeche le deep sleep) |
| `stationgateway` | 1 | Adresse de la passerelle |
| `TOGATE_COMMAND_TIMEOUT` | 60000 ms | Timeout des commandes passerelle |
| `ioMode` | 0 (USB) | Canal E/S (0=USB, 1=Bluetooth) |
| `NETIO_TIMEOUT` | 300000 ms | Timeout d'inactivite du tunnel reseau |

### Configuration des capteurs

Les capteurs actifs sont configures dans `/sensor.cfg` avec le format :

```
bme680:<0|1>:aht20:<0|1>:bmp280:<0|1>:ds18b20:<0|1>
```

Exemple pour activer le BME680 et le BMP280 :
```
bme680:1:aht20:0:bmp280:1:ds18b20:0
```

La commande `getsensor` affiche la configuration actuelle et `setsensor:<bme680>:<aht20>:<bmp280>:<ds18b20>` la modifie a chaud.

---

## Commandes

Toutes les commandes sont envoyees via le port serie (9600 bauds) ou recues par LoRa. Le format general est :

```
commande:parametre1:parametre2:...
```

### Commandes reseau

| Commande | Format | Description |
|----------|--------|-------------|
| `ping` | `ping` | Repond avec un rpin au noeud emetteur (RSSI) |
| `pigo` | `pigo` | Lance une decouverte complete des voisins (3 phases de ping) |
| `rpin` | `rpin:<rssi>` | Reponse a un ping, contient le RSSI mesure |
| `umap` | `umap:<id>:<vertex>:<v1,v2,w>:...` | Mise a jour de la topologie reseau (broadcast) |
| `dijk` | `dijk:<dest>:<src>:<len>:<id>:<payload>` | Message route via Dijkstra avec acquittement |
| `gmap` | `gmap` | Demande la carte reseau complete au noeud emetteur |
| `tmap` | `tmap:<v1,v2,w>:<v1,v2,w>:...` | Reponse a gmap, envoie la topologie |
| `prnt` | `prnt` | Affiche la table des aretes sur le port serie |
| `clear` | `clear` | Supprime toutes les aretes du graphe |
| `expv` | `expv` | Exporte la topologie par vertex sur le port serie |

### Commandes de transmission

| Commande | Format | Description |
|----------|--------|-------------|
| `send` | `send:<dest>:<payload>` | Envoie un message route. Si > 225 octets, utilise automatiquement le transfert de fichier |
| `trsm` | `trsm:<dest>:<payload>` | Envoie un message direct avec preambule de reveil |
| `trsms` | `trsms:<dest>:<payload>` | Envoie un message direct sans preambule de reveil |
| `trsp` | `trsp:<dest>:<payload>` | Envoi transparent avec generation d'ID et acquittement |
| `load` | `load:<src>:<len>:<id>:<payload>` | Envoie des donnees avec acquittement (arok) |

### Commandes d'acquittement

| Commande | Format | Description |
|----------|--------|-------------|
| `arok` | `arok:<id>:<station>` | Acquittement de reception d'un load |
| `rxok` | `rxok:<id>:<station>` | Acquittement de reception d'un dijk |

### Commandes temps

| Commande | Format | Description |
|----------|--------|-------------|
| `geth` | `geth` | Demande l'heure au noeud emetteur |
| `seth` | `seth:<epoch>` | Regle l'heure (avec compensation du delai de transmission) |
| `prgh` | `prgh:<epoch>` | Regle l'heure directement (sans compensation) |
| `stam` | `stam:<epoch>` | Regle l'heure et marque la procedure de demarrage comme terminee |
| `difh` | `difh:<id>:<epoch>` | Synchronisation horaire distribuee (broadcast) |
| `fdih` | `fdih` | Demande de resynchronisation horaire |
| `gdfh` | `gdfh` | Initie une synchronisation horaire distribuee (broadcast) |
| `acth` | `acth` | Demande asynchrone de l'heure au noeud le plus proche |
| `time` | `time` | Affiche l'heure actuelle sur le port serie |

### Commandes transfert fichier

| Commande | Format | Description |
|----------|--------|-------------|
| `stft` | `stft:<dest>:<chemin>:<supprimer>` | Demarre un transfert de fichier vers dest |
| `isrf` | `isrf:<src>` | Signale que le recepteur est pret |
| `rfok` | `rfok:<station>` | Acquittement de connexion RX fichier |
| `fend` | `fend:<src>:<chemin>:<supprimer>` | Fin de transfert, demande la compilation |
| `feok` | `feok:<station>:<chemin>:<supprimer>` | Confirmation de reception fichier OK |
| `compileFile` | `compileFile:<nom>:<origin>:<suppr>` | Assemble le fichier recu avec correction d'erreurs |
| `file` | `file:<data>` | Recoit une ligne de donnees fichier et l'ecrit dans /rx.txt |
| `startfile` | `startfile:<dest>` | Demarre l'export du fichier tx.txt manuellement |
| `stopfile` | `stopfile` | Arrete l'export du fichier tx.txt |
| `expfile` | `expfile` | Exporte une ligne du fichier tx.txt |
| `filestate` | `filestate` | Affiche l'etat du transfert fichier en cours |

### Commandes capteur et donnees

| Commande | Format | Description |
|----------|--------|-------------|
| `gmea` | `gmea:<version>` | Effectue une mesure capteur et l'enregistre dans le datadump |
| `dexp` | `dexp:<version>` | Exporte les donnees du datadump vers la passerelle |
| `data` | `data:<payload>` | Affiche les donnees recues sur le port serie |

### Commandes systeme

| Commande | Format | Description |
|----------|--------|-------------|
| `read` | `read` | Recharge la configuration depuis la carte SD |
| `write` | `write` | Sauvegarde la configuration sur la carte SD |
| `getcfg` | `getcfg` | Affiche la configuration complete courante |
| `setcfg` | `setcfg:<params>` | Applique une configuration complete |
| `getsensor` | `getsensor` | Affiche la configuration des capteurs |
| `setsensor` | `setsensor:<bme680>:<aht20>:<bmp280>:<ds18b20>` | Active/desactive les capteurs |
| `maint` | `maint` | Active le mode maintenance (empeche le deep sleep) |
| `norm` | `norm` | Desactive le mode maintenance (autorise le deep sleep) |
| `star` | `star` | Lance la procedure de demarrage reseau |
| `reboot` | `reboot` | Sauvegarde la config et redemarrage |
| `upgrade` | `upgrade` | Applique une mise a jour firmware depuis la SD |
| `version` | `version` | Affiche la version du firmware |
| `parm` | `parm:<nom>:<valeur>` | Modifie un parametre en cours d'execution |
| `slvl` | `slvl:<none\|normal\|verbose\|debug>` | Regle le niveau de log |
| `iomd` | `iomd:<usb\|bt>` | Change le canal E/S (USB ou Bluetooth) |
| `sho` | `sho` | Affiche l'etat du mode maintenance |
| `gate` | `gate` | Affiche l'adresse de la station passerelle |
| `cachestate` | `cachestate` | Affiche l'etat du cache passerelle |
| `cout` | `cout` | Force l'export du cache passerelle |
| `mkdir` | `mkdir:<chemin>` | Cree un repertoire sur la carte SD |
| `rmdir` | `rmdir:<chemin>` | Supprime un repertoire sur la carte SD |
| `rm` | `rm:<chemin>` | Supprime un fichier sur la carte SD |
| `cp` | `cp:<src>:<dossierDest>` | Copie un fichier sur la carte SD |
| `mv` | `mv:<src>:<dossierDest>` | Deplace un fichier sur la carte SD |
| `ls` | `ls:<chemin>` | Liste le contenu d'un repertoire SD |
| `adrc` | `adrc:<delai_ms>:<commande>` | Planifie une commande differee |

### Commandes de diffusion reseau

| Commande | Format | Description |
|----------|--------|-------------|
| `diff` | `diff:<chemin>` | Diffuse un fichier a tout le reseau (broadcast Reed-Solomon) |
| `bupd` | `bupd` | Declenche une mise a jour firmware sur tout le reseau |
| `fwver` | `fwver` | Diffuse la version firmware courante a tout le reseau |

### Commandes du tunnel reseau E/S (netio)

| Commande | Format | Description |
|----------|--------|-------------|
| `netio` | `netio:<adresse>` | Ouvre un tunnel E/S vers une station distante |

---

## Protocole reseau

### Format de paquet LoRa

Chaque paquet LoRa contient un entete de 4 octets suivi du payload :

```
Octet 0 : Adresse destinataire (0x00 = broadcast, 0xFF = broadcast)
Octet 1 : Adresse source
Octet 2 : ID message (compteur incrementel)
Octet 3 : Longueur du payload
Octets 4+: Payload (texte ASCII, max ~225 octets)
```

### Preambule de reveil

Si le parametre `wake` est active, un paquet contenant "wake" est envoie 500 ms avant le paquet reel. Ce preambule permet de reveiller les noeuds en deep sleep via interruption radio.

### Gestion des collisions

Pour eviter les collisions radio, les commandes recues sont mises en file d'attente avec un delai aleatoire via `scheduleCommand()`. Les broadcasts utilisent un delai proportionnel a `localAddress` (300 ms * adresse) pour espacer les retransmissions.

### Deduplication

Chaque message broadcast (umap, difh) contient un identifiant unique. Le systeme maintient un tableau `dataArray` de valeurs vues recemment (duree configurable via `DELAY`). Un message deja vu est ignore.

---

## Routage Mesh (Dijkstra)

Le reseau est modelise comme un graphe non oriente pondere :

- **Sommets** : les stations (adresse unique)
- **Aretes** : les liaisons radio entre stations
- **Poids** : la moyenne du RSSI (valeur absolue) dans les deux directions

### Decouverte du reseau

1. Un noeud envoie `ping` en broadcast
2. Les voisins repondent avec `rpin:<rssi>`
3. Le noeud calcule le poids moyen (TX RSSI + RX RSSI) / 2
4. L'arete est ajoutee ou mise a jour dans le graphe
5. La nouvelle topologie locale est diffusee via `umap`

### Algorithme de routage

L'algorithme de Dijkstra est utilise pour trouver le plus court chemin entre deux noeuds :

1. Initialisation des distances a l'infini
2. Relaxation iterative de toutes les aretes
3. Reconstruction du chemin via le tableau de predecesseurs
4. Le message est envoie au **prochain saut** (next hop)

### Messages routes (dijk)

Le format d'un message route est :
```
dijk:<destination_finale>:<source_originale>:<longueur>:<id>:<payload>
```

Chaque noeud intermediaire :
1. Envoie un acquittement `rxok` au noeud precedent
2. Recalcule le prochain saut via Dijkstra
3. Transmet le message au prochain noeud

### Auto-reparation

Si un message n'est pas acquitte apres 3 tentatives (`MAX_ENTRY_AGE` = 20 s) :
1. L'arete vers le prochain saut est supprimee
2. Un ping de verification est envoie au noeud suspect
3. La topologie mise a jour est diffusee

---

## Transfert de fichiers

Pour les donnees depassant la taille maximale d'un paquet LoRa (~225 octets), un protocole de transfert de fichier est utilise.

### Protocole de transfert

```
Emetteur                          Recepteur
   |                                  |
   |--- stft (start file transfer) -->|
   |                                  |
   |<------ isrf (is ready file) -----|
   |                                  |
   |--- rfok (receiver file ok) ----->|
   |                                  |
   |--- load (ligne 1 avec ack) ----->|
   |<------ arok (ack received ok) ---|
   |--- load (ligne 2) -------------->|
   |<------ arok ---------------------|
   |--- ...                           |
   |                                  |
   |--- fend (file end) ------------->|
   |                                  |
   |           [compileFile()]        |
   |                                  |
   |<------ feok (file end ok) -------|
   |                                  |
```

### Preparation du fichier (parseFile)

1. Le fichier source est lu depuis la carte SD
2. Un CRC32 global est calcule
3. Le fichier est decoupe en paquets de 150 octets
4. Les paquets sont groupes par 8 (GROUP_K)
5. Pour chaque groupe, 8 paquets de parite Reed-Solomon sont generes
6. Le tout est ecrit dans `/tx.txt` au format texte (META + DATA + PARITY)

### Assemblage du fichier (compileFile)

1. Le fichier `/rx.txt` est lu
2. Les paquets sont verifies par CRC32
3. Les paquets manquants sont reconstruits par correction Reed-Solomon
4. Le fichier final est assemble et son CRC32 est verifie
5. Si le CRC correspond, un acquittement `feok` est envoie

### Timeouts

- **Emission** : 60 secondes (`filetxtimeout`) sans reponse du recepteur
- **Reception** : 300 secondes (`filetimeout`) sans reception de donnees

---

## Correction d'erreurs Reed-Solomon

Le systeme utilise un code Reed-Solomon sur le corps de Galois GF(256) pour proteger les transferts de fichiers.

### Parametres

- **GROUP_K = 32** : nombre de paquets de donnees par groupe
- **PARITY_M = 6** : nombre de paquets de parite par groupe
- **PACKET_SIZE = 180** : taille de chaque paquet en octets
- **CHUNK_SIZE = 90** : taille des sous-blocs transmis (2 chunks par paquet)

### Capacite de correction

Chaque groupe de 32 paquets peut tolerer la perte de **jusqu'a 6 paquets** tant qu'au moins 32 paquets de donnees du groupe sont recus correctement.

### Implementation

Le corps de Galois GF(256) est implemente avec :
- Le polynome irreductible `0x1D` (x^8 + x^4 + x^3 + x^2 + 1)
- Des tables de logarithmes et exponentielles precalculees (512 entrees)
- Multiplication, exponentiation et inversion via les tables

L'encodage de parite utilise une matrice de Vandermonde :
```
parite[p][b] = XOR pour i=0..K-1 de (data[i][b] * alpha^(p+1)^i)
```

Le decodage reconstruit les paquets manquants par inversion de la sous-matrice correspondante sur GF(256).

---

## Planificateur Cron

Le systeme dispose d'un planificateur de taches compatible avec la syntaxe cron standard.

### Format

```
minute heure jour mois jourSemaine commande;
```

Les taches sont separees par des points-virgules (`;`). Exemple de crontab :

```
*/5 * * * * gmea:1;0 8 * * 1 dexp:1;0 */2 * * * pigo;
```

### Champs supportes

| Champ | Plage | Syntaxes |
|-------|-------|----------|
| Minute | 0-59 | `*`, `*/N`, valeur exacte |
| Heure | 0-23 | `*`, `*/N`, valeur exacte |
| Jour du mois | 1-31 | `*`, `*/N`, valeur exacte |
| Mois | 1-12 | `*`, `*/N`, valeur exacte |
| Jour de la semaine | 0-6 | `*`, `*/N`, valeur exacte |

### Valeur speciale `a`

La valeur `a` est remplacee par `localAddress`. Cela permet de decaler l'execution des taches entre les noeuds pour eviter les collisions radio.

Exemple : `a * * * * pigo;` execute `pigo` quand la minute correspond a l'adresse locale.

### Fonctionnement

- Les taches cron sont evaluees une fois par minute (quand `tm_sec == 0`)
- La variable `lastMinuteChecked` empeche la double execution
- La fonction `nextWakeup()` calcule la prochaine echeance pour programmer le deep sleep

---

## Gestion d'energie

### Conditions d'entree en deep sleep

Le noeud entre en deep sleep uniquement si **toutes** les conditions suivantes sont remplies :

1. `maintmode == false` (mode normal, pas maintenance)
2. `pingphase == 0` (pas de decouverte en cours)
3. `filesender == -1` (pas de reception fichier en cours)
4. `filereceivientstation == -1` (pas d'emission fichier en cours)
5. `startstat == 0, 7 ou 8` (procedure de demarrage terminee ou inactive)
6. `entryCount == 0` (pas de messages en attente d'acquittement)
7. `pingCount == 0` (pas de pings en attente)
8. `togateCount == 0` (pas de commandes passerelle en attente)
9. Delai d'inactivite depasse (`actiontimerdel` secondes)

### Deroulement

1. L'etat complet est sauvegarde sur la carte SD (`writetosd()`)
2. La duree de sommeil est calculee : `nextWakeup() - 5` secondes
3. Le timer de reveil est programme
4. Le GPIO 1 est configure comme source de reveil externe
5. `esp_deep_sleep_start()` est appele

### Sources de reveil

- **Timer** : reveil a la prochaine echeance cron (moins 5 secondes de marge)
- **GPIO 1** : interruption materielle externe (HIGH level)

### Au reveil

L'ESP32-C3 execute `setup()` integralement. Si `rtc.getLocalEpoch() > 10` (l'horloge est initialisee), la configuration complete est rechargee depuis la SD (`readsd(1)`), incluant la topologie reseau et l'etat de demarrage.

---

## Capteurs

Mycromesh supporte plusieurs capteurs configures independamment via `/sensor.cfg`. Chaque capteur est active ou desactive individuellement.

### BME680

Capteur Bosch 4-en-1 (temperature, humidite, pression, resistance gaz) sur bus I2C.

| Parametre | Valeur |
|-----------|--------|
| Oversampling temperature | x8 |
| Oversampling humidite | x2 |
| Oversampling pression | x4 |
| Filtre IIR | taille 3 |
| Chauffage gaz | 320 C pendant 150 ms |

### AHT20

Capteur temperature/humidite sur bus I2C. Actif si `sensor_aht20 == true`.

### BMP280

Capteur pression/temperature sur bus I2C.

| Parametre | Valeur |
|-----------|--------|
| Oversampling temperature | x2 |
| Oversampling pression | x16 |
| Filtre IIR | x16 |
| Standby | 500 ms |

### DS18B20

Capteur temperature numerique sur bus OneWire (Pin 3). Peut etre chaine avec plusieurs sondes.

### Mesure et stockage

La commande `gmea:<version>` effectue une mesure et l'ecrit dans `/<version>.datadump` :

**Version 1 (capteur reel) :**
```
ntemp:<temperature>;nhum:<humidite>;npres:<pression>;nres:<resistance_gaz>;ntime:<epoch>;
```

**Version 2 (test) :**
```
test:ok;tsp:okb;
```

### Export des donnees

La commande `dexp:<version>` :
1. Lit le fichier datadump correspondant
2. Lit la structure de donnees attendue depuis `/data.ver`
3. Calcule les moyennes (prefixe `n`) ou extrait la derniere valeur (prefixe `t`)
4. Envoie le resultat a la passerelle via le reseau mesh
5. Vide le fichier datadump

---

## Systeme de passerelle

Chaque reseau mesh a une station passerelle (`stationgateway`) qui sert de point de collecte.

### File en memoire (togateQueue)

- Capacite : 10 commandes (`MAX_TOGATE_COMMANDS`)
- Chaque commande a un ID unique et un timestamp
- Si une commande n'est pas acquittee dans `TOGATE_COMMAND_TIMEOUT` (60 s) :
  - Elle est sauvegardee dans `/togate.cache` sur la SD
  - Le compteur d'echecs est incremente
  - Apres 5 echecs, la passerelle est marquee hors ligne

### File fichier (togateQueueFile)

- Capacite : 20 commandes (`MAX_TOGATE_COMMANDS_FILE`)
- Utilisee pour les transferts de fichier ligne par ligne
- Les commandes expirees sont remises dans `/tx.txt` pour reemission

### Reprise apres panne

Si le flag `incache` est actif dans les preferences et que la passerelle est en ligne :
1. Le fichier `/togate.cache` est lu ligne par ligne
2. Chaque commande est resoumise via `interpreter()`
3. Un offset persistant (NVS) permet de reprendre apres un redemarrage
4. Une fois toutes les lignes traitees, le fichier cache est supprime

---

## Procedure de demarrage

Au premier demarrage (ou apres un `star`), le noeud execute une procedure en 8 etapes pour rejoindre le reseau :

| Etat | Description | Actions |
|------|-------------|---------|
| 0 | Idle | Aucune action |
| 1 | Decouverte | Lance `pigo` (3 phases de ping broadcast) |
| 2 | Attente reponses | Collecte les `rpin` pendant 30 secondes |
| 3 | Recherche noeud | Trouve le voisin le plus proche pour demander la carte |
| 4 | Demande carte | Envoie `gmap` (max 3 tentatives, timeout `starttimeout`) |
| 5 | Attente carte | Recoit les paquets `tmap` pendant 40 secondes |
| 6 | Recherche temps | Trouve le voisin le plus proche pour la synchro horloge |
| 7 | Demande heure | Envoie `geth` (max 3 tentatives, timeout `starttimeout`) |
| 8 | Operationnel | Noeud pret et synchronise |
| 9 | Echec | Aucun voisin trouve, procedure echouee |

Si une tentative echoue (pas de reponse apres 3 essais), l'arete vers le noeud contacte est supprimee et le noeud suivant le plus proche est essaye.

---

## Mise a jour firmware (OTA)

### Procedure

1. Placer le fichier binaire du firmware dans `/update/firmware.bin` sur la carte SD
2. Envoyer la commande `upgrade` via le port serie ou le reseau
3. Le firmware est ecrit dans la partition OTA par blocs de 4096 octets
4. En cas de succes, l'ESP32-C3 redemarrage automatiquement sur le nouveau firmware

Le fichier firmware peut etre transfere vers un noeud distant via le protocole de transfert de fichier mesh.

---

## Diffusion reseau (broadcast)

La commande `diff:<chemin>` permet de diffuser un fichier a l'ensemble du reseau sans connexion point-a-point. Le fichier est prepare avec correction d'erreurs Reed-Solomon et transmis ligne par ligne en broadcast.

### Protocole de diffusion

1. L'emetteur parse le fichier (`parseFile`) et cree `/tx.txt`
2. Un paquet `brdl` (broadcast launch) est envoye pour signaler le debut de la diffusion
3. Les lignes sont emises sequentiellement via `brdf:<seq>:<data>` en broadcast
4. Chaque noeud recepteur retransmet les lignes avec un delai proportionnel a son adresse (20 ms * adresse) pour eviter les collisions
5. Un paquet `brde` (broadcast end) signale la fin ; chaque noeud assemble le fichier avec `compileFile`

### Diffusion de mise a jour firmware

La commande `bupd` declenche une mise a jour firmware coordonnee sur tout le reseau :
1. Un paquet `bupd` est diffuse en broadcast
2. Chaque noeud recevant ce paquet programme une mise a jour dans 60 secondes
3. L'emetteur se met lui-meme a jour dans 30 secondes

Apres une mise a jour reussie, le noeud diffuse automatiquement sa nouvelle version via `fwver`.

---

## Tunnel reseau E/S (netio)

Le mode netio permet d'envoyer des commandes a un noeud distant comme si l'on etait connecte directement a son port serie. Le noeud local devient **maitre**, le noeud distant devient **esclave**.

### Ouverture du tunnel

```
netio:<adresse>
```

### Fonctionnement

1. Le maitre envoie `ntiopen` via `trsp` a l'esclave
2. L'esclave confirme avec `ntiok`
3. Toutes les commandes saisies par le maitre sont envoyees via `ntidata`
4. Les sorties de l'esclave sont transmises au maitre via `ntirsp`
5. Taper `exit` ou atteindre le timeout (`NETIO_TIMEOUT`) ferme le tunnel

### Niveaux de log en mode esclave

En mode esclave, le niveau de log est contraint entre `normal` et `verbose` pour garantir un retour utile sans saturer le tunnel.

---

## Mode Bluetooth (BLE)

Mycromesh expose un service Nordic UART (NUS) via Bluetooth Low Energy. Cela permet de controler le noeud depuis un smartphone ou un ordinateur sans connexion filaire.

### Activation

```
iomd:bt
```

Le noeud est annonce comme `Mycromesh-<adresse>`. Pour revenir en USB :

```
iomd:usb
```

### Gestion de la frequence CPU

Le mode BLE impose une frequence CPU minimale de 80 MHz (contrainte du stack BLE ESP32-C3). En mode USB inactif, la frequence descend a 20 MHz pour economiser l'energie. En mode actif (transmission LoRa, calcul), la frequence monte a 160 MHz.

| Mode | Frequence |
|------|-----------|
| Inactif (USB) | 20 MHz |
| BLE actif | 80 MHz |
| Turbo (TX/calcul) | 160 MHz |

---

## Fichiers carte SD

| Fichier | Format | Description |
|---------|--------|-------------|
| `/p.cfg` | `MAX_PING_AGE:starttimeout:localAddress:DELAY:MAX_ENTRY_AGE:actiontimerdel:maintmode:stationgateway:TOGATE_COMMAND_TIMEOUT:...:ioMode:NETIO_TIMEOUT:` | Parametres de la station |
| `/sensor.cfg` | `bme680:<0\|1>:aht20:<0\|1>:bmp280:<0\|1>:ds18b20:<0\|1>` | Configuration des capteurs actifs |
| `/map.cfg` | `tmap:<v1,v2,w>:<v1,v2,w>:...` | Topologie reseau sauvegardee |
| `/e.cfg` | `startstat:msgCount:` | Etat d'environnement (pour reprise apres deep sleep) |
| `/crontab.cfg` | Taches cron separees par `;` | Planification des taches |
| `/data.ver` | `<ver> <struct>;...` | Definitions des structures de donnees par version |
| `/togate.cache` | Une commande par ligne | Commandes passerelle en echec (attente de reenvoi) |
| `/tx.txt` | Format META/DATA/PARITY | Fichier prepare pour emission avec correction d'erreurs |
| `/rx.txt` | Format META/DATA/PARITY | Tampon d'assemblage des donnees recues |
| `/large.cmd` | Texte brut | Staging pour messages volumineux (> 225 octets) |
| `/<ver>.datadump` | Mesures separees par `;` | Accumulation des donnees capteur |
| `/update/firmware.bin` | Binaire ESP32 | Fichier firmware pour mise a jour OTA |

---

## Parametres configurables

Les parametres peuvent etre modifies en cours d'execution avec la commande `parm:<nom>:<valeur>` :

| Nom | Type | Description |
|-----|------|-------------|
| `localAddress` | int | Adresse unique de la station dans le reseau |
| `stationgateway` | int | Adresse de la station passerelle |
| `MAX_PING_AGE` | long (ms) | Duree avant reessai/suppression d'un ping |
| `MAX_ENTRY_AGE` | long (ms) | Duree avant reessai/suppression d'un message en attente |
| `starttimeout` | long (ms) | Timeout pour chaque etape de la procedure de demarrage |
| `DELAY` | long (ms) | Duree de retention des IDs de deduplication |
| `actiontimerdel` | int (s) | Delai d'inactivite avant entree en deep sleep |
| `TOGATE_COMMAND_TIMEOUT` | long (ms) | Timeout des commandes passerelle |
| `NETIO_TIMEOUT` | long (ms) | Timeout d'inactivite du tunnel reseau E/S |

---

## Limites du systeme

| Ressource | Limite | Constante |
|-----------|--------|-----------|
| Stations dans le reseau | 60 | `MAX_VERTICES` |
| Connexions dans le graphe | 300 | `MAX_EDGES` |
| Messages en vol | 10 | `MAX_ENTRIES` |
| Pings simultanees | 10 | `MAX_PINGS` |
| Commandes differees | 10 | `MAX_COMMANDS` |
| File passerelle (memoire) | 10 | `MAX_TOGATE_COMMANDS` |
| File passerelle (fichier) | 20 | `MAX_TOGATE_COMMANDS_FILE` |
| Tampon IDs deduplication | 10 | `MAX_SIZE` |
| Taille paquet RS | 180 octets | `PACKET_SIZE` |
| Taille sous-bloc RS | 90 octets | `CHUNK_SIZE` |
| Paquets donnees par groupe RS | 32 | `GROUP_K` |
| Paquets parite par groupe RS | 6 | `PARITY_M` |
| Buffer lecture fichier | 256 octets | `TAILLE_TAMPON` |
| Taille max payload LoRa | ~225 octets | (limite protocole) |
| Timeout reception fichier | 300 s | `filetimeout` |
| Timeout emission fichier | 60 s | `filetxtimeout` |

---

## Reference des fonctions

### Initialisation et boucle principale

| Fonction | Description |
|----------|-------------|
| `setup()` | Initialise le serial, LoRa, preferences, charge la config SD |
| `loop()` | Boucle principale : serial, LoRa RX, timers, cron, sleep |

### Communication LoRa

| Fonction | Parametres | Description |
|----------|------------|-------------|
| `sendMessage()` | `bool wake, String msg, int dest` | Envoie un paquet LoRa (avec preambule optionnel) |
| `onReceive()` | `int packetSize` | Traite un paquet LoRa recu |
| `scheduleCommand()` | `unsigned long delay, String cmd` | Ajoute une commande a la file differee |
| `checkDelayedCommands()` | - | Execute les commandes differees arrivees a echeance |

### Reseau mesh

| Fonction | Parametres | Description |
|----------|------------|-------------|
| `dijkstra()` | `int src, int dest, String msg` | Route un message via le plus court chemin |
| `nextstep()` | `int src, int dest` | Retourne le prochain saut vers dest |
| `addOrUpdateEdge()` | `int v1, int v2, int weight` | Ajoute ou met a jour une arete (moyenne des poids) |
| `removeEdge()` | `int v1, int v2` | Supprime une arete specifique |
| `removeEdgesByVertex()` | `int v` | Supprime toutes les aretes d'un sommet |
| `findNearestVertex()` | `int src` | Trouve le voisin direct le plus proche |
| `addVertex()` | `int v` | Ajoute un sommet au graphe |
| `updateVertices()` | - | Reconstruit la liste des sommets depuis les aretes |
| `exportEdgesAstmapCommand()` | - | Exporte la topologie au format tmap |
| `exportEdgesContainingVertex()` | `int vertex` | Exporte les aretes d'un sommet au format umap |
| `printEdges()` | - | Affiche les aretes sur le port serie |
| `clearEdges()` | - | Supprime toutes les aretes |

### Gestion des entries (messages en attente)

| Fonction | Parametres | Description |
|----------|------------|-------------|
| `addEntry()` | `String id, int retry, String msg` | Ajoute un message en attente d'acquittement |
| `removeEntryByID()` | `String id` | Supprime un message acquitte |
| `checkAndRemoveOldEntries()` | - | Reessaie ou supprime les messages expires |

### Gestion des pings

| Fonction | Parametres | Description |
|----------|------------|-------------|
| `addPingEntry()` | `int target, int phase` | Ajoute un ping en cours de suivi |
| `removePingEntryByNbtoping()` | `int target` | Supprime un ping acquitte |
| `checkAndRemoveOldPingEntries()` | - | Reessaie ou supprime les pings expires |

### Transfert de fichiers

| Fonction | Parametres | Description |
|----------|------------|-------------|
| `parseFile()` | `String path` | Prepare un fichier pour emission (decoupage + RS) |
| `compileFile()` | `String name, int origin, int remof` | Assemble un fichier recu (verification + RS) |
| `exportfile()` | - | Exporte la prochaine ligne de /tx.txt |
| `importfile()` | `String file, String input` | Ecrit une ligne dans un fichier (append) |
| `large()` | `String data, int dest` | Gere les messages > 225 octets via transfert fichier |

### Reed-Solomon GF(256)

| Fonction | Parametres | Description |
|----------|------------|-------------|
| `initGaloisField()` | - | Initialise les tables exp/log de GF(256) |
| `gfMul()` | `uint8_t a, uint8_t b` | Multiplication dans GF(256) |
| `gfPow()` | `uint8_t x, int power` | Exponentiation dans GF(256) |
| `gfInv()` | `uint8_t a` | Inversion dans GF(256) |
| `encodeGroupParity()` | `uint8_t **data, uint8_t **parity` | Genere les paquets de parite |
| `invertMatrixGF()` | `uint8_t *mat, uint8_t *inv, int n` | Inversion de matrice sur GF(256) |

### Integrite

| Fonction | Parametres | Description |
|----------|------------|-------------|
| `crc32_buf()` | `uint8_t *buf, size_t len` | CRC32 d'un buffer memoire |
| `crc32_file()` | `File &f` | CRC32 d'un fichier SD |
| `bytesToHex()` | `uint8_t *data, int len` | Convertit des octets en chaine hexadecimale |
| `hexToBytes()` | `String hex, uint8_t *out, int len` | Convertit une chaine hex en octets |

### Stockage SD

| Fonction | Parametres | Description |
|----------|------------|-------------|
| `readsd()` | `bool recover` | Charge la configuration depuis la SD |
| `writetosd()` | - | Sauvegarde la configuration sur la SD |
| `remfromsd()` | `String path` | Supprime un fichier de la SD |
| `mkdirsd()` | `String path` | Cree un repertoire sur la SD |
| `exportcache()` | - | Reexporte les commandes du cache passerelle |

### Capteur et donnees

| Fonction | Parametres | Description |
|----------|------------|-------------|
| `startsensor()` | - | Initialise le capteur BME680 |
| `measuretodump()` | `int version` | Effectue une mesure et l'enregistre |
| `exportdata()` | `String version` | Compile et exporte les donnees du datadump |

### Temps et cron

| Fonction | Parametres | Description |
|----------|------------|-------------|
| `executeCronTasks()` | - | Evalue et execute les taches cron |
| `checkCronField()` | `String field, int value, int mod` | Verifie si un champ cron correspond |
| `nextWakeup()` | - | Calcule le prochain reveil cron (en secondes) |

### Passerelle

| Fonction | Parametres | Description |
|----------|------------|-------------|
| `togateAddCommand()` | `int id, String cmd` | Ajoute a la file passerelle memoire |
| `togateRemoveById()` | `int id` | Retire une commande acquittee |
| `togatePurgeOld()` | - | Purge les commandes expirees vers le cache SD |
| `togateAddCommandFile()` | `int id, String cmd` | Ajoute a la file passerelle fichier |
| `togateRemoveByIdFile()` | `int id` | Retire une commande fichier acquittee |
| `purgeToOldFile()` | - | Remet les commandes expirees dans /tx.txt |

### Utilitaires

| Fonction | Parametres | Description |
|----------|------------|-------------|
| `getValue()` | `String data, char sep, int index` | Extrait le N-ieme champ d'une chaine delimitee |
| `generateid()` | - | Genere un identifiant unique (msgCount + address + random) |
| `formatNumber()` | `int number, int digits` | Formate un nombre avec zeros initiaux |
| `interpreter()` | `String msg` | Interpreteur principal de commandes |
| `changepval()` | `String name, String value` | Modifie un parametre d'execution |
| `startprocedure()` | - | Machine a etats de la procedure de demarrage |
| `doFirmwareUpdate()` | - | Applique la mise a jour firmware depuis la SD |
